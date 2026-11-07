#include "stadia_ble.h"

#include <cstring>
#include <cstdio>
#include "btstack.h"
#include "controller_state.h"
#include "persona/host_persona.h"
#include "pico/time.h"

// 声明 main.cpp 提供的输入上报入口
extern void host_input_submit_controller_state(const BridgeControllerState &controller_state);
extern void host_input_prepare_persona_switch(void);

namespace {

enum class StadiaState {
    Idle,
    Scanning,
    Connecting,
    Connected,
    DiscoveringServices,
    DiscoveringCharacteristics,
    Subscribing,
    Ready
};

StadiaState current_state = StadiaState::Idle;
hci_con_handle_t stadia_con_handle = HCI_CON_HANDLE_INVALID;
bd_addr_t stadia_addr{};
bd_addr_type_t stadia_addr_type = BD_ADDR_TYPE_LE_PUBLIC;

gatt_client_service_t hid_service{};
gatt_client_characteristic_t input_char{};
gatt_client_characteristic_t output_char{};
bool has_input_char = false;
bool has_output_char = false;

gatt_client_notification_t notification_listener{};
btstack_packet_callback_registration_t hci_event_callback_registration{};
btstack_packet_callback_registration_t sm_event_callback_registration{};

// 震动限频控制（防止高频震动指令拥塞蓝牙信道）
uint32_t last_rumble_send_time_ms = 0;
uint8_t last_strong = 0;
uint8_t last_weak = 0;

void parse_stadia_input_report(const uint8_t *data, uint16_t len) {
    if (len < 10) return;

    // 若首字节为 Report ID 0x03，偏移 1 字节
    const uint8_t *report = data;
    if (data[0] == 0x03) {
        report = data + 1;
        len -= 1;
    }
    if (len < 9) return;

    BridgeControllerState s{};

    // 按键掩码 (16位)
    uint16_t buttons = report[0] | (static_cast<uint16_t>(report[1]) << 8);

    s.cross    = (buttons & (1 << 0)) != 0; // A
    s.circle   = (buttons & (1 << 1)) != 0; // B
    s.square   = (buttons & (1 << 2)) != 0; // X
    s.triangle = (buttons & (1 << 3)) != 0; // Y
    s.l1       = (buttons & (1 << 4)) != 0; // LB
    s.r1       = (buttons & (1 << 5)) != 0; // RB
    s.l3       = (buttons & (1 << 6)) != 0; // L3
    s.r3       = (buttons & (1 << 7)) != 0; // R3
    s.options  = (buttons & (1 << 8)) != 0; // Start / Menu
    s.create   = (buttons & (1 << 9)) != 0; // Back / Options
    s.home     = (buttons & (1 << 10)) != 0; // Stadia / Guide
    
    // Assistant 键或 Capture 键
    bool assistant = (buttons & (1 << 11)) != 0;
    bool capture   = (buttons & (1 << 12)) != 0;
    if (assistant || capture) {
        s.home = true; // 快捷唤起 Guide
    }

    // D-Pad Hat Switch (report[2])
    uint8_t hat = report[2] & 0x0F;
    switch (hat) {
        case 0: s.dpad_up = true; break;
        case 1: s.dpad_up = true; s.dpad_right = true; break;
        case 2: s.dpad_right = true; break;
        case 3: s.dpad_down = true; s.dpad_right = true; break;
        case 4: s.dpad_down = true; break;
        case 5: s.dpad_down = true; s.dpad_left = true; break;
        case 6: s.dpad_left = true; break;
        case 7: s.dpad_up = true; s.dpad_left = true; break;
        default: break; // 8 居中释放
    }

    // 摇杆 X/Y (0~255, 中心 128)
    s.left_stick_x  = report[3];
    s.left_stick_y  = report[4];
    s.right_stick_x = report[5];
    s.right_stick_y = report[6];

    // 线性扳机 L2 / R2 (0~255)
    s.left_trigger  = report[7];
    s.right_trigger = report[8];
    s.l2_pressed    = s.left_trigger > 30;
    s.r2_pressed    = s.right_trigger > 30;

    // 提交到全局输入管道
    host_input_submit_controller_state(s);
}

void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_event_service_query_result_get_service(packet, &hid_service);
            break;
        }
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t ch;
            gatt_event_characteristic_query_result_get_characteristic(packet, &ch);
            if (ch.uuid16 == 0x2A4D) { // Report Characteristic
                if ((ch.properties & ATT_PROPERTY_NOTIFY) != 0) {
                    input_char = ch;
                    has_input_char = true;
                }
                if ((ch.properties & (ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE)) != 0) {
                    output_char = ch;
                    has_output_char = true;
                }
            }
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t status = gatt_event_query_complete_get_status(packet);
            if (current_state == StadiaState::DiscoveringServices) {
                if (status == ERROR_CODE_SUCCESS && hid_service.start_group_handle != 0) {
                    current_state = StadiaState::DiscoveringCharacteristics;
                    gatt_client_discover_characteristics_for_service(handle_gatt_client_event, stadia_con_handle, &hid_service);
                }
            } else if (current_state == StadiaState::DiscoveringCharacteristics) {
                if (status == ERROR_CODE_SUCCESS && has_input_char) {
                    current_state = StadiaState::Subscribing;
                    // 注册数据监听
                    gatt_client_listen_for_characteristic_value_updates(
                        &notification_listener,
                        handle_gatt_client_event,
                        stadia_con_handle
                    );
                    // 订阅 Notification
                    gatt_client_write_client_characteristic_configuration(
                        handle_gatt_client_event,
                        stadia_con_handle,
                        &input_char,
                        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
                    );
                }
            } else if (current_state == StadiaState::Subscribing) {
                current_state = StadiaState::Ready;
                // 成功就绪！自动激活 Xbox 360 Persona
                if (host_persona_active() != HostPersonaModeXusb360) {
                    host_persona_set_active(HostPersonaModeXusb360);
                    host_input_prepare_persona_switch();
                }
            }
            break;
        }
        case GATT_EVENT_NOTIFICATION: {
            uint16_t value_len = gatt_event_notification_get_value_length(packet);
            const uint8_t *val = gatt_event_notification_get_value(packet);
            parse_stadia_input_report(val, value_len);
            break;
        }
        default:
            break;
    }
}

void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case SM_EVENT_JUST_WORKS_REQUEST: {
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        }
        case SM_EVENT_PAIRING_COMPLETE: {
            break;
        }
        default:
            break;
    }
}

void le_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t event = hci_event_packet_get_type(packet);

    switch (event) {
        case GAP_EVENT_ADVERTISING_REPORT: {
            if (current_state != StadiaState::Scanning) return;

            uint8_t event_type = gap_event_advertising_report_get_advertising_event_type(packet);
            if (event_type != 0 && event_type != 1 && event_type != 4) return;

            uint8_t data_length = gap_event_advertising_report_get_data_length(packet);
            const uint8_t *adv_data = gap_event_advertising_report_get_data(packet);

            // 检查广播数据中是否包含 "Stadia"
            bool found_stadia = false;
            if (data_length >= 6) {
                for (int i = 0; i <= data_length - 6; ++i) {
                    if (memcmp(adv_data + i, "Stadia", 6) == 0) {
                        found_stadia = true;
                        break;
                    }
                }
            }

            if (found_stadia) {
                gap_stop_scan();
                gap_event_advertising_report_get_address(packet, stadia_addr);
                stadia_addr_type = static_cast<bd_addr_type_t>(gap_event_advertising_report_get_address_type(packet));

                current_state = StadiaState::Connecting;
                gap_connect(stadia_addr, stadia_addr_type);
            }
            break;
        }

        case HCI_EVENT_LE_META: {
            uint8_t subevent = hci_event_le_meta_get_subevent_code(packet);
            if (subevent == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                if (status == ERROR_CODE_SUCCESS) {
                    stadia_con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    current_state = StadiaState::DiscoveringServices;

                    // 协商极速连接间隔 (7.5ms ~ 10ms)，降低输入延迟
                    gap_request_connection_parameter_update(stadia_con_handle, 6, 8, 0, 400);

                    // 启动 HID 服务查找
                    has_input_char = false;
                    has_output_char = false;
                    gatt_client_discover_primary_services_by_uuid16(
                        handle_gatt_client_event,
                        stadia_con_handle,
                        0x1812 // HID Service
                    );
                } else {
                    current_state = StadiaState::Idle;
                    stadia_ble_start_scan();
                }
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            if (handle == stadia_con_handle) {
                stadia_con_handle = HCI_CON_HANDLE_INVALID;
                current_state = StadiaState::Idle;
                has_input_char = false;
                has_output_char = false;

                // 清空输入并恢复双模扫描
                BridgeControllerState neutral{};
                host_input_submit_controller_state(neutral);

                // 重新开启扫描
                stadia_ble_start_scan();
            }
            break;
        }

        default:
            break;
    }
}

} // namespace

extern "C" {

void stadia_ble_init(void) {
    gatt_client_init();

    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);

    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    hci_event_callback_registration.callback = &le_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
}

void stadia_ble_start_scan(void) {
    if (current_state == StadiaState::Ready || current_state == StadiaState::Connecting) {
        return;
    }
    current_state = StadiaState::Scanning;
    gap_set_scan_parameters(1, 0x0030, 0x0030); // 活跃扫描
    gap_start_scan();
}

void stadia_ble_stop_scan(void) {
    if (current_state == StadiaState::Scanning) {
        gap_stop_scan();
        current_state = StadiaState::Idle;
    }
}

bool stadia_ble_is_connected(void) {
    return current_state == StadiaState::Ready && stadia_con_handle != HCI_CON_HANDLE_INVALID;
}

void stadia_ble_send_rumble(uint8_t strong, uint8_t weak) {
    if (!stadia_ble_is_connected() || !has_output_char) {
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    // 20ms 限频且只在数值发生变化或心跳刷新时发送
    if (now - last_rumble_send_time_ms < 20 && strong == last_strong && weak == last_weak) {
        return;
    }

    last_rumble_send_time_ms = now;
    last_strong = strong;
    last_weak = weak;

    // 构造 Stadia BLE 5字节震动报文: [0x05, low_L, low_H, high_L, high_H]
    // 增加轻微死区增强 (Deadzone Boost)，让微弱震动也能被清晰感知
    uint16_t strong_16 = 0;
    if (strong > 0) {
        strong_16 = static_cast<uint16_t>(strong) << 8 | strong;
    }

    uint16_t weak_16 = 0;
    if (weak > 0) {
        weak_16 = static_cast<uint16_t>(weak) << 8 | weak;
    }

    uint8_t payload[5];
    payload[0] = 0x05; // Report ID
    payload[1] = static_cast<uint8_t>(strong_16 & 0xFF);
    payload[2] = static_cast<uint8_t>((strong_16 >> 8) & 0xFF);
    payload[3] = static_cast<uint8_t>(weak_16 & 0xFF);
    payload[4] = static_cast<uint8_t>((weak_16 >> 8) & 0xFF);

    gatt_client_write_value_of_characteristic_without_response(
        stadia_con_handle,
        output_char.value_handle,
        5,
        payload
    );
}

} // extern "C"
