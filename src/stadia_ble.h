#ifndef DS5_BRIDGE_STADIA_BLE_H
#define DS5_BRIDGE_STADIA_BLE_H

#include <cstdint>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 Stadia BLE 客户端协议栈
void stadia_ble_init(void);

// 开启 Stadia 手柄 BLE 广播扫描与自动回连
void stadia_ble_start_scan(void);

// 停止扫描
void stadia_ble_stop_scan(void);

// 查询当前是否有 Stadia 手柄在线
bool stadia_ble_is_connected(void);

// 向 Stadia 手柄下发马达震动数据 (strong: 0~255, weak: 0~255)
void stadia_ble_send_rumble(uint8_t strong, uint8_t weak);

#ifdef __cplusplus
}
#endif

#endif // DS5_BRIDGE_STADIA_BLE_H
