/**
 * @file protocol_bt.h
 * @brief 双车蓝牙位姿与速度二进制协议。
 *
 * 业务载荷只包含位置、航向和速度；所有多字节整数均按小端序编码，
 * 不直接发送编译器可能插入填充字节的 C 结构体。
 */
#ifndef PROTOCOL_BT_H
#define PROTOCOL_BT_H

#include <stdint.h>

#define BT_PROTOCOL_SOF_0             0xA5U
#define BT_PROTOCOL_SOF_1             0x5AU
#define BT_PROTOCOL_VERSION           0x01U
#define BT_PROTOCOL_TYPE_POSE_VELOCITY 0x01U

#define BT_PROTOCOL_POSE_PAYLOAD_SIZE 14U
#define BT_PROTOCOL_FRAME_SIZE        23U
#define BT_PROTOCOL_MAX_FRAME_SIZE    64U

/** 位置单位 mm，航向单位 0.01 度，速度单位分别为 mm/s 和 0.01 度/s。 */
typedef struct {
    int32_t x_mm;
    int32_t y_mm;
    int16_t yaw_cdeg;
    int16_t linear_mm_s;
    int16_t angular_cdeg_s;
} BtPoseVelocity;

extern volatile BtPoseVelocity g_bt_latest_pose_velocity;
extern volatile uint16_t g_bt_latest_rx_sequence;
extern volatile uint16_t g_bt_tx_sequence;
extern volatile uint32_t g_bt_rx_valid_frame_count;
extern volatile uint32_t g_bt_rx_invalid_frame_count;
extern volatile uint32_t g_bt_rx_crc_error_count;
extern volatile uint32_t g_bt_rx_byte_count;
extern volatile uint32_t g_bt_last_rx_ms;

void Protocol_Bt_Init(void);
void Protocol_Bt_Task(void);

/** 编码一帧固定 23 字节的数据，成功返回 BT_PROTOCOL_FRAME_SIZE。 */
uint16_t Protocol_Bt_EncodePoseVelocity(
    const BtPoseVelocity *data,
    uint16_t sequence,
    uint8_t *frame,
    uint16_t capacity
);

/** 校验并解码一帧；成功返回 1，否则返回 0。 */
uint8_t Protocol_Bt_DecodePoseVelocity(
    const uint8_t *frame,
    uint16_t length,
    BtPoseVelocity *data,
    uint16_t *sequence
);

/** 通过 UART_HC05 发送一帧；未配置 UART_HC05 或参数非法时返回 0。 */
uint8_t Protocol_Bt_SendPoseVelocity(const BtPoseVelocity *data);

/** 取出最近一次新数据；同一帧只返回一次。 */
uint8_t Protocol_Bt_TakeLatestPoseVelocity(
    BtPoseVelocity *data,
    uint16_t *sequence
);

/** 尚未收到合法帧时返回 UINT32_MAX。 */
uint32_t Protocol_Bt_GetRxAgeMs(void);
uint8_t Protocol_Bt_IsRxFresh(uint32_t max_age_ms);

/** CRC-16/CCITT-FALSE：多项式 0x1021，初值 0xFFFF。 */
uint16_t Protocol_Bt_Crc16(const uint8_t *data, uint16_t length);

#endif
