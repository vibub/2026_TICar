/**
 * @file protocol_imu.h
 * @brief HiPNUC HI219M 串口二进制协议解析与诊断状态。
 */
#ifndef PROTOCOL_IMU_H
#define PROTOCOL_IMU_H

#include <stdint.h>

typedef enum {
    IMU_PROTOCOL_UNKNOWN = 0,
    IMU_PROTOCOL_LEGACY_TLV = 1,
    IMU_PROTOCOL_HI91 = 2,
    IMU_PROTOCOL_LSM6DSV16X = 3
} ImuProtocol;

/**
 * 该结构可直接加入 CCS Watch。HI91 的 acc/gyro/mag 单位分别为 G、deg/s、uT；
 * 老协议没有可靠公开比例，保留 legacy_*_raw 原始计数。
 */
typedef struct {
    float acc_g[3];
    float gyro_dps[3];
    float mag_ut[3];
    float euler_deg[3];
    float quat[4];
    float pressure_pa;
    int16_t legacy_acc_raw[3];
    int16_t legacy_gyro_raw[3];
    int16_t legacy_mag_raw[3];
    uint32_t system_time_ms;
    uint32_t byte_count;
    uint32_t sync_count;
    uint32_t valid_frame_count;
    uint32_t data_frame_count;
    uint32_t sample_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t unknown_item_count;
    uint32_t unknown_frame_count;
    uint16_t last_payload_len;
    int8_t temperature_c;
    uint8_t module_id;
    uint8_t protocol;
    uint8_t frame_updated;
    uint8_t rx_state;
    uint8_t last_byte;
    uint8_t raw_tail[16];
    uint8_t raw_tail_index;
    uint8_t hardware_ready;
} ImuState;

extern volatile ImuState g_imu;

void Protocol_Imu_Init(void);
void Protocol_Imu_FeedByte(uint8_t byte);
uint8_t Protocol_Imu_ConsumeUpdated(void);

#endif
