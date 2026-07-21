/**
 * @file bsp_imu.h
 * @brief LSM6DSV16X I2C bring-up interface and CCS Watch diagnostics.
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

typedef enum {
    BSP_IMU_INIT_NO_I2C = 0,
    BSP_IMU_INIT_NOT_FOUND = 1,
    BSP_IMU_INIT_CONFIG_ERROR = 2,
    BSP_IMU_INIT_OK = 3
} BspImuInitStatus;

typedef enum {
    BSP_IMU_CALIBRATION_IDLE = 0,
    BSP_IMU_CALIBRATING = 1,
    BSP_IMU_CALIBRATED = 2
} BspImuCalibrationStatus;

extern volatile uint8_t g_imu_i2c_address;
extern volatile uint8_t g_imu_who_am_i;
extern volatile uint8_t g_imu_init_status;
extern volatile uint8_t g_imu_last_status;
extern volatile uint32_t g_imu_i2c_read_count;
extern volatile uint32_t g_imu_i2c_error_count;
extern volatile uint32_t g_imu_sample_count;
extern volatile uint16_t g_imu_fifo_level;
extern volatile uint8_t g_imu_last_fifo_tag;
extern volatile uint8_t g_imu_calibration_status;
extern volatile uint16_t g_imu_calibration_sample_count;
extern volatile uint32_t g_imu_sflp_count;
extern volatile uint32_t g_imu_fifo_overrun_count;
extern volatile float g_imu_gyro_bias_dps[3];
extern volatile float g_imu_yaw_zero_deg;
extern volatile float g_imu_heading_deg;
extern volatile uint8_t g_imu_heading_ready;
extern volatile uint32_t g_imu_heading_zero_count;
extern volatile uint32_t g_imu_last_sflp_ms;

void Bsp_Imu_Init(void);
void Bsp_Imu_Task(void);
uint8_t Bsp_Imu_IsHardwareReady(void);
uint8_t Bsp_Imu_ZeroYaw(void);
uint8_t Bsp_Imu_IsHeadingReady(void);
float Bsp_Imu_GetHeadingDeg(void);
uint8_t Bsp_Imu_IsHeadingFresh(uint32_t max_age_ms);

#endif
