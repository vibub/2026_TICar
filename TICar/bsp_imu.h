/**
 * @file bsp_imu.h
 * @brief HI219M 所用 UART3/PB2/PB3 接收层。
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

void Bsp_Imu_Init(void);
void Bsp_Imu_Task(void);
uint8_t Bsp_Imu_IsHardwareReady(void);
void Bsp_Imu_SendLegacyStartCommands(void);

#endif
