/**
 * @file bsp_imu.h
 * @brief HI219M 所用 UART3/PB2/PB3 接收层。
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

extern volatile uint32_t g_imu_uart_irq_count;
extern volatile uint32_t g_imu_uart_overflow_count;
extern volatile uint32_t g_imu_uart_error_count;
extern volatile uint32_t g_imu_uart_tx_count;
extern volatile uint32_t g_imu_legacy_start_count;

void Bsp_Imu_Init(void);
void Bsp_Imu_Task(void);
uint8_t Bsp_Imu_IsHardwareReady(void);
void Bsp_Imu_SendLegacyStartCommands(void);

#endif
