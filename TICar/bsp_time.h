/**
 * @file bsp_time.h
 * @brief 1 ms 系统时基初始化与读取接口。
 */
#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/**
 * 初始化 1 kHz SysTick，并把毫秒计数清零。应在 SysConfig 时钟初始化后调用一次。
 */
void Bsp_Time_Init(void);

/**
 * 获取系统启动后的 32 位毫秒计数。
 *
 * 计数约每 49.7 天自然回绕；超时判断应使用 uint32_t 无符号差值，不能依赖绝对值永久递增。
 */
uint32_t Bsp_Time_GetMilliseconds(void);

#endif
