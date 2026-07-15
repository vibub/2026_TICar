#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/**
 * 初始化 1 ms SysTick 系统时基。
 */
void Bsp_Time_Init(void);

/**
 * 获取系统启动后的毫秒计数。
 */
uint32_t Bsp_Time_GetMilliseconds(void);

#endif
