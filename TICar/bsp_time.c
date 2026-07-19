/**
 * @file bsp_time.c
 * @brief 使用 SysTick 提供 1 ms 单调计数，为非阻塞任务和状态机提供公共时基。
 */
#include "bsp_time.h"

#include "ti/driverlib/m0p/dl_systick.h"
#include "ti_msp_dl_config.h"

/* 由 SysTick ISR 更新；主循环只读取。32 位自然回绕由调用方的无符号时间差处理。 */
static volatile uint32_t g_bsp_time_ms;

/* 按 CPUCLK_FREQ/1000 配置 1 ms 中断周期，并从零开始计时。 */
void Bsp_Time_Init(void)
{
    g_bsp_time_ms = 0U;
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
}

uint32_t Bsp_Time_GetMilliseconds(void)
{
    return g_bsp_time_ms;
}

/* SysTick ISR 只递增计数，保持常数时间；禁止在此加入日志或阻塞操作。 */
void SysTick_Handler(void)
{
    g_bsp_time_ms++;
}
