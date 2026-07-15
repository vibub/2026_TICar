#include "bsp_time.h"

#include "ti/driverlib/m0p/dl_systick.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_bsp_time_ms;

void Bsp_Time_Init(void)
{
    g_bsp_time_ms = 0U;
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
}

uint32_t Bsp_Time_GetMilliseconds(void)
{
    return g_bsp_time_ms;
}

void SysTick_Handler(void)
{
    g_bsp_time_ms++;
}
