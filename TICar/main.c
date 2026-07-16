/**
 * @file main.c
 * @brief TICar 固件入口，完成 SysConfig 初始化后持续调度非阻塞应用主循环。
 *
 * 中断由硬件独立触发；主循环必须保持快速轮询，避免阻塞串口屏命令和控制任务。
 */
#include "app_main.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    /* 先完成 SysConfig 生成的时钟、GPIO、定时器、ADC 和 UART 初始化。 */
    SYSCFG_DL_init();
    App_Init();

    /* 所有应用任务均设计为单步返回；中断在该循环之外由硬件自动调度。 */
    while (1) {
        App_Loop();
    }
}
