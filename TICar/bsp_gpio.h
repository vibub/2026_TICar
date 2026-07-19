/**
 * @file bsp_gpio.h
 * @brief 板载心跳灯和方形赛道角点指示灯接口。
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

/** 初始化心跳灯，并直接配置未纳入 SysConfig 的 PB26 红色角点指示灯。 */
void Bsp_Gpio_Init(void);
/** 翻转板载心跳灯。 */
void Bsp_Gpio_ToggleHeartbeat(void);
/** 按 on 是否非零强制设置心跳灯状态。 */
void Bsp_Gpio_SetHeartbeat(uint8_t on);
/** 强制设置角点指示灯，同时重启软件闪烁相位。 */
void Bsp_Gpio_SetCornerIndicator(uint8_t on);
/** 按固定巡线周期调用；角点有效时软件分频闪烁，无效时保持熄灭。 */
void Bsp_Gpio_UpdateCornerIndicator(uint8_t corner_active);

#endif
