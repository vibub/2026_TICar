#ifndef BSP_PTZ_H
#define BSP_PTZ_H

#include <stdint.h>

#define BSP_PTZ_PWM_PERIOD 10000U

#define BSP_PTZ_TILT_CENTER 750U
#define BSP_PTZ_TILT_MIN 400U
#define BSP_PTZ_TILT_MAX 1075U

#define BSP_PTZ_PAN_CENTER 750U
#define BSP_PTZ_PAN_MIN 250U
#define BSP_PTZ_PAN_MAX 1250U

/**
 * 初始化双轴云台并写入中心位置，但不启动 PWM。
 */
void Bsp_Ptz_Init(void);

/**
 * 启动双轴云台的 50 Hz PWM 输出。
 */
void Bsp_Ptz_Start(void);

/**
 * 停止云台 PWM，并将两个舵机信号引脚拉低。
 */
void Bsp_Ptz_Disable(void);

/**
 * 将 Tilt 和 Pan 两轴设置到中心位置。
 */
void Bsp_Ptz_SetCenter(void);

/**
 * 设置舵机比较值，参数会自动限制到各轴安全范围。
 * Tilt 使用 PWM_PTZ C0/PB4，Pan 使用 PWM_PTZ C1/PB1。
 */
void Bsp_Ptz_SetCompare(uint16_t tilt_compare, uint16_t pan_compare);

#endif
