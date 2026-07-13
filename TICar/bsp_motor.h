#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

void Bsp_Motor_Init(void);
void Bsp_Motor_Disable(void);
void Bsp_Motor_Stop(void);
void Bsp_Motor_Coast(void);
void Bsp_Motor_Set(float left_ratio, float right_ratio);
void Bsp_Motor_SetLeftRaw(uint8_t dir_high, uint32_t compare);
void Bsp_Motor_SetRightRaw(uint8_t dir_high, uint32_t compare);

#endif
