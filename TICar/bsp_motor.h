/**
 * @file bsp_motor.h
 * @brief 电机 H 桥、编码器和双轮速度闭环的公共接口。
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/** 左右编码器累计计数的一致快照，两个值在同一临界区内读取。 */
typedef struct {
    int32_t left_count;
    int32_t right_count;
} Bsp_Motor_EncoderSnapshot;

/* H 桥和 PWM 控制。 */
void Bsp_Motor_Init(void);    /* 初始化后保持驱动桥空闲。 */
void Bsp_Motor_Disable(void); /* 停止 PWM 并让驱动桥进入电气空闲状态。 */
void Bsp_Motor_Stop(void);    /* 主动制动：两路桥输入进入短路制动组合。 */
void Bsp_Motor_Coast(void);   /* 滑行：两路桥输入关闭，不主动吸收机械能。 */
/**
 * 按新车体坐标设置逻辑左/右轮比例；正值朝新车头前进、负值后退。
 * BSP 内部负责新旧左右轮交换、方向反转、限幅和单轮修正。
 */
void Bsp_Motor_Set(float left_ratio, float right_ratio);
/** 原始硬件联调接口：left/right 按新车体左右定义，dir_high 仍表示方向引脚原始电平。 */
void Bsp_Motor_SetLeftRaw(uint8_t dir_high, uint32_t compare);
void Bsp_Motor_SetRightRaw(uint8_t dir_high, uint32_t compare);

/* 编码器接口。left/right 按新车体坐标定义，累计计数和速度在朝新车头前进时为正。 */
void Bsp_Motor_EncoderInit(void);
void Bsp_Motor_EncoderReset(void);
void Bsp_Motor_EncoderSample(void);
/** 原子读取左右累计计数，避免路线距离计算使用不同时刻的两轮数据。 */
void Bsp_Motor_GetEncoderSnapshot(Bsp_Motor_EncoderSnapshot *snapshot);
/** 将软件编码器 tick 的绝对数量换算为轮缘行驶距离，单位为 cm。 */
float Bsp_Motor_EncoderTicksToCm(uint32_t ticks);
int32_t Bsp_Motor_GetLeftEncoderCount(void);
int32_t Bsp_Motor_GetRightEncoderCount(void);
int16_t Bsp_Motor_GetLeftEncoderSpeed(void);
int16_t Bsp_Motor_GetRightEncoderSpeed(void);

/* 双轮速度闭环。应用层应以约 20 ms 固定周期调用 Update，目标和测量速度单位为 cm/s。 */
void Bsp_Motor_SpeedPidInit(void);
void Bsp_Motor_SpeedPidReset(void);
void Bsp_Motor_SetSpeedTargets(float left_cm_s, float right_cm_s);
void Bsp_Motor_SpeedPidUpdate(void);
/** 主动制动，同时清除目标、积分、历史误差和输出状态。 */
void Bsp_Motor_SpeedPidStop(void);

/* 速度闭环调试 getter：command/P/I 为 PWM 比例贡献；fault bit0 左轮、bit1 右轮。 */
float Bsp_Motor_GetLeftSpeedTarget(void);
float Bsp_Motor_GetRightSpeedTarget(void);
float Bsp_Motor_GetLeftSpeedCommand(void);
float Bsp_Motor_GetRightSpeedCommand(void);
float Bsp_Motor_GetLeftMeasuredSpeed(void);
float Bsp_Motor_GetRightMeasuredSpeed(void);
float Bsp_Motor_GetLeftSpeedPTerm(void);
float Bsp_Motor_GetRightSpeedPTerm(void);
float Bsp_Motor_GetLeftSpeedITerm(void);
float Bsp_Motor_GetRightSpeedITerm(void);
uint32_t Bsp_Motor_GetSpeedFaults(void);

#endif
