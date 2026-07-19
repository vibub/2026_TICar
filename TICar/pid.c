/**
 * @file pid.c
 * @brief 通用离散位置式 PID 实现，不包含输出限幅、积分限幅或抗饱和处理。
 *
 * 调用方应以固定周期调用 PID_Update()，并自行保证指针有效及输出范围安全。
 */
#include "pid.h"

/* 初始化参数，并清除目标、测量、积分、微分历史和输出。 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error_last = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_SetTarget(PID_Controller *pid, float target)
{
    pid->target = target;
}

/*
 * 执行一次位置式 PID。积分和微分均按“每次调用”计算，因此调用周期必须稳定；
 * 本函数不做抗饱和和输出限幅，执行器约束由上层负责。
 */
float PID_Update(PID_Controller *pid, float current)
{
    float error;
    float derivative;

    pid->current = current;
    error = pid->target - pid->current;
    pid->integral += error;
    derivative = error - pid->error_last;
    pid->error_last = error;

    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return pid->output;
}

/* 保留给应用初始化流程的兼容入口；当前工程的轮速闭环在 bsp_motor.c 内部初始化。 */
void PID_InitDefaults(void)
{
}
