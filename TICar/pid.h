/**
 * @file pid.h
 * @brief 通用 PID 控制器状态和接口；与 bsp_motor.c 内部的双轮速度控制器相互独立。
 */
#ifndef PID_H
#define PID_H

/*
 * 通用位置式 PID 状态。积分为每次调用的误差累加，微分为相邻两次误差之差；
 * 采样周期没有在模块内部换算，kp/ki/kd 必须与固定调用周期配套整定。
 */
typedef struct {
    float kp;          /* 比例系数。 */
    float ki;          /* 每次采样误差累加对应的积分系数。 */
    float kd;          /* 相邻误差差值对应的微分系数。 */
    float target;      /* 当前目标值。 */
    float current;     /* 最近一次输入的测量值。 */
    float error_last;  /* 上一次误差，用于差分。 */
    float integral;    /* 未限幅的累计误差。 */
    float output;      /* 最近一次计算结果，未做输出限幅。 */
} PID_Controller;

/** 初始化一个控制器并清除其历史状态；调用方必须传入有效指针。 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
/** 更新目标值，不清除积分或误差历史。 */
void PID_SetTarget(PID_Controller *pid, float target);
/** 使用当前测量值执行一次离散 PID 计算并返回未限幅输出。 */
float PID_Update(PID_Controller *pid, float current);
/** 当前为空的兼容占位接口，不会创建或修改任何 PID 控制器。 */
void PID_InitDefaults(void);

#endif
