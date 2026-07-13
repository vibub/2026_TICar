#include "pid.h"

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

void PID_InitDefaults(void)
{
}
