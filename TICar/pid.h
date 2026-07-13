#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float current;
    float error_last;
    float integral;
    float output;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
void PID_SetTarget(PID_Controller *pid, float target);
float PID_Update(PID_Controller *pid, float current);
void PID_InitDefaults(void);

#endif
