#include "bsp_motor.h"

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define BSP_MOTOR_PWM_PERIOD 10000U
#define BSP_MOTOR_MAX_RATIO  0.80f
#define BSP_MOTOR_LEFT_DIR   -1.0f // New-left wheel uses the old-right channel, so positive command must run old-right reverse.
#define BSP_MOTOR_RIGHT_DIR  -1.0f // New-right wheel uses the old-left channel, so positive command must run old-left reverse.
#define BSP_MOTOR_LEFT_FORWARD_SCALE  1.00f // New-left forward uses old-right reverse trim.
#define BSP_MOTOR_RIGHT_FORWARD_SCALE 0.85f // New-right forward uses old-left reverse trim.
#define BSP_MOTOR_LEFT_REVERSE_SCALE  1.00f // New-left reverse uses old-right forward trim.
#define BSP_MOTOR_RIGHT_REVERSE_SCALE 0.91f // New-right reverse uses old-left forward trim.

static float Motor_LimitRatio(float ratio)
{
    if (ratio > BSP_MOTOR_MAX_RATIO) {
        return BSP_MOTOR_MAX_RATIO;
    }
    if (ratio < -BSP_MOTOR_MAX_RATIO) {
        return -BSP_MOTOR_MAX_RATIO;
    }
    return ratio;
}

static void Motor_SetOne(float ratio, uint32_t dir_pin, DL_TIMER_CC_INDEX compare_index)
{
    uint32_t compare;

    ratio = Motor_LimitRatio(ratio);

    if (ratio >= 0.0f) {
        DL_GPIO_setPins(GPIO_DC_PORT, dir_pin);
        compare = (uint32_t) (ratio * (float) BSP_MOTOR_PWM_PERIOD);
    } else {
        DL_GPIO_clearPins(GPIO_DC_PORT, dir_pin);
        compare = (uint32_t) ((1.0f + ratio) * (float) BSP_MOTOR_PWM_PERIOD);
    }

    DL_Timer_setCaptureCompareValue(PWM_DC_INST, compare, compare_index);
}

static float Motor_ApplyTrim(float ratio, float dir, float forward_scale, float reverse_scale)
{
    if (ratio >= 0.0f) {
        return ratio * forward_scale * dir; // Choose trim by logical command, then apply wiring direction.
    }
    return ratio * reverse_scale * dir; // Choose trim by logical command, then apply wiring direction.
}

void Bsp_Motor_Init(void)
{
    Bsp_Motor_Disable(); // Leave the bridge electrically idle until a motion command is issued.
}

void Bsp_Motor_Disable(void)
{
    DL_TimerG_stopCounter(PWM_DC_INST); // Stop PWM so debug/CCD modes cannot accidentally drive the motors.
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, BSP_MOTOR_PWM_PERIOD, GPIO_PWM_DC_C0_IDX); // Keep PWM command at zero-drive side.
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, BSP_MOTOR_PWM_PERIOD, GPIO_PWM_DC_C1_IDX); // Keep PWM command at zero-drive side.
    DL_GPIO_clearPins(GPIO_DC_PORT, GPIO_DC_AIN0_PIN | GPIO_DC_AIN2_PIN); // Clear direction pins so the H-bridge is idle.
}

void Bsp_Motor_Stop(void)
{
    DL_TimerG_startCounter(PWM_DC_INST); // Keep PWM running so the output level is controlled.
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, 0U, GPIO_PWM_DC_C0_IDX); // Force PWM input high for active brake.
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, 0U, GPIO_PWM_DC_C1_IDX); // Force PWM input high for active brake.
    DL_GPIO_setPins(GPIO_DC_PORT, GPIO_DC_AIN0_PIN | GPIO_DC_AIN2_PIN); // Both H-bridge inputs high should short-brake the motors.
}

void Bsp_Motor_Coast(void)
{
    DL_TimerG_startCounter(PWM_DC_INST); // Keep PWM running so the output level is controlled.
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, BSP_MOTOR_PWM_PERIOD, GPIO_PWM_DC_C0_IDX); // Force PWM input low for coast.
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, BSP_MOTOR_PWM_PERIOD, GPIO_PWM_DC_C1_IDX); // Force PWM input low for coast.
    DL_GPIO_clearPins(GPIO_DC_PORT, GPIO_DC_AIN0_PIN | GPIO_DC_AIN2_PIN); // Both H-bridge inputs low lets the motors coast.
}

void Bsp_Motor_Set(float left_ratio, float right_ratio)
{
    DL_TimerG_startCounter(PWM_DC_INST);
    Motor_SetOne(Motor_ApplyTrim(left_ratio, BSP_MOTOR_LEFT_DIR,
                                 BSP_MOTOR_LEFT_FORWARD_SCALE,
                                 BSP_MOTOR_LEFT_REVERSE_SCALE),
                 GPIO_DC_AIN0_PIN, GPIO_PWM_DC_C0_IDX); // Logical left wheel is wired to the old-right motor channel.
    Motor_SetOne(Motor_ApplyTrim(right_ratio, BSP_MOTOR_RIGHT_DIR,
                                 BSP_MOTOR_RIGHT_FORWARD_SCALE,
                                 BSP_MOTOR_RIGHT_REVERSE_SCALE),
                 GPIO_DC_AIN2_PIN, GPIO_PWM_DC_C1_IDX); // Logical right wheel is wired to the old-left motor channel.
}

void Bsp_Motor_SetLeftRaw(uint8_t dir_high, uint32_t compare)
{
    if (compare > BSP_MOTOR_PWM_PERIOD) {
        compare = BSP_MOTOR_PWM_PERIOD;
    }

    DL_TimerG_startCounter(PWM_DC_INST);
    if (dir_high) {
        DL_GPIO_setPins(GPIO_DC_PORT, GPIO_DC_AIN0_PIN); // Logical left raw drives the old-right motor channel.
    } else {
        DL_GPIO_clearPins(GPIO_DC_PORT, GPIO_DC_AIN0_PIN); // Logical left raw drives the old-right motor channel.
    }
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, compare, GPIO_PWM_DC_C0_IDX); // Logical left raw drives the old-right motor channel.
}

void Bsp_Motor_SetRightRaw(uint8_t dir_high, uint32_t compare)
{
    if (compare > BSP_MOTOR_PWM_PERIOD) {
        compare = BSP_MOTOR_PWM_PERIOD;
    }

    DL_TimerG_startCounter(PWM_DC_INST);
    if (dir_high) {
        DL_GPIO_setPins(GPIO_DC_PORT, GPIO_DC_AIN2_PIN); // Logical right raw drives the old-left motor channel.
    } else {
        DL_GPIO_clearPins(GPIO_DC_PORT, GPIO_DC_AIN2_PIN); // Logical right raw drives the old-left motor channel.
    }
    DL_Timer_setCaptureCompareValue(PWM_DC_INST, compare, GPIO_PWM_DC_C1_IDX); // Logical right raw drives the old-left motor channel.
}
