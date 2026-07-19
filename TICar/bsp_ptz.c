/**
 * @file bsp_ptz.c
 * @brief 双轴云台 PWM 的初始化、启动、关闭、居中和软件安全限位。
 */
#include "bsp_ptz.h"

#include "ti_msp_dl_config.h"

/* 把 compare 限制在实物校准的机械安全范围内，防止舵机撞限位。 */
static uint16_t Ptz_LimitCompare(uint16_t value, uint16_t minimum, uint16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/* 停止计数器、恢复两路定时器复用并写入中心 compare；此函数本身不启动 PWM。 */
void Bsp_Ptz_Init(void)
{
    DL_TimerA_stopCounter(PWM_PTZ_INST);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_PTZ_C0_IOMUX, GPIO_PWM_PTZ_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_PTZ_C0_PORT, GPIO_PWM_PTZ_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_PTZ_C1_IOMUX, GPIO_PWM_PTZ_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_PTZ_C1_PORT, GPIO_PWM_PTZ_C1_PIN);

    Bsp_Ptz_SetCenter();
}

void Bsp_Ptz_Start(void)
{
    DL_TimerA_startCounter(PWM_PTZ_INST);
}

/* 停止 PWM，并把两路引脚切回低电平 GPIO，避免持续向舵机输出脉冲。 */
void Bsp_Ptz_Disable(void)
{
    DL_TimerA_stopCounter(PWM_PTZ_INST);

    DL_GPIO_initDigitalOutput(GPIO_PWM_PTZ_C0_IOMUX);
    DL_GPIO_clearPins(GPIO_PWM_PTZ_C0_PORT, GPIO_PWM_PTZ_C0_PIN);
    DL_GPIO_enableOutput(GPIO_PWM_PTZ_C0_PORT, GPIO_PWM_PTZ_C0_PIN);

    DL_GPIO_initDigitalOutput(GPIO_PWM_PTZ_C1_IOMUX);
    DL_GPIO_clearPins(GPIO_PWM_PTZ_C1_PORT, GPIO_PWM_PTZ_C1_PIN);
    DL_GPIO_enableOutput(GPIO_PWM_PTZ_C1_PORT, GPIO_PWM_PTZ_C1_PIN);
}

void Bsp_Ptz_SetCenter(void)
{
    Bsp_Ptz_SetCompare(BSP_PTZ_TILT_CENTER, BSP_PTZ_PAN_CENTER);
}

/* Tilt 对应 PWM C0，Pan 对应 PWM C1；写寄存器前分别执行软件安全限幅。 */
void Bsp_Ptz_SetCompare(uint16_t tilt_compare, uint16_t pan_compare)
{
    tilt_compare = Ptz_LimitCompare(
        tilt_compare, BSP_PTZ_TILT_MIN, BSP_PTZ_TILT_MAX);
    pan_compare = Ptz_LimitCompare(
        pan_compare, BSP_PTZ_PAN_MIN, BSP_PTZ_PAN_MAX);

    DL_TimerA_setCaptureCompareValue(
        PWM_PTZ_INST, tilt_compare, GPIO_PWM_PTZ_C0_IDX);
    DL_TimerA_setCaptureCompareValue(
        PWM_PTZ_INST, pan_compare, GPIO_PWM_PTZ_C1_IDX);
}
