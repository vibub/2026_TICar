/**
 * @file delivery_maneuver.c
 * @brief 送药路口动作状态机的纯逻辑实现。
 *
 * 电机、编码器、IMU 和 K230 均由 app_main.c 适配后通过输入输出结构交互。
 * 这样状态转换可以在主机环境验证，且不会在状态机内部产生阻塞等待。
 */
#include "delivery_maneuver.h"

#include <string.h>

#define MANEUVER_BRAKE_MS 300U /* 路口动作结束时的制动等待时间，单位 ms。 */
#define MANEUVER_APPROACH_CENTER_CM 50.0f /* 从首次识别路口到旋转中心的总距离，单位 cm。 */
#define MANEUVER_APPROACH_TIMEOUT_MS 6000U /* 进入路口中心阶段的最长允许时间，单位 ms。 */
#define MANEUVER_APPROACH_BLIND_SPEED_CM_S 8.0f /* 路口内航向保持前进的基础轮速，降低路口内推进速度。 */
#define MANEUVER_HEADING_HOLD_KP 0.30f /* 航向误差到轮速差修正的比例系数。 */
#define MANEUVER_HEADING_HOLD_LIMIT_CM_S 4.0f /* 航向保持单侧轮速修正上限，单位 cm/s。 */

#define MANEUVER_ALIGN_LARGE_ANGLE_DEG 8.0f /* 超过该红线角度时先执行小角度原地对正。 */
#define MANEUVER_ALIGN_SETTLE_MS 120U /* 前进切换为原地对正前的短暂稳定时间。 */
#define MANEUVER_ALIGN_ROTATE_ANGLE_KP 0.80f /* 红线角度到原地目标航向修正的比例。 */
#define MANEUVER_ALIGN_ROTATE_TARGET_LIMIT_DEG 15.0f /* 单次原地对正目标变化上限。 */
#define MANEUVER_ALIGN_ROTATE_FINISH_DEG 2.0f /* IMU 接近小角度目标后恢复低速居中。 */
#define MANEUVER_ALIGN_ROTATE_SPEED_CM_S 14.0f /* 能克服静摩擦的原地对正轮速。 */
#define MANEUVER_ALIGN_CREEP_SPEED_CM_S 5.0f /* 对正期间缓慢前进，避免慢 YOLO 时冲过路口。 */
#define MANEUVER_ALIGN_VISUAL_FALLBACK_SPEED_CM_S 4.0f /* IMU 暂失时进一步降低视觉前进轮速。 */
#define MANEUVER_ALIGN_VISUAL_FALLBACK_ERROR_KP 0.04f /* 视觉横向误差到轮速差比例。 */
#define MANEUVER_ALIGN_VISUAL_FALLBACK_ANGLE_KP 0.10f /* 视觉角度误差到轮速差比例。 */
#define MANEUVER_ALIGN_VISUAL_FALLBACK_LIMIT_CM_S 3.0f /* 视觉直控最大单侧差速。 */
#define MANEUVER_ALIGN_FILTER_ALPHA 0.35f /* 新 L 帧在对正低通中的权重。 */
#define MANEUVER_ALIGN_ERROR_TO_DEG_KP 0.08f /* 横向像素误差到目标航向偏移比例。 */
#define MANEUVER_ALIGN_ANGLE_TO_DEG_KP 0.30f /* 红线角度到目标航向偏移比例。 */
#define MANEUVER_ALIGN_OFFSET_LIMIT_DEG 6.0f /* 低速居中视觉目标航向偏移上限。 */
#define MANEUVER_ALIGN_OFFSET_SLEW_DEG 2.0f /* 每条新 L 帧目标偏移最大变化。 */
#define MANEUVER_ALIGN_FINISH_ERROR_PX 12.0f /* 允许锁存道路航向的横向误差。 */
#define MANEUVER_ALIGN_FINISH_ANGLE_DEG 3.0f /* 允许锁存道路航向的红线角度。 */
#define MANEUVER_ALIGN_STABLE_FRAMES 2U /* 对正完成所需连续新 L 帧数量。 */
#define MANEUVER_ALIGN_SAFE_TIMEOUT_MS 2000U /* 有安全航向时的最佳努力对正时间。 */
#define MANEUVER_ALIGN_HARD_TIMEOUT_MS 6000U /* 完全无安全航向时的停车等待上限。 */
#define MANEUVER_ALIGN_CREEP_MAX_CM 6.0f /* 对正阶段允许的最大总前进距离，达到后停车等待视觉。 */

#define MANEUVER_TURN_SETTLE_MS 150U /* 正式转向前的车体稳定时间，单位 ms。 */
#define MANEUVER_TURN_TIMEOUT_MS 8000U /* 实际原地旋转阶段的最长允许时间，单位 ms。 */
#define MANEUVER_TURN_MAX_TRAVEL_CM 30.0f /* 原地转向时双轮平均绝对位移安全上限。 */
#define MANEUVER_LEFT_TURN_TARGET_DEG 90.0f /* 左转相对校正道路航向的目标角度。 */
#define MANEUVER_RIGHT_TURN_TARGET_DEG -90.0f /* 右转相对校正道路航向的目标角度。 */
#define MANEUVER_TURN_COARSE_ERROR_DEG 18.0f /* 航向误差较大时使用粗转速度。 */
#define MANEUVER_TURN_FINE_ERROR_DEG 6.0f /* 航向误差进入该范围时认为到达目标。 */
#define MANEUVER_TURN_STABLE_COUNT 2U /* 航向连续满足目标范围所需控制周期。 */
#define MANEUVER_TURN_COARSE_SPEED_CM_S 18.0f /* 正式转向粗转轮速。 */
#define MANEUVER_TURN_FINE_SPEED_CM_S 14.0f /* 正式转向细转轮速。 */
#define MANEUVER_REACQUIRE_TIMEOUT_MS 2500U /* 转向后主动寻找红线的最长时间。 */
#define MANEUVER_REACQUIRE_MAX_TRAVEL_CM 18.0f /* 重捕获阶段双轮平均绝对位移上限。 */
#define MANEUVER_REACQUIRE_SWEEP_SPEED_CM_S 12.0f /* 未看到红线时原地扫描轮速。 */
#define MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG 15.0f /* 目标航向两侧的扫描角度。 */
#define MANEUVER_CROSS_MIN_CM 12.0f /* 允许确认穿过路口的最小前进距离。 */
#define MANEUVER_CROSS_FORCE_CM 24.0f /* 视觉抖动时强制完成路口动作的距离。 */
#define MANEUVER_CROSS_TIMEOUT_MS 5000U /* 路口穿越阶段最长允许时间。 */
#define MANEUVER_CROSS_BLIND_SPEED_CM_S 8.0f /* 穿越阶段丢线航向保持轮速。 */
#define MANEUVER_FOLLOW_SCALE 0.80f /* 进入和穿越路口时相对正常巡线速度。 */

static float Maneuver_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Maneuver_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float Maneuver_LimitDelta(
    float value,
    float previous,
    float maximum_delta)
{
    float delta = value - previous;

    if (delta > maximum_delta) {
        return previous + maximum_delta;
    }
    if (delta < -maximum_delta) {
        return previous - maximum_delta;
    }
    return value;
}

static float Maneuver_NormalizeAngle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static uint8_t Maneuver_Elapsed(
    uint32_t now_ms,
    uint32_t start_ms,
    uint32_t duration_ms)
{
    return ((uint32_t) (now_ms - start_ms) >= duration_ms) ? 1U : 0U;
}

static float Maneuver_CenterDistanceFrom(
    float left_start_cm,
    float right_start_cm,
    const DeliveryManeuver_Input *input)
{
    return ((input->left_position_cm - left_start_cm) +
            (input->right_position_cm - right_start_cm)) * 0.5f;
}

static float Maneuver_CenterDistance(
    const DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    return Maneuver_CenterDistanceFrom(
        maneuver->state_start_left_cm,
        maneuver->state_start_right_cm,
        input);
}

/* 原地转向左右轮方向相反，使用双轮绝对位移平均值作为宽松安全上限。 */
static float Maneuver_AverageWheelTravel(
    const DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    return (Maneuver_Abs(
                input->left_position_cm - maneuver->state_start_left_cm) +
            Maneuver_Abs(
                input->right_position_cm - maneuver->state_start_right_cm)) * 0.5f;
}

static void Maneuver_SetOutputStop(DeliveryManeuver_Output *output)
{
    output->command = DELIVERY_MANEUVER_COMMAND_STOP;
    output->follow_scale = 0.0f;
    output->left_target_cm_s = 0.0f;
    output->right_target_cm_s = 0.0f;
}

static void Maneuver_SetOutputFollow(DeliveryManeuver_Output *output)
{
    output->command = DELIVERY_MANEUVER_COMMAND_FOLLOW_LINE;
    output->follow_scale = MANEUVER_FOLLOW_SCALE;
    output->left_target_cm_s = 0.0f;
    output->right_target_cm_s = 0.0f;
}

static void Maneuver_SetOutputWheelSpeed(
    DeliveryManeuver_Output *output,
    float left_target_cm_s,
    float right_target_cm_s)
{
    output->command = DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED;
    output->follow_scale = 0.0f;
    output->left_target_cm_s = left_target_cm_s;
    output->right_target_cm_s = right_target_cm_s;
}

/* 以指定航向向前，红线低频或暂时不可见时仍由 IMU 保持直线。 */
static void Maneuver_SetHeadingHoldForward(
    float target_heading_deg,
    float base_speed_cm_s,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output)
{
    float heading_error;
    float correction;

    heading_error = Maneuver_NormalizeAngle(
        target_heading_deg - input->heading_deg);
    correction = Maneuver_Limit(
        heading_error * MANEUVER_HEADING_HOLD_KP,
        MANEUVER_HEADING_HOLD_LIMIT_CM_S);

    Maneuver_SetOutputWheelSpeed(
        output,
        base_speed_cm_s - correction,
        base_speed_cm_s + correction);
}

/* IMU 暂失时只允许低速视觉差速，限幅保证两轮继续向前而不原地反转。 */
static void Maneuver_SetAlignVisualFallback(
    const DeliveryManeuver *maneuver,
    DeliveryManeuver_Output *output)
{
    float correction = -(
        MANEUVER_ALIGN_VISUAL_FALLBACK_ERROR_KP *
            maneuver->filtered_line_error_px +
        MANEUVER_ALIGN_VISUAL_FALLBACK_ANGLE_KP *
            maneuver->filtered_line_angle_deg);

    correction = Maneuver_Limit(
        correction,
        MANEUVER_ALIGN_VISUAL_FALLBACK_LIMIT_CM_S);
    Maneuver_SetOutputWheelSpeed(
        output,
        MANEUVER_ALIGN_VISUAL_FALLBACK_SPEED_CM_S - correction,
        MANEUVER_ALIGN_VISUAL_FALLBACK_SPEED_CM_S + correction);
}

static void Maneuver_EnterState(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_State state,
    const DeliveryManeuver_Input *input)
{
    maneuver->state = state;
    maneuver->state_start_ms = input->now_ms;
    maneuver->state_start_left_cm = input->left_position_cm;
    maneuver->state_start_right_cm = input->right_position_cm;
    maneuver->heading_stable_count = 0U;
}

static void Maneuver_SetFault(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Fault fault)
{
    maneuver->state = DELIVERY_MANEUVER_STATE_FAULT;
    maneuver->fault = fault;
}

static uint8_t Maneuver_LineUsable(const DeliveryManeuver_Input *input)
{
    return ((input->line_fresh != 0U) &&
            (input->line_valid != 0U)) ? 1U : 0U;
}

static void Maneuver_SetAlignPhase(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_AlignPhase phase,
    uint32_t now_ms)
{
    maneuver->align_phase = phase;
    maneuver->align_phase_start_ms = now_ms;
}

/* 新 L 帧只更新一次对正目标，避免在 20 ms 控制周期内重复放大旧视觉误差。 */
static void Maneuver_UpdateAlignObservation(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    float raw_offset;
    float rotate_offset;

    if (maneuver->alignment_ready != 0U) {
        return;
    }
    if (input->line_new == 0U) {
        return;
    }
    if (Maneuver_LineUsable(input) == 0U) {
        maneuver->alignment_stable_count = 0U;
        return;
    }

    if (maneuver->align_filter_ready == 0U) {
        maneuver->filtered_line_error_px = (float) input->line_error_px;
        maneuver->filtered_line_angle_deg =
            (float) input->line_angle_d10 / 10.0f;
        maneuver->align_filter_ready = 1U;
    } else {
        maneuver->filtered_line_error_px +=
            MANEUVER_ALIGN_FILTER_ALPHA *
            ((float) input->line_error_px -
             maneuver->filtered_line_error_px);
        maneuver->filtered_line_angle_deg +=
            MANEUVER_ALIGN_FILTER_ALPHA *
            (((float) input->line_angle_d10 / 10.0f) -
             maneuver->filtered_line_angle_deg);
    }

    if (input->imu_fresh == 0U) {
        maneuver->alignment_stable_count = 0U;
        return;
    }

    if ((maneuver->alignment_ready == 0U) &&
        (maneuver->align_phase == DELIVERY_MANEUVER_ALIGN_CREEP) &&
        (Maneuver_Abs(maneuver->filtered_line_angle_deg) >
         MANEUVER_ALIGN_LARGE_ANGLE_DEG)) {
        rotate_offset = Maneuver_Limit(
            -MANEUVER_ALIGN_ROTATE_ANGLE_KP *
                maneuver->filtered_line_angle_deg,
            MANEUVER_ALIGN_ROTATE_TARGET_LIMIT_DEG);
        maneuver->target_heading_deg = Maneuver_NormalizeAngle(
            input->heading_deg + rotate_offset);
        maneuver->safe_heading_deg = maneuver->target_heading_deg;
        maneuver->safe_heading_valid = 1U;
        maneuver->alignment_stable_count = 0U;
        Maneuver_SetAlignPhase(
            maneuver,
            DELIVERY_MANEUVER_ALIGN_SETTLE,
            input->now_ms);
        return;
    }

    /* SETTLE/ROTATE 期间只更新滤波诊断，不用新帧覆盖正在执行的小角度目标。 */
    if (maneuver->align_phase != DELIVERY_MANEUVER_ALIGN_CREEP) {
        maneuver->alignment_stable_count = 0U;
        return;
    }

    if (maneuver->alignment_ready == 0U) {
        raw_offset = -(
            MANEUVER_ALIGN_ERROR_TO_DEG_KP *
                maneuver->filtered_line_error_px +
            MANEUVER_ALIGN_ANGLE_TO_DEG_KP *
                maneuver->filtered_line_angle_deg);
        raw_offset = Maneuver_Limit(
            raw_offset,
            MANEUVER_ALIGN_OFFSET_LIMIT_DEG);
        maneuver->align_visual_offset_deg = Maneuver_LimitDelta(
            raw_offset,
            maneuver->align_visual_offset_deg,
            MANEUVER_ALIGN_OFFSET_SLEW_DEG);
        maneuver->target_heading_deg = Maneuver_NormalizeAngle(
            input->heading_deg + maneuver->align_visual_offset_deg);
        maneuver->safe_heading_deg = maneuver->target_heading_deg;
        maneuver->safe_heading_valid = 1U;

        if ((Maneuver_Abs((float) input->line_error_px) <=
             MANEUVER_ALIGN_FINISH_ERROR_PX) &&
            (Maneuver_Abs((float) input->line_angle_d10 / 10.0f) <=
             MANEUVER_ALIGN_FINISH_ANGLE_DEG)) {
            if (maneuver->alignment_stable_count < 0xFFU) {
                maneuver->alignment_stable_count++;
            }
        } else {
            maneuver->alignment_stable_count = 0U;
        }

        if (maneuver->alignment_stable_count >=
            MANEUVER_ALIGN_STABLE_FRAMES) {
            maneuver->alignment_ready = 1U;
            maneuver->approach_heading_deg = Maneuver_NormalizeAngle(
                input->heading_deg);
            maneuver->target_heading_deg = maneuver->approach_heading_deg;
            maneuver->safe_heading_deg = maneuver->approach_heading_deg;
            maneuver->align_visual_offset_deg = 0.0f;
            Maneuver_SetAlignPhase(
                maneuver,
                DELIVERY_MANEUVER_ALIGN_READY,
                input->now_ms);
        }
    }
}

/* 没有红线时在目标航向两侧往返扫描，扫描仍受时间和距离兜底约束。 */
static void Maneuver_SetReacquireSweep(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output)
{
    float desired_heading;
    float heading_error;
    float turn_sign;

    desired_heading = Maneuver_NormalizeAngle(
        maneuver->target_heading_deg +
        (float) maneuver->reacquire_sweep_direction *
        MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG);
    heading_error = Maneuver_NormalizeAngle(
        desired_heading - input->heading_deg);
    if (Maneuver_Abs(heading_error) <= 2.0f) {
        maneuver->reacquire_sweep_direction =
            (int8_t) (-maneuver->reacquire_sweep_direction);
        desired_heading = Maneuver_NormalizeAngle(
            maneuver->target_heading_deg +
            (float) maneuver->reacquire_sweep_direction *
            MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG);
        heading_error = Maneuver_NormalizeAngle(
            desired_heading - input->heading_deg);
    }

    turn_sign = (heading_error >= 0.0f) ? 1.0f : -1.0f;
    Maneuver_SetOutputWheelSpeed(
        output,
        -turn_sign * MANEUVER_REACQUIRE_SWEEP_SPEED_CM_S,
        turn_sign * MANEUVER_REACQUIRE_SWEEP_SPEED_CM_S);
}

void DeliveryManeuver_Init(DeliveryManeuver *maneuver)
{
    DeliveryManeuver_Reset(maneuver);
}

void DeliveryManeuver_Reset(DeliveryManeuver *maneuver)
{
    if (maneuver == NULL) {
        return;
    }
    memset(maneuver, 0, sizeof(*maneuver));
    maneuver->state = DELIVERY_MANEUVER_STATE_IDLE;
    maneuver->align_phase = DELIVERY_MANEUVER_ALIGN_CREEP;
    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_SETTLE;
    maneuver->decision = ROUTE_DECISION_NONE;
    maneuver->fault = DELIVERY_MANEUVER_FAULT_NONE;
}

uint8_t DeliveryManeuver_StartAlignment(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    if ((maneuver == NULL) || (input == NULL) ||
        (maneuver->state != DELIVERY_MANEUVER_STATE_IDLE)) {
        return 0U;
    }

    DeliveryManeuver_Reset(maneuver);
    maneuver->state = DELIVERY_MANEUVER_STATE_ALIGN;
    maneuver->state_start_ms = input->now_ms;
    maneuver->align_phase_start_ms = input->now_ms;
    maneuver->state_start_left_cm = input->left_position_cm;
    maneuver->state_start_right_cm = input->right_position_cm;
    maneuver->junction_start_left_cm = input->left_position_cm;
    maneuver->junction_start_right_cm = input->right_position_cm;
    maneuver->target_heading_deg = Maneuver_NormalizeAngle(
        input->heading_deg);
    maneuver->approach_heading_deg = maneuver->target_heading_deg;
    return 1U;
}

uint8_t DeliveryManeuver_SetDecision(
    DeliveryManeuver *maneuver,
    RoutePlanner_Decision decision)
{
    if ((maneuver == NULL) ||
        (maneuver->state == DELIVERY_MANEUVER_STATE_IDLE) ||
        (maneuver->state == DELIVERY_MANEUVER_STATE_FAULT) ||
        ((decision != ROUTE_DECISION_LEFT) &&
         (decision != ROUTE_DECISION_RIGHT) &&
         (decision != ROUTE_DECISION_FRONT))) {
        return 0U;
    }

    if (maneuver->decision_ready != 0U) {
        return (maneuver->decision == decision) ? 1U : 0U;
    }

    maneuver->decision = decision;
    maneuver->decision_ready = 1U;
    maneuver->reacquire_sweep_direction =
        (decision == ROUTE_DECISION_RIGHT) ? -1 : 1;
    return 1U;
}

uint8_t DeliveryManeuver_IsAlignmentReady(
    const DeliveryManeuver *maneuver)
{
    if ((maneuver == NULL) ||
        (maneuver->state == DELIVERY_MANEUVER_STATE_IDLE) ||
        (maneuver->state == DELIVERY_MANEUVER_STATE_FAULT)) {
        return 0U;
    }
    return maneuver->alignment_ready;
}

void DeliveryManeuver_Update(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output)
{
    float center_distance; /* 当前状态内左右轮平均前进距离，单位 cm。 */
    float wheel_travel; /* 当前状态内双轮绝对位移平均值，单位 cm。 */
    float heading_error; /* 目标航向与当前航向之间的归一化误差，单位 °。 */
    float turn_sign; /* 原地转向方向，1 表示左转，-1 表示右转。 */
    float turn_speed; /* 当前粗转或细转使用的轮速，单位 cm/s。 */
    float turn_target; /* 基于校正道路航向计算的正式转向目标，单位 °。 */

    if (output == NULL) {
        return;
    }
    Maneuver_SetOutputStop(output);
    output->request_commit = 0U;

    if ((maneuver == NULL) || (input == NULL)) {
        return;
    }
    if ((maneuver->state == DELIVERY_MANEUVER_STATE_IDLE) ||
        (maneuver->state == DELIVERY_MANEUVER_STATE_FAULT)) {
        return;
    }

    maneuver->junction_progress_cm = Maneuver_CenterDistanceFrom(
        maneuver->junction_start_left_cm,
        maneuver->junction_start_right_cm,
        input);
    center_distance = Maneuver_CenterDistance(maneuver, input);
    wheel_travel = Maneuver_AverageWheelTravel(maneuver, input);

    switch (maneuver->state) {
        case DELIVERY_MANEUVER_STATE_ALIGN:
            Maneuver_UpdateAlignObservation(maneuver, input);

            if ((maneuver->alignment_ready == 0U) &&
                (maneuver->safe_heading_valid != 0U) &&
                (Maneuver_Elapsed(
                     input->now_ms,
                     maneuver->state_start_ms,
                     MANEUVER_ALIGN_SAFE_TIMEOUT_MS) != 0U)) {
                /* 视觉抖动超过最佳努力窗口时使用最近一次安全航向继续流程。 */
                maneuver->alignment_ready = 1U;
                maneuver->approach_heading_deg =
                    maneuver->safe_heading_deg;
                maneuver->target_heading_deg =
                    maneuver->approach_heading_deg;
                maneuver->align_visual_offset_deg = 0.0f;
                Maneuver_SetAlignPhase(
                    maneuver,
                    DELIVERY_MANEUVER_ALIGN_READY,
                    input->now_ms);
            }

            if ((maneuver->safe_heading_valid == 0U) &&
                (Maneuver_Elapsed(
                     input->now_ms,
                     maneuver->state_start_ms,
                     MANEUVER_ALIGN_HARD_TIMEOUT_MS) != 0U)) {
                Maneuver_SetFault(
                    maneuver,
                    DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }

            if ((maneuver->alignment_ready != 0U) &&
                (maneuver->decision_ready != 0U)) {
                maneuver->target_heading_deg =
                    maneuver->approach_heading_deg;
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_APPROACH_CENTER,
                    input);
                break;
            }

            switch (maneuver->align_phase) {
                case DELIVERY_MANEUVER_ALIGN_CREEP:
                    if (maneuver->junction_progress_cm >=
                        MANEUVER_ALIGN_CREEP_MAX_CM) {
                        break;
                    }
                    if ((input->imu_fresh != 0U) &&
                        (maneuver->safe_heading_valid != 0U)) {
                        Maneuver_SetHeadingHoldForward(
                            maneuver->target_heading_deg,
                            MANEUVER_ALIGN_CREEP_SPEED_CM_S,
                            input,
                            output);
                    } else if (Maneuver_LineUsable(input) != 0U) {
                        Maneuver_SetAlignVisualFallback(
                            maneuver,
                            output);
                    }
                    break;

                case DELIVERY_MANEUVER_ALIGN_SETTLE:
                    if ((Maneuver_Elapsed(
                             input->now_ms,
                             maneuver->align_phase_start_ms,
                             MANEUVER_ALIGN_SETTLE_MS) != 0U) &&
                        (input->imu_fresh != 0U)) {
                        Maneuver_SetAlignPhase(
                            maneuver,
                            DELIVERY_MANEUVER_ALIGN_ROTATE,
                            input->now_ms);
                    }
                    break;

                case DELIVERY_MANEUVER_ALIGN_ROTATE:
                    if (input->imu_fresh == 0U) {
                        break;
                    }
                    heading_error = Maneuver_NormalizeAngle(
                        maneuver->target_heading_deg -
                        input->heading_deg);
                    if (Maneuver_Abs(heading_error) <=
                        MANEUVER_ALIGN_ROTATE_FINISH_DEG) {
                        maneuver->alignment_stable_count = 0U;
                        Maneuver_SetAlignPhase(
                            maneuver,
                            DELIVERY_MANEUVER_ALIGN_CREEP,
                            input->now_ms);
                        break;
                    }
                    turn_sign = (heading_error > 0.0f) ? 1.0f : -1.0f;
                    Maneuver_SetOutputWheelSpeed(
                        output,
                        -turn_sign * MANEUVER_ALIGN_ROTATE_SPEED_CM_S,
                        turn_sign * MANEUVER_ALIGN_ROTATE_SPEED_CM_S);
                    break;

                case DELIVERY_MANEUVER_ALIGN_READY:
                    /*
                     * 车头与红线中心对正后立即停车，等待对正画面中的新 D 帧。
                     * 不能继续爬行，否则慢 YOLO 在 T 字路口尚未给出数字时，
                     * 小车可能先驶离主红线并造成丢线。
                     */
                    break;

                default:
                    Maneuver_SetFault(
                        maneuver,
                        DELIVERY_MANEUVER_FAULT_INVALID_START);
                    break;
            }
            break;

        case DELIVERY_MANEUVER_STATE_APPROACH_CENTER:
            if (maneuver->junction_progress_cm >=
                MANEUVER_APPROACH_CENTER_CM) {
                if ((maneuver->decision == ROUTE_DECISION_LEFT) ||
                    (maneuver->decision == ROUTE_DECISION_RIGHT)) {
                    turn_target =
                        (maneuver->decision == ROUTE_DECISION_LEFT) ?
                            MANEUVER_LEFT_TURN_TARGET_DEG :
                            MANEUVER_RIGHT_TURN_TARGET_DEG;
                    maneuver->target_heading_deg = Maneuver_NormalizeAngle(
                        maneuver->approach_heading_deg + turn_target);
                    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_SETTLE;
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_TURN,
                        input);
                } else {
                    maneuver->target_heading_deg =
                        maneuver->approach_heading_deg;
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_CROSS,
                        input);
                }
                break;
            }
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_APPROACH_TIMEOUT_MS) != 0U) {
                Maneuver_SetFault(
                    maneuver,
                    DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }
            if (input->imu_fresh != 0U) {
                Maneuver_SetHeadingHoldForward(
                    maneuver->approach_heading_deg,
                    MANEUVER_APPROACH_BLIND_SPEED_CM_S,
                    input,
                    output);
            } else if (Maneuver_LineUsable(input) != 0U) {
                Maneuver_SetOutputFollow(output);
            }
            break;

        case DELIVERY_MANEUVER_STATE_TURN:
            if (maneuver->turn_phase == DELIVERY_MANEUVER_TURN_SETTLE) {
                if ((Maneuver_Elapsed(
                         input->now_ms,
                         maneuver->state_start_ms,
                         MANEUVER_TURN_SETTLE_MS) != 0U) &&
                    (input->imu_fresh != 0U)) {
                    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_ROTATE;
                    maneuver->state_start_ms = input->now_ms;
                    maneuver->state_start_left_cm = input->left_position_cm;
                    maneuver->state_start_right_cm = input->right_position_cm;
                    maneuver->heading_stable_count = 0U;
                }
                break;
            }
            if (maneuver->turn_phase != DELIVERY_MANEUVER_TURN_ROTATE) {
                Maneuver_SetFault(
                    maneuver,
                    DELIVERY_MANEUVER_FAULT_INVALID_START);
                break;
            }
            if ((wheel_travel > MANEUVER_TURN_MAX_TRAVEL_CM) ||
                (Maneuver_Elapsed(
                     input->now_ms,
                     maneuver->state_start_ms,
                     MANEUVER_TURN_TIMEOUT_MS) != 0U)) {
                Maneuver_SetFault(
                    maneuver,
                    (wheel_travel > MANEUVER_TURN_MAX_TRAVEL_CM) ?
                        DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT :
                        DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }
            if (input->imu_fresh == 0U) {
                maneuver->heading_stable_count = 0U;
                break;
            }

            heading_error = Maneuver_NormalizeAngle(
                maneuver->target_heading_deg - input->heading_deg);
            if (Maneuver_Abs(heading_error) <= MANEUVER_TURN_FINE_ERROR_DEG) {
                if (maneuver->heading_stable_count < 0xFFU) {
                    maneuver->heading_stable_count++;
                }
                if (maneuver->heading_stable_count >=
                    MANEUVER_TURN_STABLE_COUNT) {
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_REACQUIRE,
                        input);
                }
                break;
            }

            maneuver->heading_stable_count = 0U;
            turn_sign = (heading_error > 0.0f) ? 1.0f : -1.0f;
            turn_speed = (Maneuver_Abs(heading_error) >
                          MANEUVER_TURN_COARSE_ERROR_DEG) ?
                         MANEUVER_TURN_COARSE_SPEED_CM_S :
                         MANEUVER_TURN_FINE_SPEED_CM_S;
            Maneuver_SetOutputWheelSpeed(
                output,
                -turn_sign * turn_speed,
                turn_sign * turn_speed);
            break;

        case DELIVERY_MANEUVER_STATE_REACQUIRE:
            if ((input->line_new != 0U) &&
                (Maneuver_LineUsable(input) != 0U)) {
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_CROSS,
                    input);
                break;
            }
            if ((wheel_travel >= MANEUVER_REACQUIRE_MAX_TRAVEL_CM) ||
                (Maneuver_Elapsed(
                     input->now_ms,
                     maneuver->state_start_ms,
                     MANEUVER_REACQUIRE_TIMEOUT_MS) != 0U)) {
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_CROSS,
                    input);
                break;
            }
            if (Maneuver_LineUsable(input) != 0U) {
                Maneuver_SetOutputFollow(output);
            } else if (input->imu_fresh != 0U) {
                Maneuver_SetReacquireSweep(maneuver, input, output);
            }
            break;

        case DELIVERY_MANEUVER_STATE_CROSS:
            if (center_distance >= MANEUVER_CROSS_FORCE_CM) {
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_BRAKE_AFTER,
                    input);
                break;
            }
            if ((center_distance >= MANEUVER_CROSS_MIN_CM) &&
                (input->line_new != 0U) &&
                (Maneuver_LineUsable(input) != 0U) &&
                ((input->direction_mask & K230_LINE_DIRECTION_FRONT) != 0U) &&
                (input->junction_active == 0U)) {
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_BRAKE_AFTER,
                    input);
                break;
            }
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_CROSS_TIMEOUT_MS) != 0U) {
                if (center_distance >= MANEUVER_CROSS_MIN_CM) {
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_BRAKE_AFTER,
                        input);
                } else {
                    Maneuver_SetFault(
                        maneuver,
                        DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                }
                break;
            }
            if (Maneuver_LineUsable(input) != 0U) {
                Maneuver_SetOutputFollow(output);
            } else if (input->imu_fresh != 0U) {
                Maneuver_SetHeadingHoldForward(
                    maneuver->target_heading_deg,
                    MANEUVER_CROSS_BLIND_SPEED_CM_S,
                    input,
                    output);
            }
            break;

        case DELIVERY_MANEUVER_STATE_BRAKE_AFTER:
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_BRAKE_MS) != 0U) {
                maneuver->commit_requested = 1U;
                output->request_commit = 1U;
            }
            break;

        default:
            Maneuver_SetFault(
                maneuver,
                DELIVERY_MANEUVER_FAULT_INVALID_START);
            break;
    }
}

void DeliveryManeuver_ReportCommit(
    DeliveryManeuver *maneuver,
    uint8_t success)
{
    if ((maneuver == NULL) ||
        (maneuver->state != DELIVERY_MANEUVER_STATE_BRAKE_AFTER) ||
        (maneuver->commit_requested == 0U)) {
        return;
    }
    if (success == 0U) {
        Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_COMMIT);
        return;
    }
    DeliveryManeuver_Reset(maneuver);
}

uint8_t DeliveryManeuver_IsActive(const DeliveryManeuver *maneuver)
{
    if (maneuver == NULL) {
        return 0U;
    }
    return (maneuver->state != DELIVERY_MANEUVER_STATE_IDLE) ? 1U : 0U;
}
