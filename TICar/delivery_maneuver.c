/**
 * @file delivery_maneuver.c
 * @brief 送药路口动作状态机的纯逻辑实现。
 *
 * 电机、编码器、IMU 和 K230 均由 app_main.c 适配后通过输入输出结构交互。
 * 这样状态转换可以在主机环境验证，且不会在状态机内部产生阻塞等待。
 */
#include "delivery_maneuver.h"

#include <string.h>

#define MANEUVER_BRAKE_MS 300U /* 路口动作开始和结束时的制动等待时间，单位 ms。 */
#define MANEUVER_APPROACH_CENTER_CM 50.0f /* 识别路口后继续前进到旋转中心的距离，单位 cm。 */
#define MANEUVER_APPROACH_TIMEOUT_MS 6000U /* 进入路口中心阶段的最长允许时间，单位 ms。 */
#define MANEUVER_APPROACH_BLIND_SPEED_CM_S 10.0f /* 路口内看不到红线时航向保持前进的基础轮速，单位 cm/s。 */
#define MANEUVER_HEADING_HOLD_KP 0.30f /* 丢线前进时航向误差到轮速差修正的比例系数。 */
#define MANEUVER_HEADING_HOLD_LIMIT_CM_S 4.0f /* 丢线前进时单侧轮速修正的最大值，单位 cm/s。 */
#define MANEUVER_TURN_SETTLE_MS 150U /* 开始计算相对转向目标前的车体稳定时间，单位 ms。 */
#define MANEUVER_TURN_TIMEOUT_MS 8000U /* 实际原地旋转阶段的最长允许时间，单位 ms。 */
#define MANEUVER_TURN_MAX_TRAVEL_CM 30.0f /* 原地转向时双轮平均绝对位移的宽松安全上限，单位 cm。 */
#define MANEUVER_LEFT_TURN_TARGET_DEG 90.0f /* 左转相对转角，左转为正，单位 °。 */
#define MANEUVER_RIGHT_TURN_TARGET_DEG -90.0f /* 右转相对转角，右转为负，单位 °。 */
#define MANEUVER_TURN_COARSE_ERROR_DEG 18.0f /* 航向误差大于该值时使用粗转速度，单位 °。 */
#define MANEUVER_TURN_FINE_ERROR_DEG 6.0f /* 航向误差进入该范围时认为到达目标角度，单位 °。 */
#define MANEUVER_TURN_STABLE_COUNT 2U /* 航向连续满足目标误差范围所需的控制周期数。 */
#define MANEUVER_TURN_COARSE_SPEED_CM_S 18.0f /* 航向误差较大时的原地转向轮速，单位 cm/s。 */
#define MANEUVER_TURN_FINE_SPEED_CM_S 14.0f /* 接近目标航向时仍能克服静摩擦的细转轮速，单位 cm/s。 */
#define MANEUVER_REACQUIRE_TIMEOUT_MS 2500U /* 转向后主动寻找红线的最长时间，单位 ms。 */
#define MANEUVER_REACQUIRE_MAX_TRAVEL_CM 18.0f /* 重捕获阶段双轮平均绝对位移的继续流程门槛，单位 cm。 */
#define MANEUVER_REACQUIRE_SWEEP_SPEED_CM_S 12.0f /* 未看到红线时原地扫描的轮速，单位 cm/s。 */
#define MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG 15.0f /* 重捕获时在目标航向两侧的扫描角度，单位 °。 */
#define MANEUVER_CROSS_MIN_CM 12.0f /* 允许确认已经穿过路口的最小前进距离，单位 cm。 */
#define MANEUVER_CROSS_FORCE_CM 24.0f /* 视觉持续抖动时强制完成本次路口动作的前进距离，单位 cm。 */
#define MANEUVER_CROSS_TIMEOUT_MS 5000U /* 路口穿越阶段的最长允许时间，单位 ms。 */
#define MANEUVER_CROSS_BLIND_SPEED_CM_S 8.0f /* 穿越阶段丢线时航向保持前进的轮速，单位 cm/s。 */
#define MANEUVER_FOLLOW_SCALE 0.80f /* 进入和穿越路口时相对正常巡线速度的缩放比例。 */

static float Maneuver_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
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

static float Maneuver_CenterDistance(
    const DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    return ((input->left_position_cm - maneuver->state_start_left_cm) +
            (input->right_position_cm - maneuver->state_start_right_cm)) * 0.5f;
}

/*
 * 原地转向和扫线时左右轮方向相反，不能使用有符号中心距离。
 * 取两轮绝对位移平均值可避免单轮空转或计数跳变直接代表整车转动进度。
 */
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

/* 红线不可用时，以较低速度保持指定航向向前，避免在路口中心永久停车。 */
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
    correction = heading_error * MANEUVER_HEADING_HOLD_KP;
    if (correction > MANEUVER_HEADING_HOLD_LIMIT_CM_S) {
        correction = MANEUVER_HEADING_HOLD_LIMIT_CM_S;
    } else if (correction < -MANEUVER_HEADING_HOLD_LIMIT_CM_S) {
        correction = -MANEUVER_HEADING_HOLD_LIMIT_CM_S;
    }

    Maneuver_SetOutputWheelSpeed(
        output,
        base_speed_cm_s - correction,
        base_speed_cm_s + correction);
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

/* 没有红线时在目标航向两侧往返扫描，扫描本身仍受时间和距离兜底约束。 */
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
    maneuver->decision = ROUTE_DECISION_NONE;
    maneuver->fault = DELIVERY_MANEUVER_FAULT_NONE;
}

uint8_t DeliveryManeuver_Start(
    DeliveryManeuver *maneuver,
    RoutePlanner_Decision decision,
    const DeliveryManeuver_Input *input)
{
    if ((maneuver == NULL) || (input == NULL) ||
        (maneuver->state != DELIVERY_MANEUVER_STATE_IDLE) ||
        ((decision != ROUTE_DECISION_LEFT) &&
         (decision != ROUTE_DECISION_RIGHT) &&
         (decision != ROUTE_DECISION_FRONT))) {
        return 0U;
    }

    maneuver->decision = decision;
    maneuver->fault = DELIVERY_MANEUVER_FAULT_NONE;
    maneuver->state_start_ms = input->now_ms;
    maneuver->state_start_left_cm = input->left_position_cm;
    maneuver->state_start_right_cm = input->right_position_cm;
    maneuver->target_heading_deg = input->heading_deg;
    maneuver->approach_heading_deg = input->heading_deg;
    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_SETTLE;
    maneuver->state = DELIVERY_MANEUVER_STATE_BRAKE;
    maneuver->commit_requested = 0U;
    maneuver->heading_stable_count = 0U;
    maneuver->reacquire_sweep_direction =
        (decision == ROUTE_DECISION_RIGHT) ? -1 : 1;
    return 1U;
}

void DeliveryManeuver_Update(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output)
{
    float center_distance; /* 当前状态内左右轮平均前进距离，单位 cm。 */
    float wheel_travel; /* 当前状态内双轮绝对位移平均值，单位 cm。 */
    float heading_error; /* IMU 目标航向与当前航向之间的归一化误差，单位 °。 */
    float turn_sign; /* 原地转向方向，1 表示左转，-1 表示右转。 */
    float turn_speed; /* 当前粗转或细转使用的轮速，单位 cm/s。 */
    float relative_turn; /* 本次路线决策对应的相对转角，单位 °。 */

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

    center_distance = Maneuver_CenterDistance(maneuver, input);
    wheel_travel = Maneuver_AverageWheelTravel(maneuver, input);

    switch (maneuver->state) {
        case DELIVERY_MANEUVER_STATE_BRAKE:
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_BRAKE_MS) != 0U) {
                /* 只在航向新鲜时更新基准，避免把过期读数覆盖掉已有航向。 */
                if (input->imu_fresh != 0U) {
                    maneuver->approach_heading_deg = input->heading_deg;
                }
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_APPROACH_CENTER,
                    input);
            }
            break;

        case DELIVERY_MANEUVER_STATE_APPROACH_CENTER:
            if (center_distance >= MANEUVER_APPROACH_CENTER_CM) {
                if ((maneuver->decision == ROUTE_DECISION_LEFT) ||
                    (maneuver->decision == ROUTE_DECISION_RIGHT)) {
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
            /*
             * 该阶段只负责直达旋转中心。优先使用固定 10 cm/s 的 IMU 航向保持，
             * 避免持续看到路口横线时套用普通巡线的多重降速和内侧轮反转。
             * IMU 暂时不可用时才退回红线巡线，保证仍有可恢复的前进路径。
             */
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
                    relative_turn =
                        (maneuver->decision == ROUTE_DECISION_LEFT) ?
                            MANEUVER_LEFT_TURN_TARGET_DEG :
                            MANEUVER_RIGHT_TURN_TARGET_DEG;
                    maneuver->target_heading_deg = Maneuver_NormalizeAngle(
                        input->heading_deg + relative_turn);
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
            /* 左转为左轮后退、右轮前进；右转方向相反。 */
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
                /* 找线失败也继续穿越，由 CROSS 的航向保持和距离兜底完成流程。 */
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
    maneuver->commit_requested = 0U;
    maneuver->state = DELIVERY_MANEUVER_STATE_IDLE;
    maneuver->decision = ROUTE_DECISION_NONE;
    maneuver->fault = DELIVERY_MANEUVER_FAULT_NONE;
}

uint8_t DeliveryManeuver_IsActive(const DeliveryManeuver *maneuver)
{
    if (maneuver == NULL) {
        return 0U;
    }
    return (maneuver->state != DELIVERY_MANEUVER_STATE_IDLE) ? 1U : 0U;
}
