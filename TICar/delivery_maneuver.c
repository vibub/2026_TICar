/**
 * @file delivery_maneuver.c
 * @brief 送药路口动作状态机的纯逻辑实现。
 *
 * 电机、编码器、IMU 和 K230 均由 app_main.c 适配后通过输入输出结构交互。
 * 这样状态转换可以在主机环境验证，且不会在状态机内部产生阻塞等待。
 */
#include "delivery_maneuver.h"

#include <string.h>

#define MANEUVER_BRAKE_MS 300U
#define MANEUVER_APPROACH_CENTER_CM 8.0f
#define MANEUVER_APPROACH_MAX_CM 16.0f
#define MANEUVER_APPROACH_TIMEOUT_MS 2500U
#define MANEUVER_LINE_BAD_GRACE_MS 200U
#define MANEUVER_LINE_MAX_AGE_MS 400U
#define MANEUVER_TURN_SETTLE_MS 150U
#define MANEUVER_TURN_TIMEOUT_MS 3000U
#define MANEUVER_TURN_MAX_WHEEL_CM 12.0f
#define MANEUVER_TURN_COARSE_ERROR_DEG 20.0f
#define MANEUVER_TURN_FINE_ERROR_DEG 4.0f
#define MANEUVER_TURN_STABLE_COUNT 3U
#define MANEUVER_TURN_COARSE_SPEED_CM_S 18.0f
#define MANEUVER_TURN_FINE_SPEED_CM_S 10.0f
#define MANEUVER_REACQUIRE_TIMEOUT_MS 1500U
#define MANEUVER_REACQUIRE_MAX_WHEEL_CM 6.0f
#define MANEUVER_REACQUIRE_SPEED_CM_S 8.0f
#define MANEUVER_REACQUIRE_SWEEP_SPEED_CM_S 5.0f
#define MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG 10.0f
#define MANEUVER_REACQUIRE_ERROR_PX 10
#define MANEUVER_REACQUIRE_ANGLE_D10 60
#define MANEUVER_REACQUIRE_STABLE_COUNT 3U
#define MANEUVER_CROSS_MIN_CM 16.0f
#define MANEUVER_CROSS_MAX_CM 35.0f
#define MANEUVER_CROSS_TIMEOUT_MS 3500U
#define MANEUVER_CROSS_STABLE_COUNT 3U
#define MANEUVER_LINE_CENTER_ERROR_PX 14
#define MANEUVER_LINE_CENTER_ANGLE_D10 100
#define MANEUVER_FOLLOW_SCALE 0.80f
#define MANEUVER_REACQUIRE_CORRECTION_LIMIT_CM_S 5.0f

static float Maneuver_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Maneuver_Max(float left, float right)
{
    return (left > right) ? left : right;
}

static int16_t Maneuver_AbsInt16(int16_t value)
{
    return (value < 0) ? (int16_t) (-value) : value;
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

static float Maneuver_MaxWheelDistance(
    const DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    return Maneuver_Max(
        Maneuver_Abs(input->left_position_cm - maneuver->state_start_left_cm),
        Maneuver_Abs(input->right_position_cm - maneuver->state_start_right_cm));
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

static void Maneuver_EnterState(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_State state,
    const DeliveryManeuver_Input *input)
{
    maneuver->state = state;
    maneuver->state_start_ms = input->now_ms;
    maneuver->state_start_left_cm = input->left_position_cm;
    maneuver->state_start_right_cm = input->right_position_cm;
    maneuver->line_bad_active = 0U;
    maneuver->line_stable_count = 0U;
    maneuver->junction_clear_count = 0U;
}

static void Maneuver_SetFault(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Fault fault)
{
    maneuver->state = DELIVERY_MANEUVER_STATE_FAULT;
    maneuver->fault = fault;
}

static uint8_t Maneuver_LineTransportUsable(
    const DeliveryManeuver_Input *input)
{
    return ((input->line_fresh != 0U) &&
            (input->line_age_ms <= MANEUVER_LINE_MAX_AGE_MS)) ? 1U : 0U;
}

static uint8_t Maneuver_LineUsable(
    const DeliveryManeuver_Input *input)
{
    return (Maneuver_LineTransportUsable(input) != 0U &&
            input->line_valid != 0U) ? 1U : 0U;
}

static uint8_t Maneuver_HandleLineLoss(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input)
{
    if (Maneuver_LineUsable(input) != 0U) {
        maneuver->line_bad_active = 0U;
        return 1U;
    }

    if (maneuver->line_bad_active == 0U) {
        maneuver->line_bad_active = 1U;
        maneuver->line_bad_start_ms = input->now_ms;
    }
    if (Maneuver_LineTransportUsable(input) == 0U ||
        Maneuver_Elapsed(
            input->now_ms,
            maneuver->line_bad_start_ms,
            MANEUVER_LINE_BAD_GRACE_MS) != 0U) {
        Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_LINE_TIMEOUT);
    }
    return 0U;
}

static void Maneuver_SetReacquireOutput(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output)
{
    float error;
    float correction;
    float desired_heading;
    float heading_error;
    float turn_sign;

    if (input->line_valid != 0U) {
        error = (float) input->line_error_px;
        correction = error * 0.12f;
        if (Maneuver_Abs(error) < 5.0f) {
            correction = ((float) input->line_angle_d10 / 10.0f) * 0.50f;
        }
        if (correction > MANEUVER_REACQUIRE_CORRECTION_LIMIT_CM_S) {
            correction = MANEUVER_REACQUIRE_CORRECTION_LIMIT_CM_S;
        } else if (correction < -MANEUVER_REACQUIRE_CORRECTION_LIMIT_CM_S) {
            correction = -MANEUVER_REACQUIRE_CORRECTION_LIMIT_CM_S;
        }
        Maneuver_SetOutputWheelSpeed(
            output,
            MANEUVER_REACQUIRE_SPEED_CM_S + correction,
            MANEUVER_REACQUIRE_SPEED_CM_S - correction);
        return;
    }

    /* 没有红线时在目标航向两侧往返低速扫线，避免无边界地持续旋转。 */
    desired_heading = maneuver->target_heading_deg +
        (float) maneuver->reacquire_sweep_direction *
        MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG;
    heading_error = Maneuver_NormalizeAngle(
        desired_heading - input->heading_deg);
    if (Maneuver_Abs(heading_error) <= 2.0f) {
        maneuver->reacquire_sweep_direction =
            (int8_t) (-maneuver->reacquire_sweep_direction);
        desired_heading = maneuver->target_heading_deg +
            (float) maneuver->reacquire_sweep_direction *
            MANEUVER_REACQUIRE_SWEEP_ANGLE_DEG;
        heading_error = Maneuver_NormalizeAngle(
            desired_heading - input->heading_deg);
    }
    turn_sign = (heading_error > 0.0f) ? 1.0f : -1.0f;
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
    maneuver->last_now_ms = input->now_ms;
    maneuver->state_start_left_cm = input->left_position_cm;
    maneuver->state_start_right_cm = input->right_position_cm;
    maneuver->target_heading_deg =
        (decision == ROUTE_DECISION_LEFT) ? 90.0f : -90.0f;
    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_SETTLE;
    maneuver->state = DELIVERY_MANEUVER_STATE_BRAKE;
    maneuver->commit_requested = 0U;
    maneuver->line_bad_active = 0U;
    maneuver->line_stable_count = 0U;
    maneuver->junction_clear_count = 0U;
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
    float center_distance;
    float wheel_distance;
    float heading_error;
    float turn_sign;
    float turn_speed;
    uint8_t centered;

    if (output == NULL) {
        return;
    }
    Maneuver_SetOutputStop(output);
    output->request_yaw_zero = 0U;
    output->request_commit = 0U;

    if ((maneuver == NULL) || (input == NULL)) {
        return;
    }
    maneuver->last_now_ms = input->now_ms;

    if (maneuver->state == DELIVERY_MANEUVER_STATE_IDLE) {
        return;
    }
    if (maneuver->state == DELIVERY_MANEUVER_STATE_FAULT) {
        return;
    }
    if ((input->speed_faults != 0U) &&
        ((maneuver->state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER) ||
         ((maneuver->state == DELIVERY_MANEUVER_STATE_TURN) &&
          (maneuver->turn_phase == DELIVERY_MANEUVER_TURN_ROTATE)) ||
         (maneuver->state == DELIVERY_MANEUVER_STATE_REACQUIRE) ||
         (maneuver->state == DELIVERY_MANEUVER_STATE_CROSS))) {
        Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_SPEED);
        return;
    }

    center_distance = Maneuver_CenterDistance(maneuver, input);
    wheel_distance = Maneuver_MaxWheelDistance(maneuver, input);

    switch (maneuver->state) {
        case DELIVERY_MANEUVER_STATE_BRAKE:
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_BRAKE_MS) != 0U) {
                Maneuver_EnterState(
                    maneuver,
                    DELIVERY_MANEUVER_STATE_APPROACH_CENTER,
                    input);
            }
            break;

        case DELIVERY_MANEUVER_STATE_APPROACH_CENTER:
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_APPROACH_TIMEOUT_MS) != 0U ||
                wheel_distance > MANEUVER_APPROACH_MAX_CM) {
                Maneuver_SetFault(
                    maneuver,
                    (wheel_distance > MANEUVER_APPROACH_MAX_CM) ?
                        DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT :
                        DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }
            if (Maneuver_HandleLineLoss(maneuver, input) == 0U) {
                break;
            }
            if (center_distance >= MANEUVER_APPROACH_CENTER_CM) {
                if ((maneuver->decision == ROUTE_DECISION_LEFT) ||
                    (maneuver->decision == ROUTE_DECISION_RIGHT)) {
                    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_SETTLE;
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_TURN,
                        input);
                } else {
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_CROSS,
                        input);
                }
                break;
            }
            Maneuver_SetOutputFollow(output);
            break;

        case DELIVERY_MANEUVER_STATE_TURN:
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_TURN_TIMEOUT_MS) != 0U ||
                wheel_distance > MANEUVER_TURN_MAX_WHEEL_CM) {
                Maneuver_SetFault(
                    maneuver,
                    (wheel_distance > MANEUVER_TURN_MAX_WHEEL_CM) ?
                        DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT :
                        DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }
            if ((maneuver->turn_phase != DELIVERY_MANEUVER_TURN_SETTLE) &&
                (input->imu_fresh == 0U)) {
                Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_IMU);
                break;
            }
            if (maneuver->turn_phase == DELIVERY_MANEUVER_TURN_SETTLE) {
                if (Maneuver_Elapsed(
                        input->now_ms,
                        maneuver->state_start_ms,
                        MANEUVER_TURN_SETTLE_MS) != 0U) {
                    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_WAIT_ZERO;
                                }
                break;
            }
            if (maneuver->turn_phase == DELIVERY_MANEUVER_TURN_WAIT_ZERO) {
                output->request_yaw_zero = 1U;
                break;
            }

            heading_error = Maneuver_NormalizeAngle(
                maneuver->target_heading_deg - input->heading_deg);
            if (Maneuver_Abs(heading_error) <= MANEUVER_TURN_FINE_ERROR_DEG) {
                if (maneuver->heading_stable_count < 0xFFU) {
                    maneuver->heading_stable_count++;
                }
                if (maneuver->heading_stable_count >= MANEUVER_TURN_STABLE_COUNT) {
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_REACQUIRE,
                        input);
                    break;
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
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_REACQUIRE_TIMEOUT_MS) != 0U ||
                wheel_distance > MANEUVER_REACQUIRE_MAX_WHEEL_CM) {
                Maneuver_SetFault(
                    maneuver,
                    (wheel_distance > MANEUVER_REACQUIRE_MAX_WHEEL_CM) ?
                        DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT :
                        DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }
            if (input->imu_fresh == 0U) {
                Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_IMU);
                break;
            }
            if (Maneuver_LineTransportUsable(input) == 0U) {
                Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_LINE_TIMEOUT);
                break;
            }
            centered = (input->line_valid != 0U) &&
                       (Maneuver_AbsInt16(input->line_error_px) <=
                        MANEUVER_REACQUIRE_ERROR_PX) &&
                       (Maneuver_AbsInt16(input->line_angle_d10) <=
                        MANEUVER_REACQUIRE_ANGLE_D10) ? 1U : 0U;
            if ((input->line_new != 0U) &&
                (input->line_valid != 0U) &&
                (input->line_quality >= K230_LINE_MIN_QUALITY) &&
                (centered != 0U)) {
                if (maneuver->line_stable_count < 0xFFU) {
                    maneuver->line_stable_count++;
                }
                if (maneuver->line_stable_count >=
                    MANEUVER_REACQUIRE_STABLE_COUNT) {
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_CROSS,
                        input);
                    break;
                }
            } else if (input->line_new != 0U) {
                maneuver->line_stable_count = 0U;
            }
            Maneuver_SetReacquireOutput(maneuver, input, output);
            break;

        case DELIVERY_MANEUVER_STATE_CROSS:
            if (Maneuver_Elapsed(
                    input->now_ms,
                    maneuver->state_start_ms,
                    MANEUVER_CROSS_TIMEOUT_MS) != 0U ||
                center_distance > MANEUVER_CROSS_MAX_CM) {
                Maneuver_SetFault(
                    maneuver,
                    (center_distance > MANEUVER_CROSS_MAX_CM) ?
                        DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT :
                        DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
                break;
            }
            if (Maneuver_HandleLineLoss(maneuver, input) == 0U) {
                break;
            }
            centered = (Maneuver_AbsInt16(input->line_error_px) <=
                        MANEUVER_LINE_CENTER_ERROR_PX) &&
                       (Maneuver_AbsInt16(input->line_angle_d10) <=
                        MANEUVER_LINE_CENTER_ANGLE_D10) ? 1U : 0U;
            if ((center_distance >= MANEUVER_CROSS_MIN_CM) &&
                (input->line_new != 0U) &&
                (input->direction_mask == K230_LINE_DIRECTION_FRONT) &&
                (input->junction_active == 0U) &&
                (centered != 0U)) {
                if (maneuver->junction_clear_count < 0xFFU) {
                    maneuver->junction_clear_count++;
                }
                if (maneuver->junction_clear_count >=
                    MANEUVER_CROSS_STABLE_COUNT) {
                    Maneuver_EnterState(
                        maneuver,
                        DELIVERY_MANEUVER_STATE_BRAKE_AFTER,
                        input);
                    break;
                }
            } else if (input->line_new != 0U) {
                maneuver->junction_clear_count = 0U;
            }
            Maneuver_SetOutputFollow(output);
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

void DeliveryManeuver_ReportYawZero(
    DeliveryManeuver *maneuver,
    uint8_t success)
{
    if ((maneuver == NULL) ||
        (maneuver->state != DELIVERY_MANEUVER_STATE_TURN) ||
        (maneuver->turn_phase != DELIVERY_MANEUVER_TURN_WAIT_ZERO)) {
        return;
    }
    if (success == 0U) {
        Maneuver_SetFault(maneuver, DELIVERY_MANEUVER_FAULT_IMU);
        return;
    }
    maneuver->turn_phase = DELIVERY_MANEUVER_TURN_ROTATE;
    maneuver->heading_stable_count = 0U;
    maneuver->state_start_ms = maneuver->last_now_ms;
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
