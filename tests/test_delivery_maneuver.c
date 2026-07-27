#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "delivery_maneuver.h"

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__,    \
                    #condition);                                             \
            return 0;                                                        \
        }                                                                    \
    } while (0)

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int float_near(float actual, float expected, float tolerance)
{
    return abs_float(actual - expected) <= tolerance;
}

static DeliveryManeuver_Input default_input(void)
{
    DeliveryManeuver_Input input;

    memset(&input, 0, sizeof(input));
    input.line_fresh = 1U;
    input.line_new = 1U;
    input.line_valid = 1U;
    input.line_quality = 90U;
    input.direction_mask = K230_LINE_DIRECTION_FRONT;
    input.imu_fresh = 1U;
    return input;
}

static void update_at(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output,
    uint32_t now_ms)
{
    input->now_ms = now_ms;
    DeliveryManeuver_Update(maneuver, input, output);
}

static int complete_alignment(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output,
    float heading_deg)
{
    input->heading_deg = heading_deg;
    input->line_error_px = 0;
    input->line_angle_d10 = 0;
    input->line_new = 1U;
    DeliveryManeuver_Init(maneuver);
    CHECK(DeliveryManeuver_StartAlignment(maneuver, input) == 1U);

    update_at(maneuver, input, output, 0U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_ALIGN);
    CHECK(maneuver->alignment_stable_count == 1U);

    input->line_new = 1U;
    update_at(maneuver, input, output, 20U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_ALIGN);
    CHECK(maneuver->align_phase == DELIVERY_MANEUVER_ALIGN_READY);
    CHECK(DeliveryManeuver_IsAlignmentReady(maneuver) == 1U);
    CHECK(float_near(maneuver->approach_heading_deg, heading_deg, 0.01f));
    CHECK(output->command == DELIVERY_MANEUVER_COMMAND_STOP);
    return 1;
}

static int enter_turn_rotate(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output,
    RoutePlanner_Decision decision,
    float approach_heading)
{
    CHECK(complete_alignment(
              maneuver, input, output, approach_heading));
    CHECK(DeliveryManeuver_SetDecision(maneuver, decision) == 1U);

    input->line_new = 0U;
    update_at(maneuver, input, output, 40U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);

    input->left_position_cm = 50.2f;
    input->right_position_cm = 50.2f;
    update_at(maneuver, input, output, 60U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_TURN);
    CHECK(maneuver->turn_phase == DELIVERY_MANEUVER_TURN_SETTLE);

    /* SETTLE 中车头轻微变化不能改变基于校正道路航向计算的目标。 */
    input->heading_deg = approach_heading + 4.0f;
    update_at(maneuver, input, output, 210U);
    CHECK(maneuver->turn_phase == DELIVERY_MANEUVER_TURN_ROTATE);
    return 1;
}

static int enter_left_reacquire(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output)
{
    CHECK(enter_turn_rotate(
              maneuver, input, output, ROUTE_DECISION_LEFT, 0.0f));
    input->heading_deg = 86.0f;
    update_at(maneuver, input, output, 230U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_TURN);
    update_at(maneuver, input, output, 250U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_REACQUIRE);
    return 1;
}

static int test_alignment_large_angle_rotates_then_rechecks_line(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    input.line_angle_d10 = 100;
    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_StartAlignment(&maneuver, &input) == 1U);
    update_at(&maneuver, &input, &output, 0U);

    CHECK(maneuver.align_phase == DELIVERY_MANEUVER_ALIGN_SETTLE);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);
    CHECK(float_near(maneuver.target_heading_deg, -8.0f, 0.01f));

    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 120U);
    CHECK(maneuver.align_phase == DELIVERY_MANEUVER_ALIGN_ROTATE);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);

    update_at(&maneuver, &input, &output, 140U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s < 0.0f);

    input.heading_deg = -7.0f;
    update_at(&maneuver, &input, &output, 160U);
    CHECK(maneuver.align_phase == DELIVERY_MANEUVER_ALIGN_CREEP);
    CHECK(maneuver.alignment_ready == 0U);

    input.line_new = 1U;
    input.line_angle_d10 = 0;
    input.line_error_px = 0;
    update_at(&maneuver, &input, &output, 180U);
    CHECK(maneuver.alignment_stable_count == 1U);
    update_at(&maneuver, &input, &output, 200U);
    CHECK(maneuver.alignment_ready == 1U);
    return 1;
}

static int test_lateral_error_creeps_without_in_place_rotation(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;
    float first_target;

    input.line_error_px = 40;
    input.line_angle_d10 = 0;
    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_StartAlignment(&maneuver, &input) == 1U);
    update_at(&maneuver, &input, &output, 0U);

    CHECK(maneuver.align_phase == DELIVERY_MANEUVER_ALIGN_CREEP);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s > output.right_target_cm_s);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s > 0.0f);
    first_target = maneuver.target_heading_deg;

    /* 旧 L 帧只用于 IMU 保持，不得再次改变视觉目标。 */
    input.line_new = 0U;
    input.left_position_cm = 5.9f;
    input.right_position_cm = 5.9f;
    update_at(&maneuver, &input, &output, 20U);
    CHECK(float_near(maneuver.target_heading_deg, first_target, 0.001f));
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(float_near(output.left_target_cm_s + output.right_target_cm_s,
                     10.0f, 0.01f));

    /* 对正尚未稳定时也最多缓慢前进 6 cm，防止慢 YOLO 时驶离 T 字路口。 */
    input.left_position_cm = 6.1f;
    input.right_position_cm = 6.1f;
    update_at(&maneuver, &input, &output, 40U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);
    return 1;
}

static int test_alignment_requires_fresh_imu_and_two_new_frames(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_StartAlignment(&maneuver, &input) == 1U);

    input.imu_fresh = 0U;
    update_at(&maneuver, &input, &output, 0U);
    CHECK(maneuver.alignment_ready == 0U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s > 0.0f);

    input.imu_fresh = 1U;
    input.line_new = 1U;
    update_at(&maneuver, &input, &output, 20U);
    CHECK(maneuver.alignment_stable_count == 1U);
    input.line_error_px = 30;
    update_at(&maneuver, &input, &output, 40U);
    CHECK(maneuver.alignment_stable_count == 0U);
    input.line_error_px = 0;
    update_at(&maneuver, &input, &output, 60U);
    update_at(&maneuver, &input, &output, 80U);
    CHECK(maneuver.alignment_ready == 1U);
    return 1;
}

static int test_decision_and_alignment_can_finish_in_either_order(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_StartAlignment(&maneuver, &input) == 1U);
    CHECK(DeliveryManeuver_SetDecision(
              &maneuver, ROUTE_DECISION_LEFT) == 1U);
    CHECK(DeliveryManeuver_SetDecision(
              &maneuver, ROUTE_DECISION_LEFT) == 1U);
    CHECK(DeliveryManeuver_SetDecision(
              &maneuver, ROUTE_DECISION_RIGHT) == 0U);
    update_at(&maneuver, &input, &output, 0U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_ALIGN);
    update_at(&maneuver, &input, &output, 20U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);

    input = default_input();
    CHECK(complete_alignment(&maneuver, &input, &output, 15.0f));
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_ALIGN);
    CHECK(DeliveryManeuver_SetDecision(
              &maneuver, ROUTE_DECISION_FRONT) == 1U);
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 40U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);
    return 1;
}

static int test_alignment_stops_while_waiting_and_counts_toward_center(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(complete_alignment(&maneuver, &input, &output, 0.0f));
    input.line_new = 0U;
    input.left_position_cm = 3.0f;
    input.right_position_cm = 3.0f;
    update_at(&maneuver, &input, &output, 40U);

    /* 对正完成后即使尚未达到爬行距离上限，也必须停车等待新 D 帧。 */
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);
    CHECK(float_near(maneuver.junction_progress_cm, 3.0f, 0.01f));

    CHECK(DeliveryManeuver_SetDecision(
              &maneuver, ROUTE_DECISION_LEFT) == 1U);
    update_at(&maneuver, &input, &output, 60U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);

    /* 对正期间已经走过的 3 cm 仍计入用户设置的 45 cm 路口中心距离。 */
    input.left_position_cm = 44.8f;
    input.right_position_cm = 44.8f;
    update_at(&maneuver, &input, &output, 80U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);
    input.left_position_cm = 45.1f;
    input.right_position_cm = 45.1f;
    update_at(&maneuver, &input, &output, 100U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);
    return 1;
}

static int test_alignment_timeout_paths(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    input.line_error_px = 40;
    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_StartAlignment(&maneuver, &input) == 1U);
    update_at(&maneuver, &input, &output, 0U);
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 2000U);
    CHECK(maneuver.alignment_ready == 1U);
    CHECK(maneuver.align_phase == DELIVERY_MANEUVER_ALIGN_READY);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_NONE);

    input = default_input();
    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    input.imu_fresh = 0U;
    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_StartAlignment(&maneuver, &input) == 1U);
    update_at(&maneuver, &input, &output, 5999U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_ALIGN);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);
    update_at(&maneuver, &input, &output, 6000U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
    return 1;
}

static int test_left_turn_full_path(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_turn_rotate(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT, 0.0f));
    CHECK(float_near(maneuver.target_heading_deg, 90.0f, 0.01f));

    input.heading_deg = 0.0f;
    update_at(&maneuver, &input, &output, 230U);
    CHECK(output.left_target_cm_s == -18.0f);
    CHECK(output.right_target_cm_s == 18.0f);

    input.heading_deg = 75.0f;
    update_at(&maneuver, &input, &output, 250U);
    CHECK(output.left_target_cm_s == -14.0f);
    CHECK(output.right_target_cm_s == 14.0f);

    input.heading_deg = 86.0f;
    update_at(&maneuver, &input, &output, 270U);
    update_at(&maneuver, &input, &output, 290U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_REACQUIRE);

    input.line_fresh = 1U;
    input.line_new = 1U;
    input.line_valid = 1U;
    update_at(&maneuver, &input, &output, 310U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);

    input.left_position_cm += 12.5f;
    input.right_position_cm += 12.5f;
    input.direction_mask = K230_LINE_DIRECTION_FRONT |
                           K230_LINE_DIRECTION_LEFT;
    input.junction_active = 0U;
    update_at(&maneuver, &input, &output, 330U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);

    update_at(&maneuver, &input, &output, 630U);
    CHECK(output.request_commit == 1U);
    DeliveryManeuver_ReportCommit(&maneuver, 1U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_IDLE);
    CHECK(maneuver.alignment_ready == 0U);
    CHECK(DeliveryManeuver_IsActive(&maneuver) == 0U);
    return 1;
}

static int test_relative_heading_targets_wrap(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_turn_rotate(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT, 37.0f));
    CHECK(float_near(maneuver.target_heading_deg, 127.0f, 0.01f));

    input = default_input();
    CHECK(enter_turn_rotate(
              &maneuver, &input, &output, ROUTE_DECISION_RIGHT, -150.0f));
    CHECK(float_near(maneuver.target_heading_deg, 120.0f, 0.01f));

    input.heading_deg = -150.0f;
    update_at(&maneuver, &input, &output, 230U);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s < 0.0f);
    return 1;
}

static int test_turn_uses_average_wheel_travel_limit(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_turn_rotate(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT, 0.0f));
    input.heading_deg = 0.0f;

    input.left_position_cm += 31.0f;
    update_at(&maneuver, &input, &output, 230U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_NONE);

    input.right_position_cm += 30.0f;
    update_at(&maneuver, &input, &output, 250U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT);
    return 1;
}

static int test_reacquire_and_cross_fallbacks(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_left_reacquire(&maneuver, &input, &output));
    input.line_fresh = 0U;
    input.line_new = 0U;
    input.line_valid = 0U;
    update_at(&maneuver, &input, &output, 270U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s * output.right_target_cm_s < 0.0f);
    update_at(&maneuver, &input, &output, 2750U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);

    DeliveryManeuver_Init(&maneuver);
    maneuver.state = DELIVERY_MANEUVER_STATE_CROSS;
    maneuver.target_heading_deg = 90.0f;
    input = default_input();
    input.left_position_cm = 24.0f;
    input.right_position_cm = 24.0f;
    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 100U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);

    DeliveryManeuver_Init(&maneuver);
    maneuver.state = DELIVERY_MANEUVER_STATE_CROSS;
    input = default_input();
    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 5000U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
    return 1;
}

static int test_front_uses_corrected_heading(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(complete_alignment(&maneuver, &input, &output, 32.0f));
    CHECK(DeliveryManeuver_SetDecision(
              &maneuver, ROUTE_DECISION_FRONT) == 1U);
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 40U);
    input.left_position_cm = 50.2f;
    input.right_position_cm = 50.2f;
    input.heading_deg = 30.0f;
    update_at(&maneuver, &input, &output, 60U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);
    CHECK(float_near(maneuver.target_heading_deg, 32.0f, 0.01f));

    input.line_fresh = 0U;
    input.line_valid = 0U;
    update_at(&maneuver, &input, &output, 80U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s > 0.0f);
    return 1;
}

static int test_commit_failure_is_latched(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    DeliveryManeuver_Init(&maneuver);
    maneuver.state = DELIVERY_MANEUVER_STATE_BRAKE_AFTER;
    maneuver.state_start_ms = 0U;
    update_at(&maneuver, &input, &output, 300U);
    CHECK(output.request_commit == 1U);
    DeliveryManeuver_ReportCommit(&maneuver, 0U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_COMMIT);
    return 1;
}

int main(void)
{
    if (!test_alignment_large_angle_rotates_then_rechecks_line() ||
        !test_lateral_error_creeps_without_in_place_rotation() ||
        !test_alignment_requires_fresh_imu_and_two_new_frames() ||
        !test_decision_and_alignment_can_finish_in_either_order() ||
        !test_alignment_stops_while_waiting_and_counts_toward_center() ||
        !test_alignment_timeout_paths() ||
        !test_left_turn_full_path() ||
        !test_relative_heading_targets_wrap() ||
        !test_turn_uses_average_wheel_travel_limit() ||
        !test_reacquire_and_cross_fallbacks() ||
        !test_front_uses_corrected_heading() ||
        !test_commit_failure_is_latched()) {
        return 1;
    }

    puts("delivery_maneuver tests passed");
    return 0;
}
