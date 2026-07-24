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

static DeliveryManeuver_Input default_input(void)
{
    DeliveryManeuver_Input input;

    memset(&input, 0, sizeof(input));
    input.line_fresh = 1U;
    input.line_new = 1U;
    input.line_valid = 1U;
    input.line_age_ms = 0U;
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

static int enter_turn_wait_zero(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output,
    RoutePlanner_Decision decision)
{
    DeliveryManeuver_Init(maneuver);
    CHECK(DeliveryManeuver_Start(maneuver, decision, input) == 1U);

    update_at(maneuver, input, output, 300U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);

    input->left_position_cm = 8.2f;
    input->right_position_cm = 8.2f;
    update_at(maneuver, input, output, 320U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_TURN);
    CHECK(maneuver->turn_phase == DELIVERY_MANEUVER_TURN_SETTLE);

    update_at(maneuver, input, output, 470U);
    CHECK(maneuver->turn_phase == DELIVERY_MANEUVER_TURN_WAIT_ZERO);
    update_at(maneuver, input, output, 490U);
    CHECK(output->request_yaw_zero == 1U);
    return 1;
}

static int test_left_turn_full_path(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;
    uint32_t now_ms;

    CHECK(enter_turn_wait_zero(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT));
    DeliveryManeuver_ReportYawZero(&maneuver, 1U);
    CHECK(maneuver.turn_phase == DELIVERY_MANEUVER_TURN_ROTATE);

    input.heading_deg = 0.0f;
    update_at(&maneuver, &input, &output, 510U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s < 0.0f);
    CHECK(output.right_target_cm_s > 0.0f);

    input.heading_deg = 75.0f;
    update_at(&maneuver, &input, &output, 530U);
    CHECK(output.left_target_cm_s == -10.0f);
    CHECK(output.right_target_cm_s == 10.0f);

    input.heading_deg = 88.0f;
    update_at(&maneuver, &input, &output, 550U);
    update_at(&maneuver, &input, &output, 570U);
    update_at(&maneuver, &input, &output, 590U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_REACQUIRE);

    input.line_error_px = 0;
    input.line_angle_d10 = 0;
    input.line_new = 1U;
    for (now_ms = 610U; now_ms <= 650U; now_ms += 20U) {
        update_at(&maneuver, &input, &output, now_ms);
    }
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);

    input.junction_active = 0U;
    input.direction_mask = K230_LINE_DIRECTION_FRONT;
    input.left_position_cm += 16.5f;
    input.right_position_cm += 16.5f;
    for (now_ms = 670U; now_ms <= 710U; now_ms += 20U) {
        update_at(&maneuver, &input, &output, now_ms);
    }
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);

    update_at(&maneuver, &input, &output, 1010U);
    CHECK(output.request_commit == 1U);
    DeliveryManeuver_ReportCommit(&maneuver, 1U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_IDLE);
    CHECK(DeliveryManeuver_IsActive(&maneuver) == 0U);
    return 1;
}

static int test_right_turn_uses_mirrored_wheel_targets(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_turn_wait_zero(
              &maneuver, &input, &output, ROUTE_DECISION_RIGHT));
    DeliveryManeuver_ReportYawZero(&maneuver, 1U);

    input.heading_deg = 0.0f;
    update_at(&maneuver, &input, &output, 510U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s < 0.0f);
    CHECK(maneuver.target_heading_deg == -90.0f);
    return 1;
}

static int test_front_crosses_before_commit(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;
    uint32_t now_ms;

    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_Start(
              &maneuver, ROUTE_DECISION_FRONT, &input) == 1U);
    update_at(&maneuver, &input, &output, 300U);
    input.left_position_cm = 8.2f;
    input.right_position_cm = 8.2f;
    update_at(&maneuver, &input, &output, 320U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);
    CHECK(output.request_yaw_zero == 0U);

    input.left_position_cm += 10.0f;
    input.right_position_cm += 10.0f;
    update_at(&maneuver, &input, &output, 340U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);
    CHECK(output.request_commit == 0U);

    input.left_position_cm += 6.5f;
    input.right_position_cm += 6.5f;
    input.junction_active = 0U;
    for (now_ms = 360U; now_ms <= 400U; now_ms += 20U) {
        update_at(&maneuver, &input, &output, now_ms);
    }
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);
    update_at(&maneuver, &input, &output, 700U);
    CHECK(output.request_commit == 1U);
    return 1;
}

static int test_reacquire_requires_new_frames(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;
    uint32_t now_ms;

    CHECK(enter_turn_wait_zero(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT));
    DeliveryManeuver_ReportYawZero(&maneuver, 1U);
    input.heading_deg = 90.0f;
    update_at(&maneuver, &input, &output, 510U);
    update_at(&maneuver, &input, &output, 530U);
    update_at(&maneuver, &input, &output, 550U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_REACQUIRE);

    input.line_new = 0U;
    for (now_ms = 570U; now_ms <= 650U; now_ms += 20U) {
        update_at(&maneuver, &input, &output, now_ms);
    }
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_REACQUIRE);
    CHECK(maneuver.line_stable_count == 0U);

    input.line_new = 1U;
    update_at(&maneuver, &input, &output, 670U);
    update_at(&maneuver, &input, &output, 690U);
    update_at(&maneuver, &input, &output, 710U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);
    return 1;
}

static int test_faults_stop_the_maneuver(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_turn_wait_zero(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT));
    input.imu_fresh = 0U;
    update_at(&maneuver, &input, &output, 510U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_IMU);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);

    DeliveryManeuver_Reset(&maneuver);
    input = default_input();
    CHECK(DeliveryManeuver_Start(
              &maneuver, ROUTE_DECISION_FRONT, &input) == 1U);
    update_at(&maneuver, &input, &output, 300U);
    input.line_fresh = 0U;
    input.line_age_ms = 500U;
    update_at(&maneuver, &input, &output, 320U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_LINE_TIMEOUT);
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
    if (!test_left_turn_full_path() ||
        !test_right_turn_uses_mirrored_wheel_targets() ||
        !test_front_crosses_before_commit() ||
        !test_reacquire_requires_new_frames() ||
        !test_faults_stop_the_maneuver() ||
        !test_commit_failure_is_latched()) {
        return 1;
    }

    puts("delivery_maneuver tests passed");
    return 0;
}
