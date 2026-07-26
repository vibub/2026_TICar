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

static int enter_turn_rotate(
    DeliveryManeuver *maneuver,
    DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output,
    RoutePlanner_Decision decision,
    float turn_start_heading)
{
    DeliveryManeuver_Init(maneuver);
    CHECK(DeliveryManeuver_Start(maneuver, decision, input) == 1U);

    update_at(maneuver, input, output, 300U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);

    input->left_position_cm = 30.2f;
    input->right_position_cm = 30.2f;
    update_at(maneuver, input, output, 320U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_TURN);
    CHECK(maneuver->turn_phase == DELIVERY_MANEUVER_TURN_SETTLE);

    input->heading_deg = turn_start_heading;
    update_at(maneuver, input, output, 470U);
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
    update_at(maneuver, input, output, 490U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_TURN);
    update_at(maneuver, input, output, 510U);
    CHECK(maneuver->state == DELIVERY_MANEUVER_STATE_REACQUIRE);
    return 1;
}

static int test_left_turn_full_path(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_turn_rotate(
              &maneuver, &input, &output, ROUTE_DECISION_LEFT, 0.0f));
    CHECK(maneuver.target_heading_deg == 90.0f);

    input.heading_deg = 0.0f;
    update_at(&maneuver, &input, &output, 490U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s == -18.0f);
    CHECK(output.right_target_cm_s == 18.0f);

    input.heading_deg = 75.0f;
    update_at(&maneuver, &input, &output, 510U);
    CHECK(output.left_target_cm_s == -14.0f);
    CHECK(output.right_target_cm_s == 14.0f);

    /* 正常完成只依赖连续稳定的 IMU 角度，不再要求编码器先达到 6 cm。 */
    input.heading_deg = 86.0f;
    update_at(&maneuver, &input, &output, 530U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);
    update_at(&maneuver, &input, &output, 550U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_REACQUIRE);

    /* 第一帧新且有效的红线即可交给 CROSS 继续纠偏。 */
    input.line_fresh = 1U;
    input.line_new = 1U;
    input.line_valid = 1U;
    input.line_error_px = 80;
    input.line_angle_d10 = 500;
    update_at(&maneuver, &input, &output, 570U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);

    /* FRONT 与其他方向位同时存在时也可以确认已经穿出路口。 */
    input.left_position_cm += 12.5f;
    input.right_position_cm += 12.5f;
    input.direction_mask = K230_LINE_DIRECTION_FRONT |
                           K230_LINE_DIRECTION_LEFT;
    input.junction_active = 0U;
    update_at(&maneuver, &input, &output, 590U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);

    update_at(&maneuver, &input, &output, 890U);
    CHECK(output.request_commit == 1U);
    DeliveryManeuver_ReportCommit(&maneuver, 1U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_IDLE);
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
    CHECK(maneuver.target_heading_deg == 127.0f);

    input = default_input();
    CHECK(enter_turn_rotate(
              &maneuver, &input, &output, ROUTE_DECISION_RIGHT, -150.0f));
    CHECK(maneuver.target_heading_deg == 120.0f);

    input.heading_deg = -150.0f;
    update_at(&maneuver, &input, &output, 490U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
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

    /* 单轮跳变 31 cm、另一轮不动时平均值只有 15.5 cm，不能提前故障。 */
    input.left_position_cm += 31.0f;
    update_at(&maneuver, &input, &output, 490U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_NONE);

    /* 两轮平均绝对位移超过 30 cm 后才触发宽松的最终安全保护。 */
    input.right_position_cm += 30.0f;
    update_at(&maneuver, &input, &output, 510U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT);
    return 1;
}

static int test_imu_wait_and_recovery(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_Start(
              &maneuver, ROUTE_DECISION_LEFT, &input) == 1U);
    update_at(&maneuver, &input, &output, 300U);
    input.left_position_cm = 30.2f;
    input.right_position_cm = 30.2f;
    update_at(&maneuver, &input, &output, 320U);

    /* SETTLE 到期但 IMU 不新鲜时保持停车，不启动旋转超时。 */
    input.imu_fresh = 0U;
    update_at(&maneuver, &input, &output, 470U);
    CHECK(maneuver.turn_phase == DELIVERY_MANEUVER_TURN_SETTLE);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);
    update_at(&maneuver, &input, &output, 2000U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);

    input.imu_fresh = 1U;
    input.heading_deg = 0.0f;
    update_at(&maneuver, &input, &output, 2020U);
    CHECK(maneuver.turn_phase == DELIVERY_MANEUVER_TURN_ROTATE);

    /* ROTATE 中短时丢失 IMU 只停车，恢复后继续原方向转向。 */
    input.imu_fresh = 0U;
    update_at(&maneuver, &input, &output, 2040U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_NONE);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);

    input.imu_fresh = 1U;
    update_at(&maneuver, &input, &output, 2060U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);

    input.imu_fresh = 0U;
    update_at(&maneuver, &input, &output, 10020U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
    return 1;
}

static int test_approach_uses_heading_with_or_without_line(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;
    float left_with_line;
    float right_with_line;

    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_Start(
              &maneuver, ROUTE_DECISION_LEFT, &input) == 1U);
    update_at(&maneuver, &input, &output, 300U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);

    /* 红线持续有效时也必须使用固定速度航向保持，不能套用路口巡线降速。 */
    input.heading_deg = -5.0f;
    update_at(&maneuver, &input, &output, 320U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s < output.right_target_cm_s);
    CHECK(output.left_target_cm_s > 0.0f);
    left_with_line = output.left_target_cm_s;
    right_with_line = output.right_target_cm_s;

    /* 红线丢失后保持同一套前进控制，不能因为视觉状态改变行驶速度。 */
    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 340U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s == left_with_line);
    CHECK(output.right_target_cm_s == right_with_line);

    input.left_position_cm = 30.2f;
    input.right_position_cm = 30.2f;
    update_at(&maneuver, &input, &output, 360U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_TURN);
    return 1;
}

static int test_approach_without_line_or_imu_times_out(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_Start(
              &maneuver, ROUTE_DECISION_LEFT, &input) == 1U);
    update_at(&maneuver, &input, &output, 300U);

    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    input.imu_fresh = 0U;
    update_at(&maneuver, &input, &output, 320U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_APPROACH_CENTER);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_STOP);

    update_at(&maneuver, &input, &output, 6300U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_FAULT);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT);
    return 1;
}

static int test_reacquire_falls_through_without_line(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    CHECK(enter_left_reacquire(&maneuver, &input, &output));
    input.line_fresh = 0U;
    input.line_new = 0U;
    input.line_valid = 0U;
    update_at(&maneuver, &input, &output, 530U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s * output.right_target_cm_s < 0.0f);

    update_at(&maneuver, &input, &output, 3010U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);
    CHECK(maneuver.fault == DELIVERY_MANEUVER_FAULT_NONE);
    return 1;
}

static int test_cross_completion_fallbacks(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    DeliveryManeuver_Init(&maneuver);
    maneuver.state = DELIVERY_MANEUVER_STATE_CROSS;
    maneuver.target_heading_deg = 90.0f;
    input.left_position_cm = 24.0f;
    input.right_position_cm = 24.0f;
    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 100U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);

    DeliveryManeuver_Init(&maneuver);
    maneuver.state = DELIVERY_MANEUVER_STATE_CROSS;
    maneuver.target_heading_deg = 90.0f;
    input = default_input();
    input.left_position_cm = 12.1f;
    input.right_position_cm = 12.1f;
    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    update_at(&maneuver, &input, &output, 5000U);
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

static int test_front_crosses_with_heading_fallback(void)
{
    DeliveryManeuver maneuver;
    DeliveryManeuver_Input input = default_input();
    DeliveryManeuver_Output output;

    input.heading_deg = 32.0f;
    DeliveryManeuver_Init(&maneuver);
    CHECK(DeliveryManeuver_Start(
              &maneuver, ROUTE_DECISION_FRONT, &input) == 1U);
    update_at(&maneuver, &input, &output, 300U);
    input.left_position_cm = 30.2f;
    input.right_position_cm = 30.2f;
    update_at(&maneuver, &input, &output, 320U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_CROSS);
    CHECK(maneuver.target_heading_deg == 32.0f);

    input.line_fresh = 0U;
    input.line_valid = 0U;
    input.line_new = 0U;
    input.heading_deg = 30.0f;
    update_at(&maneuver, &input, &output, 340U);
    CHECK(output.command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s > 0.0f);

    input.left_position_cm += 24.1f;
    input.right_position_cm += 24.1f;
    update_at(&maneuver, &input, &output, 360U);
    CHECK(maneuver.state == DELIVERY_MANEUVER_STATE_BRAKE_AFTER);
    update_at(&maneuver, &input, &output, 660U);
    CHECK(output.request_commit == 1U);
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
        !test_relative_heading_targets_wrap() ||
        !test_turn_uses_average_wheel_travel_limit() ||
        !test_imu_wait_and_recovery() ||
        !test_approach_uses_heading_with_or_without_line() ||
        !test_approach_without_line_or_imu_times_out() ||
        !test_reacquire_falls_through_without_line() ||
        !test_cross_completion_fallbacks() ||
        !test_front_crosses_with_heading_fallback() ||
        !test_commit_failure_is_latched()) {
        return 1;
    }

    puts("delivery_maneuver tests passed");
    return 0;
}
