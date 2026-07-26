#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "line_heading_control.h"

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

static int float_near(float left, float right, float tolerance)
{
    return abs_float(left - right) <= tolerance;
}

static LineHeadingControl_Input default_input(void)
{
    LineHeadingControl_Input input;

    memset(&input, 0, sizeof(input));
    input.imu_fresh = 1U;
    input.line_new = 1U;
    input.line_valid = 1U;
    input.base_speed_cm_s = 15.0f;
    return input;
}

static int test_centered_line_uses_imu_straight(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;

    LineHeadingControl_Init(&control);
    input.heading_deg = 23.0f;
    LineHeadingControl_Update(&control, &input, &output);

    CHECK(output.command_valid == 1U);
    CHECK(output.visual_correction_active == 0U);
    CHECK(float_near(output.route_heading_deg, 23.0f, 0.01f));
    CHECK(float_near(output.target_heading_deg, 23.0f, 0.01f));
    CHECK(float_near(output.left_target_cm_s, 15.0f, 0.01f));
    CHECK(float_near(output.right_target_cm_s, 15.0f, 0.01f));
    return 1;
}

static int test_right_line_error_changes_heading_not_directly_wheels(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;
    float first_offset;

    LineHeadingControl_Init(&control);
    input.line_error_px = 40;
    LineHeadingControl_Update(&control, &input, &output);

    /* 红线在右侧时目标航向向右偏移，IMU 内环输出左快右慢。 */
    CHECK(output.visual_correction_active == 1U);
    CHECK(output.visual_offset_deg < 0.0f);
    CHECK(output.target_heading_deg < output.route_heading_deg);
    CHECK(output.left_target_cm_s > output.right_target_cm_s);
    CHECK(output.left_target_cm_s > 0.0f);
    CHECK(output.right_target_cm_s > 0.0f);
    first_offset = output.visual_offset_deg;

    /* 两帧之间不重复放大旧像素误差，IMU 只持续追踪已生成的目标航向。 */
    input.now_ms = 20U;
    input.line_age_ms = 20U;
    input.line_new = 0U;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(float_near(output.visual_offset_deg, first_offset, 0.01f));
    CHECK(output.left_target_cm_s > output.right_target_cm_s);
    return 1;
}

static int test_visual_offset_is_limited_and_slewed(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;
    float previous_offset;
    uint32_t index;

    LineHeadingControl_Init(&control);
    input.line_error_px = 160;
    input.line_angle_d10 = 900;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(float_near(output.visual_offset_deg, -2.5f, 0.01f));
    previous_offset = output.visual_offset_deg;

    for (index = 1U; index <= 10U; index++) {
        input.now_ms = index * 100U;
        input.line_age_ms = 0U;
        input.line_new = 1U;
        LineHeadingControl_Update(&control, &input, &output);
        CHECK(abs_float(output.visual_offset_deg - previous_offset) <= 2.51f);
        CHECK(abs_float(output.visual_offset_deg) <= 10.01f);
        previous_offset = output.visual_offset_deg;
    }
    CHECK(float_near(output.visual_offset_deg, -10.0f, 0.01f));
    return 1;
}

static int test_stale_line_decays_offset_and_bridges_with_imu(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;
    float active_offset;

    LineHeadingControl_Init(&control);
    input.line_error_px = -40;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(output.visual_offset_deg > 0.0f);
    active_offset = output.visual_offset_deg;

    input.now_ms = 300U;
    input.line_age_ms = 300U;
    input.line_new = 0U;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(output.command_valid == 1U);
    CHECK(output.blind_timeout == 0U);
    CHECK(output.visual_offset_deg < active_offset);
    CHECK(output.left_target_cm_s < 15.0f);
    CHECK(output.right_target_cm_s < 15.0f);
    return 1;
}

static int test_invalid_line_stops_only_after_bridge_timeout(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;

    LineHeadingControl_Init(&control);
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(output.command_valid == 1U);

    input.line_new = 1U;
    input.line_valid = 0U;
    input.now_ms = 1000U;
    input.line_age_ms = 0U;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(output.command_valid == 1U);
    CHECK(output.blind_timeout == 0U);

    input.now_ms = 1200U;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(output.command_valid == 0U);
    CHECK(output.blind_timeout == 1U);
    return 1;
}

static int test_junction_freezes_visual_correction(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;

    LineHeadingControl_Init(&control);
    input.line_error_px = 40;
    LineHeadingControl_Update(&control, &input, &output);
    CHECK(output.visual_offset_deg < 0.0f);

    input.now_ms = 20U;
    input.line_new = 1U;
    input.junction_active = 1U;
    input.line_error_px = -160;
    input.line_angle_d10 = -900;
    LineHeadingControl_Update(&control, &input, &output);

    /* 路口横线不能重新决定普通直线路段航向，原偏移只允许向 0 衰减。 */
    CHECK(output.visual_correction_active == 0U);
    CHECK(output.visual_offset_deg <= 0.0f);
    CHECK(abs_float(output.visual_offset_deg) < 2.5f);
    return 1;
}

static int test_heading_wrap_uses_shortest_direction(void)
{
    LineHeadingControl control;
    LineHeadingControl_Input input = default_input();
    LineHeadingControl_Output output;

    LineHeadingControl_Init(&control);
    input.heading_deg = 179.0f;
    LineHeadingControl_Update(&control, &input, &output);

    input.now_ms = 20U;
    input.heading_deg = -179.0f;
    input.line_new = 0U;
    input.line_age_ms = 20U;
    LineHeadingControl_Update(&control, &input, &output);

    CHECK(output.heading_error_deg < 0.0f);
    CHECK(abs_float(output.heading_error_deg) < 3.0f);
    return 1;
}

int main(void)
{
    if (!test_centered_line_uses_imu_straight() ||
        !test_right_line_error_changes_heading_not_directly_wheels() ||
        !test_visual_offset_is_limited_and_slewed() ||
        !test_stale_line_decays_offset_and_bridges_with_imu() ||
        !test_invalid_line_stops_only_after_bridge_timeout() ||
        !test_junction_freezes_visual_correction() ||
        !test_heading_wrap_uses_shortest_direction()) {
        return 1;
    }

    puts("line_heading_control tests passed");
    return 0;
}
