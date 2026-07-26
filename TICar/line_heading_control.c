/**
 * @file line_heading_control.c
 * @brief 实现 IMU 主控直行、K230 红线低频辅助纠偏的串级控制器。
 */
#include "line_heading_control.h"

#include <stddef.h>
#include <string.h>

/* 新视觉帧只占 35%，抑制 K230 低帧率和检测抖动造成的目标航向跳变。 */
#define LINE_HEADING_VISUAL_FILTER_ALPHA 0.35f
/* 红线误差进入和退出阈值采用迟滞，避免临界值附近频繁启停纠偏。 */
#define LINE_HEADING_CORRECTION_ENTER_PX 12.0f
#define LINE_HEADING_CORRECTION_EXIT_PX 6.0f
#define LINE_HEADING_CORRECTION_ENTER_ANGLE_DEG 3.0f
#define LINE_HEADING_CORRECTION_EXIT_ANGLE_DEG 1.5f
/* 横向误差和红线角度只转换为小范围目标航向偏移，不直接控制车轮。 */
#define LINE_HEADING_ERROR_TO_DEG_KP 0.12f
#define LINE_HEADING_ANGLE_TO_DEG_KP 0.35f
#define LINE_HEADING_VISUAL_OFFSET_LIMIT_DEG 10.0f
#define LINE_HEADING_VISUAL_OFFSET_SLEW_DEG 2.5f
#define LINE_HEADING_VISUAL_OFFSET_DECAY_DEG 0.5f
/* 红线居中且方向稳定时，缓慢吸收 IMU yaw 漂移到当前直线路段基准。 */
#define LINE_HEADING_ROUTE_ADAPT_ALPHA 0.15f
/* 视觉短时无更新由 IMU 桥接；长时间无有效红线仍需停车重新捕获。 */
#define LINE_HEADING_VISUAL_HOLD_MS 250U
#define LINE_HEADING_BLIND_TIMEOUT_MS 1200U
#define LINE_HEADING_BLIND_SPEED_SCALE 0.70f
/* 红线偏差较大时仅适度降速，避免旧视觉误差让车辆几乎停住。 */
#define LINE_HEADING_MEDIUM_ERROR_PX 30.0f
#define LINE_HEADING_LARGE_ERROR_PX 45.0f
#define LINE_HEADING_MEDIUM_SPEED_SCALE 0.80f
#define LINE_HEADING_LARGE_SPEED_SCALE 0.65f
/* 20 ms IMU 航向内环参数，输出单位直接为左右轮 cm/s 差速。 */
#define LINE_HEADING_IMU_KP 0.30f
#define LINE_HEADING_IMU_KD 0.10f
#define LINE_HEADING_IMU_DEADBAND_DEG 0.5f
#define LINE_HEADING_IMU_CORRECTION_LIMIT_CM_S 4.0f
#define LINE_HEADING_IMU_CORRECTION_SLEW_CM_S 1.0f

static float LineHeading_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float LineHeading_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float LineHeading_LimitDelta(
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

static float LineHeading_MoveTowardZero(float value, float maximum_delta)
{
    if (value > maximum_delta) {
        return value - maximum_delta;
    }
    if (value < -maximum_delta) {
        return value + maximum_delta;
    }
    return 0.0f;
}

static float LineHeading_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void LineHeadingControl_Init(LineHeadingControl *control)
{
    LineHeadingControl_Reset(control);
}

void LineHeadingControl_Reset(LineHeadingControl *control)
{
    if (control == NULL) {
        return;
    }
    memset(control, 0, sizeof(*control));
}

void LineHeadingControl_Update(
    LineHeadingControl *control,
    const LineHeadingControl_Input *input,
    LineHeadingControl_Output *output)
{
    uint32_t valid_line_age_ms;
    float abs_line_error;
    float abs_line_angle;
    float raw_visual_offset;
    float heading_error;
    float heading_correction;
    float base_speed;

    if (output == NULL) {
        return;
    }
    memset(output, 0, sizeof(*output));
    if ((control == NULL) || (input == NULL) || (input->imu_fresh == 0U)) {
        return;
    }

    if (control->active == 0U) {
        control->active = 1U;
        control->route_heading_deg =
            LineHeading_NormalizeAngle(input->heading_deg);
        control->last_valid_line_ms = input->now_ms;

        if (input->line_valid != 0U) {
            if (input->line_age_ms != UINT32_MAX) {
                control->last_valid_line_ms =
                    input->now_ms - input->line_age_ms;
            }
            control->filtered_line_error_px =
                (float) input->line_error_px;
            control->filtered_line_angle_deg =
                (float) input->line_angle_d10 / 10.0f;
            control->visual_filter_ready = 1U;
        }
    }

    if ((input->line_new != 0U) && (input->line_valid != 0U)) {
        control->last_valid_line_ms = input->now_ms;

        if (control->visual_filter_ready == 0U) {
            control->filtered_line_error_px =
                (float) input->line_error_px;
            control->filtered_line_angle_deg =
                (float) input->line_angle_d10 / 10.0f;
            control->visual_filter_ready = 1U;
        } else {
            control->filtered_line_error_px +=
                LINE_HEADING_VISUAL_FILTER_ALPHA *
                ((float) input->line_error_px -
                 control->filtered_line_error_px);
            control->filtered_line_angle_deg +=
                LINE_HEADING_VISUAL_FILTER_ALPHA *
                (((float) input->line_angle_d10 / 10.0f) -
                 control->filtered_line_angle_deg);
        }

        if (input->junction_active == 0U) {
            abs_line_error =
                LineHeading_Abs(control->filtered_line_error_px);
            abs_line_angle =
                LineHeading_Abs(control->filtered_line_angle_deg);

            if (control->visual_correction_active == 0U) {
                if ((abs_line_error >= LINE_HEADING_CORRECTION_ENTER_PX) ||
                    (abs_line_angle >=
                     LINE_HEADING_CORRECTION_ENTER_ANGLE_DEG)) {
                    control->visual_correction_active = 1U;
                }
            } else if (
                (abs_line_error <= LINE_HEADING_CORRECTION_EXIT_PX) &&
                (abs_line_angle <=
                 LINE_HEADING_CORRECTION_EXIT_ANGLE_DEG)) {
                control->visual_correction_active = 0U;
            }

            if ((abs_line_error <= LINE_HEADING_CORRECTION_EXIT_PX) &&
                (abs_line_angle <=
                 LINE_HEADING_CORRECTION_EXIT_ANGLE_DEG)) {
                /* 红线稳定居中时缓慢校正基准，抵消无磁力计 yaw 的长期漂移。 */
                heading_error = LineHeading_NormalizeAngle(
                    input->heading_deg - control->route_heading_deg);
                control->route_heading_deg = LineHeading_NormalizeAngle(
                    control->route_heading_deg +
                    LINE_HEADING_ROUTE_ADAPT_ALPHA * heading_error);
            }

            raw_visual_offset = 0.0f;
            if (control->visual_correction_active != 0U) {
                /* 正误差代表红线在右侧，应降低目标航向使车辆向右纠偏。 */
                raw_visual_offset = -(
                    LINE_HEADING_ERROR_TO_DEG_KP *
                        control->filtered_line_error_px +
                    LINE_HEADING_ANGLE_TO_DEG_KP *
                        control->filtered_line_angle_deg);
                raw_visual_offset = LineHeading_Limit(
                    raw_visual_offset,
                    LINE_HEADING_VISUAL_OFFSET_LIMIT_DEG);
            }
            control->visual_offset_deg = LineHeading_LimitDelta(
                raw_visual_offset,
                control->visual_offset_deg,
                LINE_HEADING_VISUAL_OFFSET_SLEW_DEG);
        }
    }

    valid_line_age_ms =
        (uint32_t) (input->now_ms - control->last_valid_line_ms);
    if (valid_line_age_ms >= LINE_HEADING_BLIND_TIMEOUT_MS) {
        output->blind_timeout = 1U;
        return;
    }

    if ((input->line_valid == 0U) ||
        (input->junction_active != 0U) ||
        (valid_line_age_ms > LINE_HEADING_VISUAL_HOLD_MS)) {
        control->visual_offset_deg = LineHeading_MoveTowardZero(
            control->visual_offset_deg,
            LINE_HEADING_VISUAL_OFFSET_DECAY_DEG);
        if (input->junction_active != 0U) {
            control->visual_correction_active = 0U;
        }
    }

    base_speed = input->base_speed_cm_s;
    if ((input->line_valid == 0U) ||
        (valid_line_age_ms > LINE_HEADING_VISUAL_HOLD_MS)) {
        base_speed *= LINE_HEADING_BLIND_SPEED_SCALE;
    } else if ((control->visual_filter_ready != 0U) &&
               (input->junction_active == 0U)) {
        abs_line_error =
            LineHeading_Abs(control->filtered_line_error_px);
        if (abs_line_error >= LINE_HEADING_LARGE_ERROR_PX) {
            base_speed *= LINE_HEADING_LARGE_SPEED_SCALE;
        } else if (abs_line_error >= LINE_HEADING_MEDIUM_ERROR_PX) {
            base_speed *= LINE_HEADING_MEDIUM_SPEED_SCALE;
        }
    }

    output->route_heading_deg = control->route_heading_deg;
    output->visual_offset_deg = control->visual_offset_deg;
    output->target_heading_deg = LineHeading_NormalizeAngle(
        control->route_heading_deg + control->visual_offset_deg);
    heading_error = LineHeading_NormalizeAngle(
        output->target_heading_deg - input->heading_deg);
    if (LineHeading_Abs(heading_error) <=
        LINE_HEADING_IMU_DEADBAND_DEG) {
        heading_error = 0.0f;
    }

    heading_correction =
        LINE_HEADING_IMU_KP * heading_error +
        LINE_HEADING_IMU_KD *
            (heading_error - control->last_heading_error_deg);
    heading_correction = LineHeading_Limit(
        heading_correction,
        LINE_HEADING_IMU_CORRECTION_LIMIT_CM_S);
    heading_correction = LineHeading_LimitDelta(
        heading_correction,
        control->last_heading_correction_cm_s,
        LINE_HEADING_IMU_CORRECTION_SLEW_CM_S);

    output->command_valid = 1U;
    output->visual_correction_active =
        control->visual_correction_active;
    output->heading_error_deg = heading_error;
    output->heading_correction_cm_s = heading_correction;
    output->filtered_line_error_px =
        control->filtered_line_error_px;
    output->filtered_line_angle_deg =
        control->filtered_line_angle_deg;
    output->left_target_cm_s = base_speed - heading_correction;
    output->right_target_cm_s = base_speed + heading_correction;

    control->last_heading_error_deg = heading_error;
    control->last_heading_correction_cm_s = heading_correction;
}
