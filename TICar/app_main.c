#include "app_main.h"

#include "app_config.h"
#include "bsp_ccd.h"
#include "bsp_gpio.h"
#include "bsp_motor.h"
#include "bsp_ptz.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "pid.h"
#include "protocol_k230.h"
#include "protocol_tjc.h"

#define APP_MOTOR_TEST_SLOW_SPEED 0.18f
#define APP_MOTOR_TEST_FAST_SPEED 0.45f
#define APP_CCD_LOG_FRAME_COUNT 32U
#define APP_LINE_BASE_SPEED 0.49f
#define APP_LINE_BASE_TARGET_CM_S 30.0f
#define APP_LINE_KP 0.0020f
#define APP_LINE_KD 0.0005f
#define APP_LINE_STEER_LIMIT 0.130f
#define APP_LINE_MEDIUM_MIN_STEER 0.020f
#define APP_LINE_LARGE_MIN_STEER 0.090f
#define APP_LINE_CORRECTION_SLEW_LIMIT 0.035f
#define APP_LINE_MEDIUM_SPEED_SCALE 0.65f
#define APP_LINE_LARGE_SPEED_SCALE 0.39f
#define APP_LINE_DEADBAND_ERROR 5
#define APP_LINE_MEDIUM_ERROR 6
#define APP_LINE_LARGE_ERROR 12
#define APP_LINE_STEER_SIGN 1.0f
#define APP_CCD_STRAIGHT_SPEED 0.16f
#define APP_LINE_LOST_RECOVER_MAX 8U
#define APP_LINE_LOST_SPEED_SCALE 0.55f
#define APP_LINE_LOST_TURN 0.040f
#define APP_SQUARE_LOST_RECOVER_MAX 14U
#define APP_SQUARE_LOST_SPEED_SCALE 0.45f
#define APP_SQUARE_LOST_TURN 0.060f
#define APP_SQUARE_CORNER_ENTER_ERROR 32
#define APP_SQUARE_CORNER_EXIT_ERROR 5
#define APP_SQUARE_CORNER_MISSING_LOOPS 3U
#define APP_SQUARE_CORNER_BRAKE_LOOPS 5U
#define APP_SQUARE_CORNER_MAX_PIVOT_LOOPS 90U
#define APP_SQUARE_CORNER_PIVOT_SPEED 0.16f
#define APP_SQUARE_CORNER_DEFAULT_DIRECTION (-1)
#define APP_SPEED_TEST_LOW_CM_S 20.0f
#define APP_SPEED_TEST_HIGH_CM_S 30.0f
#define APP_SPEED_TEST_WARMUP_LOOPS 250U
#define APP_SPEED_TEST_HOLD_LOOPS 250U
#define APP_K230_FOLLOW_KP_NUM 8
#define APP_K230_FOLLOW_KP_DEN 100
#define APP_K230_FOLLOW_X_DEADBAND 10
#define APP_K230_FOLLOW_Y_DEADBAND 8
#define APP_K230_FOLLOW_PAN_MAX_STEP 20
#define APP_K230_FOLLOW_TILT_MAX_STEP 15
#define APP_K230_FOLLOW_PAN_DIRECTION (-1)
#define APP_K230_FOLLOW_TILT_DIRECTION 1
#define APP_K230_FOLLOW_STATE_WAIT_LINK 0U
#define APP_K230_FOLLOW_STATE_TRACKING 1U
#define APP_K230_FOLLOW_STATE_HOLD 2U
#define APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED 3U
#define APP_MODE_SWITCH_BRAKE_MS 100U
#define APP_LOOP_FAST_MS 20U
#define APP_LOOP_PRINT_MS 500U
#define APP_LOOP_SLOW_MS 1000U

#define APP_SWITCH_IDLE 0U
#define APP_SWITCH_BRAKING 1U
#define APP_SWITCH_ENTERING 2U

typedef struct {
    float base_speed;
    float kp;
    float kd;
    float steer_sign;
    float steer_limit;
    int16_t medium_error;
    int16_t large_error;
    float medium_speed_scale;
    float large_speed_scale;
    float medium_min_steer;
    float large_min_steer;
    uint16_t lost_recover_max;
    float lost_speed_scale;
    float lost_turn;
} App_FollowProfile;

typedef struct {
    uint16_t frame_count;
    uint16_t valid;
    int16_t target;
    int16_t error;
    int16_t black_left;
    int16_t black_right;
    uint16_t black_width;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t threshold;
    uint16_t contrast;
    uint16_t raw_min_index;
    uint16_t raw_max_index;
    uint16_t raw[BSP_CCD_PIXEL_COUNT];
} App_CcdLogFrame;

static const App_FollowProfile g_line_follow_profile = {
    APP_LINE_BASE_SPEED, APP_LINE_KP, APP_LINE_KD, APP_LINE_STEER_SIGN,
    APP_LINE_STEER_LIMIT, APP_LINE_MEDIUM_ERROR, APP_LINE_LARGE_ERROR,
    APP_LINE_MEDIUM_SPEED_SCALE, APP_LINE_LARGE_SPEED_SCALE,
    APP_LINE_MEDIUM_MIN_STEER, APP_LINE_LARGE_MIN_STEER,
    APP_LINE_LOST_RECOVER_MAX, APP_LINE_LOST_SPEED_SCALE, APP_LINE_LOST_TURN
};

static const App_FollowProfile g_circle_follow_profile = {
    APP_LINE_BASE_SPEED, APP_LINE_KP, APP_LINE_KD, APP_LINE_STEER_SIGN,
    APP_LINE_STEER_LIMIT, APP_LINE_MEDIUM_ERROR, APP_LINE_LARGE_ERROR,
    APP_LINE_MEDIUM_SPEED_SCALE, APP_LINE_LARGE_SPEED_SCALE,
    APP_LINE_MEDIUM_MIN_STEER, APP_LINE_LARGE_MIN_STEER,
    APP_LINE_LOST_RECOVER_MAX, APP_LINE_LOST_SPEED_SCALE, APP_LINE_LOST_TURN
};

static const App_FollowProfile g_square_follow_profile = {
    APP_LINE_BASE_SPEED, APP_LINE_KP, APP_LINE_KD, APP_LINE_STEER_SIGN,
    APP_LINE_STEER_LIMIT, APP_LINE_MEDIUM_ERROR, APP_LINE_LARGE_ERROR,
    APP_LINE_MEDIUM_SPEED_SCALE, APP_LINE_LARGE_SPEED_SCALE,
    APP_LINE_MEDIUM_MIN_STEER, APP_LINE_LARGE_MIN_STEER,
    APP_SQUARE_LOST_RECOVER_MAX, APP_SQUARE_LOST_SPEED_SCALE,
    APP_SQUARE_LOST_TURN
};

volatile uint32_t g_app_debug_mode = APP_MODE_STOPPED;
volatile uint8_t g_app_current_mode = APP_MODE_STOPPED;
volatile uint8_t g_app_requested_mode = APP_MODE_STOPPED;
volatile uint8_t g_app_switch_state = APP_SWITCH_IDLE;
volatile uint32_t g_app_mode_switch_count;
volatile uint32_t g_app_mode_reject_count;

static uint32_t g_app_switch_deadline_ms;
static uint8_t g_app_switch_request;
static uint8_t g_ccd_initialized;
static uint8_t g_encoder_initialized;
static uint8_t g_speed_pid_initialized;
static uint32_t g_mode_last_task_ms;
static uint8_t g_motor_test_phase;
static uint32_t g_motor_test_phase_start_ms;
static uint32_t g_uart_test_count;

static uint32_t g_k230_follow_seen_timeout_count;
volatile int16_t g_k230_control_error_x;
volatile int16_t g_k230_control_error_y;
volatile int16_t g_k230_pan_delta;
volatile int16_t g_k230_tilt_delta;
volatile uint16_t g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
volatile uint16_t g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
volatile uint8_t g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
volatile uint32_t g_k230_follow_update_count;

volatile int32_t g_encoder_left_count;
volatile int32_t g_encoder_right_count;
volatile int16_t g_encoder_left_speed;
volatile int16_t g_encoder_right_speed;
volatile float g_speed_left_target;
volatile float g_speed_right_target;
volatile float g_speed_left_measured;
volatile float g_speed_right_measured;
volatile float g_speed_left_cmd;
volatile float g_speed_right_cmd;
volatile float g_speed_left_p;
volatile float g_speed_right_p;
volatile float g_speed_left_i;
volatile float g_speed_right_i;
volatile uint32_t g_speed_faults;
volatile uint32_t g_speed_test_phase;
volatile uint32_t g_speed_test_loop;

volatile uint32_t g_debug_ccd_frame_count;
volatile uint8_t g_debug_ccd_valid;
volatile int16_t g_debug_ccd_target = -1;
volatile int16_t g_debug_ccd_error;
volatile int16_t g_debug_ccd_black_left = -1;
volatile int16_t g_debug_ccd_black_right = -1;
volatile uint16_t g_debug_ccd_black_width;
volatile uint16_t g_debug_ccd_raw_min;
volatile uint16_t g_debug_ccd_raw_max;
volatile uint16_t g_debug_ccd_raw_min_index;
volatile uint16_t g_debug_ccd_raw_max_index;
volatile uint16_t g_debug_ccd_contrast;
volatile uint16_t g_debug_ccd_threshold;
volatile int16_t g_debug_ccd_dx_max;
volatile int16_t g_debug_ccd_dx_min;
volatile uint16_t g_debug_ccd_dx_max_index;
volatile uint16_t g_debug_ccd_dx_min_index;
volatile uint16_t g_debug_ccd_raw[BSP_CCD_PIXEL_COUNT];
volatile uint16_t g_debug_ccd_log_write_index;
volatile uint16_t g_debug_ccd_log_filled;
volatile App_CcdLogFrame g_debug_ccd_log[APP_CCD_LOG_FRAME_COUNT];

volatile uint8_t g_line_valid;
volatile int16_t g_line_target = -1;
volatile int16_t g_line_error;
volatile int16_t g_line_error_delta;
volatile int16_t g_line_dx_max;
volatile int16_t g_line_dx_min;
volatile uint16_t g_line_dx_max_index;
volatile uint16_t g_line_dx_min_index;
volatile uint16_t g_line_raw_min;
volatile uint16_t g_line_raw_max;
volatile uint16_t g_line_raw_min_index;
volatile uint16_t g_line_raw_max_index;
volatile uint16_t g_line_contrast;
volatile uint16_t g_line_lost_count;
volatile float g_line_correction;
volatile float g_line_left_cmd;
volatile float g_line_right_cmd;
volatile int8_t g_line_recover_direction;
static int16_t g_line_last_error;
static float g_line_last_correction;

volatile uint8_t g_square_corner_state;
volatile uint32_t g_square_corner_count;
volatile int8_t g_square_corner_direction;
volatile uint32_t g_square_corner_entry_count;
volatile uint8_t g_square_seen_center_line;
volatile uint8_t g_square_center_missing_count;

static uint8_t App_TimeElapsed(uint32_t *last_ms, uint32_t period_ms)
{
    uint32_t now_ms = Bsp_Time_GetMilliseconds();

    if ((uint32_t) (now_ms - *last_ms) < period_ms) {
        return 0U;
    }

    *last_ms = now_ms;
    return 1U;
}

static uint8_t App_ModeUsesMotor(uint8_t mode)
{
    return ((mode == APP_MODE_MOTOR_PWM) ||
            (mode == APP_MODE_LINE_FOLLOW) ||
            (mode == APP_MODE_CCD_STRAIGHT) ||
            (mode == APP_MODE_SPEED_TEST) ||
            (mode == APP_MODE_CIRCLE_FOLLOW) ||
            (mode == APP_MODE_SQUARE_FOLLOW)) ? 1U : 0U;
}

static void App_EnsureCcd(void)
{
    if (g_ccd_initialized == 0U) {
        Bsp_Ccd_Init();
        g_ccd_initialized = 1U;
    }
}

static void App_EnsureEncoder(void)
{
    if (g_encoder_initialized == 0U) {
        Bsp_Motor_EncoderInit();
        g_encoder_initialized = 1U;
    }
}

static void App_EnsureSpeedPid(void)
{
    App_EnsureEncoder();
    if (g_speed_pid_initialized == 0U) {
        Bsp_Motor_SpeedPidInit();
        g_speed_pid_initialized = 1U;
    }
}

static void App_SendUint32(uint32_t value)
{
    char buf[10];
    uint8_t idx = 0U;

    do {
        buf[idx++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (idx != 0U) {
        Bsp_Uart_K230_SendByte((uint8_t) buf[--idx]);
    }
}

static float App_LimitFloat(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float App_LimitFloatDelta(float value, float last_value, float max_delta)
{
    float delta = value - last_value;

    if (delta > max_delta) {
        return last_value + max_delta;
    }
    if (delta < -max_delta) {
        return last_value - max_delta;
    }
    return value;
}

static void App_UpdateSpeedWatch(void)
{
    g_encoder_left_count = Bsp_Motor_GetLeftEncoderCount();
    g_encoder_right_count = Bsp_Motor_GetRightEncoderCount();
    g_encoder_left_speed = Bsp_Motor_GetLeftEncoderSpeed();
    g_encoder_right_speed = Bsp_Motor_GetRightEncoderSpeed();
    g_speed_left_target = Bsp_Motor_GetLeftSpeedTarget();
    g_speed_right_target = Bsp_Motor_GetRightSpeedTarget();
    g_speed_left_measured = Bsp_Motor_GetLeftMeasuredSpeed();
    g_speed_right_measured = Bsp_Motor_GetRightMeasuredSpeed();
    g_speed_left_cmd = Bsp_Motor_GetLeftSpeedCommand();
    g_speed_right_cmd = Bsp_Motor_GetRightSpeedCommand();
    g_speed_left_p = Bsp_Motor_GetLeftSpeedPTerm();
    g_speed_right_p = Bsp_Motor_GetRightSpeedPTerm();
    g_speed_left_i = Bsp_Motor_GetLeftSpeedITerm();
    g_speed_right_i = Bsp_Motor_GetRightSpeedITerm();
    g_speed_faults = Bsp_Motor_GetSpeedFaults();
}

static void App_LineApplySpeedPid(float left_reference, float right_reference)
{
    const float reference_to_cm_s = APP_LINE_BASE_TARGET_CM_S / APP_LINE_BASE_SPEED;

    Bsp_Motor_SetSpeedTargets(left_reference * reference_to_cm_s,
                              right_reference * reference_to_cm_s);
    Bsp_Motor_SpeedPidUpdate();
    App_UpdateSpeedWatch();
}

static void App_ResetLineState(void)
{
    g_line_valid = 0U;
    g_line_target = -1;
    g_line_error = 0;
    g_line_error_delta = 0;
    g_line_lost_count = 0U;
    g_line_correction = 0.0f;
    g_line_left_cmd = 0.0f;
    g_line_right_cmd = 0.0f;
    g_line_recover_direction = 0;
    g_line_last_error = 0;
    g_line_last_correction = 0.0f;
}

static void App_ResetSquareState(void)
{
    g_square_corner_state = 0U;
    g_square_corner_count = 0U;
    g_square_corner_direction = 0;
    g_square_corner_entry_count = 0U;
    g_square_seen_center_line = 0U;
    g_square_center_missing_count = 0U;
}

static void App_UpdateCcdWatchData(void)
{
    uint16_t i;
    uint16_t log_index;
    const uint16_t *raw = Bsp_Ccd_GetRawFrame();

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_debug_ccd_raw[i] = raw[i];
    }

    g_debug_ccd_target = Bsp_Ccd_GetTargetIndex();
    g_debug_ccd_error = Bsp_Ccd_GetLineError();
    g_debug_ccd_valid = Bsp_Ccd_IsLineValid();
    g_debug_ccd_black_left = Bsp_Ccd_GetBlackLeft();
    g_debug_ccd_black_right = Bsp_Ccd_GetBlackRight();
    g_debug_ccd_black_width = Bsp_Ccd_GetBlackWidth();
    g_debug_ccd_raw_min = Bsp_Ccd_GetRawMin();
    g_debug_ccd_raw_max = Bsp_Ccd_GetRawMax();
    g_debug_ccd_raw_min_index = Bsp_Ccd_GetRawMinIndex();
    g_debug_ccd_raw_max_index = Bsp_Ccd_GetRawMaxIndex();
    g_debug_ccd_contrast = Bsp_Ccd_GetContrast();
    g_debug_ccd_threshold = Bsp_Ccd_GetThreshold();
    g_debug_ccd_dx_max = Bsp_Ccd_GetDxMax();
    g_debug_ccd_dx_min = Bsp_Ccd_GetDxMin();
    g_debug_ccd_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_debug_ccd_dx_min_index = Bsp_Ccd_GetDxMinIndex();
    g_debug_ccd_frame_count++;

    log_index = g_debug_ccd_log_write_index;
    g_debug_ccd_log[log_index].frame_count = (uint16_t) g_debug_ccd_frame_count;
    g_debug_ccd_log[log_index].valid = g_debug_ccd_valid;
    g_debug_ccd_log[log_index].target = g_debug_ccd_target;
    g_debug_ccd_log[log_index].error = g_debug_ccd_error;
    g_debug_ccd_log[log_index].black_left = g_debug_ccd_black_left;
    g_debug_ccd_log[log_index].black_right = g_debug_ccd_black_right;
    g_debug_ccd_log[log_index].black_width = g_debug_ccd_black_width;
    g_debug_ccd_log[log_index].raw_min = g_debug_ccd_raw_min;
    g_debug_ccd_log[log_index].raw_max = g_debug_ccd_raw_max;
    g_debug_ccd_log[log_index].threshold = g_debug_ccd_threshold;
    g_debug_ccd_log[log_index].contrast = g_debug_ccd_contrast;
    g_debug_ccd_log[log_index].raw_min_index = g_debug_ccd_raw_min_index;
    g_debug_ccd_log[log_index].raw_max_index = g_debug_ccd_raw_max_index;
    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_debug_ccd_log[log_index].raw[i] = raw[i];
    }

    g_debug_ccd_log_write_index = (uint16_t) ((log_index + 1U) % APP_CCD_LOG_FRAME_COUNT);
    if (g_debug_ccd_log_filled < APP_CCD_LOG_FRAME_COUNT) {
        g_debug_ccd_log_filled++;
    }
}

static int32_t App_K230Follow_LimitInt32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t App_K230Follow_FilterError(int16_t error, int16_t deadband)
{
    return ((error >= -deadband) && (error <= deadband)) ? 0 : error;
}

static int16_t App_K230Follow_CalculateDelta(int16_t error, int8_t direction, int16_t max_step)
{
    int32_t absolute_error = (error < 0) ? -(int32_t) error : (int32_t) error;
    int32_t magnitude = ((absolute_error * APP_K230_FOLLOW_KP_NUM) +
                         (APP_K230_FOLLOW_KP_DEN / 2)) /
                        APP_K230_FOLLOW_KP_DEN;
    int32_t delta = ((error < 0) ? -magnitude : magnitude) * direction;

    return (int16_t) App_K230Follow_LimitInt32(delta, -max_step, max_step);
}

static void App_K230Follow_ProcessFrame(const K230_TargetFrame *frame)
{
    int16_t requested_pan_delta;
    int16_t requested_tilt_delta;
    int32_t next_pan;
    int32_t next_tilt;
    uint16_t previous_pan;
    uint16_t previous_tilt;

    if ((frame->detected == 0U) || (frame->confidence < K230_MIN_CONFIDENCE)) {
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        g_k230_follow_state = APP_K230_FOLLOW_STATE_HOLD;
        return;
    }

    g_k230_control_error_x = App_K230Follow_FilterError(
        frame->error_x, APP_K230_FOLLOW_X_DEADBAND);
    g_k230_control_error_y = App_K230Follow_FilterError(
        frame->error_y, APP_K230_FOLLOW_Y_DEADBAND);
    requested_pan_delta = App_K230Follow_CalculateDelta(
        g_k230_control_error_x, APP_K230_FOLLOW_PAN_DIRECTION,
        APP_K230_FOLLOW_PAN_MAX_STEP);
    requested_tilt_delta = App_K230Follow_CalculateDelta(
        g_k230_control_error_y, APP_K230_FOLLOW_TILT_DIRECTION,
        APP_K230_FOLLOW_TILT_MAX_STEP);

    previous_pan = g_k230_pan_compare;
    previous_tilt = g_k230_tilt_compare;
    next_pan = App_K230Follow_LimitInt32(
        (int32_t) previous_pan + requested_pan_delta,
        BSP_PTZ_PAN_MIN, BSP_PTZ_PAN_MAX);
    next_tilt = App_K230Follow_LimitInt32(
        (int32_t) previous_tilt + requested_tilt_delta,
        BSP_PTZ_TILT_MIN, BSP_PTZ_TILT_MAX);

    g_k230_pan_compare = (uint16_t) next_pan;
    g_k230_tilt_compare = (uint16_t) next_tilt;
    g_k230_pan_delta = (int16_t) (next_pan - previous_pan);
    g_k230_tilt_delta = (int16_t) (next_tilt - previous_tilt);
    Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare);
    g_k230_follow_state = APP_K230_FOLLOW_STATE_TRACKING;
    g_k230_follow_update_count++;
}

static void App_K230Follow_Task(void)
{
    K230_TargetFrame frame;

    Protocol_K230_Task();
    if (g_k230_timeout_count != g_k230_follow_seen_timeout_count) {
        g_k230_follow_seen_timeout_count = g_k230_timeout_count;
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        Bsp_Ptz_Disable();
        g_k230_follow_state = APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED;
        return;
    }

    if ((Protocol_K230_TakeLatestFrame(&frame) == 0U) ||
        (g_k230_follow_state == APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED)) {
        return;
    }

    App_K230Follow_ProcessFrame(&frame);
}

static uint8_t App_SquareCornerTask(void)
{
    uint8_t line_valid = Bsp_Ccd_IsLineValid();
    int16_t line_error = Bsp_Ccd_GetLineError();
    int16_t abs_error = (line_error < 0) ? (int16_t) (-line_error) : line_error;
    uint16_t black_width = Bsp_Ccd_GetBlackWidth();
    uint8_t center_detected = ((line_valid != 0U) && (black_width != 0U) &&
                               (abs_error < APP_SQUARE_CORNER_ENTER_ERROR)) ? 1U : 0U;
    uint8_t center_missing = ((line_valid == 0U) || (black_width == 0U) ||
                              (abs_error >= APP_SQUARE_CORNER_ENTER_ERROR)) ? 1U : 0U;

    if (center_detected != 0U) {
        g_square_seen_center_line = 1U;
        g_square_center_missing_count = 0U;
    } else if ((center_missing != 0U) &&
               (g_square_center_missing_count < APP_SQUARE_CORNER_MISSING_LOOPS)) {
        g_square_center_missing_count++;
    }

    if ((g_square_corner_state == 0U) && (g_square_seen_center_line != 0U) &&
        (g_square_center_missing_count >= APP_SQUARE_CORNER_MISSING_LOOPS)) {
        g_square_corner_state = 1U;
        g_square_corner_count = 0U;
        g_square_center_missing_count = 0U;
        g_square_corner_direction = APP_SQUARE_CORNER_DEFAULT_DIRECTION;
        g_square_corner_entry_count++;
        Bsp_Motor_SpeedPidStop();
        g_line_last_correction = 0.0f;
        return 1U;
    }

    if (g_square_corner_state == 0U) {
        return 0U;
    }

    if ((line_valid != 0U) && (black_width != 0U) &&
        (abs_error <= APP_SQUARE_CORNER_EXIT_ERROR)) {
        g_square_corner_state = 0U;
        g_square_corner_count = 0U;
        g_line_lost_count = 0U;
        g_line_last_error = line_error;
        g_line_last_correction = 0.0f;
        g_square_seen_center_line = 1U;
        return 0U;
    }

    if (g_square_corner_state == 1U) {
        Bsp_Motor_SpeedPidStop();
        g_square_corner_count++;
        if (g_square_corner_count >= APP_SQUARE_CORNER_BRAKE_LOOPS) {
            g_square_corner_state = 2U;
            g_square_corner_count = 0U;
        }
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
    } else {
        float pivot = APP_SQUARE_CORNER_PIVOT_SPEED *
                      (float) g_square_corner_direction;

        App_LineApplySpeedPid(pivot, -pivot);
        g_line_left_cmd = pivot;
        g_line_right_cmd = -pivot;
        g_square_corner_count++;
        if (g_square_corner_count >= APP_SQUARE_CORNER_MAX_PIVOT_LOOPS) {
            g_square_corner_state = 0U;
            g_square_corner_count = 0U;
            g_line_last_correction = 0.0f;
        }
    }

    g_line_valid = line_valid;
    g_line_target = line_valid ? Bsp_Ccd_GetTargetIndex() : -1;
    g_line_error = line_valid ? line_error : g_line_last_error;
    g_line_error_delta = 0;
    g_line_correction = 0.0f;
    return 1U;
}

static void App_FollowTask(const App_FollowProfile *profile, uint8_t square_mode)
{
    int16_t line_error;
    int16_t error_delta;
    int16_t abs_error;
    float base_speed;
    float steer_limit;
    float min_steer;
    float correction;
    float left_cmd;
    float right_cmd;

    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    Bsp_Ccd_ReadFrame();
    Bsp_Ccd_Process();
    g_line_raw_min = Bsp_Ccd_GetRawMin();
    g_line_raw_max = Bsp_Ccd_GetRawMax();
    g_line_raw_min_index = Bsp_Ccd_GetRawMinIndex();
    g_line_raw_max_index = Bsp_Ccd_GetRawMaxIndex();
    g_line_contrast = Bsp_Ccd_GetContrast();
    g_line_dx_max = Bsp_Ccd_GetDxMax();
    g_line_dx_min = Bsp_Ccd_GetDxMin();
    g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();

    if ((square_mode != 0U) && (App_SquareCornerTask() != 0U)) {
        Bsp_Gpio_ToggleHeartbeat();
        return;
    }

    if (Bsp_Ccd_IsLineValid() != 0U) {
        line_error = Bsp_Ccd_GetLineError();
        error_delta = (int16_t) (line_error - g_line_last_error);
        abs_error = (line_error < 0) ? (int16_t) (-line_error) : line_error;
        base_speed = profile->base_speed;
        steer_limit = profile->steer_limit;
        min_steer = 0.0f;
        correction = profile->steer_sign *
                     ((profile->kp * (float) line_error) +
                      (profile->kd * (float) error_delta));

        if (abs_error <= APP_LINE_DEADBAND_ERROR) {
            correction = 0.0f;
            error_delta = 0;
        } else if (abs_error >= profile->large_error) {
            base_speed = profile->base_speed * profile->large_speed_scale;
            min_steer = profile->large_min_steer;
        } else if (abs_error >= profile->medium_error) {
            base_speed = profile->base_speed * profile->medium_speed_scale;
            steer_limit = profile->steer_limit * 0.75f;
            min_steer = profile->medium_min_steer;
        } else {
            steer_limit = profile->steer_limit * 0.45f;
        }

        if (line_error > 0) {
            g_line_recover_direction = 1;
        } else if (line_error < 0) {
            g_line_recover_direction = -1;
        }

        if ((min_steer > 0.0f) && (correction < min_steer) &&
            (correction > -min_steer)) {
            correction = (line_error > 0) ?
                         (profile->steer_sign * min_steer) :
                         (-profile->steer_sign * min_steer);
        }

        correction = App_LimitFloat(correction, steer_limit);
        correction = App_LimitFloatDelta(
            correction, g_line_last_correction, APP_LINE_CORRECTION_SLEW_LIMIT);
        left_cmd = base_speed + correction;
        right_cmd = base_speed - correction;
        App_LineApplySpeedPid(left_cmd, right_cmd);

        g_line_valid = 1U;
        g_line_lost_count = 0U;
        g_line_target = Bsp_Ccd_GetTargetIndex();
        g_line_error = line_error;
        g_line_error_delta = error_delta;
        g_line_correction = correction;
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        g_line_last_error = line_error;
        g_line_last_correction = correction;
    } else {
        correction = 0.0f;
        left_cmd = 0.0f;
        right_cmd = 0.0f;
        if (square_mode != 0U) {
            g_line_recover_direction = APP_SQUARE_CORNER_DEFAULT_DIRECTION;
        }

        if ((g_line_lost_count < profile->lost_recover_max) &&
            (g_line_recover_direction != 0)) {
            base_speed = profile->base_speed * profile->lost_speed_scale;
            correction = profile->steer_sign * profile->lost_turn *
                         (float) g_line_recover_direction;
            correction = App_LimitFloatDelta(
                correction, g_line_last_correction, APP_LINE_CORRECTION_SLEW_LIMIT);
            left_cmd = base_speed + correction;
            right_cmd = base_speed - correction;
            App_LineApplySpeedPid(left_cmd, right_cmd);
        } else {
            Bsp_Motor_Stop();
        }

        g_line_valid = 0U;
        g_line_lost_count++;
        g_line_target = -1;
        g_line_error = g_line_last_error;
        g_line_error_delta = 0;
        g_line_correction = correction;
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        g_line_last_correction = correction;
    }

    Bsp_Gpio_ToggleHeartbeat();
}

static void App_CcdStraightTask(void)
{
    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    Bsp_Ccd_ReadFrame();
    Bsp_Ccd_Process();
    if (Bsp_Ccd_IsLineValid() != 0U) {
        Bsp_Motor_Set(APP_CCD_STRAIGHT_SPEED, APP_CCD_STRAIGHT_SPEED);
        g_line_valid = 1U;
        g_line_target = Bsp_Ccd_GetTargetIndex();
        g_line_error = Bsp_Ccd_GetLineError();
        g_line_left_cmd = APP_CCD_STRAIGHT_SPEED;
        g_line_right_cmd = APP_CCD_STRAIGHT_SPEED;
    } else {
        Bsp_Motor_Stop();
        g_line_valid = 0U;
        g_line_target = -1;
        g_line_error = 0;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
    }
    g_line_dx_max = Bsp_Ccd_GetDxMax();
    g_line_dx_min = Bsp_Ccd_GetDxMin();
    g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();
    g_line_correction = 0.0f;
    Bsp_Gpio_ToggleHeartbeat();
}

static void App_MotorTestTask(void)
{
    uint32_t now_ms = Bsp_Time_GetMilliseconds();
    uint32_t elapsed_ms = (uint32_t) (now_ms - g_motor_test_phase_start_ms);

    switch (g_motor_test_phase) {
        case 0U:
            if (elapsed_ms == 0U) {
                Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED);
            }
            if (elapsed_ms >= 2000U) {
                Bsp_Motor_Stop();
                g_motor_test_phase = 1U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
        case 1U:
            if (elapsed_ms >= 1000U) {
                Bsp_Motor_Set(APP_MOTOR_TEST_FAST_SPEED, APP_MOTOR_TEST_SLOW_SPEED);
                g_motor_test_phase = 2U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
        case 2U:
            if (elapsed_ms >= 2000U) {
                Bsp_Motor_Stop();
                g_motor_test_phase = 3U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
        default:
            if (elapsed_ms >= 2000U) {
                Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED);
                g_motor_test_phase = 0U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
    }
}

static void App_SpeedTestTask(void)
{
    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    if (g_speed_test_loop < APP_SPEED_TEST_WARMUP_LOOPS) {
        g_speed_test_phase = 0U;
        Bsp_Motor_SetSpeedTargets(0.0f, 0.0f);
    } else if (g_speed_test_loop <
               (APP_SPEED_TEST_WARMUP_LOOPS + APP_SPEED_TEST_HOLD_LOOPS)) {
        g_speed_test_phase = 1U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_LOW_CM_S, APP_SPEED_TEST_LOW_CM_S);
    } else if (g_speed_test_loop <
               (APP_SPEED_TEST_WARMUP_LOOPS + (2U * APP_SPEED_TEST_HOLD_LOOPS))) {
        g_speed_test_phase = 2U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_HIGH_CM_S, APP_SPEED_TEST_HIGH_CM_S);
    } else {
        g_speed_test_phase = 3U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_LOW_CM_S, APP_SPEED_TEST_LOW_CM_S);
    }

    Bsp_Motor_SpeedPidUpdate();
    App_UpdateSpeedWatch();
    g_speed_test_loop++;
    if (g_speed_test_loop >=
        (APP_SPEED_TEST_WARMUP_LOOPS + (3U * APP_SPEED_TEST_HOLD_LOOPS))) {
        g_speed_test_loop = 0U;
    }
    Bsp_Gpio_ToggleHeartbeat();
}

static void App_ModeExit(uint8_t mode)
{
    if ((mode == APP_MODE_LINE_FOLLOW) || (mode == APP_MODE_SPEED_TEST) ||
        (mode == APP_MODE_CIRCLE_FOLLOW) || (mode == APP_MODE_SQUARE_FOLLOW)) {
        Bsp_Motor_SpeedPidStop();
    } else if ((mode == APP_MODE_MOTOR_PWM) ||
               (mode == APP_MODE_CCD_STRAIGHT)) {
        Bsp_Motor_Stop();
    }

    if (mode == APP_MODE_K230_FOLLOW) {
        Bsp_Ptz_Disable();
    }
}

static uint8_t App_ModeEnter(uint8_t mode)
{
    uint32_t now_ms = Bsp_Time_GetMilliseconds();

    g_mode_last_task_ms = now_ms - APP_LOOP_SLOW_MS;
    Bsp_Gpio_SetHeartbeat(0U);

    switch (mode) {
        case APP_MODE_STOPPED:
            return 1U;
        case APP_MODE_HEARTBEAT:
            return 1U;
        case APP_MODE_K230_UART:
            Protocol_K230_Init();
            return 1U;
        case APP_MODE_MOTOR_PWM:
            g_motor_test_phase = 0U;
            g_motor_test_phase_start_ms = now_ms;
            Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED);
            return 1U;
        case APP_MODE_CCD_ADC:
            App_EnsureCcd();
            Bsp_Uart_K230_SendString("BOOT,CCD\r\n");
            return 1U;
        case APP_MODE_UART_TEST:
            g_uart_test_count = 0U;
            Bsp_Uart_K230_SendString("BOOT,UART_TEST\r\n");
            return 1U;
        case APP_MODE_CCD_WATCH:
            App_EnsureCcd();
            return 1U;
        case APP_MODE_LINE_FOLLOW:
            App_EnsureCcd();
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            return 1U;
        case APP_MODE_CCD_STRAIGHT:
            App_EnsureCcd();
            App_ResetLineState();
            return 1U;
        case APP_MODE_ENCODER_WATCH:
            App_EnsureEncoder();
            Bsp_Motor_EncoderReset();
            return 1U;
        case APP_MODE_SPEED_TEST:
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidStop();
            Bsp_Motor_EncoderReset();
            g_speed_test_phase = 0U;
            g_speed_test_loop = 0U;
            return 1U;
        case APP_MODE_CIRCLE_FOLLOW:
            App_EnsureCcd();
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            return 1U;
        case APP_MODE_K230_FOLLOW:
            Protocol_K230_Init();
            Bsp_Ptz_Init();
            Bsp_Ptz_Start();
            g_k230_control_error_x = 0;
            g_k230_control_error_y = 0;
            g_k230_pan_delta = 0;
            g_k230_tilt_delta = 0;
            g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
            g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
            g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
            g_k230_follow_update_count = 0U;
            g_k230_follow_seen_timeout_count = g_k230_timeout_count;
            return 1U;
        case APP_MODE_SQUARE_FOLLOW:
            App_EnsureCcd();
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            App_ResetSquareState();
            return 1U;
        default:
            return 0U;
    }
}

static void App_ModeTask(uint8_t mode)
{
    switch (mode) {
        case APP_MODE_STOPPED:
            break;
        case APP_MODE_HEARTBEAT:
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_SLOW_MS) != 0U) {
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_K230_UART:
            Protocol_K230_Task();
            break;
        case APP_MODE_MOTOR_PWM:
            App_MotorTestTask();
            break;
        case APP_MODE_CCD_ADC:
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_PRINT_MS) != 0U) {
                Bsp_Ccd_ReadFrame();
                Bsp_Ccd_Process();
                Bsp_Ccd_PrintDebugFrame();
            }
            break;
        case APP_MODE_UART_TEST:
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_SLOW_MS) != 0U) {
                Bsp_Uart_K230_SendString("UART_TEST,count=");
                App_SendUint32(g_uart_test_count++);
                Bsp_Uart_K230_SendString("\r\n");
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_CCD_WATCH:
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_PRINT_MS) != 0U) {
                Bsp_Ccd_ReadFrame();
                Bsp_Ccd_Process();
                App_UpdateCcdWatchData();
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_LINE_FOLLOW:
            App_FollowTask(&g_line_follow_profile, 0U);
            break;
        case APP_MODE_CCD_STRAIGHT:
            App_CcdStraightTask();
            break;
        case APP_MODE_ENCODER_WATCH:
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) != 0U) {
                Bsp_Motor_EncoderSample();
                g_encoder_left_count = Bsp_Motor_GetLeftEncoderCount();
                g_encoder_right_count = Bsp_Motor_GetRightEncoderCount();
                g_encoder_left_speed = Bsp_Motor_GetLeftEncoderSpeed();
                g_encoder_right_speed = Bsp_Motor_GetRightEncoderSpeed();
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_SPEED_TEST:
            App_SpeedTestTask();
            break;
        case APP_MODE_CIRCLE_FOLLOW:
            App_FollowTask(&g_circle_follow_profile, 0U);
            break;
        case APP_MODE_K230_FOLLOW:
            App_K230Follow_Task();
            break;
        case APP_MODE_SQUARE_FOLLOW:
            App_FollowTask(&g_square_follow_profile, 1U);
            break;
        default:
            break;
    }
}

static void App_CompleteModeSwitch(void)
{
    uint8_t target_mode = g_app_requested_mode;

    App_ModeExit(g_app_current_mode);
    Bsp_Motor_Disable();
    if (g_speed_pid_initialized != 0U) {
        Bsp_Motor_SpeedPidReset();
    }
    if (g_encoder_initialized != 0U) {
        Bsp_Motor_EncoderReset();
    }
    Bsp_Ptz_Disable();
    Bsp_Ptz_SetCenter();

    if (App_ModeEnter(target_mode) != 0U) {
        g_app_current_mode = target_mode;
        g_app_debug_mode = target_mode;
        g_app_switch_state = APP_SWITCH_IDLE;
        g_app_mode_switch_count++;
        Protocol_Tjc_SendResult(
            (target_mode == APP_MODE_STOPPED) ? TJC_RESULT_STOPPED : TJC_RESULT_SWITCH_OK,
            g_app_current_mode, g_app_switch_request);
    } else {
        g_app_current_mode = APP_MODE_STOPPED;
        g_app_requested_mode = APP_MODE_STOPPED;
        g_app_debug_mode = APP_MODE_STOPPED;
        g_app_switch_state = APP_SWITCH_IDLE;
        g_app_mode_reject_count++;
        Protocol_Tjc_SendResult(
            TJC_RESULT_ENTER_FAILED, g_app_current_mode, g_app_switch_request);
    }
}

static void App_ModeManagerTask(void)
{
    if (g_app_switch_state == APP_SWITCH_BRAKING) {
        if ((uint32_t) (Bsp_Time_GetMilliseconds() - g_app_switch_deadline_ms) <
            APP_MODE_SWITCH_BRAKE_MS) {
            return;
        }
        g_app_switch_state = APP_SWITCH_ENTERING;
    }

    if (g_app_switch_state == APP_SWITCH_ENTERING) {
        App_CompleteModeSwitch();
    }
}

uint8_t App_RequestMode(uint8_t mode)
{
    if (mode > APP_MODE_SQUARE_FOLLOW) {
        g_app_mode_reject_count++;
        return 0U;
    }

    if ((g_app_switch_state == APP_SWITCH_IDLE) &&
        (mode == g_app_current_mode)) {
        Protocol_Tjc_SendResult(
            (mode == APP_MODE_STOPPED) ? TJC_RESULT_STOPPED : TJC_RESULT_ALREADY_ACTIVE,
            g_app_current_mode, mode);
        return 1U;
    }

    if ((g_app_switch_state != APP_SWITCH_IDLE) &&
        (g_app_requested_mode == APP_MODE_STOPPED) &&
        (mode != APP_MODE_STOPPED)) {
        g_app_mode_reject_count++;
        Protocol_Tjc_SendResult(TJC_RESULT_ACCEPTED_BRAKING,
                                g_app_current_mode, APP_MODE_STOPPED);
        return 0U;
    }

    g_app_requested_mode = mode;
    g_app_switch_request = mode;
    Protocol_Tjc_SendResult(
        TJC_RESULT_ACCEPTED_BRAKING, g_app_current_mode, mode);

    if (App_ModeUsesMotor(g_app_current_mode) != 0U) {
        Bsp_Motor_SpeedPidStop();
        Bsp_Motor_Stop();
        g_app_switch_deadline_ms = Bsp_Time_GetMilliseconds();
        g_app_switch_state = APP_SWITCH_BRAKING;
    } else {
        g_app_switch_state = APP_SWITCH_ENTERING;
    }

    return 1U;
}

uint8_t App_GetCurrentMode(void)
{
    return g_app_current_mode;
}

void App_Init(void)
{
    Bsp_Gpio_Init();
    Bsp_Uart_Init();
    Bsp_Time_Init();
    Bsp_Motor_Init();
    Bsp_Motor_Disable();
    Bsp_Ptz_Disable();
    Bsp_Ptz_SetCenter();
    PID_InitDefaults();
    Protocol_K230_Init();
    Protocol_Tjc_Init();

    g_app_current_mode = APP_MODE_STOPPED;
    g_app_requested_mode = APP_MODE_STOPPED;
    g_app_debug_mode = APP_MODE_STOPPED;
    g_app_switch_state = APP_SWITCH_IDLE;
    g_app_mode_switch_count = 0U;
    g_app_mode_reject_count = 0U;
    g_ccd_initialized = 0U;
    g_encoder_initialized = 0U;
    g_speed_pid_initialized = 0U;
    App_ResetLineState();
    App_ResetSquareState();
    Protocol_Tjc_SendResult(TJC_RESULT_STATE, APP_MODE_STOPPED, TJC_COMMAND_QUERY);
}

void App_Loop(void)
{
    Protocol_Tjc_Task();
    App_ModeManagerTask();
    if (g_app_switch_state == APP_SWITCH_IDLE) {
        App_ModeTask(g_app_current_mode);
    }
}
