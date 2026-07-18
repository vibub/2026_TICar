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
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

#define APP_DELAY_1S CPUCLK_FREQ        // 1 second at the configured CPU clock.
#define APP_DELAY_2S (2U * CPUCLK_FREQ) // 2 seconds at the configured CPU clock.
#define APP_DELAY_PAUSE CPUCLK_FREQ     // Pause long enough to see active brake behavior.
#define APP_STRAIGHT_TEST_SPEED 0.45f   // Start slower so left/right drift is easier to observe.
#define APP_MOTOR_TEST_SLOW_SPEED 0.18f // Slow wheel command for visible left/right mapping checks.
#define APP_MOTOR_TEST_FAST_SPEED 0.45f // Fast wheel command for visible left/right mapping checks.
#define APP_CCD_PRINT_DELAY (CPUCLK_FREQ / 2U) // Print CCD frames at 2 Hz so UART output stays readable.
#define APP_CCD_LOG_FRAME_COUNT 32U // Keep a small RAM log that can be exported through the debugger.
#define APP_LINE_BASE_SPEED 0.49f // Slightly faster square-track straight speed after corner timing was stabilized.
#define APP_LINE_BASE_TARGET_CM_S 30.0f // Map the frozen 0.35 straight reference to the verified 30 cm/s wheel target.
#define APP_LINE_KP 0.0020f // STM32-style PD proportional gain for visible recovery in bends.
#define APP_LINE_KD 0.0005f // Smaller derivative reduces correction flips from CCD target jitter.
#define APP_LINE_STEER_LIMIT 0.130f // Stronger bend steering without direction lock.
#define APP_LINE_MEDIUM_MIN_STEER 0.020f // Keep medium-error correction from kicking the car on straights.
#define APP_LINE_LARGE_MIN_STEER 0.090f // Minimum recovery correction in bends before leaving the track.
#define APP_LINE_CORRECTION_SLEW_LIMIT 0.035f // Track-tested response rate that recovers cleanly when leaving bends.
#define APP_LINE_MEDIUM_SPEED_SCALE 0.65f // Medium-error speed is about 0.228 at the 0.35 baseline.
#define APP_LINE_LARGE_SPEED_SCALE 0.39f // Large-error speed is about 0.137 at the 0.35 baseline.
#define APP_LINE_DEADBAND_ERROR 5 // Keep center-line jitter from causing visible dithering.
#define APP_LINE_MEDIUM_ERROR 6 // Start reducing base speed after a small but visible CCD offset.
#define APP_LINE_LARGE_ERROR 12 // Treat larger CCD pixel errors as a clear recovery condition.
#define APP_LINE_STEER_SIGN 1.0f // Keep steering sign positive after logical left/right motor mapping is fixed.
#define APP_CCD_STRAIGHT_SPEED 0.16f // Straight-only CCD test speed uses equal wheel commands and no steering correction.
#define APP_CIRCLE_BASE_SPEED APP_LINE_BASE_SPEED // Circle mode uses the same single CCD PD baseline.
#define APP_CIRCLE_KP APP_LINE_KP // Reuse the stable circle-follow proportional gain.
#define APP_CIRCLE_KD APP_LINE_KD // Reuse the stable circle-follow derivative gain.
#define APP_CIRCLE_STEER_SIGN APP_LINE_STEER_SIGN // Keep circle sign aligned with the verified motor mapping.
#define APP_CIRCLE_STEER_LIMIT APP_LINE_STEER_LIMIT // Reuse the clean circle-follow correction limit.
#define APP_CIRCLE_MEDIUM_ERROR APP_LINE_MEDIUM_ERROR // Circle-mode medium-error threshold.
#define APP_CIRCLE_LARGE_ERROR APP_LINE_LARGE_ERROR // Circle-mode large-error threshold.
#define APP_CIRCLE_LOOP_DELAY (CPUCLK_FREQ / 50U) // Run the circle PD loop around 50 Hz.
#define APP_LINE_LOOP_DELAY (CPUCLK_FREQ / 50U) // Run line-follow updates around 50 Hz like the STM32 reference.
#define APP_LINE_LOST_RECOVER_MAX 8U // Keep lost-line recovery short so straight segments do not weave.
#define APP_LINE_LOST_SPEED_SCALE 0.55f // Slow down during lost-line recovery so the CCD has time to reacquire.
#define APP_LINE_LOST_TURN 0.040f // Keep lost-line recovery gentle so straight segments do not weave.
#define APP_SQUARE_BASE_SPEED APP_LINE_BASE_SPEED // Keep the known-good straight baseline for square-track testing.
#define APP_SQUARE_KP APP_LINE_KP // Preserve the current stable steering gain instead of re-softening the loop.
#define APP_SQUARE_KD APP_LINE_KD // Preserve the current stable derivative gain.
#define APP_SQUARE_STEER_SIGN APP_LINE_STEER_SIGN // Keep square mode aligned with the verified motor mapping.
#define APP_SQUARE_STEER_LIMIT APP_LINE_STEER_LIMIT // Keep normal steering authority unchanged on straight segments.
#define APP_SQUARE_MEDIUM_ERROR APP_LINE_MEDIUM_ERROR // Reuse the stable medium-error threshold.
#define APP_SQUARE_LARGE_ERROR APP_LINE_LARGE_ERROR // Reuse the stable large-error threshold.
#define APP_SQUARE_LOOP_DELAY APP_LINE_LOOP_DELAY // Keep the square follow loop at the same 50 Hz rate.
#define APP_SQUARE_MEDIUM_SPEED_SCALE APP_LINE_MEDIUM_SPEED_SCALE // Keep medium correction behavior unchanged.
#define APP_SQUARE_LARGE_SPEED_SCALE APP_LINE_LARGE_SPEED_SCALE // Keep large-error speed behavior unchanged.
#define APP_SQUARE_MEDIUM_MIN_STEER APP_LINE_MEDIUM_MIN_STEER // Keep straightaway correction unchanged.
#define APP_SQUARE_LARGE_MIN_STEER APP_LINE_LARGE_MIN_STEER // Keep large-error correction unchanged.
#define APP_SQUARE_LOST_RECOVER_MAX 14U // Search longer only in square mode for brief 90-degree corner dropouts.
#define APP_SQUARE_LOST_SPEED_SCALE 0.45f // Slow square-mode lost-line recovery without forcing an abrupt pivot.
#define APP_SQUARE_LOST_TURN 0.060f // Turn more than normal line mode, but less aggressively than the failed test.
#define APP_SQUARE_CORNER_ENTER_ERROR 32 // Wait until the center is closer to disappearing before entering a square corner.
#define APP_SQUARE_CORNER_EXIT_ERROR 5 // Resume normal follow after the black line returns near center.
#define APP_SQUARE_CORNER_MISSING_LOOPS 3U // Confirm the center is missing for about 60 ms before stopping to turn.
#define APP_SQUARE_CORNER_BRAKE_LOOPS 5U // Stop for about 100 ms before pivoting into a right-angle turn.
#define APP_SQUARE_CORNER_MAX_PIVOT_LOOPS 90U // Safety exit after about 1.8 s if center is not reacquired.
#define APP_SQUARE_CORNER_PIVOT_SPEED 0.16f // In-place turn command while searching for the centered line.
#define APP_SQUARE_CORNER_DEFAULT_DIRECTION -1 // Square track is counter-clockwise, so prefer left search when center is lost.

#if APP_MODE == APP_MODE_SQUARE_FOLLOW
#define APP_FOLLOW_BASE_SPEED APP_SQUARE_BASE_SPEED
#define APP_FOLLOW_KP APP_SQUARE_KP
#define APP_FOLLOW_KD APP_SQUARE_KD
#define APP_FOLLOW_STEER_SIGN APP_SQUARE_STEER_SIGN
#define APP_FOLLOW_STEER_LIMIT APP_SQUARE_STEER_LIMIT
#define APP_FOLLOW_MEDIUM_ERROR APP_SQUARE_MEDIUM_ERROR
#define APP_FOLLOW_LARGE_ERROR APP_SQUARE_LARGE_ERROR
#define APP_FOLLOW_LOOP_DELAY APP_SQUARE_LOOP_DELAY
#define APP_FOLLOW_MEDIUM_SPEED_SCALE APP_SQUARE_MEDIUM_SPEED_SCALE
#define APP_FOLLOW_LARGE_SPEED_SCALE APP_SQUARE_LARGE_SPEED_SCALE
#define APP_FOLLOW_MEDIUM_MIN_STEER APP_SQUARE_MEDIUM_MIN_STEER
#define APP_FOLLOW_LARGE_MIN_STEER APP_SQUARE_LARGE_MIN_STEER
#define APP_FOLLOW_LOST_RECOVER_MAX APP_SQUARE_LOST_RECOVER_MAX
#define APP_FOLLOW_LOST_SPEED_SCALE APP_SQUARE_LOST_SPEED_SCALE
#define APP_FOLLOW_LOST_TURN APP_SQUARE_LOST_TURN
#elif APP_MODE == APP_MODE_CIRCLE_FOLLOW
#define APP_FOLLOW_BASE_SPEED APP_CIRCLE_BASE_SPEED
#define APP_FOLLOW_KP APP_CIRCLE_KP
#define APP_FOLLOW_KD APP_CIRCLE_KD
#define APP_FOLLOW_STEER_SIGN APP_CIRCLE_STEER_SIGN
#define APP_FOLLOW_STEER_LIMIT APP_CIRCLE_STEER_LIMIT
#define APP_FOLLOW_MEDIUM_ERROR APP_CIRCLE_MEDIUM_ERROR
#define APP_FOLLOW_LARGE_ERROR APP_CIRCLE_LARGE_ERROR
#define APP_FOLLOW_LOOP_DELAY APP_CIRCLE_LOOP_DELAY
#define APP_FOLLOW_MEDIUM_SPEED_SCALE APP_LINE_MEDIUM_SPEED_SCALE
#define APP_FOLLOW_LARGE_SPEED_SCALE APP_LINE_LARGE_SPEED_SCALE
#define APP_FOLLOW_MEDIUM_MIN_STEER APP_LINE_MEDIUM_MIN_STEER
#define APP_FOLLOW_LARGE_MIN_STEER APP_LINE_LARGE_MIN_STEER
#define APP_FOLLOW_LOST_RECOVER_MAX APP_LINE_LOST_RECOVER_MAX
#define APP_FOLLOW_LOST_SPEED_SCALE APP_LINE_LOST_SPEED_SCALE
#define APP_FOLLOW_LOST_TURN APP_LINE_LOST_TURN
#else
#define APP_FOLLOW_BASE_SPEED APP_LINE_BASE_SPEED
#define APP_FOLLOW_KP APP_LINE_KP
#define APP_FOLLOW_KD APP_LINE_KD
#define APP_FOLLOW_STEER_SIGN APP_LINE_STEER_SIGN
#define APP_FOLLOW_STEER_LIMIT APP_LINE_STEER_LIMIT
#define APP_FOLLOW_MEDIUM_ERROR APP_LINE_MEDIUM_ERROR
#define APP_FOLLOW_LARGE_ERROR APP_LINE_LARGE_ERROR
#define APP_FOLLOW_LOOP_DELAY APP_LINE_LOOP_DELAY
#define APP_FOLLOW_MEDIUM_SPEED_SCALE APP_LINE_MEDIUM_SPEED_SCALE
#define APP_FOLLOW_LARGE_SPEED_SCALE APP_LINE_LARGE_SPEED_SCALE
#define APP_FOLLOW_MEDIUM_MIN_STEER APP_LINE_MEDIUM_MIN_STEER
#define APP_FOLLOW_LARGE_MIN_STEER APP_LINE_LARGE_MIN_STEER
#define APP_FOLLOW_LOST_RECOVER_MAX APP_LINE_LOST_RECOVER_MAX
#define APP_FOLLOW_LOST_SPEED_SCALE APP_LINE_LOST_SPEED_SCALE
#define APP_FOLLOW_LOST_TURN APP_LINE_LOST_TURN
#endif
#define APP_ENCODER_WATCH_DELAY (CPUCLK_FREQ / 50U) // Sample encoder counts around 50 Hz for debugger inspection.
#define APP_SPEED_TEST_LOW_CM_S 20.0f // First and final speed in the automatic step-response test.
#define APP_SPEED_TEST_HIGH_CM_S 30.0f // Middle speed in the automatic step-response test.
#define APP_SPEED_TEST_WARMUP_LOOPS 250U // Hold zero PWM for five seconds so the stationary phase is obvious.
#define APP_SPEED_TEST_HOLD_LOOPS 250U // Hold each 2025-style speed target for five seconds.
#define APP_SPEED_TEST_LOOP_DELAY (CPUCLK_FREQ / 50U) // Run speed PID around 50 Hz like the line-follow loop.
/* K230 云台调参区：优先在这里调整增益、死区、方向和单帧最大步长。 */
#define APP_K230_FOLLOW_PAN_KP_NUM 8 // Pan 比例增益分子，当前 8 / 100 = 0.08 compare/pixel。
#define APP_K230_FOLLOW_PAN_KP_DEN 100
#define APP_K230_FOLLOW_TILT_KP_NUM 8 // Tilt 比例增益分子，可与 Pan 独立调整。
#define APP_K230_FOLLOW_TILT_KP_DEN 100
#define APP_K230_FOLLOW_X_DEADBAND 10 // 水平误差绝对值不超过 10 像素时 Pan 保持不动。
#define APP_K230_FOLLOW_Y_DEADBAND 8 // 垂直误差绝对值不超过 8 像素时 Tilt 保持不动。
#define APP_K230_FOLLOW_PAN_MAX_STEP 20 // 每个新目标帧最多改变 20 个 Pan compare，减小单帧角度跳变。
#define APP_K230_FOLLOW_TILT_MAX_STEP 15 // 每个新目标帧最多改变 15 个 Tilt compare，减小单帧角度跳变。
#define APP_K230_FOLLOW_PAN_DIRECTION (-1) // 正 error_x 使 Pan compare 减小。
#define APP_K230_FOLLOW_TILT_DIRECTION (-1) // 正 error_y 时减小 Tilt compare，使云台向下追踪并形成负反馈。
#define APP_K230_FOLLOW_STARTUP_HOLD_MS 2000U // 上电后两秒内保持双轴初始化位置。
#define APP_K230_FOLLOW_STATE_WAIT_LINK 0U
#define APP_K230_FOLLOW_STATE_TRACKING 1U
#define APP_K230_FOLLOW_STATE_HOLD 2U
#define APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED 3U

volatile uint32_t g_app_debug_mode = 0U; // Watch this value to confirm the running firmware writes the selected APP_MODE.

#if APP_MODE == APP_MODE_K230_FOLLOW
static uint32_t g_k230_follow_seen_timeout_count;
static uint32_t g_k230_follow_start_ms;
volatile int16_t g_k230_control_error_x = 0; // Watch the horizontal error after validity and deadband filtering.
volatile int16_t g_k230_control_error_y = 0; // Watch the vertical error after validity and deadband filtering.
volatile int16_t g_k230_pan_delta = 0; // Watch the actual Pan compare change applied for the latest valid target frame.
volatile int16_t g_k230_tilt_delta = 0; // Watch the actual Tilt compare change applied for the latest valid target frame.
volatile uint16_t g_k230_pan_compare = BSP_PTZ_PAN_CENTER; // Watch the current Pan command sent to the BSP.
volatile uint16_t g_k230_tilt_compare = BSP_PTZ_TILT_CENTER; // Watch the current Tilt command sent to the BSP.
volatile uint8_t g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK; // 0 wait, 1 tracking, 2 hold, 3 timeout-disabled.
volatile uint32_t g_k230_follow_update_count = 0U; // Count valid target frames consumed by the controller.
#endif

#if (APP_MODE == APP_MODE_ENCODER_WATCH) || (APP_MODE == APP_MODE_SPEED_TEST) || \
    (APP_MODE == APP_MODE_LINE_FOLLOW) || (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || \
    (APP_MODE == APP_MODE_SQUARE_FOLLOW)
volatile int32_t g_encoder_left_count = 0; // Watch this value for logical-left accumulated encoder ticks.
volatile int32_t g_encoder_right_count = 0; // Watch this value for logical-right accumulated encoder ticks.
volatile int16_t g_encoder_left_speed = 0; // Watch this value for logical-left ticks in the latest sample window.
volatile int16_t g_encoder_right_speed = 0; // Watch this value for logical-right ticks in the latest sample window.
volatile float g_speed_left_target = 0.0f; // Watch the logical-left ramped speed target in cm/s.
volatile float g_speed_right_target = 0.0f; // Watch the logical-right ramped speed target in cm/s.
volatile float g_speed_left_measured = 0.0f; // Watch filtered logical-left measured speed in cm/s.
volatile float g_speed_right_measured = 0.0f; // Watch filtered logical-right measured speed in cm/s.
volatile float g_speed_left_cmd = 0.0f; // Watch this value for logical-left PWM ratio generated by speed PI.
volatile float g_speed_right_cmd = 0.0f; // Watch this value for logical-right PWM ratio generated by speed PI.
volatile float g_speed_left_p = 0.0f; // Watch logical-left proportional PWM contribution.
volatile float g_speed_right_p = 0.0f; // Watch logical-right proportional PWM contribution.
volatile float g_speed_left_i = 0.0f; // Watch logical-left integral PWM contribution.
volatile float g_speed_right_i = 0.0f; // Watch logical-right integral PWM contribution.
volatile uint32_t g_speed_faults = 0U; // Bit 0 left encoder loss, bit 1 right encoder loss.
#endif

#if APP_MODE == APP_MODE_SPEED_TEST
volatile uint32_t g_speed_test_phase = 0U; // 0 warmup, 1 at 20 cm/s, 2 at 30 cm/s, 3 returned to 20 cm/s.
volatile uint32_t g_speed_test_loop = 0U; // Current 20 ms loop index for locating target transitions in Watch.
#endif

#if APP_MODE == APP_MODE_CCD_WATCH
typedef struct {
    uint16_t frame_count; // CSV column: low 16 bits of the frame counter.
    uint16_t valid; // CSV column: 1 when a black line is detected.
    int16_t target; // CSV column: detected black-line center index.
    int16_t error; // CSV column: target minus CCD center index.
    int16_t black_left; // CSV column: detected dark-region left edge.
    int16_t black_right; // CSV column: detected dark-region right edge.
    uint16_t black_width; // CSV column: detected dark-region width.
    uint16_t raw_min; // CSV column: darkest filtered CCD sample.
    uint16_t raw_max; // CSV column: brightest filtered CCD sample.
    uint16_t threshold; // CSV column: adaptive black-line threshold.
    uint16_t contrast; // CSV column: raw_max minus raw_min.
    uint16_t raw_min_index; // CSV column: index of the darkest filtered pixel.
    uint16_t raw_max_index; // CSV column: index of the brightest filtered pixel.
    uint16_t raw[BSP_CCD_PIXEL_COUNT]; // CSV columns: raw_0 through raw_127.
} App_CcdLogFrame;
#endif

#if APP_MODE == APP_MODE_CCD_WATCH
volatile uint32_t g_debug_ccd_frame_count = 0U; // Watch this value to confirm CCD frames keep updating.
volatile uint8_t g_debug_ccd_valid = 0U; // Watch this value; 1 means the current frame has a detected black line.
volatile int16_t g_debug_ccd_target = -1; // Watch this value for the detected black-line center index.
volatile int16_t g_debug_ccd_error = 0; // Watch this value for signed center error; positive means line is right.
volatile int16_t g_debug_ccd_black_left = -1; // Watch this value for the left edge of the detected dark region.
volatile int16_t g_debug_ccd_black_right = -1; // Watch this value for the right edge of the detected dark region.
volatile uint16_t g_debug_ccd_black_width = 0U; // Watch this value for the detected dark-region width.
volatile uint16_t g_debug_ccd_raw_min = 0U; // Watch this value for the darkest filtered CCD sample.
volatile uint16_t g_debug_ccd_raw_max = 0U; // Watch this value for the brightest filtered CCD sample.
volatile uint16_t g_debug_ccd_raw_min_index = 0U; // Watch this value to see where the darkest pixel is.
volatile uint16_t g_debug_ccd_raw_max_index = 0U; // Watch this value to see where the brightest pixel is.
volatile uint16_t g_debug_ccd_contrast = 0U; // Watch this value; low contrast means CCD/lighting is not reliable.
volatile uint16_t g_debug_ccd_threshold = 0U; // Watch this value for the adaptive black-line threshold.
volatile int16_t g_debug_ccd_dx_max = 0; // Watch this value for the strongest 2025-style positive CCD edge.
volatile int16_t g_debug_ccd_dx_min = 0; // Watch this value for the strongest 2025-style negative CCD edge.
volatile uint16_t g_debug_ccd_dx_max_index = 0U; // Watch this value for the positive-edge index used by the 2025 CCD algorithm.
volatile uint16_t g_debug_ccd_dx_min_index = 0U; // Watch this value for the negative-edge index used by the 2025 CCD algorithm.
volatile uint16_t g_debug_ccd_raw[BSP_CCD_PIXEL_COUNT]; // Watch this array to inspect all 128 raw CCD pixels.
volatile uint16_t g_debug_ccd_log_write_index = 0U; // Watch this value to know which RAM log slot will be written next.
volatile uint16_t g_debug_ccd_log_filled = 0U; // Watch this value; 32 means the RAM log has one full buffer.
volatile App_CcdLogFrame g_debug_ccd_log[APP_CCD_LOG_FRAME_COUNT]; // Export this RAM block through CCS Memory Browser for CSV conversion.
#endif

#if (APP_MODE == APP_MODE_LINE_FOLLOW) || (APP_MODE == APP_MODE_CCD_STRAIGHT) || \
    (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || (APP_MODE == APP_MODE_SQUARE_FOLLOW)
volatile uint8_t g_line_valid = 0U; // Watch this value; 1 means line-follow is actively driving.
volatile int16_t g_line_target = -1; // Watch this value for the latest CCD target used by steering.
volatile int16_t g_line_error = 0; // Watch this value for the latest centered CCD error.
volatile int16_t g_line_error_delta = 0; // Watch this value for the error change used by the derivative term.
volatile int16_t g_line_dx_max = 0; // Watch this value for the strongest 2025-style positive CCD edge.
volatile int16_t g_line_dx_min = 0; // Watch this value for the strongest 2025-style negative CCD edge.
volatile uint16_t g_line_dx_max_index = 0U; // Watch this value for the positive-edge index used by the 2025 CCD algorithm.
volatile uint16_t g_line_dx_min_index = 0U; // Watch this value for the negative-edge index used by the 2025 CCD algorithm.
volatile uint16_t g_line_raw_min = 0U; // Watch the darkest filtered CCD sample in normal line-follow mode.
volatile uint16_t g_line_raw_max = 0U; // Watch the brightest filtered CCD sample in normal line-follow mode.
volatile uint16_t g_line_raw_min_index = 0U; // Watch whether the darkest CCD sample is stuck at a sensor edge.
volatile uint16_t g_line_raw_max_index = 0U; // Watch whether the brightest CCD sample is stuck at a sensor edge.
volatile uint16_t g_line_contrast = 0U; // Watch the current CCD brightness span while the motors are running.
volatile uint16_t g_line_lost_count = 0U; // Watch this value for consecutive target-missing frames.
volatile float g_line_correction = 0.0f; // Watch this value for the actual steering correction applied to the wheels.
volatile float g_line_left_cmd = 0.0f; // Watch this value for the commanded logical left motor ratio.
volatile float g_line_right_cmd = 0.0f; // Watch this value for the commanded logical right motor ratio.
volatile int8_t g_line_recover_direction = 0; // Watch this value; -1 searches left, +1 searches right after target loss.
static int16_t g_line_last_error = 0; // Keep the previous fixed-center error for the derivative steering term.
static float g_line_last_correction = 0.0f; // Smooth steering output so CCD jitter does not kick the chassis.
#endif

#if APP_MODE == APP_MODE_SQUARE_FOLLOW
volatile uint8_t g_square_corner_state = 0U; // 0 normal, 1 braking, 2 pivoting until the line returns to center.
volatile uint32_t g_square_corner_count = 0U; // Watch how many 20 ms loops have elapsed in the current corner state.
volatile int8_t g_square_corner_direction = 0; // -1 pivots left, +1 pivots right.
volatile uint32_t g_square_corner_entry_count = 0U; // Count detected square-corner entries for tuning.
volatile uint8_t g_square_seen_center_line = 0U; // 1 after a real centered black region has been detected.
volatile uint8_t g_square_center_missing_count = 0U; // Debounce early corner entry when the line only briefly shifts.
#endif

#if APP_MODE == APP_MODE_UART_TEST
static void App_SendUint32(uint32_t value)
{
    char buf[10];
    uint8_t idx = 0U;

    do {
        buf[idx] = (char) ('0' + (value % 10U));
        value /= 10U;
        idx++;
    } while (value != 0U);

    while (idx != 0U) {
        idx--;
        Bsp_Uart_K230_SendByte((uint8_t) buf[idx]); // Send decimal digits without pulling in stdio.
    }
}
#endif

#if (APP_MODE == APP_MODE_LINE_FOLLOW) || (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || \
    (APP_MODE == APP_MODE_SQUARE_FOLLOW)
static float App_LimitFloat(float value, float limit)
{
    if (value > limit) {
        return limit; // Clamp positive correction to keep first map runs gentle.
    }
    if (value < -limit) {
        return -limit; // Clamp negative correction to keep first map runs gentle.
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

static void App_LineApplySpeedPid(float left_reference, float right_reference)
{
    const float reference_to_cm_s = APP_LINE_BASE_TARGET_CM_S / APP_LINE_BASE_SPEED;

    Bsp_Motor_SetSpeedTargets(left_reference * reference_to_cm_s,
                              right_reference * reference_to_cm_s); // Preserve the frozen CCD left/right ratio in cm/s.
    Bsp_Motor_SpeedPidUpdate(); // Close each wheel independently around the CCD-derived speed targets.
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
#endif

#if APP_MODE == APP_MODE_CCD_WATCH
static void App_UpdateCcdWatchData(void)
{
    uint16_t i;
    uint16_t log_index;
    const uint16_t *raw = Bsp_Ccd_GetRawFrame();

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_debug_ccd_raw[i] = raw[i]; // Mirror raw pixels into volatile memory for CCS Watch/Expressions.
    }

    g_debug_ccd_target = Bsp_Ccd_GetTargetIndex(); // Mirror target so the debugger can read it without UART.
    g_debug_ccd_error = Bsp_Ccd_GetLineError(); // Mirror signed error so the debugger can read it without UART.
    g_debug_ccd_valid = Bsp_Ccd_IsLineValid(); // Mirror detection validity so -1 target is easy to interpret.
    g_debug_ccd_black_left = Bsp_Ccd_GetBlackLeft(); // Mirror detected dark-region left edge for tuning.
    g_debug_ccd_black_right = Bsp_Ccd_GetBlackRight(); // Mirror detected dark-region right edge for tuning.
    g_debug_ccd_black_width = Bsp_Ccd_GetBlackWidth(); // Mirror detected dark-region width for tuning.
    g_debug_ccd_raw_min = Bsp_Ccd_GetRawMin(); // Mirror darkest sample for contrast checks.
    g_debug_ccd_raw_max = Bsp_Ccd_GetRawMax(); // Mirror brightest sample for contrast checks.
    g_debug_ccd_raw_min_index = Bsp_Ccd_GetRawMinIndex(); // Mirror darkest index to catch edge artifacts.
    g_debug_ccd_raw_max_index = Bsp_Ccd_GetRawMaxIndex(); // Mirror brightest index to catch edge artifacts.
    g_debug_ccd_contrast = Bsp_Ccd_GetContrast(); // Mirror contrast so lighting can be judged in Watch.
    g_debug_ccd_threshold = Bsp_Ccd_GetThreshold(); // Mirror adaptive threshold for contrast checks.
    g_debug_ccd_dx_max = Bsp_Ccd_GetDxMax(); // Mirror strongest positive edge for 2025-style target debugging.
    g_debug_ccd_dx_min = Bsp_Ccd_GetDxMin(); // Mirror strongest negative edge for 2025-style target debugging.
    g_debug_ccd_dx_max_index = Bsp_Ccd_GetDxMaxIndex(); // Mirror positive-edge index for 2025-style target debugging.
    g_debug_ccd_dx_min_index = Bsp_Ccd_GetDxMinIndex(); // Mirror negative-edge index for 2025-style target debugging.
    g_debug_ccd_frame_count++; // Increment after a complete mirrored frame is ready.

    log_index = g_debug_ccd_log_write_index;
    g_debug_ccd_log[log_index].frame_count = (uint16_t) g_debug_ccd_frame_count; // Store compact frame id for CSV export.
    g_debug_ccd_log[log_index].valid = g_debug_ccd_valid; // Store current valid flag for CSV export.
    g_debug_ccd_log[log_index].target = g_debug_ccd_target; // Store current target for CSV export.
    g_debug_ccd_log[log_index].error = g_debug_ccd_error; // Store current error for CSV export.
    g_debug_ccd_log[log_index].black_left = g_debug_ccd_black_left; // Store detected left edge for CSV export.
    g_debug_ccd_log[log_index].black_right = g_debug_ccd_black_right; // Store detected right edge for CSV export.
    g_debug_ccd_log[log_index].black_width = g_debug_ccd_black_width; // Store detected width for CSV export.
    g_debug_ccd_log[log_index].raw_min = g_debug_ccd_raw_min; // Store darkest value for CSV export.
    g_debug_ccd_log[log_index].raw_max = g_debug_ccd_raw_max; // Store brightest value for CSV export.
    g_debug_ccd_log[log_index].threshold = g_debug_ccd_threshold; // Store threshold for CSV export.
    g_debug_ccd_log[log_index].contrast = g_debug_ccd_contrast; // Store contrast for CSV export.
    g_debug_ccd_log[log_index].raw_min_index = g_debug_ccd_raw_min_index; // Store darkest index for CSV export.
    g_debug_ccd_log[log_index].raw_max_index = g_debug_ccd_raw_max_index; // Store brightest index for CSV export.
    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_debug_ccd_log[log_index].raw[i] = raw[i]; // Store one complete raw CCD frame for CSV export.
    }

    log_index++;
    if (log_index >= APP_CCD_LOG_FRAME_COUNT) {
        log_index = 0U; // Wrap the RAM log so the newest 32 frames are always available.
    }
    g_debug_ccd_log_write_index = log_index;
    if (g_debug_ccd_log_filled < APP_CCD_LOG_FRAME_COUNT) {
        g_debug_ccd_log_filled++; // Count how many log slots have valid data after startup.
    }
}
#endif

#if APP_MODE == APP_MODE_K230_FOLLOW
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
    if ((error >= -deadband) && (error <= deadband)) {
        return 0;
    }
    return error;
}

static int16_t App_K230Follow_CalculateDelta(
    int16_t error,
    int8_t direction,
    int16_t kp_num,
    int16_t kp_den,
    int16_t max_step)
{
    int32_t absolute_error = (error < 0) ? -(int32_t) error : (int32_t) error;
    int32_t magnitude = ((absolute_error * kp_num) + (kp_den / 2)) / kp_den;
    int32_t delta = (error < 0) ? -magnitude : magnitude;

    delta *= direction;
    delta = App_K230Follow_LimitInt32(delta, -max_step, max_step); // 先限制单帧变化，再累加到当前舵机位置。
    return (int16_t) delta;
}

static void App_K230Follow_ProcessFrame(const K230_TargetFrame *frame)
{
    int16_t requested_pan_delta;
    int16_t requested_tilt_delta;
    int32_t next_pan;
    int32_t next_tilt;
    uint16_t previous_pan;
    uint16_t previous_tilt;

    if ((frame->detected == 0U) ||
        (frame->confidence < K230_MIN_CONFIDENCE)) {
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        g_k230_follow_state = APP_K230_FOLLOW_STATE_HOLD;
        return; // A valid N or low-confidence T frame holds the last safe position.
    }

    g_k230_control_error_x = App_K230Follow_FilterError(
        frame->error_x, APP_K230_FOLLOW_X_DEADBAND);
    g_k230_control_error_y = App_K230Follow_FilterError(
        frame->error_y, APP_K230_FOLLOW_Y_DEADBAND);
    requested_pan_delta = App_K230Follow_CalculateDelta(
        g_k230_control_error_x,
        APP_K230_FOLLOW_PAN_DIRECTION,
        APP_K230_FOLLOW_PAN_KP_NUM,
        APP_K230_FOLLOW_PAN_KP_DEN,
        APP_K230_FOLLOW_PAN_MAX_STEP);
    requested_tilt_delta = App_K230Follow_CalculateDelta(
        g_k230_control_error_y,
        APP_K230_FOLLOW_TILT_DIRECTION,
        APP_K230_FOLLOW_TILT_KP_NUM,
        APP_K230_FOLLOW_TILT_KP_DEN,
        APP_K230_FOLLOW_TILT_MAX_STEP);

    previous_pan = g_k230_pan_compare;
    previous_tilt = g_k230_tilt_compare;
    next_pan = App_K230Follow_LimitInt32(
        (int32_t) previous_pan + requested_pan_delta,
        BSP_PTZ_PAN_MIN,
        BSP_PTZ_PAN_MAX);
    next_tilt = App_K230Follow_LimitInt32(
        (int32_t) previous_tilt + requested_tilt_delta,
        BSP_PTZ_TILT_MIN,
        BSP_PTZ_TILT_MAX);

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

    if ((uint32_t) (Bsp_Time_GetMilliseconds() - g_k230_follow_start_ms) <
        APP_K230_FOLLOW_STARTUP_HOLD_MS) {
        (void) Protocol_K230_TakeLatestFrame(&frame); // 丢弃启动阶段旧帧，保持结束后只响应新目标。

        /*
         * 启动阶段持续写入中心命令，使 K230 已经通信时也必须先完成回中。
         * 同步软件保存值，保证两秒后的第一次跟随增量从中心位置开始计算。
         */
        g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
        g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
        Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare);

        g_k230_follow_seen_timeout_count = g_k230_timeout_count; // 启动阶段的历史超时不能在两秒后误关闭已恢复链路。
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
        return;
    }

    if (g_k230_timeout_count != g_k230_follow_seen_timeout_count) {
        g_k230_follow_seen_timeout_count = g_k230_timeout_count;
    }
    if ((g_k230_link_alive == 0U) &&
        (g_k230_timeout_count != 0U) &&
        (g_k230_follow_state != APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED)) {
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        Bsp_Ptz_Disable();
        g_k230_follow_state = APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED;
        return;
    }

    if (Protocol_K230_TakeLatestFrame(&frame) == 0U) {
        return;
    }
    if (g_k230_follow_state == APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED) {
        if (g_k230_link_alive == 0U) {
            return;
        }

        Bsp_Ptz_Init(); // 恢复 PB4/PB1 的 PWM 复用，此时定时器仍保持停止。
        Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare); // 启动前写回超时前位置，避免恢复时突然回中。
        Bsp_Ptz_Start();
    }

    App_K230Follow_ProcessFrame(&frame);
}
#endif

void App_Init(void)
{
    g_app_debug_mode = APP_MODE; // Write the mode at runtime so CCS Watch can verify the loaded firmware.
    Bsp_Gpio_Init();
    Bsp_Uart_Init();
#if (APP_MODE == APP_MODE_MOTOR_PWM) || (APP_MODE == APP_MODE_LINE_FOLLOW) || \
    (APP_MODE == APP_MODE_CCD_STRAIGHT) || (APP_MODE == APP_MODE_SPEED_TEST) || \
    (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || (APP_MODE == APP_MODE_SQUARE_FOLLOW)
    Bsp_Motor_Init();
#else
    Bsp_Motor_Disable(); // Non-motor debug modes must keep PWM and H-bridge outputs idle.
#endif
#if APP_MODE == APP_MODE_K230_FOLLOW
    Bsp_Ptz_Init(); // Load safe center compare values before the first servo PWM frame.
    Bsp_Ptz_Start(); // Only the K230 follow mode enables the dual-axis servo output.
    g_k230_control_error_x = 0;
    g_k230_control_error_y = 0;
    g_k230_pan_delta = 0;
    g_k230_tilt_delta = 0;
    g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
    g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
    g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
    g_k230_follow_update_count = 0U;
#elif (APP_MODE != APP_MODE_UART_TEST) && (APP_MODE != APP_MODE_CCD_WATCH) && (APP_MODE != APP_MODE_K230_UART)
    Bsp_Ptz_Disable(); // Full SysConfig modes that do not use the PTZ must force both servo signals low.
#endif
#if (APP_MODE == APP_MODE_ENCODER_WATCH) || (APP_MODE == APP_MODE_SPEED_TEST) || \
    (APP_MODE == APP_MODE_LINE_FOLLOW) || (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || \
    (APP_MODE == APP_MODE_SQUARE_FOLLOW)
    Bsp_Motor_EncoderInit(); // Enable 2025-board encoder pins PB13/PB20 and PB15/PB17.
#endif
#if (APP_MODE == APP_MODE_SPEED_TEST) || (APP_MODE == APP_MODE_LINE_FOLLOW) || \
    (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || (APP_MODE == APP_MODE_SQUARE_FOLLOW)
    Bsp_Motor_SpeedPidInit(); // Prepare both wheel-speed PI controllers before closed-loop driving.
#endif
#if APP_MODE == APP_MODE_CCD_ADC
    Bsp_Uart_K230_SendString("BOOT,CCD\r\n"); // Early UART marker proves the selected COM port is receiving firmware output.
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC before the first frame read.
#elif APP_MODE == APP_MODE_CCD_WATCH
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC for debugger-watch CCD sampling.
#elif (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || (APP_MODE == APP_MODE_LINE_FOLLOW) || \
    (APP_MODE == APP_MODE_SQUARE_FOLLOW)
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC before the single line-follow controller.
    Bsp_Gpio_SetHeartbeat(0U); // Keep the heartbeat under loop control during CCD bringup.
#elif APP_MODE == APP_MODE_CCD_STRAIGHT
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC before straight-only CCD gating.
#elif APP_MODE == APP_MODE_UART_TEST
    Bsp_Uart_K230_SendString("BOOT,UART_TEST\r\n"); // Early marker proves the selected COM port is receiving firmware output.
#elif APP_MODE == APP_MODE_ENCODER_WATCH
    Bsp_Gpio_SetHeartbeat(0U); // Keep heartbeat under software control while encoder samples update.
#elif APP_MODE == APP_MODE_SPEED_TEST
    Bsp_Gpio_SetHeartbeat(0U); // Keep heartbeat under software control while the speed loop is running.
#endif
    Protocol_K230_Init();
#if APP_MODE == APP_MODE_K230_FOLLOW
    g_k230_follow_seen_timeout_count = g_k230_timeout_count;
    g_k230_follow_start_ms = Bsp_Time_GetMilliseconds();
#endif
    PID_InitDefaults();
}

void App_Loop(void)
{
#if APP_MODE == APP_MODE_HEARTBEAT
    Bsp_Gpio_ToggleHeartbeat();
    delay_cycles(16000000);
#elif APP_MODE == APP_MODE_K230_UART
    Protocol_K230_Task();
#elif APP_MODE == APP_MODE_K230_FOLLOW
    Protocol_K230_Task();
    App_K230Follow_Task(); // Consume each new target frame once and update the two servo commands.
#elif APP_MODE == APP_MODE_MOTOR_PWM
    Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED); // Right command is faster; the car should yaw left if mapping is correct.
    delay_cycles(APP_DELAY_2S);
    Bsp_Motor_Stop();
    delay_cycles(APP_DELAY_PAUSE);

    Bsp_Motor_Set(APP_MOTOR_TEST_FAST_SPEED, APP_MOTOR_TEST_SLOW_SPEED); // Left command is faster; the car should yaw right if mapping is correct.
    delay_cycles(APP_DELAY_2S);
    Bsp_Motor_Stop();
    delay_cycles(APP_DELAY_2S);
#elif APP_MODE == APP_MODE_CCD_ADC
    Bsp_Ccd_ReadFrame(); // Capture one 128-pixel CCD line.
    Bsp_Ccd_Process(); // Calculate the detected line center and signed center error.
    Bsp_Ccd_PrintDebugFrame(); // Send target, error, edge strength, and raw pixels over UART.
    delay_cycles(APP_CCD_PRINT_DELAY);
#elif APP_MODE == APP_MODE_CCD_WATCH
    Bsp_Ccd_ReadFrame(); // Capture one 128-pixel CCD line for debugger inspection.
    Bsp_Ccd_Process(); // Calculate line center and signed error before mirroring debug values.
    App_UpdateCcdWatchData(); // Copy CCD results to volatile globals for CCS Watch/Expressions.
    Bsp_Gpio_ToggleHeartbeat(); // LED heartbeat confirms frames are updating without UART.
    delay_cycles(APP_CCD_PRINT_DELAY);
#elif (APP_MODE == APP_MODE_CIRCLE_FOLLOW) || (APP_MODE == APP_MODE_LINE_FOLLOW) || \
    (APP_MODE == APP_MODE_SQUARE_FOLLOW)
    Bsp_Ccd_ReadFrame(); // Capture the latest line position before each CCD steering update.
    Bsp_Ccd_Process(); // Convert CCD pixels into a valid flag and centered error.
    g_line_raw_min = Bsp_Ccd_GetRawMin();
    g_line_raw_max = Bsp_Ccd_GetRawMax();
    g_line_raw_min_index = Bsp_Ccd_GetRawMinIndex();
    g_line_raw_max_index = Bsp_Ccd_GetRawMaxIndex();
    g_line_contrast = Bsp_Ccd_GetContrast(); // Keep illumination diagnostics visible on both valid and lost frames.
#if APP_MODE == APP_MODE_SQUARE_FOLLOW
    uint8_t square_corner_handled = 0U;
    uint8_t square_line_valid = Bsp_Ccd_IsLineValid();
    int16_t square_line_error = Bsp_Ccd_GetLineError();
    int16_t square_abs_error = (square_line_error < 0) ?
                               (int16_t) (-square_line_error) :
                               square_line_error;
    uint16_t square_black_width = Bsp_Ccd_GetBlackWidth();
    uint8_t square_center_detected = ((square_line_valid != 0U) &&
                                      (square_black_width != 0U) &&
                                      (square_abs_error < APP_SQUARE_CORNER_ENTER_ERROR)) ?
                                     1U :
                                     0U;
    uint8_t square_center_missing = ((square_line_valid == 0U) ||
                                     (square_black_width == 0U) ||
                                     (square_abs_error >= APP_SQUARE_CORNER_ENTER_ERROR)) ?
                                    1U :
                                    0U;
    float square_corner_left_cmd = 0.0f;
    float square_corner_right_cmd = 0.0f;

    if (square_center_detected != 0U) {
        g_square_seen_center_line = 1U;
        g_square_center_missing_count = 0U;
    } else if (square_center_missing != 0U) {
        if (g_square_center_missing_count < APP_SQUARE_CORNER_MISSING_LOOPS) {
            g_square_center_missing_count++;
        }
    }

    if ((g_square_corner_state == 0U) &&
        (g_square_seen_center_line != 0U) &&
        (g_square_center_missing_count >= APP_SQUARE_CORNER_MISSING_LOOPS)) {
        g_square_corner_state = 1U;
        g_square_corner_count = 0U;
        g_square_center_missing_count = 0U;
        g_square_corner_direction = APP_SQUARE_CORNER_DEFAULT_DIRECTION;
        g_square_corner_entry_count++;
        Bsp_Motor_SpeedPidStop();
        g_line_last_correction = 0.0f;
        square_corner_handled = 1U;
    }

    if ((g_square_corner_state != 0U) && (square_corner_handled == 0U)) {
        if ((square_line_valid != 0U) &&
            (square_black_width != 0U) &&
            (square_abs_error <= APP_SQUARE_CORNER_EXIT_ERROR)) {
            g_square_corner_state = 0U;
            g_square_corner_count = 0U;
            g_line_lost_count = 0U;
            g_line_last_error = square_line_error;
            g_line_last_correction = 0.0f;
            g_square_seen_center_line = 1U;
        } else if (g_square_corner_state == 1U) {
            Bsp_Motor_SpeedPidStop();
            g_square_corner_count++;
            if (g_square_corner_count >= APP_SQUARE_CORNER_BRAKE_LOOPS) {
                g_square_corner_state = 2U;
                g_square_corner_count = 0U;
            }
            square_corner_handled = 1U;
        } else {
            float left_cmd;
            float right_cmd;
            float pivot = APP_SQUARE_CORNER_PIVOT_SPEED *
                          (float) g_square_corner_direction;

            left_cmd = pivot;
            right_cmd = -pivot;
            App_LineApplySpeedPid(left_cmd, right_cmd);
            square_corner_left_cmd = left_cmd;
            square_corner_right_cmd = right_cmd;
            g_square_corner_count++;
            if (g_square_corner_count >= APP_SQUARE_CORNER_MAX_PIVOT_LOOPS) {
                g_square_corner_state = 0U;
                g_square_corner_count = 0U;
                g_line_last_correction = 0.0f;
            }
            square_corner_handled = 1U;
        }

        g_line_valid = square_line_valid;
        g_line_target = square_line_valid ? Bsp_Ccd_GetTargetIndex() : -1;
        g_line_error = square_line_valid ? square_line_error : g_line_last_error;
        g_line_error_delta = 0;
        g_line_dx_max = Bsp_Ccd_GetDxMax();
        g_line_dx_min = Bsp_Ccd_GetDxMin();
        g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
        g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();
        g_line_correction = 0.0f;
        g_line_left_cmd = square_corner_left_cmd;
        g_line_right_cmd = square_corner_right_cmd;
    }

    if ((square_corner_handled == 0U) && (Bsp_Ccd_IsLineValid() != 0U)) {
#else
    if (Bsp_Ccd_IsLineValid() != 0U) {
#endif
        int16_t line_error = Bsp_Ccd_GetLineError();
        int16_t error_delta = (int16_t) (line_error - g_line_last_error);
        int16_t abs_error = (line_error < 0) ? (int16_t) (-line_error) : line_error;
        float base_speed = APP_FOLLOW_BASE_SPEED;
        float steer_limit = APP_FOLLOW_STEER_LIMIT;
        float min_steer = 0.0f;
        float correction = APP_FOLLOW_STEER_SIGN *
                           ((APP_FOLLOW_KP * (float) line_error) +
                            (APP_FOLLOW_KD * (float) error_delta));
        float left_cmd;
        float right_cmd;

        if (abs_error <= APP_LINE_DEADBAND_ERROR) {
            correction = 0.0f; // Keep center-line jitter from causing visible dithering.
            error_delta = 0; // Keep the derivative Watch value quiet inside the deadband.
        } else if (abs_error >= APP_FOLLOW_LARGE_ERROR) {
            base_speed = APP_FOLLOW_BASE_SPEED * APP_FOLLOW_LARGE_SPEED_SCALE; // Slow aggressively on large errors so bends do not run wide.
            steer_limit = APP_FOLLOW_STEER_LIMIT; // Keep full steering authority during recovery.
            min_steer = APP_FOLLOW_LARGE_MIN_STEER; // Force enough steering before the car runs wide.
        } else if (abs_error >= APP_FOLLOW_MEDIUM_ERROR) {
            base_speed = APP_FOLLOW_BASE_SPEED * APP_FOLLOW_MEDIUM_SPEED_SCALE; // Slow earlier on medium errors.
            steer_limit = APP_FOLLOW_STEER_LIMIT * 0.75f; // Moderate correction before the error becomes large.
            min_steer = APP_FOLLOW_MEDIUM_MIN_STEER; // Make medium-error correction visible on the chassis.
        } else {
            steer_limit = APP_FOLLOW_STEER_LIMIT * 0.45f; // Keep tiny corrections gentle near the center line.
        }

        if (line_error > 0) {
            g_line_recover_direction = 1; // Remember the last valid line side for short lost-line recovery.
        } else if (line_error < 0) {
            g_line_recover_direction = -1; // Remember the last valid line side for short lost-line recovery.
        }

        if ((min_steer > 0.0f) && (correction < min_steer) && (correction > -min_steer)) {
            correction = (line_error > 0) ?
                         (APP_FOLLOW_STEER_SIGN * min_steer) :
                         (-APP_FOLLOW_STEER_SIGN * min_steer);
        }

        correction = App_LimitFloat(correction, steer_limit);
        correction = App_LimitFloatDelta(correction,
                                         g_line_last_correction,
                                         APP_LINE_CORRECTION_SLEW_LIMIT);
        left_cmd = base_speed + correction;
        right_cmd = base_speed - correction;
        App_LineApplySpeedPid(left_cmd, right_cmd);

        g_line_valid = 1U; // Mirror drive state for Watch/Expressions.
        g_line_lost_count = 0U; // Reset the simple CCD lost-line recovery counter after a valid target.
        g_line_target = Bsp_Ccd_GetTargetIndex(); // Mirror CCD target for Watch/Expressions.
        g_line_error = line_error; // Mirror fixed-center error for Watch/Expressions.
        g_line_error_delta = error_delta; // Mirror derivative input for Watch/Expressions.
        g_line_dx_max = Bsp_Ccd_GetDxMax(); // Mirror strongest positive edge for Watch/Expressions.
        g_line_dx_min = Bsp_Ccd_GetDxMin(); // Mirror strongest negative edge for Watch/Expressions.
        g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex(); // Mirror positive-edge index for Watch/Expressions.
        g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex(); // Mirror negative-edge index for Watch/Expressions.
        g_line_correction = correction; // Mirror limited CCD steering correction.
        g_line_left_cmd = left_cmd; // Mirror left command for Watch/Expressions.
        g_line_right_cmd = right_cmd; // Mirror right command for Watch/Expressions.
        g_line_last_error = line_error; // Store current error for the next derivative update.
        g_line_last_correction = correction; // Store smoothed steering for the next slew-limit step.
    }
#if APP_MODE == APP_MODE_SQUARE_FOLLOW
    else if (square_corner_handled == 0U) {
        g_line_recover_direction = APP_SQUARE_CORNER_DEFAULT_DIRECTION;
#else
    else {
#endif
        float left_cmd = 0.0f;
        float right_cmd = 0.0f;
        float correction = 0.0f;

        if ((g_line_lost_count < APP_FOLLOW_LOST_RECOVER_MAX) &&
            (g_line_recover_direction != 0)) {
            float base_speed = APP_FOLLOW_BASE_SPEED * APP_FOLLOW_LOST_SPEED_SCALE;

            correction = APP_FOLLOW_STEER_SIGN *
                         APP_FOLLOW_LOST_TURN *
                         (float) g_line_recover_direction;
            correction = App_LimitFloatDelta(correction,
                                             g_line_last_correction,
                                             APP_LINE_CORRECTION_SLEW_LIMIT);
            left_cmd = base_speed + correction;
            right_cmd = base_speed - correction;
            App_LineApplySpeedPid(left_cmd, right_cmd); // Preserve the frozen lost-line search ratio through wheel-speed PI.
        } else {
            Bsp_Motor_Stop(); // Brake after sustained line loss without re-arming the cold-start kick on brief reacquisition.
        }
        g_line_valid = 0U; // Mirror lost-line state for Watch/Expressions.
        g_line_lost_count++; // Count consecutive missing CCD targets for debugging only.
        g_line_target = -1; // Mirror lost-line target for Watch/Expressions.
        g_line_error = g_line_last_error; // Keep last valid error visible while bridging a short target dropout.
        g_line_error_delta = 0; // Clear derivative display while no target is available.
        g_line_dx_max = Bsp_Ccd_GetDxMax(); // Keep edge diagnostics visible when invalid.
        g_line_dx_min = Bsp_Ccd_GetDxMin(); // Keep edge diagnostics visible when invalid.
        g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex(); // Keep edge diagnostics visible when invalid.
        g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex(); // Keep edge diagnostics visible when invalid.
        g_line_correction = correction; // Mirror held steering correction during short target dropouts.
        g_line_left_cmd = left_cmd; // Mirror lost-line straight/stop command.
        g_line_right_cmd = right_cmd; // Mirror lost-line straight/stop command.
        g_line_last_correction = correction; // Keep lost-line recovery continuous with valid steering.
    }
    Bsp_Gpio_ToggleHeartbeat(); // Heartbeat confirms the CCD line-follow loop is running.
    delay_cycles(APP_FOLLOW_LOOP_DELAY);
#elif APP_MODE == APP_MODE_CCD_STRAIGHT
    Bsp_Ccd_ReadFrame(); // Capture the latest line position before straight-only gating.
    Bsp_Ccd_Process(); // Detect whether a valid black line exists without using error for steering.
    if (Bsp_Ccd_IsLineValid() != 0U) {
        Bsp_Motor_Set(APP_CCD_STRAIGHT_SPEED, APP_CCD_STRAIGHT_SPEED); // Drive both wheels equally to isolate motor mismatch from steering logic.
        g_line_valid = 1U; // Mirror drive state for Watch/Expressions.
        g_line_target = Bsp_Ccd_GetTargetIndex(); // Mirror CCD target while straight-only driving.
        g_line_error = Bsp_Ccd_GetLineError(); // Mirror CCD error without feeding it back into motor commands.
        g_line_dx_max = Bsp_Ccd_GetDxMax(); // Mirror strongest positive edge for Watch/Expressions.
        g_line_dx_min = Bsp_Ccd_GetDxMin(); // Mirror strongest negative edge for Watch/Expressions.
        g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex(); // Mirror positive-edge index for Watch/Expressions.
        g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex(); // Mirror negative-edge index for Watch/Expressions.
        g_line_correction = 0.0f; // Mirror zero correction because straight-only mode does not steer.
        g_line_left_cmd = APP_CCD_STRAIGHT_SPEED; // Mirror fixed left command for Watch/Expressions.
        g_line_right_cmd = APP_CCD_STRAIGHT_SPEED; // Mirror fixed right command for Watch/Expressions.
    } else {
        Bsp_Motor_Stop(); // Stop immediately when CCD does not see a valid line.
        g_line_valid = 0U; // Mirror lost-line state for Watch/Expressions.
        g_line_target = -1; // Mirror lost-line target for Watch/Expressions.
        g_line_error = 0; // Mirror lost-line error for Watch/Expressions.
        g_line_dx_max = Bsp_Ccd_GetDxMax(); // Keep last edge strength visible even when the line is rejected.
        g_line_dx_min = Bsp_Ccd_GetDxMin(); // Keep last edge strength visible even when the line is rejected.
        g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex(); // Keep last edge index visible even when the line is rejected.
        g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex(); // Keep last edge index visible even when the line is rejected.
        g_line_correction = 0.0f; // Mirror stopped correction for Watch/Expressions.
        g_line_left_cmd = 0.0f; // Mirror stopped left command for Watch/Expressions.
        g_line_right_cmd = 0.0f; // Mirror stopped right command for Watch/Expressions.
    }
    Bsp_Gpio_ToggleHeartbeat(); // LED heartbeat confirms the straight-only CCD loop is running.
    delay_cycles(APP_LINE_LOOP_DELAY);
#elif APP_MODE == APP_MODE_ENCODER_WATCH
    Bsp_Motor_EncoderSample(); // Update per-window encoder speeds without driving the motors.
    g_encoder_left_count = Bsp_Motor_GetLeftEncoderCount(); // Mirror logical-left accumulated ticks for Watch/Expressions.
    g_encoder_right_count = Bsp_Motor_GetRightEncoderCount(); // Mirror logical-right accumulated ticks for Watch/Expressions.
    g_encoder_left_speed = Bsp_Motor_GetLeftEncoderSpeed(); // Mirror logical-left sample ticks for Watch/Expressions.
    g_encoder_right_speed = Bsp_Motor_GetRightEncoderSpeed(); // Mirror logical-right sample ticks for Watch/Expressions.
    g_speed_left_target = 0.0f; // Encoder-watch mode has no speed target.
    g_speed_right_target = 0.0f; // Encoder-watch mode has no speed target.
    g_speed_left_cmd = 0.0f; // Encoder-watch mode keeps motor output disabled.
    g_speed_right_cmd = 0.0f; // Encoder-watch mode keeps motor output disabled.
    Bsp_Gpio_ToggleHeartbeat(); // Heartbeat confirms encoder sampling is alive.
    delay_cycles(APP_ENCODER_WATCH_DELAY);
#elif APP_MODE == APP_MODE_SPEED_TEST
    static uint32_t speed_test_loop = 0U;

    if (speed_test_loop == 0U) {
        Bsp_Motor_SpeedPidStop(); // Reset output, integral, derivative history and target before each test cycle.
        Bsp_Motor_EncoderReset(); // Restart both corrected encoder channels from comparable totals.
    }

    if (speed_test_loop < APP_SPEED_TEST_WARMUP_LOOPS) {
        g_speed_test_phase = 0U;
        Bsp_Motor_SetSpeedTargets(0.0f, 0.0f); // Phase 0: both wheels stopped for five seconds.
    } else if (speed_test_loop < (APP_SPEED_TEST_WARMUP_LOOPS + APP_SPEED_TEST_HOLD_LOOPS)) {
        g_speed_test_phase = 1U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_LOW_CM_S, APP_SPEED_TEST_LOW_CM_S); // Phase 1: hold 20 cm/s for five seconds.
    } else if (speed_test_loop < (APP_SPEED_TEST_WARMUP_LOOPS + (2U * APP_SPEED_TEST_HOLD_LOOPS))) {
        g_speed_test_phase = 2U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_HIGH_CM_S, APP_SPEED_TEST_HIGH_CM_S); // Phase 2: step to 30 cm/s for five seconds.
    } else {
        g_speed_test_phase = 3U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_LOW_CM_S, APP_SPEED_TEST_LOW_CM_S); // Phase 3: return to 20 cm/s for five seconds.
    }

    Bsp_Motor_SpeedPidUpdate(); // Run the 2025-derived two-wheel speed controller with corrected channel pairing.
    g_speed_test_loop = speed_test_loop;
    speed_test_loop++;
    if (speed_test_loop >= (APP_SPEED_TEST_WARMUP_LOOPS + (3U * APP_SPEED_TEST_HOLD_LOOPS))) {
        speed_test_loop = 0U; // Repeat stop/20/30/20 so transitions remain easy to observe.
    }
    g_encoder_left_count = Bsp_Motor_GetLeftEncoderCount(); // Mirror logical-left accumulated ticks for Watch/Expressions.
    g_encoder_right_count = Bsp_Motor_GetRightEncoderCount(); // Mirror logical-right accumulated ticks for Watch/Expressions.
    g_encoder_left_speed = Bsp_Motor_GetLeftEncoderSpeed(); // Mirror logical-left measured ticks per sample.
    g_encoder_right_speed = Bsp_Motor_GetRightEncoderSpeed(); // Mirror logical-right measured ticks per sample.
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
    Bsp_Gpio_ToggleHeartbeat(); // Heartbeat confirms the repeating 2025-derived speed test is running.
    delay_cycles(APP_SPEED_TEST_LOOP_DELAY);
#elif APP_MODE == APP_MODE_UART_TEST
    static uint32_t uart_test_count = 0U;
    Bsp_Uart_K230_SendString("UART_TEST,count="); // Periodic marker for COM-port bringup.
    App_SendUint32(uart_test_count);
    Bsp_Uart_K230_SendString("\r\n");
    uart_test_count++;
    Bsp_Gpio_ToggleHeartbeat(); // LED heartbeat confirms firmware is running even if UART is not visible.
    delay_cycles(APP_DELAY_1S);
#else
    Bsp_Gpio_ToggleHeartbeat();
    delay_cycles(32000000);
#endif
}
