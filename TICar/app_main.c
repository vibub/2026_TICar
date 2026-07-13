#include "app_main.h"

#include "app_config.h"
#include "bsp_ccd.h"
#include "bsp_gpio.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "pid.h"
#include "protocol_k230.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

#define APP_DELAY_1S CPUCLK_FREQ        // 1 second at the configured CPU clock.
#define APP_DELAY_2S (2U * CPUCLK_FREQ) // 2 seconds at the configured CPU clock.
#define APP_DELAY_PAUSE CPUCLK_FREQ     // Pause long enough to see active brake behavior.
#define APP_STRAIGHT_TEST_SPEED 0.45f   // Start slower so left/right drift is easier to observe.
#define APP_CCD_PRINT_DELAY (CPUCLK_FREQ / 2U) // Print CCD frames at 2 Hz so UART output stays readable.
#define APP_CCD_LOG_FRAME_COUNT 32U // Keep a small RAM log that can be exported through the debugger.
#define APP_LINE_BASE_SPEED 0.20f // First map-run speed is intentionally low for safe bringup.
#define APP_LINE_KP 0.0040f // Steering gain converts CCD pixel error into left/right wheel speed difference.
#define APP_LINE_STEER_LIMIT 0.18f // Limit correction so one bad frame cannot command a sharp turn.
#define APP_LINE_LOOP_DELAY (CPUCLK_FREQ / 100U) // Run line-follow updates around 100 Hz.

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
volatile uint16_t g_debug_ccd_raw[BSP_CCD_PIXEL_COUNT]; // Watch this array to inspect all 128 raw CCD pixels.
volatile uint16_t g_debug_ccd_log_write_index = 0U; // Watch this value to know which RAM log slot will be written next.
volatile uint16_t g_debug_ccd_log_filled = 0U; // Watch this value; 32 means the RAM log has one full buffer.
volatile App_CcdLogFrame g_debug_ccd_log[APP_CCD_LOG_FRAME_COUNT]; // Export this RAM block through CCS Memory Browser for CSV conversion.
#endif

#if APP_MODE == APP_MODE_LINE_FOLLOW
volatile uint8_t g_line_valid = 0U; // Watch this value; 1 means line-follow is actively driving.
volatile int16_t g_line_target = -1; // Watch this value for the latest CCD target used by steering.
volatile int16_t g_line_error = 0; // Watch this value for the latest centered CCD error.
volatile float g_line_left_cmd = 0.0f; // Watch this value for the commanded logical left motor ratio.
volatile float g_line_right_cmd = 0.0f; // Watch this value for the commanded logical right motor ratio.
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

void App_Init(void)
{
    Bsp_Gpio_Init();
    Bsp_Uart_Init();
#if (APP_MODE == APP_MODE_MOTOR_PWM) || (APP_MODE == APP_MODE_LINE_FOLLOW)
    Bsp_Motor_Init();
#else
    Bsp_Motor_Disable(); // Non-motor debug modes must keep PWM and H-bridge outputs idle.
#endif
#if APP_MODE == APP_MODE_CCD_ADC
    Bsp_Uart_K230_SendString("BOOT,CCD\r\n"); // Early UART marker proves the selected COM port is receiving firmware output.
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC before the first frame read.
#elif APP_MODE == APP_MODE_CCD_WATCH
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC for debugger-watch CCD sampling.
#elif APP_MODE == APP_MODE_LINE_FOLLOW
    Bsp_Ccd_Init(); // Prepare CCD SI/CLK pins and ADC before closed-loop line following.
#elif APP_MODE == APP_MODE_UART_TEST
    Bsp_Uart_K230_SendString("BOOT,UART_TEST\r\n"); // Early marker proves the selected COM port is receiving firmware output.
#endif
    Protocol_K230_Init();
    PID_InitDefaults();
}

void App_Loop(void)
{
#if APP_MODE == APP_MODE_HEARTBEAT
    Bsp_Gpio_ToggleHeartbeat();
    delay_cycles(16000000);
#elif APP_MODE == APP_MODE_K230_UART
    Protocol_K230_Task();
#elif APP_MODE == APP_MODE_MOTOR_PWM
    Bsp_Motor_Set(APP_STRAIGHT_TEST_SPEED, APP_STRAIGHT_TEST_SPEED); // Forward straight-line test.
    delay_cycles(APP_DELAY_2S);
    Bsp_Motor_Stop();
    delay_cycles(APP_DELAY_PAUSE);

    Bsp_Motor_Set(-APP_STRAIGHT_TEST_SPEED, -APP_STRAIGHT_TEST_SPEED); // Reverse straight-line test.
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
#elif APP_MODE == APP_MODE_LINE_FOLLOW
    Bsp_Ccd_ReadFrame(); // Capture the latest line position before each steering update.
    Bsp_Ccd_Process(); // Convert CCD pixels into a valid flag and centered error.
    if (Bsp_Ccd_IsLineValid() != 0U) {
        float correction = App_LimitFloat(APP_LINE_KP * (float) Bsp_Ccd_GetLineError(),
                                          APP_LINE_STEER_LIMIT);
        float left_cmd = APP_LINE_BASE_SPEED + correction; // Positive error means line is right, so left wheel speeds up.
        float right_cmd = APP_LINE_BASE_SPEED - correction; // Positive error means line is right, so right wheel slows down.
        Bsp_Motor_Set(left_cmd, right_cmd);
        g_line_valid = 1U; // Mirror drive state for Watch/Expressions.
        g_line_target = Bsp_Ccd_GetTargetIndex(); // Mirror target for Watch/Expressions.
        g_line_error = Bsp_Ccd_GetLineError(); // Mirror error for Watch/Expressions.
        g_line_left_cmd = left_cmd; // Mirror left command for Watch/Expressions.
        g_line_right_cmd = right_cmd; // Mirror right command for Watch/Expressions.
    } else {
        Bsp_Motor_Stop(); // Stop immediately when the line is lost during first map runs.
        g_line_valid = 0U; // Mirror lost-line state for Watch/Expressions.
        g_line_target = -1; // Mirror lost-line target for Watch/Expressions.
        g_line_error = 0; // Mirror lost-line error for Watch/Expressions.
        g_line_left_cmd = 0.0f; // Mirror stopped left command for Watch/Expressions.
        g_line_right_cmd = 0.0f; // Mirror stopped right command for Watch/Expressions.
    }
    Bsp_Gpio_ToggleHeartbeat(); // LED heartbeat confirms the line-follow loop is running.
    delay_cycles(APP_LINE_LOOP_DELAY);
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
