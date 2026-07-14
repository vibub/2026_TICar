#include "bsp_ccd.h"

#include <stdint.h>
#include "bsp_uart.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

#define BSP_CCD_ADC_DONE_MASK DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED
#define BSP_CCD_ADC_WAIT_TIMEOUT 10000U
#define BSP_CCD_TARGET_MIN_EDGE_GAP 10U
#define BSP_CCD_MIN_CONTRAST 80U
#define BSP_CCD_WEAK_EDGE_DELTA 25
#define BSP_CCD_MIN_BLACK_WIDTH 3U
#define BSP_CCD_IGNORE_EDGE_PIXELS 16U
#define BSP_CCD_THRESHOLD_NUMERATOR 45U
#define BSP_CCD_THRESHOLD_DENOMINATOR 100U

static uint16_t g_ccd_raw[BSP_CCD_PIXEL_COUNT];
static uint16_t g_ccd_filtered[BSP_CCD_PIXEL_COUNT];
static int16_t g_ccd_dx[BSP_CCD_PIXEL_COUNT];
static int16_t g_ccd_target_index = -1;
static int16_t g_ccd_line_error = 0;
static int16_t g_ccd_dx_max = 0;
static int16_t g_ccd_dx_min = 0;
static uint16_t g_ccd_dx_max_index = 0U;
static uint16_t g_ccd_dx_min_index = 0U;
static uint8_t g_ccd_line_valid = 0U;
static uint16_t g_ccd_raw_min = 0U;
static uint16_t g_ccd_raw_max = 0U;
static uint16_t g_ccd_threshold = 0U;
static int16_t g_ccd_black_left = -1;
static int16_t g_ccd_black_right = -1;
static uint16_t g_ccd_contrast = 0U;
static uint16_t g_ccd_black_width = 0U;
static uint16_t g_ccd_raw_min_index = 0U;
static uint16_t g_ccd_raw_max_index = 0U;
static int16_t g_ccd_last_valid_target = BSP_CCD_CENTER_INDEX;
static uint8_t g_ccd_has_last_valid_target = 0U;

static void Ccd_DelayUs(uint16_t us)
{
    while (us != 0U) {
        delay_cycles(CPUCLK_FREQ / 1000000U); // One microsecond delay at current CPU clock.
        us--;
    }
}

static void Ccd_SetSi(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_setPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN); // SI high starts a new CCD integration/output sequence.
    } else {
        DL_GPIO_clearPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN); // SI low returns the CCD to normal pixel shifting.
    }
}

static void Ccd_SetClk(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_setPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN); // CLK rising edge shifts the CCD pixel pipeline.
    } else {
        DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN); // CLK low completes one CCD shift pulse.
    }
}

static void Ccd_PulseClk(uint16_t high_us, uint16_t low_us)
{
    Ccd_SetClk(1U);
    Ccd_DelayUs(high_us);
    Ccd_SetClk(0U);
    Ccd_DelayUs(low_us);
}

static uint16_t Ccd_ReadAdcSample(void)
{
    uint32_t timeout = BSP_CCD_ADC_WAIT_TIMEOUT;
    uint16_t sample;

    DL_ADC12_clearInterruptStatus(ADC12_0_INST, BSP_CCD_ADC_DONE_MASK); // Clear stale ADC completion flags before a new sample.
    DL_ADC12_startConversion(ADC12_0_INST);

    while ((DL_ADC12_getRawInterruptStatus(ADC12_0_INST, BSP_CCD_ADC_DONE_MASK) == 0U) &&
           (timeout != 0U)) {
        timeout--;
    }

    sample = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0); // Read MEM0 before clearing the completion flag.
    DL_ADC12_clearInterruptStatus(ADC12_0_INST, BSP_CCD_ADC_DONE_MASK); // Clear the completion flag for the next pixel.
    DL_ADC12_enableConversions(ADC12_0_INST); // Re-arm ADC conversions after each software-triggered sample.

    return sample;
}

static void Ccd_Flush(void)
{
    uint16_t i;

    Ccd_SetSi(1U);
    Ccd_DelayUs(1U);
    Ccd_SetClk(1U);
    Ccd_DelayUs(1U);
    Ccd_SetSi(0U);
    Ccd_DelayUs(1U);
    Ccd_SetClk(0U);
    Ccd_DelayUs(1U);

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        Ccd_PulseClk(2U, 2U); // Flush one full CCD line before reading fresh pixels.
    }
}

static uint16_t Ccd_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t min_val = a;
    uint16_t max_val = a;

    if (b < min_val) {
        min_val = b;
    }
    if (c < min_val) {
        min_val = c;
    }
    if (b > max_val) {
        max_val = b;
    }
    if (c > max_val) {
        max_val = c;
    }

    return (uint16_t) (a + b + c - min_val - max_val); // Median of three removes isolated CCD noise spikes.
}

static void Ccd_SendInt32(int32_t value)
{
    char buf[12];
    uint8_t idx = 0U;
    uint8_t start;
    uint32_t magnitude;

    if (value < 0) {
        Bsp_Uart_K230_SendByte((uint8_t) '-');
        magnitude = (uint32_t) (-value);
    } else {
        magnitude = (uint32_t) value;
    }

    do {
        buf[idx] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
        idx++;
    } while (magnitude != 0U);

    start = idx;
    while (start != 0U) {
        start--;
        Bsp_Uart_K230_SendByte((uint8_t) buf[start]);
    }
}

void Bsp_Ccd_Init(void)
{
    Ccd_SetSi(0U);
    Ccd_SetClk(0U);
    DL_ADC12_enableConversions(ADC12_0_INST); // Keep ADC ready for software-triggered CCD pixel samples.
}

void Bsp_Ccd_ReadFrame(void)
{
    uint16_t i;

    Ccd_Flush();
    Ccd_DelayUs(50U); // Short integration delay copied from the 2025 CCD bringup.

    Ccd_SetSi(1U);
    Ccd_DelayUs(1U);
    Ccd_SetClk(1U);
    Ccd_DelayUs(1U);
    Ccd_SetSi(0U);
    Ccd_DelayUs(1U);
    Ccd_SetClk(0U);
    Ccd_DelayUs(3U);

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_ccd_raw[i] = Ccd_ReadAdcSample(); // Sample the current pixel before shifting to the next pixel.
        Ccd_PulseClk(2U, 3U);
    }

    Ccd_PulseClk(2U, 0U); // 129th pulse terminates output of the 128th CCD pixel.
}

void Bsp_Ccd_Process(void)
{
    uint16_t i;
    uint16_t contrast;

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        uint16_t left = (i == 0U) ? g_ccd_raw[i] : g_ccd_raw[i - 1U];
        uint16_t right = (i == (BSP_CCD_PIXEL_COUNT - 1U)) ? g_ccd_raw[i] : g_ccd_raw[i + 1U];
        g_ccd_filtered[i] = Ccd_Median3(left, g_ccd_raw[i], right);
    }

    g_ccd_raw_min = g_ccd_filtered[0];
    g_ccd_raw_max = g_ccd_filtered[0];
    g_ccd_raw_min_index = 0U;
    g_ccd_raw_max_index = 0U;
    for (i = 1U; i < BSP_CCD_PIXEL_COUNT; i++) {
        if (g_ccd_filtered[i] < g_ccd_raw_min) {
            g_ccd_raw_min = g_ccd_filtered[i]; // Track darkest filtered pixel for adaptive thresholding.
            g_ccd_raw_min_index = i; // Track darkest pixel index to catch edge artifacts.
        }
        if (g_ccd_filtered[i] > g_ccd_raw_max) {
            g_ccd_raw_max = g_ccd_filtered[i]; // Track brightest filtered pixel for adaptive thresholding.
            g_ccd_raw_max_index = i; // Track brightest pixel index for CCD sanity checks.
        }
    }

    contrast = g_ccd_raw_max - g_ccd_raw_min;
    g_ccd_contrast = contrast;
    g_ccd_threshold = g_ccd_raw_min +
                      (uint16_t) (((uint32_t) contrast * BSP_CCD_THRESHOLD_NUMERATOR) /
                                  BSP_CCD_THRESHOLD_DENOMINATOR); // Pixels below this value are treated as black line candidates.

    g_ccd_dx_max = 0;
    g_ccd_dx_min = 0;
    g_ccd_dx_max_index = 0U;
    g_ccd_dx_min_index = 0U;
    for (i = 0U; i < (BSP_CCD_PIXEL_COUNT - 3U); i++) {
        g_ccd_dx[i] = (int16_t) g_ccd_filtered[i] - (int16_t) g_ccd_filtered[i + 3U]; // 2025 CCD algorithm: compare each pixel with the sample three positions ahead.
        if (g_ccd_dx[i] > g_ccd_dx_max) {
            g_ccd_dx_max = g_ccd_dx[i];
            g_ccd_dx_max_index = i; // Mirror 2025 MaxIdx for debugger inspection.
        }
        if (g_ccd_dx[i] < g_ccd_dx_min) {
            g_ccd_dx_min = g_ccd_dx[i];
            g_ccd_dx_min_index = i; // Mirror 2025 MinIdx for debugger inspection.
        }
    }

    if ((g_ccd_dx_max_index < g_ccd_dx_min_index) &&
        ((g_ccd_dx_min_index - g_ccd_dx_max_index) > BSP_CCD_TARGET_MIN_EDGE_GAP) &&
        (g_ccd_dx_max > 0) &&
        (g_ccd_dx_min < 0)) {
        g_ccd_line_valid = 1U;
        g_ccd_black_width = g_ccd_dx_min_index - g_ccd_dx_max_index + 1U;
        g_ccd_black_left = (int16_t) g_ccd_dx_max_index;
        g_ccd_black_right = (int16_t) g_ccd_dx_min_index;
        g_ccd_target_index = (int16_t) ((g_ccd_dx_max_index + g_ccd_dx_min_index) >> 1U); // Match 2025: center is midpoint of strongest positive/negative edges.
        g_ccd_last_valid_target = g_ccd_target_index;
        g_ccd_has_last_valid_target = 1U;
        g_ccd_line_error = g_ccd_target_index - BSP_CCD_CENTER_INDEX; // Positive error means the detected line is to the right.
    } else if (((g_ccd_dx_max - g_ccd_dx_min) < BSP_CCD_WEAK_EDGE_DELTA) &&
               (g_ccd_has_last_valid_target != 0U)) {
        g_ccd_line_valid = 1U;
        g_ccd_black_width = 0U;
        g_ccd_black_left = -1;
        g_ccd_black_right = -1;
        g_ccd_target_index = g_ccd_last_valid_target; // STM32-style continuity: keep last reliable center through weak CCD frames.
        g_ccd_line_error = g_ccd_target_index - BSP_CCD_CENTER_INDEX;
    } else {
        g_ccd_line_valid = 0U;
        g_ccd_black_width = 0U;
        g_ccd_black_left = -1;
        g_ccd_black_right = -1;
        g_ccd_target_index = -1; // -1 means no reliable black line was detected.
        g_ccd_line_error = 0;
    }
}

void Bsp_Ccd_PrintDebugFrame(void)
{
    uint16_t i;

    Bsp_Uart_K230_SendString("CCD,target=");
    Ccd_SendInt32(g_ccd_target_index);
    Bsp_Uart_K230_SendString(",error=");
    Ccd_SendInt32(g_ccd_line_error);
    Bsp_Uart_K230_SendString(",dxMax=");
    Ccd_SendInt32(g_ccd_dx_max);
    Bsp_Uart_K230_SendString(",dxMin=");
    Ccd_SendInt32(g_ccd_dx_min);
    Bsp_Uart_K230_SendString("\r\nRAW,");

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        Ccd_SendInt32(g_ccd_raw[i]);
        if (i + 1U < BSP_CCD_PIXEL_COUNT) {
            Bsp_Uart_K230_SendByte((uint8_t) ',');
        }
    }

    Bsp_Uart_K230_SendString("\r\n");
}

const uint16_t *Bsp_Ccd_GetRawFrame(void)
{
    return g_ccd_raw;
}

const uint16_t *Bsp_Ccd_GetFilteredFrame(void)
{
    return g_ccd_filtered;
}

int16_t Bsp_Ccd_GetTargetIndex(void)
{
    return g_ccd_target_index;
}

int16_t Bsp_Ccd_GetLineError(void)
{
    return g_ccd_line_error;
}

uint8_t Bsp_Ccd_IsLineValid(void)
{
    return g_ccd_line_valid;
}

uint16_t Bsp_Ccd_GetRawMin(void)
{
    return g_ccd_raw_min;
}

uint16_t Bsp_Ccd_GetRawMax(void)
{
    return g_ccd_raw_max;
}

uint16_t Bsp_Ccd_GetThreshold(void)
{
    return g_ccd_threshold;
}

int16_t Bsp_Ccd_GetBlackLeft(void)
{
    return g_ccd_black_left;
}

int16_t Bsp_Ccd_GetBlackRight(void)
{
    return g_ccd_black_right;
}

uint16_t Bsp_Ccd_GetContrast(void)
{
    return g_ccd_contrast;
}

uint16_t Bsp_Ccd_GetBlackWidth(void)
{
    return g_ccd_black_width;
}

uint16_t Bsp_Ccd_GetRawMinIndex(void)
{
    return g_ccd_raw_min_index;
}

uint16_t Bsp_Ccd_GetRawMaxIndex(void)
{
    return g_ccd_raw_max_index;
}

int16_t Bsp_Ccd_GetDxMax(void)
{
    return g_ccd_dx_max;
}

int16_t Bsp_Ccd_GetDxMin(void)
{
    return g_ccd_dx_min;
}

uint16_t Bsp_Ccd_GetDxMaxIndex(void)
{
    return g_ccd_dx_max_index;
}

uint16_t Bsp_Ccd_GetDxMinIndex(void)
{
    return g_ccd_dx_min_index;
}
