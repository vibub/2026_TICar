/**
 * @file bsp_gpio.c
 * @brief 管理板载心跳 LED 和方形赛道直角状态指示灯。
 *
 * PB26 红色 LED 由代码直接配置；角点闪烁依赖巡线任务的固定调用周期。
 */
#include "bsp_gpio.h"

#include "ti_msp_dl_config.h"

/* LP-MSPM0G3507 LED2 红色通道：PB26/PINCM57，与蓝色心跳灯分离。 */
#define BSP_GPIO_CORNER_LED_PORT GPIOB // Use LP-MSPM0G3507 LED2 red channel instead of the blue heartbeat LED.
#define BSP_GPIO_CORNER_LED_PIN DL_GPIO_PIN_26 // PB26 drives the LaunchPad RGB red LED channel.
#define BSP_GPIO_CORNER_LED_IOMUX IOMUX_PINCM57 // PINCM57 maps to PB26 GPIO output on MSPM0G3507.
#define BSP_GPIO_CORNER_BLINK_DIVIDER 20U // Toggle every twenty line-loop updates so the blink is visible on short corner events.

/* 软件分频计数和当前输出状态；20 次 × 20 ms 约每 400 ms 翻转一次。 */
static uint8_t s_corner_led_blink_count = 0U; // Count line-loop updates between corner indicator toggles.
static uint8_t s_corner_led_state = 0U; // Track the current red LED state because the indicator is software-blinked.

/* PB26 未配置在 SysConfig 中，因此在 BSP 初始化时手工设为默认关闭的数字输出。 */
void Bsp_Gpio_Init(void)
{
    DL_GPIO_initDigitalOutput(BSP_GPIO_CORNER_LED_IOMUX); // Configure PB26 red LED without editing SysConfig.
    DL_GPIO_clearPins(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Keep the corner indicator off after reset.
    DL_GPIO_enableOutput(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Enable the red LED GPIO output.
    Bsp_Gpio_SetHeartbeat(1);
}

void Bsp_Gpio_ToggleHeartbeat(void)
{
    DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
}

void Bsp_Gpio_SetHeartbeat(uint8_t on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    }
}

void Bsp_Gpio_SetCornerIndicator(uint8_t on)
{
    s_corner_led_state = (on != 0U) ? 1U : 0U; // Store the commanded indicator state for blink continuity.
    s_corner_led_blink_count = 0U; // Restart blink timing whenever the indicator is forced.
    if (on != 0U) {
        DL_GPIO_setPins(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Red LED is active-high on LP-MSPM0G3507.
    } else {
        DL_GPIO_clearPins(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Keep red LED off during straight-line mode.
    }
}

/*
 * 正常巡线时强制熄灭；进入角点后立即点亮，再按调用次数分频闪烁。
 * 该频率依赖应用按 20 ms 周期调用，不是独立的硬件定时器。
 */
void Bsp_Gpio_UpdateCornerIndicator(uint8_t corner_active)
{
    if (corner_active == 0U) {
        Bsp_Gpio_SetCornerIndicator(0U); // Straight mode keeps the corner indicator fully off.
        return;
    }

    if ((s_corner_led_blink_count == 0U) && (s_corner_led_state == 0U)) {
        s_corner_led_state = 1U; // Turn on immediately when corner mode starts so short turns are visible.
        DL_GPIO_setPins(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Apply the immediate active-high red LED state.
    }

    s_corner_led_blink_count++; // Advance the software blink divider once per line-follow loop.
    if (s_corner_led_blink_count >= BSP_GPIO_CORNER_BLINK_DIVIDER) {
        s_corner_led_blink_count = 0U; // Restart the blink divider after each visible transition.
        s_corner_led_state ^= 1U; // Toggle the stored red LED state for corner-mode blinking.
        if (s_corner_led_state != 0U) {
            DL_GPIO_setPins(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Set red LED on for the next blink phase.
        } else {
            DL_GPIO_clearPins(BSP_GPIO_CORNER_LED_PORT, BSP_GPIO_CORNER_LED_PIN); // Set red LED off for the next blink phase.
        }
    }
}
