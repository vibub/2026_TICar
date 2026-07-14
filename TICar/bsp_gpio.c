#include "bsp_gpio.h"

#include "ti_msp_dl_config.h"

#define BSP_GPIO_CORNER_LED_PORT GPIOB // Use LP-MSPM0G3507 LED2 red channel instead of the blue heartbeat LED.
#define BSP_GPIO_CORNER_LED_PIN DL_GPIO_PIN_26 // PB26 drives the LaunchPad RGB red LED channel.
#define BSP_GPIO_CORNER_LED_IOMUX IOMUX_PINCM57 // PINCM57 maps to PB26 GPIO output on MSPM0G3507.
#define BSP_GPIO_CORNER_BLINK_DIVIDER 20U // Toggle every twenty line-loop updates so the blink is visible on short corner events.

static uint8_t s_corner_led_blink_count = 0U; // Count line-loop updates between corner indicator toggles.
static uint8_t s_corner_led_state = 0U; // Track the current red LED state because the indicator is software-blinked.

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
