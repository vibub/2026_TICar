#include "bsp_gpio.h"

#include "ti_msp_dl_config.h"

void Bsp_Gpio_Init(void)
{
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
