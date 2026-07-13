#include "app_config.h"
#include "app_main.h"
#include "ti_msp_dl_config.h"

#if (APP_MODE == APP_MODE_UART_TEST) || (APP_MODE == APP_MODE_CCD_WATCH)
static void Main_SafeDebugSysInit(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_SYSCTL_init();

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_K230_IOMUX_TX, GPIO_UART_K230_IOMUX_TX_FUNC); // Enable only the UART TX pin for COM-port bringup.
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_K230_IOMUX_RX, GPIO_UART_K230_IOMUX_RX_FUNC); // Enable only the UART RX pin for COM-port bringup.

    DL_GPIO_initDigitalOutput(GPIO_LEDS_USER_LED_1_IOMUX); // Keep the heartbeat LED available in UART test mode.
    DL_GPIO_initDigitalOutput(GPIO_DC_AIN0_IOMUX); // Force motor direction input low before the motor board is powered.
    DL_GPIO_initDigitalOutput(GPIO_DC_AIN2_IOMUX); // Force motor direction input low before the motor board is powered.
    DL_GPIO_initDigitalOutput(GPIO_PWM_DC_C0_IOMUX); // Hold PWM input as GPIO low instead of enabling timer PWM.
    DL_GPIO_initDigitalOutput(GPIO_PWM_DC_C1_IOMUX); // Hold PWM input as GPIO low instead of enabling timer PWM.
    DL_GPIO_initDigitalOutput(GPIO_CCD_SI_IOMUX); // Keep CCD SI available without enabling motor PWM.
    DL_GPIO_initDigitalOutput(GPIO_CCD_CLK_IOMUX); // Keep CCD CLK available without enabling motor PWM.

    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    DL_GPIO_enableOutput(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    DL_GPIO_clearPins(GPIO_DC_PORT, GPIO_DC_AIN0_PIN | GPIO_DC_AIN2_PIN);
    DL_GPIO_enableOutput(GPIO_DC_PORT, GPIO_DC_AIN0_PIN | GPIO_DC_AIN2_PIN);
    DL_GPIO_clearPins(GPIO_PWM_DC_C0_PORT, GPIO_PWM_DC_C0_PIN | GPIO_PWM_DC_C1_PIN);
    DL_GPIO_enableOutput(GPIO_PWM_DC_C0_PORT, GPIO_PWM_DC_C0_PIN | GPIO_PWM_DC_C1_PIN);
    DL_GPIO_clearPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN);
    DL_GPIO_enableOutput(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN);
    DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
    DL_GPIO_enableOutput(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);

    SYSCFG_DL_UART_K230_init();
#if APP_MODE == APP_MODE_CCD_WATCH
    SYSCFG_DL_ADC12_0_init(); // Enable ADC only when the debugger-watch CCD mode needs pixel samples.
#endif
}
#endif

int main(void)
{
#if (APP_MODE == APP_MODE_UART_TEST) || (APP_MODE == APP_MODE_CCD_WATCH)
    Main_SafeDebugSysInit(); // Skip full SysConfig init so PWM is never enabled during safe debug modes.
#else
    SYSCFG_DL_init();
#endif
    App_Init();

    while (1) {
        App_Loop();
    }
}
