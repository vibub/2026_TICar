# 26TICup_MSPM0_Bringup

This is the clean MSPM0G3507 bringup project for the 2026 TI Cup car.

## First CCS Import

Import this project from:

`D:\2026TI_Cup\26TICup_MSPM0_Bringup\ticlang\26TICup_MSPM0_Bringup.projectspec`

Use CCS Theia with the MSPM0 SDK installed.

## Current Test

The current firmware is only a heartbeat test:

- `main.c` calls `SYSCFG_DL_init()`, `App_Init()`, then `App_Loop()`.
- `app/app_main.c` toggles `USER_LED_1`.
- `M0_Bringup.syscfg` only configures `PB22` as `GPIO_LEDS.USER_LED_1`.

If this builds and flashes, the new project base is healthy.

## Bringup Order

1. Heartbeat LED
2. UART to PC or K230
3. Motor PWM only
4. Motor direction pins
5. Encoder capture
6. CCD ADC, SI, CLK
7. PTZ PWM
8. K230 target protocol
9. PID integration

Keep each step independently testable.
