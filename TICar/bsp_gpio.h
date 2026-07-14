#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

void Bsp_Gpio_Init(void);
void Bsp_Gpio_ToggleHeartbeat(void);
void Bsp_Gpio_SetHeartbeat(uint8_t on);
void Bsp_Gpio_SetCornerIndicator(uint8_t on); // Drive the dedicated corner-mode indicator LED directly.
void Bsp_Gpio_UpdateCornerIndicator(uint8_t corner_active); // Blink the corner indicator only while corner mode is active.

#endif
