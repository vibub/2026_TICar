#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

void Bsp_Gpio_Init(void);
void Bsp_Gpio_ToggleHeartbeat(void);
void Bsp_Gpio_SetHeartbeat(uint8_t on);

#endif
