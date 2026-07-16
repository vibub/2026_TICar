#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

void App_Init(void);
void App_Loop(void);
uint8_t App_RequestMode(uint8_t mode);
uint8_t App_GetCurrentMode(void);

#endif
