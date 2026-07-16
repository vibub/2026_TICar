#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

void Bsp_Uart_Init(void);
void Bsp_Uart_K230_SendByte(uint8_t byte);
void Bsp_Uart_K230_SendString(const char *str);
uint8_t Bsp_Uart_K230_ReceiveByteBlocking(void);
uint8_t Bsp_Uart_K230_TryReceiveByte(uint8_t *byte);
uint32_t Bsp_Uart_K230_GetErrorStatus(void);
void Bsp_Uart_K230_ClearErrorStatus(uint32_t status);

void Bsp_Uart_Tjc_SendByte(uint8_t byte);
void Bsp_Uart_Tjc_SendData(const uint8_t *data, uint16_t length);
uint8_t Bsp_Uart_Tjc_TryReceiveByte(uint8_t *byte);
void Bsp_Uart_Tjc_FlushRx(void);
uint32_t Bsp_Uart_Tjc_GetOverflowCount(void);
uint32_t Bsp_Uart_Tjc_GetErrorCount(void);

#endif
