#include <stddef.h>
#include "bsp_uart.h"

#include "ti_msp_dl_config.h"

#define BSP_UART_K230_ERROR_MASK                                           \
    ((uint32_t) (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                      \
                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                     \
                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR))

void Bsp_Uart_Init(void)
{
}

void Bsp_Uart_K230_SendByte(uint8_t byte)
{
    while (DL_UART_Main_isTXFIFOFull(UART_K230_INST)) {
    }

    DL_UART_Main_transmitData(UART_K230_INST, byte);
}

void Bsp_Uart_K230_SendString(const char *str)
{
    while (*str != '\0') {
        Bsp_Uart_K230_SendByte((uint8_t) *str);
        str++;
    }
}

uint8_t Bsp_Uart_K230_TryReceiveByte(uint8_t *byte)
{
    if (byte == NULL) {
        return 0U;
    }

    if (DL_UART_Main_receiveDataCheck(UART_K230_INST, byte)) {
        return 1U;
    }

    return 0U;
}

uint8_t Bsp_Uart_K230_ReceiveByteBlocking(void)
{
    return DL_UART_Main_receiveDataBlocking(UART_K230_INST);
}

uint32_t Bsp_Uart_K230_GetErrorStatus(void)
{
    return DL_UART_Main_getRawInterruptStatus(UART_K230_INST, BSP_UART_K230_ERROR_MASK);
}

void Bsp_Uart_K230_ClearErrorStatus(uint32_t status)
{
    DL_UART_Main_clearInterruptStatus(UART_K230_INST, status);
}
