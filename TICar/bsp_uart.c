#include "bsp_uart.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define BSP_UART_K230_ERROR_MASK                                           \
    ((uint32_t) (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                      \
                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                     \
                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR))
#define BSP_UART_TJC_RX_BUFFER_SIZE 16U

static volatile uint8_t g_tjc_rx_buffer[BSP_UART_TJC_RX_BUFFER_SIZE];
static volatile uint8_t g_tjc_rx_head;
static volatile uint8_t g_tjc_rx_tail;
static volatile uint32_t g_tjc_rx_overflow_count;
static volatile uint32_t g_tjc_rx_error_count;

static void Bsp_Uart_Tjc_PushRx(uint8_t byte)
{
    uint8_t next_head = (uint8_t) ((g_tjc_rx_head + 1U) % BSP_UART_TJC_RX_BUFFER_SIZE);

    if (next_head == g_tjc_rx_tail) {
        g_tjc_rx_overflow_count++;
        return;
    }

    g_tjc_rx_buffer[g_tjc_rx_head] = byte;
    g_tjc_rx_head = next_head;
}

void Bsp_Uart_Init(void)
{
    g_tjc_rx_head = 0U;
    g_tjc_rx_tail = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;
    NVIC_ClearPendingIRQ(UART_TJC3224T124_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TJC3224T124_INST_INT_IRQN);
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

uint8_t Bsp_Uart_K230_ReceiveByteBlocking(void)
{
    return DL_UART_Main_receiveDataBlocking(UART_K230_INST);
}

uint8_t Bsp_Uart_K230_TryReceiveByte(uint8_t *byte)
{
    if (byte == NULL) {
        return 0U;
    }

    return DL_UART_Main_receiveDataCheck(UART_K230_INST, byte) ? 1U : 0U;
}

uint32_t Bsp_Uart_K230_GetErrorStatus(void)
{
    return DL_UART_Main_getRawInterruptStatus(UART_K230_INST, BSP_UART_K230_ERROR_MASK);
}

void Bsp_Uart_K230_ClearErrorStatus(uint32_t status)
{
    DL_UART_Main_clearInterruptStatus(UART_K230_INST, status);
}

void Bsp_Uart_Tjc_SendByte(uint8_t byte)
{
    while (DL_UART_Main_isTXFIFOFull(UART_TJC3224T124_INST)) {
    }

    DL_UART_Main_transmitData(UART_TJC3224T124_INST, byte);
}

void Bsp_Uart_Tjc_SendData(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == NULL) {
        return;
    }

    for (i = 0U; i < length; i++) {
        Bsp_Uart_Tjc_SendByte(data[i]);
    }
}

uint8_t Bsp_Uart_Tjc_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_tjc_rx_tail == g_tjc_rx_head)) {
        return 0U;
    }

    *byte = g_tjc_rx_buffer[g_tjc_rx_tail];
    g_tjc_rx_tail = (uint8_t) ((g_tjc_rx_tail + 1U) % BSP_UART_TJC_RX_BUFFER_SIZE);
    return 1U;
}

void Bsp_Uart_Tjc_FlushRx(void)
{
    g_tjc_rx_tail = g_tjc_rx_head;
}

uint32_t Bsp_Uart_Tjc_GetOverflowCount(void)
{
    return g_tjc_rx_overflow_count;
}

uint32_t Bsp_Uart_Tjc_GetErrorCount(void)
{
    return g_tjc_rx_error_count;
}

void UART_TJC3224T124_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    while ((interrupt_index = DL_UART_Main_getPendingInterrupt(UART_TJC3224T124_INST)) !=
           DL_UART_MAIN_IIDX_NO_INTERRUPT) {
        switch (interrupt_index) {
            case DL_UART_MAIN_IIDX_RX:
                while (!DL_UART_isRXFIFOEmpty(UART_TJC3224T124_INST)) {
                    Bsp_Uart_Tjc_PushRx(
                        DL_UART_Main_receiveDataBlocking(UART_TJC3224T124_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                g_tjc_rx_error_count++;
                while (!DL_UART_isRXFIFOEmpty(UART_TJC3224T124_INST)) {
                    (void) DL_UART_Main_receiveDataBlocking(UART_TJC3224T124_INST);
                }
                break;

            default:
                break;
        }
    }
}
