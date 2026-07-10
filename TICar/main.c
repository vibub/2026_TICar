#include <stdint.h>

#include "ti_msp_dl_config.h"

#define UART_ERROR_MASK                                                     \
    ((uint32_t) (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                      \
                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                     \
                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR))

volatile uint32_t g_uartRxCount    = 0U;
volatile uint32_t g_uartLastRxData = 0U;
volatile uint32_t g_uartErrorCount = 0U;
volatile uint32_t g_uartLastError  = 0U;

static void UART_writeByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
        ;
    }

    DL_UART_Main_transmitData(UART_0_INST, data);
}

static void UART_write(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    for (index = 0U; index < length; index++) {
        UART_writeByte(data[index]);
    }
}

static void UART_flush(void)
{
    while (DL_UART_Main_isBusy(UART_0_INST)) {
        ;
    }
}

int main(void)
{
    static const uint8_t readyMessage[] =
        "TICar UART0 ready 115200 8N1\r\n";

    SYSCFG_DL_init();
    DL_UART_Main_clearInterruptStatus(UART_0_INST, UART_ERROR_MASK);
    UART_write(readyMessage, sizeof(readyMessage) - 1U);
    UART_flush();

    while (1) {
        uint8_t data = DL_UART_Main_receiveDataBlocking(UART_0_INST);
        uint32_t errorStatus = DL_UART_Main_getRawInterruptStatus(
            UART_0_INST, UART_ERROR_MASK);

        if (errorStatus != 0U) {
            g_uartLastError = errorStatus;
            g_uartErrorCount++;
            DL_UART_Main_clearInterruptStatus(
                UART_0_INST, errorStatus);
            continue;
        }

        g_uartLastRxData = data;
        g_uartRxCount++;
        UART_writeByte(data);
    }
}
