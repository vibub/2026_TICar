#include "protocol_tjc.h"

#include "app_main.h"
#include "bsp_uart.h"

volatile uint32_t g_tjc_rx_byte_count;
volatile uint32_t g_tjc_valid_command_count;
volatile uint32_t g_tjc_invalid_command_count;
volatile uint32_t g_tjc_response_count;
volatile uint32_t g_tjc_rx_overflow_count;
volatile uint32_t g_tjc_rx_error_count;
volatile uint8_t g_tjc_last_command;
volatile uint8_t g_tjc_last_result;

void Protocol_Tjc_Init(void)
{
    g_tjc_rx_byte_count = 0U;
    g_tjc_valid_command_count = 0U;
    g_tjc_invalid_command_count = 0U;
    g_tjc_response_count = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;
    g_tjc_last_command = TJC_COMMAND_STOP;
    g_tjc_last_result = TJC_RESULT_STATE;
    Bsp_Uart_Tjc_FlushRx();
}

void Protocol_Tjc_SendResult(Tjc_Result result, uint8_t current_mode, uint8_t request)
{
    uint8_t frame[5];

    frame[0] = TJC_RESPONSE_HEADER;
    frame[1] = (uint8_t) result;
    frame[2] = current_mode;
    frame[3] = request;
    frame[4] = (uint8_t) (frame[0] ^ frame[1] ^ frame[2] ^ frame[3]);
    Bsp_Uart_Tjc_SendData(frame, (uint16_t) sizeof(frame));
    g_tjc_last_result = (uint8_t) result;
    g_tjc_response_count++;
}

void Protocol_Tjc_Task(void)
{
    uint8_t command;

    g_tjc_rx_overflow_count = Bsp_Uart_Tjc_GetOverflowCount();
    g_tjc_rx_error_count = Bsp_Uart_Tjc_GetErrorCount();
    while (Bsp_Uart_Tjc_TryReceiveByte(&command) != 0U) {
        g_tjc_rx_byte_count++;
        g_tjc_last_command = command;

        if (command == TJC_COMMAND_QUERY) {
            g_tjc_valid_command_count++;
            Protocol_Tjc_SendResult(TJC_RESULT_STATE, App_GetCurrentMode(), command);
        } else if (command <= TJC_COMMAND_LAST_MODE) {
            g_tjc_valid_command_count++;
            (void) App_RequestMode(command);
        } else {
            g_tjc_invalid_command_count++;
            Protocol_Tjc_SendResult(
                TJC_RESULT_INVALID_COMMAND, App_GetCurrentMode(), command);
        }
    }
}
