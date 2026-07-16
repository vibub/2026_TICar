/**
 * @file protocol_tjc.c
 * @brief 处理 TJC 单字节模式命令，并发送带 XOR 校验的五字节状态响应。
 */
#include "protocol_tjc.h"

#include "app_main.h"
#include "bsp_uart.h"

/*
 * CCS Watch 协议统计：接收/合法/非法命令数、响应数、BSP 缓冲溢出和 UART 错误镜像，
 * 以及最近一次原始命令和返回结果码。
 */
volatile uint32_t g_tjc_rx_byte_count;
volatile uint32_t g_tjc_valid_command_count;
volatile uint32_t g_tjc_invalid_command_count;
volatile uint32_t g_tjc_response_count;
volatile uint32_t g_tjc_rx_overflow_count;
volatile uint32_t g_tjc_rx_error_count;
volatile uint8_t g_tjc_last_command;
volatile uint8_t g_tjc_last_result;

/* 初始化统计并丢弃启用协议任务前已经积压的屏幕字节。 */
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

/*
 * 响应帧格式：A5 | RESULT | CURRENT | REQUEST | XOR。
 * CHECKSUM 是前四字节异或；current 是已生效模式，request 是触发该响应的原始命令。
 */
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

/*
 * 从 ISR 环形缓冲中尽可能取完当前命令：QUERY 立即返回状态，0～13 提交异步模式请求，
 * 其余字节返回非法命令。模式请求受理后的最终切换结果由 app_main.c 稍后发送。
 */
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
