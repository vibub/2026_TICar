/**
 * @file protocol_tjc.c
 * @brief 按固定四字节帧解析 TJC 模式命令，并发送带 XOR 校验的五字节状态响应。
 */
#include "protocol_tjc.h"

#include "app_main.h"
#include "bsp_uart.h"

/* 固定长度请求帧的逐字节解析状态；状态只在主循环中读写，不与 UART ISR 共享。 */
typedef enum {
    TJC_RX_WAIT_HEADER_0 = 0,
    TJC_RX_WAIT_HEADER_1,
    TJC_RX_WAIT_COMMAND,
    TJC_RX_WAIT_CHECKSUM
} Tjc_RxState;

static Tjc_RxState g_tjc_rx_state;
static uint8_t g_tjc_rx_command;

/*
 * CCS Watch 协议统计：接收字节、合法/非法命令、校验失败帧、响应、BSP 缓冲溢出和
 * UART 错误镜像，以及最近一次完整有效帧中的命令和最近一次返回结果码。
 */
volatile uint32_t g_tjc_rx_byte_count;
volatile uint32_t g_tjc_valid_command_count;
volatile uint32_t g_tjc_invalid_command_count;
volatile uint32_t g_tjc_invalid_frame_count;
volatile uint32_t g_tjc_response_count;
volatile uint32_t g_tjc_rx_overflow_count;
volatile uint32_t g_tjc_rx_error_count;
volatile uint8_t g_tjc_last_command;
volatile uint8_t g_tjc_last_result;

/* 丢弃当前半帧并回到第一个帧头字节的搜索状态。 */
static void Protocol_Tjc_ResetRxParser(void)
{
    g_tjc_rx_state = TJC_RX_WAIT_HEADER_0;
    g_tjc_rx_command = TJC_COMMAND_STOP;
}

/* 请求帧校验覆盖两个帧头字节和命令字，不包含校验字节自身。 */
static uint8_t Protocol_Tjc_CalculateRequestChecksum(uint8_t command)
{
    return (uint8_t) (TJC_REQUEST_HEADER_0 ^ TJC_REQUEST_HEADER_1 ^ command);
}

/* 初始化统计、组帧状态，并丢弃启用协议任务前已经积压的屏幕字节。 */
void Protocol_Tjc_Init(void)
{
    g_tjc_rx_byte_count = 0U;
    g_tjc_valid_command_count = 0U;
    g_tjc_invalid_command_count = 0U;
    g_tjc_invalid_frame_count = 0U;
    g_tjc_response_count = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;
    g_tjc_last_command = TJC_COMMAND_STOP;
    g_tjc_last_result = TJC_RESULT_STATE;
    Protocol_Tjc_ResetRxParser();
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
 * 只处理已经通过帧头和 XOR 校验的命令：QUERY 立即返回状态，0～13 提交异步模式请求，
 * 其余命令返回非法命令。模式请求受理后的最终切换结果由 app_main.c 稍后发送。
 */
static void Protocol_Tjc_ProcessCommand(uint8_t command)
{
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

/*
 * 把一个 UART 字节送入固定四字节帧状态机。
 * 在等待命令或校验时再次看到 0x5A，会立即把它视为新帧起点，从半帧或错位数据中快速恢复。
 */
static void Protocol_Tjc_ConsumeByte(uint8_t byte)
{
    switch (g_tjc_rx_state) {
        case TJC_RX_WAIT_HEADER_0:
            if (byte == TJC_REQUEST_HEADER_0) {
                g_tjc_rx_state = TJC_RX_WAIT_HEADER_1;
            }
            break;

        case TJC_RX_WAIT_HEADER_1:
            if (byte == TJC_REQUEST_HEADER_1) {
                g_tjc_rx_state = TJC_RX_WAIT_COMMAND;
            } else if (byte != TJC_REQUEST_HEADER_0) {
                g_tjc_rx_state = TJC_RX_WAIT_HEADER_0;
            }
            break;

        case TJC_RX_WAIT_COMMAND:
            if (byte == TJC_REQUEST_HEADER_0) {
                g_tjc_rx_state = TJC_RX_WAIT_HEADER_1;
            } else {
                g_tjc_rx_command = byte;
                g_tjc_rx_state = TJC_RX_WAIT_CHECKSUM;
            }
            break;

        case TJC_RX_WAIT_CHECKSUM:
            if (byte == Protocol_Tjc_CalculateRequestChecksum(g_tjc_rx_command)) {
                Protocol_Tjc_ProcessCommand(g_tjc_rx_command);
                Protocol_Tjc_ResetRxParser();
            } else {
                g_tjc_invalid_frame_count++;
                g_tjc_rx_state = (byte == TJC_REQUEST_HEADER_0) ?
                    TJC_RX_WAIT_HEADER_1 : TJC_RX_WAIT_HEADER_0;
            }
            break;

        default:
            Protocol_Tjc_ResetRxParser();
            break;
    }
}

/*
 * 一次调用尽可能取完 ISR 环形缓冲中的字节，但仅在收到完整且校验正确的请求帧后执行命令。
 * 如果 BSP 检测到缓冲溢出或 UART 错误，则先丢弃当前半帧，避免把缺失字节的数据误拼成命令。
 */
void Protocol_Tjc_Task(void)
{
    uint8_t byte;
    uint32_t overflow_count = Bsp_Uart_Tjc_GetOverflowCount();
    uint32_t error_count = Bsp_Uart_Tjc_GetErrorCount();

    if ((overflow_count != g_tjc_rx_overflow_count) ||
        (error_count != g_tjc_rx_error_count)) {
        Protocol_Tjc_ResetRxParser();
    }

    g_tjc_rx_overflow_count = overflow_count;
    g_tjc_rx_error_count = error_count;

    while (Bsp_Uart_Tjc_TryReceiveByte(&byte) != 0U) {
        g_tjc_rx_byte_count++;
        Protocol_Tjc_ConsumeByte(byte);
    }
}
