#include "protocol_k230.h"

#include <stdio.h>
#include <string.h>

#include "bsp_time.h"
#include "bsp_uart.h"

static char g_k230_line[K230_PROTOCOL_LINE_SIZE];
static uint8_t g_k230_line_length;

volatile K230_TargetFrame g_k230_latest_frame;
static uint8_t g_k230_new_frame;

volatile uint32_t g_k230_valid_frame_count;
volatile uint32_t g_k230_invalid_frame_count;
volatile uint32_t g_k230_rx_byte_count;
volatile uint8_t g_k230_link_alive;
volatile uint8_t g_k230_target_valid;
volatile uint32_t g_k230_last_frame_ms;
volatile uint32_t g_k230_timeout_count;


void Protocol_K230_Init(void)
{
    g_k230_line_length = 0U;
    g_k230_new_frame = 0U;
    g_k230_valid_frame_count = 0U;
    g_k230_invalid_frame_count = 0U;
    g_k230_rx_byte_count = 0U;
    g_k230_link_alive = 0U;
    g_k230_target_valid = 0U;
    g_k230_last_frame_ms = 0U;
    g_k230_timeout_count = 0U;

    g_k230_latest_frame.detected = 0U;
    g_k230_latest_frame.error_x = 0;
    g_k230_latest_frame.error_y = 0;
    g_k230_latest_frame.confidence = 0U;

    Bsp_Time_Init();
}

uint8_t Protocol_K230_ParseLine(
    const char *line,
    K230_TargetFrame *frame)
{
    int error_x;
    int error_y;
    int confidence;
    char extra;

    if ((line == NULL) || (frame == NULL)) {
        return 0U;
    }

    /*
     * K230 无目标帧：
     * N
     */
    if (strcmp(line, "N") == 0) {
        frame->detected = 0U;
        frame->error_x = 0;
        frame->error_y = 0;
        frame->confidence = 0U;
        return 1U;
    }

    /*
     * K230 目标帧：
     * T,error_x,error_y,confidence
     *
     * 末尾增加 %c，用来拒绝带有多余字符的帧。
     * 正确消息只能成功转换前三项，因此返回值必须是 3。
     */
    if (sscanf(
            line,
            "T,%d,%d,%d%c",
            &error_x,
            &error_y,
            &confidence,
            &extra) != 3) {
        return 0U;
    }

    /*
     * K230 使用 640×360 坐标系：
     * X 最大理论误差为 ±320；
     * Y 最大理论误差为 ±180；
     * YOLO 置信度为 0～100。
     */
    if ((error_x < -320) || (error_x > 320)) {
        return 0U;
    }

    if ((error_y < -180) || (error_y > 180)) {
        return 0U;
    }

    if ((confidence < 0) || (confidence > 100)) {
        return 0U;
    }

    frame->detected = 1U;
    frame->error_x = (int16_t) error_x;
    frame->error_y = (int16_t) error_y;
    frame->confidence = (uint8_t) confidence;

    return 1U;
}

static void Protocol_K230_ProcessCompleteLine(void)
{
    K230_TargetFrame frame;

    g_k230_line[g_k230_line_length] = '\0';

    if (Protocol_K230_ParseLine(g_k230_line, &frame) != 0U) {
        g_k230_latest_frame = frame;
        g_k230_new_frame = 1U;
        g_k230_valid_frame_count++;
        g_k230_last_frame_ms = Bsp_Time_GetMilliseconds();
        g_k230_link_alive = 1U;
        g_k230_target_valid = ((frame.detected != 0U) &&
                               (frame.confidence >= K230_MIN_CONFIDENCE)) ? 1U : 0U;

        if (frame.detected != 0U) {
            Bsp_Uart_K230_SendString("ACK,T\r\n");
        } else {
            Bsp_Uart_K230_SendString("ACK,N\r\n");
        }
    } else {
        g_k230_invalid_frame_count++;
        Bsp_Uart_K230_SendString("ERR,FRAME\r\n");
    }

    g_k230_line_length = 0U;
}

void Protocol_K230_Task(void)
{
    uint8_t byte;
    uint32_t error_status;

    /*
     * 先处理 UART 溢出、奇偶校验、帧格式和噪声错误。
     */
    error_status = Bsp_Uart_K230_GetErrorStatus();

    if (error_status != 0U) {
        Bsp_Uart_K230_ClearErrorStatus(error_status);
        g_k230_line_length = 0U;
        g_k230_invalid_frame_count++;
    }

    /*
     * 一次调用尽可能清空当前 RX FIFO。
     * TryReceiveByte 没有数据时立即返回，不会阻塞主循环。
     */
    while (Bsp_Uart_K230_TryReceiveByte(&byte) != 0U) {
        
        g_k230_rx_byte_count++;

        if (byte == (uint8_t) '\r') {
            /*
             * K230 发送的是 CRLF，忽略 CR，等待 LF 完成一帧。
             */
            continue;
        }

        if (byte == (uint8_t) '\n') {
            if (g_k230_line_length != 0U) {
                Protocol_K230_ProcessCompleteLine();
            }

            continue;
        }

        /*
         * 留出一个字节写入字符串结束符 '\0'。
         */
        if (g_k230_line_length < (K230_PROTOCOL_LINE_SIZE - 1U)) {
            g_k230_line[g_k230_line_length] = (char) byte;
            g_k230_line_length++;
        } else {
            /*
             * 当前帧过长，丢弃已经收到的数据并等待下一行。
             */
            g_k230_line_length = 0U;
            g_k230_invalid_frame_count++;
            Bsp_Uart_K230_SendString("ERR,FRAME\r\n");
        }
    }

    if ((g_k230_link_alive != 0U) &&
        ((uint32_t) (Bsp_Time_GetMilliseconds() - g_k230_last_frame_ms) >=
         K230_LINK_TIMEOUT_MS)) {
        g_k230_link_alive = 0U;
        g_k230_target_valid = 0U;
        g_k230_timeout_count++;
    }
}

uint8_t Protocol_K230_TakeLatestFrame(K230_TargetFrame *frame)
{
    if ((frame == NULL) || (g_k230_new_frame == 0U)) {
        return 0U;
    }

    *frame = g_k230_latest_frame;
    g_k230_new_frame = 0U;

    return 1U;
}

uint32_t Protocol_K230_GetValidFrameCount(void)
{
    return g_k230_valid_frame_count;
}

uint32_t Protocol_K230_GetInvalidFrameCount(void)
{
    return g_k230_invalid_frame_count;
}
