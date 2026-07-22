/**
 * @file protocol_k230.c
 * @brief 解析 K230 靶标和红线文本帧，分别维护邮箱、有效性和链路超时。
 */
#include "protocol_k230.h"

#include <stdio.h>
#include <string.h>

#include "bsp_time.h"
#include "bsp_uart.h"

/* 当前 CRLF 文本行的组帧缓冲和已接收长度。 */
static char g_k230_line[K230_PROTOCOL_LINE_SIZE];
static uint8_t g_k230_line_length;
static uint8_t g_k230_discard_until_lf;

/* 两类数据分别采用单槽发布；新帧到达会覆盖尚未消费的旧帧。 */
volatile K230_TargetFrame g_k230_latest_frame;
static uint8_t g_k230_new_frame;
volatile K230_LineFrame g_k230_latest_line_frame;
static uint8_t g_k230_new_line_frame;

volatile uint32_t g_k230_valid_frame_count;
volatile uint32_t g_k230_invalid_frame_count;
volatile uint32_t g_k230_rx_byte_count;
volatile uint8_t g_k230_link_alive;
volatile uint8_t g_k230_target_valid;
volatile uint32_t g_k230_last_frame_ms;
volatile uint32_t g_k230_timeout_count;

volatile uint32_t g_k230_line_frame_count;
volatile uint8_t g_k230_line_alive;
volatile uint8_t g_k230_line_control_valid;
volatile uint32_t g_k230_last_line_frame_ms;
volatile uint32_t g_k230_line_timeout_count;


/* 清除半帧、两个邮箱、链路状态和全部协议统计。 */
void Protocol_K230_Init(void)
{
    g_k230_line_length = 0U;
    g_k230_discard_until_lf = 0U;
    g_k230_new_frame = 0U;
    g_k230_new_line_frame = 0U;

    g_k230_valid_frame_count = 0U;
    g_k230_invalid_frame_count = 0U;
    g_k230_rx_byte_count = 0U;
    g_k230_link_alive = 0U;
    g_k230_target_valid = 0U;
    g_k230_last_frame_ms = 0U;
    g_k230_timeout_count = 0U;

    g_k230_line_frame_count = 0U;
    g_k230_line_alive = 0U;
    g_k230_line_control_valid = 0U;
    g_k230_last_line_frame_ms = 0U;
    g_k230_line_timeout_count = 0U;

    g_k230_latest_frame.detected = 0U;
    g_k230_latest_frame.error_x = 0;
    g_k230_latest_frame.error_y = 0;
    g_k230_latest_frame.confidence = 0U;

    g_k230_latest_line_frame.valid = 0U;
    g_k230_latest_line_frame.error_x = 0;
    g_k230_latest_line_frame.angle_d10 = 0;
    g_k230_latest_line_frame.quality = 0U;
    g_k230_latest_line_frame.direction_mask = 0U;
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

    if (strcmp(line, "N") == 0) {
        frame->detected = 0U;
        frame->error_x = 0;
        frame->error_y = 0;
        frame->confidence = 0U;
        return 1U;
    }

    if (sscanf(
            line,
            "T,%d,%d,%d%c",
            &error_x,
            &error_y,
            &confidence,
            &extra) != 3) {
        return 0U;
    }

    if ((error_x < -320) || (error_x > 320) ||
        (error_y < -180) || (error_y > 180) ||
        (confidence < 0) || (confidence > 100)) {
        return 0U;
    }

    frame->detected = 1U;
    frame->error_x = (int16_t) error_x;
    frame->error_y = (int16_t) error_y;
    frame->confidence = (uint8_t) confidence;
    return 1U;
}

uint8_t Protocol_K230_ParseLineFrame(
    const char *line,
    K230_LineFrame *frame)
{
    int valid;
    int error_x;
    int angle_d10;
    int quality;
    int direction_mask;
    char extra;

    if ((line == NULL) || (frame == NULL)) {
        return 0U;
    }

    if (sscanf(
            line,
            "L,%d,%d,%d,%d,%d%c",
            &valid,
            &error_x,
            &angle_d10,
            &quality,
            &direction_mask,
            &extra) != 5) {
        return 0U;
    }

    if ((valid < 0) || (valid > 1) ||
        (error_x < -160) || (error_x > 160) ||
        (angle_d10 < -900) || (angle_d10 > 900) ||
        (quality < 0) || (quality > 100) ||
        (direction_mask < 0) || (direction_mask > 7)) {
        return 0U;
    }

    /* invalid 帧只证明算法在线，禁止携带可能被误用的旧控制量。 */
    if ((valid == 0) &&
        ((error_x != 0) || (angle_d10 != 0) || (direction_mask != 0))) {
        return 0U;
    }

    frame->valid = (uint8_t) valid;
    frame->error_x = (int16_t) error_x;
    frame->angle_d10 = (int16_t) angle_d10;
    frame->quality = (uint8_t) quality;
    frame->direction_mask = (uint8_t) direction_mask;
    return 1U;
}

static void Protocol_K230_PublishTarget(const K230_TargetFrame *frame)
{
    g_k230_latest_frame = *frame;
    g_k230_new_frame = 1U;
    g_k230_valid_frame_count++;
    g_k230_last_frame_ms = Bsp_Time_GetMilliseconds();
    g_k230_link_alive = 1U;
    g_k230_target_valid = ((frame->detected != 0U) &&
                           (frame->confidence >= K230_MIN_CONFIDENCE)) ? 1U : 0U;

    Bsp_Uart_K230_SendString(
        (frame->detected != 0U) ? "ACK,T\r\n" : "ACK,N\r\n");
}

static void Protocol_K230_PublishLine(const K230_LineFrame *frame)
{
    g_k230_latest_line_frame = *frame;
    g_k230_new_line_frame = 1U;
    g_k230_line_frame_count++;
    g_k230_last_line_frame_ms = Bsp_Time_GetMilliseconds();
    g_k230_line_alive = 1U;
    g_k230_line_control_valid = ((frame->valid != 0U) &&
                                 (frame->quality >= K230_LINE_MIN_QUALITY)) ? 1U : 0U;
    Bsp_Uart_K230_SendString("ACK,L\r\n");
}

/* 根据前缀分发完整行；任一解析失败都只累计一次无效帧。 */
static void Protocol_K230_ProcessCompleteLine(void)
{
    K230_TargetFrame target_frame;
    K230_LineFrame line_frame;
    uint8_t accepted = 0U;

    g_k230_line[g_k230_line_length] = '\0';

    if ((g_k230_line[0] == 'L') && (g_k230_line[1] == ',')) {
        if (Protocol_K230_ParseLineFrame(g_k230_line, &line_frame) != 0U) {
            Protocol_K230_PublishLine(&line_frame);
            accepted = 1U;
        }
    } else if (Protocol_K230_ParseLine(g_k230_line, &target_frame) != 0U) {
        Protocol_K230_PublishTarget(&target_frame);
        accepted = 1U;
    }

    if (accepted == 0U) {
        g_k230_invalid_frame_count++;
        Bsp_Uart_K230_SendString("ERR,FRAME\r\n");
    }

    g_k230_line_length = 0U;
}

void Protocol_K230_Task(void)
{
    uint8_t byte;
    uint32_t error_status;
    uint32_t now_ms;

    error_status = Bsp_Uart_K230_GetErrorStatus();
    if (error_status != 0U) {
        Bsp_Uart_K230_ClearErrorStatus(error_status);
        g_k230_line_length = 0U;
        g_k230_discard_until_lf = 0U;
        g_k230_invalid_frame_count++;
    }

    while (Bsp_Uart_K230_TryReceiveByte(&byte) != 0U) {
        g_k230_rx_byte_count++;

        if (g_k230_discard_until_lf != 0U) {
            if (byte == (uint8_t) '\n') {
                g_k230_discard_until_lf = 0U;
                g_k230_line_length = 0U;
            }
            continue;
        }

        if (byte == (uint8_t) '\r') {
            continue;
        }

        if (byte == (uint8_t) '\n') {
            if (g_k230_line_length != 0U) {
                Protocol_K230_ProcessCompleteLine();
            }
            continue;
        }

        if (g_k230_line_length < (K230_PROTOCOL_LINE_SIZE - 1U)) {
            g_k230_line[g_k230_line_length] = (char) byte;
            g_k230_line_length++;
        } else {
            g_k230_line_length = 0U;
            g_k230_discard_until_lf = 1U;
            g_k230_invalid_frame_count++;
            Bsp_Uart_K230_SendString("ERR,FRAME\r\n");
        }
    }

    now_ms = Bsp_Time_GetMilliseconds();

    if ((g_k230_link_alive != 0U) &&
        ((uint32_t) (now_ms - g_k230_last_frame_ms) >=
         K230_TARGET_LINK_TIMEOUT_MS)) {
        g_k230_link_alive = 0U;
        g_k230_target_valid = 0U;
        g_k230_timeout_count++;
    }

    if ((g_k230_line_alive != 0U) &&
        ((uint32_t) (now_ms - g_k230_last_line_frame_ms) >=
         K230_LINE_LINK_TIMEOUT_MS)) {
        g_k230_line_alive = 0U;
        g_k230_line_control_valid = 0U;
        g_k230_line_timeout_count++;
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

uint8_t Protocol_K230_TakeLatestLineFrame(K230_LineFrame *frame)
{
    if ((frame == NULL) || (g_k230_new_line_frame == 0U)) {
        return 0U;
    }

    *frame = g_k230_latest_line_frame;
    g_k230_new_line_frame = 0U;
    return 1U;
}

uint8_t Protocol_K230_IsLineFresh(void)
{
    return g_k230_line_alive;
}

uint32_t Protocol_K230_GetLineAgeMs(void)
{
    if (g_k230_line_frame_count == 0U) {
        return UINT32_MAX;
    }

    return (uint32_t) (
        Bsp_Time_GetMilliseconds() - g_k230_last_line_frame_ms);
}

uint32_t Protocol_K230_GetValidFrameCount(void)
{
    return g_k230_valid_frame_count;
}

uint32_t Protocol_K230_GetInvalidFrameCount(void)
{
    return g_k230_invalid_frame_count;
}
