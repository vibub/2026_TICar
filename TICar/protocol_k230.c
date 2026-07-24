/**
 * @file protocol_k230.c
 * @brief 解析 K230 靶标、红线、数字和视觉命令文本帧。
 */
#include "protocol_k230.h"

#include <stdio.h>
#include <string.h>

#include "bsp_time.h"
#include "bsp_uart.h"

static char g_k230_line[K230_PROTOCOL_LINE_SIZE];
static uint8_t g_k230_line_length;
static uint8_t g_k230_discard_until_lf;

/* 三类视觉数据分别采用单槽发布，消费方不会相互抢占新帧标志。 */
volatile K230_TargetFrame g_k230_latest_frame;
static uint8_t g_k230_new_frame;
volatile K230_LineFrame g_k230_latest_line_frame;
static uint8_t g_k230_new_line_frame;
volatile K230_DigitFrame g_k230_latest_digit_frame;
static uint8_t g_k230_new_digit_frame;

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

volatile uint32_t g_k230_digit_frame_count;
volatile uint8_t g_k230_digit_alive;
volatile uint32_t g_k230_last_digit_frame_ms;
volatile uint32_t g_k230_digit_timeout_count;

volatile uint8_t g_k230_visual_ack_mode;
volatile uint8_t g_k230_visual_ack_target_digit;
volatile uint8_t g_k230_visual_ack_route_region;
volatile uint8_t g_k230_visual_ack_epoch;
volatile uint32_t g_k230_visual_ack_count;


void Protocol_K230_Init(void)
{
    g_k230_line_length = 0U;
    g_k230_discard_until_lf = 0U;
    g_k230_new_frame = 0U;
    g_k230_new_line_frame = 0U;
    g_k230_new_digit_frame = 0U;

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

    g_k230_digit_frame_count = 0U;
    g_k230_digit_alive = 0U;
    g_k230_last_digit_frame_ms = 0U;
    g_k230_digit_timeout_count = 0U;
    g_k230_visual_ack_mode = 0U;
    g_k230_visual_ack_target_digit = 0U;
    g_k230_visual_ack_route_region = 0U;
    g_k230_visual_ack_epoch = 0U;
    g_k230_visual_ack_count = 0U;

    memset((void *) &g_k230_latest_frame, 0, sizeof(g_k230_latest_frame));
    memset((void *) &g_k230_latest_line_frame, 0,
           sizeof(g_k230_latest_line_frame));
    memset((void *) &g_k230_latest_digit_frame, 0,
           sizeof(g_k230_latest_digit_frame));
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

    if (sscanf(line, "T,%d,%d,%d%c", &error_x, &error_y,
               &confidence, &extra) != 3) {
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

    if (sscanf(line, "L,%d,%d,%d,%d,%d%c", &valid, &error_x,
               &angle_d10, &quality, &direction_mask, &extra) != 5) {
        return 0U;
    }

    if ((valid < 0) || (valid > 1) ||
        (error_x < -160) || (error_x > 160) ||
        (angle_d10 < -900) || (angle_d10 > 900) ||
        (quality < 0) || (quality > 100) ||
        (direction_mask < 0) || (direction_mask > 7)) {
        return 0U;
    }

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

uint8_t Protocol_K230_ParseDigitFrame(
    const char *line,
    K230_DigitFrame *frame)
{
    int valid;
    int digit;
    int x;
    int y;
    int width;
    int height;
    int side;
    int confidence;
    int flags;
    char extra;

    if ((line == NULL) || (frame == NULL)) {
        return 0U;
    }

    if (sscanf(line, "D,%d,%d,%d,%d,%d,%d,%d,%d,%d%c",
               &valid, &digit, &x, &y, &width, &height, &side,
               &confidence, &flags, &extra) != 9) {
        return 0U;
    }

    if ((valid < 0) || (valid > 1) ||
        (digit < 0) || (digit > 8) ||
        (x < 0) || (x >= 640) ||
        (y < 0) || (y >= 360) ||
        (width < 0) || (width > 640) ||
        (height < 0) || (height > 360) ||
        (side < 0) || (side > (int) K230_DIGIT_SIDE_RIGHT) ||
        (confidence < 0) || (confidence > 100) ||
        (flags < 0) || (flags > (int) K230_DIGIT_FLAG_MASK)) {
        return 0U;
    }

    if (valid == 0) {
        if ((digit != 0) || (x != 0) || (y != 0) ||
            (width != 0) || (height != 0) ||
            (side != K230_DIGIT_SIDE_CENTER) || (confidence != 0)) {
            return 0U;
        }
    } else {
        if ((digit < 1) || (width < 1) || (height < 1) ||
            ((x + width) > 640) || ((y + height) > 360) ||
            ((flags & K230_DIGIT_FLAG_VALID) == 0)) {
            return 0U;
        }
    }

    frame->valid = (uint8_t) valid;
    frame->digit = (uint8_t) digit;
    frame->x = (uint16_t) x;
    frame->y = (uint16_t) y;
    frame->width = (uint16_t) width;
    frame->height = (uint16_t) height;
    frame->side = (uint8_t) side;
    frame->confidence = (uint8_t) confidence;
    frame->flags = (uint8_t) flags;
    return 1U;
}

static uint8_t Protocol_K230_ParseVisualAck(
    const char *line,
    uint8_t *mode,
    uint8_t *target_digit,
    uint8_t *route_region,
    uint8_t *epoch)
{
    int parsed_mode;
    int parsed_target_digit;
    int parsed_route_region;
    int parsed_epoch;
    char extra;

    if ((line == NULL) || (mode == NULL) || (target_digit == NULL) ||
        (route_region == NULL) || (epoch == NULL)) {
        return 0U;
    }
    if (sscanf(line, "A,V,%d,%d,%d,%d%c", &parsed_mode,
               &parsed_target_digit, &parsed_route_region,
               &parsed_epoch, &extra) != 4) {
        return 0U;
    }
    if ((parsed_mode < 0) ||
        (parsed_mode > (int) K230_VISUAL_MODE_TARGET) ||
        (parsed_target_digit < 0) || (parsed_target_digit > 8) ||
        (parsed_route_region < 0) ||
        (parsed_route_region > (int) K230_ROUTE_REGION_RESERVED) ||
        (parsed_epoch < 0) || (parsed_epoch > 255)) {
        return 0U;
    }
    *mode = (uint8_t) parsed_mode;
    *target_digit = (uint8_t) parsed_target_digit;
    *route_region = (uint8_t) parsed_route_region;
    *epoch = (uint8_t) parsed_epoch;
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
    /* T/N 是周期遥测帧；合法数据不逐帧确认，避免反向串口被 ACK 占满。 */
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
    /* L 帧高频上报且不会重传；仅可靠的 V 控制命令保留应答。 */
}

static void Protocol_K230_PublishDigit(const K230_DigitFrame *frame)
{
    g_k230_latest_digit_frame = *frame;
    g_k230_new_digit_frame = 1U;
    g_k230_digit_frame_count++;
    g_k230_last_digit_frame_ms = Bsp_Time_GetMilliseconds();
    g_k230_digit_alive = 1U;
    /* D 帧只更新数字邮箱和 freshness，不再向 K230 发送无用途的逐帧 ACK。 */
}

static void Protocol_K230_ProcessCompleteLine(void)
{
    K230_TargetFrame target_frame;
    K230_LineFrame line_frame;
    K230_DigitFrame digit_frame;
    uint8_t visual_mode;
    uint8_t visual_target_digit;
    uint8_t visual_route_region;
    uint8_t visual_epoch;
    uint8_t accepted = 0U;

    g_k230_line[g_k230_line_length] = '\0';

    if ((g_k230_line[0] == 'L') && (g_k230_line[1] == ',')) {
        if (Protocol_K230_ParseLineFrame(g_k230_line, &line_frame) != 0U) {
            Protocol_K230_PublishLine(&line_frame);
            accepted = 1U;
        }
    } else if ((g_k230_line[0] == 'D') && (g_k230_line[1] == ',')) {
        if (Protocol_K230_ParseDigitFrame(g_k230_line, &digit_frame) != 0U) {
            Protocol_K230_PublishDigit(&digit_frame);
            accepted = 1U;
        }
    } else if ((g_k230_line[0] == 'A') && (g_k230_line[1] == ',') &&
               (Protocol_K230_ParseVisualAck(
                   g_k230_line,
                   &visual_mode,
                   &visual_target_digit,
                   &visual_route_region,
                   &visual_epoch) != 0U)) {
        g_k230_visual_ack_mode = visual_mode;
        g_k230_visual_ack_target_digit = visual_target_digit;
        g_k230_visual_ack_route_region = visual_route_region;
        g_k230_visual_ack_epoch = visual_epoch;
        g_k230_visual_ack_count++;
        accepted = 1U;
    } else if (Protocol_K230_ParseLine(g_k230_line, &target_frame) != 0U) {
        Protocol_K230_PublishTarget(&target_frame);
        accepted = 1U;
    }

    if (accepted == 0U) {
        /* K230 周期遥测只做统计和丢弃，不回发错误文本干扰控制命令。 */
        g_k230_invalid_frame_count++;
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
            /* 超长行丢弃到 LF 后重新同步，MSPM0 不向 K230 发送错误回包。 */
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

    if ((g_k230_digit_alive != 0U) &&
        ((uint32_t) (now_ms - g_k230_last_digit_frame_ms) >=
         K230_DIGIT_LINK_TIMEOUT_MS)) {
        g_k230_digit_alive = 0U;
        g_k230_digit_timeout_count++;
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

uint8_t Protocol_K230_TakeLatestDigitFrame(K230_DigitFrame *frame)
{
    if ((frame == NULL) || (g_k230_new_digit_frame == 0U)) {
        return 0U;
    }
    *frame = g_k230_latest_digit_frame;
    g_k230_new_digit_frame = 0U;
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
    return (uint32_t) (Bsp_Time_GetMilliseconds() - g_k230_last_line_frame_ms);
}

uint8_t Protocol_K230_IsDigitFresh(void)
{
    return g_k230_digit_alive;
}

uint32_t Protocol_K230_GetDigitAgeMs(void)
{
    if (g_k230_digit_frame_count == 0U) {
        return UINT32_MAX;
    }
    return (uint32_t) (Bsp_Time_GetMilliseconds() - g_k230_last_digit_frame_ms);
}

uint8_t Protocol_K230_SendVisualCommand(
    uint8_t mode,
    uint8_t target_digit,
    uint8_t route_region,
    uint8_t epoch)
{
    char command[32];
    int length;

    if ((mode > K230_VISUAL_MODE_TARGET) ||
        (target_digit > 8U) ||
        (route_region > K230_ROUTE_REGION_RESERVED)) {
        return 0U;
    }

    length = snprintf((char *) command, sizeof(command),
                      "V,%u,%u,%u,%u\r\n",
                      (unsigned int) mode,
                      (unsigned int) target_digit,
                      (unsigned int) route_region,
                      (unsigned int) epoch);
    if ((length <= 0) || ((unsigned int) length >= sizeof(command))) {
        return 0U;
    }
    Bsp_Uart_K230_SendString(command);
    return 1U;
}

uint8_t Protocol_K230_IsVisualCommandApplied(
    uint8_t mode,
    uint8_t target_digit,
    uint8_t route_region,
    uint8_t epoch)
{
    return (g_k230_visual_ack_count != 0U) &&
           (g_k230_visual_ack_mode == mode) &&
           (g_k230_visual_ack_target_digit == target_digit) &&
           (g_k230_visual_ack_route_region == route_region) &&
           (g_k230_visual_ack_epoch == epoch);
}

uint32_t Protocol_K230_GetValidFrameCount(void)
{
    return g_k230_valid_frame_count;
}

uint32_t Protocol_K230_GetInvalidFrameCount(void)
{
    return g_k230_invalid_frame_count;
}
