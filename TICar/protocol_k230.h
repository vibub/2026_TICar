/**
 * @file protocol_k230.h
 * @brief K230 靶标、红线、数字和视觉状态文本协议接口。
 */
#ifndef PROTOCOL_K230_H
#define PROTOCOL_K230_H

#include <stdint.h>

#define K230_PROTOCOL_LINE_SIZE 32U /* 含字符串结束符的最大行缓冲区大小。 */
#define K230_TARGET_LINK_TIMEOUT_MS 300U /* 靶标 T/N 帧超过该时间未更新即超时。 */
#define K230_LINE_LINK_TIMEOUT_MS 200U /* 红线 L 帧超过该时间未更新即立即停车。 */
#define K230_DIGIT_LINK_TIMEOUT_MS 600U /* 数字 D 帧低频发布，使用独立 freshness。 */
#define K230_LINK_TIMEOUT_MS K230_TARGET_LINK_TIMEOUT_MS /* 兼容原有靶标超时名称。 */
#define K230_MIN_CONFIDENCE 70U /* 低于该置信度的 T 帧不参与云台控制。 */
#define K230_LINE_MIN_QUALITY 45U /* 低于该质量的 L 帧不参与巡线控制。 */

#define K230_LINE_DIRECTION_LEFT  0x01U
#define K230_LINE_DIRECTION_FRONT 0x02U
#define K230_LINE_DIRECTION_RIGHT 0x04U

#define K230_DIGIT_SIDE_CENTER 0U
#define K230_DIGIT_SIDE_LEFT   1U
#define K230_DIGIT_SIDE_RIGHT  2U

#define K230_DIGIT_FLAG_VALID        0x01U
#define K230_DIGIT_FLAG_TARGET_MATCH 0x02U
#define K230_DIGIT_FLAG_CONSENSUS    0x04U
#define K230_DIGIT_FLAG_LOCKED       0x08U
#define K230_DIGIT_FLAG_ERROR        0x10U
#define K230_DIGIT_FLAG_MASK         0x1FU

#define K230_VISUAL_MODE_OFF      0U
#define K230_VISUAL_MODE_PHARMACY 1U
#define K230_VISUAL_MODE_TARGET   2U

#define K230_ROUTE_REGION_PHARMACY  0U
#define K230_ROUTE_REGION_NEAR      1U
#define K230_ROUTE_REGION_MIDDLE    2U
#define K230_ROUTE_REGION_FAR       3U
#define K230_ROUTE_REGION_FAR_LEFT  4U
#define K230_ROUTE_REGION_FAR_RIGHT 5U
#define K230_ROUTE_REGION_RETURN    6U
#define K230_ROUTE_REGION_RESERVED  7U

/** K230 解析后的靶标帧；X/Y 范围分别为 ±320、±180。 */
typedef struct {
    uint8_t detected;
    int16_t error_x;
    int16_t error_y;
    uint8_t confidence;
} K230_TargetFrame;

/** K230 红线帧，来自 320×240 传统视觉通道。 */
typedef struct {
    uint8_t valid;
    int16_t error_x;
    int16_t angle_d10;
    uint8_t quality;
    uint8_t direction_mask;
} K230_LineFrame;

/** K230 数字帧，检测框使用修正方向后的 640×360 AI 坐标系。 */
typedef struct {
    uint8_t valid;
    uint8_t digit;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint8_t side;
    uint8_t confidence;
    uint8_t flags;
} K230_DigitFrame;

/* 原有靶标协议 Watch 状态，继续保持兼容。 */
extern volatile K230_TargetFrame g_k230_latest_frame;
extern volatile uint32_t g_k230_valid_frame_count;
extern volatile uint32_t g_k230_invalid_frame_count;
extern volatile uint32_t g_k230_rx_byte_count;
extern volatile uint8_t g_k230_link_alive;
extern volatile uint8_t g_k230_target_valid;
extern volatile uint32_t g_k230_last_frame_ms;
extern volatile uint32_t g_k230_timeout_count;

/* 红线协议使用独立邮箱、帧计数和 freshness。 */
extern volatile K230_LineFrame g_k230_latest_line_frame;
extern volatile uint32_t g_k230_line_frame_count;
extern volatile uint8_t g_k230_line_alive;
extern volatile uint8_t g_k230_line_control_valid;
extern volatile uint32_t g_k230_last_line_frame_ms;
extern volatile uint32_t g_k230_line_timeout_count;

/* 数字协议使用第三个独立邮箱，不能刷新靶标或红线链路。 */
extern volatile K230_DigitFrame g_k230_latest_digit_frame;
extern volatile uint32_t g_k230_digit_frame_count;
extern volatile uint8_t g_k230_digit_alive;
extern volatile uint32_t g_k230_last_digit_frame_ms;
extern volatile uint32_t g_k230_digit_timeout_count;

/* 最近一次 K230 对 V 命令的应用确认。 */
extern volatile uint8_t g_k230_visual_ack_mode;
extern volatile uint8_t g_k230_visual_ack_target_digit;
extern volatile uint8_t g_k230_visual_ack_route_region;
extern volatile uint8_t g_k230_visual_ack_epoch;
extern volatile uint32_t g_k230_visual_ack_count;

void Protocol_K230_Init(void);
void Protocol_K230_Task(void);

uint8_t Protocol_K230_ParseLine(const char *line, K230_TargetFrame *frame);
uint8_t Protocol_K230_ParseLineFrame(const char *line, K230_LineFrame *frame);
uint8_t Protocol_K230_ParseDigitFrame(const char *line, K230_DigitFrame *frame);

uint8_t Protocol_K230_TakeLatestFrame(K230_TargetFrame *frame);
uint8_t Protocol_K230_TakeLatestLineFrame(K230_LineFrame *frame);
uint8_t Protocol_K230_TakeLatestDigitFrame(K230_DigitFrame *frame);

uint8_t Protocol_K230_IsLineFresh(void);
uint32_t Protocol_K230_GetLineAgeMs(void);
uint8_t Protocol_K230_IsDigitFresh(void);
uint32_t Protocol_K230_GetDigitAgeMs(void);

/** 下发 `V,mode,target_digit,route_region,epoch`，参数非法时返回 0。 */
uint8_t Protocol_K230_SendVisualCommand(
    uint8_t mode,
    uint8_t target_digit,
    uint8_t route_region,
    uint8_t epoch
);

/** 返回最近一次 V 命令确认是否与完整工作上下文匹配。 */
uint8_t Protocol_K230_IsVisualCommandApplied(
    uint8_t mode,
    uint8_t target_digit,
    uint8_t route_region,
    uint8_t epoch
);

uint32_t Protocol_K230_GetValidFrameCount(void);
uint32_t Protocol_K230_GetInvalidFrameCount(void);

#endif
