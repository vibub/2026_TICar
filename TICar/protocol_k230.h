/**
 * @file protocol_k230.h
 * @brief K230 靶标与红线文本协议的数据结构、状态和非阻塞解析接口。
 */
#ifndef PROTOCOL_K230_H
#define PROTOCOL_K230_H

#include <stdint.h>

#define K230_PROTOCOL_LINE_SIZE 32U /* 含字符串结束符的最大行缓冲区大小。 */
#define K230_TARGET_LINK_TIMEOUT_MS 300U /* 靶标 T/N 帧超过该时间未更新即超时。 */
#define K230_LINE_LINK_TIMEOUT_MS 200U /* 红线 L 帧超过该时间未更新即立即停车。 */
#define K230_LINK_TIMEOUT_MS K230_TARGET_LINK_TIMEOUT_MS /* 兼容原有靶标超时名称。 */
#define K230_MIN_CONFIDENCE 70U /* 低于该置信度的 T 帧不参与云台控制。 */
#define K230_LINE_MIN_QUALITY 45U /* 低于该质量的 L 帧不参与巡线控制。 */

#define K230_LINE_DIRECTION_LEFT  0x01U
#define K230_LINE_DIRECTION_FRONT 0x02U
#define K230_LINE_DIRECTION_RIGHT 0x04U

/** K230 解析后的靶标帧；X/Y 范围分别为 ±320、±180。 */
typedef struct {
    uint8_t detected;   /* 1 表示 T 帧检测到目标，0 表示 N 帧无目标。 */
    int16_t error_x;    /* 目标相对图像中心的水平像素误差。 */
    int16_t error_y;    /* 目标相对图像中心的垂直像素误差。 */
    uint8_t confidence; /* 整数置信度，范围 0～100。 */
} K230_TargetFrame;

/**
 * K230 红线帧，来自 320×240 传统视觉通道。
 * error_x 右偏为正；angle_d10 右前方为正，单位 0.1°。
 */
typedef struct {
    uint8_t valid;
    int16_t error_x;
    int16_t angle_d10;
    uint8_t quality;
    uint8_t direction_mask;
} K230_LineFrame;

/* 原有靶标协议 Watch 状态，继续保持兼容。 */
extern volatile K230_TargetFrame g_k230_latest_frame;
extern volatile uint32_t g_k230_valid_frame_count;
extern volatile uint32_t g_k230_invalid_frame_count;
extern volatile uint32_t g_k230_rx_byte_count;
extern volatile uint8_t g_k230_link_alive;
extern volatile uint8_t g_k230_target_valid;
extern volatile uint32_t g_k230_last_frame_ms;
extern volatile uint32_t g_k230_timeout_count;

/* 红线协议使用独立邮箱、帧计数和 freshness，T/N 帧不得刷新这些状态。 */
extern volatile K230_LineFrame g_k230_latest_line_frame;
extern volatile uint32_t g_k230_line_frame_count;
extern volatile uint8_t g_k230_line_alive;
extern volatile uint8_t g_k230_line_control_valid;
extern volatile uint32_t g_k230_last_line_frame_ms;
extern volatile uint32_t g_k230_line_timeout_count;

void Protocol_K230_Init(void);

/** 从 UART FIFO 非阻塞读取并处理全部可用字节，同时推进两类链路超时。 */
void Protocol_K230_Task(void);

/** 解析原有 T/N 靶标消息；成功返回 1。 */
uint8_t Protocol_K230_ParseLine(
    const char *line,
    K230_TargetFrame *frame
);

/** 解析 L,valid,error_x,angle_d10,quality,direction_mask；成功返回 1。 */
uint8_t Protocol_K230_ParseLineFrame(
    const char *line,
    K230_LineFrame *frame
);

/** 获取并消费最新靶标帧。 */
uint8_t Protocol_K230_TakeLatestFrame(K230_TargetFrame *frame);

/** 获取并消费最新红线帧。 */
uint8_t Protocol_K230_TakeLatestLineFrame(K230_LineFrame *frame);

/** 返回红线链路是否仍在 200 ms freshness 窗口内。 */
uint8_t Protocol_K230_IsLineFresh(void);

/** 返回最近红线帧年龄；尚未收到红线帧时返回 UINT32_MAX。 */
uint32_t Protocol_K230_GetLineAgeMs(void);

uint32_t Protocol_K230_GetValidFrameCount(void);
uint32_t Protocol_K230_GetInvalidFrameCount(void);

#endif
