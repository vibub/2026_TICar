/**
 * @file protocol_k230.h
 * @brief K230 目标文本协议的数据结构、调试量和非阻塞解析接口。
 */
#ifndef PROTOCOL_K230_H
#define PROTOCOL_K230_H

#include <stdint.h>

#define K230_PROTOCOL_LINE_SIZE 32U /* 含字符串结束符的最大行缓冲区大小。 */
#define K230_LINK_TIMEOUT_MS 300U  /* 超过 300 ms 无完整有效帧即判定链路超时。 */
#define K230_MIN_CONFIDENCE 70U    /* 低于该置信度的 T 帧不参与云台控制。 */

/** K230 解析后的目标帧；X/Y 范围分别为 ±320、±180，confidence 范围为 0～100。 */
typedef struct {
    uint8_t detected;   /* 1 表示 T 帧检测到目标，0 表示 N 帧无目标。 */
    int16_t error_x;    /* 目标相对图像中心的水平像素误差。 */
    int16_t error_y;    /* 目标相对图像中心的垂直像素误差。 */
    uint8_t confidence; /* 整数置信度。 */
} K230_TargetFrame;

/*
 * 供 CCS Watch 观察的协议状态：最新帧、有效/无效帧数、接收字节数、链路/目标有效性、
 * 最近有效帧毫秒时间戳和累计超时次数。
 */
extern volatile K230_TargetFrame g_k230_latest_frame;
extern volatile uint32_t g_k230_valid_frame_count;
extern volatile uint32_t g_k230_invalid_frame_count;
extern volatile uint32_t g_k230_rx_byte_count;
extern volatile uint8_t g_k230_link_alive;
extern volatile uint8_t g_k230_target_valid;
extern volatile uint32_t g_k230_last_frame_ms;
extern volatile uint32_t g_k230_timeout_count;

/**
 * 初始化 K230 协议接收状态。
 */
void Protocol_K230_Init(void);

/**
 * 从 UART 接收缓冲区读取数据并组装完整文本帧。
 * 该函数不会阻塞，应在主循环中反复调用。
 */
void Protocol_K230_Task(void);

/**
 * 解析一行 K230 消息。
 *
 * 支持：
 * T,error_x,error_y,confidence
 * N
 *
 * 返回 1 表示解析成功，返回 0 表示格式或数值无效。
 */
uint8_t Protocol_K230_ParseLine(
    const char *line,
    K230_TargetFrame *frame
);

/**
 * 获取最近收到的新数据帧。
 *
 * 返回 1 表示存在新帧，并将新帧写入 frame；成功读取会清除“新帧”标志。
 * 返回 0 表示自上次读取后没有新帧。
 */
uint8_t Protocol_K230_TakeLatestFrame(K230_TargetFrame *frame);

/**
 * 获取累计有效帧数量，供 CCS Watch 调试。
 */
uint32_t Protocol_K230_GetValidFrameCount(void);

/**
 * 获取累计无效帧数量，供 CCS Watch 调试。
 */
uint32_t Protocol_K230_GetInvalidFrameCount(void);

#endif
