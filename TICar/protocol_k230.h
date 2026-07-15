#ifndef PROTOCOL_K230_H
#define PROTOCOL_K230_H

#include <stdint.h>

#define K230_PROTOCOL_LINE_SIZE 32U

typedef struct {
    uint8_t detected;
    int16_t error_x;
    int16_t error_y;
    uint8_t confidence;
} K230_TargetFrame;

/* 供 CCS Watch 观察的协议调试变量。 */
extern volatile K230_TargetFrame g_k230_latest_frame;
extern volatile uint32_t g_k230_valid_frame_count;
extern volatile uint32_t g_k230_invalid_frame_count;
extern volatile uint32_t g_k230_rx_byte_count;

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
 * 返回 1 表示存在新帧，并将新帧写入 frame。
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
