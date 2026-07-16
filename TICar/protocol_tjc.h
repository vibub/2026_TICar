/**
 * @file protocol_tjc.h
 * @brief TJC 串口屏命令字、响应结果码、调试统计和协议任务接口。
 */
#ifndef PROTOCOL_TJC_H
#define PROTOCOL_TJC_H

#include <stdint.h>

/* 单字节请求：0x00 停止，0x01～0x0D 对应 APP_MODE 1～13，0x0E 只查询当前状态。 */
#define TJC_COMMAND_STOP 0x00U
#define TJC_COMMAND_FIRST_MODE 0x01U
#define TJC_COMMAND_LAST_MODE 0x0DU
#define TJC_COMMAND_QUERY 0x0EU
/* MCU 五字节响应帧固定以 0xA5 开头。 */
#define TJC_RESPONSE_HEADER 0xA5U

/* 响应结果码区分“请求已受理”和“模式已经完成进入”两个异步阶段。 */
typedef enum {
    TJC_RESULT_STATE = 0x10U,            /* 上电或 QUERY 返回当前状态。 */
    TJC_RESULT_ACCEPTED_BRAKING = 0x11U, /* 请求已受理，可能正在执行制动等待。 */
    TJC_RESULT_SWITCH_OK = 0x12U,        /* 新模式入口已成功完成。 */
    TJC_RESULT_STOPPED = 0x13U,          /* 已处于安全停止态。 */
    TJC_RESULT_ALREADY_ACTIVE = 0x14U,   /* 请求模式已经在运行，无需切换。 */
    TJC_RESULT_INVALID_COMMAND = 0xE0U,  /* 命令字不在支持范围。 */
    TJC_RESULT_ENTER_FAILED = 0xE1U      /* 目标模式入口失败并回退停止态。 */
} Tjc_Result;

/* CCS Watch 累计统计及最近一次命令/结果；溢出和 UART 错误值从 BSP 镜像。 */
extern volatile uint32_t g_tjc_rx_byte_count;
extern volatile uint32_t g_tjc_valid_command_count;
extern volatile uint32_t g_tjc_invalid_command_count;
extern volatile uint32_t g_tjc_response_count;
extern volatile uint32_t g_tjc_rx_overflow_count;
extern volatile uint32_t g_tjc_rx_error_count;
extern volatile uint8_t g_tjc_last_command;
extern volatile uint8_t g_tjc_last_result;

/** 清零协议统计并丢弃初始化前积压的 TJC 软件缓冲数据。 */
void Protocol_Tjc_Init(void);
/** 非阻塞消费所有已入队命令；查询立即回复，模式切换最终结果由应用模式管理器稍后回复。 */
void Protocol_Tjc_Task(void);
/**
 * 发送五字节响应：[0xA5, result, current_mode, request, 前四字节 XOR]。
 * XOR 仅用于 MCU 响应；屏幕发来的请求本身是无校验的单字节。
 */
void Protocol_Tjc_SendResult(Tjc_Result result, uint8_t current_mode, uint8_t request);

#endif
