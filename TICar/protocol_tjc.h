/**
 * @file protocol_tjc.h
 * @brief TJC 串口屏固定长度请求帧、响应结果码、调试统计和协议任务接口。
 */
#ifndef PROTOCOL_TJC_H
#define PROTOCOL_TJC_H

#include <stdint.h>

// 命令	 功能	        串口屏弹起事件
// 00	安全停止	    printh 5A A5 00 FF
// 01	心跳灯模式	    printh 5A A5 01 FE
// 02	K230 UART 模式	printh 5A A5 02 FD
// 03	电机 PWM 测试	printh 5A A5 03 FC
// 04	CCD ADC 输出	printh 5A A5 04 FB
// 05	UART 测试	    printh 5A A5 05 FA
// 06	CCD Watch 模式	printh 5A A5 06 F9
// 07	普通巡线	    printh 5A A5 07 F8
// 08	CCD 固定直行	printh 5A A5 08 F7
// 09	编码器 Watch	printh 5A A5 09 F6
// 0A	速度闭环测试	printh 5A A5 0A F5
// 0B	圆形赛道巡线	printh 5A A5 0B F4
// 0C	K230 云台跟随	printh 5A A5 0C F3
// 0D	方形赛道巡线	printh 5A A5 0D F2
// 0E	查询当前模式	printh 5A A5 0E F1
// 0F	开始药房数字识别	printh 5A A5 0F F0
// 10	确认出发路线	printh 5A A5 10 EF
// 11	复位送药任务	printh 5A A5 11 EE

/*
 * 屏幕请求帧固定为四字节：
 * [0x5A, 0xA5, COMMAND, CHECKSUM]。
 * CHECKSUM = 0x5A ^ 0xA5 ^ COMMAND，解析器只处理帧头和校验均正确的完整帧。
 */
#define TJC_REQUEST_HEADER_0 0x5AU
#define TJC_REQUEST_HEADER_1 0xA5U
#define TJC_REQUEST_FRAME_SIZE 4U

/* COMMAND：0x00 停止，0x01～0x0D 对应 APP_MODE 1～13，0x0E 查询当前状态。 */
#define TJC_COMMAND_STOP 0x00U
#define TJC_COMMAND_FIRST_MODE 0x01U
#define TJC_COMMAND_LAST_MODE 0x0DU
#define TJC_COMMAND_QUERY 0x0EU
#define TJC_COMMAND_DELIVERY_IDENTIFY 0x0FU
#define TJC_COMMAND_DELIVERY_DEPART 0x10U
#define TJC_COMMAND_DELIVERY_RESET 0x11U

/* 滑杆继续使用固定四字节帧，COMMAND 直接携带 30～100 的速度百分比。 */
#define TJC_SPEED_PERCENT_MIN 30U
#define TJC_SPEED_PERCENT_MAX 100U
#define TJC_SPEED_PERCENT_DEFAULT 70U

/* MCU 五字节响应帧固定以 0xA5 开头。 */
#define TJC_RESPONSE_HEADER 0xA5U

/* 响应结果码区分“请求已受理”和“模式已经完成进入”两个异步阶段。 */
typedef enum {
    TJC_RESULT_STATE = 0x10U,            /* 上电或 QUERY 返回当前状态。 */
    TJC_RESULT_ACCEPTED_BRAKING = 0x11U, /* 请求已受理，可能正在执行制动等待。 */
    TJC_RESULT_SWITCH_OK = 0x12U,        /* 新模式入口已成功完成。 */
    TJC_RESULT_STOPPED = 0x13U,          /* 已处于安全停止态。 */
    TJC_RESULT_ALREADY_ACTIVE = 0x14U,   /* 请求模式已经在运行，无需切换。 */
    TJC_RESULT_TASK_ACCEPTED = 0x15U,     /* 送药任务命令已受理。 */
    TJC_RESULT_SPEED_UPDATED = 0x16U,     /* 调速百分比已更新。 */
    TJC_RESULT_INVALID_COMMAND = 0xE0U,  /* 命令字不在支持范围。 */
    TJC_RESULT_ENTER_FAILED = 0xE1U,     /* 目标模式入口失败并回退停止态。 */
    TJC_RESULT_TASK_REJECTED = 0xE2U     /* 送药任务状态不满足命令条件。 */
} Tjc_Result;

/*
 * CCS Watch 累计统计及最近一次命令/结果：
 * invalid_frame 统计帧头已经同步但 XOR 校验失败的帧；invalid_command 只统计校验正确但不支持的命令。
 * 溢出和 UART 错误值从 BSP 镜像。
 */
extern volatile uint32_t g_tjc_rx_byte_count;
extern volatile uint32_t g_tjc_valid_command_count;
extern volatile uint32_t g_tjc_invalid_command_count;
extern volatile uint32_t g_tjc_invalid_frame_count;
extern volatile uint32_t g_tjc_response_count;
extern volatile uint32_t g_tjc_rx_overflow_count;
extern volatile uint32_t g_tjc_rx_error_count;
extern volatile uint8_t g_tjc_last_command;
extern volatile uint8_t g_tjc_last_result;
extern volatile uint8_t g_tjc_speed_percent;

/** 清零协议统计并丢弃初始化前积压的 TJC 软件缓冲数据。 */
void Protocol_Tjc_Init(void);
/**
 * 非阻塞消费 UART 软件缓冲并按四字节请求帧同步、校验和解析。
 * 查询立即回复；模式切换最终结果由应用模式管理器稍后回复。
 */
void Protocol_Tjc_Task(void);

/** @return 当前巡线速度百分比，范围为 30～100。 */
uint8_t Protocol_Tjc_GetSpeedPercent(void);
/**
 * 发送五字节响应：[0xA5, result, current_mode, request, 前四字节 XOR]。
 * 请求帧和响应帧分别独立计算各自的 XOR 校验。
 */
void Protocol_Tjc_SendResult(Tjc_Result result, uint8_t current_mode, uint8_t request);

#endif
