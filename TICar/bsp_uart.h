/**
 * @file bsp_uart.h
 * @brief K230 和 TJC 两路 UART 的发送、接收及诊断接口。
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/** 初始化 K230/TJC 软件环形缓冲区，并使能两路 UART NVIC 中断。 */
void Bsp_Uart_Init(void);

/* K230 UART：RX ISR 写入 128 字节软件缓冲，协议层通过 TryReceiveByte 消费。 */
void Bsp_Uart_K230_SendByte(uint8_t byte);
void Bsp_Uart_K230_SendString(const char *str);
/** 阻塞等待并返回一个 K230 字节，仅适合明确需要同步等待的场景。 */
uint8_t Bsp_Uart_K230_ReceiveByteBlocking(void);
/** @return 1 表示从软件环形缓冲取出字节；0 表示参数为空或缓冲为空。 */
uint8_t Bsp_Uart_K230_TryReceiveByte(uint8_t *byte);
/** 丢弃尚未由协议层消费的 K230 字节，不清零累计溢出次数。 */
void Bsp_Uart_K230_FlushRx(void);
/** 返回 K230 软件环形缓冲累计溢出次数，正常运行应保持为 0。 */
uint32_t Bsp_Uart_K230_GetOverflowCount(void);
/** 读取错误中断原始状态；处理后应把返回位传给 ClearErrorStatus。 */
uint32_t Bsp_Uart_K230_GetErrorStatus(void);
void Bsp_Uart_K230_ClearErrorStatus(uint32_t status);

/* TJC UART：RX 由 ISR 写入软件环形缓冲，主循环通过 TryReceiveByte 消费。TX 仍采用忙等发送。 */
void Bsp_Uart_Tjc_SendByte(uint8_t byte);
/** 顺序发送 length 个字节；data 为空时直接返回。 */
void Bsp_Uart_Tjc_SendData(const uint8_t *data, uint16_t length);
/** @return 1 表示从软件缓冲取出一个字节；0 表示参数为空或缓冲为空。 */
uint8_t Bsp_Uart_Tjc_TryReceiveByte(uint8_t *byte);
/** 丢弃尚未消费的 TJC 字节，不清零溢出和错误累计计数。 */
void Bsp_Uart_Tjc_FlushRx(void);
uint32_t Bsp_Uart_Tjc_GetOverflowCount(void);
uint32_t Bsp_Uart_Tjc_GetErrorCount(void);

#endif
