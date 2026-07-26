/**
 * @file bsp_uart.c
 * @brief K230 与 TJC 两路中断 UART 的底层收发实现。
 *
 * 两路 UART ISR 都只负责排空硬件 FIFO 并写入独立环形缓冲；主循环负责协议解析，
 * 避免 IMU I2C 轮询期间 4 字节 K230 硬件 FIFO 溢出并截断数字 D 帧。
 */
#include "bsp_uart.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define BSP_UART_K230_ERROR_MASK                                           \
    ((uint32_t) (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                      \
                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                     \
                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                    \
                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR))
/* K230 遥测会连续发送 L/D 文本帧，使用 128 槽缓冲覆盖一次完整突发。 */
#define BSP_UART_K230_RX_BUFFER_SIZE 128U
/* TJC 请求固定为四字节，16 槽足以覆盖连续按键命令。 */
#define BSP_UART_TJC_RX_BUFFER_SIZE 16U

/* head 只由对应 UART ISR 推进，tail 只由主循环推进。 */
static volatile uint8_t g_k230_rx_buffer[BSP_UART_K230_RX_BUFFER_SIZE];
static volatile uint8_t g_k230_rx_head;
static volatile uint8_t g_k230_rx_tail;
static volatile uint32_t g_k230_rx_overflow_count;
static volatile uint8_t g_tjc_rx_buffer[BSP_UART_TJC_RX_BUFFER_SIZE];
static volatile uint8_t g_tjc_rx_head;
static volatile uint8_t g_tjc_rx_tail;
static volatile uint32_t g_tjc_rx_overflow_count;
static volatile uint32_t g_tjc_rx_error_count;

/* K230 ISR 专用入队；缓冲满时保留旧帧并累计溢出，协议层随后重新同步 LF。 */
static void Bsp_Uart_K230_PushRx(uint8_t byte)
{
    uint8_t next_head = (uint8_t) (
        (g_k230_rx_head + 1U) % BSP_UART_K230_RX_BUFFER_SIZE);

    if (next_head == g_k230_rx_tail) {
        g_k230_rx_overflow_count++;
        return;
    }

    g_k230_rx_buffer[g_k230_rx_head] = byte;
    g_k230_rx_head = next_head;
}

/* TJC ISR 专用入队函数。next_head 追上 tail 表示缓冲已满。 */
static void Bsp_Uart_Tjc_PushRx(uint8_t byte)
{
    uint8_t next_head = (uint8_t) ((g_tjc_rx_head + 1U) % BSP_UART_TJC_RX_BUFFER_SIZE);

    if (next_head == g_tjc_rx_tail) {
        g_tjc_rx_overflow_count++;
        return;
    }

    g_tjc_rx_buffer[g_tjc_rx_head] = byte;
    g_tjc_rx_head = next_head;
}

/* 清空两路软件接收状态后开启 NVIC；UART RX 中断源由 SysConfig 配置。 */
void Bsp_Uart_Init(void)
{
    g_k230_rx_head = 0U;
    g_k230_rx_tail = 0U;
    g_k230_rx_overflow_count = 0U;
    g_tjc_rx_head = 0U;
    g_tjc_rx_tail = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;

    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_TJC3224T124_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TJC3224T124_INST_INT_IRQN);
}

void Bsp_Uart_K230_SendByte(uint8_t byte)
{
    while (DL_UART_Main_isTXFIFOFull(UART_K230_INST)) {
    }

    DL_UART_Main_transmitData(UART_K230_INST, byte);
}

void Bsp_Uart_K230_SendString(const char *str)
{
    while (*str != '\0') {
        Bsp_Uart_K230_SendByte((uint8_t) *str);
        str++;
    }
}

uint8_t Bsp_Uart_K230_ReceiveByteBlocking(void)
{
    uint8_t byte;

    while (Bsp_Uart_K230_TryReceiveByte(&byte) == 0U) {
    }
    return byte;
}

uint8_t Bsp_Uart_K230_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_k230_rx_tail == g_k230_rx_head)) {
        return 0U;
    }

    *byte = g_k230_rx_buffer[g_k230_rx_tail];
    g_k230_rx_tail = (uint8_t) (
        (g_k230_rx_tail + 1U) % BSP_UART_K230_RX_BUFFER_SIZE);
    return 1U;
}

void Bsp_Uart_K230_FlushRx(void)
{
    g_k230_rx_tail = g_k230_rx_head;
}

uint32_t Bsp_Uart_K230_GetOverflowCount(void)
{
    return g_k230_rx_overflow_count;
}

uint32_t Bsp_Uart_K230_GetErrorStatus(void)
{
    return DL_UART_Main_getRawInterruptStatus(UART_K230_INST, BSP_UART_K230_ERROR_MASK);
}

void Bsp_Uart_K230_ClearErrorStatus(uint32_t status)
{
    DL_UART_Main_clearInterruptStatus(UART_K230_INST, status);
}

void Bsp_Uart_Tjc_SendByte(uint8_t byte)
{
    while (DL_UART_Main_isTXFIFOFull(UART_TJC3224T124_INST)) {
    }

    DL_UART_Main_transmitData(UART_TJC3224T124_INST, byte);
}

void Bsp_Uart_Tjc_SendData(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == NULL) {
        return;
    }

    for (i = 0U; i < length; i++) {
        Bsp_Uart_Tjc_SendByte(data[i]);
    }
}

uint8_t Bsp_Uart_Tjc_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_tjc_rx_tail == g_tjc_rx_head)) {
        return 0U;
    }

    *byte = g_tjc_rx_buffer[g_tjc_rx_tail];
    g_tjc_rx_tail = (uint8_t) ((g_tjc_rx_tail + 1U) % BSP_UART_TJC_RX_BUFFER_SIZE);
    return 1U;
}

void Bsp_Uart_Tjc_FlushRx(void)
{
    g_tjc_rx_tail = g_tjc_rx_head;
}

uint32_t Bsp_Uart_Tjc_GetOverflowCount(void)
{
    return g_tjc_rx_overflow_count;
}

uint32_t Bsp_Uart_Tjc_GetErrorCount(void)
{
    return g_tjc_rx_error_count;
}

/*
 * K230 UART ISR：每次 RX 中断完整排空 4 字节硬件 FIFO，随后由主循环解析文本帧。
 * 这使 IMU I2C 轮询和速度控制不会再截断较长的数字 D 帧。
 */
void UART_K230_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    while ((interrupt_index = DL_UART_Main_getPendingInterrupt(UART_K230_INST)) !=
           DL_UART_MAIN_IIDX_NO_INTERRUPT) {
        if (interrupt_index == DL_UART_MAIN_IIDX_RX) {
            while (!DL_UART_isRXFIFOEmpty(UART_K230_INST)) {
                Bsp_Uart_K230_PushRx(
                    DL_UART_Main_receiveDataBlocking(UART_K230_INST));
            }
        }
    }
}

/*
 * TJC UART3 ISR：RX 中断尽可能排空硬件 FIFO并入队；通信错误则累计计数并丢弃 FIFO 中的不可靠字节。
 * ISR 不做协议解析、不发送响应、不调用模式管理；这些工作由主循环 Protocol_Tjc_Task() 完成。
 */
void UART_TJC3224T124_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    while ((interrupt_index = DL_UART_Main_getPendingInterrupt(UART_TJC3224T124_INST)) !=
           DL_UART_MAIN_IIDX_NO_INTERRUPT) {
        switch (interrupt_index) {
            case DL_UART_MAIN_IIDX_RX:
                while (!DL_UART_isRXFIFOEmpty(UART_TJC3224T124_INST)) {
                    Bsp_Uart_Tjc_PushRx(
                        DL_UART_Main_receiveDataBlocking(UART_TJC3224T124_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                g_tjc_rx_error_count++;
                while (!DL_UART_isRXFIFOEmpty(UART_TJC3224T124_INST)) {
                    (void) DL_UART_Main_receiveDataBlocking(UART_TJC3224T124_INST);
                }
                break;

            default:
                break;
        }
    }
}
