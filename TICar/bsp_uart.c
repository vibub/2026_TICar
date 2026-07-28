/**
 * @file bsp_uart.c
 * @brief K230、TJC 与双车蓝牙三路中断 UART 的底层收发实现。
 *
 * 三路 UART ISR 都只负责排空硬件 FIFO 并写入独立环形缓冲；主循环负责协议解析，
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
#define BSP_UART_BT_ERROR_MASK BSP_UART_K230_ERROR_MASK
/* K230 遥测会连续发送 L/D 文本帧，使用 128 槽缓冲覆盖一次完整突发。 */
#define BSP_UART_K230_RX_BUFFER_SIZE 128U
/* TJC 请求固定为四字节，16 槽足以覆盖连续按键命令。 */
#define BSP_UART_TJC_RX_BUFFER_SIZE 16U
/* 蓝牙位姿速度帧为 23 字节，64 槽可覆盖两帧连续突发并保留余量。 */
#define BSP_UART_BT_RX_BUFFER_SIZE 64U

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
static volatile uint8_t g_bt_rx_buffer[BSP_UART_BT_RX_BUFFER_SIZE];
static volatile uint8_t g_bt_rx_head;
static volatile uint8_t g_bt_rx_tail;
static volatile uint32_t g_bt_rx_overflow_count;
static volatile uint32_t g_bt_rx_error_count;

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

static void Bsp_Uart_Bluetooth_PushRx(uint8_t byte)
{
    uint8_t next_head = (uint8_t) (
        (g_bt_rx_head + 1U) % BSP_UART_BT_RX_BUFFER_SIZE);

    if (next_head == g_bt_rx_tail) {
        g_bt_rx_overflow_count++;
        return;
    }

    g_bt_rx_buffer[g_bt_rx_head] = byte;
    g_bt_rx_head = next_head;
}

/* 清空三路软件接收状态后开启 NVIC；UART RX 中断源由 SysConfig 配置。 */
void Bsp_Uart_Init(void)
{
    g_k230_rx_head = 0U;
    g_k230_rx_tail = 0U;
    g_k230_rx_overflow_count = 0U;
    g_tjc_rx_head = 0U;
    g_tjc_rx_tail = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;
    g_bt_rx_head = 0U;
    g_bt_rx_tail = 0U;
    g_bt_rx_overflow_count = 0U;
    g_bt_rx_error_count = 0U;

    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_TJC3224T124_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TJC3224T124_INST_INT_IRQN);
#if defined(UART_HC05_INST)
    NVIC_ClearPendingIRQ(UART_HC05_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_HC05_INST_INT_IRQN);
#endif
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

uint8_t Bsp_Uart_Bluetooth_IsAvailable(void)
{
#if defined(UART_HC05_INST)
    return 1U;
#else
    return 0U;
#endif
}

uint8_t Bsp_Uart_Bluetooth_SendByte(uint8_t byte)
{
#if defined(UART_HC05_INST)
    while (DL_UART_Main_isTXFIFOFull(UART_HC05_INST)) {
    }
    DL_UART_Main_transmitData(UART_HC05_INST, byte);
    return 1U;
#else
    (void) byte;
    return 0U;
#endif
}

uint8_t Bsp_Uart_Bluetooth_SendData(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (Bsp_Uart_Bluetooth_IsAvailable() == 0U)) {
        return 0U;
    }

    for (i = 0U; i < length; i++) {
        (void) Bsp_Uart_Bluetooth_SendByte(data[i]);
    }
    return 1U;
}

uint8_t Bsp_Uart_Bluetooth_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (g_bt_rx_tail == g_bt_rx_head)) {
        return 0U;
    }

    *byte = g_bt_rx_buffer[g_bt_rx_tail];
    g_bt_rx_tail = (uint8_t) (
        (g_bt_rx_tail + 1U) % BSP_UART_BT_RX_BUFFER_SIZE);
    return 1U;
}

void Bsp_Uart_Bluetooth_FlushRx(void)
{
    g_bt_rx_tail = g_bt_rx_head;
}

uint32_t Bsp_Uart_Bluetooth_GetOverflowCount(void)
{
    return g_bt_rx_overflow_count;
}

uint32_t Bsp_Uart_Bluetooth_GetErrorCount(void)
{
    return g_bt_rx_error_count;
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

#if defined(UART_HC05_INST)
/* 蓝牙 ISR 只排空 FIFO 和记录错误，二进制帧解析始终放在主循环完成。 */
void UART_HC05_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    while ((interrupt_index = DL_UART_Main_getPendingInterrupt(UART_HC05_INST)) !=
           DL_UART_MAIN_IIDX_NO_INTERRUPT) {
        switch (interrupt_index) {
            case DL_UART_MAIN_IIDX_RX:
                while (!DL_UART_isRXFIFOEmpty(UART_HC05_INST)) {
                    Bsp_Uart_Bluetooth_PushRx(
                        DL_UART_Main_receiveDataBlocking(UART_HC05_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                g_bt_rx_error_count++;
                DL_UART_Main_clearInterruptStatus(
                    UART_HC05_INST, BSP_UART_BT_ERROR_MASK);
                while (!DL_UART_isRXFIFOEmpty(UART_HC05_INST)) {
                    (void) DL_UART_Main_receiveDataBlocking(UART_HC05_INST);
                }
                break;

            default:
                break;
        }
    }
}
#endif
