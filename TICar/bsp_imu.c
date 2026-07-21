/**
 * @file bsp_imu.c
 * @brief SysConfig UART_IMU 实例与纯协议解析器之间的非阻塞桥接。
 */
#include "bsp_imu.h"

#include "protocol_imu.h"
#include "ti_msp_dl_config.h"

#if defined(UART_IMU_INST)
#define BSP_IMU_RX_BUFFER_SIZE 256U
#define BSP_IMU_RX_BUFFER_MASK (BSP_IMU_RX_BUFFER_SIZE - 1U)
#define BSP_IMU_ERROR_MASK                                               \
    ((uint32_t) (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                 \
                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                   \
                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                  \
                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                 \
                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR))

static volatile uint8_t g_imu_rx_buffer[BSP_IMU_RX_BUFFER_SIZE];
static volatile uint16_t g_imu_rx_head;
static volatile uint16_t g_imu_rx_tail;
volatile uint32_t g_imu_uart_irq_count;
volatile uint32_t g_imu_uart_overflow_count;
volatile uint32_t g_imu_uart_error_count;

static void Bsp_Imu_PushRx(uint8_t byte)
{
    uint16_t next_head = (uint16_t) ((g_imu_rx_head + 1U) & BSP_IMU_RX_BUFFER_MASK);

    if (next_head == g_imu_rx_tail) {
        g_imu_uart_overflow_count++;
        return;
    }
    g_imu_rx_buffer[g_imu_rx_head] = byte;
    g_imu_rx_head = next_head;
}

static void Bsp_Imu_SendByte(uint8_t byte)
{
    while (DL_UART_Main_isTXFIFOFull(UART_IMU_INST)) {
    }
    DL_UART_Main_transmitData(UART_IMU_INST, byte);
}

static void Bsp_Imu_SendString(const char *text)
{
    while (*text != '\0') {
        Bsp_Imu_SendByte((uint8_t) *text++);
    }
}
#endif

void Bsp_Imu_Init(void)
{
    Protocol_Imu_Init();

#if defined(UART_IMU_INST)
    g_imu_rx_head = 0U;
    g_imu_rx_tail = 0U;
    g_imu_uart_irq_count = 0U;
    g_imu_uart_overflow_count = 0U;
    g_imu_uart_error_count = 0U;
    g_imu.hardware_ready = 1U;
    NVIC_ClearPendingIRQ(UART_IMU_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
#endif
}

void Bsp_Imu_Task(void)
{
#if defined(UART_IMU_INST)
    while (g_imu_rx_tail != g_imu_rx_head) {
        uint8_t byte = g_imu_rx_buffer[g_imu_rx_tail];
        g_imu_rx_tail = (uint16_t) ((g_imu_rx_tail + 1U) & BSP_IMU_RX_BUFFER_MASK);
        Protocol_Imu_FeedByte(byte);
    }
#endif
}

uint8_t Bsp_Imu_IsHardwareReady(void)
{
    return g_imu.hardware_ready;
}

void Bsp_Imu_SendLegacyStartCommands(void)
{
#if defined(UART_IMU_INST)
    Bsp_Imu_SendString("AT+MODE=0\r\n");
    delay_cycles(160000U);
    Bsp_Imu_SendString("AT+RST\r\n");
#endif
}

#if defined(UART_IMU_INST)
void UART_IMU_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    g_imu_uart_irq_count++;
    while ((interrupt_index = DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) !=
           DL_UART_MAIN_IIDX_NO_INTERRUPT) {
        switch (interrupt_index) {
            case DL_UART_MAIN_IIDX_RX:
                while (!DL_UART_isRXFIFOEmpty(UART_IMU_INST)) {
                    Bsp_Imu_PushRx(DL_UART_Main_receiveDataBlocking(UART_IMU_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                g_imu_uart_error_count++;
                DL_UART_Main_clearInterruptStatus(UART_IMU_INST, BSP_IMU_ERROR_MASK);
                while (!DL_UART_isRXFIFOEmpty(UART_IMU_INST)) {
                    (void) DL_UART_Main_receiveDataBlocking(UART_IMU_INST);
                }
                break;

            default:
                break;
        }
    }
}
#endif
