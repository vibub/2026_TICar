#include "protocol_k230.h"

#include <stdio.h>
#include <string.h>

#include "bsp_uart.h"

#define K230_RX_LINE_SIZE (32U)

static char g_rxLine[K230_RX_LINE_SIZE];
static uint8_t g_rxIndex;

static K230_DetectFrame g_latestFrame;
static uint8_t g_freshFrame;

static uint32_t g_validFrameCount;
static uint32_t g_invalidFrameCount;


void Protocol_K230_Init(void)
{
    g_rxIndex = 0U;
    g_freshFrame = 0U;

    g_latestFrame.detected = 0U;
    g_latestFrame.error_x = 0;
    g_latestFrame.error_y = 0;
    g_latestFrame.confidence = 0U;

    g_validFrameCount = 0U;
    g_invalidFrameCount = 0U;
}


uint8_t Protocol_K230_ParseLine(
    const char *line,
    K230_DetectFrame *frame)
{
    int error_x;
    int error_y;
    int confidence;
    char extra;

    if ((line == NULL) || (frame == NULL)) {
        return 0U;
    }

    /*
     * K230没有检测到目标时发送：
     * N\r\n
     */
    if (strcmp(line, "N") == 0) {
        frame->detected = 0U;
        frame->error_x = 0;
        frame->error_y = 0;
        frame->confidence = 0U;

        return 1U;
    }

    /*
     * K230检测到目标时发送：
     * T,error_x,error_y,confidence\r\n
     *
     * 使用额外的%c检查行尾，避免接受字段后还有垃圾字符的数据。
     */
    if (sscanf(
            line,
            "T,%d,%d,%d%c",
            &error_x,
            &error_y,
            &confidence,
            &extra) != 3) {
        return 0U;
    }

    /*
     * 图像分辨率为640×360，因此坐标偏差有效范围为：
     * X：-320～320
     * Y：-180～180
     */
    if ((error_x < -320) || (error_x > 320)) {
        return 0U;
    }

    if ((error_y < -180) || (error_y > 180)) {
        return 0U;
    }

    if ((confidence < 0) || (confidence > 100)) {
        return 0U;
    }

    frame->detected = 1U;
    frame->error_x = (int16_t) error_x;
    frame->error_y = (int16_t) error_y;
    frame->confidence = (uint8_t) confidence;

    return 1U;
}


static void Protocol_K230_ProcessLine(void)
{
    K230_DetectFrame frame;

    g_rxLine[g_rxIndex] = '\0';

    if (Protocol_K230_ParseLine(g_rxLine, &frame) != 0U) {
        g_latestFrame = frame;
        g_freshFrame = 1U;
        g_validFrameCount++;

        if (frame.detected != 0U) {
            Bsp_Uart_K230_SendString("ACK,T\r\n");
        } else {
            Bsp_Uart_K230_SendString("ACK,N\r\n");
        }
    } else {
        g_invalidFrameCount++;
        Bsp_Uart_K230_SendString("ERR,FRAME\r\n");
    }

    g_rxIndex = 0U;
}


void Protocol_K230_Task(void)
{
    uint8_t byte;
    uint32_t error_status;

    /*
     * 先处理UART硬件错误。
     */
    error_status = Bsp_Uart_K230_GetErrorStatus();

    if (error_status != 0U) {
        Bsp_Uart_K230_ClearErrorStatus(error_status);
        g_rxIndex = 0U;
        g_invalidFrameCount++;

        Bsp_Uart_K230_SendString("ERR,UART\r\n");
    }

    /*
     * 一次循环读完当前RX FIFO中的全部字节。
     * 没有数据时立即返回，不阻塞主循环。
     */
    while (Bsp_Uart_K230_TryReceiveByte(&byte) != 0U) {
        /*
         * K230发送的是\r\n。
         * 忽略\r，以\n作为一帧结束标志。
         */
        if (byte == (uint8_t) '\r') {
            continue;
        }

        if (byte == (uint8_t) '\n') {
            if (g_rxIndex != 0U) {
                Protocol_K230_ProcessLine();
            }

            continue;
        }

        /*
         * 保留一个字节用于字符串结尾的'\0'。
         */
        if (g_rxIndex < (K230_RX_LINE_SIZE - 1U)) {
            g_rxLine[g_rxIndex] = (char) byte;
            g_rxIndex++;
        } else {
            /*
             * 数据帧超过缓冲区，直接丢弃。
             */
            g_rxIndex = 0U;
            g_invalidFrameCount++;

            Bsp_Uart_K230_SendString("ERR,OVERFLOW\r\n");
        }
    }
}


const K230_DetectFrame *Protocol_K230_GetLatestFrame(void)
{
    return &g_latestFrame;
}


uint8_t Protocol_K230_HasFreshFrame(void)
{
    return g_freshFrame;
}


void Protocol_K230_ClearFreshFrame(void)
{
    g_freshFrame = 0U;
}


uint32_t Protocol_K230_GetValidFrameCount(void)
{
    return g_validFrameCount;
}


uint32_t Protocol_K230_GetInvalidFrameCount(void)
{
    return g_invalidFrameCount;
}
