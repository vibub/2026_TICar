/**
 * @file protocol_bt.c
 * @brief 双车蓝牙位姿与速度协议的编码、解码和非阻塞流解析。
 */
#include "protocol_bt.h"

#include <stddef.h>
#include <string.h>

#include "bsp_time.h"
#include "bsp_uart.h"

#define BT_OFFSET_VERSION       2U
#define BT_OFFSET_TYPE          3U
#define BT_OFFSET_PAYLOAD_SIZE  4U
#define BT_OFFSET_SEQUENCE      5U
#define BT_OFFSET_PAYLOAD       7U
#define BT_OFFSET_X_MM          7U
#define BT_OFFSET_Y_MM          11U
#define BT_OFFSET_YAW_CDEG      15U
#define BT_OFFSET_LINEAR_MM_S   17U
#define BT_OFFSET_ANGULAR_CDEG_S 19U
#define BT_OFFSET_CRC           21U
#define BT_CRC_INPUT_SIZE       19U

volatile BtPoseVelocity g_bt_latest_pose_velocity;
volatile uint16_t g_bt_latest_rx_sequence;
volatile uint16_t g_bt_tx_sequence;
volatile uint32_t g_bt_rx_valid_frame_count;
volatile uint32_t g_bt_rx_invalid_frame_count;
volatile uint32_t g_bt_rx_crc_error_count;
volatile uint32_t g_bt_rx_byte_count;
volatile uint32_t g_bt_last_rx_ms;

static uint8_t g_bt_new_pose_velocity;
static uint8_t g_bt_rx_frame[BT_PROTOCOL_MAX_FRAME_SIZE];
static uint16_t g_bt_rx_index;
static uint16_t g_bt_expected_frame_size;
static uint32_t g_bt_seen_uart_overflow_count;
static uint32_t g_bt_seen_uart_error_count;

static void Protocol_Bt_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
}

static void Protocol_Bt_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
    data[2] = (uint8_t) (value >> 16U);
    data[3] = (uint8_t) (value >> 24U);
}

static uint16_t Protocol_Bt_ReadU16Le(const uint8_t *data)
{
    return (uint16_t) data[0] | ((uint16_t) data[1] << 8U);
}

static uint32_t Protocol_Bt_ReadU32Le(const uint8_t *data)
{
    return (uint32_t) data[0] |
           ((uint32_t) data[1] << 8U) |
           ((uint32_t) data[2] << 16U) |
           ((uint32_t) data[3] << 24U);
}

uint16_t Protocol_Bt_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }

    for (i = 0U; i < length; i++) {
        crc ^= (uint16_t) data[i] << 8U;
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t) ((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

uint16_t Protocol_Bt_EncodePoseVelocity(
    const BtPoseVelocity *data,
    uint16_t sequence,
    uint8_t *frame,
    uint16_t capacity)
{
    uint16_t crc;

    if ((data == NULL) || (frame == NULL) ||
        (capacity < BT_PROTOCOL_FRAME_SIZE)) {
        return 0U;
    }

    frame[0] = BT_PROTOCOL_SOF_0;
    frame[1] = BT_PROTOCOL_SOF_1;
    frame[BT_OFFSET_VERSION] = BT_PROTOCOL_VERSION;
    frame[BT_OFFSET_TYPE] = BT_PROTOCOL_TYPE_POSE_VELOCITY;
    frame[BT_OFFSET_PAYLOAD_SIZE] = BT_PROTOCOL_POSE_PAYLOAD_SIZE;
    Protocol_Bt_WriteU16Le(&frame[BT_OFFSET_SEQUENCE], sequence);
    Protocol_Bt_WriteU32Le(&frame[BT_OFFSET_X_MM], (uint32_t) data->x_mm);
    Protocol_Bt_WriteU32Le(&frame[BT_OFFSET_Y_MM], (uint32_t) data->y_mm);
    Protocol_Bt_WriteU16Le(&frame[BT_OFFSET_YAW_CDEG],
                           (uint16_t) data->yaw_cdeg);
    Protocol_Bt_WriteU16Le(&frame[BT_OFFSET_LINEAR_MM_S],
                           (uint16_t) data->linear_mm_s);
    Protocol_Bt_WriteU16Le(&frame[BT_OFFSET_ANGULAR_CDEG_S],
                           (uint16_t) data->angular_cdeg_s);

    crc = Protocol_Bt_Crc16(&frame[BT_OFFSET_VERSION], BT_CRC_INPUT_SIZE);
    Protocol_Bt_WriteU16Le(&frame[BT_OFFSET_CRC], crc);
    return BT_PROTOCOL_FRAME_SIZE;
}

uint8_t Protocol_Bt_DecodePoseVelocity(
    const uint8_t *frame,
    uint16_t length,
    BtPoseVelocity *data,
    uint16_t *sequence)
{
    uint16_t expected_crc;
    uint16_t received_crc;

    if ((frame == NULL) || (data == NULL) || (sequence == NULL) ||
        (length != BT_PROTOCOL_FRAME_SIZE) ||
        (frame[0] != BT_PROTOCOL_SOF_0) ||
        (frame[1] != BT_PROTOCOL_SOF_1) ||
        (frame[BT_OFFSET_VERSION] != BT_PROTOCOL_VERSION) ||
        (frame[BT_OFFSET_TYPE] != BT_PROTOCOL_TYPE_POSE_VELOCITY) ||
        (frame[BT_OFFSET_PAYLOAD_SIZE] != BT_PROTOCOL_POSE_PAYLOAD_SIZE)) {
        return 0U;
    }

    expected_crc = Protocol_Bt_Crc16(
        &frame[BT_OFFSET_VERSION], BT_CRC_INPUT_SIZE);
    received_crc = Protocol_Bt_ReadU16Le(&frame[BT_OFFSET_CRC]);
    if (expected_crc != received_crc) {
        return 0U;
    }

    *sequence = Protocol_Bt_ReadU16Le(&frame[BT_OFFSET_SEQUENCE]);
    data->x_mm = (int32_t) Protocol_Bt_ReadU32Le(&frame[BT_OFFSET_X_MM]);
    data->y_mm = (int32_t) Protocol_Bt_ReadU32Le(&frame[BT_OFFSET_Y_MM]);
    data->yaw_cdeg = (int16_t) Protocol_Bt_ReadU16Le(&frame[BT_OFFSET_YAW_CDEG]);
    data->linear_mm_s =
        (int16_t) Protocol_Bt_ReadU16Le(&frame[BT_OFFSET_LINEAR_MM_S]);
    data->angular_cdeg_s =
        (int16_t) Protocol_Bt_ReadU16Le(&frame[BT_OFFSET_ANGULAR_CDEG_S]);
    return 1U;
}

static void Protocol_Bt_ResetRxParser(void)
{
    g_bt_rx_index = 0U;
    g_bt_expected_frame_size = 0U;
}

static void Protocol_Bt_PublishFrame(void)
{
    BtPoseVelocity decoded;
    uint16_t sequence;
    uint16_t expected_crc;
    uint16_t received_crc;

    if (g_bt_expected_frame_size != BT_PROTOCOL_FRAME_SIZE) {
        g_bt_rx_invalid_frame_count++;
        return;
    }

    if (Protocol_Bt_DecodePoseVelocity(
            g_bt_rx_frame, g_bt_expected_frame_size,
            &decoded, &sequence) == 0U) {
        expected_crc = Protocol_Bt_Crc16(
            &g_bt_rx_frame[BT_OFFSET_VERSION], BT_CRC_INPUT_SIZE);
        received_crc = Protocol_Bt_ReadU16Le(&g_bt_rx_frame[BT_OFFSET_CRC]);
        if (expected_crc != received_crc) {
            g_bt_rx_crc_error_count++;
        }
        g_bt_rx_invalid_frame_count++;
        return;
    }

    g_bt_latest_pose_velocity = decoded;
    g_bt_latest_rx_sequence = sequence;
    g_bt_new_pose_velocity = 1U;
    g_bt_rx_valid_frame_count++;
    g_bt_last_rx_ms = Bsp_Time_GetMilliseconds();
}

static void Protocol_Bt_ParseByte(uint8_t byte)
{
    if (g_bt_rx_index == 0U) {
        if (byte == BT_PROTOCOL_SOF_0) {
            g_bt_rx_frame[0] = byte;
            g_bt_rx_index = 1U;
        }
        return;
    }

    if (g_bt_rx_index == 1U) {
        if (byte == BT_PROTOCOL_SOF_1) {
            g_bt_rx_frame[1] = byte;
            g_bt_rx_index = 2U;
        } else if (byte != BT_PROTOCOL_SOF_0) {
            Protocol_Bt_ResetRxParser();
        }
        return;
    }

    g_bt_rx_frame[g_bt_rx_index++] = byte;

    if (g_bt_rx_index == (BT_OFFSET_PAYLOAD_SIZE + 1U)) {
        uint16_t payload_size = g_bt_rx_frame[BT_OFFSET_PAYLOAD_SIZE];
        g_bt_expected_frame_size = (uint16_t) (
            BT_OFFSET_PAYLOAD + payload_size + 2U);
        if ((payload_size > (BT_PROTOCOL_MAX_FRAME_SIZE -
                             BT_OFFSET_PAYLOAD - 2U)) ||
            (g_bt_expected_frame_size > BT_PROTOCOL_MAX_FRAME_SIZE)) {
            g_bt_rx_invalid_frame_count++;
            Protocol_Bt_ResetRxParser();
            return;
        }
    }

    if ((g_bt_expected_frame_size != 0U) &&
        (g_bt_rx_index == g_bt_expected_frame_size)) {
        Protocol_Bt_PublishFrame();
        Protocol_Bt_ResetRxParser();
    }
}

void Protocol_Bt_Init(void)
{
    memset((void *) &g_bt_latest_pose_velocity, 0,
           sizeof(g_bt_latest_pose_velocity));
    g_bt_latest_rx_sequence = 0U;
    g_bt_tx_sequence = 0U;
    g_bt_rx_valid_frame_count = 0U;
    g_bt_rx_invalid_frame_count = 0U;
    g_bt_rx_crc_error_count = 0U;
    g_bt_rx_byte_count = 0U;
    g_bt_last_rx_ms = 0U;
    g_bt_new_pose_velocity = 0U;
    Protocol_Bt_ResetRxParser();
    Bsp_Uart_Bluetooth_FlushRx();
    g_bt_seen_uart_overflow_count =
        Bsp_Uart_Bluetooth_GetOverflowCount();
    g_bt_seen_uart_error_count = Bsp_Uart_Bluetooth_GetErrorCount();
}

void Protocol_Bt_Task(void)
{
    uint8_t byte;
    uint32_t overflow_count = Bsp_Uart_Bluetooth_GetOverflowCount();
    uint32_t error_count = Bsp_Uart_Bluetooth_GetErrorCount();

    if ((overflow_count != g_bt_seen_uart_overflow_count) ||
        (error_count != g_bt_seen_uart_error_count)) {
        g_bt_seen_uart_overflow_count = overflow_count;
        g_bt_seen_uart_error_count = error_count;
        Bsp_Uart_Bluetooth_FlushRx();
        Protocol_Bt_ResetRxParser();
        g_bt_rx_invalid_frame_count++;
        return;
    }

    while (Bsp_Uart_Bluetooth_TryReceiveByte(&byte) != 0U) {
        g_bt_rx_byte_count++;
        Protocol_Bt_ParseByte(byte);
    }
}

uint8_t Protocol_Bt_SendPoseVelocity(const BtPoseVelocity *data)
{
    uint8_t frame[BT_PROTOCOL_FRAME_SIZE];
    uint16_t length;
    uint16_t sequence;

    if (data == NULL) {
        return 0U;
    }

    sequence = g_bt_tx_sequence;
    length = Protocol_Bt_EncodePoseVelocity(
        data, sequence, frame, sizeof(frame));
    if ((length == 0U) ||
        (Bsp_Uart_Bluetooth_SendData(frame, length) == 0U)) {
        return 0U;
    }

    g_bt_tx_sequence = (uint16_t) (sequence + 1U);
    return 1U;
}

uint8_t Protocol_Bt_TakeLatestPoseVelocity(
    BtPoseVelocity *data,
    uint16_t *sequence)
{
    if ((data == NULL) || (sequence == NULL) ||
        (g_bt_new_pose_velocity == 0U)) {
        return 0U;
    }

    *data = g_bt_latest_pose_velocity;
    *sequence = g_bt_latest_rx_sequence;
    g_bt_new_pose_velocity = 0U;
    return 1U;
}

uint32_t Protocol_Bt_GetRxAgeMs(void)
{
    if (g_bt_rx_valid_frame_count == 0U) {
        return UINT32_MAX;
    }
    return (uint32_t) (Bsp_Time_GetMilliseconds() - g_bt_last_rx_ms);
}

uint8_t Protocol_Bt_IsRxFresh(uint32_t max_age_ms)
{
    return (g_bt_rx_valid_frame_count != 0U) &&
           (Protocol_Bt_GetRxAgeMs() <= max_age_ms);
}
