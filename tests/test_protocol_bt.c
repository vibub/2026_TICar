#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_bt.h"

static uint8_t s_rx_bytes[128];
static uint16_t s_rx_length;
static uint16_t s_rx_index;
static uint32_t s_now_ms;

/* protocol_bt.c 的主机测试桩。 */
uint8_t Bsp_Uart_Bluetooth_SendData(const uint8_t *data, uint16_t length)
{
    (void) data;
    (void) length;
    return 1U;
}

uint8_t Bsp_Uart_Bluetooth_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (s_rx_index >= s_rx_length)) {
        return 0U;
    }
    *byte = s_rx_bytes[s_rx_index++];
    return 1U;
}

void Bsp_Uart_Bluetooth_FlushRx(void)
{
    s_rx_index = s_rx_length;
}

uint32_t Bsp_Time_GetMilliseconds(void)
{
    return s_now_ms;
}

uint32_t Bsp_Uart_Bluetooth_GetOverflowCount(void)
{
    return 0U;
}

uint32_t Bsp_Uart_Bluetooth_GetErrorCount(void)
{
    return 0U;
}

static void queue_bytes(const uint8_t *data, uint16_t length)
{
    assert(length <= sizeof(s_rx_bytes));
    memcpy(s_rx_bytes, data, length);
    s_rx_length = length;
    s_rx_index = 0U;
}

static void test_encode_decode_round_trip(void)
{
    const BtPoseVelocity input = {
        .x_mm = 123456,
        .y_mm = -654321,
        .yaw_cdeg = -17999,
        .linear_mm_s = 1250,
        .angular_cdeg_s = -3200
    };
    BtPoseVelocity output;
    uint8_t frame[BT_PROTOCOL_FRAME_SIZE];
    uint16_t sequence = 0U;

    assert(Protocol_Bt_EncodePoseVelocity(
               &input, 0xBEEFU, frame, sizeof(frame)) ==
           BT_PROTOCOL_FRAME_SIZE);
    assert(frame[0] == BT_PROTOCOL_SOF_0);
    assert(frame[1] == BT_PROTOCOL_SOF_1);
    assert(frame[4] == BT_PROTOCOL_POSE_PAYLOAD_SIZE);
    assert(Protocol_Bt_DecodePoseVelocity(
               frame, sizeof(frame), &output, &sequence) == 1U);
    assert(sequence == 0xBEEFU);
    assert(output.x_mm == input.x_mm);
    assert(output.y_mm == input.y_mm);
    assert(output.yaw_cdeg == input.yaw_cdeg);
    assert(output.linear_mm_s == input.linear_mm_s);
    assert(output.angular_cdeg_s == input.angular_cdeg_s);
}

static void test_crc_standard_vector(void)
{
    static const uint8_t input[] = "123456789";
    assert(Protocol_Bt_Crc16(input, 9U) == 0x29B1U);
}

static void test_crc_rejects_corruption(void)
{
    const BtPoseVelocity input = {1, 2, 3, 4, 5};
    BtPoseVelocity output;
    uint8_t frame[BT_PROTOCOL_FRAME_SIZE];
    uint16_t sequence;

    assert(Protocol_Bt_EncodePoseVelocity(
               &input, 7U, frame, sizeof(frame)) ==
           BT_PROTOCOL_FRAME_SIZE);
    frame[10] ^= 0x40U;
    assert(Protocol_Bt_DecodePoseVelocity(
               frame, sizeof(frame), &output, &sequence) == 0U);
}

static void test_stream_parser_resynchronizes_after_noise(void)
{
    const BtPoseVelocity input = {-1000, 2000, 4500, -300, 125};
    BtPoseVelocity output;
    uint8_t stream[BT_PROTOCOL_FRAME_SIZE + 4U] = {0x00U, 0xA5U, 0x11U, 0x33U};
    uint16_t sequence;

    Protocol_Bt_Init();
    assert(Protocol_Bt_EncodePoseVelocity(
               &input, 99U, &stream[4], BT_PROTOCOL_FRAME_SIZE) ==
           BT_PROTOCOL_FRAME_SIZE);
    queue_bytes(stream, sizeof(stream));
    s_now_ms = 1234U;
    Protocol_Bt_Task();

    assert(g_bt_rx_valid_frame_count == 1U);
    assert(g_bt_rx_invalid_frame_count == 0U);
    assert(Protocol_Bt_TakeLatestPoseVelocity(&output, &sequence) == 1U);
    assert(sequence == 99U);
    assert(output.x_mm == input.x_mm);
    assert(output.y_mm == input.y_mm);
    assert(output.yaw_cdeg == input.yaw_cdeg);
    assert(output.linear_mm_s == input.linear_mm_s);
    assert(output.angular_cdeg_s == input.angular_cdeg_s);
    assert(Protocol_Bt_GetRxAgeMs() == 0U);
    s_now_ms = 1300U;
    assert(Protocol_Bt_IsRxFresh(100U) == 1U);
    assert(Protocol_Bt_IsRxFresh(50U) == 0U);
}

int main(void)
{
    test_encode_decode_round_trip();
    test_crc_standard_vector();
    test_crc_rejects_corruption();
    test_stream_parser_resynchronizes_after_noise();
    puts("protocol_bt tests passed");
    return 0;
}
