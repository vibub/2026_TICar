#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_tjc.h"

static uint8_t s_rx_bytes[32];
static uint16_t s_rx_length;
static uint16_t s_rx_index;
static uint8_t s_tx_bytes[32];
static uint16_t s_tx_length;
static uint8_t s_current_mode;
static uint8_t s_requested_mode;
static uint32_t s_mode_request_count;

void Bsp_Uart_Tjc_SendData(const uint8_t *data, uint16_t length)
{
    assert(data != NULL);
    assert(length <= sizeof(s_tx_bytes));
    memcpy(s_tx_bytes, data, length);
    s_tx_length = length;
}

uint8_t Bsp_Uart_Tjc_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (s_rx_index >= s_rx_length)) {
        return 0U;
    }
    *byte = s_rx_bytes[s_rx_index++];
    return 1U;
}

void Bsp_Uart_Tjc_FlushRx(void)
{
    s_rx_index = s_rx_length;
}

uint32_t Bsp_Uart_Tjc_GetOverflowCount(void)
{
    return 0U;
}

uint32_t Bsp_Uart_Tjc_GetErrorCount(void)
{
    return 0U;
}

uint8_t App_GetCurrentMode(void)
{
    return s_current_mode;
}

uint8_t App_RequestMode(uint8_t mode)
{
    s_requested_mode = mode;
    s_mode_request_count++;
    return 1U;
}

uint8_t App_DeliveryStartIdentification(void)
{
    return 1U;
}

uint8_t App_DeliveryStartRoute(void)
{
    return 1U;
}

uint8_t App_DeliveryReset(void)
{
    return 1U;
}

static void queue_frame(uint8_t command, uint8_t checksum)
{
    s_rx_bytes[0] = TJC_REQUEST_HEADER_0;
    s_rx_bytes[1] = TJC_REQUEST_HEADER_1;
    s_rx_bytes[2] = command;
    s_rx_bytes[3] = checksum;
    s_rx_length = TJC_REQUEST_FRAME_SIZE;
    s_rx_index = 0U;
}

static void queue_valid_frame(uint8_t command)
{
    queue_frame(
        command,
        (uint8_t) (TJC_REQUEST_HEADER_0 ^ TJC_REQUEST_HEADER_1 ^ command));
}

static void reset_fixture(void)
{
    s_rx_length = 0U;
    s_rx_index = 0U;
    s_tx_length = 0U;
    s_current_mode = 7U;
    s_requested_mode = 0U;
    s_mode_request_count = 0U;
    Protocol_Tjc_Init();
}

static void assert_speed_response(uint8_t percent)
{
    uint8_t checksum;

    assert(s_tx_length == 5U);
    assert(s_tx_bytes[0] == TJC_RESPONSE_HEADER);
    assert(s_tx_bytes[1] == TJC_RESULT_SPEED_UPDATED);
    assert(s_tx_bytes[2] == s_current_mode);
    assert(s_tx_bytes[3] == percent);
    checksum = (uint8_t) (
        s_tx_bytes[0] ^ s_tx_bytes[1] ^ s_tx_bytes[2] ^ s_tx_bytes[3]);
    assert(s_tx_bytes[4] == checksum);
}

static void test_speed_range_and_response(void)
{
    static const uint8_t speeds[] = {30U, 70U, 90U, 100U};
    uint32_t i;

    reset_fixture();
    assert(Protocol_Tjc_GetSpeedPercent() == TJC_SPEED_PERCENT_DEFAULT);

    for (i = 0U; i < (sizeof(speeds) / sizeof(speeds[0])); i++) {
        s_tx_length = 0U;
        queue_valid_frame(speeds[i]);
        Protocol_Tjc_Task();
        assert(Protocol_Tjc_GetSpeedPercent() == speeds[i]);
        assert_speed_response(speeds[i]);
    }

    assert(g_tjc_valid_command_count == 4U);
    assert(g_tjc_invalid_command_count == 0U);
    assert(g_tjc_invalid_frame_count == 0U);
}

static void test_out_of_range_and_bad_checksum_do_not_change_speed(void)
{
    reset_fixture();

    queue_valid_frame(29U);
    Protocol_Tjc_Task();
    assert(Protocol_Tjc_GetSpeedPercent() == TJC_SPEED_PERCENT_DEFAULT);
    assert(g_tjc_invalid_command_count == 1U);
    assert(g_tjc_last_result == TJC_RESULT_INVALID_COMMAND);

    queue_valid_frame(101U);
    Protocol_Tjc_Task();
    assert(Protocol_Tjc_GetSpeedPercent() == TJC_SPEED_PERCENT_DEFAULT);
    assert(g_tjc_invalid_command_count == 2U);

    queue_frame(80U, 0U);
    Protocol_Tjc_Task();
    assert(Protocol_Tjc_GetSpeedPercent() == TJC_SPEED_PERCENT_DEFAULT);
    assert(g_tjc_invalid_frame_count == 1U);
}

static void test_existing_mode_command_still_dispatches(void)
{
    reset_fixture();
    queue_valid_frame(7U);
    Protocol_Tjc_Task();

    assert(s_mode_request_count == 1U);
    assert(s_requested_mode == 7U);
    assert(g_tjc_valid_command_count == 1U);
    assert(Protocol_Tjc_GetSpeedPercent() == TJC_SPEED_PERCENT_DEFAULT);
}

int main(void)
{
    test_speed_range_and_response();
    test_out_of_range_and_bad_checksum_do_not_change_speed();
    test_existing_mode_command_still_dispatches();
    puts("protocol_tjc tests passed");
    return 0;
}
