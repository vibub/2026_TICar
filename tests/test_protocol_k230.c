#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_k230.h"

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__,    \
                    #condition);                                             \
            return 0;                                                        \
        }                                                                    \
    } while (0)

static uint8_t s_rx[256];
static size_t s_rx_length;
static size_t s_rx_index;
static char s_tx[512];
static size_t s_tx_length;
static uint32_t s_now_ms;


uint32_t Bsp_Time_GetMilliseconds(void)
{
    return s_now_ms;
}

uint8_t Bsp_Uart_K230_TryReceiveByte(uint8_t *byte)
{
    if ((byte == NULL) || (s_rx_index >= s_rx_length)) {
        return 0U;
    }

    *byte = s_rx[s_rx_index++];
    return 1U;
}

uint32_t Bsp_Uart_K230_GetErrorStatus(void)
{
    return 0U;
}

void Bsp_Uart_K230_ClearErrorStatus(uint32_t status)
{
    (void) status;
}

void Bsp_Uart_K230_SendString(const char *str)
{
    size_t length = strlen(str);

    if (s_tx_length + length >= sizeof(s_tx)) {
        return;
    }

    memcpy(s_tx + s_tx_length, str, length);
    s_tx_length += length;
    s_tx[s_tx_length] = '\0';
}

static void reset_harness(void)
{
    s_rx_length = 0U;
    s_rx_index = 0U;
    s_tx_length = 0U;
    s_tx[0] = '\0';
    s_now_ms = 0U;
    Protocol_K230_Init();
}

static void feed_text(const char *text)
{
    size_t length = strlen(text);

    memcpy(s_rx, text, length);
    s_rx_length = length;
    s_rx_index = 0U;
    Protocol_K230_Task();
}

static int test_target_parser_compatibility(void)
{
    K230_TargetFrame frame;

    CHECK(Protocol_K230_ParseLine("N", &frame) == 1U);
    CHECK(frame.detected == 0U);
    CHECK(Protocol_K230_ParseLine("T,-320,180,100", &frame) == 1U);
    CHECK(frame.detected == 1U);
    CHECK(frame.error_x == -320);
    CHECK(frame.error_y == 180);
    CHECK(frame.confidence == 100U);
    CHECK(Protocol_K230_ParseLine("T,321,0,80", &frame) == 0U);
    CHECK(Protocol_K230_ParseLine("T,0,0,80,1", &frame) == 0U);
    return 1;
}

static int test_line_parser_boundaries(void)
{
    K230_LineFrame frame;

    CHECK(Protocol_K230_ParseLineFrame("L,1,-160,-900,45,7", &frame) == 1U);
    CHECK(frame.valid == 1U);
    CHECK(frame.error_x == -160);
    CHECK(frame.angle_d10 == -900);
    CHECK(frame.quality == 45U);
    CHECK(frame.direction_mask == 7U);

    CHECK(Protocol_K230_ParseLineFrame("L,0,0,0,12,0", &frame) == 1U);
    CHECK(frame.valid == 0U);
    CHECK(Protocol_K230_ParseLineFrame("L,0,1,0,12,0", &frame) == 0U);
    CHECK(Protocol_K230_ParseLineFrame("L,1,161,0,80,2", &frame) == 0U);
    CHECK(Protocol_K230_ParseLineFrame("L,1,0,901,80,2", &frame) == 0U);
    CHECK(Protocol_K230_ParseLineFrame("L,1,0,0,101,2", &frame) == 0U);
    CHECK(Protocol_K230_ParseLineFrame("L,1,0,0,80,8", &frame) == 0U);
    CHECK(Protocol_K230_ParseLineFrame("L,1,0,0,80,2,x", &frame) == 0U);
    return 1;
}

static int test_line_publish_and_mailbox(void)
{
    K230_LineFrame frame;

    reset_harness();
    s_now_ms = 25U;
    feed_text("L,1,18,35,88,6\r\n");

    CHECK(g_k230_line_frame_count == 1U);
    CHECK(g_k230_line_alive == 1U);
    CHECK(g_k230_line_control_valid == 1U);
    CHECK(g_k230_last_line_frame_ms == 25U);
    CHECK(strcmp(s_tx, "ACK,L\r\n") == 0);
    CHECK(Protocol_K230_TakeLatestLineFrame(&frame) == 1U);
    CHECK(frame.error_x == 18);
    CHECK(frame.angle_d10 == 35);
    CHECK(frame.direction_mask == 6U);
    CHECK(Protocol_K230_TakeLatestLineFrame(&frame) == 0U);
    return 1;
}

static int test_invalid_line_stays_alive_but_not_controllable(void)
{
    reset_harness();
    s_now_ms = 40U;
    feed_text("L,0,0,0,20,0\n");

    CHECK(g_k230_line_alive == 1U);
    CHECK(g_k230_line_control_valid == 0U);
    CHECK(g_k230_line_frame_count == 1U);
    CHECK(g_k230_invalid_frame_count == 0U);
    return 1;
}

static int test_independent_timeouts(void)
{
    reset_harness();
    s_now_ms = 10U;
    feed_text("L,1,0,0,90,2\n");

    s_now_ms = 100U;
    feed_text("N\n");
    CHECK(g_k230_line_alive == 1U);
    CHECK(g_k230_link_alive == 1U);

    s_now_ms = 209U;
    Protocol_K230_Task();
    CHECK(g_k230_line_alive == 1U);

    s_now_ms = 210U;
    Protocol_K230_Task();
    CHECK(g_k230_line_alive == 0U);
    CHECK(g_k230_line_control_valid == 0U);
    CHECK(g_k230_line_timeout_count == 1U);
    CHECK(g_k230_link_alive == 1U);

    s_now_ms = 400U;
    Protocol_K230_Task();
    CHECK(g_k230_link_alive == 0U);
    CHECK(g_k230_timeout_count == 1U);
    CHECK(g_k230_line_timeout_count == 1U);
    return 1;
}

static int test_overlong_line_resynchronizes(void)
{
    static const char input[] =
        "L,1,0000000000000000000000000000000000000000000\n"
        "L,1,0,0,90,2\n";

    reset_harness();
    feed_text(input);

    CHECK(g_k230_invalid_frame_count == 1U);
    CHECK(g_k230_line_frame_count == 1U);
    CHECK(g_k230_line_alive == 1U);
    CHECK(strstr(s_tx, "ERR,FRAME\r\n") != NULL);
    CHECK(strstr(s_tx, "ACK,L\r\n") != NULL);
    return 1;
}

static int test_line_age(void)
{
    reset_harness();
    CHECK(Protocol_K230_GetLineAgeMs() == UINT32_MAX);

    s_now_ms = 50U;
    feed_text("L,1,0,0,90,2\n");
    s_now_ms = 83U;
    CHECK(Protocol_K230_GetLineAgeMs() == 33U);
    return 1;
}

int main(void)
{
    if (!test_target_parser_compatibility() ||
        !test_line_parser_boundaries() ||
        !test_line_publish_and_mailbox() ||
        !test_invalid_line_stays_alive_but_not_controllable() ||
        !test_independent_timeouts() ||
        !test_overlong_line_resynchronizes() ||
        !test_line_age()) {
        return 1;
    }

    puts("protocol_k230 tests passed");
    return 0;
}
