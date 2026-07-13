#include "protocol_k230.h"

#include <stdio.h>
#include "bsp_uart.h"

void Protocol_K230_Init(void)
{
}

void Protocol_K230_Task(void)
{
    Bsp_Uart_K230_SendString("M0_WAIT\r\n");
}

uint8_t Protocol_K230_ParseLine(const char *line, K230_DetectFrame *frame)
{
    int detected;
    int target_x;
    int target_y;
    int current_x;
    int current_y;

    if (sscanf(line, "DET,%d,%d,%d,%d,%d",
               &detected, &target_x, &target_y, &current_x, &current_y) != 5) {
        return 0;
    }

    frame->detected = (uint8_t) detected;
    frame->target_x = (int16_t) target_x;
    frame->target_y = (int16_t) target_y;
    frame->current_x = (int16_t) current_x;
    frame->current_y = (int16_t) current_y;
    return 1;
}
