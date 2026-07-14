#ifndef PROTOCOL_K230_H
#define PROTOCOL_K230_H

#include <stdint.h>

typedef struct {
    uint8_t detected;
    int16_t target_x;
    int16_t target_y;
    int16_t current_x;
    int16_t current_y;
} K230_DetectFrame;

void Protocol_K230_Init(void);
void Protocol_K230_Task(void);
uint8_t Protocol_K230_ParseLine(const char *line, K230_DetectFrame *frame);

#endif
