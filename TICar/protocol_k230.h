#ifndef PROTOCOL_K230_H
#define PROTOCOL_K230_H

#include <stdint.h>

typedef struct {
    uint8_t detected;
    int16_t error_x;
    int16_t error_y;
    uint8_t confidence;
} K230_DetectFrame;

void Protocol_K230_Init(void);
void Protocol_K230_Task(void);
uint8_t Protocol_K230_ParseLine(const char *line, K230_DetectFrame *frame);

const K230_DetectFrame *Protocol_K230_GetLatestFrame(void);

uint8_t Protocol_K230_HasFreshFrame(void);
void Protocol_K230_ClearFreshFrame(void);

uint32_t Protocol_K230_GetValidFrameCount(void);
uint32_t Protocol_K230_GetInvalidFrameCount(void);

#endif
