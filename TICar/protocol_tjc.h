#ifndef PROTOCOL_TJC_H
#define PROTOCOL_TJC_H

#include <stdint.h>

#define TJC_COMMAND_STOP 0x00U
#define TJC_COMMAND_FIRST_MODE 0x01U
#define TJC_COMMAND_LAST_MODE 0x0DU
#define TJC_COMMAND_QUERY 0x0EU
#define TJC_RESPONSE_HEADER 0xA5U

typedef enum {
    TJC_RESULT_STATE = 0x10U,
    TJC_RESULT_ACCEPTED_BRAKING = 0x11U,
    TJC_RESULT_SWITCH_OK = 0x12U,
    TJC_RESULT_STOPPED = 0x13U,
    TJC_RESULT_ALREADY_ACTIVE = 0x14U,
    TJC_RESULT_INVALID_COMMAND = 0xE0U,
    TJC_RESULT_ENTER_FAILED = 0xE1U
} Tjc_Result;

extern volatile uint32_t g_tjc_rx_byte_count;
extern volatile uint32_t g_tjc_valid_command_count;
extern volatile uint32_t g_tjc_invalid_command_count;
extern volatile uint32_t g_tjc_response_count;
extern volatile uint32_t g_tjc_rx_overflow_count;
extern volatile uint32_t g_tjc_rx_error_count;
extern volatile uint8_t g_tjc_last_command;
extern volatile uint8_t g_tjc_last_result;

void Protocol_Tjc_Init(void);
void Protocol_Tjc_Task(void);
void Protocol_Tjc_SendResult(Tjc_Result result, uint8_t current_mode, uint8_t request);

#endif
