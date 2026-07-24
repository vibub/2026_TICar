#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "delivery_task.h"

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__,    \
                    #condition);                                             \
            return 0;                                                        \
        }                                                                    \
    } while (0)

static uint32_t s_now_ms;
static uint8_t s_sent_mode;
static uint8_t s_sent_target;
static uint8_t s_sent_region;
static uint8_t s_sent_epoch;
static uint32_t s_send_count;
static uint8_t s_ack_applied;

uint32_t Bsp_Time_GetMilliseconds(void)
{
    return s_now_ms;
}

uint8_t Protocol_K230_SendVisualCommand(
    uint8_t mode,
    uint8_t target_digit,
    uint8_t route_region,
    uint8_t epoch)
{
    s_sent_mode = mode;
    s_sent_target = target_digit;
    s_sent_region = route_region;
    s_sent_epoch = epoch;
    s_send_count++;
    return 1U;
}

uint8_t Protocol_K230_IsVisualCommandApplied(
    uint8_t mode,
    uint8_t target_digit,
    uint8_t route_region,
    uint8_t epoch)
{
    return (s_ack_applied != 0U) &&
           (s_sent_mode == mode) &&
           (s_sent_target == target_digit) &&
           (s_sent_region == route_region) &&
           (s_sent_epoch == epoch);
}

static void reset_harness(void)
{
    s_now_ms = 0U;
    s_sent_mode = 0U;
    s_sent_target = 0U;
    s_sent_region = 0U;
    s_sent_epoch = 0U;
    s_send_count = 0U;
    s_ack_applied = 0U;
}

static DeliveryTask_Input empty_input(void)
{
    DeliveryTask_Input input;

    memset(&input, 0, sizeof(input));
    input.line_fresh = 1U;
    input.line_valid = 1U;
    input.direction_mask = K230_LINE_DIRECTION_FRONT;
    return input;
}

static int test_target_lock_survives_conflicting_digit(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    DeliveryTask_Init(&task);
    CHECK(DeliveryTask_StartIdentification(&task, 7U) == 1U);
    CHECK(task.state == DELIVERY_STATE_IDENTIFY);
    CHECK(s_sent_mode == K230_VISUAL_MODE_PHARMACY);
    CHECK(s_sent_target == 0U);

    input.digit_new = 1U;
    input.digit_fresh = 1U;
    input.digit.valid = 1U;
    input.digit.digit = 6U;
    input.digit.flags = K230_DIGIT_FLAG_VALID |
                        K230_DIGIT_FLAG_CONSENSUS |
                        K230_DIGIT_FLAG_LOCKED;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_TARGET_LOCKED);
    CHECK(task.target_locked == 1U);
    CHECK(task.target_digit == 6U);
    CHECK(s_sent_mode == K230_VISUAL_MODE_OFF);
    CHECK(s_sent_target == 6U);
    CHECK(s_sent_region == K230_ROUTE_REGION_PHARMACY);
    CHECK(s_sent_epoch == 7U);

    input.digit.digit = 3U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.target_digit == 6U);
    return 1;
}

static int test_reset_sends_plain_off_command(void)
{
    DeliveryTask task;

    reset_harness();
    DeliveryTask_Init(&task);
    task.epoch = 8U;
    task.target_digit = 6U;
    task.target_locked = 1U;
    task.state = DELIVERY_STATE_TARGET_LOCKED;

    DeliveryTask_Reset(&task);
    CHECK(task.state == DELIVERY_STATE_IDLE);
    CHECK(s_sent_mode == K230_VISUAL_MODE_OFF);
    CHECK(s_sent_target == 0U);
    CHECK(s_sent_region == K230_ROUTE_REGION_PHARMACY);
    CHECK(s_sent_epoch == 0U);
    CHECK(s_send_count == 1U);
    CHECK(task.visual_command_pending == 1U);
    return 1;
}

static int test_route_uses_target_and_region_commands(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    DeliveryTask_Init(&task);
    DeliveryTask_StartIdentification(&task, 9U);
    task.target_digit = 6U;
    task.target_locked = 1U;
    task.state = DELIVERY_STATE_TARGET_LOCKED;
    CHECK(RoutePlanner_SetTarget(&task.planner, 6U) == 1U);
    CHECK(DeliveryTask_StartRoute(&task) == 1U);
    CHECK(task.state == DELIVERY_STATE_FOLLOW);
    CHECK(task.route_region == K230_ROUTE_REGION_NEAR);
    CHECK(s_sent_mode == K230_VISUAL_MODE_TARGET);
    CHECK(s_sent_target == 6U);
    CHECK(s_sent_region == K230_ROUTE_REGION_NEAR);

    /* 3～8 号在近端未出现时直行前往中部。 */
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_FRONT |
                           K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_RIGHT;
    input.digit_new = 1U;
    input.digit_fresh = 1U;
    input.digit.valid = 0U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_FRONT);
    CHECK(DeliveryTask_CommitPendingDecision(&task) == 1U);
    CHECK(task.route_region == K230_ROUTE_REGION_MIDDLE);
    return 1;
}

static int test_middle_target_selects_observed_side(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    DeliveryTask_Init(&task);
    task.epoch = 3U;
    task.target_digit = 5U;
    task.target_locked = 1U;
    task.state = DELIVERY_STATE_FOLLOW;
    task.route_region = K230_ROUTE_REGION_MIDDLE;
    CHECK(RoutePlanner_SetTarget(&task.planner, 5U) == 1U);
    CHECK(RoutePlanner_SetRegion(
              &task.planner, K230_ROUTE_REGION_MIDDLE) == 1U);

    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT;
    input.digit_new = 1U;
    input.digit_fresh = 1U;
    input.digit.valid = 1U;
    input.digit.digit = 5U;
    input.digit.side = K230_DIGIT_SIDE_LEFT;
    input.digit.flags = K230_DIGIT_FLAG_VALID |
                        K230_DIGIT_FLAG_TARGET_MATCH |
                        K230_DIGIT_FLAG_CONSENSUS;
    DeliveryTask_Update(&task, &input);
    CHECK(task.pending_decision == ROUTE_DECISION_LEFT);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(DeliveryTask_HasPendingTurn(&task) == 1U);
    CHECK(DeliveryTask_IsMotionAllowed(&task) == 0U);
    return 1;
}

static int test_line_timeout_enters_fault(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    DeliveryTask_Init(&task);
    task.target_digit = 4U;
    task.target_locked = 1U;
    task.state = DELIVERY_STATE_FOLLOW;
    input.line_fresh = 0U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_FAULT);
    CHECK(task.pending_decision == ROUTE_DECISION_FAULT);
    CHECK(DeliveryTask_IsMotionAllowed(&task) == 0U);
    return 1;
}

static int test_visual_command_resends_until_ack(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    DeliveryTask_Init(&task);
    DeliveryTask_StartIdentification(&task, 12U);
    CHECK(s_send_count == 1U);

    s_now_ms = 499U;
    DeliveryTask_Update(&task, &input);
    CHECK(s_send_count == 1U);
    s_now_ms = 500U;
    DeliveryTask_Update(&task, &input);
    CHECK(s_send_count == 2U);

    s_ack_applied = 1U;
    DeliveryTask_Update(&task, &input);
    s_now_ms = 1000U;
    DeliveryTask_Update(&task, &input);
    CHECK(s_send_count == 2U);
    return 1;
}

int main(void)
{
    if (!test_target_lock_survives_conflicting_digit() ||
        !test_reset_sends_plain_off_command() ||
        !test_route_uses_target_and_region_commands() ||
        !test_middle_target_selects_observed_side() ||
        !test_line_timeout_enters_fault() ||
        !test_visual_command_resends_until_ack()) {
        return 1;
    }

    puts("delivery_task tests passed");
    return 0;
}
