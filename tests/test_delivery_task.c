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

static void setup_route_task(
    DeliveryTask *task,
    uint8_t target_digit,
    uint8_t region)
{
    DeliveryTask_Init(task);
    task->epoch = 3U;
    task->target_digit = target_digit;
    task->target_locked = 1U;
    task->state = DELIVERY_STATE_FOLLOW;
    task->route_region = region;
    (void) RoutePlanner_SetTarget(&task->planner, target_digit);
    (void) RoutePlanner_SetRegion(&task->planner, region);
}

static void set_target_digit(
    DeliveryTask_Input *input,
    uint8_t digit,
    uint8_t side,
    uint8_t consensus)
{
    input->digit_new = 1U;
    input->digit_fresh = 1U;
    input->digit.valid = 1U;
    input->digit.digit = digit;
    input->digit.side = side;
    input->digit.flags = K230_DIGIT_FLAG_VALID |
                         K230_DIGIT_FLAG_TARGET_MATCH;
    if (consensus != 0U) {
        input->digit.flags |= K230_DIGIT_FLAG_CONSENSUS;
    }
}

static int test_target_lock_and_route_start(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    DeliveryTask_Init(&task);
    CHECK(DeliveryTask_StartIdentification(&task, 7U) == 1U);
    CHECK(task.state == DELIVERY_STATE_IDENTIFY);
    CHECK(s_sent_mode == K230_VISUAL_MODE_PHARMACY);

    input.digit_new = 1U;
    input.digit_fresh = 1U;
    input.digit.valid = 1U;
    input.digit.digit = 6U;
    input.digit.flags = K230_DIGIT_FLAG_VALID |
                        K230_DIGIT_FLAG_CONSENSUS;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_TARGET_LOCKED);
    CHECK(task.target_locked == 1U);
    CHECK(task.target_digit == 6U);
    CHECK(s_sent_mode == K230_VISUAL_MODE_OFF);

    CHECK(DeliveryTask_StartRoute(&task) == 1U);
    CHECK(task.state == DELIVERY_STATE_FOLLOW);
    CHECK(task.route_region == K230_ROUTE_REGION_NEAR);
    CHECK(task.target_consensus_seen == 0U);
    CHECK(task.post_alignment_digit_pending == 0U);
    CHECK(s_sent_mode == K230_VISUAL_MODE_TARGET);
    CHECK(s_sent_target == 6U);
    return 1;
}

static int test_no_target_wait_starts_after_alignment(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 6U, K230_ROUTE_REGION_NEAR);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_FRONT |
                           K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_RIGHT;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);
    CHECK(task.post_alignment_digit_pending == 0U);

    /* 对正前经过再久也不能提前按无目标 FRONT。 */
    s_now_ms = 5000U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);

    input.junction_alignment_ready = 1U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.post_alignment_digit_pending == 1U);
    CHECK(task.post_alignment_start_ms == 5000U);

    s_now_ms = 6199U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);
    s_now_ms = 6200U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_FRONT);
    CHECK(s_sent_mode == K230_VISUAL_MODE_OFF);

    CHECK(DeliveryTask_CommitPendingDecision(&task) == 1U);
    CHECK(task.state == DELIVERY_STATE_FOLLOW);
    CHECK(task.route_region == K230_ROUTE_REGION_MIDDLE);
    CHECK(task.target_consensus_seen == 0U);
    CHECK(task.post_alignment_digit_pending == 0U);
    return 1;
}

static int test_pre_alignment_consensus_uses_new_post_alignment_side(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 5U, K230_ROUTE_REGION_MIDDLE);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT;
    set_target_digit(&input, 5U, K230_DIGIT_SIDE_RIGHT, 1U);
    DeliveryTask_Update(&task, &input);

    CHECK(task.state == DELIVERY_STATE_DECIDE);
    CHECK(task.target_consensus_seen == 1U);
    CHECK(task.pending_decision == ROUTE_DECISION_NONE);

    /* 对正完成时先清掉旧 D 帧，必须等待新的画面方位。 */
    input.junction_alignment_ready = 1U;
    input.digit_new = 0U;
    s_now_ms = 20U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);

    /* 身份已有共识，因此对正后的新目标帧不必重新携带 CONSENSUS。 */
    set_target_digit(&input, 5U, K230_DIGIT_SIDE_LEFT, 0U);
    s_now_ms = 40U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_LEFT);
    CHECK(DeliveryTask_HasPendingTurn(&task) == 1U);
    return 1;
}

static int test_post_alignment_frame_requires_identity_consensus(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 6U, K230_ROUTE_REGION_MIDDLE);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_RIGHT |
                           K230_LINE_DIRECTION_FRONT;
    DeliveryTask_Update(&task, &input);

    input.junction_alignment_ready = 1U;
    set_target_digit(&input, 6U, K230_DIGIT_SIDE_RIGHT, 0U);
    s_now_ms = 20U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);
    CHECK(task.target_consensus_seen == 0U);

    input.digit.flags |= K230_DIGIT_FLAG_CONSENSUS;
    s_now_ms = 40U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_RIGHT);
    return 1;
}

static int test_fixed_near_target_decides_immediately_after_alignment(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 1U, K230_ROUTE_REGION_NEAR);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);

    input.junction_alignment_ready = 1U;
    s_now_ms = 20U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_LEFT);
    return 1;
}

static int test_duplicate_junction_edge_does_not_change_id(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();
    uint8_t first_id;

    reset_harness();
    setup_route_task(&task, 6U, K230_ROUTE_REGION_FAR);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT;
    DeliveryTask_Update(&task, &input);
    first_id = task.junction_id;
    CHECK(task.state == DELIVERY_STATE_DECIDE);

    input.junction_active = 0U;
    DeliveryTask_Update(&task, &input);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_RIGHT;
    DeliveryTask_Update(&task, &input);
    CHECK(task.junction_id == first_id);
    CHECK((task.junction_direction_mask & K230_LINE_DIRECTION_LEFT) != 0U);
    CHECK((task.junction_direction_mask & K230_LINE_DIRECTION_RIGHT) != 0U);
    return 1;
}

static int test_line_timeout_keeps_consensus_and_restarts_window(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 4U, K230_ROUTE_REGION_MIDDLE);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT;
    DeliveryTask_Update(&task, &input);

    input.junction_alignment_ready = 1U;
    input.line_fresh = 0U;
    set_target_digit(&input, 4U, K230_DIGIT_SIDE_LEFT, 1U);
    s_now_ms = 100U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.target_consensus_seen == 1U);
    CHECK(task.line_waiting == 1U);
    CHECK(task.state == DELIVERY_STATE_DECIDE);
    CHECK(task.post_alignment_start_ms == 100U);

    input.line_fresh = 1U;
    input.digit_new = 0U;
    s_now_ms = 200U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.line_waiting == 0U);
    CHECK(task.state == DELIVERY_STATE_DECIDE);

    s_now_ms = 1399U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);
    s_now_ms = 1400U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_FRONT);
    return 1;
}

static int test_far_region_no_target_keeps_waiting(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 6U, K230_ROUTE_REGION_FAR);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT |
                           K230_LINE_DIRECTION_RIGHT;
    DeliveryTask_Update(&task, &input);
    input.junction_alignment_ready = 1U;
    s_now_ms = 20U;
    DeliveryTask_Update(&task, &input);

    s_now_ms = 1220U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);
    CHECK(task.pending_decision == ROUTE_DECISION_NONE);
    CHECK(task.post_alignment_start_ms == 1220U);
    return 1;
}

static int test_direction_loss_retries_same_junction(void)
{
    DeliveryTask task;
    DeliveryTask_Input input = empty_input();

    reset_harness();
    setup_route_task(&task, 6U, K230_ROUTE_REGION_FAR);
    input.junction_active = 1U;
    input.direction_mask = K230_LINE_DIRECTION_FRONT;
    set_target_digit(&input, 6U, K230_DIGIT_SIDE_LEFT, 1U);
    DeliveryTask_Update(&task, &input);
    input.junction_alignment_ready = 1U;
    input.digit_new = 0U;
    s_now_ms = 20U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_DECIDE);

    input.direction_mask = K230_LINE_DIRECTION_LEFT |
                           K230_LINE_DIRECTION_FRONT;
    set_target_digit(&input, 6U, K230_DIGIT_SIDE_LEFT, 0U);
    s_now_ms = 40U;
    DeliveryTask_Update(&task, &input);
    CHECK(task.state == DELIVERY_STATE_HOLD);
    CHECK(task.pending_decision == ROUTE_DECISION_LEFT);
    return 1;
}

static int test_reset_clears_alignment_gates(void)
{
    DeliveryTask task;

    reset_harness();
    DeliveryTask_Init(&task);
    task.target_consensus_seen = 1U;
    task.post_alignment_digit_pending = 1U;
    task.post_alignment_start_ms = 123U;
    task.state = DELIVERY_STATE_DECIDE;
    DeliveryTask_Reset(&task);

    CHECK(task.state == DELIVERY_STATE_IDLE);
    CHECK(task.target_consensus_seen == 0U);
    CHECK(task.post_alignment_digit_pending == 0U);
    CHECK(task.post_alignment_start_ms == 0U);
    CHECK(s_sent_mode == K230_VISUAL_MODE_OFF);
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
    if (!test_target_lock_and_route_start() ||
        !test_no_target_wait_starts_after_alignment() ||
        !test_pre_alignment_consensus_uses_new_post_alignment_side() ||
        !test_post_alignment_frame_requires_identity_consensus() ||
        !test_fixed_near_target_decides_immediately_after_alignment() ||
        !test_duplicate_junction_edge_does_not_change_id() ||
        !test_line_timeout_keeps_consensus_and_restarts_window() ||
        !test_far_region_no_target_keeps_waiting() ||
        !test_direction_loss_retries_same_junction() ||
        !test_reset_clears_alignment_gates() ||
        !test_visual_command_resends_until_ack()) {
        return 1;
    }

    puts("delivery_task tests passed");
    return 0;
}
