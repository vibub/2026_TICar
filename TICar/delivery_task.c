/**
 * @file delivery_task.c
 * @brief 送药目标锁定、K230 视觉命令和动态路线决策状态机。
 */
#include "delivery_task.h"

#include <string.h>

#include "bsp_time.h"

#define DELIVERY_VISUAL_RESEND_MS 500U
#define DELIVERY_JUNCTION_ID_FIRST 1U

static uint8_t DeliveryTask_NextRegion(
    const DeliveryTask *task,
    RoutePlanner_Decision decision)
{
    if (task->route_region == K230_ROUTE_REGION_NEAR) {
        return K230_ROUTE_REGION_MIDDLE;
    }
    if (task->route_region == K230_ROUTE_REGION_MIDDLE) {
        return (decision == ROUTE_DECISION_FRONT) ?
               K230_ROUTE_REGION_FAR : K230_ROUTE_REGION_RESERVED;
    }
    if (task->route_region == K230_ROUTE_REGION_FAR) {
        return (decision == ROUTE_DECISION_LEFT) ?
               K230_ROUTE_REGION_FAR_LEFT : K230_ROUTE_REGION_FAR_RIGHT;
    }
    return K230_ROUTE_REGION_RESERVED;
}

static uint8_t DeliveryTask_SendVisualCommand(
    DeliveryTask *task,
    uint8_t mode)
{
    if (task == NULL) {
        return 0U;
    }
    task->visual_mode = mode;
    task->last_visual_command_ms = Bsp_Time_GetMilliseconds();
    task->visual_command_pending =
        Protocol_K230_SendVisualCommand(
            mode,
            (mode == K230_VISUAL_MODE_PHARMACY) ? 0U : task->target_digit,
            task->route_region,
            task->epoch) ? 1U : 0U;
    return task->visual_command_pending;
}

static void DeliveryTask_RefreshVisualCommand(DeliveryTask *task)
{
    uint32_t now_ms;

    if ((task == NULL) || (task->visual_command_pending == 0U)) {
        return;
    }
    if (Protocol_K230_IsVisualCommandApplied(
            task->visual_mode,
            (task->visual_mode == K230_VISUAL_MODE_PHARMACY) ?
                0U : task->target_digit,
            task->route_region,
            task->epoch) != 0U) {
        task->visual_command_pending = 0U;
        return;
    }

    now_ms = Bsp_Time_GetMilliseconds();
    if ((uint32_t) (now_ms - task->last_visual_command_ms) >=
        DELIVERY_VISUAL_RESEND_MS) {
        (void) DeliveryTask_SendVisualCommand(task, task->visual_mode);
    }
}

void DeliveryTask_Init(DeliveryTask *task)
{
    if (task == NULL) {
        return;
    }
    memset(task, 0, sizeof(*task));
    task->state = DELIVERY_STATE_IDLE;
    task->route_region = K230_ROUTE_REGION_PHARMACY;
    task->pending_decision = ROUTE_DECISION_NONE;
    RoutePlanner_Init(&task->planner);
}

uint8_t DeliveryTask_StartIdentification(DeliveryTask *task, uint8_t epoch)
{
    if (task == NULL) {
        return 0U;
    }
    DeliveryTask_Init(task);
    task->epoch = epoch;
    task->state = DELIVERY_STATE_IDENTIFY;
    task->route_region = K230_ROUTE_REGION_PHARMACY;
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_PHARMACY);
    return 1U;
}

uint8_t DeliveryTask_StartRoute(DeliveryTask *task)
{
    if ((task == NULL) || (task->target_locked == 0U)) {
        return 0U;
    }
    task->state = DELIVERY_STATE_FOLLOW;
    task->route_region = K230_ROUTE_REGION_NEAR;
    (void) RoutePlanner_SetRegion(&task->planner, task->route_region);
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    return 1U;
}

void DeliveryTask_Reset(DeliveryTask *task)
{
    if (task == NULL) {
        return;
    }
    DeliveryTask_Init(task);
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_OFF);
}

uint8_t DeliveryTask_SetRouteRegion(DeliveryTask *task, uint8_t region)
{
    if ((task == NULL) ||
        (RoutePlanner_SetRegion(&task->planner, region) == 0U)) {
        return 0U;
    }
    task->route_region = region;
    if ((task->state == DELIVERY_STATE_FOLLOW) ||
        (task->state == DELIVERY_STATE_DECIDE)) {
        (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    }
    return 1U;
}

static void DeliveryTask_UpdateTargetLock(
    DeliveryTask *task,
    const DeliveryTask_Input *input)
{
    if ((task->state != DELIVERY_STATE_IDENTIFY) ||
        (input->digit_new == 0U) ||
        (input->digit_fresh == 0U) ||
        (input->digit.valid == 0U) ||
        ((input->digit.flags & K230_DIGIT_FLAG_CONSENSUS) == 0U) ||
        ((input->digit.flags & K230_DIGIT_FLAG_LOCKED) == 0U) ||
        (input->digit.digit < 1U) || (input->digit.digit > 8U)) {
        return;
    }

    task->target_digit = input->digit.digit;
    task->target_locked = 1U;
    task->state = DELIVERY_STATE_TARGET_LOCKED;
    (void) RoutePlanner_SetTarget(&task->planner, task->target_digit);
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_OFF);
}

void DeliveryTask_Update(DeliveryTask *task, const DeliveryTask_Input *input)
{
    RoutePlanner_Observation observation;
    RoutePlanner_Decision decision;
    uint8_t new_junction;

    if ((task == NULL) || (input == NULL) ||
        (task->state == DELIVERY_STATE_IDLE) ||
        (task->state == DELIVERY_STATE_FAULT)) {
        return;
    }

    DeliveryTask_RefreshVisualCommand(task);
    DeliveryTask_UpdateTargetLock(task, input);

    if (task->state == DELIVERY_STATE_TARGET_LOCKED) {
        return;
    }
    if (task->state != DELIVERY_STATE_FOLLOW &&
        task->state != DELIVERY_STATE_DECIDE) {
        return;
    }

    if (input->line_fresh == 0U) {
        task->state = DELIVERY_STATE_FAULT;
        task->pending_decision = ROUTE_DECISION_FAULT;
        return;
    }

    new_junction = (input->junction_active != 0U) &&
                   (task->previous_junction_active == 0U);
    task->previous_junction_active = input->junction_active;
    if (new_junction != 0U) {
        task->junction_id++;
        if (task->junction_id == ROUTE_PLANNER_NO_JUNCTION) {
            task->junction_id = DELIVERY_JUNCTION_ID_FIRST;
        }
        task->state = DELIVERY_STATE_DECIDE;
        (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    }

    if ((task->state != DELIVERY_STATE_DECIDE) ||
        (input->digit_new == 0U)) {
        return;
    }

    observation.digit_valid = input->digit.valid;
    observation.digit = input->digit.digit;
    observation.side = input->digit.side;
    observation.digit_flags = input->digit.flags;
    observation.direction_mask = input->direction_mask;
    decision = RoutePlanner_Decide(
        &task->planner,
        task->junction_id,
        &observation);

    if ((decision == ROUTE_DECISION_LEFT) ||
        (decision == ROUTE_DECISION_RIGHT) ||
        (decision == ROUTE_DECISION_FRONT)) {
        task->pending_decision = decision;
        task->state = DELIVERY_STATE_HOLD;
    } else if (decision == ROUTE_DECISION_FAULT) {
        task->pending_decision = decision;
        task->state = DELIVERY_STATE_FAULT;
    }
}

uint8_t DeliveryTask_CommitPendingDecision(DeliveryTask *task)
{
    uint8_t next_region;

    if ((task == NULL) || (task->state != DELIVERY_STATE_HOLD) ||
        ((task->pending_decision != ROUTE_DECISION_LEFT) &&
         (task->pending_decision != ROUTE_DECISION_RIGHT) &&
         (task->pending_decision != ROUTE_DECISION_FRONT))) {
        return 0U;
    }

    next_region = DeliveryTask_NextRegion(task, task->pending_decision);
    if (RoutePlanner_CommitDecision(
            &task->planner, task->junction_id, next_region) == 0U) {
        task->state = DELIVERY_STATE_FAULT;
        task->pending_decision = ROUTE_DECISION_FAULT;
        return 0U;
    }

    task->route_region = next_region;
    RoutePlanner_ClearJunction(&task->planner, task->junction_id);
    task->previous_junction_active = 1U;
    task->pending_decision = ROUTE_DECISION_NONE;
    task->state = DELIVERY_STATE_FOLLOW;
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    return 1U;
}

uint8_t DeliveryTask_IsMotionAllowed(const DeliveryTask *task)
{
    if (task == NULL) {
        return 0U;
    }
    return ((task->state == DELIVERY_STATE_FOLLOW) ||
            (task->state == DELIVERY_STATE_DECIDE)) ? 1U : 0U;
}

uint8_t DeliveryTask_HasPendingTurn(const DeliveryTask *task)
{
    if (task == NULL) {
        return 0U;
    }
    return ((task->pending_decision == ROUTE_DECISION_LEFT) ||
            (task->pending_decision == ROUTE_DECISION_RIGHT)) ? 1U : 0U;
}
