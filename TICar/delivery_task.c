/**
 * @file delivery_task.c
 * @brief 送药目标锁定、K230 视觉命令和动态路线决策状态机。
 */
#include "delivery_task.h"

#include <string.h>

#include "bsp_time.h"

#define DELIVERY_VISUAL_RESEND_MS 500U
#define DELIVERY_JUNCTION_ID_FIRST 1U
/* 至少等待一轮 YOLO 共识窗口，禁止首个未确认 D 帧直接锁存 FRONT。 */
#define DELIVERY_DECIDE_WAIT_MS 1200U

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
    RoutePlanner_Observation observation = {0};
    RoutePlanner_Decision decision;
    uint32_t now_ms;
    uint8_t new_junction;
    uint8_t target_consensus_ready;

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

    now_ms = Bsp_Time_GetMilliseconds();
    if (input->line_fresh == 0U) {
        /*
         * 红线链路中断时只停车等待，不把可恢复的通信抖动锁存成永久故障。
         * DECIDE 的等待窗口同步暂停，避免恢复后立即按“未发现目标”直行。
         */
        task->line_waiting = 1U;
        if (task->state == DELIVERY_STATE_DECIDE) {
            task->decision_start_ms = now_ms;
        }
        return;
    }
    task->line_waiting = 0U;

    new_junction = (input->junction_active != 0U) &&
                   (task->previous_junction_active == 0U);
    task->previous_junction_active = input->junction_active;
    if (new_junction != 0U) {
        task->junction_id++;
        if (task->junction_id == ROUTE_PLANNER_NO_JUNCTION) {
            task->junction_id = DELIVERY_JUNCTION_ID_FIRST;
        }
        /*
         * 小车在 DECIDE 中会停车等待 YOLO 达成共识，时间可能超过应用层
         * 300 ms 的路口清除保持时间，因此方向证据必须由任务状态机独立保存。
         */
        task->junction_direction_mask = input->direction_mask;
        task->decision_start_ms = now_ms;
        task->state = DELIVERY_STATE_DECIDE;
        (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    } else if ((task->state == DELIVERY_STATE_DECIDE) &&
               (input->direction_mask != 0U)) {
        /* 等待数字期间继续合并新方向证据，但不能丢失最初看到的左右支路。 */
        task->junction_direction_mask |= input->direction_mask;
    }

    if (task->state != DELIVERY_STATE_DECIDE) {
        return;
    }

    target_consensus_ready =
        ((input->digit_new != 0U) &&
         (input->digit_fresh != 0U) &&
         (input->digit.valid != 0U) &&
         (input->digit.digit == task->target_digit) &&
         ((input->digit.flags & K230_DIGIT_FLAG_TARGET_MATCH) != 0U) &&
         ((input->digit.flags & K230_DIGIT_FLAG_CONSENSUS) != 0U)) ? 1U : 0U;

    /*
     * 目标尚未形成共识时先保持 DECIDE，避免第一个无效或未确认 D 帧
     * 立即被规划为 FRONT。等待完整共识窗口后才允许按“未发现目标”处理。
     */
    if ((target_consensus_ready == 0U) &&
        ((uint32_t) (now_ms - task->decision_start_ms) <
         DELIVERY_DECIDE_WAIT_MS)) {
        return;
    }

    if (input->digit_new != 0U) {
        observation.digit_valid = input->digit.valid;
        observation.digit = input->digit.digit;
        observation.side = input->digit.side;
        observation.digit_flags = input->digit.flags;
    }
    observation.direction_mask = task->junction_direction_mask;
    decision = RoutePlanner_Decide(
        &task->planner,
        task->junction_id,
        &observation);

    if ((decision == ROUTE_DECISION_LEFT) ||
        (decision == ROUTE_DECISION_RIGHT) ||
        (decision == ROUTE_DECISION_FRONT)) {
        task->pending_decision = decision;
        task->state = DELIVERY_STATE_HOLD;
        /* 路口动作期间只保留红线通道，暂停 YOLO，避免推理阻塞 L 帧和转向控制。 */
        (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_OFF);
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
    task->junction_direction_mask = 0U;
    task->decision_start_ms = 0U;
    task->pending_decision = ROUTE_DECISION_NONE;
    task->state = DELIVERY_STATE_FOLLOW;
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    return 1U;
}

void DeliveryTask_FailPendingDecision(DeliveryTask *task)
{
    if (task == NULL) {
        return;
    }
    task->state = DELIVERY_STATE_FAULT;
    task->pending_decision = ROUTE_DECISION_FAULT;
}

uint8_t DeliveryTask_IsMotionAllowed(const DeliveryTask *task)
{
    if (task == NULL) {
        return 0U;
    }
    /* 路口决策或红线链路恢复期间均保持停车，禁止沿用上一次电机输出。 */
    return ((task->state == DELIVERY_STATE_FOLLOW) &&
            (task->line_waiting == 0U)) ? 1U : 0U;
}

uint8_t DeliveryTask_HasPendingTurn(const DeliveryTask *task)
{
    if (task == NULL) {
        return 0U;
    }
    return ((task->pending_decision == ROUTE_DECISION_LEFT) ||
            (task->pending_decision == ROUTE_DECISION_RIGHT)) ? 1U : 0U;
}
