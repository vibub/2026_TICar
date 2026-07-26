/**
 * @file delivery_task.c
 * @brief 送药目标锁定、K230 视觉命令和动态路线决策状态机。
 */
#include "delivery_task.h"

#include <string.h>

#include "bsp_time.h"

#define DELIVERY_VISUAL_RESEND_MS 500U
#define DELIVERY_JUNCTION_ID_FIRST 1U
/* 对正后等待一轮目标方位采样；对正前不消耗该窗口。 */
#define DELIVERY_POST_ALIGNMENT_WAIT_MS 1200U

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

static uint8_t DeliveryTask_IsTargetDigitFrame(
    const DeliveryTask *task,
    const DeliveryTask_Input *input)
{
    return ((input->digit_new != 0U) &&
            (input->digit_fresh != 0U) &&
            (input->digit.valid != 0U) &&
            (input->digit.digit == task->target_digit) &&
            ((input->digit.flags & K230_DIGIT_FLAG_TARGET_MATCH) != 0U)) ?
           1U : 0U;
}

static uint8_t DeliveryTask_IsTargetConsensusFrame(
    const DeliveryTask *task,
    const DeliveryTask_Input *input)
{
    return ((DeliveryTask_IsTargetDigitFrame(task, input) != 0U) &&
            ((input->digit.flags & K230_DIGIT_FLAG_CONSENSUS) != 0U)) ?
           1U : 0U;
}

static void DeliveryTask_FinishDecision(
    DeliveryTask *task,
    RoutePlanner_Decision decision)
{
    if ((decision == ROUTE_DECISION_LEFT) ||
        (decision == ROUTE_DECISION_RIGHT) ||
        (decision == ROUTE_DECISION_FRONT)) {
        task->pending_decision = decision;
        task->post_alignment_digit_pending = 0U;
        task->state = DELIVERY_STATE_HOLD;
        /* 路口动作期间暂停 YOLO，避免推理阻塞 L 帧和转向控制。 */
        (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_OFF);
    } else if (decision == ROUTE_DECISION_FAULT) {
        task->pending_decision = decision;
        task->state = DELIVERY_STATE_FAULT;
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
    task->previous_junction_active = 0U;
    task->junction_direction_mask = 0U;
    task->target_consensus_seen = 0U;
    task->post_alignment_digit_pending = 0U;
    task->post_alignment_start_ms = 0U;
    task->pending_decision = ROUTE_DECISION_NONE;
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
        (input->digit.digit < 1U) || (input->digit.digit > 8U)) {
        return;
    }

    /* 药房模式下 CONSENSUS 已代表多帧锁定，不再依赖冗余的 LOCKED 兼容位。 */
    task->target_digit = input->digit.digit;
    task->target_locked = 1U;
    task->state = DELIVERY_STATE_TARGET_LOCKED;
    (void) RoutePlanner_SetTarget(&task->planner, task->target_digit);
    (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_OFF);
}

void DeliveryTask_Update(DeliveryTask *task, const DeliveryTask_Input *input)
{
    RoutePlanner_Observation observation = {0};
    RoutePlanner_Decision decision = ROUTE_DECISION_HOLD;
    uint32_t now_ms;
    uint8_t new_junction;
    uint8_t target_frame_ready;
    uint8_t fixed_near_decision;

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
    if ((task->state != DELIVERY_STATE_FOLLOW) &&
        (task->state != DELIVERY_STATE_DECIDE)) {
        return;
    }

    now_ms = Bsp_Time_GetMilliseconds();

    /* 只有 FOLLOW 可以创建新路口；DECIDE 中的重复边沿只补充当前方向证据。 */
    new_junction = 0U;
    if (input->line_fresh != 0U) {
        new_junction = ((task->state == DELIVERY_STATE_FOLLOW) &&
                        (input->junction_active != 0U) &&
                        (task->previous_junction_active == 0U)) ? 1U : 0U;
        task->previous_junction_active = input->junction_active;
    }

    if (new_junction != 0U) {
        task->junction_id++;
        if (task->junction_id == ROUTE_PLANNER_NO_JUNCTION) {
            task->junction_id = DELIVERY_JUNCTION_ID_FIRST;
        }
        task->junction_direction_mask = input->direction_mask;
        task->target_consensus_seen = 0U;
        task->post_alignment_digit_pending = 0U;
        task->post_alignment_start_ms = 0U;
        task->pending_decision = ROUTE_DECISION_NONE;
        task->state = DELIVERY_STATE_DECIDE;
        (void) DeliveryTask_SendVisualCommand(task, K230_VISUAL_MODE_TARGET);
    } else if ((task->state == DELIVERY_STATE_DECIDE) &&
               (input->line_fresh != 0U) &&
               (input->direction_mask != 0U)) {
        task->junction_direction_mask |= input->direction_mask;
    }

    if (task->state == DELIVERY_STATE_FOLLOW) {
        task->line_waiting = (input->line_fresh == 0U) ? 1U : 0U;
        return;
    }
    if (task->state != DELIVERY_STATE_DECIDE) {
        return;
    }

    /* 红线暂失也不能丢弃已经到达的目标身份共识，但此时禁止使用 side。 */
    if (DeliveryTask_IsTargetConsensusFrame(task, input) != 0U) {
        task->target_consensus_seen = 1U;
    }

    if ((input->junction_alignment_ready != 0U) &&
        (task->post_alignment_digit_pending == 0U)) {
        task->post_alignment_digit_pending = 1U;
        task->post_alignment_start_ms = now_ms;
    }

    if (input->line_fresh == 0U) {
        task->line_waiting = 1U;
        if (task->post_alignment_digit_pending != 0U) {
            task->post_alignment_start_ms = now_ms;
        }
        return;
    }
    if ((task->line_waiting != 0U) &&
        (task->post_alignment_digit_pending != 0U)) {
        /* 链路恢复后从当前时刻重新给满方位采样窗口。 */
        task->post_alignment_start_ms = now_ms;
    }
    task->line_waiting = 0U;

    if (input->junction_alignment_ready == 0U) {
        return;
    }

    observation.direction_mask = task->junction_direction_mask;
    fixed_near_decision =
        ((task->route_region == K230_ROUTE_REGION_NEAR) &&
         ((task->target_digit == 1U) ||
          (task->target_digit == 2U))) ? 1U : 0U;

    /* 近端 1/2 号方向固定，对正完成后无需额外等待数字方位帧。 */
    if (fixed_near_decision != 0U) {
        decision = RoutePlanner_Decide(
            &task->planner,
            task->junction_id,
            &observation);
        DeliveryTask_FinishDecision(task, decision);
        if (task->state != DELIVERY_STATE_DECIDE) {
            return;
        }
    }

    target_frame_ready =
        ((DeliveryTask_IsTargetDigitFrame(task, input) != 0U) &&
         ((task->target_consensus_seen != 0U) ||
          ((input->digit.flags & K230_DIGIT_FLAG_CONSENSUS) != 0U))) ?
        1U : 0U;

    if (target_frame_ready != 0U) {
        observation.digit_valid = 1U;
        observation.digit = input->digit.digit;
        observation.side = input->digit.side;
        observation.digit_flags = input->digit.flags |
                                  K230_DIGIT_FLAG_CONSENSUS;
        decision = RoutePlanner_Decide(
            &task->planner,
            task->junction_id,
            &observation);
        DeliveryTask_FinishDecision(task, decision);
        if (task->state != DELIVERY_STATE_DECIDE) {
            return;
        }
        /* 方向证据暂缺时保留该路口并重新给下一帧完整等待窗口。 */
        task->post_alignment_start_ms = now_ms;
        return;
    }

    if ((uint32_t) (now_ms - task->post_alignment_start_ms) <
        DELIVERY_POST_ALIGNMENT_WAIT_MS) {
        return;
    }

    /* 无目标超时仍由 RoutePlanner 按当前区域决定，远端返回 HOLD 时继续识别。 */
    decision = RoutePlanner_Decide(
        &task->planner,
        task->junction_id,
        &observation);
    DeliveryTask_FinishDecision(task, decision);
    if (task->state == DELIVERY_STATE_DECIDE) {
        task->post_alignment_start_ms = now_ms;
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
    task->target_consensus_seen = 0U;
    task->post_alignment_digit_pending = 0U;
    task->post_alignment_start_ms = 0U;
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
    /* 普通巡线只在 FOLLOW 中授权；DECIDE 的运动由 Maneuver ALIGN 独占控制。 */
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
