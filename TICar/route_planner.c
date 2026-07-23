/**
 * @file route_planner.c
 * @brief 动态病房路线纯状态机，不直接访问电机或 UART。
 */
#include "route_planner.h"

#include <stddef.h>
#include <string.h>


static uint8_t RoutePlanner_DirectionAvailable(
    RoutePlanner_Decision decision,
    uint8_t direction_mask)
{
    if (decision == ROUTE_DECISION_LEFT) {
        return (direction_mask & K230_LINE_DIRECTION_LEFT) != 0U;
    }
    if (decision == ROUTE_DECISION_RIGHT) {
        return (direction_mask & K230_LINE_DIRECTION_RIGHT) != 0U;
    }
    if (decision == ROUTE_DECISION_FRONT) {
        return (direction_mask & K230_LINE_DIRECTION_FRONT) != 0U;
    }
    return 0U;
}

static RoutePlanner_Decision RoutePlanner_DecisionFromSide(uint8_t side)
{
    if (side == K230_DIGIT_SIDE_LEFT) {
        return ROUTE_DECISION_LEFT;
    }
    if (side == K230_DIGIT_SIDE_RIGHT) {
        return ROUTE_DECISION_RIGHT;
    }
    return ROUTE_DECISION_HOLD;
}

void RoutePlanner_Init(RoutePlanner *planner)
{
    if (planner == NULL) {
        return;
    }
    memset(planner, 0, sizeof(*planner));
    planner->current_region = K230_ROUTE_REGION_PHARMACY;
    planner->latched_junction_id = ROUTE_PLANNER_NO_JUNCTION;
    planner->latched_decision = ROUTE_DECISION_NONE;
}

uint8_t RoutePlanner_SetTarget(RoutePlanner *planner, uint8_t target_digit)
{
    if ((planner == NULL) || (target_digit < 1U) || (target_digit > 8U)) {
        return 0U;
    }
    planner->target_digit = target_digit;
    return 1U;
}

uint8_t RoutePlanner_SetRegion(RoutePlanner *planner, uint8_t region)
{
    if ((planner == NULL) || (region > K230_ROUTE_REGION_RESERVED)) {
        return 0U;
    }
    planner->current_region = region;
    return 1U;
}

RoutePlanner_Decision RoutePlanner_Decide(
    RoutePlanner *planner,
    uint8_t junction_id,
    const RoutePlanner_Observation *observation)
{
    RoutePlanner_Decision decision;
    uint8_t target_seen;
    uint8_t consensus_ready;

    if ((planner == NULL) || (observation == NULL) ||
        (planner->target_digit < 1U) || (planner->target_digit > 8U)) {
        return ROUTE_DECISION_FAULT;
    }

    if ((planner->latched_junction_id == junction_id) &&
        (planner->latched_decision != ROUTE_DECISION_NONE)) {
        return planner->latched_decision;
    }

    target_seen = ((observation->digit_valid != 0U) &&
                   (observation->digit == planner->target_digit) &&
                   ((observation->digit_flags & K230_DIGIT_FLAG_TARGET_MATCH) != 0U)) ? 1U : 0U;
    consensus_ready = ((observation->digit_flags &
                        K230_DIGIT_FLAG_CONSENSUS) != 0U) ? 1U : 0U;

    /* 1、2 号在近端固定左右，仍必须确认该方向确实存在红线。 */
    if (planner->current_region == K230_ROUTE_REGION_NEAR) {
        if (planner->target_digit == 1U) {
            decision = ROUTE_DECISION_LEFT;
        } else if (planner->target_digit == 2U) {
            decision = ROUTE_DECISION_RIGHT;
        } else if ((target_seen != 0U) && (consensus_ready != 0U)) {
            decision = RoutePlanner_DecisionFromSide(observation->side);
        } else {
            decision = ROUTE_DECISION_FRONT;
        }
    } else if ((target_seen != 0U) && (consensus_ready != 0U)) {
        /* 3～8 号不绑定固定方向，只使用当前实际看到的目标方位。 */
        decision = RoutePlanner_DecisionFromSide(observation->side);
    } else if (planner->current_region == K230_ROUTE_REGION_MIDDLE) {
        /* 中部没有目标时继续驶向远端，不因其他数字误转。 */
        decision = ROUTE_DECISION_FRONT;
    } else {
        /* 远端及侧走廊必须找到目标后再决策，禁止盲猜。 */
        decision = ROUTE_DECISION_HOLD;
    }

    if ((decision == ROUTE_DECISION_LEFT) ||
        (decision == ROUTE_DECISION_RIGHT) ||
        (decision == ROUTE_DECISION_FRONT)) {
        if (RoutePlanner_DirectionAvailable(
                decision, observation->direction_mask) == 0U) {
            decision = ROUTE_DECISION_HOLD;
        }
    }

    planner->latched_junction_id = junction_id;
    planner->latched_decision = decision;
    return decision;
}

uint8_t RoutePlanner_CommitDecision(
    RoutePlanner *planner,
    uint8_t junction_id,
    uint8_t next_region)
{
    RoutePlanner_Step *step;

    if ((planner == NULL) ||
        (planner->latched_junction_id != junction_id) ||
        (next_region > K230_ROUTE_REGION_RESERVED) ||
        ((planner->latched_decision != ROUTE_DECISION_LEFT) &&
         (planner->latched_decision != ROUTE_DECISION_RIGHT) &&
         (planner->latched_decision != ROUTE_DECISION_FRONT))) {
        return 0U;
    }

    if (planner->path_length >= ROUTE_PLANNER_MAX_STEPS) {
        planner->fault = 1U;
        return 0U;
    }

    step = &planner->path[planner->path_length];
    step->from_region = planner->current_region;
    step->to_region = next_region;
    step->junction_id = junction_id;
    step->decision = planner->latched_decision;
    planner->path_length++;
    planner->current_region = next_region;
    return 1U;
}

void RoutePlanner_ClearJunction(RoutePlanner *planner, uint8_t junction_id)
{
    if ((planner != NULL) &&
        (planner->latched_junction_id == junction_id)) {
        planner->latched_junction_id = ROUTE_PLANNER_NO_JUNCTION;
        planner->latched_decision = ROUTE_DECISION_NONE;
    }
}

void RoutePlanner_BeginReturn(RoutePlanner *planner)
{
    if (planner == NULL) {
        return;
    }
    planner->return_index = planner->path_length;
}

RoutePlanner_Decision RoutePlanner_ReverseDecision(
    RoutePlanner_Decision decision)
{
    if (decision == ROUTE_DECISION_LEFT) {
        return ROUTE_DECISION_RIGHT;
    }
    if (decision == ROUTE_DECISION_RIGHT) {
        return ROUTE_DECISION_LEFT;
    }
    if (decision == ROUTE_DECISION_FRONT) {
        return ROUTE_DECISION_FRONT;
    }
    return decision;
}

uint8_t RoutePlanner_TakeReturnDecision(
    RoutePlanner *planner,
    RoutePlanner_Step *step)
{
    const RoutePlanner_Step *outbound;

    if ((planner == NULL) || (step == NULL) ||
        (planner->return_index == 0U)) {
        return 0U;
    }

    planner->return_index--;
    outbound = &planner->path[planner->return_index];
    step->from_region = outbound->to_region;
    step->to_region = outbound->from_region;
    step->junction_id = outbound->junction_id;
    step->decision = RoutePlanner_ReverseDecision(outbound->decision);
    return 1U;
}
