/**
 * @file route_planner.h
 * @brief 根据目标数字、数字方位和红线路口掩码生成动态路线决策。
 */
#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <stdint.h>

#include "protocol_k230.h"

#define ROUTE_PLANNER_MAX_STEPS 8U
#define ROUTE_PLANNER_NO_JUNCTION 0xFFU

typedef enum {
    ROUTE_DECISION_NONE = 0,
    ROUTE_DECISION_FRONT,
    ROUTE_DECISION_LEFT,
    ROUTE_DECISION_RIGHT,
    ROUTE_DECISION_HOLD,
    ROUTE_DECISION_FAULT
} RoutePlanner_Decision;

typedef struct {
    uint8_t from_region;
    uint8_t to_region;
    uint8_t junction_id;
    RoutePlanner_Decision decision;
} RoutePlanner_Step;

typedef struct {
    uint8_t digit_valid;
    uint8_t digit;
    uint8_t side;
    uint8_t digit_flags;
    uint8_t direction_mask;
} RoutePlanner_Observation;

typedef struct {
    uint8_t target_digit;
    uint8_t current_region;
    uint8_t latched_junction_id;
    RoutePlanner_Decision latched_decision;
    RoutePlanner_Step path[ROUTE_PLANNER_MAX_STEPS];
    uint8_t path_length;
    uint8_t return_index;
    uint8_t fault;
} RoutePlanner;

void RoutePlanner_Init(RoutePlanner *planner);
uint8_t RoutePlanner_SetTarget(RoutePlanner *planner, uint8_t target_digit);
uint8_t RoutePlanner_SetRegion(RoutePlanner *planner, uint8_t region);

/**
 * 对一个已经稳定确认的路口生成决策。同一 junction_id 会复用已锁存结果，避免重复规划。
 */
RoutePlanner_Decision RoutePlanner_Decide(
    RoutePlanner *planner,
    uint8_t junction_id,
    const RoutePlanner_Observation *observation
);

/** 转向实际完成后记录路径并切换到下一区域。 */
uint8_t RoutePlanner_CommitDecision(
    RoutePlanner *planner,
    uint8_t junction_id,
    uint8_t next_region
);

/** 路口红线恢复普通直线后解除事件锁存。 */
void RoutePlanner_ClearJunction(RoutePlanner *planner, uint8_t junction_id);

/** 从已记录出发路径末端开始按逆序读取返程动作。 */
void RoutePlanner_BeginReturn(RoutePlanner *planner);
uint8_t RoutePlanner_TakeReturnDecision(
    RoutePlanner *planner,
    RoutePlanner_Step *step
);

RoutePlanner_Decision RoutePlanner_ReverseDecision(
    RoutePlanner_Decision decision
);

#endif
