/**
 * @file delivery_task.h
 * @brief 送药目标锁定、视觉命令和动态路线决策状态机。
 */
#ifndef DELIVERY_TASK_H
#define DELIVERY_TASK_H

#include <stdint.h>

#include "protocol_k230.h"
#include "route_planner.h"

#define DELIVERY_STATE_IDLE          0U
#define DELIVERY_STATE_IDENTIFY      1U
#define DELIVERY_STATE_TARGET_LOCKED 2U
#define DELIVERY_STATE_FOLLOW        3U
#define DELIVERY_STATE_DECIDE        4U
#define DELIVERY_STATE_HOLD          5U
#define DELIVERY_STATE_FAULT         6U

typedef struct {
    uint8_t line_fresh;
    uint8_t line_valid;
    uint8_t junction_active;
    uint8_t direction_mask;
    uint8_t digit_new;
    uint8_t digit_fresh;
    K230_DigitFrame digit;
} DeliveryTask_Input;

typedef struct {
    uint8_t state;
    uint8_t epoch;
    uint8_t target_digit;
    uint8_t target_locked;
    uint8_t route_region;
    uint8_t junction_id;
    uint8_t previous_junction_active;
    uint8_t line_waiting; /* K230 红线链路中断后停车等待恢复，不锁存永久故障。 */
    /* 进入 DECIDE 时锁存当前路口方向，避免停车等待 YOLO 期间方向位被清除。 */
    uint8_t junction_direction_mask;
    uint8_t visual_mode;
    uint8_t visual_command_pending;
    uint32_t last_visual_command_ms;
    uint32_t decision_start_ms; /* 进入 DECIDE 的时间，用于等待 YOLO 完成共识窗口。 */
    RoutePlanner_Decision pending_decision;
    RoutePlanner planner;
} DeliveryTask;

void DeliveryTask_Init(DeliveryTask *task);
uint8_t DeliveryTask_StartIdentification(DeliveryTask *task, uint8_t epoch);
uint8_t DeliveryTask_StartRoute(DeliveryTask *task);
void DeliveryTask_Reset(DeliveryTask *task);
void DeliveryTask_Update(DeliveryTask *task, const DeliveryTask_Input *input);

/** 设置当前拓扑区域并立即向 K230 更新目标数字和视觉模式。 */
uint8_t DeliveryTask_SetRouteRegion(DeliveryTask *task, uint8_t region);

/** 实际完成当前转向后提交路径；当前实现不替代底盘转向控制。 */
uint8_t DeliveryTask_CommitPendingDecision(DeliveryTask *task);

/** 路口动作执行失败时锁存送药故障，禁止继续巡线或提交路径。 */
void DeliveryTask_FailPendingDecision(DeliveryTask *task);

uint8_t DeliveryTask_IsMotionAllowed(const DeliveryTask *task);
uint8_t DeliveryTask_HasPendingTurn(const DeliveryTask *task);

#endif
