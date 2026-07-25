/**
 * @file delivery_maneuver.h
 * @brief 送药路口底盘动作状态机的纯逻辑接口。
 *
 * 模块不直接依赖电机、编码器或 IMU 驱动，只根据应用层传入的观测
 * 生成停车、巡线或双轮速度目标，因此可以在主机上独立测试。
 */
#ifndef DELIVERY_MANEUVER_H
#define DELIVERY_MANEUVER_H

#include <stdint.h>

#include "route_planner.h"

typedef enum {
    DELIVERY_MANEUVER_STATE_IDLE = 0U,
    DELIVERY_MANEUVER_STATE_BRAKE,
    DELIVERY_MANEUVER_STATE_APPROACH_CENTER,
    DELIVERY_MANEUVER_STATE_TURN,
    DELIVERY_MANEUVER_STATE_REACQUIRE,
    DELIVERY_MANEUVER_STATE_CROSS,
    DELIVERY_MANEUVER_STATE_BRAKE_AFTER,
    DELIVERY_MANEUVER_STATE_FAULT
} DeliveryManeuver_State;

typedef enum {
    DELIVERY_MANEUVER_TURN_SETTLE = 0U,
    DELIVERY_MANEUVER_TURN_WAIT_ZERO,
    DELIVERY_MANEUVER_TURN_ROTATE
} DeliveryManeuver_TurnPhase;

typedef enum {
    DELIVERY_MANEUVER_COMMAND_STOP = 0U,
    DELIVERY_MANEUVER_COMMAND_FOLLOW_LINE,
    DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED
} DeliveryManeuver_Command;

typedef enum {
    DELIVERY_MANEUVER_FAULT_NONE = 0U,
    DELIVERY_MANEUVER_FAULT_INVALID_START,
    DELIVERY_MANEUVER_FAULT_LINE_TIMEOUT,
    DELIVERY_MANEUVER_FAULT_IMU,
    DELIVERY_MANEUVER_FAULT_STATE_TIMEOUT,
    DELIVERY_MANEUVER_FAULT_ENCODER_LIMIT,
    DELIVERY_MANEUVER_FAULT_SPEED,
    DELIVERY_MANEUVER_FAULT_COMMIT
} DeliveryManeuver_Fault;

typedef struct {
    uint32_t now_ms;
    /* 累计位置按新车体坐标传入，向新车头前进为正，单位为 cm。 */
    float left_position_cm;
    float right_position_cm;

    uint8_t line_fresh;
    uint8_t line_new;
    uint8_t line_valid;
    uint32_t line_age_ms;
    int16_t line_error_px;
    int16_t line_angle_d10;
    uint8_t line_quality;
    uint8_t direction_mask;
    uint8_t junction_active;

    uint8_t imu_fresh;
    float heading_deg;
    uint32_t speed_faults;
} DeliveryManeuver_Input;

typedef struct {
    DeliveryManeuver_Command command;
    float follow_scale;
    float left_target_cm_s;
    float right_target_cm_s;
    uint8_t request_yaw_zero;
    uint8_t request_commit;
} DeliveryManeuver_Output;

typedef struct {
    DeliveryManeuver_State state;
    DeliveryManeuver_TurnPhase turn_phase;
    RoutePlanner_Decision decision;
    DeliveryManeuver_Fault fault;

    uint32_t state_start_ms;
    uint32_t last_now_ms;
    float state_start_left_cm;
    float state_start_right_cm;
    float target_heading_deg;
    float approach_heading_deg; /* 进入路口中心时保持的 IMU 直行航向，单位 °。 */

    uint8_t line_stable_count; /* 红线连续满足当前阶段稳定条件的帧数。 */
    uint8_t junction_clear_count; /* 穿越路口后连续确认已离开路口的帧数。 */
    uint8_t line_wait_active; /* 红线链路或有效线丢失后是否正在停车等待恢复。 */
    uint8_t line_recovery_count; /* 等待期间已连续收到的可恢复新 L 帧数量。 */
    uint32_t line_wait_start_ms; /* 本次停车等待红线恢复的开始时间，单位 ms。 */
    uint8_t commit_requested;
    uint8_t heading_stable_count;
    int8_t reacquire_sweep_direction;
} DeliveryManeuver;

void DeliveryManeuver_Init(DeliveryManeuver *maneuver);
void DeliveryManeuver_Reset(DeliveryManeuver *maneuver);

uint8_t DeliveryManeuver_Start(
    DeliveryManeuver *maneuver,
    RoutePlanner_Decision decision,
    const DeliveryManeuver_Input *input);

void DeliveryManeuver_Update(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output);

void DeliveryManeuver_ReportYawZero(
    DeliveryManeuver *maneuver,
    uint8_t success);

void DeliveryManeuver_ReportCommit(
    DeliveryManeuver *maneuver,
    uint8_t success);

uint8_t DeliveryManeuver_IsActive(const DeliveryManeuver *maneuver);

#endif
