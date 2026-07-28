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
    DELIVERY_MANEUVER_STATE_ALIGN,
    DELIVERY_MANEUVER_STATE_APPROACH_CENTER,
    DELIVERY_MANEUVER_STATE_TURN,
    DELIVERY_MANEUVER_STATE_REACQUIRE,
    DELIVERY_MANEUVER_STATE_CROSS,
    DELIVERY_MANEUVER_STATE_BRAKE_AFTER,
    DELIVERY_MANEUVER_STATE_FAULT
} DeliveryManeuver_State;

typedef enum {
    DELIVERY_MANEUVER_ALIGN_CREEP = 0U,
    DELIVERY_MANEUVER_ALIGN_SETTLE,
    DELIVERY_MANEUVER_ALIGN_ROTATE,
    DELIVERY_MANEUVER_ALIGN_READY
} DeliveryManeuver_AlignPhase;

typedef enum {
    DELIVERY_MANEUVER_TURN_SETTLE = 0U,
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
    uint32_t now_ms; /* 当前统一毫秒时基。 */
    float left_position_cm; /* 新车体左轮累计位置，向新车头前进为正，单位 cm。 */
    float right_position_cm; /* 新车体右轮累计位置，向新车头前进为正，单位 cm。 */

    uint8_t line_fresh; /* K230 红线链路是否仍在协议层新鲜期内。 */
    uint8_t line_new; /* 本控制周期是否收到新的 L 帧。 */
    uint8_t line_valid; /* 当前红线是否已通过链路、算法和质量过滤。 */
    int16_t line_error_px; /* 红线相对图像中心的横向误差，单位 px。 */
    int16_t line_angle_d10; /* 红线角度误差，单位 0.1°。 */
    uint8_t line_quality; /* K230 红线质量，仅用于诊断和兼容现有观测结构。 */
    uint8_t direction_mask; /* 当前可行方向位掩码。 */
    uint8_t junction_active; /* 应用层是否仍认为车辆位于路口事件中。 */

    uint8_t imu_fresh; /* IMU 航向是否满足送药动作专用新鲜度要求。 */
    float heading_deg; /* 当前相对航向，左转为正，范围约为 [-180°, 180°]。 */
} DeliveryManeuver_Input;

typedef struct {
    DeliveryManeuver_Command command; /* 本周期输出命令类型。 */
    float follow_scale; /* 巡线命令相对正常速度的缩放比例。 */
    float left_target_cm_s; /* 新车体左轮目标速度，单位 cm/s。 */
    float right_target_cm_s; /* 新车体右轮目标速度，单位 cm/s。 */
    uint8_t request_commit; /* 当前动作是否请求提交路线决策。 */
} DeliveryManeuver_Output;

typedef struct {
    DeliveryManeuver_State state; /* 路口动作主状态。 */
    DeliveryManeuver_AlignPhase align_phase; /* 路口对正内部阶段。 */
    DeliveryManeuver_TurnPhase turn_phase; /* 左右转内部阶段。 */
    RoutePlanner_Decision decision; /* 本次路口正在执行的路线决策。 */
    DeliveryManeuver_Fault fault; /* 当前锁存故障。 */

    uint32_t state_start_ms; /* 当前主状态开始时间，单位 ms。 */
    uint32_t align_phase_start_ms; /* 当前对正子阶段开始时间，单位 ms。 */
    float state_start_left_cm; /* 当前状态开始时左轮累计位置，单位 cm。 */
    float state_start_right_cm; /* 当前状态开始时右轮累计位置，单位 cm。 */
    float junction_start_left_cm; /* 首次检测路口时左轮位置，单位 cm。 */
    float junction_start_right_cm; /* 首次检测路口时右轮位置，单位 cm。 */
    float junction_progress_cm; /* 从首次检测路口起累计的有符号中心距离，单位 cm。 */

    float target_heading_deg; /* 当前 IMU 闭环目标航向，单位 °。 */
    float approach_heading_deg; /* 对正后保存的道路直行航向，单位 °。 */
    float safe_heading_deg; /* 最近一次同时具备有效红线和 IMU 时得到的安全航向。 */
    float filtered_line_error_px; /* ALIGN 使用的低通横向误差，单位 px。 */
    float filtered_line_angle_deg; /* ALIGN 使用的低通红线角度，单位 °。 */
    float align_visual_offset_deg; /* ALIGN 当前视觉目标航向偏移，单位 °。 */

    uint8_t alignment_ready; /* 是否已经锁存可供后续动作使用的道路航向。 */
    uint8_t decision_ready; /* 路线任务是否已经给出 FRONT/LEFT/RIGHT。 */
    uint8_t align_filter_ready; /* ALIGN 视觉低通是否已有初值。 */
    uint8_t safe_heading_valid; /* safe_heading_deg 是否可用。 */
    uint8_t alignment_stable_count; /* 连续满足对正阈值的新 L 帧数量。 */
    uint8_t commit_requested; /* 是否已经向路线任务发出当前动作提交请求。 */
    uint8_t heading_stable_count; /* 正式转向连续进入目标误差范围的周期数。 */
    int8_t reacquire_sweep_direction; /* 重捕获红线时当前原地扫描方向。 */
} DeliveryManeuver;

void DeliveryManeuver_Init(DeliveryManeuver *maneuver);
void DeliveryManeuver_Reset(DeliveryManeuver *maneuver);

/** 新路口出现时立即启动对正，不要求路线决策已经完成。 */
uint8_t DeliveryManeuver_StartAlignment(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input);

/** 稍后锁存路线决策；重复设置相同决策幂等，冲突决策返回 0。 */
uint8_t DeliveryManeuver_SetDecision(
    DeliveryManeuver *maneuver,
    RoutePlanner_Decision decision);

/** 返回当前活跃路口动作是否已经完成道路航向对正。 */
uint8_t DeliveryManeuver_IsAlignmentReady(
    const DeliveryManeuver *maneuver);

void DeliveryManeuver_Update(
    DeliveryManeuver *maneuver,
    const DeliveryManeuver_Input *input,
    DeliveryManeuver_Output *output);

void DeliveryManeuver_ReportCommit(
    DeliveryManeuver *maneuver,
    uint8_t success);

uint8_t DeliveryManeuver_IsActive(const DeliveryManeuver *maneuver);

#endif
