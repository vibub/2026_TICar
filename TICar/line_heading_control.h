/**
 * @file line_heading_control.h
 * @brief IMU 航向内环与 K230 红线外环的纯逻辑融合控制器。
 *
 * K230 只在新视觉帧到达时缓慢调整目标航向，IMU 在快速控制周期内持续
 * 保持该航向。模块不依赖 BSP，可在主机上独立验证延迟、丢线和角度跨界逻辑。
 */
#ifndef LINE_HEADING_CONTROL_H
#define LINE_HEADING_CONTROL_H

#include <stdint.h>

typedef struct {
    uint32_t now_ms; /* 当前统一毫秒时基。 */
    uint32_t line_age_ms; /* 最近一条 K230 红线帧的年龄，单位 ms。 */
    uint8_t imu_fresh; /* 当前 IMU 航向是否可用于闭环控制。 */
    float heading_deg; /* 当前 IMU 相对航向，左转为正，范围约 [-180°, 180°]。 */

    uint8_t line_new; /* 本控制周期是否收到新的红线帧。 */
    uint8_t line_valid; /* 当前红线是否通过协议、算法和质量过滤。 */
    int16_t line_error_px; /* 红线相对图像中心的横向误差，右偏为正，单位 px。 */
    int16_t line_angle_d10; /* 红线方向角误差，右前方为正，单位 0.1°。 */
    uint8_t junction_active; /* 当前是否处于路口事件，路口内禁止更新普通直线偏移。 */

    float base_speed_cm_s; /* 本周期未纠偏前的基础轮速，单位 cm/s。 */
} LineHeadingControl_Input;

typedef struct {
    uint8_t command_valid; /* 是否生成了可直接交给双轮速度环的目标。 */
    uint8_t visual_correction_active; /* 当前是否启用了红线航向偏移。 */
    uint8_t blind_timeout; /* 连续无有效红线是否已超过允许的 IMU 桥接时间。 */

    float route_heading_deg; /* 当前直线路段的 IMU 基准航向，单位 °。 */
    float visual_offset_deg; /* K230 外环施加的目标航向偏移，单位 °。 */
    float target_heading_deg; /* IMU 内环本周期追踪的目标航向，单位 °。 */
    float heading_error_deg; /* 归一化后的目标航向误差，单位 °。 */
    float heading_correction_cm_s; /* IMU 内环左右轮差速修正，单位 cm/s。 */
    float filtered_line_error_px; /* 低通后的红线横向误差，单位 px。 */
    float filtered_line_angle_deg; /* 低通后的红线方向角误差，单位 °。 */
    float left_target_cm_s; /* 左轮速度目标，单位 cm/s。 */
    float right_target_cm_s; /* 右轮速度目标，单位 cm/s。 */
} LineHeadingControl_Output;

typedef struct {
    uint8_t active; /* 是否已经记录当前直线路段的基准航向。 */
    uint8_t visual_filter_ready; /* 红线低通状态是否已经由有效观测初始化。 */
    uint8_t visual_correction_active; /* 带迟滞的红线辅助纠偏状态。 */

    uint32_t last_valid_line_ms; /* 最近有效红线观测时刻，单位 ms。 */
    float route_heading_deg; /* 当前直线路段基准航向，单位 °。 */
    float visual_offset_deg; /* 当前视觉目标航向偏移，单位 °。 */
    float filtered_line_error_px; /* 红线横向误差低通状态。 */
    float filtered_line_angle_deg; /* 红线角度误差低通状态，单位 °。 */
    float last_heading_error_deg; /* 上一控制周期 IMU 航向误差，单位 °。 */
    float last_heading_correction_cm_s; /* 上一周期差速修正，单位 cm/s。 */
} LineHeadingControl;

void LineHeadingControl_Init(LineHeadingControl *control);
void LineHeadingControl_Reset(LineHeadingControl *control);

void LineHeadingControl_Update(
    LineHeadingControl *control,
    const LineHeadingControl_Input *input,
    LineHeadingControl_Output *output);

#endif
