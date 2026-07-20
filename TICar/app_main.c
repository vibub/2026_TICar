/**
 * @file app_main.c
 * @brief TICar 应用层运行时模式管理器和各模式的非阻塞任务实现。
 *
 * TJC 命令通过 App_RequestMode() 提交请求；模式管理器依次执行主动制动、旧模式退出、
 * 公共资源复位和新模式进入。切换期间不运行模式任务，上电固定保持 APP_MODE_STOPPED。
 * CCD、编码器和速度闭环按需初始化；本文件中的 volatile 全局量主要供 CCS Watch 观察。
 */
#include "app_main.h"

#include "app_config.h"
#include "bsp_ccd.h"
#include "bsp_gpio.h"
#include "bsp_motor.h"
#include "bsp_ptz.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "pid.h"
#include "protocol_k230.h"
#include "protocol_tjc.h"

/* 电机映射测试：用明显不同的左右轮比例观察底盘偏航方向。 */
#define APP_MOTOR_TEST_SLOW_SPEED 0.18f
#define APP_MOTOR_TEST_FAST_SPEED 0.45f

/* CCD Watch 在 RAM 中保留最近 32 帧完整像素，供 CCS Memory Browser 导出。 */
#define APP_CCD_LOG_FRAME_COUNT 32U

/*
 * 通用巡线参数。
 * 误差单位为 CCD 像素索引；base speed 是巡线层比例参考，随后映射为 cm/s 轮速目标。
 * 误差越大，基础速度越低、最小转向量越大；修正变化率限制用于抑制 CCD 抖动造成的底盘突变。
 */
#define APP_LINE_BASE_SPEED 0.49f
#define APP_LINE_BASE_TARGET_CM_S 30.0f
#define APP_LINE_KP 0.0020f
#define APP_LINE_KD 0.0005f
#define APP_LINE_STEER_LIMIT 0.130f
#define APP_LINE_MEDIUM_MIN_STEER 0.020f
#define APP_LINE_LARGE_MIN_STEER 0.090f
#define APP_LINE_CORRECTION_SLEW_LIMIT 0.035f
#define APP_LINE_MEDIUM_SPEED_SCALE 0.65f
#define APP_LINE_LARGE_SPEED_SCALE 0.39f
#define APP_LINE_DEADBAND_ERROR 5
#define APP_LINE_MEDIUM_ERROR 6
#define APP_LINE_LARGE_ERROR 12
#define APP_LINE_STEER_SIGN 1.0f
/* CCD 直行模式只做有效性门控，不使用中心误差进行转向。 */
#define APP_CCD_STRAIGHT_SPEED 0.16f

/* 普通/圆形巡线允许短时沿最后一次有效方向低速搜索，超过窗口后主动停车。 */
#define APP_LINE_LOST_RECOVER_MAX 8U
#define APP_LINE_LOST_SPEED_SCALE 0.55f
#define APP_LINE_LOST_TURN 0.040f

/*
 * 方形赛道允许更长的丢线搜索，并用独立直角状态机处理黑线中心消失。
 * 所有 *_LOOPS 均以 20 ms 快速任务周期计数。
 */
#define APP_SQUARE_LOST_RECOVER_MAX 14U
#define APP_SQUARE_LOST_SPEED_SCALE 0.45f
#define APP_SQUARE_LOST_TURN 0.060f
#define APP_SQUARE_CORNER_STATE_TRACK 0U
#define APP_SQUARE_CORNER_STATE_BRAKE 1U
#define APP_SQUARE_CORNER_STATE_PIVOT 2U
#define APP_SQUARE_CORNER_STATE_FAULT 3U
#define APP_SQUARE_CORNER_ARM_LOOPS 5U
#define APP_SQUARE_CORNER_ENTER_ERROR 32
#define APP_SQUARE_CORNER_REACQUIRE_ERROR 8
#define APP_SQUARE_CORNER_MISSING_LOOPS 6U
#define APP_SQUARE_CORNER_REACQUIRE_LOOPS 3U
#define APP_SQUARE_CORNER_BRAKE_LOOPS 5U
#define APP_SQUARE_CORNER_MAX_PIVOT_LOOPS 90U
#define APP_SQUARE_CORNER_PIVOT_SPEED 0.16f
#define APP_SQUARE_CORNER_DEFAULT_DIRECTION (-1)
/* 速度测试以 20 ms 为一步，循环执行静止、20、30、20 cm/s 四个阶段。 */
#define APP_SPEED_TEST_LOW_CM_S 20.0f
#define APP_SPEED_TEST_HIGH_CM_S 30.0f
#define APP_SPEED_TEST_WARMUP_LOOPS 250U
#define APP_SPEED_TEST_HOLD_LOOPS 250U

/*
 * K230 云台增量 PID 参数。
 * 图像误差来自 640×360 坐标系；两轴分别计算 P/I/D，合成后限制单帧 compare 增量。
 * Ki 当前保持关闭，但保留积分状态和限幅，便于后续实车调参。
 */
#define APP_K230_FOLLOW_PAN_KP_NUM 8
#define APP_K230_FOLLOW_PAN_KP_DEN 100
#define APP_K230_FOLLOW_PAN_KI_NUM 0
#define APP_K230_FOLLOW_PAN_KI_DEN 100
#define APP_K230_FOLLOW_PAN_KD_NUM 4
#define APP_K230_FOLLOW_PAN_KD_DEN 100
#define APP_K230_FOLLOW_PAN_INTEGRAL_LIMIT 1000
#define APP_K230_FOLLOW_TILT_KP_NUM 8
#define APP_K230_FOLLOW_TILT_KP_DEN 100
#define APP_K230_FOLLOW_TILT_KI_NUM 0
#define APP_K230_FOLLOW_TILT_KI_DEN 100
#define APP_K230_FOLLOW_TILT_KD_NUM 4
#define APP_K230_FOLLOW_TILT_KD_DEN 100
#define APP_K230_FOLLOW_TILT_INTEGRAL_LIMIT 1000
#define APP_K230_FOLLOW_X_DEADBAND 10
#define APP_K230_FOLLOW_Y_DEADBAND 8
#define APP_K230_FOLLOW_PAN_MAX_STEP 20
#define APP_K230_FOLLOW_TILT_MAX_STEP 15
#define APP_K230_FOLLOW_PAN_DIRECTION (-1)
#define APP_K230_FOLLOW_TILT_DIRECTION 1
#define APP_K230_FOLLOW_STARTUP_HOLD_MS 2000U
#define APP_K230_FOLLOW_STATE_WAIT_LINK 0U
#define APP_K230_FOLLOW_STATE_TRACKING 1U
#define APP_K230_FOLLOW_STATE_HOLD 2U
#define APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED 3U

/* 运动模式切换前保持 100 ms 主动制动，降低惯性带来的模式交叉影响。 */
#define APP_MODE_SWITCH_BRAKE_MS 100U
/* 快速周期用于 CCD、编码器和速度闭环；打印周期控制串口负载；慢速周期用于心跳和 UART 测试。 */
#define APP_LOOP_FAST_MS 20U
#define APP_LOOP_PRINT_MS 500U
#define APP_LOOP_SLOW_MS 1000U

/* 模式切换状态：稳定运行、制动等待、执行退出/进入提交。 */
#define APP_SWITCH_IDLE 0U
#define APP_SWITCH_BRAKING 1U
#define APP_SWITCH_ENTERING 2U

/*
 * 三种巡线模式共享的参数集合。
 * medium/large_error 以像素为单位；速度和转向字段为巡线层比例参考；lost_recover_max 为 20 ms 循环次数。
 */
typedef struct {
    float base_speed;             /* 中心附近的基础前进参考。 */
    float kp;                     /* CCD 中心误差的比例增益。 */
    float kd;                     /* 相邻帧误差差分的增益。 */
    float steer_sign;             /* 将 CCD 误差方向映射到底盘左右轮差速方向。 */
    float steer_limit;            /* 最大转向修正绝对值。 */
    int16_t medium_error;         /* 开始降速的中等误差阈值。 */
    int16_t large_error;          /* 进入强恢复的较大误差阈值。 */
    float medium_speed_scale;     /* 中等误差时的基础速度缩放。 */
    float large_speed_scale;      /* 大误差时的基础速度缩放。 */
    float medium_min_steer;       /* 中等误差必须保留的最小转向量。 */
    float large_min_steer;        /* 大误差必须保留的最小转向量。 */
    uint16_t lost_recover_max;    /* 丢线后仍尝试搜索的最大循环数。 */
    float lost_speed_scale;       /* 丢线搜索时的基础速度缩放。 */
    float lost_turn;              /* 丢线搜索时沿最后有效方向施加的转向量。 */
} App_FollowProfile;

/*
 * 一帧可导出的 CCD 调试快照。
 * valid 为 0 时 target/black edge 可能使用 -1 表示无可靠检测；raw 保存完整 128 像素 ADC 值。
 */
typedef struct {
    uint16_t frame_count;         /* 紧凑帧序号，取全局帧计数低 16 位。 */
    uint16_t valid;               /* 当前帧是否具有可用目标。 */
    int16_t target;               /* 黑线中心像素索引。 */
    int16_t error;                /* target - CCD 中心索引。 */
    int16_t black_left;
    int16_t black_right;
    uint16_t black_width;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t threshold;
    uint16_t contrast;
    uint16_t raw_min_index;
    uint16_t raw_max_index;
    uint16_t raw[BSP_CCD_PIXEL_COUNT];
} App_CcdLogFrame;

/* 普通巡线的实车参数基线。 */
static const App_FollowProfile g_line_follow_profile = {
    APP_LINE_BASE_SPEED, APP_LINE_KP, APP_LINE_KD, APP_LINE_STEER_SIGN,
    APP_LINE_STEER_LIMIT, APP_LINE_MEDIUM_ERROR, APP_LINE_LARGE_ERROR,
    APP_LINE_MEDIUM_SPEED_SCALE, APP_LINE_LARGE_SPEED_SCALE,
    APP_LINE_MEDIUM_MIN_STEER, APP_LINE_LARGE_MIN_STEER,
    APP_LINE_LOST_RECOVER_MAX, APP_LINE_LOST_SPEED_SCALE, APP_LINE_LOST_TURN
};

/* 圆形赛道当前复用普通巡线数值，但保留独立 profile 便于后续单独调参。 */
static const App_FollowProfile g_circle_follow_profile = {
    APP_LINE_BASE_SPEED, APP_LINE_KP, APP_LINE_KD, APP_LINE_STEER_SIGN,
    APP_LINE_STEER_LIMIT, APP_LINE_MEDIUM_ERROR, APP_LINE_LARGE_ERROR,
    APP_LINE_MEDIUM_SPEED_SCALE, APP_LINE_LARGE_SPEED_SCALE,
    APP_LINE_MEDIUM_MIN_STEER, APP_LINE_LARGE_MIN_STEER,
    APP_LINE_LOST_RECOVER_MAX, APP_LINE_LOST_SPEED_SCALE, APP_LINE_LOST_TURN
};

/* 方形赛道增加更长的丢线搜索窗口，并由独立直角状态机优先接管。 */
static const App_FollowProfile g_square_follow_profile = {
    APP_LINE_BASE_SPEED, APP_LINE_KP, APP_LINE_KD, APP_LINE_STEER_SIGN,
    APP_LINE_STEER_LIMIT, APP_LINE_MEDIUM_ERROR, APP_LINE_LARGE_ERROR,
    APP_LINE_MEDIUM_SPEED_SCALE, APP_LINE_LARGE_SPEED_SCALE,
    APP_LINE_MEDIUM_MIN_STEER, APP_LINE_LARGE_MIN_STEER,
    APP_SQUARE_LOST_RECOVER_MAX, APP_SQUARE_LOST_SPEED_SCALE,
    APP_SQUARE_LOST_TURN
};

/*
 * 模式管理状态，均可在 CCS Watch 中观察。
 * current 是已生效模式，requested 是待进入模式，debug_mode 镜像最终提交结果；switch_state 表示切换阶段。
 */
volatile uint32_t g_app_debug_mode = APP_MODE_STOPPED;
volatile uint8_t g_app_current_mode = APP_MODE_STOPPED;
volatile uint8_t g_app_requested_mode = APP_MODE_STOPPED;
volatile uint8_t g_app_switch_state = APP_SWITCH_IDLE;
volatile uint32_t g_app_mode_switch_count;
volatile uint32_t g_app_mode_reject_count;

/* 切换内部状态：制动起始时间、应答中的原始请求，以及按需初始化标志。 */
static uint32_t g_app_switch_deadline_ms;
static uint8_t g_app_switch_request;
static uint8_t g_ccd_initialized;
static uint8_t g_encoder_initialized;
static uint8_t g_speed_pid_initialized;
static uint32_t g_mode_last_task_ms;
static uint8_t g_motor_test_phase;
static uint32_t g_motor_test_phase_start_ms;
static uint32_t g_uart_test_count;

/*
 * K230 云台跟随诊断量。
 * control_error 是死区过滤后的图像像素误差；delta/compare 是本帧增量和当前定时器比较值。
 * 状态依次为等待链路、跟踪、无目标保持、链路超时后关闭 PWM。
 */
static uint32_t g_k230_follow_seen_timeout_count;
static uint32_t g_k230_follow_start_ms;
volatile int16_t g_k230_control_error_x;
volatile int16_t g_k230_control_error_y;
volatile int16_t g_k230_pan_delta;
volatile int16_t g_k230_tilt_delta;
volatile int32_t g_k230_pan_p_term;
volatile int32_t g_k230_pan_i_term;
volatile int32_t g_k230_pan_d_term;
volatile int32_t g_k230_tilt_p_term;
volatile int32_t g_k230_tilt_i_term;
volatile int32_t g_k230_tilt_d_term;
volatile int32_t g_k230_pan_integral;
volatile int32_t g_k230_tilt_integral;
volatile int16_t g_k230_pan_previous_error;
volatile int16_t g_k230_tilt_previous_error;
static uint8_t g_k230_pan_previous_error_valid;
static uint8_t g_k230_tilt_previous_error_valid;
volatile uint16_t g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
volatile uint16_t g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
volatile uint8_t g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
volatile uint32_t g_k230_follow_update_count;

/*
 * 编码器和速度闭环 Watch 镜像。
 * count 是累计软件计数，speed 是最近 20 ms 窗口的有符号增量；target/measured 单位为 cm/s。
 * command 是最终 PWM 比例，fault bit0/bit1 分别表示左/右编码器在受驱动时持续无反馈。
 */
volatile int32_t g_encoder_left_count;
volatile int32_t g_encoder_right_count;
volatile int16_t g_encoder_left_speed;
volatile int16_t g_encoder_right_speed;
volatile float g_speed_left_target;
volatile float g_speed_right_target;
volatile float g_speed_left_measured;
volatile float g_speed_right_measured;
volatile float g_speed_left_cmd;
volatile float g_speed_right_cmd;
volatile float g_speed_left_p;
volatile float g_speed_right_p;
volatile float g_speed_left_i;
volatile float g_speed_right_i;
volatile uint32_t g_speed_faults;
/* 速度阶跃阶段：0 静止预热、1 目标 20、2 目标 30、3 返回 20 cm/s；loop 为 20 ms 步序号。 */
volatile uint32_t g_speed_test_phase;
volatile uint32_t g_speed_test_loop;

/*
 * CCD Watch 当前帧镜像和 32 帧环形日志。
 * raw/filtered 诊断值由 CCD Watch 模式更新；log_write_index 指向下一写入槽，log_filled 表示有效槽数量。
 */
volatile uint32_t g_debug_ccd_frame_count;
volatile uint8_t g_debug_ccd_valid;
volatile int16_t g_debug_ccd_target = -1;
volatile int16_t g_debug_ccd_error;
volatile int16_t g_debug_ccd_black_left = -1;
volatile int16_t g_debug_ccd_black_right = -1;
volatile uint16_t g_debug_ccd_black_width;
volatile uint16_t g_debug_ccd_raw_min;
volatile uint16_t g_debug_ccd_raw_max;
volatile uint16_t g_debug_ccd_raw_min_index;
volatile uint16_t g_debug_ccd_raw_max_index;
volatile uint16_t g_debug_ccd_contrast;
volatile uint16_t g_debug_ccd_threshold;
volatile int16_t g_debug_ccd_dx_max;
volatile int16_t g_debug_ccd_dx_min;
volatile uint16_t g_debug_ccd_dx_max_index;
volatile uint16_t g_debug_ccd_dx_min_index;
volatile uint16_t g_debug_ccd_raw[BSP_CCD_PIXEL_COUNT];
volatile uint16_t g_debug_ccd_log_write_index;
volatile uint16_t g_debug_ccd_log_filled;
volatile App_CcdLogFrame g_debug_ccd_log[APP_CCD_LOG_FRAME_COUNT];

/*
 * 巡线实时状态。
 * 正 error 表示黑线位于 CCD 中心右侧；left/right_cmd 是巡线层参考，并非最终硬件 PWM。
 * recover_direction 为 -1 左搜、+1 右搜；last_error/last_correction 保存算法内部历史量。
 */
volatile uint8_t g_line_valid;
volatile int16_t g_line_target = -1;
volatile int16_t g_line_error;
volatile int16_t g_line_error_delta;
volatile int16_t g_line_dx_max;
volatile int16_t g_line_dx_min;
volatile uint16_t g_line_dx_max_index;
volatile uint16_t g_line_dx_min_index;
volatile uint16_t g_line_raw_min;
volatile uint16_t g_line_raw_max;
volatile uint16_t g_line_raw_min_index;
volatile uint16_t g_line_raw_max_index;
volatile uint16_t g_line_contrast;
volatile uint16_t g_line_lost_count;
volatile float g_line_correction;
volatile float g_line_left_cmd;
volatile float g_line_right_cmd;
volatile int8_t g_line_recover_direction;
static int16_t g_line_last_error;
static float g_line_last_correction;

/*
 * 方形直角状态：正常巡线、主动制动、原地找线和超时停车锁存。
 * count 是当前状态内的 20 ms 循环数；direction 为 -1 左转、+1 右转。
 * seen_center_line 表示角点检测已完成连续帧武装，其余计数用于丢线和重捕获防抖。
 */
volatile uint8_t g_square_corner_state;
volatile uint32_t g_square_corner_count;
volatile int8_t g_square_corner_direction;
volatile uint32_t g_square_corner_entry_count;
volatile uint32_t g_square_corner_timeout_count;
volatile uint8_t g_square_seen_center_line;
volatile uint8_t g_square_center_stable_count;
volatile uint8_t g_square_center_missing_count;
volatile uint8_t g_square_reacquire_count;

/*
 * 判断周期是否到期。使用 uint32_t 无符号差值可自然跨越毫秒计数回绕；到期后同步更新基准时间。
 */
static uint8_t App_TimeElapsed(uint32_t *last_ms, uint32_t period_ms)
{
    uint32_t now_ms = Bsp_Time_GetMilliseconds();

    if ((uint32_t) (now_ms - *last_ms) < period_ms) {
        return 0U;
    }

    *last_ms = now_ms;
    return 1U;
}

/* 返回该模式是否可能驱动底盘；结果决定切换前是否需要 100 ms 主动制动。 */
static uint8_t App_ModeUsesMotor(uint8_t mode)
{
    return ((mode == APP_MODE_MOTOR_PWM) ||
            (mode == APP_MODE_LINE_FOLLOW) ||
            (mode == APP_MODE_CCD_STRAIGHT) ||
            (mode == APP_MODE_SPEED_TEST) ||
            (mode == APP_MODE_CIRCLE_FOLLOW) ||
            (mode == APP_MODE_SQUARE_FOLLOW)) ? 1U : 0U;
}

/*
 * CCD、编码器和速度闭环采用首次使用时初始化，避免无关调试模式提前启动外设。
 * 每次重新进入模式只复位算法状态，不重复配置底层硬件。
 */
static void App_EnsureCcd(void)
{
    if (g_ccd_initialized == 0U) {
        Bsp_Ccd_Init();
        g_ccd_initialized = 1U;
    }
}

static void App_EnsureEncoder(void)
{
    if (g_encoder_initialized == 0U) {
        Bsp_Motor_EncoderInit();
        g_encoder_initialized = 1U;
    }
}

static void App_EnsureSpeedPid(void)
{
    App_EnsureEncoder();
    if (g_speed_pid_initialized == 0U) {
        Bsp_Motor_SpeedPidInit();
        g_speed_pid_initialized = 1U;
    }
}

/* 不依赖 printf，将无符号整数转换为十进制字符并直接发送到 K230 UART。 */
static void App_SendUint32(uint32_t value)
{
    char buf[10];
    uint8_t idx = 0U;

    do {
        buf[idx++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (idx != 0U) {
        Bsp_Uart_K230_SendByte((uint8_t) buf[--idx]);
    }
}

/* 将转向修正限制在对称范围内，避免算法指令超过实车安全调试区间。 */
static float App_LimitFloat(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

/* 限制相邻控制周期的修正变化量，降低 CCD 跳变对底盘的瞬时冲击。 */
static float App_LimitFloatDelta(float value, float last_value, float max_delta)
{
    float delta = value - last_value;

    if (delta > max_delta) {
        return last_value + max_delta;
    }
    if (delta < -max_delta) {
        return last_value - max_delta;
    }
    return value;
}

/* 将底层速度闭环状态统一镜像到全局量，便于 CCS Watch 同时观察左右轮。 */
static void App_UpdateSpeedWatch(void)
{
    g_encoder_left_count = Bsp_Motor_GetLeftEncoderCount();
    g_encoder_right_count = Bsp_Motor_GetRightEncoderCount();
    g_encoder_left_speed = Bsp_Motor_GetLeftEncoderSpeed();
    g_encoder_right_speed = Bsp_Motor_GetRightEncoderSpeed();
    g_speed_left_target = Bsp_Motor_GetLeftSpeedTarget();
    g_speed_right_target = Bsp_Motor_GetRightSpeedTarget();
    g_speed_left_measured = Bsp_Motor_GetLeftMeasuredSpeed();
    g_speed_right_measured = Bsp_Motor_GetRightMeasuredSpeed();
    g_speed_left_cmd = Bsp_Motor_GetLeftSpeedCommand();
    g_speed_right_cmd = Bsp_Motor_GetRightSpeedCommand();
    g_speed_left_p = Bsp_Motor_GetLeftSpeedPTerm();
    g_speed_right_p = Bsp_Motor_GetRightSpeedPTerm();
    g_speed_left_i = Bsp_Motor_GetLeftSpeedITerm();
    g_speed_right_i = Bsp_Motor_GetRightSpeedITerm();
    g_speed_faults = Bsp_Motor_GetSpeedFaults();
}

/*
 * 将巡线层左右比例参考按已验证的直线基准映射为 cm/s，再运行两个独立轮速控制器。
 * 因此 Watch 中的巡线命令、轮速目标和最终 PWM 输出属于三个不同层次。
 */
static void App_LineApplySpeedPid(float left_reference, float right_reference)
{
    const float reference_to_cm_s = APP_LINE_BASE_TARGET_CM_S / APP_LINE_BASE_SPEED;

    Bsp_Motor_SetSpeedTargets(left_reference * reference_to_cm_s,
                              right_reference * reference_to_cm_s);
    Bsp_Motor_SpeedPidUpdate();
    App_UpdateSpeedWatch();
}

/* 清除巡线输出、丢线计数和 PD 历史，保证每次进入巡线模式都从确定状态开始。 */
static void App_ResetLineState(void)
{
    g_line_valid = 0U;
    g_line_target = -1;
    g_line_error = 0;
    g_line_error_delta = 0;
    g_line_lost_count = 0U;
    g_line_correction = 0.0f;
    g_line_left_cmd = 0.0f;
    g_line_right_cmd = 0.0f;
    g_line_recover_direction = 0;
    g_line_last_error = 0;
    g_line_last_correction = 0.0f;
}

/* 清除方形赛道角点阶段、防抖计数和超时锁存，保证模式重入从未武装状态开始。 */
static void App_ResetSquareState(void)
{
    g_square_corner_state = APP_SQUARE_CORNER_STATE_TRACK;
    g_square_corner_count = 0U;
    g_square_corner_direction = 0;
    g_square_corner_entry_count = 0U;
    g_square_corner_timeout_count = 0U;
    g_square_seen_center_line = 0U;
    g_square_center_stable_count = 0U;
    g_square_center_missing_count = 0U;
    g_square_reacquire_count = 0U;
}

/*
 * 镜像当前 CCD 检测结果，并把完整 128 像素帧写入固定 32 槽环形日志。
 * 日志覆盖最旧帧，不进行动态内存分配，适合通过 CCS Memory Browser 导出。
 */
static void App_UpdateCcdWatchData(void)
{
    uint16_t i;
    uint16_t log_index;
    const uint16_t *raw = Bsp_Ccd_GetRawFrame();

    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_debug_ccd_raw[i] = raw[i];
    }

    g_debug_ccd_target = Bsp_Ccd_GetTargetIndex();
    g_debug_ccd_error = Bsp_Ccd_GetLineError();
    g_debug_ccd_valid = Bsp_Ccd_IsLineValid();
    g_debug_ccd_black_left = Bsp_Ccd_GetBlackLeft();
    g_debug_ccd_black_right = Bsp_Ccd_GetBlackRight();
    g_debug_ccd_black_width = Bsp_Ccd_GetBlackWidth();
    g_debug_ccd_raw_min = Bsp_Ccd_GetRawMin();
    g_debug_ccd_raw_max = Bsp_Ccd_GetRawMax();
    g_debug_ccd_raw_min_index = Bsp_Ccd_GetRawMinIndex();
    g_debug_ccd_raw_max_index = Bsp_Ccd_GetRawMaxIndex();
    g_debug_ccd_contrast = Bsp_Ccd_GetContrast();
    g_debug_ccd_threshold = Bsp_Ccd_GetThreshold();
    g_debug_ccd_dx_max = Bsp_Ccd_GetDxMax();
    g_debug_ccd_dx_min = Bsp_Ccd_GetDxMin();
    g_debug_ccd_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_debug_ccd_dx_min_index = Bsp_Ccd_GetDxMinIndex();
    g_debug_ccd_frame_count++;

    log_index = g_debug_ccd_log_write_index;
    g_debug_ccd_log[log_index].frame_count = (uint16_t) g_debug_ccd_frame_count;
    g_debug_ccd_log[log_index].valid = g_debug_ccd_valid;
    g_debug_ccd_log[log_index].target = g_debug_ccd_target;
    g_debug_ccd_log[log_index].error = g_debug_ccd_error;
    g_debug_ccd_log[log_index].black_left = g_debug_ccd_black_left;
    g_debug_ccd_log[log_index].black_right = g_debug_ccd_black_right;
    g_debug_ccd_log[log_index].black_width = g_debug_ccd_black_width;
    g_debug_ccd_log[log_index].raw_min = g_debug_ccd_raw_min;
    g_debug_ccd_log[log_index].raw_max = g_debug_ccd_raw_max;
    g_debug_ccd_log[log_index].threshold = g_debug_ccd_threshold;
    g_debug_ccd_log[log_index].contrast = g_debug_ccd_contrast;
    g_debug_ccd_log[log_index].raw_min_index = g_debug_ccd_raw_min_index;
    g_debug_ccd_log[log_index].raw_max_index = g_debug_ccd_raw_max_index;
    for (i = 0U; i < BSP_CCD_PIXEL_COUNT; i++) {
        g_debug_ccd_log[log_index].raw[i] = raw[i];
    }

    g_debug_ccd_log_write_index = (uint16_t) ((log_index + 1U) % APP_CCD_LOG_FRAME_COUNT);
    if (g_debug_ccd_log_filled < APP_CCD_LOG_FRAME_COUNT) {
        g_debug_ccd_log_filled++;
    }
}

static int32_t App_K230Follow_LimitInt32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t App_K230Follow_FilterError(int16_t error, int16_t deadband)
{
    return ((error >= -deadband) && (error <= deadband)) ? 0 : error;
}

/* 定点比例运算采用四舍五入，并保留输入符号，避免云台 PID 引入浮点计算。 */
static int32_t App_K230Follow_ScaleRound(
    int32_t value,
    int16_t numerator,
    int16_t denominator)
{
    int32_t absolute_value;
    int32_t scaled_value;

    if ((value == 0) || (numerator == 0)) {
        return 0;
    }

    absolute_value = (value < 0) ? -value : value;
    scaled_value = ((absolute_value * numerator) + (denominator / 2)) /
                   denominator;
    return (value < 0) ? -scaled_value : scaled_value;
}

/* 清除单轴 PID 历史，防止死区、超时或模式重入后继承旧积分和微分状态。 */
static void App_K230Follow_ResetPidAxis(
    volatile int32_t *integral,
    volatile int16_t *previous_error,
    uint8_t *previous_error_valid,
    volatile int32_t *p_term,
    volatile int32_t *i_term,
    volatile int32_t *d_term)
{
    *integral = 0;
    *previous_error = 0;
    *previous_error_valid = 0U;
    *p_term = 0;
    *i_term = 0;
    *d_term = 0;
}

static void App_K230Follow_ResetPidState(void)
{
    App_K230Follow_ResetPidAxis(
        &g_k230_pan_integral,
        &g_k230_pan_previous_error,
        &g_k230_pan_previous_error_valid,
        &g_k230_pan_p_term,
        &g_k230_pan_i_term,
        &g_k230_pan_d_term);
    App_K230Follow_ResetPidAxis(
        &g_k230_tilt_integral,
        &g_k230_tilt_previous_error,
        &g_k230_tilt_previous_error_valid,
        &g_k230_tilt_p_term,
        &g_k230_tilt_i_term,
        &g_k230_tilt_d_term);
}

/* 计算单轴增量 PID 输出，并在累加到舵机位置前限制本帧最大 compare 变化。 */
static int16_t App_K230Follow_CalculatePidDelta(
    int16_t error,
    int8_t direction,
    int16_t kp_num,
    int16_t kp_den,
    int16_t ki_num,
    int16_t ki_den,
    int16_t kd_num,
    int16_t kd_den,
    int32_t integral_limit,
    int16_t max_step,
    volatile int32_t *integral,
    volatile int16_t *previous_error,
    uint8_t *previous_error_valid,
    volatile int32_t *p_term,
    volatile int32_t *i_term,
    volatile int32_t *d_term)
{
    int32_t derivative_error = 0;
    int32_t delta;

    if (error == 0) {
        App_K230Follow_ResetPidAxis(
            integral, previous_error, previous_error_valid,
            p_term, i_term, d_term);
        return 0;
    }

    *integral = App_K230Follow_LimitInt32(
        *integral + error, -integral_limit, integral_limit);
    if (*previous_error_valid != 0U) {
        derivative_error = (int32_t) error - *previous_error;
    }
    *previous_error = error;
    *previous_error_valid = 1U;

    *p_term = App_K230Follow_ScaleRound(error, kp_num, kp_den);
    *i_term = App_K230Follow_ScaleRound(*integral, ki_num, ki_den);
    *d_term = App_K230Follow_ScaleRound(derivative_error, kd_num, kd_den);

    delta = (*p_term + *i_term + *d_term) * direction;
    delta = App_K230Follow_LimitInt32(delta, -max_step, max_step);
    return (int16_t) delta;
}

/*
 * 处理一帧 K230 目标：无目标或置信度不足时保持当前位置；有效目标依次经过死区、比例、
 * 单帧步长和机械范围限制后更新云台。HOLD 不关闭 PWM，与链路超时禁用状态不同。
 */
static void App_K230Follow_ProcessFrame(const K230_TargetFrame *frame)
{
    int16_t requested_pan_delta;
    int16_t requested_tilt_delta;
    int32_t next_pan;
    int32_t next_tilt;
    uint16_t previous_pan;
    uint16_t previous_tilt;

    if ((frame->detected == 0U) || (frame->confidence < K230_MIN_CONFIDENCE)) {
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        App_K230Follow_ResetPidState();
        g_k230_follow_state = APP_K230_FOLLOW_STATE_HOLD;
        return;
    }

    g_k230_control_error_x = App_K230Follow_FilterError(
        frame->error_x, APP_K230_FOLLOW_X_DEADBAND);
    g_k230_control_error_y = App_K230Follow_FilterError(
        frame->error_y, APP_K230_FOLLOW_Y_DEADBAND);
    requested_pan_delta = App_K230Follow_CalculatePidDelta(
        g_k230_control_error_x,
        APP_K230_FOLLOW_PAN_DIRECTION,
        APP_K230_FOLLOW_PAN_KP_NUM,
        APP_K230_FOLLOW_PAN_KP_DEN,
        APP_K230_FOLLOW_PAN_KI_NUM,
        APP_K230_FOLLOW_PAN_KI_DEN,
        APP_K230_FOLLOW_PAN_KD_NUM,
        APP_K230_FOLLOW_PAN_KD_DEN,
        APP_K230_FOLLOW_PAN_INTEGRAL_LIMIT,
        APP_K230_FOLLOW_PAN_MAX_STEP,
        &g_k230_pan_integral,
        &g_k230_pan_previous_error,
        &g_k230_pan_previous_error_valid,
        &g_k230_pan_p_term,
        &g_k230_pan_i_term,
        &g_k230_pan_d_term);
    requested_tilt_delta = App_K230Follow_CalculatePidDelta(
        g_k230_control_error_y,
        APP_K230_FOLLOW_TILT_DIRECTION,
        APP_K230_FOLLOW_TILT_KP_NUM,
        APP_K230_FOLLOW_TILT_KP_DEN,
        APP_K230_FOLLOW_TILT_KI_NUM,
        APP_K230_FOLLOW_TILT_KI_DEN,
        APP_K230_FOLLOW_TILT_KD_NUM,
        APP_K230_FOLLOW_TILT_KD_DEN,
        APP_K230_FOLLOW_TILT_INTEGRAL_LIMIT,
        APP_K230_FOLLOW_TILT_MAX_STEP,
        &g_k230_tilt_integral,
        &g_k230_tilt_previous_error,
        &g_k230_tilt_previous_error_valid,
        &g_k230_tilt_p_term,
        &g_k230_tilt_i_term,
        &g_k230_tilt_d_term);

    previous_pan = g_k230_pan_compare;
    previous_tilt = g_k230_tilt_compare;
    next_pan = App_K230Follow_LimitInt32(
        (int32_t) previous_pan + requested_pan_delta,
        BSP_PTZ_PAN_MIN, BSP_PTZ_PAN_MAX);
    next_tilt = App_K230Follow_LimitInt32(
        (int32_t) previous_tilt + requested_tilt_delta,
        BSP_PTZ_TILT_MIN, BSP_PTZ_TILT_MAX);

    g_k230_pan_compare = (uint16_t) next_pan;
    g_k230_tilt_compare = (uint16_t) next_tilt;
    g_k230_pan_delta = (int16_t) (next_pan - previous_pan);
    g_k230_tilt_delta = (int16_t) (next_tilt - previous_tilt);
    Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare);
    g_k230_follow_state = APP_K230_FOLLOW_STATE_TRACKING;
    g_k230_follow_update_count++;
}

/*
 * 高频轮询 K230 协议并消费最新帧。每次进入模式先保持中心两秒；链路超时后关闭 PWM，
 * 收到新的合法帧时恢复引脚复用和超时前位置，无需用户重新选择模式。
 */
static void App_K230Follow_Task(void)
{
    K230_TargetFrame frame;

    Protocol_K230_Task();

    if ((uint32_t) (Bsp_Time_GetMilliseconds() - g_k230_follow_start_ms) <
        APP_K230_FOLLOW_STARTUP_HOLD_MS) {
        (void) Protocol_K230_TakeLatestFrame(&frame);
        g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
        g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
        Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare);
        g_k230_follow_seen_timeout_count = g_k230_timeout_count;
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        App_K230Follow_ResetPidState();
        g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
        return;
    }

    if (g_k230_timeout_count != g_k230_follow_seen_timeout_count) {
        g_k230_follow_seen_timeout_count = g_k230_timeout_count;
    }
    if ((g_k230_link_alive == 0U) &&
        (g_k230_timeout_count != 0U) &&
        (g_k230_follow_state != APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED)) {
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        App_K230Follow_ResetPidState();
        Bsp_Ptz_Disable();
        g_k230_follow_state = APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED;
        return;
    }

    if (Protocol_K230_TakeLatestFrame(&frame) == 0U) {
        return;
    }

    if (g_k230_follow_state == APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED) {
        if (g_k230_link_alive == 0U) {
            return;
        }

        Bsp_Ptz_Init();
        Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare);
        Bsp_Ptz_Start();
    }

    App_K230Follow_ProcessFrame(&frame);
}

/*
 * 方形赛道直角状态机：
 * 正常巡线需要连续可靠中心帧完成武装，随后连续丢线才进入主动制动和固定方向原地转向。
 * 重捕获同样采用连续帧确认；超过最大转向周期后锁存停车，避免条件未清除时反复原地打转。
 * black_width==0 的弱边沿桥接帧不能用于武装或重捕获，但仍可由通用巡线维持短时连续性。
 *
 * @return 1 表示本周期由角点状态机接管电机；0 表示继续执行通用巡线。
 */
static uint8_t App_SquareCornerTask(void)
{
    uint8_t line_valid = Bsp_Ccd_IsLineValid();
    int16_t line_error = Bsp_Ccd_GetLineError();
    int16_t abs_error = (line_error < 0) ? (int16_t) (-line_error) : line_error;
    uint16_t black_width = Bsp_Ccd_GetBlackWidth();
    uint8_t center_detected = ((line_valid != 0U) && (black_width != 0U) &&
                               (abs_error < APP_SQUARE_CORNER_ENTER_ERROR)) ? 1U : 0U;
    uint8_t center_reacquired = ((line_valid != 0U) && (black_width != 0U) &&
                                 (abs_error <= APP_SQUARE_CORNER_REACQUIRE_ERROR)) ? 1U : 0U;

    if (g_square_corner_state == APP_SQUARE_CORNER_STATE_TRACK) {
        if (g_square_seen_center_line == 0U) {
            g_square_center_missing_count = 0U;
            if (center_detected != 0U) {
                if (g_square_center_stable_count < APP_SQUARE_CORNER_ARM_LOOPS) {
                    g_square_center_stable_count++;
                }
                if (g_square_center_stable_count >= APP_SQUARE_CORNER_ARM_LOOPS) {
                    g_square_seen_center_line = 1U;
                }
            } else {
                g_square_center_stable_count = 0U;
            }
            return 0U;
        }

        if (center_detected != 0U) {
            g_square_center_missing_count = 0U;
        } else if (g_square_center_missing_count < APP_SQUARE_CORNER_MISSING_LOOPS) {
            g_square_center_missing_count++;
        }

        if (g_square_center_missing_count < APP_SQUARE_CORNER_MISSING_LOOPS) {
            return 0U;
        }

        g_square_corner_state = APP_SQUARE_CORNER_STATE_BRAKE;
        g_square_corner_count = 0U;
        g_square_corner_direction = APP_SQUARE_CORNER_DEFAULT_DIRECTION;
        g_square_corner_entry_count++;
        g_square_seen_center_line = 0U;
        g_square_center_stable_count = 0U;
        g_square_center_missing_count = 0U;
        g_square_reacquire_count = 0U;
        Bsp_Motor_SpeedPidStop();
        g_line_last_correction = 0.0f;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
        g_line_correction = 0.0f;
        g_line_valid = line_valid;
        g_line_target = line_valid ? Bsp_Ccd_GetTargetIndex() : -1;
        g_line_error = line_valid ? line_error : g_line_last_error;
        g_line_error_delta = 0;
        return 1U;
    }

    if (g_square_corner_state == APP_SQUARE_CORNER_STATE_FAULT) {
        Bsp_Motor_SpeedPidStop();
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
        g_line_correction = 0.0f;
        g_line_valid = line_valid;
        g_line_target = line_valid ? Bsp_Ccd_GetTargetIndex() : -1;
        g_line_error = line_valid ? line_error : g_line_last_error;
        g_line_error_delta = 0;
        return 1U;
    }

    if (center_reacquired != 0U) {
        if (g_square_reacquire_count < APP_SQUARE_CORNER_REACQUIRE_LOOPS) {
            g_square_reacquire_count++;
        }
    } else {
        g_square_reacquire_count = 0U;
    }

    if (g_square_reacquire_count >= APP_SQUARE_CORNER_REACQUIRE_LOOPS) {
        g_square_corner_state = APP_SQUARE_CORNER_STATE_TRACK;
        g_square_corner_count = 0U;
        g_square_seen_center_line = 0U;
        g_square_center_stable_count = 0U;
        g_square_center_missing_count = 0U;
        g_square_reacquire_count = 0U;
        g_line_lost_count = 0U;
        g_line_last_error = line_error;
        g_line_last_correction = 0.0f;
        return 0U;
    }

    if (g_square_corner_state == APP_SQUARE_CORNER_STATE_BRAKE) {
        Bsp_Motor_SpeedPidStop();
        g_square_corner_count++;
        if (g_square_corner_count >= APP_SQUARE_CORNER_BRAKE_LOOPS) {
            g_square_corner_state = APP_SQUARE_CORNER_STATE_PIVOT;
            g_square_corner_count = 0U;
        }
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
    } else if (g_square_corner_state == APP_SQUARE_CORNER_STATE_PIVOT) {
        float pivot = APP_SQUARE_CORNER_PIVOT_SPEED *
                      (float) g_square_corner_direction;

        App_LineApplySpeedPid(pivot, -pivot);
        g_line_left_cmd = pivot;
        g_line_right_cmd = -pivot;
        g_square_corner_count++;
        if (g_square_corner_count >= APP_SQUARE_CORNER_MAX_PIVOT_LOOPS) {
            Bsp_Motor_SpeedPidStop();
            g_square_corner_state = APP_SQUARE_CORNER_STATE_FAULT;
            g_square_corner_count = 0U;
            g_square_corner_timeout_count++;
            g_square_seen_center_line = 0U;
            g_square_center_stable_count = 0U;
            g_square_center_missing_count = 0U;
            g_square_reacquire_count = 0U;
            g_line_last_correction = 0.0f;
            g_line_left_cmd = 0.0f;
            g_line_right_cmd = 0.0f;
        }
    } else {
        Bsp_Motor_SpeedPidStop();
        g_square_corner_state = APP_SQUARE_CORNER_STATE_FAULT;
        g_square_corner_count = 0U;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
    }

    g_line_valid = line_valid;
    g_line_target = line_valid ? Bsp_Ccd_GetTargetIndex() : -1;
    g_line_error = line_valid ? line_error : g_line_last_error;
    g_line_error_delta = 0;
    g_line_correction = 0.0f;
    return 1U;
}

/*
 * 通用 20 ms CCD 巡线任务：
 * 1. 采样 CCD 并更新诊断量；方形模式先允许直角状态机接管。
 * 2. 有效线时按像素误差计算 PD 修正，并根据误差分级降速、保证最小转向、限制幅值和变化率。
 * 3. 短时丢线沿最后有效方向低速搜索，超过 profile 规定窗口后主动停车。
 */
static void App_FollowTask(const App_FollowProfile *profile, uint8_t square_mode)
{
    int16_t line_error;
    int16_t error_delta;
    int16_t abs_error;
    float base_speed;
    float steer_limit;
    float min_steer;
    float correction;
    float left_cmd;
    float right_cmd;

    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    Bsp_Ccd_ReadFrame();
    Bsp_Ccd_Process();
    g_line_raw_min = Bsp_Ccd_GetRawMin();
    g_line_raw_max = Bsp_Ccd_GetRawMax();
    g_line_raw_min_index = Bsp_Ccd_GetRawMinIndex();
    g_line_raw_max_index = Bsp_Ccd_GetRawMaxIndex();
    g_line_contrast = Bsp_Ccd_GetContrast();
    g_line_dx_max = Bsp_Ccd_GetDxMax();
    g_line_dx_min = Bsp_Ccd_GetDxMin();
    g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();

    if ((square_mode != 0U) && (App_SquareCornerTask() != 0U)) {
        Bsp_Gpio_ToggleHeartbeat();
        return;
    }

    if (Bsp_Ccd_IsLineValid() != 0U) {
        line_error = Bsp_Ccd_GetLineError();
        error_delta = (int16_t) (line_error - g_line_last_error);
        abs_error = (line_error < 0) ? (int16_t) (-line_error) : line_error;
        base_speed = profile->base_speed;
        steer_limit = profile->steer_limit;
        min_steer = 0.0f;
        correction = profile->steer_sign *
                     ((profile->kp * (float) line_error) +
                      (profile->kd * (float) error_delta));

        if (abs_error <= APP_LINE_DEADBAND_ERROR) {
            correction = 0.0f;
            error_delta = 0;
        } else if (abs_error >= profile->large_error) {
            base_speed = profile->base_speed * profile->large_speed_scale;
            min_steer = profile->large_min_steer;
        } else if (abs_error >= profile->medium_error) {
            base_speed = profile->base_speed * profile->medium_speed_scale;
            steer_limit = profile->steer_limit * 0.75f;
            min_steer = profile->medium_min_steer;
        } else {
            steer_limit = profile->steer_limit * 0.45f;
        }

        if (line_error > 0) {
            g_line_recover_direction = 1;
        } else if (line_error < 0) {
            g_line_recover_direction = -1;
        }

        if ((min_steer > 0.0f) && (correction < min_steer) &&
            (correction > -min_steer)) {
            correction = (line_error > 0) ?
                         (profile->steer_sign * min_steer) :
                         (-profile->steer_sign * min_steer);
        }

        correction = App_LimitFloat(correction, steer_limit);
        correction = App_LimitFloatDelta(
            correction, g_line_last_correction, APP_LINE_CORRECTION_SLEW_LIMIT);
        left_cmd = base_speed + correction;
        right_cmd = base_speed - correction;
        App_LineApplySpeedPid(left_cmd, right_cmd);

        g_line_valid = 1U;
        g_line_lost_count = 0U;
        g_line_target = Bsp_Ccd_GetTargetIndex();
        g_line_error = line_error;
        g_line_error_delta = error_delta;
        g_line_correction = correction;
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        g_line_last_error = line_error;
        g_line_last_correction = correction;
    } else {
        correction = 0.0f;
        left_cmd = 0.0f;
        right_cmd = 0.0f;
        if (square_mode != 0U) {
            g_line_recover_direction = APP_SQUARE_CORNER_DEFAULT_DIRECTION;
        }

        if ((g_line_lost_count < profile->lost_recover_max) &&
            (g_line_recover_direction != 0)) {
            base_speed = profile->base_speed * profile->lost_speed_scale;
            correction = profile->steer_sign * profile->lost_turn *
                         (float) g_line_recover_direction;
            correction = App_LimitFloatDelta(
                correction, g_line_last_correction, APP_LINE_CORRECTION_SLEW_LIMIT);
            left_cmd = base_speed + correction;
            right_cmd = base_speed - correction;
            App_LineApplySpeedPid(left_cmd, right_cmd);
        } else {
            Bsp_Motor_Stop();
        }

        g_line_valid = 0U;
        g_line_lost_count++;
        g_line_target = -1;
        g_line_error = g_line_last_error;
        g_line_error_delta = 0;
        g_line_correction = correction;
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        g_line_last_correction = correction;
    }

    Bsp_Gpio_ToggleHeartbeat();
}

/* 基础联调模式：CCD 有效时以固定比例等速直行，无有效线立即主动停车。 */
static void App_CcdStraightTask(void)
{
    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    Bsp_Ccd_ReadFrame();
    Bsp_Ccd_Process();
    if (Bsp_Ccd_IsLineValid() != 0U) {
        Bsp_Motor_Set(APP_CCD_STRAIGHT_SPEED, APP_CCD_STRAIGHT_SPEED);
        g_line_valid = 1U;
        g_line_target = Bsp_Ccd_GetTargetIndex();
        g_line_error = Bsp_Ccd_GetLineError();
        g_line_left_cmd = APP_CCD_STRAIGHT_SPEED;
        g_line_right_cmd = APP_CCD_STRAIGHT_SPEED;
    } else {
        Bsp_Motor_Stop();
        g_line_valid = 0U;
        g_line_target = -1;
        g_line_error = 0;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
    }
    g_line_dx_max = Bsp_Ccd_GetDxMax();
    g_line_dx_min = Bsp_Ccd_GetDxMin();
    g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();
    g_line_correction = 0.0f;
    Bsp_Gpio_ToggleHeartbeat();
}

/*
 * 非阻塞电机映射测试：左慢右快 2 s、制动 1 s、左快右慢 2 s、制动 2 s 后循环。
 * 任务每次立即返回，因此串口屏 STOP 可以在任意阶段被处理。
 */
static void App_MotorTestTask(void)
{
    uint32_t now_ms = Bsp_Time_GetMilliseconds();
    uint32_t elapsed_ms = (uint32_t) (now_ms - g_motor_test_phase_start_ms);

    switch (g_motor_test_phase) {
        case 0U:
            if (elapsed_ms == 0U) {
                Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED);
            }
            if (elapsed_ms >= 2000U) {
                Bsp_Motor_Stop();
                g_motor_test_phase = 1U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
        case 1U:
            if (elapsed_ms >= 1000U) {
                Bsp_Motor_Set(APP_MOTOR_TEST_FAST_SPEED, APP_MOTOR_TEST_SLOW_SPEED);
                g_motor_test_phase = 2U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
        case 2U:
            if (elapsed_ms >= 2000U) {
                Bsp_Motor_Stop();
                g_motor_test_phase = 3U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
        default:
            if (elapsed_ms >= 2000U) {
                Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED);
                g_motor_test_phase = 0U;
                g_motor_test_phase_start_ms = now_ms;
            }
            break;
    }
}

/* 每 20 ms 推进一次 0→20→30→20 cm/s 阶跃，循环刷新速度闭环 Watch 数据。 */
static void App_SpeedTestTask(void)
{
    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    if (g_speed_test_loop < APP_SPEED_TEST_WARMUP_LOOPS) {
        g_speed_test_phase = 0U;
        Bsp_Motor_SetSpeedTargets(0.0f, 0.0f);
    } else if (g_speed_test_loop <
               (APP_SPEED_TEST_WARMUP_LOOPS + APP_SPEED_TEST_HOLD_LOOPS)) {
        g_speed_test_phase = 1U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_LOW_CM_S, APP_SPEED_TEST_LOW_CM_S);
    } else if (g_speed_test_loop <
               (APP_SPEED_TEST_WARMUP_LOOPS + (2U * APP_SPEED_TEST_HOLD_LOOPS))) {
        g_speed_test_phase = 2U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_HIGH_CM_S, APP_SPEED_TEST_HIGH_CM_S);
    } else {
        g_speed_test_phase = 3U;
        Bsp_Motor_SetSpeedTargets(APP_SPEED_TEST_LOW_CM_S, APP_SPEED_TEST_LOW_CM_S);
    }

    Bsp_Motor_SpeedPidUpdate();
    App_UpdateSpeedWatch();
    g_speed_test_loop++;
    if (g_speed_test_loop >=
        (APP_SPEED_TEST_WARMUP_LOOPS + (3U * APP_SPEED_TEST_HOLD_LOOPS))) {
        g_speed_test_loop = 0U;
    }
    Bsp_Gpio_ToggleHeartbeat();
}

/*
 * 执行旧模式专属退出动作。闭环运动模式停止速度控制，原始 PWM 模式主动制动，
 * K230 跟随关闭云台；随后 App_CompleteModeSwitch() 还会执行统一安全清理。
 */
static void App_ModeExit(uint8_t mode)
{
    if ((mode == APP_MODE_LINE_FOLLOW) || (mode == APP_MODE_SPEED_TEST) ||
        (mode == APP_MODE_CIRCLE_FOLLOW) || (mode == APP_MODE_SQUARE_FOLLOW)) {
        Bsp_Motor_SpeedPidStop();
    } else if ((mode == APP_MODE_MOTOR_PWM) ||
               (mode == APP_MODE_CCD_STRAIGHT)) {
        Bsp_Motor_Stop();
    }

    if (mode == APP_MODE_K230_FOLLOW) {
        g_k230_control_error_x = 0;
        g_k230_control_error_y = 0;
        g_k230_pan_delta = 0;
        g_k230_tilt_delta = 0;
        App_K230Follow_ResetPidState();
        Bsp_Ptz_Disable();
    }
}

/*
 * 进入目标模式并完成该模式所需的资源初始化或状态复位。
 * CCD、编码器和速度环只在首次需要时初始化；每次重入都会清理会影响下一次运行的算法历史。
 * 回拨 g_mode_last_task_ms 使新模式的首次周期任务可以立即执行。
 *
 * @return 1 表示模式号受支持且入口完成；0 表示未知模式。
 */
static uint8_t App_ModeEnter(uint8_t mode)
{
    uint32_t now_ms = Bsp_Time_GetMilliseconds();

    g_mode_last_task_ms = now_ms - APP_LOOP_SLOW_MS;
    Bsp_Gpio_SetHeartbeat(0U);

    /* 电机停用态会把 PA12/PA13 切为 GPIO 低电平，进入运动模式前必须恢复 PWM 复用。 */
    if (App_ModeUsesMotor(mode) != 0U) {
        Bsp_Motor_Init();
    }

    switch (mode) {
        case APP_MODE_STOPPED:
            return 1U;
        case APP_MODE_HEARTBEAT:
            return 1U;
        case APP_MODE_K230_UART:
            Protocol_K230_Init();
            return 1U;
        case APP_MODE_MOTOR_PWM:
            g_motor_test_phase = 0U;
            g_motor_test_phase_start_ms = now_ms;
            Bsp_Motor_Set(APP_MOTOR_TEST_SLOW_SPEED, APP_MOTOR_TEST_FAST_SPEED);
            return 1U;
        case APP_MODE_CCD_ADC:
            App_EnsureCcd();
            Bsp_Uart_K230_SendString("BOOT,CCD\r\n");
            return 1U;
        case APP_MODE_UART_TEST:
            g_uart_test_count = 0U;
            Bsp_Uart_K230_SendString("BOOT,UART_TEST\r\n");
            return 1U;
        case APP_MODE_CCD_WATCH:
            App_EnsureCcd();
            return 1U;
        case APP_MODE_LINE_FOLLOW:
            App_EnsureCcd();
            Bsp_Ccd_ResetState(); // 模式重入时丢弃其他 CCD 模式留下的目标和帧历史。
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            return 1U;
        case APP_MODE_CCD_STRAIGHT:
            App_EnsureCcd();
            Bsp_Ccd_ResetState(); // 直行判断必须从新帧开始，不能沿用最近有效黑线。
            App_ResetLineState();
            return 1U;
        case APP_MODE_ENCODER_WATCH:
            App_EnsureEncoder();
            Bsp_Motor_EncoderReset();
            return 1U;
        case APP_MODE_SPEED_TEST:
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidStop();
            Bsp_Motor_EncoderReset();
            g_speed_test_phase = 0U;
            g_speed_test_loop = 0U;
            return 1U;
        case APP_MODE_CIRCLE_FOLLOW:
            App_EnsureCcd();
            Bsp_Ccd_ResetState(); // 圆形巡线重新建立当前赛道的有效中心和边沿。
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            return 1U;
        case APP_MODE_K230_FOLLOW:
            Protocol_K230_Init();
            Bsp_Ptz_Init();
            g_k230_control_error_x = 0;
            g_k230_control_error_y = 0;
            g_k230_pan_delta = 0;
            g_k230_tilt_delta = 0;
            App_K230Follow_ResetPidState();
            g_k230_pan_compare = BSP_PTZ_PAN_CENTER;
            g_k230_tilt_compare = BSP_PTZ_TILT_CENTER;
            Bsp_Ptz_SetCompare(g_k230_tilt_compare, g_k230_pan_compare);
            Bsp_Ptz_Start();
            g_k230_follow_state = APP_K230_FOLLOW_STATE_WAIT_LINK;
            g_k230_follow_update_count = 0U;
            g_k230_follow_seen_timeout_count = g_k230_timeout_count;
            g_k230_follow_start_ms = now_ms;
            return 1U;
        case APP_MODE_SQUARE_FOLLOW:
            App_EnsureCcd();
            Bsp_Ccd_ResetState(); // 清除历史目标，防止弱边沿帧误触发直角制动和找线。
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            App_ResetSquareState();
            return 1U;
        default:
            return 0U;
    }
}

/*
 * 执行当前稳定模式的一次单步任务。各 case 必须尽快返回，任务周期由毫秒门控而非长延时实现。
 * 只观察数据的模式不会驱动电机；所有运动模式的退出安全性由模式管理器统一保证。
 */
static void App_ModeTask(uint8_t mode)
{
    switch (mode) {
        case APP_MODE_STOPPED:
            /* 停止态仅保留协议处理，不运行执行器任务。 */
            break;
        case APP_MODE_HEARTBEAT:
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_SLOW_MS) != 0U) {
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_K230_UART:
            /* 仅验证 K230 文本协议链路，不驱动电机或云台。 */
            Protocol_K230_Task();
            break;
        case APP_MODE_MOTOR_PWM:
            /* 左右轮快慢交替，验证逻辑轮与物理轮映射。 */
            App_MotorTestTask();
            break;
        case APP_MODE_CCD_ADC:
            /* 低频打印完整 CCD 帧，避免调试串口被连续大报文占满。 */
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_PRINT_MS) != 0U) {
                Bsp_Ccd_ReadFrame();
                Bsp_Ccd_Process();
                Bsp_Ccd_PrintDebugFrame();
            }
            break;
        case APP_MODE_UART_TEST:
            /* 每秒发送递增计数，验证 UART TX 和主循环存活。 */
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_SLOW_MS) != 0U) {
                Bsp_Uart_K230_SendString("UART_TEST,count=");
                App_SendUint32(g_uart_test_count++);
                Bsp_Uart_K230_SendString("\r\n");
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_CCD_WATCH:
            /* 不发送大报文，把当前帧和环形日志保存在 RAM 供调试器读取。 */
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_PRINT_MS) != 0U) {
                Bsp_Ccd_ReadFrame();
                Bsp_Ccd_Process();
                App_UpdateCcdWatchData();
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_LINE_FOLLOW:
            /* 普通赛道 profile + 双轮速度闭环。 */
            App_FollowTask(&g_line_follow_profile, 0U);
            break;
        case APP_MODE_CCD_STRAIGHT:
            /* 只用 CCD 有效性控制固定直行/停车，不进行转向反馈。 */
            App_CcdStraightTask();
            break;
        case APP_MODE_ENCODER_WATCH:
            /* 仅采样编码器累计量和窗口速度，电机保持关闭。 */
            if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) != 0U) {
                Bsp_Motor_EncoderSample();
                g_encoder_left_count = Bsp_Motor_GetLeftEncoderCount();
                g_encoder_right_count = Bsp_Motor_GetRightEncoderCount();
                g_encoder_left_speed = Bsp_Motor_GetLeftEncoderSpeed();
                g_encoder_right_speed = Bsp_Motor_GetRightEncoderSpeed();
                Bsp_Gpio_ToggleHeartbeat();
            }
            break;
        case APP_MODE_SPEED_TEST:
            /* 周期执行双轮等速阶跃，观察速度环跟踪和编码器反馈。 */
            App_SpeedTestTask();
            break;
        case APP_MODE_CIRCLE_FOLLOW:
            /* 圆形赛道使用独立 profile 入口，便于与普通巡线分开调参。 */
            App_FollowTask(&g_circle_follow_profile, 0U);
            break;
        case APP_MODE_K230_FOLLOW:
            /* 解析 K230 目标并增量更新双轴云台，链路超时后禁用并在新帧到达时恢复 PWM。 */
            App_K230Follow_Task();
            break;
        case APP_MODE_SQUARE_FOLLOW:
            /* 方形 profile + 直角制动/原地找线状态机。 */
            App_FollowTask(&g_square_follow_profile, 1U);
            break;
        default:
            break;
    }
}

/*
 * 提交一次模式切换：
 * 1. 执行旧模式 Exit；2. 统一禁用电机和云台并复位速度环/编码器；
 * 3. 进入目标模式；4. 成功后才更新 current_mode，失败则回落 STOPPED；
 * 5. 通过 TJC 返回最终成功或失败结果。
 */
static void App_CompleteModeSwitch(void)
{
    uint8_t target_mode = g_app_requested_mode;

    App_ModeExit(g_app_current_mode);
    Bsp_Motor_Disable();
    if (g_speed_pid_initialized != 0U) {
        Bsp_Motor_SpeedPidReset();
    }
    if (g_encoder_initialized != 0U) {
        Bsp_Motor_EncoderReset();
    }
    Bsp_Ptz_Disable();
    Bsp_Ptz_SetCenter();

    if (App_ModeEnter(target_mode) != 0U) {
        g_app_current_mode = target_mode;
        g_app_debug_mode = target_mode;
        g_app_switch_state = APP_SWITCH_IDLE;
        g_app_mode_switch_count++;
        Protocol_Tjc_SendResult(
            (target_mode == APP_MODE_STOPPED) ? TJC_RESULT_STOPPED : TJC_RESULT_SWITCH_OK,
            g_app_current_mode, g_app_switch_request);
    } else {
        g_app_current_mode = APP_MODE_STOPPED;
        g_app_requested_mode = APP_MODE_STOPPED;
        g_app_debug_mode = APP_MODE_STOPPED;
        g_app_switch_state = APP_SWITCH_IDLE;
        g_app_mode_reject_count++;
        Protocol_Tjc_SendResult(
            TJC_RESULT_ENTER_FAILED, g_app_current_mode, g_app_switch_request);
    }
}

/* BRAKING 保持至少 100 ms 后转入 ENTERING；ENTERING 在一次调用内完成退出、清理和进入。 */
static void App_ModeManagerTask(void)
{
    if (g_app_switch_state == APP_SWITCH_BRAKING) {
        if ((uint32_t) (Bsp_Time_GetMilliseconds() - g_app_switch_deadline_ms) <
            APP_MODE_SWITCH_BRAKE_MS) {
            return;
        }
        g_app_switch_state = APP_SWITCH_ENTERING;
    }

    if (g_app_switch_state == APP_SWITCH_ENTERING) {
        App_CompleteModeSwitch();
    }
}

/*
 * 接受外部模式请求并启动异步切换。
 * 同模式请求直接应答；运动模式先主动制动；STOP 已进入切换流程时拒绝新的非停止请求。
 * TJC_RESULT_ACCEPTED_BRAKING 只表示请求已受理，不代表目标模式已经开始运行。
 */
uint8_t App_RequestMode(uint8_t mode)
{
    if (mode > APP_MODE_SQUARE_FOLLOW) {
        g_app_mode_reject_count++;
        return 0U;
    }

    if ((g_app_switch_state == APP_SWITCH_IDLE) &&
        (mode == g_app_current_mode)) {
        Protocol_Tjc_SendResult(
            (mode == APP_MODE_STOPPED) ? TJC_RESULT_STOPPED : TJC_RESULT_ALREADY_ACTIVE,
            g_app_current_mode, mode);
        return 1U;
    }

    if ((g_app_switch_state != APP_SWITCH_IDLE) &&
        (g_app_requested_mode == APP_MODE_STOPPED) &&
        (mode != APP_MODE_STOPPED)) {
        g_app_mode_reject_count++;
        Protocol_Tjc_SendResult(TJC_RESULT_ACCEPTED_BRAKING,
                                g_app_current_mode, APP_MODE_STOPPED);
        return 0U;
    }

    g_app_requested_mode = mode;
    g_app_switch_request = mode;
    Protocol_Tjc_SendResult(
        TJC_RESULT_ACCEPTED_BRAKING, g_app_current_mode, mode);

    if (App_ModeUsesMotor(g_app_current_mode) != 0U) {
        Bsp_Motor_SpeedPidStop();
        Bsp_Motor_Stop();
        g_app_switch_deadline_ms = Bsp_Time_GetMilliseconds();
        g_app_switch_state = APP_SWITCH_BRAKING;
    } else {
        g_app_switch_state = APP_SWITCH_ENTERING;
    }

    return 1U;
}

uint8_t App_GetCurrentMode(void)
{
    return g_app_current_mode;
}

/*
 * 初始化基础 BSP、统一时基和两路协议。初始化期间显式关闭电机和云台，
 * 最终把全部运行时状态复位为 STOPPED，并主动发送一帧初始状态。
 */
void App_Init(void)
{
    Bsp_Gpio_Init();
    Bsp_Uart_Init();
    Bsp_Time_Init();
    Bsp_Motor_Init();
    Bsp_Motor_Disable();
    Bsp_Ptz_Disable();
    Bsp_Ptz_SetCenter();
    PID_InitDefaults();
    Protocol_K230_Init();
    Protocol_Tjc_Init();

    g_app_current_mode = APP_MODE_STOPPED;
    g_app_requested_mode = APP_MODE_STOPPED;
    g_app_debug_mode = APP_MODE_STOPPED;
    g_app_switch_state = APP_SWITCH_IDLE;
    g_app_mode_switch_count = 0U;
    g_app_mode_reject_count = 0U;
    g_ccd_initialized = 0U;
    g_encoder_initialized = 0U;
    g_speed_pid_initialized = 0U;
    App_ResetLineState();
    App_ResetSquareState();
    Protocol_Tjc_SendResult(TJC_RESULT_STATE, APP_MODE_STOPPED, TJC_COMMAND_QUERY);
}

/*
 * 主循环顺序具有安全意义：先读取屏幕命令，再推进制动/进入状态，最后才运行稳定模式任务。
 * 切换状态非空闲时暂停旧模式任务，防止旧控制输出与新模式初始化交叉。
 */
void App_Loop(void)
{
    Protocol_Tjc_Task();
    App_ModeManagerTask();
    if (g_app_switch_state == APP_SWITCH_IDLE) {
        App_ModeTask(g_app_current_mode);
    }
}
