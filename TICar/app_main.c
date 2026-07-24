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
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_ptz.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "delivery_maneuver.h"
#include "delivery_task.h"
#include "pid.h"
#include "protocol_k230.h"
#include "protocol_tjc.h"

#define APP_DEBUG_REQUEST_NONE 0xFFU

/* 电机映射测试：用明显不同的左右轮比例观察底盘偏航方向。 */
#define APP_MOTOR_TEST_SLOW_SPEED 0.18f // 电机映射测试中的低速轮比例参考。
#define APP_MOTOR_TEST_FAST_SPEED 0.45f // 电机映射测试中的高速轮比例参考。

/* CCD Watch 在 RAM 中保留最近若干帧完整像素，供 CCS Memory Browser 导出。 */
#define APP_CCD_LOG_FRAME_COUNT 32U // CCD 完整像素调试环形日志保存的帧数。

/*
 * 通用巡线参数。巡线层比例参考随后统一映射为 cm/s 轮速目标；CCD 与 K230
 * 使用独立误差尺度和方向符号，避免新车头坐标与旧传感器参数相互污染。
 */
#define APP_LINE_BASE_SPEED 0.49f
#define APP_LINE_BASE_TARGET_CM_S 30.0f
#define APP_LINE_CORRECTION_SLEW_LIMIT 0.035f
#define APP_LINE_MEDIUM_SPEED_SCALE 0.65f
#define APP_LINE_LARGE_SPEED_SCALE 0.39f

/* 旧 CCD 模式在车体旋转 180°后左右语义反转，仍需通过架空轮和实车重新确认。 */
#define APP_CCD_LINE_KP 0.0005f
#define APP_CCD_LINE_KD 0.0010f
#define APP_CCD_LINE_STEER_SIGN (-1.0f)
#define APP_CCD_LINE_STEER_LIMIT 0.130f
#define APP_CCD_LINE_MEDIUM_MIN_STEER 0.020f
#define APP_CCD_LINE_LARGE_MIN_STEER 0.090f
#define APP_CCD_LINE_DEADBAND_ERROR 3
#define APP_CCD_LINE_MEDIUM_ERROR 6
#define APP_CCD_LINE_LARGE_ERROR 12

/* K230 使用 320×240 图像；图像右偏和右前方角度均为正，摄像头不镜像。 */
#define APP_K230_LINE_BASE_SPEED 0.25f
#define APP_K230_LINE_KP 0.0015f
#define APP_K230_LINE_KD 0.0100f
#define APP_K230_LINE_ANGLE_KP 0.0020f
#define APP_K230_LINE_STEER_SIGN 1.0f
#define APP_K230_LINE_STEER_LIMIT 0.120f
#define APP_K230_LINE_MEDIUM_MIN_STEER 0.025f
#define APP_K230_LINE_LARGE_MIN_STEER 0.070f
#define APP_K230_LINE_DEADBAND_ERROR 4
#define APP_K230_LINE_ANGLE_DEADBAND_D10 20
#define APP_K230_LINE_MEDIUM_ERROR 20
#define APP_K230_LINE_LARGE_ERROR 45
#define APP_K230_LINE_MIN_QUALITY K230_LINE_MIN_QUALITY
#define APP_K230_LINE_LOST_RECOVER_MAX 5U
#define APP_K230_LINE_LOST_SPEED_SCALE 0.45f
#define APP_K230_LINE_LOST_TURN 0.045f
#define APP_K230_LINE_JUNCTION_SPEED_SCALE 0.55f
#define APP_K230_LINE_JUNCTION_CONFIRM_FRAMES 2U
#define APP_K230_LINE_JUNCTION_CLEAR_FRAMES 3U
#define APP_K230_LINE_JUNCTION_HOLD_MS 300U
#define APP_K230_LINE_RECOVERY_FRAMES 2U

#define APP_LINE_CONTROL_WAIT_FRAME 0U
#define APP_LINE_CONTROL_TRACKING 1U
#define APP_LINE_CONTROL_LOST_SEARCH 2U
#define APP_LINE_CONTROL_JUNCTION_SLOW 3U
#define APP_LINE_CONTROL_TIMEOUT_STOP 4U
#define APP_LINE_CONTROL_MANEUVER_TURN 5U
#define APP_LINE_CONTROL_MANEUVER_REACQUIRE 6U
/* CCD 直行模式只做有效性门控，不使用中心误差进行转向。 */
#define APP_CCD_STRAIGHT_SPEED 0.16f // 模式 8 检测到有效黑线时的固定直行比例参考。
#define APP_IMU_HEADING_KP 0.0030f
#define APP_IMU_HEADING_KD 0.0015f
#define APP_IMU_HEADING_DEADBAND_DEG 0.5f
#define APP_IMU_HEADING_CORRECTION_LIMIT 0.08f
#define APP_IMU_HEADING_CORRECTION_SLEW 0.02f
#define APP_IMU_HEADING_MAX_AGE_MS 100U

/* 普通/圆形巡线允许短时沿最后一次有效方向低速搜索，超过窗口后主动停车。 */
#define APP_LINE_LOST_RECOVER_MAX 8U // 普通和圆形巡线丢线后继续搜索的最大 20 ms 周期数。
#define APP_LINE_LOST_SPEED_SCALE 0.55f // 普通和圆形巡线丢线搜索时的基础速度缩放比例。
#define APP_LINE_LOST_TURN 0.040f // 普通和圆形巡线丢线搜索时附加的转向修正量。

/*
 * 方形路线由 CCD 负责直线纠偏、编码器负责边长和两阶段差动转向。
 * 路线持续循环；全部等待均为非阻塞毫秒时序，TURN 超时以 20 ms 控制周期计数。
 */
#define APP_SQUARE_LOST_RECOVER_MAX 14U // 方形直线段丢线后继续搜索的最大 20 ms 周期数。
#define APP_SQUARE_LOST_SPEED_SCALE 0.45f // 方形直线段丢线搜索时的基础速度缩放比例。
#define APP_SQUARE_LOST_TURN 0.060f // 方形直线段丢线搜索时附加的转向修正量。
#define APP_SQUARE_SIDE_LENGTH_CM 94.0f // 方形路线每条直线边的编码器目标距离，单位 cm。
#define APP_SQUARE_SLOWDOWN_START_CM 94.0f // 直线段开始使用末端接近速度的距离，单位 cm。
#define APP_SQUARE_APPROACH_TARGET_CM_S 20.0f // 直线末端减速接近时的目标轮速，单位 cm/s。
#define APP_SQUARE_EFFECTIVE_TRACK_CM 8.8f // 根据实车转向标定得到的有效轮距，决定编码器转角距离。
#define APP_SQUARE_WAIT_LINE_LOOPS 3U // 等待启动或转向结束后确认真实黑线所需的连续帧数。
#define APP_SQUARE_BRAKE_AFTER_STRAIGHT_MS 300U // 每条直线结束后转向前的主动制动时间，单位 ms。
#define APP_SQUARE_BRAKE_AFTER_TURN_MS 300U // 每次转向结束后重新巡线前的主动制动时间，单位 ms。
#define APP_SQUARE_TURN_TIMEOUT_LOOPS 120U // 整次转向允许的最大 20 ms 控制周期数。
#define APP_SQUARE_SPEED_FAULT_CONFIRM_LOOPS 25U // 底层速度故障连续保持 25 个控制周期后才锁存路线故障。
#define APP_SQUARE_TURN_TOTAL_ANGLE_DEG 100.0f // CCD 未提前找到线时编码器允许的最大总转角，单位度。
#define APP_SQUARE_TURN_COARSE_ANGLE_DEG 60.0f // 进入 CCD 细转找线前的编码器粗转角度，单位度。
#define APP_SQUARE_TURN_SPEED_REFERENCE 0.30f // 粗转阶段的轮速比例参考。
#define APP_SQUARE_TURN_FINE_SPEED_REFERENCE 0.19f // CCD 找线细转阶段稳定运行时的轮速比例参考。
#define APP_SQUARE_TURN_FINE_START_MIN_REFERENCE 0.19f // 细转刚启动时为克服静摩擦使用的最低比例参考。
#define APP_SQUARE_TURN_FINE_START_LOOPS 5U // 细转最低启动参考保持的 20 ms 周期数。
#define APP_SQUARE_TURN_LINE_ERROR 12 // 细转时允许判定为已对准新线的最大中心像素误差。
#define APP_SQUARE_TURN_LINE_LOOPS 3U // 细转提前结束前要求连续识别到新线的帧数。
#define APP_SQUARE_TURN_DIRECTION (-1) // 固定转向方向：-1 与 +1 分别对应相反的差动转向方向。
#define APP_SQUARE_TURN_PHASE_COARSE 0U // 方形路线转向状态中的编码器粗转阶段编号。
#define APP_SQUARE_TURN_PHASE_FINE 1U // 方形路线转向状态中的 CCD 低速找线阶段编号。
#define APP_SQUARE_STATE_WAIT_LINE 0U // 方形路线停车等待可靠黑线状态。
#define APP_SQUARE_STATE_STRAIGHT 1U // 方形路线 CCD 纠偏定距直线状态。
#define APP_SQUARE_STATE_BRAKE_AFTER_STRAIGHT 2U // 直线结束后转向前的制动等待状态。
#define APP_SQUARE_STATE_TURN 3U // 方形路线编码器粗转和 CCD 细转状态。
#define APP_SQUARE_STATE_BRAKE_AFTER_TURN 4U // 转向结束后下一条直线前的制动等待状态。
#define APP_SQUARE_STATE_FAULT 5U // 方形路线故障锁存并保持停车状态。
#define APP_SQUARE_FAULT_NONE 0U // 方形路线当前无故障。
#define APP_SQUARE_FAULT_SPEED 1U // 速度闭环或编码器反馈异常故障。
#define APP_SQUARE_FAULT_TURN_TIMEOUT 2U // 转向超过最大控制周期仍未完成故障。
#define APP_SQUARE_FAULT_INTERNAL_STATE 3U // 遇到未定义路线状态或转向阶段的内部状态故障。

/* 速度测试以 20 ms 为一步，循环执行静止、低速、高速、低速四个阶段。 */
#define APP_SPEED_TEST_LOW_CM_S 20.0f // 模式 10 低速阶段的左右轮目标速度，单位 cm/s。
#define APP_SPEED_TEST_HIGH_CM_S 30.0f // 模式 10 高速阶段的左右轮目标速度，单位 cm/s。
#define APP_SPEED_TEST_WARMUP_LOOPS 250U // 速度测试静止预热阶段持续的 20 ms 周期数。
#define APP_SPEED_TEST_HOLD_LOOPS 250U // 速度测试每个非静止速度阶段持续的 20 ms 周期数。

/*
 * K230 云台增量 PID 参数。
 * 图像误差来自 640×360 坐标系；两轴分别计算 P/I/D，合成后限制单帧 compare 增量。
 * Ki 当前保持关闭，但保留积分状态和限幅，便于后续实车调参。
 */
#define APP_K230_FOLLOW_PAN_KP_NUM 8 // 水平轴比例系数的分子。
#define APP_K230_FOLLOW_PAN_KP_DEN 100 // 水平轴比例系数的分母，实际 Kp 为 NUM/DEN。
#define APP_K230_FOLLOW_PAN_KI_NUM 0 // 水平轴积分系数的分子，0 表示关闭积分修正。
#define APP_K230_FOLLOW_PAN_KI_DEN 100 // 水平轴积分系数的分母，实际 Ki 为 NUM/DEN。
#define APP_K230_FOLLOW_PAN_KD_NUM 4 // 水平轴微分系数的分子。
#define APP_K230_FOLLOW_PAN_KD_DEN 100 // 水平轴微分系数的分母，实际 Kd 为 NUM/DEN。
#define APP_K230_FOLLOW_PAN_INTEGRAL_LIMIT 1000 // 水平轴积分误差累计值的绝对值上限。
#define APP_K230_FOLLOW_TILT_KP_NUM 8 // 垂直轴比例系数的分子。
#define APP_K230_FOLLOW_TILT_KP_DEN 100 // 垂直轴比例系数的分母，实际 Kp 为 NUM/DEN。
#define APP_K230_FOLLOW_TILT_KI_NUM 0 // 垂直轴积分系数的分子，0 表示关闭积分修正。
#define APP_K230_FOLLOW_TILT_KI_DEN 100 // 垂直轴积分系数的分母，实际 Ki 为 NUM/DEN。
#define APP_K230_FOLLOW_TILT_KD_NUM 4 // 垂直轴微分系数的分子。
#define APP_K230_FOLLOW_TILT_KD_DEN 100 // 垂直轴微分系数的分母，实际 Kd 为 NUM/DEN。
#define APP_K230_FOLLOW_TILT_INTEGRAL_LIMIT 1000 // 垂直轴积分误差累计值的绝对值上限。
#define APP_K230_FOLLOW_X_DEADBAND 10 // 水平图像误差绝对值不超过该像素数时不调整云台。
#define APP_K230_FOLLOW_Y_DEADBAND 8 // 垂直图像误差绝对值不超过该像素数时不调整云台。
#define APP_K230_FOLLOW_PAN_MAX_STEP 20 // 水平轴单帧允许变化的最大 PWM compare 增量。
#define APP_K230_FOLLOW_TILT_MAX_STEP 15 // 垂直轴单帧允许变化的最大 PWM compare 增量。
#define APP_K230_FOLLOW_PAN_DIRECTION (-1) // 水平图像误差映射到云台移动方向的符号。
#define APP_K230_FOLLOW_TILT_DIRECTION 1 // 垂直图像误差映射到云台移动方向的符号。
#define APP_K230_FOLLOW_STARTUP_HOLD_MS 2000U // 模式 12 启动后等待 K230 链路稳定的保持时间，单位 ms。
#define APP_K230_FOLLOW_STATE_WAIT_LINK 0U // K230 云台跟随等待通信链路状态编号。
#define APP_K230_FOLLOW_STATE_TRACKING 1U // K230 云台正常跟踪目标状态编号。
#define APP_K230_FOLLOW_STATE_HOLD 2U // K230 暂时无新目标时保持当前位置状态编号。
#define APP_K230_FOLLOW_STATE_TIMEOUT_DISABLED 3U // K230 链路超时后禁用云台输出状态编号。

#define APP_MODE_SWITCH_BRAKE_MS 100U // 运动模式切换前主动制动的保持时间，单位 ms。
#define APP_LOOP_FAST_MS 20U // CCD、编码器和速度闭环任务的快速调度周期，单位 ms。
#define APP_LOOP_PRINT_MS 500U // CCD 调试打印和 Watch 日志更新的低频周期，单位 ms。
#define APP_LOOP_SLOW_MS 1000U // 心跳和 UART 测试等慢速任务的调度周期，单位 ms。
#define APP_SWITCH_IDLE 0U // 模式管理器处于稳定运行且没有切换请求的状态编号。
#define APP_SWITCH_BRAKING 1U // 模式管理器正在执行切换前主动制动的状态编号。
#define APP_SWITCH_ENTERING 2U // 模式管理器正在退出旧模式并进入新模式的状态编号。

/*
 * 三种巡线模式共享的参数集合。
 * medium/large_error 以像素为单位；速度和转向字段为巡线层比例参考；lost_recover_max 为 20 ms 循环次数。
 */
typedef struct {
    float base_speed;             /* 中心附近的基础前进参考。 */
    float kp;                     /* 横向像素误差比例增益。 */
    float kd;                     /* 新观测横向误差差分增益。 */
    float angle_kp;               /* 方向角前馈增益，CCD 配置为 0。 */
    float steer_sign;             /* 感知坐标到新车体差速方向的符号。 */
    float steer_limit;            /* 最大转向修正绝对值。 */
    int16_t deadband_error;       /* 横向误差死区。 */
    int16_t angle_deadband_d10;   /* 0.1°方向角死区。 */
    int16_t medium_error;         /* 开始降速的中等误差阈值。 */
    int16_t large_error;          /* 进入强恢复的较大误差阈值。 */
    uint8_t min_quality;          /* 低于该质量的观测视为丢线。 */
    float medium_speed_scale;     /* 中等误差时的基础速度缩放。 */
    float large_speed_scale;      /* 大误差时的基础速度缩放。 */
    float medium_min_steer;       /* 中等误差必须保留的最小转向量。 */
    float large_min_steer;        /* 大误差必须保留的最小转向量。 */
    uint16_t lost_recover_max;    /* 丢线后仍尝试搜索的最大循环数。 */
    float lost_speed_scale;       /* 丢线搜索时的基础速度缩放。 */
    float lost_turn;              /* 丢线搜索时沿最后有效方向施加的转向量。 */
} App_FollowProfile;

typedef struct {
    uint8_t source;
    uint8_t fresh;
    uint8_t new_frame;
    uint8_t valid;
    int16_t target;
    int16_t error;
    int16_t angle_d10;
    uint8_t quality;
    uint8_t direction_mask;
} App_LineObservation;

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

/* 普通比赛巡线使用 K230 红线观测，初始速度和增益保持保守。 */
static const App_FollowProfile g_k230_line_follow_profile = {
    APP_K230_LINE_BASE_SPEED,
    APP_K230_LINE_KP,
    APP_K230_LINE_KD,
    APP_K230_LINE_ANGLE_KP,
    APP_K230_LINE_STEER_SIGN,
    APP_K230_LINE_STEER_LIMIT,
    APP_K230_LINE_DEADBAND_ERROR,
    APP_K230_LINE_ANGLE_DEADBAND_D10,
    APP_K230_LINE_MEDIUM_ERROR,
    APP_K230_LINE_LARGE_ERROR,
    APP_K230_LINE_MIN_QUALITY,
    APP_LINE_MEDIUM_SPEED_SCALE,
    APP_LINE_LARGE_SPEED_SCALE,
    APP_K230_LINE_MEDIUM_MIN_STEER,
    APP_K230_LINE_LARGE_MIN_STEER,
    APP_K230_LINE_LOST_RECOVER_MAX,
    APP_K230_LINE_LOST_SPEED_SCALE,
    APP_K230_LINE_LOST_TURN
};

/* 圆形和方形旧模式继续使用 CCD，并与 K230 新车头参数完全分离。 */
static const App_FollowProfile g_circle_follow_profile = {
    APP_LINE_BASE_SPEED,
    APP_CCD_LINE_KP,
    APP_CCD_LINE_KD,
    0.0f,
    APP_CCD_LINE_STEER_SIGN,
    APP_CCD_LINE_STEER_LIMIT,
    APP_CCD_LINE_DEADBAND_ERROR,
    0,
    APP_CCD_LINE_MEDIUM_ERROR,
    APP_CCD_LINE_LARGE_ERROR,
    0U,
    APP_LINE_MEDIUM_SPEED_SCALE,
    APP_LINE_LARGE_SPEED_SCALE,
    APP_CCD_LINE_MEDIUM_MIN_STEER,
    APP_CCD_LINE_LARGE_MIN_STEER,
    APP_LINE_LOST_RECOVER_MAX,
    APP_LINE_LOST_SPEED_SCALE,
    APP_LINE_LOST_TURN
};

static const App_FollowProfile g_square_follow_profile = {
    APP_LINE_BASE_SPEED,
    APP_CCD_LINE_KP,
    APP_CCD_LINE_KD,
    0.0f,
    APP_CCD_LINE_STEER_SIGN,
    APP_CCD_LINE_STEER_LIMIT,
    APP_CCD_LINE_DEADBAND_ERROR,
    0,
    APP_CCD_LINE_MEDIUM_ERROR,
    APP_CCD_LINE_LARGE_ERROR,
    0U,
    APP_LINE_MEDIUM_SPEED_SCALE,
    APP_LINE_LARGE_SPEED_SCALE,
    APP_CCD_LINE_MEDIUM_MIN_STEER,
    APP_CCD_LINE_LARGE_MIN_STEER,
    APP_SQUARE_LOST_RECOVER_MAX,
    APP_SQUARE_LOST_SPEED_SCALE,
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
volatile uint8_t g_app_debug_request_mode = APP_DEBUG_REQUEST_NONE;
volatile uint8_t g_app_debug_request_result;
volatile uint32_t g_app_debug_request_count;

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

/* K230 红线巡线 Watch 和安全状态。 */
volatile uint8_t g_line_source = APP_LINE_SOURCE_DEFAULT;
volatile uint8_t g_line_control_state = APP_LINE_CONTROL_WAIT_FRAME;
volatile uint32_t g_k230_line_age_ms = UINT32_MAX;
volatile int16_t g_k230_line_angle_d10;
volatile uint8_t g_k230_line_quality;
volatile uint8_t g_k230_line_direction_mask;
volatile uint8_t g_k230_junction_active;
volatile uint8_t g_k230_junction_confirm_count;
volatile uint8_t g_k230_junction_clear_count;
volatile uint32_t g_k230_junction_last_side_ms;
volatile uint8_t g_k230_line_recovery_count;
volatile uint32_t g_k230_line_timeout_stop_count;
volatile uint32_t g_k230_line_recovery_total;
static K230_LineFrame g_k230_cached_line_frame;
static uint8_t g_k230_has_cached_line_frame;
static uint8_t g_k230_line_timeout_latched;

/* 送药任务状态与路线决策 Watch；瞬时数字和锁定目标由 delivery_task 分离管理。 */
static DeliveryTask g_delivery_task;
static uint8_t g_delivery_start_pending;
static uint8_t g_delivery_epoch;
volatile uint8_t g_delivery_state;
volatile uint8_t g_delivery_target_digit;
volatile uint8_t g_delivery_target_locked;
volatile uint8_t g_delivery_route_region;
volatile uint8_t g_delivery_junction_id;
volatile uint8_t g_delivery_pending_decision;
volatile uint8_t g_delivery_visual_mode;
volatile uint8_t g_delivery_visual_command_pending;
volatile uint8_t g_delivery_fault;

/* 路口动作执行器 Watch：状态、故障和轮速目标用于实车标定 IMU 转角与重捕获过程。 */
static DeliveryManeuver g_delivery_maneuver;
volatile uint8_t g_delivery_maneuver_state;
volatile uint8_t g_delivery_maneuver_turn_phase;
volatile uint8_t g_delivery_maneuver_fault;
volatile float g_delivery_maneuver_heading_target_deg;
volatile float g_delivery_maneuver_left_target_cm_s;
volatile float g_delivery_maneuver_right_target_cm_s;

/* 模式 8 的 IMU 航向保持；传感器坐标约定为左转正、右转负。 */
volatile uint8_t g_imu_heading_hold_active;
volatile uint32_t g_imu_heading_hold_stale_count;
volatile float g_imu_heading_target_deg;
volatile float g_imu_heading_error_deg;
volatile float g_imu_heading_correction;
volatile float g_imu_heading_left_cmd;
volatile float g_imu_heading_right_cmd;
static float g_imu_heading_last_error_deg;
static float g_imu_heading_last_correction;

/*
 * 方形混合路线状态：CCD 只拥有直线段电机控制权，编码器只拥有转向段控制权。
 * start_count 保存各段编码器起点；距离、边数、圈数和故障量均可在 CCS Watch 中观察。
 */
volatile uint8_t g_square_route_state;
volatile uint8_t g_square_fault_reason;
volatile uint32_t g_square_fault_speed_bits;
volatile uint8_t g_square_speed_fault_stable_count;
volatile uint32_t g_square_state_start_ms;
volatile uint32_t g_square_state_elapsed_ms;
volatile uint8_t g_square_wait_line_stable_count;
volatile uint8_t g_square_side_index;
volatile uint32_t g_square_completed_side_count;
volatile uint32_t g_square_lap_count;
volatile int32_t g_square_left_start_count;
volatile int32_t g_square_right_start_count;
volatile float g_square_straight_left_cm;
volatile float g_square_straight_right_cm;
volatile float g_square_straight_center_cm;
volatile float g_square_straight_output_scale;
volatile int8_t g_square_turn_direction;
volatile uint8_t g_square_turn_phase;
volatile uint8_t g_square_turn_fine_start_count;
volatile uint8_t g_square_turn_line_stable_count;
volatile uint8_t g_square_turn_line_found;
volatile float g_square_turn_coarse_target_cm;
volatile float g_square_turn_target_cm;
volatile float g_square_turn_left_cm;
volatile float g_square_turn_right_cm;
volatile uint8_t g_square_turn_left_done;
volatile uint8_t g_square_turn_right_done;
volatile uint32_t g_square_turn_loop_count;
volatile uint32_t g_square_turn_timeout_count;

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
    Bsp_Motor_EncoderSnapshot snapshot;

    Bsp_Motor_GetEncoderSnapshot(&snapshot);
    g_encoder_left_count = snapshot.left_count;
    g_encoder_right_count = snapshot.right_count;
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

static float App_LineReferenceToCmS(float reference)
{
    return reference * (APP_LINE_BASE_TARGET_CM_S / APP_LINE_BASE_SPEED);
}

/*
 * 将巡线层左右比例参考按已验证的直线基准映射为 cm/s，再运行两个独立轮速控制器。
 * 因此 Watch 中的巡线命令、轮速目标和最终 PWM 输出属于三个不同层次。
 */
static void App_LineApplySpeedPid(float left_reference, float right_reference)
{
    Bsp_Motor_SetSpeedTargets(App_LineReferenceToCmS(left_reference),
                              App_LineReferenceToCmS(right_reference));
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

    g_line_control_state = APP_LINE_CONTROL_WAIT_FRAME;
    g_k230_line_age_ms = UINT32_MAX;
    g_k230_line_angle_d10 = 0;
    g_k230_line_quality = 0U;
    g_k230_line_direction_mask = 0U;
    g_k230_junction_active = 0U;
    g_k230_junction_confirm_count = 0U;
    g_k230_junction_clear_count = 0U;
    g_k230_junction_last_side_ms = 0U;
    g_k230_line_recovery_count = 0U;
    g_k230_has_cached_line_frame = 0U;
    g_k230_line_timeout_latched = 0U;
    g_k230_cached_line_frame.valid = 0U;
    g_k230_cached_line_frame.error_x = 0;
    g_k230_cached_line_frame.angle_d10 = 0;
    g_k230_cached_line_frame.quality = 0U;
    g_k230_cached_line_frame.direction_mask = 0U;
}

static void App_ResetImuHeadingHold(void)
{
    g_imu_heading_hold_active = 0U;
    g_imu_heading_hold_stale_count = 0U;
    g_imu_heading_target_deg = 0.0f;
    g_imu_heading_error_deg = 0.0f;
    g_imu_heading_correction = 0.0f;
    g_imu_heading_left_cmd = 0.0f;
    g_imu_heading_right_cmd = 0.0f;
    g_imu_heading_last_error_deg = 0.0f;
    g_imu_heading_last_correction = 0.0f;
}

/* 清除方形路线距离、圈数和故障锁存，模式重入后从停车等待可靠中心线开始。 */
static void App_ResetSquareState(void)
{
    g_square_route_state = APP_SQUARE_STATE_WAIT_LINE;
    g_square_fault_reason = APP_SQUARE_FAULT_NONE;
    g_square_fault_speed_bits = 0U;
    g_square_speed_fault_stable_count = 0U;
    g_square_state_start_ms = Bsp_Time_GetMilliseconds();
    g_square_state_elapsed_ms = 0U;
    g_square_wait_line_stable_count = 0U;
    g_square_side_index = 0U;
    g_square_completed_side_count = 0U;
    g_square_lap_count = 0U;
    g_square_left_start_count = 0;
    g_square_right_start_count = 0;
    g_square_straight_left_cm = 0.0f;
    g_square_straight_right_cm = 0.0f;
    g_square_straight_center_cm = 0.0f;
    g_square_straight_output_scale = 0.0f;
    g_square_turn_direction = APP_SQUARE_TURN_DIRECTION;
    g_square_turn_phase = APP_SQUARE_TURN_PHASE_COARSE;
    g_square_turn_fine_start_count = 0U;
    g_square_turn_line_stable_count = 0U;
    g_square_turn_line_found = 0U;
    g_square_turn_coarse_target_cm =
        3.14159265f * APP_SQUARE_EFFECTIVE_TRACK_CM *
        APP_SQUARE_TURN_COARSE_ANGLE_DEG / 360.0f;
    g_square_turn_target_cm =
        3.14159265f * APP_SQUARE_EFFECTIVE_TRACK_CM *
        APP_SQUARE_TURN_TOTAL_ANGLE_DEG / 360.0f;
    g_square_turn_left_cm = 0.0f;
    g_square_turn_right_cm = 0.0f;
    g_square_turn_left_done = 0U;
    g_square_turn_right_done = 0U;
    g_square_turn_loop_count = 0U;
    g_square_turn_timeout_count = 0U;
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

static uint8_t App_FollowTask(
    const App_FollowProfile *profile,
    float output_scale,
    uint8_t source);

/* 将一帧 CCD 质量和边沿信息镜像到巡线 Watch，不在此函数中驱动电机。 */
static void App_UpdateLineSensorWatch(void)
{
    g_line_raw_min = Bsp_Ccd_GetRawMin();
    g_line_raw_max = Bsp_Ccd_GetRawMax();
    g_line_raw_min_index = Bsp_Ccd_GetRawMinIndex();
    g_line_raw_max_index = Bsp_Ccd_GetRawMaxIndex();
    g_line_contrast = Bsp_Ccd_GetContrast();
    g_line_dx_max = Bsp_Ccd_GetDxMax();
    g_line_dx_min = Bsp_Ccd_GetDxMin();
    g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();
}

static uint32_t App_EncoderAbsDelta(int32_t current, int32_t start)
{
    int32_t delta = current - start;

    return (delta < 0) ? (uint32_t) (-delta) : (uint32_t) delta;
}

static void App_SquareSetState(uint8_t state)
{
    g_square_route_state = state;
    g_square_state_start_ms = Bsp_Time_GetMilliseconds();
    g_square_state_elapsed_ms = 0U;
}

/*
 * 底层速度故障是可自动清除的实时告警；只有连续保持指定周期，
 * 方形路线才将其升级为需要重新进入模式才能清除的锁存故障。
 */
static uint8_t App_SquareConfirmSpeedFault(void)
{
    if (Bsp_Motor_GetSpeedFaults() == 0U) {
        g_square_speed_fault_stable_count = 0U;
        return 0U;
    }

    if (g_square_speed_fault_stable_count < APP_SQUARE_SPEED_FAULT_CONFIRM_LOOPS) {
        g_square_speed_fault_stable_count++;
    }

    return (g_square_speed_fault_stable_count >=
            APP_SQUARE_SPEED_FAULT_CONFIRM_LOOPS) ? 1U : 0U;
}

static void App_SquareEnterFault(uint8_t reason)
{
    if (g_square_route_state == APP_SQUARE_STATE_FAULT) {
        return;
    }

    g_square_fault_reason = reason;
    g_square_fault_speed_bits = Bsp_Motor_GetSpeedFaults();
    Bsp_Motor_SpeedPidStop();
    App_UpdateSpeedWatch();
    g_line_correction = 0.0f;
    g_line_left_cmd = 0.0f;
    g_line_right_cmd = 0.0f;
    g_square_straight_output_scale = 0.0f;
    App_SquareSetState(APP_SQUARE_STATE_FAULT);
}

static void App_SquareEnterWaitLine(void)
{
    Bsp_Motor_SpeedPidStop();
    Bsp_Ccd_ResetState();
    App_ResetLineState();
    g_square_wait_line_stable_count = 0U;
    g_square_speed_fault_stable_count = 0U;
    g_square_straight_output_scale = 0.0f;
    App_SquareSetState(APP_SQUARE_STATE_WAIT_LINE);
    g_mode_last_task_ms = Bsp_Time_GetMilliseconds() - APP_LOOP_FAST_MS;
}

/*
 * 方形混合路线状态机：CCD 只控制直线方向，编码器距离是直线结束和 90°转向结束的唯一依据。
 * 所有 300 ms 制动均使用毫秒状态等待，主循环始终可以优先处理 TJC STOP 和模式切换。
 */
static void App_SquareRouteTask(void)
{
    Bsp_Motor_EncoderSnapshot snapshot;
    uint32_t now_ms = Bsp_Time_GetMilliseconds();

    g_square_state_elapsed_ms = (uint32_t) (now_ms - g_square_state_start_ms);

    if (g_square_route_state == APP_SQUARE_STATE_WAIT_LINE) {
        int16_t line_error;
        uint8_t reliable_line;

        if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
            return;
        }

        Bsp_Ccd_ReadFrame();
        Bsp_Ccd_Process();
        App_UpdateLineSensorWatch();
        line_error = Bsp_Ccd_GetLineError();
        /* 只要求连续检测到真实黑线；偏离中心时由 STRAIGHT 巡线控制自行纠正，避免启动死锁。 */
        reliable_line = ((Bsp_Ccd_IsLineValid() != 0U) &&
                         (Bsp_Ccd_GetBlackWidth() != 0U)) ? 1U : 0U;

        g_line_valid = reliable_line;
        g_line_target = reliable_line ? Bsp_Ccd_GetTargetIndex() : -1;
        g_line_error = reliable_line ? line_error : 0;
        g_line_error_delta = 0;
        g_line_correction = 0.0f;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;

        if (reliable_line != 0U) {
            if (g_square_wait_line_stable_count < APP_SQUARE_WAIT_LINE_LOOPS) {
                g_square_wait_line_stable_count++;
            }
        } else {
            g_square_wait_line_stable_count = 0U;
        }

        if (g_square_wait_line_stable_count >= APP_SQUARE_WAIT_LINE_LOOPS) {
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_GetEncoderSnapshot(&snapshot);
            g_square_left_start_count = snapshot.left_count;
            g_square_right_start_count = snapshot.right_count;
            g_square_straight_left_cm = 0.0f;
            g_square_straight_right_cm = 0.0f;
            g_square_straight_center_cm = 0.0f;
            g_square_straight_output_scale = 1.0f;
            g_line_last_error = line_error;
            g_line_last_correction = 0.0f;
            g_line_lost_count = 0U;
            g_square_speed_fault_stable_count = 0U;
            App_SquareSetState(APP_SQUARE_STATE_STRAIGHT);
            g_mode_last_task_ms = now_ms - APP_LOOP_FAST_MS;
        }
        Bsp_Gpio_ToggleHeartbeat();
        return;
    }

    if (g_square_route_state == APP_SQUARE_STATE_STRAIGHT) {
        uint32_t left_ticks;
        uint32_t right_ticks;

        Bsp_Motor_GetEncoderSnapshot(&snapshot);
        left_ticks = App_EncoderAbsDelta(snapshot.left_count, g_square_left_start_count);
        right_ticks = App_EncoderAbsDelta(snapshot.right_count, g_square_right_start_count);
        g_square_straight_left_cm = Bsp_Motor_EncoderTicksToCm(left_ticks);
        g_square_straight_right_cm = Bsp_Motor_EncoderTicksToCm(right_ticks);
        g_square_straight_center_cm =
            (g_square_straight_left_cm + g_square_straight_right_cm) * 0.5f;

        if (g_square_straight_center_cm >= APP_SQUARE_SIDE_LENGTH_CM) {
            Bsp_Motor_SpeedPidStop();
            App_UpdateSpeedWatch();
            g_line_correction = 0.0f;
            g_line_left_cmd = 0.0f;
            g_line_right_cmd = 0.0f;
            g_square_straight_output_scale = 0.0f;
            App_SquareSetState(APP_SQUARE_STATE_BRAKE_AFTER_STRAIGHT);
            return;
        }

        g_square_straight_output_scale =
            (g_square_straight_center_cm >= APP_SQUARE_SLOWDOWN_START_CM) ?
            (APP_SQUARE_APPROACH_TARGET_CM_S / APP_LINE_BASE_TARGET_CM_S) : 1.0f;
        if (App_FollowTask(&g_square_follow_profile,
                           g_square_straight_output_scale,
                           APP_LINE_SOURCE_CCD) != 0U) {
            if (App_SquareConfirmSpeedFault() != 0U) {
                App_SquareEnterFault(APP_SQUARE_FAULT_SPEED);
            }
        }
        return;
    }

    if (g_square_route_state == APP_SQUARE_STATE_BRAKE_AFTER_STRAIGHT) {
        if (g_square_state_elapsed_ms >= APP_SQUARE_BRAKE_AFTER_STRAIGHT_MS) {
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_GetEncoderSnapshot(&snapshot);
            g_square_left_start_count = snapshot.left_count;
            g_square_right_start_count = snapshot.right_count;
            g_square_turn_left_cm = 0.0f;
            g_square_turn_right_cm = 0.0f;
            g_square_turn_left_done = 0U;
            g_square_turn_right_done = 0U;
            g_square_turn_loop_count = 0U;
            g_square_turn_direction = APP_SQUARE_TURN_DIRECTION;
            g_square_turn_phase = APP_SQUARE_TURN_PHASE_COARSE;
            g_square_turn_fine_start_count = 0U;
            g_square_turn_line_stable_count = 0U;
            g_square_turn_line_found = 0U;
            g_square_speed_fault_stable_count = 0U;
            App_SquareSetState(APP_SQUARE_STATE_TURN);
            g_mode_last_task_ms = now_ms - APP_LOOP_FAST_MS;
        }
        return;
    }

    if (g_square_route_state == APP_SQUARE_STATE_TURN) {
        float active_target_cm;
        float turn_reference;
        float turn_speed_cm_s;
        float left_target_cm_s;
        float right_target_cm_s;
        uint32_t left_ticks;
        uint32_t right_ticks;

        if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
            return;
        }

        Bsp_Motor_GetEncoderSnapshot(&snapshot);
        left_ticks = App_EncoderAbsDelta(snapshot.left_count, g_square_left_start_count);
        right_ticks = App_EncoderAbsDelta(snapshot.right_count, g_square_right_start_count);
        g_square_turn_left_cm = Bsp_Motor_EncoderTicksToCm(left_ticks);
        g_square_turn_right_cm = Bsp_Motor_EncoderTicksToCm(right_ticks);

        if (g_square_turn_phase == APP_SQUARE_TURN_PHASE_COARSE) {
            active_target_cm = g_square_turn_coarse_target_cm;
            g_square_turn_left_done =
                (g_square_turn_left_cm >= active_target_cm) ? 1U : 0U;
            g_square_turn_right_done =
                (g_square_turn_right_cm >= active_target_cm) ? 1U : 0U;

            if ((g_square_turn_left_done != 0U) &&
                (g_square_turn_right_done != 0U)) {
                /* 粗转结束后重启速度环并短暂抬高细转参考，避免低速目标无法克服静摩擦。 */
                g_square_turn_phase = APP_SQUARE_TURN_PHASE_FINE;
                g_square_turn_fine_start_count = 0U;
                g_square_turn_line_stable_count = 0U;
                g_square_speed_fault_stable_count = 0U;
                Bsp_Motor_SpeedPidReset();
                Bsp_Ccd_ResetState();
            }
        } else if (g_square_turn_phase != APP_SQUARE_TURN_PHASE_FINE) {
            App_SquareEnterFault(APP_SQUARE_FAULT_INTERNAL_STATE);
            return;
        }

        if (g_square_turn_phase == APP_SQUARE_TURN_PHASE_FINE) {
            int16_t line_error;
            int16_t abs_error;
            uint8_t reliable_line;

            Bsp_Ccd_ReadFrame();
            Bsp_Ccd_Process();
            App_UpdateLineSensorWatch();
            line_error = Bsp_Ccd_GetLineError();
            abs_error = (line_error < 0) ? (int16_t) (-line_error) : line_error;
            reliable_line = ((Bsp_Ccd_IsLineValid() != 0U) &&
                             (Bsp_Ccd_GetBlackWidth() != 0U) &&
                             (abs_error <= APP_SQUARE_TURN_LINE_ERROR)) ? 1U : 0U;

            g_line_valid = reliable_line;
            g_line_target = reliable_line ? Bsp_Ccd_GetTargetIndex() : -1;
            g_line_error = reliable_line ? line_error : 0;
            g_line_error_delta = 0;
            g_line_correction = 0.0f;

            if (reliable_line != 0U) {
                if (g_square_turn_line_stable_count < APP_SQUARE_TURN_LINE_LOOPS) {
                    g_square_turn_line_stable_count++;
                }
            } else {
                g_square_turn_line_stable_count = 0U;
            }

            if (g_square_turn_line_stable_count >= APP_SQUARE_TURN_LINE_LOOPS) {
                g_square_turn_line_found = 1U;
                Bsp_Motor_SpeedPidStop();
                App_UpdateSpeedWatch();
                g_line_left_cmd = 0.0f;
                g_line_right_cmd = 0.0f;
                App_SquareSetState(APP_SQUARE_STATE_BRAKE_AFTER_TURN);
                return;
            }

            active_target_cm = g_square_turn_target_cm;
            g_square_turn_left_done =
                (g_square_turn_left_cm >= active_target_cm) ? 1U : 0U;
            g_square_turn_right_done =
                (g_square_turn_right_cm >= active_target_cm) ? 1U : 0U;
        }

        if ((g_square_turn_left_done != 0U) &&
            (g_square_turn_right_done != 0U)) {
            Bsp_Motor_SpeedPidStop();
            App_UpdateSpeedWatch();
            g_line_left_cmd = 0.0f;
            g_line_right_cmd = 0.0f;
            App_SquareSetState(APP_SQUARE_STATE_BRAKE_AFTER_TURN);
            return;
        }

        if (g_square_turn_phase == APP_SQUARE_TURN_PHASE_COARSE) {
            turn_reference = APP_SQUARE_TURN_SPEED_REFERENCE;
        } else {
            turn_reference = APP_SQUARE_TURN_FINE_SPEED_REFERENCE;
            if (g_square_turn_fine_start_count < APP_SQUARE_TURN_FINE_START_LOOPS) {
                if (turn_reference < APP_SQUARE_TURN_FINE_START_MIN_REFERENCE) {
                    turn_reference = APP_SQUARE_TURN_FINE_START_MIN_REFERENCE;
                }
                g_square_turn_fine_start_count++;
            }
        }
        turn_speed_cm_s = App_LineReferenceToCmS(turn_reference);
        left_target_cm_s = (g_square_turn_left_done != 0U) ? 0.0f :
                           ((float) g_square_turn_direction * turn_speed_cm_s);
        right_target_cm_s = (g_square_turn_right_done != 0U) ? 0.0f :
                            (-(float) g_square_turn_direction * turn_speed_cm_s);
        Bsp_Motor_SetSpeedTargets(left_target_cm_s, right_target_cm_s);
        Bsp_Motor_SpeedPidUpdate();
        App_UpdateSpeedWatch();
        g_line_left_cmd = (g_square_turn_left_done != 0U) ? 0.0f :
                          ((float) g_square_turn_direction * turn_reference);
        g_line_right_cmd = (g_square_turn_right_done != 0U) ? 0.0f :
                           (-(float) g_square_turn_direction * turn_reference);
        g_square_turn_loop_count++;

        if (App_SquareConfirmSpeedFault() != 0U) {
            App_SquareEnterFault(APP_SQUARE_FAULT_SPEED);
        } else if (g_square_turn_loop_count >= APP_SQUARE_TURN_TIMEOUT_LOOPS) {
            g_square_turn_timeout_count++;
            App_SquareEnterFault(APP_SQUARE_FAULT_TURN_TIMEOUT);
        }
        Bsp_Gpio_ToggleHeartbeat();
        return;
    }

    if (g_square_route_state == APP_SQUARE_STATE_BRAKE_AFTER_TURN) {
        if (g_square_state_elapsed_ms >= APP_SQUARE_BRAKE_AFTER_TURN_MS) {
            g_square_completed_side_count++;
            g_square_side_index++;
            if (g_square_side_index >= 4U) {
                g_square_side_index = 0U;
                g_square_lap_count++;
            }
            App_SquareEnterWaitLine();
        }
        return;
    }

    if (g_square_route_state == APP_SQUARE_STATE_FAULT) {
        return;
    }

    App_SquareEnterFault(APP_SQUARE_FAULT_INTERNAL_STATE);
}

static void App_ReadCcdLineObservation(App_LineObservation *observation)
{
    Bsp_Ccd_ReadFrame();
    Bsp_Ccd_Process();
    App_UpdateLineSensorWatch();

    observation->source = APP_LINE_SOURCE_CCD;
    observation->fresh = 1U;
    observation->new_frame = 1U;
    observation->valid = Bsp_Ccd_IsLineValid();
    observation->target = observation->valid ? Bsp_Ccd_GetTargetIndex() : -1;
    observation->error = observation->valid ? Bsp_Ccd_GetLineError() : 0;
    observation->angle_d10 = 0;
    observation->quality = observation->valid ? 100U : 0U;
    observation->direction_mask = observation->valid ?
                                  K230_LINE_DIRECTION_FRONT : 0U;
}

static void App_UpdateK230JunctionState(const App_LineObservation *observation)
{
    uint8_t side_mask;
    uint32_t now_ms;

    if ((observation->new_frame == 0U) || (observation->valid == 0U)) {
        return;
    }

    now_ms = Bsp_Time_GetMilliseconds();
    side_mask = observation->direction_mask &
                (K230_LINE_DIRECTION_LEFT | K230_LINE_DIRECTION_RIGHT);

    if (side_mask != 0U) {
        if (g_k230_junction_confirm_count < APP_K230_LINE_JUNCTION_CONFIRM_FRAMES) {
            g_k230_junction_confirm_count++;
        }
        g_k230_junction_clear_count = 0U;
        g_k230_junction_last_side_ms = now_ms;
        if (g_k230_junction_confirm_count >= APP_K230_LINE_JUNCTION_CONFIRM_FRAMES) {
            g_k230_junction_active = 1U;
        }
        return;
    }

    g_k230_junction_confirm_count = 0U;
    if (observation->direction_mask == K230_LINE_DIRECTION_FRONT) {
        if (g_k230_junction_clear_count < APP_K230_LINE_JUNCTION_CLEAR_FRAMES) {
            g_k230_junction_clear_count++;
        }
        if ((g_k230_junction_active != 0U) &&
            (g_k230_junction_clear_count >= APP_K230_LINE_JUNCTION_CLEAR_FRAMES) &&
            ((uint32_t) (now_ms - g_k230_junction_last_side_ms) >=
             APP_K230_LINE_JUNCTION_HOLD_MS)) {
            g_k230_junction_active = 0U;
        }
    } else {
        g_k230_junction_clear_count = 0U;
    }
}

static void App_ReadK230LineObservation(App_LineObservation *observation)
{
    K230_LineFrame frame;

    observation->source = APP_LINE_SOURCE_K230;
    observation->fresh = Protocol_K230_IsLineFresh();
    observation->new_frame = Protocol_K230_TakeLatestLineFrame(&frame);

    if (observation->new_frame != 0U) {
        g_k230_cached_line_frame = frame;
        g_k230_has_cached_line_frame = 1U;
    }

    g_k230_line_age_ms = Protocol_K230_GetLineAgeMs();
    if (g_k230_has_cached_line_frame == 0U) {
        observation->valid = 0U;
        observation->target = -1;
        observation->error = 0;
        observation->angle_d10 = 0;
        observation->quality = 0U;
        observation->direction_mask = 0U;
        return;
    }

    observation->error = g_k230_cached_line_frame.error_x;
    observation->angle_d10 = g_k230_cached_line_frame.angle_d10;
    observation->quality = g_k230_cached_line_frame.quality;
    observation->direction_mask = g_k230_cached_line_frame.direction_mask;
    observation->target = (int16_t) (160 + observation->error);
    observation->valid = ((observation->fresh != 0U) &&
                          (g_k230_cached_line_frame.valid != 0U)) ? 1U : 0U;

    g_k230_line_angle_d10 = observation->angle_d10;
    g_k230_line_quality = observation->quality;
    g_k230_line_direction_mask = observation->direction_mask;
}

static void App_PrepareK230LineObservation(App_LineObservation *observation)
{
    App_ReadK230LineObservation(observation);
    if (observation->quality < g_k230_line_follow_profile.min_quality) {
        observation->valid = 0U;
    }
    App_UpdateK230JunctionState(observation);
}

static void App_UpdateDeliveryWatch(void)
{
    g_delivery_state = g_delivery_task.state;
    g_delivery_target_digit = g_delivery_task.target_digit;
    g_delivery_target_locked = g_delivery_task.target_locked;
    g_delivery_route_region = g_delivery_task.route_region;
    g_delivery_junction_id = g_delivery_task.junction_id;
    g_delivery_pending_decision = (uint8_t) g_delivery_task.pending_decision;
    g_delivery_visual_mode = g_delivery_task.visual_mode;
    g_delivery_visual_command_pending = g_delivery_task.visual_command_pending;
    g_delivery_fault = ((g_delivery_task.state == DELIVERY_STATE_FAULT) ||
                        (g_delivery_task.planner.fault != 0U) ||
                        (g_delivery_maneuver.state == DELIVERY_MANEUVER_STATE_FAULT)) ? 1U : 0U;
}

/* 将本周期红线观测和数字邮箱合并为一次送药任务输入，避免重复消费 L 邮箱。 */
static void App_DeliveryTaskUpdate(const App_LineObservation *line_observation)
{
    DeliveryTask_Input input = {0};
    K230_DigitFrame digit_frame;

    if (g_delivery_task.state == DELIVERY_STATE_IDLE) {
        App_UpdateDeliveryWatch();
        return;
    }

    if (line_observation != NULL) {
        input.line_fresh = line_observation->fresh;
        input.line_valid = line_observation->valid;
        input.junction_active = g_k230_junction_active;
        input.direction_mask = line_observation->direction_mask;
    }
    input.digit_fresh = Protocol_K230_IsDigitFresh();
    input.digit_new = Protocol_K230_TakeLatestDigitFrame(&digit_frame);
    if (input.digit_new != 0U) {
        input.digit = digit_frame;
    }

    DeliveryTask_Update(&g_delivery_task, &input);
    App_UpdateDeliveryWatch();
}

static void App_LineStopForState(uint8_t state)
{
    Bsp_Motor_SpeedPidStop();
    App_UpdateSpeedWatch();
    g_line_control_state = state;
    g_line_valid = 0U;
    g_line_target = -1;
    g_line_error_delta = 0;
    g_line_correction = 0.0f;
    g_line_left_cmd = 0.0f;
    g_line_right_cmd = 0.0f;
    g_line_last_correction = 0.0f;
}

static void App_ApplyFollowObservation(
    const App_FollowProfile *profile,
    float output_scale,
    uint8_t source,
    const App_LineObservation *observation)
{
    int16_t error_delta;
    int16_t abs_error;
    int16_t abs_angle_d10;
    float base_speed;
    float steer_limit;
    float min_steer;
    float correction;
    float left_cmd;
    float right_cmd;

    if (observation->valid != 0U) {
        error_delta = observation->new_frame ?
                      (int16_t) (observation->error - g_line_last_error) : 0;
        abs_error = (observation->error < 0) ?
                    (int16_t) (-observation->error) : observation->error;
        abs_angle_d10 = (observation->angle_d10 < 0) ?
                        (int16_t) (-observation->angle_d10) : observation->angle_d10;
        base_speed = profile->base_speed;
        steer_limit = profile->steer_limit;
        min_steer = 0.0f;
        correction = profile->steer_sign *
                     ((profile->kp * (float) observation->error) +
                      (profile->kd * (float) error_delta) +
                      (profile->angle_kp * ((float) observation->angle_d10 / 10.0f)));

        if ((abs_error <= profile->deadband_error) &&
            (abs_angle_d10 <= profile->angle_deadband_d10)) {
            correction = 0.0f;
            error_delta = 0;
        } else if (abs_error >= profile->large_error) {
            base_speed *= profile->large_speed_scale;
            min_steer = profile->large_min_steer;
        } else if (abs_error >= profile->medium_error) {
            base_speed *= profile->medium_speed_scale;
            steer_limit *= 0.75f;
            min_steer = profile->medium_min_steer;
        } else {
            steer_limit *= 0.45f;
        }

        if ((source == APP_LINE_SOURCE_K230) &&
            (g_k230_junction_active != 0U)) {
            base_speed *= APP_K230_LINE_JUNCTION_SPEED_SCALE;
        }

        if (correction > 0.0f) {
            g_line_recover_direction = 1;
        } else if (correction < 0.0f) {
            g_line_recover_direction = -1;
        }

        if ((min_steer > 0.0f) &&
            (correction != 0.0f) &&
            (correction < min_steer) &&
            (correction > -min_steer)) {
            correction = (correction > 0.0f) ? min_steer : -min_steer;
        }

        correction = App_LimitFloat(correction, steer_limit);
        correction = App_LimitFloatDelta(
            correction, g_line_last_correction, APP_LINE_CORRECTION_SLEW_LIMIT);
        left_cmd = (base_speed + correction) * output_scale;
        right_cmd = (base_speed - correction) * output_scale;
        App_LineApplySpeedPid(left_cmd, right_cmd);

        g_line_control_state = ((source == APP_LINE_SOURCE_K230) &&
                                (g_k230_junction_active != 0U)) ?
                               APP_LINE_CONTROL_JUNCTION_SLOW :
                               APP_LINE_CONTROL_TRACKING;
        g_line_valid = 1U;
        g_line_lost_count = 0U;
        g_line_target = observation->target;
        g_line_error = observation->error;
        g_line_error_delta = error_delta;
        g_line_correction = correction * output_scale;
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        if (observation->new_frame != 0U) {
            g_line_last_error = observation->error;
        }
        g_line_last_correction = correction;
    } else {
        correction = 0.0f;
        left_cmd = 0.0f;
        right_cmd = 0.0f;

        if ((g_line_lost_count < profile->lost_recover_max) &&
            (g_line_recover_direction != 0)) {
            base_speed = profile->base_speed * profile->lost_speed_scale;
            correction = profile->steer_sign * profile->lost_turn *
                         (float) g_line_recover_direction;
            correction = App_LimitFloatDelta(
                correction, g_line_last_correction, APP_LINE_CORRECTION_SLEW_LIMIT);
            left_cmd = (base_speed + correction) * output_scale;
            right_cmd = (base_speed - correction) * output_scale;
            App_LineApplySpeedPid(left_cmd, right_cmd);
            g_line_control_state = APP_LINE_CONTROL_LOST_SEARCH;
        } else {
            App_LineStopForState(APP_LINE_CONTROL_WAIT_FRAME);
        }

        g_line_valid = 0U;
        if (g_line_lost_count < 0xFFFFU) {
            g_line_lost_count++;
        }
        g_line_target = -1;
        g_line_error = g_line_last_error;
        g_line_error_delta = 0;
        g_line_correction = correction * output_scale;
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        g_line_last_correction = correction;
    }
}

static uint8_t App_K230ObservationReadyForFollow(
    const App_LineObservation *observation)
{
    if (observation->fresh == 0U) {
        if (g_k230_line_timeout_latched == 0U) {
            g_k230_line_timeout_latched = 1U;
            g_k230_line_timeout_stop_count++;
        }
        g_k230_line_recovery_count = 0U;
        g_k230_junction_active = 0U;
        App_LineStopForState(APP_LINE_CONTROL_TIMEOUT_STOP);
        return 0U;
    }

    if (g_k230_line_timeout_latched == 0U) {
        return 1U;
    }
    if (observation->new_frame != 0U) {
        if (observation->valid != 0U) {
            if (g_k230_line_recovery_count < APP_K230_LINE_RECOVERY_FRAMES) {
                g_k230_line_recovery_count++;
            }
        } else {
            g_k230_line_recovery_count = 0U;
        }
    }

    if (g_k230_line_recovery_count < APP_K230_LINE_RECOVERY_FRAMES) {
        App_LineStopForState(APP_LINE_CONTROL_TIMEOUT_STOP);
        return 0U;
    }

    g_k230_line_timeout_latched = 0U;
    g_k230_line_recovery_count = 0U;
    g_k230_line_recovery_total++;
    g_line_lost_count = 0U;
    g_line_last_error = observation->error;
    g_line_last_correction = 0.0f;
    Bsp_Motor_SpeedPidReset();
    return 1U;
}

/*
 * 通用 20 ms 巡线任务：CCD 每周期产生新观测；K230 以独立频率发布观测，
 * 速度环在 freshness 窗口内复用最近值，但微分项只对新视觉帧计算。
 */
static uint8_t App_FollowTask(
    const App_FollowProfile *profile,
    float output_scale,
    uint8_t source)
{
    App_LineObservation observation = {0};

    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return 0U;
    }

    g_line_source = source;
    if (source == APP_LINE_SOURCE_K230) {
        App_PrepareK230LineObservation(&observation);

        if (App_K230ObservationReadyForFollow(&observation) == 0U) {
            Bsp_Gpio_ToggleHeartbeat();
            return 1U;
        }
    } else {
        App_ReadCcdLineObservation(&observation);
    }

    App_ApplyFollowObservation(profile, output_scale, source, &observation);
    Bsp_Gpio_ToggleHeartbeat();
    return 1U;
}

static float App_EncoderCountToCm(int32_t count)
{
    uint32_t magnitude;

    if (count >= 0) {
        return Bsp_Motor_EncoderTicksToCm((uint32_t) count);
    }
    /* 先加一再取反，避免 INT32_MIN 直接取负产生有符号溢出。 */
    magnitude = (uint32_t) (-(count + 1)) + 1U;
    return -Bsp_Motor_EncoderTicksToCm(magnitude);
}

static void App_FillDeliveryManeuverInput(
    const App_LineObservation *observation,
    DeliveryManeuver_Input *input)
{
    Bsp_Motor_EncoderSnapshot snapshot;

    Bsp_Motor_GetEncoderSnapshot(&snapshot);
    input->now_ms = Bsp_Time_GetMilliseconds();
    input->left_position_cm = App_EncoderCountToCm(snapshot.left_count);
    input->right_position_cm = App_EncoderCountToCm(snapshot.right_count);
    input->line_fresh = observation->fresh;
    input->line_new = observation->new_frame;
    input->line_valid = observation->valid;
    input->line_age_ms = g_k230_line_age_ms;
    input->line_error_px = observation->error;
    input->line_angle_d10 = observation->angle_d10;
    input->line_quality = observation->quality;
    input->direction_mask = observation->direction_mask;
    input->junction_active = g_k230_junction_active;
    input->imu_fresh = Bsp_Imu_IsHeadingFresh(APP_IMU_HEADING_MAX_AGE_MS);
    input->heading_deg = Bsp_Imu_GetHeadingDeg();
    input->speed_faults = Bsp_Motor_GetSpeedFaults();
}

static void App_UpdateDeliveryManeuverWatch(
    const DeliveryManeuver_Output *output)
{
    g_delivery_maneuver_state = (uint8_t) g_delivery_maneuver.state;
    g_delivery_maneuver_turn_phase = (uint8_t) g_delivery_maneuver.turn_phase;
    g_delivery_maneuver_fault = (uint8_t) g_delivery_maneuver.fault;
    g_delivery_maneuver_heading_target_deg =
        g_delivery_maneuver.target_heading_deg;
    g_delivery_maneuver_left_target_cm_s = output->left_target_cm_s;
    g_delivery_maneuver_right_target_cm_s = output->right_target_cm_s;
}

static void App_ResetDeliveryManeuverWatch(void)
{
    g_delivery_maneuver_state = DELIVERY_MANEUVER_STATE_IDLE;
    g_delivery_maneuver_turn_phase = DELIVERY_MANEUVER_TURN_SETTLE;
    g_delivery_maneuver_fault = DELIVERY_MANEUVER_FAULT_NONE;
    g_delivery_maneuver_heading_target_deg = 0.0f;
    g_delivery_maneuver_left_target_cm_s = 0.0f;
    g_delivery_maneuver_right_target_cm_s = 0.0f;
}

static void App_ApplyDeliveryManeuverOutput(
    const DeliveryManeuver_Output *output,
    const App_LineObservation *observation)
{
    if (output->command == DELIVERY_MANEUVER_COMMAND_FOLLOW_LINE) {
        App_ApplyFollowObservation(
            &g_k230_line_follow_profile,
            output->follow_scale,
            APP_LINE_SOURCE_K230,
            observation);
        return;
    }
    if (output->command == DELIVERY_MANEUVER_COMMAND_WHEEL_SPEED) {
        Bsp_Motor_SetSpeedTargets(
            output->left_target_cm_s,
            output->right_target_cm_s);
        Bsp_Motor_SpeedPidUpdate();
        App_UpdateSpeedWatch();
        g_line_control_state =
            (g_delivery_maneuver.state == DELIVERY_MANEUVER_STATE_REACQUIRE) ?
                APP_LINE_CONTROL_MANEUVER_REACQUIRE :
                APP_LINE_CONTROL_MANEUVER_TURN;
        g_line_valid = observation->valid;
        g_line_target = observation->target;
        g_line_error = observation->error;
        g_line_error_delta = 0;
        g_line_correction = 0.0f;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
        return;
    }
    App_LineStopForState(APP_LINE_CONTROL_WAIT_FRAME);
}

/*
 * 模式 7 的统一控制点：每周期只消费一次 L 帧，先完成路线决策，再由巡线或
 * 路口动作状态机取得唯一的电机控制权，避免 HOLD 后仍多走一个控制周期。
 */
static void App_DeliveryLineTask(void)
{
    App_LineObservation observation = {0};
    DeliveryManeuver_Input input = {0};
    DeliveryManeuver_Output output = {0};
    uint8_t request_result;

    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    g_line_source = APP_LINE_SOURCE_K230;
    App_PrepareK230LineObservation(&observation);
    App_DeliveryTaskUpdate(&observation);
    App_FillDeliveryManeuverInput(&observation, &input);

    if ((g_delivery_task.state == DELIVERY_STATE_HOLD) &&
        (g_delivery_maneuver.state == DELIVERY_MANEUVER_STATE_IDLE)) {
        if (DeliveryManeuver_Start(
                &g_delivery_maneuver,
                g_delivery_task.pending_decision,
                &input) == 0U) {
            DeliveryTask_FailPendingDecision(&g_delivery_task);
        }
    }

    if (DeliveryManeuver_IsActive(&g_delivery_maneuver) != 0U) {
        DeliveryManeuver_Update(&g_delivery_maneuver, &input, &output);

        if (output.request_yaw_zero != 0U) {
            request_result = (input.imu_fresh != 0U) ?
                Bsp_Imu_ZeroYaw() : 0U;
            DeliveryManeuver_ReportYawZero(
                &g_delivery_maneuver,
                request_result);
        }
        if (output.request_commit != 0U) {
            request_result =
                DeliveryTask_CommitPendingDecision(&g_delivery_task);
            DeliveryManeuver_ReportCommit(
                &g_delivery_maneuver,
                request_result);
        }
        if (g_delivery_maneuver.state == DELIVERY_MANEUVER_STATE_FAULT) {
            DeliveryTask_FailPendingDecision(&g_delivery_task);
        }

        App_ApplyDeliveryManeuverOutput(&output, &observation);
        App_UpdateDeliveryWatch();
        App_UpdateDeliveryManeuverWatch(&output);
        Bsp_Gpio_ToggleHeartbeat();
        return;
    }

    App_UpdateDeliveryManeuverWatch(&output);
    if ((g_delivery_task.state == DELIVERY_STATE_IDLE) ||
        (DeliveryTask_IsMotionAllowed(&g_delivery_task) != 0U)) {
        if (App_K230ObservationReadyForFollow(&observation) != 0U) {
            App_ApplyFollowObservation(
                &g_k230_line_follow_profile,
                1.0f,
                APP_LINE_SOURCE_K230,
                &observation);
        }
    } else {
        /* IDENTIFY、TARGET_LOCKED、DECIDE 和 FAULT 均停车等待下一步事件。 */
        App_LineStopForState(APP_LINE_CONTROL_WAIT_FRAME);
    }
    Bsp_Gpio_ToggleHeartbeat();
}

/* 基础联调模式：CCD 有效时以固定比例等速直行，无有效线立即主动停车。 */
static void App_CcdStraightTask(void)
{
    float heading_error;
    float correction;
    float left_cmd;
    float right_cmd;

    if (App_TimeElapsed(&g_mode_last_task_ms, APP_LOOP_FAST_MS) == 0U) {
        return;
    }

    Bsp_Ccd_ReadFrame();
    Bsp_Ccd_Process();
    if ((Bsp_Ccd_IsLineValid() != 0U) &&
        (Bsp_Imu_IsHeadingFresh(APP_IMU_HEADING_MAX_AGE_MS) != 0U)) {
        if (g_imu_heading_hold_active == 0U) {
            g_imu_heading_hold_active = Bsp_Imu_ZeroYaw();
            g_imu_heading_last_error_deg = 0.0f;
            g_imu_heading_last_correction = 0.0f;
        }

        heading_error = g_imu_heading_target_deg - Bsp_Imu_GetHeadingDeg();
        if ((heading_error > -APP_IMU_HEADING_DEADBAND_DEG) &&
            (heading_error < APP_IMU_HEADING_DEADBAND_DEG)) {
            heading_error = 0.0f;
        }
        correction = APP_IMU_HEADING_KP * heading_error +
            APP_IMU_HEADING_KD *
                (heading_error - g_imu_heading_last_error_deg);
        correction = App_LimitFloat(correction,
            APP_IMU_HEADING_CORRECTION_LIMIT);
        correction = App_LimitFloatDelta(correction,
            g_imu_heading_last_correction,
            APP_IMU_HEADING_CORRECTION_SLEW);

        /* 右偏时 heading 为负，左轮减速、右轮加速，使车辆向左纠偏。 */
        left_cmd = APP_CCD_STRAIGHT_SPEED - correction;
        right_cmd = APP_CCD_STRAIGHT_SPEED + correction;
        Bsp_Motor_Set(left_cmd, right_cmd);
        g_line_valid = 1U;
        g_line_target = Bsp_Ccd_GetTargetIndex();
        g_line_error = Bsp_Ccd_GetLineError();
        g_line_left_cmd = left_cmd;
        g_line_right_cmd = right_cmd;
        g_line_correction = correction;
        g_imu_heading_error_deg = heading_error;
        g_imu_heading_correction = correction;
        g_imu_heading_left_cmd = left_cmd;
        g_imu_heading_right_cmd = right_cmd;
        g_imu_heading_last_error_deg = heading_error;
        g_imu_heading_last_correction = correction;
    } else {
        Bsp_Motor_Stop();
        if (Bsp_Imu_IsHeadingFresh(APP_IMU_HEADING_MAX_AGE_MS) == 0U) {
            g_imu_heading_hold_active = 0U;
            g_imu_heading_hold_stale_count++;
        }
        g_line_valid = 0U;
        g_line_target = -1;
        g_line_error = 0;
        g_line_left_cmd = 0.0f;
        g_line_right_cmd = 0.0f;
        g_line_correction = 0.0f;
        g_imu_heading_error_deg = 0.0f;
        g_imu_heading_correction = 0.0f;
        g_imu_heading_left_cmd = 0.0f;
        g_imu_heading_right_cmd = 0.0f;
        g_imu_heading_last_error_deg = 0.0f;
        g_imu_heading_last_correction = 0.0f;
    }
    g_line_dx_max = Bsp_Ccd_GetDxMax();
    g_line_dx_min = Bsp_Ccd_GetDxMin();
    g_line_dx_max_index = Bsp_Ccd_GetDxMaxIndex();
    g_line_dx_min_index = Bsp_Ccd_GetDxMinIndex();
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
        if (mode == APP_MODE_LINE_FOLLOW) {
            /* 先取消路口动作，再关闭 K230 数字视觉，避免模式退出后残留转向目标。 */
            DeliveryManeuver_Reset(&g_delivery_maneuver);
            App_ResetDeliveryManeuverWatch();
            DeliveryTask_Reset(&g_delivery_task);
            g_delivery_start_pending = 0U;
        }
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
            App_EnsureSpeedPid();
            Bsp_Motor_SpeedPidReset();
            Bsp_Motor_EncoderReset();
            App_ResetLineState();
            DeliveryManeuver_Reset(&g_delivery_maneuver);
            App_ResetDeliveryManeuverWatch();
            g_line_source = APP_LINE_SOURCE_K230;
            if (g_delivery_start_pending == 0U) {
                DeliveryTask_Init(&g_delivery_task);
            } else {
                g_delivery_start_pending = 0U;
            }
            /* 首次进入必须等待连续有效红线帧，禁止消费旧缓存后立即启动。 */
            g_k230_line_timeout_latched = 1U;
            return 1U;
        case APP_MODE_CCD_STRAIGHT:
            App_EnsureCcd();
            Bsp_Ccd_ResetState(); // 直行判断必须从新帧开始，不能沿用最近有效黑线。
            App_ResetLineState();
            App_ResetImuHeadingHold();
            if (Bsp_Imu_IsHeadingFresh(APP_IMU_HEADING_MAX_AGE_MS) != 0U) {
                g_imu_heading_hold_active = Bsp_Imu_ZeroYaw();
            }
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
            App_EnsureSpeedPid();
            Bsp_Motor_EncoderReset();
            App_ResetSquareState();
            App_SquareEnterWaitLine();
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
            /* 协议已在公共主循环轮询；本模式只用于观察链路 Watch 状态。 */
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
            App_DeliveryLineTask();
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
            /* 圆形旧模式继续使用 CCD，便于与 K230 比赛巡线分开标定。 */
            (void) App_FollowTask(
                &g_circle_follow_profile, 1.0f, APP_LINE_SOURCE_CCD);
            break;
        case APP_MODE_K230_FOLLOW:
            /* 解析 K230 目标并增量更新双轴云台，链路超时后禁用并在新帧到达时恢复 PWM。 */
            App_K230Follow_Task();
            break;
        case APP_MODE_SQUARE_FOLLOW:
            /* CCD 定距直线 + 编码器 90°差动转向，完成四边后继续下一圈。 */
            App_SquareRouteTask();
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

/*
 * CCS Watch one-shot command mailbox. The debugger writes a mode number and
 * this task submits it through the same guarded switch path as the touchscreen.
 */
static void App_DebugRequestTask(void)
{
    uint8_t requested_mode = g_app_debug_request_mode;

    if (requested_mode == APP_DEBUG_REQUEST_NONE) {
        return;
    }

    g_app_debug_request_mode = APP_DEBUG_REQUEST_NONE;
    g_app_debug_request_count++;
    g_app_debug_request_result = App_RequestMode(requested_mode);
}

uint8_t App_GetCurrentMode(void)
{
    return g_app_current_mode;
}

uint8_t App_DeliveryStartIdentification(void)
{
    K230_DigitFrame stale_frame;
    uint8_t request_result;

    if ((g_app_current_mode == APP_MODE_LINE_FOLLOW) &&
        (g_speed_pid_initialized != 0U)) {
        Bsp_Motor_SpeedPidStop();
    }
    DeliveryManeuver_Reset(&g_delivery_maneuver);
    App_ResetDeliveryManeuverWatch();
    g_delivery_epoch++;
    if (g_delivery_epoch == 0U) {
        g_delivery_epoch = 1U;
    }
    g_delivery_start_pending = 1U;

    if (g_app_current_mode != APP_MODE_LINE_FOLLOW) {
        request_result = App_RequestMode(APP_MODE_LINE_FOLLOW);
        if (request_result == 0U) {
            g_delivery_start_pending = 0U;
            return 0U;
        }
    }

    (void) Protocol_K230_TakeLatestDigitFrame(&stale_frame);
    return DeliveryTask_StartIdentification(&g_delivery_task, g_delivery_epoch);
}

uint8_t App_DeliveryStartRoute(void)
{
    if ((g_app_current_mode != APP_MODE_LINE_FOLLOW) ||
        (DeliveryManeuver_IsActive(&g_delivery_maneuver) != 0U)) {
        return 0U;
    }
    return DeliveryTask_StartRoute(&g_delivery_task);
}

uint8_t App_DeliveryReset(void)
{
    if ((g_app_current_mode == APP_MODE_LINE_FOLLOW) &&
        (g_speed_pid_initialized != 0U)) {
        Bsp_Motor_SpeedPidStop();
    }
    DeliveryManeuver_Reset(&g_delivery_maneuver);
    App_ResetDeliveryManeuverWatch();
    DeliveryTask_Reset(&g_delivery_task);
    g_delivery_start_pending = 0U;
    g_delivery_state = DELIVERY_STATE_IDLE;
    g_delivery_target_digit = 0U;
    g_delivery_target_locked = 0U;
    g_delivery_route_region = K230_ROUTE_REGION_PHARMACY;
    g_delivery_junction_id = 0U;
    g_delivery_pending_decision = ROUTE_DECISION_NONE;
    g_delivery_visual_mode = K230_VISUAL_MODE_OFF;
    g_delivery_visual_command_pending = g_delivery_task.visual_command_pending;
    g_delivery_fault = 0U;
    return App_RequestMode(APP_MODE_STOPPED);
}

uint8_t App_DeliverySetRouteRegion(uint8_t region)
{
    if (DeliveryManeuver_IsActive(&g_delivery_maneuver) != 0U) {
        return 0U;
    }
    return DeliveryTask_SetRouteRegion(&g_delivery_task, region);
}

uint8_t App_DeliveryCommitPendingDecision(void)
{
    /* 路口动作执行期间只允许状态机在制动完成后提交，禁止调试接口提前跳过动作。 */
    if (DeliveryManeuver_IsActive(&g_delivery_maneuver) != 0U) {
        return 0U;
    }
    return DeliveryTask_CommitPendingDecision(&g_delivery_task);
}

/*
 * 初始化基础 BSP、统一时基和两路协议。初始化期间显式关闭电机和云台，
 * 最终把全部运行时状态复位为 STOPPED，并主动发送一帧初始状态。
 */
void App_Init(void)
{
    Bsp_Gpio_Init();
    Bsp_Uart_Init();
    Bsp_Imu_Init();
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
    g_app_debug_request_mode = APP_DEBUG_REQUEST_NONE;
    g_app_debug_request_result = 0U;
    g_app_debug_request_count = 0U;
    g_ccd_initialized = 0U;
    g_encoder_initialized = 0U;
    g_speed_pid_initialized = 0U;
    g_delivery_start_pending = 0U;
    g_delivery_epoch = 0U;
    DeliveryTask_Init(&g_delivery_task);
    DeliveryManeuver_Init(&g_delivery_maneuver);
    g_delivery_state = DELIVERY_STATE_IDLE;
    g_delivery_target_digit = 0U;
    g_delivery_target_locked = 0U;
    g_delivery_route_region = K230_ROUTE_REGION_PHARMACY;
    g_delivery_junction_id = 0U;
    g_delivery_pending_decision = ROUTE_DECISION_NONE;
    g_delivery_visual_mode = K230_VISUAL_MODE_OFF;
    g_delivery_visual_command_pending = 0U;
    g_delivery_fault = 0U;
    App_ResetDeliveryManeuverWatch();
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
    /* IMU 与 K230 始终接收，停止态也要排空 FIFO 并维护各自 freshness。 */
    Bsp_Imu_Task();
    Protocol_K230_Task();
    App_DebugRequestTask();
    Protocol_Tjc_Task();
    App_ModeManagerTask();
    if (g_app_switch_state == APP_SWITCH_IDLE) {
        App_ModeTask(g_app_current_mode);
    }
}
