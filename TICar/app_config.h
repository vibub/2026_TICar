/**
 * @file app_config.h
 * @brief 定义运行模式编号；编号同时作为 TJC 固定请求帧中的命令字段。
 *
 * 模式 1～13 为可运行功能，模式 0 为安全停止态。修改编号会改变屏幕协议，禁止随意重排。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 模式号 0：立即保持电机和云台关闭，仅继续处理协议。 */
#define APP_MODE_STOPPED 0
/* 模式号 1：板载 LED 心跳，用于确认主循环和毫秒时基工作。 */
#define APP_MODE_HEARTBEAT 1
/* 模式号 2：只运行 K230 文本协议接收与应答。 */
#define APP_MODE_K230_UART 2
/* 模式号 3：左右轮快慢交替的 PWM 映射测试，会驱动电机。 */
#define APP_MODE_MOTOR_PWM 3
/* 模式号 4：采集 CCD，并通过 K230 UART 输出像素调试帧。 */
#define APP_MODE_CCD_ADC   4
/* 模式号 5：周期发送计数报文，用于 UART 通道联调。 */
#define APP_MODE_UART_TEST 5
/* 模式号 6：采集 CCD 并镜像到 CCS Watch 和 RAM 环形日志。 */
#define APP_MODE_CCD_WATCH 6
/* 模式号 7：K230 红线巡线，使用误差/方向角和左右轮速度闭环。 */
#define APP_MODE_LINE_FOLLOW 7
/* 模式号 8：有效黑线作为运行门控，使用 IMU 相对航向闭环直行；丢线或姿态超时立即停车。 */
#define APP_MODE_CCD_STRAIGHT 8
/* 模式号 9：仅采样编码器，不驱动电机。 */
#define APP_MODE_ENCODER_WATCH 9
/* 模式号 10：自动循环执行 0、20、30、20 cm/s 速度阶跃。 */
#define APP_MODE_SPEED_TEST 10
/* 模式号 11：圆形赛道 CCD 巡线，保留独立参数入口。 */
#define APP_MODE_CIRCLE_FOLLOW 11
/* 模式号 12：根据 K230 目标误差增量控制双轴云台。 */
#define APP_MODE_K230_FOLLOW 12
/* 模式号 13：CCD 修正直线方向，编码器控制固定边长和 90°差动转向并持续循环。 */
#define APP_MODE_SQUARE_FOLLOW 13

/* 巡线数据源编号：普通模式使用 K230，圆形和方形旧模式继续显式使用 CCD。 */
#define APP_LINE_SOURCE_CCD  0U
#define APP_LINE_SOURCE_K230 1U
#define APP_LINE_SOURCE_DEFAULT APP_LINE_SOURCE_K230

/*
 * 兼容旧版编译期模式配置，仅供历史代码和调试记录参考。
 * 当前固件不会用该宏裁剪功能，上电实际模式固定为 APP_MODE_STOPPED。
 */
#define APP_MODE APP_MODE_K230_UART

#endif
