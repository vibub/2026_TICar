/**
 * @file app_main.h
 * @brief 应用层初始化、轮询调度和运行时模式管理接口。
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

/**
 * 在 SYSCFG_DL_init() 完成后初始化应用层。
 * 初始化结束时电机和云台均保持关闭，并向 TJC 上报停止态。
 */
void App_Init(void);

/**
 * 执行一次非阻塞应用轮询：先处理屏幕命令，再推进模式切换，最后运行当前模式任务。
 * 主程序必须持续高频调用，不能在外层加入长时间阻塞。
 */
void App_Loop(void);

/**
 * 请求切换运行模式。
 *
 * @param mode APP_MODE_STOPPED～APP_MODE_SQUARE_FOLLOW。
 * @return 1 表示请求被接受或目标模式已在运行；0 表示非法或被停止优先策略拒绝。
 *
 * 返回 1 只代表请求进入切换流程；电机模式可能先制动 100 ms，最终结果由 TJC 响应帧上报。
 */
uint8_t App_RequestMode(uint8_t mode);

/**
 * @return 当前已经完成进入并正在生效的模式，不返回尚在等待的请求模式。
 */
uint8_t App_GetCurrentMode(void);

#endif
