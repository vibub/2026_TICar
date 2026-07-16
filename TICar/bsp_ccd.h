/**
 * @file bsp_ccd.h
 * @brief 线阵 CCD 采集、处理和诊断数据访问接口。
 */
#ifndef BSP_CCD_H
#define BSP_CCD_H

#include <stdint.h>

#define BSP_CCD_PIXEL_COUNT 128U /* 线阵传感器一帧包含 128 个像素。 */
#define BSP_CCD_CENTER_INDEX 63  /* 沿用 2025 版巡线算法的目标中心索引。 */

/** 初始化 CCD SI/CLK 空闲电平，并使 ADC 可接受软件触发。 */
void Bsp_Ccd_Init(void);
/** 按 CCD 时序采集一帧 128 像素原始 ADC 数据。 */
void Bsp_Ccd_ReadFrame(void);
/** 对最近原始帧进行滤波、边沿配对和中心误差计算。 */
void Bsp_Ccd_Process(void);
/** 通过 K230 UART 输出检测摘要和完整原始帧；发送过程会阻塞等待 TX FIFO。 */
void Bsp_Ccd_PrintDebugFrame(void);

/*
 * 帧缓冲访问接口。返回值指向模块内部静态数组，仅供读取；下一次采集或处理会更新内容。
 */
const uint16_t *Bsp_Ccd_GetRawFrame(void);
const uint16_t *Bsp_Ccd_GetFilteredFrame(void);
/*
 * 检测结果：target 为黑线中心像素，error=target-BSP_CCD_CENTER_INDEX，正值表示黑线在右侧。
 * 无可靠目标时 valid 为 0，target 返回 -1。
 */
int16_t Bsp_Ccd_GetTargetIndex(void);
int16_t Bsp_Ccd_GetLineError(void);
uint8_t Bsp_Ccd_IsLineValid(void);

/* 亮度极值、自适应阈值、对比度和黑线区域诊断量。 */
uint16_t Bsp_Ccd_GetRawMin(void);
uint16_t Bsp_Ccd_GetRawMax(void);
uint16_t Bsp_Ccd_GetThreshold(void);
int16_t Bsp_Ccd_GetBlackLeft(void);
int16_t Bsp_Ccd_GetBlackRight(void);
uint16_t Bsp_Ccd_GetContrast(void);
uint16_t Bsp_Ccd_GetBlackWidth(void);
uint16_t Bsp_Ccd_GetRawMinIndex(void);
uint16_t Bsp_Ccd_GetRawMaxIndex(void);

/* 三像素差分中最强正/负边沿及其索引，用于观察黑线边沿配对质量。 */
int16_t Bsp_Ccd_GetDxMax(void);
int16_t Bsp_Ccd_GetDxMin(void);
uint16_t Bsp_Ccd_GetDxMaxIndex(void);
uint16_t Bsp_Ccd_GetDxMinIndex(void);

#endif
