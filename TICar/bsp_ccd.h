#ifndef BSP_CCD_H
#define BSP_CCD_H

#include <stdint.h>

#define BSP_CCD_PIXEL_COUNT 128U
#define BSP_CCD_CENTER_INDEX 63 // Match the 2025 CCD line-follow target center.

void Bsp_Ccd_Init(void);
void Bsp_Ccd_ReadFrame(void);
void Bsp_Ccd_Process(void);
void Bsp_Ccd_PrintDebugFrame(void);

const uint16_t *Bsp_Ccd_GetRawFrame(void);
const uint16_t *Bsp_Ccd_GetFilteredFrame(void);
int16_t Bsp_Ccd_GetTargetIndex(void);
int16_t Bsp_Ccd_GetLineError(void);
uint8_t Bsp_Ccd_IsLineValid(void);
uint16_t Bsp_Ccd_GetRawMin(void);
uint16_t Bsp_Ccd_GetRawMax(void);
uint16_t Bsp_Ccd_GetThreshold(void);
int16_t Bsp_Ccd_GetBlackLeft(void);
int16_t Bsp_Ccd_GetBlackRight(void);
uint16_t Bsp_Ccd_GetContrast(void);
uint16_t Bsp_Ccd_GetBlackWidth(void);
uint16_t Bsp_Ccd_GetRawMinIndex(void);
uint16_t Bsp_Ccd_GetRawMaxIndex(void);
int16_t Bsp_Ccd_GetDxMax(void);
int16_t Bsp_Ccd_GetDxMin(void);
uint16_t Bsp_Ccd_GetDxMaxIndex(void);
uint16_t Bsp_Ccd_GetDxMinIndex(void);

#endif
