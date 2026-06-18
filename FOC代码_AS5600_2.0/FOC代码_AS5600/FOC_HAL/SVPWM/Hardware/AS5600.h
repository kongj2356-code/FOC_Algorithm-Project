#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f1xx_hal.h"  // 引入HAL库核心头文件

void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t AS5600_ReadReg(uint8_t RegAddress);
uint8_t AS5600_ReadNowReg(void);
void AS5600_Init(void);
float GetAngle(void);
float GetAngle_NoTrack(void);
float GetVelocity(void);

#endif
