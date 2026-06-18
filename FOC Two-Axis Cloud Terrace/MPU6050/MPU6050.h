#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"                  // Device header
#include "IMU_I2C.h"
#include "MPU6050_Reg.h"

typedef struct{
	int16_t Acc[3];
	int16_t Gyro[3];
	float Accf[3];
	float Gyrof[3];
}MPU6050_TypeDef;


#define x 0
#define y 1
#define z 2


void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
void MPU6050_WriteRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint16_t Length);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(MPU6050_TypeDef *mpu);


#endif
