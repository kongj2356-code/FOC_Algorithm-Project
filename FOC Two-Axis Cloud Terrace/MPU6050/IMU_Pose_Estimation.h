#ifndef __IMU_POSE_ESTIMATION_H
#define __IMU_POSE_ESTIMATION_H

#include "stm32f10x.h"
#include "QMC5883P.h" 
#include "math.h"

#include "headfile.h"


typedef struct{
	float q0,q1,q2,q3;
	float exInt,eyInt,ezInt;
	float Kp,Ki;
}Mahony_TypeDef;



#define ACC_LSB         16384   
#define GYRO_LSB        131    
#define DEG2RAD         0.017453292519943295f
#define GYRO_INIT_COUNT 1000

static const float BIAS_ALPHA=0.999f;
static const float GYRO_STABLE_TH=0.05f;   
static const float ACC_NORM_TH=0.2f;
static const float INIT_CALIBRATION_TIME=1.0f;
static float gyro_bias_x=0.0f;
static float gyro_bias_y=0.0f;
static float gyro_bias_z=0.0f;
static float calibration_time=0.0f;
static float gyro_bias_x_init=0;
static float gyro_bias_y_init=0;
static float gyro_bias_z_init=0;
static uint16_t bias_count=0;
static uint8_t bias_done=0;
#define GYRO_DEADZONE 0.03f  


void Mahony_Init(Mahony_TypeDef *mah);
void Gyro_InitBias(MPU6050_TypeDef *xzy);
void Gyro_Deadzone_Filter(MPU6050_TypeDef *xzy);
void GyroBias_Update(MPU6050_TypeDef *xzy,float dt);
void Mahony_Update(Mahony_TypeDef *mahony,MPU6050_TypeDef *xzy,QMC5883P_Calibration_t *qmc,float dt);

void Mahony_GetEuler(Mahony_TypeDef *mahony,float *roll,float *pitch,float *yaw,float gz_rad,float dt);
void Compass_GetYaw(MPU6050_TypeDef *mpu, QMC5883P_Calibration_t *qmc, float *yaw_deg);
void Mahony_GetData(Mahony_TypeDef *mahony,MPU6050_TypeDef *mpu, QMC5883P_Calibration_t *qmc,float *roll,float *pitch,float *yaw,float *yaw1,float dt);



#endif
