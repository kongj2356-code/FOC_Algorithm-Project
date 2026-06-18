#ifndef _headfile_h
#define _headfile_h

#include "stm32f10x.h"                 

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "math.h"

#include "delay.h"
#include "bsp_tim.h"
#include "Serial.h"
#include "sys.h"

#include "OLED.h"
#include "MPU6050.h"
#include "IMU_Pose_Estimation.h"
#include "QMC5883P.h" 
#include "IMU_I2C.h"

#include "Serial.h"
#include "stdio.h"
#include "string.h"
#include "LED.h"

/*******************************/
#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "Delay.h"
#include "./FOC/bsp_foc.h"
#include "./ADC/bsp_adc.h"
#include "bsp_tim.h"
#include "IQ_math.h"

/*******************************/

extern float pitch;
extern float yaw;
extern float roll;
extern float pitch_acc;
extern float roll_acc;

#endif
