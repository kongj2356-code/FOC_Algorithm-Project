#ifndef _MOTOR_H  
#define _MOTOR_H  
 typedef struct
{
   short first_encoder;
   short encoder;
	 short CNT;
	 float sin_a;
	 float cos_a ;
  	float mAngle;               //机械角度
    float angle;                //角度
    float radian;               //弧度
	
}ENCODER;
extern ENCODER  motor_angle;
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "math.h"
#include "stdlib.h" 
#include "stdio.h"
#include "Filter.h"

#define _PI_2  1.57079632679f
#define _PI    3.14159265359f
#define _3PI_2 4.71238898038f
#define _2PI   6.28318530718f

#define PWM_HZ   10000
#define I_GAIN      30                   
   //增益

#define motor_L        0.00055f             //电感
#define motor_R        0.75f                //电阻
#define Banwide    _2PI*PWM_HZ/I_GAIN       //电流环带宽
#define pair           4                    //电机极对数
#define FLUX         0.00583f
#define _SQRT3 1.73205080757f
#define   PWM_DUTY  4200     
//#define  FOC_dt  (1.0f/PWM_HZ
#define  FOC_dt 0.0001f
#define  kx  ((PWM_DUTY*_SQRT3)/24.0f)

void Motor_Enable(void) ;
void Motor_Disable(void);
void FOC_Init(void);
float get_angle(ENCODER * motor_angle);
 #endif

