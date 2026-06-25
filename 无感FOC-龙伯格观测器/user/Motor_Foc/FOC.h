#ifndef _FOC_H_
#define _FOC_H_
#include "motor.h"

typedef struct
{
    /* data */
    float kp;
    float ki;
    float kd;

    float pre;      //当前
    float tar;      //目标
    float Err;
	  float sum;
 	
    float out;
    float outMax;
}I_pid;

typedef struct
{
 

    float iq_expect;            //目标Iq
    float id_expect;            //目标Id
	  float iq_current;           //真实Iq
    float id_current;           //真实Id
    float ia;                   //a 相实际电流
    float ib;                   //b 相实际电流
	  float ic;                   //c 相实际电流
    float I_uAlpha;             //clarke变换后 * 2/3后 Iα
    float I_ubeta;              //clarke变换后 * 2/3后 Iβ
	  float Uq;                   //uq
    float Ud;                   //ud
    float uAlpha;               //park反变换后 后 Uα
    float uBeta;                //park反变换后 后 Uβ
    float RPM; 
	  float WE;
	
    float (*GetEncoderAngle_func)(ENCODER *);
     void (*GetPreCurrent_func)(float  ,float ,float  );
    void (*SvpwmGenerate_func)(float  ,float  );
}X_FOC;
extern X_FOC MOTOR_FOC;
extern I_pid  D_PID;
extern I_pid  Q_PID;

#include "main.h"
 #include "math.h"
 
 #include "Filter.h"
#include "stdlib.h" 

 #include "Luenberger.h"

 


void FOC_Control(X_FOC *FOC,ENCODER * motor_angle);
void get_current(X_FOC *FOC);
void  CLARKE_PARK(X_FOC *FOC,ENCODER *motor_angle ) ; 
float PID_i(I_pid *PID);
void InversePark( X_FOC *FOC ,ENCODER *motor_angle );
void SVPWM_Generate(X_FOC *FOC);
float Limit(float value,int lim);


 
 
  #endif 
