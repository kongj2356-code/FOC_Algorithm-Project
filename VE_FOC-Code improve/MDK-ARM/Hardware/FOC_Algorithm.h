#ifndef __FOC_ALGORITHM_H
#define __FOC_ALGORITHM_H

#include "header.h"
#include "Control.h"

//结构体命名规则--->作用_具体的部分
//定义放在.h或者.c文件中
//声明全部都放在.c文件中
//在.h文件中声明extern在别处定义

/*电机停止*/
void stop_motor(void);
/*机械零点计算*/
void Mechanical_zero_point_calibration(SVPWM_t* p1, Motor_t* p2,Encoder_t * p3);
/*反变换*/
void reverse_park(SVPWM_t *p);
void reverse_clarke(SVPWM_t *p);
/*SVPWM计算*/
void SVPWM_Calculate(SVPWM_t *p);
/*正变换*/
void park(SVPWM_t *p,PID_t* p1,PID_t* p2);//得到Id Iq
void clarke(SVPWM_t *p);//得到Ia Ib

#endif
