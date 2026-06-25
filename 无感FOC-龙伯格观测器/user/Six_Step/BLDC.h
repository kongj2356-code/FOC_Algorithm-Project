#ifndef _bldc_H_
#define _bldc_H_
#include "main.h"
 
#include "gpio.h"

	#include "tim.h"

void  stop_motor(void);
void  start_motor(void);
void V_up_W_down(void);
void V_up_U_down(void);
void W_up_U_down(void);
void W_up_V_down(void);
void U_up_V_down(void);
void U_up_W_down(void);
void six_huan(uint8_t sta);
void  BLDCMotor_Start(void);
int16_t PID_Calculate(short Aima_v,short speed);
int16_t PID_Control(short in);
extern uint8_t	RT_hallPhase,LS_hallPhase;
void BLDCMotor_SetSpeed(int32_t speed);

uint8_t read_sta(void);
extern uint8_t stata ;
extern int Aima_v;
extern  int16_t duty;
extern uint8_t diretion;
extern uint16_t uwStep,ci;	
extern int32_t Ph[6]; // »ô¶ûÐÅºÅ
extern int32_t phase;	
extern int16_t speed;
extern uint8_t qh[2];

 
#define go       0
#define back     1

#endif 
