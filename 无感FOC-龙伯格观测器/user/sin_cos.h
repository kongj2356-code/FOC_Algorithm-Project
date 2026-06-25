#ifndef _sin_cos_H_
#define _sin_cos_H_
#include "main.h"
 
#include "gpio.h"
 
#include "tim.h"

//#define _PI_2  1.57079632679f
//#define _PI    3.14159265359f
//#define _3PI_2 4.71238898038f
//#define _2PI   6.28318530718f

//#define PWM_HZ   10000
//#define kbb      30

//#define LLL        0.00055f
//#define RRR        0.75f
//#define Banwide    _2PI*PWM_HZ/kbb
//  

//#define _round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
//  void  CLARKE_PARK(float ag,float Iaa,float Ibb);


//#define _SQRT3 1.73205080757f
//#define   PWM_DUTY  4200     
// 
//float fast_cos(int16_t angle);
//float fast_sin(int16_t angle);
//void   SVPWM_Generate(void);
// extern  int la ,lb ,lc ;
//extern float uAlpha, uBeta,V_Alpha,V_Beta,buchan,Ea,Ebfloat,V_Alpha_LPF,V_Beta_LPF,hmax;
//extern short JIXIE,XUNI;

//  void InversePark( float ud, float uq );
//extern float jixie_angle ,dianqi_angle ,fore_speed,We_fore_lpf;
//extern uint16_t channel1, channel2, channel3,channel4;
//extern float uAlpha, uBeta;
//extern float ud ,uq ;
//extern int TS ;
//extern float kx ;
//extern float _fast_cos[91] ;
//extern  const int16_t hSin_Cos_Table[256] ;
//extern float _fast_sin[91] ; 
//extern short first_encoder,current_encoder ;
//extern float ALPHA;
//extern short jishu   ;
//extern float Now_Forecast_Ialpha,Now_Forecast_Ibeta,New_Forecast_Ialpha,New_Forecast_Ibeta;
//extern float Theta_fore_Last , Theta_fore_now ,We_fore ;
//float get_angle(void);
// float extended_atan(float x,float y) ;
// float lowPassFilterA(float  input) ;
//  float lowPassFilterB(float input) ;
// extern float sin_a ,cos_a ;
// extern float PLL_Kp , PLL_Ki   ;

//void get_set(void);
//void Flux_Observer(void);

//   void adc_offset(void); 

//extern uint16_t Dma_Value[3]  ;
//extern uint32_t offsetValue[3];
//extern int la,lb,lc;
//extern float I_uAlpha,I_ubeta,Iaa,Ibb,id_current,iq_current,id_expect,iq_expect,flux,gama,TTSS;
// float PID_id( void);
// float PID_iq(float  iq_expect  );
//   float PID_speed(float target,float now);
//float PID_position(float target,float now);

//void get_current(void);  
//extern uint8_t foc_flag ;
//extern float speed_now ,speed_target,speed_last ;
//short lowPassSpeed(short input);
// extern float position_now ,position_last ,position_target,Kd_position;
//extern short round_z ;
//void position_angle(short encoder);

// float Vb_LPF_Filter(float data);
//float Va_LPF_Filter(float data);
//void SMO_Observer(void);
//void UploadData_vofa(float *data, uint8_t channel_count) ;

#endif 
