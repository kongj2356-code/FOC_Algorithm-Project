#ifndef _Luenberger_H  
#define _Luenberger_H  
 
  
   
 typedef struct
{
 
float Forecast_Ialpha_dt;  //预估Ialpha电流微分
float Forecast_Ibeta_dt;   //预估Ibeta电流微分
float Forecast_Ialpha;     //Ialpha电流
float Forecast_Ibeta;      //Ibeta电流
float V_Alpha;             //预估Alpha反电势
float	V_Beta;              //预估Beta反电势
float V_Alpha_dt;          //预估Alpha反电势微分
float	V_Beta_dt;           //预估Beta反电势微分
		
float	V_Alpha_LPF;          //滤波后的预估Alpha反电势
float	V_Beta_LPF;          //滤波后的预估Beta反电势
float	K1; 	
float	K2; 	
float	Err_Ialpha;
float	Err_Ibeta;
	
}LuenBerGer_Observer;
 
//Ea=-w*flux*sin; Eb=w*flux*cos
 // Ea_dt =-w*w*flux*cos
typedef struct
{
    /* data */
    float PLL_Kp;
    float PLL_Ki;
 
  
    float Err;
	  float sum;
 	
    float out;
    float outMax;
	
	  float We_fore;
	  float We_fore_lpf;
	  float Theta_fore_now;
	  float Theta_fore_Last;
	
	
	  float I_Partern;
}Luenberger_pll;
 float calculate_motor_speed(float current_angle, float dt);

 #include "FOC.h"
  void Luenberger_Observer(X_FOC *FOC,LuenBerGer_Observer*Luenberger);
extern LuenBerGer_Observer  Luenberger_member ;
extern Luenberger_pll Luenberger_PLL ;

 #endif

