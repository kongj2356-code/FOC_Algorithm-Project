#include "FOC.h"
#include "adc.h"		 
#include "tim.h"
#include "usart.h"
 
uint8_t sectionMap[7] = {0, 2, 6, 1, 4, 3, 5};
uint8_t num=0;
 uint16_t channel1=0, channel2=0, channel3=0,channel4=10;

 I_pid  Q_PID={
       /* data */
     .kp= motor_L*Banwide,
     .ki= motor_R*Banwide/PWM_HZ,
     .kd=0,

     .pre=0,      //当前
     .tar=0,      //目标
     .Err=0,
	   .sum=0,
 	
     .out=0,
     .outMax=13.9f,
};
I_pid  D_PID={
       /* data */
     .kp= motor_L*Banwide,
     .ki= motor_R*Banwide/PWM_HZ,
     .kd=0,

     .pre=0,      //当前
     .tar=0,      //目标
     .Err=0,
	   .sum=0,
 	
     .out=0,
     .outMax=13.9f,
};
 X_FOC MOTOR_FOC={

      .iq_expect=0,            //目标Iq
      .id_expect=0,           //目标Id
	    .iq_current=0,           //真实Iq
      .id_current=0,           //真实Id
      .ia=0,                   //a 相实际电流
      .ib=0,                   //b 相实际电流
	    .ic=0,                   //c 相实际电流
	    .Uq=0,
	    .Ud=0,
      .I_uAlpha=0,             //clarke变换后 * 2/3后 Iα
      .I_ubeta=0,             //clarke变换后 * 2/3后 Iβ
      .uAlpha=0,              //park反变换后 后 Uα
      .uBeta=0,               //park反变换后 后 Uβ
      .GetEncoderAngle_func=get_angle,
       
};
float Limit(float value,int lim)
{
	float tem = value;
	if(fabsf(tem)>lim)
	{
		if(tem>0)
		{	
		tem=lim;
		}
		else
		{
		tem=-lim;	
		}
	}
	return tem;
}
 
void get_current(X_FOC *FOC)
{
	   short a,b,c;
			a=-(hadc1.Instance->JDR1-2035);
	   	b=-(hadc1.Instance->JDR2-2035);
		 	c=-(hadc1.Instance->JDR3-2035);
 	 
//	switch(sectionMap[num])
//	{
//	  case 1:
//		case 6:
//			   a = 0 - b - c;
//		     break;
//		case 2:
//		case 3:
//		     b = 0 - a - c;
//		     break; 		    
// 		case 4:
//		case 5:
//		     c=0 - a - b;
//		      break;  
//	}
 
      FOC->ia=a  * 3.3f/4095/0.0804f;
      FOC->ib=b  * 3.3f/4095/0.0804f;
 	   

 }
  void  CLARKE_PARK(X_FOC *FOC,ENCODER *motor_angle )
 {
 	 FOC->I_uAlpha=FOC->ia;
	 FOC->I_ubeta=(FOC->ia + FOC->ib*2)/_SQRT3 ;
 
	 motor_angle->sin_a = sinf(motor_angle->radian);
	 motor_angle->cos_a = cosf(motor_angle->radian);
	 
	  
	 FOC->id_current= FOC->I_uAlpha * motor_angle->cos_a + FOC->I_ubeta * motor_angle->sin_a;
	 FOC->iq_current=-FOC->I_uAlpha * motor_angle->sin_a + FOC->I_ubeta * motor_angle->cos_a;
	 
	 FOC->id_current =	Filter_func(&id_filter  ,  FOC->id_current );//滤波后作为pid的当前电流
	 FOC->iq_current =	Filter_func(&iq_filter  ,  FOC->iq_current );
	 
	 D_PID.pre =	FOC->id_current;//滤波后作为pid的当前电流
	 Q_PID.pre =	FOC->iq_current;

 }
	 
 float PID_i(I_pid *PID)
{
 
 
	PID->Err  =  PID->tar  -  PID->pre;
	PID->sum +=  PID->Err;
	PID->sum  =  Limit(PID->sum , PID->outMax);
	PID->out  =  PID->ki  *  PID->sum  +  PID->kp  *  PID->Err;
	PID->out  =  Limit(PID->out , PID->outMax);
	return PID->out;
	
}	 
void InversePark(X_FOC *FOC,ENCODER *motor_angle )
   {   
 
	 
	    FOC->uAlpha = FOC->Ud * motor_angle->cos_a - FOC->Uq * motor_angle->sin_a;

      FOC->uBeta = FOC->Uq *  motor_angle->cos_a  + FOC->Ud * motor_angle->sin_a;
 	   
}
		 
void SVPWM_Generate(X_FOC *FOC)
{
    float U1, U2, U3;
    uint8_t a, b, c;
    int16_t t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
    U1 = FOC->uBeta;
    U2 = (_SQRT3 * FOC->uAlpha - FOC->uBeta) / 2.0f;
    U3 = (-_SQRT3 * FOC->uAlpha - FOC->uBeta) / 2.0f;
    
    if(U1 > 0)
        a = 1;
    else 
        a = 0;
    if(U2 > 0)
        b = 1;
    else 
        b = 0;
    if(U3 > 0)
        c = 1;
    else 
        c = 0;
    
    num = (c << 2) + (b << 1) + a;
    
    switch(sectionMap[num])
    {
        case 0:
        {
            channel1 =0;
            channel2 = 0;
            channel3 =0;
        }break;
        
        case 1:
        {
					 
              t4 =  kx*U2;
              t6 =  kx*U1;
					  if((t4+t6)>PWM_DUTY)
						{
						t4=PWM_DUTY*t4/(t4+t6);
						t6=PWM_DUTY*t6/(t4+t6);
														
						}
              t0 = (PWM_DUTY - t4 - t6) / 2.0f;
            
            channel1 = t4 + t6 + t0;
            channel2 = t6 + t0;
            channel3 = t0;
        }break;
        
        case 2:
        {
              t2 =  -kx*U2;
              t6 =  -kx*U3;
						  if((t2+t6)>PWM_DUTY)
						{
						t2=PWM_DUTY*t2/(t2+t6);
						t6=PWM_DUTY*t6/(t2+t6);
														
						}
              t0 = (PWM_DUTY - t2 - t6) / 2.0f;
            
            channel1 = t6 + t0;
            channel2 = t2 + t6 + t0;
            channel3 = t0;
        }break;
        
        case 3:
        {
              t2 =  kx*U1;
              t3 =  kx*U3;
						  if((t2+t3)>PWM_DUTY)
						{
						t2=PWM_DUTY*t2/(t2+t3);
						t3=PWM_DUTY*t3/(t2+t3);
														
						}
              t0 = (PWM_DUTY - t2 - t3) / 2.0f;
            
            channel1 = t0;
            channel2 = t2 + t3 + t0;
            channel3 = t3 + t0;
        }break;
        
        case 4:
        {
              t1 =  -kx*U1;
              t3 =  -kx*U2;
						  if((t1+t3)>PWM_DUTY)
						{
						t1=PWM_DUTY*t1/(t1+t3);
						t3=PWM_DUTY*t3/(t1+t3);
														
						}
              t0 = (PWM_DUTY - t1 - t3) / 2.0f;
            
            channel1 = t0;
            channel2 = t3 + t0;
            channel3 = t1 + t3 + t0;
        }break;
        
        case 5:
        {
              t1 =  kx*U3;
              t5 =  kx*U2;
					  if((t1+t5)>PWM_DUTY)
						{
						t1=PWM_DUTY*t1/(t1+t5);
						t5=PWM_DUTY*t5/(t1+t5);
														
						}
              t0 = (PWM_DUTY - t1 - t5) / 2.0f;
            
            channel1 = t5 + t0;
            channel2 = t0;
            channel3 = t1 + t5 + t0;
        }break;
        
        case 6:
        {
              t4 =  -kx*U3;
              t5 =  -kx*U1;
					  if((t4+t5)>PWM_DUTY)
						{
						t4=PWM_DUTY*t4/(t4+t5);
						t5=PWM_DUTY*t5/(t4+t5);
														
						}
              t0 = (PWM_DUTY - t4 - t5) / 2.0f;
            
            channel1 = t4 + t5 + t0;
            channel2 = t0;
            channel3 = t5 + t0;
        }break;
        
        default:
            break;
    }
		TIM8->CCR1=channel1;
		TIM8->CCR2=channel2;
		TIM8->CCR3=channel3;
// __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, channel1);
//__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, channel2);
//__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, channel3);
 
}
void FOC_Control(X_FOC *FOC,ENCODER * motor_angle)
{
	
	get_current(FOC);
	motor_angle->radian=Luenberger_PLL.Theta_fore_now;
		//motor_angle->radian=get_angle(motor_angle);
	CLARKE_PARK(FOC,motor_angle );

//	FOC->Ud= 0;
//	FOC->Uq= 5;
	FOC->Ud= PID_i(&D_PID);
	FOC->Uq= PID_i(&Q_PID);
  InversePark(FOC,motor_angle );
  SVPWM_Generate(FOC);

}
	 
	 
	 
	 
	 