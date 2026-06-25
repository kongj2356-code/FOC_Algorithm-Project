#include "sin_cos.h"
#include "arm_math.h"
#include "math.h"
#include "adc.h"		 
#include "tim.h"
#include "usart.h"
  #include "stdlib.h" 

/* includes ------------------------------------------------------------------*/

/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
//uint16_t channel1=0, channel2=0, channel3=0,channel4=10;
//float uAlpha, uBeta,uAlpha_last, uBeta_last,V_Alpha,V_Beta,V_Alpha_LPF,V_Beta_LPF;
//float ud=0,uq= 0;
//int TS=PWM_DUTY;
//float kx=(PWM_DUTY*_SQRT3)/24;
//float I_uAlpha=0,I_ubeta=0, I_uAlpha_last=0,I_ubeta_last=0,Iaa=0,Ibb=0,Iaa_last=0,Ibb_last=0,id_current=0,iq_current=0,id_expect=0,iq_expect=0;
// float ALPHA=0.5f,ALPHB=0.75f,Vab_alpha= 0.15f,We_alpha = 0.96f,speed_alpha=0.005f;

//uint8_t num=0;
//short jishu=0  ;
//uint32_t offsetValue[3]= {0};
//uint8_t foc_flag=0;
//short first_encoder=0,current_encoder=0;
//float jixie_angle=0,dianqi_angle=0;
//uint8_t sectionMap[7] = {0, 2, 6, 1, 4, 3, 5};
//int la=0,lb=0,lc=0;
//short JIXIE,XUNI;
//float sin_a=0,cos_a=0;
// float PLL_Kp = 1.5f, PLL_Ki= 250.0f  ;
//	float gama=1000000.0f ;
//// float PLL_Kp = 1.5f, PLL_Ki =2.2f  ;
////	float gama=130 ;

//// float Kp_id = LLL*Banwide, Ki_id =  RRR*Banwide/PWM_HZ ;
//  float Kp_id = LLL*Banwide, Ki_id =  RRR*Banwide/PWM_HZ ;
//float Kp_speed = 0.05f, Ki_speed = 0.006f  ;
//float Kp_position = 0.086f, Ki_position = 0.0f ,Kd_position =1.1f;
//float speed_now=0,speed_last=0,speed_target=0;
//float position_now=0,position_last=0,position_target=0;
//short round_z=0;

//float Now_Forecast_Ialpha=0,Now_Forecast_Ibeta=0,New_Forecast_Ialpha=0,New_Forecast_Ibeta=0;
//float Theta_fore_Last=0, Theta_fore_now=0,We_fore=0,fore_speed=0,We_fore_lpf=0;
//	float KK=0,h=3.4f,TTSS=0.0001f,buchan=0,Ea,Eb,flux=0.00583f,hmax;
//  double x_alpha=0,x_alpha_last=0,x_beta=0,x_beta_last=0,y_x_a,y_x_b,err;

//float my_abs(float x) 
//{
//    if (x < 0)
//        return -x;
//    else 
//        return x;
//}
//void get_set(void)
//{
//	uint16_t i=0;
//	for(i=0;i<1;i++)
//	{
//	offsetValue[0]+=hadc1.Instance->JDR1;
//	offsetValue[1]+=hadc1.Instance->JDR2;
//	offsetValue[2]+=hadc1.Instance->JDR3;
//	}
//	offsetValue[0]/=1;
//	offsetValue[1]/=1;
//	offsetValue[2]/=1;
//	foc_flag=1;
// }
//void get_current(void)
//{
//	
//			la=-(hadc1.Instance->JDR1-2035);
//	   	lb=-(hadc1.Instance->JDR2-2035);
//		 	lc=-(hadc1.Instance->JDR3-2035);
// 	 
//	switch(sectionMap[num])
//	{
//	  case 1:
//		case 6:
//			   la=0-lb-lc;
//		     break;
//		case 2:
//		case 3:
//		     lb=0-la-lc;
//		     break; 		    
// 		case 4:
//		case 5:
//		     lc=0-la-lb;
//		      break;  
//	}
// 
//      Iaa=la  * 3.3f/4095/0.0804f;
//      Ibb=lb  * 3.3f/4095/0.0804f;
//	   

// }
//  void  CLARKE_PARK(float ag,float Iaa,float Ibb)
// {
// 	 I_uAlpha=Iaa;
//	 I_ubeta=(Iaa + Ibb*2)/_SQRT3 ;
// 
//	   sin_a=sinf(ag );
//		 cos_a=cosf(ag );
//		 
//	 id_current= I_uAlpha * cos_a+ I_ubeta * sin_a;
//	 iq_current=-I_uAlpha * sin_a + I_ubeta * cos_a;
// 	 id_current=lowPassFilterA(id_current);
//	 iq_current=lowPassFilterB(iq_current);
// 
// }
//void InversePark( float ud, float uq)
//   {   
// 
//	 
//	    uAlpha = ud * cos_a - uq * sin_a;

//      uBeta = uq *  cos_a  + ud * sin_a;
// 	   
//}
//	 
//float Limit(float value,int lim)
//{
//	float tem = value;
//	if(fabsf(tem)>lim)
//	{
//		if(tem>0)
//		{	
//		tem=lim;
//		}
//		else
//		{
//		tem=-lim;	
//		}
//	}
//	return tem;
//}
// float lowPassFilterA(float input) {
//    static float previousOutput = 0.0f;
//    previousOutput = ALPHA * input+(1 - ALPHA)*previousOutput;
//    return previousOutput;
//}
// float lowPassFilterB(float input) {
//    static float previousOutput = 0.0f;
//    previousOutput = ALPHA * input+(1 - ALPHA)*previousOutput;
//    return previousOutput;
//}
//  short lowPassSpeed(short input) {
//    static short previousOutput = 0;
//    previousOutput = ALPHB * input+(1 - ALPHB)*previousOutput;
//    return previousOutput;
//}
//	float We_Filter(float data)
//{
//	static float y , Last_y;
// 	y=We_alpha*data+(1-We_alpha)*Last_y;
//	Last_y = y;
//	return y;
//}

//float fore_speed_Filter(float data)
//{
//	static float Last_y;
// 	data=speed_alpha*data+(1-speed_alpha)*Last_y;
//	Last_y = data;
//	return data;
//}
//float AngleB_Filter(float data)
//{
//	static float Last_y;
// 	data=Vab_alpha*data+(1-Vab_alpha)*Last_y;
//	Last_y = data;
//	return data;
//}
//float PID_position(float target,float now)
//{
//	static float Err=0 ,out=0,Err_Last=0;
//	static float sum =0;
//	Err = target - now;
//	if(Err>-0.008&&Err<0.008)Err=0;
//	sum+=  Ki_position* Err;
//	sum = Limit(sum , 4);
//	out = Kp_position*sum+ Err+Kd_position*(Err-Err_Last);
//	out = Limit(out , 4);
//	Err_Last=Err;
//	if(out>0)
//		out+=0.2;
//	else 
//		out-=0.2;
//	return out;
//	 
//	
//	 
//}
//  float PID_speed(float target,float now)

//{
//  
//	static float Err=0 ,out=0;
//	static float sum =0;
//	Err = target - now;
//	sum+=  Err;
//	sum = Limit(sum , 13);
//	out = Kp_speed*sum+ Ki_speed*Err;
//	out = Limit(out , 13);
//	
//	return out;
//	
//}
// float PID_id(void  )
//{
// 
//	static float Err_iq=0 , Err_id,out=0;
//	static float sum_iq =0, sum_id=0;
//	Err_iq = id_expect - id_current;
//	sum_iq +=    Err_iq;
//	sum_iq = Limit(sum_iq , 13);
//	out = Ki_id*sum_iq + Kp_id*Err_iq;
//	out = Limit(out , 13);
//	return out;
//	
//}
// float PID_iq(float  iq_expect)
//{
// 
//   	static float Err_iq=0 , Err_id,out=0;
//	static float sum_iq =0, sum_id=0;
//	Err_iq = iq_expect - iq_current;
//	sum_iq +=  Err_iq;
//	sum_iq = Limit(sum_iq , 13);
//	out = Ki_id* sum_iq + Kp_id*Err_iq;
//	out = Limit(out , 13);
//	return out;
//}
// 
//float get_angle(void)
//{
// 
//	  jishu=TIM2->CNT ;
// 	 current_encoder=jishu+first_encoder ;
//	 
//  // position_angle(current_encoder);		//位置环的角度值计算
//	//return _2PI*current_encoder/1000 ;
//	 //return _2PI*current_encoder/1000 ;
// 	return 4* fmod(_2PI*current_encoder/4000 ,_2PI);
//	
//}
//void position_angle(short encoder)
//{
//	static short last_encoder=0  ;
//	short i =0 ; 
//  static int total=0;
//		 i =encoder - last_encoder;
//   if (i > -4000&&i<-3500)  
//  {
//    i += 4000;
//  }
//  else if (i <4000&&i>3500) 
//	{
//    i -= 4000;
//   }
//		total += i;
//		position_now=(total)* _2PI/4000 ;
//		last_encoder  = current_encoder;
//	
//}
// 
// float extended_atan(float y,float x) 
//{
// 	uint8_t setor=0;
// 	float angle=0;
//	if(x>0)
//	 {
//		 if(y>0)setor=1;
//		 else  setor=4;
//	 }
//	 else
//	 {
//		 if(y>0)setor=2;
//		 else  setor=3;
//	 }
//	 switch(setor)
//   {
//	   case 1: angle=atanf(y/x);break;
//		 case 2: angle=_PI_2+atanf(y/x)+_PI_2;break;
//		 case 3: angle=_PI+atanf(y/x);break;
//	   case 4: angle=_PI_2+atanf(y/x)+_3PI_2;break;
//	 
//	 
//	 }  
// 
//    return angle;
//}
//void SMO_PLL(void)
//{
//	static float err , P_Partern , I_Partern ;
//	err = -cosf(Theta_fore_Last)*V_Alpha_LPF-sinf(Theta_fore_Last)*V_Beta_LPF;
// 	I_Partern += PLL_Ki *  err;
//	We_fore = PLL_Kp * err+  I_Partern;
//	We_fore = Limit(We_fore , 1256);//限幅，额定转速为3000rpm
//	We_fore = We_Filter(We_fore);
//	Theta_fore_now  += We_fore*TTSS;
//	Theta_fore_now=fmod(Theta_fore_now ,4*_2PI);
//  fore_speed=	We_fore*60/4/_2PI;
//	fore_speed=fore_speed_Filter(fore_speed);
//	Theta_fore_Last=Theta_fore_now;
//	
//	
//}
// void SMO_Observer(void)
//{
//	int signa=1;
//	float aa=0;
// 	KK=TTSS/LLL;
// 
// 
//	
// 
//	 if(Now_Forecast_Ialpha-I_uAlpha_last>1)
//	 { V_Alpha=h;
//	    signa=1;
//	 }
//  else if(Now_Forecast_Ialpha-I_uAlpha_last<-1)
//	{ V_Alpha=-h; signa=-1;}
//  else 
//		 V_Alpha=h*(Now_Forecast_Ialpha-I_uAlpha_last);
//	
//  if(Now_Forecast_Ibeta-I_ubeta_last>1)
//		 V_Beta=h;
//  else if(Now_Forecast_Ibeta-I_ubeta_last<-1)
//		 V_Beta=-h;
//  else 
//		 V_Beta=h*(Now_Forecast_Ibeta-I_ubeta_last);
//	
// // Ea=-speed_now*flux*sin_a;
////	hmax=-RRR*fabsf(New_Forecast_Ialpha-I_uAlpha_last)+Ea*signa;
////	
//	V_Alpha_LPF = Va_LPF_Filter(V_Alpha);
//	V_Beta_LPF  = Vb_LPF_Filter(V_Beta);
// 	New_Forecast_Ialpha = (1-RRR*KK)* Now_Forecast_Ialpha + KK * uAlpha_last- KK * V_Alpha_LPF;
//  New_Forecast_Ibeta  = (1-RRR*KK)* Now_Forecast_Ibeta  + KK * uBeta_last - KK * V_Beta_LPF;
// 
////	Eb=speed_now*flux*cos_a;
//	 SMO_PLL();
//	
//	Now_Forecast_Ialpha =New_Forecast_Ialpha;
//	Now_Forecast_Ibeta  =New_Forecast_Ibeta;
//	I_uAlpha_last=I_uAlpha;
//	I_ubeta_last=I_ubeta;
//	uAlpha_last=uAlpha;
//	uBeta_last=uBeta;
//}
// 
// 
//float  Va_LPF_Filter(float data)
//{
//	static float Last_y;
// 	data=Vab_alpha*data+(1-Vab_alpha)*Last_y;
//	Last_y = data;
//	return data;
//}
//float Vb_LPF_Filter(float data)
//{
//	static float Last_y;
// 	data=Vab_alpha*data+(1-Vab_alpha)*Last_y;
//	Last_y = data;
//	return data;
//}
// 
//void USART_SendArray(uint8_t* pData, uint16_t Size)
//{
//    for (uint16_t i = 0; i < Size; i++)
//    {
//        HAL_UART_Transmit(&huart1, &pData[i], 1, 0xffff);
//    }
//}
//void UploadData_vofa(float *data, uint8_t channel_count) 
//{ 
// // 定义数据缓冲区，大小由每个浮点数 4 字节 * 通道数量 + 帧尾 4 字节决定 
// uint8_t data_size = channel_count * 4 + 4; 
//uint8_t *tempData = (uint8_t *)malloc(data_size); 
// 
// if (tempData == NULL) { 
// // 错误处理：内存分配失败 
// return; 
// } 
// // 将浮点数数据复制到字节缓冲区 
// memcpy(tempData, (uint8_t *)data, channel_count * 4); 
// 
// // 设置帧尾 
// tempData[channel_count * 4] = 0x00; 
// tempData[channel_count * 4 + 1] = 0x00; 
// tempData[channel_count * 4 + 2] = 0x80; 
//tempData[channel_count * 4 + 3] = 0x7F; 
// 
////串口发送数组，不同 MCU 函数名称不同，一般都能找到标准库封装 
////的发送单字节的函数，可以自己实现一下数组的发送 
//USART_SendArray(  tempData, data_size); 
// 
// free(tempData); // 释放内存 
//}

//void SVPWM_Generate(void)
//{
//    float U1, U2, U3;
//    uint8_t a, b, c;
//    int16_t t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0,t7=0;
//    U1 = uBeta;
//    U2 = (_SQRT3 * uAlpha - uBeta) / 2.0f;
//    U3 = (-_SQRT3 * uAlpha - uBeta) / 2.0f;
//    
//    if(U1 > 0)
//        a = 1;
//    else 
//        a = 0;
//    if(U2 > 0)
//        b = 1;
//    else 
//        b = 0;
//    if(U3 > 0)
//        c = 1;
//    else 
//        c = 0;
//    
//    num = (c << 2) + (b << 1) + a;
//    
//    switch(sectionMap[num])
//    {
//        case 0:
//        {
//            channel1 =0;
//            channel2 = 0;
//            channel3 =0;
//        }break;
//        
//        case 1:
//        {
//					 
//              t4 =  kx*U2;
//              t6 =  kx*U1;
//					  if((t4+t6)>PWM_DUTY)
//						{
//						t4=PWM_DUTY*t4/(t4+t6);
//						t6=PWM_DUTY*t6/(t4+t6);
//														
//						}
//              t0 = (TS - t4 - t6) / 2.0f;
//            
//            channel1 = t4 + t6 + t0;
//            channel2 = t6 + t0;
//            channel3 = t0;
//        }break;
//        
//        case 2:
//        {
//              t2 =  -kx*U2;
//              t6 =  -kx*U3;
//						  if((t2+t6)>PWM_DUTY)
//						{
//						t2=PWM_DUTY*t2/(t2+t6);
//						t6=PWM_DUTY*t6/(t2+t6);
//														
//						}
//              t0 = (TS - t2 - t6) / 2.0f;
//            
//            channel1 = t6 + t0;
//            channel2 = t2 + t6 + t0;
//            channel3 = t0;
//        }break;
//        
//        case 3:
//        {
//              t2 =  kx*U1;
//              t3 =  kx*U3;
//						  if((t2+t3)>PWM_DUTY)
//						{
//						t2=PWM_DUTY*t2/(t2+t3);
//						t3=PWM_DUTY*t3/(t2+t3);
//														
//						}
//              t0 = (TS - t2 - t3) / 2.0f;
//            
//            channel1 = t0;
//            channel2 = t2 + t3 + t0;
//            channel3 = t3 + t0;
//        }break;
//        
//        case 4:
//        {
//              t1 =  -kx*U1;
//              t3 =  -kx*U2;
//						  if((t1+t3)>PWM_DUTY)
//						{
//						t1=PWM_DUTY*t1/(t1+t3);
//						t3=PWM_DUTY*t3/(t1+t3);
//														
//						}
//              t0 = (TS - t1 - t3) / 2.0f;
//            
//            channel1 = t0;
//            channel2 = t3 + t0;
//            channel3 = t1 + t3 + t0;
//        }break;
//        
//        case 5:
//        {
//              t1 =  kx*U3;
//              t5 =  kx*U2;
//					  if((t1+t5)>PWM_DUTY)
//						{
//						t1=PWM_DUTY*t1/(t1+t5);
//						t5=PWM_DUTY*t5/(t1+t5);
//														
//						}
//              t0 = (TS - t1 - t5) / 2.0f;
//            
//            channel1 = t5 + t0;
//            channel2 = t0;
//            channel3 = t1 + t5 + t0;
//        }break;
//        
//        case 6:
//        {
//              t4 =  -kx*U3;
//              t5 =  -kx*U1;
//					  if((t4+t5)>PWM_DUTY)
//						{
//						t4=PWM_DUTY*t4/(t4+t5);
//						t5=PWM_DUTY*t5/(t4+t5);
//														
//						}
//              t0 = (TS - t4 - t5) / 2.0f;
//            
//            channel1 = t4 + t5 + t0;
//            channel2 = t0;
//            channel3 = t5 + t0;
//        }break;
//        
//        default:
//            break;
//    }
//		TIM8->CCR1=channel1;
//		TIM8->CCR2=channel2;
//		TIM8->CCR3=channel3;
//// __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, channel1);
////__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, channel2);
////__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, channel3);
// 
//}

