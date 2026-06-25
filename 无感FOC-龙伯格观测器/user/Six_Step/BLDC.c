#include "bldc.h"
 int16_t duty=0;
uint8_t diretion=0;
 uint8_t stata=0;
uint16_t uwStep=0,ci=0;
	int32_t Ph[6] = {0}; // 霍尔信号
	int32_t phase=0;	  int16_t speed=0;
uint8_t	RT_hallPhase=0,LS_hallPhase=0;
int Aima_v=0;
uint8_t qh[2]={0};
void  stop_motor(void)
{
	pwm_off
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_TIM_Base_Stop_IT(&htim6);
	  HAL_TIMEx_HallSensor_Stop_IT(&htim5);

}
void  start_motor(void)
{
	pwm_en
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim6);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);
		  HAL_TIMEx_HallSensor_Start_IT(&htim5);

	HAL_Delay(8);

}
void V_up_W_down(void)
{
	
 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,duty);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);	
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);

}
void V_up_U_down(void)
{
	
 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,duty);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);	
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);

}
void W_up_U_down(void)
{
	
 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,duty);	
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);

}
void W_up_V_down(void)
{
	
 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,duty);	
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);

}
void U_up_V_down(void)
{
	
 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,duty);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);	
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);

}
void U_up_W_down(void)
{
	
 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,duty);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);	
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);

}
uint8_t read_sta(void)
{
   uint32_t sta=0 ;
	if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10)==SET)      sta|=0x04;
   if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11)==SET) sta|=0x02;
	 if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12)==SET) sta|=0x01;
	
//	  int32_t tmp = 0;
//  tmp |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10);//U(A)
//  tmp <<= 1;
//  tmp |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11);//V(B)
//  tmp <<= 1;
//  tmp |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12);//W(C)
//  return (tmp&0x0007); // 取低三位
	return sta;
}
void six_huan(uint8_t sta)
{
 if(diretion==go)
   {		
				switch(sta)
				{
					case 0X01:  V_up_W_down();   break;
					case 0X02:  U_up_V_down();   break;
					case 0X03:  U_up_W_down();   break;
					case 0X04:  W_up_U_down();   break;
					case 0X05:  V_up_U_down();   break;
					case 0X06:  W_up_V_down();   break;
 
	    	}	
	  }	
	    if(diretion==back)
   {		
				switch(7-sta)
				{
					case 0X01:  V_up_W_down();   break;
					case 0X02:  U_up_V_down();   break;
					case 0X03:  U_up_W_down();   break;
					case 0X04:  W_up_U_down();   break;
					case 0X05:  V_up_U_down();   break;
					case 0X06:  W_up_V_down();   break;
 
	    	}	
	  }
		
}
void  BLDCMotor_Start(void)
{ 
  
  /* 下桥臂导通,给自居电容充电 */
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);

//  HAL_TIM_GenerateEvent(&htimx_BLDCM, TIM_EVENTSOURCE_COM); // 软件生成COM事件
//  __HAL_TIM_CLEAR_FLAG(&htimx_BLDCM,TIM_FLAG_COM);
   HAL_Delay(9); //充电时间, 大概值 不需要准确
  
    stata=read_sta(); // 获取霍尔信号相位
	  six_huan(stata);  
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM); // 软件生成COM事件
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_COM);
  /* 根据当前的霍尔信号配置PWM波形 */
//  HAL_TIM_GenerateEvent(&htimx_BLDCM, TIM_EVENTSOURCE_COM); // 软件生成COM事件
//  __HAL_TIM_CLEAR_FLAG(&htimx_BLDCM, TIM_FLAG_COM);
}
int16_t PID_Calculate(short Aima_v,short speed)
{
//	float Kp =0.1, Ki = 0.9, Kd = 0.12,a=0.8;
//	static int32_t aim , error_now = 0, error_last = 0, v_now = 0, error_i = 0, out = 0,last_out=0,output=0;
//   aim = Aima_v;
//	v_now = speed;
//	error_now = aim - v_now;
//	error_i = error_i + error_now;
//	if(error_i > 1710) error_i = 1710;	if(Aima_v==0)error_i=0;
//	out = Kp * error_now + Ki * error_i + Kd * (error_now - error_last);
//	error_last=error_now;
//	output=a*last_out+(1-a)*out;
//	last_out=out;
//	return output;

	float Kp = 0.16, Ki = 0.25, Kd = 0,a=0.6;
static int32_t aim , In = 0, error_now = 0, error_last = 0, error_last_last = 0, v_now = 0, out = 0,last_out=0,output=0;
   aim= Aima_v;
	 v_now = speed;
error_now = aim - v_now;
In=Kp*(error_now-error_last)+Ki*(error_now)+Kd*(error_now+error_last_last-2*error_last);
out+=In;
error_last_last=error_last;
error_last=error_now;
	output=a*last_out+(1-a)*out;
	last_out=out;
	return output;
}

int16_t PID_Control(short in)
{
	if(in>=0)
	{ 
		diretion=go;
		if(in > 2900) 
		in = 2900;
	}
   else if(in <0)
 {  
	  diretion=back;
		in = -in;
	  if(in>2900)
	  in=2900;
 }
  return in;


}
void BLDCMotor_SetSpeed(int32_t dut)
{
  int32_t hallPhase = 0;
		 if(dut > 0 )
      {
        diretion = go;
				if(dut>=2900)dut=2900;
      }
      else if(dut<=0)
      {
        diretion = back;		
       dut=-dut;						
				if(dut>=2900)dut=2900;
	
      }

  duty = dut ;
 	stata=read_sta();
  if(diretion==go)
   {		
				switch(stata)
				{
					case 0X01: 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,duty);  break;
					case 0X02: 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,duty);  break;
					case 0X03:	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,duty);  break;
					case 0X04:  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,duty);  break;
					case 0X05:  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,duty);  break;
					case 0X06:  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,duty);  break;
 
	    	}	
	  }	
	    if(diretion==back)
   {		
				switch(7-stata)
				{
					case 0X01: 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,duty);  break;
					case 0X02: 	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,duty);  break;
					case 0X03:	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,duty);  break;
					case 0X04:  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,duty);  break;
					case 0X05:  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,duty);  break;
					case 0X06:  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,duty);  break;
 
	    	}	
	  }
}
