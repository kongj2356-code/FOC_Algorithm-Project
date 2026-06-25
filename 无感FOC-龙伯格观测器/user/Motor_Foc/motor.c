#include "motor.h"
 ENCODER  motor_angle;
ENCODER  motor_angle={
    .encoder = 0,
    .first_encoder = 0,
	  .CNT=0,
	  .sin_a=0,
    .cos_a=0,
	  .mAngle=0,               //»úÐµ½Ç¶È
    .angle=0,              //½Ç¶È
    .radian=0,   
};
  float get_angle(ENCODER * motor_angle)
{
 	  motor_angle->encoder = TIM2->CNT ;
 	  motor_angle->CNT = motor_angle->encoder + motor_angle->first_encoder ;
	  
 
 	return pair* fmod(_2PI*motor_angle->CNT/4000 ,_2PI);
	
}

 
 void Motor_Init(void)
 {
 
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
 
   	 HAL_Delay(1000);
 
 
 }
 
 void Motor_Reset(void)
 {
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,   700);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,  0);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);


 HAL_Delay(300);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  HAL_Delay(1000); 
 
	 
  	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	__HAL_TIM_SetCounter(&htim2,0);
 }
 
 
 void Inject_ADC(void)
 {
 	//__HAL_TIM_CLEAR_FLAG(&htim8,TIM_FLAG_UPDATE);
		__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_UPDATE);
		HAL_TIM_Base_Start_IT(&htim8);		
		HAL_TIM_Base_Stop_IT(&htim3);		
 	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,4197);
		 
    	HAL_ADCEx_InjectedStart(&hadc1);
   
 			 __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
 
 
 }
 void FOC_Init(void)
{
Motor_Init();
Motor_Reset();
Inject_ADC();
Motor_Enable();

}
void Motor_Enable(void)
{
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
}
void Motor_Disable(void)
{
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);

	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
}        
 

