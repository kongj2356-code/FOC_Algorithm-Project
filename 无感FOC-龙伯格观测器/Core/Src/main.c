/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include "bsp_key.h"
  #include "arm_math.h" 
#include "FOC.h"
 #include "UART_VOFA.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float F_theta=0,J_theta=0;;
uint8_t start_flag=0;
 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 /* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
 void adc_offset(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//KEY_GPIO_Init();
   
	 FOC_Init(); 
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 	 
  // OLED_ShowNum(20,2,2,2,16);
 
  	 		if(key1==0)
		{	  NVIC_SystemReset();
					//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
		}
			 		if(key2==0)
		{	    
			// pwm_off
				HAL_Delay(10);
			while(key2==0);
			start_flag=2;

			
		}
			   	if(key5==0)
		{	    
					HAL_Delay(10);
			while(key5==0);
      Motor_Disable();		

		}
					if(key3==0)
		{	    
					HAL_Delay(10);
			while(key3==0);
      Q_PID.tar+=0.4;
		}
  


 
	   	if(key4==0)
		{	    
					HAL_Delay(10);
			while(key4==0);

      Q_PID.tar-=0.4;

		}
 
  
 }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//100us
{
 	  static int num=0;
	  static uint8_t jk=0;
	  static short encoder_now=0,encoder_last=0;
		if (htim->Instance == TIM8)
	{	
		num++;
    if(num>=10)
		{
					num=0;
					if(jk==1)
					{
									encoder_now= TIM2->CNT ;
									if(encoder_now-encoder_last>-2000&&encoder_now-encoder_last<2000)//前后相差很大的编码值就不要了
										{ 
											MOTOR_FOC.RPM=(encoder_now-encoder_last)*15;//(encoder_now-encoder_last)*60*1000/4000			
											MOTOR_FOC.WE=pair*MOTOR_FOC.RPM*_2PI/60;
										}
						     //  MOTOR_FOC.WE=4*calculate_motor_speed( _2PI*encoder_now/4000.0 ,   _2PI*encoder_last /4000.0,  0.001) ;

						
 									encoder_last=encoder_now;
 							 									
					}
					else //只会计算一次
					{
								 encoder_last=TIM2->CNT;
								 jk=1;
					}
     
		}

		
 }
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{

 	    if(start_flag==1) //有感
		 {
 
			get_current(&MOTOR_FOC);
			motor_angle.radian=get_angle(&motor_angle);

			CLARKE_PARK(&MOTOR_FOC ,&motor_angle );


			MOTOR_FOC.Ud= PID_i(&D_PID);
			MOTOR_FOC.Uq= PID_i(&Q_PID);

			InversePark(&MOTOR_FOC ,&motor_angle );
			SVPWM_Generate(&MOTOR_FOC );

			Luenberger_Observer(&MOTOR_FOC, &Luenberger_member);

 		 }
     else //切换无感
		 {
			get_current(&MOTOR_FOC);
			motor_angle.radian= Luenberger_PLL.Theta_fore_now;
			CLARKE_PARK(&MOTOR_FOC ,&motor_angle );


			MOTOR_FOC.Ud= PID_i(&D_PID);
			MOTOR_FOC.Uq= PID_i(&Q_PID);
			InversePark(&MOTOR_FOC ,&motor_angle );
			SVPWM_Generate(&MOTOR_FOC );

			Luenberger_Observer(&MOTOR_FOC, &Luenberger_member);

		 }  

 sdu[0]=Luenberger_PLL.Theta_fore_now;
 sdu[1]=motor_angle.radian;

   UploadData_vofa(sdu,2);
////     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);

} 


 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint8_t first_flag=0;
 	if(GPIO_Pin == GPIO_PIN_6)
		{
			if(first_flag==0)
			{
				  first_flag=1;
				  motor_angle.first_encoder=__HAL_TIM_GET_COUNTER(&htim2);
 			    __HAL_TIM_SetCounter(&htim2,0);
			}
			else
			{
				 __HAL_TIM_SetCounter(&htim2,0);
 			}
  	}
	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

