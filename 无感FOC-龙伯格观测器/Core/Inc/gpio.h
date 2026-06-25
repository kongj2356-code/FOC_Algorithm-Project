/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#define  key1   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)
#define  key2   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)
#define  key3   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)
#define  key4   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)
#define  key5   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)

	
	
	
	
	
	
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

