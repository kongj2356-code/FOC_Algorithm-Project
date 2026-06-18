/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"


/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_RxData;		//定义串口接收的数据变量
uint8_t Serial_RxFlag;		//定义串口接收的标志位变量

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART1_ENABLE();

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}









/* USER CODE BEGIN 1 */

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
  // HAL库发送一个字节（阻塞方式）
  HAL_UART_Transmit(&huart1, &Byte, 1, 100);  // 超时时间100ms，防止卡死
}


/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
  HAL_UART_Transmit(&huart1, Array, Length, 100);  // 直接用HAL库批量发送，效率更高
}



/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
  uint16_t Length = 0;
  // 计算字符串长度
  while(String[Length] != '\0')
  {
    Length++;
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)String, Length, 100);
}



/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
  uint32_t Result = 1;
  while (Y --)
  {
    Result *= X;
  }
  return Result;
}


/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
  uint8_t i;
  for (i = 0; i < Length; i ++)
  {
    Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
  }
}


/**
  * 函    数：printf重定向（底层函数）
  * 说    明：需要在MDK中开启微库（Use MicroLIB）
  */
int fputc(int ch, FILE *f)
{
  Serial_SendByte(ch);
  return ch;
}



/**
  * 函    数：自定义printf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
  char String[100];
  va_list arg;
  va_start(arg, format);
  vsprintf(String, format, arg);
  va_end(arg);
  Serial_SendString(String);
}



/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：1表示接收到数据，0表示未接收；读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
  if (Serial_RxFlag == 1)
  {
    Serial_RxFlag = 0;
    return 1;
  }
  return 0;
}



/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的字节数据
  */
uint8_t Serial_GetRxData(void)
{
  return Serial_RxData;
}


/**
  * 函    数：HAL库串口接收中断回调函数
  * 说    明：中断接收完成后自动调用此函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    Serial_RxFlag = 1;  // 置接收标志位
    // 重新开启下一次接收中断（HAL库中断接收是一次性的，必须重新开启）
    HAL_UART_Receive_IT(&huart1, &Serial_RxData, 1);
  }
}

/* USER CODE END 1 */

