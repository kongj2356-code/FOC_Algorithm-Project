/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   重定向c库printf函数到usart端口，支持接收小数字符串并解析为浮点数
  ******************************************************************************
  */ 
	
#include "Serial.h"
#include <math.h>  
#include <string.h> 

// ********** 定义接收缓存和状态变量 **********

char Serial_RxBuffer[RX_BUF_SIZE] = {0}; // 接收字符缓存（存储完整数字字符串）
uint8_t Serial_RxIndex = 0;              // 缓存索引（记录当前存到第几个字符）
uint8_t Serial_RxFlag = 0;               // 接收完成标志位（1=解析完成）
float Serial_RxData = 0.0f;              // 最终解析后的浮点数
// ********** 定义缓存和状态变量 **********


 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口GPIO的时钟
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	// 打开串口外设的时钟
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	
	// 串口中断优先级配置
	NVIC_Configuration();
	
	// 使能串口接收中断
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
	
	// 使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);	    
}

/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
		
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}



// 转换函数（保留原逻辑，仅修正注释）
float mstrtof(char *num)
{   
	double n = 0, sign = 1, scale = 0;
  int subscale = 0, signsubscale = 1;
	if (*num == '-') 
	{
      sign = -1, num++;	/* 处理负号 */
  }
	// 跳过开头的无效0
	while (*num == '0')
	{
		num++;
	}

	// 解析整数部分
	if (*num >= '1' && *num <= '9')	
	{
		do {
			n = (n * 10.0) + (*num++ - '0');	
		} while (*num >= '0' && *num <= '9');
	}

	// 解析小数部分
	if (*num == '.' && num[1] >= '0' && num[1] <= '9') {
        num++;		
        do {	
            n = (n * 10.0) + (*num++ -'0'), scale--; 
        } while (*num >= '0' && *num <= '9');
    }	

	// 解析指数部分
	if (*num == 'e' || *num == 'E')	{	
		num++;
        if (*num == '+') {
            num++;	
        } else if (*num == '-') { 
            signsubscale = -1, num++;		
		}

        while (*num >= '0' && *num <= '9' ) { 
            subscale = (subscale * 10) + (*num++ - '0');	
        }
	}
	   n = sign * n * pow(10.0, (scale + subscale * signsubscale));	
	return (float)n; // 显式转换为float，更规范
}



// ********** 重写串口中断函数 **********
void USART1_IRQHandler(void)
{
    uint8_t rx_char; // 临时存储接收到的单个字符
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        // 1. 读取当前接收到的字符（必须是uint8_t/char类型）
        rx_char = (uint8_t)USART_ReceiveData(USART1);
        
        // 2. 判断是否是结束符（这里用换行符\n作为结束，你也可以用\r或其他）
        if (rx_char == '\n') 
        {
            // 3. 接收到结束符，添加字符串结束符\0，确保mstrtof能正确解析
            Serial_RxBuffer[Serial_RxIndex] = '\0';
            // 4. 解析完整的数字字符串为浮点数
            Serial_RxData = mstrtof(Serial_RxBuffer);
            // 5. 设置接收完成标志
            Serial_RxFlag = 1;
            // 6. 重置索引，准备下一次接收
            Serial_RxIndex = 0;
        }
        else
        {
            // 3. 未到结束符，将字符存入缓存（防止缓存溢出）
            if (Serial_RxIndex < RX_BUF_SIZE - 1) // 留1位存\0
            {
                Serial_RxBuffer[Serial_RxIndex++] = rx_char;
            }
            else
            {
                // 缓存溢出，重置索引（可选：也可加溢出标志）
                Serial_RxIndex = 0;
                memset(Serial_RxBuffer, 0, RX_BUF_SIZE);
            }
        }
        // 清除中断标志位
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}