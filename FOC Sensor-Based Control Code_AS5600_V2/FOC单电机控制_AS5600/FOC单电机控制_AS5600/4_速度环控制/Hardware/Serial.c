#include "stm32f10x.h"                  // Device header
#include <stdarg.h>
#include <string.h> 
#include <Serial.h> 
#include <math.h> 

/*定义接收缓存和状态变量 ********************/
char Serial_RxBuffer[RX_BUF_SIZE] = {0}; // 接收字符缓存（存储完整数字字符串）
uint8_t Serial_RxIndex = 0;              // 缓存索引（记录当前存到第几个字符）
uint8_t Serial_RxFlag = 0;               // 接收完成标志位（1=解析完成）
float Serial_RxData = 0.0f;              // 最终解析后的浮点数
/********************定义接收缓存和状态变量 */

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);    //开启USART1的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    //开启GPIOB的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);     //开启AFIO时钟，用于重映射（关键！）
    
    /*GPIO重映射配置*/
    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);          //使能USART1重映射到PB6/PB7（关键！）
    
    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PB6作为USART1_TX，复用推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                // PB6作为USART1_TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PB7作为USART1_RX，上拉输入（或浮空输入）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    // 浮空输入，或使用GPIO_Mode_IPU上拉输入
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         // 上拉输入也可以
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                // PB7作为USART1_RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /*USART初始化*/
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
    
    /*中断输出配置*/
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    /*NVIC中断分组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /*NVIC配置*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    /*USART使能*/
    USART_Cmd(USART1, ENABLE);
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
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
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */
uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;			//返回接收的数据变量
}


//////////////////////////////////////////////////////////////////////////////////////////

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


/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  */

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
