
#include "UART_VOFA.h"
 #include "stdlib.h" 

 float sdu[2]={1.2225f,2.775f};

void USART_SendArray(uint8_t* pData, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        HAL_UART_Transmit(&huart1, &pData[i], 1, 0xffff);
    }
}
void UploadData_vofa(float *data, uint8_t channel_count) 
{ 
 // 定义数据缓冲区，大小由每个浮点数 4 字节 * 通道数量 + 帧尾 4 字节决定 
 uint8_t data_size = channel_count * 4 + 4; 
uint8_t *tempData = (uint8_t *)malloc(data_size); 
 
 if (tempData == NULL) { 
 // 错误处理：内存分配失败 
 return; 
 } 
 // 将浮点数数据复制到字节缓冲区 
 memcpy(tempData, (uint8_t *)data, channel_count * 4); 
 
 // 设置帧尾 
 tempData[channel_count * 4] = 0x00; 
 tempData[channel_count * 4 + 1] = 0x00; 
 tempData[channel_count * 4 + 2] = 0x80; 
tempData[channel_count * 4 + 3] = 0x7F; 
 
//串口发送数组，不同 MCU 函数名称不同，一般都能找到标准库封装 
//的发送单字节的函数，可以自己实现一下数组的发送 
USART_SendArray(  tempData, data_size); 
 
 free(tempData); // 释放内存 
}



int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);	
	
	return (ch);
}

