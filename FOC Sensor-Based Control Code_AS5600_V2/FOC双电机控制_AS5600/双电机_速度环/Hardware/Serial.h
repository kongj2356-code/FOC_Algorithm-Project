#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
/*************************************/
#define RX_BUF_SIZE 32                   // 接收缓存大小，可容纳最长32位数字字符串
extern float Serial_RxData ;              // 最终解析后的浮点数
/*************************************/
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

// 转换函数（保留原逻辑，仅修正注释）
float mstrtof(char *num);
#endif
