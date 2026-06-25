#ifndef _UART_VOFA_H_
#define _UART_VOFA_H_
#include "main.h"
 #include "stdio.h"

#include "gpio.h"
    #include "usart.h"
extern float sdu[2];
  void UART_SendArray(uint8_t* array, uint16_t length);
void UploadData_vofa(float *data, uint8_t channel_count) ;

#endif 
