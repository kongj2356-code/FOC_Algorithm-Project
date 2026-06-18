#ifndef __UPPER_H
#define __UPPER_H

#include "header.h"
#include "FOC_Struct.h"
#include "Control.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void UART_Start_DMA_Transfer(void);


//´®¿Ú·¢ËÍº¯Êý
void Float_to_Byte(float f,unsigned char byte[]);
void Float_send(float *data_array,uint8_t CH_COUNT);
void Send_array(unsigned char* byte, uint8_t Number);

// AS5600 ¼Ä´æÆ÷µØÖ·
#define AS5600_I2C_ADDRESS        0x36
#define AS5600_REG_STATUS         0x0B
#define AS5600_REG_RAW_ANGLE_H    0x0C
#define AS5600_REG_RAW_ANGLE_L    0x0D
#define AS5600_REG_ANGLE_H        0x0E
#define AS5600_REG_ANGLE_L        0x0F
#define AS5600_REG_AGC            0x1A
#define AS5600_REG_MAGNITUDE_H    0x1B
#define AS5600_REG_MAGNITUDE_L    0x1C

// ×´Ì¬Î»¶¨Òå
#define AS5600_STATUS_MD          0x20  // ´ÅÌú¼ì²â
#define AS5600_STATUS_ML          0x10  // ´ÅÌúÌ«Èõ
#define AS5600_STATUS_MH          0x08  // ´ÅÌúÌ«Ç¿

//º¯ÊýÉùÃ÷
void Get_angle(SVPWM_t* svpm , Encoder_t* as5600_data, Motor_t* motor);

uint8_t AS5600_Read_All_Data(Encoder_t* data);
uint16_t AS5600_Read_Angle(void);
uint8_t AS5600_Check_Magnet(Encoder_t* data);
float AS5600_Get_Angle_Degrees(void);
float AS5600_Get_Angle_Radians(void);


void Get_speed(Vel_Pos_t* veclo, Encoder_t* as5600);
void Get_position(Vel_Pos_t* veclo, Encoder_t* as5600);

#endif
