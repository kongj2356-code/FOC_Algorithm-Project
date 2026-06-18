#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"

// 定义I2C通道枚举，明确区分两路I2C（对应两个电机）
typedef enum
{
    I2C_CHANNEL_MOTOR1 = 1,  // 电机1：SCL=PC0, SDA=PC1
    I2C_CHANNEL_MOTOR2 = 2   // 电机2：SCL=PC2, SDA=PC3
} I2C_ChannelTypeDef;

/************************ 对外公开的核心接口 ************************/
// I2C初始化（初始化两路I2C的所有引脚）
void FOC_I2C_Init(void);

/************************ 电机1专用I2C函数 ************************/
void I2C_Motor1_Start(void);
void I2C_Motor1_Stop(void);
void I2C_Motor1_SendByte(uint8_t Byte);
uint8_t I2C_Motor1_RecviveData(void);
void I2C_Motor1_SendAck(uint8_t AckBit);
uint8_t I2C_Motor1_RecviveAck(void);

/************************ 电机2专用I2C函数 ************************/
void I2C_Motor2_Start(void);
void I2C_Motor2_Stop(void);
void I2C_Motor2_SendByte(uint8_t Byte);
uint8_t I2C_Motor2_RecviveData(void);
void I2C_Motor2_SendAck(uint8_t AckBit);
uint8_t I2C_Motor2_RecviveAck(void);

/************************ 通用I2C函数（可选，按通道调用）************************/
// 如需灵活指定通道调用，可使用以下通用函数
void I2C_Start_ByChannel(I2C_ChannelTypeDef Channel);
void I2C_Stop_ByChannel(I2C_ChannelTypeDef Channel);
void I2C_SendByte_ByChannel(I2C_ChannelTypeDef Channel, uint8_t Byte);
uint8_t I2C_RecviveData_ByChannel(I2C_ChannelTypeDef Channel);
void I2C_SendAck_ByChannel(I2C_ChannelTypeDef Channel, uint8_t AckBit);
uint8_t I2C_RecviveAck_ByChannel(I2C_ChannelTypeDef Channel);

#endif
