#ifndef __AS5600_H
#define __AS5600_H



// 通道电机宏定义（0=电机1，1=电机2）
#define AS5600_CHANNEL_MOTOR1    1
#define AS5600_CHANNEL_MOTOR2    2


/************************ 核心函数声明（带通道参数，适配两路传感器） ************************/

// 向指定通道（电机）的AS5600寄存器写入数据

void AS5600_WriteReg(uint8_t Channel, uint8_t RegAddress, uint8_t Data);


// 读取指定通道（电机）的AS5600寄存器数据

uint8_t  AS5600_ReadReg(uint8_t Channel, uint8_t RegAddress);



// 读取指定通道（电机）的AS5600实时寄存器数据

uint8_t  AS5600_ReadNowReg(uint8_t Channel);



// 初始化指定通道（电机）的AS5600传感器

void AS5600_Init(uint8_t Channel);



// 获取指定通道（电机）AS5600的累计角度（带旋转圈数跟踪）

float GetAngle(uint8_t Channel);



// 获取指定通道（电机）AS5600的角度（不带旋转圈数跟踪，0~2π）

float GetAngle_NoTrack(uint8_t Channel);



// 获取指定通道（电机）的转速（rad/s）

float GetVelocity(uint8_t Channel);


#endif
