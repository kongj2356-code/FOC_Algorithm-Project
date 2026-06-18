#include "stm32f10x.h"
#include "MyI2C.h"
#include "Serial.h"
#include "Delay.h"
#include <stdio.h>
#include <math.h>

#define AS5600_ADDRESS        0x6C //加上读写位（1位1/0）
#define AS5600_RAW_ANGLE_H    0x0C
#define AS5600_RAW_ANGLE_L    0x0D

#define PI       3.14159265359f
#define _2PI       6.28318530718f


void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	I2C_RecviveAck();
	I2C_SendByte(RegAddress);
	I2C_RecviveAck();
	I2C_SendByte(Data);
	I2C_RecviveAck();
	I2C_Stop();
}

uint8_t AS5600_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	I2C_RecviveAck();
	I2C_SendByte(RegAddress);
	I2C_RecviveAck();
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();
	
	return Data;

}

uint8_t  AS5600_ReadNowReg(void)
{
	uint8_t Data;
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();
	
	return Data;
}

float AS5600_GetRawData(void)
{
	uint8_t Data_L;
	uint8_t Data_H;
	float Raw_Data = 0;
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	I2C_RecviveAck();
	I2C_SendByte(AS5600_RAW_ANGLE_H);
	I2C_RecviveAck();
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data_H = I2C_RecviveData();
	I2C_RecviveAck();
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data_L = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();
	
	Raw_Data = (Data_H << 8) | Data_L;
	
	return Raw_Data;
	
}

void AS5600_Init(void)
{
	MyI2C_Init();
}

float Last_ts = 0.0;
float last_angle = 0.0;
float GetAngle_NoTrack(void)
{
	float Angle = 0.0;

	Angle = (AS5600_GetRawData()/4096) * _2PI;
//	Angle = (Angle/4096) * 360;

	return Angle;
}

float full_rotations = 0.0;
float Last_Angle = 0.0;





float GetAngle(void)
{
	float D_Angle = 0.0;
	float Angle = GetAngle_NoTrack();
	D_Angle = Angle - Last_Angle;
	
	if( fabs(D_Angle) > (0.8f*2*PI) )
	{
		full_rotations = full_rotations + ((D_Angle > 0) ? -1 :1);
	}
	
	Last_Angle = Angle;
	
	return (full_rotations * 2 * PI + Last_Angle);
}



float Last_Vel_ts = 0.0;
float Vel_Last_Angle = 0.0;



float GetVelocity(void)
{
    // 1. 读取当前SysTick计数值和角度
    uint32_t Vel_ts = SysTick->VAL;
    float Vel_Angle = GetAngle(); // 已归一化到0~2π的机械角度
    
    // 2. 计算时间间隔dt（单位：秒）
    float dt = 0.0f;
    if (Vel_ts < Last_Vel_ts)
    {
        // 未溢出：直接计算差值
        dt = (Last_Vel_ts - Vel_ts) / 9.0f * 1e-6f;
    }
    else
    {
        // 溢出：补全溢出部分
        dt = (0xFFFFFF - Vel_ts + Last_Vel_ts) / 9.0f * 1e-6f;
    }
    
    // 3. 修复致命错误：限制dt的最小值（避免除以0），而非设为10000
    if (dt < 1e-6f) // 最小dt=1μs，避免除以0
    {
        dt = 1e-6f;
    }
    
    // 4. 计算角度差（处理跨0点，比如从2π→0的情况）
    float dv = Vel_Angle - Vel_Last_Angle;
    // 角度差归一化到[-π, π]，取最短路径
    while (dv > 3.1415) dv -= 2 * 3.1415;
    while (dv < -3.1415) dv += 2 * 3.1415;
    
    // 5. 计算角速度（rad/s）
    float velocity = dv / dt;
    
    // 6. 低通滤波（可选，让速度更平滑）
    static float Vel_Filtered = 0.0f;
    Vel_Filtered = 0.8f * Vel_Filtered + 0.2f * velocity;
    
    // 7. 更新历史值
    Last_Vel_ts = Vel_ts;
    Vel_Last_Angle = Vel_Angle;
    
    // 返回滤波后的速度（更稳定）
    return Vel_Filtered;
}
