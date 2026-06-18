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
	float dt = 0.0;
	float Vel_ts = SysTick -> VAL;
	if(Vel_ts < Last_Vel_ts) dt = (Last_Vel_ts - Vel_ts)/9*1e-6f;
	else dt = (0xFFFFFF - Vel_ts + Last_Vel_ts)/9*1e-6f;
	
	if(dt < 0.0001) dt = 10000;
	
	float Vel_Angle = GetAngle();
	
	float dv = Vel_Angle - Vel_Last_Angle;

	float velocity = (Vel_Angle - Vel_Last_Angle)/dt;

	Last_Vel_ts = Vel_ts;
  Vel_Last_Angle = Vel_Angle;
  
	
  return velocity;
}

