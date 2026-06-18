#include "stm32f10x.h"
#include "MyI2C.h"
#include "Serial.h"
#include <stdio.h>
#include <math.h>

#define AS5600_ADDRESS        0x6C //加上读写位（1位1/0）
#define AS5600_RAW_ANGLE_H    0x0C
#define AS5600_RAW_ANGLE_L    0x0D

#define PI         3.14159265359f
#define _2PI       6.28318530718f


// 定义两路传感器的独立状态结构体（避免数据串扰）
typedef struct 
{
    float Last_ts;
    float last_angle;
    float full_rotations;
    float Last_Angle;
    float Last_Vel_ts;
    float Vel_Last_Angle;
} AS5600_StateTypeDef;


// 初始化两路传感器的状态变量
AS5600_StateTypeDef AS5600_State[2] = {0};



/************************ 核心I2C操作函数（带通道参数） ************************/
void AS5600_WriteReg(uint8_t Channel, uint8_t RegAddress, uint8_t Data)
{
    // 根据通道选择对应电机的I2C函数
    switch(Channel)
    {
        case 1:  // 电机1
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS);
            I2C_Motor1_RecviveAck();
            I2C_Motor1_SendByte(RegAddress);
            I2C_Motor1_RecviveAck();
            I2C_Motor1_SendByte(Data);
            I2C_Motor1_RecviveAck();
            I2C_Motor1_Stop();
            break;
            
        case 2:  // 电机2
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS);
            I2C_Motor2_RecviveAck();
            I2C_Motor2_SendByte(RegAddress);
            I2C_Motor2_RecviveAck();
            I2C_Motor2_SendByte(Data);
            I2C_Motor2_RecviveAck();
            I2C_Motor2_Stop();
            break;
            
        default: // 防呆：非法通道不操作
            break;
    }
}

uint8_t AS5600_ReadReg(uint8_t Channel, uint8_t RegAddress)
{
    uint8_t Data = 0;
    
    switch(Channel)
    {
        case 1:  // 电机1
            // 写寄存器地址
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS);
            I2C_Motor1_RecviveAck();
            I2C_Motor1_SendByte(RegAddress);
            I2C_Motor1_RecviveAck();
            
            // 读数据
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor1_RecviveAck();
            Data = I2C_Motor1_RecviveData();
            I2C_Motor1_SendAck(1);
            I2C_Motor1_Stop();
            break;
            
        case 2:  // 电机2
            // 写寄存器地址
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS);
            I2C_Motor2_RecviveAck();
            I2C_Motor2_SendByte(RegAddress);
            I2C_Motor2_RecviveAck();
            
            // 读数据
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor2_RecviveAck();
            Data = I2C_Motor2_RecviveData();
            I2C_Motor2_SendAck(1);
            I2C_Motor2_Stop();
            break;
            
        default: // 防呆：非法通道返回0
            Data = 0;
            break;
    }
    
    return Data;
}

uint8_t AS5600_ReadNowReg(uint8_t Channel)
{
    uint8_t Data = 0;
    
    switch(Channel)
    {
        case 1:  // 电机1
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor1_RecviveAck();
            Data = I2C_Motor1_RecviveData();
            I2C_Motor1_SendAck(1);
            I2C_Motor1_Stop();
            break;
            
        case 2:  // 电机2
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor2_RecviveAck();
            Data = I2C_Motor2_RecviveData();
            I2C_Motor2_SendAck(1);
            I2C_Motor2_Stop();
            break;
            
        default: // 防呆：非法通道返回0
            Data = 0;
            break;
    }
    
    return Data;
}

float AS5600_GetRawData(uint8_t Channel)
{
    uint8_t Data_L = 0;
    uint8_t Data_H = 0;
    float Raw_Data = 0;
    
    switch(Channel)
    {
        case 1:  // 电机1
            // 写寄存器地址（高字节）
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS);
            I2C_Motor1_RecviveAck();
            I2C_Motor1_SendByte(AS5600_RAW_ANGLE_H);
            I2C_Motor1_RecviveAck();
            
            // 读高字节
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor1_RecviveAck();
            Data_H = I2C_Motor1_RecviveData();
            I2C_Motor1_RecviveAck();
            
            // 读低字节
            I2C_Motor1_Start();
            I2C_Motor1_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor1_RecviveAck();
            Data_L = I2C_Motor1_RecviveData();
            I2C_Motor1_SendAck(1);  // 读完，发送ACK=1
            I2C_Motor1_Stop();
            break;
            
        case 2:  // 电机2
            // 写寄存器地址（高字节）
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS);
            I2C_Motor2_RecviveAck();
            I2C_Motor2_SendByte(AS5600_RAW_ANGLE_H);
            I2C_Motor2_RecviveAck();
            
            // 读高字节
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor2_RecviveAck();
            Data_H = I2C_Motor2_RecviveData(); 
            
            // 读低字节
            I2C_Motor2_Start();
            I2C_Motor2_SendByte(AS5600_ADDRESS | 0x01);
            I2C_Motor2_RecviveAck();
            Data_L = I2C_Motor2_RecviveData();
            I2C_Motor2_SendAck(1);  // 读完，发送ACK=1
            I2C_Motor2_Stop();
            break;
            
        default: // 防呆：非法通道返回0
            Data_H = 0;
            Data_L = 0;
            break;
    }
    
    Raw_Data = (Data_H << 8) | Data_L;
    return Raw_Data;
}


/************************ 初始化与角度/速度获取函数 ************************/
void AS5600_Init(uint8_t Channel)
{
    MyI2C_Init();  // 初始化两路I2C引脚（PC0-PC3）
    // 重置对应通道的状态变量
    AS5600_State[Channel].Last_ts = 0.0;
    AS5600_State[Channel].last_angle = 0.0;
    AS5600_State[Channel].full_rotations = 0.0;
    AS5600_State[Channel].Last_Angle = 0.0;
    AS5600_State[Channel].Last_Vel_ts = 0.0;
    AS5600_State[Channel].Vel_Last_Angle = 0.0;
}

// 获取无跟踪的角度（0~2π）
float GetAngle_NoTrack(uint8_t Channel)
{
    float Angle = 0.0;
    Angle = (AS5600_GetRawData(Channel)/4096) * _2PI;
    AS5600_State[Channel].last_angle = Angle;
    return Angle;
}


// 获取带旋转圈数跟踪的累计角度
float GetAngle(uint8_t Channel)
{
    float D_Angle = 0.0;
    float Angle = GetAngle_NoTrack(Channel);
    D_Angle = Angle - AS5600_State[Channel].Last_Angle;
    
    // 判断是否跨0点旋转
    if( fabs(D_Angle) > (0.8f*2*PI) )
    {
        AS5600_State[Channel].full_rotations = AS5600_State[Channel].full_rotations + ((D_Angle > 0) ? -1 : 1);
    }
    
    AS5600_State[Channel].Last_Angle = Angle;
    
    return (AS5600_State[Channel].full_rotations * 2 * PI + AS5600_State[Channel].Last_Angle);
}




// 获取转速（rad/s）
float GetVelocity(uint8_t Channel)
{
    float dt = 0.0;
    float Vel_ts = SysTick -> VAL;
    
    // 计算时间差
    if(Vel_ts < AS5600_State[Channel].Last_Vel_ts) 
    {
        dt = (AS5600_State[Channel].Last_Vel_ts - Vel_ts)/9*1e-6f;
    }
    else 
    {
        dt = (0xFFFFFF - Vel_ts + AS5600_State[Channel].Last_Vel_ts)/9*1e-6f;
    }
    
    // 防止除零错误
    if(dt < 0.0001) dt = 0.0001;  // 修复原代码的dt=10000错误
    
    float Vel_Angle = GetAngle(Channel);
    float dv = Vel_Angle - AS5600_State[Channel].Vel_Last_Angle;
    float velocity = dv / dt;

    // 更新状态
    AS5600_State[Channel].Last_Vel_ts = Vel_ts;
    AS5600_State[Channel].Vel_Last_Angle = Vel_Angle;
    
    return velocity;
}
