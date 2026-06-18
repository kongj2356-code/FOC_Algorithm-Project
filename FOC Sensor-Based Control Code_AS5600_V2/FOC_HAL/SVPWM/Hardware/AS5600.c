#include "stm32f1xx_hal.h"
#include "MyI2C.h"
#include <stdio.h>
#include <math.h>

#define AS5600_ADDRESS        0x6C // 设备地址（不含读写位）
#define AS5600_RAW_ANGLE_H    0x0C
#define AS5600_RAW_ANGLE_L    0x0D

#define PI       3.14159265359f
#define _2PI     6.28318530718f

/**
  * @brief  向AS5600寄存器写入数据
  * @param  RegAddress: 寄存器地址
  * @param  Data: 要写入的数据
  * @retval 无
  */
void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS);  // HAL版I2C地址需左移1位（腾出读写位）
    I2C_RecviveAck();
    I2C_SendByte(RegAddress);
    I2C_RecviveAck();
    I2C_SendByte(Data);
    I2C_RecviveAck();
    I2C_Stop();
}

/**
  * @brief  读取AS5600单个寄存器数据
  * @param  RegAddress: 寄存器地址
  * @retval 读取到的寄存器数据
  */
uint8_t AS5600_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    
    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS);  // 写操作
    I2C_RecviveAck();
    I2C_SendByte(RegAddress);
    I2C_RecviveAck();
    
    I2C_Start();
    I2C_SendByte((AS5600_ADDRESS) | 0x01);  // 读操作（地址左移+读位）
    I2C_RecviveAck();
    Data = I2C_RecviveData();
    I2C_SendAck(1);  // 发送非应答
    I2C_Stop();
    
    return Data;
}

/**
  * @brief  读取AS5600当前寄存器数据（简化版）
  * @retval 读取到的数据
  */
uint8_t AS5600_ReadNowReg(void)
{
    uint8_t Data;
    
    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS | 0x01);  // 读操作
    I2C_RecviveAck();
    Data = I2C_RecviveData();
    I2C_SendAck(1);
    I2C_Stop();
    
    return Data;
}

/**
  * @brief  获取AS5600原始角度数据
  * @retval 原始角度值（0~4095）
  */
float AS5600_GetRawData(void)
{
    uint8_t Data_L;
    uint8_t Data_H;
    float Raw_Data = 0;
    
    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS);  // 写操作
    I2C_RecviveAck();
    I2C_SendByte(AS5600_RAW_ANGLE_H);
    I2C_RecviveAck();
    
    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS| 0x01);  // 读高位
    I2C_RecviveAck();
    Data_H = I2C_RecviveData();
    I2C_RecviveAck();
    
    I2C_Start();
    I2C_SendByte(AS5600_ADDRESS | 0x01);  // 读低位
    I2C_RecviveAck();
    Data_L = I2C_RecviveData();
    I2C_SendAck(1);  // 发送非应答
    I2C_Stop();
    
    Raw_Data = (Data_H << 8) | Data_L;
    
    return Raw_Data;
}

/**
  * @brief  AS5600初始化
  * @retval 无
  */
void AS5600_Init(void)
{
    MyI2C_Init();  // 调用HAL版I2C初始化
}

float Last_ts = 0.0;
float last_angle = 0.0;

/**
  * @brief  获取无累计的角度值（0~2π）
  * @retval 角度值（弧度）
  */
float GetAngle_NoTrack(void)
{
    float Angle = 0.0;
    Angle = (AS5600_GetRawData()/4096) * _2PI;
    return Angle;
}



float full_rotations = 0.0;
float Last_Angle = 0.0;


/**
  * @brief  获取带累计的角度值（支持多圈）
  * @retval 角度值（弧度）
  */
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


/**
  * @brief  计算角速度（rad/s）
  * @retval 角速度值
  */
float GetVelocity(void)
{
    // 1. 读取当前SysTick计数值和角度（HAL库SysTick接口）
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
    
    // 3. 限制dt的最小值（避免除以0）
    if (dt < 1e-6f)
    {
        dt = 1e-6f;
    }
    
    // 4. 计算角度差（处理跨0点）
    float dv = Vel_Angle - Vel_Last_Angle;
    while (dv > 3.1415) dv -= 2 * 3.1415;
    while (dv < -3.1415) dv += 2 * 3.1415;
    
    // 5. 计算角速度（rad/s）
    float velocity = dv / dt;
    
    // 6. 低通滤波（可选）
    static float Vel_Filtered = 0.0f;
    Vel_Filtered = 0.8f * Vel_Filtered + 0.2f * velocity;
    
    // 7. 更新历史值
    Last_Vel_ts = Vel_ts;
    Vel_Last_Angle = Vel_Angle;
    
    // 返回滤波后的速度
    return Vel_Filtered;
}
