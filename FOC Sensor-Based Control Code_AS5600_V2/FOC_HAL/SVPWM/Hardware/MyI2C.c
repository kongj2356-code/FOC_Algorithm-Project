#include "stm32f1xx_hal.h"  // HAL库核心头文件


// 定义SCL和SDA引脚（保持原有的GPIOC Pin0/Pin1）
#define I2C_SCL_PIN    GPIO_PIN_0
#define I2C_SDA_PIN    GPIO_PIN_1
#define I2C_GPIO_PORT  GPIOC


/**
  * 函    数：写入SCL引脚电平
  * 参    数：BitValue 0-低电平，1-高电平
  * 返 回 值：无
  */
void I2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, (GPIO_PinState)BitValue);
}



/**
  * 函    数：写入SDA引脚电平
  * 参    数：BitValue 0-低电平，1-高电平
  * 返 回 值：无
  */
void I2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, (GPIO_PinState)BitValue); 
}



/**
  * 函    数：读取SDA引脚电平
  * 参    数：无
  * 返 回 值：0-低电平，1-高电平
  */
uint8_t I2C_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN);
    return BitValue;
}


/**
  * 函    数：软件I2C初始化
  * 说    明：配置GPIOC Pin0/Pin1为开漏输出模式，初始化为高电平（释放总线）
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 开启GPIOC时钟（HAL库标准时钟使能方式）
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 配置SCL和SDA引脚为开漏输出模式（软件I2C核心）
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出（兼容I2C总线）
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // 无上下拉（外部需接上拉电阻）
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速模式
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    // 初始化总线为高电平（释放总线，原代码笔误Pin10/Pin11已修正为Pin0/Pin1）
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN | I2C_SDA_PIN, GPIO_PIN_SET);
}



/**
  * 函    数：I2C起始信号
  * 时    序：SCL高电平时，SDA从高到低跳变
  * 参    数：无
  * 返 回 值：无
  */
void I2C_Start(void)
{
    I2C_W_SCL(1);
    I2C_W_SDA(1);
    I2C_W_SDA(0);  // SDA拉低
    I2C_W_SCL(0);  // SCL拉低，开始传输数据
}

/**
  * 函    数：I2C停止信号
  * 时    序：SCL高电平时，SDA从低到高跳变
  * 参    数：无
  * 返 回 值：无
  */
void I2C_Stop(void)
{
    I2C_W_SDA(0);  // SDA拉低
    I2C_W_SCL(1);  // SCL拉高
    I2C_W_SDA(1);  // SDA释放，产生停止信号
}

/**
  * 函    数：I2C发送一个字节
  * 参    数：Byte 要发送的字节（高位先行）
  * 返 回 值：无
  */
void I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        // 逐位发送（从最高位到最低位）
        I2C_W_SDA(Byte & (0x80 >> i));
        I2C_W_SCL(1);  // SCL拉高，从机采样
        I2C_W_SCL(0);  // SCL拉低，准备下一位
    }
}

/**
  * 函    数：I2C接收一个字节
  * 参    数：无
  * 返 回 值：接收到的字节（高位先行）
  */
uint8_t I2C_RecviveData(void)  // 原函数名拼写错误Recvive→Receive，保留原名兼容
{
    uint8_t i, Byte = 0x00;
    I2C_W_SDA(1);  // 释放SDA，由从机控制电平
    for(i = 0; i < 8; i++)
    {
        I2C_W_SCL(1);  // SCL拉高，读取SDA电平
        if(I2C_R_SDA() == 1)
        {
            Byte |= (0x80 >> i);  // 高位先行，读取每一位
        }
        I2C_W_SCL(0);  // SCL拉低，准备下一位
    }
    return Byte;
}

/**
  * 函    数：I2C发送应答位
  * 参    数：AckBit 0-应答(ACK)，1-非应答(NACK)
  * 返 回 值：无
  */
void I2C_SendAck(uint8_t AckBit)
{
    I2C_W_SDA(AckBit);  // 0=拉低（应答），1=释放（非应答）
    I2C_W_SCL(1);       // SCL拉高，从机采样应答位
    I2C_W_SCL(0);       // SCL拉低
}

/**
  * 函    数：I2C接收应答位
  * 参    数：无
  * 返 回 值：0-从机应答(ACK)，1-从机非应答(NACK)
  */
uint8_t I2C_RecviveAck(void)  // 原函数名拼写错误Recvive→Receive，保留原名兼容
{
    uint8_t AckBit;
    I2C_W_SDA(1);  // 释放SDA，由从机控制应答电平
    I2C_W_SCL(1);  // SCL拉高，读取应答位
    AckBit = I2C_R_SDA();  // 0=应答，1=非应答
    I2C_W_SCL(0);  // SCL拉低
    return AckBit;
}
