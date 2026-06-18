#include "stm32f10x.h"
#include "Delay.h"


// 定义两路I2C的引脚结构体，方便管理
typedef struct
{
    uint16_t SCL_Pin;
    uint16_t SDA_Pin;
} I2C_PinsTypeDef;


// 电机1和电机2的I2C引脚配置
I2C_PinsTypeDef I2C_Motor1 = {GPIO_Pin_0, GPIO_Pin_1};
I2C_PinsTypeDef I2C_Motor2 = {GPIO_Pin_2, GPIO_Pin_3};



/************************ 通用I2C底层函数（带引脚参数） ************************/
// 写SCL引脚
void I2C_W_SCL_General(I2C_PinsTypeDef* I2C_Pins, uint8_t BitValue)
{
    GPIO_WriteBit(GPIOC, I2C_Pins->SCL_Pin, (BitAction)BitValue);
}

// 写SDA引脚
void I2C_W_SDA_General(I2C_PinsTypeDef* I2C_Pins, uint8_t BitValue)
{
    GPIO_WriteBit(GPIOC, I2C_Pins->SDA_Pin, (BitAction)BitValue);
}

// 读SDA引脚
uint8_t I2C_R_SDA_General(I2C_PinsTypeDef* I2C_Pins)
{
    return GPIO_ReadInputDataBit(GPIOC, I2C_Pins->SDA_Pin);
}

/************************ 通用I2C协议函数（带引脚参数） ************************/
void I2C_Start_General(I2C_PinsTypeDef* I2C_Pins)
{
    I2C_W_SCL_General(I2C_Pins, 1);
    I2C_W_SDA_General(I2C_Pins, 1);
    I2C_W_SDA_General(I2C_Pins, 0);
    I2C_W_SCL_General(I2C_Pins, 0);
}

void I2C_Stop_General(I2C_PinsTypeDef* I2C_Pins)
{
    I2C_W_SDA_General(I2C_Pins, 0);
    I2C_W_SCL_General(I2C_Pins, 1);
    I2C_W_SDA_General(I2C_Pins, 1);
}

void I2C_SendByte_General(I2C_PinsTypeDef* I2C_Pins, uint8_t Byte)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        I2C_W_SDA_General(I2C_Pins, Byte & (0x80 >> i));
        I2C_W_SCL_General(I2C_Pins, 1);
        I2C_W_SCL_General(I2C_Pins, 0);
    }
}

uint8_t I2C_RecviveData_General(I2C_PinsTypeDef* I2C_Pins)
{
    uint8_t i, Byte = 0x00;
    I2C_W_SDA_General(I2C_Pins, 1);
    for(i=0; i<8; i++)
    {
        I2C_W_SCL_General(I2C_Pins, 1);
        if(I2C_R_SDA_General(I2C_Pins) == 1) 
        {
            Byte |= (0x80 >> i);
        }
        I2C_W_SCL_General(I2C_Pins, 0);
    }
    return Byte;
}

void I2C_SendAck_General(I2C_PinsTypeDef* I2C_Pins, uint8_t AckBit)
{
    I2C_W_SDA_General(I2C_Pins, AckBit);
    I2C_W_SCL_General(I2C_Pins, 1);
    I2C_W_SCL_General(I2C_Pins, 0);
}


uint8_t I2C_RecviveAck_General(I2C_PinsTypeDef* I2C_Pins)
{
    uint8_t AckBit;
    I2C_W_SDA_General(I2C_Pins, 1);
    I2C_W_SCL_General(I2C_Pins, 1);
    AckBit = I2C_R_SDA_General(I2C_Pins);
    I2C_W_SCL_General(I2C_Pins, 0);
    return AckBit;
}



/************************ 电机1专用I2C函数（对外接口） ************************/
void I2C_Motor1_Start(void)              { I2C_Start_General(&I2C_Motor1); }
void I2C_Motor1_Stop(void)               { I2C_Stop_General(&I2C_Motor1); }
void I2C_Motor1_SendByte(uint8_t Byte)   { I2C_SendByte_General(&I2C_Motor1, Byte); }
uint8_t I2C_Motor1_RecviveData(void)     { return I2C_RecviveData_General(&I2C_Motor1); }
void I2C_Motor1_SendAck(uint8_t AckBit)  { I2C_SendAck_General(&I2C_Motor1, AckBit); }
uint8_t I2C_Motor1_RecviveAck(void)      { return I2C_RecviveAck_General(&I2C_Motor1); }



/************************ 电机2专用I2C函数（对外接口） ************************/
void I2C_Motor2_Start(void)              { I2C_Start_General(&I2C_Motor2); }
void I2C_Motor2_Stop(void)               { I2C_Stop_General(&I2C_Motor2); }
void I2C_Motor2_SendByte(uint8_t Byte)   { I2C_SendByte_General(&I2C_Motor2, Byte); }
uint8_t I2C_Motor2_RecviveData(void)     { return I2C_RecviveData_General(&I2C_Motor2); }
void I2C_Motor2_SendAck(uint8_t AckBit)  { I2C_SendAck_General(&I2C_Motor2, AckBit); }
uint8_t I2C_Motor2_RecviveAck(void)      { return I2C_RecviveAck_General(&I2C_Motor2); }




/************************ I2C初始化函数（初始化所有引脚） ************************/
void FOC_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    // 配置PC0/PC1/PC2/PC3为开漏输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 初始化所有I2C引脚为高电平（修复原代码引脚写错的问题：Pin10/Pin11→正确的Pin0-3）
    GPIO_SetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

