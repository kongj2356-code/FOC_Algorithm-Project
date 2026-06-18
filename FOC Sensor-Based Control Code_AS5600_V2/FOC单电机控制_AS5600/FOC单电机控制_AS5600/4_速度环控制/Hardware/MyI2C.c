#include "stm32f10x.h"
#include "Delay.h"


void I2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
}

void I2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
}

//读取时钟线数据
uint8_t I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);

	return BitValue;
}

void MyI2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_11);
}
	
void I2C_Start(void)
{
	I2C_W_SCL(1);
	I2C_W_SDA(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

void I2C_Stop(void)
{
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);

}

void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	
	for(i=0;i<8;i++)
	{
		I2C_W_SDA(Byte & (0x80>>i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}

}

uint8_t I2C_RecviveData(void)
{
	uint8_t i, Byte = 0x00;
	I2C_W_SDA(1);
	for(i=0;i<8;i++)
	{
		I2C_W_SCL(1);
		if(I2C_R_SDA() == 1) {Byte |= (0x80 >> i);}
		I2C_W_SCL(0);

	}
	return Byte;
	
}

void I2C_SendAck(uint8_t AckBit)
{

		I2C_W_SDA(AckBit);
		I2C_W_SCL(1);
		I2C_W_SCL(0);

}

uint8_t I2C_RecviveAck(void)
{
	uint8_t AckBit;

	I2C_W_SDA(1);
	I2C_W_SCL(1);
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);
	
	return AckBit;
}










