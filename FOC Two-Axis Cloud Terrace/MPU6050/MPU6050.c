#include "headfile.h"

#define MPU6050_ADDRESS		0xD0

#define ALPHA   0.98f
#define RAD2DEG 57.2957795131f


/*WRITE****************************************************************************/
// 指定地址写寄存器
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data) // 8位地址 + 8位数据
{
	MyI2C_Start();                     // 起始
	MyI2C_SendByte(MPU6050_ADDRESS);   // 发送一个字节(从机地址+R/W)
	MyI2C_ReceiveAck();                // 接收应答(从机返回，主机不做接收处理)
	MyI2C_SendByte(RegAddress);        // 发送一个字节(寄存器地址)
	MyI2C_ReceiveAck();                // 接收应答
	MyI2C_SendByte(Data);              // 发送一个字节(数据)
	MyI2C_ReceiveAck();                // 接收应答
	MyI2C_Stop();                      // 发送停止位
}



// 指定地址连续写多个字节
void MPU6050_WriteRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length)
{
  MyI2C_Start();                     // 起始
	MyI2C_SendByte(MPU6050_ADDRESS);   // 发送一个字节(从机地址+R/W)
	MyI2C_ReceiveAck();                // 接收应答(从机返回，主机不做接收处理)
	MyI2C_SendByte(RegAddress);        // 发送一个字节(寄存器地址)
	MyI2C_ReceiveAck();                // 接收应答
    
    for (uint8_t i = 0; i < Length; i++)
    {
        MyI2C_SendByte(Data[i]);            // 发送数据字节
        MyI2C_ReceiveAck();                 // 接收应答
    }
    
    MyI2C_Stop();                           // 发送停止位
}
/****************************************************************************WRITE*/


/*READ****************************************************************************/
// 指定地址读一个字节
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();                     // 起始
	MyI2C_SendByte(MPU6050_ADDRESS);   // 发送一个字节(从机地址+R/W)
	MyI2C_ReceiveAck();                // 接收应答(从机返回，主机不做接收处理)
	MyI2C_SendByte(RegAddress);        // 发送一个字节(寄存器地址)
	MyI2C_ReceiveAck();                // 接收应答
	
	MyI2C_Start();                            // 重复起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);   // 指定从机地址+读写位(R->1)
	MyI2C_ReceiveAck();                       // 接收应答
	Data = MyI2C_ReceiveByte();               // 主机接收从机发送的字节
	MyI2C_SendAck(1);                         // 主机发送NACK
	MyI2C_Stop();                             // 发送停止位
	
	return Data;
}


// 指定地址读多个字节
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint16_t Length)
{
  MyI2C_Start();                     // 起始
 	MyI2C_SendByte(MPU6050_ADDRESS);   // 发送一个字节(从机地址+R/W)
	MyI2C_ReceiveAck();                // 接收应答(从机返回，主机不做接收处理)
	MyI2C_SendByte(RegAddress);        // 发送一个字节(寄存器地址)
	MyI2C_ReceiveAck();                // 接收应答

  MyI2C_Start();                            // 重复起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);   // 指定从机地址+读写位(R->1)
	MyI2C_ReceiveAck();                       // 接收应答
    
    // 循环读取多个字节
    for (uint16_t i = 0; i < Length; i++)
    {
        Data[i] = MyI2C_ReceiveByte();    //主机接收从机发送的字节
   
			  // 如果不是最后一个字节发送ACK ，最后一个字节发送NACK
        if (i < Length - 1)
        {
            MyI2C_SendAck(0); // ACK
        }
        else
        {
            MyI2C_SendAck(1); // NACK
        }
    }
    MyI2C_Stop(); // 发送停止位
}
/****************************************************************************READ*/



void MPU6050_Init(void)
{
	MyI2C_Init();
	
	// 硬件复位
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);      // 复位MPU6050
	Delay_ms(100);                                  // 等待复位完成
	
	// 唤醒设备
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);      // 解除休眠
	
	// 关键配置：开启旁路模式，允许STM32直接访问外部磁力计
	MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02);     // 开启旁路模式
	
	// 关闭不需要的功能
	MPU6050_WriteReg(MPU6050_INT_ENABLE, 0x00);      // 关闭所有中断
	MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00);       // 关闭I2C主模式，关闭FIFO
	MPU6050_WriteReg(MPU6050_FIFO_EN, 0x00);         // 关闭FIFO
	
	// 传感器配置
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);      // 配置电源管理寄存器1，选择时钟源
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);      // 配置电源管理寄存器2，所有传感器工作
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);      // 采样率分频寄存器
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);          // 配置寄存器


		
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);     // 陀螺仪配置：±250 °/s (FS_SEL=0)
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);    // 加速度计配置：±2 g (AFS_SEL=0)
}



// 获取芯片的ID号
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}


void MPU6050_GetData(MPU6050_TypeDef *mpu)
{
    uint8_t DataH, DataL;

    // 读取加速度原始数据
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    mpu->Acc[0] = (int16_t)((DataH << 8) | DataL);

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    mpu->Acc[1] = (int16_t)((DataH << 8) | DataL);

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    mpu->Acc[2] = (int16_t)((DataH << 8) | DataL);

    // 读取陀螺仪原始数据
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    mpu->Gyro[0] = (int16_t)((DataH << 8) | DataL);

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    mpu->Gyro[1] = (int16_t)((DataH << 8) | DataL);

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    mpu->Gyro[2] = (int16_t)((DataH << 8) | DataL);


    for (uint8_t i = 0; i < 3; i++)
    {
        mpu->Accf[i] = (((float)mpu->Acc[i]) * 2 * 9.800665f ) / 32768;          // m/sec^2
        mpu->Gyrof[i] = (((float)mpu->Gyro[i]) / 131.0f) * 0.01745329252f;       //rad/sec       
    }

		if(bias_done)
		{
			GyroBias_Update(mpu,0.002f);
			mpu->Gyrof[x]-=gyro_bias_x+gyro_bias_x_init;
			mpu->Gyrof[y]-=gyro_bias_y+gyro_bias_y_init;
			mpu->Gyrof[z]-=gyro_bias_z+gyro_bias_z_init;
		}

}
