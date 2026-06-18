#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "Delay.h"	
#include "./FOC/bsp_foc.h"
#include "./ADC/bsp_adc.h"
#include "OLED.h"
#include "MyCAN.h"


#define _3PI_2  4.71238898038
#define _2PI    6.28318530718

// 声明外部总控制器结构体（关键！）
extern FOC_ControllerTypeDef foc;

float Zero_Angle;

void BSP_Init()
{
	USART_Config();
	AS5600_Init();
	PWM_Init();
  BSP_TIM();
}


void Zero_Get()
{
	float Angle;
	for(int i =0; i <200;i++)
	{
		Angle = _3PI_2 + _2PI * i / 200.0;
		setPhaseVoltage_FOC2(7, 0,  Angle);
		Delay_ms(1);
	}
	for(int i=200; i>=0; i--) //再转回去
	{
		Angle = _3PI_2 + _2PI * i / 200.0 ;
		setPhaseVoltage_FOC2(7, 0,  Angle);
		Delay_ms(5);
	}
	Zero_Angle = GetAngle() * 7;
  Delay_ms(20);
}


/******************************************************/
CanTxMsg TxMsg_Speed = {
	.StdId = 0x300,          // 发送ID：0x300（你可随便改）
	.ExtId = 0x00000000,
	.IDE = CAN_Id_Standard,
	.RTR = CAN_RTR_Data,
	.DLC = 4,                // 浮点数 = 4字节
	.Data = {0}
};

// 工具函数：把 float 转成 4 个字节（用于CAN发送）
void Float_to_Byte(float f, uint8_t *byteArr)
{
    uint8_t *p = (uint8_t*)&f;
    byteArr[0] = p[0];
    byteArr[1] = p[1];
    byteArr[2] = p[2];
    byteArr[3] = p[3];
}

CanRxMsg RxMsg;                         // 定义CAN接收报文结构体，用于存储接收到的CAN数据

/******************************************************/

int main(void)
{
	BSP_Init();
	AD_Init();
  DriftOffsets();
	Zero_Get();
  OLED_Init();
	MyCAN_Init();
	OLED_ShowString(1, 1, "CAN Rx");
	OLED_ShowString(2, 1, "Tri:");
	while (1)
	{  
    if (MyCAN_ReceiveFlag())        // 判断是否接收到新的CAN报文
		{
			MyCAN_Receive(&RxMsg);        // 将接收到的报文读取到RxMsg结构体中
			
			// 仅处理数据帧
			if (RxMsg.RTR == CAN_RTR_Data)
			{
	
				/* 接收并显示：ID=0x200 按键触发数据帧 */
				if (RxMsg.StdId == 0x200 && RxMsg.IDE == CAN_Id_Standard)
				{
					OLED_ShowHexNum(2, 5, RxMsg.Data[0], 2);
					OLED_ShowHexNum(2, 8, RxMsg.Data[1], 2);
					OLED_ShowHexNum(2, 11, RxMsg.Data[2], 2);
					OLED_ShowHexNum(2, 14, RxMsg.Data[3], 2);
        
				}
				
			}
		}

        M1_Set_Velocity(RxMsg.Data[0]);
    
        // ===================== 核心：发送 foc.speed =====================
        Float_to_Byte(foc.speed, TxMsg_Speed.Data);  // 速度 → 4字节
        MyCAN_Transmit(&TxMsg_Speed);                // CAN发送
        // ================================================================

        // VOFA+ JustFloat传输协议
       // JustFloat_Send3(foc.samp_volts[0], foc.samp_volts[1], 0);
       //  JustFloat_Send3(foc.ele.I_alpha, foc.ele.I_beta, 0);
        //   JustFloat_Send3(foc.ele.Id, foc.ele.Iq, 0);
       //  JustFloat_Send5(foc.iq_hope, foc.ele.Iq, foc.uq, foc.speed, foc.speed_hope);
        //   JustFloat_Send3( foc.ele.Ia, foc.ele.Ib, foc.ele.Ic);
        //  printf("%f,%f,%f,%f,%f,%f,%f\r\n", foc.iq_hope, foc.ele.Iq, foc.id_hope, foc.ele.Id, Serial_RxData, foc.uq, foc.speed);
        // printf("%f,%f,%f\r\n", foc.ele.Ia, foc.ele.Ib, foc.ele.Ic);
        // printf("%d,%d\r\n", foc.samp_volts[0], foc.samp_volts[1]);

	}
}
