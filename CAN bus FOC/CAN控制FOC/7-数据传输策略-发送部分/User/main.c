#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "MyCAN.h"

uint8_t KeyNum;
CanRxMsg RxMsg;                         // 定义CAN接收报文结构体，用于存储接收到的CAN数据


// 仅保留：按键触发发送CAN报文结构体 ID:0x200
CanTxMsg TxMsg_Trigger = {
	.StdId = 0x200,
	.ExtId = 0x00000000,
	.IDE = CAN_Id_Standard,
	.RTR = CAN_RTR_Data,
	.DLC = 4,
	.Data = {0x00, 0x00, 0x00, 0x00}
};


// ===================== 4字节转回 float =====================
float ByteToFloat(uint8_t *bytes)
{
    float f;
    uint8_t *p = (uint8_t*)&f;
    p[0] = bytes[0];
    p[1] = bytes[1];
    p[2] = bytes[2];
    p[3] = bytes[3];
    return f;
}


int main(void)
{
	OLED_Init();
	Key_Init();
	MyCAN_Init();
	
  // OLED仅显示按键触发相关内容
	OLED_ShowString(1, 1, "CAN Tx");
	OLED_ShowString(2, 1, "Tri:");
  OLED_ShowString(3, 1, "Speed:"); // 实时电机速度
	
	while (1)
	{
		// 仅保留：按键触发发送功能
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			// 数据自增
			TxMsg_Trigger.Data[0] ++;
			TxMsg_Trigger.Data[1] ++;
			TxMsg_Trigger.Data[2] ++;
			TxMsg_Trigger.Data[3] ++;
			
			// 发送CAN报文
			MyCAN_Transmit(&TxMsg_Trigger);
			
			// OLED显示发送数据
			OLED_ShowHexNum(2, 5, TxMsg_Trigger.Data[0], 2);
			OLED_ShowHexNum(2, 8, TxMsg_Trigger.Data[1], 2);
			OLED_ShowHexNum(2, 11, TxMsg_Trigger.Data[2], 2);
			OLED_ShowHexNum(2, 14, TxMsg_Trigger.Data[3], 2);
		}
    if (KeyNum == 2)
		{
			// 数据自增
			TxMsg_Trigger.Data[0] --;
			TxMsg_Trigger.Data[1] --;
			TxMsg_Trigger.Data[2] --;
			TxMsg_Trigger.Data[3] --;
			
			// 发送CAN报文
			MyCAN_Transmit(&TxMsg_Trigger);
			
			// OLED显示发送数据
			OLED_ShowHexNum(2, 5, TxMsg_Trigger.Data[0], 2);
			OLED_ShowHexNum(2, 8, TxMsg_Trigger.Data[1], 2);
			OLED_ShowHexNum(2, 11, TxMsg_Trigger.Data[2], 2);
			OLED_ShowHexNum(2, 14, TxMsg_Trigger.Data[3], 2);
		}

   		/*------------------- CAN接收处理 -------------------*/
		if (MyCAN_ReceiveFlag())
		{
			MyCAN_Receive(&RxMsg);

			if (RxMsg.RTR == CAN_RTR_Data && RxMsg.StdId == 0x300)
			{
				// 4字节转回真实速度
				float speed = ByteToFloat(RxMsg.Data);
				// OLED 显示浮点数
				OLED_ShowSignedNum(3, 7, speed, sizeof(speed)); 
			}
		}
		}

}
