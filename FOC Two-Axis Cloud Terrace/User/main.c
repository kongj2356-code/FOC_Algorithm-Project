#include "headfile.h"


#define _3PI_2 4.71238898038f
#define _2PI   6.28318530718f
#define _PI    3.14159265358f

/*FOC****************************************/
extern FOC_ControllerTypeDef foc1;
extern FOC_ControllerTypeDef foc2;

float Zero_Angle1;
float Zero_Angle2;
/****************************************FOC*/


/*IMU****************************************/
Mahony_TypeDef mahony;
QMC5883P_Calibration_t qmc;
MPU6050_TypeDef mpu; 

float roll,pitch,yaw,yaw1,yaw2;
uint8_t ID1, ID2;

volatile uint8_t imu_flag = 0;
/****************************************IMU*/


/* 
 * AS5600 Motor1:SCL-PC0、SDA-PC1
 *        Motor2:SCL-PC2、SDA-PC3
 * USART  TX-PB6 ; RX-PB7
 * PWM    Motor1:PA8、PA9、PA10(TIM1)
 *        Motor2:PA6、PA7、PB0 (TIM3)
 * ADC    Motor1:PA0、PA1
 *        Motor2:PA2、PA3
 */       

// FOC初始化集合
static void BSP_FOC_Init(void)
{
    PWM_Init_Double(); // TIM1和TIM3初始化
    AS5600_Init(AS5600_CHANNEL_MOTOR1);
    AS5600_Init(AS5600_CHANNEL_MOTOR2);
    // 获取两路电机零电角
    Zero_Get(MOTOR_1, AS5600_CHANNEL_MOTOR1, &Zero_Angle1);
    Zero_Get(MOTOR_2, AS5600_CHANNEL_MOTOR2, &Zero_Angle2);
    TIM4_Init();         // TIM4初始化
    AD_Init();
    DriftOffsets();
}


/* 
 * MPU6050  :SCL-PB10 SDA-PB11
 * QMC5883P :Expand the I2C interface to connect to the MPU6050.
 */   

// IMU初始化集合
static void BSP_IMU_Init(void)
{
//    OLED_Init();
  	MPU6050_Init();
  	QMC5883P_Init();
  	Mahony_Init(&mahony);
  	QMC5883P_StartCalibration(&qmc); 

//		OLED_ShowString(1,1,"ID1:");
//		ID1 = MPU6050_GetID();
//		OLED_ShowHexNum(1,5,ID1,2);

//		OLED_ShowString(1,8,"ID2:");
//		ID2 = QMC5883P_GetID();
//		OLED_ShowHexNum(1,12,ID2,2);

   for(uint16_t i=0;i<5000;i++)        
   {
	 	 QMC5883P_UpdateCalibration(&qmc);
		  Delay_ms(5);
   }

 	QMC5883P_EndCalibration(&qmc);
	Mahony_GetData(&mahony,&mpu,&qmc,&roll,&pitch,&yaw,&yaw2,0.002f);
	TIM2_Init();         // TIM2初始化 
//  OLED_ShowString(2,1,"Pitch:");
//  OLED_ShowString(3,1,"Roll:");
//  OLED_ShowString(4,1,"Yaw:");     
  LED_Init();
  LED2_ON();
  
}



int main(void)
{     
    USART_Config();
    BSP_IMU_Init();
    BSP_FOC_Init();

    while (1)
    {     
      M1_Set_Position(1.57-(roll*_PI/180)); // 3.34+Serial_RxData  //做自稳定云台只需+ -> - 
      M2_Set_Position(1.57+yaw*_PI/180);  //-3.02+Serial_RxData       0+yaw*_PI/180
      

//  	OLED_ShowSignedNum(2,7,(int16_t)pitch,5);
//    OLED_ShowSignedNum(3,7,(int16_t)roll,5);
//    OLED_ShowSignedNum(4,7,(int16_t)yaw,5);
      /* VOFA+ JustFloat传输协议 */
      JustFloat_Send5(mahony.q0,mahony.q1,mahony.q2,mahony.q3,0);
//     JustFloat_Send3(qmc.m[x],qmc.m[y],qmc.m[z]);
//      JustFloat_Send3(roll, pitch, yaw);

//     printf("%.3f,%.3f,%.3f,%.3f\r\n",mahony.q0,mahony.q1,mahony.q2,mahony.q3);
		// JustFloat_Send5(foc1.samp_volts[0], foc1.samp_volts[1], foc2.samp_volts[0], foc2.samp_volts[1], 0);
		// JustFloat_Send3(_normalizeAngle(GetAngle(1)), _normalizeAngle(GetAngle(2)), 0);
		// JustFloat_Send5(foc1.iq_hope, foc1.ele.Iq, foc1.uq, foc1.speed, foc1.speed_hope);
		// JustFloat_Send5(foc2.iq_hope, foc2.ele.Iq, foc2.uq, foc2.speed, foc2.speed_hope);
   //  JustFloat_Send7(mahony.q0,mahony.q1,mahony.q2,mahony.q3,roll, pitch, yaw);
    }
}


void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
    {
        Mahony_GetData(&mahony,&mpu,&qmc,&roll,&pitch,&yaw,&yaw1,0.002f);

      //  GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);

    }
}


