#include "Control.h"
#include "FOC_Algorithm.h"
#include "Upper.h"
#include "FOC_PID.h"
#include "FOC_math_utils.h"

/************************ 全局结构体变量定义 ************************/
SVPWM_t 	  Svpwm;        // SVPWM 相关参数结构体
Cal_Cur_t 	Cur_Cal;      // 电流采集与偏置校准结构体
Motor_t 	  My_motor;     // 电机本体参数结构体
Encoder_t 	AS5600;       // AS5600 编码器结构体
PID_t 		  Cur_Uq;       // Q 轴电流 PID 控制器
PID_t 		  Cur_Ud;       // D 轴电流 PID 控制器
Vel_Pos_t 	Vec_pos_data; // 速度与位置计算数据结构体
PID_t 		  Vel_PI;       // 速度环 PID 控制器
PID_t 		  Pos_PI;       // 位置环 PID 控制器
Filter_t   	LP_filter;    // 低通滤波器结构体


/************************ 全局枚举变量定义 ************************/
Mode_t 		Operate_mode;  // 电机运行模式枚举（电流模式、速度模式、位置模式）


/************************ 串口通信变量 ************************/
uint8_t rx_buffer[128];     // 串口接收缓冲区
float SendArray[3];         // 串口发送数据数组（3个float数据）


/**
  * @brief  串口发送数组初始化：配置要上传的监测数据
  * @param  无
  * @retval 无
  */
void Sendarray_Init(void)
{
    // 可根据调试需求切换注释，选择发送不同物理量
//     SendArray[0] = Svpwm.Ia;       // 发送 A 相电流
//     SendArray[1] = Svpwm.Ib;       // 发送 B 相电流
//     SendArray[2] = Svpwm.Ic;       // 发送 C 相电流

//     SendArray[0] = Svpwm.Tcomp1;       
//     SendArray[1] = Svpwm.Tcomp2;       
//     SendArray[2] = Svpwm.Tcomp3;      
     
     SendArray[0] = Svpwm.elec_angle; // 发送电角度
     SendArray[1] = Vec_pos_data.speed;// 发送电机转速
     SendArray[2] = Svpwm.Vq;         // 发送 Q 轴电压

//    SendArray[0] = AS5600.mec_angle;                 // 发送机械角度
//    SendArray[1] = Vec_pos_data.speed;               // 发送电机转速
//    SendArray[2] = My_motor.zero_encoder_angle;      // 零偏角
}



/**
  * @brief  LED 指示灯控制函数
  * @param  which: 灯号 1/2
  * @param  state: 1=点亮 0=熄灭（低电平点亮）
  * @retval 无
  */
void lighton(uint8_t which, uint8_t state)
{
    if(which == 1)
    {
        if(state == 1)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // 低电平点亮
        else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // 高电平熄灭
    }
    if(which == 2)
    {
        if(state == 1)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    }
}




/**
  * @brief  电流采样 ADC 直流偏置电压校准（去除硬件零漂）
  * @note   电机静止时，采集两相电流采样电路的零输入电压平均值
  *         得到 DC_bias1/DC_bias2（单位：V），用于后续电流计算补偿
  * @param  svp: SVPWM 结构体指针
  * @param  Cur: 电流校准结构体指针
  * @retval 无
  */
void Cur_bais_calibration(SVPWM_t* svp, Cal_Cur_t* Cur)
{
    svp->cur_count++;

    if(!svp->curbias_done) // 判断电流校准标志位，未完成时进行校准
    {
        stop_motor();      // 校准期间停止电机输出

        if(svp->cur_count < Cur->samples) // 累计采样
        {
            // 读取注入组 ADC 采样值
            Cur->adc_Value1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            Cur->adc_Value2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

            // 软件手动微调偏置补偿
            Cur->adc_Value1 += 8.0f;
            Cur->adc_Value2 += 7.2f;

            // 累加求和
            Cur->sum1 += Cur->adc_Value1;
            Cur->sum2 += Cur->adc_Value2;
        }
        else // 采样完成，计算直流偏置
        {
            Cur->sum1 *= ADC_TO_VOLTAGE;
            Cur->sum2 *= ADC_TO_VOLTAGE;

            // 求平均得到静态偏置(V)
            Cur->DC_bias1 = Cur->sum1 / Cur->samples;
            Cur->DC_bias2 = Cur->sum2 / Cur->samples;

            svp->curbias_done = 1; // 标记电流校准完成
        }
    }
}



/**
  * @brief  电机FOC运行主函数（中断中调用）
  * @param  无
  * @retval 无
  */
void Run_Motor(void)
{
    // 电流校准完成后才允许运行电机
    if(Svpwm.curbias_done)
    {
        // 运行状态指示灯翻转
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

        // 1. 读取两相电流 ADC 原始值
        Svpwm.ad1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        Svpwm.ad2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        // 2. 获取编码器角度 + 计算电角度
        Get_angle(&Svpwm, &AS5600, &My_motor);
        Svpwm.elec_angle = _normalizeAngle(Svpwm.elec_angle); // 电角度归一化

        // 3. ADC 值转电压，再计算实际相电流
        Svpwm.AD_A = (float)Svpwm.ad1 * (float)ADC_TO_VOLTAGE;
        Svpwm.AD_B = (float)Svpwm.ad2 * (float)ADC_TO_VOLTAGE;
         
        Svpwm.Ia = -(Svpwm.AD_A - Cur_Cal.DC_bias1)*2.0F;
        Svpwm.Ib = -(Svpwm.AD_B - Cur_Cal.DC_bias2)*2.0F;
        Svpwm.Ic = -Svpwm.Ia - Svpwm.Ib; // 三相电流和为 0

        // 4. 三相电流软件低通滤波
        LP_IaIbIc(&LP_filter, &Svpwm);

        // 5. FOC 坐标变换
        clarke(&Svpwm);                   // Clarke 变换（3相 -> 2相静止）
        park(&Svpwm, &Cur_Uq, &Cur_Ud);   // Park 变换  （2相静止 -> 2相旋转）

        // ===================== 闭环控制逻辑 =====================
        //只有速度环与位置环才是需要进行处理的,毕竟电流环无论如何都需要计算
		    //位置环计算一定要放在速度环计算之前进行

        // 速度环模式
        if(Operate_mode == velocity_mode)
        {
            Svpwm.speed_loop_count++;
            // 每 5 个电流环周期执行一次速度环（降频计算）
            if(Svpwm.speed_loop_count >= 5)
            {
                Svpwm.speed_loop_count = 0;
                Get_speed(&Vec_pos_data, &AS5600);        // 计算转速

                Vel_PI.Fbk = Vec_pos_data.speed;          // 速度反馈
                PID_V();                                  // 速度环 PID 计算
                Cur_Uq.Ref = Vel_PI.Out;                  // 速度环输出 -> 电流环给定
            }
        }

        // 位置环模式（位置环 -> 速度环 -> 电流环）
        else if(Operate_mode == position_mode)
        {
            Svpwm.position_loop_count++;

            // 全局计数清零（上电初始化一次）
            if(Svpwm.global_count)
            {
                Svpwm.global_count = 0;
                Vec_pos_data.position = 0;
                Vec_pos_data.last_position = 0;
                Vec_pos_data.last_speed = 0;
                Vec_pos_data.now_angle = 0;
                Vec_pos_data.speed = 0;
            }

            // 每 50 个电流环周期执行一次位置环
            if(Svpwm.position_loop_count >= 50)
            {
                Svpwm.position_loop_count = 0;
                Get_position(&Vec_pos_data, &AS5600);      // 计算位置

                Pos_PI.Fbk = Vec_pos_data.position;        // 位置反馈
                PID_P();                                   // 位置环 PID
                Vel_PI.Ref = Pos_PI.Out;                   // 位置环输出 -> 速度环给定

                PID_V();                                   // 速度环 PID
                Cur_Uq.Ref = Vel_PI.Out;                   // 速度环输出 -> 电流环给定
            }
        }

        // 6. 基础的电流环计算
        PID_C();

        // 7. 反坐标变换
        reverse_park(&Svpwm);    // 反 Park
        reverse_clarke(&Svpwm);  // 反 Clarke

        // 8. SVPWM 计算与输出
        SVPWM_Calculate(&Svpwm);

        // 9. 数据打包并通过串口发送
        Sendarray_Init();
        Float_send(SendArray, 3);
    }
}


/***************************************结构体初始化子函数***************************************/

/**
  * @brief  电机本体参数初始化
  */
static void Init_Motor_Params(Motor_t* motor)
{
    motor->zero_encoder_angle = 0.0f;    // 编码器零点偏移角度
    motor->wire_inductor = 0.0028f;       // 绕组电感
    motor->Phase_inductor = 0.00086f;     // 相电感
    motor->Flux = 0.0035f;                // 磁链
    motor->wire_res = 5.1f;               // 绕组电阻
    motor->phase_res = 2.55f;             // 相电阻
}

/**
  * @brief  电流校准参数初始化
  */
static void Init_Current_Calibration(Cal_Cur_t* cur_cal)
{
    cur_cal->samples = 200;       // 校准采样次数
    cur_cal->sum1 = 0;            // A 相采样和
    cur_cal->sum2 = 0;            // B 相采样和
    cur_cal->sum3 = 0;            // 备用
}


/**
  * @brief  SVPWM 相关参数初始化
  */
static void Init_SVPWM(SVPWM_t* svp)
{
    svp->Vdc = 12.0f;                // 母线电压
    svp->pole_pairs = 7;             // 电机极对数
    svp->Ts = 1.0f;                  // 控制周期
    svp->zerobias_done = 0;          // 零点偏置完成标志
    svp->curbias_done = 0;           // 电流偏置校准完成标志
    svp->cur_count = 0;              // 电流校准计数器
    svp->speed_loop_count = 0;       // 速度环计数器
    svp->position_loop_count = 0;    // 位置环计数器
}




/**
  * @brief  电流环 PID 参数初始化（Id、Iq）
  */
static void Init_Current_PID(PID_t* pid_q, PID_t* pid_d)
{
    // Q 轴电流 PID（转矩环）
    pid_q->Ref = 1.5f;           // 初始目标转矩
    pid_q->Kp = 0.97f;           // 比例系数
    pid_q->Ki = 0.013f;          // 积分系数
    pid_q->ErrsumMax = 7.2f;     // 积分上限
    pid_q->ErrsumMin = -7.2f;    // 积分下限
    pid_q->OutMax = 10.0f;       // 输出上限
    pid_q->OutMin = -10.0f;      // 输出下限

    // D 轴电流 PID（励磁环，默认 0）
    pid_d->Ref = 0.0f;
    pid_d->Kp = 0.97f;
    pid_d->Ki = 0.013f;
    pid_d->ErrsumMax = 7.20f;
    pid_d->ErrsumMin = -7.20f;
    pid_d->OutMax = 12.0f;
    pid_d->OutMin = -12.0f;
}


/**
  * @brief  速度/位置计算数据初始化
  */
static void Init_Velocity_Data(Vel_Pos_t* vec_pos_data)
{
    vec_pos_data->last_angle = 0;         // 上一时刻角度
    vec_pos_data->now_angle = 0;          // 当前角度
    vec_pos_data->fre_interrupt = 4000;   // 中断频率
    vec_pos_data->speed = 0;              // 当前速度
    vec_pos_data->last_speed = 0;         // 上一时刻速度
}



/**
  * @brief  速度环 PID 参数初始化
  */
//最开始的过冲有点强,但是似乎找不到更好的参数
static void Init_Velocity_PID(PID_t* vel_pid){
    vel_pid->Ref = 0.0f;            //目标速度
    vel_pid->Kp = 0.018f;           //比例增益
    vel_pid->Ki = 0.0016f;          //积分增益
    vel_pid->ErrsumMax = 7.20f;    //积分限幅
    vel_pid->ErrsumMin = -7.20f;   //积分限幅
    vel_pid->OutMax = 8.66f;       //输出限幅
    vel_pid->OutMin = -8.66f;      //输出限幅
    //限制的输出与电流环的输出一致
}


/**
  * @brief  位置环 PID 参数初始化
  */
//2,0.05
static void Init_Position_PID(PID_t* pos_pid){
    pos_pid->Ref = 0.0f;            //目标位置
    pos_pid->Kp = 1.5f;             //比例增益
    pos_pid->Ki = 0.0f;             //积分增益
    pos_pid->ErrsumMax = 0.1f;      //积分限幅
    pos_pid->ErrsumMin = -0.1f;     //积分限幅
    pos_pid->OutMax = 100.0f;       //输出限幅
    pos_pid->OutMin = -100.0f;      //输出限幅
    //这里的输出限制理论是速度环的最大速度，但是我改小了
}


/**
  * @brief  低通滤波器初始化
  */
static void Init_Filter(Filter_t* filter){
    filter->Ts = 0.00017f;  //采样周期
    filter->Fc = 6000.0f;   //截止频率
    filter->Alpha = 0.2f;   //滤波系数
}




/**
  * @brief  所有结构体统一初始化入口
  */
void Init_all_struct(void)
{
    Init_Motor_Params(&My_motor);
    Init_Current_Calibration(&Cur_Cal);
    Init_SVPWM(&Svpwm);
    Init_Current_PID(&Cur_Uq, &Cur_Ud);
    Init_Velocity_Data(&Vec_pos_data);
    Init_Velocity_PID(&Vel_PI);
    Init_Position_PID(&Pos_PI);
    Init_Filter(&LP_filter);
}

