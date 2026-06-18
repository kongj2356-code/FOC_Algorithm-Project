#include "bsp_tim.h"
#include "Delay.h"

/* ================== 全局变量定义（仅在C文件中定义一次） ================== */
uint8_t Motor_Ctrl_Mode = MOTOR_CTRL_MODE_BOTH;  // 默认同时控制两路
float Speed_hope_M1;  // 电机1期望速度
float Speed_hope_M2;  // 电机2期望速度


// 电机1实例
MotorFOC_Typedef Motor1 = {
    .as5600_channel = AS5600_CHANNEL_MOTOR1,
    .motor_id = MOTOR_1,
    .Kp = 0.1f,
    .Ki = 0.002f,
    .Kd = 0.0f
};

// 电机2实例
MotorFOC_Typedef Motor2 = {
    .as5600_channel = AS5600_CHANNEL_MOTOR2,
    .motor_id = MOTOR_2,
    .Kp = 0.1f,
    .Ki = 0.002f,
    .Kd = 0.0f
};



/* ================== 零电角标定 - 通用函数 ================== */
void Zero_Get(MotorFOC_Typedef *motor)
{
    float Elec_Angle;
    for (int i = 0; i <= 200; i++)
    {
        Elec_Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(motor->motor_id, 4.0f, 0.0f, Elec_Angle);
        Delay_ms(1);
    }

    for (int i = 200; i >= 0; i--)
    {
        Elec_Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(motor->motor_id, 4.0f, 0.0f, Elec_Angle);
        Delay_ms(5);
    }

    motor->Zero_Angle = GetAngle(motor->as5600_channel) * 7;
}

/* ================== 一阶低通 - 通用函数 ================== */
float LPE_VELOCITY(MotorFOC_Typedef *motor, float x)
{
    float y = 0.9 * motor->Speed_pre + 0.1 * x; 
    motor->Speed_pre = y;
    return y;
}

/**
 * @brief  将FOC电机的原始角度值转换为连续无跳变的角度值（通用版）
 * @param  motor: 电机实例指针
 * @param  angle_raw: 原始角度值（范围通常为0~2π，会在2π处回零）
 * @retval 连续递增/递减的角度值（无2π跳变）
 */
float FOC_Angle_Continuous(MotorFOC_Typedef *motor, float angle_raw)
{
    float delta = angle_raw - motor->angle_raw_last;  // 计算角度差值

    // 处理2π周期回绕：差值超π则修正
    if (delta >  _PI_3 * 3)      
        delta -= _2PI;           // 正向跳变，修正差值
    else if (delta < -_PI_3 * 3) 
        delta += _2PI;           // 反向跳变，修正差值

    motor->angle_foc += delta;         // 累加得到连续角度
    motor->angle_raw_last = angle_raw; // 更新上一次角度

    return motor->angle_foc;           // 返回连续FOC角度
}

/* ================== 单电机速度环控制================== */
static void Motor_Speed_Loop(MotorFOC_Typedef *motor, float speed_hope)
{
    float Angle_data, Angle_ele;

    // 1. 读取编码器角度
    motor->Angle = GetAngle(motor->as5600_channel);
    Angle_ele = motor->Angle * 7;    // 电气角转换

    // 2. 连续角度处理（防2π跳变）
    if (motor->Angle_pre - motor->Angle > 0.8 * _2PI)
    {
        Angle_data = motor->Angle + _2PI;
    }
    else if (motor->Angle - motor->Angle_pre > 0.8 * _2PI)
    {
        Angle_data = motor->Angle - _2PI;
    }
    else
    {
        Angle_data = motor->Angle;
    }

    // 3. 速度计算 (1ms采样周期，单位：rad/s)
    motor->Speed = (Angle_data - motor->Angle_pre) / 0.001f;
    motor->Speed = LPE_VELOCITY(motor, motor->Speed);  // 一阶低通滤波

    // 4. 速度闭环PID控制
    motor->Uq = Speed_PID(motor->Kp, motor->Ki, motor->Kd, (speed_hope - motor->Speed));

    // 5. FOC电角度计算与驱动
    float angle_cont = FOC_Angle_Continuous(motor, Angle_ele - motor->Zero_Angle);
    setPhaseVoltage_FOC2(motor->motor_id, motor->Uq, 0.0f, angle_cont);

    // 6. 更新上一拍角度
    motor->Angle_pre = motor->Angle;
}

/* ================== TIM2 初始化 (主控制定时器 1ms) ================== */
void BSP_TIM()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;    // 1ms 中断周期
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;   // 72MHz / 72 = 1MHz 计数时钟
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/* ================== TIM2 中断服务函数 (带模式选择) ================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        // 根据控制模式选择执行对应的电机控制逻辑
        switch(Motor_Ctrl_Mode)
        {
            case MOTOR_CTRL_MODE_M1_ONLY:  // 仅控制电机1
                Motor_Speed_Loop(&Motor1, Speed_hope_M1);
                break;
                
            case MOTOR_CTRL_MODE_M2_ONLY:  // 仅控制电机2
                Motor_Speed_Loop(&Motor2, Speed_hope_M2);
                break;
                
            case MOTOR_CTRL_MODE_BOTH:     // 同时控制两路电机
                Motor_Speed_Loop(&Motor1, Speed_hope_M1);
                Motor_Speed_Loop(&Motor2, Speed_hope_M2);
                break;
                
            default:  // 非法模式默认同时控制
                setPhaseVoltage_FOC2(Motor1.motor_id, 0.0f, 0.0f, 0);
                setPhaseVoltage_FOC2(Motor2.motor_id, 0.0f, 0.0f, 0);
                break;
        }

    
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/* ================== 初始化函数 (主函数调用) ================== */
void FOC_DualMotor_Init(void)
{  
    // 标定两路电机零电角,放前面一定要
    Zero_Get(&Motor1);
    Zero_Get(&Motor2);
    // 初始化定时器
    BSP_TIM();
}

/* ================== 控制模式设置函数（外部调用接口） ================== */
void Set_Motor_Ctrl_Mode(uint8_t mode)
{
    // 校验模式合法性
    if(mode == MOTOR_CTRL_MODE_M1_ONLY || mode == MOTOR_CTRL_MODE_M2_ONLY || mode == MOTOR_CTRL_MODE_BOTH)
    {
        Motor_Ctrl_Mode = mode;
    }
    else
    {
        Motor_Ctrl_Mode = MOTOR_CTRL_MODE_BOTH;  // 非法值默认同时控制
    }
}
