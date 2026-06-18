#include "bsp_tim.h"
#include "Delay.h"



/* ================== 全局变量定义（仅在C文件中定义一次） ================== */
uint8_t Motor_Ctrl_Mode = MOTOR_CTRL_MODE_BOTH;  // 默认同时控制两路
float Angle_hope_M1 = 0.0f;  // 电机1期望角度（替换原速度期望）
float Angle_hope_M2 = 0.0f;  // 电机2期望角度（替换原速度期望）



// 电机1实例
MotorFOC_Typedef Motor1 = {
    .as5600_channel = AS5600_CHANNEL_MOTOR1,
    .motor_id = MOTOR_1,
    // 角度环PID参数（参考单电机）
    .Angle_Kp = 15.0f,
    .Angle_Ki = 0.0f,
    .Angle_Kd = 0.0f,
    // 速度环PID参数（参考单电机）
    .Speed_Kp = 0.38f,
    .Speed_Ki = 0.0001f,
    .Speed_Kd = 0.0f,
    // 初始化状态变量
    .Angle_pre = 0.0f,
    .Speed_pre = 0.0f,
    .angle_raw_last = 0.0f,
    .angle_foc = 0.0f
};


// 电机2实例
MotorFOC_Typedef Motor2 = {
    .as5600_channel = AS5600_CHANNEL_MOTOR2,
    .motor_id = MOTOR_2,
    // 角度环PID参数
    .Angle_Kp = 15.0f,
    .Angle_Ki = 0.0f,
    .Angle_Kd = 0.0f,
    // 速度环PID参数
    .Speed_Kp = 0.38f,
    .Speed_Ki = 0.0001f,
    .Speed_Kd = 0.0f,
    // 初始化状态变量
    .Angle_pre = 0.0f,
    .Speed_pre = 0.0f,
    .angle_raw_last = 0.0f,
    .angle_foc = 0.0f
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

/* ================== 机械角解卷绕（参考单电机） ================== */
float Angle_difference(float target, float current)
{
    float diff = target - current;
    while(diff > M_PI)  diff -= 2 * M_PI;
    while(diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

/* ================== 单电机角度环控制（核心修改） ================== */
static void Motor_Angle_Loop(MotorFOC_Typedef *motor, float angle_hope)
{
    float Angle_ele, actual_speed, speed_error;

    // 1. 读取编码器机械角度，转换电角度（极对数7）
    motor->Angle = GetAngle(motor->as5600_channel);
    Angle_ele = motor->Angle * 7;

    // 2. 计算角度误差（解卷绕）
    float angle_error = Angle_difference(angle_hope, motor->Angle);
    
    // 3. 角度环PID：输出速度期望值
    float speed_hope = Angle_PID(motor->Angle_Kp, motor->Angle_Ki, motor->Angle_Kd, angle_error);

    // 4. 计算实际速度（1ms采样周期，简化版，参考单电机）
    actual_speed = (motor->Angle - motor->Angle_pre); 
    actual_speed = LPE_VELOCITY(motor, actual_speed);

    // 5. 速度环PID：输出Uq电压
    speed_error = speed_hope - actual_speed;
    motor->Uq = Speed_PID(motor->Speed_Kp, motor->Speed_Ki, motor->Speed_Kd, speed_error);

    // 6. FOC电角度计算与驱动（补偿零位）
    float angle_cont = FOC_Angle_Continuous(motor, Angle_ele - motor->Zero_Angle);
    setPhaseVoltage_FOC2(motor->motor_id, motor->Uq, 0.0f, angle_cont);

    // 7. 更新历史值
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

/* ================== TIM2 中断服务函数 (角度环适配) ================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        // 根据控制模式选择执行对应的电机控制逻辑
        switch(Motor_Ctrl_Mode)
        {
            case MOTOR_CTRL_MODE_M1_ONLY:  // 仅控制电机1
                Motor_Angle_Loop(&Motor1, Angle_hope_M1);
                break;
                
            case MOTOR_CTRL_MODE_M2_ONLY:  // 仅控制电机2
                Motor_Angle_Loop(&Motor2, Angle_hope_M2);
                break;
                
            case MOTOR_CTRL_MODE_BOTH:     // 同时控制两路电机
                Motor_Angle_Loop(&Motor1, Angle_hope_M1);
                Motor_Angle_Loop(&Motor2, Angle_hope_M2);
                break;
                
            default:  // 非法模式停止电机
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
    // 标定两路电机零电角
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
