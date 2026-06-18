#include "./ADC/bsp_adc.h"
#include "stm32f10x.h"
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "./FOC/bsp_foc.h"
#include "./PID/bsp_pid.h"
#include <math.h>
#include "Delay.h"
#include "IQ_math.h"

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI     6.28318530718f


// 全局实例化
FOC_ControllerTypeDef foc = {0};

///////////////////////////////////////////////////////////////////////////////////////
/* 串口或上层命令给出的最终目标速度。
 * 速度环不会直接跳到该值，而是通过 Speed_Ramp() 做斜坡过渡。
 */
volatile float Speed_Target_Cmd = 0.0f;   

/* 正反转换向状态机标志：
 * 0：正常闭环运行；
 * 1：检测到目标速度反向，正在先闭环降速到 0；
 * 2：降速完成，正在按新方向执行 V/F 开环拖动，建立反电动势后再切回闭环。
 */
volatile uint8_t Reverse_Flag = 0;

/* 换向过程中保存的新目标速度。
 * 例如当前 +20，收到 -20 时，先保存 -20，降速到 0 后再切换到 -20。
 */
volatile float Reverse_Target = 0.0f;
///////////////////////////////////////////////////////////////////////////////////////

extern float U_alpha,U_beta;
extern float Zero_Angle;

/***************************************************
 * [滑模观测器 SMO] 结构体实例
 *
 * Rs：相电阻，单位一般为 Ω；
 * Ls：相电感，单位一般为 H；
 * h ：滑模增益，影响反电动势估计幅值和抖振程度。
 ***************************************************/
SMO_HandleTypeDef smo = {
    .Rs = 3.5f,
    .Ls = 0.00086f,
    .h  = 0.3f,
    // 其余默认0初始化
};

/***************************************************
 * [PLL 锁相环] 结构体实例
 *
 * kp：角度误差比例增益，越大角度跟随越快，但过大可能震荡；
 * ki：角度误差积分增益，用于消除稳态误差，但过大可能造成积分漂移。
 ***************************************************/
PLL_HandleTypeDef pll = {
    .kp = 530.0f,   // 提高KP，加快角度误差响应
    .ki = 40000.0f, // 降低KI，减少积分漂移
};

/***************************************************
 * [V/F 开环启动] 结构体实例
 *
 * V/F 用于无感启动或换向时的开环拖动。
 * 当电机速度太低时反电动势很弱，SMO/PLL 不容易稳定估计角度，
 * 因此先用固定电压矢量按给定方向拖动一段时间，再切入闭环。
 ***************************************************/
VF_HandleTypeDef vf = {
    .VF_flag = 0,                   // 0：未开环；2：正在 V/F 开环运行
    .Target_Vel_openloop = 0.0f,    // V/F 当前开环目标机械角速度
    .VF_max_vel = 30.0f,            // V/F 机械角速度上限
    .VF_acc = 90.0f,                // V/F 机械角加速度
    .VF_uq_delta = 0.04f,           // Uq 随速度增加的斜率
    .shaft_angle = 0.0f,            // V/F 内部积分得到的机械角度
    .open_loop_timestamp = 0,       // V/F 运行节拍计数，TIM3 中每 1ms 加 1
    .init_time = 0,                 // V/F 已运行时间计数，单位约为 ms
    ._dir = 1,                      // 开环方向：1 正转，-1 反转
};



/****************************************************************************/
/* 低通滤波器：Q轴电流 */
float LPF_current_q(float x)
{
    float y = 0.95f * foc.lpf_q_prev + 0.05f * x;
    foc.lpf_q_prev = y;
    return y;
}

/* 低通滤波器：D轴电流 */
float LPF_current_d(float x)
{
    float y = 0.9f * foc.lpf_d_prev + 0.1f * x;
    foc.lpf_d_prev = y;
    return y;
}

/* 低通滤波器：速度 */
float LPF_current_Speed(float x)
{
    float y = 0.98f * foc.lpf_speed_prev + 0.02f * x;
    foc.lpf_speed_prev = y;
    return y;
}
/****************************************************************************/


/*
 * ADC 初始化
 *
 * 功能：
 * 1. 配置 PA0、PA1 为模拟输入；
 * 2. 配置 ADC1、ADC2 为注入同步采样模式；
 * 3. 使用 TIM1_TRGO 作为注入转换触发源；
 * 4. 开启 ADC1 注入转换完成中断；
 * 5. 初始化 PA4 作为调试 IO，可用于示波器观察中断周期。
 */
void AD_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* 使能 ADC1、ADC2 和 GPIOA 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOA, ENABLE);
    /* PA0、PA1 设置为模拟输入，对应 ADC_Channel_0 和 ADC_Channel_1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* ADC 时钟 = PCLK2 / 6。 PCLK2 = 72MHz，则 ADC 时钟为 12MHz */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;   // ADC1 和 ADC2 注入组同步转换
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;        // 单通道，不扫描多个通道
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // 不连续转换，由外部触发
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //规则组外部触发关闭，这里主要使用注入组触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;              // 规则组通道数量为 1
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);

    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5);

    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2));

    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 测试IO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}



/**********************************************************************************************************************/

/*
 * TIM2 初始化：1ms 周期中断
 *
 * 1. 从PLL读取电角度和估计速度；
 * 2. 执行速度目标斜坡；
 * 3. 执行速度环 PID，得到 iq_hope；
 * 4. 处理换向状态机。
 */
void BSP_TIM(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}


/*
 * TIM4 初始化：1ms 周期中断
 * 每 1ms 进行一次滑模观测器和PLL角度估算。
 */
void BSP_TIM4(void)
{
    // 1. 使能 TIM4 时钟（APB1 总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 2. 使用内部时钟
    TIM_InternalClockConfig(TIM4);

    // 3. 时基配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;        // 向上计数
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;            // 不分频
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;                      // 自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;                     // 预分频器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                  // 高级定时器用，通用定时器设0
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    // 4. 清除更新中断标志，使能更新中断
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // 5. 配置 NVIC 中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                       // TIM4 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;             // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                    // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 6. 启动定时器
    TIM_Cmd(TIM4, ENABLE);
}


/*
 * TIM3 初始化：1ms 周期中断
 *
 * 只在 V/F 开环运行时累加时间计数，
 * 用于控制 V/F 开环持续时间以及 VF_Run_Task() 的 1ms 节拍。
 */
void BSP_TIM3(void)
{
    // 1. 使能 TIM3 时钟（APB1 总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 2. 使用内部时钟
    TIM_InternalClockConfig(TIM3);

    // 3. 时基配置：1ms 中断
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;       // 向上计数
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;           // 不分频
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;                      // 1000us = 1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;                     // 72MHz / 72 = 1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                  // 通用定时器设0
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    // 4. 清除更新中断标志，使能更新中断
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // 5. 配置 NVIC 中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                       // TIM3 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;             
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 6. 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

/**********************************************************************************************************************/

/*  ADC1 和 ADC2 共用中断入口。 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {     
        foc.samp_volts[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1); //ADC1 对应 A 相采样通道
        foc.samp_volts[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1); //ADC2 对应 B 相采样通道
        // V/F 开环任务放在 ADC 中断中执行。
        VF_Run_Task();

        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
   //    GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
    }
}


/* 电流零点校准 */
void DriftOffsets(void)
{
    uint16_t detect_rounds = 1000;
    float sum_ia = 0, sum_ib = 0;

    setPhaseVoltage_FOC2(0.0f, 0.0f, 0.0f);
    Delay_ms(10);

    for (int i = 0; i < detect_rounds; i++)
    {
        sum_ia += foc.samp_volts[0];
        sum_ib += foc.samp_volts[1];
    }

    foc.offset_ia = sum_ia / detect_rounds;
    foc.offset_ib = sum_ib / detect_rounds;
}


/* 电流环 */
void M1_Set_CurTorque(float Target_Current)
{
    // 电流采样
    foc.ele.Ia = -(foc.samp_volts[0] - foc.offset_ia) * 3.3f / 4096.0f / 50.0f / 0.01f;
    foc.ele.Ib = -(foc.samp_volts[1] - foc.offset_ib) * 3.3f / 4096.0f / 50.0f / 0.01f;
    foc.ele.Ic = -(foc.ele.Ia + foc.ele.Ib);

    // Clark
    foc.ele.I_alpha = foc.ele.Ia;
    foc.ele.I_beta = _1_SQRT3 * foc.ele.Ia + _2_SQRT3 * foc.ele.Ib;

    float Angle_el = foc.angle_ele;
    
    // Park
    float cos_t = _cos(Angle_el);
    float sin_t = _sin(Angle_el);
    foc.ele.Iq = foc.ele.I_beta * cos_t - foc.ele.I_alpha * sin_t;
    foc.ele.Id = foc.ele.I_alpha * cos_t + foc.ele.I_beta * sin_t;

    // 滤波
    foc.ele.Iq = LPF_current_q(foc.ele.Iq);
    foc.ele.Id = LPF_current_d(foc.ele.Id);

    // PID
    float err_q = Target_Current - foc.ele.Iq;
    float err_d = foc.id_hope - foc.ele.Id;

    foc.ud = PID_Controller(PID_D_AXIS, 0.0f, 0.0f, 0.0f, err_d);
    foc.uq = PID_Controller(PID_Q_AXIS, 6.0f, 40.0f, 0.0f, err_q);

    // 输出
    setPhaseVoltage_FOC2(foc.uq, 0, Angle_el);
}


/* 速度环 */
void M1_Set_Velocity(float Target_Velocity)
{
    // 如果目标速度和当前期望速度方向相反，先降速到0
    if ((Target_Velocity * foc.speed_hope < 0.0f) && (Reverse_Flag == 0))
    {
        Reverse_Target = Target_Velocity;
        Reverse_Flag = 1;

        // 先让闭环降速到0
        Speed_Target_Cmd = 0.0f;
    }
    else if(Reverse_Flag == 0)
    {
        // 正常调速
       Speed_Target_Cmd = Target_Velocity;
    }

    // 开环拖动时不跑电流闭环，避免和VF开环抢PWM输出
    if (vf.VF_flag == 2)
        return;

    // 电流环仍然执行
    M1_Set_CurTorque(foc.iq_hope);
}


/*
 * 速度斜坡函数：限制目标速度变化率，减小机械冲击和电流冲击。
 *
 * target ：最终目标速度
 * current：当前期望速度
 * acc    ：加速斜率
 * dec    ：减速斜率
 * Ts     ：调用周期，当前为 0.001s。
 */
float Speed_Ramp(float target, float current, float acc, float dec, float Ts)
{
    float diff = target - current;

    float step_up = acc * Ts;
    float step_down = dec * Ts;

    if (diff > step_up)
        current += step_up;
    else if (diff < -step_down)
        current -= step_down;
    else
        current = target;

    return current;
}


/*
 * TIM2 中断服务函数：1ms 速度环与换向状态机
 *
 * 执行流程：
 * 1. 使用 PLL 估算角度作为 FOC 电角度；
 * 2. 将 PLL 电角速度转换为机械速度并滤波；
 * 3. 如果 V/F 正在运行，则暂停速度环；
 * 4. 对速度目标做斜坡；
 * 5. 如果换向降速到 0，则启动反向 V/F 开环；
 * 6. 正常情况下执行速度 PID，输出 q 轴目标电流 iq_hope。
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        /* PLL 估算的电角度作为 FOC 使用的电角度 */
        foc.angle_ele = pll.est_theta;
 
        foc.speed = pll.est_speed_F / 7.0f;       // PLL 估计的是电角速度，除以极对数 7 得到机械转速
        foc.speed = LPF_current_Speed(foc.speed); // 对速度估计值滤波，降低速度环抖动

        const float Ts = 0.001f;
        const float SPEED_ACC = 200.0f;
        const float SPEED_DEC = 200.0f;

        /* V/F开环拖动时不计算速度环，避免速度环输出 iq_hope 干扰开环输出 */
        if (vf.VF_flag == 2)
        {
            foc.iq_hope = 0.0f;
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
            return;
        }

        foc.speed_hope = Speed_Ramp(Speed_Target_Cmd, foc.speed_hope, SPEED_ACC, SPEED_DEC, Ts);

        /* 换向流程：
         * Reverse_Flag == 1 表示正在降速；
         * 当 speed_hope 已经降到 0，启动目标方向的V/F开环拖动。
         */
        if (Reverse_Flag == 1 && foc.speed_hope == 0.0f)
        {
            int dir = (Reverse_Target >= 0.0f) ? 1 : -1;

            /* 按新目标方向启动V/F开环 */
            VF_Start_Init(dir);
            Reverse_Flag = 2;
            foc.iq_hope = 0.0f;

            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
            return;
        }

        foc.iq_hope = Speed_PID(0.0176f, 0.120f, 0.0f, foc.speed_hope - foc.speed);

 //     GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}


/*SMO_PLL ***************************************************************************************************************/
// 饱和函数
static float sat(float err, float limits) 
{
  if (err > limits) return 1;
  else if (err < -limits) return -1;
  else
    return err / limits;
}



// 滑膜观测器
void SMO_position_estimate(void)
{
    float Ts = 0.001f;

    smo.Est_Ialpha = foc.ele.I_alpha;
    smo.Est_Ibeta  = foc.ele.I_beta;
    smo.Ualpha = U_alpha;
    smo.Ubeta  = U_beta;

    // 观测反电动势
    smo.Est_Ialpha += Ts * (-smo.Rs / smo.Ls * smo.Est_Ialpha + 1.0f / smo.Ls * (smo.Ualpha - smo.Ealpha));
    smo.Est_Ibeta  += Ts * (-smo.Rs / smo.Ls * smo.Est_Ibeta  + 1.0f / smo.Ls * (smo.Ubeta  - smo.Ebeta));

    // 电流误差
    float Ialpha_Err = smo.Est_Ialpha - foc.ele.I_alpha;
    float Ibeta_Err  = smo.Est_Ibeta  - foc.ele.I_beta;

    // 滑模控制
    smo.Ealpha = smo.h * sat(Ialpha_Err, 0.50f);
    smo.Ebeta  = smo.h * sat(Ibeta_Err, 0.50f);

    // 低通滤波
    smo.Ealpha_flt = 0.05f * smo.Ealpha_flt + 0.95f * smo.Ealpha;
    smo.Ebeta_flt  = 0.05f * smo.Ebeta_flt  + 0.95f * smo.Ebeta;

    // Dir控制：正转 = 1，反转 = -1 
    float pll_dir = (foc.speed_hope >= 0) ? 1.0f : -1.0f;

    // 给PLL的输入按方向翻转
    SMO_PLL_calc(pll_dir * smo.Ealpha_flt, pll_dir * smo.Ebeta_flt);

   // 反正切获取角度
//    smo.Est_theta = -atan2f(smo.Ebeta_flt, smo.Ealpha_flt);
//    smo.Est_theta = _normalizeAngle(smo.Est_theta);
}



// 传入SMO计算的Ealpha，Ebeta
void SMO_PLL_calc(float alpha, float beta)
{
    float Ts = 0.001f;
    
    // 角度误差
    pll.theta_err = -1.0f * alpha * _cos(pll.est_theta) - beta * _sin(pll.est_theta);
    
    // PI控制器
    pll.i_err += Ts * pll.ki * pll.theta_err;
    pll.est_speed = pll.kp * pll.theta_err + pll.i_err;
    
    // 速度滤波
    pll.est_speed_F = pll.est_speed_F * 0.85f + pll.est_speed * 0.15f;
    
    // 角度积分
    pll.est_theta += Ts * pll.est_speed;
    pll.est_theta = _normalizeAngle(pll.est_theta);
}



// TIM4 中断服务函数（1ms中断，执行SMO+PLL位置估算）
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
  
         SMO_position_estimate();
//         GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
     //  清除中断标志位
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

/***************************************************************************************************************SMO_PLL*/

/*
正转 20
收到 -20
Speed_Target_Cmd = 0
闭环降速
speed_hope 到 0
VF_Start_Init(-1)
反向开环 5 秒
切回闭环
Speed_Target_Cmd = -20
*/

void VF_Start_Init(int dir)
{
    // 标记进入 V/F 开环运行状态 
    vf.VF_flag = 2;

    // 开环目标速度从 0 开始爬升 
    vf.Target_Vel_openloop = 0.0f;

    // 保存开环方向 
    vf._dir = (dir >= 0) ? 1 : -1;

    // 清零开环角度与计时 
    vf.shaft_angle = 0.0f;
    vf.open_loop_timestamp = 0;
    vf.init_time = 0;

    // 清零 PLL 状态，准备后续重新锁定 
    pll.est_theta = 0.0f;
    pll.est_speed = 0.0f;
    pll.est_speed_F = 0.0f;
    pll.i_err = 0.0f;

    // 清除 PID 积分项，防止切换过程中的积分累积影响输出 
    PID_Clear_Integral();
}




void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        if (vf.VF_flag == 2)
        {
            vf.open_loop_timestamp++;   // 1ms节拍
            vf.init_time++;             // 运行时间累计
        }
    }
}


// V/F 启动，此处放入AD中断执行，让主程序main减轻负担
void VF_Run_Task(void)
{
    static uint32_t last_tick = 0;

    if (vf.VF_flag != 2)
        return;

    // VF运行4.3秒后，切到无感速度闭环
    if (vf.init_time >= 430)
    {
        vf.VF_flag = 0;                  // 切换到无感速度闭环
        vf.Target_Vel_openloop = 0.0f;

        // 交接时尽量平滑：先把当前开环角交给FOC/PLL
        foc.angle_ele = _normalizeAngle(vf.shaft_angle * 7);
        pll.est_theta = foc.angle_ele;

        // 如果是换向开环结束，切到新的反向闭环目标
        if (Reverse_Flag == 2)
        {
            foc.speed = vf._dir * vf.VF_max_vel;
            foc.lpf_speed_prev = foc.speed;
            foc.speed_hope = foc.speed;
            Speed_Target_Cmd = Reverse_Target;

            // 给PLL一个带方向的速度初值，避免刚切闭环时速度符号乱跳
            pll.est_speed = foc.speed * 7;
            pll.est_speed_F = pll.est_speed;

            Reverse_Flag = 0;
        }
        else
        {
            // 普通开环启动结束，保持原来的切换方式
            foc.speed = pll.est_speed_F / 7;
            foc.lpf_speed_prev = foc.speed;
            foc.speed_hope = foc.speed;
            Speed_Target_Cmd = foc.speed;
        }
         
        // 清除PI控制器积分
        PID_Clear_Integral(); 
        pll.i_err = 0;
        return;
    }

    // 没有新的1ms节拍，就不执行
    if (vf.open_loop_timestamp == last_tick)
        return;

    last_tick = vf.open_loop_timestamp;

    const float Ts = 0.001f;
    const float POLE_PAIRS = 7;

    // 速度爬升
    if (vf.Target_Vel_openloop < vf.VF_max_vel)
    {
        vf.Target_Vel_openloop += vf.VF_acc * Ts;
        if (vf.Target_Vel_openloop > vf.VF_max_vel)
            vf.Target_Vel_openloop = vf.VF_max_vel;
    }
    else
    {
        vf.Target_Vel_openloop = vf.VF_max_vel;
    }

    // 让SMO里的方向判断也能跟随开环方向
    foc.speed_hope = vf._dir * vf.Target_Vel_openloop;

    // VF内部自己积分机械角
    vf.shaft_angle = _normalizeAngle(vf.shaft_angle + vf._dir * vf.Target_Vel_openloop * Ts);

    //  直接用开环角生成电角度
    float angle_el_open = _normalizeAngle(vf.shaft_angle * POLE_PAIRS);

    // Uq随速度略增加帮助建立反电动势
    float uq_open = 2.0f + vf.Target_Vel_openloop * vf.VF_uq_delta;

    // 电压限幅
    if (uq_open > 6.0f)
        uq_open = 6.0f;

    //  纯开环输出
    setPhaseVoltage_FOC2(uq_open, 0.0f, angle_el_open);

}

