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
#define _2PI 6.28318530718f


// 全局实例化
FOC_ControllerTypeDef foc = {0};
volatile float Speed_Target_Cmd = 0.0f;   // 串口给的最终目标速度

extern float U_alpha,U_beta;

extern float Zero_Angle;


/***************************************************
 * 【滑模观测器 SMO】结构体实例（全局）
 ***************************************************/
SMO_HandleTypeDef smo = {
    .Rs = 3.5f,
    .Ls = 0.00086f,
    .h  = 0.3f,
    // 其余默认0初始化
};

/***************************************************
 * 【PLL锁相环】结构体实例（全局）
 ***************************************************/
PLL_HandleTypeDef pll = {
    .kp = 530.0f,  // 提高KP，加快角度误差响应
    .ki = 40000.0f, // 降低KI，减少积分漂移
};

/***************************************************
 * 【VF】结构体实例（全局）
 ***************************************************/
VF_HandleTypeDef vf = {
    .VF_flag = 0,
    .Target_Vel_openloop = 0.0f,
    .VF_max_vel = 30.0f,      // 机械角速度上限，先拉高一点
    .VF_acc = 30.0f,          // 机械加速度，先别太小
    .VF_uq_delta = 0.04f,     // Uq随速度增长
    .shaft_angle = 0.0f,
    .open_loop_timestamp = 0,
    .init_time = 0,
    ._dir = 1,
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


/* ADC初始化 */
void AD_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
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

/* TIM2 初始化-1ms */
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


/* TIM4 初始化-1ms */
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


/* TIM3 初始化-1ms */
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

/* ADC 中断 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {     
        VF_Run_Task();
        foc.samp_volts[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        foc.samp_volts[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
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
    // 不要直接给 foc.speed_hope
    // 这里只更新最终目标速度
    Speed_Target_Cmd = Target_Velocity;

    // 电流环仍然执行
    M1_Set_CurTorque(foc.iq_hope);
}

/* 速度斜坡函数（加减速限制，防冲击）*/
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


/* TIM2 中断 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        foc.angle_ele = pll.est_theta;

        foc.speed = pll.est_speed_F / 7.0f;
        foc.speed = LPF_current_Speed(foc.speed);

        const float Ts = 0.001f;
        const float SPEED_ACC = 150.0f;
        const float SPEED_DEC = 200.0f;

        foc.speed_hope = Speed_Ramp(Speed_Target_Cmd, foc.speed_hope, SPEED_ACC, SPEED_DEC, Ts);

        foc.iq_hope = Speed_PID(0.0150f, 0.120f, 0.0f,  foc.speed_hope - foc.speed);

 //       GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
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
0 → 空闲停止
↓（调用 VF_Start_Init）
2 → 开环运行
↓（7秒到）
0 → 停止
*/

void VF_Start_Init(int dir)
{
    vf.VF_flag = 2;
    vf.Target_Vel_openloop = 0.0f;
    vf._dir = (dir >= 0) ? 1 : -1;

    vf.shaft_angle = 0.0f;
    vf.open_loop_timestamp = 0;
    vf.init_time = 0;

    pll.est_theta = 0.0f;
    pll.est_speed = 0.0f;
    pll.est_speed_F = 0.0f;
    pll.i_err = 0.0f;

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

    // 运行5秒后停止

    // VF运行5秒后，切到无感速度闭环
    if (vf.init_time >= 5000)
    {
    
        vf.VF_flag = 0;                  // 切换到无感速度闭环
        vf.Target_Vel_openloop = 0.0f;

        // 交接时尽量平滑：先把当前开环角交给FOC/PLL
        foc.angle_ele = _normalizeAngle(vf.shaft_angle * 7);
        pll.est_theta = foc.angle_ele;

        // 速度反馈继承PLL当前估计值，避免刚切换时突变
        foc.speed = pll.est_speed_F / 7.0f;
        foc.lpf_speed_prev = foc.speed;
        foc.speed_hope = foc.speed;
        Speed_Target_Cmd = foc.speed;
         
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

