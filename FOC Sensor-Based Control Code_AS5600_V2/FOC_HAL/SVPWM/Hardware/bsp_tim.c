#include "bsp_tim.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"
#include "main.h"  
#include "tim.h"

/* ================== 状态变量 ================== */
// 棘轮/边界模式专用（阻尼模式完全不碰）
uint8_t g_knob_mode ;  
float Ratchet_Angle_pre = 0.0f;
float Ratchet_Speed_pre = 0.0f;
float Ratchet_target_detent = 0.0f;

// 阻尼/顺滑模式专用（棘轮模式完全不碰）
float Damp_Angle_pre = 0.0f;
float Damp_Speed_pre = 0.0f;
float Damp_Last_Torque = 0.0f; // 力矩平滑缓存

// 全局共享变量
float Speed;     // 仅棘轮用
float Uq;        // 仅棘轮用
extern float Zero_Angle;  
extern float Angle, Angle_ele;

extern uint8_t current_gear ;

/* ================== 滤波函数 ================== */
/**
 * 棘轮模式专用滤波
 */
static float Ratchet_LPF_VELOCITY(float x)
{
    float y = 0.9f * Ratchet_Speed_pre + 0.1f * x;
    Ratchet_Speed_pre = y;
    return y;
}

/**
 * 阻尼/顺滑模式专用滤波（系数0.8，柔）
 */
static float Damp_LPF_VELOCITY(float x)
{
    float y = 0.8f * Damp_Speed_pre + 0.2f * x;
    Damp_Speed_pre = y;
    return y;
}



/* ================== HAL库 TIM2 初始化 ================== */
void BSP_TIM2_Init(void)    // 1ms中断（和CubeMX配置的TIM2参数一致）
{
    // 1. 使能TIM2中断（HAL库方式）
    HAL_TIM_Base_Start_IT(&htim2);
    
    // 2. 配置NVIC（HAL库推荐在CubeMX中配置，这里补充代码配置兜底）
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 1);  // 抢占优先级2，子优先级1（和原标准库一致）
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}




/**
 * @brief  将角度归一化到 0 ~ 2π 范围内
 * @param  angle: 原始角度（rad）
 * @retval 归一化后的角度（rad），范围 [0, 2π)
 */
float normalize_angle_0_2pi(float angle)
{
    angle = fmodf(angle, _2PI);
    return angle >= 0 ? angle : (angle + _2PI);
}



/**
 * @brief  计算角度差值（最短旋转方向）
 * @note   输出差值范围[-π, π)，表示从current到target的最短旋转角度
 * @param  target: 目标角度（rad）
 * @param  current: 当前角度（rad）
 * @retval 角度差值（rad），正负表示旋转方向
 */
float Angle_difference(float target, float current)
{
    float diff = target - current;  // 原始角度差值
    // 调整差值至[-π, π)，取最短旋转路径
    while (diff >  M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}



/**
 * @brief  计算目标齿轮卡位角度（将当前角度匹配到最近的齿轮卡位）
 * @note   核心功能：角度归一化 + 最近卡位匹配，适用于旋转机构的定位控制
 * @param  current_angle: 当前角度（rad）
 * @retval 匹配后的目标卡位角度（rad），范围：[0, 2π)
 */
float Calculate_Target_Detent(float current_angle)
{    
    float norm_angle = normalize_angle_0_2pi(current_angle);
    
    // 计算当前挡位（0-7）
    current_gear = (int)round(norm_angle / GEAR_ANGLE_STEP) % GEAR_NUM;
    if (current_gear < 0) current_gear += GEAR_NUM;

    
    return current_gear * GEAR_ANGLE_STEP;
}


////////////////////////////////////阻尼_顺滑模式处理函数/////////////////////////////////////////////////
/**
 * 阻尼模式处理函数
 */
static void Damp_Mode_Handler(void)
{
    // 1. 阻尼专用角度差+速度计算
    float angle_diff = Angle_difference(Angle, Damp_Angle_pre);
    float actual_speed = angle_diff / 0.001f; // 1ms中断，转换为rad/s
    actual_speed = Damp_LPF_VELOCITY(actual_speed);

    // 2. 负反馈
    float torque = -KP_DAMP * actual_speed ;
  
    // 3. 力矩限幅：避免力矩过大导致“扯动”
    float torque_limit = 2.5f; 
    torque =  _constrain(torque, -torque_limit,torque_limit);

    // 4. 力矩平滑：彻底消除抖动
    torque = 0.95f * Damp_Last_Torque + 0.05f * torque;
    Damp_Last_Torque = torque;

    // 5. FOC控制
    setPhaseVoltage_FOC2(torque, 0.0f, Angle_ele - Zero_Angle);
    
    // 6. 更新阻尼专用角度基准
    Damp_Angle_pre = Angle;
}


/**
 * 顺滑模式处理函数
 */
static void Smooth_Mode_Handler(void)
{
    // 1. 阻尼专用角度差+速度计算
    float angle_diff = Angle_difference(Angle, Damp_Angle_pre);
    float actual_speed = angle_diff / 0.001f; // 1ms中断，转换为rad/s
    actual_speed = Damp_LPF_VELOCITY(actual_speed);
    
    // 2. 正反馈
    float torque = KP_SMOOTH * actual_speed;
  
    // 3. 力矩限幅：适配顺滑模式的固定限值
    float torque_limit = 2.5f; 
    torque = _constrain(torque, -torque_limit, torque_limit);
    
    // 4. 力矩平滑：消除抖动（保持原有平滑逻辑）
    torque = 0.92f * Damp_Last_Torque + 0.08f * torque;
    Damp_Last_Torque = torque;
    
    // 5. FOC控制
    setPhaseVoltage_FOC2(torque, 0.0f, Angle_ele - Zero_Angle);
    
    // 6. 更新阻尼专用角度基准
    Damp_Angle_pre = Angle;
}
//////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * 边界棘轮模式处理函数
 */
static void Boundary_Mode_Handler(void)
{
    // 计算边界范围（将角度范围转换为弧度）
    float angle_range_half = ANGLE_RANGE / 2;          // 半角度范围（deg）
    float limit_rad = angle_range_half / 180.0f * M_PI;   // 半角度范围（rad）
        
    // 基准角度（零点机械角度）
    float base_angle = normalize_angle_0_2pi(Zero_Angle / 7);
    // 计算上下边界并归一化（避免负数）
    float L1 = normalize_angle_0_2pi(base_angle + limit_rad);  // 上边界
    float L2 = normalize_angle_0_2pi(base_angle - limit_rad);  // 下边界

    // 标记是否在有效范围内
    uint8_t In_range = 0;
    /* 1.分两种情况判断：边界是否跨0点（2π）*/
    if (L1 > L2)                                          // 情况1：有效范围是 [L2, L1]（不跨0点）
    {
        In_range = (Angle >= L2 && Angle <= L1) ? 1 : 0;
    }
    else{                                                  // 情况2：有效范围跨0点，分为 [L2, 2π) 和 [0, L1]
        In_range = ((Angle >= L2 && Angle < _2PI) || (Angle >= 0 && Angle <= L1)) ? 1 : 0;
    }
    /* 2.根据是否在范围内设置目标角度 */
    if (In_range)
    {
        // 在边界内，保持原有棘轮模式，自由转动
        Ratchet_target_detent = Calculate_Target_Detent(Angle);
    }
    else{
        // 越界，计算到两个边界的最短距离，拉回最近的边界
        float diff_L1 = fabs(Angle_difference(L1, Angle));
        float diff_L2 = fabs(Angle_difference(L2, Angle));
            
        if (diff_L1 < diff_L2)
        {
            // 离上边界更近，拉回上边界
            Ratchet_target_detent = L1;
        }
        else
        {
            // 离下边界更近，拉回下边界
            Ratchet_target_detent = L2;
        }
    }
}

/**
 * 模式切换函数
 */
void Set_Knob_Mode(uint8_t mode)
{
    if (mode <= MODE_SMOOTH)
    {
        // 切换前先清空力矩，避免模式切换瞬间的抖动
        setPhaseVoltage_FOC2(0.0f, 0.0f, normalize_angle_0_2pi(Angle_ele - Zero_Angle));
        
        g_knob_mode = mode;
        
        // 分模式重置状态，避免互相干扰
        if (mode == MODE_RATCHET || mode == MODE_BOUNDARY)
        {
            Ratchet_Angle_pre = Angle;
            Ratchet_Speed_pre = 0.0f;
            Ratchet_target_detent = Calculate_Target_Detent(Angle);
        }
        else if (mode == MODE_DAMP || mode == MODE_SMOOTH)
        {
            Damp_Angle_pre = Angle;
            Damp_Speed_pre = 0.0f;
            Damp_Last_Torque = 0.0f;
        }
    }
}

/* ================== HAL库 TIM2 中断处理（核心修改） ================== */

// 2. HAL库定时器更新中断回调函数（实际业务逻辑在这里执行）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) // 仅处理TIM2的更新中断
    {
        // 1. 提前声明所有变量，避免goto跳过初始化
        float angle_error = 0.0f;
        float effective_error = 0.0f;
        float actual_speed = 0.0f;
        float speed_error = 0.0f;

        
        // 2. 读取角度并归一化
        Angle = GetAngle();
        Angle = normalize_angle_0_2pi(Angle);
        Angle_ele = Angle * 7;

        // 3. 模式调度：保留逻辑，goto仅跳转已初始化的变量区域
        switch (g_knob_mode)
        {
            case MODE_RATCHET:
                Ratchet_target_detent = Calculate_Target_Detent(Angle);
                goto RATCHET_LOGIC;
                
            case MODE_BOUNDARY:
                Boundary_Mode_Handler();
                goto RATCHET_LOGIC;
                
            case MODE_DAMP:
                Damp_Mode_Handler();
                goto IRQ_END; // 阻尼模式直接结束
                
            case MODE_SMOOTH:
                Smooth_Mode_Handler();
                goto IRQ_END; // 顺滑模式直接结束
                
            default:
                g_knob_mode = MODE_RATCHET;
                Ratchet_target_detent = Calculate_Target_Detent(Angle);
                goto RATCHET_LOGIC;
        }

        // 4. 仅棘轮/边界模式执行
    RATCHET_LOGIC:
        // 原始角度误差
        angle_error = Angle_difference(Ratchet_target_detent, Angle);
        effective_error = 0.0f; // 初始化，避免未赋值警告
        if (fabs(angle_error) > DEAD_ZONE)
        {
            effective_error = angle_error - (angle_error > 0 ? DEAD_ZONE : -DEAD_ZONE);
        }

        // 角度环（P） -> 速度期望 
        Speed = Angle_PID(50, 0.0f, 0.0f, effective_error);
        // 低通滤波后的实际速度
        actual_speed = Angle - Ratchet_Angle_pre;
        actual_speed = Ratchet_LPF_VELOCITY(actual_speed);
        // 速度环（PI） -> Uq 
        speed_error = Speed - actual_speed;
        Uq = Speed_PID(0.18f, 140.0f, 0.0f, speed_error);

        // 电角度归一化
        setPhaseVoltage_FOC2(Uq, 0.0f, Angle_ele - Zero_Angle);
        
        // 更新棘轮专用状态
        Ratchet_Angle_pre = Angle;
        Ratchet_Speed_pre = Ratchet_LPF_VELOCITY((Angle - Ratchet_Angle_pre)/0.001);

        // 5. 中断结束
    IRQ_END:
      
    }
}

/* ================== 初始化函数 ================== */
void Knob_Init(void)
{
    // 1. 初始化TIM2（HAL库方式）
    BSP_TIM2_Init();

    // 3. 读取初始角度
    Angle = GetAngle();

    // 初始化棘轮模式状态
    Ratchet_Angle_pre = Angle;
    Ratchet_Speed_pre = 0.0f;
    Ratchet_target_detent = Calculate_Target_Detent(Angle);
    
    // 初始化阻尼/顺滑模式状态
    Damp_Angle_pre = Angle;
    Damp_Speed_pre = 0.0f;
    Damp_Last_Torque = 0.0f;
    
    // 初始模式设为棘轮模式
    g_knob_mode = 0; 
    
    // 初始化时清空力矩，电角度归一化
    setPhaseVoltage_FOC2(0.0f, 0.0f, Angle*7 - Zero_Angle);
}
