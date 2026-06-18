#include "EKF.h"

// 定义卡尔曼滤波器结构体
KalmanFilter ultrasonic_filter;

/**
  * @brief  初始化卡尔曼滤波器
  * @param  kf: 卡尔曼滤波器结构体指针
  * @param  Q: 过程噪声协方差
  * @param  R: 测量噪声协方差  
  * @param  initial_value: 初始状态值
  * @retval 无
  */
void EKF_Init(KalmanFilter* kf, float Q, float R, float initial_value)
{
    kf->Q = Q;              // 过程噪声协方差
    kf->R = R;              // 测量噪声协方差
    kf->x = initial_value;  // 初始状态值
    kf->P = 1.0f;           // 初始状态协方差
    kf->K = 0.0f;           // 初始卡尔曼增益
    kf->A = 1.0f;           // 状态转移矩阵（一维系统通常为1）
    kf->H = 1.0f;           // 观测矩阵（一维系统通常为1）
}


/**
  * @brief  卡尔曼滤波更新
  * @param  kf: 卡尔曼滤波器结构体指针
  * @param  measurement: 测量值
  * @retval 滤波后的值
  */
float EKF_Update(KalmanFilter* kf, float measurement)
{
    // 预测步骤
    kf->x = kf->A * kf->x;              // 状态预测
    kf->P = kf->A * kf->P * kf->A + kf->Q; // 协方差预测
    
    // 更新步骤
    kf->K = kf->P * kf->H / (kf->H * kf->P * kf->H + kf->R); // 计算卡尔曼增益
    kf->x = kf->x + kf->K * (measurement - kf->H * kf->x);   // 状态更新
    kf->P = (1 - kf->K * kf->H) * kf->P;                     // 协方差更新
    
    return kf->x;
}


/**
  * @brief  设置过程噪声协方差
  * @param  kf: 卡尔曼滤波器结构体指针
  * @param  Q: 过程噪声协方差
  * @retval 无
  */
void EKF_SetProcessNoise(KalmanFilter* kf, float Q)
{
    kf->Q = Q;
}


/**
  * @brief  设置测量噪声协方差
  * @param  kf: 卡尔曼滤波器结构体指针
  * @param  R: 测量噪声协方差
  * @retval 无
  */
void EKF_SetMeasurementNoise(KalmanFilter* kf, float R)
{
    kf->R = R;
}


/**
  * @brief  获取当前滤波后的值
  * @param  kf: 卡尔曼滤波器结构体指针
  * @retval 当前滤波值
  */
float EKF_GetFilteredValue(KalmanFilter* kf)
{
    return kf->x;
}

