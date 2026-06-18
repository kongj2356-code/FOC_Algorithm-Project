#ifndef __EKF_H
#define __EKF_H

#include "stm32f10x.h"

// 卡尔曼滤波器结构体
typedef struct {
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float x;        // 系统状态值（滤波后的值）
    float P;        // 状态协方差
    float K;        // 卡尔曼增益
    float A;        // 状态转移矩阵（通常为1）
    float H;        // 观测矩阵（通常为1）
} KalmanFilter;

// 全局变量声明
extern KalmanFilter ultrasonic_filter;



// 函数声明
void EKF_Init(KalmanFilter* kf, float Q, float R, float initial_value);
float EKF_Update(KalmanFilter* kf, float measurement);
void EKF_SetProcessNoise(KalmanFilter* kf, float Q);
void EKF_SetMeasurementNoise(KalmanFilter* kf, float R);
float EKF_GetFilteredValue(KalmanFilter* kf);

#endif /* __EKF_H */
