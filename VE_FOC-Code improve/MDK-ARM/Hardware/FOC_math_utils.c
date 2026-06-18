#include "FOC_math_utils.h"

/****************************************************************************************************
 * 文件功能说明：
 * 1. 基于查表法的快速 sin/cos 计算（FOC 坐标变换专用）
 * 2. 更高精度的快速多项式 sin/cos
 * 3. 角度归一化、电角度计算（FOC 核心）
 * 4. 三相电流低通滤波 LP Filter
 * 5. 定时器计时（用于测量代码执行时间）
 ****************************************************************************************************/

//======================== 1. 查表法快速正弦值 ========================
// 使用 int 数组代替 float，节省 50% 存储空间
// int = 2 字节，float = 4 字节
// 数值放大 10000 倍存储，使用时再除以 10000 还原
// 表长度 200 点，对应 0~90° 正弦值

const int sine_array[200] = {
    0,79,158,237,316,395,473,552,631,710,
    789,867,946,1024,1103,1181,1260,1338,1416,1494,
    1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,
    2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,
    3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,
    3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,
    4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,
    5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,
    5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,
    6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,
    7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,
    7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,
    8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,
    8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,
    8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,
    9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,
    9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,
    9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,
    9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,
    9975,9980,9985,9989,9992,9995,9997,9999,10000,10000
};

/**
 * @brief  查表法快速正弦函数 sin(a)
 * @param  a 输入角度（弧度 0~2PI）
 * @return 正弦值（-1 ~ 1）
 * @note   精度 ±0.005，速度极快，适合 FOC
 */
float _sin(float a){
    if(a < _PI_2){
        // 0 ~ 90° 直接查表
        return 0.0001f*sine_array[_round(126.6873f* a)];
    }else if(a < _PI){
        // 90 ~ 180°，对称查表
        return 0.0001f*sine_array[398 - _round(126.6873f*a)];
    }else if(a < _3PI_2){
        // 180 ~ 270°，负值
        return -0.0001f*sine_array[-398 + _round(126.6873f*a)];
    } else {
        // 270 ~ 360°，负值
        return -0.0001f*sine_array[796 - _round(126.6873f*a)];
    }
}

/**
 * @brief  快速余弦函数 cos(a)
 * @param  a 输入角度（弧度）
 * @return 余弦值
 * @note   cos(a) = sin(a + 90°)
 */
float _cos(float a){
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}

//======================== 2. 角度归一化工具函数 ========================

/**
 * @brief  弧度角归一化到 [0, 2PI]，精度低
 */
float normalize_radianAngle(float angle){
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

/**
 * @brief  角度归一化（角度制 → 弧度制）
 * @param  angle_degrees 角度（-∞ ~ +∞）
 * @return 归一化弧度 [0, 2PI]
 */
float _normalizeAngle(float angle_degrees)
{
    // 先归一到 0~360 度
    angle_degrees = fmodf(angle_degrees, 360.0f);
    if (angle_degrees < 0.0f) {
        angle_degrees += 360.0f;
    }
    else if(angle_degrees > 0.0f){
        angle_degrees -= 360.0f;
    }
    // 转为弧度
    float angle = (1.0f*angle_degrees*PI)/180.0f;
    return angle;
}

/**
 * @brief  弧度角归一化到 [0, 2PI]，精度高
 */
float normalize_angle(float angle) {
    double normalized = fmod(angle, 2 * M_PI);
    if (normalized < 0) {
        normalized += 2 * M_PI;
    }
    return normalized;
}

//======================== 3. 电角度计算（FOC 核心） ========================

/**
 * @brief  机械角 → 电角度
 * @param  shaft_angle 机械角度（弧度）
 * @param  pole_pairs   极对数
 * @return 电角度 = 机械角度 × 极对数
 */
float ElectriAngle(float shaft_angle,int pole_pairs){
    return (shaft_angle * pole_pairs);
}

//======================== 4. 高精度快速正弦多项式 ========================
// 多项式拟合 sin/cos，比查表精度更高，速度也很快
// 用于需要高精度的 FOC 场合

float f1(float x) {
  float u = 1.3528548e-10f;
  u = u * x + -2.4703144e-08f;
  u = u * x + 2.7532926e-06f;
  u = u * x + -0.00019840381f;
  u = u * x + 0.0083333179f;
  return u * x + -0.16666666f;
}

float f2(float x) {
  float u = 1.7290616e-09f;
  u = u * x + -2.7093486e-07f;
  u = u * x + 2.4771643e-05f;
  u = u * x + -0.0013887906f;
  u = u * x + 0.041666519f;
  return u * x + -0.49999991f;
}

/**
 * @brief  高精度快速正弦
 */
float fast_sin(float x){
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * M_PI;
  if (si & 1) {
    x = x > 0.0f ? x - M_PI : x + M_PI;
  }
  return x + x * x * x * f1(x * x);
}

/**
 * @brief  高精度快速余弦
 */
float fast_cos(float x) {
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * M_PI;
  if (si & 1) {
    x = x > 0.0f ? x - M_PI : x + M_PI;
  }
  return 1.0f + x * x * f2(x * x);
}

/**
 * @brief  同时计算 sin 和 cos（FOC 最常用，效率最高）
 * @param  x 角度
 * @param  sin_x 输出正弦
 * @param  cos_x 输出余弦
 */
void fast_sin_cos(float x, float *sin_x, float *cos_x) {
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * M_PI;
  if (si & 1) {
    x = x > 0.0f ? x - M_PI : x + M_PI;
  }
  *sin_x = x + x * x * x * f1(x * x);
  *cos_x = 1.0f + x * x * f2(x * x);
}

//======================== 5. 三相电流低通滤波（一阶滤波） ========================

/**
 * @brief  三相电流 Ia Ib Ic 低通滤波
 * @param  p 滤波参数结构体（Alpha 系数）
 * @param  p1 SVPWM 电流结构体
 * @note   一阶低通：Out = α*In + (1-α)*Last_Out
 */
void LP_IaIbIc(Filter_t *p,SVPWM_t* p1)
{
    // Ia 滤波
	p->ThisVal1 = p1->Ia;
	p->OutVal = p->Alpha * p->ThisVal1 + (1 - p->Alpha) * p->LastVal1;
	p->LastVal1 = p->OutVal;
	p1->Ia = p->OutVal;

    // Ib 滤波
	p->ThisVal2 = p1->Ib;
	p->OutVal = p->Alpha * p->ThisVal2 + (1 - p->Alpha) * p->LastVal2;
	p->LastVal2 = p->OutVal; 
	p1->Ib = p->OutVal;

    // Ic 滤波
	p->ThisVal3 = p1->Ic;
	p->OutVal = p->Alpha * p->ThisVal3 + (1 - p->Alpha) * p->LastVal3;
	p->LastVal3 = p->OutVal;
	p1->Ic = p->OutVal;
}

//======================== 6. 代码执行时间测量 ========================

/**
 * @brief  开始计时（TIM6 计数器清零）
 */
void measuretime_start(void)
{
	TIM6->CNT = 0;
}

/**
 * @brief  获取计时值
 * @return 定时器计数值
 */
uint32_t measuretime_acquire(void)
{
	return TIM6->CNT;
}
