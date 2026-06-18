#ifndef __CONTROL_H
#define __CONTROL_H

#include "header.h"
#include "FOC_Struct.h"
//f代表的都是单精度运算


#define ADC_TO_VOLTAGE  0.0008057f   //3.3/4096的值就是0.0008057


//模式选择枚举类型
typedef enum{
    motor_stop_mode = 0,             //电机停止模式
    zero_point_calibrate_mode,   //零点校准模式
    cur_bias_calibrate_mode,     //电流偏置校准模式
    open_loop_mode,              //开环模式
    torque_mode,                 //力矩模式---电流环
    velocity_mode,               //速度模式---速度环+电流环
    position_mode,               //位置模式---位置环+速度环+电流环
}Mode_t;


extern Mode_t Operate_mode;     //这个直接就是枚举变量，对这个进行模式选择就行
extern float SendArray[3];//发送的数组

/*灯控制函数*/
void lighton(uint8_t which,uint8_t state);
/*电流校准函数*/
void Cur_bais_calibration(SVPWM_t* svp,Cal_Cur_t* Cur);
/*电机运行*/
void Run_Motor(void);
//全局参数初始化
void Init_all_struct(void);
//发送数组初始化
void Sendarray_Init(void);
/*单独外设的初始化*/
static void Init_Motor_Params(Motor_t* motor);
static void Init_Current_Calibration(Cal_Cur_t* cur_cal);
static void Init_SVPWM(SVPWM_t* svp);
static void Init_Current_PID(PID_t* pid_q, PID_t* pid_d);
static void Init_Velocity_Data(Vel_Pos_t* vec_pos_data);
static void Init_Velocity_PID(PID_t* vel_pid);
static void Init_Position_PID(PID_t* pos_pid);
static void Init_Filter(Filter_t* filter);

//别处声明结构体
extern Encoder_t AS5600;
extern Filter_t LP_filter;
extern Vel_Pos_t Vec_pos_data;
extern SVPWM_t Svpwm;
extern Motor_t My_motor;
extern Cal_Cur_t Cur_Cal;
extern PID_t Cur_Uq;
extern PID_t Cur_Ud;
extern PID_t Vel_PI;
extern PID_t Pos_PI;

//别处定义串口接受数组与DMA接受
extern uint8_t rx_buffer[128];
extern DMA_HandleTypeDef hdma_usart1_rx;

#endif
