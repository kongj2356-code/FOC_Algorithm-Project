#ifndef __COMMON_H
#define __COMMON_H

#include "header.h"

/*专门防止结构体定义*/

/*滤波器结构体*/
typedef struct{
	float Ts;        //采样周期(s)
	float Fc;        //截至频率(hz)
	float Alpha;     //滤波系数
	float ThisVal1,ThisVal2,ThisVal3;   //本次滤波值
	float LastVal1,LastVal2,LastVal3;   //上次滤波值
	float OutVal;    //输出值
}Filter_t;


/*Svpwm结构体*/
typedef struct{
	float Valpha,Vbeta,Ibeta,Ialpha;	//中间变量
	float Va,Vb,Vc,X,Y,Z;				      //中间变量
	float Vd,Vq,Ta,Tb,Tc;				      //输出Vq,Vd,三相作用时间,
	float Id,Iq,Tcomp1,Tcomp2,Tcomp3;	//电流Id,Iq,最终输出的比较值
	float ad1,ad2,ad3;					//
	float AD_A,AD_B,AD_C;				//
	float Ts,Tpwm;					  	//
	float Ia,Ib,Ic;						      //采集到的三相电流值
	float elec_angle;					      //电角度
	uint8_t pole_pairs,sector;			//极对数与扇区
	float Vdc;							        //供电电压
	//<flag and counting>
	uint8_t zerobias_done;		      //零点校准标志位
	uint8_t curbias_done;		        //电流校准标志位
	uint8_t global_count;
	uint8_t cur_count;
	uint8_t speed_loop_count;
	uint8_t position_loop_count;
}SVPWM_t;

//电机本体参数结构体
typedef struct{
	float zero_encoder_angle;	  //零点
	float wire_inductor;		  //线电感2.8mH
	float Phase_inductor;		  //相电感0.86mH
	float Flux;					      //
	float wire_res;				    //线电阻
	float phase_res;			    //相电阻
}Motor_t;

//电流偏置校准结构体
typedef struct{
	double samples;				        //采样个数
	double sum1,sum2,sum3; 	     	//总和
	double adc_Value1,adc_Value2,adc_Value3;  //
//	double Vol_bias1,Vol_bias2,Vol_bias3;	  //
	double DC_bias1,DC_bias2,DC_bias3;		    //直流偏置
}Cal_Cur_t;

//12位磁编码器数据结构体
typedef struct {
    uint16_t raw_angle;    	   // 原始角度 (0-4095)
	  uint16_t angle;            // 处理后的角度 (0-4095),在5600的读取函数中用到了
	  uint8_t status;            // 状态寄存器
    uint8_t agc;               // AGC值
    uint16_t magnitude;        // 磁场强度
    uint8_t magnet_detected;   // 磁铁检测标志
	  float mec_angle;		       // 真正的机械角度
}Encoder_t;

//PID参数结构体
typedef struct{
	float Kp,Ki,Kd;	      	// 参数: 比例环路,积分增益,微分系数
	float Ref,Fbk;	  	    // 输入: 参考设定点,反馈值
	float Out;	   	  	    // 输出: 控制器输出
	float Err,ErrLast;      // 数据: 误差,上次误差
	float AllowErr;   	    // 输入:允许误差
	float Ui;	              // 数据:积分项
	float ErrsumMax,ErrsumMin;  //参数: 积分上下限饱和
	float OutMax,OutMin;		   //参数: 输出上下限饱和
}PID_t;


//速度环与位置环结构体
typedef struct{
	float last_angle; 	 //上一个角度
	float now_angle;	   //现在的角度
	float fre_interrupt; //中断频率
	float speed;		     //速度
	float last_speed;	   //上一个速度
	float position;		   //位置
	float last_position; //上一个位置
}Vel_Pos_t;


#endif
