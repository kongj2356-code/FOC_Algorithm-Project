#include "header.h"
#include "FOC_Algorithm.h"
#include "FOC_PID.h"
#include "FOC_math_utils.h"
#include "Upper.h"
#include "Control.h"

/************************ 开环测试变量 ************************/
float target_speed = 0.09f;   // 开环模式目标速度

/**
  * @brief  定时器更新中断回调函数
  * @note   TIM7 用于开环运行测试
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM7)
    {
        //显示中断频率
//       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
        //Svpwm.elec_angle += 1.0f*target_speed;   //电角度自增时不需要进行角度归一化
        // 获取编码器角度并计算电角度
        Get_angle(&Svpwm, &AS5600, &My_motor);
        Svpwm.elec_angle = _normalizeAngle(Svpwm.elec_angle);

        // 开环电压给定
        Svpwm.Vq = 1.0f;
        Svpwm.Vd = 0.0f;

        // FOC 坐标反变换
        reverse_park(&Svpwm);
        reverse_clarke(&Svpwm);

        // SVPWM 输出
        SVPWM_Calculate(&Svpwm);

        // 串口数据发送
        Sendarray_Init();
        Float_send(SendArray, 3);
    }
}



/*=============================================
  串口指令格式说明：
openloop---target_speed---Vq
torloop---Vq,Vd (torque)
velloop---speed (veclocity)
posloop---angle (position)
motorstop---nothing
=============================================*/
/**
  * @brief  串口空闲中断接收回调函数
  * @note   接收上位机指令，切换电机控制模式
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (Svpwm.curbias_done)
    {
        if (huart->Instance == USART1)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, 128);   // 接收完毕后重启串口DMA模式接收数据，又快又准
          

            if (rx_buffer[0] == 's' && rx_buffer[1] == 'm')
            {
                Operate_mode = motor_stop_mode;
            }

            if (rx_buffer[0] == 't' && rx_buffer[1] == 'm')
            {
                // 这就是电流闭环模式
                // 其中target就是指向字符串中我们想要的字符出现第一次的位置的指针
                Operate_mode = torque_mode;    // 切换到力矩模式之后,读取串口的值作为力矩环的目标值

                char *target = strchr((const char *)rx_buffer, '%');    // 在rx_buffer中寻找%的值

                if (target != NULL)
                {
                    // 如果找到了,也就是非空指针的时候,进行数据的解析,格式化输入
                    float target_Vq = 0.0f;

                    if (sscanf((const char *)target + 1, "%f", &target_Vq) == 1)
                    {
                      
                        // "%f"是我们想要的格式,target_Vq就是保存数据解析之后的变量
                        Cur_Uq.Ref = target_Vq;    // 将串口的值赋值给CUr_Uq的参考值，也就是电流环的目标值
                        lighton(1, 1);             // 力矩模式指示灯点亮
                    }
                    else
                    {
                        // 解析失败的处理
                        lighton(2, 1);
                    }
                }
            }
            else if (rx_buffer[0] == 'v' && rx_buffer[1] == 'm')
            {
                // 这就是速度闭环模式
                Operate_mode = velocity_mode;
                Vel_PI.Ui = 0;
                Pos_PI.Ui = 0;

                char *target = strchr((const char *)rx_buffer, '%');    // 在rx_buffer中寻找%的值

                if (target != NULL)
                {
                    float speed = 0.0f;

                    if (sscanf((const char *)target + 1, "%f", &speed) == 1)
                    {

                        Vel_PI.Ref = speed;    // 将串口的值赋值给Velocity_Uq的参考值，也就是速度环的目标值
                        lighton(1, 1);
                    }
                    else
                    {
                        lighton(2, 1);
                    }
                }
            }
            else if (rx_buffer[0] == 'p' && rx_buffer[1] == 'm')
            {
                // 这就是位置闭环模式
                // stop_motor();    // 这里要先停止一下，因为位置闭环的逻辑是累加位置差值
                Operate_mode = position_mode;
                Svpwm.global_count = 0;
                Svpwm.global_count++;

                char *target = strchr((const char *)rx_buffer, '%');    // 在rx_buffer中寻找%的值

                if (target != NULL)
                {
                    float position = 0.0f;

                    if (sscanf((const char *)target + 1, "%f", &position) == 1)
                    {
                    
                        // "%f"是我们想要的格式,target_Vq就是保存数据解析之后的变量
                        Pos_PI.Ref = position;    // 将串口的值赋值给Velocity_Uq的参考值，也就是速度环的目标值
                        lighton(1, 1);
                    }
                    else
                    {
                        lighton(2, 1);
                    }
                }
            }

            // HAL_UART_Transmit(&huart1, rx_buffer, Size, 0xffff);  // 将接收到的数据再发出
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);        // 关闭传输完成一半的中断
            memset(rx_buffer, 0, 128);                               // 清除接收缓存
        }
    }
}




/**
  * @brief  串口错误回调函数
  * @note   出错时重新开启接收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
   if(huart->Instance == USART1)
  {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, 128);    //手动开启串口DMA模式接收数据
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		      // 手动关闭DMA_IT_HT中断
		memset(rx_buffer, 0, 128);							   // 清除接收缓存
   }
}





/*
 * 三相载波频率TIM1 = 168000000/(0+1)/(3499+1)/2 = 24K
 * RCR: 中央对齐模式下，Update事件发生在 计数到 ARR 顶点，计数到 0 底点
        故不加RCR,Update的请求频率Fupdate_raw = 2 × Fpwm = 2 × 24 kHz = 48 kHz
        RCR = 7: 每 RCR + 1 = 8 次更新请求，才真正产生一次 Update Event，Fupdate = 48 kHz / 8 = 6 kHz
   DeadTime = 5 * 1/168000000（定时器时钟周期） = 2.76ns
*/

//6k的中断频率,ADC注入组转换完成中断实现
//这个ADC的采样实际上是定时器1的更新中断触发的,直接就是硬件触发ADC采样,现在的波形很漂亮,再加上一阶低通滤波器
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{ 
   if(hadc->Instance == ADC1)
   {
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);   // 3K->6K
    switch (Operate_mode)
    {//模式选择
        case motor_stop_mode:            //电机停止模式
            stop_motor();
            break;
        case zero_point_calibrate_mode:  //零点校准模式
            break;
        case cur_bias_calibrate_mode:    //电流校准模式,只有电流校准完成之后才进行电机运行,会有标志位
			      Cur_bais_calibration(&Svpwm, &Cur_Cal);
           // Mechanical_zero_point_calibration(&Svpwm, &My_motor, &AS5600);
            break;
        case open_loop_mode://开环模式
            HAL_TIM_Base_Start_IT(&htim7);
            break;
        case torque_mode:
            Run_Motor();
            break;
        case velocity_mode:
             Run_Motor();
            break;
        case position_mode:
             Run_Motor();
            break;
        default:
            break;
    }
   }
}

