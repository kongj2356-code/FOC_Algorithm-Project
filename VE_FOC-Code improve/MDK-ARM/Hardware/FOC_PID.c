#include "FOC_PID.h"

/**
 * @brief 电流环PI调节（D/Q轴双PI控制，FOC核心电流环）
 * @note  Q轴：转矩电流环；D轴：励磁电流环
 *        输入：电流参考值、电流反馈值
 *        输出：SVPWM调制电压(Vq/Vd)
 */
void PID_C(void)
{
    // 获取当前D/Q轴电流反馈值（来自SVPWM采样）
    Cur_Uq.Fbk = Svpwm.Vq;
    Cur_Ud.Fbk = Svpwm.Vd;

    /************************** Q轴电流PI控制 **************************/
    Cur_Uq.Err = Cur_Uq.Ref - Cur_Uq.Fbk;                    // 计算电流偏差：期望值 - 实际值
    Cur_Uq.Ui += Cur_Uq.Err * Cur_Uq.Ki;                     // 积分项累加：误差 * 积分系数

    // 积分限幅（防止积分饱和，保证系统稳定）
    if (Cur_Uq.Ui > Cur_Uq.ErrsumMax)
    {
        Cur_Uq.Ui = Cur_Uq.ErrsumMax;
    }
    else if (Cur_Uq.Ui < Cur_Uq.ErrsumMin)
    {
        Cur_Uq.Ui = Cur_Uq.ErrsumMin;
    }

    Cur_Uq.Out = Cur_Uq.Kp * Cur_Uq.Err + Cur_Uq.Ui;           // PI输出：比例项 + 积分项

    // 输出限幅（限制电压输出范围，保护硬件）
    if (Cur_Uq.Out > Cur_Uq.OutMax)
    {
        Cur_Uq.Out = Cur_Uq.OutMax;
    }
    else if (Cur_Uq.Out < Cur_Uq.OutMin)
    {
        Cur_Uq.Out = Cur_Uq.OutMin;
    }

    /************************** D轴电流PI控制 **************************/
    Cur_Ud.Err = Cur_Ud.Ref - Cur_Ud.Fbk;                       // 计算电流偏差
    Cur_Ud.Ui += Cur_Ud.Err * Cur_Ud.Ki;                        // 积分项累加

    // 积分限幅
    if (Cur_Ud.Ui > Cur_Ud.ErrsumMax)
    {
        Cur_Ud.Ui = Cur_Ud.ErrsumMax;
    }
    else if (Cur_Ud.Ui < Cur_Ud.ErrsumMin)
    {
        Cur_Ud.Ui = Cur_Ud.ErrsumMin;
    }

    Cur_Ud.Out = Cur_Ud.Kp * Cur_Ud.Err + Cur_Ud.Ui;           // PI输出计算

    // 输出限幅
    if (Cur_Ud.Out > Cur_Ud.OutMax)
    {
        Cur_Ud.Out = Cur_Ud.OutMax;
    }
    else if (Cur_Ud.Out < Cur_Ud.OutMin)
    {
        Cur_Ud.Out = Cur_Ud.OutMin;
    }

    // PI输出赋值给SVPWM模块，作为电机控制电压
    Svpwm.Vq = Cur_Uq.Out;
    Svpwm.Vd = Cur_Ud.Out;
}


/**
 * @brief 速度环PI调节
 * @note  三环控制（位置-速度-电流）中间环
 *        输出作为电流环Q轴参考值
 */
void PID_V(void)
{
    Vel_PI.Fbk = Vec_pos_data.speed;                            // 获取电机实时速度反馈

    Vel_PI.Err = Vel_PI.Ref - Vel_PI.Fbk;                       // 计算速度偏差
    Vel_PI.Ui += Vel_PI.Err * Vel_PI.Ki;                        // 积分项累加

    // 积分限幅（抗积分饱和）
    if (Vel_PI.Ui > Vel_PI.ErrsumMax)
    {
        Vel_PI.Ui = Vel_PI.ErrsumMax;
    }
    else if (Vel_PI.Ui < Vel_PI.ErrsumMin)
    {
        Vel_PI.Ui = Vel_PI.ErrsumMin;
    }

    Vel_PI.Out = Vel_PI.Kp * Vel_PI.Err + Vel_PI.Ui;           // 速度环PI输出

    // 输出限幅（限制最大转矩电流给定）
    if (Vel_PI.Out > Vel_PI.OutMax)
    {
        Vel_PI.Out = Vel_PI.OutMax;
    }
    else if (Vel_PI.Out < Vel_PI.OutMin)
    {
        Vel_PI.Out = Vel_PI.OutMin;
    }
}


/**
 * @brief 位置环PI调节
 * @note  三环控制最外环
 *        输出作为速度环参考值
 */
void PID_P(void)
{
    Pos_PI.Fbk = Vec_pos_data.position;                        // 获取电机实时位置反馈

    Pos_PI.Err = Pos_PI.Ref - Pos_PI.Fbk;                      // 计算位置偏差
    Pos_PI.Ui += Pos_PI.Err * Pos_PI.Ki;                        // 积分项累加

    // 积分限幅
    if (Pos_PI.Ui > Pos_PI.ErrsumMax)
    {
        Pos_PI.Ui = Pos_PI.ErrsumMax;
    }
    else if (Pos_PI.Ui < Pos_PI.ErrsumMin)
    {
        Pos_PI.Ui = Pos_PI.ErrsumMin;
    }

    Pos_PI.Out = Pos_PI.Kp * Pos_PI.Err + Pos_PI.Ui;           // 位置环PI输出

    // 输出限幅（限制最大目标速度）
    if (Pos_PI.Out > Pos_PI.OutMax)
    {
        Pos_PI.Out = Pos_PI.OutMax;
    }
    else if (Pos_PI.Out < Pos_PI.OutMin)
    {
        Pos_PI.Out = Pos_PI.OutMin;
    }
}

