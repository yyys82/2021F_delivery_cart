/*
 * 梁山派软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：club.szlcsc.com
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-24     LCKFB-yzh    first version
 */
#include "positional_pid.h"
#include <math.h>

#define ABS(x) ((x > 0) ? x : -x)

static void positional_pid_params_init(positional_pid_params_t *positional_pid,
                                  float _kp, float _ki, float _kd,
                                  float _dead_zone, float _output_max,
                                  float _output_min)
{
    // 初始化 PID 参数
    positional_pid->kp = _kp;
    positional_pid->ki = _ki;
    positional_pid->kd = _kd;

    // 初始化死区、输出上限和输出下限
    positional_pid->dead_zone = _dead_zone;
    positional_pid->output_max = _output_max;
    positional_pid->output_min = _output_min;

    // 初始化目标值和输出值
    positional_pid->target = 0;
    positional_pid->output = 0;
}

static void positional_pid_set_value(positional_pid_params_t *positional_pid,
                                     float _kp, float _ki, float _kd,
                                     float _separation_threshold,
                                     float _integral_limit_max,
                                     float _integral_limit_min)
{
    // 设置 PID 参数
    positional_pid->kp = _kp;
    positional_pid->ki = _ki;
    positional_pid->kd = _kd;

    positional_pid->separation_threshold = _separation_threshold;
    positional_pid->integral_limit_max = _integral_limit_max;
    positional_pid->integral_limit_min = _integral_limit_min;
}


float positional_pid_compute(positional_pid_params_t *positional_pid,
                             float _target, float _measure)
{
    if (positional_pid->control == PID_ENABLE)
    {
        // 设置目标值和测量值
        positional_pid->target = _target;
        positional_pid->measure = _measure;

        // 计算误差
        positional_pid->error =
            positional_pid->target - positional_pid->measure;

        if (ABS(positional_pid->error) > positional_pid->dead_zone)
        {
            // 计算比例项
            positional_pid->p_out = positional_pid->kp * positional_pid->error;
            // 计算微分项
            positional_pid->d_out =
                positional_pid->kd
                * (positional_pid->error - positional_pid->last_error);

            // 积分分离，如果目标值和测量值（也就是误差）过大，积分快速增大会造成系统不稳定。
            // 只有误差小于积分分离阈值，才进行积分计算
            if (ABS(positional_pid->error) < positional_pid->separation_threshold)
            {
                // 计算积分项
                positional_pid->i_out += positional_pid->ki * positional_pid->error;
                // 积分限幅
                if (positional_pid->i_out > positional_pid->integral_limit_max)
                {
                    positional_pid->i_out = positional_pid->integral_limit_max;
                }
                if (positional_pid->i_out < positional_pid->integral_limit_min)
                {
                    positional_pid->i_out = positional_pid->integral_limit_min;
                }
            }
            else
            {
                positional_pid->i_out = 0.0f;
            }

            positional_pid->last_error = positional_pid->error;

            // 计算总输出
            positional_pid->output = positional_pid->p_out
                                     + positional_pid->i_out
                                     + positional_pid->d_out;
        }

        // 限制输出在输出上限和输出下限之间
        if (positional_pid->output > positional_pid->output_max)
        {
            positional_pid->output = positional_pid->output_max;
        }
        if (positional_pid->output < (positional_pid->output_min))
        {
            positional_pid->output = positional_pid->output_min;
        }

        // 更新上一次测量值、输出值和误差值
        positional_pid->last_error = positional_pid->error;

        return positional_pid->output;
    }
    else
    {
        return 0.0f;
    }
}

void positional_pid_control(positional_pid_params_t *positional_pid,positional_pid_status _status)
{
    // 控制 PID 的使能状态
    positional_pid->control = _status;
}

void positional_pid_init(positional_pid_params_t *positional_pid, float kp,
                         float ki, float kd, float dead_zone, float output_max,
                         float output_min)
{
    // 初始化 PID 控制器
    positional_pid->positional_pid_params_init = positional_pid_params_init;
    positional_pid->positional_pid_set_value = positional_pid_set_value;
    positional_pid->positional_pid_control = positional_pid_control;
    // 调用初始化函数设置参数
    positional_pid->positional_pid_params_init(
        positional_pid, kp, ki, kd, dead_zone, output_max, output_min);
    // 默认使能 PID 控制器
    positional_pid->control = PID_ENABLE;
}
