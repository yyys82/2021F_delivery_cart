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
#ifndef _POSITIONAL_PID_H
#define _POSITIONAL_PID_H

#include "stdint.h"

typedef enum {
    PID_DISABLE, /* PID失能 */
    PID_ENABLE, /* PID使能 */
} positional_pid_status;

typedef struct _positional_pid_params_t
{

    char control;

    float kp;
    float ki;
    float kd;

    float target;
    float measure;

    float error;
    float last_error;

    float p_out;
    float i_out;
    float d_out;

    float output;

    float output_max;
    float output_min;

    //积分分离阈值
    float separation_threshold;
    //积分限幅数值
    float integral_limit_max;
    float integral_limit_min;

    float dead_zone;

    void (*positional_pid_params_init)(
        struct _positional_pid_params_t *positional_pid, float kp, float ki,
        float kd, float dead_zone, float output_max, float output_min);
    void (*positional_pid_set_value)(
        struct _positional_pid_params_t *positional_pid, float kp, float ki,
        float kd, float separation_threshold, float integral_limit_max,
        float integral_limit_min);
    void (*positional_pid_control)(
        struct _positional_pid_params_t *positional_pid,
        positional_pid_status status);
} positional_pid_params_t;

void positional_pid_init(positional_pid_params_t *positional_pid, float kp,
                         float ki, float kd,

                         float dead_zone, float output_max, float output_min);
float positional_pid_compute(positional_pid_params_t *positional_pid,
                            float target, float measure);

#endif
