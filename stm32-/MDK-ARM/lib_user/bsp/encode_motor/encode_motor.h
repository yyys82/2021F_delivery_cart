#ifndef __ENCODE_MOTOR_H
#define __ENCODE_MOTOR_H
#include "stm32f4xx_hal.h"

// PID结构体定义（支持位置环和速度环）
typedef struct {
    float Kp, Ki, Kd;      // PID参数
    float target;          // 目标值
    float measure;         // 测量值
    float error;           // 当前误差
    float last_error;      // 上一次误差
    float integral;        // 积分项
    float output;          // 输出值
    float integral_limit;  // 积分限幅
    float output_limit;    // 输出限幅
	  
} PID_Controller;

extern PID_Controller pos_pid, vel_pid;  // 位置环和速度环PID
extern PID_Controller pos_pid1, vel_pid1;  // 位置环和速度环PID

void PID_Init(void);
void PID_Init_params(PID_Controller *pid,float Kp,float Ki,float Kd,float integral_limit,float output_limit);
float Velocity_PID_Calculate(PID_Controller *pid,float target,float measure);
float Position_PID_Calculate(PID_Controller *pid,float target,float measure);



#endif