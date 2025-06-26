/*
* 梁山派软硬件资料与相关扩展板软硬件资料官网全部开源
* 开发板官网：www.lckfb.com
* 技术支持常驻论坛，任何技术问题欢迎随时交流学习
* 立创论坛：club.szlcsc.com
* 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
* 不靠卖板赚钱，以培养中国工程师为己任
* Change Logs:
* Date           Author       Notes
* 2023-07-21     LCKFB-yzh    first version
 */
#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "stm32f4xx_hal.h"
#include <tim.h>
//定义定时器
#define htim_A htim1
#define TIM_CHANNEL_A1 TIM_CHANNEL_1
#define TIM_CHANNEL_A2 TIM_CHANNEL_2

#define htim_B htim1
#define TIM_CHANNEL_B1 TIM_CHANNEL_3
#define TIM_CHANNEL_B2 TIM_CHANNEL_4

void bsp_motor_init(void);
void motor1_pwm_value_set(int16_t value);
void motor2_pwm_value_set(int16_t value);

#endif //__BSP_MOTOR_H__
