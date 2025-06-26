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


#include <bsp_motor.h>


/**
 -  @brief  电机驱动的PWM控制引脚初始化
 -  @note   None
 -  @param  None
 -  @retval None
   */

/**
 -  @brief  电机驱动的硬件定时器初始化
 -  @note   该函数将两个电机定时器的频率设置为60KHz，该项目所用到的电机驱动芯片AT8870最高可以接收100KHz的频率。
 -  @param  None
 -  @retval None
   */
static void motor_timer_init(void)
{
	
	HAL_TIM_PWM_Start(&htim_A,TIM_CHANNEL_A1);
	HAL_TIM_PWM_Start(&htim_A,TIM_CHANNEL_A2);
	HAL_TIM_PWM_Start(&htim_B,TIM_CHANNEL_B1);
	HAL_TIM_PWM_Start(&htim_B,TIM_CHANNEL_B2);
    
}


void motor1_pwm_value_set(int16_t value)
{
	//输入限幅
	   if (value > 1000)
    {
        value = 1000;
    }
    else if (value < -1000)
    {
        value = -1000;
    }
  //    

			
				 __HAL_TIM_SetCompare(&htim_A, TIM_CHANNEL_A1, 500-value/2);
	       __HAL_TIM_SetCompare(&htim_A, TIM_CHANNEL_A2, 500+value/2);
		

 
}

/**
 -  @brief  电机2的PWM调节，也就是速度调节
 -  @note   None
 -  @param  value:范围-1000~+1000,正数是正传，负数是反转
 -  @retval None
   */
void motor2_pwm_value_set(int16_t value)
{
 	   if (value > 1000)
    {
        value = 1000;
    }
    else if (value < -1000)
    {
        value = -1000;
    }

			 __HAL_TIM_SetCompare(&htim_B, TIM_CHANNEL_B1, 500+value/2);
	     __HAL_TIM_SetCompare(&htim_B, TIM_CHANNEL_B2, 500-value/2);

}

 /**
  -  @brief  电机初始化
  -  @note   None
  -  @param  None
  -  @retval None
    */
void bsp_motor_init(void)
{
    
    motor_timer_init();
    return;
}
