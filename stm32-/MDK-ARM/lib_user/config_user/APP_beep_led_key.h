
#ifndef __APP_BEEP_LED_KEY_H__
#define __APP_BEEP_LED_KEY_H__

#include "main_config.h"


void led_state(uint16_t led_state);


//按键状态结构体
typedef struct key_state{
    int8_t short_flag;        //短按flag
    int8_t long_flag;         //长按flag
 
    int8_t level_state;       //电平状态    
 
    int8_t judge_flag;        //裁决抖动
    int32_t press_time_cnt;   //按下时间
	
	  uint8_t two_flag; 
    }key_state_t;
 
enum {JUDGE = 0, SINGLE = 1};//状态机的状态类型
 
extern key_state_t g_key[4];  //按键状态全局变量
void APP_Scan_Key();
void unenable_key();
void enable_key();
#endif // __SOFT_TIMER_H__
