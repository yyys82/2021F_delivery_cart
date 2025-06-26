#include "APP_beep_led_key.h"

void led_state(uint16_t led_state)
{
	if(led_state==1)
	{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);}
	else{
	{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);}
	
	}
}

enum key_t
{
	wait_down,
	down,
	up,
	_stop,

};

static enum key_t key;
//key__E02
//void 
 
static uint16_t app_scan_key=0; //定时器回调函数，已经配置为10ms触发一次定时器中断回调

key_state_t g_key[4] = {0};  //按键状态全局变量
 

void enable_key(){app_scan_key=1;}
void unenable_key(){app_scan_key=0;}
void APP_Scan_Key()
{
	if(app_scan_key==1){
    
        static int32_t press_time_cnt[4] = {0};    //按下持续计数值
        
        g_key[0].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);   //读取引脚电平值
        g_key[1].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);        
        g_key[2].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);        
        g_key[3].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);    
       
        for(int i=0; i<4; i++)
        {
            switch(g_key[i].judge_flag)
            {
                case JUDGE:
                    if(g_key[i].level_state == GPIO_PIN_SET)    //主要用于消抖
                    {
                        g_key[i].judge_flag = 1;
                    }
                    break;
            
                case SINGLE://第一次按下，长按、短按
                    if(g_key[i].level_state == GPIO_PIN_SET)        //按键按下
                    {
                        press_time_cnt[i]++;
                    }else if(g_key[i].level_state == GPIO_PIN_RESET)    //按键松开
                    {
                        if(press_time_cnt[i] >= 50)        //长按（按下持续时间>=500ms）
                        {
                            g_key[i].long_flag  = 1;    //长按标志置1
                            g_key[i].short_flag = 0;
                            g_key[i].two_flag   = 0;
                        }else if(press_time_cnt[i] >= 7)        //短按
                        {
                            g_key[i].short_flag = 1;    //短按标志置1
                            g_key[i].long_flag  = 0;
                            g_key[i].two_flag   = 0;
                        }
                        g_key[i].judge_flag = 0;
                        press_time_cnt[i] = 0;        //清空按下和松开间的计数
                    }
                    break;
            }
        
        }
        
			}
 
}
