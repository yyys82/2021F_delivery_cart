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
 
static uint16_t app_scan_key=0; //��ʱ���ص��������Ѿ�����Ϊ10ms����һ�ζ�ʱ���жϻص�

key_state_t g_key[4] = {0};  //����״̬ȫ�ֱ���
 

void enable_key(){app_scan_key=1;}
void unenable_key(){app_scan_key=0;}
void APP_Scan_Key()
{
	if(app_scan_key==1){
    
        static int32_t press_time_cnt[4] = {0};    //���³�������ֵ
        
        g_key[0].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);   //��ȡ���ŵ�ƽֵ
        g_key[1].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);        
        g_key[2].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);        
        g_key[3].level_state =     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);    
       
        for(int i=0; i<4; i++)
        {
            switch(g_key[i].judge_flag)
            {
                case JUDGE:
                    if(g_key[i].level_state == GPIO_PIN_SET)    //��Ҫ��������
                    {
                        g_key[i].judge_flag = 1;
                    }
                    break;
            
                case SINGLE://��һ�ΰ��£��������̰�
                    if(g_key[i].level_state == GPIO_PIN_SET)        //��������
                    {
                        press_time_cnt[i]++;
                    }else if(g_key[i].level_state == GPIO_PIN_RESET)    //�����ɿ�
                    {
                        if(press_time_cnt[i] >= 50)        //���������³���ʱ��>=500ms��
                        {
                            g_key[i].long_flag  = 1;    //������־��1
                            g_key[i].short_flag = 0;
                            g_key[i].two_flag   = 0;
                        }else if(press_time_cnt[i] >= 7)        //�̰�
                        {
                            g_key[i].short_flag = 1;    //�̰���־��1
                            g_key[i].long_flag  = 0;
                            g_key[i].two_flag   = 0;
                        }
                        g_key[i].judge_flag = 0;
                        press_time_cnt[i] = 0;        //��հ��º��ɿ���ļ���
                    }
                    break;
            }
        
        }
        
			}
 
}
