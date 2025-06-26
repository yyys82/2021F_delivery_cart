
#ifndef __APP_BEEP_LED_KEY_H__
#define __APP_BEEP_LED_KEY_H__

#include "main_config.h"


void led_state(uint16_t led_state);


//����״̬�ṹ��
typedef struct key_state{
    int8_t short_flag;        //�̰�flag
    int8_t long_flag;         //����flag
 
    int8_t level_state;       //��ƽ״̬    
 
    int8_t judge_flag;        //�þ�����
    int32_t press_time_cnt;   //����ʱ��
	
	  uint8_t two_flag; 
    }key_state_t;
 
enum {JUDGE = 0, SINGLE = 1};//״̬����״̬����
 
extern key_state_t g_key[4];  //����״̬ȫ�ֱ���
void APP_Scan_Key();
void unenable_key();
void enable_key();
#endif // __SOFT_TIMER_H__
