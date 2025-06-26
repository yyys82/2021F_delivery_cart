#ifndef __BSP_ENCODE_H__
#define __BSP_ENCODE_H__

#include "stm32f4xx_hal.h"



typedef struct Encode_t 
{  


   int16_t _current_count;	// ¼õËÙ±È30:1
	 int16_t _delta_count; 
	 int16_t _last_count; 
	
   float _motor_speed_mm;
	 float _motor_distance_mm;
	 float _motor_speed_r;


}Encode_t;


void bsp_encode_init(void);
void Encode_Task(Encode_t * encodeA,Encode_t * encodeB);
void Encode_Calculated_distance_r(Encode_t * encodeA);

#endif //__BSP_MOTOR_H__
