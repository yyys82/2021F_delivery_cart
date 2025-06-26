#ifndef __APP_PID_H__
#define __APP_PID_H__

#include "main_config.h"

void enable_track_pid(void) ; 
void unenable_track_pid(void);
void enable_position_pid(void); 
void unenable_position_pid(void);


void set_track_speed(float _speed);

void car_spin_position(int32_t _position_target);
void car_go_position(int32_t _position_target);


void set_car_position_max_speed(float _speed);


#endif // __SOFT_TIMER_H__
