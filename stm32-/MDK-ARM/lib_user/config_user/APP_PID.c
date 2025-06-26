///////////////////////////////////
//app_pid


#include "APP_PID.h"


Encode_t encodeA;
Encode_t encodeB;


extern uint16_t data1;
extern uint16_t data2;
extern uint16_t data3;

//
PID_Controller Pid_6050;
//速度pid
PID_Controller motor1_speed_pid;
PID_Controller motor2_speed_pid;
//位置pid
PID_Controller motor1_position_pid;
PID_Controller motor2_position_pid;
//k210循迹pid
PID_Controller motor_track_pid;




//状态机

typedef enum
{
    APP_PID_INIT = 0,
    APP_PID_RUN,
    APP_PID_STOP
} app_pid_status_t;

app_pid_status_t app_pid_status = APP_PID_INIT;




float  base_speed=0.00;

// 小车电机位置环开启标志位状态
__attribute__((used)) static char car_position_pid_enable_flag = 0;
// 小车巡红线环开启标志位状态
__attribute__((used)) static char car_find_red_line_enable_flag = 0;

void set_track_speed(float _speed)
{
	base_speed=_speed;
}



void app_pid_run()
{
	

	switch (app_pid_status)
	{
		case APP_PID_INIT:
			//初始化速度pid
		 PID_Init_params(&motor1_speed_pid,120,22,5,30,999);
		 PID_Init_params(&motor2_speed_pid,120,22,5,30,999);
		
     PID_Init_params(&motor2_position_pid,0.2,0,0.1,10,999);
		 PID_Init_params(&motor1_position_pid,0.2,0,0.1,10,999);
		
		
		 PID_Init_params(&motor_track_pid,0.005f,0,0.02f,5,1);
		 //set_track_speed(1);      	
		 app_pid_status++;
    
		
		 break;
		case APP_PID_RUN:
			
			
		  Position_PID_Calculate(&motor2_speed_pid,motor2_speed_pid.target,encodeB._motor_speed_r);
		  Position_PID_Calculate(&motor1_speed_pid,motor1_speed_pid.target,encodeA._motor_speed_r);
		
		  motor1_pwm_value_set(motor1_speed_pid.output);
		  motor2_pwm_value_set(motor2_speed_pid.output);
		
		
		if(car_position_pid_enable_flag==1)
		{
			 // Encode_Calculated_distance_r(&encodeA);
			 // Encode_Calculated_distance_r(&encodeB);
			  Position_PID_Calculate(&motor1_position_pid,(float)motor1_position_pid.target,(float)encodeA._motor_distance_mm);
				Position_PID_Calculate(&motor2_position_pid,(float)motor2_position_pid.target,(float)encodeB._motor_distance_mm);
		
		}
		else
		{
			motor1_position_pid.output=0;
		  motor2_position_pid.output=0;
		
		}
		if(car_find_red_line_enable_flag==1)
		{
			  Position_PID_Calculate(&motor_track_pid,400,data1);
		
		
		
		}
		else
		{
			motor_track_pid.output=0;
	
		
		}
		
		
		
		
		
		
		 motor1_speed_pid.target=  base_speed+motor1_position_pid.output+  motor_track_pid.output;
		 motor2_speed_pid.target=  base_speed+motor2_position_pid.output-  motor_track_pid.output;
		
		  break;
		case APP_PID_STOP:
		break;

	
	}



		

			


}





//开关循迹
void enable_track_pid(void)   {car_find_red_line_enable_flag=1;}
void unenable_track_pid(void)  {  car_find_red_line_enable_flag=0;}
//
void enable_position_pid(void)   {car_position_pid_enable_flag=1;}
void unenable_position_pid(void)  {  car_position_pid_enable_flag=0;}





uint16_t car_control_k210_to_find_number(void)
{
	return 1;

}
uint16_t car_control_k210_to_find_crossing(void)
{
	return 1;

}

void car_go_position(int32_t _position_target)
{
    motor1_position_pid.target += _position_target;
    motor2_position_pid.target -= _position_target;
}
// 位置式旋转,负数左转，正数右转。
void car_spin_position(int32_t _position_target)
{
    motor1_position_pid.target += -_position_target;
    motor2_position_pid.target -= _position_target;
}
void set_car_position_max_speed(float _speed)
{
    motor1_position_pid.output_limit = _speed;
    motor2_position_pid.output_limit = _speed;

    motor1_position_pid.output_limit = -_speed;
    motor2_position_pid.output_limit = -_speed;
}
