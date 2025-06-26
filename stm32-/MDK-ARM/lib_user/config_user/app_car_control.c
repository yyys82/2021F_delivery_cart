#include "app_car_control.h"
#include "APP_PID.h"

extern uint16_t data1;//循迹的偏差
extern uint16_t data2;//0在循迹1十字路口2空白停车位
extern uint16_t data3;//循迹摄像头正常
extern uint16_t k210_data; // 数据包就绪标志

extern Encode_t encodeA;
extern Encode_t encodeB;



extern PID_Controller motor1_speed_pid;
extern PID_Controller motor2_speed_pid;
 //位置pid
extern PID_Controller motor1_position_pid;
extern PID_Controller motor2_position_pid;
 //k210循迹pid
extern PID_Controller motor_track_pid;

uint16_t car_go_until_crossing();
uint16_t car_position_turn(float turn_angle);

//近端到十字路口
uint16_t car_go_until_crossing()
{
	
	
	typedef enum
{
    _start = 0,
    _wait_crossing,
	  _postition_start,
	  _track_start,
	  _stop,

} car_go_until_crossing_t;

  static car_go_until_crossing_t car_go_until_crossing=_start;
	
 	static uint32_t last_tick;
//
switch (car_go_until_crossing)
{
	
	case _start:
		if(data3==1)
		{
			
			set_track_speed(1);
			enable_track_pid();
			car_go_until_crossing++;
		
		}

		break;
	
	case _wait_crossing:
		if(data2==1)
		{
			
			unenable_track_pid();
			
			set_track_speed(0);
			motor1_position_pid.target=encodeA._motor_distance_mm+5;
	    motor2_position_pid.target=encodeB._motor_distance_mm+5;
			enable_position_pid();
					
	    last_tick=HAL_GetTick();
	
			car_go_until_crossing++;
			
		}
		break;
	case _postition_start:
		if(HAL_GetTick()-last_tick>=2000)
		{
		  unenable_position_pid();
			
			
			last_tick=HAL_GetTick();
			enable_track_pid();

			car_go_until_crossing++;
		}
		 
		
	
	 
	  break;
	case _track_start:
		if(HAL_GetTick()-last_tick>=3000)
		{
					unenable_track_pid();
				car_go_until_crossing++;
		
		}
		
	  break;
	case _stop:
		car_go_until_crossing=_start;
		return 1;
	
		break;
		


}
return 0;
}


















//近端到停车位置
uint16_t car_go_until_stop_postition()
{
	
	
	typedef enum
{
    _start = 0,
    _wait_crossing,
	  _postition_start,
	  _stop,

} car_go_until_crossing_t;

  static car_go_until_crossing_t car_go_until_crossing=_start;
	
 	static uint32_t last_tick;
//
switch (car_go_until_crossing)
{
	
	case _start:
	if(data3==1)
	{
			
			set_track_speed(1);
			enable_track_pid();
			car_go_until_crossing++;
		
		}

		break;
	
	case _wait_crossing:
		//2是停车标志位
		if(data2==2)
		{
			
			unenable_track_pid();
			
			set_track_speed(0);
			motor1_position_pid.target=encodeA._motor_distance_mm+5;
	    motor2_position_pid.target=encodeB._motor_distance_mm+5;
			enable_position_pid();
					
	    last_tick=HAL_GetTick();
	
			car_go_until_crossing++;
			
		}
		break;
	case _postition_start:
		if(HAL_GetTick()-last_tick>=2000)
		{
		  unenable_position_pid();
			
			
		
		

			car_go_until_crossing++;
		}
		 
	  break;

	case _stop:
		car_go_until_crossing=_start;
		return 1;
	
		break;
		


}
return 0;
}






//中端到十字路口
uint16_t car_go_until_crossing_2(uint16_t diatance)
{
	
	
	typedef enum
{
    _start = 0,
    _wait_crossing,
	  _postition_start,
	  _track_start,
	  _stop,

} car_go_until_crossing_t;

  static car_go_until_crossing_t car_go_until_crossing=_start;
	
 	static uint32_t last_tick;
//
switch (car_go_until_crossing)
{
	
	case _start:
		if(data3==1)
		{
			
			set_track_speed(1);
			enable_track_pid();
			car_go_until_crossing++;
			encodeA._motor_distance_mm=0;
			encodeB._motor_distance_mm=0;
			
		
		}

		break;
	
	case _wait_crossing:
		if((data2==1)&&(encodeB._motor_distance_mm>=diatance))
		{
			
			unenable_track_pid();
			
			set_track_speed(0);
			motor1_position_pid.target=encodeA._motor_distance_mm+5;
	    motor2_position_pid.target=encodeB._motor_distance_mm+5;
			enable_position_pid();
					
	    last_tick=HAL_GetTick();
	
			car_go_until_crossing++;
			
		}
		break;
	case _postition_start:
		if(HAL_GetTick()-last_tick>=2000)
		{
		  unenable_position_pid();
			
			
			last_tick=HAL_GetTick();
			enable_track_pid();

			car_go_until_crossing++;
		}
		 
		
	
	 
	  break;
	case _track_start:
		if(HAL_GetTick()-last_tick>=3000)
		{
					unenable_track_pid();
				car_go_until_crossing++;
		
		}
		
	  break;
	case _stop:
		car_go_until_crossing=_start;
		return 1;
	
		break;
		


}
return 0;
}





//中端到十字路口
uint16_t car_go_until_crossing_2_stop(uint16_t diatance)
{
	
	
	typedef enum
{
    _start = 0,
    _wait_crossing,
	  _postition_start,
	  _stop,

} car_go_until_crossing_t;

  static car_go_until_crossing_t car_go_until_crossing=_start;
	
 	static uint32_t last_tick;
//
switch (car_go_until_crossing)
{
	
	case _start:
		if(data3==1)
		{
			
			set_track_speed(1);
			enable_track_pid();
			car_go_until_crossing++;
			encodeA._motor_distance_mm=0;
			encodeB._motor_distance_mm=0;
			
		
		}

		break;
	
	case _wait_crossing:
		if((data2==2)&&(encodeB._motor_distance_mm>=diatance))
		{
			
			unenable_track_pid();
			
			set_track_speed(0);
			motor1_position_pid.target=encodeA._motor_distance_mm+5;
	    motor2_position_pid.target=encodeB._motor_distance_mm+5;
			enable_position_pid();
					
	    last_tick=HAL_GetTick();
	
			car_go_until_crossing++;
			
		}
		break;
	case _postition_start:
		if(HAL_GetTick()-last_tick>=2000)
		{
		  unenable_position_pid();
			car_go_until_crossing++;
		}
		 
		
	
	 
	  break;

	case _stop:
		car_go_until_crossing=_start;
		return 1;
	
		break;
		


}
return 0;
}





















//小车转向带循迹环的

uint16_t car_position_turn(float turn_angle)
{
	
typedef enum
{
    _start,
	  _postion,
	  _track,
	  _stop,
} car_turn_t;
static car_turn_t car_control=_start;

	static uint32_t last_tick;
	
	switch (car_control)
		
	{
		case _start:
			
				motor1_position_pid.target=encodeA._motor_distance_mm+turn_angle;
	      motor2_position_pid.target=encodeB._motor_distance_mm-turn_angle;
		    enable_position_pid();
		
		    last_tick=HAL_GetTick();
		 
		    car_control++;
		    break;
		case _postion:
		if(HAL_GetTick()-last_tick>=3000)
	   {
		
		   unenable_position_pid();
			 enable_track_pid();
			 
			 last_tick=HAL_GetTick();
		   car_control++;
			 

	    }
		 break;
		case _track:
		 if(HAL_GetTick()-last_tick>=3000)
	   {
		
		   unenable_track_pid();
			 
			
		   car_control++;
			 

	    }
			
		case _stop:
			car_control=_start;
			return 1;
		break;
			
	}
	return 0;
	
}

static uint16_t math;
//控制小车的在定时器的任务
void APP_car_control_()
{
	
	
	  static uint32_t temp_systick;

    enum _state_t
    {
			  _start=0,
        _wait_math,
        _wait_key,
        _select_pin,
        _stop
    };

    static enum _state_t state = _start;
	
	
	
	
	
	
	
	
	switch (state)
	{
		case _start:
			
     led_state(1);
		 enable_k210_flag();
		 k210_data=2;
		 state++;
		 break;
	
		case _wait_math:
			if(k210_data!=0)
			{
			math=k210_data;
			unenable_k210_flag();
			enable_key();
			state++;
			}
			break;
		
	  case _wait_key:
			if(g_key[0].short_flag==1)
			{
			led_state(0);
			unenable_key();
			state++;
			}
		  break;
		case _select_pin:
		{
			switch (math)
			{
				case 2:
				if(car_go_2()==1)
			  {
				state++;
			  }
				break;
				case 4:
				if(car_go_4()==1)
			  {
				state++;
			  }
				break;
				case 6:
				if(car_go_6()==1)
			  {
				state++;
			  }
				break;
					
			
			
			}
		

		
		}
			break;
	
		case _stop:
			led_state(1);
			break;
		
	
	
	
	
	}






}


uint16_t car_go_2()
{
	static uint16_t state=0;
	switch (state)
	{
		case 0:
		if(car_go_until_crossing()==1)
			state++;
		break;
		case 1:
			if(car_position_turn(-5)==1)
				state++;
			break;
	  case 2:
			if(car_go_until_stop_postition()==1)
			state++;
		  break;
		case 3:
			if(car_position_turn(10)==1)
				state++;
			break;
		case 4:
			if(car_go_until_crossing()==1)
			state++;
		  break;
		case 5:
			if(car_position_turn(5)==1)
			state++;
		  break;
		case 6:
			if(car_go_until_stop_postition()==1)
			state++;
		  break;
		case 7:
			return 1;
			break;
		
	
	
	
	
	}



return 0;


}

uint16_t car_go_4()
{
	static uint16_t state=0;
	switch (state)
	{
		case 0:
		if(car_go_until_crossing_2(30)==1)
			state++;
		break;
		case 1:
			if(car_position_turn(-5)==1)
				state++;
			break;
	  case 2:
			if(car_go_until_stop_postition()==1)
			state++;
		  break;
		case 3:
			if(car_position_turn(10)==1)
				state++;
			break;
		case 4:
			if(car_go_until_crossing()==1)
			state++;
		  break;
		case 5:
			if(car_position_turn(5)==1)
			state++;
		  break;
		case 6:
			if(car_go_until_crossing_2_stop(30)==1)
			state++;
		  break;
		case 7:
			return 1;
			break;
		
	
	
	
	
	}



return 0;


}
uint16_t car_go_6()
{
	static uint16_t state=0;
	switch (state)
	{
		case 0:
		if(car_go_until_crossing_2(60)==1)
			state++;
		break;
		case 1:
			if(car_position_turn(-5)==1)
				state++;
			break;
	  case 2:
			if(car_go_until_crossing()==1)
			state++;
		  break;
		case 3:
			if(car_position_turn(-5)==1)
				state++;
			break;
		case 4:
			if(car_go_until_stop_postition()==1)
			state++;
		  break;
		case 5:
			if(car_position_turn(10)==1)
			state++;
		  break;
		case 6:
			if(car_go_until_crossing()==1)
			state++;
		  break;
		case 7:
			if(car_position_turn(5)==1)
			state++;
		  break;
		case 8:
			if(car_go_until_crossing()==1)
			state++;
		  break;
		case 9:
			if(car_position_turn(5)==1)
			state++;
		  break;
		case 10:
			if(car_go_until_crossing_2_stop(70)==1)
			state++;
		  break;
			
	  case 11:	
		
				
				
		
		  return 1;
			break;
		
	
	
	
	
	}



return 0;

}






                             



