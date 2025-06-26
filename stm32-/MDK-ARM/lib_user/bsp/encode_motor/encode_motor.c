#include "encode_motor.h"
#include "tim.h"
#include "print.h"


////1.定义pid参速
//PID_Controller pos_pid, vel_pid;  // 位置环和速度环PID
//PID_Controller pos_pid1, vel_pid1;  // 位置环和速度环PID


// PID参数初始化


//void PID_Init(void) {
//    // 位置环参数
//    pos_pid.Kp = 10.00f;
//    pos_pid.Ki = 0.00;
//    pos_pid.Kd = 1.0f;
//    pos_pid.integral_limit = 100;
//    pos_pid.output_limit = 99;
//     // 位置环参数1
//    pos_pid1.Kp = 1.00f;
//    pos_pid1.Ki = 0.0;
//    pos_pid1.Kd = 1.0f;
//    pos_pid1.integral_limit = 100;
//    pos_pid1.output_limit = 99;
//    // 速度环参数
//    vel_pid.Kp = 1.2f;
//    vel_pid.Ki = 0.076f;
//    vel_pid.Kd = 0.1;
//    vel_pid.integral_limit = 30;
//    vel_pid.output_limit = 99; // 对应PWM最大值[3](@ref)
//	 // 速度环参数
//    vel_pid1.Kp = 1.22f;
//    vel_pid1.Ki = 0.076f;
//    vel_pid1.Kd = 0.1;
//    vel_pid1.integral_limit = 30;
//    vel_pid1.output_limit = 99; // 对应PWM最大值[3](@ref)
//}

void PID_Init_params(PID_Controller *pid,float Kp,float Ki,float Kd,float integral_limit,float output_limit)
{
	 pid->Kp = Kp;
   pid->Ki = Ki;
   pid->Kd = Kd;
   pid->integral_limit = integral_limit;
   pid->output_limit = output_limit;

}


float Velocity_PID_Calculate(PID_Controller *pid,float _target,float _measure) {
	
	
	
	
  
	
	
	 // 设置目标值和测量值
        pid->target = _target;
        pid->measure = _measure;
	  // 增量式PI算法[6](@ref)
	
	
	  // 计算误差
    float error = pid->target - pid->measure;
    float delta = error - pid->last_error;

    pid->output += pid->Kp * delta + pid->Ki * error;

    // 输出限幅
    if(pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if(pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    
    pid->last_error = error;
	
    return pid->output;
  
}
//2.pid计算
//外环位置环
float Position_PID_Calculate(PID_Controller *pid,float _target,float _measure) {
	
	 // 设置目标值和测量值
        pid->target = _target;
        pid->measure = _measure;
    // 计算误差
    pid->error = pid->target - pid->measure;
    // 计算积分
    pid->integral += pid->error;
	  // 积分项限幅[8](@ref)
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 微分项
    float derivative = pid->error - pid->last_error;
    
    // PID计算
    pid->output = pid->Kp * pid->error + 
                 pid->Ki * pid->integral + 
                 pid->Kd * derivative;
    
    // 输出限幅[8](@ref)
    if(pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if(pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    
    pid->last_error = pid->error;
    return pid->output;
}


//速度环



//uint8_t//

//uint8_t motor_control_flag;
//void motor_control(void)
//{
//		//
//       
//			// 3. 位置环计算
//			   //pos_pid.target=200;
//         pos_pid.measure = CMileage;
//		     vel_pid.target=Position_PID_Calculate(&pos_pid);
//			
//			 //  pos_pid1.target=200;
//     //   pos_pid1.measure = DMileage;
//		 //    vel_pid1.target=Position_PID_Calculate(&pos_pid1);
//			 
//			 // 4. 速度环计算
//			   vel_pid.measure=CSpeed;
//				 pwm =  Velocity_PID_Calculate(&vel_pid);
//				 
//				 vel_pid1.target=100;
//				 vel_pid1.measure=DSpeed;
//				 pwm1 =  Velocity_PID_Calculate(&vel_pid1);
//				 
//				 
//				 
//			   //printf("%f,%f\n",CMileage,DMileage);//输出转鿿
//			   //printf("%.2f\n",  pwm);//输出转鿿
//			   //pwm为百分比
//				 //printf("%f,%f,%d\n",DSpeed,pwm1,100);//输出转鿿
//				  printf("%f,%f,%d\n",DSpeed,pwm,200);//输出转鿿
//			   // 5. 电机驱动输出
//				//pwm=0;
//				//pwm1=40;
//			  // MotorA_Output(pwm);
//			  
//				//  MotorB_Output(pwm1);


//}

















