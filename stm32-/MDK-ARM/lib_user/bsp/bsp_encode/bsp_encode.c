#include "bsp_encode.h"
#include <tim.h>
#include <print.h>
//���嶨ʱ��
#define htim_A htim3
#define htim_B htim4



// ����������
#define ENCODER_LINES 11u      // ����������
#define GEAR_RATIO 30u         // ���ٱ�30:1
#define MOTOR_WHEEL_STRAIGHT_MM    65 // ����ֱ������λmm
#define wheel_circumference   MOTOR_WHEEL_STRAIGHT_MM*3.1415926 // �����ܳ�(cm) [2](@ref)
// ��ʱ������
#define QUADRATURE_FACTOR 4   // �ı�Ƶ
#define SAMPLE_TIME_MS 200    // ����50HZ
#define COUNTER_MAX 65535     // 16λ���������ֵ


void bsp_encode_init(void)
{
	//����cumax
	//�����ı�Ƶ
	//������ʱ��
  
		HAL_TIM_Encoder_Start(&htim_A, TIM_CHANNEL_ALL); //�?启编码器模式
    HAL_TIM_Encoder_Start(&htim_B, TIM_CHANNEL_ALL); //�?启编码器模式
}
//void Encode_Calculated_speed_mm(Encode_t * encode)
//{

//	//mm/s
//	encode->_motor_speed_mm = (encode->_tick * wheel_circumference) / 
//                     (ENCODER_LINES * QUADRATURE_FACTOR * GEAR_RATIO * (SAMPLE_TIME_MS / 1000.0f));
//  

//  return ;
//}


void Encode_Calculated_speed_r(Encode_t * encode)
{
	
		encode->_motor_speed_r= (float)(encode->_delta_count) / 
                     (ENCODER_LINES * QUADRATURE_FACTOR * GEAR_RATIO ) * 50;


  return;
}
 
void Encode_Calculated_distance_r(Encode_t * encodeA)
{
		 encodeA->_motor_distance_mm=encodeA->_current_count/
	(ENCODER_LINES * QUADRATURE_FACTOR * GEAR_RATIO ) * 50;


}

extern Encode_t encodeA;
extern Encode_t encodeB;

//void Encode_Task(Encode_t * encodeA,Encode_t * encodeB)
//{
//	static int16_t count,delta,last;
//	
//	// 1. ��ȡ���������������ı�Ƶ��
//     // encodeA->_current_count =-(short)( __HAL_TIM_GetCounter(&htim_A));//��ȡ������
//			encodeB->_current_count = __HAL_TIM_GetCounter(&htim_B);//��ȡ������
//	    count =-( __HAL_TIM_GetCounter(&htim_A));//��ȡ������
//			
//	
//	  // count=encodeA->_current_count;
//	   delta=count-last;
//	   encodeA->_delta_count=delta;
// 	   //encodeA->_delta_count =encodeA->_current_count - (encodeA->_last_count);
//	   encodeB->_delta_count =encodeB->_current_count - encodeB->_last_count;
//	
//	
//	  // ��������������16λģʽ�£�
//      //  if ( encodeA->_delta_count < -30000) encodeA-> _delta_count += 65536;
//     //   else if ( encodeA->_delta_count > 30000)  encodeA->_delta_count -= 65536;
//	
//	      if ( encodeB->_delta_count < -30000) encodeB->_delta_count += 65536;
//        else if ( encodeB->_delta_count > 30000)  encodeB->_delta_count -= 65536;
//	


//		 
//			 // 2. ����ʵ���ٶȣ�cm/s��
//       Encode_Calculated_speed_r(encodeA);
//		   Encode_Calculated_speed_r(encodeB);
//	
//	
//	   last=count;
//	    //encodeA->_last_count= encodeA->_current_count; // ������ʷֵ
//		  encodeB->_last_count= encodeB->_current_count; // ������ʷֵ
//			
//		 encodeA->_motor_distance_mm+=   (float)encodeA->_motor_speed_r*0.2;
//		 encodeB->_motor_distance_mm+=  (float)encodeB->_motor_speed_r*0.2;
//		 
//		 printf("%d,%d\n",count,encodeB->_current_count);
//			// printf("%d,%d\n", count,encodeB->_current_count);
//	
//			 //printf("%f,%f\n",encodeA->_motor_distance_mm,encodeB->_motor_distance_mm);
//	
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   
    if (htim == (&htim12))
    {
			
			
			//	static int16_t count,delta,last;
	
	// 1. ��ȡ���������������ı�Ƶ��
      encodeA._current_count = __HAL_TIM_GetCounter(&htim_A);//��ȡ������
			encodeB._current_count =-( __HAL_TIM_GetCounter(&htim_B));//��ȡ������
	  //  count =-( __HAL_TIM_GetCounter(&htim_A));//��ȡ������
			
		 //printf("%d,%d\n",encodeA._current_count,encodeB._current_count);
	  // count=encodeA->_current_count;
	 //  delta=count-last;
	  // encodeA->_delta_count=delta;
 	   encodeA._delta_count =encodeA._current_count - (encodeA._last_count);
	   encodeB._delta_count =encodeB._current_count - encodeB._last_count;
	
	
	  // ��������������16λģʽ�£�
        if ( encodeA._delta_count < -30000) encodeA. _delta_count += 65536;
        else if ( encodeA._delta_count > 30000)  encodeA._delta_count -= 65536;
	
	      if ( encodeB._delta_count < -30000) encodeB._delta_count += 65536;
        else if ( encodeB._delta_count > 30000)  encodeB._delta_count -= 65536;
	

		encodeA._motor_speed_r= (float)(encodeA._delta_count) / 
                     (ENCODER_LINES * QUADRATURE_FACTOR * GEAR_RATIO ) * 50;
		encodeB._motor_speed_r= (float)(encodeB._delta_count) / 
                     (ENCODER_LINES * QUADRATURE_FACTOR * GEAR_RATIO ) * 50;
		 
			 // 2. ����ʵ���ٶȣ�cm/s��
    //   Encode_Calculated_speed_r(encodeA);
		//   Encode_Calculated_speed_r(encodeB);
	
	
	
	    encodeA._last_count= encodeA._current_count; // ������ʷֵ
		  encodeB._last_count= encodeB._current_count; // ������ʷֵ
			
		 encodeA._motor_distance_mm+=  (float)encodeA._motor_speed_r*0.2;
		 encodeB._motor_distance_mm+=  (float)encodeB._motor_speed_r*0.2;
		 
	//	printf("%d,%d\n",encodeA._current_count,encodeB._current_count);
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
	

			  // Encode_Task(&encodeA,&encodeB);
       
				 //printf("%f,%f\n",encodeA._motor_speed_r,encodeB._motor_speed_r);
			
			
			
	
    }
}
