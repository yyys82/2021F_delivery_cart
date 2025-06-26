#include "task.h"
#include "APP_PID.h"
#include "app_car_control.h"
volatile	int		time_S;

#define QMC5883L_ADDR (0x0D << 1) // 转换为7位地址
#define QMC5883L_ID_REG 0x0D       // ID寄存器地址

// 全局变量
uint8_t rx_buffer[7];  // 接收缓冲区
uint8_t rx_index = 0;  // 接收位置索引
uint8_t packet_ready = 0; // 数据包就绪标志
//
uint8_t rx_buffer_k210[4];  // 接收缓冲区
uint8_t rx_index_k210 = 0;  // 接收位置索引
uint8_t packet_ready_k210 = 0; // 数据包就绪标志
//
uint16_t data1=400,data2,data3; // 数据包就绪标志
uint16_t k210_data; // 数据包就绪标志


uint8_t read_qmc5883l_id(I2C_HandleTypeDef *hi2c) {
    uint8_t id = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR, QMC5883L_ID_REG,
                                                I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
    if (status == HAL_OK) return id; // 成功时返回0xFF
    else return 0x09;               // 失败返回0
}

void app_pid_run(void);
void duoji_init();


void process_data(uint8_t *data,uint16_t *x,uint16_t *y,uint16_t *z) ;
void process_data_k210(uint8_t *data,uint16_t *x);
#define SAMPLE_PERIOD (0.05f) // replace this with actual sample period
FusionAhrs ahrs;
FusionEuler euler;

uint16_t flag__;
void Init()
{


	//HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_index], 1);
     HAL_UART_Receive_IT(&huart5, &rx_buffer[0], 1);
	
	   HAL_UART_Receive_IT(&huart3, &rx_buffer_k210[0], 1); // 重启接收
//	while(MPU6050_Init() == -1){};

	

	
	
	
	
	OLED_Init();                           //OLED初始
  OLED_Clear();   
//  OLED_ShowString(0,2,"oled",12,0);//正相显示6X8字符串
//	PID_Init();


	//OLED_Clear();   
  //OLED_ShowString(0,2,"time",12,0);//正相显示6X8字符串
//	HAL_Delay(100);
	

		
  FusionAhrsInitialise(&ahrs);		
	bsp_motor_init();
	bsp_encode_init();
	duoji_init();

	

		
	soft_timer_repeat_init(SOFT_TIMER_0, 50);
	soft_timer_repeat_init(SOFT_TIMER_1, 20);
	soft_timer_repeat_init(SOFT_TIMER_2, 200);
	soft_timer_repeat_init(SOFT_TIMER_3, 1);
	
	HAL_TIM_Base_Start_IT(&htim12);
	
  HAL_Delay(100);		//清屏	
	OLED_Clear();  
	OLED_ShowString(0,1,"init_ok",12,0);//正相显示6X8字符串
}


uint8_t id;

void app_oled(void)
{
	for (uint8_t i=0;i<4; i++)
	{
	if(g_key[i].short_flag==1)
	{
		g_key[i].short_flag=0;
		OLED_ShowString(0,0,"key_press",12,0);//正相显示6X8字符串

	}
	else
		OLED_ShowString(0,0,"key_waiting",12,0);//正相显示6X8字符串
	}
	
		
		
		
//	}
	
	 // id =read_qmc5883l_id(&hi2c2);

	//printf("oo");
	
	//OLED_ShowNum(0,0,data1,4,12,0);
	//OLED_ShowNum(36,0,data2,4,12,0);
	
	//OLED_ShowNum(0,4,data3,4,12,0);



}

void APP_beep();
void APP_Servo_control(void);



void Loop()
{
	
	Init();
		///	car_spin_position(4.5);
	while(1){
	
		if (soft_timer_is_timeout(SOFT_TIMER_0))
    {
      //app_6050();
			APP_Scan_Key();
  
    }
		
		if (soft_timer_is_timeout(SOFT_TIMER_1))
		{
		  app_pid_run();
		}
		
		if (soft_timer_is_timeout(SOFT_TIMER_2))
		{
			app_oled();
			//APP_Servo_control();
		//				 __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 500);
		//  		 __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 1500);
					// __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 2500);
		}
		
		if (soft_timer_is_timeout(SOFT_TIMER_3))
		{	
	//	motor2_pwm_value_set(270);
			
			APP_car_control_();
	
		}
		
		if(packet_ready == 1)
		{
      process_data(rx_buffer,&data1,&data2,&data3);
	    packet_ready = 0;
		}
		if(packet_ready_k210 ==0x01)
		{
      process_data_k210(rx_buffer_k210,&k210_data);
			//HAL_UART_Transmit(&huart3,"q",1,1000);
	    packet_ready_k210= 0;
		}
		   // APP_beep();

}

 
}











////////////////////////////////////////////







void app_6050()
{

    time_S=HAL_GetTick();
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		//MPU6050_Read_Temp();

	  const FusionVector gyroscope = {Mpu6050_Data.Gyro_X, Mpu6050_Data.Gyro_Y, Mpu6050_Data.Gyro_Z}; // replace this with actual gyroscope data in degrees/s
    const FusionVector accelerometer = {Mpu6050_Data.Accel_X,Mpu6050_Data.Accel_Y, Mpu6050_Data.Accel_Z}; // replace this with actual accelerometer data in g

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		
		//滤波
		euler.angle.yaw=(euler.angle.yaw+0.7f);
    euler.angle.yaw-=(time_S*0.00015);

   // printf("%1f,%f,%f\n", euler.angle.roll, euler.angle.pitch,euler.angle.yaw);
	  //ahrs.quaternion.element.w;

	 /**
				再后面的四元数 q0, q1, q2, q3 是表示物体旋转的四元数。四元数是一种扩展的复数，用于描述和计算三维空间中的旋转。四元数具有四个分量（q0, q1, q2, q3），其中q0是标量部分，q1、q2和q3是向量部分。四元数相对于欧拉角具有以下优点：

避免万向节锁：欧拉角在某些特定姿态下会出现万向节锁现象，导致某个旋转自由度丢失（在某些特定姿态下，两个旋转轴可能会重合，导致一个旋转自由度丢失）。四元数则没有这个问题，让得旋转表示更加稳定和连续。
更高的数值稳定性：四元数在计算旋转时具有更好的数值稳定性，避免了欧拉角在接近临界值时的数值不稳定现象。
简化旋转计算：四元数可以通过简单的数学运算实现旋转的组合、插值和求逆等操作，计算效率高。
		
		****/



}









////////////////////////

static uint8_t k210_flag=1;

void enable_k210_flag(){k210_flag=1;}
void unenable_k210_flag(){k210_flag=0;}
// 全局变量


// 中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t state = 0;
    uint8_t ch = rx_buffer[rx_index];
    
    if (huart->Instance == UART5) {
        switch(state) {
            case 0:  // 等待第一个帧头
                if (ch == 0x2C) {
									
                    state = 1;
                    rx_index = 0;
                    rx_buffer[rx_index++] = ch;
                }
                break;
            case 1:  // 验证第二个帧头
                if (ch == 0x12) {
											
                    state = 2;
                    rx_buffer[rx_index++] = ch;
                } else {
                    state = 0;
                }
                break;
            case 2:  // 接收数据部分
                rx_buffer[rx_index++] = ch;
													
                if (rx_index >= 6) {  // 已接收6字节（0x3C+0x12+4B数据）
                    state = 3;
                }
                break;
            case 3:  // 验证帧尾
                if (ch == 0x5B) {
                    rx_buffer[rx_index] = ch;

                    packet_ready = 1;  // 完整数据包就绪
                }
                state = 0;
                rx_index = 0;
                break;
        }
		
        HAL_UART_Receive_IT(&huart5, &rx_buffer[rx_index], 1); // 重启接收
    }
		
		
		
		static uint8_t state_k210 = 0;
		uint8_t ch_k210 = rx_buffer_k210[rx_index_k210];
    
    if (huart->Instance == USART3) {
			if(k210_flag==1){
        switch(state_k210) {
            case 0:  // 等待第一个帧头
                if (ch_k210 == 0x2C) {
									
                    state_k210 = 1;
                    rx_index_k210 = 0;
                    rx_buffer_k210[rx_index_k210++] = ch_k210;
								
                }
                break;
            case 1:  // 验证第二个帧头
                if (ch_k210 == 0x12) {
											
                    state_k210 = 2;
                    rx_buffer_k210[rx_index_k210++] = ch_k210;
                } else {
                    state_k210 = 0;
                }
                break;
            case 2:  // 接收数据部分
                rx_buffer_k210[rx_index_k210++] = ch_k210;
													
                if (rx_index_k210 >= 3) {  // 已接收6字节（0x3C+0x12+4B数据）
                    state_k210 = 3;
                }
                break;
            case 3:  // 验证帧尾
                if (ch_k210 == 0x5B) {
                    rx_buffer_k210[rx_index_k210] = ch_k210;

                    packet_ready_k210 = 1;  // 完整数据包就绪
								//	 HAL_UART_Transmit(&huart3,&rx_buffer_k210[2],1,1000);
                }
                state_k210 = 0;
                rx_index_k210 = 0;
                break;
        }
		    
        HAL_UART_Receive_IT(&huart3, &rx_buffer_k210[rx_index_k210], 1); // 重启接收
    }
	}
		
		
		
		
		
		
		
		
		
}
void process_data(uint8_t *data,uint16_t *x,uint16_t *y,uint16_t *z) {
    // 示例：将数据通过串口回传
 
	*x= (data[2]<<8)+data[3];
	*y=  data[4];
	*z= data[5];
//	printf("%d,%d",*x,*y);
  
}

void process_data_k210(uint8_t *data,uint16_t *x) {
	if(k210_flag==1){
    // 示例：将数据通过串口回传
 
	*x=data[2];
	//HAL_UART_Transmit(&huart3,x,1,1000);
  

//	printf("%d,%d",*x,*y);
  }
}
//////////////////////





void Delay_us(uint32_t us) {
    uint32_t tick = HAL_RCC_GetHCLKFreq() / 1000000 * us; // 根据主频计算计数值
    while(tick--) {
        __NOP(); // 空操作指令
    }
}

void Beep(uint16_t freq, uint16_t duration_ms) {
    uint32_t period_us = 1000000 / freq;    // 计算周期（微秒）
    uint32_t cycles = duration_ms * 1000 / period_us; // 总周期数
    
    for(uint32_t i=0; i<cycles; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        Delay_us(period_us / 2);  // 高电平半周期
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        Delay_us(period_us / 2);  // 低电平半周期
    }
}


void APP_beep()
{
//	
//	uint32_t period_us =100000;
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//Delay_us(period_us * 0.7); 
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//Delay_us(period_us * 0.3);
}






///////////////

void duoji_init()
{
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);


}
	


	// 角度转脉宽计算（0°~180° → 500~2500μs）
void Set_Servo_Angle(uint8_t angle) {
    float pulse_us = 500 + (angle / 180.0) * 2000;  // 脉宽范围：500~2500μs
    uint16_t pulse_ticks = (uint16_t)(pulse_us);     // 1MHz时钟下，1μs=1计数
   
	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse_ticks);
}

void APP_Servo_control(void)
{
	static uint16_t state;
	switch (state)
	{
		case 1:
	 // duoji_init();
		state++;

		 break;
		case 2:
			//Set_Servo_Angle(150);
	
				 __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 2000);
	
		break;
	
	
	
	
	
	
	}
	
	
	
	
	


}





