#include "string.h"
#include "usart.h"
#include "print.h"
#include "encode_motor.h"


//char rx_buffer[RX_BUFFER_SIZE];  // 接收缓冲区
//volatile uint16_t rx_index = 0;  // 缓冲区索引
//volatile uint8_t rx_complete = 0;  // 接收完成标志

int fputc(int ch, FILE *f)
 
{
 
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
 
}


//// 自定义字符串转浮点数函数（避免依赖库函数）
//float string_to_float(const char *str) {
//    float value = 0.0;
//    float fraction = 0.1;
//    int8_t sign = 1;
//    uint8_t decimal_found = 0;

//    if (*str == '-') {
//        sign = -1;
//        str++;
//    }

//    while (*str != '\0') {
//        if (*str == '.') {
//            decimal_found = 1;
//        } else if (*str >= '0' && *str <= '9') {
//            if (!decimal_found) {
//                value = value * 10.0 + (*str - '0');
//            } else {
//                value += (*str - '0') * fraction;
//                fraction *= 0.1;
//            }
//        }
//        str++;
//    }
//    return sign * value;
//}
//// 解析PID参数
//uint8_t parse_pid(const char *data,PID_Controller *pid) {
//    const char *p_ptr = strstr(data, "P=");
//    const char *i_ptr = strstr(data, "I=");
//    const char *d_ptr = strstr(data, "D=");

//    if (!p_ptr || !i_ptr || !d_ptr) {
//        return 0; // 格式错误
//    }

//    // 提取P值
//    pid->Kp = string_to_float(p_ptr + 2);
//    
//    // 提取I值
//    pid->Ki = string_to_float(i_ptr + 2);
//    
//    // 提取D值
//    pid->Kd = string_to_float(d_ptr + 2);

//    return 1; // 解析成功
//}
//void usrt_recived(void)
//{
//	
//	  if (rx_buffer[rx_index] == '#' ) {
//      rx_buffer[rx_index] = '#';  // 添加字符串结束符
//			
//			
//		
//      rx_complete = 1;            // 置位标志
//			
//			    
//			
//			
//      rx_index = 0;               // 重置索引
//    } else {
//      if (rx_index < RX_BUFFER_SIZE - 1) rx_index++;
//      else rx_index = 0;  // 溢出处理
//    }
//    
//    // 重新启用中断接收
//    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_index], 1);


//}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	 // HAL_UART_IRQHandler(&huart1);

//  if (huart->Instance == USART1) // ???????????? USART1
//  {
//     
//    // 检测结束符（如回车或换行）
//    if (rx_buffer[rx_index] == '#' ) {
//      rx_buffer[rx_index] = '#';  // 添加字符串结束符
//			
//			
//		
//      rx_complete = 1;            // 置位标志
//			
//			    
//			
//			
//      rx_index = 0;               // 重置索引
//    } else {
//      if (rx_index < RX_BUFFER_SIZE - 1) rx_index++;
//      else rx_index = 0;  // 溢出处理
//    }
//  
//    // 重新启用中断接收
//    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_index], 1);
//	  
//  }
//}

//void uart_pid_init(PID_Controller *pid)
//{
//		  if (rx_complete) {
//      // 示例：回显接收的字符串
//      HAL_UART_Transmit(&huart1, (uint8_t*)"Received: ", 10, 100);
//      HAL_UART_Transmit(&huart1, (uint8_t*)rx_buffer, strlen(rx_buffer), 100);
//      HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
//				 
//      parse_pid(rx_buffer,pid);
//      printf( "P=%.2f,I=%.2f,D=%.2f\r\n",
//           pid->Kp, pid->Ki, pid->Kd);
//      rx_complete = 0;  // 清除标志
//    }




//}