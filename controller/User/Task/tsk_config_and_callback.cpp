/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "controller_task.h"
/* Private macros ------------------------------------------------------------*/
Class_Controller controller;

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void OpticalFlow_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Opticalflow.UART_RxCpltCallback(Buffer);
}
/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART2_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Referee.UART_RxCpltCallback(Buffer);
}

/**************************************************************************
  * @brief  ????????
  * @retval 
**************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	__HAL_UNLOCK(huart);
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))!=RESET)
    {
		__HAL_UART_CLEAR_PEFLAG(huart);
    }
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))!=RESET)
    {
		__HAL_UART_CLEAR_FEFLAG(huart);
		if(huart->Instance == USART1)
		{
			UART_Init(&huart1,OpticalFlow_UART1_Callback,70);
		}
		else if(huart->Instance == USART2)
		{
			UART_Init(&huart2,Referee_UART2_Callback,128);
		}
		
    }
    
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
    {
		__HAL_UART_CLEAR_NEFLAG(huart);
		if(huart->Instance == USART1)
		{
			UART_Init(&huart1,OpticalFlow_UART1_Callback,70);
		}
		else if(huart->Instance == USART2)
		{
			UART_Init(&huart2,Referee_UART2_Callback,128);
		}
    }       
    
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
    {
		__HAL_UART_CLEAR_OREFLAG(huart);
    }    
 
}
//uint8_t data_[8] = {0 ,0,0,100,0,0,0,0};
/**
 * @brief 机械臂解算线程
 *
 */
void Slover_Task(void const * argument)
{
	static uint16_t mod50 = 0;
	while(1)
	{
		mod50++;
		if (mod50 == 10)
		{
			mod50 = 0;
			controller.Opticalflow.Task_Alive_PeriodElapsedCallback();
		}
		controller.ToEulerAngles();
		osDelay(11);
	}
}


void Communication_Task(void const * argument)
{
	while(1)
	{
		
		controller.Communication_To_Robot();
		osDelay(33);
	}
	
}

void init()
{
	UART_Init(&huart1,OpticalFlow_UART1_Callback,70);
	UART_Init(&huart2,Referee_UART2_Callback,128);
	controller.Init();
	
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
