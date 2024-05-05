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

#include "chassis_task.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern Class_Chassis Chassis;
bool init_finished = 0;

/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	uint32_t temp_id;
	if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
	}
	else if(CAN_RxMessage->Header.IDE == CAN_ID_EXT)
	{
		temp_id = CAN_RxMessage->Header.ExtId &0xff;
	}
	
    switch (temp_id)
    {
    case (0x201):
    {
		Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
	case (0x202):
    {
		Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
		Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
		Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x205):
    {
		Chassis.Motor_Rise.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    break;
    case (0x206):
    {
//        chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x207):
    {
//        chariot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
	uint32_t temp_id;
	if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
	}
	else if(CAN_RxMessage->Header.IDE == CAN_ID_EXT)
	{
		temp_id = CAN_RxMessage->Header.ExtId &0xff;
	}
	
    switch (temp_id)
    {
	case (Roboatrm_Communication_ID_0x11):
    {
        Chassis.Roboatrm_Communication.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x201):
    {

    }
    break;
    case (0x202):
    {

    }
    break;
    case (0x203):
    {

    }
    break;
    }
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Chassis.Referee.UART_RxCpltCallback(Buffer);
}

/**
 * @brief 底盘任务线程
 *
 */
 uint8_t data[8] = {1,2,3,4,5,6,7,8};
void Chassis_Task(void const * argument)
{
	while(1)
	{
		static int mod50 = 0;
		mod50++;
		if (mod50 == 100)
		{
			mod50 = 0;
			Chassis.Task_Alive_PeriodElapsedCallback();
		}
		Chassis.Task_Calculate_PeriodElapsedCallback();
		Task_CAN_PeriodElapsedCallback();
		osDelay(2);
	}
}

/**
 * @brief 底盘任务线程
 *
 */
void Referee_Task(void const * argument)
{
	while(1)
	{
		static int mod50 = 0;
		mod50++;
		if (mod50 == 100)
		{
			mod50 = 0;
			Chassis.Referee.Task_Alive_PeriodElapsedCallback();
		}
		Chassis.UI_Task();
		osDelay(300);
	}
}

void init()
{
	CAN_Init(&hcan1, Device_CAN1_Callback);
	CAN_Init(&hcan2, Device_CAN2_Callback);
	UART_Init(&huart1, Referee_UART1_Callback, 128);
	Chassis.Init(0,0,0);
	
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
