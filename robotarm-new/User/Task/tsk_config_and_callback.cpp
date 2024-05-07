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

/* Private macros ------------------------------------------------------------*/
// Just for test
#include "Joint.hpp"
#include "robotarm.hpp"
#include "drv_uart.h"
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
extern osSemaphoreId Communication_SemHandle;
/* Function prototypes -------------------------------------------------------*/



/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART3收到的消息
 * @param Length 长度
 */
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    //Robotarm.DR16.UART_RxCpltCallback(Buffer);
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    //Robotarm.Referee.UART_RxCpltCallback(Buffer);
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
		if(huart->Instance == USART3)
		{
			UART_Init(&huart3, DR16_UART3_Callback, 18);
		}
		else if(huart->Instance == USART6)
		{
			UART_Init(&huart6, Referee_UART6_Callback, 128);
		}
		
    }
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
    {
		__HAL_UART_CLEAR_NEFLAG(huart);
		if(huart->Instance == USART3)
		{
			UART_Init(&huart3, DR16_UART3_Callback, 18);
		}
		else if(huart->Instance == USART6)
		{
			UART_Init(&huart6, Referee_UART6_Callback, 128);
		}
    }
    
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
    {
		__HAL_UART_CLEAR_OREFLAG(huart);
    }
 
}

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
		case (0x01):
		{
//			Robotarm.Motor_Joint1.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		case (0x02):
		{
//			Robotarm.Motor_Joint2.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		case (0x05):
		{
			DM_motor_feedback_handler(&dmMotor_joint4, CAN_RxMessage->Data[0] & 0x0F, CAN_RxMessage->Data);
		}
		break;
		case (0x14):
		{
			AK_motor_feedback_handler(&akMotor_joint2, CAN_RxMessage->Header.ExtId, CAN_RxMessage->Data);
		}
		break;
		case (0x12):
		{
			AK_motor_feedback_handler(&akMotor_joint3, CAN_RxMessage->Header.ExtId, CAN_RxMessage->Data);
		}
		break;
		case (0x202):
		{
			m3508_joint5.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		case (0x203):
		{
			m2006_joint6.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		// 平移关节电机
		case (0x208):
		{
			m3508_joint1.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		break;
		case (0x206):
		{
	//        chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
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
    case (0x00):
    {
		switch(CAN_RxMessage->Data[0])
		{
			case 0x01:
			{
			}
			break;
			case 0x02:
			{
			}
			break;
		}
    }
    break;
//	case (0x02):
//    {
//        Robotarm.Motor_Joint2.CAN_RxCpltCallback(CAN_RxMessage->Data);
//    }
    break;
    case (0x05):
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

//uint8_t data_[8] = {0 ,0,0,100,0,0,0,0};
/**
 * @brief 机械臂解算线程，逆解耗时大概7~15ms
 *
 */
posMatrix_t posMat = {745.64, 0, 127.45};
RpyMatrix_t rpyMat = {0, 0, -PI/2.f};
void RobotarmResolution_Task(void const * argument)
{
	while(1)
	{
		posNRpyMatrix_t endPosNRpyNow;
		robotarm.Robotarm_GetEndPosNRpyNow(endPosNRpyNow);		// 获得当前末端位姿 （rpy型）
		robotarm.Robotarm_SetEndPosNRpyTarget(posMat, rpyMat);	// 设置末端位姿 （rpy型）
		robotarm.Robotarm_QNowUpdate();							// 更新当前Q矩阵
		robotarm.Robotarm_IKine();								// 逆运动学
		//Robotarm.Robotarm_Resolution.Reload_Task_Status_PeriodElapsedCallback();
		extern osThreadId RobotarmTaskHandle;
		UBaseType_t remaining = uxTaskGetStackHighWaterMark((TaskHandle_t)RobotarmTaskHandle);
		osDelay(35);
	}
}



/**
 * @brief 电机PID计算和CAN发送线程
 *
 */
void Motor_Task(void const * argument)
{
	static uint16_t mod50 = 0;
	while(1)
	{
		// 如果没校准就会校准，否则就会设置位置
		robotarm.Robotarm_CheckforCalibration();
		robotarm.Robotarm_DoJointControl();
		mod50++;
		if (mod50 == 100)
		{
			mod50 = 0;
			//Robotarm.Task_Alive_PeriodElapsedCallback();
		}
		
//		Robotarm.Task_Calculate_PeriodElapsedCallback();
//		if((Robotarm.Get_Joint_Offset_Angle(1) != Robotarm.Get_Joint_Limit_Angle(1)) && (Robotarm.Get_Joint_Offset_Angle(2) != Robotarm.Get_Joint_Limit_Angle(2))
//			&&((Robotarm.Robotarm_Resolution.Status[Robotarm_Task_Status_Calibration].Time !=0)
//			||(Robotarm.Robotarm_Resolution.Get_Now_Status_Serial() == Robotarm_Task_Status_Resolution)
//			||(Robotarm.Robotarm_Resolution.Get_Now_Status_Serial() == Robotarm_Task_Status_decision)))
//		{
//			Task_CAN_PeriodElapsedCallback();
//		}
		extern osThreadId MotorTaskHandle;
		UBaseType_t remaining = uxTaskGetStackHighWaterMark(MotorTaskHandle);
		osDelay(5);
	}
}

/**
 * @brief 遥控器检测线程
 *
 */
void Cammand_Task(void const * argument)
{
	while(1)
	{
		static uint16_t mod50 = 0;
		mod50++;
		if (mod50 == 500)
		{
			mod50 = 0;
			//Robotarm.DR16.Task_Alive_PeriodElapsedCallback();
			//Robotarm.Referee.Task_Alive_PeriodElapsedCallback();
		}
		//设置目标位姿
		//Robotarm.Task_Control_Robotarm();
		osDelay(2);
	}
}

/**
 * @brief 底盘通信线程
 *
 */
void Communication_Task(void const * argument)
{
	
	while(1)
	{
		//Robotarm.Task_Chassis_Communication_PeriodElapsedCallback();
		osDelay(2);
	}
	
}

/**
 * @brief 机械臂初始化
 *
 */
void init()
{
	CAN_Init(&hcan1, Device_CAN1_Callback);
	CAN_Init(&hcan2, Device_CAN2_Callback);
	UART_Init(&huart3, DR16_UART3_Callback, 18);
	UART_Init(&huart6, Referee_UART6_Callback, 128);
//	Robotarm.Init();
	DjiMotor_Init();
	DM_motor_Init();
	AK_motor_Init();
	
	
	// 初始化机械臂
	robotarm.Robotarm_Init();
	
//	Icac_HandleInit();
	
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
uint8_t caliFlag = 1;
void test()
{
	
	
		osDelay(2);
////		revJoint3.setBodyFrameJointAngle(0.f);
	
	// 如果有电机掉线，执行校准
//	if (Icac_GetConnectionStatus(&icac_akMotor_joint1) || Icac_GetConnectionStatus(&icac_akMotor_joint2))
//		caliFlag = 1;
	
}