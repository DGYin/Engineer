/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_opticalflow.h"

/* Private macros ------------------------------------------------------------*/
extern osSemaphoreId AngleSlover_SemHandle;
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief 裁判系统初始化
 *
 * @param __huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Class_Opticalflow::Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
}


/**
 * @brief 数据处理过程, 为节约性能不作校验但提供了接口
 * 如遇到大规模丢包或错乱现象, 可重新启用校验过程
 *
 */
void Class_Opticalflow::Data_Process()
{
    //数据处理过程
    Struct_OpticalFlow_UART_Data *tmp_buffer = (Struct_OpticalFlow_UART_Data *)UART_Manage_Object->Rx_Buffer;

    //未通过校验
    if ((tmp_buffer->Frame_Header != Frame_Header) || (tmp_buffer->Target_Add != Target_Add))
    {
        return;
    }

    switch (tmp_buffer->OpticalFlow_Command_ID)
    {
		case (IMU_DATA):
		{
			memcpy(&IMU_Raw_Data, tmp_buffer->Data, sizeof(Struct_IMU_Raw_Data));
		}
		break;
		case (QUATERNION_DATA):
		{
			memcpy(&Quaternion_Data, tmp_buffer->Data, sizeof(Struct_Quaternion_Data));
			osSemaphoreRelease(AngleSlover_SemHandle);
		}
		break;
		case (HEIGHT_DATA):
		{
			memcpy(&Height_Data, tmp_buffer->Data, sizeof(Struct_Height_Data));
		}
		break;
		case (OPTICALFLOW_DATA):
		{
			switch(tmp_buffer->Data[0])
			{
				case (MODE_0):
				{
					memcpy(&OpticalFlow_Raw_Data, tmp_buffer->Data, sizeof(Struct_OpticalFlow_Raw_Data));
				}
				break;
				case (MODE_1):
				{
					memcpy(&OpticalFlow_Integrate_Data_1, tmp_buffer->Data, sizeof(Struct_OpticalFlow_Integrate_Data_1));
				}
				break;
				case (MODE_2):
				{
					memcpy(&OpticalFlow_Integrate_Data_2, tmp_buffer->Data, sizeof(Struct_OpticalFlow_Integrate_Data_2));
					
				}
				break;
			}
		}
		break;
	}
}


/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Opticalflow::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断裁判系统是否在线
    Flag += 1;
    Data_Process();
}


/**
 * @brief TIM定时器中断定期检测裁判系统是否存活
 *
 */
void Class_Opticalflow::Task_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过裁判系统数据
    if (Flag == Pre_Flag)
    {
        //裁判系统断开连接
        OpticalFlow_Status = OpticalFlow_Status_DISABLE;
    }
    else
    {
        //裁判系统保持连接
        OpticalFlow_Status = OpticalFlow_Status_ENABLE;
    }
    Pre_Flag = Flag;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
