/**
 * @file dvc_AKmotor.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief AK电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 初稿机
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "Robotarm_Communication.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_Roboatrm_Communication_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
			case (Roboatrm_Communication_ID_0x11):
			{
				tmp_tx_data_ptr = CAN1_0xx11_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x12):
			{
				tmp_tx_data_ptr = CAN1_0xx12_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x13):
			{
				tmp_tx_data_ptr = CAN1_0xx13_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x14):
			{
				tmp_tx_data_ptr = CAN1_0xx14_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x15):
			{
				tmp_tx_data_ptr = CAN1_0xx15_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x16):
			{
				tmp_tx_data_ptr = CAN1_0xx16_Tx_Data;
			}
			break;
		}
    }
    else if (hcan == &hcan2)
    {
        switch (__CAN_ID)
        {
			case (Roboatrm_Communication_ID_0x11):
			{
				tmp_tx_data_ptr = CAN2_0xx11_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x12):
			{
				tmp_tx_data_ptr = CAN2_0xx12_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x13):
			{
				tmp_tx_data_ptr = CAN2_0xx13_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x14):
			{
				tmp_tx_data_ptr = CAN2_0xx14_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x15):
			{
				tmp_tx_data_ptr = CAN2_0xx15_Tx_Data;
			}
			break;
			case (Roboatrm_Communication_ID_0x16):
			{
				tmp_tx_data_ptr = CAN2_0xx16_Tx_Data;
			}
			break;
		}
	}
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 绑定的CAN ID
 * @param __Control_Method 电机控制方式, 默认角度
 * @param __Position_Offset 编码器偏移, 默认0
 * @param __Omega_Max 最大速度, 调参助手设置
 * @param __Torque_Max 最大扭矩, 调参助手设置
 */
void Class_Roboatrm_Communication::Init(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
	
	Data_ID_0x11.Chassis_Vx = ((1<<16)-1)/2;
	Data_ID_0x11.Chassis_Vy = ((1<<16)-1)/2;
	Data_ID_0x11.Chassis_Wz = ((1<<16)-1)/2;
}


bool Class_Roboatrm_Communication::Communication_Data(float Data,float Min_Data,float Max_Data,Enum_Roboatrm_Communication_ID Roboatrm_Communication_ID)
{
	CAN_ID = Roboatrm_Communication_ID;
	CAN_Tx_Data = allocate_tx_data(CAN_Manage_Object->CAN_Handler, Roboatrm_Communication_ID);
	uint8_t ID_Pointer = Roboatrm_Communication_ID-ID_Bias;
	uint16_t tmp_Data = Math_Float_To_Int(Data, Min_Data, Max_Data, 0, ((1 << 16) - 1));
	if((Data_Pointer[ID_Pointer] + 2) <= 8)
	{
		memcpy(&CAN_Tx_Data[(Data_Pointer[ID_Pointer])], &tmp_Data, sizeof(uint16_t));
		Data_Pointer[ID_Pointer] += 2;
		return true;
	}
	else
	{
		Data_Pointer[ID_Pointer] = 0;
		return false;
	}
}

bool Class_Roboatrm_Communication::Communication_Data(uint8_t Mode,Enum_Roboatrm_Communication_ID Roboatrm_Communication_ID)
{
	CAN_ID = Roboatrm_Communication_ID;
	CAN_Tx_Data = allocate_tx_data(CAN_Manage_Object->CAN_Handler, Roboatrm_Communication_ID);
	uint8_t ID_Pointer = Roboatrm_Communication_ID-ID_Bias;
	if((Data_Pointer[ID_Pointer] + 1) <= 8)
	{
		memcpy(&CAN_Tx_Data[(Data_Pointer[ID_Pointer])], &Mode, sizeof(uint8_t));
		Data_Pointer[ID_Pointer] += 1;
		return true;
	}
	else
	{
		Data_Pointer[ID_Pointer] = 0;
		return false;
	}
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Roboatrm_Communication::Data_Process()
{
	switch (CAN_Manage_Object->Rx_Buffer.Header.StdId)
	{
		case (Roboatrm_Communication_ID_0x11):
		{
			memcpy(&Data_ID_0x11, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Struct_Data_ID_0x11));
		}
		break;
		case (Roboatrm_Communication_ID_0x12):
		{
			
		}
		break;
		case (Roboatrm_Communication_ID_0x13):
		{
			
		}
		break;
		case (Roboatrm_Communication_ID_0x14):
		{
			
		}
		break;
		case (Roboatrm_Communication_ID_0x15):
		{
			
		}
		break;
		case (Roboatrm_Communication_ID_0x16):
		{
			
		}
		break;
	}
    
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Roboatrm_Communication::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Roboatrm_Communication::Task_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
//    if ((Flag == Pre_Flag)||(Data.error_statue != 0))
//    {
//        //电机断开连接
//        AK_Motor_Status = AK_Motor_Status_DISABLE;
//    }
//    else
//    {
//        //电机保持连接
//        AK_Motor_Status = AK_Motor_Status_ENABLE;
//    }

    //控制电机使能或失能
	
    Pre_Flag = Flag;
}


/**
 * @brief TIM定时器中断发送出去的回调函数
 *
 */
void Class_Roboatrm_Communication::Task_Process_PeriodElapsedCallback()
{
	CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID , CAN_Tx_Data, Data_Pointer[CAN_ID - ID_Bias], CAN_ID_STD);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
