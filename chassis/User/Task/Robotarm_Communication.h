/**
 * @file dvc_AKmotor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief AK电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 初稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef ROBOTARM_COMMUNICATION_H
#define ROBOTARM_COMMUNICATION_H

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"
#include "drv_math.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief AK电机状态
 *
 */
enum Enum_Roboatrm_Communication_Status
{
    Roboatrm_Communication_Status_DISABLE = 0,
    Roboatrm_Communication_Status_ENABLE,
};

/**
 * @brief AK电机的ID枚举类型
 *
 */
enum Enum_Roboatrm_Communication_ID : uint8_t
{
    Roboatrm_Communication_ID_0x11 = 0x11,
    Roboatrm_Communication_ID_0x12,
    Roboatrm_Communication_ID_0x13,
    Roboatrm_Communication_ID_0x14,
    Roboatrm_Communication_ID_0x15,
    Roboatrm_Communication_ID_0x16,
//    Roboatrm_Task_ID_0x17,
//	Roboatrm_Task_ID_0x18,
};

struct Struct_Data_ID_0x11
{
	uint16_t Chassis_Vx;
	uint16_t Chassis_Vy;
	uint16_t Chassis_Wz;
	uint16_t Robotarm_Pz;
} __attribute__((packed));

/**
 * @brief J4310无刷电机, 单片机控制输出控制帧
 * Roboatrm_Task_Control_Method_POSITION_OMEGA模式下, 需调参助手辅助设置位置环PI参数, 空载250与0
 * 
 * PMAX值需在调参助手设置为3.141593, 即PI, 此时可在MIT模式下当舵机使用
 *
 */
class Class_Roboatrm_Communication
{
public:

	void Init(CAN_HandleTypeDef *hcan);


    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void Task_Alive_PeriodElapsedCallback();
    void Task_Process_PeriodElapsedCallback();
	
	inline Enum_Roboatrm_Communication_Status Get_Roboatrm_Communication_Status();
	inline Struct_Data_ID_0x11 Get_Data_ID_0x11();
	
	//电机对外接口信息
	bool Communication_Data(float Data,float Min_Data,float Max_Data,Enum_Roboatrm_Communication_ID Roboatrm_Communication_ID);
    bool Communication_Data(uint8_t Mode,Enum_Roboatrm_Communication_ID Roboatrm_Communication_ID);
protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_Roboatrm_Communication_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
	//ID_0x11发送内容
	Struct_Data_ID_0x11 Data_ID_0x11;
    //位置反馈偏移

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
	
	uint8_t Data_Pointer[6];
	//ID偏移量
	uint8_t ID_Bias = 0x11;

    //读变量

    //电机状态
    Enum_Roboatrm_Communication_Status Roboatrm_Communication_Status = Roboatrm_Communication_Status_DISABLE;
    

    //写变量

    //读写变量
    //内部函数

    void Data_Process();
	
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Roboatrm_Task_Status 电机状态
 */
Enum_Roboatrm_Communication_Status Class_Roboatrm_Communication::Get_Roboatrm_Communication_Status()
{
    return (Roboatrm_Communication_Status);
}
Struct_Data_ID_0x11 Class_Roboatrm_Communication::Get_Data_ID_0x11()
{
	return (Data_ID_0x11);
}
	
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
