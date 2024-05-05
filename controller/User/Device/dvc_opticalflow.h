/**
 * @file crt_gimbal.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef OPTICALFLOW_TASK_H
#define OPTICALFLOW_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "drv_uart.h"
#include "drv_math.h"
#include <string>
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统状态
 *
 */
enum Enum_OpticalFlow_Status
{
    OpticalFlow_Status_DISABLE = 0,
    OpticalFlow_Status_ENABLE,
};

/**
 * @brief 光流数据类型
 *
 */
enum Enum_OpticalFlow_Mode : uint8_t
{
    MODE_0 = 0,
    MODE_1,
	MODE_2,
};



/**
 * @brief 裁判系统命令码类型
 *
 */
enum Enum_OpticalFlow_Command_ID : uint8_t
{
	IMU_DATA = 0x01,
	QUATERNION_DATA = 0x04,
	HEIGHT_DATA = 0x34,
    OPTICALFLOW_DATA = 0x51,
};


/**
 * @brief IMU源数据
 *
 */
struct Struct_IMU_Raw_Data
{
	int16_t ACC_X;
    int16_t ACC_Y;
	int16_t ACC_Z;
	int16_t GYR_X;
	int16_t GYR_Y;
	int16_t GYR_Z;
	uint8_t SHOCK_STA;
} __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_Quaternion_Data
{
	int16_t V[4];
	uint8_t FUSION_STA;
} __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_Height_Data
{
	uint8_t DIRECTION;
	uint8_t ANGLE;
	uint32_t DIST;
} __attribute__((packed));

/**
 * @brief 光流模块源数据
 *
 */
struct Struct_OpticalFlow_UART_Data
{
    uint8_t Frame_Header;
	uint8_t Target_Add;
	Enum_OpticalFlow_Command_ID OpticalFlow_Command_ID;
    uint8_t Data_Length;
	uint8_t Data[50];
} __attribute__((packed));

/**
 * @brief 光流源数据
 *
 */
struct Struct_OpticalFlow_Raw_Data
{
	Enum_OpticalFlow_Mode Mode;
    uint8_t State;
	uint8_t DX_0;
	uint8_t DY_0;
	uint8_t Quality;
} __attribute__((packed));


/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_OpticalFlow_Integrate_Data_1
{
	Enum_OpticalFlow_Mode Mode;
    uint8_t State;
	uint8_t DX_1;
	uint8_t DY_1;
	uint8_t Quality;
} __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_OpticalFlow_Integrate_Data_2
{
	Enum_OpticalFlow_Mode Mode;
    uint8_t State;
	int16_t DX_2;
	int16_t DY_2;
	int16_t DX_FIX;
	int16_t DY_FIX;
	int16_t INTEG_X;
	int16_t INTEG_Y;
	uint8_t Quality;
	
//	~Struct_OpticalFlow_Integrate_Data_2()
//	{
//		memset(this,0,sizeof(Struct_OpticalFlow_Integrate_Data_2));
//	}
} __attribute__((packed));


/**
* @brief Specialized, 自定义控制器类
 *
 */
class Class_Opticalflow
{
public:
	void Init(UART_HandleTypeDef *huart);
	
	void UART_RxCpltCallback(uint8_t *Rx_Data);
    void Task_Alive_PeriodElapsedCallback();
	
	inline Enum_OpticalFlow_Status Get_OpticalFlow_Status();
	inline Struct_OpticalFlow_Integrate_Data_2 Get_OpticalFlow_Integrate_Data_2();
	inline Struct_Height_Data Get_Struct_Height_Data();
	inline Struct_Quaternion_Data Get_Quaternion_Data();
protected:
    //初始化相关常量

	//裁判系统状态
    Enum_OpticalFlow_Status OpticalFlow_Status = OpticalFlow_Status_DISABLE;
	Enum_OpticalFlow_Mode OpticalFlow_Mode = MODE_0;
    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
    //数据包头标
    uint8_t Frame_Header = 0xAA;
	uint8_t Target_Add = 0xFF;
    //常量

    //内部变量

    //当前时刻的裁判系统接收flag
    uint32_t Flag = 0;
    //前一时刻的裁判系统接收flag
    uint32_t Pre_Flag = 0;

    //读变量
	
	Struct_IMU_Raw_Data IMU_Raw_Data;
	
	Struct_Quaternion_Data Quaternion_Data;
	
	Struct_Height_Data Height_Data;
	
	Struct_OpticalFlow_Raw_Data OpticalFlow_Raw_Data;
	
	Struct_OpticalFlow_Integrate_Data_1 OpticalFlow_Integrate_Data_1;
	
	Struct_OpticalFlow_Integrate_Data_2 OpticalFlow_Integrate_Data_2;

    //读写变量

    //内部函数
    void Data_Process();

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


Enum_OpticalFlow_Status Class_Opticalflow::Get_OpticalFlow_Status()
{
	return OpticalFlow_Status;
}
Struct_OpticalFlow_Integrate_Data_2 Class_Opticalflow::Get_OpticalFlow_Integrate_Data_2()
{
	return (OpticalFlow_Integrate_Data_2);
}

Struct_Height_Data Class_Opticalflow::Get_Struct_Height_Data()
{
	return (Height_Data);
}

Struct_Quaternion_Data Class_Opticalflow::Get_Quaternion_Data()
{
	return (Quaternion_Data);
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
