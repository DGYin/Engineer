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

#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "alg_fsm.h"
#include "drv_math.h"
#include "dvc_referee.h"
#include "dvc_opticalflow.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum Enum_Robotarm_Control_Type
{
    Robotarm_Control_Type_DISABLE = 0,
    Robotarm_Control_Type_NORMAL,
};
 
struct Struct_EulerAngles {
    float roll; 
	float pitch;
	float yaw;
};

struct Struct_Communication_Data {
	uint16_t Flow_x;
	uint16_t Flow_y;
    uint16_t roll; 
	uint16_t pitch;
	uint16_t yaw;
}__attribute__((packed));
/**
* @brief Specialized, 机械臂类
 *
 */
class Class_Controller
{
public:
	Class_Opticalflow Opticalflow;
	Class_Referee Referee;
	
	void Init();
	void ToEulerAngles();
	void Communication_To_Robot();
protected:
    //初始化相关常量

    //常量



	
    //常量


    //内部变量
	
    //读变量

    //写变量
	float Offset_Yaw;
	Struct_EulerAngles EulerAngles;
	Struct_Communication_Data Communication_Data;
	
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/




#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
