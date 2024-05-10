/**
 * @file crt_chassis.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 底盘电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "Robotarm_Communication.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_ENABLE,
};

/**
 * @brief 抬升控制类型
 *
 */
enum Enum_Rise_Control_Type
{
    Rise_Control_Type_DISABLE = 0,
    Rise_Control_Type_ENABLE,
};


/**
 * @brief Specialized, 三轮舵轮底盘类
 *
 */
class Class_Chassis
{
public:

    //斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    //斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    //斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    //裁判系统
    Class_Referee Referee;
	//机械臂通信
	Class_Roboatrm_Communication Roboatrm_Communication;

	
    //四个驱动轮电机
    Class_DJI_Motor_C620 Motor_Wheel[4];
	//机械臂抬升电机
	//Class_DJI_Motor_C620 Motor_Rise;

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);

	void Init(float __Velocity_X_Max = 3.0f, float __Velocity_Y_Max = 3.0f, float __Omega_Max = 3.0f);
    void Task_Calculate_PeriodElapsedCallback();
	void Task_Alive_PeriodElapsedCallback();
	void UI_Task();
protected:
    //初始化相关常量

    //速度X限制
    float Velocity_X_Max = 4.0f;
    //速度Y限制
    float Velocity_Y_Max = 4.0f;
    //角速度限制
    float Omega_Max = 4.0f;

    float Height_Max = 0.5f;

	float Half_Width = 0.21f;
	
	float Half_Length = 0.19f;
	
	float Wheel_Radius   = 0.08f;
	
	
    //读变量


    //写变量

    //读写变量

    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_ENABLE;

    //抬升控制方法
    Enum_Rise_Control_Type Rise_Control_Type = Rise_Control_Type_DISABLE;

    float Rise_Offset_Angle = 0.0f;
    //目标速度X
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;
    //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;
	
	float Robotarm_Height = 0.0f;
    //内部函数

    template <typename T1>
	inline void Set_Motor_Angle(T1 &Motor);

	void Target_Resolution();
	void Inverse_Kinematic();
//	void Rise_Control();
//	bool Rise_Calibration(float Cali_Omega,float Cali_Max_Out, float Target_Height);
    void Output();
};

/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/


//template <typename T1>
//void Class_Chassis::Set_Motor_Angle(T1 &Motor)
//{
//	Motor.Set_Target_Angle(Robotarm_Height+Rise_Offset_Angle);
//}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}



/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Chassis::Set_Now_Omega(float __Now_Omega)
{
    Now_Omega = __Now_Omega;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
