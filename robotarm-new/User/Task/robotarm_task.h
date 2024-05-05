/**
 * @file robotarm_task.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 机械臂电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef ROBOTARM_TASK_H
#define ROBOTARM_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "dvc_djimotor.h"
#include "dvc_AKmotor.h"
#include "dvc_referee.h"
#include "cmsis_os.h"
#include "alg_fsm.h"
#include "dvc_dr16.h"
#include "chassis_Communication.h"
#include "dvc_dmmotor.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class Class_Robotarm;
/**
 * @brief 云台控制类型
 *
 */
enum Enum_Robotarm_Control_Type
{
    Robotarm_Control_Type_DISABLE = 0,
    Robotarm_Control_Type_NORMAL,
};

/**
 * @brief 解算任务运行阶段
 *
 */
enum Enum_Robotarm_Task_Status
{
    Robotarm_Task_Status_Calibration = 0,
    Robotarm_Task_Status_Resolution,
	Robotarm_Task_Status_decision,
};

/**
 * @brief 解算任务运行阶段
 *
 */
enum Enum_Joint_Limit_Flag
{
    Joint_Limit_Flag_Min = 0,
    Joint_Limit_Flag_Max,
};

struct Struct_Custom_Communication_Data {
	uint16_t Flow_x;
	uint16_t Flow_y;
    uint16_t roll; 
	uint16_t pitch;
	uint16_t yaw;
}__attribute__((packed));

//底盘移动
struct Chassis_Move_t
{	
	//移动速度
	float Chassis_Vx;
	float Chassis_Vy;
	float Chassis_Wz;
	
	//运算符重载 !=
	bool operator != (const Chassis_Move_t &a)
	{
		if((this->Chassis_Vx != a.Chassis_Vx)||(this->Chassis_Vy != a.Chassis_Vy)||(this->Chassis_Wz != a.Chassis_Wz))
		 {
			return true;
		 }
		 else 
		 {
			return false;
		 }
		
	}
};

//机械臂目标位姿
struct Position_Orientation_t
{
	//位置
	float X_Position;
	float Y_Position;
	float Z_Position;
	//姿态
	float Pitch_Angle;
	float Yaw_Angle;
	float Roll_Angle;
	
	//运算符重载 +
	Position_Orientation_t operator + (const Position_Orientation_t &a)
	{
		Position_Orientation_t c;
		c.X_Position = this->X_Position + a.X_Position;
		c.Y_Position = this->Y_Position + a.Y_Position;
		c.Z_Position = this->Z_Position + a.Z_Position;
		c.Pitch_Angle = this->Pitch_Angle + a.Pitch_Angle;
		c.Yaw_Angle = this->Yaw_Angle + a.Yaw_Angle;
		c.Roll_Angle = this->Roll_Angle + a.Roll_Angle;
		return c;
	}
	
	//运算符重载 !=
	bool operator != (const Position_Orientation_t &a)
	{
		if((this->X_Position != a.X_Position)||(this->Y_Position != a.Y_Position)||(this->Z_Position != a.Z_Position)
		  ||(this->Pitch_Angle != a.Pitch_Angle)||(this->Yaw_Angle != a.Yaw_Angle)||(this->Roll_Angle != a.Roll_Angle))
		 {
			return true;
		 }
		 else 
		 {
			return false;
		 }
		
	}
};
/**
 * @brief Specialized, 解算策略有限自动机
 *
 */
class Class_Robotarm_Resolution : public Class_FSM
{
public:
    Class_Robotarm *Robotarm;

    void Reload_Task_Status_PeriodElapsedCallback();
};

/**
* @brief Specialized, 机械臂类
 *
 */
class Class_Robotarm
{
public:
	//机械臂解算有限状态机
	friend class Class_Robotarm_Resolution;
	Class_Robotarm_Resolution Robotarm_Resolution;
	//遥控器
	Class_DR16 DR16;
	//裁判系统
    Class_Referee Referee;
	//底盘通信
	Class_Chassis_Communication Chassis_Communication;
	
    // 1轴电机
	Class_AK_Motor_80_6 Motor_Joint1;
	// 2轴电机
	Class_AK_Motor_80_6 Motor_Joint2;
	// 3轴电机
 	Class_DM_Motor_J4310 Motor_Joint3;
	// 4轴电机
	Class_DJI_Motor_C620 Motor_Joint4;
	// 6轴电机
	Class_DJI_Motor_C610 Motor_Joint5;
	//存放对象的地址，需要用强制转换
	uint32_t Motor_Joint[6]= {reinterpret_cast<uint32_t>(&Motor_Joint1),reinterpret_cast<uint32_t>(&Motor_Joint2),reinterpret_cast<uint32_t>(&Motor_Joint3),
							 reinterpret_cast<uint32_t>(&Motor_Joint4),reinterpret_cast<uint32_t>(&Motor_Joint5)};
	//初始化类
    void Init();

	//获得关节角度
	inline float Get_Joint_World_Angle(uint8_t Num);
	
	inline float Get_Joint_Offset_Angle(uint8_t Num);
	
	inline float Get_Joint_Limit_Angle(uint8_t Num);

	//设置目标位姿
	inline Position_Orientation_t Get_Target_Position_Orientation();
	//获得实际位姿
	inline Position_Orientation_t Get_Actual_Position_Orientation();
	//设置机械臂是否使能
	inline void Set_Robotarm_Control_Type(Enum_Robotarm_Control_Type __Robotarm_Control_Type);
	//设置关节角度
	inline void Set_Joint_World_Angle(uint8_t Num,float __Joint_World_Angle);
	//设置目标位姿
	inline void Set_Target_Position_Orientation(Position_Orientation_t __Target_Position_Orientation);
	
	//实际解算函数
	void Task_Calculate_PeriodElapsedCallback();
	//机械臂状态检测
	void Task_Alive_PeriodElapsedCallback();
	//目标位姿设置
	void Task_Control_Robotarm();
	//底盘通信
	void Task_Chassis_Communication_PeriodElapsedCallback();
	
	

protected:
    //初始化相关常量

    //常量

    //关节角度限制
    const float Joint_Limit_Angle[2][5] = {{-90.00f,-179.85f,-121.8f,-90.0f,-40.6f},
										   {91.22f,135.0f,109.88f,90.0f,207.541f}};

	const Enum_Joint_Limit_Flag Joint_Limit_Flag[5] = {Joint_Limit_Flag_Max,Joint_Limit_Flag_Min,Joint_Limit_Flag_Min,Joint_Limit_Flag_Max,Joint_Limit_Flag_Min};
	
//	float Min_Joint_Angle[6] = {-167.7f,-167.7f,-105.11f,0.0f,0,0};
//    // Joint最大值
//    float Max_Joint_Angle[6] = {140.2f,140.2f,120.52f,33.0f,0,0};
	//连杆1长度(mm)
	const float Arm1_Length = 215.0f;
	//连杆1长度平方(mm)
	const float Arm1_Length_2 = Arm1_Length*Arm1_Length;
	//连杆2长度(mm)
	const float Arm2_Length = 215.0f;
	//连杆2长度平方(mm)
	const float Arm2_Length_2 = Arm2_Length*Arm2_Length;
	//连杆1长度*连杆2长度(mm)
	const float Arm1_Length_multiply_Arm2_Length = 2*Arm1_Length*Arm2_Length;
	//连杆3长度(mm)
	const float Arm3_Length = 175.0f;
	//Joint4高度(mm)
	const float Joint4_Height = 95.0f;
	
	
	//遥控器拨动的死区, 0~1
    const float DR16_Dead_Zone = 0.01f;
	const float Controller_Dead_Zone = 0.1f;

	//DR16底盘x方向灵敏度系数
    const float Chassis_X_Resolution = 1.0f ;
    //DR16底盘y方向灵敏度系数
    const float Chassis_Y_Resolution = 1.0f ;
	//DR16底盘z方向灵敏度系数
    const float Chassis_Z_Resolution = 1.0f ;
	//DR16机械臂x方向灵敏度系数
    const float Robotarm_X_Resolution = 0.7f ;
    //DR16机械臂y方向灵敏度系数
    const float Robotarm_Y_Resolution = 0.7f ;
	//DR16机械臂z方向灵敏度系数
    const float Robotarm_Z_Resolution = 0.4f ;
	//DR16机械臂yaw灵敏度系数
    const float Robotarm_Yaw_Resolution = 0.1f ;
	//DR16机械臂pitch灵敏度系数
    const float Robotarm_Pitch_Resolution = 0.1f ;
	//DR16机械臂roll灵敏度系数
    const float Robotarm_Roll_Resolution = 0.1f ;
	
    //常量


    //内部变量

    //读变量

    //写变量

    //机械臂状态
    Enum_Robotarm_Control_Type Robotarm_Control_Type = Robotarm_Control_Type_NORMAL;

    //读写变量
	//底盘移动结构体
	Chassis_Move_t Chassis_Move;
	
	Struct_Custom_Communication_Data Custom_Communication_Data;
	//Joint坐标系角度
	float Joint_World_Angle[5] = {0.0f};
	// 到达机械限位后的电机角度值
	float Joint_Offset_Angle[5] = {0.0f,0.0f,0.0f,0.0f,0.0f};
	
	// 目标位置(mm)
	Position_Orientation_t Target_Position_Orientation = {209.314f, 0.0f,30.0f};
	// 两连杆目标位置(mm)
	Position_Orientation_t Target_Position_Orientation_Two;
	// 实际位置(mm)
	Position_Orientation_t Actual_Position_Orientation;
	// 连杆3Cos分解后的长度(mm)
	float Arm3_Cos_Length = 0.0f;
	
	//内部函数
	
	//通过模版函数来设置电机角度
	template <typename T1>
	inline void Set_Motor_Angle(T1 &Motor,uint8_t num);
	
	//电机上电校准
	bool Motor_Calibration(Class_DJI_Motor_C620 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle);
	bool Motor_Calibration(Class_AK_Motor_80_6 &Motor,uint8_t num,float Cali_Omega,float Target_Angle);
	bool Motor_Calibration(Class_DJI_Motor_C610 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle);
	//机械臂上电校准
	bool Robotarm_Calibration();
	//机械臂输出
    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取第Num个Joint的世界坐标系角度
 *
 * @return float 第Num个Joint的角度
 */
float Class_Robotarm::Get_Joint_World_Angle(uint8_t Num)
{
    return (Joint_World_Angle[Num-1]);
}

/**
 * @brief 获取第Num个Joint的世界坐标系角度
 *
 * @return float 第Num个Joint的角度
 */
float Class_Robotarm::Get_Joint_Offset_Angle(uint8_t Num)
{
    return (Joint_Offset_Angle[Num-1]);
}

/**
 * @brief 获取第Num个Joint的世界坐标系角度
 *
 * @return float 第Num个Joint的角度
 */
float Class_Robotarm::Get_Joint_Limit_Angle(uint8_t Num)
{
    return (Joint_Limit_Angle[Joint_Limit_Flag[Num-1]][Num-1]);
}

template <typename T1>
void Class_Robotarm::Set_Motor_Angle(T1 &Motor,uint8_t num)
{
	Motor.Set_Target_Angle(Joint_World_Angle[num-1] + Joint_Offset_Angle[num-1] - Joint_Limit_Angle[Joint_Limit_Flag[num-1]][num-1]);
}

/**
 * @brief 获取目标位置单位(mm)
 *
 * @return Position_Orientation_t 位置单位(mm)
 */
Position_Orientation_t Class_Robotarm::Get_Target_Position_Orientation()
{
    return (Target_Position_Orientation);
}

/**
 * @brief 获取实际位置单位(mm)
 *
 * @return Position_Orientation_t 位置单位(mm)
 */
Position_Orientation_t Class_Robotarm::Get_Actual_Position_Orientation()
{
    return (Actual_Position_Orientation);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Robotarm::Set_Robotarm_Control_Type(Enum_Robotarm_Control_Type __Robotarm_Control_Type)
{
    Robotarm_Control_Type = __Robotarm_Control_Type;
}

/**
 * @brief 设置第Num个Joint的世界坐标系角度
 *
 * @param Num 第Num个Joint __Joint_World_Angle 设置角度
 */
void Class_Robotarm::Set_Joint_World_Angle(uint8_t Num,float __Joint_World_Angle)
{
	Joint_World_Angle[Num-1] = __Joint_World_Angle;
}
/**
 * @brief 设定位置X单位(mm)
 *
 */
void Class_Robotarm::Set_Target_Position_Orientation(Position_Orientation_t __Target_Position_Orientation)
{
    Target_Position_Orientation = __Target_Position_Orientation;
}



#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
