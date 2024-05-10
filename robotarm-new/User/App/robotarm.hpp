#ifndef ROBOTARM_HPP
#define ROBOTARM_HPP

#include "stdint.h"
#include "Joint.hpp"
#include "arm_math.h"
#include "string.h"
#include "robotics.h"
#include "matrix.h"
#include "alg_smoothen.h"
#include "chassis_Communication.h"
#include "dvc_dr16.h"
#include "dvc_referee.h"
//#include "robotarm_shortcut.hpp"

typedef enum{
	ROBOTARM_OK		= 0x00U,
	ROBOTARM_ERROR	= 0x01U,
} ROBOTARM_RETURN_T;

typedef enum{
	ROBOTARM_END_SUCTION_CUP	= 0x00U,
	ROBOTARM_END_ORE			= 0x01U,
} ROBOTARM_END_TYPE_T;

typedef enum{
	ROBOTARM_MUTEX_FREE		= 0x00U,
	ROBOTARM_MUTEX_OCCUPIED	= 0x01U,
} ROBOTARM_MUTEX_T;

typedef enum{
	ROBOTARM_CALIBRATED		= 0x00U,
	ROBOTARM_UNCALIBRATED	= 0x01U,
} ROBOTARM_CALIBRATE_STATUS_T;

typedef enum{
	ROBOTARM_END_SMOOTHEN_LINEAR	= 0x00U,
	
} ROBOTARM_END_SMOOTHEN_TYPE_T;

typedef struct{
	float barJ1ToJ2_Si;
	float barJ2ToJ3_Si;
	float barJ3ToJ4_Si;
	float barJ4ToJ5_Si;
	float barJ5ToSuctionCup_Si;
	float barSuctionCupToOre_Si;
} robotarm_jointToNextParameter_t;

typedef struct{
	float prismaticJoint1;
	float reeveluteJoint2;
	float reveluteJoint3;
	float reveluteJoint4;
	float reveluteJoint5;
	float reveluteJoint6;
} robotarm_6JointStatePack_t;

typedef struct{
	float x;
	float y;
	float z;
	float yaw;
	float pitch;
	float roll;
} robotarm_Pose_t;

// 金妈的逆解算所需结构体
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

// 金妈双板通讯所需结构体
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
const float Robotarm_X_Resolution = 0.5f ;
//DR16机械臂y方向灵敏度系数
const float Robotarm_Y_Resolution = 0.5f ;
//DR16机械臂z方向灵敏度系数
const float Robotarm_Z_Resolution = 0.4f ;
//DR16机械臂yaw灵敏度系数
const float Robotarm_Yaw_Resolution = 0.01f ;
//DR16机械臂pitch灵敏度系数
const float Robotarm_Pitch_Resolution	= 0.1f ;
const float Robotarm_Roll_Resolution	= 0.1f;
//DR16机械臂roll灵敏度系数
const float Robotarm_Height_Resolution = 0.0002f ;
	
typedef float qMatrix_t[6];			// q matrix
typedef float tMatrix_t[16];		// homogenius transformation matrix
typedef float posMatrix_t[3];		// [x, y, z] in mm
typedef float RpyMatrix_t[3];		// [r, p, y] in rad
typedef float posNRpyMatrix_t[6];	// [x, y, z, r, p, y] in mm or rad

static qMatrix_t lDefQ = {0, PI/3.f, -2.f*PI/3.f, PI/3.f, 0, 0};
static qMatrix_t rDefQ  = {0, -PI/3.f, 2.f*PI/3.f, -PI/3.f, 0, 0};

class robotarm_c{
	public:
		// 初始化
		ROBOTARM_RETURN_T Robotarm_Init();
		// 关节设置
		prismaticJoint_c	priJoint1;
		revoluteJoint_c		revJoint2;
		revoluteJoint_c		revJoint3;
		revoluteJoint_c		revJoint4;
		revoluteJoint_c		revJoint5;
		revoluteJoint_c		revJoint6;
		// 控制相关
		ROBOTARM_RETURN_T Robotarm_SetEndPoseMat(tMatrix_t endPoseMat);
		ROBOTARM_RETURN_T Robotarm_SetEndPosNRpyTarget(posNRpyMatrix_t posNRpyMat);
		ROBOTARM_RETURN_T Robotarm_SetQMatTarget(qMatrix_t qMat);
		ROBOTARM_RETURN_T Robotarm_DoJointControl();
		//ROBOTARM_RETURN_T Robotarm_
		// 状态获取相关
		ROBOTARM_RETURN_T Robotarm_GetQMat(qMatrix_t qMat);
		ROBOTARM_RETURN_T Robotarm_GetEndPoseMatNow(tMatrix_t endPoseMat);
		ROBOTARM_RETURN_T Robotarm_GetEndPosNRpyNow(posNRpyMatrix_t posNRpyMat);
		// 校正相关
		ROBOTARM_RETURN_T Robotarm_CheckforCalibration();
		// 运动学相关
		ROBOTARM_RETURN_T Robotarm_FKine();
		ROBOTARM_RETURN_T Robotarm_IKine();
		ROBOTARM_RETURN_T Robotarm_IKineGeo();
		ROBOTARM_RETURN_T Robotarm_QNowUpdate();
		// 金妈的底盘通信
		Class_Chassis_Communication Chassis_Communication;
		void Task_Chassis_Communication_PeriodElapsedCallback();
		// 金妈的遥控器
		Class_DR16 DR16;
		void Task_Control_Robotarm();
		// 金妈的裁判系统
		Class_Referee Referee;
	private:
		// 末端相关
		robotarm_Pose_t	endTargetPosNPose_b;	// b 代表 Body Frame，机体坐标系，符合右手定则
		tMatrix_t		endPoseMatTarget;		// 目标末端位姿，单位mm，是逆解算的输入
		tMatrix_t		endPoseMatNow;			// 当前末端位姿，单位mm，是正解算的输入
		posNRpyMatrix_t	endPosNRpyNow;			// 当前末端位姿，单位是 mm 和 rad
		posNRpyMatrix_t endPosNRpyTarget = {300.0, -0, 127.45, 0, 0, 0};		// 目标末端位姿，单位是 mm 和 rad，是几何法逆解算的输入
		ROBOTARM_END_TYPE_T endType;
		// 关节相关
		robotarm_jointToNextParameter_t	jointBarParam;
		qMatrix_t	jointQNow;		// 当前关节Q矩阵，单位mm或rad，是正解算的结果和逆解算的输入
		ROBOTARM_MUTEX_T	qTargetMutex;	// 互斥锁标记正在被占用
		qMatrix_t			jointQTarget;	// 目标关节Q矩阵，单位mm或rad，是逆解算的结果
		// 校准相关
		ROBOTARM_CALIBRATE_STATUS_T armCalibrated = ROBOTARM_UNCALIBRATED;	// 默认没校准
		// 缓动相关
		float dPosMax	= 1;
		float dRotMax	= 0.03;
		algSmoothen_uniformSmoothen_t dPosSmoothen;
		algSmoothen_uniformSmoothen_t dRotSmoothen;
		// 金妈逆解算的成员
		float Joint_World_Angle[5] = {0.0f};	//Joint坐标系角度
		Position_Orientation_t posNPyrTarget = {526.32, -10.288, 127.45};
		float targetHeight = 0;
		//Position_Orientation_t Last_Correct_Position_Orientation= {526.32, -10.288, 127.45};
		// 金妈双板通讯的成员
		//底盘移动结构体
		Chassis_Move_t Chassis_Move;
		Struct_Custom_Communication_Data Custom_Communication_Data;
		Position_Orientation_t Last_Correct_Position_Orientation;
};




extern robotarm_c robotarm;


#endif
