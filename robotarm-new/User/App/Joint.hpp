#ifndef SCARA_ARM_HPP
#define SCARA_ARM_HPP


#define DEGREE_TO_RAD 0.017253f
#define RAD_TO_DEGREE 57.2957f
#define PI 3.14159f

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "DM_motor.h"
#include "AK_motor.h"
#include "cmsis_os.h"
#include "math.h"
#include "alg_smoothen.h"
#include "string.h"

typedef enum{
	JOINT_OK			= 0x00U,
	JOINT_ERROR			= 0x01U,
	JOINT_WRONG_PARAM	= 0x02U,
	JOINT_BUZY			= 0x03U,
} JOINT_RETURN_T;

typedef enum{
	JOINT_CALI_DIRECTION_CW		= 0x00U,
	JOINT_CALI_DIRECTION_CCW	= 0x01U,
} JOINT_REVOLUTE_CALIBRATION_DIRECTION_T;

typedef enum{
	JOINT_MOTOR_DM		= 0x00U,
	JOINT_MOTOR_AK		= 0x01U,
	JOINT_MOTOR_C620	= 0x02U,
	JOINT_MOTOR_C610	= 0x03U,
} JOINT_MOTOR_TYPE_T;

typedef enum{
	JOINT_MOTOR_ONLINE	= 0x00U,
	JOINT_MOTOR_OFFLINE	= 0x01U,
} JOINT_MOTOR_CONNECTION_STATE_T;

class revoluteJoint_c{
	public:
		// 关节初始化
		JOINT_RETURN_T jointInit(DM_motor_t* DmMotor);
		JOINT_RETURN_T jointInit(AK_motor_t* AkMotor);
		JOINT_RETURN_T jointDoCalibrate(JOINT_REVOLUTE_CALIBRATION_DIRECTION_T __caliDir=JOINT_CALI_DIRECTION_CCW);
		// 设置关节角度
		float jointOmegaMax;
		float jointDOmegaMax;
		JOINT_RETURN_T setBodyFrameJointAngle(float targetRad);
		// 获得关节角度
		
		float getBodyFrameJointAngle();
		// 设置关节机械限位角度
		JOINT_RETURN_T jointSetMechLimit(float cwLim, float ccwLim, float margin=0.2f);
		// 电机连接状态
		JOINT_RETURN_T jointConnectionStateUpdate();
		JOINT_MOTOR_CONNECTION_STATE_T jointGetConnectionState();
	private:
		// 电机相关
		JOINT_MOTOR_TYPE_T motorType;
		void* motor;
		float cwMechLimitRad;	// 从电机输出轴看，CW方向转到极限时相对杆伸出方向的夹角。为负值。
		float ccwMechLimitRad;	// 从电机输出轴看，CCW方向转到极限时相对杆伸出方向的夹角。为正值。
		float mechLimitMarginRad;	// 给关节机械限位留裕量，防止直接打到限位上
		// 进行关节校准相关
		JOINT_RETURN_T jointCaliInit(	float omege=1.f,
										float delPos=0.2f,
										uint8_t afLength=30,
										float tole=0.012f,
										JOINT_REVOLUTE_CALIBRATION_DIRECTION_T __caliDir=JOINT_CALI_DIRECTION_CCW);
		float calibratedPositionRad;	// 完成校准后输出的角度
		float caliOmegaRadPerSec;		// 校准时的角速度，应为正值
		float caliDeltaPosRad;			// 校准时的角度增值，应为正值
		float caliOmeDiffTolerance;		// 校准完成时期望的角速度均值滤波结果
		float*	filterInput;			// 均值滤波的原始数据存储
		uint8_t	averageFilterLength;	// 均值滤波长度
		JOINT_REVOLUTE_CALIBRATION_DIRECTION_T caliDir;	// 校准时，电机从输出轴看的转动方向
		float averageFilter(float input);
		JOINT_RETURN_T jointCalibrate(DM_motor_t* DmMotor);
		JOINT_RETURN_T jointCalibrate(AK_motor_t* AkMotor);
		// 控制相关
		float targetPosRad;
		JOINT_MOTOR_CONNECTION_STATE_T conectionState;
};

class scaraArm_c{
	public:
		
	private:
		
};




#endif
