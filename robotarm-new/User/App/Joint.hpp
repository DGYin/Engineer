#ifndef SCARA_ARM_HPP
#define SCARA_ARM_HPP


#define DEGREE_TO_RAD 0.017253f
#define RAD_TO_DEGREE 57.2957f
#define PI 3.14159f

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
// 电机相关
#include "DM_motor.h"
#include "AK_motor.h"
#include "dvc_djimotor.h"
// 内存管理相关 （滤波用到）
#include "cmsis_os.h"
// 数学相关
#include "math.h"
#include "alg_smoothen.h"


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
	JOINT_CALI_DIRECTION_BACKWARD	= 0x00U,
	JOINT_CALI_DIRECTION_FORWARD	= 0x01U,
}	JOINT_PRISMATIC_CALI_DIRECTION_T;

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


class prismaticJoint_c{	// 关节的所有位移都为 m
	public:
		// 关节初始化
		JOINT_RETURN_T jointInit(Class_DJI_Motor_C620* djiC620Motor);
		
		JOINT_RETURN_T jointDoCalibrate(JOINT_PRISMATIC_CALI_DIRECTION_T __caliDir=JOINT_CALI_DIRECTION_BACKWARD);
		// 设置关节位移
		JOINT_RETURN_T setBodyFrameJointDisplacement(float targetDisp);
		// 获得关节角度
		float getBodyFrameJointDisplacement();
		// 设置关节机械限位角度
		JOINT_RETURN_T jointSetMechLimit(float upperLim, float lowerLim, float margin=0.01f);
		float GetLowerLimit();		// 获得带有软件缓冲角度的机械限位值
		float GetUpperLimit();
		// 电机连接状态
		JOINT_RETURN_T jointConnectionStateUpdate();
		JOINT_MOTOR_CONNECTION_STATE_T jointGetConnectionState();
	private:
		// 电机相关
		JOINT_MOTOR_TYPE_T motorType;
		void* motor;
		float upperMechLimSi;				// 向关节轴的正方向移动到极限的长度。为正值
		float lowerMechLimSi;				// 向关节轴的反方向移动到极限的长度。为正值
		float mechLimitMarginSi;		// 给关节机械限位留裕量，防止直接打到限位上
		float radToDisplacementSi;		// 将关节转动角度转化为关节位移的系数。注意有正负号
		float radPerSecondToVelocitySi;	// 将关节转动角速度转化为关节位移的系数。注意有正负号
		// 进行关节校准相关
		JOINT_RETURN_T jointCaliInit(	float velocity=0.1f,
										float acceleration=0.005f,
										uint8_t afLength=200,
										float tole=1.f,
										JOINT_PRISMATIC_CALI_DIRECTION_T __caliDir=JOINT_CALI_DIRECTION_BACKWARD);
		float calibratedDisplacementSi;		// 完成校准后输出的位移，用于矫正零点
		float caliVelocitySi;				// 校准时的角速度，应为正值
		float caliDeltaDisplacementSi;		// 校准时的角度增值，应为正值
		float caliVelocityDiffTolerance;	// 校准完成时期望的角速度均值滤波结果
		float*	filterInput;				// 均值滤波的原始数据存储
		uint8_t	averageFilterLength;		// 均值滤波长度
		JOINT_PRISMATIC_CALI_DIRECTION_T caliDir;	// 校准时，电机从输出轴看的转动方向
		float averageFilter(float input);
		JOINT_RETURN_T jointCalibrate(Class_DJI_Motor_C620* djiC620Motor);
		// 控制相关
		float targetPosSi;
		float jointVelocityMaxSi;
		JOINT_MOTOR_CONNECTION_STATE_T conectionState;
};

class revoluteJoint_c{	// 关节的所有角度都为弧度
	public:
		// 关节初始化
		JOINT_RETURN_T jointInit(DM_motor_t* DmMotor);
		JOINT_RETURN_T jointInit(AK_motor_t* AkMotor);
		JOINT_RETURN_T jointInit(Class_DJI_Motor_C620* djiC620Motor);
		JOINT_RETURN_T jointInit(Class_DJI_Motor_C610* djiC610Motor);
		
		JOINT_RETURN_T jointDoCalibrate(JOINT_REVOLUTE_CALIBRATION_DIRECTION_T __caliDir=JOINT_CALI_DIRECTION_CCW);
		// 设置关节角度
		float jointOmegaMax;
		float jointDOmegaMax;
		JOINT_RETURN_T setBodyFrameJointAngle(float targetRad);
		// 获得关节角度
		float getBodyFrameJointAngle();
		// 关节机械限位角度相关
		JOINT_RETURN_T jointSetMechLimit(float cwLim, float ccwLim, float margin=0.2f);
		float GetCwMechLimit();		// 获得不带软件缓冲角度的机械限位值
		float GetCcwMechLimit();
		float GetCwLimit();		// 获得带有软件缓冲角度的机械限位值
		float GetCcwLimit();
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
		float reductionRate = 1.f;	// 传动比，注意包含正负号表示正反转。（关节轴比电机轴尺比）
		// 进行关节校准相关
		JOINT_RETURN_T jointCaliInit(	float omege=1.2f,
										float delPos=0.1f,
										uint8_t afLength=40,
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
		JOINT_RETURN_T jointCalibrate(Class_DJI_Motor_C620* djiC620Motor);
		JOINT_RETURN_T jointCalibrate(Class_DJI_Motor_C610* djiC620Motor);
		// 控制相关
		float targetPosRad;
		JOINT_MOTOR_CONNECTION_STATE_T conectionState;
};

class scaraArm_c{
	public:
		
	private:
		
};




#endif
