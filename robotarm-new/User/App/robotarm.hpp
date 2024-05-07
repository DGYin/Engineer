#ifndef ROBOTARM_HPP
#define ROBOTARM_HPP

#include "stdint.h"
#include "Joint.hpp"
#include "arm_math.h"
#include "string.h"
#include "robotics.h"
#include "matrix.h"


typedef enum{
	ROBOTARM_OK		= 0x00U,
	ROBOTARM_ERROR	= 0x01U,
} ROBOTARM_RETURN_T;

typedef enum{
	ROBOTARM_END_SUCTION_CUP	= 0x00U,
	ROBOTARM_END_ORE			= 0x01U,
} ROBOTARM_END_TYPE_T;

typedef enum{
	ROBOTARM_CALIBRATED		= 0x00U,
	ROBOTARM_UNCALIBRATED	= 0x01U,
} ROBOTARM_CALIBRATE_STATUS_T;

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



typedef float qMatrix_t[6];			// q matrix
typedef float tMatrix_t[16];		// homogenius transformation matrix
typedef float posMatrix_t[3];		// [x, y, z] in mm
typedef float RpyMatrix_t[3];		// [r, p, y] in rad
typedef float posNRpyMatrix_t[6];	// [x, y, z, r, p, y] in mm or rad

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
		ROBOTARM_RETURN_T Robotarm_SetEndPosNRpyTarget(posMatrix_t posMat, RpyMatrix_t rpyMat);
		ROBOTARM_RETURN_T Robotarm_DoJointControl();
		// 获取姿态
		ROBOTARM_RETURN_T Robotarm_GetEndPoseMatNow(tMatrix_t* endPoseMat);
		ROBOTARM_RETURN_T Robotarm_GetEndPosNRpyNow(posNRpyMatrix_t* posNRpyMat);
		// 校正相关
		ROBOTARM_RETURN_T Robotarm_CheckforCalibration();
		// 运动学相关
		ROBOTARM_RETURN_T Robotarm_FKine();
		ROBOTARM_RETURN_T Robotarm_IKine();
		ROBOTARM_RETURN_T Robotarm_QNowUpdate();
	private:
		// 末端相关
		robotarm_Pose_t		endTargetPosNPose_b;	// b 代表 Body Frame，机体坐标系，符合右手定则
		tMatrix_t endPoseMatTarget;	// 目标末端位姿，单位mm，是逆解算的输入
		tMatrix_t endPoseMatNow;		// 当前末端位姿，单位mm，是正解算的输入
		ROBOTARM_END_TYPE_T endType;
		// 关节相关
		robotarm_jointToNextParameter_t	jointBarParam;
		qMatrix_t	jointQNow;		// 当前关节Q矩阵，单位mm或rad，是正解算的结果和逆解算的输入
		qMatrix_t	jointQTarget;	// 目标关节Q矩阵，单位mm或rad，是逆解算的结果
		// 校准相关
		ROBOTARM_CALIBRATE_STATUS_T armCalibrated = ROBOTARM_UNCALIBRATED;	// 默认没校准
		
};

extern robotarm_c robotarm;


#endif
