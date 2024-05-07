#include "robotarm.hpp"


robotarm_c robotarm;

inline float Robotarm_platformSine(float input);
inline float Robotarm_platformCosine(float input);


ROBOTARM_RETURN_T robotarm_c::Robotarm_Init()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	// 句柄初始化
	priJoint1.jointInit(&m3508_joint1);
	revJoint2.jointInit(&akMotor_joint2);
	revJoint3.jointInit(&akMotor_joint3);
	revJoint4.jointInit(&dmMotor_joint4);
	revJoint5.jointInit(&m3508_joint5);
	revJoint6.jointInit(&m2006_joint6);
	// 限位初始化
	priJoint1.jointSetMechLimit(0.f, 0.23);
	revJoint2.jointSetMechLimit(90.00*DEGREE_TO_RAD,	91.22f*DEGREE_TO_RAD);
	revJoint3.jointSetMechLimit(179.85f*DEGREE_TO_RAD,	135.00f*DEGREE_TO_RAD);
	revJoint4.jointSetMechLimit(109.88f*DEGREE_TO_RAD,	109.88f*DEGREE_TO_RAD);
	revJoint5.jointSetMechLimit(90.00*DEGREE_TO_RAD,	90.00*DEGREE_TO_RAD);
	revJoint6.jointSetMechLimit(207.541f*DEGREE_TO_RAD,	40.6f*DEGREE_TO_RAD);
	return ret;
}


ROBOTARM_RETURN_T robotarm_c::Robotarm_QNowUpdate()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	jointQNow[1-1] = priJoint1.getBodyFrameJointDisplacement();
	jointQNow[2-1] = revJoint2.getBodyFrameJointAngle();
	jointQNow[3-1] = revJoint3.getBodyFrameJointAngle();
	jointQNow[4-1] = revJoint4.getBodyFrameJointAngle();
	jointQNow[5-1] = revJoint5.getBodyFrameJointAngle();
	jointQNow[6-1] = revJoint6.getBodyFrameJointAngle();
	return ret;
}

float targetHeight = 0.04f;
float nowHeight;
ROBOTARM_RETURN_T robotarm_c::Robotarm_DoJointControl()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	if (armCalibrated == ROBOTARM_CALIBRATED)	// 完成校准后，关节才受这里的控制
	{
		priJoint1.setBodyFrameJointDisplacement(jointQTarget[1-1]);
		revJoint2.setBodyFrameJointAngle(jointQTarget[2-1]);
		revJoint3.setBodyFrameJointAngle(jointQTarget[3-1]);
		revJoint4.setBodyFrameJointAngle(jointQTarget[4-1]);
		revJoint5.setBodyFrameJointAngle(0);	
		revJoint6.setBodyFrameJointAngle(0);
	}
	else return ret = ROBOTARM_ERROR;	// 还没校准好，返回错误
	return ret;
	
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_SetEndPoseMat(tMatrix_t endPoseMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	memcpy((uint8_t*)&endPoseMatTarget[0], (uint8_t*)&endPoseMat[0], 16*sizeof(float));
	return ret;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_SetEndPosNRpyTarget(posMatrix_t posMat, RpyMatrix_t rpyMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	// 设置旋转
	float c[3] = {	Robotarm_platformCosine(rpyMat[0]),
					Robotarm_platformCosine(rpyMat[1]),
					Robotarm_platformCosine(rpyMat[2])};
	float s[3] = {	Robotarm_platformSine(rpyMat[0]),
					Robotarm_platformSine(rpyMat[1]),
					Robotarm_platformSine(rpyMat[2])};
	// 通过rpy转为旋转
	endPoseMatTarget[0] 	= c[0] * c[1];						// R11
	endPoseMatTarget[1] 	= c[0] * s[1] * s[2] - s[0] * c[2];	// R12
	endPoseMatTarget[2] 	= c[0] * s[1] * c[2] + s[0] * s[2];	// R13
	endPoseMatTarget[4]		= s[0] * c[1];						// R21
	endPoseMatTarget[5]		= s[0] * s[1] * s[2] + c[0] * c[2];	// R22
	endPoseMatTarget[6]		= s[0] * s[1] * c[2] - c[0] * s[2];	// R23
	endPoseMatTarget[8]		= -s[1];							// R31
	endPoseMatTarget[9]		= c[1] * s[2];						// R32
	endPoseMatTarget[10]	= c[1] * c[2];						// R33
	// 设置位置
	endPoseMatTarget[3] 	= posMat[0];
	endPoseMatTarget[7]		= posMat[1];
	endPoseMatTarget[11]	= posMat[2];
	return ret;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_GetEndPosNRpyNow(posNRpyMatrix_t* posNRpyMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	Matrixf<4, 4> transMat;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			transMat[i][j] = endPoseMatNow[i*4+j];
	Matrixf<3, 1> posMat = robotics::t2p(transMat);
	Matrixf<3, 1> rpyMat = robotics::t2rpy(transMat);
	for (int i=0; i<3; i++)
	{
		memcpy(&posNRpyMat[i],		&posMat[i][0], sizeof(float));
		memcpy(&posNRpyMat[i+3],	&rpyMat[i][0], sizeof(float));
	}
	return ret;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_GetEndPoseMatNow(tMatrix_t* endPoseMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	memcpy(&endPoseMat, &endPoseMatNow, 16*sizeof(float));
	return ret;
}


/**
 * @brief 进行正运动学解算
 *
 * @return ROBOTARM_RETURN_T
 */
ROBOTARM_RETURN_T robotarm_c::Robotarm_FKine()
{
	// 设定参数
	robotics::Link links[6];
	// 上交用的是 SDH 法，请注意参数不同于 MDH
    links[0] = robotics::Link(0, 0,		81,		0,		robotics::P, 0, 	0, 0);
    links[1] = robotics::Link(0, 48.6,	213.38,	0,		robotics::R, 0, 	0, 0);
    links[2] = robotics::Link(0, 48.6,	225.26,	0,		robotics::R, 0, 	0, 0);
    links[3] = robotics::Link(0, 30.25,		0,	PI/2,	robotics::R, PI/2, 	0, 0);
    links[4] = robotics::Link(0, 226,	0,		PI/2,	robotics::R, PI/2, 	0, 0);
    links[5] = robotics::Link(0, 0,		0,		0,		robotics::R, PI/2,	0, 0);
    robotics::Serial_Link<6> scara(links);
	// 正解算
	Matrixf<4, 4> tFKine = scara.fkine(jointQNow);
	// 结果拷贝出来
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			endPoseMatNow[i*4+j] = tFKine[i][j];
	return ROBOTARM_OK;
}

/**
 * @brief 进行逆运动学解算。值得注意的是：由于使用的为迭代法，需要用到当前的Q，且无需对 Q 的运动进行最优化考虑。
 *
 * @return ROBOTARM_RETURN_T
 */
ROBOTARM_RETURN_T robotarm_c::Robotarm_IKine()
{
	// 设定参数
	robotics::Link links[6];
	// 上交用的是 SDH 法，请注意参数不同于 MDH
    links[0] = robotics::Link(0, 0,		81,		0,		robotics::P, 0, 	0, 0);
    links[1] = robotics::Link(0, 48.6,	213.38,	0,		robotics::R, 0, 	0, 0);
    links[2] = robotics::Link(0, 48.6,	225.26,	0,		robotics::R, 0, 	0, 0);
    links[3] = robotics::Link(0, 30.25,		0,	PI/2,	robotics::R, PI/2, 	0, 0);
    links[4] = robotics::Link(0, 226,	0,		PI/2,	robotics::R, PI/2, 	0, 0);
    links[5] = robotics::Link(0, 0,		0,		0,		robotics::R, PI/2,	0, 0);
    robotics::Serial_Link<6> scara(links);
	// 迭代法逆解算
	Matrixf<4, 4> tIKine;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			tIKine[i][j] = endPoseMatTarget[i*4+j];
    Matrixf<6, 1> qIKine = scara.ikine(tIKine, Matrixf<6, 1>(jointQNow));
	for (int i=0; i<6; i++)
		memcpy(&jointQTarget[i], qIKine[i], sizeof(float));
	return ROBOTARM_OK;
}


ROBOTARM_RETURN_T robotarm_c::Robotarm_CheckforCalibration()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	// 执行校准。关节校准没完成返回值为1。取或，任何一个关节没校准完都会变成1（即ROBOTARM_UNCALIBRATED）
	uint8_t caliStatus = 0;
	if (armCalibrated == ROBOTARM_UNCALIBRATED)
	{
		caliStatus |= priJoint1.jointDoCalibrate(JOINT_CALI_DIRECTION_BACKWARD);
		caliStatus |= revJoint2.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
		caliStatus |= revJoint3.jointDoCalibrate(JOINT_CALI_DIRECTION_CW);
		caliStatus |= revJoint4.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
		caliStatus |= revJoint5.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
		caliStatus |= revJoint6.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
		armCalibrated = (ROBOTARM_CALIBRATE_STATUS_T) caliStatus;
	}
	
	return ret;
}


/**
 * @brief 自定义 Sine 函数
 *
 * @param input
 * @return float
 */
inline float Robotarm_platformSine(float input)
{
	return arm_sin_f32(input);
}

/**
 * @brief 自定义 Cosine 函数
 *
 * @param input
 * @return float
 */
inline float Robotarm_platformCosine(float input)
{
	return arm_cos_f32(input);
}
