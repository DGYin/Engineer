#include "robotarm.hpp"


robotarm_c robotarm;

inline float Robotarm_platformSine(float input);
inline float Robotarm_platformCosine(float input);


ROBOTARM_RETURN_T robotarm_c::Robotarm_Init()
{
	// 句柄初始化
	revjoint2.jointInit(&akMotor_joint2);
	revjoint3.jointInit(&akMotor_joint3);
	revjoint4.jointInit(&dmMotor_joint4);
	// 限位初始化
	revjoint2.jointSetMechLimit(90.00*DEGREE_TO_RAD,	91.22f*DEGREE_TO_RAD);
	revjoint3.jointSetMechLimit(179.85f*DEGREE_TO_RAD,	135.00f*DEGREE_TO_RAD);
	revjoint4.jointSetMechLimit(109.88f*DEGREE_TO_RAD,	109.88f*DEGREE_TO_RAD);
	return ROBOTARM_OK;
}


ROBOTARM_RETURN_T robotarm_c::Robotarm_QNowUpdate()
{
	jointQNow[2-1] = revjoint2.getBodyFrameJointAngle();
	jointQNow[3-1] = revjoint3.getBodyFrameJointAngle();
	jointQNow[4-1] = revjoint4.getBodyFrameJointAngle();
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_DoJointControl()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	if (armCalibrated == ROBOTARM_CALIBRATED)	// 完成校准后，关节才受这里的控制
	{
		revjoint2.setBodyFrameJointAngle(jointQTarget[2-1]);
		revjoint3.setBodyFrameJointAngle(jointQTarget[3-1]);
		revjoint4.setBodyFrameJointAngle(jointQTarget[4-1]);
	}
	else return ret = ROBOTARM_ERROR;	// 还没校准好，返回错误
	return ret;
	
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_SetEndPoseMat(tMatrix_t endPoseMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	memcpy(&endPoseMatTarget, &endPoseMat, 16*sizeof(float));
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
	// 执行校准。关节校准没完成返回值为1。取或，任何一个关节没校准完都会变成1（即ROBOTARM_UNCALIBRATED）
	uint8_t caliStatus = 0;
	if (armCalibrated == ROBOTARM_UNCALIBRATED)
	{
		caliStatus |= revjoint2.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
		caliStatus |= revjoint3.jointDoCalibrate(JOINT_CALI_DIRECTION_CW);
		caliStatus |= revjoint4.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
	}
	armCalibrated = (ROBOTARM_CALIBRATE_STATUS_T) caliStatus;
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
