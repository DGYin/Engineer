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
	priJoint1.jointSetMechLimit(0.f, 0.285);
	revJoint2.jointSetMechLimit(90.00*DEGREE_TO_RAD,	91.22f*DEGREE_TO_RAD);
	revJoint3.jointSetMechLimit(179.85f*DEGREE_TO_RAD,	135.00f*DEGREE_TO_RAD);
	revJoint4.jointSetMechLimit(109.88f*DEGREE_TO_RAD,	109.88f*DEGREE_TO_RAD);
	revJoint5.jointSetMechLimit(90.00*DEGREE_TO_RAD,	90.00*DEGREE_TO_RAD);
	revJoint6.jointSetMechLimit(207.541f*DEGREE_TO_RAD,	40.6f*DEGREE_TO_RAD);
	// 缓动初始化
	AlgSmoothen_UsHanldeInitExample(&dPosSmoothen, UNIFORM_SMOOTHEN_TYPE_RELATIVE);
	AlgSmoothen_SetUsDelta(&dPosSmoothen, dPosMax, dPosMax);
	AlgSmoothen_UsHanldeInitExample(&dRotSmoothen, UNIFORM_SMOOTHEN_TYPE_RELATIVE);
	AlgSmoothen_SetUsDelta(&dRotSmoothen, dRotMax, dRotMax);
	// 双板通信初始化
	Chassis_Communication.Init(&hcan2);
	//遥控器初始化
	DR16.Init(&huart3);
	//裁判系统初始化
	Referee.Init(&huart6);
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
	
	jointQNow[4] = 0; jointQNow[5] = 0;
	return ret;
}

float targetHeight = 0.05f;
float nowHeight;
float qTar[6];
ROBOTARM_RETURN_T robotarm_c::Robotarm_DoJointControl()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	if (armCalibrated == ROBOTARM_CALIBRATED)	// 完成校准后，关节才受这里的控制
	{
//		priJoint1.setBodyFrameJointDisplacement(jointQTarget[1-1]);
//		revJoint2.setBodyFrameJointAngle(jointQTarget[2-1]);
//		revJoint3.setBodyFrameJointAngle(jointQTarget[3-1]);
//		revJoint4.setBodyFrameJointAngle(jointQTarget[4-1]);
//		revJoint5.setBodyFrameJointAngle(jointQTarget[5-1]);	
//		revJoint6.setBodyFrameJointAngle(-PI/4);
//		revJoint6.setBodyFrameJointAngle(0);
		for (int i=0; i<6; i++)
			qTar[i] = jointQTarget[i];
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

ROBOTARM_RETURN_T robotarm_c::Robotarm_SetEndPosNRpyTarget(posNRpyMatrix_t posNRpyMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	// 进行缓动
	posNRpyMatrix_t smoothen;
//	AlgSmoothen_UniformSmoothen(&dPosSmoothen, endPosNRpyNow[0], posNRpyMat[0], &smoothen[0]);
//	AlgSmoothen_UniformSmoothen(&dPosSmoothen, endPosNRpyNow[1], posNRpyMat[1], &smoothen[1]);
//	AlgSmoothen_UniformSmoothen(&dPosSmoothen, endPosNRpyNow[2], posNRpyMat[2], &smoothen[2]);
//	AlgSmoothen_UniformSmoothen(&dPosSmoothen, endPosNRpyNow[3], posNRpyMat[3], &smoothen[3]);
//	AlgSmoothen_UniformSmoothen(&dPosSmoothen, endPosNRpyNow[4], posNRpyMat[4], &smoothen[4]);
//	AlgSmoothen_UniformSmoothen(&dPosSmoothen, endPosNRpyNow[5], posNRpyMat[5], &smoothen[5]);
	for (int i=0; i<6; i++)
	{
		smoothen[i]			= posNRpyMat[i];
		endPosNRpyTarget[i] = posNRpyMat[i];
	}
	
	// 提取转动与位置
	float posMat[3] = {smoothen[0], smoothen[1], smoothen[2]};
	float rpyMat[3] = {smoothen[3], smoothen[4], smoothen[5]};
	// 计算旋转所需
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

ROBOTARM_RETURN_T robotarm_c::Robotarm_GetEndPosNRpyNow(posNRpyMatrix_t posNRpyMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	Matrixf<4, 4> transMat;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			transMat[i][j] = endPoseMatNow[i*4+j];
	Matrixf<3, 1> posMat = robotics::t2p(transMat);
	Matrixf<3, 1> rpyMat = robotics::t2rpy(transMat);
	
//	float rpy[3] = {
//      atan2f(R[1][0], R[0][0]),                                        // yaw
//      atan2f(-R[2][0], sqrtf(R[2][1] * R[2][1] + R[2][2] * R[2][2])),  // pitch
//      atan2f(R[2][1], R[2][2])                                         // roll
//  };

	
	
	for (int i=0; i<3; i++)
	{
		memcpy(&posNRpyMat[i],		&posMat[i][0], sizeof(float));
		memcpy(&posNRpyMat[i+3],	&rpyMat[i][0], sizeof(float));
	}
	return ret;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_GetEndPoseMatNow(tMatrix_t endPoseMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	memcpy(endPoseMat, endPoseMatNow, 16*sizeof(float));
	return ret;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_GetQMat(qMatrix_t qMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	for (int i=0; i<6; i++)
		qMat[i] = jointQNow[i];
	return ret;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_SetQMatTarget(qMatrix_t qMat)
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	for (int i=0; i<6; i++)
		jointQTarget[i] = qMat[i];
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
	Robotarm_GetEndPosNRpyNow(endPosNRpyNow);	// 顺便更新当前末端位姿
	return ROBOTARM_OK;
}

/**
 * @brief 进行逆运动学解算。值得注意的是：由于使用的为迭代法，需要用到当前的Q。
 *
 * @return ROBOTARM_RETURN_T
 */
ROBOTARM_RETURN_T robotarm_c::Robotarm_IKine()
{
	if (armCalibrated == ROBOTARM_CALIBRATED)	// 完成校准后，才进行解算
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
		Matrixf<6, 1>qNow = Matrixf<6, 1>(jointQNow);
		Matrixf<6, 1> qIKine = scara.ikine(tIKine, qNow);
		// 结果输出
//		for (int i=0; i<6; i++)
//			memcpy(&jointQTarget[i], qIKine[i], sizeof(float));
		return ROBOTARM_OK;
	}
	return ROBOTARM_ERROR;	// 还没完成校准，返回错误
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_IKineGeo()
{
	ROBOTARM_RETURN_T ret;
	
	float X2_add_Y2;
	float Arm1_Length = 215.0f;		//连杆1长度(mm)
	float Arm1_Length_2 = Arm1_Length*Arm1_Length;	//连杆1长度平方(mm)
	float Arm2_Length = 215.0f;		//连杆2长度(mm)
	float Arm2_Length_2 = Arm2_Length*Arm2_Length;	//连杆2长度平方(mm)
	float Arm1_Length_multiply_Arm2_Length = 2*Arm1_Length*Arm2_Length;	//连杆1长度*连杆2长度(mm)
	float Arm3_Length = 226.0f;		//连杆3长度(mm)
	float Joint4_Height = 95.0f;	//Joint4高度(mm)
	float Joint_Limit_Angle[2][5] = {{	-revJoint2.GetCwLimit(),
										-revJoint3.GetCwLimit(),
										-revJoint4.GetCwLimit(),
										-revJoint5.GetCwLimit(),
										-revJoint6.GetCwLimit(),
										},
									{	revJoint2.GetCcwLimit(),
										revJoint3.GetCcwLimit(),
										revJoint4.GetCcwLimit(),
										revJoint5.GetCcwLimit(),
										revJoint6.GetCcwLimit(),
										}};
//	Position_Orientation_t endNow;
//	endNow.X_Position	= endPosNRpyNow[0];
//	endNow.Y_Position	= endPosNRpyNow[1];
//	endNow.Z_Position	= endPosNRpyNow[2];
//	endNow.Pitch_Angle	= endPosNRpyNow[3];
//	endNow.Yaw_Angle	= endPosNRpyNow[4];
//	endNow.Roll_Angle	= endPosNRpyNow[5] + PI/2.f;
//										
//	Last_Correct_Position_Orientation.X_Position	= endPosNRpyNow[0];
//	Last_Correct_Position_Orientation.Y_Position	= endPosNRpyNow[1];
//	Last_Correct_Position_Orientation.Z_Position	= endPosNRpyNow[2];
//	Last_Correct_Position_Orientation.Pitch_Angle	= endPosNRpyNow[3];
//	Last_Correct_Position_Orientation.Yaw_Angle		= -endPosNRpyNow[4];
//	Last_Correct_Position_Orientation.Roll_Angle	= endPosNRpyNow[5] + PI/2.f;
	//关节3的位置
	Position_Orientation_t Position_Rotation={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
//	posNPyrTarget = {	endPosNRpyTarget[0],	// X
//						endPosNRpyTarget[1],	// Y
//						endPosNRpyTarget[2],	// Z
//						endPosNRpyTarget[3],	// Pitch
//						endPosNRpyTarget[4],	// Yaw
//						endPosNRpyTarget[5]};	// Roll
	
	//posNPyrTarget = 	{526.32, -10.288, 127.45, 0, 0, 0};
	float temp_angle = (posNPyrTarget.Yaw_Angle);
	
	float endTargetLength = sqrt(Position_Rotation.X_Position*Position_Rotation.X_Position + Position_Rotation.Y_Position*Position_Rotation.Y_Position);
	
	if (endTargetLength > 670)
	{
		Position_Rotation.X_Position = 670/endTargetLength * Position_Rotation.X_Position;
		Position_Rotation.Y_Position = 670/endTargetLength * Position_Rotation.Y_Position;
	}
	
	Position_Rotation.X_Position = -(Arm3_Length * arm_cos_f32(temp_angle));
	Position_Rotation.Y_Position = -(Arm3_Length * arm_sin_f32(temp_angle));
	//获得2连杆的目标位置
	Position_Orientation_t twoLinkBarPosNPyrTarget;
	twoLinkBarPosNPyrTarget = posNPyrTarget + Position_Rotation;
	

	//关节3的x2+y2
	X2_add_Y2 = twoLinkBarPosNPyrTarget.X_Position*twoLinkBarPosNPyrTarget.X_Position+
				twoLinkBarPosNPyrTarget.Y_Position*twoLinkBarPosNPyrTarget.Y_Position;
//	if (X2_add_Y2 > 160000)
//	{
//		X2_add_Y2 = 160000;
//		twoLinkBarPosNPyrTarget.X_Position = 400/sqrt(X2_add_Y2) * twoLinkBarPosNPyrTarget.X_Position;
//		twoLinkBarPosNPyrTarget.Y_Position = 400/sqrt(X2_add_Y2) * twoLinkBarPosNPyrTarget.Y_Position;
//	}
	//为使该三角形成立，到目标点的距离√(x2+y2 )必须小于或等于两个连杆的长度之和l1+l2
	if((X2_add_Y2) <=
	(Arm1_Length_2 + Arm2_Length_2 + Arm1_Length_multiply_Arm2_Length))
	{
		//关节2的角度计算，可能是一对正负值
		float Cos_Joint2_Angle = (X2_add_Y2 - Arm1_Length_2 - Arm2_Length_2)/
								(Arm1_Length_multiply_Arm2_Length);
		float Joint2_Angle = acosf(Cos_Joint2_Angle);
		//关节2的位置与x轴夹角
		float Beta_Two = atan2f(twoLinkBarPosNPyrTarget.Y_Position,
								twoLinkBarPosNPyrTarget.X_Position);
		float sqrt_x2_y2= 0.0f;
		arm_sqrt_f32(X2_add_Y2,&sqrt_x2_y2);
		
		float Cos_Psi = (X2_add_Y2 + Arm1_Length_2 - Arm2_Length_2)/
						(2 * Arm1_Length * sqrt_x2_y2);
		float Psi = acos(Cos_Psi);
		
		float Joint1_Angle_1 = (Beta_Two + Psi);
		float Joint1_Angle_2 = (Beta_Two - Psi);

		static float Last_Joint_World_Angle[2] = {0.0f};
		Last_Joint_World_Angle[1-1] = jointQNow[1];
		Last_Joint_World_Angle[2-1] = jointQNow[2];
		
		bool Joint1_Angle_1_Limit			= Math_Judge_Threshold(Joint1_Angle_1,	Joint_Limit_Angle[0][0], Joint_Limit_Angle[1][0]);
		bool Joint2_Angle_Limit_Negative	= Math_Judge_Threshold(-Joint2_Angle,	Joint_Limit_Angle[0][1], Joint_Limit_Angle[1][1]);
		bool Joint1_Angle_2_Limit			= Math_Judge_Threshold(Joint1_Angle_2,	Joint_Limit_Angle[0][0], Joint_Limit_Angle[1][0]);
		bool Joint2_Angle_Limit				= Math_Judge_Threshold(Joint2_Angle,	Joint_Limit_Angle[0][1], Joint_Limit_Angle[1][1]);
		float Joint_World_Temp_Angle[2] = {0.0f};
		//通过限位决策不同方案
		if((Joint1_Angle_1_Limit == true)&&
		   (Joint2_Angle_Limit_Negative == true)&&
		   (Joint1_Angle_2_Limit == true)&&
		   (Joint2_Angle_Limit == true))
		{
			float Total_Positon_Angle_Delta_1 = 2*fabs(Joint1_Angle_1 - Last_Joint_World_Angle[1-1]) + fabs(-Joint2_Angle - Last_Joint_World_Angle[2-1]);
			float Total_Positon_Angle_Delta_2 = 2*fabs(Joint1_Angle_2 - Last_Joint_World_Angle[1-1]) + fabs(Joint2_Angle - Last_Joint_World_Angle[2-1]);

			if(Total_Positon_Angle_Delta_1 >= Total_Positon_Angle_Delta_2)
			{
				Joint_World_Temp_Angle[1-1] = Joint1_Angle_2;
				Joint_World_Temp_Angle[2-1] = Joint2_Angle;
			}
			else
			{
				Joint_World_Temp_Angle[1-1] = Joint1_Angle_1;
				Joint_World_Temp_Angle[2-1] = -Joint2_Angle;
			}
		}
		else if((Joint1_Angle_1_Limit == true)&&
				(Joint2_Angle_Limit_Negative == true)&&
				((Joint1_Angle_2_Limit == false)||
				  (Joint2_Angle_Limit) == false))
		{
			Joint_World_Temp_Angle[1-1] = Joint1_Angle_1;
			Joint_World_Temp_Angle[2-1] = -Joint2_Angle;
		}
		else if(((Joint1_Angle_1_Limit == false)||
				 (Joint2_Angle_Limit_Negative == false))&&
				 (Joint1_Angle_2_Limit == true)&&
				 (Joint2_Angle_Limit == true))
		{
			Joint_World_Temp_Angle[1-1] = Joint1_Angle_2;
			Joint_World_Temp_Angle[2-1] = Joint2_Angle;
		}
		else
		{
			posNPyrTarget = Last_Correct_Position_Orientation;
			return ret = ROBOTARM_ERROR;
		}
		
		
		//关节3角度赋值
		float Joint3_Angle = posNPyrTarget.Yaw_Angle - Joint_World_Angle[1-1] - Joint_World_Angle[2-1];
		
		//Joint3_Angle = -Joint3_Angle;
		static bool Joint3_Angle_Limit = Math_Judge_Threshold(Joint3_Angle, Joint_Limit_Angle[0][2], Joint_Limit_Angle[1][2]);
		if(Joint3_Angle_Limit == true)
		{
			Joint_World_Angle[1-1] = Joint_World_Temp_Angle[1-1];
			Joint_World_Angle[2-1] = Joint_World_Temp_Angle[2-1];
			Joint_World_Angle[3-1] = Joint3_Angle;
			
			Last_Correct_Position_Orientation = posNPyrTarget;
			Last_Joint_World_Angle[1-1] = Joint_World_Angle[1-1];
			Last_Joint_World_Angle[2-1] = Joint_World_Angle[2-1];
		}
	}
	else
	{
		posNPyrTarget = Last_Correct_Position_Orientation;
		return ret = ROBOTARM_ERROR;
	}
	//姿态pitch和roll角度限制
	Math_Constrain(posNPyrTarget.Pitch_Angle,	(Joint_Limit_Angle[0][4]),	(Joint_Limit_Angle[1][4]));
	Math_Constrain(posNPyrTarget.Roll_Angle,	Joint_Limit_Angle[0][3],	Joint_Limit_Angle[1][3]);
	
	Joint_World_Angle[4-1] = posNPyrTarget.Roll_Angle;
	Joint_World_Angle[5-1] = posNPyrTarget.Pitch_Angle * 3.0f;
	Last_Correct_Position_Orientation = posNPyrTarget;
	// 逆解算结果提取
	if (qTargetMutex == ROBOTARM_MUTEX_FREE)
	{
		for (int i=0; i<5; i++)
			jointQTarget[i+1] = Joint_World_Angle[i];	
	}
	return ret = ROBOTARM_OK;
}

ROBOTARM_RETURN_T robotarm_c::Robotarm_CheckforCalibration()
{
	ROBOTARM_RETURN_T ret = ROBOTARM_OK;
	// 执行校准。关节校准没完成返回值为1。取或，任何一个关节没校准完都会变成1（即ROBOTARM_UNCALIBRATED）
	uint8_t caliStatus = 0;
	if (armCalibrated == ROBOTARM_UNCALIBRATED)
	{
//		caliStatus |= priJoint1.jointDoCalibrate(JOINT_CALI_DIRECTION_BACKWARD);
//		caliStatus |= revJoint2.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
//		caliStatus |= revJoint3.jointDoCalibrate(JOINT_CALI_DIRECTION_CW);
//		caliStatus |= revJoint4.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
//		caliStatus |= revJoint5.jointDoCalibrate(JOINT_CALI_DIRECTION_CCW);
//		caliStatus |= revJoint6.jointDoCalibrate(JOINT_CALI_DIRECTION_CW);
		armCalibrated = (ROBOTARM_CALIBRATE_STATUS_T) caliStatus;
	}
	
	return ret;
}

/**
 * @brief 底盘通信任务，当遥控器值更新时才发送
 *

 */
void robotarm_c::Task_Chassis_Communication_PeriodElapsedCallback()
{
	//同步信号量等待
//	extern osSemaphoreId Communication_SemHandle;
//	osSemaphoreWait(Communication_SemHandle,osWaitForever);
	//底盘速度内容填充
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Vx, -1.0f, 1.0f , Chassis_Communication_ID_0x11,0);
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Vy, -1.0f, 1.0f , Chassis_Communication_ID_0x11,2);
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Wz, -1.0f, 1.0f , Chassis_Communication_ID_0x11,4);	
	
	//发送函数
	Chassis_Communication.Task_Process_PeriodElapsedCallback();
}

/**
 * @brief 遥控器控制任务
 *
 */
void robotarm_c::Task_Control_Robotarm()
{
	//角度目标值
//    float tmp_robotarm_x, tmp_robotarm_y,robotarm_yaw;
	//遥控器摇杆值
	memcpy(&Custom_Communication_Data,&Referee.Interaction_Custom_Controller,sizeof(Struct_Custom_Communication_Data));
	static Position_Orientation_t Last_Position_Orientation = posNPyrTarget;
	static Chassis_Move_t Last_Chassis_Move = Chassis_Move;
	
	if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
	{
//		extern osSemaphoreId Communication_SemHandle;
//		osSemaphoreRelease(Communication_SemHandle);
		float dr16_left_x, dr16_left_y,dr16_right_x,dr16_right_y,dr16_yaw;
		// 排除遥控器死区
		dr16_left_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
		dr16_left_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
		dr16_right_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
		dr16_right_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;
		dr16_yaw = (Math_Abs(DR16.Get_Yaw()) > DR16_Dead_Zone) ? DR16.Get_Yaw() : 0;
		switch(DR16.Get_Left_Switch())
		{
			case DR16_Switch_Status_UP:
				//底盘移动
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						
					break;
					case DR16_Switch_Status_MIDDLE:
						posNPyrTarget.Z_Position += dr16_right_y * Robotarm_Z_Resolution;
						Chassis_Move.Chassis_Vx = dr16_left_x * Chassis_X_Resolution;
						Chassis_Move.Chassis_Vy = dr16_left_y * Chassis_Y_Resolution;
						Chassis_Move.Chassis_Wz = dr16_right_x * Chassis_Z_Resolution;
					break;
					case DR16_Switch_Status_DOWN:
						
					break;
				}
				if((Last_Position_Orientation != posNPyrTarget)||(Last_Chassis_Move != Chassis_Move))
				{
					
					Last_Position_Orientation = posNPyrTarget;
					Last_Chassis_Move = Chassis_Move;
				}
				
			break;
			case DR16_Switch_Status_DOWN:
				//机械臂移动
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);
						// 标记回到默认位置
						qTargetMutex = ROBOTARM_MUTEX_OCCUPIED;
						posNPyrTarget = {526.32, -10.288, 127.45};
						Last_Correct_Position_Orientation = {526.32, -10.288, 127.45};
						for (int i=0; i<6; i++)
							jointQTarget[i] = lDefQ[i];
					break;
					case DR16_Switch_Status_MIDDLE:
						qTargetMutex = ROBOTARM_MUTEX_FREE;
						posNPyrTarget.X_Position += dr16_left_y * Robotarm_X_Resolution;
						posNPyrTarget.Y_Position -= dr16_left_x * Robotarm_Y_Resolution;
						
						//posNPyrTarget.Pitch_Angle -= dr16_right_y * Robotarm_Pitch_Resolution;
						posNPyrTarget.Yaw_Angle += dr16_right_x * Robotarm_Yaw_Resolution;
						jointQTarget[0] += dr16_right_y * Robotarm_Height_Resolution;
						if (jointQTarget[0]<priJoint1.GetLowerLimit()) jointQTarget[0] = priJoint1.GetLowerLimit();
						if (jointQTarget[0]>priJoint1.GetUpperLimit()) jointQTarget[0] = priJoint1.GetUpperLimit();
					break;
					case DR16_Switch_Status_DOWN:
						// 标记回到默认位置
						qTargetMutex = ROBOTARM_MUTEX_OCCUPIED;
						posNPyrTarget = {526.32, -10.288, 127.45};
						Last_Correct_Position_Orientation = {526.32, -10.288, 127.45};
						for (int i=0; i<6; i++)
							jointQTarget[i] = rDefQ[i];
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
					break;
				}
				
			break;
			case DR16_Switch_Status_MIDDLE:
				//整车无力
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						
					break;
					case DR16_Switch_Status_MIDDLE:
//						Target_Position_Orientation = {209.314f, 0 ,30.0f};
					break;
					case DR16_Switch_Status_DOWN:
						//Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
					break;
				}
				
			break;
		}
	}
	if(Referee.Get_Referee_Status() == Referee_Status_ENABLE)
	{
		float Controller_x, Controller_y,Controller_pitch,Controller_roll,Controller_yaw;
		
		Controller_x = Math_Int_To_Float(Custom_Communication_Data.Flow_x, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_y = Math_Int_To_Float(Custom_Communication_Data.Flow_y, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_pitch = Math_Int_To_Float(Custom_Communication_Data.pitch, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_roll = Math_Int_To_Float(Custom_Communication_Data.roll, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_yaw = Math_Int_To_Float(Custom_Communication_Data.yaw, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		
		Controller_x = (Math_Abs(Controller_x) > Controller_Dead_Zone) ? Controller_x : 0;
		Controller_y = (Math_Abs(Controller_y) > Controller_Dead_Zone) ? Controller_y : 0;
		Controller_pitch = (Math_Abs(Controller_pitch) > Controller_Dead_Zone) ? Controller_pitch : 0;
		Controller_roll = (Math_Abs(Controller_roll) > Controller_Dead_Zone) ? Controller_roll : 0;
		Controller_yaw = (Math_Abs(Controller_yaw) > Controller_Dead_Zone) ? Controller_yaw : 0;
		
		posNPyrTarget.X_Position -= (float)Controller_x * Robotarm_X_Resolution;
		posNPyrTarget.Y_Position -= (float)Controller_y * Robotarm_Y_Resolution ;
		
		posNPyrTarget.Pitch_Angle -= Controller_pitch * Robotarm_Pitch_Resolution;
		posNPyrTarget.Roll_Angle -= Controller_roll * Robotarm_Roll_Resolution;
		posNPyrTarget.Yaw_Angle -= Controller_yaw * Robotarm_Yaw_Resolution;
	}
//	else
//	{
//		Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
//	}

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
