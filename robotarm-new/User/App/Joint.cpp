#include "Joint.hpp"




uint8_t* ScaraArm_platformMalloc(uint16_t byteSize);
JOINT_RETURN_T ScaraArm_platformFree(void* add);

/*------------------------------*/
/*								*/
/*    下面是平移关节相关的函数     */
/*								*/
/*------------------------------*/


/**
* @brief 初始化关节，适用于C620电调系列电机
 *
 * @param DmMotor
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T prismaticJoint_c::jointInit(Class_DJI_Motor_C620* djiC620Motor)
{
	JOINT_RETURN_T ret;
	motor		= djiC620Motor;
	motorType	= JOINT_MOTOR_C620;
	jointVelocityMaxSi	= 0.2f;
	radToDisplacementSi			= (0.023*PI)/(2.f*PI);
	radPerSecondToVelocitySi	= (0.023*PI)/(2.f*PI);
	jointCaliInit();	// 校准初始化
	return ret;
}

/**
 * @brief 初始化校准程序
 *
 * @param velocity		校准时最大移动速度
 * @param acceleration	最大加速度
 * @param afLength		均值滤波长度
 * @param tole			均值滤波速度容忍值
 * @param __caliDir		校准平移方向
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T prismaticJoint_c::jointCaliInit(	float velocity,
												float acceleration,
												uint8_t afLength,
												float tole,
												JOINT_PRISMATIC_CALI_DIRECTION_T __caliDir)
{
	JOINT_RETURN_T ret = 	JOINT_OK;
	caliVelocitySi				= velocity;
	caliDeltaDisplacementSi		= acceleration;
	averageFilterLength			= afLength;
	caliVelocityDiffTolerance	= tole;
	caliDir						= __caliDir;
	filterInput = (float*) ScaraArm_platformMalloc(afLength*sizeof(float));
	if (filterInput == NULL)	// 看看有没有申请到空间
		return ret = JOINT_ERROR;	// 没申请到，返回错误
	for (int i=0; i<afLength; i++)
		filterInput[i] = 1000.f;	// 塞进去很大的数，让初始的偏差很大
	return ret;
}

JOINT_RETURN_T prismaticJoint_c::jointSetMechLimit(float lowerLim, float upperLim, float margin)
{
	JOINT_RETURN_T ret;
	lowerMechLimSi		= lowerLim;
	upperMechLimSi		= upperLim;
	mechLimitMarginSi	= margin;
	return ret;
}



JOINT_RETURN_T prismaticJoint_c::jointDoCalibrate(JOINT_PRISMATIC_CALI_DIRECTION_T __caliDir)
{
	JOINT_RETURN_T ret;
	caliDir = __caliDir;
	// 没有完成校准，才需要进行校准
	if (caliStatus == JOINT_UNCALIBRATED)
	{
		// 根据电机类型，进行校准
		switch(motorType)
		{
			case JOINT_MOTOR_DM:
				break;
			case JOINT_MOTOR_AK:
				break;
			case JOINT_MOTOR_C620:
				ret = jointCalibrate((Class_DJI_Motor_C620*) motor);
				break;
			case JOINT_MOTOR_C610:
				break;
			default:
				return ret = JOINT_ERROR;
		}
	}
	else
	{
		// 根据校准方向，停在对应位置
		switch((uint8_t) caliDir)
		{
			case JOINT_CALI_DIRECTION_BACKWARD:
				setBodyFrameJointDisplacement(-GetLowerLimit());
				break;
			case JOINT_CALI_DIRECTION_FORWARD:
				setBodyFrameJointDisplacement(GetUpperLimit());
				break;
			default:
				ret = JOINT_ERROR;
				break;
		}
		ret = JOINT_ERROR;
	}
	return ret;
}


/**
* @brief 用于校准平移关节初始位置，适用于C620配套电机
 *
 * @param AkMotor
 * @param caliDir
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T prismaticJoint_c::jointCalibrate(Class_DJI_Motor_C620* djiC620Motor)
{
	JOINT_RETURN_T ret = JOINT_ERROR;
	// 设定位速环始终在运动
	float targetPosition;
	float currentDis = djiC620Motor->Get_Now_Angle()*radToDisplacementSi;		// 获取当前位移值 m
	float currentVel = djiC620Motor->Get_Now_Omega()*radPerSecondToVelocitySi;	// 获取当前速度值 m/s			
	if (currentDis==0)	// 浮点数角度理论上几乎没可能是0，是0只会是电机断联
	{
		targetPosition	= 0;
		return JOINT_ERROR;
	}
	// 转矩很大，可能堵转了，需要减小变化角度
	float slowerCoeff = 1.f;
	if (fabs(djiC620Motor->Get_Now_Torque()) > 2.f)
		slowerCoeff = 5.f;
	// 根据校准角度选择策略
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			targetPosition	= currentDis + caliDeltaDisplacementSi/slowerCoeff;
			djiC620Motor->Set_Target_Angle(targetPosition/radToDisplacementSi);
			break;
		case JOINT_CALI_DIRECTION_CW:
			targetPosition	= currentDis - caliDeltaDisplacementSi/slowerCoeff;
			djiC620Motor->Set_Target_Angle(targetPosition/radToDisplacementSi);
			break;
		default:
			return ret;
	}
	djiC620Motor->Task_PID_PeriodElapsedCallback();	// 进行一次PID运算
	CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);	// 进行一次 CAN 发送
	// 检测堵转，堵转就说明到了机械限位
	float caliDiff = fabs(averageFilter(currentVel));	// 最近速度的均值滤波
	if (caliDiff < caliVelocityDiffTolerance)			// 最近速度都很小，说明卡到限位了
	{
		caliStatus	= JOINT_CALIBRATED;		// 标记已校准
		calibratedDisplacementSi = currentDis;	// 记录下当前位移，作为校正完成的结果
		return JOINT_OK;
	}
	return ret;
}


/**
 * @brief 获得节点坐标系下的电机角度
 *
 * @return float
 */
float prismaticJoint_c::getBodyFrameJointDisplacement()
{
	float ret;
	float motorRad;
	// 根据电机类型，读取当前角度
	switch(motorType)
	{
		case JOINT_MOTOR_DM:
			break;
		case JOINT_MOTOR_AK:
			break;
		case JOINT_MOTOR_C620:
			ret = ((Class_DJI_Motor_C620*)motor)->Get_Now_Angle()*radToDisplacementSi;
			break;
		case JOINT_MOTOR_C610:
			break;
		default:
			return ret = 0;
	}
	// 计算相对限位角度
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_BACKWARD:
			ret = ret - calibratedDisplacementSi + lowerMechLimSi;
			break;
		case JOINT_CALI_DIRECTION_FORWARD:
			ret = ret - calibratedDisplacementSi + upperMechLimSi;
			break;
		default:
			return ret = 0;
	}
	return ret;
}

/**
 * @brief 设置节点坐标系下的电机角度
 *
 * @param targetRad
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T prismaticJoint_c::setBodyFrameJointDisplacement(float targetDisp)
{
	JOINT_RETURN_T ret;
	// 限制目标角度
	if (targetDisp > upperMechLimSi)
		targetDisp = upperMechLimSi - mechLimitMarginSi;
	if (targetDisp < -lowerMechLimSi)
		targetDisp = -lowerMechLimSi + mechLimitMarginSi;
	// 计算相对限位角度
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_FORWARD:
			targetDisp = calibratedDisplacementSi - (upperMechLimSi - targetDisp);
			break;
		case JOINT_CALI_DIRECTION_BACKWARD:
			targetDisp = calibratedDisplacementSi + targetDisp + lowerMechLimSi;
			break;
		default:
			return ret = JOINT_ERROR;
	}

	switch(motorType)
	{
		case JOINT_MOTOR_DM:
			return ret;
			break;
		case JOINT_MOTOR_AK:
			break;
		case JOINT_MOTOR_C620:
			((Class_DJI_Motor_C620*)motor)->Set_Target_Angle(targetDisp/radToDisplacementSi);
			((Class_DJI_Motor_C620*)motor)->Task_PID_PeriodElapsedCallback();	// 进行一次 PID 运算
			#if defined(FREERTOS_ENABLE)
			{
				extern osMutexId can1TransMutexHandle;
				osSemaphoreWait( can1TransMutexHandle, portMAX_DELAY );// 加锁
				{
					CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);	// 进行一次 CAN 发送
				}
				osSemaphoreRelease( can1TransMutexHandle );// 解锁
			}
			#endif
					return ret = JOINT_OK;
			break;
		case JOINT_MOTOR_C610:
			break;
		default:
			return ret = JOINT_ERROR;
	}
	return ret;
}

JOINT_CALIBRATED_STATUS_T prismaticJoint_c::jointGetCaliStatus()
{
	return caliStatus;
}

float prismaticJoint_c::GetUpperLimit()
{
	return upperMechLimSi - mechLimitMarginSi;
}

float prismaticJoint_c::GetLowerLimit()
{
	return lowerMechLimSi + mechLimitMarginSi;
}

/**
 * @brief 均值滤波，在校准中用到了
 *
 * @param input
 * @return float
 */
float prismaticJoint_c::averageFilter(float input)
{
	float sum = 0;
	for (int i=0; i<averageFilterLength-1; i++)
	{
		filterInput[i]=filterInput[i+1];
		sum = sum + filterInput[i];
	}
	filterInput[averageFilterLength-1] = input;
	sum = sum + filterInput[averageFilterLength-1];
	return(sum/averageFilterLength);
}

/*------------------------------*/
/*								*/
/*    下面是转动关节相关的函数     */
/*								*/
/*------------------------------*/


/**
* @brief 初始化转动关节，适用于达妙电机
 *
 * @param DmMotor
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointInit(DM_motor_t* DmMotor)
{
	JOINT_RETURN_T ret;
	motor		= DmMotor;
	motorType	= JOINT_MOTOR_DM;
	jointOmegaMax	= 0.05f;
	jointDOmegaMax	= 1.f;
	jointCaliInit();	// 校准初始化
	return ret;
}

/**
* @brief 初始化关节，适用于AK电机
 *
 * @param AkMotor
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointInit(AK_motor_t* AkMotor)
{
	JOINT_RETURN_T ret;
	motor		= AkMotor;
	motorType	= JOINT_MOTOR_AK;
	jointOmegaMax	= 0.15f;
	jointDOmegaMax	= 0.03f;
	jointCaliInit();	// 校准初始化
	return ret;
}

/**
* @brief 初始化关节，适用于C620电调配套电机
 *
 * @param AkMotor
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointInit(Class_DJI_Motor_C620* djiC620Motor)
{
	JOINT_RETURN_T ret;
	motor		= djiC620Motor;
	motorType	= JOINT_MOTOR_C620;
	jointOmegaMax	= 0.2f;
	jointDOmegaMax	= 0.1f;
	jointCaliInit();	// 校准初始化
	return ret;
}

/**
* @brief 初始化关节，适用于C610电调配套电机
 *
 * @param Class_DJI_Motor_C610
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointInit(Class_DJI_Motor_C610* djiC610Motor)
{
	JOINT_RETURN_T ret;
	motor		= djiC610Motor;
	motorType	= JOINT_MOTOR_C610;
	jointOmegaMax	= 0.2f;
	jointDOmegaMax	= 0.1f;
	reductionRate	= -3.f;	// 设置传动比
	jointCaliInit();	// 校准初始化
	return ret;
}


/**
 * @brief 初始化校准程序
 *
 * @param omege 校准时最大转动角速度
 * @param delPos 每次转动的步长
 * @param afLength 均值滤波长度
 * @param tole 均值滤波角速度容忍值
 * @param __caliDir 校准转动方向
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointCaliInit(float omege, float delPos, uint8_t afLength, float tole, JOINT_REVOLUTE_CALIBRATION_DIRECTION_T __caliDir)
{
	JOINT_RETURN_T ret = JOINT_OK;
	caliOmegaRadPerSec		= omege;
	caliDeltaPosRad			= delPos;
	averageFilterLength		= afLength;
	caliOmeDiffTolerance	= tole;
	caliDir					= __caliDir;
	filterInput = (float*) ScaraArm_platformMalloc(afLength*sizeof(float));
	if (filterInput == NULL)	// 看看有没有申请到空间
		return ret = JOINT_ERROR;	// 没申请到，返回错误
	for (int i=0; i<afLength; i++)
		filterInput[i] = 1000.f;	// 塞进去很大的数，让初始的偏差很大
	return ret;
}


JOINT_RETURN_T revoluteJoint_c::jointDoCalibrate(JOINT_REVOLUTE_CALIBRATION_DIRECTION_T __caliDir)
{
	JOINT_RETURN_T ret;
	caliDir = __caliDir;
	// 没有完成校准，才需要进行校准
	if (caliStatus == JOINT_UNCALIBRATED)
	{
		// 根据电机类型，进行校准
		switch(motorType)
		{
			case JOINT_MOTOR_DM:
				ret = jointCalibrate((DM_motor_t*) motor);
				break;
			case JOINT_MOTOR_AK:
				ret = jointCalibrate((AK_motor_t*) motor);
				break;
			case JOINT_MOTOR_C620:
				ret = jointCalibrate((Class_DJI_Motor_C620*)motor);
				break;
			case JOINT_MOTOR_C610:
				ret = jointCalibrate((Class_DJI_Motor_C610*)motor);
				break;
			default:
				return ret = JOINT_ERROR;
		}
	}
	else
	{
		// 根据校准方向，停在对应位置
		switch((uint8_t) caliDir)
		{
			case JOINT_CALI_DIRECTION_CW:
				setBodyFrameJointAngle(-GetCwLimit());
				break;
			case JOINT_CALI_DIRECTION_CCW:
				setBodyFrameJointAngle(GetCcwLimit());
				break;
			default:
				ret = JOINT_ERROR;
				break;
		}
		ret = JOINT_ERROR;
	}
	return ret;
}



/**
 * @brief 获得节点坐标系下的电机角度
 *
 * @return float
 */
float revoluteJoint_c::getBodyFrameJointAngle()
{
	float ret;
	// 根据电机类型，读取当前角度
	switch(motorType)
	{
		case JOINT_MOTOR_DM:
			ret = DmMotor_GetOutputPositionRad((DM_motor_t*) motor);
			break;
		case JOINT_MOTOR_AK:
			ret = AkMotor_GetOutputPositionRad((AK_motor_t*) motor);
			break;
		case JOINT_MOTOR_C620:
			ret = ((Class_DJI_Motor_C620*)motor)->Get_Now_Angle();
			break;
		case JOINT_MOTOR_C610:
			ret = ((Class_DJI_Motor_C610*)motor)->Get_Now_Angle();
			break;
		default:
			return ret = 0;
	}
	// 传动比处理
	ret = ret / reductionRate;
	// 计算相对限位角度
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			ret = ret - calibratedPositionRad + ccwMechLimitRad;
			break;
		case JOINT_CALI_DIRECTION_CW:
			ret = ret - calibratedPositionRad - cwMechLimitRad;
			break;
		default:
			return ret = 0;
	}
	return ret;
}

/**
 * @brief 设置节点坐标系下的电机角度
 *
 * @param targetRad
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::setBodyFrameJointAngle(float targetRad)
{
	JOINT_RETURN_T ret;
	// 限制目标角度
	if (targetRad > ccwMechLimitRad)
		targetRad = ccwMechLimitRad - mechLimitMarginRad;
	if (targetRad < -cwMechLimitRad)
		targetRad = -cwMechLimitRad + mechLimitMarginRad;
	// 计算相对限位角度
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			targetRad = calibratedPositionRad - (ccwMechLimitRad - targetRad)*reductionRate;
			break;
		case JOINT_CALI_DIRECTION_CW:
			targetRad = calibratedPositionRad + (targetRad + cwMechLimitRad)*reductionRate;
			break;
		default:
			return ret = JOINT_ERROR;
			break;
	}

	DM_motor_posVelModeCommand_t dmCommand;
	switch(motorType)
	{
		case JOINT_MOTOR_DM:
			dmCommand.targetPosition	= targetRad;
			dmCommand.targetVelocity	= PI/2.f;	// 随便设置一个最大速度
//			ret = (JOINT_RETURN_T) DM_motor_setDisable((DM_motor_t*) motor);	// 测试用的
			ret = (JOINT_RETURN_T) DM_motor_setPosVelControl((DM_motor_t*) motor, dmCommand);
			return ret;
			break;
		case JOINT_MOTOR_AK:
			
			AK_motor_SetMultiPositionSpeedAcceleration_SI(	(AK_motor_t*) motor,
															targetRad*RAD_TO_DEGREE,
															jointOmegaMax*RAD_TO_DEGREE,
															jointDOmegaMax*RAD_TO_DEGREE);
			return ret;
			break;
		case JOINT_MOTOR_C620:
			((Class_DJI_Motor_C620*)motor)->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			((Class_DJI_Motor_C620*)motor)->Set_Target_Angle(targetRad);
			((Class_DJI_Motor_C620*)motor)->Task_PID_PeriodElapsedCallback();	// 进行一次 PID 运算
			CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);	// 进行一次 CAN 发送
			return ret;
			break;
		case JOINT_MOTOR_C610:
			((Class_DJI_Motor_C610*)motor)->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			((Class_DJI_Motor_C610*)motor)->Set_Target_Angle(targetRad);	// 有传动比，要反向
			((Class_DJI_Motor_C610*)motor)->Task_PID_PeriodElapsedCallback();	// 进行一次 PID 运算
			CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);	// 进行一次 CAN 发送
			return ret;
			break;
		default:
			return ret = JOINT_ERROR;
	}
	return ret;
}


/**
 * @brief 用于校准关节位置，适用于达妙电机
 *
 * @param DmMotor 电机句柄
 * @param caliDir 校准方向（顺逆时针）
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointCalibrate(DM_motor_t* DmMotor)
{
	JOINT_RETURN_T ret = JOINT_ERROR;
	// 设定位速环始终在运动
	DM_motor_posVelModeCommand_t caliCommand;
	float currentPos = DmMotor_GetOutputPositionRad(DmMotor);	// 获取当前角度
	float currentOme = DmMotor_GetPutputOmegaRadPerSecond(DmMotor);
	if (currentPos==0)	// 浮点数角度理论上几乎没可能是0，是0只会是电机断联
	{
		caliCommand.targetPosition	= 0;
		caliCommand.targetVelocity	= 0;
		DM_motor_setPosVelControl(DmMotor, caliCommand);	// 达妙电机需要随便发点东西过去，保证能够收到数据
		return JOINT_ERROR;
	}
	// 转矩很大，可能堵转了，需要减小变化角度
	float slowerCoeff = 1.f;
	if (fabs(DmMotor_GetTorqueSi(DmMotor)) > 1.f)
		slowerCoeff = 7.f;
	// 根据校准角度选择策略
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			caliCommand.targetPosition	= currentPos + caliDeltaPosRad/slowerCoeff;
			caliCommand.targetVelocity	= caliOmegaRadPerSec;
			DM_motor_setPosVelControl(DmMotor, caliCommand);
			break;
		case JOINT_CALI_DIRECTION_CW:
			caliCommand.targetPosition	= currentPos - caliDeltaPosRad/slowerCoeff;
			caliCommand.targetVelocity	= -caliOmegaRadPerSec;
			DM_motor_setPosVelControl(DmMotor, caliCommand);
			break;
		default:
			return ret;
	}
	// 检测堵转，堵转就说明到了机械限位
	float caliDiff = fabs(averageFilter(currentOme));	// 最近角速度的均值滤波
	if (caliDiff < caliOmeDiffTolerance) // 最近角速度都很小，说明卡到限位了
	{
		caliStatus	= JOINT_CALIBRATED;		// 标记已校准
		calibratedPositionRad = currentPos;	// 记录下当前角度，作为校正完成的结果
		return JOINT_OK;
	}
	return ret;
	
//	// 输出绝对值编码器，直接写入角度，转到预定角度就返回校准完毕
//	JOINT_RETURN_T ret = JOINT_ERROR;
//	calibratedPositionRad = -0.49;
//	float currentPos = DmMotor_GetOutputPositionRad(DmMotor);	// 获取当前角度
//	if (fabs(currentPos - calibratedPositionRad)<caliOmeDiffTolerance)
//		ret = JOINT_OK;
//	DM_motor_posVelModeCommand_t caliCommand;
//	caliCommand.targetPosition	= calibratedPositionRad;
//	caliCommand.targetVelocity	= caliOmegaRadPerSec;
//	DM_motor_setPosVelControl(DmMotor, caliCommand);
//	return ret;
}

/**
 * @brief 用于校准关节位置，适用于AK电机
 *
 * @param AkMotor
 * @param caliDir
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointCalibrate(AK_motor_t* AkMotor)
{
	JOINT_RETURN_T ret = JOINT_ERROR;
	float targetPosition, targetVelocity, targetAcc=0.6f;	// 加速度是随便取的
	// 设定位速环始终在运动
	float currentPos = AkMotor_GetOutputPositionRad(AkMotor);		// 获取当前角度
	float currentOme = AkMotor_GetPutputOmegaRadPerSecond(AkMotor);	// 获取当前角速度
	if (currentPos==0)	// 浮点数角度理论上几乎没可能是0，是0只会是电机断联
		return JOINT_ERROR;
	// 转矩很大，可能堵转了，需要减小变化角度
	float slowerCoeff = 1.f;
	if (fabs(AkMotor_GetOutputTorqueSi(AkMotor)) > 4.f)
		slowerCoeff = 5.f;
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			targetPosition	= (currentPos + caliDeltaPosRad/slowerCoeff)*RAD_TO_DEGREE;
			targetVelocity	= caliOmegaRadPerSec/2.f/PI*60.f;
			AK_motor_SetMultiPositionSpeedAcceleration_SI(AkMotor, targetPosition, targetVelocity, targetAcc);
			break;
		case JOINT_CALI_DIRECTION_CW:
			targetPosition	= (currentPos - caliDeltaPosRad/slowerCoeff)*RAD_TO_DEGREE;
			targetVelocity	= -caliOmegaRadPerSec/2.f/PI*60.f;
			AK_motor_SetMultiPositionSpeedAcceleration_SI(AkMotor, targetPosition, targetVelocity, targetAcc);
			break;
		default:
			return ret;
	}
	// 检测堵转，堵转就说明到了机械限位
	float caliDiff = fabs(averageFilter(currentOme));	// 最近角速度的均值滤波
	if (caliDiff < caliOmeDiffTolerance) // 最近角速度都很小，说明卡到限位了
	{
		caliStatus	= JOINT_CALIBRATED;		// 标记已校准
		calibratedPositionRad = currentPos;	// 记录下当前角度，作为校正完成的结果
		return JOINT_OK;
	}
	return ret;
}

/**
* @brief 用于校准关节位置，适用于 C620 电调配套电机
 *
 * @param djiC620Motor 电机句柄
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointCalibrate(Class_DJI_Motor_C620* djiC620Motor)
{
	JOINT_RETURN_T ret = JOINT_ERROR;
	// 设定位速环始终在运动
	float targetPosition;
	float currentPos = djiC620Motor->Get_Now_Angle();	// 获取当前角度 Rad
	float currentOme = djiC620Motor->Get_Now_Omega();
	if (currentPos==0)	// 浮点数角度理论上几乎没可能是0，是0只会是电机断联
	{
		targetPosition	= 0;
		return JOINT_ERROR;
	}
	// 转矩很大，可能堵转了，需要减小变化角度
	float slowerCoeff = 1.f;
	if (fabs(djiC620Motor->Get_Now_Torque()) > 2.f)
		slowerCoeff = 5.f;
	// 根据校准角度选择策略
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			djiC620Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
			djiC620Motor->Set_Target_Omega(caliOmegaRadPerSec);
			break;
		case JOINT_CALI_DIRECTION_CW:
			djiC620Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
			djiC620Motor->Set_Target_Omega(-caliOmegaRadPerSec);
			break;
		default:
			return ret;
	}
	djiC620Motor->Task_PID_PeriodElapsedCallback();	// 进行一次PID运算
	CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);	// 进行一次 CAN 发送
	// 检测堵转，堵转就说明到了机械限位
	float caliDiff = fabs(averageFilter(currentOme));	// 最近角速度的均值滤波
	if (caliDiff < caliOmeDiffTolerance) // 最近角速度都很小，说明卡到限位了
	{
		caliStatus	= JOINT_CALIBRATED;		// 标记已校准
		calibratedPositionRad = currentPos;	// 记录下当前角度，作为校正完成的结果
		return JOINT_OK;
	}
	return ret;
}


/**
* @brief 用于校准关节位置，适用于 C610 电调配套电机
 *
 * @param djiC620Motor 电机句柄
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T revoluteJoint_c::jointCalibrate(Class_DJI_Motor_C610* djiC610Motor)
{
	JOINT_RETURN_T ret = JOINT_ERROR;
	// 设定位速环始终在运动
	float targetPosition;
	float currentPos = djiC610Motor->Get_Now_Angle();	// 获取当前角度 Rad
	float currentOme = djiC610Motor->Get_Now_Omega();
	if (currentPos==0)	// 浮点数角度理论上几乎没可能是0，是0只会是电机断联
	{
		targetPosition	= 0;
		return JOINT_ERROR;
	}
	// 转矩很大，可能堵转了，需要减小变化角度
	float slowerCoeff = 1.f;
	if (fabs(djiC610Motor->Get_Now_Torque()) > 2.f)
		slowerCoeff = 5.f;
	// 根据校准角度选择策略
	switch(caliDir)
	{
		case JOINT_CALI_DIRECTION_CCW:
			djiC610Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
			djiC610Motor->Set_Target_Omega(reductionRate*caliOmegaRadPerSec);	// 有传动反向
			break;
		case JOINT_CALI_DIRECTION_CW:
			djiC610Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
			djiC610Motor->Set_Target_Omega(-reductionRate*caliOmegaRadPerSec);		// 有传动反向
			break;
		default:
			return ret;
	}
	djiC610Motor->Task_PID_PeriodElapsedCallback();	// 进行一次PID运算
	CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);	// 进行一次 CAN 发送
	// 检测堵转，堵转就说明到了机械限位
	float caliDiff = fabs(averageFilter(currentOme));	// 最近角速度的均值滤波
	if (caliDiff < caliOmeDiffTolerance) // 最近角速度都很小，说明卡到限位了
	{
		caliStatus	= JOINT_CALIBRATED;		// 标记已校准
		calibratedPositionRad = currentPos;	// 记录下当前角度，作为校正完成的结果
		return JOINT_OK;
	}
	return ret;
}

JOINT_CALIBRATED_STATUS_T revoluteJoint_c::jointGetCaliStatus()
{
	return caliStatus;
}

JOINT_MOTOR_CONNECTION_STATE_T revoluteJoint_c::jointGetConnectionState()
{
	return conectionState;
}

JOINT_RETURN_T revoluteJoint_c::jointConnectionStateUpdate()
{
	static uint32_t lastCnt;
	uint32_t nowCnt;
	switch(motorType)
	{
		case JOINT_MOTOR_DM:
			nowCnt = DmMotor_GetResolvedCnt((DM_motor_t*) motor);
			break;
		case JOINT_MOTOR_AK:
			nowCnt = AkMotor_GetResolvedCnt((AK_motor_t*) motor);
			break;
		default:
			return JOINT_ERROR;
			break;
	}
	
	if (nowCnt > lastCnt)
		conectionState = JOINT_MOTOR_ONLINE;
	else
		conectionState = JOINT_MOTOR_OFFLINE;
	return JOINT_OK;
}

JOINT_RETURN_T revoluteJoint_c::jointSetMechLimit(float cwLim, float ccwLim, float margin)
{
	JOINT_RETURN_T ret;
	cwMechLimitRad	= cwLim;
	ccwMechLimitRad	= ccwLim;
	mechLimitMarginRad	= margin;
	return ret;
}

float revoluteJoint_c::GetCwMechLimit()
{
	return cwMechLimitRad;
}

float revoluteJoint_c::GetCcwMechLimit()
{
	return ccwMechLimitRad;
}

float revoluteJoint_c::GetCwLimit()
{
	return cwMechLimitRad + mechLimitMarginRad;
}

float revoluteJoint_c::GetCcwLimit()
{
	return ccwMechLimitRad - mechLimitMarginRad;
}
/**
 * @brief 均值滤波，在校准中用到了
 *
 * @param input
 * @return float
 */
float revoluteJoint_c::averageFilter(float input)
{
	float sum = 0;
	for (int i=0; i<averageFilterLength-1; i++)
	{
		filterInput[i]=filterInput[i+1];
		sum = sum + filterInput[i];
	}
	filterInput[averageFilterLength-1] = input;
	sum = sum + filterInput[averageFilterLength-1];
	return(sum/averageFilterLength);
}

/**
 * @brief Malloc函数自定义
 *
 * @param byteSize
 * @return uint8_t*
 */
uint8_t* ScaraArm_platformMalloc(uint16_t byteSize)
{
	#if defined(INC_FREERTOS_H)
		return (uint8_t*)pvPortMalloc(byteSize);
	#else
		return (uint8_t*)malloc(byteSize);
	#endif
}

/**
 * @brief Free函数自定义
 *
 * @param add
 * @return JOINT_RETURN_T
 */
JOINT_RETURN_T ScaraArm_platformFree(void* add)
{
	#if defined(INC_FREERTOS_H)
		vPortFree(add);
		return JOINT_OK;
	#else
		free(add);
		add = NULL;
		return NATURAL_INTERP_OK;
	#endif
}
