/* Includes ------------------------------------------------------------------*/
#include "DM_motor.h"

#if defined(STM32F105) | (STM32F407)
	#include "can.h"
#endif



/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
    CAN_TxHeaderTypeDef DM_motor_CAN_TxHeaderStruct;
    uint32_t  DM_motor_pTxMailbox;
#endif


uint8_t DM_motor_platformTrans(void *peripheral_handle, uint32_t CANID, uint8_t data[]);


/* Motor Handle Create -------------------------------------------------------*/
DM_motor_t dmMotor_joint4;



/**
 * @brief 用于处置非严重错误的函数，以此保证电机控制正常使用
 *
 * @param motor 达妙电机的句柄
 * @return DM_MOTOR_RETURN_T 0为正常
 */
DM_MOTOR_RETURN_T DM_motor_mildStatecodeHandle(DM_motor_t *motor)
{
	uint32_t ret;
	switch (motor->status.statecode)
	{
		case DM_MOTOR_STATECODE_ENABLE:				// 正常状态，返回 OK
			ret = DM_MOTOR_OK;
			break;
		case DM_MOTOR_STATECODE_DISABLE:			// 未使能，重新使能
			ret = DM_motor_setEnable(motor);
			//HAL_Delay(1);
			break;
		case DM_MOTOR_STATECODE_LOST_CONNECTION:	// 长时间未通讯掉线，清除连接错误
			ret = DM_motor_clearError(motor);
			//HAL_Delay(1);
			break;
		case DM_MOTOR_STATECODE_COIL_OVERHEAT:
			if (motor->status.rotorTempretureCelsiusDegree < 60.f)
			ret = DM_motor_clearError(motor);
			break;
		case DM_MOTOR_STATECODE_MOS_OVERHEAT:
			if (motor->status.mosTempretureCelsiusDegree < 60.f)
			ret = DM_motor_clearError(motor);
			break;
				
		default:									// 严重错误，返回Err
			ret = DM_MOTOR_ERROR;
			break;
	}
	return ret;
}

/**
 * @brief 基于 MIT 控制模式，设置电机力矩的函数。
 *
 * @param motor 达妙电机的句柄
 * @param torque_nm 设置电机的力矩值，单位是 牛*米
 * @return DM_MOTOR_RETURN_T
 */
DM_MOTOR_RETURN_T DM_motor_setTorqueSI(DM_motor_t *motor, float torque_nm)
{
	uint32_t ret;
	if (motor->parameter.operationMode != DM_MOTOR_CONTROL_MIT_MODE) // 非 MIT 模式无法力矩控制
		return DM_MOTOR_ERROR;
	if (DM_motor_mildStatecodeHandle(motor) == DM_MOTOR_OK) // 检查电机状态，正常才能发送
	{
		memset(&motor->command.mitModeCommand, 0, sizeof(motor->command.mitModeCommand)); // 其他指令清零，实现力矩控制
		if (torque_nm > 0) 
			motor->command.mitModeCommand.targetTorque	= torque_nm + motor->parameter.compensationTorque;
		else if (torque_nm < 0)
			motor->command.mitModeCommand.targetTorque	= torque_nm - motor->parameter.compensationTorque;
		else motor->command.mitModeCommand.targetTorque = 0;
		return ret = DM_motor_dataFrameTransmit(&motor->parameter.ctx, motor);
	}
	else return DM_MOTOR_ERROR;
}

/**
 * @brief 基于 MIT 控制模式，实现 MIT 协议控制电机
 *
 * @param motor 达妙电机的句柄
 * @param command 符合 MIT 协议的控制值
 * @return DM_MOTOR_RETURN_T
 */
DM_MOTOR_RETURN_T DM_motor_setMitControl(DM_motor_t *motor, DM_motor_mitModeCommand_t command)
{
	uint32_t ret;
	if (motor->parameter.operationMode != DM_MOTOR_CONTROL_MIT_MODE) // 非 MIT 模式，不合法
		return DM_MOTOR_ERROR;
	if (DM_motor_mildStatecodeHandle(motor) == DM_MOTOR_OK) // 检查电机状态，正常才能发送
	{
		memcpy(&motor->command.mitModeCommand, &command, sizeof(command));
		return ret = DM_motor_dataFrameTransmit(&motor->parameter.ctx, motor);
	}
	else return DM_MOTOR_ERROR;
}

/**
 * @brief 基于位置速度环控制模式，实现位置速度环控制电机
 *
 * @param motor 达妙电机的句柄
 * @param command 符合位置速度环控制模式的控制值
 * @return DM_MOTOR_RETURN_T
 */
DM_MOTOR_RETURN_T DM_motor_setPosVelControl(DM_motor_t *motor, DM_motor_posVelModeCommand_t command)
{
	uint32_t ret;
	if (motor->parameter.operationMode != DM_MOTOR_CONTROL_POS_VEL_MODE) // 非位置速度环模式，不合法
		return DM_MOTOR_ERROR;
	if (DM_motor_mildStatecodeHandle(motor) == DM_MOTOR_OK) // 检查电机状态，正常才能发送
	{
		memcpy(&motor->command.posVelModeCommand, &command, sizeof(command));
		return ret = DM_motor_dataFrameTransmit(&motor->parameter.ctx, motor);
	}
	else return DM_MOTOR_ERROR;
}

/**
 * @brief 基于速度环控制模式，实现速度环控制电机
 *
 * @param motor 达妙电机的句柄
 * @param command 符合速度环控制模式的控制值
 * @return DM_MOTOR_RETURN_T
 */
DM_MOTOR_RETURN_T DM_motor_setVelControl(DM_motor_t *motor, DM_motor_velModeCommand_t command)
{
	uint32_t ret;
	if (motor->parameter.operationMode != DM_MOTOR_CONTROL_VEL_MODE) // 非速度环模式，不合法
		return DM_MOTOR_ERROR;
	if (DM_motor_mildStatecodeHandle(motor) == DM_MOTOR_OK) // 检查电机状态，正常才能发送
	{
		memcpy(&motor->command.velModeCommand, &command, sizeof(command));
		return ret = DM_motor_dataFrameTransmit(&motor->parameter.ctx, motor);
	}
	else return DM_MOTOR_ERROR;
}

DM_MOTOR_RETURN_T DM_motor_setZeroPosition(DM_motor_t *motor)
{
	uint32_t ret;
	if (DM_motor_mildStatecodeHandle(motor) == DM_MOTOR_OK) // 检查电机状态，正常才能发送
	{
		motor->command.mode	= DM_MOTOR_SET_ZERO_POSITION;
		return ret = DM_motor_modeFrameTransmit(&motor->parameter.ctx, motor);
	}
	else return DM_MOTOR_ERROR;
}

DM_MOTOR_RETURN_T DM_motor_clearError(DM_motor_t *motor)
{
	uint32_t ret;
	motor->command.mode = DM_MOTOR_CLEAR_ERROR;
	ret = DM_motor_modeFrameTransmit(&motor->parameter.ctx, motor);
	return ret;
}

DM_MOTOR_RETURN_T DM_motor_setEnable(DM_motor_t *motor)
{
	uint32_t ret;
	motor->command.mode = DM_MOTOR_SET_MODE_ENABLE;
	ret = DM_motor_modeFrameTransmit(&motor->parameter.ctx, motor);
	return ret;
}

DM_MOTOR_RETURN_T DM_motor_setDisable(DM_motor_t *motor)
{
	uint32_t ret;
	motor->command.mode = DM_MOTOR_SET_MODE_DISABLE;
	ret = DM_motor_modeFrameTransmit(&motor->parameter.ctx, motor);
	return ret;
}

DM_motor_handleList_unit_t DM_motor_handleList[MAXIMUM_DM_MOTOR_HANDLE_NUM];
DM_MOTOR_RETURN_T DM_motor_handleListAdd(DM_motor_t *motor)
{
	for (int i=0; i<MAXIMUM_DM_MOTOR_HANDLE_NUM; i++) // 从表头开始找有没有空位
		if (DM_motor_handleList[i].motor_handle == 0)
		{
			DM_motor_handleList[i].motor_handle			= motor;
			DM_motor_handleList[i].CANID				= motor->parameter.deviceCanId;
			DM_motor_handleList[i].peripheral_handle	= motor->parameter.peripheral_handle;
			return DM_MOTOR_OK;
		}
	return DM_MOTOR_ERROR;
}
/**
 * @brief 在AK电机列表中确认是否有某个AK电机的句柄，确保句柄合法性
 * @param motor 电机的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
DM_MOTOR_RETURN_T DM_motor_CheckHandleLegitimacy(DM_motor_t *motor)
{
	for (int i=0; i<MAXIMUM_DM_MOTOR_HANDLE_NUM; i++) // 从表头开始找
		if (DM_motor_handleList[i].motor_handle == motor)
			return DM_MOTOR_OK;
		return DM_MOTOR_ILLEGAL_HANDLE; // 句柄找不到，不合法
}
/**
 * @brief 通过CANID和外设在AK电机列表中查询电机的句柄
 * @param motor 电机的句柄
 * @return 如果成功返回电机的句柄，否则返回空句柄
 */
DM_motor_t* DM_motor_FindSteeringHandle_via_CANIDandPeripheral(uint8_t CANID, void *peripheral_handle)
{
	for (int i=0; i<MAXIMUM_DM_MOTOR_HANDLE_NUM; i++)
		if (DM_motor_handleList[i].CANID == CANID && DM_motor_handleList[i].peripheral_handle == peripheral_handle) // 在记录的同一个外设，以及同一个CANID
			return DM_motor_handleList[i].motor_handle;
	return DM_MOTOR_NULL_HANDLE;
}


void DM_motor_handleInit(DM_motor_t *motor, DM_motor_parameter_t param)
{
	//motor->status.statecode	= DM_MOTOR_STATECODE_LOST_CONNECTION; // 默认上电电机失联，进行重新连接
	memcpy(&motor->parameter, &param, sizeof(param));
	// 指定发送的CAN函数
	motor->parameter.ctx.peripheral_handle	= param.peripheral_handle;
	motor->parameter.ctx.tx_cmd = DM_motor_platformTrans;
	DM_motor_handleListAdd(motor);
}

float DmMotor_GetOutputPositionRad(DM_motor_t *motor)
{
	return motor->status.outputPositionRad;
}

float DmMotor_GetPutputOmegaRadPerSecond(DM_motor_t *motor)
{
	return motor->status.outputVelocityRadPerSecond;
}

float DmMotor_GetTorqueSi(DM_motor_t *motor)
{
	return motor->status.torqueSI;
}

uint32_t DmMotor_GetResolvedCnt(DM_motor_t *motor)
{
	return motor->status.resolvedCnt;
}

void DM_motor_Init(void)
{
	// 初始化电机列表
	memset(&DM_motor_handleList, 0, sizeof(DM_motor_handleList));

	// 电机初始化用的句柄
	DM_motor_parameter_t init_struct;
	memset(&init_struct, 0, sizeof(init_struct));
	// 初始化左前电机
	init_struct.deviceCanId			= 0x01;
	init_struct.masterCanId			= 0x05;
	init_struct.peripheral_handle	= &hcan1;
	init_struct.operationMode		= DM_MOTOR_CONTROL_POS_VEL_MODE;
	init_struct.mitModeParameter.minimumTargetTorque	= -10.f;
	init_struct.mitModeParameter.maximumTargetTorque	= 10.f;
	init_struct.mitModeParameter.minimumTargetPosition	= -12.5f;
	init_struct.mitModeParameter.maximumTargetPosition	= 12.5;
	init_struct.mitModeParameter.minimumTargetVelocity	= -30.f;
	init_struct.mitModeParameter.maximumTargetVelocity	= 30.f;
	DM_motor_handleInit(&dmMotor_joint4, init_struct);


	// 初始化 CAN 发送所需的成员
	#if defined(STM32F105) | defined(STM32F407)
		// CAN发送时用到了的结构体
		DM_motor_CAN_TxHeaderStruct.StdId = 0;
        DM_motor_CAN_TxHeaderStruct.DLC = 8;
        DM_motor_CAN_TxHeaderStruct.IDE = CAN_ID_STD; // 使用标准帧 
        DM_motor_CAN_TxHeaderStruct.RTR = CAN_RTR_DATA;
        DM_motor_CAN_TxHeaderStruct.TransmitGlobalTime = DISABLE;
	#endif

	// 收腿

	//while (HAL_GetTick() < 9000)


		
		//HAL_Delay(300);
//		DM_motor_RF.command.mode	= DM_MOTOR_SET_MODE_ENABLE;
//		DM_motor_modeFrameTransmit(&DM_motor_RF.parameter.ctx, &DM_motor_RF);
		//HAL_Delay(1);

		
//		HAL_Delay(1);
//		
//		
//		HAL_Delay(1);
	


	HAL_Delay(1);

}

void DM_motor_feedback_handler(DM_motor_t *motor, uint32_t CAN_ID, uint8_t data[])
{
	DM_motor_FeedbackProcess(motor, CAN_ID, data);
}

/**
 * @brief 供用户自定义的发送函数
 * @param peripheral_handle 电机发送时外设的句柄
 * @param CANID 发送消息的拓展帧
 * @param data[] 发送消息的数据帧
 * @return 发送是否成功
 */
uint8_t DM_motor_platformTrans(void *peripheral_handle, uint32_t CANID, uint8_t data[])
{
	uint8_t ret, freeLevel;
	#if defined(STM32F105) | defined(STM32F407)
		DM_motor_CAN_TxHeaderStruct.StdId = CANID;
		
		do{
			freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(peripheral_handle);
		}
		while (freeLevel != 3); // 等待邮箱清空
        ret = HAL_CAN_AddTxMessage(peripheral_handle, &DM_motor_CAN_TxHeaderStruct, data, &DM_motor_pTxMailbox);
	
		do{
			freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(peripheral_handle);
		}
		while (freeLevel != 3); // 等待发送完成
		return ret;
	#endif
}

uint8_t DM_motor_platformMsDelay(uint32_t ms)
{
	#if defined(STM32F105)
		HAL_Delay(ms);
	#endif
}
