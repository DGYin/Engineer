#include "DM_motor_bsp.h"

DM_MOTOR_BSP_RETURN_T DM_motor_FeedbackProcess(DM_motor_t *motor, uint32_t CAN_ID, uint8_t data[])
{
	// 先把数据存起来
	// 这个电机和大疆的一样烦，居然是LSB在后 (╯‵□′)╯︵┻━┻
	motor->feedback.id								= (data[0])&0x0F;
	motor->feedback.statecode						= (data[0])>>4;
	motor->feedback.outputPositionLsb				= (data[1]<<8)|data[2];
	motor->feedback.outputVelocityLsb				= (data[3]<<4)|(data[4]>>4);
	motor->feedback.torqueLsb						= ((data[4]&0xF)<<8)|data[5];
	motor->feedback.mosTempretureCelsiusDegree		= data[6];
	motor->feedback.rotorTempretureCelsiusDegree	= data[7];
	// 把原始数据进行转化
	motor->status.statecode						= motor->feedback.statecode;
	motor->status.outputPositionRad				= DM_motor_From_intLsb_To_floatValue(	motor->feedback.outputPositionLsb,
																						motor->parameter.mitModeParameter.minimumTargetPosition,
																						motor->parameter.mitModeParameter.maximumTargetPosition,
																						16);
	motor->status.outputVelocityRadPerSecond	= DM_motor_From_intLsb_To_floatValue(	motor->feedback.outputVelocityLsb,
																						motor->parameter.mitModeParameter.minimumTargetVelocity,
																						motor->parameter.mitModeParameter.maximumTargetVelocity,
																						12);
	motor->status.torqueSI						= DM_motor_From_intLsb_To_floatValue(	motor->feedback.torqueLsb,
																						motor->parameter.mitModeParameter.minimumTargetTorque,
																						motor->parameter.mitModeParameter.maximumTargetTorque,
																						12);
	motor->status.mosTempretureCelsiusDegree	= motor->feedback.mosTempretureCelsiusDegree;
	motor->status.rotorTempretureCelsiusDegree	= motor->feedback.rotorTempretureCelsiusDegree;
	motor->status.resolvedCnt++;
	return DM_MOTOR_BSP_OK;
}

DM_MOTOR_BSP_RETURN_T DM_motor_dataFrameTransmit(DM_motor_ctx_t *ctx, DM_motor_t *motor)
{
	uint32_t ret;
	// 创建并初始化发送数据区
	uint8_t data[8];
	memset(&data, 0, sizeof(data));
	// 准备好需要发送的数据
	uint32_t transCanId	= motor->parameter.deviceCanId + motor->parameter.operationMode;
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	switch (motor->parameter.operationMode) // 枚举当前控制模式
	{
		case DM_MOTOR_CONTROL_MIT_MODE:			// MIT 协议控制模式 （力控专用）
			// 数据转换
			pos_tmp = DM_motor_From_floatValue_To_intLsb(	motor->command.mitModeCommand.targetPosition,
															motor->parameter.mitModeParameter.minimumTargetPosition,	
															motor->parameter.mitModeParameter.maximumTargetPosition,  
															16);
			vel_tmp = DM_motor_From_floatValue_To_intLsb(	motor->command.mitModeCommand.targetVelocity,
															motor->parameter.mitModeParameter.minimumTargetVelocity,
															motor->parameter.mitModeParameter.maximumTargetVelocity,
															12);
			kp_tmp  = DM_motor_From_floatValue_To_intLsb(	motor->command.mitModeCommand.kP,
															motor->parameter.mitModeParameter.minimumKP,
															motor->parameter.mitModeParameter.maximumKP,
															12);
			kd_tmp  = DM_motor_From_floatValue_To_intLsb(	motor->command.mitModeCommand.kD,
															motor->parameter.mitModeParameter.minimumKD,
															motor->parameter.mitModeParameter.maximumKD,
															12);
			tor_tmp = DM_motor_From_floatValue_To_intLsb(	motor->command.mitModeCommand.targetTorque,
															motor->parameter.mitModeParameter.minimumTargetTorque,
															motor->parameter.mitModeParameter.maximumTargetTorque,
															12);
			// 数据写入
			data[0] = (pos_tmp >> 8);	// data[0]存放pos_tmp高八位
			data[1] = pos_tmp;			// data[1]存放pos_tmp低八位
			data[2] = (vel_tmp >> 4);	// data[2]存放vel_tmp11位到4位
			data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
			data[4] = kp_tmp;
			data[5] = (kd_tmp >> 4);
			data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
			data[7] = tor_tmp;
			// 调用发送
			ret = ctx->tx_cmd(ctx->peripheral_handle, motor->parameter.deviceCanId+DM_MOTOR_CONTROL_MIT_MODE, data);
			break;
		case DM_MOTOR_CONTROL_POS_VEL_MODE:	// 位置速度环模式
			// 数据写入
			memcpy(&data[0], (uint8_t*) &motor->command.posVelModeCommand.targetPosition, 4);
			memcpy(&data[4], (uint8_t*) &motor->command.posVelModeCommand.targetVelocity, 4);
			// 调用发送
			ret = ctx->tx_cmd(ctx->peripheral_handle, transCanId, data);
			break;
		case DM_MOTOR_CONTROL_VEL_MODE:		// 速度环模式
			// 数据写入
			memcpy(&data, (uint8_t*) &motor->command.velModeCommand.targetVelocity, 4);
			// 调用发送
			ret = ctx->tx_cmd(ctx->peripheral_handle, motor->parameter.deviceCanId+DM_MOTOR_CONTROL_VEL_MODE, data);
			break;
		default:
			return DM_MOTOR_BSP_WRONG_PARAM;
			break;
	}
	return ret;
}

DM_MOTOR_BSP_RETURN_T DM_motor_modeFrameTransmit(DM_motor_ctx_t *ctx, DM_motor_t *motor)
{
	uint32_t ret;
	// 创建并初始化发送数据区
	uint8_t data[8];
	memset(&data, 0xFF, sizeof(data));
	data[7] = motor->command.mode;
	switch (motor->parameter.operationMode) // 枚举当前控制模式
	{
		case DM_MOTOR_CONTROL_MIT_MODE:			// MIT 协议控制模式 （力控专用）
			ret =  ctx->tx_cmd(ctx->peripheral_handle, motor->parameter.deviceCanId+DM_MOTOR_CONTROL_MIT_MODE, data);
			break;
		case DM_MOTOR_CONTROL_POS_VEL_MODE:	// 位置速度环模式
			ret = ctx->tx_cmd(ctx->peripheral_handle, motor->parameter.deviceCanId+DM_MOTOR_CONTROL_POS_VEL_MODE, data);
			break;
		case DM_MOTOR_CONTROL_VEL_MODE:		// 速度环模式
			ret = ctx->tx_cmd(ctx->peripheral_handle, motor->parameter.deviceCanId+DM_MOTOR_CONTROL_VEL_MODE, data);
			break;
		default:
			return DM_MOTOR_BSP_WRONG_PARAM;
			break;
	}
	return ret;
}
float DM_motor_From_intLsb_To_floatValue(uint16_t input, float input_min, float input_max, uint8_t bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = input_max - input_min;
	float offset = input_min;
	return ((float)input)*span/((float)((1<<bits)-1)) + offset;

}

int16_t DM_motor_From_floatValue_To_intLsb(float input, float input_min, float input_max, uint8_t bits)
{
	float span = input_max - input_min;
	float offset = input_min;
	return (int) ((input-offset)*((float)((1<<bits)-1))/span);
}