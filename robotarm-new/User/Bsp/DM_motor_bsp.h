#ifndef DM_MOTOR_BSP_H
#define DM_MOTOR_BSP_H

#ifdef __cplusplus
extern "C" {
#endif


/* USER Settings -------------------------------------------------------------*/
#define STM32F105


/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

typedef uint8_t (*DM_motor_tx_ptr)(void *, uint32_t , uint8_t[]);



typedef enum
{
	DM_MOTOR_BSP_OK,
	DM_MOTOR_BSP_ERROR, 
	DM_MOTOR_BSP_WRONG_PARAM,
}DM_MOTOR_BSP_RETURN_T;

/* 电机固有参数 --------------------------------------------------------------*/

// 工作模式参数

typedef enum
{
	DM_MOTOR_CONTROL_MIT_MODE		= 0x0000U, // MIT 协议控制模式 （力控专用）
	DM_MOTOR_CONTROL_POS_VEL_MODE	= 0x0100U, // 位置速度环模式
	DM_MOTOR_CONTROL_VEL_MODE		= 0x0200U, // 速度环模式
} DM_MOTOR_OPERATION_MODE_T;

// 控制模式参数

typedef enum
{
	DM_MOTOR_SET_MODE_ENABLE	= 0xFC,
	DM_MOTOR_SET_MODE_DISABLE	= 0xFD,
	DM_MOTOR_SET_ZERO_POSITION	= 0xFE,
	DM_MOTOR_CLEAR_ERROR		= 0xFB,
} DM_MOTOR_MODE_T;

typedef enum
{
	DM_MOTOR_STATECODE_DISABLE			= 0x00,
	DM_MOTOR_STATECODE_ENABLE			= 0x01,
	DM_MOTOR_STATECODE_OVERVOLTAGE		= 0x08,
	DM_MOTOR_STATECODE_UNDERVOLTAGE		= 0x00,
	DM_MOTOR_STATECODE_OVERCURRENT		= 0x0A,
	DM_MOTOR_STATECODE_MOS_OVERHEAT		= 0x0B,
	DM_MOTOR_STATECODE_COIL_OVERHEAT	= 0x0C,
	DM_MOTOR_STATECODE_LOST_CONNECTION	= 0x0D,
	DM_MOTOR_STATECODE_OVERLOAD			= 0x0E,
} DM_MOTOR_STATECODE_T;


// MIT 模式下电机的指令和参数

typedef struct
{
	float targetPosition;
	float targetVelocity;
	float kP;
	float kD;
	float targetTorque;
} DM_motor_mitModeCommand_t;

typedef struct
{
	float minimumTargetPosition;
	float maximumTargetPosition;
	float minimumTargetVelocity;
	float maximumTargetVelocity;
	float minimumKP;
	float maximumKP;
	float minimumKD;
	float maximumKD;
	float minimumTargetTorque;
	float maximumTargetTorque;
} DM_motor_mitModeParameter_t;

// 位置速度环模式下电机的指令和参数

typedef struct
{
	float targetPosition;
	float targetVelocity;
} DM_motor_posVelModeCommand_t;

// 速度环模式下电机的指令和参数

typedef struct
{
	float targetVelocity;
} DM_motor_velModeCommand_t;

/* 电机句柄成员 --------------------------------------------------------------*/

// command 指令成员定义

typedef struct
{
	
	DM_MOTOR_MODE_T			mode;	

	// 控制指令
	
	// 运动指令
	DM_motor_mitModeCommand_t		mitModeCommand;
	DM_motor_posVelModeCommand_t	posVelModeCommand;
	DM_motor_velModeCommand_t		velModeCommand;
}DM_motor_command_t;

// parameter 参数成员定义

typedef struct
{
	DM_motor_tx_ptr	tx_cmd;
	void *peripheral_handle;
} DM_motor_ctx_t;

typedef struct
{
	// 外设相关
	void*			peripheral_handle;
	DM_motor_ctx_t	ctx;
	uint16_t		masterCanId;
	uint16_t		deviceCanId;
	// 数据转换相关
	DM_motor_mitModeParameter_t		mitModeParameter;

	// 电机固有参数相关
	DM_MOTOR_OPERATION_MODE_T operationMode; 
	float compensationTorque;
}DM_motor_parameter_t;

// status 状态成员定义
typedef struct
{
	DM_MOTOR_STATECODE_T	statecode;
	float					outputPositionRad;
	float					outputVelocityRadPerSecond;
	float					torqueSI;
	uint8_t					mosTempretureCelsiusDegree;
	uint8_t					rotorTempretureCelsiusDegree;
	uint32_t				resolvedCnt;
}DM_motor_status_t;

// feedback 反馈成员定义

typedef struct
{
	uint8_t id;
	uint8_t statecode;
	int16_t outputPositionLsb;
	int16_t	outputVelocityLsb;
	int16_t	torqueLsb;
	uint8_t	mosTempretureCelsiusDegree;
	uint8_t	rotorTempretureCelsiusDegree;
}DM_motor_feedback_t;

// 电机句柄定义

typedef struct
{
	DM_motor_parameter_t	parameter;
	DM_motor_command_t		command;
	DM_motor_feedback_t		feedback;
	DM_motor_status_t		status;
} DM_motor_t;


float DM_motor_From_intLsb_To_floatValue(uint16_t input, float input_min, float input_max, uint8_t bits);
int16_t DM_motor_From_floatValue_To_intLsb(float input, float input_min, float input_max, uint8_t bits);

DM_MOTOR_BSP_RETURN_T DM_motor_FeedbackProcess(DM_motor_t *motor, uint32_t CAN_ID, uint8_t data[]);
DM_MOTOR_BSP_RETURN_T DM_motor_dataFrameTransmit(DM_motor_ctx_t *ctx, DM_motor_t *motor);
DM_MOTOR_BSP_RETURN_T DM_motor_modeFrameTransmit(DM_motor_ctx_t *ctx, DM_motor_t *motor);

#ifdef __cplusplus
}
#endif
#endif
