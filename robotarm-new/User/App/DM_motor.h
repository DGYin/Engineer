#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105

/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define MAXIMUM_DM_MOTOR_HANDLE_NUM 4



#if defined(STM32F105) | defined(STM32F407)
	#define MOTOR_ON_BUS_1 hcan1
	#define MOTOR_ON_BUS_2 hcan2
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "DM_motor_bsp.h"


typedef enum
{
	DM_MOTOR_OK,
	DM_MOTOR_ERROR, 
	
	DM_MOTOR_ILLEGAL_HANDLE
}DM_MOTOR_RETURN_T;

typedef struct
{
	DM_motor_t	*motor_handle;
	void		*peripheral_handle;
	uint8_t		CANID;
	//uint8_t		RX_CANID;
}DM_motor_handleList_unit_t;




extern DM_motor_t dmMotor_joint4;

DM_motor_t *DM_motor_FindSteeringHandle_via_CANIDandPeripheral(uint8_t CANID, void *peripheral_handle);
void DM_motor_Init(void);
void DM_motor_feedback_handler(DM_motor_t *motor, uint32_t CAN_ID, uint8_t data[]);

DM_MOTOR_RETURN_T DM_motor_setZeroPosition(DM_motor_t *motor);
DM_MOTOR_RETURN_T DM_motor_clearError(DM_motor_t *motor);
DM_MOTOR_RETURN_T DM_motor_setEnable(DM_motor_t *motor);
DM_MOTOR_RETURN_T DM_motor_setDisable(DM_motor_t *motor);



DM_MOTOR_RETURN_T DM_motor_setTorqueSI(DM_motor_t *motor, float torque_nm);
DM_MOTOR_RETURN_T DM_motor_SetCurrent_SI(DM_motor_t *motor, float current_ampire);
DM_MOTOR_RETURN_T DM_motor_SetMultiPosition_SI(DM_motor_t *motor, float position_degree);
DM_MOTOR_RETURN_T DM_motor_SetMultiPositionSpeedAcceleration_SI(DM_motor_t *motor, int32_t position_degree, float output_speed_limit_SI, float output_acc_limit_SI);
DM_MOTOR_RETURN_T DM_motor_setPosVelControl(DM_motor_t *motor, DM_motor_posVelModeCommand_t command);

float DmMotor_GetOutputPositionRad(DM_motor_t *motor);
float DmMotor_GetPutputOmegaRadPerSecond(DM_motor_t *motor);
float DmMotor_GetTorqueSi(DM_motor_t *motor);
uint32_t DmMotor_GetResolvedCnt(DM_motor_t *motor);
/* CRITICAL Settings, NEVER CHANGE! ------------------------------------------*/
#define DM_MOTOR_NULL_HANDLE (DM_motor_t*)NULL


#ifdef __cplusplus
}
#endif
#endif
