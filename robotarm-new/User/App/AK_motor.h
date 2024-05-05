#ifndef AK_MOTOR_H
#define AK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105
#define FREERTOS_ENABLE
/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define MAXIMUM_AK_MOTOR_HANDLE_NUM 10



#if defined(STM32F105) | defined(STM32F407)
	#define AK_MOTOR_ON_BUS_1 hcan1
	#define AK_MOTOR_ON_BUS_2 hcan2
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "AK_motor_bsp.h"
#if defined(FREERTOS_ENABLE)
	#include "cmsis_os.h"
#endif

typedef enum
{
	AK_MOTOR_OK,
	AK_MOTOR_ERROR, 
	
	AK_MOTOR_ILLEGAL_HANDLE
}AK_MOTOR_RETURN_T;

typedef struct
{
	AK_motor_t	*motor_handle;
	void		*peripheral_handle;
	uint8_t		CANID;
}AK_motor_handle_list_unit_t;






AK_motor_t *AK_motor_FindSteeringHandle_via_CANIDandPeripheral(uint8_t CANID, void *peripheral_handle);
void AK_motor_Init(void);
void AK_motor_feedback_handler(AK_motor_t *motor, uint32_t CAN_ID, uint8_t data[]);

void AK_motor_SetTorque_SI(AK_motor_t *motor, float torque_nm);
void AK_motor_SetCurrent_SI(AK_motor_t *motor, float current_ampire);
void AK_motor_SetMultiPosition_SI(AK_motor_t *motor, float position_degree);
void AK_motor_SetMultiPositionSpeedAcceleration_SI(AK_motor_t *motor, float position_degree, float output_speed_limit_SI, float output_acc_limit_SI);

float AkMotor_GetOutputPositionRad(AK_motor_t* motor);
float AkMotor_GetPutputOmegaRadPerSecond(AK_motor_t* motor);
float AkMotor_GetOutputTorqueSi(AK_motor_t* motor);
uint32_t AkMotor_GetResolvedCnt(AK_motor_t* motor);


extern AK_motor_t akMotor_joint2, akMotor_joint3;

/* CRITICAL Settings, NEVER CHANGE! ------------------------------------------*/
#define AK_MOTOR_NULL_HANDLE (AK_motor_t*)NULL


#ifdef __cplusplus
}
#endif
#endif
