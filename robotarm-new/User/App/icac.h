/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ICAC_H
#define ICAC_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/

/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define ICAC_LIST_MAXIMUM_NUM 20

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported typedef -----------------------------------------------------*/
typedef enum
{
	ICAC_OK				= 0x00U,
	ICAC_ERROR			= 0x01U,
	ICAC_WRONG_PARAM	= 0x02U,
} ICAC_RETURN_T;


typedef enum
{
	ICAC_TYPE_WATCHDOG	= 0x00U,
	ICAC_TYPE_COUNTER	= 0x01U,
	ICAC_TYPE_KEEPALIVE	= 0x02U,
} ICAC_TYPE_T;

typedef enum
{
	ICAC_OFFLINE		= 0x00U,
	ICAC_PACKAGE_LOST	= 0x01U,
	ICAC_ONLINE			= 0x02U,
} ICAC_CONNECTION_STATUS_T;

typedef struct
{
	struct{
		// 通用参数
		ICAC_TYPE_T icacType;
		uint32_t	callingPeriodMs;
		uint16_t	icacListPosition;
		// 适用于 ICAC_TYPE_COUNTER 类型的参数
		uint32_t*	pCounter;
	} parameter;
	
	struct{
		ICAC_CONNECTION_STATUS_T connectionStatus;
	} status;
	
	struct{
		// 适用于 ICAC_TYPE_COUNTER 类型的返回值
		uint32_t lastCounter;
		uint32_t nowCounter;
	} feedback;
} icac_t;


typedef struct
{
	icac_t* 	icac;
	uint16_t	remainningMs;
} icacCallingListElement_t;





ICAC_RETURN_T Icac_CounterInit(icac_t* icac, uint32_t callingPeriodMs, uint32_t* pCounter);
void Icac_HandleInit(void);
void Icac_TaskScheduler(void);
ICAC_CONNECTION_STATUS_T Icac_GetConnectionStatus(icac_t* icac);
/* Extern variables ---------------------------------------------------*/
extern icac_t icac_akMotor_joint1, icac_akMotor_joint2, icac_dmMotor_joint3;
#ifdef __cplusplus
}
#endif
#endif