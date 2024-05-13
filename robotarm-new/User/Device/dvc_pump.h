#ifndef DEV_PUMP_H
#define DEV_PUMP_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* USER Settings -------------------------------------------------------------*/
// Define board and chips
#define ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C	// 配置适用于 C 板的设置


	
	
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#ifdef __cplusplus
	#include "string.h"
#endif
typedef enum{
	DVC_PUMP_RETURN_OK		= 0x00U,
	DVC_PUMP_RETURN_ERROR	= 0x00U,
}DVC_PUMP_RETURN_T;

typedef enum{
	DVC_PUMP_OFF	= 0x00U,
	DVC_PUMP_ON		= 0x01U,
	
} DVC_PUMP_SWITCHES_T;


typedef uint8_t (*dvcPump_setSwitchPtr)	(	void*,
											uint16_t,
											DVC_PUMP_SWITCHES_T);


typedef struct
{
	dvcPump_setSwitchPtr setSwitch;
	void*		peripheral_handle;
	uint16_t	pinNum;
} dvcPump_parameter_t;
typedef struct
{
	dvcPump_parameter_t parameter;
	struct{
		DVC_PUMP_SWITCHES_T switches;
	} status;
	
} dvcPump_t;

DVC_PUMP_RETURN_T DvcPump_SetStatus(dvcPump_t* pump, DVC_PUMP_SWITCHES_T switches);
DVC_PUMP_RETURN_T DvcPump_InitExample(void);

extern dvcPump_t dvcPump_storage, dvcPump_suction;

#ifdef __cplusplus
}
#endif
#endif
