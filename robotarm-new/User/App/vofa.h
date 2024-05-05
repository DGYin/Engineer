/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VOFA_H
#define VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
// Define board and chips
#define ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C		// 配置适用于 C 板的设置
//#define MOSASAURUS_ELITE_BOARD				// 配置适用于菁英板的设置
//#define MOSASAURUS_STEERING_CONTROL_BOARD		// 配置适用于舵轮控制板的设置

#if defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined(MOSASAURUS_ELITE_BOARD) | defined(MOSASAURUS_STEERING_CONTROL_BOARD) // 默认都用USB
	#define VOFA_USE_USB_CDC // 使用usb作为数据传输接口
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported typedef -----------------------------------------------------*/
typedef enum
{
	VOFA_OK				= 0x00U,
	VOFA_ERROR			= 0x01U,
	VOFA_WRONG_PARAM	= 0x02U,
} VOFA_RETURN_T;




typedef uint8_t (*vofa_transmit)		(void*, uint8_t*, uint16_t);
typedef struct
{
	vofa_transmit	pPlatformTransmit;
	void *peripheral_handle;
} vofa_ctx_t;

typedef struct{
	vofa_ctx_t ctx;
	void *peripheral_handle;
} vofaParameter_t;

typedef struct
{
	vofaParameter_t parameter;
	struct{
		uint8_t nothingHere;
	} status;
	
} vofa_t;


VOFA_RETURN_T Vofa_InitExample(vofa_t* vofa);
VOFA_RETURN_T Vofa_JustFloatInSeperatedChannelTransmit(vofa_t* vofa, float* pFloat, uint8_t floatNum);


extern vofa_t vofa;

#ifdef __cplusplus
}
#endif
#endif