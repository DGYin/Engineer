/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_SMOOTHEN_H
#define ALG_SMOOTHEN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

typedef enum{
	ALG_SMOOTHEN_OK		= 0x00U,
	ALG_SMOOTHEN_ERROR	= 0x01U,
} ALG_SMOOTHEN_RETURN_T;

typedef enum{
	UNIFORM_SMOOTHEN_TYPE_ABSOLUTE	= 0x00U,	// 绝对增量
	UNIFORM_SMOOTHEN_TYPE_RELATIVE	= 0x01U,	// 相对增量
} ALG_SMOOTHEN_UNIFORM_SMOOTHEN_TYPE_T;

typedef struct{
	// 输入输出值
	float input;		// 输入值
	float output;		// 输出值
	float target;		// 目标值
	float currentValue;	// 当前运算值（相对模式没用）
	// 增量参数
	ALG_SMOOTHEN_UNIFORM_SMOOTHEN_TYPE_T uniformType;	// 增量类型
	float positiveDelta;	// 正增量，应该为正值
	float negativeDelta;	// 负增量,应该为正值，运算时会变为减法
} algSmoothen_uniformSmoothen_t;


ALG_SMOOTHEN_RETURN_T AlgSmoothen_UniformSmoothen(	algSmoothen_uniformSmoothen_t* ush,
													float input, float target, float* output);
ALG_SMOOTHEN_RETURN_T AlgSmoothen_SetUsDelta(	algSmoothen_uniformSmoothen_t* ush,
												float posDel, float negDel);
ALG_SMOOTHEN_RETURN_T AlgSmoothen_UsHanldeInitExample(	algSmoothen_uniformSmoothen_t* ush,
														ALG_SMOOTHEN_UNIFORM_SMOOTHEN_TYPE_T usType);
extern algSmoothen_uniformSmoothen_t joint1AngleSmoothen, joint2AngleSmoothen;


#ifdef __cplusplus
}
#endif
#endif
