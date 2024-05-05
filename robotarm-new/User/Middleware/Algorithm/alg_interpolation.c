#include "alg_interpolation.h"
#include "interpolation_functions.h"

uint8_t* AlgInterp_platformMalloc(uint16_t byteSize);
ALG_NATURAL_INTERP_RETURN_T AlgInterp_platformFree(void* add);




ALG_NATURAL_INTERP_RETURN_T AlgInterp_NaturalInterp(float inputXn[], float inputYn[], uint8_t inputLength, float* outputXn, float* outputYn, uint8_t outputLength)
{
	float coeffs[3*(inputLength-1)];		// 插补系数缓冲
	float tempBuffer[2 * inputLength - 1]; 	// 插补临时缓冲
	
	arm_spline_instance_f32 S;
	arm_spline_init_f32(&S,
						ARM_SPLINE_NATURAL, 
						inputXn,
						inputYn,
						inputLength,
						coeffs,
						tempBuffer);
	// 样条计算
	float scale2 = (inputXn[inputLength-1]-inputXn[0])/(outputLength-1.f);
	for (int i=0; i<outputLength; i++) // 插值后 X 坐标值，需要自己设置
		outputXn[i] = inputXn[0] + i*scale2;
	arm_spline_f32	(&S,
					 outputXn,
					 outputYn,
					 outputLength);
	return NATURAL_INTERP_OK;
}

ALG_NATURAL_INTERP_RETURN_T AlgInterp_ParabolicRunoutInterp(float* inputXn, float* inputYn, uint8_t inputLength, float* outputXn, float* outputYn, uint8_t outputLength)
{
	float coeffs[3*(inputLength-1)];		// 插补系数缓冲
	float tempBuffer[2 * inputLength - 1]; 	// 插补临时缓冲
	
	arm_spline_instance_f32 S;
	arm_spline_init_f32(&S,
						ARM_SPLINE_PARABOLIC_RUNOUT, 
						inputXn,
						inputYn,
						inputLength,
						coeffs,
						tempBuffer);
	// 样条计算
	float scale2 = (inputXn[inputLength-1]-inputXn[0])/(outputLength-1.f);
	for (int i=0; i<outputLength; i++) // 插值后 X 坐标值，需要自己设置
		outputXn[i] = inputXn[0] + i*scale2;
	arm_spline_f32	(&S,
					 outputXn,
					 outputYn,
					 outputLength);
	return NATURAL_INTERP_OK;
}


uint8_t* AlgInterp_platformMalloc(uint16_t byteSize)
{
	#if defined(INC_FREERTOS_H)
		return pvPortMalloc(byteSize);
	#else
		return (uint8_t*)malloc(size);
	#endif
}

ALG_NATURAL_INTERP_RETURN_T AlgInterp_platformFree(void* add)
{
	#if defined(INC_FREERTOS_H)
		vPortFree(add);
		return NATURAL_INTERP_OK;
	#else
		free(add);
		add = NULL;
		return NATURAL_INTERP_OK;
	#endif
}