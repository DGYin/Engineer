/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_INTERPOLATION_H
#define ALG_INTERPOLATION_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "cmsis_os.h"
#include "arm_math.h"

typedef enum{
	NATURAL_INTERP_OK		= 0x00U,
	NATURAL_INTERP_ERROR	= 0x01U,
} ALG_NATURAL_INTERP_RETURN_T;



ALG_NATURAL_INTERP_RETURN_T AlgInterp_NaturalInterp(float inputXn[], float inputYn[], uint8_t inputLength, float* outputXn, float* outputYn, uint8_t outputLength);
ALG_NATURAL_INTERP_RETURN_T AlgInterp_ParabolicRunoutInterp(float* inputXn, float* inputYn, uint8_t inputLength, float* outputXn, float* outputYn, uint8_t outputLength);

#endif
