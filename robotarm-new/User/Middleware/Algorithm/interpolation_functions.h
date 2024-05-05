/******************************************************************************
 * @file     interpolation_functions.h
 * @brief    Public header file for CMSIS DSP Library
 * @version  V1.10.0
 * @date     08 July 2021
 * Target Processor: Cortex-M and Cortex-A cores
 ******************************************************************************/
/*
 * Copyright (c) 2010-2020 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

 
#ifndef _INTERPOLATION_FUNCTIONS_H_
#define _INTERPOLATION_FUNCTIONS_H_

//#include "arm_math_types.h"
//#include "arm_math_memory.h"
#include "stdint.h"

#define float32_t float

#ifdef   __cplusplus
extern "C"
{
#endif


/**
 * @defgroup groupInterpolation Interpolation Functions
 * These functions perform 1- and 2-dimensional interpolation of data.
 * Linear interpolation is used for 1-dimensional data and
 * bilinear interpolation is used for 2-dimensional data.
 */


  /**
   * @brief Struct for specifying cubic spline type
   */
  typedef enum
  {
    ARM_SPLINE_NATURAL = 0,           /**< Natural spline */
    ARM_SPLINE_PARABOLIC_RUNOUT = 1   /**< Parabolic runout spline */
  } arm_spline_type;

  /**
   * @brief Instance structure for the floating-point cubic spline interpolation.
   */
  typedef struct
  {
    arm_spline_type type;      /**< Type (boundary conditions) */
    const float32_t * x;       /**< x values */
    const float32_t * y;       /**< y values */
    uint32_t n_x;              /**< Number of known data points */
    float32_t * coeffs;        /**< Coefficients buffer (b,c, and d) */
  } arm_spline_instance_f32;


  /**
   * @brief Processing function for the floating-point cubic spline interpolation.
   * @param[in]  S          points to an instance of the floating-point spline structure.
   * @param[in]  xq         points to the x values ot the interpolated data points.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples of output data.
   */
  void arm_spline_f32(
        arm_spline_instance_f32 * S, 
  const float32_t * xq,
        float32_t * pDst,
        uint32_t blockSize);

  /**
   * @brief Initialization function for the floating-point cubic spline interpolation.
   * @param[in,out] S        points to an instance of the floating-point spline structure.
   * @param[in]     type     type of cubic spline interpolation (boundary conditions)
   * @param[in]     x        points to the x values of the known data points.
   * @param[in]     y        points to the y values of the known data points.
   * @param[in]     n        number of known data points.
   * @param[in]     coeffs   coefficients array for b, c, and d
   * @param[in]     tempBuffer   buffer array for internal computations
   */
  void arm_spline_init_f32(
          arm_spline_instance_f32 * S,
          arm_spline_type type,
    const float32_t * x,
    const float32_t * y,
          uint32_t n, 
          float32_t * coeffs,
          float32_t * tempBuffer);

#ifdef   __cplusplus
}
#endif

#endif /* ifndef _INTERPOLATION_FUNCTIONS_H_ */
