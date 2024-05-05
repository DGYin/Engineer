/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        InterpolationFunctions.c
 * Description:  Combination of all interpolation function source files.
 *
 * $Date:        22. July 2020
 * $Revision:    V1.0.0
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2020 ARM Limited or its affiliates. All rights reserved.
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


#include "interpolation_functions.h"


/**
 * @brief Initialization function for the floating-point cubic spline interpolation.
 * @param[in,out] S           points to an instance of the floating-point spline structure.
 * @param[in]     type        type of cubic spline interpolation (boundary conditions)
 * @param[in]     x           points to the x values of the known data points.
 * @param[in]     y           points to the y values of the known data points.
 * @param[in]     n           number of known data points.
 * @param[in]     coeffs      coefficients array for b, c, and d
 * @param[in]     tempBuffer  buffer array for internal computations
 *
 */

void arm_spline_init_f32(
        arm_spline_instance_f32 * S,
        arm_spline_type type,
  const float32_t * x,
  const float32_t * y,
        uint32_t n, 
        float32_t * coeffs,
        float32_t * tempBuffer)
{
    /*** COEFFICIENTS COMPUTATION ***/
    /* Type (boundary conditions):
        - Natural spline          ( S1''(x1) = 0 ; Sn''(xn) = 0 )
        - Parabolic runout spline ( S1''(x1) = S2''(x2) ; Sn-1''(xn-1) = Sn''(xn) ) */
    
    /* (n-1)-long buffers for b, c, and d coefficients */
    float32_t * b = coeffs;
    float32_t * c = coeffs+(n-1);
    float32_t * d = coeffs+(2*(n-1));    

    float32_t * u = tempBuffer;   /* (n-1)-long scratch buffer for u elements */
    float32_t * z = tempBuffer+(n-1); /* n-long scratch buffer for z elements */

    float32_t hi, hm1; /* h(i) and h(i-1) */
    float32_t Bi; /* B(i), i-th element of matrix B=LZ */
    float32_t li; /* l(i), i-th element of matrix L    */
    float32_t cp1; /* Temporary value for c(i+1) */

    int32_t i; /* Loop counter */

    S->x = x;
    S->y = y;
    S->n_x = n;

    /* == Solve LZ=B to obtain z(i) and u(i) == */

    /* -- Row 1 -- */
    /* B(0) = 0, not computed */
    /* u(1,2) = a(1,2)/a(1,1) = a(1,2) */
    if(type == ARM_SPLINE_NATURAL)
        u[0] = 0;  /* a(1,2) = 0 */
    else if(type == ARM_SPLINE_PARABOLIC_RUNOUT)
        u[0] = -1; /* a(1,2) = -1 */

    z[0] = 0;  /* z(1) = B(1)/a(1,1) = 0 always */

    /* -- Rows 2 to N-1 (N=n+1) -- */
    hm1 = x[1] - x[0]; /* Initialize h(i-1) = h(1) = x(2)-x(1) */

    for (i=1; i<(int32_t)n-1; i++)
    {
        /* Compute B(i) */
        hi = x[i+1]-x[i];
        Bi = 3*(y[i+1]-y[i])/hi - 3*(y[i]-y[i-1])/hm1;

        /* l(i) = a(i)-a(i,i-1)*u(i-1) = 2[h(i-1)+h(i)]-h(i-1)*u(i-1) */
        li = 2*(hi+hm1) - hm1*u[i-1];

        /* u(i) = a(i,i+1)/l(i) = h(i)/l(i) */
        u[i] = hi/li;

        /* z(i) = [B(i)-h(i-1)*z(i-1)]/l(i) */
        z[i] = (Bi-hm1*z[i-1])/li;

        /* Update h(i-1) for next iteration */
        hm1 = hi;
    }

    /* -- Row N -- */
    /* l(N) = a(N,N)-a(N,N-1)u(N-1) */
    /* z(N) = [-a(N,N-1)z(N-1)]/l(N) */
    if(type == ARM_SPLINE_NATURAL)
    {
        /* li = 1;     a(N,N) = 1; a(N,N-1) = 0 */
        z[n-1] = 0; /* a(N,N-1) = 0 */
    }
    else if(type == ARM_SPLINE_PARABOLIC_RUNOUT)
    {
        li = 1+u[n-2];      /* a(N,N) = 1; a(N,N-1) = -1 */
        z[n-1] = z[n-2]/li; /* a(N,N-1) = -1 */
    }

    /* == Solve UX = Z to obtain c(i) and    */
    /*    compute b(i) and d(i) from c(i) == */

    cp1 = z[n-1]; /* Initialize c(i+1) = c(N) = z(N) */

    for (i=n-2; i>=0; i--) 
    {
        /* c(i) = z(i)-u(i+1)c(i+1) */
        c[i] = z[i]-u[i]*cp1;

        hi = x[i+1]-x[i];
        /* b(i) = [y(i+1)-y(i)]/h(i)-h(i)*[c(i+1)+2*c(i)]/3 */
        b[i] = (y[i+1]-y[i])/hi-hi*(cp1+2*c[i])/3;

        /* d(i) = [c(i+1)-c(i)]/[3*h(i)] */
        d[i] = (cp1-c[i])/(3*hi);

        /* Update c(i+1) for next iteration */
        cp1 = c[i];
    }

    /* == Finally, store the coefficients in the instance == */

    S->coeffs = coeffs;
}

/**
  @addtogroup SplineInterpolate
  @{
 */

/**
 * @brief Processing function for the floating-point cubic spline interpolation.
 * @param[in]  S          points to an instance of the floating-point spline structure.
 * @param[in]  xq         points to the x values of the interpolated data points.
 * @param[out] pDst       points to the block of output data.
 * @param[in]  blockSize  number of samples of output data.
 */

void arm_spline_f32(
        arm_spline_instance_f32 * S, 
  const float32_t * xq,
        float32_t * pDst,
        uint32_t blockSize)
{
    const float32_t * x = S->x;
    const float32_t * y = S->y;
    int32_t n = S->n_x;

    /* Coefficients (a==y for i<=n-1) */
    float32_t * b = (S->coeffs);
    float32_t * c = (S->coeffs)+(n-1);
    float32_t * d = (S->coeffs)+(2*(n-1));    

    const float32_t * pXq = xq;
    int32_t blkCnt = (int32_t)blockSize;
    int32_t blkCnt2;
    int32_t i;
    float32_t x_sc;

    /* Create output for x(i)<x<x(i+1) */
    for (i=0; i<n-1; i++)
    {
        while( *pXq <= x[i+1] && blkCnt > 0 )
        {
            x_sc = *pXq++;

            *pDst = y[i]+b[i]*(x_sc-x[i])+c[i]*(x_sc-x[i])*(x_sc-x[i])+d[i]*(x_sc-x[i])*(x_sc-x[i])*(x_sc-x[i]);

            pDst++;
            blkCnt--;
        }
    }

    /* Create output for remaining samples (x>=x(n)) */                                                     
    blkCnt2 = blkCnt;                                          
    while(blkCnt2 > 0)                                       
    { 
        x_sc = *pXq++; 
  
        *pDst = y[i-1]+b[i-1]*(x_sc-x[i-1])+c[i-1]*(x_sc-x[i-1])*(x_sc-x[i-1])+d[i-1]*(x_sc-x[i-1])*(x_sc-x[i-1])*(x_sc-x[i-1]);
 
        pDst++; 
        blkCnt2--;   
    }   
}
