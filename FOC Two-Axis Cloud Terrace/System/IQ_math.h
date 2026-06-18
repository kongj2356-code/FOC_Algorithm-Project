//############################################################
// FILE: IQ_math.h
// Created on: 2017年1月18日
// Author: XQ
// summary: IQ_math_ 
//############################################################
   
#ifndef _IQ_math_H
#define _IQ_math_H 

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 角度映射宏
#define _2PI_      65536U    // 2π 对应 65536
#define _PI_2_     16384U    // π/2

float _sin(float a);
float _cos(float a);
void _sincos(float a, float* s, float* c);

#ifdef __cplusplus
}
#endif


#endif /* __IQ_math_H */

//===========================================================================
// No more.
//===========================================================================

