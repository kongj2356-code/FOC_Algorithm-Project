#ifndef __FOC_MATH_UTILS_H
#define __FOC_MATH_UTILS_H

#include "header.h"
#include "FOC_Struct.h"
#include "Control.h"

#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)

//utility defines
#define PI_2 1.5707963
#define PI_3 1.0471975
#define SQRT3_Half 0.866025388F
#define SQRT3 1.73205080757

#define _2_SQRT3 		(1.154700f)
#define _SQRT3 			(1.732050f)
#define _1_SQRT3 		(0.577350f)
#define _SQRT3_2 		(0.866025f)
#define _SQRT2 			(1.414213f)
#define _120_D2R 		(2.094395f)
#define _PI 			(3.141592f)
#define _PI_2 			(1.570796f)
#define _PI_3 			(1.047197f)
#define _2PI 			(6.283185f)
#define _3PI_2 			(4.712388f)
#define _PI_6 			(0.523598f)
#define _RPM_TO_RADS 	(0.104719f)
#define M_PI 			(3.141592f)

#define NOT_SET -12345.0
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC (NOT_SET)

//normal calculation of sin and cos
float _sin(float a);
float _cos(float a);
float normalize_radianAngle(float angle);
float _normalizeAngle(float angle);
float NormalizeTo360(float angle_degrees);
float ElectriAngle(float shaft_angle, int pole_pairs);

//fast calculation sin and cos
float f1(float x);
float f2(float x);
float fast_sin(float x);
float fast_cos(float x);
void  fast_sin_cos(float x, float *sin_x, float *cos_x);

//filter function
void LP_IaIbIc(Filter_t *p,SVPWM_t* p1);

#endif
