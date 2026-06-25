#ifndef _SineTable_h_
#define _SineTable_h_

/* constants */
#define TableSize	8193
#define pi 			3.141592653f
#define TWO_PI 		6.283185306f
#define HALF_PI 	1.5707963265f

/* the LUT */
extern const float sinetable[8193];

/* public f(x) */
extern float lookup_sin (float x);
extern float lookup_cos (float x);
extern float lookup_tan (float x);
extern float lookup_cot (float x);

#endif	// _SineTable_h_

