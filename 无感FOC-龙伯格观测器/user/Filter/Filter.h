#ifndef Filter_H  
#define Filter_H  
 
#include "tim.h"
#include "main.h"
#include "math.h"
#include "stdlib.h" 
#include "stdio.h"

 typedef struct
{
 
float data;
float last_data;
float  alpha;
 
}filter_kind;
 
extern filter_kind iq_filter; 
extern filter_kind id_filter; 
 
extern filter_kind we_fore_filter;
 
extern filter_kind Luenberger_VAlpha_filter ;
extern filter_kind Luenberger_VBeta_filter; 

float Filter_func(filter_kind * filter,float sth );

 #endif

