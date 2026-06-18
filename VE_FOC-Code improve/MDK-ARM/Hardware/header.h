#ifndef __HEADER_H
#define __HEADER_H


#include "RTE_Components.h"      // Component selection

#include "main.h"
#include "stdio.h"
#include <stdlib.h>
#include "stdint.h"
#include "gpio.h"       //gpio需要先进行初始化
#include <math.h>		   //后面的adc，usart，tim，spi，i2c才能用到
#include "adc.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
#include "dma.h"

#include "arm_math.h"

//按键检测
#include "lwbtn.h"
#include "lwbtn_opt.h"
#include "lwbtn_opts.h"

#endif
