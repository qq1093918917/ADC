#ifndef _HMC470LP3_H
#define _HMC470LP3_H
#include "sys.h"
#define V5out(x) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,x)
#define V4out(x) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,x)
#define V3out(x) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,x)
#define V2out(x) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,x)
#define V1out(x) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,x)
extern u8 input;
void HMC470LP3_RUN ();
#endif