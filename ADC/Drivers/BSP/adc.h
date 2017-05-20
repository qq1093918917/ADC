#ifndef _adc_h
#define _adc_h
#include "sys.h"
#include "stm32f4xx_hal.h"
u16 Get_Adc(ADC_HandleTypeDef* hadc);
u16 Get_Adc_Average(ADC_HandleTypeDef* hadc,u8 times);
#endif