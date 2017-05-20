#include "adc.h"
#include "delay.h"
u16 Get_Adc(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Start(hadc);                               //¿ªÆôADC
//	delay_ms(50);
   HAL_ADC_PollForConversion(hadc,10); 
//	delay_ms(50);
	return (u16)HAL_ADC_GetValue(hadc);
}
u16 Get_Adc_Average(ADC_HandleTypeDef* hadc,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(hadc);
		delay_ms(5);
	}
	return temp_val/times;
} 