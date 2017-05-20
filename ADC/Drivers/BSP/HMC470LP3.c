#include "HMC470LP3.h"
#include "key.h"
#include "lcd.h"
#include "sys.h"
u8 input;
void HMC470LP3_RUN ()
{
	if(KEY0==0)
	{
		HAL_Delay(5);
		if(KEY0==0)
		{
			input = 0;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
			while(KEY0==0);
		}
	}
	if(KEY1==0)
	{
		HAL_Delay(5);
		if(KEY1==0)
		{
			input=input+5;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			while(KEY1==0);
		}
	}
	if(KEY2==0)
	{
		HAL_Delay(5);
		if(KEY2==0)
		{
			input=input+1;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
			while(KEY2==0);
		}
	}
	if(input > 31)
	{
		input = 0;
	}
	if (input>15&&input<32) V1out(0);
	else V1out(1);
	if (input&0x08) V2out(0);
	else V2out(1);
	if (input&0x04) V3out(0);
	else V3out(1);
	if	(input&0x02) V4out(0);
	else V4out(1);
	if	(input&0x01) V5out(0);
	else V5out(1);
	LCD_ShowNum(100,260,input,3,32);
	LCD_ShowString(148,260,32,32,32,"dBBBB");
}