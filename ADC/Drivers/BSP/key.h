#ifndef _KEY_H
#define _KEY_H
#include "stm32f4xx_hal.h"
#define KEY0 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)
#define KEY1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)
#define KEY2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)

#endif