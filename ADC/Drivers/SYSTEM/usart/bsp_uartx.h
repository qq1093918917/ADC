#ifndef __BSP_UARTX_H__
#define __BSP_UARTX_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define UARTx                                 UART4
#define UARTx_BAUDRATE                        115200
#define UART_RCC_CLK_ENABLE()                 __HAL_RCC_UART4_CLK_ENABLE()
#define UART_RCC_CLK_DISABLE()                __HAL_RCC_UART4_CLK_DISABLE()

#define UARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define UARTx_Tx_GPIO_PIN                     GPIO_PIN_10
#define UARTx_Tx_GPIO                         GPIOC
#define UARTx_Rx_GPIO_PIN                     GPIO_PIN_11
#define UARTx_Rx_GPIO                         GPIOC

#define UARTx_IRQHANDLER                      UART4_IRQHandler
#define UARTx_IRQn                            UART4_IRQn


/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef huartx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_UARTx_Init(void);


#endif  /* __BSP_UARTX_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
