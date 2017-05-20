#ifndef __BSP_UARTX_H__
#define __BSP_UARTX_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
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


/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef huartx;

/* �������� ------------------------------------------------------------------*/
void MX_UARTx_Init(void);


#endif  /* __BSP_UARTX_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
