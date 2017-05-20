/**
  ******************************************************************************
  * 文件名程: bsp_uartx.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: 板载串口底层驱动程序
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_uartx.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
UART_HandleTypeDef huartx;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 串口硬件初始化配置
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{

//  GPIO_InitTypeDef GPIO_InitStruct;
//  if(huart->Instance==UARTx)
//  {
//    /* 串口外设时钟使能 */
//    UART_RCC_CLK_ENABLE();
//  
//    /* 串口外设功能GPIO配置 */
//    GPIO_InitStruct.Pin = UARTx_Tx_GPIO_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(UARTx_Tx_GPIO, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = UARTx_Rx_GPIO_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(UARTx_Rx_GPIO, &GPIO_InitStruct);
//  }
//}

/**
  * 函数功能: 串口硬件反初始化配置
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
//void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
//{

//  if(huart->Instance==UARTx)
//  {
//    /* 串口外设时钟禁用 */
//    UART_RCC_CLK_DISABLE();
//  
//    /* 串口外设功能GPIO配置 */
//    HAL_GPIO_DeInit(UARTx_Tx_GPIO, UARTx_Tx_GPIO_PIN);
//    HAL_GPIO_DeInit(UARTx_Rx_GPIO, UARTx_Rx_GPIO_PIN);
//    
//    /* 串口中断禁用 */
//    HAL_NVIC_DisableIRQ(UARTx_IRQn);
//  }
//}

/**
  * 函数功能: NVIC配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
//static void MX_NVIC_UARTx_Init(void)
//{
//  /* USART1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(UARTx_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(UARTx_IRQn);
//}

///**
//  * 函数功能: 串口参数配置.
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明：无
//  */
//void MX_UARTx_Init(void)
//{
//  /* 使能串口功能引脚GPIO时钟 */
//  UARTx_GPIO_ClK_ENABLE();
//  
//  huartx.Instance = UARTx;
//  huartx.Init.BaudRate = UARTx_BAUDRATE;
//  huartx.Init.WordLength = UART_WORDLENGTH_8B;
//  huartx.Init.StopBits = UART_STOPBITS_1;
//  huartx.Init.Parity = UART_PARITY_NONE;
//  huartx.Init.Mode = UART_MODE_TX_RX;
//  huartx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huartx.Init.OverSampling = UART_OVERSAMPLING_16;
//  HAL_UART_Init(&huartx);
//  
//  /* 配置串口中断并使能，需要放在HAL_UART_Init函数后执行修改才有效 */
//  MX_NVIC_UARTx_Init();
//}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
