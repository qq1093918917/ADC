/**
  ******************************************************************************
  * �ļ�����: bsp_uartx.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ���ش��ڵײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_uartx.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef huartx;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{

//  GPIO_InitTypeDef GPIO_InitStruct;
//  if(huart->Instance==UARTx)
//  {
//    /* ��������ʱ��ʹ�� */
//    UART_RCC_CLK_ENABLE();
//  
//    /* �������蹦��GPIO���� */
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
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
//void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
//{

//  if(huart->Instance==UARTx)
//  {
//    /* ��������ʱ�ӽ��� */
//    UART_RCC_CLK_DISABLE();
//  
//    /* �������蹦��GPIO���� */
//    HAL_GPIO_DeInit(UARTx_Tx_GPIO, UARTx_Tx_GPIO_PIN);
//    HAL_GPIO_DeInit(UARTx_Rx_GPIO, UARTx_Rx_GPIO_PIN);
//    
//    /* �����жϽ��� */
//    HAL_NVIC_DisableIRQ(UARTx_IRQn);
//  }
//}

/**
  * ��������: NVIC����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
//static void MX_NVIC_UARTx_Init(void)
//{
//  /* USART1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(UARTx_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(UARTx_IRQn);
//}

///**
//  * ��������: ���ڲ�������.
//  * �������: ��
//  * �� �� ֵ: ��
//  * ˵    ������
//  */
//void MX_UARTx_Init(void)
//{
//  /* ʹ�ܴ��ڹ�������GPIOʱ�� */
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
//  /* ���ô����жϲ�ʹ�ܣ���Ҫ����HAL_UART_Init������ִ���޸Ĳ���Ч */
//  MX_NVIC_UARTx_Init();
//}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
