#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2015/6/23
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.0�޸�˵�� 
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern UART_HandleTypeDef huart1; //UART���
#define Receive_All_Flag (USART_RX_STA&0x8000)
#define Receive_Len (USART_RX_STA&0x3fff)
#define RXBUFFERSIZE   1 //�����С
#define Wait_Transmit_Finish while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET)
#define Wait_Receive_Finish while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=SET)
#define HAL_UART_Receive_IT_Open(x) HAL_UART_Receive_IT(&x,aRxBuffer,RXBUFFERSIZE)
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void USART1_Data_Real(u8 buf[],u8 len);

#endif
