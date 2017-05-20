#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.csom
//修改日期:2015/6/23
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.0修改说明 
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef huart1; //UART句柄
#define Receive_All_Flag (USART_RX_STA&0x8000)
#define Receive_Len (USART_RX_STA&0x3fff)
#define RXBUFFERSIZE   1 //缓存大小
#define Wait_Transmit_Finish while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET)
#define Wait_Receive_Finish while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=SET)
#define HAL_UART_Receive_IT_Open(x) HAL_UART_Receive_IT(&x,aRxBuffer,RXBUFFERSIZE)
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void USART1_Data_Real(u8 buf[],u8 len);

#endif
