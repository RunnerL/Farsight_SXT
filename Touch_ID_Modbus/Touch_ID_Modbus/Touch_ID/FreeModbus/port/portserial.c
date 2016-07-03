/*
* FreeModbus Libary: BARE Port
* Copyright (C) 2006 Christian Walter <wolti@sil.at>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
*/

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//STM32操作相关头文件
#include "stm32f0xx_hal.h"
#include "usart.h"

extern uint8_t ModBusSlave ;         
extern uint8_t ModBusMaster;

/* ----------------------- static functions ---------------------------------*/

/*------------------------------- define ------------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
/**
* @brief  控制接收和发送状态
* @param  xRxEnable 接收使能、
*         xTxEnable 发送使能
* @retval None
*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  if(xRxEnable)
  {
    //使能接收和接收中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    //MAX485操作 低电平为接收模式
//    GPIO_ResetBits(GPIOF,GPIO_Pin_4);
  }
  else
  {
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    //MAX485操作 高电平为发送模式
//    GPIO_SetBits(GPIOF,GPIO_Pin_4);
  }
  
  if(xTxEnable)
  {
    //使能发送完成中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
  }
  else
  {
    //禁止发送完成中断
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
  }
  
}

/**
* @brief  串口初始化
* @param  ucPORT      串口号
*         ulBaudRate  波特率
*         ucDataBits  数据位
*         eParity     校验位 
* @retval None
*/
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG uBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  return TRUE;
}

/**
* @brief  通过串口发送数据
* @param  None
* @retval None
*/
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  //发送数据
  if(HAL_UART_Transmit(&huart1,&ucByte,1,1) != HAL_OK)
		return FALSE;
	else
	  return TRUE;
}

/**
* @brief  从串口获得数据
* @param  None
* @retval None
*/
BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  //接收数据
  if(HAL_UART_Receive(&huart1,(uint8_t *)pucByte,1,1) != HAL_OK)
		return FALSE;
	else
		return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
* (or an equivalent) for your target processor. This function should then
* call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
* a new character can be sent. The protocol stack will then call 
* xMBPortSerialPutByte( ) to send the character.
*/
void prvvUARTTxReadyISR( void )
{
  //mb.c eMBInit函数中
  //pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM 
  //发送状态机
  pxMBFrameCBTransmitterEmpty();
}

/* Create an interrupt handler for the receive interrupt for your target
* processor. This function should then call pxMBFrameCBByteReceived( ). The
* protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
* character.
*/
void prvvUARTRxISR( void )
{
  //mb.c eMBInit函数中
  //pxMBFrameCBByteReceived = xMBRTUReceiveFSM
  //接收状态机
  pxMBFrameCBByteReceived();
}






