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

//STM32�������ͷ�ļ�
#include "stm32f0xx_hal.h"
#include "usart.h"

extern uint8_t ModBusSlave ;         
extern uint8_t ModBusMaster;

/* ----------------------- static functions ---------------------------------*/

/*------------------------------- define ------------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
/**
* @brief  ���ƽ��պͷ���״̬
* @param  xRxEnable ����ʹ�ܡ�
*         xTxEnable ����ʹ��
* @retval None
*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  if(xRxEnable)
  {
    //ʹ�ܽ��պͽ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    //MAX485���� �͵�ƽΪ����ģʽ
//    GPIO_ResetBits(GPIOF,GPIO_Pin_4);
  }
  else
  {
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    //MAX485���� �ߵ�ƽΪ����ģʽ
//    GPIO_SetBits(GPIOF,GPIO_Pin_4);
  }
  
  if(xTxEnable)
  {
    //ʹ�ܷ�������ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
  }
  else
  {
    //��ֹ��������ж�
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
  }
  
}

/**
* @brief  ���ڳ�ʼ��
* @param  ucPORT      ���ں�
*         ulBaudRate  ������
*         ucDataBits  ����λ
*         eParity     У��λ 
* @retval None
*/
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG uBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  return TRUE;
}

/**
* @brief  ͨ�����ڷ�������
* @param  None
* @retval None
*/
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  //��������
  if(HAL_UART_Transmit(&huart1,&ucByte,1,1) != HAL_OK)
		return FALSE;
	else
	  return TRUE;
}

/**
* @brief  �Ӵ��ڻ������
* @param  None
* @retval None
*/
BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  //��������
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
  //mb.c eMBInit������
  //pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM 
  //����״̬��
  pxMBFrameCBTransmitterEmpty();
}

/* Create an interrupt handler for the receive interrupt for your target
* processor. This function should then call pxMBFrameCBByteReceived( ). The
* protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
* character.
*/
void prvvUARTRxISR( void )
{
  //mb.c eMBInit������
  //pxMBFrameCBByteReceived = xMBRTUReceiveFSM
  //����״̬��
  pxMBFrameCBByteReceived();
}






