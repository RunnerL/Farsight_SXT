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
#include "main.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

extern uint8_t ModBusSlave ;         
extern uint8_t ModBusMaster;

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

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
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //MAX485���� �͵�ƽΪ����ģʽ
//    GPIO_ResetBits(GPIOF,GPIO_Pin_4);
  }
  else
  {
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); 
    //MAX485���� �ߵ�ƽΪ����ģʽ
//    GPIO_SetBits(GPIOF,GPIO_Pin_4);
  }
  
  if(xTxEnable)
  {
    //ʹ�ܷ�������ж�
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  }
  else
  {
    //��ֹ��������ж�
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
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
  (void)ucPORT;     //���޸Ĵ���
  (void)ucDataBits; //���޸�����λ����
  (void)eParity;    //���޸�У���ʽ
  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
 
  
  //ʹ��USART1��GPIOA
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  //GPIOA.2 USART1_Tx,GPIOA.3 USART1_Rx
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = uBaudRate;            //ֻ�޸Ĳ�����
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //���ڳ�ʼ��
  USART_Init(USART1, &USART_InitStructure);
  //ʹ��USART1
  USART_Cmd(USART1, ENABLE);
  
  USART_ITConfig(USART1, USART_IT_PE, ENABLE);    //����PE��������ж�Bit 8PEIE: PE interrupt enable
  //CR2 ����ERR�ж�
  USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
  
  NVIC_InitTypeDef NVIC_InitStructure;
  //�趨USART1 �ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������485���ͺͽ���ģʽ
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  //GPIOb.0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
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
  USART_SendData(USART1, ucByte);
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
  *pucByte = USART_ReceiveData(USART1);
  return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
* (or an equivalent) for your target processor. This function should then
* call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
* a new character can be sent. The protocol stack will then call 
* xMBPortSerialPutByte( ) to send the character.
*/
static void prvvUARTTxReadyISR( void )
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
static void prvvUARTRxISR( void )
{
  //mb.c eMBInit������
  //pxMBFrameCBByteReceived = xMBRTUReceiveFSM
  //����״̬��
  pxMBFrameCBByteReceived();
}

extern uint8_t Rx_Modbus_buf[64];
extern uint8_t RxModbuslen;



/**
* @brief  USART1�жϷ�����
* @param  None
* @retval None
*/
void USART1_IRQHandler(void)
{
  CHAR  pucByte;
  //���������ж�
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    prvvUARTRxISR(); 
    //����жϱ�־λ    
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);   
  }
  
  //��������ж�
  if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
  {
    prvvUARTTxReadyISR();
    //����жϱ�־
    USART_ClearITPendingBit(USART1, USART_IT_TC);
  }
  
  //����CR3,bit0��EIE: Error interrupt enable, ����USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE   ����
  if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
  {//ͬ  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
    
    pucByte = USART_ReceiveData(USART1); //ȡ�����ӵ�
    USART_ClearFlag(USART1, USART_FLAG_ORE);
  }
  
  if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
  {//ͬ  @arg USART_IT_NE     : Noise Error interrupt
    USART_ClearFlag(USART1, USART_FLAG_NE);
  }
  
  
  if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
  {//ͬ   @arg USART_IT_FE     : Framing Error interrupt
    USART_ClearFlag(USART1, USART_FLAG_FE);
  }
  
  if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
  {//ͬ  @arg USART_IT_PE     : Parity Error interrupt
    USART_ClearFlag(USART1, USART_FLAG_PE);
  }
  
  //���Կ��Ƿ����ȥ�� 2012-07-23
  //���-������������Ҫ�ȶ�SR,�ٶ�DR�Ĵ��� �������������жϵ�����
  /*
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
  {
  USART_ClearFlag(USART1,USART_FLAG_ORE); //��SR
  USART_ReceiveData(USART1);              //��DR
}
  */


}





