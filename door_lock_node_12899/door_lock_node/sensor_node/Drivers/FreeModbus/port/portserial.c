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
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //MAX485操作 低电平为接收模式
//    GPIO_ResetBits(GPIOF,GPIO_Pin_4);
  }
  else
  {
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); 
    //MAX485操作 高电平为发送模式
//    GPIO_SetBits(GPIOF,GPIO_Pin_4);
  }
  
  if(xTxEnable)
  {
    //使能发送完成中断
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  }
  else
  {
    //禁止发送完成中断
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
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
  (void)ucPORT;     //不修改串口
  (void)ucDataBits; //不修改数据位长度
  (void)eParity;    //不修改校验格式
  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
 
  
  //使能USART1，GPIOA
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
  
  USART_InitStructure.USART_BaudRate = uBaudRate;            //只修改波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //串口初始化
  USART_Init(USART1, &USART_InitStructure);
  //使能USART1
  USART_Cmd(USART1, ENABLE);
  
  USART_ITConfig(USART1, USART_IT_PE, ENABLE);    //开启PE错误接收中断Bit 8PEIE: PE interrupt enable
  //CR2 开启ERR中断
  USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
  
  NVIC_InitTypeDef NVIC_InitStructure;
  //设定USART1 中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //最后配置485发送和接收模式
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
* @brief  通过串口发送数据
* @param  None
* @retval None
*/
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  //发送数据
  USART_SendData(USART1, ucByte);
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
static void prvvUARTRxISR( void )
{
  //mb.c eMBInit函数中
  //pxMBFrameCBByteReceived = xMBRTUReceiveFSM
  //接收状态机
  pxMBFrameCBByteReceived();
}

extern uint8_t Rx_Modbus_buf[64];
extern uint8_t RxModbuslen;



/**
* @brief  USART1中断服务函数
* @param  None
* @retval None
*/
void USART1_IRQHandler(void)
{
  CHAR  pucByte;
  //发生接收中断
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    prvvUARTRxISR(); 
    //清除中断标志位    
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);   
  }
  
  //发生完成中断
  if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
  {
    prvvUARTTxReadyISR();
    //清除中断标志
    USART_ClearITPendingBit(USART1, USART_IT_TC);
  }
  
  //开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE   错误
  if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
  {//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
    
    pucByte = USART_ReceiveData(USART1); //取出来扔掉
    USART_ClearFlag(USART1, USART_FLAG_ORE);
  }
  
  if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
  {//同  @arg USART_IT_NE     : Noise Error interrupt
    USART_ClearFlag(USART1, USART_FLAG_NE);
  }
  
  
  if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
  {//同   @arg USART_IT_FE     : Framing Error interrupt
    USART_ClearFlag(USART1, USART_FLAG_FE);
  }
  
  if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
  {//同  @arg USART_IT_PE     : Parity Error interrupt
    USART_ClearFlag(USART1, USART_FLAG_PE);
  }
  
  //测试看是否可以去除 2012-07-23
  //溢出-如果发生溢出需要先读SR,再读DR寄存器 则可清除不断入中断的问题
  /*
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
  {
  USART_ClearFlag(USART1,USART_FLAG_ORE); //读SR
  USART_ReceiveData(USART1);              //读DR
}
  */


}





