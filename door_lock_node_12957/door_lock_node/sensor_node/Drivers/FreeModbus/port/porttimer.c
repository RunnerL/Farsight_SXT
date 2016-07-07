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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//STM32���ͷ�ļ�
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

static volatile uint32_t tim3_cnt;

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );



/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief  ��ʱ����ʼ������
  * @param  None
  * @retval None
  */
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //
  uint16_t PrescalerValue = 0;
  
  //ʹ�ܶ�ʱ��3ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  //��ʱ��ʱ�������˵��
  //HCLKΪ48MHz��APB����ƵΪ48MHz
  //TIM3�Ĺ�������Ƶ��Ϊ20KHz��Ԥ��ƵֵΪ48M/20K-1
  //TIM3���ڲ���Ƶ������Ƶ
  //TIM������ֵΪusTim1Timerout50u
  PrescalerValue = (uint16_t) ((SystemCoreClock / 20000) - 1);    // TIM3����Ƶ��
  // ����TIM3����Ƶ��
  //TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
  //��ʱ��1��ʼ��
  TIM_TimeBaseStructure.TIM_Period = (uint16_t) usTim1Timerout50us;
  //tim3_cnt=usTim1Timerout50us;
  //TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  //Ԥװ��ʹ��
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  //��ʱ��3�ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������жϱ�־λ
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
  //��ʱ��3����жϹر�
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  //��ʱ��3����
  TIM_Cmd(TIM3,  DISABLE);
  return TRUE;
}


void
vMBPortTimersEnable(  )
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  //�趨��ʱ��3�ĳ�ʼֵ
  TIM_SetCounter(TIM3,0x0000); 
  //��ʱ��3����
  TIM_Cmd(TIM3, ENABLE);
}

void
vMBPortTimersDisable(  )
{
   /* Disable any pending timers. */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  TIM_SetCounter(TIM3,0x0000); 
  //�رն�ʱ��3
  TIM_Cmd(TIM3, DISABLE);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired();
}

/**
  * @brief  ��ʱ��3�жϷ�����
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    tim3_cnt = TIM3->CNT;
    //�����ʱ��T3����жϱ�־λ
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    prvvTIMERExpiredISR( );
  }
}