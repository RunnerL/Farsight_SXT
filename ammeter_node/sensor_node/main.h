/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-May-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stdio.h>
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"   
#include "myiic.h"

/*****************Modbus相关****************/
//modbus从机地址
#define MSlaveID  0x01
//modbus用到的串口
#define MODBUS_UART_NUMBER    1
#define MODBUS_UART_BAUDRATE  9600
//保持寄存器起始地址
#define REG_HOLDING_START     0
//保持寄存器数量
#define REG_HOLDING_NREGS     2
#if defined (DEBUG)
#define DEBUG_UART USART2
#endif

#define BEEP_PORT   GPIOC
#define BEEP_PIN   GPIO_Pin_13

#define GET_DATA_INTEVAL 1000    //单位是ms



#define BEEP_OFF()          GPIO_ResetBits(BEEP_PORT,BEEP_PIN)
#define BEEP_ON()           GPIO_SetBits(BEEP_PORT,BEEP_PIN)
#define BEEP_TOGGLE()      if (GPIO_ReadOutputDataBit(BEEP_PORT, BEEP_PIN)) BEEP_OFF(); else BEEP_ON()
extern  volatile uint8_t get_data_flag;
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
/* Private macro -------------------------------------------------------------*/


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
