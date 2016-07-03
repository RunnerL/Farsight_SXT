#include "infrared_sensor.h"

void  sensor_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct; 
  /* Enable  GPIOB clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  /*!< GPIO configuration */  
  GPIO_InitStruct.GPIO_Pin = INFRARED_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			
  GPIO_Init(INFRARED_PORT , &GPIO_InitStruct); 
}

void restore_data()
{
  //modbus从机地址
  
  
  usRegHoldingBuf[SLAVE_ID_OFF] = MSlaveID;
  usRegHoldingBuf[STATUS_OFF] = GPIO_ReadInputDataBit(INFRARED_PORT, INFRARED_PIN);
}

void get_sensor_data(void)
{
  //将实时数据放入保持寄存器
  usRegHoldingBuf[STATUS_OFF] = GPIO_ReadInputDataBit(INFRARED_PORT, INFRARED_PIN) ? 0 : 1;
}