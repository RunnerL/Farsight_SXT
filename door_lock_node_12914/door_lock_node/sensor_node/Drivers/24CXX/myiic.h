#ifndef __MYIIC_H
#define __MYIIC_H
#include "stm32f0xx_gpio.h"
#include "main.h"
void IIC_Init(void);
uint8_t I2C_Write_one_Byte(uint8_t SendByte,uint8_t DeviceAddress , uint16_t WriteAddress);
uint8_t I2C_Read_one_Byte(uint8_t *ReadByte, uint8_t DeviceAddress,uint16_t WriteAddress );
void AT24CXX_Read(uint8_t ReadAddr,uint8_t *pBuffer,uint8_t NumToRead);
void AT24CXX_Write(uint8_t WriteAddr,uint8_t *pBuffer,uint8_t NumToWrite);
#endif
















