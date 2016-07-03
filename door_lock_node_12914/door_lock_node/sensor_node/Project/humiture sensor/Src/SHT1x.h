#ifndef  __SHT1x_H
#define  __SHT1x_H

/*********************************************
***  INCLUDE
**********************************************/
#include "main.h"
#define  MSlaveID          0x06 
#define SLAVE_ID_OFF 0
#define TEMPERATURE_DATA_OFF 1
#define HUMIDITY_DATA_OFF 2
#define FALSE   0
#define TRUE    1

#define STATUS_REG_W 0x06       //000 0011 0
#define STATUS_REG_R 0x07       //000 0011 1
#define MEASURE_TEMP 0x03       //0x0011        //温度测量
#define MEASURE_HUMI 0x05       //0x0101        //湿度测量

//PB8 时钟
#define SCK_H         GPIOB->BSRR = GPIO_Pin_8    //PB8设置高电平
#define SCK_L         GPIOB->BRR  = GPIO_Pin_8    //PB8设置低电平
//PB9 数据   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_9    //PB9设置高电平      
#define SDA_L         GPIOB->BRR  = GPIO_Pin_9    //PB9设置低电平


#define SCK_read      GPIOB->IDR  & GPIO_Pin_8
#define SDA_read      GPIOB->IDR  & GPIO_Pin_9

#define SDA_IN       GPIOB->MODER = GPIOB->MODER & ~GPIO_Pin_9
#define SDA_OUT      GPIOB->MODER = GPIOB->MODER | GPIO_Pin_9

#define noACK 0
#define ACK 1



enum {TEMP,HUMI};

typedef union{
  uint32_t i;
  float f;
}Value;
  
/*********************************************
***  FUNCTION
**********************************************/

void  sensor_init(void);
void restore_data();
void get_sensor_data(void);
#endif

