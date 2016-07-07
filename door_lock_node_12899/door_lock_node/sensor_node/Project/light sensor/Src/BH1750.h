#ifndef __BH1750_H_
#define __BH1750_H_

#include "main.h"
#define  MSlaveID          0x04         
#define SLAVE_ID_OFF     0
#define LIGHT_VALUE_OFF  1
//IIC的从机地址，根据手册ADDRESS地址随ADDR引脚的电平高低而变，
//ALT  ADDRESS引脚接地时地址为0x46，接电源时地址为0xB8
#define MMA_ADRESS	 		 0x46  /*MMA_Device Address*/

enum {IN,OUT};


#define FALSE 0
#define TRUE 1

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


/**********
**功能代码
**********/
#define         BH1750_ADDR_POWER_Down          0x00    //断电
#define         BH1750_ADDR_POWER_ON            0x01    //通电
#define         BH1750_ADDR_Reset               0x07    //复位

#define         BH1750_ADDR_Cont_H_Mode         0x10    //连续H分辨率模式
#define         BH1750_ADDR_Cont_H_Mode2        0x11    //连续H分辨率率模式2
#define         BH1750_ADDR_Cont_L_Mode         0x13    //连续L分辨率模式

#define         BH1750_ADDR_One_H_Mode          0x20    //一次H分辨率模式
#define         BH1750_ADDR_One_H_Mode2         0x21    //一次分辨率模式2
#define         BH1750_ADDR_One_L_Mode          0x23    //一次L分辨率模式



void  sensor_init(void);
void restore_data();
void get_sensor_data(void);  


#endif

