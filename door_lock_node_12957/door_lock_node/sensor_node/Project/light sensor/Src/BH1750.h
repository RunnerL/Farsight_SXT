#ifndef __BH1750_H_
#define __BH1750_H_

#include "main.h"
#define  MSlaveID          0x04         
#define SLAVE_ID_OFF     0
#define LIGHT_VALUE_OFF  1
//IIC�Ĵӻ���ַ�������ֲ�ADDRESS��ַ��ADDR���ŵĵ�ƽ�ߵͶ��䣬
//ALT  ADDRESS���Žӵ�ʱ��ַΪ0x46���ӵ�Դʱ��ַΪ0xB8
#define MMA_ADRESS	 		 0x46  /*MMA_Device Address*/

enum {IN,OUT};


#define FALSE 0
#define TRUE 1

//PB8 ʱ��
#define SCK_H         GPIOB->BSRR = GPIO_Pin_8    //PB8���øߵ�ƽ
#define SCK_L         GPIOB->BRR  = GPIO_Pin_8    //PB8���õ͵�ƽ
//PB9 ����   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_9    //PB9���øߵ�ƽ      
#define SDA_L         GPIOB->BRR  = GPIO_Pin_9    //PB9���õ͵�ƽ


#define SCK_read      GPIOB->IDR  & GPIO_Pin_8
#define SDA_read      GPIOB->IDR  & GPIO_Pin_9

#define SDA_IN       GPIOB->MODER = GPIOB->MODER & ~GPIO_Pin_9
#define SDA_OUT      GPIOB->MODER = GPIOB->MODER | GPIO_Pin_9


/**********
**���ܴ���
**********/
#define         BH1750_ADDR_POWER_Down          0x00    //�ϵ�
#define         BH1750_ADDR_POWER_ON            0x01    //ͨ��
#define         BH1750_ADDR_Reset               0x07    //��λ

#define         BH1750_ADDR_Cont_H_Mode         0x10    //����H�ֱ���ģʽ
#define         BH1750_ADDR_Cont_H_Mode2        0x11    //����H�ֱ�����ģʽ2
#define         BH1750_ADDR_Cont_L_Mode         0x13    //����L�ֱ���ģʽ

#define         BH1750_ADDR_One_H_Mode          0x20    //һ��H�ֱ���ģʽ
#define         BH1750_ADDR_One_H_Mode2         0x21    //һ�ηֱ���ģʽ2
#define         BH1750_ADDR_One_L_Mode          0x23    //һ��L�ֱ���ģʽ



void  sensor_init(void);
void restore_data();
void get_sensor_data(void);  


#endif

