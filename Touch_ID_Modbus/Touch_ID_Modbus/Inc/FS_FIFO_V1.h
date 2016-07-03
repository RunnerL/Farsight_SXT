#ifndef __FS_FIFO_V1_H
#define __FS_FIFO_V1_H

#include<stdio.h>
#include "string.h"
#include "stm32f0xx_hal.h"


#define FIFO_SIZE 	64 			
#define FIFOFULL 		0				
#define FIFOEmpty 	1			
#define FIFOR_OK 		2 			

//����ָ��ģ����Ҫ�ϱ�modbus����������
#define  FINGER_NOT_FIND  	0x0000    //ָ��ʶ��ʧ��
#define  FINGER_FOUND     	0x0001    //ָ��ʶ��ɹ�
#define  FINGER_WRITE_OK  	0x0002    //ָ��¼��ɹ�
#define  FINGER_WRITE_FAILD 0x0003    //ָ��¼��ʧ��
#define  NO_FINGER        	0x0004    //û�м�⵽ָ��

//����modbus�Ĵ������뿪ʼ��ַ0x00���Ĵ�������
#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 1

//modbus�ӻ���ַ
#define MSlaveID 0x0b

//modbus�õ��Ĵ���
#define MODBUS_UART_NUMBER 1
#define MODBUS_UART_BAUD   115200

struct FIFOqueue 
{
	uint8_t front;       			
	uint8_t rear;							
	uint8_t count;						
	uint8_t dat[FIFO_SIZE];	
};

void QueueInit(struct FIFOqueue *Queue);									
uint8_t QueueIn(struct FIFOqueue *Queue,uint8_t sdata);			
uint8_t QueueOut(struct FIFOqueue *Queue,uint8_t *sdata);		
#endif /*FS_FIFO_V1.h*/



