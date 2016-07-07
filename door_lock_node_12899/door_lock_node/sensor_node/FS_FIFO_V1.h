#ifndef __FS_FIFO_V1_H
#define __FS_FIFO_V1_H

#include<stdio.h>
#include "string.h"
#include "stm32f0xx.h"
//#include "stm32f10x.h"

#define FIFO_SIZE 	64 			//ÿ���������Ĵ�С
#define FIFOFULL 		0				//������������
#define FIFOEmpty 	1				//����������û������
#define FIFOR_OK 		2 			//��������ȡ���ݲ����ɹ���

struct FIFOqueue 
{
	uint8_t front;       			//�洢��ǰ�����ݵ�λ��
	uint8_t rear;							//ָ����һ��ָ��
	uint8_t count;						//��ǰ��Ч���ݸ���
	uint8_t dat[FIFO_SIZE];		//��������
};

void QueueInit(struct FIFOqueue *Queue);										//���г�ʼ����
uint8_t QueueIn(struct FIFOqueue *Queue,uint8_t sdata);			//��������
uint8_t QueueOut(struct FIFOqueue *Queue,uint8_t *sdata);		//ȡ����
#endif /*FS_FIFO_V1.h*/



