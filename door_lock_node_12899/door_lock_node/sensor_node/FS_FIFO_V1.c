#include "FS_FIFO_V1.h"
#include "stdlib.h"
/*
*  ��ʼ������
*
**/
void QueueInit(struct FIFOqueue *Queue)
{
	Queue->rear = 0;
	Queue->front = Queue->rear;									//��ʼ��������ͷβ����
	Queue->count = 0;														//���м��������㣻
	memset(Queue->dat, 0, sizeof(Queue->dat));
}
/*
*  ���
*
**/
uint8_t QueueIn(struct FIFOqueue *Queue,uint8_t sdata) //�����ݣ�
{
	if((Queue->front == Queue->rear) && (Queue->count == FIFO_SIZE))
	{
		return FIFOFULL;
	}
	else
	{
		Queue->dat[Queue->rear] = sdata;
		Queue->rear = (Queue->rear + 1) % FIFO_SIZE;
		Queue->count += 1;
		return FIFOR_OK;
	}
}
/*
* ���Ӻ���
*
**/
uint8_t QueueOut(struct FIFOqueue *Queue, uint8_t *sdata)
{
	if((Queue->front == Queue->rear) && (Queue->count == 0))
	{
		return FIFOEmpty;
	}
	else
	{
		*sdata = Queue->dat[Queue->front];                 //�����ӵ����ݷ��봫��ָ���Ӧ�ĵ�ַ
		Queue->front = (Queue->front + 1) % FIFO_SIZE;
		Queue->count -= 1;
		return FIFOR_OK;
	}
}
