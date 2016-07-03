#include "FS_FIFO_V1.h"
#include "stdlib.h"
/*
*  初始化队列
*
**/
void QueueInit(struct FIFOqueue *Queue)
{
	Queue->rear = 0;
	Queue->front = Queue->rear;									//初始化，队列头尾相连
	Queue->count = 0;														//队列计数器清零；
	memset(Queue->dat, 0, sizeof(Queue->dat));
}
/*
*  入队
*
**/
uint8_t QueueIn(struct FIFOqueue *Queue,uint8_t sdata) //存数据；
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
* 出队函数
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
		*sdata = Queue->dat[Queue->front];                 //将出队的数据放入传入指针对应的地址
		Queue->front = (Queue->front + 1) % FIFO_SIZE;
		Queue->count -= 1;
		return FIFOR_OK;
	}
}
