#ifndef __FS_FIFO_V1_H
#define __FS_FIFO_V1_H

#include<stdio.h>
#include "string.h"
#include "stm32f0xx.h"
//#include "stm32f10x.h"

#define FIFO_SIZE 	64 			//每个缓冲区的大小
#define FIFOFULL 		0				//缓冲区数据满
#define FIFOEmpty 	1				//缓冲区里面没有数据
#define FIFOR_OK 		2 			//缓冲区存取数据操作成功，

struct FIFOqueue 
{
	uint8_t front;       			//存储当前的数据的位置
	uint8_t rear;							//指向下一个指针
	uint8_t count;						//当前有效数据个数
	uint8_t dat[FIFO_SIZE];		//数据内容
};

void QueueInit(struct FIFOqueue *Queue);										//队列初始化；
uint8_t QueueIn(struct FIFOqueue *Queue,uint8_t sdata);			//存入数据
uint8_t QueueOut(struct FIFOqueue *Queue,uint8_t *sdata);		//取数据
#endif /*FS_FIFO_V1.h*/



