#include "FPM_Finger.h"
#include "usart.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdarg.h"
#include "lcd.h"
#include "logo.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_flash_ex.h"
#include "FS_FIFO_V1.h"

extern uint8_t uart2_recv_buf[128];
extern uint8_t dma_state;
extern uint8_t UART_SEND_FLAG;
extern volatile uint16_t FingerCount;
extern unsigned char Show_find_man_num[128];
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

_SYS_CMD_PACK FPM_CMD;       //ϵͳָ��ʵ��
uint8_t FPM_Data[128] = {0}; //���ڴ��ָ���������ֵ
uint8_t UART_FPM_CMD[256];   //���ڴ��ָ����洮�ڷ���
uint8_t fcmd_length;
uint16_t Person_num = 0;     //���ڱ��ָ��

//FINGERPRINTͨ��Э�鶨��
unsigned char FP_Get_Img[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x01,0x00,0x05};    //���ָ��ͼ��
unsigned char FP_Templete_Num[12] ={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x1D,0x00,0x21 }; //���ģ������
unsigned char FP_Search[17]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x0,0x08,0x04,0x01,0x0,0x0,0x03,0xA1,0x0,0xB2}; //����ָ��������Χ0 - 929
unsigned char FP_Search_0_9[17]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x0,0x08,0x04,0x01,0x0,0x0,0x0,0x13,0x0,0x21}; //����0-9��ָ��
unsigned char FP_Img_To_Buffer1[13]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x0,0x04,0x02,0x01,0x0,0x08}; //��ͼ����뵽BUFFER1
unsigned char FP_Img_To_Buffer2[13]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x0,0x04,0x02,0x02,0x0,0x09}; //��ͼ����뵽BUFFER2
unsigned char FP_Reg_Model[12]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x05,0x00,0x09}; //��BUFFER1��BUFFER2�ϳ�����ģ��
unsigned char FP_Delet_All_Model[12]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x0,0x03,0x0d,0x00,0x11};//ɾ��ָ��ģ�������е�ģ��
volatile unsigned char  FP_Save_Finger[15]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x06,0x06,0x01,0x00,0x0B,0x00,0x19};//��BUFFER1�е��������ŵ�ָ����λ��
volatile unsigned char FP_Delete_Model[16]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x0C,0x0,0x0,0x0,0x1,0x0,0x0}; //ɾ��ָ����ģ��
unsigned char FP_Match_Model[12]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x03,0x00,0x07};//ģ�龫ȷ�ȶԣ�1:1��CharBuffer1 ��CharBuffer2 �е������ļ�
unsigned char FP_LoadChar_Model[15]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x06,0x07,0x01,0x00,0x03,0x00,0x07};//��flash ���ݿ���ָ��ID �ŵ�ָ��ģ����뵽ģ�建����CharBuffer1 ��CharBuffer2

/*
*brif:�Զ����ӳٺ���
*/
void FingerDelay(uint32_t nTime)
{
	uint16_t i,j;
	for(i=0;i<nTime;i++)
	{
		for(j=0;j<10000;j++)
		{}
	}
}

/*
*brif:������״̬��ʾ
*
*/
void TIP_SUCCESS(void) //�ɹ�ʶ��ָ��  ����1��
{
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(25);
	GPIOA->BSRR = GPIO_PIN_0;
}

void TIP_FAILED(void) //ָ��ʶ��ʧ��  ����1��
{
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(70);
	GPIOA->BSRR = GPIO_PIN_0;
}

void TIP_WRITE_SUCCESS(void) //ָ��¼��ɹ� ���� 2��
{
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(25);
	GPIOA->BSRR = GPIO_PIN_0;
	FingerDelay(35);
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(25);
	GPIOA->BSRR = GPIO_PIN_0;
}

void TIP_WRITE_FAILED(void) //ָ��¼��ʧ��   ����1�� ����1��
{
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(25);
	GPIOA->BSRR = GPIO_PIN_0;
	FingerDelay(35);
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(70);
	GPIOA->BSRR = GPIO_PIN_0;
}

void TIP_DELATE_ALL(void)  //ɾ������ָ��ģ�� ����1��  ����1��
{
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(70);
	GPIOA->BSRR = GPIO_PIN_0;	
	FingerDelay(35);
	GPIOA->BRR = GPIO_PIN_0;
	FingerDelay(25);
	GPIOA->BSRR = GPIO_PIN_0;	
}


/***************************
*brif:����2���������
*para:*command  �����ַ
*     num       �����
*return : 0     �ɹ�
*         1     ʧ��
****************************/
uint8_t Commands_Send(uint8_t *command, uint8_t num)
{
	HAL_UART_Transmit_IT(&huart2,command,num);
	FingerDelay(30);
	if(UART_SEND_FLAG == 1)
	{
		UART_SEND_FLAG = 0;
		return 0;
	}
	else
	{
		return 1;
	}
}
/********************************
*brief:��ȡϵͳָ��
*para:pid �����ʶ��
*     *data ����������ָ��
*     data_length ���������򳤶�
*return: ���ظ�ָ����ܳ���
*********************************/
uint8_t Get_FPM_CMD(uint8_t pid, uint8_t *data, uint8_t data_length)
{
	uint8_t i = 0, temp = 0;
	FPM_CMD.Head[0] = 0xEF;
	FPM_CMD.Head[1] = 0x01;
	UART_FPM_CMD[0] = FPM_CMD.Head[0];
	UART_FPM_CMD[1] = FPM_CMD.Head[1];
		
	FPM_CMD.Addr[0] = 0xFF;
	FPM_CMD.Addr[1] = 0xFF;
	FPM_CMD.Addr[2] = 0xFF;
	FPM_CMD.Addr[3] = 0xFF;
	UART_FPM_CMD[2] = FPM_CMD.Addr[0];
	UART_FPM_CMD[3] = FPM_CMD.Addr[1];
	UART_FPM_CMD[4] = FPM_CMD.Addr[2];
	UART_FPM_CMD[5] = FPM_CMD.Addr[3];
	
	FPM_CMD.PID = pid;
	UART_FPM_CMD[6] = FPM_CMD.PID;
	
	FPM_CMD.Length[0] = (data_length + 2) >> 8;
	FPM_CMD.Length[1] = data_length + 2;
	UART_FPM_CMD[7] = FPM_CMD.Length[0];
	UART_FPM_CMD[8] = FPM_CMD.Length[1];
	
	FPM_CMD.Data = data;
	for(i=0; i<data_length; i++)
	{
		temp += FPM_CMD.Data[i];
		UART_FPM_CMD[9+i] = FPM_CMD.Data[i];
	}
	
	FPM_CMD.SUM[0] = (pid + temp + data_length + 2) >> 8;
	FPM_CMD.SUM[1] = (pid + temp + data_length + 2);
	UART_FPM_CMD[9+data_length] = FPM_CMD.SUM[0];
	UART_FPM_CMD[10+data_length] = FPM_CMD.SUM[1];
	
	return 11+data_length;
}

/***********************************
*brief��ϵͳָ��������ֵ��ȡ
*para��para_num �ú�����������ĸ���
*      ...  ��㴫����Ҫ�Ĳ���
*return: ����������ĳ���
************************************/
uint8_t Get_FPM_CMD_Data_area(uint8_t para_num, ...)
{
	uint8_t num = 0;
	va_list valist; //���ڴ�Ų���
	va_start(valist, para_num);
	for(num=0; num < para_num; num++){
		FPM_Data[num] = va_arg(valist,int); //���ڶ�����������ȫ��
	}
	va_end(valist);
	return para_num;
}



/********************
*brief:���ڴ�ӡ����
*********************/
void SHOW_CMD(uint8_t *ch, uint8_t size)
{
	uint8_t i;
	for(i=0; i<size; i++)
	{
		printf("%02x ",ch[i]);
	}
	printf("\n");
}

/*******************************************************
*brif:̽��ָ�ƣ����̽�⵽��ָ��ͼ�����ImageBuffer
*para:no
*return: 0 ̽��ɹ�
*        1 2 3 4̽��ʧ��
********************************************************/
uint8_t Find_Finger_And_Save_ImageBuffer(void)
{
	while(Commands_Send(FP_Get_Img,12) != 0);
	FingerDelay(350);
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07)) //��ʾָ��¼��ɹ�
	{
#if DEBUG
		printf("ָ��̽��ɹ�\n");
#endif
		return 0;
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
		printf("�հ��д�\n");
#endif		
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x02)
	{
#if DEBUG
		printf("û�м�⵽��ָ\n");
#endif		
		return 2;
	}
	else if(uart2_recv_buf[9] == 0x03)
	{
#if DEBUG
		printf("��ָ¼�벻�ɹ�\n");
#endif
		return 3;
	}
	else
	{
		return 4;
	}
}

/************************************************************
*brif:��ȡָ��ͼ�������룬��ŵ�CharBuffer1
*para: area_num ����
*return : 0         success
*         1 2 3 4 5 failed
*************************************************************/
uint8_t Get_Finger_Feature_Save_To_CharBuffer1(void)
{
	while(Commands_Send(FP_Img_To_Buffer1,13) != 0);
	FingerDelay(500);
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))//��ʾ���������ɹ�
	{
#if DEBUG
		printf("���������ɹ�\n");
#endif
		return 0;		
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
		printf("�հ�����\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x06)
	{
#if DEBUG
		printf("ָ��ͼ��̫�Ҷ�����������\n");
#endif
		return 2;
	}
	else if(uart2_recv_buf[9] == 0x07)
	{
#if DEBUG
		printf("ָ��ͼ����������������̫�ٶ�����������\n");
#endif
		return 3;		
	}
	else if(uart2_recv_buf[9] == 0x15)
	{
#if DEBUG
		printf("ͼ�񻺳�����û����Чԭʼͼ��������ͼ��\n");
#endif
		return 4;		
	}
	else
	{
		return 5;
	}
}
/************************************************************
*brif:��ȡָ��ͼ�������룬��ŵ�CharBuffer2
*para: area_num ����
*return : 0         success
*         1 2 3 4 5 failed
*************************************************************/
uint8_t Get_Finger_Feature_Save_To_CharBuffer2(void)
{
	while(Commands_Send(FP_Img_To_Buffer2,13) != 0);
	FingerDelay(500);
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))//��ʾ���������ɹ�
	{
#if DEBUG
		printf("���������ɹ�\n");
#endif
		return 0;		
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
		printf("�հ�����\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x06)
	{
#if DEBUG
		printf("ָ��ͼ��̫�Ҷ�����������\n");
#endif
		return 2;
	}
	else if(uart2_recv_buf[9] == 0x07)
	{
#if DEBUG
		printf("ָ��ͼ����������������̫�ٶ�����������\n");
#endif
		return 3;		
	}
	else if(uart2_recv_buf[9] == 0x15)
	{
#if DEBUG
		printf("ͼ�񻺳�����û����Чԭʼͼ��������ͼ��\n");
#endif
		return 4;		
	}
	else
	{
		return 5;
	}
}
/*******************************************************
*brif:�ϲ�CharBuffer1��CharBuffer2�е��������ָ��ģ��
*para:no
*return:  0     success
*         1 2 3 failed
********************************************************/
uint8_t Combine_Finger_Feature(void)
{
	while(Commands_Send(FP_Reg_Model,12) != 0);
	FingerDelay(200);
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))//��ʾ�����ϲ��ɹ�
	{
#if DEBUG
		printf("����ģ��ϲ��ɹ�\n");
#endif
		return 0;				
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
			printf("�հ��д�\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x0a)
	{
#if DEBUG
		printf("�ϲ�ʧ�ܣ���öָ�Ʋ�����ͬһ��ָ\n");
#endif
		return 2;
	}
	else
	{
		return 3;
	}
}

/*************************************************************************
*brif:��ָ��ģ���ŵ�Flashָ��λ��
*para:BufferID  ������CharBuffer1����CharBuffer2��BufferID�ֱ�Ϊ0x01 0x02
*return:0       success
*       1 2 3 4 failed
**************************************************************************/
uint8_t Save_Finger_Form_To_Flash_Position(uint8_t BufferID, uint16_t PageID)
{
  memset(uart2_recv_buf,0,sizeof(uart2_recv_buf));
	fcmd_length = Get_FPM_CMD(0x01,FPM_Data,Get_FPM_CMD_Data_area(4,STORE,BufferID,(PageID>>8),PageID)); 
#if DEBUG
	SHOW_CMD(UART_FPM_CMD,fcmd_length);
#endif
	while(Commands_Send(UART_FPM_CMD,fcmd_length) != 0);
	FingerDelay(300);
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))//��ʾ�����ϲ��ɹ�
	{
#if DEBUG
		printf("ָ��ģ���ųɹ�\n");
#endif
		return 0;
	}		
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
		printf("�հ��д�\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x0B)
	{
#if DEBUG
		printf("PageID����ָ�ƿⷶΧ\n");
#endif
		return 2;
	}
	else if(uart2_recv_buf[9] == 0x18)
	{
#if DEBUG
		printf("дFLASH����\n");
#endif
		return 3;
	}
	else
	{
		return 4;
	}
}
/**************************************************************************
*brif:����ָ�ƿ�
*para:BufferID  ������CharBuffer1��CharBuffer2��BufferID�ֱ�Ϊ0x01 �� 0x02
*     Page_start  ����ָ�ƿ����ʼҳ
*     Page_num    ��Ҫ������ҳ��
*return:0     success
*       1 2 3 failed
***************************************************************************/
uint16_t Search_Finger_In_Finlib(uint8_t BufferID, uint16_t Page_start, uint16_t Page_num)
{
	uint16_t page = 0;
#if DEBUG
	uint16_t score = 0;
#endif
  memset(uart2_recv_buf,0,sizeof(uart2_recv_buf));
	fcmd_length = Get_FPM_CMD(0x01,FPM_Data,Get_FPM_CMD_Data_area(6,SEARCH,BufferID,(Page_start>>8),Page_start,(Page_num>>8),Page_num)); 
#if DEBUG
	SHOW_CMD(UART_FPM_CMD,fcmd_length);
#endif
	while(Commands_Send(UART_FPM_CMD,fcmd_length) != 0);
//	HAL_Delay(100);//��ʱ����̫��	
	FingerDelay(80);
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))//��ʾָ��������
	{
		page = ((uart2_recv_buf[10] & 0x00FF) << 8) + (uart2_recv_buf[11]);
#if DEBUG
		score = ((uart2_recv_buf[12] & 0x00FF) << 8) + (uart2_recv_buf[13]);
#endif
		Person_num = page;
#if DEBUG
		printf("������ָ�ƣ������: %d ҳ���÷�: %d\n",page,score);
#endif
		return 0;
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
		printf("�հ��д�\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x09)
	{
#if DEBUG
		printf("û����������ָ��\n");
#endif
		return 2;
	}
	else
	{
		return 3;
	}
}

/*****************************
*brif:���ָ�ƿ�
*para:no
*return: 0     success
*        1 2 3 failed
******************************/
uint8_t Empty_The_FinLib(void)
{
  memset(uart2_recv_buf,0,sizeof(uart2_recv_buf));
	fcmd_length = Get_FPM_CMD(0x01,FPM_Data,Get_FPM_CMD_Data_area(1,EMPTY)); 
#if DEBUG
	SHOW_CMD(UART_FPM_CMD,fcmd_length);
#endif
	while(Commands_Send(UART_FPM_CMD,fcmd_length) != 0);
	HAL_Delay(600);//��ʱ����̫��	
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))//��ʾ��ճɹ�
	{
#if DEBUG
		printf("���ָ�ƿ�ɹ�\n");
#endif
		return 0;
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUF
		pirntf("�հ�����\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x11)
	{
#if DEBUG
		printf("���ָ�ƿ�ʧ��\n");
#endif
		return 2;
	}
	else
	{
		return 3;
	}
}

/*******************************************************************************
*brif:��flash���ݿ���ָ��ID�ŵ�ָ��ģ����뵽ģ�建����CharBuffer1��CharBuffer2
*para:BufferID  ������CharBuffer1��CharBuffer2��BufferID�ֱ�Ϊ0x01 �� 0x02
*     PageID    ָ��ҳID
*return:0   success   else  failed
********************************************************************************/
uint8_t Read_Finger_Form_From_Flash(uint8_t BufferID,uint16_t PageID)
{
  memset(uart2_recv_buf,0,sizeof(uart2_recv_buf));
	fcmd_length = Get_FPM_CMD(0x01,FPM_Data,Get_FPM_CMD_Data_area(4,LOADCHAR,BufferID,(PageID>>8),PageID)); 
#if DEBUG
	SHOW_CMD(UART_FPM_CMD,fcmd_length);
#endif
	while(Commands_Send(UART_FPM_CMD,fcmd_length) != 0);
	HAL_Delay(600);//��ʱ����̫��	
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))
	{
#if DEBUG
		printf("��ָ��ģ��ɹ�\n");
#endif
		return 0;
	}
	else if(uart2_recv_buf[9] == 0x01)
	{
#if DEBUG
		printf("�հ��д�\n");
#endif
		return 1;
	}
	else if(uart2_recv_buf[9] == 0x0c)
	{
#if DEBUG
		printf("�����д��ģ����Ч\n");
#endif
		return 2;
	}
	else if(uart2_recv_buf[9] == 0x0B)
	{
#if DEBUG
		printf("PageID����ָ�ƿⷶΧ\n");
#endif
		return 3;
	}
	else 
	{
		return 4;
	}
}


//��Ⲣ¼��ָ��
void Write_Finger(void)
{
buffer1:		
			while(Find_Finger_And_Save_ImageBuffer() != 0);
			while(Get_Finger_Feature_Save_To_CharBuffer1() != 0){goto buffer1;}
buffer2:			
			while(Find_Finger_And_Save_ImageBuffer() != 0);
			while(Get_Finger_Feature_Save_To_CharBuffer2() != 0){goto buffer2;}
			while(Combine_Finger_Feature() != 0);
			if(Save_Finger_Form_To_Flash_Position(0x01,ReadEffectiveTemplate()) == 0)
			{		
				usRegHoldingBuf[0] = FINGER_WRITE_OK;
#if DEBUG
				printf("ָ��¼��ɹ�\n");
#endif
			  Gui_DrawFont_GBK16(15,80,BLACK,GREEN,(uint8_t *)"              ");
				Gui_DrawFont_GBK16(15,80,BLACK,GREEN,(uint8_t *)"Finger In OK!");
				TIP_WRITE_SUCCESS();
			}
			else
			{
				usRegHoldingBuf[0] = FINGER_WRITE_FAILD;
#if DEBUG
				printf("ָ��¼��ʧ��\n");
#endif				
				TIP_WRITE_FAILED();
			}			
}
//ɾ��ָ��ģ��
void Delate_Finger(void)
{
		while(Empty_The_FinLib() != 0);
		Gui_DrawFont_GBK16(15,80,BLACK,GREEN,(uint8_t *)"              ");
		Gui_DrawFont_GBK16(18,80,BLACK,GREEN,(uint8_t *)"Delate All!");
		TIP_DELATE_ALL();
}
//��Ѱָ��
void Find_Finger(void)
{
	if(Find_Finger_And_Save_ImageBuffer() == 0){
		if(Get_Finger_Feature_Save_To_CharBuffer1() == 0){
			if(Search_Finger_In_Finlib(0x01,0x0000,0x0050) == 0){
				usRegHoldingBuf[0] = FINGER_FOUND;
				sprintf((char *)Show_find_man_num,"Find man %d!",Person_num);
				Gui_DrawFont_GBK16(15,80,BLACK,GREEN,(uint8_t *)"              ");
				Gui_DrawFont_GBK16(15,80,BLACK,GREEN,Show_find_man_num);
				TIP_SUCCESS();	
			}
			else
			{
				usRegHoldingBuf[0] = FINGER_NOT_FIND;
				Gui_DrawFont_GBK16(15,80,BLACK,GREEN,(uint8_t *)"              ");
				Gui_DrawFont_GBK16(18,80,BLACK,GREEN,(uint8_t *)"Can`t Find!");
				TIP_FAILED();
			}					
		}
	}
	else 
	{
		usRegHoldingBuf[0] = NO_FINGER;
	}
}

//��س�ʼ��
void Init_sth(void)
{
	Lcd_Init();
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
	Lcd_Clear(GREEN);
	showimage_farsight(gImage_logo);
	Gui_DrawFont_GBK16(28,40,BLUE,GREEN,(uint8_t *)"Touch ID");
}

/*brif:��ȡ��Чģ�����
*para:no
*return: success finger numbers saved
*        faild   0 1
*/

uint16_t ReadEffectiveTemplate(void)
{
  memset(uart2_recv_buf,0,sizeof(uart2_recv_buf));
	fcmd_length = Get_FPM_CMD(0x01,FPM_Data,Get_FPM_CMD_Data_area(1,TEMPLETENUM)); 
#if DEBUG
	SHOW_CMD(UART_FPM_CMD,fcmd_length);
#endif
	while(Commands_Send(UART_FPM_CMD,fcmd_length) != 0);
	HAL_Delay(600);//��ʱ����̫��	
	if((uart2_recv_buf[9] == 0x00) && (uart2_recv_buf[6] == 0x07))
	{
		FingerCount = (uart2_recv_buf[10] << 8) + uart2_recv_buf[11];		
#if DEBUG
		printf("�Ѵ洢��Чģ�� : %d\n",FingerCount);
#endif
		return FingerCount;		
	}
	else 	if((uart2_recv_buf[9] == 0x01) && (uart2_recv_buf[6] == 0x07))
  {
#if DEBUG
		printf("�հ��д�\n");
#endif	
		return 0;
	}	
  else
	{
#if DEBUG
		printf("��֮����\n");
#endif			
		return 1;
	}		
}

