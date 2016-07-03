#include "gpio.h"
#include "stdint.h"
#include "lcd.h"
#include "font_lcd.h"





void Delay_ms(int time)
{
	int i,j;
	for(i=0;i<time*10;i++)
	{
		for(j=0;j<100;j++)
		{
		
		}
	}
}

void SPI_WriteData(uint8_t Data)
{
	unsigned char i;
	for(i=8; i>0; i--)
	{
		if(Data & 0x80)
		{
			GPIOA->BSRR = GPIO_PIN_7;
		}
		else
		{
			GPIOA->BRR = GPIO_PIN_7;
		}	
			GPIOA->BSRR = GPIO_PIN_5;
		  GPIOA->BRR  = GPIO_PIN_5;
		  Data  <<=  1;
	}
}

void LCD_WriteData_16Bit(uint16_t Data)
{
	 GPIOA->BRR = GPIO_PIN_4;
	 GPIOA->BSRR = GPIO_PIN_6;
	 SPI_WriteData(Data>>8); 	//д���8λ����
	 SPI_WriteData(Data); 			//д���8λ����	
	 GPIOA->BSRR = GPIO_PIN_4;
}

//��Һ����дһ��8λָ��
void Lcd_WriteIndex(uint8_t Index)
{
   //SPI д����ʱ��ʼ
	 GPIOA->BRR = GPIO_PIN_4;
	 GPIOA->BRR = GPIO_PIN_6;
	 SPI_WriteData(Index);
	 GPIOA->BSRR = GPIO_PIN_4;
}
//��Һ����дһ��8λ����
void Lcd_WriteData(uint8_t Data)
{
	GPIOA->BRR = GPIO_PIN_4;
	GPIOA->BSRR = GPIO_PIN_6;
  SPI_WriteData(Data);
	GPIOA->BSRR = GPIO_PIN_4;
}

void Lcd_WriteReg(uint8_t Index,uint8_t Data)
{
	GPIOA->BRR = GPIO_PIN_4;
	Lcd_WriteIndex(Index);
  Lcd_WriteData(Data);
	GPIOA->BSRR = GPIO_PIN_4;
}



//LCD Init For 1.44Inch LCD Panel with ST7735R.
void Lcd_Init(void)
{	
	Lcd_WriteIndex(0x11);//Sleep exit 
	HAL_Delay(120);
 
	Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
	Lcd_WriteData(0xC8); 
	
	Lcd_WriteIndex(0x3A); //65k mode 
	Lcd_WriteData(0x05); 
	
	Lcd_WriteIndex(0x29);//Display on		
}

/*************************************************
��������LCD_Set_Region
���ܣ�����lcd��ʾ�����ڴ�����д�������Զ�����
��ڲ�����xy�����յ�
����ֵ����
*************************************************/
void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{		
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_start+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_end+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_start+3);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_end+3);
	
	Lcd_WriteIndex(0x2c);

}
/*************************************************
��������LCD_Set_XY
���ܣ�����lcd��ʾ��ʼ��
��ڲ�����xy����
����ֵ����
*************************************************/
void Lcd_SetXY(uint16_t x,uint16_t y)
{
  	Lcd_SetRegion(x,y,x,y);
}

	
/*************************************************
��������LCD_DrawPoint
���ܣ���һ����
��ڲ�������
����ֵ����
*************************************************/
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Data);

}    

/*****************************************
 �������ܣ���TFTĳһ�����ɫ                          
 ���ڲ�����color  ����ɫֵ                                 
******************************************/
unsigned int Lcd_ReadPoint(uint16_t x,uint16_t y)
{
  unsigned int Data;
  Lcd_SetXY(x,y);

  //Lcd_ReadData();//���������ֽ�
  //Data=Lcd_ReadData();
  Lcd_WriteData(Data);
  return Data;
}
/*************************************************
��������Lcd_Clear
���ܣ�ȫ����������
��ڲ����������ɫCOLOR
����ֵ����
*************************************************/
void Lcd_Clear(uint16_t Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	  	LCD_WriteData_16Bit(Color);
    }   
}
//ȡģ��ʽ ˮƽɨ�� ������ ��λ��ǰ
void showimage_farsight(const unsigned char *p) //��ʾ128*35 QQͼƬ
{
  int i; 
	unsigned char picH,picL;
	//Lcd_Clear(WHITE); //����  
	Lcd_SetRegion(0,0,127,34);		//��������
	for(i=0;i<128*35;i++)
 {	
	picL=*(p+i*2);	//���ݵ�λ��ǰ
	picH=*(p+i*2+1);				
	LCD_WriteData_16Bit(picH<<8|picL);  						
 }		
}

void showimage(const unsigned char *p) //��ʾ128*35 QQͼƬ
{
  int i; 
	unsigned char picH,picL;
	//Lcd_Clear(WHITE); //����  
	Lcd_SetRegion(0,0,127,127);		//��������
	for(i=0;i<128*128;i++)
 {	
	picL=*(p+i*2);	//���ݵ�λ��ǰ
	picH=*(p+i*2+1);				
	LCD_WriteData_16Bit(picH<<8|picL);  						
 }		
}

void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
	unsigned char i,j;
	unsigned short k,x0;
	x0=x;

	while(*s) 
	{	
		if((*s) < 128) 
		{
			k=*s;
			if (k == 13) 
			{
				x=x0;
				y+=16;
			}
			else 
			{
				if (k>32) k-=32; else k=0;
	
			    for(i=0;i<16;i++)
				for(j=0;j<8;j++) 
					{
				    	if(asc16[k*16+i]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
						else 
						{
							if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
						}
					}
				x+=8;
			}
			s++;
		}
			
		else 
		{
		

			for (k=0;k<hz16_num;k++) 
			{
			  if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<16;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
								else {
									if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
								}
							}
				    }
				}
			  }
			s+=2;x+=16;
		} 
		
	}
}

