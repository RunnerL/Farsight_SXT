#include "myiic.h"

GPIO_InitTypeDef        GPIO_InitStructure;  
#define FALSE 0
#define TRUE 1   
void delayus(uint8_t i)
{	
  while(i) 
  { 
    i--; 
  } 
}

/**
  * @brief  IIC Init
  * @param  A:
  * @retval None
  */
void IIC_Init(void)
{					     
	RCC_AHBPeriphClockCmd(	RCC_AHBPeriph_GPIOF, ENABLE );	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //í?íìê?3?
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF,GPIO_Pin_6|GPIO_Pin_7); 	//PB10,PB11 ê?3???
}

/**
  * @brief  Set SDA Pin as Output Mode
  * @retval None
  */
void SDA_OUT()  
{  
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;  
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_Init(GPIOF, &GPIO_InitStructure);  
}  

/**
  * @brief  Set SDA Pin as Input Mode
  * @retval None
  */
void SDA_IN()  
{  
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;  
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;// !!!
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_Init(GPIOF, &GPIO_InitStructure);  
} 

/** 
  * @brief  output high form SCL pin 
  * @retval None 
  */  
void IIC_SCL_1()  
{  
GPIO_SetBits(GPIOF, GPIO_Pin_6);  
}  
  
/** 
  * @brief  output LOW form SCL pin 
  * @retval None 
  */  
void IIC_SCL_0()  
{  
GPIO_ResetBits(GPIOF, GPIO_Pin_6);    
}  
  
/**
  * @brief  read input voltage from SDA pin
  * @retval None
  */
uint8_t SDA_READ()
{
  return GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_7);
}

/**
  * @brief  output high form SDA pin
  * @retval None
  */
void IIC_SDA_1()
{
  GPIO_SetBits(GPIOF, GPIO_Pin_7);
}

/**
  * @brief  output low form SDA pin
  * @retval None
  */
void IIC_SDA_0()
{
  GPIO_ResetBits(GPIOF, GPIO_Pin_7);
}

/** 
* @brief  Simulate IIC conmunication :Create Start signal 
  * @retval None 
  */  
void IIC_Start(void)  
{  
    SDA_OUT();     //sda output  
    IIC_SDA_1();            
    IIC_SCL_1();  
    delayus(4);  
    IIC_SDA_0();   //START:when CLK is high,DATA change form high to low   
    delayus(4);  
    IIC_SCL_0();   //hold scl line, prepare to transmit data  
}   
  
/** 
  * @brief  Simulate IIC conmunication : Create Stop signal 
  * @retval None 
  */  
void IIC_Stop(void)  
{  
    SDA_OUT();    //sda output mode   
    IIC_SCL_0();  
    IIC_SDA_0();  //STOP:when CLK is high DATA change form low to high  
    delayus(4);  
    IIC_SCL_1();   
    IIC_SDA_1();  //indicate transmit over  
    delayus(4);                               
}  
  
/** 
* @brief  Simulate IIC conmunication : wait for target device's ACK 
* @retval ACK (0) : receive success 
* @retval NACK(1) : receive unsuccess 
  */  
uint8_t IIC_Wait_Ack(void)  
{  
    uint8_t ucErrTime = 0;  
    SDA_IN();      //set as input mode  
    IIC_SDA_1();  
  delayus(1);        
    IIC_SCL_1();  
  delayus(1);      
    while(SDA_READ())  
    {  
        ucErrTime++;  
        if(ucErrTime > 250)  
        {  
            IIC_Stop();  
            return FALSE;  
        }  
    }  
    IIC_SCL_0(); //release scl line  
    return TRUE;    
}   
  
/** 
  * @brief  Simulate IIC conmunication : make an ACK 
  * @retval None 
  */  
void IIC_Ack(void)  
{  
    IIC_SCL_0();  
    SDA_OUT();  
    IIC_SDA_0();  
    delayus(2);  
    IIC_SCL_1();  
    delayus(2);  
    IIC_SCL_0();  
}  
  
/** 
  * @brief  Simulate IIC conmunication : don't make an ACK 
  * @retval None 
  */  
void IIC_NAck(void)  
{  
    IIC_SCL_0();  
    SDA_OUT();  
    IIC_SDA_1();  
    delayus(2);  
    IIC_SCL_1();  
    delayus(2);  
    IIC_SCL_0();  
}                                          
  
  
/** 
  * @brief  Simulate IIC conmunication : Transmit one byte Data 
  * @param  txd: data to be transmit 
  * @retval None 
  */  
void IIC_Send_Byte(uint8_t txd)  
{                          
  uint8_t i;     
  SDA_OUT();          
  IIC_SCL_0();//push down scl  to start transmit data  
  for(i = 0; i < 8; ++i)  
  {                
    if(txd & 0x80)  
    {  
      IIC_SDA_1();  
    }  
    else  
    {  
      IIC_SDA_0();  
    }  
    txd <<= 1;        
    delayus(2);     
    IIC_SCL_1();  
    delayus(2);   
    IIC_SCL_0();      
    delayus(2);  
  }    
}       
  
//?á1??×??ú￡?ack=1ê±￡?·￠?íACK￡?ack=0￡?·￠?ínACK     
/** 
  * @brief  Simulate IIC conmunication : Receive one byte Data 
  * @param  ack: Whether transmit ACK 
  * @retval the data have been receive 
  */  
uint8_t IIC_Read_Byte(unsigned char ack)  
{  
    unsigned char i, res = 0;  
    SDA_IN();               //SDA input mode  
  for(i = 0; i < 8; ++i )  
    {  
    IIC_SCL_0();   
    delayus(2);  
    IIC_SCL_1();  
    res <<= 1;  
    if(SDA_READ())  
    {  
      res++;   
    }        
        delayus(1);   
  }                    
  if (!ack)  
  {  
    IIC_NAck();//make NACK  
  }  
  else  
  {  
    IIC_Ack(); //make ACK  
  }  
  return res;  
}  


//写入1字节数据       待写入数据    待写入寄存器地址       器件类型(24c16或SD2403)
uint8_t I2C_Write_one_Byte(uint8_t SendByte, uint8_t DeviceAddress,uint16_t WriteAddress)
{		
  IIC_Start();
  IIC_Send_Byte(WriteAddress);     //写设备地址
  if(!IIC_Wait_Ack()){IIC_Stop(); return FALSE;}
  IIC_Send_Byte(DeviceAddress);   //写寄存器   
  IIC_Wait_Ack();	
  IIC_Send_Byte(SendByte);
  IIC_Wait_Ack();   
  IIC_Stop();
  Delay(1);
  return TRUE;
}

//写入1字节数据       待写入数据    待写入寄存器地址       器件类型(24c16或SD2403)
uint8_t I2C_Read_one_Byte(uint8_t *ReadByte,  uint8_t DeviceAddress,uint16_t WriteAddress)
{		
  IIC_Start();
  IIC_Send_Byte(WriteAddress);     //写设备地址
  if(!IIC_Wait_Ack()){IIC_Stop(); return FALSE;}
  IIC_Send_Byte(DeviceAddress);   //写寄存器   
  IIC_Wait_Ack();	
  IIC_Start();
  IIC_Send_Byte(WriteAddress|0x01);     //写设备地址
  // IIC_Send_Byte(ReadByte);
  IIC_Wait_Ack();   
  *ReadByte = IIC_Read_Byte(1);
  IIC_Stop(); 
  //注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
  //Systick_Delay_1ms(10);
  Delay(1);
  return TRUE;
}

void AT24CXX_Write(uint8_t WriteAddr,uint8_t *pBuffer,uint8_t NumToWrite)
{
  while (NumToWrite--) {
    I2C_Write_one_Byte(*pBuffer, WriteAddr, 0xa0);
    WriteAddr++;
    pBuffer++;
  }
}

void AT24CXX_Read(uint8_t ReadAddr,uint8_t *pBuffer,uint8_t NumToRead)
{
  while (NumToRead){
    I2C_Read_one_Byte(pBuffer++, ReadAddr++, 0xa0);
    NumToRead--;
  }
}