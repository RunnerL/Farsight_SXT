#include "BH1750.h"


void I2C_delay(uint8_t i)
{	
  // uint8_t i=100; //这里可以优化速度	，经测试最低到5uf还能写入
  while(i) 
  { 
    i--; 
  } 
}

/* Private functions ---------------------------------------------------------*/


void Set_PIN_Mode(uint8_t IN_OUT)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  if(IN_OUT == 0)
  {
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB , &GPIO_InitStruct);
  }
  else if(IN_OUT == 1)
  {
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB , &GPIO_InitStruct);
  }
}

uint8_t I2C_Start(void)
{
  SDA_H;
  SCK_H;
  I2C_delay(100);
  if(!SDA_read) return FALSE;	//SDA线为低电平则总线忙,退出
  SDA_L;
  I2C_delay(100);
  if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
  SDA_L;
  I2C_delay(100);
  return TRUE;
}

void I2C_Stop(void)
{
  SCK_L;
  I2C_delay(100);
  SDA_L;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SDA_H;
  I2C_delay(100);
}


void I2C_Ack(void)
{	
  SCK_L;
  I2C_delay(100);
  SDA_L;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
}

void I2C_NoAck(void)
{	
  SCK_L;
  I2C_delay(100);
  SDA_H;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
}

uint8_t I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
  Set_PIN_Mode(IN);
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  if(SDA_read)
  {
    SCK_L;
    return FALSE;
  }
  SCK_L;
  I2C_delay(100);
  Set_PIN_Mode(OUT);

  return TRUE;
}

void I2C_SendByte(uint8_t SendByte) //数据从高位到低位//
{
  uint8_t i=8;
  while(i--)
  {
    SCK_L;
    I2C_delay(100);
    if(SendByte&0x80)
      SDA_H;  
    else 
      SDA_L;   
    SendByte<<=1;
    I2C_delay(100);
    SCK_H;
    I2C_delay(100);
  }
   SCK_L;
}

uint8_t I2C_ReceiveByte(void)  //数据从高位到低位//
{ 
  uint8_t i=8;
  uint8_t ReceiveByte=0;
  Set_PIN_Mode(IN);
  I2C_delay(100);
  SDA_H;				
  while(i--)
  {
    ReceiveByte<<=1;      
    SCK_L;
    I2C_delay(100);
    SCK_H;
    I2C_delay(100);	
    if(SDA_read)
    {
      ReceiveByte|=0x01;
    }
  }
  SCK_L;
  Set_PIN_Mode(OUT);
  return ReceiveByte;
}

static uint8_t setModeControl(uint8_t mctl)
{
  uint8_t buf[2];
  
    buf[0] = BH1750_ADDR_POWER_ON;       //要写入的寄存器的地址
    buf[1] = mctl;                       //向寄存写的数据
    
  if(!I2C_Start())return FALSE;
  I2C_SendByte(MMA_ADRESS);     //写设备地址
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(BH1750_ADDR_POWER_ON);   //写寄存器   
  I2C_WaitAck();	 
  I2C_Stop(); 
}


/***********************************************************************************************************
* 函数名称: Multiple_Read_BH1750()
* 输入参数: 无
*               
* 返回参数: 无
*
* 功    能: 连续读取BH750内部数据
*
* 作    者: 2016/3/19, by SongLei
************************************************************************************************************/
uint8_t Multiple_Read_BH1750(uint8_t *pBuffer, uint8_t length, uint8_t SlaveAddress, uint8_t DeviceAddres)
{           
  if(!I2C_Start()) return FALSE;
//  I2C_SendByte(SlaveAddress);           //设置高起始地址+器件地址 
//  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}  //接收从机应答
//  I2C_SendByte(DeviceAddress);          //设置低起始地址      
//  I2C_WaitAck();                        
//  I2C_Start();
  I2C_SendByte(SlaveAddress | 0x0001);
  I2C_WaitAck();
  while(length)
  {
    *pBuffer = I2C_ReceiveByte();
    if(length == 1)I2C_NoAck();
    else I2C_Ack(); 
    pBuffer++;
    length--;
  }
  I2C_Stop();
  return TRUE;
}


//写入1串数据      待写入数组地址    待写入长度      待写入地址       器件类型(24c16或SD2403)
uint8_t I2C_BufferWrite(uint8_t* pBuffer, uint8_t length,uint8_t DeviceAddress, uint16_t WriteAddress)
{
  if(!I2C_Start())return FALSE;
  I2C_SendByte(WriteAddress);		//设置高起始地址+器件地址 
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(DeviceAddress);   //设置低起始地址      
  I2C_WaitAck();	
  
  while(length--)
  {
    I2C_SendByte(* pBuffer);
    I2C_WaitAck();
    pBuffer++;
  }
  I2C_Stop();
  //注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
  //Systick_Delay_1ms(10);
  return TRUE;
}

//读出1串数据         存放读出数据  待读出长度      待读出地址       器件类型(24c16或SD2403)	
uint8_t I2C_ReadByte(uint8_t* pBuffer, uint8_t length, uint8_t DeviceAddress, uint16_t ReadAddress )
{		
  if(!I2C_Start())return FALSE;
  I2C_SendByte(ReadAddress);//设置高起始地址+器件地址 
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(DeviceAddress);   //设置低起始地址      
  I2C_WaitAck();
  I2C_Start();
  I2C_SendByte(ReadAddress | 0x0001);
  I2C_WaitAck();
  while(length)
  {
    *pBuffer = I2C_ReceiveByte();
    if(length == 1)I2C_NoAck();
    else I2C_Ack(); 
    pBuffer++;
    length--;
  }
  I2C_Stop();
  return TRUE;
}

/***********************************************************************************************************
* 函数名称: BH1750_Init()
* 输入参数: 无
*               
* 返回参数: 无
*
* 功    能: 初始化BH1750模块
*
* 作    者: 2016/3/19, by SongLei
************************************************************************************************************/
void BH1750_Init(void)
{
  /* set to measurement mode by default */
  setModeControl(BH1750_ADDR_POWER_ON);
}
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure the used I/O ports pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void sensor_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct; 
  
  /* Enable  GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	
  
  /*!< GPIO configuration */  
  /*!< Configure I2C pins: SCL */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;  	//推挽			
  GPIO_Init(GPIOB , &GPIO_InitStruct); 
  
  /*!< Configure I2C pins: SDA */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;     //GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB , &GPIO_InitStruct);
  
  BH1750_Init();
}

void restore_data()
{
  //modbus从机地址
  usRegHoldingBuf[SLAVE_ID_OFF] = MSlaveID;
  usRegHoldingBuf[LIGHT_VALUE_OFF] = 0;
}
void get_sensor_data(void)
{
  int    dis_data = 0;                       //变量
  uint8_t buf[3] = {0};
  
  I2C_BufferWrite(&buf[1],0, BH1750_ADDR_POWER_ON, MMA_ADRESS);     //BH1750只写入寄存器地址即可
  I2C_BufferWrite(&buf[1],0, BH1750_ADDR_Cont_H_Mode, MMA_ADRESS);  //BH1750只写入寄存器地址即可
  
  Delay(18); //180Ms

  Multiple_Read_BH1750(buf, 3 , MMA_ADRESS, 0x00);
  
  dis_data=buf[0];
  dis_data=(dis_data<<8)+buf[1];//合成数据，即光照数据
#if defined(DEBUG)
  printf("----%d\n",(unsigned int)((float)dis_data/1.2)); 
#endif
  usRegHoldingBuf[LIGHT_VALUE_OFF] = (unsigned int)((float)dis_data/1.2);
}









