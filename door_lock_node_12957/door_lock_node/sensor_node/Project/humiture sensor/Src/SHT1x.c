#include "SHT1x.h"

#define IN 0
#define OUT 1

GPIO_InitTypeDef  GPIO_InitStruct;
void I2C_delay(uint8_t i)
{	
  // uint8_t i=100; //这里可以优化速度	，经测试最低到5uf还能写入
  while(i) 
  { 
    i--; 
  } 
}

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure the used I/O ports pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct; 
  
  /* Enable  GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	
  
  /*!< GPIO configuration */  
  /*!< Configure I2C pins: SCL */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;  	//开漏			
  GPIO_Init(GPIOB , &GPIO_InitStruct); 
  
  /*!< Configure I2C pins: SDA */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB , &GPIO_InitStruct);
}

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

/********************************************************
SHT1x启动传输
********************************************************/
void Trans_Start(void)
{
  SDA_H;  SCK_L;        //准备
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SDA_L;
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SDA_H;
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
}

uint8_t I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
  Set_PIN_Mode(IN);
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
  //  Set_PIN_Mode(OUT);
  I2C_delay(100);
  //  SDA_H;        //释放数据线
  I2C_delay(100);
  return TRUE;
}

//向从设备写一个字节
uint8_t I2C_SendByte(uint8_t SendByte) //数据从高位到低位//
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
  SDA_H;                                //释放数据线
  I2C_delay(100);
  return I2C_WaitAck();
}

/***********************************************************************************************************
* 函数名称: void SHT1x_Init(void)
* 输入参数: 无
*               
* 返回参数: 无
*
* 功    能: SHTx模块复位
*
* 作    者: 2016/3/18, by SongLei
************************************************************************************************************/
void SHT1x_Init(void)
{
  uint8_t i;
  
  SDA_H;                //时钟、数据准备
  SCK_L;
  I2C_delay(100);
  for(i=0;i<9;i++)      //高电平保持高，SCK时钟触发9次，发送启动传输，通讯即复位
  {
    SCK_H;
    I2C_delay(100);
    SCK_L;
    I2C_delay(100);
  }
  
  Trans_Start();        //启动传输
}

uint8_t I2C_ReceiveByte(uint8_t ack)  //数据从高位到低位//
{ 
  uint8_t i=8;
  uint8_t ReceiveByte=0;
  
  //  SDA_H;
  Set_PIN_Mode(IN);
  I2C_delay(100);
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
  I2C_delay(100);
  for(;;)
  {
    if(SDA_read){
      break;
    }
  }
  Set_PIN_Mode(OUT);
  I2C_delay(100);
  //  SCK_H;
  //  I2C_delay(100);
  if(ack){              //ack=1时
    SDA_L;
  }
  else SDA_H;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
  //  SDA_H;  //释放数据线
  
  return ReceiveByte;
}

/*读状态寄存器*/
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
//----------------------------------------------------------------------------------
// reads the status register with checksum (8-bit)
{ 
  unsigned char error=0;
  Trans_Start(); //transmission start
  error=I2C_SendByte(STATUS_REG_R); //send command to sensor
  *p_value=I2C_ReceiveByte(ACK); //read status register (8-bit)
  *p_checksum=I2C_ReceiveByte(noACK); //read checksum (8-bit) 
  return error; //error=1 in case of no response form the sensor
}

//写状态寄存器
char s_write_statusreg(unsigned char *p_value)
// writes the status register with checksum (8-bit)
{ 
  unsigned char error=0;
  Trans_Start(); //transmission start
  error+=I2C_SendByte(STATUS_REG_W);//send command to sensor
  error+=I2C_SendByte(*p_value); //send value of status register
  return error; //error>=1 in case of no response form the sensor
} 

//SHT1x 温湿度检测
/***********************************************************************************************************
* 函数名称: SHT1x_Read(uint8_t * p_value, uint8_t *p_checkSum,uint8_t mode)
* 输入参数: p_value：指针类型的数据存取温度或湿度值；   
*           p_checkSum：存储接收到的CRC-8校验;
*           mode：读取数据的类型（温度还是湿度）;
* 返回参数: 无
*
* 功    能: 读取温度或湿度的值并存储在p_value中，校验值存储p_checkSum中
*
* 作    者: 2016/3/18, by SongLei
************************************************************************************************************/
uint8_t SHT1x_Read(uint8_t * p_value, uint8_t *p_checkSum, uint8_t mode)
{
  uint8_t error = 0;
  uint32_t i=0;
  
  Delay(1);
  Trans_Start();        //启动传输
  
  switch(mode)
  {
  case TEMP:  error = I2C_SendByte(MEASURE_TEMP); break;        //返回1成功，0表示没有应答
  case HUMI:  error = I2C_SendByte(MEASURE_HUMI); break;        //返回1成功，0表示没有应答
  default:break;
  }

#if 0  
  if(error == 0)
  {
    printf("send ack 错误\n");
   // return FALSE;
  }
#endif
  
    Delay(1);
    Set_PIN_Mode(IN);
    
    for(;;)     //等待测量结束 （80Ms-->12位，320ms-->14位）
    {
      if(SDA_read)   //低电平时跳出循环
      {
        i++;
        if(i > 6553500 ) //如果长时间（最大320Ms）Data线没有被拉低，则返回错误error =0
        {
          error = 0;
          break;
        }
      }
      else{
        break;
      }
    }
    
    *(p_value+1) = I2C_ReceiveByte(ACK);   //高8位 存储在int型的高字节
    *(p_value) = I2C_ReceiveByte(ACK);     //低8位 存储在int型的低字节
    *(p_checkSum) = I2C_ReceiveByte(noACK); //read CRC校验码
    
  if(error == 0)
    return FALSE;
  
  return TRUE;
}


/***********************************************************************************************************
* 函数名称: SHT10_Cal(uint16_t Temp,uint16_t Hum, float* pTempValue,float* pHumValue)
* 输入参数: Temp:采集的温度值，Hum：采集的湿度值;
*           pTempValue：返回转换后的温度值，pHumValue：返回转换后的湿度值;
* 输出参数: 无
*
* 功    能: SHT1x系列温湿度值标度转换及温度补偿
*
* 作    者: 2016/3/18, by SongLei
************************************************************************************************************/
void SHT10_Cal(uint16_t Temp,uint16_t Hum, float* pTempValue,float* pHumValue)
{
  const float d1 = -39.7;      //温度补偿参数（因为是3.3v的电压）
  const float d2 = 0.01;
  float Temp_C;                 //温度℃
  
  const float C1 = -2.0468;           // 12位湿度精度 修正公式       
  const float C2 = +0.0367;           // 12位湿度精度 修正公式       
  const float C3 = -0.0000015955;    // 12位湿度精度 修正公式     
  const float T1 = +0.01;             // 14位温度精度  修正公式
  const float T2 = +0.00008;          // 14位温度精度  修正公式
  
  float RH_Lin;        //湿度线性值
  float RH_True;       //湿度真实值
  
  //温度结果，换算                 
  Temp_C = d1 + d2 * Temp;   //温度是14位
  
  //RH线性结果               湿度是12位
  RH_Lin = C1 + C2 * Hum + C3 * Hum *Hum;     //相对湿度非线性补偿
  RH_True = (Temp_C - 25) * (T1 + T2 * Hum) + RH_Lin; //相对湿度对于温度依赖性补偿
  //限定范围
  if( RH_True > 100 ) RH_True = 100;  //湿度最大修正
  if( RH_True < 0.01) RH_True = 0.01; //湿度最小修正
  
  *pTempValue = Temp_C;               //返回温度结果
  *pHumValue = RH_True;               //返回湿度结果
}

void  sensor_init(void)
{
  I2C_GPIO_Configuration();
  SHT1x_Init();
}

/***********************************************************************************************************
* 函数名称: Sht1x_Measure()
* 输入参数: 无       
*               
* 返回参数: 无
*
* 功    能: 对SHT1x温湿度传感器进行操作
*
* 作    者: 2016/3/19, by SongLei
************************************************************************************************************/
void Sht1x_Measure(void)
{
 
    
}
void restore_data()
{
  //modbus从机地址
  usRegHoldingBuf[SLAVE_ID_OFF] = MSlaveID;
  usRegHoldingBuf[TEMPERATURE_DATA_OFF] = 0; 
  usRegHoldingBuf[HUMIDITY_DATA_OFF] = 0;
}

void get_sensor_data(void)
{
  uint16_t TempValue;  //温度结果  16bit
  uint16_t HumValue;    //湿度结果 16b  
  uint8_t error;
  Value Humi_val,Temp_val;  //定义两个共同体，一个存储湿度，一个存储温度
  uint8_t checkSum;
  
    error = 0;     
    Humi_val.i = 0;
    Temp_val.i = 0;
    error += SHT1x_Read((uint8_t *)&Temp_val.i, &checkSum, TEMP);
    error += SHT1x_Read((uint8_t *)&Humi_val.i, &checkSum, HUMI);

      if(error == 0)
    {
#if defined(DEBUG)      
      printf("error\n");
#endif
      Delay(50);
      SHT1x_Init();
      //I2C_SendByte(0x1e);
    }
    else{
      TempValue = (uint16_t)Temp_val.i;  //温度转换成16位数据
      HumValue  = (uint16_t)Humi_val.i;  //湿度转换成16位数据
      
      SHT10_Cal(TempValue ,HumValue,&Temp_val.f,&Humi_val.f);
#if defined(DEBUG)
      printf("温度 %2.1fC 湿度 %2.1f%%\n",Temp_val.f,Humi_val.f);
      printf("________________________________\n\n");
#endif
      usRegHoldingBuf[TEMPERATURE_DATA_OFF] = (unsigned int)(Temp_val.f * 10); 
      usRegHoldingBuf[HUMIDITY_DATA_OFF] = (unsigned int)(Humi_val.f * 10);
    }
}

