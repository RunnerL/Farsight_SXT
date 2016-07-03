#include "SHT1x.h"

#define IN 0
#define OUT 1

GPIO_InitTypeDef  GPIO_InitStruct;
void I2C_delay(uint8_t i)
{	
  // uint8_t i=100; //��������Ż��ٶ�	����������͵�5uf����д��
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
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;  	//��©			
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
SHT1x��������
********************************************************/
void Trans_Start(void)
{
  SDA_H;  SCK_L;        //׼��
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

uint8_t I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
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
  //  SDA_H;        //�ͷ�������
  I2C_delay(100);
  return TRUE;
}

//����豸дһ���ֽ�
uint8_t I2C_SendByte(uint8_t SendByte) //���ݴӸ�λ����λ//
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
  SDA_H;                                //�ͷ�������
  I2C_delay(100);
  return I2C_WaitAck();
}

/***********************************************************************************************************
* ��������: void SHT1x_Init(void)
* �������: ��
*               
* ���ز���: ��
*
* ��    ��: SHTxģ�鸴λ
*
* ��    ��: 2016/3/18, by SongLei
************************************************************************************************************/
void SHT1x_Init(void)
{
  uint8_t i;
  
  SDA_H;                //ʱ�ӡ�����׼��
  SCK_L;
  I2C_delay(100);
  for(i=0;i<9;i++)      //�ߵ�ƽ���ָߣ�SCKʱ�Ӵ���9�Σ������������䣬ͨѶ����λ
  {
    SCK_H;
    I2C_delay(100);
    SCK_L;
    I2C_delay(100);
  }
  
  Trans_Start();        //��������
}

uint8_t I2C_ReceiveByte(uint8_t ack)  //���ݴӸ�λ����λ//
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
  if(ack){              //ack=1ʱ
    SDA_L;
  }
  else SDA_H;
  I2C_delay(100);
  SCK_H;
  I2C_delay(100);
  SCK_L;
  I2C_delay(100);
  //  SDA_H;  //�ͷ�������
  
  return ReceiveByte;
}

/*��״̬�Ĵ���*/
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

//д״̬�Ĵ���
char s_write_statusreg(unsigned char *p_value)
// writes the status register with checksum (8-bit)
{ 
  unsigned char error=0;
  Trans_Start(); //transmission start
  error+=I2C_SendByte(STATUS_REG_W);//send command to sensor
  error+=I2C_SendByte(*p_value); //send value of status register
  return error; //error>=1 in case of no response form the sensor
} 

//SHT1x ��ʪ�ȼ��
/***********************************************************************************************************
* ��������: SHT1x_Read(uint8_t * p_value, uint8_t *p_checkSum,uint8_t mode)
* �������: p_value��ָ�����͵����ݴ�ȡ�¶Ȼ�ʪ��ֵ��   
*           p_checkSum���洢���յ���CRC-8У��;
*           mode����ȡ���ݵ����ͣ��¶Ȼ���ʪ�ȣ�;
* ���ز���: ��
*
* ��    ��: ��ȡ�¶Ȼ�ʪ�ȵ�ֵ���洢��p_value�У�У��ֵ�洢p_checkSum��
*
* ��    ��: 2016/3/18, by SongLei
************************************************************************************************************/
uint8_t SHT1x_Read(uint8_t * p_value, uint8_t *p_checkSum, uint8_t mode)
{
  uint8_t error = 0;
  uint32_t i=0;
  
  Delay(1);
  Trans_Start();        //��������
  
  switch(mode)
  {
  case TEMP:  error = I2C_SendByte(MEASURE_TEMP); break;        //����1�ɹ���0��ʾû��Ӧ��
  case HUMI:  error = I2C_SendByte(MEASURE_HUMI); break;        //����1�ɹ���0��ʾû��Ӧ��
  default:break;
  }

#if 0  
  if(error == 0)
  {
    printf("send ack ����\n");
   // return FALSE;
  }
#endif
  
    Delay(1);
    Set_PIN_Mode(IN);
    
    for(;;)     //�ȴ��������� ��80Ms-->12λ��320ms-->14λ��
    {
      if(SDA_read)   //�͵�ƽʱ����ѭ��
      {
        i++;
        if(i > 6553500 ) //�����ʱ�䣨���320Ms��Data��û�б����ͣ��򷵻ش���error =0
        {
          error = 0;
          break;
        }
      }
      else{
        break;
      }
    }
    
    *(p_value+1) = I2C_ReceiveByte(ACK);   //��8λ �洢��int�͵ĸ��ֽ�
    *(p_value) = I2C_ReceiveByte(ACK);     //��8λ �洢��int�͵ĵ��ֽ�
    *(p_checkSum) = I2C_ReceiveByte(noACK); //read CRCУ����
    
  if(error == 0)
    return FALSE;
  
  return TRUE;
}


/***********************************************************************************************************
* ��������: SHT10_Cal(uint16_t Temp,uint16_t Hum, float* pTempValue,float* pHumValue)
* �������: Temp:�ɼ����¶�ֵ��Hum���ɼ���ʪ��ֵ;
*           pTempValue������ת������¶�ֵ��pHumValue������ת�����ʪ��ֵ;
* �������: ��
*
* ��    ��: SHT1xϵ����ʪ��ֵ���ת�����¶Ȳ���
*
* ��    ��: 2016/3/18, by SongLei
************************************************************************************************************/
void SHT10_Cal(uint16_t Temp,uint16_t Hum, float* pTempValue,float* pHumValue)
{
  const float d1 = -39.7;      //�¶Ȳ�����������Ϊ��3.3v�ĵ�ѹ��
  const float d2 = 0.01;
  float Temp_C;                 //�¶ȡ�
  
  const float C1 = -2.0468;           // 12λʪ�Ⱦ��� ������ʽ       
  const float C2 = +0.0367;           // 12λʪ�Ⱦ��� ������ʽ       
  const float C3 = -0.0000015955;    // 12λʪ�Ⱦ��� ������ʽ     
  const float T1 = +0.01;             // 14λ�¶Ⱦ���  ������ʽ
  const float T2 = +0.00008;          // 14λ�¶Ⱦ���  ������ʽ
  
  float RH_Lin;        //ʪ������ֵ
  float RH_True;       //ʪ����ʵֵ
  
  //�¶Ƚ��������                 
  Temp_C = d1 + d2 * Temp;   //�¶���14λ
  
  //RH���Խ��               ʪ����12λ
  RH_Lin = C1 + C2 * Hum + C3 * Hum *Hum;     //���ʪ�ȷ����Բ���
  RH_True = (Temp_C - 25) * (T1 + T2 * Hum) + RH_Lin; //���ʪ�ȶ����¶������Բ���
  //�޶���Χ
  if( RH_True > 100 ) RH_True = 100;  //ʪ���������
  if( RH_True < 0.01) RH_True = 0.01; //ʪ����С����
  
  *pTempValue = Temp_C;               //�����¶Ƚ��
  *pHumValue = RH_True;               //����ʪ�Ƚ��
}

void  sensor_init(void)
{
  I2C_GPIO_Configuration();
  SHT1x_Init();
}

/***********************************************************************************************************
* ��������: Sht1x_Measure()
* �������: ��       
*               
* ���ز���: ��
*
* ��    ��: ��SHT1x��ʪ�ȴ��������в���
*
* ��    ��: 2016/3/19, by SongLei
************************************************************************************************************/
void Sht1x_Measure(void)
{
 
    
}
void restore_data()
{
  //modbus�ӻ���ַ
  usRegHoldingBuf[SLAVE_ID_OFF] = MSlaveID;
  usRegHoldingBuf[TEMPERATURE_DATA_OFF] = 0; 
  usRegHoldingBuf[HUMIDITY_DATA_OFF] = 0;
}

void get_sensor_data(void)
{
  uint16_t TempValue;  //�¶Ƚ��  16bit
  uint16_t HumValue;    //ʪ�Ƚ�� 16b  
  uint8_t error;
  Value Humi_val,Temp_val;  //����������ͬ�壬һ���洢ʪ�ȣ�һ���洢�¶�
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
      TempValue = (uint16_t)Temp_val.i;  //�¶�ת����16λ����
      HumValue  = (uint16_t)Humi_val.i;  //ʪ��ת����16λ����
      
      SHT10_Cal(TempValue ,HumValue,&Temp_val.f,&Humi_val.f);
#if defined(DEBUG)
      printf("�¶� %2.1fC ʪ�� %2.1f%%\n",Temp_val.f,Humi_val.f);
      printf("________________________________\n\n");
#endif
      usRegHoldingBuf[TEMPERATURE_DATA_OFF] = (unsigned int)(Temp_val.f * 10); 
      usRegHoldingBuf[HUMIDITY_DATA_OFF] = (unsigned int)(Humi_val.f * 10);
    }
}

