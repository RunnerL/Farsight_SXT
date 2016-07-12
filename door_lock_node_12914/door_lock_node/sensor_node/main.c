/*****************ModBus������******************�ӵ�ַ(ID)
����������	        B	                 00 02
���մ�����        	L                        00 04
����������		S                        00 05
��ʪ�ȴ�����		T                        00 06
�񶯴�����	        V	                 00 07
��ȼ�崫����	        G	                 00 08
�����դ������	        I	                 00 09
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mb.h"
#include "mbutils.h"
#include "string.h"
#include "stm32f0xx_tim.h"
#include "FS_FIFO_V1.h"
#include "door_lock.h"

#define CHECKSUM 1

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/** @addtogroup Template_Project
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;
volatile uint8_t get_data_flag = 0;
extern struct FIFOqueue lock_fifo;
//���ּĴ�������
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0};
//���ּĴ�����ʼ��ַ
uint16_t usRegHoldingStart = REG_HOLDING_START;

extern uint8_t Door_Flag;
extern uint8_t Door_Flag1 ;
extern uint16_t DoorIDL;
extern uint16_t DoorIDH;


/* Private functions ---------------------------------------------------------*/
void send_data(USART_TypeDef* USARTx, uint8_t* pbuf, uint8_t bytes);
void Usart_Init(USART_TypeDef* USARTx,uint32_t ulBaudRate);
uint8_t at24xx_check(void);
void timer2_init(void);
void timer14_init(void);
void zigbee_send_data(uint16_t* pbuf, uint8_t bytes);




void beep_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStruct.GPIO_Pin = BEEP_PIN  ;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(BEEP_PORT, &GPIO_InitStruct);
  GPIO_SetBits(BEEP_PORT,BEEP_PIN );
}


void board_init(void)
{
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  timer2_init();
  timer14_init();
  Usart_Init(USART2,9600);
  send_data(USART2, "uart2 is OK!\r\n", strlen("uart2 is OK!\r\n"));
  //��ͬ�Ĵ����������ż��õ��İ弶���費ͬ
  //sensor_init();
#if defined (BEEP)
  //���������������豸
  beep_init();
  Delay(20);
  BEEP_OFF();
#endif
#if defined (DEBUG)
  //�����������Զ˿�
  Usart_Init(DEBUG_UART,115200);
  send_data(DEBUG_UART, "uart is OK!\r\n", strlen("uart is OK!\r\n"));
#endif
#if defined (EEPROM)
  //24cxx IIC�ӿڳ�ʼ��
  IIC_Init();
#endif
} 

void system_init(void)
{
#if defined(EEPROM)
  //���24cxx�洢оƬ�Ƿ�����
  while (at24xx_check()){
#if defined (BEEP)
    //��������
    BEEP_TOGGLE();
#endif
#if defined (DEBUG)
    send_data(DEBUG_UART, "24cxx check fail!!\r\n", strlen("24cxx check fail!!\r\n"));
#endif
    Delay(100);
  }
  //�ָ�����
  restore_data();
#endif
}

int main(void)
{
  board_init();
  system_init();
  QueueInit(&lock_fifo);
  //modbus485���ߵ�����
  eMBInit(MB_RTU, MSlaveID, MODBUS_UART_NUMBER, MODBUS_UART_BAUDRATE, MB_PAR_NONE); 
  eMBEnable();
  while (1) {
    
    if(Timer2_Get_Sensor_Data() == 0){
      QueueInit(&lock_fifo);
      if(Door_Flag1 == 1)
    {
        SetAndSendRemoteOpenDoorCmd();
        
        Door_Flag1 = 0;
    }      
    }
    
  }
}

uint8_t at24xx_check(void)
{
  uint8_t status = 0;
  I2C_Read_one_Byte(&status, 255, 0xa0);			   
  if (status == 0X66) 
    return 0;
  else{
    I2C_Write_one_Byte(0x66, 255, 0xa0);
    I2C_Read_one_Byte(&status, 255, 0xa0);	  
    if (status == 0X66)
      return 0;
  }
  return 1;
}
/**
* @brief  ���ּĴ��������������ּĴ����ɶ����ɶ���д
* @param  pucRegBuffer  ������ʱ--��������ָ�룬д����ʱ--��������ָ��
*         usAddress     �Ĵ�����ʼ��ַ
*         usNRegs       �Ĵ�������
*         eMode         ������ʽ��������д
* @retval eStatus       �Ĵ���״̬
*/
eMBErrorCode 
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                eMBRegisterMode eMode )
{
  //����״̬
  eMBErrorCode    eStatus = MB_ENOERR;
  //ƫ����
  int16_t         iRegIndex;
  usAddress--;      // ��ʼ��ַΪ0����Ҫ��仰�������ʼ��ַΪ1����PLCϵͳ����ע�͵���仰
  
  //�жϼĴ����ǲ����ڷ�Χ��
  if( ( usAddress >= REG_HOLDING_START ) \
    && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
  {
    //����ƫ����
    iRegIndex = ( int16_t )( usAddress - usRegHoldingStart );
    
    switch ( eMode )
    {
      //��������  
    case MB_REG_READ:
      while( usNRegs > 0 )
      {
        *pucRegBuffer++ = ( uint8_t )( usRegHoldingBuf[iRegIndex] >> 8 );
        *pucRegBuffer++ = ( uint8_t )( usRegHoldingBuf[iRegIndex] & 0xFF );
        if(Door_Flag == 1)
        {
//            usRegHoldingBuf[0] = 0x0000;//Modbus�������͸��ӻ�������
            usRegHoldingBuf[1] = DoorIDH;//�Ž�������
            usRegHoldingBuf[2] = DoorIDL;//�Ž���ID��
            usRegHoldingBuf[3] = 0;//��¼״̬
            usRegHoldingBuf[4] = 1;//�̵���״̬
            usRegHoldingBuf[5] = 0;//��¼״̬λ 
          Door_Flag = 0;
        }
        iRegIndex++;
        usNRegs--;
      }
      break;
      
      //д������ 
    case MB_REG_WRITE:
      while( usNRegs > 0 )
      {
        usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
        usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
        iRegIndex++;
        usNRegs--;
        
        if(usRegHoldingBuf[0] == 0x0031){
          Door_Flag1 = 1;
          usRegHoldingBuf[0] == 0x0000;
    }
      }
      //*********************************************�ı����ݵ�ʱ��Ӧ��д��EEPROM*************************************************************/
      
    }
  }
  else
  {
    //���ش���״̬
    eStatus = MB_ENOREG;
  }
  
  return eStatus;
}
void zigbee_send_data(uint16_t* pbuf, uint8_t bytes)
{
  uint8_t i = 0;
  for(i = 0; i < bytes; i++)
  { 
    USART_SendData(USART1, (uint8_t)(pbuf[i] >> 8));
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
    
    USART_SendData(USART1, (uint8_t)(pbuf[i]));
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
  }
}

void send_data(USART_TypeDef* USARTx, uint8_t* pbuf, uint8_t bytes)
{
  int i = 0;
  for(i = 0; i < bytes; i++)
  { 
    USART_SendData(USARTx, pbuf[i]);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    {}
  }
}

void Usart_Init(USART_TypeDef* USARTx,uint32_t ulBaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  if(USARTx==USART1)   //ʹ��USART1,GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  else
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  if(USARTx==USART1)
  {
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  }else
  {
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  }
  
  if(USARTx==USART1)
    //GPIOA.2 USART2_Tx,GPIOA.3 USART2_Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  else
    //GPIOA.2 USART1_Tx,GPIOA.3 USART1_Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = ulBaudRate;            //������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //���ڳ�ʼ��
  USART_Init(USARTx, &USART_InitStructure);
  
  NVIC_InitTypeDef NVIC_InitStructure; 
  //�趨USART2 ���ȼ� 
  if(USARTx==USART1) 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
  else 
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 

  NVIC_InitStructure.NVIC_IRQChannelPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USARTx, ENABLE);
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
  

}


void timer2_init(void)
{     	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t PrescalerValue = 0;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  PrescalerValue = (uint16_t) ((SystemCoreClock / 20000) - 1); 
  TIM_TimeBaseStructure.TIM_Period = GET_DATA_INTEVAL * 20;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  //Ԥװ��ʹ��
  TIM_ARRPreloadConfig(TIM2, ENABLE); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������жϱ�־λ
  TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}


void timer14_init(void)
{     	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t PrescalerValue = 0;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
  PrescalerValue = (uint16_t) ((SystemCoreClock / 20000) - 1); 
  TIM_TimeBaseStructure.TIM_Period = 200;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
  //Ԥװ��ʹ��
  TIM_ARRPreloadConfig(TIM14, ENABLE); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������жϱ�־λ
  TIM_ClearITPendingBit(TIM14,TIM_IT_Update);
  TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM14, ENABLE);
}

/**
* @brief  ����Ĵ���������������Ĵ����ɶ���������д��
* @param  pucRegBuffer  ��������ָ��
*         usAddress     �Ĵ�����ʼ��ַ
*         usNRegs       �Ĵ�������
* @retval eStatus       �Ĵ���״̬
*/
eMBErrorCode 
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode    eStatus = MB_ENOERR;
//  int16_t         iRegIndex;
//  
//  usAddress--;      // ��ʼ��ַΪ0����Ҫ��仰�������ʼ��ַΪ1����PLCϵͳ����ע�͵���仰
//  
//  //��ѯ�Ƿ��ڼĴ�����Χ��
//  //Ϊ�˱��⾯�棬�޸�Ϊ�з�������
//  if( ( (int16_t)usAddress >= REG_INPUT_START ) \
//    && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
//  {
//    //��ò���ƫ���������β�����ʼ��ַ-����Ĵ����ĳ�ʼ��ַ
//    iRegIndex = ( int16_t )( usAddress - usRegInputStart );
//    //�����ֵ
//    while( usNRegs > 0 )
//    {
//      //��ֵ���ֽ�
//      *pucRegBuffer++ = ( uint8_t )( usRegInputBuf[iRegIndex] >> 8 );
//      //��ֵ���ֽ�
//      *pucRegBuffer++ = ( uint8_t )( usRegInputBuf[iRegIndex] & 0xFF );
//      //ƫ��������
//      iRegIndex++;
//      //�������Ĵ��������ݼ�
//      usNRegs--;
//    }
//  }
//  else
//  {
//    //���ش���״̬���޼Ĵ���  
//    eStatus = MB_ENOREG;
//  }
//  
  return eStatus;
}




/**
* @brief  ��Ȧ�Ĵ�������������Ȧ�Ĵ����ɶ����ɶ���д
* @param  pucRegBuffer  ������---��������ָ�룬д����--��������ָ��
*         usAddress     �Ĵ�����ʼ��ַ
*         usNRegs       �Ĵ�������
*         eMode         ������ʽ��������д
* @retval eStatus       �Ĵ���״̬
*/
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode )
{
//  //����״̬
  eMBErrorCode    eStatus = MB_ENOERR;
//  //�Ĵ�������
//  int16_t         iNCoils = ( int16_t )usNCoils;
//  //�Ĵ���ƫ����
//  int16_t         usBitOffset;
//  
//  usAddress--;      // ��ʼ��ַΪ0����Ҫ��仰�������ʼ��ַΪ1����PLCϵͳ����ע�͵���仰
//  
//  //���Ĵ����Ƿ���ָ����Χ��
//  if( ( (int16_t)usAddress >= REG_COILS_START ) &&
//     ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
//  {
//    //����Ĵ���ƫ����
//    usBitOffset = ( int16_t )( usAddress - REG_COILS_START );
//    switch ( eMode )
//    {
//      //������
//    case MB_REG_READ:
//      while( iNCoils > 0 )
//      {
//        *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
//                                         ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ) );
//        iNCoils -= 8;
//        usBitOffset += 8;
//      }
//      break;
//      
//      //д����
//    case MB_REG_WRITE:
//      while( iNCoils > 0 )
//      {
//        xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
//                       ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),
//                       *pucRegBuffer++ );
//        iNCoils -= 8;
//      }
//      break;
//    }
//    
//  }
//  else
//  {
//    eStatus = MB_ENOREG;
//  }
  return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
//  //����״̬
  eMBErrorCode    eStatus = MB_ENOERR;
//  //�����Ĵ�������
//  int16_t         iNDiscrete = ( int16_t )usNDiscrete;
//  //ƫ����
//  uint16_t        usBitOffset;
//  
//  usAddress--;      // ��ʼ��ַΪ0����Ҫ��仰�������ʼ��ַΪ1����PLCϵͳ����ע�͵���仰
//  
//  //�жϼĴ���ʱ�����ƶ���Χ��
//  if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
//     ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
//  {
//    //���ƫ����
//    usBitOffset = ( uint16_t )( usAddress - REG_DISCRETE_START );
//    
//    while( iNDiscrete > 0 )
//    {
//      *pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
//                                       ( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );
//      iNDiscrete -= 8;
//      usBitOffset += 8;
//    }
//    
//  }
//  else
//  {
//    eStatus = MB_ENOREG;
//  }
  return eStatus;
}

/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}
  return ch;
}

/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in 10 ms.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/
/*
  uint8_t a = 0x33;
  uint8_t *pbuf1 = "eeprom test!!!!@#$%^&**(())_+\r\n", buf[32] = 0;
#ifdef DEBUG
//24c02�����ֽ�д�����
#if 0   
  I2C_Write_one_Byte(0x97, 0, 0xa0);
  I2C_Read_one_Byte(&a, 0, 0xa0);
  send_data(DEBUG_UART, &a, 1);
#else
  AT24CXX_Write(0,pbuf1,strlen(pbuf1));
  AT24CXX_Read(0, buf, strlen(pbuf1));
  send_data(DEBUG_UART, buf, strlen(pbuf1));
#endif
#endif
*/
/************************ (C) COPYRIGHT Huatai Air tech Ltd.,Co *****END OF FILE****/
