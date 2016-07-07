#include "vibration.h"

__IO uint32_t TempSensVoltmv = 0, VrefIntVoltmv = 0,MiniSense100Voltmv = 0;
__IO uint32_t HCS_LTS_Voltmv = 0 ,HCS_LTS_Voltma = 0;
uint32_t ADC_Number = 0;
__IO uint16_t RegularConvData_Tab[3] = {0};
uint16_t MQ2_Voltmv = 0,MQ2_PPm = 0;
uint16_t upper_range_value = 0;
uint8_t  sensor_info_update_events = 0;  //��Ϊ1��ʱ������ּĴ������ݸ���,���Ը��ݲ�ͬ��λ�ж��ǼĴ����е��ĸ�ֵ�����˱仯
/***********************************************************************************************************
* ��������: ADC1_CH_DMA_Config()
* �������: ��
*               
* ���ز���: ��
*
* ��    ��: ��ʼ��AD0
*
* ��    ��: 2016/3/21, by SongLei
************************************************************************************************************/
void ADC1_CH_DMA_Config(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;
  
  /* ADC1 DeInit */  
  ADC_DeInit(ADC1);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
  
  RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4); 
  
  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC DMA request in circular mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  
  /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);  
  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_Init(ADC1, &ADC_InitStructure); 
 
//  /* Convert the ADC1 temperature sensor  with 55.5 Cycles as sampling time */ 
//  ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor , ADC_SampleTime_55_5Cycles);  
//  ADC_TempSensorCmd(ENABLE);
//  
//  /* Convert the ADC1 Vref  with 55.5 Cycles as sampling time */ 
//  ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint , ADC_SampleTime_55_5Cycles); 
//  ADC_VrefintCmd(ENABLE);
  
  ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_55_5Cycles); 
//  ADC_VrefintCmd(ENABLE);
  
  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADCEN falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);

}

/***********************************************************************************************************
* ��������: ADC_GPIO_Init()
* �������: ��
*               
* ���ز���: ��
*
* ��    ��: ADC--GPIO������������---�ڴ˿�������ADC-ANI0·�ⲿ����ͨ��
*
* ��    ��: 2016/3/21, by SongLei
************************************************************************************************************/
void ADC_GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
     
     GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
     GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/***********************************************************************************************************
* ��������: MiniSense100_Init()
* �������: ��
*               
* ���ز���: ��
*
* ��    ��: ��ʼ���񶯴���������ADC����PA0�ĳ�ʼ��
*
* ��    ��: 2016/3/21, by SongLei
************************************************************************************************************/
void MiniSense100_Init(void)
{
  //ADC����������PA0
  ADC_GPIO_Init();
  /* ADC1 channel with DMA configuration */
  ADC1_CH_DMA_Config();
}

void  sensor_init(void)
{
 MiniSense100_Init();
}
/***********************************************************************************************************
* ��������: MiniSense100_Read()
* �������: 
*               
* ���ز���: 
*
* ��    ��: --
*
* ��    ��: 2016/3/21, by SongLei
************************************************************************************************************/


void restore_data()
{
  uint8_t val_buf[2] = {0};

  //modbus�ӻ���ַ
  usRegHoldingBuf[SLAVE_ID_OFF] = MSlaveID;
  //�Ƿ񳬱�
  usRegHoldingBuf[OOS_FLAG_OFF] = 0;
  //����Ũ��ֵ
//  usRegHoldingBuf[GAS_DATA_OFF] = 0;
//  //Ũ������,��eeprom��߶�ȡ
//  AT24CXX_Read(UPPER_RANGE_VALUE_ADDR, val_buf, 2);
//  upper_range_value = (uint16_t)val_buf[0] << 8 | val_buf[1];
//  
//  if (upper_range_value >= 300 && upper_range_value <= 8500) 
//    usRegHoldingBuf[UPPER_RANGE_OFF] = upper_range_value;
//  else {
//    upper_range_value = 3000;
//    usRegHoldingBuf[UPPER_RANGE_OFF] = upper_range_value;     
//  }
}

void get_sensor_data(void)
{
  uint8_t i = 0 ;
  for (i = 0; i < 10; i++) {
    /* Test DMA1 TC flag */
    while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
    
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
    
    MiniSense100Voltmv  = (uint32_t)((RegularConvData_Tab[0]* 3300) / 0xFFF);
      
    ADC_Number += MiniSense100Voltmv;
  }
  ADC_Number = ADC_Number / 10;
#if defined(DEBUG)
  printf("AdcNumber: %d\n",ADC_Number);
#endif
    if((int16_t)(ADC_Number - Minisnse_Base) < 0 )
      ;
    else if((int16_t)(ADC_Number - Minisnse_Base) > Minisnse_Threshold) {   
#if defined(DEBUG)
        printf("����:%d\n",(ADC_Number - Minisnse_Base));
#endif
    }
}

void sensor_info_update(void)
{
//  uint8_t val_buf[2] = {0};
//  if (sensor_info_update_events == 1) {
//    val_buf[0] = (uint8_t)(usRegHoldingBuf[UPPER_RANGE_OFF] >> 8);
//    val_buf[1] = (uint8_t)usRegHoldingBuf[UPPER_RANGE_OFF];
//    upper_range_value = (uint16_t)val_buf[0] << 8 | val_buf[1];
//    
//    if (upper_range_value >= 300 && upper_range_value <= 8500) 
//      AT24CXX_Write(UPPER_RANGE_VALUE_ADDR, val_buf, 2);
//  
//    sensor_info_update_events = 0;
//  }
}
