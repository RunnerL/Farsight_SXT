#include "HCS_LTS.h"

__IO uint32_t TempSensVoltmv = 0, VrefIntVoltmv = 0,MiniSense100Voltmv = 0;
__IO uint32_t HCS_LTS_Voltmv = 0 ,HCS_LTS_Voltma = 0;
uint32_t ADC_Number = 0;
__IO uint16_t RegularConvData_Tab[3] = {0};
uint16_t current_value = 0;
uint16_t current_threshold_value = 0;
uint8_t  sensor_info_update_events = 0;  //不为1的时候代表保持寄存器内容更新,可以根据不同的位判断是寄存器中的哪个值发生了变化
uint16_t reference_voltage = 0;
/***********************************************************************************************************
* 函数名称: ADC1_CH_DMA_Config()
* 输入参数: 无
*               
* 返回参数: 无
*
* 功    能: 初始化AD0
*
* 作    者: 2016/3/21, by SongLei
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
* 函数名称: ADC_GPIO_Init()
* 输入参数: 无
*               
* 返回参数: 无
*
* 功    能: ADC--GPIO输入引脚配置---在此可以设置ADC-ANI0路外部输入通道
*
* 作    者: 2016/3/21, by SongLei
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
* 函数名称: MiniSense100_Init()
* 输入参数: 无
*               
* 返回参数: 无
*
* 功    能: 初始化振动传感器，对ADC引脚PA0的初始化
*
* 作    者: 2016/3/21, by SongLei
************************************************************************************************************/
void MiniSense100_Init(void)
{
  //ADC的引脚配置PA0
  ADC_GPIO_Init();
  /* ADC1 channel with DMA configuration */
  ADC1_CH_DMA_Config();
}

void  sensor_init(void)
{
 MiniSense100_Init();
}
/***********************************************************************************************************
* 函数名称: MiniSense100_Read()
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: --
*
* 作    者: 2016/3/21, by SongLei
************************************************************************************************************/


void restore_data()
{
  uint8_t val_buf[2] = {0};

  //modbus从机地址
  usRegHoldingBuf[SLAVE_ID_OFF] = MSlaveID;
  //是否超标
  usRegHoldingBuf[OOS_FLAG_OFF] = 0;
  //电流值
  usRegHoldingBuf[CURRENT_VALUE_OFF] = 0;
  //电流阀值从eeprom里边读取
  AT24CXX_Read(CURRENT_THRESHOLD_ADDR, val_buf, 2);
  current_threshold_value = (uint16_t)val_buf[0] << 8 | val_buf[1];
  
  if (current_threshold_value >= 500 && current_threshold_value <= 50000) 
    usRegHoldingBuf[CURRENT_THRESHOLD_OFF] = current_threshold_value;
  else {
    current_threshold_value = 3000;
    usRegHoldingBuf[CURRENT_THRESHOLD_OFF] = current_threshold_value;     
  }
}

void get_sensor_data(void)
{
  uint8_t i = 0 ;
  current_value = 0;
  MiniSense100Voltmv = 0;
  ADC_Number = 0;
  for (i = 0; i < 10; i++) {
    /* Test DMA1 TC flag */
    while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
    
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
    
    MiniSense100Voltmv  = (uint32_t)((RegularConvData_Tab[0]* 3300) / 0xFFF);
      
    ADC_Number += MiniSense100Voltmv;
  }
  ADC_Number = ADC_Number / 10;
  if (reference_voltage == 0) 
    reference_voltage = ADC_Number;
 
#if defined(DEBUG)
  printf("AdcNumber: %d\n",ADC_Number);
#endif

#if defined(DEBUG)
  if (ADC_Number <= reference_voltage)
    printf("电流: ADC  : %d ma\n",(reference_voltage - ADC_Number) * 10000 / 125);  
    else
      printf("电流: ADC  : %d ma\n",(ADC_Number - reference_voltage) * 10000 / 125);
        
#endif
    if (ADC_Number <= reference_voltage)
      current_value = (reference_voltage - ADC_Number) * 10000 / 125;
    else
      current_value = (ADC_Number - reference_voltage) * 10000 / 125;
    if (current_value < 500)
      current_value = 0;
    //将实时数据放入保持寄存器
    usRegHoldingBuf[CURRENT_VALUE_OFF] = current_value;
    //判断是否超标
    if (current_value >= usRegHoldingBuf[CURRENT_THRESHOLD_OFF]) 
      usRegHoldingBuf[OOS_FLAG_OFF] = 1;
    else
      usRegHoldingBuf[OOS_FLAG_OFF] = 0;
}

void sensor_info_update(void)
{
  uint8_t val_buf[2] = {0};
  if (sensor_info_update_events == 1) {
    val_buf[0] = (uint8_t)(usRegHoldingBuf[CURRENT_THRESHOLD_OFF] >> 8);
    val_buf[1] = (uint8_t)usRegHoldingBuf[CURRENT_THRESHOLD_OFF];
    current_threshold_value = (uint16_t)val_buf[0] << 8 | val_buf[1];
    
    if (current_threshold_value >= 500 && current_threshold_value <= 50000) 
      AT24CXX_Write(CURRENT_THRESHOLD_ADDR, val_buf, 2);
  
    sensor_info_update_events = 0;
  }
}
