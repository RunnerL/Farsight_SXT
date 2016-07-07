#include "mq2.h"

__IO uint32_t TempSensVoltmv = 0, VrefIntVoltmv = 0,MiniSense100Voltmv = 0;
__IO uint32_t HCS_LTS_Voltmv = 0 ,HCS_LTS_Voltma = 0;
uint32_t ADC_Number = 0;
__IO uint16_t RegularConvData_Tab[3] = {0};
uint16_t MQ2_Voltmv = 0,MQ2_PPm = 0;
uint16_t upper_range_value = 0;
uint8_t  sensor_info_update_events = 0;  //不为1的时候代表保持寄存器内容更新,可以根据不同的位判断是寄存器中的哪个值发生了变化
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
  //气体浓度值
  usRegHoldingBuf[GAS_DATA_OFF] = 0;
  //浓度上限,从eeprom里边读取
  AT24CXX_Read(UPPER_RANGE_VALUE_ADDR, val_buf, 2);
  upper_range_value = (uint16_t)val_buf[0] << 8 | val_buf[1];
  
  if (upper_range_value >= 300 && upper_range_value <= 8500) 
    usRegHoldingBuf[UPPER_RANGE_OFF] = upper_range_value;
  else {
    upper_range_value = 3000;
    usRegHoldingBuf[UPPER_RANGE_OFF] = upper_range_value;     
  }
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
    MQ2_Voltmv  = ADC_Number * 2;  //mv
	if(MQ2_Voltmv <1750)
      MQ2_PPm = 300;
	      
    if(1500 < MQ2_Voltmv && MQ2_Voltmv <= 2000 )
      MQ2_PPm = (MQ2_Voltmv - 1300) * (480 - 300) / (2000 - 1750);
    
    if(2000 < MQ2_Voltmv && MQ2_Voltmv <= 2500 )
      MQ2_PPm = (MQ2_Voltmv - 1400) * (750 - 480) / (2500 - 2000);
       
    if(2500 < MQ2_Voltmv && MQ2_Voltmv <= 3000 )
      MQ2_PPm = (MQ2_Voltmv - 1750) * (1500 - 750) / (3000 - 2500);
    
    if(3000 < MQ2_Voltmv && MQ2_Voltmv <= 3500 )
      MQ2_PPm = (MQ2_Voltmv - 1750) * (2600 - 1500) / (3500 - 3000);
    
    if(3500 < MQ2_Voltmv && MQ2_Voltmv <= 4000 )
      MQ2_PPm = (MQ2_Voltmv - 2500) * (4200 - 2600) / (4000 - 3500);
    
    if(4000 < MQ2_Voltmv && MQ2_Voltmv <= 4500 )
      MQ2_PPm = (MQ2_Voltmv - 3500) * (8500 - 4200) / (4500 - 4000);
	  
    if(MQ2_Voltmv > 4500)
      MQ2_PPm = 8500;
#if defined(DEBUG)    
    printf("烟雾: PPm  : %d\n",MQ2_PPm);
#endif
    
    //将实时数据放入保持寄存器
    usRegHoldingBuf[GAS_DATA_OFF] = (uint16_t)MQ2_PPm;
    //判断是否超标
    if (usRegHoldingBuf[UPPER_RANGE_OFF] <= (uint16_t)MQ2_PPm) 
      usRegHoldingBuf[OOS_FLAG_OFF] = 1;
    else
      usRegHoldingBuf[OOS_FLAG_OFF] = 0;
    
    MQ2_Voltmv = 0; MQ2_PPm = 0;
}

void sensor_info_update(void)
{
  uint8_t val_buf[2] = {0};
  if (sensor_info_update_events == 1) {
    val_buf[0] = (uint8_t)(usRegHoldingBuf[UPPER_RANGE_OFF] >> 8);
    val_buf[1] = (uint8_t)usRegHoldingBuf[UPPER_RANGE_OFF];
    upper_range_value = (uint16_t)val_buf[0] << 8 | val_buf[1];
    
    if (upper_range_value >= 300 && upper_range_value <= 8500) 
      AT24CXX_Write(UPPER_RANGE_VALUE_ADDR, val_buf, 2);
  
    sensor_info_update_events = 0;
  }
}
