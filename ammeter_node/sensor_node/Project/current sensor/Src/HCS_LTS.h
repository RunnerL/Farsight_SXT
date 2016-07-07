#ifndef __HCS_LTS_H_
#define __HCS_LTS_H_

#include "main.h"
extern uint16_t current_threshold_value;
extern uint8_t  sensor_info_update_events;
#define  MSlaveID          0x02         

#define SLAVE_ID_OFF 0 
#define OOS_FLAG_OFF   1
#define CURRENT_VALUE_OFF 2
#define CURRENT_THRESHOLD_OFF 3


//eeprom ��ַ
#define CURRENT_THRESHOLD_ADDR 3      //ռ�������ֽ�
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address                0x40012440  //���ݼĴ�����ַ
#define Minisnse_Threshold             1000      //�񶯷�ֵ mv
#define Minisnse_Base                  850       //�񶯻��� mv      



void  sensor_init(void);
void restore_data(void);
void get_sensor_data(void);

void sensor_info_update(void);
#endif
