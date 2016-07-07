#ifndef __MQ2_H_
#define __MQ2_H_

#include "main.h"
extern uint16_t upper_range_value;
extern uint8_t  sensor_info_update_events;
#define  MSlaveID          0x08         

#define SLAVE_ID_OFF 0 
#define OOS_FLAG_OFF   1
#define GAS_DATA_OFF 2
#define UPPER_RANGE_OFF 3


//eeprom 地址
#define UPPER_RANGE_VALUE_ADDR 3      //占用两个字节
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address                0x40012440  //数据寄存器地址
#define Minisnse_Threshold             1000      //振动阀值 mv
#define Minisnse_Base                  850       //振动基数 mv      



void  sensor_init(void);
void restore_data(void);
void get_sensor_data(void);

void sensor_info_update(void);
#endif
