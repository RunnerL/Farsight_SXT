#ifndef __INFRARED_SENSOR_H__
#define __INFRARED_SENSOR_H__
#include "main.h"

#define INFRARED_PORT  GPIOB
#define INFRARED_PIN  GPIO_Pin_9
#define  MSlaveID          0x09         

#define SLAVE_ID_OFF 0
#define STATUS_OFF 1

void  sensor_init(void);
void restore_data();
void get_sensor_data(void);
#endif