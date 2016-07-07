#ifndef __DOOR_LOCK_H
#define __DOOR_LOCK_H

#include "stdint.h"

#define LOCK_CMD_SIZE 34

void get_sensor_data(void);
void send_lock_cmd(uint8_t *cmd);
void get_door_lock_msgs(uint16_t *usRegBuf, uint8_t *lock_cmd);
void SetAndSendRemoteOpenDoorCmd(void);
uint8_t Timer2_Get_Sensor_Data(void);


#endif
