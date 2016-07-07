#ifndef __DOOR_LOCK_H
#define __DOOR_LOCK_H

#include "stdint.h"

#define LOCK_CMD_SIZE 34

void get_sensor_data(void);
void send_lock_cmd(uint8_t *cmd);
void get_door_lock_msgs(uint16_t *usRegBuf, uint8_t *lock_cmd);


//this is a search command to get id of door_lock os;
uint8_t Lock_Command_To_Search[LOCK_CMD_SIZE] = {
              0x7e, //start
              0x9d, //--------|
              0x32, //--------|board addr
              0x81, //--------|
              0x10, //--------|function bits
              0x00, //----------------------\__________________
              0x00, //                       |                |
              0x00, //             The latest index bis       |
              0x00, //                       |                |
              0x00, //----------------------/                 |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                    Data area
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, //                                        |
              0x00, // _______________________________________| 
              0x60, //---------|
              0x01, //---------|The checksum
              0x0d  //Stop
};


#endif
