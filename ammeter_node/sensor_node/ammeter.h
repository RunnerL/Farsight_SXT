#ifndef __AMMETER_H
#define __AMMETER_H

#include "stdint.h"

#define AMT_CMD_SIZE 14

uint8_t ammeter_command[AMT_CMD_SIZE] = {
        0x68,   //start
        0x68,   //----------¡¢
        0x04,   //          |
        0x14,   //    device addr
        0x00,   //          |
        0x80,   //          |
        0x13,   //----------|
        0x68,   //identification
        0x01,   //command
        0x02,   //length
        0x43,   //----------¡¢
        0xc3,   //----------|data identification
        0xec,   //checksum
        0x16    //stop
};


#endif