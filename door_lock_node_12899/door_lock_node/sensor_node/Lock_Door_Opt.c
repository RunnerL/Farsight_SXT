/*************************************************************************
*门禁系统驱动版本：V88
*门禁控制器S/N：12914
*V88版中，在读取门禁系统运行状态的时候，返回的命令包中第26字节表示记录状态位-->
*要判断门的状态需要使用完整卡号+记录状态位来判定，具体见88版本的协议手册9.2.2.2
*V53版中，在读取门禁系统运行状态的时候，返回的命令包中第26字节表示继电器状态-->
*所以，判断门的状态的时候直接查询该位即可。

*卡号=(第19字节 << 16) + (第18字节 << 8) + 第17字节
**************************************************************************/


#include "stdint.h"
#include "door_lock.h"
#include "stm32f0xx.h"
#include "FS_FIFO_V1.h"
#include "main.h"

struct FIFOqueue lock_fifo;
uint16_t Door_state = 0; //存放门的状态，开， 关
uint32_t Index = 0; //存放最新的索引位
uint8_t Door_Flag = 0;
uint8_t Door_Flag1 = 0;
uint16_t DoorIDL = 0;
uint16_t DoorIDH = 0;

extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

//this is a search command to get id of door_lock os;
uint8_t Lock_Command_To_Search[LOCK_CMD_SIZE] = {
              0x7e, //start
              0x63, //--------|
              0x32, //--------|board addr  SN:12914
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
              0x26, //---------|
              0x01, //---------|The checksum
              0x0d  //Stop
};

//set lock door para is on_line
uint8_t Lock_Command_Set_Para[LOCK_CMD_SIZE] = {
              0x7e,
              0x63,
              0x32,
              0x8f,
              0x10,
              0x01,//----------door num (1-4)
              0x03,//----------control ways(1:always open  2:always close  3:on_line)
              0x1e,//---------|
              0x00,//---------/delay(0.1s as a unit, here is 3s)
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x56,
              0x01,
              0x0d
};

//the remote to open the door
uint8_t Lock_Command_Remote_Open_Door[LOCK_CMD_SIZE] = {
              0x7e,
              0x63,
              0x32,
              0x9d,
              0x10,
              0x01,//----------door num
              0x01,//----------open the door
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x00,
              0x44,
              0x01,
              0x0d
};
/**************************************************
功能：串口打印
参数：需要打印的数据首地址
返回值：无
***************************************************/
void UART_SHOW(uint8_t *cmd){
   uint8_t num;
   for(num=0;num<34;num++)
   {
      printf("%02x ",cmd[num]);
   }
      printf("\n");
}
              
/**************************************************
功能：给modbus保持寄存器赋值
参数：无
返回值：无
***************************************************/
void get_door_lock_msgs(uint16_t *usRegBuf, uint8_t *lock_cmd)
{
  usRegBuf[0] = 0x0000;//Modbus主机发送给从机的命令
  usRegBuf[1] = lock_cmd[19];//门禁卡区号
  usRegBuf[2] = (lock_cmd[18] << 8) + lock_cmd[17];//门禁卡ID号
  usRegBuf[3] = lock_cmd[20];//记录状态
  usRegBuf[4] = Door_state;//继电器状态
  usRegBuf[5] = lock_cmd[26];//记录状态位 
}
/**************************************************
功能：获取最新的查询指令
参数：最新的索引地址
返回值：最新的指令首地址
***************************************************/
void Get_Search_Cmd(uint32_t index)
{
  uint8_t i;
  uint16_t check = 0;
  Lock_Command_To_Search[5] = index;
  Lock_Command_To_Search[6] = index >> 8;
  Lock_Command_To_Search[7] = index >> 16;
  for(i=1; i<31; i++)
  {
    check += Lock_Command_To_Search[i];  
  }
  Lock_Command_To_Search[31] = check;
  Lock_Command_To_Search[32] = (check >> 8);
}
/**************************************************
功能：给门禁系统发送命令
参数：命令包首地址
返回值：无
***************************************************/
void send_lock_cmd(uint8_t *cmd)
{
  send_data(USART2, cmd, LOCK_CMD_SIZE);
}
/**************************************************
功能：获取设备数据
参数：无
返回值：无
***************************************************/
void get_sensor_data(void){
  Get_Search_Cmd(Index);
  send_lock_cmd(Lock_Command_To_Search);
}
/**************************************************
功能：获取门的状态
参数：无
返回值：门的状态值 1 表示开门  0 表示关门
***************************************************/
uint16_t Get_State_Of_Door(uint8_t *cmd){
  uint32_t id = 0;
  uint16_t state = 0;
  
  id = (cmd[19] << 16) + (cmd[18] << 8) + cmd[17];
//  printf("id: %d\n",id);
  
  if(((id > 100) || (id == 0) || (id == 5)) && (cmd[25] == 0)){
    state = 1;
  }
  else 
    state = 0;
  
  return state;
}
/**************************************************
功能：定时器定时获取设备数据
参数：无
返回值：0 成功 1 失败
***************************************************/
uint8_t Timer2_Get_Sensor_Data(void)
{
    uint8_t num;
    uint16_t checksum;
    if (lock_fifo.count >= 34) {
      checksum = 0;
      for(num=1;num<31;num++)
      {
        checksum += lock_fifo.dat[num];
      }
      if(((checksum & 0x0FF) == lock_fifo.dat[31]) && ((checksum >> 8) == lock_fifo.dat[32])){
        Index = (lock_fifo.dat[14] << 16) + (lock_fifo.dat[13] << 8) + lock_fifo.dat[12] + 1;
        Door_state = Get_State_Of_Door(lock_fifo.dat);
        get_door_lock_msgs(usRegHoldingBuf,lock_fifo.dat);
        
        if(usRegHoldingBuf[4] == 1)
        {
          DoorIDH = usRegHoldingBuf[1];
          DoorIDL = usRegHoldingBuf[2];
          Door_Flag = 1;
          
        }
        return 0;
      }
      else
      {
        return 1;
      }
    }
    else
    {
        return 1;
    }    
}
/**************************************************
功能：设置门禁在线状态，并发送远程开锁指令
参数：无
返回值：0 成功 1 失败
***************************************************/
void SetAndSendRemoteOpenDoorCmd(void)
{
//  send_lock_cmd(Lock_Command_Set_Para);
  send_lock_cmd(Lock_Command_Remote_Open_Door);
}