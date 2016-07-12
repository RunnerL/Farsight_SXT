/*************************************************************************
*�Ž�ϵͳ�����汾��V88
*�Ž�������S/N��12914
*V88���У��ڶ�ȡ�Ž�ϵͳ����״̬��ʱ�򣬷��ص�������е�26�ֽڱ�ʾ��¼״̬λ-->
*Ҫ�ж��ŵ�״̬��Ҫʹ����������+��¼״̬λ���ж��������88�汾��Э���ֲ�9.2.2.2
*V53���У��ڶ�ȡ�Ž�ϵͳ����״̬��ʱ�򣬷��ص�������е�26�ֽڱ�ʾ�̵���״̬-->
*���ԣ��ж��ŵ�״̬��ʱ��ֱ�Ӳ�ѯ��λ���ɡ�

*����=(��19�ֽ� << 16) + (��18�ֽ� << 8) + ��17�ֽ�
**************************************************************************/


#include "stdint.h"
#include "door_lock.h"
#include "stm32f0xx.h"
#include "FS_FIFO_V1.h"
#include "main.h"

struct FIFOqueue lock_fifo;
uint16_t Door_state = 0; //����ŵ�״̬������ ��
uint32_t Index = 0; //������µ�����λ
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
���ܣ����ڴ�ӡ
��������Ҫ��ӡ�������׵�ַ
����ֵ����
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
���ܣ���modbus���ּĴ�����ֵ
��������
����ֵ����
***************************************************/
void get_door_lock_msgs(uint16_t *usRegBuf, uint8_t *lock_cmd)
{
  usRegBuf[0] = 0x0000;//Modbus�������͸��ӻ�������
  usRegBuf[1] = lock_cmd[19];//�Ž�������
  usRegBuf[2] = (lock_cmd[18] << 8) + lock_cmd[17];//�Ž���ID��
  usRegBuf[3] = lock_cmd[20];//��¼״̬
  usRegBuf[4] = Door_state;//�̵���״̬
  usRegBuf[5] = lock_cmd[26];//��¼״̬λ 
}
/**************************************************
���ܣ���ȡ���µĲ�ѯָ��
���������µ�������ַ
����ֵ�����µ�ָ���׵�ַ
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
���ܣ����Ž�ϵͳ��������
������������׵�ַ
����ֵ����
***************************************************/
void send_lock_cmd(uint8_t *cmd)
{
  send_data(USART2, cmd, LOCK_CMD_SIZE);
}
/**************************************************
���ܣ���ȡ�豸����
��������
����ֵ����
***************************************************/
void get_sensor_data(void){
  Get_Search_Cmd(Index);
  send_lock_cmd(Lock_Command_To_Search);
}
/**************************************************
���ܣ���ȡ�ŵ�״̬
��������
����ֵ���ŵ�״ֵ̬ 1 ��ʾ����  0 ��ʾ����
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
���ܣ���ʱ����ʱ��ȡ�豸����
��������
����ֵ��0 �ɹ� 1 ʧ��
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
���ܣ������Ž�����״̬��������Զ�̿���ָ��
��������
����ֵ��0 �ɹ� 1 ʧ��
***************************************************/
void SetAndSendRemoteOpenDoorCmd(void)
{
//  send_lock_cmd(Lock_Command_Set_Para);
  send_lock_cmd(Lock_Command_Remote_Open_Door);
}