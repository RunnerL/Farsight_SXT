#ifndef __FPM_FINGER_H
#define __FPM_FINGER_H

#include "stdint.h"

/* 确认码定义表 */
#define COMMAND_OK 	        			0x00 //表示指令执行完毕或OK
#define RECV_PACKAGE_ERR  				0x01 //表示数据包接收错误
#define NO_FINGER           			0x02 //表示传感器上没有手指
#define INPUT_FINGER_IMAGE_ERR		0x03 //表示录入指纹图像失败
#define FINGER_IMAGE_CHAOS      	0x06 //表示指纹图像太乱而生不成特征
#define FEATURE_POINT_LESS				0x07 //表示指纹图像正常，但特征点太少（或面积太小）而生不成特征
#define FINGER_NOT_MATCH					0x08 //表示指纹不匹配
#define REACH_NO_FINGER						0x09 //表示没搜索到指纹
#define FEATURE_COMBINE_FAIL			0x0A //表示特征合并失败
#define ADDR_NUM_OVERSTEP       	0x0B //表示访问指纹库时地址序号超出指纹库范围
#define READ_TEMPLATE_FAIL      	0x0C //表示从指纹库读模板出错或无效
#define UPLOAD_FEATURE_FAIL     	0x0D //表示上传特征失败
#define MODER_CANOT_RECV_PACKAGE	0x0E //表示模块不能接受后续数据包
#define UPLOAD_IMAGE_FAIL         0x0F //表示上传图像失败
#define DELATE_TEMPLATE_FAIL      0x10 //表示删除模板失败
#define CLEAN_FINGERLIB_EMPTY     0x11 //表示清空指纹库失败
#define COMMAND_ISNOT_CURRENT     0x13 //表示口令不正确
#define NO_EFFACT_IMAGE_IN_BUFF   0x15 //表示缓冲区内没有有效原始图而生不成图像
#define READ_WRITE_FLASH_ERROR    0x18 //表示读写FLASH出错
#define UNDEFINED_ERROR           0x19 //表示未定义的错误
#define INVALID_REGISTER          0x1A //表示无效寄存器号
#define REGISTER_SET_CONTENT_ERR  0x1B //表示寄存器设定内容错误
#define TEXT_PAGE_ERROR           0x1C //表示记事本页码指定错误
#define PORT_OPERATION_ERROR      0x1D //端口操作失败
#define CONFIRMATION_CODE_LENGTH  24


//系统类指令
#define VFYPWD   							0x13  //验证模块握手口令
#define SETPWD                0x12  //设置模块握手口令
#define SETADDER              0x15  //设置模块地址
#define SETSYSPARA            0x0E  //工作参数设置                 
#define CONTROL               0x17  //UART：对USB通讯端口进行开关控制；USB：对UART端口进行开关控制
#define READSYSPARA           0x0F  //读取模块的状态寄存器和系统基本配置参数
#define TEMPLETENUM           0x1D  //读模块内以存储的指纹模板个数
//指纹处理类指令
#define GENIMG                0x01  //探测手指，探测到后录入指纹图像存于ImageBuffer，并返回录入成功确认码，若探测不到手指，直接 返回无手指确认码
#define UPIMAGE               0x0A  //将模块图像缓冲区Img_Buffer中的数据上传给上位机
#define DOWMIMAGE             0x0B  //上位机下载图像数据到模块图像缓冲区Img_Buffer中
#define IMG2TZ                0x02  //将ImageBuffer中的原始图像生成指纹特征，文件存于CharBuffer或CharBuffer2
#define REGMODEL              0x05  //将CharBuffer1与CharBuffer2中的特征文件合并生成模板，结果存于CharBuffer1与CharBuffer2（两者内容相同）
#define UPCHAR                0x08  //将特征缓冲区CharBuffer1或CharBuffer2中的特征文件上传给上位机
#define DOWNCHAR              0x09  //上位机下载特征文件到模块的一个特征缓冲区中
#define STORE                 0x06  //将指定的特征缓冲区(buffer1 或 Buffer2)中的模板数据存储到Flash指纹库中指定位置
#define LOADCHAR              0x07  //将flash数据库中指定ID号的指纹模板读入到模板缓冲区CharBuffer1 或 CharBuffer2
#define DELETCHAR             0x0C  //删除模块指纹库中指定的一段(指定ID号开始的N个指纹模板)模板
#define EMPTY                 0x0D  //删除模块中指纹库内所有指纹模板
#define Match                 0x03  //模块精确比对（1：1）CharBuffer1 与 CharBuffer2中的特征文件，并给比对结果。
#define SEARCH                0x04  //以CharBuffer1或CharBuffer2中的特征文件搜索整个或部分指纹库。若搜索到，则返回页码。
//其他指令
#define GETRANDOMCODE         0x14  //令模块芯片生成一个随机数并返回给上位机
#define WRITENOTEPAD          0x18  //上位机将数据写入记事本指定Flash页。该指令与READNOTEPAD对应
#define READNOTEPAD           0x19  //读取记事本指定页数据。该指令与Writenotepad对应

#define COMMAND_CODE_LENGTH   23

#define DEBUG 0

/*                           指令包格式                          
---------------------------------------------------------------------|
    包头    模块地址    包标识    包长度   指令码    口令    校验和  |
	 0xEF01    xxxx        01H       0x7H     13H    PassWord    sum   |   
	 2bytes   4bytes      1byte     2bytes   1byte     4byte    2byte  |
---------------------------------------------------------------------|

                             应答包格式
----------------------------------------------------------------------|   
     包头     模块地址      包标识      包长度      确认码     校验和 |
    0xEF01     xxxx          07H          03H         xxH        sum  |
    2bytes    4bytes        1bytes       2bytes      1byte      2bytes|
----------------------------------------------------------------------|
*/

#define DATA_LENGTH 128
/* 命令包定义 */
typedef struct System_Command_Package{
	uint8_t Head[2];						//命令包头 0xEF,0x01
	uint8_t Addr[4];						//地址码   0xFF,0xFF,0xFF,0xFF
	uint8_t PID;								//包标识   0x01(cmd_package),0x02(data_package),0x07(ack_package),0x08(end_package)
	uint8_t Length[2];					//包长度 = DATA_LENGTH + 2
	uint8_t *Data;              //包内容
	uint8_t SUM[2];             //校验和 = PID + Length + Data
}_SYS_CMD_PACK;


uint8_t Commands_Send(uint8_t *command, uint8_t num);
uint8_t Get_FPM_CMD(uint8_t pid, uint8_t *data, uint8_t data_length);
uint8_t Get_FPM_CMD_Data_area(uint8_t para_num, ...);
void SHOW_CMD(uint8_t *ch, uint8_t size);
uint8_t Find_Finger_And_Save_ImageBuffer(void);
uint8_t Get_Finger_Feature_Save_To_CharBuffer1(void);
uint8_t Get_Finger_Feature_Save_To_CharBuffer2(void);
uint8_t Combine_Finger_Feature(void);
uint8_t Save_Finger_Form_To_Flash_Position(uint8_t BufferID, uint16_t PageID);
uint16_t Search_Finger_In_Finlib(uint8_t BufferID, uint16_t Page_start, uint16_t Page_num);
uint8_t Empty_The_FinLib(void);
uint8_t Read_Finger_Form_From_Flash(uint8_t BufferID,uint16_t PageID);
uint16_t ReadEffectiveTemplate(void);
void TIP_SUCCESS(void);
void TIP_FAILED(void);
void TIP_WRITE_SUCCESS(void);
void TIP_WRITE_FAILED(void);
void TIP_DELATE_ALL(void);
void Write_Finger(void);
void Delate_Finger(void);
void Find_Finger(void);
void Init_sth(void);
void FingerDelay(uint32_t nTime);

#endif
