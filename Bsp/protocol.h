//
// Created by Administrator on 2022/8/5.
//

//#ifndef ELECTRONBOT_FW_PROTOCOL_H
//#define ELECTRONBOT_FW_PROTOCOL_H

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "flash.h"
#include <cmath>
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

#define FrameHead  0xea
#define FrameTail  0xea

//#define CMD_SetAllJointStatus  0x01
//#define CMD_GetAllJointStatus  0x02
//#define CMD_SetJointID         0x03
//#define CMD_SetElectronBotID   0x04
//#define CMD_GetElectronBotID   0x05


#define CMD_WriteAllJointStatus  0x01
#define CMD_ReadAllJointStatus   0x02
#define CMD_WriteJointID         0x03
#define CMD_WriteElectronBotID   0x04
#define CMD_ReadElectronBotID    0x05


#define uint8_t  unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned int
#define uint64_t unsigned long


struct ElectronBotJointStatus_t
{
    // uint8_t id;
    float angleMin;
    float angleMax;
    float angle;
    float modelAngelMin;
    float modelAngelMax;
    bool  inverted = false;
    float initAngle;
    float torqueLimit;
    float kp;
    float ki;
    float kv;
    float kd;
    bool enable;  //舵机使能
};
//ElectronBotJointStatus_t ElectronBotjoint[6];

struct JointStatus_t
{
    uint8_t id;
    float angleMin;
    float angleMax;
    float angle;
    float modelAngelMin;
    float modelAngelMax;
    bool inverted = false;
};
//JointStatus_t joint[7];

enum CmdOrResFrame { CommandFrame, ResponseFrame };
struct ProtocolItem_t {
    uint32_t ElectronBotID;
    uint8_t  cmd;
    uint8_t  jointID;
    uint16_t dataLen;
    uint8_t* data;

    CmdOrResFrame frame; //设置帧类型  命令帧或者应答帧  若为应答帧 cmd的最高bit位被值1;
    bool    SaveEn;
};
//ProtocolItem_t ProtocolItem;


struct txbuf_t {
    uint16_t dataLen;
    uint8_t buf[300];
};
//txbuf_t txbuf;

struct rxbuf_t {
    uint16_t dataLen;
    uint8_t* frameHead;
    uint8_t buf[300];
};
//rxbuf_t rxbuf;

void testComposeProtocolFrame();
void ProtocolProcessing(uint8_t* buf, uint16_t len);//处理数据
bool ProtocolLookUp(uint8_t* frameBuf, uint8_t* inBuf, uint16_t inLen);//查找inBuf里是否有协议帧,有则将协议帧拷贝到frameBuf
void ComposeProtocolFrame(uint8_t* frameBuf, uint16_t* frameLen, ProtocolItem_t* items);//组帧协议帧拷贝到frameBuf
void testReceiveMasterUsbData(uint8_t *buf, uint16_t len);
void test_protocol();

#endif //PROTOCOL_H
