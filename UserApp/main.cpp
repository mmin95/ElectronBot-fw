/*#include <cmath>
#include "common_inc.h"
#include "screen.h"
#include "robot.h"


Robot electron(&hspi1, &hi2c1);
float jointSetPoints[6];
bool isEnabled = false;


void Main(void)
{
    HAL_Delay(2000);
    electron.lcd->Init(Screen::DEGREE_0);
    electron.lcd->SetWindow(0, 239, 0, 239);


    electron.UpdateJointAngle(electron.joint[1], 0);
    electron.UpdateJointAngle(electron.joint[2], 0);
    electron.UpdateJointAngle(electron.joint[3], 0);
    electron.UpdateJointAngle(electron.joint[4], 0);
    electron.UpdateJointAngle(electron.joint[5], 0);
    electron.UpdateJointAngle(electron.joint[6], 0);

    electron.SetJointTorqueLimit(electron.joint[2],0.5);
    electron.SetJointKp(electron.joint[2],40);

    electron.SetJointTorqueLimit(electron.joint[3],0.5);
    electron.SetJointKp(electron.joint[3],40);

    electron.SetJointTorqueLimit(electron.joint[4],0.5);
    electron.SetJointKp(electron.joint[4],40);


    electron.SetJointTorqueLimit(electron.joint[5],0.5);
    electron.SetJointKp(electron.joint[5],40);


    float t = 0;

    while (true)
    {
#if 1
        for (int p = 0; p < 4; p++)
        {
            // send joint angles
            for (int j = 0; j < 6; j++)
                for (int i = 0; i < 4; i++)
                {
                    auto* b = (unsigned char*) &(electron.joint[j + 1].angle);
                    electron.usbBuffer.extraDataTx[j * 4 + i + 1] = *(b + i);
                }
            electron.SendUsbPacket(electron.usbBuffer.extraDataTx, 32);

            electron.ReceiveUsbPacketUntilSizeIs(224); // last packet is 224bytes

            // get joint angles
            uint8_t* ptr = electron.GetExtraDataRxPtr();
            if (isEnabled != (bool) ptr[0])
            {
                isEnabled = ptr[0];
                electron.SetJointEnable(electron.joint[1], isEnabled);
                electron.SetJointEnable(electron.joint[2], isEnabled);
                electron.SetJointEnable(electron.joint[3], isEnabled);
                electron.SetJointEnable(electron.joint[4], isEnabled);
                electron.SetJointEnable(electron.joint[5], isEnabled);
                electron.SetJointEnable(electron.joint[6], isEnabled);
            }
            for (int j = 0; j < 6; j++)
            {
                jointSetPoints[j] = *((float*) (ptr + 4 * j + 1));
            }

            while (electron.lcd->isBusy);
            if (p == 0)
                electron.lcd->WriteFrameBuffer(electron.GetLcdBufferPtr(),
                                               60 * 240 * 3);
            else
                electron.lcd->WriteFrameBuffer(electron.GetLcdBufferPtr(),
                                               60 * 240 * 3, true);
        }
        HAL_Delay(1);
#endif


        t += 0.01;

        electron.UpdateJointAngle(electron.joint[1], jointSetPoints[0]);
        electron.UpdateJointAngle(electron.joint[2], jointSetPoints[1]);
        electron.UpdateJointAngle(electron.joint[3], jointSetPoints[2]);
        electron.UpdateJointAngle(electron.joint[4], jointSetPoints[3]);
        electron.UpdateJointAngle(electron.joint[5], jointSetPoints[4]);
        electron.UpdateJointAngle(electron.joint[6], jointSetPoints[5]);

        HAL_Delay(1);

//      electron.UpdateJointAngle(electron.joint[ANY], 65 + 75 * std::sin(t));

        myprintf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               jointSetPoints[0], jointSetPoints[1], jointSetPoints[2],
               jointSetPoints[3], jointSetPoints[4], jointSetPoints[5]);
    }
}


extern "C"
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    electron.lcd->isBusy = false;
}
*/


#include <cmath>
#include "common_inc.h"
#include "screen.h"
#include "robot.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "user_flash.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "protocol.h"
#include "flash.h"

#include "paj7620u2.h"





Robot electron(&hspi1, &hi2c1);
float jointSetPoints[6];
bool isEnabled = false;
uint8_t testuart[]={"testuart"};

/*
int fputc(int ch,FILE *f)
{
    uint8_t temp[1] = {(uint8_t)ch};
    //HAL_UART_Transmit(&huart2, temp, 1, 2);
    HAL_UART_Transmit(&huart1,temp, 1,10);
    return ch;
}*/

/*
#define FrameHead  0xea
#define FrameTail  0xea

#define CMD_SetAllJointStatus  0x01
#define CMD_GetAllJointStatus  0x02
#define CMD_SetJointID         0x03
#define CMD_SetElectronBotID   0x04
#define CMD_GetElectronBotID   0x05

//uint8_t rxbuf[256]={0};
//uint8_t txbuf[256]={0};


#define FLASH_JointStatusData ADDR_FLASH_SECTOR_11

// uint8_t id;
struct
ElectronBotJointStatus_t
{
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
ElectronBotJointStatus_t ElectronBotjoint[6];


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
JointStatus_t joint[7];

uint8_t CheckSum(uint8_t* buf, uint16_t dataAreaLen)
{
    uint16_t i = 0, checkLen = 0;
    uint8_t* checkBuf;
    uint8_t checkValue = 0;
    uint32_t sum = 0;

    checkBuf = &buf[1];
    checkLen = dataAreaLen + 8;
    //i=dataAreaLen+8;
    for (i = 0; i < checkLen; i++)
    {
        sum = sum + checkBuf[i];
    }
    checkValue = sum % 256;
    return checkValue;
}




//DataResolution()
//{

//}
enum CmdOrResFrame{CommandFrame,ResponseFrame};
struct ProtocolItem_t{
    uint32_t ElectronBotID;
    uint8_t  cmd;
    uint8_t  jointID;
    uint16_t dataLen;
    uint8_t  *data;

    CmdOrResFrame frame;
    bool    SaveEn;
} ;
ProtocolItem_t ProtocolItem;

void SaveJointStatusToFlash(uint8_t *buf ,uint16_t len)
{
    //STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)
    STMFLASH_Write(FLASH_JointStatusData, (uint32_t *)buf,len);
}

void ReadJointStatusFromFlash(uint8_t *buf ,uint16_t len)
{
    //STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)
    //STMFLASH_Write(FLASH_JointStatusData, (uint32_t *)buf,len);
    //STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead);
    STMFLASH_Read(FLASH_JointStatusData,(uint32_t *)buf,len);
}


void BufClear(uint8_t *buf,uint8_t value,uint16_t len)
{
    memset(buf,value,len);
}


void myprintf(const char *format,...)
{
    //char str[80];
    //sprintf(str,format);
    //HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),10);
    printf(format);
}


struct txbuf_t{
    uint16_t dataLen;
    uint8_t buf[300];
};
txbuf_t txbuf;

struct rxbuf_t{
    uint16_t dataLen;
    uint16_t frameHead;
    uint8_t buf[300];
};
rxbuf_t rxbuf;


//void AssemblyProtocolFrame(ProtocolItem_t * items,bool cmd)
void AssemblyProtocolFrame(ProtocolItem_t * items)
{
    uint16_t len=0,itemLen=0;
    uint8_t checkValue=0;

    txbuf.buf[0]=FrameHead;
    len++;

    itemLen=sizeof(items->ElectronBotID);
    memcpy(&txbuf.buf[len],&items->ElectronBotID,itemLen);
    len+=itemLen;

    if(items->frame == ResponseFrame)
    {
        items->cmd=items->cmd|0x80;
    }
    itemLen = sizeof(items->cmd);
    memcpy(&txbuf.buf[len],&items->cmd,itemLen);
    len+=itemLen;

    itemLen=sizeof(items->jointID);
    memcpy(&txbuf.buf[len],&items->jointID,itemLen);
    len+=itemLen;

    itemLen=sizeof(items->dataLen);
    memcpy(&txbuf.buf[len],&items->dataLen,itemLen);
    len+=itemLen;

    //itemLen=sizeof(items->dataLen);
    if(items->dataLen>0)
    {
        memcpy(&txbuf.buf[len], items->data, items->dataLen);
        len += items->dataLen;
    }

    checkValue=CheckSum((uint8_t *)&txbuf.buf[0],items->dataLen);
    txbuf.buf[len]=checkValue;
    len++;

    txbuf.buf[len]=FrameTail;
    len++;
    txbuf.dataLen=len;
   // memcpy(txbuf,&items->ElectronBotID,sizeof(items->ElectronBotID));
}



void ComposeProtocolFrame(uint8_t* buf ,uint16_t *dataLen, ProtocolItem_t* items)
{
    uint16_t len = 0, itemLen = 0;
    uint8_t checkValue = 0;

    buf[0] = FrameHead;
    len++;

    itemLen = sizeof(items->ElectronBotID);
    memcpy(&buf[len], &items->ElectronBotID, itemLen);
    len += itemLen;

    if (items->frame == ResponseFrame)
    {
        items->cmd = items->cmd | 0x80;
    }
    itemLen = sizeof(items->cmd);
    memcpy(&buf[len], &items->cmd, itemLen);
    len += itemLen;

    itemLen = sizeof(items->jointID);
    memcpy(&buf[len], &items->jointID, itemLen);
    len += itemLen;

    itemLen = sizeof(items->dataLen);
    memcpy(&buf[len], &items->dataLen, itemLen);
    len += itemLen;

    //itemLen=sizeof(items->dataLen);
    if (items->dataLen > 0)
    {
        memcpy(&buf[len], items->data, items->dataLen);
        len += items->dataLen;
    }

    checkValue = CheckSum((uint8_t*)&txbuf.buf[0], items->dataLen);
    buf[len] = checkValue;
    len++;

    buf[len] = FrameTail;
    len++;

    *dataLen = len;
    // memcpy(txbuf,&items->ElectronBotID,sizeof(items->ElectronBotID));
}

bool ProtocolLookUp(uint8_t* outBuf, uint16_t *outFrameHead, uint8_t* inBuf, uint16_t inLen)
{
    uint16_t i;
    uint16_t datalen=0;
    uint8_t checkValue=0;
    if(inLen<11)
    {
        return false;
    }
    for(i=0;i<inLen;i++)
    {
        if(inBuf[i]==FrameHead && inLen-i>=11)
        {
            datalen=inBuf[i+7]+inBuf[i+8]*256;
            if(inBuf[i+7+2+datalen+2-1]==FrameTail)
            {
                checkValue=inBuf[i+7+2+datalen+1-1];
                if(checkValue==CheckSum(&inBuf[i],datalen))
                {
                    memcpy(outBuf,&inBuf[i],7+2+datalen+2);
                    *outFrameHead=i;
                    return true;
                   // ProtocolProcessing(rxbuf);
                }
            }
        }
    }
    return false;
}


void ProtocolProcessing(uint8_t *inbuf, uint16_t len)
{

    //ElectronBotJointStatus_t *p;

    if(ProtocolLookUp(rxbuf.buf,&rxbuf.frameHead,inbuf,len)== false)
    {
        return;
    }

    ElectronBotJointStatus_t rx,local,*p;
    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    uint8_t idbuf=0;
    uint8_t *buf;
    buf=&rxbuf.buf[rxbuf.frameHead];

    ProtocolItem.ElectronBotID=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
    ProtocolItem.cmd=buf[5];
    ProtocolItem.jointID=buf[6];
    ProtocolItem.dataLen=buf[7]+buf[8]*256;
    ProtocolItem.data=&buf[9];
    ProtocolItem.SaveEn=false;

    idbuf=ProtocolItem.jointID/2;
    memcpy(&local,&ElectronBotjoint[idbuf],sizeof(ElectronBotJointStatus_t));

    if(ProtocolItem.cmd == CMD_SetAllJointStatus )
    {
        memcpy(&rx,ProtocolItem.data,sizeof(ElectronBotJointStatus_t));
        if(rx.angleMin!=local.angleMin)
        {
            electron.joint[idbuf].angleMin=rx.angleMin;
            local.angleMin=rx.angleMin;
            ProtocolItem.SaveEn = true;
        }

        if(rx.angleMax!=local.angleMax)
        {
            electron.joint[idbuf].angleMin=rx.angleMax;
            local.angleMax=rx.angleMax;
            ProtocolItem.SaveEn = true;
        }

        if(rx.angle!=local.angle)
        {
            electron.joint[idbuf].angle=rx.angle;
           // ProtocolItem.SaveEn = true;
            electron.UpdateJointAngle(electron.joint[idbuf], rx.angle);
            HAL_Delay(20);
        }

        if(rx.modelAngelMin!=local.modelAngelMin)
        {
            electron.joint[idbuf].modelAngelMin=rx.modelAngelMin;
            local.modelAngelMin=rx.modelAngelMin;
            ProtocolItem.SaveEn = true;
        }

        if(rx.modelAngelMax!=local.modelAngelMax)
        {
            electron.joint[idbuf].modelAngelMax=rx.modelAngelMax;
            local.modelAngelMax=rx.modelAngelMax;
            ProtocolItem.SaveEn = true;
        }

        if(rx.inverted!=local.inverted)
        {
            electron.joint[idbuf].inverted=rx.inverted;
            local.inverted=rx.inverted;
            ProtocolItem.SaveEn = true;
        }

        if(rx.initAngle!=local.initAngle)
        {
           // electron.SetJointInitAngle(electron.joint[idbuf], rx.initAngle);
            HAL_Delay(20);
            local.initAngle=rx.initAngle;
            ProtocolItem.SaveEn = true;
        }

        if(rx.torqueLimit!=local.torqueLimit)
        {
            electron.SetJointTorqueLimit(electron.joint[idbuf], rx.torqueLimit);
            HAL_Delay(20);
            local.torqueLimit=rx.torqueLimit;
            ProtocolItem.SaveEn = true;
        }

        if(rx.kp!=local.kp)
        {
            electron.SetJointKp(electron.joint[idbuf],rx.kp);
            HAL_Delay(20);
            local.kp=rx.kp;
            ProtocolItem.SaveEn = true;
        }

        if(rx.ki!=local.ki)
        {
            electron.SetJointKi(electron.joint[idbuf],rx.ki);
            HAL_Delay(20);
            local.ki=rx.kp;
            ProtocolItem.SaveEn = true;
        }

        if(rx.kv!=local.kv)
        {
            electron.SetJointKv(electron.joint[idbuf],rx.kv);
            HAL_Delay(20);
            local.kv=rx.kv;
            ProtocolItem.SaveEn = true;
        }

        if(rx.kd!=local.kd)
        {
            electron.SetJointKd(electron.joint[idbuf],rx.kd);
            HAL_Delay(20);
            local.kd=rx.kd;
            ProtocolItem.SaveEn = true;
        }

        if(rx.enable!=local.enable)
        {
            electron.SetJointEnable(electron.joint[idbuf],rx.enable);
            HAL_Delay(20);
            local.enable=rx.enable;
            ProtocolItem.SaveEn = true;
        }

        if(ProtocolItem.SaveEn == true)
        {
            memcpy(&ElectronBotjoint[idbuf],&local,sizeof(ElectronBotJointStatus_t));
            HAL_Delay(20);
            //p=&ElectronBotjoint[0];
            p=ElectronBotjoint;
            SaveJointStatusToFlash((uint8_t *)p,sizeof(ElectronBotJointStatus_t)*6);
        }
    }

    BufClear((uint8_t* )&rxbuf,0,sizeof(rxbuf));
}

void testReceiveMasterUsbData(uint8_t *inbuf, uint16_t len)
{
    if(ProtocolLookUp(rxbuf.buf,&rxbuf.frameHead,inbuf,len)== false)
    {
        return;
    }

    ElectronBotJointStatus_t rx,local,*p;
    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    uint8_t idbuf=0;
    uint8_t *buf;
    //buf=&rxbuf.buf[rxbuf.frameHead];
    buf=inbuf;

    ProtocolItem.ElectronBotID=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
    ProtocolItem.cmd=buf[5];
    ProtocolItem.jointID=buf[6];
    ProtocolItem.dataLen=buf[7]+buf[8]*256;
    ProtocolItem.data=&buf[9];
    ProtocolItem.SaveEn=false;

    idbuf=ProtocolItem.jointID/2;
    memcpy(&local,ProtocolItem.data,sizeof(ElectronBotJointStatus_t));

    myprintf("ElectronBotID=%d\r\n",ProtocolItem.ElectronBotID);
    HAL_Delay(200);
    myprintf("cmd=%d\r\n",ProtocolItem.cmd);
    HAL_Delay(200);
    myprintf("jointID=%d\r\n",ProtocolItem.jointID);
    HAL_Delay(200);
    myprintf("angleMin=%f\r\n",local.angleMin);
    //char str[80];
    // smyprintf(str,"angleMin=%f\r\n",local.angleMin);
    // HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),10);

    HAL_Delay(200);
    myprintf("angleMax=%f\r\n",local.angleMax);
    HAL_Delay(200);
    myprintf("angle=%f\r\n",local.angle);
    HAL_Delay(200);
    myprintf("modelAngelMin=%f\r\n",local.modelAngelMin);
    HAL_Delay(200);
    myprintf("modelAngelMax=%f\r\n",local.modelAngelMax);
    HAL_Delay(200);
    //myprintf("inverted=%d\r\n",local.inverted);
    myprintf("inverted=%s\r\n",local.inverted?"true" :"false");
    HAL_Delay(200);
    myprintf("initAngle=%f\r\n",local.initAngle);
    HAL_Delay(200);
    myprintf("torqueLimit=%f\r\n",local.torqueLimit);
    HAL_Delay(200);
    myprintf("kp=%f\r\n",local.kp);
    HAL_Delay(200);
    myprintf("ki=%f\r\n",local.ki);
    HAL_Delay(200);
    myprintf("kv=%f\r\n",local.kv);
    HAL_Delay(200);
    myprintf("kd=%f\r\n",local.kd);
    HAL_Delay(200);
    myprintf("enable=%s\r\n",local.enable?"true" :"false");
    HAL_Delay(200);
}
int testDataSaveToFlash()
{
    ElectronBotJointStatus_t rx,local,*p;
    uint8_t buf[200]={0};
    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    rx.angleMin=10;
    rx.angleMax=30;
    rx.angle=0;
    rx.modelAngelMin=12;
    rx.modelAngelMax=32;
    rx.inverted=true;
    rx.initAngle=15;
    rx.torqueLimit=0.5;
    rx.kp=20;
    rx.ki=1;
    rx.kv=2;
    rx.kd=3;
    rx.enable=true;

    p=&rx;
    SaveJointStatusToFlash((uint8_t *)p,sizeof(ElectronBotJointStatus_t));
    HAL_Delay(500);
    p=&local;
    ReadJointStatusFromFlash((uint8_t *)buf,sizeof(ElectronBotJointStatus_t));
    memcpy(&local,buf,sizeof(ElectronBotJointStatus_t));
    //HAL_Delay(500);

    // myprintf("angleMin=%f\r\n",local.angleMin);
    myprintf("angleMin=%f\r\n",local.angleMin);
    //char str[80];
    // smyprintf(str,"angleMin=%f\r\n",local.angleMin);
    // HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),10);

    HAL_Delay(200);
    myprintf("angleMax=%f\r\n",local.angleMax);
    HAL_Delay(200);
    myprintf("angle=%f\r\n",local.angle);
    HAL_Delay(200);
    myprintf("modelAngelMin=%f\r\n",local.modelAngelMin);
    HAL_Delay(200);
    myprintf("modelAngelMax=%f\r\n",local.modelAngelMax);
    HAL_Delay(200);
    //myprintf("inverted=%d\r\n",local.inverted);
    myprintf("inverted=%s\r\n",local.inverted?"true" :"false");
    HAL_Delay(200);
    myprintf("initAngle=%f\r\n",local.initAngle);
    HAL_Delay(200);
    myprintf("torqueLimit=%f\r\n",local.torqueLimit);
    HAL_Delay(200);
    myprintf("kp=%f\r\n",local.kp);
    HAL_Delay(200);
    myprintf("ki=%f\r\n",local.ki);
    HAL_Delay(200);
    myprintf("kv=%f\r\n",local.kv);
    HAL_Delay(200);
    myprintf("kd=%f\r\n",local.kd);
    HAL_Delay(200);
    myprintf("enable=%s\r\n",local.enable?"true" :"false");
    HAL_Delay(200);
    return 0;
}

uint8_t buf[200]={0};
void testAssemblyProtocolFrame()
{
    ElectronBotJointStatus_t rx,local,*p;

    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    rx.angleMin=10;
    rx.angleMax=30;
    rx.angle=0;
    rx.modelAngelMin=12;
    rx.modelAngelMax=32;
    rx.inverted=true;
    rx.initAngle=15;
    rx.torqueLimit=0.5;
    rx.kp=20;
    rx.ki=1;
    rx.kv=2;
    rx.kd=3;
    rx.enable=true;

    p=&rx;
    memcpy((uint8_t *)buf,(uint8_t *)p,sizeof(ElectronBotJointStatus_t));

    //p=&ProtocolItem;
    ProtocolItem_t *p2;
    p2=&ProtocolItem;
    BufClear((uint8_t *)p2,0,sizeof(ProtocolItem_t));

    ProtocolItem.ElectronBotID=0x00000000;
    ProtocolItem.ElectronBotID=0x12345678;
    ProtocolItem.cmd=CMD_SetAllJointStatus;
    ProtocolItem.jointID=12;
    ProtocolItem.dataLen=sizeof(ElectronBotJointStatus_t);
    ProtocolItem.data=buf;

    ProtocolItem.frame=CommandFrame;
    ProtocolItem.SaveEn=false;


    AssemblyProtocolFrame(&ProtocolItem);
   // ProtocolLookUp(txbuf.buf,txbuf.dataLen);
     uint8_t usbtestBUF[300]={0};
     for(int i=0x65;i<(0x65+300);i++)
     {
         usbtestBUF[i]=i;
     }
    electron.SendUsbPacket(txbuf.buf,txbuf.dataLen+2);
   // memset(usbtestBUF,0x65,sizeof(usbtestBUF)/2);
   // electron.SendUsbPacket(usbtestBUF,sizeof(usbtestBUF));
   // electron.SendUsbPacket(usbtestBUF,100);
    ProtocolProcessing(txbuf.buf,txbuf.dataLen);
}*/

uint8_t usbTestBuf[300]={0};
extern usbRxBuf_t usbRx;
void Main(void)
{


   HAL_Delay(2000);
   electron.lcd->Init(Screen::DEGREE_0);
   electron.lcd->SetWindow(0, 239, 0, 239);
  // testAssemblyProtocolFrame();
  // testDataSaveToFlash();
  //  test_protocol();
  //  test_flash();
    //paj7620u2_init();
  //  test_paj7260u2();
    MX_USART1_UART_Init();
    HAL_Delay(2000);
    electron.lcd->Init(Screen::DEGREE_0);
    electron.lcd->SetWindow(0, 239, 0, 239);
    //HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    //HAL_UART_Transmit(&huart1,&testuart[0], sizeof(testuart),50);
    HAL_UART_Transmit(&huart1,testuart, sizeof(testuart),50);
    HAL_Delay(200);
    printf("printf:%s",testuart);
   // testAssemblyProtocolFrame();
    Gesture_test();

#if 1
    // 0.先只连接一个舵机,不设置地址，测试硬件和舵机固件是否OK。

    // 1.确保广播Joint的变量正确，直接更新 UpdateJointAngle
    //   可能会因为角度不在变量范围内不发送指令。（请详细读代码）
    electron.joint[0].id = 2;
    electron.joint[0].angleMax = 180;
    electron.joint[0].angle = 0;
    electron.joint[0].modelAngelMin = -90;
    electron.joint[0].modelAngelMax = 90;

    electron.joint[0].angleMin = 0;
    // 2.使用广播地址是能
    electron.SetJointEnable(electron.joint[0], true);

    for (int j = 0; j < 300; ++j) {
        usbTestBuf[j]=j;
    }

    // 3.这时候就能看到舵机做往复运动了。
    while (1)
    {
        for (int i = -15; i < 15; i += 1)
        {
            float angle = i;
            electron.UpdateJointAngle(electron.joint[0], angle);
            HAL_Delay(20);
          //  electron.SendUsbPacket(testuart,sizeof(testuart));
           /* memset(usbTestBuf,0x00,sizeof(usbTestBuf));
            usbTestBuf[0]=0x01;
            usbTestBuf[1]=0x01;
            usbTestBuf[2]=0x01;
            usbTestBuf[3]=0x00;
            usbTestBuf[4]=0x01;
            usbTestBuf[5]=0x01;
            usbTestBuf[6]=0x01;
            usbTestBuf[7]=0x01;
            usbTestBuf[8]=0x01;
            usbTestBuf[9]=0x01;*/
          //  electron.SendUsbPacket(usbTestBuf,sizeof(usbTestBuf));
            //testAssemblyProtocolFrame();
            //testAssemblyProtocolFrame();

           // if(usbRx.len)
           // {
           //     testReceiveMasterUsbData(usbRx.buf,usbRx.len);
           //     memset(usbRx.buf,0,100);
           //     usbRx.len=0;
           // }
        }
        for (int i = 15; i > -15; i -= 1)
        {
            float angle = i;
            electron.UpdateJointAngle(electron.joint[0], angle);
            HAL_Delay(20);
            //electron.SendUsbPacket(testuart,sizeof(testuart));
           // memset(usbTestBuf,0x66,sizeof(usbTestBuf));
          //  electron.SendUsbPacket(usbTestBuf,sizeof(usbTestBuf));
            //testAssemblyProtocolFrame();
            //testAssemblyProtocolFrame();
            //if(usbRx.len)
           // {
           //     testReceiveMasterUsbData(usbRx.buf,usbRx.len);
           //     memset(usbRx.buf,0,100);
            //    usbRx.len=0;
            //}
        }
    }
#endif

#if  0
    // 0.当能够正常用广播地址控制单个舵机后，现在进行舵机的ID设置、
    //   还是只连接一个舵机

    // 1. 比如需要把这个舵机ID设置为2

    // 2.确保广播Joint的变量正确，直接更新 UpdateJointAngle
    //   可能会因为角度不在变量范围内不发送指令。（请详细读代码）
    electron.joint[0].id = 0;  //这里还是用广播
    electron.joint[0].angleMin = 0;
    electron.joint[0].angleMax = 180;
    electron.joint[0].angle = 0;
    electron.joint[0].modelAngelMin = -90;
    electron.joint[0].modelAngelMax = 90;
    electron.SetJointId(electron.joint[0], 6);

    HAL_Delay(1000);
    // 3.等待舵机参数保存。将主控 joint[0].id改为 2
    electron.joint[0].id = 6;  //这里改为新地址
    electron.joint[0].angleMin = 0;
    electron.joint[0].angleMax = 180;
    electron.joint[0].angle = 0;
    electron.joint[0].modelAngelMin = -90;
    electron.joint[0].modelAngelMax = 90;

     while (1);
    // 4.使用新设置的2地址通讯
    electron.SetJointEnable(electron.joint[0], true);
    while (1)
    {
        for (int i = -90; i < 90; i += 1)
        {
            float angle = i;
            electron.UpdateJointAngle(electron.joint[0], angle);
            HAL_Delay(20);
        }
        for (int i = 90; i > -90; i -= 1)
        {
            float angle = i;
            electron.UpdateJointAngle(electron.joint[0], angle);
            HAL_Delay(20);
        }
    }

    // 5.之后的程序就不需要 步骤2中的 设置地址了。
    // 只需要用2地址，就能通讯上这个舵机，其他的舵机也是一样。

#endif

    float t = 0;

    while (true)
    {
#if 1
        for (int p = 0; p < 4; p++)
        {
            // send joint angles
            for (int j = 0; j < 6; j++)
                for (int i = 0; i < 4; i++)
                {
                    auto* b = (unsigned char*) &(electron.joint[j + 1].angle);
                    electron.usbBuffer.extraDataTx[j * 4 + i + 1] = *(b + i);
                }
            electron.SendUsbPacket(electron.usbBuffer.extraDataTx, 32);

            electron.ReceiveUsbPacketUntilSizeIs(224); // last packet is 224bytes

            // get joint angles
            uint8_t* ptr = electron.GetExtraDataRxPtr();
            if (isEnabled != (bool) ptr[0])
            {
                isEnabled = ptr[0];
                electron.SetJointEnable(electron.joint[1], isEnabled);
                electron.SetJointEnable(electron.joint[2], isEnabled);
                electron.SetJointEnable(electron.joint[3], isEnabled);
                electron.SetJointEnable(electron.joint[4], isEnabled);
                electron.SetJointEnable(electron.joint[5], isEnabled);
                electron.SetJointEnable(electron.joint[6], isEnabled);
            }
            for (int j = 0; j < 6; j++)
            {
                jointSetPoints[j] = *((float*) (ptr + 4 * j + 1));
            }

            while (electron.lcd->isBusy);
            if (p == 0)
                electron.lcd->WriteFrameBuffer(electron.GetLcdBufferPtr(),
                                               60 * 240 * 3);
            else
                electron.lcd->WriteFrameBuffer(electron.GetLcdBufferPtr(),
                                               60 * 240 * 3, true);
        }
        HAL_Delay(1);
#endif


        t += 0.01;

        electron.UpdateJointAngle(electron.joint[1], jointSetPoints[0]);
        electron.UpdateJointAngle(electron.joint[2], jointSetPoints[1]);
        electron.UpdateJointAngle(electron.joint[3], jointSetPoints[2]);
        electron.UpdateJointAngle(electron.joint[4], jointSetPoints[3]);
        electron.UpdateJointAngle(electron.joint[5], jointSetPoints[4]);
        electron.UpdateJointAngle(electron.joint[6], jointSetPoints[5]);

        HAL_Delay(1);

//      electron.UpdateJointAngle(electron.joint[ANY], 65 + 75 * std::sin(t));

        printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               jointSetPoints[0], jointSetPoints[1], jointSetPoints[2],
               jointSetPoints[3], jointSetPoints[4], jointSetPoints[5]);
    }
}


extern "C"
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    electron.lcd->isBusy = false;
}