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
#include "user_flash.h"


Robot electron(&hspi1, &hi2c1);
float jointSetPoints[6];
bool isEnabled = false;
uint8_t testuart[]={"testuart"};

int fputc(int ch,FILE *f)
{
    uint8_t temp[1] = {(uint8_t)ch};
    //HAL_UART_Transmit(&huart2, temp, 1, 2);
    HAL_UART_Transmit(&huart1,temp, 1,2);
    return ch;
}


#define Framehead  0xea
#define Frametail  0xea

#define CMD_SetAlljointStatus  0x01
#define CMD_GetAlljointStatus  0x02
#define CMD_SetjointID         0x03

uint8_t rxbuf[256]={0};
uint8_t txbuf[256]={0};


#define FLASH_JointStatusData 0x080f0000


struct ElectronBotJointStatus_t
{
    uint8_t id;
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

uint8_t CheckSum(uint8_t *buf,uint16_t dataAreaLen)
{
    uint16_t i=0,checkLen=0;
    uint8_t *checkBuf;
    uint8_t checkValue=0;
    uint32_t sum=0;

    checkBuf=&buf[1];
    checkLen=dataAreaLen+8;
    //i=dataAreaLen+8;
    for(i=0;i<checkLen;i++)
    {
        sum=sum+checkBuf[i];
    }
    checkValue=sum%256;
    return checkValue;
}

//DataResolution()
//{

//}

struct ProtocolItem_t{
    uint32_t ElectronBotID;
    uint8_t  cmd;
    uint8_t  jointID;
    uint16_t dataLen;
    uint8_t  *data;
    bool    SaveEn;
} ;
ProtocolItem_t ProtocolItem;

/*
void CompositeDataFrame()
{

}*/

void SaveJointStatusToFalsh(uint8_t *buf ,uint16_t len)
{

}

void BufClear(uint8_t *buf,uint8_t value,uint16_t len)
{
    memset(buf,value,len);
}

void ProtocolProcessing(uint8_t *buf)
{
    ElectronBotJointStatus_t rx,local,*p;
    //ElectronBotJointStatus_t *p;

    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    uint8_t idbuf=0;
    ProtocolItem.ElectronBotID=buf[1]*256*256*256*256+buf[2]*256*256*256+buf[3]*256*256+buf[4]*256;
    ProtocolItem.cmd=buf[5];
    ProtocolItem.jointID=buf[6];
    ProtocolItem.dataLen=buf[7]*256+buf[8];
    ProtocolItem.data=&buf[9];

    ProtocolItem.SaveEn=false;

    idbuf=ProtocolItem.jointID/2;
    memcpy(&local,&ElectronBotjoint[idbuf],sizeof(ElectronBotJointStatus_t));

    if(ProtocolItem.cmd == CMD_SetAlljointStatus )
    {
        memcpy(&rx,ProtocolItem.data,sizeof(ElectronBotJointStatus_t));
        if(rx.angleMin!=local.angleMin)
        {
            electron.joint[idbuf].angleMin=rx.angleMin;
            ProtocolItem.SaveEn = true;
        }

        if(rx.angleMin!=local.angleMin)
        {

        }

        if(rx.kp!=local.kp)
        {
            electron.SetJointKp(electron.joint[ProtocolItem.jointID/2],rx.kp);
            local.kp=rx.kp;
            ProtocolItem.SaveEn = true;
        }
    }

    if(ProtocolItem.SaveEn == true)
    {
        memcpy(&ElectronBotjoint[idbuf],&local,sizeof(ElectronBotJointStatus_t));
       // SaveJointStatusToFalsh(&ElectronBotjoint,sizeof(ElectronBotJointStatus_t)*6);
    }
    //BufClear(rx,0,sizeof(rxbuf));
}


void ProtocolLookUp(uint8_t *buf,uint16_t len)
{
    uint16_t i;
    uint16_t datalen=0;
    uint8_t checkValue=0;
    if(len<11)
    {
        return ;
    }
    for(i=0;i<len;i++)
    {
        if(buf[i]==Framehead && len-i>=11)
        {
            datalen=buf[i+7]*256+buf[i+8];
            if(buf[i+7+2+datalen+2]==Frametail)
            {
                checkValue=buf[i+7+2+datalen+1];
                if(checkValue==CheckSum(&buf[i],datalen))
                {
                    memcpy(rxbuf,&buf[i],7+2+datalen+2);
                    ProtocolProcessing(rxbuf);
                }
            }
        }
    }
}


void Main(void)
{
    MX_USART1_UART_Init();
    HAL_Delay(2000);
    electron.lcd->Init(Screen::DEGREE_0);
    electron.lcd->SetWindow(0, 239, 0, 239);
    //HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    //HAL_UART_Transmit(&huart1,&testuart[0], sizeof(testuart),50);
    HAL_UART_Transmit(&huart1,testuart, sizeof(testuart),50);
    HAL_Delay(200);
    printf("printf:%s",testuart);

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

    // 3.这时候就能看到舵机做往复运动了。
    while (1)
    {
        for (int i = -15; i < 15; i += 1)
        {
            float angle = i;
            electron.UpdateJointAngle(electron.joint[0], angle);
            HAL_Delay(20);
        }
        for (int i = 15; i > -15; i -= 1)
        {
            float angle = i;
            electron.UpdateJointAngle(electron.joint[0], angle);
            HAL_Delay(20);
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