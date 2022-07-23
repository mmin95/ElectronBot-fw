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


#define BOARD_NUM_ADDR 0x0800C000

#define STM32_FLASH_BASE 0x08000000 //STM32 FLASH的起始地址
#define FLASH_WAITETIME 50000       //FLASH等待超时时间

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  //扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  //扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  //扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  //扇区3起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  //扇区4起始地址, 64 Kbytes
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  //扇区5起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  //扇区6起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  //扇区7起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  //扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  //扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) //扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) //扇区11起始地址,128 Kbytes

uint32_t STMFLASH_ReadWord(uint32_t faddr);                                      //读出字
void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite); //从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead);    //从指定地址开始读出指定长度的数据


/**------------------------------------------
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  --------------------------------------------*/
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
    if (addr < ADDR_FLASH_SECTOR_1)
        return FLASH_SECTOR_0;
    else if (addr < ADDR_FLASH_SECTOR_2)
        return FLASH_SECTOR_1;
    else if (addr < ADDR_FLASH_SECTOR_3)
        return FLASH_SECTOR_2;
    else if (addr < ADDR_FLASH_SECTOR_4)
        return FLASH_SECTOR_3;
    else if (addr < ADDR_FLASH_SECTOR_5)
        return FLASH_SECTOR_4;
    else if (addr < ADDR_FLASH_SECTOR_6)
        return FLASH_SECTOR_5;
    else if (addr < ADDR_FLASH_SECTOR_7)
        return FLASH_SECTOR_6;
    else if (addr < ADDR_FLASH_SECTOR_8)
        return FLASH_SECTOR_7;
    else if (addr < ADDR_FLASH_SECTOR_9)
        return FLASH_SECTOR_8;
    else if (addr < ADDR_FLASH_SECTOR_10)
        return FLASH_SECTOR_9;
    else if (addr < ADDR_FLASH_SECTOR_11)
        return FLASH_SECTOR_10;
    return FLASH_SECTOR_11;
}

uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(__IO uint32_t *)faddr;
}

void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t Num)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    uint32_t SectorError = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;
    if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
        return; //非法地址

    HAL_FLASH_Unlock();            //解锁
    addrx = WriteAddr;             //写入的起始地址
    endaddr = WriteAddr + Num * 4; //写入的结束地址

    if (addrx < 0X080C1000)
    {
        while (addrx < endaddr)
        {
            if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)
            {
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     //擦除类型，扇区擦除
                FlashEraseInit.Sector = STMFLASH_GetFlashSector(addrx); //要擦除的扇区
                FlashEraseInit.NbSectors = 1;                           //一次只擦除一个扇区
                FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;    //电压范围，VCC=2.7~3.6V之间!!
                if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
                {
                    break; //发生错误了
                }
            }
            else
                addrx += 4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
        }
    }
    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
    if (FlashStatus == HAL_OK)
    {
        while (WriteAddr < endaddr) //写数据
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer) != HAL_OK) //写入数据
            {
                break; //写入异常
            }
            WriteAddr += 4;
            pBuffer++;
        }
    }
    HAL_FLASH_Lock(); //上锁
}

void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t size)
{
    uint32_t i;
    for (i = 0; i < size; i++)
    {
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //读取4个字节.
        ReadAddr += 4;                            //偏移4个字节.
    }
}



Robot electron(&hspi1, &hi2c1);
float jointSetPoints[6];
bool isEnabled = false;
uint8_t testuart[]={"testuart"};

int fputc(int ch,FILE *f)
{
    uint8_t temp[1] = {(uint8_t)ch};
    //HAL_UART_Transmit(&huart2, temp, 1, 2);
    HAL_UART_Transmit(&huart1,temp, 1,10);
    return ch;
}


#define Framehead  0xea
#define Frametail  0xea

#define CMD_SetAlljointStatus  0x01
#define CMD_GetAlljointStatus  0x02
#define CMD_SetjointID         0x03

uint8_t rxbuf[256]={0};
uint8_t txbuf[256]={0};


#define FLASH_JointStatusData ADDR_FLASH_SECTOR_11


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
    /*char str[80];
    sprintf(str,format);
    HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),10);*/
    printf(format);
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
    myprintf("inverted=%d\r\n",local.inverted);
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
    myprintf("enable=%d\r\n",local.enable);
    //HAL_Delay(200);
    return 0;
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
            electron.SetJointInitAngle(electron.joint[idbuf], rx.initAngle);
            local.initAngle=rx.initAngle;
            ProtocolItem.SaveEn = true;
        }

        if(rx.torqueLimit!=local.torqueLimit)
        {
            electron.SetJointTorqueLimit(electron.joint[idbuf], rx.torqueLimit);
            local.torqueLimit=rx.torqueLimit;
            ProtocolItem.SaveEn = true;
        }

        if(rx.kp!=local.kp)
        {
            electron.SetJointKp(electron.joint[idbuf],rx.kp);
            local.kp=rx.kp;
            ProtocolItem.SaveEn = true;
        }

        if(rx.ki!=local.ki)
        {
            electron.SetJointKi(electron.joint[idbuf],rx.ki);
            local.ki=rx.kp;
            ProtocolItem.SaveEn = true;
        }

        if(rx.kv!=local.kv)
        {
            electron.SetJointKv(electron.joint[idbuf],rx.kv);
            local.kv=rx.kv;
            ProtocolItem.SaveEn = true;
        }

        if(rx.kd!=local.kd)
        {
            electron.SetJointKd(electron.joint[idbuf],rx.kd);
            local.kd=rx.kd;
            ProtocolItem.SaveEn = true;
        }


        if(rx.enable!=local.enable)
        {
            electron.SetJointEnable(electron.joint[idbuf],rx.enable);
            local.enable=rx.enable;
            ProtocolItem.SaveEn = true;
        }
    }

    if(ProtocolItem.SaveEn == true)
    {
        memcpy(&ElectronBotjoint[idbuf],&local,sizeof(ElectronBotJointStatus_t));
        //p=&ElectronBotjoint[0];
        p=ElectronBotjoint;
        SaveJointStatusToFlash((uint8_t *)p,sizeof(ElectronBotJointStatus_t)*6);
    }
    
    BufClear(rxbuf,0,sizeof(rxbuf));
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
    HAL_Delay(200);
    MX_USART1_UART_Init();
    //HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    //HAL_UART_Transmit(&huart1,&testuart[0], sizeof(testuart),50);
    HAL_UART_Transmit(&huart1,testuart, sizeof(testuart),50);
    HAL_Delay(200);
    myprintf("\r\n");
    myprintf("myprintf:%s\r\n",testuart);
    myprintf("\r\n");
    //testDataSaveToFlash();

    myprintf("myprintf:%s\r\n",testuart);
    HAL_Delay(2000);
    electron.lcd->Init(Screen::DEGREE_0);
    electron.lcd->SetWindow(0, 239, 0, 239);

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