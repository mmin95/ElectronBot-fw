//
// Created by Administrator on 2022/8/5.
//
#include "protocol.h"
#include "robot.h"


#define FLASH_JointStatusData ADDR_FLASH_SECTOR_11

void test_protocol()
{
    int i=1;
}

flashSave_t flashSave;
//ElectronBotJointStatus_t ElectronBotjoint[6];
JointStatus_t joint[7];
ProtocolItem_t ProtocolItem;
txbuf_t txbuf;
rxbuf_t rxbuf;


/*int fputc(int ch,FILE *f)
{
    uint8_t temp[1] = {(uint8_t)ch};
    //HAL_UART_Transmit(&huart2, temp, 1, 2);
    HAL_UART_Transmit(&huart1,temp, 1,10);
    return ch;
}

void myPrintf(const char* format, ...)
{
    printf(format);
}

#define  myDebugEn 0
void myDebug(const char* format, ...)
{
#if  myDebugEn
    printf(format);
#endif
}
*/

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


void BufClear(uint8_t* buf, uint8_t value, uint16_t len)
{
    memset(buf, value, len);
}

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


void ComposeProtocolFrame(uint8_t* frameBuf, uint16_t* frameLen, ProtocolItem_t* items)
{
    uint16_t len = 0, itemLen = 0;
    uint8_t checkValue = 0;

    frameBuf[0] = FrameHead;
    len++;

    itemLen = sizeof(items->ElectronBotID);
    memcpy(&frameBuf[len], &items->ElectronBotID, itemLen);
    len += itemLen;

    if (items->frame == ResponseFrame)
    {
        items->cmd = items->cmd | 0x80;
    }
    itemLen = sizeof(items->cmd);
    memcpy(&frameBuf[len], &items->cmd, itemLen);
    len += itemLen;

    itemLen = sizeof(items->jointID);
    memcpy(&frameBuf[len], &items->jointID, itemLen);
    len += itemLen;

    itemLen = sizeof(items->dataLen);
    memcpy(&frameBuf[len], &items->dataLen, itemLen);
    len += itemLen;

    //itemLen=sizeof(items->dataLen);
    if (items->dataLen > 0)
    {
        memcpy(&frameBuf[len], items->data, items->dataLen);
        len += items->dataLen;
    }

    checkValue = CheckSum(frameBuf, items->dataLen);
    frameBuf[len] = checkValue;
    len++;

    frameBuf[len] = FrameTail;
    len++;

    *frameLen = len;
}


//查找inBuf里是否有协议帧,有则将协议帧拷贝到outBuf
bool ProtocolLookUp(uint8_t* frameBuf, uint8_t* inBuf, uint16_t inLen)
{
    uint16_t i;
    uint16_t datalen = 0;
    uint8_t checkValue = 0;
    if (inLen < 11)
    {
        return false;
    }
    for (i = 0; i < inLen; i++)
    {
        if (inBuf[i] == FrameHead && inLen - i >= 11)
        {
            datalen = inBuf[i + 7] + inBuf[i + 8] * 256;
            if (inBuf[i + 7 + 2 + datalen + 2 - 1] == FrameTail)
            {
                checkValue = inBuf[i + 7 + 2 + datalen + 1 - 1];
                if (checkValue == CheckSum(&inBuf[i], datalen))
                {
                    memcpy(frameBuf, &inBuf[i], 7 + 2 + datalen + 2);
                    // *outframeHead = i;
                    return true;
                    // ProtocolProcessing(rxbuf);
                }
            }
        }
    }
    return false;
}

//此函数可以自己重新写
//extern Robot electron();
//extern Robot electron(&hspi1, &hi2c1);
extern Robot electron;
void ProtocolProcessing(uint8_t* buf, uint16_t len)
{
    uint8_t frameBuf[1024] = { 0 };
    uint8_t idbuf = 0;
    ElectronBotJointStatus_t* test;
    ElectronBotJointStatus_t rx,local,*p;
    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    if (ProtocolLookUp(frameBuf, buf, len) == false)
    {
        myPrintf("\r\n  No Protocol!!!!!!! \r\n");
        return;
    }

    ProtocolItem.ElectronBotID = frameBuf[1] + frameBuf[2] * 256 + frameBuf[3] * 256 * 256 + frameBuf[4] * 256 * 256 * 256;
    ProtocolItem.cmd = frameBuf[5];
    ProtocolItem.jointID = frameBuf[6];
    ProtocolItem.dataLen = frameBuf[7] + frameBuf[8] * 256;
    ProtocolItem.data = &frameBuf[9];

    myPrintf("ElectronBotID=%d\r\n", ProtocolItem.ElectronBotID);
    myPrintf("jointID=%d\r\n", ProtocolItem.jointID);
    myPrintf("dataLen=%d\r\n", ProtocolItem.dataLen);

    test = (ElectronBotJointStatus_t*)ProtocolItem.data;

    idbuf = ProtocolItem.jointID / 2;
    if (ProtocolItem.cmd == CMD_WriteAllJointStatus || ProtocolItem.cmd == CMD_WriteAllJointStatus |0x80)
    {
        myPrintf("cmd=CMD_WriteAllJointStatus\r\n");
        myPrintf("angleMax=%f\r\n", test->angleMax);
        myPrintf("angle=%f\r\n", test->angle);
        myPrintf("modelAngelMin=%f\r\n", test->modelAngelMin);
        myPrintf("modelAngelMax=%f\r\n", test->modelAngelMax);
        myPrintf("inverted=%s\r\n", test->inverted ? "true" : "false");
        myPrintf("initAngle=%f\r\n", test->initAngle);
        myPrintf("torqueLimit=%f\r\n", test->torqueLimit);
        myPrintf("kp=%f\r\n", test->kp);
        myPrintf("ki=%f\r\n", test->ki);
        myPrintf("kv=%f\r\n", test->kv);
        myPrintf("kd=%f\r\n", test->kd);
        myPrintf("enable=%s\r\n", test->enable ? "true" : "false");
    }

    //uint8_t idbuf=0;
    //uint8_t *buf;
    // buf=&rxbuf.buf[rxbuf.frameHead];

    ProtocolItem.ElectronBotID=buf[1]+buf[2]*256+buf[3]*256*256+buf[4]*256*256*256;
    ProtocolItem.cmd=buf[5];
    ProtocolItem.jointID=buf[6];
    ProtocolItem.dataLen=buf[7]+buf[8]*256;
    ProtocolItem.data=&buf[9];
    ProtocolItem.SaveEn=false;

    idbuf=ProtocolItem.jointID/2;
    memcpy(&local,&flashSave.ElectronBotjoint[idbuf],sizeof(ElectronBotJointStatus_t));

    if(ProtocolItem.cmd == CMD_WriteAllJointStatus )
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
            ProtocolItem.SaveEn = false;

            memcpy(&flashSave.ElectronBotjoint[idbuf],&local,sizeof(ElectronBotJointStatus_t));
            HAL_Delay(20);
            //p=&ElectronBotjoint[0];
           // p=ElectronBotjoint;
            flashSave.saveFlag=0x5555aaaa;
            //SaveJointStatusToFlash((uint8_t *)&flashSave,sizeof(flashSave_t));
            //electron.SetJointEnable(electron.joint[0], true);

            ProtocolItem.dataLen=0;
            ProtocolItem.frame=ResponseFrame;
            ComposeProtocolFrame(txbuf.buf,&txbuf.dataLen,&ProtocolItem);

            /*uint8_t ret;
            do
            {
                ret = CDC_Transmit_HS(txbuf.buf,txbuf.dataLen);
            } while (ret != USBD_OK);
             */
            //electron.SendUsbPacket(frameBuf,sizeof(frameBuf));
           // electron.SendUsbPacket(txbuf.buf,txbuf.dataLen);

            BufClear((uint8_t* )&txbuf,0,sizeof(txbuf));

        }
    }


    if(ProtocolItem.cmd == CMD_ReadAllJointStatus)
    {
        ProtocolItem.dataLen= sizeof(ElectronBotJointStatus_t);
        ProtocolItem.frame=ResponseFrame;
        ProtocolItem.data=(uint8_t*)&flashSave.ElectronBotjoint[idbuf];
        ComposeProtocolFrame(txbuf.buf,&txbuf.dataLen,&ProtocolItem);
    }

    if(ProtocolItem.cmd == CMD_WriteJointID )
    {
        uint8_t newId=0;
        newId=ProtocolItem.data[0];

        electron.joint[0].id = 0;  //这里还是用广播
        electron.joint[0].angleMin = 0;
        electron.joint[0].angleMax = 180;
        electron.joint[0].angle = 0;
        electron.joint[0].modelAngelMin = -90;
        electron.joint[0].modelAngelMax = 90;
        electron.SetJointId(electron.joint[0], newId);

        HAL_Delay(1000);
        // 3.等待舵机参数保存。将主控 joint[0].id改为 2
        electron.joint[0].id = newId;  //这里改为新地址
        electron.joint[0].angleMin = 0;
        electron.joint[0].angleMax = 180;
        electron.joint[0].angle = 0;
        electron.joint[0].modelAngelMin = -90;
        electron.joint[0].modelAngelMax = 90;
        // 4.使用新设置的2地址通讯
        electron.SetJointEnable(electron.joint[0], true);

        ProtocolItem.dataLen=0;
        ProtocolItem.frame=ResponseFrame;
        ComposeProtocolFrame(txbuf.buf,&txbuf.dataLen,&ProtocolItem);
    }

    BufClear((uint8_t* )&rxbuf,0,sizeof(rxbuf));
}


void JointStatusUpdata(void)
{
    ElectronBotJointStatus_t flashSaved;
    int i=0;

    ReadJointStatusFromFlash((uint8_t *)&flashSave,sizeof(flashSave_t));

    if(flashSave.saveFlag!=0x5555aaaa)
    {
        for(i=1;i<7;i++)
        {

            flashSave.ElectronBotjoint[i].angleMin = electron.joint[i].angleMin;
            flashSave.ElectronBotjoint[i].angleMax = electron.joint[i].angleMax;

            flashSave.ElectronBotjoint[i].modelAngelMax=electron.joint[i].modelAngelMax;
            flashSave.ElectronBotjoint[i].modelAngelMin=electron.joint[i].modelAngelMin;

            //flashSave.ElectronBotjoint[i].enable=electron.joint[i].enable;
            flashSave.ElectronBotjoint[i].inverted=electron.joint[i].inverted;

            flashSave.ElectronBotjoint[i].enable=false;
            flashSave.ElectronBotjoint[i].torqueLimit=0.5;
            flashSave.ElectronBotjoint[i].kp=10;
            flashSave.ElectronBotjoint[i].ki=0;
            flashSave.ElectronBotjoint[i].kv=0;
            flashSave.ElectronBotjoint[i].kd=50;

            electron.UpdateJointAngle(electron.joint[i], 0);
        }
    }
    else
    {
        for(i=1;i<7;i++)
        {
            memcpy(&flashSaved,&flashSave.ElectronBotjoint[i],sizeof(ElectronBotJointStatus_t));
            electron.joint[i].angleMin=flashSaved.angleMin;
            electron.joint[i].angleMax=flashSaved.angleMax;

            electron.joint[i].modelAngelMax =flashSaved.modelAngelMax;
            electron.joint[i].modelAngelMin=flashSaved.modelAngelMin;

            electron.joint[i].inverted=flashSaved.inverted;

            electron.UpdateJointAngle(electron.joint[i], 0);//

            electron.SetJointInitAngle(electron.joint[i], flashSaved.initAngle);
            electron.SetJointTorqueLimit(electron.joint[i], flashSaved.torqueLimit);
            electron.SetJointKp(electron.joint[i],flashSaved.kp);
            electron.SetJointKi(electron.joint[i],flashSaved.ki);
            electron.SetJointKv(electron.joint[i],flashSaved.kv);
            electron.SetJointKd(electron.joint[i],flashSaved.kd);

            electron.SetJointEnable(electron.joint[i],flashSaved.kd);
        }
    }
}

uint8_t testbuf[300] = { 0 };
uint8_t testbuf2[300] = { 0 };

void testComposeProtocolFrame()
{
    ElectronBotJointStatus_t test, * p;

    p = &test;
    BufClear((uint8_t*)p, 0, sizeof(ElectronBotJointStatus_t));

    test.angleMin = 10;
    test.angleMax = 30;
    test.angle = 0;
    test.modelAngelMin = 12;
    test.modelAngelMax = 32;
    test.inverted = true;
    test.initAngle = 15;
    test.torqueLimit = 0.5;
    test.kp = 20;
    test.ki = 1;
    test.kv = 2;
    test.kd = 3;
    test.enable = true;

    memcpy((uint8_t*)testbuf, (uint8_t*)p, sizeof(ElectronBotJointStatus_t));

    ProtocolItem_t* p2;
    p2 = &ProtocolItem;
    BufClear((uint8_t*)p2, 0, sizeof(ProtocolItem_t));

    ProtocolItem.ElectronBotID = 12345678;
    ProtocolItem.cmd = CMD_WriteAllJointStatus;
    ProtocolItem.jointID = 12;
    ProtocolItem.dataLen = sizeof(ElectronBotJointStatus_t);
    ProtocolItem.data = testbuf;
    ProtocolItem.frame = CommandFrame;

    ComposeProtocolFrame(txbuf.buf, &txbuf.dataLen ,&ProtocolItem);//组帧
    ProtocolProcessing(txbuf.buf, txbuf.dataLen);//解帧

    BufClear((uint8_t*)&txbuf, 0, sizeof(txbuf));
}

//uint8_t frameBuf[1024] = {0};
void testReceiveMasterUsbData(uint8_t *buf, uint16_t len)
{
    //uint8_t frameBuf[200] = {0};
    uint8_t frameBuf[400] = {0};
    ElectronBotJointStatus_t rx,local,*p;
    p=&rx;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));
    p=&local;
    BufClear((uint8_t *)p,0,sizeof(ElectronBotJointStatus_t));

    if (ProtocolLookUp(frameBuf, buf, len) == false)
    {
        myPrintf("\r\n testReceiveMasterUsbData() not found protocol\r\n");
        return;
    }

    //uint8_t *buf;
    //buf=&rxbuf.buf[rxbuf.frameHead];
    //buf=inbuf;

    ProtocolItem.ElectronBotID=frameBuf[1]+frameBuf[2]*256+frameBuf[3]*256*256+frameBuf[4]*256*256*256;
    ProtocolItem.cmd=frameBuf[5];
    ProtocolItem.jointID=frameBuf[6];
    ProtocolItem.dataLen=frameBuf[7]+frameBuf[8]*256;
    ProtocolItem.data=&frameBuf[9];
    ProtocolItem.SaveEn=false;

    memcpy(&local,ProtocolItem.data,sizeof(ElectronBotJointStatus_t));

    myPrintf("ElectronBotID=%d\r\n",ProtocolItem.ElectronBotID);
    HAL_Delay(200);
    myPrintf("cmd=%d\r\n",ProtocolItem.cmd);
    HAL_Delay(200);
    myPrintf("jointID=%d\r\n",ProtocolItem.jointID);
    HAL_Delay(200);
    myPrintf("angleMin=%f\r\n",local.angleMin);
    //char str[80];
    // smyPrintf(str,"angleMin=%f\r\n",local.angleMin);
    // HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),10);

    HAL_Delay(200);
    myPrintf("angleMax=%f\r\n",local.angleMax);
    HAL_Delay(200);
    myPrintf("angle=%f\r\n",local.angle);
    HAL_Delay(200);
    myPrintf("modelAngelMin=%f\r\n",local.modelAngelMin);
    HAL_Delay(200);
    myPrintf("modelAngelMax=%f\r\n",local.modelAngelMax);
    HAL_Delay(200);
    //myPrintf("inverted=%d\r\n",local.inverted);
    myPrintf("inverted=%s\r\n",local.inverted?"true" :"false");
    HAL_Delay(200);
    myPrintf("initAngle=%f\r\n",local.initAngle);
    HAL_Delay(200);
    myPrintf("torqueLimit=%f\r\n",local.torqueLimit);
    HAL_Delay(200);
    myPrintf("kp=%f\r\n",local.kp);
    HAL_Delay(200);
    myPrintf("ki=%f\r\n",local.ki);
    HAL_Delay(200);
    myPrintf("kv=%f\r\n",local.kv);
    HAL_Delay(200);
    myPrintf("kd=%f\r\n",local.kd);
    HAL_Delay(200);
    myPrintf("enable=%s\r\n",local.enable?"true" :"false");
    HAL_Delay(200);

}