//
// Created by Administrator on 2022/8/5.
//
#include "protocol.h"
void test_protocol()
{
    int i=1;
}

ElectronBotJointStatus_t ElectronBotjoint[6];
JointStatus_t joint[7];
ProtocolItem_t ProtocolItem;
txbuf_t txbuf;
rxbuf_t rxbuf;


int fputc(int ch,FILE *f)
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
void ProtocolProcessing(uint8_t* buf, uint16_t len)
{
    uint8_t frameBuf[1024] = { 0 };
    uint8_t idbuf = 0;
    ElectronBotJointStatus_t* test;

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