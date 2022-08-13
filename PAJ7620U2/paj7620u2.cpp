//
// Created by Administrator on 2022/8/7.
//
#include "stm32f4xx.h"
#include "paj7620u2.h"
#include "paj7620u2_iic.h"
#include "paj7620u2_cfg.h"
#include "stdio.h"
#include "protocol.h"

void test_paj7260u2(void)
{
    int i=0;
}


//选择PAJ7620U2 BANK区域
void paj7620u2_selectBank(bank_e bank)
{
    switch(bank)
    {
        case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0寄存器区域
        case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1寄存器区域
    }

}

//PAJ7620U2唤醒
u8 paj7620u2_wakeup(void)
{
    u8 data=0x0a;
    GS_WakeUp();//唤醒PAJ7620U2
    HAL_Delay(5);//唤醒时间>400us
    GS_WakeUp();//唤醒PAJ7620U2
    HAL_Delay(5);//唤醒时间>400us
    paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
    data = GS_Read_Byte(0x00);//读取状态
    if(data!=0x20) return 0; //唤醒失败

    return 1;
}

//PAJ7620U2初始化
//返回值：0:失败 1:成功
u8 paj7620u2_init(void)
{
    u8 i;
    u8 status;
    uint8_t readdata=0;

    GS_i2c_init();//IIC初始化
    status = paj7620u2_wakeup();//唤醒PAJ7620U2
    if(!status) return 0;
    paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
    for(i=0;i<INIT_SIZE;i++)
    {
        GS_Write_Byte(init_Array[i][0],init_Array[i][1]);//初始化PAJ7620U2
        myPrintf("write address %x  write data %x\r\n",init_Array[i][0],init_Array[i][1]);
        HAL_Delay(20);
        readdata=GS_Read_Byte(init_Array[i][0]);//初始化PAJ7620U2
        myPrintf("read address %x  read data %x\r\n",init_Array[i][0],readdata);
        HAL_Delay(20);
        myPrintf("********");
        HAL_Delay(10);
    }
    paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域

    return 1;
}

//手势识别测试
void Gesture_test(void)
{
    u8 i;
    u8 status;
    u8 key;
    u8 data[2]={0x00};
    u16 gesture_data;
    u8 ledflash=0;

    paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
    for(i=0;i<GESTURE_SIZE;i++)
    {
        GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
    }
    paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
    i=0;
    //POINT_COLOR=BLUE;//设置字体为蓝色
    //LCD_Fill(30,170,300,300,WHITE);
    //LCD_ShowString(30,180,200,16,16,"KEY_UP: Exit the test");
    //LCD_ShowString(30,210,200,16,16,"Gesture test");
    //POINT_COLOR=RED;//设置字体为蓝色
    while(1)
    {
        //key = KEY_Scan(0);
        //if(key==WKUP_PRES)
       // {
        //    GS_Write_Byte(PAJ_SET_INT_FLAG1,0X00);//关闭手势识别中断输出
         //   GS_Write_Byte(PAJ_SET_INT_FLAG2,0X00);
         //   break;
        //}
        status = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&data[0]);//读取手势状态
        if(!status)
        {
            gesture_data =(u16)data[1]<<8 | data[0];
            if(gesture_data)
            {
                switch(gesture_data)
                {
                    case GES_UP:
                        myPrintf("Left\r\n");            ledflash=1;      break; //向上Left
                        //myPrintf("Up\r\n");            ledflash=1;      break; //向上Left
                    case GES_DOWM:
                        myPrintf("Right\r\n");          ledflash=1;      break; //向下Right
                        //myPrintf("Dowm\r\n");          ledflash=1;      break; //向下Right
                    case GES_LEFT:
                        myPrintf("Dowm\r\n");          ledflash=1;      break; //向左Dowm
                        //myPrintf("Left\r\n");          ledflash=1;      break; //向左Dowm
                    case GES_RIGHT:
                        myPrintf("Up\r\n");         ledflash=1;      break; //向右Up
                        //myPrintf("Right\r\n");         ledflash=1;      break; //向右Up
                    case GES_FORWARD:
                        myPrintf("Forward\r\n");       ledflash=1;      break; //向前
                    case GES_BACKWARD:
                        myPrintf("Backward\r\n");      ledflash=1;      break; //向后
                    case GES_CLOCKWISE:
                        myPrintf("Clockwise\r\n");     ledflash=1;      break; //顺时针
                    case GES_COUNT_CLOCKWISE:
                        myPrintf("AntiClockwise\r\n"); ledflash=1;      break; //逆时针
                    case GES_WAVE:
                        myPrintf("Wave\r\n");          ledflash=1;      break; //挥动
                    default:  ledflash=0; break;
               }
                if(ledflash)//DS1闪烁
                {
                   // LED1=0;delay_ms(80);LED1=1;delay_ms(80);
                   // LED1=0;delay_ms(80);LED1=1;delay_ms(80);
                    //LCD_ShowString(40,250,200,16,24,"                        ");
                    ledflash=0;
                }
            }

        }
        //delay_ms(50);
        //myPrintf("%d\r\n",status);
        HAL_Delay(50);
        //i++;
        //if(i==5)
        //{
        //    LED0=!LED0;//提示系统正在运行
        //    i=0;
        //}
    }
}

//手势识别测试
void Gesture(void)
{
    u8 i;
    u8 status;
    u8 key;
    u8 data[2]={0x00};
    u16 gesture_data;
    u8 ledflash=0;

    static bool initOK= false;

    if(initOK!= true) {
        paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
        for (i = 0; i < GESTURE_SIZE; i++) {
            GS_Write_Byte(gesture_arry[i][0], gesture_arry[i][1]);//手势识别模式初始化
        }
        paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
        i = 0;
        initOK= true;
    }
    //while(1) {
        status = GS_Read_nByte(PAJ_GET_INT_FLAG1, 2, &data[0]);//读取手势状态
        if (!status) {
            gesture_data = (u16) data[1] << 8 | data[0];
            if (gesture_data) {
                switch (gesture_data) {
                    case GES_UP:
                        myPrintf("Left\r\n");
                        ledflash = 1;
                        break; //向上Left
                        //myPrintf("Up\r\n");            ledflash=1;      break; //向上Left
                    case GES_DOWM:
                        myPrintf("Right\r\n");
                        ledflash = 1;
                        break; //向下Right
                        //myPrintf("Dowm\r\n");          ledflash=1;      break; //向下Right
                    case GES_LEFT:
                        myPrintf("Dowm\r\n");
                        ledflash = 1;
                        break; //向左Dowm
                        //myPrintf("Left\r\n");          ledflash=1;      break; //向左Dowm
                    case GES_RIGHT:
                        myPrintf("Up\r\n");
                        ledflash = 1;
                        break; //向右Up
                        //myPrintf("Right\r\n");         ledflash=1;      break; //向右Up
                    case GES_FORWARD:
                        myPrintf("Forward\r\n");
                        ledflash = 1;
                        break; //向前
                    case GES_BACKWARD:
                        myPrintf("Backward\r\n");
                        ledflash = 1;
                        break; //向后
                    case GES_CLOCKWISE:
                        myPrintf("Clockwise\r\n");
                        ledflash = 1;
                        break; //顺时针
                    case GES_COUNT_CLOCKWISE:
                        myPrintf("AntiClockwise\r\n");
                        ledflash = 1;
                        break; //逆时针
                    case GES_WAVE:
                        myPrintf("Wave\r\n");
                        ledflash = 1;
                        break; //挥动
                    default:
                        ledflash = 0;
                        break;
                }

            }

        }

        HAL_Delay(10);
   // }
}