//
// Created by Administrator on 2022/8/10.
//

/*
 * paj7620.cpp
 * A library for Grove-Guesture 1.0
 *
 * Copyright (c) 2015 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Wuruibin & Xiangnan
 * Modified Time: June 2015
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//#include <Wire.h>
#include "paj7620.h"
//#include <Arduino.h>


#include "stm32f4xx.h"
//#include "paj7620u2.h"
#include "paj7620u2_iic.h"
#include "paj7620u2_cfg.h"
#include "stdio.h"
#include "protocol.h"

// PAJ7620U2_20140305.asc
/* Registers' initialization data */
unsigned char initRegisterArray[][2] = {	// Initial Gesture
        {0xEF,0x00},
        {0x32,0x29},
        {0x33,0x01},
        {0x34,0x00},
        {0x35,0x01},
        {0x36,0x00},
        {0x37,0x07},
        {0x38,0x17},
        {0x39,0x06},
        {0x3A,0x12},
        {0x3F,0x00},
        {0x40,0x02},
        {0x41,0xFF},
        {0x42,0x01},
        {0x46,0x2D},
        {0x47,0x0F},
        {0x48,0x3C},
        {0x49,0x00},
        {0x4A,0x1E},
        {0x4B,0x00},
        {0x4C,0x20},
        {0x4D,0x00},
        {0x4E,0x1A},
        {0x4F,0x14},
        {0x50,0x00},
        {0x51,0x10},
        {0x52,0x00},
        {0x5C,0x02},
        {0x5D,0x00},
        {0x5E,0x10},
        {0x5F,0x3F},
        {0x60,0x27},
        {0x61,0x28},
        {0x62,0x00},
        {0x63,0x03},
        {0x64,0xF7},
        {0x65,0x03},
        {0x66,0xD9},
        {0x67,0x03},
        {0x68,0x01},
        {0x69,0xC8},
        {0x6A,0x40},
        {0x6D,0x04},
        {0x6E,0x00},
        {0x6F,0x00},
        {0x70,0x80},
        {0x71,0x00},
        {0x72,0x00},
        {0x73,0x00},
        {0x74,0xF0},
        {0x75,0x00},
        {0x80,0x42},
        {0x81,0x44},
        {0x82,0x04},
        {0x83,0x20},
        {0x84,0x20},
        {0x85,0x00},
        {0x86,0x10},
        {0x87,0x00},
        {0x88,0x05},
        {0x89,0x18},
        {0x8A,0x10},
        {0x8B,0x01},
        {0x8C,0x37},
        {0x8D,0x00},
        {0x8E,0xF0},
        {0x8F,0x81},
        {0x90,0x06},
        {0x91,0x06},
        {0x92,0x1E},
        {0x93,0x0D},
        {0x94,0x0A},
        {0x95,0x0A},
        {0x96,0x0C},
        {0x97,0x05},
        {0x98,0x0A},
        {0x99,0x41},
        {0x9A,0x14},
        {0x9B,0x0A},
        {0x9C,0x3F},
        {0x9D,0x33},
        {0x9E,0xAE},
        {0x9F,0xF9},
        {0xA0,0x48},
        {0xA1,0x13},
        {0xA2,0x10},
        {0xA3,0x08},
        {0xA4,0x30},
        {0xA5,0x19},
        {0xA6,0x10},
        {0xA7,0x08},
        {0xA8,0x24},
        {0xA9,0x04},
        {0xAA,0x1E},
        {0xAB,0x1E},
        {0xCC,0x19},
        {0xCD,0x0B},
        {0xCE,0x13},
        {0xCF,0x64},
        {0xD0,0x21},
        {0xD1,0x0F},
        {0xD2,0x88},
        {0xE0,0x01},
        {0xE1,0x04},
        {0xE2,0x41},
        {0xE3,0xD6},
        {0xE4,0x00},
        {0xE5,0x0C},
        {0xE6,0x0A},
        {0xE7,0x00},
        {0xE8,0x00},
        {0xE9,0x00},
        {0xEE,0x07},
        {0xEF,0x01},
        {0x00,0x1E},
        {0x01,0x1E},
        {0x02,0x0F},
        {0x03,0x10},
        {0x04,0x02},
        {0x05,0x00},
        {0x06,0xB0},
        {0x07,0x04},
        {0x08,0x0D},
        {0x09,0x0E},
        {0x0A,0x9C},
        {0x0B,0x04},
        {0x0C,0x05},
        {0x0D,0x0F},
        {0x0E,0x02},
        {0x0F,0x12},
        {0x10,0x02},
        {0x11,0x02},
        {0x12,0x00},
        {0x13,0x01},
        {0x14,0x05},
        {0x15,0x07},
        {0x16,0x05},
        {0x17,0x07},
        {0x18,0x01},
        {0x19,0x04},
        {0x1A,0x05},
        {0x1B,0x0C},
        {0x1C,0x2A},
        {0x1D,0x01},
        {0x1E,0x00},
        {0x21,0x00},
        {0x22,0x00},
        {0x23,0x00},
        {0x25,0x01},
        {0x26,0x00},
        {0x27,0x39},
        {0x28,0x7F},
        {0x29,0x08},
        {0x30,0x03},
        {0x31,0x00},
        {0x32,0x1A},
        {0x33,0x1A},
        {0x34,0x07},
        {0x35,0x07},
        {0x36,0x01},
        {0x37,0xFF},
        {0x38,0x36},
        {0x39,0x07},
        {0x3A,0x00},
        {0x3E,0xFF},
        {0x3F,0x00},
        {0x40,0x77},
        {0x41,0x40},
        {0x42,0x00},
        {0x43,0x30},
        {0x44,0xA0},
        {0x45,0x5C},
        {0x46,0x00},
        {0x47,0x00},
        {0x48,0x58},
        {0x4A,0x1E},
        {0x4B,0x1E},
        {0x4C,0x00},
        {0x4D,0x00},
        {0x4E,0xA0},
        {0x4F,0x80},
        {0x50,0x00},
        {0x51,0x00},
        {0x52,0x00},
        {0x53,0x00},
        {0x54,0x00},
        {0x57,0x80},
        {0x59,0x10},
        {0x5A,0x08},
        {0x5B,0x94},
        {0x5C,0xE8},
        {0x5D,0x08},
        {0x5E,0x3D},
        {0x5F,0x99},
        {0x60,0x45},
        {0x61,0x40},
        {0x63,0x2D},
        {0x64,0x02},
        {0x65,0x96},
        {0x66,0x00},
        {0x67,0x97},
        {0x68,0x01},
        {0x69,0xCD},
        {0x6A,0x01},
        {0x6B,0xB0},
        {0x6C,0x04},
        {0x6D,0x2C},
        {0x6E,0x01},
        {0x6F,0x32},
        {0x71,0x00},
        {0x72,0x01},
        {0x73,0x35},
        {0x74,0x00},
        {0x75,0x33},
        {0x76,0x31},
        {0x77,0x01},
        {0x7C,0x84},
        {0x7D,0x03},
        {0x7E,0x01},
};


/****************************************************************
 * Function Name: paj7620WriteReg
 * Description:  PAJ7620 Write reg cmd
 * Parameters: addr:reg address; cmd:function data
 * Return: error code; success: return 0
****************************************************************/
//uint8_t paj7620WriteReg(uint8_t addr, uint8_t cmd)
//{
//    char i = 1;
//    Wire.beginTransmission(PAJ7620_ID);		// start transmission to device
//    //write cmd
//    Wire.write(addr);						// send register address
//    Wire.write(cmd);						// send value to write
//    i = Wire.endTransmission();  		    // end transmission
//    if(0 != i)
//    {
//        Serial.print("Transmission error!!!\n");
//    }
//    return i;
//}

uint8_t paj7620WriteReg(uint8_t addr, uint8_t cmd)
{
    HAL_StatusTypeDef state = HAL_ERROR;

    do
    {
        HAL_I2C_Master_Transmit(&hi2c1, PAJ7620_ID, &addr, 1, 10);
    } while (state != HAL_OK);


    do
    {
        HAL_I2C_Master_Transmit(&hi2c1, PAJ7620_ID, &cmd, 1, 10);
    } while (state != HAL_OK);

    return 0;
}



/****************************************************************
 * Function Name: paj7620ReadReg
 * Description:  PAJ7620 read reg data
 * Parameters: addr:reg address;
 *			   qty:number of data to read, addr continuously increase;
 *			   data[]:storage memory start address
 *
 * Return: error code; success: return 0
****************************************************************/
//uint8_t paj7620ReadReg(uint8_t addr, uint8_t qty, uint8_t data[])
//{
//
//
//    uint8_t error;
//    Wire.beginTransmission(PAJ7620_ID);
//    Wire.write(addr);
//    error = Wire.endTransmission();
//
//    if(0 != error)
//    {
//        Serial.print("Transmission error!!!\n");
//        return error; //return error code
//    }
//
//    Wire.requestFrom((int)PAJ7620_ID, (int)qty);
//
//    while (Wire.available())
//    {
//        *data = Wire.read();
//
//#ifdef debug    //debug
//        Serial.print("addr:");
//    Serial.print(addr++, HEX);
//    Serial.print("  data:");
//    Serial.println(*data, HEX);
//#endif
//
//        data++;
//    }
//    return 0;
//}

uint8_t paj7620ReadReg(uint8_t addr, uint8_t qty, uint8_t* data)
{
    HAL_StatusTypeDef state = HAL_ERROR;

    do
    {
        HAL_I2C_Master_Transmit(&hi2c1, PAJ7620_ID, &addr, 1, 10);
    } while (state != HAL_OK);

    do
    {
        HAL_I2C_Master_Receive(&hi2c1, PAJ7620_ID|0x01, data, qty, 10);
    } while (state != HAL_OK);

    return 0;
}

/****************************************************************
 * Function Name: paj7620SelectBank
 * Description:  PAJ7620 select register bank
 * Parameters: BANK0, BANK1
 * Return: none
****************************************************************/
void paj7620SelectBank(bank_e bank)
{
    switch(bank){
        case BANK0:
            paj7620WriteReg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK0);
            break;
        case BANK1:
            paj7620WriteReg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK1);
            break;
        default:
            break;
    }
}

/****************************************************************
 * Function Name: paj7620Init
 * Description:  PAJ7620 REG INIT
 * Parameters: none
 * Return: error code; success: return 0
****************************************************************/
//uint8_t paj7620Init(void)
//{
//    //Near_normal_mode_V5_6.15mm_121017 for 940nm
//    int i = 0;
//    uint8_t error;
//    uint8_t data0 = 0, data1 = 0;
//    //wakeup the sensor
//    delayMicroseconds(700);	//Wait 700us for PAJ7620U2 to stabilize
//
//    Wire.begin();
//
//    Serial.println("INIT SENSOR...");
//
//    paj7620SelectBank(BANK0);
//    paj7620SelectBank(BANK0);
//
//    error = paj7620ReadReg(0, 1, &data0);
//    if (error)
//    {
//        return error;
//    }
//    error = paj7620ReadReg(1, 1, &data1);
//    if (error)
//    {
//        return error;
//    }
//    Serial.print("Addr0 =");
//    Serial.print(data0 , HEX);
//    Serial.print(",  Addr1 =");
//    Serial.println(data1 , HEX);
//
//    if ( (data0 != 0x20 ) || (data1 != 0x76) )
//    {
//        return 0xff;
//    }
//    if ( data0 == 0x20 )
//    {
//        Serial.println("wake-up finish.");
//    }
//
//    for (i = 0; i < INIT_REG_ARRAY_SIZE; i++)
//    {
//        paj7620WriteReg(initRegisterArray[i][0], initRegisterArray[i][1]);
//    }
//
//    paj7620SelectBank(BANK0);  //gesture flage reg in Bank0
//
//    Serial.println("Paj7620 initialize register finished.");
//    return 0;
//}


uint8_t paj7620Init(void)
{
    //Near_normal_mode_V5_6.15mm_121017 for 940nm
    int i = 0;
    uint8_t error;
    uint8_t data0 = 0, data1 = 0;
    //wakeup the sensor
    MX_I2C1_Init();
    HAL_Delay(700);	//Wait 700us for PAJ7620U2 to stabilize

   // Wire.begin();

   // Serial.println("INIT SENSOR...");

    paj7620SelectBank(BANK0);
    paj7620SelectBank(BANK0);

    error = paj7620ReadReg(0, 1, &data0);
    if (error)
    {
        return error;
    }
    error = paj7620ReadReg(1, 1, &data1);
    if (error)
    {
        return error;
    }
    myPrintf("Addr0 =%x",data0);
    HAL_Delay(5);
    myPrintf(",  Addr1 =%x",data1);
    HAL_Delay(5);

    if ( (data0 != 0x20 ) || (data1 != 0x76) )
    {
        return 0xff;
    }
    if ( data0 == 0x20 )
    {
        myPrintf("wake-up finish.");
        HAL_Delay(5);
    }

    for (i = 0; i < INIT_REG_ARRAY_SIZE; i++)
    {
        paj7620WriteReg(initRegisterArray[i][0], initRegisterArray[i][1]);
    }

    paj7620SelectBank(BANK0);  //gesture flage reg in Bank0

    myPrintf("Paj7620 initialize register finished.");
    HAL_Delay(20);
    return 0;
}


void Gesture_test2(void)
{
    u8 i;
    u8 status;
    u8 key;
    u8 data[2]={0x00};
    u16 gesture_data;
    u8 ledflash=0;

  //  paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
  //  for(i=0;i<GESTURE_SIZE;i++)
  //  {
  //      GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
  //  }
  //  paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
  //  i=0;
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
        //status = GS_Read_nByte(0x43,2,&data[0]);//读取手势状态
        status = paj7620ReadReg(0x43,2,&data[0]);
        if(!status)
        {
            gesture_data =(u16)data[1]<<8 | data[0];
            if(gesture_data)
            {
                switch(gesture_data)
                {
                    case GES_UP:
                        myPrintf("Up\r\n");            ledflash=1;      break; //向上
                    case GES_DOWM:
                        myPrintf("Dowm\r\n");          ledflash=1;      break; //向下
                    case GES_LEFT:
                        myPrintf("Left\r\n");          ledflash=1;      break; //向左
                    case GES_RIGHT:
                        myPrintf("Right\r\n");         ledflash=1;      break; //向右
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

