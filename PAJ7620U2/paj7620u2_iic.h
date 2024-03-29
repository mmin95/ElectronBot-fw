//
// Created by Administrator on 2022/8/8.
//

#ifndef CPROJECT_PAJ7620U2_IIC_H
#define CPROJECT_PAJ7620U2_IIC_H

#include "i2c.h"


typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
extern I2C_HandleTypeDef hi2c1;
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 阿波罗STM32F429开发板
//PAJ7620U2 IIC驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////


#define GS_SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB11输入模式
#define GS_SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;}    //PB11输出模式

//IO操作函数
#define GS_IIC_SCL    PBout(10) 		//SCL
#define GS_IIC_SDA    PBout(11) 		//SDA
#define GS_READ_SDA   PBin(11) 		    //输入SDA


#define iicMaxTry 5
u8 GS_Write_Byte(u8 REG_Address,u8 REG_data);
u8 GS_Read_Byte(u8 REG_Address);
u8 GS_Read_nByte(u8 REG_Address,u16 len,u8 *buf);
void GS_i2c_init(void);
void GS_WakeUp(void);

#endif //CPROJECT_PAJ7620U2_IIC_H
