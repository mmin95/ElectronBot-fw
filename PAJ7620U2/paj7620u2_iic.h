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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ������STM32F429������
//PAJ7620U2 IIC��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////


#define GS_SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB11����ģʽ
#define GS_SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;}    //PB11���ģʽ

//IO��������
#define GS_IIC_SCL    PBout(10) 		//SCL
#define GS_IIC_SDA    PBout(11) 		//SDA
#define GS_READ_SDA   PBin(11) 		    //����SDA


#define iicMaxTry 5
u8 GS_Write_Byte(u8 REG_Address,u8 REG_data);
u8 GS_Read_Byte(u8 REG_Address);
u8 GS_Read_nByte(u8 REG_Address,u16 len,u8 *buf);
void GS_i2c_init(void);
void GS_WakeUp(void);

#endif //CPROJECT_PAJ7620U2_IIC_H
