//
// Created by Administrator on 2022/8/13.
//
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "flash.h"
#include <cmath>
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

#ifndef CPROJECT_MYPRINTF_H
#define CPROJECT_MYPRINTF_H

void myPrintf(const char* format, ...);
void myDebug(const char* format, ...);


#endif //CPROJECT_MYPRINTF_H
