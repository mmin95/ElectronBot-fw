//
// Created by Administrator on 2022/8/13.
//

#include "myPrintf.h"

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