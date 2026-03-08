#ifndef __usart_H
#define __usart_H


#if 1
#include "system.h" 
#include "stdio.h"

typedef struct 
{
    u8 rbuf[255];//接收缓冲区
    u8 tbuf[255];//发送缓冲区
    u8 length;//接收数据长度
    u8 flag;   //接收标志位
    /* data */
}UASRT_1;

extern UASRT_1 usart1;

void USART1_Init(u32 bound);
// ·???????×???
void Usart1_SendByte(u8 ch);


// ·???×?·???
void Usart1_SendNBytes(u8 *buff, u8 len);
#endif

#endif


