#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"

/*--------USART2 相关变量---------*/
extern char Serial2_RxPacket[500]; // 数据包缓冲区
extern uint8_t Serial2_RxFlag;     // 数据包接收完成标志

/*--------USART2 函数---------*/
void Serial2_Init(void); // 新增：初始化接收 DMA

// 新增：专门处理 DMA 接收事件的函数，供 main.c 回调调用
// 注意：这个 Size 是 HAL 库在中断里算好传给你的
void Serial2_DMA_RxEvent(uint16_t Size);

void Serial2_SendByte(uint8_t byte);
void Serial2_SendArray(uint8_t *array, uint16_t length);
void Serial2_SendString(char *str);
void Serial2_SendNumber(uint32_t num, uint8_t length);
void Serial2_Printf(char *format, ...);

#endif