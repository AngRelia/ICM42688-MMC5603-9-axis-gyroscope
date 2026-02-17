#ifndef __KEY_H
#define __KEY_H

#include "main.h"

// 定义按键ID
#define KEY_ID_1   1
//#define KEY_ID_2   2
//#define KEY_ID_3   3
//#define KEY_ID_4   4
uint8_t Key_GetNum(void);     // 获取一次按键值（无按键返回0）
uint8_t Key_GetLongNum(void);    // 获取长按键值
void Key_Tick(void);          // 定时器1ms调用一次

#endif
