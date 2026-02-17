#ifndef __MY_FLASH_H
#define __MY_FLASH_H

#include "main.h"

#define FLASH_SAVE_ADDR   0x0800FC00
#define FLASH_FLAG_VALUE  0xA5A5A5A5 

// 修改函数声明，增加 status 参数
void Flash_Save_Settings(float x, float y, float z, uint8_t status);
void Flash_Load_Settings(float *x, float *y, float *z, uint8_t *status);

#endif
