#ifndef __MMC5603_H
#define __MMC5603_H

#include "main.h"

// MMC5603 I2C 地址 (7-bit 0x30)
// HAL库通常使用 8-bit 地址 (左移一位)
// 写地址: 0x60, 读地址: 0x61 (HAL库会自动处理读写位，建议统一用 0x60)
#define MMC5603_ADDR          (0x30 << 1) 

// 寄存器定义
#define MMC5603_XOUT0         0x00
#define MMC5603_ODR           0x1A
#define MMC5603_CTRL0         0x1B
#define MMC5603_CTRL1         0x1C
#define MMC5603_CTRL2         0x1D
#define MMC5603_PRODUCT_ID    0x39

// 灵敏度 (20-bit mode)
#define MMC5603_SENSITIVITY   16384.0f
#define MMC5603_NULL_POINT    524288.0f

// mmc5603.h

typedef struct {
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
    // 新增：校准偏移量
    float offset_x;
    float offset_y;
    float offset_z;
	//修正后结果
	float calib_x;
	float calib_y;
	float calib_z;
} MMC5603_Data_t;

extern MMC5603_Data_t mmc5603_data;

uint8_t MMC5603_CheckID(I2C_HandleTypeDef *hi2c);
void MMC5603_ReadData_Single(I2C_HandleTypeDef *hi2c); // 改用单次读取

#endif