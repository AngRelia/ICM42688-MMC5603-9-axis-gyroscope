#include "mmc5603.h"
#include "stdio.h"  // 用于调试打印 (如果需要)

MMC5603_Data_t mmc5603_data;

// 辅助写函数，带错误检查
static uint8_t MMC5603_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    if (HAL_I2C_Mem_Write(hi2c, MMC5603_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK) {
        return 1; // 写入失败
    }
    return 0;
}

// 检查传感器ID
uint8_t MMC5603_CheckID(I2C_HandleTypeDef *hi2c) {
    uint8_t id = 0;
    // 读取 Product ID (寄存器 0x39)
    if (HAL_I2C_Mem_Read(hi2c, MMC5603_ADDR, MMC5603_PRODUCT_ID, I2C_MEMADD_SIZE_8BIT, &id, 1, 100) != HAL_OK) {
        return 0xFF; // I2C 通信失败
    }
    // MMC5603 的 ID 通常是 0x10 (根据规格书 Bit4=1)，但也可能是其他值
    // 这里我们只要能读到非0、非FF的值，就认为连接正常
    return id; 
}

// 初始化
// 返回值: 0=成功, 1=I2C写失败, 0xFF=ID读取失败
uint8_t MMC5603_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id = MMC5603_CheckID(hi2c);
    
    // 如果读不到ID，或者ID是全0/全1，说明硬件连接有问题
    if (id == 0xFF || id == 0x00) return 0xFF; 

    // 1. 复位 (向 CTRL1 写入 0x80)
    if (MMC5603_WriteReg(hi2c, MMC5603_CTRL1, 0x80) != 0) return 1;
    HAL_Delay(20);

    // 2. 配置 Auto Set/Reset (CTRL0: Bit 5 = 1)
    // 开启自动Set/Reset可以消除磁偏置，强烈建议开启
    // 写入 0x20 (Auto_SR_en)
    MMC5603_WriteReg(hi2c, MMC5603_CTRL0, 0x20);

    // 3. 设置带宽/采样率 (ODR) - 单次模式下这个主要影响内部滤波
    MMC5603_WriteReg(hi2c, MMC5603_ODR, 100);

    return 0; // 初始化成功
}

// 单次测量模式读取 (Polling)
void MMC5603_ReadData_Single(I2C_HandleTypeDef *hi2c) {
    uint8_t raw_data[9];
    uint32_t x_raw, y_raw, z_raw;
    
    // 1. 发送测量命令 (TM_M)
    // 向 CTRL0 (0x1B) 写入 0x01 (Take_meas_M) | 0x20 (保持 Auto_SR_en)
    if (MMC5603_WriteReg(hi2c, MMC5603_CTRL0, 0x01 | 0x20) != 0) return;
    
    // 2. 等待测量完成
    // 规格书显示 BW=00 时测量时间约 6.6ms，BW=11 时 1.2ms
    // 我们保险起见延时 10ms
    // (在实际项目中，如果不允许阻塞，可以用定时器或轮询 Status 寄存器的 Meas_m_done 位)
    HAL_Delay(10); 

    // 3. 读取数据
    // 读取 0x00 开始的 9 个字节
    if (HAL_I2C_Mem_Read(hi2c, MMC5603_ADDR, MMC5603_XOUT0, I2C_MEMADD_SIZE_8BIT, raw_data, 9, 10) == HAL_OK) {
        
        // 拼接 20-bit 数据
        x_raw = ((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | ((uint32_t)raw_data[6] >> 4);
        y_raw = ((uint32_t)raw_data[2] << 12) | ((uint32_t)raw_data[3] << 4) | ((uint32_t)raw_data[7] >> 4);
        z_raw = ((uint32_t)raw_data[4] << 12) | ((uint32_t)raw_data[5] << 4) | ((uint32_t)raw_data[8] >> 4);
        
        // 转换为 Gauss
        mmc5603_data.mag_x_gauss = ((float)x_raw - MMC5603_NULL_POINT) / MMC5603_SENSITIVITY;
        mmc5603_data.mag_y_gauss = ((float)y_raw - MMC5603_NULL_POINT) / MMC5603_SENSITIVITY;
        mmc5603_data.mag_z_gauss = ((float)z_raw - MMC5603_NULL_POINT) / MMC5603_SENSITIVITY;
    } else {
        // 读取失败，可以在这里打印错误或者标记数据无效
        // 为了调试，我们可以故意设置成一个特殊值，看是否进入了这里
        // mmc5603_data.mag_x_gauss = -999.0f; 
    }
}