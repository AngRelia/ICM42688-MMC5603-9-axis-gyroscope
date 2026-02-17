/****************************************************************************************
 * @file       icm42688_driver.c
 * @brief      ICM42688 6轴驱动 (基于参考代码移植，适配STM32 HAL)
 * @author     移植优化版
 * @MCUcore    STM32F103C8T6
 * @date       2025-02-13
****************************************************************************************/

#include "icm42688_driver.h"
#include "spi.h" // 确保包含 spi.h 以获取 hspi1 句柄

// 引用 SPI1 句柄
extern SPI_HandleTypeDef hspi1;

// =================================================================================
// 全局变量定义
// =================================================================================
float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z;       // 单位: g
float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z;    // 单位: dps (°/s)
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;  // 陀螺仪零偏 (dps)
uint8_t icm42688_id = 0; // 存储读取到的芯片ID

// 灵敏度系数 (硬编码 16G, 2000dps)
// Acc: 16G / 32768
static const float ACC_SENSITIVITY = 16.0f / 32768.0f;
// Gyro: 2000dps / 32768
static const float GYRO_SENSITIVITY = 2000.0f / 32768.0f;

float acc_offset_x = 0.00f; // 填入你平放时读到的 AccX
float acc_offset_y = 0.00f; // 填入你平放时读到的 AccY
float acc_offset_z = 0.00f; // 填入 (平放时读到的 AccZ - 1.0f)

// =================================================================================
// 静态底层函数声明
// =================================================================================
static uint8_t ICM_SPI_ReadWriteByte(uint8_t tx_data);
static void ICM_WriteReg(uint8_t reg, uint8_t value);
static void ICM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);

// =================================================================================
// 底层 SPI 实现 (参考代码风格，但使用 HAL 库优化)
// =================================================================================

/**
 * @brief SPI 单字节读写 (标准全双工)
 */
static uint8_t ICM_SPI_ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data = 0;
    // 使用 HAL_SPI_TransmitReceive 保证时钟连续
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 10);
    return rx_data;
}

/**
 * @brief 写寄存器
 */
static void ICM_WriteReg(uint8_t reg, uint8_t value)
{
    ICM42688_CS_LOW();
    ICM_SPI_ReadWriteByte(reg & 0x7F); // 写操作，最高位为0
    ICM_SPI_ReadWriteByte(value);
    ICM42688_CS_HIGH();
}

/**
 * @brief 连续读寄存器
 */
static void ICM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM42688_CS_LOW();
    ICM_SPI_ReadWriteByte(reg | 0x80); // 读操作，最高位为1
    for (uint16_t i = 0; i < len; i++)
    {
        buf[i] = ICM_SPI_ReadWriteByte(0xFF); // 发送空字节以读取数据
    }
    ICM42688_CS_HIGH();
}

// =================================================================================
// 用户 API 函数
// =================================================================================

/**
 * @brief ICM42688 初始化 (包含复位、AAF抗混叠配置、低噪声模式)
 */
uint8_t Init_ICM42688(void)
{
    // 1. 上电等待
    ICM42688_CS_HIGH();
    HAL_Delay(50);

    // 2. 切换 Bank0 并软复位
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    ICM_WriteReg(ICM42688_DEVICE_CONFIG, 0x01); // Soft Reset
    HAL_Delay(50); // 等待复位完成

    // 3. 检查 ID (WHO_AM_I)
    uint8_t retry = 10;
    while (retry--)
    {
        ICM_ReadRegs(ICM42688_WHO_AM_I, &icm42688_id, 1);
        if (icm42688_id == 0x47) break;
        HAL_Delay(10);
    }
    
    // // 如果ID不对，可以在这里添加错误处理，目前直接返回
    // if (icm42688_id != 0x47) return;

    // --------------------------------------------------------
    // 开始配置 (参考代码流程)
    // --------------------------------------------------------

    // 4. 基础配置 (Bank 0)
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    
    // Gyro: 2000dps, 1kHz (ODR)
    ICM_WriteReg(ICM42688_GYRO_CONFIG0, 0x06); 
    // Accel: 16G, 1kHz (ODR)
    ICM_WriteReg(ICM42688_ACCEL_CONFIG0, 0x06);
    
    // 开启电源 (Low Noise Mode: Gyro+Accel)
    ICM_WriteReg(ICM42688_PWR_MGMT0, 0x0F);
    HAL_Delay(30);

    // 5. 详细滤波器参数 (Bank 0)
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    // Gyro Config1: BW=82Hz, Latency=2ms (初始)
    ICM_WriteReg(ICM42688_GYRO_CONFIG1, 0x56);
    // Gyro/Accel Config: 1BW
    ICM_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0, 0x11);
    // Accel Config1: Null
    ICM_WriteReg(ICM42688_ACCEL_CONFIG1, 0x0D);

    // 6. 中断源配置 (Bank 0) - 虽然不一定用中断引脚，但配置上无妨
    ICM_WriteReg(ICM42688_INT_CONFIG0, 0x00);
    ICM_WriteReg(ICM42688_INT_SOURCE0, 0x08); // DRDY INT1

    // --------------------------------------------------------
    // 7. 配置抗混叠滤波器 (AAF) @ 536Hz (关键步骤)
    // --------------------------------------------------------
    
    // --- Gyro AAF (Bank 1) ---
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x01); // 切换 Bank 1
    ICM_WriteReg(0x0B, 0xA0); // Enable AAF & Notch
    ICM_WriteReg(0x0C, 0x0C); // DELT
    ICM_WriteReg(0x0D, 0x90); // DELTSQR
    ICM_WriteReg(0x0E, 0x80); // BITSHIFT

    // --- Accel AAF (Bank 2) ---
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x02); // 切换 Bank 2
    ICM_WriteReg(0x03, 0x18); // Enable Filter
    ICM_WriteReg(0x04, 0x90); // DELTSQR
    ICM_WriteReg(0x05, 0x80); // BITSHIFT

    // --------------------------------------------------------
    // 8. 自定义低通滤波器 @ 50Hz (增强滤波效果)
    // --------------------------------------------------------
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00); // 切回 Bank 0
    
    // 滤波器阶数 (3rd Order)
    ICM_WriteReg(ICM42688_GYRO_CONFIG1, 0x1A);
    ICM_WriteReg(ICM42688_ACCEL_CONFIG1, 0x15);
    // 带宽设置 50Hz (ODR/20)
    ICM_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0, 0x66);

    // 9. 最终确保电源开启
    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    ICM_WriteReg(ICM42688_PWR_MGMT0, 0x0F);
    
    HAL_Delay(50); // 准备就绪
    return icm42688_id;
}

void IMU_Calibrate(void)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 500;
    
    // 提示用户保持静止
    // OLED_ShowString("Calibrating..."); 

    for(int i=0; i<samples; i++)
    {
        Get_Gyro_ICM42688();
        sum_x += icm42688_gyro_x;
        sum_y += icm42688_gyro_y;
        sum_z += icm42688_gyro_z;
        HAL_Delay(2);
    }
    
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;
}

/**
 * @brief 读取加速度数据 (结果存入全局变量 icm42688_acc_x/y/z)
 */
void Get_Acc_ICM42688(void)
{
    uint8_t buf[6];
    
    // 一次性读取 6 个字节 (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
    // 地址从 ICM42688_ACCEL_DATA_X1 (0x1F) 开始连续
    ICM_ReadRegs(ICM42688_ACCEL_DATA_X1, buf, 6);

    // 拼接数据
    icm42688_acc_x = (int16_t)((buf[0] << 8) | buf[1]) * ACC_SENSITIVITY;
    icm42688_acc_y = (int16_t)((buf[2] << 8) | buf[3]) * ACC_SENSITIVITY;
    icm42688_acc_z = (int16_t)((buf[4] << 8) | buf[5]) * ACC_SENSITIVITY;
	
	icm42688_acc_x -= acc_offset_x; 
    icm42688_acc_y -= acc_offset_y;
    icm42688_acc_z -= acc_offset_z;
}

/**
 * @brief 读取角速度数据 (结果存入全局变量 icm42688_gyro_x/y/z)
 */
void Get_Gyro_ICM42688(void)
{
    uint8_t buf[6];

    // 一次性读取 6 个字节
    // 地址从 ICM42688_GYRO_DATA_X1 (0x25) 开始连续
    ICM_ReadRegs(ICM42688_GYRO_DATA_X1, buf, 6);

    // 拼接数据
    icm42688_gyro_x = (int16_t)((buf[0] << 8) | buf[1]) * GYRO_SENSITIVITY;
    icm42688_gyro_y = (int16_t)((buf[2] << 8) | buf[3]) * GYRO_SENSITIVITY;
    icm42688_gyro_z = (int16_t)((buf[4] << 8) | buf[5]) * GYRO_SENSITIVITY;

    icm42688_gyro_x -= gyro_offset_x;
    icm42688_gyro_y -= gyro_offset_y;
    icm42688_gyro_z -= gyro_offset_z;
}
