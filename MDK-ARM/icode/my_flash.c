#include "my_flash.h"

// 保存所有设置（偏移量 + 状态）
void Flash_Save_Settings(float x, float y, float z, uint8_t status)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_SAVE_ADDR;
    EraseInitStruct.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return; 
    }

    uint32_t data_buffer[5]; // 增加到5个字
    data_buffer[0] = FLASH_FLAG_VALUE;
    data_buffer[1] = *((uint32_t*)&x);
    data_buffer[2] = *((uint32_t*)&y);
    data_buffer[3] = *((uint32_t*)&z);
    data_buffer[4] = (uint32_t)status; // 存储状态 (0或1)

    for (int i = 0; i < 5; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SAVE_ADDR + (i * 4), data_buffer[i]) != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();
}

// 读取所有设置
void Flash_Load_Settings(float *x, float *y, float *z, uint8_t *status)
{
    uint32_t flag = *(__IO uint32_t*)(FLASH_SAVE_ADDR);

    if (flag == FLASH_FLAG_VALUE)
    {
        uint32_t temp_x = *(__IO uint32_t*)(FLASH_SAVE_ADDR + 4);
        uint32_t temp_y = *(__IO uint32_t*)(FLASH_SAVE_ADDR + 8);
        uint32_t temp_z = *(__IO uint32_t*)(FLASH_SAVE_ADDR + 12);
        uint32_t temp_s = *(__IO uint32_t*)(FLASH_SAVE_ADDR + 16); // 读取状态

        *x = *((float*)&temp_x);
        *y = *((float*)&temp_y);
        *z = *((float*)&temp_z);
        *status = (uint8_t)temp_s;
    }
    else
    {
        // 默认值：偏移为0，默认开启(1)
        *x = 0.0f;
        *y = 0.0f;
        *z = 0.0f;
        *status = 1; // 默认开启
    }
}