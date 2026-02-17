#include "key.h"

static uint8_t Key_Num = 0;       // 短按键值缓存
static uint8_t Key_Long_Num = 0;  // 长按键值缓存


static uint8_t Key_GetState(void)
{
    if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_RESET) return 1;
    if (HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == GPIO_PIN_RESET) return 2;
//    if (HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin) == GPIO_PIN_RESET) return 3;
//	if (HAL_GPIO_ReadPin(KEY_4_GPIO_Port, KEY_4_Pin) == GPIO_PIN_RESET) return 4;
    return 0;
}

uint8_t Key_GetNum(void)
{
    uint8_t temp = Key_Num;
    Key_Num = 0;   // 取走后清零，保证一次只响应一次
    return temp;
}

uint8_t Key_GetLongNum(void)
{
    uint8_t temp = Key_Long_Num;
    Key_Long_Num = 0;
    return temp;
}

void Key_Tick(void)
{
    static uint8_t Scan_Cnt = 0;          // 消抖计数器
    static uint16_t Press_Time_Cnt = 0;   // 按下持续时间计数器
    static uint8_t CurrState = 0, PrevState = 0;
    static uint8_t Long_Press_Lock = 0;   // 长按锁定标志位

    Scan_Cnt++;
    if (Scan_Cnt >= 20) // 每20ms进行一次按键逻辑处理
    {
        Scan_Cnt = 0;
        PrevState = CurrState;
        CurrState = Key_GetState();

        // --- 逻辑分支 ---

        if (CurrState != 0) // 1. 检测到按键按下
        {
            if (CurrState == PrevState) // 状态稳定
            {
                Press_Time_Cnt += 20; // 累加时间（因为是20ms进一次）

                // 判断长按 (2000ms)
                if (Press_Time_Cnt >= 2000)
                {
                    if (Long_Press_Lock == 0) // 如果还没触发过长按
                    {
                        Key_Long_Num = CurrState; // 记录长按键值
                        Long_Press_Lock = 1;      // 上锁，防止一直触发
                    }
                }
            }
            else // 或者是按键切换了（比如双指操作），重置计数
            {
                Press_Time_Cnt = 0;
                Long_Press_Lock = 0;
            }
        }
        else // 2. 按键松开 (CurrState == 0)
        {
            // 如果松开前是按下的 (PrevState != 0)
            if (PrevState != 0)
            {
                // 关键点：只有没触发过长按，才算短按
                if (Long_Press_Lock == 0)
                {
                    Key_Num = PrevState; // 触发短按
                }
            }
            
            // 复位所有临时变量
            Press_Time_Cnt = 0;
            Long_Press_Lock = 0;
        }
    }
}