#include "delay.h"

void delay_us(uint32_t us) //利用CPU循环实现的非精准应用的微秒延时函数
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 8000000 * us); //使用HAL_RCC_GetHCLKFreq()函数获取主频值，经算法得到1微秒的循环次数
    while (delay--); //循环delay次，达到1微秒延时
}
