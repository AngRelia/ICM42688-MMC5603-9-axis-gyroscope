#include "Serial.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//usart2
char Serial2_RxPacket[500];
uint8_t Serial2_RxFlag;

// 外部引用 CubeMX 生成的串口句柄
extern UART_HandleTypeDef huart2;

// --- DMA 接收专用变量 ---
#define RX_BUFFER_SIZE 500
uint8_t RxBuffer[RX_BUFFER_SIZE]; // DMA 自动搬运的目标仓库

/*==========================================================
 * 内部辅助函数
 *==========================================================*/

static void Serial2_WaitTxFinish(void)
{
    while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
}

/*==========================================================
 * 初始化函数
 *==========================================================*/
void Serial2_Init(void)
{
    // 开启 DMA 接收，并启用空闲中断 (ReceiveToIdle)
    // 这里的接收是 Circular (循环) 模式
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, RX_BUFFER_SIZE);
}

/*==========================================================
 * 发送函数集 (保持 DMA 发送逻辑)
 *==========================================================*/

void Serial2_SendByte(uint8_t byte)
{
    static uint8_t tempByte; 
    Serial2_WaitTxFinish();
    tempByte = byte;
    HAL_UART_Transmit_DMA(&huart2, &tempByte, 1);
}

void Serial2_SendArray(uint8_t *array, uint16_t length)
{
    Serial2_WaitTxFinish();
    HAL_UART_Transmit_DMA(&huart2, array, length);
}

void Serial2_SendString(char *str)
{
    Serial2_WaitTxFinish();
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, strlen(str));
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t result = 1;
    while (Y--) result *= X;
    return result;
}

void Serial2_SendNumber(uint32_t num, uint8_t length)
{
    static char numBuffer[20];
    if (length > 19) length = 19;
    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t digit = (num / Serial_Pow(10, length - i - 1)) % 10;
        numBuffer[i] = digit + '0';
    }
    numBuffer[length] = '\0';
    Serial2_SendString(numBuffer);
}

void Serial2_Printf(char *format, ...)
{
    static char buffer_Serial2[500]; 
    Serial2_WaitTxFinish();
    va_list args;
    va_start(args, format);
    vsprintf(buffer_Serial2, format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer_Serial2, strlen(buffer_Serial2));
}

/*==========================================================
 * 接收处理逻辑 (核心修改)
 *==========================================================*/

/**
 * @brief 内部函数：处理单个字节的协议解析
 * 将原来 Serial2_RxHandler 的逻辑移到这里
 */
static void Serial2_ProcessByte(uint8_t byte)
{
    static uint8_t RxState2 = 0;
    static uint8_t pRxPacket2 = 0;

    if (RxState2 == 0)
    {
        if (byte == '@' && Serial2_RxFlag == 0)
        {
            RxState2 = 1;
            pRxPacket2 = 0;
        }
    }
    else if (RxState2 == 1)
    {
        if (byte == '\r')
        {
            RxState2 = 2;
        }
        else
        {
            if(pRxPacket2 < 499) 
            {
                Serial2_RxPacket[pRxPacket2++] = byte;
            }
        }
    }
    else if (RxState2 == 2)
    {
        if (byte == '\n')
        {
            RxState2 = 0;
            Serial2_RxPacket[pRxPacket2] = '\0';
            Serial2_RxFlag = 1;
        }
    }
}

/**
 * @brief Serial2 专用的 DMA 接收事件处理函数
 * @note  这个函数由 main.c 中的 HAL_UARTEx_RxEventCallback 调用
 * @param Size: 当前 DMA 缓冲区接收到的数据总量 (由 HAL 库自动计算并传入)
 */
void Serial2_DMA_RxEvent(uint16_t Size)
{
    static uint16_t oldPos = 0; // 上一次处理到的位置

    // 计算新接收的数据长度
    // 情况1: Size > oldPos (正常接收)
    // 情况2: Size < oldPos (发生了环形回绕)
    
    if (Size > oldPos)
    {
        // 处理从 oldPos 到 Size 的数据
        for (int i = oldPos; i < Size; i++)
        {
            Serial2_ProcessByte(RxBuffer[i]);
        }
    }
    else
    {
        // 发生了回绕 (End of buffer reached)
        // 1. 先处理 oldPos 到 Buffer 结尾的数据
        for (int i = oldPos; i < RX_BUFFER_SIZE; i++)
        {
            Serial2_ProcessByte(RxBuffer[i]);
        }
        // 2. 再处理 0 到 Size 的数据
        for (int i = 0; i < Size; i++)
        {
            Serial2_ProcessByte(RxBuffer[i]);
        }
    }
	
    // 更新位置，供下一次中断使用
    oldPos = Size;
}