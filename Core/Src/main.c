/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "delay.h" 
#include "Serial.h"
#include "MahonyAHRS.h"
#include "icm42688_driver.h"   
#include <math.h>
#include "key.h"
#include "mmc5603.h"
#include "my_flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 定义全局标志位，volatile 防止编译器优化
volatile uint8_t imu_update_flag = 0;
volatile uint8_t imu_update_count = 0;

float yaw_offset = 0.0f; // 用于记录切换时的偏置
float yaw_display = 0.0f; // 最终用于显示或控制的Yaw值

uint8_t mag_enabled = 1;    // 是否启用磁力计数据参与融合
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Mag_Calibrate(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2; 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Serial2_Init();
  HAL_Delay(100); // 增加延时，确保上电稳定
  
  // 初始化 OLED
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "Init Sensor...", OLED_6X8);
  OLED_Update();

  // 等待传感器ID正确
  while(Init_ICM42688()!=0x47);
  
  // 传感器校准 (注意：上电时保持静止)
  OLED_ShowString(0, 10, "Calib IMU...", OLED_6X8);
  OLED_Update();
  IMU_Calibrate();
  
  // 初始化 MMC5603
  uint8_t mag_status = MMC5603_Init(&hi2c2);

  // 初始化 Mahony
  Mahony_Init(100.0f);
  
  // 初始化校准偏移 (默认为0，后续通过校准更新)
  mmc5603_data.offset_x = 0.0f; 
  mmc5603_data.offset_y = 0.0f;
  mmc5603_data.offset_z = 0.0f;

  // 启动定时器中断
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3); 
  
  // 从Flash加载校准数据和磁力计开启的状态
  Flash_Load_Settings(&mmc5603_data.offset_x, &mmc5603_data.offset_y, &mmc5603_data.offset_z, &mag_enabled);
	if (mag_status == 0) {
	  // 显示状态
	  if(mag_enabled) OLED_ShowString(0, 20, "Mag: ON ", OLED_6X8);
	  else            OLED_ShowString(0, 20, "Mag: OFF", OLED_6X8);
	} else {
	  OLED_ShowString(0, 20, "Mag Err", OLED_6X8);
	}
	OLED_Update();
  HAL_Delay(500); // 让用户看清提示
  OLED_Clear();
  OLED_Update();
  // 可以在串口打印一下看有没有加载成功
  Serial2_Printf("Loaded Offsets: %.3f, %.3f, %.3f\r\n", mmc5603_data.offset_x, mmc5603_data.offset_y, mmc5603_data.offset_z);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t display_count = 0;
  
  // 页面索引
  // 0: Roll/Pitch/Yaw
  // 1: Raw Acc & Gyro (新)
  // 2: Mag Raw (原始数据)
  // 3: Offset Data (校准偏移)
  // 4: Mag Calibrated (修正后数据)
  uint8_t page_index = 0; 

  while (1)
  {
    // 处理短按 - 切换显示页面
    uint8_t key = Key_GetNum();
    if(key == 1)
    {
        page_index++;
        if(page_index > 4) page_index = 0; // 0~4 共5页
        OLED_Clear(); // 切换页面时清屏
    }
    if (key == 2) 
    {
        mag_enabled = !mag_enabled; // 翻转状态
		if (mag_enabled == 0) 
        {
            // 刚切到 6轴模式：把当前的绝对 Yaw 值记录为偏置
            // 这样：显示值 = 当前值 - 偏置 = 0
            yaw_offset = yaw_mahony; 
        }
        else 
        {
            // 刚切回 9轴模式：偏置清零，恢复绝对航向
            yaw_offset = 0.0f; 
        }

        // 保存状态到 Flash (需要同时保存当前的偏移量，否则会覆盖成0)
        Flash_Save_Settings(mmc5603_data.offset_x, mmc5603_data.offset_y, mmc5603_data.offset_z, mag_enabled);

        // UI 提示
        OLED_Clear();
        if(mag_enabled) {
          OLED_ShowString(0, 0, "Mag Enabled", OLED_6X8);
        } else {
          OLED_ShowString(0, 0, "Mag Disabled", OLED_6X8);
        }
        OLED_Update();
        HAL_Delay(1000); // 提示停留1秒
        OLED_Clear();
    }

    // 处理长按 - 触发校准
    uint8_t long_key = Key_GetLongNum();
    if(long_key == 2)
    {
        Serial2_Printf("Start Calibration...\r\n");
        // 暂停定时器中断防止冲突（可选，视中断负载而定）
        HAL_TIM_Base_Stop_IT(&htim2); 
        Mag_Calibrate(); 
        HAL_TIM_Base_Start_IT(&htim2);
        
        imu_update_count = 0; // 清除堆积的计数
        OLED_Clear(); // 校准完清屏
    }
    
  if (mag_enabled == 1)
      {
          // 只有开启时，才发送 I2C 命令读取
          MMC5603_ReadData_Single(&hi2c2);

          // 计算校准值
          mmc5603_data.calib_x = mmc5603_data.mag_x_gauss - mmc5603_data.offset_x;
          mmc5603_data.calib_y = mmc5603_data.mag_y_gauss - mmc5603_data.offset_y;
          mmc5603_data.calib_z = mmc5603_data.mag_z_gauss - mmc5603_data.offset_z;
      }
    
    // IMU 解算循环
    while (imu_update_count > 0)
    {
        imu_update_count--; 
        Get_Acc_ICM42688();
        Get_Gyro_ICM42688();
        
        // 根据开关选择算法
        if (mag_enabled == 1)
        {
            // 9轴融合 (使用 MMC5603)
            // 注意：这里需要你把被注释掉的 Mahony_update 函数取消注释
            Mahony_update(
                icm42688_gyro_x * 0.0174533f, 
                icm42688_gyro_y * 0.0174533f, 
                icm42688_gyro_z * 0.0174533f,
                icm42688_acc_x, icm42688_acc_y, icm42688_acc_z,
                -mmc5603_data.calib_y, 
                mmc5603_data.calib_x, 
                mmc5603_data.calib_z
            );
        }
        else
        {
            // 6轴融合 (只用陀螺仪和加速度计)
            MahonyAHRSupdateIMU(
                icm42688_gyro_x * 0.0174533f, 
                icm42688_gyro_y * 0.0174533f, 
                icm42688_gyro_z * 0.0174533f,
                icm42688_acc_x, icm42688_acc_y, icm42688_acc_z
            );
        }
        Mahony_computeAngles();
        display_count++;
    }
    yaw_display = yaw_mahony - yaw_offset;
	
	// 处理角度回绕问题 (确保在 -180 到 +180 之间)
    if (yaw_display > 180.0f)  yaw_display -= 360.0f;
    if (yaw_display < -180.0f) yaw_display += 360.0f;
	
    // 刷新显示和串口发送 (约 10Hz)
    if (display_count >= 10) 
    {
        display_count = 0;
        
        // --- OLED 显示部分 (根据页面切换) ---
        if (page_index == 0) 
        {
            // Page 0: 姿态角 (RPY)
            if(mag_enabled){
                OLED_ShowString(0, 0, "--- Attitude (9-Axis) ---", OLED_6X8);
            } else {
                OLED_ShowString(0, 0, "--- Attitude (6-Axis) ---", OLED_6X8);
            }
            OLED_ShowString(0, 16, "Roll:", OLED_6X8);
            OLED_ShowFloatNum(40, 16, roll_mahony, 3, 2, OLED_6X8);
            
            OLED_ShowString(0, 32, "Pitch:", OLED_6X8);
            OLED_ShowFloatNum(40, 32, pitch_mahony, 3, 2, OLED_6X8);
            
            OLED_ShowString(0, 48, "Yaw:", OLED_6X8);
            OLED_ShowFloatNum(40, 48, yaw_display, 3, 2, OLED_6X8);
        }
        else if (page_index == 1)
        {
            // Page 1: 原始 Acc & Gyro
            OLED_ShowString(0, 0, "--- Raw IMU ---", OLED_6X8);
            OLED_ShowString(0, 8, "ax:", OLED_6X8);  OLED_ShowFloatNum(20, 8, icm42688_acc_x, 2, 2, OLED_6X8);
            OLED_ShowString(0, 16, "ay:", OLED_6X8); OLED_ShowFloatNum(20, 16, icm42688_acc_y, 2, 2, OLED_6X8);
            OLED_ShowString(0, 24, "az:", OLED_6X8); OLED_ShowFloatNum(20, 24, icm42688_acc_z, 2, 2, OLED_6X8);
            
            OLED_ShowString(0, 32, "gx:", OLED_6X8); OLED_ShowFloatNum(20, 32, icm42688_gyro_x, 2, 1, OLED_6X8);
            OLED_ShowString(0, 40, "gy:", OLED_6X8); OLED_ShowFloatNum(20, 40, icm42688_gyro_y, 2, 1, OLED_6X8);
            OLED_ShowString(0, 48, "gz:", OLED_6X8); OLED_ShowFloatNum(20, 48, icm42688_gyro_z, 2, 1, OLED_6X8);
        }
        else if (page_index == 2)
        {
            // Page 2: 原始磁力计数据 (Mag Raw)
            OLED_ShowString(0, 0, "--- Mag Raw ---", OLED_6X8);
            OLED_ShowString(0, 16, "MX:", OLED_6X8);
            OLED_ShowFloatNum(24, 16, mmc5603_data.mag_x_gauss, 3, 3, OLED_6X8);
            
            OLED_ShowString(0, 32, "MY:", OLED_6X8);
            OLED_ShowFloatNum(24, 32, mmc5603_data.mag_y_gauss, 3, 3, OLED_6X8);
            
            OLED_ShowString(0, 48, "MZ:", OLED_6X8);
            OLED_ShowFloatNum(24, 48, mmc5603_data.mag_z_gauss, 3, 3, OLED_6X8);
        }
        else if (page_index == 3)
        {
            // Page 3: 校准偏移量 (Offset)
            OLED_ShowString(0, 0, "--- Mag Offset ---", OLED_6X8);
            OLED_ShowString(0, 16, "OX:", OLED_6X8);
            OLED_ShowFloatNum(24, 16, mmc5603_data.offset_x, 3, 3, OLED_6X8);
            
            OLED_ShowString(0, 32, "OY:", OLED_6X8);
            OLED_ShowFloatNum(24, 32, mmc5603_data.offset_y, 3, 3, OLED_6X8);
            
            OLED_ShowString(0, 48, "OZ:", OLED_6X8);
            OLED_ShowFloatNum(24, 48, mmc5603_data.offset_z, 3, 3, OLED_6X8);
        }
        else 
        {
            // Page 4: 修正后的磁力计数据 (Corrected)
            OLED_ShowString(0, 0, "--- Mag Calib ---", OLED_6X8);
            OLED_ShowString(0, 16, "CX:", OLED_6X8);
            OLED_ShowFloatNum(24, 16, mmc5603_data.calib_x, 3, 3, OLED_6X8);
            
            OLED_ShowString(0, 32, "CY:", OLED_6X8);
            OLED_ShowFloatNum(24, 32, mmc5603_data.calib_y, 3, 3, OLED_6X8);
            
            OLED_ShowString(0, 48, "CZ:", OLED_6X8);
            OLED_ShowFloatNum(24, 48, mmc5603_data.calib_z, 3, 3, OLED_6X8);
        }
        OLED_Update();

        // --- 串口发送部分 ---
        // 打印全部传感器原始数据，用于上位机波形观察
//        Serial2_Printf("Acc:%.3f,%.3f,%.3f,Gyro:%.3f,%.3f,%.3f,Mag:%.3f,%.3f,%.3f\r\n", 
//            icm42688_acc_x, 
//            icm42688_acc_y,
//            icm42688_acc_z,
//            icm42688_gyro_x,
//            icm42688_gyro_y, 
//            icm42688_gyro_z,
//            mmc5603_data.mag_x_gauss,
//            mmc5603_data.mag_y_gauss,
//            mmc5603_data.mag_z_gauss
//        );
		// Serial2_Printf("raw:%.3f,%.3f,%.3f,offset:%.3f,%.3f,%.3f,cal:%.3f,%.3f,%.3f\r\n", 
        //     mmc5603_data.mag_x_gauss,
        //     mmc5603_data.mag_y_gauss,
        //     mmc5603_data.mag_z_gauss,
        //     mmc5603_data.offset_x,
        //     mmc5603_data.offset_y,
        //     mmc5603_data.offset_z,
        //     mmc5603_data.calib_x,
        //     mmc5603_data.calib_y,
        //     mmc5603_data.calib_z
        // );
        Serial2_Printf("roll:%.2f,pitch:%.2f,yaw:%.2f\r\n", 
            roll_mahony, 
            pitch_mahony, 
            yaw_display
        );
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 定时器中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        imu_update_count++; 
    }
    else if (htim->Instance == TIM3)
    {
        Key_Tick(); 
    }
}

// 磁力计校准函数
void Mag_Calibrate(void)
{
    float x_min = 1000.0f, x_max = -1000.0f;
    float y_min = 1000.0f, y_max = -1000.0f;
    float z_min = 1000.0f, z_max = -1000.0f;
    uint16_t sample_count = 0;
    
    // 给串口一点时间缓冲
    HAL_Delay(50);
    Serial2_Printf("Start Mag Calibration...\r\n");
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Mag Calibrating", OLED_6X8);
    OLED_ShowString(0, 16, "Rotate 8 shape!", OLED_6X8);
    OLED_Update();

    // 采集数据约 15 秒 
    for(int i = 0; i < 1500; i++) 
    {
        MMC5603_ReadData_Single(&hi2c2); 
        
        // 更新极值
        if(mmc5603_data.mag_x_gauss < x_min) x_min = mmc5603_data.mag_x_gauss;
        if(mmc5603_data.mag_x_gauss > x_max) x_max = mmc5603_data.mag_x_gauss;
        
        if(mmc5603_data.mag_y_gauss < y_min) y_min = mmc5603_data.mag_y_gauss;
        if(mmc5603_data.mag_y_gauss > y_max) y_max = mmc5603_data.mag_y_gauss;
        
        if(mmc5603_data.mag_z_gauss < z_min) z_min = mmc5603_data.mag_z_gauss;
        if(mmc5603_data.mag_z_gauss > z_max) z_max = mmc5603_data.mag_z_gauss;
        
        sample_count++;
        // 降低串口打印频率，避免阻塞，每200ms打印一次
        if(sample_count % 20 == 0) {
            // 只更新OLED，不一定要打印串口
            OLED_ShowNum(0, 32, (1500-i)/100, 2, OLED_6X8); 
            OLED_Update();
        }
        
        HAL_Delay(10);
    }
    
    // 计算中心偏移量
    mmc5603_data.offset_x = (x_min + x_max) / 2.0f;
    mmc5603_data.offset_y = (y_min + y_max) / 2.0f;
    mmc5603_data.offset_z = (z_min + z_max) / 2.0f;
    
    Serial2_Printf("Saving to Flash...\r\n");
    Flash_Save_Settings(mmc5603_data.offset_x, mmc5603_data.offset_y, mmc5603_data.offset_z, mag_enabled);
    Serial2_Printf("Save Done!\r\n");
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Calib Done", OLED_6X8);
    OLED_ShowString(0, 16, "Saved to Flash", OLED_6X8); // 提示已保存
    OLED_Update();
    
    // 一次性打印结果
    HAL_Delay(100); 
    Serial2_Printf("Mag Calibration Done!\r\n");
    HAL_Delay(10);
    Serial2_Printf("Offset X: %.4f\r\n", mmc5603_data.offset_x);
    HAL_Delay(10);
    Serial2_Printf("Offset Y: %.4f\r\n", mmc5603_data.offset_y);
    HAL_Delay(10);
    Serial2_Printf("Offset Z: %.4f\r\n", mmc5603_data.offset_z);
    
    HAL_Delay(2000);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
