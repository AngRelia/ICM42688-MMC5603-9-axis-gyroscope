/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_2_Pin GPIO_PIN_0
#define KEY_2_GPIO_Port GPIOA
#define KEY_1_Pin GPIO_PIN_1
#define KEY_1_GPIO_Port GPIOA
#define ICM_CS_Pin GPIO_PIN_4
#define ICM_CS_GPIO_Port GPIOA
/* 【注意：本板 PCB 把 OLED 的 SCL/SDA 走线画反了，此处刻意为反接写法】
 *      PB8 = OLED_SDA、PB9 = OLED_SCL      （与常规接法相反）
 *   这是为了适配实物 PCB 的故意写法：改回常规顺序（PB8=SCL）会导致屏幕不亮。
 *   三处保持一致：本处宏、工程 .ioc 的 PB8/PB9.GPIO_Label、以及 OLED.c 的使用。
 *   下方 USER CODE BEGIN Private defines 里另存一份说明（防 CubeMX 重新生成时丢失）。 */
#define OLED_SDA_Pin GPIO_PIN_8
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* 供 icode/proto.c（IMU 数据帧协议）读取的全局状态标志。
   定义都在 Core/Src/main.c，类型必须与定义严格一致（含 volatile）。
   说明：2026-10-16 本文件曾被工具误截断，从 git 恢复 v1.2 时丢了这段未提交的声明，
   由编译错误（proto.c 的 fusion_ready / mag_enabled undefined）定位后补回。 */
extern volatile uint8_t fusion_ready;      /* 姿态解算已就绪 */
extern uint8_t          mag_enabled;       /* 是否启用磁力计（九轴/六轴） */
extern volatile uint8_t gyro_az_done;      /* 陀螺 Z 轴零偏学习完成 */
extern volatile uint8_t heading_settling;  /* 航向收敛中 */
extern uint8_t          mag_ref_valid;     /* 磁力计参考有效 */
/* 【防误改】本板 OLED 的 SCL/SDA 是反的：PB8=OLED_SDA、PB9=OLED_SCL 属故意适配，
 *   与上方引脚宏、工程 .ioc 的 PB8/PB9.GPIO_Label 三处一致。
 *   若"纠正"成 PB8=SCL/PB9=SDA，OLED 将不显示。OLED.c 直接使用上面两个宏。 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
