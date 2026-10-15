/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention v1.2 区分了6/9轴模式的显示和Kp设置.
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
#include "debug_log.h"
#include "proto.h"      /* 串口帧协议: 数据帧走这里, 非阻塞 DMA */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===================== 调参区 (只改这里) =====================
 * Mahony 增益 twoKp: 修正强度, 时间常数约 1/twoKp 秒。
 *   6 轴: 加速度计只约束 Roll/Pitch, 1.5 比较稳。
 *   9 轴: 磁力计约束 Yaw。
 *         太大 -> Yaw 紧跟磁力计, 磁干扰/标定误差会 1:1 灌进 Yaw;
 *         太小 -> 陀螺积分主导, Yaw 缓慢漂移。
 *         2.0 是保守起点, 调试时建议每次加减 0.5。
 * 两个值都必须 > 0。
 * ============================================================ */
#define MAHONY_TWOKP_6AXIS    1.5f
#define MAHONY_TWOKP_9AXIS    2.0f
/* 上电收敛期专用增益 (只在静止且尚未完成首次零偏学习时使用)。
 * 目的: 让上电初值误差尽快收敛, 而不是慢慢飘过去。
 *   时间常数 = 1/twoKp。仿真(200Hz, 初值误差90度, 收敛到1度):
 *     twoKp=2.0 -> 2.23 s
 *     twoKp=6.0 -> 0.74 s
 *     twoKp=15  -> 0.29 s
 * 之所以敢在上电时用大增益: 只对静止段生效, 且只持续到 auto-zero 首次学完
 * (约 1 秒)。稳定后自动切回 MAHONY_TWOKP_9AXIS, 抗磁干扰能力不受影响。
 */
#define MAHONY_TWOKP_9AXIS_BOOT   6.0f

/* ============ 航向方向约定 (与陀螺符号绑定, 改必须成对改!) ============
 * 实测: 板子【顺时针】(往下看)转时, icm42688_gyro_z 读【负数】,
 *       而原来的磁力计重映射 (-my, +mx) 让 yaw 【逆时针增大】。
 *       两者一致地反着, 所以能稳定锁定, 但约定与标准罗盘(顺时针增大)相反。
 *
 * HZ_SIGN 与 MAG_SWAP 必须【成对】设置, 否则陀螺与磁力计旋向相反,
 * 误差项会互相抵消不了, yaw 会震荡或被拉飞。
 *
 *   0 = 旧约定: 陀螺 +gz, 磁力计 (-my, +mx)   -> 顺时针【减小】
 *   1 = 标准罗盘: 陀螺 -gz, 磁力计 (+my, -mx) -> 顺时针【增大】 (当前)
 *
 * 注意: 受 MMC5603 的 90 度安装角限制, 两种约定的 yaw=0 都不在正北,
 *       而是偏 +/-90 度。若需要 yaw=0 对齐磁北, 在显示处叠加一个常量偏移即可
 * ==================================================================== */
#define HEADING_STD_CONVENTION   1

/* 六轴专用的陀螺 Z 符号 (与九轴解耦)。
 *
 * 为什么要单独一个宏:
 *   六轴下 yaw 没有绝对基准, 完全是 陀螺 Z 的积分, 起点由 MahonyAHRSinit
 *   决定(六轴时磁力计传 0, 故 init_yaw = 0)。因此【旋向】纯属约定,
 *   没有对错, 只影响【顺时针为正】还是【逆时针为正】。
 *   而九轴的旋向是实测确认过的(顺时针增大), 不该被六轴的偏好牵连,
 *   所以这里独立出来。
 *
 *   +1 = 原始方向: 顺时针读数减小
 *   -1 = 反向:     顺时针读数增大
 *
 * 当前取 +1 (用户习惯的原始行为)。
 */
#define MAHONY_GZ_SIGN_6AXIS    (+1.0f)

/* 上电把 yaw 归零 (航向相对起点)。
 *
 * 为什么合理: 六轴下 yaw 没有绝对基准, 完全由陀螺 Z 积分而来,
 * 起点由 MahonyAHRSinit 决定 —— 数学上就是 0。但实际会有两个来源让它偏离 0:
 *   1) 上电那一秒 IMU_Calibrate 期间板子若被碰过, 零偏估计会带出一个偏置;
 *   2) 上电后陀螺残余零偏持续积分。
 * 把上电瞬间的 yaw 记为基准并减掉, 读数就从 0 开始, 符合【相对航向】的物理意义。
 *
 * 只影响显示(yaw_display), 不参与融合, 所以不会改变任何动力学。
 * 九轴下不建议用它 —— 九轴 yaw 是磁北绝对方位, 归零会破坏这个语义。
 */
#define YAW_ZERO_ON_BOOT          1

/* 航向零点偏移 (度)。
 * 受 MMC5603 的 90 度安装角限制, yaw 的 0 点不在磁北。
 * 标定方法: 把板子放平指向你认定的磁北, 读出当时的 yaw, 取相反数填这里。
 *   例如指向磁北时显示 -179, 就填 179.0f。
 * 只影响显示, 不影响融合内部的角度。 */
#define HEADING_ZERO_DEG       179.0f



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 定义全局标志位，volatile 防止编译器优化
volatile uint8_t imu_update_flag = 0;
/* 融合使能: 上电校准与 MahonyAHRSinit 完成前保持 0,
   避免中断在四元数/零偏就绪之前就开始融合。 */
volatile uint8_t fusion_ready = 0;

uint8_t mag_enabled = 0;    // 是否启用磁力计 (真值由 Flash 决定; 这里给 0 与默认六轴一致)

/* 静止自动零偏修正: 0=尚未学习到, 1=已学习过。供诊断页显示。 */
volatile uint8_t gyro_az_done = 0;
/* 上电收敛期: fusion_ready 之后、auto-zero 首次学完之前, 且判定为静止。
 * Fusion_Task 在此时用 MAHONY_TWOKP_9AXIS_BOOT 提高增益, 加快初值收敛。 */
volatile uint8_t heading_settling = 0;

/* 上电 yaw 基准 (度)。yaw_display = wrap180(yaw_mahony - yaw_boot_base) */
float yaw_boot_base = 0.0f;
/* 供主循环显示的 yaw (已去掉上电基准, 仅六轴使用) */
float yaw_display  = 0.0f;

/* 磁力计直接算出的参考航向(度), 仅九轴有效。用于诊断页对照, 判断轴映射。
 * 由 Fusion_Task 在九轴时更新。 */
float    mag_ref_yaw = 0.0f;
uint8_t  mag_ref_valid = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Mag_Calibrate(void);

/* 融合任务: 由 TIM2 中断节拍以 1kHz 触发, 定义在文件末尾 */
static void Fusion_Task(void);
/* 角度归一化到 (-180, +180], 供航向显示使用。 */
static float Wrap180(float deg);
/* 静止自动零偏修正: 检测到静止时缓慢学习陀螺残余零偏, 抑制 Yaw 漂移。 */
static void GyroAutoZero_Task(void);
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
  Proto_Init();          /* 初始化 TX 环形缓冲 (必须在 USART2 初始化之后) */
  HAL_Delay(50); // 上电稳定 (原 100ms, 实测 50ms 足够)
  
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
  /* 融合频率必须与 TIM2 中断节拍严格一致。
   * 当前: TIM2 = 200Hz (tim.c: PSC=719, ARR=4999 -> 5ms)。
   * invSampleFreq = 1/200, 每 5ms 积分一次陀螺。
   * 若这个数字与真实节拍不符, 角度会按比例偏大或偏小。 */
  Mahony_Init(200.0f);

  /* ================= 上电初始化顺序 (顺序很重要, 别调换) =================
   *   1) 从 Flash 读回磁力计偏移 与 6/9 轴开关
   *   2) 按模式设置 Mahony 增益
   *   3) 用一次静止测量初始化四元数 (MahonyAHRSinit)
   *   4) 置 fusion_ready 并开定时器 —— 必须在 3) 之后!
   *      否则中断会用未初始化的四元数先跑一段, 上电姿态会错。
   * ===================================================================== */

  /* flash_flag 用来区分: 设置是「从 Flash 读来的」还是「走了默认值」。
   * 若它不等于 FLASH_FLAG_VALUE_V2, 说明存储无效, 用的是下面的默认值。
   * 这个显示很重要: 之前调默认模式时, 就是被 Flash 里的旧值盖住了。 */
  uint32_t flash_flag = 0;
  Flash_Load_Settings(&mmc5603_data.offset_x, &mmc5603_data.offset_y, &mmc5603_data.offset_z, &mag_enabled, &flash_flag);

  /* 1) 增益: 9 轴的 twoKp 决定磁航向被"信任"的程度。
   *    太大 -> Yaw 紧跟磁力计, 磁干扰与标定误差会 1:1 灌进 Yaw;
   *    太小 -> 陀螺积分占主导, Yaw 缓慢漂移。
   *    MAHONY_TWOKP_9AXIS 是保守起点, 想调就改它 (建议每次加减 0.5)。 */
  Mahony_SetKp(mag_enabled ? MAHONY_TWOKP_9AXIS : MAHONY_TWOKP_6AXIS);

  /* 上电提示 (第 3 行): 一眼看清当前状态。
   *   Mag: ON  / Mag: OFF  -> 九轴 / 六轴 (磁力计是否参与融合)
   *   F / D                 -> 设置来自 Flash 有效存储 / 走了默认值
   *   MagErr                -> MMC5603 初始化失败 (ID 读不到)
   * 说明: Mag: ON/OFF 看的是 mag_enabled (是否参与融合),
   *       而 MagErr 看的是 mag_status (芯片是否真的应答)。
   *       两者独立: 芯片坏了但开关是 ON, 会显示 "Mag: ON MagErr"。 */
  OLED_ShowString(0, 20, mag_enabled ? "Mag: ON " : "Mag: OFF", OLED_6X8);
  OLED_ShowString(60, 20, (flash_flag == FLASH_FLAG_VALUE_V2) ? "F" : "D", OLED_6X8);
  if (mag_status != 0) {
      OLED_ShowString(72, 20, "MagErr", OLED_6X8);
  }
  OLED_Update();
  HAL_Delay(400); // 让用户看清提示 (原 800ms, 缩短以加快上电)
  OLED_Clear();
  OLED_Update();
  LOG_DEBUG("Loaded Offsets: %.3f, %.3f, %.3f\r\n", mmc5603_data.offset_x, mmc5603_data.offset_y, mmc5603_data.offset_z);

  /* 2) 用一次静止测量初始化四元数。
   *    九轴时磁力计分量必须用与 Fusion_Task 相同的轴重映射 (-my, mx, mz),
   *    否则初值航向与融合航向不在同一坐标系, 上电会有一次跳变。 */
  /* 多次采样取平均再初始化。只取一次的话, 磁力计单次噪声会直接变成
   * 初值航向误差, 那个误差就要靠融合慢慢收敛掉 —— 这正是上电感觉
   * 收敛慢的原因之一。取平均后初值准了, 残余误差就小得多。
   * 代价: 磁力计每次读取内部有 10ms 延时, 3 次约 30ms, 可接受。 */
  {
      const uint8_t N_ACC = 5, N_MAG = 3;
      float sax = 0.0f, say = 0.0f, saz = 0.0f;
      float smx = 0.0f, smy = 0.0f, smz = 0.0f;
      uint8_t k;

      for (k = 0; k < N_ACC; k++)
      {
          Get_AccGyro_ICM42688();
          sax += icm42688_acc_x; say += icm42688_acc_y; saz += icm42688_acc_z;
          HAL_Delay(2);
      }
      icm42688_acc_x = sax / (float)N_ACC;
      icm42688_acc_y = say / (float)N_ACC;
      icm42688_acc_z = saz / (float)N_ACC;

      if (mag_enabled != 0)
      {
          for (k = 0; k < N_MAG; k++)
          {
              MMC5603_ReadData_Single(&hi2c2);
              smx += mmc5603_data.mag_x_gauss;
              smy += mmc5603_data.mag_y_gauss;
              smz += mmc5603_data.mag_z_gauss;
          }
          mmc5603_data.calib_x = smx / (float)N_MAG - mmc5603_data.offset_x;
          mmc5603_data.calib_y = smy / (float)N_MAG - mmc5603_data.offset_y;
          mmc5603_data.calib_z = smz / (float)N_MAG - mmc5603_data.offset_z;
      }
  }
  /* 磁力计实参必须与 Fusion_Task 用同一套重映射, 否则上电初值
   * 与融合航向不在一个坐标系, 上电会有一次跳变。 */
#if HEADING_STD_CONVENTION
  MahonyAHRSinit(
      icm42688_acc_x, icm42688_acc_y, icm42688_acc_z,
      (mag_enabled != 0) ? ( mmc5603_data.calib_y) : 0.0f,
      (mag_enabled != 0) ? (-mmc5603_data.calib_x) : 0.0f,
      (mag_enabled != 0) ? ( mmc5603_data.calib_z) : 0.0f
  );
#else
  MahonyAHRSinit(
      icm42688_acc_x, icm42688_acc_y, icm42688_acc_z,
      (mag_enabled != 0) ? (-mmc5603_data.calib_y) : 0.0f,
      (mag_enabled != 0) ? ( mmc5603_data.calib_x) : 0.0f,
      (mag_enabled != 0) ? ( mmc5603_data.calib_z) : 0.0f
  );
#endif

  /* 3) 初始化完成, 使能融合并开节拍 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);   

#if YAW_ZERO_ON_BOOT
  /* 先让融合跑一小会儿再取基准: MahonyAHRSinit 之后马上读, 姿态还带着
   * 加速度计的初始瞬态。等 200ms (40 个节拍) 再取, 基准更稳。
   * 注意必须在 fusion_ready=1 之后, 否则融合没跑, yaw 不更新。 */
  fusion_ready = 1;
  {
      uint8_t k;
      for (k = 0; k < 40; k++) HAL_Delay(5);   /* 约 200ms */
      yaw_boot_base = yaw_mahony;
  }
#else
  fusion_ready = 1;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t display_count = 0;
  /* tx_tick 已删：发帧改用 TIM2 中断里的静态分频变量 tx_div */
  
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
		/* 与上电时一致, 统一走调参区的宏, 避免两处不一致。 */
		Mahony_SetKp(mag_enabled ? MAHONY_TWOKP_9AXIS : MAHONY_TWOKP_6AXIS);
		// if (mag_enabled == 0) 
    //     {
    //         // 刚切到 6轴模式：把当前的绝对 Yaw 值记录为偏置
    //         // 这样：显示值 = 当前值 - 偏置 = 0
    //         yaw_offset = yaw_mahony; 
    //     }
    //     else 
    //     {
    //         // 刚切回 9轴模式：偏置清零，恢复绝对航向
    //         yaw_offset = 0.0f; 
    //     }

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
        LOG_DEBUG("Start Calibration...\r\n");
        // 暂停定时器中断防止冲突（可选，视中断负载而定）
        HAL_TIM_Base_Stop_IT(&htim2); 
        Mag_Calibrate(); 
        HAL_TIM_Base_Start_IT(&htim2);
        
        OLED_Clear(); // 校准完清屏
    }
    
  /* 磁力计只在九轴模式下才读取。
   * MMC5603_ReadData_Single 内部有 10ms 阻塞延时, 六轴时读它纯属浪费,
   * 而且会把主循环周期从 1ms 拖到 10ms 以上。 */
  if (mag_enabled != 0)
      {
          // 只有开启时，才发送 I2C 命令读取
          MMC5603_ReadData_Single(&hi2c2);

          // 计算校准值
          mmc5603_data.calib_x = mmc5603_data.mag_x_gauss - mmc5603_data.offset_x;
          mmc5603_data.calib_y = mmc5603_data.mag_y_gauss - mmc5603_data.offset_y;
          mmc5603_data.calib_z = mmc5603_data.mag_z_gauss - mmc5603_data.offset_z;
      }

    /* 融合已经在 TIM2 中断里按固定节拍完成(见 Fusion_Task 与 HAL_TIM_PeriodElapsedCallback),
     * 主循环不再参与采样与解算, 只负责显示与按键。
     * 之前把融合放在主循环取标志的写法会丢样本: 主循环频率远低于采样率,
     * 标志只能记住「有一次」, 中间的样本被整批丢弃, 积分出的角度会严重偏小。 */
    GyroAutoZero_Task();   // 静止时学习陀螺残余零偏 (抑制 Yaw 漂移)

    /* 统一 yaw: 六轴下减去上电基准, 让读数从 0 开始 (相对上电的转角);
     * 九轴下 yaw 是磁北绝对方位, 不加基准。
     *
     * ?? 这一步必须在下面 Proto_SendImu() 之前!
     *   之前它被放在显示刷新之前, 而协议打包在它前面,
     *   结果 OLED 显示的是【归零后】的 yaw、协议发的是【未归零】的原始值,
     *   同一时刻两处相差一个上电基准(实测差了 52 度), 会污染训练数据标签。
     *   现在两者统一用 yaw_display。 */
    /* yaw 归零已随发帧一起移到 TIM2 中断里（打包前一步），
     * 保证「协议发出的 yaw = 屏幕显示的 yaw」。屏幕读的就是中断里写的 yaw_display。 */

    /* 串口数据帧: 主循环每 2 圈发一帧。
     * 主循环一轮约 12ms (九轴时磁力计读取占 10ms), 2 圈约 24ms -> 约 42Hz。
     * 想要严格 100Hz, 需要用独立定时器节拍 (见 docs/串口帧协议.md 的说明);
     * 这里先用简单可靠的方式跑通链路, 上位机侧用 timestamp_ms 校验实际间隔。 */
    /* 发帧已移到 TIM2 中断里做（200 Hz 解算二分频 = 严格 100 Hz）。
     * 原来在这里"每 2 圈发一帧"，而主循环受磁力计 10 ms 阻塞与 OLED
     * 刷屏影响一轮约 12 ms，实测只有约 42 Hz 且间隔不匀，
     * 会把节点 B 的 ML 特征（差分 Jerk、过零率、头尾 0.5 s 切片）全部弄错。
     * 详见 HAL_TIM_PeriodElapsedCallback 里的说明。 */

    display_count++;        // 每 10 次刷新一次显示(200Hz/10 = 20Hz)

    /* 刷新率 = 主循环频率 / 阈值。主循环一轮约 12ms(九轴时磁力计读取占 10ms):
     *   阈值 10 -> 约 8Hz(偏慢, 肉眼可见跳字)
     *   阈值  4 -> 约 20Hz(流畅)  <- 当前
     * 想更快需先把磁力计改成非阻塞读取(MMC5603 驱动里有 10ms 延时)。 */
    if (display_count >= 4) 
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
            /* 六轴: 已减去上电基准, 上电即 0 度, 表示【相对起点的航向】
             * 九轴: 磁北绝对方位 + HEADING_ZERO_DEG 零点偏移
             * 具体由主循环算好的 yaw_display 给出 */
            OLED_ShowFloatNum(40, 48, Wrap180(yaw_display + (mag_enabled ? HEADING_ZERO_DEG : 0.0f)), 3, 2, OLED_6X8);
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
            /* Page 2: 融合诊断 / 磁力计参考航向。
             * 六轴 (mag OFF): 看陀螺残余零偏与校准质量。
             * 九轴 (mag ON ): 看磁力计参考航向 H, 用来判断轴映射是否正确。 */
            if (!mag_enabled)
            {
                OLED_ShowString(0, 0, "--- Fusion Diag ---", OLED_6X8);

                /* GZ: 当前陀螺 Z 轴零偏(dps)。六轴下 Yaw 无绝对基准,
                 *     Yaw 漂移速度就等于这个值: 0.1dps 即每秒 0.1 度。 */
                OLED_ShowString(0, 16, "GZ :", OLED_6X8);
                OLED_ShowFloatNum(30, 16, icm42688_gyro_z, 2, 3, OLED_6X8);

                /* RES: 上电加速度计校准残差(g), 正常 < 0.02。 */
                OLED_ShowString(0, 32, "RES:", OLED_6X8);
                OLED_ShowFloatNum(30, 32, imu_calib_residual, 2, 3, OLED_6X8);

                /* ST: 上电校准质量 (RES<0.02 为 OK)。AZ: 静止自动零偏是否已学到。 */
                OLED_ShowString(0, 48, "ST:", OLED_6X8);
                if (imu_calib_residual < 0.02f) OLED_ShowString(18, 48, "OK ", OLED_6X8);
                else                            OLED_ShowString(18, 48, "BAD", OLED_6X8);
                OLED_ShowString(54, 48, "AZ:", OLED_6X8);
                if (gyro_az_done) OLED_ShowString(78, 48, "1", OLED_6X8);
                else              OLED_ShowString(78, 48, "0", OLED_6X8);
            }
            else
            {
                /* 九轴: 显示磁力计原始值 + 参考航向 H。
                 * 绕竖轴慢慢转 360 度, H 与第 1 页的 Yaw 都应单调变化。
                 *   H 单调而 Yaw 反向/乱跳  -> (-my, mx, mz) 重映射方向不对;
                 *   H 不随转动变化/显示 noMag -> 磁力计没读到有效值。 */
                OLED_ShowString(0, 0, "--- Mag / Heading ---", OLED_6X8);

                OLED_ShowString(0, 16, "MX:", OLED_6X8);
                OLED_ShowFloatNum(24, 16, mmc5603_data.mag_x_gauss, 3, 2, OLED_6X8);
                OLED_ShowString(0, 32, "MY:", OLED_6X8);
                OLED_ShowFloatNum(24, 32, mmc5603_data.mag_y_gauss, 3, 2, OLED_6X8);

                OLED_ShowString(0, 48, "H  :", OLED_6X8);
                if (mag_ref_valid) OLED_ShowFloatNum(30, 48, Wrap180(mag_ref_yaw + HEADING_ZERO_DEG), 3, 0, OLED_6X8);
                else               OLED_ShowString(30, 48, "noMag", OLED_6X8);
            }
        }        else if (page_index == 3)
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
//        LOG_DEBUG("Acc:%.3f,%.3f,%.3f,Gyro:%.3f,%.3f,%.3f,Mag:%.3f,%.3f,%.3f\r\n", 
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
		// LOG_DEBUG("raw:%.3f,%.3f,%.3f,offset:%.3f,%.3f,%.3f,cal:%.3f,%.3f,%.3f\r\n", 
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
        /* 串口输出默认关闭(见 debug_log.h)。开启前请先把 USART2 波特率从
         * 9600 改成 115200, 否则 9600 下每帧约 35ms 会阻塞主循环。 */
        LOG_DEBUG("roll:%.2f,pitch:%.2f,yaw:%.2f\r\n", roll_mahony, pitch_mahony, yaw_mahony);
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
/* HAL_TIM_PeriodElapsedCallback 已移到文件末尾, 与 Fusion_Task 放在一起 */
// 磁力计校准函数
void Mag_Calibrate(void)
{
    float x_min = 1000.0f, x_max = -1000.0f;
    float y_min = 1000.0f, y_max = -1000.0f;
    float z_min = 1000.0f, z_max = -1000.0f;
    uint16_t sample_count = 0;
    
    // 给串口一点时间缓冲
    HAL_Delay(50);
    LOG_DEBUG("Start Mag Calibration...\r\n");
    
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
    
    LOG_DEBUG("Saving to Flash...\r\n");
    Flash_Save_Settings(mmc5603_data.offset_x, mmc5603_data.offset_y, mmc5603_data.offset_z, mag_enabled);
    LOG_DEBUG("Save Done!\r\n");
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Calib Done", OLED_6X8);
    OLED_ShowString(0, 16, "Saved to Flash", OLED_6X8); // 提示已保存
    OLED_Update();
    
    // 一次性打印结果
    HAL_Delay(100); 
    LOG_DEBUG("Mag Calibration Done!\r\n");
    HAL_Delay(10);
    LOG_DEBUG("Offset X: %.4f\r\n", mmc5603_data.offset_x);
    HAL_Delay(10);
    LOG_DEBUG("Offset Y: %.4f\r\n", mmc5603_data.offset_y);
    HAL_Delay(10);
    LOG_DEBUG("Offset Z: %.4f\r\n", mmc5603_data.offset_z);
    
    HAL_Delay(2000);
}
/* USER CODE END 4 */
/* =================================================================================
 * 静止自动零偏修正 (Gyro Auto-Zero)
 *
 * 为什么需要:
 *   六轴模式下 Yaw 没有任何绝对基准(加速度计只能观测重力方向, 不约束航向),
 *   因此 Yaw 的漂移速度【精确等于】陀螺 Z 轴残余零偏:
 *       0.1 dps  ->  每秒 0.1 度  ->  每分钟 6 度
 *   上电那次静态标定(1 秒)受上电倾斜、温度、噪声影响, 往往残留 0.05~0.2 dps。
 *   实测本板残留约 0.11 dps, 即每分钟漂 6.8 度, 几分钟就累积成几十度。
 *
 * 做法 (可穿戴设备的标准手段):
 *   当陀螺三轴都明显安静(判定为静止)时, 累积一段时间求均值,
 *   再把该均值补进 gyro_offset。驱动层读取时已经 -= gyro_offset,
 *   所以补进去之后静止时的读数会自动收敛到 0。
 *
 * 安全性:
 *   - 需要连续静止 5 秒才更新一次, 剧烈运动时计数清零, 不会把真实转动学成零偏;
 *   - 阈值 0.5 dps / 0.08 g 足够宽, 正常手持晃动也会被判为运动而不学习;
 *   - 由 fusion_ready 门控, 上电校准与 AHRS 初始化完成前不动作。
 * ================================================================================= */
#define GYRO_AZ_THRESHOLD_DPS   0.5f     /* 三轴都小于此值才算静止 */
#define ACC_AZ_THRESHOLD_G      0.08f    /* 加速度模值偏离 1g 小于此值 */
#define GYRO_AZ_SAMPLES         500      /* 100ms x 500 = 静止 50 秒... 见下 */

static void GyroAutoZero_Task(void)
{
    static uint32_t tick_last  = 0;
    static uint32_t n_samples  = 0;
    static float    sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    float amag;
    uint32_t now;

    if (!fusion_ready) return;

    /* 只在静止时累积。剧烈运动立刻清零重来, 避免把真实转动学成零偏。 */
    amag = sqrtf(icm42688_acc_x * icm42688_acc_x
               + icm42688_acc_y * icm42688_acc_y
               + icm42688_acc_z * icm42688_acc_z);

    if (fabsf(icm42688_gyro_x) > GYRO_AZ_THRESHOLD_DPS ||
        fabsf(icm42688_gyro_y) > GYRO_AZ_THRESHOLD_DPS ||
        fabsf(icm42688_gyro_z) > GYRO_AZ_THRESHOLD_DPS ||
        fabsf(amag - 1.0f) > ACC_AZ_THRESHOLD_G)
    {
        n_samples = 0;
        sum_x = sum_y = sum_z = 0.0f;
        heading_settling = 0;   /* 有运动, 退出收敛期(此时大增益不安全) */
        return;
    }

    /* 判定静止: 若首次零偏还没学完, 就处于收敛期, 用大增益快速收敛。 */
    heading_settling = (gyro_az_done == 0) ? 1u : 0u;

    /* 每 100ms 取一个样本。首次 10 个样本(约1秒)即修正, 之后 50 个样本。
     * 首次均值噪声约 0.028/sqrt(10) = 0.009 dps, 之后降到约 0.004 dps。 */
    now = HAL_GetTick();
    if ((now - tick_last) < 100u) return;
    tick_last = now;

    sum_x += icm42688_gyro_x;
    sum_y += icm42688_gyro_y;
    sum_z += icm42688_gyro_z;
    n_samples++;

    /* 阈值分两档: 首次只要 10 个样本(静止约1秒)就修正, 让开机头几秒的漂移尽快消失;
     * 之后回到 50 个样本(静止5秒), 长期跟踪更稳, 不易被扰动带偏。 */
    if (n_samples >= ((gyro_az_done == 0) ? 10u : 50u))
    {
        /* 读数 = 真值 - gyro_offset, 静止时真值=0, 故读数均值就是残余零偏。
         * 把它补进 gyro_offset 即可让静止读数收敛到 0。 */
        gyro_offset_x += sum_x / (float)n_samples;
        gyro_offset_y += sum_y / (float)n_samples;
        gyro_offset_z += sum_z / (float)n_samples;

        n_samples = 0;
        sum_x = sum_y = sum_z = 0.0f;
        gyro_az_done = 1;
    }
}
/* 把角度归一化到 (-180, +180]。显示航向必须做这一步, 否则加偏移后会跑出范围。 */
static float Wrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/**
  * @brief  单样本融合任务 (由 TIM2 中断节拍以 200Hz 触发)
  * @note   一次 12 字节 burst 同时取回加速度与陀螺, 保证六轴同源;
  *         磁力计仅在九轴模式下参与, 其读取在中断外完成(主循环)。
  */
static void Fusion_Task(void)
{
    Get_AccGyro_ICM42688();

    if (mag_enabled == 1)
    {
        /* 九轴融合 (MMC5603 参与)。
         * 保持原有轴重映射 (-my, mx, mz): 磁力计 Y 轴对应板子前方。
         * 开九轴前请先验证: 绕竖轴转 360 度, Yaw 应单调变化并转回原点。 */
        /* HZ_SIGN 与 MAG_SWAP 必须成对: 见文件头 航向方向约定 的说明。
         * 这里用宏, 切换约定时只改一处, 不会漏改导致陀螺与磁力计打架。 */
#if HEADING_STD_CONVENTION
        Mahony_update(
            icm42688_gyro_x * 0.0174533f,
            icm42688_gyro_y * 0.0174533f,
           -icm42688_gyro_z * 0.0174533f,          /* 陀螺 Z 取反 */
            icm42688_acc_x, icm42688_acc_y, icm42688_acc_z,
             mmc5603_data.calib_y,                 /* 磁力计 (+my, -mx) */
            -mmc5603_data.calib_x,
             mmc5603_data.calib_z
        );
#else
        Mahony_update(
            icm42688_gyro_x * 0.0174533f,
            icm42688_gyro_y * 0.0174533f,
            icm42688_gyro_z * 0.0174533f,
            icm42688_acc_x, icm42688_acc_y, icm42688_acc_z,
            -mmc5603_data.calib_y,
             mmc5603_data.calib_x,
             mmc5603_data.calib_z
        );
#endif
    }
    else
    {
        // 六轴融合 (只用陀螺仪和加速度计) —— 上电默认走这条路
        MahonyAHRSupdateIMU(
            icm42688_gyro_x * 0.0174533f,
            icm42688_gyro_y * 0.0174533f,
            MAHONY_GZ_SIGN_6AXIS * icm42688_gyro_z * 0.0174533f,   /* 六轴独立符号 */
            icm42688_acc_x, icm42688_acc_y, icm42688_acc_z
        );
    }

    /* 收敛期切换增益: 静止且未学完首次零偏时用大增益, 加速上电收敛。
     * 用 SetKp 而不是改 Mahony_Init, 因为 Init 会重置四元数与积分项。
     * 只对九轴生效(六轴没有磁力计, Yaw 本就不可观测, 提增益无意义)。 */
    if (mag_enabled == 1)
    {
        float want_kp = heading_settling ? MAHONY_TWOKP_9AXIS_BOOT : MAHONY_TWOKP_9AXIS;
        if (want_kp != Mahony_GetKp()) Mahony_SetKp(want_kp);
    }

    Mahony_computeAngles();

    /* 九轴时顺便算出磁力计参考航向, 供诊断页对照。
     * 复刻 MahonyAHRSinit 的倾斜补偿公式, 但用与融合一致的轴重映射。
     * 若这个角度单调变化而 Yaw 反向或乱跳, 说明重映射方向错了。 */
    if (mag_enabled == 1)
    {
        float cr2 = cosf(roll_mahony  * 0.0174533f);
        float sr2 = sinf(roll_mahony  * 0.0174533f);
        float cp2 = cosf(pitch_mahony * 0.0174533f);
        float sp2 = sinf(pitch_mahony * 0.0174533f);
#if HEADING_STD_CONVENTION
        float bx  =  mmc5603_data.calib_y;   /* 与 Fusion_Task 的重映射一致 */
        float by  = -mmc5603_data.calib_x;
#else
        float bx  = -mmc5603_data.calib_y;
        float by  =  mmc5603_data.calib_x;
#endif
        float bz  =  mmc5603_data.calib_z;
        float magX, magY;

        magX = bx * cp2 + by * sp2 * sr2 + bz * sp2 * cr2;
        magY = by * cr2 - bz * sr2;
        mag_ref_yaw = atan2f(-magY, magX) * 57.29578f;
        mag_ref_valid = ((bx * bx + by * by + bz * bz) > 0.01f) ? 1u : 0u;
    }
    else
    {
        mag_ref_valid = 0u;
    }
}

/**
  * @brief  TIM2 周期回调: 200Hz 融合节拍
  * @note   TIM2 优先级已下调为 1(低于 SysTick), 因此这里即使耗时偏长,
  *         HAL_GetTick / HAL_Delay 的超时判断也不会失效。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 融合就在这里做, 每个节拍恰好消耗一个样本, 一个都不丢。
         * 这是角度准确的关键: 中断不会被 OLED / 串口 / 磁力计阻塞,
         * 因此 invSampleFreq = 1/200 与实际间隔严格一致。
         * 就绪门控: 上电校准与 MahonyAHRSinit 完成前不做融合。 */
        if (fusion_ready)
        {
            static uint8_t tx_div = 0U;   /* 200 Hz -> 100 Hz 发帧分频 */

            Fusion_Task();

            /* yaw 归零必须在打包之前：保证「发出的值 = 屏幕显示的值」
             * （曾经协议发原始 yaw、屏幕显归零后的 yaw，差一个上电基准实测 52 度，
             *   污染训练标签）。 */
#if YAW_ZERO_ON_BOOT
            yaw_display = (mag_enabled == 0) ? Wrap180(yaw_mahony - yaw_boot_base)
                                            : yaw_mahony;
#else
            yaw_display = yaw_mahony;
#endif

            /* ★ 严格 100 Hz 发帧（2026-10-14 修正）
             * 原先在主循环里"每 2 圈发一帧"：主循环受 MMC5603_ReadData_Single()
             * 的 10 ms 阻塞与 OLED 软 I2C 刷新影响，一轮约 12 ms，
             * 实测只有约 42 Hz 且间隔不匀；而节点 B 的 ML 特征
             * （差分 Jerk × FS、过零率、头尾 0.5 s 切片）全部假设严格 100 Hz。
             * 放到本中断里二分频，间隔严格 10 ms，且不影响磁力计与显示。
             * Proto_SendImu() 是非阻塞设计（DMA 忙则丢帧并计入 stat_dropped），
             * 可安全在中断上下文调用。 */
            tx_div ^= 1U;
            if (tx_div == 0U)
            {
                Proto_SendImu();
            }
        }
    }
    else if (htim->Instance == TIM3)
    {
        Key_Tick();
    }
}

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
