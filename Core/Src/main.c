/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (主程序体)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "computeMotorCommands.h"
#include <math.h>
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define D2R_CONVERT 0.0174532925f  // 角度转弧度转换系数 (pi / 180)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
eepromConfig_t eepromConfig;             // EEPROM 配置结构体，保存系统参数
uint32_t last_print_tick = 0;            // 记录上一次串口打印的时间戳
const uint32_t print_interval = 100;     // 每 100 ms 打印一次（10 Hz）
bool mpu_ok = false;                     // MPU6050 初始化状态标志

sensors_t sensors;                       // 传感器数据结构体（加速度、角速度等）

float testPhase = -1.0f * D2R;           // 测试相位（设为-1度）
float testPhaseDelta = 10.0f * D2R;      // 测试相位增量（设为10度）

uint16_t timerValue;                     // 定时器计数值

// 性能分析与循环耗时计算相关变量
uint32_t currentTime;
uint32_t deltaTime1000Hz, previous1000HzTime, executionTime1000Hz;
uint32_t deltaTime500Hz, previous500HzTime, executionTime500Hz;

float dt500Hz;                           // 500Hz 控制环的实际间隔时间 (秒)

// [TIMING TEST] 记录 computeMotorCommands() 的起始时间戳（us），用于分析函数执行时间
uint32_t dbg_compute_start_us = 0;
// [TIMING TEST] 标记当前是否已有有效的 compute 起始时间戳
uint8_t dbg_has_compute_stamp = 0;
// [TIMING TEST] 打印分频计数，避免 500 Hz 下串口刷屏
uint16_t dbg_timing_print_div = 0;

uint32_t loopStartTime;                  // 主循环起始时间

// 系统计时状态
uint32_t sysTickUptime = 0;              // 系统毫秒计数器
uint32_t sysTickCycleCounter = 0;        // DWT (Data Watchpoint and Trace) 周期计数器

bool systemReady = false;                // 系统初始化完成标志

uint16_t frameCounter = 0;               // 帧计数器

uint32_t usTicks = 0;                    // 微秒级时间滴答

bool frame_500Hz = false;                // 500Hz 循环触发标志位（通常在定时器中断中置位）

// 开机姿态收敛延时计数器
uint32_t startup_delay_counter = 0;      // 记录启动后的运行帧数
float attitude_deg_prev[3] = {0.0f, 0.0f, 0.0f}; // 记录上一帧的姿态角，用于判断姿态是否收敛
uint8_t attitude_delta_initialized = 0;  // 姿态差值初始化标志

/**
 * @brief  获取系统运行的微秒数
 * @note   结合 HAL_GetTick (毫秒) 和 SysTick 寄存器 (倒计时) 来计算高精度的微秒时间
 * @retval 系统的微秒时间戳
 */
uint32_t micros(void)
{
  uint32_t m0 = HAL_GetTick();           // 获取当前毫秒数
  uint32_t u0 = SysTick->VAL;            // 获取当前 SysTick 的倒计数值
  uint32_t m1 = HAL_GetTick();           // 再次获取毫秒数以防在读取过程中发生滴答中断
  uint32_t u1 = SysTick->VAL;            // 再次获取 SysTick 倒计数值
  const uint32_t tms = SysTick->LOAD + 1; // SysTick 重装载值（1ms的周期数）

  // 如果两次获取的毫秒数不同，说明刚好跨越了一个毫秒边界
  if (m1 != m0)
  {
    return (m1 * 1000 + ((tms - u1) * 1000) / tms); // 使用新的毫秒数和倒计数值计算
  }
  else
  {
    return (m0 * 1000 + ((tms - u0) * 1000) / tms); // 没有跨越边界，使用旧的数值计算
  }
}

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // 打开 SysTick 中断，周期设置为 1ms
  HAL_SYSTICK_Config(SystemCoreClock / 1000);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // 打开 DWT (Data Watchpoint and Trace) 周期计数器，用于精确测量代码执行时间（时钟周期级别）
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 先初始化 EEPROM 配置，恢复之前的系统参数
  init_eepromConfig(true);

  // 初始化 PWM 控制，用于驱动电机
  PWM_Motor_Init();
  // HAL_Delay(3000); // 预留的延时（已注释掉）

  // 初始化一阶低通滤波器，用于平滑传感器数据
  initFirstOrderFilter();

  // 初始化 PID 控制器状态参数
  initPID();

  // 最后初始化 MPU6050
  orientIMU(); // 设定 IMU 的安装方向矩阵

  // 初始化 MPU6050 硬件：加速度计量程 ±4g，陀螺仪量程 ±500dps，低通滤波带宽 260Hz
  if (MPU6050_Init(ACCEL_FS_4G, GYRO_FS_500DPS, DLPF_CFG_260HZ))
  {
    HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET); // 点亮 LED 指示初始化成功
    printf(">>> MPU6050 初始化成功！准备开始读取数据。\r\n");
    //printf(">>> 正在执行温度校准，请耐心等待约 2 分钟...\r\n");
    //mpu6050Calibration(); // 执行 IMU 零偏和温度补偿校准
    //printf(">>> 校准完成！开始输出数据。\r\n\r\n");
    // (逻辑上的提示可能有重叠，这里提示跳过温度校准可能是为了调试修改的文案)
    printf(">>> 已跳过温度校准，开始输出数据。\r\n\r\n");
  }
  else
  {
    printf(">>> MPU6050 初始化失败。\r\n\r\n");
  }

  // 初始化姿态控制状态
  // 先关闭电机的各个控制轴使能，等待 AHRS（航姿参考系统）算法收敛后再打开
  eepromConfig.pitchEnabled = false;
  eepromConfig.rollEnabled = false;
  eepromConfig.yawEnabled = false;

  // 初始化 Yaw (偏航轴) PID 参数
  eepromConfig.PID[YAW_PID].P = 0.03f;
  eepromConfig.PID[YAW_PID].I = 0.01f;
  eepromConfig.PID[YAW_PID].D = 0.016f;          // 加一点 D（微分项），减轻自激振荡
  eepromConfig.PID[YAW_PID].lastDcalcValue = 0.0f; // 清空开机时的微分状态缓存
  eepromConfig.PID[YAW_PID].lastDterm = 0.0f;
  eepromConfig.PID[YAW_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[YAW_PID].windupGuard = 0.1f;    // 积分限幅防超调

  // 初始化 Pitch (俯仰轴) PID 参数
  eepromConfig.PID[PITCH_PID].P = 0.01f;
  eepromConfig.PID[PITCH_PID].I = 0.0f;        // I项明显减小，防止累积引起过冲
  eepromConfig.PID[PITCH_PID].D = 0.008f;
  eepromConfig.PID[PITCH_PID].lastDcalcValue = 0.0f;
  eepromConfig.PID[PITCH_PID].lastDterm = 0.0f;
  eepromConfig.PID[PITCH_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[PITCH_PID].windupGuard = 0.5236f; // 积分限幅（约 30 度弧度值）

  // 初始化 Roll (横滚轴) PID 参数
  eepromConfig.PID[ROLL_PID].P = 0.03f;          // 先按与 Pitch 同量级的数值起步
  eepromConfig.PID[ROLL_PID].I = 0.0f;
  eepromConfig.PID[ROLL_PID].D = 0.008f;
  eepromConfig.PID[ROLL_PID].lastDcalcValue = 0.0f;
  eepromConfig.PID[ROLL_PID].lastDterm = 0.0f;
  eepromConfig.PID[ROLL_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[ROLL_PID].windupGuard = 0.5236f;

  systemReady = true; // 系统准备就绪标志位置位
  __HAL_TIM_SET_COUNTER(&htim6, 0); // 清零 TIM6 计数器
  previous500HzTime = micros();     // 记录首次循环的起始微秒时间

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 检查 500Hz 帧触发标志（可能是 TIM 中断触发置位）
    if (frame_500Hz)
    {
      frame_500Hz = false; // 清除标志位，等待下一次中断

      // 计算本帧与上一帧的时间差 (微秒)
      currentTime = micros();
      deltaTime500Hz = currentTime - previous500HzTime;
      previous500HzTime = currentTime;

      // 转换为秒 (dt)，用于积分运算
      dt500Hz = (float)deltaTime500Hz / 1000000.0f;

      // 读取 MPU6050 的原始数据并进行初步处理（如滤波）
      MPU6050_Read_And_Process();

      // 剔除加速度计的偏置误差，并乘以比例因子转换为物理单位 (通常为 m/s^2 或 g)
      sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
      sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
      // 轴向符号已经在 MPU6050_Read_And_Process() 中调整过，
      // 这里保持 Z 轴向下为正，避免对重力方向再次取反。
      sensors.accel500Hz[ZAXIS] = ((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

      // 剔除陀螺仪的运行偏置(RTBias)和温度校准偏置(TCBias)，并转换为弧度/秒或度/秒
      sensors.gyro500Hz[ROLL] = ((float)rawGyro[ROLL].value - gyroRTBias[ROLL] - gyroTCBias[ROLL]) * GYRO_SCALE_FACTOR;
      sensors.gyro500Hz[PITCH] = ((float)rawGyro[PITCH].value - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
      sensors.gyro500Hz[YAW] = ((float)rawGyro[YAW].value - gyroRTBias[YAW] - gyroTCBias[YAW]) * GYRO_SCALE_FACTOR;

      // 运行 MARG (磁传感器、角速率和重力传感器) 航姿参考更新算法
      // 注意：这里磁力计输入均传入 0，表示当前只作为 IMU (6轴) 在运行 Mahony/Madgwick 滤波
      MargAHRSupdate(sensors.gyro500Hz[ROLL],
                     sensors.gyro500Hz[PITCH],
                     sensors.gyro500Hz[YAW],
                     sensors.accel500Hz[XAXIS],
                     sensors.accel500Hz[YAXIS],
                     sensors.accel500Hz[ZAXIS],
                     0, 0, 0, false, dt500Hz);

      // 等待 AHRS（姿态算法）收敛后再打开控制，防止开机瞬间电机乱动
      if (startup_delay_counter < 1000) // 计数 1000 次，500Hz 下相当于延时 2 秒 (1000 * 2ms = 2s)
      {
        startup_delay_counter++;
        if (startup_delay_counter == 1000)
        {
          zeroPIDintegralError();           // 清空在收敛阶段由于姿态偏差累计的 PID 积分量
          zeroPIDstates();                  // 清空微分项的状态变量，防止接通瞬间产生瞬态尖峰
          eepromConfig.pitchEnabled = true; // 姿态稳定后，开启对应的电机控制使能
          eepromConfig.rollEnabled = false;
          eepromConfig.yawEnabled = true;
          printf(">>> AHRS 收敛完成，俯仰轴控制已使能。\r\n");
        }
      }

      // 计算 PID 控制器并输出最终的电机 PWM 指令
      computeMotorCommands(dt500Hz);

      // 记录整个 500Hz 循环的执行耗时，用于性能监控
      executionTime500Hz = micros() - currentTime;
    }

    // 处理 10Hz 的低频非实时任务（如状态检测、串口打印等）
    if (HAL_GetTick() - last_print_tick >= 100) // 100ms = 10Hz 触发条件
    {
      last_print_tick = HAL_GetTick(); // 重置 10Hz 循环的时间戳

      if (systemReady)
      {
          // 这里检查某个归位/返回状态的计数器
    	  // 如果超过阈值且 Pitch 控制已使能，则解除归位状态标志
    	  if(return_state_count>=650 && eepromConfig.pitchEnabled == true)
    	  {
    		  return_state = false;
    		  eepromConfig.rollEnabled = true;
    	  }

    	  if(return_state_count_roll>=650 && eepromConfig.rollEnabled == true)
		  {
			  return_state_roll = false;
		  }
      }

      printf("%.6f,%d\r\n",sensors.margAttitude500Hz[ROLL],return_state_count_roll);
    }
    ////////////////////////////////////////////


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
