/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#define D2R_CONVERT 0.0174532925f
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
eepromConfig_t eepromConfig;
uint32_t last_print_tick = 0;
const uint32_t print_interval = 100; // 每 100ms 打印一次 (10Hz)
bool mpu_ok = false;

sensors_t sensors;

float testPhase = -1.0f * D2R;
float testPhaseDelta = 10.0f * D2R;

uint16_t timerValue;

uint32_t currentTime;
uint32_t deltaTime1000Hz, previous1000HzTime, executionTime1000Hz;
uint32_t deltaTime500Hz, previous500HzTime, executionTime500Hz;

float dt500Hz;

// [TIMING TEST] 记录 computeMotorCommands() 起点时间戳（us）
uint32_t dbg_compute_start_us = 0;
// [TIMING TEST] 标记是否已有有效的 compute 起点时间戳
uint8_t dbg_has_compute_stamp = 0;
// [TIMING TEST] 打印分频计数，避免 500Hz 下串口刷屏
uint16_t dbg_timing_print_div = 0;

uint32_t loopStartTime;

// 给sysytemReady转换的全局变量
uint32_t sysTickUptime = 0;       // 系统毫秒计数器
uint32_t sysTickCycleCounter = 0; // DWT 周期计数器

bool systemReady = false;

uint16_t frameCounter = 0;

uint32_t usTicks = 0;

bool frame_500Hz = false;

// 开机姿态收敛延时计数器
uint32_t startup_delay_counter = 0;
float attitude_deg_prev[3] = {0.0f, 0.0f, 0.0f};
uint8_t attitude_delta_initialized = 0;

uint32_t micros(void)
{
  uint32_t m0 = HAL_GetTick();
  uint32_t u0 = SysTick->VAL;
  uint32_t m1 = HAL_GetTick();
  uint32_t u1 = SysTick->VAL;
  const uint32_t tms = SysTick->LOAD + 1;

  if (m1 != m0)
  {
    return (m1 * 1000 + ((tms - u1) * 1000) / tms);
  }
  else
  {
    return (m0 * 1000 + ((tms - u0) * 1000) / tms);
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
  //-1.打开中断
  HAL_SYSTICK_Config(SystemCoreClock / 1000);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // 0.打开DWT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 1.先是初始化ee结构体
  init_eepromConfig(true);
  // 2.初始化PWM
  PWM_Motor_Init();
  // HAL_Delay(3000);
  // 3.初始化滤波的
  initFirstOrderFilter();
  // 4.初始化pid
  initPID();
  // 5.最后初始化mpu6050
  orientIMU();
  if (MPU6050_Init(ACCEL_FS_4G, GYRO_FS_500DPS, DLPF_CFG_42HZ))
  {
    HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET);
    printf(">>> MPU6050 Init SUCCESS! Ready to read data.\r\n");
    printf(">>> 正在执行温度校准（请耐心等待约 2 分钟）...\r\n");
    mpu6050Calibration(); // 这是计算温度的
    printf(">>> 校准完成！开始输出数据。\r\n\r\n");
  }
  else
  {
    printf(">>> 初始化失败了。\r\n\r\n");
  }

  // 6.初始姿态角
  //  【修改】先关闭电机使能，等AHRS收敛后再开//
  eepromConfig.pitchEnabled = false;
  eepromConfig.rollEnabled = false;
  eepromConfig.yawEnabled = false;

  // yaw
  eepromConfig.PID[YAW_PID].P = 1.0f;
  eepromConfig.PID[YAW_PID].I = 0.01f;
  eepromConfig.PID[YAW_PID].D = 0.01f;
  eepromConfig.PID[YAW_PID].lastDcalcValue = 0.0f; // 开机姿态可能不对，先给0
  eepromConfig.PID[YAW_PID].lastDterm = 0.0f;
  eepromConfig.PID[YAW_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[YAW_PID].windupGuard = 0.1f;
  // pitch
  eepromConfig.PID[PITCH_PID].P = 0.01f;
  eepromConfig.PID[PITCH_PID].I = 0.0f; // 大幅减小，防止爆炸
  eepromConfig.PID[PITCH_PID].D = 0.0f;
  eepromConfig.PID[PITCH_PID].lastDcalcValue = 0.0f;
  eepromConfig.PID[PITCH_PID].lastDterm = 0.0f;
  eepromConfig.PID[PITCH_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[PITCH_PID].windupGuard = 0.5236f;

  // roll
  eepromConfig.PID[ROLL_PID].P = 0.005f; // 跟 Pitch 给一样的值作为起步
  eepromConfig.PID[ROLL_PID].I = 0.00f;
  eepromConfig.PID[ROLL_PID].D = 0.0f;
  eepromConfig.PID[ROLL_PID].lastDcalcValue = 0.0f;
  eepromConfig.PID[ROLL_PID].lastDterm = 0.0f;
  eepromConfig.PID[ROLL_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[ROLL_PID].windupGuard = 0.5236f;

  systemReady = true;
  /* if (MPU6050_Init(ACCEL_FS_4G, GYRO_FS_500DPS, DLPF_CFG_42HZ)) {
         mpu_ok = true;
         HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET); // 成功：亮灯 (注意检查你的宏定义是 LED1_GPIO 还是 LED1_GPIO_Port)
         printf(">>> MPU6050 Init SUCCESS! Ready to read data.\r\n");
     } else {
         mpu_ok = false;
         HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_RESET); // 失败：灭灯
         printf(">>> ERROR: MPU6050 Init FAILED! Check wiring (I2C SDA/SCL) and power.\r\n");
     }*/

  // ======= 防止第一圈 dt 爆炸引发 NaN =======
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  previous500HzTime = micros();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (frame_500Hz)
    {
      frame_500Hz = false;

      currentTime = micros();
      deltaTime500Hz = currentTime - previous500HzTime;
      previous500HzTime = currentTime;

      // 【我帮你补全：正确输出 gx】
      /*float test_sensor_x = (float)rawGyro[XAXIS].value / 16.4f;
      printf("TEST 传感器X角速度 = %.2f\n", test_sensor_x);*/

      /*// 读取陀螺仪原始数据并转换成物理量
      float gx = ((float)rawGyro[ROLL].value - gyroRTBias[ROLL] - gyroTCBias[ROLL]) * GYRO_SCALE_FACTOR;
      float gy = ((float)rawGyro[PITCH].value - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;

      // 输出 gx（直接用）
      printf("gy = %.2f rad/s\r\n", gy);*/

      dt500Hz = (float)deltaTime500Hz / 1000000.0f;

      MPU6050_Read_And_Process();

      sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
      sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
      // 原来是有负号的，但是应该是在MPU6050_Read_And_Process()调整了轴方向，所以去掉符号保证向下加速度，避免反重力
      sensors.accel500Hz[ZAXIS] = ((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;
      sensors.gyro500Hz[ROLL] = ((float)rawGyro[ROLL].value - gyroRTBias[ROLL] - gyroTCBias[ROLL]) * GYRO_SCALE_FACTOR;
      sensors.gyro500Hz[PITCH] = ((float)rawGyro[PITCH].value - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
      sensors.gyro500Hz[YAW] = ((float)rawGyro[YAW].value - gyroRTBias[YAW] - gyroTCBias[YAW]) * GYRO_SCALE_FACTOR;

      // 输出加速度 + 陀螺仪 6个物理量
      /* printf("ACC: X=%.3f  Y=%.3f  Z=%.3f | GYRO: ROLL=%.4f  PITCH=%.4f  YAW=%.4f\r\n",
              sensors.accel500Hz[XAXIS],
              sensors.accel500Hz[YAXIS],
              sensors.accel500Hz[ZAXIS],
              sensors.gyro500Hz[ROLL],
              sensors.gyro500Hz[PITCH],
              sensors.gyro500Hz[YAW]);*/

      // [TIMING TEST] 测量“上一次 computeMotorCommands() 开始 -> 本次 MargAHRSupdate() 开始”的耗时
      /*if (dbg_has_compute_stamp)
      {
        uint32_t dbg_marg_start_us = micros();
        uint32_t dbg_compute_to_next_marg_us = dbg_marg_start_us - dbg_compute_start_us;
        float dbg_compute_to_next_marg_ms = (float)dbg_compute_to_next_marg_us * 0.001f;
        float dbg_dt500Hz_ms = dt500Hz * 1000.0f;

        dbg_timing_print_div++;
        if (dbg_timing_print_div >= 50) // 500Hz 下约每 100ms 打印一次
        {
          dbg_timing_print_div = 0;
          // [TIMING TEST] 两列都以 ms 输出：compute->nextMarg(ms), dt500Hz(ms)
          printf("%.3f,%.3f\r\n",
                 dbg_compute_to_next_marg_ms,
                 dbg_dt500Hz_ms);
        }
      }*/

      MargAHRSupdate(sensors.gyro500Hz[ROLL],
                     sensors.gyro500Hz[PITCH],
                     sensors.gyro500Hz[YAW],
                     sensors.accel500Hz[XAXIS],
                     sensors.accel500Hz[YAXIS],
                     sensors.accel500Hz[ZAXIS],
                     0, 0, 0, false, dt500Hz);

      // =============== 【新增】等待 AHRS 收敛 ================
      if (startup_delay_counter < 1000) // 1000 * 2ms = 2秒
      {
        startup_delay_counter++;
        if (startup_delay_counter == 1000)
        {
          zeroPIDintegralError();           // 清空这2秒内乱算的积分
          zeroPIDstates();                  // 清空D项微分的毛刺
          eepromConfig.pitchEnabled = true; // 姿态稳定，放开PID控制
          eepromConfig.rollEnabled = true;
          eepromConfig.yawEnabled = false;
          printf(">>> AHRS收敛完成，电机使能！\r\n");
        }
      }

      // 先不要补偿影响
      // [TIMING TEST] 记录本次 computeMotorCommands() 开始时间，供下一轮 MargAHRSupdate() 统计耗时
      // dbg_compute_start_us = micros();
      // dbg_has_compute_stamp = 1;
      computeMotorCommands(dt500Hz);
      // PWM_Motor_TestAllAngles();
      // printf("%f\r\n",dt500Hz);

      executionTime500Hz = micros() - currentTime;
    }

    if (HAL_GetTick() - last_print_tick >= 100) // 100ms = 10Hz
    {
      last_print_tick = HAL_GetTick(); // 重置时间戳

      // 确保已经度过了开机不稳定的阶段再开始打印数据
      /*if (systemReady && eepromConfig.pitchEnabled)
      {
        float pitch_angle_deg = sensors.margAttitude500Hz[PITCH] * 57.29578f;
        float target_deg      = pointingCmd[PITCH] * 57.29578f;//这个计算感觉有点问题啊
        float error_deg       = target_deg - pitch_angle_deg;

        printf("[PITCH DEBUG] 目标:%.2f | 当前:%.2f | 误差:%.2f | 积分:%.4f | 输出:%.3f | 时间:%f\r\n",
             target_deg,
             pitch_angle_deg,
             error_deg,
             eepromConfig.PID[PITCH_PID].iTerm,
             pidCmd[PITCH],dt500Hz);
      }

    if (systemReady && eepromConfig.rollEnabled)
    {
      float roll_angle_deg = sensors.margAttitude500Hz[ROLL] * 57.29578f;
      float target_deg      = pointingCmd[ROLL] * 57.29578f;
      float error_deg       = target_deg - roll_angle_deg;

      printf("[ROLL DEBUG] 目标:%.2f | 当前:%.2f | 误差:%.2f | 积分:%.4f | 输出:%.3f\r\n",
      target_deg,
      roll_angle_deg,
      error_deg,
      eepromConfig.PID[ROLL_PID].iTerm,
      pidCmd[ROLL]);
    }*/




      if (systemReady)
      {
        /*float roll_angle_deg = sensors.margAttitude500Hz[ROLL] * 57.29578f;
        float pitch_angle_deg = sensors.margAttitude500Hz[PITCH] * 57.29578f;

        printf("ROLL:%.2f | PITCH:%.2f\r\n",
            roll_angle_deg,pitch_angle_deg);*/

       printf("%.2f,%.2f,%.2f,%.2f\r\n",
               sensors.margAttitude500Hz[ROLL],
             sensors.margAttitude500Hz[PITCH],
			 pidCmd[ROLL],
             pidCmd[PITCH]);
        /*printf("%.2f,%.2f,%.2f,%.2f\r\n",
                     q0,
                   q1,
                   q2,
                   q3);*/

        /*printf("X=%.3f,Y=%.3f,Z=%.3f\r\n",
                     sensors.gyro500Hz[ROLL],
                     sensors.gyro500Hz[PITCH],
                     sensors.gyro500Hz[YAW]);*/;

		/* printf("Err:%.3f | Cmd:%.3f | Angle:%.3f\n",
				 pointingCmd[PITCH] - sensors.margAttitude500Hz[PITCH], pidCmd[PITCH], sensors.margAttitude500Hz[PITCH]);*/
      }
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
