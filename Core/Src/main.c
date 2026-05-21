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
const uint32_t print_interval = 100; // 姣?100ms 鎵撳嵃涓€娆?(10Hz)
bool mpu_ok = false;

sensors_t sensors;

float testPhase = -1.0f * D2R;
float testPhaseDelta = 10.0f * D2R;

uint16_t timerValue;

uint32_t currentTime;
uint32_t deltaTime1000Hz, previous1000HzTime, executionTime1000Hz;
uint32_t deltaTime500Hz, previous500HzTime, executionTime500Hz;

float dt500Hz;

// [TIMING TEST] 璁板綍 computeMotorCommands() 璧风偣鏃堕棿鎴筹紙us锛?
uint32_t dbg_compute_start_us = 0;
// [TIMING TEST] 鏍囪鏄惁宸叉湁鏈夋晥鐨?compute 璧风偣鏃堕棿鎴?
uint8_t dbg_has_compute_stamp = 0;
// [TIMING TEST] 鎵撳嵃鍒嗛璁℃暟锛岄伩鍏?500Hz 涓嬩覆鍙ｅ埛灞?
uint16_t dbg_timing_print_div = 0;

uint32_t loopStartTime;

// 缁檚ysytemReady杞崲鐨勫叏灞€鍙橀噺
uint32_t sysTickUptime = 0;       // 绯荤粺姣璁℃暟鍣?
uint32_t sysTickCycleCounter = 0; // DWT 鍛ㄦ湡璁℃暟鍣?

bool systemReady = false;

uint16_t frameCounter = 0;

uint32_t usTicks = 0;

bool frame_500Hz = false;

// 寮€鏈哄Э鎬佹敹鏁涘欢鏃惰鏁板櫒
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
  //-1.鎵撳紑涓柇
  HAL_SYSTICK_Config(SystemCoreClock / 1000);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // 0.鎵撳紑DWT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 1.鍏堟槸鍒濆鍖杄e缁撴瀯浣?
  init_eepromConfig(true);
  // 2.鍒濆鍖朠WM
  PWM_Motor_Init();
  // HAL_Delay(3000);
  // 3.鍒濆鍖栨护娉㈢殑
  initFirstOrderFilter();
  // 4.鍒濆鍖杙id
  initPID();
  // 5.鏈€鍚庡垵濮嬪寲mpu6050
  orientIMU();
  if (MPU6050_Init(ACCEL_FS_4G, GYRO_FS_500DPS, DLPF_CFG_260HZ))//
  {
    HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET);
    printf(">>> MPU6050 Init SUCCESS! Ready to read data.\r\n");
    printf(">>> 姝ｅ湪鎵ц娓╁害鏍″噯锛堣鑰愬績绛夊緟绾?2 鍒嗛挓锛?..\r\n");
    mpu6050Calibration(); // 杩欐槸璁＄畻娓╁害鐨?
    printf(">>> 鏍″噯瀹屾垚锛佸紑濮嬭緭鍑烘暟鎹€俓r\n\r\n");
  }
  else
  {
    printf(">>> 鍒濆鍖栧け璐ヤ簡銆俓r\n\r\n");
  }

  // 6.鍒濆濮挎€佽
  //  銆愪慨鏀广€戝厛鍏抽棴鐢垫満浣胯兘锛岀瓑AHRS鏀舵暃鍚庡啀寮€//
  eepromConfig.pitchEnabled = false;
  eepromConfig.rollEnabled = false;
  eepromConfig.yawEnabled = false;

  // yaw
  eepromConfig.PID[YAW_PID].P = 2.0f;
  eepromConfig.PID[YAW_PID].I = 0.0f;
  eepromConfig.PID[YAW_PID].D = 0.016f;//涓嶅姞D鍚戜細鏈夎交寰嚜浼狅紝鍔犱簡濂戒簡寰堝浜嗭紝鍒硅溅浣滅敤
  eepromConfig.PID[YAW_PID].lastDcalcValue = 0.0f; // 寮€鏈哄Э鎬佸彲鑳戒笉瀵癸紝鍏堢粰0
  eepromConfig.PID[YAW_PID].lastDterm = 0.0f;
  eepromConfig.PID[YAW_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[YAW_PID].windupGuard = 0.1f;
  // pitch
  eepromConfig.PID[PITCH_PID].P = 0.5f;
  eepromConfig.PID[PITCH_PID].I = 0.05f; // 澶у箙鍑忓皬锛岄槻姝㈢垎鐐?
  eepromConfig.PID[PITCH_PID].D = 0.008f;
  eepromConfig.PID[PITCH_PID].lastDcalcValue = 0.0f;
  eepromConfig.PID[PITCH_PID].lastDterm = 0.0f;
  eepromConfig.PID[PITCH_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[PITCH_PID].windupGuard = 0.5236f;

  // roll
  eepromConfig.PID[ROLL_PID].P = 0.03f; // 璺?Pitch 缁欎竴鏍风殑鍊间綔涓鸿捣姝?
  eepromConfig.PID[ROLL_PID].I = 0.0f;
  eepromConfig.PID[ROLL_PID].D = 0.008f;
  eepromConfig.PID[ROLL_PID].lastDcalcValue = 0.0f;
  eepromConfig.PID[ROLL_PID].lastDterm = 0.0f;
  eepromConfig.PID[ROLL_PID].lastLastDterm = 0.0f;
  eepromConfig.PID[ROLL_PID].windupGuard = 0.5236f;

  systemReady = true;
  /* if (MPU6050_Init(ACCEL_FS_4G, GYRO_FS_500DPS, DLPF_CFG_42HZ)) {
         mpu_ok = true;
         HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET); // 鎴愬姛锛氫寒鐏?(娉ㄦ剰妫€鏌ヤ綘鐨勫畯瀹氫箟鏄?LED1_GPIO 杩樻槸 LED1_GPIO_Port)
         printf(">>> MPU6050 Init SUCCESS! Ready to read data.\r\n");
     } else {
         mpu_ok = false;
         HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_RESET); // 澶辫触锛氱伃鐏?
         printf(">>> ERROR: MPU6050 Init FAILED! Check wiring (I2C SDA/SCL) and power.\r\n");
     }*/

  // ======= 闃叉绗竴鍦?dt 鐖嗙偢寮曞彂 NaN =======
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


      dt500Hz = (float)deltaTime500Hz / 1000000.0f;

      MPU6050_Read_And_Process();

      sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
      sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
      // 鍘熸潵鏄湁璐熷彿鐨勶紝浣嗘槸搴旇鏄湪MPU6050_Read_And_Process()璋冩暣浜嗚酱鏂瑰悜锛屾墍浠ュ幓鎺夌鍙蜂繚璇佸悜涓嬪姞閫熷害锛岄伩鍏嶅弽閲嶅姏
      sensors.accel500Hz[ZAXIS] = ((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;
      sensors.gyro500Hz[ROLL] = ((float)rawGyro[ROLL].value - gyroRTBias[ROLL] - gyroTCBias[ROLL]) * GYRO_SCALE_FACTOR;
      sensors.gyro500Hz[PITCH] = ((float)rawGyro[PITCH].value - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
      sensors.gyro500Hz[YAW] = ((float)rawGyro[YAW].value - gyroRTBias[YAW] - gyroTCBias[YAW]) * GYRO_SCALE_FACTOR;

      MargAHRSupdate(sensors.gyro500Hz[ROLL],
                     sensors.gyro500Hz[PITCH],
                     sensors.gyro500Hz[YAW],
                     sensors.accel500Hz[XAXIS],
                     sensors.accel500Hz[YAXIS],
                     sensors.accel500Hz[ZAXIS],
                     0, 0, 0, false, dt500Hz);

      // =============== 銆愭柊澧炪€戠瓑寰?AHRS 鏀舵暃 ================
      if (startup_delay_counter < 1000) // 1000 * 2ms = 2绉?
      {
        startup_delay_counter++;
        if (startup_delay_counter == 1000)
        {
          zeroPIDintegralError();           // 娓呯┖杩?绉掑唴涔辩畻鐨勭Н鍒?
          zeroPIDstates();                  // 娓呯┖D椤瑰井鍒嗙殑姣涘埡
          eepromConfig.pitchEnabled = true; // 濮挎€佺ǔ瀹氾紝鏀惧紑PID鎺у埗
          eepromConfig.rollEnabled = false;
          eepromConfig.yawEnabled = false;
          printf(">>> AHRS鏀舵暃瀹屾垚锛岀數鏈轰娇鑳斤紒\r\n");
        }
      }


      computeMotorCommands(dt500Hz);

      executionTime500Hz = micros() - currentTime;
    }

    if (HAL_GetTick() - last_print_tick >= 100) // 100ms = 10Hz
    {
      last_print_tick = HAL_GetTick(); // 閲嶇疆鏃堕棿鎴?

      if (systemReady)
      {
    	  printf("%.6f,%.6f\r\n",
			 sensors.margAttitude500Hz[ROLL],
			 pidCmd[ROLL]);
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
