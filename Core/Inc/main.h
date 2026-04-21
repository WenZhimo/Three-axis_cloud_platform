/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "math.h"  // 新增：引入HAL库GPIO/延时接口（根据你的工程调整）
#include "gpio.h"
#include "drv_pwmMotors.h"
#include "i2c.h"
#include "bgc32.h"
#include "evvgcCF.h"
#include "firstOrderFilter.h"
#include "MargAHRS.h"
#include "computeMotorCommands.h"

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
uint32_t micros(void);

extern uint32_t sysTickUptime;
extern uint32_t sysTickCycleCounter;
extern bool systemReady;
extern uint16_t frameCounter;
extern uint32_t deltaTime1000Hz;
extern uint32_t previous1000HzTime;
extern uint32_t executionTime1000Hz;

extern uint32_t deltaTime500Hz;
extern uint32_t previous500HzTime;
extern uint32_t executionTime500Hz;
extern bool frame_500Hz;

extern float dt500Hz;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define FRAME_COUNT   1000
#define COUNT_500HZ   2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
