#ifndef DRV_PWMMOTORS_H
#define DRV_PWMMOTORS_H

#include "main.h"
#include <math.h>

/* --- 参数定义 --- */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define SIN_TABLE_SIZE 720
#define SIN_SCALE      32767

/* --- 枚举：定义三个电机轴 --- */
typedef enum {

	MOTOR_PITCH = 0, // 对应 Mot0
	MOTOR_ROLL ,   // 对应 Mot1
    MOTOR_YAW  ,    // 对应 Mot2
} MotorAxis_t;

/* --- 外部变量引用：三个核心定时器 --- */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* --- 驱动函数声明 --- */
HAL_StatusTypeDef PWM_Motor_Init(void);
void PWM_Motor_SetAngle(MotorAxis_t axis, float angle_rad, float power_percent);
void PWM_Motor_TestAllAngles(void);
void PWM_Motor_Hardware_Diagnosis(void);

#endif /* DRV_PWMMOTORS_H */
