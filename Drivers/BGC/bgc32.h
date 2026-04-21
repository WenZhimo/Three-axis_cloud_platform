/*
 * bgc32.h
 *
 *  Created on: Mar 24, 2026
 *      Author: lenovo
 */

#ifndef BGC_BGC32_H_
#define BGC_BGC32_H_

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// 包含顺序：先PID → 再mpu6050（因为mpu6050需要PID）
#include "pid.h"
#include "mpu6050.h"

///////////////////////////////////////////////////////////////////////////////
// PID Definitions
///////////////////////////////////////////////////////////////////////////////

#define NUMBER_OF_PIDS 3

#define ROLL_PID  0
#define PITCH_PID 1
#define YAW_PID   2

#define     PI 3.14159265f
#define TWO_PI 6.28318531f

#define D2R  (PI / 180.0f)

#define R2D  (180.0f / PI)

#define SQR(x)  ((x) * (x))

extern float   testPhase;
extern float   testPhaseDelta;

#define NUMAXIS  3

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000

typedef union
{
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union
{
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

typedef struct sensors_t
{
	// 加速度计数据
    float accel500Hz[3];
    // 互补滤波姿态角
    float evvgcCFAttitude500Hz[3];
    //卡尔曼姿态角
    float margAttitude500Hz[3];
    // 陀螺仪数据
    float gyro500Hz[3];
    // 磁力计数据毫无作用
    float mag10Hz[3];

} sensors_t;

extern sensors_t sensors;



///////////////////////////////////////////////////////////////////////////////
// MPU6000 DLPF Configurations
///////////////////////////////////////////////////////////////////////////////

enum { DLPF_256HZ, DLPF_188HZ, DLPF_98HZ, DLPF_42HZ };

//float accelOneG = 9.8065;//原本应放在mpu6050里的，注意！！！因为他没写完所以暂时放在这里

#endif /* BGC_BGC32_H_ */
