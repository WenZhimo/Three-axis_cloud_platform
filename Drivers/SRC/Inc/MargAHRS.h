/*
 * MargAHRS.h
 *
 *  Created on: Mar 24, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_MARGAHRS_H_
#define SRC_INC_MARGAHRS_H_

#pragma once

#include "main.h"
#include <math.h>
#include "mpu6050.h"
#include "bgc32.h"

//----------------------------------------------------------------------------------------------------
// 宏定义
#define ROLL    0
#define PITCH   1
#define YAW     2

//----------------------------------------------------------------------------------------------------
// 外部变量声明
extern float accConfidenceDecay;

extern float q0, q1, q2, q3;
extern float q0q0, q0q1, q0q2, q0q3;
extern float q1q1, q1q2, q1q3;
extern float q2q2, q2q3;
extern float q3q3;

extern float kpAcc, kiAcc;

//----------------------------------------------------------------------------------------------------
// 函数声明
void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt);



#endif /* SRC_INC_MARGAHRS_H_ */

