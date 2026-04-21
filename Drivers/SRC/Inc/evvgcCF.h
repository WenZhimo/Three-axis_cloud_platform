/*
 * evvgcCF.h
 *
 *  Created on: Mar 26, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_EVVGCCF_H_
#define SRC_INC_EVVGCCF_H_

#include "main.h"
#include "mpu6050.h"        // 你的传感器驱动
#include "bgc32.h"          // 云台定义

#pragma once

///////////////////////////////////////////////////////////////////////////////

extern float accAngleSmooth[3];

///////////////////////////////////////////////////////////////////////////////

//void initOrientation(void);
void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt);

#endif /* SRC_INC_EVVGCCF_H_ */
