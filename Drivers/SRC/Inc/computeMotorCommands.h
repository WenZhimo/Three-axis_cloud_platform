/*
 * computeMotorCommands.h
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_COMPUTEMOTORCOMMANDS_H_
#define SRC_INC_COMPUTEMOTORCOMMANDS_H_

#include "main.h"
#include "pid.h"             // 必须加！否则 updatePID 找不到
#include "bgc32.h"           // 必须加！否则传感器结构体找不到
#include "mpu6050.h"         // 必须加！否则 pointingCmd / holdIntegrators 找不到


#pragma once

//==================== 全局变量声明 ====================
// 机械角度 → 电机电角度转换系数（ROLL/PITCH/YAW）
extern float mechanical2electricalDegrees[3];
// 电机电角度 → 机械角度转换系数
extern float electrical2mechanicalDegrees[3];

// 三轴 PID 最终输出控制量（给电机驱动）
extern float pidCmd[3];

extern float outputRate[3];

//==================== 函数声明 ====================
/**
 * @brief  电机控制核心函数：计算三轴电机输出
 * @param  dt: 距离上次调用的时间间隔（单位：秒）
 * @retval 无
 */
void computeMotorCommands(float dt);

#endif
