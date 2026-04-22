/*
 * evvgcCF.c
 *
 *  Created on: Mar 26, 2026
 *      Author: lenovo
 */

#include "evvgcCF.h"

// 仅保留 PITCH 平滑角度数组
float accAngleSmooth[3];

///////////////////////////////////////////////////////////////////////////////
// @brief  初始化姿态（开机获取水平初始角度）
// @note   仅初始化 PITCH，ROLL/YAW 全部注释
///////////////////////////////////////////////////////////////////////////////
void initOrientation()
{
    int initLoops = 150;
    float accAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };
    int i;

    for (i = 0; i < initLoops; i++)
    {
        // ========== HAL 库 MPU6050 读取 ==========
    	MPU6050_Read_And_Process();      // 读取加速度、陀螺仪原始值（HAL 库驱动）
    	MPU6050_ComputeTemError();  // 计算温度补偿（保留，PITCH 需要）

        // 加速度计数据转换（单位：g）
        sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[ZAXIS] = -((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

        // ==============================================
        // 🔥 仅计算 PITCH 俯仰角（ROLL 已注释）
        // ==============================================
        accAngle[ROLL]  += atan2f(-sensors.accel500Hz[YAXIS], -sensors.accel500Hz[ZAXIS]);
        accAngle[PITCH] += atan2f(sensors.accel500Hz[XAXIS], -sensors.accel500Hz[ZAXIS]);

        // 求平均值（仅 PITCH）
        accAngleSmooth[ROLL ] = accAngle[ROLL ] / (float)initLoops;
        accAngleSmooth[PITCH] = accAngle[PITCH] / (float)initLoops;

        HAL_Delay(2);  // 👈 替换为 HAL 库延时
    }

    // ==============================================
    // 🔥 仅初始化 PITCH 姿态
    // ==============================================

    // 这里有问题，直接把一开始的位置设为0了，但开始的时候样子是千奇百怪的，要找到他确定的位置
    sensors.evvgcCFAttitude500Hz[PITCH] = accAngleSmooth[PITCH];
    sensors.evvgcCFAttitude500Hz[ROLL ] = accAngleSmooth[ROLL ];
    // sensors.evvgcCFAttitude500Hz[YAW  ] = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////
// @brief  互补滤波核心：只解算 PITCH 俯仰角
// @param  *smoothAcc: 加速度平滑角度
// @param  *orient:    最终输出姿态角
// @param  *accData:   加速度数据
// @param  *gyroData:  陀螺仪数据
// @param  dt:         时间间隔
///////////////////////////////////////////////////////////////////////////////
void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
    float accAngle[3];
    float gyroRate[3];

    // ==============================================
    // 🔥 加速度计 → 仅计算 PITCH
    // ==============================================
    // accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
    accAngle[PITCH] = atan2f(accData[XAXIS], -accData[ZAXIS]);

    // ==============================================
    // 🔥 加速度计角度低通滤波 → 仅 PITCH
    // ==============================================
    // smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
    smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;

    // ==============================================
    // 🔥 仅使用 PITCH 陀螺仪，转化为弧度
    // ==============================================
    gyroRate[PITCH] =  gyroData[PITCH] * 0.0174532925f;

    // ==============================================
    // 🔥 互补滤波核心（PITCH 专用）
    // 短期靠陀螺仪积分，长期靠加速度计校正
    // ==============================================
    orient[PITCH] = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);

    // ==============================================
    // ❌ 以下 ROLL / YAW 代码全部注释（不需要）
    // ==============================================
    /*
    gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);
    orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);
    gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);
    orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);
    */
}
