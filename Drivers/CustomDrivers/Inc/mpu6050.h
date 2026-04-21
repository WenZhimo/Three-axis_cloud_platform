/*
 * mpu6050.h
 *
 *  Created on: Mar 16, 2026
 *      Author: Administrator
 */

#ifndef CUSTOMDRIVERS_INC_MPU6050_H_
#define CUSTOMDRIVERS_INC_MPU6050_H_

#include <stdint.h>    // 添加这个
#include <stdbool.h>   // 添加这个
#include "pid.h"
#include "mpu6050Calibration.h"

// 轴定义 (保持与你原代码一致)
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

// 飞控常用轴定义 (保持与你原代码一致)
#define ROLL  0
#define PITCH 1
#define YAW   2


// 1. 基础定义
#define MPU6050_I2C_ADDR        (0x68<<1)    // I2C 设备地址 (AD0 接地)
// #define MPU6050_I2C_ADDR      0x69    // 如果 AD0 接 VCC，则使用此地址

#define MPU_WHO_AM_I_VAL        0x68    // 正常的 ID 返回值
#define REG_WHO_AM_I            0x75    // 设备 ID 寄存器



// 2. 寄存器地址映射 (Register Map)

#define REG_SMPLRT_DIV          0x19    // 采样率分频器
#define REG_CONFIG              0x1A    // 配置寄存器 (DLPF 滤波器)
#define REG_GYRO_CONFIG         0x1B    // 陀螺仪配置 (量程)
#define REG_ACCEL_CONFIG        0x1C    // 加速度计配置 (量程)
#define REG_PWR_MGMT_1          0x6B    // 电源管理 1 (唤醒与时钟)
#define REG_PWR_MGMT_2          0x6C    // 电源管理 2 (休眠控制)


// 数据输出起始地址
#define REG_ACCEL_XOUT_H        0x3B    // 加速度计 X 轴高 8 位
#define REG_TEMP_OUT_H          0x41    // 温度高 8 位
#define REG_GYRO_XOUT_H         0x43    // 陀螺仪 X 轴高 8 位

// 连续读取总字节数: 6(ACC) + 2(Temp) + 6(Gyro) = 14 Bytes
#define MPU_DATA_LENGTH         14


// 3. 寄存器配置值 (按 Bit 位逻辑定义)

// --- [REG_SMPLRT_DIV] ---
#define SMPLRT_DIV_100HZ        9
#define SMPLRT_DIV_200HZ        4
#define SMPLRT_DIV_500HZ        1


// --- [REG_PWR_MGMT_1] 0x6B: 电源与时钟 ---
// Bit 7: DEVICE_RESET (0:正常)
// Bit 6: SLEEP         (0:唤醒, 1:睡眠)  <-- 必须为 0
// Bit 5: CYCLE         (0:正常)
// Bit 4: TEMP_DIS      (0:开启温度)
// Bit 3-0: CLKSEL      (0011: Z 轴陀螺仪时钟，最稳定)
#define PWR1_WAKEUP_Z_GYRO      0x03    // 二进制: 0000 0011

// --- [REG_SMPLRT_DIV] 0x19: 采样率 ---
// 公式: Sample Rate = Gyro_Output_Rate / (1 + DIV)
// 当 DLPF 启用时，Gyro_Output_Rate = 1kHz
// 目标 100Hz -> 1000 / (1 + 9) = 100Hz
#define SMPLRT_DIV_100HZ        9       // 写入值: 9
#define SMPLRT_DIV_200HZ        4       // 写入值: 4 (1000/5 = 200Hz)
#define SMPLRT_DIV_500HZ        1       // 写入值: 1 (1000/2 = 500Hz)

// --- [REG_CONFIG] 0x1A: 数字低通滤波器 (DLPF) ---
// Bit 7: EXT_SYNC_SET (0: 禁用外部同步)
// Bit 6-4: DLPF_CFG (滤波器设置)
// Bit 3-0: 保留
// 注意: DLPF 非 0/7 时，内部频率锁定为 1kHz；否则为 8kHz
#define DLPF_CFG_260HZ          0x00    // 260Hz (带宽), 延迟 0ms
#define DLPF_CFG_184HZ          0x01    // 184Hz, 延迟 2.0ms
#define DLPF_CFG_98HZ           0x02    // 98Hz,  延迟 3.0ms
#define DLPF_CFG_42HZ           0x03    // 42Hz,  延迟 4.9ms (云台推荐：平滑度与延迟的平衡)
#define DLPF_CFG_20HZ           0x04    // 20Hz,  延迟 8.3ms (极平滑，但延迟大)
#define DLPF_CFG_10HZ           0x05    // 10Hz,  延迟 13.4ms
#define DLPF_CFG_5HZ            0x06    // 5Hz,   延迟 18.6ms
#define DLPF_CFG_BYPASS         0x07    // 旁路模式 (内部频率变 8kHz)


// --- [REG_GYRO_CONFIG] 0x1B: 陀螺仪量程 ---
// Bit 7-5: 保留
// Bit 4-3: FS_SEL (满量程选择)
// Bit 2: FCHOICE_B (0: 启用 DLPF, 1: 旁路) -> 通常设为 0 以配合 CONFIG 寄存器
// Bit 1-0: 保留
#define GYRO_FS_250DPS          0x00    // 0000 0000 -> ±250 °/s  (灵敏度 ~131 LSB/°/s)
#define GYRO_FS_500DPS          0x08    // 0000 1000 -> ±500 °/s  (灵敏度 ~65.5 LSB/°/s) [云台推荐]
#define GYRO_FS_1000DPS         0x10    // 0001 0000 -> ±1000 °/s (灵敏度 ~32.8 LSB/°/s)
#define GYRO_FS_2000DPS         0x18    // 0001 1000 -> ±2000 °/s (灵敏度 ~16.4 LSB/°/s) [穿越机推荐]

// --- [REG_ACCEL_CONFIG] 0x1C: 加速度计量程 ---
// Bit 7-5: 保留
// Bit 4-3: AFS_SEL (满量程选择)
// Bit 2-0: 保留
#define ACCEL_FS_2G             0x00    // 0000 0000 -> ±2g  (灵敏度 ~16384 LSB/g)
#define ACCEL_FS_4G             0x08    // 0000 1000 -> ±4g  (灵敏度 ~8192 LSB/g)  [云台推荐]
#define ACCEL_FS_8G             0x10    // 0001 0000 -> ±8g  (灵敏度 ~4096 LSB/g)
#define ACCEL_FS_16G            0x18    // 0001 1000 -> ±16g (灵敏度 ~2048 LSB/g)


// 4. 灵敏度宏定义 (方便计算物理量)
// ============================================================================
// 陀螺仪灵敏度 (LSB per degree per second)
#define GYRO_SENS_250           131.0f
#define GYRO_SENS_500           65.5f
#define GYRO_SENS_1000          32.8f
#define GYRO_SENS_2000          16.4f

// 加速度计灵敏度 (LSB per g)
#define ACCEL_SENS_2G           16384.0f
#define ACCEL_SENS_4G           8192.0f
#define ACCEL_SENS_8G           4096.0f
#define ACCEL_SENS_16G          2048.0f

// 温度灵敏度 (通常固定)
#define TEMP_SENS               340.0f  // LSB/°C
#define TEMP_ROOM_OFFSET        36.53f  // 35°C 时的偏移量 (根据手册公式)

#define ACCEL_RAWZ 9.8065

#define ACCEL_SCALE_FACTOR 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00026646f  // (1/65.5) * pi/180   (65.5 LSB = 1 DPS)


// 1. 定义联合体 (大小自动匹配，无需担心)
typedef union {
    int16_t value;
    uint8_t bytes[2];
} Int16_Converter_t;

typedef struct eepromConfig_t
{
    uint8_t version;

    float accelTCBiasSlope[3];
    float accelTCBiasIntercept[3];

    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    float magBias[3];

    float accelCutoff;

    float KpAcc;

    float KiAcc;

    float KpMag;

    float KiMag;

    uint8_t dlpfSetting;

    float midCommand;

    PIDdata_t PID[3];

    float rollPower;
    float pitchPower;
    float yawPower;

    uint8_t rollEnabled;
    uint8_t pitchEnabled;
    uint8_t yawEnabled;

    uint8_t rollAutoPanEnabled;
    uint8_t pitchAutoPanEnabled;
    uint8_t yawAutoPanEnabled;

    uint8_t imuOrientation;

    float   rollMotorPoles;
    float   pitchMotorPoles;
    float   yawMotorPoles;

    float   rateLimit;

    uint8_t rollRateCmdInput;
    uint8_t pitchRateCmdInput;
    uint8_t yawRateCmdInput;

    float   gimbalRollRate;
    float   gimbalPitchRate;
    float   gimbalYawRate;

    float   gimbalRollLeftLimit;
    float   gimbalRollRightLimit;
    float   gimbalPitchDownLimit;
    float   gimbalPitchUpLimit;
    float   gimbalYawLeftLimit;
    float   gimbalYawRightLimit;

    float   accelX500HzLowPassTau;
    float   accelY500HzLowPassTau;
    float   accelZ500HzLowPassTau;

    float   rollRatePointingCmd50HzLowPassTau;
    float   pitchRatePointingCmd50HzLowPassTau;
    float   yawRatePointingCmd50HzLowPassTau;

    float   rollAttPointingCmd50HzLowPassTau;
    float   pitchAttPointingCmd50HzLowPassTau;
    float   yawAttPointingCmd50HzLowPassTau;

} eepromConfig_t;

extern eepromConfig_t eepromConfig;
// 应该这样：
extern int16_t orientationMatrix[9];
// 2. 全局数据变量声明 (模拟原代码的全局数组)
// 外部需要在 .c 文件或 main.c 中定义这些数组
extern Int16_Converter_t rawAccel[3];       // [XAXIS, YAXIS, ZAXIS]
extern Int16_Converter_t rawGyro[3];        // [ROLL, PITCH, YAW]
extern Int16_Converter_t rawMPU6050Temperature;

extern float accelTCBias[3];
extern float gyroTCBias[3];

extern float accelOneG;//放到bgc32.h里了

// 增加这一行，用来存加速度计的静态零偏
extern float accelRTBias[3];
extern float gyroRTBias[3];

extern uint8_t mpu6050Calibrating;

extern float pointingCmd[3];
extern float mpu6050Temperature;
extern int16_t accelData500Hz[3];
extern int16_t gyroData500Hz[3];

// 初始化 MPU6050
// 参数: accel_fs (如 ACCEL_FS_4G), gyro_fs (如 GYRO_FS_500DPS), dlpf (如 DLPF_CFG_42HZ)
// 返回: true=成功 (ID 匹配), false=失败
bool MPU6050_Init(uint8_t accel_fs, uint8_t gyro_fs, uint8_t dlpf);

// 读取原始数据
void MPU6050_Read_And_Process(void);

// 读取并转换为物理量数据
// 需要根据初始化时的量程来计算，所以通常需要知道当前量程，或者在结构体中保存量程
// 简单做法：在 Init 时设置全局变量，或者传入灵敏度系数
void MPU6050_Get_Physical_Data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

void MPU6050_Send_VOFA_Plus_Float(float ax, float ay, float az, float gx, float gy, float gz, float temp);

void matrixMultiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, int16_t matrixC[], int16_t matrixA[], int16_t matrixB[]);

void orientIMU(void);

//  lyl
void MPU6050_ComputeStaticError();

void MPU6050_ComputeTemError();


#endif /* CUSTOMDRIVERS_INC_MPU6050_H_ */
