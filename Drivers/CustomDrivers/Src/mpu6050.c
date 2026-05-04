/*
 * mpu6050.c
 *
 *  Created on: Mar 24, 2026
 *      Author: lenovo
 */


#include "mpu6050.h"
#include "main.h"       // 补充这个：为了拿到 HAL 库的各种定义和 hi2c2 句柄
#include "bgc32.h"      // 补充这个：为了拿到 SQR 宏定义
#include <string.h>     // 补充这个：为了使用 memset
#include <math.h>       // 补充这个：为了使用 sqrt

float accelOneG = 9.8065;
// 在 mpu6050.c 中添加实体定义
float accelTCBias[3] = {1.0f, 0.0f, 0.0f};
float gyroTCBias[3]  = {0.0f, 0.0f, 0.0f};
float accelRTBias[3] = {0.0f, 0.0f, 0.0f};
float gyroRTBias[3]  = {0.0f, 0.0f, 0.0f};
int16_t accelData500Hz[3];
int16_t gyroData500Hz[3];
uint8_t mpu6050Calibrating = false;
// 外部引用 I2C 句柄

extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef huart3;

// 内部静态变量，用于记录当前初始化的灵敏度
static float g_accel_sens = ACCEL_SENS_2G;
static float g_gyro_sens = GYRO_SENS_250;

// 这里定义实际的内存空间
Int16_Converter_t rawAccel[3];        // 0:X, 1:Y, 2:Z
Int16_Converter_t rawGyro[3];         // 0:Roll, 1:Pitch, 2:Yaw
Int16_Converter_t rawMPU6050Temperature;

// 安装误差旋转矩阵 (3x3)
// 如果没有安装误差，可以是单位矩阵 {{1,0,0}, {0,1,0}, {0,0,1}}
// 如果像原代码那样交换了XY，这里需要配合调整，或者在读取阶段处理
int16_t orientationMatrix[9];

// 简单的 3x3 矩阵 * 3x1 向量 乘法
void matrixMultiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, int16_t matrixC[], int16_t matrixA[], int16_t matrixB[])
{
    uint8_t i, j, k;

    for (i = 0; i < aRows * bCols; i++)
    {
        matrixC[i] = 0.0;
    }

    for (i = 0; i < aRows; i++)
    {
        for (j = 0; j < aCols_bRows; j++)
        {
            for (k = 0;  k < bCols; k++)
            {
                matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
            }
        }
    }
}

/**
 * @brief 底层 I2C 写寄存器函数
 */
static HAL_StatusTypeDef MPU_Write_Reg(uint8_t reg_addr, uint8_t data) {
    // 使用 hi2c2 (根据你的 i2c.h 定义)
    return HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief 底层 I2C 读寄存器函数
 */
static HAL_StatusTypeDef MPU_Read_Reg(uint8_t reg_addr, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief 初始化 MPU6050
 */
bool MPU6050_Init(uint8_t accel_fs, uint8_t gyro_fs, uint8_t dlpf) {
	 uint8_t check_id = 0;
	    HAL_StatusTypeDef status;
	    int retry_count = 0;

	    // 上电后多等一会儿，确保 MPU6050 内部稳压稳定
	    HAL_Delay(200);
	    printf(">>> Starting MPU6050 Init (Addr 0x68)...\r\n");

	    // --- 1. 检查设备 ID (增加重试机制) ---
	    while (retry_count < 3) {
	        status = MPU_Read_Reg(REG_WHO_AM_I, &check_id, 1);

	        if (status == HAL_OK && check_id == 0x68) {
	            break; // 成功，跳出循环
	        }

	        printf("    [Retry %d] Read ID Failed. Status: %d, Read Val: 0x%02X\r\n",
	               retry_count + 1, status, check_id);

	        retry_count++;
	        HAL_Delay(50); // 重试前延时
	    }

	    // 如果重试 3 次还是失败，直接报错并返回
	    if (status != HAL_OK || check_id != 0x68) {
	        printf("!!! CRITICAL ERROR: MPU6050 Not Found!\r\n");
	        printf("    - Last Status: %d (1=NoAck, 2=Error, 3=Timeout, 4=Busy)\r\n", status);
	        printf("    - Expected ID: 0x68, Got: 0x%02X\r\n", check_id);
	        printf("    - Action: Check SDA/SCL wiring, Power (3.3V), and Pull-up Resistors.\r\n");
	        printf("    - Note: If Status is 1, the chip did not respond at all. Check pins!\r\n");
	        return false;
	    }

	    printf("    [OK] Device ID Verified: 0x%02X\r\n", check_id);
    // 2. 唤醒设备
    status = MPU_Write_Reg(REG_PWR_MGMT_1, PWR1_WAKEUP_Z_GYRO);
    if (status != HAL_OK) return false;
    HAL_Delay(10);

    // 3. 配置采样率
    status = MPU_Write_Reg(REG_SMPLRT_DIV, SMPLRT_DIV_100HZ);
    if (status != HAL_OK) return false;

    // 4. 配置数字低通滤波器
    status = MPU_Write_Reg(REG_CONFIG, dlpf);
    if (status != HAL_OK) return false;

    // 5. 配置陀螺仪量程
    status = MPU_Write_Reg(REG_GYRO_CONFIG, gyro_fs);
    if (status != HAL_OK) return false;

    // 6. 配置加速度计量程
    status = MPU_Write_Reg(REG_ACCEL_CONFIG, accel_fs);
    if (status != HAL_OK) return false;

    // 7. 根据传入的宏，更新内部灵敏度变量 (直接使用头文件中的宏)
    // 通过掩码 0x18 提取 FS_SEL 位 (Bit 4 和 Bit 3)
    switch (accel_fs & 0x18) {
        case ACCEL_FS_2G:  g_accel_sens = ACCEL_SENS_2G; break;
        case ACCEL_FS_4G:  g_accel_sens = ACCEL_SENS_4G; break;
        case ACCEL_FS_8G:  g_accel_sens = ACCEL_SENS_8G; break;
        case ACCEL_FS_16G: g_accel_sens = ACCEL_SENS_16G; break;
        default:           g_accel_sens = ACCEL_SENS_2G; break;
    }

    switch (gyro_fs & 0x18) {
        case GYRO_FS_250DPS:  g_gyro_sens = GYRO_SENS_250; break;
        case GYRO_FS_500DPS:  g_gyro_sens = GYRO_SENS_500; break;
        case GYRO_FS_1000DPS: g_gyro_sens = GYRO_SENS_1000; break;
        case GYRO_FS_2000DPS: g_gyro_sens = GYRO_SENS_2000; break;
        default:              g_gyro_sens = GYRO_SENS_250; break;
    }


    MPU6050_ComputeStaticError();

    return true;
}

/**
 * @brief 【核心函数】读取原始数据 -> 联合体解析 -> 矩阵旋转
 * 此函数完全复刻了你提供的源码逻辑：
 * 1. 使用 HAL 读取 14 字节
 * 2. 使用联合体交叉赋值 (处理大小端 + 轴映射)
 * 3. 执行矩阵乘法
 * 4. 结果存回 global rawAccel/rawGyro 数组
 */
void MPU6050_Read_And_Process(void) {
    uint8_t axis;
    uint8_t I2C2_Buffer_Rx[14];

    int16_t straightAccelData[3];
	int16_t rotatedAccelData[3];

	int16_t straightGyroData[3];
	int16_t rotatedGyroData[3];

    // 1. HAL 读取
    if (MPU_Read_Reg(REG_ACCEL_XOUT_H, I2C2_Buffer_Rx, MPU_DATA_LENGTH) != HAL_OK) {
        memset(rawAccel, 0, sizeof(rawAccel));
        memset(rawGyro, 0, sizeof(rawGyro));
        return;
    }

    // 2. 联合体解析 (严格保持原代码的轴映射和大小端处理)
    // 注意：这里完全照搬你提供的源码逻辑
    // Buffer[0,1] (物理X) -> 赋给 rawAccel[YAXIS]
    // Buffer[2,3] (物理Y) -> 赋给 rawAccel[XAXIS]
    // 这种映射通常是因为传感器安装旋转了90度

    // --- 加速度计 ---
    rawAccel[XAXIS].bytes[1] = I2C2_Buffer_Rx[0]; // High
    rawAccel[XAXIS].bytes[0] = I2C2_Buffer_Rx[1]; // Low

    rawAccel[YAXIS].bytes[1] = I2C2_Buffer_Rx[2];
    rawAccel[YAXIS].bytes[0] = I2C2_Buffer_Rx[3];

    rawAccel[ZAXIS].bytes[1] = I2C2_Buffer_Rx[4];
    rawAccel[ZAXIS].bytes[0] = I2C2_Buffer_Rx[5];

    // --- 温度 ---
    rawMPU6050Temperature.bytes[1] = I2C2_Buffer_Rx[6];
    rawMPU6050Temperature.bytes[0] = I2C2_Buffer_Rx[7];

	// 严格按照物理轴 X, Y, Z 提取，不要在这里擅自映射为 ROLL/PITCH
	rawGyro[YAXIS].bytes[1] = I2C2_Buffer_Rx[8];
	rawGyro[YAXIS].bytes[0] = I2C2_Buffer_Rx[9];

	rawGyro[XAXIS].bytes[1] = I2C2_Buffer_Rx[10];
	rawGyro[XAXIS].bytes[0] = I2C2_Buffer_Rx[11];

	rawGyro[ZAXIS].bytes[1] = I2C2_Buffer_Rx[12];
	rawGyro[ZAXIS].bytes[0] = I2C2_Buffer_Rx[13];

	straightAccelData[XAXIS] = (float)rawAccel[XAXIS].value;
	straightAccelData[YAXIS] = (float)rawAccel[YAXIS].value;
	straightAccelData[ZAXIS] = (float)rawAccel[ZAXIS].value;

	straightGyroData[XAXIS] = (float)rawGyro[XAXIS].value;
	straightGyroData[YAXIS] = (float)rawGyro[YAXIS].value;
	straightGyroData[ZAXIS] = (float)rawGyro[ZAXIS].value;

    // 4. 矩阵旋转
    matrixMultiply(3, 3, 1, rotatedAccelData, orientationMatrix, straightAccelData);
    matrixMultiply(3, 3, 1, rotatedGyroData, orientationMatrix, straightGyroData);

    // 5. 将旋转后的结果写回联合体 (覆盖原始值)
    for (axis = 0; axis < 3; axis++) {
        rawAccel[axis].value = (int16_t)rotatedAccelData[axis];
        rawGyro[axis].value  = (int16_t)rotatedGyroData[axis];
    }
}

void MPU6050_Get_Physical_Data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    *ax = (float)rawAccel[XAXIS].value / g_accel_sens;
    *ay = (float)rawAccel[YAXIS].value / g_accel_sens;
    *az = (float)rawAccel[ZAXIS].value / g_accel_sens;

    *gx = (float)rawGyro[ROLL].value / g_gyro_sens;
    *gy = (float)rawGyro[PITCH].value / g_gyro_sens;
    *gz = (float)rawGyro[YAW].value / g_gyro_sens;
}

void orientIMU(void)
{
	// 原初默认为4状态
	eepromConfig.imuOrientation = 4;
    switch (eepromConfig.imuOrientation)
    {
        case 1: // Dot Front/Left/Top
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 2: // Dot Front/Right/Top
            orientationMatrix[0] =  0;
            orientationMatrix[1] = -1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 3: // Dot Back/Right/Top
            orientationMatrix[0] = -1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] = -1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 4: // Dot Back/Left/Top
            orientationMatrix[0] =  0;
            orientationMatrix[1] =  1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] = -1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 5: // Dot Front/Left/Bottom
            orientationMatrix[0] =  0;
            orientationMatrix[1] = -1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] = -1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 6: // Dot Front/Right/Bottom
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] = -1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 7: // Dot Back/Right/Bottom
            orientationMatrix[0] =  0;
            orientationMatrix[1] =  1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 8: // Dot Back/Left/Bottom
            orientationMatrix[0] = -1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;
        case 9: // Custom test: rotate frame +90 deg around X axis
			orientationMatrix[0] =  1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  0;
			orientationMatrix[5] = -1;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  1;
			orientationMatrix[8] =  0;
			break;

		case 10:
			orientationMatrix[0] =  1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  0;
			orientationMatrix[5] =  1;
			orientationMatrix[6] =  0;
			orientationMatrix[7] = -1;
			orientationMatrix[8] =  0;
			break;

		case 11: // Custom: 左侧安装，排针朝后
			// 输出 Gimbal X (前方) = 输入 Sensor Y
			orientationMatrix[0] =  0;
			orientationMatrix[1] =  1;
			orientationMatrix[2] =  0;

			// 输出 Gimbal Y (右方) = 输入 Sensor -Z (Z朝左，所以加负号)
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  0;
			orientationMatrix[5] = -1;

			// 输出 Gimbal Z (下方) = 输入 Sensor -X (X朝上，所以加负号)
			orientationMatrix[6] = -1;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  0;
			break;

		case 12: // 左侧安装，排针朝后
			// 云台 X 轴 (横滚 Roll, 向前) = 传感器 Y 轴
			orientationMatrix[0] =  0;
			orientationMatrix[1] =  1;
			orientationMatrix[2] =  0;

			// 云台 Y 轴 (俯仰 Pitch, 向右) = 传感器 -Z 轴 (因为Z朝左)
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  0;
			orientationMatrix[5] = -1;

			// 云台 Z 轴 (重力方向, 向下) = 传感器 -X 轴 (因为X朝上)
			orientationMatrix[6] = -1;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  0;
			break;
		case 13:
			// 云台 X 轴 (原固件认为是 Pitch) = 传感器 -Z 轴
			orientationMatrix[0] =  0;
			orientationMatrix[1] =  0;
			orientationMatrix[2] = -1;

			// 云台 Y 轴 (原固件认为是 Roll) = 传感器 -Y 轴
			orientationMatrix[3] =  0;
			orientationMatrix[4] = -1;
			orientationMatrix[5] =  0;

			// 云台 Z 轴 (重力依然向下) = 传感器 -X 轴
			orientationMatrix[6] = -1;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  0;
			break;

		case 14:
			orientationMatrix[0] =  0;
			orientationMatrix[1] =  1;
			orientationMatrix[2] =  0;

			orientationMatrix[3] = -1;
			orientationMatrix[4] =  0;
			orientationMatrix[5] =  0;

			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] = -1;

        default: // Dot Front/Left/Top
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;
    }
}


/**
 * @brief 简单的自检函数 (可选)
 * @return true: 基本通信正常, false: 通信失败
 * 注意：完整的自检需要运行芯片内置自检程序并比较结果，这里仅做通信检查
 */
void MPU6050_Send_VOFA_Plus_Float(float ax, float ay, float az, float gx, float gy, float gz, float temp) {
    printf("%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, temp);
}

void MPU6050_ComputeStaticError()
{
	uint8_t  axis;
	uint16_t samples;

	double accelSum[3]    = { 0.0f, 0.0f, 0.0f };
	double gyroSum[3]     = { 0.0f, 0.0f, 0.0f };

	for (samples = 0; samples < 5000; samples++)
	{
		MPU6050_Read_And_Process();

		MPU6050_ComputeTemError();
				//矩阵相乘后的结果-温度补偿偏差，每次都对结果进行积分，一共积分5000次
		accelSum[XAXIS] += (float)rawAccel[XAXIS].value - accelTCBias[XAXIS];
		accelSum[YAXIS] += (float)rawAccel[YAXIS].value - accelTCBias[YAXIS];
		accelSum[ZAXIS] += (float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS];

		gyroSum[ROLL ]  += (float)rawGyro[ROLL ].value  - gyroTCBias[ROLL ];
		gyroSum[PITCH]  += (float)rawGyro[PITCH].value  - gyroTCBias[PITCH];
		gyroSum[YAW  ]  += (float)rawGyro[YAW  ].value  - gyroTCBias[YAW  ];

		// HAL里没有延时us的待会在改
		DWT_Delay_us(1000);
	}

	for (axis = 0; axis < 3; axis++)
	{
			/*//对之前积分的5000次加速度数据的结果求平均后然后乘以最小刻度值(1/8192) * 9.8065
		accelSum[axis]   = accelSum[axis] / 5000.0f * ACCEL_SCALE_FACTOR;

			//对之前积分的5000次加速度数据的结果直接求平均
		gyroRTBias[axis] = gyroSum[axis]  / 5000.0f;
		*/
		// 算出平均值后，赋值给全局变量！
		accelRTBias[axis] = accelSum[axis] / 5000.0f * ACCEL_SCALE_FACTOR; // 【修改点】

		gyroRTBias[axis] = gyroSum[axis]  / 5000.0f;
	}

		//加速度标准化
	accelOneG = sqrt(SQR(accelSum[XAXIS]) + SQR(accelSum[YAXIS]) + SQR(accelSum[ZAXIS]));
}

void MPU6050_ComputeTemError()
{
	float temp = ((float)rawMPU6050Temperature.value / 340.0f) + 36.53f;

	//	计算温度补偿偏差值
	accelTCBias[XAXIS] = eepromConfig.accelTCBiasSlope[XAXIS] * temp + eepromConfig.accelTCBiasIntercept[XAXIS];
	accelTCBias[YAXIS] = eepromConfig.accelTCBiasSlope[YAXIS] * temp + eepromConfig.accelTCBiasIntercept[YAXIS];
	accelTCBias[ZAXIS] = eepromConfig.accelTCBiasSlope[ZAXIS] * temp + eepromConfig.accelTCBiasIntercept[ZAXIS];

	gyroTCBias[ROLL ]  = eepromConfig.gyroTCBiasSlope[ROLL ]  * temp + eepromConfig.gyroTCBiasIntercept[ROLL ];
	gyroTCBias[PITCH]  = eepromConfig.gyroTCBiasSlope[PITCH]  * temp + eepromConfig.gyroTCBiasIntercept[PITCH];
	gyroTCBias[YAW  ]  = eepromConfig.gyroTCBiasSlope[YAW  ]  * temp + eepromConfig.gyroTCBiasIntercept[YAW  ];
}
