/*
 * mpu6050Calibration.c
 *
 *  Created on: Apr 1, 2026
 *      Author: lenovo
 */
#include "main.h"


void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;          // 记录当前计数器值
    uint32_t ticks = us * (SystemCoreClock / 1000000); // 换算：微秒 → 周期
    while ((DWT->CYCCNT - start) < ticks); // 等待计数器差值达到目标
}

void mpu6050Calibration(void)
{
	for(int i = 0; i < 3; i++)
	{
		eepromConfig.accelTCBiasSlope[i] = 0.0f;
		eepromConfig.accelTCBiasIntercept[i] = 0.0f;
		eepromConfig.gyroTCBiasSlope[i] = 0.0f;
		eepromConfig.gyroTCBiasIntercept[i] = 0.0f;
	}
	uint16_t sampleRate      = 1000;
	uint16_t numberOfSamples = 2000;

	float accelBias1[3]       = { 0.0f, 0.0f, 0.0f };
	float gyroBias1[3]        = { 0.0f, 0.0f, 0.0f };
	float mpu6050Temperature1 = 0.0f;

	float accelBias2[3]       = { 0.0f, 0.0f, 0.0f };
	float gyroBias2[3]        = { 0.0f, 0.0f, 0.0f };
	float mpu6050Temperature2 = 0.0f;

	uint16_t index;

	mpu6050Calibrating = true;

	printf("\n>>> MPU6050 温度漂移校准开始...\n");

	///////////////////////////////////
	// 第一阶段：冷机采样 (Temperature 1)
	///////////////////////////////////
	printf("   [1/3] 正在采集冷机数据 (2秒)... \n");

	for (index = 0; index < numberOfSamples; index++)
	{
		MPU6050_Read_And_Process();

		// 注意：这里手动减去 8192 是为了抵消重力吗？
		// 如果你的 Z 轴垂直向上，重力约等于 8192 (4G量程下)。
		// 这样做的目的是让 Z 轴也接近 0，方便计算。
		rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192;

		// 累加数据
		accelBias1[XAXIS]    += rawAccel[XAXIS].value;
		accelBias1[YAXIS]    += rawAccel[YAXIS].value;
		accelBias1[ZAXIS]    += rawAccel[ZAXIS].value;

		gyroBias1[ROLL ]     += rawGyro[ROLL ].value;
		gyroBias1[PITCH]     += rawGyro[PITCH].value;
		gyroBias1[YAW  ]     += rawGyro[YAW  ].value;

		// 累加温度 (先不加偏移，最后再算)
		mpu6050Temperature1  += (float)(rawMPU6050Temperature.value);

		DWT_Delay_us(sampleRate);
	}

	// 【修正点】计算平均值：这里应该除以 Bias1，不是 Bias2！
	// 注意：温度最后再除以次数，避免精度损失
	for(int i=0; i<3; i++) {
		accelBias1[i] /= (float)numberOfSamples;
		gyroBias1[i]  /= (float)numberOfSamples;
	}
	mpu6050Temperature1 = mpu6050Temperature1 / (float)numberOfSamples / 340.0f + 35.0f; // 换算成摄氏度

	printf("   -> 冷机温度: %.2f°C\n", mpu6050Temperature1);

	///////////////////////////////////
	// 等待升温
	///////////////////////////////////
	printf("   [2/3] 等待传感器升温 (2分钟，请勿触碰)... \n");
	HAL_Delay(10000); // 120秒

	///////////////////////////////////
	// 第二阶段：热机采样 (Temperature 2)
	///////////////////////////////////
	printf("   [3/3] 正在采集热机数据 (2秒)... \n");

	for (index = 0; index < numberOfSamples; index++)
	{
		MPU6050_Read_And_Process();

		rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192; // 同样减去重力

		accelBias2[XAXIS]    += rawAccel[XAXIS].value;
		accelBias2[YAXIS]    += rawAccel[YAXIS].value;
		accelBias2[ZAXIS]    += rawAccel[ZAXIS].value;

		gyroBias2[ROLL ]     += rawGyro[ROLL ].value;
		gyroBias2[PITCH]     += rawGyro[PITCH].value;
		gyroBias2[YAW  ]     += rawGyro[YAW  ].value;

		mpu6050Temperature2  += (float)(rawMPU6050Temperature.value);

		DWT_Delay_us(sampleRate);
	}

	// 计算第二阶段平均值
	for(int i=0; i<3; i++) {
		accelBias2[i] /= (float)numberOfSamples;
		gyroBias2[i]  /= (float)numberOfSamples;
	}
	mpu6050Temperature2 = mpu6050Temperature2 / (float)numberOfSamples / 340.0f + 35.0f;

	printf("   -> 热机温度: %.2f°C\n", mpu6050Temperature2);

	///////////////////////////////////
	// 计算斜率和截距 (线性拟合 y = kx + b)
	///////////////////////////////////
	float tempDiff = mpu6050Temperature2 - mpu6050Temperature1;

	// 防止除以 0 (如果温度没变化)
	if (tempDiff > 0.5f)
	{
		// 加速度计斜率
		eepromConfig.accelTCBiasSlope[XAXIS]     = (accelBias2[XAXIS] - accelBias1[XAXIS]) / tempDiff;
		eepromConfig.accelTCBiasSlope[YAXIS]     = (accelBias2[YAXIS] - accelBias1[YAXIS]) / tempDiff;
		eepromConfig.accelTCBiasSlope[ZAXIS]     = (accelBias2[ZAXIS] - accelBias1[ZAXIS]) / tempDiff;

		// 加速度计截距 (b = y - kx)
		eepromConfig.accelTCBiasIntercept[XAXIS] = accelBias2[XAXIS] - eepromConfig.accelTCBiasSlope[XAXIS] * mpu6050Temperature2;
		eepromConfig.accelTCBiasIntercept[YAXIS] = accelBias2[YAXIS] - eepromConfig.accelTCBiasSlope[YAXIS] * mpu6050Temperature2;
		eepromConfig.accelTCBiasIntercept[ZAXIS] = accelBias2[ZAXIS] - eepromConfig.accelTCBiasSlope[ZAXIS] * mpu6050Temperature2;

		// 陀螺仪斜率
		eepromConfig.gyroTCBiasSlope[ROLL ]      = (gyroBias2[ROLL ] - gyroBias1[ROLL ]) / tempDiff;
		eepromConfig.gyroTCBiasSlope[PITCH]      = (gyroBias2[PITCH] - gyroBias1[PITCH]) / tempDiff;
		eepromConfig.gyroTCBiasSlope[YAW  ]      = (gyroBias2[YAW  ] - gyroBias1[YAW  ]) / tempDiff;

		// 陀螺仪截距
		eepromConfig.gyroTCBiasIntercept[ROLL ]  = gyroBias2[ROLL ] - eepromConfig.gyroTCBiasSlope[ROLL ] * mpu6050Temperature2;
		eepromConfig.gyroTCBiasIntercept[PITCH]  = gyroBias2[PITCH] - eepromConfig.gyroTCBiasSlope[PITCH] * mpu6050Temperature2;
		eepromConfig.gyroTCBiasIntercept[YAW  ]  = gyroBias2[YAW  ] - eepromConfig.gyroTCBiasSlope[YAW  ] * mpu6050Temperature2;

		printf("\n>>> 温度校准计算完成！\n");
	}
	else
	{
		printf("\n>>> 警告：温差过小 (%.2f)，校准可能无效！\n", tempDiff);
	}

	mpu6050Calibrating = false;



}

