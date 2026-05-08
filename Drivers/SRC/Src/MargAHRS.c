/*
 * MargAHRS.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */


#include "MargAHRS.h"

#define PITCH_OUTPUT_LIMIT_RAD (85.0f * D2R)
#define GIMBAL_LOCK_COS_THRESH (0.08715574f) // cos(85deg)

//----------------------------------------------------------------------------------------------------
// 全局变量定义
float exAcc    = 0.0f, eyAcc    = 0.0f, ezAcc    = 0.0f;
float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f;

float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f;
float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f;

float kpAcc, kiAcc;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3, q3q3;

float halfT;
uint8_t MargAHRSinitialized = 0;

float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;

// 你提供的约束函数
float constrain(float input, float minValue, float maxValue)
{
    if (input < minValue)
        return minValue;
    else if (input > maxValue)
        return maxValue;
    else
        return input;
}

#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)

static float wrapToPif(float angle)
{
    while (angle > PI)
    {
        angle -= TWO_PI;
    }

    while (angle < -PI)
    {
        angle += TWO_PI;
    }

    return angle;
}

//----------------------------------------------------------------------------------------------------
void calculateAccConfidence(float accMag)
{
    static float accMagP = 1.0f;

    accMag /= accelOneG;

    accMag  = HardFilter(accMagP, accMag);
    accMagP = accMag;

    accConfidence = constrain(1.0f - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);
}

//----------------------------------------------------------------------------------------------------
void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
	// 加入 Roll 轴的初始角度计算
	float initialRoll  = atan2f(-ay, -az);
	float initialPitch = atan2f(ax, -az);

	float cosRoll  = cosf(initialRoll * 0.5f);
	float sinRoll  = sinf(initialRoll * 0.5f);
	float cosPitch = cosf(initialPitch * 0.5f);
	float sinPitch = sinf(initialPitch * 0.5f);

	// 初始化四元数 (假设初始 Yaw 为 0)
	q0 = cosRoll * cosPitch;
	q1 = sinRoll * cosPitch;
	q2 = cosRoll * sinPitch;
	q3 = -sinRoll * sinPitch;

	// 更新四元数乘积
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}

//----------------------------------------------------------------------------------------------------
void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
    float norm, normR;
    float vx, vy, vz;
    float q0i, q1i, q2i, q3i;

    // 清空所有NaN
    if (isnan(gx) || isinf(gx)) gx = 0.0f;
    if (isnan(gy) || isinf(gy)) gy = 0.0f;
    if (isnan(gz) || isinf(gz)) gz = 0.0f;
    if (isnan(ax) || isinf(ax)) ax = 0.0f;
    if (isnan(ay) || isinf(ay)) ay = 0.0f;
    if (isnan(az) || isinf(az)) az = 0.0f;

    if (MargAHRSinitialized == 0)
    {
        MargAHRSinit(ax, ay, az, mx, my, mz);
        MargAHRSinitialized = 1;
    }

    if (MargAHRSinitialized == 1)
    {
        halfT = dt * 0.5f;
        norm = sqrt(ax*ax + ay*ay + az*az);

        if (norm > 0.1f)
        {
            normR = 1.0f / norm;
            ax *= normR;
            ay *= normR;
            az *= normR;

            vx = 2.0f * (q1q3 - q0q2);
            vy = 2.0f * (q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;

            exAcc = vy * az - vz * ay;
            eyAcc = vz * ax - vx * az;
            ezAcc = vx * ay - vy * ax;

            gx += exAcc * eepromConfig.KpAcc;
            gy += eyAcc * eepromConfig.KpAcc;
            gz += ezAcc * eepromConfig.KpAcc;
        }

        // 四元数更新
        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1i = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2i = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3i = ( q0 * gz + q1 * gy - q2 * gx) * halfT;

        q0 += q0i;
        q1 += q1i;
        q2 += q2i;
        q3 += q3i;

        // =============== 归一化防NaN ===============
        float norm_q = q0*q0 + q1*q1 + q2*q2 + q3*q3;

        if (norm_q < 1e-10f || isnan(norm_q))
        {
            q0 = 1.0f;
            q1 = q2 = q3 = 0.0f;
        }
        else
        {
            normR = 1.0f / sqrtf(norm_q);
            q0 *= normR;
            q1 *= normR;
            q2 *= normR;
            q3 *= normR;
        }

        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Quaternion -> Euler output with singular-zone protection.
        {
            static uint8_t eulerInitialized = 0;
            static float rollContinuous = 0.0f;
            static float yawContinuous = 0.0f;

            float sinp = 2.0f * (q1q3 - q0q2);
            float pitch;
            float rollRaw;
            float yawRaw;

            if (sinp > 1.0f)
            {
                sinp = 1.0f;
            }
            if (sinp < -1.0f)
            {
                sinp = -1.0f;
            }

            pitch = -asinf(sinp);
            pitch = constrain(pitch, -PITCH_OUTPUT_LIMIT_RAD, PITCH_OUTPUT_LIMIT_RAD);

            rollRaw = atan2f(
                2.0f * (q0q1 + q2q3),
                q0q0 - q1q1 - q2q2 + q3q3);
            yawRaw = atan2f(
                2.0f * (q0q3 + q1q2),
                q0q0 + q1q1 - q2q2 - q3q3);

            if (eulerInitialized == 0)
            {
                rollContinuous = rollRaw;
                yawContinuous = yawRaw;
                eulerInitialized = 1;
            }
            else
            {
                // Roll becomes non-unique near pitch ~= +/-90deg.
                if (fabsf(cosf(pitch)) >= GIMBAL_LOCK_COS_THRESH)
                {
                    rollContinuous += wrapToPif(rollRaw - rollContinuous);
                }

                yawContinuous += wrapToPif(yawRaw - yawContinuous);
            }

            sensors.margAttitude500Hz[ROLL] = wrapToPif(rollContinuous);
            sensors.margAttitude500Hz[PITCH] = pitch;
            sensors.margAttitude500Hz[YAW] = wrapToPif(yawContinuous);
        }

        // 最后保护一次
        if (isnan(sensors.margAttitude500Hz[PITCH]))
        {
            sensors.margAttitude500Hz[PITCH] = 0.0f;
        }
        if (isnan(sensors.margAttitude500Hz[ROLL]))
        {
        	sensors.margAttitude500Hz[ROLL]  = 0.0f;
       	}
        if (isnan(sensors.margAttitude500Hz[YAW]))
        {
            sensors.margAttitude500Hz[YAW] = 0.0f;
        }

    }
}
