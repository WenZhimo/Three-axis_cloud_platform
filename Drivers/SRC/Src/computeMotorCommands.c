/*
 * computeMotorCommands.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#include "computeMotorCommands.h"
#include <math.h>

static float clampf(float value, float minValue, float maxValue)
{
    if (value > maxValue)
        return maxValue;
    if (value < minValue)
        return minValue;
    return value;
}

static float moveTowardsf(float current, float target, float maxStep)
{
    float delta = target - current;

    if (delta > maxStep)
        return current + maxStep;
    if (delta < -maxStep)
        return current - maxStep;

    return target;
}

static float wrapToPif(float angle)
{
    while (angle > PI)
        angle -= TWO_PI;
    while (angle < -PI)
        angle += TWO_PI;
    return angle;
}

static float moveTowardsAnglef(float current, float target, float maxStep)
{
    float delta = wrapToPif(target - current);

    if (delta > maxStep)
        delta = maxStep;
    if (delta < -maxStep)
        delta = -maxStep;

    return wrapToPif(current + delta);
}

// 机械弧度 -> 电角弧度转换系数
float mechanical2electricalDegrees[3] = {7.0f, 7.0f, 7.0f};
float electrical2mechanicalDegrees[3] = {1.0f / 7.0f, 1.0f / 7.0f, 1.0f / 7.0f};

// 横滚 / 俯仰 / 偏航 目标角（弧度）
float pointingCmd[3] = {-1.54f, 0.0f, 0.0f};

float outputRate[3];
float pidCmd[3];
float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};
rollDiag_t rollDiag = {0};

float yawCmd;

// 自动回中参数（偏航辅助）
#define YAP_DEADBAND 2.00f
#define MOTORPOS2SETPNT 0.35f
#define AUTOPANSMOOTH 40.00f

float centerPoint = 0.0f;
float stepSmooth = 0.0f;
float step = 0.0f;
float step_speed = 0.0f;

// 横滚轴参数（与 pitch 同结构）
#define ROLL_SENSOR_SIGN (-1.0f) // 若横滚反馈方向相反，改为 +1.0f
#define ROLL_STATOR_SIGN (-1.0f) // 若校正方向相反，改为 +1.0f
#define ROLL_CMD_LIMIT_RAD (0.30f)
#define ROLL_I_ENABLE_ERR_RAD (1.20f)  // 误差超过该电角值时冻结积分
#define ROLL_TARGET_SLEW_RAD_S (0.90f) // 机械目标角最大斜率（rad/s）
// 俯仰轴稳定参数
#define PITCH_CMD_LIMIT_RAD (0.30f)
#define PITCH_I_ENABLE_ERR_RAD (1.20f)
#define PITCH_TARGET_SLEW_RAD_S (0.90f)
#define YAW_STATOR_SIGN (-1.0f)
#define YAW_CMD_LIMIT_RAD (0.30f)
#define YAW_I_ENABLE_ERR_RAD (1.20f)
#define YAW_TARGET_SLEW_RAD_S (0.90f)
#define AXIS_MIN_STEP_LIMIT_RAD (0.001f)

// 双轴联调：先让 roll 到位，再放开 pitch
#define ROLL_SETTLE_ERR_RAD (0.20f) // roll 到位误差阈值（电角）
#define ROLL_SETTLE_CMD_RAD (0.12f) // roll 输出较小阈值（电角）
#define ROLL_SETTLE_TIME_S (0.25f)  // 连续满足阈值的时间（秒）

static float rollTargetSlew = 0.0f;
static uint8_t rollAxisWasEnabled = 0;
static float pitchTargetSlew = 0.0f;
static uint8_t pitchAxisWasEnabled = 0;
static float yawTargetSlew = 0.0f;
static uint8_t yawAxisWasEnabled = 0;
static uint8_t pitchGateOpen = 0;
static float rollSettledTime = 0.0f;

float autoPan(float motorPos, float setpoint)
{
    if (motorPos < centerPoint - YAP_DEADBAND)
    {
        centerPoint = YAP_DEADBAND;
        step = MOTORPOS2SETPNT * motorPos;
    }
    else if (motorPos > centerPoint + YAP_DEADBAND)
    {
        centerPoint = -YAP_DEADBAND;
        step = MOTORPOS2SETPNT * motorPos;
    }
    else
    {
        step = 0.0f;
        centerPoint = 0.0f;
    }

    stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;

    return (setpoint -= stepSmooth);
}

void computeMotorCommands(float dt)
{
    holdIntegrators = false;

    // ========================= roll =========================
	if (eepromConfig.rollEnabled == true)
    {
        float safeDt = dt;
        if (safeDt > 0.01f || safeDt < 0.0001f || isnan(safeDt) || isinf(safeDt))
        {
            safeDt = 0.002f;
        }

        float roll_angle = ROLL_SENSOR_SIGN * sensors.margAttitude500Hz[ROLL];
        if (isnan(roll_angle) || isinf(roll_angle))
        {
            roll_angle = 0.0f;
        }
        roll_angle = wrapToPif(roll_angle);

        if (rollAxisWasEnabled == 0)
        {
            rollTargetSlew = roll_angle;
            pidCmdPrev[ROLL] = 0.0f;
            eepromConfig.PID[ROLL_PID].iTerm = 0.0f;
            eepromConfig.PID[ROLL_PID].lastDcalcValue = 0.0f;
            eepromConfig.PID[ROLL_PID].lastDterm = 0.0f;
            eepromConfig.PID[ROLL_PID].lastLastDterm = 0.0f;
            rollAxisWasEnabled = 1;
        }

        rollTargetSlew = moveTowardsAnglef(
            rollTargetSlew,
            wrapToPif(pointingCmd[ROLL]),
            ROLL_TARGET_SLEW_RAD_S * safeDt);

        {
            float roll_error_mech = wrapToPif(rollTargetSlew - roll_angle);
            float current_electrical_angle = roll_angle * mechanical2electricalDegrees[ROLL];
            float target_electrical_angle = current_electrical_angle + roll_error_mech * mechanical2electricalDegrees[ROLL];
            float roll_error_electrical = target_electrical_angle - current_electrical_angle;
            uint8_t rollHoldIntegrators = (fabsf(roll_error_electrical) > ROLL_I_ENABLE_ERR_RAD);

            if (rollHoldIntegrators)
            {
                holdIntegrators = true;
            }

            pidCmd[ROLL] = updatePID(
                target_electrical_angle,
                current_electrical_angle,
                safeDt,
                rollHoldIntegrators,
                &eepromConfig.PID[ROLL_PID]);

            if (isnan(pidCmd[ROLL]) || isinf(pidCmd[ROLL]))
            {
                pidCmd[ROLL] = 0.0f;
            }

            pidCmd[ROLL] = clampf(pidCmd[ROLL], -ROLL_CMD_LIMIT_RAD, ROLL_CMD_LIMIT_RAD);

            {
                float rollStepLimit = eepromConfig.rateLimit * safeDt;
                if (rollStepLimit < AXIS_MIN_STEP_LIMIT_RAD)
                {
                    rollStepLimit = AXIS_MIN_STEP_LIMIT_RAD;
                }

                outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL];
                if (outputRate[ROLL] > rollStepLimit)
                {
                    pidCmd[ROLL] = pidCmdPrev[ROLL] + rollStepLimit;
                }
                if (outputRate[ROLL] < -rollStepLimit)
                {
                    pidCmd[ROLL] = pidCmdPrev[ROLL] - rollStepLimit;
                }
            }

            pidCmdPrev[ROLL] = pidCmd[ROLL];

            {
                float stator_electrical_angle = current_electrical_angle + ROLL_STATOR_SIGN * pidCmd[ROLL];
                PWM_Motor_SetAngle(MOTOR_ROLL, stator_electrical_angle, 25.0f);
            }
        }
    }
    else
    {
        rollAxisWasEnabled = 0;
        pidCmdPrev[ROLL] = 0.0f;
    }

    // ========================= pitch =========================
    if (eepromConfig.pitchEnabled == true)
	{
		// =============== 【角度防NaN】===============
		float pitch_angle = sensors.margAttitude500Hz[PITCH];

		if (isnan(pitch_angle) || isinf(pitch_angle))
		{
			pitch_angle = 0.0f;
		}

		// ==========================================
		// 🔥 提取电角度计算：方便复用
		// ==========================================
		// 目标电角度 (通常 pointingCmd 是 0)
		float target_electrical_angle = pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH];
		// 当前物理电角度 (机械角度 * 极对数)
		float current_electrical_angle = pitch_angle * mechanical2electricalDegrees[PITCH];

		// 调用 PID 计算补偿量
		pidCmd[PITCH] = updatePID(
			target_electrical_angle,
			current_electrical_angle,
			dt,
			holdIntegrators,
			&eepromConfig.PID[PITCH_PID]
		);

		// =============== PID输出防NaN ===============
		if (isnan(pidCmd[PITCH]) || isinf(pidCmd[PITCH]))
		{
			pidCmd[PITCH] = 0.0f;
		}

		// ==========================================
		// 防止 PID 没调好时，输出过大的补偿角导致电机暴力抽搐
		// ==========================================
		if (pidCmd[PITCH] >  0.5f) pidCmd[PITCH] =  0.5f;
		if (pidCmd[PITCH] < -0.5f) pidCmd[PITCH] = -0.5f;

		// =============== 速率限制 (防抖) ===============
		outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];

		if (outputRate[PITCH] > eepromConfig.rateLimit)
			pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;

		if (outputRate[PITCH] < -eepromConfig.rateLimit)
			pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;

		pidCmdPrev[PITCH] = pidCmd[PITCH];

		// ==============================================
		// 🔥🔥🔥 核心 FOC 修复：定子磁场超前/滞后
		// 目标磁场角度 = 当前真实位置 + PID要求补偿的偏差量
		// ==============================================
		float stator_electrical_angle = current_electrical_angle + pidCmd[PITCH];

		// 发送给电机驱动 (35.0f 是功率，觉得没力气可以稍微加大，但别超过电机额定发热范围)
		PWM_Motor_SetAngle(MOTOR_PITCH, stator_electrical_angle, 45.0f);//调roll的时候先把pitch锁住不动

	}

    // ========================= yaw =========================
    if (eepromConfig.yawEnabled == true)
    {
        float yaw_angle = sensors.margAttitude500Hz[YAW];
        if (isnan(yaw_angle) || isinf(yaw_angle))
        {
            yaw_angle = 0.0f;
        }

        yaw_angle = wrapToPif(yaw_angle);

        // Yaw soft-start on enable to avoid a large initial step.
        if (yawAxisWasEnabled == 0)
        {
            yawTargetSlew = yaw_angle;

            pidCmdPrev[YAW] = 0.0f;
            eepromConfig.PID[YAW_PID].iTerm = 0.0f;
            eepromConfig.PID[YAW_PID].lastDcalcValue = 0.0f;
            eepromConfig.PID[YAW_PID].lastDterm = 0.0f;
            eepromConfig.PID[YAW_PID].lastLastDterm = 0.0f;

            yawAxisWasEnabled = 1;
        }

        yawTargetSlew = moveTowardsAnglef(
            yawTargetSlew,
            wrapToPif(pointingCmd[YAW]),
            YAW_TARGET_SLEW_RAD_S * dt);

        {
            float yaw_error_mech = wrapToPif(yawTargetSlew - yaw_angle);
            float current_electrical_angle = yaw_angle * mechanical2electricalDegrees[YAW];
            float target_electrical_angle = current_electrical_angle + yaw_error_mech * mechanical2electricalDegrees[YAW];
            float yaw_error_electrical = target_electrical_angle - current_electrical_angle;
            uint8_t yawHoldIntegrators = (fabsf(yaw_error_electrical) > YAW_I_ENABLE_ERR_RAD);

            if (yawHoldIntegrators)
            {
                holdIntegrators = true;
            }

            pidCmd[YAW] = updatePID(
                target_electrical_angle,
                current_electrical_angle,
                dt,
                yawHoldIntegrators,
                &eepromConfig.PID[YAW_PID]);

            if (isnan(pidCmd[YAW]) || isinf(pidCmd[YAW]))
            {
                pidCmd[YAW] = 0.0f;
            }

            pidCmd[YAW] = clampf(pidCmd[YAW], -YAW_CMD_LIMIT_RAD, YAW_CMD_LIMIT_RAD);

            {
                float yawStepLimit = eepromConfig.rateLimit * dt;
                if (yawStepLimit < AXIS_MIN_STEP_LIMIT_RAD)
                {
                    yawStepLimit = AXIS_MIN_STEP_LIMIT_RAD;
                }

                outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];
                if (outputRate[YAW] > yawStepLimit)
                {
                    pidCmd[YAW] = pidCmdPrev[YAW] + yawStepLimit;
                }
                if (outputRate[YAW] < -yawStepLimit)
                {
                    pidCmd[YAW] = pidCmdPrev[YAW] - yawStepLimit;
                }
            }

            pidCmdPrev[YAW] = pidCmd[YAW];

            {
                float stator_electrical_angle = current_electrical_angle + YAW_STATOR_SIGN * pidCmd[YAW];
                PWM_Motor_SetAngle(MOTOR_YAW, stator_electrical_angle, eepromConfig.yawPower);
            }
        }
    }
    else
    {
        yawAxisWasEnabled = 0;
        pidCmdPrev[YAW] = 0.0f;
    }
}
