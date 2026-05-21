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

// 机械角 / 电角转换系数
float mechanical2electricalDegrees[3] = {7.0f, 6.0f, 7.0f};
float electrical2mechanicalDegrees[3] = {1.0f / 7.0f, 1.0f / 6.0f, 1.0f / 7.0f};

// 逻辑轴目标角：roll / pitch / yaw
float pointingCmd[3] = {0.0f, 0.0f, 0.0f}; // 0.07f, -2.35f,0.0f

float outputRate[3];
float pidCmd[3];
float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};
rollDiag_t rollDiag = {0};

float yawCmd;

// Yaw 自动跟随参数
#define YAP_DEADBAND 2.00f
#define MOTORPOS2SETPNT 0.35f
#define AUTOPANSMOOTH 40.00f

float centerPoint = 0.0f;
float stepSmooth = 0.0f;
float step = 0.0f;
float step_speed = 0.0f;

// Roll 轴配置；当前逻辑 roll 实际驱动物理 pitch 电机
#define ROLL_SENSOR_SIGN (-1.0f) // 若 roll 反馈方向相反，改为 +1.0f
#define ROLL_STATOR_SIGN (1.0f)  // 若 roll 校正方向相反，改为 +1.0f
#define ROLL_CMD_LIMIT_RAD (0.15f)
#define ROLL_I_ENABLE_ERR_RAD (1.20f)  // 电角误差过大时暂时关闭积分
#define ROLL_TARGET_SLEW_RAD_S (0.90f) // roll 目标角变化速率限制，单位 rad/s
// Pitch 轴配置
#define PITCH_SENSOR_SIGN (1.0f) // 若逻辑 pitch 反馈方向相反，改为 +1.0f
#define PITCH_STATOR_SIGN (1.0f) // 若 pitch 校正方向相反，改为 +1.0f
#define PITCH_CMD_LIMIT_RAD (0.5f)// 10 5
#define PITCH_I_ENABLE_ERR_RAD (1.20f)
#define PITCH_TARGET_SLEW_RAD_S (0.90f)
#define YAW_STATOR_SIGN (+1.0f)
#define YAW_CMD_LIMIT_RAD (1.0f)
#define YAW_I_ENABLE_ERR_RAD (1.20f)
#define YAW_TARGET_SLEW_RAD_S (0.90f)
#define YAW_CTRL_LPF_TAU_S (0.06f)
#define YAW_ERR_DEADBAND_RAD (0.03f)
#define AXIS_MIN_STEP_LIMIT_RAD (0.001f)

// Roll 收敛后再切给 Pitch 的判定阈值
#define ROLL_SETTLE_ERR_RAD (0.20f) // roll 误差小于该值时认为接近收敛
#define ROLL_SETTLE_CMD_RAD (0.12f) // roll 控制量小于该值时认为接近收敛
#define ROLL_SETTLE_TIME_S (0.25f)  // 上述条件连续保持的最短时间

static float rollTargetSlew = 0.0f;
static uint8_t rollAxisWasEnabled = 0;
static float pitchTargetSlew = 0.0f;
static uint8_t pitchAxisWasEnabled = 0;
static float yawTargetSlew = 0.0f;
static float yawCtrlAngle = 0.0f;
static uint8_t yawAxisWasEnabled = 0;
static uint8_t pitchGateOpen = 0;
static float rollSettledTime = 0.0f;



float jd = 0.01; // 手动调试用变量

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
            pointingCmd[ROLL] = roll_angle;
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
            float roll_error_electrical = wrapToPif(target_electrical_angle - current_electrical_angle);
            uint8_t rollHoldIntegrators = (fabsf(roll_error_electrical) > ROLL_I_ENABLE_ERR_RAD);

            if (rollHoldIntegrators)
            {
                holdIntegrators = true;
            }

            float rollPidRaw = updatePID(
                target_electrical_angle,
                current_electrical_angle,
                safeDt,
                rollHoldIntegrators,
                &eepromConfig.PID[ROLL_PID]);

            if (isnan(rollPidRaw) || isinf(rollPidRaw))
            {
                rollPidRaw = 0.0f;
            }

            pidCmd[ROLL] = clampf(rollPidRaw, -ROLL_CMD_LIMIT_RAD, ROLL_CMD_LIMIT_RAD);

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
                PWM_Motor_SetAngle(MOTOR_PITCH, stator_electrical_angle, 40.0f);
            }

            rollDiag.targetMechRad = wrapToPif(pointingCmd[ROLL]);
            rollDiag.targetSlewMechRad = rollTargetSlew;
            rollDiag.currentRawMechRad = sensors.margAttitude500Hz[ROLL];
            rollDiag.currentCtrlMechRad = roll_angle;
            rollDiag.errMechRad = roll_error_mech;
            rollDiag.errElecRad = roll_error_electrical;
            rollDiag.targetElecRad = target_electrical_angle;
            rollDiag.currentElecRad = current_electrical_angle;
            rollDiag.pidRaw = rollPidRaw;
            rollDiag.pidClamped = clampf(rollPidRaw, -ROLL_CMD_LIMIT_RAD, ROLL_CMD_LIMIT_RAD);
            rollDiag.pidApplied = pidCmd[ROLL];
            rollDiag.dPidRaw = outputRate[ROLL];
            rollDiag.stepLimit = eepromConfig.rateLimit * safeDt;
            rollDiag.sensorSign = ROLL_SENSOR_SIGN;
            rollDiag.statorSign = ROLL_STATOR_SIGN;
            rollDiag.holdI = rollHoldIntegrators;
        }
    }
    else
    {
        rollAxisWasEnabled = 0;
        pidCmdPrev[ROLL] = 0.0f;
        pidCmd[ROLL] = 0.0f;
    }


    // ========================= pitch =========================
	if (eepromConfig.pitchEnabled == true)
	{
		// =============== 【角度防NaN】===============
		float pitch_angle = PITCH_SENSOR_SIGN * sensors.margAttitude500Hz[PITCH];

		// 1. 目标值（弧度）。如果是 0.0f，就是水平
		float target_angle = pointingCmd[PITCH];

		if (isnan(pitch_angle) || isinf(pitch_angle))
		{
			pitch_angle = 1.5f;
		}

		// 1. 🚨 先算机械误差，并利用 wrapToPif 强制限制在 ±180 度（±PI）以内！
		float error_mech = wrapToPif(target_angle - pitch_angle);

		// 调用 PID 计算补偿量
		pidCmd[PITCH] = updatePID(
			target_angle,
			pitch_angle,
			dt,
			holdIntegrators,
			&eepromConfig.PID[PITCH_PID]);

		// =============== PID输出防NaN ===============
		if (isnan(pidCmd[PITCH]) || isinf(pidCmd[PITCH]))
		{
			pidCmd[PITCH] = 0.0f;
		}

		// ==========================================
		// 防止 PID 没调好时，输出过大的补偿角导致电机暴力抽搐
		// ==========================================
		if (pidCmd[PITCH] > PITCH_CMD_LIMIT_RAD)
			pidCmd[PITCH] = PITCH_CMD_LIMIT_RAD;
		if (pidCmd[PITCH] < -PITCH_CMD_LIMIT_RAD)
			pidCmd[PITCH] = -PITCH_CMD_LIMIT_RAD;

		// =============== 速率限制 (防抖) ===============
		outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];

		if (outputRate[PITCH] > eepromConfig.rateLimit)
			pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;

		if (outputRate[PITCH] < -eepromConfig.rateLimit)
			pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;

		pidCmdPrev[PITCH] = pidCmd[PITCH];

		// ==============================================
		// 核心 FOC：定子磁场围绕当前转子电角做超前/滞后补偿
		// 注意：逻辑 pitch 故意驱动物理 roll 电机，这里保持既有映射，
		// 只修正定子角计算，避免大角度时位置项错误抵消力矩角。
		// ==============================================
		float pitch_elec_offset = -1.0f;
		float current_elec = pitch_angle * mechanical2electricalDegrees[PITCH];
		//float stator_electrical_angle = current_elec + PITCH_STATOR_SIGN * pidCmd[PITCH];
		float stator_electrical_angle = current_elec + pitch_elec_offset ;
		stator_electrical_angle = wrapToPif(stator_electrical_angle);
		//printf("\r\nPWM_Motor_SetAngle:%f",stator_electrical_angle);
		PWM_Motor_SetAngle(MOTOR_ROLL, stator_electrical_angle, 60.0f);

	}

    // ========================= yaw =========================
   if (eepromConfig.yawEnabled == true)
	{
		// =============== 角度防 NaN ===============
		float yaw_angle = sensors.margAttitude500Hz[YAW];

		// 1. 目标角（弧度），0.0f 表示中性朝向
		float target_angle = pointingCmd[YAW];

		if (isnan(yaw_angle) || isinf(yaw_angle))
		{
			yaw_angle = 0.0f;
		}

		// 2. 先计算机械误差，并用 wrapToPif 限制到 ±PI
		float error_mech = wrapToPif(target_angle - yaw_angle);

		float safe_target = yaw_angle + error_mech;

		// 对包裹后的目标角执行 PID
		pidCmd[YAW] = updatePID(
			safe_target,
			yaw_angle,
			dt,
			holdIntegrators,
			&eepromConfig.PID[YAW_PID]);

		// =============== PID 输出防 NaN ===============
		if (isnan(pidCmd[YAW]) || isinf(pidCmd[YAW]))
		{
			pidCmd[YAW] = 0.0f;
		}

		// ==========================================
		// 限制 PID 输出，避免电机出现剧烈抽搐
		// ==========================================
		if (pidCmd[YAW] > YAW_CMD_LIMIT_RAD)
			pidCmd[YAW] = YAW_CMD_LIMIT_RAD;
		if (pidCmd[YAW] < -YAW_CMD_LIMIT_RAD)
			pidCmd[YAW] = -YAW_CMD_LIMIT_RAD;

		// =============== 速率限制（平滑输出） ===============
		outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];

		if (outputRate[YAW] > eepromConfig.rateLimit)
			pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;

		if (outputRate[YAW] < -eepromConfig.rateLimit)
			pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;

		pidCmdPrev[YAW] = pidCmd[YAW];

		// ==============================================
		// 核心 FOC：定子磁场角 = 转子电角 + 力矩角
		// ==============================================
		//pidCmd[YAW] = 0.0f;
		float current_elec = yaw_angle * mechanical2electricalDegrees[YAW];
		float stator_electrical_angle = -current_elec + pidCmd[YAW];

		stator_electrical_angle = wrapToPif(stator_electrical_angle);

		// printf("%.5f\r\n",stator_electrical_angle);
		PWM_Motor_SetAngle(MOTOR_YAW, stator_electrical_angle, 40.0f);
		// jd += 0.01;
	}



}
