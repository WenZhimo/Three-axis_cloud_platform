/**
 * @file   computeMotorCommands.c
 * @brief  电机控制指令计算模块
 * @note   包含 Roll、Pitch、Yaw 三轴的角度换算、PID 控制及输出限制
 * @author lenovo
 * @date   Mar 25, 2026
 */

#include "computeMotorCommands.h"
#include <math.h>

/**
 * @brief  限制浮点数在指定的最小值和最大值之间
 * @param  value 当前输入值
 * @param  minValue 允许的最小值
 * @param  maxValue 允许的最大值
 * @return 限制在范围内的浮点数
 */
static float clampf(float value, float minValue, float maxValue)
{
    if (value > maxValue)
        return maxValue;
    if (value < minValue)
        return minValue;
    return value;
}

/**
 * @brief  以给定的最大步长向目标值线性逼近
 * @param  current 当前值
 * @param  target 期望逼近的目标值
 * @param  maxStep 一次允许改变的最大步长
 * @return 步进后的新值
 */
static float moveTowardsf(float current, float target, float maxStep)
{
    float delta = target - current;

    if (delta > maxStep)
        return current + maxStep;
    if (delta < -maxStep)
        return current - maxStep;

    return target;
}

/**
 * @brief  将弧度角度值包裹归一化到 [-PI, PI] 范围内
 * @param  angle 输入角度（弧度）
 * @return 归一化后的角度（弧度）
 */
static float wrapToPif(float angle)
{
    while (angle > PI)
        angle -= TWO_PI;
    while (angle < -PI)
        angle += TWO_PI;
    return angle;
}

/**
 * @brief  以给定的最大步长向目标角度逼近（考虑角度最短路径环绕）
 * @param  current 当前角度（弧度）
 * @param  target 目标角度（弧度）
 * @param  maxStep 一次允许改变的最大步长（弧度）
 * @return 步进后的新角度（弧度）
 */
static float moveTowardsAnglef(float current, float target, float maxStep)
{
    float delta = wrapToPif(target - current);

    if (delta > maxStep)
        delta = maxStep;
    if (delta < -maxStep)
        delta = -maxStep;

    return wrapToPif(current + delta);
}

/** @brief 机械角到电角的转换系数 (rpy) */
float mechanical2electricalDegrees[3] = {7.0f, 7.0f, 7.0f};
/** @brief 电角到机械角的转换系数 (rpy) */
float electrical2mechanicalDegrees[3] = {1.0f / 7.0f, 1.0f / 7.0f, 1.0f / 7.0f};

/** @brief 逻辑轴目标指令角：roll / pitch / yaw */
float pointingCmd[3] = {1.57f, 0.0f, 0.0f}; // roll想这个轴在里面设0.0，在下面设1.57

float outputRate[3];
float pidCmd[3];
float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};
rollDiag_t rollDiag = {0};

float yawCmd;

#define ROLL_TARGET_SLEW_RAD_S (0.90f)
#define ROLL_CMD_LIMIT_RAD (0.15f)
#define ROLL_I_ENABLE_ERR_RAD (1.20f)

// Yaw 自动跟随参数
#define YAP_DEADBAND 2.00f
#define MOTORPOS2SETPNT 0.35f
#define AUTOPANSMOOTH 40.00f

float centerPoint = 0.0f;
float stepSmooth = 0.0f;
float step = 0.0f;
float step_speed = 0.0f;

bool return_state = true;

float return_state_avg = 0;
int return_state_count = 0;

bool return_state_roll = true;
int return_state_count_roll = 0;

#define ROLL_SENSOR_SIGN (-1.0f)
#define ROLL_STATOR_SIGN (-1.0f)
#define ROLL_ELEC_OFFSET (0.0f)
#define ROLL_SLEW_RAD_S (0.35f)
#define ROLL_POWER (60.0f)
// Pitch 轴配置
#define PITCH_SENSOR_SIGN (-1.0f) // 若逻辑 pitch 反馈方向相反，改为 +1.0f
#define PITCH_STATOR_SIGN (2.0f) // 若 pitch 校正方向相反，改为 +1.0f
#define PITCH_CMD_LIMIT_RAD (1.5f)// 10 5
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

/**
 * @brief  偏航角(Yaw)的自动平移/跟随处理
 * @param  motorPos 电机的当前位置
 * @param  setpoint 当前的设定目标点
 * @return 经过平移补偿和平滑滤波后的新设定目标点
 */
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

/**
 * @brief  计算各轴电机控制指令的核心循环
 * @note   包含各轴防NaN处理、PID计算、输出速率限幅及电角合成计算，最终下发至 PWM 模块
 * @param  dt 控制周期的时间差 (单位：秒)
 */
void computeMotorCommands(float dt)
{
    holdIntegrators = false;

	if (eepromConfig.rollEnabled == true)
	{
		if(return_state_roll == true)
		{
			if(fabsf(pointingCmd[ROLL] - sensors.margAttitude500Hz[ROLL]) < 0.2f)
				return_state_count_roll += 1;
			else
				return_state_count_roll = 0;

			float safeDt = (dt > 0.0001f && dt < 0.01f) ? dt : 0.002f;
			float roll_angle = wrapToPif(ROLL_SENSOR_SIGN * sensors.margAttitude500Hz[ROLL]);

			if (rollAxisWasEnabled == 0)
			{
				rollTargetSlew = roll_angle;
				rollAxisWasEnabled = 1;
			}

			if(pointingCmd[ROLL] == 0.0f)
			{
				rollTargetSlew = moveTowardsAnglef(
				rollTargetSlew,
				wrapToPif(pointingCmd[ROLL]),
				ROLL_SLEW_RAD_S * safeDt);
			}


			PWM_Motor_SetAngle(
				MOTOR_PITCH,
				wrapToPif(ROLL_STATOR_SIGN * rollTargetSlew * mechanical2electricalDegrees[ROLL] + ROLL_ELEC_OFFSET),
				ROLL_POWER);
		}
		else
		{
			mechanical2electricalDegrees[ROLL] = 14.0f;
			electrical2mechanicalDegrees[ROLL] = 1.0f/14.0f;
			float safeDt = dt;
			if (safeDt > 0.01f || safeDt < 0.0001f || isnan(safeDt) || isinf(safeDt))
			{
				safeDt = 0.002f;
			}
			// 如果这里改成0呢
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

		float pitch_elec_offset = -1.25f;

		float current_elec;

		if(return_state == true)
		{
			 if(fabs(sensors.margAttitude500Hz[PITCH]) < 0.3)
				 return_state_count += 1;
			 else
				 return_state_count = 0;

			 current_elec = pitch_angle * mechanical2electricalDegrees[PITCH];
		}
		else
		{
			 current_elec = -pitch_angle * mechanical2electricalDegrees[PITCH];

			 mechanical2electricalDegrees[PITCH] = 20.0f;
			 electrical2mechanicalDegrees[PITCH] = 1.0f/20.0f;
		}
		float stator_electrical_angle = current_elec + pitch_elec_offset + PITCH_STATOR_SIGN * pidCmd[PITCH];
		stator_electrical_angle = wrapToPif(stator_electrical_angle);
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

		float current_elec = yaw_angle * mechanical2electricalDegrees[YAW];
		float stator_electrical_angle = -current_elec + YAW_STATOR_SIGN * pidCmd[YAW];

		stator_electrical_angle = wrapToPif(stator_electrical_angle);

		PWM_Motor_SetAngle(MOTOR_YAW, stator_electrical_angle, 30.0f);
	}
}
