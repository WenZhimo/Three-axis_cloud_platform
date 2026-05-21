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

// 閺堢儤顫褍瀹?-> 閻絻顫楀褍瀹虫潪顒佸床缁粯鏆?
float mechanical2electricalDegrees[3] = {7.0f, 6.0f, 7.0f};
float electrical2mechanicalDegrees[3] = {1.0f / 7.0f, 1.0f / 6.0f, 1.0f / 7.0f};

// 濡亝绮?/ 娣囶垯璇?/ 閸嬪繗鍩?閻╊喗鐖ｇ憴鎺炵礄瀵冨閿?/閻滄澘婀弰顖欏垔娴?/ 濡亝绮?/ 閸嬪繗鍩?
float pointingCmd[3] = {0.0f, 0.0f, 0.0f}; // 0.07f, -2.35f,0.0f

float outputRate[3];
float pidCmd[3];
float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};
rollDiag_t rollDiag = {0};

float yawCmd;

// 閼奉亜濮╅崶鐐拌厬閸欏倹鏆熼敍鍫濅焊閼割亣绶熼崝鈺嬬礆
#define YAP_DEADBAND 2.00f
#define MOTORPOS2SETPNT 0.35f
#define AUTOPANSMOOTH 40.00f

float centerPoint = 0.0f;
float stepSmooth = 0.0f;
float step = 0.0f;
float step_speed = 0.0f;

// 濡亝绮存潪鏉戝棘閺佸府绱欐稉?pitch 閸氬瞼绮ㄩ弸鍕剁礆
#define ROLL_SENSOR_SIGN (-1.0f) // 閼汇儲铆濠婃艾寮芥＃鍫熸煙閸氭垹娴夐崣宥忕礉閺€閫涜礋 +1.0f
#define ROLL_STATOR_SIGN (1.0f)  // 閼汇儲鐗庡锝嗘煙閸氭垹娴夐崣宥忕礉閺€閫涜礋 +1.0f
#define ROLL_CMD_LIMIT_RAD (0.15f)
#define ROLL_I_ENABLE_ERR_RAD (1.20f)  // 鐠囶垰妯婄搾鍛扮箖鐠囥儳鏁哥憴鎺戔偓鍏兼閸愯崵绮ㄧ粔顖氬瀻
#define ROLL_TARGET_SLEW_RAD_S (0.90f) // 閺堢儤顫惄顔界垼鐟欐帗娓舵径褎鏋╅悳鍥风礄rad/s閿?
// 娣囶垯璇濇潪瀵盖旂€规艾寮弫?
#define PITCH_STATOR_SIGN (-1.0f) // 閼汇儰鍒婃禒鐗堢墡濮濓絾鏌熼崥鎴犳祲閸欏稄绱濋弨閫涜礋 +1.0f
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

// 閸欏矁閰遍懕鏃囩殶閿涙艾鍘涚拋?roll 閸掗缍呴敍灞藉晙閺€鎯х磻 pitch
#define ROLL_SETTLE_ERR_RAD (0.20f) // roll 閸掗缍呯拠顖氭▕闂冨牆鈧》绱欓悽浣冾潡閿?
#define ROLL_SETTLE_CMD_RAD (0.12f) // roll 鏉堟挸鍤潏鍐ㄧ毈闂冨牆鈧》绱欓悽浣冾潡閿?
#define ROLL_SETTLE_TIME_S (0.25f)  // 鏉╃偟鐢诲陇鍐婚梼鍫濃偓鑲╂畱閺冨爼妫块敍鍫㈩潡閿?

static float rollTargetSlew = 0.0f;
static uint8_t rollAxisWasEnabled = 0;
static float pitchTargetSlew = 0.0f;
static uint8_t pitchAxisWasEnabled = 0;
static float yawTargetSlew = 0.0f;
static float yawCtrlAngle = 0.0f;
static uint8_t yawAxisWasEnabled = 0;
static uint8_t pitchGateOpen = 0;
static float rollSettledTime = 0.0f;
// 俯仰轴稳定参数
#define PITCH_SENSOR_SIGN (1.0f) // 若逻辑 pitch 反馈方向相反，改为 +1.0f
#define PITCH_STATOR_SIGN (1.0f) // 若俯仰校正方向相反，改为 +1.0f
#define PITCH_CMD_LIMIT_RAD (0.5f)// 10 5
#define PITCH_I_ENABLE_ERR_RAD (1.20f)
#define PITCH_TARGET_SLEW_RAD_S (0.90f)


float jd = 0.01; // 濞村鐦潏鎾冲毉閸婂吋顒滈弰顖氭倻娑撳﹨铔嬫潻妯绘Ц閸氭垳绗呯挧?

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

    /*if (eepromConfig.rollEnabled == true)
	{
		// =============== 閵嗘劘顫楁惔锕傛ЩNaN閵?==============
		float roll_angle = sensors.margAttitude500Hz[ROLL];

		// 1. 閻╊喗鐖ｉ崐纭风礄瀵冨閿涘鈧倸顩ч弸婊勬Ц 0.0f閿涘苯姘ㄩ弰顖涙寜楠?
		float target_angle = pointingCmd[ROLL];

		if (isnan(roll_angle) || isinf(roll_angle))
		{
			roll_angle = 0.0f;
		}

		// 1. 棣冩瘍 閸忓牏鐣婚張鐑橆潾鐠囶垰妯婇敍灞借嫙閸掆晝鏁?wrapToPif 瀵搫鍩楅梽鎰煑閸?鍗?80 鎼达讣绱欏崵PI閿涘浜掗崘鍜冪磼
		float error_mech = wrapToPif(target_angle - roll_angle);

		// 鐠嬪啰鏁?PID 鐠侊紕鐣荤悰銉ヤ缉闁?
		pidCmd[ROLL] = updatePID(
			target_angle,
			roll_angle,
			dt,
			holdIntegrators,
			&eepromConfig.PID[ROLL_PID]);

		// =============== PID鏉堟挸鍤梼鐫砤N ===============
		if (isnan(pidCmd[ROLL]) || isinf(pidCmd[ROLL]))
		{
			pidCmd[ROLL] = 0.0f;
		}

		// ==========================================
		// 闂冨弶顒?PID 濞屄ょ殶婵傝姤妞傞敍宀冪翻閸戦缚绻冩径褏娈戠悰銉ヤ缉鐟欐帒顕遍懛瀵告暩閺堢儤姣氶崝娑欏▕閹?
		// ==========================================
		if (pidCmd[ROLL] > ROLL_CMD_LIMIT_RAD)
			pidCmd[ROLL] = ROLL_CMD_LIMIT_RAD;
		if (pidCmd[ROLL] < -ROLL_CMD_LIMIT_RAD)
			pidCmd[ROLL] = -ROLL_CMD_LIMIT_RAD;

		// ==============================================
		// 閺嶇绺?FOC閿涙艾鐣剧€涙劗顥嗛崷楦跨Т閸?濠婄偛鎮?
		// 閻╊喗鐖ｇ壕浣告簚鐟欐帒瀹?= 瑜版挸澧犻惇鐔风杽娴ｅ秶鐤?+ PID鐟曚焦鐪扮悰銉ヤ缉閻ㄥ嫬浜稿顕€鍣?
		// ==============================================
		float current_elec = roll_angle * mechanical2electricalDegrees[ROLL];
		float stator_electrical_angle = current_elec + pidCmd[ROLL];

		PWM_Motor_SetAngle(MOTOR_PITCH, stator_electrical_angle, 40.0f);
	}*/


    // ========================= pitch =========================
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
                float current_elec = pitch_angle * mechanical2electricalDegrees[PITCH];
                float stator_electrical_angle = current_elec + PITCH_STATOR_SIGN * pidCmd[PITCH];

                PWM_Motor_SetAngle(MOTOR_ROLL, stator_electrical_angle, 60.0f);

            }

    // ========================= yaw =========================
       if (eepromConfig.yawEnabled == true)
		{
			// =============== 閵嗘劘顫楁惔锕傛ЩNaN閵?==============
			float yaw_angle = sensors.margAttitude500Hz[YAW];

			// 1. 閻╊喗鐖ｉ崐纭风礄瀵冨閿涘鈧倸顩ч弸婊勬Ц 0.0f閿涘苯姘ㄩ弰顖涙寜楠?
			float target_angle = pointingCmd[YAW];

			if (isnan(yaw_angle) || isinf(yaw_angle))
			{
				yaw_angle = 0.0f;
			}

			// 1. 棣冩瘍 閸忓牏鐣婚張鐑橆潾鐠囶垰妯婇敍灞借嫙閸掆晝鏁?wrapToPif 瀵搫鍩楅梽鎰煑閸?鍗?80 鎼达讣绱欏崵PI閿涘浜掗崘鍜冪磼
			float error_mech = wrapToPif(target_angle - yaw_angle);

			float safe_target = yaw_angle + error_mech;

			// 鐠嬪啰鏁?PID 鐠侊紕鐣荤悰銉ヤ缉闁?
			pidCmd[YAW] = updatePID(
				safe_target,
				yaw_angle,
				dt,
				holdIntegrators,
				&eepromConfig.PID[YAW_PID]);

			// =============== PID鏉堟挸鍤梼鐫砤N ===============
			if (isnan(pidCmd[YAW]) || isinf(pidCmd[YAW]))
			{
				pidCmd[YAW] = 0.0f;
			}

			// ==========================================
			// 闂冨弶顒?PID 濞屄ょ殶婵傝姤妞傞敍宀冪翻閸戦缚绻冩径褏娈戠悰銉ヤ缉鐟欐帒顕遍懛瀵告暩閺堢儤姣氶崝娑欏▕閹?
			// ==========================================
			if (pidCmd[YAW] > YAW_CMD_LIMIT_RAD)
				pidCmd[YAW] = YAW_CMD_LIMIT_RAD;
			if (pidCmd[YAW] < -YAW_CMD_LIMIT_RAD)
				pidCmd[YAW] = -YAW_CMD_LIMIT_RAD;

			// =============== 闁喓宸奸梽鎰煑 (闂冨弶濮? ===============
			outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];

			if (outputRate[YAW] > eepromConfig.rateLimit)
				pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;

			if (outputRate[YAW] < -eepromConfig.rateLimit)
				pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;

			pidCmdPrev[YAW] = pidCmd[YAW];

			// ==============================================
			// 閺嶇绺?FOC閿涙艾鐣剧€涙劗顥嗛崷楦跨Т閸?濠婄偛鎮?
			// 閻╊喗鐖ｇ壕浣告簚鐟欐帒瀹?= 瑜版挸澧犻惇鐔风杽娴ｅ秶鐤?+ PID鐟曚焦鐪扮悰銉ヤ缉閻ㄥ嫬浜稿顕€鍣?
			// ==============================================
			//pidCmd[YAW] = 0.0f;
			float current_elec = yaw_angle * mechanical2electricalDegrees[YAW];
			float stator_electrical_angle = -current_elec + pidCmd[YAW];

			stator_electrical_angle = wrapToPif(stator_electrical_angle);

			// printf("%.5f\r\n",stator_electrical_angle);
			PWM_Motor_SetAngle(MOTOR_YAW, stator_electrical_angle, 40.0f);
			// jd += 0.01;
		}


   /* if (eepromConfig.yawEnabled == true)
    {
        float safeDt = dt;
        if (safeDt > 0.01f || safeDt < 0.0001f || isnan(safeDt) || isinf(safeDt))
        {
            safeDt = 0.002f;
        }

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
            yawCtrlAngle = yaw_angle;
            // Align yaw target to current pose at enable to avoid first-step kick.
            pointingCmd[YAW] = yaw_angle;

            pidCmdPrev[YAW] = 0.0f;
            eepromConfig.PID[YAW_PID].iTerm = 0.0f;
            eepromConfig.PID[YAW_PID].lastDcalcValue = 0.0f;
            eepromConfig.PID[YAW_PID].lastDterm = 0.0f;
            eepromConfig.PID[YAW_PID].lastLastDterm = 0.0f;

            yawAxisWasEnabled = 1;
        }

        {
            float alpha = safeDt / (YAW_CTRL_LPF_TAU_S + safeDt);
            float yawDelta = wrapToPif(yaw_angle - yawCtrlAngle);
            yawCtrlAngle = wrapToPif(yawCtrlAngle + alpha * yawDelta);
        }

        yawTargetSlew = moveTowardsAnglef(
            yawTargetSlew,
            wrapToPif(pointingCmd[YAW]),
            YAW_TARGET_SLEW_RAD_S * safeDt);

        {
            float yaw_error_mech = wrapToPif(yawTargetSlew - yawCtrlAngle);
            if (fabsf(yaw_error_mech) < YAW_ERR_DEADBAND_RAD)
            {
                yaw_error_mech = 0.0f;
            }

            float current_electrical_angle = yawCtrlAngle * mechanical2electricalDegrees[YAW];
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
                safeDt,
                yawHoldIntegrators,
                &eepromConfig.PID[YAW_PID]);

            if (isnan(pidCmd[YAW]) || isinf(pidCmd[YAW]))
            {
                pidCmd[YAW] = 0.0f;
            }

            pidCmd[YAW] = clampf(pidCmd[YAW], -YAW_CMD_LIMIT_RAD, YAW_CMD_LIMIT_RAD);

            {
                float yawStepLimit = eepromConfig.rateLimit * safeDt;
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
                // jd += 0.01;
            }
        }
    }
    else
    {
        yawAxisWasEnabled = 0;
        yawCtrlAngle = 0.0f;
        pidCmdPrev[YAW] = 0.0f;
    }*/
}
