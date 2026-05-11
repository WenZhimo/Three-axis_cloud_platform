/*
 * pid.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#include "pid.h"
#include "MargAHRS.h"
#include "bgc32.h"      // 补充这个：为了拿到 PI, TWO_PI, NUMBER_OF_PIDS
#include "mpu6050.h"    // 补充这个：为了拿到 eepromConfig 全局变量
#include "math.h"       // 补充这个：如果里面有用 math 库函数

///////////////////////////////////////////////////////////////////////////////

uint8_t holdIntegrators = true;//积分总开关

#define F_CUT 20.0f

static float rc; // 滤波系数rc = 1 / (2π * F_CUT)，只在pid用


// 调整角度再(-π,π)
float standardRadianFormat(float angle)
{
    if (angle >= PI)
        return (angle - 2 * PI);
    else if (angle < -PI)
        return (angle + 2 * PI);
    else
        return (angle);
}

///////////////////////////////////////////////////////////////////////////////

void initPID(void)
{
    uint8_t index;

    rc = 1.0f / ( TWO_PI * F_CUT );

    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
    	eepromConfig.PID[index].iTerm          = 0.0f;
    	eepromConfig.PID[index].lastDcalcValue = 0.0f;
    	eepromConfig.PID[index].lastDterm      = 0.0f;
    	eepromConfig.PID[index].lastLastDterm  = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////

/*float updatePID(float command,
                float state,
                float deltaT,
                uint8_t iHold,
                struct PIDdata *PIDparameters)
{
    float error;
    float dTerm;
    float dTermFiltered;
    float dAverage;

    // 防过大的
    if (deltaT > 0.01f || deltaT < 0.0001f)
	{
		deltaT = 0.002f;
	}

    // =============== 【终极防NaN：时间保护】===============
    if (deltaT < 0.0001f || isnan(deltaT) || isinf(deltaT))
    {
        deltaT = 0.002f;
    }

    // =============== 角度误差 ===============
    error = command - state;

    if (PIDparameters->type == ANGULAR)
    {
    	// 直接用角度了
        // error = standardRadianFormat(error);
    }

    // 本来是那个constrain限制函数的，完全不起作用，不知道为啥··，限制函数也没解决关键，他依旧巨大，没找到问题
    // =============== 积分 ===============
    if (iHold == false)
    {
		float temp_iTerm = PIDparameters->iTerm + error * deltaT;
		//printf("限制之前的：=%f\r\n",temp_iTerm);

		if (temp_iTerm > PIDparameters->windupGuard)
		{
			temp_iTerm = PIDparameters->windupGuard;
		}
		else if (temp_iTerm < -PIDparameters->windupGuard)
		{
			temp_iTerm = -PIDparameters->windupGuard;
		}

		// 赋值回 iTerm
		PIDparameters->iTerm = temp_iTerm;

		//printf("限制之后的：=%f\r\n",PIDparameters->iTerm);

		//deltat会出几个很大的值
		//printf("deltaT=%f,error=%f\r\n",deltaT,error);
    }

    // =============== 【防NaN：D项初始化保护】===============
    if (PIDparameters->lastDcalcValue == 0.0f)
    {
        PIDparameters->lastDcalcValue = state;
    }


    // =============== D项计算 ===============
    if (PIDparameters->dErrorCalc == D_ERROR)
    {
        dTerm = (error - PIDparameters->lastDcalcValue) / deltaT;
        PIDparameters->lastDcalcValue = error;
    }
    else
    {
        dTerm = (PIDparameters->lastDcalcValue - state) / deltaT;
        if (PIDparameters->type == ANGULAR)
        {
            dTerm = standardRadianFormat(dTerm);
        }
        PIDparameters->lastDcalcValue = state;
    }

    // =============== D项超强限幅（防NaN）===============
    if (isnan(dTerm) || isinf(dTerm)) dTerm = 0.0f;
    if (dTerm >  300.0f) dTerm =  300.0f;
    if (dTerm < -300.0f) dTerm = -300.0f;

    // =============== D项滤波 ===============
    dTermFiltered = PIDparameters->lastDterm + deltaT / (rc + deltaT) * (dTerm - PIDparameters->lastDterm);

    if (isnan(dTermFiltered) || isinf(dTermFiltered)) dTermFiltered = 0.0f;

    dAverage = (dTermFiltered + PIDparameters->lastDterm + PIDparameters->lastLastDterm) * 0.333333f;

    PIDparameters->lastLastDterm = PIDparameters->lastDterm;
    PIDparameters->lastDterm = dTermFiltered;

    // =============== 输出计算（防NaN）===============
    float output = 0.0f;

    if (PIDparameters->type == ANGULAR)
    {
        output = PIDparameters->P * error
               + PIDparameters->I * PIDparameters->iTerm
               + PIDparameters->D * dAverage;
    }
    else
    {
        output = PIDparameters->P * PIDparameters->B * command
               + PIDparameters->I * PIDparameters->iTerm
               + PIDparameters->D * dAverage
               - PIDparameters->P * state;
    }

    // 最后一次防爆
    if (isnan(output) || isinf(output))
    {
        output = 0.0f;
    }



   // printf("command=%f,state=%f,error=%f\r\n",command,state,error);

    return output;
}*/

float updatePID(float command,
                float state,
                float deltaT,
                uint8_t iHold,
                struct PIDdata *PIDparameters)
{
    float error;
    float dTerm;
    float dTermFiltered;
    float dAverage;

    if (deltaT > 0.01f || deltaT < 0.0001f || isnan(deltaT) || isinf(deltaT))
    {
        deltaT = 0.002f;
    }

    error = command - state;

    if (PIDparameters->type == ANGULAR)
    {
    }

    // =============== 积分 + 超强抗饱和 ===============
    if (iHold == false)
    {
        float temp_iTerm = PIDparameters->iTerm + error * deltaT;

        // 全局积分限幅（所有轴）
        if (temp_iTerm >  1.0f) temp_iTerm =  1.0f;
        if (temp_iTerm < -1.0f) temp_iTerm = -1.0f;

        // PITCH 轴额外更严格限幅（防止爆炸）
        if (PIDparameters == &eepromConfig.PID[PITCH_PID])
        {
            if (temp_iTerm >  3.14f) temp_iTerm =  3.14f;
            if (temp_iTerm < -3.14f) temp_iTerm = -3.14f;
        }

        PIDparameters->iTerm = temp_iTerm;
    }

    if (PIDparameters->lastDcalcValue == 0.0f)
    {
        PIDparameters->lastDcalcValue = state;
    }

    if (PIDparameters->dErrorCalc == D_ERROR)
    {
        dTerm = (error - PIDparameters->lastDcalcValue) / deltaT;
        PIDparameters->lastDcalcValue = error;
    }
    else
    {
        dTerm = (PIDparameters->lastDcalcValue - state) / deltaT;
        if (PIDparameters->type == ANGULAR)
        {
            dTerm = standardRadianFormat(dTerm);
        }
        PIDparameters->lastDcalcValue = state;
    }

    if (isnan(dTerm) || isinf(dTerm)) dTerm = 0.0f;
    if (dTerm >  300.0f) dTerm =  300.0f;
    if (dTerm < -300.0f) dTerm = -300.0f;

    dTermFiltered = PIDparameters->lastDterm + deltaT / (rc + deltaT) * (dTerm - PIDparameters->lastDterm);
    if (isnan(dTermFiltered) || isinf(dTermFiltered)) dTermFiltered = 0.0f;

    dAverage = (dTermFiltered + PIDparameters->lastDterm + PIDparameters->lastLastDterm) * 0.333333f;

    PIDparameters->lastLastDterm = PIDparameters->lastDterm;
    PIDparameters->lastDterm = dTermFiltered;

    float output = 0.0f;

    if (PIDparameters->type == ANGULAR)
    {
        output = PIDparameters->P * error
               + PIDparameters->I * PIDparameters->iTerm
               + PIDparameters->D * dAverage;
    }
    else
    {
        output = PIDparameters->P * PIDparameters->B * command
               + PIDparameters->I * PIDparameters->iTerm
               + PIDparameters->D * dAverage
               - PIDparameters->P * state;
    }

    if (isnan(output) || isinf(output))
    {
        output = 0.0f;
    }

    return output;
}

/////////////////////////////////////////////////////
/// 目前没找到下面几个函数在那用，纯看
/////////////////////////////////////////////////////
// 函数功能：设置指定轴PID的积分项（iTerm）
// 入口参数：
//     IDPid  ：轴号（ROLL_PID / PITCH_PID / YAW_PID）
//     value  ：要设置的积分值
void setPIDintegralError(uint8_t IDPid, float value)
{
    // 把指定轴的积分累加值直接设置为 value
    eepromConfig.PID[IDPid].iTerm = value;
}

// 函数功能：将3个轴的PID积分项全部清零（最常用！）
void zeroPIDintegralError(void)
{
    uint8_t index;

    // 遍历 0、1、2 三个PID（Roll、Pitch、Yaw）
    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
         // 每个轴都调用上面的函数，把积分设为 0
         setPIDintegralError(index, 0.0f);
    }
}

// 函数功能：重置指定轴PID的所有D项历史状态
// （上次误差、上次D、上上次D 全部设为同一个值）
void setPIDstates(uint8_t IDPid, float value)
{
    // 上一帧的误差/状态值
    eepromConfig.PID[IDPid].lastDcalcValue = value;

    // 上一帧的D项
    eepromConfig.PID[IDPid].lastDterm      = value;

    // 上两帧的D项
    eepromConfig.PID[IDPid].lastLastDterm  = value;
}

// 函数功能：将3个轴的D项历史状态全部清零（开机必调用）
void zeroPIDstates(void)
{
    uint8_t index;

    // 遍历三个轴
    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
         // 每个轴的D历史都设为0
         setPIDstates(index, 0.0f);
    }
}
