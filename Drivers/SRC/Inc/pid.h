/*
 * pid.h
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_PID_H_
#define SRC_INC_PID_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// 应在pid.h那里，一转移一堆暴错，懒得换了
typedef struct PIDdata
{
	float   B, P, I, D;         // 4个核心PID参数
	float   iTerm;              // 积分累加值
	float   windupGuard;        // 积分限幅（防饱和）
	float   lastDcalcValue;     // 上一时刻的误差/状态
	float   lastDterm;          // 上一时刻的D项
	float   lastLastDterm;      // 上上个时刻的D项
	uint8_t dErrorCalc;         // D项来源选择
	uint8_t type;               // 角度PID / 速率PID
} PIDdata_t;


#define OTHER   0
#define ANGULAR 1

#define D_ERROR 1
#define D_STATE 0

// PID Variables

extern uint8_t holdIntegrators;

void initPID(void);
float updatePID(
    float command,    // 【目标值】遥控器/程序给的 期望角度
    float state,      // 【当前值】AHRS姿态解算出来的 实际角度
    float deltaT,     // 【时间】两次调用之间的时间间隔（秒）
    uint8_t iHold,    // 【积分开关】true=禁止积分，false=允许积分
    PIDdata_t *PIDparameters  // 【PID参数】指向当前轴的P/I/D/历史数据
);
void setPIDintegralError(uint8_t IDPid, float value);
void zeroPIDintegralError(void);
void setPIDstates(uint8_t IDPid, float value);
void zeroPIDstates(void);

#endif /* SRC_INC_PID_H_ */
