/*
 * computeMotorCommands.h
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_COMPUTEMOTORCOMMANDS_H_
#define SRC_INC_COMPUTEMOTORCOMMANDS_H_

#include "main.h"
#include "pid.h"
#include "bgc32.h"
#include "mpu6050.h"

#pragma once

extern float mechanical2electricalDegrees[3];
extern float electrical2mechanicalDegrees[3];

extern float pidCmd[3];
extern float outputRate[3];

extern bool return_state;
extern float return_state_avg;
extern int return_state_count;
extern bool return_state_roll;
extern int return_state_count_roll;

typedef struct
{
    float targetMechRad;
    float targetSlewMechRad;
    float currentRawMechRad;
    float currentCtrlMechRad;
    float errMechRad;
    float errElecRad;
    float targetElecRad;
    float currentElecRad;
    float pidRaw;
    float pidClamped;
    float pidApplied;
    float dPidRaw;
    float stepLimit;
    float errIfSensorPlusDeg;
    float errIfSensorMinusDeg;
    float sensorSign;
    float statorSign;
    uint8_t holdI;
} rollDiag_t;

extern rollDiag_t rollDiag;

void computeMotorCommands(float dt);

#endif
