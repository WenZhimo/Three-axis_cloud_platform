/*
 * firstOrderFilter.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#include "firstOrderFilter.h"
#include "main.h"


firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(void)
{
    float a;

    a = 2.0f * eepromConfig.accelX500HzLowPassTau * 500.0f;
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousOutput = 0.0f;

    a = 2.0f * eepromConfig.accelY500HzLowPassTau * 500.0f;
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].previousOutput = 0.0f;

    a = 2.0f * eepromConfig.accelZ500HzLowPassTau * 500.0f;
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].previousInput  = -9.8065f;
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].previousOutput = -9.8065f;

    /*a = 2.0f * eepromConfig.rollRatePointingCmd50HzLowPassTau * 50.0f;
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;*/

    a = 2.0f * eepromConfig.pitchRatePointingCmd50HzLowPassTau * 50.0f;
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

   /* a = 2.0f * eepromConfig.yawRatePointingCmd50HzLowPassTau * 50.0f;
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;*/

  /*  a = 2.0f * eepromConfig.rollAttPointingCmd50HzLowPassTau * 50.0f;
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;*/

    a = 2.0f * eepromConfig.pitchAttPointingCmd50HzLowPassTau * 50.0f;
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    /*a = 2.0f * eepromConfig.yawAttPointingCmd50HzLowPassTau * 50.0f;
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;*/
}

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}
