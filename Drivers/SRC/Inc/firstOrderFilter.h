/*
 * firstOrderFilter.h
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_FIRSTORDERFILTER_H_
#define SRC_INC_FIRSTORDERFILTER_H_

#include "main.h"  // HAL 库必需

#define NUMBER_OF_FIRST_ORDER_FILTERS 9

#define ACCEL_X_500HZ_LOWPASS  0
#define ACCEL_Y_500HZ_LOWPASS  1
#define ACCEL_Z_500HZ_LOWPASS  2

#define ROLL_RATE_POINTING_50HZ_LOWPASS  3
#define PITCH_RATE_POINTING_50HZ_LOWPASS 4
#define YAW_RATE_POINTING_50HZ_LOWPASS   5

#define ROLL_ATT_POINTING_50HZ_LOWPASS  6
#define PITCH_ATT_POINTING_50HZ_LOWPASS 7
#define YAW_ATT_POINTING_50HZ_LOWPASS   8

typedef struct firstOrderFilterData
{
    float   gx1;
    float   gx2;
    float   gx3;
    float   previousInput;
    float   previousOutput;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(void);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters);

#endif /* SRC_INC_FIRSTORDERFILTER_H_ */
