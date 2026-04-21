/*
 * fastTrig.h
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#ifndef SRC_INC_FASTTRIG_H_
#define SRC_INC_FASTTRIG_H_

#include "main.h"  // HAL 库必须包含
#include <stdint.h>

#define SINARRAYSIZE     1024
#define SINARRAYSCALE    32767

extern short int sinDataI16[SINARRAYSIZE];

void initSinArray(void);
float fastSin(float x);

#endif /* SRC_INC_FASTTRIG_H_ */
