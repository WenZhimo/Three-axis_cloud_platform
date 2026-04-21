/*
 * fastTrig.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lenovo
 */

#include "fastTrig.h"
#include "math.h"       // 用于 sinf()
#include "stdlib.h"     // 用于 round()

short int sinDataI16[SINARRAYSIZE];

void initSinArray(void)
{
    int i;

    for (i = 0; i < SINARRAYSIZE; i++)
    {
        float x = i * 2.0f * 3.1415926535f / SINARRAYSIZE;
        sinDataI16[i] = (short int)round(sinf(x) * SINARRAYSCALE);
    }
}

float fastSin(float x)
{
    if (x >= 0)
    {
        int ix = ((int)(x / (2.0f * 3.1415926535f) * (float)SINARRAYSIZE)) % SINARRAYSIZE;
        return sinDataI16[ix] / (float)SINARRAYSCALE;
    }
    else
    {
        int ix = ((int)(-x / (2.0f * 3.1415926535f) * (float)SINARRAYSIZE)) % SINARRAYSIZE;
        return -sinDataI16[ix] / (float)SINARRAYSCALE;
    }
}
