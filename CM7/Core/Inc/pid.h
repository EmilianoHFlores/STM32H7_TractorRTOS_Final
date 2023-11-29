#ifndef __PID_H
#define __PID_H

#include "stdlib.h"
#include "stdint.h"

typedef struct
{
    // all starts as 0
    float kp;
    float ki;
    float kd;
    float iMax;
    float error_sum;
    float error_last;
    float outMin;
    float outMax;
} PID;

void PID_init(PID* pid, float kp, float ki, float kd, float iMax, float outMax, float outMin);
float PID_calc(PID *pid, float target, float current);

#endif
