#ifndef __PID_H
#define __PID_H
#include "macro.h"
typedef enum
{
    PIDINC=0,//增量式
    PIDPOS=1 //位置式
} PIDMode;

typedef struct
{
    float KP;
    float KI;
    float KD;
    float SetVal;
    float CurVal;
    float output;
    float err[3];
    PIDMode mode;
} PIDType;

void PID_Init(PIDType *pid,float kp,float ki,float kd,PIDMode mode);
void PID_Reset(PIDType *pid);
float PID_Caculate(PIDType *pid);

#endif
