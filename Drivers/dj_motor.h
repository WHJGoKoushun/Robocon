#ifndef __DJ_MOTOR_H
#define __DJ_MOTOR_H

#include "stm32h7xx_hal.h"
#include "macro.h"
#include "pid.h"

//电机数量配置:8 台,M2006 占 ID1~4,M3508 占 ID5~8
#define USE_DJ 1
#define M2006_NUM 4
#define M3508_NUM 4
#define USE_DJNUM (M2006_NUM+M3508_NUM)

#define M2006_RATIO 36.0f
#define M3508_RATIO (3591.0f/187.0f)

#define Zero_Distance 5

#define CAN_RX_QUEUE_SIZE 16

typedef enum
{
    DJ_Disable=0,/* 关: transmit 0 current */
    DJ_RPM=1,/* 速度 mode */
    DJ_Position=2,/* 位置 mode */
    DJ_Zero=3,/* 寻零 mode */
    DJ_Current=4,/* 电流/扭矩 */
} DJmotor_mode_t;

typedef enum
{
    DJ_POS_PID=0,/* 位置环 */
    DJ_VEL_PID=1 /* 速度环 */
} DJmotorPID;

typedef struct
{
    volatile int16_t current_raw; // 直接设置电流
    volatile float angle_deg; // 输出角度，degree
    volatile int16_t speed_rpm; // valSet: 输出轴 rpm;valNow: 转子 rpm(原始反馈)
    volatile float current_A; // 反馈电流，A
    volatile int16_t PulseRead; // raw encoder pulse
    volatile int16_t PulseGap; // pulse delta
    volatile int32_t PulseTotal; // accumulated pulse
    volatile int8_t temperature_C; // ℃
} DJmotorVal;

typedef struct
{
    uint16_t PulsePerRound; // 8191
    float Gear_ratio; // mechanism ratio
    float Reduction_ratio; // motor reducer ratio
    uint32_t ParamID; // CAN receive ID base
    int16_t CurrentLimit_raw; // output current limit, raw
} DJmotorParam;

typedef struct
{
    bool RPMLimitFlag;
    bool PosAngleLimitFlag;
    bool PosRPMFlag;
    bool CurrentLimitFlag;
    float MaxAngle_deg; // degree
    float MinAngle_deg; // degree
    int16_t SpeedRPMLimit; // rpm
    int32_t PosRPMLimit; // rpm
    int16_t ZeroRPMLimit; // rpm
    int16_t ZeroCurrentLimit_raw; // raw
    bool IsLooseStuck;
} DJmotorLimit;

typedef struct
{
    bool IsSetZero;
    bool Overtimeflag;
    bool StuckFlag;
    bool ZeroFlag;
} DJmotorStatus;

typedef struct
{
    int32_t pulseLock; // 锁存脉冲,有符号
    uint32_t zeroCnt; // 寻零计数
    uint32_t GapCnt; // 预留
} DJmotorArgum;

typedef struct
{
    uint32_t lastRxTime; // 距上次收包周期数
    uint32_t stuckCount; // 堵转计数
    uint32_t timeoutCount; // 超时计数
} DJmotorError;

typedef struct
{
    FDCAN_RxHeaderTypeDef Rxheader;
    uint8_t data[8];
} DJmotorRxPacket;

typedef struct
{
    uint8_t ID;
    volatile bool Begin; // true 运行 MODE;false 失能
    volatile DJmotor_mode_t MODE_Set; // DJ_Disable 即失能(发 0 电流)
    volatile DJmotor_mode_t MODE_Cur; // 实际运行模式,任务层可读

    DJmotorParam param;
    DJmotorVal valSet;
    DJmotorVal valNow;
    DJmotorVal valPre;
    DJmotorStatus statusFlag;
    DJmotorLimit limit;
    DJmotorArgum argum;
    DJmotorError error;
    PIDType posPID;
    PIDType velPID;
} DJMotor,*DJMotorPointer;

#if USE_DJ
extern DJMotor DJmotor[USE_DJNUM];

void DJmotor_Init(void);
void DJmotor_Func(void);
void DJmotor_Receive(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);
void DJmotor_PID_Reload(DJMotorPointer motor,DJmotorPID pid_reload);
void DJmotor_SetZero(DJMotorPointer motor);
FDCAN_HandleTypeDef *DJmotor_GetCanHandle(void);
void DJmotor_RxPush(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);
void DJmotor_RxProcess(void);

#endif

#endif
