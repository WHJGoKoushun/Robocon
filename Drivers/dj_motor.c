#include "dj_motor.h"

//全局电机实例
DJMotor DJmotor[USE_DJNUM];

//CubeMX生成FDCAN1句柄
extern FDCAN_HandleTypeDef hfdcan1;

FDCAN_HandleTypeDef *DJmotor_GetCanHandle(void)
{
    return &hfdcan1;
}

void DJmotor_SetZero(DJMotorPointer motor)
{
    motor->statusFlag.IsSetZero=false;
    motor->valNow.angle_deg=0.0f;
    motor->valNow.PulseTotal=0;
    motor->argum.pulseLock=0;
}

void DJmotor_PID_Reload(DJMotorPointer motor,DJmotorPID pid_reload)
{
    if(pid_reload==DJ_POS_PID)
    {
        PID_Init(&motor->posPID,0.07f,0.0005f,0.0f,PIDPOS);
    }
    else
    {
        PID_Init(&motor->velPID,5.5f,0.3f,0.01f,PIDINC);
    }
}

static void EncodeS16Data(const volatile int16_t *val,uint8_t *buf)
{
    buf[0]=(uint8_t)((*val)&0xFF);
    buf[1]=(uint8_t)(((*val)>>8)&0xFF);
}

static void ChangeDataByte(uint8_t *a,uint8_t *b)
{
    uint8_t tmp=*a;
    *a=*b;
    *b=tmp;
}

static void DJmotor_Monitor(DJMotorPointer motor);
static void DJmotor_SwitchMode(DJMotorPointer motor);
void DJmotor_SpeedMode(DJMotorPointer motor);
void DJmotor_PositionMode(DJMotorPointer motor);
void DJmotor_ZeroMode(DJMotorPointer motor);

//CAN接收中断塞包,TIM中断取包解包,软件队列解耦收包与解包
static DJmotorRxPacket rxQueue[CAN_RX_QUEUE_SIZE];
static volatile uint16_t rxHead=0;
static volatile uint16_t rxTail=0;
static volatile uint16_t rxCount=0;

void DJmotor_RxPush(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data)
{
    if(rxCount>=CAN_RX_QUEUE_SIZE)
    {
        return;
    }
    rxQueue[rxHead].Rxheader=Rxheader;
    for(uint8_t i=0;i<8U;i++)
    {
        rxQueue[rxHead].data[i]=Rx_data[i];
    }
    rxHead=(uint16_t)((rxHead+1U)%CAN_RX_QUEUE_SIZE);
    rxCount++;
}

void DJmotor_RxProcess(void)
{
    while(rxCount>0U)
    {
        DJmotor_Receive(rxQueue[rxTail].Rxheader,rxQueue[rxTail].data);
        rxTail=(uint16_t)((rxTail+1U)%CAN_RX_QUEUE_SIZE);
        rxCount--;
    }
}

void DJmotor_Init(void)
{
    DJmotorParam dj2006_param;
    DJmotorParam dj3508_param;
    DJmotorLimit limit;
    DJmotorStatus statusFlag;
    DJmotorArgum argum;
    DJmotorError error;

    dj2006_param.ParamID=0x1ffU;
    dj2006_param.Gear_ratio=1.0f;
    dj2006_param.Reduction_ratio=M2006_RATIO;
    dj2006_param.PulsePerRound=8191U;
    dj2006_param.CurrentLimit_raw=4500;

    dj3508_param.ParamID=0x200U;
    dj3508_param.Gear_ratio=1.0f;
    dj3508_param.Reduction_ratio=M3508_RATIO;
    dj3508_param.PulsePerRound=8191U;
    dj3508_param.CurrentLimit_raw=10000;

    limit.CurrentLimitFlag=true;
    limit.IsLooseStuck=false;

    limit.MaxAngle_deg=270.0f;
    limit.MinAngle_deg=-270.0f;
    limit.PosAngleLimitFlag=false;
    limit.PosRPMFlag=true;
    limit.PosRPMLimit=8000;

    limit.RPMLimitFlag=false;
    limit.SpeedRPMLimit=10000;
    limit.ZeroCurrentLimit_raw=3000;
    limit.ZeroRPMLimit=500;

    statusFlag.IsSetZero=true;
    statusFlag.Overtimeflag=false;
    statusFlag.StuckFlag=false;
    statusFlag.ZeroFlag=false;

    argum.pulseLock=0;
    argum.zeroCnt=0;
    argum.GapCnt=0;

    error.lastRxTime=0;
    error.stuckCount=0;
    error.timeoutCount=0;

    //-----------------------------------------------------------------------
    for(uint32_t i=0;i<USE_DJNUM;i++)
    {
        DJmotor[i].Begin=false;
        DJmotor[i].MODE_Set=DJ_Disable; /* 上电失能:发 0 电流 */
        DJmotor[i].statusFlag=statusFlag;
        DJmotor[i].limit=limit;
        DJmotor[i].argum=argum;
        DJmotor[i].error=error;
        DJmotor[i].valSet.current_raw=0;
        DJmotor[i].valSet.angle_deg=0.0f;
        DJmotor[i].valSet.speed_rpm=0;
        DJmotor[i].valSet.PulseTotal=0;
        DJmotor[i].valNow.PulseTotal=0;
        DJmotor[i].valPre.PulseRead=0;
    }

    //这里可以使用表封装的参数进行替换赋值
    for(uint32_t i=0;i<M2006_NUM;i++)
    {
        DJmotor[i].ID=(uint8_t)(i+1U);
        DJmotor[i].param=dj2006_param;
    }

    for(uint32_t i=0;i<M3508_NUM;i++)
    {
        DJmotor[i+M2006_NUM].ID=(uint8_t)(i+M2006_NUM+1U);
        DJmotor[i+M2006_NUM].param=dj3508_param;
    }

    for(uint32_t i=0;i<USE_DJNUM;i++)
    {
        PID_Init(&DJmotor[i].posPID,0.07f,0.0005f,0.0f,PIDPOS);
        PID_Init(&DJmotor[i].velPID,5.5f,0.3f,0.01f,PIDINC);
    }
}

void DJmotor_AngleCalculate(DJMotorPointer motor)
{
    motor->valNow.PulseGap=(int16_t)(motor->valNow.PulseRead-motor->valPre.PulseRead);

    if(ABS(motor->valNow.PulseGap)>4096)
    {
        motor->valNow.PulseGap=(int16_t)(motor->valNow.PulseGap-
            GetSign(motor->valNow.PulseGap)*(int32_t)motor->param.PulsePerRound);
    }

    motor->valNow.PulseTotal+=motor->valNow.PulseGap;
    motor->valNow.angle_deg=(float)motor->valNow.PulseTotal*360.0f/
        ((float)motor->param.PulsePerRound*motor->param.Gear_ratio*
            motor->param.Reduction_ratio);

    if(motor->Begin) //废弃字段
    {
        motor->argum.pulseLock=motor->valNow.PulseTotal;
    }

    if(motor->statusFlag.IsSetZero)
    {
        DJmotor_SetZero(motor);
        motor->statusFlag.IsSetZero=false;
    }
}

void DJmotor_Receive(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data)
{
    if((Rxheader.IdType!=FDCAN_STANDARD_ID)||
        (Rxheader.RxFrameType!=FDCAN_DATA_FRAME)||
        (Rxheader.Identifier<0x201U)||
        (Rxheader.Identifier>0x208U))
    {
        return;
    }

    uint32_t card_id=Rxheader.Identifier-0x200U;
    if(card_id>USE_DJNUM)
    {
        return;
    }

    DJMotorPointer motor=&DJmotor[card_id-1U];

    motor->valNow.PulseRead=(int16_t)(((uint16_t)Rx_data[0]<<8)|Rx_data[1]);
    motor->valNow.speed_rpm=(int16_t)(((uint16_t)Rx_data[2]<<8)|Rx_data[3]);
    motor->valNow.current_raw=(int16_t)(((uint16_t)Rx_data[4]<<8)|Rx_data[5]);

    if(motor->param.Reduction_ratio==M3508_RATIO)
    {
        motor->valNow.temperature_C=(int8_t)Rx_data[6];
        motor->valNow.current_A=(float)motor->valNow.current_raw*0.0012207f;
    }
    else
    {
        motor->valNow.current_A=(float)motor->valNow.current_raw/10000.0f*10.0f;
    }

    motor->error.lastRxTime=0;
    DJmotor_AngleCalculate(motor);
}

void DJmotor_CurrentTransmit(DJMotorPointer motor)
{
    static uint8_t tx_data[8]={0};
    FDCAN_TxHeaderTypeDef tx_header={0};
    uint8_t tag=0;

    tx_header.Identifier=0x200U;
    tx_header.IdType=FDCAN_STANDARD_ID;
    tx_header.TxFrameType=FDCAN_DATA_FRAME;
    tx_header.DataLength=FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch=FDCAN_BRS_OFF;
    tx_header.FDFormat=FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker=0;

    if(motor->ID<=4U)
    {
        tx_header.Identifier=0x1FFU;
        tag=(uint8_t)((motor->ID-1U)*2U);
    }
    else
    {
        tx_header.Identifier=0x200U;
        tag=(uint8_t)((motor->ID-5U)*2U);
    }

    EncodeS16Data(&motor->valSet.current_raw,&tx_data[tag]);
    ChangeDataByte(&tx_data[tag],&tx_data[tag+1U]);


    HAL_FDCAN_AddMessageToTxFifoQ(DJmotor_GetCanHandle(),&tx_header,tx_data);
    
}

void DJmotor_Func(void)
{
    for(uint32_t i=0;i<USE_DJNUM;i++)
    {
        if(!DJmotor[i].Begin)
        {
            DJmotor[i].valSet.current_raw=0;
            continue;
        }

        switch(DJmotor[i].MODE_Set)
        {
        case DJ_Disable:
            DJmotor[i].valSet.current_raw=0;
            break;
        case DJ_Current:
            DJmotor[i].valSet.current_raw=(int16_t)ClampPeak(DJmotor[i].valSet.current_raw,DJmotor[i].param.CurrentLimit_raw);
            break;
        case DJ_RPM:
            DJmotor_SpeedMode(&DJmotor[i]);
            break;
        case DJ_Position:
            DJmotor_PositionMode(&DJmotor[i]);
            break;
        case DJ_Zero:
            DJmotor_ZeroMode(&DJmotor[i]);
            break;
        default:
            break;
        }

        DJmotor_SwitchMode(&DJmotor[i]);
        //DJmotor_Monitor(&DJmotor[i]);
        DJmotor_CurrentTransmit(&DJmotor[i]);
    }
}

static void DJmotor_SwitchMode(DJMotorPointer motor)
{
    if(motor->MODE_Set!=motor->MODE_Cur)
    {
        motor->MODE_Cur=motor->MODE_Set;
        motor->valSet.current_raw=0;
        motor->valSet.speed_rpm=0;
        //跑完速度再进位置,先设零点,避免累积脉冲导致电机猛回跳
        if(motor->MODE_Cur==DJ_Position)
        {
            DJmotor_SetZero(motor);
            motor->valSet.angle_deg=0.0f;
        }
        else
        {
            motor->valSet.angle_deg=motor->valNow.angle_deg;
        }
        //清误差历史与位置环累加的目标速度(velPID.SetVal),避免残留值冲击新模式
        PID_Reset(&motor->posPID);
        PID_Reset(&motor->velPID);
        motor->statusFlag.ZeroFlag=false;
        motor->statusFlag.Overtimeflag=false;
        motor->statusFlag.StuckFlag=false;
    }
}

void DJmotor_SpeedMode(DJMotorPointer motor)
{
    motor->velPID.SetVal=(float)motor->valSet.speed_rpm*motor->param.Gear_ratio*
        motor->param.Reduction_ratio;
    motor->velPID.CurVal=(float)motor->valNow.speed_rpm;

    if(motor->limit.RPMLimitFlag)
    {
        motor->velPID.SetVal=ClampPeak(motor->velPID.SetVal,motor->limit.SpeedRPMLimit);
    }

    motor->valSet.current_raw+=PID_Caculate(&motor->velPID);
    motor->valSet.current_raw=(int16_t)ClampPeak(motor->valSet.current_raw,motor->param.CurrentLimit_raw);
}

void DJmotor_PositionMode(DJMotorPointer motor)
{
    motor->valSet.PulseTotal=(int32_t)(motor->valSet.angle_deg*motor->param.Gear_ratio*
        motor->param.Reduction_ratio*(float)motor->param.PulsePerRound/360.0f);
    motor->posPID.SetVal=(float)motor->valSet.PulseTotal;
    if(motor->limit.PosAngleLimitFlag)
    {
        const int32_t max_pulse=(int32_t)(motor->limit.MaxAngle_deg*
            (float)motor->param.PulsePerRound*motor->param.Gear_ratio*motor->param.Reduction_ratio/360.0f);
        const int32_t min_pulse=(int32_t)(motor->limit.MinAngle_deg*
            (float)motor->param.PulsePerRound*motor->param.Gear_ratio*motor->param.Reduction_ratio/360.0f);
        motor->posPID.SetVal=Clamp(motor->valSet.PulseTotal,min_pulse,max_pulse);
    }

    motor->posPID.CurVal=(float)motor->valNow.PulseTotal;

    motor->velPID.SetVal=PID_Caculate(&motor->posPID);
    motor->velPID.CurVal=(float)motor->valNow.speed_rpm;
    if(motor->limit.PosRPMFlag)
    {
        motor->velPID.SetVal=ClampPeak(motor->velPID.SetVal,motor->limit.PosRPMLimit);
    }

    motor->valSet.current_raw+=PID_Caculate(&motor->velPID);
    motor->valSet.current_raw=(int16_t)ClampPeak(motor->valSet.current_raw,motor->param.CurrentLimit_raw);
}

void DJmotor_ZeroMode(DJMotorPointer motor)
{
    motor->velPID.SetVal=(float)motor->limit.ZeroRPMLimit;
    motor->velPID.CurVal=(float)motor->valNow.speed_rpm;
    motor->valSet.current_raw+=PID_Caculate(&motor->velPID);
    motor->valSet.current_raw=(int16_t)ClampPeak(motor->valSet.current_raw,motor->limit.ZeroCurrentLimit_raw);

    if(ABS(motor->valNow.PulseGap)<Zero_Distance)
    {
        if(motor->argum.zeroCnt++>100U)
        {
            motor->argum.zeroCnt=0;
            motor->statusFlag.ZeroFlag=true;
            motor->Begin=false;
            //寻零结束不走 SwitchMode,这里手动清 PID 历史,重新使能时从零起步
            PID_Reset(&motor->posPID);
            PID_Reset(&motor->velPID);
            DJmotor_SetZero(motor);
        }
    }
}

static void DJmotor_Monitor(DJMotorPointer motor)
{
    if(ABS(motor->valNow.PulseGap)<5&&motor->valNow.current_raw>3000)
    {
        if(motor->error.stuckCount++>500U)
        {
            motor->error.stuckCount=0;
            motor->statusFlag.StuckFlag=true;
            if(motor->limit.IsLooseStuck)
            {
                motor->MODE_Set=DJ_Disable;
            }
        }
    }
    else
    {
        motor->error.stuckCount=0;
    }

    if(motor->error.lastRxTime++>50U)
    {
        if(motor->error.timeoutCount++>20U)
        {
            motor->error.timeoutCount=0;
            motor->MODE_Set=DJ_Disable;
            motor->statusFlag.Overtimeflag=true;
        }
    }
}

