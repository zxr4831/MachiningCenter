#ifndef __TIOCOMMAND_H
#define __TIOCOMMAND_H
#include "common.h"
#include "IOCommand.h"
#include "ErrorFunction.h"
#include "Channel.h"
#define TOOLCHANGE_IDLE 0
#define TOOLCHANGE_SETSERVO_ENABLED 1
#define TOOLCHANGE_OUTPUT_TOOLNUM 2
#define TOOLCHANGE_WAITFOR_SENDTRIG 3
#define TOOLCHANGE_SEND_TRIGSIGN 4
#define TOOLCHANGE_WAIT_GET_INPLACESIGN 5
#define TOOLCHANGE_WAIT_GET_TOOLNUM 6
#define TOOLCHANGE_WAIT_GETLOCK 7
#define TOOLCHANGE_WAIT_LOCKED 8
#define TOOLCHANGE_DONE 9
#define TOOLCHANGE_ERROR 10

#define LOCK 1
#define UNLOCK 0

#define LOCK_ELECTRICAL_LEVEL 0
#define UNLOCK_ELECTRICAL_LEVEL 1

#define READIO_NUM_MAX 5

struct _SIOReadGroup
{
    uint8 ucNum;
#if STM32_MODE
    GPIO_TypeDef* unGPIOxArrays[READIO_NUM_MAX];

#else

    uint32 unGPIOxArrays[READIO_NUM_MAX];
#endif
    uint8 ucPinArrays[READIO_NUM_MAX];
    uint8 ucIOLowLevelTimesArray[READIO_NUM_MAX];
    uint8 ucIOAlias[READIO_NUM_MAX];
};

int8 TIOSTRUCT_InitEditablePara();
int8 ReadToolNum_AfterLoadSysPara(struct _STIOCommand* pSTIOCommand, struct _SRun* pSRun,
                                  struct SInterpolationPre* pSPreProcess);
uint8 TIOToRunBuff(uint8 ucTIOCommand, uint8 ucToolCompNum, struct _STIOCommand* pSTIOCommand);
int8 runTIO(struct _STIOCommand* pSTIOCommand);
int8 getToolNumFromServo();
int8 SelectToolNum(int8 cToolNum);
void ClearSelectedToolNumIO();
int8 WaitFor_SendTrigSignal(struct _STIOCommand* pSTIOCommand);
int8 Trig_ToolChange(struct _STIOCommand* pSTIOCommand);
int8 getToolChangeStatus(struct _STIOCommand* pSTIOCommand);
int8 getToolCarrier_LockedStatus();
int8 setToolCarrier_Locked(int8 cCommand);
int8 getToolChange_InPlaceStatus();
void set_ToolServo_EnabledStatus(int8 cCommand);
int8 MultiSample(struct SChannel* pSChannel, uint8* ptimes, uint8 ucDuration, struct _SIOReadGroup* pIOReadGroup);
#endif
