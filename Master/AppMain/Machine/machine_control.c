/**
 * @file         : machine_control.c
 * @author       : zxr
 * @brief        : 独立整组轴执行接口；由外部周期提交整数增量
 */

#include "machine_control.h"
#include "axis_interface.h"
#include "tim.h" /* zxr: 复用板级微秒时基，主站不依赖数控内核或其端口。 */
#include <string.h>

/* zxr: 沿用已实测的启动等待，不将这 1 秒宣称为驱动器抱闸释放时间。 */
#define ENABLE_TIMEOUT_TICKS (5000000U / AXIS_CYCLE_PERIOD_US)
#define READY_HOLD_TICKS (1000000U / AXIS_CYCLE_PERIOD_US)
#define RESET_TIMEOUT_TICKS (1000000U / AXIS_CYCLE_PERIOD_US)

#define MC_CIA402_MASK_BASIC 0x006FU
#define MC_CIA402_VAL_OPERATION_ENABLE 0x0027U
#define MC_CIA402_MASK_FAULT 0x004FU
#define MC_CIA402_VAL_FAULT 0x0008U
#define MC_CIA402_VAL_FAULT_REACTION 0x000FU

typedef struct
{
    MACHINE_STATE eState;
    MACHINE_ERROR eError;
    uint8_t ucFaultAxis;
    int32_t nTargetPosition[MACHINE_PHYSICAL_AXIS_COUNT];
    int32_t nActualPosition[MACHINE_PHYSICAL_AXIS_COUNT];
    uint32_t unReadyTicks;
    uint32_t unCycleUs, unMaxCycleUs;
} SMachineControlStatus;

/* zxr: 前台改 s_unPendingCommand；s_SMachineStatus 只观察，一致性读取用 mc_getSnapshot。 */
static volatile uint32_t s_unPendingCommand;
static volatile SMachineControlStatus s_SMachineStatus;
static uint8_t s_ucActiveAxisCount;
static int32_t s_nStepLimit[MACHINE_PHYSICAL_AXIS_COUNT];
static int8_t s_cDirectionInvert[MACHINE_PHYSICAL_AXIS_COUNT];
static bool s_bAligned[MACHINE_PHYSICAL_AXIS_COUNT];
static uint32_t s_unWaitTicks, s_unLastLostWkc;
static bool s_bStepSubmitted;
static bool s_bResetIssued;
static uint32_t s_unCycleCount;
static bool s_bFeedbackValid;
static volatile uint32_t s_unSnapshotSequence;
static SMachineControlSnapshot s_SSnapshot;

static bool isAxisHardwareFault(uint16_t usStatusWord)
{
    uint16_t usFaultBits = usStatusWord & MC_CIA402_MASK_FAULT;
    return usFaultBits == MC_CIA402_VAL_FAULT || usFaultBits == MC_CIA402_VAL_FAULT_REACTION;
}

static void publishSnapshot(void)
{
    s_unSnapshotSequence++;
    __asm__ volatile("" ::: "memory");

    s_SSnapshot.eState = s_SMachineStatus.eState;
    s_SSnapshot.eError = s_SMachineStatus.eError;
    s_SSnapshot.ucFaultAxis = s_SMachineStatus.ucFaultAxis;
    s_SSnapshot.ucActiveAxisCount = s_ucActiveAxisCount;

    for (uint8_t i = 0; i < MACHINE_PHYSICAL_AXIS_COUNT; i++)
    {
        if (i < s_ucActiveAxisCount)
        {
            s_SSnapshot.nTargetPosition[i] = s_SMachineStatus.nTargetPosition[i];
            s_SSnapshot.nActualPosition[i] = s_SMachineStatus.nActualPosition[i];
            s_SSnapshot.usStatusWord[i] = axis_getStatusWord(i);
            s_SSnapshot.nFollowingError[i] = axis_getFollowingError(i);
        }
        else
        {
            s_SSnapshot.nTargetPosition[i] = 0;
            s_SSnapshot.nActualPosition[i] = 0;
            s_SSnapshot.usStatusWord[i] = 0;
            s_SSnapshot.nFollowingError[i] = 0;
        }
    }

    s_SSnapshot.unReadyTicks = s_SMachineStatus.unReadyTicks;
    s_SSnapshot.unCycleUs = s_SMachineStatus.unCycleUs;
    s_SSnapshot.unMaxCycleUs = s_SMachineStatus.unMaxCycleUs;
    s_SSnapshot.unLostWkcCount = axis_getLostWkcCount();
    s_SSnapshot.bFeedbackValid = s_bFeedbackValid;
    s_SSnapshot.unCycleCount = s_unCycleCount;

    __asm__ volatile("" ::: "memory");
    s_unSnapshotSequence++;
}

/* zxr: 统一去使能；普通停止不能覆盖故障，复位期间收到去使能须退回故障态保留原错误。 */
static void stopMachineGroup(MACHINE_ERROR eError, uint8_t ucFaultAxis)
{
    /* zxr: 主站只撤销轴使能并发布状态，调用方负责停止推进自己的轨迹。 */
    for (uint8_t i = 0; i < AXIS_MAX_NUM; i++)
    {
        axis_setEnable(i, false);
        axis_cancelReset(i);
    }
    s_bResetIssued = false;

    if (s_SMachineStatus.eState == MC_STATE_RESETTING && eError == MC_ERROR_NONE)
    {
        s_SMachineStatus.eState = MC_STATE_FAULT;
        return;
    }

    if (s_SMachineStatus.eState != MC_STATE_FAULT)
    {
        s_SMachineStatus.eError = eError;
        s_SMachineStatus.ucFaultAxis = (eError == MC_ERROR_NONE) ? MACHINE_AXIS_NONE : ucFaultAxis;
        s_SMachineStatus.eState = eError == MC_ERROR_NONE ? MC_STATE_NOT_READY : MC_STATE_FAULT;
    }
}

static void handleResettingState(bool bLostFrame)
{
    for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
    {
        axis_setEnable(ucAxis, false);
        s_SMachineStatus.nTargetPosition[ucAxis] = s_SMachineStatus.nActualPosition[ucAxis];
    }

    if (!s_bResetIssued)
    {
        bool bCanIssue = !bLostFrame;
        for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
        {
            if ((axis_getStatusWord(ucAxis) & MC_CIA402_MASK_BASIC) == MC_CIA402_VAL_OPERATION_ENABLE)
            {
                bCanIssue = false;
                break;
            }
        }

        if (bCanIssue)
        {
            bool bAllResetting = true;
            for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
            {
                axis_resetFault(ucAxis);
                bAllResetting = bAllResetting && axis_isResetting(ucAxis);
            }
            if (bAllResetting)
            {
                s_bResetIssued = true;
            }
        }
    }
    else
    {
        bool bAllFinished = true;
        for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
        {
            if (axis_isResetting(ucAxis))
            {
                bAllFinished = false;
                break;
            }
        }

        if (bAllFinished)
        {
            if (bLostFrame || axis_isBusError())
            {
                stopMachineGroup(MC_ERROR_BUS, MACHINE_AXIS_NONE);
                return;
            }
            for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
            {
                uint16_t usStatusWord = axis_getStatusWord(ucAxis);
                bool bStillOp = (usStatusWord & MC_CIA402_MASK_BASIC) == MC_CIA402_VAL_OPERATION_ENABLE;
                if (axis_isFault(ucAxis) || isAxisHardwareFault(usStatusWord) || bStillOp)
                {
                    stopMachineGroup(MC_ERROR_AXIS, ucAxis);
                    return;
                }
            }

            s_SMachineStatus.eError = MC_ERROR_NONE;
            s_SMachineStatus.ucFaultAxis = MACHINE_AXIS_NONE;
            s_SMachineStatus.unReadyTicks = 0;
            s_bResetIssued = false;
            s_SMachineStatus.eState = MC_STATE_NOT_READY;
            return;
        }
    }

    if (++s_unWaitTicks >= RESET_TIMEOUT_TICKS)
    {
        if (bLostFrame || axis_isBusError())
        {
            stopMachineGroup(MC_ERROR_BUS, MACHINE_AXIS_NONE);
            return;
        }
        for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
        {
            if (axis_isFault(ucAxis) || isAxisHardwareFault(axis_getStatusWord(ucAxis)))
            {
                stopMachineGroup(MC_ERROR_AXIS, ucAxis);
                return;
            }
        }
        stopMachineGroup(s_SMachineStatus.eError != MC_ERROR_NONE ? s_SMachineStatus.eError : MC_ERROR_AXIS, s_SMachineStatus.ucFaultAxis);
    }
}

/* 物理轴号沿用轴层索引，调用方传入的保护参数数量必须与在线驱动轴数逐一匹配。 */
void mc_init(const SMachineAxisLimits *pSAxisLimits, uint8_t ucAxisCount)
{
    memset((void *)&s_SMachineStatus, 0, sizeof(s_SMachineStatus));
    s_SMachineStatus.ucFaultAxis = MACHINE_AXIS_NONE;
    s_unPendingCommand = MC_CMD_NONE;
    s_unWaitTicks = 0;
    s_unLastLostWkc = axis_getLostWkcCount();
    s_bStepSubmitted = false;
    s_bResetIssued = false;
    s_unCycleCount = 0;
    s_bFeedbackValid = false;
    s_unSnapshotSequence = 0;
    memset(s_bAligned, 0, sizeof(s_bAligned));
    memset(s_cDirectionInvert, 0, sizeof(s_cDirectionInvert));
    memset(s_nStepLimit, 0, sizeof(s_nStepLimit));
    memset(&s_SSnapshot, 0, sizeof(s_SSnapshot));
    s_ucActiveAxisCount = 0;

    uint8_t ucActiveAxes = axis_getActiveCount();
    if (ucActiveAxes == 0 || ucActiveAxes > AXIS_MAX_NUM || ucActiveAxes > MACHINE_PHYSICAL_AXIS_COUNT || pSAxisLimits == NULL || ucAxisCount != ucActiveAxes)
    {
        stopMachineGroup(MC_ERROR_CONFIG, MACHINE_AXIS_NONE);
        publishSnapshot();
        return;
    }

    for (uint8_t ucAxis = 0; ucAxis < ucActiveAxes; ucAxis++)
    {
        const SMachineAxisLimits *pSLimits = &pSAxisLimits[ucAxis];
        if (!axis_isAvailable(ucAxis) || (pSLimits->cDirectionInvert != 1 && pSLimits->cDirectionInvert != -1) || pSLimits->nMaxStepLimit <= 0 || pSLimits->nMaxFollowingErrorLimit <= 0)
        {
            stopMachineGroup(MC_ERROR_CONFIG, ucAxis);
            publishSnapshot();
            return;
        }

        s_nStepLimit[ucAxis] = pSLimits->nMaxStepLimit;
        s_cDirectionInvert[ucAxis] = pSLimits->cDirectionInvert;
        axis_setSafetyLimits(ucAxis, pSLimits->nMaxStepLimit, pSLimits->nMaxFollowingErrorLimit);
    }

    s_ucActiveAxisCount = ucActiveAxes;
    stopMachineGroup(MC_ERROR_NONE, MACHINE_AXIS_NONE);
    publishSnapshot();
}

/* zxr: 单生产者发布一个 32 位命令，不带多字段载荷；TIM3 消费后清零。
 * 运动数组只由同一周期上下文调用 mc_submitStep，不经过此命令槽。 */
bool mc_requestCommand(MACHINE_COMMAND eCommand)
{
    if (eCommand < MC_CMD_ENABLE || eCommand > MC_CMD_RESET || s_unPendingCommand != MC_CMD_NONE)
        return false;
    s_unPendingCommand = eCommand;
    return true;
}

static void runCycleStep(void)
{
    MACHINE_COMMAND eCommand = (MACHINE_COMMAND)s_unPendingCommand;
    s_unPendingCommand = MC_CMD_NONE;
    s_bStepSubmitted = false;

    if (eCommand == MC_CMD_DISABLE)
        stopMachineGroup(MC_ERROR_NONE, MACHINE_AXIS_NONE); /* zxr: 去使能先于本拍 PDO 交换；故障锁存不被清除。 */

    axis_runSyncCycle(); /* zxr: 每拍唯一一次收发，处理上一拍暂存目标并取得最新反馈。 */

    s_unCycleCount++;
    s_bFeedbackValid = axis_isFeedbackValid();
    bool bLostFrame = !s_bFeedbackValid || (axis_getLostWkcCount() != s_unLastLostWkc);
    s_unLastLostWkc = axis_getLostWkcCount();

    for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
    {
        s_SMachineStatus.nActualPosition[ucAxis] = axis_getActualPos(ucAxis);
    }

    if (s_SMachineStatus.eState == MC_STATE_FAULT)
    {
        if (eCommand == MC_CMD_RESET && s_SMachineStatus.eError != MC_ERROR_CONFIG && s_ucActiveAxisCount > 0)
        {
            for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
                axis_setEnable(ucAxis, false);
            s_SMachineStatus.eState = MC_STATE_RESETTING;
            s_unWaitTicks = 0;
            s_bResetIssued = false;
        }
        return;
    }

    if (s_SMachineStatus.eState == MC_STATE_RESETTING)
    {
        handleResettingState(bLostFrame);
        return;
    }

    if (eCommand == MC_CMD_ENABLE && s_SMachineStatus.eState == MC_STATE_NOT_READY)
    {
        s_SMachineStatus.eState = MC_STATE_ENABLING;
        s_unWaitTicks = 0;
        s_SMachineStatus.unReadyTicks = 0;
        memset(s_bAligned, 0, sizeof(s_bAligned));
    }
    if (s_SMachineStatus.eState == MC_STATE_NOT_READY)
        return; /* zxr: 未使能时允许启动首拍等历史丢帧；启动当拍及使能期间不忽略新丢帧。 */
    if (bLostFrame || axis_isBusError())
    {
        stopMachineGroup(MC_ERROR_BUS, MACHINE_AXIS_NONE);
        return;
    }
    if (axis_getCycleIntervalUs() > 2U * AXIS_CYCLE_PERIOD_US)
    {
        stopMachineGroup(MC_ERROR_CYCLE, MACHINE_AXIS_NONE);
        return;
    }

    bool bAllReady = true;
    uint8_t ucNotReadyAxis = MACHINE_AXIS_NONE;
    for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
    {
        if (axis_isFault(ucAxis))
        {
            stopMachineGroup(MC_ERROR_AXIS, ucAxis);
            return;
        }
        bool bReady = axis_isReady(ucAxis);
        if (!bReady && ucNotReadyAxis == MACHINE_AXIS_NONE)
        {
            ucNotReadyAxis = ucAxis;
        }
        bAllReady = bAllReady && bReady;
        if (s_SMachineStatus.eState == MC_STATE_ENABLING)
        {
            /* zxr: 各轴首次就绪时记录与轴层相同的起点；等待结束不再追随实际位置改目标。 */
            if (!bReady)
                s_bAligned[ucAxis] = false;
            else if (!s_bAligned[ucAxis])
            {
                s_SMachineStatus.nTargetPosition[ucAxis] = s_SMachineStatus.nActualPosition[ucAxis];
                s_bAligned[ucAxis] = true;
            }
            axis_setEnable(ucAxis, true);
        }
    }
    if (s_SMachineStatus.eState == MC_STATE_ENABLING)
    {
        s_SMachineStatus.unReadyTicks = bAllReady ? s_SMachineStatus.unReadyTicks + 1U : 0U;
        if (++s_unWaitTicks >= ENABLE_TIMEOUT_TICKS)
            stopMachineGroup(MC_ERROR_ENABLE_TIMEOUT, MACHINE_AXIS_NONE);
        else if (s_SMachineStatus.unReadyTicks >= READY_HOLD_TICKS)
            s_SMachineStatus.eState = MC_STATE_READY;
    }
    else if (!bAllReady)
        stopMachineGroup(MC_ERROR_NOT_READY, ucNotReadyAxis);
}

/* zxr: 这里只测主站服务和使能管理；接入后总体周期的耗时预算由调用方另行测量。 */
void mc_runCycle(void)
{
    uint32_t unStartUs = tim_getTimeUs();
    runCycleStep();
    s_SMachineStatus.unCycleUs = (uint32_t)(tim_getTimeUs() - unStartUs);
    if (s_SMachineStatus.unCycleUs > s_SMachineStatus.unMaxCycleUs)
        s_SMachineStatus.unMaxCycleUs = s_SMachineStatus.unCycleUs;
    if (s_SMachineStatus.eState != MC_STATE_NOT_READY && s_SMachineStatus.unCycleUs >= AXIS_CYCLE_PERIOD_US)
        stopMachineGroup(MC_ERROR_CYCLE, MACHINE_AXIS_NONE);
    publishSnapshot();
}

bool mc_canStep(void) { return s_SMachineStatus.eState == MC_STATE_READY && !s_bStepSubmitted; }

bool mc_submitStep(const int32_t pnDeltaPulse[MACHINE_PHYSICAL_AXIS_COUNT])
{
    if (s_SMachineStatus.eState != MC_STATE_READY)
        return false;
    if (pnDeltaPulse == NULL || s_bStepSubmitted)
    {
        stopMachineGroup(MC_ERROR_STEP, MACHINE_AXIS_NONE);
        publishSnapshot();
        return false;
    }
    int32_t nNextTarget[MACHINE_PHYSICAL_AXIS_COUNT] = {0};
    for (uint8_t i = 0; i < MACHINE_PHYSICAL_AXIS_COUNT; i++)
    {
        if (i >= s_ucActiveAxisCount)
        {
            if (pnDeltaPulse[i] != 0)
            {
                stopMachineGroup(MC_ERROR_STEP, i); /* zxr: XYZ 任务不能静默丢掉未接 Z 轴后变成 XY。 */
                publishSnapshot();
                return false;
            }
            continue;
        }
        int64_t lStep = (int64_t)pnDeltaPulse[i] * s_cDirectionInvert[i];
        int64_t lTarget = (int64_t)s_SMachineStatus.nTargetPosition[i] + lStep;
        if (lStep > s_nStepLimit[i] || lStep < -(int64_t)s_nStepLimit[i] || lTarget > INT32_MAX || lTarget < INT32_MIN)
        {
            stopMachineGroup(MC_ERROR_STEP, i);
            publishSnapshot();
            return false;
        }
        nNextTarget[i] = (int32_t)lTarget;
    }
    /* zxr: 先算完并检查全部轴，再发布；后轴非法时前轴也不会提交。 */
    for (uint8_t ucAxis = 0; ucAxis < s_ucActiveAxisCount; ucAxis++)
    {
        s_SMachineStatus.nTargetPosition[ucAxis] = nNextTarget[ucAxis];
        axis_setTargetPos(ucAxis, nNextTarget[ucAxis]);
    }
    s_bStepSubmitted = true;
    publishSnapshot();
    return true;
}

MACHINE_STATE mc_getState(void) { return s_SMachineStatus.eState; }
MACHINE_ERROR mc_getError(void) { return s_SMachineStatus.eError; }
uint8_t mc_getFaultAxis(void) { return s_SMachineStatus.ucFaultAxis; }
uint8_t mc_getActiveAxisCount(void) { return s_ucActiveAxisCount; }

bool mc_getSnapshot(SMachineControlSnapshot *pSSnapshot)
{
    if (pSSnapshot == NULL)
        return false;

    for (;;)
    {
        uint32_t unSequenceBefore = s_unSnapshotSequence;
        if ((unSequenceBefore & 1U) != 0U)
            continue;
        __asm__ volatile("" ::: "memory");
        *pSSnapshot = s_SSnapshot;
        __asm__ volatile("" ::: "memory");
        if (unSequenceBefore == s_unSnapshotSequence)
            return true;
    }
}

int32_t mc_getTargetPos(uint8_t ucPhysicalAxis) { return ucPhysicalAxis < s_ucActiveAxisCount ? s_SMachineStatus.nTargetPosition[ucPhysicalAxis] : 0; }
int32_t mc_getActualPos(uint8_t ucPhysicalAxis) { return ucPhysicalAxis < s_ucActiveAxisCount ? s_SMachineStatus.nActualPosition[ucPhysicalAxis] : 0; }
