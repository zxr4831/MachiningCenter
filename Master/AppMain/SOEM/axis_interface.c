/**
 * @file         : axis_interface.c
 * @author       : zxr
 * @brief        : SOEM EtherCAT 主站轴层极简统一接口实现
 */

#include "axis_interface.h"
#include "app_soem.h"
#include "ethercat.h"
#include "slave_config.h"
#include "stm32h7xx_hal.h"
#include "tim.h" /* zxr: 读取 TIM2 微秒计数，测量轴循环开始间隔。 */
#include <stdlib.h>
#include <string.h>

/* CiA 402 状态掩码与命令字定义 */
#define CIA402_MASK_FAULT 0x004F
#define CIA402_VAL_FAULT 0x0008

#define CIA402_MASK_BASIC 0x006F
#define CIA402_VAL_SWITCH_DISABLED 0x0040
#define CIA402_VAL_READY_TO_SWITCH 0x0021
#define CIA402_VAL_SWITCHED_ON 0x0023
#define CIA402_VAL_OPERATION_ENABLE 0x0027

#define CIA402_CMD_SHUTDOWN 0x0006
#define CIA402_CMD_SWITCH_ON 0x0007
#define CIA402_CMD_ENABLE_OP 0x000F
#define CIA402_CMD_FAULT_RESET 0x0080

/* 轴内部驱动控制结构体 */
typedef struct
{
    uint8_t *pucPdoOut;             // PDO 输出缓冲基址（初始化时缓存）
    const uint8_t *pucPdoIn;        // PDO 输入缓冲基址（初始化时缓存）
    const SServoProfile *pSProfile; // 绑定设备描述（PDO 偏移与运行模式）
    uint16_t usSlaveIndex;          // EtherCAT 从站序号 (1 开始)

    int32_t nCommandTargetPosition; // 外部请求目标位置（脉冲）
    int32_t nActiveTargetPosition;  // 实际下发目标位置（脉冲）
    int32_t nLastTargetPosition;    // 上一周期下发目标位置
    int32_t nActualPosition;        // 当前实际位置（脉冲）
    int32_t nFollowingError;        // 跟随误差（脉冲）

    uint16_t usStatusWord;  // 当前状态字
    uint16_t usControlWord; // 当前控制字
    uint8_t ucCurrentMode;  // 当前控制模式

    bool bEnableCommand;        // 外部使能请求标志
    bool bResetCommand;         // 外部故障复位请求标志
    uint16_t usResetPulseCount; // 复位脉冲维持计时器

    uint16_t usStartupResetTicks; // 开机自愈计时器
    bool bStartupResetDone;       // 开机自愈完成标志

    bool bIsReady;  // CiA402 是否处于 Operation Enabled
    bool bWasReady; // 上一周期就绪状态（用于边沿检测防位置跳变）
    bool bIsFault;  // 是否处于故障态

    int32_t nMaxStepLimit;           // 单周期最大允许步进增量（脉冲/ms）
    int32_t nMaxFollowingErrorLimit; // 最大允许跟随误差（脉冲）
} SAxisDriver;

/* 内部静态变量 */
static SAxisDriver s_SAxisDrivers[AXIS_MAX_NUM];
static uint8_t s_ucActiveAxisCount = 0;
static bool s_bIsSystemInitialized = false;
static uint32_t s_unLostWkcCount = 0;
static uint32_t s_unConsecutiveLostWkc = 0;
static bool s_bBusLost = false;
static bool s_bLastFeedbackValid = false;

/* zxr: 统计量由周期中断更新、前台读取；volatile 保证每次读取当前值。 */
static volatile uint32_t s_unLastCycleUs = 0;
static volatile uint32_t s_unMaxCycleUs = 0;
static volatile uint32_t s_unLastCycleIntervalUs = 0;
static volatile uint32_t s_unMinCycleIntervalUs = UINT32_MAX;
static volatile uint32_t s_unMaxCycleIntervalUs = 0;
static volatile uint32_t s_unCycleOverrunCount = 0;
static uint32_t s_unPreviousCycleStartUs = 0;
static bool s_bHasPreviousCycle = false;

static void writeAxisOutputPdo(SAxisDriver *pSAxis, int32_t nTargetPosition, uint16_t usControlWord)
{
    const SServoPdoLayout *pSLayout = &pSAxis->pSProfile->SPdoLayout;
    slavePdo_writeI32(pSAxis->pucPdoOut, pSLayout->usOffsetTargetPosition, nTargetPosition);
    slavePdo_writeU16(pSAxis->pucPdoOut, pSLayout->usOffsetControlWord, usControlWord);
    slavePdo_writeU8(pSAxis->pucPdoOut, pSLayout->usOffsetControlMode, pSAxis->pSProfile->ucControlMode);
}

/* zxr: 发送前统一撤销整组使能并恢复本拍之前的目标；故障复位脉冲仍由原状态机处理。 */
static void stopAxisGroup(const int32_t pnPreviousTargets[AXIS_MAX_NUM])
{
    for (uint8_t i = 0; i < s_ucActiveAxisCount; i++)
    {
        SAxisDriver *pSAxis = &s_SAxisDrivers[i];
        pSAxis->bEnableCommand = false;
        pSAxis->bIsReady = false;
        pSAxis->bWasReady = false;
        if (s_unConsecutiveLostWkc != 0)
            pSAxis->bResetCommand = false;
        if (!pSAxis->bResetCommand)
            pSAxis->usControlWord = 0x0000;
        pSAxis->nCommandTargetPosition = pSAxis->nActiveTargetPosition = pSAxis->nLastTargetPosition = pnPreviousTargets[i];
        if (pSAxis->pucPdoOut != NULL && pSAxis->pSProfile != NULL)
        {
            const SServoPdoLayout *pSLayout = &pSAxis->pSProfile->SPdoLayout;
            slavePdo_writeU16(pSAxis->pucPdoOut, pSLayout->usOffsetControlWord, pSAxis->usControlWord);
            slavePdo_writeI32(pSAxis->pucPdoOut, pSLayout->usOffsetTargetPosition, pnPreviousTargets[i]);
        }
    }
}

/* DWT 硬件周期计数器工具函数 */
static inline void initCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t getCycleCount(void) { return DWT->CYCCNT; }

/**
 * @brief  总线与从站全自动初始化 (由主循环在启动阶段调用，非周期非中断上下文)
 */
bool axis_initSystem(void)
{
    initCycleCounter();

    memset(s_SAxisDrivers, 0, sizeof(s_SAxisDrivers));
    s_unLostWkcCount = 0;
    s_unConsecutiveLostWkc = 0;
    s_bBusLost = false;
    s_bLastFeedbackValid = false;
    s_unLastCycleUs = 0;
    s_unMaxCycleUs = 0;
    /* zxr: 初始化时重置测量，第一拍只记录起点。 */
    s_unLastCycleIntervalUs = 0;
    s_unMinCycleIntervalUs = UINT32_MAX;
    s_unMaxCycleIntervalUs = 0;
    s_unCycleOverrunCount = 0;
    s_unPreviousCycleStartUs = 0;
    s_bHasPreviousCycle = false;

    /* zxr: 上面的 memset 已将限值清零；应用须在初始化成功后下发，漏配置会被 axis_setEnable 拒绝。 */

    if (!master_init())
    {
        return false;
    }

    if (master_initSlaves() != 1)
    {
        return false;
    }

    uint8_t ucExpectedCount = slaveConfig_getCount();
    if (ucExpectedCount == 0 || ucExpectedCount > AXIS_MAX_NUM || ec_slavecount != ucExpectedCount)
    {
        return false;
    }

    bool bAxisBound[AXIS_MAX_NUM] = {false};
    for (uint8_t ucSlot = 0; ucSlot < ucExpectedCount; ucSlot++)
    {
        const SSlaveBinding *pSBinding = slaveConfig_getBinding(ucSlot);
        if (pSBinding == NULL || pSBinding->ucPhysicalAxis >= ucExpectedCount || bAxisBound[pSBinding->ucPhysicalAxis] || pSBinding->usSlaveIndex < 1U || pSBinding->usSlaveIndex > (uint16_t)ec_slavecount
            || !slaveProfile_validateLayout(pSBinding->pSProfile))
        {
            memset(s_SAxisDrivers, 0, sizeof(s_SAxisDrivers));
            return false;
        }

        const SServoProfile *pSProfile = pSBinding->pSProfile;
        const SServoPdoLayout *pSLayout = &pSProfile->SPdoLayout;
        SAxisDriver *pSAxis = &s_SAxisDrivers[pSBinding->ucPhysicalAxis];

        pSAxis->pucPdoOut = (uint8_t *)ec_slave[pSBinding->usSlaveIndex].outputs;
        pSAxis->pucPdoIn = (const uint8_t *)ec_slave[pSBinding->usSlaveIndex].inputs;
        pSAxis->pSProfile = pSProfile;
        pSAxis->usSlaveIndex = pSBinding->usSlaveIndex;

        if (pSAxis->pucPdoIn == NULL || pSAxis->pucPdoOut == NULL)
        {
            memset(s_SAxisDrivers, 0, sizeof(s_SAxisDrivers));
            return false;
        }

        bAxisBound[pSBinding->ucPhysicalAxis] = true;
        pSAxis->nActualPosition = slavePdo_readI32(pSAxis->pucPdoIn, pSLayout->usOffsetActualPosition);
        pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
        pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
        pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
        pSAxis->nFollowingError = 0;
        writeAxisOutputPdo(pSAxis, pSAxis->nActualPosition, 0x0000U);
    }

    s_ucActiveAxisCount = ucExpectedCount;
    s_bIsSystemInitialized = true;
    return true;
}

/**
 * @brief  1ms 周期同步函数 (zxr: 挂载于 TIM3 中断，实际耗时由统计量记录)
 */
void axis_runSyncCycle(void)
{
    uint32_t unStartCycles = getCycleCount();
    /* zxr: 相邻入口时间作无符号差，跨 TIM2 回绕仍可计时；首次调用不计间隔。 */
    uint32_t unStartUs = tim_getTimeUs();
    if (s_bHasPreviousCycle)
    {
        uint32_t unIntervalUs = unStartUs - s_unPreviousCycleStartUs;
        s_unLastCycleIntervalUs = unIntervalUs;
        if (unIntervalUs < s_unMinCycleIntervalUs)
            s_unMinCycleIntervalUs = unIntervalUs;
        if (unIntervalUs > s_unMaxCycleIntervalUs)
            s_unMaxCycleIntervalUs = unIntervalUs;
    }
    s_unPreviousCycleStartUs = unStartUs;
    s_bHasPreviousCycle = true;
    int32_t nPreviousTargets[AXIS_MAX_NUM] = {0}; /* zxr: 任一轴拒绝目标时，整组本拍均不提交。 */

    // zxr: 1. 接收从站过程数据，使用 200us 超时参数轮询等待。
    int nWkc = ec_receive_processdata(EC_RX_TIMEOUT_US);

    // 2. 校验 WKC，检测总线丢帧与掉线
    int nExpectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    bool bFeedbackValid = nExpectedWkc > 0 && nWkc == nExpectedWkc; /* zxr: 丢帧当拍不推进状态或使用旧反馈。 */
    s_bLastFeedbackValid = bFeedbackValid;
    if (!bFeedbackValid)
    {
        s_unLostWkcCount++;
        if (s_unConsecutiveLostWkc < 5)
            s_unConsecutiveLostWkc++;
        if (s_unConsecutiveLostWkc >= 5)
        {
            s_bBusLost = true;
        }
    }
    else
    {
        s_unConsecutiveLostWkc = 0;
        /* zxr: 总线报警保持锁存，通信恢复后仍须显式复位及重新使能。 */
    }

    // 3. 逐轴执行 CiA 402 单步非阻塞状态机与指令计算
    for (uint8_t i = 0; i < s_ucActiveAxisCount; i++)
    {
        SAxisDriver *pSAxis = &s_SAxisDrivers[i];
        pSAxis->bIsReady = false; /* zxr: 每拍重算，模式变化及复位分支不能沿用旧就绪状态。 */
        if (pSAxis->pucPdoIn == NULL || pSAxis->pucPdoOut == NULL || pSAxis->pSProfile == NULL)
        {
            continue;
        }
        const SServoPdoLayout *pSLayout = &pSAxis->pSProfile->SPdoLayout;
        uint8_t ucExpectedMode = pSAxis->pSProfile->ucControlMode;
        nPreviousTargets[i] = slavePdo_readI32(pSAxis->pucPdoOut, pSLayout->usOffsetTargetPosition);
        if (!bFeedbackValid)
            continue; /* zxr: 保持已发送输出，暂不读取无效 PDO。 */

        // 3.1 读入最新输入 PDO 数据
        pSAxis->nActualPosition = slavePdo_readI32(pSAxis->pucPdoIn, pSLayout->usOffsetActualPosition);
        pSAxis->usStatusWord = slavePdo_readU16(pSAxis->pucPdoIn, pSLayout->usOffsetStatusWord);
        pSAxis->ucCurrentMode = slavePdo_readU8(pSAxis->pucPdoIn, pSLayout->usOffsetCurrentMode);

        uint16_t usStatus = pSAxis->usStatusWord;
        uint16_t usControlWord = 0x0000;

        // 3.2 开机自愈故障复位（实测解决伺服开机报 Err78）
        /* zxr: 锁存后不再自动复位。 */
        if (!pSAxis->bStartupResetDone && !pSAxis->bIsFault && !s_bBusLost && (((usStatus & CIA402_MASK_FAULT) == CIA402_VAL_FAULT) || pSAxis->usStartupResetTicks != 0))
        {
            uint16_t usResetTick = ++pSAxis->usStartupResetTicks;
            pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
            pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
            pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
            usControlWord = (usResetTick > 1000 && usResetTick <= 1020) ? CIA402_CMD_FAULT_RESET : 0x0000;

            if (usResetTick > 1020)
            {
                if ((usStatus & CIA402_MASK_FAULT) != CIA402_VAL_FAULT)
                {
                    pSAxis->bStartupResetDone = true;
                }
                else if (usResetTick >= 2020)
                {
                    pSAxis->bStartupResetDone = true;
                    pSAxis->bIsFault = true;
                }
            }

            pSAxis->usControlWord = usControlWord;
            writeAxisOutputPdo(pSAxis, pSAxis->nActiveTargetPosition, usControlWord);
            continue;
        }

        // 3.3 外部故障复位脉冲维持处理 (20ms 脉冲宽度)
        if (pSAxis->bResetCommand)
        {
            pSAxis->bWasReady = false; /* zxr: 复位期间保持未就绪，随后重新对齐。 */
            if (++pSAxis->usResetPulseCount <= 20)
            {
                usControlWord = CIA402_CMD_FAULT_RESET;
            }
            else
            {
                pSAxis->bResetCommand = false;
                pSAxis->usResetPulseCount = 0;
                usControlWord = 0x0000;
                /* zxr: Fault Reaction 和仍使能均不能确认复位成功。 */
                if ((usStatus & CIA402_MASK_FAULT) != CIA402_VAL_FAULT && (usStatus & CIA402_MASK_FAULT) != 0x000F && (usStatus & CIA402_MASK_BASIC) != CIA402_VAL_OPERATION_ENABLE)
                {
                    pSAxis->bIsFault = false;
                    s_bBusLost = false; /* zxr: 仅在有效通信、复位完成后清总线报警，不恢复使能。 */
                }
            }

            pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
            pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
            pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
            pSAxis->usControlWord = usControlWord;
            writeAxisOutputPdo(pSAxis, pSAxis->nActiveTargetPosition, usControlWord);
            continue;
        }

        // 3.4 运行时故障检测
        /* zxr: 锁存软件报警及运行态意外退出。 */
        if (pSAxis->bIsFault || (usStatus & CIA402_MASK_FAULT) == CIA402_VAL_FAULT || (usStatus & CIA402_MASK_FAULT) == 0x000F
            || (pSAxis->bWasReady && pSAxis->bEnableCommand && ((usStatus & CIA402_MASK_BASIC) != CIA402_VAL_OPERATION_ENABLE || pSAxis->ucCurrentMode != ucExpectedMode)))
        {
            pSAxis->bIsFault = true;
            pSAxis->bIsReady = false;
            pSAxis->bWasReady = false;
            pSAxis->usControlWord = 0x0000;
            pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
            pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
            pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
            writeAxisOutputPdo(pSAxis, pSAxis->nActiveTargetPosition, 0x0000U);
            continue;
        }

        // 3.5 CiA 402 标准状态机非阻塞逐步推进
        if (pSAxis->bEnableCommand && !s_bBusLost)
        {
            if ((usStatus & 0x004F) == CIA402_VAL_SWITCH_DISABLED)
            {
                usControlWord = CIA402_CMD_SHUTDOWN;
                pSAxis->bIsReady = false;
            }
            else if ((usStatus & CIA402_MASK_BASIC) == CIA402_VAL_READY_TO_SWITCH)
            {
                usControlWord = CIA402_CMD_SWITCH_ON;
                pSAxis->bIsReady = false;
            }
            else if ((usStatus & CIA402_MASK_BASIC) == CIA402_VAL_SWITCHED_ON)
            {
                usControlWord = CIA402_CMD_ENABLE_OP;
                pSAxis->bIsReady = false;
            }
            else if ((usStatus & CIA402_MASK_BASIC) == CIA402_VAL_OPERATION_ENABLE)
            {
                usControlWord = CIA402_CMD_ENABLE_OP;
                if (pSAxis->ucCurrentMode == ucExpectedMode)
                {
                    pSAxis->bIsReady = true;
                    pSAxis->bStartupResetDone = true; /* zxr: 首次正常使能后，后续故障只能显式复位。 */
                }
            }
            else
            {
                usControlWord = CIA402_CMD_SHUTDOWN;
                pSAxis->bIsReady = false;
            }
        }
        else
        {
            // 去使能状态
            if ((usStatus & CIA402_MASK_BASIC) == CIA402_VAL_OPERATION_ENABLE)
            {
                usControlWord = CIA402_CMD_SWITCH_ON; // 0x0007 退出运行态
            }
            else
            {
                usControlWord = CIA402_CMD_SHUTDOWN; // 0x0006
            }
            pSAxis->bIsReady = false;
        }

        // 3.6 目标位置保护与安全限幅（防使能飞车、防超速冲击、跟随误差监控）
        if (!pSAxis->bIsReady)
        {
            // 未就绪状态：目标位置强行对齐实际编码器位置
            pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
            pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
            pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
            pSAxis->nFollowingError = 0;
            pSAxis->bWasReady = false;
        }
        else
        {
            // 刚就绪跳变瞬态：重置对齐一次，防止使能瞬间发生任何脉冲阶跃
            if (!pSAxis->bWasReady)
            {
                pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
                pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
                pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
                pSAxis->nFollowingError = 0;
                pSAxis->bWasReady = true;
            }
            else
            {
                /* zxr: lStepDelta 比较本次目标与上次目标，检查指令是否跳变过大；
                 * following_error 比较本次目标与本拍实际反馈，包含通信及跟随滞后的影响。
                 * 两者均用指令单位，正负方向都检查；超限后由下方 stopAxisGroup 撤销整组使能。
                 * 超限拒绝整拍，不逐轴截短轨迹；用 64 位差值避免有符号溢出。 */
                int64_t lStepDelta = (int64_t)pSAxis->nCommandTargetPosition - pSAxis->nLastTargetPosition;
                int64_t lFollowingError = (int64_t)pSAxis->nCommandTargetPosition - pSAxis->nActualPosition;
                /* zxr: 保留报警时误差；极端差值饱和到接口可表示范围。 */
                pSAxis->nFollowingError = (int32_t)(lFollowingError > INT32_MAX ? INT32_MAX : lFollowingError < INT32_MIN ? INT32_MIN : lFollowingError);
                if (lStepDelta > pSAxis->nMaxStepLimit || lStepDelta < -(int64_t)pSAxis->nMaxStepLimit || lFollowingError > pSAxis->nMaxFollowingErrorLimit || lFollowingError < -(int64_t)pSAxis->nMaxFollowingErrorLimit)
                {
                    pSAxis->bIsFault = true;
                    pSAxis->bIsReady = false;
                    pSAxis->bWasReady = false;
                    usControlWord = 0x0000; // 触发停机
                    pSAxis->nActiveTargetPosition = pSAxis->nActualPosition;
                    pSAxis->nLastTargetPosition = pSAxis->nActualPosition;
                    pSAxis->nCommandTargetPosition = pSAxis->nActualPosition;
                }
                else
                {
                    pSAxis->nActiveTargetPosition = pSAxis->nLastTargetPosition = pSAxis->nCommandTargetPosition;
                }
            }
        }

        // 3.7 写入输出 PDO 缓冲区
        pSAxis->usControlWord = usControlWord;
        writeAxisOutputPdo(pSAxis, pSAxis->nActiveTargetPosition, usControlWord);
    }

    /* zxr: 最后统一裁决，后处理轴报警时也撤回前面各轴本拍已写入的目标。 */
    bool bStopGroup = s_bBusLost;
    for (uint8_t i = 0; i < s_ucActiveAxisCount; i++)
        bStopGroup = bStopGroup || s_SAxisDrivers[i].bIsFault;
    if (bStopGroup)
        stopAxisGroup(nPreviousTargets);

    // 4. 发送过程数据回总线
    ec_send_processdata();

    // 5. 记录执行耗时
    uint32_t unEndCycles = getCycleCount();
    uint32_t unCyclesPerUs = SystemCoreClock / 1000000UL;
    if (unCyclesPerUs == 0)
    {
        unCyclesPerUs = 400; // 默认 400MHz
    }
    s_unLastCycleUs = (unEndCycles - unStartCycles) / unCyclesPerUs;
    if (s_unLastCycleUs > s_unMaxCycleUs)
    {
        s_unMaxCycleUs = s_unLastCycleUs;
    }
    /* zxr: 记录执行耗时达到或超过 1ms 的次数，仅统计，不触发停机。 */
    if (s_unLastCycleUs >= AXIS_CYCLE_PERIOD_US)
        s_unCycleOverrunCount++;
}

void axis_setEnable(uint8_t ucAxisId, bool bEnable)
{
    /* zxr: 整组故障或复位未完成时不接收新的使能；去使能仍可随时请求。 */
    if (bEnable)
    {
        /* zxr: 使能前要求两项限值均大于 0，防止漏调用启动配置后无保护运行；去使能不受此检查限制。 */
        if (ucAxisId >= s_ucActiveAxisCount || s_SAxisDrivers[ucAxisId].nMaxStepLimit <= 0 || s_SAxisDrivers[ucAxisId].nMaxFollowingErrorLimit <= 0)
            return;
        if (s_bBusLost || s_unConsecutiveLostWkc != 0)
            return;
        for (uint8_t i = 0; i < s_ucActiveAxisCount; i++)
            if (s_SAxisDrivers[i].bIsFault || s_SAxisDrivers[i].bResetCommand)
                return;
    }
    if (ucAxisId < s_ucActiveAxisCount)
    {
        s_SAxisDrivers[ucAxisId].bEnableCommand = bEnable;
    }
}

void axis_setTargetPos(uint8_t ucAxisId, int32_t nTargetPositionPulse)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        s_SAxisDrivers[ucAxisId].nCommandTargetPosition = nTargetPositionPulse;
    }
}

int32_t axis_getActualPos(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        return s_SAxisDrivers[ucAxisId].nActualPosition;
    }
    return 0;
}

bool axis_isReady(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        /* zxr: 请求改变后立即撤销就绪。 */
        return s_SAxisDrivers[ucAxisId].bIsReady && s_SAxisDrivers[ucAxisId].bEnableCommand && !s_SAxisDrivers[ucAxisId].bIsFault && !s_SAxisDrivers[ucAxisId].bResetCommand && !s_bBusLost;
    }
    return false;
}

void axis_resetFault(uint8_t ucAxisId)
{
    /* zxr: 有效反馈且整组退出使能后接受复位；先准备计数，再发布请求。 */
    if (s_unConsecutiveLostWkc != 0)
        return;
    for (uint8_t i = 0; i < s_ucActiveAxisCount; i++)
        if (s_SAxisDrivers[i].bEnableCommand || (s_SAxisDrivers[i].usStatusWord & CIA402_MASK_BASIC) == CIA402_VAL_OPERATION_ENABLE)
            return;
    if (ucAxisId < s_ucActiveAxisCount && !s_SAxisDrivers[ucAxisId].bResetCommand)
    {
        s_SAxisDrivers[ucAxisId].usResetPulseCount = 0;
        s_SAxisDrivers[ucAxisId].bStartupResetDone = true;
        s_SAxisDrivers[ucAxisId].bResetCommand = true;
    }
}

void axis_cancelReset(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        SAxisDriver *pSAxis = &s_SAxisDrivers[ucAxisId];
        pSAxis->bResetCommand = false;
        pSAxis->usResetPulseCount = 0;
        if (pSAxis->pucPdoOut != NULL && pSAxis->pSProfile != NULL)
        {
            uint16_t usOffset = pSAxis->pSProfile->SPdoLayout.usOffsetControlWord;
            uint16_t usControlWord = slavePdo_readU16(pSAxis->pucPdoOut, usOffset);
            if ((usControlWord & CIA402_CMD_FAULT_RESET) != 0U)
            {
                pSAxis->usControlWord &= (uint16_t)(~CIA402_CMD_FAULT_RESET);
                slavePdo_writeU16(pSAxis->pucPdoOut, usOffset, (uint16_t)(usControlWord & (uint16_t)(~CIA402_CMD_FAULT_RESET)));
            }
        }
    }
}

bool axis_isGroupReady(void)
{
    if (s_ucActiveAxisCount == 0 || !s_bIsSystemInitialized || s_bBusLost)
    {
        return false;
    }
    for (uint8_t i = 0; i < s_ucActiveAxisCount; i++)
    {
        if (!axis_isReady(i)) /* zxr: 与单轴就绪判据保持一致。 */
        {
            return false;
        }
    }
    return true;
}

uint16_t axis_getStatusWord(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        return s_SAxisDrivers[ucAxisId].usStatusWord;
    }
    return 0;
}

int32_t axis_getFollowingError(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        return s_SAxisDrivers[ucAxisId].nFollowingError;
    }
    return 0;
}

bool axis_isFault(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        return s_SAxisDrivers[ucAxisId].bIsFault;
    }
    return true;
}

bool axis_isResetting(uint8_t ucAxisId)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        return s_SAxisDrivers[ucAxisId].bResetCommand;
    }
    return false;
}

uint32_t axis_getCycleTimeUs(void) { return s_unLastCycleUs; }

uint32_t axis_getMaxCycleTimeUs(void) { return s_unMaxCycleUs; }

/* zxr: 返回单项周期统计；尚无有效间隔时返回 0。 */
uint32_t axis_getCycleIntervalUs(void) { return s_unLastCycleIntervalUs; }

uint32_t axis_getMinCycleIntervalUs(void) { return s_unMinCycleIntervalUs == UINT32_MAX ? 0U : s_unMinCycleIntervalUs; }

uint32_t axis_getMaxCycleIntervalUs(void) { return s_unMaxCycleIntervalUs; }

uint32_t axis_getCycleOverrunCount(void) { return s_unCycleOverrunCount; }

uint32_t axis_getLostWkcCount(void) { return s_unLostWkcCount; }

bool axis_isBusError(void) { return s_bBusLost; }

bool axis_isFeedbackValid(void) { return s_bLastFeedbackValid; }

uint8_t axis_getActiveCount(void) { return s_bIsSystemInitialized ? s_ucActiveAxisCount : 0; }

bool axis_isAvailable(uint8_t ucAxisId) { return s_bIsSystemInitialized && ucAxisId < s_ucActiveAxisCount && s_SAxisDrivers[ucAxisId].pucPdoIn != NULL && s_SAxisDrivers[ucAxisId].pucPdoOut != NULL && s_SAxisDrivers[ucAxisId].pSProfile != NULL; }

/* zxr: 仅保存主站保护门槛：步长为指令单位/周期，跟随误差为指令单位；供 axis_runSyncCycle 每拍比较。 */
void axis_setSafetyLimits(uint8_t ucAxisId, int32_t nMaxStepLimit, int32_t nMaxFollowingErrorLimit)
{
    if (ucAxisId < s_ucActiveAxisCount)
    {
        if (nMaxStepLimit > 0)
        {
            s_SAxisDrivers[ucAxisId].nMaxStepLimit = nMaxStepLimit;
        }
        if (nMaxFollowingErrorLimit > 0)
        {
            s_SAxisDrivers[ucAxisId].nMaxFollowingErrorLimit = nMaxFollowingErrorLimit;
        }
    }
}
