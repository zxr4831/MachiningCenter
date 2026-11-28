/**
 * @file         : machine_control.h
 * @author       : zxr
 * @brief        : 轴接口上一层：整组使能、位置对齐和周期增量提交
 */

#ifndef MACHINE_CONTROL_H
#define MACHINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 按数控物理轴号提交六个槽位，未接驱动的槽位只允许零增量。 */
#define MACHINE_PHYSICAL_AXIS_COUNT 6
#define MACHINE_AXIS_NONE UINT8_MAX /* 无特定故障物理轴时的占位值 */

/* zxr: 调用方在初始化时传入的物理轴执行保护参数；主站不再硬编码具体机床或电机参数。 */
typedef struct
{
    int8_t cDirectionInvert;         /* 运动方向换向系数（仅允许 1 或 -1） */
    int32_t nMaxStepLimit;           /* 单周期最大允许步进增量（指令单位/拍，须 > 0） */
    int32_t nMaxFollowingErrorLimit; /* 最大允许跟随误差（指令单位，须 > 0） */
} SMachineAxisLimits;

/* zxr: 当前是轴执行层状态，READY 表示允许接收增量，不代表程序完成或实际到位。 */
typedef enum
{
    MC_STATE_NOT_READY = 0,
    MC_STATE_ENABLING = 1,
    MC_STATE_READY = 2,
    MC_STATE_RESETTING = 3,
    MC_STATE_FAULT = 4 /* 仅非配置故障允许通过 MC_CMD_RESET 在线复位 */
} MACHINE_STATE;

typedef enum
{
    MC_CMD_NONE = 0,
    MC_CMD_ENABLE = 1,
    MC_CMD_DISABLE = 2, /* zxr: 撤销使能，不是沿轨迹减速暂停，也不替代硬件急停。 */
    MC_CMD_RESET = 3    /* 在线故障复位：仅从非配置故障态触发，完成后回 NOT_READY，不自动使能。 */
} MACHINE_COMMAND;

typedef enum
{
    MC_ERROR_NONE = 0,
    MC_ERROR_CONFIG,
    MC_ERROR_BUS,
    MC_ERROR_AXIS,
    MC_ERROR_NOT_READY,
    MC_ERROR_ENABLE_TIMEOUT,
    MC_ERROR_CYCLE,
    MC_ERROR_STEP /* zxr: 空指针、未映射轴运动、重复提交、步长或位置范围越界。 */
} MACHINE_ERROR;

/* 一致性诊断快照：供非实时任务/串口无锁读取 */
typedef struct
{
    MACHINE_STATE eState;
    MACHINE_ERROR eError;
    uint8_t ucFaultAxis; /* 触发故障的物理轴号；无特定轴时为 MACHINE_AXIS_NONE */
    uint8_t ucActiveAxisCount;
    int32_t nTargetPosition[MACHINE_PHYSICAL_AXIS_COUNT];
    int32_t nActualPosition[MACHINE_PHYSICAL_AXIS_COUNT];
    uint16_t usStatusWord[MACHINE_PHYSICAL_AXIS_COUNT];
    int32_t nFollowingError[MACHINE_PHYSICAL_AXIS_COUNT];
    uint32_t unReadyTicks;
    uint32_t unCycleUs;
    uint32_t unMaxCycleUs;
    uint32_t unLostWkcCount;
    bool bFeedbackValid;   /* 本拍输入 PDO 反馈是否有效；false 表示尚未同步或本拍丢帧 */
    uint32_t unCycleCount; /* mc_runCycle 累计拍号；同拍 mc_submitStep 发布快照沿用该编号 */
} SMachineControlSnapshot;

/* zxr: axis_initSystem 成功后、TIM3 启动前调用；由调用方传入各物理轴保护参数，不自动使能。 */
void mc_init(const SMachineAxisLimits *pSAxisLimits, uint8_t ucAxisCount);
/* zxr: 外部1ms周期每拍调用一次，内部唯一一次 axis_runSyncCycle；故障、暂停、零增量时也要调用。
 * 返回后在同一周期上下文检查 mc_canStep，再计算并提交整组结果；主站不调用上层算法。 */
void mc_runCycle(void);
/* zxr: 前台发使能/去使能/复位请求；返回 true 仅表示命令已入槽，不代表周期任务必然受理
 * (例如 MC_ERROR_CONFIG 下 MC_CMD_RESET 入槽后仍会被拒绝)。实际状态与复位结果以快照为准；槽忙返回 false。 */
bool mc_requestCommand(MACHINE_COMMAND eCommand);
/* zxr: mc_runCycle 返回后为 true 才允许提交增量。本拍最多提交一次，故障/未就绪返回 false。 */
bool mc_canStep(void);
/* zxr: 数组下标为从零开始的物理轴号。增量用驱动器指令单位，正负按机床轴方向，本层换向一次。
 * 成功仅表示整组目标已暂存，下一拍 axis_runSyncCycle 仍会校验后发送，不表示实际到位。
 * 仅限同一周期上下文，六个槽位均须赋值；未接驱动的物理轴只允许 0。
 * 返回 false 或后续就绪丢失时，调用方必须处理任务中止/恢复，不能丢弃该拍后继续旧轨迹。
 * 当前检查整数范围和步长；回零/机床坐标尚未接入，不把启动位置当作真实软限位原点。 */
bool mc_submitStep(const int32_t pnDeltaPulse[MACHINE_PHYSICAL_AXIS_COUNT]);

/* 单项诊断与整组一致性快照 */
MACHINE_STATE mc_getState(void);
MACHINE_ERROR mc_getError(void);
uint8_t mc_getFaultAxis(void);
uint8_t mc_getActiveAxisCount(void);
bool mc_getSnapshot(SMachineControlSnapshot *pSSnapshot);
/* 按同一物理轴号读取驱动器有符号原始计数。反馈位移转为机床方向时使用该物理轴的方向系数。 */
int32_t mc_getTargetPos(uint8_t ucPhysicalAxis);
int32_t mc_getActualPos(uint8_t ucPhysicalAxis);

#endif /* MACHINE_CONTROL_H */
