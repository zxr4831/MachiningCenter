/**
 * @file         : machine_control.h
 * @author       : zxr
 * @brief        : 轴接口上一层：整组使能、位置对齐和周期增量提交
 */

#ifndef __MACHINE_CONTROL_H
#define __MACHINE_CONTROL_H

#include "machine_config.h"
#include <stdbool.h>
#include <stdint.h>

#define MACHINE_AXIS_NONE UINT8_MAX /* 无特定故障物理轴时的占位值 */

/* zxr: 当前是轴执行层状态，READY 表示允许接收增量，不代表程序完成或实际到位。 */
typedef enum
{
    MC_STATE_NOT_READY = 0,
    MC_STATE_ENABLING = 1,
    MC_STATE_READY = 2,
    MC_STATE_RESETTING = 3,
    MC_STATE_FAULT = 4 /* 仅非配置故障允许通过 MC_CMD_RESET 在线复位 */
} MachineState_e;

typedef enum
{
    MC_CMD_NONE = 0,
    MC_CMD_ENABLE = 1,
    MC_CMD_DISABLE = 2, /* zxr: 撤销使能，不是沿轨迹减速暂停，也不替代硬件急停。 */
    MC_CMD_RESET = 3    /* 在线故障复位：仅从非配置故障态触发，完成后回 NOT_READY，不自动使能。 */
} MachineCommand_e;

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
} MachineError_e;

/* 一致性诊断快照：供非实时任务/串口无锁读取 */
typedef struct
{
    MachineState_e state;
    MachineError_e error;
    uint8_t fault_axis; /* 触发故障的物理轴号；无特定轴时为 MACHINE_AXIS_NONE */
    uint8_t active_axis_count;
    int32_t target_pos[MACHINE_PHYSICAL_AXIS_COUNT];
    int32_t actual_pos[MACHINE_PHYSICAL_AXIS_COUNT];
    uint16_t status_word[MACHINE_PHYSICAL_AXIS_COUNT];
    int32_t following_error[MACHINE_PHYSICAL_AXIS_COUNT];
    uint32_t ready_ticks;
    uint32_t cycle_us;
    uint32_t max_cycle_us;
    uint32_t lost_wkc_count;
    bool feedback_valid;  /* 本拍输入 PDO 反馈是否有效；false 表示尚未同步或本拍丢帧 */
    uint32_t cycle_count; /* MachineControl_Cycle 累计拍号；同拍 SubmitStep 发布快照沿用该编号 */
} MachineControlSnapshot_t;

/* zxr: Axis_SystemInit 成功后、TIM3 启动前调用；初始化保护参数，不自动使能。 */
void MachineControl_Init(void);
/* zxr: 外部1ms周期每拍调用一次，内部唯一一次 Axis_SyncLoop；故障、暂停、零增量时也要调用。
 * 返回后在同一周期上下文检查 CanStep，再计算并提交整组结果；主站不调用上层算法。 */
void MachineControl_Cycle(void);
/* zxr: 前台发使能/去使能/复位请求；返回 true 仅表示命令已入槽，不代表周期任务必然受理
 * (例如 MC_ERROR_CONFIG 下 MC_CMD_RESET 入槽后仍会被拒绝)。实际状态与复位结果以快照为准；槽忙返回 false。 */
bool MachineControl_Request(MachineCommand_e command);
/* zxr: Cycle 返回后为 true 才允许提交增量。本拍最多提交一次，故障/未就绪返回 false。 */
bool MachineControl_CanStep(void);
/* zxr: 数组下标为从零开始的物理轴号。增量用驱动器指令单位，正负按机床轴方向，本层换向一次。
 * 成功仅表示整组目标已暂存，下一拍 Axis_SyncLoop 仍会校验后发送，不表示实际到位。
 * 仅限同一周期上下文，六个槽位均须赋值；未接驱动的物理轴只允许 0。
 * 返回 false 或后续就绪丢失时，调用方必须处理任务中止/恢复，不能丢弃该拍后继续旧轨迹。
 * 当前检查整数范围和步长；回零/机床坐标尚未接入，不把启动位置当作真实软限位原点。 */
bool MachineControl_SubmitStep(const int32_t delta_pulse[MACHINE_PHYSICAL_AXIS_COUNT]);

/* 单项诊断与整组一致性快照 */
MachineState_e MachineControl_GetState(void);
MachineError_e MachineControl_GetError(void);
uint8_t MachineControl_GetFaultAxis(void);
uint8_t MachineControl_GetActiveAxisCount(void);
bool MachineControl_GetSnapshot(MachineControlSnapshot_t *snapshot);
/* 按同一物理轴号读取驱动器有符号原始计数。反馈位移转为机床方向时使用该物理轴的方向系数。 */
int32_t MachineControl_GetTargetPos(uint8_t physical_axis);
int32_t MachineControl_GetActualPos(uint8_t physical_axis);

#endif /* __MACHINE_CONTROL_H */
