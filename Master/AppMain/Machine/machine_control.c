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
    MachineState_e state;
    MachineError_e error;
    uint8_t fault_axis;
    int32_t target_pos[MACHINE_PHYSICAL_AXIS_COUNT];
    int32_t actual_pos[MACHINE_PHYSICAL_AXIS_COUNT];
    uint32_t ready_ticks;
    uint32_t cycle_us, max_cycle_us;
} MachineControlStatus_t;

/* zxr: 前台改 s_command；s_mc 只观察，一致性读取用 MachineControl_GetSnapshot。 */
static volatile uint32_t s_command;
static volatile MachineControlStatus_t s_mc;
static uint8_t s_active_axis_count;
static int32_t s_step_limit[MACHINE_PHYSICAL_AXIS_COUNT];
static int8_t s_dir_invert[MACHINE_PHYSICAL_AXIS_COUNT];
static bool s_aligned[MACHINE_PHYSICAL_AXIS_COUNT];
static uint32_t s_wait_ticks, s_last_lost_wkc;
static bool s_step_submitted;
static bool s_reset_issued;
static uint32_t s_cycle_count;
static bool s_feedback_valid;
static volatile uint32_t s_snapshot_seq;
static MachineControlSnapshot_t s_snapshot;

static bool IsAxisHardwareFault(uint16_t status_word)
{
    uint16_t fault_bits = status_word & MC_CIA402_MASK_FAULT;
    return fault_bits == MC_CIA402_VAL_FAULT || fault_bits == MC_CIA402_VAL_FAULT_REACTION;
}

static void PublishSnapshot(void)
{
    s_snapshot_seq++;
    __asm__ volatile("" ::: "memory");

    s_snapshot.state = s_mc.state;
    s_snapshot.error = s_mc.error;
    s_snapshot.fault_axis = s_mc.fault_axis;
    s_snapshot.active_axis_count = s_active_axis_count;

    for (uint8_t i = 0; i < MACHINE_PHYSICAL_AXIS_COUNT; i++)
    {
        if (i < s_active_axis_count)
        {
            s_snapshot.target_pos[i] = s_mc.target_pos[i];
            s_snapshot.actual_pos[i] = s_mc.actual_pos[i];
            s_snapshot.status_word[i] = Axis_GetStatusWord(i);
            s_snapshot.following_error[i] = Axis_GetFollowingError(i);
        }
        else
        {
            s_snapshot.target_pos[i] = 0;
            s_snapshot.actual_pos[i] = 0;
            s_snapshot.status_word[i] = 0;
            s_snapshot.following_error[i] = 0;
        }
    }

    s_snapshot.ready_ticks = s_mc.ready_ticks;
    s_snapshot.cycle_us = s_mc.cycle_us;
    s_snapshot.max_cycle_us = s_mc.max_cycle_us;
    s_snapshot.lost_wkc_count = Axis_GetLostWkcCount();
    s_snapshot.feedback_valid = s_feedback_valid;
    s_snapshot.cycle_count = s_cycle_count;

    __asm__ volatile("" ::: "memory");
    s_snapshot_seq++;
}

/* zxr: 统一去使能；普通停止不能覆盖故障，复位期间收到去使能须退回故障态保留原错误。 */
static void Stop(MachineError_e error, uint8_t fault_axis)
{
    /* zxr: 主站只撤销轴使能并发布状态，调用方负责停止推进自己的轨迹。 */
    for (uint8_t i = 0; i < AXIS_MAX_NUM; i++)
    {
        Axis_SetEnable(i, false);
        Axis_CancelReset(i);
    }
    s_reset_issued = false;

    if (s_mc.state == MC_STATE_RESETTING && error == MC_ERROR_NONE)
    {
        s_mc.state = MC_STATE_FAULT;
        return;
    }

    if (s_mc.state != MC_STATE_FAULT)
    {
        s_mc.error = error;
        s_mc.fault_axis = (error == MC_ERROR_NONE) ? MACHINE_AXIS_NONE : fault_axis;
        s_mc.state = error == MC_ERROR_NONE ? MC_STATE_NOT_READY : MC_STATE_FAULT;
    }
}

static void HandleResettingState(bool lost_frame)
{
    for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
    {
        Axis_SetEnable(axis, false);
        s_mc.target_pos[axis] = s_mc.actual_pos[axis];
    }

    if (!s_reset_issued)
    {
        bool can_issue = !lost_frame;
        for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
        {
            if ((Axis_GetStatusWord(axis) & MC_CIA402_MASK_BASIC) == MC_CIA402_VAL_OPERATION_ENABLE)
            {
                can_issue = false;
                break;
            }
        }

        if (can_issue)
        {
            bool all_resetting = true;
            for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
            {
                Axis_ResetFault(axis);
                all_resetting = all_resetting && Axis_IsResetting(axis);
            }
            if (all_resetting)
            {
                s_reset_issued = true;
            }
        }
    }
    else
    {
        bool all_finished = true;
        for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
        {
            if (Axis_IsResetting(axis))
            {
                all_finished = false;
                break;
            }
        }

        if (all_finished)
        {
            if (lost_frame || Axis_IsBusError())
            {
                Stop(MC_ERROR_BUS, MACHINE_AXIS_NONE);
                return;
            }
            for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
            {
                uint16_t sw = Axis_GetStatusWord(axis);
                bool still_op = (sw & MC_CIA402_MASK_BASIC) == MC_CIA402_VAL_OPERATION_ENABLE;
                if (Axis_IsFault(axis) || IsAxisHardwareFault(sw) || still_op)
                {
                    Stop(MC_ERROR_AXIS, axis);
                    return;
                }
            }

            s_mc.error = MC_ERROR_NONE;
            s_mc.fault_axis = MACHINE_AXIS_NONE;
            s_mc.ready_ticks = 0;
            s_reset_issued = false;
            s_mc.state = MC_STATE_NOT_READY;
            return;
        }
    }

    if (++s_wait_ticks >= RESET_TIMEOUT_TICKS)
    {
        if (lost_frame || Axis_IsBusError())
        {
            Stop(MC_ERROR_BUS, MACHINE_AXIS_NONE);
            return;
        }
        for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
        {
            if (Axis_IsFault(axis) || IsAxisHardwareFault(Axis_GetStatusWord(axis)))
            {
                Stop(MC_ERROR_AXIS, axis);
                return;
            }
        }
        Stop(s_mc.error != MC_ERROR_NONE ? s_mc.error : MC_ERROR_AXIS, s_mc.fault_axis);
    }
}

/* 物理轴号沿用轴层索引，台架配置必须与在线的驱动轴逐一对应。 */
void MachineControl_Init(void)
{
    memset((void *)&s_mc, 0, sizeof(s_mc));
    s_mc.fault_axis = MACHINE_AXIS_NONE;
    s_command = MC_CMD_NONE;
    s_wait_ticks = 0;
    s_last_lost_wkc = Axis_GetLostWkcCount();
    s_step_submitted = false;
    s_reset_issued = false;
    s_cycle_count = 0;
    s_feedback_valid = false;
    s_snapshot_seq = 0;
    memset(s_aligned, 0, sizeof(s_aligned));
    memset(s_dir_invert, 0, sizeof(s_dir_invert));
    memset(s_step_limit, 0, sizeof(s_step_limit));
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_active_axis_count = 0;

    uint8_t active_axes = Axis_GetActiveCount();
    if (active_axes == 0 || active_axes > AXIS_MAX_NUM || active_axes > MACHINE_PHYSICAL_AXIS_COUNT || Machine_GetAxisConfig(active_axes) != NULL)
    {
        Stop(MC_ERROR_CONFIG, MACHINE_AXIS_NONE);
        PublishSnapshot();
        return;
    }

    for (uint8_t axis = 0; axis < active_axes; axis++)
    {
        const MachineAxisConfig_t *cfg = Machine_GetAxisConfig(axis);
        if (!Axis_IsAvailable(axis) || cfg == NULL || (cfg->direction_invert != 1 && cfg->direction_invert != -1))
        {
            Stop(MC_ERROR_CONFIG, axis);
            PublishSnapshot();
            return;
        }

        // 计算单拍步长与跟随误差门槛，并同步下发到底层轴接口
        int32_t following_limit = 0;
        if (!Machine_GetSafetyLimits(axis, AXIS_CYCLE_PERIOD_US, &s_step_limit[axis], &following_limit))
        {
            Stop(MC_ERROR_CONFIG, axis);
            PublishSnapshot();
            return;
        }

        s_dir_invert[axis] = cfg->direction_invert;
        Axis_SetSafetyLimits(axis, s_step_limit[axis], following_limit);
    }

    s_active_axis_count = active_axes;
    Stop(MC_ERROR_NONE, MACHINE_AXIS_NONE);
    PublishSnapshot();
}

/* zxr: 单生产者发布一个 32 位命令，不带多字段载荷；TIM3 消费后清零。
 * 运动数组只由同一周期上下文调用 SubmitStep，不经过此命令槽。 */
bool MachineControl_Request(MachineCommand_e command)
{
    if (command < MC_CMD_ENABLE || command > MC_CMD_RESET || s_command != MC_CMD_NONE)
        return false;
    s_command = command;
    return true;
}

static void CycleStep(void)
{
    MachineCommand_e command = (MachineCommand_e)s_command;
    s_command = MC_CMD_NONE;
    s_step_submitted = false;

    if (command == MC_CMD_DISABLE)
        Stop(MC_ERROR_NONE, MACHINE_AXIS_NONE); /* zxr: 去使能先于本拍 PDO 交换；故障锁存不被清除。 */

    Axis_SyncLoop(); /* zxr: 每拍唯一一次收发，处理上一拍暂存目标并取得最新反馈。 */

    s_cycle_count++;
    s_feedback_valid = Axis_IsFeedbackValid();
    bool lost_frame = !s_feedback_valid || (Axis_GetLostWkcCount() != s_last_lost_wkc);
    s_last_lost_wkc = Axis_GetLostWkcCount();

    for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
    {
        s_mc.actual_pos[axis] = Axis_GetActualPos(axis);
    }

    if (s_mc.state == MC_STATE_FAULT)
    {
        if (command == MC_CMD_RESET && s_mc.error != MC_ERROR_CONFIG && s_active_axis_count > 0)
        {
            for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
                Axis_SetEnable(axis, false);
            s_mc.state = MC_STATE_RESETTING;
            s_wait_ticks = 0;
            s_reset_issued = false;
        }
        return;
    }

    if (s_mc.state == MC_STATE_RESETTING)
    {
        HandleResettingState(lost_frame);
        return;
    }

    if (command == MC_CMD_ENABLE && s_mc.state == MC_STATE_NOT_READY)
    {
        s_mc.state = MC_STATE_ENABLING;
        s_wait_ticks = 0;
        s_mc.ready_ticks = 0;
        memset(s_aligned, 0, sizeof(s_aligned));
    }
    if (s_mc.state == MC_STATE_NOT_READY)
        return; /* zxr: 未使能时允许启动首拍等历史丢帧；启动当拍及使能期间不忽略新丢帧。 */
    if (lost_frame || Axis_IsBusError())
    {
        Stop(MC_ERROR_BUS, MACHINE_AXIS_NONE);
        return;
    }
    if (Axis_GetCycleIntervalUs() > 2U * AXIS_CYCLE_PERIOD_US)
    {
        Stop(MC_ERROR_CYCLE, MACHINE_AXIS_NONE);
        return;
    }

    bool all_ready = true;
    uint8_t not_ready_axis = MACHINE_AXIS_NONE;
    for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
    {
        if (Axis_IsFault(axis))
        {
            Stop(MC_ERROR_AXIS, axis);
            return;
        }
        bool ready = Axis_IsReady(axis);
        if (!ready && not_ready_axis == MACHINE_AXIS_NONE)
        {
            not_ready_axis = axis;
        }
        all_ready = all_ready && ready;
        if (s_mc.state == MC_STATE_ENABLING)
        {
            /* zxr: 各轴首次就绪时记录与轴层相同的起点；等待结束不再追随实际位置改目标。 */
            if (!ready)
                s_aligned[axis] = false;
            else if (!s_aligned[axis])
            {
                s_mc.target_pos[axis] = s_mc.actual_pos[axis];
                s_aligned[axis] = true;
            }
            Axis_SetEnable(axis, true);
        }
    }
    if (s_mc.state == MC_STATE_ENABLING)
    {
        s_mc.ready_ticks = all_ready ? s_mc.ready_ticks + 1U : 0U;
        if (++s_wait_ticks >= ENABLE_TIMEOUT_TICKS)
            Stop(MC_ERROR_ENABLE_TIMEOUT, MACHINE_AXIS_NONE);
        else if (s_mc.ready_ticks >= READY_HOLD_TICKS)
            s_mc.state = MC_STATE_READY;
    }
    else if (!all_ready)
        Stop(MC_ERROR_NOT_READY, not_ready_axis);
}

/* zxr: 这里只测主站服务和使能管理；接入后总体周期的耗时预算由调用方另行测量。 */
void MachineControl_Cycle(void)
{
    uint32_t begin = get_us();
    CycleStep();
    s_mc.cycle_us = (uint32_t)(get_us() - begin);
    if (s_mc.cycle_us > s_mc.max_cycle_us)
        s_mc.max_cycle_us = s_mc.cycle_us;
    if (s_mc.state != MC_STATE_NOT_READY && s_mc.cycle_us >= AXIS_CYCLE_PERIOD_US)
        Stop(MC_ERROR_CYCLE, MACHINE_AXIS_NONE);
    PublishSnapshot();
}

bool MachineControl_CanStep(void) { return s_mc.state == MC_STATE_READY && !s_step_submitted; }

bool MachineControl_SubmitStep(const int32_t delta_pulse[MACHINE_PHYSICAL_AXIS_COUNT])
{
    if (s_mc.state != MC_STATE_READY)
        return false;
    if (delta_pulse == NULL || s_step_submitted)
    {
        Stop(MC_ERROR_STEP, MACHINE_AXIS_NONE);
        PublishSnapshot();
        return false;
    }
    int32_t next_target[MACHINE_PHYSICAL_AXIS_COUNT] = {0};
    for (uint8_t i = 0; i < MACHINE_PHYSICAL_AXIS_COUNT; i++)
    {
        if (i >= s_active_axis_count)
        {
            if (delta_pulse[i] != 0)
            {
                Stop(MC_ERROR_STEP, i); /* zxr: XYZ 任务不能静默丢掉未接 Z 轴后变成 XY。 */
                PublishSnapshot();
                return false;
            }
            continue;
        }
        int64_t step = (int64_t)delta_pulse[i] * s_dir_invert[i];
        int64_t target = (int64_t)s_mc.target_pos[i] + step;
        if (step > s_step_limit[i] || step < -(int64_t)s_step_limit[i] || target > INT32_MAX || target < INT32_MIN)
        {
            Stop(MC_ERROR_STEP, i);
            PublishSnapshot();
            return false;
        }
        next_target[i] = (int32_t)target;
    }
    /* zxr: 先算完并检查全部轴，再发布；后轴非法时前轴也不会提交。 */
    for (uint8_t axis = 0; axis < s_active_axis_count; axis++)
    {
        s_mc.target_pos[axis] = next_target[axis];
        Axis_SetTargetPos(axis, next_target[axis]);
    }
    s_step_submitted = true;
    PublishSnapshot();
    return true;
}

MachineState_e MachineControl_GetState(void) { return s_mc.state; }
MachineError_e MachineControl_GetError(void) { return s_mc.error; }
uint8_t MachineControl_GetFaultAxis(void) { return s_mc.fault_axis; }
uint8_t MachineControl_GetActiveAxisCount(void) { return s_active_axis_count; }

bool MachineControl_GetSnapshot(MachineControlSnapshot_t *snapshot)
{
    if (snapshot == NULL)
        return false;

    for (;;)
    {
        uint32_t seq1 = s_snapshot_seq;
        if ((seq1 & 1U) != 0U)
            continue;
        __asm__ volatile("" ::: "memory");
        *snapshot = s_snapshot;
        __asm__ volatile("" ::: "memory");
        if (seq1 == s_snapshot_seq)
            return true;
    }
}

int32_t MachineControl_GetTargetPos(uint8_t physical_axis) { return physical_axis < s_active_axis_count ? s_mc.target_pos[physical_axis] : 0; }
int32_t MachineControl_GetActualPos(uint8_t physical_axis) { return physical_axis < s_active_axis_count ? s_mc.actual_pos[physical_axis] : 0; }
