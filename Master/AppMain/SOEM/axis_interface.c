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
    PdoOutput *pPdoOut; // PDO 输出指针（初始化时缓存）
    PdoInput *pPdoIn;   // PDO 输入指针（初始化时缓存）

    int32_t cmd_target_pos;    // 外部请求目标位置（脉冲）
    int32_t active_target_pos; // 实际下发目标位置（脉冲）
    int32_t last_target_pos;   // 上一周期下发目标位置
    int32_t actual_pos;        // 当前实际位置（脉冲）
    int32_t following_error;   // 跟随误差（脉冲）

    uint16_t status_word;  // 当前状态字
    uint16_t control_word; // 当前控制字
    uint8_t current_mode;  // 当前控制模式

    bool enable_cmd;          // 外部使能请求标志
    bool reset_cmd;           // 外部故障复位请求标志
    uint16_t reset_pulse_cnt; // 复位脉冲维持计时器

    uint16_t startup_reset_ticks; // 开机自愈计时器
    bool startup_reset_done;      // 开机自愈完成标志

    bool is_ready;  // CiA402 是否处于 Operation Enabled
    bool was_ready; // 上一周期就绪状态（用于边沿检测防位置跳变）
    bool is_fault;  // 是否处于故障态

    int32_t max_step_limit;          // 单周期最大允许步进增量（脉冲/ms）
    int32_t max_following_err_limit; // 最大允许跟随误差（脉冲）
} AxisDriver_t;

/* 内部静态变量 */
static AxisDriver_t s_axisDrivers[AXIS_MAX_NUM];
static uint8_t s_active_axis_count = 0;
static bool s_is_system_inited = false;
static uint32_t s_lost_wkc_count = 0;
static uint32_t s_consecutive_lost_wkc = 0;
static bool s_bus_lost = false;
static bool s_last_feedback_valid = false;

/* zxr: 统计量由周期中断更新、前台读取；volatile 保证每次读取当前值。 */
static volatile uint32_t s_last_cycle_us = 0;
static volatile uint32_t s_max_cycle_us = 0;
static volatile uint32_t s_last_cycle_interval_us = 0;
static volatile uint32_t s_min_cycle_interval_us = UINT32_MAX;
static volatile uint32_t s_max_cycle_interval_us = 0;
static volatile uint32_t s_cycle_overrun_count = 0;
static uint32_t s_previous_cycle_start_us = 0;
static bool s_has_previous_cycle = false;

/* zxr: 发送前统一撤销整组使能并恢复本拍之前的目标；故障复位脉冲仍由原状态机处理。 */
static void Axis_StopGroup(const int32_t previous_targets[AXIS_MAX_NUM])
{
    for (uint8_t i = 0; i < s_active_axis_count; i++)
    {
        AxisDriver_t *pAxis = &s_axisDrivers[i];
        pAxis->enable_cmd = false;
        pAxis->is_ready = false;
        pAxis->was_ready = false;
        if (s_consecutive_lost_wkc != 0)
            pAxis->reset_cmd = false;
        if (!pAxis->reset_cmd)
            pAxis->control_word = 0x0000;
        pAxis->cmd_target_pos = pAxis->active_target_pos = pAxis->last_target_pos = previous_targets[i];
        if (pAxis->pPdoOut != NULL)
        {
            pAxis->pPdoOut->usControlWord = pAxis->control_word;
            pAxis->pPdoOut->nTargetPosition = previous_targets[i];
        }
    }
}

/* DWT 硬件周期计数器工具函数 */
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWT_GetCycles(void) { return DWT->CYCCNT; }

/**
 * @brief  总线与从站全自动初始化 (由主循环在启动阶段调用，非周期非中断上下文)
 */
bool Axis_SystemInit(void)
{
    DWT_Init();

    memset(s_axisDrivers, 0, sizeof(s_axisDrivers));
    s_lost_wkc_count = 0;
    s_consecutive_lost_wkc = 0;
    s_bus_lost = false;
    s_last_feedback_valid = false;
    s_last_cycle_us = 0;
    s_max_cycle_us = 0;
    /* zxr: 初始化时重置测量，第一拍只记录起点。 */
    s_last_cycle_interval_us = 0;
    s_min_cycle_interval_us = UINT32_MAX;
    s_max_cycle_interval_us = 0;
    s_cycle_overrun_count = 0;
    s_previous_cycle_start_us = 0;
    s_has_previous_cycle = false;

    /* zxr: 上面的 memset 已将限值清零；应用须在初始化成功后下发，漏配置会被 Axis_SetEnable 拒绝。 */

    if (!masterInit())
    {
        return false;
    }

    if (masterSlaveInfoInit() != 1)
    {
        return false;
    }

    if (ec_slavecount != SLAVE_NUM || SLAVE_NUM > AXIS_MAX_NUM)
    {
        return false;
    }

    s_active_axis_count = (uint8_t)ec_slavecount;

    for (uint8_t i = 0; i < s_active_axis_count; i++)
    {
        uint8_t slave_idx = i + 1; // EtherCAT 从站索引从 1 开始
        s_axisDrivers[i].pPdoOut = (PdoOutput *)ec_slave[slave_idx].outputs;
        s_axisDrivers[i].pPdoIn = (PdoInput *)ec_slave[slave_idx].inputs;

        if (s_axisDrivers[i].pPdoIn == NULL || s_axisDrivers[i].pPdoOut == NULL)
        {
            s_active_axis_count = 0;
            return false;
        }

        s_axisDrivers[i].actual_pos = s_axisDrivers[i].pPdoIn->nActualPosition;
        s_axisDrivers[i].cmd_target_pos = s_axisDrivers[i].actual_pos;
        s_axisDrivers[i].active_target_pos = s_axisDrivers[i].actual_pos;
        s_axisDrivers[i].last_target_pos = s_axisDrivers[i].actual_pos;
        s_axisDrivers[i].following_error = 0;
        s_axisDrivers[i].pPdoOut->nTargetPosition = s_axisDrivers[i].actual_pos;
        s_axisDrivers[i].pPdoOut->usControlWord = 0x0000;
        s_axisDrivers[i].pPdoOut->ucControlMode = CSP_CONTROL_MODE;
    }

    s_is_system_inited = true;
    return true;
}

/**
 * @brief  1ms 周期同步函数 (zxr: 挂载于 TIM3 中断，实际耗时由统计量记录)
 */
void Axis_SyncLoop(void)
{
    uint32_t t_start = DWT_GetCycles();
    /* zxr: 相邻入口时间作无符号差，跨 TIM2 回绕仍可计时；首次调用不计间隔。 */
    uint32_t start_us = get_us();
    if (s_has_previous_cycle)
    {
        uint32_t interval_us = start_us - s_previous_cycle_start_us;
        s_last_cycle_interval_us = interval_us;
        if (interval_us < s_min_cycle_interval_us)
            s_min_cycle_interval_us = interval_us;
        if (interval_us > s_max_cycle_interval_us)
            s_max_cycle_interval_us = interval_us;
    }
    s_previous_cycle_start_us = start_us;
    s_has_previous_cycle = true;
    int32_t previous_targets[AXIS_MAX_NUM] = {0}; /* zxr: 任一轴拒绝目标时，整组本拍均不提交。 */

    // zxr: 1. 接收从站过程数据，使用 200us 超时参数轮询等待。
    int wkc = ec_receive_processdata(EC_RX_TIMEOUT_US);

    // 2. 校验 WKC，检测总线丢帧与掉线
    int expectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    bool feedback_valid = expectedWkc > 0 && wkc == expectedWkc; /* zxr: 丢帧当拍不推进状态或使用旧反馈。 */
    s_last_feedback_valid = feedback_valid;
    if (!feedback_valid)
    {
        s_lost_wkc_count++;
        if (s_consecutive_lost_wkc < 5)
            s_consecutive_lost_wkc++;
        if (s_consecutive_lost_wkc >= 5)
        {
            s_bus_lost = true;
        }
    }
    else
    {
        s_consecutive_lost_wkc = 0;
        /* zxr: 总线报警保持锁存，通信恢复后仍须显式复位及重新使能。 */
    }

    // 3. 逐轴执行 CiA 402 单步非阻塞状态机与指令计算
    for (uint8_t i = 0; i < s_active_axis_count; i++)
    {
        AxisDriver_t *pAxis = &s_axisDrivers[i];
        pAxis->is_ready = false; /* zxr: 每拍重算，模式变化及复位分支不能沿用旧就绪状态。 */
        if (pAxis->pPdoIn == NULL || pAxis->pPdoOut == NULL)
        {
            continue;
        }
        previous_targets[i] = pAxis->pPdoOut->nTargetPosition;
        if (!feedback_valid)
            continue; /* zxr: 保持已发送输出，暂不读取无效 PDO。 */

        // 3.1 读入最新输入 PDO 数据
        pAxis->actual_pos = pAxis->pPdoIn->nActualPosition;
        pAxis->status_word = pAxis->pPdoIn->usStatusWord;
        pAxis->current_mode = pAxis->pPdoIn->ucCurrentMode;

        uint16_t status = pAxis->status_word;
        uint16_t ctrl = 0x0000;

        // 3.2 开机自愈故障复位（实测解决伺服开机报 Err78）
        /* zxr: 锁存后不再自动复位。 */
        if (!pAxis->startup_reset_done && !pAxis->is_fault && !s_bus_lost && (((status & CIA402_MASK_FAULT) == CIA402_VAL_FAULT) || pAxis->startup_reset_ticks != 0))
        {
            uint16_t t = ++pAxis->startup_reset_ticks;
            pAxis->active_target_pos = pAxis->actual_pos;
            pAxis->last_target_pos = pAxis->actual_pos;
            pAxis->cmd_target_pos = pAxis->actual_pos;
            ctrl = (t > 1000 && t <= 1020) ? CIA402_CMD_FAULT_RESET : 0x0000;

            if (t > 1020)
            {
                if ((status & CIA402_MASK_FAULT) != CIA402_VAL_FAULT)
                {
                    pAxis->startup_reset_done = true;
                }
                else if (t >= 2020)
                {
                    pAxis->startup_reset_done = true;
                    pAxis->is_fault = true;
                }
            }

            pAxis->control_word = ctrl;
            pAxis->pPdoOut->usControlWord = ctrl;
            pAxis->pPdoOut->nTargetPosition = pAxis->active_target_pos;
            pAxis->pPdoOut->ucControlMode = CSP_CONTROL_MODE;
            continue;
        }

        // 3.3 外部故障复位脉冲维持处理 (20ms 脉冲宽度)
        if (pAxis->reset_cmd)
        {
            pAxis->was_ready = false; /* zxr: 复位期间保持未就绪，随后重新对齐。 */
            if (++pAxis->reset_pulse_cnt <= 20)
            {
                ctrl = CIA402_CMD_FAULT_RESET;
            }
            else
            {
                pAxis->reset_cmd = false;
                pAxis->reset_pulse_cnt = 0;
                ctrl = 0x0000;
                /* zxr: Fault Reaction 和仍使能均不能确认复位成功。 */
                if ((status & CIA402_MASK_FAULT) != CIA402_VAL_FAULT && (status & CIA402_MASK_FAULT) != 0x000F && (status & CIA402_MASK_BASIC) != CIA402_VAL_OPERATION_ENABLE)
                {
                    pAxis->is_fault = false;
                    s_bus_lost = false; /* zxr: 仅在有效通信、复位完成后清总线报警，不恢复使能。 */
                }
            }

            pAxis->active_target_pos = pAxis->actual_pos;
            pAxis->last_target_pos = pAxis->actual_pos;
            pAxis->cmd_target_pos = pAxis->actual_pos;
            pAxis->control_word = ctrl;
            pAxis->pPdoOut->usControlWord = ctrl;
            pAxis->pPdoOut->nTargetPosition = pAxis->active_target_pos;
            pAxis->pPdoOut->ucControlMode = CSP_CONTROL_MODE;
            continue;
        }

        // 3.4 运行时故障检测
        /* zxr: 锁存软件报警及运行态意外退出。 */
        if (pAxis->is_fault || (status & CIA402_MASK_FAULT) == CIA402_VAL_FAULT || (status & CIA402_MASK_FAULT) == 0x000F
            || (pAxis->was_ready && pAxis->enable_cmd && ((status & CIA402_MASK_BASIC) != CIA402_VAL_OPERATION_ENABLE || pAxis->current_mode != CSP_CONTROL_MODE)))
        {
            pAxis->is_fault = true;
            pAxis->is_ready = false;
            pAxis->was_ready = false;
            pAxis->control_word = 0x0000;
            pAxis->active_target_pos = pAxis->actual_pos;
            pAxis->last_target_pos = pAxis->actual_pos;
            pAxis->cmd_target_pos = pAxis->actual_pos;
            pAxis->pPdoOut->usControlWord = 0x0000;
            pAxis->pPdoOut->nTargetPosition = pAxis->active_target_pos;
            pAxis->pPdoOut->ucControlMode = CSP_CONTROL_MODE;
            continue;
        }

        // 3.5 CiA 402 标准状态机非阻塞逐步推进
        if (pAxis->enable_cmd && !s_bus_lost)
        {
            if ((status & 0x004F) == CIA402_VAL_SWITCH_DISABLED)
            {
                ctrl = CIA402_CMD_SHUTDOWN;
                pAxis->is_ready = false;
            }
            else if ((status & CIA402_MASK_BASIC) == CIA402_VAL_READY_TO_SWITCH)
            {
                ctrl = CIA402_CMD_SWITCH_ON;
                pAxis->is_ready = false;
            }
            else if ((status & CIA402_MASK_BASIC) == CIA402_VAL_SWITCHED_ON)
            {
                ctrl = CIA402_CMD_ENABLE_OP;
                pAxis->is_ready = false;
            }
            else if ((status & CIA402_MASK_BASIC) == CIA402_VAL_OPERATION_ENABLE)
            {
                ctrl = CIA402_CMD_ENABLE_OP;
                if (pAxis->current_mode == CSP_CONTROL_MODE)
                {
                    pAxis->is_ready = true;
                    pAxis->startup_reset_done = true; /* zxr: 首次正常使能后，后续故障只能显式复位。 */
                }
            }
            else
            {
                ctrl = CIA402_CMD_SHUTDOWN;
                pAxis->is_ready = false;
            }
        }
        else
        {
            // 去使能状态
            if ((status & CIA402_MASK_BASIC) == CIA402_VAL_OPERATION_ENABLE)
            {
                ctrl = CIA402_CMD_SWITCH_ON; // 0x0007 退出运行态
            }
            else
            {
                ctrl = CIA402_CMD_SHUTDOWN; // 0x0006
            }
            pAxis->is_ready = false;
        }

        // 3.6 目标位置保护与安全限幅（防使能飞车、防超速冲击、跟随误差监控）
        if (!pAxis->is_ready)
        {
            // 未就绪状态：目标位置强行对齐实际编码器位置
            pAxis->active_target_pos = pAxis->actual_pos;
            pAxis->last_target_pos = pAxis->actual_pos;
            pAxis->cmd_target_pos = pAxis->actual_pos;
            pAxis->following_error = 0;
            pAxis->was_ready = false;
        }
        else
        {
            // 刚就绪跳变瞬态：重置对齐一次，防止使能瞬间发生任何脉冲阶跃
            if (!pAxis->was_ready)
            {
                pAxis->active_target_pos = pAxis->actual_pos;
                pAxis->last_target_pos = pAxis->actual_pos;
                pAxis->cmd_target_pos = pAxis->actual_pos;
                pAxis->following_error = 0;
                pAxis->was_ready = true;
            }
            else
            {
                /* zxr: step_delta 比较本次目标与上次目标，检查指令是否跳变过大；
                 * following_error 比较本次目标与本拍实际反馈，包含通信及跟随滞后的影响。
                 * 两者均用指令单位，正负方向都检查；超限后由下方 Axis_StopGroup 撤销整组使能。
                 * 超限拒绝整拍，不逐轴截短轨迹；用 64 位差值避免有符号溢出。 */
                int64_t step_delta = (int64_t)pAxis->cmd_target_pos - pAxis->last_target_pos;
                int64_t following_error = (int64_t)pAxis->cmd_target_pos - pAxis->actual_pos;
                /* zxr: 保留报警时误差；极端差值饱和到接口可表示范围。 */
                pAxis->following_error = (int32_t)(following_error > INT32_MAX ? INT32_MAX : following_error < INT32_MIN ? INT32_MIN : following_error);
                if (step_delta > pAxis->max_step_limit || step_delta < -(int64_t)pAxis->max_step_limit || following_error > pAxis->max_following_err_limit || following_error < -(int64_t)pAxis->max_following_err_limit)
                {
                    pAxis->is_fault = true;
                    pAxis->is_ready = false;
                    pAxis->was_ready = false;
                    ctrl = 0x0000; // 触发停机
                    pAxis->active_target_pos = pAxis->actual_pos;
                    pAxis->last_target_pos = pAxis->actual_pos;
                    pAxis->cmd_target_pos = pAxis->actual_pos;
                }
                else
                {
                    pAxis->active_target_pos = pAxis->last_target_pos = pAxis->cmd_target_pos;
                }
            }
        }

        // 3.7 写入输出 PDO 缓冲区
        pAxis->control_word = ctrl;
        pAxis->pPdoOut->usControlWord = ctrl;
        pAxis->pPdoOut->nTargetPosition = pAxis->active_target_pos;
        pAxis->pPdoOut->ucControlMode = CSP_CONTROL_MODE;
    }

    /* zxr: 最后统一裁决，后处理轴报警时也撤回前面各轴本拍已写入的目标。 */
    bool stop_group = s_bus_lost;
    for (uint8_t i = 0; i < s_active_axis_count; i++)
        stop_group = stop_group || s_axisDrivers[i].is_fault;
    if (stop_group)
        Axis_StopGroup(previous_targets);

    // 4. 发送过程数据回总线
    ec_send_processdata();

    // 5. 记录执行耗时
    uint32_t t_end = DWT_GetCycles();
    uint32_t cycles_per_us = SystemCoreClock / 1000000UL;
    if (cycles_per_us == 0)
    {
        cycles_per_us = 400; // 默认 400MHz
    }
    s_last_cycle_us = (t_end - t_start) / cycles_per_us;
    if (s_last_cycle_us > s_max_cycle_us)
    {
        s_max_cycle_us = s_last_cycle_us;
    }
    /* zxr: 记录执行耗时达到或超过 1ms 的次数，仅统计，不触发停机。 */
    if (s_last_cycle_us >= AXIS_CYCLE_PERIOD_US)
        s_cycle_overrun_count++;
}

void Axis_SetEnable(uint8_t axis_id, bool enable)
{
    /* zxr: 整组故障或复位未完成时不接收新的使能；去使能仍可随时请求。 */
    if (enable)
    {
        /* zxr: 使能前要求两项限值均大于 0，防止漏调用启动配置后无保护运行；去使能不受此检查限制。 */
        if (axis_id >= s_active_axis_count || s_axisDrivers[axis_id].max_step_limit <= 0 || s_axisDrivers[axis_id].max_following_err_limit <= 0)
            return;
        if (s_bus_lost || s_consecutive_lost_wkc != 0)
            return;
        for (uint8_t i = 0; i < s_active_axis_count; i++)
            if (s_axisDrivers[i].is_fault || s_axisDrivers[i].reset_cmd)
                return;
    }
    if (axis_id < s_active_axis_count)
    {
        s_axisDrivers[axis_id].enable_cmd = enable;
    }
}

void Axis_SetTargetPos(uint8_t axis_id, int32_t target_pos_pulse)
{
    if (axis_id < s_active_axis_count)
    {
        s_axisDrivers[axis_id].cmd_target_pos = target_pos_pulse;
    }
}

int32_t Axis_GetActualPos(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        return s_axisDrivers[axis_id].actual_pos;
    }
    return 0;
}

bool Axis_IsReady(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        /* zxr: 请求改变后立即撤销就绪。 */
        return s_axisDrivers[axis_id].is_ready && s_axisDrivers[axis_id].enable_cmd && !s_axisDrivers[axis_id].is_fault && !s_axisDrivers[axis_id].reset_cmd && !s_bus_lost;
    }
    return false;
}

void Axis_ResetFault(uint8_t axis_id)
{
    /* zxr: 有效反馈且整组退出使能后接受复位；先准备计数，再发布请求。 */
    if (s_consecutive_lost_wkc != 0)
        return;
    for (uint8_t i = 0; i < s_active_axis_count; i++)
        if (s_axisDrivers[i].enable_cmd || (s_axisDrivers[i].status_word & CIA402_MASK_BASIC) == CIA402_VAL_OPERATION_ENABLE)
            return;
    if (axis_id < s_active_axis_count && !s_axisDrivers[axis_id].reset_cmd)
    {
        s_axisDrivers[axis_id].reset_pulse_cnt = 0;
        s_axisDrivers[axis_id].startup_reset_done = true;
        s_axisDrivers[axis_id].reset_cmd = true;
    }
}

void Axis_CancelReset(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        s_axisDrivers[axis_id].reset_cmd = false;
        s_axisDrivers[axis_id].reset_pulse_cnt = 0;
        if (s_axisDrivers[axis_id].pPdoOut != NULL && (s_axisDrivers[axis_id].pPdoOut->usControlWord & CIA402_CMD_FAULT_RESET) != 0)
        {
            s_axisDrivers[axis_id].control_word &= (uint16_t)(~CIA402_CMD_FAULT_RESET);
            s_axisDrivers[axis_id].pPdoOut->usControlWord &= (uint16_t)(~CIA402_CMD_FAULT_RESET);
        }
    }
}

bool Axis_IsAllReady(void)
{
    if (s_active_axis_count == 0 || !s_is_system_inited || s_bus_lost)
    {
        return false;
    }
    for (uint8_t i = 0; i < s_active_axis_count; i++)
    {
        if (!Axis_IsReady(i)) /* zxr: 与单轴就绪判据保持一致。 */
        {
            return false;
        }
    }
    return true;
}

uint16_t Axis_GetStatusWord(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        return s_axisDrivers[axis_id].status_word;
    }
    return 0;
}

int32_t Axis_GetFollowingError(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        return s_axisDrivers[axis_id].following_error;
    }
    return 0;
}

bool Axis_IsFault(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        return s_axisDrivers[axis_id].is_fault;
    }
    return true;
}

bool Axis_IsResetting(uint8_t axis_id)
{
    if (axis_id < s_active_axis_count)
    {
        return s_axisDrivers[axis_id].reset_cmd;
    }
    return false;
}

uint32_t Axis_GetCycleTimeUs(void) { return s_last_cycle_us; }

uint32_t Axis_GetMaxCycleTimeUs(void) { return s_max_cycle_us; }

/* zxr: 返回单项周期统计；尚无有效间隔时返回 0。 */
uint32_t Axis_GetCycleIntervalUs(void) { return s_last_cycle_interval_us; }

uint32_t Axis_GetMinCycleIntervalUs(void) { return s_min_cycle_interval_us == UINT32_MAX ? 0U : s_min_cycle_interval_us; }

uint32_t Axis_GetMaxCycleIntervalUs(void) { return s_max_cycle_interval_us; }

uint32_t Axis_GetCycleOverrunCount(void) { return s_cycle_overrun_count; }

uint32_t Axis_GetLostWkcCount(void) { return s_lost_wkc_count; }

bool Axis_IsBusError(void) { return s_bus_lost; }

bool Axis_IsFeedbackValid(void) { return s_last_feedback_valid; }

uint8_t Axis_GetActiveCount(void) { return s_is_system_inited ? s_active_axis_count : 0; }

bool Axis_IsAvailable(uint8_t axis_id) { return s_is_system_inited && axis_id < s_active_axis_count && s_axisDrivers[axis_id].pPdoIn != NULL && s_axisDrivers[axis_id].pPdoOut != NULL; }

/* zxr: 仅保存主站保护门槛：步长为指令单位/周期，跟随误差为指令单位；供 Axis_SyncLoop 每拍比较。 */
void Axis_SetSafetyLimits(uint8_t axis_id, int32_t max_step_limit, int32_t max_following_err_limit)
{
    if (axis_id < s_active_axis_count)
    {
        if (max_step_limit > 0)
        {
            s_axisDrivers[axis_id].max_step_limit = max_step_limit;
        }
        if (max_following_err_limit > 0)
        {
            s_axisDrivers[axis_id].max_following_err_limit = max_following_err_limit;
        }
    }
}
