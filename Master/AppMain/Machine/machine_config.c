/**
 * @file         : machine_config.c
 * @author       : zxr
 * @brief        : 主站物理轴保护参数定义
 */

#include "machine_config.h"
#include <math.h>

/* zxr: 步长保护相对于联调转速基准的余量。 */
#define MOTOR_STEP_MARGIN 1.5

/* zxr: 两台裸电机已核对为 10240 指令单位/圈，60rpm、10度是待实测调整的联调初值。
 * 转速用于计算指令步长门槛，另加 MOTOR_STEP_MARGIN 余量；配置本身不会让电机转动。
 * 机床导程、mm换算和软限位由接入方管理；主站只检查指令单位。数组下标为物理轴号。 */
static const MachineAxisConfig_t s_axis_configs[] = {
    [0] =
        {
            .pulses_per_rev = 10240,
            .direction_invert = 1,
            .max_motor_rpm = 60.0,
            .max_following_error_deg = 10.0,
        },
    [1] =
        {
            .pulses_per_rev = 10240,
            .direction_invert = 1,
            .max_motor_rpm = 60.0,
            .max_following_error_deg = 10.0,
        },
};

const MachineAxisConfig_t *Machine_GetAxisConfig(uint8_t physical_axis)
{
    if (physical_axis >= sizeof(s_axis_configs) / sizeof(s_axis_configs[0]))
    {
        return 0;
    }
    return &s_axis_configs[physical_axis];
}

/* zxr: 裸电机联调使用转速、跟随角度换算；初始化时一次计算，两层共用同一值。
 * 60rpm × 10240单位/圈 × 1ms × 1.5余量 → 步长16；
 * 10度 ÷ 360 × 10240单位/圈 → 跟随误差285。ceil最多放宽不到1个单位。 */
bool Machine_GetSafetyLimits(uint8_t physical_axis, uint32_t cycle_us, int32_t *step_limit, int32_t *following_limit)
{
    const MachineAxisConfig_t *cfg = Machine_GetAxisConfig(physical_axis);
    if (cfg == 0 || step_limit == 0 || following_limit == 0 || cycle_us == 0 || cfg->pulses_per_rev <= 0 || !(cfg->max_motor_rpm > 0.0) || !(cfg->max_following_error_deg > 0.0))
        return false;

    double step = cfg->max_motor_rpm / 60.0 * cfg->pulses_per_rev * cycle_us / 1000000.0 * MOTOR_STEP_MARGIN;
    double error = cfg->max_following_error_deg / 360.0 * cfg->pulses_per_rev;
    if (!(step > 0.0 && step <= INT32_MAX && error > 0.0 && error <= INT32_MAX))
        return false;
    *step_limit = (int32_t)ceil(step);
    *following_limit = (int32_t)ceil(error);
    return true;
}
