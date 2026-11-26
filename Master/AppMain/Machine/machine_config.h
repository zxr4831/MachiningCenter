/**
 * @file         : machine_config.h
 * @author       : zxr
 * @brief        : 主站物理轴方向及裸电机保护参数
 */

#ifndef __MACHINE_CONFIG_H
#define __MACHINE_CONFIG_H

#include <stdbool.h> /* zxr: 参数校验结果。 */
#include <stdint.h>

/* 按数控物理轴号提交六个槽位，未接驱动的槽位只允许零增量。 */
#define MACHINE_PHYSICAL_AXIS_COUNT 6

// 轴参数结构体
typedef struct
{
    int32_t pulses_per_rev;  // 驱动器一圈指令单位数，当前两台已核对为10240
    int8_t direction_invert; // 运动方向 (1: 正向, -1: 反向)
    /* zxr: 裸电机联调保护参数，0 表示尚未配置，禁止使能。 */
    double max_motor_rpm;           // 联调转速基准 (转/分钟)，步长保护另留余量
    double max_following_error_deg; // 最大允许跟随角度误差 (度)
} MachineAxisConfig_t;

/* 按物理轴号取驱动参数，台架未配置的轴返回空指针。 */
const MachineAxisConfig_t *Machine_GetAxisConfig(uint8_t physical_axis);

/* zxr: 同一处计算保护值，机床层预检与轴层最终检查共用；本函数不操作驱动器。 */
bool Machine_GetSafetyLimits(uint8_t physical_axis, uint32_t cycle_us, int32_t *step_limit, int32_t *following_limit);

#endif /* __MACHINE_CONFIG_H */
