/**
 * @file         : axis_interface.h
 * @author       : zxr
 * @brief        : SOEM EtherCAT 主站轴层极简统一接口 (7大标准接口)
 */

#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* 轴最大支持数（适配双轴/三轴） */
#define AXIS_MAX_NUM 3

/* zxr: 与当前 TIM3 的 1ms 周期对应，供保护限值换算及超周期统计共用。
 * 此宏不会设置硬件定时器；以后修改 TIM3 周期时，须同步更新此值。 */
#define AXIS_CYCLE_PERIOD_US 1000U

    /* ==================== 7 大核心轴接口 ==================== */

    /**
     * @brief  总线与从站全自动初始化 (PHY链接检测、从站扫描、PDO映射、进入OP模式)
     * @retval true: 初始化并进入 OP 成功, false: 初始化失败
     */
    bool axis_initSystem(void);

    /**
     * @brief  1ms 周期同步函数 (挂载于定时器中断: 收PDO -> 校验WKC -> 更新状态 -> CiA402非阻塞推进 -> 防冲幅值保护 -> 发PDO)
     */
    void axis_runSyncCycle(void);

    /**
     * @brief  轴使能/去使能控制 (非阻塞请求)
     * @param  ucAxisId: 轴索引 (0, 1, 2...)
     * @param  bEnable:  true: 请求使能, false: 请求去使能
     */
    void axis_setEnable(uint8_t ucAxisId, bool bEnable);

    /**
     * @brief  下发轴目标位置 (脉冲单位，内置使能跟踪与步进限幅保护)
     * @param  ucAxisId:           轴索引
     * @param  nTargetPositionPulse:  目标脉冲绝对位置
     */
    void axis_setTargetPos(uint8_t ucAxisId, int32_t nTargetPositionPulse);

    /**
     * @brief  读取轴实际编码器位置 (脉冲单位)
     * @param  ucAxisId: 轴索引
     * @retval int32_t: 编码器当前实际脉冲位置
     */
    int32_t axis_getActualPos(uint8_t ucAxisId);

    /**
     * @brief  查询轴是否处于就绪运行状态 (CiA402 处于 Operation Enabled 且无故障)
     * @param  ucAxisId: 轴索引
     * @retval true: 已就绪可接收插补脉冲, false: 未就绪
     */
    bool axis_isReady(uint8_t ucAxisId);

    /**
     * @brief  轴故障复位请求 (触发 20ms 非阻塞复位脉冲)
     * @param  ucAxisId: 轴索引
     */
    void axis_resetFault(uint8_t ucAxisId);
    void axis_cancelReset(uint8_t ucAxisId);

    /* ==================== 辅助状态与实验指标接口 ==================== */

    bool axis_isGroupReady(void);
    uint16_t axis_getStatusWord(uint8_t ucAxisId);
    int32_t axis_getFollowingError(uint8_t ucAxisId);
    bool axis_isFault(uint8_t ucAxisId);
    bool axis_isResetting(uint8_t ucAxisId);
    /* zxr: 轴循环入口到收发结束的执行耗时，包含期间被其他中断抢占的时间。 */
    uint32_t axis_getCycleTimeUs(void);
    uint32_t axis_getMaxCycleTimeUs(void);
    /* zxr: 相邻 axis_runSyncCycle 入口间隔，正常约 1000us；前两次调用形成首个间隔。
     * 尚无间隔时返回 0；各项独立读取，统计在 axis_initSystem 时重置。 */
    uint32_t axis_getCycleIntervalUs(void);
    uint32_t axis_getMinCycleIntervalUs(void);
    uint32_t axis_getMaxCycleIntervalUs(void);
    /* zxr: 执行耗时达到或超过 1000us 的次数，仅统计，不触发停机。 */
    uint32_t axis_getCycleOverrunCount(void);
    uint32_t axis_getLostWkcCount(void);
    bool axis_isBusError(void);
    bool axis_isFeedbackValid(void);
    uint8_t axis_getActiveCount(void);
    bool axis_isAvailable(uint8_t ucAxisId);
    void axis_setSafetyLimits(uint8_t ucAxisId, int32_t nMaxStepLimit, int32_t nMaxFollowingErrorLimit);

#ifdef __cplusplus
}
#endif

#endif /* AXIS_INTERFACE_H */
