/**
 * @file         : DcSync.c
 * @author       : ShadowFury
 * @date         : 2025-09-02
 * @version      : 1.0
 * @brief        : EtherCAT DC同步补偿算法
 */
#include "DcSync.h"

// =============================
// PI 控制器参数配置
// =============================
#define TARGET_PHASE_OFFSET_US 100 // 目标相位偏移 (SM2提前量)
#define PROPORTIONAL_GAIN_DIV 2    // 比例增益分母 (Kp = 1/PROP_DIV)
#define INTEGRAL_GAIN_DIV 3        // 积分增益分母 (Ki = 1/INTEG_DIV)
#define MAX_COMPENSATION_RATIO 0.2 // ±20%周期范围
// =============================

// 控制器状态变量
static int64_t s_lIntegralTerm = 0; // 积分项累加器

/**
 * @brief EtherCAT DC 同步补偿 (PI 控制器实现)
 *
 * @param lRefTimeNs         参考从站的 DC 时间 (单位: ns)
 * @param lCycleTimeUs       期望通信周期 (单位: us)
 * @param[out] pOffsetTimeUs 周期补偿量 (单位: us)
 */
void masterSync(int64_t lRefTimeNs, int64_t lCycleTimeUs, int64_t *pOffsetTimeUs)
{
    // 1. 计算当前相位偏差
    int64_t lPhaseErrorUs = (lRefTimeNs / 1000 + TARGET_PHASE_OFFSET_US) % lCycleTimeUs;

    // 2. 相位归一化 (处理周期边界)
    if (lPhaseErrorUs > (lCycleTimeUs / 2))
    {
        lPhaseErrorUs -= lCycleTimeUs;
    }

    // 3. PI控制器 - 积分项更新
    if (lPhaseErrorUs > 0)
    {
        s_lIntegralTerm++; // 相位超前 → 增加积分量 (需减慢)
    }
    else if (lPhaseErrorUs < 0)
    {
        s_lIntegralTerm--; // 相位落后 → 减少积分量 (需加快)
    }

    // 4. PI控制器 - 输出计算
    int64_t lProportionalTerm = -(lPhaseErrorUs / PROPORTIONAL_GAIN_DIV);
    int64_t lIntegralOutput = -(s_lIntegralTerm / INTEGRAL_GAIN_DIV);

    *pOffsetTimeUs = lProportionalTerm + lIntegralOutput;

    // 5. 输出限幅
    const int64_t lMaxCompensation = lCycleTimeUs * MAX_COMPENSATION_RATIO;
    if (*pOffsetTimeUs > lMaxCompensation)
    {
        *pOffsetTimeUs = lMaxCompensation;
    }
    else if (*pOffsetTimeUs < -lMaxCompensation)
    {
        *pOffsetTimeUs = -lMaxCompensation;
    }
}
