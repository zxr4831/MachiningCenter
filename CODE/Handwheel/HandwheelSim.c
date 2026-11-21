/**
Copyright (C) 版权归属沿用原项目，本次注释整理不变更权属。
@file HandwheelSim.c
@author Codex（本次移植与注释整理）
@version 1.5
@date 2026-10-06
@brief 实现 G0 独立轴和 G1/G2/G3 合成路径授权、整周期制动及单次输入分发。
*/
#include "HandwheelSim.h"

#if HANDWHEEL_SIM_DISTANCE_ENABLE
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "Channel.h"
#include "Motion.h"
#include "GeneralFunctions.h"
#include "RunAPI.h"
#include "SysParaAPI.h"
#include "ringbuff.h"
#include "ErrorCompensation.h"
#include "lookAhead.h"
/* [HWSIM] 阅读顺序与控制模型
 * 1. 外层 interruptFuncAPI() 每通道每拍先调用 processHandwheelInput()，然后才由
 *    IRQHandler() 分发到 motionQuickMove()/motionLine()/motionArc()。
 *    采样只增加“允许移动的距离”，不直接输出轴脉冲，也不等于把手轮转速当轴速度。
 * 2. 首次进入程序段：加载原几何 -> 零速绑定约束 -> 等待新输入。
 *    后续每拍：采样授权 -> calculateHandwheelMotionStep() -> 输出检查 -> updateHandwheelMotionState()。
 * 3. G0 在 G0.c 中给每个参与轴维护 SAxisStateArray[i]；各轴独立走自己的段进度。
 *    G1/G2/G3 由 executeHandwheelPathStep() 共用 SPath.SState；全轴共享一个路径进度。
 *    G2/G3 只在原圆弧几何方向上有区别，反摇表示路径进度减小，不改写 G 指令。
 * 4. 核心状态：dSegmentPosition 是已提交的理论进度，dPendingDistance 是剩余授权；
 *    两者之和为授权目标。dBoundaryVelocity 是周期末速度，不是显示的平均速度。
 *    G0 用本轴 mm/度，合成路径用自动公共进度单位；时间统一以 ms 计。
 * 5. preview 只生成候选，不扣距离；全轴输出被软件链接受后才能 commit。
 *    软件提交不等于伺服反馈已到位。停摇、程序暂停、段结束是三种不同事件。
 * 6. G84 等自动旁路段由 isHandwheelAutomaticMoveRequired() 识别，不进入距离规划器；
 *    不改变用户的手轮模拟开关，后续普通段仍可以重新绑定并等待新输入。
 */

/*************************************************
  函数名称：isHandwheelDistanceReplacementNeeded
  函数功能：判断新样本是否替换停摇或旧方向的未执行请求
  输入参数：pGroup - 通道；pState - 标量账本；dInput - 新增距离
  输出参数：无
  返 回 值：TRUE - 从零授权开始接收；FALSE - 累加
  备    注：只用于生产采样，直接距离接口仍保留精确抵消语义
****************************************/
static bool isHandwheelDistanceReplacementNeeded(const struct SHandwheelSimGroup* pGroup,
                                                 const struct SHandwheelSim* pState, double dInput)
{
    /* [HWSIM] 普通低速输入仍按正负距离抵消；只有进入到速限额或停摇阶段后，
     * 新方向/重新摇动才替换旧请求，避免先消耗大量旧余量才响应操作。
     * 这里只决定是否替换授权，当前非零速度仍必须经过规划器减速，不能直接反向。
     */
    return dInput != 0.0 && (pState->bFeedLimitActive || pState->bInputStop) &&
           (pState->bInputStop || (pGroup->cInputDirection != 0 && dInput * pGroup->cInputDirection < 0.0));
}

/** 同一套执行器使用的原插补对象视图，不复制或改变原几何。 */
struct SHandwheelPathView
{
    struct SChannel* pChannel; /**< 所属通道。 */
    struct SPlan* pPlan;       /**< 原段参数单位的规划记录。 */
    SPointDouble* pStart;      /**< 原几何起点。 */
    SPointDouble* pEnd;        /**< 原几何终点。 */
    double* pPulseArray;       /**< 原轴脉冲当量。 */
    double* pSpeed;            /**< 原插补显示速度。 */
    uint8* pStatus;            /**< 原插补生命周期状态。 */
    VAL_STRUCT_TYPE ucValid;   /**< 参与轴掩码。 */
    double dFeedrate;          /**< 程序进给速度。 */
    uint32 unCommand;          /**< 原运动类型及模态。 */
    bool bArc;                 /**< G1 或 G2/G3。 */
};

/*************************************************
  函数名称：selectHandwheelPathData
  函数功能：选择原直线或圆弧对象的公共字段
  输入参数：pChannel - 通道；bArc - 圆弧标志
  输出参数：pView - 借用原对象指针的视图
  返 回 值：无
  备    注：不修改几何和原自动加工参数
****************************************/
static void selectHandwheelPathData(struct SChannel* pChannel, bool bArc, struct SHandwheelPathView* pView)
{
    /* [HWSIM] 统一借用直线 SG01 或圆弧 SG02 的字段，避免复制出另一套几何。
     * pStatus、pSpeed 等是指针，后续执行器通过视图回写原插补状态。
     */
    pView->pChannel = pChannel;
    pView->bArc = bArc;
    pView->pPlan = bArc ? &pChannel->SG02.SPlanData : &pChannel->SG01.SPlanData;
    pView->pStart = bArc ? &pChannel->SG02.SStartPoint : &pChannel->SG01.SStartPoint;
    pView->pEnd = bArc ? &pChannel->SG02.SEndPoint : &pChannel->SG01.SEndPoint;
    pView->pPulseArray = bArc ? pChannel->SG02.dPulseToMm : pChannel->SG01.dPulseToMm;
    pView->pSpeed = bArc ? &pChannel->SG02.dCurrentSpeed : &pChannel->SG01.dCurrentSpeed;
    pView->pStatus = bArc ? &pChannel->SG02.ucInpStatus : &pChannel->SG01.ucInpStatus;
    pView->ucValid = bArc ? pChannel->SG02.ucValStatus : pChannel->SG01.ucValStatus;
    pView->dFeedrate = bArc ? pChannel->SG02.dFeedrate : pChannel->SG01.dFeedrate;
    pView->unCommand = bArc ? pChannel->SG02.unCommandType : pChannel->SG01.unCommandType;
}

/*************************************************
  函数名称：reportHandwheelPathError
  函数功能：锁存合成路径故障，禁止转入旧时间规划
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 报警及候选脉冲
  返 回 值：-1
  备    注：原报警停车流程负责后续停车
****************************************/
static int8 reportHandwheelPathError(struct SChannel* pChannel)
{
    /* [HWSIM] 候选失败后不能改走旧自动算法，否则同一段会被两套规划器接管。
     * 清除本拍候选脉冲并锁存报警；上一成功帧另存在 SPath.sLastPulseArray，
     * 后续 transferHandwheelStateToStopControl() 用它向原报警停车链交接速度信息。
     */
    pChannel->SHandwheelSim.bFault = TRUE;
    memset(pChannel->SRun.sAxisDeltaPulse, 0, sizeof(pChannel->SRun.sAxisDeltaPulse));
    parseRunningError(pChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
    return -1;
}

/*************************************************
  函数名称：limitHandwheelPathValue
  函数功能：校验正有限约束后取最小值
  输入参数：dLimit - 新上限
  输出参数：pValue - 合并后的上限
  返 回 值：TRUE - 有效；FALSE - 参数异常
  备    注：先验证再比较，不能由 fmin 掩盖 NaN
****************************************/
static bool limitHandwheelPathValue(double* pValue, double dLimit)
{
    if (!isfinite(dLimit) || dLimit <= 0.0)
    {
        return FALSE;
    }
    *pValue = fmin(*pValue, dLimit);
    return TRUE;
}

/*************************************************
  函数名称：calculateHandwheelPathPoint
  函数功能：按自动插补公共进度计算各轴实际坐标
  输入参数：pView - 原几何视图；dPosition - 公共进度
  输出参数：pPoint - 全轴理论坐标
  返 回 值：1 - 成功；-1 - 几何失败
  备    注：不单独缩放直径轴坐标，不改变 G2/G3 几何方向
****************************************/
static int calculateHandwheelPathPoint(struct SHandwheelPathView* pView, double dPosition, SPointDouble* pPoint)
{
    struct SHandwheelPath* pPath = &pView->pChannel->SHandwheelSim.SPath;
    uint8 ucAxis;
    *pPoint = *pView->pStart;
    if (pView->bArc)
    {
        return calculateArcInterpolationPoint(dPosition, &pView->pChannel->SG02, pPoint->dArray);
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        if ((getbit(pView->ucValid, ucAxis)) == VAL_VALID)
        {
            pPoint->dArray[ucAxis] += dPosition * pPath->dAxisMovePerProgressArray[ucAxis];
        }
    }
    return 1;
}

/*************************************************
  函数名称：isHandwheelPathAxisValid
  函数功能：验证混合轴映射和连续坐标下的整段软限位范围
  输入参数：pView - 几何视图；ucAxis - 参与轴
  输出参数：无
  返 回 值：TRUE - 有效；FALSE - 拒绝
  备    注：圆弧采用整圆包围范围，宁可提前拒绝也不裁剪轨迹
****************************************/
static bool isHandwheelPathAxisValid(struct SHandwheelPathView* pView, uint8 ucAxis)
{
    struct SChannel* pChannel = pView->pChannel;
    uint8 ucPhysical = pChannel->pMapAxisArray[ucAxis], ucPrevious;
    double dStart = pView->pStart->dArray[ucAxis], dEnd = pView->pEnd->dArray[ucAxis];
    double dRatio = pChannel->SHandwheelSim.SPath.dAxisMovePerProgressArray[ucAxis]; /**< 实际轴比例。 */
    double dLow = fmin(dStart, dEnd), dHigh = fmax(dStart, dEnd);
    if (ucPhysical >= AXIS_PHYSICS_NUM_MAX || !isfinite(dStart) || !isfinite(dEnd) ||
        !isfinite(pView->pPulseArray[ucAxis]) || pView->pPulseArray[ucAxis] <= 0.0 || !isfinite(dRatio) ||
        (g_SSystemPara.SAxisPara[ucPhysical].ucAxisType != LINEAR_AXIS &&
         g_SSystemPara.SAxisPara[ucPhysical].ucAxisType != ROTATION_AXIS))
    {
        return FALSE;
    }
    for (ucPrevious = 0; ucPrevious < ucAxis; ++ucPrevious)
    {
        if ((getbit(pView->ucValid, ucPrevious)) == VAL_VALID && pChannel->pMapAxisArray[ucPrevious] == ucPhysical)
        {
            return FALSE;
        }
    }
    if (pView->bArc && (ucAxis == pChannel->SG02.ucAxis[0] || ucAxis == pChannel->SG02.ucAxis[1]))
    {
        if (g_SSystemPara.SAxisPara[ucPhysical].ucAxisType != LINEAR_AXIS)
        {
            return FALSE;
        }
        /* [HWSIM] 圆弧中途可能超过起终点的坐标范围，不能只检查两个端点。
         * 这里采用圆心 +/- 半径的保守范围，可能拒绝实际未越界的小圆弧，
         * 但不会通过裁剪候选坐标把原圆弧改成另一条轨迹。
         */
        dLow = pChannel->SG02.SCenterPoint.dArray[ucAxis] - fabs(pChannel->SG02.dRadius);
        dHigh = pChannel->SG02.SCenterPoint.dArray[ucAxis] + fabs(pChannel->SG02.dRadius);
    }
    else
    {
        dEnd = dStart + dRatio * pView->pPlan->dDistance;
        dLow = fmin(dStart, dEnd);
        dHigh = fmax(dStart, dEnd);
    }
    return isfinite(dLow) && isfinite(dHigh) &&
           !(g_SSystemPara.MOTOR_WARN[ucPhysical].soft_neg &&
             (!isfinite(g_SSystemPara.MOTOR_WARN[ucPhysical].neg_soft) ||
              dLow < g_SSystemPara.MOTOR_WARN[ucPhysical].neg_soft)) &&
           !(g_SSystemPara.MOTOR_WARN[ucPhysical].soft_pos &&
             (!isfinite(g_SSystemPara.MOTOR_WARN[ucPhysical].pos_soft) ||
              dHigh > g_SSystemPara.MOTOR_WARN[ucPhysical].pos_soft));
}

/*************************************************
  函数名称：prepareHandwheelAxisRatios
  函数功能：复用自动几何并缓存实际轴位移与公共进度的比例
  输入参数：pView - 已加载的原几何
  输出参数：pView - 通道内各轴实际位移比例
  返 回 值：TRUE - 有效；FALSE - 无效
  备    注：G1 当量向量仅在此还原角度，圆弧附加轴沿用原几何单位
****************************************/
static bool prepareHandwheelAxisRatios(struct SHandwheelPathView* pView)
{
    struct SG2* pArc = &pView->pChannel->SG02;
    struct SHandwheelPath* pPath = &pView->pChannel->SHandwheelSim.SPath; /**< 公共进度状态。 */
    uint8 ucAxis;
    if (pView->bArc)
    {
        if (pArc->ucAxis[0] >= AXIS_NUM_MAX || pArc->ucAxis[1] >= AXIS_NUM_MAX || pArc->ucAxis[0] == pArc->ucAxis[1] ||
            (getbit(pView->ucValid, pArc->ucAxis[0])) != VAL_VALID ||
            (getbit(pView->ucValid, pArc->ucAxis[1])) != VAL_VALID || !isfinite(pArc->dRadius) ||
            fabs(pArc->dRadius) <= HWSIM_DISTANCE_EPS || !isfinite(pArc->dRad) || pArc->dRad <= 0.0 ||
            pArc->dRad > 6.283185307180 || (pArc->cArcDir != 1 && pArc->cArcDir != -1))
        {
            return FALSE;
        }
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        uint8 ucPhysical = pView->pChannel->pMapAxisArray[ucAxis]; /**< 实际物理轴。 */
        double dRatio = pView->bArc ? pArc->SStartVector.dArray[ucAxis]
                                    : pView->pChannel->SG01.SStartVector.dArray[ucAxis]; /**< 自动几何比例。 */
        if ((getbit(pView->ucValid, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        if (ucPhysical >= AXIS_PHYSICS_NUM_MAX)
        {
            return FALSE;
        }
        if (!pView->bArc && g_SSystemPara.ucInterpolationMode != INTERPOLATION_LONGAXIS_MODE &&
            g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == ROTATION_AXIS)
        {
            double dScale = g_SSystemPara.SAxisPara[ucPhysical].dCoefOfAngleToLength; /**< 角度当量系数。 */
            if (!isfinite(dScale) || dScale < 0.0)
            {
                return FALSE;
            }
            if (!isZero(dScale))
            {
                dRatio /= dScale;
            }
        }
        pPath->dAxisMovePerProgressArray[ucAxis] = dRatio;
        if (!isHandwheelPathAxisValid(pView, ucAxis))
        {
            return FALSE;
        }
    }
    return TRUE;
}

/*************************************************
  函数名称：applyHandwheelAxisLimitsToPath
  函数功能：把各轴速度及切向和法向加速度约束反算到合成轴
  输入参数：pView - 原几何；ucAxis - 轴号
  输出参数：pSegment - 全段保守运动约束
  返 回 值：TRUE - 有效；FALSE - 参数异常
  备    注：圆弧平面轴各留一半加速度预算给切向和曲率项
****************************************/
static bool applyHandwheelAxisLimitsToPath(struct SHandwheelPathView* pView, uint8 ucAxis,
                                           struct SHandwheelSimSegment* pSegment)
{
    struct SG2* pArc = &pView->pChannel->SG02;
    uint8 ucPhysical = pView->pChannel->pMapAxisArray[ucAxis];
    double dTangent = fabs(pView->pChannel->SHandwheelSim.SPath.dAxisMovePerProgressArray[ucAxis]);
    double dAcc = g_SSystemPara.SAxisPara[ucPhysical].dSysPlanTMaxAcc;
    double dDec = g_SSystemPara.SAxisPara[ucPhysical].dSysPlanTMaxDec;
    double dSpeed = g_SSystemPara.SAxisPara[ucPhysical].dVmax;
    if (!isfinite(dAcc) || dAcc <= 0.0 || !isfinite(dDec) || dDec <= 0.0 || !isfinite(dSpeed) || dSpeed <= 0.0)
    {
        return FALSE;
    }
    if (pView->bArc && (ucAxis == pArc->ucAxis[0] || ucAxis == pArc->ucAxis[1]))
    {
        /* [HWSIM] 曲线运动同时消耗切向加减速和法向加速度预算。
         * dCurvature 把路径速度平方映射成法向加速度；由 a=v*v*k 反算限速，
         * 再把轴预算的一半留给切向变化，避免合成速度合法但某轴加速度超限。
         */
        double dAnglePerProgress = pArc->dRad / pSegment->dLength; /**< 角度对公共进度的比例。 */
        double dCurvature = fabs(pArc->dRadius) * dAnglePerProgress * dAnglePerProgress;
        dTangent = fabs(pArc->dRadius) * pArc->dRad / pSegment->dLength;
        dAcc *= 0.5;
        dDec *= 0.5;
        if (!limitHandwheelPathValue(&pSegment->dSpeedLimit, sqrt(fmin(dAcc, dDec) / dCurvature)))
        {
            return FALSE;
        }
    }
    if (dTangent == 0.0)
    {
        return TRUE;
    }
    /* [HWSIM] dTangent 是该轴位移对路径进度的比例（圆弧取保守上界）。
     * 路径速度乘此比例才是轴速度，所以用轴上限除以该比例反算路径上限。
     * 所有轴逐项取最小值，最终约束必须同时满足每个参与轴。
     */
    return limitHandwheelPathValue(&pSegment->dSpeedLimit, dSpeed / dTangent) &&
           limitHandwheelPathValue(&pSegment->dAcceleration, dAcc / dTangent) &&
           limitHandwheelPathValue(&pSegment->dDeceleration, dDec / dTangent);
}

/*************************************************
  函数名称：calculateHandwheelDisplayDistanceRate
  函数功能：计算显示合成长度相对于归一化进度的导数
  输入参数：pView - 原几何；dFraction - 归一化进度
  输出参数：无
  返 回 值：显示合成距离导数
  备    注：直径仅用于输入度量，不修改执行轨迹
****************************************/
static double calculateHandwheelDisplayDistanceRate(struct SHandwheelPathView* pView, double dFraction)
{
    /* [HWSIM] 输入的“显示距离”和执行几何距离可能不同：直径显示轴的同一
     * 实际位移显示为两倍。这里只计算显示度量的导数，不放大机床轨迹本身。
     * 圆弧切向方向随进度变化，因此不能给整条圆弧简单乘一个固定直径系数。
     */
    struct SG2* pArc = &pView->pChannel->SG02;
    double dRate = 0.0, dX = 0.0, dY = 0.0;
    uint8 ucAxis;
    if (pView->bArc)
    {
        double dAngle = -pArc->cArcDir * pArc->dRad * dFraction;
        double dSin = sin(dAngle), dCos = cos(dAngle); /**< 同一次导数计算复用三角函数。 */
        double dStartX = pArc->SStartPoint.dArray[pArc->ucAxis[0]] - pArc->SCenterPoint.dArray[pArc->ucAxis[0]];
        double dStartY = pArc->SStartPoint.dArray[pArc->ucAxis[1]] - pArc->SCenterPoint.dArray[pArc->ucAxis[1]];
        dX = (-dStartX * dSin - dStartY * dCos) * pArc->dRad;
        dY = (dStartX * dCos - dStartY * dSin) * pArc->dRad;
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        double dComponent;
        if ((getbit(pView->ucValid, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        dComponent = pView->pChannel->SHandwheelSim.SPath.dAxisMovePerProgressArray[ucAxis] * pView->pPlan->dDistance;
        if (pView->bArc && ucAxis == pArc->ucAxis[0])
        {
            dComponent = dX;
        }
        if (pView->bArc && ucAxis == pArc->ucAxis[1])
        {
            dComponent = dY;
        }
        if (g_SSystemPara.SAxisPara[pView->pChannel->pMapAxisArray[ucAxis]].ucAxisType == LINEAR_AXIS &&
            getShowByDiameter(pView->pChannel->pMapAxisArray[ucAxis]) == 1)
        {
            dComponent *= 2.0;
        }
        dRate = hypot(dRate, dComponent);
    }
    return dRate * pView->pChannel->SHandwheelSim.SPath.dInputRateToProgress;
}

/*************************************************
  函数名称：prepareHandwheelInputScale
  函数功能：一次性确定直径显示对公共授权的换算比例
  输入参数：pView - 已校验几何
  输出参数：pView - 常数比例或分拍建表标志
  返 回 值：TRUE - 成功；FALSE - 无效
  备    注：无直径换算时为 1；旋转轴不作直径折半
****************************************/
static bool prepareHandwheelInputScale(struct SHandwheelPathView* pView)
{
    struct SHandwheelPath* pPath = &pView->pChannel->SHandwheelSim.SPath; /**< 当前公共路径。 */
    struct SG2* pArc = &pView->pChannel->SG02;                            /**< 原圆弧几何。 */
    double dBaseSquare = 0.0, dDisplaySquare = 0.0;                       /**< 基础与显示位移平方和。 */
    bool bDiameter = FALSE;                                               /**< 是否存在参与的直径显示直线轴。 */
    uint8 ucAxis;                                                         /**< 逻辑轴索引。 */
    pPath->dInputDistanceScale = 1.0;
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        uint8 ucPhysical = pView->pChannel->pMapAxisArray[ucAxis];                         /**< 物理轴。 */
        double dMove = pPath->dAxisMovePerProgressArray[ucAxis] * pView->pPlan->dDistance; /**< 实际位移。 */
        bool bAxisDiameter;                                                                /**< 本轴是否作直径换算。 */
        if ((getbit(pView->ucValid, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        bAxisDiameter =
            g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == LINEAR_AXIS && getShowByDiameter(ucPhysical) == 1;
        bDiameter = bDiameter || bAxisDiameter;
        if (pView->bArc && (ucAxis == pArc->ucAxis[0] || ucAxis == pArc->ucAxis[1]))
        {
            continue;
        }
        if (!pView->bArc && g_SSystemPara.ucInterpolationMode != INTERPOLATION_LONGAXIS_MODE &&
            g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == ROTATION_AXIS)
        {
            double dScale = g_SSystemPara.SAxisPara[ucPhysical].dCoefOfAngleToLength; /**< 自动当量系数。 */
            dMove *= isZero(dScale) ? 1.0 : dScale;
        }
        dBaseSquare += dMove * dMove;
        dDisplaySquare += dMove * dMove * (bAxisDiameter ? 4.0 : 1.0);
    }
    if (!bDiameter)
    {
        return TRUE;
    }
    if (pView->bArc)
    {
        double dArcLength = fabs(pArc->dRadius) * pArc->dRad; /**< 平面弧长。 */
        dBaseSquare += dArcLength * dArcLength;
        pPath->bUseDistanceMap = TRUE;
        pPath->dInputRateToProgress = pView->pPlan->dDistance / sqrt(dBaseSquare);
        return isfinite(pPath->dInputRateToProgress) && pPath->dInputRateToProgress > 0.0;
    }
    pPath->dInputDistanceScale = sqrt(dDisplaySquare / dBaseSquare);
    return isfinite(pPath->dInputDistanceScale) && pPath->dInputDistanceScale > 0.0;
}

/*************************************************
  函数名称：buildHandwheelInputMapStep
  函数功能：分拍建立直径圆弧的输入换算表
  输入参数：pView - 已验证的几何
  输出参数：pView - 通道内固定大小积分表
  返 回 值：1 - 完成；0 - 下拍继续；-1 - 无效导数
  备    注：每拍最多 HWSIM_MAP_INTERVALS_PER_TICK 个区间，准备期间不运动
****************************************/
static int buildHandwheelInputMapStep(struct SHandwheelPathView* pView)
{
    struct SHandwheelPath* pPath = &pView->pChannel->SHandwheelSim.SPath;
    int nIndex, nLast = pPath->nDistanceMapIndex + HWSIM_MAP_INTERVALS_PER_TICK;
    if (!pPath->bUseDistanceMap)
    {
        return 1;
    }
    if (pPath->nDistanceMapIndex == 0)
    {
        pPath->dMetricArray[0] = 0.0;
        pPath->dMetricRateArray[0] = calculateHandwheelDisplayDistanceRate(pView, 0.0);
        if (!isfinite(pPath->dMetricRateArray[0]) || pPath->dMetricRateArray[0] <= 0.0)
        {
            return -1;
        }
    }
    if (nLast > HWSIM_PATH_MAP_COUNT)
    {
        nLast = HWSIM_PATH_MAP_COUNT;
    }
    for (nIndex = pPath->nDistanceMapIndex + 1; nIndex <= nLast; ++nIndex)
    {
        double dMiddle = calculateHandwheelDisplayDistanceRate(pView, (nIndex - 0.5) / HWSIM_PATH_MAP_COUNT);
        double dRate = calculateHandwheelDisplayDistanceRate(pView, (double)nIndex / HWSIM_PATH_MAP_COUNT);
        pPath->dMetricRateArray[nIndex] = dRate;
        pPath->dMetricArray[nIndex] =
            pPath->dMetricArray[nIndex - 1] +
            (pPath->dMetricRateArray[nIndex - 1] + 4.0 * dMiddle + dRate) / (6.0 * HWSIM_PATH_MAP_COUNT);
        if (!isfinite(pPath->dMetricArray[nIndex]) || !isfinite(dRate) || dRate <= 0.0 ||
            pPath->dMetricArray[nIndex] <= pPath->dMetricArray[nIndex - 1])
        {
            return -1;
        }
    }
    pPath->nDistanceMapIndex = nLast;
    return nLast == HWSIM_PATH_MAP_COUNT ? 1 : 0;
}

/*************************************************
  函数名称：calculateHandwheelDisplayDistance
  函数功能：用三次 Hermite 插值读取授权目标的显示合成距离
  输入参数：pPath - 合成路径；dPosition - 实际进度
  输出参数：无
  返 回 值：显示合成距离
  备    注：段外余量按端点导数延伸，只作抵消账本，不允许越界输出
****************************************/
static double calculateHandwheelDisplayDistance(const struct SHandwheelPath* pPath, double dPosition)
{
    double dFraction = dPosition / pPath->SState.dSegmentLength;
    double dLocal, dLocal2, dLocal3;
    int nIndex;
    /* [HWSIM] 授权可以暂时超出几何段，用于保留反摇抵消关系；段外只线性外推
     * “输入账本”。真正的候选位置由 calculateHandwheelMotionStep() 限制在段内。
     */
    if (dFraction <= 0.0)
    {
        return dFraction * pPath->dMetricRateArray[0];
    }
    if (dFraction >= 1.0)
    {
        return pPath->dMetricArray[HWSIM_PATH_MAP_COUNT] +
               (dFraction - 1.0) * pPath->dMetricRateArray[HWSIM_PATH_MAP_COUNT];
    }
    nIndex = (int)(dFraction * HWSIM_PATH_MAP_COUNT);
    /* [HWSIM] dLocal 是当前表格小区间内的 0..1 位置；三次 Hermite 同时利用
     * 两端累计距离和两端导数，使显示距离映射比简单折线更平滑。
     */
    dLocal = dFraction * HWSIM_PATH_MAP_COUNT - nIndex;
    dLocal2 = dLocal * dLocal;
    dLocal3 = dLocal2 * dLocal;
    return (2.0 * dLocal3 - 3.0 * dLocal2 + 1.0) * pPath->dMetricArray[nIndex] +
           (dLocal3 - 2.0 * dLocal2 + dLocal) * pPath->dMetricRateArray[nIndex] / HWSIM_PATH_MAP_COUNT +
           (-2.0 * dLocal3 + 3.0 * dLocal2) * pPath->dMetricArray[nIndex + 1] +
           (dLocal3 - dLocal2) * pPath->dMetricRateArray[nIndex + 1] / HWSIM_PATH_MAP_COUNT;
}

/*************************************************
  函数名称：findHandwheelAuthorizedProgress
  函数功能：从直径圆弧的累计输入距离反查公共进度
  输入参数：pPath - 已完成的换算表；dTarget - 有限输入目标
  输出参数：无
  返 回 值：对应的公共进度目标
  备    注：先定位表格区间，再固定迭代32次；不调用三角函数
****************************************/
static double findHandwheelAuthorizedProgress(const struct SHandwheelPath* pPath, double dTarget)
{
    double dLength = pPath->SState.dSegmentLength;          /**< 自动插补公共总长。 */
    double dLow, dHigh;                                     /**< 累计表定位后的公共进度区间。 */
    int nLow = 0, nHigh = HWSIM_PATH_MAP_COUNT, nIteration; /**< 有界查找索引。 */
    if (dTarget <= 0.0)
    {
        return dTarget * dLength / pPath->dMetricRateArray[0];
    }
    if (dTarget >= pPath->dMetricArray[HWSIM_PATH_MAP_COUNT])
    {
        return dLength * (1.0 + (dTarget - pPath->dMetricArray[HWSIM_PATH_MAP_COUNT]) /
                                    pPath->dMetricRateArray[HWSIM_PATH_MAP_COUNT]);
    }
    while (nHigh - nLow > 1)
    {
        int nMiddle = (nLow + nHigh) / 2; /**< 累计距离表中点。 */
        if (pPath->dMetricArray[nMiddle] < dTarget)
        {
            nLow = nMiddle;
        }
        else
        {
            nHigh = nMiddle;
        }
    }
    dLow = dLength * nLow / HWSIM_PATH_MAP_COUNT;
    dHigh = dLength * nHigh / HWSIM_PATH_MAP_COUNT;
    for (nIteration = 0; nIteration < 32; ++nIteration)
    {
        double dMiddle = 0.5 * (dLow + dHigh); /**< 单区间进度中点。 */
        if (calculateHandwheelDisplayDistance(pPath, dMiddle) < dTarget)
        {
            dLow = dMiddle;
        }
        else
        {
            dHigh = dMiddle;
        }
    }
    return 0.5 * (dLow + dHigh);
}

/*************************************************
  函数名称：addHandwheelPathAuthorizedDistance
  函数功能：将一次手轮输入换算成公共进度授权
  输入参数：pGroup - 通道状态；dDistance - 有符号输入；bResponse - 是否应用到速替换
  输出参数：pGroup - 剩余公共授权及输入方向
  返 回 值：1 - 成功；-1 - 无效输入
  备    注：普通路径直接换算，只有直径圆弧需要查表
****************************************/
static int addHandwheelPathAuthorizedDistance(struct SHandwheelSimGroup* pGroup, double dDistance, bool bResponse)
{
    struct SHandwheelPath* pPath = &pGroup->SPath; /**< 公共路径。 */
    struct SHandwheelSim* pState = &pPath->SState; /**< 公共授权状态。 */
    double dPending = bResponse && isHandwheelDistanceReplacementNeeded(pGroup, pState, dDistance)
                          ? 0.0
                          : pState->dPendingDistance; /**< 应保留的原授权。 */
    double dPosition;                                 /**< 新授权目标，不是本拍输出坐标。 */
    if (!isfinite(dDistance) || pGroup->bFault)
    {
        return -1;
    }
    if (!pPath->bUseDistanceMap)
    {
        dPosition = pState->dSegmentPosition + dPending + dDistance / pPath->dInputDistanceScale;
    }
    else
    {
        double dTarget = calculateHandwheelDisplayDistance(pPath, pState->dSegmentPosition + dPending) + dDistance;
        if (!isfinite(dTarget))
        {
            return -1;
        }
        dPosition = findHandwheelAuthorizedProgress(pPath, dTarget);
    }
    if (!isfinite(dPosition))
    {
        return -1;
    }
    /* 段首负授权必须保留到端点判定，否则无法请求倒读历史。
     * executeHandwheelPathStep 在没有合法历史时清除此授权，不输出越界坐标。
     */
    pState->dPendingDistance = dPosition - pState->dSegmentPosition;
    pGroup->cInputDirection = dDistance > 0.0 ? INP_GO_AHEAD : INP_GO_BACK;
    return 1;
}

/*************************************************
  函数名称：calculateHandwheelPathLimits
  函数功能：建立系统及各参与轴共同允许的固定合成约束
  输入参数：pView - 几何；pSegment - 已建立长度及位置的段
  输出参数：pSegment - 速度及加减速度
  返 回 值：TRUE - 成功；FALSE - 参数异常
  备    注：约束在绑定时固定，倍率变动采用停稳重绑
****************************************/
static bool calculateHandwheelPathLimits(struct SHandwheelPathView* pView, struct SHandwheelSimSegment* pSegment)
{
    uint8 ucAxis;
    pSegment->dSpeedLimit = g_SSystemPara.dSimulaVmax;
    pSegment->dAcceleration = g_SSystemPara.dSimulaMaxAcc;
    pSegment->dDeceleration = g_SSystemPara.dSimulaMaxDec;
    /* [HWSIM] 先合并模拟参数、系统上限和 G98 程序 F*倍率，再逐轴收紧。
     * G99 的 F 是每转进给，不可直接当速度；其主轴相关上限由
     * updateHandwheelG99SpeedLimit() 在运行时更新，其余几何约束仍固定。
     */
    if (!limitHandwheelPathValue(&pSegment->dSpeedLimit, pSegment->dSpeedLimit) ||
        !limitHandwheelPathValue(&pSegment->dAcceleration, pSegment->dAcceleration) ||
        !limitHandwheelPathValue(&pSegment->dDeceleration, pSegment->dDeceleration) ||
        !limitHandwheelPathValue(&pSegment->dSpeedLimit, g_SSystemPara.dSysMaxVel) ||
        (getBits(pView->unCommand, CMD_FEED_ADDR) == CMD_FEED_PER_MIN &&
         !limitHandwheelPathValue(&pSegment->dSpeedLimit, pView->dFeedrate * pView->pChannel->SRun.dCommonFeedRatio)) ||
        !limitHandwheelPathValue(&pSegment->dAcceleration, g_SSystemPara.dSysPlanTMaxAcc) ||
        !limitHandwheelPathValue(&pSegment->dDeceleration, g_SSystemPara.dSysPlanTMaxDec))
    {
        return FALSE;
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        if ((getbit(pView->ucValid, ucAxis)) == VAL_VALID && !applyHandwheelAxisLimitsToPath(pView, ucAxis, pSegment))
        {
            return FALSE;
        }
    }
    if (pView->bArc)
    {
        struct SG2* pArc = &pView->pChannel->SG02;
        double dCurvature = fabs(pArc->dRadius) * pow(pArc->dRad / pSegment->dLength, 2.0);
        if (!isfinite(g_SSystemPara.dArcMaxNormalAcc) || g_SSystemPara.dArcMaxNormalAcc <= 0.0 ||
            !isfinite(g_SSystemPara.dArcMaxChordError) || g_SSystemPara.dArcMaxChordError <= 0.0)
        {
            return FALSE;
        }
        /* 1-cos(x) <= x*x/2，故此上限保守覆盖每拍弦误差。 */
        return limitHandwheelPathValue(&pSegment->dSpeedLimit, sqrt(g_SSystemPara.dArcMaxNormalAcc / dCurvature)) &&
               limitHandwheelPathValue(&pSegment->dSpeedLimit,
                                       sqrt(8.0 * g_SSystemPara.dArcMaxChordError / dCurvature) / pSegment->dPeriod);
    }
    return TRUE;
}

/*************************************************
  函数名称：isHandwheelEndPointMatched
  函数功能：核对公共进度末端与程序端点，保留旋转轴连续圈数
  输入参数：pView - 当前几何；pPoint - 已算出的连续终点
  输出参数：无
  返 回 值：TRUE - 一致；FALSE - 拒绝
  备    注：只有绝对旋转指令允许相差整圈，相对多圈必须精确匹配
****************************************/
static bool isHandwheelEndPointMatched(struct SHandwheelPathView* pView, const SPointDouble* pPoint)
{
    uint8 ucAxis; /**< 有效逻辑轴。 */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        uint8 ucPhysical = pView->pChannel->pMapAxisArray[ucAxis];            /**< 物理轴。 */
        double dError = pPoint->dArray[ucAxis] - pView->pEnd->dArray[ucAxis]; /**< 端点误差。 */
        if ((getbit(pView->ucValid, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        if (!isfinite(dError))
        {
            return FALSE;
        }
        if (g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == ROTATION_AXIS &&
            !(getbit(pView->pChannel->SPreProcess.SUserData.ucRelaVal, ucAxis)))
        {
            dError = remainder(dError, 360.0);
        }
        if (fabs(dError) > pView->pPulseArray[ucAxis] * 0.5)
        {
            return FALSE;
        }
    }
    return TRUE;
}

/*************************************************
  函数名称：saveHandwheelContinuousGeometry
  函数功能：保存已选定的连续转角，暂停和倒读不重新选择最短方向
  输入参数：pView - 已校验几何；pEnd - 连续终点
  输出参数：pView - 当前输入及对应历史段的连续起终点
  返 回 值：无
  备    注：旋转轴使用相对标记保留整圈；不改变持久化参数布局
****************************************/
static void saveHandwheelContinuousGeometry(struct SHandwheelPathView* pView, const SPointDouble* pEnd)
{
    struct SChannel* pChannel = pView->pChannel;                                   /**< 当前通道。 */
    struct SInput* pInput = &pChannel->SPreProcess.SUserData;                      /**< 当前预处理记录。 */
    struct SToolCompensation* pBlock = NULL;                                       /**< 可用时同步原历史记录。 */
    INDEX_DATA_TYPE nIndex = g_SRingBuff[pChannel->ucChannelNum].nInterpIndex - 1; /**< 本段索引。 */
    VAL_STRUCT_TYPE ucRotary = 0;                                                  /**< 旋转参与轴。 */
    uint8 ucAxis;                                                                  /**< 逻辑轴。 */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        if ((getbit(pView->ucValid, ucAxis)) == VAL_VALID &&
            g_SSystemPara.SAxisPara[pChannel->pMapAxisArray[ucAxis]].ucAxisType == ROTATION_AXIS)
        {
            ucRotary |= 1 << ucAxis;
        }
    }
    if (ucRotary == 0)
    {
        return;
    }
    *pView->pEnd = *pEnd;
    pInput->SStartPoint = *pView->pStart;
    pInput->SEndPoint = *pEnd;
    pInput->ucRelaVal |= ucRotary;
    if (nIndex >= g_SRingBuff[pChannel->ucChannelNum].nBackIndex && nIndex >= 0 &&
        nIndex < g_SRingBuff[pChannel->ucChannelNum].nLookAheadIndex)
    {
        pBlock = getSToolCompPointerByIndex(nIndex, pChannel->ucChannelNum);
    }
    if (pBlock != NULL)
    {
        pBlock->SStartPoint = *pView->pStart;
        pBlock->SEndPoint = *pEnd;
        pBlock->ucRelaVal |= ucRotary;
    }
}

/*************************************************
  函数名称：prepareHandwheelPath
  函数功能：在零速和几何一致条件下绑定合成轴
  输入参数：pView - 已完成原几何加载的视图
  输出参数：pView - 合成状态及原插补运行状态
  返 回 值：1 - 几何有效；负数 - 拒绝
  备    注：复用公共段长，不接管特殊坐标变换；直径圆弧随后分拍准备
****************************************/
static int prepareHandwheelPath(struct SHandwheelPathView* pView)
{
    struct SChannel* pChannel = pView->pChannel;
    struct SHandwheelPath* pPath = &pChannel->SHandwheelSim.SPath;
    struct SHandwheelSimSegment SSegment = {0};
    SPointDouble SPoint;
    uint8 ucAxis;
    /* [HWSIM] 绑定阶段 1：确认原运动已软件零速且几何/模式可接管。
     * 自动旁路段应在调用本函数前分流；绑定失败不是继续用旧规划的理由。
     * 这里的负返回值会保存在 cBindResult，便于区别模式、几何和约束错误。
     */
    if (pChannel->pMapAxisArray == NULL || pChannel->SHandwheelSim.bFault ||
        !isHandwheelSimGroupStopped(&pChannel->SHandwheelSim) || pChannel->SRun.dCurrentSpeed != 0.0 ||
        (getBits(pView->unCommand, CMD_FEED_ADDR) != CMD_FEED_PER_MIN &&
         getBits(pView->unCommand, CMD_FEED_ADDR) != CMD_FEED_PER_R) ||
        getBits(pView->unCommand, CMD_INTERPOLATION_MODE_ADDR) != CMD_INTERPOLATION_COMMON ||
        pChannel->SPreProcess.SUserData.dG84Ratio != 0.0 || !isfinite(pView->dFeedrate) || pView->dFeedrate <= 0.0 ||
        !isfinite(pChannel->SRun.dCommonFeedRatio) || pChannel->SRun.dCommonFeedRatio <= 0.0 ||
        !isfinite(pView->pPlan->dDistance) || pView->pPlan->dDistance <= 0.0)
    {
        return -2;
    }
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE
    if (pChannel->bUseVirtualY)
    {
        return -1;
    }
#endif
#if ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
    return -1;
#endif
    clearHandwheelSimMotion(&pChannel->SHandwheelSim);
    SSegment.dLength = pView->pPlan->dDistance;
    if (!prepareHandwheelAxisRatios(pView))
    {
        return -3;
    }
    SSegment.dPeriod = g_SSystemPara.dInterpolationTime;
    SSegment.dPosition = pView->pPlan->dDistance - pView->pPlan->dCurLeftDist;
    if (!calculateHandwheelPathLimits(pView, &SSegment))
    {
        return -4;
    }
    /* 公共进度与原插补器单位一致，段间不迁移剩余授权。 */
    if (prepareHandwheelMotionSegment(&pPath->SState, &SSegment) < 0)
    {
        return -5;
    }
    pPath->bArc = pView->bArc;
    pPath->dFeedRatio = pChannel->SRun.dCommonFeedRatio;
    pPath->dFixedSpeedLimit = SSegment.dSpeedLimit;
    pPath->dProgramFeed = pView->dFeedrate;
    pPath->bPerRevolution = getBits(pView->unCommand, CMD_FEED_ADDR) == CMD_FEED_PER_R;
    /* [HWSIM] 绑定阶段 4：先确认几何函数算出的段尾与给定终点相符，
     * 再建显示距离映射表，并核对当前理论点与实时指令坐标是否在脉冲容差内。
     * 不允许通过“强行贴到轨迹”掩盖暂停恢复或坐标衔接错误。
     */
    if (calculateHandwheelPathPoint(pView, SSegment.dLength, &SPoint) < 0 ||
        !isHandwheelEndPointMatched(pView, &SPoint))
    {
        return -6;
    }
    /* 连续几何只在端点和当前位置都验证成功后写回。 */
    if (calculateHandwheelPathPoint(pView, SSegment.dPosition, &SPoint) < 0 || !prepareHandwheelInputScale(pView))
    {
        return -6;
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        double dActual;
        if ((getbit(pView->ucValid, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        getSingleAxisRealTimePoint(&dActual, ucAxis, pChannel->pMapAxisArray);
        if (!isfinite(SPoint.dArray[ucAxis]) || !isfinite(dActual) ||
            fabs(SPoint.dArray[ucAxis] - dActual) > pView->pPulseArray[ucAxis] * 1.01)
        {
            return -7;
        }
    }
    if (calculateHandwheelPathPoint(pView, SSegment.dLength, &SPoint) < 0)
    {
        return -6;
    }
    saveHandwheelContinuousGeometry(pView, &SPoint);
    pPath->bPreparing = pPath->bUseDistanceMap;
    pPath->bBound = !pPath->bPreparing;
    /* [HWSIM] 全部校验成功后才开放采样；终点速度设为零，段间不带速拼接。
     * 此标志在校验前不能提前置位，否则采样可能给尚未有效的路径授权。
     */
    *pView->pStatus = pPath->bPreparing ? INP_ASSIGNMENT_START : INP_RUNNING;
    *pView->pSpeed = 0.0;
    pView->pPlan->dVe = pView->pPlan->dPrimalVe = 0.0;
    return 1;
}

/*************************************************
  函数名称：generateHandwheelPathPulses
  函数功能：经原输出链生成全轴脉冲，失败恢复全部软件候选
  输入参数：pView - 原对象；pPoint - 理论候选
  输出参数：pView - 输出成功才更新实时坐标
  返 回 值：1 - 全部成功；-1 - 输出失败
  备    注：距离提交在本函数成功之后执行
****************************************/
static int generateHandwheelPathPulses(struct SHandwheelPathView* pView, SPointDouble* pPoint)
{
    struct _SRun* pRun = &pView->pChannel->SRun;
    SPointDouble SLast = pRun->SLastAbsCoord, SAbs = pRun->SAbsCoord, SActual;
    extern SScrewComp g_SScrewComp[AXIS_PHYSICS_NUM_MAX];
    SScrewComp SCompArray[AXIS_PHYSICS_NUM_MAX];
    uint8 ucAxis;
    int nResult;
    /* [HWSIM] 输出链可能更新补偿记录和实时指令坐标，先保留软件快照。
     * 这里检查的是本拍所需脉冲是否可表示；不是重新计算段内运动规划。
     */
    memcpy(SCompArray, g_SScrewComp, sizeof(SCompArray));
    getRealTimePoint(SActual.dArray, AXIS_NUM_MAX, pView->pChannel->pMapAxisArray, NULL);
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        double dPulse;
        if ((getbit(pView->ucValid, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        dPulse = (pPoint->dArray[ucAxis] - SActual.dArray[ucAxis]) / pView->pPulseArray[ucAxis];
        if (!isfinite(dPulse) || fabs(dPulse) > 32766.0)
        {
            return -1;
        }
    }
#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
    nResult = outPutAbsPointWithScrewComp(pRun, pPoint->dArray, AXIS_NUM_MAX, pView->pChannel->pMapAxisArray,
                                          pView->pPulseArray, &pView->ucValid);
#else
    nResult = outPutAbsPoint(pRun, pPoint->dArray, AXIS_NUM_MAX, pView->pChannel->pMapAxisArray, pView->pPulseArray,
                             &pView->ucValid);
#endif
    if (nResult == TRUE && !isErrorHasBeenWritten())
    {
        return 1;
    }
    /* [HWSIM] 任一输出失败就还原整帧的软件副作用，调用方随后清候选脉冲并报警。
     * 只有返回成功，executeHandwheelPathStep() 才能扣除本拍距离和提交末速度；
     * 这不是已经发送硬件脉冲后的物理回滚。
     */
    memcpy(g_SScrewComp, SCompArray, sizeof(SCompArray));
    pRun->SLastAbsCoord = SLast;
    pRun->SAbsCoord = SAbs;
    updateRealTimePoint(SActual.dArray, AXIS_NUM_MAX, pView->pChannel->pMapAxisArray, &pView->ucValid);
    return -1;
}

/*************************************************
  函数名称：saveHandwheelPathPauseState
  函数功能：停稳后按原几何参数单位回写当前段
  输入参数：pView - 当前插补对象
  输出参数：pView - 原历史游标、暂停状态及已走距离
  返 回 值：1 - 成功；-1 - 历史不可用
  备    注：不调用旧时间减速规划，不提前回写预测停车点
****************************************/
static int saveHandwheelPathPauseState(struct SHandwheelPathView* pView)
{
    struct SChannel* pChannel = pView->pChannel;
    struct SHandwheelPath* pPath = &pChannel->SHandwheelSim.SPath;
    struct SToolCompensation* pBlock;
    INDEX_DATA_TYPE nIndex = g_SRingBuff[pChannel->ucChannelNum].nInterpIndex - 1;
    /* [HWSIM] 只有实际规划停到零速后才来这里。读游标通常已指向下一记录，
     * 因此 nInterpIndex-1 是本段；先验证记录仍在保留窗口内，再退回读取位置。
     */
    if (nIndex < 0 || nIndex < g_SRingBuff[pChannel->ucChannelNum].nBackIndex ||
        nIndex >= g_SRingBuff[pChannel->ucChannelNum].nLookAheadIndex)
    {
        return -1;
    }
    pBlock = getSToolCompPointerByIndex(nIndex, pChannel->ucChannelNum);
    if (pBlock == NULL || !moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pChannel->ucChannelNum))
    {
        return -1;
    }
    pBlock->dPreDist = pPath->SState.dSegmentPosition;
    /* [HWSIM] 回写已成功提交的进度，不写“预测将来停车点”。恢复原 F 并使前瞻
     * 重新计算；清除本段已处理 IO，避免暂停恢复时重复执行相同外部动作。
     */
    pBlock->dFeedrate = pBlock->dUncorrectedFeedrate;
    pBlock->bHasVeerFlag = FALSE;
    pBlock->bLookAheadVParaCalcOverFlag = FALSE;
    pChannel->bLookAheadNewBeginFlag = TRUE;
    initLookAheadDealed(pChannel->ucChannelNum);
    clearIOCommand(pBlock->ucIOCommandArray, IO_ALL_ACTION, IO_MAXGROUP_IN_SINGLE_LINE);
    pBlock->sChannelCommand = pChannel->SPreProcess.sChannelCommand;
    pChannel->SPreProcess.sChannelCommand.ucType = UNASSIGNED_VALUE;
    setBits(pChannel->SPreProcess.unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
    pChannel->SPreProcess.ucInpStatus = INP_PAUSING;
    return 1;
}

/*************************************************
  函数名称：updateHandwheelG99SpeedLimit
  函数功能：按实测主轴转速更新 G99 上限并锁存停转制动
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 路径实时限速与停转状态
  返 回 值：1 - 可采样；0 - 停转制动或等待；-1 - 非有限速度
  备    注：单位 mm/r 乘 r/ms；降低上限仍由逐拍减速兑现，不强行裁剪速度
****************************************/
static int updateHandwheelG99SpeedLimit(struct SChannel* pChannel)
{
    struct SHandwheelPath* pPath = &pChannel->SHandwheelSim.SPath; /**< 当前绑定路径。 */
    double dSpeed;                                                 /**< F 与实测转速换算的 mm/ms 上限。 */
    if (!pPath->bBound || !pPath->bPerRevolution)
    {
        return 1;
    }
    /* [HWSIM] G99 的速度上限 = |主轴 r/ms| * 程序 mm/r * 进给倍率，
     * 再与固定机床/几何上限取小值。上限突降只改变约束，不直接截断当前速度。
     */
    dSpeed = fabs(pChannel->SSpindleTest.dSpindleAvgSpeed) * pPath->dProgramFeed * pPath->dFeedRatio;
    if (!isfinite(dSpeed))
    {
        return -1;
    }
    pPath->SState.dSpeedLimit = fmin(pPath->dFixedSpeedLimit, dSpeed);
    if (dSpeed == 0.0)
    {
        pPath->bSpindleStop = TRUE;
    }
    if (pPath->bSpindleStop)
    {
        /* [HWSIM] 停转一旦锁存，即使主轴下一拍恢复，也要等进给轴完成制动且
         * 旧授权已清空才重新放行输入，防止主轴短暂停转后执行积攒的距离。
         */
        if (dSpeed > 0.0 && pPath->SState.dBoundaryVelocity == 0.0 && pPath->SState.dPendingDistance == 0.0)
        {
            pPath->bSpindleStop = FALSE;
            pPath->SState.bInputStop = FALSE;
        }
        else
        {
            pPath->SState.bInputStop = TRUE;
            return 0;
        }
    }
    return 1;
}

/*************************************************
  函数名称：executeHandwheelPathStep
  函数功能：预览、输出和提交一个合成轴，统一处理端点和暂停
  输入参数：pChannel - 通道；bArc - 圆弧标志
  输出参数：pChannel - 所有参与轴坐标及合成账本
  返 回 值：1 - 执行或等待；0 - 完成；-1 - 故障
  备    注：段首受原历史屏障约束，段间零速交接且不传递剩余授权
****************************************/
int8 executeHandwheelPathStep(struct SChannel* pChannel, bool bArc)
{
    struct SHandwheelPathView SView;
    struct SHandwheelPath* pPath = &pChannel->SHandwheelSim.SPath;
    struct SHandwheelSimTrial STrial;
    SPointDouble SPoint;
    bool bStop = pChannel->ucChannelStatus == SYS_RUN_TO_PAUSE;
    bool bBack = FALSE;
    /* [HWSIM] 每拍步骤 1：选择 G1/G2/G3 几何视图；只有首次进入
     * INP_ASSIGNMENT_START 且尚未绑定时才做完整绑定，正常周期不重建几何。
     */
    selectHandwheelPathData(pChannel, bArc, &SView);
    if (!isHandwheelDistanceControlActive(pChannel))
    {
        return reportHandwheelPathError(pChannel);
    }
    if (!pPath->bBound && !pPath->bPreparing && *SView.pStatus == INP_ASSIGNMENT_START)
    {
        pPath->cBindResult = (int8)prepareHandwheelPath(&SView);
    }
    if (pPath->bPreparing)
    {
        int nPrepared; /**< 每拍有限工作量；暂停不等待建表完成。 */
        if (pChannel->SHandwheelSim.bFault || isErrorHasBeenWritten())
        {
            return reportHandwheelPathError(pChannel);
        }
        nPrepared = bStop ? 1 : buildHandwheelInputMapStep(&SView);
        if (nPrepared < 0)
        {
            return reportHandwheelPathError(pChannel);
        }
        if (nPrepared == 0)
        {
            memset(pChannel->SRun.sAxisDeltaPulse, 0, sizeof(pChannel->SRun.sAxisDeltaPulse));
            return 1;
        }
        pPath->bPreparing = FALSE;
        pPath->bBound = TRUE;
        *SView.pStatus = INP_RUNNING;
    }
    if (pChannel->SHandwheelSim.bFault || isErrorHasBeenWritten() || !pPath->bBound || pPath->bArc != bArc)
    {
        return reportHandwheelPathError(pChannel);
    }
    if (pChannel->SRun.dCommonFeedRatio != pPath->dFeedRatio)
    {
        /* [HWSIM] 绑定约束基于当时倍率。倍率变化先沿当前轨迹减速、回写暂停，
         * 不在带速时重新绑定或把原速度按新倍率瞬间缩放。
         */
        pauseButtonAPI(pChannel->ucChannelNum);
        bStop = TRUE;
    }
    /* [HWSIM] 每拍步骤 2~5：更新 G99 限速 -> 预览标量运动 -> 几何求点 ->
     * 全轴输出。逻辑或 || 从左到右短路，前一步失败就不执行后续步骤。
     * preview 生成的 STrial 只是候选，输出失败时绝不能提交它。
     */
    if (updateHandwheelG99SpeedLimit(pChannel) < 0 ||
        calculateHandwheelMotionStep(&pPath->SState, bStop, &STrial) < 0 ||
        calculateHandwheelPathPoint(&SView, STrial.dPosition, &SPoint) < 0 ||
        generateHandwheelPathPulses(&SView, &SPoint) < 0)
    {
        return reportHandwheelPathError(pChannel);
    }
    /* [HWSIM] 每拍步骤 6：先用新旧位置差求本拍平均速度，再提交理论进度和
     * 周期末速度。若先 commit 再做差，显示速度会被错误算成零。
     */
    pChannel->SRun.dCurrentSpeed = *SView.pSpeed =
        fabs(STrial.dPosition - pPath->SState.dSegmentPosition) / pPath->SState.dPeriod;
    updateHandwheelMotionState(&pPath->SState, &STrial);
    /* [HWSIM] 保存成功脉冲帧供报警停车交接；同时回写同单位的公共剩余进度。
     * 原 SPlan 不负责本拍距离规划，但暂停、历史读段等旧流程仍要读取这些字段。
     */
    memcpy(pPath->sLastPulseArray, pChannel->SRun.sAxisDeltaPulse, sizeof(pPath->sLastPulseArray));
    SView.pPlan->dCurLeftDist = pPath->SState.dSegmentLength - STrial.dPosition;
    SView.pPlan->cInterpDir = STrial.dVelocity < 0.0 ? INP_GO_BACK : INP_GO_AHEAD;
    /* [HWSIM] 每拍步骤 7：端点事件不等于无条件换段。退到段首先检查历史，
     * 无合法历史就取消向外授权并等待；普通 WAIT/MOVE/INPUT_IDLE 都保留绑定。
     * INPUT_IDLE 只是停摇停稳，不能误当成程序暂停去回写缓冲区。
     */
    if (STrial.eStep == HWSIM_STEP_BACKWARD_END)
    {
        bBack = hasHandwheelReverseHistory(pChannel, FALSE);
        if (!bBack)
        {
            pPath->SState.dPendingDistance = 0.0;
        }
    }
    if (!bBack && STrial.eStep != HWSIM_STEP_FORWARD_END && STrial.eStep != HWSIM_STEP_STOPPED)
    {
        return 1;
    }
    if (STrial.eStep == HWSIM_STEP_STOPPED && STrial.dPosition < pPath->SState.dSegmentLength)
    {
        if (saveHandwheelPathPauseState(&SView) < 0)
        {
            return reportHandwheelPathError(pChannel);
        }
    }
    else
    {
        pChannel->SPreProcess.ucInpStatus = INP_FINISHED;
    }
    *SView.pStatus = INP_FINISHED;
    pChannel->SPreProcess.cInterpDir = bBack ? INP_GO_BACK : INP_GO_AHEAD;
    *SView.pSpeed = pChannel->SRun.dCurrentSpeed = 0.0;
    /* [HWSIM] 段尾完成、合法回退或暂停回写后才解除绑定。只留下换段方向，
     * 不把本段剩余授权带入下一段；下一段绑定成功后需要新的有效手轮输入。
     */
    clearHandwheelSimMotion(&pChannel->SHandwheelSim);
    if (!bStop)
    {
        pChannel->SHandwheelSim.cBoundaryDirection = pChannel->SPreProcess.cInterpDir;
    }
    return 0;
}
#if STM32_MODE
#include "limit_warn.h"
#endif

/*************************************************
  函数名称：isHandwheelDistanceControlRequested
  函数功能：判断用户是否请求距离型手轮，排除恢复加工
  输入参数：pChannel - 待检查通道，允许为空
  输出参数：无
  返 回 值：TRUE - 请求开启；FALSE - 未开启或正在恢复加工
  备    注：普通手轮移动后的返回暂停点过程不属于距离型 G0
****************************************/
static bool isHandwheelDistanceControlRequested(const struct SChannel* pChannel)
{
    /* [HWSIM] 面板开关只是“请求”，不是当前段一定被接管的结论。
     * 普通选轴手轮模式不走本模块；暂停后返回加工点的用户数据运动也先走原流程，
     * 回到加工点后才能重新绑定程序轨迹。
     */
    return pChannel != NULL && pChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_ON &&
           (g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE) &&
           !(pChannel->ucChannelStatus == SYS_PAUSE_TO_RUN && pChannel->SG00.bIsUserData);
}

/*************************************************
  函数名称：isHandwheelBlockAutomaticOnly
  函数功能：按段类型、轴单位和插补模式识别尚未接入的运动
  输入参数：pChannel - 轴映射；unCommand - 段指令；ucValid - 参与轴；bAutomatic - 显式自动标记
  输出参数：无
  返 回 值：TRUE - 自动执行；FALSE - 可模拟或需进一步报错校验
  备    注：未知轴型及非法映射不是能力缺口，仍交给参数校验拒绝
****************************************/
static bool isHandwheelBlockAutomaticOnly(const struct SChannel* pChannel, uint32 unCommand, VAL_STRUCT_TYPE ucValid,
                                          bool bAutomatic)
{
    uint32 unMotion = getBits(unCommand, CMD_MOTION_ADDR); /**< 当前运动类型。 */
    uint8 ucAxis;                                          /**< 参与运动的逻辑轴。 */
    uint8 ucPlaneArray[2];                                 /**< 原自动插补选定平面。 */
    /* [HWSIM] 显式标记和已知未接入的运动走自动旁路，包括特殊插补模式。
     * 但非法轴号/未知轴型不能被解释为“自动执行即可”，仍需由后续校验报错。
     * 普通 G1 和圆弧平面外旋转轴由公共进度接管；旋转圆弧平面仍走原流程。
     */
    if (bAutomatic)
    {
        return TRUE;
    }
    if (unMotion != CMD_MOTION_QUICKMOVE && unMotion != CMD_MOTION_LINE && unMotion != CMD_MOTION_ARC &&
        unMotion != CMD_MOTION_ANTI_ARC)
    {
        return TRUE;
    }
    if (getBits(unCommand, CMD_INTERPOLATION_MODE_ADDR) != CMD_INTERPOLATION_COMMON)
    {
        return TRUE;
    }
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE
    if (pChannel->bUseVirtualY)
    {
        return TRUE;
    }
#endif
#if ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
    return TRUE;
#endif
    if (unMotion == CMD_MOTION_QUICKMOVE || unMotion == CMD_MOTION_LINE || pChannel->pMapAxisArray == NULL)
    {
        return FALSE;
    }
    chooseInterpPlane(ucPlaneArray, unCommand);
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        uint8 ucPhysical = pChannel->pMapAxisArray[ucAxis]; /**< 已映射的物理轴。 */
        if ((getbit(ucValid, ucAxis)) == VAL_VALID && ucPhysical < AXIS_PHYSICS_NUM_MAX &&
            (ucAxis == ucPlaneArray[0] || ucAxis == ucPlaneArray[1]) &&
            g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == ROTATION_AXIS)
        {
            return TRUE;
        }
    }
    return FALSE;
}

/*************************************************
  函数名称：isHandwheelAutomaticMoveRequired
  函数功能：判断当前程序段是否绕过手轮距离控制
  输入参数：pChannel - 待检查通道
  输出参数：无
  返 回 值：TRUE - 原自动路径；FALSE - 未请求或可接管
  备    注：G84 非有限倍率仍属于非法参数，不用自动回退掩盖
****************************************/
bool isHandwheelAutomaticMoveRequired(const struct SChannel* pChannel)
{
    const struct SInput* pInput; /**< 当前段输入，不能使用前段残留的插补对象。 */
    if (!isHandwheelDistanceControlRequested(pChannel))
    {
        return FALSE;
    }
    pInput = &pChannel->SPreProcess.SUserData;
    /* [HWSIM] G84 展开的快速定位段不一定有攻丝倍率，因此需要 bHandwheelAuto
     * 来源标记，不能只看当前 G0/G1 或 dG84Ratio。使用当前输入记录，也避免
     * 上一段 SG01/SG02 残留的倍率影响新段。NaN/Inf 不作为正常旁路的依据。
     */
    return isHandwheelBlockAutomaticOnly(pChannel, pChannel->SRun.unCommandType, pInput->ucValStatus,
                                         pInput->bHandwheelAuto ||
                                             (isfinite(pInput->dG84Ratio) && pInput->dG84Ratio != 0.0));
}

/*************************************************
  函数名称：isHandwheelDistanceControlActive
  函数功能：判断当前段是否由距离手轮接管
  输入参数：pChannel - 待检查通道
  输出参数：无
  返 回 值：TRUE - 接管；FALSE - 原自动或恢复加工路径
  备    注：不修改用户选择的手轮模式
****************************************/
bool isHandwheelDistanceControlActive(const struct SChannel* pChannel)
{
    /* [HWSIM] 外层采样和 G0/G1/G2/G3 接管位置都用同一判断，防止采样层接收
     * 授权而执行层却走自动算法。此函数只判断，不关闭用户选择的模拟模式。
     */
    return isHandwheelDistanceControlRequested(pChannel) && !isHandwheelAutomaticMoveRequired(pChannel);
}

/*************************************************
  函数名称：prepareHandwheelAutomaticMove
  函数功能：为自动回退段隔离旧授权并恢复正常运动倍率
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 清理后的输入和正向倍率
  返 回 值：1 - 可执行；-1 - 已有故障或绑定未完成
  备    注：不得把仍在运动的距离段切换为自动规划
****************************************/
int prepareHandwheelAutomaticMove(struct SChannel* pChannel)
{
    uint8 ucAxis, ucPrevious;    /**< 有效轴及重复映射检查索引。 */
    const struct SInput* pInput; /**< 原自动规划输入。 */
    if (!isHandwheelAutomaticMoveRequired(pChannel))
    {
        return 1;
    }
    pInput = &pChannel->SPreProcess.SUserData;
    /* [HWSIM] 自动旁路只能在旧距离段已解除绑定、软件零速且无故障时接管。
     * 不能为了让 G84 开始就直接清掉仍在运动的距离状态；非法几何同样必须拒绝。
     */
    if (pChannel->SHandwheelSim.bFault || isErrorHasBeenWritten() || pChannel->SHandwheelSim.bInstructionBound ||
        pChannel->SHandwheelSim.SPath.bBound || pChannel->SHandwheelSim.SPath.bPreparing ||
        pChannel->pMapAxisArray == NULL || !isfinite(pInput->dG84Ratio) ||
        !isHandwheelSimGroupStopped(&pChannel->SHandwheelSim))
    {
        return reportHandwheelPathError(pChannel);
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        if ((getbit(pInput->ucValStatus, ucAxis)) != VAL_VALID)
        {
            continue;
        }
        if (pChannel->pMapAxisArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX || !isfinite(pInput->SStartPoint.dArray[ucAxis]) ||
            !isfinite(pInput->SEndPoint.dArray[ucAxis]))
        {
            return reportHandwheelPathError(pChannel);
        }
        for (ucPrevious = 0; ucPrevious < ucAxis; ++ucPrevious)
        {
            if ((getbit(pInput->ucValStatus, ucPrevious)) == VAL_VALID &&
                pChannel->pMapAxisArray[ucPrevious] == pChannel->pMapAxisArray[ucAxis])
            {
                return reportHandwheelPathError(pChannel);
            }
        }
    }
    /* [HWSIM] 校验通过后隔离旧输入，恢复正向 1.0 手轮速度倍率，使旁路段不受
     * 手轮零输入或反向输入控制。这里只恢复手轮倍率，不覆盖普通进给倍率。
     */
    resetHandwheelSimGroup(&pChannel->SHandwheelSim);
    pChannel->SHandwheelSim.bAwaitingFreshInput = TRUE;
    pChannel->SRun.dHandWheelSpeedRatio = 1.0;
    return 1;
}

/*************************************************
  函数名称：resetHandwheelSim
  函数功能：清空单轴距离状态并将绑定轴置为 AXIS_NULL
  输入参数：pState - 待清理状态，允许为空
  输出参数：pState - 清零后的单轴状态
  返 回 值：无
  备    注：不是执行机构停止命令；仅初始化、确认零速或原停止路径接管后调用
****************************************/
void resetHandwheelSim(struct SHandwheelSim* pState)
{
    /* [HWSIM] 这是内存初始化，不会产生减速脉冲。实际带速运动必须先完成规划
     * 停车或交接给原停止链，不能通过 memset 让机床“瞬间停住”。
     */
    if (pState != NULL)
    {
        memset(pState, 0, sizeof(*pState));
        pState->ucAxis = AXIS_NULL;
    }
}

/*************************************************
  函数名称：clearHandwheelSimMotion
  函数功能：全轴零速时清除已结束运动的授权和几何绑定
  输入参数：pGroup - 已初始化的通道状态
  输出参数：pGroup - 全轴零速才清理，带速或空指针时不修改
  返 回 值：无
  备    注：保留系统参数中的共享手轮档位，不用本函数代替减速停车
****************************************/
void clearHandwheelSimMotion(struct SHandwheelSimGroup* pGroup)
{
    /* [HWSIM] 正常段结束使用这个带零速门槛的清理入口；带速时直接返回，
     * 防止清掉仍在执行的授权、几何和速度。故障交接另走 transferHandwheelStateToStopControl。
     */
    if (!isHandwheelSimGroupStopped(pGroup))
        return;
    resetHandwheelSimGroup(pGroup);
}

/*************************************************
  函数名称：resetHandwheelSimGroup
  函数功能：初始化通道及全部独立轴、合成轴的距离状态
  输入参数：pGroup - 通道状态，允许为空
  输出参数：pGroup - 清零后的通道及轴状态
  返 回 值：无
  备    注：仅初始化、确认零速或原停止路径接管后调用，不修改共享手轮档位
****************************************/
void resetHandwheelSimGroup(struct SHandwheelSimGroup* pGroup)
{
    uint8 ucAxis; /**< 逻辑轴索引。 */
    if (pGroup == NULL)
    {
        return;
    }
    /* [HWSIM] 清除表前的轴/路径控制状态及表后的通道输入状态，保留两张表的
     * 存储内容。bBound、bPreparing 和建表索引均在清零范围内；
     * 重新绑定从表项 0 开始逐拍覆盖，完整建表后才允许输入查表，旧内容不会复用。
     * 自动旁路每拍可能多次调用本函数，不能反复清空 4112 B 的无效表。
     */
    memset(pGroup, 0, offsetof(struct SHandwheelSimGroup, SPath) + offsetof(struct SHandwheelPath, dMetricArray));
    memset((unsigned char*)pGroup + offsetof(struct SHandwheelSimGroup, bInstructionBound), 0,
           sizeof(*pGroup) - offsetof(struct SHandwheelSimGroup, bInstructionBound));
    /* 标量状态已随前缀清零，仅补回未绑定轴哨兵，避免重复清理。 */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        pGroup->SAxisStateArray[ucAxis].ucAxis = AXIS_NULL;
    }
    pGroup->SPath.SState.ucAxis = AXIS_NULL;
    /* 空闲清理和段间解绑后也等待首个有效方向，不能将过渡样本误判为运行故障。 */
    pGroup->bAwaitingFreshInput = TRUE;
}

/*************************************************
  函数名称：isHandwheelSimGroupStopped
  函数功能：检查独立轴及合成轴的已提交周期末速度是否全部为零
  输入参数：pGroup - 通道距离状态
  输出参数：无
  返 回 值：TRUE - 全部为零；FALSE - 空指针或任一轴带速
  备    注：不使用平均速度判断；软件零速不代表伺服反馈机械停稳
****************************************/
bool isHandwheelSimGroupStopped(const struct SHandwheelSimGroup* pGroup)
{
    uint8 ucAxis; /**< 逻辑轴索引。 */
    if (pGroup == NULL)
        return FALSE;
    /* [HWSIM] 只用“已提交周期末速度”判零。最后一拍仍可能有位移，所以
     * 本拍平均速度非零并不妨碍周期末已经停稳；反之也不能用平均速度代替末速。
     */
    if (pGroup->SPath.SState.dBoundaryVelocity != 0.0)
        return FALSE;
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
        if (pGroup->SAxisStateArray[ucAxis].dBoundaryVelocity != 0.0)
            return FALSE;
    return TRUE;
}

/*************************************************
  函数名称：isHandwheelHistoryBlockReversible
  函数功能：检查历史段是否属于已编译开启且允许回退的普通运动
  输入参数：pChannel - 当前轴映射；pBlock - 原刀补历史记录
  输出参数：无
  返 回 值：TRUE - 可回退；FALSE - 未开启、特殊运动或指令屏障
  备    注：空段也必须先验证 IO、程序结束及特殊指令限制
****************************************/
static bool isHandwheelHistoryBlockReversible(const struct SChannel* pChannel, struct SToolCompensation* pBlock)
{
    bool bMotion;
    if (pBlock == NULL)
    {
        return FALSE;
    }
    /* [HWSIM] 回退不能越过自动旁路段或有副作用的指令屏障。即使该记录没有
     * 位移，也必须先检查 IO、程序结束、坐标设置等，不能把它当空白直接跳过。
     * 这也阻止反摇重新进入 G84 攻丝循环。
     */
    if (isHandwheelBlockAutomaticOnly(pChannel, pBlock->unCommandType, pBlock->ucValStatus,
                                      pBlock->bHandwheelAuto || pBlock->dG84Ratio != 0.0))
    {
        return FALSE;
    }
    bMotion = getBits(pBlock->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE;
    if ((getBits(pBlock->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN ||
         getBits(pBlock->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_R) &&
        pBlock->dG84Ratio == 0.0 && isfinite(pBlock->dFeedrate) && pBlock->dFeedrate > 0.0)
    {
        bMotion = bMotion || getBits(pBlock->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE;
        bMotion = bMotion || ((getBits(pBlock->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
                               getBits(pBlock->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC) &&
                              isfinite(pBlock->dRadius) && fabs(pBlock->dRadius) > HWSIM_DISTANCE_EPS);
    }
    return pBlock != NULL && judgeCanMoveBack(pBlock) == 1 && pBlock->ucG28ZeroingValAxis == 0 &&
           pBlock->sChannelCommand.ucType == UNASSIGNED_VALUE && bMotion &&
           getBits(pBlock->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_CONTINUE &&
           getBits(pBlock->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_COMMON &&
           !pBlock->bG50Appear && !pBlock->bG65SystemMacroWait && pBlock->cPointType != G50 &&
           pBlock->cPointType != G10 && pBlock->cToolAxisMod == 0 && pBlock->cTwistCommand == 0;
}

/*************************************************
  函数名称：hasHandwheelReverseHistory
  函数功能：确认保留窗口内存在可回退运动，避免退入程序头空段
  输入参数：pChannel - 通道；bCompleted - TRUE 从刚完成段查起，FALSE 从上一段查起
  输出参数：无
  返 回 值：TRUE - 存在有效运动历史；FALSE - 已到回退边界
  备    注：只读扫描，不改变游标；空段前存在屏障时不得继续查找
****************************************/
bool hasHandwheelReverseHistory(const struct SChannel* pChannel, bool bCompleted)
{
    INDEX_DATA_TYPE nIndex;          /**< 未取模索引，必须在原历史窗口内。 */
    const struct SRingBuff* pBuffer; /**< 当前通道的原缓冲区。 */
    if (pChannel == NULL || pChannel->ucChannelNum >= CHANNEL_NUM_MAX)
        return FALSE;
    pBuffer = &g_SRingBuff[pChannel->ucChannelNum];
    /* [HWSIM] 插补读游标通常在当前记录之后：
     * bCompleted=TRUE 从刚完成记录开始检查；FALSE 从当前段的上一记录开始。
     * 使用未取模逻辑索引与保留窗口比较，不能读已经被环形缓冲覆盖的历史。
     */
    nIndex = pBuffer->nInterpIndex - (bCompleted ? 1 : 2);
    for (; nIndex >= 0 && nIndex >= pBuffer->nBackIndex && nIndex < pBuffer->nLookAheadIndex; --nIndex)
    {
        struct SToolCompensation* pBlock = getSToolCompPointerByIndex(nIndex, pChannel->ucChannelNum); /**< 历史段。 */
        bool bHasMotion = FALSE; /**< 至少一个有效轴存在实际行程。 */
        uint8 ucAxis;            /**< 逻辑轴索引。 */
        if (!isHandwheelHistoryBlockReversible(pChannel, pBlock))
            return FALSE;
        /* [HWSIM] 只跳过已经确认无屏障的零位移普通段，找到真实可回退运动才返回。
         * 此处仅探查，不移动读游标；真正倒读由外层读段流程完成。
         */
        /* 整圆起终点相同，但弧长不为零；几何细项由重新加载时统一校验。 */
        if (getBits(pBlock->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
            getBits(pBlock->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
        {
            return isfinite(pBlock->dLength) && pBlock->dLength > HWSIM_DISTANCE_EPS;
        }
        for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
        {
            double dDistance; /**< 本轴原几何位移，不使用合成段长。 */
            if (VAL_VALID != getbit(pBlock->ucValStatus, ucAxis))
                continue;
            dDistance = pBlock->SEndPoint.dArray[ucAxis] - pBlock->SStartPoint.dArray[ucAxis];
            if (!isfinite(dDistance))
                return FALSE;
            if (sgn(dDistance) != 0)
                bHasMotion = TRUE;
        }
        if (bHasMotion)
            return TRUE;
    }
    return FALSE;
}

/*************************************************
  函数名称：addScaledHandwheelGroupAuthorizedDistance
  函数功能：校验同一样本后按轴换算新增授权，首端继续反摇静默丢弃
  输入参数：pGroup - 通道状态；nSignedRaw - 有符号计数；ucMultiple - 倍率；
            pScaleArray - 各逻辑轴换算系数，空指针表示不换算
  输出参数：pGroup - 更新后的账本及输入方向
  返 回 值：1 - 接收；0 - 丢弃距离；-1 - 校验失败
  备    注：未绑定时仅为刚完成段记录换向请求，不积攒运动距离
****************************************/
static int addScaledHandwheelGroupAuthorizedDistance(struct SHandwheelSimGroup* pGroup, int32 nSignedRaw,
                                                     uint8 ucMultiple, const double* pScaleArray)
{
    struct SHandwheelSim SInput;      /**< 复用标量输入校验，只计算本次距离，不存历史。 */
    double dInputArray[AXIS_NUM_MAX]; /**< 本拍各轴的实际新增授权，不重新缩放历史余量。 */
    uint8 ucAxis;                     /**< 当前逻辑轴索引。 */
    if (pGroup == NULL || pGroup->bFault)
        return -1;
    /* [HWSIM] SInput 是本次调用的局部换算器，清它不会清实际运动状态。
     * 先得到一份原始距离，再分发给 G0 各轴或合成路径，避免重复读取编码器。
     */
    resetHandwheelSim(&SInput);
    if (addHandwheelAuthorizedDistance(&SInput, nSignedRaw, ucMultiple) < 0)
    {
        pGroup->bFault = TRUE;
        return -1;
    }
    if (pGroup->SPath.bBound)
    {
        /* [HWSIM] 合成路径只接收一份授权；pScaleArray[0] 在这里传递输入增益，
         * 直径显示的非线性换算由路径映射表处理，而不是按物理轴逐个累加。
         */
        if (pGroup->SPath.bSpindleStop)
        {
            return 0;
        }
        int nResult = addHandwheelPathAuthorizedDistance(
            pGroup, SInput.dPendingDistance * (pScaleArray != NULL ? pScaleArray[0] : 1.0), pScaleArray != NULL);
        if (nResult < 0)
        {
            pGroup->bFault = TRUE;
        }
        return nResult;
    }
    if (!pGroup->bInstructionBound)
    {
        /* [HWSIM] 未绑定阶段不积攒运动距离。刚完成段后的新方向只用于协调
         * 下一次历史读段，不能让段间摇动变成新段一绑定就自动执行的余量。
         */
        if (pGroup->cBoundaryDirection != 0 && nSignedRaw != 0)
            pGroup->cInputDirection = nSignedRaw > 0 ? INP_GO_AHEAD : INP_GO_BACK;
        return 0;
    }
    if (pGroup->bReverseBlocked && nSignedRaw < 0)
        return 0;
    /* [HWSIM] G0 分两轮：先算每轴新增量并验证所有候选，全部成功后才统一累加。
     * 否则可能 X 已收到新授权，Y 却因溢出失败，造成同一份输入只分发给部分轴。
     * 换向替换旧请求时通过减去旧余量实现，但不修改已提交位置和速度。
     */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        const struct SHandwheelSim* pState = &pGroup->SAxisStateArray[ucAxis]; /**< 本轴账本。 */
        dInputArray[ucAxis] = SInput.dPendingDistance * (pScaleArray != NULL ? pScaleArray[ucAxis] : 1.0);
        if (pScaleArray != NULL && isHandwheelDistanceReplacementNeeded(pGroup, pState, dInputArray[ucAxis]))
        {
            dInputArray[ucAxis] -= pState->dPendingDistance;
        }
        if (pState->bSegmentBound && (pState->bFault || !isfinite(pState->dPendingDistance + dInputArray[ucAxis])))
        {
            pGroup->bFault = TRUE;
            return -1;
        }
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        if (pGroup->SAxisStateArray[ucAxis].bSegmentBound)
        {
            pGroup->SAxisStateArray[ucAxis].dPendingDistance += dInputArray[ucAxis];
        }
    }
    if (nSignedRaw != 0)
        pGroup->cInputDirection = nSignedRaw > 0 ? INP_GO_AHEAD : INP_GO_BACK;
    if (nSignedRaw > 0)
        pGroup->bReverseBlocked = FALSE;
    return 1;
}

/*************************************************
  函数名称：addHandwheelGroupAuthorizedDistance
  函数功能：提供不依赖机床轴配置的基础距离授权接口
  输入参数：pGroup - 通道状态；nSignedRaw - 有符号计数；ucMultiple - 倍率
  输出参数：pGroup - 全部校验通过后更新各轴授权
  返 回 值：1 - 接收；0 - 丢弃；-1 - 校验失败
  备    注：生产编码器采样通过 addHandwheelChannelAuthorizedDistance 按物理轴换算
****************************************/
int addHandwheelGroupAuthorizedDistance(struct SHandwheelSimGroup* pGroup, int32 nSignedRaw, uint8 ucMultiple)
{
    /* [HWSIM] 基础接口不传缩放表，保留精确正负抵消语义；生产采样使用
     * addHandwheelChannelAuthorizedDistance()，额外应用输入增益、轴显示单位和响应替换规则。
     */
    return addScaledHandwheelGroupAuthorizedDistance(pGroup, nSignedRaw, ucMultiple, NULL);
}

/*************************************************
  函数名称：resetHandwheelSpeedMeasurement
  函数功能：仅清转速增益测速状态，不修改已授权距离和运动状态
  输入参数：pGroup - 有效通道状态
  输出参数：pGroup - 清零的输入测速记录
  返 回 值：无
  备    注：暂停、停转、换向和非绑定阶段不继承高速状态
****************************************/
static void resetHandwheelSpeedMeasurement(struct SHandwheelSimGroup* pGroup)
{
    /* [HWSIM] 仅重置“输入转得多快”的估计，不取消已授权距离、不改运动速度，
     * 也不重置独立的停摇计时。它与 resetHandwheelSimGroup() 不是同一类清理。
     */
    pGroup->cInputRateDirection = 0;
    pGroup->bFastInput = FALSE;
    pGroup->dFilteredInputRate = 0.0;
    pGroup->dInputGain = 1.0;
}

/*************************************************
  函数名称：calculateHandwheelDistanceMultiplier
  函数功能：按平均格速换算转速比例，乘面板倍率并限制在一至面板上限
  输入参数：pGroup - 通道；nSignedRaw - 原始计数；dPeriod - 周期 ms
  输出参数：pGroup - 格速和倍率状态
  返 回 值：不小于一的倍率；-1 - 参数无效
  备    注：零输入只衰减估计，不制造距离；换向不继承旧方向增益
****************************************/
static double calculateHandwheelDistanceMultiplier(struct SHandwheelSimGroup* pGroup, int32 nSignedRaw, double dPeriod)
{
    double dMax = g_SSystemPara.fHandleWheelSpeedMax;       /**< 面板最大倍率。 */
    double dAlpha, dFraction, dTarget;                      /**< 测速权重、线性比例、目标倍率。 */
    uint16 usPeriod = g_SSystemPara.usHandleWheelAvgPeriod; /**< 测速时间常数的周期数。 */
    int8 cDirection = nSignedRaw > 0 ? INP_GO_AHEAD : (nSignedRaw < 0 ? INP_GO_BACK : 0); /**< 输入方向。 */
    if (!isfinite(dPeriod) || dPeriod <= 0.0 || usPeriod < 1 || usPeriod > 1000 || (!isfinite(dMax) || dMax < 0.0))
    {
        return -1.0;
    }
    if ((!pGroup->bInstructionBound && !pGroup->SPath.bBound) || (pGroup->bReverseBlocked && nSignedRaw < 0))
    {
        resetHandwheelSpeedMeasurement(pGroup);
        return 1.0;
    }
    if (cDirection != 0 && cDirection != pGroup->cInputRateDirection)
    {
        resetHandwheelSpeedMeasurement(pGroup);
        pGroup->cInputRateDirection = cDirection;
        /* 首个样本的计数间隔未知；只授权基础距离，不猜测高速。 */
        return 1.0;
    }
    /* [HWSIM] 原编码器 4 个计数对应 1 格，周期单位是 ms，
     * 所以 abs(raw)*250/T = abs(raw)/4*1000/T，结果为格/秒。
     * 先低通格速，再换算相对于最大 RPM 的 0..1 比例，抑制稀疏计数的跳变。
     */
    dAlpha = 1.0 / (1.0 + usPeriod);
    pGroup->dFilteredInputRate += dAlpha * (fabs((double)nSignedRaw) * 250.0 / dPeriod - pGroup->dFilteredInputRate);
    dFraction =
        fmin(1.0, fmax(0.0, pGroup->dFilteredInputRate * 60.0 / HWSIM_RESPONSE_TICKS_PER_REV / HWSIM_RESPONSE_MAX_RPM));
    dMax = fmax(1.0, dMax);
    dTarget = fmax(1.0, dMax * dFraction);
    /* [HWSIM] 对目标增距倍率再做时间平滑，倍率只影响“本次新增授权”。
     * 它不直接乘轴速度，也不重新放大已经积攒的距离；无输入仍可衰减估计，
     * 但没有原始计数就不会凭空制造运动距离。
     */
    pGroup->dInputGain = fmax(1.0, pGroup->dInputGain);
    pGroup->dInputGain += dPeriod / (HWSIM_RESPONSE_GAIN_TIME + dPeriod) * (dTarget - pGroup->dInputGain);
    pGroup->dInputGain = fmin(dMax, pGroup->dInputGain);
    pGroup->bFastInput = pGroup->dInputGain > 1.0 + HWSIM_DISTANCE_EPS;
    return pGroup->dInputGain;
}

/*************************************************
  函数名称：updateHandwheelDistanceLimitAndStop
  函数功能：按已提交进给速度启用授权限额，或请求停摇制动
  输入参数：pState - 标量账本；bInput - 有新输入或尚未达到停摇超时
  输出参数：pState - 受限授权及停摇标志
  返 回 值：无
  备    注：到速后锁存至零速；比较仅容忍相对机器精度误差，不使用通用 isZero
****************************************/
static void updateHandwheelDistanceLimitAndStop(struct SHandwheelSim* pState, bool bInput)
{
    double dSpeed, dBudget; /**< 前瞻速度和允许保留的距离。 */
    if (!pState->bSegmentBound)
    {
        return;
    }
    double dSpeedError = 32.0 * DBL_EPSILON * pState->dSpeedLimit; /**< 只覆盖到速计算舍入。 */
    /* [HWSIM] 用上一成功周期的末速判断是否到达限速，并锁存到下一次零速。
     * 低速精确走距阶段不削减授权；进入到速响应阶段后才限制缓冲并处理停摇。
     */
    if (fabs(pState->dBoundaryVelocity) > 0.0 && fabs(pState->dBoundaryVelocity) >= pState->dSpeedLimit - dSpeedError)
    {
        pState->bFeedLimitActive = TRUE;
    }
    if (!pState->bFeedLimitActive)
    {
        /* 未到速不削减授权，也不因无计数取消距离；新输入解除上次停摇标志。 */
        if (bInput)
        {
            pState->bInputStop = FALSE;
        }
        return;
    }
    pState->bInputStop = !bInput;
    if (bInput)
    {
        /* [HWSIM] 缓冲限额覆盖前瞻运动和从该速度制动所需距离，正负对称限幅。
         * 这里限的是剩余授权，实际速度仍由 preview 受加减速度约束地变化。
         */
        dSpeed = fmin(pState->dSpeedLimit,
                      fabs(pState->dBoundaryVelocity) + pState->dAcceleration * HWSIM_RESPONSE_LOOKAHEAD);
        /* 连续制动距离加一拍上界，覆盖离散末拍梯形积分。 */
        dBudget =
            dSpeed * dSpeed / (2.0 * pState->dDeceleration) + dSpeed * (pState->dPeriod + HWSIM_RESPONSE_LOOKAHEAD);
        pState->dPendingDistance = fmin(dBudget, fmax(-dBudget, pState->dPendingDistance));
    }
}

/*************************************************
  函数名称：updateHandwheelGroupDistanceLimitAndStop
  函数功能：独立于测速窗口检测停摇，并统一更新参与轴或合成轴
  输入参数：pGroup - 通道；nSignedRaw - 有效计数；dPeriod - 本拍 ms
  输出参数：pGroup - 停摇时钟与请求状态
  返 回 值：无
  备    注：只设置制动请求，实际停车和余量清除必须经过输出提交
****************************************/
static void updateHandwheelGroupDistanceLimitAndStop(struct SHandwheelSimGroup* pGroup, int32 nSignedRaw,
                                                     double dPeriod)
{
    double dTimeout;                  /**< 有界停摇检测时间，ms。 */
    bool bContinue = nSignedRaw != 0; /**< 允许正常执行，超时后请求制动。 */
    uint8 ucAxis;                     /**< 独立 G0 轴索引。 */
    if ((!pGroup->bInstructionBound && !pGroup->SPath.bBound) || (pGroup->bReverseBlocked && nSignedRaw < 0))
    {
        return;
    }
    if (nSignedRaw != 0)
    {
        /* [HWSIM] 每份有效输入更新输入间隔估计并重置空闲时钟。
         * 慢摇的计数可能隔好几拍才出现，不能把中间每个零计数拍都判为停摇。
         */
        pGroup->dInputGapEstimate = pGroup->bInputActive
                                        ? 0.75 * pGroup->dInputGapEstimate + 0.25 * (pGroup->dInputIdleTime + dPeriod)
                                        : dPeriod;
        pGroup->dInputIdleTime = 0.0;
        pGroup->bInputActive = TRUE;
    }
    else
    {
        if (!pGroup->bInputActive)
        {
            return;
        }
        /* [HWSIM] 超时取平均输入间隔的 2.5 倍，并限制在固定上下界内。
         * 超时仅重置输入增益并传递制动请求；是否取消余量要等真正停稳并提交。
         */
        pGroup->dInputIdleTime = fmin(HWSIM_RESPONSE_IDLE_MAX + dPeriod, pGroup->dInputIdleTime + dPeriod);
        dTimeout = fmax(HWSIM_RESPONSE_IDLE_MIN, fmin(HWSIM_RESPONSE_IDLE_MAX, 2.5 * pGroup->dInputGapEstimate));
        if (pGroup->dInputIdleTime >= dTimeout)
        {
            resetHandwheelSpeedMeasurement(pGroup);
        }
    }
    /* 到速限额逐拍检查，包括稀疏输入间的零计数拍；超时前仍允许正常执行。 */
    if (nSignedRaw == 0)
    {
        dTimeout = fmax(HWSIM_RESPONSE_IDLE_MIN, fmin(HWSIM_RESPONSE_IDLE_MAX, 2.5 * pGroup->dInputGapEstimate));
        bContinue = pGroup->dInputIdleTime < dTimeout;
    }
    if (pGroup->SPath.bBound)
    {
        updateHandwheelDistanceLimitAndStop(&pGroup->SPath.SState, bContinue);
        return;
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        updateHandwheelDistanceLimitAndStop(&pGroup->SAxisStateArray[ucAxis], bContinue);
    }
}

/*************************************************
  函数名称：addHandwheelChannelAuthorizedDistance
  函数功能：对新增输入应用转速比例增距倍率，再按物理轴直径参数换算
  输入参数：pChannel - 当前通道；nSignedRaw - 有符号计数；ucMultiple - 档位；
            dInputGain - 本拍高速增距倍率
  输出参数：pChannel - 各轴新增授权，映射异常时锁存故障
  返 回 值：1 - 接收；0 - 丢弃；-1 - 校验失败
  备    注：旋转轴及半径显示不缩放；不改程序几何、历史余量和速度
****************************************/
static int addHandwheelChannelAuthorizedDistance(struct SChannel* pChannel, int32 nSignedRaw, uint8 ucMultiple,
                                                 double dInputGain)
{
    double dScaleArray[AXIS_NUM_MAX]; /**< 本次采样各逻辑轴的换算系数。 */
    uint8 ucAxis;                     /**< 逻辑轴索引。 */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        uint8 ucPhysical; /**< 本逻辑轴映射的物理轴号。 */
        dScaleArray[ucAxis] = dInputGain;
        if (!pChannel->SHandwheelSim.bInstructionBound ||
            !pChannel->SHandwheelSim.SAxisStateArray[ucAxis].bSegmentBound)
        {
            continue;
        }
        if (pChannel->pMapAxisArray == NULL || pChannel->pMapAxisArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
        {
            pChannel->SHandwheelSim.bFault = TRUE;
            return -1;
        }
        ucPhysical = pChannel->pMapAxisArray[ucAxis];
        if (g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == LINEAR_AXIS && getShowByDiameter(ucPhysical) == 1)
        {
            /* [HWSIM] G0 直径显示轴：显示走 1 mm 对应实际半径位移 0.5 mm。
             * 只缩放新增授权，不缩放原段起终点，也不重复折半历史余量。
             */
            dScaleArray[ucAxis] *= 0.5;
        }
    }
    return addScaledHandwheelGroupAuthorizedDistance(&pChannel->SHandwheelSim, nSignedRaw, ucMultiple, dScaleArray);
}

/*************************************************
  函数名称：transferHandwheelStateToStopControl
  函数功能：报警或复位时向原停止路径交接各轴已提交速度
  输入参数：pChannel - 当前通道，调用者保证有效
  输出参数：pChannel - 原运动停车状态及清理后的模拟状态
  返 回 值：无
  备    注：合成路径使用上一成功脉冲帧交接，不把合成速度直接写给各物理轴
****************************************/
static void transferHandwheelStateToStopControl(struct SChannel* pChannel)
{
    uint8 ucAxis; /**< 需要交接的逻辑轴。 */
    /* [HWSIM] 报警/复位不再由正常距离流程消费输入，而交给原停止链。
     * 合成路径交接最后成功的各轴脉冲；G0 则交接各轴已提交末速及路径方向。
     * 必须先保存这些速度来源，再清距离状态，不能先清零导致停车链认为已停稳。
     */
    if (pChannel->SHandwheelSim.SPath.bBound)
    {
        /* 原停车由上一成功帧各轴脉冲反算速度，绝不能把合成速度塞给每根轴。 */
        memcpy(pChannel->SRun.sAxisDeltaPulse, pChannel->SHandwheelSim.SPath.sLastPulseArray,
               sizeof(pChannel->SRun.sAxisDeltaPulse));
    }
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        const struct SHandwheelSim* pState =
            &pChannel->SHandwheelSim.SAxisStateArray[ucAxis]; /**< 上一拍已提交状态。 */
        if (pState->bSegmentBound)
        {
            pChannel->SSingleAxisArray[ucAxis].dCurrentSpeed = fabs(pState->dBoundaryVelocity);
            if (pState->dBoundaryVelocity != 0.0)
                pChannel->SSingleAxisArray[ucAxis].SPlanData.cInterpDir =
                    pState->dBoundaryVelocity > 0.0 ? INP_GO_AHEAD : INP_GO_BACK;
        }
    }
    resetHandwheelSimGroup(&pChannel->SHandwheelSim);
    pChannel->SHandwheelSim.bAwaitingFreshInput = TRUE;
}

/*************************************************
  函数名称：leaveHandwheelSim
  函数功能：退出距离型手轮模拟并交接当前周期末速度
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 普通运动状态和距离模拟状态
  返 回 值：无
  备    注：幂等调用；重新接管前等待编码器给出有效方向
****************************************/
void leaveHandwheelSim(struct SChannel* pChannel)
{
    if (pChannel == NULL)
        return;
    if (isHandwheelAutomaticMoveRequired(pChannel))
    {
        /* [HWSIM] 自动旁路和报警退出分别处理：带着旧绑定进入报警/复位时先交接，
         * 正常旁路则必须通过 prepareHandwheelAutomaticMove() 的零速、无绑定检查。
         */
        if ((pChannel->SHandwheelSim.SPath.bBound || pChannel->SHandwheelSim.bInstructionBound) &&
            (isErrorHasBeenWritten() || pChannel->ucChannelStatus == SYS_RESET ||
             pChannel->ucChannelStatus == SYS_EMERGENCY_STOP))
        {
            transferHandwheelStateToStopControl(pChannel);
            return;
        }
        prepareHandwheelAutomaticMove(pChannel);
        return;
    }
    if (pChannel->SHandwheelSim.SPath.bBound && isErrorHasBeenWritten())
    {
        transferHandwheelStateToStopControl(pChannel);
        return;
    }
    if (pChannel->SHandwheelSim.SPath.bBound &&
        (pChannel->ucChannelStatus == SYS_RUN || pChannel->ucChannelStatus == SYS_RUN_TO_PAUSE))
    {
        /* 未经过暂停回写不能交给旧时间规划；保留末拍脉冲供原报警停车。 */
        pChannel->SHandwheelSim.bFault = TRUE;
        parseRunningError(pChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
        return;
    }
    /* [HWSIM] 其它退出场景：带速就先交接，已零速则直接清理。
     * 再进入模拟时等待有效编码器方向，不把模式切换中的不完整样本当正常输入。
     */
    if (!isHandwheelSimGroupStopped(&pChannel->SHandwheelSim))
        transferHandwheelStateToStopControl(pChannel);
    else
        resetHandwheelSimGroup(&pChannel->SHandwheelSim);
    pChannel->SHandwheelSim.bAwaitingFreshInput = TRUE;
}

/*************************************************
  函数名称：clearHandwheelAverageSpeedAfterRecovery
  函数功能：恢复加工完成后隔离末拍平均速度，供距离运动零速重绑
  输入参数：pChannel - 当前通道；bWasRecovering - 本拍调度前正在模拟恢复加工
  输出参数：pChannel - 仅更新已完成恢复的通道平均速度记录
  返 回 值：无
  备    注：在原 IRQ 调度后调用，不清本拍脉冲、坐标或活动轴速度
****************************************/
void clearHandwheelAverageSpeedAfterRecovery(struct SChannel* pChannel, bool bWasRecovering)
{
    /* [HWSIM] 在 IRQHandler 之后调用，只处理“本拍刚恢复完成”的边沿。
     * 恢复末拍的平均速度可以非零，但它不应成为下一距离段的起速；
     * 必须同时确认无活动绑定、无故障、标量末速为零，才清这一个显示字段。
     */
    if (!bWasRecovering || !isHandwheelDistanceControlActive(pChannel) || pChannel->ucChannelStatus != SYS_RUN ||
        pChannel->SG00.bIsUserData || pChannel->SG01.ucInpStatus != INP_FINISHED ||
        (getBits(pChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_QUICKMOVE &&
         getBits(pChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_LINE &&
         getBits(pChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_ARC &&
         getBits(pChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_ANTI_ARC) ||
        pChannel->SHandwheelSim.SPath.bBound || pChannel->SHandwheelSim.bInstructionBound ||
        pChannel->SHandwheelSim.bFault || !isHandwheelSimGroupStopped(&pChannel->SHandwheelSim) ||
        isErrorHasBeenWritten())
    {
        return;
    }
    /* 恢复 G1 已结束，末拍位移/dTc 不是下一条距离运动的非零起速。 */
    pChannel->SRun.dCurrentSpeed = 0.0;
}

/*************************************************
  函数名称：addHandwheelAuthorizedDistance
  函数功能：将原始计数按有效每格距离系统参数和手轮档位累计为待走距离
  输入参数：pState - 接收状态；nSignedRaw - 有符号原始计数；ucMultiple - 1、10、100 档位
  输出参数：pState - 待走距离；参数无效时仅锁存故障
  返 回 值：1 - 接受输入；-1 - 空指针、已有故障或参数无效
  备    注：非正数参数按默认 1 处理；仅新增授权使用新值，不参与测速
****************************************/
int addHandwheelAuthorizedDistance(struct SHandwheelSim* pState, int32 nSignedRaw, uint8 ucMultiple)
{
    double dNewPending;   /**< 叠加本次输入后的候选待走距离，通过有效性检查后才提交。 */
    int nDistancePerTick; /**< 本次输入的有效整数距离快照，单位为微米或千分之一度。 */
    if (pState == NULL)
    {
        return -1;
    }
    nDistancePerTick = getHandwheelSimDistancePerTick();
    /* [HWSIM] /1000 将微米或千分之一度换算为 mm 或度，/4 将计数换为格数。
     * getter 为未初始化的旧参数提供默认距离，不会把零值带入运动。
     * 换档不会追溯放大之前的 dPendingDistance。
     */
    dNewPending =
        pState->dPendingDistance + (double)nSignedRaw * (double)ucMultiple * ((double)nDistancePerTick / 1000.0) / 4.0;
    /* [HWSIM] 在写回前统一检查范围、档位和浮点溢出，失败只锁存故障。
     * 先转 double 再乘，避免在整数类型中乘档位发生溢出。
     */
    if (pState->bFault || nSignedRaw < -65535 || nSignedRaw > 65535 ||
        (ucMultiple != 1 && ucMultiple != 10 && ucMultiple != 100) || !isfinite(dNewPending))
    {
        pState->bFault = TRUE;
        return -1;
    }
    pState->dPendingDistance = dNewPending;
    return 1;
}

/*************************************************
  函数名称：processHandwheelEncoderInput
  函数功能：读取单份编码器样本并按当前高速倍率分发新增授权
  输入参数：pChannel - 已确认可采样的运行通道
  输出参数：pChannel - 测速及授权状态
  返 回 值：1 - 已接收；0 - 无输入或已丢弃；-1 - 参数错误
  备    注：不清共享编码器计数，不修改面板参数；调用者统一报警
****************************************/
static int processHandwheelEncoderInput(struct SChannel* pChannel)
{
    struct SEncoder* pEncoder = pChannel->pSHandleWheelEncoder; /**< 共享编码器，只读。 */
    uint32 unInputMultiple;                                     /**< 先校验共享档位再缩窄，避免非法值截断。 */
    double dInputGain;                                          /**< 本拍新增授权的倍率。 */
    int32 nSignedRaw;                                           /**< 本拍有符号原始计数。 */
    /* [HWSIM] 采样步骤 1：只读共享编码器。没有编码器等同本拍无新输入，
     * 仍更新停摇时钟；不能清 usDelta，否则其它使用同一编码器的通道会丢样本。
     */
    if (pEncoder == NULL)
    {
        resetHandwheelSpeedMeasurement(&pChannel->SHandwheelSim);
        updateHandwheelGroupDistanceLimitAndStop(&pChannel->SHandwheelSim, 0, g_SSystemPara.dInterpolationTime);
        return 0;
    }
    if (pEncoder->usDelta != 0 && pEncoder->cEncoderDir != INP_GO_AHEAD && pEncoder->cEncoderDir != INP_GO_BACK)
    {
        /* 模式交接方向尚未更新时，只丢弃过渡样本。 */
        resetHandwheelSpeedMeasurement(&pChannel->SHandwheelSim);
        return pChannel->SHandwheelSim.bAwaitingFreshInput ? 0 : -1;
    }
    nSignedRaw = pEncoder->usDelta == 0 ? 0 : (int32)pEncoder->cEncoderDir * pEncoder->usDelta;
    /* [HWSIM] 采样步骤 2：先以宽类型验证档位，再传给 uint8 接口，
     * 避免非法大数截断后刚好变成合法的 1/10/100。零计数不新增距离。
     */
    unInputMultiple = g_SSystemPara.MOTOR_WORK[0].dHandWheelFeedRatio;
    if (nSignedRaw != 0 && unInputMultiple != 1 && unInputMultiple != 10 && unInputMultiple != 100)
    {
        return -1;
    }
    dInputGain =
        calculateHandwheelDistanceMultiplier(&pChannel->SHandwheelSim, nSignedRaw, g_SSystemPara.dInterpolationTime);
    if (dInputGain < 1.0)
    {
        return -1;
    }
    if (pEncoder->usDelta == 0)
    {
        /* [HWSIM] 零输入拍仍经过增益衰减和停摇检测，但跳过授权分发。
         * 这与“立刻把轴速度清零”不同，旧授权是否继续由响应状态和规划器决定。
         */
        updateHandwheelGroupDistanceLimitAndStop(&pChannel->SHandwheelSim, 0, g_SSystemPara.dInterpolationTime);
        return 0;
    }
    pChannel->SHandwheelSim.bAwaitingFreshInput = FALSE;
    /* [HWSIM] 采样步骤 3：先把新距离加入账本，再按上一提交末速处理到速限额
     * 与停摇标记。本函数结束后，外层才进入运动函数消费这份授权。
     */
    if (addHandwheelChannelAuthorizedDistance(pChannel, nSignedRaw, (uint8)unInputMultiple, dInputGain) < 0)
    {
        return -1;
    }
    updateHandwheelGroupDistanceLimitAndStop(&pChannel->SHandwheelSim, nSignedRaw, g_SSystemPara.dInterpolationTime);
    return 1;
}

/*************************************************
  函数名称：processHandwheelInput
  函数功能：每插补周期采样一次，按转速比例倍率和直径参数累计新授权
  输入参数：pChannel - 当前通道，允许为空
  输出参数：pChannel - 输入测速及授权状态，非法参数锁存故障
  返 回 值：无
  备    注：零计数周期也更新测速；共享编码器只读，暂停减速沿用原路径
****************************************/
void processHandwheelInput(struct SChannel* pChannel)
{
    /* [HWSIM] 每周期采样总入口，不在 G0/G1/G2/G3 内重复读取编码器。
     * 首次读段时通常尚未绑定，本拍输入不会缓存成新段的初始运动距离。
     */
    if (pChannel == NULL)
    {
        return;
    }
    if (!isHandwheelDistanceControlActive(pChannel))
    {
        leaveHandwheelSim(pChannel);
        return;
    }
    if (pChannel->ucChannelStatus == SYS_RESET || pChannel->ucChannelStatus == SYS_EMERGENCY_STOP ||
        isErrorHasBeenWritten()
#if STM32_MODE
        || Check_bootwarning() == 0
#endif
    )
    {
        transferHandwheelStateToStopControl(pChannel);
        return;
    }
    /* [HWSIM] 暂停中或正在减速暂停时不接收新授权；原有运动状态仍保留，
     * 后续运动函数以 bStop=TRUE 继续逐拍制动，直到输出提交得到零末速。
     */
    if (pChannel->ucChannelStatus != SYS_RUN)
    {
        resetHandwheelSpeedMeasurement(&pChannel->SHandwheelSim);
        return;
    }
    if (pChannel->SHandwheelSim.SPath.bBound)
    {
        /* [HWSIM] 采样前先处理 G99 停转，禁止停转期间积攒距离。
         * 外层本拍稍后还会更新主轴测速，executeHandwheelPathStep() 执行前再次更新限速，
         * 因此不能把这里的预检查当作本拍唯一一次主轴约束检查。
         */
        int nSpindle = updateHandwheelG99SpeedLimit(pChannel); /**< 停转等待或限速错误时禁止接收新输入。 */
        if (nSpindle <= 0)
        {
            resetHandwheelSpeedMeasurement(&pChannel->SHandwheelSim);
            if (nSpindle < 0)
            {
                reportHandwheelPathError(pChannel);
            }
            return;
        }
    }
    /* 快移倍率为零时丢弃本拍输入并请求可控暂停，禁止积压授权后突然恢复。 */
    if (pChannel->SHandwheelSim.SPath.bBound
            ? (!isfinite(pChannel->SRun.dCommonFeedRatio) || pChannel->SRun.dCommonFeedRatio <= 0.0)
            : (!isfinite(pChannel->SRun.dQuickMoveFeedRatio) || pChannel->SRun.dQuickMoveFeedRatio <= 0.0))
    {
        resetHandwheelSpeedMeasurement(&pChannel->SHandwheelSim);
        pauseButtonAPI(pChannel->ucChannelNum);
        return;
    }
    if (processHandwheelEncoderInput(pChannel) < 0)
    {
        /* [HWSIM] 输入非法属于故障，不当成“没摇手轮”静默忽略；
         * 锁存后由原报警流程停止，防止后续采样继续向异常账本追加距离。
         */
        pChannel->SHandwheelSim.bFault = TRUE;
        parseRunningError(pChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
    }
}

/*************************************************
  函数名称：prepareHandwheelMotionSegment
  函数功能：在零速时绑定单轴几何段及固定规划约束
  输入参数：pState - 单轴状态；pSegment - 几何和机床约束参数
  输出参数：pState - 校验成功后的绑定状态，失败时不修改
  返 回 值：1 - 成功；-1 - 状态或参数无效
  备    注：调用者先检查轴类型、映射及限位；不强行裁剪位置或清除非零速度
****************************************/
int prepareHandwheelMotionSegment(struct SHandwheelSim* pState, const struct SHandwheelSimSegment* pSegment)
{
    /* [HWSIM] G0 每轴和 G1/G2/G3 合成路径都复用这一个标量绑定函数。
     * 只接受零末速状态及段内位置；恢复/倒读的初始进度可以不为零。
     * 它不会自行清旧授权，调用者负责在正常结束后清理，再提供完整的新段约束。
     */
    if (pState == NULL || pSegment == NULL || pState->bFault || pState->dBoundaryVelocity != 0.0 ||
        !isfinite(pSegment->dLength) || pSegment->dLength < 0.0 || !isfinite(pSegment->dPosition) ||
        pSegment->dPosition < 0.0 || pSegment->dPosition > pSegment->dLength || !isfinite(pSegment->dPeriod) ||
        pSegment->dPeriod <= 0.0 || !isfinite(pSegment->dSpeedLimit) || pSegment->dSpeedLimit <= 0.0 ||
        !isfinite(pSegment->dAcceleration) || pSegment->dAcceleration <= 0.0 || !isfinite(pSegment->dDeceleration) ||
        pSegment->dDeceleration <= 0.0 || pSegment->ucAxis >= AXIS_NUM_MAX)
        return -1;
    pState->dSegmentLength = pSegment->dLength;
    pState->dSegmentPosition = pSegment->dPosition;
    pState->dPeriod = pSegment->dPeriod;
    pState->dSpeedLimit = pSegment->dSpeedLimit;
    pState->dAcceleration = pSegment->dAcceleration;
    pState->dDeceleration = pSegment->dDeceleration;
    pState->ucAxis = pSegment->ucAxis;
    pState->bSegmentBound = TRUE;
    pState->bFeedLimitActive = FALSE;
    return 1;
}

/*************************************************
  函数名称：calculateHandwheelStopDistance
  函数功能：按最大减速度及完整插补周期计算到零速的离散制动距离
  输入参数：dSpeed - 非负速度；dDeceleration - 正减速度；dPeriod - 插补周期
  输出参数：无
  返 回 值：制动距离，路径单位；速度不大于零时返回零
  备    注：速度单位为路径单位/ms，减速度为路径单位/ms^2，周期为 ms
****************************************/
/* dCycles = ceil(dSpeed / (dDeceleration * dPeriod))，
 * 最后一拍使用恰好降到零的较小减速度。下方返回表达式为各拍梯形面积之和。
 * 与连续制动距离 dSpeed * dSpeed / (2 * dDeceleration) 不同，
 * 该式与实际执行采用相同离散周期，
 * 且在整周期交界处连续、随速度单调递增。 */
static double calculateHandwheelStopDistance(double dSpeed, double dDeceleration, double dPeriod)
{
    double dCycles; /**< 向上取整的制动周期数，用浮点数参与距离求和。 */
    if (dSpeed <= 0.0)
        return 0.0;
    dCycles = ceil(dSpeed / (dDeceleration * dPeriod));
    return dPeriod * ((dCycles - 0.5) * dSpeed - 0.5 * dDeceleration * dPeriod * dCycles * (dCycles - 1.0));
}

/*************************************************
  函数名称：calculateHandwheelMotionStep
  函数功能：按约束预览一拍运动、暂停或停摇制动，不修改账本
  输入参数：pState - 已绑定状态；bStop - 程序暂停请求
  输出参数：pTrial - 候选进度、周期末速度及事件
  返 回 值：1 - 成功；-1 - 参数无效或段边界内无法制动
  备    注：bInputStop 请求独立停摇事件；非零速度不得直接反向
****************************************/
/* 搜索最大的可达周期末速度 dNextSpeed，使本拍梯形位移
 * (dSpeed + dNextSpeed) * dPeriod / 2 加上 dNextSpeed 对应的离散制动距离
 * 不超过前方可用距离。反摇抵消目标后若来不及停在目标处，则按最大减速度制动，
 * 超过目标的距离保留为有符号余量，停稳后再回走。
 * 即使授权目标位于段外，也必须用同样的条件保护当前段端点。 */
int calculateHandwheelMotionStep(const struct SHandwheelSim* pState, bool bStop, struct SHandwheelSimTrial* pTrial)
{
    double dSpeed,            /**< 当前周期起始速度幅值，单位为路径单位/ms。 */
        dMinSpeed,            /**< 按最大减速度执行一拍后的最小可达速度幅值。 */
        dMaxSpeed,            /**< 同时满足加速度和限速要求的搜索速度上界。 */
        dNextSpeed,           /**< 本拍最终采用的周期末速度幅值，也是二分搜索下界。 */
        dDirection;           /**< 本拍段进度方向，+1 为前进，-1 为后退。 */
    double dBoundaryDistance, /**< 沿当前运动方向到当前段边界的可用距离。 */
        dTargetDistance,      /**< 授权目标和段边界共同限定的前方可用距离。 */
        dMinTravel,           /**< 本拍按最大减速度执行时所需的位移幅值。 */
        dCandidate;           /**< 二分搜索的候选周期末速度幅值。 */
    double dTravel,           /**< 本拍实际采用的理论位移幅值。 */
        dNextPosition,        /**< 由当前段进度和本拍位移计算的候选段进度。 */
        dRequired,            /**< 本拍位移与下一拍开始继续制动到零的距离之和。 */
        dTolerance;           /**< 按段长缩放的浮点误差容差，不用于丢弃授权距离。 */
    int nIteration;           /**< 固定次数二分搜索的循环计数。 */
    bool bInputStop = !bStop && pState != NULL && pState->bInputStop; /**< 停摇等待不等于程序暂停。 */
    if (pState == NULL || pTrial == NULL || !pState->bSegmentBound || pState->bFault ||
        !isfinite(pState->dPendingDistance) || !isfinite(pState->dBoundaryVelocity))
        return -1;
    bStop = bStop || bInputStop;
    /* [HWSIM] 规划步骤 1：把输出候选初始化为“原位、原速度、等待”。
     * 程序暂停和停摇都要减速，但完成事件不同：STOPPED 要交给暂停回写，
     * INPUT_IDLE 只清余量并保留当前段，等下一次输入继续。
     */
    pTrial->dPosition = pState->dSegmentPosition;
    pTrial->dVelocity = pState->dBoundaryVelocity;
    pTrial->eStep = HWSIM_STEP_WAIT;
    dSpeed = fabs(pState->dBoundaryVelocity);
    if (dSpeed == 0.0)
    {
        /* [HWSIM] 规划步骤 2：只有零速才能按授权正负选择新的运动方向。
         * 零速无授权就等待；零速且已在请求方向的端点则报告端点，不越界走一步。
         */
        if (bStop)
        {
            pTrial->eStep = bInputStop ? HWSIM_STEP_INPUT_IDLE : HWSIM_STEP_STOPPED;
            return 1;
        }
        if (fabs(pState->dPendingDistance) <= HWSIM_DISTANCE_EPS)
            return 1;
        dDirection = pState->dPendingDistance > 0.0 ? 1.0 : -1.0;
        if ((dDirection > 0.0 && pState->dSegmentPosition >= pState->dSegmentLength) ||
            (dDirection < 0.0 && pState->dSegmentPosition <= 0.0))
        {
            pTrial->eStep = dDirection > 0.0 ? HWSIM_STEP_FORWARD_END : HWSIM_STEP_BACKWARD_END;
            return 1;
        }
    }
    else
        dDirection = pState->dBoundaryVelocity > 0.0 ? 1.0 : -1.0;

    /* [HWSIM] 规划步骤 3：区分“几何边界”和“授权目标”。
     * 几何边界绝不允许越过；反摇可能把授权目标拉到当前位置后方，此时前方
     * 授权为零，但仍需沿旧方向完成物理上可实现的减速，不能立即翻转速度符号。
     */
    dBoundaryDistance = dDirection > 0.0 ? pState->dSegmentLength - pState->dSegmentPosition : pState->dSegmentPosition;
    dTargetDistance = fmin(dBoundaryDistance, fmax(0.0, dDirection * pState->dPendingDistance));
    dMinSpeed = fmax(0.0, dSpeed - pState->dDeceleration * pState->dPeriod);
    /* 最后一拍相减可能留下机器精度尾速；不能用距离/脉冲容差吞掉真实低速。 */
    if (dMinSpeed <= 32.0 * DBL_EPSILON * fmax(dSpeed, pState->dDeceleration * pState->dPeriod))
    {
        dMinSpeed = 0.0;
    }
    dMinTravel = 0.5 * (dSpeed + dMinSpeed) * pState->dPeriod;
    /* [HWSIM] 规划步骤 4：先尝试最大减速度这一保底方案。
     * dRequired = 本拍制动位移 + 从本拍末速继续制动到零的离散距离。
     * 连这个方案都停不在段内就报错，不能强制裁剪位置伪装成正常减速。
     */
    dRequired = dMinTravel + calculateHandwheelStopDistance(dMinSpeed, pState->dDeceleration, pState->dPeriod);
    dTolerance = HWSIM_DISTANCE_EPS * fmax(1.0, pState->dSegmentLength);
    if (!isfinite(dRequired) || dRequired > dBoundaryDistance + dTolerance)
        return -1;

    dNextSpeed = dMinSpeed;
    /* 与零速等待使用相同的授权容差；尾部余量不再触发加速搜索，
     * 否则不可表示的位置增量会反复生成非零尾速，锁住 DRAIN 的零速交接。
     * 保留上面算出的最大减速结果，不直接清零速度或取消剩余授权。 */
    if (!bStop && fabs(pState->dPendingDistance) > HWSIM_DISTANCE_EPS && dRequired < dTargetDistance)
    {
        /* [HWSIM] 规划步骤 5：只有没有停车请求且目标前方还有制动裕量才搜索更快末速。
         * 搜索下界为最大减速后的末速，上界由本拍加速度与限速共同决定。
         * G99 限速突然降到当前速度以下时，上界仍不能低于 dMinSpeed，
         * 否则等于要求超过最大减速度的瞬间降速。
         */
        dMaxSpeed = fmax(dMinSpeed, fmin(pState->dSpeedLimit, dSpeed + pState->dAcceleration * pState->dPeriod));
        /* 固定迭代次数，保证微小行程下中断内搜索次数也有明确上界。 */
        for (nIteration = 0; nIteration < 48; ++nIteration)
        {
            dCandidate = 0.5 * (dNextSpeed + dMaxSpeed);
            dRequired = 0.5 * (dSpeed + dCandidate) * pState->dPeriod +
                        calculateHandwheelStopDistance(dCandidate, pState->dDeceleration, pState->dPeriod);
            /* [HWSIM] 候选速度能在授权目标之前停住则抬高下界，否则压低上界。
             * 每次都把本拍位移计入，不能只用 v*v/(2*a) 检查下一拍之后的制动。
             */
            if (dRequired <= dTargetDistance)
                dNextSpeed = dCandidate;
            else
                dMaxSpeed = dCandidate;
        }
    }
    /* [HWSIM] 规划步骤 6：以周期初末速度的平均值积分本拍位移，
     * 再沿本拍方向推进理论进度。只生成 pTrial，不修改 pState。
     */
    dTravel = 0.5 * (dSpeed + dNextSpeed) * pState->dPeriod;
    dNextPosition = pState->dSegmentPosition + dDirection * dTravel;
    if (!isfinite(dNextPosition) || !isfinite(dNextSpeed))
        return -1;
    /* 只在零速时修正端点浮点舍入误差，不能截断仍在运动的一拍。 */
    if (dNextSpeed == 0.0)
    {
        if (fabs(dNextPosition) <= dTolerance)
            dNextPosition = 0.0;
        if (fabs(dNextPosition - pState->dSegmentLength) <= dTolerance)
            dNextPosition = pState->dSegmentLength;
    }
    if (dNextPosition < 0.0 || dNextPosition > pState->dSegmentLength)
        return -1;
    pTrial->dPosition = dNextPosition;
    pTrial->dVelocity = dDirection * dNextSpeed;
    /* [HWSIM] 规划步骤 7：只在末速真正为零时报告停车完成；
     * 否则仍是 MOVE，即使这一拍已在减速或待走授权已经被反摇抵消。
     * 段首/段尾事件由后续条件补充，是否换段仍由 G0 全轴或合成路径上层决定。
     */
    pTrial->eStep =
        bStop && dNextSpeed == 0.0 ? (bInputStop ? HWSIM_STEP_INPUT_IDLE : HWSIM_STEP_STOPPED) : HWSIM_STEP_MOVE;
    /* 恰好到达端点时，即使余量为零也报告本轴端点事件。
     * 多轴总控仍需等待所有轴满足完成条件，不能在此处解除绑定。 */
    if (!bStop && dNextSpeed == 0.0 && dTravel > 0.0 &&
        dDirection * (pState->dPendingDistance - (dNextPosition - pState->dSegmentPosition)) >= -HWSIM_DISTANCE_EPS)
    {
        if (dDirection > 0.0 && dNextPosition == pState->dSegmentLength)
            pTrial->eStep = HWSIM_STEP_FORWARD_END;
        else if (dDirection < 0.0 && dNextPosition == 0.0)
            pTrial->eStep = HWSIM_STEP_BACKWARD_END;
    }
    return 1;
}

/*************************************************
  函数名称：updateHandwheelMotionState
  函数功能：提交输出已接受的理论位置和末速，更新剩余授权
  输入参数：pTrial - 同一中断中通过预览及输出检查的候选
  输出参数：pState - 已提交的账本，调用方保证指针有效
  返 回 值：无
  备    注：暂停或停摇停稳时取消余量；成功零速解除到速限额，停摇保持几何绑定
****************************************/
void updateHandwheelMotionState(struct SHandwheelSim* pState, const struct SHandwheelSimTrial* pTrial)
{
    /* [HWSIM] 提交顺序不能颠倒：先用新旧进度差扣剩余授权，再覆盖旧进度。
     * 正向位移扣正距离，反向位移扣负距离，因而统一保留有符号抵消关系。
     * 未加新输入且未触发清余量时，“进度 + 剩余授权”在一次提交前后保持不变。
     * 调用者必须保证本拍输出已成功；这里不是再次规划，也不验证硬件到位。
     */
    pState->dPendingDistance -= pTrial->dPosition - pState->dSegmentPosition;
    pState->dSegmentPosition = pTrial->dPosition;
    pState->dBoundaryVelocity = pTrial->dVelocity;
    if (pTrial->dVelocity == 0.0)
    {
        pState->bFeedLimitActive = FALSE;
    }
    /* [HWSIM] 普通 WAIT 或到端点不在这里一律清余量，G0 还需据此判断全轴方向。
     * 仅程序暂停/停摇已停稳时取消剩余请求；段结束后的通道清理由上层统一完成。
     */
    if (pTrial->eStep == HWSIM_STEP_STOPPED || pTrial->eStep == HWSIM_STEP_INPUT_IDLE)
    {
        pState->dPendingDistance = 0.0;
    }
}
#endif
