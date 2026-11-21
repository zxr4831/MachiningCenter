#include "G0.h"
#include <stdio.h>
#include "Motion.h"
#include "GeneralFunctions.h"
#include <stdlib.h>
#include "CTrajPlanning.h"
#include "G1.h"
#include "Spindle.h"
#include "ringbuff.h"
#include "Channel.h"
#include "Config.h"
#include "ErrorCompensation.h"

#if HANDWHEEL_SIM_DISTANCE_ENABLE
#include "HandwheelSim.h"
#include <math.h>
#include <string.h>
/* [HWSIM] G0 距离型手轮的阅读路线
 * 首次：motionQuickMove() 装载几何 -> prepareG0HandwheelMove() ->
 *       对每个非零行程轴 prepareG0HandwheelAxis() -> prepareHandwheelMotionSegment()。
 * 每拍：外层已先 processHandwheelInput() -> singleAxisMotionLoop() 分流 ->
 *       executeG0HandwheelStep() -> 全轴 preview -> 全轴 output -> 全轴 commit ->
 *       processG0HandwheelBoundary() 汇总完成/回退/暂停状态。
 * 与 G1/G2/G3 不同，G0 不是一份合成进度带所有轴，而是每轴一份标量状态；
 * 短轴到端点仍保持绑定，等其它轴期间也可以响应反摇。
 * 以下代码计算的是软件脉冲缓存；不在这些函数中直接完成硬件发脉冲。
 */
extern SScrewComp g_SScrewComp[AXIS_PHYSICS_NUM_MAX]; /**< 原补偿账本，本拍失败时恢复。 */

/*************************************************
  函数名称：reportG0HandwheelDistanceError
  函数功能：锁存通道错误并逐轴交接真实速度；清除本拍候选脉冲，不回退到速度型模拟。
  输入参数：pG0 - 当前 G0 运动状态
  输出参数：通道故障、轴速交接和候选脉冲
  返 回 值：-1 - 已锁存运动故障
  备    注：迁移保留原函数执行逻辑，未改变接口或单位
****************************************/
static int8 reportG0HandwheelDistanceError(struct SG0* pG0)
{
    struct SChannel* pChannel = pG0->pSChannel; /**< 所属通道。 */
    uint8 ucAxis;                               /**< 逻辑轴索引。 */
    /* [HWSIM] 清掉整帧失败候选，并用最后已提交末速向原报警停车流程交接。
     * 不能使用失败候选的速度，也不能让某些轴继续执行本拍部分成功的输出。
     */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        struct SHandwheelSim* pState = &pChannel->SHandwheelSim.SAxisStateArray[ucAxis]; /**< 本轴已提交状态。 */
        pChannel->SRun.sAxisDeltaPulse[ucAxis] = 0;
        if (pState->bSegmentBound)
        {
            pG0->pSSingleAxisArray[ucAxis].dCurrentSpeed = fabs(pState->dBoundaryVelocity);
            if (pState->dBoundaryVelocity != 0.0)
                pG0->pSSingleAxisArray[ucAxis].SPlanData.cInterpDir =
                    pState->dBoundaryVelocity > 0.0 ? INP_GO_AHEAD : INP_GO_BACK;
        }
    }
    pChannel->SHandwheelSim.bFault = TRUE;
    parseRunningError(pChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
    return -1;
}

/*************************************************
  函数名称：isG0HandwheelAxisValid
  函数功能：检查单轴几何、物理映射和整段软限位；不裁剪几何来规避限位。
  输入参数：pG0 - G0 状态；ucAxis - 逻辑轴号
  输出参数：无
  返 回 值：TRUE - 轴几何与约束有效；FALSE - 参数或限位不满足
  备    注：迁移保留原函数执行逻辑，未改变接口或单位
****************************************/
static bool isG0HandwheelAxisValid(struct SG0* pG0, uint8 ucAxis)
{
    struct SSingleAxis* pAxis = &pG0->pSSingleAxisArray[ucAxis]; /**< 待检查轴。 */
    uint8 ucPhysical = pG0->pSChannel->pMapAxisArray[ucAxis];    /**< 对应物理轴。 */
    uint8 ucPrevious;                                            /**< 检查之前参与轴，禁止两个逻辑轴控制同一物理轴。 */
    double dLow = fmin(pAxis->dStartPoint, pAxis->dEndPoint);    /**< 段内最低坐标。 */
    double dHigh = fmax(pAxis->dStartPoint, pAxis->dEndPoint);   /**< 段内最高坐标。 */
    /* [HWSIM] 段方向必须为 +/-1，脉冲当量为正，绑定前轴速度为零。
     * dLow/dHigh 用于检查整段范围，不只检查本次手轮目标；
     * 逻辑轴映射重复会让两份独立授权控制同一物理轴，所以必须拒绝。
     */
    if (ucPhysical >= AXIS_PHYSICS_NUM_MAX || pAxis->ucRunMode != SINGLEAXIS_MODE || !isfinite(pAxis->dStartPoint) ||
        !isfinite(pAxis->dEndPoint) || !isfinite(pAxis->dStartVector) ||
        fabs(fabs(pAxis->dStartVector) - 1.0) > HWSIM_DISTANCE_EPS || !isfinite(pAxis->dPulseToMm) ||
        pAxis->dPulseToMm <= 0.0 || pAxis->dCurrentSpeed != 0.0)
        return FALSE;
    for (ucPrevious = 0; ucPrevious < ucAxis; ++ucPrevious)
        if (pG0->pSChannel->SHandwheelSim.SAxisStateArray[ucPrevious].bSegmentBound &&
            pG0->pSChannel->pMapAxisArray[ucPrevious] == ucPhysical)
            return FALSE;
    if (g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == LINEAR_AXIS)
        return !(g_SSystemPara.MOTOR_WARN[ucPhysical].soft_neg &&
                 (!isfinite(g_SSystemPara.MOTOR_WARN[ucPhysical].neg_soft) ||
                  dLow < g_SSystemPara.MOTOR_WARN[ucPhysical].neg_soft)) &&
               !(g_SSystemPara.MOTOR_WARN[ucPhysical].soft_pos &&
                 (!isfinite(g_SSystemPara.MOTOR_WARN[ucPhysical].pos_soft) ||
                  dHigh > g_SSystemPara.MOTOR_WARN[ucPhysical].pos_soft));
    return g_SSystemPara.SAxisPara[ucPhysical].ucAxisType == ROTATION_AXIS;
}

/*************************************************
  函数名称：prepareG0HandwheelAxis
  函数功能：为一个参与轴建立固定约束，复用标量规划器，不增加多轴插补算法。
  输入参数：pG0 - G0 状态；ucAxis - 待绑定逻辑轴号
  输出参数：本轴手轮距离绑定状态
  返 回 值：绑定结果；负数表示参数或绑定失败
  备    注：迁移保留原函数执行逻辑，未改变接口或单位
****************************************/
static int prepareG0HandwheelAxis(struct SG0* pG0, uint8 ucAxis)
{
    struct SChannel* pChannel = pG0->pSChannel;                  /**< 所属通道。 */
    struct SSingleAxis* pAxis = &pG0->pSSingleAxisArray[ucAxis]; /**< 原 G0 轴对象。 */
    struct SHandwheelSimSegment SSegment;                        /**< 本轴几何及运动约束。 */
    double dAcceleration = getSingleAxisAcc(ucAxis, VEL_PLAN_T_CURVE, pChannel->pMapAxisArray); /**< 轴加速度。 */
    double dDeceleration = getSingleAxisDec(ucAxis, VEL_PLAN_T_CURVE, pChannel->pMapAxisArray); /**< 轴减速度。 */
    if (!isG0HandwheelAxisValid(pG0, ucAxis) || !isfinite(pAxis->dFeedrate) || pAxis->dFeedrate <= 0.0 ||
        !isfinite(dAcceleration) || dAcceleration <= 0.0 || !isfinite(dDeceleration) || dDeceleration <= 0.0)
        return -1;
    SSegment.ucAxis = ucAxis;
    /* [HWSIM] 初始进度 = 总长 - 剩余距离，适用于新段、暂停恢复和历史倒读。
     * 每轴分别取模拟限速、原轴进给和机床约束中的较小值；
     * 不按各轴长度比例同步速度，G0 仍然保持独立轴运动语义。
     */
    SSegment.dLength = pAxis->SPlanData.dTargetDist;
    SSegment.dPosition = SSegment.dLength - pAxis->SPlanData.dCurLeftDist;
    SSegment.dPeriod = pAxis->SPlanData.dTc;
    SSegment.dSpeedLimit = fmin(HANDWHEEL_SIM_G0_SPEED_LIMIT, fmin(pAxis->dFeedrate, g_SSystemPara.dSimulaG0Vmax));
    SSegment.dAcceleration = fmin(dAcceleration, g_SSystemPara.dSimulaMaxAcc);
    SSegment.dDeceleration = fmin(dDeceleration, g_SSystemPara.dSimulaMaxDec);
    if (prepareHandwheelMotionSegment(&pChannel->SHandwheelSim.SAxisStateArray[ucAxis], &SSegment) < 0)
        return -1;
    /* [HWSIM] 每条距离段要求零末速交接，不使用旧时间规划器的带速拼接终速。
     * INP_RUNNING 表示几何已准备好；没有新授权时仍然原位等待。
     */
    pAxis->SPlanData.dVe = pAxis->SPlanData.dPrimalVe = 0.0;
    pAxis->ucInpStatus = INP_RUNNING;
    return 1;
}

/*************************************************
  函数名称：prepareG0HandwheelMove
  函数功能：每条指令只绑定一次；先清旧授权，检查全部参与轴后才开放输入。
  输入参数：pG0 - 当前 G0 运动状态
  输出参数：通道及各参与轴的距离绑定状态
  返 回 值：非负数 - 绑定处理完成；负数 - 绑定失败
  备    注：迁移保留原函数执行逻辑，未改变接口或单位
****************************************/
static int8 prepareG0HandwheelMove(struct SG0* pG0)
{
    struct SChannel* pChannel = pG0->pSChannel; /**< 当前通道。 */
    uint8 ucAxis;                               /**< 参与轴扫描索引。 */
    /* [HWSIM] 此入口只在本条 G0 装载成功后调用，不是每个周期重新绑定。
     * 先确认软件零速和固定参数有效，再清旧授权，避免带速丢失上一段状态。
     */
    if (pG0->bIsUserData || pChannel->SHandwheelSim.bFault || !isHandwheelSimGroupStopped(&pChannel->SHandwheelSim) ||
        getBits(pChannel->SRun.unCommandType, CMD_INTERPOLATION_MODE_ADDR) != CMD_INTERPOLATION_COMMON ||
        !isfinite(g_SSystemPara.dSimulaG0Vmax) || g_SSystemPara.dSimulaG0Vmax <= 0.0 ||
        !isfinite(g_SSystemPara.dSimulaMaxAcc) || g_SSystemPara.dSimulaMaxAcc <= 0.0 ||
        !isfinite(g_SSystemPara.dSimulaMaxDec) || g_SSystemPara.dSimulaMaxDec <= 0.0 ||
        !isfinite(HANDWHEEL_SIM_G0_SPEED_LIMIT) || HANDWHEEL_SIM_G0_SPEED_LIMIT <= 0.0)
        return reportG0HandwheelDistanceError(pG0);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE
    if (pChannel->bUseVirtualY)
        return reportG0HandwheelDistanceError(pG0);
#endif
#if ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
    /* 双表补偿在后续独立阶段才计算脉冲，尚未纳入本拍全轴事务，保守拒绝。 */
    return reportG0HandwheelDistanceError(pG0);
#endif
    clearHandwheelSimMotion(&pChannel->SHandwheelSim);
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        struct SSingleAxis* pAxis = &pG0->pSSingleAxisArray[ucAxis]; /**< 当前轴原几何。 */
        if (VAL_VALID != getbit(pG0->ucValStatus, ucAxis))
            continue;
        if (!isfinite(pAxis->SPlanData.dTargetDist) || pAxis->SPlanData.dTargetDist < 0.0)
            return reportG0HandwheelDistanceError(pG0);
        if (pAxis->SPlanData.dTargetDist <= HWSIM_DISTANCE_EPS)
        {
            /* [HWSIM] 零行程轴直接标记完成，不需要建立距离状态；
             * 非零行程轴逐个绑定，全部成功后才用 bInstructionBound 开放采样。
             */
            pAxis->ucInpStatus = INP_FINISHED;
            setbit(pG0->ucCurrentSigned, ucAxis);
        }
        else if (prepareG0HandwheelAxis(pG0, ucAxis) < 0)
            return reportG0HandwheelDistanceError(pG0);
    }
    pChannel->SHandwheelSim.bInstructionBound = TRUE;
    pChannel->SRun.dCurrentSpeed = 0.0;
    return 1;
}

/** 本拍全轴候选；只在所有规划和输出计算成功后写入距离状态与实时坐标。 */
struct SG0HandwheelFrame
{
    struct SHandwheelSimTrial STrialArray[AXIS_NUM_MAX]; /**< 各轴的预览结果。 */
    double dCoordinateArray[AXIS_NUM_MAX];               /**< 补偿及量化后的候选绝对坐标。 */
};

/*************************************************
  函数名称：calculateG0HandwheelMotionStep
  函数功能：预览全部轴；短轴在端点保持绑定，后续输入仍可令其反向。
  输入参数：pG0 - G0 状态；pFrame - 当前拍预览帧
  输出参数：pFrame - 各轴候选位置与规划结果
  返 回 值：非负数 - 预览完成；负数 - 预览失败
  备    注：迁移保留原函数执行逻辑，未改变接口或单位
****************************************/
static int calculateG0HandwheelMotionStep(struct SG0* pG0, struct SG0HandwheelFrame* pFrame)
{
    uint8 ucAxis;                                                     /**< 逻辑轴索引。 */
    bool bStop = pG0->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE; /**< 全轴暂停请求。 */
    /* [HWSIM] 第一遍只生成全轴候选，不更新各轴进度、末速和实时指令坐标。
     * 暂停请求同时传给每个参与轴，但由于速度/减速度不同，各轴可能不同拍停稳。
     */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        struct SHandwheelSim* pState = &pG0->pSChannel->SHandwheelSim.SAxisStateArray[ucAxis]; /**< 本轴账本。 */
        struct SSingleAxis* pAxis = &pG0->pSSingleAxisArray[ucAxis];                           /**< 本轴原几何。 */
        if (!pState->bSegmentBound)
            continue;
        if (calculateHandwheelMotionStep(pState, bStop, &pFrame->STrialArray[ucAxis]) < 0)
            return -1;
        /* [HWSIM] 标量进度从段首累计，乘原几何方向后才得到机床绝对坐标。
         * 因而正摇表示沿程序段前进，不保证轴的机床坐标数值一定增加。
         */
        pFrame->dCoordinateArray[ucAxis] =
            pAxis->dStartPoint + pAxis->dStartVector * pFrame->STrialArray[ucAxis].dPosition;
        if (!isfinite(pFrame->dCoordinateArray[ucAxis]))
            return -1;
    }
    return 1;
}

/*************************************************
  函数名称：generateG0HandwheelPulses
  函数功能：计算全轴候选脉冲；失败时恢复补偿记录，禁止部分轴坐标提前提交。
  输入参数：pG0 - G0 状态；pFrame - 当前拍预览帧
  输出参数：通道候选脉冲与补偿状态
  返 回 值：非负数 - 输出处理完成；负数 - 输出失败
  备    注：迁移保留原函数执行逻辑，未改变接口或单位
****************************************/
static int generateG0HandwheelPulses(struct SG0* pG0, struct SG0HandwheelFrame* pFrame)
{
    struct SChannel* pChannel = pG0->pSChannel;          /**< 所属通道。 */
    struct _SRun* pRun = &pChannel->SRun;                /**< 原脉冲缓存，不在此处发送硬件脉冲。 */
    SPointDouble SLastCoordinate = pRun->SLastAbsCoord;  /**< 输出前的运行坐标快照。 */
    SScrewComp SCompensationArray[AXIS_PHYSICS_NUM_MAX]; /**< 原补偿记录的小型快照。 */
    uint8 ucAxis;                                        /**< 逻辑轴索引。 */
    memcpy(SCompensationArray, g_SScrewComp, sizeof(SCompensationArray));
    /* [HWSIM] 第二遍统一计算脉冲：保留原补偿和指令坐标快照，以便任一轴失败
     * 时撤销软件副作用。禁止“一个轴输出后立即提交，下一轴再检查”的顺序。
     */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        struct SSingleAxis* pAxis = &pG0->pSSingleAxisArray[ucAxis]; /**< 当前轴。 */
        if (!pChannel->SHandwheelSim.SAxisStateArray[ucAxis].bSegmentBound)
            continue;
#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
        pFrame->dCoordinateArray[ucAxis] = outPutAbsPointWithScrewCompForG0(
            pRun, pFrame->dCoordinateArray[ucAxis], ucAxis, pChannel->pMapAxisArray, pAxis->dPulseToMm);
#else
        double dLastCoordinate; /**< 上一拍实际指令坐标。 */
        double dPulseCount;     /**< 转为整数前校验范围的候选脉冲数。 */
        getSingleAxisRealTimePoint(&dLastCoordinate, ucAxis, pChannel->pMapAxisArray);
        /* [HWSIM] 理论绝对点减去上次量化后的指令坐标，再除脉冲当量取整。
         * 小于一脉冲的理论进度可以先保留，后续目标继续变化时自然累积成脉冲；
         * 路径理论位置与量化后的实时指令位置不是完全相同的一份状态。
         */
        dPulseCount = round((pFrame->dCoordinateArray[ucAxis] - dLastCoordinate) / pAxis->dPulseToMm);
        if (!isfinite(dPulseCount) || dPulseCount < -32768.0 || dPulseCount > 32767.0)
            break;
        pRun->sAxisDeltaPulse[ucAxis] = (int16)dPulseCount;
        pFrame->dCoordinateArray[ucAxis] = dLastCoordinate + dPulseCount * pAxis->dPulseToMm;
#endif
        if (isErrorHasBeenWritten() || !isfinite(pFrame->dCoordinateArray[ucAxis]))
            break;
    }
    if (ucAxis == AXIS_NUM_MAX)
        return 1;
    /* [HWSIM] 循环提前退出表示整帧无效。这里只恢复补偿和坐标快照；
     * executeG0HandwheelStep() 接到失败后统一清全部候选脉冲并报警。
     */
    memcpy(g_SScrewComp, SCompensationArray, sizeof(SCompensationArray));
    pRun->SLastAbsCoord = SLastCoordinate;
    return -1; /* 调用方统一清候选脉冲并交给原报警停止路径。 */
}

/*************************************************
  函数名称：updateG0HandwheelMotionState
  函数功能：全轴提交已经通过输出检查的候选
  输入参数：pG0 - 当前 G0；pFrame - 本拍候选
  输出参数：pG0 - 坐标、速度和各轴距离状态
  返 回 值：无
  备    注：提交与端点汇总分离，不单独结束先到端点的短轴
****************************************/
static void updateG0HandwheelMotionState(struct SG0* pG0, const struct SG0HandwheelFrame* pFrame)
{
    struct SChannel* pChannel = pG0->pSChannel; /**< 当前通道。 */
    uint8 ucAxis;                               /**< 逻辑轴索引。 */
    pChannel->SRun.dCurrentSpeed = 0.0;
    /* [HWSIM] 第三遍仅在全轴输出检查成功后执行：
     * 用理论位移求平均速度 -> 写量化后的指令坐标 -> 提交理论进度/周期末速度。
     * 两种位置各司其职，不能用脉冲取整后的位移替代标量规划器的距离账本。
     */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        struct SHandwheelSim* pState = &pChannel->SHandwheelSim.SAxisStateArray[ucAxis]; /**< 本轴账本。 */
        struct SSingleAxis* pAxis = &pG0->pSSingleAxisArray[ucAxis];                     /**< 原轴运行数据。 */
        const struct SHandwheelSimTrial* pTrial = &pFrame->STrialArray[ucAxis];          /**< 已验证候选。 */
        double dCoordinate = pFrame->dCoordinateArray[ucAxis]; /**< 原坐标接口要求可写指针。 */
        if (!pState->bSegmentBound)
            continue;
        pAxis->dCurrentSpeed = fabs(pTrial->dPosition - pState->dSegmentPosition) / pState->dPeriod;
        updateSingleAxisRealTimePoint(&dCoordinate, ucAxis, pChannel->pMapAxisArray);
        updateHandwheelMotionState(pState, pTrial);
        /* [HWSIM] 通道显示速度取各轴平均速度的最大值，不是欧氏合成速度。
         * 剩余距离和方向仍回写旧轴对象，供外层结束、暂停回写等流程使用。
         */
        pChannel->SRun.dCurrentSpeed = fmax(pChannel->SRun.dCurrentSpeed, pAxis->dCurrentSpeed);
        pAxis->SPlanData.dCurLeftDist = pState->dSegmentLength - pState->dSegmentPosition;
        if (pTrial->dVelocity != 0.0)
            pAxis->SPlanData.cInterpDir = pTrial->dVelocity > 0.0 ? INP_GO_AHEAD : INP_GO_BACK;
    }
}

/*************************************************
  函数名称：processG0HandwheelBoundary
  函数功能：汇总全轴零速端点，首端无法继续倒读时取消向外负授权
  输入参数：pG0 - 当前 G0
  输出参数：pG0 - 首端等待标记及各轴无效负授权
  返 回 值：1 - 正向完成或暂停；-1 - 倒读上一段；0 - 继续等待
  备    注：倒读绑定在段尾时必须等待新输入，不能立即自动正向完成
****************************************/
static int8 processG0HandwheelBoundary(struct SG0* pG0)
{
    struct SHandwheelSimGroup* pGroup = &pG0->pSChannel->SHandwheelSim; /**< 通道账本。 */
    bool bForward = TRUE, bBackward = TRUE, bHasAxis = FALSE;           /**< 全轴边界汇总。 */
    bool bNegative = FALSE;                                             /**< 至少一个轴仍请求回退。 */
    uint8 ucAxis;                                                       /**< 逻辑轴索引。 */
    if (!isHandwheelSimGroupStopped(pGroup))
        return 0;
    /* [HWSIM] 必须全部轴的已提交末速为零才能讨论换段。
     * 暂停先交给外层 quickMoveWriteBack() 回写停点，不继续做历史倒读。
     */
    if (pG0->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE)
        return INP_GO_AHEAD;
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
    {
        const struct SHandwheelSim* pState = &pGroup->SAxisStateArray[ucAxis]; /**< 本轴状态。 */
        if (!pState->bSegmentBound)
            continue;
        bHasAxis = TRUE;
        if (pState->dSegmentPosition != pState->dSegmentLength || pState->dPendingDistance < -HWSIM_DISTANCE_EPS)
            bForward = FALSE;
        if (pState->dSegmentPosition != 0.0 || pState->dPendingDistance > HWSIM_DISTANCE_EPS)
            bBackward = FALSE;
        if (pState->dPendingDistance < -HWSIM_DISTANCE_EPS)
            bNegative = TRUE;
    }
    /* [HWSIM] 正向完成不仅看各轴在段尾，还要求没有反向待执行授权。
     * 倒读刚绑定在段尾时，需等待输入表明意图，不能因“已经在段尾”立即正向结束。
     */
    if (bForward &&
        (pG0->cInterpDir != INP_GO_BACK || (bHasAxis ? pGroup->cInputDirection != 0 : pGroup->cInputDirection > 0)))
        return INP_GO_AHEAD;
    if (!bBackward || !(bNegative || (!bHasAxis && pG0->cInterpDir == INP_GO_BACK)))
        return 0;
    if (hasHandwheelReverseHistory(pG0->pSChannel, FALSE))
        return INP_GO_BACK;
    /* [HWSIM] 全轴退到段首但历史不可回退：取消向外的负授权，静默等待。
     * 后续正向输入可解除 bReverseBlocked；不是报警，也不是越过屏障继续倒读。
     */
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
        if (pGroup->SAxisStateArray[ucAxis].dPendingDistance < 0.0)
            pGroup->SAxisStateArray[ucAxis].dPendingDistance = 0.0;
    pGroup->bReverseBlocked = TRUE;
    return 0;
}

/*************************************************
  函数名称：executeG0HandwheelStep
  函数功能：执行全轴候选事务并在零速后统一报告正向或反向完成
  输入参数：pG0 - 当前 G0
  输出参数：pG0 - 全轴状态、完成位和读段方向
  返 回 值：1 - 执行或等待；-1 - 锁存输出错误
  备    注：换段清除距离，仅保留端点方向供段间游标校准
****************************************/
static int8 executeG0HandwheelStep(struct SG0* pG0)
{
    struct SChannel* pChannel = pG0->pSChannel; /**< 当前通道。 */
    struct SG0HandwheelFrame SFrame = {0};      /**< 本拍全轴候选，局部存储，不跨通道共享。 */
    uint8 ucAxis;                               /**< 完成收尾时的逻辑轴索引。 */
    int8 cBoundary;                             /**< 本拍全轴完成方向。 */
    /* [HWSIM] 每拍步骤 1~3：检查绑定/故障 -> 预览全部轴 -> 计算全部轴输出。
     * || 从左到右短路，任何一步失败都不提交 SFrame。
     */
    if (!pChannel->SHandwheelSim.bInstructionBound || pChannel->SHandwheelSim.bFault || isErrorHasBeenWritten() ||
        calculateG0HandwheelMotionStep(pG0, &SFrame) < 0 || generateG0HandwheelPulses(pG0, &SFrame) < 0)
        return reportG0HandwheelDistanceError(pG0);
    /* [HWSIM] 每拍步骤 4~5：整帧提交后才汇总全轴边界。
     * 某短轴先到端点时这里只继续等待，不能提前解除该轴绑定。
     */
    updateG0HandwheelMotionState(pG0, &SFrame);
    cBoundary = processG0HandwheelBoundary(pG0);
    if (cBoundary == 0)
        return 1;
    /* [HWSIM] 每拍步骤 6：确认完成/暂停后统一写各轴完成位及读段方向。
     * 本函数完成时仍返回 1，外层通过 ucCurrentSigned == ucFinishedSigned
     * 识别完成，不可把这个返回值单独理解为“所有轴仍在运动”。
     */
    pG0->cInterpDir = cBoundary;
    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ++ucAxis)
        if (pChannel->SHandwheelSim.SAxisStateArray[ucAxis].bSegmentBound)
        {
            pG0->pSSingleAxisArray[ucAxis].ucInpStatus = INP_FINISHED;
            pG0->pSSingleAxisArray[ucAxis].SPlanData.cInterpDir = cBoundary;
            pG0->pSSingleAxisArray[ucAxis].dCurrentSpeed = 0.0;
            setbit(pG0->ucCurrentSigned, ucAxis);
        }
    clearHandwheelSimMotion(&pChannel->SHandwheelSim);
    if (pChannel->ucChannelStatus == SYS_RUN)
        pChannel->SHandwheelSim.cBoundaryDirection = cBoundary;
    pChannel->SRun.dCurrentSpeed = 0.0;
    return 1;
}
#endif

//struct SG0 g_SG0;
//struct SSingleAxis g_SSingleAxisArray[AXIS_NUM_MAX];

/*************************************************
  函数名称：getSingleAxisEndPoint(uint8 ucAxis)
  函数功能  获取单轴的终点坐标
  输入参数：pSG0	G0结构体指针
  输出参数：
  返 回 值：
  备    注：运行前 通道必须已经初始化
**************************************************/
int8 SG0LoadSysPara(struct SG0* pSG0)
{
    int i;
    struct SSingleAxis* pSSingleAxisArray; //单轴结构体数组指针
    const uint8* ucMapArray;               //映射
    if (pSG0 == NULL)
    {
        return -1;
    }

    pSSingleAxisArray = pSG0->pSChannel->SSingleAxisArray; //单轴结构体数组指针
    ucMapArray = pSG0->pSChannel->pMapAxisArray;           //映射

    pSG0->pSSingleAxisArray = pSSingleAxisArray; //连接 单轴结构体
    pSG0->ucFinishedSigned = 0;                  //单轴完成标志

    for (i = 0; i < AXIS_NUM_MAX; i++)
    {
        pSSingleAxisArray[i].ucAxis = i;                 //赋值轴号
        pSG0->ucFinishedSigned += 1 << i;                //生成G0完成标志
        pSSingleAxisArray[i].ucInpStatus = INP_FINISHED; //状态置为完成
    }

    pSG0->ucCurrentSigned = pSG0->ucFinishedSigned; //当前标志置为完成

    for (i = 0; i < AXIS_NUM_MAX; i++) //初始化进给倍率 插补周期  单轴的脉冲当量
    {
        pSSingleAxisArray[i].dFeedRatio = 1.0;                                 //进给倍率
        pSSingleAxisArray[i].SPlanData.dTc = g_SSystemPara.dInterpolationTime; //插补周期
        if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX)
            continue; //空轴时，不进行初始化脉冲当量
        pSSingleAxisArray[i].dPulseToMm = g_SSystemPara.SAxisPara[ucMapArray[i]].dPulseToMm; //获取脉冲当量
    }
    return 1;
}

/*************************************************
  函数名称：getSingleAxisEndPoint(uint8 ucAxis)
  函数功能  获取单轴的终点坐标
  输入参数：ucAxis	轴号
  输出参数：
  返 回 值：相应轴的终点坐标
  备    注：
**************************************************/
double getSingleAxisEndPoint(struct SChannel* pSChannel, uint8 ucAxis)
{
    double dOutput;

    if (VAL_VALID == getbit(pSChannel->SG00.ucCurrentSigned, ucAxis)) //当前轴不处于运动状态（当前标记为1）
    {
        getSingleAxisRealTimePoint(&dOutput, ucAxis, pSChannel->pMapAxisArray);
        return dOutput;
    }
    else //处于运动状态
    {
        return pSChannel->SG00.pSSingleAxisArray[ucAxis].dEndPoint;
    }
}

/*************************************************
  函数名称：setFinishedStatus(struct SG0* pSG0Pointer, int nAxisNum)
  函数功能  设置G0位完成状态
  输入参数：pSG0Pointer	G0结构体指针
			nAxisNum	轴的数量
  输出参数：
  返 回 值：-1 空指针
			-2 数量超过总轴数
			1  完成
  备    注：
**************************************************/
int8 setFinishedStatus(struct SG0* pSG0Pointer, int nAxisNum)
{
    int i = 0;
    if (pSG0Pointer == NULL)
    {
        return -1;
    }
    if (nAxisNum > AXIS_NUM_MAX)
    {
        return -2;
    }

    //标志位
    pSG0Pointer->ucCurrentSigned = pSG0Pointer->ucFinishedSigned;
    //单轴状态
    for (i = 0; i < nAxisNum; i++)
    {
        if (pSG0Pointer->pSSingleAxisArray[i].ucRunMode == SINGLEAXIS_MODE)
        {
            pSG0Pointer->pSSingleAxisArray[i].ucInpStatus = INP_FINISHED;
        }
    }

    return 1;
}

/*************************************************
  函数名称：prepareToPauseSingle(struct SSingleAxis* pSingleAxis, double dVmin, double dDec)
  函数功能  根据插补预处理结构体 以减速模型来初始化 原插补预处理结构体，确定距离和终点速度，方便进行速度规划
  输入参数：pSingleAxis			SSingleAxis结构体指针，当前插补的结构体
			dVmin				减速停的目标速度
			dDec				减速度
  输出参数：
  返 回 值：-1 输入错误
			1 初始化完成
  备    注：
**************************************************/
int8 prepareToPauseSingle(struct SSingleAxis* pSSingleAxis, double dVmin, double dDec)
{
    double dVs;       //起始速度 （最大速度  纯减速）
    double dLeftDist; //剩余距离
    double dSde;      //从最大速度减速到起跳速度所需距离
    double dVe;       //最终速度
    double dEndPoint; //插补终点

    struct SToolCompensation* pSToolCompPointer = NULL;

    if (pSSingleAxis == NULL) //空指针不运行
    {
        return -1;
    }
    if (dDec <= 0)
    {
        parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
        return -1;
    }

    pSSingleAxis->ucInpStatus = INP_INITIATION;       //改变插补预处理结构体的状态
    dEndPoint = pSSingleAxis->dEndPoint;              //终点信息
    dLeftDist = pSSingleAxis->SPlanData.dCurLeftDist; //剩余距离

    //起始速度
    dVs = pSSingleAxis->dCurrentSpeed;

    if (dVs <= dVmin) //当前速度小于最小速度 可以直接停
    {
        dSde = 0; //S4型
        pSSingleAxis->ucInpStatus = INP_FINISHED;
    }
    else //不可以直接停
    {
        //计算减速距离
        dSde = fabs(3 * (dVs * dVs - dVmin * dVmin) / (4 * dDec)); //S4型
    }

    if (pSSingleAxis->SPlanData.cInterpDir != INP_GO_BACK) // 正向
    {
        //确定速度规划的距离和终点速度
        if (isZero(dSde - dLeftDist)) //刚好停下
        {
            dVe = dVmin; //速度规划末速度 等于 系统起跳速度
        }
        else if (dSde < dLeftDist) //距离足够减速
        {
            dVe = dVmin; //速度规划末速度 等于 系统起跳速度

            //计算终点位置 //2025/01/02 洪冠鹏 pSSingleAxis->SPlanData.dDistance - dLeftDist + dSde -> pSSingleAxis->SPlanData.dTargetDist - dLeftDist + dSde
            dEndPoint = pSSingleAxis->dStartPoint +
                        pSSingleAxis->dStartVector * (pSSingleAxis->SPlanData.dTargetDist - dLeftDist + dSde);
            pSSingleAxis->dEndPoint = dEndPoint; //终点改变

            //新的剩余距离
            pSSingleAxis->SPlanData.dCurLeftDist = dSde; //剩余距离 = 减速所需的长度
            //pSSingleAxis->SPlanData.dDistance -= dLeftDist - dSde;//当前线段长度： 减去暂停后剩下的距离
            pSSingleAxis->SPlanData.dTargetDist -= dLeftDist - dSde; //目标距离
        }
        else //当前段不能停下
        {
            dVe = sqrt(dVs * dVs - 4 * dDec * dLeftDist / 3); //最终速度不能减到起跳速度  重算实际最终速度
        }
    }
    else //INP_GO_BACK
    {
        double dWriteBackDist = pSSingleAxis->SPlanData.dTargetDist - dLeftDist - dSde;

        if (isEqualOrGreaterThanZero(dWriteBackDist)) //足够距离减速
        {
            dVe = dVmin; //速度规划末速度 等于 系统起跳速度
            //计算新的起点
            //
            pSSingleAxis->dStartPoint += dWriteBackDist;

            //新的剩余距离
            //pSSingleAxis->SPlanData.dCurLeftDist -= dWriteBackDist;//剩余距离 = 减速所需的长度
            //pSSingleAxis->SPlanData.dDistance -= dLeftDist - dSde;//当前线段长度： 减去暂停后剩下的距离
            pSSingleAxis->SPlanData.dTargetDist -= dWriteBackDist; //目标距离
            pSSingleAxis->SPlanData.dPreDist -= dWriteBackDist;
            pSSingleAxis->SPlanData.dDistance -= dWriteBackDist;
        }
        else // 不足距离减速
        {
            dVe = sqrt(dVs * dVs - 4 * dDec * (dWriteBackDist + dSde) / 3); //最终速度不能减到起跳速度  重算实际最终速度
        }
    }

    //暂停插补预处理结构体赋值
    pSSingleAxis->SPlanData.dVs = pSSingleAxis->SPlanData.dVmax = dVs; //起始速度 （最大速度）
    pSSingleAxis->SPlanData.dVe = dVe;                                 //最终速度
    pSSingleAxis->SPlanData.dDec = dDec;                               //减速度
    pSSingleAxis->SPlanData.ucAccVelPlanType = VEL_PLAN_S4_CURVE;      //速度曲线类型   1：梯形，2：3次S型，3：4次S型
    pSSingleAxis->SPlanData.ucDecVelPlanType = VEL_PLAN_S4_CURVE;      //速度曲线类型   1：梯形，2：3次S型，3：4次S型
    pSSingleAxis->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;  //平摊曲线类型，1：梯形，2：5次，3：比例系数
    pSSingleAxis->SPlanData.unCurrentT = 0;                            //时间

    return 1; //预处理完成 返回1
}

/*************************************************
  函数名称：quickMoveWriteBack()
  函数功能  将快速运动暂停后，剩余的距离写回刀补缓冲区
  输入参数：
  输出参数：
  返 回 值：0 距离为0 不回写
			1 回写成功
  备    注：
****************************************/
int8 quickMoveWriteBack(struct SG0* pSG0)
{
    SPointDouble SStartPoint;                      //起点坐标
    SPointDouble RelaDist;                         // 相对位移
    double dDistance;                              //距离
    struct SToolCompensation* pSToolCompPointer;   //刀补缓冲区指针
    struct SInterpolationPre* pPreProcessPointer;  //预处理指针
    bool bRunOverFlag;                             //G0是否执行完成
    memset(&SStartPoint, 0, sizeof(SPointDouble)); //初始化起点坐标
    memset(&RelaDist, 0, sizeof(SPointDouble));    //初始化相对位移
    if (pSG0 == NULL)
    {
        return -1;
    }
    pPreProcessPointer = &pSG0->pSChannel->SPreProcess; //预处理结构体
    //获取实时坐标作为起点
    if (getRealTimePoint((double*)&SStartPoint, AXIS_NUM_MAX, pSG0->pSChannel->pMapAxisArray, NULL) != 1)
    {
        return -1;
    }

    //实时坐标和终点坐标相减
    subtractPoint((double*)&pSG0->SEndPoint, (double*)&SStartPoint, (double*)&RelaDist, AXIS_NUM_MAX,
                  &pSG0->ucValStatus);

    // G0是否执行完  等价于 当前点和终点是否重合
    bRunOverFlag = isZeroPoint((double*)&RelaDist, AXIS_NUM_MAX, &pSG0->ucValStatus);

    if (bRunOverFlag == FALSE ||
        pSG0->pSChannel->SPreProcess.sChannelCommand.ucType != UNASSIGNED_VALUE) //没有到终点 或者通道指令还未执行完
    {
        //写回刀补缓冲区
        //移回前瞻指针
        moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_BACK, pSG0->pSChannel->ucChannelNum);
        //移回插补指针
        moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pSG0->pSChannel->ucChannelNum);

        //获取插补读指针
        pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSG0->pSChannel->ucChannelNum);

        pSToolCompPointer->dPreDist = 1; //写入不为0的数值  表示已经执行过
        pSToolCompPointer->bHasVeerFlag = FALSE;
        pSToolCompPointer->bLookAheadVParaCalcOverFlag = FALSE;
        pSG0->pSChannel->bLookAheadNewBeginFlag = TRUE;     //初始化前瞻开始标志位
        initLookAheadDealed(pSG0->pSChannel->ucChannelNum); //初始化前瞻指针，修改2025.12.6

        //将数据回写到刀补缓冲区
        //pSToolCompPointer->SStartPoint = SStartPoint;
        //pSToolCompPointer->SEndPoint = pSG0->SEndPoint;
        //pSToolCompPointer->ucRelaVal = 0x0;

        if (bRunOverFlag == TRUE) //指令执行完 将执行后作用IO
        {
            //删除缓冲区的所有IO
            clearIOCommand(pSToolCompPointer->ucIOCommandArray, IO_ALL_ACTION, IO_MAXGROUP_IN_SINGLE_LINE);
        }
        else
        {
            //删除当前段的后作用IO
            clearIOCommand(pPreProcessPointer->ucIOCommandArray, IO_POST_ACTION, IO_MAXGROUP_IN_SINGLE_LINE);

            //删除缓存区的前作用IO
            clearIOCommand(pSToolCompPointer->ucIOCommandArray, IO_PRE_ACTION, IO_MAXGROUP_IN_SINGLE_LINE);
        }

        //写回通道指令
        pSToolCompPointer->sChannelCommand = pPreProcessPointer->sChannelCommand;
        pPreProcessPointer->sChannelCommand.ucType = UNASSIGNED_VALUE;

        //防止当前段的M30/M02，导致程序提前停止
        setBits(pPreProcessPointer->unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
        return 2;
    }
    else //到达终点且无通道指令
    {
        pSG0->pSChannel->bLookAheadNewBeginFlag = TRUE;     //初始化前瞻开始标志位
        initLookAheadDealed(pSG0->pSChannel->ucChannelNum); //初始化前瞻指针，修改2025.12.6
        //		clearIOCommand(pPreProcessPointer->ucIOCommandArray, IO_ALL_ACTION, IO_MAXGROUP_IN_SINGLE_LINE); //在手轮模拟且为M30指令时，M30被清楚导致一直卡住2026.6.5
        //		setBits(pPreProcessPointer->unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
        return 1;
    }
}

///*************************************************
//  函数名称：singleAxisPre(struct SG0 * pSG0Pointer,uint8 ucAxis,SPointDouble SStartPoint, double dDistance, struct SInput* pSInputPointer)
//  函数功能  单轴初始化  移动指定的距离
//  输入参数：pSG0Pointer		G0结构体指针
//			ucAxis			轴号
//			SStartPoint		起点
//			dDistance		距离
//			pSInputPointer	运动参数（可以为空   传入空指针则读取系统参数）
//  输出参数：
//  返 回 值：-1	空指针
//			-2	物理轴为空
//			0	距离为0
//			1	初始化完成
//  备    注：
//**************************************************/
//int8 singleAxisPre(struct SG0* pSG0Pointer, uint8 ucAxis, SPointDouble SStartPoint, double dDistance, struct SInput* pSInputPointer)
//{
//	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
//	bool bAssignBySys = TRUE; //是否根据系统参数初始化G0
//	const uint8* ucMapArray;//映射
//	int8* pcInterpDir = &pSG0Pointer->cInterpDir;
//
//	if (pSG0Pointer == NULL)  //传入结构体为空
//	{
//		return -1;
//	}
//
//	ucMapArray = pSG0Pointer->pSChannel->pMapAxisArray;//映射
//	pSSingleAxisPointer = &pSG0Pointer->pSSingleAxisArray[ucAxis];//获取单轴结构体
//
//	if (ucMapArray[ucAxis] == AXIS_NULL) // 物理轴指向为空
//	{
//		return -2;
//	}
//
//	//如果输入参数为空 则根据系统参数赋值
//	if (pSInputPointer != NULL)
//	{
//		pcInterpDir = &pSInputPointer->cInterpDir;
//		bAssignBySys = FALSE;
//	}
//
//	//移动距离为0 或 模式不为单轴模式
//	if (isZero(dDistance) || pSSingleAxisPointer->ucRunMode != SINGLEAXIS_MODE)
//	{
//		return 0;
//	}
//
//	//确定插补方向
//	pSSingleAxisPointer->dStartVector = sgn(dDistance);
//	//获取当前点坐标 作为起点
//	pSSingleAxisPointer->dStartPoint = SStartPoint.dArray[ucAxis];
//	//确定终点坐标
//	pSSingleAxisPointer->dEndPoint = pSSingleAxisPointer->dStartPoint + pSSingleAxisPointer->dStartVector * dDistance;
//	//当前速度为0
//	pSSingleAxisPointer->dCurrentSpeed = 0;
//	//插补方向
//	pSSingleAxisPointer->SPlanData.cInterpDir = *pcInterpDir;
//
//	if (pSSingleAxisPointer->SPlanData.cInterpDir != INP_GO_BACK)//正走
//	{
//		pSSingleAxisPointer->SPlanData.dDistance = pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SPlanData.dTargetDist = fabs(dDistance);//剩余距离、目标距离
//		pSSingleAxisPointer->SPlanData.dPreDist = 0;//规划前已走距离
//	}
//	else//反走
//	{
//		pSSingleAxisPointer->SPlanData.dCurLeftDist = 0;	//剩余距离
//		//规划前已走距离、目标距离
//		pSSingleAxisPointer->SPlanData.dDistance = pSSingleAxisPointer->SPlanData.dPreDist = pSSingleAxisPointer->SPlanData.dTargetDist = fabs(dDistance);
//	}
//
//	//平摊曲线类型，1：梯形，2：5次，3：比例系数
//	pSSingleAxisPointer->SPlanData.ucDistReAverageType = g_SSystemPara.ucDistReAverageType;
//	//插补时间
//	pSSingleAxisPointer->SPlanData.dTc = g_SSystemPara.dInterpolationTime;
//
//	if (bAssignBySys)//判断是否使用系统参数
//	{
//		//速度曲线类型，1：梯形，2：3次S型，3：4次S型
//		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
//		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
//
//		//加、减速度
//		pSSingleAxisPointer->SPlanData.dAcc = getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SUserData.ucAccVelPlanType, ucMapArray);
//		pSSingleAxisPointer->SPlanData.dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SUserData.ucDecVelPlanType, ucMapArray);
//
//		//初速度 末速度
//		pSSingleAxisPointer->SPlanData.dVs = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVs;
//		pSSingleAxisPointer->SPlanData.dPrimalVe = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe;
//
//		//倍率负数为无效值 ，运行过程中从SRun结构体获取倍率
//		pSSingleAxisPointer->dFeedRatio = -1;
//		//进给速度为单轴最大速度
//		pSSingleAxisPointer->dFeedrate = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax;
//		//标志位：使用SRun的倍率
//		pSSingleAxisPointer->bUseSRunFeedRatio = TRUE;
//	}
//	else
//	{
//		//加、减速类型 1：梯形，2：3次S型，3：4次S型
//		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = pSInputPointer->ucAccVelPlanType;
//		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = pSInputPointer->ucDecVelPlanType;
//
//		//加、减速度
//		pSSingleAxisPointer->SPlanData.dAcc = pSInputPointer->dAcc;
//		pSSingleAxisPointer->SPlanData.dDec = pSInputPointer->dDec;
//
//		//初、末速度
//		pSSingleAxisPointer->SPlanData.dVs = pSInputPointer->dVs;
//		pSSingleAxisPointer->SPlanData.dPrimalVe = pSInputPointer->dPrimalVe;
//
//		pSSingleAxisPointer->dFeedRatio = pSInputPointer->dFeedRatio;//倍率
//		pSSingleAxisPointer->dFeedrate = pSInputPointer->dFeedrate;//进给速度
//
//		//倍率为负或者0  使用SRun的倍率
//		pSSingleAxisPointer->bUseSRunFeedRatio = (pSSingleAxisPointer->dFeedRatio < 0 || isZero(pSSingleAxisPointer->dFeedRatio)) ? TRUE : FALSE;
//	}
//
//	//状态置为赋值开始
//	pSSingleAxisPointer->ucInpStatus = INP_ASSIGNMENT_START;
//	//当前标志位置0  表示该轴运动未完成
//	clrbit(pSG0Pointer->ucCurrentSigned, ucAxis);
//
//	return 1;
//}

/*************************************************
  函数名称：singleAxisPre(struct SG0 * pSG0Pointer,uint8 ucAxis,SPointDouble SStartPoint, double dDistance, struct SInput* pSInputPointer)
  函数功能  单轴初始化  移动指定的距离
  输入参数：pSG0Pointer		G0结构体指针
			ucAxis			轴号
			SStartPoint		起点
			dDistance		距离
			pSInputPointer	运动参数（可以为空   传入空指针则读取系统参数）
  输出参数：
  返 回 值：-1	空指针
			-2	物理轴为空
			0	距离为0
			1	初始化完成
  备    注：
**************************************************/
int8 singleAxisPre(struct SG0* pSG0Pointer, uint8 ucAxis, SPointDouble SStartPoint, SPointDouble SEndPoint,
                   bool bReCalculate, struct SInput* pSInputPointer)
{
    struct SSingleAxis* pSSingleAxisPointer; //单轴结构体指针
    bool bAssignBySys = TRUE;                //是否根据系统参数初始化G0
    const uint8* ucMapArray;                 //映射
    double dCurrentPoint;
    double dDistance;
    int8* pcInterpDir = &pSG0Pointer->cInterpDir;

    if (pSG0Pointer == NULL) //传入结构体为空
    {
        return -1;
    }

    ucMapArray = pSG0Pointer->pSChannel->pMapAxisArray;            //映射
    pSSingleAxisPointer = &pSG0Pointer->pSSingleAxisArray[ucAxis]; //获取单轴结构体

    if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX) // 物理轴指向为空
    {
        return -2;
    }

    //获取当前点坐标
    getSingleAxisRealTimePoint(&dCurrentPoint, ucAxis, ucMapArray);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (pSG0Pointer->pSChannel->bUseVirtualY == TRUE &&
        g_RunMode != RE0_MODE) //未开启虚拟Y轴及回原点模式下，以实际机床坐标系下的实时点作为起点（不需换算）
    { //起点为倾斜坐标系下物理轴的实时点，终点为直角坐标系下缓冲区的值，因此要将起点换算至直角坐标系。
        if (ucAxis == AXIS_X)
        {
            double dY1;
            getSingleAxisRealTimePoint(&dY1, AXIS_Y, ucMapArray); //Y1轴的机床坐标（倾斜坐标系）
            SStartPoint.dArray[AXIS_X] = dCurrentPoint =
                dCurrentPoint - 1.0 * dY1 * pSG0Pointer->pSChannel->dSINAngleYY1;
        }
        if (ucAxis == AXIS_Y)
        {
            SStartPoint.dArray[AXIS_Y] = dCurrentPoint = dCurrentPoint * pSG0Pointer->pSChannel->dCOSAngleYY1;
        }
    }
#endif

    if (bReCalculate) //重算起点
    {
        SStartPoint.dArray[ucAxis] = dCurrentPoint;
    }

    //如果输入参数为空 则根据系统参数赋值
    if (pSInputPointer != NULL)
    {
        pcInterpDir = &pSInputPointer->cInterpDir;
        bAssignBySys = FALSE;
    }

    if (!bReCalculate) // 计算当前点是否在起点和终点组成的线段上
    {
        double dEndVector = SEndPoint.dArray[ucAxis] - SStartPoint.dArray[ucAxis] //终点向量
            ,
               dCurrVector = dCurrentPoint - SStartPoint.dArray[ucAxis]; //当前点向量
        if (isUnEqualAndLessThanZero(dEndVector * dCurrVector) ||
            isUnEqualAndLessThanZero(fabs(dEndVector + sgn(dEndVector) * pSSingleAxisPointer->dPulseToMm) -
                                     fabs(dCurrVector))) //反向？  或当前点与起点的距离超过终点与起点距离
        {
            SStartPoint.dArray[ucAxis] = dCurrentPoint;
        }
    }

    dDistance = SEndPoint.dArray[ucAxis] - SStartPoint.dArray[ucAxis]; //移动距离
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    if (isHandwheelDistanceControlActive(pSG0Pointer->pSChannel) && !pSG0Pointer->bIsUserData && isZero(dDistance))
    {
        /* 与下方 sgn 使用相同的数值容差，避免累计坐标的浮点残差被当成运动，
		 * 却得到零方向而报警。只清旧计划，不修改实时坐标，不使用单脉冲死区。 */
        clearSPlanData(&pSSingleAxisPointer->SPlanData);
        pSSingleAxisPointer->dCurrentSpeed = 0.0;
        pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
        setbit(pSG0Pointer->ucCurrentSigned, ucAxis);
        return 0;
    }
#endif

    //移动距离为0 或 模式不为单轴模式
    //移动距离为0 或 模式不为单轴模式
    //if ((dDistance >= -pSSingleAxisPointer->dPulseToMm) && (dDistance <= pSSingleAxisPointer->dPulseToMm) || pSSingleAxisPointer->ucRunMode != SINGLEAXIS_MODE)//2024/12/17注释
    if ((dDistance >= -pSSingleAxisPointer->dPulseToMm) && (dDistance <= pSSingleAxisPointer->dPulseToMm)
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        && !(isHandwheelDistanceControlActive(pSG0Pointer->pSChannel) && !pSG0Pointer->bIsUserData)
#endif
    )
    {
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
        if (pSG0Pointer->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE && ucAxis == AXIS_X &&
            VAL_VALID == (getbit(pSG0Pointer->ucValStatus,
                                 AXIS_Y))) //未有X指令（起终点距离为0），但有Y指令，X1轴需要参与合成Y指令，不能退出
        {
        }
        else
        {
            return 0;
        }
#else
        return 0;
#endif
    }
    //确定插补方向
    pSSingleAxisPointer->dStartVector = sgn(dDistance);
    //获取当前点坐标 作为起点
    pSSingleAxisPointer->dStartPoint = SStartPoint.dArray[ucAxis];
    //确定终点坐标
    pSSingleAxisPointer->dEndPoint =
        SEndPoint.dArray[ucAxis]; //pSSingleAxisPointer->dStartPoint + pSSingleAxisPointer->dStartVector * dDistance;
    //当前速度为0
    //pSSingleAxisPointer->dCurrentSpeed = 0; //2024/5/27 洪冠鹏 单轴运动的G0加入前瞻后，单轴的当前速度由SRun决定
    //插补方向
    pSSingleAxisPointer->SPlanData.cInterpDir = *pcInterpDir; //有SInput时，根据SInput赋方向 否则赋G0结构体的方向

    //if (pSSingleAxisPointer->SPlanData.cInterpDir != INP_GO_BACK)//正走
    //{
    pSSingleAxisPointer->SPlanData.dCurLeftDist = fabs(SEndPoint.dArray[ucAxis] - dCurrentPoint);            //剩余距离
    pSSingleAxisPointer->SPlanData.dDistance = pSSingleAxisPointer->SPlanData.dTargetDist = fabs(dDistance); //目标距离
    pSSingleAxisPointer->SPlanData.dPreDist = fabs(dCurrentPoint - SStartPoint.dArray[ucAxis]); //规划前已走距离
                                                                                                //}
                                                                                                //else//反走
                                                                                                //{
    //	pSSingleAxisPointer->SPlanData.dCurLeftDist = 0;	//剩余距离
    //	//规划前已走距离、目标距离
    //	pSSingleAxisPointer->SPlanData.dDistance = pSSingleAxisPointer->SPlanData.dPreDist = pSSingleAxisPointer->SPlanData.dTargetDist = fabs(dDistance);
    //}
    pSSingleAxisPointer->SPlanData.ucTconstForTrapezoid =
        g_SSystemPara
            .ucTconstForTrapezoid; //三角形速度曲线下指定匀速时间占总周期百分比 （%） （测试用） 洪冠鹏 2024/3/22
                                   //平摊曲线类型，1：梯形，2：5次，3：比例系数
    pSSingleAxisPointer->SPlanData.ucDistReAverageType = g_SSystemPara.ucDistReAverageType;
    //插补时间
    pSSingleAxisPointer->SPlanData.dTc = g_SSystemPara.dInterpolationTime;

    if (bAssignBySys) //判断是否使用系统参数
    {
        pSSingleAxisPointer->dCurrentSpeed =
            pSG0Pointer->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL
                ? 0
                : pSG0Pointer->pSChannel->SRun.dCurrentSpeed * pSG0Pointer->SUserData.SStartVector.dArray[ucAxis];
        //速度曲线类型，1：梯形，2：3次S型，3：4次S型
        pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
        pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;

        //加、减速度
        pSSingleAxisPointer->SPlanData.dAcc = pSSingleAxisPointer->dAcc =
            getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
        pSSingleAxisPointer->SPlanData.dDec = pSSingleAxisPointer->dDec =
            getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);

        //初速度 末速度 //2024/5/27 洪冠鹏 需要前瞻的G0：初末速度来源缓冲区；不需前瞻的G0：初末速度来源系统参数
        pSSingleAxisPointer->SPlanData.dVs =
            pSG0Pointer->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL
                ? g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVs
                : pSG0Pointer->pSChannel->SPreProcess.SUserData.dVs;
        pSSingleAxisPointer->SPlanData.dPrimalVe =
            pSG0Pointer->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL
                ? g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe
                : pSG0Pointer->pSChannel->SPreProcess.SUserData.dPrimalVe;

        //倍率负数为无效值 ，运行过程中从SRun结构体获取倍率
        pSSingleAxisPointer->dFeedRatio = -1;
        //进给速度为单轴最大速度
        pSSingleAxisPointer->dFeedrate = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax;
        if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS)
        { //旋转轴时  不超过旋转轴的最大速度 2023.3.28朱稳中
            pSSingleAxisPointer->dFeedrate =
                min(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax, g_SSystemPara.dRotationAxisG0SpeedMax);
        }
        //标志位：使用SRun的倍率
        pSSingleAxisPointer->bUseSRunFeedRatio = TRUE;
    }

    else //2025.4.30 熊骏
    {
        //加、减速类型 1：梯形，2：3次S型，3：4次S型
        pSSingleAxisPointer->SPlanData.ucAccVelPlanType = pSSingleAxisPointer->SUserData.ucAccVelPlanType;
        pSSingleAxisPointer->SPlanData.ucDecVelPlanType = pSSingleAxisPointer->SUserData.ucDecVelPlanType;

        //加、减速度
        pSSingleAxisPointer->SPlanData.dAcc = pSSingleAxisPointer->dAcc = pSSingleAxisPointer->SUserData.dAcc;
        pSSingleAxisPointer->SPlanData.dDec = pSSingleAxisPointer->dDec = pSSingleAxisPointer->SUserData.dDec;

        //初、末速度
        pSSingleAxisPointer->SPlanData.dVs = pSSingleAxisPointer->SUserData.dVs;
        pSSingleAxisPointer->SPlanData.dPrimalVe = pSSingleAxisPointer->SUserData.dPrimalVe;

        pSSingleAxisPointer->dFeedRatio = pSSingleAxisPointer->SUserData.dFeedRatio; //倍率
        pSSingleAxisPointer->dFeedrate = pSSingleAxisPointer->SUserData.dFeedrate;   //速度;

        if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS)
        {
            pSSingleAxisPointer->dFeedrate =
                min(pSSingleAxisPointer->SUserData.dFeedrate, g_SSystemPara.dRotationAxisG0SpeedMax);
        }
        //倍率为负或者0  使用SRun的倍率
        pSSingleAxisPointer->bUseSRunFeedRatio =
            (pSSingleAxisPointer->dFeedRatio < 0 || isZero(pSSingleAxisPointer->dFeedRatio)) ? TRUE : FALSE;
    }
    if (isZero(pSSingleAxisPointer->SPlanData.dAcc) ||
        isZero(pSSingleAxisPointer->SPlanData.dDec)) //参数检测，避免一些参数出现为0的情况
    {
        parseRunningError(pSG0Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
        return -1;
    }
    //状态置为赋值开始
    pSSingleAxisPointer->ucInpStatus = INP_ASSIGNMENT_START;
    //当前标志位置0  表示该轴运动未完成
    clrbit(pSG0Pointer->ucCurrentSigned, ucAxis);

    return 1;
}

/*************************************************
  函数名称：singleAxisPreLoop(struct SG0 *pSG0Pointer,SPointDouble SStartPoint, SPointDouble SEndPoint)
  函数功能  G0赋值函数  根据起点和终点赋值初始化单轴结构体
  输入参数：pSG0Pointer	G0指针
			SStartPoint 起点
			SEndPoint	终点
  输出参数：
  返 回 值：-1空指针
			1 赋值完成
  备    注：
**************************************************/
int8 singleAxisPreLoop(struct SG0* pSG0Pointer, SPointDouble SStartPoint, SPointDouble SEndPoint,
                       struct SInput* pSInputPointer, VAL_STRUCT_TYPE* ucValStatus, bool bReCalculate)
{
    int i = 0;
    const uint8* ucMapArray; //映射
    int nRes = 0;
    if (pSG0Pointer == NULL)
        return -1;

    ucMapArray = pSG0Pointer->pSChannel->pMapAxisArray; //从通道获取映射

    if (ucValStatus == NULL) // 空指针  全部初始化
    {
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
        for (i = AXIS_NUM_MAX - 1; i >= 0; i--)
#else
        for (i = 0; i < AXIS_NUM_MAX; i++)
#endif
        {
            //单轴初始化
            nRes = singleAxisPre(pSG0Pointer, i, SStartPoint, SEndPoint, bReCalculate, pSInputPointer);
        }
    }
    else //非空 根据ucValStatus初始化
    {
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
        for (i = AXIS_NUM_MAX - 1; i >= 0; i--)
#else
        for (i = 0; i < AXIS_NUM_MAX; i++)
#endif
        {
            if (VAL_VALID != getbit(*ucValStatus, i))
                continue; //无效轴 跳过
            //单轴初始化
            nRes = singleAxisPre(pSG0Pointer, i, SStartPoint, SEndPoint, bReCalculate, pSInputPointer);
            if (nRes == -1 || nRes == -2)
            {
                parseRunningError(pSG0Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
                clrbit(pSG0Pointer->ucCurrentSigned, i);
                return nRes;
            }
        }
    }
    return 1;
}

/*************************************************
  函数名称：loadSG0DataFromInput(struct SG0* pSG0Pointer, struct SInput* pSInput, bool bReCalculate)
  函数功能  从Input参数赋值到G0/单轴结构体
  输入参数：pSG0Pointer	结构体SG1指针
			pSInput			SInput结构体指针
			bReCalculate	是否重算起点和向量
  输出参数：
  返 回 值：-1	空指针
			1	处理完成
  备    注：
**************************************************/
int8 loadSG0DataFromInput(struct SG0* pSG0Pointer, struct SInput* pSInput)
{
    SPointDouble SRelaDist;
    int i;
    int nRes = 0;
    bool bReCalculate = TRUE;
    if (pSG0Pointer == NULL || pSInput == NULL)
        return -1;                                                         // 结构体为空
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (pSG0Pointer->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE &&
        VAL_VALID == getbit(pSInput->ucValStatus, AXIS_Y) &&
        VAL_VALID != getbit(pSInput->ucValStatus, AXIS_X)) //只有Y轴方向运动时
    {
        setbit(pSInput->ucValStatus, AXIS_X); //设置X轴有效 进入初始化
    }
#endif
    //G0有效轴（不能除去刀补）
    pSG0Pointer->ucValStatus = pSInput->ucValStatus;

    //获取终点
    pSG0Pointer->SEndPoint = pSInput->SEndPoint;
    //获取插补方向
    pSG0Pointer->cInterpDir = pSInput->cInterpDir;
    //获取相对
    //pSG0Pointer->SUserData.ucRelaVal = pSInput->ucRelaVal;   2022.12.12注释

    //获取位移分量 2024/5/27 洪冠鹏
    pSG0Pointer->SUserData.SStartVector = pSInput->SStartVector;

    switch (pSInput->cDealType)
    {
    default:
    case DEALTYPE_RECAL:           //重算  不回写
    case DEALTYPE_RECAL_WRITEBACK: //重算 回写

        //判断相对坐标  修正终点
        if (pSInput->ucRelaVal != VAL_ABS_POINT) //不完全是绝对坐标
        {
            modifyRelaEndPoint((double*)&pSInput->SStartPoint, (double*)&pSG0Pointer->SEndPoint,
                               pSG0Pointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSInput->ucRelaVal);
        }

        //修正旋转轴坐标
        modifyRotationAxisPoint((double*)&pSInput->SStartPoint, (double*)&pSG0Pointer->SEndPoint,
                                (double*)&pSInput->SStartVector, AXIS_NUM_MAX, pSG0Pointer->pSChannel->pMapAxisArray,
                                &pSInput->ucValStatus, pSInput->ucRelaVal);

        if (pSInput->cDealType == DEALTYPE_RECAL_WRITEBACK)
        {
            int8 cIndex = 0;
            struct SToolCompensation* pWriteBackBuff;

            cIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSG0Pointer->pSChannel->ucChannelNum); //获取索引
            pWriteBackBuff =
                getSToolCompPointerByIndex(cIndex - 1, pSG0Pointer->pSChannel->ucChannelNum); //根据索引的前一个进行回写
            pWriteBackBuff->SEndPoint = pSG0Pointer->SEndPoint;                               //绝对坐标终点
            getRealTimePoint((double*)&pWriteBackBuff->SStartPoint, AXIS_NUM_MAX, pSG0Pointer->pSChannel->pMapAxisArray,
                             &pSInput->ucValStatus); //起点
            //pWriteBackBuff->SStartPoint = pSG1Pointer->SStartPoint;	//绝对坐标 起点
            pWriteBackBuff->ucRelaVal = 0x0; // 相对坐标标记置为0
            pWriteBackBuff->dPreDist = 0;    //将缓冲区的已走的距离置为0
        }
        //根据终点初始化单轴结构体
        nRes = singleAxisPreLoop(pSG0Pointer, pSInput->SStartPoint, pSG0Pointer->SEndPoint, NULL, &pSInput->ucValStatus,
                                 bReCalculate);
        break;
    case DEALTYPE_NO_PROCESS: // 不进行处理

        if (!isZero(pSInput->dPreDist) && pSInput->cInterpDir == INP_GO_AHEAD) //已经走距离不为0
        {
            int8 cIndex = 0;
            struct SToolCompensation* pWriteBackBuff;

            cIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSG0Pointer->pSChannel->ucChannelNum); //获取索引
            pWriteBackBuff =
                getSToolCompPointerByIndex(cIndex - 1, pSG0Pointer->pSChannel->ucChannelNum); //根据索引的前一个进行回写
            pWriteBackBuff->dPreDist = 0; //将缓冲区的已走的距离置为0
        }
        bReCalculate = FALSE;
        //根据终点初始化单轴结构体
        nRes = singleAxisPreLoop(pSG0Pointer, pSInput->SStartPoint, pSG0Pointer->SEndPoint, NULL, &pSInput->ucValStatus,
                                 bReCalculate);
        break;
    case DEALTYPE_USER_INPUT: // 用户输入
        nRes = singleAxisPreLoop(pSG0Pointer, pSInput->SStartPoint, pSG0Pointer->SEndPoint, pSInput,
                                 &pSInput->ucValStatus, bReCalculate);
    }

    return nRes;
}

/*************************************************
  函数名称：singleAxisMotion(struct SG0* pSG0, uint8 ucAxis, bool bPauseOn, bool bRatioOn)
  函数功能  单轴插补,运动前先进行初始化，如果运动的过程中倍率改变会重新赋值，但会保留已走过的距离
  输入参数：pSG0			SG0结构体指针
			ucAxis			轴号
			bPauseOn		是否开启暂停
			bRatioOn		是否开启倍率
  输出参数：
  返 回 值：-1 空指针
			-2 未初始化
			-3 无映射 不运行
			0  距离为0
			1  运行完成
  备    注：换向锁仅冻结手轮倍率响应；停稳后采纳最新倍率，暂停和限位仍走原流程。
**************************************************/
int8 singleAxisMotion(struct SG0* pSG0, uint8 ucAxis, bool bPauseOn, bool bRatioOn)
{
    double dDeltaL;
    double dLastAbsCoord;    //上一点坐标
    double dAbsCoord;        //当前坐标
    const uint8* ucMapArray; //映射
    struct SSingleAxis* pSSingleAxisPointer;
    struct _SRun* pSRun;             //运行结构体
    SPointDouble SStartPoint = {0};  //起点坐标 2024/4/11 洪冠鹏
    double dVmaxToLimitiedPoint = 0; //反算的最大速度
    double dSingleAxisMaxDist = 0;   //最大单轴运动距离

    //局部变量保存参数 防止规划失败
    double dVmax, dVs, dVe, dAcc, dDec;

    //空指针
    if (pSG0 == NULL)
    {
        return -1;
    }

    //读取结构体
    pSSingleAxisPointer = &pSG0->pSSingleAxisArray[ucAxis]; //单轴结构体
    ucMapArray = pSG0->pSChannel->pMapAxisArray;            //映射
    pSRun = &pSG0->pSChannel->SRun;                         //运行结构体

    ////如果未初始化 不运行
    //if (getbit(pSG0->ucCurrentSigned, ucAxis) == 1)
    //{
    //	return -2;
    //}
    //朱稳中 2023.6.21
    if (pSSingleAxisPointer->ucRunMode != SINGLEAXIS_MODE &&
        pSSingleAxisPointer->ucRunMode != SINGLEAXIS_LIMIT_POS    //正限位
        && pSSingleAxisPointer->ucRunMode != SINGLEAXIS_LIMIT_NEG //负限位
    )
    {
        return 0;
    }
    //移动距离为0  置为完成
    if (isZero(pSSingleAxisPointer->SPlanData.dDistance) && pSSingleAxisPointer->ucInpStatus == INP_RUNNING
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        && !(isHandwheelDistanceControlActive(pSG0->pSChannel) && !pSG0->bIsUserData)
#endif
    )
    {

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
        if (pSG0->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE && ucAxis == AXIS_X &&
            VAL_VALID == getbit(pSG0->ucValStatus, AXIS_Y)) //只有Y轴运动，X轴不运动
        {
        }
        else
        {
            pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
        }
#else
        pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
#endif
    }

    if (pSSingleAxisPointer->ucInpStatus == INP_FINISHED) //判断状态，FINISHED时退出
    {
        setbit(pSG0->ucCurrentSigned, ucAxis); // 当前轴对应的标志位置为1  表示完成
        return 0;                              //插补已完成
    }

    //没有映射到物理轴  不运行
    if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
    {
        parseRunningError(pSG0->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
        return -3;
    }

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* 在旧倍率和换向规划之前接管；内部手动运动、
	 * 复位及报警减速继续由原路径负责。 */
    if (bPauseOn && bRatioOn && isHandwheelDistanceControlActive(pSG0->pSChannel) && !pSG0->bIsUserData &&
        (pSG0->pSChannel->ucChannelStatus == SYS_RUN || pSG0->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE) &&
        (pSSingleAxisPointer->ucInpStatus == INP_RUNNING || pSSingleAxisPointer->ucInpStatus == INP_PAUSING))
        return reportG0HandwheelDistanceError(pSG0); /* 距离模式只能经全轴入口，禁止单轴提前输出。 */
#endif

    if (bRatioOn && pSSingleAxisPointer->bUseSRunFeedRatio == TRUE) //倍率开启
    {
        //在运行的过程中速率改变
        if (pSSingleAxisPointer->ucInpStatus == INP_RUNNING &&
            pSSingleAxisPointer->dFeedRatio != pSRun->dQuickMoveFeedRatio * pSRun->dHandWheelSpeedRatio)
        {
            //在运行的过程中速率改变	//陈敏聪	2023/12/5	为保证加速度连续性，提高平顺度，G0的速度重新赋值只在匀速段触发
            if ((pSSingleAxisPointer->SPlanData.unCurrentT <= pSSingleAxisPointer->SPlanData.unT3 &&
                 pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT2) ||
                pSSingleAxisPointer->SPlanData.unCurrentT > pSSingleAxisPointer->SPlanData.unT5)
            {
                pSSingleAxisPointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值
            }
        }
    }
    switch (pSSingleAxisPointer->ucInpStatus)
    {

    case INP_FINISHED: //完成  退出
        break;

    case INP_INITIATION: //输入参数 赋值给运动和规划  初始化 2024.12.9熊骏
        pSSingleAxisPointer->SPlanData.cInterpDir = pSSingleAxisPointer->SUserData.cInterpDir; //插补方向
        pSSingleAxisPointer->SPlanData.dAcc = pSSingleAxisPointer->dAcc = pSSingleAxisPointer->SUserData.dAcc; //加速度
        pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SUserData.dCurLeftDist; //剩余距离
        pSSingleAxisPointer->SPlanData.dDec = pSSingleAxisPointer->dDec = pSSingleAxisPointer->SUserData.dDec; //减速度
        pSSingleAxisPointer->SPlanData.dDistance = pSSingleAxisPointer->SUserData.dDistance;                   //总距离
        pSSingleAxisPointer->SPlanData.dPreDist = pSSingleAxisPointer->SUserData.dPreDist;                 //已走距离
        pSSingleAxisPointer->SPlanData.dPrimalVe = pSSingleAxisPointer->SUserData.dPrimalVe;               //末速度
        pSSingleAxisPointer->SPlanData.dTargetDist = pSSingleAxisPointer->SUserData.dTargetDist;           //目标距离
        pSSingleAxisPointer->SPlanData.dVs = pSSingleAxisPointer->SUserData.dVs;                           //初速度
        pSSingleAxisPointer->SPlanData.ucAccVelPlanType = pSSingleAxisPointer->SUserData.ucAccVelPlanType; //加速类型
        pSSingleAxisPointer->SPlanData.ucDecVelPlanType = pSSingleAxisPointer->SUserData.ucDecVelPlanType; //减速类型
        pSSingleAxisPointer->SPlanData.ucDistReAverageType =
            pSSingleAxisPointer->SUserData.ucDistReAverageType; //平摊类型

        pSSingleAxisPointer->dStartPoint = pSSingleAxisPointer->SUserData.SStartPoint.dArray[ucAxis];   //起点
        pSSingleAxisPointer->dStartVector = pSSingleAxisPointer->SUserData.SStartVector.dArray[ucAxis]; //方向
        pSSingleAxisPointer->dEndPoint = pSSingleAxisPointer->SUserData.SEndPoint.dArray[ucAxis];       //终点
        pSSingleAxisPointer->dFeedrate = pSSingleAxisPointer->SUserData.dFeedrate;                      //进给速度
        pSSingleAxisPointer->dFeedRatio = pSSingleAxisPointer->SUserData.dFeedRatio;                    //进给倍率
        pSSingleAxisPointer->bUseSRunFeedRatio =
            (pSSingleAxisPointer->dFeedRatio < 0 || isZero(pSSingleAxisPointer->dFeedRatio))
                ? TRUE
                : FALSE;                                         //倍率为负或者0  使用SRun的倍率
        pSSingleAxisPointer->ucInpStatus = INP_ASSIGNMENT_START; //下一步

    case INP_ASSIGNMENT_START: //速度规划   开始赋值
        //局部变量保存参数 防止规划失败
        dVmax = pSSingleAxisPointer->SPlanData.dVmax;
        dVs = pSSingleAxisPointer->SPlanData.dVs;
        dVe = pSSingleAxisPointer->SPlanData.dVe;
        dAcc = pSSingleAxisPointer->SPlanData.dAcc;
        dDec = pSSingleAxisPointer->SPlanData.dDec;
        pSSingleAxisPointer->SPlanData.dAcc = pSSingleAxisPointer->dAcc;
        pSSingleAxisPointer->SPlanData.dDec = pSSingleAxisPointer->dDec;

        if (bRatioOn)
        {
            //进给速率
            pSSingleAxisPointer->dFeedRatio = (pSSingleAxisPointer->bUseSRunFeedRatio)
                                                  ? pSRun->dQuickMoveFeedRatio * pSRun->dHandWheelSpeedRatio
                                                  : pSSingleAxisPointer->dFeedRatio;
            if (pSSingleAxisPointer->SPlanData.cInterpDir * pSSingleAxisPointer->dFeedRatio > 0) //插补方向和倍率同向
            {
                //最大速度  结合倍率计算
                pSSingleAxisPointer->SPlanData.dVmax =
                    fabs(pSSingleAxisPointer->dFeedRatio) * pSSingleAxisPointer->dFeedrate;
            }
            else //<=0    插补方向和倍率反向
            {
                if (isZero(pSSingleAxisPointer->dCurrentSpeed)) //当前速度为0
                {
                    pSSingleAxisPointer->SPlanData.cInterpDir =
                        (pSSingleAxisPointer->dFeedRatio < 0) ? INP_GO_BACK : INP_GO_AHEAD;
                    //最大速度  结合倍率计算
                    pSSingleAxisPointer->SPlanData.dVmax =
                        fabs(pSSingleAxisPointer->dFeedRatio) * pSSingleAxisPointer->dFeedrate;
                }
                else
                {
                    pSSingleAxisPointer->SPlanData.dVmax = 0;
                }
            }
        }
        else
        {
            //最大速度 （没有开启倍率 直接赋值）
            pSSingleAxisPointer->SPlanData.dVmax = pSSingleAxisPointer->dFeedrate;
        }
        if (pSG0->pSChannel->cHandWheelSimulaStatus != HANWHEELSIMULA_OFF &&
            (g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE)
#if HANDWHEEL_SIM_DISTANCE_ENABLE
            && !isHandwheelAutomaticMoveRequired(pSG0->pSChannel)
#endif
        )
        { //手轮模拟 最大速度限制
            pSSingleAxisPointer->SPlanData.dVmax =
                min(pSSingleAxisPointer->SPlanData.dVmax,
                    fabs(pSSingleAxisPointer->dFeedRatio) * g_SSystemPara.dSimulaG0Vmax); //2025.5.5熊骏
            if (pSG0->pSChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_ON)
            {
                pSSingleAxisPointer->SPlanData.dAcc =
                    min(pSSingleAxisPointer->SPlanData.dAcc, g_SSystemPara.dSimulaMaxAcc);
                pSSingleAxisPointer->SPlanData.dDec =
                    min(pSSingleAxisPointer->SPlanData.dDec, g_SSystemPara.dSimulaMaxDec); //2025.4.19熊骏
            }
        }
        //最大速度不能超过单轴最大速度
        pSSingleAxisPointer->SPlanData.dVmax =
            min(pSSingleAxisPointer->SPlanData.dVmax,
                g_SSystemPara.SAxisPara[ucMapArray[pSSingleAxisPointer->ucAxis]].dVmax);

        //初速度继承当前速度，若为0，则继承系统对应的单轴起跳速度(不超过Vmax)
        pSSingleAxisPointer->SPlanData.dVs =
            (isZero(pSSingleAxisPointer->dCurrentSpeed))
                ? min(g_SSystemPara.SAxisPara[ucMapArray[pSSingleAxisPointer->ucAxis]].dVs,
                      pSSingleAxisPointer->SPlanData.dVmax)
                : pSSingleAxisPointer->dCurrentSpeed;

        //当前线段之前已触发软限位,重设速度规划参数后执行（减速）速度规划
        if (pSRun->cSoftLimitedOn == 3 && pSSingleAxisPointer->ucRunMode != SINGLEAXIS_MODE)
        {
            //重设曲线类型（与前段轨迹保持相同曲线类型）
            pSSingleAxisPointer->SPlanData.dVmax = pSSingleAxisPointer->SPlanData.dVs; //最大速度 = 初速度 = 当前速度
            pSSingleAxisPointer->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;        //加速类型
            pSSingleAxisPointer->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;        //减速类型
            pSSingleAxisPointer->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;   //平摊类型
            pSSingleAxisPointer->SPlanData.dAcc =
                getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray); //加速度
            pSSingleAxisPointer->SPlanData.dDec =
                getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray); //减速度

            if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_POS) //触发正限位
            {
                dSingleAxisMaxDist = g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].pos_soft -
                                     pSSingleAxisPointer->dStartPoint; //计算限位点到起点的最大单轴运动距离
                if (dSingleAxisMaxDist <=
                    pSSingleAxisPointer->SPlanData
                        .dTargetDist) //限位点在当前线段内：1.重设末速度；2.重算减速度；3.重置开关变量。 -> 执行T型减速的速度规划实时判断减速停
                {
                    pSSingleAxisPointer->SPlanData.dVe =
                        g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe; //1.重置末速度 = 起跳速度
                    pSSingleAxisPointer->SPlanData.dDec =
                        (pow(pSSingleAxisPointer->SPlanData.dVs, 2) - pow(pSSingleAxisPointer->SPlanData.dVe, 2)) /
                        (2.0 * pSSingleAxisPointer->SPlanData.dCurLeftDist); //2.重算减速度(T型)
                    pSRun->cSoftLimitedOn = 1;                               //3.重置软限位开关变量
                }
                else //限位点在当前线段外：1.重算末速度 -> 执行T型减速的速度规划
                {
                    pSSingleAxisPointer->SPlanData.dVe = sqrt(
                        pow(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe, 2) +
                        2 * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dSysPlanTMaxDec *
                            fabs(dSingleAxisMaxDist - pSSingleAxisPointer->SPlanData.dTargetDist)); //T型公式重算末速度
                }
            }
            else if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_NEG) //触发负限位
            {
                dSingleAxisMaxDist =
                    pSSingleAxisPointer->dStartPoint -
                    g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].neg_soft; //计算限位点到起点的最大单轴运动距离
                if (dSingleAxisMaxDist <=
                    pSSingleAxisPointer->SPlanData
                        .dTargetDist) //限位点在当前线段内：1.重设末速度；2.重算减速度；3.重置开关变量。 -> 执行T型减速的速度规划实时判断减速停
                {
                    pSSingleAxisPointer->SPlanData.dVe =
                        g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe; //1.重置末速度 = 起跳速度
                    pSSingleAxisPointer->SPlanData.dDec =
                        (pow(pSSingleAxisPointer->SPlanData.dVs, 2) - pow(pSSingleAxisPointer->SPlanData.dVe, 2)) /
                        (2.0 * pSSingleAxisPointer->SPlanData.dCurLeftDist); //2.重算减速度(T型)
                    pSRun->cSoftLimitedOn = 1;                               //3.重置软限位开关变量
                }
                else //限位点在当前线段外：1.重算末速度 -> 执行T型减速的速度规划
                {
                    pSSingleAxisPointer->SPlanData.dVe = sqrt(
                        pow(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe, 2) +
                        2 * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dSysPlanTMaxDec *
                            fabs(dSingleAxisMaxDist - pSSingleAxisPointer->SPlanData.dTargetDist)); //T型公式重算末速度
                }
            }
        }
        else
        { //末速度
            pSSingleAxisPointer->SPlanData.dVe = pSSingleAxisPointer->SPlanData.dPrimalVe;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
            if (!pSG0->bIsUserData && isHandwheelAutomaticMoveRequired(pSG0->pSChannel))
            {
                pSSingleAxisPointer->SPlanData.dVe = pSSingleAxisPointer->SPlanData.dPrimalVe = 0.0;
            }
#endif
        }
#if MACHINE_CONFIG ==                                                                                                  \
    TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机 X轴的速度参数与Y轴的速度参数保持一致，使X1轴与Y1轴同时到达终点，避免X1轴超出终点再跟随返回终点
        if (pSG0->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE && ucAxis == AXIS_X &&
            VAL_VALID != getbit(pSG0->ucCurrentSigned, AXIS_Y)) //Y轴方向有运动
        {
            if ((pSSingleAxisPointer->dStartVector * pSG0->pSChannel->SSingleAxisArray[AXIS_Y].dStartVector) < 0)
            {

                uint32 unTac;
                uint32 unTconst;
                uint32 unTde;

                unTac = pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.unT2;
                unTconst = pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.unT3 -
                           pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.unT2;
                unTde = pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.unT5 -
                        pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.unT3;

                double dDeltaVac;
                double dDeltaVde;
                dDeltaVac = pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.dVmax -
                            pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.dVs;
                dDeltaVde = pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.dVmax -
                            pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.dPrimalVe;

                pSSingleAxisPointer->SPlanData.dVmax =
                    (2.0 * pSSingleAxisPointer->SPlanData.dTargetDist - pSSingleAxisPointer->SPlanData.dVs * unTac -
                     pSSingleAxisPointer->SPlanData.dVe * unTde) /
                    (unTac + 2.0 * unTconst + unTde);
                pSSingleAxisPointer->SPlanData.dAcc =
                    isZero(dDeltaVac) ? 0
                                      : (pSSingleAxisPointer->SPlanData.dVmax - pSSingleAxisPointer->SPlanData.dVs) /
                                            dDeltaVac * pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.dAcc;
                pSSingleAxisPointer->SPlanData.dDec =
                    isZero(dDeltaVde)
                        ? 0
                        : (pSSingleAxisPointer->SPlanData.dVmax - pSSingleAxisPointer->SPlanData.dPrimalVe) /
                              dDeltaVde * pSG0->pSChannel->SSingleAxisArray[AXIS_Y].SPlanData.dDec;
            }
        }
#endif
        if (preparePlanningAndDistReAvg(&pSSingleAxisPointer->SPlanData, TRUE) == -2) //速度规划+平摊
        {                                                                             //规划失败  换回原来的参数
            pSSingleAxisPointer->SPlanData.dVmax = dVmax;
            pSSingleAxisPointer->SPlanData.dVs = dVs;
            pSSingleAxisPointer->SPlanData.dVe = dVe;
            pSSingleAxisPointer->SPlanData.dAcc = dAcc;
            pSSingleAxisPointer->SPlanData.dDec = dDec;
        }
        else
        { //规划成功  时间置0
            pSSingleAxisPointer->SPlanData.unCurrentT = 0;
        }
        //软限位的判断
        getRealTimePoint((double*)&SStartPoint, AXIS_NUM_MAX, pSG0->pSChannel->pMapAxisArray,
                         NULL); //获取实时坐标作为起点

        //修改软限位
        int8 cCurrentSoftLimited; //用于保存当前通道软限位状态
        cCurrentSoftLimited =
            softLimit(pSG0->pSChannel, SStartPoint, pSG0->SEndPoint, pSG0->SEndPoint, pSG0->SEndPoint, ucAxis);
        pSRun->cSoftLimitedOn =
            isZero(cCurrentSoftLimited)
                ? pSRun->cSoftLimitedOn
                : cCurrentSoftLimited; //修改2025.12.10	避免因后续轴没触发软限位导致前轴软限位状态被覆盖，对于G0来说，只有多个轴为1或者1个轴为2（由于速度前瞻才会出现2，而G0只有1个轴才能前瞻）的情况

        //判断后的处理
        switch (pSRun->cSoftLimitedOn)
        {
        case 1: //触发软限位：限位点在当前线段内,当前段能停下 -> 1.重设目标距离及剩余距离；2.重设末速度；3.设置运行状态。
            if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_POS) //当前轴触发正限位
            {
                dSingleAxisMaxDist = g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].pos_soft -
                                     pSSingleAxisPointer->dStartPoint; //计算限位点到起点的最大单轴运动距离
            }
            else if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_NEG) //当前轴触发负限位
            {
                dSingleAxisMaxDist =
                    pSSingleAxisPointer->dStartPoint -
                    g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].neg_soft; //计算限位点到起点的最大单轴运动距离
            }
            else //当前轴未触发限位，但其他轴触发限位
            {
                pSSingleAxisPointer->ucInpStatus = INP_RUNNING; //下一步
                break;
            }
            //1.重设目标距离 = 最大单轴运动距离；剩余距离 = 最大单轴运动距离 - 已走距离
            pSSingleAxisPointer->SPlanData.dTargetDist = fabs(dSingleAxisMaxDist);
            pSSingleAxisPointer->SPlanData.dCurLeftDist =
                fabs(dSingleAxisMaxDist - pSSingleAxisPointer->SPlanData.dPreDist);
            //2.重算的末速度 = 起跳速度
            pSSingleAxisPointer->dReCalculateVe = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe;
            //3.运行状态：开始运动
            pSSingleAxisPointer->ucInpStatus = INP_RUNNING;
            break;
        case 2: //触发软限位：限位点在当前线段外，当前线段不停 -> 1.重算末速度；2.设置运行状态。
            if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_POS) //当前轴触发正限位
            {
                dSingleAxisMaxDist = g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].pos_soft -
                                     pSSingleAxisPointer->dStartPoint; //计算限位点到起点的最大单轴运动距离
            }
            else if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_NEG) //当前轴触发负限位
            {
                dSingleAxisMaxDist =
                    pSSingleAxisPointer->dStartPoint -
                    g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].neg_soft; //计算限位点到起点的最大单轴运动距离
            }
            else //当前轴未触发限位，但其他轴触发限位
            {
                pSSingleAxisPointer->ucInpStatus = INP_RUNNING; //下一步
                break;
            }
            //1.重算末速度
            pSSingleAxisPointer->dReCalculateVe =
                sqrt(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe +
                     2 * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dSysPlanTMaxDec *
                         fabs(dSingleAxisMaxDist - pSSingleAxisPointer->SPlanData.dTargetDist));
            //2.运行状态：开始运动
            pSSingleAxisPointer->ucInpStatus = INP_RUNNING;
            break;
        default:                                            //未触发软限位
            pSSingleAxisPointer->ucInpStatus = INP_RUNNING; //下一步
            break;
        }

    case INP_RUNNING: //执行运动

        if (bPauseOn) //开启暂停处理
        {
            //判断是否为暂停状态
            if (pSG0->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE) //系统TO_PAUSE状态
            {
                if (pSSingleAxisPointer->ucInpStatus != INP_PAUSING &&
                        pSSingleAxisPointer->SPlanData.unCurrentT < pSSingleAxisPointer->SPlanData.unT3 ||
                    pSSingleAxisPointer->SPlanData.unCurrentT >
                        pSSingleAxisPointer->SPlanData.unT5) //插补结构体的状态不处于PAUSING 且不在减速
                {                                            //原运动开始减速后不执行暂停
                    double dVmin = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe; //获取起跳速度 //修改2026.3.10
                    double dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType,
                                                   ucMapArray); //获取最大减速度

                    //插补结构体初始化 为减速模型
                    prepareToPauseSingle(pSSingleAxisPointer, dVmin, dDec);
                    //速度规划+平摊
                    preparePlanningAndDistReAvg(&pSSingleAxisPointer->SPlanData, TRUE);
                    pSSingleAxisPointer->ucInpStatus = INP_PAUSING; //改变状态为正在暂停
                }
            }
        }
    case INP_PAUSING:                                   //暂停
        pSSingleAxisPointer->SPlanData.unCurrentT += 1; //插补的当前时间点

        //获取一个Tc的位置值
        dDeltaL =
            nextstepPlanningAndDistReAvg(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);

        //计算新的当前点坐标
        dAbsCoord = pSSingleAxisPointer->dStartPoint + (pSSingleAxisPointer->dStartVector * dDeltaL);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //插补Y轴机
        if (pSG0->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE)
        {
            //将直角坐标系下的坐标转化为倾斜坐标系下的坐标
            if (ucAxis == AXIS_X)
            {
                double dY1;
                getSingleAxisRealTimePoint(&dY1, AXIS_Y, ucMapArray);
                dAbsCoord = dAbsCoord + 1.0 * dY1 * pSG0->pSChannel->dSINAngleYY1;
            }
            if (ucAxis == AXIS_Y)
            {
                dAbsCoord = dAbsCoord / pSG0->pSChannel->dCOSAngleYY1;
            }
        }
#endif

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
        dAbsCoord =
            outPutAbsPointWithScrewCompForG0(pSRun, dAbsCoord, ucAxis, ucMapArray, pSSingleAxisPointer->dPulseToMm);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
        dAbsCoord = outPutAbsPointWithBilateralScrewCompForG0(pSRun, dAbsCoord, ucAxis, ucMapArray,
                                                              pSSingleAxisPointer->dPulseToMm);
#else
        //获取实时点作为上一点的位置
        getSingleAxisRealTimePoint(&dLastAbsCoord, ucAxis, ucMapArray);
        //各个轴的位移增量，    注意数组的下标减1,单位:pulse(输出指令单位)
        pSRun->sAxisDeltaPulse[ucAxis] = round((dAbsCoord - dLastAbsCoord) / pSSingleAxisPointer->dPulseToMm);

        //修正当前绝对坐标（利用上一点坐标和取整后的脉冲 计算实际坐标）
        dAbsCoord = dLastAbsCoord + (double)pSRun->sAxisDeltaPulse[ucAxis] * pSSingleAxisPointer->dPulseToMm;
#endif
        //计算当前速度 根据距离计算  = (剩余距离(上周期) - 剩余距离(当前周期))/dTc
        pSSingleAxisPointer->dCurrentSpeed =
            fabs(pSSingleAxisPointer->SPlanData.dCurLeftDist - (pSSingleAxisPointer->SPlanData.dTargetDist - dDeltaL)) /
            pSSingleAxisPointer->SPlanData.dTc;
        pSRun->dCurrentSpeed = pSG0->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL
                                   ? 0
                                   : pSSingleAxisPointer->dCurrentSpeed;
        //当前留下未移动的距离
        pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SPlanData.dTargetDist - dDeltaL;

        //判断 触发软限位的轴 开始减速的时机：监测当前速度 >= 实时反算的最大速度（T型系统参数）//暂停后不进入判断
        if ((pSRun->cSoftLimitedOn == 1 || pSRun->cSoftLimitedOn == 2) &&
            pSSingleAxisPointer->ucRunMode != SINGLEAXIS_MODE)
        {
            dVmaxToLimitiedPoint = sqrt(pSSingleAxisPointer->dReCalculateVe * pSSingleAxisPointer->dReCalculateVe +
                                        2 * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dSysPlanTMaxDec *
                                            pSSingleAxisPointer->SPlanData.dCurLeftDist);
            if (pSSingleAxisPointer->dCurrentSpeed >= dVmaxToLimitiedPoint ||
                isZero(
                    pSSingleAxisPointer->SPlanData
                        .dCurLeftDist)) //修改2025.12.17增加isZero特殊情况，当前点为软限位点且速度小于最大允许速度时也进入减速停
            {
                for (int i = 0; i < AXIS_NUM_MAX; i++) //当前轴进入减速后，其余有效轴也同时进入减速
                {
                    if (VAL_VALID != getbit(pSG0->ucValStatus, i))
                        continue; //无效轴 跳过
                    setSingleAxisStop(pSG0->pSChannel->ucChannelNum, i, INP_DECELERATE,
                                      g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec); //设置进入减速停
                }
                if (pSRun->cSoftLimitedOn == 2) //限位点在当前线段外
                {
                    pSRun->cSoftLimitedOn = 3; //软限位开关变量置为3,需要后续线段参与减速
                }
                if (!isZero(pSSingleAxisPointer->SPlanData.dCurLeftDist)) //当前段剩余距离还未走完：进入减速停
                {
                    pSSingleAxisPointer->SPlanData.dDec =
                        (pow(pSSingleAxisPointer->dCurrentSpeed, 2) - pow(pSSingleAxisPointer->dReCalculateVe, 2)) /
                        (2.0 * pSSingleAxisPointer->SPlanData.dCurLeftDist); //重算T型减速度
                }
                else //当前段剩余距离已走完：进入减速停报警
                {
                    pSSingleAxisPointer->SPlanData.dDec = 0;
                    break;
                }
            }
        }
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        if ((pSG0->bIsUserData || !isHandwheelAutomaticMoveRequired(pSG0->pSChannel) ||
             pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5) &&
            ((pSSingleAxisPointer->SPlanData.dCurLeftDist <= EPSINON &&
              pSSingleAxisPointer->SPlanData.cInterpDir != INP_GO_BACK) ||
             (pSSingleAxisPointer->SPlanData.dTargetDist - pSSingleAxisPointer->SPlanData.dCurLeftDist <= EPSINON &&
              pSSingleAxisPointer->SPlanData.cInterpDir == INP_GO_BACK)))
#else
        if ((pSSingleAxisPointer->SPlanData.dCurLeftDist <= EPSINON &&
             pSSingleAxisPointer->SPlanData.cInterpDir != INP_GO_BACK) ||
            (pSSingleAxisPointer->SPlanData.dTargetDist - pSSingleAxisPointer->SPlanData.dCurLeftDist <= EPSINON &&
             pSSingleAxisPointer->SPlanData.cInterpDir == INP_GO_BACK)) //运行结束条件
#endif
        {
            //pSSingleAxisPointer->dCurrentSpeed = 0;//运动结束 当前速度置为0 //2024/5/21 洪冠鹏 注释原因：G0加入前瞻后当前速度需要保存置运行结构体里
            pSSingleAxisPointer->dCurrentSpeed =
                pSG0->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL
                    ? 0
                    : pSSingleAxisPointer->dCurrentSpeed;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
            if (!pSG0->bIsUserData && isHandwheelAutomaticMoveRequired(pSG0->pSChannel) &&
                pSSingleAxisPointer->SPlanData.dVe == 0.0)
            {
                pSSingleAxisPointer->dCurrentSpeed = 0.0;
                if (pSG0->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == ucAxis)
                {
                    pSRun->dCurrentSpeed = 0.0;
                }
            }
#endif
            pSG0->cInterpDir = pSSingleAxisPointer->SPlanData.cInterpDir;
            pSSingleAxisPointer->ucInpStatus = INP_FINISHED; //修改2025.11.8
            setbit(pSG0->ucCurrentSigned, ucAxis);
            //pSSingleAxisPointer->ucRunMode = SINGLEAXIS_MODE; //若为限位模式 切换为单轴模式 朱稳中 2023.6.21 //2024/12/17 注释：在多段参与软限位减速停时单轴状态仍应保持为限位模式
        }
        //改变实时点的位置
        updateSingleAxisRealTimePoint(&dAbsCoord, ucAxis, ucMapArray);
        break;

    case INP_DECELERATE: //根据单轴系统参数的加速度（不用规划）
        if (pSRun->cSoftLimitedOn == 0 ||
            pSSingleAxisPointer->ucRunMode ==
                SINGLEAXIS_MODE) //未触发软限位的减速运动[速度模式]:手动/回零下的减速运动以停下来为准
        {
            if (pSSingleAxisPointer->dCurrentSpeed <=
                g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe) // 速度若达到末速度范围内  立即停
            {

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE          //倾斜Y轴机
                if (pSG0->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE) //非回零模式
                {
                    if (ucAxis == AXIS_X && isZero(pSG0->pSSingleAxisArray[AXIS_X].dStartVector))
                    {
                        //X1轴插补距离为0，仅跟随Y1轴运动补偿随动量->不判断结束
                    }
                    else
                    {
                        pSRun->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed = 0; //运动结束 当前速度置为0
                        pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
                        break;
                    }
                }
                else //回零模式
                {
                    pSRun->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed = 0; //运动结束 当前速度置为0
                    pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
                    break;
                }
#else

                pSRun->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed = 0; //运动结束 当前速度置为0
                pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
                break;
#endif
            }
        }
        if (pSSingleAxisPointer->dCurrentSpeed >=
            pSSingleAxisPointer->SPlanData.dDec * pSSingleAxisPointer->SPlanData.dTc)
        {
            //计算长轴相对起点的距离 利用加速度计算//2024/12/17 注释：减速距离公式错误，匀减速每个周期的位移增量为Vcur * 1 - 0.5 * Dec * 1 * 1。
            //dDeltaL = pSSingleAxisPointer->SPlanData.dPreDist + pSSingleAxisPointer->SPlanData.dTargetDist - pSSingleAxisPointer->SPlanData.dCurLeftDist + pSSingleAxisPointer->dCurrentSpeed - pSSingleAxisPointer->SPlanData.dDec * pSSingleAxisPointer->SPlanData.dTc;
            dDeltaL = pSSingleAxisPointer->SPlanData.dTargetDist - pSSingleAxisPointer->SPlanData.dCurLeftDist +
                      pSSingleAxisPointer->dCurrentSpeed - 0.5 * pSSingleAxisPointer->SPlanData.dDec;
        }
        else
        {
            if (pSRun->cSoftLimitedOn == 0 ||
                pSSingleAxisPointer->ucRunMode ==
                    SINGLEAXIS_MODE) //未触发软限位的减速运动[速度模式]：最后一个周期要求速度停下，速度停下来处即为终点
            {
                dDeltaL = pSSingleAxisPointer->SPlanData.dTargetDist - pSSingleAxisPointer->SPlanData.dCurLeftDist;
            }
            else //触发软限位的减速运动[位置模式]：最后一个周期要求停到终点位置
            {
                dDeltaL = pSSingleAxisPointer->SPlanData.dTargetDist;
                pSSingleAxisPointer->SPlanData.dCurLeftDist = 0;
            }
        }

        //判断是否会超过终点
        if (dDeltaL > pSSingleAxisPointer->SPlanData.dTargetDist)
        {
            dDeltaL = pSSingleAxisPointer->SPlanData.dTargetDist;
        }

        //计算新的当前点坐标
        dAbsCoord = pSSingleAxisPointer->dStartPoint + (pSSingleAxisPointer->dStartVector * dDeltaL);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE  //倾斜Y轴机
        if (pSG0->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE) //非回零模式
        {
            //将直角坐标系下的坐标转化为倾斜坐标系下的坐标
            if (ucAxis == AXIS_X)
            {
                double dY1;
                getSingleAxisRealTimePoint(&dY1, AXIS_Y, ucMapArray);
                dAbsCoord += 1.0 * dY1 * pSG0->pSChannel->dSINAngleYY1;
            }
            if (ucAxis == AXIS_Y)
            {
                dAbsCoord = dAbsCoord / pSG0->pSChannel->dCOSAngleYY1;
            }
        }
#endif

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
        dAbsCoord =
            outPutAbsPointWithScrewCompForG0(pSRun, dAbsCoord, ucAxis, ucMapArray, pSSingleAxisPointer->dPulseToMm);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
        dAbsCoord = outPutAbsPointWithBilateralScrewCompForG0(pSRun, dAbsCoord, ucAxis, ucMapArray,
                                                              pSSingleAxisPointer->dPulseToMm);
#else
        //获取实时点作为上一点的位置
        getSingleAxisRealTimePoint(&dLastAbsCoord, ucAxis, ucMapArray);
        //各个轴的位移增量，    注意数组的下标减1,单位:pulse(输出指令单位)
        pSRun->sAxisDeltaPulse[ucAxis] = round((dAbsCoord - dLastAbsCoord) / pSSingleAxisPointer->dPulseToMm);

        //修正当前绝对坐标（利用上一点坐标和取整后的脉冲 计算实际坐标）
        dAbsCoord = dLastAbsCoord + (double)pSRun->sAxisDeltaPulse[ucAxis] * pSSingleAxisPointer->dPulseToMm;
#endif
        ////获取实时点作为上一点的位置
        //getSingleAxisRealTimePoint(&dLastAbsCoord, ucAxis, ucMapArray);

        ////计算新的当前点坐标
        //dAbsCoord = pSSingleAxisPointer->dStartPoint + (pSSingleAxisPointer->dStartVector * dDeltaL);

        ////各个轴的位移增量，    注意数组的下标减1,单位:pulse(输出指令单位)
        //pSRun->sAxisDeltaPulse[ucAxis] = round((dAbsCoord - dLastAbsCoord) / pSSingleAxisPointer->dPulseToMm);

        ////修正当前绝对坐标（利用上一点坐标和取整后的脉冲 计算实际坐标）
        //dAbsCoord = dLastAbsCoord + (double)pSRun->sAxisDeltaPulse[ucAxis] * pSSingleAxisPointer->dPulseToMm;

        //计算当前速度 根据距离计算  = (剩余距离(上周期) - 剩余距离(当前周期))/dTc   //2024/12/12 洪冠鹏注释：该公式计算的是插补周期1ms的平均速度（移动量/1ms），而在减速过程计算相对起点的公式计算使用的是移动后的瞬时速度
        //pSSingleAxisPointer->dCurrentSpeed = fabs(pSSingleAxisPointer->SPlanData.dCurLeftDist - (pSSingleAxisPointer->SPlanData.dTargetDist - dDeltaL)) / pSSingleAxisPointer->SPlanData.dTc;
        pSSingleAxisPointer->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed -
                                             pSSingleAxisPointer->SPlanData.dDec * pSSingleAxisPointer->SPlanData.dTc;
        pSRun->dCurrentSpeed = pSG0->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == ucAxis
                                   ? pSSingleAxisPointer->dCurrentSpeed
                                   : 0;
        //未走距离
        pSSingleAxisPointer->SPlanData.dCurLeftDist =
            pSSingleAxisPointer->SPlanData.dTargetDist - dDeltaL; //当前留下未移动的距离

        if (pSSingleAxisPointer->SPlanData.dCurLeftDist <= EPSINON) //运行结束条件
        {
            if (pSRun->cSoftLimitedOn == 1 &&
                pSG0->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE) //触发软限位：当前线段能够停下
            {
                if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_POS)
                {
                    parseRunningError(pSG0->pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM41 + ucAxis, 9999);
                    // setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数
                }
                if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_NEG)
                {
                    parseRunningError(pSG0->pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM47 + ucAxis, 9999);
                    // setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数
                }
                pSRun->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed = 0; //运动结束 当前速度置为0
                pSSingleAxisPointer->ucRunMode = SINGLEAXIS_MODE;              ///若为限位模式 切换为单轴模式
            }
            else if (pSRun->cSoftLimitedOn == 3 &&
                     pSG0->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE) //触发软限位：限位点在当前线段外，仍需减速
            {
                pSRun->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed; //当前速度保存在运行结构体里
            }
            else //未触发软限位的减速停止
            {

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE          //倾斜Y轴机
                if (pSG0->pSChannel->bUseVirtualY == TRUE && g_RunMode != RE0_MODE) //非回零模式
                {
                    if (ucAxis == AXIS_X && isZero(pSG0->pSSingleAxisArray[AXIS_X].dStartVector))
                    {
                        //X1轴插补距离为0 ->不置为完成状态，否则丢失随动补偿
                    }
                    else
                    {
                        pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
                    }
                }
                else //回零模式
                {
                    pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
                }
#else

                pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
#endif
            }
            setbit(pSG0->ucCurrentSigned, ucAxis); //当前轴对应的标志位为完成
        }

        //改变实时点的位置
        updateSingleAxisRealTimePoint(&dAbsCoord, ucAxis, ucMapArray);

        break;

    case INP_STOP:                                                     //立即停
        pSRun->dCurrentSpeed = pSSingleAxisPointer->dCurrentSpeed = 0; //运动结束 当前速度置为0

        setbit(pSG0->ucCurrentSigned, ucAxis); //当前轴对应的标志位为完成
        pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
        pSSingleAxisPointer->ucRunMode = SINGLEAXIS_MODE; ///若为限位模式 切换为单轴模式 朱稳中 2023.6.21
        break;
    }

    return 1; // 正常运行 插补完成
}

/*************************************************
  函数名称：singleAxisMotionLoop(struct SG0* pSG0Pointer, int nAxisNum, bool bPauseOn, bool bRatioOn)
  函数功能  单轴插补运动循环
  输入参数：pSG0Pointer				G0结构体
			nAxisNum				轴数
			bPauseOn 是否开启暂停
			bRatioOn 是否开启倍率
  输出参数：
  返 回 值：-1 空指针
			1  运行完成
  备    注：
**************************************************/
int8 singleAxisMotionLoop(struct SG0* pSG0Pointer, int nAxisNum, bool bPauseOn, bool bRatioOn)
{
    int i = 0;
    int nRes = 0;
    //空指针
    if (pSG0Pointer == NULL)
    {
        return -1;
    }

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* [HWSIM] G0 的逐拍距离入口：程序运动且允许暂停/倍率控制时由全轴执行器接管，
     * 直接 return，避免下面的旧 singleAxisMotion() 再消费同一周期。
     * SYS_RUN_TO_PAUSE 仍要进入，才能沿距离规划器逐拍减速，而不是突然停算。
     */
    if (bPauseOn && bRatioOn && !pSG0Pointer->bIsUserData && isHandwheelDistanceControlActive(pSG0Pointer->pSChannel) &&
        (pSG0Pointer->pSChannel->ucChannelStatus == SYS_RUN ||
         pSG0Pointer->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE))
        return executeG0HandwheelStep(pSG0Pointer);
#endif
    //循环执行单轴插补指令
#if (MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE) //倾斜Y轴机
    for (i = nAxisNum - 1; i >= 0; i--)                                      //X轴的移动要依据Y轴移动量确定补偿量
    {
#else
    for (i = 0; i < nAxisNum; i++)
    {
#endif
        nRes = singleAxisMotion(pSG0Pointer, i, bPauseOn, bRatioOn);
        if (nRes < 0)
            return nRes;
    }
    return 1;
}

/*************************************************
  函数名称：motionQuickMove(struct SG0 *pSG0Pointer)
  函数功能  快速运动函数
  输入参数：pSG0Pointer		G0运动结构体
  输出参数：
  返 回 值：-1 空指针
			1  运行中
  备    注：
****************************************/
int8 motionQuickMove(struct SG0* pSG0Pointer)
{
    struct SInterpolationPre* pSPreProcess; //预处理指针
    const uint8* ucMapArray;                //映射
    int i;
    int nRes = 1;
    VAL_STRUCT_TYPE ucValStatus;
    if (pSG0Pointer == NULL) //空指针
    {
        return -1;
    }
    pSPreProcess = &pSG0Pointer->pSChannel->SPreProcess; //预处理结构体
    ucMapArray = pSG0Pointer->pSChannel->pMapAxisArray;  //映射

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* [HWSIM] 先处理 G84 等自动旁路段的交接；普通可模拟段此函数直接通过。
	 * bIsUserData 用于用户数据/恢复运动，不应被程序段手轮距离控制误接管。
	 */
    if (!pSG0Pointer->bIsUserData && prepareHandwheelAutomaticMove(pSG0Pointer->pSChannel) < 0)
    {
        return -1;
    }
#endif
    switch (pSG0Pointer->ucRunStep)
    {
    case 0:
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        /* 在原极坐标 G0 可能跳过执行之前，先拒绝不支持的插补模式。 */
        if (isHandwheelDistanceControlActive(pSG0Pointer->pSChannel) &&
            getBits(pSG0Pointer->pSChannel->SRun.unCommandType, CMD_INTERPOLATION_MODE_ADDR) !=
                CMD_INTERPOLATION_COMMON)
            return reportG0HandwheelDistanceError(pSG0Pointer);
#endif
        //无前瞻时 G0前肯定能暂停完成
        //if (pSG0Pointer->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE)// 系统状态为TO_PAUSE
        //{
        //	// 插补读指针往回移动一个位置
        //	//(因为快速运动起始速度在允许直接停的速度范围内，所以可以不判断，直接停下不做运动，指针移回
        //	// 而合成运动的速度未必在直接停的范围内，因此还需要判断)
        //	moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_BACK,pSG0Pointer->pSChannel->ucChannelNum);  //移回前瞻指针
        //	moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pSG0Pointer->pSChannel->ucChannelNum);  //移回插补指针

        //	pSG0Pointer->pSChannel->ucChannelStatus = SYS_PAUSE;    //将系统状态改为暂停
        //	pSPreProcess->ucInpStatus = INP_FINISHED; //插补结构体为完成插补
        //	return;  //返回
        //}
        //极坐标插补
        if (getBits(pSPreProcess->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
        {
            pSPreProcess->ucInpStatus = INP_FINISHED;
            pSG0Pointer->ucCurrentSigned = pSG0Pointer->ucFinishedSigned;
            pSG0Pointer->ucRunStep = 3;
            break;
        }
        pSPreProcess->ucInpStatus = INP_RUNNING; //插补预处理状态为RUNNING

        //释放旧的有效轴
        releaseLastCommandAxis(pSPreProcess);
        ucValStatus =
            pSG0Pointer->bIsUserData ? pSG0Pointer->SUserData.ucValStatus : pSPreProcess->SUserData.ucValStatus;
        //判断轴占用(根据有效轴+映射)

#if (TURRET_AXIS_TEST == 1)
        if (axisManagerOccupyByChannel(ucValStatus & (~pSPreProcess->ucOccupideTurretAxis),
                                       pSG0Pointer->pSChannel->ucChannelNum, &pSPreProcess->ucOccupideValStatus) == 1)
        { //占用成功

            pSG0Pointer->ucRunStep += 1; //完成  下一步 执行运动
        }
        else
        { //占用失败
            break;
        }
#else
        if (axisManagerOccupyByChannel(ucValStatus & (~pSPreProcess->ucToolAxis), pSG0Pointer->pSChannel->ucChannelNum,
                                       NULL) == 1)
        { //占用成功
            //上一条指令的占用（除去刀补）
            pSPreProcess->ucLastValStatus = ucValStatus & (~pSPreProcess->ucToolAxis);
            pSG0Pointer->ucRunStep += 1; //完成  下一步 执行运动
        }
        else
        { //占用失败
            break;
        }
#endif
    case 1:
        //加载插补数据
        if (pSG0Pointer->bIsUserData) //根据标志位读取数据 2022.12.6
        {                             //读取用户数据
            nRes = loadSG0DataFromInput(pSG0Pointer, &pSG0Pointer->SUserData);
        }
        else
        { //读取预处理参数
            nRes = loadSG0DataFromInput(pSG0Pointer, &pSPreProcess->SUserData);
        }
        if (nRes != 1)
            return -1;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        /* [HWSIM] 首次装载成功后才绑定：此时各轴起终点、方向、长度和周期已有效。
		 * case 1 完成后会继续进入 case 2，因此本次调用就可能执行第一拍；
		 * 绑定清掉旧授权，没有新输入时第一拍仅等待。
		 */
        if (isHandwheelDistanceControlActive(pSG0Pointer->pSChannel) &&
            (pSG0Pointer->pSChannel->ucChannelStatus == SYS_RUN ||
             pSG0Pointer->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE) &&
            prepareG0HandwheelMove(pSG0Pointer) < 0)
            return -1;
#endif

        pSG0Pointer->ucRunStep += 1; //初始化完成  下一步 执行运动
    case 2:
        //单轴插补循环
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        nRes = singleAxisMotionLoop(pSG0Pointer, AXIS_NUM_MAX, TRUE, TRUE);
        if (nRes < 0)
            return nRes;
#else
        singleAxisMotionLoop(pSG0Pointer, AXIS_NUM_MAX, TRUE, TRUE);
#endif

        if (pSG0Pointer->ucCurrentSigned == pSG0Pointer->ucFinishedSigned) //运动完成
        {
            //都完成插补  该段快速运动结束
            //预处理状态置为完成
            pSPreProcess->ucInpStatus = INP_FINISHED;
            pSPreProcess->cInterpDir = pSG0Pointer->cInterpDir;
            //pSG0Pointer->pSChannel->SRun.dCurrentSpeed = 0;//当前合成速度为0//2024.5.26注释
            for (i = 0; i < AXIS_NUM_MAX; i++)
            {
                if (pSG0Pointer->pSSingleAxisArray[i].ucRunMode == SINGLEAXIS_MODE)
                {
                    pSG0Pointer->pSSingleAxisArray[i].ucInpStatus = INP_FINISHED;
                    pSG0Pointer->pSSingleAxisArray[i].dCurrentSpeed =
                        pSG0Pointer->pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == i
                            ? pSG0Pointer->pSSingleAxisArray[i].dCurrentSpeed
                            : 0;
                }
            }
            //判断系统状态是否为TO_PAUSE  若是 进行回写
            if (pSG0Pointer->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE
#if !HANDWHEEL_SIM_DISTANCE_ENABLE
                && pSG0Pointer->pSChannel->cHandWheelSimulaStatus != HANWHEELSIMULA_BUILDING
#endif
            )
            {
                int8 cRes = quickMoveWriteBack(pSG0Pointer); //回写函数
                if (cRes == 2)                               //当前段未完成进行回写
                {
                    pSPreProcess->ucInpStatus = INP_PAUSING;
                }
            }
            //触发软限位在各轴停下后重置开关变量及系统状态
            if (pSG0Pointer->pSChannel->SRun.cSoftLimitedOn == 1 &&
                pSG0Pointer->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE)
            {
                setErrorWriteFlag(TRUE); //修改2025.12.11 各轴减速停完成，恢复报警写入标志位，主轴停止
                pSG0Pointer->pSChannel->SRun.cSoftLimitedOn = 0;    //软限位开关变量:置0关闭
                pSG0Pointer->pSChannel->ucChannelStatus = SYS_IDLE; //停下后系统状态:空闲
            }
            pSG0Pointer->ucRunStep = 3; //运行完成 将Step赋值为一个不会运动的状态
        }
        break;
    case 3:
        return 0;
    }

    return 1;
}

/*************************************************
  函数名称：checkSingleAxisSoftLimit()
  函数功能：检查当前轴是否触发软限位
  输入参数：pSChannel - 通道结构体指针
			ucAxis - 轴号
			SMaxVal - 最大值点
			SMinVal - 最小值点
  返 回 值：  1 - 触发限位
			 0 - 无效轴
			-1 - 有效轴但未触发限位
  备    注：//2025.10.9
**************************************************/
int8 checkSingleAxisSoftLimit(struct SChannel* pSChannel, uint8 ucAxis, SPointDouble SMaxVal, SPointDouble SMinVal)
{
    uint8* pMapAxisArray = pSChannel->pMapAxisArray;

    // 检查轴映射和类型
    if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][ucAxis] == AXIS_NULL ||
        g_SSystemPara.SAxisPara[pMapAxisArray[ucAxis]].ucAxisType == ROTATION_AXIS)
        return 0; // 无效轴

    if (VAL_VALID != getbit(pSChannel->SPreProcess.SUserData.ucValStatus, ucAxis))
        return 0; // 无效轴

    // 正限位检查
    if (g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAxis]].soft_pos == 1) // 开启正限位
    {
        if (SMaxVal.dArray[ucAxis] > g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAxis]].pos_soft) // 最大值大于正限位值
        {
            pSChannel->SSingleAxisArray[ucAxis].ucRunMode = SINGLEAXIS_LIMIT_POS; // 设置正限位模式
            return 1;
        }
    }

    // 负限位检查
    if (g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAxis]].soft_neg == 1) // 开启负限位
    {
        if (SMinVal.dArray[ucAxis] < g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAxis]].neg_soft) // 最小值小于负限位值
        {
            pSChannel->SSingleAxisArray[ucAxis].ucRunMode = SINGLEAXIS_LIMIT_NEG; // 设置负限位模式
            return 1;
        }
    }

    return -1; // 有效轴但未触发限位
}
