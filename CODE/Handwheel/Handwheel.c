#include "Handwheel.h"
#include "Motion.h"
#include "Channel.h"
#include "Config.h"
#include <stdlib.h>
#include "GeneralFunctions.h"
#include "CTrajPlanning.h"
#if HANDWHEEL_STOP_BOUNDARY_ENABLE
/*
 * 普通选轴手轮新路径。建议先读 getHandWheelData 中的 HW-1 ~ HW-7，
 * 再回头读上面的辅助函数；#else 下完整保留 H7 原双缓存实现。
 *
 * 本路径只有“正常”和“制动”两态：
 *   正常：net = 相对 SStartPoint 的有符号目标距离。
 *   制动：net = 相对锁定刹停点的有符号待走距离。
 * 输入只做 net += input；进入/取消制动、零速换起点时转换 net 的基准。
 * 制动中同向余量重新越过刹停边界时允许带速恢复；真正换向仍须先到零速。
 *
 * 所有位置/速度均为规划指令，不是伺服编码器反馈。
 * 正向为 +1、反向为 -1；SPlan 始终计算正的行程，最终由 cDir 加方向。
 * 直线轴使用 mm、mm/ms、mm/ms^2，旋转轴使用 degree、degree/ms、
 * degree/ms^2。dHandwheelNetDist 不是电机脉冲数，也不是每拍位移。
 */
#include <math.h>
#include "SysParaAPI.h"
#if STM32_MODE
#include "limit_warn.h"
#endif

/*************************************************
  函数名称：handwheelAccumulateNetInput
  函数功能  将本周期输入换算为有符号距离，正反输入在一个变量中抵消
  输入参数：pSChannel 通道；轴号、倍率及原始脉冲来自手轮编码器
  输出参数：编码器 dHandwheelNetDist、四倍频残余
  返 回 值：无
  备    注：无选轴/限位只拒绝输入，不能打断其他轴已经建立的制动
****************************************/
static void handwheelAccumulateNetInput(struct SChannel* pSChannel)
{
    struct SEncoder* pEncoder = pSChannel->pSHandleWheelEncoder;
    uint8 axis = pEncoder->AXISNum;
    uint8 physical;
    int32 raw, ticks;
    double point, ratio, input, quantum;

    /*
	 * 本函数只负责本次选中轴的输入；return 只退出输入采集。
	 * 主函数仍会遍历所有轴，继续运行已建立的减速计划。
	 */
    if (axis >= AXIS_NUM_MAX || pSChannel->pMapAxisArray[axis] >= AXIS_PHYSICS_NUM_MAX)
    {
        pEncoder->dDelataAccmulate = 0;
        return;
    }
    physical = pSChannel->pMapAxisArray[axis];
    getSingleAxisRealTimePoint(&point, axis, pSChannel->pMapAxisArray);
    if ((pEncoder->cEncoderDir == INP_GO_BACK && g_SSystemPara.MOTOR_WARN[physical].soft_neg &&
         point < g_SSystemPara.MOTOR_WARN[physical].neg_soft) ||
        (pEncoder->cEncoderDir == INP_GO_AHEAD && g_SSystemPara.MOTOR_WARN[physical].soft_pos &&
         point > g_SSystemPara.MOTOR_WARN[physical].pos_soft))
    {
        pEncoder->dDelataAccmulate = 0;
        return;
    }
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE
    if (pSChannel->bUseVirtualY && axis == AXIS_Y)
    {
        uint8 x = pSChannel->pMapAxisArray[AXIS_X];
        if (x >= AXIS_PHYSICS_NUM_MAX)
            return;
        getSingleAxisRealTimePoint(&point, AXIS_X, pSChannel->pMapAxisArray);
        if ((pEncoder->cEncoderDir == INP_GO_BACK && g_SSystemPara.MOTOR_WARN[x].soft_neg &&
             point < g_SSystemPara.MOTOR_WARN[x].neg_soft) ||
            (pEncoder->cEncoderDir == INP_GO_AHEAD && g_SSystemPara.MOTOR_WARN[x].soft_pos &&
             point > g_SSystemPara.MOTOR_WARN[x].pos_soft))
            return;
    }
#endif
    if (pEncoder->cEncoderDir != INP_GO_AHEAD && pEncoder->cEncoderDir != INP_GO_BACK) //静止时退出
        return;

    //四倍频余数也带符号，未满一格时换向应先抵消，而不是累出一个错误方向的格数。
    raw = pEncoder->dDelataAccmulate + (int32)pEncoder->cEncoderDir * pEncoder->usDelta;
    ticks =
        raw /
        4; //ticks代表本周期转过的完整物理格数（带符号：+2格 / -3格等）。作用是取出已经凑满 1 整格的整数部分送去走位移
    pEncoder->dDelataAccmulate =
        (int16)(raw - ticks * 4); //dDelataAccmulate保留本周期不足 4 的余数（-3 ~ +3），留给下周期积分
    quantum = g_SSystemPara.SAxisPara[physical].dPulseToMm; //quantum表示该轴 1 个电机脉冲对应的物理位移当量dPulseToMm
    /*
	 * 沿用 H7 的手轮单位换算：四倍频计数 -> 格数 -> 倍率 ->
	 * 理论电机脉冲 -> 本轴距离。直径显示时，直线轴实际位移取一半。
	 * ratio 和 quantum 的旋转轴含义沿用原配置，不能再额外乘一次 360。
	 */
    if (g_SSystemPara.SAxisPara[physical].ucAxisType == LINEAR_AXIS) //直线轴
    { //数控行业标准规定手轮在 x1 倍率下，每转 1 格走 0.001 mm（或旋转轴 0.001°）
        ratio =
            g_SSystemPara.SAxisPara[physical].usMmToPulse /
            1000.0; //usMmToPulse是1mm对应的电机脉冲数，则0.001mm（1格基准）对应的理论电机脉冲数为 ratio = usMmToPulse/ 1000
        if (getShowByDiameter(physical) == 1)
            ratio *= 0.5; //车床直径编程，手轮转 1 格显示直径变 0.001mm，但刀架实际单边半径位移只有一半
    }
    else //旋转轴
    {
        ratio = g_SSystemPara.SAxisPara[physical].usMmToPulse /
                360000.0; //一圈 360° 对应 usMmToPulse 个脉冲，则 0.001° 对应 usMmToPulse / 360000.0 个理论脉冲
    }
    input =
        (double)ticks * pEncoder->ucWheelMultiple * ratio *
        quantum; //总位移（mm，°） = 格数 * 倍率 * 脉冲/格 * 距离/脉冲，input就是当前周期手轮输入的绝对物理距离（mm 或 degree），自带方向正负符号。
    if (!isfinite(input) || !isfinite(quantum) || quantum <= 0.0 ||
        !isfinite(pEncoder->dHandwheelNetDist[axis] + input))
    {
        parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
        return;
    }
    /*
	 * 正常时抵消原目标；制动时抵消停后余量。两种状态都只写这个变量，
	 * 不在这里判断轴的运动方向，更不能把输入直接加到实时坐标上。
	 * 小于一个电机脉冲的量仍保留为 double，最后交给原输出函数量化。
	 */
    pEncoder->dHandwheelNetDist[axis] += input;
}

/*************************************************
  函数名称：handwheelBoundarySpeed
  函数功能  由当前 T 型计划取指定周期末的速度大小
  输入参数：pPlan 当前计划；k 已执行的周期数
  输出参数：无
  返 回 值：周期边界速度，单位与 SPlan 一致
  备    注：普通手轮每拍重规划时 k=1；固定制动段的 k 必须逐拍递增
****************************************/
static double handwheelBoundarySpeed(const struct SPlan* pPlan, uint32 k)
{
    /*
	 * unT1 是加速终点，unT3 是匀速终点，unT5 是减速终点。
	 * 减速段从末速度向前反算，使用与 nextstepPlanningT 相同的周期节点。
	 * 这里不是“本拍脉冲 / Tc”：后者是周期平均值，不能混用作边界速度。
	 * 本路径普通重规划前已清平摊，专用小段/制动段也不附加平摊速度。
	 */
    if (k == 0)
        return pPlan->dVs;
    if (k <= pPlan->unT1)
        return max(0.0, pPlan->dVs + pPlan->dAcc * k * pPlan->dTc);
    if (k <= pPlan->unT3)
        return max(0.0, pPlan->dVmax);
    if (k < pPlan->unT5)
        return max(0.0, pPlan->dVe + pPlan->dDec * (pPlan->unT5 - k) * pPlan->dTc);
    return max(0.0, pPlan->dVe);
}

/*************************************************
  函数名称：handwheelPrepareShortMove
  函数功能  为零速起步的短余量建立周期对齐的对称三角形计划
  输入参数：pPlan 普通计划；distance 目标距离；dec 减速度；vmax 限速
  输出参数：pPlan
  返 回 值：1 成功；-1 周期数或计算结果无效
  备    注：仅处理数个周期以内的小段，避免原新增距离规划取整后在终点往复
****************************************/
static int handwheelPrepareShortMove(struct SPlan* pPlan, double distance, double dec, double vmax)
{
    if (!isfinite(distance) || distance <= 0.0 || !isfinite(dec) || dec <= 0.0 || !isfinite(vmax) || vmax <= 0.0 ||
        !isfinite(pPlan->dTc) || pPlan->dTc <= 0.0)
        return -1;
    double periods = max(1.0, ceil(sqrt(distance / dec) / pPlan->dTc));
    double duration; // 单边（加速段或减速段）的时长
    /*
	 * 加速、减速各 periods 拍，单边时长 duration。
	 * 三角形面积 distance = vmaxUsed * duration，
	 * accUsed = distance / duration^2，不超过 dec；峰值也不超过 vmax。
	 * 上取整后反算实际加速度，而不是把不足一拍的距离瞬间发完。
	 */
    periods = max(periods, ceil(distance / (vmax * pPlan->dTc)));
    duration = periods * pPlan->dTc; // 计算单边实际物理时长（拍数 × 插补周期）
    if (!isfinite(periods) || periods > 1073741823.0 || !isfinite(duration) || duration <= 0.0)
        return -1;
    clearSPlanData(pPlan);                                                  // 清空规划结构体历史残余数据
    pPlan->dDistance = pPlan->dTargetDist = pPlan->dCurLeftDist = distance; // 初始化总行程、目标行程与剩余待走距离
    pPlan->dVs = pPlan->dVe = 0.0;                                          // 起速与末速设为零（纯零速起停对称三角波）
    pPlan->dVmax = distance / duration; // 反算实际峰值速度（由三角形面积公式 S = Vmax * duration 倒推）
    pPlan->dAcc = pPlan->dDec = pPlan->dVmax / duration;       // 反算实际加/减速度（a = Vmax / duration <= dec）
    pPlan->dDistAcc = pPlan->dDistDec = 0.5 * distance;        // 加速段与减速段位移各占一半（严格对称）
    pPlan->unT1 = pPlan->unT2 = pPlan->unT3 = (uint32)periods; // 加速结束/匀速起止周期统一为 periods（无匀速巡航段）
    pPlan->unT4 = pPlan->unT5 = 2 * (uint32)periods;           // 减速起止总周期置为 2*periods（总共执行 2*periods 拍）
    /*
	 * 调用方紧接着输出第 1 拍，故记录 1。后续没有新目标时从第 2 拍续算。
	 * 这仍是正常态的短段子路径，并不是第三种换向状态。
	 */
    pPlan->unCurrentT = 1;            // 标记已处于第 1 拍（调用方立即下发首拍位移）
    pPlan->cInterpDir = INP_GO_AHEAD; // 规划器内部插补方向恒为正向（物理方向由外部 cDir 决定）
    pPlan->ucAccVelPlanType = pPlan->ucDecVelPlanType = VEL_PLAN_T_CURVE; // 加减速模型标记为 T 型（线性加减速）
    pPlan->ucDistReAverageType = DIST_REAVG_T_CURVE;                      // 平摊算法配置为 T 型曲线平摊
    return 1;                                                             // 成功建立微型对称三角规划
}

/*************************************************
  函数名称：handwheelTryBoundaryBrake
  函数功能  更新实时刹停边界，目标退到边界内时建立一次纯减速计划
  输入参数：pEncoder 手轮；axis 轴号；pPlan 当前计划；dec 制动减速度
  输出参数：固定的制动 SPlan、锁定方向、停止后净待走距离
  返 回 值：-1 参数错误；0 仍使用普通规划；1 已建立/正在执行制动
  备    注：预测与执行使用相同的整数周期、减速度，制动期间不重算
****************************************/
static int handwheelTryBoundaryBrake(struct SEncoder* pEncoder, int axis, struct SPlan* pPlan, double dec)
{
    double done, speed, cycles, duration, brakeDist, stop;
    /* 调用方未满足同向恢复条件时，继续锁定计划，不能带速改变运动方向。 */
    if (pEncoder->bHandwheelBraking[axis])
        return 1; // 已处于锁定制动态且未被 HW-3A 拉起，直接维持制动，严禁带速改变运动方向
    /*
	 * 必须先读旧计划，再清计划/覆盖目标。
	 * done 是本段从 SStartPoint 沿运动方向走过的非负行程；
	 * 它和下面 stop 都是“段起点坐标系中的距离”，不是机械绝对坐标。
	 */
    done = pPlan->dTargetDist - pPlan->dCurLeftDist; // 提取旧计划中沿当前方向已走过的物理行程
    speed = pEncoder->dAxisCurrentSpeed[axis];       // 读取当前末边界线速度
    if (!isfinite(done) || done < 0.0 || !isfinite(speed) || speed < 0.0 || !isfinite(dec) || dec <= 0.0 ||
        !isfinite(pPlan->dTc) || pPlan->dTc <= 0.0)
        return -1; // 浮点数值合法性校验
    if (speed == 0.0)
    {
        pEncoder->dHandwheelStopDist[axis] = done; // 绝对静止时刹停点即当前位置，无需制动
        return 0;
    }
    if (pEncoder->cHandwheelMoveDir[axis] != INP_GO_AHEAD && pEncoder->cHandwheelMoveDir[axis] != INP_GO_BACK)
        return -1; // 带速运行时必须已锁定有效方向，严禁无方向或带速突变
    /*
	 * 连续制动为 v^2/(2a)。这里执行器按整拍插补，采用：
	 * N=ceil(v/(a*Tc)), T=N*Tc, aUsed=v/T, brakeDist=v*T/2。
	 * 预测和执行共用 N/T/aUsed，不能预测用连续距离、执行另行取整。
	 */
    cycles = ceil(speed / (dec * pPlan->dTc)); // 减速所需整数拍数 N = ceil(v / (a * Tc))
    if (!isfinite(cycles) || cycles > 2147483647.0)
        return -1;
    cycles = max(1.0, cycles);          // 至少保留 1 拍用于停稳
    duration = cycles * pPlan->dTc;     // 制动总时长 T = N * Tc
    brakeDist = 0.5 * speed * duration; // 滑行位移 S_brake = 0.5 * v * T，对应实际减速度 aUsed = v / T <= dec
    stop = done + brakeDist;            // 绝对刹停边界 = 已走距离 + 滑行距离
    if (!isfinite(stop))
        return -1;
    pEncoder->dHandwheelStopDist[axis] = stop; // 缓存记录刹停点（供 HW-3A 顺向补摇拉起判定）
    /*
	 * 将带符号目标投影到原方向，再与 stop 比较，正反运动共用一个式子。
	 * 不可用 fabs(net)：一次反向输入跨过段起点时会丢失方向信息。
	 * 等于边界也制动。此处每拍检查，无新输入时进入自然停车边界也会制动。
	 */
    if (pEncoder->cHandwheelMoveDir[axis] * pEncoder->dHandwheelNetDist[axis] > stop)
        return 0; // 目标仍在刹停边界前方：目标充沛无需刹车，继续走普通随动重规划（HW-5B）

    /*
	 * 从这一拍开始锁定制动。累计变量改为“新目标 - 带方向的刹停点”：
	 * 原方向为正，新目标 65、刹停点 70，则净待走距离为 -5。
	 * 后续输入先改变该变量；调用方满足同向恢复条件时才取消制动。
	 * 未取消时不改变 pPlan 或锁定的运动方向。
	 * 无新输入时若正常运动也到达停车边界，同样先减到零，不能带速翻向。
	 */
    pEncoder->dHandwheelNetDist[axis] -= pEncoder->cHandwheelMoveDir[axis] * stop;
    /*
	 * clearSPlanData 同时清旧周期、距离和平摊参数，但保留 dTc。
	 * dPreDist=done 保持“相对原段起点”的累计行程连续；
	 * 新曲线只再走 brakeDist，末速度必须是 0，而不是轴配置的 dVe。
	 */
    clearSPlanData(pPlan);
    pPlan->dDistance = pPlan->dTargetDist = stop; // 物理执行层：锁定当前段终点为 stop，走满剩余滑行位移
    pPlan->dPreDist = done;                       // 继承已走行程，保证段内绝对坐标连续
    pPlan->dCurLeftDist = brakeDist;              // 当前计划仅剩余滑行距离待走
    pPlan->dVs = pPlan->dVmax = speed;            // 起速与峰值速度锁定为当前实际速度
    pPlan->dVe = 0.0;                             // 强制末速度为 0（绝对停死）
    pPlan->dAcc = dec;                            // 记录减速度上限
    pPlan->dDec = speed / duration;               // 实际执行减速度 aUsed = v / duration <= dec
    pPlan->dDistDec = brakeDist;                  // 减速段距离
    /* T1/T2/T3 已清零：没有加速和匀速，T4/T5=N 只留下减速段。 */
    pPlan->unT4 = pPlan->unT5 = (uint32)cycles; // 仅保留 N 拍纯减速段（T1/T2/T3 清零，无加速和匀速）
    pPlan->unCurrentT = 0;                      // 周期计数器置 0，交由调用方逐拍步进
    pPlan->cInterpDir = INP_GO_AHEAD;           // 规划器内部方向恒正（物理方向由锁定的 cDir 控制）
    pPlan->ucAccVelPlanType = pPlan->ucDecVelPlanType = VEL_PLAN_T_CURVE; // 加减速模型标记为 T 型
    pPlan->ucDistReAverageType = DIST_REAVG_T_CURVE;                      // 距离平摊算法配置为 T 型曲线平摊
    pEncoder->bHandwheelBraking[axis] = TRUE;                             // 正式置位制动锁标志，进入不可逆匀减速执行
    return 1;                                                             // 返回 1：已切入锁定制动计划
}
/*************************************************
  函数名称：getHandWheelData
  函数功能  手轮单拍插补流水线核心调度（HW-1 ~ HW-7 流水线执行）
  输入参数：pSChannel 当前通道，含手轮编码器及轴映射
  输出参数：各轴规划状态与 SRun 本周期输出指令坐标及脉冲
  返 回 值：无
  备    注：每拍 (1ms) 周期性调用；实现净输入累加、刹停判定、轨迹步进及停稳换段
****************************************/
void getHandWheelData(struct SChannel* pSChannel)
{
    if (pSChannel == NULL || pSChannel->pSHandleWheelEncoder == NULL || pSChannel->pMapAxisArray == NULL)
        return;
    //报警停车已接管轴时，不得在同一周期覆盖其输出或继续执行旧余量。
    if (isErrorHasBeenWritten() || pSChannel->ucChannelStatus == SYS_RESET
#if STM32_MODE
        || Check_bootwarning() == 0
#endif
    )
    {
        for (int axis = 0; axis < AXIS_NUM_MAX; ++axis)
        {
            pSChannel->pSHandleWheelEncoder->bHandwheelBraking[axis] = FALSE;
            pSChannel->pSHandleWheelEncoder->dHandwheelNetDist[axis] = 0.0;
            pSChannel->pSHandleWheelEncoder->cHandwheelMoveDir[axis] = 0;
            pSChannel->pSHandleWheelEncoder->dHandwheelStopDist[axis] = 0.0;
        }
        pSChannel->pSHandleWheelEncoder->dDelataAccmulate = 0;
        return;
    }
    struct SEncoder* pEncoder;
    pEncoder = pSChannel->pSHandleWheelEncoder;
    uint8* pMapAxisArray = pSChannel->pMapAxisArray;
    int i = 0;
    double dDeltaL[AXIS_NUM_MAX];    // 本拍末相对段起点的累计物理行程（非单拍增量）
    VAL_STRUCT_TYPE ucValStatus = 0; // 本周期有效输出轴标志位

    /* ==================== [HW-1] 净输入代数累加与限位拦截 ==================== */
    handwheelAccumulateNetInput(pSChannel);
    if (isErrorHasBeenWritten())
        return;

    //虚拟轴的映射不为空
    {
        double dAxisPulseToMm = 0;             // 当前轴本拍规划的目标总位移（非负标量，mm 或 °）
        SPointDouble STargetPoint;             // 各轴理论绝对坐标结构体，最终输出给底层插补器
        struct SPlan* pSPlanPointer;           // 当前轴专属 SPlan 轨迹规划器指针
        uint8 ucPlanState[AXIS_NUM_MAX] = {0}; // 各轴规划状态（0:未激活，2:正在插补，3:降到零速停稳）

        double dVmax = 0;         // 手轮最大速度
        double dDec = 0;          // 手轮最大减速度
        double dSlowDownDistance; // 最大减速滑行距离
        double dSurplusDistance;  // 溢出未走距离
        double dSurplusPulse;     // 溢出未走脉冲数
        bool bIsRotationAxis;     // 是否为旋转轴

        //SPointDouble SRealTimePoint;  //获取当前实时点
        //getRealTimePoint((double*)&SRealTimePoint, AXIS_NUM_MAX, pMapAxisArray, NULL);  //获取实时坐标
        getRealTimePoint((double*)&STargetPoint, AXIS_NUM_MAX, pMapAxisArray, NULL);

        /*
		 * HW-2：逐轴推进。手轮只选一个输入轴，但切换选轴后旧轴可能仍在制动。
		 * 因此不能把“没有选轴/没有输入”等同于“整个手轮函数不执行”。
		 */
        for (i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (pMapAxisArray[i] >= AXIS_PHYSICS_NUM_MAX)
                continue;
            dDeltaL[i] = 0; //初始化各个轴的位移增量
            pSPlanPointer = &pSChannel->SSingleAxisArray[i].SPlanData;
            //STargetPoint.dArray[i] = 0; //初始化各个轴目标坐标
            pSChannel->SRun.sAxisDeltaPulse[i] = 0; // 清空本周期发送脉冲缓存

            /*
		 * 轴激活与推进条件（满足其一即进入）：
		 * 1. 处于急刹制动态（bHandwheelBraking == TRUE，锁定减速推进）；
		 * 2. 处于带速运动中（speed > 0.0，包含加速、匀速与随动减速）；
		 * 3. 静止起步（speed == 0 且净目标超过 0.5 脉冲死区门槛，准备启动）。
		 */
            if (pEncoder->bHandwheelBraking[i] || pEncoder->dAxisCurrentSpeed[i] > 0.0 ||
                fabs(pEncoder->dHandwheelNetDist[i]) >
                    (0.5 + 1e-9) * g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm)
            {
                /*
				 * HW-3：只在零速且非制动态选择方向。反向摇手轮不等于轴立即反向。
				 * 正常带速时和整个制动期间都沿 cHandwheelMoveDir 原方向输出。
				 */
                if (!pEncoder->bHandwheelBraking[i] && pEncoder->dAxisCurrentSpeed[i] == 0.0)
                    pEncoder->cHandwheelMoveDir[i] = pEncoder->dHandwheelNetDist[i] > 0.0 ? INP_GO_AHEAD : INP_GO_BACK;
                int8 cDir = pEncoder->cHandwheelMoveDir[i];
                dAxisPulseToMm = fabs(pEncoder->dHandwheelNetDist[i]);
                if (!pEncoder->bHandwheelBraking[i] &&
                    !(pSPlanPointer->unCurrentT > 0 && pSPlanPointer->unCurrentT < pSPlanPointer->unT5))
                    pSPlanPointer->dTc = g_SSystemPara.dInterpolationTime;
                pSChannel->SG00.bIsUserData =
                    TRUE; // 手轮发生转动，转动数据来自用户输入，据此判断恢是否执行复加工  2023/12/14 洪冠鹏

                //企业增加 2023/12/20 新增程序起始位置 限制手轮停下后的脉冲数
                bIsRotationAxis = g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS ? FALSE : TRUE;
                // dVmax = min(g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVmax, g_SSystemPara.dHandWheelVmax);//2025/3/11限制手轮最大速度
                dVmax =
                    g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS
                        ? g_SSystemPara.dHandWheelVmax
                        : g_SSystemPara.dHandWheelVmax *
                              DEGREE_OF_CIRCLE; //手轮的最大速度   dHandWheelVmax的单位：mm/ms（r/ms） 计算旋转轴时统一单位°/ms
                dVmax = min(g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVmax,
                            dVmax); //2025/3/11限制手轮最大速度 //2025/9/23修改设置最大速度位置
                dDec =
                    g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS
                        ? g_SSystemPara.dHandWheelAcc
                        : g_SSystemPara.dHandWheelAcc *
                              DEGREE_OF_CIRCLE; //手轮的最大减速度 dHandWheelAcc的单位：mm/ms2（r/ms2） 计算旋转轴时统一单位°/ms2
                                                /*
				 * HW-3A：输入已由 HW-1 累加，先判断能否取消制动，再锁定 dDec。
				 * 此处 dDec/dVmax 是普通规划将使用的当前配置，不是旧制动参数。
				 * 只允许沿原方向恢复：dir*net 必须超过一个脉冲当量，
				 * 零/反向/不足门槛的余量继续保留，不在这里清除。
				 */
                if (pEncoder->bHandwheelBraking[i])
                {
                    double speed = pEncoder->dAxisCurrentSpeed[i];
                    double quantum = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm;
                    if ((cDir == INP_GO_AHEAD || cDir == INP_GO_BACK) && isfinite(speed) && speed > 0.0 &&
                        isfinite(dVmax) && dVmax > 0.0 && speed <= dVmax && isfinite(dDec) && dDec > 0.0 &&
                        isfinite(quantum) && quantum > 0.0 && isfinite(pSPlanPointer->dTc) &&
                        pSPlanPointer->dTc > 0.0 && pSPlanPointer->unCurrentT < pSPlanPointer->unT5 &&
                        cDir * pEncoder->dHandwheelNetDist[i] > quantum)
                    {
                        /*
						 * 制动余量换回原起点下的目标：例 net=+30、stop=70，
						 * 则 target=100；不能把 30 直接交给普通规划器。
						 * done 先从旧计划读取；此处不清计划、不换起点、不清速度。
						 */
                        double target = pEncoder->dHandwheelNetDist[i] + cDir * pEncoder->dHandwheelStopDist[i];
                        double done = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;
                        double cycles = ceil(speed / (dDec * pSPlanPointer->dTc));
                        if (isfinite(target) && isfinite(done) && done >= 0.0 && isfinite(cycles) &&
                            cycles <= 2147483647.0) //防止溢出
                        {
                            /* 与 HW-4 使用同一整拍停距，防止参数变化后立即再次制动。 */
                            double stopNow = done + 0.5 * speed * max(1.0, cycles) * pSPlanPointer->dTc;
                            if (isfinite(stopNow) && cDir * target > stopNow + quantum)
                            {
                                pEncoder->dHandwheelNetDist[i] = target; // 账本还原为全局目标
                                pEncoder->bHandwheelBraking[i] = FALSE;  // 解除制动锁，退回正常随动态
                                                                         /*
								 * 上方局部距离还是 fabs(net)=30，必须同步更新为 100。
								 * HW-4 随后刷新停点，HW-5B 用旧计划提取 done，
								 * 以当前速度重规划剩余距离；方向和 SStartPoint 不变。
								 */
                                dAxisPulseToMm = fabs(target);           // 规划目标同步刷新
                            }
                        }
                    }
                }
                if (pEncoder->bHandwheelBraking[i])
                    dDec = pSPlanPointer->dDec; // 制动已建立时锁定既定减速度，不受运行中参数变动影响
                if (!isfinite(dDec) || dDec <= 0.0 ||
                    (!pEncoder->bHandwheelBraking[i] && (!isfinite(dVmax) || dVmax <= 0.0)))
                {
                    parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
                    return;
                }
                /*
				 * HW-4：先查刹停边界，再决定要不要普通重规划。
				 * 小段目标未变时沿原三角形走完；目标变了仍重新检查边界。
				 * brakeState: -1 错误；0 普通重规划；1 制动；2 续算固定小段。
				 * 这里的 2 是局部分支标记，不是新增的运动状态。
				 */
                bool continueShort =
                    !pEncoder->bHandwheelBraking[i] && pSPlanPointer->unCurrentT > 0 &&
                    pSPlanPointer->unCurrentT < pSPlanPointer->unT5 &&
                    pEncoder->cHandwheelMoveDir[i] * pEncoder->dHandwheelNetDist[i] == pSPlanPointer->dTargetDist;
                int brakeState = continueShort ? 2 : handwheelTryBoundaryBrake(pEncoder, i, pSPlanPointer, dDec);
                if (brakeState < 0) // [-1] 轨迹/参数异常保护
                {
                    parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
                    return;
                }
                if (brakeState >= 1) // [1 或 2] 底层代码复用：逐拍推进既有曲线（1:场景B锁定制动 / 2:场景A微短段续走）
                {
                    /*
					 * HW-5A：固定制动/小段只建立一次，必须累计 k。
					 * 制动触发本拍从 k=1 开始，不多等一个正常运动周期。
					 * 完成看 k>=unT5（且末速度为零），不是只看剩余距离很小。
					 */
                    ++pSPlanPointer->unCurrentT;
                    dDeltaL[i] = nextstepPlanningAndDistReAvg(pSPlanPointer->unCurrentT, pSPlanPointer);
                    ucPlanState[i] =
                        (pSPlanPointer->unCurrentT >= pSPlanPointer->unT5)
                            ? 3
                            : 2; //拍数走满置 3（标记停稳，触发后续 HW-7 换段与反向折返）；未满置 2 保持插补
                }
                else // [0] HW-5B：场景 A 正常随动态长段（逐拍重新规划）
                {
                    /*
				 * HW-5B：正常长段仍用 handWheelAddNewPlan。
				 * 保留原 dhandwheelcontrol 限制选项；开启时会主动舍弃部分积压目标，
				 * 因而“所有手轮输入严格守恒”的测试应先把该选项设为 0。
				 */
                    //实时计算最大的减速距离，用于手轮停下后限制脉冲数
                    dSlowDownDistance = fabs(dVmax * dVmax - g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe *
                                                                 g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe) /
                                        2.0 / dDec;
                    if (dSlowDownDistance <
                        g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm) //减速距离小于一个脉冲对应的位移量
                    {
                        dSlowDownDistance =
                            g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm * 2; //预留多一个脉冲的位移量
                    }
                    else
                    {
                        dSlowDownDistance =
                            ((int)((fabs(dSlowDownDistance) / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm) +
                                   1)) *
                            g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm; //预留多一个脉冲的位移量
                    }
                    if (g_SSystemPara.dhandwheelcontrol == 1)
                    {
                        if (fabs(dAxisPulseToMm - (pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist)) >
                            dSlowDownDistance)
                        {
                            dSurplusDistance = (int)(fabs((dAxisPulseToMm -
                                                           (pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist) -
                                                           dSlowDownDistance)));
                            dSurplusPulse =
                                g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS
                                    ? dSurplusDistance * g_SSystemPara.SAxisPara[pMapAxisArray[i]].usMmToPulse
                                    : dSurplusDistance * g_SSystemPara.SAxisPara[pMapAxisArray[i]].usMmToPulse /
                                          g_SSystemPara.SAxisPara[pMapAxisArray[i]].fDistPerCircle;
                            //保留原“限制手轮积压距离”选项，但不能把目标裁到实时刹停点以内。
                            dAxisPulseToMm = max(
                                pEncoder->dHandwheelStopDist[i],
                                dAxisPulseToMm - dSurplusPulse * g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm);
                            pEncoder->dHandwheelNetDist[i] = cDir * dAxisPulseToMm;
                        }
                    }
                    //企业增加 2023/12/20 新增程序终点位置

                    //进行手轮控制速度规划
                    pSChannel->SSingleAxisArray[i].SPlanData.cInterpDir = INP_GO_AHEAD;
                    pSChannel->SSingleAxisArray[i].SPlanData.dVmax = bIsRotationAxis ? dVmax / DEGREE_OF_CIRCLE : dVmax;
                    // pSChannel->SSingleAxisArray[i].SPlanData.dVmax = min(g_SSystemPara.dHandWheelVmax, g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVmax);//2025/3/11限制手轮最大速度
                    //当前边界速度同时传作起速，避免极低非零速度被 isZero 判成零速起步。
                    //静止时该值本来为零；目标末速度仍为零，不改通用规划器。
                    clearDistReAvgPara(pSPlanPointer);
                    ucPlanState[i] = handWheelAddNewPlan(pSPlanPointer, dAxisPulseToMm, pEncoder->dAxisCurrentSpeed[i],
                                                         0.0, pEncoder->dAxisCurrentSpeed[i], bIsRotationAxis);
                    if (pEncoder->dAxisCurrentSpeed[i] == 0.0 && pSPlanPointer->dPreDist == 0.0 &&
                        dAxisPulseToMm <= 8.0 * dDec * pSPlanPointer->dTc * pSPlanPointer->dTc)
                    {
                        /* 仅零速起步且距离很短时用对称三角形，避免原取整路径终点往复。 */
                        if (handwheelPrepareShortMove(pSPlanPointer, dAxisPulseToMm, dDec, dVmax) < 0)
                        {
                            parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
                            return;
                        }
                    }
                    /* 普通长段每拍重规划，所以总是执行本次新计划的第 1 拍。 */
                    dDeltaL[i] = nextstepPlanningAndDistReAvg(1, &pSChannel->SSingleAxisArray[i].SPlanData);
                }
                /*
				 * HW-6：SPlan 给出正的累计行程；加回原起点并乘锁定方向，
				 * 再走原坐标变换和 outPutAbsPoint，不能把累计行程当增量再累加。
				 */
                STargetPoint.dArray[i] =
                    pEncoder->SStartPoint.dArray[i] + cDir * dDeltaL[i];   //STargetPoint = SStartPoint + cDir * dDeltaL
                setbit(ucValStatus, i);                                    // 设置当前轴本拍输出有效
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE // 倾斜 Y 轴机床运动学转换
                if (pSChannel->bUseVirtualY == TRUE)
                {
                    if (i == AXIS_X)
                    {
                        //X1轴实际移动量（机床坐标:倾斜坐标系）
                        STargetPoint.dArray[i] =
                            STargetPoint.dArray[i] + 1.0 * STargetPoint.dArray[AXIS_Y] * pSChannel->dSINAngleYY1;
                    }

                    if (i == AXIS_Y)
                    {
                        //Y1轴移动量（机床坐标:倾斜坐标系）
                        STargetPoint.dArray[i] = STargetPoint.dArray[i] / pSChannel->dCOSAngleYY1;

                        //X1轴移动量（机床坐标:倾斜坐标系）
                        setbit(ucValStatus, AXIS_X);
                        STargetPoint.dArray[AXIS_X] = pEncoder->SStartPoint.dArray[AXIS_X] +
                                                      1.0 * STargetPoint.dArray[AXIS_Y] * pSChannel->dSINAngleYY1;
                    }
                }
#endif
            }
        }
        /*
		 * 原输出函数完成指令坐标/脉冲换算，后面的 PWM/总线输出链不在此改动。
		 * 成功仅表示软件接受本次指令，不表示伺服已到位或机械已经停稳。
		 */
        if (!outPutAbsPoint(&pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, pMapAxisArray,
                            pSChannel->SG01.dPulseToMm, &ucValStatus))
            return;

        /* ==================== [HW-7] 周期末测速与零速停稳换段 ==================== */
        for (i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (pSChannel->pMapAxisArray[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != getbit(ucValStatus, i))
                continue; //映射为空的物理轴或旋转轴  不计算
            if (ucPlanState[i] == 0)
                continue; //坐标变换带动的从轴不消费本轴手轮计划

            pSPlanPointer = &pSChannel->SSingleAxisArray[i].SPlanData;
            /* HW-7：记录本拍末边界速度，下一拍用它计算刹停边界。 */
            pEncoder->dAxisCurrentSpeed[i] = handwheelBoundarySpeed(
                pSPlanPointer, (pSPlanPointer->unCurrentT > 0 && pSPlanPointer->unCurrentT <= pSPlanPointer->unT5)
                                   ? pSPlanPointer->unCurrentT
                                   : 1);
            pSPlanPointer->dCurLeftDist = max(0.0, pSPlanPointer->dTargetDist - dDeltaL[i]);
            //普通段已到零速也立即换起点，不能留到下一拍收新输入后才清旧段。
            if (!pEncoder->bHandwheelBraking[i] && pEncoder->dAxisCurrentSpeed[i] == 0.0)
                ucPlanState[i] = 3;

            if (ucPlanState[i] == 3)
            {
                double oldStart = pEncoder->SStartPoint.dArray[i];
                double signedGoal = pEncoder->dHandwheelNetDist[i];
                /*
				 * 制动时 net 的基准是停点；先加回 dir*stop 得到相对旧起点的目标。
				 * 普通段的 net 本来就相对旧起点，不加 stop。
				 */
                if (pEncoder->bHandwheelBraking[i])
                    signedGoal += pEncoder->cHandwheelMoveDir[i] * pEncoder->dHandwheelStopDist[i];

                pEncoder->bHandwheelBraking[i] = FALSE;
                pEncoder->dAxisCurrentSpeed[i] = 0.0;
                clearSPlanData(&pSChannel->SSingleAxisArray[i].SPlanData);
                getSingleAxisRealTimePoint((double*)&pEncoder->SStartPoint.dArray[i], i, pMapAxisArray);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE // 倾斜 Y 轴机床起点逆变换
                if (pSChannel->bUseVirtualY == TRUE)
                {
                    if (i == AXIS_X)
                    {
                        double dY1;
                        getSingleAxisRealTimePoint(&dY1, AXIS_Y, pSChannel->pMapAxisArray);
                        pEncoder->SStartPoint.dArray[AXIS_X] -= 1.0 * dY1 * pSChannel->dSINAngleYY1;
                    }
                    if (i == AXIS_Y)
                    {
                        pEncoder->SStartPoint.dArray[AXIS_Y] =
                            pEncoder->SStartPoint.dArray[AXIS_Y] * pSChannel->dCOSAngleYY1;
                    }
                }
#endif

                /*
				 * 换基准但不丢目标：
				 *   绝对目标 = oldStart + signedGoal
				 *   新 net   = 绝对目标 - 新 SStartPoint
				 * 新起点取软件已量化的指令点，因此可保留脉冲量化尾差。
				 * 这里只修正指令量化，不能修正伺服跟随误差。
				 * 本轴本拍循环已经结束，下一拍才按新 net 的正负从零速启动。
				 */
                pEncoder->dHandwheelNetDist[i] = signedGoal - (pEncoder->SStartPoint.dArray[i] - oldStart);
                pEncoder->dHandwheelStopDist[i] = 0.0;
                pEncoder->cHandwheelMoveDir[i] = 0; // 解除方向锁定，允许下一拍重新定方向
            }
        }
    }
}
#else
////全局变量 企业增加 2023/12/20
//double  cdAxisPulseToMm[2];
//double  bdAxisPulseToMm[2];
//double tDistance;
//手轮有速度规划
void getHandWheelData(struct SChannel* pSChannel)
{
    int16 sSendValue; //脉冲变量，其定义变化为手轮理论脉冲->电机理论脉冲->电机理论脉冲(变化后取整)
    struct SEncoder* pEncoder;
    pEncoder = pSChannel->pSHandleWheelEncoder;
    uint8 ucAXISNum = g_SEncoder[HANDEWHEEL_ENCODER - 1].AXISNum;
    uint8* pMapAxisArray = pSChannel->pMapAxisArray;
    int i = 0;                    //循环次数
    double dDeltaL[AXIS_NUM_MAX]; //各个轴的位移增量
    VAL_STRUCT_TYPE ucValStatus = 0;

    double dRealPoint; //获取实时点 2024/12/17洪冠鹏
    getSingleAxisRealTimePoint(&dRealPoint, ucAXISNum, pMapAxisArray);

    double dRatioFactor; //脉冲当量比例系数 2025/1/11 洪冠鹏

    if (ucAXISNum > AXIS_NUM_MAX || pMapAxisArray[ucAXISNum] >= AXIS_PHYSICS_NUM_MAX || pEncoder == NULL)
    {
        return;
    }

    //触发手轮软限位后限制该方向的运动
    if (pEncoder->cEncoderDir == INP_GO_BACK && g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAXISNum]].soft_neg == 1 &&
        dRealPoint < g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAXISNum]]
                         .neg_soft) //开启负限位检测：当前点已超出限位点 不允许继续执行 2024/12/17
    {
        return;
    }
    if (pEncoder->cEncoderDir == INP_GO_AHEAD && g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAXISNum]].soft_pos == 1 &&
        dRealPoint > g_SSystemPara.MOTOR_WARN[pMapAxisArray[ucAXISNum]]
                         .pos_soft) //开启正限位检测：当前点已超出限位点 不允许继续执行 2024/12/17
    {
        return;
    }
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (pSChannel->bUseVirtualY == TRUE)
    {
        double dX1Mach; //X1轴机床坐标
        getSingleAxisRealTimePoint(&dX1Mach, AXIS_X, pMapAxisArray);
        if (pEncoder->cEncoderDir == INP_GO_BACK && ucAXISNum == AXIS_Y &&
            g_SSystemPara.MOTOR_WARN[pMapAxisArray[AXIS_X]].soft_neg == 1 &&
            dX1Mach < g_SSystemPara.MOTOR_WARN[pMapAxisArray[AXIS_X]].neg_soft) //摇动Y轴时，发现X1已经触发限位
        {
            return;
        }
        if (pEncoder->cEncoderDir == INP_GO_AHEAD && ucAXISNum == AXIS_Y &&
            g_SSystemPara.MOTOR_WARN[pMapAxisArray[AXIS_X]].soft_pos == 1 &&
            dX1Mach > g_SSystemPara.MOTOR_WARN[pMapAxisArray[AXIS_X]].pos_soft) //摇动Y轴时，发现X1已经触发限位
        {
            return;
        }
    }
#endif

    //得到去频化后的手轮脉冲
    if (abs(pEncoder->usDelta) % 4 != 0)
    {
        pEncoder->dDelataAccmulate += pEncoder->usDelta;
        pEncoder->usDelta = 0;
        if (abs(pEncoder->dDelataAccmulate) >= 4)
        {
            pEncoder->usDelta = pEncoder->dDelataAccmulate - (pEncoder->dDelataAccmulate % 4);
            pEncoder->dDelataAccmulate -= pEncoder->usDelta;
        }
    }
    //获取手轮旋转对应的脉冲数
    sSendValue = pEncoder->usDelta / 4; //手轮理论脉冲=手轮反馈的脉冲/4，4为四倍率
    sSendValue =
        pEncoder->cEncoderDir * sSendValue * pEncoder->ucWheelMultiple; //电机理论脉冲=编码器方向*手轮理论脉冲*倍率

    if (g_SSystemPara.SAxisPara[pMapAxisArray[ucAXISNum]].ucAxisType == LINEAR_AXIS) //直线轴 2025/6/13 熊骏 洪冠鹏
    {
        dRatioFactor = g_SSystemPara.SAxisPara[pMapAxisArray[ucAXISNum]].usMmToPulse /
                       1000.0; //脉冲比例系数（保证1倍下，手轮转动一格就是0.001mm）
        if (getShowByDiameter(g_SChannel[CHANNEL_NUM_MAX - 1].pMapAxisArray[ucAXISNum])) //直径显示
        {
            dRatioFactor /= 2.0; //直径显示相当于发脉冲数除以2
        }
        else //半径显示
        {
        }
    }
    else //旋转轴
    {
        dRatioFactor = g_SSystemPara.SAxisPara[pMapAxisArray[ucAXISNum]].usMmToPulse /
                       360000.0; //脉冲比例系数（保证1倍下，手轮转动一格就是0.001°，此处为转动一格为dRatioFactor个脉冲）
    }
    double
        dIdealPulse; //电机理论脉冲，由于脉冲当量比例系数可能存在小数，导致计算的电机理论脉冲存在小数，小数部分需要累计，避免脉冲丢失
    dIdealPulse = sSendValue * dRatioFactor;
    dIdealPulse += pEncoder->dAxisAccmulateRemainder[ucAXISNum];
    sSendValue = trunc(dIdealPulse);                                         //取整数部分
    pEncoder->dAxisAccmulateRemainder[ucAXISNum] = dIdealPulse - sSendValue; //取小数部分

    //虚拟轴的映射不为空
    if (pMapAxisArray[ucAXISNum] != AXIS_NULL)
    {
        double dAxisPulseToMm = 0; //运动轴脉冲数对应的距离
        SPointDouble STargetPoint; //当前点的理论绝对坐标
        struct SPlan* pSPlanPointer =
            &pSChannel->SSingleAxisArray[ucAXISNum].SPlanData; //手轮所控制运动轴的速度规划结构体
        double dPreDist = 0;                                   //已插补长度
        dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;
        uint8 ucPlanState[AXIS_NUM_MAX] = {0}; //速度规划状态标志位

        double dVmax = 0;         //手轮的最大速度
        double dDec = 0;          //手轮的最大减速度
        double dSlowDownDistance; //最大减速距离
        double dSurplusDistance;  //多余未走距离
        double dSurplusPulse;     //多余未走距离对应脉冲数
        bool bIsRotationAxis;     //是否为旋转轴

        int8 cVector = sgn(pEncoder->sAxisAccmulate[ucAXISNum] + sSendValue);
        int8 cVectorBack = sgn(sSendValue);

        //手轮有拨动
        if (sSendValue != 0)
        {
            //手轮方向发生变化
            if (cVector != cVectorBack)
            {
                pEncoder->sPulseForChangeDie[ucAXISNum] += sSendValue; //换向时的脉冲缓冲（数值不为0即为换向标志位）
            }
            else
            {
                pEncoder->sAxisAccmulate[ucAXISNum] += sSendValue; //虚拟轴
            }
        }

        //SPointDouble SRealTimePoint;  //获取当前实时点
        //getRealTimePoint((double*)&SRealTimePoint, AXIS_NUM_MAX, pMapAxisArray, NULL);  //获取实时坐标
        getRealTimePoint((double*)&STargetPoint, AXIS_NUM_MAX, pMapAxisArray, NULL); //获取实时坐标

        //由于可能存在多轴运动的情况，所以得这么处理
        for (i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (AXIS_NULL == pMapAxisArray[ucAXISNum])
                continue;   //轴映射为空 跳过处理 2024/10/31
            dDeltaL[i] = 0; //初始化各个轴的位移增量
            pSPlanPointer = &pSChannel->SSingleAxisArray[i].SPlanData;
            //STargetPoint.dArray[i] = 0; //初始化各个轴目标坐标
            pSChannel->SRun.sAxisDeltaPulse[i] = 0; //初始化脉冲

            //运动轴的存在未规划的脉冲数
            if (pEncoder->sAxisAccmulate[i] != 0)
            {
                int8 cDir = sgn(pEncoder->sAxisAccmulate[i]); //规划轴插补的方向
                pSChannel->SG00.bIsUserData =
                    TRUE; //手轮发生转动，转动数据来自用户输入，据此判断恢是否执行复加工  2023/12/14 洪冠鹏

                dAxisPulseToMm =
                    fabs(pEncoder->sAxisAccmulate[i] *
                         g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm); //运动轴所要运动的脉冲对应的插补距离

                //企业增加 2023/12/20 新增程序起始位置 限制手轮停下后的脉冲数
                bIsRotationAxis = g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS ? FALSE : TRUE;
                // dVmax = min(g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVmax, g_SSystemPara.dHandWheelVmax);//2025/3/11限制手轮最大速度
                dVmax =
                    g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS
                        ? g_SSystemPara.dHandWheelVmax
                        : g_SSystemPara.dHandWheelVmax *
                              DEGREE_OF_CIRCLE; //手轮的最大速度   dHandWheelVmax的单位：mm/ms（r/ms） 计算旋转轴时统一单位°/ms
                dVmax = min(g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVmax,
                            dVmax); //2025/3/11限制手轮最大速度  2025/9/23修改设置最大速度位置
                dDec =
                    g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS
                        ? g_SSystemPara.dHandWheelAcc
                        : g_SSystemPara.dHandWheelAcc *
                              DEGREE_OF_CIRCLE; //手轮的最大减速度 dHandWheelAcc的单位：mm/ms2（r/ms2） 计算旋转轴时统一单位°/ms2
                if (dDec <= 0)
                {
                    return;
                }
                //实时计算最大的减速距离，用于手轮停下后限制脉冲数
                dSlowDownDistance = fabs(dVmax * dVmax - g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe *
                                                             g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe) /
                                    2.0 / dDec;
                if (dSlowDownDistance <
                    g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm) //减速距离小于一个脉冲对应的位移量
                {
                    dSlowDownDistance =
                        g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm * 2; //预留多一个脉冲的位移量
                }
                else
                {
                    dSlowDownDistance =
                        ((int)((fabs(dSlowDownDistance) / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm) + 1)) *
                        g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm; //预留多一个脉冲的位移量
                }
                if (g_SSystemPara.dhandwheelcontrol == 1) //手轮停下后的脉冲数限制功能开启
                {
                    if (fabs(dAxisPulseToMm - (pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist)) >
                        dSlowDownDistance) //剩余未走距离大于最大减速距离时，舍去减速距离之外未走的距离
                    {
                        dSurplusDistance =
                            (int)(fabs((dAxisPulseToMm - (pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist) -
                                        dSlowDownDistance)));
                        dSurplusPulse = g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS
                                            ? dSurplusDistance * g_SSystemPara.SAxisPara[pMapAxisArray[i]].usMmToPulse
                                            : dSurplusDistance * g_SSystemPara.SAxisPara[pMapAxisArray[i]].usMmToPulse /
                                                  g_SSystemPara.SAxisPara[pMapAxisArray[i]]
                                                      .fDistPerCircle; //将多余未走距离转化为脉冲数
                        if (pEncoder->sAxisAccmulate[i] > 0)
                        {
                            pEncoder->sAxisAccmulate[i] -= dSurplusPulse;
                        }
                        else
                        {
                            pEncoder->sAxisAccmulate[i] += dSurplusPulse;
                        }
                        dAxisPulseToMm =
                            fabs(pEncoder->sAxisAccmulate[i] * g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm);
                    }
                }
                //企业增加 2023/12/20 新增程序终点位置

                //进行手轮控制速度规划
                pSChannel->SSingleAxisArray[i].SPlanData.cInterpDir =
                    INP_GO_AHEAD; //冠鹏 陈敏聪 2023/12/11	插补方向恒为正，保证速度规划输出距离恒为正，实际输出的方向由cDir决定
                pSChannel->SSingleAxisArray[i].SPlanData.dVmax =
                    g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == LINEAR_AXIS ? dVmax
                                                                                        : dVmax / DEGREE_OF_CIRCLE;
                ; //2025/9/23修改设置最大速度位置  //2026.3.17避免旋转轴手轮dVmax二次乘DEGREE_OF_CIRCLE导致限速抬高与超速风险
                // pSChannel->SSingleAxisArray[i].SPlanData.dVmax = min(g_SSystemPara.dHandWheelVmax, g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVmax);//2025/3/11限制手轮最大速度
                ucPlanState[i] = handWheelAddNewPlan(&pSChannel->SSingleAxisArray[i].SPlanData, dAxisPulseToMm,
                                                     g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVs,
                                                     g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe,
                                                     pEncoder->dAxisCurrentSpeed[i], bIsRotationAxis);
                dDeltaL[i] =
                    nextstepPlanningAndDistReAvg(1, &pSChannel->SSingleAxisArray[i].SPlanData); //计算出轴的位移增量
                STargetPoint.dArray[i] = pEncoder->SStartPoint.dArray[i] + cDir * dDeltaL[i];   //计算理论插补坐标
                setbit(ucValStatus, i);                                                         //设置有效轴
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
                if (pSChannel->bUseVirtualY == TRUE)
                {
                    if (i == AXIS_X) //X轴方向运动
                    {
                        //X1轴实际移动量（机床坐标:倾斜坐标系）
                        STargetPoint.dArray[i] =
                            STargetPoint.dArray[i] + 1.0 * STargetPoint.dArray[AXIS_Y] * pSChannel->dSINAngleYY1;
                    }

                    if (i == AXIS_Y) //Y轴方向运动（绝对坐标:直角坐标系）
                    {
                        //Y1轴移动量（机床坐标:倾斜坐标系）
                        STargetPoint.dArray[i] = STargetPoint.dArray[i] / pSChannel->dCOSAngleYY1;

                        //X1轴移动量（机床坐标:倾斜坐标系）
                        setbit(ucValStatus, AXIS_X); //设置X1轴有效
                        STargetPoint.dArray[AXIS_X] = pEncoder->SStartPoint.dArray[AXIS_X] +
                                                      1.0 * STargetPoint.dArray[AXIS_Y] * pSChannel->dSINAngleYY1;
                    }
                }
#endif
            }
        }
        outPutAbsPoint(&pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray,
                       pSChannel->SG01.dPulseToMm, &ucValStatus); //利用理论坐标生成SRun结构体数据（未完成）

        for (int i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (pSChannel->pMapAxisArray[i] >= AXIS_PHYSICS_NUM_MAX)
                continue; //映射为空的物理轴或旋转轴,不设置有效轴
            if (pEncoder->dAxisAccmulateRemainder[i] != 0)
            {
                setbit(ucValStatus, i); //设置有效轴
            }
        }

        for (i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (pSChannel->pMapAxisArray[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != (getbit(ucValStatus, i)))
                continue; //映射为空的物理轴或旋转轴  不计算

            //如果当前留下未移动的距离大于等于0
            if (pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist > 0 ||
                isZero(pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist))
            {
                //计算当前理论速度
                pEncoder->dAxisCurrentSpeed[i] =
                    fabs(pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist -
                         (pSChannel->SSingleAxisArray[i].SPlanData.dTargetDist - dDeltaL[i])) /
                    pSChannel->SSingleAxisArray[i].SPlanData.dTc;
                pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist =
                    pSChannel->SSingleAxisArray[i].SPlanData.dTargetDist - dDeltaL[i]; //计算理论未走距离
            }
            else
            {
                pEncoder->dAxisCurrentSpeed[i] = 0; //当前速度为0
            }

            if (ucPlanState[i] == 3)
            {
                pSChannel->SSingleAxisArray[i].SPlanData.dDistance = 0;    //长轴的插补长度
                pSChannel->SSingleAxisArray[i].SPlanData.dPreDist = 0;     //已插补长度
                pSChannel->SSingleAxisArray[i].SPlanData.dTargetDist = 0;  //目标距离无总长度限制
                pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist = 0; //计算剩余距离（规划用)
                pEncoder->sAxisAccmulate[i] = 0;

                //换向还未移动的距离
                if (pEncoder->sPulseForChangeDie[i] != 0)
                {
                    pEncoder->sAxisAccmulate[i] += pEncoder->sPulseForChangeDie[i];
                    pEncoder->sPulseForChangeDie[i] = 0;
                }
                getSingleAxisRealTimePoint((double*)&pEncoder->SStartPoint.dArray[i], i,
                                           pMapAxisArray); //2025.5.14 熊骏

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
                if (pSChannel->bUseVirtualY == TRUE)
                {
                    if (i == AXIS_X)
                    {
                        double dY1;
                        getSingleAxisRealTimePoint(&dY1, AXIS_Y, pSChannel->pMapAxisArray); //当前Y1轴的位置
                        pEncoder->SStartPoint.dArray[AXIS_X] -=
                            1.0 * dY1 * pSChannel->dSINAngleYY1; //将倾斜坐标系的点转化为直角坐标系的点
                    }
                    if (i == AXIS_Y)
                    {
                        pEncoder->SStartPoint.dArray[AXIS_Y] =
                            pEncoder->SStartPoint.dArray[AXIS_Y] *
                            pSChannel->dCOSAngleYY1; //将倾斜坐标系的点转化为直角坐标系的点
                    }
                }
#endif
            }
        }

        for (i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (ucPlanState[i] != 0 || pEncoder->dAxisAccmulateRemainder[i] == 0)
                continue;
            if (pSChannel->pMapAxisArray[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != (getbit(ucValStatus, i)))
                continue; //映射为空的物理轴或旋转轴  不计算
            pEncoder->dPulseAccmulate[i] += pEncoder->dAxisAccmulateRemainder[i];
            g_dRealTimePointArray[pMapAxisArray[i]] +=
                pEncoder->dAxisAccmulateRemainder[i] *
                g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm;                                //赋值到实时坐标
            getSingleAxisRealTimePoint((double*)&pEncoder->SStartPoint.dArray[i], i, pMapAxisArray); //2025.5.14 熊骏

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
            if (pSChannel->bUseVirtualY == TRUE)
            {
                if (i == AXIS_X)
                {
                    double dY1;
                    getSingleAxisRealTimePoint(&dY1, AXIS_Y, pSChannel->pMapAxisArray); //当前Y1轴的位置
                    pEncoder->SStartPoint.dArray[AXIS_X] -=
                        1.0 * dY1 * pSChannel->dSINAngleYY1; //将倾斜坐标系的点转化为直角坐标系的点
                }
                if (i == AXIS_Y)
                {
                    pEncoder->SStartPoint.dArray[AXIS_Y] =
                        pEncoder->SStartPoint.dArray[AXIS_Y] *
                        pSChannel->dCOSAngleYY1; //将倾斜坐标系的点转化为直角坐标系的点
                }
            }
#endif
            pEncoder->dAxisAccmulateRemainder[i] = 0;
            sSendValue = 0;

            if (fabs(pEncoder->dPulseAccmulate[i]) >= 1) //存储够一个脉冲
            {
                sSendValue = trunc(pEncoder->dPulseAccmulate[i]); //取整数部分
                pEncoder->dPulseAccmulate[i] -= sSendValue;       //存储部分减去整数部分（即将发出）

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
                if (pSChannel->bUseVirtualY == TRUE && i == AXIS_Y)
                {
                    int16 sX1DeltaPulse;
                    int16 sY1DeltaPulse;

                    sX1DeltaPulse = round(
                        1.0 * pEncoder->dPulseAccmulate[i] * g_SSystemPara.SAxisPara[pMapAxisArray[AXIS_Y]].dPulseToMm *
                        pSChannel->dTANAngleYY1 * g_SSystemPara.SAxisPara[pMapAxisArray[AXIS_X]].usMmToPulse);
                    sY1DeltaPulse = round(pEncoder->dPulseAccmulate[i] / pSChannel->dCOSAngleYY1);

                    g_dRealTimePointArray[pMapAxisArray[AXIS_X]] +=
                        sX1DeltaPulse * g_SSystemPara.SAxisPara[pMapAxisArray[AXIS_X]].dPulseToMm; //赋值到实时坐标
                    g_dRealTimePointArray[pMapAxisArray[AXIS_Y]] +=
                        sY1DeltaPulse * g_SSystemPara.SAxisPara[pMapAxisArray[AXIS_Y]].dPulseToMm; //赋值到实时坐标
                }
#endif

                pSChannel->SRun.sAxisDeltaPulse[i] += sSendValue; //位移增量加入整数部分
                g_dRealTimePointArray[pMapAxisArray[i]] +=
                    sSendValue * g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm; //赋值到实时坐标
            }
            //pSChannel->SRun.sAxisDeltaPulse[i] += sSendValue;
        }
    }
}

#endif /* HANDWHEEL_STOP_BOUNDARY_ENABLE */
