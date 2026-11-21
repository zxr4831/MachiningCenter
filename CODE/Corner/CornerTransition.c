#include "Gcode.h"
#include "CornerTransition.h"
#include "GCodeCommandAPI.h"
#include "RoundGeometry.h"

/* 无持久状态；这里只计算相邻两段，不访问缓冲、报警或释放输入资源。 */
/*************************************************
  函数名称：static bool isCornerMotionSupported(const struct SGCodeParse *p)
  函数功能：检查单段是否满足当前倒圆版本的运动和附带指令限制。
  输入参数：p 已转换模态的缓冲记录，不能为空。
  输出参数：无。
  返 回 值：TRUE 支持；FALSE 存在不支持的模态、附带动作或坐标。
  备    注：限定 G18/G21/G40、普通 G1/G2/G3 和非平面轴不动；几何可解性另行检查。
**************************************************/
static bool isCornerMotionSupported(const struct SGCodeParse *p)
{
    int i;
    if (p->SModalInfo.sGa < 1 || p->SModalInfo.sGa > 3
        || (p->EntityType != L1NE && p->EntityType != ARC)
        || p->SModalInfo.sGc != G18 || p->SModalInfo.sGd != G40
        || p->SModalInfo.sGe != G21 || p->cInterpolationState != 0
        || p->pSG50Offset || p->cPointType || p->bG65SystemMacroWait
        || p->unDelayTime || p->bG31Flag || p->cTwistCommand
        || p->bChipBreakFlag || p->cToolAxisMod
        || (p->unCommandType & CMD_PROGRAM_ADDR)
        || p->sChannelCommand.ucType != UNASSIGNED_VALUE)
        return FALSE;
    for (i=0; i<IO_MAXGROUP_IN_SINGLE_LINE; ++i)
        if (p->ucIOCommandArray[i]) return FALSE;
    for (i=0; i<AXIS_NUM_MAX; ++i) {
        if (!isRoundValueFinite(p->SStartPoint.dArray[i]) || !isRoundValueFinite(p->SEndPoint.dArray[i]))
            return FALSE;
        if (i != AXIS_Z && i != AXIS_X
            && fabs(p->SStartPoint.dArray[i]-p->SEndPoint.dArray[i]) > RG_TOL)
            return FALSE;
    }
    return TRUE;
}
/*************************************************
  函数名称：bool isCornerPairCompatible(const struct SGCodeParse *a, const struct SGCodeParse *b)
  函数功能：检查前后段是否连续且允许在连接处生成倒角或圆角。
  输入参数：a 前段、b 后段，均为已转换模态的非空记录。
  输出参数：无。
  返 回 值：TRUE 可继续求解；FALSE 存在边界或不连续。
  备    注：检查刀具、相关模态及主轴状态；不允许越过前段结束标志。
**************************************************/
bool isCornerPairCompatible(const struct SGCodeParse *a, const struct SGCodeParse *b)
{
    int i;
    if (!isCornerMotionSupported(a) || !isCornerMotionSupported(b) || a->bLastLineMark
        || a->ucToolID != b->ucToolID || a->ucToolCompNum != b->ucToolCompNum
        || a->ucTurretNum != b->ucTurretNum
        || a->SModalInfo.sGb != b->SModalInfo.sGb
        || a->SModalInfo.sGf != b->SModalInfo.sGf
        || a->SModalInfo.sGg != b->SModalInfo.sGg
        || a->dSpindleSpeed != b->dSpindleSpeed
        || a->dSecondSpindleSpeed != b->dSecondSpindleSpeed)
        return FALSE;
    for (i=0; i<AXIS_NUM_MAX; ++i)
        if (fabs(a->SEndPoint.dArray[i]-b->SStartPoint.dArray[i]) > RG_TOL)
            return FALSE;
    return TRUE;
}
/*************************************************
  函数名称：static int8 calcCornerRoundArc(const struct SGCodeParse *a, const struct SGCodeParse *b, double r, SArc *out)
  函数功能：按两段实体类型分发四种自动倒圆计算。
  输入参数：a 前段；b 后段；r 真实毫米半径，记录不能为空。
  输出参数：out 仅在成功时写入过渡圆弧。
  返 回 值：1 成功；0 同向共线无需倒圆；-1 无解；-2 非法输入。
  备    注：只构造几何输入，不裁剪原记录，不推进缓冲指针。
**************************************************/
static int8 calcCornerRoundArc(const struct SGCodeParse *a, const struct SGCodeParse *b,
                              double r, SArc *out)
{
    SLine la = {0}, lb = {0};
    SArc aa = {0}, ab = {0};
    la.EntityType=lb.EntityType=L1NE;
    la.SStartPoint=a->SStartPoint; la.SEndPoint=a->SEndPoint;
    lb.SStartPoint=b->SStartPoint; lb.SEndPoint=b->SEndPoint;
    aa.EntityType=ab.EntityType=ARC;
    aa.SStartPoint=a->SStartPoint; aa.SEndPoint=a->SEndPoint;
    aa.SCenterPoint=a->SCenterPoint; aa.dRadius=a->dRadius; aa.cArcDir=a->cArcDir;
    ab.SStartPoint=b->SStartPoint; ab.SEndPoint=b->SEndPoint;
    ab.SCenterPoint=b->SCenterPoint; ab.dRadius=b->dRadius; ab.cArcDir=b->cArcDir;
    if (a->EntityType == L1NE && b->EntityType == L1NE)
        return calAngleRoundedInfoInLineAndLine(&la,&lb,r,out);
    if (a->EntityType == L1NE && b->EntityType == ARC)
        return calAngleRoundedInfoInLineAndArc(&la,&ab,r,out);
    if (a->EntityType == ARC && b->EntityType == L1NE)
        return calAngleRoundedInfoInArcAndLine(&aa,&lb,r,out);
    if (a->EntityType == ARC && b->EntityType == ARC)
        return calAngleRoundedInfoInArcAndArc(&aa,&ab,r,out);
    return RG_INVALID;
}
/*************************************************
  函数名称：static void updateCornerBounds(struct SGCodeParse *p)
  函数功能：重算裁剪段或新增段在 G18 平面中的坐标极值。
  输入参数：p 几何已定型的直线或圆弧记录，不能为空。
  输出参数：p 的 dHorMax/dHorMin/dVerMax/dVerMin。
  返 回 值：无。
  备    注：横轴为 Z、纵轴为 X；圆弧不能只用起终点估计极值。
**************************************************/
static void updateCornerBounds(struct SGCodeParse *p)
{
    if (p->EntityType == ARC) {
        SGetAxisMaxAndMin bounds = {0};
        bounds.cArcDir=p->cArcDir; bounds.dRadius=p->dRadius;
        bounds.SStartPoint=&p->SStartPoint; bounds.SEndPoint=&p->SEndPoint;
        bounds.SCenterPoint=&p->SCenterPoint;
        bounds.ucHor=AXIS_Z; bounds.ucVer=AXIS_X;
        getAxisMaxAndMin(&bounds);
        p->dHorMax=bounds.dHorMax; p->dHorMin=bounds.dHorMin;
        p->dVerMax=bounds.dVerMax; p->dVerMin=bounds.dVerMin;
    } else {
        p->dHorMax=fmax(p->SStartPoint.dArray[AXIS_Z],p->SEndPoint.dArray[AXIS_Z]);
        p->dHorMin=fmin(p->SStartPoint.dArray[AXIS_Z],p->SEndPoint.dArray[AXIS_Z]);
        p->dVerMax=fmax(p->SStartPoint.dArray[AXIS_X],p->SEndPoint.dArray[AXIS_X]);
        p->dVerMin=fmin(p->SStartPoint.dArray[AXIS_X],p->SEndPoint.dArray[AXIS_X]);
    }
}
/*************************************************
  函数名称：static void buildCornerTransition(struct SGCodeParse *out, const struct SGCodeParse *owner, const SArc *arc)
  函数功能：从求解结果构造完整的新增过渡运动记录。
  输入参数：owner 请求所属前段；arc 求解结果，均不能为空。
  输出参数：out 全新记录，继承前段进给、刀具和来源信息。
  返 回 值：无。
  备    注：out 必须是不持有待释放资源的新对象；不复制一次性动作、结束标志和 D/L 请求。
**************************************************/
static void buildCornerTransition(struct SGCodeParse *out, const struct SGCodeParse *owner,
                             const SArc *arc)
{
    /* 新建局部对象只清零，不能调用会释放旧指针的槽位初始化接口。 */
    memset(out,0,sizeof(*out));
    out->sChannelCommand.ucType=UNASSIGNED_VALUE;
    out->SModalInfo=owner->SModalInfo;
    out->SModalInfo.pSGaRegulation=NULL;
    out->SModalInfo.sGa=owner->dRoundedRadius > 0 ? (arc->cArcDir == ARC_CW ? G02 : G03) : G01;
    out->EntityType=owner->dRoundedRadius > 0 ? ARC : L1NE;
    out->SStartPoint=arc->SStartPoint; out->SEndPoint=arc->SEndPoint;
    out->SCenterPoint=arc->SCenterPoint; out->dRadius=arc->dRadius;
    out->cArcDir=arc->cArcDir;
    out->ucToolID=owner->ucToolID; out->ucToolCompNum=owner->ucToolCompNum;
    out->ucTurretNum=owner->ucTurretNum; out->ucTurretAxis=owner->ucTurretAxis;
    out->dFeedrate=owner->dFeedrate;
    out->dSpindleSpeed=owner->dSpindleSpeed;
    out->dSecondSpindleSpeed=owner->dSecondSpindleSpeed;
    out->dLimitedSpindleSpeed=owner->dLimitedSpindleSpeed;
    out->unGCodeLineNum=owner->unGCodeLineNum;
    memcpy(out->cFilename,owner->cFilename,sizeof(out->cFilename));
    refreshGCodeGeometry(out);
    updateCornerBounds(out);
}
/*************************************************
  函数名称：static int8 calcLineLineChamfer(const struct SGCodeParse *a, const struct SGCodeParse *b, SArc *geometry)
  函数功能：计算两直线之间按等距退让约定生成的 L 倒角。
  输入参数：a 前段，dChamferLen 为连接直线长度；b 后段，均不能为空。
  输出参数：geometry 成功时写入两个裁剪点，其余字段清零。
  返 回 值：1 成功；0 同向共线不变；-1 无解；-2 非法实体或零长度。
  备    注：调用前已验证正 L 和段兼容性；失败或无需倒角时不写 geometry。
**************************************************/
static int8 
calcLineLineChamfer(const struct SGCodeParse *a, const struct SGCodeParse *b,
                             SArc *geometry)
{
    RGPoint u, v, p, t1, t2;
    double l1,l2,dot,trim;
    if (a->EntityType != L1NE || b->EntityType != L1NE) return RG_INVALID;
    p=makeRoundPoint(a->SEndPoint.dArray[AXIS_Z],a->SEndPoint.dArray[AXIS_X]);
    u=subtractRoundPoints(p,makeRoundPoint(a->SStartPoint.dArray[AXIS_Z],a->SStartPoint.dArray[AXIS_X]));
    v=subtractRoundPoints(makeRoundPoint(b->SEndPoint.dArray[AXIS_Z],b->SEndPoint.dArray[AXIS_X]),p);
    l1=calcRoundVectorLength(u); l2=calcRoundVectorLength(v);
    if (l1 <= RG_TOL || l2 <= RG_TOL) return RG_INVALID;
    u=scaleRoundPoint(u,1.0/l1); v=scaleRoundPoint(v,1.0/l2); dot=calcRoundDotProduct(u,v);
    if (fabs(calcRoundCrossProduct(u,v)) <= RG_ANGLE_TOL)
        return dot > 0 ? RG_UNCHANGED : RG_NO_SOLUTION;
    /* L 是连接直线的实际长度；两边等距退让量为 L / (2*cos(转角/2))。 */
    trim=a->dChamferLen/sqrt(2.0+2.0*dot);
    if (!isRoundValueFinite(trim) || trim >= l1-RG_TOL || trim >= l2-RG_TOL)
        return RG_NO_SOLUTION;
    t1=subtractRoundPoints(p,scaleRoundPoint(u,trim)); t2=addRoundPoints(p,scaleRoundPoint(v,trim));
    memset(geometry,0,sizeof(*geometry));
    geometry->SStartPoint=geometry->SEndPoint=a->SEndPoint;
    geometry->SStartPoint.dArray[AXIS_Z]=t1.x;
    geometry->SStartPoint.dArray[AXIS_X]=t1.y;
    geometry->SEndPoint.dArray[AXIS_Z]=t2.x;
    geometry->SEndPoint.dArray[AXIS_X]=t2.y;
    return RG_OK;
}

/*************************************************
  函数名称：bool isCornerRequestValid(const struct SGCodeParse *pSegment)
  函数功能：校验已转换段上的 D/L 参数及倒角适用条件。
  输入参数：pSegment 已转换模态的段；空指针判为非法。
  输出参数：无。
  返 回 值：TRUE 合法；FALSE 参数、运动或结束标志不支持。
  备    注：D/L 均为零允许通过；是否存在请求由调用方判断。
**************************************************/
bool isCornerRequestValid(const struct SGCodeParse *pSegment)
{
    return pSegment && isCornerMotionSupported(pSegment) && !pSegment->bLastLineMark
        && isRoundValueFinite(pSegment->dRoundedRadius)
        && pSegment->dRoundedRadius >= 0 && pSegment->dRoundedRadius <= 1000
        && isRoundValueFinite(pSegment->dChamferLen)
        && pSegment->dChamferLen >= 0 && pSegment->dChamferLen <= 1000
        && !(pSegment->dRoundedRadius > 0 && pSegment->dChamferLen > 0);
}

/*************************************************
  函数名称：int8 calcCornerTransition(...)
  函数功能：计算两段连接处的 D 倒圆或 L 倒角，构造裁剪段和过渡段。
  输入参数：pPrevSegment、pNextSegment 为已转换模态的段，前段须有正 D 或 L。
  输出参数：pTrimmedPrev、pTransition、pTrimmedNext，仅返回 1 时写入。
  返 回 值：1 成功；0 同向共线无需过渡；-1 无解；-2 非法或不支持。
  备    注：输入和输出对象不得重叠，输出不得持有待释放资源。
            输出中的原段指针只是借用；仅缓冲提交方转移所有权。
            不转换模态、不修改输入、不报警、不推进指针，不分配或释放内存。
**************************************************/
int8 calcCornerTransition(const struct SGCodeParse *pPrevSegment,
    const struct SGCodeParse *pNextSegment, struct SGCodeParse *pTrimmedPrev,
    struct SGCodeParse *pTransition, struct SGCodeParse *pTrimmedNext)
{
    SArc sArc = {0};
    int8 cStatus;
    //指针合法性校验（防重名别名）
    if (!pPrevSegment || !pNextSegment || !pTrimmedPrev || !pTransition || !pTrimmedNext
        || pPrevSegment == pNextSegment
        || pTrimmedPrev == pTransition || pTrimmedPrev == pTrimmedNext || pTransition == pTrimmedNext
        || pTrimmedPrev == pPrevSegment || pTrimmedPrev == pNextSegment
        || pTransition == pPrevSegment || pTransition == pNextSegment
        || pTrimmedNext == pPrevSegment || pTrimmedNext == pNextSegment)
        return RG_INVALID;

    //语义校验（请求是否合理 + 两段是否兼容）  
    if (!isCornerRequestValid(pPrevSegment)
        || !(pPrevSegment->dRoundedRadius > 0 || pPrevSegment->dChamferLen > 0)
        || !isCornerPairCompatible(pPrevSegment,pNextSegment)
        || ((pNextSegment->dRoundedRadius != 0 || pNextSegment->dChamferLen != 0)
            && !isCornerRequestValid(pNextSegment)))
        return RG_INVALID;

    //几何求解
    cStatus = pPrevSegment->dRoundedRadius > 0
        ? calcCornerRoundArc(pPrevSegment,pNextSegment,pPrevSegment->dRoundedRadius,&sArc)
        : calcLineLineChamfer(pPrevSegment,pNextSegment,&sArc);
    if (cStatus != RG_OK) return cStatus;

    // 全部可能失败的检查已完成，裁剪旧段，构造新段；以下只构造结果，不发布至全局缓冲。
    *pTrimmedPrev = *pPrevSegment;
    *pTrimmedNext = *pNextSegment;
    pTrimmedPrev->SEndPoint = sArc.SStartPoint;
    pTrimmedPrev->dRoundedRadius = pTrimmedPrev->dChamferLen = 0.0;
    pTrimmedNext->SStartPoint = sArc.SEndPoint;
    buildCornerTransition(pTransition,pPrevSegment,&sArc);
    //若不为倒圆，过渡段强制转为 G01 直线
    if (pPrevSegment->dRoundedRadius <= 0) {
        pTransition->EntityType=L1NE; pTransition->SModalInfo.sGa=G01;
        pTransition->cArcDir=0; pTransition->dRadius=0;
        refreshGCodeGeometry(pTransition);
        updateCornerBounds(pTransition);
    }
    //刷新几何缓存
    refreshGCodeGeometry(pTrimmedPrev); updateCornerBounds(pTrimmedPrev);
    refreshGCodeGeometry(pTrimmedNext); updateCornerBounds(pTrimmedNext);
    return RG_OK;
}
