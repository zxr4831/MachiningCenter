#include "Gcode.h"
#include "GCodeCommandAPI.h"
#include "RoundGeometry.h"

/* 自动倒圆适配层：四种组合共用有限轮廓求解器。
 * 坐标与半径均为真实毫米；非成功返回不修改输出，不操作解析状态或环形缓冲。
 */
/*************************************************
  函数名称：static RGCurve convertLineToRoundCurve(const SLine *line)
  函数功能：将项目直线结构映射为几何内核的二维直线。
  输入参数：line 已完成坐标归一化的非空直线。
  输出参数：无。
  返 回 值：RGCurve，dir 为 0。
  备    注：内核坐标顺序为 (Z,X)；这里不进行直径到半径或单位换算。
**************************************************/
static RGCurve convertLineToRoundCurve(const SLine* line)
{
    RGCurve c = {0};
    c.start = makeRoundPoint(line->SStartPoint.dArray[AXIS_Z], line->SStartPoint.dArray[AXIS_X]);
    c.end = makeRoundPoint(line->SEndPoint.dArray[AXIS_Z], line->SEndPoint.dArray[AXIS_X]);
    return c;
}
/*************************************************
  函数名称：static RGCurve convertArcToRoundCurve(const SArc *arc)
  函数功能：将项目圆弧结构映射为几何内核的有向圆弧。
  输入参数：arc 非空圆弧，方向已由公共入口校验。
  输出参数：无。
  返 回 值：包含起终点、圆心、半径和方向的 RGCurve。
  备    注：项目 ARC_CW=1、ARC_ANTI_CW=-1；内核相反，因此方向取负。
**************************************************/
static RGCurve convertArcToRoundCurve(const SArc* arc)
{
    RGCurve c = {0};
    c.start = makeRoundPoint(arc->SStartPoint.dArray[AXIS_Z], arc->SStartPoint.dArray[AXIS_X]);
    c.end = makeRoundPoint(arc->SEndPoint.dArray[AXIS_Z], arc->SEndPoint.dArray[AXIS_X]);
    c.center = makeRoundPoint(arc->SCenterPoint.dArray[AXIS_Z], arc->SCenterPoint.dArray[AXIS_X]);
    c.radius = arc->dRadius;
    /* 坐标轴序保持 (Z,X)，只转换项目与内核相反的圆弧方向编码。 */
    c.dir = -arc->cArcDir;
    return c;
}
/*************************************************
  函数名称：static int8 calcRoundArc(const RGCurve *a, const RGCurve *b, double radius, const SPointDouble *base, SArc *out)
  函数功能：调用统一求解器并将成功结果转换成项目 SArc。
  输入参数：a、b 内核曲线；radius 真实半径；base 非空连接点。
  输出参数：out 仅在返回 1 时写入完整圆角结构。
  返 回 值：1 成功；0 同向共线不变；-1 无解；-2 非法输入。
  备    注：非平面轴从 base 继承；先构造临时结果，失败时保持 out 不变。
**************************************************/
static int8 calcRoundArc(const RGCurve* a, const RGCurve* b, double radius, const SPointDouble* base, SArc* out)
{
    RGResult result;
    SArc temp = {0};
    int status;
    if (!out)
        return RG_INVALID;
    status = solveRoundGeometry(a, b, radius, &result);
    if (status != RG_OK)
        return (int8)status;
    temp.EntityType = ARC;
    /* 先继承全部轴坐标，再覆盖平面分量，避免非平面轴残留未初始化值。 */
    temp.SStartPoint = temp.SEndPoint = temp.SCenterPoint = *base;
    temp.SStartPoint.dArray[AXIS_Z] = result.first.x;
    temp.SStartPoint.dArray[AXIS_X] = result.first.y;
    temp.SEndPoint.dArray[AXIS_Z] = result.second.x;
    temp.SEndPoint.dArray[AXIS_X] = result.second.y;
    temp.SCenterPoint.dArray[AXIS_Z] = result.center.x;
    temp.SCenterPoint.dArray[AXIS_X] = result.center.y;
    temp.dRadius = result.radius;
    temp.cArcDir = (int8)-result.dir;
    *out = temp;
    return RG_OK;
}
/*************************************************
  函数名称：int8 calAngleRoundedInfoInLineAndLine(SLine* a, SLine* b, double r, SArc* out)
  函数功能：计算直线接直线的自动圆角。
  输入参数：a 前直线；b 后直线；r 真实毫米半径。
  输出参数：out 成功时返回切点、圆心、半径及项目方向编码。
  返 回 值：1 成功；0 仅同向共线直线不变；-1 无解；-2 非法输入。
  备    注：只处理 G18 的有限轮廓几何；模态与边界由上层检查，非成功时不修改 out。
**************************************************/
int8 calAngleRoundedInfoInLineAndLine(SLine* a, SLine* b, double r, SArc* out)
{
    RGCurve ca, cb;
    if (!a || !b)
        return RG_INVALID;
    ca = convertLineToRoundCurve(a);
    cb = convertLineToRoundCurve(b);
    return calcRoundArc(&ca, &cb, r, &a->SEndPoint, out);
}
/*************************************************
  函数名称：int8 calAngleRoundedInfoInLineAndArc(SLine* a, SArc* b, double r, SArc* out)
  函数功能：计算直线接圆弧的自动圆角。
  输入参数：a 前直线；b 后圆弧；r 真实毫米半径。
  输出参数：out 成功时返回切点、圆心、半径及项目方向编码。
  返 回 值：1 成功；0 仅同向共线直线不变；-1 无解；-2 非法输入。
  备    注：只处理 G18 的有限轮廓几何；模态与边界由上层检查，非成功时不修改 out。
**************************************************/
int8 calAngleRoundedInfoInLineAndArc(SLine* a, SArc* b, double r, SArc* out)
{
    RGCurve ca, cb;
    if (!a || !b || (b->cArcDir != ARC_CW && b->cArcDir != ARC_ANTI_CW))
        return RG_INVALID;
    ca = convertLineToRoundCurve(a);
    cb = convertArcToRoundCurve(b);
    return calcRoundArc(&ca, &cb, r, &a->SEndPoint, out);
}
/*************************************************
  函数名称：int8 calAngleRoundedInfoInArcAndLine(SArc* a, SLine* b, double r, SArc* out)
  函数功能：计算圆弧接直线的自动圆角。
  输入参数：a 前圆弧；b 后直线；r 真实毫米半径。
  输出参数：out 成功时返回切点、圆心、半径及项目方向编码。
  返 回 值：1 成功；0 仅同向共线直线不变；-1 无解；-2 非法输入。
  备    注：只处理 G18 的有限轮廓几何；模态与边界由上层检查，非成功时不修改 out。
**************************************************/
int8 calAngleRoundedInfoInArcAndLine(SArc* a, SLine* b, double r, SArc* out)
{
    RGCurve ca, cb;
    if (!a || !b || (a->cArcDir != ARC_CW && a->cArcDir != ARC_ANTI_CW))
        return RG_INVALID;
    ca = convertArcToRoundCurve(a);
    cb = convertLineToRoundCurve(b);
    return calcRoundArc(&ca, &cb, r, &a->SEndPoint, out);
}
/*************************************************
  函数名称：int8 calAngleRoundedInfoInArcAndArc(SArc* a, SArc* b, double r, SArc* out)
  函数功能：计算圆弧接圆弧的自动圆角。
  输入参数：a 前圆弧；b 后圆弧；r 真实毫米半径。
  输出参数：out 成功时返回切点、圆心、半径及项目方向编码。
  返 回 值：1 成功；0 仅同向共线直线不变；-1 无解；-2 非法输入。
  备    注：只处理 G18 的有限轮廓几何；模态与边界由上层检查，非成功时不修改 out。
**************************************************/
int8 calAngleRoundedInfoInArcAndArc(SArc* a, SArc* b, double r, SArc* out)
{
    RGCurve ca, cb;
    if (!a || !b || (a->cArcDir != ARC_CW && a->cArcDir != ARC_ANTI_CW) ||
        (b->cArcDir != ARC_CW && b->cArcDir != ARC_ANTI_CW))
        return RG_INVALID;
    ca = convertArcToRoundCurve(a);
    cb = convertArcToRoundCurve(b);
    return calcRoundArc(&ca, &cb, r, &a->SEndPoint, out);
}
