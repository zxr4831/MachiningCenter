#ifndef ROUND_GEOMETRY_H
#define ROUND_GEOMETRY_H

#include <math.h>
#include <float.h>

/* 纯几何计算。G18 平面按 (Z,X) 轴序表示。
 * dir 为 +1 表示逆时针，-1 表示顺时针，0 表示直线。
 * 拒绝整圆、裁剪后剩余长度为零及圆弧相切退化情况。
 */
#define RG_PI 3.14159265358979323846
#define RG_TOL 1.0e-6
#define RG_ANGLE_TOL 1.0e-9
/* 状态值与公共适配接口共用；仅 RG_OK 状态写入求解结果。 */
enum { RG_INVALID = -2, RG_NO_SOLUTION = -1, RG_UNCHANGED = 0, RG_OK = 1 };
typedef struct { double x, y; } RGPoint;
typedef struct {
    RGPoint start, end, center;
    double radius;
    int dir;
} RGCurve;
typedef struct {
    RGPoint first, second, center;
    double radius;
    int dir;
} RGResult;
typedef struct {
    RGPoint origin, unit;
    double radius, signedRadius;
    int circle;
} RGLocus;

/*************************************************
  函数名称：static RGPoint makeRoundPoint(double x, double y)
  函数功能：构造局部平面坐标点。
  输入参数：x 对应 Z，y 对应 X，单位为实际毫米。
  输出参数：无。
  返 回 值：包含 x、y 分量的坐标点。
  备    注：不进行轴映射或直径值换算。
**************************************************/
static RGPoint makeRoundPoint(double x, double y)
{
    RGPoint p; p.x = x; p.y = y; return p;
}
/*************************************************
  函数名称：static RGPoint addRoundPoints(RGPoint a, RGPoint b)
  函数功能：计算两个平面向量的和。
  输入参数：a、b：平面向量。
  输出参数：无。
  返 回 值：向量和 a + b。
  备    注：也可用于按给定向量平移坐标点。
**************************************************/
static RGPoint addRoundPoints(RGPoint a, RGPoint b) { return makeRoundPoint(a.x+b.x, a.y+b.y); }
/*************************************************
  函数名称：static RGPoint subtractRoundPoints(RGPoint a, RGPoint b)
  函数功能：计算两个平面点或向量的差。
  输入参数：a、b：平面点或向量。
  输出参数：无。
  返 回 值：向量差 a - b。
  备    注：输入为坐标点时，结果为从 b 指向 a 的向量。
**************************************************/
static RGPoint subtractRoundPoints(RGPoint a, RGPoint b) { return makeRoundPoint(a.x-b.x, a.y-b.y); }
/*************************************************
  函数名称：static RGPoint scaleRoundPoint(RGPoint a, double s)
  函数功能：按比例缩放平面向量。
  输入参数：a：平面向量；s：有符号缩放系数。
  输出参数：无。
  返 回 值：缩放后的向量 s * a。
  备    注：不对结果进行归一化或有效性检查。
**************************************************/
static RGPoint scaleRoundPoint(RGPoint a, double s) { return makeRoundPoint(a.x*s, a.y*s); }
/*************************************************
  函数名称：static double calcRoundDotProduct(RGPoint a, RGPoint b)
  函数功能：计算平面向量的点积。
  输入参数：a、b：平面向量。
  输出参数：无。
  返 回 值：点积标量。
  备    注：输入为单位向量时，结果为两向量夹角的余弦值。
**************************************************/
static double calcRoundDotProduct(RGPoint a, RGPoint b) { return a.x*b.x + a.y*b.y; }
/*************************************************
  函数名称：static double calcRoundCrossProduct(RGPoint a, RGPoint b)
  函数功能：计算平面向量的有符号叉积。
  输入参数：a、b：按 (Z,X) 轴序表示的平面向量。
  输出参数：无。
  返 回 值：叉积标量 a.x*b.y - a.y*b.x。
  备    注：结果为正表示从 a 转向 b 时左转。
**************************************************/
static double calcRoundCrossProduct(RGPoint a, RGPoint b) { return a.x*b.y - a.y*b.x; }
/*************************************************
  函数名称：static double calcRoundVectorLength(RGPoint a)
  函数功能：计算向量的欧氏长度。
  输入参数：a：平面向量。
  输出参数：无。
  返 回 值：输入有限且运算未溢出时，返回非负长度。
  备    注：调用方另行检查几何数据是否为有限数。
**************************************************/
static double calcRoundVectorLength(RGPoint a) { return sqrt(calcRoundDotProduct(a,a)); }
/*************************************************
  函数名称：static RGPoint rotateRoundVectorLeft(RGPoint a)
  函数功能：将平面向量逆时针旋转 90 度。
  输入参数：a：按 (Z,X) 轴序表示的平面向量。
  输出参数：无。
  返 回 值：左法向量 (-a.y,a.x)。
  备    注：保持原向量长度，不进行归一化。
**************************************************/
static RGPoint rotateRoundVectorLeft(RGPoint a) { return makeRoundPoint(-a.y,a.x); }
/*************************************************
  函数名称：static int isRoundValueFinite(double x)
  函数功能：检查标量是否为有限数。
  输入参数：x：待检查标量。
  输出参数：无。
  返 回 值：1：有限数；0：非有限数。
  备    注：排除非数值 NaN 和正、负无穷大。
**************************************************/
static int isRoundValueFinite(double x) { return x == x && fabs(x) <= DBL_MAX; }
/*************************************************
  函数名称：static int isRoundPointFinite(RGPoint p)
  函数功能：检查坐标点的两个分量是否均为有限数。
  输入参数：p：局部平面坐标点。
  输出参数：无。
  返 回 值：1：两个分量均有限；0：至少一个分量非有限。
  备    注：不检查坐标点是否位于指定轮廓上。
**************************************************/
static int isRoundPointFinite(RGPoint p) { return isRoundValueFinite(p.x) && isRoundValueFinite(p.y); }
/*************************************************
  函数名称：static double calcRoundSweepAngle(RGPoint from, RGPoint to, int dir)
  函数功能：计算沿指定方向的非负转角。
  输入参数：from、to：起止径向向量；dir：+1 逆时针，-1 顺时针。
  输出参数：无。
  返 回 值：范围为 [0,2*pi) 的转角，单位为弧度。
  备    注：两个向量同向时返回零，不按整圆处理。
**************************************************/
static double calcRoundSweepAngle(RGPoint from, RGPoint to, int dir)
{
    double a = atan2(dir * calcRoundCrossProduct(from,to), calcRoundDotProduct(from,to));
    if (a < 0.0) a += 2.0 * RG_PI;
    return a;
}
/*************************************************
  函数名称：static int isRoundCurveValid(const RGCurve *c)
  函数功能：检查原始轮廓是否为有效的非零有限几何。
  输入参数：c：非空轮廓指针；dir 为 0 表示直线，+1 逆时针，-1 顺时针。
  输出参数：无。
  返 回 值：1：有效；0：无效。
  备    注：拒绝起终点重合、整圆以及起终点半径不一致的圆弧。
**************************************************/
static int isRoundCurveValid(const RGCurve *c)
{
    double r1, r2;
    if (!isRoundPointFinite(c->start) || !isRoundPointFinite(c->end)) return 0;
    if (calcRoundVectorLength(subtractRoundPoints(c->end,c->start)) <= RG_TOL) return 0;
    if (!c->dir) return 1;
    if ((c->dir != 1 && c->dir != -1) || !isRoundPointFinite(c->center)
        || !isRoundValueFinite(c->radius) || c->radius <= RG_TOL) return 0;
    r1 = calcRoundVectorLength(subtractRoundPoints(c->start,c->center));
    r2 = calcRoundVectorLength(subtractRoundPoints(c->end,c->center));
    return fabs(r1-c->radius) <= RG_TOL * (1.0+c->radius)
        && fabs(r2-c->radius) <= RG_TOL * (1.0+c->radius);
}
/*************************************************
  函数名称：static RGPoint calcRoundUnitTangent(const RGCurve *c, RGPoint p)
  函数功能：计算沿运行方向的单位切向量。
  输入参数：c：有效非零轮廓；p：位于该轮廓上的切点。
  输出参数：无。
  返 回 值：沿运行方向的单位切向量。
  备    注：调用前须验证 c；圆弧切向由 c->dir 决定。
**************************************************/
static RGPoint calcRoundUnitTangent(const RGCurve *c, RGPoint p)
{
    RGPoint d = c->dir ? scaleRoundPoint(rotateRoundVectorLeft(subtractRoundPoints(p,c->center)),c->dir)
                      : subtractRoundPoints(c->end,c->start);
    return scaleRoundPoint(d,1.0/calcRoundVectorLength(d));
}
/*************************************************
  函数名称：static double calcRoundCurveLength(const RGCurve *c)
  函数功能：计算有限轮廓沿运行方向的长度。
  输入参数：c：有效非零直线或有向圆弧。
  输出参数：无。
  返 回 值：轮廓长度，单位为实际毫米。
  备    注：圆弧长度由半径乘有向扫角得到，不使用弦长。
**************************************************/
static double calcRoundCurveLength(const RGCurve *c)
{
    return c->dir ? c->radius * calcRoundSweepAngle(subtractRoundPoints(c->start,c->center),
                                       subtractRoundPoints(c->end,c->center),c->dir)
                  : calcRoundVectorLength(subtractRoundPoints(c->end,c->start));
}
/*************************************************
  函数名称：static int calcRoundCurvePosition(const RGCurve *c, RGPoint p, double *s)
  函数功能：计算切点在有限轮廓上的运行位置。
  输入参数：c：有效轮廓；p：候选切点。
  输出参数：s：从起点沿运行方向到切点的距离，失败时也可能被修改。
  返 回 值：1：切点在容差范围内属于有限轮廓；0：不属于。
  备    注：仅在返回成功后，才可将 s 用作有效位置。
**************************************************/
static int calcRoundCurvePosition(const RGCurve *c, RGPoint p, double *s)
{
    if (c->dir) {
        if (fabs(calcRoundVectorLength(subtractRoundPoints(p,c->center))-c->radius) >
            RG_TOL*(1.0+c->radius)) return 0;
        *s = c->radius * calcRoundSweepAngle(subtractRoundPoints(c->start,c->center),
                                subtractRoundPoints(p,c->center),c->dir);
    } else {
        RGPoint u = calcRoundUnitTangent(c,c->start), d = subtractRoundPoints(p,c->start);
        if (fabs(calcRoundCrossProduct(u,d)) > RG_TOL) return 0;
        *s = calcRoundDotProduct(u,d);
    }
    return isRoundValueFinite(*s) && *s >= -RG_TOL && *s <= calcRoundCurveLength(c)+RG_TOL;
}
/*************************************************
  函数名称：static int buildRoundCenterLocus(const RGCurve *c, double r, int side, RGLocus *l)
  函数功能：构造圆角候选圆心所在的偏置轨迹。
  输入参数：c：有效轮廓；r：圆角半径；side：+1 左侧，-1 右侧。
  输出参数：l：偏置直线或带有符号半径的偏置圆。
  返 回 值：1：偏置轨迹可用；0：偏置圆退化为点。
  备    注：保留 signedRadius，用于恢复正确的内切或外切切点。
**************************************************/
static int buildRoundCenterLocus(const RGCurve *c, double r, int side, RGLocus *l)
{
    l->circle = c->dir != 0;
    if (l->circle) {
        l->origin = c->center;
        /* 当圆角半径 R 大于原圆半径时，有符号半径用于保留正确的相切分支。 */
        l->signedRadius = c->radius - side*c->dir*r;
        l->radius = fabs(l->signedRadius);
        l->unit = makeRoundPoint(0,0);
        return l->radius > RG_TOL;
    }
    l->unit = calcRoundUnitTangent(c,c->start);
    l->origin = addRoundPoints(c->start,scaleRoundPoint(rotateRoundVectorLeft(l->unit),side*r));
    l->radius = l->signedRadius = 0.0;
    return 1;
}
/*************************************************
  函数名称：static int calcRoundLocusIntersections(const RGLocus *a, const RGLocus *b, RGPoint out[2])
  函数功能：计算两条无限延伸的圆心偏置轨迹的交点。
  输入参数：a、b：有效偏置轨迹。
  输出参数：out：最多两个候选圆心，仅返回数量以内的元素有效。
  返 回 值：候选圆心数量：0、1 或 2。
  备    注：本函数不检查原轮廓的有限范围，由 solveRoundGeometry 筛选候选解。
**************************************************/
static int calcRoundLocusIntersections(const RGLocus *a, const RGLocus *b, RGPoint out[2])
{
    RGPoint d = subtractRoundPoints(b->origin,a->origin), base, u;
    double den, t, h2, dist, along, tol;
    /* 圆心接近时切点仍可能相距很远，以下仅精确重根合并。 */
    if (!a->circle && !b->circle) {
        den = calcRoundCrossProduct(a->unit,b->unit);
        if (fabs(den) <= RG_ANGLE_TOL) return 0;
        out[0] = addRoundPoints(a->origin,scaleRoundPoint(a->unit,calcRoundCrossProduct(d,b->unit)/den));
        return 1;
    }
    if (a->circle && !b->circle) return calcRoundLocusIntersections(b,a,out);
    if (!a->circle) {
        t = calcRoundDotProduct(d,a->unit);
        base = addRoundPoints(a->origin,scaleRoundPoint(a->unit,t));
        h2 = b->radius*b->radius - calcRoundDotProduct(subtractRoundPoints(base,b->origin),subtractRoundPoints(base,b->origin));
        tol = RG_TOL*(1.0+b->radius*b->radius);
        if (h2 < -tol) return 0;
        h2 = sqrt(h2 > 0.0 ? h2 : 0.0);
        out[0] = addRoundPoints(base,scaleRoundPoint(a->unit,h2));
        out[1] = subtractRoundPoints(base,scaleRoundPoint(a->unit,h2));
        return h2 == 0.0 ? 1 : 2;
    }
    dist = calcRoundVectorLength(d);
    if (dist <= RG_TOL || dist > a->radius+b->radius+RG_TOL
        || dist < fabs(a->radius-b->radius)-RG_TOL) return 0;
    u = scaleRoundPoint(d,1.0/dist);
    along = (a->radius*a->radius-b->radius*b->radius+dist*dist)/(2.0*dist);
    h2 = a->radius*a->radius-along*along;
    tol = RG_TOL*(1.0+a->radius*a->radius);
    if (h2 < -tol) return 0;
    h2 = sqrt(h2 > 0.0 ? h2 : 0.0);
    base = addRoundPoints(a->origin,scaleRoundPoint(u,along));
    out[0] = addRoundPoints(base,scaleRoundPoint(rotateRoundVectorLeft(u),h2));
    out[1] = subtractRoundPoints(base,scaleRoundPoint(rotateRoundVectorLeft(u),h2));
    return h2 == 0.0 ? 1 : 2;
}
/*************************************************
  函数名称：static RGPoint calcRoundContactPoint(const RGCurve *c, const RGLocus *l, RGPoint center)
  函数功能：由候选圆心恢复原轮廓上的切点。
  输入参数：c：原轮廓；l：对应的非退化偏置轨迹；center：候选圆心。
  输出参数：无。
  返 回 值：原直线或原圆上的切点。
  备    注：使用有符号半径区分内切和外切分支。
**************************************************/
static RGPoint calcRoundContactPoint(const RGCurve *c, const RGLocus *l, RGPoint center)
{
    if (c->dir)
        return addRoundPoints(c->center,scaleRoundPoint(subtractRoundPoints(center,c->center),c->radius/l->signedRadius));
    return addRoundPoints(c->start,scaleRoundPoint(l->unit,calcRoundDotProduct(subtractRoundPoints(center,c->start),l->unit)));
}
/*************************************************
  函数名称：static int solveRoundGeometry(const RGCurve *a, const RGCurve *b, double radius, RGResult *out)
  函数功能：求解两条有限轮廓之间相切的小圆弧过渡。
  输入参数：a、b：首尾相连的轮廓；radius：正的圆角半径，单位为实际毫米。
  输出参数：out：切点、圆心、半径和方向，仅在返回 RG_OK 时写入。
  返 回 值：1：求解成功；0：同向共线直线不处理；-1：无解；-2：输入非法。
  备    注：要求两侧保留正长度；不修改缓冲区、模态或机床状态。
**************************************************/
static int solveRoundGeometry(const RGCurve *a, const RGCurve *b, double radius, RGResult *out)
{
    RGPoint ta, tb, centers[2];
    RGLocus la, lb;
    RGResult best;
    double cross, dot, trim = 0.0, bestScore = DBL_MAX;
    int side, count, i, linePair, found = 0;
    if (!a || !b || !out || !isRoundCurveValid(a) || !isRoundCurveValid(b)
        || !isRoundValueFinite(radius) || radius <= RG_TOL
        || calcRoundVectorLength(subtractRoundPoints(a->end,b->start)) > RG_TOL) return RG_INVALID;
    ta = calcRoundUnitTangent(a,a->end); tb = calcRoundUnitTangent(b,b->start);
    cross = calcRoundCrossProduct(ta,tb); dot = calcRoundDotProduct(ta,tb);
    if (fabs(cross) <= RG_ANGLE_TOL) {
        /* 仅对同向共线的两条直线明确采用不生成圆角的处理规则。 */
        if (!a->dir && !b->dir && dot > 0.0) return RG_UNCHANGED;
        return RG_NO_SOLUTION;
    }
    /* 使用运行切向判断圆角所在侧，不能用起终点之间的弦方向代替。 */
    side = cross > 0.0 ? 1 : -1;
    if (!buildRoundCenterLocus(a,radius,side,&la) || !buildRoundCenterLocus(b,radius,side,&lb))
        return RG_NO_SOLUTION;
    /* 完全共端点的两直线使用稳定半角式；容差内有缝隙时仍按原支撑线求交。 */
    linePair = !a->dir && !b->dir
        && a->end.x == b->start.x && a->end.y == b->start.y;
    if (linePair) {
        /* 小转角避免 1-cos 的消减；近折返避免 1+cos 的消减。 */
        trim = radius * (dot >= 0.0 ? fabs(cross)/(1.0+dot) : (1.0-dot)/fabs(cross));
        if (!isRoundValueFinite(trim)) return RG_NO_SOLUTION;
        centers[0] = addRoundPoints(subtractRoundPoints(a->end,scaleRoundPoint(ta,trim)),
                                   scaleRoundPoint(rotateRoundVectorLeft(ta),side*radius));
        count = 1;
    } else {
        count = calcRoundLocusIntersections(&la,&lb,centers);
    }
    /* 偏置轨迹交点仅为候选圆心，仍须检查两侧切点是否属于有限轮廓。 */
    for (i=0; i<count; ++i) {
        RGPoint p = linePair ? subtractRoundPoints(a->end,scaleRoundPoint(ta,trim))
                            : calcRoundContactPoint(a,&la,centers[i]);
        RGPoint q = linePair ? addRoundPoints(b->start,scaleRoundPoint(tb,trim))
                            : calcRoundContactPoint(b,&lb,centers[i]);
        RGPoint rp = subtractRoundPoints(p,centers[i]), rq = subtractRoundPoints(q,centers[i]);
        double sa, sb, score, sweep, rpLength, rqLength;
        if (!isRoundPointFinite(p) || !isRoundPointFinite(q) || !isRoundPointFinite(centers[i])) continue;
        rpLength = calcRoundVectorLength(rp); rqLength = calcRoundVectorLength(rq);
        if (!isRoundValueFinite(rpLength) || !isRoundValueFinite(rqLength)
            || rpLength <= RG_TOL || rqLength <= RG_TOL) continue;
        if (fabs(rpLength-radius) > RG_TOL*(1.0+radius)
            || fabs(rqLength-radius) > RG_TOL*(1.0+radius)) continue;
        if (!calcRoundCurvePosition(a,p,&sa) || !calcRoundCurvePosition(b,q,&sb)) continue;
        /* 两侧原轮廓均须保留正长度；连续倒圆时也必须满足此条件。 */
        if (sa <= 2*RG_TOL || sa >= calcRoundCurveLength(a)-2*RG_TOL
            || sb <= 2*RG_TOL || sb >= calcRoundCurveLength(b)-2*RG_TOL
            || calcRoundVectorLength(subtractRoundPoints(p,q)) <= 2*RG_TOL) continue;
        /* 半径误差已独立检查；切向按实际模长归一化，不能混入请求半径误差。 */
        if (calcRoundDotProduct(calcRoundUnitTangent(a,p),scaleRoundPoint(rotateRoundVectorLeft(rp),side/rpLength)) < 1.0-1.0e-7
            || calcRoundDotProduct(calcRoundUnitTangent(b,q),scaleRoundPoint(rotateRoundVectorLeft(rq),side/rqLength)) < 1.0-1.0e-7) continue;
        sweep = calcRoundSweepAngle(rp,rq,side);
        if (sweep <= RG_ANGLE_TOL || sweep > RG_PI+RG_ANGLE_TOL) continue;
        /* 优先选择两侧原轮廓裁剪总长度最小的有效局部解。 */
        score = calcRoundCurveLength(a)-sa+sb;
        if (score < bestScore) {
            best.first=p; best.second=q; best.center=centers[i];
            best.radius=radius; best.dir=side;
            bestScore=score; found=1;
        }
    }
    if (!found) return RG_NO_SOLUTION;
    /* 全部检查通过后才写入结果；失败或无需处理时保持调用方输出不变。 */
    *out = best;
    return RG_OK;
}
#endif
