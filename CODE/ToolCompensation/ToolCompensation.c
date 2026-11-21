#include "ToolCompensation.h"
#include "ringbuff.h"
#include "Config.h"
#include "Gcode.h"

#if STM32_MODE
#include "dy_link.h"
#endif

SSToolRunPoint SToolRunPoint;
//SSToolCompensationRecord SToolCompensationRecord//注释原因：企业要以动态内存的形式管理

SSToolCompensationRecord* SToolCompensationRecord = NULL; //存放刀补过程的中间点以及刀补的一些状态的结构体指针
static FILE *channelFile1, *channelFile2;
static uint32 lineNum1, lineNum2;

//阶跃函数
double Sgn(double dRadius)
{
    if (dRadius > 0)
        return 1;

    else if (dRadius < 0)
        return -1;

    return 0;
}

/*************************************************
  函数名称：struct SChannel* initSChannel()
  函数功能: 在动态内存中开辟存放刀补过程的中间点以及刀补的状态的结构体
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
SSToolCompensationRecord* initSToolCompensationRecord()
{
    SSToolCompensationRecord* pToolCompensationRecord =
        (SSToolCompensationRecord*)mallocGCode(sizeof(SSToolCompensationRecord), ORAM_NUM340);
    if (pToolCompensationRecord == NULL)
    {
        parseGCodeErrorB(9999, GCODE_ERROR_NUM406);
        return NULL;
    }
    return pToolCompensationRecord;
}

/*************************************************
  函数名称：getLineVector(SPointDouble &SP1, SPointDouble &SP2, SVector &LineVec)
  函数功能  利用起点和终点坐标来求直线矢量
  输入参数：SP1	起点坐标（绝对坐标）
			SP2	终点坐标（绝对坐标）
			LineVec 要求的直线矢量

  输出参数：LineVec		直线的方向矢量
  返 回 值：
  备    注：
**************************************************/
//求直线的方向矢量
void getLineVector(SPointDouble SPoint1, SPointDouble SPoint2, SVector* LineVec, uint8* ucAxisNum)
{
    if (LineVec == NULL)
    {
        return;
    }
    double denominator = sqrt(pow((SPoint2.dArray[ucAxisNum[0]] - SPoint1.dArray[ucAxisNum[0]]), 2) +
                              pow((SPoint2.dArray[ucAxisNum[1]] - SPoint1.dArray[ucAxisNum[1]]), 2));
    double denominator2 = sqrt(pow((SPoint2.dArray[ucAxisNum[0]] - SPoint1.dArray[ucAxisNum[0]]), 2) +
                               pow((SPoint2.dArray[ucAxisNum[1]] - SPoint1.dArray[ucAxisNum[1]]), 2));
    if (isZero(denominator) == TRUE || isZero(denominator2) == TRUE)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM132);
        return;
    }

    LineVec->dArray[ucAxisNum[0]] = (SPoint2.dArray[ucAxisNum[0]] - SPoint1.dArray[ucAxisNum[0]]) / denominator;
    LineVec->dArray[ucAxisNum[1]] = (SPoint2.dArray[ucAxisNum[1]] - SPoint1.dArray[ucAxisNum[1]]) / denominator2;
}

/*************************************************
  函数名称：getArcVectorAtStart(double dI, double dJ, SVector &ArcVec, int iArcDir)
  函数功能  利用指定的IK，圆弧方向求圆弧起点的方向矢量
  输入参数：dI          圆心相对于起点在x方向上的距离
			dJ          圆心相对于起点在y方向上的距离
			dK          圆心相对于起点在Z方向上的距离
			ArcVec      要求的圆弧矢量
			iArcDir     圆弧方向

  输出参数：ArcVec		圆弧在起点的方向矢量
  返 回 值：
  备    注：
**************************************************/
//求转接点在圆弧起点的方向矢量
void getArcVectorAtStart(double dK, double dI, SVector* SArcVec, int iArcDir, uint8* ucAxisNum)
{
    double dR = sqrt(pow(dK, 2) + pow(dI, 2));

    if (isZero(dR) == TRUE)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM132);
        return;
    }
    if (SArcVec == NULL)
    {
        return;
    }

    if (iArcDir == ARC_ANTI_CW)
        dR = -dR; //-1为逆圆弧
    SArcVec->dArray[ucAxisNum[0]] = dI / dR;
    SArcVec->dArray[ucAxisNum[1]] = -dK / dR;
}

/*************************************************
  函数名称：getArcVectorAtEnd(SPointDouble&SP1, SPointDouble&SP2, double dI, double dJ, SVector &ArcVec, int iArcDir)
  函数功能  利用指定的IK，起点坐标、终点坐标和圆弧方向求圆弧终点的方向矢量
  输入参数：SP1	起点坐标（绝对坐标）
			SP2	终点坐标（绝对坐标）
			dI          圆心相对于起点在x方向上的距离
			dJ          圆心相对于起点在y方向上的距离
			ArcVec      要求的圆弧矢量
			iArcDir     圆弧方向

  输出参数：
  返 回 值：ArcVec		圆弧在起点的方向矢量
  备    注：
**************************************************/
//求转接点在圆弧终点的方向矢量
void getArcVectorAtEnd(SPointDouble SPoint1, SPointDouble SPoint2, double dK, double dI, SVector* SArcVec, int iArcDir,
                       uint8* ucAxisNum)
{
    double dX_01, dZ_01, dR;
    if (SArcVec == NULL)
    {
        return;
    }
    dZ_01 = SPoint1.dArray[ucAxisNum[0]] - dK - SPoint2.dArray[ucAxisNum[0]];
    dX_01 = SPoint1.dArray[ucAxisNum[1]] - dI - SPoint2.dArray[ucAxisNum[1]];
    dR = sqrt(pow(dK, 2) + pow(dI, 2));
    if (isZero(dR) == TRUE)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM132);
        return;
    }
    if (iArcDir == ARC_ANTI_CW)
        dR = -dR; //-1为逆圆弧
    SArcVec->dArray[ucAxisNum[0]] = -dX_01 / dR;
    SArcVec->dArray[ucAxisNum[1]] = dZ_01 / dR;
}

/*************************************************
  函数名称：JudgeAngle(double dr, SVector Vec1, SVector Vec2)
  函数功能  根据刀补半径，两条轨迹的方向矢量判断转接角属于什么类型
  输入参数：dr          刀具半径
			Vec1        第一条轨迹的方向矢量
			Vec2		第二条轨迹的方向矢量

  输出参数：
  返 回 值：int	1代表缩短型、2代表伸长型、3代表插入型
  备    注：-1			返回代表出错
**************************************************/
//转接角类型的判断
int JudgeAngle(double dRadius, SVector SVec1, SVector SVec2, uint8* ucAxisNum)
{
    double dSin_a, dCos_a;
    dSin_a = -Sgn(dRadius) * (SVec2.dArray[ucAxisNum[1]] * SVec1.dArray[ucAxisNum[0]] -
                              SVec2.dArray[ucAxisNum[0]] * SVec1.dArray[ucAxisNum[1]]);
    dCos_a = -(SVec1.dArray[ucAxisNum[0]] * SVec2.dArray[ucAxisNum[0]] +
               SVec1.dArray[ucAxisNum[1]] * SVec2.dArray[ucAxisNum[1]]);
    if (dSin_a < 0 || isZero(dCos_a + 1)) //大于等于180度小于`360为缩短型
    {
        return 1;
    }
    else if (dSin_a > 0 && (dCos_a < 0 || isZero(dCos_a))) //大于等于90度小于180度为伸长型
    {
        return 2;
    }
    else if (dSin_a > 0 && (dCos_a > 0 || isZero(dCos_a))) //大于等于0小于90度为插入型
    {
        return 3;
    }
    return -1;
}

/*************************************************
  函数名称：SpecialAngleJudge(double dr, SVector Vec1, SVector Vec2)
  函数功能  根据刀具半径、两条轨迹的方向向量
  输入参数：dr			刀具半径
			Vec1		第一个轨迹的矢量
			Vec2		第一个轨迹的矢量

  输出参数：
  返 回 值：0\180		判断出的特殊角度数
  备    注：-1			返回代表出错
**************************************************/
//特殊角180或0度判断
int SpecialAngleJudge(double dRadius, SVector SVec1, SVector SVec2, uint8* ucAxisNum)
{
    double dSin_a, dCos_a;
    dSin_a = (-1) * Sgn(dRadius) *
             (SVec2.dArray[ucAxisNum[1]] * SVec1.dArray[ucAxisNum[0]] -
              SVec2.dArray[ucAxisNum[0]] * SVec1.dArray[ucAxisNum[1]]);
    dCos_a = (-1) * (SVec1.dArray[ucAxisNum[0]] * SVec2.dArray[ucAxisNum[0]] +
                     SVec1.dArray[ucAxisNum[1]] * SVec2.dArray[ucAxisNum[1]]);

    if (isZero(dCos_a + 1))
    {
        return 180;
    }

    else if (isZero(dCos_a - 1))
    {
        return 0;
    }

    return -1;
}

/*************************************************
  函数名称：offsetRadiusVector(double dToolRadius, SVector LineVec, SPointDouble &SP1, SPointDouble &SP2)
  函数功能  根据刀具半径、转接点以及方向矢量求出偏置一个刀具半径的刀补之后的补偿点
  输入参数：dToolRadius	刀具半径
			LineVec		直线矢量
			SP1			转接点
			SP2			用于记录刀补之后的补偿点

  输出参数：SP2			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
//求偏移一个半径矢量的转接点函数
void offsetRadiusVector(double dToolRadius, SVector SLineVec, SPointDouble SP1, SPointDouble* SP2, uint8* ucAxisNum)
{
    if (SP2 == NULL)
    {
        return;
    }
    SP2->dArray[ucAxisNum[0]] = SP1.dArray[ucAxisNum[0]] - dToolRadius * SLineVec.dArray[ucAxisNum[1]];
    SP2->dArray[ucAxisNum[1]] = SP1.dArray[ucAxisNum[1]] + dToolRadius * SLineVec.dArray[ucAxisNum[0]];
}

/*************************************************
  函数名称：Intersection_Of_Quidistant_Lines(double dToolRadius, SVector LineVec1, SVector LineVec2, SPointDouble &SP1, SPointDouble &SP2)
  函数功能  根据刀具半径、转接点以及两条直线的方向矢量求出刀补之后的补偿点
  输入参数：dToolRadius	刀具半径
			LineVec1	第一条直线矢量
			LineVec2	第二条直线矢量
			SP1			转接点
			SP2			用于记录刀补之后的补偿点

  输出参数：SP2			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
//求两等距线交点
void Intersection_Of_Quidistant_Lines(double dToolRadius, SVector LineVec1, SVector LineVec2, SPointDouble SP1,
                                      SPointDouble* SP2, uint8* ucAxisNum)
{
    if (SP2 == NULL)
    {
        return;
    }

    //防止分母为0，报错并返回
    if (isZero(LineVec1.dArray[ucAxisNum[0]] * LineVec2.dArray[ucAxisNum[1]] -
               LineVec2.dArray[ucAxisNum[0]] * LineVec1.dArray[ucAxisNum[1]]))
    {
        return;
    }

    SP2->dArray[ucAxisNum[0]] =
        SP1.dArray[ucAxisNum[0]] + dToolRadius * (LineVec2.dArray[ucAxisNum[0]] - LineVec1.dArray[ucAxisNum[0]]) /
                                       (LineVec1.dArray[ucAxisNum[0]] * LineVec2.dArray[ucAxisNum[1]] -
                                        LineVec2.dArray[ucAxisNum[0]] * LineVec1.dArray[ucAxisNum[1]]);
    SP2->dArray[ucAxisNum[1]] =
        SP1.dArray[ucAxisNum[1]] + dToolRadius * (LineVec2.dArray[ucAxisNum[1]] - LineVec1.dArray[ucAxisNum[1]]) /
                                       (LineVec1.dArray[ucAxisNum[0]] * LineVec2.dArray[ucAxisNum[1]] -
                                        LineVec2.dArray[ucAxisNum[0]] * LineVec1.dArray[ucAxisNum[1]]);
}

/*************************************************
  函数名称：offsetRadiusVector1(double dToolRadius, SVector LineVec, SPointDouble &SP1, SPointDouble &SP2)
  函数功能  计算插入型偏置点1
  输入参数：dToolRadius	刀具半径
			LineVec1	第一条直线矢量
			SP1			转接点
			SP2			用于记录刀补之后的补偿点

  输出参数：SP2			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
void offsetRadiusVector1(double dToolRadius, SVector LineVec, SPointDouble SP1, SPointDouble* SP2, uint8* ucAxisNum)
{
    if (SP2 == NULL)
    {
        return;
    }
    SP2->dArray[ucAxisNum[1]] = SP1.dArray[ucAxisNum[1]] + dToolRadius * LineVec.dArray[ucAxisNum[0]] +
                                fabs(dToolRadius) * LineVec.dArray[ucAxisNum[1]];
    SP2->dArray[ucAxisNum[0]] = SP1.dArray[ucAxisNum[0]] - dToolRadius * LineVec.dArray[ucAxisNum[1]] +
                                fabs(dToolRadius) * LineVec.dArray[ucAxisNum[0]];
}

/*************************************************
  函数名称：offsetRadiusVector2(double dToolRadius, SVector LineVec, SPointDouble &SP1, SPointDouble &SP2)
  函数功能  计算插入型偏置点2
  输入参数：dToolRadius	刀具半径
			LineVec1	第一条直线矢量
			SP1			转接点
			SP2			用于记录刀补之后的补偿点

  输出参数：SP2			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
void offsetRadiusVector2(double dToolRadius, SVector LineVec, SPointDouble SP1, SPointDouble* SP2, uint8* ucAxisNum)
{
    if (SP2 == NULL)
    {
        return;
    }
    SP2->dArray[ucAxisNum[1]] = SP1.dArray[ucAxisNum[1]] + dToolRadius * LineVec.dArray[ucAxisNum[0]] -
                                fabs(dToolRadius) * LineVec.dArray[ucAxisNum[1]];
    SP2->dArray[ucAxisNum[0]] = SP1.dArray[ucAxisNum[0]] - dToolRadius * LineVec.dArray[ucAxisNum[1]] -
                                fabs(dToolRadius) * LineVec.dArray[ucAxisNum[0]];
}

/*************************************************
  函数名称：Intersection_Of_Line_And_Arc(SPointDouble &SP1, SVector LineVec, double dI, double dJ, SPointDouble &SP2)
  函数功能  根据转接点、直线的方向矢量、圆弧的IK值求出刀补之后的补偿点
  输入参数：SP1			转接点
			LineVec		直线矢量
			dI			圆弧圆心与起点在x方向的距离
			dJ			圆弧圆心与起点在y方向的距离
			SP2			用于记录刀补之后的补偿点

  输出参数：SP2			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
//直线与圆弧等距线的交点
void Intersection_Of_Line_And_Arc(double dToolRadius, SPointDouble SP1, SVector LineVec, double dK, double dI,
                                  SPointDouble* SP2, int8 cArcDirection, uint8* ucAxisNum)
{
    double dR = sqrt(pow(dK, 2) + pow(dI, 2));
    double f = 0;
    if (SP2 == NULL)
    {
        return;
    }

    if (cArcDirection == ARC_CW) //顺圆弧时
    {
        f = sqrt(fabs(pow((dR + dToolRadius), 2) -
                      pow((-dI * LineVec.dArray[ucAxisNum[0]] + dK * LineVec.dArray[ucAxisNum[1]] - dToolRadius), 2)));
    }
    if (cArcDirection == ARC_ANTI_CW) //逆圆弧时
    {
        f = sqrt(fabs(pow((dR - dToolRadius), 2) -
                      pow((-dI * LineVec.dArray[ucAxisNum[0]] + dK * LineVec.dArray[ucAxisNum[1]] - dToolRadius), 2)));
    }
    SP2->dArray[ucAxisNum[0]] =
        SP1.dArray[ucAxisNum[0]] -
        LineVec.dArray[ucAxisNum[0]] * (LineVec.dArray[ucAxisNum[0]] * dK + LineVec.dArray[ucAxisNum[1]] * dI) -
        dToolRadius * LineVec.dArray[ucAxisNum[1]] +
        Sgn(LineVec.dArray[ucAxisNum[0]] * dK + LineVec.dArray[ucAxisNum[1]] * dI) * LineVec.dArray[ucAxisNum[0]] * f;

    SP2->dArray[ucAxisNum[1]] =
        SP1.dArray[ucAxisNum[1]] -
        LineVec.dArray[ucAxisNum[1]] * (LineVec.dArray[ucAxisNum[0]] * dK + LineVec.dArray[ucAxisNum[1]] * dI) +
        dToolRadius * LineVec.dArray[ucAxisNum[0]] +
        Sgn(LineVec.dArray[ucAxisNum[0]] * dK + LineVec.dArray[ucAxisNum[1]] * dI) * LineVec.dArray[ucAxisNum[1]] * f;
}

/*************************************************
  函数名称：Intersection_Of_Arc_And_Line(SPointDouble &SP1, SPointDouble &SP2, SPointDouble &SP3, SVector LineVec, double dI, double dJ)
  函数功能  根据起点、转接点、直线的方向矢量、圆弧的IK值求出刀补之后的补偿点
  输入参数：SP1			起点
			SP2			转接点
			LineVec		直线矢量
			dI			圆弧圆心与起点在x方向的距离
			dJ			圆弧圆心与起点在y方向的距离
			SP3			用于记录刀补之后的补偿点

  输出参数：SP3			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
//圆弧转直线等距线的交点
void Intersection_Of_Arc_And_Line(double dToolRadius, SPointDouble SP1, SPointDouble SP2, SPointDouble* SP3,
                                  SVector LineVec, double dK, double dI, int8 cArcDirection, uint8* ucAxisNum)
{
    int8 cSgn = 0;
    double f = 0;
    double dR = sqrt(pow(dK, 2) + pow(dI, 2));
    double dZ_01 = 0, dX_01 = 0;
    if (SP3 == NULL)
    {
        return;
    }
    dZ_01 = -SP1.dArray[ucAxisNum[0]] + dK + SP2.dArray[ucAxisNum[0]];
    dX_01 = -SP1.dArray[ucAxisNum[1]] + dI + SP2.dArray[ucAxisNum[1]];

    if (cArcDirection == ARC_CW) //顺圆弧
    {
        f = sqrt(
            fabs(pow((dR + dToolRadius), 2) -
                 pow((-dX_01 * LineVec.dArray[ucAxisNum[0]] + dZ_01 * LineVec.dArray[ucAxisNum[1]] - dToolRadius), 2)));
    }
    else if (cArcDirection == ARC_ANTI_CW) //逆圆弧
    {
        f = sqrt(
            fabs(pow((dR - dToolRadius), 2) -
                 pow((-dX_01 * LineVec.dArray[ucAxisNum[0]] + dZ_01 * LineVec.dArray[ucAxisNum[1]] - dToolRadius), 2)));
    }

    if (isZero(Sgn(LineVec.dArray[ucAxisNum[0]] * dZ_01 + LineVec.dArray[ucAxisNum[1]] * dX_01)))
    {
        cSgn = 1;
    }
    else
        cSgn = Sgn(LineVec.dArray[ucAxisNum[0]] * dZ_01 + LineVec.dArray[ucAxisNum[1]] * dX_01);

    SP3->dArray[ucAxisNum[0]] =
        SP2.dArray[ucAxisNum[0]] -
        LineVec.dArray[ucAxisNum[0]] * (LineVec.dArray[ucAxisNum[0]] * dZ_01 + LineVec.dArray[ucAxisNum[1]] * dX_01) -
        dToolRadius * LineVec.dArray[ucAxisNum[1]] + cSgn * LineVec.dArray[ucAxisNum[0]] * f;

    SP3->dArray[ucAxisNum[1]] =
        SP2.dArray[ucAxisNum[1]] -
        LineVec.dArray[ucAxisNum[1]] * (LineVec.dArray[ucAxisNum[0]] * dZ_01 + LineVec.dArray[ucAxisNum[1]] * dX_01) +
        dToolRadius * LineVec.dArray[ucAxisNum[0]] + cSgn * LineVec.dArray[ucAxisNum[1]] * f;
}

/*************************************************
  函数名称：Intersection_Of_Arc_And_Arc(SPointDouble&SP1, SPointDouble&SP2, SPointDouble&SP3, double I1, double J1, double I2, double J2)
  函数功能  根据起点、转接点、圆弧的方向矢量、圆弧1的IK值以及圆弧2的IK值求出刀补之后的补偿点
  输入参数：SP1			          起点
			SP2			          转接点
			K1			          圆弧1圆心与起点在Z方向的距离
			I1			          圆弧1圆心与起点在X方向的距离
			K2			          圆弧2圆心与起点在Z方向的距离
			I2			          圆弧2圆心与起点在X方向的距离
			SP3			          用于记录刀补之后的补偿点
			cArc1Direction        圆弧1方向
			cArc2Direction        圆弧2方向
			ucPosFlag             位置标志，1为两圆不相切，2为两圆外切，3为两圆内切

  输出参数：SP3			刀补之后的补偿点
  返 回 值：
  备    注：
**************************************************/
//圆弧转圆弧等距线的交点
void Intersection_Of_Arc_And_Arc(double dToolRadius, SPointDouble SP1, SPointDouble SP2, SPointDouble* SP3, double dK1,
                                 double dI1, double dK2, double dI2, int8 cArc1Direction, int8 cArc2Direction,
                                 uint8* ucAxisNum, uint8 ucPosFlag)
{

    double Z_01, X_01, Z_02, X_02;
    double R1, R2, f;
    double d, Z_L, X_L, r;
    int8 cSgn = 0;

    if (SP3 == NULL)
    {
        return;
    }

    Z_01 = SP1.dArray[ucAxisNum[0]] - dK1 - SP2.dArray[ucAxisNum[0]];
    X_01 = SP1.dArray[ucAxisNum[1]] - dI1 - SP2.dArray[ucAxisNum[1]];
    Z_02 = -dK2;
    X_02 = -dI2;
    R1 = sqrt(pow(dK1, 2) + pow(dI1, 2));
    R2 = sqrt(pow(dK2, 2) + pow(dI2, 2));
    d = sqrt(pow((X_02 - X_01), 2) + pow((Z_02 - Z_01), 2));

    //如果整圆时,增加进行d为0判断，防止r参数为零
    if (isZero(d))
    {
        Z_L = 0;
        X_L = 0;
        r = 0;
        f = R2 + dToolRadius;
    }

    else
    {
        Z_L = (X_02 - X_01) / d;
        X_L = -(Z_02 - Z_01) / d;

        if (cArc1Direction == ARC_CW && cArc2Direction == ARC_CW) //两顺圆弧
        {
            r = (R2 - R1) * dToolRadius / d;
            f = sqrt(fabs(pow((R2 + dToolRadius), 2) - pow((-X_02 * Z_L + Z_02 * X_L - r), 2)));
        }

        else if (cArc1Direction == ARC_ANTI_CW && cArc2Direction == ARC_ANTI_CW) //两逆圆弧
        {
            r = (R1 - R2) * dToolRadius / d;
            f = sqrt(fabs(pow((R2 - dToolRadius), 2) - pow((-X_02 * Z_L + Z_02 * X_L - r), 2)));
        }

        else if (cArc1Direction == ARC_CW && cArc2Direction == ARC_ANTI_CW) //第一条圆弧顺，第二条圆弧逆
        {
            r = -(R1 + R2) * dToolRadius / d;
            f = sqrt(fabs(pow((R2 - dToolRadius), 2) - pow((-X_02 * Z_L + Z_02 * X_L - r), 2)));
        }

        else if (cArc1Direction == ARC_ANTI_CW && cArc2Direction == ARC_CW) //第一条圆弧逆，第二条圆弧顺
        {
            r = (R1 + R2) * dToolRadius / d;
            f = sqrt(fabs(pow((R2 + dToolRadius), 2) - pow((-X_02 * Z_L + Z_02 * X_L - r), 2)));
        }
    }

    //当两圆相切时根据外切或内切进行额外的处理
    if (isZero(Sgn(X_02 * X_L + Z_02 * Z_L)))
    {
        //如果为左刀补
        if ((dToolRadius > 0 && ucPosFlag == 2) || (dToolRadius < 0 && ucPosFlag == 3))
        {
            cSgn = 1;
        }

        //如果为右刀补
        else if ((dToolRadius < 0 && ucPosFlag == 2) || (dToolRadius > 0 && ucPosFlag == 3))
        {
            cSgn = -1;
        }

        else
            cSgn = 0;
    }

    //不相切时直接取原来的符号
    else
        cSgn = Sgn(X_02 * X_L + Z_02 * Z_L);

    SP3->dArray[ucAxisNum[0]] = SP2.dArray[ucAxisNum[0]] + Z_L * (X_02 * X_L + Z_02 * Z_L) + r * X_L - cSgn * f * Z_L;
    SP3->dArray[ucAxisNum[1]] = SP2.dArray[ucAxisNum[1]] + X_L * (X_02 * X_L + Z_02 * Z_L) - r * Z_L - cSgn * f * X_L;
}

/*************************************************
  函数名称：SaveCommonParameter(SGCodeParse *Gcode, SToolCompensation &Toolcode,SPointDouble TcodePrevPoint,SPointDouble TcodeEndPoint)
  函数功能  初始化并继承刀补缓冲区的公共参数
  输入参数：Gcode	           G代码解析后的结构体指针
			Toolcode	       刀补后的结构体
			TcodePrevPoint     前一个刀补偿点
			TcodeEndPoint      新计算的刀补偿点
  输出参数：
  返 回 值：
  备    注：改变刀补结构体的各成员值
**************************************************/
void SaveCommonParameter(struct SGCodeParse* pGcode, SPointDouble STcodePrevPoint, SPointDouble STcodeEndPoint,
                         uint8* ucAxisNum, uint8 ucChannelNum)
{
    uint8 ucCounter;
    struct SToolCompensation* pToolcode;
    pToolcode = getSToolCompPointerByType(STOOL_WRITE, ucChannelNum); // 获取刀补的写指针

    memset(pToolcode, 0, sizeof(struct SToolCompensation));

    pToolcode->bHasVeerFlag = FALSE;
    pToolcode->bLookAheadVParaCalcOverFlag = FALSE;    //郭强2022/11/28
    pToolcode->ucAxisNumOfNeedLookAhead = AXIS_NULL;   //洪冠鹏 2024/5/27 需要前瞻的单轴轴号初始化为空
    pToolcode->cAxisDirOfNeedLookAhead = INP_GO_AHEAD; //洪冠鹏 2024/5/27 需要前瞻的单轴方向初始化为正方向
    strncpy(pToolcode->cFilename, pGcode->cFilename, FILENAME_LENGTH_LIMIT);
    pToolcode->unGCodeLineNum = pGcode->unGCodeLineNum;
    pToolcode->bLastLineMark = pGcode->bLastLineMark;
    pToolcode->SToolPara.ucToolID = pGcode->ucToolID;
    pToolcode->ucToolCompNum = pGcode->ucToolCompNum;
    pToolcode->uG10ToolID = pGcode->uG10ToolID;
    pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed; //fyh 2023.4.6
    pToolcode->dSpindleSpeed = pGcode->dSpindleSpeed;
    pToolcode->dSecondSpindleSpeed = pGcode->dSecondSpindleSpeed;
    pToolcode->sChannelCommand = pGcode->sChannelCommand;

    pToolcode->cToolAxisMod = pGcode->cToolAxisMod;                 //刀具轴模式
    pToolcode->dToolAxisFollowRatio = pGcode->dToolAxisFollowRatio; //指令转速比
    pToolcode->dToolAxisPhase = pGcode->dToolAxisPhase;             //指令相位角
    pToolcode->cTwistCommand = pGcode->cTwistCommand;               //刀具轴扭转指令（G0,G1,G2）
    pToolcode->dTwistAngle = pGcode->dTwistAngle;                   //刀具轴扭转角

    pToolcode->dFeedrate = pToolcode->dUncorrectedFeedrate =
        pGcode->dFeedrate; //郭强增加dUncorrectedFeedrate成员 2022/11/25(用于回写时)
    pToolcode->ulDelayTime = pGcode->unDelayTime;
    pToolcode->unCommandType = pGcode->unCommandType;
    pToolcode->dQ = pGcode->dQ;
    pToolcode->bValidQ = pGcode->bValidQ;
    pToolcode->SIOContrlInfo = pGcode->SIOContrlInfo;   //陈敏聪	7/10
    pToolcode->bChipBreakFlag = pGcode->bChipBreakFlag; //断屑指令G4.9
    pToolcode->unChipBreakDelayTime = pGcode->unChipBreakDelayTime;
    pToolcode->dChipBreakLength = pGcode->dChipBreakLength;

    pToolcode->bG31Flag = pGcode->bG31Flag;

    //先继承G代码缓存区的起点终点，以获取所有非刀补平面坐标轴的值
    pToolcode->SStartPoint = pGcode->SStartPoint;
    pToolcode->SEndPoint = pGcode->SEndPoint;
    pToolcode->dEllipseFactorA = pGcode->dEllipseFactorA;
    pToolcode->dEllipseFactorB = pGcode->dEllipseFactorB;
    pToolcode->fCylinderRadius = pGcode->fCylinderRadius;
    pToolcode->dHorMax = pGcode->dHorMax; //获取圆弧轨迹上横轴方向的最大值  洪冠鹏 2024/4/19
    pToolcode->dHorMin = pGcode->dHorMin; //获取圆弧轨迹上横轴方向的最小值
    pToolcode->dVerMax = pGcode->dVerMax; //获取圆弧轨迹上纵轴方向的最大值
    pToolcode->dVerMin = pGcode->dVerMin; //获取圆弧轨迹上纵轴方向的最小值

    //将刀补平面计算后的坐标赋给刀补缓存区中
    pToolcode->SStartPoint.dArray[ucAxisNum[0]] = STcodePrevPoint.dArray[ucAxisNum[0]];
    pToolcode->SStartPoint.dArray[ucAxisNum[1]] = STcodePrevPoint.dArray[ucAxisNum[1]];
    pToolcode->SEndPoint.dArray[ucAxisNum[0]] = STcodeEndPoint.dArray[ucAxisNum[0]];
    pToolcode->SEndPoint.dArray[ucAxisNum[1]] = STcodeEndPoint.dArray[ucAxisNum[1]];
    pToolcode->cQuickHandleType = pGcode->cQuickHandleType;
    pToolcode->cMergedCounter = pGcode->cMergedCounter;
    pToolcode->cWorkpieceCoordNum = pGcode->SModalInfo.sGb; //工件坐标号
    //pToolcode->pSG50Offset = pGcode->pSG50Offset;

    pToolcode->cPointType = pGcode->cPointType;
    pToolcode->uG10ToolID = pGcode->uG10ToolID;
    pToolcode->SPointInfo = pGcode->SPointInfo;

    //添加G84所需要参数2025 11 15
    pToolcode->dG84Ratio = pGcode->dG84Ratio;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    pToolcode->bHandwheelAuto = pGcode->bHandwheelAuto || pGcode->cToolAxisMod != 0 || pGcode->cTwistCommand != 0;
#endif

    //	pToolcode->bG50Appear = pGcode->bG50Appear;//传入G50临时坐标标志位 2023-08-3洪冠鹏
    pToolcode->bG65SystemMacroWait = pGcode->bG65SystemMacroWait;
    pGcode->bG65SystemMacroWait = FALSE;                                     //复位标志位 2023.11.23 陈育楠
    for (ucCounter = 0; ucCounter < IO_MAXGROUP_IN_SINGLE_LINE; ucCounter++) //IO类型 M代码赋值
    {
        pToolcode->ucIOCommandArray[ucCounter] = pGcode->ucIOCommandArray[ucCounter];
    }
}

/*************************************************
  函数名称：calcG0VectorAndDistance(struct SToolCompensation* pToolcode)
  函数功能  快速运动复合位移长度及方向向量（各轴比例）计算
  输入参数： SStartPoint                     起点坐标
			SEndPoint                       中点坐标
			nArraySize                      数组长度
			uint8* ucMapArray               映射
			VAL_STRUCT_TYPE* ucValStatus    有效轴
			VAL_STRUCT_TYPE	ucRelaVal       相对值

  输出参数： SStartVector                     起点向量指针
			SEndVector	                    终点向量指针
			dLength                         长度指针

  返 回 值：-1指针为空
			1指针不为空
  备    注：
**************************************************/
int8 calcG0VectorAndDistance(SPointDouble SStartPoint, SPointDouble SEndPoint, SVector* pSStartVector,
                             SVector* pSEndVector, double* pdLength, int nArraySize, uint8* ucMapArray,
                             VAL_STRUCT_TYPE* ucValStatus, VAL_STRUCT_TYPE ucRelaVal)
{
    //1.创建各轴位移分量数组（也可直接用方向向量数组保存）
    int i = 0;
    double dLength2 = 0;
    if (pSStartVector == NULL || pSEndVector == NULL || pdLength == NULL)
    {
        //空指针
        return -1;
    }
    //2.依次遍历所有轴，ucValStatus中无效轴分量为0，有效轴进行计算，确定各轴分量
    //2.1无效轴
    //2.2有效轴（直线轴）

    for (i = 0; i < nArraySize; i++)
    {
        if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX ||
            (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))) //映射为空  或不是有效轴 跳过
        {
            //对应轴位移分量为0，暂存在起点向量中
            pSStartVector->dArray[i] = 0;
            continue;
        }

        if (g_SSystemPara.SAxisPara[ucMapArray[i]].ucAxisType == LINEAR_AXIS) // 判断为直线轴
        {
            //线性轴位移分量直接终点减起点
            pSStartVector->dArray[i] = SEndPoint.dArray[i] - SStartPoint.dArray[i];
        }
        else //旋转轴
        {
            //获取旋转轴起点坐标直接取余取正
            double dRotationStartPoint = surplusDoubleToPositive(SStartPoint.dArray[i], 360); //取余取正;

            double dForwardDist = 0.0;

            double dBackwardDist = 0.0;

            double dCoefOfAngleToLength = 0.0;

            //对旋转轴角度位移转换系数进行0判断，为0时取1.0，防止用户参数设置错误  郭强 2023.5.26
            if (isZero(g_SSystemPara.SAxisPara[ucMapArray[i]].dCoefOfAngleToLength))
            {
                dCoefOfAngleToLength = 1.0;
            }
            else
            {
                dCoefOfAngleToLength = g_SSystemPara.SAxisPara[ucMapArray[i]].dCoefOfAngleToLength;
            }

            //C轴为相对坐标 //郭强 2023.5.19
            if (getbit(ucRelaVal, i) == 1) // 相对坐标
            {
                //根据相对坐标计算
                SEndPoint.dArray[i] = dRotationStartPoint + SEndPoint.dArray[i] - SStartPoint.dArray[i];

                pSStartVector->dArray[i] = (SEndPoint.dArray[i] - dRotationStartPoint) *
                                           dCoefOfAngleToLength; //先将旋转轴位移分量记录在向量分量中

                continue;
            }

            //计算顺时针和逆时针的位移（无符号）备用
            dForwardDist = surplusDoubleToPositive(SEndPoint.dArray[i] - dRotationStartPoint,
                                                   360); //顺时针的距离(终点坐标不用先取余)
            dBackwardDist = surplusDoubleToPositive(dRotationStartPoint - SEndPoint.dArray[i], 360); //逆时针的距离

            //按旋转轴运动方式分情况确定方向和位移增量（旋转轴要加上角度位移转化系数算增量）郭强 2023.5.19
            if (g_SSystemPara.ucRotationAxisInterpolationMode ==
                ROTATION_INTERPOLATION_SHORT_DISTANCE) //根据距离最短确定方向
            {
                if (dForwardDist < dBackwardDist) //顺时针距离较短
                {
                    pSStartVector->dArray[i] = dForwardDist * dCoefOfAngleToLength; //符号为正
                }
                else //逆时针距离较短
                {
                    pSStartVector->dArray[i] = -1.0 * dBackwardDist * dCoefOfAngleToLength; //符号为负
                }
            }
            else //根据符号确定方向
            {
                pSStartVector->dArray[i] = sgn(SEndPoint.dArray[i]) < 0 ? -1.0 * dBackwardDist * dCoefOfAngleToLength
                                                                        : dForwardDist * dCoefOfAngleToLength;
            }
        }
    }

    //3.计算当量合成位移
    //3.1计算当量位移平方
    for (i = 0; i < nArraySize; i++)
    {
        dLength2 += pow(pSStartVector->dArray[i], 2);
    }

    //3.2开方
    *pdLength = sqrt(dLength2);

    //4.计算各轴比例系数保存在方向向量中
    for (i = 0; i < nArraySize; i++)
    {
        //注：长度为零应在DOT类型判断时过滤，此函数计算时长度不为零
        pSStartVector->dArray[i] = isZero(*pdLength) ? 0 : pSStartVector->dArray[i] / *pdLength;

        //5.终点向量等于起点向量（G1）
        pSEndVector->dArray[i] = pSStartVector->dArray[i];
    }

    return 1;
}
/*************************************************
  函数名称：calcG1VectorAndDistance(struct SToolCompensation* pToolcode)
  函数功能  多轴插补复合位移长度及方向向量（各轴比例）计算
  输入参数： SStartPoint                     起点坐标
			SEndPoint                       中点坐标
			nArraySize                      数组长度
			uint8* ucMapArray               映射
			VAL_STRUCT_TYPE* ucValStatus    有效轴
			VAL_STRUCT_TYPE	ucRelaVal       相对值

  输出参数： SStartVector                     起点向量指针
			SEndVector	                    终点向量指针
			dLength                         长度指针

  返 回 值：-1指针为空
			1指针不为空
  备    注： 此时刀补单元中已经存继承了G代码解析的起点终点坐标，
			 需要根据第二层映射判断轴是否为旋转轴
			 根据旋转轴运动模式确定旋转轴的运动方向和复合位移（各轴比例保存在方向向量中）填入此刀补单元
**************************************************/
int8 calcG1VectorAndDistance(SPointDouble SStartPoint, SPointDouble SEndPoint, SVector* pSStartVector,
                             SVector* pSEndVector, double* pdLength, int nArraySize, uint8* ucMapArray,
                             VAL_STRUCT_TYPE* ucValStatus, VAL_STRUCT_TYPE ucRelaVal)
{
    //1.创建各轴位移分量数组（也可直接用方向向量数组保存）
    int i = 0;
    double dLength2 = 0;
    if (pSStartVector == NULL || pSEndVector == NULL || pdLength == NULL)
    {
        //空指针
        return -1;
    }
    //2.依次遍历所有轴，ucValStatus中无效轴分量为0，有效轴进行计算，确定各轴分量
    //2.1无效轴
    //2.2有效轴
    //2.2.1直线轴
    //2.2.2旋转轴
    //2.2.2.1按符号
    //2.2.2.2按最短长度

    for (i = 0; i < nArraySize; i++)
    {
        if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX ||
            (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))) //映射为空  或不是有效轴 跳过
        {
            //对应轴位移分量为0，暂存在起点向量中
            pSStartVector->dArray[i] = 0;
            continue;
        }

        if (g_SSystemPara.SAxisPara[ucMapArray[i]].ucAxisType == LINEAR_AXIS) // 判断为直线轴
        {
            //线性轴位移分量直接终点减起点
            pSStartVector->dArray[i] = SEndPoint.dArray[i] - SStartPoint.dArray[i];
        }
        else //旋转轴
        {
            //获取旋转轴起点坐标直接取余取正
            double dRotationStartPoint = surplusDoubleToPositive(SStartPoint.dArray[i], 360); //取余取正;

            double dForwardDist = 0.0;

            double dBackwardDist = 0.0;

            double dCoefOfAngleToLength = 0.0;

            //对旋转轴角度位移转换系数进行0判断，为0时取1.0，防止用户参数设置错误  郭强 2023.5.26
            if (isZero(g_SSystemPara.SAxisPara[ucMapArray[i]].dCoefOfAngleToLength))
            {
                dCoefOfAngleToLength = 1.0;
            }
            else
            {
                dCoefOfAngleToLength = g_SSystemPara.SAxisPara[ucMapArray[i]].dCoefOfAngleToLength;
            }

            //C轴为相对坐标 //郭强 2023.5.19
            if (getbit(ucRelaVal, i) == 1) // 相对坐标
            {
                //根据相对坐标计算
                SEndPoint.dArray[i] = dRotationStartPoint + SEndPoint.dArray[i] - SStartPoint.dArray[i];

                pSStartVector->dArray[i] = (SEndPoint.dArray[i] - dRotationStartPoint) *
                                           dCoefOfAngleToLength; //先将旋转轴位移分量记录在向量分量中

                continue;
            }

            //计算顺时针和逆时针的位移（无符号）备用
            dForwardDist = surplusDoubleToPositive(SEndPoint.dArray[i] - dRotationStartPoint,
                                                   360); //顺时针的距离(终点坐标不用先取余)
            dBackwardDist = surplusDoubleToPositive(dRotationStartPoint - SEndPoint.dArray[i], 360); //逆时针的距离

            //按旋转轴运动方式分情况确定方向和位移增量（旋转轴要加上角度位移转化系数算增量）郭强 2023.5.19
            if (g_SSystemPara.ucRotationAxisInterpolationMode ==
                ROTATION_INTERPOLATION_SHORT_DISTANCE) //根据距离最短确定方向
            {
                if (dForwardDist < dBackwardDist) //顺时针距离较短
                {
                    pSStartVector->dArray[i] = dForwardDist * dCoefOfAngleToLength; //符号为正
                }
                else //逆时针距离较短
                {
                    pSStartVector->dArray[i] = -1.0 * dBackwardDist * dCoefOfAngleToLength; //符号为负
                }
            }
            else //根据符号确定方向
            {
                pSStartVector->dArray[i] = sgn(SEndPoint.dArray[i]) < 0 ? -1.0 * dBackwardDist * dCoefOfAngleToLength
                                                                        : dForwardDist * dCoefOfAngleToLength;
            }
        }
    }

    //3.计算当量合成位移
    //3.1计算当量位移平方
    for (i = 0; i < nArraySize; i++)
    {
        dLength2 += pow(pSStartVector->dArray[i], 2);
    }

    //3.2开方
    *pdLength = sqrt(dLength2);

    //4.计算各轴比例系数保存在方向向量中
    for (i = 0; i < nArraySize; i++)
    {
        //注：长度为零应在DOT类型判断时过滤，此函数计算时长度不为零
        pSStartVector->dArray[i] = isZero(*pdLength) ? 0 : pSStartVector->dArray[i] / *pdLength;

        //5.终点向量等于起点向量（G1）
        pSEndVector->dArray[i] = pSStartVector->dArray[i];
    }

    return 1;
}

/*************************************************
  函数名称：GcodeToTCode(SGCodeParse *Gcode, SToolCompensation &Toolcode,SPointDouble TcodePrevPoint,SPointDouble TcodeEndPoint)
  函数功能  更新刀补后的结构体，并每次更新前一个刀补偿点
  输入参数：Gcode	           G代码解析后的结构体指针
			Toolcode	       刀补后的结构体
			TcodePrevPoint     前一个刀补偿点
			TcodeEndPoint      新计算的刀补偿点
  输出参数：
  返 回 值：
  备    注：通过引用改变刀补结构体的各成员值
**************************************************/
void GcodeToTCode(struct SGCodeParse* pGcode, SPointDouble STcodePrevPoint, SPointDouble STcodeEndPoint, bool bPFlag,
                  uint8* ucAxisNum, uint8 ucChannelNum)
{
    struct SToolCompensation* pToolcode;
    double Rad;
    uint8 ucCounter = 0;
    bool bIsNoMoveFlag = TRUE;
    SPointDouble SCenterPoint;
    if (pGcode == NULL)
    {
        return;
    }

    pToolcode = getSToolCompPointerByType(STOOL_WRITE, ucChannelNum); // 获取刀补的写指针

    //这里保存到刀补结构体的进给速度是G代码中的速度dFeedrate，
    //此速度按照企业要求应该代表长轴速度，长轴模式下此速度不需要修正，因为长轴模式是对长轴位移规划，向量在G1程序中代表的是对应轴位移对长轴位移的倍率
    //但是当有前瞻的多轴模式时是对复合位移规划，所以后边在向量计算后要对此值进行修正  2022.11.21郭强注释
    SaveCommonParameter(pGcode, STcodePrevPoint, STcodeEndPoint, ucAxisNum, ucChannelNum);

    //遍历所有轴，查看是否起点终点相同
    for (ucCounter = 0; ucCounter < AXIS_NUM_MAX; ucCounter++) //2023.3.24日08:58冼浩基
    {
        if (!isZero(pGcode->SStartPoint.dArray[ucCounter] - pGcode->SEndPoint.dArray[ucCounter]))
        {
            bIsNoMoveFlag = FALSE;
            break;
        }
    }

    //如果起点终点标志位为true时
    if (bIsNoMoveFlag)
    {
        pToolcode->EntityType = DOT;
        pToolcode->unCommandType = pGcode->unCommandType;
        pToolcode->SEndVector = pToolcode->SStartVector;
        pToolcode->ucValStatus = pGcode->ucValStatus;
        pToolcode->ucRelaVal = pGcode->ucRelaVal;
        pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
        pToolcode->sChannelCommand = pGcode->sChannelCommand;
        pToolcode->dQ = pGcode->dQ;
        pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;

        switch (pGcode->SModalInfo.sGa) //陈敏聪 2023/6/15增
        {
        case 32:
        {
            pToolcode->ucG32SubMode = pGcode->ucG32SubMode;
        }
        }
    }

    else
    {
        if (!bPFlag) //未存储第一个补偿点
        {
            switch (pGcode->SModalInfo.sGa)
            {
            case 0: //类型为G00
                //继承原pGcode没有改变的信息，增加刀补计算之后的过渡点、长度、方向向量......
                pToolcode->unCommandType = pGcode->unCommandType;
                pToolcode->EntityType = OTHERTYPE;
                pToolcode->dLength =
                    sqrt(pow((STcodePrevPoint.dArray[ucAxisNum[1]] - STcodeEndPoint.dArray[ucAxisNum[1]]), 2) +
                         pow((STcodePrevPoint.dArray[ucAxisNum[0]] - STcodeEndPoint.dArray[ucAxisNum[0]]), 2));
                pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                    (STcodeEndPoint.dArray[ucAxisNum[1]] - STcodePrevPoint.dArray[ucAxisNum[1]]) / pToolcode->dLength;
                pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                    (STcodeEndPoint.dArray[ucAxisNum[0]] - STcodePrevPoint.dArray[ucAxisNum[0]]) / pToolcode->dLength;
                pToolcode->SEndVector = pToolcode->SStartVector;
                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
                //根据插补模式选择不同的长度向量计算方式
                if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE) //长轴插补模式
                {
                    //此时刀补根据插补平面只计算插补平面两个轴的合成长度和向量
                    pToolcode->dLength =
                        sqrt(pow((STcodePrevPoint.dArray[ucAxisNum[1]] - STcodeEndPoint.dArray[ucAxisNum[1]]), 2) +
                             pow((STcodePrevPoint.dArray[ucAxisNum[0]] - STcodeEndPoint.dArray[ucAxisNum[0]]), 2));
                    pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                        (STcodeEndPoint.dArray[ucAxisNum[1]] - STcodePrevPoint.dArray[ucAxisNum[1]]) /
                        pToolcode->dLength;
                    pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                        (STcodeEndPoint.dArray[ucAxisNum[0]] - STcodePrevPoint.dArray[ucAxisNum[0]]) /
                        pToolcode->dLength;
                    pToolcode->SEndVector = pToolcode->SStartVector;
                }
                else //固定参数扩展线位移插补 、自适应参数扩展线位移插补
                {
                    //1.此时直接考虑了所有有效轴的向量分量,必须在前瞻之前处理好，前瞻要用到因此和长轴模式不同
                    calcG0VectorAndDistance(pToolcode->SStartPoint, pToolcode->SEndPoint, &pToolcode->SStartVector,
                                            &pToolcode->SEndVector, &pToolcode->dLength, AXIS_NUM_MAX,
                                            g_SChannel[ucChannelNum].pMapAxisArray, &pToolcode->ucValStatus,
                                            pToolcode->ucRelaVal);
                }
                //判断是否为单个轴的运动，若是，则保存运动轴的轴号  洪冠鹏 2024/5/27
                for (int i = 0; i < AXIS_NUM_MAX; i++)
                {
                    if ((1.0 - fabs(pToolcode->SStartVector.dArray[i])) <
                        EPSINON) //位移分量等于1，说明当前只有一个轴参与运动
                    {
                        pToolcode->ucAxisNumOfNeedLookAhead = i;                                     //保存轴号
                        pToolcode->cAxisDirOfNeedLookAhead = sgn(pToolcode->SStartVector.dArray[i]); //保存插补方向
                        break;
                    }
                }
                break;

            case 1: //类型为G01
                pToolcode->unCommandType = pGcode->unCommandType;
                pToolcode->EntityType = L1NE;

                //2022/1/10以下四个变量调整到此处  郭强
                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
                //根据插补模式选择不同的长度向量计算方式      2022.11.21郭强
                if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE) //长轴插补模式
                {
                    //此时刀补根据插补平面只计算插补平面两个轴的合成长度和向量
                    pToolcode->dLength =
                        sqrt(pow((STcodePrevPoint.dArray[ucAxisNum[1]] - STcodeEndPoint.dArray[ucAxisNum[1]]), 2) +
                             pow((STcodePrevPoint.dArray[ucAxisNum[0]] - STcodeEndPoint.dArray[ucAxisNum[0]]), 2));
                    pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                        (STcodeEndPoint.dArray[ucAxisNum[1]] - STcodePrevPoint.dArray[ucAxisNum[1]]) /
                        pToolcode->dLength;
                    pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                        (STcodeEndPoint.dArray[ucAxisNum[0]] - STcodePrevPoint.dArray[ucAxisNum[0]]) /
                        pToolcode->dLength;
                    pToolcode->SEndVector = pToolcode->SStartVector;
                }
                else //固定参数扩展线位移插补 、自适应参数扩展线位移插补     2022.11.21郭强
                {
                    //1.此时直接考虑了所有有效轴(包括旋转轴)的合成位移和向量,必须在前瞻之前处理好，前瞻要用到因此和长轴模式不同
                    calcG1VectorAndDistance(pToolcode->SStartPoint, pToolcode->SEndPoint, &pToolcode->SStartVector,
                                            &pToolcode->SEndVector, &pToolcode->dLength, AXIS_NUM_MAX,
                                            g_SChannel[ucChannelNum].pMapAxisArray, &pToolcode->ucValStatus,
                                            pToolcode->ucRelaVal);
                }
                //判断是否为单个轴的运动，若是，则保存运动轴的轴号  洪冠鹏 2024/5/27
                for (int i = 0; i < AXIS_NUM_MAX; i++)
                {
                    if ((1.0 - fabs(pToolcode->SStartVector.dArray[i])) <
                        EPSINON) //位移分量等于1，说明当前只有一个轴参与运动
                    {
                        pToolcode->ucAxisNumOfNeedLookAhead = i;                                     //保存轴号
                        pToolcode->cAxisDirOfNeedLookAhead = sgn(pToolcode->SStartVector.dArray[i]); //保存插补方向
                        break;
                    }
                }
                break;

            case 2: //类型为G02
                SCenterPoint = pGcode->SCenterPoint;
                pToolcode->unCommandType = pGcode->unCommandType;
                if (getBits(pToolcode->unCommandType, CMD_INTERPOLATION_MODE_ADDR) ==
                    CMD_INTERPOLATION_POLAR) //极坐标插补 横纵轴由系统参数给定
                {
                    ucAxisNum[0] = g_SSystemPara.ucPolarInterpolationAxis[0];
                    ucAxisNum[1] = g_SSystemPara.ucPolarInterpolationAxis[1];
                }
                else if (getBits(pToolcode->unCommandType, CMD_INTERPOLATION_MODE_ADDR) ==
                         CMD_INTERPOLATION_CYLINDRICAL)
                {
                    chooseInterpPlane(ucAxisNum, pToolcode->unCommandType);
                }
                radCalculateIK(STcodePrevPoint.dArray, STcodeEndPoint.dArray, (double*)&SCenterPoint, ARC_CW, &Rad,
                               ucAxisNum[0], ucAxisNum[1]);

                pToolcode->EntityType = ARC;
                pToolcode->SCenterPoint = pGcode->SCenterPoint;
                pToolcode->cArcDir = ARC_CW;

                //如果弧度大于π，半径给负值
                if (Rad > PI)
                {
                    pToolcode->dRadius = -sqrt(
                        pow((STcodePrevPoint.dArray[ucAxisNum[1]] - pGcode->SCenterPoint.dArray[ucAxisNum[1]]), 2) +
                        pow((STcodePrevPoint.dArray[ucAxisNum[0]] - pGcode->SCenterPoint.dArray[ucAxisNum[0]]), 2));
                }

                //小于π给正值
                else
                    pToolcode->dRadius = sqrt(
                        pow((STcodePrevPoint.dArray[ucAxisNum[1]] - pGcode->SCenterPoint.dArray[ucAxisNum[1]]), 2) +
                        pow((STcodePrevPoint.dArray[ucAxisNum[0]] - pGcode->SCenterPoint.dArray[ucAxisNum[0]]), 2));

                //插补平面长度和向量
                pToolcode->dLength = Rad * fabs(pToolcode->dRadius);
                pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                    (pToolcode->SCenterPoint.dArray[ucAxisNum[0]] - pToolcode->SStartPoint.dArray[ucAxisNum[0]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;
                pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                    (pToolcode->SStartPoint.dArray[ucAxisNum[1]] - pToolcode->SCenterPoint.dArray[ucAxisNum[1]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;
                pToolcode->SEndVector.dArray[ucAxisNum[1]] =
                    (pToolcode->SCenterPoint.dArray[ucAxisNum[0]] - pToolcode->SEndPoint.dArray[ucAxisNum[0]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;
                pToolcode->SEndVector.dArray[ucAxisNum[0]] =
                    (pToolcode->SEndPoint.dArray[ucAxisNum[1]] - pToolcode->SCenterPoint.dArray[ucAxisNum[1]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;

                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
                break;

            case 3: //类型为G03
                SCenterPoint = pGcode->SCenterPoint;
                pToolcode->unCommandType = pGcode->unCommandType;
                if (getBits(pToolcode->unCommandType, CMD_INTERPOLATION_MODE_ADDR) ==
                    CMD_INTERPOLATION_POLAR) //极坐标插补 横纵轴由系统参数给定
                {
                    ucAxisNum[0] = g_SSystemPara.ucPolarInterpolationAxis[0];
                    ucAxisNum[1] = g_SSystemPara.ucPolarInterpolationAxis[1];
                }
                else if (getBits(pToolcode->unCommandType, CMD_INTERPOLATION_MODE_ADDR) ==
                         CMD_INTERPOLATION_CYLINDRICAL)
                {
                    chooseInterpPlane(ucAxisNum, pToolcode->unCommandType);
                }
                radCalculateIK(STcodePrevPoint.dArray, STcodeEndPoint.dArray, (double*)&SCenterPoint, ARC_ANTI_CW, &Rad,
                               ucAxisNum[0], ucAxisNum[1]);

                pToolcode->EntityType = ARC;
                pToolcode->SCenterPoint = pGcode->SCenterPoint;
                pToolcode->cArcDir = ARC_ANTI_CW;
                //如果弧度大于π，半径给负值
                if (Rad > PI)
                {
                    pToolcode->dRadius = -sqrt(
                        pow((STcodePrevPoint.dArray[ucAxisNum[1]] - pGcode->SCenterPoint.dArray[ucAxisNum[1]]), 2) +
                        pow((STcodePrevPoint.dArray[ucAxisNum[0]] - pGcode->SCenterPoint.dArray[ucAxisNum[0]]), 2));
                }

                //小于π给正值
                else
                    pToolcode->dRadius = sqrt(
                        pow((STcodePrevPoint.dArray[ucAxisNum[1]] - pGcode->SCenterPoint.dArray[ucAxisNum[1]]), 2) +
                        pow((STcodePrevPoint.dArray[ucAxisNum[0]] - pGcode->SCenterPoint.dArray[ucAxisNum[0]]), 2));

                //插补平面长度和向量
                pToolcode->dLength = Rad * fabs(pToolcode->dRadius);
                pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                    (pToolcode->SCenterPoint.dArray[ucAxisNum[0]] - pToolcode->SStartPoint.dArray[ucAxisNum[0]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;
                pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                    (pToolcode->SStartPoint.dArray[ucAxisNum[1]] - pToolcode->SCenterPoint.dArray[ucAxisNum[1]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;
                pToolcode->SEndVector.dArray[ucAxisNum[1]] =
                    (pToolcode->SCenterPoint.dArray[ucAxisNum[0]] - pToolcode->SEndPoint.dArray[ucAxisNum[0]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;
                pToolcode->SEndVector.dArray[ucAxisNum[0]] =
                    (pToolcode->SEndPoint.dArray[ucAxisNum[1]] - pToolcode->SCenterPoint.dArray[ucAxisNum[1]]) /
                    fabs(pToolcode->dRadius) * pToolcode->cArcDir;

                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
                break;

            case 32:
            case 33:
            case 92:
            case 34: //类型为G32
                pToolcode->unCommandType = pGcode->unCommandType;
                pToolcode->EntityType = L1NE;
                pToolcode->dLength =
                    sqrt(pow((STcodePrevPoint.dArray[ucAxisNum[1]] - STcodeEndPoint.dArray[ucAxisNum[1]]), 2) +
                         pow((STcodePrevPoint.dArray[ucAxisNum[0]] - STcodeEndPoint.dArray[ucAxisNum[0]]), 2));

                pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                    (STcodeEndPoint.dArray[ucAxisNum[1]] - STcodePrevPoint.dArray[ucAxisNum[1]]) / pToolcode->dLength;
                pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                    (STcodeEndPoint.dArray[ucAxisNum[0]] - STcodePrevPoint.dArray[ucAxisNum[0]]) / pToolcode->dLength;

                pToolcode->SEndVector = pToolcode->SStartVector;
                pToolcode->dF = pGcode->dFeedrate; //最主要是这一句
                pToolcode->dJ = pGcode->dJ;
                pToolcode->dK = pGcode->dK;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->ucG32SubMode = pGcode->ucG32SubMode; //陈敏聪 2023/6/15增

                pToolcode->SCenterPoint = pGcode->SCenterPoint;
                pToolcode->cArcDir = pGcode->cArcDir;
                if (pToolcode->cArcDir != 0)
                {
                    pToolcode->dRadius = sqrt(
                        pow((STcodePrevPoint.dArray[ucAxisNum[1]] - pGcode->SCenterPoint.dArray[ucAxisNum[1]]), 2) +
                        pow((STcodePrevPoint.dArray[ucAxisNum[0]] - pGcode->SCenterPoint.dArray[ucAxisNum[0]]), 2));
                }
                else
                {
                    pToolcode->dRadius = 0;
                }

                pToolcode->ucL = pGcode->usL; //郭强11.22
                pToolcode->bRepeatG32Flag = pGcode->bRepeatG32Flag;
                pToolcode->bG34RunFlag = pGcode->bG34RunFlag;
                pToolcode->bG34BackTailFlag = pGcode->bG34BackTailFlag;
                pToolcode->bG32LetterQ = pGcode->bG32LetterQ;
                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->G33RotateAxis = pGcode->G33RotateAxis;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
                break;

            default:
                break;
            }
        }

        else if (bPFlag) //已经存储了第一个补偿点
        {
            bIsNoMoveFlag = TRUE;

            //刀补过程中出现多段非移动代码，可能会存在保存点的情况
            //遍历所有轴，查看是否起点终点相同
            for (ucCounter = 0; ucCounter < AXIS_NUM_MAX; ucCounter++) //2023.3.24日08:58冼浩基
            {
                if (!isZero(STcodePrevPoint.dArray[ucCounter] - STcodeEndPoint.dArray[ucCounter]))
                {
                    bIsNoMoveFlag = FALSE;
                    break;
                }
            }

            if (bIsNoMoveFlag)
            {
                pToolcode->EntityType = DOT;
                pToolcode->unCommandType = pGcode->unCommandType;
                pToolcode->SEndVector = pToolcode->SStartVector;
                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
            }

            else
            {
                pToolcode->unCommandType = setBits(pGcode->unCommandType, CMD_MOTION_LINE, CMD_MOTION_ADDR);
                pToolcode->EntityType = L1NE;
                pToolcode->dLength =
                    sqrt(pow((STcodePrevPoint.dArray[ucAxisNum[1]] - STcodeEndPoint.dArray[ucAxisNum[1]]), 2) +
                         pow((STcodePrevPoint.dArray[ucAxisNum[0]] - STcodeEndPoint.dArray[ucAxisNum[0]]), 2));

                pToolcode->SStartVector.dArray[ucAxisNum[1]] =
                    (STcodeEndPoint.dArray[ucAxisNum[1]] - STcodePrevPoint.dArray[ucAxisNum[1]]) / pToolcode->dLength;
                pToolcode->SStartVector.dArray[ucAxisNum[0]] =
                    (STcodeEndPoint.dArray[ucAxisNum[0]] - STcodePrevPoint.dArray[ucAxisNum[0]]) / pToolcode->dLength;
                pToolcode->SEndVector = pToolcode->SStartVector;
                pToolcode->ucValStatus = pGcode->ucValStatus;
                pToolcode->ucRelaVal = pGcode->ucRelaVal;
                pToolcode->cChannelBufNum = pGcode->cChannelBufNum;
                pToolcode->sChannelCommand = pGcode->sChannelCommand;
                pToolcode->dQ = pGcode->dQ;
                pToolcode->dLimitedSpindleSpeed = pGcode->dLimitedSpindleSpeed;
                //判断是否为单个轴的运动，若是，则保存运动轴的轴号  洪冠鹏 2024/5/27
                for (int i = 0; i < AXIS_NUM_MAX; i++)
                {
                    if ((1.0 - fabs(pToolcode->SStartVector.dArray[i])) <
                        EPSINON) //位移分量等于1，说明当前只有一个轴参与运动
                    {
                        pToolcode->ucAxisNumOfNeedLookAhead = i;                                     //保存轴号
                        pToolcode->cAxisDirOfNeedLookAhead = sgn(pToolcode->SStartVector.dArray[i]); //保存插补方向
                        break;
                    }
                }
            }
        }
    }

#if ENABLE_TOOL_DEBUG
    PrintDataByChannel(ucChannelNum);
    //CloseFile(ucChannelNum);
#endif

    //刀具补偿缓冲区往下移动,以便存放下一个数据
    moveSToolCompBufPointer(STOOL_WRITE, BUF_AHEAD, ucChannelNum);
}

/*************************************************
  函数名称：RecordLastToolPoint(int8 cPointNum)
  函数功能  记录刀补计算之后产生的最后一个补偿点的信息
  输入参数：cPointNum			最后一个补偿点


  输出参数：改变全局变量SToolCompensationRecord->TcodePrevPoint
  返 回 值：
  备    注：
**************************************************/
//根据刀补后的点数记录最后一个刀补点
void RecordLastToolPoint(int8 cPointNum, uint8 ucChannelNum)
{
    if (cPointNum == 1) //一个补偿点
    {
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS1;
    }
    else if (cPointNum == 2) //两个补偿点
    {
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS2;
    }
    else if (cPointNum == 3) //三个补偿点
    {
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS3;
    }
    else if (cPointNum == 4) //四个补偿点
    {
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS4;
    }
}

/*************************************************
  函数名称：LineToLine(SGCodeParse *pSGcode, SGCodeParse  *pSNextGcode, int8 StageSelect)
  函数功能  根据存放第一段轨迹的SGcode结构体、第二段轨迹的SNextGcode结构体，以及刀补的阶段进行直线接直线刀补计算
  输入参数：SGcode			第一段轨迹
			SNextGcode		第二段轨迹
			StageSelect     刀补的阶段

  输出参数：
  返 回 值：刀补的补偿点数	cPointNum
  备    注：
**************************************************/
int8 LineToLine(struct SGCodeParse* pSGcode, struct SGCodeParse* pSNextGcode, double dToolRadius, int8 cStageSelect,
                uint8* ucAxisNum)
{

    int16 sAngleType; //转接角类型
    int8 cPointNum = 0;
    SVector SLine1Vec, SLine2Vec;
    if (pSGcode == NULL || pSNextGcode == NULL)
    {
        return -1;
    }
    getLineVector(pSGcode->SStartPoint, pSGcode->SEndPoint, &SLine1Vec, ucAxisNum);
    getLineVector(pSNextGcode->SStartPoint, pSNextGcode->SEndPoint, &SLine2Vec, ucAxisNum);
    sAngleType = JudgeAngle(dToolRadius, SLine1Vec, SLine2Vec, ucAxisNum); //判断属于什么类型
    switch (sAngleType)
    {
    case 1:
        //缩短型
        switch (cStageSelect)
        {
        case 1: //刀补建立
            offsetRadiusVector(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            cPointNum = 1;
            break;
        case 2: //刀补进行
            //两直线共线时直接偏移一个刀具半径
            if (isZero(
                    (pSGcode->SEndPoint.dArray[ucAxisNum[0]] - pSGcode->SStartPoint.dArray[ucAxisNum[0]]) *
                        (pSNextGcode->SEndPoint.dArray[ucAxisNum[1]] - pSNextGcode->SStartPoint.dArray[ucAxisNum[1]]) -
                    (pSGcode->SEndPoint.dArray[ucAxisNum[1]] - pSGcode->SStartPoint.dArray[ucAxisNum[1]]) *
                        (pSNextGcode->SEndPoint.dArray[ucAxisNum[0]] - pSNextGcode->SStartPoint.dArray[ucAxisNum[0]])))
            {
                offsetRadiusVector(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            }

            //不共线
            else
            {
                Intersection_Of_Quidistant_Lines(dToolRadius, SLine1Vec, SLine2Vec, pSGcode->SEndPoint,
                                                 &SToolRunPoint.SS1, ucAxisNum);
            }
            cPointNum = 1;
            break;
        case 3: //刀补撤消
            offsetRadiusVector(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            cPointNum = 1;
            break;
        }
        break;
    case 2:
        //伸长型
        switch (cStageSelect)
        {
        case 1: //刀补建立
            offsetRadiusVector(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            cPointNum = 1;
            break;
        case 2: //刀补进行
            Intersection_Of_Quidistant_Lines(dToolRadius, SLine1Vec, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1,
                                             ucAxisNum);
            cPointNum = 1;
            break;
        case 3: //刀补撤消
            Intersection_Of_Quidistant_Lines(dToolRadius, SLine1Vec, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1,
                                             ucAxisNum);
            offsetRadiusVector(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            cPointNum = 2;
            break;
        }
        break;
    case 3:
        //插入型
        switch (cStageSelect)
        {
        case 1: //刀补建立
            offsetRadiusVector(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector1(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            offsetRadiusVector2(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            cPointNum = 3;
            break;
        case 2: //刀补进行
            offsetRadiusVector1(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector2(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            cPointNum = 2;
            break;
        case 3: //刀补撤消
            offsetRadiusVector1(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector2(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            offsetRadiusVector(dToolRadius, SLine2Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            cPointNum = 3;
            break;
        }

        break;
    }

    SToolRunPoint.cPointNum = cPointNum;
    return cPointNum;
}

/*************************************************
  函数名称：LineToArc(SGCodeParse *pSGcode, SGCodeParse  *pSNextGcode, int8 StageSelect)
  函数功能  根据存放第一段轨迹的SGcode结构体、第二段轨迹的SNextGcode结构体，以及刀补的阶段进行直线接圆弧刀补计算
  输入参数：SGcode			第一段轨迹
			SNextGcode		第二段轨迹
			StageSelect     刀补的阶段

  输出参数：
  返 回 值：刀补的补偿点数	cPointNum
  备    注：
**************************************************/
int8 LineToArc(struct SGCodeParse* pSGcode, struct SGCodeParse* pSNextGcode, double dToolRadius, int8 cStageSelect,
               uint8* ucAxisNum)
{

    int16 sAngleType;
    int8 cPointNum = 0;
    int16 sSpecialAngleJudge = -1;
    double dK = pSNextGcode->SStartPoint.dArray[ucAxisNum[0]] - pSNextGcode->SCenterPoint.dArray[ucAxisNum[0]];
    double dI = pSNextGcode->SStartPoint.dArray[ucAxisNum[1]] - pSNextGcode->SCenterPoint.dArray[ucAxisNum[1]];
    SVector SLine1Vec, SArcVec;
    getLineVector(pSGcode->SStartPoint, pSGcode->SEndPoint, &SLine1Vec, ucAxisNum);
    getArcVectorAtStart(dK, dI, &SArcVec, pSNextGcode->cArcDir, ucAxisNum);
    sAngleType = JudgeAngle(dToolRadius, SLine1Vec, SArcVec, ucAxisNum);
    if (pSGcode == NULL || pSNextGcode == NULL)
    {
        return -1;
    }
    switch (sAngleType)
    {
    case 1:
        //缩短型
        switch (cStageSelect)
        {
        case 1: //刀补建立
            offsetRadiusVector(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            cPointNum = 1;
            break;
        case 2: //刀补进行

            //特殊角度处理
            sSpecialAngleJudge = SpecialAngleJudge(dToolRadius, SLine1Vec, SArcVec, ucAxisNum);
            if (sSpecialAngleJudge == 180) //直线与圆弧外切夹角为180度
            {
                offsetRadiusVector(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                cPointNum = 1;
            }
            else if (sSpecialAngleJudge == 0) //直线与圆弧外切夹角为0度
            {
                if ((dToolRadius > 0 && pSNextGcode->cArcDir == ARC_CW) ||
                    (dToolRadius < 0 && pSNextGcode->cArcDir == ARC_ANTI_CW)) //左刀补顺圆弧或右刀补逆圆弧
                {
                    Intersection_Of_Line_And_Arc(dToolRadius, pSGcode->SEndPoint, SLine1Vec, dK, dI, &SToolRunPoint.SS1,
                                                 pSNextGcode->cArcDir, ucAxisNum);
                    cPointNum = 1;
                }

                if ((dToolRadius > 0 && pSNextGcode->cArcDir == ARC_ANTI_CW) ||
                    (dToolRadius < 0 && pSNextGcode->cArcDir == ARC_CW)) //左刀补逆圆弧或右刀补顺圆弧
                {
                    offsetRadiusVector1(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                    offsetRadiusVector2(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
                    offsetRadiusVector(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
                    cPointNum = 3;
                }
            }
            else
            {
                Intersection_Of_Line_And_Arc(dToolRadius, pSGcode->SEndPoint, SLine1Vec, dK, dI, &SToolRunPoint.SS1,
                                             pSNextGcode->cArcDir, ucAxisNum);
                cPointNum = 1;
            }

            break;
        }
        break;
    case 2:
        //伸长型
        switch (cStageSelect)
        {
        case 1: //刀补建立
            offsetRadiusVector(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            Intersection_Of_Quidistant_Lines(dToolRadius, SLine1Vec, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS2,
                                             ucAxisNum);
            offsetRadiusVector(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            cPointNum = 3;
            break;
        case 2: //刀补进行

            Intersection_Of_Quidistant_Lines(dToolRadius, SLine1Vec, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS1,
                                             ucAxisNum);
            offsetRadiusVector(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            cPointNum = 2;
            break;
        }
        break;
    case 3:
        //插入型
        switch (cStageSelect)
        {
        case 1: //刀补建立
            offsetRadiusVector(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector1(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            offsetRadiusVector2(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            offsetRadiusVector(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
            cPointNum = 4;
            break;
        case 2: //刀补进行
            offsetRadiusVector1(dToolRadius, SLine1Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector2(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            offsetRadiusVector(dToolRadius, SArcVec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            cPointNum = 3;
            break;
        }
        break;
    }
    SToolRunPoint.cPointNum = cPointNum;
    return cPointNum;
}

/*************************************************
  函数名称：ArcToLine(SGCodeParse *SGcode, SGCodeParse *SNextcode, int8 StageSelect)
  函数功能  调用SGcode和SNextcode中的点数据求两连接类型的之间的刀补偿点，转接类型为圆弧接直线，
			求取的点存放于全局变量的结构体SToolRunPoint中
  输入参数：SGcode			本段结构体指针
			SNextcode		下段结构体指针
			StageSelect     刀补阶段（1：进行  2：撤销）

  输出参数：
  返 回 值：PointNum			刀补之后的补偿点的数量
  备    注：
**************************************************/
//圆弧接直线
int8 ArcToLine(struct SGCodeParse* pSGcode, struct SGCodeParse* pSNextcode, double dToolRadius, int8 cStageSelect,
               uint8* ucAxisNum)
{
    SVector Arc_Vec, L_Vec;
    int16 sAngleJudge;
    int16 cPointNum = 0;
    double dK = pSGcode->SStartPoint.dArray[ucAxisNum[0]] - pSGcode->SCenterPoint.dArray[ucAxisNum[0]];
    double dI = pSGcode->SStartPoint.dArray[ucAxisNum[1]] - pSGcode->SCenterPoint.dArray[ucAxisNum[1]];
    getArcVectorAtEnd(pSGcode->SStartPoint, pSGcode->SEndPoint, dK, dI, &Arc_Vec, pSGcode->cArcDir, ucAxisNum);
    getLineVector(pSNextcode->SStartPoint, pSNextcode->SEndPoint, &L_Vec, ucAxisNum);
    sAngleJudge = JudgeAngle(dToolRadius, Arc_Vec, L_Vec, ucAxisNum);
    if (pSGcode == NULL || pSNextcode == NULL)
    {
        return -1;
    }
    switch (sAngleJudge)
    {
    case 1: //缩短型
    {
        switch (cStageSelect)
        {
        case 2: //进行
            sAngleJudge = SpecialAngleJudge(dToolRadius, Arc_Vec, L_Vec, ucAxisNum);
            if (sAngleJudge == 180) //圆弧与直线外切夹角等于180
            {
                offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                cPointNum = 1;
            }
            if (sAngleJudge == 0) //圆弧与直线外切夹角等于0
            {
                if ((dToolRadius > 0 && pSGcode->cArcDir == ARC_CW) ||
                    (dToolRadius < 0 && pSGcode->cArcDir == ARC_ANTI_CW)) //左刀补顺圆弧或右刀补逆圆弧
                {
                    Intersection_Of_Arc_And_Line(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint,
                                                 &SToolRunPoint.SS1, L_Vec, dK, dI, pSGcode->cArcDir, ucAxisNum);
                    cPointNum = 1;
                }
                else if ((dToolRadius > 0 && pSGcode->cArcDir == ARC_ANTI_CW) ||
                         (dToolRadius < 0 && pSGcode->cArcDir == ARC_CW)) //左刀补逆圆弧或右刀补顺圆弧
                {
                    offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                    offsetRadiusVector1(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
                    offsetRadiusVector2(dToolRadius, L_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
                    cPointNum = 3;
                }
            }
            else
            {
                Intersection_Of_Arc_And_Line(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint, &SToolRunPoint.SS1,
                                             L_Vec, dK, dI, pSGcode->cArcDir, ucAxisNum);
                cPointNum = 1;
            }
            break;
        case 3: //撤销
            offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            cPointNum = 1;
            break;
        }
    }
    break;
    case 2: //伸长型
    {
        switch (cStageSelect)
        {
        case 2: //进行
            offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            Intersection_Of_Quidistant_Lines(dToolRadius, Arc_Vec, L_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2,
                                             ucAxisNum);
            cPointNum = 2;
            break;
        case 3: //撤销
            offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            Intersection_Of_Quidistant_Lines(dToolRadius, Arc_Vec, L_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2,
                                             ucAxisNum);
            offsetRadiusVector(dToolRadius, L_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            cPointNum = 3;
            break;
        }
    }
    break;
    case 3: //插入型
    {
        switch (cStageSelect)
        {
        case 2: //进行
            offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector1(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            offsetRadiusVector2(dToolRadius, L_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            cPointNum = 3;
            break;
        case 3: //撤销
            offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
            offsetRadiusVector1(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
            offsetRadiusVector2(dToolRadius, L_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
            offsetRadiusVector(dToolRadius, Arc_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
            cPointNum = 4;
            break;
        }
    }
    break;
    }
    SToolRunPoint.cPointNum = cPointNum; //记录所算的点数
    return cPointNum;
}

/*************************************************
  函数名称：ArcToArc(SGCodeParse *SGcode, SGCodeParse *SNextcode, int8 StageSelect)
  函数功能  调用SGcode和SNextcode中的点数据求两连接类型的之间的刀补偿点，转接类型为圆弧接圆弧，
			求取的点存放于全局变量的结构体SToolRunPoint中
  输入参数：SGcode			本段结构体指针
			SNextcode		下段结构体指针
			StageSelect     刀补阶段

  输出参数：
  返 回 值：PointNum			刀补之后的补偿点的数量
  备    注：
**************************************************/
//圆弧接圆弧
int8 ArcToArc(struct SGCodeParse* pSGcode, struct SGCodeParse* pSNextcode, double dToolRadius, int8 StageSelect,
              uint8* ucAxisNum)
{
    double dK1 = pSGcode->SStartPoint.dArray[ucAxisNum[0]] - pSGcode->SCenterPoint.dArray[ucAxisNum[0]];
    double dI1 = pSGcode->SStartPoint.dArray[ucAxisNum[1]] - pSGcode->SCenterPoint.dArray[ucAxisNum[1]];
    double dK2 = pSNextcode->SStartPoint.dArray[ucAxisNum[0]] - pSNextcode->SCenterPoint.dArray[ucAxisNum[0]];
    double dI2 = pSNextcode->SStartPoint.dArray[ucAxisNum[1]] - pSNextcode->SCenterPoint.dArray[ucAxisNum[1]];
    SVector Arc1_Vec, Arc2_Vec;
    int8 cAngleJudge, cPointNum;
    int16 sSecialAngleJudge;
    double dDistance;
    getArcVectorAtEnd(pSGcode->SStartPoint, pSGcode->SEndPoint, dK1, dI1, &Arc1_Vec, pSGcode->cArcDir, ucAxisNum);
    getArcVectorAtStart(dK2, dI2, &Arc2_Vec, pSNextcode->cArcDir, ucAxisNum);
    cAngleJudge = JudgeAngle(dToolRadius, Arc1_Vec, Arc2_Vec, ucAxisNum);
    if (pSGcode == NULL || pSNextcode == NULL)
    {
        return -1;
    }
    switch (cAngleJudge)
    {
    case 1: //缩短型
        sSecialAngleJudge = SpecialAngleJudge(dToolRadius, Arc1_Vec, Arc2_Vec, ucAxisNum);
        dDistance =
            sqrt(pow((pSGcode->SCenterPoint.dArray[ucAxisNum[0]] - pSNextcode->SCenterPoint.dArray[ucAxisNum[0]]), 2) +
                 pow((pSGcode->SCenterPoint.dArray[ucAxisNum[1]] - pSNextcode->SCenterPoint.dArray[ucAxisNum[1]]), 2));
        if (sSecialAngleJudge == 0) //两圆弧夹角为0
        {
            if (isZero(dDistance - pSGcode->dRadius - pSNextcode->dRadius)) //两圆弧外切
            {
                if (pSGcode->cArcDir == ARC_CW && pSNextcode->cArcDir == ARC_CW) //两顺圆弧
                {
                    if (dToolRadius > 0) //左刀补按缩短型公式
                    {
                        Intersection_Of_Arc_And_Arc(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint,
                                                    &SToolRunPoint.SS1, dK1, dI1, dK2, dI2, pSGcode->cArcDir,
                                                    pSNextcode->cArcDir, ucAxisNum, 2);
                        cPointNum = 1;
                    }

                    else if (dToolRadius < 0) //右刀补按插入型公式
                    {
                        offsetRadiusVector(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                        offsetRadiusVector1(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
                        offsetRadiusVector2(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
                        offsetRadiusVector(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
                        cPointNum = 4;
                    }
                }

                else if (pSGcode->cArcDir == ARC_ANTI_CW && pSNextcode->cArcDir == ARC_ANTI_CW) //两逆圆弧
                {
                    if (dToolRadius < 0) //右刀补按缩短型公式
                    {
                        Intersection_Of_Arc_And_Arc(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint,
                                                    &SToolRunPoint.SS1, dK1, dI1, dK2, dI2, pSGcode->cArcDir,
                                                    pSNextcode->cArcDir, ucAxisNum, 2);
                        cPointNum = 1;
                    }

                    else if (dToolRadius > 0) //左刀补按插入型公式
                    {
                        offsetRadiusVector(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                        offsetRadiusVector1(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
                        offsetRadiusVector2(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
                        offsetRadiusVector(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
                        cPointNum = 4;
                    }
                }
            }
            if (isZero(dDistance - fabs(pSGcode->dRadius - pSNextcode->dRadius))) //两圆弧内切
            {
                if (pSGcode->dRadius < pSNextcode->dRadius) //第一段圆弧半径小于第二段圆弧半径
                {
                    if ((dToolRadius > 0 && pSGcode->cArcDir == ARC_CW) ||
                        (dToolRadius < 0 &&
                         pSGcode->cArcDir == ARC_ANTI_CW)) //左刀补，第一段为顺圆弧或者右刀补，第一段为逆圆弧
                    {
                        Intersection_Of_Arc_And_Arc(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint,
                                                    &SToolRunPoint.SS1, dK1, dI1, dK2, dI2, pSGcode->cArcDir,
                                                    pSNextcode->cArcDir, ucAxisNum, 3);
                        cPointNum = 1;
                    }

                    else if ((dToolRadius < 0 && pSGcode->cArcDir == ARC_CW) ||
                             (dToolRadius > 0 &&
                              pSGcode->cArcDir == ARC_ANTI_CW)) //右刀补，第一段为顺圆弧或者左刀补，第一段为逆圆弧
                    {
                        offsetRadiusVector(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                        offsetRadiusVector1(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
                        offsetRadiusVector2(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
                        offsetRadiusVector(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
                        cPointNum = 4;
                    }
                }

                else if (pSGcode->dRadius > pSNextcode->dRadius) //第一段圆弧半径大于第二段圆弧半径
                {
                    if ((dToolRadius < 0 && pSGcode->cArcDir == ARC_CW) ||
                        (dToolRadius > 0 &&
                         pSGcode->cArcDir == ARC_ANTI_CW)) //右刀补，第一段为顺圆弧或者左刀补，第一段为逆圆弧
                    {
                        Intersection_Of_Arc_And_Arc(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint,
                                                    &SToolRunPoint.SS1, dK1, dI1, dK2, dI2, pSGcode->cArcDir,
                                                    pSNextcode->cArcDir, ucAxisNum, 3);
                        cPointNum = 1;
                    }

                    else if ((dToolRadius > 0 && pSGcode->cArcDir == ARC_CW) ||
                             (dToolRadius < 0 &&
                              pSGcode->cArcDir == ARC_ANTI_CW)) //左刀补，第一段为顺圆弧或者右刀补，第一段为逆圆弧
                    {
                        offsetRadiusVector(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
                        offsetRadiusVector1(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
                        offsetRadiusVector2(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
                        offsetRadiusVector(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
                        cPointNum = 4;
                    }
                }
            }
        }
        else
        {
            Intersection_Of_Arc_And_Arc(dToolRadius, pSGcode->SStartPoint, pSGcode->SEndPoint, &SToolRunPoint.SS1, dK1,
                                        dI1, dK2, dI2, pSGcode->cArcDir, pSNextcode->cArcDir, ucAxisNum, 1);
            cPointNum = 1;
        }
        break;
    case 2: //伸长型
        offsetRadiusVector(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
        Intersection_Of_Quidistant_Lines(dToolRadius, Arc1_Vec, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2,
                                         ucAxisNum);
        offsetRadiusVector(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
        cPointNum = 3;

        break;
    case 3: //插入型
        offsetRadiusVector(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);
        offsetRadiusVector1(dToolRadius, Arc1_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS2, ucAxisNum);
        offsetRadiusVector2(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS3, ucAxisNum);
        offsetRadiusVector(dToolRadius, Arc2_Vec, pSGcode->SEndPoint, &SToolRunPoint.SS4, ucAxisNum);
        cPointNum = 4;
        break;
    }
    SToolRunPoint.cPointNum = cPointNum;
    return cPointNum;
}

/*************************************************
  函数名称：toolCompensation(SGCodeParse *pSGcode, SGCodeParse *pSNextGcode)
  函数功能  根据C刀补原理，计算相连运动的两条移动代码段的刀补轨迹
  输入参数：pSGcode，pSNextGcode分别是G代码解释后的轨迹结构体
  输出参数：
  返 回 值：是否完成刀补计算

  备    注：
**************************************************/
bool toolCompensation(struct SGCodeParse* pSGcode, struct SGCodeParse* pSNextGcode, uint8* ucAxisNum,
                      uint8 ucChannelNum)
{
    int8 cStageSelect = 0;    //记录刀补的阶段（1.建立 2.进行 3.撤消）
    double dToolNoseWear = 0; //刀尖磨损值
    double dToolRadius = 0;   //刀尖圆弧半径

    //指针为空返回
    if (pSGcode == NULL || pSNextGcode == NULL)
    {
        return FALSE;
    }

    //当第一段为G40，表明在上一次c刀补计算时已经完成刀补撤消补偿点的计算，因此直接记录G40位置，直接退刀到G40设置的位置
    //if (pSGcode->SModalInfo.sGd == 40 && SToolCompensationRecord->bG4142Flag[ucChannelNum])
    //{
    //	SToolRunPoint.SS1 = pSGcode->SEndPoint;

    //	SToolCompensationRecord->bG4142Flag[ucChannelNum] = FALSE;//恢复刀补建立未开始的标志

    //	SToolRunPoint.cPointNum = 1;

    //	return TRUE;
    //}

    //当前程序段未G41或G42，但刀补标志为FALSE
    if (((pSGcode->SModalInfo.sGd == 41) || (pSGcode->SModalInfo.sGd == 42)) &&
        !SToolCompensationRecord->bG4142Flag[ucChannelNum])
    {
        SToolCompensationRecord->bG4142Flag[ucChannelNum] = TRUE; //表明刀具补偿已经建立，标志位重设为1

        cStageSelect = 1; //1为刀补建立阶段
    }

    //当读进的两段数控代码的第二段为G40，计算刀补撤消的补偿点
    else if (pSNextGcode->SModalInfo.sGd == 40)
    {
        cStageSelect = 3; //3为刀补撤消阶段
    }

    else //刀补进行
    {
        cStageSelect = 2; //2为刀补进行阶段
    }

    //获取刀尖磨损值，改变实际计算中用到的刀具半径
    dToolNoseWear = getToolNoseWear(ucChannelNum, pSGcode->ucToolCompNum);
    dToolRadius = getToolRadius(ucChannelNum, pSGcode->ucToolCompNum) - dToolNoseWear;

    if (pSGcode->SModalInfo.sGd == 42)
    {
        dToolRadius = -dToolRadius; //右刀补半径取负
    }

    //直线接直线
    if ((pSGcode->SModalInfo.sGa == 0 || pSGcode->SModalInfo.sGa == 1) &&
        (pSNextGcode->SModalInfo.sGa == 0 || pSNextGcode->SModalInfo.sGa == 1))
    {
        LineToLine(pSGcode, pSNextGcode, dToolRadius, cStageSelect, ucAxisNum);
        return TRUE;
    }

    //直接接圆弧
    else if ((pSGcode->SModalInfo.sGa == 0 || pSGcode->SModalInfo.sGa == 1) &&
             (pSNextGcode->SModalInfo.sGa == 2 || pSNextGcode->SModalInfo.sGa == 3))
    {
        LineToArc(pSGcode, pSNextGcode, dToolRadius, cStageSelect, ucAxisNum);
        return TRUE;
    }

    //圆弧接直线
    else if ((pSGcode->SModalInfo.sGa == 2 || pSGcode->SModalInfo.sGa == 3) &&
             (pSNextGcode->SModalInfo.sGa == 0 || pSNextGcode->SModalInfo.sGa == 1))
    {
        ArcToLine(pSGcode, pSNextGcode, dToolRadius, cStageSelect, ucAxisNum);
        return TRUE;
    }

    //圆弧接圆弧
    else if ((pSGcode->SModalInfo.sGa == 2 || pSGcode->SModalInfo.sGa == 3) &&
             (pSNextGcode->SModalInfo.sGa == 2 || pSNextGcode->SModalInfo.sGa == 3))
    {

        ArcToArc(pSGcode, pSNextGcode, dToolRadius, cStageSelect, ucAxisNum);
        return TRUE;
    }
    else
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM409);
        return FALSE;
    }

    return FALSE;
}

/*************************************************
  函数名称：saveToolCode(struct SGCodeParse *pSGcode, int8 cPointNum)
  函数功能  根据toolCompensation()计算出的过渡点保存到刀补之后的缓冲区
  输入参数：SGCodeParse *pSGcode继承原Gcode不变的信息， int8 cPointNum需要保存的点数
  输出参数：
  返 回 值：无

  备    注：
**************************************************/
void saveToolCode(struct SGCodeParse* pSGcode, uint8* ucAxisNum, uint8 ucChannelNum)
{
    if (pSGcode == NULL || ucAxisNum == NULL)
    {
        return;
    }

    //在保存过渡线段时，判断过渡线段是否小于一个脉冲，若小于则进行优化，防止出现小线段(6.14 XHJ)
    judgeNewLineLength(ucAxisNum, ucChannelNum);

    if (SToolRunPoint.cPointNum == 1)
    {
        //根据需要的补偿点个数，执行GcodeToTCode次数，往刀补补偿缓冲区存放刀补之后的补偿点
        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS1,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS1;
        SToolCompensationRecord->bPointFlag[ucChannelNum] = FALSE; //恢复标志位
    }
    else if (SToolRunPoint.cPointNum == 2) //存放2个数据
    {
        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS1,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS1; //更新刀补前一个补偿点
        SToolCompensationRecord->bPointFlag[ucChannelNum] = TRUE;                  //已存放第一个点

        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS2,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS2; //更新刀补前一个补偿点
        SToolCompensationRecord->bPointFlag[ucChannelNum] = FALSE;                 //恢复标志位
    }
    else if (SToolRunPoint.cPointNum == 3) //存放3个数据
    {
        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS1,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS1; //更新刀补前一个补偿点
        SToolCompensationRecord->bPointFlag[ucChannelNum] = TRUE;                  //已存放第一个点

        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS2,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS2; //更新刀补前一个补偿点

        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS3,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS3; //更新刀补前一个补偿点
        SToolCompensationRecord->bPointFlag[ucChannelNum] = FALSE;                 //恢复标志位
    }
    else if (SToolRunPoint.cPointNum == 4) //存放4个数据
    {
        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS1,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS1; //更新刀补前一个补偿点
        SToolCompensationRecord->bPointFlag[ucChannelNum] = TRUE;                  //已存放第一个点

        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS2,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS2; //更新刀补前一个补偿点

        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS3,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS3; //更新刀补前一个补偿点

        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS4,
                     SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS4; //更新刀补前一个补偿点
        SToolCompensationRecord->bPointFlag[ucChannelNum] = FALSE;                 //恢复标志位
    }
}

/*************************************************
  函数名称：JudgeNoMoveStruct(struct SGCodeParse *pSGcode)
  函数功能  判断G代码中是否出现了点没移动的情况
  输入参数：SGCodeParse *pSGcode当前G代码结构体的信息
  输出参数：
  返 回 值：TRUE  or  FALSE

  备    注：
**************************************************/
bool JudgeNoMoveStruct(struct SGCodeParse* pSGcode)
{
    uint8 ucCounter;

    if (pSGcode == NULL)
    {
        return FALSE;
    }

    //遍历所有轴，查看是否起点终点相同
    for (ucCounter = 0; ucCounter < AXIS_NUM_MAX; ucCounter++)
    {
        if (!isZero(pSGcode->SStartPoint.dArray[ucCounter] - pSGcode->SEndPoint.dArray[ucCounter]))
        {
            return FALSE;
        }
    }

    return TRUE;
}

/*************************************************
  函数名称：Init_ToolCompensation_Flag()
  函数功能  初始化刀补的标志位
  输入参数：指向G代码解析结构体的指针的地址
  输出参数：
  返 回 值：
  备    注：
**************************************************/
//初始化标志位
void toolCompensationInit(uint8 ucChannelNum)
{
    //memset(&SToolCompensationRecord-> 0, sizeof(SToolCompensationRecord->);		//2023/12/7	陈敏聪注释	原因：会把另外通道的标志位等都清除
    memset(&SToolCompensationRecord->SGcodeRecord[ucChannelNum], 0,
           sizeof(struct SGCodeParse)); //陈敏聪	2023/12/7增	清除本通道的标志位等
    memset(&SToolCompensationRecord->TcodePrevPoint[ucChannelNum], 0, sizeof(SPointDouble));
    memset(&SToolCompensationRecord->bG4142Flag[ucChannelNum], 0, sizeof(bool));
    memset(&SToolCompensationRecord->bPointFlag[ucChannelNum], 0, sizeof(bool));
    memset(&SToolCompensationRecord->bTurnBackFlag[ucChannelNum], 0, sizeof(bool));
    memset(&SToolCompensationRecord->ucNoMoveCounter[ucChannelNum], 0, sizeof(uint8));
    memset(&SToolCompensationRecord->bFristSavePrePointFlag[ucChannelNum], 0, sizeof(bool));

    g_SChannel[ucChannelNum].bGcodeWriteOverFlag = FALSE; //开机、新开始需要置FALSE
    g_SChannel[ucChannelNum].bToolWriteOverFlag = FALSE;  //冼浩基 2023/05/20 初始化刀补写入完毕标志位
}

//不同平面坐标映射
void CoordinateMap(struct SGCodeParse* pSGcode, uint8* ucAxisNum)
{
    if (pSGcode == NULL || ucAxisNum == NULL)
    {
        return;
    }

    if (getBits(pSGcode->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
    { //极坐标插补
        ucAxisNum[0] = g_SSystemPara.ucPolarInterpolationAxis[0];
        ucAxisNum[1] = g_SSystemPara.ucPolarInterpolationAxis[1];
        return;
    }
    else if (getBits(pSGcode->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_CYLINDRICAL)
    {                                                            //圆柱插补
        switch (getBits(pSGcode->unCommandType, CMD_PLANE_ADDR)) //读取指令类型
        {
        case CMD_PLANE_G18: //G18
        default:            //默认G18
            ucAxisNum[0] = AXIS_Z;
            ucAxisNum[1] = AXIS_C;
            break;
        case CMD_PLANE_G17: //G17
            ucAxisNum[0] = AXIS_X;
            ucAxisNum[1] = AXIS_C;
            break;
        case CMD_PLANE_G19: //G19
            ucAxisNum[0] = AXIS_C;
            ucAxisNum[1] = AXIS_Z;
            break;
        }
        return;
    }

    //不同平面映射相应轴坐标
    switch (pSGcode->SModalInfo.sGc)
    {
    case 17:
        ucAxisNum[0] = AXIS_X;
        ucAxisNum[1] = AXIS_Y;
        break;
    case 18:
        ucAxisNum[0] = AXIS_Z;
        ucAxisNum[1] = AXIS_X;
        break;
    case 19:
        ucAxisNum[0] = AXIS_Y;
        ucAxisNum[1] = AXIS_Z;
        break;
    default:
        ucAxisNum[0] = AXIS_Z;
        ucAxisNum[1] = AXIS_X;
        break;
    }
}

//根据不同的假象刀尖号计算横轴偏移值
double getHorizontalAxisOffset(uint8 ucToolNum, double dToolRadius)
{
    double dRetVal = 0; //返回值

    switch (ucToolNum)
    {
    case 1:
    case 4:
    case 5:
        dRetVal = dToolRadius;
        break;

    case 2:
    case 3:
    case 7:
        dRetVal = -dToolRadius;
        break;

    case 6:
    case 8:
        dRetVal = 0;
        break;

    default:
        dRetVal = 0;
        break;
    }
    return dRetVal;
}

//根据不同的假象刀尖号计算纵轴偏移值
double getVerticalAxisOffset(uint8 ucToolNum, double dToolRadius)
{
    double dRetVal = 0; //返回值

    switch (ucToolNum)
    {
    case 1:
    case 2:
    case 6:
        dRetVal = dToolRadius;
        break;

    case 3:
    case 4:
    case 8:
        dRetVal = -dToolRadius;
        break;

    case 5:
    case 7:
        dRetVal = 0;
        break;

    default:
        dRetVal = 0;
        break;
    }
    return dRetVal;
}

//刀补前根据假象刀尖号更改坐标
int8 modifyCoorByToolNum(struct SGCodeParse* pGcode, struct SGCodeParse* SGetGode, uint8* ucAxisNum, uint8 ucChannelNum)
{
    double dHorOffset = 0; //横轴偏移量
    double dVerOffset = 0; //纵轴偏移量

    if (pGcode == NULL)
    {
        return -1;
    }

    *SGetGode = *pGcode; //复制原来结构体的值

    //有刀补指令时，根据假象刀尖号偏移坐标
    if (pGcode->SModalInfo.sGd == 41 || pGcode->SModalInfo.sGd == 42)
    {
        uint8 ucToolNoseID = getToolNoseNum(ucChannelNum, pGcode->ucToolCompNum);
        double dToolRadius = getToolRadius(ucChannelNum, pGcode->ucToolCompNum);
        dToolRadius = fabs(dToolRadius); //坐标按刀尖方向偏移时，全部取正，跟输入半径正负无关(6.10)

        //获取偏移量
        dHorOffset = getHorizontalAxisOffset(ucToolNoseID, dToolRadius);
        dVerOffset = getVerticalAxisOffset(ucToolNoseID, dToolRadius);

        //偏移起点和终点坐标
        SGetGode->SStartPoint.dArray[ucAxisNum[0]] = pGcode->SStartPoint.dArray[ucAxisNum[0]] + dHorOffset;
        SGetGode->SStartPoint.dArray[ucAxisNum[1]] = pGcode->SStartPoint.dArray[ucAxisNum[1]] + dVerOffset;

        SGetGode->SEndPoint.dArray[ucAxisNum[0]] = pGcode->SEndPoint.dArray[ucAxisNum[0]] + dHorOffset;
        SGetGode->SEndPoint.dArray[ucAxisNum[1]] = pGcode->SEndPoint.dArray[ucAxisNum[1]] + dVerOffset;

        //如果为圆弧需偏移圆心
        if (pGcode->SModalInfo.sGa == 2 || pGcode->SModalInfo.sGa == 3)
        {
            SGetGode->SCenterPoint.dArray[ucAxisNum[0]] = pGcode->SCenterPoint.dArray[ucAxisNum[0]] + dHorOffset;
            SGetGode->SCenterPoint.dArray[ucAxisNum[1]] = pGcode->SCenterPoint.dArray[ucAxisNum[1]] + dVerOffset;
        }
    }
    return 1;
}

/*************************************************
  函数名称：judgeNewLineLength(SPointDouble SStartPoint, SPointDouble SEndPoint)
  函数功能  判断增加的转接线段是否小于一个脉冲
  输入参数：SStartPoint 起点，SEndPoint 终点，ucAxisNum 轴映射参数
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 judgeNewLineLength(uint8* ucAxisNum, uint8 ucChannelNum)
{
    if (ucAxisNum == NULL)
    {
        return -1;
    }

    double dLength1 = 0;
    double dLength2 = 0;
    double dPulseNum = g_SSystemParaUnion.SSystemParaInfo.SAxisPara[ucChannelNum].dPulseToMm; //脉冲当量值

    //只有过渡点个数为2和3个的情况下，才会出现接线段小于1个脉冲的情况，4个转接点时在刀补建立且为插入型时才会有，此时补偿线长度一定大于1个脉冲长度
    if (SToolRunPoint.cPointNum == 2)
    {
        dLength1 = sqrt(pow((SToolRunPoint.SS1.dArray[ucAxisNum[0]] - SToolRunPoint.SS2.dArray[ucAxisNum[0]]), 2) +
                        pow((SToolRunPoint.SS1.dArray[ucAxisNum[1]] - SToolRunPoint.SS2.dArray[ucAxisNum[1]]), 2));
        if (dLength1 < dPulseNum || isZero(dLength1 - dPulseNum))
        {
            SToolRunPoint.cPointNum = 1;
        }
    }

    else if (SToolRunPoint.cPointNum == 3)
    {
        dLength1 = sqrt(pow((SToolRunPoint.SS1.dArray[ucAxisNum[0]] - SToolRunPoint.SS2.dArray[ucAxisNum[0]]), 2) +
                        pow((SToolRunPoint.SS1.dArray[ucAxisNum[1]] - SToolRunPoint.SS2.dArray[ucAxisNum[1]]), 2));
        dLength2 = sqrt(pow((SToolRunPoint.SS2.dArray[ucAxisNum[0]] - SToolRunPoint.SS3.dArray[ucAxisNum[0]]), 2) +
                        pow((SToolRunPoint.SS2.dArray[ucAxisNum[1]] - SToolRunPoint.SS3.dArray[ucAxisNum[1]]), 2));

        //第一段插入长度小于等于一个脉冲，第二段长度大于一个脉冲时，将补偿点减少为两个，只用计算的第二段轨迹过渡
        if ((dLength1 < dPulseNum || isZero(dPulseNum)) && dLength2 > dPulseNum)
        {
            SToolRunPoint.SS1 = SToolRunPoint.SS2;
            SToolRunPoint.SS2 = SToolRunPoint.SS3;
            SToolRunPoint.cPointNum = 2;
        }

        //第二段插入长度小于等于一个脉冲，第一段长度大于一个脉冲时，将补偿点减少为两个，只用计算的第一段轨迹过渡
        else if ((dLength2 < dPulseNum || isZero(dLength2 - dPulseNum)) && dLength1 > dPulseNum)
        {
            SToolRunPoint.cPointNum = 2;
        }

        //两段都小于一个脉冲长度，补偿点减少为1个，只用中间点过渡
        else if ((dLength2 < dPulseNum || isZero(dLength2 - dPulseNum)) &&
                 (dLength1 < dPulseNum || isZero(dLength1 - dPulseNum)))
        {
            SToolRunPoint.SS1 = SToolRunPoint.SS2;
            SToolRunPoint.cPointNum = 1;
        }

        //其他大于一个脉冲的情况，不作额外处理，按原计算轨迹走
    }

    return 1;
}

/*************************************************
  函数名称：void ToolCompensation_Caculate(uint8 ucChannelNum)
  函数功能：消费已定型的 G 代码轨迹，执行现有刀补流程并输出刀补段。
  输入参数：ucChannelNum 有效通道号。
  输出参数：刀补缓冲、读取和预读游标，以及通道刀补状态。
  返 回 值：无；无数据、无容量或待定尾段时返回等待，错误经现有报警状态报告。
  备    注：带未处理 D/L 的尾段不得消费或预读；确认程序结束仍未解决请求时报警。
**************************************************/
void ToolCompensation_Caculate(uint8 ucChannelNum)
{
    /* 在取当前槽并映射坐标前检查有效数据，禁止错误后继续规划。 */
    if (isErrorHasBeenWritten() || getSGCodeParsePointerBufRes(GCODE_READ, ucChannelNum) <= 0)
        return;
    bool bGetNextGcodeFlag = FALSE;         //获取到下一段移动代码段标志位
    bool bAheadReadEndFlag = FALSE;         //预读指针已移动到G代码最后一行标志位
    struct SGCodeParse* pSGcode = NULL;     //G代码当前行指针
    struct SGCodeParse* pSNextGcode = NULL; //G代码下一行指针
    struct SGCodeParse SCurGcode;           //当前G代码段信息
    struct SGCodeParse SNextGcode;          //下一段G代码信息
    uint8 ucAxisNum[2];                     //两个轴 元素0为横轴  元素1位纵轴
    uint8 ucGCodeReadIndex = 0;             //G代码当前读指针索引
    uint8 ucSToolReadAheadIndex = 0;        //刀补预读指针索引

    struct _SQuickHandleFlag SQuickHandleFlag; //4/18 陈敏聪 刀补循环处理
    SQuickHandleFlag.cNeedQuickHandleFlag = FALSE;
    SQuickHandleFlag.cCycleCounter = 0;

    //刀补缓存区不能存入数据时，不进行刀补处理，退出
    if (getSToolCompPointerBufRes(STOOL_WRITE, ucChannelNum) <= 0)
    {
        return;
    }

    pSGcode = getSGCodeParsePointerByType(GCODE_READ, ucChannelNum); //初始化第一个刀补数据
    CoordinateMap(pSGcode, ucAxisNum);                               //映射不同平面坐标

    //根据假象刀尖号偏移坐标,错误返回
    if (modifyCoorByToolNum(pSGcode, &SCurGcode, ucAxisNum, ucChannelNum) == -1)
    {
        return;
    }

    //刀补运行时将G代码起点赋值给前一个刀补点	陈敏聪	2023/5/27
    if (!SToolCompensationRecord->bFristSavePrePointFlag[ucChannelNum])
    {
        if (getSGCodeParsePointerBufRes(GCODE_READ, ucChannelNum) > 0)
        {
            SToolCompensationRecord->bFristSavePrePointFlag[ucChannelNum] = TRUE;
            SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = pSGcode->SStartPoint; //记录起点
        }
    }

    do
    {
        SQuickHandleFlag.cNeedQuickHandleFlag = FALSE; //陈敏聪 5/12
        if (SQuickHandleFlag.cCycleCounter != 0)
        {
            pSGcode = getSGCodeParsePointerByType(GCODE_READ, ucChannelNum); //初始化第一个刀补数据

            CoordinateMap(pSGcode, ucAxisNum); //映射不同平面坐标

            //根据假象刀尖号偏移坐标,错误返回
            if (modifyCoorByToolNum(pSGcode, &SCurGcode, ucAxisNum, ucChannelNum) == -1)
            {
                return;
            }
        }

        //G代码只有一段且已经解析完成

        /* 尾段虽已占槽但终点尚未定型；等待后继段裁剪，真正结束仍有请求则报警。 */
        if (pSGcode->dRoundedRadius != 0.0 || pSGcode->dChamferLen != 0.0)
        {
            if (g_SChannel[ucChannelNum].bGcodeWriteOverFlag || pSGcode->bLastLineMark)
                parseGCodeErrorB(pSGcode->unGCodeLineNum, 283);
            return;
        }
        if (getSGCodeParsePointerBufRes(GCODE_READ, ucChannelNum) == 1 &&
            (g_SChannel[ucChannelNum].bGcodeWriteOverFlag || pSGcode->bLastLineMark))
        {
            //如果当前为G41、G42模态，且最后一段为非移动指令(如程序无取消刀补指令G40，最后一段代码为M30，刀补终点和最终G代码终点不相同，此时赋刀补最后终点)
            if ((pSGcode->SModalInfo.sGd == G41 || pSGcode->SModalInfo.sGd == G42) && JudgeNoMoveStruct(pSGcode))
            {
                GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum],
                             SToolCompensationRecord->TcodePrevPoint[ucChannelNum],
                             SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);
            }

            // 其他情况
            else
            {
                // 最后一段是无移动 G40：只清状态，不产生取消运动
                if (pSGcode->SModalInfo.sGd == G40 && JudgeNoMoveStruct(pSGcode))
                {
                    GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum],
                                 SToolCompensationRecord->TcodePrevPoint[ucChannelNum],
                                 SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);

                    SToolCompensationRecord->bG4142Flag[ucChannelNum] = FALSE;
                }
                // G40 带移动，或者其他有移动的最后一段
                else
                {
                    GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], pSGcode->SEndPoint,
                                 SToolCompensationRecord->bPointFlag[ucChannelNum], ucAxisNum, ucChannelNum);

                    SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = pSGcode->SEndPoint;

                    if (pSGcode->SModalInfo.sGd == G40)
                    {
                        SToolCompensationRecord->bG4142Flag[ucChannelNum] = FALSE;
                    }
                }
            }

            g_SChannel[ucChannelNum].bToolWriteOverFlag = TRUE;
            moveSGCodeParsePointer(GCODE_READ, ucChannelNum);       //移动G代码缓冲区读指针
            moveSGCodeParsePointer(STOOL_READ_AHEAD, ucChannelNum); //移动刀补预读指针
            SQuickHandleFlag.cNeedQuickHandleFlag = FALSE;          //解析完成不需要循环处理
            SQuickHandleFlag.cCycleCounter += 1;
        }

        //G代码缓冲区数据大于两个且内存足够存放算出的补偿点数
        if (getSGCodeParsePointerBufRes(GCODE_READ, ucChannelNum) >= 2 &&
            getSToolCompPointerBufRes(STOOL_WRITE, ucChannelNum) >= 4)
        {
            //当前指针指向程序段无刀补指令，或者当前为G40指令但刀补已取消，将数据直接继承并存入刀补缓存区
            if ((pSGcode->SModalInfo.sGd != 41 && pSGcode->SModalInfo.sGd != 42 && pSGcode->SModalInfo.sGd != 40) ||
                (pSGcode->SModalInfo.sGd == 40 && !SToolCompensationRecord->bG4142Flag[ucChannelNum]))
            {
                //直接保存当前段入刀补缓存区
                GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], pSGcode->SEndPoint, 0,
                             ucAxisNum, ucChannelNum);
                SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = pSGcode->SEndPoint;

                SQuickHandleFlag.cNeedQuickHandleFlag = TRUE; //可以循环处理
                SQuickHandleFlag.cCycleCounter += 1;

                moveSGCodeParsePointer(GCODE_READ, ucChannelNum);       //移动G代码缓冲区读指针
                moveSGCodeParsePointer(STOOL_READ_AHEAD, ucChannelNum); //移动刀补预读指针
            }

            //当前指针指向程序段有刀补指令
            else
            {
                //指针指向程序段为非移动代码段
                if (JudgeNoMoveStruct(pSGcode))
                {

                    //直接保存当前段入刀补缓存区
                    GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum],
                                 SToolCompensationRecord->TcodePrevPoint[ucChannelNum], 0, ucAxisNum, ucChannelNum);
                    // 在无移动块分支中，遇到 G40 且刀补标志仍有效时
                    if (pSGcode->SModalInfo.sGd == G40 && SToolCompensationRecord->bG4142Flag[ucChannelNum] == TRUE)
                    {
                        SToolCompensationRecord->bG4142Flag[ucChannelNum] = FALSE;
                    }
                    //获取当前G代码读指针和刀补预读指针索引
                    ucGCodeReadIndex = getSGCodeParsePointerIndex(GCODE_READ, ucChannelNum);
                    ucSToolReadAheadIndex = getSGCodeParsePointerIndex(STOOL_READ_AHEAD, ucChannelNum);

                    SQuickHandleFlag.cNeedQuickHandleFlag = TRUE; //可以循环处理 陈敏聪	5/12
                    SQuickHandleFlag.cCycleCounter += 1;

                    moveSGCodeParsePointer(GCODE_READ, ucChannelNum); //移动G代码缓冲区读指针

                    //两个读索引相等时，保存非移动代码需同时移动预读指针位置
                    if (ucGCodeReadIndex == ucSToolReadAheadIndex)
                    {
                        moveSGCodeParsePointer(STOOL_READ_AHEAD, ucChannelNum);
                    }

                    //非移动代码计数大于0时，消除非移动代码计数值
                    if (SToolCompensationRecord->ucNoMoveCounter[ucChannelNum] > 0)
                    {
                        SToolCompensationRecord->ucNoMoveCounter[ucChannelNum]--;
                    }
                }

                //指针指向程序段为移动代码段
                else
                {
                    SQuickHandleFlag.cNeedQuickHandleFlag = FALSE; //不需要循环处理 陈敏聪	5/12

                    //如果回退标志位为TRUE，说明在刀补过程中出现了连续的超过规定数量的非移动代码段
                    if (SToolCompensationRecord->bTurnBackFlag[ucChannelNum])
                    {
                        //根据保存的上段移动轨迹，和当前段移动轨迹进行刀补计算
                        if (toolCompensation(&SToolCompensationRecord->SGcodeRecord[ucChannelNum], &SCurGcode,
                                             ucAxisNum, ucChannelNum))
                        {
                            //保存第一个标志置TRUE
                            SToolCompensationRecord->bPointFlag[ucChannelNum] = TRUE;

                            saveToolCode(&SCurGcode, ucAxisNum, ucChannelNum);
                        }

                        //复位标志位
                        SToolCompensationRecord->bTurnBackFlag[ucChannelNum] = FALSE;
                    }

                    //如果当前段为G40，下段轨迹移动到该段终点
                    if (pSGcode->SModalInfo.sGd == 40)
                    {
                        GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], pSGcode->SEndPoint,
                                     FALSE, ucAxisNum, ucChannelNum);
                        SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = pSGcode->SEndPoint;
                        moveSGCodeParsePointer(GCODE_READ, ucChannelNum);          //移动G代码读指针
                        moveSGCodeParsePointer(STOOL_READ_AHEAD, ucChannelNum);    //移动刀补预读指针
                        SToolCompensationRecord->bG4142Flag[ucChannelNum] = FALSE; //恢复刀补建立未开始的标志
                    }

                    else
                    {
                        //如果下一段为G代码最后一段
                        if (getSGCodeParsePointerBufRes(STOOL_READ_AHEAD, ucChannelNum) == 1 &&
                            g_SChannel[ucChannelNum].bGcodeWriteOverFlag)
                        {
                            bAheadReadEndFlag = TRUE; //预读指针已移动到G代码最后一行标志位置TRUE
                        }

                        //如果不为G代码最后一段
                        else
                        {
                            //移动刀补预读指针，获取下一段移动代码
                            while (getSGCodeParsePointerBufRes(STOOL_READ_AHEAD, ucChannelNum) > 1)
                            {
                                struct SGCodeParse* candidate = getSGCodeParsePointerByIndex(
                                    getSGCodeParsePointerIndex(STOOL_READ_AHEAD, ucChannelNum) + 1, ucChannelNum);
                                if (candidate->dRoundedRadius != 0.0 || candidate->dChamferLen != 0.0)
                                    return; /* 先检查再移动预读游标，防止保存尚会变化的几何。 */
                                moveSGCodeParsePointer(STOOL_READ_AHEAD, ucChannelNum); //移动刀补预读指针到下一段
                                pSNextGcode = getSGCodeParsePointerByType(STOOL_READ_AHEAD, ucChannelNum);

                                //查找下一段代码段是否为非移动代码,跳出循环
                                if (!JudgeNoMoveStruct(pSNextGcode))
                                {
                                    bGetNextGcodeFlag = TRUE; //标志置TRUE
                                    break;
                                }

                                else
                                {
                                    bGetNextGcodeFlag = FALSE; //获取到下一段移动代码段标志置FALSE
                                    SToolCompensationRecord->ucNoMoveCounter[ucChannelNum]++; //非移动代码段数+1
                                }

                                //如果非移动代码段数超过规定的数量,跳出循环
                                if (SToolCompensationRecord->ucNoMoveCounter[ucChannelNum] >= MAX_COUNTER_NO_MOVE)
                                    break;
                            }
                        }

                        //查找到了下一段移动代码段，进行刀补处理
                        if (bGetNextGcodeFlag)
                        {
                            //根据假象刀尖号偏移坐标为刀尖中心轨迹
                            modifyCoorByToolNum(pSNextGcode, &SNextGcode, ucAxisNum, ucChannelNum);

                            //刀补计算
                            if (toolCompensation(&SCurGcode, &SNextGcode, ucAxisNum, ucChannelNum))
                            {
                                //将刀补计算出的过渡点存于刀补缓冲区
                                saveToolCode(&SCurGcode, ucAxisNum, ucChannelNum);

                                moveSGCodeParsePointer(GCODE_READ, ucChannelNum); //移动G代码缓冲区读指针
                            }
                        }

                        //查找不到移动代码段且超过了规定的数量,或者查找直到G代码最后一段也找不到移动代码
                        else if ((!bGetNextGcodeFlag &&
                                  SToolCompensationRecord->ucNoMoveCounter[ucChannelNum] >= MAX_COUNTER_NO_MOVE) ||
                                 bAheadReadEndFlag)
                        {
                            //当前轨迹在终点处按偏移一个半径矢量处理
                            dealMaxCounterNoMoveTrack(&SCurGcode, ucAxisNum, ucChannelNum);

                            //如果是查找不到移动代码段且超过了规定的数量的情况
                            if (!bAheadReadEndFlag)
                            {
                                //保存当前段代码信息，后面作回退处理
                                SToolCompensationRecord->SGcodeRecord[ucChannelNum] = SCurGcode;

                                //回退标志置为TRUE
                                SToolCompensationRecord->bTurnBackFlag[ucChannelNum] = TRUE;
                            }

                            //移动G代码读指针
                            moveSGCodeParsePointer(GCODE_READ, ucChannelNum);
                        }
                    }
                }
            }
        }
        else
        {
            break;
        }

    }
    //G代码循环处理功能开 以及 当前G代码行可以循环处理
    while (g_SSystemPara.bGcodeQuickHandleModeEnabled &&
           (SQuickHandleFlag.cNeedQuickHandleFlag == TRUE &&
            SQuickHandleFlag.cCycleCounter < g_SSystemPara.sToolComp_MultiLineHandleMaxNum &&
            getSToolCompPointerBufRes(STOOL_WRITE, ucChannelNum) > 0));
}

/*************************************************
  函数名称：dealMaxCounterNoMoveTrack(uint8 ucChannelNum)
  函数功能  处理连续出现的超过规定数量非移动代码段
  输入参数：通道号
  输出参数：
  返 回 值：返回1 计算完成；返回-1 计算出错
  备    注：
**************************************************/
int8 dealMaxCounterNoMoveTrack(struct SGCodeParse* pSGcode, uint8* ucAxisNum, uint8 ucChannelNum)
{
    SVector SCurGCodeVec;   //当前轨迹方向向量
    double dToolRadius = 0; //刀具半径
    double dK = 0;          //圆心坐标在横轴方向上与起点的差值
    double dI = 0;          //圆心坐标在纵轴方向上与起点的差值

    if (pSGcode == NULL)
    {
        return -1;
    }

    //计算补偿半径
    if (pSGcode->SModalInfo.sGd == 41)
    {
        dToolRadius =
            getToolRadius(ucChannelNum, pSGcode->ucToolCompNum) - getToolNoseWear(ucChannelNum, pSGcode->ucToolCompNum);
    }

    //G42时刀具半径取负值
    else if (pSGcode->SModalInfo.sGd == 42)
    {
        dToolRadius = -fabs(getToolRadius(ucChannelNum, pSGcode->ucToolCompNum) -
                            getToolNoseWear(ucChannelNum, pSGcode->ucToolCompNum));
    }

    //判断当前段轨迹类型，进行相应处理
    //如果为直线
    if (pSGcode->SModalInfo.sGa == 0 || pSGcode->SModalInfo.sGa == 1)
    {
        //计算直线的方向矢量
        getLineVector(pSGcode->SStartPoint, pSGcode->SEndPoint, &SCurGCodeVec, ucAxisNum);
    }

    //如果为圆弧段
    else if (pSGcode->SModalInfo.sGa == 2 || pSGcode->SModalInfo.sGa == 3)
    {
        dK = pSGcode->SStartPoint.dArray[ucAxisNum[0]] - pSGcode->SCenterPoint.dArray[ucAxisNum[0]];
        dI = pSGcode->SStartPoint.dArray[ucAxisNum[1]] - pSGcode->SCenterPoint.dArray[ucAxisNum[1]];

        //计算圆弧在终点处的方向矢量
        getArcVectorAtEnd(pSGcode->SStartPoint, pSGcode->SEndPoint, dK, dI, &SCurGCodeVec, pSGcode->cArcDir, ucAxisNum);
    }
    else
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM409);
        return -1;
    }

    //偏移一个半径矢量
    offsetRadiusVector(dToolRadius, SCurGCodeVec, pSGcode->SEndPoint, &SToolRunPoint.SS1, ucAxisNum);

    //保存到缓存区中
    GcodeToTCode(pSGcode, SToolCompensationRecord->TcodePrevPoint[ucChannelNum], SToolRunPoint.SS1, FALSE, ucAxisNum,
                 ucChannelNum);

    //记录下段轨迹起点
    SToolCompensationRecord->TcodePrevPoint[ucChannelNum] = SToolRunPoint.SS1;

    return 1;
}

//打开文件(测试用)
void TCODE_OPEN_FILE(uint8 ucChannelNum)
{
    FILE* file = NULL;
    switch (ucChannelNum)
    {
    case 0:
        file = fopen("./outputFile/Cnc0.txt", "w");
        break;
    case 1:
        file = fopen("./outputFile/Cnc1.txt", "w");
        break;
    default:
        break;
    }

    if (!file)
    {
        printf("打开文件错误!\n");
        exit(0);
    }
    fclose(file);

    switch (ucChannelNum)
    {
    case 0:
        file = fopen("./outputFile/Cnc0.txt", "a"); //追加方式方式打开
        break;
    case 1:
        file = fopen("./outputFile/Cnc1.txt", "a");
        break;
    default:
        break;
    }

    if (!file)
    {
        exit(0);
    }

    switch (ucChannelNum)
    {
    case 0:
        channelFile1 = file;
        break;
    case 1:
        channelFile2 = file;
        break;
    default:
        break;
    }
}

//关闭文件(测试用)
void CloseFile(uint8 ucChannelNum)
{
    FILE* file = NULL;
    lineNum1 = 0;
    lineNum2 = 0;
    switch (ucChannelNum)
    {
    case 0:
        file = channelFile1;
        break;
    case 1:
        file = channelFile2;
        break;
    default:
        break;
    }

    if (file)
    {
        fclose(file);
        file = NULL;
    }
}

//根据通道打印数据(测试用)
void PrintDataByChannel(uint8 ucChannelNum)
{
    //	FILE* file1 = NULL;
    //	int16 idx = (g_SRingBuff[ucChannelNum].nSToolIndex + TOOLCOMPBUF_MAX) % TOOLCOMPBUF_MAX;// (g_SRingBuff[ucChannelNum].sSToolIndex + TOOLCOMPBUF_MAX) % TOOLCOMPBUF_MAX;  //前一个缓冲的索引
    //	int32 *lineNum = NULL;
    //	switch (ucChannelNum)
    //	{
    //	case 0:
    //		file1 = channelFile1;
    //		lineNum = &lineNum1;
    //		break;
    //	case 1:
    //		file1 = channelFile2;
    //		lineNum = &lineNum2;
    //		break;
    //	default:
    //		break;
    //	}
    //
    //	*lineNum += 1;
    //	//fprintf(file1, "行号%d\n", g_SRingBuff[ucChannelNum].SGCodeParseBuf[idx].unGCodeLineNum);
    //	fprintf(file1, "No%d:起点xyz:%8f %8f %8f\n", *lineNum
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SStartPoint.dArray[AXIS_X]
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SStartPoint.dArray[AXIS_Y]
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SStartPoint.dArray[AXIS_Z]);
    //
    //	fprintf(file1, "终点xyz:%8f %8f %8f\n"
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SEndPoint.dArray[AXIS_X]
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SEndPoint.dArray[AXIS_Y]
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SEndPoint.dArray[AXIS_Z]);
    //
    //	fprintf(file1, "圆心xyz:%8f %8f %8f\n"
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SCenterPoint.dArray[AXIS_X]
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SCenterPoint.dArray[AXIS_Y]
    //		, g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].SCenterPoint.dArray[AXIS_Z]);
    //
    //	fprintf(file1, "圆弧方向:%d\n", g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].cArcDir);
    //	fprintf(file1, "实体类型：%d\t ,运动指令类型：%d\t,程序状态指令类型：%d\t\n", g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].EntityType,
    //		getBits(g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].unCommandType, CMD_MOTION_ADDR),
    //		getBits(g_SRingBuff[ucChannelNum].SToolCompensationBuf[idx].unCommandType, CMD_PROGRAM_ADDR) >> 8);
}
