#include "G2.h"
#include "GeneralFunctions.h"
#include "Motion.h"
#include "lookAhead.h"
#include <stdlib.h>
#include "CTrajPlanning.h"
#include <math.h>
#include "ringbuff.h"
#include "Channel.h"
//struct SG2 g_SG2;
#include "Config.h"
#include "ErrorCompensation.h"
#if HANDWHEEL_SIM_DISTANCE_ENABLE
/*************************************************
  函数名称：calculateHandwheelArcGeometry
  函数功能：保留有效圆心定义的整圆，其余圆弧沿用原半径求解
  输入参数：pArc - 已加载的 G2/G3 几何
  输出参数：pArc - 圆心及扫角，保留 R 符号用于暂停后重建优劣弧
  返 回 值：1 - 有效；-1 - 几何不明确
  备    注：仅距离模拟调用，首尾同点必须有匹配半径的有效圆心
****************************************/
static int8 calculateHandwheelArcGeometry(struct SG2* pArc)
{
    uint8 ucHorizontal = pArc->ucAxis[0], ucVertical = pArc->ucAxis[1];
    double dRadius = fabs(pArc->dRadius);
    if (ucHorizontal >= AXIS_NUM_MAX || ucVertical >= AXIS_NUM_MAX
        || !isfinite(dRadius) || dRadius <= HWSIM_DISTANCE_EPS)
    {
        return -1;
    }
    if (pArc->SStartPoint.dArray[ucHorizontal] == pArc->SEndPoint.dArray[ucHorizontal]
        && pArc->SStartPoint.dArray[ucVertical] == pArc->SEndPoint.dArray[ucVertical])
    {
        double dActualRadius = hypot(pArc->SStartPoint.dArray[ucHorizontal] - pArc->SCenterPoint.dArray[ucHorizontal],
            pArc->SStartPoint.dArray[ucVertical] - pArc->SCenterPoint.dArray[ucVertical]);
        if (!isfinite(dActualRadius) || fabs(dActualRadius - dRadius) > HWSIM_DISTANCE_EPS * fmax(1.0, dRadius))
        {
            return -1;
        }
        pArc->dRad = 2.0 * PI;
    }
    else if (radCalculateR(pArc->SStartPoint.dArray, pArc->SEndPoint.dArray,
        pArc->dRadius, pArc->SCenterPoint.dArray, pArc->cArcDir, &pArc->dRad, ucHorizontal, ucVertical) != TRUE)
    {
        return -1;
    }
    return 1;
}
#endif


/*************************************************
  函数名称：SG2LoadSysPara(struct SG2* pSG2)
  函数功能  SG2初始化函数
  输入参数：pSG2		SG2指针
  输出参数：
  返 回 值：-1 传入空指针
			1  处理完成
  备    注：必须在通道初始化之后执行
**************************************************/
int8 SG2LoadSysPara(struct SG2* pSG2)
{
	int i;
	const uint8* ucMapArray;//映射
	if (pSG2 == NULL)
	{
		return -1;
	}

	//映射
	ucMapArray = pSG2->pSChannel->pMapAxisArray;

	//G1脉冲当量初始化
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX) continue;//空轴 跳过
		pSG2->dPulseToMm[i] = g_SSystemPara.SAxisPara[ucMapArray[i]].dPulseToMm;
	}
	//插补周期
	pSG2->SPlanData.dTc = g_SSystemPara.dInterpolationTime;
	//进给倍率
	pSG2->dFeedRatio = 1.0;
}





/*************************************************
  函数名称：calArcVectorAndDistance(struct SG02* pSG2Pointer)
  函数功能  计算长轴和距离分量  （ 圆弧 插补使用  ）  圆弧的距离分量 存储在横轴轴号对应的数组元素中
  输入参数：pSG2Pointer		G02的插补结构体
  输出参数：
  返 回 值：-1 空指针
			1  处理完成
  备    注：
**************************************************/
int8 calArcVectorAndDistance(struct SG2* pSG2Pointer)
{
	int i = 0;
	uint8 ucHorizontalAxis = pSG2Pointer->ucAxis[0];// 横轴
	uint8 ucVerticalAxis = pSG2Pointer->ucAxis[1]; //纵轴
	double dMaxDistance = 0;	//最大位移量
	double distance = 0;		//距离临时变量
	uint8 *pMapAxisArray = pSG2Pointer -> pSChannel ->pMapAxisArray; // 获取映射

	if (pSG2Pointer == NULL)//空指针
	{
		return -1;
	}

	//计算圆弧距离
	distance = fabs(pSG2Pointer->dRad * pSG2Pointer->dRadius);
	dMaxDistance = max(dMaxDistance, distance);

	//寻找最长轴  （绝对值）
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (VAL_VALID != getbit(pSG2Pointer->ucValStatus, i))  continue;// 无效轴，跳过
		if (i == ucHorizontalAxis || i == ucVerticalAxis)//跳过圆弧的横轴 或纵轴
		{
			continue;
		}
		//计算距离 fabs(终点-起点)
		distance = fabs(pSG2Pointer->SEndPoint.dArray[i] - pSG2Pointer->SStartPoint.dArray[i]);
		dMaxDistance = max(dMaxDistance, distance);
	}

	if(isZero(dMaxDistance))
	{
		parseRunningError(pSG2Pointer->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM132,9999);
		return -1;
	}

	//计算圆弧距离分量
	//圆弧的距离分量存储在横轴的距离分量
	pSG2Pointer->SStartVector.dArray[ucHorizontalAxis] = pSG2Pointer->dRad * pSG2Pointer->dRadius / dMaxDistance;
	//纵轴的距离分量为0
	pSG2Pointer->SStartVector.dArray[ucVerticalAxis] = 0;

	//计算距离分量
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (i == ucHorizontalAxis || i == ucVerticalAxis)//跳过圆弧的横轴 或纵轴
		{
			continue;
		}
		//其他直线计算
		distance = pSG2Pointer->SEndPoint.dArray[i] - pSG2Pointer->SStartPoint.dArray[i];
		//距离分量 = 当前轴位移 / 最长轴位移
		pSG2Pointer->SStartVector.dArray[i] = distance / dMaxDistance;
	}

	//最长轴用于速度规划
	pSG2Pointer->SPlanData.dDistance = pSG2Pointer->SPlanData.dTargetDist = dMaxDistance;
	return 1;
}


/*************************************************
  函数名称：calculateArcInterpolationPoint(double dDeltaL, struct SG2* pSG2Pointer, double* pdTargetPoint)
  函数功能  计算圆弧坐标点函数
  输入参数：dDeltaL			相对起点的距离
			pSG2Pointer		G02的插补结构体
  输出参数：pdTargetPoint	输出坐标点
  返 回 值：-1 空指针
			1  处理完成
  备    注：
**************************************************/
int8 calculateArcInterpolationPoint(double dDeltaL, struct SG2* pSG2Pointer, double* pdTargetPoint)
{
	int i = 0;
	//计算距离分量
	uint8 ucHorizontalAxis = pSG2Pointer->ucAxis[0];// 横轴
	uint8 ucVerticalAxis = pSG2Pointer->ucAxis[1]; //纵轴
	SPointDouble SStartPoint = pSG2Pointer->SStartPoint; //起点坐标
	SPointDouble SCenterPoint = pSG2Pointer->SCenterPoint; //圆心坐标
	SPointDouble SRelaCenterPoint;//圆心坐标（相对起点）
	SVector SStartVector = pSG2Pointer->SStartVector;    //距离分量
	int8 cArcDir = pSG2Pointer->cArcDir; //圆弧方向
	double dDeltaRad;//圆心角增量 （相对起点）

	if (isZero(pSG2Pointer->dRadius))
	{
		parseRunningError(pSG2Pointer->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return -1;
	}	

	//计算圆心相对坐标
	subtractPoint((double*)&SCenterPoint, (double*)&SStartPoint, (double*)&SRelaCenterPoint, AXIS_NUM_MAX, NULL);

	//先计算圆弧对应两个轴的坐标
	//计算圆心角增量
	dDeltaRad = (-1) * cArcDir * dDeltaL * pSG2Pointer->SStartVector.dArray[ucHorizontalAxis] / pSG2Pointer->dRadius;

	//计算绝对坐标  （=圆心坐标+相对坐标）
	//计算坐标（横轴
	pdTargetPoint[ucHorizontalAxis] = SCenterPoint.dArray[ucHorizontalAxis] - SRelaCenterPoint.dArray[ucHorizontalAxis] * cos(dDeltaRad) + SRelaCenterPoint.dArray[ucVerticalAxis] * sin(dDeltaRad);
	//计算坐标(纵轴
	pdTargetPoint[ucVerticalAxis] = SCenterPoint.dArray[ucVerticalAxis] - SRelaCenterPoint.dArray[ucVerticalAxis] * cos(dDeltaRad) - SRelaCenterPoint.dArray[ucHorizontalAxis] * sin(dDeltaRad);

	//再计算其他轴的坐标
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (i == ucHorizontalAxis || i == ucVerticalAxis)//横轴 纵轴 已计算  跳过
		{
			continue;
		}

		pdTargetPoint[i] = SStartPoint.dArray[i] + SStartVector.dArray[i] * dDeltaL;
	}

	return 1;
}


int8 commonArc2Ellipse(double* dPoint, double* dCenterPoint, double* dOutputPoint, int nArraySize, double dA, double dB, double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{//圆弧坐标转换成椭圆坐标
	double dTempPoint[2];
	double dTempCenterPoint[2];
	if (dPoint == NULL || dCenterPoint == NULL || dOutputPoint == NULL
		|| ucHorizontalAxis >= nArraySize || ucVerticalAxis >= nArraySize)
	{
		return -1;
	}
	dTempPoint[AXIS_HORIZONTAL] = dPoint[ucHorizontalAxis];
	dTempPoint[AXIS_VERTICAL] = dPoint[ucVerticalAxis];
	dTempCenterPoint[AXIS_HORIZONTAL] = dCenterPoint[ucHorizontalAxis];
	dTempCenterPoint[AXIS_VERTICAL] = dCenterPoint[ucVerticalAxis];
	//先进行坐标旋转
	//rotatingCoordinate(dTempPoint, 2, dTempPoint, dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);
	//然后计算相对圆心的坐标
	//dTempPoint[AXIS_HORIZONTAL] -= dTempCenterPoint[AXIS_HORIZONTAL];
	//dTempPoint[AXIS_VERTICAL] -= dTempCenterPoint[AXIS_VERTICAL];
	//压缩
	if (dA > dB)
	{  //压缩纵轴
		dTempPoint[AXIS_VERTICAL] = dTempPoint[AXIS_VERTICAL] * dB / dA;
	}
	else
	{  //压缩横轴
		dTempPoint[AXIS_HORIZONTAL] = dTempPoint[AXIS_HORIZONTAL] * dB / dA;
	}
	//加上圆心坐标
	dTempPoint[AXIS_HORIZONTAL] += dTempCenterPoint[AXIS_HORIZONTAL];
	dTempPoint[AXIS_VERTICAL] += dTempCenterPoint[AXIS_VERTICAL];
	//坐标旋转
	rotatingCoordinate(dTempPoint, 2, dTempPoint, -dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);
	dOutputPoint[ucHorizontalAxis] = dTempPoint[AXIS_HORIZONTAL];
	dOutputPoint[ucVerticalAxis] = dTempPoint[AXIS_VERTICAL];
	return 1;
}


//椭圆转化为普通圆
int8 ellipse2CommonArc(double* dPoint, double* dCenterPoint, double* dOutputPoint, int nArraySize, double dA, double dB, double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{
	double dTempPoint[2];
	double dTempCenterPoint[2];
	if (dPoint == NULL || dCenterPoint == NULL || dOutputPoint == NULL
		|| ucHorizontalAxis >= nArraySize || ucVerticalAxis >= nArraySize)
	{
		return -1;
	}
	dTempPoint[AXIS_HORIZONTAL] = dPoint[ucHorizontalAxis];
	dTempPoint[AXIS_VERTICAL] = dPoint[ucVerticalAxis];
	dTempCenterPoint[AXIS_HORIZONTAL] = dCenterPoint[ucHorizontalAxis];
	dTempCenterPoint[AXIS_VERTICAL] = dCenterPoint[ucVerticalAxis];
	//先进行坐标旋转
	rotatingCoordinate(dTempPoint, 2, dTempPoint, dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);
	//然后计算相对圆心的坐标
	dTempPoint[AXIS_HORIZONTAL] -= dTempCenterPoint[AXIS_HORIZONTAL];
	dTempPoint[AXIS_VERTICAL] -= dTempCenterPoint[AXIS_VERTICAL];
	//拉伸
	if (dA > dB)
	{  //拉伸纵轴
		dTempPoint[AXIS_VERTICAL] = dTempPoint[AXIS_VERTICAL] * dA / dB;
	}
	else
	{  //拉伸横轴
		dTempPoint[AXIS_HORIZONTAL] = dTempPoint[AXIS_HORIZONTAL] * dA / dB;
	}
	//加上圆心坐标
	//dTempPoint[AXIS_HORIZONTAL] += dTempCenterPoint[AXIS_HORIZONTAL];
	//dTempPoint[AXIS_VERTICAL] += dTempCenterPoint[AXIS_VERTICAL];
	//坐标旋转
	//rotatingCoordinate(dTempPoint, 2, dTempPoint, -dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);
	dOutputPoint[ucHorizontalAxis] = dTempPoint[AXIS_HORIZONTAL];
	dOutputPoint[ucVerticalAxis] = dTempPoint[AXIS_VERTICAL];
	return 1;
}

/*************************************************
  函数名称：prepareToPauseG2(struct SG2* pSG2Pointer, double dVmin, double dDec, bool bWriteBackFlag)
  函数功能  根据插补预处理结构体 以减速模型来初始化 原插补预处理结构体，确定距离和终点速度，方便进行速度规划
  输入参数：pSG2Pointer			SG02 结构体指针，当前插补的结构体
			dVmin				减速停的目标速度
			dDec				减速度
			bWriteBackFlag		bool类型	是否执行回写操作
  输出参数：
  返 回 值：-1 输入错误
			1 初始化完成
  备    注：
**************************************************/
int8 prepareToPauseG2(struct SG2* pSG2Pointer, double dVmin, double dDec, bool bWriteBackFlag)
{
	double dVs;   //起始速度 （最大速度  纯减速）
	double dLeftDist;//剩余距离
	double dSde;	//从最大速度减速到起跳速度所需距离
	double dVe;		//最终速度
	SPointDouble SEndPoint; //插补终点
	struct SToolCompensation* pSToolCompPointer = NULL; //刀补指针
	struct SInterpolationPre* pPreProcessPointer;  //预处理指针
	double dWriteBackDist = 0;//回写的已走距离
	bool bIsSlowingDown = FALSE; //是否正在减速
	bool bStoppable = FALSE;
	double dDecFactor; //减速公式系数
	int8 cRes;
	if (pSG2Pointer == NULL || dDec <= 0)   //空指针不运行
	{
		return -1;
	}

	SEndPoint = pSG2Pointer->SEndPoint; //终点信息
	dLeftDist = pSG2Pointer->SPlanData.dCurLeftDist;  //剩余距离
	pPreProcessPointer = &pSG2Pointer->pSChannel->SPreProcess;//预处理结构体
	dDecFactor = getFactor(pSG2Pointer->SPlanData.ucDecVelPlanType);//根据减速曲线类型获取系数

	if (pSG2Pointer->SPlanData.unCurrentT < pSG2Pointer->SPlanData.unT5 && pSG2Pointer->SPlanData.unCurrentT >pSG2Pointer->SPlanData.unT3)//当前在减速
	{
		bIsSlowingDown = TRUE;
		dSde = dLeftDist;
	}
	else //当前不在减速  计算减速距离 和终点位置
	{
		//起始速度
		dVs = pSG2Pointer->dCurrentSpeed;

		if (dVs <= dVmin)//当前速度小于最小速度 可以直接停
		{
			dSde = 0;//S4型
		}
		else//不可以直接停
		{
			//计算减速距离
			//dSde = fabs(3 * (dVs * dVs - dVmin * dVmin) / (4 * dDec));//S4型
			dSde = fabs(dDecFactor * (dVs * dVs - dVmin * dVmin) / dDec);//减速公式
		}

		if (pSG2Pointer->SPlanData.cInterpDir != INP_GO_BACK) //正向
		{
			//计算终点位置
			if (dSde < dLeftDist || isZero(dSde - dLeftDist))//距离足够减速
			{
				dVe = dVmin;//速度规划末速度 等于 系统起跳速度

				//计算终点位置 //2025/01/02洪冠鹏 pSG2Pointer->SPlanData.dDistance - dLeftDist + dSde -> pSG2Pointer->SPlanData.dTargetDist - dLeftDist + dSde
				cRes = calculateArcInterpolationPoint(pSG2Pointer->SPlanData.dTargetDist - dLeftDist + dSde, pSG2Pointer, (double*)&SEndPoint);
				if (cRes < 0) return cRes;
				dWriteBackDist = pSG2Pointer->SPlanData.dTargetDist - dLeftDist + dSde;
				pSG2Pointer->SPlanData.dCurLeftDist = dSde;
				pSG2Pointer->SPlanData.dTargetDist -= dLeftDist - dSde;
				pSG2Pointer->SEndPoint = SEndPoint;//终点改变
				bStoppable = TRUE;
			}
			else//当前段不能停下
			{
				//最终速度不能减到起跳速度  重算实际最终速度
				//dVe = sqrt(dVs * dVs - 4 * dDec * dLeftDist / 3);
				dVe = sqrt(dVs * dVs - dDec * dLeftDist / dDecFactor);  //最终速度不能减到起跳速度  重算实际最终速度
				dSde = dLeftDist;	//减速距离等于剩余距离
				SEndPoint = pSG2Pointer->SEndPoint; //终点等于原来的终点
			}
		}
		else
		{
			SPointDouble SNewStartPoint;
			dWriteBackDist = pSG2Pointer->SPlanData.dTargetDist - dLeftDist - dSde;

			if (isEqualOrGreaterThanZero(dWriteBackDist))//当前段能够停下
			{
				dVe = dVmin;
				//计算新的起点位置
				cRes = calculateArcInterpolationPoint(dWriteBackDist, pSG2Pointer, (double*)&SNewStartPoint);
				if (cRes < 0) return cRes;
				pSG2Pointer->SPlanData.dTargetDist -= dWriteBackDist;//目标距离
				pSG2Pointer->SPlanData.dPreDist -= dWriteBackDist;
				pSG2Pointer->SPlanData.dDistance -= dWriteBackDist;
				pSG2Pointer->SStartPoint = SNewStartPoint;
				bStoppable = TRUE;
			}
			else // 当前段不能停下
			{
				//dVe = sqrt(dVs * dVs - 4 * dDec * (dWriteBackDist + dSde) / 3);  //最终速度不能减到起跳速度  重算实际最终速度
				dVe = sqrt(dVs * dVs - dDec * (dWriteBackDist + dSde) / dDecFactor);  //最终速度不能减到起跳速度  重算实际最终速度
			}

		}

	}




	if (bWriteBackFlag == TRUE && (bIsSlowingDown == FALSE || pPreProcessPointer->sChannelCommand.ucType != UNASSIGNED_VALUE) && bStoppable == TRUE)//条件:通道指令或插补执行不完
	{
		//写回刀补缓冲区
		// moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_BACK, pSG2Pointer->pSChannel->ucChannelNum);  //移回前瞻指针 //注释不需要
		if(moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pSG2Pointer->pSChannel->ucChannelNum) == FALSE)  //移回规划指针
		{
			return -1;
		}
		pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSG2Pointer->pSChannel->ucChannelNum);//获取插补读指针

		////写回数据
		//回写的起点等于当前段的终点
		//pSToolCompPointer->SStartPoint = SEndPoint;
		////回写的终点等于G1的终点（已经过修正）
		//pSToolCompPointer->SEndPoint = pSG2Pointer->SEndPoint;
		////相对坐标标记置0
		//pSToolCompPointer->ucRelaVal = 0x0;

		pSToolCompPointer->dPreDist = dWriteBackDist;//写回距离
		pSToolCompPointer->dFeedrate = pSToolCompPointer->dUncorrectedFeedrate;//覆盖速度
		pSToolCompPointer->bHasVeerFlag = FALSE;
		pSToolCompPointer->bLookAheadVParaCalcOverFlag = FALSE;
		pSG2Pointer->pSChannel->bLookAheadNewBeginFlag = TRUE;
		initLookAheadDealed(pSG2Pointer->pSChannel->ucChannelNum);//初始化前瞻指针

		//if (bStoppable == TRUE)//指令将要执行完 将执行后作用IO  //朱稳中 2023.6.15
		if (pSG2Pointer->SPlanData.cInterpDir == INP_GO_AHEAD && isZero(pSG2Pointer->SPlanData.dDistance - pSG2Pointer->SPlanData.dTargetDist))
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

		//防止当前段的M30/M02，导致程序提前停止  (回写段有M30/M02，当前段的M30/M02需要删除)
		setBits(pPreProcessPointer->unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
	}
	if (bIsSlowingDown == FALSE)//当前段没有在减速 重新规划
	{
		//新的剩余距离= 减速所需的长度
		//pSG2Pointer->SPlanData.dCurLeftDist = dSde;
		//当前线段长度： 减去暂停后剩下的距离
		/*pSG2Pointer->SPlanData.dDistance -= dLeftDist - dSde;*/
		//目标距离
		//pSG2Pointer->SPlanData.dTargetDist -= dLeftDist - dSde;

		//暂停插补预处理结构体赋值
		pSG2Pointer->SPlanData.dVs = pSG2Pointer->SPlanData.dVmax = dVs;  //起始速度 （最大速度）
		pSG2Pointer->SPlanData.dVe = dVe;			//最终速度
		pSG2Pointer->SPlanData.dDec = dDec;			//减速度

		//曲线类型 1：梯形，2：3次S型，3：4次S型
		pSG2Pointer->SPlanData.ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		pSG2Pointer->SPlanData.ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		//平摊曲线类型，1：梯形，2：5次，3：比例系数
		pSG2Pointer->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;

		pSG2Pointer->SPlanData.unCurrentT = 0;  //时间
		preparePlanningAndDistReAvg(&pSG2Pointer->SPlanData, TRUE);//速度规划+平摊
	}

	return 1;   //预处理完成 返回1

}

/*************************************************
  函数名称：loadSG2DataFromInput(struct SG02* pSG2Pointer, struct SInput* pSInput)
  函数功能  从Input参数赋值到G2结构体
  输入参数：pSG2Pointer	结构体SG2指针
			pSInput		SInput结构体指针
			bReCalculate	根据当前起点重算圆心等信息
  输出参数：
  返 回 值：-1	指针为空
			1	处理完成
  备    注：
**************************************************/
int8 loadSG2DataFromInput(struct SG2* pSG2Pointer, struct SInput* pSInput)
{
	double dRad;
	SPointDouble SEndPoint;
	SPointDouble SStartPoint;
	int8 cRes;
	if (pSG2Pointer == NULL || pSInput == NULL) return -1; // 指针为空

	pSG2Pointer->ucValStatus = pSInput->ucValStatus;			//有效轴
	pSG2Pointer->SPlanData.cInterpDir = pSInput->cInterpDir;	//插补方向
	pSG2Pointer->SPlanData.dAcc = pSG2Pointer->dAcc = pSInput->dAcc;	//加速度
	pSG2Pointer->SPlanData.dCurLeftDist = pSInput->dCurLeftDist;//剩余距离
	pSG2Pointer->SPlanData.dDec = pSG2Pointer->dDec = pSInput->dDec;	//减速度
	pSG2Pointer->SPlanData.dPrimalVe = pSInput->dPrimalVe;		//末速度
	pSG2Pointer->SPlanData.dTargetDist = pSInput->dTargetDist;	//目标距离
	pSG2Pointer->SPlanData.dVs = pSInput->dVs;					//初速度
	pSG2Pointer->SPlanData.ucAccVelPlanType = pSInput->ucAccVelPlanType;	//加速类型
	pSG2Pointer->SPlanData.ucDecVelPlanType = pSInput->ucDecVelPlanType;	//减速类型
	pSG2Pointer->SPlanData.ucDistReAverageType = pSInput->ucDistReAverageType;	//平摊类型
	pSG2Pointer->SPlanData.ucTconstForTrapezoid = pSInput->ucTconstForTrapezoid;//三角形修调后匀速时间占总周期的百分比 （ % ） 洪冠鹏 2024/3/22

	pSG2Pointer->SPlanData.dDistance = pSInput->dDistance;		//总距离
	pSG2Pointer->SPlanData.dPreDist = pSInput->dPreDist;		//已走距离

	pSG2Pointer->dFeedrate = pSInput->dFeedrate;		//进给速度
	pSG2Pointer->dFeedRatio = pSInput->dFeedRatio;		//进给倍率
	//倍率为负或者0  使用SRun的倍率
	pSG2Pointer->bUseSRunFeedRatio = (pSG2Pointer->dFeedRatio < 0 || isZero(pSG2Pointer->dFeedRatio)) ? TRUE : FALSE;

	pSG2Pointer->SStartPoint = pSInput->SStartPoint;	//起点
	pSG2Pointer->SStartVector = pSInput->SStartVector;	//起点向量
	pSG2Pointer->SEndPoint = pSInput->SEndPoint;		//终点

	pSG2Pointer->cArcDir = pSInput->cArcDir;			//圆弧方向
	pSG2Pointer->dRadius = pSInput->dRadius;			//圆弧半径
	pSG2Pointer->SCenterPoint = pSInput->SCenterPoint;	//圆心
	pSG2Pointer->dA = pSInput->dEllipseFactorA;//椭圆参数A
	pSG2Pointer->dB = pSInput->dEllipseFactorB;//椭圆参数B
	pSG2Pointer->dCoordRotation = pSInput->dCoordRotation;// 坐标系旋转弧度  rad
	pSG2Pointer->fCylinderRadius = pSInput->fCylinderRadius;// 圆柱半径
	for (int i = 0; i < AXIS_NUM_MAX; i++)//计算最值 洪冠鹏 2024/4/12
	{
		if (i == pSG2Pointer->ucAxis[0])//插补横轴
		{
			pSG2Pointer->SMaxVal.dArray[i] = pSInput->dHorMax;
			pSG2Pointer->SMinVal.dArray[i] = pSInput->dHorMin;
		}
		else if (i == pSG2Pointer->ucAxis[1])//插补纵轴
		{
			pSG2Pointer->SMaxVal.dArray[i] = pSInput->dVerMax;
			pSG2Pointer->SMinVal.dArray[i] = pSInput->dVerMin;
		}
		else //其他轴的最值取终点
		{
			pSG2Pointer->SMaxVal.dArray[i] = pSInput->SEndPoint.dArray[i];
			pSG2Pointer->SMinVal.dArray[i] = pSInput->SEndPoint.dArray[i];
		}
	}
	switch (pSInput->cDealType)//根据处理方式计算
	{
	default:
	case DEALTYPE_RECAL: //重算 不回写(用户输入)
	case DEALTYPE_RECAL_WRITEBACK://重算 回写

		//判断相对坐标  修正终点
		if (pSInput->ucRelaVal != VAL_ABS_POINT)//不完全是绝对坐标
		{
			modifyRelaEndPoint((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->SEndPoint, pSG2Pointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSInput->ucRelaVal);
		}

		if (getBits(pSG2Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
		{
			pSG2Pointer->ucAxis[0] = g_SSystemPara.ucPolarInterpolationAxis[0];
			pSG2Pointer->ucAxis[1] = g_SSystemPara.ucPolarInterpolationAxis[1];
		}
		else if (getBits(pSG2Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_CYLINDRICAL)
		{//圆柱插补

		}
		else
		{
			//修正旋转轴坐标
			modifyRotationAxisPoint((double*)&pSInput->SStartPoint, (double*)&pSG2Pointer->SEndPoint, (double*)&pSG2Pointer->SStartVector, AXIS_NUM_MAX, pSG2Pointer->pSChannel->pMapAxisArray, &pSG2Pointer->ucValStatus, pSInput->ucRelaVal);

			//获取当前坐标
			getRealTimePoint((double*)&pSG2Pointer->SStartPoint, AXIS_NUM_MAX, pSG2Pointer->pSChannel->pMapAxisArray, &pSG2Pointer->ucValStatus);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
			if (pSG2Pointer->pSChannel->bUseVirtualY == TRUE)
			{
				//起点为倾斜坐标系（X1-Y1-Z1）物理轴的实时点，终点为直角坐标系(X-Y-Z)缓冲区的值，因此要将起点换算至直角坐标系。
			//if (VAL_VALID == (getbit(pSG2Pointer->ucValStatus, AXIS_Y)))//Y轴方向要运动
			//{
			//getSingleAxisRealTimePoint(&pSG2Pointer->SStartPoint.dArray[AXIS_X], AXIS_X, pSG2Pointer->pSChannel->pMapAxisArray);//当前X1轴的位置
			//getSingleAxisRealTimePoint(&pSG2Pointer->SStartPoint.dArray[AXIS_Y], AXIS_Y, pSG2Pointer->pSChannel->pMapAxisArray);//当前Y1轴的位置
				pSG2Pointer->SStartPoint.dArray[AXIS_X] -= 1.0 * pSG2Pointer->SStartPoint.dArray[AXIS_Y] * pSG2Pointer->pSChannel->dSINAngleYY1;//将倾斜坐标系的点转化为直角坐标系的点的
				pSG2Pointer->SStartPoint.dArray[AXIS_Y] = pSG2Pointer->SStartPoint.dArray[AXIS_Y] * pSG2Pointer->pSChannel->dCOSAngleYY1;
				//}
			}
#endif
		}

#if EllipseTEST  //朱稳中 2023.3.29
		if (getBits(pSG2Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ELLIPSE_ARC
			|| getBits(pSG2Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ELLIPSE_ANTI_ARC)
		{
			pSG2Pointer->cArcDir = (getBits(pSG2Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ELLIPSE_ARC) ? 1 : -1;
			pSG2Pointer->ucValStatus = (0x01 << pSG2Pointer->ucAxis[0]) | (0x01 << pSG2Pointer->ucAxis[1]);
			//重新计算圆心
			SStartPoint = pSG2Pointer->SStartPoint;
			SEndPoint = pSG2Pointer->SEndPoint;
			ellipseCenterCalculate((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->SEndPoint, AXIS_NUM_MAX, pSG2Pointer->dA, pSG2Pointer->dB, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->cArcDir, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
			rotatingCoordinate((double*)&pSG2Pointer->SCenterPoint, AXIS_NUM_MAX, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
			ellipse2CommonArc((double*)&SStartPoint, (double*)&pSG2Pointer->SCenterPoint, (double*)&SStartPoint, AXIS_NUM_MAX, pSG2Pointer->dA, pSG2Pointer->dB, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
			ellipse2CommonArc((double*)&SEndPoint, (double*)&pSG2Pointer->SCenterPoint, (double*)&SEndPoint, AXIS_NUM_MAX, pSG2Pointer->dA, pSG2Pointer->dB, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
			pSG2Pointer->SEllipseSCenterPoint = pSG2Pointer->SCenterPoint;
			pSG2Pointer->dRadius = max(pSG2Pointer->dA, pSG2Pointer->dB);
			radCalculateR((double*)&SStartPoint, (double*)&SEndPoint, pSG2Pointer->dRadius, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->cArcDir, &pSG2Pointer->dRad, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		}
		else {
			radCalculateR((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->SEndPoint, pSG2Pointer->dRadius, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->cArcDir, &pSG2Pointer->dRad, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
			SStartPoint = pSG2Pointer->SStartPoint;
			SEndPoint = pSG2Pointer->SEndPoint;
		}
#else
		//计算圆心和弧度
#if HANDWHEEL_SIM_DISTANCE_ENABLE
		if (!pSG2Pointer->bIsUserData && isHandwheelDistanceControlActive(pSG2Pointer->pSChannel))
		{
			cRes = calculateHandwheelArcGeometry(pSG2Pointer);
		}
		else
#endif
		cRes = radCalculateR((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->SEndPoint, pSG2Pointer->dRadius, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->cArcDir, &pSG2Pointer->dRad, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		if (cRes != 1)//圆弧计算失败
		{
			parseRunningError(pSG2Pointer->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
			return -1;
		}
#endif // EllipseTEST


		//根据弧度和半径计算距离
		//pSG2Pointer->SPlanData.dDistance = pSG2Pointer->dRad * pSG2Pointer->dRadius;

		//将绝对坐标回写入缓冲区  并把相对坐标标记置0
		if (pSInput->cDealType == DEALTYPE_RECAL_WRITEBACK)//回写
		{
			int8 cIndex = 0;
			struct SToolCompensation* pWriteBackBuff;

			cIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSG2Pointer->pSChannel->ucChannelNum);//获取索引
			pWriteBackBuff = getSToolCompPointerByIndex(cIndex - 1, pSG2Pointer->pSChannel->ucChannelNum);//根据索引的前一个进行回写
			pWriteBackBuff->SEndPoint = pSG2Pointer->SEndPoint;		//绝对坐标终点
			pWriteBackBuff->SStartPoint = pSG2Pointer->SStartPoint;	//绝对坐标 起点
			pWriteBackBuff->SCenterPoint = pSG2Pointer->SCenterPoint; //圆心
			pWriteBackBuff->dRadius = pSG2Pointer->dRadius; // 半径
			pWriteBackBuff->ucRelaVal = 0x0; // 相对坐标标记置为0
			pWriteBackBuff->dPreDist = 0; //将缓冲区的已走的距离置为0
		}
#if EllipseTEST
		pSG2Pointer->SStartPoint = SStartPoint;
		pSG2Pointer->SEndPoint = SEndPoint;
#endif // EllipseTEST

		if (getBits(pSG2Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
		{	//极坐标计算
			pSG2Pointer->ucValStatus |= 0x01 << g_SSystemPara.ucPolarInterpolationAxis[0];
			//subtractPoint((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->pSChannel->SRun.SCurrentOffset.dArray, (double*)&pSG2Pointer->SStartPoint, AXIS_NUM_MAX, &pSG2Pointer->ucValStatus);
			//subtractPoint((double*)&pSG2Pointer->SEndPoint, (double*)&pSG2Pointer->pSChannel->SRun.SCurrentOffset.dArray, (double*)&pSG2Pointer->SEndPoint, AXIS_NUM_MAX, &pSG2Pointer->ucValStatus);
			pSG2Pointer->ucValStatus |= 0x01 << g_SSystemPara.ucPolarInterpolationAxis[1];
		}
		//计算长轴和位移分量（圆弧）
		cRes = calArcVectorAndDistance(pSG2Pointer);
		if(cRes != 1){return cRes;}

		pSG2Pointer->SPlanData.dCurLeftDist = pSG2Pointer->SPlanData.dDistance;
		pSG2Pointer->SPlanData.dPreDist = 0;

		break;
	case DEALTYPE_NO_PROCESS://不进行处理

		if (!isZero(pSG2Pointer->SPlanData.dPreDist) && pSInput->cInterpDir == INP_GO_AHEAD)//已经走距离不为0
		{
			int8 cIndex = 0;
			struct SToolCompensation* pWriteBackBuff;

			cIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSG2Pointer->pSChannel->ucChannelNum);//获取索引
			pWriteBackBuff = getSToolCompPointerByIndex(cIndex - 1, pSG2Pointer->pSChannel->ucChannelNum);//根据索引的前一个进行回写
			pWriteBackBuff->dPreDist = 0; //将缓冲区的已走的距离置为0
		}

		//椭圆处理
#if EllipseTEST
		SStartPoint = pSG2Pointer->SStartPoint;
		SEndPoint = pSG2Pointer->SEndPoint;
		ellipseCenterCalculate((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->SEndPoint, AXIS_NUM_MAX, pSG2Pointer->dA, pSG2Pointer->dB, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->cArcDir, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		ellipse2CommonArc((double*)&SStartPoint, (double*)&pSG2Pointer->SCenterPoint, (double*)&SStartPoint, AXIS_NUM_MAX, pSG2Pointer->dA, pSG2Pointer->dB, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		ellipse2CommonArc((double*)&SEndPoint, (double*)&pSG2Pointer->SCenterPoint, (double*)&SEndPoint, AXIS_NUM_MAX, pSG2Pointer->dA, pSG2Pointer->dB, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		pSG2Pointer->SStartPoint = SStartPoint;
		pSG2Pointer->SEndPoint = SEndPoint;
		pSG2Pointer->dRadius = max(pSG2Pointer->dA, pSG2Pointer->dB);
#endif // EllipseTEST

		//计算圆心和弧度
#if HANDWHEEL_SIM_DISTANCE_ENABLE
		if (!pSG2Pointer->bIsUserData && isHandwheelDistanceControlActive(pSG2Pointer->pSChannel))
		{
			cRes = calculateHandwheelArcGeometry(pSG2Pointer);
		}
		else
#endif
		cRes = radCalculateR((double*)&pSG2Pointer->SStartPoint, (double*)&pSG2Pointer->SEndPoint, pSG2Pointer->dRadius, (double*)&pSG2Pointer->SCenterPoint, pSG2Pointer->cArcDir, &pSG2Pointer->dRad, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		if (cRes != 1)//圆弧计算失败
		{
			parseRunningError(pSG2Pointer->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
			return -1;
		}
		//计算长轴和位移分量（圆弧）
		cRes = calArcVectorAndDistance(pSG2Pointer);
		if(cRes != 1){return cRes;}

		pSG2Pointer->SPlanData.dCurLeftDist = (!isZero(pSG2Pointer->SPlanData.dPreDist) || !(pSInput->cInterpDir == INP_GO_BACK)) ? //2022.12.8  cInterpDir == INP_GO_AHEAD --> !(cInterpDir == INP_GO_BACK)
			pSG2Pointer->SPlanData.dDistance - pSG2Pointer->SPlanData.dPreDist : 0;//计算剩余距离  正走：剩余距离= 总距离-已走距离  反走： 剩余距离为零
		break;
	}

	if (isZero(pSG2Pointer->SPlanData.dTargetDist))// 对0距离判断  2023.3.24 朱稳中
	{
		return 0;
	}

	return 1;
}


/*************************************************
  函数名称：calculateArcLimitDistance
  函数功能：计算沿原圆弧首次到达指定轴限位的合成规划距离
  输入参数：pArc - 已加载几何及距离分量的圆弧，非空
            ucAxis - 限位逻辑轴号；dLimit - 与起点同坐标系的限位值
  输出参数：无
  返 回 值：非负值 - 从原起点累计的规划距离；-1 - 几何或比例无效
  备    注：只读圆弧；平面弧长按原比例换算，起点已在边界时保守返回零
****************************************/

/*************************************************
  函数名称：motionArc(struct SG02* pSG2Pointer, bool bPauseOn, bool bRatioOn)
  函数功能  根据预处理结构体内的数据，根据进给速度和倍率执行速度规划和平摊，计算时间节点currentT的绝对坐标以及脉冲增量值
  输入参数：pSG2Pointer	结构体SG2指针
			bPauseOn	开启暂停
			bRatioOn	开启倍率
  输出参数：
  返 回 值：-1	空指针
			0	插补完成
			1	插补中
  备    注：换向锁仅冻结手轮倍率响应；停稳后采纳最新倍率，暂停和限位仍走原流程。
**************************************************/
int8 motionArc(struct SG2* pSG2Pointer, bool bPauseOn, bool bRatioOn)
{
	int8 res;// 调用函数返回
	double dDeltaL; //相对起点的距离 (或弧长)
	SPointDouble STargetPoint;   //当前点的理论绝对坐标
	struct SInterpolationPre* pSPreprocess;//预处理指针
	const uint8* ucMapArray;//映射
	struct _SRun* pSRun;//运行结构体
	struct SInput* pSUserData;//数据来源
	//局部变量保存参数 防止规划失败
	double dVmax, dVs, dVe, dAcc, dDec;
	double dTargetSpeed;
	double dSpeedPercentage;
	double dCurrentSpeed;
	VAL_STRUCT_TYPE ucValStatus; // 有效轴
	
	//软限位参数 2024/4/11 洪冠鹏
	double dRad1 = 0;
    double dRad2 = 0;
	double dNewEndPoint = 0;
	double dSingleAxisLeftDist;//剩余未走单轴运动距离
	double dResultantLeftDist;//剩余未走单轴运动距离
	double dMinTDec;		  //最小的T型减速度（系统参数）

	double dVmaxToLimitiedPoint;//减速到软限位点反算的最大速度
	if (pSG2Pointer == NULL) //空指针  不运行
	{
		return -1;
	}

	if (pSG2Pointer->ucInpStatus == INP_FINISHED)//如果插补结构体为完成状态，返回
	{
		return 0;
	}

	pSPreprocess = &pSG2Pointer->pSChannel->SPreProcess;//预处理指针
	ucMapArray = pSG2Pointer->pSChannel->pMapAxisArray;//映射
	pSRun = &pSG2Pointer->pSChannel->SRun;//运行结构体
	pSUserData = pSG2Pointer->bIsUserData ? &pSG2Pointer->SUserData : &pSPreprocess->SUserData;//数据来源

#if HANDWHEEL_SIM_DISTANCE_ENABLE
	/* [HWSIM] G2 与 G3 共用 motionArc() 和本入口，没有另一个 G3 距离执行器。
	 * 首次先占轴、选择平面、装载圆弧几何，到 INP_ASSIGNMENT_START 再绑定；
	 * 后续周期从下面直接进入 executeHandwheelPathStep(TRUE)，不重复计算圆心和扫角。
	 * 编码器采样已经在外层 interruptFuncAPI() 完成。
	 */
	if (!pSG2Pointer->bIsUserData && prepareHandwheelAutomaticMove(pSG2Pointer->pSChannel) < 0)
	{
		return -1;
	}
	/* [HWSIM] 自动旁路继续旧算法；当前距离路径已绑定时仍进入统一入口做检查，
	 * 防止模式变化使同一条圆弧中途由另一套时间规划器接管。
	 * 用户数据运动和初始占轴/加载阶段不在此处提前返回。
	 */
	if (!pSG2Pointer->bIsUserData && (isHandwheelDistanceControlActive(pSG2Pointer->pSChannel)
		|| pSG2Pointer->pSChannel->SHandwheelSim.SPath.bBound)
		&& pSG2Pointer->ucInpStatus != INP_OCCUPATION && pSG2Pointer->ucInpStatus != INP_INITIATION)
	{
		/* [HWSIM] TRUE 选择 SG02 圆弧视图。每拍先预览一份合成路径进度，
		 * 再由 calculateHandwheelPathPoint() 调 calculateArcInterpolationPoint() 求各轴点，
		 * 全轴输出成功后才提交。反摇减小进度，不修改 cArcDir 或把 G2 改成 G3。
		 */
		return executeHandwheelPathStep(pSG2Pointer->pSChannel, TRUE);
	}
#endif

	if (bRatioOn && pSG2Pointer->bUseSRunFeedRatio == TRUE)//倍率开启
	{
		//插补正在进行的状态 且当前倍率不等于目标倍率
		if (pSG2Pointer->ucInpStatus == INP_RUNNING && pSG2Pointer->dFeedRatio != pSRun->dCommonFeedRatio * pSRun->dHandWheelSpeedRatio)
		{
			if (pSG2Pointer->SPlanData.unCurrentT < pSG2Pointer->SPlanData.unT3 || pSG2Pointer->SPlanData.unCurrentT > pSG2Pointer->SPlanData.unT5)	//陈敏聪	2023/7/11
			{
				pSG2Pointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值
			}
		}
	}

	if (getBits(pSG2Pointer->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_R && pSG2Pointer->ucInpStatus == INP_RUNNING)//G99 每转进给
	{
		dTargetSpeed = fabs(pSG2Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed) * pSG2Pointer->dFeedrate * pSG2Pointer->dFeedRatio;
		dCurrentSpeed = pSRun->dCurrentFeedRate;

		if (isZero(dTargetSpeed))// 目标转速为0   （分母为0)
		{
			if (isZero(dCurrentSpeed) || isZero(g_SSystemPara.fG99SpindleSpeedDeviation - 1.0))//编码器相对值为0  或者允许误差为100%
			{
				dSpeedPercentage = 1.0;//认为速度达到目标值的100%
			}
		}
		else
		{
			dSpeedPercentage = dCurrentSpeed / dTargetSpeed;
		}

		if ((fabs(dSpeedPercentage - 1.0 * SGN(dSpeedPercentage)) > g_SSystemPara.fG99SpindleSpeedDeviation))//计算得到的进给速度发生改变
		{
			pSG2Pointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值
		}

		////计算得到的进给速度发生改变
		//if (!isZero(pSRun->dCurrentFeedRate - fabs(pSG2Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed) * pSG2Pointer->dFeedrate * 60 * 1000 * pSG2Pointer->dFeedRatio))
		//{
		//	pSG2Pointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值
		//}

	}



	switch (pSG2Pointer->ucInpStatus)
	{
	case INP_FINISHED://已经完成
		break;

	case INP_OCCUPATION:
		//释放旧的有效轴
		releaseLastCommandAxis(pSPreprocess);
		ucValStatus = pSG2Pointer->bIsUserData ? pSG2Pointer->SUserData.ucValStatus : pSPreprocess->SUserData.ucValStatus;
#if(TURRET_AXIS_TEST == 1)
		//判断轴占用(根据结构体轴+映射)   目标结构体轴减去刀塔轴
		if (axisManagerOccupyByChannel(ucValStatus & (~pSPreprocess->ucOccupideTurretAxis), pSG2Pointer->pSChannel->ucChannelNum, &pSPreprocess->ucOccupideValStatus) == 1)
		{	//占用成功
			//G2有效轴（不能除去刀补）
			pSG2Pointer->ucValStatus = ucValStatus;//在G2插补结构体中记录占用成功的结构体轴
			pSG2Pointer->ucInpStatus = INP_INITIATION;
		}
		else
		{	//占用失败
			break;
		}

#else
		//判断轴占用(根据有效轴+映射)   有效轴减去刀补轴
		if (axisManagerOccupyByChannel(ucValStatus & (~pSPreprocess->ucToolAxis), pSG2Pointer->pSChannel->ucChannelNum, NULL) == 1)
		{	//占用成功
			//上一条指令的占用（除去刀补）
			pSPreprocess->ucLastValStatus = ucValStatus & (~pSPreprocess->ucToolAxis);
			//G2有效轴（不能除去刀补）
			pSG2Pointer->ucValStatus = ucValStatus;
			pSG2Pointer->ucInpStatus = INP_INITIATION;
		}
		else
		{	//占用失败
			break;
		}
#endif

	case INP_INITIATION://初始化
		//选择插补平面
		chooseInterpPlane(pSG2Pointer->ucAxis, pSG2Pointer->unCommandType);

		if (pSG2Pointer->bIsUserData)//根据标志位读取数据 2022.12.6
		{	//读取输入参数（用户参数）
			res = loadSG2DataFromInput(pSG2Pointer, &pSG2Pointer->SUserData);
		}
		else
		{
			//读取预处理参数
			res = loadSG2DataFromInput(pSG2Pointer, &pSPreprocess->SUserData);
		}
		if(res == -1)return -1;	

		if (isZero(pSG2Pointer->SPlanData.dAcc) || isZero(pSG2Pointer->SPlanData.dDec))//参数检测，避免一些参数出现为0的情况
		{
			parseRunningError(pSG2Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
			return -1;	
		}
		if (res == 0) //距离为0
		{
			pSG2Pointer->ucInpStatus = INP_FINISHED;
			pSPreprocess->ucInpStatus = INP_FINISHED;
			break;
		}
		pSG2Pointer->dCurrentSpeed = pSRun->dCurrentSpeed;
		//计算长轴和分量
		pSG2Pointer->ucInpStatus = INP_ASSIGNMENT_START;

	case INP_ASSIGNMENT_START://赋值
#if HANDWHEEL_SIM_DISTANCE_ENABLE
		/* [HWSIM] 首次圆弧距离入口：原几何已经有效，此时创建/读取路径；
		 * 下一次/后续显示延迟只算一次，
		 * 恢复加工后直接返回，执行路径自动运行时曲线规划。
		 */
		if (!pSG2Pointer->bIsUserData && isHandwheelDistanceControlActive(pSG2Pointer->pSChannel))
		{
			return executeHandwheelPathStep(pSG2Pointer->pSChannel, TRUE);
		}
#endif
		//局部变量保存参数 防止规划失败
		dVmax = pSG2Pointer->SPlanData.dVmax;
		dVs = pSG2Pointer->SPlanData.dVs;
		dVe = pSG2Pointer->SPlanData.dVe;
		dAcc = pSG2Pointer->SPlanData.dAcc;
		dDec = pSG2Pointer->SPlanData.dDec;
		pSG2Pointer->SPlanData.dAcc = pSG2Pointer->dAcc;
		pSG2Pointer->SPlanData.dDec = pSG2Pointer->dDec;


		if (bRatioOn)
		{
			double dSpeedMax = (getBits(pSG2Pointer->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN) ?  //指令类型为每分钟进给？
				pSG2Pointer->dFeedrate //每分钟进给 最大速度为进给速度
				: fabs(pSG2Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed) * pSG2Pointer->dFeedrate;//每转进给  最大速度根据主轴转速和进给速度(mm/r)计算 2023.3.14

			//继承进给速率
			pSG2Pointer->dFeedRatio = (pSG2Pointer->bUseSRunFeedRatio) ? pSRun->dCommonFeedRatio * pSRun->dHandWheelSpeedRatio : pSG2Pointer->dFeedRatio;

			if (pSG2Pointer->SPlanData.cInterpDir * pSG2Pointer->dFeedRatio > 0) //插补方向和倍率同向
			{
				//根据倍率和主轴转速计算最大速度（当前进给速度保存到g_SRun）
				pSG2Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate
					= dSpeedMax * fabs(pSG2Pointer->dFeedRatio);
			}
			else  //<=0    插补方向和倍率反向
			{
				//可能的情况 ①倍率为0 ②当前速度为0 ③ 都不为0
				if (isZero(pSG2Pointer->dCurrentSpeed))//当前速度为0
				{
					pSG2Pointer->SPlanData.cInterpDir = (pSG2Pointer->dFeedRatio < 0) ? INP_GO_BACK : INP_GO_AHEAD;//赋值方向
					pSG2Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate  //赋值最大速度
						= dSpeedMax * fabs(pSG2Pointer->dFeedRatio);
				}
				else//当前速度不为0  先减速到0
				{
					pSG2Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate = 0;
				}
			}
		}
		else
		{
			//最大速度赋值，不开启倍率时赋进给速度
			pSG2Pointer->SPlanData.dVmax = pSG2Pointer->dFeedrate;

		}
		if (pSG2Pointer->pSChannel->cHandWheelSimulaStatus != HANWHEELSIMULA_OFF && (g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE)
#if HANDWHEEL_SIM_DISTANCE_ENABLE
			&& !isHandwheelAutomaticMoveRequired(pSG2Pointer->pSChannel)
#endif
			)
		{//手轮模拟 最大速度限制
		
			pSG2Pointer->SPlanData.dVmax = min(pSG2Pointer->SPlanData.dVmax, g_SSystemPara.dSimulaVmax* fabs(pSG2Pointer->dFeedRatio));//2025.5.5熊骏
			if (pSG2Pointer->pSChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_ON)	//陈敏聪	2023/12/14
			{
				pSG2Pointer->SPlanData.dAcc = min(pSG2Pointer->SPlanData.dAcc, g_SSystemPara.dSimulaMaxAcc);
				pSG2Pointer->SPlanData.dDec = min(pSG2Pointer->SPlanData.dDec, g_SSystemPara.dSimulaMaxDec);//2025.4.19熊骏
			}
		}

		if (g_SSystemPara.ucInterpolationMode != INTERPOLATION_LONGAXIS_MODE && !pSG2Pointer->bIsUserData)
		{	//使用缓冲区数据 而且为 自适应插补模式 或 固定参数插补模式
			pSG2Pointer->SPlanData.dVmax = min(pSG2Pointer->SPlanData.dVmax, pSPreprocess->SUserData.dArcMaxNormalAccVmax);//法向加速度约束
			pSG2Pointer->SPlanData.dVmax = min(pSG2Pointer->SPlanData.dVmax, pSPreprocess->SUserData.dArcMaxChordErrorVmax);//弓高误差约束

			//防止超过最大速度
			if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE)
			{ // 自适应参数插补
				pSG2Pointer->SPlanData.dVmax = min(pSG2Pointer->SPlanData.dVmax, pSPreprocess->SUserData.dCommonVMaxFromSingleAxis);
			}
		}
		else
		{ //使用用户数据 或者为 长轴插补模式
			pSG2Pointer->SPlanData.dVmax = min(pSG2Pointer->SPlanData.dVmax, g_SSystemPara.dSysMaxVel);//防止最大速度超限
		}


		
		//计算初速度 分两种情况 ：G02刚开始(当前速度为0)  执行过程中进给速度改变(当前速度不为0)
		//①刚开始：初速度和最大速度取最小值。初速度比最大速度大的情况下，直接以最大速度运动
		//②非刚开始：一律继承当前速度
		pSG2Pointer->SPlanData.dVs = (pSG2Pointer->bIsUserData) ? pSG2Pointer->SUserData.dVs : pSPreprocess->SUserData.dMinVs; //获取初速度
		pSG2Pointer->SPlanData.dVs = (isZero(pSG2Pointer->dCurrentSpeed)) ? min(pSG2Pointer->SPlanData.dVs, pSG2Pointer->SPlanData.dVmax) : pSG2Pointer->dCurrentSpeed;
		
		if (pSRun->cSoftLimitedOn == 3) //当前线段之前已触发软限位,重设速度规划参数后进入（减速）速度规划
		{
			for (int i = 0; i < AXIS_NUM_MAX; i++)//循环各轴找出（优先）触发软限位的轴，重新设定运动参数
			{
				if (VAL_VALID != getbit(pSPreprocess->SUserData.ucValStatus, i)) continue;  //无效轴 跳过
				if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)//触发正限位
				{
					dSingleAxisLeftDist = g_SSystemPara.MOTOR_WARN[i].pos_soft - pSG2Pointer->SMaxVal.dArray[i];//1.计算单轴限位点到最值点的距离
				}
				else if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)//触发负限位
				{
					dSingleAxisLeftDist = pSG2Pointer->SMinVal.dArray[i] - g_SSystemPara.MOTOR_WARN[i].neg_soft;//1.计算单轴限位点到最值点的距离
				}
				else//未触发限位的轴：跳过
				{
					continue;
				}
				//重设曲线类型（与前段轨迹保持相同的T型曲线）
				pSG2Pointer->SPlanData.dVmax = pSG2Pointer->SPlanData.dVs;//最大速度 = 初速度 = 当前速度
				pSG2Pointer->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;	//加速类型
				pSG2Pointer->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;	//减速类型
				pSG2Pointer->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;//平摊类型
				pSG2Pointer->SPlanData.dAcc = min(getSysAcc(pSG2Pointer->SPlanData.ucAccVelPlanType), g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxAcc / fabs(pSG2Pointer->SStartVector.dArray[i]));//加速度
				pSG2Pointer->SPlanData.dDec = min(getSysDec(pSG2Pointer->SPlanData.ucDecVelPlanType), g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec / fabs(pSG2Pointer->SStartVector.dArray[i]));//减速度

				if (dSingleAxisLeftDist >= EPSINON)//限位点在当前线段外：1.重算末速度-> 执行T型减速的速度规划
				{
					//取 系统T型合成减速度 与 系统T型单轴减速度 的最小值作为合成运动的T型减速度
					dMinTDec = min(g_SSystemPara.dSysPlanTMaxDec, g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec);
					//1.重算末速度：以T型减速度重算当前线段的末速度（若当前线段是最后一段，则直接设定为系统参数的最小速度进行减速停止）
					pSG2Pointer->SPlanData.dVe = isZero(pSG2Pointer->SPlanData.dPrimalVe - g_SSystemPara.dSysMinVel) ? g_SSystemPara.dSysMinVel : sqrt(pSG2Pointer->SPlanData.dVs * pSG2Pointer->SPlanData.dVs - 2 * g_SSystemPara.dSysPlanTMaxDec * pSG2Pointer->SPlanData.dCurLeftDist);
				}
				else//限位点在当前线段内：1.重算末速度；；2.重算减速度；3.重置开关变量。 -> 执行T型减速的速度规划实时判断减速停
				{
					pSG2Pointer->SPlanData.dVe = g_SSystemPara.dSysMinVel;//1.重算末速度 = 终止速度
					pSG2Pointer->SPlanData.dDec = (pow(pSG2Pointer->SPlanData.dVs, 2) - pow(pSG2Pointer->SPlanData.dVe, 2)) / (2.0 * pSG2Pointer->SPlanData.dCurLeftDist);//2.重算减速度(T型)
					pSRun->cSoftLimitedOn = 1;//3.重置软限位开关变量
				}
			}
		}
		else
		{
			//换回去前瞻的末速度
			pSG2Pointer->SPlanData.dVe = pSG2Pointer->SPlanData.dPrimalVe;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
			if (!pSG2Pointer->bIsUserData && isHandwheelAutomaticMoveRequired(pSG2Pointer->pSChannel))
			{
				pSG2Pointer->SPlanData.dVe = pSG2Pointer->SPlanData.dPrimalVe = 0.0;
			}
#endif
		}

		if (preparePlanningAndDistReAvg(&pSG2Pointer->SPlanData, TRUE) == -2)  //速度规划+平摊
		{//规划失败  换回原来的参数
			pSG2Pointer->SPlanData.dVmax = dVmax;
			pSG2Pointer->SPlanData.dVs = dVs;
			pSG2Pointer->SPlanData.dVe = dVe;
			pSG2Pointer->SPlanData.dAcc = dAcc;
			pSG2Pointer->SPlanData.dDec = dDec;
		}
		else
		{//规划成功  时间置0
			pSG2Pointer->SPlanData.unCurrentT = 0;
		}
		//软限位的判断
		pSRun->cSoftLimitedOn = softLimit(pSG2Pointer->pSChannel, pSG2Pointer->SStartPoint, pSG2Pointer->SEndPoint, pSG2Pointer->SMaxVal, pSG2Pointer->SMinVal, AXIS_NULL);
		//判断后的处理
		switch (pSRun->cSoftLimitedOn)
		{
		case 1://触发软限位：限位点在当前线段内,当前段能停下 -> 1.重设目标距离;2.重设末速度;3.设置插补运行状态。
			for (int i = 0; i < AXIS_NUM_MAX; i++)//循环各轴找出（优先）触发软限位的轴，重新设定运动参数
			{
				if (VAL_VALID != getbit(pSPreprocess->SUserData.ucValStatus, i)) continue;  //无效轴 跳过
				if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)//触发正限位
				{
					dNewEndPoint = g_SSystemPara.MOTOR_WARN[i].pos_soft;//新终点设为限位点
				}
				else if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)//触发负限位
				{
					dNewEndPoint = g_SSystemPara.MOTOR_WARN[i].neg_soft;//新终点设为限位点
				}
				else//未触发限位的轴：跳过
				{
					continue;
				}
				/* 选界和截停共用有向距离，保留长弧象限及其他联动轴的距离比例。 */
                if (!isZero(dNewEndPoint -
                            pSG2Pointer->SEndPoint
                                .dArray[i])) //触发软限位后终点发生改变，目标距离（弧长）也因终点的改变发生变化
				{
                    if (i == pSG2Pointer->ucAxis[0]) //触发限位的轴为插补平面的横轴
                    {
                        dRad1 = acos(fabs((pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i]) /
                                          pSG2Pointer->dRadius)); //起点相对于横轴的夹角1，单位：rad
                        dRad2 = acos(fabs((dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) /
                                          pSG2Pointer->dRadius)); //新终点相对于横轴的夹角2，单位：rad

                        //起点与新终点在圆心两侧，则运动的弧度为PI-夹角1-夹角2；起点与新终点在圆心同侧，则运动的弧度为|夹角1-夹角2|
                        if ((dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) *
                                    (pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i]) >
                                0 ||
                            isZero(dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) ||
                            isZero(pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i]))
				{
                            pSG2Pointer->SPlanData.dTargetDist = fabs((dRad1 - dRad2) * pSG2Pointer->dRadius);
				pSG2Pointer->SPlanData.dCurLeftDist =
                                fabs(pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dPreDist);
				}
                        else
                        {
                            pSG2Pointer->SPlanData.dTargetDist = fabs((PI - dRad1 - dRad2) * pSG2Pointer->dRadius);
                            pSG2Pointer->SPlanData.dCurLeftDist =
                                fabs(pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dPreDist);
                        }
                        //pSG2Pointer->SPlanData.dTargetDist = pSG2Pointer->SPlanData.dCurLeftDist = (sgn(dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) != sgn(pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i])) ?
                        //	fabs((PI - dRad1 - dRad2) * pSG2Pointer->dRadius) : fabs((dRad1 - dRad2) * pSG2Pointer->dRadius);
                    }
                    else if (i == pSG2Pointer->ucAxis[1]) //触发限位的轴为插补平面的纵轴
                    {
                        dRad1 = asin(fabs((pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i]) /
                                          pSG2Pointer->dRadius)); //起点相对于横轴的夹角1，单位：rad
                        dRad2 = asin(fabs((dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) /
                                          pSG2Pointer->dRadius)); //新终点相对于横轴的夹角2，单位：rad

                        //起点与新终点在圆心两侧，则运动的弧度为PI-夹角1-夹角2；起点与新终点在圆心同侧，则运动的弧度为|夹角1-夹角2|
                        // 修改2025.9.29 ，增加判断两浮点数相减是否相等，若不判断则值可能有正有负，导致运动夹角计算错误
                        if ((dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) *
                                    (pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i]) >
                                0 ||
                            isZero(dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) ||
                            isZero(pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i]))
                        {
                            pSG2Pointer->SPlanData.dTargetDist = fabs((dRad1 - dRad2) * pSG2Pointer->dRadius);
                            pSG2Pointer->SPlanData.dCurLeftDist =
                                fabs(pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dPreDist);
                        }
                        else
                        {
                            pSG2Pointer->SPlanData.dTargetDist = fabs((PI - dRad1 - dRad2) * pSG2Pointer->dRadius);
                            pSG2Pointer->SPlanData.dCurLeftDist =
                                fabs(pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dPreDist);
                        }
                        //pSG2Pointer->SPlanData.dTargetDist = pSG2Pointer->SPlanData.dCurLeftDist = (sgn(dNewEndPoint - pSG2Pointer->SCenterPoint.dArray[i]) != sgn(pSG2Pointer->SStartPoint.dArray[i] - pSG2Pointer->SCenterPoint.dArray[i])) ?
                        //	fabs((PI - dRad1 - dRad2) * pSG2Pointer->dRadius) : fabs((dRad1 - dRad2) * pSG2Pointer->dRadius);//弧长因终点的变化发生改变，原轨迹不变
                    }
                    else //对于非平面圆弧触发的软限位，则根据位移分量重算目标距离。
                    {
                        pSG2Pointer->SPlanData.dTargetDist = fabs((dNewEndPoint - pSG2Pointer->SStartPoint.dArray[i]) /
                                                                  pSG2Pointer->SStartVector.dArray[i]);
                        pSG2Pointer->SPlanData.dCurLeftDist =
                            fabs(pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dPreDist);
                    }
                }
				//2.重算末速度 = 终止速度
				pSG2Pointer->dReCalculateVe = g_SSystemPara.dSysMinVel;
				//3.插补状态：运行
				pSG2Pointer->ucInpStatus = INP_RUNNING;
				break;
			}
			break;
		case 2://触发软限位：限位点在当前线段外，当前线段不停 -> 1.重算末速度;2.设置插补运行状态。
			for (int i = 0; i < AXIS_NUM_MAX; i++)//循环各轴找出（优先）触发软限位的轴，重新设定运动参数
			{
				if (VAL_VALID != getbit(pSPreprocess->SUserData.ucValStatus, i)) continue;  //无效轴 跳过
				if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)//触发正限位
				{
					dSingleAxisLeftDist = g_SSystemPara.MOTOR_WARN[i].pos_soft - pSG2Pointer->SEndPoint.dArray[i];//1.计算单轴限位点到终点的剩余未走的单轴运动距离
				}
				else if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)//触发负限位
				{
					dSingleAxisLeftDist = pSG2Pointer->SEndPoint.dArray[i] - g_SSystemPara.MOTOR_WARN[i].neg_soft;//1.计算单轴限位点到终点的剩余未走的单轴运动距离
				}
				else//未触发限位的轴：跳过
				{
					continue;
				}
				if (i == pSG2Pointer->ucAxis[0])//触发限位的轴为插补平面的横轴
				{
					dResultantLeftDist = dSingleAxisLeftDist / cos(pSG2Pointer->dCurEndPointAngle);
				}
				else if (i == pSG2Pointer->ucAxis[1])//触发限位的轴为插补平面的纵轴
				{
					dResultantLeftDist = dSingleAxisLeftDist / sin(pSG2Pointer->dCurEndPointAngle);
				}
				else
				{
					dResultantLeftDist = dSingleAxisLeftDist / fabs(pSG2Pointer->SStartVector.dArray[i]);
				}
				//取 系统T型合成减速度 与 系统T型单轴减速度反算的合成减速度 的最小值作为合成运动的T型减速度(防止用户输入过大的合成减速度参数)
				dMinTDec = min(g_SSystemPara.dSysPlanTMaxDec, g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec / fabs(pSG2Pointer->SStartVector.dArray[i]));
				//1.重算末速度:以T型减速度重算当前线段的末速度
				pSG2Pointer->dReCalculateVe = sqrt(pow(g_SSystemPara.dSysMinVel, 2) + 2.0 * dMinTDec * fabs(dResultantLeftDist));
				//2.插补状态：运行
				pSG2Pointer->ucInpStatus = INP_RUNNING;
				break;
			}
			break;
		default://未触发软限位
			pSG2Pointer->ucInpStatus = INP_RUNNING;//插补状态：运行
			break;
		}

	case INP_RUNNING://执行运动

		if (bPauseOn) //开启暂停处理
		{
			//系统TO_PAUSE状态 插补结构体的状态不处于PAUSING
			if (pSG2Pointer->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE && pSG2Pointer->ucInpStatus != INP_PAUSING)
			{
				res = prepareToPauseG2(pSG2Pointer, pSUserData->dMinVe, pSUserData->dDec, TRUE);//插补结构体初始化 为减速模型
				if (res < 0)
				{
					parseRunningError(pSG2Pointer->pSChannel->SPreProcess.unGCodeLineNum,GCODE_ERROR_NUM300,9999);
					return res;
				}
				pSG2Pointer->ucInpStatus = INP_PAUSING;  //改变状态为正在暂停
			}
		}

	case INP_PAUSING://暂停
		//运行时间增加
		pSG2Pointer->SPlanData.unCurrentT += 1;

		//获取到长轴走的距离
		dDeltaL = nextstepPlanningAndDistReAvg(pSG2Pointer->SPlanData.unCurrentT, &pSG2Pointer->SPlanData);

		//计算理论坐标点
		res =calculateArcInterpolationPoint(dDeltaL, pSG2Pointer, (double*)&STargetPoint);
		if (res < 0) return res;

		//极坐标处理
		if (getBits(pSG2Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
		{
			double dCPoint;
			int8 cDir = 1;
			if (getBits(pSG2Pointer->unCommandType, CMD_POLAR_DIR_ADDR) == CMD_POLAR_DIR_NEGATIVE)
			{
				cDir = -1;
			}
			getSingleAxisRealTimePoint(&dCPoint, AXIS_C, ucMapArray);
			//极坐标转化为绝对坐标
			polarCoord2CommonCoord((double*)&STargetPoint, AXIS_NUM_MAX, g_SSystemPara.ucPolarInterpolationAxis[0], g_SSystemPara.ucPolarInterpolationAxis[1], cDir);
			//绝对坐标转化为机械坐标
			absPoint2RealTimePoint(pSG2Pointer->pSChannel->ucChannelNum, (double*)&STargetPoint, AXIS_NUM_MAX, pSG2Pointer->ucValStatus ^ (0x01 << AXIS_C));

			//旋转轴合法性判断 当前点和目标点距离过大
			STargetPoint.dArray[AXIS_C] = STargetPoint.dArray[AXIS_C] < 0 ? 360.0 + STargetPoint.dArray[AXIS_C] : STargetPoint.dArray[AXIS_C];
			STargetPoint.dArray[AXIS_C] = (int)(dCPoint / 360) * 360 + STargetPoint.dArray[AXIS_C];
			if (fabs(STargetPoint.dArray[AXIS_C] - dCPoint) >= 180)
			{
				STargetPoint.dArray[AXIS_C] = round(dCPoint / 360) * 360 - (int)(dCPoint / 360) * 360 + STargetPoint.dArray[AXIS_C];
			}
			if (fabs(STargetPoint.dArray[AXIS_C] - dCPoint) >= 180)
			{
				STargetPoint.dArray[AXIS_C] -= sgn(STargetPoint.dArray[AXIS_C] - dCPoint) * 360;
			}
		}
		else if (getBits(pSG2Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_CYLINDRICAL)
		{
			STargetPoint.dArray[AXIS_C] = cylinderCoord2Common(STargetPoint.dArray[AXIS_C], pSG2Pointer->fCylinderRadius);
		}
		//椭圆 2023.3.29
#if	EllipseTEST
		if (getBits(pSG2Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ELLIPSE_ARC
			|| getBits(pSG2Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ELLIPSE_ANTI_ARC)
		{
			//圆弧坐标转换成椭圆坐标
			commonArc2Ellipse((double*)&STargetPoint, (double*)&pSG2Pointer->SEllipseSCenterPoint, (double*)&STargetPoint, AXIS_NUM_MAX,
				pSG2Pointer->dA, pSG2Pointer->dB, pSG2Pointer->dCoordRotation, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		}
#endif
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
		if (pSG2Pointer->pSChannel->bUseVirtualY == TRUE)
		{
			if ((getbit(pSG2Pointer->ucValStatus, AXIS_Y)) == TRUE)//Y轴方向要运动
			{
				STargetPoint.dArray[AXIS_X] += 1.0 * STargetPoint.dArray[AXIS_Y] * pSG2Pointer->pSChannel->dTANAngleYY1;//将直角坐标系下的坐标点转化为倾斜坐标系下的点
				STargetPoint.dArray[AXIS_Y] = STargetPoint.dArray[AXIS_Y] / pSG2Pointer->pSChannel->dCOSAngleYY1;
		}
		}
#endif
#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE //单表螺距补偿+ 反向间隙补偿
		//利用理论坐标生成SRun结构体数据
		outPutAbsPointWithScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG2Pointer->dPulseToMm, &pSG2Pointer->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP //双表螺距补偿
		outPutAbsPointWithBilateralScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG2Pointer->dPulseToMm, &pSG2Pointer->ucValStatus);
#else
		//利用理论坐标生成SRun结构体数据
		outPutAbsPoint(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG2Pointer->dPulseToMm, &pSG2Pointer->ucValStatus);
#endif

		//计算当前速度
		pSRun->dCurrentSpeed = pSG2Pointer->dCurrentSpeed = fabs(pSG2Pointer->SPlanData.dCurLeftDist - (pSG2Pointer->SPlanData.dTargetDist - dDeltaL)) / pSG2Pointer->SPlanData.dTc;

		//计算   !理论! 未走距离
		pSG2Pointer->SPlanData.dCurLeftDist = pSG2Pointer->SPlanData.dTargetDist - dDeltaL;

		//判断软限位开始减速的时机：监测当前速度 >= 实时反算的最大速度（T型系统参数）
		if ((pSRun->cSoftLimitedOn == 1 || pSRun->cSoftLimitedOn == 2) && pSG2Pointer->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE)
		{
			dVmaxToLimitiedPoint = sqrt(pSG2Pointer->dReCalculateVe * pSG2Pointer->dReCalculateVe + 2 * g_SSystemPara.dSysPlanTMaxDec * pSG2Pointer->SPlanData.dCurLeftDist);//根据末速度、T型加速度、剩余距离实时反算T型曲线下的最大速度
			if (pSG2Pointer->dCurrentSpeed >= dVmaxToLimitiedPoint)
			{
				pSG2Pointer->ucInpStatus = INP_DECELERATE;//插补状态：减速停
				if (pSRun->cSoftLimitedOn == 2)//限位点在当前线段外
				{
					pSRun->cSoftLimitedOn = 3;//软限位开关变量置为3,需要后续线段参与减速
				}
				if (!isZero(pSG2Pointer->SPlanData.dCurLeftDist))//当前段剩余距离还未走完：进入减速停
				{
					pSG2Pointer->SPlanData.dDec = (pow(pSG2Pointer->dCurrentSpeed, 2) - pow(pSG2Pointer->dReCalculateVe, 2)) / (2.0 * pSG2Pointer->SPlanData.dCurLeftDist);//重算T型减速度
				}
				else//当前段剩余距离已走完：进入减速停报警
				{
					pSG2Pointer->SPlanData.dDec = 0;
					break;
				}
			}
		}
#if HANDWHEEL_SIM_DISTANCE_ENABLE
		if (
			(pSG2Pointer->bIsUserData || !isHandwheelAutomaticMoveRequired(pSG2Pointer->pSChannel)
				|| pSG2Pointer->SPlanData.unCurrentT >= pSG2Pointer->SPlanData.unT5) &&
			((pSG2Pointer->SPlanData.dCurLeftDist <= EPSINON && pSG2Pointer->SPlanData.cInterpDir == INP_GO_AHEAD)
			|| (pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dCurLeftDist <= EPSINON && pSG2Pointer->SPlanData.cInterpDir == INP_GO_BACK)))
#else
		if ((pSG2Pointer->SPlanData.dCurLeftDist <= EPSINON && pSG2Pointer->SPlanData.cInterpDir == INP_GO_AHEAD)
			|| (pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dCurLeftDist <= EPSINON && pSG2Pointer->SPlanData.cInterpDir == INP_GO_BACK))
#endif
		{
			pSRun->dCurrentSpeed = pSG2Pointer->dCurrentSpeed;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
			if (!pSG2Pointer->bIsUserData && isHandwheelAutomaticMoveRequired(pSG2Pointer->pSChannel)
				&& pSG2Pointer->SPlanData.dVe == 0.0)
			{
				pSRun->dCurrentSpeed = pSG2Pointer->dCurrentSpeed = 0.0;
			}
#endif
			pSG2Pointer->ucInpStatus = INP_FINISHED;
			pSPreprocess->cInterpDir = pSG2Pointer->SPlanData.cInterpDir;
			pSPreprocess->ucInpStatus = isZero(pSG2Pointer->SPlanData.dDistance - pSG2Pointer->SPlanData.dTargetDist) ? INP_FINISHED : INP_PAUSING;
			return 0;
		}

		break;

	case INP_DECELERATE://指令减速 (梯形减速.无需规划(利用加速度和当前速度计算))
		if (pSRun->cSoftLimitedOn == 0)//未触发软限位的减速运动[速度模式]
		{
			if (pSG2Pointer->dCurrentSpeed <= g_SSystemPara.dSysMinVel)// 速度达到末速度范围内  立即停
			{
				pSG2Pointer->dCurrentSpeed = 0;//运动结束 当前速度置为0
				pSG2Pointer->ucInpStatus = INP_FINISHED;
				pSPreprocess->ucInpStatus = INP_FINISHED;
				break;
			}
			pSG2Pointer->SPlanData.dDec = g_SSystemPara.dSysPlanTMaxDec;//获取系统参数设置的T型减速度
		}

		if (pSG2Pointer->dCurrentSpeed >= pSG2Pointer->SPlanData.dDec * pSG2Pointer->SPlanData.dTc)
		{
			//计算长轴相对起点的距离 利用加速度计算//2024/12/17 注释：减速距离公式错误，匀减速每个周期的位移增量为Vcur * 1 - 0.5 * Dec * 1 * 1。
			//dDeltaL = pSG2Pointer->SPlanData.dPreDist + pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dCurLeftDist + pSG2Pointer->dCurrentSpeed - pSG2Pointer->SPlanData.dDec;
			dDeltaL = pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dCurLeftDist + pSG2Pointer->dCurrentSpeed - 0.5 * pSG2Pointer->SPlanData.dDec;
		}
		else
		{
			if (pSRun->cSoftLimitedOn == 0)//未触发软限位的减速运动[速度模式]：最后一个周期要求速度停下，位置无要求
			{
				dDeltaL = pSG2Pointer->SPlanData.dTargetDist - pSG2Pointer->SPlanData.dCurLeftDist;
			}
			else//触发软限位的减速运动[位置模式]：最后一个周期要求停到终点位置
			{
				dDeltaL = pSG2Pointer->SPlanData.dTargetDist;
				pSG2Pointer->SPlanData.dCurLeftDist = 0;
			}
		}

		//判断是否会超过终点
		if (dDeltaL > pSG2Pointer->SPlanData.dTargetDist)
		{
			dDeltaL = pSG2Pointer->SPlanData.dTargetDist;
		}

		//计算理论坐标点
		res = calculateArcInterpolationPoint(dDeltaL, pSG2Pointer, (double*)&STargetPoint);
		if (res < 0) return res;
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
		if (pSG2Pointer->pSChannel->bUseVirtualY == TRUE)
		{
			if ((getbit(pSG2Pointer->ucValStatus, AXIS_Y)) == TRUE)//Y轴方向要运动
			{
				STargetPoint.dArray[AXIS_X] += 1.0 * STargetPoint.dArray[AXIS_Y] * pSG2Pointer->pSChannel->dTANAngleYY1;//将直角坐标系下的坐标点转化为倾斜坐标系下的点
				STargetPoint.dArray[AXIS_Y] = STargetPoint.dArray[AXIS_Y] / pSG2Pointer->pSChannel->dCOSAngleYY1;
			}
		}
#endif
#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE //单表螺距补偿+ 反向间隙补偿
		//利用理论坐标生成SRun结构体数据
		outPutAbsPointWithScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG2Pointer->dPulseToMm, &pSG2Pointer->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP //双表螺距补偿
		outPutAbsPointWithBilateralScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG2Pointer->dPulseToMm, &pSG2Pointer->ucValStatus);
#else
		//利用理论坐标生成SRun结构体数据
		outPutAbsPoint(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG2Pointer->dPulseToMm, &pSG2Pointer->ucValStatus);
#endif

		//计算当前速度 //2024/12/12 注释：该公式计算的是插补周期1ms的平均速度（移动量/1ms），而在减速过程计算相对起点的公式计算使用的是移动后的瞬时速度
		//pSRun->dCurrentSpeed = pSG2Pointer->dCurrentSpeed = fabs(pSG2Pointer->SPlanData.dCurLeftDist - (pSG2Pointer->SPlanData.dTargetDist - dDeltaL)) / pSG2Pointer->SPlanData.dTc;
		pSRun->dCurrentSpeed = pSG2Pointer->dCurrentSpeed = pSG2Pointer->dCurrentSpeed - pSG2Pointer->SPlanData.dDec * pSG2Pointer->SPlanData.dTc;
		//计算   !理论! 未走距离
		pSG2Pointer->SPlanData.dCurLeftDist = pSG2Pointer->SPlanData.dTargetDist - dDeltaL;

		if (pSG2Pointer->SPlanData.dCurLeftDist <= EPSINON)//运行结束条件
		{
			if (pSRun->cSoftLimitedOn == 1 && pSG2Pointer->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE)//当前线段能够停下
			{
				for (int i = 0; i < AXIS_NUM_MAX; i++)//寻找出触发软限位的轴并报警
				{
					/* 只消费本段有效轴的限位标记，其他运动留下的标记不在此报警或清除。 */
					if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)
					{
						parseRunningError(pSG2Pointer->pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM41 + i, 9999);
						pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_MODE; //限位模式 切换为单轴模式
//						setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数 //注释2025.12.11 在软限位报警后需要主轴停
						break;
					}
					if (pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)
					{
						parseRunningError(pSG2Pointer->pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM47 + i, 9999);
						pSG2Pointer->pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_MODE; //限位模式 切换为单轴模式
//						setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数 //注释2025.12.11 在软限位报警后需要主轴停
						break;
					}
				}
				pSG2Pointer->dCurrentSpeed = 0;//运动结束 当前速度置为0
				pSRun->cSoftLimitedOn = 0;//软限位开关关闭：开关变量置0
				pSG2Pointer->pSChannel->ucChannelStatus = SYS_IDLE;//停下后系统状态置为空闲
			}
			else if (pSRun->cSoftLimitedOn == 3 && pSG2Pointer->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE)//软限位点不在当前线段，仍需减速
			{
				pSRun->dCurrentSpeed = pSG2Pointer->dCurrentSpeed;//当前速度保存在运行结构体里
			}
			else
			{
				pSG2Pointer->dCurrentSpeed = 0;//运动结束 当前速度置为0
			}
			pSG2Pointer->ucInpStatus = INP_FINISHED; //状态改变为FINISHED
			pSPreprocess->ucInpStatus = INP_FINISHED;
			return 0;
		}
		break;

	case INP_STOP://指令立即停

		pSG2Pointer->dCurrentSpeed = 0;//运动结束 当前速度置为0
		pSG2Pointer->ucInpStatus = INP_FINISHED; //状态改变为FINISHED
		pSPreprocess->ucInpStatus = INP_FINISHED;
		break;
	}
	return 1;
}

/*************************************************
  函数名称：double calcArcAngle(double *dCenterPoint, double *dStartPoint, double *dEndPoint, uint8 ucInpDir)
  函数功能：计算圆弧上两点之间的沿插补方向的圆心角（0-360度）
  输入参数：pCenterPoint-指向圆心坐标数组，pStartPoint-指向起点坐标数组、pEndPoint-指向终点坐标数组，插补方向ucInpDir（0=G02，1=G03）
  输出参数：
  返 回 值：起始向量与终点向量的夹角（0-360度）
  备    注：
**************************************************/
double calcArcAngle(double *pCenterPoint, double *pStartPoint, double *pEndPoint, uint8 ucInpDir) // 2025.9.27
{
	// 步骤1：计算向量OA、OB（A、B相对于圆心的向量）
	double dStartVector[2];								// 起始向量
	double dEndVector[2];								// 结束向量
	dStartVector[0] = pStartPoint[0] - pCenterPoint[0]; // 起始向量的横轴
	dStartVector[1] = pStartPoint[1] - pCenterPoint[1]; // 起始向量的纵轴
	dEndVector[0] = pEndPoint[0] - pCenterPoint[0];		// 结束向量的横轴
	dEndVector[1] = pEndPoint[1] - pCenterPoint[1];		// 结束向量的纵轴

	// 步骤2：计算向量的点积和模（求夹角的余弦值）
	double dDot = dStartVector[0] * dEndVector[0] + dStartVector[1] * dEndVector[1];				// 向量的点积
	double dStartLen = sqrt(dStartVector[0] * dStartVector[0] + dStartVector[1] * dStartVector[1]); // 起始向量的模
	double dEndlen = sqrt(dEndVector[0] * dEndVector[0] + dEndVector[1] * dEndVector[1]);			// 结束向量的模

	// 步骤3：计算“最小圆心角”（0~π）
	double dCosValue = dDot / (dStartLen * dEndlen);
	// 限制dCosValue在[-1,1]内（避免浮点误差导致acos返回NaN）
	dCosValue = (dCosValue > 1.0) ? 1.0 : (dCosValue < -1.0) ? -1.0
															 : dCosValue;
	double dAngle = acos(dCosValue);

	// 步骤4：根据ucInpDir方向，确定“实际路径的圆心角”（0~2π）
	double dCrossDir = (dStartVector[0] * dEndVector[1]) - (dStartVector[1] * dEndVector[0]); // 起点向量 × 终点向量（叉乘判断方向）
	if (ucInpDir == 1)																		  // G03（逆时针）：若叉积为负，需走2π - dAngle（长弧）
	{
		return (dCrossDir >= 0) ? dAngle : (2 * PI - dAngle);
	}
	else // G02（顺时针）：若叉积为正，需走2π - dAngle（长弧）
	{
		return (dCrossDir <= 0) ? dAngle : (2 * PI - dAngle);
	}
}

/*************************************************
  函数名称：checkG2FirstAxisSoftLimit
  函数功能：比较有效直线轴的正负软限位，标记沿圆弧最先到达的边界
  输入参数：pSChannel - 已加载 SG02 的通道；SStartPoint - 当前圆弧起点
            SMaxVal、SMinVal - 当前圆弧各轴最值，与限位使用同一坐标系
  输出参数：重建本段有效轴的限位标记；无效几何时标记该轴交由 motionArc 报警
  返 回 值：1 - 有待处理限位；-1 - 无限位或通道参数无效
  备    注：只读系统轴映射、轴类型和限位配置；不提交脉冲，不跳过同轴另一侧
****************************************/
int8 checkG2FirstAxisSoftLimit(struct SChannel *pSChannel, SPointDouble SStartPoint, SPointDouble SMaxVal, SPointDouble SMinVal)
{
    double dNearestPosLimited = 9999.9; // 用于判断最先触发正软限位轴的夹角变量
    double dNearestNegLimited = 9999.9; // 用于判断最先触发负软限位轴的夹角变量
    int8 cPosLimitedAXIS = -1;          // 最先触发正软限位的轴号
    int8 cNegLimitedAXIS = -1;          // 最先触发负软限位的轴号
    double dRad1 = 0;                   // 圆弧与软限位点之间的运动夹角1
    double dRad2 = 0;                   // 圆弧与软限位点之间的运动夹角2
    double dPosMinRad = 10000;          // 正软限位点最小运动夹角
    double dNegMinRad = 10000;          // 负软限位点最小运动夹角
    double dCenterPoint[2];             // 0-圆心横轴坐标,1-圆心纵轴坐标
    double dStartPoint[2];              // 0-起点横轴坐标,1-起点纵轴坐标

    uint8* pMapAxisArray = pSChannel->pMapAxisArray; // 获取映射
    struct SG2* pSG2Pointer = &pSChannel->SG02;      // 获取G2结构体指针

    // 圆弧参数
    uint8 ucInpDir =
        (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC) ? 1 : 0; // 插补方向
    dCenterPoint[0] = pSG2Pointer->SCenterPoint.dArray[pSG2Pointer->ucAxis[0]];                          // 圆心横轴坐标
    dCenterPoint[1] = pSG2Pointer->SCenterPoint.dArray[pSG2Pointer->ucAxis[1]];                          // 圆心纵轴坐标
    dStartPoint[0] = SStartPoint.dArray[pSG2Pointer->ucAxis[0]];                                         // 起点横轴坐标
    dStartPoint[1] = SStartPoint.dArray[pSG2Pointer->ucAxis[1]];                                         // 起点纵轴坐标
    double dRadius = pSG2Pointer->dRadius;                                                               // 圆弧半径
	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		/* 沿用有效轴及旋转轴排除规则，映射越界不访问物理轴参数。 */
		if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][i] == AXIS_NULL
			||
            g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == ROTATION_AXIS) // 轴映射为空或旋转轴，跳过
            continue;

        if (VAL_VALID != getbit(pSChannel->SPreProcess.SUserData.ucValStatus, i)) // 无效轴，跳过
            continue;

        // 正限位检查
        if (g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].soft_pos == 1) // 开启正限位检测
		{
            if (SMaxVal.dArray[i] > g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft) // 最值点 > 正限位点
            {
                if (i == pSG2Pointer->ucAxis[0]) // 触发限位的轴为插补平面的横轴
                {
                    // 计算交点
                    double dValue = sqrt(dRadius * dRadius - pow(g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft -
                                                                     pSG2Pointer->SCenterPoint.dArray[i],
                                                                 2));
                    double dCrossPoint1 = dValue + dCenterPoint[1];  // 计算横轴软限位与圆弧的交点纵轴坐标较大值
                    double dCrossPoint2 = -dValue + dCenterPoint[1]; // 计算横轴软限位与圆弧的交点纵轴坐标较小值
                    double dEndPoint1[2] = {g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft,
                                            dCrossPoint1}; // 圆弧1的终点
                    double dEndPoint2[2] = {g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft,
                                            dCrossPoint2}; // 圆弧2的终点
                    dRad1 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint1,
                                         ucInpDir); // 计算圆弧与软限位点之间的运动夹角1
                    dRad2 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint2,
                                         ucInpDir); // 计算圆弧与软限位点之间的运动夹角2
                    dPosMinRad = min(dRad1, dRad2); // 取最小值，设置最先触发限位的夹角
                }
                else if (i == pSG2Pointer->ucAxis[1]) // 触发限位的轴为插补平面的纵轴
                {
                    // 计算交点
                    double dValue = sqrt(dRadius * dRadius - pow(g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft -
                                                                     pSG2Pointer->SCenterPoint.dArray[i],
                                                                 2));
                    double dCrossPoint1 = dValue + dCenterPoint[0];  // 计算纵轴软限位与圆弧的交点横轴坐标较大值
                    double dCrossPoint2 = -dValue + dCenterPoint[0]; // 计算纵轴软限位与圆弧的交点横轴坐标较小值
                    double dEndPoint1[2] = {dCrossPoint1,
                                            g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft}; // 圆弧1的终点
                    double dEndPoint2[2] = {dCrossPoint2,
                                            g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft}; // 圆弧2的终点
                    dRad1 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint1,
                                         ucInpDir); // 计算圆弧与软限位点之间的运动夹角1
                    dRad2 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint2,
                                         ucInpDir); /// 计算圆弧与软限位点之间的运动夹角2
                    dPosMinRad = min(dRad1, dRad2); // 取最小值，设置最先触发限位的夹角
                }

                if (dNearestPosLimited - dPosMinRad > EPSINON) // 寻找最先触发限位的轴
                {
                    cPosLimitedAXIS = i; // 保存轴号
                    dNearestPosLimited = dPosMinRad;
			continue;
			}
		}
	}
	/* 确有边界候选才发布限位模式，后续仍由原状态机减速和报警。 */
        if (g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].soft_neg == 1)
	{
            if (SMinVal.dArray[i] < g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft)
            {
                if (i == pSG2Pointer->ucAxis[0]) // 触发限位的轴为插补平面的横轴
                {
                    // 计算交点
                    double dValue = sqrt(dRadius * dRadius - pow(g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft -
                                                                     pSG2Pointer->SCenterPoint.dArray[i],
                                                                 2));
                    double dCrossPoint1 = dValue + dCenterPoint[1];  // 计算横轴软限位与圆弧的交点纵轴坐标较大值
                    double dCrossPoint2 = -dValue + dCenterPoint[1]; // 计算横轴软限位与圆弧的交点纵轴坐标较小值
                    double dEndPoint1[2] = {g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft, dCrossPoint1};
                    double dEndPoint2[2] = {g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft, dCrossPoint2};
                    dRad1 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint1,
                                         ucInpDir); // 计算圆弧与软限位点之间的运动夹角1
                    dRad2 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint2,
                                         ucInpDir); /// 计算圆弧与软限位点之间的运动夹角2
                    dNegMinRad = min(dRad1, dRad2); // 取最小值，设置最先触发限位的夹角
                }
                else if (i == pSG2Pointer->ucAxis[1]) // 触发限位的轴为插补平面的纵轴
                {
                    // 计算交点
                    double dValue = sqrt(dRadius * dRadius - pow(g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft -
                                                                     pSG2Pointer->SCenterPoint.dArray[i],
                                                                 2));
                    double dCrossPoint1 = dValue + dCenterPoint[0];  // 计算纵轴软限位与圆弧的交点横轴坐标较大值
                    double dCrossPoint2 = -dValue + dCenterPoint[0]; // 计算纵轴软限位与圆弧的交点横轴坐标较小值
                    double dEndPoint1[2] = {dCrossPoint1, g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft};
                    double dEndPoint2[2] = {dCrossPoint2, g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft};
                    dRad1 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint1,
                                         ucInpDir); // 计算圆弧与软限位点之间的运动夹角1
                    dRad2 = calcArcAngle(dCenterPoint, dStartPoint, dEndPoint2,
                                         ucInpDir); /// 计算圆弧与软限位点之间的运动夹角2
                    dNegMinRad = min(dRad1, dRad2); // 取最小值，设置最先触发限位的夹角
                }
                if (dNearestNegLimited - dNegMinRad > EPSINON) // 寻找最先触发限位的轴
                {
                    cNegLimitedAXIS = i; // 保存轴号
                    dNearestNegLimited = dNegMinRad;
                    continue;
			}
            }
        }
    }

    if (cPosLimitedAXIS != -1 || cNegLimitedAXIS != -1) // 判断是否有触发软限位的轴，并设置模式为限位模式
    {
        if (cPosLimitedAXIS != -1 &&
            cNegLimitedAXIS != -1) // 比较正软限位和负软限位的最小距离比值，则为优先触发轴限位，并设置模式为限位模式
        {
            if (dNearestPosLimited > dNearestNegLimited)
			{
				pSChannel->SSingleAxisArray[cNegLimitedAXIS].ucRunMode =
                    SINGLEAXIS_LIMIT_NEG; // 单轴运行模式设置为负限位
            }
            else
            {
                pSChannel->SSingleAxisArray[cPosLimitedAXIS].ucRunMode =
                    SINGLEAXIS_LIMIT_POS; // 单轴运行模式设置为正限位
            }
        }
        else if (cPosLimitedAXIS != -1)
	{
		pSChannel->SSingleAxisArray[cPosLimitedAXIS].ucRunMode = SINGLEAXIS_LIMIT_POS; // 单轴运行模式设置为正限位
        }
        else // cNegLimitedAXIS != -1
        {
            pSChannel->SSingleAxisArray[cNegLimitedAXIS].ucRunMode = SINGLEAXIS_LIMIT_NEG; // 单轴运行模式设置为负限位
        }
		return 1;
	}
	return -1;
}
