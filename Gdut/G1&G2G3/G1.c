#include "G1.h"
#include "GeneralFunctions.h"
#include "Motion.h"
#include "lookAhead.h"
#include "GeneralFunctions.h"
#include <stdlib.h>
#include "CTrajPlanning.h"
#include "Channel.h"
#include "ringbuff.h"
#include "Config.h"
#include "ErrorCompensation.h"
#include "math.h"
#include "ToolCompensation.h"
//struct SG1 g_SG1;

///*************************************************
//  函数名称：calculateExactInterpolationPoint(double dDeltaL ,struct SInterpolationPre *pInterpPrePointer)
//  函数功能  根据传入的插补结构体和dDeltaL 计算当前插补点的绝对坐标(没有取整)
//  输入参数：dDeltaL				double类型变量			目标点距起点的位移（或弧长）
//			pInterpPrePointer	SInterpolationPre指针	当前插补结构体地址
//  输出参数：
//  返 回 值：SPointDouble SAbsCoord    目标坐标点 （绝对坐标）
//  备    注：
//**************************************************/
//SPointDouble calculateExactInterpolationPoint(double dDeltaL, struct SG1* pSG123Pointer)
//{
//	//不需要初始化的参数
//	SPointDouble SRelacoord;    //相对坐标  相对起点(直线)  或  相对圆心(圆弧)
//
//
//	//需要初始化的参数
//	SPointDouble SStartPoint;		//起点，单位：mm
//	SPointDouble SRelaCenterPoint;	//圆心相对坐标(相对起点)，单位：mm
//	double dRadius;					//圆弧半径，单位：mm
//	int8 cArcDir;					//圆弧方向，顺时针=1，逆时针=-1
//	SPointDouble SAbsCoord;		//目标坐标点 （绝对坐标）
//
//	SAbsCoord.dX = 0;  //初始化目标坐标点
//	SAbsCoord.dY = 0;
//	SAbsCoord.dZ = 0;
//
//	if (pSG123Pointer == NULL) //空指针  不运行
//	{
//		return SAbsCoord;
//	}
//
//	//初始化参数
//	SStartPoint = pSG123Pointer->SStartPoint;	//起点，单位：mm
//	dRadius = pSG123Pointer->dRadius;		//圆弧半径，单位：mm
//	cArcDir = pSG123Pointer->cArcDir;		//圆弧方向，顺时针=1，逆时针=-1
//
//	SRelaCenterPoint.dX = pSG123Pointer->SCenterPoint.dX - SStartPoint.dX;  //圆心坐标计算（相对起点）
//	SRelaCenterPoint.dY = pSG123Pointer->SCenterPoint.dY - SStartPoint.dY;
//	SRelaCenterPoint.dZ = pSG123Pointer->SCenterPoint.dZ - SStartPoint.dZ;
//
//
//
//	if (getBits(pSG123Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC || getBits(pSG123Pointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)//圆弧插补
//	{
//		double dDeltaRad;//圆心角增量 （相对起点）
//
////		dDeltaRad = (-1) * cArcDir * dDeltaL / dRadius;	//计算圆心角增量
//		dDeltaRad = cArcDir * dDeltaL / dRadius;	//计算圆心角增量（后置刀架）
//		SRelacoord.dX = -SRelaCenterPoint.dX * cos(dDeltaRad) + SRelaCenterPoint.dZ * sin(dDeltaRad);  //计算坐标x（相对圆心）
//		SRelacoord.dZ = -SRelaCenterPoint.dZ * cos(dDeltaRad) - SRelaCenterPoint.dX * sin(dDeltaRad);//计算坐标z（相对圆心）
//
//		//计算绝对坐标  （=起点坐标+圆心相对坐标+相对坐标）
//		SAbsCoord.dX = SStartPoint.dX + SRelaCenterPoint.dX + SRelacoord.dX;
//		SAbsCoord.dZ = SStartPoint.dZ + SRelaCenterPoint.dZ + SRelacoord.dZ;
//	}
//	else 	//直线插补
//	{
//		//计算相对坐标（相对起点）
//		SRelacoord.dX = pSG123Pointer->SStartVector.dIX * dDeltaL;
//		SRelacoord.dY = pSG123Pointer->SStartVector.dIY * dDeltaL;
//		SRelacoord.dZ = pSG123Pointer->SStartVector.dIZ * dDeltaL;
//
//		//计算绝对坐标  （=起点坐标+相对坐标）
//		SAbsCoord.dX = SRelacoord.dX + SStartPoint.dX;
//		SAbsCoord.dY = SRelacoord.dY + SStartPoint.dY;
//		SAbsCoord.dZ = SRelacoord.dZ + SStartPoint.dZ;
//	}
//
//
//
//	return SAbsCoord;
//
//}

/*************************************************
  函数名称：SG1LoadSysPara(struct SG1* pSG1)
  函数功能  SG1初始化函数
  输入参数：pSG1		SG1指针
  输出参数：
  返 回 值：-1 数组长度错误
			-2 传入空指针
			1  处理完成
  备    注：必须在通道初始化之后执行
**************************************************/
void SG1LoadSysPara(struct SG1* pSG1)
{
    int i;
    const uint8* ucMapArray; //映射
    if (pSG1 == NULL)
    {
        return -1;
    }
    //通过通道获取映射
    ucMapArray = pSG1->pSChannel->pMapAxisArray;

    //G1脉冲当量初始化
    for (i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX)
            continue; //映射为空  跳过
        pSG1->dPulseToMm[i] = g_SSystemPara.SAxisPara[ucMapArray[i]].dPulseToMm;
    }
    //插补周期
    pSG1->SPlanData.dTc = g_SSystemPara.dInterpolationTime;
    //进给倍率
    pSG1->dFeedRatio = 1.0;
}

/*************************************************
  函数名称：calVectorAndDistance(double * pdStatPoint, double *pdEndPoint,double *dDistance,double  *pdOutputVector,int nArraySize)
  函数功能  根据起点和终点   计算长轴和   距离分量
  输入参数：pdStatPoint	起点坐标
			pdEndPoint	终点坐标
			nArraySize  起点终点分量的数组长度
  输出参数：dDistance	 长轴距离
			pdOutputVector 距离分量
  返 回 值：TRUE:  计算成功
			FALSE: 参数错误
  备    注：
**************************************************/
int8 calVectorAndDistance(double* pdStatPoint, double* pdEndPoint, double* dDistance, double* pdOutputVector,
                          int nArraySize, VAL_STRUCT_TYPE ucValStatus)
{
    int8 res = 0;
    double dMaxDistance = 0; // 计算的距离
    int i = 0;

    //寻找最长轴  （绝对值）
    for (i = 0; i < nArraySize; i++)
    {
        if (VAL_VALID != getbit(ucValStatus, i))
            continue;                                        //无效轴  跳过
        double dDist = fabs(pdEndPoint[i] - pdStatPoint[i]); //当前轴移动的距离
        if (dDist > dMaxDistance)
        {
            res = i;
            dMaxDistance = dDist;
        }
        //dMaxDistance = max(dMaxDistance, fabs(pdEndPoint[i] - pdStatPoint[i]));
    }

    //传出距离值
    if (dDistance != NULL)
    {
        *dDistance = dMaxDistance;
    }

    //距离值为0  分量全部为0  （防止除0操作）
    if (isZero(dMaxDistance))
    {
        if (pdOutputVector != NULL) //指针不为空 传出数据
        {
            for (i = 0; i < nArraySize; i++)
            {
                pdOutputVector[i] = 0; //距离分量都为0
            }
        }
        return -1;
    }

    //传出距离分量
    if (pdOutputVector != NULL) //指针不为空 传出数据
    {
        for (i = 0; i < nArraySize; i++)
        {
            //距离分量 = 当前轴位移/最长轴位移
            if (VAL_VALID != getbit(ucValStatus, i))
            {
                pdOutputVector[i] = 0;
            }
            else
            {
                pdOutputVector[i] = (pdEndPoint[i] - pdStatPoint[i]) / dMaxDistance;
            }
        }
    }
    return res;
}

/*************************************************
  函数名称：calcG84VectorAndDistance
  函数功能：G84指令Z/C轴分量计算函数（以Z轴为长轴，计算C轴的运动分量）
  输入参数：pdStatPoint		 起点
		   pdEndPoint	     终点
		   dMotionRatio	Z轴与C轴相对运动系数
		   nArraySize   数组长度
  输出参数：pdDistance 	      长轴距离
		   pdOutputVector	 距离分量
  返 回 值：res 长轴轴号
			0  不执行
  备    注：2025.11.15
**************************************************/
int8 calcG84VectorAndDistance(double* pdStatPoint, double* pdEndPoint, double* pdDistance, double* pdOutputVector,
                              int nArraySize, VAL_STRUCT_TYPE ucValStatus, double dMotionRatio)
{
    double dMaxDistance = 0; // 长轴距离
    int8 res = 0;            // 最长轴轴号

    // 1. 获取c轴作为长轴距离，取绝对值确保为正
    dMaxDistance = fabs(pdEndPoint[AXIS_Z] - pdStatPoint[AXIS_Z]) * fabs(dMotionRatio); // 由z轴转换为c轴移动的距离
    res = AXIS_C;
    *pdDistance = dMaxDistance; // 传出距离值

    // 距离值为0  分量全部为0  （防止除0操作）
    if (isZero(dMaxDistance))
    {
        if (pdOutputVector != NULL) // 指针不为空 传出数据
        {
            for (int i = 0; i < nArraySize; i++)
            {
                pdOutputVector[i] = 0; // 距离分量都为0
            }
        }
        return -1;
    }

    // 2. 获取距离分量
    if (pdOutputVector != NULL) // 指针不为空 传出数据
    {
        for (int i = 0; i < nArraySize; i++)
        {
            // 距离分量 = 当前轴位移/最长轴位移
            if (VAL_VALID != (getbit(ucValStatus, i)))
            {
                pdOutputVector[i] = 0; // 其他距离分量都为0
            }
            else if (i == AXIS_C)
            {
                pdOutputVector[AXIS_C] = sgn(dMotionRatio); // 按比例计算C轴分量（匹配攻丝进给/退刀方向）
            }
            else
            {
                pdOutputVector[i] = (pdEndPoint[i] - pdStatPoint[i]) / dMaxDistance; // 距离分量 = 当前轴位移/最长轴位移
            }
        }
    }

    return res; // 计算成功

    return res; // 计算成功
}

/*************************************************
  函数名称：calculateLinearInterpolationPoint( double * pdStartPoint, double * pdVector,double dDeltaL,double *pdOutPutPoint ,int nArraySize)
  函数功能  根据传入的插补结构体和dDeltaL 计算当前插补点的绝对坐标(没有取整)
  输入参数：pdStartPoint		起点
			pdVector			距离分量
			dDeltaL				长轴距起点的距离
			nArraySize			数组长度
  输出参数：pdOutPutPoint		传出目标点的指针变量
  返 回 值：-1 数组长度错误
			-2 传入空指针
			1  处理完成
  备    注：
**************************************************/
int8 calculateLinearInterpolationPoint(double* pdStartPoint, double* pdVector, double dDeltaL, double* pdOutPutPoint,
                                       int nArraySize)
{
    int i = 0;

    if (nArraySize <= 0)
        return -1; //数组长度错误
    if (pdStartPoint == NULL || pdVector == NULL || pdOutPutPoint == NULL)
    {
        return -2; // 空指针
    }

    //根据距离和距离分量 计算下一点的坐标
    for (i = 0; i < nArraySize; i++)
    {
        pdOutPutPoint[i] = pdStartPoint[i] + pdVector[i] * dDeltaL;
    }

    return 1;
}

/*************************************************
  函数名称：prepareToPauseG1(struct SG01* pSG1Pointer, double dVmin, double dDec, bool bWriteBackFlag)
  函数功能  根据插补预处理结构体 以减速模型来初始化G1，确定距离和终点速度，方便进行速度规划
  输入参数：pSG1Pointer			SG01 结构体指针，当前插补的结构体
			dVmin				减速停的目标速度
			dDec				减速度
			bWriteBackFlag		bool类型	是否执行回写操作
  输出参数：
  返 回 值：-1 输入错误
			1 初始化完成
  备    注：
**************************************************/
int8 prepareToPauseG1(struct SG1* pSG1Pointer, double dVmin, double dDec, bool bWriteBackFlag)
{
    double dVs;                                         //起始速度 （最大速度  纯减速）
    double dLeftDist;                                   //剩余距离
    double dSde;                                        //从最大速度减速到起跳速度所需距离
    double dVe;                                         //最终速度
    SPointDouble SEndPoint;                             //插补终点
    struct SToolCompensation* pSToolCompPointer = NULL; //刀补指针
    struct SInterpolationPre* pPreProcessPointer;       //预处理指针
    double dWriteBackDist = 0;                          //回写的已走距离
    bool bIsSlowingDown = FALSE;                        //是否正在减速
    bool bStoppable = FALSE;                            //当前段能否停下  2023.6.3 朱稳中
    double dDecFactor;                                  //减速公式系数
    if (pSG1Pointer == NULL || dDec <= 0)               //空指针不运行
    {
        return -1;
    }

    pPreProcessPointer = &pSG1Pointer->pSChannel->SPreProcess;       //预处理结构体
    dLeftDist = pSG1Pointer->SPlanData.dCurLeftDist;                 //剩余距离
    SEndPoint = pSG1Pointer->SEndPoint;                              //终点信息
    dDecFactor = getFactor(pSG1Pointer->SPlanData.ucDecVelPlanType); //根据减速曲线类型获取系数

    //当前在减速
    if (pSG1Pointer->SPlanData.unCurrentT < pSG1Pointer->SPlanData.unT5 &&
        pSG1Pointer->SPlanData.unCurrentT > pSG1Pointer->SPlanData.unT3)
    {
        bIsSlowingDown = TRUE;
        dSde = dLeftDist;
        dVe = pSG1Pointer->SPlanData.dVe;
    }
    else //当前不在减速  计算减速距离 和终点位置
    {
        //起始速度
        dVs = pSG1Pointer->dCurrentSpeed;

        if (dVs <= dVmin) //当前速度小于最小速度 可以直接停
        {
            dSde = 0; //S4型
        }
        else //不可以直接停
        {
            //计算减速距离
            dSde = fabs(dDecFactor * (dVs * dVs - dVmin * dVmin) / dDec); //减速公式
        }

        if (pSG1Pointer->SPlanData.cInterpDir != INP_GO_BACK) //正向
        {
            //计算终点位置
            if (dSde < dLeftDist || isZero(dSde - dLeftDist)) //距离足够减速
            {
                dVe = dVmin; //速度规划末速度 等于 系统起跳速度

                //计算终点位置 //2025/01/02 洪冠鹏  pSG1Pointer->SPlanData.dDistance - dLeftDist + dSde  -> pSG1Pointer->SPlanData.dTargetDist - dLeftDist + dSde
                calculateLinearInterpolationPoint(
                    (double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SStartVector,
                    pSG1Pointer->SPlanData.dTargetDist - dLeftDist + dSde, (double*)&SEndPoint, AXIS_NUM_MAX);
                dWriteBackDist = pSG1Pointer->SPlanData.dTargetDist - dLeftDist + dSde;
                pSG1Pointer->SPlanData.dCurLeftDist = dSde;
                pSG1Pointer->SPlanData.dTargetDist -= dLeftDist - dSde;
                //pSG1Pointer->SEndPoint = SEndPoint;//终点改变
                bStoppable = TRUE;
            }
            else //当前段不能停下
            {
                dVe = sqrt(dVs * dVs - dDec * dLeftDist / dDecFactor); //最终速度不能减到起跳速度  重算实际最终速度
                dSde = dLeftDist;
                //SEndPoint = pSG1Pointer->SEndPoint;//终点为原G1终点
            }
        }
        else // 反向   INP_GO_BACK
        {
            SPointDouble SNewStartPoint;
            dWriteBackDist = pSG1Pointer->SPlanData.dTargetDist - dLeftDist - dSde;

            if (isEqualOrGreaterThanZero(dWriteBackDist)) //当前段能够停下
            {
                dVe = dVmin;
                //计算新的起点位置
                calculateLinearInterpolationPoint((double*)&pSG1Pointer->SStartPoint,
                                                  (double*)&pSG1Pointer->SStartVector, dWriteBackDist,
                                                  (double*)&SNewStartPoint, AXIS_NUM_MAX);

                pSG1Pointer->SPlanData.dTargetDist -= dWriteBackDist; //目标距离
                pSG1Pointer->SPlanData.dPreDist -= dWriteBackDist;
                pSG1Pointer->SPlanData.dDistance -= dWriteBackDist;
                pSG1Pointer->SStartPoint = SNewStartPoint;
                bStoppable = TRUE;
            }
            else // 当前段不能停下
            {
                dVe = sqrt(dVs * dVs -
                           dDec * (dWriteBackDist + dSde) / dDecFactor); //最终速度不能减到起跳速度  重算实际最终速度
            }
        }
    }

    //执行回写
    if (bWriteBackFlag == TRUE &&
        (bIsSlowingDown == FALSE || pPreProcessPointer->sChannelCommand.ucType != UNASSIGNED_VALUE) &&
        bStoppable == TRUE) //条件:通道指令或插补执行不完
    {
        //写回刀补缓冲区
        moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_BACK, pSG1Pointer->pSChannel->ucChannelNum); //移回前瞻指针
        moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pSG1Pointer->pSChannel->ucChannelNum);  //移回前瞻指针
        pSToolCompPointer =
            getSToolCompPointerByType(INTERPOLATION_READ, pSG1Pointer->pSChannel->ucChannelNum); //获取插补读指针

        ////写回数据
        //回写的起点等于当前段的终点
        //pSToolCompPointer->SStartPoint = SEndPoint;
        //回写的终点等于G1的终点（已经过修正）
        //pSToolCompPointer->SEndPoint = pSG1Pointer->SEndPoint;
        //相对坐标标记置0
        //pSToolCompPointer->ucRelaVal = 0x0;

        pSToolCompPointer->dPreDist = dWriteBackDist;                           //写回距离
        pSToolCompPointer->dFeedrate = pSToolCompPointer->dUncorrectedFeedrate; //覆盖速度
        pSToolCompPointer->bHasVeerFlag = FALSE;
        pSToolCompPointer->bLookAheadVParaCalcOverFlag = FALSE;
        pSG1Pointer->pSChannel->bLookAheadNewBeginFlag = TRUE;
        initLookAheadDealed(pSG1Pointer->pSChannel->ucChannelNum); //初始化前瞻指针

        //if (bStoppable == TRUE)//指令将要执行完 将执行后作用IO  2023.6.15 朱稳中
        if (pSG1Pointer->SPlanData.cInterpDir == INP_GO_AHEAD &&
            isZero(pSG1Pointer->SPlanData.dDistance - pSG1Pointer->SPlanData.dTargetDist))
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

    if (bIsSlowingDown == FALSE) //当前段没有在减速 重新规划
    {
        //新的剩余距离= 减速所需的长度
        //pSG1Pointer->SPlanData.dCurLeftDist = dSde;

        ////当前线段长度： 减去暂停后剩下的距离
        //pSG1Pointer->SPlanData.dDistance -= dLeftDist - dSde;
        ////目标距离
        //pSG1Pointer->SPlanData.dTargetDist = pSG1Pointer->SPlanData.dDistance;
        //当前线段长度： 减去暂停后剩下的距离
        //pSG1Pointer->SPlanData.dDistance -= dLeftDist - dSde;
        //目标距离 = 原距离 - 减速停后剩余的距离
        //pSG1Pointer->SPlanData.dTargetDist -= dLeftDist - dSde;

        pSG1Pointer->SPlanData.dVs = pSG1Pointer->SPlanData.dVmax = dVs; //起始速度 （最大速度）
        pSG1Pointer->SPlanData.dVe = dVe;                                //最终速度
        pSG1Pointer->SPlanData.dDec = dDec;                              //减速度

        //曲线类型 1：梯形，2：3次S型，3：4次S型
        //pSG1Pointer->SPlanData.ucAccVelPlanType = VEL_PLAN_S4_CURVE;
        //pSG1Pointer->SPlanData.ucDecVelPlanType = VEL_PLAN_S4_CURVE;
        //平摊曲线类型，1：梯形，2：5次，3：比例系数
        //pSG1Pointer->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;

        preparePlanningAndDistReAvg(&pSG1Pointer->SPlanData, TRUE); //速度规划+平摊
        pSG1Pointer->SPlanData.unCurrentT = 0;                      //时间
    }

    return 1; //预处理完成 返回1
}

///*************************************************
//  函数名称：prepareToPauseG1(struct SG01* pSG1Pointer, double dVmin, double dDec, bool bWriteBackFlag)
//  函数功能  根据插补预处理结构体 以减速模型来初始化 原插补预处理结构体，确定距离和终点速度，方便进行速度规划
//  输入参数：pSG1Pointer			SG01 结构体指针，当前插补的结构体
//			dVmin				减速停的目标速度
//			dDec				减速度
//			bWriteBackFlag		bool类型	是否执行回写操作
//  输出参数：
//  返 回 值：-1 输入错误
//			1 初始化完成
//  备    注：
//**************************************************/
//int8 prepareToPauseG1(struct SG1* pSG1Pointer, double dVmin, double dDec, bool bWriteBackFlag)
//{
//	double dVs;   //起始速度 （最大速度  纯减速）
//	double dLeftDist;//剩余距离
//	double dSde;	//从最大速度减速到起跳速度所需距离
//	double dVe;		//最终速度
//	SPointDouble SEndPoint; //插补终点
//	struct SToolCompensation* pSToolCompPointer = NULL;
//
//
//	if (pSG1Pointer == NULL)   //空指针不运行
//	{
//		return -1;
//	}
//
//	pSG1Pointer->ucInpStatus = INP_INITIATION;//改变插补预处理结构体的状态
//	SEndPoint = pSG1Pointer->SEndPoint; //终点信息
//	dLeftDist = pSG1Pointer->SPlanData.dCurLeftDist;  //剩余距离
//	if (bWriteBackFlag == TRUE)
//	{
//		if (getSToolCompPointerBufRes(INTERPOLATION_READ, pSG1Pointer->pSChannel->ucChannelNum) <= 0)//前瞻处理过的段数为0
//		{
//			moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_AHEAD, pSG1Pointer->pSChannel->ucChannelNum);  //移动前瞻
//			pSG1Pointer->pSChannel->bLookAheadNewBeginFlag = TRUE;  //开机、新开始需要置TRUE
//		}
//	}
//
//	//起始速度
//	dVs = pSG1Pointer->dCurrentSpeed;
//
//	if (dVs <= dVmin)//当前速度小于最小速度 可以直接停
//	{
//		dSde = 0;//S4型
//		pSG1Pointer->ucInpStatus = INP_FINISHED;
//	}
//	else//不可以直接停
//	{
//		int nIndex = doubleArrayMax(pSG1Pointer->dPulseToMm, AXIS_NUM_MAX, NULL);
//		//计算减速距离
//		dSde = round(fabs(3 * (dVs * dVs - dVmin * dVmin) / (4 * dDec)) / pSG1Pointer->dPulseToMm[nIndex]) * pSG1Pointer->dPulseToMm[nIndex];//S4型
//	}
//
//	//确定速度规划的距离和终点速度
//	if (isZero(dSde - dLeftDist))//刚好停下
//	{
//		dVe = dVmin;		//速度规划末速度 等于 系统起跳速度
//
//		//重置前瞻指针
//		initLookAheadDealed(pSG1Pointer->pSChannel->ucChannelNum);
//		pSG1Pointer->pSChannel->bLookAheadNewBeginFlag = TRUE;//开机、新开始需要置TRUE
//		//g_bLookAheadNewBeginFlag = TRUE;
//	}
//	else if (dSde < dLeftDist)//距离足够减速
//	{
//		dVe = dVmin;		//速度规划末速度 等于 系统起跳速度
//
//		//计算终点位置
//		calculateLinearInterpolationPoint(&pSG1Pointer->SStartPoint, &pSG1Pointer->SStartVector, pSG1Pointer->SPlanData.dDistance - dLeftDist + dSde, &SEndPoint, AXIS_NUM_MAX);
//
//		pSG1Pointer->SEndPoint = SEndPoint;//终点改变
//
//		if (bWriteBackFlag == TRUE) //执行回写
//		{
//			//写回刀补缓冲区
//			//g_sSToolDealedNum += 1;// 刀补处理过的数据+1  插补读指针往回移动一个位置
//			//g_sLookAheadDealedNum += 1;
//			moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_BACK, pSG1Pointer->pSChannel->ucChannelNum);  //移回前瞻指针
//			pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSG1Pointer->pSChannel->ucChannelNum);//获取插补读指针
//
//			//写回数据  (终点不变 不用回写)
//			pSToolCompPointer->SStartPoint = SEndPoint;//回写的起点等于当前段的终点
//			pSToolCompPointer->dLength = pSG1Pointer->SPlanData.dCurLeftDist - dSde;
//
//			//重置前瞻指针
//			initLookAheadDealed(pSG1Pointer->pSChannel->ucChannelNum);
//			pSG1Pointer->pSChannel->bLookAheadNewBeginFlag = TRUE;//开机、新开始需要置TRUE
//			//g_bLookAheadNewBeginFlag = TRUE;  //开机、新开始需要置TRUE
//
//
//			//防止当前段的M30/M02，导致程序提前停止  (回写段有M30/M02，当前段的M30/M02需要删除)
//			setBits(pSG1Pointer->pSChannel->SPreProcess.unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
//
//		}
//
//		//新的剩余距离
//		pSG1Pointer->SPlanData.dCurLeftDist = dSde;//剩余距离 = 减速所需的长度
//		pSG1Pointer->SPlanData.dDistance -= dLeftDist - dSde;//当前线段长度： 减去暂停后剩下的距离
//		pSG1Pointer->SPlanData.dTargetDist = pSG1Pointer->SPlanData.dDistance;//目标距离
//
//	}
//	else//当前段不能停下
//	{
//		dVe = sqrt(dVs * dVs - 4 * dDec * dLeftDist / 3);  //最终速度不能减到起跳速度  重算实际最终速度
//
//	}
//
//	//暂停插补预处理结构体赋值
//	pSG1Pointer->SPlanData.dVs = pSG1Pointer->SPlanData.dVmax = dVs;  //起始速度 （最大速度）
//	pSG1Pointer->SPlanData.dVe = dVe;			//最终速度
//	pSG1Pointer->SPlanData.dDec = dDec;			//减速度
//	pSG1Pointer->SPlanData.ucAccVelPlanType = VEL_PLAN_S4_CURVE;		//速度曲线类型   1：梯形，2：3次S型，3：4次S型
//	pSG1Pointer->SPlanData.ucDecVelPlanType = VEL_PLAN_S4_CURVE;		//速度曲线类型   1：梯形，2：3次S型，3：4次S型
//	pSG1Pointer->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;		//平摊曲线类型，1：梯形，2：5次，3：比例系数
//	pSG1Pointer->SPlanData.unCurrentT = 0;  //时间
//
//	if (bWriteBackFlag == TRUE) //执行回写
//	{
//		pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSG1Pointer->pSChannel->ucChannelNum);
//		pSToolCompPointer->dVs = dVe;  //传递最终速度
//	}//预删除 因为合成运动的起始速度继承当前速度
//
//	return 1;   //预处理完成 返回1
//}

/*************************************************
  函数名称：modifyRotationAxisPoint(double *pdEndPoint,double *pdVector,int nArraySize,uint8 *ucMapArray,VAL_STRUCT_TYPE *ucValStatus)
  函数功能  修正旋转轴的终点
  输入参数：pdEndPoint		终点坐标
			pdVector		位移方向
			nArraySize		数组长度
			ucMapArray		映射
			ucValStatus		有效轴
  输出参数：pdEndPoint		终点坐标
			pdVector		位移方向
  返 回 值：-1 输入错误
			1 处理完成
  备    注：
**************************************************/
int8 modifyRotationAxisPoint(double* pdStartPoint, double* pdEndPoint, double* pdVector, int nArraySize,
                             uint8* ucMapArray, VAL_STRUCT_TYPE* ucValStatus, VAL_STRUCT_TYPE ucRelaVal)
{
    int i;
    double dRotationStartPoint;
    if (pdEndPoint == NULL || pdVector == NULL || ucMapArray == NULL)
    {
        //空指针
        return -1;
    }

    for (i = 0; i < nArraySize; i++)
    {
        if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX ||
            (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))) //映射为空  或不是有效轴 跳过
        {
            continue;
        }
        if (g_SSystemPara.SAxisPara[ucMapArray[i]].ucAxisType == ROTATION_AXIS) // 判断旋转轴
        {
            //计算旋转轴终点 和插补方向

            //更新旋转轴当前点坐标
            getSingleAxisRealTimePoint(&dRotationStartPoint, i, ucMapArray);         //获取当前坐标
            dRotationStartPoint = surplusDoubleToPositive(dRotationStartPoint, 360); //取余取正
            updateSingleAxisRealTimePoint(&dRotationStartPoint, i, ucMapArray);      //更新当前坐标

            //C轴为相对坐标 //朱稳中 2023.5.19
            if (getbit(ucRelaVal, i) == 1) // 相对坐标
            {
                //根据相对坐标计算
                pdEndPoint[i] = dRotationStartPoint + pdEndPoint[i] - pdStartPoint[i];
                if (pdVector != NULL)
                {
                    pdVector[i] = sgn(pdEndPoint[i] - dRotationStartPoint); //赋值方向
                }
                continue;
            }

            // 根据符号确定方向 //朱稳中 2023.5.19
            if (g_SSystemPara.ucRotationAxisInterpolationMode == ROTATION_INTERPOLATION_ACCORDING_DIRECTION)
            {
                //①获取方向
                pdVector[i] = sgn(pdEndPoint[i]);

                //②取余
                pdEndPoint[i] = surplusDoubleToPositive(pdEndPoint[i], 360);

                //③终点修正
                if ((pdEndPoint[i] - dRotationStartPoint) * pdVector[i] <
                    -EPSINON) //终点与方向不符	//陈敏聪	2023/10/23	判断条件<0改为<-EPSINON,防止起点和终点相近时错判
                {
                    pdEndPoint[i] += pdVector[i] * DEGREE_OF_CIRCLE;
                }
            }
            else
            { //距离最短确定方向
                double dForwardDist = surplusDoubleToPositive(pdEndPoint[i] - dRotationStartPoint, 360) //顺时针的距离
                    ,
                       dBackwardDist = surplusDoubleToPositive(dRotationStartPoint - pdEndPoint[i], 360); //逆时针的距离

                if (dForwardDist < dBackwardDist) //顺时针距离较短
                {
                    pdEndPoint[i] = dRotationStartPoint + dForwardDist; //终点=起点+位移
                    if (pdVector != NULL)
                    {
                        pdVector[i] = 1; //方向为正 表示顺时针
                    }
                }
                else //逆时针距离较短
                {
                    pdEndPoint[i] = dRotationStartPoint - dBackwardDist; //终点=起点-位移
                    if (pdVector != NULL)
                    {
                        pdVector[i] = -1; //方向为负 表示逆时针
                    }
                }
            }
        }
    }
    return 1;
}

/*************************************************
  函数名称：loadSG1DataFromInput(struct SG01* pSG1Pointer, struct SInput* pSInput)
  函数功能  从Input参数赋值到G1结构体
  输入参数：pSG1Pointer	结构体SG1指针
			pSInput			SInput结构体指针
			bReCalculate	是否重算起点和向量
  输出参数：
  返 回 值：-1	空指针
			1	处理完成
  备    注：
**************************************************/
int8 loadSG1DataFromInput(struct SG1* pSG1Pointer, struct SInput* pSInput)
{
    SPointDouble SRelaDist;

    const uint8* ucMapArray;                            //映射
    ucMapArray = pSG1Pointer->pSChannel->pMapAxisArray; //增加2026.1.22，用于读取C轴数据

    if (pSG1Pointer == NULL || pSInput == NULL)
        return -1;                                                         // 结构体为空
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (pSG1Pointer->pSChannel->bUseVirtualY == TRUE)
    {
        if (!isZero(pSInput->SStartVector.dArray[AXIS_Y])) //Y轴方向需要运动
        {
            if (VAL_VALID == (getbit(pSInput->ucValStatus, AXIS_Y)) &&
                VAL_VALID != (getbit(pSInput->ucValStatus, AXIS_X)))
            {
                setbit(pSInput->ucValStatus, AXIS_X); //设置X轴有效
            }
        }
    }
#endif
    pSG1Pointer->ucValStatus = pSInput->ucValStatus;                 //有效轴
    pSG1Pointer->SPlanData.cInterpDir = pSInput->cInterpDir;         //插补方向
    pSG1Pointer->SPlanData.dAcc = pSG1Pointer->dAcc = pSInput->dAcc; //加速度
    pSG1Pointer->SPlanData.dDec = pSG1Pointer->dDec = pSInput->dDec; //减速度
    pSG1Pointer->SPlanData.dCurLeftDist = pSInput->dCurLeftDist;     //剩余距离

    pSG1Pointer->SPlanData.dPrimalVe = pSInput->dPrimalVe;                     //末速度
    pSG1Pointer->SPlanData.dTargetDist = pSInput->dTargetDist;                 //目标距离
    pSG1Pointer->SPlanData.dVs = pSInput->dVs;                                 //初速度
    pSG1Pointer->SPlanData.ucAccVelPlanType = pSInput->ucAccVelPlanType;       //加速类型
    pSG1Pointer->SPlanData.ucDecVelPlanType = pSInput->ucDecVelPlanType;       //减速类型
    pSG1Pointer->SPlanData.ucDistReAverageType = pSInput->ucDistReAverageType; //平摊类型
    pSG1Pointer->SPlanData.ucTconstForTrapezoid =
        pSInput->ucTconstForTrapezoid; //三角形修调后匀速时间占总周期的百分比 （ % ） 洪冠鹏 2024/3/22

    pSG1Pointer->SPlanData.dDistance = pSInput->dDistance; //总距离
    pSG1Pointer->SPlanData.dPreDist = pSInput->dPreDist;   //已走距离

    pSG1Pointer->dG84Ratio = pSInput->dG84Ratio;   //G84倍率 2025.11.15
    pSG1Pointer->dFeedrate = pSInput->dFeedrate;   //进给速度
    pSG1Pointer->dFeedRatio = pSInput->dFeedRatio; //进给倍率
    pSG1Pointer->bUseSRunFeedRatio =
        (pSG1Pointer->dFeedRatio < 0 || isZero(pSG1Pointer->dFeedRatio)) ? TRUE : FALSE; //倍率为负或者0  使用SRun的倍率

    pSG1Pointer->SStartPoint = pSInput->SStartPoint;         //起点
    pSG1Pointer->SStartVector = pSInput->SStartVector;       //起点向量
    pSG1Pointer->SEndPoint = pSInput->SEndPoint;             //终点
    pSG1Pointer->fCylinderRadius = pSInput->fCylinderRadius; // 圆柱半径

    switch (pSInput->cDealType) //根据处理类型  处理数据
    {
    default:
    case DEALTYPE_RECAL:           //重算 不回写    用户输入
    case DEALTYPE_RECAL_WRITEBACK: //重算 回写   从缓冲区读取

#if CHANNEL_NUM_MAX > 1

        //判断相对坐标  修正终点（终点）  (G1 通道>=2 而且是长轴才判断)
        if (pSInput->ucRelaVal != VAL_ABS_POINT &&
            g_SSystemPara.ucInterpolationMode ==
                INTERPOLATION_LONGAXIS_MODE) //不完全是绝对坐标 ucRelaVal用位1表示对应轴是相对坐标，当其不等于0x0时表示有一个或以上个轴为相对坐标，此时需要修正
        {
            modifyRelaEndPoint((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SEndPoint,
                               pSG1Pointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSInput->ucRelaVal);
        }
#endif // CHANNEL_NUM_MAX >1

        if (getBits(pSG1Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
        {
        }
        else if (getBits(pSG1Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_CYLINDRICAL)
        { //圆柱插补
        }
        else
        {
            if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE)
            { //长轴模式
                //修正旋转轴坐标
                modifyRotationAxisPoint((double*)&pSInput->SStartPoint, (double*)&pSG1Pointer->SEndPoint,
                                        (double*)&pSG1Pointer->SStartVector, AXIS_NUM_MAX,
                                        pSG1Pointer->pSChannel->pMapAxisArray, &pSG1Pointer->ucValStatus,
                                        pSInput->ucRelaVal);
            }
            // else   /注释掉，不起作用
            // {//自适应和固定参数插补
            // 	//修正旋转轴坐标
            // 	modifyRotationAxisPoint((double*)&pSInput->SStartPoint, (double*)&pSG1Pointer->SEndPoint, NULL, AXIS_NUM_MAX, pSG1Pointer->pSChannel->pMapAxisArray, &pSG1Pointer->ucValStatus, pSInput->ucRelaVal);
            // }
            //获取当前坐标
            getRealTimePoint((double*)&pSG1Pointer->SStartPoint, AXIS_NUM_MAX, pSG1Pointer->pSChannel->pMapAxisArray,
                             &pSG1Pointer->ucValStatus);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
            //X方向或者Y方向运动时，以当前位置作为起点
            //起点为倾斜坐标系（X1-Y1-Z1）物理轴的实时点，终点为直角坐标系(X-Y-Z)缓冲区的值，因此要将起点换算至直角坐标系。
            if (pSG1Pointer->pSChannel->bUseVirtualY == TRUE)
            {
                getSingleAxisRealTimePoint(&pSG1Pointer->SStartPoint.dArray[AXIS_X], AXIS_X,
                                           pSG1Pointer->pSChannel->pMapAxisArray); //当前X1轴的位置
                getSingleAxisRealTimePoint(&pSG1Pointer->SStartPoint.dArray[AXIS_Y], AXIS_Y,
                                           pSG1Pointer->pSChannel->pMapAxisArray); //当前Y1轴的位置
                pSG1Pointer->SStartPoint.dArray[AXIS_X] -=
                    1.0 * pSG1Pointer->SStartPoint.dArray[AXIS_Y] *
                    pSG1Pointer->pSChannel->dSINAngleYY1; //将倾斜坐标系的点转化为直角坐标系的点
                pSG1Pointer->SStartPoint.dArray[AXIS_Y] =
                    pSG1Pointer->SStartPoint.dArray[AXIS_Y] * pSG1Pointer->pSChannel->dCOSAngleYY1;
            }
#endif
        }
        //将绝对坐标回写入缓冲区  并把相对坐标标记置0
        if (pSInput->cDealType == DEALTYPE_RECAL_WRITEBACK) //回写
        {
            int8 cIndex = 0;
            struct SToolCompensation* pWriteBackBuff;

            cIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSG1Pointer->pSChannel->ucChannelNum); //获取索引
            pWriteBackBuff =
                getSToolCompPointerByIndex(cIndex - 1, pSG1Pointer->pSChannel->ucChannelNum); //根据索引的前一个进行回写
            pWriteBackBuff->SEndPoint = pSG1Pointer->SEndPoint;                               //绝对坐标终点
            pWriteBackBuff->SStartPoint = pSG1Pointer->SStartPoint;                           //绝对坐标 起点
            pWriteBackBuff->ucRelaVal = 0x0;                                                  // 相对坐标标记置为0
            pWriteBackBuff->dPreDist = 0; //将缓冲区的已走的距离置为0
        }
        //极坐标插补
        if (getBits(pSG1Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
        {
            pSG1Pointer->ucValStatus |= 0x01 << g_SSystemPara.ucPolarInterpolationAxis[0];
            //subtractPoint((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->pSChannel->SRun.SCurrentOffset.dArray, (double*)&pSG1Pointer->SStartPoint,AXIS_NUM_MAX,&pSG1Pointer->ucValStatus);
            //subtractPoint((double*)&pSG1Pointer->SEndPoint, (double*)&pSG1Pointer->pSChannel->SRun.SCurrentOffset.dArray, (double*)&pSG1Pointer->SEndPoint, AXIS_NUM_MAX, &pSG1Pointer->ucValStatus);
            pSG1Pointer->ucValStatus |= 0x01 << g_SSystemPara.ucPolarInterpolationAxis[1];
        }
        if (!isZero(pSG1Pointer->dG84Ratio)) // G84指令下 //修改2025.11.15
        {
            // 计算运动距离和距离分量
            calcG84VectorAndDistance((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SEndPoint,
                                     &pSG1Pointer->SPlanData.dDistance, (double*)&pSG1Pointer->SStartVector,
                                     AXIS_NUM_MAX, pSG1Pointer->ucValStatus, pSG1Pointer->dG84Ratio);
            pSG1Pointer->SPlanData.dPreDist = 0; // 重算后已走距离为零
            if (ucMapArray[AXIS_C] >= AXIS_PHYSICS_NUM_MAX)
            {
                parseRunningError(pSG1Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
                return -1;
            }
            /*修改2026.1.22以c轴作为长轴，使用c轴数据并转换单位*/
            pSG1Pointer->SPlanData.dPrimalVe = g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVe;             //末速度°/ms
            pSG1Pointer->SPlanData.dVs = pSInput->dMinVs = g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVs; //初速度°/ms
            pSG1Pointer->dFeedrate = pSInput->dFeedrate * fabs(pSG1Pointer->dG84Ratio); //进给速度°/ms
            pSG1Pointer->SPlanData.dAcc = pSG1Pointer->dAcc =
                getSingleAxisAcc(AXIS_C, pSG1Pointer->SPlanData.ucAccVelPlanType, ucMapArray); //加速度°/s2
            pSG1Pointer->SPlanData.dDec = pSG1Pointer->dDec =
                getSingleAxisDec(AXIS_C, pSG1Pointer->SPlanData.ucDecVelPlanType, ucMapArray); //减速度°/s2
            break;
        }
        if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE) //长轴模式下
        {
            //计算长轴和距离分量
            calVectorAndDistance((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SEndPoint,
                                 &pSG1Pointer->SPlanData.dDistance, (double*)&pSG1Pointer->SStartVector, AXIS_NUM_MAX,
                                 pSG1Pointer->ucValStatus);
        }
        else //固定或自适应参数扩展线位移插补
        {
            //计算距离和向量
            calcG1VectorAndDistance(pSG1Pointer->SStartPoint, pSG1Pointer->SEndPoint, &pSG1Pointer->SStartVector,
                                    &pSG1Pointer->SStartVector, &pSG1Pointer->SPlanData.dDistance, AXIS_NUM_MAX,
                                    g_SChannel[pSG1Pointer->pSChannel->ucChannelNum].pMapAxisArray,
                                    &pSG1Pointer->ucValStatus, pSInput->ucRelaVal);
        }
        pSG1Pointer->SPlanData.dPreDist = 0; //重算后已走距离为零

        break;
    case DEALTYPE_NO_PROCESS: //不进行处理  //反走

        if (!isZero(pSG1Pointer->SPlanData.dPreDist) && pSInput->cInterpDir == INP_GO_AHEAD) //已经走距离不为0
        {
            int8 cIndex = 0;
            struct SToolCompensation* pWriteBackBuff; //回写指针

            cIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSG1Pointer->pSChannel->ucChannelNum); //获取索引
            pWriteBackBuff =
                getSToolCompPointerByIndex(cIndex - 1, pSG1Pointer->pSChannel->ucChannelNum); //根据索引的前一个进行回写
            pWriteBackBuff->dPreDist = 0; //将缓冲区的已走的距离置为0
        }
        if (!isZero(pSG1Pointer->dG84Ratio)) // G84指令下 //修改2025.11.15
        {
            // 计算运动距离和距离分量
            calcG84VectorAndDistance((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SEndPoint,
                                     &pSG1Pointer->SPlanData.dDistance, (double*)&pSG1Pointer->SStartVector,
                                     AXIS_NUM_MAX, pSG1Pointer->ucValStatus, pSG1Pointer->dG84Ratio);
            if (ucMapArray[AXIS_C] >= AXIS_PHYSICS_NUM_MAX)
            {
                parseRunningError(pSG1Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
                return -1;
            }
            /*修改2026.1.22以c轴作为长轴，使用c轴数据并转换单位*/
            pSG1Pointer->SPlanData.dPrimalVe = g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVe;             //末速度°/ms
            pSG1Pointer->SPlanData.dVs = pSInput->dMinVs = g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVs; //初速度°/ms
            pSG1Pointer->dFeedrate = pSInput->dFeedrate * fabs(pSG1Pointer->dG84Ratio); //进给速度°/ms
            pSG1Pointer->SPlanData.dAcc = pSG1Pointer->dAcc =
                getSingleAxisAcc(AXIS_C, pSG1Pointer->SPlanData.ucAccVelPlanType, ucMapArray); //加速度°/s2
            pSG1Pointer->SPlanData.dDec = pSG1Pointer->dDec =
                getSingleAxisDec(AXIS_C, pSG1Pointer->SPlanData.ucDecVelPlanType, ucMapArray); //减速度°/s2

            break;
        }
        if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE) //长轴模式下
        {
            //计算长轴和距离分量
            calVectorAndDistance((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SEndPoint,
                                 &pSG1Pointer->SPlanData.dDistance, (double*)&pSG1Pointer->SStartVector, AXIS_NUM_MAX,
                                 pSG1Pointer->ucValStatus);
        }
        else //固定或自适应参数扩展线位移插补
        {
            //计算距离和向量
            calcG1VectorAndDistance(pSG1Pointer->SStartPoint, pSG1Pointer->SEndPoint, &pSG1Pointer->SStartVector,
                                    &pSG1Pointer->SStartVector, &pSG1Pointer->SPlanData.dDistance, AXIS_NUM_MAX,
                                    g_SChannel[pSG1Pointer->pSChannel->ucChannelNum].pMapAxisArray,
                                    &pSG1Pointer->ucValStatus, pSInput->ucRelaVal);
        }
        break;
    }

    pSG1Pointer->SPlanData.dTargetDist = pSG1Pointer->SPlanData.dDistance; //目标距离=总长度
    pSG1Pointer->SPlanData.dCurLeftDist =
        (!isZero(pSG1Pointer->SPlanData.dPreDist) || !(pSInput->cInterpDir == INP_GO_BACK))
            ? //2022.12.8
            pSG1Pointer->SPlanData.dDistance - pSG1Pointer->SPlanData.dPreDist
            : 0; //计算剩余距离   正走或dPreDist不为0：总长度-已走距离   其他：为零

    if (isZero(pSG1Pointer->SPlanData.dTargetDist)) // 对0距离判断  2023.3.24 朱稳中
    {
        return 0;
    }

    return 1;
}
/*************************************************
  函数名称：loadSG1DataAfterPause(struct SG1* pSG1Pointer, SPointDouble SStartPoint, SPointDouble SEndPoint, struct SInput* pSInput)
  函数功能  暂停后加载参数赋值到G1结构体
  输入参数：
  输出参数：
  返 回 值：

  备    注：
**************************************************/
int8 loadSG1DataAfterPause(struct SG1* pSG1Pointer, SPointDouble SStartPoint, SPointDouble SEndPoint, bool bIsG0Move,
                           struct SInput* pSInputPointer)
{
    bool bAssignBySys = FALSE;                                       //是否根据缓冲区参数初始化恢复加工的G1
    const uint8* ucMapArray = pSG1Pointer->pSChannel->pMapAxisArray; //映射
    double dDistance;
    double dLength = 0;

    if (pSG1Pointer == NULL)
        return -1; // 结构体为空

    pSG1Pointer->SStartPoint = SStartPoint; //起点
    pSG1Pointer->SEndPoint = SEndPoint;     //终点
    pSG1Pointer->dCurrentSpeed = 0;

    //如果标志位为TRUE，表示暂停前执行的是单轴运动，需要根据系统参数赋值 陈敏聪 2024/1/11
    if (bIsG0Move == TRUE)
    {
        bAssignBySys = TRUE;
    }

    //计算合成位移
    for (int i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (ucMapArray[i] == AXIS_NULL || ((getbit(pSG1Pointer->pSChannel->SPreProcess.SUserData.ucValStatus, i)) == 0))
            continue; //映射为空或者不为有效轴时，跳过计算。  陈敏聪 洪冠鹏 2024/1/11
        dLength += pow(SStartPoint.dArray[i] - SEndPoint.dArray[i], 2);
    }
    pSG1Pointer->SPlanData.dTargetDist = pSG1Pointer->SPlanData.dDistance = pSG1Pointer->SPlanData.dCurLeftDist =
        sqrt(dLength);
    if (isZero(dLength)) //合成距离为0，直接返回
    {
        return 1;
    }

    //计算各轴的位移分量
    for (int i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (ucMapArray[i] == AXIS_NULL || ((getbit(pSG1Pointer->pSChannel->SPreProcess.SUserData.ucValStatus, i)) == 0))
            continue; //映射为空或者不为有效轴时，跳过计算该轴的位移分量。 陈敏聪 洪冠鹏 2024/1/11
        pSG1Pointer->SStartVector.dArray[i] =
            (SEndPoint.dArray[i] - SStartPoint.dArray[i]) / pSG1Pointer->SPlanData.dDistance;
    }

    pSG1Pointer->SPlanData.dPreDist = 0;                                   //重算后已走距离为零
    pSG1Pointer->SPlanData.dTargetDist = pSG1Pointer->SPlanData.dDistance; //目标距离=总长度
    pSG1Pointer->SPlanData.dCurLeftDist =
        pSG1Pointer->SPlanData.dDistance - pSG1Pointer->SPlanData.dPreDist; //计算剩余距离
    pSG1Pointer->SPlanData.ucDistReAverageType = g_SSystemPara.ucDistReAverageType;
    pSG1Pointer->SPlanData.dTc = g_SSystemPara.dInterpolationTime;

    if (bAssignBySys) //恢复加工前,缓冲区执行G0指令,则速度参数是获取系统的参数
    {
        //速度曲线类型，1：梯形，2：3次S型，3：4次S型
        pSG1Pointer->SPlanData.ucAccVelPlanType = g_SSystemPara.ucCommonAccVelPlanType;
        pSG1Pointer->SPlanData.ucDecVelPlanType = g_SSystemPara.ucCommonDecVelPlanType;

        //加、减速度
        pSG1Pointer->SPlanData.dAcc = pSG1Pointer->dAcc = getSysAcc(g_SSystemPara.ucCommonAccVelPlanType);
        pSG1Pointer->SPlanData.dDec = pSG1Pointer->dDec = getSysDec(g_SSystemPara.ucCommonDecVelPlanType);

        //初速度 末速度
        pSG1Pointer->SPlanData.dVs = g_SSystemPara.dSysMinVel;
        pSG1Pointer->SPlanData.dPrimalVe = g_SSystemPara.dSysMinVel;

        //倍率负数为无效值 ，运行过程中从SRun结构体获取倍率
        pSG1Pointer->dFeedRatio = -1;
        //进给速度为恢复加工最大速度：区分G98和G99对进给速度的处理  2024/1/11 洪冠鹏
        pSG1Pointer->dFeedrate = (getBits(pSG1Pointer->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN)
                                     ? g_SSystemPara.dRecoveryMaxVel / (60.0 * 1000)
                                     : g_SSystemPara.dRecoveryMaxVel / (60.0 * 1000) /
                                           fabs(pSG1Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed);
        //pSG1Pointer->dFeedrate = g_SSystemPara.dRecoveryMaxVel / (60.0 * 1000);
        //标志位：使用SRun的倍率
        pSG1Pointer->bUseSRunFeedRatio = TRUE;
    }
    else //恢复加工前，缓冲区已经有速度参数
    {
        //加、减速类型 1：梯形，2：3次S型，3：4次S型
        pSG1Pointer->SPlanData.ucAccVelPlanType = pSInputPointer->ucAccVelPlanType;
        pSG1Pointer->SPlanData.ucDecVelPlanType = pSInputPointer->ucDecVelPlanType;

        //加、减速度
        pSG1Pointer->SPlanData.dAcc = pSG1Pointer->dAcc = pSInputPointer->dAcc;
        pSG1Pointer->SPlanData.dDec = pSG1Pointer->dDec = pSInputPointer->dDec;

        //初、末速度
        pSG1Pointer->SPlanData.dVs = pSInputPointer->dVs;
        pSG1Pointer->SPlanData.dPrimalVe = pSInputPointer->dPrimalVe;

        pSG1Pointer->dFeedRatio = pSInputPointer->dFeedRatio; //倍率
        //进给速度为恢复加工最大速度和原本进给速度的最小值：区分G98和G99对进给速度的处理 2024/1/11 洪冠鹏
        pSG1Pointer->dFeedrate =
            (getBits(pSG1Pointer->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN)
                ? min(pSInputPointer->dFeedrate, g_SSystemPara.dRecoveryMaxVel / (60.0 * 1000))
                : min(pSInputPointer->dFeedrate, g_SSystemPara.dRecoveryMaxVel / (60.0 * 1000) /
                                                     fabs(pSG1Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed));
        //pSG1Pointer->dFeedrate = min(pSInputPointer->dFeedrate, g_SSystemPara.dRecoveryMaxVel / (60.0 * 1000));//进给速度
        //倍率为负或者0  使用SRun的倍率
        pSG1Pointer->bUseSRunFeedRatio =
            (pSG1Pointer->dFeedRatio < 0 || isZero(pSG1Pointer->dFeedRatio)) ? TRUE : FALSE;
    }

    pSG1Pointer->ucInpStatus = INP_ASSIGNMENT_START;
    return 1;
}

/*************************************************
  函数名称：motionLine(struct SG1* pSG1Pointer, bool bPauseOn, bool bRatioOn)
  函数功能  根据预处理结构体内的数据，根据进给速度和倍率执行速度规划和平摊，计算时间节点currentT的绝对坐标以及脉冲增量值
  输入参数：pSG1Pointer		结构体SG1指针
			bPauseOn		开启暂停
			bRatioOn		开启倍率
  输出参数：
  返 回 值：-1	输入错误
			0	插补完成
			1	插补中
  备    注：换向锁仅冻结手轮倍率响应；停稳后采纳最新倍率，暂停和限位仍走原流程。
**************************************************/
int8 motionLine(struct SG1* pSG1Pointer, bool bPauseOn, bool bRatioOn)
{
    int8 res;                               // 调用函数返回
    double dDeltaL;                         //相对起点的距离 (或弧长)
    SPointDouble STargetPoint;              //当前点的理论绝对坐标
    struct SInterpolationPre* pSPreprocess; //预处理指针
    const uint8* ucMapArray;                //映射
    struct _SRun* pSRun;                    //运行结构体
    struct SInput* pSUserData;              //数据来源
    //局部变量保存参数 防止规划失败
    double dVmax, dVs, dVe, dAcc, dDec;
    double dTargetSpeed;
    double dSpeedPercentage = 0;
    double dCurrentSpeed;
    VAL_STRUCT_TYPE ucValStatus; // 有效轴
    //软限位参数 2024/12/12 洪冠鹏
    double dResultantMaxDist;    //最大合成运动距离
    double dResultantLeftDist;   //剩余未走合成运动距离
    double dVmaxToLimitiedPoint; //反算的最大速度
    double dMinTDec;             //最小的T型减速度（系统参数）
    if (pSG1Pointer == NULL)     //空指针  不运行
    {
        return -1;
    }

    if (pSG1Pointer->ucInpStatus == INP_FINISHED) //如果插补结构体为完成状态，返回
    {
        return 0;
    }

    pSPreprocess = &pSG1Pointer->pSChannel->SPreProcess; //预处理指针
    ucMapArray = pSG1Pointer->pSChannel->pMapAxisArray;  //映射
    pSRun = &pSG1Pointer->pSChannel->SRun;               //运行结构体
    pSUserData =
        pSG1Pointer->bIsUserData
            ? &pSG1Pointer->SUserData
            : &pSPreprocess
                   ->SUserData; //数据来源，zxr 默认为后者(在预处理init置位false)，前者靠applyMotionParaAPI()接口提供

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* [HWSIM] G1 手轮入口分为两个阶段：
	 * 首次先经过轴占用和 loadSG1DataFromInput()，在 INP_ASSIGNMENT_START
	 * 进腥 executeHandwheelPathStep(FALSE) 绑定原直线几何；后续周期在下方提前 return。
	 * 编码器已由外层 interruptFuncAPI() 采样，本函数不重复读取手轮。
	 */
    if (!pSG1Pointer->bIsUserData && prepareHandwheelAutomaticMove(pSG1Pointer->pSChannel) < 0)
    {
        return -1;
    }
    /* [HWSIM] 普通程序段由距离执行器接管，G84 等旁路段继续原自动流程。
	 * bBound 也参与判断，是为了已接管期间即使模拟条件变化，也先进入统一
	 * 执行器检查/报错，不能带着旧距离绑定静默落回下面的时间规划器。
	 */
    if (!pSG1Pointer->bIsUserData &&
        (isHandwheelDistanceControlActive(pSG1Pointer->pSChannel) ||
         pSG1Pointer->pSChannel->SHandwheelSim.SPath.bBound) &&
        pSG1Pointer->ucInpStatus != INP_OCCUPATION && pSG1Pointer->ucInpStatus != INP_INITIATION)
    {
        /* [HWSIM] FALSE 选择直线视图。每拍顺序在 HandwheelSim.c 内：
		 * 视图/绑定检查 -> 预览合成进度 -> 直线求各轴点 -> 全轴输出 ->
		 * 提交进度和末速 -> 端点/暂停判断。这里直接返回，避免重复插补。
		 */
        return executeHandwheelPathStep(pSG1Pointer->pSChannel, FALSE);
    }
#endif

    if (bRatioOn && pSG1Pointer->bUseSRunFeedRatio == TRUE) //普通倍率开启
    { //插补正在进行的状态 且（前）当前正生效执行的倍率不等于实时的最新目标倍率
        if (pSG1Pointer->ucInpStatus == INP_RUNNING &&
            pSG1Pointer->dFeedRatio != pSRun->dCommonFeedRatio * pSRun->dHandWheelSpeedRatio)
        {
            if (pSG1Pointer->SPlanData.unCurrentT < pSG1Pointer->SPlanData.unT3 ||
                pSG1Pointer->SPlanData.unCurrentT > pSG1Pointer->SPlanData.unT5)
            {
                pSG1Pointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值，触发S/T曲线重规划
            }
        }
    }
    /*增加G84在G99作用无效，主轴无用编码器*/
    if (getBits(pSG1Pointer->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_R &&
        pSG1Pointer->ucInpStatus == INP_RUNNING && isZero((pSG1Pointer->dG84Ratio))) //G99每转进给，运行，非G84
    {
        dTargetSpeed = fabs(pSG1Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed) * pSG1Pointer->dFeedrate *
                       pSG1Pointer->dFeedRatio;
        dCurrentSpeed = pSRun->dCurrentFeedRate; //当前插补规划实际使用的进给速度

        if (isZero(dTargetSpeed)) // 零速度特判防除以零：主轴未转停转时的速度比率计算
        {
            if (isZero(dCurrentSpeed) ||
                isZero(g_SSystemPara.fG99SpindleSpeedDeviation - 1.0)) //编码器相对值为0  或者允许误差为100%
            {
                dSpeedPercentage = 1.0; //认为速度已匹配
            }
        }
        else
        {
            dSpeedPercentage = dCurrentSpeed / dTargetSpeed; // 计算当前速度相对于目标速度的达成比率
        }

        if ((fabs(dSpeedPercentage - 1.0 * SGN(dSpeedPercentage)) >
             g_SSystemPara.fG99SpindleSpeedDeviation)) //计算得到的进给速度发生改变
        {
            pSG1Pointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值
        }

        //if (!isZero(pSRun->dCurrentFeedRate - fabs(pSG1Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed) * pSG1Pointer->dFeedrate * 60 * 1000 * pSG1Pointer->dFeedRatio))//计算得到的进给速度发生改变
        //{
        //	pSG1Pointer->ucInpStatus = INP_ASSIGNMENT_START; //进入赋值
        //}
    }

    switch (pSG1Pointer->ucInpStatus)
    {
    case INP_FINISHED: //已经完成
        break;

    case INP_OCCUPATION: //在预处理阶段赋值
        // 释放上一条已执行完毕指令的轴占用，腾出物理电机轴资源
        releaseLastCommandAxis(pSPreprocess);
        ucValStatus =
            pSG1Pointer->bIsUserData ? pSG1Pointer->SUserData.ucValStatus : pSPreprocess->SUserData.ucValStatus;
#if (TURRET_AXIS_TEST == 1)
        // 【新架构-独立刀塔轴模式】：向轴管理器申请电机轴所有权，剔除当前通道已占用的刀塔轴（~ucOccupideTurretAxis）以避免自锁冲突
        if (axisManagerOccupyByChannel(ucValStatus & (~pSPreprocess->ucOccupideTurretAxis),
                                       pSG1Pointer->pSChannel->ucChannelNum, &pSPreprocess->ucOccupideValStatus) == 1)
        {
            //占用成功
            //G1有效轴（不能除去刀塔轴）
            pSG1Pointer->ucValStatus = ucValStatus; //在G1插补结构体中记录占用成功的结构体轴
            pSG1Pointer->ucInpStatus = INP_INITIATION;
        }
        else
        {
            break;
        }

#else
        // 【传统架构-刀补轴模式】：向轴管理器申请电机轴所有权，剔除已在刀补流程中占用的轴（~ucToolAxis）以避免冲突
        if (axisManagerOccupyByChannel(ucValStatus & (~pSPreprocess->ucToolAxis), pSG1Pointer->pSChannel->ucChannelNum,
                                       NULL) == 1)
        {
            // 记录本次实际新占用的指令轴状态（除去刀补轴，供后续指令释放用）
            pSPreprocess->ucLastValStatus = ucValStatus & (~pSPreprocess->ucToolAxis);
            // 记录 G1 完整的运动轴信息，推进状态至初始化
            pSG1Pointer->ucValStatus = ucValStatus;
            pSG1Pointer->ucInpStatus = INP_INITIATION;
        }
        else
        {
            break;
        }
#endif
    case INP_INITIATION: //初始化

        //根据标志位读取数据
        if (pSG1Pointer->bIsUserData) //数据来源  ①用户输入  修改插补状态后读取数据运行  ②从预处理结构体中读取运行数据
        {
            //读取输入参数（用户参数）
            res = loadSG1DataFromInput(pSG1Pointer, &pSG1Pointer->SUserData);
        }
        else
        {
            //读取预处理数据
            res = loadSG1DataFromInput(pSG1Pointer, &pSPreprocess->SUserData);
        }
        if (res < 0)
            return -1; //参数错误
        if (isZero(pSG1Pointer->SPlanData.dAcc) ||
            isZero(pSG1Pointer->SPlanData.dDec)) //参数检测，避免一些参数出现为0的情况
        {
            parseRunningError(pSG1Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
            return -1;
        }
        if (res == 0) //这行指令要走的移动距离为0，置为完成，相当于跳过这行指令执行下一行G代码
        {
            pSG1Pointer->ucInpStatus = INP_FINISHED;
            pSPreprocess->ucInpStatus = INP_FINISHED;
            break;
        }
        pSG1Pointer->dCurrentSpeed = pSRun->dCurrentSpeed; //新的一行 G 代码在初始化时，继承上一段的退场速度
        pSG1Pointer->ucInpStatus = INP_ASSIGNMENT_START;

    case INP_ASSIGNMENT_START: // 【核心阶段：插补启动规划】动力学参数装载、进给速度/倍率修调解算、初末速度衔接、速度规划及软限位预检

#if HANDWHEEL_SIM_DISTANCE_ENABLE
        /* [HWSIM] 首次接管点：轴占用、非零长度及原几何已经加载完成。
		 * executeHandwheelPathStep() 内部在未绑定时调用 prepareHandwheelPath()；
		 * 之后直接进腥路径自动规划与逐拍插补。
		 * 一旦合成路径接管直线，每拍直接输出对应位置。
		 */
        if (!pSG1Pointer->bIsUserData && isHandwheelDistanceControlActive(pSG1Pointer->pSChannel))
        {
            return executeHandwheelPathStep(pSG1Pointer->pSChannel, FALSE);
        }
#endif
        // 备份当前动力学规划参数（用于 preparePlanningAndDistReAvg 规划失败时的原子回滚，防止系统状态污染）
        dVmax = pSG1Pointer->SPlanData.dVmax;
        dVs = pSG1Pointer->SPlanData.dVs;
        dVe = pSG1Pointer->SPlanData.dVe;
        dAcc = pSG1Pointer->SPlanData.dAcc;
        dDec = pSG1Pointer->SPlanData.dDec;

        pSG1Pointer->SPlanData.dAcc = pSG1Pointer->dAcc;
        pSG1Pointer->SPlanData.dDec = pSG1Pointer->dDec;

        if (bRatioOn) // 开启倍率修调处理  zxr  G84复用G1运动。普通G1段：dG84Ratio保持为0，iszero(dG84Ratio)为TRUE，反之为G84
        {
            double dSpeedMax =
                (getBits(pSG1Pointer->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN)
                    ? pSG1Pointer
                          ->dFeedrate // G98 每分钟进给：基准最大速度直接等于指令进给速度 zxr 和G99最终都要换成mm/ms
                    : fabs(pSG1Pointer->pSChannel->SSpindleTest.dSpindleAvgSpeed) *
                          pSG1Pointer
                              ->dFeedrate; // G99 每转进给：根据主轴实测转速与进给量计算线速度 zxr 进给速度（mm/min）＝ 主轴转速（r/min）× F（mm/r）
            dSpeedMax = (isZero(pSG1Pointer->dG84Ratio))
                            ? dSpeedMax
                            : pSG1Pointer->dFeedrate; // G84 攻丝特殊处理：攻丝轴与主轴固定同步，避免与 G99 闭环冲突
            //继承进给速率
            pSG1Pointer->dFeedRatio = (pSG1Pointer->bUseSRunFeedRatio)
                                          ? pSRun->dCommonFeedRatio * pSRun->dHandWheelSpeedRatio
                                          : pSG1Pointer->dFeedRatio;

            if (pSG1Pointer->SPlanData.cInterpDir * pSG1Pointer->dFeedRatio > 0)
            {
                //根据倍率和主轴转速计算最大速度（当前进给速度保存到g_SRun）
                pSG1Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate = dSpeedMax * fabs(pSG1Pointer->dFeedRatio);
            }
            else // 插补方向与倍率符号反向（例如手轮反向摇动或倍率调整导致运动反向）
            {
                //可能的情况 ①倍率为0 ②当前速度为0 ③ 都不为0
                if (isZero(pSG1Pointer->dCurrentSpeed)) // 当前处于静止状态：允许直接切换运动方向并施加目标速度
                {
                    pSG1Pointer->SPlanData.cInterpDir = (pSG1Pointer->dFeedRatio < 0) ? INP_GO_BACK : INP_GO_AHEAD;
                    pSG1Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate = dSpeedMax * fabs(pSG1Pointer->dFeedRatio);
                }
                else // 仍在运动中：禁止突变换向，目标速度强制归零，先受控减速刹停到 0
                {
                    pSG1Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate = 0;
                }
            }
        }
        else // 未开启倍率：直接按原始指令进给速度执行
        {
            //最大速度赋值，不开启倍率时赋进给速度
            pSG1Pointer->SPlanData.dVmax = pSRun->dCurrentFeedRate = pSG1Pointer->dFeedrate;
        }

        if (pSG1Pointer->pSChannel->cHandWheelSimulaStatus != HANWHEELSIMULA_OFF &&
            (g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE)
#if HANDWHEEL_SIM_DISTANCE_ENABLE
            && !isHandwheelAutomaticMoveRequired(pSG1Pointer->pSChannel)
#endif
        )
        {
            pSG1Pointer->SPlanData.dVmax =
                isZero((pSG1Pointer->dG84Ratio))
                    ? min(pSG1Pointer->SPlanData.dVmax, g_SSystemPara.dSimulaVmax * fabs(pSG1Pointer->dFeedRatio))
                    : min(pSG1Pointer->SPlanData.dVmax,
                          g_SSystemPara.dSimulaVmax * DEGREE_OF_CIRCLE / 10 * fabs(pSG1Pointer->dFeedRatio));
            if (pSG1Pointer->pSChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_ON)
            {
                pSG1Pointer->SPlanData.dAcc = min(pSG1Pointer->SPlanData.dAcc, g_SSystemPara.dSimulaMaxAcc);
                pSG1Pointer->SPlanData.dDec = min(pSG1Pointer->SPlanData.dDec, g_SSystemPara.dSimulaMaxDec);
            }
        }
        // 系统与单轴动力学极限约束（防止超速）
        if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE && !pSG1Pointer->bIsUserData &&
            getBits(pSG1Pointer->pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE)
        {
            /*修改2026.1.22以c轴作为长轴，使用c轴数据并转换单位，取消限制速度单位不一致*/
            //pSG1Pointer->SPlanData.dVmax = min(pSG1Pointer->SPlanData.dVmax, pSPreprocess->SUserData.dCommonVMaxFromSingleAxis);
            pSG1Pointer->SPlanData.dVmax =
                isZero((pSG1Pointer->dG84Ratio))
                    ? min(pSG1Pointer->SPlanData.dVmax, pSPreprocess->SUserData.dCommonVMaxFromSingleAxis)
                    : pSG1Pointer->SPlanData.dVmax;
        }
        else
        {
            // 普通模式：受限于机床系统全局最大安全速度 dSysMaxVel，G84刚性攻丝保持不动
            pSG1Pointer->SPlanData.dVmax = isZero((pSG1Pointer->dG84Ratio))
                                               ? min(pSG1Pointer->SPlanData.dVmax, g_SSystemPara.dSysMaxVel)
                                               : pSG1Pointer->SPlanData.dVmax;
        }

        //计算初速度 分两种情况 ：G01刚开始(当前速度为0)  执行过程中进给速度改变(当前速度不为0)
        //①刚开始：初速度和最大速度取最小值。初速度比最大速度大的情况下，直接以最大速度运动
        //②非刚开始：一律继承当前速度
        pSG1Pointer->SPlanData.dVs =
            (pSG1Pointer->bIsUserData) ? pSG1Pointer->SUserData.dVs : pSPreprocess->SUserData.dMinVs; //获取初速度
        pSG1Pointer->SPlanData.dVs = (isZero(pSG1Pointer->dCurrentSpeed))
                                         ? min(pSG1Pointer->SPlanData.dVs, pSG1Pointer->SPlanData.dVmax)
                                         : pSG1Pointer->dCurrentSpeed;

        // --- 跨段软限位接力减速判断 (cSoftLimitedOn == 3) ---
        // 若前置程序段在运行中发现未来路径将触碰软限位（当时置 cSoftLimitedOn=2并在出段时标记为3），本段需承接紧急减速刹停
        if (pSRun->cSoftLimitedOn == 3)
        {
            for (int i = 0; i < AXIS_NUM_MAX; i++) // 遍历所有有效轴，找出触发限位的轴并以 T 型曲线紧急减速
            {
                if (VAL_VALID != getbit(pSPreprocess->SUserData.ucValStatus, i))
                    continue; // 无效轴跳过
                // zxr: 仅对实际触限轴检查方向，未触限轴的零分量不应报错
                if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode != SINGLEAXIS_LIMIT_POS &&
                    pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode != SINGLEAXIS_LIMIT_NEG)
                    continue;
                if (isZero(pSG1Pointer->SStartVector.dArray[i]))
                {
                    parseRunningError(pSG1Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM132, 9999);
                    return -1;
                }
                if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS) // 触发正向软限位
                {
                    dResultantMaxDist = (g_SSystemPara.MOTOR_WARN[i].pos_soft - pSG1Pointer->SStartPoint.dArray[i]) /
                                        fabs(pSG1Pointer->SStartVector.dArray[i]); // 计算合成位移上限
                }
                else if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode ==
                         SINGLEAXIS_LIMIT_NEG) // 触发负向软限位
                {
                    dResultantMaxDist = (pSG1Pointer->SStartPoint.dArray[i] - g_SSystemPara.MOTOR_WARN[i].neg_soft) /
                                        fabs(pSG1Pointer->SStartVector.dArray[i]); // 计算合成位移上限
                }
                else
                {
                    continue;
                }
                // 强制重设为 T 型加减速曲线，初速即为当前速度，只减不加
                pSG1Pointer->SPlanData.dVmax = pSG1Pointer->SPlanData.dVs;
                pSG1Pointer->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;
                pSG1Pointer->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;
                pSG1Pointer->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;
                pSG1Pointer->SPlanData.dAcc = min(getSysAcc(pSG1Pointer->SPlanData.ucAccVelPlanType),
                                                  g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxAcc /
                                                      fabs(pSG1Pointer->SStartVector.dArray[i]));
                pSG1Pointer->SPlanData.dDec = min(getSysDec(pSG1Pointer->SPlanData.ucDecVelPlanType),
                                                  g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec /
                                                      fabs(pSG1Pointer->SStartVector.dArray[i]));

                if (dResultantMaxDist <=
                    pSG1Pointer->SPlanData.dTargetDist) // 限位停止点就在当前线段内：本段内完全减速到停止
                {
                    pSG1Pointer->SPlanData.dVe = g_SSystemPara.dSysMinVel; // 末速度设为起跳最小速度
                    pSG1Pointer->SPlanData.dDec =
                        (pow(pSG1Pointer->SPlanData.dVs, 2) - pow(pSG1Pointer->SPlanData.dVe, 2)) /
                        (2.0 * pSG1Pointer->SPlanData.dCurLeftDist); // 按运动学 v^2 - v0^2 = 2as 重算减速度
                    pSRun->cSoftLimitedOn = 1;                       // 切换标志为 1（当前段内减速停）
                }
                else // 限位停止点仍超出当前线段：按最大减速度能力重算当前线段出段末速度
                {
                    //取 系统T型合成减速度 与 系统T型单轴减速度反算的合成减速度 的最小值作为合成运动的T型减速度
                    dMinTDec =
                        min(g_SSystemPara.dSysPlanTMaxDec, g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec /
                                                               fabs(pSG1Pointer->SStartVector.dArray[i]));
                    //1.重算末速度:以T型减速度重算当前线段的末速度（若当前线段是最后一段，则直接设定为系统参数的最小速度进行减速停止）
                    pSG1Pointer->SPlanData.dVe =
                        isZero(pSG1Pointer->SPlanData.dPrimalVe - g_SSystemPara.dSysMinVel)
                            ? g_SSystemPara.dSysMinVel
                            : sqrt(pow(g_SSystemPara.dSysMinVel, 2) + 2 * dMinTDec * dResultantMaxDist);
                }
                break;
            }
        }
        else // 未处于跨段软限位减速状态：使用前瞻流水线算好的原始末速度
        {
            //换回去前瞻的末速度
            pSG1Pointer->SPlanData.dVe = pSG1Pointer->SPlanData.dPrimalVe;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
            if (!pSG1Pointer->bIsUserData && isHandwheelAutomaticMoveRequired(pSG1Pointer->pSChannel))
            {
                pSG1Pointer->SPlanData.dVe = pSG1Pointer->SPlanData.dPrimalVe = 0.0;
            }
#endif
        }

        if (preparePlanningAndDistReAvg(&pSG1Pointer->SPlanData, TRUE) == -2) //速度规划+平摊
        {                                                                     //规划失败  换回原来的参数
            pSG1Pointer->SPlanData.dVmax = dVmax;
            pSG1Pointer->SPlanData.dVs = dVs;
            pSG1Pointer->SPlanData.dVe = dVe;
            pSG1Pointer->SPlanData.dAcc = dAcc;
            pSG1Pointer->SPlanData.dDec = dDec;
        }
        else
        {
            pSG1Pointer->SPlanData.unCurrentT = 0;
        }

        //软限位的判断
        pSRun->cSoftLimitedOn = softLimit(pSG1Pointer->pSChannel, pSG1Pointer->SStartPoint, pSG1Pointer->SEndPoint,
                                          pSG1Pointer->SEndPoint, pSG1Pointer->SEndPoint, AXIS_NULL);
        ;
        //判断后的处理
        switch (pSRun->cSoftLimitedOn)
        {
        case 1: // 【触发软限位 Case 1】限位点在当前线段内（本段内能够刹停）：
            for (int i = 0; i < AXIS_NUM_MAX; i++)
            {
                if (VAL_VALID != getbit(pSPreprocess->SUserData.ucValStatus, i))
                    continue;
                // zxr: 仅对实际触限轴检查方向，未触限轴的零分量不应报错
                if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode != SINGLEAXIS_LIMIT_POS &&
                    pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode != SINGLEAXIS_LIMIT_NEG)
                    continue;
                if (isZero(pSG1Pointer->SStartVector.dArray[i]))
                {
                    parseRunningError(pSG1Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM132, 9999);
                    return -1;
                }
                if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)
                {
                    dResultantMaxDist = (g_SSystemPara.MOTOR_WARN[i].pos_soft - pSG1Pointer->SStartPoint.dArray[i]) /
                                        fabs(pSG1Pointer->SStartVector.dArray[i]);
                }
                else if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)
                {
                    dResultantMaxDist = (pSG1Pointer->SStartPoint.dArray[i] - g_SSystemPara.MOTOR_WARN[i].neg_soft) /
                                        fabs(pSG1Pointer->SStartVector.dArray[i]);
                }
                else
                {
                    continue;
                }
                // 1. 截断目标距离与剩余距离至软限位边界
                pSG1Pointer->SPlanData.dTargetDist = fabs(dResultantMaxDist);
                pSG1Pointer->SPlanData.dCurLeftDist = fabs(dResultantMaxDist - pSG1Pointer->SPlanData.dPreDist);
                // 2. 重设重算末速度为系统最小速度
                pSG1Pointer->dReCalculateVe = g_SSystemPara.dSysMinVel;
                // 3. 推进状态到执行运动
                pSG1Pointer->ucInpStatus = INP_RUNNING;
                break;
            }
            break;
        case 2: // 【触发软限位 Case 2】限位点在当前线段之外（本段走完全程也不会撞，但后续段会撞，本段必须提前减速）：
            for (int i = 0; i < AXIS_NUM_MAX; i++)
            {
                if (VAL_VALID != getbit(pSPreprocess->SUserData.ucValStatus, i))
                    continue;
                // zxr: 仅对实际触限轴检查方向，未触限轴的零分量不应报错
                if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode != SINGLEAXIS_LIMIT_POS &&
                    pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode != SINGLEAXIS_LIMIT_NEG)
                    continue;
                if (isZero(pSG1Pointer->SStartVector.dArray[i]))
                {
                    parseRunningError(pSG1Pointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM132, 9999);
                    return -1;
                }
                if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)
                {
                    dResultantLeftDist = (g_SSystemPara.MOTOR_WARN[i].pos_soft - pSG1Pointer->SEndPoint.dArray[i]) /
                                         fabs(pSG1Pointer->SStartVector.dArray[i]);
                }
                else if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)
                {
                    dResultantLeftDist = (pSG1Pointer->SEndPoint.dArray[i] - g_SSystemPara.MOTOR_WARN[i].neg_soft) /
                                         fabs(pSG1Pointer->SStartVector.dArray[i]);
                }
                else
                {
                    continue;
                }

                // 取系统全局 T 型减速度与单轴反算减速度的保守值，避免过大减速度引起超程冲击
                dMinTDec = min(g_SSystemPara.dSysPlanTMaxDec, g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec /
                                                                  fabs(pSG1Pointer->SStartVector.dArray[i]));
                // 依据运动学方程 v_e = sqrt(v_min^2 + 2*a*s) 计算当前段末允许通过的最大末速度
                pSG1Pointer->dReCalculateVe =
                    sqrt(pow(g_SSystemPara.dSysMinVel, 2) + 2.0 * dMinTDec * fabs(dResultantLeftDist));
                //2.插补状态:运行
                pSG1Pointer->ucInpStatus = INP_RUNNING;
                break;
            }
            break;
        default: // 未触发软限位：按正常规划推进至执行运动阶段
            pSG1Pointer->ucInpStatus = INP_RUNNING;
            break;
        }

    case INP_RUNNING: //执行运动
        //判断是否为暂停状态
        if (bPauseOn) //开启暂停处理
        {
            double dPauseMinVe = 0;
            double dPauseDec = pSUserData->dDec;
            //系统TO_PAUSE状态  且当前不处于正在暂停状态
            if (pSG1Pointer->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE && pSG1Pointer->ucInpStatus != INP_PAUSING)
            {
                dPauseMinVe = isZero((pSG1Pointer->dG84Ratio)) ? pSUserData->dMinVe
                                                               : g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVe;
                dPauseDec = isZero((pSG1Pointer->dG84Ratio))
                                ? pSUserData->dDec
                                : getSingleAxisDec(AXIS_C, pSG1Pointer->SPlanData.ucDecVelPlanType, ucMapArray);
                prepareToPauseG1(pSG1Pointer, dPauseMinVe, dPauseDec, TRUE); //根据当前速度重新规划减速段并回写剩余轨迹
                pSG1Pointer->ucInpStatus = INP_PAUSING;                      //改变状态为正在暂停
            }
        }
    case INP_PAUSING: //暂停

        //运行时间增加 1 个插补周期（1ms）
        pSG1Pointer->SPlanData.unCurrentT += 1;

        //获取长轴（或空间轨迹）走到第 unCurrentT 周期时的累计位移 S(t)
        dDeltaL = nextstepPlanningAndDistReAvg(pSG1Pointer->SPlanData.unCurrentT, &pSG1Pointer->SPlanData);

        //计算本周期各物理轴的理论绝对坐标点
        calculateLinearInterpolationPoint((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SStartVector,
                                          dDeltaL, (double*)&STargetPoint, AXIS_NUM_MAX);

        //极坐标处理
        if (getBits(pSG1Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_POLAR)
        {
            double dCPoint;
            int8 cDir = 1;
            if (getBits(pSG1Pointer->unCommandType, CMD_POLAR_DIR_ADDR) == CMD_POLAR_DIR_NEGATIVE)
            {
                cDir = -1;
            }
            getSingleAxisRealTimePoint(&dCPoint, AXIS_C, ucMapArray);
            //极坐标转化为绝对坐标
            polarCoord2CommonCoord((double*)&STargetPoint, AXIS_NUM_MAX, g_SSystemPara.ucPolarInterpolationAxis[0],
                                   g_SSystemPara.ucPolarInterpolationAxis[1], cDir);
            //绝对坐标转化为机械坐标
            absPoint2RealTimePoint(pSG1Pointer->pSChannel->ucChannelNum, (double*)&STargetPoint, AXIS_NUM_MAX,
                                   pSG1Pointer->ucValStatus ^ (0x01 << AXIS_C));

            //旋转轴合法性判断 当前点和目标点距离过大
            STargetPoint.dArray[AXIS_C] =
                STargetPoint.dArray[AXIS_C] < 0 ? 360.0 + STargetPoint.dArray[AXIS_C] : STargetPoint.dArray[AXIS_C];
            STargetPoint.dArray[AXIS_C] = (int)(dCPoint / 360) * 360 + STargetPoint.dArray[AXIS_C];
            if (fabs(STargetPoint.dArray[AXIS_C] - dCPoint) >= 180)
            {
                STargetPoint.dArray[AXIS_C] =
                    round(dCPoint / 360) * 360 - (int)(dCPoint / 360) * 360 + STargetPoint.dArray[AXIS_C];
            }
            if (fabs(STargetPoint.dArray[AXIS_C] - dCPoint) >= 180)
            {
                STargetPoint.dArray[AXIS_C] -= sgn(STargetPoint.dArray[AXIS_C] - dCPoint) * 360;
            }
        }
        else if (getBits(pSG1Pointer->unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_CYLINDRICAL)
        { //圆柱插补处理 2023.3.30
            STargetPoint.dArray[AXIS_C] =
                cylinderCoord2Common(STargetPoint.dArray[AXIS_C], pSG1Pointer->fCylinderRadius);
        }
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
        if (pSG1Pointer->pSChannel->bUseVirtualY == TRUE)
        {
            //将直角坐标系下的坐标点转化为倾斜坐标系下的点
            STargetPoint.dArray[AXIS_X] += 1.0 * STargetPoint.dArray[AXIS_Y] * pSG1Pointer->pSChannel->dTANAngleYY1;
            STargetPoint.dArray[AXIS_Y] = STargetPoint.dArray[AXIS_Y] / pSG1Pointer->pSChannel->dCOSAngleYY1;
        }
#endif

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
        //利用理论坐标生成SRun结构体数据
        outPutAbsPointWithScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG1Pointer->dPulseToMm,
                                    &pSG1Pointer->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
        outPutAbsPointWithBilateralScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray,
                                             pSG1Pointer->dPulseToMm, &pSG1Pointer->ucValStatus);
#else
        //利用理论坐标生成SRun结构体数据
        outPutAbsPoint(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG1Pointer->dPulseToMm,
                       &pSG1Pointer->ucValStatus);
#endif
        //pSG1Pointer->pSChannel->ucChannelNum==1
        //计算当前1ms瞬时速度并同步写入 pSRun 看板，(上一周期剩余距离 - 本周期剩余距离) / 周期时间 dTc (1ms)
        pSRun->dCurrentSpeed = pSG1Pointer->dCurrentSpeed =
            fabs(pSG1Pointer->SPlanData.dCurLeftDist - (pSG1Pointer->SPlanData.dTargetDist - dDeltaL)) /
            pSG1Pointer->SPlanData.dTc;

        //计算 !理论! 未走距离
        pSG1Pointer->SPlanData.dCurLeftDist = pSG1Pointer->SPlanData.dTargetDist - dDeltaL;

        //判断软限位开始减速的时机：监测当前速度 >= 实时反算的最大速度（T型系统参数）//暂停后不进行判断
        if (pSRun->cSoftLimitedOn == 1 || pSRun->cSoftLimitedOn == 2)
        { //dVmaxToLimitiedPoint = sqrt(Ve^2 + 2 * a_max * dCurLeftDist)
            dVmaxToLimitiedPoint = sqrt(
                pSG1Pointer->dReCalculateVe * pSG1Pointer->dReCalculateVe +
                2 * g_SSystemPara.dSysPlanTMaxDec *
                    pSG1Pointer->SPlanData.dCurLeftDist); //根据末速度、T型加速度、剩余距离实时反算T型曲线下的最大速度
            if (pSG1Pointer->dCurrentSpeed >=
                dVmaxToLimitiedPoint) //一旦实时速度 dCurrentSpeed >= dVmaxToLimitiedPoint，说明已经到达最后刹车点
            {
                pSG1Pointer->ucInpStatus = INP_DECELERATE; //插补状态置为减速停
                if (pSRun->cSoftLimitedOn == 2)            //限位点在当前线段外
                {
                    pSRun->cSoftLimitedOn = 3; //软限位开关变量置为3,需要后续线段参与减速
                }
                if (!isZero(pSG1Pointer->SPlanData.dCurLeftDist)) //当前段剩余距离还未走完：进入减速停
                { //dDec = (dCurrentSpeed^2 - dReCalculateVe^2) / (2.0 * dCurLeftDist)
                    pSG1Pointer->SPlanData.dDec =
                        (pow(pSG1Pointer->dCurrentSpeed, 2) - pow(pSG1Pointer->dReCalculateVe, 2)) /
                        (2.0 * pSG1Pointer->SPlanData.dCurLeftDist); //重算T型减速度
                }
                else //当前段剩余距离已走完：进入减速停报警
                {
                    pSG1Pointer->SPlanData.dDec = 0;
                    break;
                }
            }
        }
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        if ((pSG1Pointer->bIsUserData || !isHandwheelAutomaticMoveRequired(pSG1Pointer->pSChannel) ||
             pSG1Pointer->SPlanData.unCurrentT >= pSG1Pointer->SPlanData.unT5) &&
            ((pSG1Pointer->SPlanData.dCurLeftDist <= EPSINON && pSG1Pointer->SPlanData.cInterpDir == INP_GO_AHEAD) ||
             (pSG1Pointer->SPlanData.dTargetDist - pSG1Pointer->SPlanData.dCurLeftDist <= EPSINON &&
              pSG1Pointer->SPlanData.cInterpDir == INP_GO_BACK &&
              pSG1Pointer->pSChannel->ucChannelStatus != SYS_PAUSE_TO_RUN)))
#else
        if ((pSG1Pointer->SPlanData.dCurLeftDist <= EPSINON && pSG1Pointer->SPlanData.cInterpDir == INP_GO_AHEAD) ||
            (pSG1Pointer->SPlanData.dTargetDist - pSG1Pointer->SPlanData.dCurLeftDist <= EPSINON &&
             pSG1Pointer->SPlanData.cInterpDir == INP_GO_BACK &&
             pSG1Pointer->pSChannel->ucChannelStatus != SYS_PAUSE_TO_RUN))
#endif
        {
            pSRun->dCurrentSpeed = pSG1Pointer->dCurrentSpeed;
            pSRun->dCurrentSpeed = isZero(pSG1Pointer->dG84Ratio) ? pSRun->dCurrentSpeed : 0;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
            if (!pSG1Pointer->bIsUserData && isHandwheelAutomaticMoveRequired(pSG1Pointer->pSChannel) &&
                pSG1Pointer->SPlanData.dVe == 0.0)
            {
                /* 规划已执行到零末速，末拍平均速度不是下一段起速。 */
                pSRun->dCurrentSpeed = pSG1Pointer->dCurrentSpeed = 0.0;
            }
#endif
            pSG1Pointer->ucInpStatus = INP_FINISHED;
            pSPreprocess->cInterpDir = pSG1Pointer->SPlanData.cInterpDir;
            pSPreprocess->ucInpStatus = isZero(pSG1Pointer->SPlanData.dDistance - pSG1Pointer->SPlanData.dTargetDist)
                                            ? INP_FINISHED
                                            : INP_PAUSING;
            return 0;
        }

        break;

    case INP_DECELERATE:
        if (pSRun->cSoftLimitedOn == 0) //未触发软限位的减速运动[速度模式]
        {
            if (pSG1Pointer->dCurrentSpeed <= g_SSystemPara.dSysMinVel) // 速度达到末速度范围内  立即停
            {
                pSG1Pointer->dCurrentSpeed = 0; //运动结束 当前速度置为0
                pSG1Pointer->ucInpStatus = INP_FINISHED;
                pSPreprocess->ucInpStatus = INP_FINISHED;
                break;
            }
            pSG1Pointer->SPlanData.dDec = g_SSystemPara.dSysPlanTMaxDec; //获取系统参数设置的T型减速度
        }

        if (pSG1Pointer->dCurrentSpeed >= pSG1Pointer->SPlanData.dDec * pSG1Pointer->SPlanData.dTc)
        {
            //计算长轴相对起点的距离 利用加速度计算  //2024/12/17 注释：减速距离公式错误，匀减速每个周期的位移增量为Vcur * 1 - 0.5 * Dec * 1 * 1。
            //dDeltaL = pSG1Pointer->SPlanData.dPreDist + pSG1Pointer->SPlanData.dTargetDist - pSG1Pointer->SPlanData.dCurLeftDist + pSG1Pointer->dCurrentSpeed - pSG1Pointer->SPlanData.dDec;
            dDeltaL = pSG1Pointer->SPlanData.dTargetDist - pSG1Pointer->SPlanData.dCurLeftDist +
                      pSG1Pointer->dCurrentSpeed - 0.5 * pSG1Pointer->SPlanData.dDec;
        }
        else
        {
            if (pSRun->cSoftLimitedOn == 0) //未触发软限位的减速运动[速度模式]：最后一个周期要求速度停下，位置无要求
            {
                dDeltaL = pSG1Pointer->SPlanData.dTargetDist - pSG1Pointer->SPlanData.dCurLeftDist;
            }
            else //触发软限位的减速运动[位置模式]：最后一个周期要求停到终点位置
            {
                dDeltaL = pSG1Pointer->SPlanData.dTargetDist;
                pSG1Pointer->SPlanData.dCurLeftDist = 0;
            }
        }

        //判断是否会超过终点
        if (dDeltaL > pSG1Pointer->SPlanData.dTargetDist)
        {
            dDeltaL = pSG1Pointer->SPlanData.dTargetDist;
        }
        //计算理论坐标点
        calculateLinearInterpolationPoint((double*)&pSG1Pointer->SStartPoint, (double*)&pSG1Pointer->SStartVector,
                                          dDeltaL, (double*)&STargetPoint, AXIS_NUM_MAX);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
        if (pSG1Pointer->pSChannel->bUseVirtualY == TRUE)
        {
            //将直角坐标系下的坐标点转化为倾斜坐标系下的点
            STargetPoint.dArray[AXIS_X] += 1.0 * STargetPoint.dArray[AXIS_Y] * pSG1Pointer->pSChannel->dTANAngleYY1;
            STargetPoint.dArray[AXIS_Y] = STargetPoint.dArray[AXIS_Y] / pSG1Pointer->pSChannel->dCOSAngleYY1;
        }
#endif
#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE //单表螺距误差+反向间隙补偿
        //利用理论坐标生成SRun结构体数据
        outPutAbsPointWithScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG1Pointer->dPulseToMm,
                                    &pSG1Pointer->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP //双表螺距误差补偿
        outPutAbsPointWithBilateralScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray,
                                             pSG1Pointer->dPulseToMm, &pSG1Pointer->ucValStatus);
#else                                      //无补偿
                                           //利用理论坐标生成SRun结构体数据
        outPutAbsPoint(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, pSG1Pointer->dPulseToMm,
                       &pSG1Pointer->ucValStatus);
#endif

        //计算当前速度 //2024/12/12 注释：该公式计算的是插补周期1ms的平均速度（移动量/1ms），而在减速过程计算相对起点的公式计算使用的是移动后的瞬时速度
        //pSRun->dCurrentSpeed = pSG1Pointer->dCurrentSpeed = fabs(pSG1Pointer->SPlanData.dCurLeftDist - (pSG1Pointer->SPlanData.dTargetDist - dDeltaL)) / pSG1Pointer->SPlanData.dTc;
        pSRun->dCurrentSpeed = pSG1Pointer->dCurrentSpeed =
            pSG1Pointer->dCurrentSpeed - pSG1Pointer->SPlanData.dDec * pSG1Pointer->SPlanData.dTc;

        //计算   !理论! 未走距离
        pSG1Pointer->SPlanData.dCurLeftDist = pSG1Pointer->SPlanData.dTargetDist - dDeltaL;

        if (pSG1Pointer->SPlanData.dCurLeftDist <= EPSINON) //运行结束条件
        {
            if (pSRun->cSoftLimitedOn == 1 &&
                pSG1Pointer->pSChannel->ucChannelStatus != SYS_RUN_TO_PAUSE) //触发软限位：限位点在当前线段内
            {
                for (int i = 0; i < AXIS_NUM_MAX; i++) //寻找出触发软限位的轴并报警
                {
                    if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS)
                    {
                        parseRunningError(pSG1Pointer->pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM41 + i,
                                          9999);
                        pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode =
                            SINGLEAXIS_MODE; //限位模式 切换为单轴模式
                        //setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数 //注释2025.12.11 在软限位报警后需要主轴停
                        break;
                    }
                    if (pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)
                    {
                        parseRunningError(pSG1Pointer->pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM47 + i,
                                          9999);
                        pSG1Pointer->pSChannel->SSingleAxisArray[i].ucRunMode =
                            SINGLEAXIS_MODE; //限位模式 切换为单轴模式
                        //setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数 //注释2025.12.11 在软限位报警后需要主轴停
                        break;
                    }
                }
                pSG1Pointer->dCurrentSpeed = 0;                     //运动结束：当前速度置0
                pSRun->cSoftLimitedOn = 0;                          //软限位开关关闭：开关变量置0
                pSG1Pointer->pSChannel->ucChannelStatus = SYS_IDLE; //停下后系统状态置为空闲
            }
            else if (pSRun->cSoftLimitedOn == 3 && pSG1Pointer->pSChannel->ucChannelStatus !=
                                                       SYS_RUN_TO_PAUSE) //触发软限位：限位点在当前线段外，仍需减速
            {
                pSRun->dCurrentSpeed = pSG1Pointer->dCurrentSpeed; //当前速度保存在运行结构体里
            }
            else //未触发软限位的减速停止
            {
                pSG1Pointer->dCurrentSpeed = 0; //运动结束 当前速度置为0
            }
            pSG1Pointer->ucInpStatus = INP_FINISHED; //状态改变为FINISHED
            pSPreprocess->ucInpStatus = INP_FINISHED;
            return 0;
        }
        break;
    case INP_STOP:
        pSG1Pointer->dCurrentSpeed = 0;          //瞬间把当前速度置为0（不走任何减速斜坡）
        pSG1Pointer->ucInpStatus = INP_FINISHED; //强行把插补器打到结束
        pSPreprocess->ucInpStatus = INP_FINISHED;
        break;
    }
    return 1;
}

/*************************************************
  函数名称：checkG1FirstAxisSoftLimit()
  函数功能：遍历各轴，检查G1哪根轴最先触发软限位
  输入参数：pSChannel - 通道结构体指针
			SStartPoint - 起点
			SMaxVal - 最大值点
			SMinVal - 最小值点
  返 回 值：-1 - 未触发限位
		   1 - 触发限位
  备    注：//2025.10.9
**************************************************/
int8 checkG1FirstAxisSoftLimit(struct SChannel* pSChannel, SPointDouble SStartPoint, SPointDouble SMaxVal,
                               SPointDouble SMinVal)
{
    double dNearestPosLimited = 9999.9; // 用于判断正限位最先触发软限位轴的距离变量
    double dNearestNegLimited = 9999.9; // 用于判断负限位最先触发软限位轴的距离变量
    int8 cPosLimitedAXIS = -1;          // 优先触发正限位轴号
    int8 cNegLimitedAXIS = -1;          // 优先触发负限位轴号
    uint8* pMapAxisArray = pSChannel->pMapAxisArray;

    for (int i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][i] == AXIS_NULL ||
            g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == ROTATION_AXIS) // 轴映射为空或旋转轴，跳过
            continue;

        if (VAL_VALID != getbit(pSChannel->SPreProcess.SUserData.ucValStatus, i)) // 无效轴，跳过
            continue;

        // 正限位检查
        if (g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].soft_pos == 1) // 开启正限位检测
        {
            if (SMaxVal.dArray[i] > g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft) // 最值点 > 正限位点
            {
                double dStartToMaxDistance = fabs(SMaxVal.dArray[i] - SStartPoint.dArray[i]); // 起点到最值点的距离
                double dStartToSoftLimitDistance = fabs(g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].pos_soft -
                                                        SStartPoint.dArray[i]); // 起点到软限位点的距离
                double dPosLimitedDistanceRatio =
                    dStartToSoftLimitDistance / dStartToMaxDistance; // 计算软限位点到起点的距离与最值点到起点的距离比值

                if (dNearestPosLimited - dPosLimitedDistanceRatio > EPSINON) // 寻找最先触发限位的轴
                {
                    cPosLimitedAXIS = i; // 保存轴号
                    dNearestPosLimited = dPosLimitedDistanceRatio;
                    continue;
                }
            }
        }

        // 负限位检查
        if (g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].soft_neg == 1) // 开启负限位检测
        {
            if (SMinVal.dArray[i] < g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft) // 最值点 < 负限位点
            {
                double dMinToStartDistance = fabs(SStartPoint.dArray[i] - SMinVal.dArray[i]); // 起点到最值点的距离
                double dStartToSoftLimitDistance =
                    fabs(SStartPoint.dArray[i] -
                         g_SSystemPara.MOTOR_WARN[pMapAxisArray[i]].neg_soft); // 起点到软限位点的距离
                double dNegLimitedDistanceRatio =
                    dStartToSoftLimitDistance / dMinToStartDistance; // 计算软限位点到起点的距离与最值点到起点的距离比值

                if (dNearestNegLimited - dNegLimitedDistanceRatio > EPSINON) // 寻找最先触发软限位的轴
                {
                    cNegLimitedAXIS = i; // 保存轴号
                    dNearestNegLimited = dNegLimitedDistanceRatio;
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
        else
        {
            pSChannel->SSingleAxisArray[cNegLimitedAXIS].ucRunMode = SINGLEAXIS_LIMIT_NEG; // 单轴运行模式设置为负限位
        }
        return 1;
    }

    return -1;
}
