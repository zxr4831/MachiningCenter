/*
M3 S1000
G51.2 P1 Q2 R0
G1 Z50 F1000
始终不能进入跟随
*/
/*
G0Z0
M3S50
G51.2 P1Q2R350
G1Z50F100
角度跟随上瞬间，但速度未跟随上
*/
//跟随方案1
//速度规划，若存在加速周期，则输出Vs+Acc；若存在匀速周期，则输出Vs；若存在减速周期，则输出Vs-Dec


#include "common.h"
#include <stdio.h>
#include "Spindle.h"
#include "toolAxis.h"
#include <math.h>
#include "CTrajPlanning.h"
#include "Channel.h"
#include "GeneralFunctions.h"
#include <stdlib.h>

#if STM32_MODE
#include "diwen.h"
#include "daoju.h"
#include "usart2.h"
#endif

SToolAxis g_SToolAxisInfo;
struct SEncoder g_SToolAxisEncoder;

#if TOOLAXIS_DEBUG_VS
extern FILE* pWriteFid;
#endif

int8 initToolAxis(struct SChannel* pSChannel, struct SSingleAxis* pSSingleAxisPointer)
{
	memset(&g_SToolAxisInfo, 0, sizeof(SToolAxis));
	initToolAxisPara();
	g_SToolAxisInfo.pSChannel = pSChannel;
	g_SToolAxisInfo.pSToolAxis = pSSingleAxisPointer;	//主轴的单轴结构体
}

//刀具轴信息初始化
void initToolAxisPara()
{	
	g_SToolAxisInfo.bFollowed = FALSE;

	g_SToolAxisInfo.dCommandRatio = 0;
	g_SToolAxisInfo.dCommandAngle = 0;

	g_SToolAxisInfo.dLastCommandRatio = 0;	//上一指令转速比
	g_SToolAxisInfo.dLastCommandAngle = 0;	//上一指令角度

	g_SToolAxisInfo.dCurTwistAngle = 0;			//扭转角度（扭转多边形用到）
	g_SToolAxisInfo.dLagAngle = 0;			//滞后角度（伺服驱动器存在响应时间导致）

	g_SToolAxisInfo.dSpindleSpeed = 0;
	g_SToolAxisInfo.dLastSpindleSpeed = 0;

	g_SToolAxisInfo.ucT = 0;
	g_SToolAxisInfo.dLagAngle = 0;

}

//1ms调用一次
int8 toolAxisMotion(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo, struct SEncoder* pSSpindleEncoder)
{
	struct SSingleAxis* pSToolAxis = pSToolAxisInfo->pSToolAxis;
	const uint8* ucMapArray = pSToolAxisInfo->pSChannel->pMapAxisArray;
	uint8 ucToolAxisNum = pSToolAxis->ucAxis;
	struct _SRun* pSRun = &pSToolAxisInfo->pSChannel->SRun;

	switch (pSToolAxis->ucRunMode)
	{
	//单轴模式
	case TOOLAXIS_SINGLE_MODE:
		pSToolAxisInfo->unTime = 0;
		break;
	//多边形加工指令开启	G51.2
	case TOOLAXIS_FOLLOW_MODE:
		pSToolAxisInfo->unTime++;
		//if (ENCODER_FOLLOW)
		if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 1)
			toolAxisFollowMotion(pSToolAxisInfo, pSSpindleInfo, pSSpindleEncoder);//跟随主轴编码器值
		else
			toolAxisFollowMotion2(pSToolAxisInfo, pSSpindleInfo, pSSpindleEncoder);//无算法
		break;
	//多边形加工指令结束	G50.2
	case TOOLAXIS_COMMONSTOP_MODE:
		pSToolAxisInfo->unTime++;
		toolAxisCommonStop(pSToolAxisInfo);
		break;
	//恒转速模式			M50、M51
	case TOOLAXIS_COMMON_MODE:
		toolAxisCommonMotion(pSToolAxisInfo);
		break;
	default:
		break;
	}


	//根据脉冲反算得到B轴实际坐标	
	double dCurrentPoint;
	getSingleAxisRealTimePoint(&dCurrentPoint, ucToolAxisNum, ucMapArray);

	if (pSToolAxis->ucRunMode != TOOLAXIS_SINGLE_MODE)
	{
		//根据刀具轴方向修正脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] *= TOOLAXIS_FOLLOWDIR;

		//更新刀具轴坐标
		dCurrentPoint += pSRun->sAxisDeltaPulse[ucToolAxisNum] * g_SSystemPara.SAxisPara[ucMapArray[ucToolAxisNum]].dPulseToMm;//脉冲数 × 每脉冲对应的角度
		dCurrentPoint = surplusDoubleData(dCurrentPoint + DEGREE_OF_CIRCLE, (int)DEGREE_OF_CIRCLE);//取余

		updateSingleAxisRealTimePoint(&dCurrentPoint, ucToolAxisNum, ucMapArray);

#if TOOLAXIS_DEBUG_VS
		//VS输出
		if (ENCODER_FOLLOW)
			outputVS();
		else
			outputVS_2();
#elif TOOLAXIS_DEBUG_32
		//串口输出
		output485();
#endif

	}

	return 1;

}

//跟随模式(跟随主轴编码器值)
void toolAxisFollowMotion(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo, struct SEncoder* pSSpindleEncoder)
{
	const uint8* pMapArray = pSToolAxisInfo->pSChannel->pMapAxisArray;
	
	struct SSingleAxis* pSToolAxis = pSToolAxisInfo->pSToolAxis;		//刀具轴单轴结构体
	struct SSingleAxis* pSSpindleAxis = pSSpindleInfo->pSSpindleAxis;	//主轴单轴结构体

	uint8 ucToolAxisNum = pSToolAxis->ucAxis;			//刀具轴轴名
	uint8 ucSpindleAxisNum = pSSpindleAxis->ucAxis;		//刀具轴轴名

	struct _SRun* pSRun = &pSToolAxisInfo->pSChannel->SRun;
	

	//读取主轴速度
	pSToolAxisInfo->dLastSpindleSpeed = pSToolAxisInfo->dSpindleSpeed;
	pSToolAxisInfo->dSpindleSpeed = pSSpindleAxis->dSpindleSpeed;

	//判断刀具轴相位差是否发生变化
	if (!isZero(pSToolAxisInfo->dCommandAngle - pSToolAxisInfo->dLastCommandAngle))
	{
		pSToolAxisInfo->dLastCommandAngle = pSToolAxisInfo->dCommandAngle;
		pSToolAxisInfo->bFollowed = FALSE;
	}
	//判断 指令跟随比 主轴目标转速 是否发生变化
	if (!isZero(pSToolAxisInfo->dCommandRatio - pSToolAxisInfo->dLastCommandRatio) ||
		!isZero(pSToolAxisInfo->dSpindleSpeed - pSToolAxisInfo->dLastSpindleSpeed))
	{	
		pSToolAxisInfo->dLastCommandRatio = pSToolAxisInfo->dCommandRatio;
		pSToolAxisInfo->dLastSpindleSpeed = pSToolAxisInfo->dSpindleSpeed;

		//刀具轴重新进行转速过渡
		pSToolAxis->ucInpStatus = INP_ASSIGNMENT_START;
		pSToolAxisInfo->bFollowed = FALSE;
	}
	
	//计算总扭转角度
	pSToolAxisInfo->dTotalTwistAngle = calcCurCurTwistAngle();

	//执行跟随模式对应步骤
	switch (pSToolAxis->ucInpStatus)
	{
	//转速过渡（速度规划）
	case INP_ASSIGNMENT_START:
	{
		//末速度：刀具轴目标转速
		double dRealSpindleSpeed;
		dRealSpindleSpeed = (int)((pSToolAxisInfo->dSpindleSpeed / 0.036) + 0.5) * 0.036;
		pSToolAxis->SPlanData.dVe = dRealSpindleSpeed * pSToolAxisInfo->dCommandRatio;

		//初速度：当前速度
		pSToolAxis->SPlanData.dVs = pSToolAxis->dCurrentSpeed;

		//加减速曲线类型
		pSToolAxis->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucAccVelPlanType;
		pSToolAxis->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucDecVelPlanType;

		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		pSToolAxis->SPlanData.dAcc = getSingleAxisAcc(ucToolAxisNum, pSToolAxis->SPlanData.ucAccVelPlanType, pMapArray);
		pSToolAxis->SPlanData.dDec = getSingleAxisDec(ucToolAxisNum, pSToolAxis->SPlanData.ucDecVelPlanType, pMapArray);

		//速度规划
		preparePlanningVelCombine(&pSToolAxis->SPlanData, g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dVeerDeltaVMax);

		//周期置0
		pSToolAxis->SPlanData.unCurrentT = 0;

		pSToolAxis->ucInpStatus = INP_STAGE_1;
	}
	//转速过渡（计算下一周期发送脉冲数）
	case INP_STAGE_1:
	{
		//计算刀具轴下一周期的速度
		double dSpeed;
		pSToolAxis->SPlanData.unCurrentT += 1;
		pSToolAxis->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSToolAxis->SPlanData.unCurrentT, &pSToolAxis->SPlanData);

		//计算刀具轴下一周期的脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);
		
		//判断刀具轴是否达到目标转速
		if (pSToolAxis->SPlanData.unCurrentT >= pSToolAxis->SPlanData.unT5)
		{
			//刀具轴达到目标转速，进入主轴转速检测
			pSToolAxis->ucInpStatus = INP_BLOCKED;
#if TOOLAXIS_DEBUG_32
			uart6_set("BLOCK\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "BLOCK\r");
#endif
		}
		break;

	}
	//主轴转速检测
	case INP_BLOCKED:
	{		
		//主轴已达到目标转速
		if (pSSpindleInfo->pSSpindleAxis->ucInpStatus == INP_FINISHED)
		{
			pSToolAxisInfo->ucT = 0;
			//进入测试阶段
			pSToolAxis->ucInpStatus = INP_TEST_1;
#if TOOLAXIS_DEBUG_32
			uart6_set("WAIT\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "WAIT\r");
#endif

		}
		//主轴未达到目标转速
		else
		{
			//刀具轴下一周期继续按当前速度输出脉冲
			double dSpeed;
			pSToolAxis->dCurrentSpeed = dSpeed = pSToolAxis->SPlanData.dVe;
			//根据速度换算脉冲
			pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

			break;
		}
	}
	//计算电机滞后角度_1：等待编码器达到目标转速
	case INP_TEST_1:
	{
		//刀具轴下一周期继续按当前速度输出脉冲
		double dSpeed;
		pSToolAxis->dCurrentSpeed = dSpeed = pSToolAxis->SPlanData.dVe;
		//根据速度换算脉冲
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

		pSToolAxisInfo->ucT++;
		//达到等待周期
		if (pSToolAxisInfo->ucT == WAIT_PERIOD)
		{
			pSToolAxisInfo->ucT = 0;
			pSToolAxisInfo->dLagAngle = 0;
			pSToolAxis->ucInpStatus = INP_TEST_2;
#if TOOLAXIS_DEBUG_32
			uart6_set("TEST\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "TEST\r");
#endif
		}
		break;
	}
	//计算电机滞后角度_2：计算编码器滞后角度
	case INP_TEST_2:
	{
		//当前周期开始编码器理论位置
		double dTargetPos;
		getSingleAxisRealTimePoint(&dTargetPos, ucToolAxisNum, pMapArray);
		dTargetPos -= pSRun->sLastDeltaPulse[ucToolAxisNum] * 0.036;//sLastDeltaPulse为当前周期理论脉冲增量

		//当前周期开始时编码器位置
		double dRealPos;
		dRealPos = getToolAxisEncoderPos(0);

		//当前周期编码器滞后值
		double dCurrEncoderLag;
		dCurrEncoderLag = surplusDoubleToPositive(dTargetPos - dRealPos, 360);

		//累计编码器滞后值
		pSToolAxisInfo->dLagAngle += dCurrEncoderLag;
		
		pSToolAxisInfo->ucT++;
		//达到采样周期
		if (pSToolAxisInfo->ucT == TEST_PERIOD)
		{
			//计算刀具轴当前转速下，由伺服响应响应时间带来的角度滞后
			pSToolAxisInfo->dLagAngle = surplusDoubleToPositive(pSToolAxisInfo->dLagAngle / TEST_PERIOD, 360.0);
			pSToolAxisInfo->ucT = 0;
			//进入角度过渡
			pSToolAxis->ucInpStatus = INP_STAGE_2;

#if TOOLAXIS_DEBUG_32
			uart6_set("ING\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "FOLLOWING\r");
#endif
		}
		//未达到计算周期，需要继续发送脉冲
		else
		{
			//刀具轴下一周期继续按当前速度输出脉冲
			double dSpeed;
			pSToolAxis->dCurrentSpeed = dSpeed = pSToolAxis->SPlanData.dVe;
			//根据速度换算脉冲
			pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);
			break;
		}
		
	}
	//角度过渡
	case INP_STAGE_2:
	{
		//1.读取主轴编码器上一周期脉冲增量 与 当前周期开始时的绝对数值
		uint16 usLastDelta;			//主轴编码器上一周期脉冲增量
		int64 lCurrEncoderAbsValue;	//主轴编码器当前周期开始时的绝对数值
		usLastDelta = pSSpindleEncoder->usDelta;
		lCurrEncoderAbsValue = pSSpindleEncoder->lEncoderAbsValue;

		//2.计算主轴编码器当前周期结束时的角度（此时当前周期内实际脉冲增量未知，这里假设当前周期内脉冲增量与上一周期脉冲增量相同）
		double dSpindleEncoderCurrPos;	//单位：°值域：[0 - 360)
		dSpindleEncoderCurrPos = getEncoderPosByValue(lCurrEncoderAbsValue + usLastDelta, pSSpindleEncoder->usEncoderCirclePulseNum);
		//dSpindleEncoderCurrPos = 360.0 * surplusDoubleToPositive(lCurrEncoderAbsValue + usLastDelta, pSSpindleEncoder->usEncoderCirclePulseNum) / 10000;

		//3.计算刀具轴当前周期结束时的理论角度
		double dToolAxisTargetPos;	//单位：°值域：[0-360)
		dToolAxisTargetPos = getToolAxisEncoderTargetPos(dSpindleEncoderCurrPos, FALSE);
		//dToolAxisTargetPos = surplusDoubleToPositive(dSpindleEncoderCurrPos * pSToolAxisInfo->dCommandRatio + pSToolAxisInfo->dPhase, 360.0);

		//4.读取刀具轴当前周期结束时的角度（上一周期已计算）
		double dToolAxisCurrPos;	//单位：°值域：[0-360)
		getSingleAxisRealTimePoint(&dToolAxisCurrPos, ucToolAxisNum, pMapArray);

		//5.计算刀具轴当前周期结束时的落后角度
		double dToolAxisCurrLag;	//单位：°值域：[0-360)
		dToolAxisCurrLag = surplusDoubleToPositive(dToolAxisTargetPos - dToolAxisCurrPos, 360.0);
		dToolAxisCurrLag = isZero(dToolAxisCurrLag - 360) ? 0 : dToolAxisCurrLag;

		//7.修正落后角度
		//当刀具轴跟随上主轴后，由于主轴转速波动，若主轴转速突减可能导致刀具轴超前主轴，即dToolAxisCurrLag值从1->359
		if (pSToolAxisInfo->bFollowed == TRUE)
			dToolAxisCurrLag = dToolAxisCurrLag > 180 ? dToolAxisCurrLag = dToolAxisCurrLag - 360 : dToolAxisCurrLag;

		//8.读取主轴下一周期的角度增量
		double dSpindle;	//单位：°值域：[0-360)
		dSpindle = g_SChannel[0].SRun.sAxisDeltaPulse[ucSpindleAxisNum] * 0.036;

		//9.计算刀具轴下一周期追赶距离
		double dLeftDistance;
		//dLeftDistance = dToolAxisCurrLag;
		dLeftDistance = dToolAxisCurrLag + dSpindle * pSToolAxisInfo->dCommandRatio;

		//10.对刀具轴剩余追赶距离进行速度规划
		//10.1速度规划参数赋值
		pSToolAxis->SPlanData.dVs = pSToolAxis->dCurrentSpeed;
		pSToolAxis->SPlanData.dVmax = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dVmax;		//刀具轴追赶最大速度(2000rpm)后续改为系统变量
		//pSToolAxis->SPlanData.dVe = pSToolAxisInfo->dSpindleSpeed * pSToolAxisInfo->dCommandRatio;	//  °/ms	目标转速
		pSToolAxis->SPlanData.dAcc = 0.036;// 加速度(动态速度规划*2)后续改为系统变量
		pSToolAxis->SPlanData.dDec = 0.036;// 加速度(动态速度规划*2)后续改为系统变量
	
		//跟随上后加减速*3
		if (pSToolAxisInfo->bFollowed == TRUE)
		{
			pSToolAxis->SPlanData.dAcc *= 3;
			pSToolAxis->SPlanData.dDec *= 3;
		}

		pSToolAxis->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;  //加速曲线类型
		pSToolAxis->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;  //减速曲线类型
		pSToolAxis->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;  //平摊类型
		pSToolAxis->SPlanData.dCurLeftDist = dLeftDistance;

		//10.速度规划，并去除残余距离
#if PARA_EDITABLE//参数可编辑
		if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 0)
		{
			//preparePlanningTNewAddForToolAxis_0(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 1)
		{
			preparePlanningTNewAddForToolAxis_1(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 2)
		{
			preparePlanningTNewAddForToolAxis_2(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 3)
		{
		}

#else
#if PLANNING_METHOD == 0
		preparePlanningTNewAddForToolAxis_0(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 1
		preparePlanningTNewAddForToolAxis_1(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 2
		preparePlanningTNewAddForToolAxis_2(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 3

#endif

#endif
		//11.计算刀具轴下一周期的角度增量
		double dDeltaL = 0;

#if PARA_EDITABLE//参数可编辑
		if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 0)
		{
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 1)
		{
			dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 2)
		{
			dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 3)
		{
		}

#else
#if PLANNING_METHOD == 0

#elif PLANNING_METHOD == 1
		dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 2
		dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 3
		dDeltaL = nextStepPlanningAndDistReAvgToolAxis_2(&pSToolAxis->SPlanData);

#endif

#endif

		//跟随判断
		if (pSToolAxisInfo->bFollowed == FALSE)
		{
			//提前进入跟随，防止主轴转速突降
			if (isEqualOrLessThanZero(pSToolAxis->SPlanData.dCurLeftDist - dDeltaL - 0.5))
			{
				pSToolAxisInfo->bFollowed = TRUE;
#if TOOLAXIS_DEBUG_32
				uart6_set("ED\r");
#elif TOOLAXIS_DEBUG_VS
				fprintf(pWriteFid, "FOLLOWED\r");
#endif

			}
				
		}

		//12.计算刀具轴下一周期的脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] = round(dDeltaL / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);


		//13.更新刀具轴当前速度
		pSToolAxis->dCurrentSpeed = pSRun->sAxisDeltaPulse[ucToolAxisNum] * 0.036;


	}
	default:
		break;
	}

}

//跟随模式(跟随主轴编码器值)
void toolAxisFollowMotion2(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo, struct SEncoder* pSSpindleEncoder)
{
	const uint8* pMapArray = pSToolAxisInfo->pSChannel->pMapAxisArray;

	struct SSingleAxis* pSToolAxis = pSToolAxisInfo->pSToolAxis;		//刀具轴单轴结构体
	struct SSingleAxis* pSSpindleAxis = pSSpindleInfo->pSSpindleAxis;	//主轴单轴结构体

	uint8 ucToolAxisNum = pSToolAxis->ucAxis;			//刀具轴轴名
	uint8 ucSpindleAxisNum = pSSpindleAxis->ucAxis;		//刀具轴轴名

	struct _SRun* pSRun = &pSToolAxisInfo->pSChannel->SRun;

	//读取主轴速度
	pSToolAxisInfo->dLastSpindleSpeed = pSToolAxisInfo->dSpindleSpeed;
	pSToolAxisInfo->dSpindleSpeed = pSSpindleAxis->dSpindleSpeed;

	//判断刀具轴相位差是否发生变化
	if (!isZero(pSToolAxisInfo->dCommandAngle - pSToolAxisInfo->dLastCommandAngle))
	{
		pSToolAxisInfo->dLastCommandAngle = pSToolAxisInfo->dCommandAngle;
		pSToolAxisInfo->bFollowed = FALSE;
	}
	//判断 指令跟随比 主轴目标转速 是否发生变化
	if (!isZero(pSToolAxisInfo->dCommandRatio - pSToolAxisInfo->dLastCommandRatio) ||
		!isZero(pSToolAxisInfo->dSpindleSpeed - pSToolAxisInfo->dLastSpindleSpeed))
	{
		pSToolAxisInfo->dLastCommandRatio = pSToolAxisInfo->dCommandRatio;
		pSToolAxisInfo->dLastSpindleSpeed = pSToolAxisInfo->dSpindleSpeed;

		//刀具轴重新进行转速过渡
		pSToolAxis->ucInpStatus = INP_ASSIGNMENT_START;
		pSToolAxisInfo->bFollowed = FALSE;
	}

	//计算扭转角度	扭转角度什么时候清零
	//pSToolAxisInfo->dCurTwistAngle;

	//执行跟随模式对应步骤
	switch (pSToolAxis->ucInpStatus)
	{
	//转速过渡（速度规划）
	case INP_ASSIGNMENT_START:
	{
		//末速度：刀具轴目标转速
		double dRealSpindleSpeed;
		dRealSpindleSpeed = (int)((pSToolAxisInfo->dSpindleSpeed / 0.036) + 0.5) * 0.036;
		pSToolAxis->SPlanData.dVe = dRealSpindleSpeed * pSToolAxisInfo->dCommandRatio;

		//初速度：当前速度
		pSToolAxis->SPlanData.dVs = pSToolAxis->dCurrentSpeed;

		//加减速曲线类型
		pSToolAxis->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucAccVelPlanType;
		pSToolAxis->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucDecVelPlanType;

		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		pSToolAxis->SPlanData.dAcc = getSingleAxisAcc(ucToolAxisNum, pSToolAxis->SPlanData.ucAccVelPlanType, pMapArray);
		pSToolAxis->SPlanData.dDec = getSingleAxisDec(ucToolAxisNum, pSToolAxis->SPlanData.ucDecVelPlanType, pMapArray);

		//速度规划
		preparePlanningVelCombine(&pSToolAxis->SPlanData, g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dVeerDeltaVMax);

		//周期置0
		pSToolAxis->SPlanData.unCurrentT = 0;

		pSToolAxis->ucInpStatus = INP_STAGE_1;
	}
	//转速过渡（计算下一周期发送脉冲数）
	case INP_STAGE_1:
	{
		//计算刀具轴下一周期的速度
		double dSpeed;
		pSToolAxis->SPlanData.unCurrentT += 1;
		pSToolAxis->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSToolAxis->SPlanData.unCurrentT, &pSToolAxis->SPlanData);

		//计算刀具轴下一周期的脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

		//判断刀具轴是否达到目标转速
		if (pSToolAxis->SPlanData.unCurrentT >= pSToolAxis->SPlanData.unT5)
		{
			//刀具轴达到目标转速，进入主轴转速检测
			pSToolAxis->ucInpStatus = INP_BLOCKED;
#if TOOLAXIS_DEBUG_32
			uart6_set("BLOCK\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "BLOCK\r");
#endif
		}
		break;

	}
	//主轴转速检测
	case INP_BLOCKED:
	{
		//主轴已达到目标转速
		if (pSSpindleInfo->pSSpindleAxis->ucInpStatus == INP_FINISHED)
		{
			pSToolAxisInfo->ucT = 0;
			//进入测试阶段
			pSToolAxis->ucInpStatus = INP_STAGE_2;
#if TOOLAXIS_DEBUG_32
			uart6_set("ING\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "FOLLOWING\r");
#endif
		}
		//主轴未达到目标转速
		else
		{
			//刀具轴下一周期继续按当前速度输出脉冲
			double dSpeed;
			pSToolAxis->dCurrentSpeed = dSpeed = pSToolAxis->SPlanData.dVe;
			//根据速度换算脉冲
			pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

			break;
		}
	}
	//角度过渡
	case INP_STAGE_2:
	{
		//1.主轴下一周期结束时的角度
		double dSpindleNextPos = 0;	//单位：°值域：[0 - 360)
		getSingleAxisRealTimePoint(&dSpindleNextPos, ucSpindleAxisNum, pMapArray);//此角度为下一周期结束时的角度

		//2.计算刀具轴下一周期结束时的理论角度
		double dToolAxisTargetPos;	//单位：°值域：[0-360)
		dToolAxisTargetPos = getToolAxisEncoderTargetPos(dSpindleNextPos, TRUE);//不考虑伺服系统响应时间

		//3.读取刀具轴当前周期结束时的角度（上一周期已计算）
		double dToolAxisCurrPos;	//单位：°值域：[0-360)
		getSingleAxisRealTimePoint(&dToolAxisCurrPos, ucToolAxisNum, pMapArray);

		//4.计算刀具轴下一周期总运动的角度
		double dLeftDistance;	//单位：°值域：[0-360)
		dLeftDistance = surplusDoubleToPositive(dToolAxisTargetPos - dToolAxisCurrPos, 360.0);
		dLeftDistance = isZero(dLeftDistance - 360) ? 0 : dLeftDistance;

		//5.修正刀具轴下一周期总运动的角度
		//当刀具轴跟随上主轴后，由于主轴转速波动，若主轴转速突减可能导致刀具轴超前主轴，即dToolAxisCurrLag值从1->359
		if (pSToolAxisInfo->bFollowed == TRUE)
			dLeftDistance = dLeftDistance > 180 ? dLeftDistance = dLeftDistance - 360 : dLeftDistance;

		//10.对刀具轴剩余追赶距离进行速度规划
		//10.1速度规划参数赋值
		pSToolAxis->SPlanData.dVs = pSToolAxis->dCurrentSpeed;
		pSToolAxis->SPlanData.dVmax = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dVmax;		//刀具轴追赶最大速度(2000rpm)后续改为系统变量
		//pSToolAxis->SPlanData.dVe = pSToolAxisInfo->dSpindleSpeed * pSToolAxisInfo->dCommandRatio;	//  °/ms	目标转速
		pSToolAxis->SPlanData.dAcc = 0.036;// 加速度(动态速度规划*2)后续改为系统变量
		pSToolAxis->SPlanData.dDec = 0.036;// 加速度(动态速度规划*2)后续改为系统变量

		//跟随上后加减速*3
		if (pSToolAxisInfo->bFollowed == TRUE)
		{
			pSToolAxis->SPlanData.dAcc *= 3;
			pSToolAxis->SPlanData.dDec *= 3;
		}

		pSToolAxis->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;  //加速曲线类型
		pSToolAxis->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;  //减速曲线类型
		pSToolAxis->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;  //平摊类型
		pSToolAxis->SPlanData.dCurLeftDist = dLeftDistance;

		//10.速度规划，并去除残余距离
#if PARA_EDITABLE//参数可编辑
		if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 0)
		{
			//preparePlanningTNewAddForToolAxis_0(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 1)
		{
			preparePlanningTNewAddForToolAxis_1(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 2)
		{
			preparePlanningTNewAddForToolAxis_2(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 3)
		{
		}

#else
#if PLANNING_METHOD == 0
		preparePlanningTNewAddForToolAxis_0(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 1
		preparePlanningTNewAddForToolAxis_1(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 2
		preparePlanningTNewAddForToolAxis_2(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 3

#endif

#endif
		//11.计算刀具轴下一周期的角度增量
		double dDeltaL = 0;

#if PARA_EDITABLE//参数可编辑
		if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 0)
		{
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 1)
		{
			dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 2)
		{
			dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
		}
		else if (g_SSystemPara.sGcode_MultiLineHandleMaxNum == 3)
		{
		}

#else
#if PLANNING_METHOD == 0

#elif PLANNING_METHOD == 1
		dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 2
		dDeltaL = nextStepPlanningAndDistReAvgToolAxis_1(&pSToolAxis->SPlanData);
#elif PLANNING_METHOD == 3
		dDeltaL = nextStepPlanningAndDistReAvgToolAxis_2(&pSToolAxis->SPlanData);

#endif

#endif

		//跟随判断
		if (pSToolAxisInfo->bFollowed == FALSE)
		{
			//提前进入跟随，防止主轴转速突降
			if (isEqualOrLessThanZero(pSToolAxis->SPlanData.dCurLeftDist - dDeltaL - 0.5))
			{
				pSToolAxisInfo->bFollowed = TRUE;
#if TOOLAXIS_DEBUG_32
				uart6_set("ED\r");
#elif TOOLAXIS_DEBUG_VS
				fprintf(pWriteFid, "FOLLOWED\r");
#endif

			}

		}

		//12.计算刀具轴下一周期的脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] = round(dDeltaL / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);


		//13.更新刀具轴当前速度
		pSToolAxis->dCurrentSpeed = pSRun->sAxisDeltaPulse[ucToolAxisNum] * 0.036;


	}
	default:
		break;
	}

}

//跟随模式(直接启动) 
void toolAxisFollowMotion3(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo, struct SEncoder* pSSpindleEncoder)
{
	const uint8* pMapArray = pSToolAxisInfo->pSChannel->pMapAxisArray;

	struct SSingleAxis* pSToolAxis = pSToolAxisInfo->pSToolAxis;		//刀具轴单轴结构体
	struct SSingleAxis* pSSpindleAxis = pSSpindleInfo->pSSpindleAxis;	//主轴单轴结构体

	uint8 ucToolAxisNum = pSToolAxis->ucAxis;			//刀具轴轴名
	uint8 ucSpindleAxisNum = pSSpindleAxis->ucAxis;		//刀具轴轴名

	struct _SRun* pSRun = &pSToolAxisInfo->pSChannel->SRun;

	//读取主轴速度
	pSToolAxisInfo->dLastSpindleSpeed = pSToolAxisInfo->dSpindleSpeed;
	pSToolAxisInfo->dSpindleSpeed = pSSpindleAxis->dSpindleSpeed;

	//判断刀具轴相位差是否发生变化
	if (!isZero(pSToolAxisInfo->dCommandAngle - pSToolAxisInfo->dLastCommandAngle))
	{
		pSToolAxisInfo->dLastCommandAngle = pSToolAxisInfo->dCommandAngle;
		pSToolAxisInfo->bFollowed = FALSE;
	}
	//判断 指令跟随比 主轴目标转速 是否发生变化
	if (!isZero(pSToolAxisInfo->dCommandRatio - pSToolAxisInfo->dLastCommandRatio) ||
		!isZero(pSToolAxisInfo->dSpindleSpeed - pSToolAxisInfo->dLastSpindleSpeed))
	{
		pSToolAxisInfo->dLastCommandRatio = pSToolAxisInfo->dCommandRatio;
		pSToolAxisInfo->dLastSpindleSpeed = pSToolAxisInfo->dSpindleSpeed;

		//刀具轴重新进行转速过渡
		pSToolAxis->ucInpStatus = INP_ASSIGNMENT_START;
		pSToolAxisInfo->bFollowed = FALSE;
	}

	//计算扭转角度	扭转角度什么时候清零
	//pSToolAxisInfo->dTwistAngle;

	//执行跟随模式对应步骤
	switch (pSToolAxis->ucInpStatus)
	{
	//转速过渡（速度规划）
	case INP_ASSIGNMENT_START:
	{
		//末速度：刀具轴目标转速
		double dRealSpindleSpeed;
		dRealSpindleSpeed = (int)((pSToolAxisInfo->dSpindleSpeed / 0.036) + 0.5) * 0.036;
		pSToolAxis->SPlanData.dVe = dRealSpindleSpeed * pSToolAxisInfo->dCommandRatio;
		pSToolAxis->SPlanData.dVe = 8.4;//测试取值1400rpm

		//初速度：当前速度
		pSToolAxis->SPlanData.dVs = pSToolAxis->dCurrentSpeed;

		//加减速曲线类型
		pSToolAxis->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucAccVelPlanType;
		pSToolAxis->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucDecVelPlanType;

		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		pSToolAxis->SPlanData.dAcc = getSingleAxisAcc(ucToolAxisNum, pSToolAxis->SPlanData.ucAccVelPlanType, pMapArray);
		pSToolAxis->SPlanData.dDec = getSingleAxisDec(ucToolAxisNum, pSToolAxis->SPlanData.ucDecVelPlanType, pMapArray);

		//速度规划
		preparePlanningVelCombine(&pSToolAxis->SPlanData, g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dVeerDeltaVMax);

		//周期置0
		pSToolAxis->SPlanData.unCurrentT = 0;

		pSToolAxis->ucInpStatus = INP_STAGE_1;
	}
	//转速过渡（计算下一周期发送脉冲数）
	case INP_STAGE_1:
	{
		//计算刀具轴下一周期的速度
		double dSpeed;
		pSToolAxis->SPlanData.unCurrentT += 1;
		pSToolAxis->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSToolAxis->SPlanData.unCurrentT, &pSToolAxis->SPlanData);

		//计算刀具轴下一周期的脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

		//判断刀具轴是否达到目标转速
		if (pSToolAxis->SPlanData.unCurrentT >= pSToolAxis->SPlanData.unT5)
		{
			//刀具轴达到目标转速，进入主轴转速检测
			pSToolAxis->ucInpStatus = INP_STAGE_2;
#if TOOLAXIS_DEBUG_32
			uart6_set("ED\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "FOLLOWED\r");
#endif
		}
		break;
	}
	//跟随
	case INP_STAGE_2:
	{
		//刀具轴下一周期继续按当前速度输出脉冲
		double dSpeed;
		pSToolAxis->dCurrentSpeed = dSpeed = pSToolAxis->SPlanData.dVe;
		//根据速度换算脉冲
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

		break;
	}
	default:
		break;
	}

}

//减速停模式
int8 toolAxisCommonStop(SToolAxis* pSToolAxisPointer)
{
	double dSpeed;//速度输出
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体
	uint8 ucAxis;//当前轴号
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射

	if (pSToolAxisPointer == NULL)//空指针
	{
		return -1;
	}

	//获取单轴结构体
	pSSingleAxisPointer = pSToolAxisPointer->pSToolAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSToolAxisPointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSToolAxisPointer->pSChannel->SRun;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return -3;
	}

	switch (pSSingleAxisPointer->ucInpStatus)
	{
	//初始化
	case INP_INITIATION:
	{
		//目标末速度
		pSSingleAxisPointer->SPlanData.dPrimalVe = 0;
		//起跳速度  与当前速度取最小值
		//pSSingleAxisPointer->SPlanData.dVe = sgn(pSSingleAxisPointer->dCurrentSpeed) * min(fabs(pSSingleAxisPointer->dCurrentSpeed), g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe);
		pSSingleAxisPointer->SPlanData.dVe = 0;
		//起始速度(r/ms)
		pSSingleAxisPointer->SPlanData.dVs = pSSingleAxisPointer->dCurrentSpeed;

		//加减速曲线
		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
		
		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
		pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);

		//速度规划
		preparePlanningVelCombine(&pSSingleAxisPointer->SPlanData,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVeerDeltaVMax);

		//初始化变量
		pSSingleAxisPointer->SPlanData.unCurrentT = 0;//时间置0
		pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//跳转到输出速度
	}
	//输出速度
	case INP_RUNNING:
	{
		//时间增加
		pSSingleAxisPointer->SPlanData.unCurrentT += 1;
		//获得当前速度 (r/ms)
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);
		//计算脉冲增量
		pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);

		if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)//加速/减速完成?
		{
			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成
		}
		break;
	}
	//主轴停下
	case INP_FINISHED:
	{	
		pSSingleAxisPointer->ucRunMode = TOOLAXIS_SINGLE_MODE;
		initToolAxisPara();
		break;
	}
	}
	return 1;

}

//恒转速
void toolAxisCommonMotion(SToolAxis* pSToolAxisInfo)
{
	//刀具轴单轴结构体
	struct SSingleAxis* pSToolAxis = pSToolAxisInfo->pSToolAxis;	

	const uint8* pMapArray = pSToolAxisInfo->pSChannel->pMapAxisArray;
	uint8 ucToolAxisNum = pSToolAxis->ucAxis;			//刀具轴轴名
	struct _SRun* pSRun = &pSToolAxisInfo->pSChannel->SRun;

	switch (pSToolAxis->ucInpStatus)
	{
	//速度规划
	case INP_ASSIGNMENT_START:
	{
		//末速度：刀具轴目标转速
		double dRealSpindleSpeed;
		dRealSpindleSpeed = (int)((pSToolAxisInfo->dSpindleSpeed / 0.036) + 0.5) * 0.036;
		pSToolAxis->SPlanData.dVe = dRealSpindleSpeed * pSToolAxisInfo->dCommandRatio;

		//初速度：当前速度
		pSToolAxis->SPlanData.dVs = pSToolAxis->dCurrentSpeed;

		//加减速曲线类型
		pSToolAxis->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucAccVelPlanType;
		pSToolAxis->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].ucDecVelPlanType;

		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		pSToolAxis->SPlanData.dAcc = getSingleAxisAcc(ucToolAxisNum, pSToolAxis->SPlanData.ucAccVelPlanType, pMapArray);
		pSToolAxis->SPlanData.dDec = getSingleAxisDec(ucToolAxisNum, pSToolAxis->SPlanData.ucDecVelPlanType, pMapArray);

		//速度规划
		preparePlanningVelCombine(&pSToolAxis->SPlanData,g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dVeerDeltaVMax);

		//周期置0
		pSToolAxis->SPlanData.unCurrentT = 0;

		pSToolAxis->ucInpStatus = INP_STAGE_1;
	}
	//加速阶段
	case INP_STAGE_1:
	{
		//计算刀具轴下一周期的速度
		double dSpeed;
		pSToolAxis->SPlanData.unCurrentT += 1;
		pSToolAxis->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSToolAxis->SPlanData.unCurrentT, &pSToolAxis->SPlanData);

		//计算刀具轴下一周期的脉冲数
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

		//判断刀具轴是否达到目标转速
		if (pSToolAxis->SPlanData.unCurrentT >= pSToolAxis->SPlanData.unT5)
		{
			//刀具轴达到目标转速，进入主轴转速检测
			pSToolAxis->ucInpStatus = INP_BLOCKED;
#if TOOLAXIS_DEBUG_32
			uart6_set("BLOCK\r");
#elif TOOLAXIS_DEBUG_VS
			fprintf(pWriteFid, "BLOCK\r");
#endif
		}
		break;

	}
	//匀速阶段
	case INP_BLOCKED:
	{
		//刀具轴下一周期继续按当前速度输出脉冲
		double dSpeed;
		pSToolAxis->dCurrentSpeed = dSpeed = pSToolAxis->SPlanData.dVe;
		//根据速度换算脉冲
		pSRun->sAxisDeltaPulse[ucToolAxisNum] += round(dSpeed / g_SSystemPara.SAxisPara[pMapArray[ucToolAxisNum]].dPulseToMm);

		break;
	}
	}
}

//计算总扭转角度获取指令中的扭转角度
double calcCurCurTwistAngle()
{
	double dTotalTwistAngle = 0;

	//对不同指令进行计算
	switch (g_SToolAxisInfo.cTwistCommand)
	{
	case 0:
		break;
	case 1:
	{
		if (g_SToolAxisInfo.pSChannel->SG01.ucInpStatus == INP_RUNNING || g_SToolAxisInfo.pSChannel->SG01.ucInpStatus == INP_FINISHED)
		{
			struct SPlan* pSPlanData = &(g_SToolAxisInfo.pSChannel->SG01.SPlanData);
			dTotalTwistAngle = g_SToolAxisInfo.dCurTwistAngle + g_SToolAxisInfo.dCommandTwistAngle * (1 - pSPlanData->dCurLeftDist / pSPlanData->dDistance);
		
			if (pSPlanData->dCurLeftDist <= EPSINON)
			{
				//指令角度累积到当前扭转角度中
				g_SToolAxisInfo.dCurTwistAngle += g_SToolAxisInfo.dCommandTwistAngle;
				//清除指令扭转角度
				g_SToolAxisInfo.dCommandTwistAngle = 0;
			}
		}
		break;
	}
	default:
		break;
	}

	return dTotalTwistAngle;
}







//****************************************************
//以下为编码器模块相关接口
//****************************************************

//获取值对应的角度
double getEncoderPosByValue(int64 lEncoderValue, uint16 usEncoderCirclePulseNum)
{
	double dEncoderPos;
	dEncoderPos = 360.0 * surplusDoubleToPositive(lEncoderValue, usEncoderCirclePulseNum) / usEncoderCirclePulseNum;

	return dEncoderPos;
}

//计算目标编码器的角度[0-360)
double getEncoderPos(uint8 ucEncoderNum)
{
	double dEncoderPos;
	dEncoderPos = 360.0 * surplusDoubleToPositive(g_SEncoder[ucEncoderNum].lEncoderAbsValue, g_SEncoder[ucEncoderNum].usEncoderCirclePulseNum) / g_SEncoder[ucEncoderNum].usEncoderCirclePulseNum;
	return dEncoderPos;
}

//获取当前主轴编码器角度
double getSpindleEncoderPos(uint8 ucChannelNum)
{
	//获取主轴编码器物理编号
	uint8 ucEncoderNum;
	ucEncoderNum = g_SSystemPara.ucEncoderMapArray[ucChannelNum][SPINDLE_ENCODER - 1];

	//获取主轴编码器对应角度
	double dSpindleEncoderPos;
	dSpindleEncoderPos = getEncoderPos(ucEncoderNum);
	
	return dSpindleEncoderPos;
}

//获取当前刀具轴编码器角度
double getToolAxisEncoderPos(uint8 ucChannelNum)
{
	//获取刀具轴编码器物理编号
	uint8 ucEncoderNum;
	ucEncoderNum = g_SSystemPara.ucEncoderMapArray[ucChannelNum][TOOLAXIS_ENCODER - 1];

	double dToolAxisEncoderPos;
	dToolAxisEncoderPos = getEncoderPos(ucEncoderNum);

	return dToolAxisEncoderPos;
}

//获取刀具轴编码器目标角度
double getToolAxisEncoderTargetPos(double ucFollowAngle, bool bFlag)
{
	//计算刀具轴目标位置
	double dToolAxisPos;
	//刀具轴编码器目标位置
	if (bFlag == TRUE)
		dToolAxisPos = g_SToolAxisInfo.dCommandRatio * ucFollowAngle + g_SToolAxisInfo.dCommandAngle;//+ g_SToolAxisInfo.dCurTwistAngle
	//刀具轴坐标目标位置
	else
		dToolAxisPos = g_SToolAxisInfo.dCommandRatio * ucFollowAngle + g_SToolAxisInfo.dCommandAngle + g_SToolAxisInfo.dTotalTwistAngle + g_SToolAxisInfo.dLagAngle;

		dToolAxisPos = surplusDoubleToPositive(dToolAxisPos, 360.0);

	return dToolAxisPos;
}


//速度规划（对时间节点圆整处理）
int8 preparePlanningTNewAddForToolAxis(struct SPlan* pSPlanPointer)
{
	int8 res = 1;
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离
	uint8 flag = FALSE;//输入是否为最大速度
	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期
	const uint8* pMapArray;
	pMapArray = g_SChannel[0].pMapAxisArray;


	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	dDistance = pSPlanPointer->dCurLeftDist; //对剩余距离进行规划
	dTc = pSPlanPointer->dTc;		//插补周期

	////函数运行开始
	//if (isZero(dVmax - g_SSystemPara.SAxisPara[pMapArray[4]].dVmax))
	//{//输入的最大速度为系统参数的最大速度
	//	//作用：区分   追赶、随动阶段  与  减速阶段，不同阶段使用不同的取整方式(Tconst)，重算的对象(Ve, Dec)也不同
	//	flag = TRUE;
	//}

	//决定dTac、dTconst、dTde的圆整方式
	flag = TRUE;
	//flag = FALSE;

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}

	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3 = 0; //距离对应的时间节点置0
		pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0;

		return 0;
	}

	dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
	dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移
	
	//1.规划距离不足Vs->Vmax->Ve，需要调小Vmax
	if (dDistance < dSac + dSde)
	{
		//重算最大速度
		dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));
		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
		//注：
		//调小Vmax后，规划距离恰好Vs->Vmax->Ve
		//但若规划距离过小，可能不足以Vs->Ve
		//则重算dVmax<Ve，即在减速段时仍在加速，导致减速距离<0。
		//从而满足规划距离 = 加速距离(+) + 减速距离(-)
	}


	dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
	dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
	dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

	//好情况下这里得到的dTac、dTde、dTconst都应该>=0
	//但某些坏情况下可能导致dTac、dTde、dTconst < 0，原因如下
	//1.规划距离恰好Vs->Vmax->Ve，导致dTconst=0（这里应该不需要处理该情况）
	//2.规划距离过小不足以Vs->Ve，导致规划距离 < |加速距离(+)| + |减速距离(-)|，使得dTconst<0
	//  这种情况下的dTac或dTde会小于0
	//即需要将这种坏情况进行修正
	if (dTconst <= 0)
	{
		//规划距离过小不足以Vs->Ve，解决方法（这里修正Ve）
		//1.修正Ve
		//2.修正Vs
		//2.修正Aac
		if (dTac < 0 || dTde < 0)
		{
			//Vs不能减速到Ve，根据规划距离倒算Ve
			//重算Ve后变大，使得Vs能减速到Ve
			if (dVs > dVe)
			{
				dVe = sqrt(dVs * dVs - dDistance * 2.0 * dDec);//根据减速距离公式计算末速度(不会出现负值)
				dVmax = dVs;//由于是减速  最大速度等于初速度
			}
			//Vs不能加速到Ve，根据规划距离倒算Ve
			//重算Ve后变小，使得Vs能加速到Ve
			else
			{
				dVe = sqrt(dVs * dVs + 2.0 * dAcc * dDistance);//根据加速距离公式计算末速度
				dVmax = dVe;//由于是加速  最大速度等于末速度
			}
			//坏情况已处理完成，重新计算dTac与dTde
			dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		}
		dTconst = 0;//匀速周期不能小于0  将其置为0
	}

	//规划距离足够Vs->Vmax->Ve，但Vs>Vmax导致dTac<0，这里对dTac与dAcc取反即可
	if (dTac < 0)
	{
		dTac = fabs(dTac);
		dAcc = -(dAcc);
	}

	//处理dTac与dTde过小的情况（小于0.5）
	//若dTac过小，则将规划的dVs提高到dVmax，取消加速周期
	//若dTde过小，则将规划的dVe提高到dVmax，取消减速周期
	//这里不明白为什么dTac与dTde过小时不向上取整???
	if (((int)(dTac + 0.5)) == 0)//加速周期四舍五入后为0  (判断是否小于0.5)
	{
		dTac = 0;//加速周期为0
		dVs = dVmax;//起始速度等于最大速度
	}
	//if (((int)(dTde + 0.5)) == 0)//减速周期四舍五入后为0 (判断是否小于0.5)
	//{
	//	dTde = 0;//减速周期设为0
	//	dVe = dVmax;//末速度等于最大速度
	//}

	//
	if (isZero(dTde) && isZero(dTac) && dVmax * dTc >= dDistance)
	{
		res = 2;//随动阶段
	}

	//速度规划需要做离散化处理，需要圆整dTac、dTde，这里对其向上取整
	//dTac向上取整后，实际Vmax大于规划Vmax
	//dTde向上取整后，实际Vmax大于规划Vmax
	dTac = ceil(dTac);
	dTde = ceil(dTde);

	//前面已经处理过dTac=0的情况，这里重复处理
	if (isZero(dTac) || isZero(dVs - dVmax))//加速周期为0
	{
		dVs = dVmax;
		dTac = 0;
	}
	else
	{
		//dTac向上取整后，规划参数Vmax != Vs + Acc*Tac，有两种调整方法
		//1.重算Vmax
		//2.重算Acc
		if (1)
		{
			dVmax = dAcc * (dTac * dTc) + dVs;//重算最大速度
		}
		else
		{
			dAcc = (dVmax - dVs) / (dTac * dTc);//重算加速度
		}

	}
	if (dTde < 0 || isZero(dTde))  //减速周期为0 （前提是最大速度不会比终点速度小很多(>dDec)）   普通规划限制了末速度不超过最大速度  新增距离的规划没有限制  因此需要判断<=0
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{                                  
		//dTac向上取整后，规划参数Ve != Vmax - Dec*Tde，有两种调整方法
		//1.重算Ve
		//2.重算Dec
		if (flag)
		{
			//以免减速度重算为0，影响计算
			//为什么减速度重算为0？是因为dVmax = dVe？
			dVe = dVmax - dDec * (dTde * dTc);
		}
		else
		{
			dDec = (dVmax - dVe) / (dTde * dTc);
		}
	}

	//规划参数Vs、Vmax、Ve、Acc、Dec、Tac、Tde全部重新调整，导致Distance != Sac + Sde + Sconst
	//需要重算Sconst、Tconst
	dSac = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //加速段位移（时间取整后）
	dSde = (dVmax * dVmax - dVe * dVe) / (dDec * 2);   //减速段位移 
	dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

	//此时Tac、Tconst、Tde中，仅Tconst未进行圆整
	//Tac、Tde向上取整后，导致Sac、Sde变大
	//在规划距离不变的情况下，可能导致Sconst<0
	//这里出现残余距离!!!
	if (dTconst <= 0)  //没有匀速周期
	{
		dTconst = 0;
	}
	if (flag || (isZero(dTac) && isZero(dTde)))
	{
		//向上取整
		dTconst = ceil(dTconst);  //匀速周期取整
	}
	else
	{
		//四舍五入
		dTconst = (int)(dTconst + 0.5);  //匀速周期取整
	}

	unT1 = unT2 = (uint32)dTac;               //时间节点T1  （绝对时间） 
	unT3 = unT2 + (uint32)dTconst; //时间节点T2  （绝对时间） 
	unT4 = unT5 = unT3 + (uint32)dTde;       //时间节点T3  （绝对时间） 

	dSconst = dVmax * (unT3 - unT2) * dTc; //计算匀速段位移

	dDistRemain = dDistance - dSac - dSde - dSconst;  //计算残余距离

	//传出参数
	pSPlanPointer->unT1 = unT1;		//时间节点T1,2,3
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;

	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("T形速度规划\n");
	printf("dTac = %lf\r\n", dTac);
	printf("dTconst = %lf\r\n", dTconst);
	printf("dTde = %lf\r\n", dTde);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dDe = %lf\r\n", dDistRemain);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("dVmax = %lf\r\n", dVmax);
#endif
	return res;

}
//速度规划1（不对时间节点圆整处理）
int8 preparePlanningTNewAddForToolAxis_1(struct SPlan* pSPlanPointer)
{
	int8 res = 1;
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	double dT1, dT2, dT3, dT4, dT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离
	uint8 flag = FALSE;//输入是否为最大速度
	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期
	const uint8* pMapArray;
	pMapArray = g_SChannel[0].pMapAxisArray;


	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	dDistance = pSPlanPointer->dCurLeftDist; //对剩余距离进行规划
	dTc = pSPlanPointer->dTc;		//插补周期

	////函数运行开始
	//if (isZero(dVmax - g_SSystemPara.SAxisPara[pMapArray[4]].dVmax))
	//{//输入的最大速度为系统参数的最大速度
	//	//作用：区分   追赶、随动阶段  与  减速阶段，不同阶段使用不同的取整方式(Tconst)，重算的对象(Ve, Dec)也不同
	//	flag = TRUE;
	//}

	//决定dTac、dTconst、dTde的圆整方式
	flag = TRUE;
	//flag = FALSE;

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}

	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3 = 0; //距离对应的时间节点置0
		pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0;

		return 0;
	}

	dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
	dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移

	//1.规划距离不足Vs->Vmax->Ve，需要调小Vmax
	if (dDistance < dSac + dSde)
	{
		//重算最大速度
		dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));
		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
		//注：
		//调小Vmax后，规划距离恰好Vs->Vmax->Ve
		//但若规划距离过小，可能不足以Vs->Ve
		//则重算dVmax<Ve，即在减速段时仍在加速，导致减速距离<0。
		//从而满足规划距离 = 加速距离(+) + 减速距离(-)
	}


	dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
	dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
	dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

	//好情况下这里得到的dTac、dTde、dTconst都应该>=0
	//但某些坏情况下可能导致dTac、dTde、dTconst < 0，原因如下
	//1.规划距离恰好Vs->Vmax->Ve，导致dTconst=0（这里应该不需要处理该情况）
	//2.规划距离过小不足以Vs->Ve，导致规划距离 < |加速距离(+)| + |减速距离(-)|，使得dTconst<0
	//  这种情况下的dTac或dTde会小于0
	//即需要将这种坏情况进行修正
	if (isUnEqualAndLessThanZero(dTconst))//dTconst < 0
	{
		//规划距离过小不足以Vs->Ve，解决方法（这里修正Ve）
		//1.修正Ve或Vs
		//2.修正Aac或Dec
		//3.修正Dist(√)
		if (dTac < 0 || dTde < 0)
		{
			//Vs不能减速到Ve，根据规划距离倒算Ve
			if (dVs > dVe)
			{
				dVmax = dVs;//由于是减速  最大速度等于初速度
				dDistance = -(dVe * dVe - dVs * dVs) / 2 / dDec;
			}
			//Vs不能加速到Ve，根据规划距离倒算Ve
			else
			{
				dVmax = dVe;//由于是加速  最大速度等于末速度
				dDistance = (dVe * dVe - dVs * dVs) / 2 / dAcc;
			}
			//坏情况已处理完成，重新计算dTac与dTde
			dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		}
		dTconst = 0;//匀速周期不能小于0  将其置为0
	}

	//规划距离足够Vs->Vmax->Ve，但Vs>Vmax导致dTac<0，这里对dTac与dAcc取反即可
	if (dTac < 0)
	{
		dTac = fabs(dTac);
		dAcc = -(dAcc);
	}


	dT1 = dT2 = dTac;               //时间节点T1  （绝对时间） 
	dT3 = dT2 + dTconst; //时间节点T2  （绝对时间） 
	dT4 = dT5 = dT3 + dTde;       //时间节点T3  （绝对时间） 

	dSconst = dVmax * (dT3 - dT2) * dTc; //计算匀速段位移

	dDistRemain = dDistance - dSac - dSde - dSconst;  //计算残余距离

	//传出参数
	pSPlanPointer->dT1 = dT1;		//时间节点T1,2,3
	pSPlanPointer->dT2 = dT2;
	pSPlanPointer->dT3 = dT3;
	pSPlanPointer->dT4 = dT4;
	pSPlanPointer->dT5 = dT5;

	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("T形速度规划\n");
	printf("dTac = %lf\r\n", dTac);
	printf("dTconst = %lf\r\n", dTconst);
	printf("dTde = %lf\r\n", dTde);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dDe = %lf\r\n", dDistRemain);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("dVmax = %lf\r\n", dVmax);
#endif
	return res;

}
//速度规划2（不对时间节点圆整处理），当加速时间小于0，进行二次速度规划
int8 preparePlanningTNewAddForToolAxis_2(struct SPlan* pSPlanPointer)
{
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	double dT1, dT2, dT3, dT4, dT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离
	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期


	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	dDistance = pSPlanPointer->dCurLeftDist; //对剩余距离进行规划
	dTc = pSPlanPointer->dTc;		//插补周期

	//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	if (dDistance < 0)
	{
		dDistance = 0;
	}

	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3 = 0; //距离对应的时间节点置0
		pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0;

		return 0;
	}

	dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
	dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移

	//1.规划距离不足Vs->Vmax->Ve，需要调小Vmax
	if (dDistance < dSac + dSde)
	{
		//重算最大速度
		dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));
		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
		//注：
		//调小Vmax后，规划距离恰好Vs->Vmax->Ve
		//但若规划距离过小，可能不足以Vs->Ve
		//则重算dVmax<Ve，即在减速段时仍在加速，导致减速距离<0。
		//从而满足规划距离 = 加速距离(+) + 减速距离(-)
	}


	dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
	dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
	dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

	//好情况下这里得到的dTac、dTde、dTconst都应该>=0
	//但某些坏情况下可能导致dTac、dTde、dTconst < 0，原因如下
	//1.规划距离恰好Vs->Vmax->Ve，导致dTconst=0（这里应该不需要处理该情况）
	//2.规划距离过小不足以Vs->Ve，导致规划距离 < |加速距离(+)| + |减速距离(-)|，使得dTconst<0
	//  这种情况下的dTac或dTde会小于0
	//即需要将这种坏情况进行修正
	if (isUnEqualAndLessThanZero(dTconst))//dTconst < 0
	{
		//规划距离过小不足以Vs->Ve，解决方法（这里修正Ve）
		//1.修正Ve或Vs
		//2.修正Aac或Dec
		//3.修正Dist(√)
		if (dTac < 0 || dTde < 0)
		{
			//Vs不能减速到Ve，根据规划距离倒算Ve
			if (dVs > dVe)
			{
				dVmax = dVs;//由于是减速  最大速度等于初速度
				dDistance = -(dVe * dVe - dVs * dVs) / 2 / dDec;
			}
			//Vs不能加速到Ve，根据规划距离倒算Ve
			else
			{
				dVmax = dVe;//由于是加速  最大速度等于末速度
				dDistance = (dVe * dVe - dVs * dVs) / 2 / dAcc;
			}
			//坏情况已处理完成，重新计算dTac与dTde
			dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		}
		dTconst = 0;//匀速周期不能小于0  将其置为0
	}

	//规划距离足够Vs->Vmax->Ve，但Vs>Vmax导致dTac<0，这里对dTac与dAcc取反即可
	if (dTac < 0)
	{
		dTac = fabs(dTac);
		dAcc = -(dAcc);
	}

	//加入未来新增距离进行二次规划
	if (isZero(dTac))
	{
		dVs = pSPlanPointer->dVs;		//起始速度
		dVmax = pSPlanPointer->dVmax;		//最大速度
		dVe = pSPlanPointer->dVe;		//最终速度
		dAcc = pSPlanPointer->dAcc;		//加速度
		dDec = pSPlanPointer->dDec;		//减速度
		dDistance = pSPlanPointer->dCurLeftDist; //对剩余距离进行规划
		dTc = pSPlanPointer->dTc;		//插补周期

		double dNewAddDist;
		dNewAddDist = (dTconst + dTde - 1.05) * dVe;//1.05
		dDistance += dNewAddDist; //对剩余距离进行规划


		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移

		//1.规划距离不足Vs->Vmax->Ve，需要调小Vmax
		if (dDistance < dSac + dSde)
		{
			//重算最大速度
			dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));
			dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));
			dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
			//注：
			//调小Vmax后，规划距离恰好Vs->Vmax->Ve
			//但若规划距离过小，可能不足以Vs->Ve
			//则重算dVmax<Ve，即在减速段时仍在加速，导致减速距离<0。
			//从而满足规划距离 = 加速距离(+) + 减速距离(-)
		}


		dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
		dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

		//好情况下这里得到的dTac、dTde、dTconst都应该>=0
		//但某些坏情况下可能导致dTac、dTde、dTconst < 0，原因如下
		//1.规划距离恰好Vs->Vmax->Ve，导致dTconst=0（这里应该不需要处理该情况）
		//2.规划距离过小不足以Vs->Ve，导致规划距离 < |加速距离(+)| + |减速距离(-)|，使得dTconst<0
		//  这种情况下的dTac或dTde会小于0
		//即需要将这种坏情况进行修正
		if (isUnEqualAndLessThanZero(dTconst))//dTconst < 0
		{
			//规划距离过小不足以Vs->Ve，解决方法（这里修正Ve）
			//1.修正Ve或Vs
			//2.修正Aac或Dec
			//3.修正Dist(√)
			if (dTac < 0 || dTde < 0)
			{
				//Vs不能减速到Ve，根据规划距离倒算Ve
				if (dVs > dVe)
				{
					dVmax = dVs;//由于是减速  最大速度等于初速度
					dDistance = -(dVe * dVe - dVs * dVs) / 2 / dDec;
				}
				//Vs不能加速到Ve，根据规划距离倒算Ve
				else
				{
					dVmax = dVe;//由于是加速  最大速度等于末速度
					dDistance = (dVe * dVe - dVs * dVs) / 2 / dAcc;
				}
				//坏情况已处理完成，重新计算dTac与dTde
				dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
				dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
			}
			dTconst = 0;//匀速周期不能小于0  将其置为0
		}

		//规划距离足够Vs->Vmax->Ve，但Vs>Vmax导致dTac<0，这里对dTac与dAcc取反即可
		if (dTac < 0)
		{
			dTac = fabs(dTac);
			dAcc = -(dAcc);
		}


	}



	dT1 = dT2 = dTac;               //时间节点T1  （绝对时间） 
	dT3 = dT2 + dTconst; //时间节点T2  （绝对时间） 
	dT4 = dT5 = dT3 + dTde;       //时间节点T3  （绝对时间） 

	dSconst = dVmax * (dT3 - dT2) * dTc; //计算匀速段位移

	dDistRemain = dDistance - dSac - dSde - dSconst;  //计算残余距离

	//传出参数
	pSPlanPointer->dT1 = dT1;		//时间节点T1,2,3
	pSPlanPointer->dT2 = dT2;
	pSPlanPointer->dT3 = dT3;
	pSPlanPointer->dT4 = dT4;
	pSPlanPointer->dT5 = dT5;

	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("T形速度规划\n");
	printf("dTac = %lf\r\n", dTac);
	printf("dTconst = %lf\r\n", dTconst);
	printf("dTde = %lf\r\n", dTde);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dDe = %lf\r\n", dDistRemain);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("dVmax = %lf\r\n", dVmax);
#endif
	return 1;

}



//计算下一周期角度
double nextStepPlanningAndDistReAvgToolAxis(uint32 unK, struct SPlan* pSPlanPointer)
{
	double dDistance = 0; //速度规划走的距离
	double dPreDist = 0;//规划前已走的距离
	double dTotalDist = 0;
	uint8 ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数
	double dAverageFactor;		///<比例平摊的比例系数

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return 0;
	}
	if (isZero(pSPlanPointer->dDistance))
	{
		//pSPlanPointer->ucInpStatus = INP_FINISHED;
		//return 0;
	}

	//初始化
	ucDistReAverageType = pSPlanPointer->ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数
	dAverageFactor = pSPlanPointer->dAverageFactor;		///<比例平摊的比例系数
	dPreDist = pSPlanPointer->dPreDist;

	dDistance = nextstepPlanningCombine(unK, pSPlanPointer);


	//switch (ucDistReAverageType)//根据预设的DEAL_DE_TYPE值,处理速度规划后的位移
	//{
	//case 1:
	//	dDistance += nextstepDistReAvgT(unK, pSPlanPointer);//梯形均摊后的位移S(t)
	//	break;
	//case 2:
	//	dDistance += nextstepDistReAvgS5(unK, pSPlanPointer);//五次多项式均摊后的位移S(t)
	//	break;
	//case 3:
	//	dDistance *= dAverageFactor;//按比例均摊后的位移S(t)
	//	break;
	//default:
	//	dDistance += nextstepDistReAvgT(unK, pSPlanPointer);//Default:梯形均摊后的位移S(t)
	//}

	dTotalDist = dDistance;//不算规划  已走的距离

	return dTotalDist;//返回均摊后的总位移
}
//计算下一周期角度（柔性跟随）
double nextStepPlanningAndDistReAvgToolAxis_1(struct SPlan* pSPlanPointer)
{
	double dDeltaL = 0;
	
	double dVs = pSPlanPointer->dVs;
	double dVmax = pSPlanPointer->dVmax;
	double dAcc = pSPlanPointer->dAcc;
	double dDec = pSPlanPointer->dDec;
	double dTc = pSPlanPointer->dTc;
	double dDist = pSPlanPointer->dCurLeftDist;
	double dDistMin = dVs * dTc - dDec * dTc * dTc;	//允许输出最小距离
	double dDistMax = dVs * dTc + dAcc * dTc * dTc;	//允许输出最大距离
 
	//1.dDist < dDistMin
	if (isUnEqualAndLessThanZero(dDist - dDistMin))
	{
		dDeltaL = dDistMin;
	}	
	//2.dDistMin < dDist < dDistMax
	else if (isBetweenA2B(pSPlanPointer->dCurLeftDist, dDistMin, dDistMax))
	{
		dDeltaL = dDist;
	}
	//3.dDistMax < dDist
	else if (isUnEqualAndGreaterThanZero(dDist - dDistMax))
	{
		//if
		//3.1加速（能加速则加速）
		if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT1 - 0.5))
		{
			dDeltaL = dDistMax;
		}
		//3.2匀速
		else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT3))
		{
			dDeltaL = dVs * dTc;
		}
		//3.3减速
		else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT4))
		{
			dDeltaL = dDistMin;
		}
	}

	////1.加速周期 > 1,则加速
	//else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT1 - 0.5))
	//{
	//	dDeltaL = pSPlanPointer->dVs + pSPlanPointer->dAcc;
	//}
	////2.有匀速周期，则匀速
	//else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT3 - 0.5))
	//{
	//	dDeltaL = pSPlanPointer->dVs ;
	//}
	////3.有减速周期，则减速
	//else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT4))
	//{
	//	dDeltaL = pSPlanPointer->dVs - pSPlanPointer->dDec;
	//}

	return dDeltaL;

}
//计算下一周期角度（刚性跟随）
double nextStepPlanningAndDistReAvgToolAxis_2(struct SPlan* pSPlanPointer)
{
	double dDeltaL = 0;

	double dVs = pSPlanPointer->dVs;
	double dVmax = pSPlanPointer->dVmax;
	double dAcc = pSPlanPointer->dAcc;
	double dDec = pSPlanPointer->dDec;
	double dTc = pSPlanPointer->dTc;
	double dDist = pSPlanPointer->dCurLeftDist;
	double dDistMin = dVs * dTc - dDec * dTc * dTc;	//允许输出最小距离
	double dDistMax = dVs * dTc + dAcc * dTc * dTc;	//允许输出最大距离
	if (g_SToolAxisInfo.bFollowed)
	{
		dDeltaL = dDist;
	}
	//1.dDist < dDistMin
	if (isUnEqualAndLessThanZero(dDist - dDistMin))
	{
		dDeltaL = dDistMin;
	}
	//2.dDistMin < dDist < dDistMax
	else if (isBetweenA2B(pSPlanPointer->dCurLeftDist, dDistMin, dDistMax))
	{
		dDeltaL = dDist;
	}
	//3.dDistMax < dDist
	else if (isUnEqualAndGreaterThanZero(dDist - dDistMax))
	{
		//if
		//3.1加速（能加速则加速）
		if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT1 - 0.5))
		{
			dDeltaL = dDistMax;
		}
		//3.2匀速
		else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT3))
		{
			dDeltaL = dVs * dTc;
		}
		//3.3减速
		else if (isUnEqualAndGreaterThanZero(pSPlanPointer->dT4))
		{
			dDeltaL = dDistMin;
		}
	}

	return dDeltaL;

}




#if TOOLAXIS_DEBUG_32
//数控面板显示数据初始化
void toolAxisScreenPrintInit()
{
	wordx(220, 260, 2, 0, 2, green, black, "主增");
	wordx(450, 260, 2, 0, 2, green, black, "主编");
	wordx(220, 300, 2, 0, 2, green, black, "副增");
	wordx(450, 300, 2, 0, 2, green, black, "副编");
	wordx(450, 216, 2, 0, 2, green, black, "滞后");
}

//数控面板显示数据(非1ms显示一次)
void toolAxisScreenPrint()
{
	uint8 ucSpinleAxisNum;
	uint8 ucSpindleEncoderNum;
	uint8 ucToolAxisNum;
	uint8 ucToolAxisEncoderNum;

	ucSpinleAxisNum = SPINDLE_AXIS;
	ucSpindleEncoderNum = SPINDLE_ENCODER;
	ucToolAxisNum = TOOL_AXIS;
	ucToolAxisEncoderNum = TOOLAXIS_ENCODER;

	//主轴编码器脉冲增量
	double dSpindleEncoderDelta;
	dSpindleEncoderDelta = g_SEncoder[0].usDelta;
	Trans_Data_Clear(220, 280, 9, 0, 2, white, black, dSpindleEncoderDelta);

	//主轴编码器位置
	double dSpindleEncoderPos;
	dSpindleEncoderPos = getEncoderPos(0);
	Trans_Data_Clear(420, 280, 9, 0, 2, white, black, (int32)(dSpindleEncoderPos * 1000));

	//刀具轴编码器增量
	double dToolAxisEncoderDelta;
	dToolAxisEncoderDelta = g_SEncoder[1].usDelta;
	Trans_Data_Clear(220, 325, 9, 0, 2, white, black, dToolAxisEncoderDelta);

	//刀具轴编码器位置
	double dToolAxisEncoderPos;
	dToolAxisEncoderPos = getEncoderPos(1);
	Trans_Data_Clear(420, 325, 9, 0, 2, white, black, (int32)(dToolAxisEncoderPos * 1000));

	//刀具轴落后角度
	double dToolAxisTargetPos;
	dToolAxisTargetPos = getToolAxisEncoderTargetPos(dSpindleEncoderPos, TRUE);
	double dToolAxisCurrLag;
	dToolAxisCurrLag = surplusDoubleToPositive(dToolAxisTargetPos - dToolAxisEncoderPos, 360.0);
	dToolAxisCurrLag = isZero(dToolAxisCurrLag - 360) ? 0 : dToolAxisCurrLag;
	//修正落后角度
	//当刀具轴跟随上主轴后，由于主轴转速波动，若主轴转速突减可能导致刀具轴超前主轴，即dToolAxisCurrLag值从1->359
	if (g_SToolAxisInfo.bFollowed == TRUE)
		dToolAxisCurrLag = dToolAxisCurrLag > 345 ? dToolAxisCurrLag = dToolAxisCurrLag - 360 : dToolAxisCurrLag;
	Trans_Data_Clear(420, 240, 9, 0, 2, white, black, (int32)(dToolAxisCurrLag * 1000));

}
void output485()
{
	uint8 ucSpinleAxisNum;
	uint8 ucSpindleEncoderNum;
	uint8 ucToolAxisNum;
	uint8 ucToolAxisEncoderNum;

	ucSpinleAxisNum = SPINDLE_AXIS;
	ucSpindleEncoderNum = SPINDLE_ENCODER;
	ucToolAxisNum = TOOL_AXIS;
	ucToolAxisEncoderNum = TOOLAXIS_ENCODER;

	//0.时间
	char cTime[7];
	itoaNC(g_SToolAxisInfo.unTime, cTime, 10);
	uart6_set(cTime);
	uart6_set("\t");

	
	//1.编码器0增量
	outputEncoderDelta(0);
	
	//2.编码器1增量
	outputEncoderDelta(1);

	//3.编码器0位置
	outputEncoderPos(0);

	//4.编码器1位置
	outputEncoderPos(1);

	//5.编码器1落后角度
	double dToolAxisTargetPos;
	dToolAxisTargetPos = getToolAxisEncoderTargetPos(getEncoderPos(0), TRUE);
	double dToolAxisCurrLag;
	dToolAxisCurrLag = surplusDoubleToPositive(dToolAxisTargetPos - getEncoderPos(1), 360.0);
	dToolAxisCurrLag = isZero(dToolAxisCurrLag - 360) ? 0 : dToolAxisCurrLag;
	//修正落后角度
	//当刀具轴跟随上主轴后，由于主轴转速波动，若主轴转速突减可能导致刀具轴超前主轴，即dToolAxisCurrLag值从1->359
	if (g_SToolAxisInfo.bFollowed == TRUE)
		dToolAxisCurrLag = dToolAxisCurrLag > 345 ? dToolAxisCurrLag = dToolAxisCurrLag - 360 : dToolAxisCurrLag;

	char cToolAxisCurrLag[7];
	itoaNC((int32)(dToolAxisCurrLag * 1000), cToolAxisCurrLag, 10);
	uart6_set(cToolAxisCurrLag);
	uart6_set("\t");

	//6.下一周期发送脉冲数
	outputNextPulse();


	uart6_set("\r");
}

void outputEncoderDelta(uint8 ucEncoderNum)
{
	char cEncoderDelta[4];
	itoaNC(g_SEncoder[ucEncoderNum].usDelta, cEncoderDelta, 10);
	uart6_set(cEncoderDelta);
	uart6_set("\t");
}

void outputEncoderDir(uint8 ucEncoderNum)
{
	char cEncoderDir[2];
	itoaNC(g_SEncoder[ucEncoderNum].cEncoderDir, cEncoderDir, 10);
	uart6_set(cEncoderDir);
	uart6_set("\t");
}

void outputEncoderPos(uint8 ucEncoderNum)
{
	double dEncoderPos;
	dEncoderPos = getEncoderPos(ucEncoderNum);
	char cEncoderPos[7];
	itoaNC((int32)(dEncoderPos * 1000), cEncoderPos, 10);
	uart6_set(cEncoderPos);
	uart6_set("\t");
}

void outputIdealDelta(uint8 ucAxisNum)
{
	char cIdealDelta[4];
	itoaNC(g_SChannel[0].SRun.sLastDeltaPulse[ucAxisNum], cIdealDelta, 10);
	uart6_set(cIdealDelta);
	uart6_set("\t");
}

void outputIdealDir(uint8 ucAxisNum)
{
	int8 cDir;
	cDir = g_SChannel[0].SRun.sLastDeltaPulse[ucAxisNum] > 0 ? 1 : -1;
	char cIdealDir[2];
	itoaNC(cDir, cIdealDir, 10);
	uart6_set(cIdealDir);
	uart6_set("\t");
}

void outputIdealPos(uint8 ucAxisNum)
{
	double dAxisPos;
	getSingleAxisRealTimePoint(&dAxisPos, ucAxisNum, g_SChannel[0].pMapAxisArray);
	dAxisPos = dAxisPos - g_SChannel[0].SRun.sAxisDeltaPulse[0] * 0.036;
	char cAxisPos[7];
	itoaNC((int32)(dAxisPos * 1000), cAxisPos, 10);
	uart6_set(cAxisPos);
	uart6_set("\t");
}

void outputIdealPulse(uint8 ucAxisNum)
{
	//下一周期发送脉冲
	int16 sPulse;
	sPulse = g_SChannel[0].SRun.sAxisDeltaPulse[ucAxisNum];
	char cPulse[4];
	itoaNC(sPulse, cPulse, 10);
	uart6_set(cPulse);
	uart6_set("\t");
}

void outputNextPulse()
{
	char cNextPulse[4];
	itoaNC(g_SChannel[0].SRun.sAxisDeltaPulse[TOOL_AXIS], cNextPulse, 10);
	uart6_set(cNextPulse);
	uart6_set("\t");
}



#elif TOOLAXIS_DEBUG_VS

//输出编码器数据
void outputVS()
{
	uint8 ucSpinleAxisNum;
	uint8 ucSpindleEncoderNum;
	uint8 ucToolAxisNum;
	uint8 ucToolAxisEncoderNum;

	ucSpinleAxisNum = SPINDLE_AXIS;
	ucSpindleEncoderNum = SPINDLE_ENCODER;
	ucToolAxisNum = TOOL_AXIS;
	ucToolAxisEncoderNum = TOOLAXIS_ENCODER;

	uint8* ucMapArray = g_SChannel[0].pMapAxisArray;
	uint8* ucEncoderMapArray = g_SSystemPara.ucEncoderMapArray[0];

	//0.当前编号
	fprintf(pWriteFid, "编号：\t%d\t", g_SToolAxisInfo.unTime);

	//1.编码器0增量
	fprintf(pWriteFid, "主轴上一周期脉冲增量：\t%d\t", g_SEncoder[0].usDelta);

	//2.编码器1增量
	fprintf(pWriteFid, "刀轴上一周期脉冲增量：\t%d\t", g_SEncoder[1].usDelta);

	//3.编码器0位置
	double dEncoderPos_0;
	dEncoderPos_0 = getEncoderPos(0);
	fprintf(pWriteFid, "主轴当前周期开始角度：\t%.3f\t", dEncoderPos_0);

	//4.编码器1位置
	double dEncoderPos_1;
	dEncoderPos_1 = getEncoderPos(1);
	fprintf(pWriteFid, "刀轴当前周期开始角度：\t%.3f\t", dEncoderPos_1);

	//5.刀具轴目标角度
	double dToolAxisTargetPos;
	dToolAxisTargetPos = getToolAxisEncoderTargetPos(dEncoderPos_0, TRUE);
	fprintf(pWriteFid, "刀轴当前周期开始目标角度：\t%.3f\t", dToolAxisTargetPos);

	//5.刀具轴落后角度
	double dToolAxisCurrLag;
	dToolAxisCurrLag = surplusDoubleToPositive(dToolAxisTargetPos - dEncoderPos_1, 360);
	dToolAxisCurrLag = isZero(dToolAxisCurrLag - 360) ? 0 : dToolAxisCurrLag;
	//修正落后角度
	//当刀具轴跟随上主轴后，由于主轴转速波动，若主轴转速突减可能导致刀具轴超前主轴，即dToolAxisCurrLag值从1->359
	if (g_SToolAxisInfo.bFollowed == TRUE)
		dToolAxisCurrLag = dToolAxisCurrLag > 345 ? dToolAxisCurrLag = dToolAxisCurrLag - 360 : dToolAxisCurrLag;

	fprintf(pWriteFid, "刀轴当前周期开始落后角度：\t%.3f\t", dToolAxisCurrLag);

	//6.刀具轴追赶距离
	double dSpindle;
	dSpindle = dSpindle = g_SChannel[0].SRun.sAxisDeltaPulse[ucSpinleAxisNum] * 0.036;
	double dLeftDistance;
	dLeftDistance = dToolAxisCurrLag + dSpindle * g_SToolAxisInfo.dCommandRatio;
	fprintf(pWriteFid, "刀轴追赶距离：\t%.3f\t", dLeftDistance);

	//7.下一周期发送脉冲
	int16 sPulse;
	sPulse = g_SChannel[0].SRun.sAxisDeltaPulse[ucToolAxisNum];
	fprintf(pWriteFid, "下一周期发送脉冲：\t%d\t", sPulse);

	fprintf(pWriteFid, "\n");
}
//输出坐标数据
void outputVS_2()
{
	uint8 ucSpinleAxisNum;
	uint8 ucSpindleEncoderNum;
	uint8 ucToolAxisNum;
	uint8 ucToolAxisEncoderNum;

	ucSpinleAxisNum = SPINDLE_AXIS;
	ucSpindleEncoderNum = SPINDLE_ENCODER;
	ucToolAxisNum = TOOL_AXIS;
	ucToolAxisEncoderNum = TOOLAXIS_ENCODER;

	uint8* ucMapArray = g_SChannel[0].pMapAxisArray;
	uint8* ucEncoderMapArray = g_SSystemPara.ucEncoderMapArray[0];
	
	struct _SRun* pSRun = &(g_SChannel[0].SRun);

	//0.当前编号
	fprintf(pWriteFid, "编号：\t%d\t", g_SToolAxisInfo.unTime);

	//1.主轴理论增量
	fprintf(pWriteFid, "主轴下一周期脉冲增量：\t%d\t", pSRun->sAxisDeltaPulse[SPINDLE_AXIS]);

	//2.刀具轴理论增量
	fprintf(pWriteFid, "刀轴下一周期脉冲增量：\t%d\t", pSRun->sAxisDeltaPulse[TOOL_AXIS]);

	//3.主轴当前周期结束坐标
	double dSpindlePos; 
	getSingleAxisRealTimePoint(&dSpindlePos, SPINDLE_AXIS, ucMapArray);

	fprintf(pWriteFid, "主轴当前周期结束角度：\t%.3f\t", dSpindlePos);

	//4.刀具轴当前周期结束坐标
	double dToolAxisPos;
	getSingleAxisRealTimePoint(&dToolAxisPos, TOOL_AXIS, ucMapArray);
	fprintf(pWriteFid, "刀轴当前周期结束角度：\t%.3f\t", dToolAxisPos);

	//5.刀具轴当前周期结束目标坐标
	double dToolAxisTargetPos;
	dToolAxisTargetPos = getToolAxisEncoderTargetPos(dSpindlePos, TRUE);
	fprintf(pWriteFid, "刀轴当前周期结束目标角度：\t%.3f\t", dToolAxisTargetPos);

	//6.刀具轴落后角度
	double dToolAxisCurrLag;
	dToolAxisCurrLag = surplusDoubleToPositive(dToolAxisTargetPos - dToolAxisPos, 360);
	dToolAxisCurrLag = isZero(dToolAxisCurrLag - 360) ? 0 : dToolAxisCurrLag;
	//修正落后角度
	//当刀具轴跟随上主轴后，由于主轴转速波动，若主轴转速突减可能导致刀具轴超前主轴，即dToolAxisCurrLag值从1->359
	if (g_SToolAxisInfo.bFollowed == TRUE)
		dToolAxisCurrLag = dToolAxisCurrLag > 180 ? dToolAxisCurrLag = dToolAxisCurrLag - 360 : dToolAxisCurrLag;

	fprintf(pWriteFid, "刀轴当前周期结束落后角度：\t%.3f\t", dToolAxisCurrLag);

	//6.刀具轴追赶距离
	//double dSpindle;
	//dSpindle = dSpindle = g_SChannel[0].SRun.sAxisDeltaPulse[ucSpinleAxisNum] * 0.036;
	//double dLeftDistance;
	//dLeftDistance = dToolAxisCurrLag + dSpindle * g_SToolAxisInfo.dCommandRatio;
	//fprintf(pWriteFid, "刀轴追赶距离：\t%.3f\t", dLeftDistance);

	//7.下一周期发送脉冲
	int16 sPulse;
	sPulse = g_SChannel[0].SRun.sAxisDeltaPulse[ucToolAxisNum];
	fprintf(pWriteFid, "下一周期发送脉冲：\t%d\t", sPulse);

	fprintf(pWriteFid, "\n");
}

#endif