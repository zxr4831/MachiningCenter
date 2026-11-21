#include "ErrorCompensation.h"
#include <math.h>
#include "CTrajPlanning.h"
#include "GeneralFunctions.h"
#include <stdlib.h>
#include "Channel.h"
#include "motion.h"
SRevClearanceComp g_SRevClearanceComp[AXIS_NUM_MAX]; //反向间隙补偿


/*************************************************
  函数名称：initAxisRevData(SRevClearanceComp *pSRevClearanceComp)
  函数功能  初始化单轴固定参数
  输入参数：SRevClearanceComp * 反向误差补偿结构体指针
  输出参数：
  返 回 值：1 成功
  备    注：
**************************************************/
int8 initAxisRevData(SRevClearanceComp* pSRevClearanceComp)
{
	struct SPlan* pSPlan = &pSRevClearanceComp->SPlanData;
	pSPlan->dTc = g_SSystemPara.dInterpolationTime;//插补时间
	pSPlan->unCurrentT = 0;
	pSPlan->ucAccVelPlanType = VEL_PLAN_T_CURVE;     // T型规划
	pSPlan->ucDecVelPlanType = VEL_PLAN_T_CURVE;     // T型规划
	pSPlan->ucDistReAverageType = DIST_REAVG_RATIO;  // 比例均摊   其他均摊（T型）需要改程序   unT5->unDistReT3
	pSPlan->cInterpDir = INP_GO_AHEAD;//选择方向
	return 1;
}

/*************************************************
  函数名称：updateSingleAxisRevPlan(uint8 ucPhysicalAxis)
  函数功能  更新单轴反向间隙补偿的速度规划（用系统参数）
  输入参数：ucPhysicalAxis 物理轴号
  输出参数：
  返 回 值：-1 物理轴号错误
			0  距离为0
			1  成功
  备    注：
**************************************************/
int8 updateSingleAxisRevPlan(uint8 ucPhysicalAxis)
{
	SRevClearanceComp* pSRevClearanceComp;//单轴反向间隙补偿结构体
	struct SPlan* pSPlan;//规划指针
	double dDist;

	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}

	pSRevClearanceComp = &g_SRevClearanceComp[ucPhysicalAxis];//单轴反向间隙补偿结构体
	pSPlan = &pSRevClearanceComp->SPlanData;//规划指针
	dDist = fabs(g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist);// 从系统参数获取补偿距离

	if (isZero(dDist))
	{
		pSRevClearanceComp->ucStatus = ERROR_COMPENSATION_OFF;//进入无补偿的状态
		return 0;
	}

	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)//旋转轴
	{
		//pSPlan->dDistance = pSPlan->dCurLeftDist = pSPlan->dTargetDist = dDist;//赋值距离  -->脉冲数
		//pSPlan->dAcc = RToAngle(g_SSystemPara.dRevClearanceCompAcc, ucPhysicalAxis)/g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; // 加速度
		//pSPlan->dDec = RToAngle(g_SSystemPara.dRevClearanceCompDec, ucPhysicalAxis) / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; // 减速度
		//pSPlan->dVmax = RToAngle(g_SSystemPara.dRevClearanceCompVmax, ucPhysicalAxis) / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; // 最大速度
		//pSPlan->dVs = pSPlan->dVe = RToAngle(g_SSystemPara.dRevClearanceCompVs, ucPhysicalAxis) / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; //初速度 末速度

		pSPlan->dDistance = pSPlan->dCurLeftDist = pSPlan->dTargetDist = dDist * 1000 ;//赋值距离  -->脉冲数  // mm转脉冲 1mm为1000脉冲
		pSPlan->dAcc = g_SSystemPara.dRevClearanceCompAcc / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; // 加速度
		pSPlan->dDec = g_SSystemPara.dRevClearanceCompDec / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; // 减速度
		pSPlan->dVmax = g_SSystemPara.dRevClearanceCompVmax / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; // 最大速度
		pSPlan->dVs = pSPlan->dVe = g_SSystemPara.dRevClearanceCompVs / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm; //初速度 末速度
	}
	else//直线轴
	{
		pSPlan->dDistance = pSPlan->dCurLeftDist = pSPlan->dTargetDist = dDist;//赋值距离 -->mm
		pSPlan->dAcc = g_SSystemPara.dRevClearanceCompAcc; // 加速度
		pSPlan->dDec = g_SSystemPara.dRevClearanceCompDec; // 减速度
		pSPlan->dVmax = g_SSystemPara.dRevClearanceCompVmax; // 最大速度
		pSPlan->dVs = pSPlan->dVe = g_SSystemPara.dRevClearanceCompVs; //初速度 末速度
	}

	if(pSPlan->dAcc <= 0 || pSPlan->dDec <= 0)//判断加速度是否为正
	{
		pSRevClearanceComp->ucStatus = ERROR_COMPENSATION_OFF;
		return -1;
	}
	if(preparePlanningAndDistReAvg(pSPlan, FALSE) != 1)//速度规划+比例平摊
	{
		pSRevClearanceComp->ucStatus = ERROR_COMPENSATION_OFF;
		clearSPlanData(pSPlan);
		return -1;
	}

	pSPlan->unCurrentT = 0; // 时间
	pSRevClearanceComp->dCurDist = 0;//已补偿距离

	pSRevClearanceComp->ucStatus = ERROR_COMPENSATION_ON;//开启反向间隙补偿
	return 1;
}

/*************************************************
  函数名称：initSingleAxisRev(uint8 ucPhysicalAxis, int8 cDir)
  函数功能  初始化单轴反向间隙补偿
  输入参数：ucPhysicalAxis 物理轴号
			cDir  补偿方向    每个物理轴首次调用时有效  ， 方向与回零方向有关
  输出参数：
  返 回 值：-1 物理轴号错误
			 1 成功
			 2 成功 待生效
  备    注：
**************************************************/
int8 initSingleAxisRev(uint8 ucPhysicalAxis, int8 cDir)
{
	SRevClearanceComp* pSRevClearanceComp;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	pSRevClearanceComp = &g_SRevClearanceComp[ucPhysicalAxis];

	switch (pSRevClearanceComp->ucStatus)
	{
	case ERROR_COMPENSATION_INIT:
		if (cDir != INP_GO_AHEAD && cDir != INP_GO_BACK)
		{
			pSRevClearanceComp->cDir = -pSRevClearanceComp->cCurrentDir;//根据最近的运动方向判断补偿方向
		}
		else
		{
			pSRevClearanceComp->cDir = cDir; // 设置方向  只能在回零后初始化的时候赋值方向
		}
		if (pSRevClearanceComp->cDir == INP_GO_AHEAD || pSRevClearanceComp->cDir == INP_GO_BACK)//只有轴方向不为0时，补偿状态才置为on 2026.4.9
		{
			updateSingleAxisRevPlan(ucPhysicalAxis);
		}
		break;
	default:
	case ERROR_COMPENSATION_OFF:     //反向误差补偿关闭
	case ERROR_COMPENSATION_ON:   	//回零后 反向误差补偿开启
	case ERROR_COMPENSATION_MODIFY:	 //重新设置反向误差后 待currentT回到0时，才能重新规划 使用新的补偿距离
		if(pSRevClearanceComp->cDir == 0) //若补偿方向为0,则不得写入 MODIFY
		{
			return 1;
		}
		pSRevClearanceComp->ucStatus = ERROR_COMPENSATION_MODIFY;
		if (pSRevClearanceComp->SPlanData.unCurrentT == 0)
		{
			return updateSingleAxisRevPlan(ucPhysicalAxis);
		}
		return 2;
	}
	return 1;
}


/*************************************************
  函数名称：revCompEnabled(uint8 ucPhysicalAxis, bool bEnable)
  函数功能  使能反向间隙补偿
  输入参数：
  输出参数：
  返 回 值：-1 物理轴号错误
			0  状态为INIT  忽略操作
			1  成功
  备    注：
**************************************************/
int8 revCompEnabled(uint8 ucPhysicalAxis, bool bEnable)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SRevClearanceComp[ucPhysicalAxis].ucStatus == ERROR_COMPENSATION_INIT)
	{
		return 0;
	}
	if (bEnable == TRUE && fabs(g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist) > 0) // 轴使能且补偿距离不为0 2026.4.9
	{
		g_SRevClearanceComp[ucPhysicalAxis].ucStatus = ERROR_COMPENSATION_ON;
	}
	else
	{
		g_SRevClearanceComp[ucPhysicalAxis].ucStatus = ERROR_COMPENSATION_OFF;
	}
	return 1;
}


/*************************************************
  函数名称：initAxisRevDataAPI()
  函数功能  初始化单轴固定参数循环
  输入参数：
  输出参数：
  返 回 值：1 成功
  备    注：
**************************************************/
int8 initAxisRevDataAPI()
{
	uint8 ucCounter = 0;
	for (ucCounter = 0; ucCounter < AXIS_PHYSICS_NUM_MAX; ucCounter++)
	{
		initAxisRevData(&g_SRevClearanceComp[ucCounter]);//初始化固定参数
	}
	return 1;
}


/*************************************************
  函数名称：revCompEnabledAPI(uint8 ucChannelNum, uint8 ucAxis, bool bEnable)
  函数功能  使能反向间隙补偿API函数
  输入参数：
  输出参数：
  返 回 值：-1 物理轴号错误
			0  状态为INIT  忽略操作
			1  成功
  备    注：
**************************************************/
int8 revCompEnabledAPI(uint8 ucChannelNum, uint8 ucAxis, bool bEnable)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	return revCompEnabled(g_SChannel[ucChannelNum].pMapAxisArray[ucAxis],bEnable);
}


/*************************************************
  函数名称：initSingleAxisRevAPI(uint8 ucChannelNum ,uint8 ucAxis , int8 cDir)
  函数功能  初始化单轴反向间隙补偿接口函数
  输入参数：ucChannelNum	通道号
			ucAxis			轴号
  输出参数：
  返 回 值：-2 错误的方向
			-1 通道号或轴号错误
			 1 成功
			 2 成功 待生效
  备    注：
**************************************************/
int8 initRevCompAfterHomeAPI(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX || g_SChannel->pMapAxisArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	g_SRevClearanceComp[g_SChannel[ucChannelNum].pMapAxisArray[ucAxis]].ucStatus = ERROR_COMPENSATION_INIT;//进入初始状态
	return initSingleAxisRev(g_SChannel[ucChannelNum].pMapAxisArray[ucAxis], 0);
}


/*************************************************
  函数名称：initRevCompAfterDistEditedAPI(uint8 ucPhysicalAxis)
  函数功能  参数编辑时  初始化单轴反向间隙补偿接口函数
  输入参数：ucPhysicalAxis	物理轴号
  输出参数：
  返 回 值：-1 物理轴号错误
			0  补偿值为0 
			1  完成
  备    注：
**************************************************/
int8 initRevCompAfterDistEditedAPI(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (isZero(g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist))
	{//反向间隙补偿为0   进入测量模式  直接关闭反向间隙补偿 
		g_SRevClearanceComp[ucPhysicalAxis].ucStatus = ERROR_COMPENSATION_OFF;
		return 0;
	}
	initSingleAxisRev(ucPhysicalAxis, 0);//重新补偿
	return 1;
}



/*************************************************
  函数名称：initSingleAxisRevLoopAPI()
  函数功能  初始化单轴反向误差循环接口
  输入参数：
  输出参数：
  返 回 值：1 成功
  备    注：
**************************************************/
int8 initSingleAxisRevLoopAPI()
{
	uint8 ucCounter;
	for (ucCounter = 0; ucCounter < AXIS_PHYSICS_NUM_MAX; ucCounter++)
	{
		initSingleAxisRev(ucCounter, 0);
	}
	return 1;
}

/*************************************************
  zxr: 停止接管时清除续补方向，保留曲线进度和已补偿距离
  函数名称：holdRevCompensationAPI(uint8 ucPhysicalAxis)
  函数功能  保持单轴反向间隙补偿，等待新的运动方向
  输入参数：ucPhysicalAxis 物理轴号
  输出参数：
  返 回 值：无
  备    注：
**************************************************/
void holdRevCompensationAPI(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis < AXIS_PHYSICS_NUM_MAX)
	{
		g_SRevClearanceComp[ucPhysicalAxis].cCurrentDir = 0;
	}
}

/*************************************************
  函数名称：getCompensationPulseAPI(uint8 ucPhysicalAxis , int8 cDir)
  函数功能  获取单轴反向间隙补偿值 接口
  输入参数：ucPhysicalAxis	物理轴号
			cDir			移动方向
  输出参数：
  返 回 值：补偿值  pulse
  备    注：
**************************************************/
int16 getCompensationPulseAPI(uint8 ucPhysicalAxis, int8 cDir)
{
	float fTargetDist;
	int16 sOutputPulse;
	SRevClearanceComp* pSRevClearanceComp;
	struct SPlan* pSPlan;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return 0;
	}

	pSRevClearanceComp = &g_SRevClearanceComp[ucPhysicalAxis];
	pSPlan = &pSRevClearanceComp->SPlanData;

	//记录方向
	pSRevClearanceComp->cCurrentDir = (cDir == 0) ? pSRevClearanceComp->cCurrentDir : cDir;
	// zxr: 停止保持后无新方向不续补；正常运行的零脉冲周期仍沿用原方向
	if (pSRevClearanceComp->cCurrentDir == 0)
	{
		return 0;
	}

	//根据状态处理
	switch (pSRevClearanceComp->ucStatus)
	{
	case ERROR_COMPENSATION_INIT:
	default:
	case ERROR_COMPENSATION_OFF:     //回零前 反向误差补偿关闭
		return 0;

	case ERROR_COMPENSATION_MODIFY:	 //重新设置反向误差后 待currentT回到0时，才能重新规划 使用新的补偿距离
		if (pSRevClearanceComp->SPlanData.unCurrentT == 0)
		{
			if(updateSingleAxisRevPlan(ucPhysicalAxis) != 1)
			{
				return 0;
			}
			return getCompensationPulseAPI(ucPhysicalAxis, cDir);
		}
	case ERROR_COMPENSATION_ON:   	//回零后 反向误差补偿开启

		if (pSRevClearanceComp->cCurrentDir == pSRevClearanceComp->cDir)  //判断补偿的反向
		{

			pSPlan->unCurrentT = min(pSPlan->unCurrentT + 1, pSPlan->unT5);//unT5表示补偿完毕
		}
		else if (pSRevClearanceComp->cCurrentDir == -pSRevClearanceComp->cDir)
		{
			if (pSPlan->unCurrentT != 0)//反向不能超过0时间节点
			{
				pSPlan->unCurrentT--;
			}
		}

		if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
		{
			fTargetDist = (double)pSRevClearanceComp->cDir * nextstepPlanningAndDistReAvg(pSPlan->unCurrentT, pSPlan); //计算目标位置
			sOutputPulse = round((double)fTargetDist - pSRevClearanceComp->dCurDist);//计算到达目标位置需要的脉冲数
			pSRevClearanceComp->dCurDist += sOutputPulse;
		}
		else
		{
			fTargetDist = (double)pSRevClearanceComp->cDir * nextstepPlanningAndDistReAvg(pSPlan->unCurrentT, pSPlan); //计算目标位置
			// sOutputPulse = round(((double)fTargetDist - pSRevClearanceComp->dCurDist) / g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm);
			//计算到达目标位置需要的脉冲数
			calcPulseWithCheck((double)fTargetDist - pSRevClearanceComp->dCurDist,g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm,&sOutputPulse);
			pSRevClearanceComp->dCurDist += sOutputPulse * g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm;//修正当前已补偿值
		}

		//printf(" 当前补偿值:%lf ", pSRevClearanceComp->dCurDist);
		return sOutputPulse;
	}
}

/*************************************************
  函数名称：isRevCompFinishedAPI
  函数功能：只读检查物理轴反向间隙补偿规划是否已走到当前方向终点
  输入参数：ucPhysicalAxis - 物理轴号，小于 AXIS_PHYSICS_NUM_MAX
  输出参数：无，不推进补偿时间或输出脉冲
  返 回 值：TRUE 无待完成补偿；FALSE 参数无效或补偿尚未结束
  备    注：插补周期内只读共享补偿状态，供同步段完成门控使用
****************************************/
bool isRevCompFinishedAPI(uint8 ucPhysicalAxis)
{
    const SRevClearanceComp* pComp;
    /* 拒绝无效物理轴，避免访问补偿数组之外的状态。 */
    if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
    {
        return FALSE;
    }
    /* 引用共享 g_SRevClearanceComp 的当前物理轴，仅检查，不改变补偿规划。 */
    pComp = &g_SRevClearanceComp[ucPhysicalAxis];
    /* 未初始化或关闭的补偿没有待完成脉冲。 */
    if (pComp->ucStatus == ERROR_COMPENSATION_INIT
        || pComp->ucStatus == ERROR_COMPENSATION_OFF)
    {
        return TRUE;
    }
    /* MODIFY 在零端点会重载参数；另一端已排空时允许先交接，下次反向再更新。 */
    if (pComp->ucStatus == ERROR_COMPENSATION_MODIFY
        && pComp->SPlanData.unCurrentT == 0)
    {
        return FALSE;
    }
    /* 正向补偿必须走到规划末端，才能交接同步段。 */
    if (pComp->cCurrentDir == pComp->cDir)
    {
        return pComp->SPlanData.unCurrentT == pComp->SPlanData.unT5;
    }
    /* 反向补偿必须退回零端点，才能交接同步段。 */
    if (pComp->cCurrentDir == -pComp->cDir)
    {
        return pComp->SPlanData.unCurrentT == 0;
    }
    return pComp->SPlanData.unT5 == 0;
}

/*******************************螺距补偿*************************************/

//螺距误差补偿 数据
SScrewComp g_SScrewComp[AXIS_PHYSICS_NUM_MAX];

/*************************************************
  函数名称：screwCompDisableAPI(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  取消单表螺距误差补偿
  输入参数：ucChannelNum	通道号
			ucAxis			轴号
  输出参数：
  返 回 值：-2 错误的映射
			-1 通道号或轴号错误
			 1 成功
  备    注：
**************************************************/
int8 screwCompDisableAPI(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis>=AXIS_NUM_MAX)
	{
		return -1;
	}
	if (g_SChannel[ucChannelNum].pMapAxisArray[ucAxis] == AXIS_NULL)
	{
		return -2;
	}

	g_SScrewComp[g_SChannel[ucChannelNum].pMapAxisArray[ucAxis]].bEnabled = FALSE;
	return 1;
}

/*************************************************
  函数名称：initScrewComp
  函数功能：初始化螺距补偿数据
  输入参数：ucPhysicalAxis	物理轴号
  输出参数：g_SScrewComp 螺距补偿结构体
  返 回 值：-1 输入物理轴超出最大值， 0 螺距补偿补偿功能未开启，1 初始化成功
  备    注：
**************************************************/
int8 initScrewComp(uint8 ucPhysicalAxis) // 初始化，并开始螺距补偿
{
	int16 sTargetSeg;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.bScrewCompEnable[ucPhysicalAxis] != TRUE || g_SSystemPara.ScrewCompGap[ucPhysicalAxis] <= 0)
	{
		return 0;
	}
	//计算当前所处区间
	sTargetSeg = (int)(g_dRealTimePointArray[ucPhysicalAxis]) / g_SSystemPara.ScrewCompGap[ucPhysicalAxis] + g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis];

	//正数的区间 整体移动一个位置  // zxr: 根据目标坐标区分正负区间，仅零点及正区间的段号加1，避免负区间选错补偿值
	if (g_dRealTimePointArray[ucPhysicalAxis] >= 0)
	{
		sTargetSeg += 1;
	}

	//开启螺距补偿
	g_SScrewComp[ucPhysicalAxis].bEnabled = TRUE;

	//判断区间是否在数组范围内
	if (sTargetSeg < SCREWDEVIATION_NUM_MAX && sTargetSeg >= 0)
	{
		//加载新区间的参数
		g_SScrewComp[ucPhysicalAxis].sCurSeg = sTargetSeg;
		g_SScrewComp[ucPhysicalAxis].cCurError = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
		g_SScrewComp[ucPhysicalAxis].dDeltaError = (double)g_SScrewComp[ucPhysicalAxis].cCurError / (g_SSystemPara.ScrewCompGap[ucPhysicalAxis] * 1000);
	}
	else
	{	//区间不存在   退化成无螺距补偿
		g_SScrewComp[ucPhysicalAxis].sCurSeg = sTargetSeg;
		g_SScrewComp[ucPhysicalAxis].cCurError = 0;
		g_SScrewComp[ucPhysicalAxis].dDeltaError = 0;
		g_SScrewComp[ucPhysicalAxis].bEnabled = FALSE;
		return 0;
	}
	g_SScrewComp[ucPhysicalAxis].dRecorder = 0;  //初始化记录器  2023.3.24 朱稳中
	return 1;
}


/*************************************************
  函数名称：outPutAbsPointWithScrewComp
  函数功能：利用理论坐标和螺距补偿生成SRun结构体数据
  输入参数：ucPhysicalAxis	物理轴号
  输出参数：g_SScrewComp 螺距补偿结构体
  返 回 值：-1 输入物理轴超出最大值， 0 螺距补偿补偿功能未开启，1 初始化成功
  备    注：
**************************************************/
int8 outPutAbsPointWithScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;
	double dPulseToMm;
	double dSum;
	uint8 ucPhysicalAxis;
	int16 sTargetSeg; // 目标的区间

	if (pSRun == NULL)//空指针
	{
		return FALSE;
	}

	//获取实时点作为上一点的位置
	getRealTimePoint((double*)&pSRun->SLastAbsCoord, AXIS_NUM_MAX, ucAxisMap, NULL);

	for (i = 0; i < nArraySize; i++)
	{
		ucPhysicalAxis = ucAxisMap[i];

		if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX || (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))) continue;  //映射为空的物理轴  不计算
		//目标轴的脉冲当量
		dPulseToMm = (dPulseToMmArray == NULL) ? g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm : dPulseToMmArray[i];
		if(isZero(dPulseToMm) || !isfinite(dPulseToMm))//增加校验
		{
			initSRunPulseBuf(pSRun, 0x3F); //清空所有轴脉冲
			parseRunningError(9999,GCODE_ERROR_NUM153,9999);
			return FALSE;
		}
		if (g_SScrewComp[ucPhysicalAxis].bEnabled && g_SSystemPara.ScrewCompGap[ucPhysicalAxis] != 0)//开启了螺距补偿且补偿间隔不能为0
		{
			//目标点的区间
			sTargetSeg = (int)(pdTargetPoint[i]) / g_SSystemPara.ScrewCompGap[ucPhysicalAxis] + g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis];

			if (pdTargetPoint[i] >= 0)//正数的区间 整体移动一个位置  // zxr: 根据目标坐标区分正负区间，仅零点及正区间的段号加1，避免负区间选错补偿值
			{
				sTargetSeg += 1;
			}

			if (sTargetSeg == g_SScrewComp[ucPhysicalAxis].sCurSeg)
			{	//①区间相同
				dSum = (pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) * (1 + g_SScrewComp[ucPhysicalAxis].dDeltaError) + g_SScrewComp[ucPhysicalAxis].dRecorder;

				//根据目标点和当前点计算脉冲当量
				//pSRun->sAxisDeltaPulse[i] = round(dSum / dPulseToMm);
				if (calcPulseWithCheck(dSum, dPulseToMm, &pSRun->sAxisDeltaPulse[i]) != 1)
				{
					return FALSE; 
                }
	
				//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
				g_SScrewComp[ucPhysicalAxis].dRecorder = dSum - pSRun->sAxisDeltaPulse[i] * dPulseToMm;
			}
			else
			{	//②区间不同    默认最多跨越一个区间
				double dDivisionPoint;
				int16 sNewSegPulse;

				//分割点
				if (sTargetSeg > g_SScrewComp[ucPhysicalAxis].sCurSeg)//反向 
				{
					dDivisionPoint = (g_SScrewComp[ucPhysicalAxis].sCurSeg - g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis]) * (int)g_SSystemPara.ScrewCompGap[ucPhysicalAxis];
				}
				else //正向
				{
					dDivisionPoint = (sTargetSeg - g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis]) * g_SSystemPara.ScrewCompGap[ucPhysicalAxis];
				}

				//计算起点到分割点
				dSum = (dDivisionPoint - pSRun->SLastAbsCoord.dArray[i]) * (1 + g_SScrewComp[ucPhysicalAxis].dDeltaError) + g_SScrewComp[ucPhysicalAxis].dRecorder;

				//根据分割点和当前点计算脉冲当量
				//pSRun->sAxisDeltaPulse[i] = round(dSum / dPulseToMm);
				if (calcPulseWithCheck(dSum, dPulseToMm, &pSRun->sAxisDeltaPulse[i]) != 1)
				{
					return FALSE;
                }
				//判断区间是否在数组范围内
				if (sTargetSeg < SCREWDEVIATION_NUM_MAX && sTargetSeg >= 0)
				{
					//加载新区间的参数
					g_SScrewComp[ucPhysicalAxis].sCurSeg = sTargetSeg;
					g_SScrewComp[ucPhysicalAxis].cCurError = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
					g_SScrewComp[ucPhysicalAxis].dDeltaError = (double)g_SScrewComp[ucPhysicalAxis].cCurError / (g_SSystemPara.ScrewCompGap[ucPhysicalAxis] * 1000);
				}
				else
				{	//区间不存在   退化成无螺距补偿
					g_SScrewComp[ucPhysicalAxis].sCurSeg = sTargetSeg;
					g_SScrewComp[ucPhysicalAxis].cCurError = 0;
					g_SScrewComp[ucPhysicalAxis].dDeltaError = 0;
				}

				//计算分割点到目标点
				dSum = (pdTargetPoint[i] - dDivisionPoint) * (1 + g_SScrewComp[ucPhysicalAxis].dDeltaError);

				//根据目标点和分割点计算脉冲当量
				//sNewSegPulse = round(dSum / dPulseToMm);
				if (calcPulseWithCheck(dSum, dPulseToMm, &sNewSegPulse) != 1)
				{
					return FALSE;
                }
				//计算记录器的值
				g_SScrewComp[ucPhysicalAxis].dRecorder = dSum - sNewSegPulse * dPulseToMm;
				int32 nPulseNum = pSRun->sAxisDeltaPulse[i] + sNewSegPulse;
				//脉冲之和
				if (nPulseNum > 32767 || nPulseNum < -32768) 
				{
        			parseRunningError(9999, GCODE_ERROR_NUM152, 9999); 
					return FALSE;
    			} 
				pSRun->sAxisDeltaPulse[i] += sNewSegPulse;
			}

			pSRun->SAbsCoord.dArray[i] = pdTargetPoint[i];//将目标点赋值到 SAbsCoord  改变实时点的位置时统一处理
		}
		else
		{
			//没有使能螺距补偿

			//根据目标点和当前点计算脉冲当量
			//pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm);
			if (calcPulseWithCheck(pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i], dPulseToMm, &pSRun->sAxisDeltaPulse[i]) != 1)
			{
				return FALSE;
            }
			//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
			pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm;
		}
	}

	//改变实时点的位置
	updateRealTimePoint((double*)&pSRun->SAbsCoord, AXIS_NUM_MAX, ucAxisMap, ucValStatus);
	return TRUE;
}

/*************************************************
  函数名称：outPutAbsPointWithScrewComp
  函数功能：利用理论坐标和螺距补偿生成SRun结构体数据
  输入参数：ucPhysicalAxis	物理轴号
  输出参数：g_SScrewComp 螺距补偿结构体
  返 回 值：修正后的实时点
  备    注：
**************************************************/
double outPutAbsPointWithScrewCompForG0(struct _SRun* pSRun, double dTargetPoint, uint8 ucAxis, const uint8* ucAxisMap, double dPulseToMm)
{
	double dSum;
	uint8 ucPhysicalAxis;
	int16 sTargetSeg; // 目标的区间
	ucPhysicalAxis = ucAxisMap[ucAxis];
	double dAbsCoord;
	//获取当前点的坐标
	getSingleAxisRealTimePoint(&pSRun->SLastAbsCoord.dArray[ucAxis], ucAxis, ucAxisMap);

	if (g_SScrewComp[ucPhysicalAxis].bEnabled && g_SSystemPara.ScrewCompGap[ucPhysicalAxis] != 0)//开启了螺距补偿且补偿间隔不能为0
	{
		//目标点的区间
		sTargetSeg = (int)(dTargetPoint) / g_SSystemPara.ScrewCompGap[ucPhysicalAxis] + g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis];
		
		if (dTargetPoint >= 0)//正数的区间 整体移动一个位置  // zxr: 根据目标坐标区分正负区间，仅零点及正区间的段号加1，避免负区间选错补偿值
		{
			sTargetSeg += 1;
		}

		if (sTargetSeg == g_SScrewComp[ucPhysicalAxis].sCurSeg)
		{	//①区间相同
			dSum = (dTargetPoint - pSRun->SLastAbsCoord.dArray[ucAxis]) * (1 + g_SScrewComp[ucPhysicalAxis].dDeltaError) + g_SScrewComp[ucPhysicalAxis].dRecorder;

			//根据目标点和当前点计算脉冲当量
			//pSRun->sAxisDeltaPulse[ucAxis] = round(dSum / dPulseToMm);
			calcPulseWithCheck(dSum,dPulseToMm,&pSRun->sAxisDeltaPulse[ucAxis]);
			//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
			g_SScrewComp[ucPhysicalAxis].dRecorder = dSum - pSRun->sAxisDeltaPulse[ucAxis] * dPulseToMm;
		}
		else
		{	//②区间不同    默认最多跨越一个区间
			double dDivisionPoint;
			int16 sNewSegPulse;

			//分割点
			if (sTargetSeg > g_SScrewComp[ucPhysicalAxis].sCurSeg)//反向 
			{
				dDivisionPoint = (g_SScrewComp[ucPhysicalAxis].sCurSeg - g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis]) * (int)g_SSystemPara.ScrewCompGap[ucPhysicalAxis];
			}
			else //正向
			{
				dDivisionPoint = (sTargetSeg - g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis]) * g_SSystemPara.ScrewCompGap[ucPhysicalAxis];
			}

			//计算起点到分割点
			dSum = (dDivisionPoint - pSRun->SLastAbsCoord.dArray[ucAxis]) * (1 + g_SScrewComp[ucPhysicalAxis].dDeltaError) + g_SScrewComp[ucPhysicalAxis].dRecorder;

			//根据分割点和当前点计算脉冲当量
			//pSRun->sAxisDeltaPulse[ucAxis] = round(dSum / dPulseToMm);
			calcPulseWithCheck(dSum,dPulseToMm,&pSRun->sAxisDeltaPulse[ucAxis]);
			//判断区间是否在数组范围内
			if (sTargetSeg < SCREWDEVIATION_NUM_MAX && sTargetSeg >= 0)
			{
				//加载新区间的参数
				g_SScrewComp[ucPhysicalAxis].sCurSeg = sTargetSeg;
				g_SScrewComp[ucPhysicalAxis].cCurError = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
				g_SScrewComp[ucPhysicalAxis].dDeltaError = (double)g_SScrewComp[ucPhysicalAxis].cCurError / (g_SSystemPara.ScrewCompGap[ucPhysicalAxis] * 1000);
			}
			else
			{	//区间不存在   退化成无螺距补偿
				g_SScrewComp[ucPhysicalAxis].sCurSeg = sTargetSeg;
				g_SScrewComp[ucPhysicalAxis].cCurError = 0;
				g_SScrewComp[ucPhysicalAxis].dDeltaError = 0;
			}

			//计算分割点到目标点
			dSum = (dTargetPoint - dDivisionPoint) * (1 + g_SScrewComp[ucPhysicalAxis].dDeltaError);

			//根据目标点和分割点计算脉冲当量
			//sNewSegPulse = round(dSum / dPulseToMm);
			calcPulseWithCheck(dSum,dPulseToMm,&sNewSegPulse);
			//计算记录器的值
			g_SScrewComp[ucPhysicalAxis].dRecorder = dSum - sNewSegPulse * dPulseToMm;

			//脉冲之和
			int32 nPulseNum = pSRun->sAxisDeltaPulse[ucAxis] + sNewSegPulse;
			//脉冲之和
			if (nPulseNum > 32767 || nPulseNum < -32768) 
			{
        		parseRunningError(9999, GCODE_ERROR_NUM152, 9999); 
				return -1;
    		} 
			pSRun->sAxisDeltaPulse[ucAxis] += sNewSegPulse;
		}

		dAbsCoord = dTargetPoint;//将目标点赋值到 SAbsCoord  改变实时点的位置时统一处理
	}
	else
	{
		//没有使能螺距补偿

			//根据目标点和当前点计算脉冲当量
		//pSRun->sAxisDeltaPulse[ucAxis] = round((dTargetPoint - pSRun->SLastAbsCoord.dArray[ucAxis]) / dPulseToMm);
		calcPulseWithCheck(dTargetPoint - pSRun->SLastAbsCoord.dArray[ucAxis],dPulseToMm,&pSRun->sAxisDeltaPulse[ucAxis]);
			//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
		dAbsCoord = pSRun->SLastAbsCoord.dArray[ucAxis] + pSRun->sAxisDeltaPulse[ucAxis] * dPulseToMm;
	}
	return dAbsCoord;
}

#if SCREWCOMPTESTCODE

void Vs_ScrewCompLoadPara(uint8 ucPhysicalAxis)
{
	g_SSystemPara.bScrewCompEnable[ucPhysicalAxis] = TRUE;
	g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][9] = 8; //8um
	g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][10] = 5; //5um
	g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][11] = 4; //5um
	g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][12] = 8; //8um
	g_SSystemPara.ScrewCompGap[ucPhysicalAxis] = 10; //10mm
	g_SSystemPara.sScrewCompHomeNum[ucPhysicalAxis] = 10;
}
#endif // ScrewCpmpTestCode


//双向螺距补偿
SBilateralScrewComp g_SBilateralScrewComp[AXIS_PHYSICS_NUM_MAX];






/*************************************************
  函数名称：initSBilateralScrewComp(uint8 ucPhysicalAxis, int8 cDir)
  函数功能：初始化双向螺距   需赋方向
  输入参数：ucPhysicalAxis	物理轴号
			cDir			方向
  输出参数：
  返 回 值：-1 输入物理轴超出最大值
			-2 螺距补偿补偿功能未开启
			1  初始化成功
  备    注：
**************************************************/
int8 initSBilateralScrewComp(uint8 ucPhysicalAxis, int8 cDir)
{
	int16 sSegNum;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.bBilateralScrewEnable[ucPhysicalAxis] != TRUE)
	{
		return -2;
	}
	//获取区间号
	sSegNum = getSegNum(g_dRealTimePointArray[ucPhysicalAxis], ucPhysicalAxis);
	//获取补偿值（相对+绝对）
	getCompVal(sSegNum, ucPhysicalAxis, &g_SBilateralScrewComp[ucPhysicalAxis]);

	//根据方向计算当前点的绝对补偿值
	g_SBilateralScrewComp[ucPhysicalAxis].cMoveDir = cDir;
	cDir = (cDir == 1) ? SCREWCOMP_AHEAD : SCREWCOMP_BACK;
	g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal = g_SBilateralScrewComp[ucPhysicalAxis].sAbsCompVal[cDir] / 1000.0 + //分割点的补偿绝对值
		(g_dRealTimePointArray[ucPhysicalAxis] - g_SBilateralScrewComp[ucPhysicalAxis].dDivisionPoint) * g_SBilateralScrewComp[ucPhysicalAxis].dDeltaComp[cDir];//+相对值x每mm对应的补偿值
	g_SBilateralScrewComp[ucPhysicalAxis].fVel = g_SSystemPara.dRevClearanceCompVs;
	g_SBilateralScrewComp[ucPhysicalAxis].fAcc = g_SSystemPara.dRevClearanceCompAcc;
	g_SBilateralScrewComp[ucPhysicalAxis].dLimitVmax = g_SSystemPara.dRevClearanceCompVmax;	//2023/2/28	陈敏聪
	g_SBilateralScrewComp[ucPhysicalAxis].cDir = 1;
	g_SBilateralScrewComp[ucPhysicalAxis].ucStatus = BILATERAL_SCREW_ON;
	g_SBilateralScrewComp[ucPhysicalAxis].dRecorder = 0;  //初始化记录器  2023.3.24 朱稳中
	return 1;
}

/*************************************************
  函数名称：getSegNum(double curPoint, uint8 ucPhysicalAxis)
  函数功能：根据坐标获取区间号码
  输入参数：curPoint		坐标点
			ucPhysicalAxis	物理轴号
  输出参数：
  返 回 值：区间号码(int16)
  备    注：
**************************************************/
int16 getSegNum(double curPoint, uint8 ucPhysicalAxis)
{
	int16 sSeg;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	//目标点的区间
	sSeg = (int)((curPoint - g_SSystemPara.fScrewReferencePoint[ucPhysicalAxis]) / g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis]) + g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis];

	if (curPoint < g_SSystemPara.fScrewReferencePoint[ucPhysicalAxis])//负数的区间 整体移动一个位置
	{
		sSeg -= 1;
	}
	else {
		sSeg += 1;
	}
	return sSeg;
}

/*************************************************
  函数名称：getCompVal(int16 sSeg, uint8 ucPhysicalAxis, SBilateralScrewComp* pSScrewComp)
  函数功能：根据区间号码获取补偿数据
  输入参数：sSeg			区间
			ucPhysicalAxis	物理轴号
			pSScrewComp		双表螺距补偿指针
  输出参数：
  返 回 值：-1	参数错误
			1	完成
  备    注：
**************************************************/
int8 getCompVal(int16 sSeg, uint8 ucPhysicalAxis, SBilateralScrewComp* pSScrewComp)
{
	int16 curSeg;
	int16 sAbsCompVal[2] = { 0 };
	int16 sDir;
	int16 sTargetSeg = sSeg;


	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX || pSScrewComp == NULL)
	{
		return -1;
	}
	curSeg = g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis];

	sAbsCompVal[SCREWCOMP_AHEAD] = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][curSeg];//绝对值
	sAbsCompVal[SCREWCOMP_BACK] = g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][curSeg];//绝对值

	if (sTargetSeg >= g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis])
	{//右边  递增
		curSeg += 1;
		for (; (curSeg < sTargetSeg && curSeg <= SCREWDEVIATION_NUM_MAX - 1); curSeg++)
		{
			sAbsCompVal[SCREWCOMP_AHEAD] += g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][curSeg];//相对值
			sAbsCompVal[SCREWCOMP_BACK] += g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][curSeg];//相对值
		}
	}
	else
	{//左边  递减
		curSeg -= 1;
		for (; (curSeg > sTargetSeg && curSeg >= 0); curSeg--)
		{
			sAbsCompVal[SCREWCOMP_AHEAD] -= g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][curSeg];//相对值
			sAbsCompVal[SCREWCOMP_BACK] -= g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][curSeg];//相对值
		}
	}

	//传出数据  （绝对值）
	pSScrewComp->sAbsCompVal[SCREWCOMP_AHEAD] = sAbsCompVal[SCREWCOMP_AHEAD];
	pSScrewComp->sAbsCompVal[SCREWCOMP_BACK] = sAbsCompVal[SCREWCOMP_BACK];

	//获得相对值
	if (sTargetSeg < SCREWDEVIATION_NUM_MAX && sTargetSeg >= 0)
	{//有效区间内

		pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
		pSScrewComp->dDeltaComp[SCREWCOMP_BACK] = g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][sTargetSeg];
	}
	else
	{
		//有效区间外  当前区间的补偿值为0
		pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] = pSScrewComp->dDeltaComp[SCREWCOMP_BACK] = 0;
	}

	//计算每1mm对应的补偿值
	pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] /= ((double)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] * 1000);
	pSScrewComp->dDeltaComp[SCREWCOMP_BACK] /= ((double)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] * 1000);
	
	//赋值区间号
	pSScrewComp->sCurSeg = sTargetSeg;
	
	//计算分割点
	pSScrewComp->dDivisionPoint = (sTargetSeg >= g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis]) ?
		(sTargetSeg - 1 - g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis]) * g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] + g_SSystemPara.fScrewReferencePoint[ucPhysicalAxis] :
		(sTargetSeg + 1 - g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis]) * g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] + g_SSystemPara.fScrewReferencePoint[ucPhysicalAxis];
	return 1;
}


/*************************************************
  函数名称：moveScrewSeg(int16 sTargetSeg, uint8 ucPhysicalAxis, SBilateralScrewComp* pSScrewComp)
  函数功能：移动区间，获取补偿数据
  输入参数：sTargetSeg		目标区间
			ucPhysicalAxis	物理轴号
			pSScrewComp		双表螺距补偿指针
  输出参数：
  返 回 值：-1	参数错误
			1	完成
  备    注：
**************************************************/
int8 moveScrewSeg(int16 sTargetSeg, uint8 ucPhysicalAxis, SBilateralScrewComp* pSScrewComp)
{
	int16 sDir;
	sDir = sTargetSeg - pSScrewComp->sCurSeg;
	int16 sReferenceNum;

	if (sDir == 0)
	{
		return 0;
	}
	//参考点号码
	sReferenceNum = g_SSystemPara.sScrewReferenceNum[ucPhysicalAxis];

	if (sTargetSeg == sReferenceNum - 1 || sTargetSeg == sReferenceNum + 1)
	{//特殊区间 直接得出结果

		pSScrewComp->sAbsCompVal[SCREWCOMP_AHEAD] = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sReferenceNum];
		pSScrewComp->sAbsCompVal[SCREWCOMP_BACK] = g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][sReferenceNum];

		pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
		pSScrewComp->dDeltaComp[SCREWCOMP_BACK] = g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][sTargetSeg];
		//计算每1mm补偿的脉冲值
		pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] /= ((double)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] * 1000);
		pSScrewComp->dDeltaComp[SCREWCOMP_BACK] /= ((double)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] * 1000);

		//区间号
		pSScrewComp->sCurSeg = sTargetSeg;
		//分割点
		pSScrewComp->dDivisionPoint = g_SSystemPara.fScrewReferencePoint[ucPhysicalAxis];

		return 2;
	}

	if (sDir >= 2 || sDir <= -2)
	{//跨度较大 使用迭代重新计算
		return getCompVal(sTargetSeg, ucPhysicalAxis, pSScrewComp);
	}

	if (sTargetSeg >= SCREWDEVIATION_NUM_MAX || sTargetSeg < 0)
	{ //目标区间越界

		//补偿绝对值
		//pSScrewComp->sAbsCompVal[SCREWCOMP_AHEAD] = pSScrewComp->sAbsCompVal[SCREWCOMP_AHEAD];
		//pSScrewComp->sAbsCompVal[SCREWCOMP_BACK] = pSScrewComp->sAbsCompVal[SCREWCOMP_BACK];

		//每1mm补偿的脉冲值
		pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] = 0;
		pSScrewComp->dDeltaComp[SCREWCOMP_BACK] = 0;

		//区间号
		pSScrewComp->sCurSeg = sTargetSeg;
		//分割点
		pSScrewComp->dDivisionPoint += sDir * (int32)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis];
		return 1;
	}

	//目标区间没有越界

	//存放的是相对值  累加
	if (sDir * (sTargetSeg - sReferenceNum) > 0)
	{
		pSScrewComp->sAbsCompVal[SCREWCOMP_AHEAD] += sDir * g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][pSScrewComp->sCurSeg];
		pSScrewComp->sAbsCompVal[SCREWCOMP_BACK] += sDir * g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][pSScrewComp->sCurSeg];
	}
	else
	{
		pSScrewComp->sAbsCompVal[SCREWCOMP_AHEAD] += sDir * g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
		pSScrewComp->sAbsCompVal[SCREWCOMP_BACK] += sDir * g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][sTargetSeg];
	}

	//获取当前区间相对值
	pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] = g_SSystemPara.cAheadScrewCompVal[ucPhysicalAxis][sTargetSeg];
	pSScrewComp->dDeltaComp[SCREWCOMP_BACK] = g_SSystemPara.cBackScrewCompVal[ucPhysicalAxis][sTargetSeg];

	//计算每1mm补偿的脉冲值
	pSScrewComp->dDeltaComp[SCREWCOMP_AHEAD] /= ((double)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] * 1000);
	pSScrewComp->dDeltaComp[SCREWCOMP_BACK] /= ((double)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis] * 1000);

	//区间号
	pSScrewComp->sCurSeg = sTargetSeg;
	//分割点
	pSScrewComp->dDivisionPoint += sDir * (int16)g_SSystemPara.ucBilateralScrewGap[ucPhysicalAxis];
	return 2;
}

/*************************************************
  函数名称：screwVelLimit(double  dTargetComp, SBilateralScrewComp* pSScrewComp)
  函数功能：补偿速度限制  （单周期补偿量过大时，进行速度限制，使用梯形曲线进行加减速）
  输入参数：dTargetComp		目标补偿值
			pSScrewComp		双表螺距补偿指针
  输出参数：
  返 回 值：-1	参数错误
			1	完成
  备    注：
**************************************************/
double screwVelLimit(double  dTargetComp, SBilateralScrewComp* pSScrewComp)
{
	float fDelta;
	float fDistLimtVel;//距离限制的速度
	double dRes;
	float fVs = g_SSystemPara.dRevClearanceCompVs;;
	if (pSScrewComp == NULL)
	{
		return dTargetComp;
	}
	//补偿相对值
	fDelta = (float)(dTargetComp - pSScrewComp->dCurAbsCompVal);

	if (fDelta * pSScrewComp->cDir < 0)// 方向相反  
	{
		pSScrewComp->fVel = g_SSystemPara.dRevClearanceCompVs;//方向改变  速度初始化为初速度
		pSScrewComp->cDir = sgn((double)fDelta);//赋值补偿方向
	}
	//补偿的绝对值
	fDelta = fabs(fDelta);

	if (fDelta < fVs)//速度没有超限  直接输出
	{
		pSScrewComp->ucStatus = BILATERAL_SCREW_ON;//状态改变为ON，因为补偿在这个周期可以全部输出完成
		return dTargetComp;
	}
	//距离限制的速度
	fDistLimtVel = sqrt(2 * pSScrewComp->fAcc * (float)fDelta + fVs * fVs);
	//计算速度
	pSScrewComp->fVel = min(pSScrewComp->fVel, fDelta);
	pSScrewComp->fVel = min(pSScrewComp->fVel, pSScrewComp->dLimitVmax);	//2023.2.28	陈敏聪
	pSScrewComp->fVel = min(pSScrewComp->fVel, fDistLimtVel);

	//输出当前补偿 + 速度
	dRes = pSScrewComp->dCurAbsCompVal + pSScrewComp->cDir * pSScrewComp->fVel;
	//梯形速度曲线 下个周期允许的速度增加一个加速度
	pSScrewComp->fVel += pSScrewComp->fAcc * g_SSystemPara.dInterpolationTime;
	//将状态改变为补偿活跃 （补偿的量未输出完成）
	pSScrewComp->ucStatus = BILATERAL_SCREW_ACTIVE;
	return dRes;
}



///*************************************************
//  函数名称：outPutAbsPointWithScrewComp
//  函数功能：利用理论坐标和螺距补偿生成SRun结构体数据
//  输入参数：ucPhysicalAxis	物理轴号
//  输出参数：g_SScrewComp 螺距补偿结构体
//  返 回 值：-1 输入物理轴超出最大值， 0 螺距补偿补偿功能未开启，1 初始化成功
//  备    注：
//**************************************************/
//int8 outPutAbsPointWithBilateralScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
//{
//	int i = 0;
//	double dPulseToMm;
//	double dSum;
//	uint8 ucPhysicalAxis;
//	int16 sTargetSeg; // 目标的区间
//
//	if (pSRun == NULL)//空指针
//	{
//		return FALSE;
//	}
//
//	//获取实时点作为上一点的位置
//	getRealTimePoint((double*)&pSRun->SLastAbsCoord, AXIS_NUM_MAX, ucAxisMap, NULL);
//
//	for (i = 0; i < nArraySize; i++)
//	{
//		ucPhysicalAxis = ucAxisMap[i];
//
//		if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX || (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))) continue;  //映射为空的物理轴  不计算
//		//目标轴的脉冲当量
//		dPulseToMm = (dPulseToMmArray == NULL) ? g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm : dPulseToMmArray[i];
//
//		if (g_SBilateralScrewComp[ucPhysicalAxis].ucStatus)//开启了螺距补偿
//		{
//			double dCurCompVal;//当前补偿绝对值
//			//目标点的区间
//			sTargetSeg = getSegNum(pdTargetPoint[i], ucPhysicalAxis);
//
//			//移动区间（区间相同时，内部不操作）
//			moveScrewSeg(sTargetSeg, ucPhysicalAxis, &g_SBilateralScrewComp[ucPhysicalAxis]);
//
//			//根据方向  计算补偿绝对值
//			if (pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i] >0)
//			{//正向   计算正向的补偿绝对值
//				dCurCompVal =g_SBilateralScrewComp[ucPhysicalAxis].sAbsCompVal[SCREWCOMP_AHEAD]/1000.0  + //分割点的补偿绝对值
//					(pdTargetPoint[i] - g_SBilateralScrewComp[ucPhysicalAxis].dDivisionPoint) * g_SBilateralScrewComp[ucPhysicalAxis].dDeltaComp[SCREWCOMP_AHEAD];//+相对值x每mm对应的补偿值
//			}
//			else
//			{//反向   计算反向的补偿绝对值
//				dCurCompVal = g_SBilateralScrewComp[ucPhysicalAxis].sAbsCompVal[SCREWCOMP_BACK]/1000.0 + //分割点的补偿绝对值
//					(pdTargetPoint[i] - g_SBilateralScrewComp[ucPhysicalAxis].dDivisionPoint) * g_SBilateralScrewComp[ucPhysicalAxis].dDeltaComp[SCREWCOMP_BACK];//+相对值x每mm对应的补偿值
//			}
//
//			//补偿速度限制判断
//			dCurCompVal = screwVelLimit(dCurCompVal, &g_SBilateralScrewComp[ucPhysicalAxis]);
//
//			//计算总距离
//			dSum = (pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) + (dCurCompVal - g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal) + g_SBilateralScrewComp[ucPhysicalAxis].dRecorder;
//			
//			//计算脉冲
//			pSRun->sAxisDeltaPulse[i] = round(dSum / dPulseToMm);
//
//			//记录补偿的余值
//			g_SBilateralScrewComp[ucPhysicalAxis].dRecorder = dSum - pSRun->sAxisDeltaPulse[i] * dPulseToMm;
//
//			//记录补偿绝对值
//			g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal = dCurCompVal;
//
//			printf("abs = %lf ", g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal);
//
//			pSRun->SAbsCoord.dArray[i] = pdTargetPoint[i];//将目标点赋值到 SAbsCoord  改变实时点的位置时统一处理
//		}
//		else
//		{
//			//没有使能螺距补偿
//
//			//根据目标点和当前点计算脉冲当量
//			pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm);
//
//			//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
//			pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm;
//		}
//	}
//
//	//改变实时点的位置
//	updateRealTimePoint((double*)&pSRun->SAbsCoord, AXIS_NUM_MAX, ucAxisMap, ucValStatus);
//	return TRUE;
//}


/*************************************************
  函数名称：outPutAbsPointWithBilateralScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
  函数功能：利用理论坐标和螺距补偿生成SRun结构体数据
  输入参数：pdTargetPoint	目标点数组
			nArraySize		目标点数组大小
			ucAxisMap		映射
			dPulseToMmArray	脉冲当量
			ucValStatus		有效值
  输出参数：pSRun 运行结构体指针
  返 回 值：FALSE	空指针
			TRUE	完成
  备    注：
**************************************************/
int8 outPutAbsPointWithBilateralScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;
	double dPulseToMm;
	double dSum;
	uint8 ucPhysicalAxis;
	int16 sTargetSeg; // 目标的区间

	if (pSRun == NULL)//空指针
	{
		return FALSE;
	}

	//获取实时点作为上一点的位置
	getRealTimePoint((double*)&pSRun->SLastAbsCoord, AXIS_NUM_MAX, ucAxisMap, NULL);

	pSRun->ucValStatus = *ucValStatus;

	for (i = 0; i < nArraySize; i++)
	{
		ucPhysicalAxis = ucAxisMap[i];

		if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX || (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))) continue;  //映射为空的物理轴  不计算
		//目标轴的脉冲当量
		dPulseToMm = (dPulseToMmArray == NULL) ? g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm : dPulseToMmArray[i];

		if (g_SBilateralScrewComp[ucPhysicalAxis].ucStatus)//开启了螺距补偿
		{

			pSRun->SAbsCoord.dArray[i] = pdTargetPoint[i];//将目标点赋值到 SAbsCoord  改变实时点的位置时统一处理
		}
		else
		{
			//没有使能螺距补偿

			//根据目标点和当前点计算脉冲当量
			pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm);
			calcPulseWithCheck(pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i],g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm,&pSRun->sAxisDeltaPulse[i]);
			//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
			pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm;
		}
	}

	//改变实时点的位置
	updateRealTimePoint((double*)&pSRun->SAbsCoord, AXIS_NUM_MAX, ucAxisMap, ucValStatus);
	return TRUE;
}

/*************************************************
  函数名称：outPutAbsPointWithBilateralScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
  函数功能：利用理论坐标和螺距补偿生成SRun结构体数据
  输入参数：pdTargetPoint	目标点
			ucAxisMap		映射
			dPulseToMm		脉冲当量
  输出参数：pSRun 运行结构体指针
  返 回 值：修正后的实时点
  备    注：
**************************************************/
double outPutAbsPointWithBilateralScrewCompForG0(struct _SRun* pSRun, double dTargetPoint, uint8 ucAxis, const uint8* ucAxisMap, double dPulseToMm)
{
	double dSum;
	uint8 ucPhysicalAxis;
	int16 sTargetSeg; // 目标的区间
	ucPhysicalAxis = ucAxisMap[ucAxis];
	double dAbsCoord;

	getSingleAxisRealTimePoint(&pSRun->SLastAbsCoord.dArray[ucAxis], ucAxis, ucAxisMap);

	if (g_SBilateralScrewComp[ucPhysicalAxis].ucStatus)//开启了螺距补偿
	{
		pSRun->SAbsCoord.dArray[ucAxis] = dTargetPoint;//将目标点赋值到 SAbsCoord  改变实时点的位置时统一处理
		dAbsCoord = dTargetPoint;
		pSRun->ucValStatus |= 0x01 << ucAxis;//有效值
	}
	else
	{
		//没有使能螺距补偿

		//根据目标点和当前点计算脉冲当量
		pSRun->sAxisDeltaPulse[ucAxis] = round((dTargetPoint - pSRun->SLastAbsCoord.dArray[ucAxis]) / dPulseToMm);
		calcPulseWithCheck(dTargetPoint - pSRun->SLastAbsCoord.dArray[ucAxis],dPulseToMm,&pSRun->sAxisDeltaPulse[ucAxis]);
		//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
		dAbsCoord = pSRun->SLastAbsCoord.dArray[ucAxis] + pSRun->sAxisDeltaPulse[ucAxis] * dPulseToMm;
	}
	return dAbsCoord;
}

/*************************************************
  函数名称：BilateralScrewComp(struct _SRun* pSRun, int nArraySize, const uint8* ucAxisMap)
  函数功能：根据SRUN结构体数据计算补偿值和生成脉冲
  输入参数：pSRun 运行结构体指针
			nArraySize		数组大小
			ucAxisMap		映射
  输出参数：pSRun 运行结构体指针
  返 回 值：FALSE	空指针
			TRUE	完成
  备    注：
**************************************************/
int8 BilateralScrewComp(struct _SRun* pSRun, int nArraySize, const uint8* ucAxisMap)
{
	int i;
	double dSum;
	uint8 ucPhysicalAxis;
	int16 sTargetSeg; // 目标的区间
	double dPulseToMm;
	double dDist;
	for (i = 0; i < nArraySize; i++)
	{
		double dCurCompVal;//当前补偿绝对值
		ucPhysicalAxis = ucAxisMap[i];
		if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX) continue;  //映射为空的物理轴  不计算
		if (g_SBilateralScrewComp[ucPhysicalAxis].ucStatus != BILATERAL_SCREW_ACTIVE)
		{//非补偿活跃状态下，判断有效值
			if (VAL_VALID != getbit(pSRun->ucValStatus, i))
			{
				continue;
			}
		}
		if (g_SBilateralScrewComp[ucPhysicalAxis].ucStatus == BILATERAL_SCREW_OFF)
		{//补偿未开启，跳过
			continue;
		}
		//脉冲当量
		dPulseToMm = g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm;
		//目标点的区间
		sTargetSeg = getSegNum(pSRun->SAbsCoord.dArray[i], ucPhysicalAxis);

		//移动区间（区间相同时，内部不操作）
		moveScrewSeg(sTargetSeg, ucPhysicalAxis, &g_SBilateralScrewComp[ucPhysicalAxis]);

		dDist = pSRun->SAbsCoord.dArray[i] - pSRun->SLastAbsCoord.dArray[i];
		if (dDist * g_SBilateralScrewComp[ucPhysicalAxis].cMoveDir < 0)
		{
			g_SBilateralScrewComp[ucPhysicalAxis].cMoveDir = sgn(dDist);
		}
		//根据方向  计算补偿绝对值
		if (g_SBilateralScrewComp[ucPhysicalAxis].cMoveDir > 0)
		{//正向   计算正向的补偿绝对值
			dCurCompVal = g_SBilateralScrewComp[ucPhysicalAxis].sAbsCompVal[SCREWCOMP_AHEAD] / 1000.0 + //分割点的补偿绝对值
				(pSRun->SAbsCoord.dArray[i] - g_SBilateralScrewComp[ucPhysicalAxis].dDivisionPoint) * g_SBilateralScrewComp[ucPhysicalAxis].dDeltaComp[SCREWCOMP_AHEAD];//+相对值x每mm对应的补偿值
		}
		else
		{//反向   计算反向的补偿绝对值
			dCurCompVal = g_SBilateralScrewComp[ucPhysicalAxis].sAbsCompVal[SCREWCOMP_BACK] / 1000.0 + //分割点的补偿绝对值
				(pSRun->SAbsCoord.dArray[i] - g_SBilateralScrewComp[ucPhysicalAxis].dDivisionPoint) * g_SBilateralScrewComp[ucPhysicalAxis].dDeltaComp[SCREWCOMP_BACK];//+相对值x每mm对应的补偿值
		}

		//补偿速度限制判断
		dCurCompVal = screwVelLimit(dCurCompVal, &g_SBilateralScrewComp[ucPhysicalAxis]);

		//计算总距离
		dSum = (pSRun->SAbsCoord.dArray[i] - pSRun->SLastAbsCoord.dArray[i]) + (dCurCompVal - g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal) + g_SBilateralScrewComp[ucPhysicalAxis].dRecorder;

		//计算脉冲
		//pSRun->sAxisDeltaPulse[i] = round(dSum / dPulseToMm);
		calcPulseWithCheck(dSum,dPulseToMm,&pSRun->sAxisDeltaPulse[i]);
		//记录补偿的余值
		g_SBilateralScrewComp[ucPhysicalAxis].dRecorder = dSum - pSRun->sAxisDeltaPulse[i] * dPulseToMm;

		//记录补偿绝对值
		g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal = dCurCompVal;
#if ABSCOMPVAL_PRINT
		printf("abs = %lf ", g_SBilateralScrewComp[ucPhysicalAxis].dCurAbsCompVal);
#endif // ABSCOMPVAL_PRINT
		pSRun->SLastAbsCoord.dArray[i] = pSRun->SAbsCoord.dArray[i];
	}

	pSRun->ucValStatus = 0;
	return 1;
}

/*************************************************
  函数名称：initSBilateralScrewCompAfterHomeAPI(uint8 ucChannelNum ,uint8 ucAxis , int8 cDir)
  函数功能  初始化单轴反向间隙补偿接口函数
  输入参数：ucChannelNum	通道号
			ucAxis			轴号
  输出参数：
  返 回 值：-2 错误的方向
			-1 通道号或轴号错误
			 1 成功
			 2 成功 待生效
  备    注：
**************************************************/
int8 initSBilateralScrewCompAfterHomeAPI(uint8 ucChannelNum, uint8 ucAxis, int8 cDir)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX || g_SChannel->pMapAxisArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	return initSBilateralScrewComp(g_SChannel[ucChannelNum].pMapAxisArray[ucAxis], cDir);
}


/*************************************************
  函数名称：bilateralScrewCompDisableAPI(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  取消双表补偿
  输入参数：ucChannelNum	通道号
			ucAxis			轴号
  输出参数：
  返 回 值：-2 错误的映射
			-1 通道号或轴号错误
			 1 成功
  备    注：
**************************************************/
int8 bilateralScrewCompDisableAPI(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	if (g_SChannel[ucChannelNum].pMapAxisArray[ucAxis] == AXIS_NULL)
	{
		return -2;
	}

	//g_SBilateralScrewComp[g_SChannel[ucChannelNum].pMapAxisArray[ucAxis]].ucStatus = BILATERAL_SCREW_ON;
	g_SBilateralScrewComp[g_SChannel[ucChannelNum].pMapAxisArray[ucAxis]].ucStatus = BILATERAL_SCREW_OFF;//206.5.6 修改状态为取消双表补偿，对应函数接口功能
	return 1;
}
