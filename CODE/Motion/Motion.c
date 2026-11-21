#include "Motion.h"
#include <stdio.h>
#include "G33.h"
#include "G32.h"
#include "G0.h"
#include "G1.h"
#include "IOCommand.h"
#include "GeneralFunctions.h"
#include <stdlib.h>
#include "Spindle.h"
#include "toolAxis.h"
#include "CTrajPlanning.h"
#include "Gcode.h"
#include "G2.h"
#include "Channel.h"
#include "ringbuff.h"
#include "Config.h"
#include "ErrorCompensation.h"
#include <string.h>
#include "SecondSpindle.h"


#if STM32_MODE
#include "dir.h"
#endif
//struct SInterpolationPre g_SPreProcess;

#define EMPTYBUF_TIMEOUTVALUE 10000 // 空读计数阈值
//超前6个CNT时的数组const    //修改将企业库移植到学校库
const uint16 usTimeComp[1002]={0,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,20,17,16,26,22,27,16,16,16,29,26,16,26,34,22,16,26,26,38,19,36,16,16,27,26,40,22,16,50,45,38,26,16,26,34,27,56,16,54,17,16,33,26,47,34,47,22,21,44,24,26,50,26,22,38,74,56,56,74,33,16,78,16,29,68,40,26,26,40,68,22,77,56,47,50,65,92,36,86,51,26,16,16,16,26,51,86,26,80,37,110,84,66,56,54,60,74,96,16,47,92,26,86,33,108,68,34,16,110,95,86,83,86,95,110,131,24,56,94,138,50,105,26,92,22,99,38,126,74,26,130,91,56,25,150,128,110,96,86,80,78,80,86,96,110,128,150,16,40,73,110,151,26,74,126,16,68,131,22,92,166,65,146,50,138,47,142,56,158,77,186,110,36,155,86,19,148,86,26,165,110,57,16,158,112,68,26,191,154,119,86,55,26,210,186,164,144,126,110,96,84,74,66,60,56,54,54,56,60,66,74,84,96,110,126,144,164,186,210,236,26,55,86,119,154,191,230,26,68,112,158,206,16,57,110,165,222,26,86,148,212,19,86,155,226,36,110,186,264,77,158,241,56,142,230,47,138,231,50,146,244,65,166,269,92,198,22,131,242,68,182,16,126,245,74,196,26,151,278,110,240,73,206,40,176,16,150,291,128,272,110,257,96,246,86,239,80,236,78,237,80,242,86,251,96,264,110,281,128,302,150,327,176,25,206,56,240,91,278,130,320,173,26,220,74,271,126,326,182,38,242,99,306,164,22,233,92,306,166,26,244,105,326,188,50,275,138,366,230,94,326,191,56,292,158,24,264,131,374,242,110,357,226,95,346,216,86,341,212,83,342,214,86,349,222,95,362,236,110,381,256,131,16,282,158,34,314,191,68,352,230,108,396,275,154,33,326,206,86,383,264,145,26,328,210,92,398,281,164,47,358,242,126,16,326,211,96,416,302,188,74,399,286,173,60,390,278,166,54,389,278,167,56,396,286,176,66,411,302,193,84,434,326,218,110,465,358,251,144,37,398,292,186,80,446,341,236,131,26,398,294,190,86,463,360,257,154,51,434,332,230,128,26,415,314,213,112,16,406,306,206,106,16,407,308,209,110,16,418,320,222,124,26,439,342,245,148,51,470,374,278,182,86,511,416,321,226,131,36,468,374,280,186,92,530,437,344,251,158,65,510,418,326,234,142,50,502,411,320,229,138,47,506,416,326,236,146,56,522,433,344,255,166,77,550,462,374,286,198,110,22,503,416,329,242,155,68,556,470,384,298,212,126,40,536,451,366,281,196,111,26,530,446,362,278,194,110,26,538,455,372,289,206,123,40,560,478,396,314,232,150,68,596,515,434,353,272,191,110,29,566,486,406,326,246,166,86,16,552,473,394,315,236,157,78,632,554,476,398,320,242,164,86,16,572,495,418,341,264,187,110,33,606,530,454,378,302,226,150,74,656,581,506,431,356,281,206,131,56,648,574,500,426,352,278,204,130,56,658,585,512,439,366,293,220,147,74,686,614,542,470,398,326,254,182,110,38,661,590,519,448,377,306,235,164,93,22,656,586,516,446,376,306,236,166,96,26,671,602,533,464,395,326,257,188,119,50,706,638,570,502,434,366,298,230,162,94,26,694,627,560,493,426,359,292,225,158,91,24,704,638,572,506,440,374,308,242,176,110,44,736,671,606,541,476,411,346,281,216,151,86,21,726,662,598,534,470,406,342,278,214,150,86,22,740,677,614,551,488,425,362,299,236,173,110,47,778,716,654,592,530,468,406,344,282,220,158,96,34,779,718,657,596,535,474,413,352,291,230,169,108,47,806,746,686,626,566,506,446,386,326,266,206,146,86,26,800,741,682,623,564,505,446,387,328,269,210,151,92,33,822,764,706,648,590,532,474,416,358,300,242,184,126,68,16,815,758,701,644,587,530,473,416,359,302,245,188,131,74,17,838,782,726,670,614,558,502,446,390,334,278,222,166,110,54,891,836,781,726,671,616,561,506,451,396,341,286,231,176,121,66,16,866,812,758,704,650,596,542,488,434,380,326,272,218,164,110,56,928,875,822,769,716,663,610,557,504,451,398,345,292,239,186,133,80,27,918,866,814,762,710,658,606,554,502,450,398,346,294,242,190,138,86,34,944,893,842,791,740,689,638,587,536,485,434,383,332,281,230,179,128,77,26,956,906,856,806,756,706,656,606,556,506,456,406,356,306,256,206,156,106,56,16,};


/*************************************************
  函数名称：interpolationInit()
  函数功能  插补初始化，初始化与插补有关的变量，适用于系统开机/新一段程序
  输入参数：
  输出参数：
  返 回 值：TRUE 初始化完成
  备    注：
**************************************************/
bool interpolationInit(struct SInterpolationPre* pSPreProcess)
{
	struct SChannel* pSChannel;
	int i = 0;
	if (pSPreProcess == NULL)
	{
		return FALSE;
	}

	pSChannel = pSPreProcess->pSChannel;

	memset(pSPreProcess->cOriginFilename, '\0', sizeof(pSPreProcess->cOriginFilename));	//陈敏聪	//2023/9/6

	//将主轴转速置为无效
	//pSPreProcess->dSpindleSpeed = pSPreProcess->dLimitedSpindleSpeed = -1;
	pSPreProcess->pSChannel->SSpindle.dSpindleSpeed = pSPreProcess->pSChannel->SSpindle.dSpindleLineSpeed = pSPreProcess->pSChannel->SSpindle.dLimitedSpindleSpeed = -1;
	pSPreProcess->dSecondSpindleSpeed = -1;
	pSPreProcess->ucInpStatus = INP_FINISHED;
	pSPreProcess->cChannelBufNum = -1;
	pSPreProcess->sChannelCommand.ucType = UNASSIGNED_VALUE;
	pSPreProcess->cInterpDir = INP_GO_AHEAD;

	InitG33GlobalVariable(pSPreProcess->pSChannel);
	for (i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
	{
		pSPreProcess->ucIOCommandArray[i] = 0;
	}
	////清除所有指令的规划参数
	//for (i = 0; i < AXIS_NUM_MAX; i++)
	//{
	//	//if (i == AXIS_C && g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[i]].ucAxisType == ROTATION_AXIS)continue;//2027/7/8 主轴速度参数不清除
	//	clearSPlanData(&pSChannel->SSingleAxisArray[i].SPlanData);
	//}
	//clearSPlanData(&pSChannel->SG01.SPlanData);
	//clearSPlanData(&pSChannel->SG02.SPlanData);

	// G0当前标志置完成
	pSChannel->SG00.ucCurrentSigned = pSChannel->SG00.ucFinishedSigned;
	pSChannel->SG00.ucRunStep = 5;					//初始化G0123状态 朱稳中 2023.3.24
	pSChannel->SG01.ucInpStatus = INP_FINISHED;
	pSChannel->SG02.ucInpStatus = INP_FINISHED;
	pSChannel->dLessChipBreakLPreDist = 0; //清除不足断屑间距的累计量
	return TRUE;
}

/*************************************************
  函数名称：interpolationInitAfterLoad(struct _SRun *pSRun,struct SInterpolationPre *pSPreProcess)
  函数功能  插补在系统参数加载之后的初始化
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 interpolationInitAfterLoad(struct _SRun* pSRun, struct SInterpolationPre* pSPreProcess)
{
	uint8 ucToolNum;
	uint8 ucToolCompNum;
	ucToolNum = g_SSystemPara.ucToolNum;
	ucToolCompNum = g_SSystemPara.ucToolCompNum;
	if (ucToolNum < 1 || ucToolNum >TOOL_NUM_MAX)
	{
		g_SSystemPara.ucToolNum = ucToolNum = 1;
	}
	if (ucToolCompNum < 1 || ucToolCompNum >TOOL_NUM_MAX)
	{
		g_SSystemPara.ucToolCompNum = ucToolCompNum = 1;
	}
	pSPreProcess->ucToolNum = pSRun->ucToolNum = ucToolNum;
	pSPreProcess->ucToolCompNum = pSRun->ucToolCompNum = ucToolCompNum;
	pSPreProcess->pSChannel->STIOCommand.ucReadIO_Counter = pSPreProcess->pSChannel->STIOCommand.ucReadIO_Counter = 0;
	pSPreProcess->pSChannel->STIOCommand.ucToolNum = ucToolNum;
	pSPreProcess->pSChannel->STIOCommand.ucToolCompNum = ucToolCompNum;
	pSPreProcess->pSChannel->STIOCommand.ucToolChangeStatus = TOOLCHANGE_IDLE;
	pSPreProcess->pSChannel->STIOCommand.ucTIORun = 0;
	pSPreProcess->SWorkPieceOffsetByG50 = g_SZeroPtOfCoordSys->SWorkPieceOffsetByG50;//用系统参数初始化G50临时坐标系偏移量 2023-8-3洪冠鹏
	//不使用工件坐标  //2023/10/14 企业要求开机后默认使用G54
	pSPreProcess->cWorkpieceCoordNum = pSRun->cWorkpieceCoordNum = 54;
	//计算新的偏移坐标
	TChangeTotalOffset(pSPreProcess->pSChannel->ucChannelNum, pSRun->ucToolCompNum, pSRun->cWorkpieceCoordNum, &pSRun->SCurrentOffset, &pSRun->SWorkPieceOffsetByG50);




	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		pSPreProcess->pSChannel->SRelaCoorStartPoint.dArray[i] = getAbsCoordDouble(pSPreProcess->pSChannel->ucChannelNum, i);	//陈敏聪 2023/5/27 相对坐标开机初始化为当前点绝对坐标
	}

	//指令类型初始化
	pSPreProcess->unCommandType = pSRun->unCommandType = 0x0;
	setBits(pSRun->unCommandType, CMD_MOTION_QUICKMOVE, CMD_MOTION_ADDR);//开机默认G0	陈敏聪	2023/6/1
	setBits(pSRun->unCommandType, CMD_PLANE_G17, CMD_PLANE_ADDR);          //加工中心开机默认 G17，指定 XY 插补平面
	setBits(pSRun->unCommandType, CMD_TOOLCOMP_CANCEL, CMD_TOOLCOMP_ADDR);//开机默认取消刀补G40
	setBits(pSRun->unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR); // 开机默认主轴恒转速G97
	setBits(pSRun->unCommandType, CMD_G90G91_G90, CMD_G90G91_ADDR);	//开机默认G90,绝对编程
	if (g_SSystemPara.bDefaultG99 == TRUE)
	{//开机默认G99 ?  2023.4.27朱稳中
		setBits(pSRun->unCommandType, CMD_FEED_PER_R, CMD_FEED_ADDR);
	}
	else
	{
		setBits(pSRun->unCommandType, CMD_FEED_PER_MIN, CMD_FEED_ADDR);	//开机默认G98，陈敏聪	2023/6/1
	}

	//快速运动倍率
	pSRun->dQuickMoveFeedRatio = 1.0;
	//进给倍率 1.0
	pSRun->dCommonFeedRatio = pSPreProcess->SUserData.dFeedRatio = 1.0;
	//主轴倍率
	pSRun->dSpindleSpeedRatio = 1.0;
	//手轮倍率
	pSRun->dHandWheelSpeedRatio = 1.0;
	//插补方向
	pSPreProcess->SUserData.cInterpDir = INP_GO_AHEAD;
	//需要前瞻的单轴轴号
	pSPreProcess->SUserData.ucAxisNumOfNeedLookAhead = AXIS_NULL;


	//主轴转速置为无效
	//pSPreProcess->dSpindleSpeed = SPINDLE_INVALID_VAL;
	//G50转速置为无效
	//pSPreProcess->dLimitedSpindleSpeed = SPINDLE_INVALID_VAL;
	//通道指令置为无效
	pSPreProcess->pSChannel->SSpindle.dSpindleSpeed = pSPreProcess->pSChannel->SSpindle.dSpindleLineSpeed = pSPreProcess->pSChannel->SSpindle.dLimitedSpindleSpeed = -1;
	pSPreProcess->cChannelBufNum = -1;
	pSPreProcess->sChannelCommand.ucType = UNASSIGNED_VALUE;
}



#if STM32_MODE
void G28FindZero(VAL_STRUCT_TYPE* ucG28ZeroingValAxis, uint8 ucChannelNum)
{
	//空指针
	if (ucG28ZeroingValAxis == NULL)
	{
		return;
	}
	//如果传入的回零轴有效
	if (*ucG28ZeroingValAxis != 0)
	{
		//遍历各个轴
		int8 cTemp = 0;
		for (cTemp = 0; cTemp < AXIS_NUM_MAX; cTemp++)
		{
			uint8 axisManag = g_SChannel[ucChannelNum].pMapAxisArray[cTemp];
			if(axisManag >= AXIS_NUM_MAX)
			continue;
			//需要回零
			if (getbit((*ucG28ZeroingValAxis), cTemp) == 1)
			{
				origin_start(cTemp, g_SSystemPara.MOTOR_WARN[axisManag].origin_way, g_SSystemPara.MOTOR_WARN[axisManag].origin_dire, getPhysicalAxisType(cTemp), ucChannelNum);
				Axisorigin_wayInt();

			}
			//不需要回零
			else
			{
				continue;
			}

			//单轴成功回零
			if (check_origintask(cTemp, ucChannelNum) == 0)
			{
				//把轴位清零
				clrbit((*ucG28ZeroingValAxis), cTemp);
			}

		}


	}

}
#endif
/*************************************************
  函数名称：softLimitHandWheel(uint8 ucChannelNum)
  函数功能  软限位外挂函数（手轮使用）
  输入参数：ucChannelNum 通道号
  输出参数： -1 通道号错误或软限位不起作用
			  0 触发软限位
			  1 没触发软限位
  返 回 值：
  备    注：软限位复位状态下也不启用
**************************************************/
int8 softLimitForHandWheel(uint8 ucChannelNum)//朱稳中 2023.6.15
{
	SPointDouble SCurPoint; // 当前点
	int i;
	uint8* pMapAxisArray;//映射
	uint8 ucPhysicalAxis;//物理轴
	double dDecDist; //减速距离
	double dCurSpeed; //当前速度
	double dVe; //减速度
	double dDecFactor;//减速计算系数
	double dDec;
	bool bIsHandWheelMode = (g_RunMode == HANDWHEEL_MODE);
	bool bIsSimulaActive = ((g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE) && g_SChannel[ucChannelNum].cHandWheelSimulaStatus == HANWHEELSIMULA_ON);
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	if (bIsSimulaActive && isHandwheelAutomaticMoveRequired(&g_SChannel[ucChannelNum]))
	{
		bIsSimulaActive = FALSE; /* 自动运动沿用原有处理，不按手轮模拟处理。 */
	}
#endif
	if (!(bIsHandWheelMode || bIsSimulaActive))//手轮模式/手轮模拟下进入检测
	{
		return -1;
	}

	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_RESET) //复位时不起作用
	{
		return -1;
	}

	//获取映射
	pMapAxisArray = g_SChannel[ucChannelNum].pMapAxisArray;

	//获取当前实时点
	getRealTimePointByChannel(ucChannelNum, (double*)&SCurPoint);

	//对每个轴进行判断
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		ucPhysicalAxis = pMapAxisArray[i];
		if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)//物理轴映射为空 不判断
		{
			continue;
		}
		if (g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[i] == 0                   //脉冲为0
			|| g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS) //旋转轴不判断
		{
			continue;
		}

		//获取末速度
		dVe = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe;
		//获取减速距离计算系数
		dDecFactor = getFactor(g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType);//根据减速曲线类型获取系数
		dDec = getSingleAxisDec(i, g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType, pMapAxisArray);
		if (dDec <= 0)
		{
			parseRunningError(9999,GCODE_ERROR_NUM153,9999);
			return -1;
		}

		//计算当前速度
		dCurSpeed = g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[i] * g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm / g_SSystemPara.dInterpolationTime;

		//计算减速距离
		dDecDist = dDecFactor * (dCurSpeed * dCurSpeed - dVe * dVe) / dDec;

		//dDecDist = (dCurSpeed * dCurSpeed - dVe * dVe) / (2 * g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxDec);   // T型减速距离计算

		//判断是否即将超过软限位
		if (nSgn(g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[i]) > 0)//正向运动
		{
			if (g_SSystemPara.MOTOR_WARN[i].soft_pos == 1 //开启正软限位
				&& SCurPoint.dArray[i] + dDecDist > g_SSystemPara.MOTOR_WARN[i].pos_soft)  //当前点+减速距离>正限位
			{
				g_SChannel[ucChannelNum].SSingleAxisArray[i].ucRunMode = SINGLEAXIS_LIMIT_POS;//正限位
				resetButtonAPI(ucChannelNum); // 复位
				//printf("正限位\n");
				//报警函数
				parseRunningError(g_SChannel[ucChannelNum].SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM41 + i, 9999);
				setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数
				return 0;
			}
		}
		else //负向运动
		{
			if (g_SSystemPara.MOTOR_WARN[i].soft_neg == 1 //开启负软限位
				&& SCurPoint.dArray[i] - dDecDist < g_SSystemPara.MOTOR_WARN[i].neg_soft)//当前点+减速距离<负限位
			{
				g_SChannel[ucChannelNum].SSingleAxisArray[i].ucRunMode = SINGLEAXIS_LIMIT_NEG;//负限位
				resetButtonAPI(ucChannelNum); // 复位
				//printf("负限位\n");
				//报警函数
				parseRunningError(g_SChannel[ucChannelNum].SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM47 + i, 9999);
				setErrorWriteFlag(FALSE);//清除报警写入标志位，防止进入报警轴停函数
				return 0;
			}
		}
	}

	return 1;
}
/*************************************************
  函数名称： softLimit(struct SChannel* pSChannel, SPointDouble SStartPoint, SPointDouble SEndPoint, SPointDouble SMaxVal, SPointDouble SMinVal)
  函数功能：软限位函数
  输入参数：pSChannel	通道结构体指针
			SStartPoint   终点
			SEndPoint     终点
			SMaxVal 插补平面上各轴的最大值
			SMinVal 插补平面上各轴的最小值

  输出参数：
  返 回 值：  0 通道号错误或未触发软限位
			  1 当前触发软限位且软限位点在当前线段内
			  2 当前触发软限位且软限位点在当前线段外
			  3 当前线段之前已触发软限位

  备    注：1.判断最值是否回超出限位点；2.判断单轴末速度减速到终止速度的减速距离是否超出限位点
**************************************************/
// int8 softLimit(struct SChannel* pSChannel, SPointDouble SStartPoint, SPointDouble SEndPoint, SPointDouble SMaxVal, SPointDouble SMinVal)
// {
// 	uint8* pMapAxisArray;//映射
// 	double dDecDist1 = 0; //减速距离1：当前段单轴末速度减速至终止速度需要的距离
// 	double dDecDist2 = 0; //减速距离2：下一段单轴初速度减速至终止速度需要的距离
// 	double dCurVe = 0;	 //当前线段单轴末速度
// 	double dNextVs = 0;	 //下一线段单轴初速度
// 	double dVmin1 = 0, dVmin2 = 0;	//前后线段的终止速度
// 	double dDec1 = 0, dDec2 = 0;    //前后线段的减速度
// 	double dDecFactor1, dDecFactor2;//前后线段的减速系数
// 	double dNearestPosLimited = 9999.9;//用于判断最先触发软限位轴的距离变量
// 	double dNearestNegLimited = 9999.9;
// 	int8 cPosLimitedAXIS = -1;//最先触发软限位的轴号
// 	int8 cNegLimitedAXIS = -1;//最先触发软限位的轴号
// 	struct SToolCompensation* pNextSToolCompPointer = NULL;   //刀补指针
// 	enum RingBufPointerType PointerType;//指针类型
// 	PointerType = (pSChannel->SPreProcess.cInterpDir != INP_GO_BACK) ? INTERPOLATION_READ : INTERPOLATION_BACK;//根据插补方向获取取下一段
// 	pNextSToolCompPointer = getSToolCompPointerByType(PointerType, pSChannel->ucChannelNum);                   //插补下一段的刀补指针
// 	if (pSChannel->ucChannelNum >= CHANNEL_NUM_MAX
// 		|| g_RunMode == RE0_MODE                        //回零不起作用 2024/6/4 洪冠鹏
// 		|| g_RunMode == HAND_MODE				        //手动不起作用 2024/12/12
// 		|| g_SChannel[pSChannel->ucChannelNum].cHandWheelSimulaStatus == HANWHEELSIMULA_ON  //手轮模拟不起作用
// 		|| g_SChannel[pSChannel->ucChannelNum].ucChannelStatus == SYS_RESET) //复位时不起作用)
// 	{
// 		return 0;
// 	}
// 	//获取映射
// 	pMapAxisArray = pSChannel->pMapAxisArray;
// 	if (pSChannel->SRun.cSoftLimitedOn == 3)//当前线段之前已触发软限位 该标志位在触发软限位段处置TRUE
// 	{
// 		return 3;
// 	}
// 	/******************1.判断最值点是否超出限位点************************/
// 	for (int i = 0; i < AXIS_NUM_MAX; i++)
// 	{
// 		if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][i] == AXIS_NULL || g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == ROTATION_AXIS)continue; //轴映射为空或旋转轴不判断
// 		if (VAL_VALID != getbit(pSChannel->SPreProcess.SUserData.ucValStatus, i)) continue;  //无效轴 跳过
// 		if (g_SSystemPara.MOTOR_WARN[i].soft_pos == 1)//开启正限位检测
// 		{
// 			if (SMaxVal.dArray[i] > g_SSystemPara.MOTOR_WARN[i].pos_soft)//最值点 > 正限位点
// 			{
// 				double  dNearestPosLimitedTemp = SMaxVal.dArray[i] - g_SSystemPara.MOTOR_WARN[i].pos_soft;//计算限位点到最值点的距离
// 				if (dNearestPosLimited - dNearestPosLimitedTemp > EPSINON)//寻找最先触发限位的轴
// 				{
// 					cPosLimitedAXIS = i;//保存轴号
// 					dNearestPosLimited = dNearestPosLimitedTemp;
// 					continue;
// 				}
// 				else//同时触发限位的轴（圆弧需特殊处理）
// 				{
// 					if (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
// 					{
// 						cPosLimitedAXIS = dNearestPosLimited * dNearestPosLimitedTemp < 0 ? pSChannel->SG02.ucAxis[0] : pSChannel->SG02.ucAxis[1];
// 						dNearestPosLimited = dNearestPosLimitedTemp;
// 						continue;
// 					}
// 					if (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
// 					{
// 						cPosLimitedAXIS = dNearestPosLimited * dNearestPosLimitedTemp < 0 ? pSChannel->SG02.ucAxis[1] : pSChannel->SG02.ucAxis[0];
// 						dNearestPosLimited = dNearestPosLimitedTemp;
// 						continue;
// 					}
// 				}
// 			}
// 		}
// 		if (g_SSystemPara.MOTOR_WARN[i].soft_neg == 1) //开启负限位检测
// 		{
// 			if (SMinVal.dArray[i] < g_SSystemPara.MOTOR_WARN[i].neg_soft)//最值点 < 负限位点
// 			{
// 				double  dNearestNegLimitedTemp = g_SSystemPara.MOTOR_WARN[i].neg_soft - SMinVal.dArray[i];//计算最值点到起点的距离
// 				if (dNearestNegLimited - dNearestNegLimitedTemp > EPSINON)//寻找最先触发软限位的轴
// 				{
// 					cNegLimitedAXIS = i;//保存轴号
// 					dNearestNegLimited = dNearestNegLimitedTemp;
// 					continue;
// 				}
// 				else//同时触发限位的轴（圆弧需特殊处理）
// 				{
// 					if (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
// 					{
// 						cNegLimitedAXIS = dNearestNegLimited * dNearestNegLimitedTemp < 0 ? pSChannel->SG02.ucAxis[0] : pSChannel->SG02.ucAxis[1];
// 						dNearestNegLimited = dNearestNegLimitedTemp;
// 						continue;
// 					}
// 					if (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
// 					{
// 						cNegLimitedAXIS = dNearestNegLimited * dNearestNegLimitedTemp < 0 ? pSChannel->SG02.ucAxis[1] : pSChannel->SG02.ucAxis[0];
// 						dNearestNegLimited = dNearestNegLimitedTemp;
// 						continue;
// 					}
// 				}
// 			}
// 		}
// 	}
// 	if (cPosLimitedAXIS != -1)
// 	{
// 		pSChannel->SSingleAxisArray[cPosLimitedAXIS].ucRunMode = SINGLEAXIS_LIMIT_POS;//单轴运行模式设置为正限位
// 		return 1;
// 	}
// 	if (cNegLimitedAXIS != -1)
// 	{
// 		pSChannel->SSingleAxisArray[cNegLimitedAXIS].ucRunMode = SINGLEAXIS_LIMIT_NEG;//单轴运行模式设置为负限位
// 		return 1;
// 	}
// 	/******2.判断当前段单轴末速度和下一段单轴初速度减速到终止速度的减速距离是否超出限位点*******/
// 	for (int i = 0; i < AXIS_NUM_MAX; i++)
// 	{
// 		//1.获取当前段的单轴末速度
// 		if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][i] == AXIS_NULL || g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == ROTATION_AXIS)continue; //轴映射为空或旋转轴不判断
// 		if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)//G0
// 		{
// 			dCurVe = fabs(pSChannel->SSingleAxisArray[i].SPlanData.dVe);//快速运动为各单轴独立的运动
// 			dVmin1 = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe;
// 			dDec1 = getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);
// 			dDecFactor1 = getFactor(g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType);
// 		}
// 		else if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE)//G1
// 		{
// 			dCurVe = pSChannel->SG01.SPlanData.dVe * fabs(pSChannel->SG01.SStartVector.dArray[i]);//速度规划得到的是合成轨迹的末速度，乘位移分量得到单轴的末速度
// 			dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * fabs(pSChannel->SG01.SStartVector.dArray[i]);
// 			dDec1 = pSChannel->SPreProcess.SUserData.dDec * fabs(pSChannel->SG01.SStartVector.dArray[i]);
// 			dDecFactor1 = getFactor(pSChannel->SPreProcess.SUserData.ucDecVelPlanType);
// 		}
// 		else if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC || getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)//G2G3
// 		{
// 			//普通平面圆弧
// 			if (i == pSChannel->SG02.ucAxis[0])//当前轴为插补横轴
// 			{
// 				pSChannel->SG02.dCurEndPointAngle = asin((fabs(pSChannel->SG02.SCenterPoint.dArray[i] - pSChannel->SG02.SEndPoint.dArray[i])) / fabs(pSChannel->SG02.dRadius));//终点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
// 				dCurVe = pSChannel->SG02.SPlanData.dVe * (double)cos(pSChannel->SG02.dCurEndPointAngle);//横轴的末速度由速度规划圆弧终点的末速度乘cos夹角得到
// 				dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * (double)cos(pSChannel->SG02.dCurEndPointAngle);
// 				dDec1 = pSChannel->SPreProcess.SUserData.dDec * (double)cos(pSChannel->SG02.dCurEndPointAngle);
// 			}
// 			else if (i == pSChannel->SG02.ucAxis[1])//当前轴为插补纵轴
// 			{
// 				pSChannel->SG02.dCurEndPointAngle = acos((fabs(pSChannel->SG02.SCenterPoint.dArray[i] - pSChannel->SG02.SEndPoint.dArray[i])) / fabs(pSChannel->SG02.dRadius));//终点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
// 				dCurVe = pSChannel->SG02.SPlanData.dVe * (double)sin(pSChannel->SG02.dCurEndPointAngle);//纵轴的末速度由速度规划圆弧终点的末速度乘sin夹角得到
// 				dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * (double)sin(pSChannel->SG02.dCurEndPointAngle);
// 				dDec1 = pSChannel->SPreProcess.SUserData.dDec * (double)sin(pSChannel->SG02.dCurEndPointAngle);
// 			}
// 			else//其他轴的末速度由速度规划的合成位移的末速度乘距离分量得到
// 			{
// 				dCurVe = pSChannel->SG02.SPlanData.dVe * pSChannel->SG02.SStartVector.dArray[i];
// 				dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * pSChannel->SG02.SStartVector.dArray[i];
// 				dDec1 = pSChannel->SPreProcess.SUserData.dDec * pSChannel->SG02.SStartVector.dArray[i];
// 			}
// 			dDecFactor1 = getFactor(pSChannel->SPreProcess.SUserData.ucDecVelPlanType);
// 		}
// 		else
// 		{
// 			return 0;
// 		}
// 		//2.获取下一段的单轴初速度   //考虑拐角约束下的单轴速度可能会变小或变大，计算下一线段的单轴初速度减速到终止速度的减速距离
// 		if (getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)//G0
// 		{
// 			dNextVs = pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == i ? fabs(pNextSToolCompPointer->dVs) : g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVs;//快速运动没有前瞻，各段始末速度来自系统设定
// 			dVmin2 = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe;
// 			dDec2 = getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);
// 			dDecFactor2 = getFactor(g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType);
// 		}
// 		else if (getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE)//G1
// 		{
// 			dNextVs = pNextSToolCompPointer->dVs * fabs(pNextSToolCompPointer->SStartVector.dArray[i]);//下一段的初速度乘以位移分量得到单轴的初速度
// 			dVmin2 = pNextSToolCompPointer->dMinVe * fabs(pNextSToolCompPointer->SStartVector.dArray[i]);
// 			dDec2 = pNextSToolCompPointer->dDec * fabs(pNextSToolCompPointer->SStartVector.dArray[i]);
// 			dDecFactor2 = getFactor(pNextSToolCompPointer->ucDecVelPlanType);
// 		}
// 		else if (getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC || getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)//G2G3
// 		{
// 			//普通平面圆弧
// 			if (i == pSChannel->SG02.ucAxis[0])//当前轴为插补横轴
// 			{
// 				pSChannel->SG02.dNextStartPointAngle = asin((fabs(pNextSToolCompPointer->SCenterPoint.dArray[i] - pNextSToolCompPointer->SStartPoint.dArray[i])) / fabs(pNextSToolCompPointer->dRadius));//起点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
// 				dNextVs = pNextSToolCompPointer->dVs * (double)cos(pSChannel->SG02.dNextStartPointAngle);//横轴的初速度由圆弧初速度乘cos夹角得到
// 				dVmin2 = pNextSToolCompPointer->dMinVe * (double)cos(pSChannel->SG02.dNextStartPointAngle);
// 				dDec2 = pNextSToolCompPointer->dDec * (double)cos(pSChannel->SG02.dNextStartPointAngle);
// 			}
// 			else if (i == pSChannel->SG02.ucAxis[1])//当前轴为插补纵轴
// 			{
// 				pSChannel->SG02.dNextStartPointAngle = acos((fabs(pNextSToolCompPointer->SCenterPoint.dArray[i] - pNextSToolCompPointer->SStartPoint.dArray[i])) / fabs(pNextSToolCompPointer->dRadius));//起点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
// 				dNextVs = pNextSToolCompPointer->dVs * (double)sin(pSChannel->SG02.dNextStartPointAngle);//纵轴的初速度由圆弧初速度乘sin夹角得到
// 				dVmin2 = pNextSToolCompPointer->dMinVe * (double)sin(pSChannel->SG02.dNextStartPointAngle);
// 				dDec2 = pNextSToolCompPointer->dDec * (double)sin(pSChannel->SG02.dNextStartPointAngle);
// 			}
// 			else//其他轴的初速度由初速度乘距离分量得到
// 			{
// 				dNextVs = pNextSToolCompPointer->dVs * pNextSToolCompPointer->SStartVector.dArray[i];
// 				dVmin2 = pNextSToolCompPointer->dMinVe * pNextSToolCompPointer->SStartVector.dArray[i];
// 				dDec2 = pNextSToolCompPointer->dDec * pNextSToolCompPointer->SStartVector.dArray[i];
// 			}
// 			dDecFactor2 = getFactor(pNextSToolCompPointer->ucDecVelPlanType);
// 		}
// 		else
// 		{
// 			return 0;
// 		}
// 		//3.获取单轴终止速度
// 		//dVmin = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe;
// 		//4.获取减速距离的计算系数
// 		//dDecFactor = getFactor(g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType);//根据减速曲线类型获取系数
// 		//dDecFactor = 0.5;//	T型减速度
// 		//5.计算减速距离
// 		if ((dCurVe < dVmin1 || isZero(dCurVe - dVmin1)) || (dNextVs < dVmin2 || isZero(dNextVs - dVmin2)))//当前段单轴末速度或下一段单轴初速度两者之一小于终止速度时，能直接停，不会触发软限位，减速距离为0
// 		{
// 			continue;
// 		}
// 		else
// 		{
// 			//dDecDist1 = dDecFactor * (dCurVe * dCurVe - dVmin * dVmin) / getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);//减速距离1
// 			//dDecDist2 = dDecFactor * (dNextVs * dNextVs - dVmin * dVmin) / getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);//减速距离2
// 			//dDecDist1 = dDecFactor * (dCurVe * dCurVe - dVmin * dVmin) / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dSysPlanTMaxDec;  //减速距离1（T型公式）
// 			//dDecDist2 = dDecFactor * (dNextVs * dNextVs - dVmin * dVmin) / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dSysPlanTMaxDec;//减速距离2（T型公式）
// 			dDecDist1 = dDecFactor1 * (dCurVe * dCurVe - dVmin1 * dVmin1) / dDec1;//减速距离1 (依据指令类型取系统参数)
// 			dDecDist2 = dDecFactor2 * (dNextVs * dNextVs - dVmin2 * dVmin2) / dDec2;//减速距离2
// 		}
// 		if (g_SSystemPara.MOTOR_WARN[i].soft_pos == 1)//开启正软限位检测
// 		{
// 			if ((SEndPoint.dArray[i] + dDecDist1 > g_SSystemPara.MOTOR_WARN[i].pos_soft) && (SEndPoint.dArray[i] + dDecDist2 > g_SSystemPara.MOTOR_WARN[i].pos_soft)) //②终点 + 减速距离1 > 正限位点 且 终点 + 减速距离2 > 正限位点（在终点处单轴末速度没有足够的距离减速至终止速度）
// 			{
// 				pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_LIMIT_POS;//单轴运行模式设置为正限位
// 				return 2;
// 			}
// 		}
// 		if (g_SSystemPara.MOTOR_WARN[i].soft_neg == 1) //开启负软限位检测
// 		{
// 			if ((SEndPoint.dArray[i] - dDecDist1 < g_SSystemPara.MOTOR_WARN[i].neg_soft) && (SEndPoint.dArray[i] - dDecDist2 < g_SSystemPara.MOTOR_WARN[i].neg_soft))//②终点距离 + 减速距离1 < 负限位点 且 终点距离 + 减速距离2 < 负限位点（在终点处单轴末速度没有足够的距离减速至终止速度）
// 			{
// 				pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_LIMIT_NEG;//单轴运行模式设置为负限位
// 				return 2;
// 			}
// 		}
// 	}
// 	return 0;
// }
/*************************************************
  函数名称： softLimit(struct SChannel* pSChannel, SPointDouble SStartPoint, SPointDouble SEndPoint, SPointDouble SMaxVal, SPointDouble SMinVal, uint8 ucAxis)
  函数功能：软限位函数
  输入参数：pSChannel	通道结构体指针
			SStartPoint   终点
			SEndPoint     终点
			SMaxVal 插补平面上各轴的最大值
			SMinVal 插补平面上各轴的最小值
			ucAxis  轴号
  输出参数：
  返 回 值：  0 通道号错误或未触发软限位
			  1 当前触发软限位且软限位点在当前线段内
			  2 当前触发软限位且软限位点在当前线段外
			  3 当前线段之前已触发软限位

  备    注：1.判断最值是否回超出限位点；2.判断单轴末速度减速到终止速度的减速距离是否超出限位点(上述方案存在优先触发轴检测问题)
**************************************************/
int8 softLimit(struct SChannel *pSChannel, SPointDouble SStartPoint, SPointDouble SEndPoint, SPointDouble SMaxVal, SPointDouble SMinVal, uint8 ucAxis)
{
	uint8 *pMapAxisArray;			 // 映射
	double dDecDist1 = 0;			 // 减速距离1：当前段单轴末速度减速至终止速度需要的距离
	double dDecDist2 = 0;			 // 减速距离2：下一段单轴初速度减速至终止速度需要的距离
	double dCurVe = 0;				 // 当前线段单轴末速度
	double dNextVs = 0;				 // 下一线段单轴初速度
	double dVmin1 = 0, dVmin2 = 0;	 // 前后线段的终止速度
	double dDec1 = 0, dDec2 = 0;	 // 前后线段的减速度
	double dDecFactor1, dDecFactor2; // 前后线段的减速系数

	struct SToolCompensation *pNextSToolCompPointer = NULL;														// 刀补指针
	enum RingBufPointerType PointerType;																		// 指针类型
	PointerType = (pSChannel->SPreProcess.cInterpDir != INP_GO_BACK) ? INTERPOLATION_READ : INTERPOLATION_BACK; // 根据插补方向获取取下一段
	if(PointerType == INTERPOLATION_BACK)
	{
		if(getSToolCompPointerBufRes(PointerType, pSChannel->ucChannelNum) > 0)
		{
			pNextSToolCompPointer = getSToolCompPointerByType(PointerType, pSChannel->ucChannelNum);// 插补下一段的刀补指针
		}
		else
		{
			INDEX_DATA_TYPE nIndex = getSToolCompPointerIndex(INTERPOLATION_READ, pSChannel->ucChannelNum) - 1; //倒走时获取当前段指针
		    pNextSToolCompPointer = getSToolCompPointerByIndex(nIndex, pSChannel->ucChannelNum);
		}
	}
	else
	{
		pNextSToolCompPointer = getSToolCompPointerByType(PointerType, pSChannel->ucChannelNum);					// 插补下一段的刀补指针
	}

	//  pNextSToolCompPointer = getSToolCompPointerByType(PointerType, pSChannel->ucChannelNum);					// 插补下一段的刀补指针

	

	if (pSChannel->ucChannelNum >= CHANNEL_NUM_MAX || g_RunMode == RE0_MODE				   // 回零不起作用 2024/6/4 洪冠鹏
		|| g_RunMode == HAND_MODE										   // 手动模式不起作用 2024/12/12
		|| g_SChannel[pSChannel->ucChannelNum].cHandWheelSimulaStatus == HANWHEELSIMULA_ON
#if HANDWHEEL_SIM_DISTANCE_ENABLE
			&& !isHandwheelAutomaticMoveRequired(pSChannel)
#endif
		|| g_SChannel[pSChannel->ucChannelNum].ucChannelStatus == SYS_RESET)			   // 复位时不起作用)
	{
		return 0;
	}

	// 获取映射
	pMapAxisArray = pSChannel->pMapAxisArray;

	if (pSChannel->SRun.cSoftLimitedOn == 3) // 当前线段之前已触发软限位 该标志位在触发软限位段处置TRUE
	{
		return 3;
	}
	int8 cSoftLimetResult = -1;
	/**修改2025.10.9*/
	/******************1.判断当前段最值点是否超出限位点************************/
	switch (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR)) // 指令类型判断
	{
	case CMD_MOTION_QUICKMOVE:															  // 快速运动
		cSoftLimetResult = checkSingleAxisSoftLimit(pSChannel, ucAxis, SMaxVal, SMinVal); // 检查当前轴是否触发软限位
		if (cSoftLimetResult == 1)														  // 当前轴触发软限位
		{
			return 1;
		}
		else if (cSoftLimetResult == 0) // 当前轴为无效轴
		{
			return 0;
		}
		break;
	case CMD_MOTION_LINE:																		// 合成运动
		cSoftLimetResult = checkG1FirstAxisSoftLimit(pSChannel, SStartPoint, SMaxVal, SMinVal); // 检查各轴运动，哪段轴最先触发软限位
		if (cSoftLimetResult == 1)																// 有轴触发软限位
		{
			return 1;
		}
		break;
	case CMD_MOTION_ARC:																		// 顺圆
	case CMD_MOTION_ANTI_ARC:																	// 逆圆
		cSoftLimetResult = checkG2FirstAxisSoftLimit(pSChannel, SStartPoint, SMaxVal, SMinVal); // 检查各轴运动，哪段轴最先触发软限位
		if (cSoftLimetResult == 1)																// 有轴触发软限位
		{
			return 1;
		}
		break;
	/**2026.1.5 新增对G32,G33软限位判断 */
	case CMD_MOTION_SCREWCUT:
	case CMD_MOTION_G33:
	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][i] == AXIS_NULL || g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == ROTATION_AXIS)continue; //轴映射为空或旋转轴不判断
		if (VAL_VALID != getbit(pSChannel->SPreProcess.SUserData.ucValStatus, i)) continue;  //无效轴 跳过
		if (g_SSystemPara.MOTOR_WARN[i].soft_pos == 1)//开启正限位检测
		{
			if (SMaxVal.dArray[i] > g_SSystemPara.MOTOR_WARN[i].pos_soft)//最值点 > 正限位点
			{
					parseRunningError(pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM41 + i, 9999);//触发正限位报警
			}
		}
		if (g_SSystemPara.MOTOR_WARN[i].soft_neg == 1) //开启负限位检测
		{
			if (SMinVal.dArray[i] < g_SSystemPara.MOTOR_WARN[i].neg_soft)//最值点 < 负限位点
			{
					parseRunningError(pSChannel->SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM47 + i, 9999);//触发负限位报警
			}
		}
	}
		if(isErrorHasBeenWritten())//触发软限位已写入
	{
		return 1;
	}

	default:
		break;
	}
	/**修改2025.10.9*/

	/******2.判断当前段单轴末速度和下一段单轴初速度减速到终止速度的减速距离是否超出限位点*******/
	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		// 1.获取当前段的单轴末速度
		if (g_SSystemPara.ucAxisMapArray[pSChannel->ucChannelNum][i] == AXIS_NULL || g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucAxisType == ROTATION_AXIS)
			continue; // 轴映射为空或旋转轴不判断

		if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) // G0
		{
			dCurVe = fabs(pSChannel->SSingleAxisArray[i].SPlanData.dVe); // 快速运动为各单轴独立的运动
			dVmin1 = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe;
			dDec1 = getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);
			dDecFactor1 = getFactor(g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType);
		}
		else if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE) // G1
		{
			dCurVe = pSChannel->SG01.SPlanData.dVe * fabs(pSChannel->SG01.SStartVector.dArray[i]); // 速度规划得到的是合成轨迹的末速度，乘位移分量得到单轴的末速度
			dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * fabs(pSChannel->SG01.SStartVector.dArray[i]);
			dDec1 = pSChannel->SPreProcess.SUserData.dDec * fabs(pSChannel->SG01.SStartVector.dArray[i]);
			dDecFactor1 = getFactor(pSChannel->SPreProcess.SUserData.ucDecVelPlanType);
		}
		else if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC || getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC) // G2G3
		{
			// 普通平面圆弧
			if (i == pSChannel->SG02.ucAxis[0]) // 当前轴为插补横轴
			{
				pSChannel->SG02.dCurEndPointAngle = asin((fabs(pSChannel->SG02.SCenterPoint.dArray[i] - pSChannel->SG02.SEndPoint.dArray[i])) / fabs(pSChannel->SG02.dRadius)); // 终点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
				dCurVe = pSChannel->SG02.SPlanData.dVe * (double)cos(pSChannel->SG02.dCurEndPointAngle);																		// 横轴的末速度由速度规划圆弧终点的末速度乘cos夹角得到
				dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * (double)cos(pSChannel->SG02.dCurEndPointAngle);
				dDec1 = pSChannel->SPreProcess.SUserData.dDec * (double)cos(pSChannel->SG02.dCurEndPointAngle);
			}
			else if (i == pSChannel->SG02.ucAxis[1]) // 当前轴为插补纵轴
			{
				pSChannel->SG02.dCurEndPointAngle = acos((fabs(pSChannel->SG02.SCenterPoint.dArray[i] - pSChannel->SG02.SEndPoint.dArray[i])) / fabs(pSChannel->SG02.dRadius)); // 终点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
				dCurVe = pSChannel->SG02.SPlanData.dVe * (double)sin(pSChannel->SG02.dCurEndPointAngle);																		// 纵轴的末速度由速度规划圆弧终点的末速度乘sin夹角得到
				dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * (double)sin(pSChannel->SG02.dCurEndPointAngle);
				dDec1 = pSChannel->SPreProcess.SUserData.dDec * (double)sin(pSChannel->SG02.dCurEndPointAngle);
			}
			else // 其他轴的末速度由速度规划的合成位移的末速度乘距离分量得到
			{
				dCurVe = pSChannel->SG02.SPlanData.dVe * pSChannel->SG02.SStartVector.dArray[i];
				dVmin1 = pSChannel->SPreProcess.SUserData.dMinVe * pSChannel->SG02.SStartVector.dArray[i];
				dDec1 = pSChannel->SPreProcess.SUserData.dDec * pSChannel->SG02.SStartVector.dArray[i];
			}
			dDecFactor1 = getFactor(pSChannel->SPreProcess.SUserData.ucDecVelPlanType);
		}
		else
		{
			return 0;
		}

		// 2.获取下一段的单轴初速度   //考虑拐角约束下的单轴速度可能会变小或变大，计算下一线段的单轴初速度减速到终止速度的减速距离
		if (getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) // G0
		{
			dNextVs = pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == i ? fabs(pNextSToolCompPointer->dVs) : g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVs; // 快速运动没有前瞻，各段始末速度来自系统设定
			dVmin2 = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe;
			dDec2 = getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);
			dDecFactor2 = getFactor(g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType);
		}
		else if (getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE) // G1
		{
			dNextVs = pNextSToolCompPointer->dVs * fabs(pNextSToolCompPointer->SStartVector.dArray[i]); // 下一段的初速度乘以位移分量得到单轴的初速度
			dVmin2 = pNextSToolCompPointer->dMinVe * fabs(pNextSToolCompPointer->SStartVector.dArray[i]);
			dDec2 = pNextSToolCompPointer->dDec * fabs(pNextSToolCompPointer->SStartVector.dArray[i]);
			dDecFactor2 = getFactor(pNextSToolCompPointer->ucDecVelPlanType);
		}
		else if (getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC || getBits(pNextSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC) // G2G3
		{
			// 普通平面圆弧
			if (i == pSChannel->SG02.ucAxis[0]) // 当前轴为插补横轴
			{
				pSChannel->SG02.dNextStartPointAngle = asin((fabs(pNextSToolCompPointer->SCenterPoint.dArray[i] - pNextSToolCompPointer->SStartPoint.dArray[i])) / fabs(pNextSToolCompPointer->dRadius)); // 起点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
				dNextVs = pNextSToolCompPointer->dVs * (double)cos(pSChannel->SG02.dNextStartPointAngle);																								  // 横轴的初速度由圆弧初速度乘cos夹角得到
				dVmin2 = pNextSToolCompPointer->dMinVe * (double)cos(pSChannel->SG02.dNextStartPointAngle);
				dDec2 = pNextSToolCompPointer->dDec * (double)cos(pSChannel->SG02.dNextStartPointAngle);
			}
			else if (i == pSChannel->SG02.ucAxis[1]) // 当前轴为插补纵轴
			{
				pSChannel->SG02.dNextStartPointAngle = acos((fabs(pNextSToolCompPointer->SCenterPoint.dArray[i] - pNextSToolCompPointer->SStartPoint.dArray[i])) / fabs(pNextSToolCompPointer->dRadius)); // 起点与纵轴方向的夹角，也是合成速度与横轴方向的夹角，单位：rad
				dNextVs = pNextSToolCompPointer->dVs * (double)sin(pSChannel->SG02.dNextStartPointAngle);																								  // 纵轴的初速度由圆弧初速度乘sin夹角得到
				dVmin2 = pNextSToolCompPointer->dMinVe * (double)sin(pSChannel->SG02.dNextStartPointAngle);
				dDec2 = pNextSToolCompPointer->dDec * (double)sin(pSChannel->SG02.dNextStartPointAngle);
			}
			else // 其他轴的初速度由初速度乘距离分量得到
			{
				dNextVs = pNextSToolCompPointer->dVs * pNextSToolCompPointer->SStartVector.dArray[i];
				dVmin2 = pNextSToolCompPointer->dMinVe * pNextSToolCompPointer->SStartVector.dArray[i];
				dDec2 = pNextSToolCompPointer->dDec * pNextSToolCompPointer->SStartVector.dArray[i];
			}
			dDecFactor2 = getFactor(pNextSToolCompPointer->ucDecVelPlanType);
		}
		else
		{
			return 0;
		}

		// 3.获取单轴终止速度
		// dVmin = g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe;
		// 4.获取减速距离的计算系数
		// dDecFactor = getFactor(g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType);//根据减速曲线类型获取系数
		// dDecFactor = 0.5;//	T型减速度
		// 5.计算减速距离
		if ((dCurVe < dVmin1 || isZero(dCurVe - dVmin1)) || (dNextVs < dVmin2 || isZero(dNextVs - dVmin2))) // 当前段单轴末速度或下一段单轴初速度两者之一小于终止速度时，能直接停，不会触发软限位，减速距离为0
		{
			continue;
		}
		else
		{
			// dDecDist1 = dDecFactor * (dCurVe * dCurVe - dVmin * dVmin) / getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);//减速距离1
			// dDecDist2 = dDecFactor * (dNextVs * dNextVs - dVmin * dVmin) / getSingleAxisDec(i, g_SSystemPara.SAxisPara[pMapAxisArray[i]].ucDecVelPlanType, pMapAxisArray);//减速距离2
			// dDecDist1 = dDecFactor * (dCurVe * dCurVe - dVmin * dVmin) / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dSysPlanTMaxDec;  //减速距离1（T型公式）
			// dDecDist2 = dDecFactor * (dNextVs * dNextVs - dVmin * dVmin) / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dSysPlanTMaxDec;//减速距离2（T型公式）
			if (dDec1 <= 0 || dDec2 <= 0)
			{
				parseRunningError(9999,GCODE_ERROR_NUM153,9999);
				return -1;
			}
			
			dDecDist1 = dDecFactor1 * (dCurVe * dCurVe - dVmin1 * dVmin1) / dDec1;	 // 减速距离1 (依据指令类型取系统参数)
			dDecDist2 = dDecFactor2 * (dNextVs * dNextVs - dVmin2 * dVmin2) / dDec2; // 减速距离2
		}

		if (g_SSystemPara.MOTOR_WARN[i].soft_pos == 1) // 开启正软限位检测
		{
			if ((SEndPoint.dArray[i] + dDecDist1 > g_SSystemPara.MOTOR_WARN[i].pos_soft) && (SEndPoint.dArray[i] + dDecDist2 > g_SSystemPara.MOTOR_WARN[i].pos_soft)) // ②终点 + 减速距离1 > 正限位点 且 终点 + 减速距离2 > 正限位点（在终点处单轴末速度没有足够的距离减速至终止速度）
			{
				pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_LIMIT_POS; // 单轴运行模式设置为正限位
				return 2;
			}
		}
		if (g_SSystemPara.MOTOR_WARN[i].soft_neg == 1) // 开启负软限位检测
		{
			if ((SEndPoint.dArray[i] - dDecDist1 < g_SSystemPara.MOTOR_WARN[i].neg_soft) && (SEndPoint.dArray[i] - dDecDist2 < g_SSystemPara.MOTOR_WARN[i].neg_soft)) // ②终点距离 + 减速距离1 < 负限位点 且 终点距离 + 减速距离2 < 负限位点（在终点处单轴末速度没有足够的距离减速至终止速度）
			{
				pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_LIMIT_NEG; // 单轴运行模式设置为负限位
				return 2;
			}
		}
	}
	return 0;
}

#if ENCODER_MODE
/*************************************************
  函数名称：clearHandWheelPara(struct SChannel* pSChannel)
  函数功能  清除与手轮相关的参数
  输入参数：pSChannel	通道结构体指针
  输出参数：
  返 回 值：
  备    注：陈敏聪 2023/6/24增
****************************************/
void clearHandWheelPara(struct SChannel* pSChannel)
{
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	/* 初始化或复位时清理手轮模拟状态。 */
	if (pSChannel != NULL)
		resetHandwheelSimGroup(&pSChannel->SHandwheelSim);
#endif

#if HANDWHEEL_STOP_BOUNDARY_ENABLE
	if (pSChannel == NULL || pSChannel->pMapAxisArray == NULL)
		return;
#endif
	uint8* pMapAxisArray = pSChannel->pMapAxisArray;
	//初始化手轮的当前起点坐标
	getRealTimePointByChannel(pSChannel->ucChannelNum, (double*)&g_SEncoder[HANDEWHEEL_ENCODER - 1].SStartPoint);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
	if (pSChannel->bUseVirtualY == TRUE)//开启合成虚拟Y轴功能：将起点坐标从机床坐标系换算至工件坐标系
	{
		g_SEncoder[HANDEWHEEL_ENCODER - 1].SStartPoint.dArray[AXIS_Y] *= pSChannel->dCOSAngleYY1;
		g_SEncoder[HANDEWHEEL_ENCODER - 1].SStartPoint.dArray[AXIS_X] -= 1.0 * g_SEncoder[HANDEWHEEL_ENCODER - 1].SStartPoint.dArray[AXIS_Y] * pSChannel->dTANAngleYY1;
	}
#endif

	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
#if HANDWHEEL_STOP_BOUNDARY_ENABLE
		bool hadHandwheelMotion =
			g_SEncoder[HANDEWHEEL_ENCODER - 1].bHandwheelBraking[i] ||
			g_SEncoder[HANDEWHEEL_ENCODER - 1].cHandwheelMoveDir[i] != 0;
		g_SEncoder[HANDEWHEEL_ENCODER - 1].bHandwheelBraking[i] = FALSE;
		g_SEncoder[HANDEWHEEL_ENCODER - 1].cHandwheelMoveDir[i] = 0;
		g_SEncoder[HANDEWHEEL_ENCODER - 1].dHandwheelNetDist[i] = 0.0;
		g_SEncoder[HANDEWHEEL_ENCODER - 1].dHandwheelStopDist[i] = 0.0;
		/*
		 * H7 原复位循环会跳过部分轴；手轮持有的旧计划必须一并清掉，
		 * 否则新起点配旧 dPreDist/unCurrentT 会造成下次运动错位。
		 * 新增分支不清主轴模式的 SPlan；下方 H7 原分支原样保留。
		 */
		if (hadHandwheelMotion &&
			pSChannel->SSingleAxisArray[i].ucRunMode != SPINDLE_SINGLEAXIS_MODE)
			clearSPlanData(&pSChannel->SSingleAxisArray[i].SPlanData);
#endif
		g_SEncoder[HANDEWHEEL_ENCODER - 1].dDelataAccmulate = 0;		//清空编码器差值累加
		g_SEncoder[HANDEWHEEL_ENCODER - 1].sAxisAccmulate[i] = 0;		//清空各个运动轴未规划的脉冲数
		g_SEncoder[HANDEWHEEL_ENCODER - 1].dAxisCurrentSpeed[i] = 0;	//清空各个运动轴的当前速度
		g_SEncoder[HANDEWHEEL_ENCODER - 1].sPulseForChangeDie[i] = 0;	//清空换向时所存储的脉冲数
		g_SEncoder[HANDEWHEEL_ENCODER - 1].dAxisAccmulateRemainder[i] = 0;			//清空X轴存储手轮不能被8取余的脉冲
		if(pSChannel->SSingleAxisArray[i].ucRunMode == SPINDLE_SINGLEAXIS_MODE) continue;
#if HANDWHEEL_STOP_BOUNDARY_ENABLE
		if (pMapAxisArray[i] < AXIS_PHYSICS_NUM_MAX)
#endif
		g_dRealTimePointArray[pMapAxisArray[i]] -= g_SEncoder[HANDEWHEEL_ENCODER - 1].dPulseAccmulate[i] * g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm;
		g_SEncoder[HANDEWHEEL_ENCODER - 1].dPulseAccmulate[i] = 0;
		clearDistReAvgPara(&pSChannel->SSingleAxisArray[i].SPlanData);
		clearSPlanData(&pSChannel->SSingleAxisArray[i].SPlanData);
	}

}
#endif

/*************************************************
  函数名称：releaseLastCommandAxis(struct SInterpolationPre* pSPreprocess)
  函数功能  释放上一个指令的轴占用
  输入参数：pSPreprocess	预处理结构体指针
  输出参数：
  返 回 值：
  备    注：
****************************************/
int8 releaseLastCommandAxis(struct SInterpolationPre* pSPreprocess)
{
	uint8* ucMapArray;
	if (pSPreprocess == NULL || pSPreprocess->pSChannel == NULL || pSPreprocess->pSChannel->pMapAxisArray == NULL)
	{
		return -1;
	}
	/* 释放上一运动的轴即结束连续攻丝参考；相邻 G84 与孔底停留不走此分支。 */
	ucMapArray = pSPreprocess->pSChannel->pMapAxisArray;//获取映射  2022.12.06 修改位置

#if(TURRET_AXIS_TEST == 1)
	//释放旧的非刀塔有效轴
	axisManagerReleaseByStruct(pSPreprocess->ucOccupideValStatus & (~pSPreprocess->ucOccupideTurretAxis), ucMapArray, AXIS_NUM_MAX, &pSPreprocess->ucOccupideValStatus);
#else
	if (pSPreprocess->ucLastValStatus == 0x0)//上一条指令的轴占用为空
	{
		return 0; //返回
	}

	//释放旧的有效轴
	axisManagerReleaseByStruct(pSPreprocess->ucLastValStatus & (~pSPreprocess->ucToolAxis), ucMapArray, AXIS_NUM_MAX, NULL);
	//将上一条指令的轴占用置为空  防止重复释放
	pSPreprocess->ucLastValStatus = 0x0;
#endif
	return 1;
}


/*************************************************
  函数名称：initSRunPulseBuf(int8 cArrayNumMap)
  函数功能  清空结构体SRun脉冲增量缓存区
  输入参数：cArrayNumMap	0000 0000,  低六位分别代表轴号  1 2 3 4 5 6 （XYZABC)是否清空缓存 1为清空 0为不清空
  输出参数：
  返 回 值：
  备    注：
****************************************/
void initSRunPulseBuf(struct _SRun* pSRun, int8 cArrayNumMap)
{
	int i;
	//检查指针  值的变化为（0000 0001） （0000 0010） ...(1000 0000)  利用与运算检查传入参数cArrayNumMap各个位是否为1
	int8 cCheckBit = 0x01;

	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if ((cArrayNumMap & cCheckBit))  //与运算  检查某一位是否为1
		{
			pSRun->sLastDeltaPulse[i] = pSRun->sAxisDeltaPulse[i];
			pSRun->sAxisDeltaPulse[i] = 0;   //为1清空缓存
		}
		cCheckBit = cCheckBit << 1;  //将"1"往左移
	}
}

// /*************************************************
//   函数名称：outPutAbsPoint(SPointDouble STargetPoint)
//   函数功能  根据传入的理论绝对坐标，获取实时坐标，计算脉冲增量、当前坐标和更新实时坐标
//   输入参数：STargetPoint	SPointDouble结构体变量  目标点
//   输出参数：
//   返 回 值：
//   备    注：
// **************************************************/
// bool outPutAbsPoint(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
// {
// 	int i = 0;
// 	bool bUseSysPara = FALSE;//是否使用系统参数

// 	if (pSRun == NULL)//空指针
// 	{
// 		return FALSE;
// 	}
// 	if (dPulseToMmArray == NULL)// 传入指针为空 使用系统参数
// 	{
// 		bUseSysPara = TRUE;
// 	}

// 	//获取实时点作为上一点的位置
// 	getRealTimePoint((double*)&pSRun->SLastAbsCoord, AXIS_NUM_MAX, ucAxisMap, NULL);

// 	if (ucValStatus == NULL) //空指针  全部计算
// 	{
// 		if (bUseSysPara)//是否使用系统参数52
// 		{
// 			for (i = 0; i < nArraySize; i++)
// 			{
// 				if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX) continue;  //映射为空的物理轴  不计算

// 				//根据目标点和当前点计算脉冲当量
// 				if(g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm == 0 || g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm >65535)
// 				continue;
// 				pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm);

// 				//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
// 				pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm;
// 			}
// 		}
// 		else
// 		{
// 			for (i = 0; i < nArraySize; i++)
// 			{
// 				if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX) continue;  //映射为空的物理轴  不计算
				
// 				if(dPulseToMmArray[i] == 0 || dPulseToMmArray[i] >65535)continue;
// 				//根据目标点和当前点计算脉冲当量
// 				pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / dPulseToMmArray[i]);

// 				//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
// 				pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * dPulseToMmArray[i];
// 			}
// 		}
// 	}
// 	else //不为空 个别计算
// 	{
// 		if (bUseSysPara)//是否使用系统参数
// 		{
// 			for (i = 0; i < nArraySize; i++)
// 			{
// 				if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != getbit(*ucValStatus, i)) continue;  //映射为空的物理轴  不计算

// 				//根据目标点和当前点计算脉冲当量
// 				if(g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm == 0 || g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm >65535)
// 				continue;
// 				pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm);

// 				//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
// 				pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * g_SSystemPara.SAxisPara[ucAxisMap[i]].dPulseToMm;
// 			}
// 		}
// 		else
// 		{
// 			for (i = 0; i < nArraySize; i++)
// 			{
// 				if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != getbit(*ucValStatus, i)) continue;  //映射为空的物理轴  不计算
				
// 				if(dPulseToMmArray[i] == 0 || dPulseToMmArray[i] >65535)continue;
// 				//根据目标点和当前点计算脉冲当量
// 				pSRun->sAxisDeltaPulse[i] = round((pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i]) / dPulseToMmArray[i]);

// 				//修正该点绝对坐标(根据上一点坐标和脉冲增量计算)
// 				pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + pSRun->sAxisDeltaPulse[i] * dPulseToMmArray[i];
// 			}
// 		}
// 	}

// 	//改变实时点的位置
// 	updateRealTimePoint((double*)&pSRun->SAbsCoord, AXIS_NUM_MAX, ucAxisMap, ucValStatus);

// 	return TRUE;
// }

/*************************************************
  函数名称：outPutAbsPoint
  函数功能：根据目标绝对坐标，获取实时坐标，计算各轴脉冲增量并更新实时坐标
  输入参数： pSRun           运行结构体指针
            pdTargetPoint   目标绝对坐标数组
            nArraySize      数组大小
            ucAxisMap       轴映射表
            dPulseToMmArray 脉冲当量数组 (若为NULL则使用系统参数)
            ucValStatus     有效轴状态位
  返回值：  TRUE: 成功
            FALSE: 失败 (参数异常或脉冲当量无效)
  备注：    统一校验 dPulseToMm 的有效性，失败时清零本轴脉冲、写报警并中止
**************************************************/
bool outPutAbsPoint(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;
    bool bUseSysPara = FALSE; // 是否使用系统参数

    // 1. 指针合法性检查
    if (pSRun == NULL || pdTargetPoint == NULL || ucAxisMap == NULL)
	{
		return FALSE;
	}

    // 确定脉冲当量来源
    if (dPulseToMmArray == NULL)
	{
		bUseSysPara = TRUE;
	}

    // 2. 获取当前实时位置作为起始点
	getRealTimePoint((double*)&pSRun->SLastAbsCoord, AXIS_NUM_MAX, ucAxisMap, NULL);

    // 3. 遍历各轴计算脉冲
	for (i = 0; i < nArraySize; i++)
	{
        uint8 ucPhysicalAxis = ucAxisMap[i];
        double dPulseToMm = 0.0;
        double dDist = 0.0;
        int16 nPulse = 0;

        // 3.1 跳过无效轴或未启用的轴
        if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
        {
				continue;
        }

        // 1、如果指定了有效状态位，且当前轴无效，则跳过   2、有效轴状态位为空全部计算
        if (ucValStatus != NULL && VAL_VALID != getbit(*ucValStatus, i))
        {
            continue;
        }

        // 3.2 获取当前轴的脉冲当量
        if (bUseSysPara)
        {
            dPulseToMm = g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm;
		}
		else
		{
            dPulseToMm = dPulseToMmArray[i];
        }

        // 3.3 【核心修复】校验脉冲当量的有效性
        // 必须是非零、有限数值，且大于最小精度 EPSINON
        if (!isfinite(dPulseToMm) || isZero(dPulseToMm))
        {
			initSRunPulseBuf(pSRun, 0x3F); // 清零所有轴脉冲
            parseRunningError(9999, GCODE_ERROR_NUM153, 9999); // 上报错误：脉冲当量异常

            return FALSE; //返回失败
        }

        // 3.4 计算本 1ms 周期的位移量
        dDist = pdTargetPoint[i] - pSRun->SLastAbsCoord.dArray[i];

        // 3.5 计算脉冲数 (四舍五入)
		//nPulse = (int16)round(dDist / dPulseToMm);
        
        // 可选：在此处增加脉冲溢出检查 (例如超过 int16 范围)，若需要可调用 calcPulseWithCheck
        if (calcPulseWithCheck(dDist, dPulseToMm, &nPulse) != 1) //位移除以脉冲当量 dPulseToMm，计算本周期脉冲增量 nPulse
		{ 
			return FALSE; 
		}
        // 写入 pSRun 的 1ms 增量脉冲数组中
        pSRun->sAxisDeltaPulse[i] = nPulse;

        // 3.6 修正该点绝对坐标 (根据上一点坐标和脉冲增量反算，保证一致性)
        pSRun->SAbsCoord.dArray[i] = pSRun->SLastAbsCoord.dArray[i] + (double)nPulse * dPulseToMm;
	}

    // 4. 更新实时坐标位置
	updateRealTimePoint((double*)&pSRun->SAbsCoord, AXIS_NUM_MAX, ucAxisMap, ucValStatus);

	return TRUE;
}

/* 复位保留主轴时，只豁免仍在主轴模式的逻辑轴；同轴作进给轴时仍应停。 */
static bool isResetSpindleKept(const struct SChannel* pSChannel, uint8 ucLogicalAxis)
{
    const struct SSingleAxis* pMainAxis = pSChannel->SSpindle.pSSpindleAxis;
    const struct SSingleAxis* pSecondAxis = pSChannel->SSecondSpindle.pSSpindleAxis;
    return pSChannel->bIsStopSpindleAndCoolantwhileReset
        && ((pMainAxis != NULL && pMainAxis->ucAxis == ucLogicalAxis && pMainAxis->ucRunMode != SINGLEAXIS_MODE)
            || (pSecondAxis != NULL && pSecondAxis->ucAxis == ucLogicalAxis && pSecondAxis->ucRunMode != SINGLEAXIS_MODE));
}

/*************************************************
  函数名称：calculatePWMValue()
  函数功能：将本拍通道逻辑轴脉冲增量经两级映射转换为硬件PWM定时器参数与方向电平
  输入参数：pSRun         运行结构体指针（读取sAxisDeltaPulse）
			ucAxisMap     第一级映射（逻辑轴 -> 物理轴）
			ucPWMAxisMap  第二级映射（物理轴 -> PWM输出槽）
			isCompFlag    是否启用反向间隙补偿
  输出参数：更新以下5个全局数组（下标均为目标PWM槽位）：
			g_usReloadValue(ARR)    = 50000000 / (|pulse| * 1000) - 1（50MHz、1ms发|pulse|个脉冲的单周期计数值）
			g_usCompareValue(CCR)   = ARR / 2（50%占空比；无脉冲时置0关输出）
			g_usCounterValue(CNT)   = 50000 - usTimeComp[|pulse|]（补偿整除截断余数，对齐1ms内脉冲总数）
			g_cMotionDirection(方向)= (pulse * ucMotorphase) > 0 ? 0 : 1（0:正转 1:反转）
			g_sAxisDeltaPulse(脉冲) = 落到目标PWM槽位的本拍带符号实际脉冲数
  返 回 值：无
  备    注：重点关注参数：算法/仿真定位核对重点看 g_sAxisDeltaPulse；板端硬件发波重点看 g_usReloadValue(ARR)、g_usCompareValue(CCR)、g_usCounterValue(CNT) 与 g_cMotionDirection。
**************************************************/
void calculatePWMValue(struct _SRun* pSRun, const uint8* ucAxisMap, const uint8* ucPWMAxisMap, bool isCompFlag)
{
	int8 cCounter;
	int32 nPulseValue;
	if (pSRun->pSChannel->ucChannelNum == 0)	//仅通道0每拍清零上一拍全局输出，避免覆盖多通道数据
	{
		memset(g_usCompareValue, 0, sizeof(g_usCompareValue));
		memset(g_sAxisDeltaPulse, 0, sizeof(g_sAxisDeltaPulse));
	}

	for (cCounter = 0; cCounter < AXIS_NUM_MAX; cCounter++)
	{
		if (ucAxisMap[cCounter] >= AXIS_PHYSICS_NUM_MAX)
		{
			continue;
		}

		if (!(pSRun->pSChannel->SSpindle.cSpindleClampStatus == STATUS_UNCLAMPED || pSRun->pSChannel->SSpindle.cSpindleClampStatus == STATUS_GET_CLAMP_INSTRUCTION) && AXIS_C == cCounter)	//主轴夹紧或夹紧中禁止C轴输出脉冲
		{
			if (pSRun->sAxisDeltaPulse[cCounter] != 0)
			{
				parseRunningError(9999, GCODE_ERROR_NUM148, 9999);
				pSRun->sAxisDeltaPulse[cCounter] = 0;
				g_sAxisDeltaPulse[ucAxisMap[cCounter]] = pSRun->sAxisDeltaPulse[cCounter];
			}
		}

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE	//反向间隙误差补偿
	if (isCompFlag)
	{
		// zxr: 复位、急停时保持旧补偿；复位保留的主轴仍按原流程运行
		if (pSRun->pSChannel->ucChannelStatus == SYS_EMERGENCY_STOP
			|| (pSRun->pSChannel->ucChannelStatus == SYS_RESET
				&& !isResetSpindleKept(pSRun->pSChannel, cCounter)))
		{
			holdRevCompensationAPI(ucAxisMap[cCounter]);
		}
		else
		{
			pSRun->sAxisDeltaPulse[cCounter] += getCompensationPulseAPI(ucAxisMap[cCounter], nSgn(pSRun->sAxisDeltaPulse[cCounter]));
		}
	}
#endif
		if (pSRun->sAxisDeltaPulse[cCounter] != 0)  //脉冲非0：按50MHz时钟、1ms周期计算ARR与50%占空比CCR
		{
			g_usReloadValue[ucAxisMap[cCounter]] = 50000000 / (abs(pSRun->sAxisDeltaPulse[cCounter]) * 1000) - 1;
			g_usCompareValue[ucAxisMap[cCounter]] = g_usReloadValue[ucAxisMap[cCounter]] / 2;
			g_sAxisDeltaPulse[ucAxisMap[cCounter]] = pSRun->sAxisDeltaPulse[cCounter];
		}
		else if (pSRun->pSChannel->ucChannelNum == 0 && pSRun->sAxisDeltaPulse[cCounter] == 0)	//通道0无脉冲输出时清零该物理轴输出
		{
			g_usCompareValue[ucAxisMap[cCounter]] = 0;
			g_sAxisDeltaPulse[ucAxisMap[cCounter]] = pSRun->sAxisDeltaPulse[cCounter];
		}
	}

	//第二级映射：将按物理轴写入的结果重排到最终PWM硬件输出槽位
	uint16 g_usTempCompareValue[AXIS_PHYSICS_NUM_MAX];
	uint16 g_usTempReloadValue[AXIS_PHYSICS_NUM_MAX];
	int16 g_sTempAxisDeltaPulse[AXIS_PHYSICS_NUM_MAX];
	for (cCounter = 0; cCounter < AXIS_NUM_MAX; cCounter++)
	{
		g_usTempCompareValue[cCounter] = g_usCompareValue[cCounter];
		g_usTempReloadValue[cCounter] = g_usReloadValue[cCounter];
		g_sTempAxisDeltaPulse[cCounter] = g_sAxisDeltaPulse[cCounter];
		g_usCompareValue[cCounter] = 0;
		g_usReloadValue[cCounter] = 0;
		g_sAxisDeltaPulse[cCounter] = 0;
	}
	for (cCounter = 0; cCounter < AXIS_NUM_MAX; cCounter++)
	{
		if (ucPWMAxisMap[cCounter] >= AXIS_PHYSICS_NUM_MAX)
		{
			continue;
		}
		g_usCompareValue[ucPWMAxisMap[cCounter]] = g_usTempCompareValue[cCounter];
		g_usReloadValue[ucPWMAxisMap[cCounter]] = g_usTempReloadValue[cCounter];
		g_sAxisDeltaPulse[ucPWMAxisMap[cCounter]] = g_sTempAxisDeltaPulse[cCounter];
	}

	//结合电机极性计算方向电平（0:正转 1:反转）
	for (cCounter = 0; cCounter < AXIS_NUM_MAX; cCounter++)
	{
		if (ucPWMAxisMap[cCounter] >= AXIS_PHYSICS_NUM_MAX)
		{
			continue;
		}
		if(g_sAxisDeltaPulse[ucPWMAxisMap[cCounter]] != 0)
		{
			g_cMotionDirection[ucPWMAxisMap[cCounter]] = (g_sAxisDeltaPulse[ucPWMAxisMap[cCounter]] * g_SSystemPara.ucMotorphase[ucPWMAxisMap[cCounter]]) > 0 ? 0:1;
		}
	}

	//查表补偿整数除法截断余数，计算CNT初值以对齐1ms内脉冲总数
	for (cCounter = 0; cCounter < AXIS_NUM_MAX; cCounter++)
	{
		if (ucPWMAxisMap[cCounter] >= AXIS_PHYSICS_NUM_MAX)
		{
			continue;
		}
		nPulseValue = abs((int32)g_sAxisDeltaPulse[ucPWMAxisMap[cCounter]]);
		nPulseValue = nPulseValue > 1001 ? 0 : nPulseValue;
		g_usCounterValue[ucPWMAxisMap[cCounter]] = 50000 - usTimeComp[nPulseValue];
	}

}

/*************************************************
  函数名称：calcPulseWithCheck(double dSpeed, double dPulseToMm, int16* pSpindlePulse)
  功能描述：计算脉冲增量，并进行 int16 范围检测及越界报警
  输入参数： dSpeed          当前速度 (度/ms 或 r/ms，需与 dPulseToMm 单位匹配)
            dPulseToMm      脉冲当量 (度/pulse 或 mm/pulse)

  输出参数： pSpindlePulse   脉冲数
  返回值：   错误码
  注意：
            如果发生越界，会调用 parseRunningError 报警。
**************************************************/
int16 calcPulseWithCheck(double dSpeed, double dPulseToMm, int16* pPulse)
{
    int32 tempPulse;
    
    // 防止除以零
    if (isZero(dPulseToMm) || !isfinite(dPulseToMm)) 
	{
		*pPulse = 0;
        parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
        return -1;
    }

    // 计算原始脉冲值，使用 int32 接收 round 的结果以防止溢出
    tempPulse = (int32)round(dSpeed / dPulseToMm);

    // 饱和处理及报警
    if (tempPulse > 32767 || tempPulse < -32768) 
	{
        *pPulse = 0;
        parseRunningError(9999, GCODE_ERROR_NUM152, 9999); 
		return -1;
    } 
	if(fabs(tempPulse) > 1000)
	{
		tempPulse = 0;
	}
	*pPulse = (int16)tempPulse;
    return 1;
}

/*************************************************
  函数名称：isMotionChangeDirection()
  功能描述：将当前通道是否有轴反向，并且若有轴则重算PWM值
  输入参数：ucChannelNum - 通道号
  返回值：无
  备注：增加2026.2.7
****************************************/
bool isMotionChangeDirection(uint8 ucChannelNum)
{
	int8 cCounter;
	uint8* ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;
	static int8  cMotionDir[CHANNEL_NUM_MAX][AXIS_NUM_MAX] = {0}; //1为正转 -1为反转，初始化为0，表示未确定
	bool bDirectionChanged = FALSE;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
   		 return FALSE; // 或者返回错误码
	}
	for (cCounter = 0; cCounter < AXIS_NUM_MAX; cCounter++)
	{
		//跳过无效轴
		if (ucMapArray[cCounter] >= AXIS_PHYSICS_NUM_MAX)
		{
			continue;
		}
		int8 currentDir = sgn(g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[cCounter]);		//获取当前轴方向
		if (cMotionDir[ucChannelNum][cCounter] != currentDir && g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[cCounter] != 0) //轴方向改变且脉冲不为0
		{
			cMotionDir[ucChannelNum][cCounter] = currentDir; //轴方向改变
			updateSingleAxisRealTimePoint(&g_SChannel->SRun.SLastAbsCoord.dArray[cCounter], cCounter, ucMapArray);//不输出脉冲，恢复轴坐标
			g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[cCounter] = 0;
			bDirectionChanged = TRUE; //返回方向已改变
		}
	}

    if (bDirectionChanged) //若修改方向，重新计算pwm
    {
		calculatePWMValue(&g_SChannel[ucChannelNum].SRun, g_SChannel[ucChannelNum].pMapAxisArray,g_SChannel[ucChannelNum].pPWMMapAxisArray,FALSE);//计算PWM值,不需要补偿,已经计算了补偿
        return TRUE; // 返回方向已改变
    }

	return FALSE;
}

/*************************************************
  函数名称：int8 readHandWheelSpeed(struct SChannel pSChannel)
  函数功能  读取通道手轮的转速和方向  （仅在手轮模拟时 执行）
  输入参数：pSChannel	通道结构体指针
  输出参数：
  返 回 值：-1	输入错误
			0	完成/通道没开启手轮模拟/非通道0
  备    注：
**************************************************/
int8 handWheelSpeedTest(struct SChannel* pSChannel)
{
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	if (isHandwheelAutomaticMoveRequired(pSChannel))
	{
		return prepareHandwheelAutomaticMove(pSChannel) < 0 ? -1 : 0;
	}
	/* 距离模式不生成测速滤波样本，也不参与旧方向投票。 */
	if (isHandwheelDistanceControlActive(pSChannel))
		return 0;
#endif

	struct _SEncoderTest* pEncoderTest;
	struct SEncoder* pSEncoder;
	int nTempTotal;
	double dShrinkfactor = 1;//倍率调小系数（低速时用）

	if (pSChannel == NULL || pSChannel->pSHandleWheelEncoder == NULL)
	{
		return -1;
	}

#if(MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE)		//机器配置为双通道Y轴机
	if (pSChannel->ucChannelNum != 0)		//应企业要求，双通道Y轴机0通道和1通道共用同一个手轮，手轮模拟时的倍率也用同一个
	{
		return 0;
	}
#endif

	if (pSChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_OFF)//在开启手轮模拟的状态下才进行手轮转速检测
	{
		return 0;
	}

	pEncoderTest = &pSChannel->SHandWheelTest;//转速检测结构体
	pSEncoder = pSChannel->pSHandleWheelEncoder;//手轮结构体

	//pEncoderTest->dSpindleAvgSpeed = 

	
#if !STM32_MODE //模拟手轮  (根据宏定义赋值)
	//直接赋值
	pSEncoder->cEncoderDir = HANDWHEEL_DIR;
	pEncoderTest->dSpindleAvgSpeed = HANDWHEEL_DELTA;
	pEncoderTest->pSChannel->SRun.dHandWheelSpeedRatio = HANDWHEEL_DELTA / g_SSystemPara.fHandleWheelSpeedMax * HANDWHEEL_DIR;


#else
	if (g_SSystemPara.usHandleWheelAvgPeriod  > 1000) //设定周期不超过1秒
	{
		g_SSystemPara.usHandleWheelAvgPeriod = 1000;
	}
	switch (pEncoderTest->ucStep)
	{
	case 0:
		pEncoderTest->usPeriod = 0;//初始化周期
		pEncoderTest->nTotalPulse = 0;//初始化脉冲
		pEncoderTest->ucStep = 1;
	case 1:

		pEncoderTest->usPeriod++;//周期++
		pEncoderTest->nTotalPulse += pSEncoder->cEncoderDir * pSEncoder->usDelta;//脉冲数叠加

		if (pEncoderTest->usPeriod >= g_SSystemPara.usHandleWheelAvgPeriod)//计数达到目标计数
		{
			//反向瞬间的平滑处理   2024/3/19 陈敏聪 洪冠鹏
			if (pEncoderTest->nTotalPulse > 0 && pEncoderTest->dSpindleAvgSpeed < 0)//反向的瞬间做一次滤波
			{
				pEncoderTest->ucSmoothCount = 2;//方向瞬间平滑次数
				pEncoderTest->nTotalPulse = 0;//滤去手轮因刚性问题正向移动的脉冲
			}
			if (pEncoderTest->ucSmoothCount > 0)
			{
				pEncoderTest->ucSmoothCount--;
			}
			//滤波处理
			nTempTotal = fabs(pEncoderTest->nTotalPulse);
			if (nTempTotal == 1)//滤去一个脉冲
			{
				nTempTotal = pEncoderTest->nTotalPulse = 0;
			}
			if (nTempTotal <= 4 && nTempTotal > 0)	//手轮4倍频的情况下，如果检测到的脉冲小于4，当作一个脉冲
			{
				pEncoderTest->nTotalPulse = pSEncoder->cEncoderDir * 1;
				dShrinkfactor = 0.1;//为避免手轮低速摇进给量过大，将转速调小，从而调小倍率。 2025/4/11

			}

			//计算主轴平均转速  (pulse/ms)
			pEncoderTest->dSpindleAvgSpeed = dShrinkfactor * (double)pEncoderTest->nTotalPulse / pEncoderTest->usPeriod;	//2023/12/5	陈敏聪	编码器的转速计算位于滤波后

			if (pEncoderTest->nTotalPulse >= 0)
			{
				//计算倍率  (不可超过最大倍率)
				pEncoderTest->pSChannel->SRun.dHandWheelSpeedRatio = min(100 * pEncoderTest->dSpindleAvgSpeed / g_SSystemPara.fHandleWheelSpeedMax, MAX_HANDWHEEL_RATIO);
			}
			else
			{
				//计算倍率  (不可超过最大倍率)
				pEncoderTest->pSChannel->SRun.dHandWheelSpeedRatio = max(100 * pEncoderTest->dSpindleAvgSpeed / g_SSystemPara.fHandleWheelSpeedMax, -MAX_HANDWHEEL_RATIO);
			}
			pEncoderTest->ucStep = 0;//初始化
        }

#if(MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE || MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)		//机器配置为双通道Y轴机
		g_SChannel[1].SRun.dHandWheelSpeedRatio = pEncoderTest->pSChannel->SRun.dHandWheelSpeedRatio;//应企业要求，双通道Y轴机0通道和1通道共用同一个手轮，手轮模拟时的倍率也用同一个
#endif

		break;
	default:
		pEncoderTest->ucStep = 0;
	}
#endif // HANDWHEEL_SIMULATION


	return 1;
}
/*************************************************
  函数名称：judgeCanMoveBack(struct SToolCompensation* pSToolCompPointer)
  函数功能  判断是否满足手轮模拟反摇条件
  输入参数：*pSToolCompPointer，结构体struct SToolCompensation指针
  输出参数：
  返 回 值：0	不满足手轮手轮模拟反摇条件
			1	满足手轮手轮模拟反摇条件
  备    注：//2023/12/12	陈敏聪增
**************************************************/
int8 judgeCanMoveBack(struct SToolCompensation* pSToolCompPointer)
{
	if (pSToolCompPointer == NULL)//空指针  不运行
	{
		return -1;
	}

	if (pSToolCompPointer->bLastLineMark		//陈敏聪	2023/12/12 如果最后一行为M99，不允许反转
		|| hasIOCommand(pSToolCompPointer->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE)// 存在 M 代码 / 辅助 IO
#if CHANNEL_NUM_MAX > 1
		|| pSToolCompPointer->sChannelCommand.ucType != UNASSIGNED_VALUE// 存在多通道同步等待指令
#endif // CHANNEL_NUM_MAX > 1)			//反走的条件  无M代码  无通道指令
		)
	{
		return 0;
	}

	return 1;
}

/*************************************************
  函数名称：initInterpStructByTool
  函数功能：根据系统参数和刀补记录初始化插补预处理，消费G50/G10偏置事件
  输入参数：pSToolCompPointer 刀补记录；pSPrePointer 预处理结构；cInterpDir 插补方向
  输出参数：更新预处理结构；G10前向消费时累加实时刀表并清零本记录X/Z增量
  返 回 值：-1 输入错误；0 不允许反走；1 初始化完成
  备    注：同一刀补记录顺序重复初始化不重复累加G10；保留cPointType供历史边界判断
**************************************************/
int8 initInterpStructByTool(struct SToolCompensation* pSToolCompPointer, struct SInterpolationPre* pSPrePointer, int8 cInterpDir)
{
	uint8 ucCounter;
	double dRad;
	bool bReCalculate = TRUE;
	struct SG0* pSG0;
	struct SG1* pSG1;
	struct SG2* pSG2;
	struct _SG32* pSG32;
	struct _SSpindle* pSpindle;
	union  G50G10Info* pSG50Info = NULL;
	struct SG50RingBuff* pSG50RingBuff;

	if (pSToolCompPointer == NULL || pSPrePointer == NULL)//空指针  不运行
	{
		return -1;
	}
	if (cInterpDir == INP_GO_BACK)		//陈敏聪	2023/9/5  将这段代码移到函数最前面，且判断的对象改为刀补缓冲区而非预处理结构体
	{
		if (!judgeCanMoveBack(pSToolCompPointer))	//陈敏聪	2023/12/12	不满足手轮反摇条件就返回函数
		{
			pSPrePointer->ucInpStatus = INP_FINISHED;
			return 0;
		}
	}
	//pSPrePointer->ucInpStatus = INP_INITIATION;//插补预处理结构体状态为初始化
	pSpindle = &pSPrePointer->pSChannel->SSpindle;
	/*******************从系统参数获取*********************/
	pSPrePointer->SUserData.ucDistReAverageType = g_SSystemPara.ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数
	pSPrePointer->SUserData.ucTconstForTrapezoid = g_SSystemPara.ucTconstForTrapezoid;//三角形修调后匀速时间占总周期的百分比 （ % ） 洪冠鹏 2024/3/22
	/*******************从前瞻结构体获取*******************/
	strncpy(pSPrePointer->cFilename, pSToolCompPointer->cFilename, FILENAME_LENGTH_LIMIT);
	pSPrePointer->unGCodeLineNum = pSToolCompPointer->unGCodeLineNum;		//解析到第几行G代码
	pSPrePointer->unCommandType = pSToolCompPointer->unCommandType;			//指令类型
	pSPrePointer->SUserData.SStartPoint = pSToolCompPointer->SStartPoint;	//起点，单位：mm
	pSPrePointer->SUserData.SEndPoint = pSToolCompPointer->SEndPoint;		//起点，单位：mm
	pSPrePointer->SUserData.SCenterPoint = pSToolCompPointer->SCenterPoint;	//圆心，单位：mm
	pSPrePointer->SUserData.cArcDir = pSToolCompPointer->cArcDir;			//圆弧方向，顺时针=1，逆时针=-1
	pSPrePointer->SUserData.dRadius = pSToolCompPointer->dRadius;			//半径，单位：mm
	pSPrePointer->SUserData.dFeedrate = pSToolCompPointer->dFeedrate;		//进给速率
	pSPrePointer->SUserData.SStartVector = pSToolCompPointer->SStartVector;	//起点矢量 （必须为单位矢量）
	pSPrePointer->SUserData.dAcc = pSToolCompPointer->dAcc;	 				//加速段加速度	mm/ms2
	pSPrePointer->SUserData.dDec = pSToolCompPointer->dDec;	 				//减速段加速度	mm/ms2
	pSPrePointer->SUserData.ucAccVelPlanType = pSToolCompPointer->ucAccVelPlanType;		//速度曲线类型，1：梯形，2：3次S型，3：4次S型
	pSPrePointer->SUserData.ucDecVelPlanType = pSToolCompPointer->ucDecVelPlanType;		//速度曲线类型，1：梯形，2：3次S型，3：4次S型
	pSPrePointer->SUserData.dVs = pSToolCompPointer->dVs;					//起点速度,单位:mm/ms
	pSPrePointer->SUserData.dPrimalVe = pSToolCompPointer->dVe;				//终点速度,单位:mm/ms
	if(pSPrePointer ->pSChannel->bSingleLineMode == TRUE) //在单节执行下，运动不以速度前瞻数据来来规划，而是取起跳速度，2026.6.5
	{
		pSPrePointer->SUserData.dPrimalVe = pSToolCompPointer->dMinVe;
	}
	pSPrePointer->SUserData.dG84Ratio = pSToolCompPointer->dG84Ratio;		//用于G84倍率 2025.11.15
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	pSPrePointer->SUserData.bHandwheelAuto = pSToolCompPointer->bHandwheelAuto;
#endif

	pSPrePointer->SUserData.dHorMax = pSToolCompPointer->dHorMax;//获取圆弧轨迹上横轴方向的最大值  洪冠鹏 2024/4/19
	pSPrePointer->SUserData.dHorMin = pSToolCompPointer->dHorMin;//获取圆弧轨迹上横轴方向的最小值
	pSPrePointer->SUserData.dVerMax = pSToolCompPointer->dVerMax;//获取圆弧轨迹上纵轴方向的最大值
	pSPrePointer->SUserData.dVerMin = pSToolCompPointer->dVerMin;//获取圆弧轨迹上纵轴方向的最小值

	pSPrePointer->SUserData.dArcMaxNormalAccVmax = pSToolCompPointer->dArcMaxNormalAccVmax;//圆弧法向加速度约束下的最大速度    2022/11/29郭强
	pSPrePointer->SUserData.dArcMaxChordErrorVmax = pSToolCompPointer->dArcMaxChordErrorVmax;//圆弧弓高误差约束下的最大速度     2022/11/29郭强
	pSPrePointer->SUserData.dCommonVMaxFromSingleAxis = pSToolCompPointer->dCommonVMaxFromSingleAxis;//G1，G2,G3复合运动最大速度由各插补轴计算得到     2022/11/29郭强
	pSPrePointer->SUserData.dUncorrectedFeedrate = pSToolCompPointer->dUncorrectedFeedrate;//未修正的进给速度  mm/ms   2022/11/29郭强
	pSPrePointer->SUserData.dMinVs = pSToolCompPointer->dMinVs;//起点最小速度,单位:mm/ms       2022/11/29郭强
	pSPrePointer->SUserData.dMinVe = pSToolCompPointer->dMinVe;//终点最小速度,单位:mm/ms       2022/11/29郭强
	pSPrePointer->SUserData.ucAxisNumOfNeedLookAhead = pSToolCompPointer->ucAxisNumOfNeedLookAhead; //需要前瞻的单轴轴号

	pSPrePointer->ulDelayTime = pSToolCompPointer->ulDelayTime;				//延时指令G04
	pSPrePointer->ucToolNum = pSToolCompPointer->SToolPara.ucToolID;		//刀具信息
	pSPrePointer->ucToolCompNum = pSToolCompPointer->ucToolCompNum;		//磨损组号
	pSPrePointer->cWorkpieceCoordNum = pSToolCompPointer->cWorkpieceCoordNum;//工件坐标号
	pSPrePointer->bG50Appear = pSToolCompPointer->bG50Appear;			//G50传入标志位 2023-8-3洪冠鹏
	pSPrePointer->bG65SystemMacroWait = pSToolCompPointer->bG65SystemMacroWait;

	pSToolCompPointer->bG65SystemMacroWait = FALSE;//复位标志位 2023.11.23 陈育楠
	pSPrePointer->ucG28ZeroingValAxis = pSToolCompPointer->ucG28ZeroingValAxis;//G28回零的有效轴 2024.12.1 cyn

	pSPrePointer->SUserData.dDistance = pSToolCompPointer->dLength;			//插补距离	mm
	pSPrePointer->SUserData.dPreDist = pSToolCompPointer->dPreDist;			//已走距离

	pSPrePointer->ucRunStep = 0;											//运行步骤
	pSPrePointer->SUserData.ucValStatus = pSToolCompPointer->ucValStatus;  //有效轴
	pSPrePointer->SUserData.ucRelaVal = pSToolCompPointer->ucRelaVal;		//相对坐标标志位
	pSPrePointer->cChannelBufNum = pSToolCompPointer->cChannelBufNum;		//通道等待信号下标
	pSPrePointer->sChannelCommand = pSToolCompPointer->sChannelCommand; //通道指令

	pSPrePointer->bLastLineMark = pSToolCompPointer->bLastLineMark; //  最后一行标志位
	pSPrePointer->SUserData.dFeedRatio = -1;  // zxr 在 G1.c 的 loadSG1DataFromInput() 和 motionLine() 的倍率处理中使用

	//朱稳中2023.3.29
	pSPrePointer->SUserData.dEllipseFactorA = pSToolCompPointer->dEllipseFactorA;//椭圆参数A
	pSPrePointer->SUserData.dEllipseFactorB = pSToolCompPointer->dEllipseFactorB;//椭圆参数B
	pSPrePointer->SUserData.dCoordRotation = deg2rad(pSToolCompPointer->dQ);// 坐标系旋转弧度  rad
	pSPrePointer->SUserData.fCylinderRadius = pSToolCompPointer->fCylinderRadius;// 圆柱半径

	pSPrePointer->unChipBreakDelayTime = pSToolCompPointer->unChipBreakDelayTime;//断屑暂停时间
	pSPrePointer->dChipBreakLength = pSToolCompPointer->dChipBreakLength;   //断屑间隔长度
	pSPrePointer->bChipBreakFlag = pSToolCompPointer->bChipBreakFlag;       //断屑标志位

	pSpindle->dLimitedSpindleSpeed = pSToolCompPointer->dLimitedSpindleSpeed;//赋值限制速度
	if (getBits(pSPrePointer->unCommandType, CMD_SPINDLE_SPEED_ADDR) == CMD_SPINDLE_SPEED_CV)//赋值主轴转速
	{
		pSpindle->dSpindleSpeed = pSToolCompPointer->dSpindleSpeed;//转速
	}
	else
	{
		pSpindle->dSpindleLineSpeed = pSToolCompPointer->dSpindleSpeed;//线速度
	}

	pSPrePointer->dSecondSpindleSpeed = pSToolCompPointer->dSecondSpindleSpeed;	//第二转轴/动力刀转速	//陈敏聪 2023/3/6改

	//刀具轴预处理 刀具轴模式切换
	if (pSToolCompPointer->cToolAxisMod != 0)
	{
		if (pSToolCompPointer->cToolAxisMod == 1)
		{
			//刀具轴模式转为跟随模式
			g_SToolAxisInfo.pSToolAxis->ucRunMode = TOOLAXIS_FOLLOW_MODE;
			//更新跟随比
			g_SToolAxisInfo.dCommandRatio = pSToolCompPointer->dToolAxisFollowRatio;
			//更新相位角
			g_SToolAxisInfo.dCommandAngle = pSToolCompPointer->dToolAxisPhase;
		}
		else if (pSToolCompPointer->cToolAxisMod == 2)
		{
			//刀具轴模式转为减速停模式
			g_SToolAxisInfo.pSToolAxis->ucRunMode = TOOLAXIS_COMMONSTOP_MODE;

			//插补状态
			g_SToolAxisInfo.pSToolAxis->ucInpStatus = INP_INITIATION;

		}
	}
	g_SToolAxisInfo.cTwistCommand = pSToolCompPointer->cTwistCommand;//刀具轴扭转指令（G0,G1,G2）
	g_SToolAxisInfo.dCommandTwistAngle = pSToolCompPointer->dTwistAngle;//刀具轴扭转角
	//G50偏置
	if (pSToolCompPointer->cPointType == G50)
	{
		//更新系统参数
		g_SZeroPtOfCoordSys->SWorkPieceOffsetByG50 = pSToolCompPointer->SPointInfo;
	}
	//G10磨耗：前向执行时，将本条增量累加到实时刀表
    else if (pSToolCompPointer->cPointType == G10)
	{
        g_SSystemPara.SWearPara[pSPrePointer->pSChannel->ucChannelNum]
			[pSToolCompPointer->uG10ToolID - 1].SToolCoord =
            pSToolCompPointer->SPointInfo;
	}

	if (pSPrePointer->bG65SystemMacroWait == TRUE)
	{
		pSPrePointer->pSChannel->cG65SystemMacroRun = TRUE;
		pSPrePointer->bG65SystemMacroWait = FALSE;//复位标志位 2023.11.23 陈育楠
	}

#if CHANNEL_NUM_MAX >1 //多通道系统 开启刀补轴占用
#if(TURRET_AXIS_TEST == 0)
	{


		if ((pSPrePointer->ucToolCompensationStatus == TOOL_COMPENSATION_IDLE || pSPrePointer->ucToolCompensationStatus == TOOL_COMPENSATION_CANCEL)
			&& getBits(pSPrePointer->unCommandType, CMD_TOOLCOMP_ADDR) != CMD_TOOLCOMP_CANCEL)//刀补状态为空  指令类型为G41或G42
		{	//陈敏聪	2023/9/22
			//建立刀补轴占用
			uint8 ucAxis[2] = { 0 };
			chooseInterpPlane(ucAxis, pSPrePointer->unCommandType);//根据指令类型（插补平面）获取轴号

			axisManagerReleaseByStruct(pSPrePointer->ucToolAxis, pSPrePointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, NULL);//释放刀补的轴（释放旧的刀补轴）
			pSPrePointer->ucToolAxis = 0; //清空
			//改变刀补状态
			pSPrePointer->ucToolCompensationStatus = TOOL_COMPENSATION_BUILD;
			releaseLastCommandAxis(pSPrePointer);//释放上一条指令的轴占用 （不然会导致刀补无法占用,例如更换插补平面的状况）
			//根据插补平面的轴  初始化刀补的轴占用
			setbit(pSPrePointer->ucToolAxis, ucAxis[0]);
			setbit(pSPrePointer->ucToolAxis, ucAxis[1]);
		}

		if (pSPrePointer->ucToolCompensationStatus != TOOL_COMPENSATION_IDLE && getBits(pSPrePointer->unCommandType, CMD_TOOLCOMP_ADDR) == CMD_TOOLCOMP_CANCEL)
		{	//旧刀补状态不为IDLE，且刀补指令为G40（取消刀补）

			switch (pSPrePointer->ucToolCompensationStatus)//根据当前刀补的状态执行
			{
			case TOOL_COMPENSATION_PROCEED://刀补进行中
				pSPrePointer->ucToolCompensationStatus = TOOL_COMPENSATION_CANCEL;//将状态改为取消第一阶段
				break;

			case TOOL_COMPENSATION_CANCEL://刀补取消第一阶段
				axisManagerReleaseByStruct(pSPrePointer->ucToolAxis, pSPrePointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, NULL);//释放刀补的轴
				pSPrePointer->ucToolAxis = 0;//清空刀补的有效轴
				pSPrePointer->ucToolCompensationStatus = TOOL_COMPENSATION_IDLE;//刀补置为空闲
				break;

			default:
				break;
}
		}
	}
#endif//  TURRET_AXIS_TEST==0
#endif 

	//IO类型 M代码赋值
	for (ucCounter = 0; ucCounter < IO_MAXGROUP_IN_SINGLE_LINE; ucCounter++)
	{
		//从刀补缓冲区继承IO指令
		pSPrePointer->ucIOCommandArray[ucCounter] = pSToolCompPointer->ucIOCommandArray[ucCounter];
	}

	if (findIO(5, pSPrePointer->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 5 || findIO(52, pSPrePointer->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 52)
	{
		uint8 ucCounter = 0;
		pSPrePointer->preciseStopDegree = pSToolCompPointer->dQ;// 根据Q值赋值准停角度
		pSPrePointer->bValidQ = pSToolCompPointer->bValidQ;    // Q值有效标志位  2024/7/12
		// zxr: 仅无有效Q时删除重复M5，保留静止主轴的准停请求
		if(getMCodeStatusAPI(pSPrePointer->pSChannel->ucChannelNum,MCODE_GROUP_SPINDLE) == 5 && pSPrePointer->bValidQ == FALSE) //主轴已经是M5情况下 //2026.7.2
		{
			if(findIOPosition(5, pSPrePointer->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE,&ucCounter) == 5) //找到M5对应索引值
			{
				pSPrePointer->ucIOCommandArray[ucCounter] = 0; // 删除M5指令
			}

		}
	}

	//根据方向选择数据的处理方式
	if (cInterpDir != INP_GO_BACK)//正走(不是反走)
	{
		pSPrePointer->SUserData.cDealType = isZero(pSPrePointer->SUserData.dPreDist) ?
			DEALTYPE_RECAL_WRITEBACK //一般处理方式  重算 回写
			: DEALTYPE_NO_PROCESS;	 //已走距离不为0  说明已经重算过 此时不进行处理  直接运行
		pSPrePointer->SUserData.cInterpDir = INP_GO_AHEAD;
	}
	else// 反走
	{
		pSPrePointer->SUserData.cDealType = DEALTYPE_NO_PROCESS;//不进行处理
		pSPrePointer->SUserData.cInterpDir = INP_GO_BACK;
	}

	switch (getBits(pSPrePointer->unCommandType, CMD_MOTION_ADDR))//指令类型判断
	{
	case CMD_MOTION_QUICKMOVE://快速运动
		//if (pSToolCompPointer->EntityType == DOT) break;
		pSG0 = &pSPrePointer->pSChannel->SG00;
		//陈敏聪	2023/10/25	应企业要求，主轴还没停下来时获取下一行为G0,且存在C轴定位的信息的话，主轴以C轴的定位作为准停的角度
		if ((getbit(pSPrePointer->SUserData.ucValStatus, AXIS_C)) && (pSPrePointer->pSChannel->SSpindle.pSSpindleAxis->ucRunMode == SPINDLE_COMMONSTOP_MODE || pSPrePointer->pSChannel->SSpindle.pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE))
		{
			clrbit(pSPrePointer->SUserData.ucValStatus, AXIS_C);
			pSPrePointer->pSChannel->SSpindle.fPreciseStopDegree = surplusDoubleData(pSPrePointer->SUserData.SEndPoint.dArray[AXIS_C], 360);
			pSPrePointer->pSChannel->SSpindle.pSSpindleAxis->ucRunMode = SPINDLE_PRECISESTOP_MODE;//设置准停状态
			pSPrePointer->pSChannel->SSpindle.pSSpindleAxis->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		}
		//pSG0->SUserData = pSPrePointer->SUserData;
		pSG0->bIsUserData = FALSE; // 不是用户数据  从预处理读取数据
		pSG0->ucRunStep = 0;
		break;
	case CMD_MOTION_LINE://合成运动
		pSG1 = &pSPrePointer->pSChannel->SG01;
		//if (pSToolCompPointer->EntityType == DOT) bReCalculate = FALSE; //判断是否重算数据  为点时不重算?

		//赋值G01
		pSG1->unCommandType = pSPrePointer->unCommandType;// 指令类型
		//pSG1->SUserData = pSPrePointer->SUserData;//赋参数
		//pSG1->SUserData.dFeedRatio = -1;// -1值为无效值
		pSG1->bIsUserData = FALSE;//使G1读取预处理结构体数据
		pSG1->ucInpStatus = INP_OCCUPATION; //进入轴占用流程
		break;

	case CMD_MOTION_ARC://顺圆
	case CMD_MOTION_ANTI_ARC://逆圆
	case CMD_MOTION_ELLIPSE_ARC:
	case CMD_MOTION_ELLIPSE_ANTI_ARC:
		pSG2 = &pSPrePointer->pSChannel->SG02;
		//if (pSToolCompPointer->EntityType == DOT) bReCalculate = FALSE;//判断是否重算数据  为点时不重算?

		//赋值G02
		pSG2->dRad = pSToolCompPointer->dArcEndAngle - pSToolCompPointer->dArcStartAngle;  //弧度
		pSG2->unCommandType = pSPrePointer->unCommandType; //指令类型
		//pSG2->SUserData = pSPrePointer->SUserData;//赋参数
		//pSG2->SUserData.dFeedRatio = -1;// -1值为无效值
		pSG2->bIsUserData = FALSE;// 不是用户数据  从预处理读取数据
		pSG2->ucInpStatus = INP_OCCUPATION; //进入轴占用流程

		break;
	case CMD_MOTION_SCREWCUT:  //螺纹切削
		pSG32 = &pSPrePointer->pSChannel->SG32;
		pSG32->unCommandType = pSPrePointer->unCommandType;  //指令类型
		chooseInterpPlaneForG32(pSG32);
		loadSG32DataFromInput(pSToolCompPointer, &pSPrePointer->SUserData, pSG32, NULL);  //G32参数初始化
		if (pSG32->bG34BackTailFlag)
		{
			pSG32->ucLastAxis[0] = pSG32->ucAxis[0];
			pSG32->ucLastAxis[1] = pSG32->ucAxis[1];

		}
		break;
	case CMD_MOTION_SCREWCUT_VP:
	{
		pSG32 = &pSPrePointer->pSChannel->SG32;
		pSG32->SG34EndPoint = pSPrePointer->SUserData.SEndPoint;

	}
	break;
	case CMD_MOTION_G33://G33螺纹攻牙循环
		//G33结构体参数
		G33initStructByTool(pSToolCompPointer, pSPrePointer);
		break;
	case CMD_MOTION_G10://自动刀具补偿功能
	{
		pSG50Info = getSG50PointerByType(SG50_READ, 0);//获取G10写入指针
		if (pSPrePointer->unGCodeLineNum == pSG50Info->SG10Info.unG10LineNum)
		{
			pSPrePointer->SToolCMPByG10 = pSG50Info->SG10Info.SToolCMPByG10;//获取当前G10的刀具磨损量
			pSPrePointer->uG10ToolID = pSToolCompPointer->uG10ToolID;
			moveSG50Pointer(SG50_READ, 0);//写入操作完成后，移动G10写入指针指向下一个G10
		}
		if (pSPrePointer->SToolCMPByG10.dX != 0)
		{
			inputToolWear(pSPrePointer->pSChannel->ucChannelNum, pSToolCompPointer->ucToolCompNum, AXIS_X, pSPrePointer->SToolCMPByG10.dX, TRUE);
		}
		if (pSPrePointer->SToolCMPByG10.dZ != 0)
		{
			inputToolWear(pSPrePointer->pSChannel->ucChannelNum, pSToolCompPointer->ucToolCompNum, AXIS_Z, pSPrePointer->SToolCMPByG10.dZ, TRUE);
		}
		if (pSPrePointer->ucToolNum == pSPrePointer->uG10ToolID)
		{
			updataTotalOffset(pSPrePointer->pSChannel->ucChannelNum, 0, TRUE);// 更新当前总偏移
		}
		break;
	}
	break;
	default:
		break;

		};

#if EN_DEBUG
	printf("起点:Z: %lf    X: %lf \n", pSToolCompPointer->SStartPoint.dZ, pSToolCompPointer->SStartPoint.dX);
	printf("终点:Z: %lf    X: %lf \n", pSToolCompPointer->SEndPoint.dZ, pSToolCompPointer->SEndPoint.dX);
	printf("线段长度:dlength : %lf \n", pSToolCompPointer->dLength);
	printf("起点矢量: Z: %lf    X: %lf \n ", pSToolCompPointer->SStartVector.dIZ, pSToolCompPointer->SEndVector.dIX);
	printf("指令类型:%d\n", pSToolCompPointer->unCommandType);
#endif

	return 1;
	}

//陈敏聪 2023/4/19 暂时注释
///*************************************************
//  函数名称：interpolationPre()
//  函数功能  判断"规划读"指针是否有内容可读，没有-返回报错
//			有-根据读出的参数进行插补预处理
//  输入参数：
//  输出参数：
//  返 回 值：
//			3:  预处理成功 不执行IO
//			2:  预处理成功 执行IO
//			1:  暂停
//			0:  预处理失败
//			-1: 缓存不可读，预处理失败
//			-2: 不应该出现的系统状态
//			-3	空指针
//  备    注：
//**************************************************/
//int8 interpolationPre(struct SInterpolationPre* pSPreProcess)
//{
//	struct SToolCompensation* pSToolCompPointer = NULL;   //刀补后产生的结构体指针
//	struct SChannel* pSChannel;
//	enum SYSTEMSTATUS* pSystemStatus;//系统状态
//	struct _SRun* pSRun;//运行结构体
//	enum RingBufPointerType PointerType;//指针类型
//	bool bApplyPreIO;//是否执行前作用IO
//	int nCount = 0;
//
//	if (pSPreProcess == NULL)
//	{
//		return -3;
//	}
//
//	pSChannel = pSPreProcess->pSChannel;//通道指针
//	pSystemStatus = &pSChannel->ucChannelStatus;//通道状态
//	pSRun = &pSChannel->SRun;//运行结构体指针
//
//	//根据插补方向选择不同的指针类型
//	PointerType = (pSPreProcess->cInterpDir != INP_GO_BACK) ? INTERPOLATION_READ : INTERPOLATION_BACK;
//	
//	if (*pSystemStatus == SYS_PAUSE_TO_RUN) // PAUSE TO RUN 不能进入该函数 返回
//	{
//		return -2;
//	}
//
//	if (*pSystemStatus == SYS_RUN_TO_PAUSE) // PAUSE TO RUN 无前瞻模式下 当前行能停下
//	{
//		savePausePoint(pSPreProcess);// 保存暂停点
//		*pSystemStatus = SYS_PAUSE; //修改系统状态
//		pSPreProcess->ucInpStatus = INP_FINISHED;
//		return 1;
//	}
//
//
//	if (getSToolCompPointerBufRes(PointerType, pSChannel->ucChannelNum) <= 0)//若"速度规划读"指针没有可读内容
//	{
//		if (g_RunMode == MDI_MODE)
//		{
//			if (pSChannel->bGcodeWriteOverFlag == TRUE && pSChannel->bToolWriteOverFlag == TRUE && getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum) <= 0)
//			{// MDI 模式下无内容可读  系统状态置为FINISHED
//				*pSystemStatus = SYS_IDLE;
//			}
//		}
//
//		if (*pSystemStatus == SYS_RUN) //插补方向为正时，进入该语句 说明处理不过来，反馈错误  //插补方向为负时，进入该语句说明不能再倒退，此时判断手轮方向改变插补方向
//		{
//			//根据手轮模拟倍率来赋值方向
//			pSPreProcess->cInterpDir = pSChannel->SRun.dHandWheelSpeedRatio > 0 ? INP_GO_AHEAD : INP_GO_BACK;
//#if EN_DEBUG
//			printf("No ReadBufs -- SYS_RUN\n");
//#endif
//		}
//		if (*pSystemStatus == SYS_RUN_TO_PAUSE)//RUN_TO_PAUSE情况下没有可读缓存  说明暂停完成直接将状态置为PAUSE
//		{
//			//注释原因：旧版本有前瞻需要该判断，现版本无前瞻
//			//系统正在暂停 则暂停完成
//			//if (pSChannel->SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED)//主轴停下
//			//{
//			//	*pSystemStatus = SYS_PAUSE;				//系统状态为暂停
//			//}
//			//return 0;
//		}
//
//		if (*pSystemStatus == SYS_PAUSE ||
//			*pSystemStatus == SYS_IDLE ||
//			*pSystemStatus == SYS_PAUSE_TO_RUN)//运行过程中无数据可读 中断处理不过来 （或者程序没有M30 M02）报错
//		{
//#if EN_DEBUG
//			printf("error System status (%d)\n", *pSystemStatus);
//#endif
//		}
//
//		if (*pSystemStatus == SYS_RESET) //一般情况下进不来
//		{
//			initAllBuffer(pSChannel->ucChannelNum); //复位  把缓冲区初始化
//			*pSystemStatus = SYS_IDLE; //系统状态改为空闲
//		}
//
//		return -1;
//	}
//
//	//获取"速度规划读"指针
//	pSToolCompPointer = getSToolCompPointerByType(PointerType, pSChannel->ucChannelNum);
//
//	machineTimerStart(pSPreProcess->pSChannel->ucChannelNum);//加工计时启动
//	//根据刀补缓冲区数据 初始化 插补预处理结构体
//	if (initInterpStructByTool(pSToolCompPointer, pSPreProcess,pSPreProcess->cInterpDir) != 1)
//	{
//		//仅用于倒走
//		moveSToolCompBufPointer(INTERPOLATION_BACK,  BUF_AHEAD, pSChannel->ucChannelNum);
//		clearInterpolationDealedNum(pSChannel->ucChannelNum);//清除插补处理过的数据
//		return 0;
//	}
//
//	
//	bApplyPreIO = (isZero(pSPreProcess->SUserData.dPreDist) && pSPreProcess ->cInterpDir == INP_GO_AHEAD)? TRUE : FALSE;//根据是否已执行过判断是否执行前作用IO
//
//	//偏移计算
//	pSRun->ucToolNum = pSPreProcess->ucToolNum;						//刀具号
//	pSRun->ucToolCompNum = pSPreProcess->ucToolCompNum;			//磨损组号
//	pSRun->cWorkpieceCoordNum = pSPreProcess->cWorkpieceCoordNum;	//工件坐标号
//
//	//计算总偏移  赋值给SRun结构体  用于显示
//	TChangeTotalOffset(pSPreProcess->pSChannel->ucChannelNum, pSRun->ucToolCompNum, pSRun->cWorkpieceCoordNum, &pSRun->SCurrentOffset);
//
//	//指令类型赋值 (传递到SRun)
//	pSRun->unCommandType = pSPreProcess->unCommandType;//指令类型
//    strncpy(pSRun->cFilename, pSPreProcess->cFilename, FILENAME_LENGTH_LIMIT);//复制文件名
//#if STM32_MODE
//
//    strncpy(Npro_name.Name, pSPreProcess->cFilename, FILENAME_LENGTH_LIMIT);//修改当前显示程序的程序名
//
//#endif
//	pSRun->unGCodeLineNum = pSPreProcess->unGCodeLineNum;//G代码行号
//
//	pSPreProcess->ucInpStatus = INP_BLOCKED;				//阻塞状态（等待主轴达到指令转速才能运行）
//	pSChannel->SApplyTime.unSpindleSpeedUpAlarmRunTime = 0;	//主轴加速超时计时器
//
//
//	if (bApplyPreIO)
//	{
//		//根据M3 M4改变C轴的方向 并启动主轴
//		if (findIO(3, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 3)//有M03
//		{
//			//主轴方向 -> 正转
//			setSpindleDir(&pSChannel->SSpindle, ARC_CW);
//			//主轴启动
//			spindleBegin(&pSChannel->SSpindle);
//		}
//		else if (findIO(4, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 4)//有M04
//		{
//			//主轴方向 -> 反转
//			setSpindleDir(&pSChannel->SSpindle, ARC_ANTI_CW);
//			//主轴启动
//			spindleBegin(&pSChannel->SSpindle);
//		}
//		//根据M26 M27改变C轴的方向 并启动动力轴
//		else if (findIO(50, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 50)//有M50
//		{
//			//主轴方向 -> 正转
//			setSpindleDir(&pSChannel->SSecondSpindle, ARC_CW);
//			//主轴启动
//			spindleBegin(&pSChannel->SSecondSpindle);
//		}
//		else if (findIO(51, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 51)//有M51
//		{
//			//主轴方向 -> 反转
//			setSpindleDir(&pSChannel->SSecondSpindle, ARC_ANTI_CW);
//			//主轴启动
//			SSecondSpindleBegin(&pSChannel->SSecondSpindle);
//		}
//	}
//
//
//	//从预处理结构体中读取主轴转速数据  G50数据
//	//readCommandAndSpindleSpeed(pSPreProcess, &pSChannel->SSpindle);
//
//	readSecondSpindleSpeed(pSPreProcess, &pSChannel->SSecondSpindle);
//
//	//处理完成移动指针
//	moveSToolCompBufPointer(PointerType,  BUF_AHEAD, pSChannel->ucChannelNum);
//
//	return bApplyPreIO ? 2 : 3;//返回2： 执行前作用IO  3：不执行前作用IO
//
//}

/*************************************************
  函数名称：CommoninterpolationPre()
  函数功能  代替原来的interpolationPre()，判断"规划读"指针是否有内容可读，没有-返回报错
			有-根据读出的参数进行插补预处理
  输入参数：
  输出参数：
  返 回 值：
			4	快速预处理完成，单节运行，释放出合并的G代码行
			3:  预处理成功 不执行IO
			2:  预处理成功 执行IO
			-3	空指针

  备    注：
**************************************************/
int8 CommoninterpolationPre(struct SInterpolationPre* pSPreProcess, struct SToolCompensation* pSToolCompPointer, enum RingBufPointerType PointerType, struct _SQuickHandleFlag* pSQuickHandleFlag)
{
	struct SChannel* pSChannel;
	struct _SRun* pSRun;//运行结构体
	bool bApplyPreIO;//是否执行前作用IO
	if (pSPreProcess == NULL || pSToolCompPointer == NULL)
	{
		return -3;
	}

	pSChannel = pSPreProcess->pSChannel;//通道指针
	pSRun = &pSChannel->SRun;//运行结构体指针

	bApplyPreIO = (isZero(pSPreProcess->SUserData.dPreDist) && pSPreProcess->cInterpDir == INP_GO_AHEAD) ? TRUE : FALSE;//根据是否已执行过判断是否执行前作用IO

	if (bApplyPreIO)
	{
		//根据M3 M4改变C轴的方向 并启动主轴
		if (findIO(3, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 3)//有M03
		{
			//主轴方向 -> 正转
			setSpindleDir(&pSChannel->SSpindle, ARC_CW);
			//主轴启动
			spindleBegin(&pSChannel->SSpindle);
		}
		else if (findIO(4, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 4)//有M04
		{
			//主轴方向 -> 反转
			setSpindleDir(&pSChannel->SSpindle, ARC_ANTI_CW);
			//主轴启动
			spindleBegin(&pSChannel->SSpindle);
		}
		//根据M26 M27改变C轴的方向 并启动动力轴
		else if (findIO(50, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 50)//有M50
		{
			//主轴方向 -> 正转
			setSpindleDir(&pSChannel->SSecondSpindle, ARC_CW);
			//主轴启动
			SSecondSpindleBegin(&pSChannel->SSecondSpindle);
		}
		else if (findIO(51, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 51)//有M51
		{
			//主轴方向 -> 反转
			setSpindleDir(&pSChannel->SSecondSpindle, ARC_ANTI_CW);
			//主轴启动
			SSecondSpindleBegin(&pSChannel->SSecondSpindle);
		}
	}

	//从预处理结构体中读取主轴转速数据  G50数据
	//readCommandAndSpindleSpeed(pSPreProcess, &pSChannel->SSpindle);

	readSecondSpindleSpeed(pSPreProcess, &pSChannel->SSecondSpindle);

	if (pSChannel->bSingleLineMode == TRUE && pSToolCompPointer->cMergedCounter != 0)
	{
		pSToolCompPointer->cMergedCounter--;
		pSQuickHandleFlag->cNeedQuickHandleFlag = TRUE;

		pSQuickHandleFlag->cCycleCounter += 1;
		pSPreProcess->unGCodeLineNum -= pSToolCompPointer->cMergedCounter;
		pSRun->unGCodeLineNum = pSPreProcess->unGCodeLineNum;
		return 4;
	}

	//处理完成移动指针
	moveSToolCompBufPointer(PointerType, BUF_AHEAD, pSChannel->ucChannelNum);

	if (pSPreProcess->bLastLineMark	//陈敏聪	2023/12/14增加  zxr 当前装载的段带有特殊动作时（涉及程序结束或循环边界的bLastLineMark、M/IO、G28、通道指令），不能让以后反摇手轮跨过它
		|| hasIOCommand(pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) || (pSPreProcess->ucG28ZeroingValAxis != 0)
#if CHANNEL_NUM_MAX > 1
		|| pSPreProcess->sChannelCommand.ucType != UNASSIGNED_VALUE
#endif // CHANNEL_NUM_MAX > 1)			//反走的条件  无M代码  无通道指令) 无M99指令 （无G28 CYN 2024.12.9）
		)
		clearInterpolationDealedNum(pSPreProcess->pSChannel->ucChannelNum);	//清除插补处理过的数据  zxr 满足以上情况则重设回退边界，使倒走可读段数为0，其余情况仍可倒走

	pSQuickHandleFlag->cNeedQuickHandleFlag = FALSE;
	pSQuickHandleFlag->cCycleCounter = 0;

	pSPreProcess->bRunPostIO = (pSPreProcess->cInterpDir == INP_GO_AHEAD);// zxr 正走标记段末后作用待处理（IRQHandler最后的后作用IO是否提交执行的判断条件），倒走不执行后作用
	if (bApplyPreIO)
	{
		//前作用IO
		ApplyIO(pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE, IO_PRE_ACTION, &pSChannel->SIOCommand, &pSToolCompPointer->SIOContrlInfo);
		if (pSChannel->SRun.ucToolNum != pSChannel->STIOCommand.ucToolNum)
		{
			TIOToRunBuff(pSPreProcess->ucToolNum, pSPreProcess->ucToolCompNum, &(pSChannel->STIOCommand));
		}
		//pSPreProcess->bRunPostIO = TRUE;//执行后作用IO
		return 2;
	}
	return 3;
}

/*************************************************
  函数名称：QuicklyinterpolationPre()
  函数功能：中断多行快速预处理
  输入参数：
  输出参数：
  返 回 值：4	预处理完成，单节运行中释放出合并的G代码行
			5	预处理完成，前作用和后作用IO一起执行
			6	预处理完成，不需要执行IO
  备    注：
**************************************************/
int8 QuicklyinterpolationPre(struct SInterpolationPre* pSPreProcess, struct SToolCompensation* pSToolCompPointer, enum RingBufPointerType PointerType, struct _SQuickHandleFlag* pSQuickHandleFlag)
{
	struct SChannel* pSChannel;
	struct _SRun* pSRun;//运行结构体
	bool bApplyALLIO;//是否执行前作用IO
	if (pSPreProcess == NULL || pSToolCompPointer == NULL)
	{
		return -3;
	}

	pSChannel = pSPreProcess->pSChannel;//通道指针
	pSRun = &pSChannel->SRun;//运行结构体指针

	bApplyALLIO = (isZero(pSPreProcess->SUserData.dPreDist) && pSPreProcess->cInterpDir == INP_GO_AHEAD) ? TRUE : FALSE;//根据是否已执行过判断是否执行前作用IO

	if (pSChannel->bSingleLineMode == TRUE)
	{
		pSQuickHandleFlag->cNeedQuickHandleFlag = FALSE;
		pSQuickHandleFlag->cCycleCounter = 0;
		if (pSToolCompPointer->cMergedCounter != 0)
		{
			pSToolCompPointer->cMergedCounter--;
			pSPreProcess->unGCodeLineNum -= pSToolCompPointer->cMergedCounter;
			pSRun->unGCodeLineNum = pSPreProcess->unGCodeLineNum;
			return 4;
		}
		else
		{
			moveSToolCompBufPointer(PointerType, BUF_AHEAD, pSChannel->ucChannelNum);
		}
	}
	else
	{
		pSQuickHandleFlag->cNeedQuickHandleFlag = TRUE;
		pSQuickHandleFlag->cCycleCounter += 1;
		//处理完成移动指针
		moveSToolCompBufPointer(PointerType, BUF_AHEAD, pSChannel->ucChannelNum);
	}

	if (bApplyALLIO)
	{
		//前作用IO
		ApplyIO(pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE, IO_PRE_ACTION, &pSChannel->SIOCommand, NULL);
		ApplyIO(pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE, IO_POST_ACTION, &pSChannel->SIOCommand, NULL);
		if (pSChannel->SRun.ucToolNum != pSChannel->STIOCommand.ucToolNum)
		{
			TIOToRunBuff(pSPreProcess->ucToolNum, pSPreProcess->ucToolCompNum, &(pSChannel->STIOCommand));
		}
		pSPreProcess->bRunPostIO = TRUE;//执行后作用IO
		return 5;
	}
	return 6;
}


///*************************************************
//  函数名称：recoveryToRun()
//  函数功能  恢复加工函数，利用当前点的坐标和暂停完成点的坐标来初始化单轴运动，以单轴运动模式移动到暂停完成点，然后把系统状态转为RUN
//  输入参数：
//  输出参数：
//  返 回 值：
//  备    注：暂停完成点的数据来源 为预处理结构体中的暂停点
//****************************************/
//void recoveryToRun(struct SG0* pSG0Pointer)
//{
//	if (pSG0Pointer->ucCurrentSigned == pSG0Pointer->ucFinishedSigned)
//	{   //三个轴都不在运行状态，初始化
//
//		//SPointDouble SEndPoint;//移动的目标坐标（终点）
//		//struct SToolCompensation* pSToolCompPointer;//刀补缓冲区指针
//
//		//if (getSToolCompPointerBufRes(INTERPOLATION_READ, pSG0Pointer->pSChannel->ucChannelNum) > 0 || getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, pSG0Pointer->pSChannel->ucChannelNum) > 0)//判断是否可读下一段的信息
//		//{
//		//	//获取插补读指针
//		//	pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSG0Pointer->pSChannel->ucChannelNum);
//
//		//	//刀补缓冲区当前线段的起点作为终点
//		//	SEndPoint = pSToolCompPointer->SStartPoint;
//		//}
//		//else  //不可读 说明三个指针重合  利用前一个缓冲区的终点作为恢复运动的终点信息
//		//{
//		//	//获取插补指针前一个缓冲区的指针
//		//	pSToolCompPointer = getSToolCompPointerByIndex((int16)(getSToolCompPointerIndex(INTERPOLATION_READ, pSG0Pointer->pSChannel->ucChannelNum) - 1), pSG0Pointer->pSChannel->ucChannelNum);
//
//		//	SEndPoint = pSToolCompPointer->SEndPoint;//上一线段的终点作为终点坐标
//		//}
//		//以终点和起点坐标初始化单轴
//		//singleAxisPreLoop(pSG0Pointer, SEndPoint, NULL);
//		SPointDouble SStartPoint;
//		getRealTimePoint((double*)&SStartPoint, AXIS_NUM_MAX, pSG0Pointer->pSChannel->pMapAxisArray,NULL);
//		if (pSG0Pointer->pSChannel->SPreProcess.ucLastValStatus != NULL)
//		{
//			singleAxisPreLoop(pSG0Pointer, SStartPoint, pSG0Pointer->pSChannel->SPreProcess.SPausePoint, &pSG0Pointer->pSChannel->SPreProcess.SUserData, &pSG0Pointer->pSChannel->SPreProcess.SUserData.ucValStatus, TRUE);
//		}
//		else
//		{
//			singleAxisPreLoop(pSG0Pointer, SStartPoint, pSG0Pointer->pSChannel->SPreProcess.SPausePoint, NULL, NULL, TRUE);
//		}
//	}
//
//	//单轴插补运动循环
//	singleAxisMotionLoop(pSG0Pointer, AXIS_NUM_MAX, TRUE, TRUE);
//
//	//根据当前标记来判断G0（恢复加工）是否执行完成
//	if (pSG0Pointer->ucCurrentSigned == pSG0Pointer->ucFinishedSigned)
//	{
//		pSG0Pointer->pSChannel->SPreProcess.ucInpStatus = INP_FINISHED;
//		for (int i = 0; i < AXIS_NUM_MAX; i++)
//		{
//			if(getbit(pSG0Pointer->ucFinishedSigned, i))
//			pSG0Pointer->pSChannel->SSingleAxisArray[i].ucInpStatus = INP_FINISHED;		//陈敏聪	2023/11/8	执行完成将所有轴的插补
//		}
//		pSG0Pointer->pSChannel->ucChannelStatus = SYS_RUN;  //改变系统状态
//	}
//}
/*************************************************
  函数名称：recoveryToRun(struct SG1* pSG1Pointer)
  函数功能  恢复加工函数，利用当前点的坐标和暂停完成点的坐标来初始化SG1结构体，以直线运动模式移动到暂停完成点，然后把系统状态转为RUN
  输入参数：
  输出参数：
  返 回 值：
  备    注：暂停完成点的数据来源 为预处理结构体中的暂停点
****************************************/
void recoveryToRun(struct SG1* pSG1Pointer)
{
	struct SChannel* pSChannel;
	const uint8* ucMapArray;
	uint8 ucChannelNum;
	SPointDouble SStartPoint;
	int i = 0;

	ucMapArray = pSG1Pointer->pSChannel->pMapAxisArray;
	pSChannel = pSG1Pointer->pSChannel;
	ucChannelNum = pSG1Pointer->pSChannel->ucChannelNum;
	memset(SStartPoint.dArray, 0, sizeof(SStartPoint));//初始化恢复加工起点坐标

	if (pSChannel->SG00.bIsUserData == FALSE)//暂停后轴未发生移动，恢复时将程序直接启动
	{
		pSChannel->SRun.unCommandType = pSG1Pointer->pSChannel->SPreProcess.unCommandType;//继续执行原来的运动指令
		if (pSChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_BUILDING)
		{
			pSChannel->cHandWheelSimulaStatus = HANWHEELSIMULA_ON;
		}
		pSChannel->ucChannelStatus = SYS_RUN;
		return;
	}
	else//暂停后轴发生移动（手动/手轮），对通道的所有轴进行占用，执行直线运动的恢复加工
	{
		if (pSChannel->ucChannelStatus != SYS_RUN && pSG1Pointer->ucInpStatus == INP_FINISHED)//恢复加工
		{
			//1.轴占用：对当前通道内的轴进行占用
			for (i = 0; i < AXIS_NUM_MAX; i++) //根据通道映射获取当前通道下的所有轴
			{
				if (g_SSystemPara.ucAxisMapArray[ucChannelNum][i] == AXIS_NULL || ((i == AXIS_C) && (pSChannel->SSpindle.pSSpindleAxis->ucRunMode != SPINDLE_SINGLEAXIS_MODE)))continue; //物理轴映射为空或者C轴作为主轴时跳过有效轴的设置 2024/1/11 陈敏聪 洪冠鹏
				else
				{
					setbit(pSChannel->SPreProcess.SUserData.ucValStatus, i);
				}
			}
			releaseLastCommandAxis(&pSChannel->SPreProcess);
#if(TURRET_AXIS_TEST == 1)
			//判断轴占用(根据结构体轴+映射)   目标结构体轴减去刀塔轴
			if (axisManagerOccupyByChannel(pSChannel->SPreProcess.SUserData.ucValStatus & (~pSChannel->SPreProcess.ucOccupideTurretAxis), pSG1Pointer->pSChannel->ucChannelNum, &pSChannel->SPreProcess.ucOccupideValStatus) == 1)
			{
				//占用成功
				//G1有效轴（不能除去刀塔轴）
				pSG1Pointer->ucValStatus = pSChannel->SPreProcess.SUserData.ucValStatus;
			}
			else
			{	//占用失败
				return;
			}
#else
			//判断轴占用(根据有效轴+映射)   有效轴减去刀补轴
			if (axisManagerOccupyByChannel(pSChannel->SPreProcess.SUserData.ucValStatus & (~pSChannel->SPreProcess.ucToolAxis), pSG1Pointer->pSChannel->ucChannelNum, NULL) == 1)
			{	//占用成功
				//上一条指令的占用（除去刀补）
				pSChannel->SPreProcess.ucLastValStatus = pSChannel->SPreProcess.SUserData.ucValStatus & (~pSChannel->SPreProcess.ucToolAxis);//将当前的轴占用变量保存，以区分暂停后是否发生过轴移动
				//G1有效轴（不能除去刀补）
				pSG1Pointer->ucValStatus = pSChannel->SPreProcess.SUserData.ucValStatus;
			}
			else //轴占用失败后退出函数
			{
				return;
			}
#endif
			//2.初始化：根据暂停前正在处理的指令所对应的速度参数，对恢复加工的SG1运动结构体进行初始化。
			getRealTimePoint((double*)&SStartPoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, NULL);//获取当前通道所有轴的实时点作为恢复加工的起点，暂停时保存的暂停点作为恢复加工的起点
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
			if (pSChannel->bUseVirtualY == TRUE)//开启合成虚拟Y轴功能：起点为倾斜坐标系（X1-Y1-Z1）物理轴的实时点，终点为直角坐标系(X-Y-Z)缓冲区的值，因此要将起点换算至直角坐标系。
			{
				SStartPoint.dArray[AXIS_X] -= 1.0 * SStartPoint.dArray[AXIS_Y] * pSChannel->dSINAngleYY1;//将倾斜坐标系的点转化为直角坐标系的点
				SStartPoint.dArray[AXIS_Y] = SStartPoint.dArray[AXIS_Y] * pSChannel->dCOSAngleYY1;
			}
#endif

			if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)//暂停前正在执行单轴运动：单轴结构体的速度参数从系统各单轴参数获得，而恢复加工只需要一个速度参数。因此从系统参数获得速度参数到预处理结构体。
			{
				loadSG1DataAfterPause(pSG1Pointer, SStartPoint, pSChannel->SPreProcess.SPausePoint, TRUE, &pSChannel->SPreProcess.SUserData);
			}
			else//暂停前正在执行合成运动：从缓冲区的插补预处理结构体获取速度参数。
			{
				loadSG1DataAfterPause(pSG1Pointer, SStartPoint, pSChannel->SPreProcess.SPausePoint, FALSE, &pSChannel->SPreProcess.SUserData);
			}
		}
		setBits(pSChannel->SRun.unCommandType, CMD_MOTION_LINE, CMD_MOTION_ADDR);//执行直线运动，因为执行手动/手轮的底层是单轴运动，因此要在此处修改运动结构体的运动指令。
		if (motionLine(pSG1Pointer, TRUE, TRUE) == 0)
		{
			pSChannel->SRun.unCommandType = pSG1Pointer->pSChannel->SPreProcess.unCommandType;//继续执行原来的运动指令
			for (i = 0; i < AXIS_NUM_MAX; i++) //恢复加工结束切换回缓冲区指令时清除单轴速度规划参数 2024/1/11 陈敏聪
			{
				if ((getbit(pSChannel->SPreProcess.SUserData.ucValStatus, i)) == 0)continue;
				clearDistReAvgPara(&pSChannel->SSingleAxisArray[i].SPlanData);//清除平摊参数
				clearSPlanData(&pSChannel->SSingleAxisArray[i].SPlanData); //清除规划参数
			}
			pSChannel->SG00.bIsUserData = FALSE;
			pSChannel->ucChannelStatus = SYS_RUN;  //恢复加工执行完后改变系统状态
			return;
		}
	}
}

/*************************************************
  函数名称： singleAxisSlowDown(uint8 ucAxis)
  函数功能  单轴减速初始化 （读SRun中上个周期发送的脉冲作为起始速度） 做减速的速度规划，不考虑轨迹
  输入参数：pSSingleAxisPointer 单轴结构体指针
  输出参数：
  返 回 值：0：不用减速直接停
			1：减速初始化完成
			-1：空指针
			-2  没有映射
  备    注：
**************************************************/
int8 singleAxisSlowDown(struct SSingleAxis* pSSingleAxisPointer, struct _SRun* pSRun, uint8* ucMapArray, G0_SIGN_TYPE* currentSign)
{
	double dRealTimePoint;  //实时点
	uint8 ucPhysicalAxis;	//物理轴号
	uint8 ucAxis;			//结构体轴号

	//空指针
	if (pSSingleAxisPointer == NULL || currentSign == NULL || pSRun == NULL || ucMapArray == NULL)
	{
		return -1;
	}

	ucAxis = pSSingleAxisPointer->ucAxis;//轴号
	ucPhysicalAxis = ucMapArray[ucAxis];
	//查询映射
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
		return -2;  //没有映射  返回
	}

	//插补周期
	pSSingleAxisPointer->SPlanData.dTc = g_SSystemPara.dInterpolationTime;
	//获取当前速度 ∣pulse∣×(mm/pulse)​/ms = mm/ms
	pSSingleAxisPointer->SPlanData.dVs = fabs(pSRun->sAxisDeltaPulse[ucAxis] * g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm) / pSSingleAxisPointer->SPlanData.dTc;
	//最大速度等于当前速度
	pSSingleAxisPointer->SPlanData.dVmax = pSSingleAxisPointer->SPlanData.dVs;
	//末速度为单轴末速度
	pSSingleAxisPointer->SPlanData.dVe = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe;

	if (pSSingleAxisPointer->SPlanData.dVs <= pSSingleAxisPointer->SPlanData.dVe)//不用减速 直接停
	{
		pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
		return 0;
	}

	//速度曲线类型
	pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAccVelPlanType;
	pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType;

	//加减速度
	pSSingleAxisPointer->SPlanData.dAcc = getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
	pSSingleAxisPointer->SPlanData.dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);
	if(pSSingleAxisPointer->SPlanData.dAcc <= 0|| pSSingleAxisPointer->SPlanData.dDec <= 0)
	{
		parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
		return -1;
	}

	//平摊类型
	pSSingleAxisPointer->SPlanData.ucDistReAverageType = g_SSystemPara.ucDistReAverageType;
	//减速距离(根据曲线类型计算)
	pSSingleAxisPointer->SPlanData.dDistance = getFactor(pSSingleAxisPointer->SPlanData.ucDecVelPlanType) * (pow(pSSingleAxisPointer->SPlanData.dVs, 2) - pow(pSSingleAxisPointer->SPlanData.dVe, 2)) / (pSSingleAxisPointer->SPlanData.dDec);

	//获取当前点  作为起点
	getSingleAxisRealTimePoint(&pSSingleAxisPointer->dStartPoint, ucAxis, ucMapArray);
	//起点向量 根据距离的符号设置
	pSSingleAxisPointer->dStartVector = sgn(pSRun->sAxisDeltaPulse[ucAxis]);
	// 终点（根据起点和距离计算）
	pSSingleAxisPointer->dEndPoint = pSSingleAxisPointer->dStartPoint + pSSingleAxisPointer->dStartVector * pSSingleAxisPointer->SPlanData.dDistance;

	if (g_RunMode != RE0_MODE && g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].soft_pos == 1 && pSSingleAxisPointer->dEndPoint > g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].pos_soft)//终点超出正限位点时，限制终点位置 2024/12/17
	{
		pSSingleAxisPointer->dEndPoint = g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].pos_soft;
		pSSingleAxisPointer->SPlanData.dDistance = fabs(pSSingleAxisPointer->dEndPoint - pSSingleAxisPointer->dStartPoint);
	}
	if (g_RunMode != RE0_MODE && g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].soft_neg == 1 && pSSingleAxisPointer->dEndPoint < g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].neg_soft)//终点超出负限位点时，限制终点位置
	{
		pSSingleAxisPointer->dEndPoint = g_SSystemPara.MOTOR_WARN[ucMapArray[ucAxis]].neg_soft;
		pSSingleAxisPointer->SPlanData.dDistance = fabs(pSSingleAxisPointer->dEndPoint - pSSingleAxisPointer->dStartPoint);
	}
	//软限位开关变量清0
	pSRun->cSoftLimitedOn = 0;
	//已走距离 为0
	pSSingleAxisPointer->SPlanData.dPreDist = 0;
	//剩余距离 =计算得到的减速距离
	pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SPlanData.dTargetDist = pSSingleAxisPointer->SPlanData.dDistance;
	//插补时间置0
	pSSingleAxisPointer->SPlanData.unCurrentT = 0;

	//速度规划+平摊
	preparePlanningAndDistReAvg(&pSSingleAxisPointer->SPlanData, TRUE);
	//单轴状态改为运行
	pSSingleAxisPointer->ucInpStatus = INP_RUNNING;
	//G0当前标记置0
	clrbit(*currentSign, ucAxis);//zxr 清除 SG00.ucCurrentSigned 中本轴对应的位，让外层ucCurrentSigned != ucFinishedSigned
	return 1;
}

//朱稳中 2023.6.21
/*************************************************
  函数名称： singleAxisToLimit(uint8 ucAxis)
  函数功能  单轴减速到软限位点 ，根据单轴状态判断为哪个软限位（正/负）
  输入参数：pSSingleAxisPointer 单轴结构体指针
  输出参数：
  返 回 值：0：不用减速直接停
			1：减速初始化完成
			-1：空指针
			-2  没有映射
  备    注：
**************************************************/
int8 singleAxisToLimit(struct SSingleAxis* pSSingleAxisPointer, struct _SRun* pSRun, uint8* ucMapArray, G0_SIGN_TYPE* currentSign)
{
	double dRealTimePoint;  //实时点
	uint8 ucPhysicalAxis;	//物理轴号
	uint8 ucAxis;			//结构体轴号

	//空指针
	if (pSSingleAxisPointer == NULL || currentSign == NULL || pSRun == NULL || ucMapArray == NULL)
	{
		return -1;
	}

	ucAxis = pSSingleAxisPointer->ucAxis;//轴号
	ucPhysicalAxis = ucMapArray[ucAxis];
	//查询映射
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
		return -2;  //没有映射  返回
	}

	//插补周期
	pSSingleAxisPointer->SPlanData.dTc = g_SSystemPara.dInterpolationTime;
	//获取当前速度
	pSSingleAxisPointer->SPlanData.dVs = fabs(pSRun->sAxisDeltaPulse[ucAxis] * g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm) / pSSingleAxisPointer->SPlanData.dTc;
	//获取速度规划后的初速度
	//pSSingleAxisPointer->SPlanData.dVs;
	//最大速度等于当前速度
	pSSingleAxisPointer->SPlanData.dVmax = pSSingleAxisPointer->SPlanData.dVs;
	//末速度为单轴末速度
	pSSingleAxisPointer->SPlanData.dVe = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe;

	//if (pSSingleAxisPointer->SPlanData.dVs <= pSSingleAxisPointer->SPlanData.dVe)//不用减速 直接停
	//{
	//	pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
	//	return 0;
	//}

	//速度曲线类型
	pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAccVelPlanType;
	pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType;

	//加减速度
	pSSingleAxisPointer->SPlanData.dAcc = getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
	pSSingleAxisPointer->SPlanData.dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);

	//平摊类型
	pSSingleAxisPointer->SPlanData.ucDistReAverageType = g_SSystemPara.ucDistReAverageType;


	//获取当前点  作为起点
	getSingleAxisRealTimePoint(&pSSingleAxisPointer->dStartPoint, ucAxis, ucMapArray);

	// 终点（根据起点和距离计算）
	if (pSSingleAxisPointer->ucRunMode == SINGLEAXIS_LIMIT_POS) // 正限位
	{
		pSSingleAxisPointer->dEndPoint = (g_SSystemPara.MOTOR_WARN[ucAxis].pos_soft - pSSingleAxisPointer->dStartPoint > EPSINON) ? g_SSystemPara.MOTOR_WARN[ucAxis].pos_soft : pSSingleAxisPointer->dStartPoint;
	}
	else // 负限位
	{
		pSSingleAxisPointer->dEndPoint = (pSSingleAxisPointer->dStartPoint - g_SSystemPara.MOTOR_WARN[ucAxis].neg_soft > EPSINON) ? g_SSystemPara.MOTOR_WARN[ucAxis].neg_soft : pSSingleAxisPointer->dStartPoint;
	}
	//减速距离(根据曲线类型计算)
	pSSingleAxisPointer->SPlanData.dDistance = fabs(pSSingleAxisPointer->dEndPoint - pSSingleAxisPointer->dStartPoint);

	//起点向量 根据距离的符号设置
	pSSingleAxisPointer->dStartVector = sgn(pSSingleAxisPointer->dEndPoint - pSSingleAxisPointer->dStartPoint);

	//减速度 根据初末速度和距离进行计算
	if(!isZero(pSSingleAxisPointer->SPlanData.dDistance))
	{
		pSSingleAxisPointer->SPlanData.dDec =getFactor(pSSingleAxisPointer->SPlanData.ucDecVelPlanType) * (pow(pSSingleAxisPointer->SPlanData.dVs, 2) - pow(pSSingleAxisPointer->SPlanData.dVe, 2)) / pSSingleAxisPointer->SPlanData.dDistance;
	}



	//已走距离 为0
	pSSingleAxisPointer->SPlanData.dPreDist = 0;
	//剩余距离 =计算得到的减速距离
	pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SPlanData.dTargetDist = pSSingleAxisPointer->SPlanData.dDistance;
	//插补时间置0
	pSSingleAxisPointer->SPlanData.unCurrentT = 0;

	//速度规划+平摊
	preparePlanningAndDistReAvg(&pSSingleAxisPointer->SPlanData, TRUE);
	//单轴状态改为运行
	pSSingleAxisPointer->ucInpStatus = INP_RUNNING;
	//G0当前标记置0
	clrbit(*currentSign, ucAxis);

	//软限位开关变量置0
	pSRun->cSoftLimitedOn = 0;
	return 1;
}


/*************************************************
  函数名称：singleAxisSlowDownLoop(struct SG0 *pSG0Pointer,int nAxisNum)
  函数功能  循环调用单轴减速函数
  输入参数：ucAxis，轴号
  输出参数：
  返 回 值：-1 空指针
			-2 轴数错误
			1  处理完成
  备    注：
**************************************************/
int8 singleAxisSlowDownLoop(struct SG0* pSG0Pointer, int nAxisNum)
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

	//循环单轴减速
	for (i = 0; i < nAxisNum; i++)
	{
		if (g_RunMode == HANDWHEEL_MODE
			&& (pSG0Pointer->pSSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG || pSG0Pointer->pSSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS))
		{
			singleAxisToLimit(&pSG0Pointer->pSSingleAxisArray[i], &pSG0Pointer->pSChannel->SRun, pSG0Pointer->pSChannel->pMapAxisArray, &pSG0Pointer->ucCurrentSigned);
			continue;
		}

		//if (pSG0Pointer->pSSingleAxisArray[i].ucRunMode != SINGLEAXIS_MODE)
		if ((pSG0Pointer->pSChannel->SSpindle.pSSpindleAxis->ucAxis == i || pSG0Pointer->pSChannel->SSecondSpindle.pSSpindleAxis->ucAxis == i) && pSG0Pointer->pSSingleAxisArray[i].ucRunMode != SINGLEAXIS_MODE)//2024/12/17 主轴及第二主轴改用轴号判断
		{// 主轴不进行初始化
			continue;
		}

		//单轴减速初始化
		singleAxisSlowDown(&pSG0Pointer->pSSingleAxisArray[i], &pSG0Pointer->pSChannel->SRun, pSG0Pointer->pSChannel->pMapAxisArray, &pSG0Pointer->ucCurrentSigned);
	}
	return 1;
}


/*************************************************
  函数名称：channelStopToIDLE(struct SChannel* pSChannel)
  函数功能  停止运行切换到IDLE
  输入参数：pSChannel 通道指针
  输出参数：
  返 回 值：1  - 通道停止成功
			-1 - 通道无效
			0  - 通道需等待主轴停止后程序才结束
  备    注：
**************************************************/
int8 channelStopToIDLE(struct SChannel* pSChannel)
{
	uint32 unOutputIOIndex = 0; //IO输出端口的索引
	if (pSChannel == NULL)
	{
		return -1;
	}
	pSChannel->ucChannelStatus = SYS_IDLE;//系统状态 空闲
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* 程序结束或复位终止当前授权，下一次运行不得继承旧待走距离。 */
    resetHandwheelSimGroup(&pSChannel->SHandwheelSim);
#endif

	pSChannel->SRun.unGCodeLineNum = (getBits(pSChannel->SPreProcess.unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_STOP_M02) ? pSChannel->SRun.unGCodeLineNum : 0;
//	pSChannel->SRun.unGCodeLineNum = 0;

	//if (bStopSpindle && pSChannel->SSpindle.pSSpindleAxis->ucRunMode != SINGLEAXIS_MODE&&g_RunMode == AUTOMATIC_MODE)//要使主轴停下 ，并且主轴在转  2025.4.26熊骏删除
	//{
	//	spindleStopAPI(&pSChannel->SSpindle, &g_SSystemPara.fSpindleStopDegree, FALSE);//主轴准停
	//}
	//if (bStopSpindle && pSChannel->SSecondSpindle.pSSpindleAxis->ucRunMode != SINGLEAXIS_MODE&&g_RunMode == AUTOMATIC_MODE)//要使动力刀停下 ，并且动力刀在转
	//{
	//	spindleStopAPI(&pSChannel->SSecondSpindle, &g_SSystemPara.fSpindleStopDegree, FALSE);//第二转轴/动力刀准停
	//}
	//清空XYZABC轴的脉冲增量缓存区 0011 1111
	initSRunPulseBuf(&pSChannel->SRun, 0x3F);
	//初始化缓冲区
	initAllBuffer(pSChannel->ucChannelNum);

#if(TURRET_AXIS_TEST == 1)
	releaseAllOccupiedAxis(&pSChannel->SPreProcess);//释放所有占用的轴（刀塔轴+非刀塔轴）;
	releaseAllOccupiedTurret(&pSChannel->SPreProcess);//释放所有占用的刀塔

#else
	//释放上条指令的轴占用
	releaseLastCommandAxis(&pSChannel->SPreProcess);

	//释放刀补的轴
	axisManagerReleaseByStruct(pSChannel->SPreProcess.ucToolAxis, pSChannel->pMapAxisArray, AXIS_NUM_MAX, NULL);
	pSChannel->SPreProcess.ucToolAxis = 0;//清空刀补的有效轴
	pSChannel->SPreProcess.ucToolCompensationStatus = TOOL_COMPENSATION_IDLE;//刀补置为空闲
#endif
	memset(pSChannel->SPreProcess.cOriginFilename, '\0', sizeof(pSChannel->SPreProcess.cOriginFilename));

	//interpolationInit(&pSChannel->SPreProcess);//初始化插补  2023.3.28 朱稳中

	//删除插补结构体的M30 M02
	setBits(pSChannel->SPreProcess.unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
	pSChannel->SPreProcess.bLastLineMark = FALSE; // zxr: 通道结束或取消本轮程序时清除末行标志，避免重启时重复计件。
	//切换成普通插补
	setBits(pSChannel->SPreProcess.unCommandType, CMD_INTERPOLATION_MODE_ADDR, CMD_INTERPOLATION_COMMON);
	//单轴结构体状态
	setFinishedStatus(&pSChannel->SG00, AXIS_NUM_MAX);

	// G0当前标志置完成
	pSChannel->SG00.ucCurrentSigned = pSChannel->SG00.ucFinishedSigned;
	pSChannel->SG00.ucRunStep = 5;					//初始化G0123状态 朱稳中 2023.3.24
	pSChannel->SG01.ucInpStatus = INP_FINISHED;
	pSChannel->SG02.ucInpStatus = INP_FINISHED;
	memset((void*)&pSChannel->SPreProcess.SUserData, 0, sizeof(struct SInput));
	pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead = AXIS_NULL;//需要前瞻的单轴轴号置空

	pSChannel->SPreProcess.ucRunStep = 0;//运行步骤为0
	pSChannel->SG01.dCurrentSpeed = 0;	//G1当前速度为0
	pSChannel->SG02.dCurrentSpeed = 0;	//G2当前速度为0
	pSChannel->SRun.dCurrentSpeed = 0; //合成当前速度置0

	if (g_SSystemPara.userset.M20_mode == 0 && pSChannel->ucChannelNum == 0) //关闭M20输出(仅在通道0复位后有效)  2023/10/7 洪冠鹏 企业要求增加仅在通道0复位后才可关闭M20输出口
	{
		for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE; ucounter++)
		{
			if (pSChannel->SIOCommand.SVaribleIOInfoArray[ucounter].unOutputIOIndexOfM20 != 0)
			{
				for (int i = 0; i < 32; i++)
				{
					if ((pSChannel->SIOCommand.SVaribleIOInfoArray[ucounter].unOutputIOIndexOfM20 & 0x01) == 0x01)
					{
						unOutputIOIndex = i;
						out[unOutputIOIndex] = LOWLEVEL;
					}
					pSChannel->SIOCommand.SVaribleIOInfoArray[ucounter].unOutputIOIndexOfM20 = pSChannel->SIOCommand.SVaribleIOInfoArray[ucounter].unOutputIOIndexOfM20 >> 1;
				}
			}
		}
	}
	pSChannel->bChuckALoose = g_SSystemPara.userset.Chuck_status[0];
	pSChannel->bChuckBLoose = g_SSystemPara.userset.Chuck_status[1]; //获取系统参数设定的夹紧状态 洪冠鹏 2023/10/14
	pSChannel->SPreProcess.ulDelayTime = 0; //清空G04的延时 洪冠鹏 2023/10/12
	pSChannel->bProgramStop = FALSE; //洪冠鹏 2023/10/11
	//memset(&pSChannel->SIOCommand, 0, sizeof(struct _SIOCommand));		//陈敏聪	2023/7/11
	pSChannel->SIOCommand.pSChannel = pSChannel;
	pSChannel->STIOCommand.pSChannel = pSChannel;
	pSChannel->STIOCommand.ucToolChangeStatus = TOOLCHANGE_IDLE;
	pSChannel->STIOCommand.ucTIORun = 0;
	pSChannel->STIOCommand.ucToolCompNumRun = 0;
	pSChannel->STIOCommand.ucReadIO_Counter = 0;
	//if((!(pSChannel->bIsStopSpindleAndCoolantwhileM30 || pSChannel->bIsStopSpindleAndCoolantwhileReset))&&g_RunMode == AUTOMATIC_MODE)	//陈敏聪	2023/12/23 应企业要求，没特殊配置关冷却液 //洪冠鹏 2024/6/18 复位关主轴关冷却液功能仅按下复位按键后有效 //2024/7/25 M30关主轴关冷却液功能仅在通道0有效
		//IOStructToBuff(9, &pSChannel->SIOCommand, NULL);2025.4.26熊骏删除

	G33ResetInit(pSChannel);//G33复位初始化
	changeNoRepeatG32(pSChannel);//将下一段G32修改为非连续G32
	//IOStructToBuff(15, &pSChannel->SIOCommand);//M15 主轴不使能  没有阻塞		陈敏聪 2023/4/19 苏总要求 复位主轴不失能，保持原本状态

	pSChannel->SMachineTimer.isTimerBegined = FALSE;//加工计时停止
	pSChannel->cRunChannelGcodeAgain = FALSE;	//陈敏聪	2023/12/23	复位或工件数达到都需要清除此标志位
	pSChannel->bSingleLineModeUnFinished = FALSE; //洪冠鹏  2024/5/21  重置单节执行下的当前行未完成标志位
	pSChannel->bIsStopSpindleAndCoolantwhileReset = TRUE; //洪冠鹏 2024/6/18 初始化复位关闭主轴关闭冷却液标志位，等待复位按键按下后重新获取系统参数
	pSChannel->dLessChipBreakLPreDist = 0;//不足断屑间隔长度但已走的距离清0
	pSChannel->bChipBreakStart = FALSE;	  //清除断屑开始标志位
	pSChannel->SRun.cSoftLimitedOn = 0; //软限位开关标志位置0 2025/01/02
	for (int i = 0; i < AXIS_NUM_MAX; i++)//若为限位模式 切换为单轴模式
	{
		if (pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_POS || pSChannel->SSingleAxisArray[i].ucRunMode == SINGLEAXIS_LIMIT_NEG)
		{
			pSChannel->SSingleAxisArray[i].ucRunMode = SINGLEAXIS_MODE;
		}
	}
	return 1;
}

/* 复位第 1 步：本拍减速脉冲以及上一拍提交到仿真输出槽的值均为零才可收尾。
 * calculatePWMValue 每拍用物理轴到 PWM 槽的映射更新全局数组；VS 没有硬件待发帧。 */
static bool isResetPulseOutputIdle(const struct SChannel* pSChannel)
{
    uint8 ucAxis;
    if (pSChannel->pMapAxisArray == NULL || pSChannel->pPWMMapAxisArray == NULL)
        return FALSE;

    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ucAxis++)
    {
        uint8 ucSource = pSChannel->pMapAxisArray[ucAxis];
        uint8 ucOutput;
        if (ucSource >= AXIS_PHYSICS_NUM_MAX || isResetSpindleKept(pSChannel, ucAxis))
            continue;
        if (pSChannel->SRun.sAxisDeltaPulse[ucAxis] != 0)
            return FALSE;
        ucOutput = pSChannel->pPWMMapAxisArray[ucSource];
        if (ucOutput < AXIS_PHYSICS_NUM_MAX
            && (g_sAxisDeltaPulse[ucOutput] != 0 || g_usCompareValue[ucOutput] != 0))
            return FALSE;
    }
    return TRUE;
}

/*************************************************
  函数名称：resetProgram()
  函数功能  复位函数，开通时 各轴减速停，不考虑轨迹 然后将系统状态置为IDLE
  输入参数：ucAxis，轴号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void resetProgram(struct SChannel* pSChannel)
{
	struct SG0* pSG0;
	SPointDouble SCurPoint; // 当前点
	uint8 ucPhysicalAxis;//物理轴
	uint8* pMapAxisArray;//映射
	bool bAlarmStop = FALSE;
	if (pSChannel == NULL)//复位
	{
		return;
	}
#if STM32_MODE
		if(Check_bootwarning() == 0)//有报警则退出函数，监控企业管理的主轴报警、伺服报警、超时报警、限位报警
    	{
       		 bAlarmStop = TRUE;
    	}
#endif
	if ((bAlarmStop || isErrorHasBeenWritten() == TRUE) && pSChannel->ucChannelStatus != SYS_IDLE)//有报警则退出函数，监控学校管理的报警
	{
		return 1; //有错误优先完成报警轴停运动 //增加判断2026.5.7
	}
	if (!(pSChannel->ucChannelStatus == SYS_RESET))
	{
		return;
	}
	pSG0 = &pSChannel->SG00;

	switch (pSChannel->SPreProcess.ucRunStep)
	{
	case 0://初始化
		//单轴减速停（速度规划）
		singleAxisSlowDownLoop(pSG0, AXIS_NUM_MAX);

		//复位后G32的处理 6.24
		initResetForG32(pSChannel);

		if (!pSChannel->bIsStopSpindleAndCoolantwhileReset)	//陈敏聪 2023/12/23 应企业要求 判断是否复位执行关主轴同时关冷却
		{
			//主轴停
			spindleCommonStop(&pSChannel->SSpindle);

			//动力刀停
			spindleCommonStop(&pSChannel->SSecondSpindle);

			//刀具轴停
			g_SToolAxisInfo.pSToolAxis->ucRunMode = TOOLAXIS_SINGLE_MODE;
			initToolAxisPara();
		}

		//下一步
		pSChannel->SPreProcess.ucRunStep += 1;
	case 1:
		//清空XYZABC轴的脉冲增量缓存区 0011 1111
		initSRunPulseBuf(&pSChannel->SRun, 0x3F);

		//单轴运动循环
		singleAxisMotionLoop(pSG0, AXIS_NUM_MAX, FALSE, FALSE);

		//结束条件，C轴为FINISHED XYZ轴的输出脉冲为0
		if (pSG0->ucCurrentSigned == pSG0->ucFinishedSigned)
		{
            /* 苏向标 2026-10-14：等待取消使能和资源收尾，不以插补结束代替停止完成。 */
            if (((isSpindleStopFinished(&pSChannel->SSpindle)
                && isSpindleStopFinished(&pSChannel->SSecondSpindle))
                || pSChannel->bIsStopSpindleAndCoolantwhileReset)
                && isResetPulseOutputIdle(pSChannel))
            {
                pSChannel->SPreProcess.ucRunStep += 1;
            }
	    }
		if (g_pChannelWaitSignalMannager != NULL)
		{
			memset(g_pChannelWaitSignalMannager, 0, sizeof(struct SChannelWaitSignalMannager));
		}
		break;
	case 2:
#if ENCODER_MODE
		clearHandWheelPara(pSChannel);		//陈敏聪 2023/6/24
#endif
		pSChannel->bProgramNoFinished = FALSE;		//G代码用，决定G代码初始化是否使用固定参数	陈敏聪	2023/7/29
		clearApplyRunTime(pSChannel);	//2023/9/6 陈敏聪	清空部分运行时间
#if STM32_MODE	//2023/9/6	陈敏聪	复位将主程序名字返回给企业
		//if(pSChannel->SPreProcess.cOriginFilename[0] != '\0')
		//strncpy(Npro_name[pSChannel->ucChannelNum].Name, pSChannel->SPreProcess.cOriginFilename, FILENAME_LENGTH_LIMIT);//修改当前显示程序的程序名
#endif
		cancelWaitingInputIO(&pSChannel->SIOCommand);
		channelStopToIDLE(pSChannel);
		memset(&pSChannel->SIOCommand, 0, sizeof(struct _SIOCommand));//清理IO缓存 增加2026.4.2
		pSChannel->SIOCommand.pSChannel = pSChannel; //清空时需重新指向指针
		break;
	default:    //2025/6/14 增
		break;
    }
	return;
}
/*************************************************
  函数名称：stopAxisAfterError(struct SChannel* pSChannel)
  函数功能  报警后轴停，开通时 各轴不考虑轨迹减速停 然后将系统状态置为IDLE
  输入参数：
  输出参数：
  返 回 值：
  备    注：不清除当前数据
**************************************************/
void stopAxisAfterError(struct SChannel* pSChannel)
{
	struct SG0* pSG0;
	uint8 ucFinishedAxisNum;//完成插补的轴数
	if (pSChannel == NULL)//通道为空
	{
		return;
	}
	if (
#if STM32_MODE
		Check_bootwarning() == 1 &&//没有报警则退出函数，监控企业管理的主轴报警、伺服报警、超时报警、限位报警
#endif
		isErrorHasBeenWritten() == FALSE)//没有报警则退出函数，监控学校管理的报警
	{
		return;
	}
	pSG0 = &pSChannel->SG00;

	switch (pSChannel->SPreProcess.ucRunStep)
	{
	case 0://初始化进给轴参数以及设置主轴减速停

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
		pSChannel->bUseVirtualY = FALSE;//减速过程关闭合成虚拟Y轴功能
#endif 
		//进给轴减速停
		singleAxisSlowDownLoop(pSG0, AXIS_NUM_MAX);

		//主轴减速停
		spindleCommonStop(&pSChannel->SSpindle);

		//动力刀减速停
		spindleCommonStop(&pSChannel->SSecondSpindle);

		//下一步
		pSChannel->SPreProcess.ucRunStep += 1;
	case 1:
		//清空XYZABC轴的脉冲增量缓存区 0011 1111
		initSRunPulseBuf(&pSChannel->SRun, 0x3F);

		//单轴运动循环
		singleAxisMotionLoop(pSG0, AXIS_NUM_MAX, FALSE, FALSE);

		if (pSG0->ucCurrentSigned == pSG0->ucFinishedSigned) //参考复位运动检测轴运动结束，2026.4.30
		{
            /* 苏向标 2026-10-14：报警停止同样等待完整收尾。 */
            if (isSpindleStopFinished(&pSChannel->SSpindle)
                && isSpindleStopFinished(&pSChannel->SSecondSpindle))
			{
				pSChannel->SPreProcess.ucRunStep += 1;//下一步
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
			pSChannel->bUseVirtualY = g_SSystemPara.bUseVirtualY;//减速完成重新获取系统参数：是否开启合成虚拟Y轴功能
#endif 
			}
		}
		break;
	case 2:
		initSRunPulseBuf(&pSChannel->SRun, 0x3F);//增加清空XYZABC轴的脉冲,修改2025.11.10
		/* 报警停轴收尾时释放等待节点，再清空 IO 记录。 */
		cancelWaitingInputIO(&pSChannel->SIOCommand);
		memset(&pSChannel->SIOCommand, 0, sizeof(struct _SIOCommand));//清理IO缓存 增加2026.4.2
		pSChannel->SIOCommand.pSChannel = pSChannel; //清空时需重新指向指针
		pSChannel->STIOCommand.pSChannel = pSChannel;
		pSChannel->STIOCommand.ucToolChangeStatus = TOOLCHANGE_IDLE;
		pSChannel->STIOCommand.ucTIORun = 0;
		pSChannel->STIOCommand.ucToolCompNumRun = 0;
		pSChannel->STIOCommand.ucReadIO_Counter = 0;
		G33ResetInit(pSChannel);//G33复位初始化
		if (pSChannel->ucChannelStatus != SYS_EMERGENCY_STOP)//非急停报警时，在轴停后设置系统状态为空闲状态
		{
			pSChannel->ucChannelStatus = SYS_IDLE;
		}
		if (pSChannel->ucChannelStatus == SYS_EMERGENCY_STOP && ioEMERGENCY_STOP == g_SSystemPara.ucEmergencyStopType)//2025.5.7熊骏
		{ //急停被拉起 并且在急停状态  修改为空闲
			pSChannel->ucChannelStatus = SYS_IDLE;
		}
		break;
}
	return;
}

/*************************************************
  函数名称：savePausePoint(struct SInterpolationPre* pSPreProcess)
  函数功能  保存暂停点
  输入参数：ucAxis，轴号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 savePausePoint(struct SInterpolationPre* pSPreProcess)
{
	if (pSPreProcess == NULL)
	{
		return -1;
	}
	memset(pSPreProcess->SPausePoint.dArray, 0, sizeof(SPointDouble));//初始化暂停点 2023-12-13 洪冠鹏
	////全部赋值  当前点保存到预处理结构体的暂停点
	getRealTimePoint((double*)&pSPreProcess->SPausePoint, AXIS_NUM_MAX, pSPreProcess->pSChannel->pMapAxisArray, NULL);
#if(MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
	//多通道共用轴的机型：暂停点应保存通道都停下来后的实时点。如通道1先停，则通道1的暂停点需等到通道0停下后才可最终确定。 2023-12-13 洪冠鹏
	if (g_SChannel[0].ucChannelStatus == SYS_RUN_TO_PAUSE && g_SChannel[1].ucChannelStatus == SYS_PAUSE)
	{
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			g_SChannel[1].SPreProcess.SPausePoint.dArray[i] = g_SChannel[0].SPreProcess.SPausePoint.dArray[i];
		}
	}
	if (g_SChannel[1].ucChannelStatus == SYS_RUN_TO_PAUSE && g_SChannel[0].ucChannelStatus == SYS_PAUSE)
	{
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			g_SChannel[0].SPreProcess.SPausePoint.dArray[i] = g_SChannel[1].SPreProcess.SPausePoint.dArray[i];
		}
	}
#endif
	return 1;
}

/*************************************************
  函数名称：ChipBreak(struct SChannel* pSChannel)
  函数功能  连续进给运动的断屑功能 G4.9 P__ Q__ (P:断屑暂停时间，单位ms；Q:断屑间隔长度，单位mm)
  输入参数：pSChannel 通道结构体指针
  输出参数：
  返 回 值：-1:未开启断屑指令/不支持断屑的运动指令
			0：不为连续进给运动,清空累计值
			1：进给轴的进给量满足断屑间距，触发暂停后开始断屑
			2：进给轴运动结束，记录不足断屑间距但已走的距离到累计值
  备    注：
**************************************************/
int8 ChipBreak(struct SChannel* pSChannel)
{
	struct SInterpolationPre* pSPreProcess;//预处理结构体
	pSPreProcess = &pSChannel->SPreProcess;//预处理结构体

	if (pSPreProcess->bChipBreakFlag == FALSE //断屑功能未开启/不为G1G2G3运动
		|| (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_LINE && getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_ARC && getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_ANTI_ARC))
	{
		pSChannel->dLessChipBreakLPreDist = 0;//累计值清0
		return -1;
	}
	if (pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE && pSChannel->bChipBreakStart == FALSE)//程序暂停
	{
		pSChannel->dLessChipBreakLPreDist = 0;//累计值清0
		return 0;
	}
	if (pSChannel->SG00.bIsUserData == TRUE || pSChannel->SG01.bIsUserData == TRUE || pSChannel->SG02.bIsUserData == TRUE)//手动/手轮下进给轴发生移动（目前只用SG00.bIsUserData）
	{
		pSChannel->dLessChipBreakLPreDist = 0;//累计值清0
		return 0;
	}
	if (isZero(pSPreProcess->SUserData.dDistance))//非运动指令
	{
		pSChannel->dLessChipBreakLPreDist = 0;//累计值清0
		return 0;
	}
	if (pSChannel->ucChannelStatus == SYS_RUN)//运行过程1.检测进给距离是否达到断屑间隔长度 2.计算不足断屑间隔长度的已走距离
	{
		//底层运动中相对起点的距离是局部变量，此处采用“剩余未走距离”作为求解进给距离的中间变量
		double dCurLeft = 0;
		dCurLeft = getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE ? pSChannel->SG01.SPlanData.dCurLeftDist : pSChannel->SG02.SPlanData.dCurLeftDist;

		if (pSPreProcess->dChipBreakLength < 1.0)//断屑间隔长度缺省值为1.0
		{
			pSPreProcess->dChipBreakLength = 1.0;//输入小于1mm时取1mm
		}

		//线段执行过程中，每进给“断屑间隔长度”后暂停一次(进给距离需考虑上一段不足断屑间隔长度但已走的距离)
		if (pSChannel->SG01.ucInpStatus == INP_RUNNING || pSChannel->SG02.ucInpStatus == INP_RUNNING)
		{
			if (pSChannel->dLessChipBreakLPreDist + pSPreProcess->SUserData.dDistance - pSPreProcess->SUserData.dPreDist - dCurLeft >= pSPreProcess->dChipBreakLength)//进给距离达到断屑间隔长度
			{
				pauseButtonAPI(pSChannel->ucChannelNum);//调用暂停函数
				pSChannel->dLessChipBreakLPreDist = 0;//进入断屑后累计值清0
				pSChannel->bChipBreakStart = TRUE;//开始断屑标志位置TRUE zxr // 暂停接口pauseButtonAPI已清标志；这里重新标记为断屑暂停，供暂停收尾时转入延时续行。此标志位还用于区分普通暂停和断屑暂停
				return 1;
			}
		}

		//线段完成插补后，计算出不足间隔长度的距离，累计至下一线段判断是否到达断屑间隔长度
		if (pSPreProcess->ucInpStatus == INP_FINISHED)
		{
			if (pSChannel->bSingleLineMode == TRUE)
			{
				pSChannel->dLessChipBreakLPreDist = 0;//单节执行完当前线段后累计值清0
			}
			else
			{
				pSChannel->dLessChipBreakLPreDist += pSPreProcess->SUserData.dDistance - pSPreProcess->SUserData.dPreDist;//计算不足间隔长度的距离 zxr 当前断执行完dCurLeft为0
			}
			return 2;
		}
	}
}

/*************************************************
  函数名称：isChannelSpindleStopPending
  函数功能：判断当前通道主、副轴是否仍有停止请求或停止收尾
  输入参数：pSChannel - 当前通道，不能为空
  输出参数：无
  返 回 值：TRUE - 已有停止尚未完成；FALSE - 无待完成的停止
  备    注：周期内只读当前通道；正常旋转不触发等待，不主动停止主轴
****************************************/
static bool isChannelSpindleStopPending(const struct SChannel* pSChannel)
{
    const struct _SSpindle* pSSpindle = &pSChannel->SSpindle;
    const struct _SSpindle* pSecondSpindle = &pSChannel->SSecondSpindle;

    return isSpindleStopping(pSSpindle) || isSpindleStopping(pSecondSpindle)
        || (pSSpindle->pSSpindleAxis != NULL
            && pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE)
        || (pSecondSpindle->pSSpindleAxis != NULL
            && pSecondSpindle->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE);
}

/*************************************************
  函数名称：handleSystemPause
  函数功能：进给暂停回写完成后，等待已有主轴停止收尾并发布系统暂停
  输入参数：pSChannel - 当前通道；空指针直接返回
  输出参数：pSChannel - 等待时保留暂停中状态，完成时保存暂停点并更新状态
  返 回 值：无；是否发布暂停以通道状态 SYS_PAUSE 为准
  备    注：周期内调用；等待时使用 INP_PAUSING 保留公共出口的重试路径，
            不重复回写余程；完成后关闭断屑并保留手轮构建续行处理
****************************************/
void handleSystemPause(struct SChannel* pSChannel)
{
    // 1. 可靠性检查：确保通道指针有效
    if (pSChannel == NULL)
    {
        return;
    }
    struct SInterpolationPre* pSPreProcess = &pSChannel->SPreProcess;

    /* 已回写的进给保持暂停中；G84 首段等待也须能从公共出口再次检查。 */
    if (isChannelSpindleStopPending(pSChannel))
    {
        /* 已完成的父状态仍供主轴接管旧占用；未完成的直接调用转为可重试状态。 */
        if (pSPreProcess->ucInpStatus != INP_FINISHED)
        {
            pSPreProcess->ucInpStatus = INP_PAUSING;
        }
        return;
    }

    // 2. 业务逻辑处理
    //程序暂停后断屑功能关闭
    pSPreProcess->bChipBreakFlag = FALSE;

    //保存暂停点
    savePausePoint(pSPreProcess); 

    //修改系统状态为暂停
    pSChannel->ucChannelStatus = SYS_PAUSE; 

    //插补状态设为完成
    pSPreProcess->ucInpStatus = INP_FINISHED;

    //手轮模拟特殊处理
    if (pSChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_BUILDING)
    {
        // runButtonAPI 通常需要通道号
        runButtonAPI(pSChannel->ucChannelNum, TRUE);
    }
}



#if HANDWHEEL_SIM_DISTANCE_ENABLE
/*************************************************
  函数名称：adjustHandwheelHistoryReadPosition
  函数功能：段间换向时校准读游标，避免端点立即反摇跳过刚完成的段
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 原环形缓冲游标与读段方向
  返 回 值：无
  备    注：不使用前移 API 扩大历史窗口；只在已完成且全轴零速时调用
****************************************/
static void adjustHandwheelHistoryReadPosition(struct SChannel* pChannel)
{
    struct SHandwheelSimGroup* pGroup = &pChannel->SHandwheelSim; /**< 段间方向记录。 */
    struct SRingBuff* pBuffer = &g_SRingBuff[pChannel->ucChannelNum]; /**< 原历史缓冲。 */
    if (pGroup->bInstructionBound || pChannel->ucChannelStatus != SYS_RUN
        || pGroup->SPath.bBound
        || !isHandwheelSimGroupStopped(pGroup))
        return;
    if (pGroup->cBoundaryDirection == INP_GO_AHEAD && pGroup->cInputDirection == INP_GO_BACK
        && hasHandwheelReverseHistory(pChannel, TRUE))
    {
        ++pBuffer->nInterpIndex; /* BACK 读取 index-2，校准后恰好读刚完成的段。 */
        pChannel->SPreProcess.cInterpDir = INP_GO_BACK;
        pGroup->cBoundaryDirection = 0;
    }
    else if (pGroup->cBoundaryDirection == INP_GO_BACK && pGroup->cInputDirection == INP_GO_AHEAD)
    {
        --pBuffer->nInterpIndex; /* 刚退到本段起点，转正后应重读本段而非跳到下一段。 */
        pChannel->SPreProcess.cInterpDir = INP_GO_AHEAD;
        pGroup->cBoundaryDirection = 0;
    }
    pGroup->cInputDirection = 0;
}
#endif

/*************************************************
  函数名称：MultiLine_interpolationPre()
  函数功能  判断"规划读"指针是否有内容可读，没有-返回报错
			有-根据读出的参数进行多行插补预处理
  输入参数：
  输出参数：
  返 回 值：6	预处理完成，不需要执行IO
			5	预处理完成，前作用和后作用IO一起执行
			4	预处理完成，单节运行中释放出合并的G代码行
			1:  暂停
			0:  预处理成功
			-1: 缓存不可读，预处理失败
			-2: 不应该出现的系统状态
			-3	空指针
  备    注：
**************************************************/
int8 MultiLine_interpolationPre(struct SInterpolationPre* pSPreProcess, struct _SQuickHandleFlag* pSQuickHandleFlag)
{

	struct SChannel* pSChannel;
	enum SYSTEMSTATUS* pSystemStatus;//系统状态
	struct _SRun* pSRun;//运行结构体
	enum RingBufPointerType PointerType;//指针类型
	struct SToolCompensation* pSToolCompPointer = NULL;		  //刀补后产生的结构体指针 指向当前缓冲区单元
	struct SToolCompensation* pNextSToolCompPointer = NULL;   //刀补后产生的结构体指针 指向下一个缓冲区单元
	int nCount = 0;

	if (pSPreProcess == NULL || pSQuickHandleFlag == NULL)
	{
		return -3;
	}

	pSChannel = pSPreProcess->pSChannel;//通道指针
	pSystemStatus = &pSChannel->ucChannelStatus;//通道状态
	pSRun = &pSChannel->SRun;//运行结构体指针

	//根据插补方向选择不同的指针类型
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    if (isHandwheelDistanceControlActive(pSChannel))
        adjustHandwheelHistoryReadPosition(pSChannel);
#endif

	PointerType = (pSPreProcess->cInterpDir != INP_GO_BACK) ? INTERPOLATION_READ : INTERPOLATION_BACK;//zxr 初始化为正向读，在本文件61行，第一次进入此处只是选择正向读指针

	if (*pSystemStatus == SYS_PAUSE_TO_RUN) // PAUSE TO RUN 不能进入该函数 返回
	{
		return -2;
	}

	if (getSToolCompPointerBufRes(PointerType, pSChannel->ucChannelNum) <= 0)//若"速度规划读"指针没有可读内容
	{
		if (*pSystemStatus == SYS_RUN)
		{
#if HANDWHEEL_SIM_DISTANCE_ENABLE
			if (isHandwheelDistanceControlActive(pSChannel))
				return -1; /* 未绑定运动段：丢弃输入并等待。 */
#endif
			//根据手轮模拟倍率来赋值方向
			pSPreProcess->cInterpDir = pSChannel->SRun.dHandWheelSpeedRatio < 0 ? INP_GO_BACK : INP_GO_AHEAD;//2025.5.7熊骏
		#if EN_DEBUG
		    printf("No ReadBufs -- SYS_RUN\n");
		#endif
		}
		if (*pSystemStatus == SYS_RUN_TO_PAUSE)//无可读缓存时进入暂停收尾；已有主副轴停止未完成时仍需等待
		{
			if (pSChannel->bChipBreakStart == TRUE  //G4.9 断屑指令：以暂停方式完成断屑，停下后则进入断屑暂停时间的延时  2024/7/25 洪冠鹏
				&& (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE || getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC || getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC))
			{
				pSPreProcess->ulDelayTime = pSPreProcess->unChipBreakDelayTime;
				//*pSystemStatus = SYS_PAUSE; //修改系统状态,若不修改则无法启动
				//runButtonAPI(pSChannel->ucChannelNum, TRUE);
				*pSystemStatus = SYS_RUN;
				pSChannel->bChipBreakStart = FALSE;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	if (pSQuickHandleFlag->cFunctionReturnValue < 0)
		return pSQuickHandleFlag->cFunctionReturnValue;
#endif
	            if (pSPreProcess->pSChannel->bSingleLineMode == TRUE)
				{
					pSPreProcess->pSChannel->ucRunCounter++;//运行计数+1
				}
			}
			else
			{
				handleSystemPause(pSChannel);
			}

			return 0;
		}

		if (*pSystemStatus == SYS_PAUSE ||
			*pSystemStatus == SYS_IDLE ||
			*pSystemStatus == SYS_PAUSE_TO_RUN)//运行过程中无数据可读 中断处理不过来 （或者程序没有M30 M02）报错
		{
#if EN_DEBUG
			printf("error System status (%d)\n", *pSystemStatus);
#endif
		}

		if (*pSystemStatus == SYS_RESET) //一般情况下进不来
		{
			initAllBuffer(pSChannel->ucChannelNum); //复位  把缓冲区初始化
			*pSystemStatus = SYS_IDLE; //系统状态改为空闲
		}
		pSQuickHandleFlag->cNeedQuickHandleFlag = FALSE;
		pSQuickHandleFlag->cCycleCounter = 0;
		return -1;
	}

	//获取"速度规划读"指针
	pSToolCompPointer = getSToolCompPointerByType(PointerType, pSChannel->ucChannelNum);
	// zxr: 仅下一条有效M5准停等待上次准停收尾，保留G0 C等后继指令的原有接续流程
	if (pSToolCompPointer->bValidQ == TRUE
		&& findIO(5, pSToolCompPointer->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 5
		&& pSChannel->SSpindle.pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE)
	{
		return -1;
	}
	machineTimerStart(pSPreProcess->pSChannel->ucChannelNum);//加工计时启动
	//根据刀补缓冲区数据 初始化 插补预处理结构体
	if (initInterpStructByTool(pSToolCompPointer, pSPreProcess, pSPreProcess->cInterpDir) != 1)
	{
		// zxr：仅倒走遇到不可回退段执行（带`bLastLineMark`的程序末段、带M代码／辅助IO的段、多通道下带通道指令的段），调整游标并截断回退历史，不执行该段
		moveSToolCompBufPointer(INTERPOLATION_BACK, BUF_AHEAD, pSChannel->ucChannelNum);// zxr 游标后退跳过这一行 (nInterpIndex -= 1)
		clearInterpolationDealedNum(pSChannel->ucChannelNum);//清除插补处理过的数据   zxr 重设回退边界，使倒走可读段数为0，不清空缓存内容
		return 0;//zxr 本次未装载新段，退出预处理
	}

	//指令类型赋值 (传递到SRun)
	pSRun->unCommandType = pSPreProcess->unCommandType;//指令类型
	strncpy(pSRun->cFilename, pSPreProcess->cFilename, FILENAME_LENGTH_LIMIT);//复制文件名
	if (pSPreProcess->cOriginFilename[0] == '\0')
	{
		strncpy(pSPreProcess->cOriginFilename, pSRun->cFilename, FILENAME_LENGTH_LIMIT);
	}
#if STM32_MODE

	strncpy(Npro_name[pSChannel->ucChannelNum].Name, pSPreProcess->cFilename, FILENAME_LENGTH_LIMIT);//修改当前显示程序的程序名

#endif
	pSRun->unGCodeLineNum = pSPreProcess->unGCodeLineNum;//G代码行号

	pSPreProcess->ucInpStatus = INP_BLOCKED;				//阻塞状态（等待主轴达到指令转速才能运行）
	pSChannel->SApplyTime.unSpindleSpeedUpAlarmRunTime = 0;	//主轴加速超时计时器
	//偏移计算
	pSRun->ucToolNum = pSPreProcess->ucToolNum;						//刀具号
	pSRun->ucToolCompNum = pSPreProcess->ucToolCompNum;			//磨损组号
	pSRun->cWorkpieceCoordNum = pSPreProcess->cWorkpieceCoordNum;	//工件坐标号
	pSRun->SWorkPieceOffsetByG50 = g_SZeroPtOfCoordSys->SWorkPieceOffsetByG50; //获取G50临时坐标系偏移量并传递给系统参数 2023-8-3洪冠鹏
	//pSRun->SWorkPieceOffsetByG50 = g_SZeroPtOfCoordSys->SWorkPieceOffsetByG50 = pSPreProcess->SWorkPieceOffsetByG50; //获取G50临时坐标系偏移量并传递给系统参数 2023-8-3洪冠鹏

	//计算总偏移  赋值给SRun结构体  用于显示
	TChangeTotalOffset(pSPreProcess->pSChannel->ucChannelNum, pSRun->ucToolCompNum, pSRun->cWorkpieceCoordNum, &pSRun->SCurrentOffset, &pSRun->SWorkPieceOffsetByG50);

	switch (pSToolCompPointer->cQuickHandleType)
	{
	case GCODE_FULL_MERGED:		//G代码完全合并类型
	case GCODE_QUICKHANDLE:		//G代码可快速处理类型
		pSQuickHandleFlag->cFunctionReturnValue = QuicklyinterpolationPre(pSPreProcess, pSToolCompPointer, PointerType, pSQuickHandleFlag);//zxr 内部调用前作用IO函数入队及发起主轴M03/M04启动
		break;
	default:	//G代码不可快速处理，原本的预处理版本
		pSQuickHandleFlag->cFunctionReturnValue = CommoninterpolationPre(pSPreProcess, pSToolCompPointer, PointerType, pSQuickHandleFlag);//zxr 内部调用前作用IO函数入队及发起主轴M03/M04启动
		break;
	}
	//单节执行  1.当前行未完成标志位置TURE；2.复合指令/程序结束指令/程序循环指令需对运行计数+1  洪冠鹏 2024/5/21
	if (pSPreProcess->pSChannel->bSingleLineMode == TRUE)
	{
		pSPreProcess->pSChannel->bSingleLineModeUnFinished = TRUE;	//当前行未完成标志位置TRUE,使运行按键在单节执行过程时按下后不修改运行计数
		// 2.判断当前单元是否为最后一行，若为最后一行，则运行计数+1
		if (getBits(pSPreProcess->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_STOP || pSPreProcess->bLastLineMark == TRUE)//最后一行指令（程序结束/循环指令）
		{
			pSPreProcess->pSChannel->ucRunCounter++;//运行计数+1，M30/M20/M99为后作用指令
		}
	}
	return pSQuickHandleFlag->cFunctionReturnValue;
}


/*************************************************
  函数名称： IRQHandler(struct SChannel *pSChannel)
  函数功能  1ms的中断内需要处理的内容
  输入参数：pSChannel 通道结构体指针
  输出参数：
  返 回 值：
  备    注：包含功能：1、G04暂停 2、阻塞功能 3、预处理  4、通道指令执行 5、刀补轴占用 6、检测主轴转速  7、按指令类型运动 8、恢复加工  9、前后作用IO
**************************************************/
void IRQHandler(struct SChannel* pSChannel)//中断服务函数
{
	struct SInterpolationPre* pSPreProcess;//预处理结构体
	enum SYSTEMSTATUS* pSystemStatus;
	struct _SSpindle* pSSpindle;
	struct _SSpindle* pSecondSpindle;

	struct _SQuickHandleFlag SQuickHandleFlag;			//4.17 陈敏聪
	SQuickHandleFlag.cNeedQuickHandleFlag = FALSE;		//初始化快速处理结构体，本周期不需要快速处理
	SQuickHandleFlag.cCycleCounter = 0;					//初始化快速处理结构体，本周期已处理循环0次
	int8 ucTurretAxisOccupyRes = 0;   //刀塔轴占用结果
	if (pSChannel == NULL)
	{
		return;
	}

	pSPreProcess = &pSChannel->SPreProcess;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	if (isHandwheelDistanceControlActive(pSChannel)
		&& (pSChannel->ucChannelStatus == SYS_PAUSE || pSChannel->ucChannelStatus == SYS_IDLE)
		&& isHandwheelSimGroupStopped(&pSChannel->SHandwheelSim))
		clearHandwheelSimMotion(&pSChannel->SHandwheelSim);
#endif

	pSystemStatus = &pSChannel->ucChannelStatus;
	pSSpindle = &pSChannel->SSpindle;
	pSecondSpindle = &pSChannel->SSecondSpindle;

	if (*pSystemStatus == SYS_RESET)//复位不执行
	{
		return;
	}

	// initSRunPulseBuf(&pSChannel->SRun, 0x3F); //清空XYZABC轴的脉冲增量缓存区 0011 1111 //修改2025.10.28调整位置

	if (
#if STM32_MODE
		Check_bootwarning() == 0 || //正在报警则退出函数，企业管理的报警主轴报警、伺服报警、超时报警、限位报警
#endif
		isErrorHasBeenWritten() == TRUE)//正在报警则退出函数，学校管理的报警  2024/10/16
	{
		return;
	}
	initSRunPulseBuf(&pSChannel->SRun, 0x3F); // 清空XYZABC轴的脉冲增量缓存区 0011 1111 //修改2025.10.28调整位置
	if (pSPreProcess->ulDelayTime != 0)  //G04暂停
	{
		/* G84 孔底停留仍处于攻丝循环，保留前段轴所有权直到下一同步段接管。 */
			releaseLastCommandAxis(pSPreProcess);

        /* 主轴尚在收尾时冻结 G4 余时和读指针，不消耗最后一拍。 */
        if (*pSystemStatus == SYS_RUN_TO_PAUSE && isChannelSpindleStopPending(pSChannel))
        {
            return;
        }
		pSPreProcess->ulDelayTime -= 1;  //暂停时间-1

		if (*pSystemStatus == SYS_RUN_TO_PAUSE)//2024.8.10
		{
			struct SToolCompensation* pSToolCompPointer;//指向刀补后产生的结构体的指针
			moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pSChannel->ucChannelNum);  //移回插补读指针
			pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSChannel->ucChannelNum);//获取插补读指针
			pSToolCompPointer->ulDelayTime = pSPreProcess->ulDelayTime; //回写延时时间
			pSPreProcess->ulDelayTime = 0;
			savePausePoint(pSPreProcess);// 保存暂停点
			*pSystemStatus = SYS_PAUSE;//系统状态 暂停

		}

		if (pSPreProcess->ulDelayTime == 0)//G4暂停时间为0   G4暂停完成
		{
			//G04暂停完毕 下一段预处理
			pSPreProcess->ucInpStatus = INP_FINISHED;

			//if (*pSystemStatus == SYS_RUN_TO_PAUSE && g_SChannel->cHandWheelSimulaStatus == HANWHEELSIMULA_OFF) //在非手轮试切的情况下，G4过程中暂停 直接更改系统状态  2023/12/22
			//{
			//	savePausePoint(pSPreProcess);// 保存暂停点	//陈敏聪	2023/12/1
			//	*pSystemStatus = SYS_PAUSE;//系统状态 暂停
			//}
		}
		return;
	}

	if (pSChannel->SIOCommand.ucIOBlockedCounter != 0)
	{
		//处于阻塞状态时 只改变主轴转速 和执行快速运动
		for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE; ucounter++)
		{
			if (pSChannel->SIOCommand.SVaribleIOInfoArray[ucounter].pIOContrlInfo != NULL && *pSystemStatus != SYS_RUN_TO_PAUSE)
				return;
		}
	}

	if (pSChannel->STIOCommand.ucToolChangeStatus != TOOLCHANGE_IDLE && getBits(pSPreProcess->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)
	{
		//刀塔模式正在换刀 只改变主轴转速 和执行快速运动
		if (pSPreProcess->ucInpStatus == INP_FINISHED)
		{
			return;
		}
	}


	if (pSChannel->SSpindle.cSpindleClampStatus == STATUS_GET_CLAMP_INSTRUCTION || pSChannel->SSpindle.cSpindleClampStatus == STATUS_GET_UNCLAMP_INSTRUCTION)
	{
		//主轴正在锁紧或松开状态中阻塞
		return;
	}
#if SWING_ARM_MACHINE == 1 
	if (pSChannel->SIOCommand.ucMStatus == 1)//摆臂机M码状态 2024/6/5
	{
		pSPreProcess->ucInpStatus = INP_BLOCKED;//M码含运动指令，设置阻塞状态确保逐个M码顺序执行
	}
#endif

	/*********************************初始化部分****************************************/
    /* 苏向标 2026-10-14：后续合法指令延后取出，不丢弃，也不覆盖停止。 */
	if (!isSpindleStopping(pSSpindle) && !isSpindleStopping(pSecondSpindle)
        && (*pSystemStatus == SYS_RUN || *pSystemStatus == SYS_RUN_TO_PAUSE) //初始化条件：系统状态为运行
		&& (pSPreProcess->ucInpStatus == INP_FINISHED || pSPreProcess->ucInpStatus == INP_PAUSING)//插补完成
#if STM32_MODE
		&& (pSPreProcess->ucG28ZeroingValAxis == 0)//没有需要回零的轴
#endif
#if CHANNEL_NUM_MAX > 1
		&& pSPreProcess->sChannelCommand.ucType == UNASSIGNED_VALUE   //多通道需要判断通道指令执行完成   2022.12.6    <0 改为 <=0
#endif // CHANNEL_NUM_MAX > 1
		)
	{
		//朱稳中 2023.5.19 -> 陈敏聪 2023/5/26 增加自动模式的判断 -> 陈敏聪 2023/10/5 增加MDI模式的判断 -> 洪冠鹏 2024/5/19 已对复合指令处理，删去G34单节运行不停下标志位的判断
		//if (pSChannel->ucRunCounter == 0 && pSPreProcess->bUnStoptableLine == FALSE && (g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE))//单节执行
		if (pSChannel->ucRunCounter == 0 && (g_RunMode == AUTOMATIC_MODE || g_RunMode == MDI_MODE))//单节执行结束的判断
		{
			if (*pSystemStatus != SYS_RUN_TO_PAUSE)
			{
				pSChannel->bSingleLineModeUnFinished = FALSE;//当前行已完成  2024/5/21 洪冠鹏
				return;
			}
		}

		if (getBits(pSPreProcess->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_PAUSE
			|| getBits(pSPreProcess->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_OPTION_PAUSE)
		{
			/* M0/M1 都在主、副轴已有的停止收尾后才消费本段。 */
			if (isChannelSpindleStopPending(pSChannel))
				return;
			*pSystemStatus = SYS_PAUSE;
			setBits(pSPreProcess->unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);
			savePausePoint(pSPreProcess);// 保存暂停点
			return;
		}

		//当前段为M30或M02程序结束
		if (getBits(pSPreProcess->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_STOP || getBits(pSPreProcess->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_STOP_M02)
		{
			// zxr: 主副轴任一仍处于准停模式时，本周期返回；待均退出准停后，再执行后续计件和通道结束处理
			if ((pSSpindle->pSSpindleAxis != NULL
				&& pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE)
				|| (pSecondSpindle->pSSpindleAxis != NULL
					&& pSecondSpindle->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE))
			{
				return;
			}
			calWorkpieceNum(pSChannel->ucChannelNum);//计件  //2024/7/12 调整位置 需等待主轴停下后程序才停止
			machineTimerMark(pSChannel->ucChannelNum);//计时
			g_SSystemPara.bIsG50ProgameHasFinished = 0;//G50所在程序执行完后结束 2023/12/23 洪冠鹏
			//通道停止运行

			channelStopToIDLE(pSChannel); // 陈敏聪	2023/12/23
			
			if (pSPreProcess->bLastLineMark == TRUE) // 陈敏聪	2023/9/1增
			{
				pSPreProcess->bLastLineMark = FALSE;	//标志位复位
			}
			return;
		}

		if (pSPreProcess->bLastLineMark == TRUE)	//M30判断完后再判断是否为最后一行(M99）zxr： bLastLineMark会被M02、M30和M99置位，可以用来判断M99
		{
			pSPreProcess->bLastLineMark = FALSE;
			machineTimerMark(pSChannel->ucChannelNum);//计时

			pSChannel->cRunChannelGcodeAgain = TRUE;	//陈敏聪	2023/8/28增 处于M99的情况下，让G代码重新启动

			if (pSChannel->bSingleLineMode == TRUE) //洪冠鹏 2024/5/29 单节执行下的M99等待按下运行按键后才开启新的循环
			{
				pSChannel->ucRunCounter = 0;//运行计数置0
				return;						//M99所在行执行完毕
			}
			// 计件
			if (calWorkpieceNum(pSChannel->ucChannelNum) == 1)
			{
				if(g_RunMode == AUTOMATIC_MODE)//自动执行模式，达标才关闭旋转轴 // 2026.4.30
				{
					//主轴减速停
					spindleCommonStop(&pSChannel->SSpindle);

					//动力刀减速停
					spindleCommonStop(&pSChannel->SSecondSpindle);
				}

				//通道停止运行
				channelStopToIDLE(pSChannel);
				return;
			}

			//			if (pSChannel->bSelectableStop == TRUE)//选择停止启动状态  由于新增M01选择停止功能，企业要求注释原有功能  2023/10/7 洪冠鹏
			//			{
			//				//通道停止运行
			//				channelStopToIDLE(pSChannel, TRUE);
			//				return;
			//			}

		}
		//手动完成后停止
		if (getBits(pSPreProcess->unCommandType, CMD_PROGRAM_ADDR) == CMD_PROGRAM_FINISH)
		{
			//通道停止运行，但是不停主轴
			channelStopToIDLE(pSChannel);
			return;
		}

		do
		{
			SQuickHandleFlag.cFunctionReturnValue = MultiLine_interpolationPre(pSPreProcess, &SQuickHandleFlag);			//进行下一线段的插补预处理
		}
		//1. 系统开启了多行快速处理 2. 底层返回正常处理状态(>3) 3.单次中断处理行数未达到硬件安全上限 4.底层主动请求继续连续处理(cNeedQuickHandleFlag == TRUE)
		//zxr 有效M5准停必须先派发，禁止多行预处理直接覆盖本段角度请求
		while (g_SSystemPara.bGcodeQuickHandleModeEnabled && (SQuickHandleFlag.cFunctionReturnValue > 3) &&
			!(pSPreProcess->bValidQ == TRUE && findIO(5, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 5) &&
			(SQuickHandleFlag.cCycleCounter < g_SSystemPara.sInterpolation_MultiLineHandleMaxNum && SQuickHandleFlag.cNeedQuickHandleFlag == TRUE));

		if (pSChannel->bSingleLineMode == TRUE && SQuickHandleFlag.cFunctionReturnValue > 0)
		{
			pSChannel->ucRunCounter = (pSChannel->ucRunCounter == 0) ? 0 : pSChannel->ucRunCounter - 1;
		}

		/* 本拍新装入的 G04/G84 停留从下拍倒计时，不把空运动参数送入 G1。 */
		if (pSPreProcess->ulDelayTime != 0)
		{
			return;
		}

		//更新G96的主轴转速  2023.5.13 朱稳中
		pSChannel->SSpindle.toUpdateG96SpindleSpeed = TRUE;

		if (pSChannel->SIOCommand.ucIOBlockedCounter != 0 && getBits(pSPreProcess->unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_QUICKMOVE)
		{//处于阻塞状态时 只改变主轴转速 和执行快速运动 (此时插补结构体状态为BLOCKED，不会再一次预处理)
			return;
		}

		if (pSChannel->STIOCommand.ucToolChangeStatus != TOOLCHANGE_IDLE && getBits(pSPreProcess->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)
		{
			if (pSPreProcess->ucInpStatus == INP_FINISHED)
			{
				return;
			}
		}

		if (pSChannel->SSpindle.cSpindleClampStatus == STATUS_GET_CLAMP_INSTRUCTION || pSChannel->SSpindle.cSpindleClampStatus == STATUS_GET_UNCLAMP_INSTRUCTION)
		{
			//主轴正在锁紧或松开状态中阻塞
			return;
		}
	}

#if CHANNEL_NUM_MAX > 1  //处于多通道时启动功能(刀补占用 通道指令)
#if(TURRET_AXIS_TEST == 1)
	{
		//尝试占用刀塔和刀塔轴
		ucTurretAxisOccupyRes = turretAxisAndTurretOccupyByChannel(&pSPreProcess->SUserData.ucValStatus, pSChannel->ucChannelNum, &pSPreProcess->ucOccupideValStatus, &pSPreProcess->ucOccupideTurretAxis, &pSPreProcess->ucOccupideTurret);
		if (ucTurretAxisOccupyRes == 1)//占用成功
		{

		}
		else if (ucTurretAxisOccupyRes == 0)//占用失败的原因是刀塔物理轴未使能  ，可以继续调用该函数进行占用
		{
			return;
		}
		else//出错
		{
			return;
		}

	}
#else
	//刀补轴占用判断模块
	if (pSPreProcess->ucToolCompensationStatus == TOOL_COMPENSATION_BUILD)//刀补建立阶段 需要对两个轴进行占用
	{
		if (axisManagerOccupyByChannel(pSPreProcess->ucToolAxis, pSChannel->ucChannelNum, NULL) == 1)	//洪冠鹏 陈敏聪 2024/3/1 //占用刀补的轴
		{	//占用成功
			pSPreProcess->ucToolCompensationStatus = TOOL_COMPENSATION_PROCEED;//刀补状态变为 “进行”
		}
		else
		{	//占用失败
			return;
		}
		}
#endif  //TURRET_AXIS_TEST 
#endif // CHANNEL_NUM_MAX > 1

#if STM32_MODE
	//G28回零（调用企业接口）
	G28FindZero(&pSPreProcess->ucG28ZeroingValAxis, pSChannel->ucChannelNum);
#endif

	/*********************************主轴转速检测（根据编码器数值判断当前主轴转速是否达到指定值）***************************************/
	if (pSPreProcess->ucInpStatus == INP_BLOCKED)//预处理结构体状态为阻塞 进入主轴转速检测
	{
		int8 cIsSecondSpindleSpeedReached;
		cIsSecondSpindleSpeedReached = getSecondSpindleSpeedReached(pSecondSpindle);

		if (pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_HANDWHEEL_MODE
			|| pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE
			|| pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_COMMONSTOP_MODE || (pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE && pSPreProcess->bValidQ == FALSE))//不进行主轴转速检测的状态 //2024/7/5 应企业要求，增加主轴停止时不检测 洪冠鹏
		{
			if (cIsSecondSpindleSpeedReached)//不进行主轴转速检测的两种状态
			{
				//退出主轴转速检测 开始非G0代码的执行
				pSPreProcess->ucInpStatus = INP_RUNNING;
			}
		}
		else //主轴处于其他状态 需要检测主轴转速
		{ 
			double dSpeedPercentage = 0; //当前转速百分比
			double dTargetSpeed = 0;	//目标转速(r/ms)带方向
			double dCurrentPos;			//主轴跟随轴的当前坐标

			//调用接口函数，读取主轴当前目标转速值(r/ms) 带方向
			if(calTargetSpindleSpeed(pSSpindle,&dTargetSpeed)!= 1)return;

			//计算当前速度与目标速度的比值（计算公式带方向）
			if (isZero(dTargetSpeed))// 目标转速为0   （分母为0)
			{
				if (isZero(pSChannel->SSpindleTest.dSpindleAvgSpeed) || isZero(g_SSystemPara.fSpindleSpeedDeviation - 1.0))//编码器相对值为0  或者允许误差为100%
				{
					dSpeedPercentage = 1.0;//认为速度达到目标值的100%
				}
			}
			else//根据转速计算当前转速百分比
			{
				//编码器方向 * 编码器数值 * 当量(pulseToR) / 目标转速
				//dSpeedPercentage = pSChannel->SSpindleTest.dSpindleAvgSpeed / dTargetSpeed;
				if (pSSpindle->pSSpindleAxis->cSpindleDir == ARC_CW)//正转
				{
					// 当前转速百分比 = 平均转速 / 目标转速
					dSpeedPercentage = pSChannel->SSpindleTest.dSpindleAvgSpeed / dTargetSpeed;
				}
				else
				{
					// 当前转速百分比 = -1.0 * 平均转速 / 目标转速
					dSpeedPercentage = -1.0 * pSChannel->SSpindleTest.dSpindleAvgSpeed / dTargetSpeed;
				}
			}

			pSChannel->dSpindleSpeedRatio = fabs(dSpeedPercentage - 1.0 * SGN(dSpeedPercentage));

			if ((fabs(dSpeedPercentage - 1.0 * SGN(dTargetSpeed)) <= g_SSystemPara.fSpindleSpeedDeviation)  //误差范围内内且方向相同
				&& (pSSpindle->pSSpindleAxis->ucInpStatus == INP_FINISHED || getBits(pSSpindle->pSSpindleAxis->unCommandType, CMD_SPINDLE_SPEED_ADDR) == CMD_SPINDLE_SPEED_CV))//主轴插补完成 或为恒转速
			{ //G96一定要加速完（status = FINISHED)才能启动G1+
				pSChannel->dSpindleSpeedRatio = 123;
				if (cIsSecondSpindleSpeedReached)//不进行主轴转速检测的两种状态
				{
					pSChannel->dSpindleSpeedRatio = 999;
					pSPreProcess->ucInpStatus = INP_RUNNING; //退出主轴转速检测 开始非G0代码的执行
				}
			}
		}
	}


	/* 新停留段在本拍预处理后才装入，必须下一拍开始计时，不能下发残留运动。 */
	if (pSPreProcess->ulDelayTime != 0)
	{
		return;
	}

	/*********************************运动执行部分***************************************/
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	/* 未接入的运动继续调用原算法，但不能继承手轮负倍率或未结束的授权。 */
	if ((*pSystemStatus == SYS_RUN || *pSystemStatus == SYS_RUN_TO_PAUSE)
		&& prepareHandwheelAutomaticMove(pSChannel) < 0)
	{
		return;
	}
#endif

	switch (*pSystemStatus)
	{
	case SYS_RUN_TO_PAUSE://正在暂停
	case SYS_RUN://运行状态
		if (
			//1.快速运动不阻塞
			getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE ||
			(
				//2.主轴加速完成不阻塞
				pSPreProcess->ucInpStatus != INP_BLOCKED
				//3.刀具轴在跟随状态下已跟随上不阻塞
#if TOOLAXIS_TEST
				&& ((g_SToolAxisInfo.bFollowed == TRUE && g_SToolAxisInfo.pSToolAxis->ucRunMode == TOOLAXIS_FOLLOW_MODE) ||
					g_SToolAxisInfo.pSToolAxis->ucRunMode != TOOLAXIS_FOLLOW_MODE)
#endif
				)
			)
		{
			switch (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR))//指令类型判断		//陈敏聪	//2023/12/1	指令类型的判断从判断预处理结构体改为判断运行结构体
			{
			case CMD_MOTION_QUICKMOVE:	//快速运动
				motionQuickMove(&pSChannel->SG00);
				break;

			case CMD_MOTION_LINE:		//合成运动
				motionLine(&pSChannel->SG01, TRUE, TRUE);
				break;

			case CMD_MOTION_ARC:		//顺圆
			case CMD_MOTION_ANTI_ARC:	//逆圆
			case CMD_MOTION_ELLIPSE_ARC:	//顺椭圆
			case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆
				motionArc(&pSChannel->SG02, TRUE, TRUE);
				break;

			case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削，zxr G92
			case CMD_MOTION_SCREWCUT_VP:  //zxr G34 遇到这两个指令类型时，不在这里进行轨迹插补，直接把当前预处理记录标记为完成，让流程继续往下走
				pSPreProcess->ucInpStatus = INP_FINISHED;
				break;

			case CMD_MOTION_SCREWCUT:  //螺纹切削
				G32Run(&pSChannel->SG32);
				break;

			case CMD_MOTION_G33://G33螺纹攻牙循环
			{
				struct _SSpindle* pG33RotateAxis = (pSChannel->SG33.G33RotateAxis == AXIS_A) ? (&pSChannel->SSecondSpindle) : (&pSChannel->SSpindle);
				struct SEncoder* pG33Encoder = (pSChannel->SG33.G33RotateAxis == AXIS_A) ? (pSChannel->pSSecondSpindleEncoder) : (pSChannel->pSSpindleEncoder);

				MotionG33(pSPreProcess, pG33RotateAxis, &pSChannel->SG33, pG33Encoder);

				break;
			}
			case CMD_MOTION_G10://G10自动刀具补偿功能
				pSPreProcess->ucInpStatus = INP_FINISHED;
				break;
			default:
				break;

			};
		}
		else
		{
			//计时计时  主轴加速超时报警
		}
		break;

	case SYS_IDLE:
	case SYS_PAUSE:
		//case SYS_PREPARE:
			//  手动  单轴运动
		//singleAxisMotionLoop(&pSChannel->SG00, AXIS_NUM_MAX, TRUE, TRUE);		//陈敏聪	2023/11/8注释，单轴运动换成G0运动
        /* 空闲和暂停仅推进已提交的手动 G0，避免旧程序 G0 覆盖 G4 暂停余量。 */
        if (pSChannel->SG00.bIsUserData)
        {
            motionQuickMove(&pSChannel->SG00);
        }
		break;
	case SYS_PAUSE_TO_RUN:
		recoveryToRun(&pSChannel->SG01); //恢复加工函数
		break;

	case SYS_RESET:
		break;
	default:
#if EN_DEBUG
		printf("error system status\n");
#endif
		break;
	}
	ChipBreak(pSChannel);//断屑功能

#if CHANNEL_NUM_MAX > 1  //处于多通道时启动功能(刀塔轴释放， 通道指令)

//运动指令完成时才执行通道指令（通道指令当作后作用指令）
	if (pSPreProcess->ucInpStatus == INP_FINISHED)
	{

		//通道指令
		if (pSPreProcess->sChannelCommand.ucType != UNASSIGNED_VALUE)//通道指令有效
		{

			if (*pSystemStatus == SYS_RUN)
			{

#if(TURRET_AXIS_TEST == 1)

				if (pSPreProcess->sChannelCommand.ucType == CHANNEL_SIGNAL_WAIT)//为通道等待指令时
				{
					releaseAllOccupiedAxis(pSPreProcess);//释放所有占用的轴（刀塔轴+非刀塔轴）
					releaseAllOccupiedTurret(pSPreProcess);//释放所有占用的刀塔
				}
#endif
				if (channelSignalApply(&pSPreProcess->cChannelBufNum, pSChannel->ucChannelNum, &pSPreProcess->sChannelCommand) == 1)//执行通道指令
				{	//返回1 指令执行完成
					//将通道指令置为无效
					pSPreProcess->cChannelBufNum = -1;
					pSPreProcess->sChannelCommand.ucType = UNASSIGNED_VALUE;
				}
			}
			else if (*pSystemStatus == SYS_RUN_TO_PAUSE && pSPreProcess->ucInpStatus == INP_FINISHED)
			{
				//只有通道指令 没有运动指令  回写（按下暂停键时，如果只有通道指令没有运动指令，在运动底层函数中因为距离为0所以插补状态为INP_FINISHED退出来，系统状态依旧为 SYS_RUN_TO_PAUSE）
				struct SToolCompensation* pSToolCompPointer;//刀补指针

				moveSToolCompBufPointer(INTERPOLATION_READ, BUF_BACK, pSChannel->ucChannelNum);  //移回插补读指针
				pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSChannel->ucChannelNum);//获取插补读指针

				//赋值
				//获取当前点写回缓冲区
				getRealTimePoint((double*)&pSToolCompPointer->SStartPoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, &pSToolCompPointer->ucValStatus);
				pSToolCompPointer->SEndPoint = pSToolCompPointer->SStartPoint;//起点等于终点
				pSToolCompPointer->dLength = 0;//长度为0
				pSToolCompPointer->ucRelaVal = 0x0;

				//写回通道指令
				pSToolCompPointer->sChannelCommand = pSPreProcess->sChannelCommand;
				//这里需要回写等待信号数值下标是因为存在等待信号执行到一半（已经写入等待信号管理器中，但还没集齐所有等待通道）而按下暂停键的情况
				pSToolCompPointer->cChannelBufNum = pSPreProcess->cChannelBufNum;
				//当前指令置为无效（下周期可进入预处理）
				pSPreProcess->sChannelCommand.ucType = UNASSIGNED_VALUE;
				pSToolCompPointer->cChannelBufNum = -1;

				clearIOCommand(pSToolCompPointer->ucIOCommandArray, IO_ALL_ACTION, IO_MAXGROUP_IN_SINGLE_LINE);//删除前后作用的IO
				setBits(pSPreProcess->unCommandType, CMD_PROGRAM_CONTINUE, CMD_PROGRAM_ADDR);//删除插补预处理结构体的M30 M02

			}
		}
	}

#endif
	// zxr 后作用处理：当前段插补完成、系统处于 SYS_RUN，且本次后作用尚待处理（bRunPostIO为TRUE）时进入
	// bRunPostIO 由预处理（Multi函数）设置；普通预处理路径中，正向取段置 TRUE，反向置 FALSE
	if (pSPreProcess->ucInpStatus == INP_FINISHED && *pSystemStatus == SYS_RUN && pSPreProcess->bRunPostIO == TRUE)
	{
		pSPreProcess->bRunPostIO = FALSE;//zxr 先清标志，避免重复执行
		
		ApplyIO(pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE, IO_POST_ACTION, &pSChannel->SIOCommand, NULL);//zxr 提交后作用IO，执行是在非实时任务执行

		//查找IObuff中是否有M05
		if (findIO(5, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 5)//2023.5.13 朱稳中
		{
			spindleStopAPI(pSSpindle, &pSPreProcess->preciseStopDegree, pSPreProcess->bValidQ);//主轴准停
		}
		//查找IObuff中是否有M52
		if (findIO(52, pSPreProcess->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 52)
		{
			spindleStopAPI(pSecondSpindle, &pSPreProcess->preciseStopDegree, FALSE);//第二轴/动力头准停
		}
	}

}
