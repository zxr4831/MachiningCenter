#include "motionAPI.h"
#include "common.h"
#include "CTrajPlanning.h"
#include "ErrorFunction.h"
#include "Gcode.h"
#include "GeneralFunctions.h"
#include "lookAhead.h"
#include "ToolCompensation.h"
#include <stdlib.h>
#include "IOCommand.h"
#include "ToolChanger.h"
#include "Spindle.h"
#include "G32.h"
#include "G1.h" 
#include "G0.h"  
#include "Motion.h"	
#include <string.h>
#include "G2.h"
#include "Channel.h"
#include "Config.h"

#define SWITCH_SEPPD_PARA(target,dVal,ucType) {switch(ucType)\
{\
case VEL_VS:\
	target.SPlanData.dVs = dVal;break;\
case VEL_VMAX:\
	target.dFeedrate = dVal;break;\
case VEL_VE:\
	target.SPlanData.dPrimalVe = dVal;break;\
case PLAN_ACC:\
	target.dAcc = dVal;break;\
case PLAN_DEC:\
	target.dDec = dVal;break;\
case PLAN_DISTANCE:\
	target.SPlanData.dPreDist = 0;\
	target.SPlanData.dDistance = target.SPlanData.dTargetDist = target.SPlanData.dCurLeftDist = dVal; break; \
default:\
return -3;\
} }


/*************************************************
  函数名称：getRealFeedRatio(uint8 ucChannelNum)
  函数功能  获得当前速率
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：INT 0~100
  备    注：
****************************************/
int getRealFeedRatio(uint8 ucChannelNum)
{
	int res = 0;
	uint8 ucPhysicalAxis; ;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	ucPhysicalAxis = g_SChannel[ucChannelNum].pMapAxisArray[AXIS_X];//X轴对应的物理轴

	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE)
	{
		//空闲状态下  显示X轴的倍率   当前速度/
		res = round(g_usCompareValue[ucPhysicalAxis] * g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm /
			(g_SChannel[ucChannelNum].SSingleAxisArray[AXIS_X].dFeedRatio * g_SChannel[ucChannelNum].SSingleAxisArray[AXIS_X].dFeedrate) * 100.0);
	}
	else
	{
		switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR))// 主轴加速完成 或者当前为快速运动  2024/10/15 熊骏
		{
		case CMD_MOTION_QUICKMOVE:
			// 显示X轴的倍率   当前速度/
			res = round(g_usCompareValue[ucPhysicalAxis] * g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm /
				(g_SChannel[ucChannelNum].SSingleAxisArray[AXIS_X].dFeedRatio * g_SChannel[ucChannelNum].SSingleAxisArray[AXIS_X].dFeedrate) * 100.0);
			break;
		default:
		case CMD_MOTION_LINE://直线插补
			res = round(g_SChannel[ucChannelNum].SG01.dCurrentSpeed / g_SChannel[ucChannelNum].SRun.dCurrentFeedRate * 100.0);
			break;

		case CMD_MOTION_ARC://顺逆圆插补
		case CMD_MOTION_ANTI_ARC:
			res = round(g_SChannel[ucChannelNum].SG02.dCurrentSpeed / g_SChannel[ucChannelNum].SRun.dCurrentFeedRate * 100.0);
			break;
		}
	}
	return res;
}


/*************************************************
  函数名称：getCurrentFeedRate(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获得当前理论进给速度
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号(G0或空闲时  返回的进给速度)
  输出参数：
  返 回 值：double   mm/min
  备    注：
****************************************/
double getCurrentFeedRate(uint8 ucChannelNum, uint8 ucAxis)
{
	double dRes = 0.0;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE)
	{
		dRes = g_SChannel[ucChannelNum].SRun.dQuickMoveFeedRatio //倍率
			* g_SChannel[ucChannelNum].SSingleAxisArray[ucAxis].dFeedrate * 60.0 * 1000.0;//进给速度
	}
	else
	{
		switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR))
		{
		case CMD_MOTION_QUICKMOVE:
			dRes = g_SChannel[ucChannelNum].SRun.dQuickMoveFeedRatio  //倍率
				* g_SChannel[ucChannelNum].SSingleAxisArray[ucAxis].dFeedrate * 60.0 * 1000.0;  //进给速度
			break;
		default:
		case CMD_MOTION_LINE:
			//dRes = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR) == CMD_FEED_PER_MIN?
			//	g_SChannel[ucChannelNum].SG01.dFeedrate * g_SChannel[ucChannelNum].SRun.dCommonFeedRatio* 60.0 * 1000.0//进给速度
			//	: fabs(g_SChannel[ucChannelNum].SSpindleTest.dSpindleAvgSpeed) * g_SChannel[ucChannelNum].SG01.dFeedrate * 60 * 1000 * 60 * 1000;//乘倍率 转换成mm/min

			//dRes = g_SChannel[ucChannelNum].SG01.dFeedrate *g_SChannel[ucChannelNum].SRun.dCommonFeedRatio * 60.0 * 1000.0;//2024/4/29注释：进给速率未考虑G99
			int i;
			double dTemp;
			double dMaxNum = 0;
			for (i = 0; i < AXIS_NUM_MAX; i++) //获取长轴分量
			{
				dTemp = fabs(g_SChannel[ucChannelNum].SG01.SStartVector.dArray[i]);
				if (dTemp > dMaxNum)
				{
					dMaxNum = dTemp;
				}
			}

			dRes = getBits(g_SChannel[ucChannelNum].SG01.unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN ?//2024/4/29 洪冠鹏
				g_SChannel[ucChannelNum].SG01.dFeedrate* dTemp * g_SChannel[ucChannelNum].SRun.dCommonFeedRatio* 60.0 * 1000.0//mm/ms -> mm/min 
				: fabs(g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->dSpindleSpeed) * (60.0 * 1000) / DEGREE_OF_CIRCLE * g_SChannel[ucChannelNum].SG01.dFeedrate* dTemp * g_SChannel[ucChannelNum].SRun.dCommonFeedRatio;// r/min * mm/r
			dRes = isZero(g_SChannel[ucChannelNum].SG01.dG84Ratio) ? dRes : g_SChannel[ucChannelNum].SG01.dFeedrate  * (60.0 * 1000)/ fabs(g_SChannel[ucChannelNum].SG01.dG84Ratio); //对于G84转换Z轴进给速度，其余获取长轴速度
			break;
		case CMD_MOTION_ARC:
		case CMD_MOTION_ANTI_ARC:
			//dRes = g_SChannel[ucChannelNum].SG02.dFeedrate *//进给速度
			//	g_SChannel[ucChannelNum].SRun.dCommonFeedRatio * 60.0 * 1000.0;//乘倍率 转换成mm/min //2024/4/29注释：进给速率未考虑G99

			dRes = getBits(g_SChannel[ucChannelNum].SG02.unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN ?//2024/4/29 洪冠鹏
				g_SChannel[ucChannelNum].SG02.dFeedrate * g_SChannel[ucChannelNum].SRun.dCommonFeedRatio * 60.0 * 1000.0//mm/ms -> mm/min 
				: fabs(g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->dSpindleSpeed) * (60.0 * 1000) / DEGREE_OF_CIRCLE * g_SChannel[ucChannelNum].SG02.dFeedrate * g_SChannel[ucChannelNum].SRun.dCommonFeedRatio;// r/min * mm/r
			break;
		case CMD_MOTION_SCREWCUT:	//陈敏聪	2023/8/19
			dRes = g_SChannel[ucChannelNum].SG32.dF * g_SChannel[ucChannelNum].SSpindle.dSpindleSpeed * 60.0 * 1000.0 / DEGREE_OF_CIRCLE; //转换成mm/min
			break;	
		case CMD_MOTION_G33:
			dRes = g_SChannel[ucChannelNum].SG33.dF * g_SChannel[ucChannelNum].SSpindle.dSpindleSpeed * 60.0 * 1000.0 / DEGREE_OF_CIRCLE; //转换成mm/min
			break;		

			break;

		}
	}
	return dRes;
}

/*************************************************
  函数名称：getCurrentRealFeedRate(uint8 ucChannelNum, uint8* pAxis)
  函数功能  获取当前真实进给速度
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：pMaxAxis 传出G0或手动的时候最大速度的轴号
  返 回 值：double   mm/min
  备    注：
****************************************/
double getCurrentRealFeedRate(uint8 ucChannelNum, uint8* pMaxAxis)
{
	double dRes = 0.0;
	int i;
	struct SSingleAxis* pSSingleAxisPointer = NULL;//单轴结构体指针
	double dMaxVal = 0.0; // G0 手动最大的速度
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	pSSingleAxisPointer = g_SChannel[ucChannelNum].SSingleAxisArray;//指向第一个轴
	dMaxVal = pSSingleAxisPointer->dCurrentSpeed;//暂时拿第一个轴的当前速度
	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE)
	{
		if (g_RunMode == HANDWHEEL_MODE)//手轮模式
		{
			uint8 ucAXISNum; //手轮控制的轴号
			ucAXISNum = g_SChannel[ucChannelNum].pSHandleWheelEncoder->AXISNum;
			if (g_SSystemPara.SAxisPara[ucAXISNum].ucAxisType == LINEAR_AXIS)//2025.4.8熊骏
			{
				dRes = g_SChannel[ucChannelNum].pSHandleWheelEncoder->dAxisCurrentSpeed[ucAXISNum] * 60.0 * 1000.0;
			}
		}
		else//非手轮模式
		{
			for (i = 1; i < AXIS_NUM_MAX; i++)
			{
				if (g_SChannel[ucChannelNum].SSingleAxisArray[i].dCurrentSpeed > dMaxVal)
				{//寻找单轴最大的速度
					if (g_SChannel[ucChannelNum].SSingleAxisArray[i].ucAxis
						== g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucAxis)//为主轴的轴
					{
						continue;
					}
					if (g_SChannel[ucChannelNum].SSingleAxisArray[i].ucAxis
						== g_SChannel[ucChannelNum].SSecondSpindle.pSSpindleAxis->ucAxis)//为主轴的轴
					{
						continue;
					}
					dMaxVal = g_SChannel[ucChannelNum].SSingleAxisArray[i].dCurrentSpeed;
					pSSingleAxisPointer = &g_SChannel[ucChannelNum].SSingleAxisArray[i];
				}
			}
			if (pMaxAxis != NULL)//传出轴号
			{
				*pMaxAxis = pSSingleAxisPointer->ucAxis;//轴号
			}
			dRes = dMaxVal * 60.0 * 1000.0;
		}
	}
	else
	{
		switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR))
		{
		case CMD_MOTION_QUICKMOVE:
			for (i = 1; i < AXIS_NUM_MAX; i++)
			{
				if (g_SChannel[ucChannelNum].SSingleAxisArray[i].dCurrentSpeed > dMaxVal)
				{//寻找单轴最大的速度
					if (g_SChannel[ucChannelNum].SSingleAxisArray[i].ucAxis
						== g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucAxis)//为主轴的轴
					{
						continue;
					}
					if (g_SChannel[ucChannelNum].SSingleAxisArray[i].ucAxis
						== g_SChannel[ucChannelNum].SSecondSpindle.pSSpindleAxis->ucAxis)//为主轴的轴
					{
						continue;
					}
					dMaxVal = g_SChannel[ucChannelNum].SSingleAxisArray[i].dCurrentSpeed;
					pSSingleAxisPointer = &g_SChannel[ucChannelNum].SSingleAxisArray[i];
				}
			}
			if (pMaxAxis != NULL)//传出轴号
			{
				*pMaxAxis = pSSingleAxisPointer->ucAxis;//轴号
			}

			dRes = dMaxVal * 60.0 * 1000.0;//返回最大的单轴速度  并转换成mm/min
			break;
		default:
		case CMD_MOTION_LINE:
		    double dTemp;
			double dMaxNum = 0;
			dRes = g_SChannel[ucChannelNum].SG01.dCurrentSpeed //G1当前速度
				* 60.0 * 1000.0; //转换成mm/min
			for (i = 0; i < AXIS_NUM_MAX; i++) //获取长轴分量
			{
				dTemp = fabs(g_SChannel[ucChannelNum].SG01.SStartVector.dArray[i]);
				if (dTemp > dMaxNum)
				{
					dMaxNum = dTemp;
				}
			}
			dRes = isZero(g_SChannel[ucChannelNum].SG01.dG84Ratio) ? dRes * dMaxNum : dRes / fabs(g_SChannel[ucChannelNum].SG01.dG84Ratio); //对于G84转换Z轴进给速度，其余获取长轴速度
			break;
		case CMD_MOTION_ARC:
		case CMD_MOTION_ANTI_ARC:
			dRes = g_SChannel[ucChannelNum].SG02.dCurrentSpeed //G2G3当前速度
				* 60.0 * 1000.0; //转换成mm/min
			break;

		case CMD_MOTION_SCREWCUT:	//陈敏聪	2023/8/19
			dRes = g_SChannel[ucChannelNum].SG32.dCurrentSpeed * 60.0 * 1000.0; //转换成mm/min
			break;	
		case CMD_MOTION_G33:
			dRes = g_SChannel[ucChannelNum].SG33.dCurrentSpeed * 60.0 * 1000.0; //转换成mm/min
			break;
		}
	}
	return dRes;
}

/*************************************************
  函数名称：getCurLeftDist(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取剩余距离
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号   0~AXIS_NUM_MAX-1 使用宏定义AXIS_X  AXIS_Y ...
  输出参数：
  返 回 值：double   mm
  备    注：
****************************************/
double getCurLeftDist(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return 0.0;
	}
	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE)
	{
		return 0.0;
	}
	uint8 ucLongAxis;
	uint8 ucShortAxis;
	switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR))
	{
	case CMD_MOTION_QUICKMOVE:
		return g_SChannel[ucChannelNum].SSingleAxisArray[ucAxis].SPlanData.dCurLeftDist;//单轴对应的剩余距离
	case CMD_MOTION_LINE:
		return g_SChannel[ucChannelNum].SG01.SPlanData.dCurLeftDist
			* fabs(g_SChannel[ucChannelNum].SG01.SStartVector.dArray[ucAxis]);//直线剩余乘对应向量
	case CMD_MOTION_ARC:
	case CMD_MOTION_ANTI_ARC:
		//陈敏聪 2023/6/4
		if (ucAxis == g_SChannel[ucChannelNum].SG02.ucAxis[0] || ucAxis == g_SChannel[ucChannelNum].SG02.ucAxis[1])	//横轴和纵轴的剩余距离
		{
			const uint8* ucMapArray;
			ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;
			//这里不需要直径显示的坐标
			if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].bShowByDiameter == TRUE)
				return  fabs(g_SChannel[ucChannelNum].SG02.SEndPoint.dArray[ucAxis] - getMachCoordDouble(ucChannelNum, ucAxis) / 2);//剩余距离返回“终点 - 当前点”
			else
				return  fabs(g_SChannel[ucChannelNum].SG02.SEndPoint.dArray[ucAxis] - getMachCoordDouble(ucChannelNum, ucAxis));//剩余距离返回“终点 - 当前点”

		}
		else if (!isZero(g_SChannel[ucChannelNum].SG02.SStartVector.dArray[ucAxis]))	//带有第三轴的圆弧指令的第三轴
		{
			return  g_SChannel[ucChannelNum].SG02.SEndPoint.dArray[ucAxis] - getMachCoordDouble(ucChannelNum, ucAxis);//剩余距离返回“终点 - 当前点”
		}
		else
		{
			return 0.0;	//非横轴纵轴返回0
		}
	case CMD_MOTION_SCREWCUT:	//陈敏聪	2023/7/27
		ucLongAxis = g_SChannel[ucChannelNum].SG32.ucAxis[0];	//长轴
		ucShortAxis = g_SChannel[ucChannelNum].SG32.ucAxis[1];	//短轴
		if (ucLongAxis == ucAxis)
		{
			//修改2025.11.27 增加退尾阶段，长轴剩余距离为当前未移动距离
			if (g_SChannel[ucChannelNum].SG32.ucG32Step == G32_INIT_AND_ASSIGNMENT || g_SChannel[ucChannelNum].SG32.ucG32Step == G32_BACKTAIL || g_SChannel[ucChannelNum].SG32.ucG32Step == G32_OCCUPATION) // 长轴退尾阶段或初始化分配阶段，轴占用阶段
			{
				return g_SChannel[ucChannelNum].SG32.SPlanData.dCurLeftDist; // 长轴剩余距离
			}
			else
			{
				return g_SChannel[ucChannelNum].SG32.SPlanData.dDistance - g_SChannel[ucChannelNum].SG32.SPlanData.dPreDist; // 计算长轴剩余距离
			}

		}
		else if (ucShortAxis == ucAxis)
		{
			// 修改2025.11.27 增加退尾阶段，短轴剩余距离为当前长轴未移动距离乘以系数
			if (g_SChannel[ucChannelNum].SG32.ucG32Step == G32_INIT_AND_ASSIGNMENT || g_SChannel[ucChannelNum].SG32.ucG32Step == G32_BACKTAIL || g_SChannel[ucChannelNum].SG32.ucG32Step == G32_OCCUPATION) // 短轴退尾阶段或初始化分配阶段，轴占用阶段
			{
				return g_SChannel[ucChannelNum].SG32.SPlanData.dCurLeftDist *g_SChannel[ucChannelNum].SG32.dRatioXZ; // 计算短轴剩余距离
			}
			else
			{
				return (g_SChannel[ucChannelNum].SG32.SPlanData.dDistance - g_SChannel[ucChannelNum].SG32.SPlanData.dPreDist) * g_SChannel[ucChannelNum].SG32.dRatioXZ; // 计算短轴剩余距离
			}

		}
		else
		{
			return 0.0;	//非横轴纵轴返回0
		}
		break;
	case CMD_MOTION_G33:	//陈敏聪	2023/7/27
		if (ucAxis == g_SChannel[ucChannelNum].SG33.ucAxis[0] || ucAxis == g_SChannel[ucChannelNum].SG33.ucAxis[1])
		{
			return (g_SChannel[ucChannelNum].SG33.SPlanData.dDistance - g_SChannel[ucChannelNum].SG33.SPlanData.dPreDist)	//计算长/短轴剩余距离
				* fabs(g_SChannel[ucChannelNum].SG33.SStartVector.dArray[ucAxis]);
		}
		else
		{
			return 0.0;
		}
		break;
	default:
		return 0;
	}
}
/*************************************************
  函数名称：getAxisAccmulate(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取各轴未规划的脉冲数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis		轴号   0~AXIS_NUM_MAX-1 使用宏定义AXIS_X  AXIS_Y ...
  输出参数：
  返 回 值： -1   通道号错误
			-2   轴号错误
			-3   空指针
  备    注：
****************************************/
int32 getAxisAccmulate(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	if (ucAxis >= AXIS_NUM_MAX )//轴号判断合法性
	{
		return -2;
	}
	if(g_SChannel[ucChannelNum].pSHandleWheelEncoder == NULL)
	{
		return -3;
	}
	return g_SChannel[ucChannelNum].pSHandleWheelEncoder->sAxisAccmulate[ucAxis];
}

/*************************************************
  函数名称：getGCodeStatusAPI(uint8 ucChannelNum,uint8 ucGroupNum)
  函数功能：获取G代码的状态
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucGroupNum		G代码组号 宏定义
  输出参数：
  返 回 值：int   G代码的值
  备    注：
****************************************/
int getGCodeStatusAPI(uint8 ucChannelNum, uint8 ucGroupNum)
{
	int res = 0;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	switch (ucGroupNum)
	{
	case GCODE_GROUP_MOTION://G0 G1 G2...
		switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR))
		{
		case CMD_MOTION_QUICKMOVE:	res = 0; break;
		case CMD_MOTION_LINE:		res = 1; break;
		case CMD_MOTION_ARC:		res = 2; break;
		case CMD_MOTION_ANTI_ARC:	res = 3; break;
		case CMD_MOTION_SCREWCUT:	res = 32; break;
		case CMD_MOTION_SCREWCUT_LOOP:	res = 92; break;
		case CMD_MOTION_G33:		res = 33; break;
		default:
			break;
		}
		break;

	case GCODE_GROUP_PLANE: // G17 G18 G19
		switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_PLANE_ADDR))
		{
		case CMD_PLANE_G17:res = 17; break;
		case CMD_PLANE_G19:res = 19; break;
		case CMD_PLANE_G18:default:res = 18; break;
		}
		break;
	case GCODE_GROUP_TOOLCOMP://刀补G40 G41 G42
		switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_TOOLCOMP_ADDR))
		{
		case CMD_TOOLCOMP_LEFT:res = 41; break;
		case CMD_TOOLCOMP_RIGHT:res = 42; break;
		case CMD_TOOLCOMP_CANCEL:default:res = 40; break;
		}
		break;

	case GCODE_GROUP_WORKPIECE://工件坐标 G54  G55 ...
		res = g_SChannel[ucChannelNum].SRun.cWorkpieceCoordNum;
		break;

	case GCODE_GROUP_SPINDLE_SPEED://主轴模式G96 G97
		res = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_SPINDLE_SPEED_ADDR) == CMD_SPINDLE_SPEED_CV ?
			97 : 96;
		break;

	case GCODE_GROUP_FEED_TYPE://进给模式 G98 G99
		res = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN ?
			98 : 99;
		break;

	default:
		break;
	}
	return res;
}

/*************************************************
  函数名称：getMCodeStatusAPI(uint8 ucChannelNum,uint8 ucGroupNum)
  函数功能：获取M代码组的状态
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucGroupNum		M代码组号
  输出参数：
  返 回 值：int   M代码的值
  备    注：
****************************************/
int getMCodeStatusAPI(uint8 ucChannelNum, uint8 ucGroupNum)
{
	uint8 val = 0;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	if (getOrSetMCodeStatus(ucChannelNum, ucGroupNum, &val, FALSE) == 1)//读取对应M代码组的状态
	{
		return val;
	}
	else
	{
		return 0;
	}
}

/*************************************************
  函数名称：getSingleAxisStatus(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取单轴状态
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis		轴号
  输出参数：
  返 回 值：1		完成
  备    注：
  ****************************************/
int8 getSingleAxisStatus(uint8 ucChannelNum, uint8 ucAxis)
{
	struct SSingleAxis* pSSingleAxis;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	pSSingleAxis = &g_SChannel[ucChannelNum].SSingleAxisArray[ucAxis];
	if (pSSingleAxis->ucInpStatus == INP_FINISHED)
	{
		return 1;
	}
	return 0;
}

/*************************************************
  函数名称：setSingleAxisStop(uint8 ucChannelNum, uint8 ucAxis, uint8 mode, double dDec)
  函数功能  设置单轴减速停  立即停
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis		轴号
			mode		模式   INP_DECELERATE： 减速停    ；INP_STOP： 立即停
			dDec		减速停的减速度   （单位  mm/ms)
  输出参数：
  返 回 值：-1		轴号错误
			-2		模式错误
			1		设置完成
  备    注：
  ****************************************/
int8 setSingleAxisStop(uint8 ucChannelNum, uint8 ucAxis, uint8 mode, double dDec)
{
	struct SSingleAxis* pSSingleAxis;
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	//单轴结构体
	pSSingleAxis = &g_SChannel[ucChannelNum].SSingleAxisArray[ucAxis];

	if (mode != INP_DECELERATE && mode != INP_STOP)//检查模式
	{
		return -2;
	}

	if (pSSingleAxis->ucInpStatus == INP_FINISHED)
	{
		clearSPlanData(&pSSingleAxis->SPlanData);//停下后清除速度规划参数 2024/7/26 洪冠鹏
		return 1;
	}
    /* === **状态优先级保护：INP_STOP 不允许被降级为 INP_DECELERATE** === */
    if (pSSingleAxis->ucInpStatus == INP_STOP && mode == INP_DECELERATE)
    {
        /* **维持INP_STOP：拒绝覆盖** */
        return 1;
    }
	switch (mode)//根据模式赋值
	{
	case INP_DECELERATE:
		if (pSSingleAxis->ucInpStatus == INP_DECELERATE)
		{
			break;
		}
		pSSingleAxis->ucInpStatus = INP_DECELERATE;
		  /*对dDec做基本防御，避免异常值下发 */
	    if (!isfinite(dDec) || dDec <= 0)
	    return -2;
		pSSingleAxis->SPlanData.dDec = dDec;//赋值减速度
		break;
	case INP_STOP:
		if (pSSingleAxis->ucInpStatus == INP_STOP)
		{
			break;
		}
		pSSingleAxis->ucInpStatus = INP_STOP;
	default:
		break;
	}

	return 0;
}





/*************************************************
  函数名称：setSG0SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucAxis, uint8 ucType)
  函数功能  设置G0 单轴的速度参数
  输入参数：dVal			参数值		速度单位为 mm/ms   加速度单位为mm/ms^2
			ucChannelNum	通道号
			ucAxis			轴号		AXIS_X   AXIS_Y ...    AXIS_ALL 为全部轴
			ucType			参数类型	VEL_VS--起始速度	VEL_VMAX--最大速度	VEL_VE--末速度	PLAN_ACC--加速度	PLAN_DEC--减速段
  输出参数：
  返 回 值：-1		通道号错误
			-2		轴号错误
			-3		类型错误
			1		完成
  备    注：
  ****************************************/
int8 setSG0SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucAxis, uint8 ucType)
{
	uint8 ucCounter;
	struct SSingleAxis* pSSingleAxisArray;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号判断合法性
	{
		return -1;
	}
	if (ucAxis >= AXIS_NUM_MAX && ucAxis != AXIS_ALL)//轴号判断合法性
	{
		return -2;
	}
	pSSingleAxisArray = g_SChannel[ucChannelNum].SG00.pSSingleAxisArray;//单轴结构体数组

	if (ucAxis != AXIS_ALL)//指定单轴
	{
		SWITCH_SEPPD_PARA(pSSingleAxisArray[ucAxis], dVal, ucType);//根据类型赋值
		if (ucType == PLAN_DISTANCE)//2024/10/15 熊骏
		{
			getSingleAxisRealTimePoint(&pSSingleAxisArray[ucAxis].dStartPoint, ucAxis, g_SChannel[ucChannelNum].pMapAxisArray);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
			if (g_SChannel[ucChannelNum].bUseVirtualY == TRUE)//开启合成虚拟Y轴功能：将物理轴的坐标从 倾斜坐标系 转换至 直角坐标系
			{
				if (ucAxis == AXIS_X)
				{
					double dY1RealTimeCoord = 0.0;
					getSingleAxisRealTimePoint(&dY1RealTimeCoord, AXIS_Y, g_SChannel[ucChannelNum].pMapAxisArray);//获取Y1轴实时点坐标
					pSSingleAxisArray[ucAxis].dStartPoint -= 1.0 * dY1RealTimeCoord * g_SChannel[ucChannelNum].dSINAngleYY1;
				}
				if (ucAxis == AXIS_Y)
				{
					pSSingleAxisArray[ucAxis].dStartPoint *= g_SChannel[ucChannelNum].dCOSAngleYY1;
				}
			}
#endif
		}
		pSSingleAxisArray[ucAxis].ucInpStatus = INP_ASSIGNMENT_START;//重新速度规划
	}
	else//指定所有轴
	{
		for (ucCounter = 0; ucCounter < AXIS_NUM_MAX; ucCounter++)
		{
			SWITCH_SEPPD_PARA(pSSingleAxisArray[ucCounter], dVal, ucType);//根据类型赋值
			if (ucType == PLAN_DISTANCE)
			{
				getSingleAxisRealTimePoint(&pSSingleAxisArray[ucCounter].dStartPoint, ucCounter, g_SChannel[ucChannelNum].pMapAxisArray);//获取当前坐标
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
				if (g_SChannel[ucChannelNum].bUseVirtualY == TRUE)//开启合成虚拟Y轴功能：将物理轴的坐标从 倾斜坐标系 转换至 直角坐标系
				{
					if (ucAxis == AXIS_X)
					{
						double dY1RealTimeCoord = 0.0;
						getSingleAxisRealTimePoint(&dY1RealTimeCoord, AXIS_Y, g_SChannel[ucChannelNum].pMapAxisArray);//获取Y1轴实时点坐标
						pSSingleAxisArray[ucAxis].dStartPoint -= 1.0 * dY1RealTimeCoord * g_SChannel[ucChannelNum].dSINAngleYY1;
					}
					if (ucAxis == AXIS_Y)
					{
						pSSingleAxisArray[ucAxis].dStartPoint *= g_SChannel[ucChannelNum].dCOSAngleYY1;
					}
				}
#endif
			}
			pSSingleAxisArray[ucCounter].ucInpStatus = INP_ASSIGNMENT_START;//重新速度规划
		}
	}
	return 1;
}


/*************************************************
  函数名称：setSG1SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucType)
  函数功能  设置G1速度参数
  输入参数：dVal			参数值		速度单位为 mm/ms   加速度单位为mm/ms^2
			ucChannelNum	通道号
			ucType			参数类型	VEL_VS--起始速度	VEL_VMAX--最大速度	VEL_VE--末速度	PLAN_ACC--加速度	PLAN_DEC--减速段
  输出参数：
  返 回 值：-1		通道号错误
			-3		类型错误
			1		完成
  备    注：
  ****************************************/
int8 setSG1SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucType)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号判断合法性
	{
		return -1;
	}

	SWITCH_SEPPD_PARA(g_SChannel[ucChannelNum].SG01, dVal, ucType);//根据类型赋值
	if (ucType == PLAN_DISTANCE)
	{
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			getSingleAxisRealTimePoint(&g_SChannel[ucChannelNum].SG01.SStartPoint.dArray[i], i, g_SChannel[ucChannelNum].pMapAxisArray);//获取当前坐标
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
			if (g_SChannel[ucChannelNum].bUseVirtualY == TRUE)//开启合成虚拟Y轴功能：将物理轴的坐标从 倾斜坐标系 转换至 直角坐标系
			{
				if (i == AXIS_X)
				{
					double dY1RealTimeCoord = 0.0;
					getSingleAxisRealTimePoint(&dY1RealTimeCoord, AXIS_Y, g_SChannel[ucChannelNum].pMapAxisArray);//获取Y1轴实时点坐标
					g_SChannel[ucChannelNum].SG01.SStartPoint.dArray[i] -= 1.0 * dY1RealTimeCoord * g_SChannel[ucChannelNum].dSINAngleYY1;
				}
				if (i == AXIS_Y)
				{
					g_SChannel[ucChannelNum].SG01.SStartPoint.dArray[i] *= g_SChannel[ucChannelNum].dCOSAngleYY1;
				}
			}
#endif
		}
	}
	g_SChannel[ucChannelNum].SG01.ucInpStatus = INP_ASSIGNMENT_START;//重新速度规划

	return 1;
}


/*************************************************
  函数名称：setSG2SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucType)
  函数功能  设置G2速度参数
  输入参数：dVal			参数值		速度单位为 mm/ms   加速度单位为mm/ms^2
			ucChannelNum	通道号
			ucType			参数类型	VEL_VS--起始速度	VEL_VMAX--最大速度	VEL_VE--末速度	PLAN_ACC--加速度	PLAN_DEC--减速段
  输出参数：
  返 回 值：-1		通道号错误
			-3		类型错误
			1		完成
  备    注：
  ****************************************/
int8 setSG2SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucType)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号判断合法性
	{
		return -1;
	}

	SWITCH_SEPPD_PARA(g_SChannel[ucChannelNum].SG02, dVal, ucType);//根据类型赋值
	if (ucType == PLAN_DISTANCE)
	{
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			getSingleAxisRealTimePoint(&g_SChannel[ucChannelNum].SG02.SStartPoint.dArray[i], i, g_SChannel[ucChannelNum].pMapAxisArray);//获取当前坐标
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
			if (g_SChannel[ucChannelNum].bUseVirtualY == TRUE)//开启合成虚拟Y轴功能：将物理轴的坐标从 倾斜坐标系 转换至 直角坐标系
			{
				if (i == AXIS_X)
				{
					double dY1RealTimeCoord = 0.0;
					getSingleAxisRealTimePoint(&dY1RealTimeCoord, AXIS_Y, g_SChannel[ucChannelNum].pMapAxisArray);//获取Y1轴实时点坐标
					g_SChannel[ucChannelNum].SG02.SStartPoint.dArray[i] -= 1.0 * dY1RealTimeCoord * g_SChannel[ucChannelNum].dSINAngleYY1;
				}
				if (i == AXIS_Y)
				{
					g_SChannel[ucChannelNum].SG02.SStartPoint.dArray[i] *= g_SChannel[ucChannelNum].dCOSAngleYY1;
				}
			}
#endif
		}
	}
	g_SChannel[ucChannelNum].SG02.ucInpStatus = INP_ASSIGNMENT_START;//重新速度规划

	return 1;
}

/*************************************************
  函数名称：setCurSpeedPara(double dVal, uint8 ucChannelNum, uint8 ucAxis, uint8 ucType)
  函数功能  设置当前指令的速度参数
  输入参数：dVal			参数值		速度单位为 mm/ms   加速度单位为mm/ms^2
			ucChannelNum	通道号
			ucAxis			轴号		AXIS_X   AXIS_Y ...    AXIS_ALL 为全部轴
			ucType			参数类型	VEL_VS--起始速度	VEL_VMAX--最大速度	VEL_VE--末速度	PLAN_ACC--加速度	PLAN_DEC--减速段
  输出参数：
  返 回 值：-1		通道号错误
			-2		轴号错误
			-3		类型错误
			-4		当前运动不能修改速度值
			1		完成
  备    注：
  ****************************************/
int8 setCurSpeedPara(double dVal, uint8 ucChannelNum, uint8 ucAxis, uint8 ucType)
{
	uint8 ucCounter;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号判断合法性
	{
		return -1;
	}
	if (ucAxis >= AXIS_NUM_MAX && ucAxis != AXIS_ALL)//轴号判断合法性
	{
		return -2;
	}

	switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR))//指令类型判断
	{
	case CMD_MOTION_QUICKMOVE:	//快速运动
		return setSG0SpeedPara(dVal, ucChannelNum, ucAxis, ucType);
	case CMD_MOTION_LINE:		//合成运动
		return setSG1SpeedPara(dVal, ucChannelNum, ucType);
	case CMD_MOTION_ARC:		//顺圆
	case CMD_MOTION_ANTI_ARC:	//逆圆
		return setSG2SpeedPara(dVal, ucChannelNum, ucType);
	}
	return -4;
}

/*************************************************
  函数名称：writeSInputAPI(uint8 ucChannelNum, struct SInput* pSUserData, int8 cType)
  函数功能  将SG0 SG1 SG2的SUserData（自定义参数）或SPreProcess（缓存区）的SUserData的数据写入SInput结构体
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
		   cType  类型：-1 正在运动中  0，1，2，3从静止到运动  0:G0 1:G1 2:G2 3:G3
  输出参数： pSUserData  接受自定义参数的结构体
  返 回 值：-1	  通道号错误
			1	  完成
			0     未完成
  备    注：
  ****************************************/
int8 writeSInputAPI(uint8 ucChannelNum, struct SInput* pSUserData, int8 cType)//2025.4.30 熊骏
{
	struct SChannel* pSChannel;
	const uint8* ucMapArray;//映射
	int cUserDataCount = 0;
	pSChannel = &g_SChannel[ucChannelNum];
	ucMapArray = pSChannel->pMapAxisArray;//映射
	if (cType == -1)
	{
		switch (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR))//指令类型判断		
		{
		default:
		case CMD_MOTION_QUICKMOVE:	//快速运动
			for (int i = 0; i < AXIS_NUM_MAX; i++)
			{
				if (getbit(pSUserData->ucValStatus, i))
				{
					*(pSUserData + cUserDataCount) = pSChannel->SPreProcess.SUserData;;
					cUserDataCount++;
					//速度曲线类型，1：梯形，2：3次S型，3：4次S型
					pSUserData->ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[i]].ucAccVelPlanType;
					pSUserData->ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[i]].ucDecVelPlanType;
					//加、减速度
					pSUserData->dAcc = getSingleAxisAcc(i, pSUserData->ucAccVelPlanType, ucMapArray);
					pSUserData->dDec = getSingleAxisDec(i, pSUserData->ucDecVelPlanType, ucMapArray);
					//初速度 末速度  需要前瞻的G0：初末速度来源缓冲区；不需前瞻的G0：初末速度来源系统参数
					pSUserData->dVs = pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL ? g_SSystemPara.SAxisPara[ucMapArray[i]].dVs : pSChannel->SPreProcess.SUserData.dMinVs;
					pSUserData->dPrimalVe = pSChannel->SPreProcess.SUserData.ucAxisNumOfNeedLookAhead == AXIS_NULL ? g_SSystemPara.SAxisPara[ucMapArray[i]].dVe : pSChannel->SPreProcess.SUserData.dPrimalVe;
					pSUserData->dVs = pSChannel->SG00.pSSingleAxisArray[i].dCurrentSpeed;
					//pSUserData->dFeedRatio = 1.0;
				}
			}
			break;
		case CMD_MOTION_LINE:
			*pSUserData = pSChannel->SPreProcess.SUserData;
			pSUserData->dVs = pSChannel->SG01.dCurrentSpeed;
			//pSUserData->dFeedRatio = 1.0;
			break;
		case CMD_MOTION_ARC:		//顺圆
		case CMD_MOTION_ANTI_ARC:	//逆圆
			*pSUserData = pSChannel->SPreProcess.SUserData;
			pSUserData->dVs = pSChannel->SG02.dCurrentSpeed;
			//pSUserData->dFeedRatio = 1.0;
			break;
		}
	}
	else
	{
		switch (cType)
		{
		case 0:
			for (int i = 0; i < AXIS_NUM_MAX; i++)
			{
				if (getbit(pSUserData->ucValStatus, i))
				{
					pSChannel->SG00.pSSingleAxisArray[i].SUserData.ucValStatus = pSUserData->ucValStatus;
					*(pSUserData + cUserDataCount) = pSChannel->SG00.pSSingleAxisArray[i].SUserData;
					cUserDataCount++;
				}
			}
			break;
		default://默认 G01
		case 1:
			*pSUserData = pSChannel->SG01.SUserData;
			break;
		case 2:
		case 3:
			*pSUserData = pSChannel->SG02.SUserData;
			break;
		}
	}

	return 1;
}

/* 最小校验：保证不会把“必炸参数”送进 adjustDec */
static int8 ValidateAdjustDecInput(const struct SInput* in)
{
    if (in == NULL) return -1;

    /* 关键浮点必须有限 */
    if (!isfinite(in->dVs) || !isfinite(in->dFeedrate) || !isfinite(in->dPrimalVe) ||
        !isfinite(in->dAcc) || !isfinite(in->dDec) || !isfinite(in->dDistance))
    {
        return -1;
    }

    /* 关键物理量必须为正（至少大于EPS） */
    if (in->dAcc <= EPSINON || in->dDec <= EPSINON)
        return -1;

    /* 距离必须有效：距离太小会触发 “/dDistance” 放大与不稳定 */

    /* 速度合理性（按你算法习惯：速度通常应>=0） */
    if (in->dVs < -EPSINON || in->dFeedrate < -EPSINON || in->dPrimalVe < -EPSINON)
        return -1;

	return 1;
}

/*************************************************
  函数名称：applyMotionParaAPI(uint8 ucChannelNum, struct SInput* pSInputPointer, int8 cType)
  函数功能  运动接口函数
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			pInterpPrePointer   传入运动参数的结构体
			cType				运动类型  //0 快速运动  1直线  2顺圆 3逆圆
  输出参数：
  返 回 值：-2		空指针
			-1		结构体状态错误
			 0		圆弧参数错误
			 1		成功
  备    注：
  ****************************************/
int8 applyMotionParaAPI(uint8 ucChannelNum, struct SInput* pSInputPointer, int8 cType)  //0 快速运动  1直线  2顺圆 3逆圆//2025.4.30 熊骏
{
	struct SChannel* pSchannel;
	uint32 unCommandType;
	uint8 ucChannelStatus; //保存进入手动模式时的系统状态  情况一：空闲时切换进手动模式(SYS_IDLE) ； 情况二：自动执行切换进手动模式 2023/11/1 洪冠鹏
	SPointDouble SRelaPoint;
	int8 cUserDataCount = 0;//有效轴个数
	int i = 0;
	int8 cRes = 0;
	if (pSInputPointer == NULL)
	{
		return -2;
	}
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	pSchannel = &g_SChannel[ucChannelNum];
	unCommandType = pSchannel->SRun.unCommandType;
	ucChannelStatus = pSchannel->ucChannelStatus;
	if (pSchannel->ucChannelStatus == SYS_IDLE)//如果处于空闲状态 启动   
	{
		pSchannel->ucChannelStatus = SYS_RUN;
	}

	if(pSchannel->SSpindle.pSSpindleAxis == NULL)  //没有主轴映射
	{
		if(getbit(pSInputPointer->ucValStatus,AXIS_C)==1)//开通C轴
		{
			clrbit(pSInputPointer->ucValStatus,AXIS_C);//清0C轴
			if(pSInputPointer->ucValStatus == 0)//没有其他轴要开通
			{
				return -1;//退出
			}

		}
	}
	if (g_RunMode != AUTOMATIC_MODE)
	{

#if(TURRET_AXIS_TEST == 1)
		//释放所有通道占用的所有轴和刀塔
		int8 temp = 0;
		for (temp; temp < CHANNEL_NUM_MAX; temp++)
		{
			releaseAllOccupiedAxis(&g_SChannel[temp].SPreProcess);//释放该通道所有占用的轴（刀塔轴+非刀塔轴）;
			releaseAllOccupiedTurret(&g_SChannel[temp].SPreProcess);//释放该通道所有占用的刀塔
		}
#else

		for (int i = 0; i < CHANNEL_NUM_MAX; i++)
		{
			releaseLastCommandAxis(&g_SChannel[i].SPreProcess);//2023/12/15 洪冠鹏 切换到手动运动时应把轴都释放（除刀补轴），防止其他通道占用轴而无法手动
		}
#endif



		getRealTimePoint((double*)&pSInputPointer->SStartPoint, AXIS_NUM_MAX, pSchannel->pMapAxisArray, &pSInputPointer->ucValStatus);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机 
		if (g_SSystemPara.bUseVirtualY == TRUE && g_RunMode != RE0_MODE)//未开启虚拟Y轴及回原点模式下，以实际机床坐标系下的实时点作为起点
		{
			//起点为倾斜坐标系（X1-Y1-Z1）物理轴的实时点，终点为直角坐标系(X-Y-Z)缓冲区的值，因此要将起点换算至直角坐标系。
			if (VAL_VALID == getbit(pSInputPointer->ucValStatus, AXIS_X))
			{
				double dY1Mach;
				getSingleAxisRealTimePoint(&dY1Mach, AXIS_Y, pSchannel->pMapAxisArray);
				pSInputPointer->SStartPoint.dArray[AXIS_X] -= 1.0 * dY1Mach * pSchannel->dSINAngleYY1;
			}
			if (VAL_VALID == getbit(pSInputPointer->ucValStatus, AXIS_Y))
			{
				pSInputPointer->SStartPoint.dArray[AXIS_Y] *= pSchannel->dCOSAngleYY1;
			}
		}
#endif
		//subtractPoint((double*)&pSInputPointer->SEndPoint, (double*)&pSInputPointer->SStartPoint, (double*)&SRelaPoint, AXIS_NUM_MAX, &pSInputPointer->ucValStatus);
		//生成ValStatus
		//for (i = 0; i < AXIS_NUM_MAX; i++)
		//{
		//	if (!isZero(SRelaPoint.dArray[i]))
		//	{
		//		setbit(pSInputPointer->ucValStatus, i);
		//	}
		//}
		if (pSchannel->pMapAxisArray[pSchannel->SSpindle.pSSpindleAxis->ucAxis] != AXIS_NULL)//2025/2/28 主轴映射不为空时才使能
		{
			IOStructToBuff(14, &g_SChannel[ucChannelNum].SIOCommand, NULL);//主轴使能  没有阻塞
		}

		if (cType == 2 || cType == 3)
		{
			uint8 ucAxis[2] = { 0 };
			chooseInterpPlane(ucAxis, unCommandType);//选择插补平面
			//根据插补平面的轴  初始化刀补的轴占用
			setbit(pSInputPointer->ucValStatus, ucAxis[0]);
			setbit(pSInputPointer->ucValStatus, ucAxis[1]);

			//if (g_SSystemPara.ucInterpolationMode != INTERPOLATION_LONGAXIS_MODE )
			//{
			//	if (isZero(pSInputPointer->dArcMaxChordErrorVmax))
			//	{
			//		pSInputPointer->dArcMaxChordErrorVmax = g_SSystemPara.dSysMaxVel;
			//	}
			//	if (isZero(pSInputPointer->dArcMaxNormalAccVmax))
			//	{
			//		pSInputPointer->dArcMaxNormalAccVmax = g_SSystemPara.dSysMaxVel;
			//	}
			//}
		}

		////防止自适应插补下 最大速度为0导致不运动
		//if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE && isZero(pSInputPointer->dCommonVMaxFromSingleAxis))
		//{
		//	pSInputPointer->dCommonVMaxFromSingleAxis = g_SSystemPara.dSysMaxVel;
		//}



		//if (isZero(pSInputPointer->dAcc))//防止加速度为0导致不运动
		//{
		//	pSInputPointer->dAcc = (cType == 0) ? 0 : getSysAcc(pSInputPointer->ucAccVelPlanType);
		//}
		//if (isZero(pSInputPointer->dDec))
		//{
		//	pSInputPointer->dDec = (cType == 0) ? 0 : getSysDec(pSInputPointer->ucDecVelPlanType);
		//}

		//固定参数
		//g_SPreProcess.ucRunStep = 0;//运行步骤
		//g_SPreProcess.SUserData.dTc = g_SSystemPara.dInterpolationTime;//插补周期
		pSchannel->SPreProcess.ucInpStatus = INP_BLOCKED; //插补结构体状态
	}
	//根据距离调整减速度
	switch (cType)//2024.12.9熊骏
	{
	case 0:
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			if (getbit(pSInputPointer->ucValStatus, i))
			{
			 /* **按当前cUserDataCount取本轴SInput做校验** */
				cRes = ValidateAdjustDecInput(pSInputPointer + cUserDataCount);
				if (cRes < 0)
				{
					parseRunningError(9999,GCODE_ERROR_NUM153,9999);
					return cRes;
				} 
				cRes = adjustDec(ucChannelNum, pSInputPointer, cType, i, cUserDataCount);
				if (cRes == 3)
				{
					parseRunningError(9999,GCODE_ERROR_NUM153,9999);
					return cRes;
				} 
			}
		}
		cUserDataCount = 0;
		break;
	default://默认G01
	case 1:
	case 2:
	case 3:
	    cRes = ValidateAdjustDecInput(pSInputPointer);
		if (cRes < 0)
		{
			parseRunningError(9999,GCODE_ERROR_NUM153,9999);
			return cRes;
		} 
		cRes = adjustDec(ucChannelNum, pSInputPointer, cType, NULL, 0);
		if (cRes == 3)
		{
			parseRunningError(9999,GCODE_ERROR_NUM153,9999);
			return cRes;
		} 
		break;
	}
	switch (cType) //根据类型赋予指令类型
	{
	case 0://快速运动
		setBits(unCommandType, CMD_MOTION_QUICKMOVE, CMD_MOTION_ADDR);
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			if (getbit(pSInputPointer->ucValStatus, i))
			{
				pSchannel->SG00.pSSingleAxisArray[i].SUserData = *(pSInputPointer + cUserDataCount);
				cUserDataCount++;
			}
		}
		pSchannel->SG00.SUserData.ucValStatus = pSInputPointer->ucValStatus;
		pSchannel->SG00.SUserData.SEndPoint = pSInputPointer->SEndPoint;
		pSchannel->SG00.SUserData.cInterpDir = pSInputPointer->cInterpDir;
		pSchannel->SG00.SUserData.cDealType = DEALTYPE_USER_INPUT;
		pSchannel->SG00.bIsUserData = TRUE;//2022.12.6  使用用户数据标志位置1
		pSchannel->SG00.ucRunStep = 0;
		break;
	default://默认 G01
	case 1://直线插补
		setBits(unCommandType, CMD_MOTION_LINE, CMD_MOTION_ADDR);
		pSchannel->SG01.unCommandType = unCommandType;
		pSchannel->SG01.SUserData = *pSInputPointer;
		pSchannel->SG01.bIsUserData = TRUE;   //2022.12.6  使用用户数据标志位置1
		pSchannel->SG01.ucInpStatus = INP_OCCUPATION;
		break;
	case 2://顺圆
		setBits(unCommandType, CMD_MOTION_ARC, CMD_MOTION_ADDR);
		pSchannel->SG02.unCommandType = unCommandType;
		pSInputPointer->cArcDir = ARC_CW;
		pSchannel->SG02.SUserData = *pSInputPointer;
		pSchannel->SG02.bIsUserData = TRUE;   //2022.12.6  使用用户数据标志位置1
		pSchannel->SG02.ucInpStatus = INP_OCCUPATION;
		break;
	case 3://逆圆
		setBits(unCommandType, CMD_MOTION_ANTI_ARC, CMD_MOTION_ADDR);
		pSchannel->SG02.unCommandType = unCommandType;
		pSInputPointer->cArcDir = ARC_ANTI_CW;
		pSchannel->SG02.SUserData = *pSInputPointer;
		pSchannel->SG02.bIsUserData = TRUE;   //2022.12.6  使用用户数据标志位置1
		pSchannel->SG02.ucInpStatus = INP_OCCUPATION;
		break;
	}
	//继承指令类型
	//pSchannel->SRun.unCommandType = pSchannel->SPreProcess.unCommandType = unCommandType;
	pSchannel->SRun.unCommandType = unCommandType;
	if (g_RunMode != AUTOMATIC_MODE && (ucChannelStatus == SYS_IDLE || ucChannelStatus == SYS_RUN))	//陈敏聪2023/11/8  //洪冠鹏 2024/6/5 增加条件：非自动执行模式下设置手动停止，开放函数供企业使用（摆臂机）
	{
		setBits(pSchannel->SPreProcess.unCommandType, CMD_PROGRAM_FINISH, CMD_PROGRAM_ADDR);//设置程序停止
	}
	return 1;
}

/*************************************************
  函数名称：adjustDec(uint8 ucChannelNum, struct SInput* pSInputPointer, int8 cType, uint8 ucAxis)
  函数功能  调整SInput的减速度
  输入参数：	ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			pInterpPrePointer   传入运动参数的结构体
			cType				运动类型  //0 快速运动  1直线  2顺圆 3逆圆
			ucAxis				轴号  //用于G0，即cType=0，其他情况无作用，可置为NULL
  输出参数：
  返 回 值：
			 1		未修改减速度
			 2		修改了减速度
			 3      参数错误
  备    注：
  ****************************************/
int8 adjustDec(uint8 ucChannelNum, struct SInput* pSInputPointer, int8 cType, uint8 ucAxis, int8 cUserDataCount)//2025.4.30 熊骏
{
	struct SChannel* pSchannel;
	struct SG2* pSG2Pointer;
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance;
	double dAccFactor = 0, dDecFactor = 0; //加速公式系数  减速公式系数  （根据曲线类型取值）
	uint8 ucHorizontalAxis;// 横轴
	uint8 ucVerticalAxis; //纵轴
	double dMaxDistance = 0;	//最大位移量
	double distance = 0;		//距离临时变量
	double dRad;//弧度
	struct SInput* pSInput;
	int8 cRes;
	pSchannel = &g_SChannel[ucChannelNum];
	pSInput = pSInputPointer + cUserDataCount;

	dVs = pSInput->dVs;		//起始速度
	dVmax = pSInput->dFeedrate;		//最大速度
	dVe = pSInput->dPrimalVe;		//最终速度
	dAcc = pSInput->dAcc;		//加速度
	dDec = pSInput->dDec;		//减速度
	dDistance = pSInput->dDistance; //总距离

    if (!isfinite(dAcc) || !isfinite(dDec) || !isfinite(dDistance) ||
        !isfinite(dVs)  || !isfinite(dVmax) || !isfinite(dVe))
    {
        return 3;
    }

    /* dDistance 极小：要么直接报错，要么认为无需重算 */
  

    /* dAcc/dDec 必须大于EPS，否则拒绝下发 */
    if (dAcc <= EPSINON || dDec <= EPSINON)
    {
        return 3;
    }
	getRealTimePoint((double*)&pSInputPointer->SStartPoint, AXIS_NUM_MAX, pSchannel->pMapAxisArray, &pSInputPointer->ucValStatus);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
	if (g_SSystemPara.bUseVirtualY == TRUE && g_RunMode != RE0_MODE)//开启合成虚拟Y轴功能：将物理轴的坐标从 倾斜坐标系 转换至 直角坐标系
	{
		//起点为倾斜坐标系（X1-Y1-Z1）物理轴的实时点，终点为直角坐标系(X-Y-Z)缓冲区的值，因此要将起点换算至直角坐标系。
		if (VAL_VALID == getbit(pSInputPointer->ucValStatus, AXIS_X))
		{
			double dY1Mach;
			getSingleAxisRealTimePoint(&dY1Mach, AXIS_Y, pSchannel->pMapAxisArray);
			pSInputPointer->SStartPoint.dArray[AXIS_X] -= 1.0 * dY1Mach * pSchannel->dSINAngleYY1;
		}
		if (VAL_VALID == getbit(pSInputPointer->ucValStatus, AXIS_Y))
		{
			pSInputPointer->SStartPoint.dArray[AXIS_Y] *= pSchannel->dCOSAngleYY1;
		}
	}
#endif

	dAccFactor = getFactor(pSInput->ucAccVelPlanType);//根据加速曲线类型获取系数
	dDecFactor = getFactor(pSInput->ucDecVelPlanType);//根据减速曲线类型获取系数

	switch (cType)
	{
	case 0:
		dDistance = fabs(pSInputPointer->SEndPoint.dArray[ucAxis] - pSInputPointer->SStartPoint.dArray[ucAxis]);
		break;
	default:
	case 1:
		if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE)//长轴模式下
		{
			//计算长轴和距离分量
			calVectorAndDistance((double*)&pSInputPointer->SStartPoint, (double*)&pSInputPointer->SEndPoint, &dDistance, (double*)&pSInputPointer->SStartVector,
				AXIS_NUM_MAX, pSInputPointer->ucValStatus);
		}
		else
		{
			//计算距离和向量
			calcG1VectorAndDistance(pSInputPointer->SStartPoint, pSInputPointer->SEndPoint, &pSInputPointer->SStartVector, &pSInputPointer->SStartVector, &dDistance,
				AXIS_NUM_MAX, pSchannel->pMapAxisArray, &pSInputPointer->ucValStatus, pSInputPointer->ucRelaVal);
		}
		break;
	case 2:
	case 3:
		pSG2Pointer = &g_SChannel[ucChannelNum].SG02;
		ucHorizontalAxis = pSG2Pointer->ucAxis[0];// 横轴
		ucVerticalAxis = pSG2Pointer->ucAxis[1]; //纵轴

		if (pSG2Pointer == NULL)//空指针
		{
			return -1;
		}
		//计算圆心和弧度
		cRes = radCalculateR((double*)&pSInputPointer->SStartPoint, (double*)&pSInputPointer->SEndPoint, pSInputPointer->dRadius, (double*)&pSInputPointer->SCenterPoint,pSInputPointer->cArcDir, &dRad, pSG2Pointer->ucAxis[0], pSG2Pointer->ucAxis[1]);
		if (cRes != 1)//圆弧计算失败
		{
			return 3;
		}
		//计算长轴和位移分量（圆弧）
		//计算圆弧距离
		distance = fabs(dRad * pSInputPointer->dRadius);
		dMaxDistance = max(dMaxDistance, distance);
		if (isZero(dMaxDistance))
		{
			parseRunningError(9999,GCODE_ERROR_NUM132,9999);
			return 3;
		}
		//寻找最长轴  （绝对值）
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			if (i == ucHorizontalAxis || i == ucVerticalAxis)//跳过圆弧的横轴 或纵轴
			{
				continue;
			}
			//计算距离 fabs(终点-起点)
			if (dDistance <= EPSINON) return 3;
			distance = fabs(pSInputPointer->SEndPoint.dArray[i] - pSInputPointer->SStartPoint.dArray[i]);
			dMaxDistance = max(dMaxDistance, distance);
		}

		//计算圆弧距离分量
		//圆弧的距离分量存储在横轴的距离分量
		pSInputPointer->SStartVector.dArray[ucHorizontalAxis] = dRad * pSInputPointer->dRadius / dMaxDistance;
		//纵轴的距离分量为0
		pSInputPointer->SStartVector.dArray[ucVerticalAxis] = 0;

		//计算距离分量
		for (int i = 0; i < AXIS_NUM_MAX; i++)
		{
			if (i == ucHorizontalAxis || i == ucVerticalAxis)//跳过圆弧的横轴 或纵轴
			{
				continue;
			}
			//其他直线计算
			distance = pSInputPointer->SEndPoint.dArray[i] - pSInputPointer->SStartPoint.dArray[i];
			//距离分量 = 当前轴位移 / 最长轴位移
			pSInputPointer->SStartVector.dArray[i] = distance / dMaxDistance;
		}

		//最长轴用于速度规划
		dDistance = pSInputPointer->dTargetDist = dMaxDistance;
		break;
	}
	if (dVe > dVmax)
	{
		dVe = dVmax;//限制末速度
	}
	if (dVmax >= dVs)//最大速度大于Vs,此时计算极限减速距离时，只需考虑从Vs到Ve以减速度减速（最大速度调整至等于Vs）
	{
		if (dDec <= EPSINON) return 3;
		if (dDistance < dDecFactor * (dVs * dVs - dVe * dVe) / dDec)//总距离比从一开始就以Vs减速到Ve的距离还小
		{
			dDec = dDecFactor * (dVs * dVs - dVe * dVe) / dDistance;//以距离反算减速度
		}
		else
		{
		}
	}
	else//最大速度小于Vs,此时计算极限减速距离时，只需考虑从Vs到Vmax以加速度减速和从Vmax到Ve以减速度减速
	{
		if (dDec <= EPSINON) return 3;
		if (dDistance < dAccFactor * (dVs * dVs - dVmax * dVmax) / dAcc + dDecFactor * (dVmax*dVmax - dVe * dVe) / dDec)
			//总距离不够先从Vs减速至最大速度在减速到Ve
		{
			dDec = dDecFactor * (dVs * dVs - dVe * dVe) / dDistance;//以距离反算减速度
		}
		else
		{
		}
	}
    /* === 回写防线：阻断 NaN/Inf/负数/过大异常值下传 === */
    if (!isfinite(dDec) || dDec <= EPSINON)
    {
        return 3;
    }


    if (!isfinite(dAcc) || dAcc <= EPSINON) return 3;
	pSInput->dVs = dVs;		//起始速度
	pSInput->dFeedrate = dVmax;		//最大速度
	pSInput->dPrimalVe = dVe;		//最终速度
	pSInput->dAcc = dAcc;		//加速度
	pSInput->dDec = dDec;		//减速度
	pSInput->dDistance = dDistance; //总距离
	return 1;

}


///*************************************************
//  函数名称：applyG32MotionParaAPI(struct _SG32* pSG32)
//  函数功能  运动接口函数
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：
//  返 回 值：0:上一次插补还未结束
//			1：G32插补成功
//			2：输入的指针存在为空
//  备    注：
//  ****************************************/
//int8 applyG32MotionParaAPI(struct _SG32* pSG32)  //没办法检测连续G32
//{
//	if (pSG32 == NULL)
//	{
//		return -2;
//	}
//	if (g_SPreProcess.ucInpStatus != INP_FINISHED)
//	{
//		return 0;
//	}
//	if (g_SystemStatus == SYS_IDLE)
//	{
//		g_SystemStatus = SYS_RUN;
//	}
//
//	g_SG32.SUserData.dVs = pSG32->SUserData.dVs;                     //起始速度
//	g_SG32.SUserData.dVe = pSG32->SUserData.dVe;                     //最终速度
//	g_SG32.SUserData.dVmax = pSG32->SUserData.dVmax;
//	g_SG32.SUserData.dAcc = pSG32->SUserData.dAcc;                   //加速度
//	g_SG32.SUserData.dDec = pSG32->SUserData.dDec;                   //减速度
//	g_SG32.SUserData.SStarCutPoint = getRealTimePoint();             //获取实时点作为切削起点
//	g_SG32.SUserData.SEndPoint = pSG32->SUserData.SEndPoint;         //终点
//	g_SG32.SUserData.dJ = pSG32->SUserData.dJ;                       //短轴退尾量
//	g_SG32.SUserData.dK = pSG32->SUserData.dK;                       //长轴退尾量
//	g_SG32.SUserData.dQ = pSG32->SUserData.dQ;                       //起始角
//	g_SG32.SUserData.dF = pSG32->SUserData.dF;						 //长轴螺纹导程
//
//
//	//固定参数
//	g_SG32.ucG32Step = 0;                                            //运行步骤
//	g_SG32.SPlanData.dTc = g_SSystemPara.dInterpolationTime;         //插补周期
//	g_SPreProcess.ucInpStatus = INP_ASSIGNMENT_START;                //插补结构体状态为开始赋值
//	setBits(g_SPreProcess.unCommandType, CMD_MOTION_SCREWCUT, CMD_MOTION_ADDR);   //指令类型为螺牙切削
//	setBits(g_SPreProcess.unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);     //设置程序停止
//
//	return 1;
//
//
//}
//
///*************************************************
//  函数名称：int8 applyG33ParaAPI(struct SPreProcess* pSPreProcess, struct G33* pSG33)
//  函数功能  运动接口函数
//  输入参数：pSPreProcess   传入运动参数的结构体
//		   pSG33	           传入G33螺纹参数的结构体
//  输出参数：
//  返 回 值：0:上一次插补还未结束
//			1：G33插补成功
//			-2：输入的指针存在为空
//  备    注：
//  ****************************************/
//int8 applyG33ParaAPI(struct SInterpolationPre* pSPreProcess, struct G33* pSG33)
//{
//	if (pSPreProcess == NULL || pSG33 == NULL)
//	{
//		return -2;
//	}
//	if (g_SPreProcess.ucInpStatus != INP_FINISHED)
//	{
//		return 0;
//	}
//	if (g_SystemStatus == SYS_IDLE)
//	{
//		g_SystemStatus = SYS_RUN;
//	}
//
//	g_SPreProcess.unCommandType = g_SRun.unCommandType;//指令类型  继承SRun
//	g_SPreProcess.unGCodeLineNum = g_SRun.unGCodeLineNum = g_SRun.unGCodeLineNum + 1;
//
//	//起点终点
//	g_SPreProcess.SUserData.SStartPoint = getRealTimePoint();                      //获取实时点作为起点
//	g_SPreProcess.SUserData.SEndPoint = pSPreProcess->SUserData.SEndPoint;         //终点
//
//	//计算长度和起点向量
//	calVectorAndDistance(g_SPreProcess.SUserData.SStartPoint, g_SPreProcess.SUserData.SEndPoint, &g_SPreProcess.SUserData.dDistance, &g_SPreProcess.SUserData.SStartVector);
//
//	//固定参数
//	g_SPreProcess.ucRunStep = 0;//运行步骤
//	g_SPreProcess.ucInpStatus = INP_BLOCKED; //插补结构体状态
//
//	//设置指令类型为攻牙
//	setBits(g_SPreProcess.unCommandType, CMD_MOTION_G33, CMD_MOTION_ADDR);   //指令类型为螺牙切削
//
//	//gG33所需参数
//	gG33.G33StartPoint = g_SPreProcess.SUserData.SStartPoint;
//	gG33.G33EndPoint = g_SPreProcess.SUserData.SEndPoint;
//	gG33.G33StartVector = g_SPreProcess.SUserData.SStartVector;
//	gG33.dF = pSG33->dF;
//	gG33.ucL = pSG33->ucL;
//
//	g_SRun.unCommandType = g_SPreProcess.unCommandType;
//	setBits(g_SPreProcess.unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);//设置程序停止
//
//	return 1;
//}


/*************************************************
  函数名称：returendStartFollowDisAPI()
  函数功能  返回开始随动所需的长度(开始随动后，才能获得)
  输入参数：
  输出参数：
  返 回 值：0:还未获得
			others: 该值为开始随动所需的长度
  备    注：
  ****************************************/
double returendStartFollowDisAPI()
{
	return g_SChannel[0].SG32.dStartFollowDis;
}




/*************************************************
  函数名称：getMotionStatus(uint8 ucChannelNum)
  函数功能  获取运动状态
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：0  正在运动
			1  运动完成
  备    注：
  ****************************************/
int8 getMotionStatus(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号不正确
	{
		return 1;
	}

	if (g_SChannel[ucChannelNum].SPreProcess.ucInpStatus == INP_FINISHED
		&& g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE)//运动完成 且系统状态空闲
	{
		return 1;
	}
	return 0;
}

/*************************************************
  函数名称：enum RUNMODE getRunMode()
  函数功能  返回g_RunMode
  输入参数：
  输出参数：
  返 回 值：EDIT_MODE,            //编辑模式
			AUTOMATIC_MODE,       //自动执行
			MDI_MODE,             //MDI
			HANDWHEEL_MODE,       //手轮
			HAND_MODE             //手动
  备    注：
  ****************************************/
enum RUNMODE getRunMode()
{
	return g_RunMode;
}



/*************************************************
  函数名称：getSpindleStatus(uint8 ucChannelNum)
  函数功能  获取主轴状态
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：-1 反转
			0  主轴停
			1  正转
  备    注：
  ****************************************/
int8 getSpindleStatus(uint8 ucChannelNum)
{
	struct SSingleAxis* pSSingleAxisPointer;
	int8 res = 0;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return 0;
	}
	pSSingleAxisPointer = g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis;
    if(pSSingleAxisPointer == NULL)
	return 0;
	switch (pSSingleAxisPointer->ucRunMode)
	{
	case SPINDLE_G97_MODE:				//G97
	case SPINDLE_G96ANDG0_MODE:			//G96 G0
	case SPINDLE_G96ANDG1_MODE:			//G96 G1+
		if (pSSingleAxisPointer->ucInpStatus == INP_FINISHED)
		{
			return sgn(pSSingleAxisPointer->SPlanData.dPrimalVe);
		}
		else
		{
			return (isZero(pSSingleAxisPointer->dCurrentSpeed)) ? 0 : sgn(nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData));
		}
	case SPINDLE_COMMONSTOP_MODE:		//减速停
	case SPINDLE_PRECISESTOP_MODE:		//准停
		return (isZero(pSSingleAxisPointer->dCurrentSpeed)) ? 0 : pSSingleAxisPointer->cSpindleDir;
	case SPINDLE_SINGLEAXIS_MODE:		//单轴模式
		if (pSSingleAxisPointer->ucInpStatus == INP_FINISHED) //由于G0加入前瞻，执行分度指令后的当前速度为最小速度。单轴模式下如果主轴未执行运动应返回值0。2024/7/15 洪冠鹏
		{
			return 0;
		}
		else
		{
			return (isZero(pSSingleAxisPointer->dCurrentSpeed)) ? 0 : pSSingleAxisPointer->dStartVector;
		}

	case SPINDLE_HANDWHEEL_MODE:		//手轮模式
	default:
		if (pSSingleAxisPointer->ucInpStatus == INP_FINISHED)
		{
			return 0;
		}
		break;
	}
	return 0;
}


/*************************************************
  函数名称：setSpindleModeAPI(uint8 ucChannelNum, int8 cMode)
  函数功能  设置主轴模式 G96 G97,用于开启中断前，与机床主轴正转和反转按钮有关
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			cMode			模式类型  96: 指定G96恒线速度    97:指定G97恒转速
  输出参数：
  返 回 值：-1 参数错误
			1  设置完成
  备    注：
  ****************************************/
int8 setSpindleModeAPI(uint8 ucChannelNum, int8 cMode)
{
	struct _SSpindle* pSSpindle;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL )
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return -1;
	}
	
	pSSpindle = &g_SChannel[ucChannelNum].SSpindle;
	switch (cMode)
	{
	case 96:
		setBits(pSSpindle->pSChannel->SPreProcess.unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G96
		setBits(pSSpindle->pSSpindleAxis->unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G96
		setBits(pSSpindle->pSChannel->SRun.unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G96
		break;
	case 97:
		setBits(pSSpindle->pSChannel->SPreProcess.unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G97
		setBits(pSSpindle->pSSpindleAxis->unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G97
		setBits(pSSpindle->pSChannel->SRun.unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G97
		break;

	default:
		return -1;
		break;
	}

	if (pSSpindle->pSSpindleAxis->ucRunMode != SPINDLE_COMMONSTOP_MODE && pSSpindle->pSSpindleAxis->ucRunMode != SPINDLE_PRECISESTOP_MODE)
	{//不处于停的功能状态
		pSSpindle->pSSpindleAxis->ucInpStatus = INP_INITIATION;//重新规划
	}


	return 1;
}


/*************************************************
  函数名称：setSpindleModeInInterpolation(uint8 ucChannelNum, int8 cMode)
  函数功能  设置主轴模式 G96 G97 (主要用于插补中G96,G97状态的转换)
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			cMode			模式类型  96: 指定G96恒线速度    97:指定G97恒转速
  输出参数：
  返 回 值：-1 参数错误
			1  设置完成
  备    注：
  ****************************************/
int8 setSpindleModeInInterpolation(uint8 ucChannelNum, int8 cMode)//2024 / 10 / 15 熊骏
{
	struct _SSpindle* pSSpindle;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	pSSpindle = &g_SChannel[ucChannelNum].SSpindle;
	switch (cMode)
	{
	case 96:
		setBits(pSSpindle->pSChannel->SPreProcess.unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G96
		//setBits(pSSpindle->pSSpindleAxis->unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G96
		setBits(pSSpindle->pSChannel->SRun.unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G96
		break;
	case 97:
		setBits(pSSpindle->pSChannel->SPreProcess.unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G97
		//setBits(pSSpindle->pSSpindleAxis->unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G97
		setBits(pSSpindle->pSChannel->SRun.unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);//设置主轴模式 G97
		break;

	default:
		return -1;
		break;
	}

	if (pSSpindle->pSSpindleAxis->ucRunMode != SPINDLE_COMMONSTOP_MODE && pSSpindle->pSSpindleAxis->ucRunMode != SPINDLE_PRECISESTOP_MODE)
	{//不处于停的功能状态
		pSSpindle->pSSpindleAxis->ucInpStatus = INP_INITIATION;//重新规划
	}


	return 1;
}






/*************************************************
  函数名称：getSpindleEncoderSpeed(uint8 ucChannelNum)
  函数功能  获取主轴真实转速
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：主轴转速 (r/min) 带方向
  备    注：
  ****************************************/
double getSpindleEncoderSpeed(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return 0.0;
	}
	return g_SChannel[ucChannelNum].SSpindleTest.dSpindleAvgSpeed * 60.0 * 1000.0;
}


/*************************************************
  函数名称：getSpindleSpeedAPI(uint8 ucChannelNum)
  函数功能  获取主轴理论转速
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：主轴转速 (r/min) 带方向
  备    注：
  ****************************************/
double getSpindleSpeedAPI(uint8 ucChannelNum)
{
	double dTargetSpeed = 0;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return 0.0;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL)
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return 0.0;
	}
	if(calTargetSpindleSpeed(&g_SChannel[ucChannelNum].SSpindle, &dTargetSpeed)!= 1)
	{
		return 0.0;
	}
	return dTargetSpeed * 60.0 * 1000.0;
}

/*************************************************
  函数名称：setSpindleDirAPI(uint8 ucChannelNum, int8 cDir)
  函数功能  设置主轴转动方向
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			cDir    ARC_CW  正转 （1）   ARC_ANTI_CW反转 (-1)
  输出参数：
  返 回 值：-3 通道号错误
			-2  参数错误  方向必须为1或-1
			-1  空指针
			0   方向相同 无需设置
			1	完成
  备    注：
**************************************************/
int8 setSpindleDirAPI(uint8 ucChannelNum, int8 cDir)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL )
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return -1;
	}
	return setSpindleDir(&g_SChannel[ucChannelNum].SSpindle, cDir);
}

/*************************************************
  函数名称：spindleBeginAPI(uint8 ucChannelNum)
  函数功能  主轴启动接口函数  ，当主轴在停下过程或停下时，调用此函数可让主轴按照停下前的状态运动
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：-3  通道号错误
			-1  空指针
			0   已在启动过程中
			1	完成
  备    注：
**************************************************/
int8 spindleBeginAPI(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL )
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return -1;
	}
	
	return spindleBegin(&g_SChannel[ucChannelNum].SSpindle);
}

/*************************************************
  函数名称：spindleCommonStopAPI(uint8 ucChannelNum)
  函数功能  主轴停
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：-3  空指针
			-1	空指针
			0   已在停下过程中
			1	完成
  备    注：
**************************************************/
int8 spindleCommonStopAPI(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL )
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return -1;
	}
	return spindleCommonStop(&g_SChannel[ucChannelNum].SSpindle);
}

/*************************************************
  函数名称：spindlePreciseStopAPI(uint8 ucChannelNum)
  函数功能  主轴准停
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：-3  通道号错误
			-2  处于不能准停的状态
			-1  空指针
			0   已在停下过程中
			1	完成
  备    注：
**************************************************/
int8 spindlePreciseStopAPI(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL )
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return -1;
	}
	return spindlePreciseStop(&g_SChannel[ucChannelNum].SSpindle, NULL);
}

/*************************************************
  函数名称：spindleStopNowAPI(uint8 ucChannelNum)
  函数功能  主轴立即停接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：-3 通道号错误
			-1  空指针
			0	错误的模式
			1	完成
  备    注：
**************************************************/
int8 spindleStopNowAPI(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	if (&g_SChannel[ucChannelNum].SSpindle == NULL )
	{
		parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
		return -1;
	}
	return spindleStopNow(&g_SChannel[ucChannelNum].SSpindle);
}


/*************************************************
  函数名称：setSpindleSpeedAPI(uint8 ucChannelNum, double dSpindleSpeed, uint16 type)
  函数功能  设置主轴转速
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			dSpindleSpeed		主轴转速G96(m/min) G97(r/min)
			type				CMD_SPINDLE_SPEED_CLV -> G96(m/min)   ; CMD_SPINDLE_SPEED_CV -> G97(r/min)
  输出参数：
  返 回 值：-3 通道号错误
			-1 空指针
			1  执行成功
  备    注：
**************************************************/
int8 setSpindleSpeedAPI(uint8 ucChannelNum, double dSpindleSpeed, uint16 type)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	return setSpindleSpeed(&g_SChannel[ucChannelNum].SSpindle, dSpindleSpeed, type);
}


/*************************************************
  函数名称：setSpindleSpeedAPI(uint8 ucChannelNum, double dSpindleSpeed, uint16 type)
  函数功能  获得该通道主轴发送脉冲和主轴编码器的偏差
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：-3 通道号错误
			-1 空指针
			1  执行成功
  备    注：
**************************************************/
int16 getSpindlePulseDeviation(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return 32767;
	}
	return g_SChannel[ucChannelNum].lIdealSpindlePulse - g_SChannel[ucChannelNum].pSSpindleEncoder->lEncoderAbsValue;
}


////使能主轴
//int8 setSpindleEnabledAPI(uint8 ucChannelNum,bool isEnable)
//{
//	if (ucChannelNum >= CHANNEL_NUM_MAX)
//	{
//		return -1;
//	}
//	return setSpindleEnabled(&g_SChannel[ucChannelNum].SSpindle, isEnable);
//}

// 使能主轴
int8 setSpindleEnabledAPI(uint8 ucChannelNum, bool isEnable)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	uint8 ucIOCommand = (isEnable == TRUE) ? 14 : 15;   //M14 主轴使能  M15主轴取消使能
	if (findIO(ucIOCommand, g_SChannel[ucChannelNum].SIOCommand.ucIORunBuff, IO_MAXGROUP_IN_SINGLE_LINE) == 0)
	{
		return IOStructToBuff(ucIOCommand, &g_SChannel[ucChannelNum].SIOCommand, NULL);
	}

	return 1;
}



