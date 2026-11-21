#include "ToolChanger.h"

#include "G1.h" 
#include "G0.h"  
#include "Motion.h"	
#include <stdio.h>
#include <string.h>
#include "Config.h"
#include "Channel.h"
#include "common.h"
#include "Gcode.h"

//初始化刀具参数（刀号）
int8 toolInitAfterLoad()
{
	uint8 ucCounter = 0, ucCounter1 = 0;
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		for (ucCounter1 = 1; ucCounter1 <= TOOL_NUM_MAX; ucCounter1++)
		{
			g_SSystemPara.SToolPara[ucCounter][ucCounter1 - 1].ucToolID = ucCounter1;
		}
	}

	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		for (ucCounter1 = 1; ucCounter1 <= TOOL_NUM_MAX; ucCounter1++)
		{
			g_SSystemPara.SWearPara[ucCounter][ucCounter1 - 1].ucToolID = (ucCounter1 - 1) % TOOL_NUM_MAX + 1;
		}
	}

}

///*************************************************
//  函数名称：clearToolOffset( uint8 ucToolID )
//  函数功能：清除指定刀号的刀补偏移
//  输入参数：ucToolID 刀具号
//  输出参数：
//  返 回 值：1 清除完成
//			-1 刀具号超过刀具数量
//			-2 空指针
//  备    注：
//**************************************************/
//int8 clearAllToolOffset(SToolParaInfo* STargetPara, uint8 ucTargetParaSize)
//{
//	SPointDouble SZeroPoint;
//	uint8 ucCounter;
//	if (STargetPara == NULL)
//	{
//		return -2;
//	}
//
//	SZeroPoint.dX = 0;
//	SZeroPoint.dY = 0;
//	SZeroPoint.dZ = 0;
//
//	for (ucCounter = 0; ucCounter < ucTargetParaSize; ucCounter++)
//	{
//		STargetPara[ucCounter].SToolCoord = SZeroPoint;//偏移坐标置0
//		STargetPara[ucCounter].dToolRadius = 0;
//	}
//
//	return 1;
//}
//
//
//
///*************************************************
//  函数名称：clearSingleToolOffset(uint8 ucToolID,SToolParaInfo* STargetPara,  uint8 ucTargetParaSize)
//  函数功能：设置指定刀号的刀补偏移
//  输入参数：ucToolID  刀具号
//			* STargetPara  指定刀具参数的起始指针
//			ucTargetParaSize 参数的最大值
//  输出参数：
//  返 回 值：1 设置完成
//			-1 刀具号超过刀具数量
//			-3 空指针
//  备    注：
//**************************************************/
//int8 clearSingleToolOffset(uint8 ucToolID, SToolParaInfo* STargetPara, uint8 ucTargetParaSize)
//{
//	SPointDouble SZeroPoint;
//	if (STargetPara == NULL)
//	{
//		return -3;
//	}
//
//
//	if (ucToolID > ucTargetParaSize)//刀号大于刀具数量
//	{
//		return -1;
//	}
//	SZeroPoint.dX = 0;
//	SZeroPoint.dY = 0;
//	SZeroPoint.dZ = 0;
//
//	STargetPara[ucToolID - 1].SToolCoord = SZeroPoint;//偏移坐标置0
//	STargetPara[ucToolID - 1].dToolRadius = 0;//半径置0
//	return 1;
//}
//
///*************************************************
//  函数名称：setToolOffset( uint8 ucToolID,uint8 ucAxis ,double dOffsetVal,SPointDouble STargetCoor)
//  函数功能：设置指定刀号的刀补偏移
//  输入参数：ucToolID			刀具号
//			ucAxis				轴号
//			dOffsetVal			输入偏移值
//			*STargetPara		修改刀具（磨损）的起始地址
//			ucTargetParaSize	地址对应数据的容量
//  输出参数：
//  返 回 值：1 设置完成
//			-1 刀具号超过刀具数量
//			-2 轴号错误
//			-3 空指针
//  备    注：
//**************************************************/
//int8 setToolOffset(uint8 ucToolID, uint8 ucAxis, double dOffsetVal, SToolParaInfo* STargetPara, uint8 ucTargetParaSize)
//{
//	double* pdToolCoord;//目标的刀具坐标
//	double* pdRealTimeCoord = &g_SRealTimePoint.dX; //实时点坐标
//	if (STargetPara == NULL)
//	{
//		return -3;
//	}
//
//
//	if (ucToolID > ucTargetParaSize)//刀号大于刀具数量
//	{
//		return -1;
//	}
//	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
//	{
//		return -2;
//	}
//	if (ucAxis == AXIS_X)//X轴将输入除2
//	{
//		dOffsetVal /= 2;
//	}
//
//	pdToolCoord = &STargetPara[ucToolID - 1].SToolCoord.dX;//目标的刀具坐标
//
//	pdToolCoord[ucAxis - 1] = dOffsetVal + pdRealTimeCoord[ucAxis - 1];//输入值+机械坐标
//
//
//	return 1;
//}
//
//
///*************************************************
//  函数名称：inputToolOffset( uint8 ucToolID,uint8 ucAxis ,double dOffsetVal)
//  函数功能：设置指定刀号的刀补偏移
//  输入参数：ucToolID			刀具号
//			ucAxis				轴号
//			dOffsetVal			输入偏移值
//			*STargetPara		修改刀具（磨损）的起始地址
//			ucTargetParaSize	地址对应数据的容量
//			bIsRelaVal			是不是相对值（在原基础上加？)
//  输出参数：
//  返 回 值：1 设置完成
//						-1 刀具号超过刀具数量
//						-2 轴号错误
//  备    注：
//**************************************************/
//int8 inputToolOffset(uint8 ucToolID, uint8 ucAxis, double dOffsetVal, SToolParaInfo* STargetPara, uint8 ucTargetParaSize,bool bIsRelaVal)
//{
//	double* pdToolCoord;
//	if (ucToolID > ucTargetParaSize)//刀号大于刀具数量
//	{
//		return -1;
//	}
//	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
//	{
//		return -2;
//	}
//	if (ucAxis == AXIS_X)//X轴为输入值的1/2
//	{
//		dOffsetVal /= 2;
//	}
//	if (bIsRelaVal)
//	{
//		pdToolCoord = &STargetPara[ucToolID - 1].SToolCoord.dX;//确定目标坐标
//		pdToolCoord[ucAxis - 1] += dOffsetVal;//设置为输入的偏移值
//	}
//	else
//	{
//		pdToolCoord = &STargetPara[ucToolID - 1].SToolCoord.dX;//确定目标坐标
//		pdToolCoord[ucAxis - 1] = dOffsetVal;//设置为输入的偏移值
//	}
//
//
//
//	return 1;
//}



/*************************************************
  函数名称：clearAllToolOffset(uint8 ucChannelNum)
  函数功能：清除所有刀具偏移
  输入参数：
  输出参数：
  返 回 值：1 清除完成
			-1 通道错误
  备    注：
**************************************************/
int8 clearAllToolOffset(uint8 ucChannelNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];
	SPointDouble SZeroPoint = { {0} };//零点坐标
	uint8 ucCounter;

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)//遍历该通道下的刀具参数
	{
		STargetPara[ucCounter].SToolCoord = SZeroPoint;//偏移坐标置0
	}
	updataTotalOffset(ucChannelNum, 0, TRUE);// 更新当前总偏移
	return 1;
}



///*************************************************
//  函数名称：clearSingleToolOffset(uint8 ucToolID)
//  函数功能：清除一行的刀具偏移
//  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
//			ucToolID  刀具号
//  输出参数：
//  返 回 值：1 设置完成
//			-1 通道错误
//			-2 刀具号超过刀具数量
//  备    注：
//**************************************************/
//int8 clearSingleToolOffset(uint8 ucChannelNum, uint8 ucToolID)
//{
//	SToolParaInfo* STargetPara = (SToolParaInfo*)&g_SSystemPara.SToolPara[0];//指向刀具偏移
//	SPointDouble SZeroPoint = { {0} };//零点坐标
//
//	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
//	{
//		return -1;
//	}
//	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
//	{
//		return -2;
//	}
//
//	STargetPara[ucToolID - 1].SToolCoord = SZeroPoint;//偏移坐标置0
//	updataTotalOffset(ucChannelNum, ucToolID, 0, FALSE);// 更新当前总偏移
//	return 1;
//}



/*************************************************
  函数名称：setToolOffset(uint8 ucChannelNum, uint8 ucToolLengthNum, uint8 ucAxis, double dOffsetVal)
  函数功能：设置指定刀号的刀补偏移
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucToolLengthNum		刀长号
			ucAxis				轴号      从0开始   0~5
			dOffsetVal			输入偏移值
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
			-3 轴号错误
  备    注：
**************************************************/
int8 setToolOffset(uint8 ucChannelNum, uint8 ucToolLengthNum, uint8 ucAxis, double dOffsetVal)
{
	double* pdToolCoord;//目标的刀具坐标
	double* pdCurToolCoord;//当前使用的刀具号
	double dRealTimeCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolLengthNum > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	if (ucAxis != AXIS_X && ucAxis != AXIS_Z && ucAxis != AXIS_Y)
	{
		return -3;
	}


	if (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter)//直径显示设为输入值的1/2	//陈敏聪	2023/7/22
	{
		dOffsetVal /= 2;
	}

	getSingleAxisRealTimePointByChannel(ucChannelNum, &dRealTimeCoord, ucAxis); //实时点坐标

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机
	if (g_SSystemPara.bUseVirtualY == TRUE)//开启合成虚拟Y轴功能
	{
		if (ucAxis == AXIS_X)
		{
			double dY1RealTimeCoord = 0.0;
			getSingleAxisRealTimePointByChannel(ucChannelNum, &dY1RealTimeCoord, AXIS_Y); //获取Y1轴实时点坐标
			dRealTimeCoord -= 1.0 * dY1RealTimeCoord * g_SChannel[ucChannelNum].dSINAngleYY1;
		}
		if (ucAxis == AXIS_Y)
		{
			dRealTimeCoord *= g_SChannel[ucChannelNum].dCOSAngleYY1;
		}
	}
#endif

	struct _SRun* pSRun = &g_SChannel[ucChannelNum].SRun;
	pdToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][ucToolLengthNum - 1].SToolCoord.dArray;//光标对应的刀号的刀具坐标
	pdCurToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][pSRun->ucToolCompNum - 1].SToolCoord.dArray;//当前使用的刀号的刀具坐标

	//pdToolCoord[ucAxis] = pdCurToolCoord[ucAxis] + (dRealTimeCoord - pSRun->SCurrentOffset.dArray[ucAxis] - dOffsetVal);//根据绝对坐标的增量来修改刀具坐标（绝对坐标加多少，刀具坐标就减多少）陈育楠2023.10.13
	pdToolCoord[ucAxis] = dRealTimeCoord - dOffsetVal;//陈育楠2024.3.1 
	updataTotalOffset(ucChannelNum, 0, TRUE);// 更新当前总偏移
	return 1;
}


/*************************************************
  函数名称：inputToolOffset(uint8 ucChannelNum, uint8 ucToolLengthNum, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
  函数功能：设置指定刀号的刀补偏移
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucToolLengthNum		刀长
			ucAxis				轴号
			dOffsetVal			输入偏移值
			bIsRelaVal			是不是相对值（在原基础上加？)
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
			-3 轴号错误
  备    注：
**************************************************/
int8 inputToolOffset(uint8 ucChannelNum, uint8 ucToolLengthNum, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
{
	double* pdToolCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolLengthNum > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	if (ucAxis != AXIS_X && ucAxis != AXIS_Z && ucAxis != AXIS_Y)
	{
		return -3;
	}

	if (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter)//直径显示设为输入值的1/2	//陈敏聪	2023/7/22
	{
		dOffsetVal /= 2;
	}

	pdToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][ucToolLengthNum - 1].SToolCoord.dArray;//确定目标坐标

	if (bIsRelaVal)
	{
		pdToolCoord[ucAxis] += dOffsetVal;//设置为输入的偏移值
	}
	else
	{
		pdToolCoord[ucAxis] = dOffsetVal;//设置为输入的偏移值
	}
	updataTotalOffset(ucChannelNum, 0, TRUE);// 更新当前总偏移
	return 1;
}

/*************************************************
  函数名称：getToolOffset(uint8 ucChannelNum, uint8 ucToolLengthNum, uint8 ucAxis)
  函数功能：获取指定刀号的刀补偏移
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucToolLengthNum		刀长号
			ucAxis				轴号
  输出参数：
  返 回 值：指定刀号对应轴的偏移值
  备    注：
**************************************************/
double getToolOffset(uint8 ucChannelNum, uint8 ucToolLengthNum, uint8 ucAxis)
{
	double* pdToolCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return 0.0;
	}
	if (ucToolLengthNum > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return 0.0;
	}
	if (ucAxis != AXIS_X && ucAxis != AXIS_Z && ucAxis != AXIS_Y)
	{
		return 0.0;
	}

	pdToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][ucToolLengthNum - 1].SToolCoord.dArray;//确定目标坐标

	return (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter) ? pdToolCoord[ucAxis] * 2.0 : pdToolCoord[ucAxis];	//返回值	陈敏聪	2023/7/22
}



/*************************************************
  函数名称：clearAllToolOffset()
  函数功能：清除所有刀具ID的半径
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1 清除完成
			-1 通道错误
  备    注：
**************************************************/
int8 clearAllToolRadius(uint8 ucChannelNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];
	uint8 ucCounter;

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)//循环  置刀具半径为0
	{
		STargetPara[ucCounter].dToolRadius = 0;
	}

	return 1;
}

/*************************************************
  函数名称：clearSingleToolOffset(uint8 ucToolID)
  函数功能：清除一行刀具ID的半径
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucToolID		刀具号
  输出参数：
  返 回 值：1 清除完成
			-1 通道错误
			-2 刀具号超过刀具数量
  备    注：
**************************************************/
int8 clearSingleToolRadius(uint8 ucChannelNum, uint8 ucToolID)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}

	STargetPara[ucToolID - 1].dToolRadius = 0;//半径置0
	return 1;
}

/*************************************************
  函数名称：inputToolRadius(uint8 ucToolID, double dToolRadius)
  函数功能：设置指定刀号的刀补半径
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucToolID			刀具号
			dToolRadius			半径值
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
  备    注：
**************************************************/
int8 inputToolRadius(uint8 ucChannelNum, uint8 ucToolID, double dToolRadius)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}

	STargetPara[ucToolID - 1].dToolRadius = dToolRadius;//设置为输入的偏移值

	return 1;
}

/*************************************************
  函数名称：getToolRadius(uint8 ucChannelNum, uint8 ucToolID)
  函数功能：获取指定刀号的半径
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucToolID			刀具号
  输出参数：
  返 回 值：刀具半径
  备    注：
**************************************************/
double getToolRadius(uint8 ucChannelNum, uint8 ucToolID)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1.0;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2.0;
	}

	return STargetPara[ucToolID - 1].dToolRadius;//偏移值
}

/*************************************************
  函数名称：clearAllToolWear(uint8 ucChannelNum)
  函数功能：清除所有刀具ID的磨损值
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1 清除完成
			-1 通道错误
  备    注：
**************************************************/
int8 clearAllToolWear(uint8 ucChannelNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SWearPara[ucChannelNum][0];
	SPointDouble SZeroPoint = { {0} };//零点坐标
	uint8 ucCounter;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX ; ucCounter++)
	{
		STargetPara[ucCounter].SToolCoord = SZeroPoint;//偏移坐标置0
		//STargetPara[ucCounter].dToolRadius = 0;
	}
	updataTotalOffset(ucChannelNum, 0, TRUE);// 更新当前总偏移
	return 1;
}


///*************************************************2023/2/14 cmc
//  函数名称：clearSingleToolWear(uint8 ucToolID,uint8 ucGroupNum)
//  函数功能：清除一行刀具ID的磨损值
//  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
//			ucToolID	刀具号    1~TOOL_NUM_MAX
//			ucGroupNum	磨损组号  1~TOOLWEAR_GROUP_NUM
//  输出参数：
//  返 回 值：1 设置完成
//			-1 通道错误
//			-2 刀具号超过刀具数量
//			-3 组号错误
//  备    注：
//**************************************************/
//int8 clearSingleToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucToolCompNum)
//{
//	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];
//	SPointDouble SZeroPoint = { {0} };
//	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
//	{
//		return -1;
//	}
//	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
//	{
//		return -2;
//	}
//	if (ucToolCompNum > TOOL_NUM_MAX) return -3;
//
//
//	STargetPara[ucToolCompNum - 1].SToolCoord = SZeroPoint;//偏移坐标置0
//	//STargetPara[ucToolID - 1].dToolRadius = 0;//半径置0
//	updataTotalOffset(ucChannelNum, ucToolID, 0, FALSE);// 更新当前总偏移
//	return 1;
//}


///*************************************************2023/2/14 cmc
//  函数名称：inputToolWear(uint8 ucToolID, uint8 ucGroupNum,uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
//  函数功能：设置指定刀号的刀具磨损值
//  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
//			ucToolID			刀具号 1~TOOL_NUM_MAX
//			ucGroupNum			磨损组号 1~TOOLWEAR_GROUP_NUM
//			ucAxis				轴号   从0开始
//			dOffsetVal			输入偏移值
//			bIsRelaVal			是不是相对值（在原基础上加？)
//  输出参数：
//  返 回 值：1 设置完成
//			-1 通道错误
//			-2 刀具号超过刀具数量
//			-3 轴号错误
//			-4 组号错误
//  备    注：
//**************************************************/
//int8 inputToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucToolCompNum, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
//{
//	double* pdToolCoord;
//	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
//	{
//		return -1;
//	}
//	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
//	{
//		return -2;
//	}
//	if (ucToolCompNum > TOOL_NUM_MAX)
//	{
//		return -4;
//	}
//
//	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
//	{
//		return -3;
//	}
//	if (ucAxis == AXIS_X)//X轴为输入值的1/2
//	{
//		dOffsetVal /= 2;
//	}
//
//	pdToolCoord = (double*)&g_SSystemPara.SWearPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray;//确定目标坐标
//
//	if (bIsRelaVal)
//	{
//		pdToolCoord[ucAxis] += dOffsetVal;//设置为输入的偏移值
//	}
//	else
//	{
//		pdToolCoord[ucAxis] = dOffsetVal;//设置为输入的偏移值
//	}
//	updataTotalOffset(ucChannelNum, ucToolID, 0, FALSE);// 更新当前总偏移
//	return 1;
//}

/*************************************************
  函数名称：inputToolWear(uint8 ucGroupNum,uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
  函数功能：设置指定刀号的刀具磨损值
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucWearCompNum		磨损号 1~TOOLWEAR_GROUP_NUM
			ucAxis				轴号   从0开始
			dOffsetVal			输入偏移值
			bIsRelaVal			是不是相对值（在原基础上加？)
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
			-3 轴号错误
			-4 组号错误
  备    注：
**************************************************/
int8 inputToolWear(uint8 ucChannelNum, uint8 ucWearCompNum, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
{
	double* pdToolCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucWearCompNum > TOOL_NUM_MAX || ucWearCompNum < 1)//刀号大于刀具数量
	{
		return -2;
	}

	if (ucAxis != AXIS_X && ucAxis != AXIS_Z && ucAxis != AXIS_Y)//刀具偏移只有X和Z
	{
		return -3;
	}
	//if (ucAxis == AXIS_X)//X轴为输入值的1/2    陈育楠 注释掉 5.27
	//{
	//	dOffsetVal /= 2;
	//}

	if (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter)//直径显示设为输入值的1/2	半径显示设为原来的值  /陈育楠 5.27
	{
		dOffsetVal /= 2;
	}
	pdToolCoord = (double*)&g_SSystemPara.SWearPara[ucChannelNum][ucWearCompNum - 1].SToolCoord.dArray;//确定目标坐标

	if (bIsRelaVal)
	{
		pdToolCoord[ucAxis] += dOffsetVal;//设置为输入的偏移值
	}
	else
	{
		pdToolCoord[ucAxis] = dOffsetVal;//设置为输入的偏移值
	}
	updataTotalOffset(ucChannelNum, 0, TRUE);// 更新当前总偏移 
	return 1;
}


/*************************************************
  函数名称：getToolWear(uint8 ucChannelNum, uint8 ucGroupNum, uint8 ucAxis)
  函数功能：设置指定刀号的刀具磨损值
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			ucWearCompNum		磨损号 从1开始 1~TOOL_NUM_MAX
			ucAxis				轴号   从0开始
  输出参数：
  返 回 值：刀具磨损值
  备    注：
**************************************************/
double getToolWear(uint8 ucChannelNum, uint8 ucWearCompNum, uint8 ucAxis)
{
	double dRes;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return 0.0;
	}

	if (ucWearCompNum > TOOL_NUM_MAX)
	{
		return 0.0;
	}

	if (ucAxis != AXIS_X && ucAxis != AXIS_Z && ucAxis != AXIS_Y)//刀具偏移只有X和Z
	{
		return 0.0;
	}

	dRes = g_SSystemPara.SWearPara[ucChannelNum][ucWearCompNum - 1].SToolCoord.dArray[ucAxis];//确定目标坐标
	//return (ucAxis == AXIS_X) ? dRes * 2.0 : dRes;//   陈育楠 注释掉 5.27
	return (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter) ? dRes * 2.0 : dRes;//直径显示设为输入值的1/2	半径显示设为原来的值  /陈育楠 5.27
}


//清除所有刀具ID的假象刀尖号
int8 clearAllToolNoseNum(uint8 ucChannelNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	uint8 ucCounter;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)
	{
		STargetPara[ucCounter].ucToolNoseID = 0;
	}

	return 1;
}


//清除一行刀具ID的假象刀尖号
int8 clearSingleToolNoseNum(uint8 ucChannelNum, uint8 ucToolID)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//假象刀尖号大于假象刀尖总数量
	{
		return -2;
	}

	STargetPara[ucToolID - 1].ucToolNoseID = 0;//假象刀尖号设为0
	return 1;
}


//输入假象刀尖号
int8 inputToolNoseNum(uint8 ucChannelNum, uint8 ucToolID, uint8 ucToolNoseNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}

	if (ucToolNoseNum > TOOLNOSE_NUM_MAX)//假象刀尖号大于总数量
	{
		return -3;
	}

	STargetPara[ucToolID - 1].ucToolNoseID = ucToolNoseNum;//设置为输入假象刀尖号

	return 1;
}


//获取刀具假象刀尖号
int8 getToolNoseNum(uint8 ucChannelNum, uint8 ucToolID)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	return STargetPara[ucToolID - 1].ucToolNoseID;//偏移值
}


//清除所有刀具ID的刀尖磨损
int8 clearAllToolNoseWear(uint8 ucChannelNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	uint8 ucCounter;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)
	{
		STargetPara[ucCounter].dToolNoseWear = 0;
	}

	return 1;
}

//清除一行刀具ID的刀尖磨损
int8 clearSingleToolNoseWear(uint8 ucChannelNum, uint8 ucToolID)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//假象刀尖号大于假象刀尖总数量
	{
		return -2;
	}

	STargetPara[ucToolID - 1].dToolNoseWear = 0;//假象刀尖号设为0
	return 1;
}

//输入刀尖磨损
int8 inputToolNoseWear(uint8 ucChannelNum, uint8 ucToolID, double dToolNoseWear)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}

	STargetPara[ucToolID - 1].dToolNoseWear = dToolNoseWear;//设置为输入刀尖磨损

	return 1;
}

//获取刀具假象刀尖磨损
double getToolNoseWear(uint8 ucChannelNum, uint8 ucToolID)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	return STargetPara[ucToolID - 1].dToolNoseWear;//返回刀尖磨损值
}


///*************************************************2023/2/14 cmc
//  函数名称：getToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum, uint8 ucAxis)
//  函数功能：更新当前偏移
//  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
//			ucToolID			刀具号 从1开始 1~TOOL_NUM_MAX
//			cWorkpieceCoordNum	工件坐标系 54 ~ 59
//			bForceUpdate		是否强制更新总偏移
//  输出参数：
//  返 回 值：
//  备    注：
//**************************************************/
//int8 updataTotalOffset(uint8 ucChannelNum, uint8 ucToolNum,int8 cWorkpieceCoordNum,bool bForceUpdate)
//{
//
//	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
//	{
//		return -1;
//	}
//
//	if (bForceUpdate ||g_SChannel[ucChannelNum].SRun.ucToolNum == ucToolNum || g_SChannel[ucChannelNum].SRun.cWorkpieceCoordNum ==cWorkpieceCoordNum)//刀具号 或坐标偏移坐标系相同  更新
//	{
//		struct _SRun* pSRun = &g_SChannel[ucChannelNum].SRun;
//		TChangeTotalOffset(ucChannelNum, pSRun->ucToolNum, pSRun->ucToolCompNum,pSRun->cWorkpieceCoordNum, &pSRun->SCurrentOffset);
//		return 1;
//	}
//	return 0;
//}

/*************************************************
  函数名称：getToolWear(uint8 ucChannelNum,uint8 ucGroupNum, uint8 ucAxis)
  函数功能：更新当前偏移
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
			cWorkpieceCoordNum	工件坐标系 54 ~ 59
			bForceUpdate		是否强制更新总偏移
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 updataTotalOffset(uint8 ucChannelNum, int8 cWorkpieceCoordNum, bool bForceUpdate)
{

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	if (bForceUpdate || g_SChannel[ucChannelNum].SRun.cWorkpieceCoordNum == cWorkpieceCoordNum)//刀具号 或坐标偏移坐标系相同  更新
	{
		struct _SRun* pSRun = &g_SChannel[ucChannelNum].SRun;
		TChangeTotalOffset(ucChannelNum, pSRun->ucToolCompNum, pSRun->cWorkpieceCoordNum, &pSRun->SCurrentOffset, &pSRun->SWorkPieceOffsetByG50);
		return 1;
	}
	return 0;
}

/*************************************************
  函数名称：clearRotateAXISToolWear(uint8 ucChannelNum)
  函数功能：清除ABC轴的刀具磨损参数
  输入参数：ucChannelNum		通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 clearRotateAXISToolWear(uint8 ucChannelNum)
{
	SToolParaInfo* STargetToolPara = &g_SSystemPara.SToolPara[ucChannelNum][0];
	SToolParaInfo* STargetWearPara = &g_SSystemPara.SWearPara[ucChannelNum][0];
	uint8 ucCounter;

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)
	{
		STargetToolPara[ucCounter].SToolCoord.dArray[AXIS_A] = 0;
		STargetToolPara[ucCounter].SToolCoord.dArray[AXIS_B] = 0;
		STargetToolPara[ucCounter].SToolCoord.dArray[AXIS_C] = 0;

		STargetWearPara[ucCounter].SToolCoord.dArray[AXIS_A] = 0;
		STargetWearPara[ucCounter].SToolCoord.dArray[AXIS_B] = 0;
		STargetWearPara[ucCounter].SToolCoord.dArray[AXIS_C] = 0;
	}
	updataTotalOffset(ucChannelNum, 0, TRUE);// 更新当前总偏移
	return 1;
}