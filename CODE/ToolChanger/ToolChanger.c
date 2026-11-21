#include "ToolChanger.h"

#include "G1.h" 
#include "G0.h"  
#include "Motion.h"	
#include <stdio.h>
#include <string.h>



int8 toolInitAfterLoad()
{
	uint8 ucCounter=0,ucCounter1=0;
	for ( ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		for (ucCounter1 = 1; ucCounter1 <= TOOL_NUM_MAX; ucCounter1++)
		{
			g_SSystemPara.SToolPara[ucCounter][ucCounter1 - 1].ucToolID = ucCounter1;
		}
	}

	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		for (ucCounter1 = 1; ucCounter1 <= TOOLWEAR_GROUP_NUM * TOOL_NUM_MAX; ucCounter1++)
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
  函数名称：clearAllToolOffset()
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

	if (ucChannelNum>=CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)//遍历该通道下的刀具参数
	{
		STargetPara[ucCounter].SToolCoord = SZeroPoint;//偏移坐标置0
	}

	return 1;
}



/*************************************************
  函数名称：clearSingleToolOffset(uint8 ucToolID)
  函数功能：清除一行的刀具偏移
  输入参数：ucToolID  刀具号
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
  备    注：
**************************************************/
int8 clearSingleToolOffset(uint8 ucChannelNum,uint8 ucToolID)
{
	SToolParaInfo* STargetPara = (SToolParaInfo*)&g_SSystemPara.SToolPara[0];//指向刀具偏移
	SPointDouble SZeroPoint = { {0} };//零点坐标

	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}

	STargetPara[ucToolID - 1].SToolCoord = SZeroPoint;//偏移坐标置0
	return 1;
}



/*************************************************
  函数名称：setToolOffset(uint8 ucToolID, uint8 ucAxis, double dOffsetVal)
  函数功能：设置指定刀号的刀补偏移
  输入参数：ucToolID			刀具号
			ucAxis				轴号      从0开始   0~5
			dOffsetVal			输入偏移值
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
			-3 轴号错误
  备    注：
**************************************************/
int8 setToolOffset(uint8 ucChannelNum, uint8 ucToolID, uint8 ucAxis, double dOffsetVal)
{
	double* pdToolCoord;//目标的刀具坐标
	double dRealTimeCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
	{
		return -3;
	}
	if (ucAxis == AXIS_X)//X轴将输入除2
	{
		dOffsetVal /= 2;
	}
	getSingleAxisRealTimePointByChannel(ucChannelNum, &dRealTimeCoord, ucAxis); //实时点坐标
	pdToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][ucToolID - 1].SToolCoord.dArray;//目标的刀具坐标

	pdToolCoord[ucAxis] = dOffsetVal + dRealTimeCoord;//输入值+机械坐标

	return 1;
}


/*************************************************
  函数名称：inputToolOffset( uint8 ucToolID,uint8 ucAxis ,double dOffsetVal)
  函数功能：设置指定刀号的刀补偏移
  输入参数：ucToolID			刀具号
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
int8 inputToolOffset(uint8 ucChannelNum, uint8 ucToolID, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
{
	double* pdToolCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
	{
		return -3;
	}
	if (ucAxis == AXIS_X)//X轴为输入值的1/2
	{
		dOffsetVal /= 2;
	}
	pdToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][ucToolID - 1].SToolCoord.dArray;//确定目标坐标

	if (bIsRelaVal)
	{
		pdToolCoord[ucAxis] += dOffsetVal;//设置为输入的偏移值
	}
	else
	{
		pdToolCoord[ucAxis] = dOffsetVal;//设置为输入的偏移值
	}

	return 1;
}

//获取偏移值
double getToolOffset(uint8 ucChannelNum, uint8 ucToolID, uint8 ucAxis)
{
	double* pdToolCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return 0.0;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return 0.0;
	}
	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
	{
		return 0.0;
	}

	pdToolCoord = (double*)&g_SSystemPara.SToolPara[ucChannelNum][ucToolID - 1].SToolCoord.dArray;//确定目标坐标

}


/*************************************************
  函数名称：clearAllToolOffset()
  函数功能：清除所有刀具ID的半径
  输入参数：
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
  输入参数：ucToolID  刀具号
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
  输入参数：ucToolID			刀具号
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

	if (ucChannelNum >=CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}

	STargetPara[ucToolID].dToolRadius = dToolRadius;//设置为输入的偏移值

	return 1;
}

/*************************************************
  函数名称：getToolRadius(uint8 ucChannelNum, uint8 ucToolID)
  函数功能：获取指定刀号的半径
  输入参数：ucToolID			刀具号
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

	return STargetPara[ucToolID].dToolRadius ;//偏移值
}

/*************************************************
  函数名称：clearAllToolWear()
  函数功能：清除所有刀具ID的磨损值
  输入参数：
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

	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX * TOOLWEAR_GROUP_NUM; ucCounter++)
	{
		STargetPara[ucCounter].SToolCoord = SZeroPoint;//偏移坐标置0
		//STargetPara[ucCounter].dToolRadius = 0;
	}

	return 1;
}


/*************************************************
  函数名称：clearSingleToolWear(uint8 ucToolID,uint8 ucGroupNum)
  函数功能：清除一行刀具ID的磨损值
  输入参数：ucToolID	刀具号
			ucGroupNum	磨损组号
  输出参数：
  返 回 值：1 设置完成
			-1 通道错误
			-2 刀具号超过刀具数量
			-3 组号错误
  备    注：
**************************************************/
int8 clearSingleToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum)
{
	SToolParaInfo* STargetPara = &g_SSystemPara.SToolPara[ucChannelNum][0];
	SPointDouble SZeroPoint = { {0} };
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	if (ucGroupNum > TOOLWEAR_GROUP_NUM) return -3;


	STargetPara[ucToolID + (ucGroupNum - 1) * TOOL_NUM_MAX - 1].SToolCoord = SZeroPoint;//偏移坐标置0
	//STargetPara[ucToolID - 1].dToolRadius = 0;//半径置0
	return 1;
}


/*************************************************
  函数名称：inputToolWear(uint8 ucToolID, uint8 ucGroupNum,uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
  函数功能：设置指定刀号的刀具磨损值
  输入参数：ucToolID			刀具号
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
int8 inputToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
{
	double* pdToolCoord;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return -1;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return -2;
	}
	if (ucGroupNum > TOOLWEAR_GROUP_NUM)
	{
		return -4;
	}

	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
	{
		return -3;
	}
	if (ucAxis == AXIS_X)//X轴为输入值的1/2
	{
		dOffsetVal /= 2;
	}

	pdToolCoord = (double*)&g_SSystemPara.SWearPara[ucChannelNum][ucToolID + (ucGroupNum -1)* TOOL_NUM_MAX - 1].SToolCoord.dArray;//确定目标坐标

	if (bIsRelaVal)
	{
		pdToolCoord[ucAxis] += dOffsetVal;//设置为输入的偏移值
	}
	else
	{
		pdToolCoord[ucAxis] = dOffsetVal;//设置为输入的偏移值
	}

	return 1;
}


/*************************************************
  函数名称：inputToolWear(uint8 ucToolID, uint8 ucGroupNum,uint8 ucAxis, double dOffsetVal, bool bIsRelaVal)
  函数功能：设置指定刀号的刀具磨损值
  输入参数：ucToolID			刀具号
			ucAxis				轴号   从0开始
			dOffsetVal			输入偏移值
			bIsRelaVal			是不是相对值（在原基础上加？)
  输出参数：
  返 回 值：刀具磨损值
  备    注：
**************************************************/
double getToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum, uint8 ucAxis)
{
	double dRes;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//通道号错误
	{
		return 0.0;
	}
	if (ucToolID > TOOL_NUM_MAX)//刀号大于刀具数量
	{
		return 0.0;
	}
	if (ucGroupNum > TOOLWEAR_GROUP_NUM)
	{
		return 0.0;
	}

	if (ucAxis != AXIS_X && ucAxis != AXIS_Z)//刀具偏移只有X和Z
	{
		return 0.0;
	}

	dRes = g_SSystemPara.SWearPara[ucChannelNum][ucToolID+(ucGroupNum - 1) * TOOL_NUM_MAX - 1].SToolCoord.dArray[ucAxis];//确定目标坐标
	return (ucAxis == AXIS_X) ? dRes / 2.0 : dRes;
}