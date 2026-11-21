#include "coordAPI.h"
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

/*************************************************
  函数名称：getMachCoordDouble(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取机械坐标 Double
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
  输出参数：
  返 回 值：Double XYZ轴:mm  C轴:°  X轴为直径值
  备    注：
**************************************************/
double getMachCoordDouble(uint8 ucChannelNum, uint8 ucAxis)
{
    double dTargetPoint = 0.0;
    const uint8* ucMapArray;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return dTargetPoint;
    }
    ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;
    if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
    {
        return 0;
    }
    getSingleAxisRealTimePoint(&dTargetPoint, ucAxis, ucMapArray); //获取单轴坐标

    if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS) //获取的轴为旋转轴
    {
        dTargetPoint = surplusDoubleToPositive(dTargetPoint, 360); //取余取正
    }
    else if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].bShowByDiameter == TRUE)
    {
        dTargetPoint *= 2;
    }
    return dTargetPoint;
}

/*************************************************
  函数名称：getMachCoordInt(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取机械坐标Int
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号 AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
  输出参数：
  返 回 值：Int XYZ轴:um  C轴:°*1000   X轴是直径值
  备    注：
**************************************************/
int getMachCoordInt(uint8 ucChannelNum, uint8 ucAxis)
{
    return round(getMachCoordDouble(ucChannelNum, ucAxis) * 1000.0);
}

/*************************************************
  函数名称：setMachCoord(uint8 ucChannelNum, uint8 ucAxis, double val)
  函数功能  设置机械坐标
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis	：AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
			val		：目标值  XYZ轴：mm  C轴：角度   X轴是直径值
  输出参数：
  返 回 值：- 1 轴号错误
			- 2 空指针
			- 3 无映射
			- 4 通道号错误
  备    注：
**************************************************/
int8 setMachCoord(uint8 ucChannelNum, uint8 ucAxis, double val)
{

    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -4;
    }

    return updateSingleAxisRealTimePoint(&val, ucAxis, g_SChannel[ucChannelNum].pMapAxisArray); // 更新目标轴当前点坐标
}

/*************************************************
  函数名称：getAbsCoordDouble(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取绝对坐标 Double
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号	AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
  输出参数：
  返 回 值：Double XYZ轴:mm  C轴:°X轴是直径值
  备    注：
**************************************************/
double getAbsCoordDouble(uint8 ucChannelNum, uint8 ucAxis)
{
    double dMachPoint = 0.0;
    double dTargetPoint = 0.0;
    const uint8* ucMapArray;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return dMachPoint;
    }
    ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray; //获取映射
    if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
    {
        return 0;
    }
    getSingleAxisRealTimePoint(&dMachPoint, ucAxis, ucMapArray);           //获取单轴机械坐标
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (g_SSystemPara.bUseVirtualY == TRUE) //开启合成虚拟Y轴功能：将物理轴的坐标 从 倾斜坐标系 转换至 直角坐标系
    {
        if (ucAxis == AXIS_X)
        {
            double dY1Mach = 0;
            double dYAbs = 0;
            getSingleAxisRealTimePoint(&dY1Mach, AXIS_Y, ucMapArray); //Y1轴坐标(半径值)
            dYAbs = dY1Mach * g_SChannel[ucChannelNum].dCOSAngleYY1;  //Y轴坐标
            dTargetPoint = dMachPoint - 1.0 * dYAbs * g_SChannel[ucChannelNum].dTANAngleYY1 -
                           g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray[ucAxis];
        }
        else if (ucAxis == AXIS_Y)
        {
            dTargetPoint = dMachPoint * g_SChannel[ucChannelNum].dCOSAngleYY1 -
                           g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray[ucAxis];
        }
        else
        {
            dTargetPoint = dMachPoint - g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray[ucAxis];
        }
    }
    else
    {
        dTargetPoint = dMachPoint - g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray[ucAxis];
    }
    if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS)
    {
        dTargetPoint = surplusDoubleToPositive(dTargetPoint, 360); //机械坐标减去总偏移坐标  取余取正
    }
    else if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].bShowByDiameter == TRUE)
    {
        dTargetPoint *= 2;
    }
#else
    dTargetPoint = dMachPoint - g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray[ucAxis];

    if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS)
    {
        dTargetPoint = surplusDoubleToPositive(dTargetPoint, 360); //机械坐标减去总偏移坐标  取余取正
    }
    else if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].bShowByDiameter == TRUE)
    {
        dTargetPoint *= 2;
    }
#endif
    return dTargetPoint;
}

/*************************************************
  函数名称：getAbsCoordInt(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取绝对坐标Int
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号  AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
  输出参数：
  返 回 值：Int XYZ轴:um  C轴:°*1000   X轴是直径值
  备    注：
**************************************************/
int getAbsCoordInt(uint8 ucChannelNum, uint8 ucAxis)
{
    return round(getAbsCoordDouble(ucChannelNum, ucAxis) * 1000.0);
}

/*************************************************
  函数名称：getRelaCoordDouble(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取相对坐标 Double
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号  AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
  输出参数：
  返 回 值：Double XYZ轴:mm  C轴:°  X轴是直径值
  备    注：
**************************************************/
double getRelaCoordDouble(uint8 ucChannelNum, uint8 ucAxis)
{
    double dMachPoint = 0.0;
    double dTargetPoint = 0.0;
    const uint8* ucMapArray;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return dMachPoint;
    }
    ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray; //获取映射
    if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
    {
        return 0;
    }
    //getSingleAxisRealTimePoint(&dMachPoint, ucAxis, ucMapArray);//获取单轴机械坐标
    //dTargetPoint = dMachPoint - g_SChannel[ucChannelNum].SRelaCoorStartPoint.dArray[ucAxis];//机械坐标减去相对坐标起点

    dTargetPoint =
        getAbsCoordDouble(ucChannelNum, ucAxis) -
        g_SChannel[ucChannelNum]
            .SRelaCoorStartPoint.dArray[ucAxis]; //陈敏聪 2023/5/27	相对坐标读取改为当前点的绝对坐标减相对坐标系起点

    if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS) //获取的轴为旋转轴
    {                                                                            //取余取正
        dTargetPoint = surplusDoubleToPositive(
            getAbsCoordDouble(ucChannelNum, ucAxis) - g_SChannel[ucChannelNum].SRelaCoorStartPoint.dArray[ucAxis], 360);
    }

    //if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS)//获取的轴为旋转轴?  //洪冠鹏 2024/5/21注释原因：读取绝对坐标已经对旋转轴和直径显示进行处理
    //{ //取余取正
    //	dTargetPoint = surplusDoubleToPositive(getAbsCoordDouble(ucChannelNum, ucAxis) - g_SChannel[ucChannelNum].SRelaCoorStartPoint.dArray[ucAxis], 360);
    //}
    //else if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].bShowByDiameter == TRUE)
    //{
    //	dTargetPoint *= 2;
    //}

    return dTargetPoint;
}

/*************************************************
  函数名称：getRelaCoordInt(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  获取相对坐标 Double
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
  输出参数：
  返 回 值：Int XYZ轴:mm  C轴:°  X轴是直径值
  备    注：
**************************************************/
int getRelaCoordInt(uint8 ucChannelNum, uint8 ucAxis)
{
    return round(getRelaCoordDouble(ucChannelNum, ucAxis) * 1000.0);
}

/*************************************************
  函数名称：setRelaCoord(uint8 ucChannelNum, uint8 ucAxis, double val)
  函数功能  设置当前相对坐标
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			AXIS_X,AXIS_Y,AXIS_Z,AXIS_C
			val				目标值
  输出参数：
  返 回 值：1  成功
			-1 失败
  备    注：
**************************************************/
int8 setRelaCoord(uint8 ucChannelNum, uint8 ucAxis, double val) //设置相对坐标
{
    double dAbsPoint;
    double dMachPoint = 0.0;
    const uint8* ucMapArray;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray; //获取映射
    if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
    {
        return 0;
    }
    //getSingleAxisRealTimePoint(&dMachPoint, ucAxis, ucMapArray);//获取单轴机械坐标
    //dAbsPoint = dMachPoint - g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray[ucAxis];
    dAbsPoint = getAbsCoordDouble(ucChannelNum, ucAxis);

    if (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAxisType == ROTATION_AXIS)
    {
        dAbsPoint = surplusDoubleToPositive(dAbsPoint, 360); //机械坐标减去总偏移坐标  取余取正
    }

    g_SChannel[ucChannelNum].SRelaCoorStartPoint.dArray[ucAxis] =
        dAbsPoint - val; //相对坐标 = 当前点的绝对坐标 - 目标值 陈敏聪 2023/5/27

    return 1;
}

/*************************************************
  函数名称：clearRalaCoor(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  将相对坐标清零
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis		 轴号  AXIS_X(0) AXIS_Y(1) AXIS_Z(2)  ...  AXIS_ALL(所有轴)
  输出参数：
  返 回 值：1 设置完成
			-1 通道号错误
  备    注：
****************************************/
//int8 clearRalaCoor(uint8 ucChannelNum, uint8 ucAxis)
//{
//	if (ucChannelNum >= CHANNEL_NUM_MAX)
//	{
//		return -1;
//	}
//
//	if (ucAxis == AXIS_ALL)//将相对坐标起点设置为当前点
//	{
//		getRealTimePoint((double*)&g_SChannel[ucChannelNum].SRelaCoorStartPoint.dArray, AXIS_NUM_MAX, g_SChannel[0].pMapAxisArray, NULL);
//	}
//	else
//	{
//		getSingleAxisRealTimePoint(&g_SChannel[ucChannelNum].SRelaCoorStartPoint.dArray[ucAxis], ucAxis, g_SChannel[0].pMapAxisArray);
//	}
//	return 1;
//}

/*************************************************
  函数名称：modifyCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis, double dOffset)
  函数功能 修改坐标系 《+输入》
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucNum 0 EXT; 1 G54; 2 G55 ; 3 G56 ; 4 G57 ; 5 G58 ; 6 G59
			ucAxis 轴号  仅为XYZ
			dOffset 输入值 单位mm
  输出参数：
  返 回 值：1 设置完成
			-1 通道号错误
			-2 ucNum 错误
			-3 轴号错误
  备    注：
****************************************/
int8 modifyCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis,
                      double dOffset) // ucNum : 0 EXT; 1 G54; 2 G55 ; 3 G56 ; 4 G57 ; 5 G58 ; 6 G59
{
    SPointDouble* pSCoorOffset = &g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset; //外部工件坐标零点EXT
    double* pdTargetCoor;
    double* pdEXTCoor = pSCoorOffset->dArray;

    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    if (ucNum > WORKPIECE_OFFSET_NUM) //超过偏移总数
    {
        return -2;
    }
    if (ucAxis != AXIS_X && ucAxis != AXIS_Y && ucAxis != AXIS_Z && ucAxis != AXIS_C && ucAxis != AXIS_A) //轴号错误
    {
        return -3;
    }

    pdTargetCoor = (double*)pSCoorOffset[ucNum].dArray; //指向目标偏移

    pdTargetCoor[ucAxis] += dOffset;          //修改目标轴号的值
    updataTotalOffset(ucChannelNum, 0, TRUE); // 更新当前总偏移
    return 1;
}

/*************************************************
  函数名称：setCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis, double dOffset)
  函数功能  设置外部/工件坐标系
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucNum 0 EXT; 1 G54; 2 G55 ; 3 G56 ; 4 G57 ; 5 G58 ; 6 G59
			ucAxis 轴号  仅为XYZ
			dOffset 输入值 单位mm
  输出参数：
  返 回 值：-1 ucNum 错误
			-2 轴号错误
			 1 设置完成
  备    注：
****************************************/
int8 setCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis,
                   double dOffset) // ucNum : 0 EXT; 1 G54; 2 G55 ; 3 G56 ; 4 G57 ; 5 G58 ; 6 G59
{
    SPointDouble* pSCoorOffset =
        (SPointDouble*)&g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset; //外部工件坐标零点EXT
    double* pdTargetCoor;
    double* pdEXTCoor = (double*)&pSCoorOffset[0].dArray;
    double dRealTimeValue;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    if (ucNum > WORKPIECE_OFFSET_NUM) //超过偏移总数
    {
        return -2;
    }
    if (ucAxis != AXIS_X && ucAxis != AXIS_Y && ucAxis != AXIS_Z && ucAxis != AXIS_C && ucAxis != AXIS_A) //轴号错误
    {
        return -3;
    }

    getSingleAxisRealTimePointByChannel(ucChannelNum, &dRealTimeValue, ucAxis);
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (g_SSystemPara.bUseVirtualY ==
        TRUE) //开启合成虚拟Y轴功能：获取的物理轴坐标是基于倾斜坐标系，偏移值应基于直角坐标系显示。因此，倾斜坐标系 -> 直角坐标系
    {
        if (ucAxis == AXIS_X)
        {
            double dY1RealTimeValue = 0.0;
            getSingleAxisRealTimePointByChannel(ucChannelNum, &dY1RealTimeValue, AXIS_Y); //获取Y1轴坐标
            dRealTimeValue -= 1.0 * dY1RealTimeValue * g_SChannel[ucChannelNum].dSINAngleYY1;
        }
        if (ucAxis == AXIS_Y)
        {
            dRealTimeValue *= g_SChannel[ucChannelNum].dCOSAngleYY1;
        }
    }
#endif
    uint8 ucPhysicalAxis = g_SChannel[ucChannelNum].pMapAxisArray[ucAxis];
    if (ucPhysicalAxis < AXIS_PHYSICS_NUM_MAX && g_SSystemPara.SAxisPara[ucPhysicalAxis].bShowByDiameter)
        dRealTimeValue *= 2.0; // 按实际物理轴的直径编程设置换算。

    pdTargetCoor = (double*)pSCoorOffset[ucNum].dArray; //指向目标偏移

    if (ucNum == 0) //EXT?
    {
        pdTargetCoor[ucAxis] = dRealTimeValue + dOffset;
    }
    else
    {
        pdTargetCoor[ucAxis] = dRealTimeValue - pdEXTCoor[ucAxis] + dOffset;
    }
    updataTotalOffset(ucChannelNum, 0, TRUE); // 更新当前总偏移
    return 1;
}

/*************************************************
  函数名称：inputCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis, double dOffset)
  函数功能  直接输入外部/工件坐标系偏移
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucNum 0 EXT; 1 G54; 2 G55 ; 3 G56 ; 4 G57 ; 5 G58 ; 6 G59
			ucAxis 轴号  仅为XYZ
			dOffset 输入值 单位mm
  输出参数：
  返 回 值： 1 设置完成
			-1 通道号错误
			-2 ucNum 错误
			-3 轴号错误
  备    注：
****************************************/
int8 inputCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis, double dOffset)
{
    SPointDouble* pSCoorOffset = &g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset; //外部工件坐标零点EXT
    double* pdTargetCoor;

    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    if (ucNum > WORKPIECE_OFFSET_NUM) //超过偏移总数
    {
        return -2;
    }
    if (ucAxis != AXIS_X && ucAxis != AXIS_Y && ucAxis != AXIS_Z && ucAxis != AXIS_C && ucAxis != AXIS_A) //轴号错误
    {
        return -3;
    }

    pdTargetCoor = (double*)pSCoorOffset[ucNum].dArray; //指向目标偏移

    pdTargetCoor[ucAxis] = dOffset;           //修改目标偏移
    updataTotalOffset(ucChannelNum, 0, TRUE); // 更新当前总偏移
    return 1;
}

/*************************************************
  函数名称：getCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis)
  函数功能  获取外部/工件坐标系偏移
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucChannel	通道号 0~CHANNEL_NUM_MAX - 1
			ucNum		0 EXT; 1 G54; 2 G55 ; 3 G56 ; 4 G57 ; 5 G58 ; 6 G59
			ucAxis		轴号  仅为XYZ
  输出参数：
  返 回 值：对应偏移值  单位 ：mm    X轴直径值
  备    注：
****************************************/
double getCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis)
{
    SPointDouble* pSCoorOffset = &g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset; //外部工件坐标零点EXT
    double* pdTargetCoor;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return 0.0;
    }
    if (ucNum > WORKPIECE_OFFSET_NUM) //超过偏移总数
    {
        return 0.0;
    }
    if (ucAxis != AXIS_X && ucAxis != AXIS_Y && ucAxis != AXIS_Z && ucAxis != AXIS_C && ucAxis != AXIS_A) //轴号错误
    {
        return 0.0;
    }
    pdTargetCoor = pSCoorOffset[ucNum].dArray; //指向目标
    return pdTargetCoor[ucAxis];
}

/*************************************************
  函数名称：getCoorOffsetByG50(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis)
  函数功能  获取系统G50临时坐标系偏移API
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号  仅为XZ
  输出参数：
  返 回 值：对应偏移值  单位 ：mm    X轴半径值
  备    注：
****************************************/
double getCoorOffsetByG50(uint8 ucChannelNum, uint8 ucAxis)
{
    double pSCoorOffsetByG50 =
        g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffsetByG50.dArray[ucAxis]; //获取系统G50临时坐标系偏移值
    if (ucChannelNum >= CHANNEL_NUM_MAX)                                        //通道号错误
    {
        return 0.0;
    }
    if (ucAxis != AXIS_X && ucAxis != AXIS_Z) //轴号错误
    {
        return 0.0;
    }
    if (ucAxis == AXIS_X)
    {
        pSCoorOffsetByG50 = pSCoorOffsetByG50 * 2.0;
    }
    return pSCoorOffsetByG50;
}
/*************************************************
  函数名称：setCoorOffsetByG50(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis)
  函数功能  设置G50临时坐标系偏移API
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
			ucAxis			轴号  仅为XZ
			dOffset 输入值 单位mm
  输出参数：
  返 回 值：-1 通道号错误
			-2 轴号错误
			 1 设置完成
  备    注：
****************************************/
int8 setCoorOffsetByG50(uint8 ucChannelNum, uint8 ucAxis, double dOffset)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    if (ucAxis != AXIS_X && ucAxis != AXIS_Z) //轴号错误
    {
        return -2;
    }
    if (ucAxis == AXIS_X)
    {
        g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffsetByG50.dArray[ucAxis] = dOffset / 2;
    }
    else
    {
        g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffsetByG50.dArray[ucAxis] = dOffset;
    }
    g_SChannel[ucChannelNum].SRun.SWorkPieceOffsetByG50 =
        g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffsetByG50; //将系统参数放在运行结构体中		陈敏聪	2023/11/11
    updataTotalOffset(ucChannelNum, 0, TRUE);                    // 更新当前总偏移
    return 1;
}
/*************************************************
  函数名称：clearToolOffset(uint8 ucChannelNum)
  函数功能：清除G50坐标系偏移API
  输入参数：
  输出参数：
  返 回 值：1 清除完成
			-1 通道错误
  备    注：
**************************************************/
int8 clearCoorOffset(uint8 ucChannelNum)
{
    uint8 ucCounter;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    for (ucCounter = 0; ucCounter < AXIS_PHYSICS_NUM_MAX; ucCounter++)
    {
        g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffsetByG50.dArray[ucCounter] = 0.0; //偏移坐标置0
    }
    updataTotalOffset(ucChannelNum, 0, TRUE); // 更新当前总偏移
    return 1;
}

/*************************************************
  函数名称：clearTotalToolOffset(uint8 ucChannelNum)
  函数功能：清除坐标系总偏移API
  输入参数：
  输出参数：
  返 回 值：1 清除完成
			-1 通道错误
  备    注：
**************************************************/
int8 clearTotalCoorOffset(uint8 ucChannelNum)
{
    uint8 ucCounter;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    for (ucCounter = 0; ucCounter < AXIS_PHYSICS_NUM_MAX; ucCounter++)
    {
        g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset.dArray[ucCounter] = 0.0; //偏移坐标置0
    }
    updataTotalOffset(ucChannelNum, 0, TRUE); // 更新当前总偏移
    return 1;
}
