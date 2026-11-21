/*
G2 Z10 R0 按点输出
*/

#include <stdio.h>
#include "common.h"
#include "GeneralFunctions.h"

#include "GCode.h"
#include "ringbuff.h"
#include "ErrorFunction.h"



static SG2Para* pSG2ParaCheck[CHANNEL_NUM_MAX];
static SG2Para* pSG2ParaAssign[CHANNEL_NUM_MAX];





//添加G2规则结构体
void addG2RegulationAPI()
{
	//指令：G2
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	int32 a=10;
	addGCommadRegulation(2, sGa, 1, mallocG2ParaStruct, freeG2ParaStruct, getG2Para, CheckG2, AssignG2);
}


/*************************************************
  函数名称：int8 mallocG2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G2参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG2ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG2ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG2Para), ORAM_NUM400);//暂未对内存编号
		if (pSG2ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG2ParaCheck[ucChannelNum], 0, sizeof(SG2Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG2ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG2ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG2Para), ORAM_NUM401);
		if (pSG2ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG2ParaAssign[ucChannelNum], 0, sizeof(SG2Para));
		return 1;
	}

	
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG2ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG2ParaCheck[ucChannelNum]);
			pSG2ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG2ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG2ParaAssign[ucChannelNum]);
			pSG2ParaAssign[ucChannelNum] = NULL;
		}
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{

}

/*************************************************
  函数名称：CheckG2
  函数功能：检查 G2 的坐标输入并计算当前检查记录。
  输入参数：有效检查配置和记录；本段非运动模态已更新，扫描坐标尚未转换。
  输出参数：沿用原坐标转换和记录写入；G16 输入拒绝时扫描数据和记录不变。
  返 回 值：2 本段检查完成；-1 G16 输入非法或原有失败分支。
  备    注：坐标处理区先校验 G16 输入；其他参数和几何检查沿用原流程。
**************************************************/
int8 CheckG2(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		//报错
		return -1;
	}
	//0.判断G2程序段中是否出现坐标信息 
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		// 按本段坐标模态检查输入，通过后再进行坐标转换。
		if (checkPolarCoordinateInput(&pSSyntaxConfig->SScanContainer, &pSSyntaxConfig->SGCodeCurrSysState) < 0)
		{
			return -1;
		}

		//1.指令字符检查
		checkParaLetter("XYZABCUVWIJKRLD", &pSSyntaxConfig->SScanContainer);

		//2.参数值范围检查
		checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('A', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('B', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('C', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('I', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('J', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('K', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('R', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('L', &pSSyntaxConfig->SScanContainer, 0, 999);
		checkParaRange('D', &pSSyntaxConfig->SScanContainer, 0, 999);
		// 2.1 显式拦截互斥参数同时出现
		if (pSSyntaxConfig->SScanContainer.dD>0 && pSSyntaxConfig->SScanContainer.dL>0)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, 280);
			return -1;                             
		}
		//1.获取G2参数
		//1.1获取G2横轴、纵轴
		SG2Para* pSG2Para = pSG2ParaCheck[pSSyntaxConfig->ucChannelNum];
		bool bIsAbsMethod;
		getHorAndVerAxis(&pSG2Para->ucG2Hor, &pSG2Para->ucG2Ver, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);
		//1.2获取G2的起点与终点坐标（机械坐标）
		pSG2Para->SG2EndPoint = pSG2Para->SG2StartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		//获取当前的坐标编程方式
		bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
		//计算坐标
		getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSG2Para->SG2StartPoint, &pSG2Para->SG2EndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
		//1.3获取G2的方向
		pSG2Para->cG2Dir = ARC_CW;
		//1.4获取G2圆弧的圆心与半径
		getArcCenterPointAndRadius(&pSSyntaxConfig->SScanContainer, &pSG2Para->SG2StartPoint, &pSG2Para->SG2EndPoint, pSG2Para->cG2Dir, &pSG2Para->SG2CenterPoint, &pSG2Para->dG2Radius, pSG2Para->ucG2Hor, pSG2Para->ucG2Ver);
		//1.5获取G2类型（0:点||1:圆弧||2:整圆）
		SArc SArcInfo;//存圆弧段信息（可能要将此变量放在赋值结构体中）
		memset(&SArcInfo, 0, sizeof(SArc));

		//1.写圆弧段信息SArcInfo
		SArcInfo.SStartPoint = pSG2Para->SG2StartPoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG2Para->SG2EndPoint;
		SArcInfo.SCenterPoint = pSG2Para->SG2CenterPoint;
		SArcInfo.dRadius = pSG2Para->dG2Radius;
		SArcInfo.cArcDir = pSG2Para->cG2Dir;
		SArcInfo.EntityType = pSG2Para->cType == 0 ? DOT : ARC;
		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		return 2;
	}
	return 2;
									
}


/*************************************************
  函数名称：int8 AssignG2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值顺时针圆弧；普通整圆沿既有流程拆成两个半圆。
  输入参数：pSAssignConfig 赋值配置及步骤状态；pGCodeBufUnit 当前写单元。
  输出参数：圆弧记录及后续赋值步骤。
  返 回 值：-1 G16 输入非法或整圆倒角边界错误；0 预处理完成待续；1 已写中间段待续；2 本指令完成。
  备    注：当前整圆有 L/D，或前段有待处理 L/D 时拒绝；避免在整圆内部拆分点处理倒角。
**************************************************/
int8 AssignG2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SG2Para* pSG2Para = pSG2ParaAssign[pSAssignConfig->ucChannelNum];
	bool bIsAbsMethod;

	switch (pSG2Para->cStep)
	{
	//赋值预处理
	case 0:
	{
		// 赋值重新扫描并求值，校验本次实际用于生成运动的输入。
		if (checkPolarCoordinateInput(&pSAssignConfig->SScanContainer, &pSAssignConfig->SGCodeCurrSysState) < 0)
		{
			return -1;
		}

		//1.获取G2参数
		//1.1获取G2横轴、纵轴
		getHorAndVerAxis(&pSG2Para->ucG2Hor, &pSG2Para->ucG2Ver, pSAssignConfig->pGCodeBuf[pSAssignConfig->usIndex].cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);
		//1.2获取G2的起点与终点坐标（机械坐标）
		pSG2Para->SG2EndPoint = pSG2Para->SG2StartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		//获取当前的坐标编程方式
		
		bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
		//计算坐标
		getMacCoordByScanInfo(pSAssignConfig->ucChannelNum,&pSAssignConfig->SScanContainer, &pSG2Para->SG2StartPoint, &pSG2Para->SG2EndPoint, &pSAssignConfig->SGCodeCurrSysState);
		//1.3获取G2的方向
		pSG2Para->cG2Dir = ARC_CW;
		//1.4获取G2圆弧的圆心与半径
		getArcCenterPointAndRadius(&pSAssignConfig->SScanContainer, &pSG2Para->SG2StartPoint, &pSG2Para->SG2EndPoint, pSG2Para->cG2Dir, &pSG2Para->SG2CenterPoint, &pSG2Para->dG2Radius, pSG2Para->ucG2Hor, pSG2Para->ucG2Ver);
		//1.5获取G2类型（0:点||1:圆弧||2:整圆）
		pSG2Para->cType = getArcType(&pSG2Para->SG2StartPoint, &pSG2Para->SG2EndPoint, &pSG2Para->SG2CenterPoint, pSG2Para->dG2Radius, pSG2Para->ucG2Hor, pSG2Para->ucG2Ver);
		
		//2.整圆需要额外计算SG2MiddlePoint，用于后续割圆
        /* 必须在整圆拆段前检查请求；内部半圆连接不等于源程序段连接。 */
        if (pSG2Para->cType == 2) {
            int16 wi=getSGCodeParsePointerIndex(GCODE_WRITE,pSAssignConfig->ucChannelNum);
            struct SGCodeParse *tail=getSGCodeParsePointerByIndex(wi-1,pSAssignConfig->ucChannelNum);
            if (pSAssignConfig->SScanContainer.bD || pSAssignConfig->SScanContainer.bL
                || (getSGCodeParsePointerBufRes(GCODE_READ,pSAssignConfig->ucChannelNum)>0
                    && (tail->dRoundedRadius != 0.0 || tail->dChamferLen != 0.0))) {
                parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum,283);
                return -1;
            }
        }
		if (pSG2Para->cType == 2)
		{
			uint8 ucStraightMoveAxis ;  //直线移动的轴  整圆拆分时需要分半到两个半圆上
			int16 sGcCommand = pGCodeBufUnit->SModalInfo.sGc;

			if (sGcCommand == G17 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{
				ucStraightMoveAxis = AXIS_Z;
			}
			else if (sGcCommand == G18 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{
				ucStraightMoveAxis = AXIS_Y;
			}
			else if (sGcCommand == G19 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{
				ucStraightMoveAxis = AXIS_X;
			}
			else
			{
				ucStraightMoveAxis = AXIS_Y;
			}
			pSG2Para->SG2MiddlePoint.dArray[pSG2Para->ucG2Hor] = pSG2Para->SG2CenterPoint.dArray[pSG2Para->ucG2Hor] - (pSG2Para->SG2StartPoint.dArray[pSG2Para->ucG2Hor] - pSG2Para->SG2CenterPoint.dArray[pSG2Para->ucG2Hor]);
			pSG2Para->SG2MiddlePoint.dArray[pSG2Para->ucG2Ver] = pSG2Para->SG2CenterPoint.dArray[pSG2Para->ucG2Ver] - (pSG2Para->SG2StartPoint.dArray[pSG2Para->ucG2Ver] - pSG2Para->SG2CenterPoint.dArray[pSG2Para->ucG2Ver]);
			//直线移动轴的计算
			pSG2Para->SG2MiddlePoint.dArray[ucStraightMoveAxis] = pSG2Para->SG2StartPoint.dArray[ucStraightMoveAxis]+ (pSG2Para->SG2EndPoint.dArray[ucStraightMoveAxis] - pSG2Para->SG2StartPoint.dArray[ucStraightMoveAxis]) / 2;
		}

		pSG2Para->cStep = pSG2Para->cType == 2 ? 2 : 1;

		return 0;
		break;
	}
	//赋值点||圆弧
	case 1:
	{
		SArc SArcInfo;//存圆弧段信息（可能要将此变量放在赋值结构体中）
		memset(&SArcInfo, 0, sizeof(SArc));

		//1.写圆弧段信息SArcInfo
		SArcInfo.SStartPoint = pSG2Para->SG2StartPoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG2Para->SG2EndPoint;
		SArcInfo.SCenterPoint = pSG2Para->SG2CenterPoint;
		SArcInfo.dRadius = pSG2Para->dG2Radius;
		SArcInfo.cArcDir = pSG2Para->cG2Dir;
		SArcInfo.EntityType = pSG2Para->cType == 0 ? DOT : ARC;
		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		
		//3.自动倒角/倒圆参数转写（L/D后缀，Fanuc风格）
			if (pSAssignConfig->SScanContainer.bD)
			{
				double dD = pSAssignConfig->SScanContainer.dD;
				pGCodeBufUnit->dRoundedRadius = dD;
			}
			if (pSAssignConfig->SScanContainer.bL)
			{
				double dL = pSAssignConfig->SScanContainer.dL;
				pGCodeBufUnit->dChamferLen = dL;
			}

		//极值点
		pSAssignConfig->SGetAxisMaxAndMin.cArcDir = pGCodeBufUnit->cArcDir;
		pSAssignConfig->SGetAxisMaxAndMin.dRadius = pGCodeBufUnit->dRadius;
		pSAssignConfig->SGetAxisMaxAndMin.SStartPoint = &pGCodeBufUnit->SStartPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SEndPoint = &pGCodeBufUnit->SEndPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SCenterPoint = &pGCodeBufUnit->SCenterPoint;
		pSAssignConfig->SGetAxisMaxAndMin.ucHor = pSG2Para->ucG2Hor;
		pSAssignConfig->SGetAxisMaxAndMin.ucVer = pSG2Para->ucG2Ver;
		getAxisMaxAndMin(&pSAssignConfig->SGetAxisMaxAndMin);

		pGCodeBufUnit->dHorMax = pSAssignConfig->SGetAxisMaxAndMin.dHorMax;
		pGCodeBufUnit->dHorMin = pSAssignConfig->SGetAxisMaxAndMin.dHorMin;
		pGCodeBufUnit->dVerMax = pSAssignConfig->SGetAxisMaxAndMin.dVerMax;
		pGCodeBufUnit->dVerMin = pSAssignConfig->SGetAxisMaxAndMin.dVerMin;

		pSG2Para->cStep = 0;
		return 2;
		break;
	}
	//赋值整圆1
	case 2:
	{
		//1.写圆弧段信息SArcInfo
		SArc SArcInfo;
		memset(&SArcInfo, 0, sizeof(SArc));

		SArcInfo.SStartPoint = pSG2Para->SG2StartPoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG2Para->SG2MiddlePoint;
		SArcInfo.SCenterPoint = pSG2Para->SG2CenterPoint;
		SArcInfo.dRadius = pSG2Para->dG2Radius;
		SArcInfo.cArcDir = pSG2Para->cG2Dir;
		SArcInfo.EntityType = ARC;

		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		//极值点
		pSAssignConfig->SGetAxisMaxAndMin.cArcDir = pGCodeBufUnit->cArcDir;
		pSAssignConfig->SGetAxisMaxAndMin.dRadius = pGCodeBufUnit->dRadius;
		pSAssignConfig->SGetAxisMaxAndMin.SStartPoint = &pGCodeBufUnit->SStartPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SEndPoint = &pGCodeBufUnit->SEndPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SCenterPoint = &pGCodeBufUnit->SCenterPoint;
		pSAssignConfig->SGetAxisMaxAndMin.ucHor = pSG2Para->ucG2Hor;
		pSAssignConfig->SGetAxisMaxAndMin.ucVer = pSG2Para->ucG2Ver;
		getAxisMaxAndMin(&pSAssignConfig->SGetAxisMaxAndMin);

		pGCodeBufUnit->dHorMax = pSAssignConfig->SGetAxisMaxAndMin.dHorMax;
		pGCodeBufUnit->dHorMin = pSAssignConfig->SGetAxisMaxAndMin.dHorMin;
		pGCodeBufUnit->dVerMax = pSAssignConfig->SGetAxisMaxAndMin.dVerMax;
		pGCodeBufUnit->dVerMin = pSAssignConfig->SGetAxisMaxAndMin.dVerMin;

		pSG2Para->cStep = 3;

		return 1;
	}
	//赋值整圆2
	case 3:
	{
		//1.写圆弧段信息SArcInfo
		SArc SArcInfo;
		memset(&SArcInfo, 0, sizeof(SArc));

		SArcInfo.SStartPoint = pSG2Para->SG2MiddlePoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG2Para->SG2EndPoint;
		SArcInfo.SCenterPoint = pSG2Para->SG2CenterPoint;
		SArcInfo.dRadius = pSG2Para->dG2Radius;
		SArcInfo.cArcDir = pSG2Para->cG2Dir;
		SArcInfo.EntityType = ARC;

		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		//极值点
		pSAssignConfig->SGetAxisMaxAndMin.cArcDir = pGCodeBufUnit->cArcDir;
		pSAssignConfig->SGetAxisMaxAndMin.dRadius = pGCodeBufUnit->dRadius;
		pSAssignConfig->SGetAxisMaxAndMin.SStartPoint = &pGCodeBufUnit->SStartPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SEndPoint = &pGCodeBufUnit->SEndPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SCenterPoint = &pGCodeBufUnit->SCenterPoint;
		pSAssignConfig->SGetAxisMaxAndMin.ucHor = pSG2Para->ucG2Hor;
		pSAssignConfig->SGetAxisMaxAndMin.ucVer = pSG2Para->ucG2Ver;
		getAxisMaxAndMin(&pSAssignConfig->SGetAxisMaxAndMin);

		pGCodeBufUnit->dHorMax = pSAssignConfig->SGetAxisMaxAndMin.dHorMax;
		pGCodeBufUnit->dHorMin = pSAssignConfig->SGetAxisMaxAndMin.dHorMin;
		pGCodeBufUnit->dVerMax = pSAssignConfig->SGetAxisMaxAndMin.dVerMax;
		pGCodeBufUnit->dVerMin = pSAssignConfig->SGetAxisMaxAndMin.dVerMin;

		pSG2Para->cStep = 0;
		return 2;
	}
	default:
		break;
	}
}





