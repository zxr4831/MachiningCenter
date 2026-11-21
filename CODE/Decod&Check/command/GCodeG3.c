/*
G3 Z10 R0 按点输出
*/

#include <stdio.h>
#include "common.h"
#include "GeneralFunctions.h"

#include "GCode.h"
#include "ringbuff.h"
#include "ErrorFunction.h"



static SG3Para* pSG3ParaCheck[CHANNEL_NUM_MAX];
static SG3Para* pSG3ParaAssign[CHANNEL_NUM_MAX];



//添加G3规则结构体
void addG3RegulationAPI()
{
	//指令：G3
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	int32 a = 10;
	addGCommadRegulation(3, sGa, 1, mallocG3ParaStruct, freeG3ParaStruct, getG3Para, CheckG3, AssignG3);
}


/*************************************************
  函数名称：int8 mallocG3ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G3参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG3ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG3ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG3ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG3Para), ORAM_NUM402);//暂未对内存编号
		if (pSG3ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG3ParaCheck[ucChannelNum], 0, sizeof(SG3Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG3ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG3ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG3Para), ORAM_NUM403);
		if (pSG3ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG3ParaAssign[ucChannelNum], 0, sizeof(SG3Para));
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
int8 freeG3ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG3ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG3ParaCheck[ucChannelNum]);
			pSG3ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG3ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG3ParaAssign[ucChannelNum]);
			pSG3ParaAssign[ucChannelNum] = NULL;
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
int8 getG3Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{

}

/*************************************************
  函数名称：CheckG3
  函数功能：检查 G3 的坐标输入并计算当前检查记录。
  输入参数：有效检查配置和记录；本段非运动模态已更新，扫描坐标尚未转换。
  输出参数：沿用原坐标转换和记录写入；G16 输入拒绝时扫描数据和记录不变。
  返 回 值：2 本段检查完成；-1 G16 输入非法或原有失败分支。
  备    注：坐标处理区先校验 G16 输入；其他参数和几何检查沿用原流程。
**************************************************/
int8 CheckG3(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		//报错
		return -1;
	}
	//0.判断G3程序段中是否出现坐标信息 
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

		//1.获取G3参数
		//1.1获取G3横轴、纵轴
		SG3Para* pSG3Para = pSG3ParaCheck[pSSyntaxConfig->ucChannelNum];
		bool bIsAbsMethod;
		getHorAndVerAxis(&pSG3Para->ucG3Hor, &pSG3Para->ucG3Ver, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);
		//1.2获取G3的起点与终点坐标（机械坐标）
		pSG3Para->SG3EndPoint = pSG3Para->SG3StartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		//获取当前的坐标编程方式
		bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
		//计算坐标
		getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSG3Para->SG3StartPoint, &pSG3Para->SG3EndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
		//1.3获取G3的方向
		pSG3Para->cG3Dir = ARC_ANTI_CW;
		//1.4获取G3圆弧的圆心与半径
		getArcCenterPointAndRadius(&pSSyntaxConfig->SScanContainer, &pSG3Para->SG3StartPoint, &pSG3Para->SG3EndPoint, pSG3Para->cG3Dir, &pSG3Para->SG3CenterPoint, &pSG3Para->dG3Radius, pSG3Para->ucG3Hor, pSG3Para->ucG3Ver);
		//1.5获取G3类型（0:点||1:圆弧||2:整圆）
		SArc SArcInfo;//存圆弧段信息（可能要将此变量放在赋值结构体中）
		memset(&SArcInfo, 0, sizeof(SArc));

		//1.写圆弧段信息SArcInfo
		SArcInfo.SStartPoint = pSG3Para->SG3StartPoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG3Para->SG3EndPoint;
		SArcInfo.SCenterPoint = pSG3Para->SG3CenterPoint;
		SArcInfo.dRadius = pSG3Para->dG3Radius;
		SArcInfo.cArcDir = pSG3Para->cG3Dir;
		SArcInfo.EntityType = pSG3Para->cType == 0 ? DOT : ARC;
		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		return 2;
	}
	return 2;
}


/*************************************************
  函数名称：int8 AssignG3(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值逆时针圆弧；普通整圆沿既有流程拆成两个半圆。
  输入参数：pSAssignConfig 赋值配置及步骤状态；pGCodeBufUnit 当前写单元。
  输出参数：圆弧记录及后续赋值步骤。
  返 回 值：-1 G16 输入非法或整圆倒角边界错误；0 预处理完成待续；1 已写中间段待续；2 本指令完成。
  备    注：当前整圆有 L/D，或前段有待处理 L/D 时拒绝；避免在整圆内部拆分点处理倒角。
**************************************************/
int8 AssignG3(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SG3Para* pSG3Para = pSG3ParaAssign[pSAssignConfig->ucChannelNum];
	bool bIsAbsMethod;
	switch (pSG3Para->cStep)
	{
		//赋值预处理
	case 0:
	{
		// 赋值重新扫描并求值，校验本次实际用于生成运动的输入。
		if (checkPolarCoordinateInput(&pSAssignConfig->SScanContainer, &pSAssignConfig->SGCodeCurrSysState) < 0)
		{
			return -1;
		}

		//1.获取G3参数
		//1.1获取G3横轴、纵轴
		getHorAndVerAxis(&pSG3Para->ucG3Hor, &pSG3Para->ucG3Ver, pSAssignConfig->pGCodeBuf[pSAssignConfig->usIndex].cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);
		//1.2获取G3的起点与终点坐标（机械坐标）
		pSG3Para->SG3EndPoint = pSG3Para->SG3StartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		//获取当前的坐标编程方式
		bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
		//计算机械坐标
		getMacCoordByScanInfo(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &pSG3Para->SG3StartPoint, &pSG3Para->SG3EndPoint, &pSAssignConfig->SGCodeCurrSysState);
		//1.3获取G3的方向
		pSG3Para->cG3Dir = ARC_ANTI_CW;
		//1.4获取G3圆弧的圆心与半径
		getArcCenterPointAndRadius(&pSAssignConfig->SScanContainer, &pSG3Para->SG3StartPoint, &pSG3Para->SG3EndPoint, pSG3Para->cG3Dir, &pSG3Para->SG3CenterPoint, &pSG3Para->dG3Radius, pSG3Para->ucG3Hor, pSG3Para->ucG3Ver);
		//1.5获取G3类型（0:点||1:圆弧||2:整圆）
		pSG3Para->cType = getArcType(&pSG3Para->SG3StartPoint, &pSG3Para->SG3EndPoint, &pSG3Para->SG3CenterPoint, pSG3Para->dG3Radius, pSG3Para->ucG3Hor, pSG3Para->ucG3Ver);

		//2.整圆需要额外计算SG3MiddlePoint，用于后续割圆
        /* 与 G2 一致，在拆成两个半圆之前阻止整圆参与当前版本倒圆。 */
        if (pSG3Para->cType == 2) {
            int16 wi=getSGCodeParsePointerIndex(GCODE_WRITE,pSAssignConfig->ucChannelNum);
            struct SGCodeParse *tail=getSGCodeParsePointerByIndex(wi-1,pSAssignConfig->ucChannelNum);
            if (pSAssignConfig->SScanContainer.bD || pSAssignConfig->SScanContainer.bL
                || (getSGCodeParsePointerBufRes(GCODE_READ,pSAssignConfig->ucChannelNum)>0
                    && (tail->dRoundedRadius != 0.0 || tail->dChamferLen != 0.0))) {
                parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum,283);
                return -1;
            }
        }
		if (pSG3Para->cType == 2)
		{
			uint8 ucStraightMoveAxis;  //直线移动的轴  整圆拆分时需要分半到两个半圆上
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
			pSG3Para->SG3MiddlePoint = pSG3Para->SG3StartPoint;
			pSG3Para->SG3MiddlePoint.dArray[pSG3Para->ucG3Hor] = pSG3Para->SG3CenterPoint.dArray[pSG3Para->ucG3Hor] - (pSG3Para->SG3StartPoint.dArray[pSG3Para->ucG3Hor] - pSG3Para->SG3CenterPoint.dArray[pSG3Para->ucG3Hor]);
			pSG3Para->SG3MiddlePoint.dArray[pSG3Para->ucG3Ver] = pSG3Para->SG3CenterPoint.dArray[pSG3Para->ucG3Ver] - (pSG3Para->SG3StartPoint.dArray[pSG3Para->ucG3Ver] - pSG3Para->SG3CenterPoint.dArray[pSG3Para->ucG3Ver]);
			//直线移动轴的计算
			pSG3Para->SG3MiddlePoint.dArray[ucStraightMoveAxis] = pSG3Para->SG3StartPoint.dArray[ucStraightMoveAxis]
				+ (pSG3Para->SG3EndPoint.dArray[ucStraightMoveAxis] - pSG3Para->SG3StartPoint.dArray[ucStraightMoveAxis]) / 2;
		}

		pSG3Para->cStep = pSG3Para->cType == 2 ? 2 : 1;

		return 0;
		break;
	}
	//赋值点||圆弧
	case 1:
	{
		SArc SArcInfo;//存圆弧段信息（可能要将此变量放在赋值结构体中）
		memset(&SArcInfo, 0, sizeof(SArc));

		//1.写圆弧段信息SArcInfo
		SArcInfo.SStartPoint = pSG3Para->SG3StartPoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG3Para->SG3EndPoint;
		SArcInfo.SCenterPoint = pSG3Para->SG3CenterPoint;
		SArcInfo.dRadius = pSG3Para->dG3Radius;
		SArcInfo.cArcDir = pSG3Para->cG3Dir;
		SArcInfo.EntityType = pSG3Para->cType == 0 ? DOT : ARC;
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
		pSAssignConfig->SGetAxisMaxAndMin.ucHor = pSG3Para->ucG3Hor;
		pSAssignConfig->SGetAxisMaxAndMin.ucVer = pSG3Para->ucG3Ver;
		getAxisMaxAndMin(&pSAssignConfig->SGetAxisMaxAndMin);

		pGCodeBufUnit->dHorMax = pSAssignConfig->SGetAxisMaxAndMin.dHorMax;
		pGCodeBufUnit->dHorMin = pSAssignConfig->SGetAxisMaxAndMin.dHorMin;
		pGCodeBufUnit->dVerMax = pSAssignConfig->SGetAxisMaxAndMin.dVerMax;
		pGCodeBufUnit->dVerMin = pSAssignConfig->SGetAxisMaxAndMin.dVerMin;

		//		
		pSG3Para->cStep = 0;

		return 2;
		break;
	}
	//赋值整圆1
	case 2:
	{
		//1.写圆弧段信息SArcInfo
		SArc SArcInfo;
		memset(&SArcInfo, 0, sizeof(SArc));

		SArcInfo.SStartPoint = pSG3Para->SG3StartPoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG3Para->SG3MiddlePoint;
		SArcInfo.SCenterPoint = pSG3Para->SG3CenterPoint;
		SArcInfo.dRadius = pSG3Para->dG3Radius;
		SArcInfo.cArcDir = pSG3Para->cG3Dir;
		SArcInfo.EntityType = ARC;

		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		//极值点
		pSAssignConfig->SGetAxisMaxAndMin.cArcDir = pGCodeBufUnit->cArcDir;
		pSAssignConfig->SGetAxisMaxAndMin.dRadius = pGCodeBufUnit->dRadius;
		pSAssignConfig->SGetAxisMaxAndMin.SStartPoint = &pGCodeBufUnit->SStartPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SEndPoint = &pGCodeBufUnit->SEndPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SCenterPoint = &pGCodeBufUnit->SCenterPoint;
		pSAssignConfig->SGetAxisMaxAndMin.ucHor = pSG3Para->ucG3Hor;
		pSAssignConfig->SGetAxisMaxAndMin.ucVer = pSG3Para->ucG3Ver;
		getAxisMaxAndMin(&pSAssignConfig->SGetAxisMaxAndMin);

		pGCodeBufUnit->dHorMax = pSAssignConfig->SGetAxisMaxAndMin.dHorMax;
		pGCodeBufUnit->dHorMin = pSAssignConfig->SGetAxisMaxAndMin.dHorMin;
		pGCodeBufUnit->dVerMax = pSAssignConfig->SGetAxisMaxAndMin.dVerMax;
		pGCodeBufUnit->dVerMin = pSAssignConfig->SGetAxisMaxAndMin.dVerMin;

		pSG3Para->cStep = 3;
		return 1;



	}
	//赋值整圆2
	case 3:
	{
		//1.写圆弧段信息SArcInfo
		SArc SArcInfo;
		memset(&SArcInfo, 0, sizeof(SArc));

		SArcInfo.SStartPoint = pSG3Para->SG3MiddlePoint;//系统状态结构体的当前点
		SArcInfo.SEndPoint = pSG3Para->SG3EndPoint;
		SArcInfo.SCenterPoint = pSG3Para->SG3CenterPoint;
		SArcInfo.dRadius = pSG3Para->dG3Radius;
		SArcInfo.cArcDir = pSG3Para->cG3Dir;
		SArcInfo.EntityType = ARC;

		//2.调用圆弧段赋值函数,写入缓冲区
		writeArcIntoGCodeBuf(&SArcInfo, pGCodeBufUnit);
		//极值点
		pSAssignConfig->SGetAxisMaxAndMin.cArcDir = pGCodeBufUnit->cArcDir;
		pSAssignConfig->SGetAxisMaxAndMin.dRadius = pGCodeBufUnit->dRadius;
		pSAssignConfig->SGetAxisMaxAndMin.SStartPoint = &pGCodeBufUnit->SStartPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SEndPoint = &pGCodeBufUnit->SEndPoint;
		pSAssignConfig->SGetAxisMaxAndMin.SCenterPoint = &pGCodeBufUnit->SCenterPoint;
		pSAssignConfig->SGetAxisMaxAndMin.ucHor = pSG3Para->ucG3Hor;
		pSAssignConfig->SGetAxisMaxAndMin.ucVer = pSG3Para->ucG3Ver;
		getAxisMaxAndMin(&pSAssignConfig->SGetAxisMaxAndMin);

		pGCodeBufUnit->dHorMax = pSAssignConfig->SGetAxisMaxAndMin.dHorMax;
		pGCodeBufUnit->dHorMin = pSAssignConfig->SGetAxisMaxAndMin.dHorMin;
		pGCodeBufUnit->dVerMax = pSAssignConfig->SGetAxisMaxAndMin.dVerMax;
		pGCodeBufUnit->dVerMin = pSAssignConfig->SGetAxisMaxAndMin.dVerMin;

		pSG3Para->cStep = 0;

		return 2;
	}
	default:
		break;
	}
}





