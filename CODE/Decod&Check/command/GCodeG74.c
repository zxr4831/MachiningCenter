#include"NCDataPara.h"
#include "GeneralFunctions.h"
#include"ErrorFunction.h"
#include "Gcode.h"

static SG74Para* pSG74ParaCheck[CHANNEL_NUM_MAX];
static SG74Para* pSG74ParaAssign[CHANNEL_NUM_MAX];

//添加G74规则结构体
void addG74RegulationAPI()
{
	//指令：G74
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(74, sGz, 2, mallocG75ParaStruct, freeG75ParaStruct, getG75Para, CheckG75, AssignG75);
}

int8 mallocG74ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG74ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG74Para), ORAM_NUM424);//暂未对内存编号
		if (pSG74ParaCheck[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG74ParaCheck[ucChannelNum], 0, sizeof(SG74Para));


	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG74ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG74Para), ORAM_NUM425);
		if (pSG74ParaAssign[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG74ParaAssign[ucChannelNum], 0, sizeof(SG74Para));

	}

	return 1;

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
int8 freeG74ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG74ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG74ParaCheck[ucChannelNum]);
			pSG74ParaCheck[ucChannelNum] = NULL;
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG74ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG74ParaAssign[ucChannelNum]);
			pSG74ParaAssign[ucChannelNum] = NULL;
			return 1;
		}
	}
	return 0;
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
int8 getG74Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
	//1.检查
	if (ucMode == GC_SYNTAX)
	{
		SSyntaxConfiguration* pSSyntaxConfig = &pChannelConfig->SSyntaxConfig;
		SG74Para* pSG74Para = pSG74ParaCheck[pSSyntaxConfig->ucChannelNum];

		//1.1检查时，获取第一程序段的参数
		if (pSSyntaxConfig->cCommandSegmentCnt == 1)
		{
			//1.指令字符检查
			checkParaLetter("R", &pSSyntaxConfig->SScanContainer);

			//2.检查范围
			if (pSScanContainer->bRIJK[0] == TRUE)
			{
				if (pSScanContainer->dRIJK[0] < 0 || pSScanContainer->dRIJK[0]>99.999)
				{
					const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
					while (*pStartPtr != 'r' && *pStartPtr != 'R')
					{
						pStartPtr += 1;
					}
					setErrorType(GCODE_ERROR_NUM429);                          //G74 或 G75 中的单次退刀量 R(e) 超出允许范围
					findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
					return -1;
				}
			}
			return 1;

		}
		//1.2检查时，获取第二程序段的参数
		else if (pSSyntaxConfig->cCommandSegmentCnt == 2)
		{
			//1.指令字符检查
			checkParaLetter("XZUWPQR", &pSSyntaxConfig->SScanContainer);
			//2.范围检查
			checkParaRange('X', &pSScanContainer, -99999999, 99999999);
			checkParaRange('Z', &pSScanContainer, -99999999, 99999999);
			checkParaRange('U', &pSScanContainer, -99999999, 99999999);
			checkParaRange('W', &pSScanContainer, -99999999, 99999999);

			//3.G74要求出现Z
			if (pSScanContainer->bXYZ[2] == FALSE && pSScanContainer->bUVW[2] == FALSE)
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM425);   ////G74 指令中未输入 Z 的值 
				return -1;
			}
			if (pSScanContainer->bP == FALSE || isZero(pSScanContainer->dP))
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM428);   //G74 指令中 P 的值为 0 或未输入
				return -1;
			}

			//4.PQR范围检查
			if (pSScanContainer->bP == TRUE)
			{
				if (pSScanContainer->dP < 0 || pSScanContainer->dP>9999999 * TEMP_LEAST_INPUT_INCREMENT)
				{
					const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
					while (*pStartPtr != 'p' && *pStartPtr != 'P')
					{
						pStartPtr += 1;
					}
					setErrorType(GCODE_ERROR_NUM430);                           //G74 或 G75 中 X 或 Z 方向的单次切削量超出允许范围
					findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
					return -1;
				}
			}

			if (pSScanContainer->bQ == TRUE)
			{
				if (pSScanContainer->dQ < 0 || pSScanContainer->dQ> 9999999 * TEMP_LEAST_INPUT_INCREMENT)
				{
					const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
					while (*pStartPtr != 'q' && *pStartPtr != 'Q')
					{
						pStartPtr += 1;
					}
					setErrorType(GCODE_ERROR_NUM430);                           //G74 或 G75 中 X 或 Z 方向的单次切削量超出允许范围
					findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
					return -1;
				}
			}
			if (pSScanContainer->bRIJK[0] == TRUE)
			{
				if (pSScanContainer->dRIJK[0] < 0 || pSScanContainer->dRIJK[0] > 99999999 * TEMP_LEAST_INPUT_INCREMENT)
				{
					const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
					while (*pStartPtr != 'r' && *pStartPtr != 'R')
					{
						pStartPtr += 1;
					}
					setErrorType(GCODE_ERROR_NUM429);                           //G74 或 G75 中的单次退刀量R(Δd)超出允许范围
					findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
					return -1;
				}
			}
			return 1;
		}

	}
	//2.赋值
	else if (ucMode == GC_ASSIGN)
	{
		SAssignConfiguration* pSAssignConfig = &pChannelConfig->SAssignConfig;       //赋值结构体
		SG74Para* pSG74Para = pSG74ParaAssign[pSAssignConfig->ucChannelNum];
		//2.1赋值时，获取第一程序段的参数
		if (pSAssignConfig->cCommandSegmentCnt == 1)
		{
			if (pSScanContainer->bRIJK[0]==TRUE)
			{
				pSG74Para->dRetract_Z = pSAssignConfig->SScanContainer.dRIJK[0];//Z轴退刀量
			}		
		    else
		    {
			    pSG74Para->dRetract_Z = NC_DATA_PARAMETER_042;
		    }
			return 1;
		}
		//2.2赋值时，获取第二程序段的参数
		else if (pSAssignConfig->cCommandSegmentCnt == 2)
		{
			getCoordInfoFromScanInfo(&pSG74Para->SCoordInfo, &pSAssignConfig->SScanContainer);//存储坐标信息
			pSG74Para->dFeed_X = pSAssignConfig->SScanContainer.dP;//X轴进刀量
			pSG74Para->dFeed_Z = pSAssignConfig->SScanContainer.dQ;//Z轴进刀量

			pSG74Para->dRetract_X = pSAssignConfig->SScanContainer.dRIJK[0];//X轴退刀量
		}
	}

}

int8 CheckG74(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	return 2;
}

int8 AssignG74(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	SG74Para* pSG74Para = pSG74ParaAssign[pSAssignConfig->ucChannelNum];
	SPointDouble SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;


	switch (pSG74Para->cStep)
	{
	//0.赋值预处理（整理参数结构体数据）
	case 0:
	{
		//1.记录指令起点坐标
		pSG74Para->SStartPoint = SCurPoint;
		
		//2.记录指令终点坐标
		bool bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
		getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG74Para->SCoordInfo, &SCurPoint, &pSG74Para->SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
		
		//3.计算X,Z轴加工方向
		pSG74Para->cDir_X = pSG74Para->SEndPoint.dArray[AXIS_X] - SCurPoint.dArray[AXIS_X] > 0 ? 1 : -1;
		pSG74Para->cDir_Z = pSG74Para->SEndPoint.dArray[AXIS_Z] - SCurPoint.dArray[AXIS_Z] > 0 ? 1 : -1;

		//4.下一步Z轴进刀
		pSG74Para->cStep = 1;
		pSG74Para->bFirstFeed_Z = TRUE;

		return 0;
	}

	//1.Z轴进刀
	case 1:
	{
		bool bModify = FALSE;	//Z轴坐标修正标志

		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		if (pSG74Para->bFirstFeed_Z == TRUE)
		{
			//1.2.1首次进刀，进刀量为：dFeed_Z
			SLineInfo.SEndPoint.dArray[AXIS_Z] = SCurPoint.dArray[AXIS_Z] + pSG74Para->cDir_Z * pSG74Para->dFeed_Z;
			pSG74Para->bFirstFeed_Z = FALSE;
		}
		else
		{
			//1.2.2首次进刀，进刀量为：dFeed_Z + dRetract_Z
			SLineInfo.SEndPoint.dArray[AXIS_Z] = SCurPoint.dArray[AXIS_Z] + pSG74Para->cDir_Z * (pSG74Para->dFeed_Z + pSG74Para->dRetract_Z);
		}
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : L1NE;
		
		//2.修正Z轴进刀后的坐标
		if (isEqualOrGreaterThanZero(pSG74Para->cDir_Z * (SLineInfo.SEndPoint.dArray[AXIS_Z] - pSG74Para->SEndPoint.dArray[AXIS_Z])))
		{
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG74Para->SEndPoint.dArray[AXIS_Z];
			bModify = TRUE;
		}
		//3.写入轨迹
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//4.1下一步X轴退刀
		if (bModify == TRUE)
		{
			pSG74Para->cStep = 3;
		}
		//4.2下一步Z轴退刀
		else
		{
			pSG74Para->cStep = 2;
		}

		return 1;
	}
	//Z轴退刀
	case 2:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		memset(&SLineInfo, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_Z] = SCurPoint.dArray[AXIS_Z] - pSG74Para->cDir_Z * pSG74Para->dRetract_Z;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//3.下一步Z轴进刀
		pSG74Para->cStep = 1;

		return 1;

	}
	//X轴退刀
	case 3:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = SCurPoint.dArray[AXIS_X] - pSG74Para->cDir_X * pSG74Para->dRetract_X;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.写入轨迹
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//3.下一步Z轴回到起点
		pSG74Para->cStep = 5;

		return 1;
	}
	//X轴进刀
	case 4:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = SCurPoint.dArray[AXIS_X] + pSG74Para->cDir_X * (pSG74Para->dFeed_X + pSG74Para->dRetract_X);
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.修正X轴进刀后的坐标
		if (isEqualOrGreaterThanZero(pSG74Para->cDir_X * (SLineInfo.SEndPoint.dArray[AXIS_X] - pSG74Para->SEndPoint.dArray[AXIS_X])))
		{
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG74Para->SEndPoint.dArray[AXIS_X];
			pSG74Para->bLastFeed = TRUE;
		}
		//3.写入轨迹
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//4.下一步Z轴进刀
		pSG74Para->cStep = 1;
		pSG74Para->bFirstFeed_Z = TRUE;


		return 1;
	}
	//Z轴回到起点
	case 5:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		memset(&SLineInfo, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG74Para->SStartPoint.dArray[AXIS_Z];
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//3.1下一步X轴进刀
		if (pSG74Para->bLastFeed == FALSE)
		{
			pSG74Para->cStep = 4;
		}
		//3.2下一步X轴回到起点
		else
		{
			pSG74Para->cStep = 6;
		}

		return 1;

	}
	//X轴回到起点
	case 6:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		memset(&SLineInfo, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG74Para->SStartPoint.dArray[AXIS_X];
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//3.下一步X轴进刀
		pSG74Para->cStep = 4;

		//4.指令轨迹全部写入完成
		return 2;
	}

	default:
		break;
	}



}
