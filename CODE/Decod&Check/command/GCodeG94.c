#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"

static SG94Para* pSG94ParaCheck[CHANNEL_NUM_MAX];
static SG94Para* pSG94ParaAssign[CHANNEL_NUM_MAX];

// R在G20 下英寸转毫米，在G21 下直接保存毫米值
static void normalizeG94RToMm(SG94Para* pPara, bool bImperial)
{
    if (pPara->bRNeedsConvert == TRUE)
    {
        pPara->dR = bImperial ? pPara->dRRaw * 25.4 : pPara->dRRaw;
        pPara->bRNeedsConvert = FALSE;
    }
}


//添加G93规则结构体
void addG94RegulationAPI()
{
	//指令：G93
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(94, sGa, 1, mallocG94ParaStruct, freeG94ParaStruct, getG94Para, CheckG94, AssignG94);
}





/*************************************************
函数名称：int8 mallocG94ParaStruct(uint8 ucMode, uint8 ucChannelNum)
函数功能：开辟G94参数结构体（总框架）
输入参数：ucMode			模式（检查模式：0 赋值模式：1）
ucChannelNum	通道号
输出参数：
返 回 值：
备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
2.开辟后同时进行初始化
* *************************************************/
int8 mallocG94ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG94ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG94ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG94Para), ORAM_NUM450);//暂未对内存编号
		if (pSG94ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG94ParaCheck[ucChannelNum], 0, sizeof(SG94Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG94ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG94ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG94Para), ORAM_NUM451);
		if (pSG94ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG94ParaAssign[ucChannelNum], 0, sizeof(SG94Para));
		return 1;
	}
}


/*************************************************
  函数名称：int8 freeG94ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG94ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG94ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG94ParaCheck[ucChannelNum]);
			pSG94ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG94ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG94ParaAssign[ucChannelNum]);
			pSG94ParaAssign[ucChannelNum] = NULL;
		}
	}
}


/*************************************************
函数名称：int8 getG94Para(uint8 ucMode, SScan * pSScanContainer, uint8 ucChannelNum, SGCodeSysState * SGCodeCurrSysState)
函数功能：获取G94的参数
输入参数：ucMode			模式（检查模式：0 赋值模式：1）
           SScan * pSScanContainer    扫描结构体指针
		   SGCodeSysState * SGCodeCurrSysState  通道当前状态结构体指针
          ucChannelNum	通道号

输出参数：
返 回 值：
备    注：
**************************************************/
int8 getG94Para(uint8 ucMode, SScan * pSScanContainer, uint8 ucChannelNum, SGCodeSysState * SGCodeCurrSysState)
{

	if (ucMode == GC_SYNTAX)
	{
		pSG94ParaCheck[ucChannelNum]->cStep = 0;
		//1.获取R
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG94ParaCheck[ucChannelNum]->dRRaw = pSScanContainer->dRIJK[0];
			pSG94ParaCheck[ucChannelNum]->bRNeedsConvert = TRUE;

		}

		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG94ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);

	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG94ParaAssign[ucChannelNum]->cStep = 0;
		//1.获取R
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG94ParaAssign[ucChannelNum]->dRRaw = pSScanContainer->dRIJK[0];
			pSG94ParaAssign[ucChannelNum]->bRNeedsConvert = TRUE;
		}


		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG94ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);


	}
}



int8 CheckG94(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		//报错
		return -1;
	}
	normalizeG94RToMm(pSG94ParaCheck[pSSyntaxConfig->ucChannelNum], pSSyntaxConfig->SGCodeCurrSysState.bImperialProgramming); //R：当前为 G20 时将英寸换算为毫米，G21 时直接保存毫米值
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		//平面检查，当前只允许G18
		if (pGCodeBufUnit->SModalInfo.sGc != (int16)(G18 * pow(10, GCOMMAND_DECIMAL_PLACE)))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
			return -1;
		}

		//1.指令字符检查
		checkParaLetter("XYZUVWR", &pSSyntaxConfig->SScanContainer);

		//2.参数值范围检查
		checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('R', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);

		//3.当R与W的符号不同时，要求│R│≤│W│
		SPointDouble* pSG94PointA = NULL; //起点
		SPointDouble  pSG94PointC = { 0 };//切削终点
		double dG94R;//切削起点与切削终点Z 轴绝对坐标的差值，
		double dG94U;//切削终点与起点X 轴绝对坐标的差值；
		double dG94W;//切削终点与起点Z轴绝对坐标的差值

		bool bIsAbsMethod;
		pSG94PointA = &pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);

		//计算C点的机械坐标
		getMacCoordByCoordInfo(pSSyntaxConfig->ucChannelNum, &pSG94ParaCheck[pSSyntaxConfig->ucChannelNum]->SCoordInfo, pSG94PointA, &pSG94PointC, &pSSyntaxConfig->SGCodeCurrSysState);

		dG94R = pSG94ParaCheck[pSSyntaxConfig->ucChannelNum]->dR;

		dG94W = pSG94PointC.dArray[AXIS_Z] - pSG94PointA->dArray[AXIS_Z];
		//当 R 与 U 的符号不一致时
		if (dG94R * dG94W <= 0)
		{
			if (fabs(dG94W) < fabs(dG94R))
			{

				const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
				while (*pStartPtr != 'r' && *pStartPtr != 'R')
				{
					pStartPtr += 1;
				}
				setErrorType(GCODE_ERROR_NUM416);
				findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
				return -1;
			}
		}
		//6.赋值检查的最终终点
		pSSyntaxConfig->SCurrRecord.SEndPoint = *pSG94PointA;
		return 2;
	}
	return 2;
}


/*************************************************
  函数名称：int8 AssignG93(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG94(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SPointDouble* pSG94PointA = NULL;
	SPointDouble* pSG94PointC = NULL;
	double dG94R;
	pSG94PointA = &pSG94ParaAssign[pSAssignConfig->ucChannelNum]->SPointA;
	pSG94PointC = &pSG94ParaAssign[pSAssignConfig->ucChannelNum]->SPointC;
	normalizeG94RToMm(pSG94ParaAssign[pSAssignConfig->ucChannelNum],pSAssignConfig->SGCodeCurrSysState.bImperialProgramming);
	dG94R = pSG94ParaAssign[pSAssignConfig->ucChannelNum]->dR;


	switch (pSG94ParaAssign[pSAssignConfig->ucChannelNum]->cStep)
	{
	//0.赋值预处理（整理参数结构体数据）
	case 0:
	{
		int8 cRet = 0;
		bool bIsAbsMethod;
		//0.判断G94程序段中是否出现坐标信息
		cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
		//0.1程序段出现了坐标信息G93才赋值
		if (cRet == TRUE)
		{
			// G70重放直接进入赋值阶段，生成循环轨迹前必须检查本段生效平面。
			if (pGCodeBufUnit->SModalInfo.sGc != (int16)(G18 * pow(10, GCOMMAND_DECIMAL_PLACE)))
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
				return -1;
			}
			//获取G94的A点坐标
			*pSG94PointC=*pSG94PointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			//获取G94的C点坐标
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			//计算C点的机械坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum,&pSG94ParaAssign[pSAssignConfig->ucChannelNum]->SCoordInfo, pSG94PointA, pSG94PointC, &pSAssignConfig->SGCodeCurrSysState);

			pSG94ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 1;
		}
		//0.2程序段未出现坐标信息G93不赋值	
		else
		{
			return 2;
		}
		return 0;
		break;
	}

	//写入AB段 :G0
	case 1:
	{
		//1.填写SLine
		SLine SLineInfo_1;
		memset(&SLineInfo_1, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSG94ParaAssign[pSAssignConfig->ucChannelNum]->SPointA;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_1.SEndPoint.dArray[AXIS_Z] = pSG94PointC->dArray[AXIS_Z] + dG94R;

		SLineInfo_1.EntityType = OTHERTYPE;//G0
	//2.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_1, pGCodeBufUnit);

		pSG94ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 2;

		return 1;
	}


	case 2:
	{
		//1.填写SLine
		SLine SLineInfo_2;
		memset(&SLineInfo_2, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_2.SEndPoint = SLineInfo_2.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_2.SEndPoint = *pSG94PointC;

		SLineInfo_2.EntityType = L1NE;//G1

	//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_2, pGCodeBufUnit);

		pSG94ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 3;

		return 1;
	}
	
	//写入CD段 :G01
	case 3:
	{

		//1.填写SLine
		SLine SLineInfo_3;
		memset(&SLineInfo_3, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_3.SEndPoint.dArray[AXIS_Z] = pSG94PointA->dArray[AXIS_Z];

		SLineInfo_3.EntityType = L1NE;//G1

	//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);

		pSG94ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 4;

		return 1;
	}


	case 4:
	{
		//1.填写SLine
		SLine SLineInfo_4;
		memset(&SLineInfo_4, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_4.SEndPoint = *pSG94PointA;


		SLineInfo_4.EntityType = OTHERTYPE;//G0

	//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_4, pGCodeBufUnit);

		pSG94ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 0;

		return 2;
	}

	default:
		break;
	}




}



