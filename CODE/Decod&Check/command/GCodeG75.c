#include "GeneralFunctions.h"
#include"ErrorFunction.h"
#include "GCode.h"
#include"NCDataPara.h"
/*
指令格式： G75 R(e)；
G75 X(U)_ Z(W)_ P(Δ i) Q(Δ k) R(Δ d) F_ ；

相关定义：
1.R(e)：每次径向(X 轴)进刀后的径向退刀量，取值范围0～99.999(单位：mm，半径值)，无符号。
        R(e)执行后指定值保持有效，未输入R(e)时，以数据参数042号的值作为径向退刀量。
2.X：切削终点Bf 的X 轴绝对坐标值。
  U：切削终点Bf 与起点A 的X 轴绝对坐标的差值。
  Z：切削终点Bf 的Z 轴的绝对坐标值。
  W：切削终点Bf 与起点A 的Z 轴绝对坐标的差值
3.P(Δ i) ：径向(X 轴)进刀时，X 轴断续进刀的进刀量，取值范围0＜Δ i≤9999999（单位：mm，无符号,原文为um）。
4.Q(Δ k)：单次径向切削循环的轴向(Z 轴)进刀量，取值范围0＜Δ k≤ 9999999（单位：最小输入增量，无符号，原文为um）。
5.R(Δ d)：切削至径向切削终点后，轴向(Z 轴)的退刀量，取值范围0～99999999×最小输入增量，（单位：mm/inch，无符号）。
   省略R(Δ d)时，系统默认径向切削终点后，轴向(Z 轴)的退刀量为0。

*/

static SG75Para* pSG75ParaCheck[CHANNEL_NUM_MAX];
static SG75Para* pSG75ParaAssign[CHANNEL_NUM_MAX];

// 添加G75规则结构体
void addG75RegulationAPI()
{
	//指令：G75
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(75, sGz, 2, mallocG75ParaStruct, freeG75ParaStruct, getG75Para, CheckG75, AssignG75);
}

//开辟G75参数结构体
int8 mallocG75ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG75ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG75Para), ORAM_NUM426);//暂未对内存编号
		if (pSG75ParaCheck[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG75ParaCheck[ucChannelNum], 0, sizeof(SG75Para));


	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG75ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG75Para), ORAM_NUM427);
		if (pSG75ParaAssign[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG75ParaAssign[ucChannelNum], 0, sizeof(SG75Para));

	}

	return 1;

}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：释放G75参数结构体
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG75ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG75ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG75ParaCheck[ucChannelNum]);
			pSG75ParaCheck[ucChannelNum] = NULL;
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG75ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG75ParaAssign[ucChannelNum]);
			pSG75ParaAssign[ucChannelNum] = NULL;
			return 1;
		}
	}
	return 0;
}


/*************************************************
  函数名称：int8 getG75Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能：获取G76参数
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
			SScan* pSScanContainer  扫描结构体
			SGCodeSysState* SGCodeCurrSysState  通道状态结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG75Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);//获取通道结构体
	//检查
	if (ucMode == GC_SYNTAX)
	{
		SSyntaxConfiguration* pSSyntaxConfig = &pChannelConfig->SSyntaxConfig;
		SG75Para* pSG75Para = pSG75ParaCheck[pSSyntaxConfig->ucChannelNum];

		//G75 R(e) H；
		//1.1检查时，获取第一程序段的参数
		if (pSSyntaxConfig->cCommandSegmentCnt == 1)
		{
			//1.指令字符检查
			checkParaLetter("R", &pSSyntaxConfig->SScanContainer);
			//2.检查范围
			if (pSScanContainer->bRIJK[0]==TRUE)
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
			//检查输入的H指令
			if (pSScanContainer->bH == TRUE)
			{
				pSScanContainer->bA2Z['H' - 65] = TRUE; //添加H指令参数范围检查
				if ((checkParaRange('H', &pSSyntaxConfig->SScanContainer, 0, 2) == -1) || (checkParaInteger('H', &pSSyntaxConfig->SScanContainer) == -1))
				{
					return -1;
				}
			}
			return 1;
		}
		//1.2检查时，获取第二程序段的参数 
		//G75 X(U)_ Z(W)_ P(Δ i) Q(Δ k) R(Δ d) ；
		else if (pSSyntaxConfig->cCommandSegmentCnt == 2)
		{
			//1.指令字符检查
			checkParaLetter("XZUWPQR", &pSSyntaxConfig->SScanContainer);
			//2.范围检查
			checkParaRange('X', pSScanContainer, -99999999, 99999999);
			checkParaRange('Z', pSScanContainer, -99999999, 99999999);
			checkParaRange('U', pSScanContainer, -99999999, 99999999);
			checkParaRange('W', pSScanContainer, -99999999, 99999999);

			//3. G74 轴向切削要求 Z/W、Q；G75 径向切削要求 X/U、P
			bool bG74 = pSSyntaxConfig->sCurrCommandVal == G74 * pow(10, GCOMMAND_DECIMAL_PLACE);
			if (bG74)
			{
				if (pSScanContainer->bXYZ[AXIS_Z] == FALSE&& pSScanContainer->bUVW[AXIS_Z] == FALSE)
				{
					parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM425);
					return -1;
				}

				if (pSScanContainer->bQ == FALSE || isZero(pSScanContainer->dQ))
				{
					parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM426);
					return -1;
				}
			}
			else
			{
				/* G75 必填约束。 */
				if (pSScanContainer->bXYZ[AXIS_X] == FALSE&& pSScanContainer->bUVW[AXIS_X] == FALSE)
				{
					parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM427);
					return -1;
				}

				if (pSScanContainer->bP == FALSE|| isZero(pSScanContainer->dP))
				{
					parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM428);
					return -1;
				}
			}

			//4.PQR范围检查
			if (pSScanContainer->bP == TRUE)
			{
				if (!isfinite(pSScanContainer->dP) || pSScanContainer->dP < 0 || pSScanContainer->dP>9999999 * TEMP_LEAST_INPUT_INCREMENT)
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

			if (pSScanContainer->bQ==TRUE)
			{
				if (!isfinite(pSScanContainer->dQ) || pSScanContainer->dQ<0||pSScanContainer->dQ> 9999999 * TEMP_LEAST_INPUT_INCREMENT)
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
			if (pSScanContainer->bRIJK[0]==TRUE)
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
			if (pSScanContainer->bRIJK[3] == TRUE)
			{
				pSScanContainer->bA2Z['K' - 65] = TRUE; //添加H指令参数范围检查
				if (checkParaInteger('K', &pSSyntaxConfig->SScanContainer) == -1)
				{
					return -1;
				}
				pSG75Para->cTotalLayNum = pSScanContainer->dRIJK[3];
			}
			return 1;
		}
	}
	//赋值
	else if (ucMode == GC_ASSIGN)
	{
		SAssignConfiguration* pSAssignConfig = &pChannelConfig->SAssignConfig;       //赋值结构体
		SG75Para* pSG75Para = pSG75ParaAssign[pSAssignConfig->ucChannelNum];
		//1.赋值时，获取第一程序段的参数
		if (pSAssignConfig->cCommandSegmentCnt == 1)
		{
			//1.1判断是否为G74
			pSG75Para->bG74 = pSAssignConfig->sCurrCommandVal / pow(10, GCOMMAND_DECIMAL_PLACE) == G74 ? 1 : 0;

			//1.2获取X轴退刀量
			if (pSScanContainer->bRIJK[0] == TRUE)
			{
				pSG75Para->dRetract_X = pSScanContainer->dRIJK[0] / 2;
			}
			else
			{
				pSG75Para->dRetract_X = NC_DATA_PARAMETER_042;
			}

			//1.3加工层数 K
			if (pSScanContainer->bRIJK[3] == TRUE)
			{
				pSG75Para->cTotalLayNum = pSScanContainer->dRIJK[3];
			}

			pSG75Para->ucH = (uint8)pSScanContainer->dH;
			//检查输入的H指令
			if (pSScanContainer->bH == TRUE)
			{
				pSScanContainer->bA2Z['H' - 65] = TRUE; //添加H指令参数范围检查
				if ((checkParaRange('H', &pSAssignConfig->SScanContainer, 0, 2) == -1) || (checkParaInteger('H', &pSAssignConfig->SScanContainer) == -1))
				{
					return -1;
				}
			}
			return 1;
		}
		//2.赋值时，获取第二程序段的参数
		else if (pSAssignConfig->cCommandSegmentCnt == 2)
		{
			//2.1  存储坐标信息
			getCoordInfoFromScanInfo(&pSG75Para->SCoordInfo, pSScanContainer);

			//2.2.2 获取X轴进刀量P(Δ i)
			if (pSScanContainer->bP == TRUE)
			{
				if (pSG75Para->bG74)
				{
					pSG75Para->dFeed_Z = pSScanContainer->dP / 2;//Z轴进刀量
					if (g_SSystemPara.bSetG7456mmTOum == TRUE)
					{
						pSG75Para->dFeed_Z *= 0.001;
					}
				}
				else
				{
					pSG75Para->dFeed_X = pSScanContainer->dP / 2;//X轴进刀量
					if (g_SSystemPara.bSetG7456mmTOum == TRUE)
					{
						pSG75Para->dFeed_X *= 0.001;
					}
				}
			}
			//2.2.3 获取Z轴进刀量Q(Δ k)：
			if (pSScanContainer->bQ == TRUE)
			{
				if (pSG75Para->bG74)
				{
					pSG75Para->dFeed_X = pSScanContainer->dQ;//X轴进刀量
					if (g_SSystemPara.bSetG7456mmTOum == TRUE)
					{
						pSG75Para->dFeed_X *= 0.001;
					}
				}
				else
				{
					pSG75Para->dFeed_Z = pSScanContainer->dQ;//Z
					if (g_SSystemPara.bSetG7456mmTOum == TRUE)
					{
						pSG75Para->dFeed_Z *= 0.001;
					}
				}

			}

			//2.2.4 获取Z轴退刀量R(Δ d)：
			if (pSScanContainer->bRIJK[0] == TRUE)
			{
				pSG75Para->dRetract_Z = pSScanContainer->dRIJK[0];
			}

			//2.2.5 初始化赋值步骤
			pSG75Para->cStep = 0;

			return 1;


		}
	}
}

int8 AssignG75(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig==NULL||pSGCodeParseUnit==NULL)
	{
		return -1;
	}

	int8 cRet = 0;
	SG75Para* pSG75Para = pSG75ParaAssign[pSAssignConfig->ucChannelNum];
	SPointDouble SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
	//G74交换系统状态当前点XZ坐标
	if (pSG75Para->bG74 && pSG75Para->cStep != 0)
	{
		double dTemp;
		dTemp = SCurPoint.dArray[AXIS_X];
		SCurPoint.dArray[AXIS_X] = SCurPoint.dArray[AXIS_Z];
		SCurPoint.dArray[AXIS_Z] = dTemp;
	}
	
	//Ⅰ型
	if (pSG75Para->ucH == 0)
	{
		switch (pSG75Para->cStep)
		{
		//0.G73赋值预处理
		case 0:
		{
			//1.计算起点终点机械坐标
			pSG75Para->SStartPoint = pSG75Para->SEndPoint = SCurPoint;
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG75Para->SCoordInfo, &SCurPoint, &pSG75Para->SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
			if (pSG75Para->bG74 == TRUE)
			{
				double dTemp;
				dTemp = pSG75Para->SStartPoint.dArray[AXIS_X];
				pSG75Para->SStartPoint.dArray[AXIS_X] = pSG75Para->SStartPoint.dArray[AXIS_Z];
				pSG75Para->SStartPoint.dArray[AXIS_Z] = dTemp;

				dTemp = pSG75Para->SEndPoint.dArray[AXIS_X];
				pSG75Para->SEndPoint.dArray[AXIS_X] = pSG75Para->SEndPoint.dArray[AXIS_Z];
				pSG75Para->SEndPoint.dArray[AXIS_Z] = dTemp;
			}
			////行程与步距一致性判断
			//if (pSG75Para->bG74)
			{
				/* G74 交换 X/Z 后，内部 Z 对应实际径向。 */
				if (isZero(pSG75Para->SEndPoint.dArray[AXIS_Z]- pSG75Para->SStartPoint.dArray[AXIS_Z]))
				{
					/* 实际没有径向行程，只加工当前这一列。 */
					pSG75Para->dFeed_Z = 0;
				}
				else if (!isfinite(pSG75Para->dFeed_Z)|| pSG75Para->dFeed_Z <= 0|| isZero(pSG75Para->dFeed_Z))
				{
					/* 有径向行程时，必须有有效的换列步距 */
					parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM430);
					return -1;
				}
			}
			pSG75Para->cStep = 1;
		}
		
		//1.获取循环计算参数
		case 1:
		{
			pSG75Para->SG75LoopPara.cStepInLoop = 0;

			pSG75Para->SG75LoopPara.dSX = pSG75Para->SStartPoint.dArray[AXIS_X];
			pSG75Para->SG75LoopPara.dSZ = pSG75Para->SStartPoint.dArray[AXIS_Z];
			pSG75Para->SG75LoopPara.dEX = pSG75Para->SEndPoint.dArray[AXIS_X];
			pSG75Para->SG75LoopPara.dEZ = pSG75Para->SEndPoint.dArray[AXIS_Z];

			pSG75Para->SG75LoopPara.dFeed_X = pSG75Para->dFeed_X;
			pSG75Para->SG75LoopPara.dFeed_Z = pSG75Para->dFeed_Z;
			pSG75Para->SG75LoopPara.dRetract_X = pSG75Para->dRetract_X;
			pSG75Para->SG75LoopPara.dRetract_Z = pSG75Para->dRetract_Z;

			pSG75Para->SG75LoopPara.bFirstFeed_X = TRUE;
			pSG75Para->SG75LoopPara.bLastFeed = FALSE;

			pSG75Para->SG75LoopPara.cDir_X = sgn(pSG75Para->SG75LoopPara.dEX - pSG75Para->SG75LoopPara.dSX);
			pSG75Para->SG75LoopPara.cDir_Z = sgn(pSG75Para->SG75LoopPara.dEZ - pSG75Para->SG75LoopPara.dSZ);

			pSG75Para->cStep = 2;

			cRet = 0;
			break;
		}
		//2.循环计算
		case 2:
		{
			//判断循环计算是否完成
			if (calcG75Loop(&pSG75Para->SG75LoopPara, SCurPoint, pSGCodeParseUnit) == 2)
			{
				pSG75Para->cStep = 3;
			}
			cRet = 1;
			break;
		}
		//3.X轴退刀
		case 3:
		{
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->SStartPoint.dArray[AXIS_X];
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;
			
			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			pSG75Para->cStep = 4;

			cRet = 1;
			break;
		}
		//4.Z轴退刀
		case 4:
		{		
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG75Para->SStartPoint.dArray[AXIS_Z];
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			cRet = 2;
			break;
		}
		}
	}
	//Ⅱ型
	else if (pSG75Para->ucH == 1)
	{
		switch (pSG75Para->cStep)
		{
		//0.G73赋值预处理
		case 0:
		{
			//1.计算起点终点机械坐标
			pSG75Para->SStartPoint = pSG75Para->SEndPoint = SCurPoint;
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG75Para->SCoordInfo, &SCurPoint, &pSG75Para->SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
			if (pSG75Para->bG74 == TRUE)
			{
				double dTemp;
				dTemp = pSG75Para->SStartPoint.dArray[AXIS_X];
				pSG75Para->SStartPoint.dArray[AXIS_X] = pSG75Para->SStartPoint.dArray[AXIS_Z];
				pSG75Para->SStartPoint.dArray[AXIS_Z] = dTemp;

				dTemp = pSG75Para->SEndPoint.dArray[AXIS_X];
				pSG75Para->SEndPoint.dArray[AXIS_X] = pSG75Para->SEndPoint.dArray[AXIS_Z];
				pSG75Para->SEndPoint.dArray[AXIS_Z] = dTemp;
			}
			////行程与步距一致性判断
			//if (pSG75Para->bG74)
			{
				/* G74 交换 X/Z 后，内部 Z 对应实际径向。 */
				if (isZero(pSG75Para->SEndPoint.dArray[AXIS_Z]- pSG75Para->SStartPoint.dArray[AXIS_Z]))
				{
					/* 实际没有径向行程，只加工当前这一列。 */
					pSG75Para->dFeed_Z = 0;
				}
				else if (!isfinite(pSG75Para->dFeed_Z)|| pSG75Para->dFeed_Z <= 0|| isZero(pSG75Para->dFeed_Z))
				{
					/* 有径向行程时，必须有有效的换列步距。 */
					parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM430);
					return -1;
				}
			}

			pSG75Para->cCommandDir_X = sgn(pSG75Para->SEndPoint.dArray[AXIS_X] - pSG75Para->SStartPoint.dArray[AXIS_X]);
			pSG75Para->cCommandDir_Z = sgn(pSG75Para->SEndPoint.dArray[AXIS_Z] - pSG75Para->SStartPoint.dArray[AXIS_Z]);

			if (pSG75Para->cTotalLayNum <= 0)
			{
				cRet = -1;
				break;
			}
			//2.层参数计算
			pSG75Para->cCurrLayNum = 1;
			pSG75Para->dLayerDepth = fabs(pSG75Para->SEndPoint.dArray[AXIS_X] - pSG75Para->SStartPoint.dArray[AXIS_X]) / pSG75Para->cTotalLayNum;
			pSG75Para->dCurrLayStartX = pSG75Para->cCurrLayNum == 1 ? pSG75Para->SStartPoint.dArray[AXIS_X] : (pSG75Para->cCurrLayNum - 1) * pSG75Para->dLayerDepth;
			pSG75Para->dCurrLayEndX = pSG75Para->SStartPoint.dArray[AXIS_X] + pSG75Para->cCommandDir_X * pSG75Para->dLayerDepth;
			
			pSG75Para->cStep = 1;
		}
		//1.获取循环计算参数
		case 1:
		{
			pSG75Para->SG75LoopPara.cStepInLoop = 0;

			pSG75Para->SG75LoopPara.dSX = pSG75Para->dCurrLayStartX;
			pSG75Para->SG75LoopPara.dSZ = pSG75Para->SStartPoint.dArray[AXIS_Z];
			pSG75Para->SG75LoopPara.dEX = pSG75Para->dCurrLayEndX;
			pSG75Para->SG75LoopPara.dEZ = pSG75Para->SEndPoint.dArray[AXIS_Z];

			pSG75Para->SG75LoopPara.dFeed_X = pSG75Para->dFeed_X;
			pSG75Para->SG75LoopPara.dFeed_Z = pSG75Para->dFeed_Z;
			pSG75Para->SG75LoopPara.dRetract_X = pSG75Para->dRetract_X;
			pSG75Para->SG75LoopPara.dRetract_Z = pSG75Para->dRetract_Z;

			pSG75Para->SG75LoopPara.bFirstFeed_X = TRUE;
			pSG75Para->SG75LoopPara.bLastFeed = FALSE;

			pSG75Para->SG75LoopPara.cDir_X = sgn(pSG75Para->SG75LoopPara.dEX - pSG75Para->SG75LoopPara.dSX);
			pSG75Para->SG75LoopPara.cDir_Z = sgn(pSG75Para->SG75LoopPara.dEZ - pSG75Para->SG75LoopPara.dSZ);

			pSG75Para->cStep = 2;

			cRet = 0;
			break;
		}
		//2.循环计算
		case 2:
		{
			//判断循环计算是否完成
			if (calcG75Loop(&pSG75Para->SG75LoopPara, SCurPoint, pSGCodeParseUnit) == 2)
			{
				pSG75Para->cStep = 3;
			}
			cRet = 1;
			break;
		}
		//3.层参数计算
		case 3:
		{
			pSG75Para->cCurrLayNum++;
			//所有层加工完成
			if (pSG75Para->cCurrLayNum > pSG75Para->cTotalLayNum)
			{
				//下一步X轴退刀至指令起点
				pSG75Para->cStep = 7;
			}
			//未加工完成，计算下一层参数
			else
			{
				pSG75Para->dCurrLayStartX = pSG75Para->SStartPoint.dArray[AXIS_X] + pSG75Para->SG75LoopPara.cDir_X * (pSG75Para->cCurrLayNum - 1) * pSG75Para->dLayerDepth;
				pSG75Para->dCurrLayEndX = pSG75Para->SStartPoint.dArray[AXIS_X] + pSG75Para->SG75LoopPara.cDir_X * pSG75Para->cCurrLayNum * pSG75Para->dLayerDepth;

				//下一步X轴退刀至层起点
				pSG75Para->cStep = 4;
			}
			cRet = 0;
			break;
		}
		//4.定位至下一层，X轴退刀至层起点
		case 4:
		{
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->dCurrLayStartX - pSG75Para->SG75LoopPara.cDir_X * pSG75Para->dRetract_X;
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			pSG75Para->cStep = 5;

			cRet = 1;
			break;
		}
		//5.定位至下一层，Z轴退刀至层起点
		case 5:
		{
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG75Para->SStartPoint.dArray[AXIS_Z];
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			pSG75Para->cStep = 6;

			cRet = 1;
			break;
		}
		//6.定位至下一层，X轴进刀至层起点
		case 6:
		{
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->dCurrLayStartX;
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : L1NE;

			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//下一步获取循环计算参数
			pSG75Para->cStep = 1;

			cRet = 1;
			break;

		}
		//3.X轴退刀至指令起点
		case 7:
		{
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->SStartPoint.dArray[AXIS_X];
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//下一步Z轴退刀至指令起点
			pSG75Para->cStep = 8;

			cRet = 1;
			break;
		}
		//4.Z轴退刀至指令起点
		case 8:
		{
			//1.填写SLine
			SLine SLineInfo = { 0 };
			//1.1初始化SLine的起点终点（机械坐标）
			SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
			//1.2获取SLine的终点（机械坐标）
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG75Para->SStartPoint.dArray[AXIS_Z];
			//1.3获取SLine的实体类型
			SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

			//2.将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			cRet = 2;
			break;
		}
		}
	}
	
	//Ⅲ型
	else if (pSG75Para->ucH == 2)
	{
	switch (pSG75Para->cStep)
	{
	//0.G75赋值预处理
	case 0:
	{
		//1.计算起点终点机械坐标
		pSG75Para->SStartPoint = pSG75Para->SEndPoint = SCurPoint;
		getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG75Para->SCoordInfo, &SCurPoint, &pSG75Para->SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
		if (pSG75Para->bG74 == TRUE)
		{
			double dTemp;
			dTemp = pSG75Para->SStartPoint.dArray[AXIS_X];
			pSG75Para->SStartPoint.dArray[AXIS_X] = pSG75Para->SStartPoint.dArray[AXIS_Z];
			pSG75Para->SStartPoint.dArray[AXIS_Z] = dTemp;

			dTemp = pSG75Para->SEndPoint.dArray[AXIS_X];
			pSG75Para->SEndPoint.dArray[AXIS_X] = pSG75Para->SEndPoint.dArray[AXIS_Z];
			pSG75Para->SEndPoint.dArray[AXIS_Z] = dTemp;
		}
		////行程与步距一致性判断
		//if (pSG75Para->bG74)
		{
			/* G74 交换 X/Z 后，内部 Z 对应实际径向。 */
			if (isZero(pSG75Para->SEndPoint.dArray[AXIS_Z]- pSG75Para->SStartPoint.dArray[AXIS_Z]))
			{
				/* 实际没有径向行程，只加工当前这一列。 */
				pSG75Para->dFeed_Z = 0;
			}
			else if (!isfinite(pSG75Para->dFeed_Z)|| pSG75Para->dFeed_Z <= 0|| isZero(pSG75Para->dFeed_Z))
			{
				/* 有径向行程时，必须有有效的换列步距。 */
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM430);
				return -1;
			}
		}

		pSG75Para->cCommandDir_X = sgn(pSG75Para->SEndPoint.dArray[AXIS_X] - pSG75Para->SStartPoint.dArray[AXIS_X]);
		pSG75Para->cCommandDir_Z = sgn(pSG75Para->SEndPoint.dArray[AXIS_Z] - pSG75Para->SStartPoint.dArray[AXIS_Z]);

		pSG75Para->cCurrMachineDir = pSG75Para->cCommandDir_X;
		if (pSG75Para->cTotalLayNum <= 0)
		{
			cRet = -1;
			break;
		}
		//2.层参数计算
		pSG75Para->cCurrLayNum = 1;
		pSG75Para->dLayerDepth = fabs(pSG75Para->SEndPoint.dArray[AXIS_X] - pSG75Para->SStartPoint.dArray[AXIS_X]) / pSG75Para->cTotalLayNum;
		pSG75Para->dCurrLayStartX = pSG75Para->cCurrLayNum == 1 ? pSG75Para->SStartPoint.dArray[AXIS_X] : (pSG75Para->cCurrLayNum - 1) * pSG75Para->dLayerDepth;
		pSG75Para->dCurrLayEndX = pSG75Para->SStartPoint.dArray[AXIS_X] + pSG75Para->cCommandDir_X * pSG75Para->dLayerDepth;

		pSG75Para->cStep = 1;
	}
	//1.获取循环计算参数
	case 1:
	{
		pSG75Para->SG75LoopPara.cStepInLoop = 0;

		pSG75Para->SG75LoopPara.dSX = pSG75Para->dCurrLayStartX;
		pSG75Para->SG75LoopPara.dSZ = pSG75Para->cCurrMachineDir == pSG75Para->cCommandDir_X ? pSG75Para->SStartPoint.dArray[AXIS_Z] : pSG75Para->SEndPoint.dArray[AXIS_Z];
		pSG75Para->SG75LoopPara.dEX = pSG75Para->dCurrLayEndX;
		pSG75Para->SG75LoopPara.dEZ = pSG75Para->cCurrMachineDir == pSG75Para->cCommandDir_X ? pSG75Para->SEndPoint.dArray[AXIS_Z] : pSG75Para->SStartPoint.dArray[AXIS_Z];

		pSG75Para->SG75LoopPara.dFeed_X = pSG75Para->dFeed_X;
		pSG75Para->SG75LoopPara.dFeed_Z = pSG75Para->dFeed_Z;
		pSG75Para->SG75LoopPara.dRetract_X = pSG75Para->dRetract_X;
		pSG75Para->SG75LoopPara.dRetract_Z = pSG75Para->dRetract_Z;

		pSG75Para->SG75LoopPara.bFirstFeed_X = TRUE;
		pSG75Para->SG75LoopPara.bLastFeed = FALSE;

		pSG75Para->SG75LoopPara.cDir_X = sgn(pSG75Para->SG75LoopPara.dEX - pSG75Para->SG75LoopPara.dSX);
		pSG75Para->SG75LoopPara.cDir_Z = sgn(pSG75Para->SG75LoopPara.dEZ - pSG75Para->SG75LoopPara.dSZ);

		pSG75Para->cStep = 2;

		cRet = 0;
		break;
	}
	//2.循环计算
	case 2:
	{
		//判断循环计算是否完成
		if (calcG75Loop(&pSG75Para->SG75LoopPara, SCurPoint, pSGCodeParseUnit) == 2)
		{
			pSG75Para->cStep = 3;
		}
		cRet = 1;
		break;
	}
	//3.层参数计算
	case 3:
	{
		pSG75Para->cCurrLayNum++;
		//所有层加工完成
		if (pSG75Para->cCurrLayNum > pSG75Para->cTotalLayNum)
		{
			//下一步X轴退刀至指令起点
			pSG75Para->cStep = 7;
		}
		//未加工完成，计算下一层参数
		else
		{
			pSG75Para->dCurrLayStartX = pSG75Para->SStartPoint.dArray[AXIS_X] + pSG75Para->SG75LoopPara.cDir_X * (pSG75Para->cCurrLayNum - 1) * pSG75Para->dLayerDepth;
			pSG75Para->dCurrLayEndX = pSG75Para->SStartPoint.dArray[AXIS_X] + pSG75Para->SG75LoopPara.cDir_X * pSG75Para->cCurrLayNum * pSG75Para->dLayerDepth;

			//下一步X轴退刀至层起点
			pSG75Para->cStep = 4;
		}
		cRet = 0;
		break;
	}
	//4.定位至下一层，X轴退刀至层起点
	case 4:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->dCurrLayStartX - pSG75Para->SG75LoopPara.cDir_X * pSG75Para->dRetract_X;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		pSG75Para->cStep = 5;

		cRet = 1;
		break;
	}
	//5.定位至下一层，Z轴退刀至层起点/终点
	case 5:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG75Para->cCurrMachineDir == pSG75Para->cCommandDir_X ? pSG75Para->SEndPoint.dArray[AXIS_Z] : pSG75Para->SStartPoint.dArray[AXIS_Z];
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		pSG75Para->cStep = 6;

		cRet = 1;
		break;
	}
	//6.定位至下一层，X轴进刀至层起点
	case 6:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->dCurrLayStartX;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : L1NE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//3.当前加工方向取反
		pSG75Para->cCurrMachineDir *= -1;

		//下一步获取循环计算参数
		pSG75Para->cStep = 1;

		cRet = 1;
		break;

	}
	//3.X轴退刀至指令起点
	case 7:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75Para->SStartPoint.dArray[AXIS_X];
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//下一步Z轴退刀至指令起点
		pSG75Para->cStep = 8;

		cRet = 1;
		break;
	}
	//4.Z轴退刀至指令起点
	case 8:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG75Para->SStartPoint.dArray[AXIS_Z];
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		cRet = 2;
		break;
	}
	}
	}

	//
	if (pSG75Para->bG74 && cRet != 0)
	{
		swapBufXZCoord(pSGCodeParseUnit);
	}

	return cRet;
}



int8 CheckG75(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	return 2;
}

int8 calcG75Loop(SG75LoopPara* pSG75LoopPara, SPointDouble SCurPoint, struct SGCodeParse* pSGCodeParseUnit)
{
	switch (pSG75LoopPara->cStepInLoop)
	{
	//0.X进刀
	case 0:
	{
		bool bModify = FALSE;	//X轴坐标修正标志

		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		if (pSG75LoopPara->bFirstFeed_X == TRUE)
		{
			//1.2.1首次进刀，进刀量为：dFeed_X
			SLineInfo.SEndPoint.dArray[AXIS_X] = SCurPoint.dArray[AXIS_X] + pSG75LoopPara->cDir_X * pSG75LoopPara->dFeed_X;
			pSG75LoopPara->bFirstFeed_X = FALSE;
		}
		else
		{
			//1.2.2非首次进刀，进刀量为：dFeed_X + dRetract_X
			SLineInfo.SEndPoint.dArray[AXIS_X] = SCurPoint.dArray[AXIS_X] + pSG75LoopPara->cDir_X * (pSG75LoopPara->dFeed_X + pSG75LoopPara->dRetract_X);
		}
		//1.3修正X轴进刀后的坐标（终点超出循环终点X）
		if (isEqualOrGreaterThanZero(pSG75LoopPara->cDir_X * (SLineInfo.SEndPoint.dArray[AXIS_X] - pSG75LoopPara->dEX)))
		{
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75LoopPara->dEX;
			bModify = TRUE;
		}
		//1.4获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : L1NE;

		//1.5写入轨迹
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		if ((bModify == TRUE) && (pSG75LoopPara->dFeed_Z == 0))
		{

			return 2;
		}

		//2.判断下一步
		if (bModify)
			pSG75LoopPara->cStepInLoop = 3;	//下一步为Z轴退刀
		//2.2 下一步为X轴退刀
		else
			pSG75LoopPara->cStepInLoop = 1;	//下一步为X轴退刀

		return 1;
	}
	//1.X退刀
	case 1:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = SCurPoint.dArray[AXIS_X] - pSG75LoopPara->cDir_X * pSG75LoopPara->dRetract_X;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;
		//1.4.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//2.下一步X轴进刀
		pSG75LoopPara->cStepInLoop = 0;

		return 1;
	}
	//2.Z进刀
	case 2:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_Z] = SCurPoint.dArray[AXIS_Z] + pSG75LoopPara->cDir_Z * (pSG75LoopPara->dRetract_Z + pSG75LoopPara->dFeed_Z);
		//1.3 修正终点坐标
		//判断Z轴进刀是否超过或到达最后终点的Z坐标
		//如果到达或者超过
		if (isEqualOrGreaterThanZero(pSG75LoopPara->cDir_Z * (SLineInfo.SEndPoint.dArray[AXIS_Z] - pSG75LoopPara->dEZ)))
		{
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG75LoopPara->dEZ;
			pSG75LoopPara->bLastFeed = TRUE;
		}

		//1.4获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//1.5.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//2.判断下一步
		pSG75LoopPara->cStepInLoop = 0;//X轴进刀
		pSG75LoopPara->bFirstFeed_X = TRUE;

		return 1;
	}
	//3.Z退刀
	case 3:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_Z] = SCurPoint.dArray[AXIS_Z] - pSG75LoopPara->cDir_Z * pSG75LoopPara->dRetract_Z;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;
		//1.4将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//2.判断下一步
		if (pSG75LoopPara->bLastFeed)
			return 2;//结束
		else
			pSG75LoopPara->cStepInLoop = 4;//X回起点
		return 1;
	}
	//4.X回起点
	case 4:
	{
		//1.填写SLine
		SLine SLineInfo = { 0 };
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = SCurPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG75LoopPara->dSX;
		//1.3获取SLine的实体类型
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;
		//1.4将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//2.判断下一步
		pSG75LoopPara->cStepInLoop = 2;//Z进刀

		return 1;
	}
	default:
		break;
	}
}




