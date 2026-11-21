/*
ns～nf程序段中，只能有G功能：
Ga组指令：G00、G01、G02、G03、G04、G05、G6.2、G6.3、G7.2、G7.3
Gz组指令：G04
速度控制指令：G96、G97、G98、G99
刀补指令：G40、G41、G42				pSAssignConfig->SGetTextStateInfo = pSG71Para->SGetTextStateInfo;

*/


#include <math.h>
#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"
#include "GeneralFunctions.h"
#include "GCode.h"
#include"list.h"



static SG71Para* pSG71ParaCheck[CHANNEL_NUM_MAX];
static SG71Para* pSG71ParaAssign[CHANNEL_NUM_MAX];

SPointDouble g_SPathStartPoint;//绝对坐标

//添加G71规则结构体
void addG71RegulationAPI()
{
	//指令：G71
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(71, sGz, 2, mallocG71ParaStruct, freeG71ParaStruct, getG71Para, CheckG71, AssignG71);
}
//添加G72规则结构体
void addG72RegulationAPI()
{
	//指令：G71
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(72, sGz, 2, mallocG71ParaStruct, freeG71ParaStruct, getG71Para, CheckG71, AssignG71);
}

//参数结构体初始化函数？


/*************************************************
  函数名称：int8 mallocG71ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G71参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG71ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG71ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG71Para), ORAM_NUM420);//暂未对内存编号
		if (pSG71ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG71ParaCheck[ucChannelNum], 0, sizeof(SG71Para));
		return 1;
		//pSG71ParaAssign[ucChannelNum]->SPathList = initList();
	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG71ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG71Para), ORAM_NUM421);
		if (pSG71ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG71ParaAssign[ucChannelNum], 0, sizeof(SG71Para));
		return 1;
		//pSG71ParaAssign[ucChannelNum]->SPathList = initList();
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
int8 freeG71ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG71ParaCheck[ucChannelNum] != NULL)
		{
			//轨迹链表非空
			if (pSG71ParaCheck[ucChannelNum]->pSPathList != NULL)
			{
				freeList(&pSG71ParaCheck[ucChannelNum]->pSPathList);
			}
			//槽链表非空
			if (pSG71ParaCheck[ucChannelNum]->pSGrooveList != NULL)
			{
				freeList_2(&pSG71ParaCheck[ucChannelNum]->pSGrooveList, freeGroovePathList);

			}


			freeGCode(pSG71ParaCheck[ucChannelNum]);
			pSG71ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG71ParaAssign[ucChannelNum] != NULL)
		{
			//轨迹链表非空
			if(pSG71ParaAssign[ucChannelNum]->pSPathList!=NULL)
			{
				freeList(&pSG71ParaAssign[ucChannelNum]->pSPathList);
			}
			//槽链表非空
			if (pSG71ParaAssign[ucChannelNum]->pSGrooveList!=NULL)
			{
				freeList_2(&pSG71ParaAssign[ucChannelNum]->pSGrooveList, freeGroovePathList);

			}
			freeGCode(pSG71ParaAssign[ucChannelNum]);
			pSG71ParaAssign[ucChannelNum] = NULL;
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
int8 getG71Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
	//1.检查
	if (ucMode == GC_SYNTAX)
	{
		SSyntaxConfiguration* pSSyntaxConfig = &pChannelConfig->SSyntaxConfig;
		SG71Para* pSG71Para = pSG71ParaCheck[pSSyntaxConfig->ucChannelNum];
		//1.1检查时，获取第一程序段的参数
		if (pSSyntaxConfig->cCommandSegmentCnt == 1)
		{
			pSG71Para->bG72 = pSSyntaxConfig->sCurrCommandVal / pow(10, GCOMMAND_DECIMAL_PLACE) == G72 ? 1 : 0;
			pSG71Para->dDeltaD = pSG71Para->bG72 ? pSScanContainer->dUVW[2] : pSScanContainer->dUVW[0];
			pSG71Para->dDeltaE = pSScanContainer->dRIJK[0];

			pSG71Para->cEndPointPos = pSScanContainer->bP == TRUE ? 1 : pSG71Para->cEndPointPos;
			pSG71Para->cEndPointPos = pSScanContainer->bQ == TRUE ? 2 : pSG71Para->cEndPointPos;

			pSG71Para->ucH = pSScanContainer->bH == TRUE ? pSScanContainer->dH : 0;

		}
		//1.2检查时，获取第二程序段的参数
		else if (pSSyntaxConfig->cCommandSegmentCnt == 2)
		{
			pSG71Para->unNS = pSScanContainer->dP;
			pSG71Para->unNF = pSScanContainer->dQ;
			pSG71Para->dDeltaU = pSScanContainer->dUVW[0];
			pSG71Para->dDeltaW = pSScanContainer->dUVW[2];
		}

	}
	//2.赋值
	else if (ucMode == GC_ASSIGN)
	{
		SAssignConfiguration* pSAssignConfig = &pChannelConfig->SAssignConfig;       //赋值结构体
		SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];
		//2.1赋值时，获取第一程序段的参数
		if (pSAssignConfig->cCommandSegmentCnt == 1)
		{
			pSG71Para->bG72 = pSAssignConfig->sCurrCommandVal / pow(10, GCOMMAND_DECIMAL_PLACE) == G72 ? 1 : 0;
			pSG71Para->dDeltaD = pSG71Para->bG72 ? pSScanContainer->dUVW[2] : pSScanContainer->dUVW[0];
			pSG71Para->dDeltaE = pSScanContainer->dRIJK[0];

			pSG71Para->cEndPointPos = pSScanContainer->bP == TRUE ? 1 : pSG71Para->cEndPointPos;
			pSG71Para->cEndPointPos = pSScanContainer->bQ == TRUE ? 2 : pSG71Para->cEndPointPos;

			
			pSG71Para->ucH = pSScanContainer->bH == TRUE ? pSScanContainer->dH : 0;
			pSG71Para->cEndPointPos = pSScanContainer->dRIJK[2] > 0 ? 2 : 0;

		}
		//2.2赋值时，获取第二程序段的参数
		else if (pSAssignConfig->cCommandSegmentCnt == 2)
		{
			pSG71Para->unNS = pSScanContainer->dP;
			pSG71Para->unNF = pSScanContainer->dQ;
			pSG71Para->dDeltaU = pSScanContainer->dUVW[0];
			pSG71Para->dDeltaW = pSScanContainer->dUVW[2];

			//pSG71Para->ucH = pSScanContainer->bH == TRUE ? pSScanContainer->dH : 0;
			//pSG71Para->cEndPointPos = pSScanContainer->bRIJK[2] == TRUE ? 1 : 0;
		}
	}
}

int8 CheckG71(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	SG71Para* pSG71Para = pSG71ParaCheck[pSSyntaxConfig->ucChannelNum];

	//1.参数检查
		// 切深必须为有限正量，零/负/非有限值在获取加工轨迹之前拒绝 唐子杰2026.10.13
		if (isnan(pSG71Para->dDeltaD) || isinf(pSG71Para->dDeltaD) || !(pSG71Para->dDeltaD > 0))
		{
			if (pSG71Para->bG72)
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM107);
			else
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM110);
			return -1;
		}
	

		// P 是否输入
		if (pSSyntaxConfig->SScanContainer.bP != TRUE)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM103);
			return -1;
		}
		// Q 是否输入
		if (pSSyntaxConfig->SScanContainer.bQ != TRUE)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM104);
			return -1;
		}
		// P 是否整数且范围合法（1 ~ 99999999）
		if (checkParaInteger('P', &pSSyntaxConfig->SScanContainer) == -1 ||
			checkParaRange('P', &pSSyntaxConfig->SScanContainer, 1, 99999999) == -1)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM82);
			return -1;
		}
		// Q 是否整数且范围合法（1 ~ 99999999）
		if (checkParaInteger('Q', &pSSyntaxConfig->SScanContainer) == -1 ||
			checkParaRange('Q', &pSSyntaxConfig->SScanContainer, 1, 99999999) == -1)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM82);
			return -1;
		}
	
	
	//2.获取加工轨迹
	SProgram* pSCurrProgram = NULL;   //当前程序结构体
	pSCurrProgram = FindNodeData(getProgramList(pSSyntaxConfig->ucChannelNum), 0);
	if (pSCurrProgram == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	int8 cReturnVal;
	cReturnVal = getMachiningPath(GC_SYNTAX, pSCurrProgram, pSG71Para->unNS, pSG71Para->unNF, pSG71Para->dDeltaU / 2, pSG71Para->dDeltaW, &pSG71Para->pSPathList, &pSG71Para->SGetTextStateInfo, pSSyntaxConfig->ucChannelNum, G71);
	//判断加工链表是否获取完成
	if (cReturnVal == -1)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM110);
		return -1;
	}
	else if (cReturnVal == 0)
	{
		return 0;
	}
	if (cReturnVal == 1)
	{
		if (pSG71Para->bG72)
		{
			//交换轨迹链表XZ坐标
			swapPathListXZCoord(&pSG71Para->pSPathList);
		}
	}

	//2.2检查轨迹单调性
	//checkMachiningPathMonotonicity
	//if (pSG71Para->ucH == 0 && checkPathXZMonotonicity(pSG71Para->pSPathList) == -1)
	//{
	//	if (pSG71Para->bG72)
	//		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM421);
	//	else
	//		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM418);

	if (pSG71Para->ucH == 0)
	{
		SList SContourPath = *pSG71Para->pSPathList;
		SLine * pSEntry = FindNodeData(pSG71Para->pSPathList, SContourPath.cSize - 1);
	/* 提取时已确认首段为带坐标的直线；头插链表的尾项就是首段。
	 * G72 已交换 X/Z，只校验入刀轴，不用入刀方向限制轮廓。 */
	if (SContourPath.cSize < 2 || pSEntry == NULL ||!isZero(pSEntry->SEndPoint.dArray[AXIS_Z] - pSEntry->SStartPoint.dArray[AXIS_Z]))
	{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum,
			pSG71Para->bG72 ? GCODE_ERROR_NUM422 : GCODE_ERROR_NUM417);
			return -1;
	}
	/* 只缩短栈上检查视图，保留真实链表及首段的快速/进给类型。 */
			SContourPath.cSize--;
	if (checkPathXZMonotonicity(&SContourPath) == -1)
	{
		    parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum,
			pSG71Para->bG72 ? GCODE_ERROR_NUM421 : GCODE_ERROR_NUM418);
			return -1;
			}
	}

	//2.3检查指令参数范围
	if (pSSyntaxConfig->SScanContainer.bH == TRUE)
	{
		pSSyntaxConfig->SScanContainer.bA2Z['H' - 65] = TRUE;
		pSSyntaxConfig->SScanContainer.dA2Z['H' - 65] = pSSyntaxConfig->SScanContainer.dH;

		if (checkParaRange('H', &pSSyntaxConfig->SScanContainer, 0, 2) == -1 ||
			checkParaInteger('H', &pSSyntaxConfig->SScanContainer) == -1)
		{
			return -1;
		}
	}
	return 2;


}

int8 AssignG71(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];
	
	//G71Ⅰ型
	if (pSG71Para->ucH == 0)
	{
		//判断是否需要交换当前点坐标
		//当cStep为0时，正在获取G72加工轨迹链表，不需要交换坐标
		if (pSG71Para->bG72 == TRUE && pSG71Para->cStep != 0)
		{
			//G72交换系统状态当前点XZ坐标
			double dTemp;
			dTemp = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] = dTemp;

			pSG71Para->bSwapFlag = TRUE;
		}

		int8 cRet = 0;

		switch (pSG71Para->cStep)
		{
		//0.获取NSNF轨迹链表
		case 0:
		{
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}

			//获取加工链表
			int8 cReturnVal;
			cReturnVal = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG71Para->unNS, pSG71Para->unNF, pSG71Para->dDeltaU / 2, pSG71Para->dDeltaW, &pSG71Para->pSPathList, &pSG71Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum,G71);
			//判断加工链表是否获取完成
			if (cReturnVal == 1)
			{			
				pSG71Para->cStep = 1;
				if (pSG71Para->bG72)
				{
					//交换轨迹链表XZ坐标
					swapPathListXZCoord(&pSG71Para->pSPathList);
				}

#if G71_DEBUG
				printf("-----------------------------轨迹预览cPart == 0----------------------------\n");
				printList(pSG71Para->pSPathList, printDataG71G72I);
				printf("\n");
#endif

			}
			else if (cReturnVal == -1)
			{
				return -1;
			}
			
			cRet = 0;
			break;
		}

		//1.赋值预处理
		case 1:
		{
			if (pSG71Para->pSPathList->cSize < 1)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				pSG71Para->cStep = 0;
				return -1;
			}
			//注：因为各实体类型结构体的前三个变量相同，所以可用SLine存放不同轨迹信息
			SLine* pSFirstPath = FindNodeData(pSG71Para->pSPathList, pSG71Para->pSPathList->cSize - 1);
			SLine* pSLastPath = FindNodeData(pSG71Para->pSPathList, 0);
			
			//1.0获取A点坐标
			pSG71Para->SPointA1 = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			//1.1获取A'点坐标
			pSG71Para->SPointA2 = pSFirstPath->SStartPoint;

			//获取起点绝对坐标
			g_SPathStartPoint.dArray[AXIS_X] = pSG71Para->SPointA1.dArray[AXIS_X] - pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_X];
			g_SPathStartPoint.dArray[AXIS_Z] = pSG71Para->SPointA1.dArray[AXIS_Z] - pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_Z];

			//1.4获取轨迹加工方向
			getPathMachineDir(pSG71Para->pSPathList, &pSG71Para->cMachineXDir, &pSG71Para->cMachineZDir);

			//1.5初始化槽链表
			//1.5.1 槽链初始化
			pSG71Para->pSGrooveList = initList(ORAM_NUM357, ORAM_NUM358);

			//获取轨迹最后一段实体
			SLine* pSEntity = NULL;
			pSEntity = FindNodeData(pSG71Para->pSPathList, 0);

			//若轨迹最后一段实体终点X低于轨迹起点X小，需要补齐
			if (pSG71Para->cMachineXDir * (pSEntity->SEndPoint.dArray[AXIS_X] - pSG71Para->SPointA2.dArray[AXIS_X]) > 0)
			{
				SLine* pSLineInfo = NULL;
				pSLineInfo = mallocGCode(sizeof(SLine), ORAM_NUM355);
				pSLineInfo->EntityType = L1NE;
				pSLineInfo->SStartPoint = pSLineInfo->SEndPoint = pSEntity->SEndPoint;
				pSLineInfo->SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];

				if (insertList(pSG71Para->pSPathList, 0, pSLineInfo, ORAM_NUM354) == FALSE)
				{
					freeGCode(pSLineInfo);
					parseGCodeErrorB(0, GCODE_ERROR_NUM300);
					pSG71Para->cStep = 0;
					return -1;
				}
			}

			pSG71Para->SPointC2 = pSEntity->SEndPoint;

			pSG71Para->cCurrPointGrooveNum = 1;
			pSG71Para->cCurrMachineGrooveNum = 1;
			pSG71Para->cCurrMachineDir = 1;

			//1.6
			pSG71Para->cStep = 2;


			cRet = 0;//未写入轨迹
			
			break;

		}
		
		//2.分槽（将轨迹链表 改为 槽链表）
		case 2:
		{
			int8 cReturnVal = divideGroove(&pSG71Para->pSPathList, &pSG71Para->pSGrooveList, AXIS_X, pSG71Para->cMachineXDir);
			//分槽完毕
			if (cReturnVal == TRUE)
			{
#if G71_DEBUG
				printf("-----------------------------轨迹预览cPart == 7----------------------------\n");
				printListII(pSG71Para->pSGrooveList, printDataG71G72II);
				printf("\n");
#endif				
				pSG71Para->cStep = 3;
			}

			cRet = 0;//未写入轨迹
			break;
		}

		//3.写入轨迹当前点-A'
		case 3:
		{
			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//判断下一目标块
			SGrooveData* pSFirstGrooveData;
			pSFirstGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, 1);

			//计算起始边长度
			double StartLineX;
			StartLineX = fabs(pSFirstGrooveData->SPointA.dX - pSFirstGrooveData->SPointC.dX);
			//进给量>=起始边
			if (isEqualOrGreaterThanZero(pSG71Para->dDeltaD - StartLineX))
			{
				//下一步执行写入加工轨迹链表
				pSG71Para->cStep = 5;
			}
			else
			{
				//下一步执行正循环块
				pSG71Para->cStep = 4;
			}

			//计算进给余量
			pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;

			cRet = 1;//写入轨迹
			break;
		}
		
		//4.正循环块
		case 4:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//进给
			case 0:
			{
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 1;
				}
				cRet = 1;
				break;
			}
			//切削
			case 1:
			{
				if (cut_1(pSAssignConfig, pSGCodeParseUnit, AXIS_Z))
				{
					pSG71Para->cStepInBlock = 2;
				}
				cRet = 1;
				break;
			}
			//退刀
			case 2:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] -= pSG71Para->cMachineXDir * pSG71Para->dDeltaE;
				SLineInfo.SEndPoint.dArray[AXIS_Z] -= pSG71Para->cMachineZDir * pSG71Para->dDeltaE;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步执行回退
				pSG71Para->cStepInBlock = 3;

				cRet = 1;
				break;
			}
			//回退
			case 3:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步执行定位
				pSG71Para->cStepInBlock = 4;

				cRet = 1;
				break;
			}
			//定位
			case 4:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SCycleBlockPointA.dX;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SCycleBlockPointA.dZ;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步判断目标块
				pSG71Para->cStepInBlock = 5;

				cRet = 1;
				break;
			}
			//判断下一轨迹块
			case 5:
			{
				//计算下一次进刀X值
				double dNextTargetX;
				dNextTargetX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dDeltaD;

				//获取进给段所在槽号的槽底X值
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//下一次进刀后X未达到槽底
				if (isUnEqualAndLessThanZero(pSG71Para->cMachineXDir * (dNextTargetX - pSGrooveData->SPointC.dX)))
				{
					//计算正循环块进给余量
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;

					//下一目标块为正循环块
					pSG71Para->cStep = 4;

				}
				else
				{
					//下一目标块为写入加工轨迹链表
					pSG71Para->cStep = 5;
				}

				//块内步骤置0
				pSG71Para->cStepInBlock = 0;

				cRet = 0;
				break;
			}
			default:
				break;
			}

			break;
		}

		//5.写入加工轨迹链表
		case 5:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//写入当前点-第一段实体终点
			case 0:
			{			
				//找到第一段实体
				SGrooveData* pSGrooveData;
				SLine* pSLine;
				pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrMachineGrooveNum);
				if (pSGrooveData->pSGroovePathList->cSize < 1)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM300);
					pSG71Para->cStep = 0;
					pSG71Para->cStepInBlock = 0;
					return -1;
				}
				pSLine = FindNodeData(pSGrooveData->pSGroovePathList, pSGrooveData->pSGroovePathList->cSize - 1);

				//写入当前点-第一段实体终点
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = L1NE;
				SLineInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint = pSLine->SEndPoint;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
				if (pSGrooveData->pSGroovePathList->cSize < 2)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM300);
					pSG71Para->cStep = 0;
					pSG71Para->cStepInBlock = 0;
					return -1;
				}
				pSG71Para->usCurPos = pSGrooveData->pSGroovePathList->cSize - 2;

				pSG71Para->cStepInBlock = 1;

				cRet = 1;
				break;
			}
			case 1:
			{
				//找到当前加工槽数据
				SGrooveData* pSGrooveData;
				pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrMachineGrooveNum);

				//将槽轨迹链表中的实体写入缓冲区
				uint16 usStartPos = 0;
				if (pSGrooveData->pSGroovePathList->cSize < 2)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM300);
					pSG71Para->cStep = 0;
					pSG71Para->cStepInBlock = 0;
					return -1;
				}
				uint16 usEndPos = pSGrooveData->pSGroovePathList->cSize - 2;
				
				//写入加工轨迹链表
				int8 cReturnVal;

				cReturnVal = writePathListIntoGCodeBuf(pSGrooveData->pSGroovePathList, usStartPos, usEndPos, &pSG71Para->usCurPos, pSGCodeParseUnit, pSG71Para->cCurrMachineDir);

				//判断当前加工轨迹链表实体是否全部写入缓冲区
				if (cReturnVal)
				{
					pSG71Para->cStep = 6;
					//pSG71Para->cStepInBlock = 0;
				}
				cRet = 1;
				break;
			}
			}
			break;
		}

		//6.结束块
		case 6:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//X退刀
			case 0:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				if (pSG71Para->cEndPointPos == 0)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA1.dArray[AXIS_X];
				else if (pSG71Para->cEndPointPos == 1)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];
				else if (pSG71Para->cEndPointPos == 2)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointC2.dArray[AXIS_X];

				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSG71Para->cStepInBlock = 1;

				cRet = 1;
				break;
			}
			//Z轴退刀
			case 1:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				if (pSG71Para->cEndPointPos == 0)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA1.dArray[AXIS_Z];
				else if (pSG71Para->cEndPointPos == 1)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
				else if (pSG71Para->cEndPointPos == 2)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointC2.dArray[AXIS_Z];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSAssignConfig->SGetTextStateInfo = pSG71Para->SGetTextStateInfo;

				cRet = 2;
				break;
			}

			}
			break;
		}
		
		default:
			break;
		}

		//判断是否需要交换当前点坐标
		if (pSG71Para->bG72 == TRUE && pSG71Para->bSwapFlag)
		{
			//G72再次交换系统状态当前点XZ坐标（还原）
			double dTemp;
			dTemp = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] = dTemp;
			
			pSG71Para->bSwapFlag = FALSE;
		}

		//若G72写了轨迹，则交换缓冲区点XZ坐标
		if (pSG71Para->bG72 && pSGCodeParseUnit->EntityType != DOT)
			swapBufXZCoord(pSGCodeParseUnit);

		return cRet;

	}

	//G71Ⅱ型
	else if (pSG71Para->ucH == 1)
	{		
		//判断是否需要交换当前点坐标
		//当cStep为0时，正在获取G72加工轨迹链表，不需要交换坐标
		if (pSG71Para->bG72 == TRUE && pSG71Para->cStep != 0)
		{
			//G72交换系统状态当前点XZ坐标
			double dTemp;
			dTemp = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] = dTemp;

			pSG71Para->bSwapFlag = TRUE;
		}

		int8 cRet = 0;

		switch (pSG71Para->cStep)
		{
		//0.获取NSNF轨迹链表（写G71参数结构体的部分数据）
		case 0:
		{
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//获取加工链表
			int8 cReturnVal;
			cReturnVal = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG71Para->unNS, pSG71Para->unNF, pSG71Para->dDeltaU / 2, pSG71Para->dDeltaW, &pSG71Para->pSPathList, &pSG71Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum,G71);
			//判断加工链表是否获取完成
			if (cReturnVal == 1)
			{
				if (pSG71Para->bG72)
				{
					//交换轨迹链表XZ坐标
					swapPathListXZCoord(&pSG71Para->pSPathList);
				}
#if G71_DEBUG
				printf("-----------------------------轨迹预览cPart == 0----------------------------\n");
				printList(pSG71Para->pSPathList, printDataG71G72I);
				printf("\n");
#endif
				pSG71Para->cStep = 1;
			}
			else if (cReturnVal == -1)
			{
				return -1;
			}
			
			cRet = 0;//未写入轨迹
			break;
		}

		//1.赋值预处理（写G71参数结构体的部分数据）
		case 1:
		{
			//注：因为各实体类型结构体的前三个变量相同，所以可用SLine存放不同轨迹信息
			if (pSG71Para->pSPathList->cSize < 1)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				pSG71Para->cStep = 0;
				return -1;
			}
			SLine* pSFirstPath = FindNodeData(pSG71Para->pSPathList, pSG71Para->pSPathList->cSize - 1);

			//1.0获取A点坐标
			pSG71Para->SPointA1 = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			//1.1获取A'点坐标
			pSG71Para->SPointA2 = pSFirstPath->SStartPoint;

			//获取起点绝对坐标
			g_SPathStartPoint.dArray[AXIS_X] = pSG71Para->SPointA1.dArray[AXIS_X] - pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_X];
			g_SPathStartPoint.dArray[AXIS_Z] = pSG71Para->SPointA1.dArray[AXIS_Z] - pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_Z];

			//1.4获取轨迹加工方向
			getPathMachineDir(pSG71Para->pSPathList, &pSG71Para->cMachineXDir, &pSG71Para->cMachineZDir);

			//1.5初始化槽链表
			//1.5.1 槽链初始化
			pSG71Para->pSGrooveList = initList(ORAM_NUM357, ORAM_NUM358);

			//获取轨迹最后一段实体
			SLine* pSEntity = NULL;
			pSEntity = FindNodeData(pSG71Para->pSPathList, 0);
		
			//若轨迹最后一段实体终点X低于轨迹起点X小，需要补齐
			if (pSG71Para->cMachineXDir * (pSEntity->SEndPoint.dArray[AXIS_X] - pSG71Para->SPointA2.dArray[AXIS_X]) > 0)
			{
				SLine* pSLineInfo = NULL;
				pSLineInfo = mallocGCode(sizeof(SLine), ORAM_NUM355);
				pSLineInfo->EntityType = L1NE;
				pSLineInfo->SStartPoint = pSLineInfo->SEndPoint = pSEntity->SEndPoint;
				pSLineInfo->SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];


				insertList(pSG71Para->pSPathList, 0, pSLineInfo, ORAM_NUM354);
			}

			pSG71Para->SPointC2 = pSEntity->SEndPoint;

			//1.6初始化轨迹链表的遍历索引,为最后一个节点的位置
			//pSG71Para->sCurrPathPos = getListSize(pSG71Para->pSPathList) - 1;

			pSG71Para->cCurrMachineGrooveNum = 1;		//当前加工槽号（新）
			pSG71Para->cCurrPointGrooveNum = 1;			//当前点所在槽号（新）
			pSG71Para->cCurrMachineDir = 1;

			pSG71Para->cStep = 2;

			cRet = 0;//未写入轨迹

			break;

		}

		//2.分槽（将轨迹链表 改为 槽链表）
		case 2:
		{
			int8 cReturnVal = divideGroove(&pSG71Para->pSPathList, &pSG71Para->pSGrooveList, AXIS_X, pSG71Para->cMachineXDir);

			//分槽完毕
			if (cReturnVal == TRUE)
			{
#if G71_DEBUG
				printf("-----------------------------轨迹预览cPart == 7----------------------------\n");
				printListII(pSG71Para->pSGrooveList, printDataG71G72II);
				printf("\n");
#endif				
				pSG71Para->cStep = 3;
			}

			cRet = 0;//未写入轨迹

			break;
		}

		//3.写入A-A'
		case 3:	
		{
			//写入切削轨迹
			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;//G0
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//判断下一目标块
			SGrooveData* pSFirstGrooveData;
			pSFirstGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, 1);

			//计算起始边长度
			double StartLineX;
			StartLineX = fabs(pSFirstGrooveData->SPointA.dX - pSFirstGrooveData->SPointC.dX);
			//进给量>=起始边
			if (isEqualOrGreaterThanZero(pSG71Para->dDeltaD - StartLineX))
			{
				//下一步执行正收尾块
				pSG71Para->cStep = 5;
			}
			else
			{
				//下一步执行正循环块
				pSG71Para->cStep = 4;
			}

			//计算进给回切余量
			pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
			pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

			//写入轨迹，但指令轨迹未写完
			cRet = 1;
			break;
		}

		//4.正循环块
		case 4:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//进给
			case 0:
			{
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 1;
				}
				if (pSGCodeParseUnit->EntityType == OTHERTYPE)
				{
					pSGCodeParseUnit->EntityType = L1NE;
				}
				cRet = 1;
				break;
			}
			//切削
			case 1:
			{
				if (cut_1(pSAssignConfig, pSGCodeParseUnit, AXIS_Z))
				{
					pSG71Para->cStepInBlock = 2;
				}
				cRet = 1;
				break;
			}
			//回切
			case 2:
			{
				back(pSAssignConfig, pSGCodeParseUnit, AXIS_X);
				if (isZero(pSG71Para->dRestBackVal))
				{
					//下一步执行回退
					pSG71Para->cStepInBlock = 3;
				}
				cRet = 1;
				break;
			}
			//回退
			case 3:
			{
				//写入切削轨迹
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;//G0
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SCycleBlockPointA.dZ;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步执行定位
				pSG71Para->cStepInBlock = 4;

				pSG71Para->cCurrPointGrooveNum = pSG71Para->cCurrMachineGrooveNum;

				//写入轨迹，但指令轨迹未写完
				cRet = 1;
				break;
			}
			//定位
			case 4:
			{
				//写入切削轨迹
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SCycleBlockPointA.dX;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步执行定位
				pSG71Para->cStepInBlock = 5;

				//写入轨迹，但指令轨迹未写完
				cRet = 1;
				break;
			}
			//判断下一轨迹块
			case 5:
			{
				//计算下一次进刀X值
				double dNextTargetX;
				dNextTargetX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dDeltaD;

				//获取进给段所在槽号的槽底X值
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//下一次进刀后X未达到槽底
				if (isUnEqualAndLessThanZero(pSG71Para->cMachineXDir * (dNextTargetX - pSGrooveData->SPointC.dX)))
				{
					//计算正循环块进给回切余量
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
					pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

					//下一目标块为正循环块
					pSG71Para->cStep = 4;
				}
				else
				{
					//计算反收尾块进给回切余量
					pSG71Para->dRestFeedVal = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGrooveData->SPointC.dX);
					pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

					//下一目标块为正收尾块
					pSG71Para->cStep = 5;
				}

				//块内步骤置0
				pSG71Para->cStepInBlock = 0;
				cRet = 0;
				break;
			}
			default:
				break;
			}

			break;
		}

		//5.正收尾块
		case 5:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//进刀
			case 0:
			{	
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					//下一步执行回切
					pSG71Para->cStepInBlock = 1;
				}
				if (pSGCodeParseUnit->EntityType == OTHERTYPE)
				{
					pSGCodeParseUnit->EntityType = L1NE;
				}
				cRet = 1;
				break;
			}
			//写收尾块轨迹1（沿当前加工槽起始边进刀）
			case 1:
			{
				back(pSAssignConfig, pSGCodeParseUnit, AXIS_X);
				//正收尾块回切不超过槽顶点
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
				if (isZero(pSG71Para->dRestBackVal) ||
					(isZero(pSGCodeParseUnit->SEndPoint.dArray[AXIS_X] - pSGrooveData->SPointB.dX) && isZero(pSGCodeParseUnit->SEndPoint.dArray[AXIS_Z] - pSGrooveData->SPointB.dZ)))
				{
					//加工方向为1
					pSG71Para->cCurrMachineDir = 1;
					//下一步执行目标块判断
					pSG71Para->cStepInBlock = 2;
				}
				cRet = 1;
				break;
			}
			//判断下一目标块
			case 2:
			{
				//G71Ⅲ型轨迹全部计算完成
				if (pSG71Para->cCurrPointGrooveNum == pSG71Para->pSGrooveList->cSize)
				{
					//下一步执行结束块
					pSG71Para->cStep = 7;
				}
				else
				{
					//下一步执行过渡块
					pSG71Para->cStep = 6;
				}
				//块内步骤至0
				pSG71Para->cStepInBlock = 0;
				cRet = 0;
				break;

			}
			}
			
			break;
		}
		
		//6.过渡块
		case 6:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//写过渡块轨迹1（定位到当前槽终止边顶点的X值）
			case 0:
			{
				//找到当前加工槽号的槽数据节点
				SGrooveData* pSCurrGrooveData;
				pSCurrGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//写入切削轨迹
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSCurrGrooveData->SPointB.dX;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步写过渡块轨迹2
				pSG71Para->cStepInBlock = 1;

				//写入轨迹，但指令轨迹未写完
				cRet = 1;
				break;
			}
			//写过渡块轨迹2（定位到当前槽终止边顶点的Z值）
			case 1:
			{
				//找到当前加工槽号的槽数据节点
				SGrooveData* pSCurrGrooveData;
				pSCurrGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//写入切削轨迹
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = L1NE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSCurrGrooveData->SPointB.dZ;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//过渡完成，更新槽号
				pSG71Para->cBeforeTransitGrooveNum = pSG71Para->cCurrPointGrooveNum;
				pSG71Para->cAfterTransitGrooveNum = pSG71Para->cCurrPointGrooveNum + 1;
				pSG71Para->cCurrPointGrooveNum++;
				pSG71Para->cCurrMachineGrooveNum++;

				//下一步判断目标块
				pSG71Para->cStepInBlock = 2;

				//写入轨迹，但指令轨迹未写完
				cRet = 1;
				break;
			}
			//判断下一目标块（正循环块or正收尾块）
			case 2:
			{
				//获取过渡前与过渡后槽数据
				SGrooveData* pSGroove_1;	//过渡前槽数据
				SGrooveData* pSGroove_2;	//过渡后槽数据
				pSGroove_1 = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cBeforeTransitGrooveNum);
				pSGroove_2 = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cAfterTransitGrooveNum);

				//更新剩余进给量
				pSG71Para->dRestFeedVal = fabs(pSGroove_1->SPointB2.dX - pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X]);

				//计算下一次进刀X值
				double dNextTargetX;
				dNextTargetX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dDeltaD;

				//获取进给段所在槽号的槽底X值
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//下一次进刀后X未达到槽底
				if (isUnEqualAndLessThanZero(pSG71Para->cMachineXDir * (dNextTargetX - pSGrooveData->SPointC.dX)))
				{
					//更新剩余回切量
					pSG71Para->dRestBackVal = pSG71Para->dDeltaD + pSG71Para->dDeltaE;

					//下一目标块为正循环块
					pSG71Para->cStep = 4;
				}
				else
				{					
					//更新剩余回切量(此处回切余量计算存在问题)
					pSG71Para->dRestBackVal = fabs(pSGroove_1->SPointA2.dX - pSGroove_2->SPointC.dX) + pSG71Para->dDeltaE;

					//下一目标块为正收尾块
					pSG71Para->cStep = 5;
				}

				//块内步骤置0
				pSG71Para->cStepInBlock = 0;

				cRet = 0;

			}
			default:
				break;
			}
			break;
		}

		//7.结束块
		case 7:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//X退刀
			case 0:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				if (pSG71Para->cEndPointPos == 0)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA1.dArray[AXIS_X];
				else if (pSG71Para->cEndPointPos == 1)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];
				else if (pSG71Para->cEndPointPos == 2)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointC2.dArray[AXIS_X];

				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSG71Para->cStepInBlock = 1;

				cRet = 1;
				break;
			}
			//Z轴退刀
			case 1:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				if (pSG71Para->cEndPointPos == 0)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA1.dArray[AXIS_Z];
				else if (pSG71Para->cEndPointPos == 1)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
				else if (pSG71Para->cEndPointPos == 2)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointC2.dArray[AXIS_Z];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSAssignConfig->SGetTextStateInfo = pSG71Para->SGetTextStateInfo;

				cRet = 2;
				break;
			}

			}
			break;
		}
		default:
			break;

		}

		//判断是否需要交换当前点坐标
		if (pSG71Para->bG72 == TRUE && pSG71Para->bSwapFlag)
		{
			//G72再次交换系统状态当前点XZ坐标（还原）
			double dTemp;
			dTemp = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] = dTemp;

			pSG71Para->bSwapFlag;
		}

		//若G72写了轨迹，则交换缓冲区点XZ坐标
		if (pSG71Para->bG72 && pSGCodeParseUnit->EntityType != DOT)
			swapBufXZCoord(pSGCodeParseUnit);

		return cRet;

	}

	//G71Ⅲ型
	else if (pSG71Para->ucH == 2)
	{
		//判断是否需要交换当前点坐标
		//当cStep为0时，正在获取G72加工轨迹链表，不需要交换坐标
		if (pSG71Para->bG72 == TRUE && pSG71Para->cStep != 0)
		{
			//G72交换系统状态当前点XZ坐标
			double dTemp;
			dTemp = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] = dTemp;

			pSG71Para->bSwapFlag = TRUE;
		}

		int8 cRet = 0;
		switch (pSG71Para->cStep)
		{
		//0.获取NSNF轨迹链表（写G71参数结构体的部分数据）
		case 0:
		{
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}

			//获取加工链表
			int8 cG71orG72;
			cG71orG72 = pSG71Para->bG72 == FALSE ? G71 : G72;
			int8 cReturnVal;
			cReturnVal = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG71Para->unNS, pSG71Para->unNF, pSG71Para->dDeltaU / 2, pSG71Para->dDeltaW, &pSG71Para->pSPathList, &pSG71Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, cG71orG72);
			//判断加工链表是否获取完成
			if (cReturnVal == 1)
			{
				if (pSG71Para->bG72)
				{
					//交换轨迹链表XZ坐标
					swapPathListXZCoord(&pSG71Para->pSPathList);
				}
	#if G71_DEBUG
				printf("-----------------------------轨迹预览cPart == 0----------------------------\n");
				printList(pSG71Para->pSPathList, printDataG71G72I);
				printf("\n");
	#endif
				pSG71Para->cStep = 1;
			}
			else if (cReturnVal == -1)
			{
				return -1;
			}
			cRet = 0;//未写入轨迹
			break;
		}

		//1.赋值预处理（写G71参数结构体的部分数据）
		case 1:
		{
			if (pSG71Para->pSPathList->cSize < 1)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				pSG71Para->cStep = 0;
				return -1;
			}

			//注：因为各实体类型结构体的前三个变量相同，所以可用SLine存放不同轨迹信息
			SLine* pSFirstPath = FindNodeData(pSG71Para->pSPathList, pSG71Para->pSPathList->cSize - 1);

			//1.0获取A点坐标
			pSG71Para->SPointA1 = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			//1.1获取A'点坐标
			pSG71Para->SPointA2 = pSFirstPath->SStartPoint;

			//获取起点绝对坐标
			g_SPathStartPoint.dArray[AXIS_X] = pSG71Para->SPointA1.dArray[AXIS_X] - pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_X];
			g_SPathStartPoint.dArray[AXIS_Z] = pSG71Para->SPointA1.dArray[AXIS_Z] - pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_Z];

			//1.4获取轨迹加工方向
			getPathMachineDir(pSG71Para->pSPathList, &pSG71Para->cMachineXDir, &pSG71Para->cMachineZDir);

			pSG71Para->cCurrPointGrooveNum = 1;
			pSG71Para->cCurrMachineGrooveNum = 1;
			pSG71Para->cCurrMachineDir = 1;

			//1.5初始化槽链表
			//1.5.1 槽链初始化
			pSG71Para->pSGrooveList = initList(ORAM_NUM357, ORAM_NUM358);

			//获取轨迹最后一段实体
			SLine* pSEntity = NULL;
			pSEntity = FindNodeData(pSG71Para->pSPathList, 0);

			//若轨迹最后一段实体终点X低于轨迹起点X小，需要补齐
			if (pSG71Para->cMachineXDir * (pSEntity->SEndPoint.dArray[AXIS_X] - pSG71Para->SPointA2.dArray[AXIS_X]) > 0)
			{
				SLine* pSLineInfo = NULL;
				pSLineInfo = mallocGCode(sizeof(SLine), ORAM_NUM355);
				pSLineInfo->EntityType = L1NE;
				pSLineInfo->SStartPoint = pSLineInfo->SEndPoint = pSEntity->SEndPoint;
				pSLineInfo->SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];

				insertList(pSG71Para->pSPathList, 0, pSLineInfo, ORAM_NUM354);
			}

			pSG71Para->SPointC2 = pSEntity->SEndPoint;

			pSG71Para->cStep = 2;


			cRet = 0;//未写入轨迹
			break;

		}

		//2.分槽（将轨迹链表 改为 槽链表）
		case 2:
		{
			int8 cReturnVal = divideGroove(&pSG71Para->pSPathList, &pSG71Para->pSGrooveList, AXIS_X, pSG71Para->cMachineXDir);

			//分槽完毕
			if (cReturnVal == TRUE)
			{
	#if G71_DEBUG
				printf("-----------------------------轨迹预览cPart == 7----------------------------\n");
				printListII(pSG71Para->pSGrooveList, printDataG71G72II);
				printf("\n");
	#endif				
				pSG71Para->cStep = 3;
			}

			cRet = 0;//未写入轨迹
			break;
		}

		//3.写入A-A'
		case 3:
		{
			//写入切削轨迹
			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = L1NE;
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];
			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//判断下一目标块
			SGrooveData* pSFirstGrooveData;
			pSFirstGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, 1);

			//计算起始边长度
			double StartLineX;
			StartLineX = fabs(pSFirstGrooveData->SPointA.dX - pSFirstGrooveData->SPointC.dX);
			//进给量>=起始边
			if (isEqualOrGreaterThanZero(pSG71Para->dDeltaD - StartLineX))
			{
				//下一步执行正收尾块
				pSG71Para->cStep = 5;
			}
			else
			{
				//下一步执行正循环块
				pSG71Para->cStep = 4;
			}

			//计算进给回切余量
			pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
			pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

			//写入轨迹，但指令轨迹未写完
			cRet = 1;
			break;
		}

		//4.正循环块
		case 4:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//正循环块预处理
			case 0:
			{
				////进给余量
				//pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
				////回切余量
				//pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;


				pSG71Para->cStepInBlock = 1;
			}
			//进给
			case 1:
			{
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 2;
				}
				if (pSGCodeParseUnit->EntityType == OTHERTYPE)
				{
					pSGCodeParseUnit->EntityType = L1NE;
				}
				cRet = 1;
				break;
			}
			//切削
			case 2:
			{
				if (cut_1(pSAssignConfig, pSGCodeParseUnit, AXIS_Z))
				{
					pSG71Para->cStepInBlock = 3;
				}
				cRet = 1;
				break;
			}
			//回切
			case 3:
			{
				back(pSAssignConfig, pSGCodeParseUnit, AXIS_X);
				if (isZero(pSG71Para->dRestBackVal))
				{
					//计算进给余量，供定位使用
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD + pSG71Para->dDeltaE;

					//正循环块中定位轨迹，加工方向为-1
					pSG71Para->cCurrMachineDir = -1;

					//定位
					pSG71Para->cStepInBlock = 4;
				}
				cRet = 1;
				break;
			}
			//定位
			case 4:
			{
				if (locate(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 5;
				}
				cRet = 1;
				break;
			}
			//判断下一轨迹块
			case 5:
			{
				//计算下一次进刀X值
				double dNextTargetX;
				dNextTargetX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dDeltaD;

				//获取进给段所在槽号的槽底X值
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//下一次进刀后X未达到槽底
				if (isUnEqualAndLessThanZero(pSG71Para->cMachineXDir * (dNextTargetX - pSGrooveData->SPointC.dX)))
				{
					//计算反循环块进给回切余量
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
					pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

					//下一目标块为反循环块
					pSG71Para->cStep = 5;

				}
				else
				{
					//反向切削超出槽底，且不为当前加工槽
					if (pSG71Para->cCurrPointGrooveNum != pSG71Para->cCurrMachineGrooveNum)
					{
						double dGrooveDeep;
						dGrooveDeep = fabs(pSGrooveData->SPointA.dX - pSGrooveData->SPointC.dX) < fabs(pSGrooveData->SPointB.dX - pSGrooveData->SPointC.dX) ?
							fabs(pSGrooveData->SPointA.dX - pSGrooveData->SPointC.dX) : fabs(pSGrooveData->SPointB.dX - pSGrooveData->SPointC.dX);

						pSG71Para->dRestFeedVal = dGrooveDeep / 2;
						pSG71Para->dRestBackVal = dGrooveDeep / 2;

						//下一目标块为反循环块
						pSG71Para->cStep = 5;
					}
					else
					{
						//计算反收尾块进给回切余量
						pSG71Para->dRestFeedVal = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGrooveData->SPointC.dX);
						pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

						//下一目标块为反收尾块
						pSG71Para->cStep = 7;
					}
				}

				//块内步骤置0
				pSG71Para->cStepInBlock = 0;
				cRet = 0;
				break;
			}
			default:
				break;
			}

			break;
		}

		//5.反循环块
		case 5:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//反循环块初始化
			case 0:
			{
				////进给余量
				//pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
				////回切余量
				//pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;



				pSG71Para->cStepInBlock = 1;
			}
			//进给
			case 1:
			{
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 2;
				}
				if (pSGCodeParseUnit->EntityType == OTHERTYPE)
				{
					pSGCodeParseUnit->EntityType = L1NE;
				}
				cRet = 1;
				break;
			}
			//切削
			case 2:
			{
				if (cut_2(pSAssignConfig, pSGCodeParseUnit, AXIS_Z))
				{
					pSG71Para->dRestBackVal = pSG71Para->dDeltaD + pSG71Para->dDeltaE;
					pSG71Para->cStepInBlock = 3;
				}
				cRet = 1;
				break;
			}
			//回切
			case 3:
			{
				back(pSAssignConfig, pSGCodeParseUnit, AXIS_X);
				if (isZero(pSG71Para->dRestBackVal))
				{
					//计算进给余量，供定位使用
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD + pSG71Para->dDeltaE;

					//反向循环块定位轨迹，加工方向为1
					pSG71Para->cCurrMachineDir = 1;

					pSG71Para->cStepInBlock = 4;
				}
				cRet = 1;
				break;
			}
			//定位
			case 4:
			{
				if (locate(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 5;
				}
				cRet = 1;

				break;
			}
			//判断下一轨迹块
			case 5:
			{
				//计算下一次进刀X值
				double dNextTargetX;
				dNextTargetX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dDeltaD;

				//获取进给段所在槽号的槽底X值
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//下一次进刀后X未达到槽底
				if (isUnEqualAndLessThanZero(pSG71Para->cMachineXDir * (dNextTargetX - pSGrooveData->SPointC.dX)))
				{
					//计算正循环块进给回切余量
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
					pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

					//下一目标块为正循环块
					pSG71Para->cStep = 4;
				}
				else
				{
					//计算正收尾块进给回切余量
					pSG71Para->dRestFeedVal = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGrooveData->SPointC.dX);
					pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

					//下一目标块为正收尾块
					pSG71Para->cStep = 6;
				}

				//块内步骤置0
				pSG71Para->cStepInBlock = 0;
				cRet = 0;
				break;
			}

			}
			
			break;
		}

		//6.正收尾块
		case 6:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//正收尾块预处理
			case 0:
			{
				////找到当前点所在槽数据
				//SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				////进给余量
				//pSG71Para->dRestFeedVal = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGrooveData->SPointC.dX);
				////回切余量
				//pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

				pSG71Para->cStepInBlock = 1;
			}
			//进给
			case 1:
			{
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 2;
				}
				if (pSGCodeParseUnit->EntityType == OTHERTYPE)
				{
					pSGCodeParseUnit->EntityType = L1NE;
				}
				cRet = 1;
				break;
			}
			//回切
			case 2:
			{
				back(pSAssignConfig, pSGCodeParseUnit, AXIS_X);
				//正收尾块回切不超过槽顶点
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
				if (isZero(pSG71Para->dRestBackVal) ||
					(isZero(pSGCodeParseUnit->SEndPoint.dArray[AXIS_X] - pSGrooveData->SPointB.dX) && isZero(pSGCodeParseUnit->SEndPoint.dArray[AXIS_Z] - pSGrooveData->SPointB.dZ)))
				{
					//加工方向为1
					pSG71Para->cCurrMachineDir = 1;

					pSG71Para->cStepInBlock = 3;
				}
				cRet = 1;
				break;
			}
			//判断下一目标块
			case 3:
			{
				//G71Ⅲ型轨迹全部计算完成
				if (pSG71Para->cCurrPointGrooveNum == pSG71Para->pSGrooveList->cSize)
				{
					//下一步执行结束块
					pSG71Para->cStep = 9;
				}
				else
				{
					//下一步执行过渡块
					pSG71Para->cStep = 8;
				}

				pSG71Para->cStepInBlock = 0;
				cRet = 0;
				break;
			}
			}

			break;
		}

		//7.反收尾块
		case 7:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//反收尾块预处理
			case 0:
			{
				////找到当前点所在槽数据
				//SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				////进给余量
				//pSG71Para->dRestFeedVal = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGrooveData->SPointC.dX);
				////回切余量
				//pSG71Para->dRestBackVal = pSG71Para->dRestFeedVal + pSG71Para->dDeltaE;

				pSG71Para->cStepInBlock = 1;
			}
			//进给
			case 1:
			{
				if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
				{
					pSG71Para->cStepInBlock = 2;
				}
				if (pSGCodeParseUnit->EntityType == OTHERTYPE)
				{
					pSGCodeParseUnit->EntityType = L1NE;
				}
				cRet = 1;
				break;
			}
			//回切
			case 2:
			{
				back(pSAssignConfig, pSGCodeParseUnit, AXIS_X);

				//正收尾块回切不超过槽顶点
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
				if (isZero(pSG71Para->dRestBackVal) ||
					(isZero(pSGCodeParseUnit->SEndPoint.dArray[AXIS_X] - pSGrooveData->SPointB.dX) && isZero(pSGCodeParseUnit->SEndPoint.dArray[AXIS_Z] - pSGrooveData->SPointB.dZ)))
				{
					pSG71Para->cCurrMachineDir = 1;

					pSG71Para->cStepInBlock = 3;
				}
				cRet = 1;
				break;
			}
			//判断下一目标块
			case 3:
			{
				//G71Ⅲ型轨迹全部计算完成
				if (pSG71Para->cCurrPointGrooveNum == pSG71Para->pSGrooveList->cSize)
				{
					//下一步执行结束块
					pSG71Para->cStep = 9;
				}
				else
				{
					//下一步执行过渡块
					pSG71Para->cStep = 8;
				}

				pSG71Para->cStepInBlock = 0;
				cRet = 0;
				break;
			}

			}

			break;
		}

		//8.过渡块
		case 8:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//计算过渡块ABC点
			case 0:
			{
				SGrooveData* pSGrooveData;
				//1.计算前ABC点置0
				pSG71Para->STransitBlockPointA;

				//找到过渡后的槽号
				int16 cTransitGrooveNum;
				cTransitGrooveNum = pSG71Para->cCurrPointGrooveNum + 1;
				for (; cTransitGrooveNum <= pSG71Para->pSGrooveList->cSize; cTransitGrooveNum++)
				{
					pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, cTransitGrooveNum);
					if (pSGrooveData->bFinished == FALSE)
					{
						//获取过渡块C点
						pSG71Para->STransitBlockPointC = pSGrooveData->bPointA2 == TRUE ? pSGrooveData->SPointA2 : pSGrooveData->SPointA;
						break;
					}
				}
				pSG71Para->cAfterTransitGrooveNum = cTransitGrooveNum;


				//获取过渡块A点
				pSG71Para->cCurrPointGrooveNum++;
				pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
				pSG71Para->STransitBlockPointA.dX = pSGrooveData->SPointA.dX;
				pSG71Para->STransitBlockPointA.dZ = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
				for (; pSG71Para->cCurrPointGrooveNum < cTransitGrooveNum; pSG71Para->cCurrPointGrooveNum++)
				{
					pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
					double dTemp;

					dTemp = fabs(pSGrooveData->SPointA.dX) > fabs(pSGrooveData->SPointB.dX) ? pSGrooveData->SPointA.dX : pSGrooveData->SPointB.dX;
					pSG71Para->STransitBlockPointA.dX = fabs(pSG71Para->STransitBlockPointA.dX) > fabs(dTemp) ? pSG71Para->STransitBlockPointA.dX : dTemp;
				}
				
				//获取过渡块B点
				pSG71Para->STransitBlockPointB.dX = pSG71Para->STransitBlockPointA.dX;
				pSG71Para->STransitBlockPointB.dZ = pSG71Para->STransitBlockPointC.dZ;

				pSG71Para->cStepInBlock = 1;
			}
			//当前点->过渡块A点（需要增设高度判断！！！）
			case 1:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = L1NE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->STransitBlockPointA.dX;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->STransitBlockPointA.dZ;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSG71Para->cStepInBlock = 2;

				cRet = 1;
				break;
			}
			//当前点->过渡块B点
			case 2:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = L1NE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->STransitBlockPointB.dX;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->STransitBlockPointB.dZ;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSG71Para->cStepInBlock = 3;

				cRet = 1;
				break;
			}
			//当前点->过渡块C点
			case 3:
			{			
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = L1NE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->STransitBlockPointC.dX;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->STransitBlockPointC.dZ;
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//过渡完成，更新槽号
				pSG71Para->cBeforeTransitGrooveNum = pSG71Para->cCurrMachineGrooveNum;
				pSG71Para->cCurrPointGrooveNum = pSG71Para->cAfterTransitGrooveNum;
				pSG71Para->cCurrMachineGrooveNum = pSG71Para->cAfterTransitGrooveNum;

				pSG71Para->cStepInBlock = 4;

				cRet = 1;
				break;
			}
			//判断下一目标块
			case 4:
			{
				SGrooveData* pSGroove_1;//过渡前槽数据
				SGrooveData* pSGroove_2;//过渡后槽数据

				pSGroove_1 = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cBeforeTransitGrooveNum);
				pSGroove_2 = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cAfterTransitGrooveNum);

				//1.计算过渡后的进给量
				//1.1当前点 与 过渡后槽起始边顶点 重合，则进给量为
				if (isZero(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGroove_2->SPointA.dX) &&
					isZero(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] - pSGroove_2->SPointA.dZ))
				{
					pSG71Para->dRestFeedVal = fabs(pSGroove_1->SPointB2.dX - pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X]);
				}
				//1.1当前点 与 过渡后槽起始边顶点 不重合，则进给量为D
				else
				{		
					pSG71Para->dRestFeedVal = pSG71Para->dDeltaD;
				}

				//计算下一次进刀X值
				double dNextTargetX;
				dNextTargetX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dRestFeedVal;

				//获取进给段所在槽号的槽底X值
				SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

				//下一次进刀后X未达到槽底
				if (isUnEqualAndLessThanZero(pSG71Para->cMachineXDir * (dNextTargetX - pSGrooveData->SPointC.dX)))
				{
					//计算回切余量
					pSG71Para->dRestBackVal = pSG71Para->dDeltaD + pSG71Para->dDeltaE;

					//下一目标块为正循环块
					pSG71Para->cStep = 4;
				}
				else
				{
					//计算回切余量(此处回切余量计算存在问题)
					pSG71Para->dRestBackVal = fabs(pSGroove_1->SPointA2.dX - pSGroove_2->SPointC.dX) + pSG71Para->dDeltaE;

					//下一目标块为正收尾块
					pSG71Para->cStep = 6;
				}

				//块内步骤置0
				pSG71Para->cStepInBlock = 0;

				cRet = 0;
				break;
			}
			default:
				break;
			}
		
			break;
		}

		//9.结束块
		case 9:
		{
			switch (pSG71Para->cStepInBlock)
			{
			//X轴退刀
			case 0:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				if (pSG71Para->cEndPointPos == 0)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA1.dArray[AXIS_X];
				else if (pSG71Para->cEndPointPos == 1)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointA2.dArray[AXIS_X];
				else if (pSG71Para->cEndPointPos == 2)
					SLineInfo.SEndPoint.dArray[AXIS_X] = pSG71Para->SPointC2.dArray[AXIS_X];

				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				pSG71Para->cStepInBlock = 1;

				cRet = 1;
				break;
			}
			//Z轴退刀
			case 1:
			{
				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				if (pSG71Para->cEndPointPos == 0)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA1.dArray[AXIS_Z];
				else if (pSG71Para->cEndPointPos == 1)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointA2.dArray[AXIS_Z];
				else if (pSG71Para->cEndPointPos == 2)
					SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG71Para->SPointC2.dArray[AXIS_Z];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//恢复文本获取结构体数据，继续解析NF后的NC程序
				pSAssignConfig->SGetTextStateInfo = pSG71Para->SGetTextStateInfo;

				cRet = 2;
				break;
			}

			}
			break;
		}
		}

		//判断是否需要交换当前点坐标
		if (pSG71Para->bG72 == TRUE && pSG71Para->bSwapFlag)
		{
			//G72再次交换系统状态当前点XZ坐标（还原）
			double dTemp;
			dTemp = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] = dTemp;

			pSG71Para->bSwapFlag;
		}

		//若G72写了轨迹，则交换缓冲区点XZ坐标
		if (pSG71Para->bG72 && pSGCodeParseUnit->EntityType != DOT)
			swapBufXZCoord(pSGCodeParseUnit);

		return cRet;

	}


}

//沿起始边or起始边进给轨迹段函数
int8 feed(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cFeedAxis)
{
	SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];
	bool bStartLine = FALSE;

	//获取当前点所在的槽数据
	SGrooveData* pSGrooveData;
	pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

	//找到当前点在槽链表中的位置
	bStartLine = pSG71Para->cCurrMachineDir == 1 ? TRUE : FALSE;//正向加工则在起始边上进给
	int16 cPos = findEntityPosInGroove(pSGrooveData, TRUE, &pSAssignConfig->SGCodeCurrSysState.SCurrPoint, bStartLine);

	//1.当前点在槽底
	if (pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] == pSGrooveData->SPointC.dX &&
		pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] == pSGrooveData->SPointC.dZ)
	{
		//feed完成，记录坐标
		if (pSGrooveData->bPointA2 == FALSE)
		{
			pSGrooveData->SPointA2.dX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dRestFeedVal;
			pSGrooveData->SPointA2.dZ = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSGrooveData->bPointA2 = TRUE;
		}
		if (pSGrooveData->bPointB2 == FALSE)
		{
			pSGrooveData->SPointB2.dX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] + pSG71Para->cMachineXDir * pSG71Para->dRestFeedVal;
			pSGrooveData->SPointB2.dZ = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			pSGrooveData->bPointB2 = TRUE;
		}
		return 1;
	}
	//2.当前点不在槽底
	else
	{
		//2.1找到当前点所在实体
		SLine* pEntity = (SLine*)FindNodeData(pSGrooveData->pSGroovePathList, cPos);

		if (pEntity == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.2计算该实体剩余进给量
		double dEntityRest = 0;
		if (bStartLine == TRUE)
		{			
			dEntityRest = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cFeedAxis] - pEntity->SEndPoint.dArray[cFeedAxis]);
		}
		else
		{
			dEntityRest = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cFeedAxis] - pEntity->SStartPoint.dArray[cFeedAxis]);
		}
		//2.3当前实体内，能达到目标进给量(实体剩余 > 进给剩余)
		if (isUnEqualAndGreaterThanZero(dEntityRest - pSG71Para->dRestFeedVal))
		{
			//当前点->修正轨迹终点
			SPointDouble STargetStartPoint;
			SPointDouble STargetEndPoint;
			STargetStartPoint = STargetEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

			STargetStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			STargetEndPoint.dArray[cFeedAxis] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cFeedAxis] + pSG71Para->cMachineXDir * pSG71Para->dRestFeedVal;
			//若进给方向为AXIS_X，则根据 终点X 找 终点Z
			if (cFeedAxis == AXIS_X)
			{
				findTargetPointInEntity((enum ENTITY*)pEntity, cFeedAxis, STargetEndPoint.dArray[cFeedAxis], &STargetEndPoint.dArray[AXIS_Z]);
			}
			//若进给方向为AXIS_Z，则根据 终点Z 找 终点X
			else if (cFeedAxis == AXIS_Z)
			{
				findTargetPointInEntity((enum ENTITY*)pEntity, cFeedAxis, STargetEndPoint.dArray[cFeedAxis], &STargetEndPoint.dArray[AXIS_X]);
			}

			//向缓冲区写入轨迹
			writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &STargetStartPoint, &STargetEndPoint, pSG71Para->cCurrMachineDir);

			//记录循环块A点
			pSG71Para->SCycleBlockPointA.dX = STargetEndPoint.dArray[AXIS_X];
			pSG71Para->SCycleBlockPointA.dZ = STargetEndPoint.dArray[AXIS_Z];

			//feed完成，记录坐标
			if (bStartLine == TRUE && pSGrooveData->bPointA2 == FALSE)
			{
				pSGrooveData->SPointA2.dX = STargetEndPoint.dArray[AXIS_X];
				pSGrooveData->SPointA2.dZ = STargetEndPoint.dArray[AXIS_Z];
				pSGrooveData->bPointA2 = TRUE;
			}
			if (bStartLine == FALSE && pSGrooveData->bPointB2 == FALSE)
			{
				pSGrooveData->SPointB2.dX = STargetEndPoint.dArray[AXIS_X];
				pSGrooveData->SPointB2.dZ = STargetEndPoint.dArray[AXIS_Z];
				pSGrooveData->bPointB2 = TRUE;
			}

			return 1;
		}
		
		//2.4当前实体内，不能达到目标进给量
		else
		{
			//修正剩余进给距离
			pSG71Para->dRestFeedVal = pSG71Para->dRestFeedVal - dEntityRest;

			SPointDouble STargetStartPoint;
			STargetStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			//若为起始边，则写入当前点->实体终点
			if (bStartLine == TRUE)
			{
				writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &STargetStartPoint, &pEntity->SEndPoint, pSG71Para->cCurrMachineDir);
			}
			//若为终止边，则写入当前点->实体起点
			else
			{
				writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &STargetStartPoint, &pEntity->SStartPoint, pSG71Para->cCurrMachineDir);
			}
			//feed未完成
			return 0;
		}
	}

}

//回切
int8 back(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cBackAxis)
{
	SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];
	bool bStartLine = FALSE;

	//获取当前点所在的槽数据
	SGrooveData* pSGrooveData;
	pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

	//找到当前点在槽链表中的位置
	bStartLine = pSG71Para->cCurrMachineDir == 1 ? FALSE : TRUE;//正向加工回切在终止边上
	int16 cPos = findEntityPosInGroove(pSGrooveData, FALSE, &pSAssignConfig->SGCodeCurrSysState.SCurrPoint, bStartLine);
	//当前点在槽底时cPos判断不准
	//if (isZero(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] - pSGrooveData->SPointC.dX) &&
	//	isZero(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] - pSGrooveData->SPointC.dZ) &&
	//	pSG71Para->cCurrMachineDir == 1)
	//{
	//	cPos--;
	//}

	//1.当前点在起始边or终止边顶点
	if ((pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] == pSGrooveData->SPointA.dX && pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] == pSGrooveData->SPointA.dZ) ||
		(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X] == pSGrooveData->SPointB.dX && pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z] == pSGrooveData->SPointB.dZ))
	{
		//写入轨迹
		SLine SLineInfo;
		memset(&SLineInfo, 0, sizeof(SLine));
		SLineInfo.EntityType = L1NE;
		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		SLineInfo.SEndPoint.dArray[cBackAxis] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cBackAxis] - pSG71Para->cMachineXDir * pSG71Para->dRestBackVal;
		
		//20250920企业要求回切时不能超出槽边
		SLineInfo.SEndPoint = SLineInfo.SStartPoint;
		
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		//判断是否超出顶点
		if (!isZero(pSG71Para->dRestBackVal))
		{
			pSG71Para->bOutOfEdge = TRUE;
			pSG71Para->dOutVal = pSG71Para->dRestBackVal;
		}
		pSG71Para->dRestBackVal = 0;

		//back完成
		return 1;
	}
	//2.当前点不在起始边or终止边顶点
	else
	{
		//2.1找到当前点所在实体
		SLine* pEntity = (SLine*)FindNodeData(pSGrooveData->pSGroovePathList, cPos);

		//2.2计算该实体剩余回切量
		double dEntityRest;
		if (bStartLine == TRUE)
		{
			dEntityRest = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cBackAxis] - pEntity->SStartPoint.dArray[cBackAxis]);
		}
		else
		{
			dEntityRest = fabs(pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cBackAxis] - pEntity->SEndPoint.dArray[cBackAxis]);
		}

		//2.3当前实体内，能达到目标回切量(实体剩余 > 回切剩余)
		if (isUnEqualAndGreaterThanZero(dEntityRest - pSG71Para->dRestBackVal))
		{
			//当前点->修正轨迹终点
			SPointDouble STargetStartPoint;
			SPointDouble STargetEndPoint;
			STargetStartPoint = STargetEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

			STargetStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			STargetEndPoint.dArray[cBackAxis] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[cBackAxis] - pSG71Para->cMachineXDir * pSG71Para->dRestBackVal;
			if (cBackAxis == AXIS_X)
			{			
				findTargetPointInEntity((enum ENTITY*)pEntity, AXIS_X, STargetEndPoint.dArray[AXIS_X], &STargetEndPoint.dArray[AXIS_Z]);
			}
			else if (cBackAxis == AXIS_Z)
			{
				findTargetPointInEntity((enum ENTITY*)pEntity, AXIS_Z, STargetEndPoint.dArray[AXIS_Z], &STargetEndPoint.dArray[AXIS_X]);
			}

			writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &STargetStartPoint, &STargetEndPoint, pSG71Para->cCurrMachineDir);

			//修正剩余进给距离
			pSG71Para->dRestBackVal = 0;
		}
		//2.4当前实体内，不能达到目标回切量
		else
		{
			//修正剩余进给距离
			pSG71Para->dRestBackVal = pSG71Para->dRestBackVal - dEntityRest;
	
			SPointDouble STargetStartPoint;
			STargetStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

			//若为起始边，则写入当前点->实体起点
			if (bStartLine == TRUE)
			{
				writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &STargetStartPoint, &pEntity->SStartPoint, pSG71Para->cCurrMachineDir);
			}
			//若为终止边，则写入当前点->实体终点
			else
			{
				writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &STargetStartPoint, &pEntity->SEndPoint, pSG71Para->cCurrMachineDir);
			}
		}

		//back未完成
		return 0;

	}

}

//正向切削轨迹段函数
int8 cut_1(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cCutAxis)
{
	SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];
	SPoint SCutEndPoint = { 0 };
	int16 cTargetGrooveNum = 0;

	//1.计算切削终点坐标
	//1.1切削方向为AXIS_Z，则根据 切削终点X 找 切削终点Z
	if (cCutAxis == AXIS_Z)
	{	
		SCutEndPoint.dX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
		cTargetGrooveNum = findCutEndPoint(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum, AXIS_X, SCutEndPoint.dX, 1, &SCutEndPoint.dZ, pSG71Para->cMachineXDir);
	}
	//1.2切削方向为AXIS_X，则根据 切削终点Z 找 切削终点X
	else if (cCutAxis == AXIS_X)
	{
		SCutEndPoint.dZ = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
		cTargetGrooveNum = findCutEndPoint(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum, AXIS_Z, SCutEndPoint.dZ, 1, &SCutEndPoint.dX, pSG71Para->cMachineXDir);
	}

	//2.写入切削轨迹
	SLine SLineInfo;
	memset(&SLineInfo, 0, sizeof(SLine));
	SLineInfo.EntityType = L1NE;
	SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
	SLineInfo.SEndPoint.dArray[AXIS_X] = SCutEndPoint.dX;
	SLineInfo.SEndPoint.dArray[AXIS_Z] = SCutEndPoint.dZ;
	writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

	//3.修正当前点所在槽号
	pSG71Para->cCurrPointGrooveNum = cTargetGrooveNum;

	//4.判断是否第一次切削到终止边
	SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
	if (pSGrooveData->bPointB2 == FALSE)
	{
		pSGrooveData->SPointB2.dX = SLineInfo.SEndPoint.dArray[AXIS_X];
		pSGrooveData->SPointB2.dZ = SLineInfo.SEndPoint.dArray[AXIS_Z];
		pSGrooveData->bPointB2 = TRUE;
	}

	return 1;
}

//反向切削轨迹段函数
int8 cut_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cCutAxis)
{
	SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];

	switch (pSG71Para->cStepInCut_2)
	{
	//切削
	case 0:
	{
		SPoint SCutEndPoint = { 0 };
		int16 cTargetGrooveNum = 0;

		//读取切削深度
		pSG71Para->dCurrCutDepth = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];

		//1.计算切削终点坐标
		//1.1切削方向为AXIS_Z，则根据 切削终点X 找 切削终点Z
		if (cCutAxis == AXIS_Z)
		{
			SCutEndPoint.dX = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			cTargetGrooveNum = findCutEndPoint(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum, AXIS_X, SCutEndPoint.dX, -1, &SCutEndPoint.dZ, pSG71Para->cMachineXDir);
		}
		//1.2切削方向为AXIS_X，则根据 切削终点Z 找 切削终点X
		else if (cCutAxis == AXIS_X)
		{
			SCutEndPoint.dZ = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];
			cTargetGrooveNum = findCutEndPoint(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum, AXIS_Z, SCutEndPoint.dZ, -1, &SCutEndPoint.dX, pSG71Para->cMachineXDir);
		}

		//2.写入切削轨迹
		SLine SLineInfo;
		memset(&SLineInfo, 0, sizeof(SLine));
		SLineInfo.EntityType = L1NE;
		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		SLineInfo.SEndPoint.dArray[AXIS_X] = SCutEndPoint.dX;
		SLineInfo.SEndPoint.dArray[AXIS_Z] = SCutEndPoint.dZ;
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);


		//3.修正当前点所在槽号
		pSG71Para->cCurrPointGrooveNum = cTargetGrooveNum;

		//4.判断是否第一次切削到起始边
		SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
		if (pSGrooveData->bPointA2 == FALSE)
		{
			pSGrooveData->SPointA2.dX = SLineInfo.SEndPoint.dArray[AXIS_X];
			pSGrooveData->SPointA2.dZ = SLineInfo.SEndPoint.dArray[AXIS_Z];
			pSGrooveData->bPointA2 = TRUE;
		}
		//反向切削完成
		if (cTargetGrooveNum == pSG71Para->cCurrMachineGrooveNum)
		{
			//cut_2完成
			return 1;
		}
		//反向切削未完成，需要跨越
		else
		{
			//获取目标槽数据
			SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, cTargetGrooveNum);

			//计算 跨越回切与跨越进给的 余量
			if (cCutAxis == AXIS_X)
			{
				//余量
				pSG71Para->dRestFeedVal = fabs(pSG71Para->dRestBackVal = pSGrooveData->SPointA.dZ - SLineInfo.SEndPoint.dArray[AXIS_Z]);
			}
			else if (cCutAxis == AXIS_Z)
			{
				//余量
				pSG71Para->dRestFeedVal = fabs(pSG71Para->dRestBackVal = pSGrooveData->SPointA.dX - SLineInfo.SEndPoint.dArray[AXIS_X]);
			}
				
			pSG71Para->cStepInCut_2 = 1;
			//cut_2未完成
			return 0;
		}

	}
	//跨越回切
	case 1:
	{
		int8 cBackAxis = cCutAxis == AXIS_Z ? AXIS_X : AXIS_Z;
		if (back(pSAssignConfig, pSGCodeParseUnit, cBackAxis))
		{
			pSG71Para->cCurrPointGrooveNum--;
			SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
			//跨越进给不会到达槽底
			if (isUnEqualAndGreaterThanZero(fabs(pSGrooveData->SPointB.dX - pSGrooveData->SPointC.dX) - pSG71Para->dRestFeedVal))
			{
				//下一步执行跨越进给
				pSG71Para->cStepInCut_2 = 2;
			}
			//跨越进给到达槽底
			else
			{
				//下一步执行沿槽轨迹加工
				pSG71Para->cStepInCut_2 = 3;
			}
		}
		return 0;
	}
	//跨越进给
	case 2:
	{
		int8 cFeedAxis = cCutAxis == AXIS_Z ? AXIS_X : AXIS_Z;
		if (feed(pSAssignConfig, pSGCodeParseUnit, cFeedAxis))
		{
			pSG71Para->cStepInCut_2 = 0;
		}
		if (pSGCodeParseUnit->EntityType == OTHERTYPE)
		{
			pSGCodeParseUnit->EntityType = L1NE;
		}
		return 0;
	}
	//沿槽轨迹进给加工
	case 3:
	{
		SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

		int16 cPos = findEntityPosInGroove(pSGrooveData, FALSE, &pSAssignConfig->SGCodeCurrSysState.SCurrPoint, FALSE);

		//2.1找到当前点所在实体
		SLine* pEntity = (SLine*)FindNodeData(pSGrooveData->pSGroovePathList, cPos);
		if (pEntity == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//写入实体
		writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &pEntity->SEndPoint, &pEntity->SStartPoint, pSG71Para->cCurrMachineDir);

		//槽轨迹终止边加工完成
		if (cPos == pSGrooveData->cTotalEndLineEntityNum - 1)
		{
			//槽加工完成
			pSGrooveData->bFinished = TRUE;

			pSG71Para->cCurrPointGrooveNum--;

			pSG71Para->dRestFeedVal = fabs(pEntity->SStartPoint.dArray[AXIS_X] - pSG71Para->dCurrCutDepth);

			//下一步执行沿槽轨迹回切加工
			pSG71Para->cStepInCut_2 = 4;

		}
		return 0;
	}
	//沿槽轨迹回切加工
	case 4:
	{
		SGrooveData* pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);
		
		int16 cPos = findEntityPosInGroove(pSGrooveData, FALSE, &pSAssignConfig->SGCodeCurrSysState.SCurrPoint, TRUE);

		//2.1找到当前点所在实体
		SLine* pEntity = (SLine*)FindNodeData(pSGrooveData->pSGroovePathList, cPos);

		if (pEntity == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		
		//写入实体
		writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeParseUnit, &pEntity->SEndPoint, &pEntity->SStartPoint, pSG71Para->cCurrMachineDir);

		//槽轨迹起始边加工完成
		if (cPos == getListSize(pSGrooveData->pSGroovePathList) - 1)
		{
			//槽加工完成
			pSGrooveData->bFinished = TRUE;

			pSG71Para->cCurrPointGrooveNum--;

			pSG71Para->dRestFeedVal = fabs(pEntity->SStartPoint.dArray[AXIS_X] - pSG71Para->dCurrCutDepth);
		
			//下一步执行跨越进给
			pSG71Para->cStepInCut_2 = 2;

		}
		return 0;
	}
	default:
		break;
	}

}

//定位
int8 locate(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cFeedAxis)
{
	SG71Para* pSG71Para = pSG71ParaAssign[pSAssignConfig->ucChannelNum];

	if (pSG71Para->bOutOfEdge == TRUE)
	{
		//获取当前点所在的槽数据
		SGrooveData* pSGrooveData;
		pSGrooveData = getGrooveDataByGrooveNum(pSG71Para->pSGrooveList, pSG71Para->cCurrPointGrooveNum);

		//写入轨迹
		SLine SLineInfo;
		memset(&SLineInfo, 0, sizeof(SLine));
		SLineInfo.EntityType = L1NE;
		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		if (cFeedAxis == AXIS_X)
		{
			//进给到槽A点
			if (isZero(SLineInfo.SEndPoint.dArray[AXIS_Z] - pSGrooveData->SPointA.dZ))
			{
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSGrooveData->SPointA.dX;
			}
			//进给到槽B点
			else if(isZero(SLineInfo.SEndPoint.dArray[AXIS_Z] - pSGrooveData->SPointB.dZ))
			{
				SLineInfo.SEndPoint.dArray[AXIS_X] = pSGrooveData->SPointB.dX;
			}
		}
		else if (cFeedAxis == AXIS_Z)
		{
			//进给到槽A点
			if (isZero(SLineInfo.SEndPoint.dArray[AXIS_X] - pSGrooveData->SPointA.dX))
			{
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSGrooveData->SPointA.dZ;
			}
			//进给到槽B点
			else if (isZero(SLineInfo.SEndPoint.dArray[AXIS_X] - pSGrooveData->SPointB.dX))
			{
				SLineInfo.SEndPoint.dArray[AXIS_Z] = pSGrooveData->SPointB.dZ;
			}
		}

		////20250920企业要求回切时不能超出槽边
		SLineInfo.SEndPoint = SLineInfo.SStartPoint;

		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

		pSG71Para->bOutOfEdge = FALSE;

		//更新进给余量
		pSG71Para->dRestFeedVal = pSG71Para->dRestFeedVal - pSG71Para->dOutVal;

		//locate未完成
		return 0;
	}
	else
	{
		if (feed(pSAssignConfig, pSGCodeParseUnit, AXIS_X))
		{
			//locate完成
			return 1;
		}
		if (pSGCodeParseUnit->EntityType == OTHERTYPE)
		{
			pSGCodeParseUnit->EntityType = L1NE;
		}
		return 0;
	}
}

//交换轨迹链表中XZ坐标
void swapPathListXZCoord(SList** ppSPathList)
{
	for (int i = 0; i < (*ppSPathList)->cSize; i++)
	{
		enum ENTITY* pEntity;
		pEntity = FindNodeData((*ppSPathList), i);
		switch (*pEntity)
		{
		case OTHERTYPE:case L1NE:case DOT:
		{			
			double dTemp;
			SLine* pSLine = (SLine*)pEntity;
			//交换起点坐标
			dTemp = pSLine->SStartPoint.dArray[AXIS_X];
			pSLine->SStartPoint.dArray[AXIS_X] = pSLine->SStartPoint.dArray[AXIS_Z];
			pSLine->SStartPoint.dArray[AXIS_Z] = dTemp;
			//交换终点坐标
			dTemp = pSLine->SEndPoint.dArray[AXIS_X];
			pSLine->SEndPoint.dArray[AXIS_X] = pSLine->SEndPoint.dArray[AXIS_Z];
			pSLine->SEndPoint.dArray[AXIS_Z] = dTemp;

			break;
		}
		case ARC:
		{
			double dTemp;
			SArc* pSArc = (SArc*)pEntity;
			//交换起点坐标
			dTemp = pSArc->SStartPoint.dArray[AXIS_X];
			pSArc->SStartPoint.dArray[AXIS_X] = pSArc->SStartPoint.dArray[AXIS_Z];
			pSArc->SStartPoint.dArray[AXIS_Z] = dTemp;
			//交换终点坐标
			dTemp = pSArc->SEndPoint.dArray[AXIS_X];
			pSArc->SEndPoint.dArray[AXIS_X] = pSArc->SEndPoint.dArray[AXIS_Z];
			pSArc->SEndPoint.dArray[AXIS_Z] = dTemp;
			//交换圆心点坐标
			dTemp = pSArc->SCenterPoint.dArray[AXIS_X];
			pSArc->SCenterPoint.dArray[AXIS_X] = pSArc->SCenterPoint.dArray[AXIS_Z];
			pSArc->SCenterPoint.dArray[AXIS_Z] = dTemp;
			//改变圆弧方向
			pSArc->cArcDir *= -1;

			break;
		}
		default:
			break;
		}
	}
	
}

//交换缓冲区中XZ坐标
void swapBufXZCoord(struct SGCodeParse* pSGCodeParseUnit)
{
	//交换起点坐标
	swapVal(&pSGCodeParseUnit->SStartPoint.dArray[AXIS_X], &pSGCodeParseUnit->SStartPoint.dArray[AXIS_Z]);
	//交换终点坐标
	swapVal(&pSGCodeParseUnit->SEndPoint.dArray[AXIS_X], &pSGCodeParseUnit->SEndPoint.dArray[AXIS_Z]);
	//交换圆心点坐标
	swapVal(&pSGCodeParseUnit->SCenterPoint.dArray[AXIS_X], &pSGCodeParseUnit->SCenterPoint.dArray[AXIS_Z]);
	//改变圆弧方向
	pSGCodeParseUnit->cArcDir *= -1;
}

void swapVal(double* pA, double* pB)
{
	double dTemp;
	dTemp = *pA;
	*pA = *pB;
	*pB = dTemp;
}

/*************************************************
  函数名称：void freeGroovePathList(void * pData)
  函数功能: 释放G71槽结构体中的槽轨迹链表
  输入参数：*pData void 节点数据域指针
  输出参数：
  返 回 值：

  备    注：pData指向G71槽链表中的槽数据结构体，即槽链表节点的数据域
**************************************************/
void freeGroovePathList(void * pData)
{
	//链表为空，退出
	if (pData ==NULL)
	{
		return;
	}
	//当前节点（从头节点开始）
	SGrooveData*pSGrooveData=(SGrooveData*)pData;//类型转换为槽数据结构体
	SList* pGroovePath = pSGrooveData->pSGroovePathList;//获取该槽数据结构体内部的槽轨迹链表

	//下一个节点
	if (pGroovePath !=NULL)
	{
		//释放该槽数据结构体内部的槽轨迹链表
		freeList(&pGroovePath);
	}

	return;

}

















