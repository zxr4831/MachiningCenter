/*
ns～nf程序段中，只能有G功能：
Ga组指令：G00、G01、G02、G03、G04、G05、G6.2、G6.3、G7.2、G7.3
Gz组指令：G04
速度控制指令：G96、G97、G98、G99
刀补指令：G40、G41、G42				pSAssignConfig->SGetTextStateInfo = pSG73Para->SGetTextStateInfo;

*/


#include <math.h>
#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"
#include "GeneralFunctions.h"
#include "GCode.h"


static SG73Para* pSG73ParaCheck[CHANNEL_NUM_MAX];
static SG73Para* pSG73ParaAssign[CHANNEL_NUM_MAX];


//添加G73规则结构体
void addG73RegulationAPI()
{
	//指令：G73
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(73, sGz, 2, mallocG73ParaStruct, freeG73ParaStruct, getG73Para, CheckG73, AssignG73);
}

//参数结构体初始化函数？


/*************************************************
  函数名称：int8 mallocG73ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G73参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG73ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG73ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG73Para), ORAM_NUM422);//暂未对内存编号
		if (pSG73ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG73ParaCheck[ucChannelNum], 0, sizeof(SG73Para));
		//pSG73ParaAssign[ucChannelNum]->SPathList = initList();
	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG73ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG73Para), ORAM_NUM423);
		if (pSG73ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG73ParaAssign[ucChannelNum], 0, sizeof(SG73Para));
		//pSG73ParaAssign[ucChannelNum]->SPathList = initList();
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
int8 freeG73ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG73ParaCheck[ucChannelNum] != NULL)
		{
			if (pSG73ParaCheck[ucChannelNum]->pSPathList!=NULL)
			{
				//释放G73轨迹链表
				freeList(&pSG73ParaCheck[ucChannelNum]->pSPathList);
			}
			if (pSG73ParaCheck[ucChannelNum]->pSCurrMachinePathList != NULL)
			{
				freeList(&pSG73ParaCheck[ucChannelNum]->pSCurrMachinePathList);
			}
			//释放G73参数结构体
			freeGCode(pSG73ParaCheck[ucChannelNum]);
			pSG73ParaCheck[ucChannelNum] = NULL;
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG73ParaAssign[ucChannelNum] != NULL)
		{
			//G73轨迹链表不为空
			if (pSG73ParaAssign[ucChannelNum]->pSPathList != NULL)
			{
				//释放G73轨迹链表
				freeList(&pSG73ParaAssign[ucChannelNum]->pSPathList);
			}
			if (pSG73ParaAssign[ucChannelNum]->pSCurrMachinePathList != NULL)
			{
				freeList(&pSG73ParaAssign[ucChannelNum]->pSCurrMachinePathList);
			}
			//释放G73参数结构体
			freeGCode(pSG73ParaAssign[ucChannelNum]);
			pSG73ParaAssign[ucChannelNum] = NULL;
			return 1;
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
int8 getG73Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
	//1.检查
	if (ucMode == GC_SYNTAX)
	{
		SSyntaxConfiguration* pSSyntaxConfig = &pChannelConfig->SSyntaxConfig;
		SG73Para* pSG73Para = pSG73ParaCheck[pSSyntaxConfig->ucChannelNum];

		// 仅显式输入了 R(D) 才做校验
		if (pSSyntaxConfig->SScanContainer.bRIJK[0] == TRUE)
		{
			// R 是否整数——R1.5、R2.3 等小数拒绝
			if (checkParaInteger('R', &pSSyntaxConfig->SScanContainer) == -1)
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM102);
				return -1;
			}
			// R取值范围 1~9999——R0、R-1、R10000 越界拒绝
			if (checkParaRange('R', &pSSyntaxConfig->SScanContainer, 1, 9999) == -1)
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM102);
				return -1;
			}
		}

		//1.1检查时，获取第一程序段的参数
		if (pSSyntaxConfig->cCommandSegmentCnt == 1)
		{
			pSG73Para->dDeltaI = pSScanContainer->bUVW[0] == TRUE ? pSScanContainer->dUVW[0] : 0;
			pSG73Para->dDeltaK = pSScanContainer->bUVW[2] == TRUE ? pSScanContainer->dUVW[2] : 0;
			pSG73Para->ucD = pSScanContainer->bRIJK[0] == TRUE ? pSScanContainer->dRIJK[0] : 0;
			pSG73Para->ucH = pSScanContainer->dH;


		}
		//1.2检查时，获取第二程序段的参数
		else if (pSSyntaxConfig->cCommandSegmentCnt == 2)
		{
			pSG73Para->unNS = pSScanContainer->dP;
			pSG73Para->unNF = pSScanContainer->dQ;
			pSG73Para->dDeltaU = pSScanContainer->bUVW[0] == TRUE ? pSScanContainer->dUVW[0] : 0;
			pSG73Para->dDeltaW = pSScanContainer->bUVW[2] == TRUE ? pSScanContainer->dUVW[2] : 0;
		}

	}
	//2.赋值
	else if (ucMode == GC_ASSIGN)
	{
		SAssignConfiguration* pSAssignConfig = &pChannelConfig->SAssignConfig;       //赋值结构体
		SG73Para* pSG73Para = pSG73ParaAssign[pSAssignConfig->ucChannelNum];
		//2.1赋值时，获取第一程序段的参数
		if (pSAssignConfig->cCommandSegmentCnt == 1)
		{
			pSG73Para->dDeltaI = pSScanContainer->bUVW[0] == TRUE ? pSScanContainer->dUVW[0] : 0;
			pSG73Para->dDeltaK = pSScanContainer->bUVW[2] == TRUE ? pSScanContainer->dUVW[2] : 0;
			pSG73Para->ucD = pSScanContainer->bRIJK[0] == TRUE ? pSScanContainer->dRIJK[0] : 0;
			pSG73Para->ucH = pSScanContainer->dH;
		}
		//2.2赋值时，获取第二程序段的参数
		else if (pSAssignConfig->cCommandSegmentCnt == 2)
		{
			pSG73Para->unNS = pSScanContainer->dP;
			pSG73Para->unNF = pSScanContainer->dQ;
			pSG73Para->dDeltaU = pSScanContainer->bUVW[0] == TRUE ? pSScanContainer->dUVW[0] : 0;
			pSG73Para->dDeltaW = pSScanContainer->bUVW[2] == TRUE ? pSScanContainer->dUVW[2] : 0;
		}
	}
	return 1;
}

int8 CheckG73(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	SG73Para* pSG73Para = pSG73ParaCheck[pSSyntaxConfig->ucChannelNum];

	// 参数检查
	// 查 P/Q 段号是否存在、是否整数、范围是否合法，以及 U/W 精加工余量
	
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
	// X 轴精加工余量 U 取值范围 -99999.999 ~ 99999.999
	if (!(-99999.999 <= pSG73Para->dDeltaU && pSG73Para->dDeltaU <= 99999.999))
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
		return -1;
	}
	// Z 轴精加工余量 W 取值范围 -99999.999 ~ 99999.999
	if (!(-99999.999 <= pSG73Para->dDeltaW && pSG73Para->dDeltaW <= 99999.999))
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
		return -1;
	}
	

	//4.检查指令参数范围
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

int8 AssignG73(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	SG73Para* pSG73Para = pSG73ParaAssign[pSAssignConfig->ucChannelNum];

	//G73Ⅰ型
	if (pSG73Para->ucH == 0)
	{
		switch (pSG73Para->cStep)
		{
		//0.获取最外层轨迹链表
		case 0:
		{
			//
			int8 cRet;
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//2.获取当前加工轨迹链表（这里获取的轨迹偏移值）
			double dDeltaX = pSG73Para->dDeltaU / 2 + pSG73Para->dDeltaI;
			double dDeltaZ = pSG73Para->dDeltaW + pSG73Para->dDeltaK;
			cRet = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG73Para->unNS, pSG73Para->unNF, dDeltaX, dDeltaZ, &pSG73Para->pSPathList, &pSG73Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, G71);
			if (cRet == 1)
			{
				if( pSG73Para->pSPathList== NULL ||pSG73Para->pSPathList->cSize<=0)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM300);
					return -1;
				}
				pSG73Para->cStep = 1;
			}
			else if (cRet == -1)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			return 0;//未写入轨迹
			break;
		}
		//1.赋值预处理
		case 1:
		{
			//1.0获取A点坐标
			pSG73Para->SPointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

			//1.4获取轨迹加工方向
			//getPathMachineDir(pSG73Para->pSPathList, &pSG73Para->cMachineXDir, &pSG73Para->cMachineZDir);
			pSG73Para->cCurrMachineDir = 1;
			//1.6
			pSG73Para->cStep = 2;

			pSG73Para->dRestDeltaI = pSG73Para->dDeltaI;
			pSG73Para->dRestDeltaK = pSG73Para->dDeltaK;

			pSG73Para->cRestCount = pSG73Para->ucD;

			return 0;//未写入轨迹

			break;

		}
		//2.写入轨迹当前点-轨迹链表起点（快速运动）
		case 2:
		{
			//计算An点坐标
			SLine* pSLine = FindNodeData(pSG73Para->pSPathList, pSG73Para->pSPathList->cSize - 1);
			SPointDouble SPointAn = pSLine->SStartPoint;

			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint = SPointAn;
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//下一步写当前加工轨迹链表
			pSG73Para->cStep = 3;
			pSG73Para->usCurPos = pSG73Para->pSPathList->cSize - 1;

			return 1;//写入轨迹，但指令轨迹未写完

			break;
		}
		//3.写入当前加工轨迹链表
		case 3:
		{
			int8 cRet;

			//将轨迹链表中的实体写入缓冲区
			uint16 usStartPos = 0;
			uint16 usEndPos = pSG73Para->pSPathList->cSize - 1;
			cRet = writePathListIntoGCodeBuf(pSG73Para->pSPathList, usStartPos, usEndPos, &pSG73Para->usCurPos, pSGCodeParseUnit, pSG73Para->cCurrMachineDir);

			//判断当前加工轨迹链表实体是否全部写入缓冲区
			if (cRet)
			{
				//若循环切削次数 >= 0，则下一次移动到An点，反之移动到A点
				if (pSG73Para->cRestCount > 0)
				{
					pSG73Para->cStep = 2;

					//计算下一次加工轨迹相对于当前轨迹的偏移量
					double dDeltaX = -pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
					double dDeltaZ = -pSG73Para->dRestDeltaK / pSG73Para->cRestCount;
					
					//圆整偏移量(最小单位0.001mm)
					dDeltaX = round(dDeltaX * 1000) / 1000.0;
					dDeltaZ = round(dDeltaZ * 1000) / 1000.0;

					//修正dRestDeltaI与dRestDeltaK
					pSG73Para->dRestDeltaI += dDeltaX;
					pSG73Para->dRestDeltaK += dDeltaZ;

					//偏置当前加工轨迹
					offsetPathList(&pSG73Para->pSPathList, dDeltaX, dDeltaZ);

					pSG73Para->cRestCount--;
				}
				else
				{
					pSG73Para->cStep = 4;
				}
			}

			return 1;
			break;
		}
		//4.写入轨迹当前点-A（快速运动）
		case 4:
		{
			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint = pSG73Para->SPointA;
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			pSAssignConfig->SGetTextStateInfo = pSG73Para->SGetTextStateInfo;

			return 2;
			break;
		}

		default:
			break;
		}

	}
	//G73Ⅱ型
	else if (pSG73Para->ucH == 1)
	{
		switch (pSG73Para->cStep)
		{
		//0.获取NSNF轨迹链表
		case 0:
		{
			//
			int8 cRet;
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//2.获取当前加工轨迹链表（这里获取的轨迹偏移值）
			double dDeltaX = pSG73Para->dDeltaU / 2;
			double dDeltaZ = pSG73Para->dDeltaW;
			cRet = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG73Para->unNS, pSG73Para->unNF, dDeltaX, dDeltaZ, &pSG73Para->pSPathList, &pSG73Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, G71);
			if (cRet == 1)
			{
				pSG73Para->cStep = 1;

#if G73_DEBUG
			printf("-----------------------------原始的轨迹----------------------------\n");
			printList(pSG73Para->pSPathList, printDataG71G72I);
			printf("\n");
#endif
			}
			if (cRet == -1)
			{
				return -1;
			}

			return 0;//未写入轨迹
			break;
		}
		//1.赋值预处理
		case 1:
		{
			//1.0获取A点坐标
			pSG73Para->SPointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

			//1.4获取轨迹加工方向
			//getPathMachineDir(pSG73Para->pSPathList, &pSG73Para->cMachineXDir, &pSG73Para->cMachineZDir);
			pSG73Para->cCurrMachineDir = 1;
			//1.6
			pSG73Para->cStep = 2;

			pSG73Para->dRestDeltaI = pSG73Para->dDeltaI;
			pSG73Para->dRestDeltaK = pSG73Para->dDeltaK;

			pSG73Para->cRestCount = pSG73Para->ucD;

			//去除原始轨迹中的进刀段，退刀段（不修正导致等距刀路效果不佳）
			removeEntryAndExitPath(pSG73Para->pSPathList, AXIS_X, TRUE, FALSE);

			return 0;//未写入轨迹

			break;

		}
		//2.计算当前加工轨迹链表
		case 2:
		{
			double dDeltaX = pSG73Para->dRestDeltaI;
			double dDeltaZ = pSG73Para->dRestDeltaK;
			int8 cOffsetRet = offsetPathList_2(pSG73Para->pSPathList, &pSG73Para->pSCurrMachinePathList, dDeltaX, dDeltaZ);
		    if (cOffsetRet != 0)
		    {
		        // offsetPathList_2 内部已释放临时链表，此处不再推进 cStep
		        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
		        return -1;
		    }

#if G73_DEBUG
			printf("-----------------------------当前层的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif

			pSG73Para->cStep = 3;

			return 0;
			break;
		}
		//3.分割当前加工轨迹
		case 3:
		{
			int8 cOffsetRet = splitPathByBlank(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
		    if (cOffsetRet != 0)
			{
				// offsetPathList_2 内部已释放临时链表，此处不再推进 cStep
				return -1;
			}
#if G73_DEBUG
			printf("-----------------------------分割后的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif
			pSG73Para->cStep = 4;

			return 0;
			break;
		}
		//4.删除与添加轨迹中的实体
		case 4:
		{
			//删除毛坯外的实体
			deleteEntityInPath(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
#if G73_DEBUG
			printf("-----------------------------删除后的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif
			//补充轨迹中的实体
			int8 cOffsetRet = addEntityInPath(pSG73Para->pSCurrMachinePathList);
		    if (cOffsetRet != 0)
		    {
		        // offsetPathList_2 内部已释放临时链表，此处不再推进 cStep
		        return -1;
		    }
#if G73_DEBUG
			printf("-----------------------------添加后的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif
			//加工轨迹无实体
			if (pSG73Para->pSCurrMachinePathList->cSize > 0)
			{
				pSG73Para->cStep = 5;
			}
			else
			{
				//释放当前加工轨迹链表
				freeList(&pSG73Para->pSCurrMachinePathList);

				//下一步计算加工轨迹链表
				pSG73Para->cStep = 2;

				//修正dRestDeltaI与dRestDeltaK
				pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
				pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;

				pSG73Para->cRestCount--;

			}
			return 0;
			break;
		}
		//5.写入当前点-轨迹链表起点(Z轴)（快速运动）
		case 5:
		{
			//计算An点坐标
			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
			SPointDouble SPathStartPoint = pSLine->SStartPoint;

			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathStartPoint.dArray[AXIS_Z];
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//下一步写当前加工轨迹链表
			pSG73Para->cStep = 6;

			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;

			return 1;//写入轨迹，但指令轨迹未写完

			break;

		}
		//6.写入当前点-轨迹链表起点(X轴)（快速运动）
		case 6:
		{
			//计算An点坐标
			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
			SPointDouble SPathStartPoint = pSLine->SStartPoint;

			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint.dArray[AXIS_X] = SPathStartPoint.dArray[AXIS_X];
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			//下一步写当前加工轨迹链表
			pSG73Para->cStep = 7;

			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;

			return 1;//写入轨迹，但指令轨迹未写完

			break;

		}
		//7.写入轨迹链表中所有实体
		case 7:
		{
			int8 cRet;

			//将轨迹链表中的实体写入缓冲区
			uint16 usStartPos = 0;
			uint16 usEndPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
			cRet = writePathListIntoGCodeBuf(pSG73Para->pSCurrMachinePathList, usStartPos, usEndPos, &pSG73Para->usCurPos, pSGCodeParseUnit, pSG73Para->cCurrMachineDir);

			//判断当前加工轨迹链表实体是否全部写入缓冲区
			if (cRet)
			{
				//若循环切削次数 >= 0，则下一次移动到An点，反之移动到A点
				if (pSG73Para->cRestCount > 0)
				{
					//释放当前加工轨迹链表
					freeList(&pSG73Para->pSCurrMachinePathList);

					//下一步计算加工轨迹链表
					pSG73Para->cStep = 2;

					//修正dRestDeltaI与dRestDeltaK
					pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
					pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;

					pSG73Para->cRestCount--;
				}
				else
				{
					pSG73Para->cStep = 8;
				}
			}

			return 1;
			break;

		}
		//8.G73Ⅱ型计算完成
		case 8:
		{
			//写入轨迹当前点-G73起刀点（快速运动）
			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint = pSG73Para->SPointA;
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			pSAssignConfig->SGetTextStateInfo = pSG73Para->SGetTextStateInfo;

			return 2;
		}
		default:
			break;
		}

	}
	//G73Ⅲ型
	else if (pSG73Para->ucH == 2)
	{
		switch (pSG73Para->cStep)
		{
		//0.获取NSNF轨迹链表
		case 0:
		{
			//
			int8 cRet;
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//2.获取当前加工轨迹链表（这里获取的轨迹偏移值）
			double dDeltaX = pSG73Para->dDeltaU / 2;
			double dDeltaZ = pSG73Para->dDeltaW;
			cRet = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG73Para->unNS, pSG73Para->unNF, dDeltaX, dDeltaZ, &pSG73Para->pSPathList, &pSG73Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, G71);
			if (cRet == 1)
			{
				pSG73Para->cStep = 1;

#if G73_DEBUG
				printf("-----------------------------原始的轨迹----------------------------\n");
				printList(pSG73Para->pSPathList, printDataG71G72I);
				printf("\n");
#endif
			}
			else if (cRet == -1)
			{
				return -1;
			}


			return 0;//未写入轨迹
			break;
		}
		//1.赋值预处理
		case 1:
		{
			//1.0获取A点坐标
			pSG73Para->SPointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

			//1.4获取轨迹加工方向
			//getPathMachineDir(pSG73Para->pSPathList, &pSG73Para->cMachineXDir, &pSG73Para->cMachineZDir);
			pSG73Para->cCurrMachineDir = 1;
			//1.6
			pSG73Para->cStep = 2;

			pSG73Para->dRestDeltaI = pSG73Para->dDeltaI;
			pSG73Para->dRestDeltaK = pSG73Para->dDeltaK;

			pSG73Para->cRestCount = pSG73Para->ucD;

			//去除原始轨迹中的进刀段，退刀段（不修正导致等距刀路效果不佳）
			removeEntryAndExitPath(pSG73Para->pSPathList, AXIS_X, TRUE, FALSE);

			return 0;//未写入轨迹

			break;

		}
		//2.计算当前加工轨迹链表
		case 2:
		{
			double dDeltaX = pSG73Para->dRestDeltaI;
			double dDeltaZ = pSG73Para->dRestDeltaK;
			int8 cOffsetRet = offsetPathList_2(pSG73Para->pSPathList, &pSG73Para->pSCurrMachinePathList, dDeltaX, dDeltaZ);
		    if (cOffsetRet != 0)
		    {
		        // offsetPathList_2 内部已释放临时链表，此处不再推进 cStep
		        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
		        return -1;
		    }

#if G73_DEBUG
			printf("-----------------------------当前层的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif

			pSG73Para->cStep = 3;

			return 0;
			break;
		}
		//3.分割当前加工轨迹
		case 3:
		{
			int8 cOffsetRet = splitPathByBlank(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
		    if (cOffsetRet != 0)
			{
				// offsetPathList_2 内部已释放临时链表，此处不再推进 cStep
				return -1;
			}
#if G73_DEBUG
			printf("-----------------------------分割后的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif
			pSG73Para->cStep = 4;

			return 0;
			break;
		}
		//4.删除与添加轨迹中的实体
		case 4:
		{
			//删除毛坯外的实体
			deleteEntityInPath(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
#if G73_DEBUG
			printf("-----------------------------删除后的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif
			//补充轨迹中的实体
			int8 cOffsetRet = addEntityInPath(pSG73Para->pSCurrMachinePathList);
		    if (cOffsetRet != 0)
		    {
		        // offsetPathList_2 内部已释放临时链表，此处不再推进 cStep
		        return -1;
		    }
#if G73_DEBUG
			printf("-----------------------------添加后的轨迹----------------------------\n");
			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
			printf("\n");
#endif
			//加工轨迹无实体
			if (pSG73Para->pSCurrMachinePathList->cSize > 0)
			{
				pSG73Para->cStep = 5;
			}
			else
			{
				//释放当前加工轨迹链表
				freeList(&pSG73Para->pSCurrMachinePathList);

				//下一步计算加工轨迹链表
				pSG73Para->cStep = 2;

				//修正dRestDeltaI与dRestDeltaK
				pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
				pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;

				pSG73Para->cRestCount--;

			}
			return 0;
			break;
		}
		//5.写入当前点-轨迹链表起点(Z轴)正向（快速运动）
		//5.写入当前点-轨迹链表终点(Z轴)反向（快速运动）
		case 5:
		{
			//当前为正向，写入当前点-轨迹链表起点(Z轴)
			if (pSG73Para->cCurrMachineDir == 1)
			{
				//计算当前轨迹起点坐标
				SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
				SPointDouble SPathStartPoint = pSLine->SStartPoint;

				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathStartPoint.dArray[AXIS_Z];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步写当前加工轨迹链表
				pSG73Para->cStep = 6;

				pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;	
			}
			//当前为反向，写入当前点-轨迹链表终点(Z轴)
			else
			{
				//计算当前轨迹终点坐标
				SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, 0);
				SPointDouble SPathEndPoint = pSLine->SEndPoint;

				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathEndPoint.dArray[AXIS_Z];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步写当前加工轨迹链表
				pSG73Para->cStep = 6;

				pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;

			}
			return 1;//写入轨迹，但指令轨迹未写完

			break;

		}
		//6.写入当前点-轨迹链表起点(X轴)正向（快速运动）
		//6.写入当前点-轨迹链表终点(X轴)反向（快速运动）
		case 6:
		{
			//当前为正向，写入当前点-轨迹链表起点(X轴)
			if (pSG73Para->cCurrMachineDir == 1)
			{
				SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
				SPointDouble SPathStartPoint = pSLine->SStartPoint;

				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = SPathStartPoint.dArray[AXIS_X];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步写当前加工轨迹链表
				pSG73Para->cStep = 7;

				pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
			}
			//当前为反向，写入当前点-轨迹链表终点(Z轴)
			else
			{
				SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, 0);
				SPointDouble SPathEndPoint = pSLine->SEndPoint;

				SLine SLineInfo;
				memset(&SLineInfo, 0, sizeof(SLine));
				SLineInfo.EntityType = OTHERTYPE;
				SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
				SLineInfo.SEndPoint.dArray[AXIS_X] = SPathEndPoint.dArray[AXIS_X];
				writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

				//下一步写当前加工轨迹链表
				pSG73Para->cStep = 7;

				pSG73Para->usCurPos = 0;

			}

			return 1;//写入轨迹，但指令轨迹未写完
			break;

		}
		//7.正向写入轨迹链表中所有实体
		//7.反向写入轨迹链表中所有实体
		case 7:
		{
			int8 cRet;

			//将轨迹链表中的实体写入缓冲区
			uint16 usStartPos = 0;
			uint16 usEndPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
			cRet = writePathListIntoGCodeBuf(pSG73Para->pSCurrMachinePathList, usStartPos, usEndPos, &pSG73Para->usCurPos, pSGCodeParseUnit, pSG73Para->cCurrMachineDir);

			//判断当前加工轨迹链表实体是否全部写入缓冲区
			if (cRet)
			{
				//若循环切削次数 >= 0，则下一次移动到An点，反之移动到A点
				if (pSG73Para->cRestCount > 0)
				{
					//释放当前加工轨迹链表
					freeList(&pSG73Para->pSCurrMachinePathList);

					//下一步计算加工轨迹链表
					pSG73Para->cStep = 2;

					//修正dRestDeltaI与dRestDeltaK
					pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
					pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;

					pSG73Para->cRestCount--;

					pSG73Para->cCurrMachineDir *= -1;
				}
				else
				{
					pSG73Para->cStep = 8;
				}
			}

			return 1;
			break;

		}
		//8.G73Ⅲ型计算完成
		case 8:
		{
			//写入轨迹当前点-G73起刀点（快速运动）
			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint = pSG73Para->SPointA;
			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);

			pSAssignConfig->SGetTextStateInfo = pSG73Para->SGetTextStateInfo;

			return 2;
		}
		default:
			break;
		}

	}
//	//G73Ⅳ型
//	else if (pSG73Para->ucH == 3 && pSG73Para->dDeltaI != 0)
//	{
//		switch (pSG73Para->cStep)
//		{
//		//0.获取NSNF原始轨迹链表
//		case 0:
//		{
//			//
//			int8 cRet;
//			SProgram* pSCurrProgram = NULL;   //当前程序结构体
//			//1.获取当前程序结构体
//			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
//			if (pSCurrProgram == NULL)
//			{
//				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
//				return -1;
//			}
//			//2.获取当前加工轨迹链表（这里获取的轨迹偏移值）
//			double dDeltaX = pSG73Para->dDeltaU / 2;
//			double dDeltaZ = pSG73Para->dDeltaW;
//			cRet = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG73Para->unNS, pSG73Para->unNF, dDeltaX, dDeltaZ, &pSG73Para->pSPathList, &pSG73Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, G71);
//			if (cRet == 1)
//			{
//				pSG73Para->cStep = 1;
//
//#if G73_DEBUG
//				printf("-----------------------------原始的轨迹----------------------------\n");
//				printList(pSG73Para->pSPathList, printDataG71G72I);
//				printf("\n");
//#endif
//			}
//			else if (cRet == -1)
//			{
//				return -1;
//			}
//
//
//			return 0;//未写入轨迹
//			break;
//		}
//		//1.赋值预处理
//		case 1:
//		{
//			//1.0获取A点坐标
//			pSG73Para->SPointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//
//			//1.4获取轨迹加工方向
//			//getPathMachineDir(pSG73Para->pSPathList, &pSG73Para->cMachineXDir, &pSG73Para->cMachineZDir);
//			pSG73Para->cCurrMachineDir = 1;
//
//			pSG73Para->cRestCount = pSG73Para->ucD;
//
//			//去除原始轨迹中的进刀段，退刀段（不修正导致等距刀路效果不佳）
//			removeEntryAndExitPath(pSG73Para->pSPathList, AXIS_X, TRUE, TRUE);
//
//#if G73_DEBUG
//			printf("-----------------------------去除进刀段的轨迹----------------------------\n");
//			printList(pSG73Para->pSPathList, printDataG71G72I);
//			printf("\n");
//#endif
//
//			//1.6
//			pSG73Para->cStep = 2;
//
//			return 0;//未写入轨迹
//
//			break;
//
//		}
//		//2.对原始加工轨迹等距
//		case 2:
//		{
//			double dDist = pSG73Para->cRestCount * pSG73Para->dDeltaI;
//			equidistantPath(pSG73Para->pSPathList, &pSG73Para->pSCurrMachinePathList, dDist, -1);
//
//#if G73_DEBUG
//			printf("-----------------------------等距后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//			pSG73Para->cStep = 3;
//
//			return 0;
//			break;
//		}
//		//3.在当前加工轨迹中补充圆弧，并分割当前加工轨迹的圆弧
//		case 3:
//		{
//			double dDist = pSG73Para->cRestCount * pSG73Para->dDeltaI;
//			addArcInequidistanLine(pSG73Para->pSPathList, pSG73Para->pSCurrMachinePathList, dDist);
//#if G73_DEBUG
//			printf("-----------------------------添加圆弧后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif			
//			//分割圆弧
//			divideArcInPath(pSG73Para->pSCurrMachinePathList);
//#if G73_DEBUG
//			printf("-----------------------------分割圆弧后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//			//修正等距轨迹
//			//原始轨迹起始段
//			SLine* pOriEntity1 = FindNodeData(pSG73Para->pSPathList, pSG73Para->pSPathList->cSize - 1);
//			//原始轨迹终止段
//			SLine* pOriEntity2 = FindNodeData(pSG73Para->pSPathList, 0);
//			//加工轨迹起始段
//			SLine* pMachineEntity1 = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//			//加工轨迹终止段
//			SLine* pMachineEntity2 = FindNodeData(pSG73Para->pSCurrMachinePathList, 0);
//
//			////在轨迹头加一段直线
//			//SLine* pSAddLine1;
//			//pSAddLine1 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//			//pSAddLine1->EntityType = L1NE;
//			//pSAddLine1->SEndPoint = pSAddLine1->SStartPoint = pMachineEntity1->SStartPoint;
//			//pSAddLine1->SStartPoint.dArray[AXIS_X] = 1000;
//			//insertList(pSG73Para->pSCurrMachinePathList, -1, pSAddLine1, ORAM_NUM354);
//
//			//在轨迹头加两段直线
//			//（Z轴方向）
//			SLine* pSAddLine1;
//			pSAddLine1 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//			pSAddLine1->EntityType = L1NE;
//			pSAddLine1->SEndPoint = pSAddLine1->SStartPoint = pMachineEntity1->SStartPoint;
//			pSAddLine1->SStartPoint.dArray[AXIS_Z] = pOriEntity1->SStartPoint.dArray[AXIS_Z];
//			insertList(pSG73Para->pSCurrMachinePathList, -1, pSAddLine1, ORAM_NUM354);
//
//			//（X轴方向）
//			SLine* pSAddLine2;
//			pSAddLine2 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//			pSAddLine2->EntityType = L1NE;
//			pSAddLine2->SEndPoint = pSAddLine2->SStartPoint = pSAddLine1->SStartPoint;
//			pSAddLine2->SStartPoint.dArray[AXIS_X] = 5000;
//			insertList(pSG73Para->pSCurrMachinePathList, -1, pSAddLine2, ORAM_NUM354);
//
//			//在加工轨迹尾加两段直线
//			//（Z轴方向）
//			SLine* pSAddLine3;
//			pSAddLine3 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//			pSAddLine3->EntityType = L1NE;
//			pSAddLine3->SStartPoint = pSAddLine3->SEndPoint = pMachineEntity2->SEndPoint;
//			pSAddLine3->SEndPoint.dArray[AXIS_Z] = pOriEntity2->SEndPoint.dArray[AXIS_Z];
//			insertList(pSG73Para->pSCurrMachinePathList, 0, pSAddLine3, ORAM_NUM354);
//
//			//（X轴方向）
//			SLine* pSAddLine4; 
//			pSAddLine4 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//			pSAddLine4->EntityType = L1NE;
//			pSAddLine4->SStartPoint = pSAddLine4->SEndPoint = pSAddLine3->SEndPoint;
//			pSAddLine4->SEndPoint.dArray[AXIS_X] = 5000;
//			insertList(pSG73Para->pSCurrMachinePathList, 0, pSAddLine4, ORAM_NUM354);
//
//#if G73_DEBUG
//			printf("-----------------------------修正后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//
//			pSG73Para->cStep = 4;
//
//			return 0;
//			break;
//		}
//		//4.删除环
//		case 4:
//		{
//			//删除环
//			deleteRings(pSG73Para->pSCurrMachinePathList);
//			//删除加工轨迹中的辅助线
//			removeEntryAndExitPath(pSG73Para->pSCurrMachinePathList, AXIS_X, TRUE, TRUE);
//#if G73_DEBUG
//			printf("-----------------------------删除环后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//			pSG73Para->cStep = 5;
//			return 0;
//			break;
//		}
//		//5分割当前加工轨迹
//		case 5:
//		{
//			splitPathByBlank(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
//#if G73_DEBUG
//			printf("-----------------------------分割后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//			pSG73Para->cStep = 6;
//			return 0;
//			break;
//		}
//		//6.删除与添加轨迹中的实体
//		case 6:
//		{
//			//删除毛坯外的实体
//			deleteEntityInPath(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
//#if G73_DEBUG
//			printf("-----------------------------删除后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//			//补充轨迹中的实体
//			addEntityInPath(pSG73Para->pSCurrMachinePathList);
//#if G73_DEBUG
//			printf("-----------------------------添加后的轨迹----------------------------\n");
//			printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//			printf("\n");
//#endif
//			//加工轨迹无实体
//			if (pSG73Para->pSCurrMachinePathList->cSize > 0)
//			{
//				pSG73Para->cStep = 7;
//			}
//			else
//			{
//				//释放当前加工轨迹链表
//				freeList(&pSG73Para->pSCurrMachinePathList);
//
//				//下一步计算加工轨迹链表
//				pSG73Para->cStep = 2;
//
//				//修正dRestDeltaI与dRestDeltaK
//				pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
//				pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;
//
//				pSG73Para->cRestCount--;
//
//			}
//			return 0;
//			break;
//		}
//		
//		//当前加工轨迹链表已处理完成，以下开始写入轨迹
//		
//		//写入当前点X-毛坯X（快速运动）
//		case 7:
//		{
//			//计算An点坐标
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//			SPointDouble SPathStartPoint = pSLine->SStartPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = OTHERTYPE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG73Para->SPointA.dArray[AXIS_X];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 8;
//
//			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//
//			return 1;
//			break;
//		}
//		//7.写入当前点-轨迹链表起点(Z轴)（快速运动）
//		case 8:
//		{
//			//计算An点坐标
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//			SPointDouble SPathStartPoint = pSLine->SStartPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = OTHERTYPE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathStartPoint.dArray[AXIS_Z];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 9;
//
//			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//
//			return 1;//写入轨迹，但指令轨迹未写完
//
//			break;
//
//		}
//		//8.写入当前点-轨迹链表起点(X轴)（快速运动）
//		case 9:
//		{
//			//计算An点坐标
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//			SPointDouble SPathStartPoint = pSLine->SStartPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = OTHERTYPE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_X] = SPathStartPoint.dArray[AXIS_X];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 10;
//
//			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//
//			return 1;//写入轨迹，但指令轨迹未写完
//
//			break;
//
//		}
//		//9.写入轨迹链表中所有实体
//		case 10:
//		{
//			int8 cRet;
//
//			//将轨迹链表中的实体写入缓冲区
//			uint16 usStartPos = 0;
//			uint16 usEndPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//			cRet = writePathListIntoGCodeBuf(pSG73Para->pSCurrMachinePathList, usStartPos, usEndPos, &pSG73Para->usCurPos, pSGCodeParseUnit, pSG73Para->cCurrMachineDir);
//
//			//判断当前加工轨迹链表实体是否全部写入缓冲区
//			if (cRet)
//			{
//				//若循环切削次数 >= 0，则下一次移动到An点，反之移动到A点
//				if (pSG73Para->cRestCount > 0)
//				{
//					//释放当前加工轨迹链表
//					freeList(&pSG73Para->pSCurrMachinePathList);
//
//					//下一步对原始轨迹进行等距
//					pSG73Para->cStep = 2;
//
//					pSG73Para->cRestCount--;
//				}
//				else
//				{
//					pSG73Para->cStep = 11;
//				}
//			}
//
//			return 1;
//			break;
//
//		}
//		//X轴方向移动到起刀点（快速运动）
//		case 11:
//		{
//			//写入轨迹当前点-G73起刀点
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = OTHERTYPE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_X] = pSG73Para->SPointA.dArray[AXIS_X];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//
//			pSG73Para->cStep = 12;
//
//			return 1;
//		}
//		//Z轴方向移动到起刀点（快速运动）
//		case 12:
//		{
//			//写入轨迹当前点-G73起刀点
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = OTHERTYPE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG73Para->SPointA.dArray[AXIS_Z];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			pSAssignConfig->SGetTextStateInfo = pSG73Para->SGetTextStateInfo;
//
//			return 2;
//		}
//
//
//		default:
//			break;
//		}
//
//	}
//	//G73Ⅴ型
//	else if (pSG73Para->ucH == 4 && pSG73Para->dDeltaI != 0)
//	{
//	switch (pSG73Para->cStep)
//	{
//	//0.获取NSNF原始轨迹链表
//	case 0:
//	{
//		//
//		int8 cRet;
//		SProgram* pSCurrProgram = NULL;   //当前程序结构体
//		//1.获取当前程序结构体
//		pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
//		if (pSCurrProgram == NULL)
//		{
//			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
//			return -1;
//		}
//		//2.获取当前加工轨迹链表（这里获取的轨迹偏移值）
//		double dDeltaX = pSG73Para->dDeltaU / 2;
//		double dDeltaZ = pSG73Para->dDeltaW;
//		cRet = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG73Para->unNS, pSG73Para->unNF, dDeltaX, dDeltaZ, &pSG73Para->pSPathList, &pSG73Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, G71);
//		if (cRet == 1)
//		{
//			pSG73Para->cStep = 1;
//
//#if G73_DEBUG
//			printf("-----------------------------原始的轨迹----------------------------\n");
//			printList(pSG73Para->pSPathList, printDataG71G72I);
//			printf("\n");
//#endif
//		}
//		else if (cRet == -1)
//		{
//			return -1;
//		}
//
//
//		return 0;//未写入轨迹
//		break;
//	}
//	//1.赋值预处理
//	case 1:
//	{
//		//1.0获取A点坐标
//		pSG73Para->SPointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//
//		//1.1获取轨迹加工方向
//		pSG73Para->cCurrMachineDir = 1;
//
//		pSG73Para->cRestCount = pSG73Para->ucD;
//
//		//去除原始轨迹中的进刀段，退刀段（不修正导致等距刀路效果不佳）
//		removeEntryAndExitPath(pSG73Para->pSPathList, AXIS_X, TRUE, TRUE);
//
//#if G73_DEBUG
//		printf("-----------------------------去除进刀段的轨迹----------------------------\n");
//		printList(pSG73Para->pSPathList, printDataG71G72I);
//		printf("\n");
//#endif
//
//		//1.6
//		pSG73Para->cStep = 2;
//
//		return 0;//未写入轨迹
//
//		break;
//
//	}
//	//2.对原始加工轨迹等距
//	case 2:
//	{
//		double dDist = pSG73Para->cRestCount * pSG73Para->dDeltaI;
//		equidistantPath(pSG73Para->pSPathList, &pSG73Para->pSCurrMachinePathList, dDist, -1);
//
//#if G73_DEBUG
//		printf("-----------------------------等距后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//		pSG73Para->cStep = 3;
//
//		return 0;
//		break;
//	}
//	//3.在当前加工轨迹中补充圆弧，并分割当前加工轨迹的圆弧
//	case 3:
//	{
//		double dDist = pSG73Para->cRestCount * pSG73Para->dDeltaI;
//		addArcInequidistanLine(pSG73Para->pSPathList, pSG73Para->pSCurrMachinePathList, dDist);
//#if G73_DEBUG
//		printf("-----------------------------添加圆弧后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif			
//		//分割圆弧
//		divideArcInPath(pSG73Para->pSCurrMachinePathList);
//#if G73_DEBUG
//		printf("-----------------------------分割圆弧后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//		//修正等距轨迹
//		//原始轨迹起始段
//		SLine* pOriEntity1 = FindNodeData(pSG73Para->pSPathList, pSG73Para->pSPathList->cSize - 1);
//		//原始轨迹终止段
//		SLine* pOriEntity2 = FindNodeData(pSG73Para->pSPathList, 0);
//		//加工轨迹起始段
//		SLine* pMachineEntity1 = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//		//加工轨迹终止段
//		SLine* pMachineEntity2 = FindNodeData(pSG73Para->pSCurrMachinePathList, 0);
//
//		//在轨迹头加一段直线
//		SLine* pSAddLine1;
//		pSAddLine1 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//		pSAddLine1->EntityType = L1NE;
//		pSAddLine1->SEndPoint = pSAddLine1->SStartPoint = pMachineEntity1->SStartPoint;
//		pSAddLine1->SStartPoint.dArray[AXIS_X] = 5000;
//		insertList(pSG73Para->pSCurrMachinePathList, -1, pSAddLine1, ORAM_NUM354);
//
//		//在加工轨迹尾加两段直线
//		SLine* pSAddLine2;
//		pSAddLine2 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//		pSAddLine2->EntityType = L1NE;
//		pSAddLine2->SStartPoint = pSAddLine2->SEndPoint = pMachineEntity2->SEndPoint;
//		pSAddLine2->SEndPoint.dArray[AXIS_Z] = pOriEntity2->SEndPoint.dArray[AXIS_Z];
//		insertList(pSG73Para->pSCurrMachinePathList, 0, pSAddLine2, ORAM_NUM354);
//
//		SLine* pSAddLine3;
//		pSAddLine3 = mallocGCode(sizeof(SLine), ORAM_NUM355);
//		pSAddLine3->EntityType = L1NE;
//		pSAddLine3->SStartPoint = pSAddLine3->SEndPoint = pSAddLine2->SEndPoint;
//		pSAddLine3->SEndPoint.dArray[AXIS_X] = 5000;
//		insertList(pSG73Para->pSCurrMachinePathList, 0, pSAddLine3, ORAM_NUM354);
//
//#if G73_DEBUG
//		printf("-----------------------------修正后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//
//		pSG73Para->cStep = 4;
//
//		return 0;
//		break;
//	}
//	//4.删除环
//	case 4:
//	{
//		//删除环
//		deleteRings(pSG73Para->pSCurrMachinePathList);
//		//删除加工轨迹中的辅助线
//		removeEntryAndExitPath(pSG73Para->pSCurrMachinePathList, AXIS_X, TRUE, TRUE);
//
//#if G73_DEBUG
//		printf("-----------------------------删除环后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//		pSG73Para->cStep = 5;
//		return 0;
//		break;
//	}
//	//5分割当前加工轨迹
//	case 5:
//	{
//		splitPathByBlank(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
//#if G73_DEBUG
//		printf("-----------------------------分割后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//		pSG73Para->cStep = 6;
//		return 0;
//		break;
//	}
//	//6.删除与添加轨迹中的实体
//	case 6:
//	{
//		//删除毛坯外的实体
//		deleteEntityInPath(pSG73Para->pSCurrMachinePathList, pSG73Para->SPointA.dArray[AXIS_X], pSG73Para->SPointA.dArray[AXIS_Z]);
//#if G73_DEBUG
//		printf("-----------------------------删除后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//		//补充轨迹中的实体
//		addEntityInPath(pSG73Para->pSCurrMachinePathList);
//#if G73_DEBUG
//		printf("-----------------------------添加后的轨迹----------------------------\n");
//		printList(pSG73Para->pSCurrMachinePathList, printDataG71G72I);
//		printf("\n");
//#endif
//		//加工轨迹无实体
//		if (pSG73Para->pSCurrMachinePathList->cSize > 0)
//		{
//			pSG73Para->cStep = 8;
//		}
//		else
//		{
//			//释放当前加工轨迹链表
//			freeList(&pSG73Para->pSCurrMachinePathList);
//
//			//下一步计算加工轨迹链表
//			pSG73Para->cStep = 2;
//
//			//修正dRestDeltaI与dRestDeltaK
//			pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
//			pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;
//
//			pSG73Para->cRestCount--;
//
//		}
//		return 0;
//		break;
//	}
//
//	//当前加工轨迹链表已处理完成，以下开始写入轨迹
//
//	//7.写入当前点X-毛坯X
//	case 7:
//	{
//		//计算An点坐标
//		SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//		SPointDouble SPathStartPoint = pSLine->SStartPoint;
//
//		SLine SLineInfo;
//		memset(&SLineInfo, 0, sizeof(SLine));
//		SLineInfo.EntityType = L1NE;
//		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG73Para->SPointA.dArray[AXIS_X];
//		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//		//下一步写当前加工轨迹链表
//		pSG73Para->cStep = 8;
//
//		pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//
//		return 1;
//		break;
//	}
//	//8.写入当前点-轨迹链表起点/终点(Z轴)
//	case 8:
//	{
//		//当前为正向，写入当前点-轨迹链表起点(Z轴)
//		if (pSG73Para->cCurrMachineDir == 1)
//		{
//			//计算当前轨迹起点坐标
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//			SPointDouble SPathStartPoint = pSLine->SStartPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = L1NE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathStartPoint.dArray[AXIS_Z];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 9;
//
//			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//		}
//		//当前为反向，写入当前点-轨迹链表终点(Z轴)
//		else
//		{
//			//计算当前轨迹终点坐标
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, 0);
//			SPointDouble SPathEndPoint = pSLine->SEndPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = L1NE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathEndPoint.dArray[AXIS_Z];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 9;
//
//			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//
//		}
//		return 1;//写入轨迹，但指令轨迹未写完
//
//		break;
//
//	}
//	//9.写入当前点-轨迹链表起点/终点(X轴)
//	case 9:
//	{
//		//当前为正向，写入当前点-轨迹链表起点(X轴)
//		if (pSG73Para->cCurrMachineDir == 1)
//		{
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, pSG73Para->pSCurrMachinePathList->cSize - 1);
//			SPointDouble SPathStartPoint = pSLine->SStartPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = L1NE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_X] = SPathStartPoint.dArray[AXIS_X];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 10;
//
//			pSG73Para->usCurPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//		}
//		//当前为反向，写入当前点-轨迹链表终点(Z轴)
//		else
//		{
//			SLine* pSLine = FindNodeData(pSG73Para->pSCurrMachinePathList, 0);
//			SPointDouble SPathEndPoint = pSLine->SEndPoint;
//
//			SLine SLineInfo;
//			memset(&SLineInfo, 0, sizeof(SLine));
//			SLineInfo.EntityType = L1NE;
//			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//			SLineInfo.SEndPoint.dArray[AXIS_X] = SPathEndPoint.dArray[AXIS_X];
//			writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//			//下一步写当前加工轨迹链表
//			pSG73Para->cStep = 10;
//
//			pSG73Para->usCurPos = 0;
//
//		}
//
//		return 1;//写入轨迹，但指令轨迹未写完
//		break;
//	}
//	//10.写入轨迹链表中所有实体正向/反向
//	case 10:
//	{
//		int8 cRet;
//
//		//将轨迹链表中的实体写入缓冲区
//		uint16 usStartPos = 0;
//		uint16 usEndPos = pSG73Para->pSCurrMachinePathList->cSize - 1;
//		cRet = writePathListIntoGCodeBuf(pSG73Para->pSCurrMachinePathList, usStartPos, usEndPos, &pSG73Para->usCurPos, pSGCodeParseUnit, pSG73Para->cCurrMachineDir);
//
//		//判断当前加工轨迹链表实体是否全部写入缓冲区
//		if (cRet)
//		{
//			//若循环切削次数 >= 0，则下一次移动到An点，反之移动到A点
//			if (pSG73Para->cRestCount > 0)
//			{
//				//释放当前加工轨迹链表
//				freeList(&pSG73Para->pSCurrMachinePathList);
//
//				//下一步计算加工轨迹链表
//				pSG73Para->cStep = 2;
//
//				//修正dRestDeltaI与dRestDeltaK
//				pSG73Para->dRestDeltaI -= pSG73Para->dRestDeltaI / pSG73Para->cRestCount;
//				pSG73Para->dRestDeltaK -= pSG73Para->dRestDeltaK / pSG73Para->cRestCount;
//
//				pSG73Para->cRestCount--;
//
//				pSG73Para->cCurrMachineDir *= -1;
//			}
//			else
//			{
//				pSG73Para->cStep = 11;
//			}
//		}
//
//		return 1;
//		break;
//
//	}
//	//X轴方向移动到起刀点（快速运动）
//	case 11:
//	{
//		//写入轨迹当前点-G73起刀点
//		SLine SLineInfo;
//		memset(&SLineInfo, 0, sizeof(SLine));
//		SLineInfo.EntityType = OTHERTYPE;
//		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//		SLineInfo.SEndPoint.dArray[AXIS_X] = pSG73Para->SPointA.dArray[AXIS_X];
//		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//		//
//		pSG73Para->cStep = 12;
//
//		return 1;
//	}
//	//Z轴方向移动到起刀点（快速运动）
//	case 12:
//	{
//		//写入轨迹当前点-G73起刀点
//		SLine SLineInfo;
//		memset(&SLineInfo, 0, sizeof(SLine));
//		SLineInfo.EntityType = OTHERTYPE;
//		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
//		SLineInfo.SEndPoint.dArray[AXIS_Z] = pSG73Para->SPointA.dArray[AXIS_Z];
//		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeParseUnit);
//
//		pSAssignConfig->SGetTextStateInfo = pSG73Para->SGetTextStateInfo;
//
//		return 2;
//	}	
//	default:
//		break;
//	}
//
//	}

}

