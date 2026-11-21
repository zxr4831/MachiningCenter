/*
ns～nf程序段中，只能有G功能：
Ga组指令：G00、G01、G02、G03、G04、G05、G6.2、G6.3、G7.2、G7.3
Gz组指令：G04
速度控制指令：G96、G97、G98、G99
刀补指令：G40、G41、G42
*/


#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"

#include "GCode.h"

static SG70Para* pSG70ParaCheck[CHANNEL_NUM_MAX];
static SG70Para* pSG70ParaAssign[CHANNEL_NUM_MAX];


//添加G70规则结构体
void addG70RegulationAPI()
{
	//指令：G70
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(70, sGz, 1, mallocG70ParaStruct, freeG70ParaStruct, getG70Para, CheckG70, AssignG70);
}

//参数结构体初始化函数？


/*************************************************
  函数名称：int8 mallocG70ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G70参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG70ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{

	//1.G70检查参数结构体
	if (ucMode == GC_SYNTAX)
	{
		//1.1开辟内存
		pSG70ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG70Para), ORAM_NUM418);//暂未对内存编号
		if (pSG70ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG70ParaCheck[ucChannelNum], 0, sizeof(SG70Para));

		//1.2初始化G70参数结构体
		return 1;
	}
	//2.G70赋值参数结构体
	else if (ucMode == GC_ASSIGN)
	{
		//2.1开辟内存
		pSG70ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG70Para), ORAM_NUM419);
		if (pSG70ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG70ParaAssign[ucChannelNum], 0, sizeof(SG70Para));

		//2.2初始化G70参数结构体
		pSG70ParaAssign[ucChannelNum]->cPart = ASSIGN_PART_1;


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
int8 freeG70ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG70ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG70ParaCheck[ucChannelNum]);
			pSG70ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG70ParaAssign[ucChannelNum] != NULL)
		{
			//轨迹链表非空
			if (pSG70ParaAssign[ucChannelNum]->pSPathList != NULL)
			{
				freeList(&pSG70ParaAssign[ucChannelNum]->pSPathList);
			}

			freeGCode(pSG70ParaAssign[ucChannelNum]);
			pSG70ParaAssign[ucChannelNum] = NULL;
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
int8 getG70Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
	//1.检查
	if (ucMode == GC_SYNTAX)
	{
		if (pSScanContainer->bP == TRUE)
		{
			pSG70ParaCheck[ucChannelNum]->unNS = (uint32)pSScanContainer->dP;
		}
		if (pSScanContainer->bQ == TRUE)
		{
			pSG70ParaCheck[ucChannelNum]->unNF = (uint32)pSScanContainer->dQ;
		}

		if (pSScanContainer->bH == TRUE)
		{
			pSG70ParaCheck[ucChannelNum]->ucH = (uint8)pSScanContainer->dH;
		}
		else
		{
			pSScanContainer->bA2Z['H' - 65] = TRUE; //添加H指令参数范围检查
			pSG70ParaCheck[ucChannelNum]->ucH = 0;
		}
	}
	//2.赋值
	else if (ucMode == GC_ASSIGN)
	{
		if (pSScanContainer->bP == TRUE)
		{
			pSG70ParaAssign[ucChannelNum]->unNS = (uint32)pSScanContainer->dP;
		}
		if (pSScanContainer->bQ == TRUE)
		{
			pSG70ParaAssign[ucChannelNum]->unNF = (uint32)pSScanContainer->dQ;
		}

		pSG70ParaAssign[ucChannelNum]->dDelta[AXIS_X] = pSScanContainer->dUVW[0];
		pSG70ParaAssign[ucChannelNum]->dDelta[AXIS_Z] = pSScanContainer->dUVW[2];

		if (pSScanContainer->bH == TRUE)
		{
			pSG70ParaAssign[ucChannelNum]->ucH = (uint8)pSScanContainer->dH;
		}
		else
		{
			pSScanContainer->bA2Z['H' - 65] = TRUE; //添加H指令参数范围检查
			pSG70ParaAssign[ucChannelNum]->ucH = 0;
		}

	}
}
int8 CheckG70(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	// 判断必须有P Q
	if (!(pSSyntaxConfig->SScanContainer.bP && pSSyntaxConfig->SScanContainer.bQ))
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM432);
		return -1;
	}

	// 1.指令字符检查
	if (checkParaLetter("PQH", &pSSyntaxConfig->SScanContainer) == -1) {
		return -1;
	}

	// 2.批量检查参数范围
	if (checkParaRange('P', &pSSyntaxConfig->SScanContainer, 0, 99999999) == -1 ||
		checkParaRange('Q', &pSSyntaxConfig->SScanContainer, 0, 99999999) == -1 ||
		checkParaRange('H', &pSSyntaxConfig->SScanContainer, 0, 3) == -1)
	{
		return -1;
	}
	// 3.批量检查参数是否为整数
	if (checkParaInteger('P', &pSSyntaxConfig->SScanContainer) == -1 ||
		checkParaInteger('Q', &pSSyntaxConfig->SScanContainer) == -1 ||
		checkParaInteger('H', &pSSyntaxConfig->SScanContainer) == -1) 
	{
		return -1;
	}

	return 2;
}

/*************************************************
  函数名称：AssignG70
  函数功能：分阶段生成 G70 轮廓及回程，复用现有缓冲提交和状态继承。
  输入参数：pSAssignConfig 为有效通道赋值状态；pGCodeBufUnit 为当前未转换模态的写槽，均非空。
  输出参数：G70 阶段、写槽、编程状态及提交指针，按当前阶段更新。
  返 回 值：0 等待或内部处理；1 已写段但未完成；2 完成；3 内部已处理提交；-1 错误。
  备    注：F 校验失败不推进内部切片或发布；此前已执行的模态和前作用赋值不回滚。
**************************************************/
int8 AssignG70(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SG70Para* pSG70Para = pSG70ParaAssign[pSAssignConfig->ucChannelNum];

	//反向
	if (pSG70Para->ucH == 2 || pSG70Para->ucH == 3)
	{
		switch (pSG70Para->cStep)
		{
		//1.获取轨迹链表
		case 0:
		{			
			int8 cRet;

			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//2.获取当前加工轨迹链表（这里获取的轨迹偏移值）
			double dDeltaX = pSG70Para->dDelta[0];
			double dDeltaZ = pSG70Para->dDelta[2];

			//G70反向时，修改起点坐标，防止获取NSNF时继承出错
			SGCodeSysState* pSCurrGCodeSysState = getCurrSystemState(pSAssignConfig->ucChannelNum, GC_ASSIGN);
			extern SPointDouble g_SPathStartPoint;

			pSCurrGCodeSysState->SCurrPoint.dArray[AXIS_X] = g_SPathStartPoint.dArray[AXIS_X] + pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_X];
			pSCurrGCodeSysState->SCurrPoint.dArray[AXIS_Z] = g_SPathStartPoint.dArray[AXIS_Z] + pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_Z];

			cRet = getMachiningPath(GC_ASSIGN, pSCurrProgram, pSG70Para->unNS, pSG70Para->unNF, dDeltaX, dDeltaZ, &pSG70Para->pSPathList, &pSG70Para->SGetTextStateInfo, pSAssignConfig->ucChannelNum, G71);
			
			if (cRet == -1)
			{
				return -1;
			}
			else if (cRet == 1)
			{
				pSG70Para->cStep = 1;
			}
			return 0;
		}
		//2.镜像轨迹链表
		case 1:
		{
			double dDeltaX = pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_X];
			double dDeltaZ = pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset.dArray[AXIS_Z];
			//轨迹链表转为绝对坐标
			offsetPathList(&pSG70Para->pSPathList, -dDeltaX, -dDeltaZ);

			//镜像
			if (pSG70Para->ucH == 3)
			{
				mirrorPathList(&pSG70Para->pSPathList);
			}
			pSG70Para->cStep = 2;

			//轨迹链表转为机械坐标
			offsetPathList(&pSG70Para->pSPathList, dDeltaX, dDeltaZ);

		}
		
		//3.定位到轨迹链表终点
		case 2:
		{
			SLine* pSLine = FindNodeData(pSG70Para->pSPathList, 0);
			SPointDouble SPathEndPoint = pSLine->SEndPoint;

			SLine SLineInfo;
			memset(&SLineInfo, 0, sizeof(SLine));
			SLineInfo.EntityType = OTHERTYPE;
			SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			SLineInfo.SEndPoint.dArray[AXIS_X] = SPathEndPoint.dArray[AXIS_X];
			SLineInfo.SEndPoint.dArray[AXIS_Z] = SPathEndPoint.dArray[AXIS_Z];

			writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);

			pSG70Para->usCurPos = 0;
			pSG70Para->cStep = 3;
			return 1;
		}

		//4.输出轨迹链表
		case 3:
		{
			//反向输出
			uint16 usStartPos = 0;
			uint16 usEndPos = pSG70Para->pSPathList->cSize - 2;
			if (writePathListIntoGCodeBuf(pSG70Para->pSPathList, usStartPos, usEndPos, &pSG70Para->usCurPos, pGCodeBufUnit, -1))
			{
				return 2;
			}
			return 1;
		}
		}

	}
	switch (pSG70Para->cStep)
	{
	//0.G70内部NS至NF程序段赋值开始，保存主赋值流程的数据
	case 0:
	{
		//保存本次循环起点；后续轮廓解析会不断更新系统当前位置。
    	pSG70Para->SCycleStartPoint =pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		
		//1.G70内部NS至NF程序段赋值完成后，需要继续G70后的程序段解析
		pSG70Para->SGetTextStateInfo = pSAssignConfig->SGetTextStateInfo;
		initGetTextStateInfo(&pSAssignConfig->SGetTextStateInfo);
		//pSAssignConfig->SGetTextStateInfo.unGetLineNum =

		//2.G70内部NS至NF程序段赋值完成后，需要赋值G70程序段中的后作用M
		pSG70Para->SScanContainer = pSAssignConfig->SScanContainer;
		initScanContainer(&pSAssignConfig->SScanContainer, TRUE);

		pSG70ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 1;

		break;
	}

	//1.找到目标程序段号NS
	case 1:
	{
		int8 cRet;
		SProgram* pSCurrProgram = NULL;   //当前程序结构体
		pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);  //获取当前程序结构体
		if (pSCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		cRet = findTargetSegment(GC_ASSIGN, pSAssignConfig->ucChannelNum, pSCurrProgram, pSG70ParaAssign[pSAssignConfig->ucChannelNum]->unNS, &pSAssignConfig->SGetTextStateInfo);
		if (cRet == 1)
		{
			pSG70ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 2;
		}
		else if (cRet == -1)
		{
			return -1;//没有找到
		}

		break;
	}

	//2.赋值G70内部NS至NF程序段
	case 2:
	{
		switch (pSG70Para->cPart)
		{
		//1.处理文本
		case ASSIGN_PART_1:
		{
			int8 cRet;
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			cRet = processProgramText(GC_ASSIGN, &pSAssignConfig->SGetTextStateInfo, pSAssignConfig->ucChannelNum, pSCurrProgram);
			if (cRet == 1)
			{
				pSG70Para->cCommandSegmentCnt++;
				pSG70Para->cPart = ASSIGN_PART_2;
			}
			break;
		}

		//初始化扫描结构体
		case ASSIGN_PART_2:
		{
		//1.初始化G70的扫描结构体
			//1.1当前扫描程序段为指令的第一段，需要清空扫描结构体
			if (pSAssignConfig->cCommandSegmentCnt == 1)	
			{
				initScanContainer(&pSAssignConfig->SScanContainer, TRUE);
			}
			//1.2当前扫描程序段非指令的第一段，不需清空MFST值，防止多程序段丢失MFST的值
			else
			{
				initScanContainer(&pSAssignConfig->SScanContainer, FALSE);
			}

		//2.pCurrPtr需要重新指向程序段字符串首字符
			pSAssignConfig->SGetTextStateInfo.pCurrPtr = pSAssignConfig->SGetTextStateInfo.cStr;

		//3.进入下一切片
			pSG70Para->cPart = ASSIGN_PART_3;

			break;
		}

		//扫描
		case ASSIGN_PART_3:
		{
			int8 cRet;

			int16 sLastCommandVal;
			int16 sCurrCommandVal;
			const SGCommandRegulation* pSLastCommandRegulation;
			const SGCommandRegulation* pSCurrCommandRegulation;

			SProgram* pCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0);		//当前程序结构体
			if (pCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}

			//扫描程序段
			cRet = scanSegment(GC_ASSIGN, &pSAssignConfig->SGetTextStateInfo, &pSAssignConfig->SScanContainer, pCurrProgram, pSAssignConfig->ucChannelNum);

			//当前程序段全部扫描完成
			if (cRet == TRUE)
			{
				//G70坐标偏移
				for (int i = 0; i < 3; i++)
				{
					if (pSAssignConfig->SScanContainer.bXYZ[i] == TRUE)
					{
						pSAssignConfig->SScanContainer.dXYZ[i] += pSG70Para->dDelta[i];
					}
					if (pSAssignConfig->SScanContainer.bUVW[i] == TRUE)
					{
						pSAssignConfig->SScanContainer.dUVW[i] += pSG70Para->dDelta[i];
					}
				}


				//G70镜像
				if (pSG70Para->ucH == 1 || pSG70Para->ucH == 3)
				{
					pSAssignConfig->SScanContainer.dXYZ[0] *= -1;

					SGCommandRegulation* pGaRegulation =
						pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation;

					// 当前段没有显式 Ga 指令时，从系统模态状态继承
					if (pGaRegulation == NULL)
					{
						pGaRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
					}

					if (pGaRegulation != NULL)
					{
						if (pGaRegulation->nCommandVal == 20) // G02 → 镜像为 G03
						{
							pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation = (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, 30);
						}
						else if (pGaRegulation->nCommandVal == 30) // G03 → 镜像为 G02
						{
							pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation = (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, 20);
						}
					}
					//if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal == 20)//圆弧方向
					//{
					//	pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation = (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, 30);
					//}
					//else if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal == 30)
					//{
					//	pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation = (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, 20);
					//}
				}

				//1.获取上一程序段运动相关指令（Gz组优先级高于Ga组）
				if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGz != 0)
				{
					sLastCommandVal = pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGz;
					pSLastCommandRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGzRegulation;
				}
				else
				{
					sLastCommandVal = pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa;
					pSLastCommandRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
				}

				//2.获取当前程序段运动相关指令
				//2.1当前程序段出现Ga组指令
				if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
				{
					sCurrCommandVal = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
					pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
					pSG70Para->pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
				}
				//2.2当前程序段出现Gz组指令（G70内部可能出现Gz组指令，如：G04）
				else if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
				{
					sCurrCommandVal = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
					pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
					pSG70Para->pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
				}
				//2.3当前程序段未出现GaGz组指令,则取当前系统状态Ga组指令
				else
				{
					sCurrCommandVal = pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa;
					pSCurrCommandRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
					pSG70Para->pSCurrCommandRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
				}

				//3. G70ns nf段只有普通的G指令
				if (pSCurrCommandRegulation->cSegmentNum > 1)
				{
					parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
					break;
				}
				//3.判断多程序段指令的每一程序段运动相关指令是否一致          
				//if (pSG70Para->cCommandSegmentCnt == 1)
				//{
				//	pSG70Para->sCurrCommandVal = sCurrCommandVal;
				//}
				//else
				//{
				//	if (pSG70Para->sCurrCommandVal != sCurrCommandVal)
				//	{
				//		//多程序段指令的第一段程序段的指令与后续程序段指令不一致，如：第一段为G71，第二段非G71
				//		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
				//		break;
				//	}
				//}

				//4.判断是否需要释放开辟指令参数结构体内存（仅运动相关指令第一程序段需要判断）
				if (pSG70Para->cCommandSegmentCnt == 1 && (sLastCommandVal != sCurrCommandVal))
				{
					//4.1释放上一指令字的参数结构体
					if (pSLastCommandRegulation->pFreeParaStructAPI != NULL)
					{
						pSLastCommandRegulation->pFreeParaStructAPI(GC_ASSIGN, pSAssignConfig->ucChannelNum);
					}
					//4.2开辟当前指令字的参数结构体
					if (pSCurrCommandRegulation->pMallocParaStructAPI != NULL)
					{
						if (pSCurrCommandRegulation->pMallocParaStructAPI(GC_ASSIGN, pSAssignConfig->ucChannelNum) <= 0)
						{
							parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
							break;
						}
					}
				}

				//5.判断当前指令字是否需要写参数结构体
				if (pSCurrCommandRegulation->pGetParaAPI != NULL)
				{
					pSCurrCommandRegulation->pGetParaAPI(GC_ASSIGN, &pSAssignConfig->SScanContainer, pSAssignConfig->ucChannelNum, &pSAssignConfig->SGCodeCurrSysState);
				}

				//6.判断cGa,cGz指令所有程序段是否扫描完成
				if (pSG70Para->cCommandSegmentCnt == pSCurrCommandRegulation->cSegmentNum)
				{
					//6.1运动相关指令所有程序段已全部扫描
					pSG70Para->cPart = ASSIGN_PART_4;
				}

			}

			break;
		}

		//初始化缓冲区单元，并完成对当前系统状态的继承，赋值前作用指令
		case ASSIGN_PART_4:
		{

			//NS程序段对应的缓冲区单元已写入内容，这里不能进行初始化
			//如：G55 G70 *** ；若在此初始化会导致G55数据丢失
			if(!(pSAssignConfig->SScanContainer.bN == TRUE && pSAssignConfig->SScanContainer.dN == pSG70Para->unNS))
			//if (!(pSG70Para->SScanContainer.bN == TRUE && pSG70Para->SScanContainer.dN == pSG70Para->unNS))
			{
				//1.缓冲区初始化
				initGCodeBufUnit(pGCodeBufUnit);

				//2.缓冲区继承：从当前解析通道状态中继承（如：刀具号，刀补号等）
				inheritFromGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pGCodeBufUnit);
			}

			//3.赋值cGa、cGz以外的G指令
			assignGbCommand(pSAssignConfig, pGCodeBufUnit);
			assignGcCommand(pSAssignConfig, pGCodeBufUnit);
			assignGdCommand(pSAssignConfig, pGCodeBufUnit);
			assignGeCommand(pSAssignConfig, pGCodeBufUnit);
			assignGfCommand(pSAssignConfig, pGCodeBufUnit);
			assignGgCommand(pSAssignConfig, pGCodeBufUnit);
			assignGjCommand(pSAssignConfig, pGCodeBufUnit);
			assignGhCommand(pSAssignConfig, pGCodeBufUnit);
			assignGxCommand(pSAssignConfig, pGCodeBufUnit);

			//4.赋值前作用M
			for (int i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
			{
				if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i] != NULL)
				{
					if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->cType == IO_PRE_ACTION)
					{
						if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI != NULL)
						{
							pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI(pSAssignConfig, pGCodeBufUnit);
						}
					}
				}
			}

			//5.赋值F,S,T
			if (assignFCommand(pSAssignConfig, pGCodeBufUnit) < 0)
			{
				return -1;
			}
			assignSCommand(pSAssignConfig, pGCodeBufUnit);//注：变量cSpindleSign不同S的赋值不同，cSpindleSign在前作用M中赋值，所以此函数必须写在赋值前作用之后	lxw20241030
			assignTCommand(pSAssignConfig, pGCodeBufUnit);

			pSG70Para->cPart = ASSIGN_PART_5;

			break;
		}

		//赋值Ga组指令
		case ASSIGN_PART_5:
		{
			//赋值接口
			int8 cRet = 0;
			//2.调用对应指令的赋值接口
			if (pSG70Para->pSCurrCommandRegulation->pAssignCommandAPI != NULL)
			{
				//注意！！！执行一次赋值接口只能写一个缓冲区
				//指令接口内写完一段轨迹后必须返回

				//*********若需调用指令赋值，则必须用到赋值结构体
				//*********G70内部NS至NF程序段赋值借用主流程赋值结构体
				//*********先保存赋值结构体中的扫描结构体等数据，G70赋值完成后再恢复

				//坐标偏置处理
				//for (int i = 0; i < 3; i++)
				//{
				//	if (pSAssignConfig->SScanContainer.bXYZ[i] == TRUE)
				//	{
				//		pSAssignConfig->SScanContainer.dXYZ[i] += pSG70Para->dDelta[i];
				//	}
				//	if (pSAssignConfig->SScanContainer.bUVW[i] == TRUE)
				//	{
				//		pSAssignConfig->SScanContainer.dUVW[i] += pSG70Para->dDelta[i];
				//	}
				//}

				cRet = pSG70Para->pSCurrCommandRegulation->pAssignCommandAPI(pSAssignConfig, pGCodeBufUnit);

			}

			if (cRet > 0)
			{
				//写入文件名
				SProgram* pSCurrProgram = getCurrProgramStruct(pSAssignConfig->ucChannelNum);
				strcpy(pGCodeBufUnit->cFilename, pSCurrProgram->cFileName);

				//写缓冲区中G代码行号信息
				pGCodeBufUnit->unGCodeLineNum = pSAssignConfig->SGetTextStateInfo.unCurrLineNum;
				//2.1写缓冲区中Ga组模态信息
				if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
				{
					pGCodeBufUnit->SModalInfo.sGa = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
					pGCodeBufUnit->SModalInfo.pSGaRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
				}
				//2.2写缓冲区中Gz组模态信息
				if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
				{
					pGCodeBufUnit->SModalInfo.sGz = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
					pGCodeBufUnit->SModalInfo.pSGzRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
				}

				//Ga/Gz组指令已赋值，但未赋值完
				if (cRet == 1)
				{
					//更新G代码系统状态
					updataGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pGCodeBufUnit);
					//移动指针
					moveGCodeWritePtr(pSAssignConfig->ucChannelNum);

					//指针移动，需要再次获取当前G代码缓冲区写单元
					int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
					struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];
					//初始化G代码缓冲区单元
					initGCodeBufUnit(pSGCodeParseUnit);
					//从G代码系统状态中继承
					inheritFromGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pSGCodeParseUnit);
				}
				//cGa/cGz组指令赋值完成
				else if (cRet == 2)
				{
					pSG70Para->cPart = ASSIGN_PART_6;
				}
			}


			break;
		}

		//赋值后作用指令
		case ASSIGN_PART_6:
		{
			//赋值后作用M
			for (int i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
			{
				if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i] != NULL)
				{
					if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->cType == IO_POST_ACTION)
					{
						if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI != NULL)
						{
							pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI(pSAssignConfig, pGCodeBufUnit);
						}
					}
				}
			}

			pSG70Para->cPart = ASSIGN_PART_7;
			break;
		}

		//赋值IF/WHILE指令
		case ASSIGN_PART_7:
		{
		

		}

		//判断G70是否结束
		case ASSIGN_PART_8:
		{
			//当前程序段号为NF：先提交最后一段轮廓，再进入回程阶段
			if (pSAssignConfig->SScanContainer.bN == TRUE && pSAssignConfig->SScanContainer.dN == pSG70Para->unNF)
			{
				if (isErrorHasBeenWritten())
				{
					return -1;
				}

				//沿用写入层的两槽预留约定；不足时保持当前阶段等待。
				if (getSGCodeParsePointerBufRes(GCODE_WRITE, pSAssignConfig->ucChannelNum) <= 1)
				{
					return 0;
				}

				//必须在模态转换和发布前，用原始记录更新编程状态。
				updataGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pGCodeBufUnit);

				moveGCodeWritePtr(pSAssignConfig->ucChannelNum);

				if (isErrorHasBeenWritten())
				{
					return -1;
				}

				//Q段已经提交。本次调用不再写第二段，下一次调用生成回程。
				pSG70Para->cStep = 3;
				return 0;
			}
			//2.当前程序段号不为NF
			else
			{
				//非NF程序段在G70内部提交，然后继续解析下一段。
				//保存到当前G代码系统状态结构体
				updataGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pGCodeBufUnit);

				moveGCodeWritePtr(pSAssignConfig->ucChannelNum);


				pSG70Para->cCommandSegmentCnt = 0;
				pSG70Para->cPart = ASSIGN_PART_1;
				break;
			}

		}
		default:
			break;
		}

		break;
	}

	//3.Q段已提交：生成返回循环起点的运动，并恢复主赋值流程
	case 3:
	{
		SLine SReturnLine = {0};
		SProgram* pSCurrProgram;

		if (isErrorHasBeenWritten())
		{
			return -1;
		}

		//不足时等待，不恢复扫描信息、不提前结束G70。
		if (getSGCodeParsePointerBufRes(GCODE_WRITE, pSAssignConfig->ucChannelNum) <= 1)
		{
			return 0;
		}

		pSCurrProgram = getCurrProgramStruct(pSAssignConfig->ucChannelNum);
		if (pSCurrProgram == NULL ||pSG70Para->SScanContainer.SScanModalInfo.pSGzRegulation == NULL)
		{
			parseGCodeErrorB(pSG70Para->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM300);
			return -1;
		}

		//这里是外层本次调用重新取得的当前写槽，不是已提交的Q段槽位。
		//先初始化，避免把Q段的圆弧数据或一次性动作带入回程。
		initGCodeBufUnit(pGCodeBufUnit);
		inheritFromGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pGCodeBufUnit);

		//内部循环的快速定位使用OTHERTYPE，不能写成L1NE。
		SReturnLine.EntityType = OTHERTYPE;
		SReturnLine.SStartPoint =pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		SReturnLine.SEndPoint = pSG70Para->SCycleStartPoint;

		writeLineIntoGCodeBuf(&SReturnLine, pGCodeBufUnit);

		//恢复G70所在主程序段，使后作用指令附着在回程记录上。
		pSAssignConfig->SGetTextStateInfo = pSG70Para->SGetTextStateInfo;
		pSAssignConfig->SScanContainer = pSG70Para->SScanContainer;

		pGCodeBufUnit->SModalInfo.sGz =pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
		pGCodeBufUnit->SModalInfo.pSGzRegulation =pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation;

		//返回3时，外层不会走返回1/2的来源信息补充分支，需要这里填写。
		pGCodeBufUnit->unGCodeLineNum =pSAssignConfig->SGetTextStateInfo.unCurrLineNum;

		strncpy(pGCodeBufUnit->cFilename,pSCurrProgram->cFileName,FILENAME_LENGTH_LIMIT);
		pGCodeBufUnit->cFilename[FILENAME_LENGTH_LIMIT - 1] = '\0';

		//回程尚未在这里提交。
		//外层继续处理后作用指令，再更新系统状态并提交当前回程记录。
		return 3;
	}
	default:
		break;

	}

	return 0;

}


