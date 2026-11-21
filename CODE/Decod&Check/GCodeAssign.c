#include<stdio.h>
#include <stdlib.h>
#include<string.h>
#include<math.h>
#include "Config.h"
#include"Motion.h"
#include"Gcode.h"
#include"GeneralFunctions.h"
#include"ErrorFunction.h"
#include"NCDataPara.h"
#include"IOCommand.h"
#include"ToolChanger.h"
#include"Channel.h"
#include "ringbuff.h"



/*************************************************
  函数名称：SAssignConfiguration* getAssignConfig(uint8 ucChannelNum)
  函数功能: 获取某个通道的G代码赋值结构体
  输入参数：ucChannelNum     通道号

  输出参数：
  返 回 值：
  备    注：
**************************************************/
SAssignConfiguration* getAssignConfig(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM301);
		return NULL;
	}
	SChannelConfiguration* pSChannelConfig = getChannelConfig(ucChannelNum);

	if (pSChannelConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return NULL;
	}

	return &pSChannelConfig->SAssignConfig;
}

/*************************************************
  函数名称：
  函数功能: 初始化检查结构体
  输入参数：pSAssignConfig 语法检查结构体
  输出参数：
  返 回 值：
  备    注：初始化结构体变量的步骤如下
			1.释放结构体中开辟的内存
			2.结构体内全部置0
			3.对结构体变量初始化
			4.对初始值非0的变量赋值
**************************************************/
void initAssignConfig(struct _SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum)
{
	//1.释放赋值结构体中开辟的内存
	//结构体内未开辟内存

	//2.清空检查结构体
	memset(pSAssignConfig, 0, sizeof(struct _SAssignConfiguration));

//3.初始化结构体变量
	//3.1初始化获取文本状态
	initGetTextStateInfo(&pSAssignConfig->SGetTextStateInfo);
	//3.2初始化扫描结构体
	initScanContainer(&pSAssignConfig->SScanContainer, TRUE);
	//3.3初始化系统状态结构体
	initGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState,  ucChannelNum);

//4.初始化非0的变量
	//4.1初始化通道号
	pSAssignConfig->ucChannelNum = ucChannelNum;
	//4.2初始化G代码检查状态机
	pSAssignConfig->ucStatus = GCODE_IDLE;
	pSAssignConfig->ucTargetStatus = UNASSIGNED_VALUE;
	//4.3初始化G代码切片
	pSAssignConfig->cPart = SYNTAX_PART_1;
	//4.4初始化起始单节
#if STM32_MODE
	//pSAssignConfig->usMainProStartLine = re_gcoderunline(ucChannelNum);//获取起始单节
	pSAssignConfig->SGetTextStateInfo.unGetLineNum = re_gcoderunline(ucChannelNum);

	if (g_SChannel[0].cSubRunMode == 0)
	{
		pSAssignConfig->SGetTextStateInfo.unGetLineNum = 0;
	}
#else
	//pSAssignConfig->usMainProStartLine = 0;//获取起始单节
	pSAssignConfig->SGetTextStateInfo.unGetLineNum = 0;
#endif
	//4.5初始化G代码环形缓冲区首地址
	pSAssignConfig->pGCodeBuf = getGCodeHeadAddress(ucChannelNum);

	pSAssignConfig->SGetTextStateInfo.nEndLineNum = UNASSIGNED_VALUE;
	//4.6偏置值的初始化

	//。。。。。。。。。。。。。
}

/*************************************************
  函数名称：void assignStatusMachine(SAssignConfiguration *pSAssignConfig, uint8 ucChannelNum)
  函数功能：赋值状态机
  输入参数：pSAssignConfig赋值结构体，ucChannelNum通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void assignStatusMachine(SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum)
{

	switch (pSAssignConfig->ucStatus)
	{
	case GCODE_IDLE:  //不运行
	{
		break;
	}

	case GCODE_RUN:  //直接运行
	{
		SSyntaxConfiguration* pSSyntaxConfig = getSyntaxConfig(ucChannelNum);
		SProgram* pSCurProgram = getCurrProgramStruct(ucChannelNum);

		//赋值必须同时满足以下条件：
		//1.有程序起始行号 || （无起始行号 && 切片不为Part1）
		//说明：为了保证Part1-Part8执行的连贯性，不被打断（如：加入程序节点后程序起始行号变化）
		//2.检查赋值差距大于指定值 || （检查赋值差距小于指定值 && 检查状态不为RUN）
		//说明：为了保证检查进度快于赋值进度
		if (pSCurProgram->sBeginLineNum != UNASSIGNED_VALUE || (pSCurProgram->sBeginLineNum == UNASSIGNED_VALUE && (pSAssignConfig->cPart != ASSIGN_PART_1 || pSSyntaxConfig->ucStatus == GCODE_FINISHED)))
		{
			if (pSSyntaxConfig->unTotalCommandCnt - pSAssignConfig->unTotalCommandCnt > 5 || (pSSyntaxConfig->unTotalCommandCnt - pSAssignConfig->unTotalCommandCnt <= 5 && pSSyntaxConfig->ucStatus != GCODE_RUN))
			{
				assignSegments(pSAssignConfig, ucChannelNum);
			}
		}
		
		//	if (pSCurProgram->sBeginLineNum == UNASSIGNED_VALUE ||
		//	(pSSyntaxConfig->ucStatus == GCODE_RUN &&
		//		pSSyntaxConfig->unTotalCommandCnt - pSAssignConfig->unTotalCommandCnt <= 5))
		//{
		//	if (pSAssignConfig->cPart != ASSIGN_PART_1)
		//	{
		//		
		//	}
		//}
		//else
		//{
		//	assignSegments(pSAssignConfig, ucChannelNum);
		//}

		//assignSegments(pSAssignConfig, ucChannelNum);

		break;
	}

	case GCODE_RUN2FINISHED:
	{
		assignRun2Finished(pSAssignConfig, ucChannelNum);
		break;
	}

	case GCODE_FINISHED:  //不运行
	{
		break;
	}

	case GCODE_WARN:
	{
		break;
	}
	case GCODE_WAIT:  //只做了双通道的等待//lxw:G代码解析各通道独立，取消通道间等待，只需在切换程序文本时进行等待
	{
		struct SChannel* pSChannel = g_SChannel;
		if (pSChannel->cRunChannelGcodeAgain == TRUE)//(陈敏聪	2023/8/28增 处于M99的情况下，让G代码重新启动)  zxd:20251029添加
		{
			pSChannel->cRunChannelGcodeAgain = FALSE;
            changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, 0);
		}

		//void changeGCodeStatus(uint8 ucSyntaxOrAssign, uint8 ucChannelNum, uint8 ucTargetStatus, bool bReport, uint32 unLineNum)

		//changeGCodeStatus(ASSIGN,ucChannelNum, GCODE_RUN, 1, 0);
		break;
	}
	case GCODE_ASSIGN_WAIT:  //不运行
	{
		break;
	}

	default:
		parseGCodeErrorB(0, GCODE_ERROR_NUM306);
		break;
	}
}


/*************************************************
  函数名称：void assignSegments(struct _SAssignConfiguration*pSAssignConfig, uint8 ucChannelNum)
  函数功能：分阶段将当前程序行赋值为 G 代码缓冲记录并提交。
  输入参数：pSAssignConfig 赋值状态；ucChannelNum 通道号。
  输出参数：写槽、赋值阶段、编程状态和提交指针，按当前阶段更新。
  返 回 值：无；F 校验失败时报警并返回，不推进当前赋值阶段。
  备    注：F 失败后不执行 S/T 或运动赋值；此前已执行的模态和前作用赋值不回滚。
**************************************************/
void assignSegments(struct _SAssignConfiguration*pSAssignConfig, uint8 ucChannelNum)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}
	if (ucChannelNum > CHANNEL_NUM_MAX - 1)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM301);
		return;
	}

	switch (pSAssignConfig->cPart)
	{
	//Part1：处理程序文本
	case ASSIGN_PART_1:
	{
		int8 cRet;
		SProgram* pSCurrProgram = NULL;   //当前程序结构体
		//1.获取当前程序结构体
		pSCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);
		if (pSCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		//2.处理程序文本
		cRet = processProgramText(GC_ASSIGN, &pSAssignConfig->SGetTextStateInfo, ucChannelNum, pSCurrProgram);
		//获取到程序正文
		if (cRet == 1)
		{
			//if (pSAssignConfig->SGetTextStateInfo.unCurrLineNum == 42);
			pSAssignConfig->cCommandSegmentCnt++;
			pSAssignConfig->cPart = ASSIGN_PART_2;
		}
		//(ucRet == 0 || ucRet == 2 || ucRet == 3)，不需要执行后续切片
		//0：程序文本未处理完、2：已处理%、3：已记录DEFINE
		break;
	}

	//Part2：初始化扫描结构体
	case ASSIGN_PART_2:
	{
		//1.扫描前需要初始化扫描结构体
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
		pSAssignConfig->cPart = ASSIGN_PART_3;

		break;
	}

	//Part3：扫描程序段
	case ASSIGN_PART_3:
	{
		int8 cRet;

		int16 sLastCommandVal;//记录上一程序段Ga/Gz组指令
		int16 sCurrCommandVal;//记录当前程序段Ga/Gz组指令
		const SGCommandRegulation* pSLastCommandRegulation;//记录上一程序段Ga/Gz组指令规则地址
		const SGCommandRegulation* pSCurrCommandRegulation;//记录当前程序段Ga/Gz组指令规则地址

		//1.获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		//2.扫描程序段
		cRet = scanSegment(GC_ASSIGN, &pSAssignConfig->SGetTextStateInfo, &pSAssignConfig->SScanContainer, pCurrProgram, ucChannelNum);
		
		//3.当前程序段全部扫描完成
		if (cRet == TRUE)
		{
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
			if(pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
			{
				sCurrCommandVal = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
				pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
				pSAssignConfig->pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
			}
			//2.2当前程序段出现Gz组指令
			else if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
			{
				sCurrCommandVal = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
				pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
				pSAssignConfig->pSCurrCommandRegulation = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
			}
			//2.3当前程序段未出现GaGz组指令,则取当前系统状态Ga组指令
			else
			{
				sCurrCommandVal = pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa;
				pSCurrCommandRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
				pSAssignConfig->pSCurrCommandRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
			}

			//3.判断多程序段指令的每一程序段运动相关指令是否一致（赋值不需要判断）
			if (pSAssignConfig->cCommandSegmentCnt == 1)
			{
				pSAssignConfig->sCurrCommandVal = sCurrCommandVal;
			}
			else
			{
				if (pSAssignConfig->sCurrCommandVal != sCurrCommandVal)
				{
					//多程序段指令的第一段程序段的指令与后续程序段指令不一致，如：第一段为G71，第二段非G71
					parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
					break;
				}
			}

			//4.判断是否需要释放开辟指令参数结构体内存（仅运动相关指令第一程序段需要判断）
			if (pSAssignConfig->cCommandSegmentCnt == 1 && (sLastCommandVal != sCurrCommandVal))
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
			else if (pSAssignConfig->cCommandSegmentCnt == 1 && (sLastCommandVal == sCurrCommandVal))
			{
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
				pSCurrCommandRegulation->pGetParaAPI(GC_ASSIGN, &pSAssignConfig->SScanContainer, ucChannelNum, &pSAssignConfig->SGCodeCurrSysState);
			}

			//6.判断cGa,cGz指令所有程序段是否扫描完成
			if (pSAssignConfig->cCommandSegmentCnt == pSCurrCommandRegulation->cSegmentNum)
			{
				//6.1运动相关指令所有程序段已全部扫描
				pSAssignConfig->cPart = ASSIGN_PART_4;
			}
			else
			{
				//6.2还需继续获取运动相关指令程序段
				pSAssignConfig->cPart = ASSIGN_PART_1;
			}
		}

		break;
	}

	//Part4：初始化缓冲区单元，并完成对上一缓冲区部分数据的继承
	//		 赋值非Ga/Gz组指令、前作用M指令、FST指令
	case ASSIGN_PART_4:
	{
		//1.判断容量
		if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) <= 0)
		{
			break;
		}

		//2.获取当前G代码缓冲区写单元
		int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
		struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];

		//3.缓冲区当前G代码缓冲区写单元
		initGCodeBufUnit(pSGCodeParseUnit);

		//4.继承上一缓冲区部分数据（如：刀具号，刀补号等）
		inheritFromGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pSGCodeParseUnit);

		//5.赋值非Ga/Gz组指令
		assignGbCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGcCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGdCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGeCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGfCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGgCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGjCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGhCommand(pSAssignConfig, pSGCodeParseUnit);
		assignGxCommand(pSAssignConfig, pSGCodeParseUnit);
		

		//6.赋值前作用M指令
		for (int i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
		{
			if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i] != NULL)
			{
				if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->cType == IO_PRE_ACTION)
				{
					if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI != NULL)
					{
						pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
					}
				}
			}
		}

		assignFCommand(pSAssignConfig, pSGCodeParseUnit);
		assignSCommand(pSAssignConfig, pSGCodeParseUnit);
		assignTCommand(pSAssignConfig, pSGCodeParseUnit);

		pSAssignConfig->cPart = ASSIGN_PART_5;

		break;
	}

	//Part5：赋值Ga/Gz组指令
	case ASSIGN_PART_5:
	{
		//1.判断容量
		if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) <= 1)
		{
			break;
		}
		
		//2.获取当前G代码缓冲区写单元
		int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
		struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];

		int8 cRet = 0;
		
		//3.调用对应指令的赋值接口
		if (pSAssignConfig->pSCurrCommandRegulation->pAssignCommandAPI != NULL)
		{
			//注意！！！
			//调用一次赋值接口只能写一个缓冲区，即指令接口内写完一段轨迹后必须返回
			cRet = pSAssignConfig->pSCurrCommandRegulation->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		}

		//4.1Ga/Gz组指令赋值接口内不补充缓冲区剩余内容、不处理多个缓冲区间的过渡
		if (cRet == 1 || cRet == 2)
		{
			//写入文件名
			SProgram* pSCurrProgram = getCurrProgramStruct(ucChannelNum);
			strncpy(pSGCodeParseUnit->cFilename,pSCurrProgram->cFileName, FILENAME_LENGTH_LIMIT);
			pSGCodeParseUnit->cFilename[FILENAME_LENGTH_LIMIT - 1] = 0;

			//写缓冲区中G代码行号信息
			pSGCodeParseUnit->unGCodeLineNum = pSAssignConfig->SGetTextStateInfo.unCurrLineNum;
			//写缓冲区中Ga组模态信息
			if (pSAssignConfig->pSCurrCommandRegulation->cGroup == sGa)
			{
				pSGCodeParseUnit->SModalInfo.sGa = pSAssignConfig->sCurrCommandVal;
				pSGCodeParseUnit->SModalInfo.pSGaRegulation = pSAssignConfig->pSCurrCommandRegulation;
			}
			//写缓冲区中Gz组模态信息
			else
			{
				pSGCodeParseUnit->SModalInfo.sGz = pSAssignConfig->sCurrCommandVal;
				pSGCodeParseUnit->SModalInfo.pSGzRegulation = pSAssignConfig->pSCurrCommandRegulation;
			}

			//4.1在Ga/Gz组指令赋值接口中赋了值，但未赋值完（对缓冲区单元间进行过渡）
			if (cRet == 1)
			{
				//更新G代码系统状态
				updataGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pSGCodeParseUnit);
				//移动指针
				moveGCodeWritePtr(pSAssignConfig->ucChannelNum);
        /* 倒圆写入失败时不计完成段数，也不切回下一行赋值阶段。 */
        if (isErrorHasBeenWritten()) return;

				//指针移动，需要再次获取当前G代码缓冲区写单元
				int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
				struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];
				//初始化G代码缓冲区单元
				initGCodeBufUnit(pSGCodeParseUnit);
				//从G代码系统状态中继承
				inheritFromGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pSGCodeParseUnit);
			}
			//4.2在Ga/Gz组指令赋值接口中赋了值，且已赋值完
			else if (cRet == 2)
			{
				pSAssignConfig->cPart = ASSIGN_PART_6;
			}
		}
		//4.2Ga/Gz组指令赋值接口内补充缓冲区剩余内容、且处理了多个缓冲区间的过渡
		if (cRet == 3)
		{
			pSAssignConfig->cPart = ASSIGN_PART_6;
		}

		break;
	}

	//Part6：赋值后作用指令
	case ASSIGN_PART_6:
	{		
		//1.判断容量
		if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) <= 0)
		{
			break;
		}

		//2.获取当前G代码缓冲区写单元
		int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
		struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];

		//3.赋值后作用M
		for (int i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
		{
			if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i] != NULL)
			{
				if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->cType == IO_POST_ACTION)
				{
					if (pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI != NULL)
					{
						pSAssignConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
					}
				}
			}
		}
	   
		pSAssignConfig->cPart = ASSIGN_PART_7;
		break;
	}

	//Part7：赋值特殊指令
	case ASSIGN_PART_7:
	{
		//1.判断容量
		if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) <= 0)
		{
			break;
		}

		int8 cRet;
		int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
		struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];

		cRet = assignSpecialCommand(pSAssignConfig, pSGCodeParseUnit);
		
		if (cRet == 1)
		{
			pSAssignConfig->cPart = ASSIGN_PART_8;
		}
		break;
	}


	//Part7：赋值特殊指令，并完成缓冲区单元间的过渡
	case ASSIGN_PART_8:
	{		
		// 1.判断容量，当前段与可能插入的过渡段最多占两槽；容量不足保持本阶段，等待下游释放。
		if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) <= 1)
		{
			break;
		}

		//2.获取当前G代码缓冲区写单元
		int16 sIndex = getSGCodeParsePointerIndex(GCODE_WRITE, pSAssignConfig->ucChannelNum);
		struct SGCodeParse* pSGCodeParseUnit = &pSAssignConfig->pGCodeBuf[sIndex];

		//保存到当前G代码系统状态结构体
		updataGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pSGCodeParseUnit);

		moveGCodeWritePtr(pSAssignConfig->ucChannelNum);
        if (isErrorHasBeenWritten()) return;
		
		pSAssignConfig->unTotalCommandCnt += pSAssignConfig->cCommandSegmentCnt;

		pSAssignConfig->cCommandSegmentCnt = 0;
		pSAssignConfig->cPart = ASSIGN_PART_1;

		//赋值判断是否结束
		//当前赋值行号==程序结束行号


		//判断是否改变赋值状态
		if (pSAssignConfig->ucTargetStatus != UNASSIGNED_VALUE && pSAssignConfig->ucStatus != pSAssignConfig->ucTargetStatus)
		{
			changeGCodeStatus(GC_ASSIGN, ucChannelNum, pSAssignConfig->ucTargetStatus, TRUE, 0);
			pSAssignConfig->ucTargetStatus = UNASSIGNED_VALUE;
		}



		break;
	}
	
	default:
		parseGCodeErrorB(0, GCODE_ERROR_NUM304);
		break;
	}

}

/*************************************************
  函数名称：void assignRun2Finished(SAssignConfiguration *pSAssignConfig, uint8 ucChannelNum)
  函数功能：赋值完成前的操作
  输入参数：pSAssignConfig赋值结构体，ucChannelNum通道号
  输出参数：
  返 回 值：
  备    注：G

**************************************************/
void assignRun2Finished(SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
	if (pChannelConfig == NULL)
	{
		return;
	}




	//1.释放G代码解析开辟的内存
	//freeAllGcode();    注释掉的原因是为了适配双通道
	freeGCodeChannelConfig(pChannelConfig);
	pChannelConfig->ucStatus = GCODE_IDLE;
	freeAllCommandParaStruct(ucChannelNum);

	//2.在G代码缓冲区中写入标志bLastLineMark

	//3.

	//4.





//	setBFinshed(&pSAssignConfig->bFinshed, GC_ASSIGN, ucChannelNum);    //设置同步标记BFinshed

	if (pSAssignConfig->SGCodeCurrSysState.sMb == M30)  //M30回到第0行	陈敏聪	2023/9/1	陈敏聪注释	运行结束无论情况，直接将行号复位
	{
		pSAssignConfig->SGCodeCurrSysState.sMb = UNASSIGNED_VALUE;
	}

	//freeMemAssign(pSAssignConfig);	lxw:赋值结构体中无动态内存


	//pSAssignConfig->sBeginLineNum = UNASSIGNED_VALUE;	//lxw beginLineNum移动


	g_SChannel[ucChannelNum].bGcodeWriteOverFlag = TRUE;   //G代码写完标志位



	changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_FINISHED, TRUE, pSAssignConfig->SGetTextStateInfo.unCurrLineNum);  //状态更改为运行到完成

	//g_bGCodeDraw[ucChannelNum] = FALSE;

}

//赋值Gb组指令
//- 1：参数异常或API缺失；
//0：本段无该模态指令（pTemp == NULL）；
//1：执行成功。
int8 assignGbCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGbRegulation;

	//当前程序段出现Gb组指令
	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
//赋值Gc组指令
int8 assignGcCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGcRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
int8 assignGdCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGdRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
int8 assignGeCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGeRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1 ;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
int8 assignGfCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1 ;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGfRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1 ;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
int8 assignGgCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1 ;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGgRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
int8 assignGjCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGjRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}

int8 assignGhCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanModalInfo.pSGhRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}
int8 assignGxCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1 ;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSAssignConfig->SScanContainer.SScanNonModalInfo.pSGxRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pAssignCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		pTemp->pAssignCommandAPI(pSAssignConfig, pSGCodeParseUnit);
		return 1;
	}
	else
	{
		return 0;
	}

}

/*************************************************
  函数名称：assignFCommand
  函数功能：校验本次扫描得到的 F，并按当前单位和进给模态写入进给。
  输入参数：pSAssignConfig 含本段扫描及继承模态；pGCodeBufUnit 为未转换模态的写槽，均非空。
  输出参数：成功时更新 dFeedrate；未写 F 或失败时保持写槽和扫描容器不变。
  返 回 值：1 成功或未写 F；-1 非法 F，已报 130，调用方必须停止当前段。
  备    注：仅实际执行的 G84/G88 允许负 F；保留 F0，不校验未写 F 时的继承进给。
**************************************************/
int8 assignFCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	//程序段中出现F才赋值
	if (pSAssignConfig->SScanContainer.bF == TRUE)
	{
		// 用局部变量保存原始F，不修改扫描容器
		double dFeedMm = pSAssignConfig->SScanContainer.dF;
		int16 sCurrCommandVal = pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa;

		// G70 借用主赋值结构，必须按本段扫描及继承模态识别实际指令，顺序与指令分派一致。
		if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
		{
			sCurrCommandVal = pSAssignConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
		}
		else if (pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
		{
			sCurrCommandVal = pSAssignConfig->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
		}

		if (!isfinite(dFeedMm) || (dFeedMm < 0
			&& sCurrCommandVal != G84 * pow(10, GCOMMAND_DECIMAL_PLACE)
			&& sCurrCommandVal != G88 * pow(10, GCOMMAND_DECIMAL_PLACE)))
		{
			parseGCodeErrorB(pSAssignConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM130);
			return -1;
		}
		if (pGCodeBufUnit->SModalInfo.sGe ==G20 * pow(10, GCOMMAND_DECIMAL_PLACE))// G20下，将长度单位由英寸转换为毫米，G21下不进入此分支，数值保持不变
		{
			dFeedMm *= 25.4;
		}
		// 有限的英寸值也可能在换算时溢出；校验完成前不写入进给。
		if (!isfinite(dFeedMm))
		{
			parseGCodeErrorB(pSAssignConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM130);
			return -1;
		}
		pGCodeBufUnit->dFeedrate = pGCodeBufUnit->SModalInfo.sGg == G99 * pow(10, GCOMMAND_DECIMAL_PLACE) ?
			dFeedMm : dFeedMm / (60 * 1000); // G98时转速单位由 mm / min 转成 mm / ms
	}
	return 1;
}

int8 assignSCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig->SScanContainer.bS == TRUE)
	{
		//1.对dSpindleSpeed赋值
		if (pSAssignConfig->cSpindleSpeedAssignFlag == 0)
		{
			pGCodeBufUnit->dSpindleSpeed = pSAssignConfig->SScanContainer.dS;

		}
		//2.对dSecondSpindleSpeed赋值
		else if (pSAssignConfig->cSpindleSpeedAssignFlag == 1)
		{
			pGCodeBufUnit->dSecondSpindleSpeed = pSAssignConfig->SScanContainer.dS;


		}
	}
	return 1;
}
int8 assignTCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	//程序段中出现T才赋值
	if (pSAssignConfig->SScanContainer.bT == TRUE)
	{
		int8 cToolNum;//刀具号
		int8 cToolCompNum;//刀补号
		int32 nT = (int32)pSAssignConfig->SScanContainer.dT;

		if (nT < 100)
		{
			cToolCompNum = nT;
			cToolNum = pSAssignConfig->SGCodeCurrSysState.ucToolID;
		}
		else
		{
			cToolNum = nT / 100;
			cToolCompNum = nT % 100;
		}
		pGCodeBufUnit->ucToolID = cToolNum;
		pGCodeBufUnit->ucToolCompNum = cToolCompNum;

		if (cToolCompNum < 1 || cToolCompNum > TOOL_NUM_MAX)
		{
			setErrorType(GCODE_ERROR_NUM129);
			findError(pSAssignConfig->SGetTextStateInfo.pStartPtr, pSAssignConfig->SGetTextStateInfo.unCurrLineNum);
			return -1;
		}

		//更新到系统状态
		pSAssignConfig->SGCodeCurrSysState.ucToolID = cToolNum;				//实时跟新刀号
		pSAssignConfig->SGCodeCurrSysState.ucToolCompNum = cToolCompNum;	//实时跟新刀补号

		//更新刀具偏置值（长度+磨耗）
		pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolLengthOffset = g_SSystemPara.SToolPara[pSAssignConfig->ucChannelNum][cToolCompNum - 1].SToolCoord;
		pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolWearOffset = pSAssignConfig->SGCodeCurrSysState.SToolWearPara[cToolCompNum - 1];

		//更新总偏置
		calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	}
	return 1;
}

int8 assignSpecialCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	int8 cRet = 1;

	//1.B类宏关键字
	if (pSAssignConfig->SScanContainer.cMacroType != 0)
	{
		switch (pSAssignConfig->SScanContainer.cMacroType)
		{
			//GOTO
		case 1:
		{
			cRet = assignGOTO(pSAssignConfig, pGCodeBufUnit);
			break;
		}
		//IF
		case 2:
		{
			cRet = assignIF(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &pSAssignConfig->SGetTextStateInfo);
			break;
		}
		//ELSEIF
		case 3:
		{
			cRet = assignELSEIF(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &pSAssignConfig->SGetTextStateInfo);
			break;
		}
		//ELSE
		case 4:
		{
			cRet = assignELSE(pSAssignConfig->ucChannelNum, &pSAssignConfig->SGetTextStateInfo);
			break;
		}
		//ENDIF
		case 5:
		{
			cRet = assignENDIF(pSAssignConfig->ucChannelNum, &pSAssignConfig->SGetTextStateInfo);
			break;
		}
		//WHILE
		case 6:
		{
			cRet = assignWHILE(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &pSAssignConfig->SGetTextStateInfo);
			break;

		}
		//ENDWHILE
		case 7:
		{
			cRet = assignENDWHILE(pSAssignConfig->ucChannelNum, &pSAssignConfig->SGetTextStateInfo);
			break;
		}
		default:
			cRet = 1;
			break;
		}

	}

	//2.G66宏程序调用
	SProgram* pSCurrProgram = getCurrProgramStruct(pSAssignConfig->ucChannelNum);
	if (pSCurrProgram->ucG66MarkAssign && isTriggerG66Command(pSAssignConfig->sCurrCommandVal) && pGCodeBufUnit->EntityType != DOT)
	{
		SChannelConfiguration* pSGCodeChannel = getChannelConfig(pSAssignConfig->ucChannelNum);
		SG66InfoStruct* pSG66Info = pSGCodeChannel->pSG66InfoArr[pSCurrProgram->ucG66ArrIndex];
		triggerG66ByAssign(pSAssignConfig, pSG66Info);
	}

	return cRet;
}



