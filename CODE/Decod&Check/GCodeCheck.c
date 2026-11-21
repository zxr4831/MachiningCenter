#include<stdio.h>
#include <stdlib.h>
#include<string.h>
#include<math.h>

#include "ringbuff.h"
#include "Config.h"
#include"GeneralFunctions.h"


#include"Gcode.h"

#include"NCDataPara.h"
#include"IOCommand.h"
#include"ToolChanger.h"
#include"Channel.h"


#if STM32_MODE
#include "dir.h"  
#include "app_warning.h"
#endif





/*************************************************
  函数名称：SSyntaxConfiguration* getSyntaxConfig(uint8 ucChannelNum)
  函数功能: 获取某个通道的G代码检查结构体
  输入参数：ucChannelNum     通道号

  输出参数：
  返 回 值：
  备    注：
**************************************************/
SSyntaxConfiguration* getSyntaxConfig(uint8 ucChannelNum)
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

	return &pSChannelConfig->SSyntaxConfig;
}


/*************************************************
  函数名称：void initSyntaxConfig(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum)
  函数功能: 初始化检查结构体
  输入参数：pSSyntaxConfig	检查结构体
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：初始化结构体变量的步骤如下
			1.释放结构体中开辟的内存
			2.结构体内全部置0
			3.对结构体变量初始化
			4.对初始值非0的变量赋值
			（一次G代码解析仅初始化一次）
**************************************************/
void initSyntaxConfig(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.释放赋值结构体中开辟的内存
	//结构体内未开辟内存

	//2.清空检查结构体
	memset(pSSyntaxConfig, 0, sizeof(struct _SSyntaxConfiguration));

//3.初始化结构体变量
	//3.1初始化获取文本状态
	initGetTextStateInfo(&pSSyntaxConfig->SGetTextStateInfo);
	//3.2初始化扫描结构体
	initScanContainer(&pSSyntaxConfig->SScanContainer, TRUE);
	//3.3初始化系统状态结构体
	initGCodeSystemState(&pSSyntaxConfig->SGCodeCurrSysState,  ucChannelNum);

//4.初始化非0的变量
	//4.1初始化通道号
	pSSyntaxConfig->ucChannelNum = ucChannelNum;
	//4.2初始化G代码检查状态机
	pSSyntaxConfig->ucStatus = GCODE_IDLE;
	//4.3初始化G代码切片
	pSSyntaxConfig->cPart = SYNTAX_PART_1;
	//4.4
	pSSyntaxConfig->cTargetStatus = UNASSIGNED_VALUE;

	pSSyntaxConfig->SGetTextStateInfo.nEndLineNum= UNASSIGNED_VALUE;

	//4.4初始化起始单节
#if STM32_MODE
	//pSSyntaxConfig->usMainProStartLine = re_gcoderunline(ucChannelNum);//获取起始单节
	pSSyntaxConfig->SGetTextStateInfo.unGetLineNum = re_gcoderunline(ucChannelNum);
	if (g_SChannel[0].cSubRunMode == 0)
	{
		pSSyntaxConfig->SGetTextStateInfo.unGetLineNum = 0;
	}
#else
	//pSSyntaxConfig->usMainProStartLine = 0;//获取起始单节
	pSSyntaxConfig->SGetTextStateInfo.unGetLineNum = 0;
#endif
	//4.5初始化G代码解析结构体
	initGCodeBufUnit(&pSSyntaxConfig->SCurrRecord);
	//4.6偏置值的初始化

	//。。。。。。。。。。。。。

}


/*************************************************
  函数名称：void syntaxStatusMachine(struct _SSyntaxConfiguration *pSSyntaxConfig, uint8 ucChannelNum)
  函数功能：语法检查状态机
  输入参数：pSSyntaxConfig语法结构体
			ucChannelNum通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void syntaxStatusMachine(struct _SSyntaxConfiguration *pSSyntaxConfig, uint8 ucChannelNum)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	switch (pSSyntaxConfig->ucStatus)
	{
	case GCODE_IDLE:  //不运行
	{
		break;
	}

	case GCODE_RUN:  //直接运行
	{
		checkSegments(pSSyntaxConfig, ucChannelNum);
		//判断目标状态是否发生改变
		if (pSSyntaxConfig->cTargetStatus != pSSyntaxConfig->ucStatus && pSSyntaxConfig->cPart == SYNTAX_PART_1)
		{
			//changeGCodeStatus(GC_SYNTAX, ucChannelNum, pSSyntaxConfig->ucTargetStatus, TRUE, 0);

		}
		break;
	}
	case GCODE_RUN2FINISHED:
	{
		changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_FINISHED, TRUE, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
		break;
	}

	case GCODE_FINISHED:  //不运行
	{
		break;
	}

	case GCODE_WARN:
	{
	///	//重读字符串
	///	pSSyntaxConfig->ucStatus = GCODE_RUN;  //修改状态，再次检查此行   G71好像不行啊！！！
		break;
	}
	case GCODE_WAIT:  //等待指令
	{
		SAssignConfiguration* pSAssignConfig = getAssignConfig(ucChannelNum);
		
		if (pSSyntaxConfig->unTotalCommandCnt == pSAssignConfig->unTotalCommandCnt)
		{
			changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN, TRUE, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);

			//遇到程序控制指令，如M98，M99，G65，G66
			//检查流程不要找获取行号。因为跨程序文件时，检查找行号找不对
			//将赋值流程的获取行号 赋值 检查流程的获取行号
			pSSyntaxConfig->SGetTextStateInfo.unGetLineNum = pSAssignConfig->SGetTextStateInfo.unGetLineNum;
			pSSyntaxConfig->SGetTextStateInfo.ucOriStrIndex = pSAssignConfig->SGetTextStateInfo.ucOriStrIndex;
			//行内偏移必须配套原始行和注释状态；扫描指针仍使用检查流程自身的缓冲。
			memcpy(pSSyntaxConfig->SGetTextStateInfo.cOriginalStr, pSAssignConfig->SGetTextStateInfo.cOriginalStr, sizeof(pSSyntaxConfig->SGetTextStateInfo.cOriginalStr));
			pSSyntaxConfig->SGetTextStateInfo.bIgnored = pSAssignConfig->SGetTextStateInfo.bIgnored;

			pSSyntaxConfig->unTotalCommandCnt = 0;
			pSAssignConfig->unTotalCommandCnt = 0;
		}


		////新架构中G代码解析各通道完全独立，等待状态机仅用于程序文本间的切换？
		//if (getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_IDLE || getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_FINISHED 
  //          || getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_ASSIGN_WAIT)
		//{
		//	changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
		//	
		//}
		//else if (getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_RUN || getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_WAIT 
  //          || getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_RUN2FINISHED )
		//{
		//	//不用处理
		//}

		//else 
		//{
		//	//用于捕获未考虑的情况
		//	parseGCodeErrorB(0, GCODE_ERROR_NUM306);
		//}

		break;
	}
	case GCODE_SYNTAX_WAIT:  //不运行
	{

		//SAssignConfiguration* pSAssignConfig_Channel = getAssignConfig(ucChannelNum);  //赋值结构体
		//if (pSAssignConfig_Channel->bFinshed==TRUE)
		//{
		//	pSAssignConfig_Channel->bFinshed = FALSE;//防止语法进入此状态时，赋值的bfinshed为1，而语法的bfinshed为0
		//}
		//break;
	}

	default:
		parseGCodeErrorB(0, GCODE_ERROR_NUM306);
		break;
	}
}

/*************************************************
  函数名称：void checkSegments(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：pSSyntaxConfig语法结构体
			ucChannelNum通道号
  输出参数：
  返 回 值：
  备    注：！！！注意检查对规则结构体函数指针访问前一定要判断是否为空！！！
**************************************************/
void checkSegments(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}
	if (ucChannelNum > CHANNEL_NUM_MAX - 1)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM301);
		return;
	}

	switch (pSSyntaxConfig->cPart)
	{
	//从文本中获取一段程序段
	case SYNTAX_PART_1:
	{
		uint8 ucRet;
		SProgram* pSCurrProgram = NULL;   //当前程序结构体
		pSCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);  //获取当前程序结构体
		if (pSCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		//ucRet = processProgramTextCheck(pSSyntaxConfig, ucChannelNum, pSCurrProgram);
		ucRet = processProgramText(GC_SYNTAX, &pSSyntaxConfig->SGetTextStateInfo, ucChannelNum, pSCurrProgram);

		//程序文本未处理完||已处理%||已记录DEFINE，不需要执行后续切片
		if (ucRet == 0 || ucRet == 2 || ucRet == 3)
		{
			//不需要处理
		}
		else if (ucRet == 1)
		{
			pSSyntaxConfig->cCommandSegmentCnt++;
			pSSyntaxConfig->cPart = SYNTAX_PART_2;
		}
		break;
	}

	//扫描程序段，将指令字放入模态结构体，扫描结构体
	case SYNTAX_PART_2:
	{
		//扫描前需要初始化扫描结构体
		//当前扫描程序段为指令的第一段，需要清空扫描结构体
		if (pSSyntaxConfig->cCommandSegmentCnt == 1)
		{
			initScanContainer(&pSSyntaxConfig->SScanContainer, 1);
		}
		//当前扫描程序段非指令的第一段，不需清空MFST值
		//防止多程序段丢失MFST的值
		else
		{
			initScanContainer(&pSSyntaxConfig->SScanContainer, 0);
		}

		//扫描前将pCurrPtr指向程序段字符串首字符
		pSSyntaxConfig->SGetTextStateInfo.pCurrPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;

		pSSyntaxConfig->cPart = SYNTAX_PART_3;

		break;
	}

	case SYNTAX_PART_3:
	{
		uint8 ucRet;
		int16 sLastCommandVal;
		int16 sCurrCommandVal;
		const SGCommandRegulation* pSLastCommandRegulation;
		const SGCommandRegulation* pSCurrCommandRegulation;


		//获取当前程序结构体
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);   
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		//扫描程序段
		ucRet = scanSegment(GC_SYNTAX, &pSSyntaxConfig->SGetTextStateInfo, &pSSyntaxConfig->SScanContainer, pCurrProgram, ucChannelNum);

		//当前程序段扫描完成
		if (ucRet == TRUE)
		{
			//1.获取上一程序段运动相关指令（Gz组优先级高于Ga组）
			if (pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.sGz != 0)
			{
				sLastCommandVal = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.sGz;		
				pSLastCommandRegulation = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.pSGzRegulation;

			}
			else
			{
				sLastCommandVal = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.sGa;
				pSLastCommandRegulation = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
			}

			//2.获取当前程序段运动相关指令
			//2.1当前程序段出现Ga组指令
			if (pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
			{
				sCurrCommandVal = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
				pSCurrCommandRegulation = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
				pSSyntaxConfig->pSCurrCommandRegulation = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGaRegulation;
			}
			//2.2当前程序段出现Gz组指令
			else if (pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
			{
				sCurrCommandVal = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
				pSCurrCommandRegulation = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
				pSSyntaxConfig->pSCurrCommandRegulation = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGzRegulation;
			}
			//2.3当前程序段未出现GaGz组指令,则取当前系统状态Ga组指令
			else
			{
				sCurrCommandVal = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.sGa;
				pSCurrCommandRegulation = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
				pSSyntaxConfig->pSCurrCommandRegulation = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;
			}

			//3.判断多程序段指令的每一程序段运动相关指令是否一致
			if (pSSyntaxConfig->cCommandSegmentCnt == 1)
			{
				pSSyntaxConfig->sCurrCommandVal = sCurrCommandVal;
			}
			else
			{
				if (pSSyntaxConfig->sCurrCommandVal != sCurrCommandVal)
				{
					//多程序段指令的第一段程序段的指令与后续程序段指令不一致，如：第一段为G71，第二段非G71
					parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
					break;
				}
			}

			//4.判断是否需要释放开辟指令参数结构体内存（仅运动相关指令第一程序段需要判断）
			if (pSSyntaxConfig->cCommandSegmentCnt == 1 && (sLastCommandVal != sCurrCommandVal))
			{
				//4.1释放上一指令字的参数结构体
				if (pSLastCommandRegulation->pFreeParaStructAPI != NULL)
				{
					pSLastCommandRegulation->pFreeParaStructAPI(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
				}
				//4.2开辟当前指令字的参数结构体
				if (pSCurrCommandRegulation->pMallocParaStructAPI != NULL)
				{
					if (pSCurrCommandRegulation->pMallocParaStructAPI(GC_SYNTAX, ucChannelNum) <= 0)
					{
						parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
						break;
					}
				}
			}
			else if (pSSyntaxConfig->cCommandSegmentCnt == 1 && (sLastCommandVal == sCurrCommandVal))
			{
				//4.2开辟当前指令字的参数结构体
				if (pSCurrCommandRegulation->pMallocParaStructAPI != NULL)
				{
					if (pSCurrCommandRegulation->pMallocParaStructAPI(GC_SYNTAX, ucChannelNum) <= 0)
					{
						parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
						break;
					}
				}
			}

			//5.判断当前指令字是否需要写参数结构体
			if (pSCurrCommandRegulation->pGetParaAPI != NULL)
			{
				pSCurrCommandRegulation->pGetParaAPI(GC_SYNTAX, &pSSyntaxConfig->SScanContainer, ucChannelNum, &pSSyntaxConfig->SGCodeCurrSysState);
			}

			//6.判断cGa,cGz指令所有程序段是否扫描完成
			if (pSSyntaxConfig->cCommandSegmentCnt == pSCurrCommandRegulation->cSegmentNum)
			{				
				//6.1运动相关指令所有程序段已全部扫描
				pSSyntaxConfig->cPart = SYNTAX_PART_4;
			}
			else
			{
				//6.2还需继续获取运动相关指令程序段
				pSSyntaxConfig->cPart = SYNTAX_PART_1;
			}
		}

		break;
	}

	case SYNTAX_PART_4:
	{
		//缓冲区初始化
		initGCodeBufUnit(&pSSyntaxConfig->SCurrRecord);

		//缓冲区继承1：从当前解析通道状态中继承（如：刀具号，刀补号等）
		inheritFromGCodeSystemState(&pSSyntaxConfig->SGCodeCurrSysState, &pSSyntaxConfig->SCurrRecord);

		//检查cGa、cGz以外的G指令
		checkGbCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGcCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGdCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGeCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGfCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGgCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGjCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGhCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
		checkGxCommand(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);

		//检查前作用M
		for (int i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
		{
			if (pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i] != NULL)
			{
				if (pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->cType == IO_PRE_ACTION)
				{
					if (pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pCheckCommandAPI != NULL)
					{
						pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pCheckCommandAPI(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
					}
				}
			}
		}



		//检查F,S,T
		checkFCommand(pSSyntaxConfig, &pSSyntaxConfig->SCurrRecord);
		checkSCommand(pSSyntaxConfig, &pSSyntaxConfig->SCurrRecord);
		checkTCommand(pSSyntaxConfig, &pSSyntaxConfig->SCurrRecord);

		pSSyntaxConfig->cPart = SYNTAX_PART_5;

		break;
	}

	case SYNTAX_PART_5:
	{
		int8 cRet = 0;
		struct SGCodeParse* pSGCodeParseUnit = &pSSyntaxConfig->SCurrRecord;
		//对当前cGa、cGz指令进行检查
		
		//方式1：用函数包装起来
		//checkGaOrGzCommand(struct _SSyntaxConfiguration* pSSyntaxConfig)

		//方式2：不进行包装
		//要加上空指针检查
		//1.检查写入缓冲区？？？？？？？？？？
		//2. 检查直接写入状态结构体？？？？？？？？？？？
		if (pSSyntaxConfig->pSCurrCommandRegulation->pCheckCommandAPI != NULL)
		{	
			cRet = pSSyntaxConfig->pSCurrCommandRegulation->pCheckCommandAPI(pSSyntaxConfig, &pSSyntaxConfig->SCurrRecord);
		}

		//cGa、cGz指令检查完成
		if (cRet == 1 || cRet == 2)
		{
			//写缓冲区中G代码行号信息
			pSGCodeParseUnit->unGCodeLineNum = pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum;
			//写缓冲区中Ga组模态信息
			if (pSSyntaxConfig->pSCurrCommandRegulation->cGroup == sGa)
			{
				pSGCodeParseUnit->SModalInfo.sGa = pSSyntaxConfig->sCurrCommandVal;
				pSGCodeParseUnit->SModalInfo.pSGaRegulation = pSSyntaxConfig->pSCurrCommandRegulation;
			}
			//写缓冲区中Gz组模态信息
			else
			{
				pSGCodeParseUnit->SModalInfo.sGz = pSSyntaxConfig->sCurrCommandVal;
				pSGCodeParseUnit->SModalInfo.pSGzRegulation = pSSyntaxConfig->pSCurrCommandRegulation;
			}
			//4.1在Ga/Gz组指令赋值接口中赋了值，但未赋值完（对缓冲区单元间进行过渡）
			if (cRet == 1)
			{
				//更新G代码系统状态
				updataGCodeSystemState(&pSSyntaxConfig->SGCodeCurrSysState, pSGCodeParseUnit);

				//初始化G代码缓冲区单元
				initGCodeBufUnit(pSGCodeParseUnit);
				//从G代码系统状态中继承
				inheritFromGCodeSystemState(&pSSyntaxConfig->SGCodeCurrSysState, pSGCodeParseUnit);
			}
			else if (cRet == 2)
			{
				pSSyntaxConfig->cPart = ASSIGN_PART_6;
			}
		}

		break;
	}

	case SYNTAX_PART_6:
	{
		//检查后作用M
		for (int i = 0; i < IO_MAXGROUP_IN_SINGLE_LINE; i++)
		{
			if (pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i] != NULL)
			{
				if (pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->cType == IO_POST_ACTION)
				{
					if (pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pCheckCommandAPI != NULL)
					{
						pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSMCommandArray[i]->pCheckCommandAPI(pSSyntaxConfig,&pSSyntaxConfig->SCurrRecord);
					}
				}
			}
		}

		pSSyntaxConfig->cPart = SYNTAX_PART_7;

		break;
	}

	case SYNTAX_PART_7:
	{		
		int8 cRet = 0;
		cRet = checkSpecialCommand(pSSyntaxConfig, &pSSyntaxConfig->SCurrRecord);

		if (cRet == 1)
		{
			pSSyntaxConfig->cPart = ASSIGN_PART_8;
		}
		break;

	}

	case SYNTAX_PART_8:
	{

		if (pSSyntaxConfig->cTargetStatus != UNASSIGNED_VALUE)
		{
			changeGCodeStatus(GC_SYNTAX, ucChannelNum, (uint8)pSSyntaxConfig->cTargetStatus, TRUE, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
			pSSyntaxConfig->cTargetStatus = UNASSIGNED_VALUE;
		}

		//更新G代码系统状态
		updataGCodeSystemState(&pSSyntaxConfig->SGCodeCurrSysState,&pSSyntaxConfig->SCurrRecord);

		pSSyntaxConfig->unTotalCommandCnt += pSSyntaxConfig->cCommandSegmentCnt;
		pSSyntaxConfig->cCommandSegmentCnt = 0;

		pSSyntaxConfig->cPart = SYNTAX_PART_1;
		break;
	}
	default:
		parseGCodeErrorB(0, GCODE_ERROR_NUM304);
		break;
	}
}


//检查非Ga、Gz组指令
void checkGbCommand(struct _SSyntaxConfiguration* pSSyntaxConfig,struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGbRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
void checkGcCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGcRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
void checkGdCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGdRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
void checkGeCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGeRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
void checkGfCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGfRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
void checkGgCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGgRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
/*************************************************
  函数名称：checkGjCommand
  函数功能：调用当前程序段 j 组指令的检查接口。
  输入参数：pSSyntaxConfig 为检查配置；pSGCodeParseUnit 为有效的当前检查记录。
  输出参数：由对应接口更新检查状态；配置为空或检查接口缺失时报 300。
  返 回 值：无。
  备    注：本段无 j 组指令时保留原模态；报警处理沿用其他组分发函数。
**************************************************/
void checkGjCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	const SGCommandRegulation* pTemp; // 当前程序段的 j 组指令规则。
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGjRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			// 未给出指令的检查函数。
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}

void checkGhCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanModalInfo.pSGhRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}
void checkGxCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	SGCommandRegulation* pTemp;
	pTemp = pSSyntaxConfig->SScanContainer.SScanNonModalInfo.pSGxRegulation;

	if (pTemp != NULL)
	{
		if (pTemp->pCheckCommandAPI == NULL)
		{
			//未给出指令的检查函数
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		pTemp->pCheckCommandAPI(pSSyntaxConfig, pSGCodeParseUnit);
	}
}


//检查FST指令
void checkFCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	//程序段中出现F才检查
	if (pSSyntaxConfig->SScanContainer.bF == TRUE)
	{
		//2026.10.10 唐子杰 进给速度F不能为负数（G84/G88攻丝指令的F允许为负，表示左旋）
		if (pSSyntaxConfig->SScanContainer.dF < 0 
			&&pSSyntaxConfig->sCurrCommandVal != 84 * pow(10, GCOMMAND_DECIMAL_PLACE) 
			&&pSSyntaxConfig->sCurrCommandVal != 88 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM130);
			return;
		}
		pGCodeBufUnit->dFeedrate = pSSyntaxConfig->SScanContainer.dF;
	}
}
void checkSCommand(struct _SSyntaxConfiguration*pSSyntaxConfig,  struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig->SScanContainer.bS == TRUE )
	{
		if(pSSyntaxConfig->SScanContainer.dS < 0.0)
		{
			setErrorType(GCODE_ERROR_NUM130);
			findError(pSSyntaxConfig->SGetTextStateInfo.pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
			return;
		}
		
		//1.对dSpindleSpeed赋值
		if (pSSyntaxConfig->cSpindleSpeedAssignFlag == 0)
		{
			pGCodeBufUnit->dSpindleSpeed = pSSyntaxConfig->SScanContainer.dS;
		}
		//2.对dSecondSpindleSpeed赋值
		else if (pSSyntaxConfig->cSpindleSpeedAssignFlag == 1)
		{
			pGCodeBufUnit->dSecondSpindleSpeed = pSSyntaxConfig->SScanContainer.dS;
		}
	}
}
void checkTCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}
	//程序段中出现T才检查
	if (pSSyntaxConfig->SScanContainer.bT == TRUE)
	{
		int8 cToolNum;//刀具号
		int8 cToolCompNum;//刀补号

	//以下对dT进行检查：
		//1.dT必须为正整数
		if (!isZero(pSSyntaxConfig->SScanContainer.dT - (int)pSSyntaxConfig->SScanContainer.dT) || pSSyntaxConfig->SScanContainer.dT < 0)
		{
			setErrorType(GCODE_ERROR_NUM129);
			findError(pSSyntaxConfig->SGetTextStateInfo.pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
			return;
		}

		//2.dT最大为4位数
		if (pSSyntaxConfig->SScanContainer.dT > 9999)
		{
			setErrorType(GCODE_ERROR_NUM129);
			findError(pSSyntaxConfig->SGetTextStateInfo.pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
			return;
		}
	//nT检查结束

		int32 nT = (int32)pSSyntaxConfig->SScanContainer.dT;
		
		if (nT < 100)
		{
			//企业要求，当长度小于等于2时，如T02,改变的是刀长号加磨损号
			cToolNum = pSSyntaxConfig->SGCodeCurrSysState.ucToolID;
			cToolCompNum = nT;

		}
		else
		{
			cToolNum = nT / 100;
			cToolCompNum = nT % 100;

		}

		if (cToolCompNum < 1 || cToolCompNum > TOOL_NUM_MAX)
		{
			setErrorType(GCODE_ERROR_NUM129);
			findError(pSSyntaxConfig->SGetTextStateInfo.pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
			return;
		}

		pSGCodeParseUnit->ucToolID = cToolNum;
		pSGCodeParseUnit->ucToolCompNum = cToolCompNum;


		pSSyntaxConfig->SGCodeCurrSysState.ucToolID = cToolNum;				//实时跟新刀号
		pSSyntaxConfig->SGCodeCurrSysState.ucToolCompNum = cToolCompNum;	//实时跟新刀补号

		//更新刀具偏置值（长度+磨耗）
		pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolLengthOffset = g_SSystemPara.SToolPara[pSSyntaxConfig->ucChannelNum][cToolCompNum - 1].SToolCoord;
		pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolWearOffset = pSSyntaxConfig->SGCodeCurrSysState.SToolWearPara[cToolCompNum - 1];

		//更新总偏置
		calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	}
}

//检查特殊指令
int8 checkSpecialCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	int8 cRet = 1;
	//1.B类宏关键字
	if (pSSyntaxConfig->SScanContainer.cMacroType != 0)
	{
		switch (pSSyntaxConfig->SScanContainer.cMacroType)
		{
			//GOTO
		case 1:
		{
			cRet = checkGOTO(pSSyntaxConfig, pGCodeBufUnit);
			break;
		}
		//IF
		case 2:
		{
			cRet = checkIF(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSSyntaxConfig->SGetTextStateInfo);
			break;
		}
		//ELSEIF
		case 3:
		{
			cRet = checkELSEIF(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSSyntaxConfig->SGetTextStateInfo);
			break;
		}
		//ELSE
		case 4:
		{
			cRet = checkELSE(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SGetTextStateInfo);
			break;
		}
		//ENDIF
		case 5:
		{
			cRet = checkENDIF(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SGetTextStateInfo);
			break;
		}
		//WHILE
		case 6:
		{
			cRet = checkWHILE(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSSyntaxConfig->SGetTextStateInfo);
			break;
		}
		//ENDWHILE
		case 7:
		{
			cRet = checkENDWHILE(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SGetTextStateInfo);
			break;
		}
		default:
			cRet = 1;
			break;
		}

	}

	return cRet;

}

/*************************************************
  函数名称：void findErrorStart(struct _SSyntaxConfiguration *pSSyntaxConfig,uint16 ErrorType,int8 character)
  函数功能：找出错误，并传入错误缓冲区
  输入参数：pSSyntaxConfig 语法检查结构体,ErrorType错误类型，character小写字母
  输出参数：
  返 回 值：

  备    注：
**************************************************/
void findErrorStart(struct _SSyntaxConfiguration* pSSyntaxConfig, uint16 ErrorType, int8 character)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
	while (*pStartPtr != character && *pStartPtr != character + 32)
	{
		pStartPtr += 1;
	}
	setErrorType(ErrorType);
	findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
}

/*************************************************
  函数名称：void findError(const int8* const pStartPtr,  const uint32 SGetTextStateInfo.unLineNum)
  函数功能：找出错误，并传入错误缓冲区
  输入参数：pStartPtr指针指向词法单元开头，SGetTextStateInfo.unLineNum行号
  输出参数：
  返 回 值：

  备    注：
**************************************************/
void findError(const int8* pStartPtr, const uint32 unLineNum)
{
	if (pStartPtr == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	const int8* pAssistStart = NULL;                  //用于辅助记录错误起点
	const int8* pAssistEnd = NULL;                    //用于辅助记录错误终点
	setErrorWriteFlag(TRUE);

	pAssistStart = pStartPtr;
	pAssistEnd = pStartPtr + 1;

	while (1)
	{
		if ((*pAssistEnd >= 'a' && *pAssistEnd <= 'z') || (*pAssistEnd >= 'A' && *pAssistEnd <= 'Z') 
			|| *pAssistEnd == '\0' || *pAssistEnd == '\r' || *pAssistEnd == '\n')
		{
			break;
		}

		pAssistEnd += 1;
	}

	if(parseGCodeError(pAssistStart, unLineNum, (uint8)(pAssistEnd - pAssistStart)) == FALSE)  //调佣错误函数
	{
        #if STM32_MODE
		if(Check_ErrorNum(OTHER_ERROR_NUM410)==0)// 查其他故障编号是否重复
		Other_Error_Add(OTHER_ERROR_NUM410);
        #endif
	}
	pAssistStart = NULL;
	pAssistEnd = NULL;
}
