#include "common.h"
#include "GeneralFunctions.h"
#include "GCode.h"
#include"ErrorFunction.h"
#include "channel.h"


#if STM32_MODE
#include"flread_task.h"    //M98/M97用到企业的查找文件名函数
#endif

#define ProgramVal_MAX_Length 4 //程序名数字最大长度  当前为4 如O1114

//*******************************M00***************************************
//添加M00规则
void addM00RegulationAPI()
{
	addMCommadRegulation(0, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM00, AssignM00);
}
//检查M00
int8 CheckM00(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}
	return 1;
}
//赋值M00
int8 AssignM00(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSAssignConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}

	//M00添加进IO缓冲区中，以及指令类型赋值
	setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_PAUSE, CMD_PROGRAM_ADDR);
	setIO(M00, pSGCodeBuffUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	return 1;

}
//*******************************M01***************************************
//添加M01规则
void addM01RegulationAPI()
{
	addMCommadRegulation(1, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM01, AssignM01);
}

//检查M01
int8 CheckM01(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
    if (pSSyntaxConfig == NULL || pSGCodeBuffUnit == NULL)
        return -1;
    /* 原有 M01 的 K/L/J 外部输入等待属于车床扩展语法。 */
    if (pSSyntaxConfig->SScanContainer.bL || pSSyntaxConfig->SScanContainer.bRIJK[2] ||
        pSSyntaxConfig->SScanContainer.bRIJK[3])
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM87);
		return -1;
	}
	return 1;
}
//赋值M01
int8 AssignM01(SAssignConfiguration* pSAssignConfig, struct SGCodeParse*pSGCodeBuffUnit)
{
	if (pSAssignConfig==NULL|| pSGCodeBuffUnit==NULL)
        return -1;
    if (pSAssignConfig->SScanContainer.bL|| pSAssignConfig->SScanContainer.bRIJK[2] ||
        pSAssignConfig->SScanContainer.bRIJK[3])
	{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM87);
		return -1;
    }

	//获取赋值结构体中的扫描结构体
			pSGCodeBuffUnit->SIOContrlInfo.cInputIOIndex = UNASSIGNED_VALUE;
			pSGCodeBuffUnit->SIOContrlInfo.nInputDetectTime = UNASSIGNED_VALUE;

	//M01添加进IO缓冲区中，以及指令类型赋值
	setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_OPTION_PAUSE, CMD_PROGRAM_ADDR);
	setIO(M01, pSGCodeBuffUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	return 1;

}


//*******************************M02***************************************
//添加M02规则
void addM02RegulationAPI()
{
	addMCommadRegulation(2, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM02, AssignM02);
}

//赋值M02
int8 AssignM02(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSAssignConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}

	setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_STOP_M02, CMD_PROGRAM_ADDR);
	//2.标记bLastLineMark
	pSGCodeBuffUnit->bLastLineMark = TRUE;

	pSAssignConfig->ucTargetStatus = GCODE_RUN2FINISHED;
	pSAssignConfig->SGetTextStateInfo.nEndLineNum = pSAssignConfig->SGetTextStateInfo.unCurrLineNum;
	pSAssignConfig->bAssignFinished = TRUE;
	return 1;
}

//检查M02
int8 CheckM02(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		return -1;
	}
	//1.子程序未出现M99
	SList* pSProgramList = getProgramList(0);
	if (pSProgramList->cSize > 1)
	{
		parseGCodeErrorB(0, 0);
		return -1;
	}

	pSSyntaxConfig->cTargetStatus = GCODE_RUN2FINISHED;
	pSSyntaxConfig->SGetTextStateInfo.nEndLineNum = pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum;
	pSSyntaxConfig->bCheckFinished = TRUE;
	//setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);
	return 1;
}

//*******************************M29***************************************
//添加M29规则
void addM29RegulationAPI()
{
	addMCommadRegulation(29, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM29, AssignM29);
}
//检查M29
int8 CheckM29(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		return -1;
	}

	pSSyntaxConfig->SGCodeCurrSysState.dTappingSpeed = pSSyntaxConfig->SScanContainer.dS;

	//清除扫描结构体的S值
	pSSyntaxConfig->SScanContainer.bS = FALSE;
	pSSyntaxConfig->SScanContainer.dS = 0;

	return 1;
}

//赋值M29
int8 AssignM29(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSAssignConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}
	pSAssignConfig->SGCodeCurrSysState.dTappingSpeed = pSAssignConfig->SScanContainer.dS;

	//清除扫描结构体的S值
	pSAssignConfig->SScanContainer.bS = FALSE;
	pSAssignConfig->SScanContainer.dS = 0;
	return 1;
}
//*******************************M30***************************************

//添加M30规则
void addM30RegulationAPI()
{
	addMCommadRegulation(30, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM30, AssignM30);
}
//检查M30
int8 CheckM30(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse*pSGCodeBuffUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		return -1;
	}
	//1.子程序未出现M99
	SList* pSProgramList = getProgramList(pSSyntaxConfig->ucChannelNum);
	if (pSProgramList->cSize > 1)
	{
		parseGCodeErrorB(0, 0);
		return -1;
	}
	//2.获取轨迹NS-NF时出现M30
	if(pSSyntaxConfig->cPart != 0)
	{
	
	}

	pSSyntaxConfig->cTargetStatus = GCODE_RUN2FINISHED;
	pSSyntaxConfig->SGetTextStateInfo.nEndLineNum = pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum;
	pSSyntaxConfig->bCheckFinished = TRUE;
	//setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);
	return 1;
}

//赋值M30
int8 AssignM30(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSAssignConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}
	//1.写指令类型
	setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);
	//2.标记bLastLineMark
	pSGCodeBuffUnit->bLastLineMark = TRUE;
	//3.设置赋值目标状态
	pSAssignConfig->ucTargetStatus = GCODE_RUN2FINISHED;
	pSAssignConfig->SGetTextStateInfo.nEndLineNum = pSAssignConfig->SGetTextStateInfo.unCurrLineNum;
	pSAssignConfig->bAssignFinished = TRUE;

	//4.1当前模式：自动模式

	return 1;
}

//*******************************M98***************************************


void addM98RegulationAPI()
{
	addMCommadRegulation(98, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM98, AssignM98);
}

/*************************************************
  函数名称：int8 CheckM98(SSyntaxConfiguration* pSSyntaxConfig)
  函数功能: 检查M98
  输入参数：pSSyntaxConfig		赋值结构体
  输出参数：
  返 回 值：
  备    注：1.格式M98 P****xxxx
**************************************************/
int8 CheckM98(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig==NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	//获取P值整数部分
	int32 nVal = (int32)pSSyntaxConfig->SScanContainer.dP;
	int8 cTempArr[TEMP_FILENAME_MAX_LENGTH] = { 0 };   //文件名

	//1.检查P值
	if (pSSyntaxConfig->SScanContainer.bP == TRUE)
	{
		//1.1参数P值不能小于0
		//1.2参数P值必须为整数
		//1.3参数P值位数大于8
		if (pSSyntaxConfig->SScanContainer.dP < 0 ||
			!isZero(pSSyntaxConfig->SScanContainer.dP - nVal) ||
			nVal> 99999999)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
			return -1;
		}
	}
	else
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
		return -1;
	}
	if (pSSyntaxConfig->SScanContainer.bL == TRUE)
	{
	    double dL = pSSyntaxConfig->SScanContainer.dL;
	    if (dL <= 0 || dL != (int32)dL || dL > INT16_MAX) {
	    	parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
	        return -1;
	    }
	}

	//2.STM32模式下，在FLASH中查找文件名是否存在
#if STM32_MODE
	convertProgramName((int32)pSSyntaxConfig->SScanContainer.dP, cTempArr);   //数值转字符串 获取文件名
	uint8 ret = FindFileInfo(cTempArr);//在FLASH中查找文件名
	if (ret == 0xF3)
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM89);//查找不到文件名
		return -1;
	}
	else if(ret == 0xF4 || ret == 0xF5 ||ret == 0xF6)
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM459);//查找不到文件名
		return -1;
	}
#endif

	pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
	return 1;
}

/*************************************************
  函数名称：int8 CheckM98(SSyntaxConfiguration* pSSyntaxConfig)
  函数功能: 检查M98
  输入参数：pSSyntaxConfig		赋值结构体
  输出参数：
  返 回 值：
  备    注：1.格式M98 P****xxxx
**************************************************/
int8 AssignM98(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{

	//1.开辟程序节点内存
	SProgram* pSubProgram = createAProgramStruct();   //创建一个子程序结构体
	if (pSubProgram == NULL)
	{
		return -1;
	}

	//2.获取子程序调用次数(前四位)
	int16 sRemainingCalls = pSAssignConfig->SScanContainer.dP / 10000;
	if (sRemainingCalls == 0)
	{
		sRemainingCalls = 1;
	}
	if (pSAssignConfig->SScanContainer.bL == TRUE)
	{
		sRemainingCalls = (int16)pSAssignConfig->SScanContainer.dL;
	}

	//3.获取子程序名的数值（后四位）
	int16 sFileNameVal;
	sFileNameVal = (int32)pSAssignConfig->SScanContainer.dP % 10000;
	
	//4.赋值子程序结构体
	//4.1写入子程序名
	convertProgramName(sFileNameVal, pSubProgram->cFileName);
	//4.2写入子程序调用次数
	pSubProgram->sRemainingCalls = sRemainingCalls;
	//4.2.写入M98的父程序行号
	pSubProgram->unFatherLineNum = pSAssignConfig->SGetTextStateInfo.unGetLineNum;

	//5.将子程序结构体插入链表中（头插）
	int8 cRet = insertList(getProgramList(pSAssignConfig->ucChannelNum), 0, pSubProgram, ORAM_NUM302);   //插入链表,头插
	if (cRet == FALSE)
	{
		freeProgramData(pSubProgram);   //插入失败，立即释放
		freeGCode(pSubProgram);
		pSubProgram = NULL;
		parseGCodeErrorB(0, GCODE_ERROR_NUM406);
		return -1;
	}

    //6.修改文本状态结构体
	pSAssignConfig->SGetTextStateInfo.unGetLineNum = 0;
	pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
	pSubProgram->sBeginLineNum = 0;

	return 1;

} 


//*******************************M99***************************************

/*************************************************
  函数名称：
  函数功能:
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void addM99RegulationAPI()
{
	addMCommadRegulation(99, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM99, AssignM99);
}


/*************************************************
  函数名称：SAssignConfiguration* getAssignConfig(uint8 ucChannelNum)
  函数功能: 检查M99
  输入参数：pSSyntaxConfig		赋值结构体
  输出参数：
  返 回 值：
  备    注：1.有三种存在方式：M99、M99 P**、M99 L**
			2.主程序中只允许出现M99或M99 L**，不允许出现M99 P**
			3.子程序中只允许出现M99或M99 P**，不允许出现M99 L**
**************************************************/
int8 CheckM99(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		parseGCodeErrorB(0, 0);
		return -1;
	}
	
	SList* pSProgramList = getProgramList(pSSyntaxConfig->ucChannelNum);
	SProgram* pSCurrProgram = getCurrentProgram(pSSyntaxConfig->ucChannelNum);

	//1.主程序调用M99
	if (isMainProgram(pSSyntaxConfig->ucChannelNum))
	{
		//1.1主程序：M99P**（不允许出现此情况）
		if (pSSyntaxConfig->SScanContainer.bP == TRUE)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, 0);
			return -1;
		}
		//1.2主程序：M99L**（主程序循环执行L次）
		else if (pSSyntaxConfig->SScanContainer.bL == TRUE)
		{
		    double dL = pSSyntaxConfig->SScanContainer.dL;
		    if (dL <= 0 || dL != (int32)dL || dL > INT16_MAX) {
		    	parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
		        return -1;
		    }
			//注：检查流程不能修改pSCurrProgram->sRemainingCalls
			int16 sRemainingCalls = pSCurrProgram->sRemainingCalls;

			if (sRemainingCalls == 0)
			{
				//此前已出现M99L**，报错，不允许主程序嵌套M99L**循环
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, 0);
				return -1;
			}

			//1.2.1剩余调用次数未写入（主程序第一次调用M99）
			if (sRemainingCalls == UNASSIGNED_VALUE)
			{
				//写入剩余调用次数
				sRemainingCalls = round(pSSyntaxConfig->SScanContainer.dL);
				//检查流程状态置为WAIT
				pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
			}
			//1.2.2剩余调用次数已写入（主程序非第一次调用M99）
			else
			{
				//剩余调用次数--
				sRemainingCalls--;
				//主程序剩余调用次数 > 0，继续调用主程序
				if (sRemainingCalls > 0)
				{
					//检查流程状态置为WAIT
					pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
				}
				//主程序无剩余调用次数，继续检查M99 L**后的程序
				else
				{
					//无需操作，如何防止主程序M99嵌套问题？？？
					//解决方法：主程序无调用次数后，结束检查
				}
			}
		}
		//1.2主程序：M99（主程序无限循环）
		else
		{
			//检查流程状态置为WAIT
			pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
		}
	}

	//2.子程序调用M99
	else
	{

		//1.1主程序：M99P**（不允许出现此情况）
		if (pSSyntaxConfig->SScanContainer.bL == TRUE)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, 0);
			return -1;
		}
		//注：检查流程不能修改pSCurrProgram->sRemainingCalls
		int16 sRemainingCalls = pSCurrProgram->sRemainingCalls;
		//调用次数--
		sRemainingCalls--;
		//2.1剩余调用次数 > 0，继续调用当前子程序
		if (sRemainingCalls > 0)
		{
			//检查流程状态置为WAIT
			pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
		}
		//2.2子程序无剩余调用次数：返回父程序目标行号or目标程序段号
		else
		{
			//2.1.1返回父程序调用的下一行：M99
			if (pSSyntaxConfig->SScanContainer.bP == FALSE)
			{
				//检查流程状态置为WAIT
				pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
			}
			//2.1.2返回父程序目标程序段号：M99 P**
			else
			{
				//检查流程状态置为WAIT
				pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
			}
		}
	}
	
	return 1;

}

int8 AssignM99(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	SList* pSProgramList = getProgramList(pSAssignConfig->ucChannelNum);
	SProgram* pSCurrProgram = getCurrentProgram(pSAssignConfig->ucChannelNum);

	//1.主程序调用M99
	if (isMainProgram(pSAssignConfig->ucChannelNum))
	{
		//1.1主程序：M99L**（主程序循环执行L次）
		if (pSAssignConfig->SScanContainer.bL == TRUE)
		{

			//1.1.1剩余调用次数未写入（主程序第一次调用M99）
			if (pSCurrProgram->sRemainingCalls == UNASSIGNED_VALUE)
			{
				//写入剩余调用次数
				pSCurrProgram->sRemainingCalls = round(pSAssignConfig->SScanContainer.dL) == 0 ? 1 : round(pSAssignConfig->SScanContainer.dL);
				//修改赋值流程获取行号
				pSAssignConfig->SGetTextStateInfo.unGetLineNum = pSCurrProgram->sBeginLineNum;
				//修改检查流程获取索引
				pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
			}
			//1.1.2剩余调用次数已写入（主程序非第一次调用M99）
			else
			{
				//调用次数--
				pSCurrProgram->sRemainingCalls--;
				//主程序剩余调用次数 > 0，继续调用主程序
				if (pSCurrProgram->sRemainingCalls > 0)
				{
					//修改赋值流程获取行号
					pSAssignConfig->SGetTextStateInfo.unGetLineNum = pSCurrProgram->sBeginLineNum;
					//修改赋值流程获取索引
					pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
				}
				//主程序无剩余调用次数，继续检查M99 L**后的程序
				else
				{
					//如何防止主程序M99嵌套问题？？？（嵌套报错）
					//无需操作，继续执行M99L**后面内容
				}
			}
		
		}
		//1.2主程序：M99（主程序无限循环）
		else
		{
			//pSAssignConfig->SGetTextStateInfo.unGetLineNum = pSCurrProgram->sBeginLineNum;
			pSAssignConfig->SGetTextStateInfo.unGetLineNum = 0;
			pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
			pSAssignConfig->ucTargetStatus = GCODE_WAIT;
			s_pSChannelConfig[0]->bSkipMode = g_SChannel[0].bSkipMode;
		}
		pSGCodeBuffUnit->bLastLineMark = TRUE;//最后一行标志位 ：在M99中bLastLineMark用于计件标志位
	}

	//2.子程序调用M99
	else
	{
		//0.调用次数--
		pSCurrProgram->sRemainingCalls--;

		//1.剩余调用次数 > 0：继续调用当前子程序
		if (pSCurrProgram->sRemainingCalls > 0)
		{
			pSAssignConfig->SGetTextStateInfo.unGetLineNum = pSCurrProgram->sBeginLineNum;
			pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
		}
		//2.剩余调用次数 = 0：返回父程序目标行号or目标程序段号
		else if (pSCurrProgram->sRemainingCalls == 0)
		{
			//2.1修改文本状态结构体
			//2.1.1返回目标行号
			if (pSAssignConfig->SScanContainer.bP == FALSE)
			{
				pSAssignConfig->SGetTextStateInfo.unGetLineNum = pSCurrProgram->unFatherLineNum;
				pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;

			}
			//2.1.2返回目标程序段号（需要找到程序段号对应的行号）
			else
			{
				pSAssignConfig->SScanContainer.cMacroType = 1;
				pSAssignConfig->SScanContainer.bGOTO = TRUE;
				pSAssignConfig->SScanContainer.dGOTO = pSAssignConfig->SScanContainer.dP;
			}

			//2.2释放当前程序节点内部开辟的内存（注：程序节点自身内存未释放）
			freeProgramData(pSCurrProgram);

			//2.3删除当前子程序节点（注：程序节点自身内存释放）
			removeNode(pSProgramList, 0, FALSE);
		}

	}

	return 1;

}


//*******************************M96***************************************

/*************************************************
  函数名称：
  函数功能:
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void addM96RegulationAPI()
{
	addMCommadRegulation(96, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM96, AssignM96);
}

/*************************************************
  函数名称：int8 CheckM96(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
  函数功能: 检查M96
  输入参数：
  输出参数：
  返 回 值：
  备    注
**************************************************/
int8 CheckM96(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if(pSSyntaxConfig==NULL||pSGCodeBuffUnit==NULL)
	{
		return -1;
	}
	//目标状态置为等待状态（等待赋值）？？？
	//M96应该不需要等待

	return 1;
}

/*************************************************
  函数名称：int8 AssignM96(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
  函数功能: 赋值M96
  输入参数：
  输出参数：
  返 回 值：-1 错误
             0 通道指令管理器中没有空闲的位置，退出
			 1 成功

  备    注
**************************************************/
int8 AssignM96(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSAssignConfig==NULL||pSGCodeBuffUnit==NULL)
	{
		return -1;
	}
	//目标通道
	uint8 ucTargetChannel = 1;
#if CHANNEL_NUM_MAX==2
	//双通道下，目标通道为通道1
	 ucTargetChannel = 1;
#else
	//多通道下，目标通道看实际情况
#endif
	 //将通道指令类型以及检测目标通道写入缓冲区中
	pSGCodeBuffUnit->sChannelCommand.ucType = CHANNEL_SIGNAL_CHECK;
	pSGCodeBuffUnit->sChannelCommand.SChannelData.ucTargetChannelNum = ucTargetChannel;

	return 1;
}



//*******************************M97***************************************
/*************************************************
  函数名称：
  函数功能:
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void addM97RegulationAPI()
{
	if(CHANNEL_NUM_MAX > 1)
	addMCommadRegulation(97, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM97, AssignM97);
}

/*************************************************
  函数名称：int8 CheckM97(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
  函数功能: 检查M97
  输入参数：pSSyntaxConfig 检查结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 CheckM97(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig==NULL||pSGCodeBuffUnit==NULL)
	{
		return -1;
	}
	//1.如果没出现参数P，报错
	if (pSSyntaxConfig->SScanContainer.bP == FALSE)
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM441);
		return -1;
	}
	else
	{
	    // 2. 出现参数P
	    int32 pNumber = (int32)(pSSyntaxConfig->SScanContainer.dP);
	    int maxVal = (int)pow(10, ProgramVal_MAX_Length);  // 如最大值为9999（若是4位）
	    //bool isUnEqualAndLessThanZero(pSSyntaxConfig->SScanContainer.dP-pow(10, ProgramVal_MAX_Length)==FALSE)
	   // bool pNumber = isUnEqualAndLessThanZero(pSSyntaxConfig->SScanContainer.dP-pow(10, ProgramVal_MAX_Length));
		//2.1如果P值大于4位数，报错 
		//if (isUnEqualAndLessThanZero(pSSyntaxConfig->SScanContainer.dP-pow(10, ProgramVal_MAX_Length)==FALSE))
	    if (pNumber < 0 || pNumber > maxVal)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM441);
			return -1;
		}
		//2.2 STM32模式下，查找是否有该文件名
#if STM32_MODE
		int8 cTempArr[TEMP_FILENAME_MAX_LENGTH] = { 0 };   //文件名
		convertProgramName(pSSyntaxConfig->SScanContainer.dP, cTempArr);   //数值转字符串 获取文件名
		uint8 ret = FindFileInfo(cTempArr);//在FLASH中查找文件名
		if (ret == 0xF3)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM89);//查找不到文件名
			return -1;
		}
		else if (ret == 0xF4 || ret == 0xF5 || ret == 0xF6)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM459);//查找不到文件名
			return -1;
		}
#endif
		return 1;
	}
}

/*************************************************
  函数名称：int8 AssignM97(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
  函数功能: 赋值M97
  输入参数：pSAssignConfig 赋值结构体  pSGCodeBuffUnit G代码缓冲区指针
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 AssignM97(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{

	if (pSAssignConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}

	////目标通道
	uint8 ucTargetChannel = 1;
#if CHANNEL_NUM_MAX==2
	//双通道下，目标通道为通道1
	ucTargetChannel = 1;
#else
	//多通道下，目标通道看实际情况
#endif

	//1.将通道指令类型、文件名数值以及目标通道写入缓冲区中
	pSGCodeBuffUnit->sChannelCommand.ucType = CHANNEL_SIGNAL_START;
	pSGCodeBuffUnit->sChannelCommand.SChannelData.cFileNameVal = (int32)(pSAssignConfig->SScanContainer.dP);
	pSGCodeBuffUnit->sChannelCommand.SChannelData.ucTargetChannelNum = ucTargetChannel;
	return 1;
}


//*******************************M100—M199***************************************
/*************************************************
  函数名称：
  函数功能:
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void addM100_M199RegulationAPI()
{
	if(CHANNEL_NUM_MAX > 1)
	addMCommadRegulation(CHANNEL_COMMAND_MIN_VAL, IO_POST_ACTION, cMb, 1, NULL, NULL, NULL, CheckM100_M199, AssignM100_M199);
}

/*************************************************
  函数名称：int8 CheckM100_M199(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
  函数功能: 检查M100-M199
  输入参数：pSSyntaxConfig 检查结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 CheckM100_M199(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit)
{
	if (pSSyntaxConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}
	
		return 1;
}

/*************************************************
  函数名称：int8 AssignM100_M199(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
  函数功能: 赋值M100-M199
  输入参数：pSAssignConfig 赋值结构体  pSGCodeBuffUnit G代码缓冲区指针
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 AssignM100_M199(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit)
{

	if (pSAssignConfig == NULL || pSGCodeBuffUnit == NULL)
	{
		return -1;
	}

	////相互等待的目标通道
	int8 cTargetChannel = 3; //0000 0011
#if CHANNEL_NUM_MAX==2
	//双通道下，相互等待的目标通道为通道0和通道1 （以位的方式表示位 0000 0011）
	cTargetChannel = 3;
#else
	//多通道下，目标通道看实际情况
#endif

	//1.将通道指令类型、以及相互等待的目标通道写入缓冲区中
	pSGCodeBuffUnit->sChannelCommand.ucType = CHANNEL_SIGNAL_WAIT;
	pSGCodeBuffUnit->sChannelCommand.SChannelData.ucTargetChannelNum = cTargetChannel;
	return 1;
}
