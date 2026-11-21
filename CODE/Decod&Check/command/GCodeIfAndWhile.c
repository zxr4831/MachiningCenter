
#include "ErrorFunction.h"
#include "list.h"
#include "GCode.h"

static int8 s_cIfAndWhielStepCheck[CHANNEL_NUM_MAX];
static int8 s_cIfAndWhielStepAssign[CHANNEL_NUM_MAX];


static SList* getCurrEnableStack(uint8 ucMod, uint8 ucChannelNum)
{
	SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
	if (pCurrProgram == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return NULL;
	}

	SList* pCurrEnableStack = ucMod == GC_SYNTAX ? pCurrProgram->SMacro.pEnableStackSyntax : pCurrProgram->SMacro.pEnableStackAssign;

	return pCurrEnableStack;
}


//添加IF等特殊指令到G代码状态机  2025.11.27
int8 checkIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepCheck[ucChannelNum])
	{
	case 0:
	{
		//0.开辟可执行栈节点内存
		SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
		if(pCurrNodeData == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM406);
			return -1;
		}
		memset(pCurrNodeData, 0, sizeof(SMacroExecutable));

		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_SYNTAX, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.赋值当前IF单元，并压入可执行栈
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			//当前dIF为真
			if (!isZero(pSScan->dIF))
			{
				pCurrNodeData->ucType = MACRO_IF;
				pCurrNodeData->ucExecuteSign = MACRO_OK;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}

			}
			//当前dIF为假
			else
			{
				pCurrNodeData->ucType = MACRO_IF;
				pCurrNodeData->ucExecuteSign = MACRO_NOT_OK;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}

			}
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dIF为真
					if (!isZero(pSScan->dIF))
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dIF为假
					else
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_NOT_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
				//栈顶IF单元 不满足执行or禁止执行
				else
				{
					pCurrNodeData->ucType = MACRO_IF;
					pCurrNodeData->ucExecuteSign = MACRO_FORBID;
					//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
					{
					    freeGCode(pCurrNodeData);
					    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					    return -1;
					}
				}
			}
			//2.2.2栈顶单元为WHILE
			else if (pTopNodeData->ucType == MACRO_WHILE)
			{
				//栈顶WHILE单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dIF为真
					if (!isZero(pSScan->dIF))
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dIF为假
					else
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_NOT_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
				//栈顶WHILE单元 不满足执行or禁止执行
				else
				{
					pCurrNodeData->ucType = MACRO_IF;
					pCurrNodeData->ucExecuteSign = MACRO_FORBID;
					//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
					{
					    freeGCode(pCurrNodeData);
					    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					    return -1;
					}
				}

			}
		}

		//3.
		if (pCurrNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 1;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_SYNTAX, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
            s_cIfAndWhielStepCheck[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}
	return -1;
}
int8 assignIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepAssign[ucChannelNum])
	{
	case 0:
	{
		//0.开辟可执行栈节点内存
		SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
		if(pCurrNodeData == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM406);
			return -1;
		}
		memset(pCurrNodeData, 0, sizeof(SMacroExecutable));

		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_ASSIGN, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.赋值当前IF单元，并压入可执行栈
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			//当前dIF为真
			if (!isZero(pSScan->dIF))
			{
				pCurrNodeData->ucType = MACRO_IF;
				pCurrNodeData->ucExecuteSign = MACRO_OK;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}
			}
			//当前dIF为假
			else
			{
				pCurrNodeData->ucType = MACRO_IF;
				pCurrNodeData->ucExecuteSign = MACRO_NOT_OK;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}
			}
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dIF为真
					if (!isZero(pSScan->dIF))
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dIF为假
					else
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_NOT_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
				//栈顶IF单元 不满足执行or禁止执行
				else
				{
					pCurrNodeData->ucType = MACRO_IF;
					pCurrNodeData->ucExecuteSign = MACRO_FORBID;
					//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
					{
					    freeGCode(pCurrNodeData);
					    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					    return -1;
					}
				}
			}
			//2.2.2栈顶单元为WHILE
			else if (pTopNodeData->ucType == MACRO_WHILE)
			{
				//栈顶WHILE单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dIF为真
					if (!isZero(pSScan->dIF))
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dIF为假
					else
					{
						pCurrNodeData->ucType = MACRO_IF;
						pCurrNodeData->ucExecuteSign = MACRO_NOT_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
				//栈顶WHILE单元 不满足执行or禁止执行
				else
				{
					pCurrNodeData->ucType = MACRO_IF;
					pCurrNodeData->ucExecuteSign = MACRO_FORBID;
					//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
					{
					    freeGCode(pCurrNodeData);
					    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					    return -1;
					}
				}

			}
		}

		//3.
		if (pCurrNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 1;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_ASSIGN, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
        	s_cIfAndWhielStepAssign[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}
	return -1;
}

int8 checkELSEIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepCheck[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_SYNTAX, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元 满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					pTopNodeData->ucExecuteSign = MACRO_FORBID;


				}
				//栈顶IF单元 不满足执行
				else if (pTopNodeData->ucExecuteSign == MACRO_NOT_OK)
				{
					//当前dELSEIF为真
					if (!isZero(pSScan->dELSEIF))
					{
						pTopNodeData->ucExecuteSign = MACRO_OK;
					}
					//当前dIF为假
					else
					{
					}
				}
				//栈顶IF单元 禁止执行
				else
				{

				}
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}

		//3.
		SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
		if (pTopNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 1;
			return 0;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_SYNTAX, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
            s_cIfAndWhielStepCheck[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}
    return -1;
}
int8 assignELSEIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepAssign[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_ASSIGN, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元 满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					pTopNodeData->ucExecuteSign = MACRO_FORBID;


				}
				//栈顶IF单元 不满足执行
				else if (pTopNodeData->ucExecuteSign == MACRO_NOT_OK)
				{
					//当前dELSEIF为真
					if (!isZero(pSScan->dELSEIF))
					{
						pTopNodeData->ucExecuteSign = MACRO_OK;
					}
					//当前dIF为假
					else
					{
					}
				}
				//栈顶IF单元 禁止执行
				else
				{

				}
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}

		//3.
		SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
		if (pTopNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 1;
			return 0;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_ASSIGN, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
        	s_cIfAndWhielStepAssign[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}
   return -1;
}

int8 checkELSE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepCheck[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_SYNTAX, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元 满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					pTopNodeData->ucExecuteSign = MACRO_FORBID;
				}
				//栈顶IF单元 不满足执行
				else if (pTopNodeData->ucExecuteSign == MACRO_NOT_OK)
				{
					pTopNodeData->ucExecuteSign = MACRO_OK;
				}
				//栈顶IF单元 禁止执行
				else
				{
				}
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}

		//3.
		SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
		if (pTopNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 1;
			return 0;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_SYNTAX, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
            s_cIfAndWhielStepCheck[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}
    return -1;
}
int8 assignELSE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepAssign[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_ASSIGN, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元 满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					pTopNodeData->ucExecuteSign = MACRO_FORBID;
				}
				//栈顶IF单元 不满足执行
				else if (pTopNodeData->ucExecuteSign == MACRO_NOT_OK)
				{
					pTopNodeData->ucExecuteSign = MACRO_OK;
				}
				//栈顶IF单元 禁止执行
				else
				{
				}
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}

		//3.
		SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
		if (pTopNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 1;
			return 0;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_ASSIGN, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}
   return -1;
}

int8 checkENDIF(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepCheck[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_SYNTAX, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元出栈
				removeNode(pCurrEnableStack, 0, FALSE);
				return 1;
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}
		break;
	}
	default:
		break;
	}
	return -1;
}
int8 assignENDIF(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepAssign[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_ASSIGN, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//栈顶IF单元出栈
				removeNode(pCurrEnableStack, 0, FALSE);
				return 1;
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}
		break;
	}
	default:
		break;
	}
   return -1;
}

int8 checkWHILE(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepCheck[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_SYNTAX, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.1栈顶为空，需要开辟栈节点，赋值后并压入
		if (pCurrEnableStack->cSize == 0)
		{
			//开辟可执行栈节点内存
			SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
			if(pCurrNodeData == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				return -1;
			}
			memset(pCurrNodeData, 0, sizeof(SMacroExecutable));
			pCurrNodeData->unEndLineNum == UNASSIGNED_VALUE;

			//当前dWHILE为真
			if (!isZero(pSScan->dWHILE))
			{
				pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
				pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
				pCurrNodeData->ucType = MACRO_WHILE;
				pCurrNodeData->ucExecuteSign = MACRO_OK;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}
			}
			//当前dWHILE为假
			else
			{
				pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
				pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
				pCurrNodeData->ucType = MACRO_WHILE;
				pCurrNodeData->ucExecuteSign = MACRO_FORBID;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}
			}
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF，需要开辟栈节点，赋值后并压入
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//开辟可执行栈节点内存
				SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
				if(pCurrNodeData == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return -1;
				}
				memset(pCurrNodeData, 0, sizeof(SMacroExecutable));
				pCurrNodeData->unEndLineNum == UNASSIGNED_VALUE;

				//栈顶IF单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dWHILE为真
					if (!isZero(pSScan->dWHILE))
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dWHILE为假
					else
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_FORBID;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
				//栈顶IF单元 不满足执行or禁止执行
				else
				{
					pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
					pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
					pCurrNodeData->ucType = MACRO_WHILE;
					pCurrNodeData->ucExecuteSign = MACRO_FORBID;
					//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
					{
					    freeGCode(pCurrNodeData);
					    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					    return -1;
					}
				}
			}

			//2.2.2栈顶单元为WHILE，且与当前WHILE为同一单元，则不需要开辟栈节点内存，仅修改栈顶单元
			else if (pTopNodeData->ucType == MACRO_WHILE && pTopNodeData->unLineNum == pSGetTextStateInfo->unCurrLineNum)
			{
				//当前dWHILE为真
				if (!isZero(pSScan->dWHILE))
				{
					pTopNodeData->ucExecuteSign = MACRO_OK;
				}
				//当前dWHILE为假
				else
				{
					pTopNodeData->ucExecuteSign = MACRO_FORBID;
					if (pTopNodeData->unEndLineNum != UNASSIGNED_VALUE)
					{
						pSGetTextStateInfo->unGetLineNum = pTopNodeData->unEndLineNum;
						pSGetTextStateInfo->ucOriStrIndex = 0;
						return 1;
					}
					//应该不存在
					else
					{
						parseGCodeErrorB(0, 0);
						return 1;
					}

				}

			}

			//2.2.3栈顶单元为WHILE，且与当前WHILE为不同单元，需要开辟栈节点，赋值后并压入
			else if (pTopNodeData->ucType == MACRO_WHILE && pTopNodeData->unLineNum != pSGetTextStateInfo->unCurrLineNum)
			{
				//开辟可执行栈节点内存
				SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
				if(pCurrNodeData == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return -1;
				}
				memset(pCurrNodeData, 0, sizeof(SMacroExecutable));
				pCurrNodeData->unEndLineNum == UNASSIGNED_VALUE;

				//栈顶WHILE单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dWHILE为真
					if (!isZero(pSScan->dWHILE))
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dWHILE为假
					else
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_FORBID;
						insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					}
				}


			}
		}

		//3.判断是否WHILE是否赋值完成
		SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
		if (pTopNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 1;
			return 0;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_SYNTAX, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
            s_cIfAndWhielStepCheck[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}

}
int8 assignWHILE(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepAssign[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_ASSIGN, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.1栈顶为空，需要开辟栈节点，赋值后并压入
		if (pCurrEnableStack->cSize == 0)
		{
			//开辟可执行栈节点内存
			SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
			if(pCurrNodeData == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				return -1;
			}
			memset(pCurrNodeData, 0, sizeof(SMacroExecutable));
			pCurrNodeData->unEndLineNum == UNASSIGNED_VALUE;

			//当前dWHILE为真
			if (!isZero(pSScan->dWHILE))
			{
				pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
				pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
				pCurrNodeData->ucType = MACRO_WHILE;
				pCurrNodeData->ucExecuteSign = MACRO_OK;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}
			}
			//当前dWHILE为假
			else
			{
				pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
				pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
				pCurrNodeData->ucType = MACRO_WHILE;
				pCurrNodeData->ucExecuteSign = MACRO_FORBID;
				//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
				if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
				{
				    freeGCode(pCurrNodeData);
				    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				    return -1;
				}
			}
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为IF，需要开辟栈节点，赋值后并压入
			if (pTopNodeData->ucType == MACRO_IF)
			{
				//开辟可执行栈节点内存
				SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
				if(pCurrNodeData == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return -1;
				}
				memset(pCurrNodeData, 0, sizeof(SMacroExecutable));
				pCurrNodeData->unEndLineNum == UNASSIGNED_VALUE;

				//栈顶IF单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dWHILE为真
					if (!isZero(pSScan->dWHILE))
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dWHILE为假
					else
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_FORBID;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
				//栈顶IF单元 不满足执行or禁止执行
				else
				{
					pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
					pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
					pCurrNodeData->ucType = MACRO_WHILE;
					pCurrNodeData->ucExecuteSign = MACRO_FORBID;
					//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
					if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
					{
					    freeGCode(pCurrNodeData);
					    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					    return -1;
					}
				}
			}

			//2.2.2栈顶单元为WHILE，且与当前WHILE为同一单元，则不需要开辟栈节点内存，仅修改栈顶单元
			else if (pTopNodeData->ucType == MACRO_WHILE && pTopNodeData->unLineNum == pSGetTextStateInfo->unCurrLineNum)
			{
				//当前dWHILE为真
				if (!isZero(pSScan->dWHILE))
				{
					pTopNodeData->ucExecuteSign = MACRO_OK;
				}
				//当前dWHILE为假
				else
				{
					pTopNodeData->ucExecuteSign = MACRO_FORBID;
					if (pTopNodeData->unEndLineNum != UNASSIGNED_VALUE)
					{
						pSGetTextStateInfo->unGetLineNum = pTopNodeData->unEndLineNum;
						pSGetTextStateInfo->ucOriStrIndex = 0;
						return 1;
					}
					//应该不存在
					else
					{
						parseGCodeErrorB(0, 0);
						return 1;
					}

				}

			}

			//2.2.3栈顶单元为WHILE，且与当前WHILE为不同单元，需要开辟栈节点，赋值后并压入
			else if (pTopNodeData->ucType == MACRO_WHILE && pTopNodeData->unLineNum != pSGetTextStateInfo->unCurrLineNum)
			{
				//开辟可执行栈节点内存
				SMacroExecutable* pCurrNodeData = mallocGCode(sizeof(SMacroExecutable), ORAM_NUM309);
				if(pCurrNodeData == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return -1;
				}
				memset(pCurrNodeData, 0, sizeof(SMacroExecutable));
				pCurrNodeData->unEndLineNum == UNASSIGNED_VALUE;

				//栈顶WHILE单元满足执行
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//当前dWHILE为真
					if (!isZero(pSScan->dWHILE))
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_OK;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
					//当前dWHILE为假
					else
					{
						pCurrNodeData->unLineNum = pSGetTextStateInfo->unCurrLineNum;
						pCurrNodeData->unEndLineNum = UNASSIGNED_VALUE;
						pCurrNodeData->ucType = MACRO_WHILE;
						pCurrNodeData->ucExecuteSign = MACRO_FORBID;
						//insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308);
						if (insertList(pCurrEnableStack, 0, pCurrNodeData, ORAM_NUM308) == FALSE)
						{
						    freeGCode(pCurrNodeData);
						    parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						    return -1;
						}
					}
				}
			


			}
		}

		//3.判断是否WHILE是否赋值完成
		SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
		if (pTopNodeData->ucExecuteSign == MACRO_OK)
		{
			return 1;
		}
		else
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 1;
			return 0;
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_ASSIGN, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
		}
        else if(cRet != 0)
        {
        	s_cIfAndWhielStepAssign[ucChannelNum] = 0;
            parseGCodeErrorB(0, 0);
            return -1;
		}
		return cRet;
		break;
	}
	default:
		break;
	}

}

int8 checkENDWHILE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepCheck[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_SYNTAX, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为WHILE
			if (pTopNodeData->ucType == MACRO_WHILE)
			{
				//2.2.1.1栈顶单元满足执行，则需要标记ENDWHILE行号 并 回到栈顶WHILE所在程序段
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//标记ENDWHILE行号
					pTopNodeData->unEndLineNum = pSGetTextStateInfo->unCurrLineNum;
					//回到栈顶WHILE所在程序段

					pSGetTextStateInfo->unGetLineNum = pTopNodeData->unLineNum;
					pSGetTextStateInfo->ucOriStrIndex = 0;
					return 1;

				}
				//2.2.1.1栈顶单元禁止执行，则需要继续获取
				else
				{
					removeNode(pCurrEnableStack, 0, FALSE);
					return 1;
				}
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_SYNTAX, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 0;
		}
		else if(cRet < 0)
		{
			s_cIfAndWhielStepCheck[ucChannelNum] = 0;
			parseGCodeErrorB(0, 0);
			return -1;
		}
		return cRet;
		break;

		break;
	}
	default:
		break;
	}
	return 1;
}
int8 assignENDWHILE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo)
{
	switch (s_cIfAndWhielStepAssign[ucChannelNum])
	{
	case 0:
	{
		//1.获取当前可执行栈
		SList* pCurrEnableStack = getCurrEnableStack(GC_ASSIGN, ucChannelNum);
		if (pCurrEnableStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.修改栈顶IF单元的可执行标志
		//2.1栈顶为空
		if (pCurrEnableStack->cSize == 0)
		{
			parseGCodeErrorB(0, 0);
			return -1;
		}
		//2.2栈顶不为空
		else
		{
			SMacroExecutable* pTopNodeData = FindNodeData(pCurrEnableStack, 0);
			//2.2.1栈顶单元为WHILE
			if (pTopNodeData->ucType == MACRO_WHILE)
			{
				//2.2.1.1栈顶单元满足执行，则需要标记ENDWHILE行号 并 回到栈顶WHILE所在程序段
				if (pTopNodeData->ucExecuteSign == MACRO_OK)
				{
					//标记ENDWHILE行号
					pTopNodeData->unEndLineNum = pSGetTextStateInfo->unCurrLineNum;
					//回到栈顶WHILE所在程序段

					pSGetTextStateInfo->unGetLineNum = pTopNodeData->unLineNum;
					pSGetTextStateInfo->ucOriStrIndex = 0;
					return 1;

				}
				//2.2.1.1栈顶单元禁止执行，则需要继续获取
				else
				{
					removeNode(pCurrEnableStack, 0, FALSE);
					return 1;
				}
			}
			//2.2.2栈顶不为IF
			else
			{
				parseGCodeErrorB(0, 0);
				return -1;
			}
		}
		break;
	}
	case 1:
	{
		//获取当前解析的程序文本
		SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
		if (pCurrProgram == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		int8 cRet = 0;
		cRet = findIfAndWhileSegment(GC_ASSIGN, ucChannelNum, pCurrProgram, pSGetTextStateInfo);
		if (cRet == 1)
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
		}
		else if(cRet < 0)
		{
			s_cIfAndWhielStepAssign[ucChannelNum] = 0;
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		return cRet;
		break;

		break;
	}
	default:
		break;
	}
	return 1;

}




