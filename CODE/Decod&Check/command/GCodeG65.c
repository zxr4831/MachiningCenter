
#include "GCode.h"
#include "GeneralFunctions.h"
#include "ErrorFunction.h"


static int8 validateProgramP(double dP, uint16 * pOutP, uint32 unLineNum);
//添加G65规则结构体
void addG65RegulationAPI()
{
	//指令：G65
	//组别：sGz
	//指令占程序段数：1
	addGCommadRegulation(65, sGz, 1, NULL, NULL, NULL, CheckG65, AssignG65);
}



int8 CheckG65(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	// 已有错误时，不再执行宏运算或推进检查流程。 
    if (isErrorHasBeenWritten())
    {
        return -1;
    }

	 // 1. G65 H**：A 类宏。 
	if (pSSyntaxConfig->SScanContainer.bHFirst == TRUE)
	{
		handleTypeAMacro(GC_SYNTAX,&pSSyntaxConfig->SScanContainer,pSSyntaxConfig->ucChannelNum,pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
		 //handleTypeAMacro() 返回 void，通过全局错误标志报告失败。
	  	if (isErrorHasBeenWritten())
        {
            return -1;
        }

		//等赋值阶段完成本条宏运算或跳转，再检查后续程序段
		pSSyntaxConfig->cTargetStatus = GCODE_WAIT;

		return 2;
	}

	//2.G65 P** L**（非模态宏程序调用）
	else if (pSSyntaxConfig->SScanContainer.bPFirst == TRUE)
	{
		uint16 usP = 0;
		if (validateProgramP(pSSyntaxConfig->SScanContainer.dP, &usP,
		                     pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum) < 0)
		{
		    return -1;
		}
		pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
		return 2;
	}
	 // ***关键兜底：G65 必须有 H 或 P，否则稳定报错并失败返回***
	    parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
	 return -1;
}


//赋值G65
int8 AssignG65(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (isErrorHasBeenWritten())
	{
		return -1;
	}
	
	//1.G65 H** P#i Q#j R#k（A类宏）
    //1.1  Q为#1000以下为系统宏
    //1.2  Q为#1000或以上为系统宏
	if (pSAssignConfig->SScanContainer.bHFirst == TRUE)
	{
        handleTypeAMacro(GC_ASSIGN, &pSAssignConfig->SScanContainer, pSAssignConfig->ucChannelNum, pSAssignConfig->SGetTextStateInfo.unCurrLineNum);
		
		//宏处理失败时，不进入外层的本段完成处理。
		if (isErrorHasBeenWritten())
		{
			return -1;
		}
		return 2;
     }

    //2.G65 P** L**（非模态宏程序调用）
    else if (pSAssignConfig->SScanContainer.bPFirst == TRUE)
    {
		//1.手册：取消当前程序的G66标记
		SProgram* pSCurrProgram = getCurrProgramStruct(pSAssignConfig->ucChannelNum); 
		pSCurrProgram->ucG66MarkAssign = FALSE;

		//2.局部宏变量
		SLocalMacroVar STempLocalVar;
		initLocalVar(&STempLocalVar);

		//3.重新扫描程序段
		if(getLocalMacroVarBySegment(&pSAssignConfig->SGetTextStateInfo, &pSAssignConfig->SScanContainer, &STempLocalVar) != 1)
		{
			parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM300);
			return -1;
		}
        //4.开辟程序节点内存
        SProgram* pSSubProgram = createAProgramStruct();
        if (pSSubProgram == NULL)
        {
            //报错
        	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM300);
            return -1;
        }

		//5.赋值子程序结构体
        //5.1写入子程序名
		convertProgramName(round(pSAssignConfig->SScanContainer.dP), pSSubProgram->cFileName);
        //5.2赋值子程序调用次数
		if (pSAssignConfig->SScanContainer.bL == TRUE)
		{
			double dL = pSAssignConfig->SScanContainer.dL;
			if (dL <= 0 || dL != (int32)dL || dL > INT16_MAX) {
				freeProgramData(pSSubProgram);
				freeGCode(pSSubProgram);
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
				return -1;
			}
			pSSubProgram->sRemainingCalls = isZero(pSAssignConfig->SScanContainer.dL) ? 1 : round(pSAssignConfig->SScanContainer.dL);
		}
		else
		{
			pSSubProgram->sRemainingCalls = 1; //默认次数为1
		}
        
		//5.3赋值返回父程序的行号
        pSSubProgram->unFatherLineNum = pSAssignConfig->SGetTextStateInfo.unGetLineNum;

		//5.4赋值局部变量
        pSSubProgram->SMacro.SLocalVarSyntax = STempLocalVar;
        pSSubProgram->SMacro.SLocalVarAssign = STempLocalVar;

        //6.将子程序节点插入链表
        if(insertList(getProgramList(pSAssignConfig->ucChannelNum), 0, pSSubProgram, ORAM_NUM302) == FALSE)
        {
	    	freeProgramData(pSSubProgram);
			freeGCode(pSSubProgram);
        	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM300);
        	return -1;
        }
		//7.修改文本状态结构体
		pSAssignConfig->SGetTextStateInfo.unGetLineNum = 0;
		pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
        return 2;
    }
    // ***关键兜底：无 H / 无 P 的 G65 必须稳定失败***
    parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
    return -1;
}



/*************************************************
  函数名称：handleTypeAMacro
  函数功能：语法阶段检查A类宏参数，赋值阶段执行宏运算或设置跳转。
  输入参数：ucSyntaxOrAssign为GC_SYNTAX或GC_ASSIGN，pSScan为当前段扫描结果。
            ucChannelNum为有效通道号，unLineNum用于报警定位。
  输出参数：语法阶段不改扫描结果和宏变量；赋值阶段沿用各指令的写入行为。
  返 回 值：无；调用方通过isErrorHasBeenWritten()判断失败。
  备    注：语法检查不验证运行时操作数值，也不查找目标程序段。
**************************************************/
void handleTypeAMacro(uint8 ucSyntaxOrAssign, SScan* pSScan, uint8 ucChannelNum, uint32 unLineNum)
{
	//1.G65 H** P#i Q#j R#k（A类宏）
	//1.1  Q为#1000以下为系统宏
	//1.2  Q为#1000或以上为系统宏

	float fLocalMacroVal = 0;//局部宏变量名的值（浮点型）
	float fResult;    //计算结果

	if (pSScan == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}
	if (ucChannelNum > CHANNEL_NUM_MAX - 1)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM301);
		return;
	}

	//语法阶段只检查指令及必需参数，不读取宏值、不运算，也不设置跳转。
	if (ucSyntaxOrAssign == GC_SYNTAX)
	{
		bool bMacroP = TRUE;
		bool bNeedQ = FALSE;
		bool bNeedR = FALSE;

		//先检查H的有效性，避免小数截断或非有限数转换后选错指令。
		if (!isfinite(pSScan->dH) || pSScan->dH < 1 || pSScan->dH > 86 || pSScan->dH != floor(pSScan->dH))
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM450);
			return;
		}

		switch ((int32)pSScan->dH)
		{
		case 1:case 21:case 22:    //P#i、Q
			bNeedQ = TRUE;
			break;
		case 2:case 3:case 4:case 5:
		case 11:case 12:case 13:
		case 23:case 26:case 27:
		case 31:case 32:case 33:case 34:    //P#i、Q、R
			bNeedQ = TRUE;
			bNeedR = TRUE;
			break;
		case 80:    //无条件转移只要求P
			bMacroP = FALSE;
			break;
		case 81:case 82:case 83:case 84:case 85:case 86:    //条件转移要求P、Q、R
			bMacroP = FALSE;
			bNeedQ = TRUE;
			bNeedR = TRUE;
			break;
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM450);
			return;
		}

		//空宏变量也代表参数已写出；其数值在执行阶段再检查。
		if ((bMacroP && pSScan->bPoundP == FALSE) ||
			(!bMacroP && pSScan->bP == FALSE && pSScan->bPoundP == FALSE) ||
			(bNeedQ && pSScan->bQ == FALSE && pSScan->bPoundQ == FALSE) ||
			(bNeedR && pSScan->bRIJK[0] == FALSE && pSScan->bPoundR == FALSE))
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM451);
			return;
		}

		//与setMacroVar一致，运算结果只能写入局部宏变量#1～#33。
		if (bMacroP && (pSScan->usPoundP < 1 || pSScan->usPoundP > 33))
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			return;
		}

		//除零、开方及跳转段号等数值检查沿用下面的执行分支。
		return;
	}

	//获取当前程序结构体
	SProgram* pCurrProgram = getCurrProgramStruct(ucChannelNum);
	if (pCurrProgram == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//语法阶段已返回，此处使用赋值阶段的局部宏。
	SLocalMacroVar* pSLocalMacroVar = &pCurrProgram->SMacro.SLocalVarAssign;

	switch ((int32)pSScan->dH)
	{
		//H01：G65赋值运算 P#i = Q#j
	case 1:
	{
		//P#i没有出现 或 Q没出现  （P后面必须是变量，Q是变量或常数）
		if (pSScan->bPoundP == FALSE || pSScan->bQ == FALSE)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM451);
			break;
		}
		//系统宏不能被赋值
		if (pSScan->usPoundP >= 1000)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			break;
		}
		//从Q中取值
		//Q需要解引用
		if (pSScan->bPoundQ == TRUE)
		{
			//getMacroVar(pSLocalMacroVar, pSScan->usPoundQ, &fLocalMacroVal, NULL, unLineNum);
		    uint8* pSystemMacroVal = NULL;   // ***新增：接收系统宏地址***
		    // ***不再传 NULL***
		    int8 cRet = getMacroVar(pSLocalMacroVar, pSScan->usPoundQ, &fLocalMacroVal, &pSystemMacroVal, unLineNum);
		    if (cRet < 0)
		    {
		        return; // 或 break; 取决于你这层的错误处理约定
		    }

		    // 如果Q解引用的是系统宏（#1000~#1037），值在 *pSystemMacroVal
		    if (pSScan->usPoundQ >= 1000)
		    {
		        if (pSystemMacroVal == NULL)
		        {
		            parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		            return;
		        }
		        fLocalMacroVal = (float)(*pSystemMacroVal);
		    }
		}
		//不需要解引用
		else
		{
			fLocalMacroVal = (float)pSScan->dQ;
		}
		//将Q值赋给P
		setMacroVar(pSLocalMacroVar, pSScan->usPoundP, &fLocalMacroVal, unLineNum);
		break;
	}

	//	P#i =  Q#j  运算符号  R#k
	case 2:case 3:case 4:case 5:  //加法 减法 乘法 除法
	case 11:case 12:case 13:// 或 与 异或
	case 23://取余
	case 26:case 27://#i=#i×#j÷#k  乘除运算    复合平方根
	case 31:case 32:case 33:case 34://正弦 余弦 正切 反正切
	{
		int32 nTemp_1;
		int32 nTemp_2;

		//pTemp = &fResult;    //pTemp指向计算结果

		if (pSScan->bPoundP == FALSE || pSScan->bQ == FALSE || pSScan->bRIJK[0] == FALSE)    //如果#P、Q、R没出现，报错
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM451);
			break;
		}

		//根据H执行不同的计算，结果存入局部变量fResult
		switch ((int32)pSScan->dH)
		{

		case 2:    //+
		{
			fResult = (float)(pSScan->dQ + pSScan->dRIJK[0]);
			break;
		}
		case 3:    //-
		{
			fResult = (float)(pSScan->dQ - pSScan->dRIJK[0]);
			break;
		}
		case 4:    //*
		{
			fResult = (float)(pSScan->dQ * pSScan->dRIJK[0]);
			break;
		}
		case 5:    //     / 除
		{
			if (isZero(pSScan->dRIJK[0]))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM132);   //除数不能为0
				return;
			}

			fResult = (float)(pSScan->dQ / pSScan->dRIJK[0]);
			break;
		}
		case 11:    // | 或
		{
			nTemp_1 = (int32)pSScan->dQ;
			nTemp_2 = (int32)pSScan->dRIJK[0];
			fResult = (float)(nTemp_1 | nTemp_2);
			break;
		}
		case 12:    // &
		{
			nTemp_1 = (int32)pSScan->dQ;
			nTemp_2 = (int32)pSScan->dRIJK[0];
			fResult = (float)(nTemp_1 & nTemp_2);
			break;
		}
		case 13:    // ^  异或
		{
			nTemp_1 = (int32)pSScan->dQ;
			nTemp_2 = (int32)pSScan->dRIJK[0];
			fResult = (float)(nTemp_1 ^ nTemp_2);
			break;
		}
		case 23:    // % 取余
		{
			if (isZero(pSScan->dRIJK[0]))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM132);   //除数不能为0
				return;
			}
			fResult = (float)(fmod(pSScan->dQ, pSScan->dRIJK[0]));
			break;
		}

		case 26:    //十进制乘除运算
		{
			if (isZero(pSScan->dRIJK[0]))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM132);   //除数不能为0
				return;
			}

			fResult = (float)(pSScan->dP * pSScan->dQ / pSScan->dRIJK[0]);
			break;
		}
		case 27:    //复合平方根
		{
			fResult = (float)(sqrt(pow(pSScan->dQ, 2) + pow(pSScan->dRIJK[0], 2)));
			break;
		}

		case 31:    //# i = # j×sin(# k)
		{
			fResult = (float)(pSScan->dQ * sin(ANGLE_TO_RAD(pSScan->dRIJK[0])));
			break;
		}
		case 32:    //# i = # j×cos(# k)
		{
			fResult = (float)(pSScan->dQ * cos(ANGLE_TO_RAD(pSScan->dRIJK[0])));
			break;
		}
		case 33:    //# i = # j×tan(# k)
		{
			fResult = (float)(pSScan->dQ * tan(ANGLE_TO_RAD(pSScan->dRIJK[0])));
			break;
		}
		case 34:    //# i = ATAN(# j / # k)
        {
            double dQ = pSScan->dQ;
            double dR = pSScan->dRIJK[0];

            /* 拒绝非有限输入及0/0，失败时不进入公共变量写回。 */
            if (!isfinite(dQ) || !isfinite(dR) || (dQ == 0.0 && dR == 0.0))
            {
                parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
                return;
            }

            /* 保留原有角度计算规则，不改为atan2。 */
            fResult = (float)(RAD_TO_ANGEL(atan(dQ / dR)) + 180 * TEMP_PARA_NO180);

            if (!isfinite(fResult))
            {
                parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
                return;
            }

            break;
        }
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM450);
			break;
		}
		//将计算结果存入P后面的#i
		setMacroVar(pSLocalMacroVar, pSScan->usPoundP, &fResult, unLineNum);
		break;
	}

	case 21: //开方   # i = sqrt(# j)
	{
		if (pSScan->dQ < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM452);
			return;
		}
		// 计算（dQ是double，sqrt返回double，再转float）
		double dTmp = sqrt(pSScan->dQ);

		// **有限值校验：防 NaN/Inf 写回**
		if (!isfinite(dTmp))
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM452);
			return;
		}

		fResult = (float)dTmp;
		setMacroVar(pSLocalMacroVar, pSScan->usPoundP, &fResult, unLineNum);
		break;
	}
	case 22: //取绝对值  #i=|#j|
	{
	    fResult = (float)fabs(pSScan->dQ);  // **关键修复点：abs -> fabs**
		setMacroVar(pSLocalMacroVar, pSScan->usPoundP, &fResult, unLineNum);
		break;
	}
	//case 24: //十进制变为二进制  # i = BIN(# j )
	//{

	//}
	//case 25: //二进制变为十进制  # i = BCD(# j )
	//{

	//}

	//转移
	//无条件转移
	case 80:  //H80 Pn；  无条件转移  跳转至程序段 n
	{
		//没有P，报错
		if (pSScan->bP == FALSE)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM451);
			break;
		}
		//P值小于0 或 位数超过5位 或 不为整数
		if (pSScan->dP < 0 || pSScan->dP>99999 || isZero(pSScan->dP - (int32)pSScan->dP) == FALSE)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM82);
			break;
		}

		
		pSScan->cMacroType = 1;
		pSScan->bGOTO = TRUE;
		//p#i中的#i值存储在dp中，由于dp类型为double，经过运算后,原本为整数可能变为小数，所以用round圆整（前提：dp与整数p的误差在小数点后6位）
		pSScan->dGOTO = round(pSScan->dP);
		break;
	}

	//条件转移

	case 81:case 82:case 83:case 84:case 85:case 86:    //G65 H81 Pn Q#j R#k； 条件转移   如果# j与 # k的关系满足条件，则跳转至程序段 n，否则顺序执行
	{
		
			//没有P,或没有Q,或没有R，报错
			if (pSScan->bP == FALSE || pSScan->bQ == FALSE || pSScan->bRIJK[0] == FALSE)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM451);
				break;
			}
			//P值小于0 或 位数超过5位 或 不为整数
			if (pSScan->dP < 0 || pSScan->dP>99999 || isZero(pSScan->dP - (int32)pSScan->dP) == FALSE)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM82);
				break;
			}
		

		//用 calculateMacro运算，如果用pSScan->dQ和pSScan->dR直接计算，则如果#j或#K的值为空，但在pSScan->dQ和pSScan->dR的值会记为0，则关于空值的判断不准确（0和空不一致）
		//void calculateMacro(uint8 ucLevel, uint8 ucType, struct _SLocalMacroVar* pSLocalMacroVar, SCalculateUnit* pLeftInputUnit, SCalculateUnit* pRightInputUnit,  uint32 unLineNum)
		uint8 ucLevel= MACRO_LEVEL_6;    //等级
		uint8 ucType;    //类型
		SCalculateUnit SLeftInputUnit = { 0 };    //左输入单元 Q#j
		SCalculateUnit SRightInputUnit = { 0 };    //右输入单元  R#k
		//不同的条件转移
		switch ((int32)pSScan->dH)
		{
		case 81://如果# j = # k，则跳转至程序段 n，否则顺序执行
		{
			ucType = MACRO_EQ;
			ucLevel = MACRO_LEVEL_5;
			break;
		}
		case 82://如果# j ≠# k，则跳转至程序段 n，否则顺序执行
		{
			ucType = MACRO_NE;
			ucLevel = MACRO_LEVEL_5;
			break;
		}
		case 83://如果# j > # k，则跳转至程序段 n，否则顺序执行
		{
			ucType = MACRO_GT;
			ucLevel = MACRO_LEVEL_6;
			break;
		}
		case 84://如果# j < # k，则跳转至程序段 n，否则顺序执行
		{
			ucType = MACRO_LT;
			ucLevel = MACRO_LEVEL_6;
			break;
		}
		case 85://果# j ≥# k，则跳转至程序段 n，否则顺序执行
		{
			ucType = MACRO_GE;
			ucLevel = MACRO_LEVEL_6;
			break;
		}
		case 86://如果# j ≤# k，则跳转至程序段 n，否则顺序执行
		{
			ucType = MACRO_LE;
			ucLevel = MACRO_LEVEL_6;
			break;
		}
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM450);
			break;
		}
		if (pSScan->bPoundQ == TRUE)    //带#
		{
			SLeftInputUnit.bPound = TRUE;
			SLeftInputUnit.fVal = pSScan->usPoundQ;
		}
		else    //不带#
		{
			SLeftInputUnit.fVal = (float)(pSScan->dQ);
		}

		if (pSScan->bPoundR == TRUE)
		{
			SRightInputUnit.bPound = TRUE;
			SRightInputUnit.fVal = pSScan->usPoundR;
		}
		else
		{
			SRightInputUnit.fVal = (float)(pSScan->dRIJK[0]);
		}
		//比较，结果保留到左输入单元的fVal中
		calculateMacro(ucLevel, ucType, pSLocalMacroVar, &SLeftInputUnit, &SRightInputUnit, unLineNum);
		if (SLeftInputUnit.fVal > 0)    //满足执行
		{
			pSScan->cMacroType = 1;
			pSScan->bGOTO = TRUE;
			//p#i中的#i值存储在dp中，由于dp类型为double，经过运算后,原本为整数可能变为小数，所以用round圆整（前提：dp与整数p的误差在小数点后6位）
			pSScan->dGOTO = round(pSScan->dP);
		}
		break;
	}
	}


}



static int8 validateProgramP(double dP, uint16 * pOutP, uint32 unLineNum)
{
    if (pOutP == NULL)
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
        return -1;
    }

    // **非负**
    if (dP < 0.0)
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // **整数性**
    if (dP != floor(dP))
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // **范围 0~9999**
    if (dP > 9999.0)
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    *pOutP = (uint16)dP;
    return 1;
}
