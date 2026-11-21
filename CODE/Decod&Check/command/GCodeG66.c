
#include "GCode.h"
#include "ErrorFunction.h"


static int8 validateProgramP(double dP, uint16 * pOutP, uint32 unLineNum);

//添加G66规则结构体
void addG66RegulationAPI()
{
    //指令：G66
    //组别：sGz
    //指令占程序段数：1
    addGCommadRegulation(66, sGz, 1, NULL, NULL, NULL, CheckG66, AssignG66);
}

//检查G66
int8 CheckG66(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    SProgram* pSCurrProgram = getCurrProgramStruct(pSSyntaxConfig->ucChannelNum);
    if(pSCurrProgram == NULL)
    {
    	parseGCodeErrorB(0, GCODE_ERROR_NUM300);
    	 return -1;
    }
    if(pSSyntaxConfig->SScanContainer.bP != TRUE)
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
        return -1;
    }
    // **关键：P 合法性校验（非负整数 + 0~9999）**
    uint16 usP = 0;
    if (validateProgramP(pSSyntaxConfig->SScanContainer.dP, &usP,
                         pSSyntaxConfig->SGetTextStateInfo.unGetLineNum) < 0)
    {
        return -1;
    }
    pSCurrProgram->ucG66MarkCheck = TRUE;
    return 2;
}

//赋值G66
int8 AssignG66(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    //1.开辟G66信息结构体
    SG66InfoStruct* pSG66Info = mallocGCode(sizeof(SG66InfoStruct), ORAM_NUM417);
    if (pSG66Info == NULL)
    {
        //报错
        return -1;
    }
    memset(pSG66Info, 0, sizeof(SG66InfoStruct));
    initLocalVar(&pSG66Info->SLocalVar);

    //2.重新扫描程序段
    if(getLocalMacroVarBySegment(&pSAssignConfig->SGetTextStateInfo, &pSAssignConfig->SScanContainer, &pSG66Info->SLocalVar) != 1)
	{
    	freeGCode(pSG66Info);
		return -1;
	}
    if(pSAssignConfig->SScanContainer.bP != TRUE)
    {
    	freeGCode(pSG66Info);
        parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
        return -1;
    }

    //3.赋值G66信息结构体
    //3.1赋值程序名
    uint16 usP = 0;
    if (validateProgramP(pSAssignConfig->SScanContainer.dP, &usP,
                         pSAssignConfig->SGetTextStateInfo.unGetLineNum) < 0)
    {
        freeGCode(pSG66Info);
        return -1;
    }
    convertProgramName(round(pSAssignConfig->SScanContainer.dP), pSG66Info->cFileName);   //数值转字符串 获取文件名
    //3.2赋值子程序调用次数
    if(pSAssignConfig->SScanContainer.bL == TRUE)
    {
        double dL = pSAssignConfig->SScanContainer.dL;
        if (dL <= 0 || dL != (int32)dL || dL > 9999) {
    		freeGCode(pSG66Info);
            parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
            return -1;
        }
        pSG66Info->sCalls = dL;
    }
    else
    pSG66Info->sCalls = 1;



    //4.将G66信息结构体放入
    SChannelConfiguration* pSGCodeChannel = getChannelConfig(pSAssignConfig->ucChannelNum);
    int8 cCount = 0;
    for (cCount = 0; cCount < NESTING_LIMIT; cCount++)
    {
        if (pSGCodeChannel->pSG66InfoArr[cCount] == NULL)
        {
            pSGCodeChannel->pSG66InfoArr[cCount] = pSG66Info;
            SProgram* pSCurrProgram = getCurrProgramStruct(pSAssignConfig->ucChannelNum);
            pSCurrProgram->ucG66ArrIndex = cCount;
            pSCurrProgram->ucG66MarkAssign = TRUE;
            break;
        }
    }
    if(cCount>=NESTING_LIMIT)
    {
    	freeGCode(pSG66Info);
    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM93);
    	return -1;
    }

    return 2;
}

//G66触发指令
bool isTriggerG66Command(int16 sCurrCommandVal)
{
    if (sCurrCommandVal == G01 * pow(10, GCOMMAND_DECIMAL_PLACE) ||
        sCurrCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE) ||
        sCurrCommandVal == G03 * pow(10, GCOMMAND_DECIMAL_PLACE))
    {
        return TRUE;
    }
    return FALSE;
}

//赋值触发G66程序调用
int8 triggerG66ByAssign(SAssignConfiguration* pSAssignConfig, SG66InfoStruct* pSG66Info)
{
    //1.开辟程序节点内存，并赋值
    SProgram* pSSubProgram = createAProgramStruct();
    if (pSSubProgram == NULL)
    {
        //报错
        return -1;
    }

    //2.1赋值程序名
    strcpy(pSSubProgram->cFileName, pSG66Info->cFileName);

    //2.2赋值子程序调用次数
    pSSubProgram->sRemainingCalls = pSG66Info->sCalls;

    //2.3赋值返回父程序的行号
    pSSubProgram->unFatherLineNum = pSAssignConfig->SGetTextStateInfo.unGetLineNum;

    //2.4赋值局部变量
    pSSubProgram->SMacro.SLocalVarSyntax = pSG66Info->SLocalVar;
    pSSubProgram->SMacro.SLocalVarAssign = pSG66Info->SLocalVar;

    //3.将子程序节点插入链表
    if(insertList(getProgramList(pSAssignConfig->ucChannelNum), 0, pSSubProgram, ORAM_NUM302) == FALSE)
    {
    	freeProgramData(pSSubProgram);
		freeGCode(pSSubProgram);
    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM300);
    	return -1;
    }
    //4.赋值获取行号置0
    pSAssignConfig->SGetTextStateInfo.ucOriStrIndex = 0;
    pSAssignConfig->SGetTextStateInfo.unGetLineNum = 0;
    return 1;

}

//检查触发G66程序调用
int8 triggerG66ByCheck(SSyntaxConfiguration* pSSyntaxConfig)
{
    //1.检查获取行号置0
    pSSyntaxConfig->SGetTextStateInfo.unGetLineNum = 0;
    //3.检查状态置WAIT
    pSSyntaxConfig->cTargetStatus = GCODE_WAIT;
    return 1;
}




//添加G67规则结构体
void addG67RegulationAPI()
{
    //指令：G67
    //组别：sGz
    //指令占程序段数：1
    addGCommadRegulation(67, sGz, 1, NULL, NULL, NULL, CheckG67, AssignG67);
}

//检查G67
int8 CheckG67(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    SProgram* pSCurrProgram = getCurrProgramStruct(pSSyntaxConfig->ucChannelNum);
    pSCurrProgram->ucG66MarkCheck = FALSE;
    return 2;
}

//赋值G67
int8 AssignG67(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    SProgram* pSCurrProgram = getCurrProgramStruct(pSAssignConfig->ucChannelNum);
    //2.释放G66信息结构体
    SChannelConfiguration* pSGCodeChannel = getChannelConfig(pSAssignConfig->ucChannelNum);

    if(pSCurrProgram->ucG66MarkAssign == TRUE)
    {
    	SG66InfoStruct* pSG66Info = pSGCodeChannel->pSG66InfoArr[pSCurrProgram->ucG66ArrIndex];
    	if(pSG66Info != NULL)
    	freeGCode(pSGCodeChannel->pSG66InfoArr[pSCurrProgram->ucG66ArrIndex]);
    	pSGCodeChannel->pSG66InfoArr[pSCurrProgram->ucG66ArrIndex] = NULL;
    pSCurrProgram->ucG66MarkAssign = FALSE;
    	pSCurrProgram->ucG66ArrIndex= FALSE;
    	pSCurrProgram->ucG66MarkAssign= FALSE;
    	pSCurrProgram->ucG66MarkCheck = FALSE;

    }


    return 2;
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
