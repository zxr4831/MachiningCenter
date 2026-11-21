
#include "common.h"
#include "GCode.h"
#include "ErrorFunction.h"



//添加G96规则结构体
void addG96RegulationAPI()
{
	//指令：G96
	//组别：sGf
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(96, sGf, 1, NULL, NULL, NULL, CheckG96, AssignG96);
}
//添加G97规则结构体
void addG97RegulationAPI()
{
	//指令：G97
	//组别：sGf
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(97, sGf, 1, NULL, NULL, NULL, CheckG97, AssignG97);
}


//检查G96
int8 CheckG96(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	 if (pSSyntaxConfig->SScanContainer.bP == TRUE ||
        pSSyntaxConfig->SScanContainer.bQ == TRUE ||
        pSSyntaxConfig->SScanContainer.bL == TRUE)
    {
        //外层不接收检查返回值，必须实际写入报警。
        parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM403);
        return -1;
    }
	return 2;
}
//检查G97
int8 CheckG97(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	 if (pSSyntaxConfig->SScanContainer.bP == TRUE ||
        pSSyntaxConfig->SScanContainer.bQ == TRUE ||
        pSSyntaxConfig->SScanContainer.bL == TRUE)
    {
        //外层不接收检查返回值，必须实际写入报警。
        parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM403);
        return -1;
    }
	return 2;
}


//赋值G96
int8 AssignG96(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGf = G96 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}
//赋值G97
int8 AssignG97(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGf = G97 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}