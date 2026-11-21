
#include "common.h"

#include "GCode.h"


//添加G17规则结构体
void addG17RegulationAPI()
{
	//指令：G17
	//组别：sGc
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(17, sGc, 1, NULL, NULL, NULL, CheckG17, AssignG17);
}
//添加G18规则结构体
void addG18RegulationAPI()
{
	//指令：G18
	//组别：sGc
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(18, sGc, 1, NULL, NULL, NULL, CheckG18, AssignG18);
}
//添加G19规则结构体
void addG19RegulationAPI()
{
	//指令：G19
	//组别：sGc
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(19, sGc, 1, NULL, NULL, NULL, CheckG19, AssignG19);
}

//检查G17
int8 CheckG17(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pGCodeBufUnit->SModalInfo.sGc = G17 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}
//检查G18
int8 CheckG18(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pGCodeBufUnit->SModalInfo.sGc = G18 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}
//检查G19
int8 CheckG19(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pGCodeBufUnit->SModalInfo.sGc = G19 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}


//赋值G17
int8 AssignG17(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pGCodeBufUnit->SModalInfo.sGc = G17 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 1;
}
//赋值G18
int8 AssignG18(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pGCodeBufUnit->SModalInfo.sGc = G18 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 1;
}
//赋值G19
int8 AssignG19(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pGCodeBufUnit->SModalInfo.sGc = G19 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 1;
}
