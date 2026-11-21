
#include "common.h"
#include "ringbuff.h"


#include "GCode.h"

//添加G98规则结构体
void addG98RegulationAPI()
{
	//指令：G98
	//组别：sGg
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(98, sGg, 1, NULL, NULL, NULL, CheckG98, AssignG98);
}
//添加G99规则结构体
void addG99RegulationAPI()
{
	//指令：G99
	//组别：sGg
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(99, sGg, 1, NULL, NULL, NULL, CheckG99, AssignG99);
}


//检查G98
int8 CheckG98(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	return 2;
}
//检查G99
int8 CheckG99(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	return 2;
}


//赋值G98
int8 AssignG98(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGg = G98 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}
//赋值G99
int8 AssignG99(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGg = G99 * pow(10, GCOMMAND_DECIMAL_PLACE);
	return 2;
}