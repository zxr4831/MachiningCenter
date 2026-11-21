
#include "common.h"
#include "GCode.h"


//英寸(G20)	毫米(G21)
//1.0 inch	25.4 mm

//添加G20规则结构体
void addG20RegulationAPI()
{
	//指令：G20
	//组别：sGe
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(20, sGe, 1, NULL, NULL, NULL, CheckG20, AssignG20);
}
//添加G21规则结构体
void addG21RegulationAPI()
{
	//指令：G21
	//组别：sGe
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(21, sGe, 1, NULL, NULL, NULL, CheckG21, AssignG21);
}


//检查G20
int8 CheckG20(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGe = G20 * pow(10, GCOMMAND_DECIMAL_PLACE);
	pSSyntaxConfig->SGCodeCurrSysState.bImperialProgramming = TRUE;
	return 2;
}
//检查G21
int8 CheckG21(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGe = G21 * pow(10, GCOMMAND_DECIMAL_PLACE);
	pSSyntaxConfig->SGCodeCurrSysState.bImperialProgramming = FALSE;
	return 2;
}

//赋值G20
int8 AssignG20(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGe = G20 * pow(10, GCOMMAND_DECIMAL_PLACE); 
	pSAssignConfig->SGCodeCurrSysState.bImperialProgramming = TRUE;
	return 2;
}
//赋值G21
int8 AssignG21(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSGCodeParseUnit->SModalInfo.sGe = G21 * pow(10, GCOMMAND_DECIMAL_PLACE);
	pSAssignConfig->SGCodeCurrSysState.bImperialProgramming = FALSE;
	return 2;
}