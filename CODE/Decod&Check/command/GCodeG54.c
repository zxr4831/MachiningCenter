#include "common.h"
#include "GCode.h"


//添加G54规则结构体
void addG54RegulationAPI()
{
	//指令：G54
	//组别：sGb
	//指令占程序段数：1
	//参数结构体：无
	addGCommadRegulation(54, sGb, 1, NULL, NULL, NULL, CheckG54, AssignG54);
}
//添加G55规则结构体
void addG55RegulationAPI()
{
	//指令：G2
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	addGCommadRegulation(55, sGb, 1, NULL, NULL, NULL, CheckG55, AssignG55);
}
//添加G56规则结构体
void addG56RegulationAPI()
{
	//指令：G2
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	addGCommadRegulation(56, sGb, 1, NULL, NULL, NULL, CheckG56, AssignG56);
}
//添加G57规则结构体
void addG57RegulationAPI()
{
	//指令：G2
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	addGCommadRegulation(57, sGb, 1, NULL, NULL, NULL, CheckG57, AssignG57);
}
//添加G58规则结构体
void addG58RegulationAPI()
{
	//指令：G2
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	addGCommadRegulation(58, sGb, 1, NULL, NULL, NULL, CheckG58, AssignG58);
}
//添加G59规则结构体
void addG59RegulationAPI()
{
	//指令：G2
	//组别：sGa
	//指令占程序段数：1
	//参数结构体：无
	addGCommadRegulation(59, sGb, 1, NULL, NULL, NULL, CheckG59, AssignG59);
}


//检查G54
int8 CheckG54(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入临时缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G54 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSSyntaxConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G54;
	
	//3.更新状态结构体中的工件偏置坐标
	pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSSyntaxConfig->ucChannelNum].SWorkPieceOffset[G54 - 54];
		
	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 1;
}
//检查G55
int8 CheckG55(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G59写入临时缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G55 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSSyntaxConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G55;

	//3.更新状态结构体中的工件偏置坐标
	pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSSyntaxConfig->ucChannelNum].SWorkPieceOffset[G55 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 1;
}
//检查G56
int8 CheckG56(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G59写入临时缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G56 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSSyntaxConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G56;

	//3.更新状态结构体中的工件偏置坐标
	pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSSyntaxConfig->ucChannelNum].SWorkPieceOffset[G56 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 1;
}
//检查G57
int8 CheckG57(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G59写入临时缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G57* pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSSyntaxConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G57;

	//3.更新状态结构体中的工件偏置坐标
	pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSSyntaxConfig->ucChannelNum].SWorkPieceOffset[G57 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 1;
}
//检查G58
int8 CheckG58(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G59写入临时缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G58* pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSSyntaxConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G58;

	//3.更新状态结构体中的工件偏置坐标
	pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSSyntaxConfig->ucChannelNum].SWorkPieceOffset[G58 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 1;
}
//检查G59
int8 CheckG59(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G59写入临时缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G59* pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSSyntaxConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G59;

	//3.更新状态结构体中的工件偏置坐标
	pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSSyntaxConfig->ucChannelNum].SWorkPieceOffset[G59 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 1;
}


//赋值G54
int8 AssignG54(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G54 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的G54~G59 工件坐标号
	pSAssignConfig->SGCodeCurrSysState.sWorkpieceCoordNum= G54;
	
	//3.更新状态结构体中的工件偏置坐标
	pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSAssignConfig->ucChannelNum].SWorkPieceOffset[G54 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 1;
}
//赋值G55
int8 AssignG55(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G55 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的/G54~G59 工件坐标号
	pSAssignConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G55;

	//3.更新状态结构体中的工件偏置坐标
	pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSAssignConfig->ucChannelNum].SWorkPieceOffset[G55 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 1;
}
//赋值G56
int8 AssignG56(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G56 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的/G54~G59 工件坐标号
	pSAssignConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G56;

	//3.更新状态结构体中的工件偏置坐标
	pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSAssignConfig->ucChannelNum].SWorkPieceOffset[G56 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 1;
}
//赋值G57
int8 AssignG57(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G57 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的/G54~G59 工件坐标号
	pSAssignConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G57;

	//3.更新状态结构体中的工件偏置坐标
	pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSAssignConfig->ucChannelNum].SWorkPieceOffset[G57 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 1;
}
//赋值G58
int8 AssignG58(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G58 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的/G54~G59 工件坐标号
	pSAssignConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G58;

	//3.更新状态结构体中的工件偏置坐标
	pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSAssignConfig->ucChannelNum].SWorkPieceOffset[G58 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 1;
}
//赋值G59
int8 AssignG59(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1.将G54写入缓冲区
	pSGCodeParseUnit->SModalInfo.sGb = G59 * pow(10, GCOMMAND_DECIMAL_PLACE);

	//2.更新状态结构体中的/G54~G59 工件坐标号
	pSAssignConfig->SGCodeCurrSysState.sWorkpieceCoordNum = G59;

	//3.更新状态结构体中的工件偏置坐标
	pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SWorkPieceOffset = g_SZeroPtOfCoordSys[pSAssignConfig->ucChannelNum].SWorkPieceOffset[G59 - 54];

	//4.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 1;
}