

#include "common.h"

#include "GCode.h"

//添加G15规则结构体
void addG15RegulationAPI()
{
	//指令：G15
	//组别：sGg
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(15, sGj, 1, NULL, NULL, NULL, CheckG15, AssignG15);
}
//添加G16规则结构体
void addG16RegulationAPI()
{
	//指令：G16
	//组别：sGg
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(16, sGj, 1, NULL, NULL, NULL, CheckG16, AssignG16);
}


/*************************************************
  函数名称：CheckG15
  函数功能：关闭检查阶段的极坐标编程模态。
  输入参数：pSSyntaxConfig 为有效检查配置；pGCodeBufUnit 为统一回调的记录参数。
  输出参数：将检查状态中的 bPolarCoordinateProgramming 设置为 FALSE。
  返 回 值：2，本指令检查完成。
  备    注：记录参数当前未使用，保留以匹配统一检查回调接口。
**************************************************/
int8 CheckG15(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pSSyntaxConfig->SGCodeCurrSysState.bPolarCoordinateProgramming = FALSE;
	return 2;
}
/*************************************************
  函数名称：CheckG16
  函数功能：开启检查阶段的极坐标编程模态。
  输入参数：pSSyntaxConfig 为有效检查配置；pGCodeBufUnit 为统一回调的记录参数。
  输出参数：将检查状态中的 bPolarCoordinateProgramming 设置为 TRUE。
  返 回 值：2，本指令检查完成。
  备    注：记录参数当前未使用，保留以匹配统一检查回调接口。
**************************************************/
int8 CheckG16(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	pSSyntaxConfig->SGCodeCurrSysState.bPolarCoordinateProgramming = TRUE;
	return 2;
}


//赋值G15
int8 AssignG15(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSAssignConfig->SGCodeCurrSysState.bPolarCoordinateProgramming = FALSE;
	return 2;
}
//赋值G16
int8 AssignG16(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	pSAssignConfig->SGCodeCurrSysState.bPolarCoordinateProgramming = TRUE;
	return 2;
}