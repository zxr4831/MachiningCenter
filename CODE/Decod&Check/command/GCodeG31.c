
#include "ringbuff.h"
#include "common.h"
#include "GCode.h"
#include "toolAxis.h"
#include "ErrorFunction.h"


//添加G1规则结构体
void addG31RegulationAPI()
{
	//指令：G31
	//组别：sGa
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(31, sGz, 1, NULL, NULL, NULL, CheckG31, AssignG31);
}

//检查G31
int8 CheckG31(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		//报错
		return -1;
	}

	//3.记录
	//1.填写SLineInfo
	SLine SLineInfo;//存直线段信息（可能要将此变量放在赋值结构体中）
	memset(&SLineInfo, 0, sizeof(SLine));
	//1.1初始化SLine的起点终点（机械坐标）
	SLineInfo.SEndPoint = SLineInfo.SStartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
	//1.2获取SLine的终点（机械坐标）
	//计算坐标
	getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &SLineInfo.SStartPoint, &SLineInfo.SEndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
	//1.3获取实体类型
	SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : L1NE;

//2.将SLine写入缓冲区
	writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);

	return 2;
}

//赋值G31
int8 AssignG31(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	//1.填写SLineInfo
	SLine SLineInfo;//存直线段信息（可能要将此变量放在赋值结构体中）
	memset(&SLineInfo, 0, sizeof(SLine));
	//1.1初始化SLine的起点终点（机械坐标）
	SLineInfo.SEndPoint = SLineInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
	//1.2获取SLine的终点（机械坐标）
	//计算坐标
	getMacCoordByScanInfo(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &SLineInfo.SStartPoint, &SLineInfo.SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
	//1.3获取实体类型
	SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : L1NE;

	//2.将SLine写入缓冲区
	writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);

	pGCodeBufUnit->bG31Flag = TRUE;

	return 2;
}
