#include "common.h"
#include "GCode.h"
#include"ErrorFunction.h"
#define M32M33ENABLE 0

//*******************************M32***************************************
// 润滑油开
//添加M32规则
void addM32RegulationAPI()
{
#if M32M33ENABLE == 1
	addMCommadRegulation(32, IO_PRE_ACTION, cMe, 1, NULL, NULL, NULL, NULL, AssignM32);
#endif
}

//赋值M32
int8 AssignM32(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	if(setIO(M32, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) != 1)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM460);//查找不到文件名
		return -1;
	}
	return 1;
}

//*******************************M33***************************************
// 润滑油关
//添加M33规则
void addM33RegulationAPI()
{
#if M32M33ENABLE == 1
	addMCommadRegulation(33, IO_PRE_ACTION, cMe, 1, NULL, NULL, NULL, NULL, AssignM33);
#endif
}

//赋值M33
int8 AssignM33(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	if(setIO(M33, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) != 1)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM460);//查找不到文件名
		return -1;
	}
	return 1;
}
