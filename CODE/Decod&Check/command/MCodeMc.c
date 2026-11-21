#include "common.h"
#include "GCode.h"
#include"ErrorFunction.h"

//*******************************M08***************************************
// 冷却液开
//添加M08规则
void addM08RegulationAPI()
{
	addMCommadRegulation(8, IO_PRE_ACTION, cMc, 1, NULL, NULL, NULL, NULL, AssignM08);
}

//赋值M08
int8 AssignM08(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M08, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M09***************************************
// 冷却液关
//添加M09规则
void addM09RegulationAPI()
{
	addMCommadRegulation(9, IO_PRE_ACTION, cMc, 1, NULL, NULL, NULL, NULL, AssignM09);
}

//赋值M09
int8 AssignM09(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	if (setIO(M09, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) != 1)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM460);
		return -1;
	}
	return 1;
}
