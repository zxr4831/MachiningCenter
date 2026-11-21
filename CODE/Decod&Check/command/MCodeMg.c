#include "common.h"
#include "GCode.h"



//*******************************M14***************************************
// 主轴使能
//添加M14规则
void addM14RegulationAPI()
{
	addMCommadRegulation(14, IO_PRE_ACTION, cMg, 1, NULL, NULL, NULL, NULL, AssignM14);
}

//赋值M14
int8 AssignM14(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M14, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;

}

//*******************************M15***************************************
// 取消主轴使能
//添加M15规则
void addM15RegulationAPI()
{
	addMCommadRegulation(15, IO_POST_ACTION, cMg, 1, NULL, NULL, NULL, NULL, AssignM15);
}

//赋值M15
int8 AssignM15(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M15, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}