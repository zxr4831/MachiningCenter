#include "common.h"
#include "GCode.h"



//*******************************M38***************************************
// 主轴刹车锁紧
//添加M38规则
void addM38RegulationAPI()
{
	addMCommadRegulation(38, IO_POST_ACTION, cMi, 1, NULL, NULL, NULL, NULL, AssignM38);
	return 1;
}

//赋值M38
int8 AssignM38(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M39***************************************
// 主轴刹车松开
//添加M39规则
void addM39RegulationAPI()
{
	addMCommadRegulation(39, IO_PRE_ACTION, cMi, 1, NULL, NULL, NULL, NULL, AssignM39);
}

//赋值M39
int8 AssignM39(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}