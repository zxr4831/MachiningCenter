#include "common.h"
#include "GCode.h"


//*******************************M50***************************************
//动力刀正转
//添加M50规则
void addM50RegulationAPI()
{
	addMCommadRegulation(50, IO_PRE_ACTION, cMh, 1, NULL, NULL, NULL, NULL, AssignM50);
}

//赋值M50
int8 AssignM50(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(50, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	pSAssignConfig->cSSecondpindleStatus = 1;	//1:正转 0:停止 -1:反转
	pSAssignConfig->cSpindleSpeedAssignFlag = 1;//0:S值赋给主轴 1:S值赋给第二主轴

	return 1;
}



///*******************************M51***************************************
//动力刀反转
//添加M51规则
void addM51RegulationAPI()
{
	addMCommadRegulation(51, IO_PRE_ACTION, cMh, 1, NULL, NULL, NULL, NULL, AssignM51);
}

//赋值M51
int8 AssignM51(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(51, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	pSAssignConfig->cSSecondpindleStatus = -1;	//1:正转 0:停止 -1:反转
	pSAssignConfig->cSpindleSpeedAssignFlag = 1;//0:S值赋给主轴 1:S值赋给第二主轴

	return 1;
}




//*******************************M52***************************************
//添加M52规则
void addM52RegulationAPI()
{
	addMCommadRegulation(52, IO_POST_ACTION, cMh, 1, NULL, NULL, NULL, NULL, AssignM52);
}

//赋值M52
int8 AssignM52(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(52, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	pSAssignConfig->cSSecondpindleStatus = 0;	//1:正转 0:停止 -1:反转

	return 1;
}
