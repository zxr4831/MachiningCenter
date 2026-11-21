#include "common.h"
#include "GCode.h"
#include"ErrorFunction.h"


//*******************************M10***************************************
// 第一主轴工件夹紧
//添加M10规则
void addM10RegulationAPI()
{
	addMCommadRegulation(10, IO_PRE_ACTION, cMf, 1, NULL, NULL, NULL, NULL, AssignM10);
}

//赋值M10
int8 AssignM10(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	if(setIO(M10, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) != 1)
	{
	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM460);
	return -1;
	}
	return 1;
}

//*******************************M11***************************************
// 第一主轴工件松开
//添加M11规则
void addM11RegulationAPI()
{
	addMCommadRegulation(11, IO_PRE_ACTION, cMf, 1, NULL, NULL, NULL, NULL, AssignM11);
}

//赋值M11
int8 AssignM11(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M11, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M12***************************************
// 第二主轴工件夹紧
//添加M12规则
void addM12RegulationAPI()
{
	addMCommadRegulation(12, IO_PRE_ACTION, cMf, 1, NULL, NULL, NULL, NULL, AssignM12);
}

//赋值M12
int8 AssignM12(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M12, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M13***************************************
// 第二主轴工件松开
//添加M13规则
void addM13RegulationAPI()
{
	addMCommadRegulation(13, IO_PRE_ACTION, cMf, 1, NULL, NULL, NULL, NULL, AssignM13);
}

//赋值M13
int8 AssignM13(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M13, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}





