#include "common.h"
#include "GCode.h"


//*******************************M03***************************************
//添加M03规则
void addM03RegulationAPI()
{
	addMCommadRegulation(3, IO_PRE_ACTION, cMa, 1, NULL, NULL, NULL, NULL, AssignM03);

}

//赋值M03
int8 AssignM03(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(03, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	pSAssignConfig->cSpindleStatus = 1;	//1:正转 0:停止 -1:反转
	pSAssignConfig->cSpindleSpeedAssignFlag = 0;//0:S值赋给主轴 1:S值赋给第二主轴

	return 1;
}



//*******************************M04***************************************
//添加M04规则
void addM04RegulationAPI()
{
	addMCommadRegulation(4, IO_PRE_ACTION, cMa, 1, NULL, NULL, NULL, NULL, AssignM04);
}

//赋值M04
int8 AssignM04(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(4, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	pSAssignConfig->cSpindleStatus = -1;	//1:正转 0:停止 -1:反转
	pSAssignConfig->cSpindleSpeedAssignFlag = 0;//0:S值赋给主轴 1:S值赋给第二主轴

	return 1;
}




//*******************************M05***************************************
//添加M05规则
void addM05RegulationAPI()
{
	addMCommadRegulation(5, IO_POST_ACTION, cMa, 1, NULL, NULL, NULL, NULL, AssignM05);
}

//赋值M05
int8 AssignM05(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}
	checkParaRange('Q', &pSAssignConfig->SScanContainer, 0, 360);

	setIO(5, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

	if (pSAssignConfig->SScanContainer.bQ == TRUE)
	{
		pGCodeBufUnit->bValidQ = TRUE;
		pGCodeBufUnit->dQ = pSAssignConfig->SScanContainer.dQ;
	}
	else
	{
		//系统参数要求准停
		if (g_SSystemPara.ucSpindleStopMode == 0)//0:准停
		{
			pGCodeBufUnit->bValidQ = TRUE;
			pGCodeBufUnit->dQ = (double)g_SSystemPara.fSpindleStopDegree;
		}
	}

	pSAssignConfig->cSpindleStatus = 0;	//1:正转 0:停止 -1:反转

	return 1;
}


