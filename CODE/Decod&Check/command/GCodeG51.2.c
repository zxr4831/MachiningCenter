#include "GCode.h"
#include "toolAxis.h"

//添加G51_2规则结构体
void addG51_2RegulationAPI()
{
	//指令：G51_2
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(51.2, sGz, 1, NULL, NULL, NULL, CheckG51_2, AssignG51_2);
}

int8 CheckG51_2(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	pSSyntaxConfig->SGCodeCurrSysState.cToolAxisStatus = TOOLAXIS_FOLLOW_MODE;

	return 2;
}


//赋值G51.2
int8 AssignG51_2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	//写入标志位
	pSGCodeBufUnit->cToolAxisMod = TOOLAXIS_FOLLOW_MODE;

	//写入刀具轴目标速度
	pSGCodeBufUnit->dToolAxisFollowRatio = pSAssignConfig->SScanContainer.dQ / pSAssignConfig->SScanContainer.dP;

	//写入刀具轴目标相位角
	pSGCodeBufUnit->dToolAxisPhase = pSAssignConfig->SScanContainer.bRIJK[0] == TRUE ? pSAssignConfig->SScanContainer.dRIJK[0] : 0;

	pSAssignConfig->SGCodeCurrSysState.cToolAxisStatus = TOOLAXIS_FOLLOW_MODE;

	return 2;
}