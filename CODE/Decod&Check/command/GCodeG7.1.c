#include "common.h"

#include "GCode.h"



int8 CheckG7_1(SSyntaxConfiguration* pSSyntaxConfig);

int8 AssignG7_1(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);


//添加G15规则结构体
void addG7_1RegulationAPI()
{
	//指令：G7.1
	//组别：sG
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(7.1, sGz, 1, NULL, NULL, NULL, CheckG7_1, AssignG7_1);
}



//检查G7.1
int8 CheckG7_1(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1. C=0，则关闭圆柱插补
	if (isZero(pSSyntaxConfig->SScanContainer.dABC[2]))
	{
		pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState = 0;//0：普通插补
	}
	//2. C!=0，则开启圆柱插补
	else
	{
		pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState = 2;//0：圆柱插补
	}

	return 2;
}

//赋值G7.1
int8 AssignG7_1(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	//1. C=0，则关闭圆柱插补
	if (isZero(pSAssignConfig->SScanContainer.dABC[2]))
	{
		pSAssignConfig->SGCodeCurrSysState.cInterpolationState = 0;//0：普通插补
		pSAssignConfig->SGCodeCurrSysState.fCylinderRadius = pSAssignConfig->SScanContainer.dABC[2];
		
		pSGCodeParseUnit->cInterpolationState = 0;
		pSGCodeParseUnit->fCylinderRadius = pSAssignConfig->SScanContainer.dABC[2];


	}
	//2. C!=0，则开启圆柱插补
	else
	{
		pSAssignConfig->SGCodeCurrSysState.cInterpolationState = 2;//2：圆柱插补
		pSAssignConfig->SGCodeCurrSysState.fCylinderRadius = pSAssignConfig->SScanContainer.dABC[2];

		pSGCodeParseUnit->cInterpolationState = 2;
		pSGCodeParseUnit->fCylinderRadius = pSAssignConfig->SScanContainer.dABC[2];
	}
	
	return 2;
}