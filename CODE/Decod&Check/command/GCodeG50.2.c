#include "GCode.h"

//添加G51_2规则结构体
void addG50_2RegulationAPI()
{
	//指令：G50_2
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(50.2, sGz, 1, NULL, NULL, NULL, CheckG50_2, AssignG50_2);
}

int8 CheckG50_2(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	return 2;


}


//赋值G51.2
int8 AssignG50_2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	//写入标志位
	pSGCodeBufUnit->cToolAxisMod = 2;

	return 2;
}