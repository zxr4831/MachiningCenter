#include "common.h"
#include "GCode.h"



//*******************************M14***************************************
// 主轴使能
//添加M14规则
void addM14RegulationAPI();


//赋值M14
int8 AssignM14(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//*******************************M15***************************************
// 第一主轴工件松开
//添加M15规则
void addM15RegulationAPI();


//赋值M15
int8 AssignM15(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

