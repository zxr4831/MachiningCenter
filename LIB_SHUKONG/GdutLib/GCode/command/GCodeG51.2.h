#ifndef _GCODE_G51_2_H
#define _GCODE_G51_2_H

#include "common.h"
#include "GCodeCommon.h"


//添加G83规则结构体
void addG51_2RegulationAPI();

//检查G51_2
int8 CheckG51_2(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

//赋值G51_2
int8 AssignG51_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);







#endif