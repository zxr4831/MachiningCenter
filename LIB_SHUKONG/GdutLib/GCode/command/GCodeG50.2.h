#ifndef _GCODE_G50_2_H
#define _GCODE_G50_2_H

#include "common.h"
#include "GCodeCommon.h"

//添加G50_2规则结构体
void addG50_2RegulationAPI();

//检查G50_2
int8 CheckG50_2(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

//赋值G50_2
int8 AssignG50_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
