#ifndef GCODE_Me_H
#define GCODE_Me_H

//************************M32**************************************
void addM32RegulationAPI();

//赋值M32
int8 AssignM32(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M33**************************************
void addM33RegulationAPI();

//赋值M32
int8 AssignM33(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
