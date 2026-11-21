#ifndef GCODE_Mi_H
#define GCODE_Mi_H

//************************M38**************************************
void addM38RegulationAPI();

//赋值M38
int8 AssignM38(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M39**************************************
void addM39RegulationAPI();

//赋值M32
int8 AssignM39(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
