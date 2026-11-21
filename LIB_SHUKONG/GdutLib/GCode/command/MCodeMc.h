#ifndef GCODE_Mc_H
#define GCODE_Mc_H

//************************M08**************************************
void addM08RegulationAPI();

//赋值M08
int8 AssignM08(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M09**************************************
void addM09RegulationAPI();

//赋值M09
int8 AssignM09(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
