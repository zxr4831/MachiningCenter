#ifndef __GCODE_G1_H
#define __GCODE_G1_H


void addG1RegulationAPI();

int8 CheckG1(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG1(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);



#endif
