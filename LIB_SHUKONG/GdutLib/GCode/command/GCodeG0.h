#ifndef __GCODE_G0_H
#define __GCODE_G0_H

void addG0RegulationAPI();

int8 CheckG0(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG0(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
