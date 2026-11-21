#ifndef __GCODE_G31_H
#define __GCODE_G31_H

void addG31RegulationAPI();

int8 CheckG31(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG31(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
