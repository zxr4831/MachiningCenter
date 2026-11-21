#ifndef __GCODE_G10_H
#define __GCODE_G10_H

void addG10RegulationAPI();

int8 CheckG10(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBufUnit);

int8 AssignG10(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBufUnit);

#endif
