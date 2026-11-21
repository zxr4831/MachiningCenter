#ifndef __GCODE_G20G21_H
#define __GCODE_G20G21_H

void addG20RegulationAPI();
void addG21RegulationAPI();

int8 CheckG20(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 CheckG21(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 AssignG20(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG21(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
