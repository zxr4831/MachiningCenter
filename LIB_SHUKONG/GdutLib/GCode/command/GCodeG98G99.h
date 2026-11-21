#ifndef __GCODE_G98G99_H
#define __GCODE_G98G99_H

void addG98RegulationAPI();
void addG99RegulationAPI();

int8 CheckG98(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckG99(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 AssignG98(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG99(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


#endif
