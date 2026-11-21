#ifndef __GCODE_G17G18G19_H
#define __GCODE_G17G18G19_H

void addG17RegulationAPI();
void addG18RegulationAPI();
void addG19RegulationAPI();

int8 CheckG17(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckG18(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckG19(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 AssignG17(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG18(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG19(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif