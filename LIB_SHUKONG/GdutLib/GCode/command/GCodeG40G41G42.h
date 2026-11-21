#ifndef __GCODE_G40G41G42_H
#define __GCODE_G40G41G42_H

void addG40RegulationAPI();
void addG41RegulationAPI();
void addG42RegulationAPI();

int8 CheckG40(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckG41(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckG42(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 AssignG40(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG41(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG42(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
