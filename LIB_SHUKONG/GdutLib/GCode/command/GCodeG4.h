#ifndef __GCODE_G4_H
#define __GCODE_G4_H


void addG4RegulationAPI();
int8 CheckG4(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG4(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);


void addG4_9RegulationAPI();
int8 CheckG4_9(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG4_9(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

#endif
