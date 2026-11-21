#ifndef __GCODE_G54_H
#define __GCODE_G54_H

void addG54RegulationAPI();
void addG55RegulationAPI();
void addG56RegulationAPI();
void addG57RegulationAPI();
void addG58RegulationAPI();
void addG59RegulationAPI();

int8 CheckG54(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 CheckG55(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 CheckG56(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 CheckG57(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 CheckG58(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 CheckG59(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 AssignG54(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG55(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG56(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG57(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG58(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG59(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

#endif
