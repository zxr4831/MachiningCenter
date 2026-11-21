#ifndef __GCODE_G65_H
#define __GCODE_G65_H

void addG65RegulationAPI();

int8 CheckG65(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 AssignG65(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

void handleTypeAMacro(uint8 ucSyntaxOrAssign, SScan* pSScan, uint8 ucChannelNum, uint32 unLineNum);

#endif
