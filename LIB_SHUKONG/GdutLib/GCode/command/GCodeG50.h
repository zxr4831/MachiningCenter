#ifndef __GCODE_G50_H
#define __GCODE_G50_H

void addG50RegulationAPI();

int8 CheckG50(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBufUnit);

int8 AssignG50(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);


int8 calculateG50Offset(uint8 ucMod, uint8 ucChannelNum, SPointDouble* pSCurrMacPoint, SPointDouble* pSCurrG50Offset, SPointDouble* pSCurrTotalOffset, SPointDouble* pSNewG50Offset, SScan* pSScan, bool bImperialProgramming);

#endif