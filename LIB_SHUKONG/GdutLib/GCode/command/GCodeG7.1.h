#ifndef __GCODE_G7_1_H
#define __GCODE_G7_1_H

void addG7_1RegulationAPI();


//ºÏ≤ÈG15
int8 CheckG7_1(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

//∏≥÷µG16
int8 AssignG7_1(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);



#endif
