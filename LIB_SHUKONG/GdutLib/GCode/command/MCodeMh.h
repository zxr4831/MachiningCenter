#ifndef GCODE_Mh_H
#define GCODE_Mh_H

//************************M50**************************************
void addM50RegulationAPI();

//赋值M50
int8 AssignM50(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M51**************************************
void addM51RegulationAPI();

//赋值M51
int8 AssignM51(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M52**************************************
void addM52RegulationAPI();

//赋值M52
int8 AssignM52(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif //
