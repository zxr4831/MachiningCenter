#ifndef   GCODE_Mc_H
#define   GCODE_Mc_H

//************************M08**************************************
void addM08RegulationAPI();

//¸³ÖµM08
int8 AssignM08(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//************************M09**************************************
void addM09RegulationAPI();

//¸³ÖµM09
int8 AssignM09(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif