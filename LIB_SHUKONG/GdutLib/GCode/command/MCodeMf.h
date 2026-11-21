#ifndef   GCODE_Mf_H
#define   GCODE_Mf_H

//************************M10**************************************
void addM10RegulationAPI();

//赋值M10
int8 AssignM10(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//************************M11**************************************
void addM11RegulationAPI();

//赋值M11
int8 AssignM11(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M12**************************************
void addM12RegulationAPI();

//赋值M12
int8 AssignM12(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//************************M13**************************************
void addM13RegulationAPI();

//赋值M13
int8 AssignM13(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);




#endif
