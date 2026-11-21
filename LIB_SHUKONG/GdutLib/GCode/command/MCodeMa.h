#ifndef GCODE_Ma_H
#define GCODE_Ma_H

//************************M03**************************************
void addM03RegulationAPI();

//赋值M03
int8 AssignM03(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M04**************************************
void addM04RegulationAPI();

//赋值M05
int8 AssignM04(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//************************M05**************************************
void addM05RegulationAPI();

//赋值M05
int8 AssignM05(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif // !  GCODE_M05_H
