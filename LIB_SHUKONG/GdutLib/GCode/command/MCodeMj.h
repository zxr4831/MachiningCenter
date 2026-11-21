#ifndef   GCODE_Mj_H
#define   GCODE_Mj_H


//****************************************M20***********************************
//添加M20规则
void addM20RegulationAPI();

//赋值M20
int8 AssignM20(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//***************************************M21**************************************

//添加M21规则
void addM21RegulationAPI();

//赋值M21
int8 AssignM21(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//***************************************M22**************************************

//添加M22
void addM22RegulationAPI();

//赋值M22
int8 AssignM22(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//***************************************M35**************************************
//添加M35
void addM35RegulationAPI();

//赋值M35
int8 AssignM35(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//***************************************M36**************************************
//添加M36
void addM36RegulationAPI();

//赋值M36
int8 AssignM36(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif // !  GCODE_M20_H
