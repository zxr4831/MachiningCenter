#ifndef   GCODE_Mk_H
#define   GCODE_Mk_H


//****************************************M80***********************************
//添加M80规则
void addM80RegulationAPI();

//赋值M80
int8 AssignM80(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//****************************************M81***********************************
//添加M81规则
void addM81RegulationAPI();

//赋值M81
int8 AssignM81(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//****************************************M82***********************************
//添加M82规则
void addM82RegulationAPI();

//赋值M82
int8 AssignM82(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//****************************************M83***********************************
//添加M83规则
void addM83RegulationAPI();

//赋值M83
int8 AssignM83(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//****************************************M84***********************************
//添加M84规则
void addM84RegulationAPI();

//赋值M84
int8 AssignM84(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//****************************************M85***********************************
//添加M85规则
void addM85RegulationAPI();

//赋值M85
int8 AssignM85(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//****************************************M86***********************************
//添加M86规则
void addM86RegulationAPI();

//赋值M86
int8 AssignM86(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);


//****************************************M87***********************************
//添加M83规则
void addM87RegulationAPI();

//赋值M87
int8 AssignM87(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
