#ifndef GCODE_Mb_H
#define GCODE_Mb_H
//***********************************************M01****************************************
//添加M00规则
void addM00RegulationAPI();
//检查M00
int8 CheckM00(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit);
//赋值M00
int8 AssignM00(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
//***********************************************M01****************************************
//添加M01规则
void addM01RegulationAPI();
//检查M01
int8 CheckM01(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit);
//赋值M01
int8 AssignM01(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//***********************************************M02****************************************
//添加M02规则
void addM02RegulationAPI();

//赋值M02
int8 AssignM02(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckM02(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit);

//***********************************************M29****************************************
//添加M29规则
void addM29RegulationAPI();

//赋值M29
int8 AssignM29(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckM29(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
//***********************************************M30****************************************
//添加M30规则
void addM30RegulationAPI();

//赋值M30
int8 AssignM30(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 CheckM30(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

//****************************************M98************************
void addM98RegulationAPI();

int8 CheckM98(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 AssignM98(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

//****************************************M99************************
void addM99RegulationAPI();

int8 CheckM99(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 AssignM99(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
//****************************************M96************************
void addM96RegulationAPI();

int8 CheckM96(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit);

int8 AssignM96(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit);

//****************************************M97************************
void addM97RegulationAPI();

int8 CheckM97(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit);

int8 AssignM97(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit);

//****************************************M100-M199************************
void addM100_M199RegulationAPI();

int8 CheckM100_M199(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBuffUnit);

int8 AssignM100_M199(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBuffUnit);

#endif // !GCODE_M01_H
