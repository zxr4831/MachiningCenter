#pragma once
#ifndef __GCODE_G66_H
#define __GCODE_G66_H

//G66信息结构体
typedef struct _SG66InfoStruct
{
	int8 cFileName[TEMP_FILENAME_MAX_LENGTH];   //程序名
	uint16 sCalls;                          //调用次数
	struct _SLocalMacroVar SLocalVar;      //局部宏变量结构体 
}SG66InfoStruct;

void addG66RegulationAPI();

int8 CheckG66(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 AssignG66(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

bool isTriggerG66Command(int16 sCurrCommandVal);

int8 triggerG66ByAssign(SAssignConfiguration* pSAssignConfig, SG66InfoStruct* pSG66Info);
int8 triggerG66ByCheck(SSyntaxConfiguration* pSSyntaxConfig);


//添加G67规则结构体
void addG67RegulationAPI();
//检查G67
int8 CheckG67(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
//赋值G67
int8 AssignG67(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);



#endif