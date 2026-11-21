#ifndef __GCODEREGULATION_H
#define __GCODEREGULATION_H
#include<stdio.h>
#include <stdlib.h>
#include "common.h"

//各指令允许出现的共有参数
#define CommonPara "GMFSTRCN"//GM FST RC

/**********************/
//全局变量声明
/**********************/

extern Tree* g_pSGCommandRegulationTree;//G指令规则结构体树
extern Tree* g_pSMCommandRegulationTree;//M指令规则结构体树


typedef int8(*MallocParaStruct)(uint8, uint8);		//开辟参数结构体函数指针
typedef int8(*FreeParaStruct)(uint8, uint8);		//释放参数结构体函数指针
typedef int8(*GetPara)(uint8, struct _SScan*, uint8, struct _SGCodeSysState*);				//获取参数函数指针
//typedef uint8(*CheckCommand)(void*, void*);		//检查指令函数指针
//typedef uint8(*AssignCommand)(void*, void*);		//赋值指令函数指针


typedef int8(*CheckCommand)(struct _SSyntaxConfiguration*, struct SGCodeParse*);		//检查指令函数指针
typedef int8(*AssignCommand)(struct _SAssignConfiguration*, struct SGCodeParse*);		//赋值指令函数指针

/**********************/
//结构体定义
/**********************/

typedef struct _SGCommandRegulation
{
	int nCommandVal;			//指令值 nCommandVal = 实际值 * DEFINE
	int8 cGroup;				//所属组别
	int8 cSegmentNum;			//指令程序段数（仅cGa、cGz组G指令用到）
	//bool bHasParaStruct;//是否使用参数结构体  
	MallocParaStruct pMallocParaStructAPI;//开辟参数结构体API  
	FreeParaStruct pFreeParaStructAPI;//释放参数结构体API  
	GetPara pGetParaAPI;//获取参数API  
	CheckCommand pCheckCommandAPI;//检查指令API  
	AssignCommand pAssignCommandAPI;//赋值指令API
}SGCommandRegulation;

typedef struct _SMCommandRegulation
{
	int32 nCommandVal;			//指令值 nCommandVal = 实际值 * DEFINE
	int8 cType;//前后作用类型 0：前作用 1：后作用
	int32 nAllow;					//暂时未用到
	int8 cGroup;				//所属组别
	int8 cSegmentNum;			//指令程序段数（仅cGa、cGz组G指令用到）
	//bool bHasParaStruct;//是否使用参数结构体  
	MallocParaStruct pMallocParaStructAPI;//开辟参数结构体API  
	FreeParaStruct pFreeParaStructAPI;//释放参数结构体API  
	GetPara pGetParaAPI;//获取参数API  
	CheckCommand pCheckCommandAPI;//检查指令API  
	AssignCommand pAssignCommandAPI;//赋值指令API
}SMCommandRegulation;

/**********************/
//函数声明
/**********************/


//G指令规则结构体树中添加指令
int8 addGCommadRegulation(double dCommandVal, int8 cGroup, int8 cSegmentNum,
	MallocParaStruct pMallocParaStructAPI, FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI,
	CheckCommand pCheckCommandAPI, AssignCommand pAssignCommandAPI);

//M指令规则结构体树中添加指令
int8 addMCommadRegulation(double dCommandVal, int8 cType,int8 cGroup, int8 cSegmentNum,
	MallocParaStruct pMallocParaStructAPI, FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI,
	CheckCommand pCheckCommandAPI, AssignCommand pAssignCommandAPI);

//加载所有G指令的规则结构体(总接口，开机时调用)
void addAllGCommandRegulationAPI();

//加载所有M指令的规则结构体(总接口，开机时调用)
void addAllMCommandRegulationAPI();




#endif
