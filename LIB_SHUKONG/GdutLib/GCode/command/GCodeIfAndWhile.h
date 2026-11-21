#ifndef __GCODEMACRO_H
#define __GCODEMACRO_H

#include "common.h"
//#define MACRO_IF 0        //宏B IF
//#define MACRO_WHILE 1     //宏B WHILE
//#define MACRO_OK 0     //满足执行
//#define MACRO_NOT_OK 1 //不满足执行
//#define MACRO_FORBID 2 //禁止执行

/**********************/
//结构体定义
/**********************/
//可执行结构体
typedef struct _SMacroExecutable
{
    uint8 ucType; //控制语句的类型  IF:0  WHILE:1

    uint8 ucExecuteSign; //控制语句执行标记   0:满足执行条件 1:不满足执行条件  2:禁止执行

    uint32 unLineNum; //该控制语句所在行行号		目前只有WHILE用到

    uint32 unEndLineNum; //该控制语句对应END行号   目前只有WHILE会用到

    uint8 ucMainNum; //控制语句的主编号    ///lxw未用到
                     //uint8 ucSubNum;         //控制语句的次编号///lxw未用到
} SMacroExecutable;

int8 checkGOTO(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 assignGOTO(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

int8 checkIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo);
int8 assignIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo);

int8 checkELSEIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo);
int8 assignELSEIF(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo);

int8 checkELSE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo);
int8 assignELSE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo);

int8 checkENDIF(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo);
int8 assignENDIF(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo);

int8 checkWHILE(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo);
int8 assignWHILE(uint8 ucChannelNum, SScan* pSScan, SGetTextStateInfo* pSGetTextStateInfo);

int8 checkENDWHILE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo);
int8 assignENDWHILE(uint8 ucChannelNum, SGetTextStateInfo* pSGetTextStateInfo);

#endif // !#ifndef __GCODEMACRO_H
