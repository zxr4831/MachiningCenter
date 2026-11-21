#ifndef __GCODEPROGRAM_H
#define __GCODEPROGRAM_H

#include "common.h"
#include "list.h"

//lxw:头文件包含关系
#include "GCodeCommon.h"
#include "GCodeCalcExpression.h"

/**********************/
//结构体定义
/**********************/

//宏结构体
typedef struct _SMacro
{
    SList* pDefineList; //存放自定义宏的链表 数据:SUserDefineData

    struct _SLocalMacroVar SLocalVarSyntax; //局部宏变量结构体  语法
    struct _SLocalMacroVar SLocalVarAssign; //局部宏变量结构体  赋值

    SList* pEnableStackSyntax; //存放可执行结构体的栈  语法 数据:SMacroExecutable结构体
    SList* pEnableStackAssign; //存放可执行结构体的栈  赋值 数据:SMacroExecutable结构体

    uint8 ucIfCntSyntax; //栈中if语句的个数
    uint8 ucIfCntAssign;

    uint8 ucWhileCntSyntax; //栈中while语句的个数
    uint8 ucWhileCntAssign;

} SMacro;

//程序结构体
typedef struct _SProgram
{
    int8 cFileName[TEMP_FILENAME_MAX_LENGTH]; //程序名
    int16 sRemainingCalls;                    //剩余调用次数   权限方面:语法检查只读，赋值读写
    uint32 unFatherLineNum;                   //调用子程序时，父程序M98的下一行的行号

    //宏相关
    int8
        cMacroStatus; //标志位，当前程序是否启用了宏  0(GC_MODE_NORMAL):没启用宏   1(GC_MODE_NORMAL_2_MACRO_1):获取自定义宏阶段1  2(GC_MODE_NORMAL_2_MACRO_2):获取自定义宏阶段2  3(GC_MODE_MACRO):宏运行
    uint16 usBeginLineNumMarco; //最后一个宏定义的下一行的行号,由语法检查写入（功能与sBeginLineNum相同）
    int16 sBeginLineNum;        //程序开始行号（正文的第一行，由检查流程写入）
    int32 nEndLineNum;          //程序结束

    uint8
        ucG66ArrIndex; //G66信息结构体的在数组pSG66InfoArr中的索引加1,使用时要-1   0:表示没使用G66   1-4:分别对应数组索引0-3
    SMacro SMacro;     //宏结构体(为什么做成动态内存，释放要直接实例化)

    uint8 ucG66MarkAssign;
    uint8 ucG66MarkCheck;

} SProgram;

/**********************/
//函数声明
/**********************/

//初始化程序结构体
int8 initProgramData(SProgram* pSProgram);

//初始化宏结构体
int8 initMacroStruct(SMacro* pMacro);

//初始化局部变量
void initLocalVar(SLocalMacroVar* pSLocalMacroVar);

//释放程序结构体内开辟的内存
void freeProgramData(SProgram* pSProgram);

//释放宏结构体中开辟的内存
void freeMacroData(SMacro* pSMacro);

//获取程序链表
SList* getProgramList(uint8 ucChannelNum);

//获取当前加工程序结构体
SProgram* getCurrentProgram(uint8 ucChannelNum);

//添加主程序数据节点
int8 addMainProgramNode(uint8 ucChannelNum);

//添加程序数据节点
int8 addProgramNode(int32 cFileName, uint8 ucChannelNum);

//创建并初始化程序结构体
SProgram* createAProgramStruct();

//获取当前程序结构体
SProgram* getCurrProgramStruct(uint8 ucChannelNum);

//当前程序是否为主程序
int8 isMainProgram(uint8 ucChannelNum);

//传入数值，返回文件名
void convertProgramName(int32 nVal, int8 cArr[TEMP_FILENAME_MAX_LENGTH]);

#endif
