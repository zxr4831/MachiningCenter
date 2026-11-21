/*
后续需要将此文件中的函数声明归类到各模块的头文件中
*/

#ifndef __GCODE_H
#define __GCODE_H

#pragma warning(disable : 4996) //处理VS环境下读文件的警告

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "common.h"
#include "manageMem.h"
#if STM32_MODE
#include "dy_link.h"
#endif
#include "tree.h"
#include "list.h"

#include "GCodeCommon.h"

#include "GCodeCalcExpression.h"

#include "GCodeProgram.h"

#include "GCodeCheckAndAssign.h"
#include "GCodeQuickHandle.h"
#include "GCodeCheck.h"
#include "GCodeAssign.h"

#include "GCodeRegulation.h"

#include "GCodeCommandAPI.h"

#include "GCodeDefine.h"

#include "GCodeRegulation.h"
#include "GCodeTest.h"

#define PRINTMODE 0    //1:打G代码印缓冲区单元内容 2:打印G代码NC文件 3:打印运动插补点NC文件  4.打印运动插补点到test文件
#define CONSOLEPRINT 0 //控制台打印G代码NC文件

#define CHANNEL_COMMAND_MIN_VAL 100
#define CHANNEL_COMMAND_MAX_VAL 199
//extern struct SG50RingBuff g_SG50RingBuff[CHANNEL_NUM_MAX];//G50环形缓冲区

//通道结构体
typedef struct _SChannelConfiguration
{
    int8 cUserID;       //使用者的ID
    uint8 ucChannelNum; //通道号

    int8 s_cMDIStep; //MDI步骤

    uint8 ucStatus; //运行状态

    bool bSkipMode;

    struct _SSyntaxConfiguration SSyntaxConfig; //语法检查结构体（内部无动态内存）
    struct _SAssignConfiguration SAssignConfig; //赋值结构体（内部无动态内存）
    SList* pProgramList;                        //程序链表  pData是SProgram
    uint8
        ucMainMacroTablePos; //主宏程序(即第一次出现的宏程序)在链表中的位置加一  (因为程序表用的是头插法，所以此位置是从程序表最后一个位置开始数)  还没用 利用ucMacroProgramCnt进行确认
    uint8 ucMacroProgramCnt; //宏程序数量

    struct _G66InfoStruct* pSG66InfoArr[NESTING_LIMIT]; //G66信息结构体 指针变量数组

    //SMacroCalculate g_SMacroCalCheck;   //与宏运算有关的结构体（内含动态内存）
    //SMacroCalculate g_SMacroCalAssign;

} SChannelConfiguration;

/**********************  宏相关  *************************/

struct _SystemMacro //系统宏结构体
{
    uint8* MarcoAdd[32]; //IO输出口地址
};

extern struct _SystemMacro g_SystemMacro; //系统宏

extern SChannelConfiguration* s_pSChannelConfig[CHANNEL_NUM_MAX]; //G代码通道结构体指针

//*****************G50 2023.7.28***************************************
struct SG50Info //G50信息结构体，记录因G50产生的工件坐标偏移量 和G50所在的行号
{
    SPointDouble SWorkPieceOffsetByG50; //因G50引起的工件坐标偏移量
    uint32 unG50LineNum;                //G50所在的行号
};

//*******************G50 end****************************************
//*******************G10 start****************************************
struct SG10Info //G10信息结构体
{
    SPoint SToolCMPByG10; //G10设置的刀具磨损补偿量
    uint32 unG10LineNum;  //G10所在的行号
};
//*******************G10 end****************************************
union G50G10Info //G50G10信息联合体
{
    struct SG50Info SG50Info;
    struct SG10Info SG10Info;
};

struct SG50RingBuff //G50环形缓冲区结构体
{
    int16 sSG50_WRITEIndex;                           //G50写入索引
    int16 sSG50_WRITEDNum;                            //G50已经写入的且未读走的个数
    union G50G10Info G50G10Ringbuf[G50_INFO_BUF_MAX]; //G50环形数组
};

#if !STM32_MODE
//extern int8  g_cMapBufferTEST[FILESIZE];      //测试使用，将G代码文本放入此字符串     通道1
//extern int8  g_cMapBufferS1[FILESIZE];     //测试使用，将G代码文本放入此字符串     通道2

extern int8 g_cName_C0[20]; //通道0
extern int8 g_cName_C1[20]; //通道1
extern uint8 g_ucNameType;
extern int8 g_textBuf[20][MAX_LENGTH_OF_LINE];
#endif

extern bool g_bGCodeDraw[CHANNEL_NUM_MAX];

#if ((MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE) && (CHANNEL_NUM_MAX > 1)) //双主轴模式下的刀具号，磨损号
extern int8 g_cToolCompNumAssiagn; //双主轴模式下，双通道的刀具号（语法赋值的）
extern int8 g_cToolNumAssiagn;
extern int8 g_cToolNumSyntax;     //双主轴模式下，双通道的刀具号（语法检查的）
extern int8 g_cToolCompNumSyntax; //专双主轴模式下，双通道的刀具磨损号

extern int8 g_cWorkPieceOffsetNumSyntax;  //双主轴模式下，双通道的工件坐标号 G54-G59（语法检查的）
extern int8 g_cWorkPieceOffsetNumAssiagn; //双主轴模式下，双通道的工件坐标号 G54-G59（语法赋值的）

extern SPointDouble g_SWorkPieceOffsetByG50Syntax;  //双主轴模式下，双通道的G50偏移值（语法检查的）
extern SPointDouble g_SWorkPieceOffsetByG50Assiagn; //双主轴模式下，双通道的G50偏移值（语法检查的）

#endif

//译码模块初始化
void gcodeInit(uint8 ucChannelNum, uint8 ucIsAllowGodeRun, uint8 ucIsGetBeforeProgram);

//开辟G代码通道结构体内存
SChannelConfiguration* mallocGCodeChannelConfig();

//初始化G代码通道结构体
void initGCodeChannelConfig(uint8 ucChannelNum);

//释放译码模块开辟的内存
void freeAllGcode();

//释放G代码结构体内开辟的内存
void freeGCodeChannelConfig(SChannelConfiguration* pSGCodeChannelConfig);

//获取通道结构体地址
SChannelConfiguration* getChannelConfig(uint8 ucChannelNum);

//译码模块总接口
int8 mainFuncGCode(uint8 ucChannelNum);

//AUTO模式译码总接口
int8 mainFuncGCodeAuto(uint8 ucChannelNum);

//MDI模式译码总接口
int8 mainFuncGCodeMDI(uint8 ucChannelNum);

//译码模块的运行按钮中接口
void runbuttonAPI_GCode(uint8 ucChannelNum);

//获取当前运行的行号
uint32 getSRunLine(uint8 ucChannelNum);

//**********************************************GCodeG50.c**************************************************************
//G50赋值
void assignG50(uint8 ucSyntaxOrAssign, uint8 ucChannelNum, SSyntaxConfiguration* pSyntax,
               SAssignConfiguration* pAssign);
//初始化G50环形缓冲区
int8 initG50RingBuffer(uint8 ucChannelNum);
//	获取G50指针的索引值（通过类型）
int16 getSG50PointerIndex(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum);
//获取G50环形数组元素指针(通过索引)
union G50G10Info* getSG50PointerByIndex(short sIndex, uint8 ucChannelNum);
//	获取G50环形数组元素指针(通过类型)
union G50G10Info* getSG50PointerByType(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum);
//	获取G50指针剩余容量
int16 getSG50PointerBufRes(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum);
//	移动G50指针
bool moveSG50Pointer(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum);
//	返回G50缓冲区的地址
union G50G10Info* getG50RingBuffHeadAddr(uint8 ucChannelNum);

//圆弧最值点计算
void getAxisMaxAndMin(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin);

//系统参数宏变量初始化
void initMacroVariable(uint8 ucControl);
//系统宏初始化
void InitSystemMacro();

#endif // !_GCODE
