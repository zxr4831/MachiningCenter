/*
G代码.c文件介绍
GCodeCheck.c		检查框架模块
GCodeAssign.c		赋值框架模块
GCodeRegulation.c	指令规则模块


GCodeG**.c          G**指令模块


*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Config.h"
#include "Gcode.h"
#include "GeneralFunctions.h"
#include "ErrorFunction.h"
#include "NCDataPara.h"
#include "Channel.h"
#include "ringbuff.h"

#if STM32_MODE
#include "dir.h"
#include "app_num.h"
#include "menu_app.h"
#endif

//const double EPSINON2 = 0.001;//用于变量数值比较
//static SChannelConfiguration s_pSChannelConfig[CHANNEL_NUM_MAX];

Tree* g_pSGCommandRegulationTree = NULL; //G指令规则结构体树
Tree* g_pSMCommandRegulationTree = NULL; //M指令规则结构体树

SChannelConfiguration* s_pSChannelConfig[CHANNEL_NUM_MAX]; //G代码通道结构体指针
struct _SystemMacro g_SystemMacro;                         //系统宏

bool g_bGCodeDraw[CHANNEL_NUM_MAX] = {0}; //G代码绘图变量，为TURE时，运行G代码，并绘图

///***********************************************************  废除以下变量  **************************************************************************/
//VAL_STRUCT_TYPE g_ucGcodeAxisOccupation;//G代码解析轴占用变量
//struct SInheritPointContainer  g_InheritPointContainer; // 通道间继承点容器
//struct SToolInfoContainer g_ToolInfoContainer;//通道间刀具信息容器
//struct SChannelToolAxisOccupy g_ChannelToolAxisOccupy;//通道刀补轴占用

#if ((MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE) && (CHANNEL_NUM_MAX > 1)) //专机模式下的刀具号，磨损号
int8 g_cToolNumAssiagn = 0;     //专机模式下，双通道的刀具号(语法赋值的)
int8 g_cToolCompNumAssiagn = 0; //专机模式下，双通道的刀具磨损号
int8 g_cToolNumSyntax = 0;      //专机模式下，双通道的刀具号（语法检查的）
int8 g_cToolCompNumSyntax = 0;  //专机模式下，双通道的刀具磨损号

int8 g_cWorkPieceOffsetNumSyntax = 0;  //双主轴模式下，双通道的工件坐标号 G54-G59（语法检查的）
int8 g_cWorkPieceOffsetNumAssiagn = 0; //双主轴模式下，双通道的工件坐标号 G54-G59（语法赋值的）
//
//SPointDouble g_SWorkPieceOffsetByG50Syntax=0;//双主轴模式下，双通道的G50偏移值（语法检查的）
//SPointDouble g_SWorkPieceOffsetByG50Assiagn=0;//双主轴模式下，双通道的G50偏移值（语法检查的）
SPointDouble g_SWorkPieceOffsetByG50Syntax;
SPointDouble g_SWorkPieceOffsetByG50Assiagn;

#endif

/*************************************************
  函数名称：void gcodeInit(uint8 ucChannelNum, uint8 ucIsAllowGodeRun, uint8 ucIsGetBeforeProgram)
  函数功能：G代码初始化函数（对G代码所有模块初始化）
  输入参数：ucIsGetBeforeProgram     是否继承原来的程序链表
            ucIsAllowGodeRun        是否开启解析
            ucIsGetBeforeProgram    是否保留程序链表
  输出参数：
  返 回 值：

  备    注：按运行键触发，初始化全部通道
            上一执行的程序中无M30，则继承上一程序信息???
            是否需要将 主程序节点的添加 写在 程序链表初始化 中
            宏变量初始化放哪？？？
            
**************************************************/
void gcodeInit(uint8 ucChannelNum, uint8 ucIsAllowGodeRun, uint8 ucIsGetBeforeProgram)
{
    SChannelConfiguration* pChannelConfig = NULL; //= getChannelConfig(0);  //获取首地址

    SList* pLastProgramList = NULL; //用于保留程序链表 2023/8/26 陈敏聪增

    if (ucChannelNum > CHANNEL_NUM_MAX - 1)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM301);
        return;
    }

    //1.G代码指令模块初始化
    initGCodeCommand(ucChannelNum);

    //2.G代码解析框架模块初始化
    //2.1初始化通道结构体（检查模块、赋值模块、程序模块）
    initGCodeChannelConfig(ucChannelNum);

    //2.2初始化所有计算结构体（计算模块）
    initAllCalculate(ucChannelNum);

#if !STM32_MODE
    //2.3初始化文本缓冲区（文本测试模块）
    initTextBuf(); //初始化测试文本缓冲区（文本调试模块）
#endif

    //初始化G代码解析完成标志
    g_SChannel[ucChannelNum].bGcodeWriteOverFlag = FALSE;

#if TEMP_ACTIVATIVE_CHANNEL_ONE
    if (getRunMode() == AUTOMATIC_MODE && ucChannelNum == 0) ///暂时在此处启动通道0
    {
        changeGCodeStatus(GC_SYNTAX, 0, GCODE_RUN, FALSE, 0);
    }

    if (getRunMode() == AUTOMATIC_MODE && ucChannelNum == 1)
    {
        changeGCodeStatus(GC_SYNTAX, 1, GCODE_RUN, FALSE, 0); //主动启动通道1
    }
#endif
}

/*************************************************
  函数名称：int8 mallocGCodeChannelConfig()
  函数功能：在动态内存中开辟G代码通道结构体
  输入参数：
  输出参数：
  返 回 值：

  备    注：
**************************************************/
SChannelConfiguration* mallocGCodeChannelConfig()
{
    SChannelConfiguration* pSChannelConfig;
    pSChannelConfig = (SChannelConfiguration*)mallocGCode(sizeof(struct _SChannelConfiguration), ORAM_NUM300);
    if (pSChannelConfig == NULL)
    {
        parseGCodeErrorB(9999, GCODE_ERROR_NUM406);
        return NULL;
    }
    return pSChannelConfig;
}

/*************************************************
  函数名称：int8 mainFuncGCode()
  函数功能：初始化G代码通道结构体
  输入参数：
  输出参数：
  返 回 值：0：正常
            1：允许报错
            2：MDI完成
            3：
  备    注：初始化结构体变量的步骤如下
            1.释放结构体中开辟的内存
            2.结构体内全部置0
            3.对初始值非0的变量赋值
**************************************************/
void initGCodeChannelConfig(uint8 ucChannelNum)
{
    SChannelConfiguration* pSGCodeChannelConfig = s_pSChannelConfig[ucChannelNum];
    if (pSGCodeChannelConfig == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }
    //1.释放G代码通道结构体中开辟的内存
    freeGCodeChannelConfig(pSGCodeChannelConfig);

    //2.G代码通道结构体置0
    memset(pSGCodeChannelConfig, 0, sizeof(SChannelConfiguration));

    pSGCodeChannelConfig->bSkipMode = g_SChannel[ucChannelNum].bSkipMode;

    //3.初始化结构体变量
    //3.1初始化检查结构体
    initSyntaxConfig(&pSGCodeChannelConfig->SSyntaxConfig, ucChannelNum);

    //3.2初始化赋值结构体
    initAssignConfig(&pSGCodeChannelConfig->SAssignConfig, ucChannelNum);

    //3.3初始化程序链表
    if (pSGCodeChannelConfig->pProgramList == NULL)
    {
        pSGCodeChannelConfig->pProgramList = initList(ORAM_NUM301, ORAM_NUM302);
        if (pSGCodeChannelConfig->pProgramList == NULL)
        {
            parseGCodeErrorB(0, GCODE_ERROR_NUM406);
            return;
        }
    }

    //4.初始化非结构体变量
    //4.1初始化UserID
    pSGCodeChannelConfig->cUserID;

    //4.2初始化通道号
    pSGCodeChannelConfig->ucChannelNum = ucChannelNum;

    //4.3初始化G代码通道状态
    pSGCodeChannelConfig->ucStatus = GCODE_IDLE;

    //....
}

/*************************************************
  函数名称：freeAllGcode()
  函数功能：G代码释放内存，给企业按下复位按钮时调用
  输入参数：
  输出参数：
  返 回 值：

  备    注：
**************************************************/
void freeAllGcode()
{
    //1.释放G代码解析框架内存
    //1.1释放所有G代码通道结构体内存
    for (uint8 unChannelNum = 0; unChannelNum < CHANNEL_NUM_MAX; unChannelNum++)
    {
        SChannelConfiguration* pSGCodeChannelConfig = NULL;    //= getChannelConfig(0);  //获取首地址
        pSGCodeChannelConfig = getChannelConfig(unChannelNum); //获取某个通道结构体

        freeGCodeChannelConfig(pSGCodeChannelConfig);

        pSGCodeChannelConfig->ucStatus =
            GCODE_IDLE; //通道状态置为空闲 以防复位后通道0的M97启动通道1时没有给通道1的程序链表开辟内存 2024.5.22陈育楠 //2441004 罗祥玮：放入G代码初始化中做
    }

    //2.释放G代码指令内存
    for (uint8 unChannelNum = 0; unChannelNum < CHANNEL_NUM_MAX; unChannelNum++)
    {
        freeAllCommandParaStruct(unChannelNum);
    }
}

/*************************************************
  函数名称：SChannelConfiguration* getChannelConfig(uint8 ucChannelNum)
  函数功能: 释放G代码通道结构体中开辟的内存
  输入参数：pSGCodeChannelConfig     G代码通道结构体

  输出参数：
  返 回 值：
  备    注：1.释放检查结构体中开辟的内存
            2.释放赋值结构体中开辟的内存
            3.释放程序链表
**************************************************/
void freeGCodeChannelConfig(SChannelConfiguration* pSGCodeChannelConfig)
{
    //pSGCodeChannelConfig不能为空
    if (pSGCodeChannelConfig == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    //1.释放程序表
    freeList_2(&pSGCodeChannelConfig->pProgramList, freeProgramData);

    //2.释放G66信息结构体
    for (int8 cTemp = 0; cTemp < NESTING_LIMIT; cTemp++)
    {
        freeGCode(pSGCodeChannelConfig->pSG66InfoArr[cTemp]);
        pSGCodeChannelConfig->pSG66InfoArr[cTemp] = NULL;
    }
}

/*************************************************
  函数名称：SChannelConfiguration* getChannelConfig(uint8 ucChannelNum)
  函数功能: 获取某个通道的G代码解析通道结构体
  输入参数：ucChannelNum     通道号

  输出参数：
  返 回 值：G代码解析通道结构体
  备    注：
**************************************************/
SChannelConfiguration* getChannelConfig(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM301);
        return NULL;
    }
    return s_pSChannelConfig[ucChannelNum];
}

/*************************************************
  函数名称：int8 mainFuncGCode()
  函数功能：G代码主函数
  输入参数：
  输出参数：
  返 回 值：0：正常
            1：允许报错
            2：MDI完成
            3：
  备    注：
**************************************************/
int8 mainFuncGCode(uint8 ucChannelNum)
{
    const enum RUNMODE ECurrRunMode = getRunMode();
    int8 cRet = 0;
    if (ECurrRunMode == AUTOMATIC_MODE || ECurrRunMode == MDI_MODE || ECurrRunMode == HAND_MODE)
    {
        cRet = mainFuncGCodeAuto(ucChannelNum); //自动运行模式
    }
    //else if (ECurrRunMode == MDI_MODE)
    //{
    //    cRet = mainFuncGCodeMDI(ucChannelNum);    //MDI模式
    //}
    return cRet;
}

/*************************************************
  函数名称：int8 mainFuncGCodeAuto()
  函数功能：G代码自动运行模式流程函数
  输入参数：
  输出参数：
  返 回 值：0：正常
            1：语法出错
            2：赋值完成（测试用）
            3：扫描到最后一行（测试用）
  备    注：
**************************************************/
int8 mainFuncGCodeAuto(uint8 ucChannelNum)
{
    SSyntaxConfiguration* pSyntax = getSyntaxConfig(ucChannelNum);
    SAssignConfiguration* pAssign = getAssignConfig(ucChannelNum);

    if (pSyntax == NULL || pAssign == NULL)
    {
        return 1;
    }

    memset(&pAssign->SQuickHandleFlag, 0, sizeof(pAssign->SQuickHandleFlag)); //清空快速处理标志位

    do
    {

        syntaxStatusMachine(pSyntax, ucChannelNum); //语法检查状态机

        assignStatusMachine(pAssign, ucChannelNum); //赋值状态机

        //if (pSyntax->bFinshed && pAssign->bFinshed)  //两者为真时，令标志位为FALSE，同步开启
        //{
        //    pSyntax->bFinshed = FALSE;
        //    pAssign->bFinshed = FALSE;
        //}

    }
    //G代码循环处理功能开 以及 当前G代码行可以循环处理
    while (g_SSystemPara.bGcodeQuickHandleModeEnabled &&
           (pAssign->SQuickHandleFlag.cCycleCounter < g_SSystemPara.sGcode_MultiLineHandleMaxNum &&
            pAssign->SQuickHandleFlag.cNeedQuickHandleFlag != FALSE) &&
           (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) > 1));

    return 0;
}

/*************************************************
  函数名称：int8 mainFuncGCodeMDI()
  函数功能：G代码MDI模式流程函数
  输入参数：
  输出参数：
  返 回 值：0：正常
            1：允许报错
            2：赋值完成
            3：传入指针为空
  备    注：(lxw暂时注释，auto模式完成后，在写此函数)
**************************************************/
int8 mainFuncGCodeMDI(uint8 ucChannelNum)
{
    //0.读取当前MDI步骤
    SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
    if (pChannelConfig == NULL)
    {
        return 3;
    }

    //1.执行MDI对应步骤
    switch (pChannelConfig->s_cMDIStep)
    {
        //1.1检查程序文本
    case 0:
    {
        //检查程序段
        checkSegments(&pChannelConfig->SSyntaxConfig, ucChannelNum);
        //程序文本检查完成
        if (pChannelConfig->SSyntaxConfig.bCheckFinished) //得到结束行号 开始赋值
        {
            pChannelConfig->s_cMDIStep = 1;
        }
        break;
    }
    //1.2赋值程序文本
    case 1:
    {
        //赋值程序段
        assignSegments(&pChannelConfig->SAssignConfig, ucChannelNum);
        //程序文本赋值完成
        if (pChannelConfig->SAssignConfig.cPart == 1 && pChannelConfig->SAssignConfig.bAssignFinished)
        {
            pChannelConfig->s_cMDIStep = 2;
        }
        break;
    }
    //1.3解析结束，释放内存
    case 2:
    {
        assignRun2Finished(&pChannelConfig->SAssignConfig, pChannelConfig->ucChannelNum); //赋值完成前的操作
        pChannelConfig->s_cMDIStep = 3;
        break;
    }
    }

    //2.判断是否写入错误
    if (isErrorFull()                                                    //错误满
        || (isErrorHasBeenWritten() && pChannelConfig->s_cMDIStep == 1)) //赋值过程中出错  或  语法检查完成有错
    {
        return 1; //报错
    }

    return 0;
    //if (pChannelConfig->s_cMDIStep == 0)//初始化，目前因为按运行键会触发gcodeInit，所以不再初始化
    //{
    //	//gcodeInitMDI(ucChannelNum);
    //	pChannelConfig->s_cMDIStep = 1;
    //}
    //else if (pChannelConfig->s_cMDIStep == 1)//语法检查
    //{
    //	//暂时注释 L
    //	//checkSyntax(&pChannelConfig->SSyntaxConfig, pChannelConfig->ucChannelNum);

    //}
    //else if (pChannelConfig->s_cMDIStep == 2)//赋值
    //{
    //	assignSegments(&pChannelConfig->SAssignConfig, pChannelConfig->ucChannelNum);

    //	if (pChannelConfig->SAssignConfig.SGetTextStateInfo.nEndLineNum != UNASSIGNED_VALUE)  //得到结束行号 赋值完成
    //	{
    //		pChannelConfig->s_cMDIStep = 3;
    //		//return 2;
    //	}

    //}
    //   else if (pChannelConfig->s_cMDIStep == 3)
    //   {
    //       ///加不加停止指令？ 暂时不加  motion.c已经根据bGcodeWriteOverFlag 使系统状态变成闲置

    //
    //   }
}

/*************************************************
  函数名称：void runbuttonAPI_GCode(uint8 ucChannelNum)
  函数功能：运行按钮接口_G代码
  输入参数：
  输出参数：
  返 回 值：
  备    注：运行按钮接口中，G代码模块执行内容
**************************************************/
void runbuttonAPI_GCode(uint8 ucChannelNum)
{
    //1.G代码解析初始化
    gcodeInit(ucChannelNum, TRUE, 0);

    //1.所有通道的通道结构体中添加主运行程序
    addMainProgramNode(ucChannelNum);

    //1.G代码解析状态置为GCODE_RUN
    changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN, TRUE, 0); //启动对应的通道
    changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, 0); //启动对应的通道
}

/*************************************************
  函数名称：uint32 getSRunLine(uint8 ucChannelNum)
  函数功能: 获取SRun中的行号
  输入参数：value数字，str目标字符串，radix进制
  输出参数：
  返 回 值：str目标字符串
  备    注：STM32_MODE显示当前运行的行号
**************************************************/
uint32 getSRunLine(uint8 ucChannelNum)
{
    uint32 unGCodeLineNum;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return 0;
    }
    if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE || g_SChannel[ucChannelNum].ucChannelStatus == SYS_RESET ||
        g_SChannel[ucChannelNum].ucChannelStatus == SYS_EMERGENCY_STOP) //洪冠鹏 2024/5/29 空闲时返回首行
    {
        return 0;
    }
    unGCodeLineNum = g_SChannel[ucChannelNum].SRun.unGCodeLineNum; //非单节执行时，行号为当前执行
    if (g_SChannel[ucChannelNum].bSingleLineMode == TRUE &&
        g_SChannel[ucChannelNum].bSingleLineModeUnFinished == FALSE) //单节执行时，当前行执行完，执向下一行
    {
        unGCodeLineNum += 1;
    }
    return unGCodeLineNum;
}
