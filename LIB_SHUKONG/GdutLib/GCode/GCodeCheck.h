#ifndef __GCODECHECK_H
#define __GCODECHECK_H

#include "common.h"
#include "list.h"

//lxw:头文件包含关系
//#include "GCodeCommon.h"
//#include "GCodeCheckAndAssign.h"
//#include "GCodeCalcExpression.h"

/**********************/
//结构体定义
/**********************/

//检查结构体
typedef struct _SSyntaxConfiguration
{
    uint8 ucChannelNum; //通道号
    uint8 ucStatus;     //检查状态机

    int8 cPart; //检查切片
    int8 cTargetStatus;

    uint16 usMainProStartLine; //主程序起始单节(行）

    //int16 sBeginLineNum;         //G代码开始的行号，第一个%号行	//是否放入程序结构体中    (移动到程序结构体)
    //int32 nEndLineNum;			//G代码最后一行的行号，第二个%号行	//是否放入程序结构体中

    int8 cCommandSegmentCnt; //指令的程序段实时计数	//计划放入SGetTextStateInfo中，因为G71小循环需要用到此变量

    uint32 unTotalCommandCnt;
    uint32 unLineCnt; //已运行行数，只在切换程序时清0  用于判断检查赋值速度

    SGetTextStateInfo SGetTextStateInfo; //获取文本状态信息（结构体内不需开辟动态内存）

    //int8* pCurrPtr;                             //当前字符的指针，用于移动，（lxw移动到文本结构体）
    //int8* pStartPtr;                            //指向字符串开头，用于标记
    //int8* pEndPtr;                              //指向字符串后一位，用于标记

    //struct SGCodeModalInfo SModalInfo;			//模态结构体（结构体内不需开辟动态内存）（lxw移动到扫描结构体）
    SScan SScanContainer; //扫描结构体

    bool bCheckFinished; ////20250122：不能够通过检查结构体知道检查是否完成，lxw新增此标记

    //SGroupViolation SGroupViolator;                  //模态冲突结构体
    uint8 ucMLetterCnt;  //M功能出现次数
    int8 cCounterNoMove; //C刀补没移动代码计数器

    int16 sCurrCommandVal; //判断多程序段指令，每一段是否相同

    SGCommandRegulation*
        pSCurrCommandRegulation; //当前程序段需要执行的cGa、cGz指令的规则结构体地址。流程中需要多次用到cGa，cGz的规则结构体

    struct _SGCodeSysState SGCodeCurrSysState; //当前通道状态结构体     赋值前使用函数进行初始化一次
    struct SGCodeParse SCurrRecord;            //创建一个结构体，记录本行的信息，类似G代码缓冲区

    //SPointDouble SWorkPieceOffsetByG50;             //因G50引起的工件坐标偏移量(与解析框架无关，删除???)

    //SPointDouble STotalOffset;                        //总的偏移量 注意:总偏移的X是半径值（可进行记录，防止偏置值不变时，每个程序段都需要重新计算偏置）

    bool bErrorFlag; //辅助标志位，作用:如果为TRUE，不令G71进入ns~nf的检查

    //等待指令相关

    uint8 ucRelevantChannel; //等待标记相关的通道  说明:例如WAITM(4,1,3,5),此时sWaitMark=4，ucRelevantChannel=0001 0101

    int8 cSpindleStatus;       //1:正转 0:停止 -1:反转 2:夹紧（只更新，不继承）
    int8 cSSecondpindleStatus; //1:正转 0:停止 -1:反转（只更新，不继承）

    int8
        cSpindleSpeedAssignFlag; //lxw:为了区分程序段单独出现S，S值该赋给主轴速度or第二主轴速度，若标志为0，S赋给主轴；若标志为1，S赋给第二主轴。
                                 //该标志在主轴/第二主轴控制指令赋值接口中写入（M3M4/M51M52）

    //	uint8 ucG66MarkAssign;

    //    int8 *cOriginalStr; //当前行字符串
    //    uint8 ucOriStrIndex;  //当前行字符串的索引
    //    uint8 ucInsertIndex;//插入索引
    //    bool bIgnored;  //忽略括号中内容，TRUE:括号内  FALSE:非括号内

    //*************************************双主轴机相关********************************************
    bool bToolCmpBuild;               //刀补建立标志位	（放入G41文件中？？？）
    bool bToolCmpPreCancel;           //刀补预取消标志位（放入G41文件中？？？）
    bool bG40Appear;                  //G40出现标志位（放入G41文件中？？？）
    VAL_STRUCT_TYPE ToolAxic;         //刀补占用轴
    bool bToolChange;                 //通道换刀标志位，用于通道间换刀的判断
    bool bWorkPieceOffsetNumChange;   //工件坐标号改变标志位
    bool bWorkPieceOffsetByG50Change; //G50改变标志位
    bool bG92IsWorkingByGcode;        //测试用

} SSyntaxConfiguration;

/**********************/
//函数声明
/**********************/

//获取检查结构体
SSyntaxConfiguration* getSyntaxConfig(uint8 ucChannelNum);

//初始化检查结构体
void initSyntaxConfig(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum);

//检查状态机
void syntaxStatusMachine(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum);

//检查程序段总接口
void checkSegments(struct _SSyntaxConfiguration* pSSyntaxConfig, uint8 ucChannelNum);

//检查cGa、cGz以外的G指令(part4)
void checkGbCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGcCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGdCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGeCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGfCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGgCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGjCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGhCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkGxCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

//检查FST指令(part4)
void checkFCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkSCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
void checkTCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

//检查特殊指令
int8 checkSpecialCommand(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

//检查找到错误
void findErrorStart(struct _SSyntaxConfiguration* pSSyntaxConfig, uint16 ErrorType, int8 character);
//检查找到错误
void findError(const int8* pStartPtr, const uint32 unLineNum);

#endif
