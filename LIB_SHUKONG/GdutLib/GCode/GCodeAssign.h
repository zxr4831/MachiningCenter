#ifndef __GCODEASSIGN_H
#define __GCODEASSIGN_H

#include "common.h"
#include "list.h"

//lxw:头文件包含关系
//#include "GCodeCommon.h"
//#include "GCodeCheckAndAssign.h"

/**********************/
//结构体定义
/**********************/

//圆弧最值点计算专用结构体   洪冠鹏 2024/4/12
typedef struct _SGetAxisMaxAndMin
{
    //int8 cArcDir;	            //圆弧方向，顺时针=1，逆时针=-1
    //uint8 ucHor;                //横轴
    //uint8 ucVer;                //纵轴
    //double dRadius;			    //半径，单位：mm
    //SPointDouble SCenterPoint;	//圆心，单位：mm
    //SPointDouble SStartPoint;	//起点，单位：mm
    //SPointDouble SEndPoint;		//终点，单位：mm
    //SPointDouble SMaxVal;       //最大值点的集合，单位：mm
    //SPointDouble SMinVal;       //最小值点的集合，单位：mm
    int8 cArcDir;               //圆弧方向，顺时针=1，逆时针=-1
    uint8 ucHor;                //横轴
    uint8 ucVer;                //纵轴
    double dRadius;             //半径，单位：mm
    SPointDouble* SCenterPoint; //圆心，单位：mm
    SPointDouble* SStartPoint;  //起点，单位：mm
    SPointDouble* SEndPoint;    //终点，单位：mm
    double dHorMax;             //插补横轴方向的最大值   //计算平面圆弧的最值   洪冠鹏 2024/4/19
    double dVerMax;             //插补纵轴方向的最大值
    double dHorMin;             //插补横轴方向的最小值
    double dVerMin;             //插补纵轴方向的最小值
} SGetAxisMaxAndMin;

//赋值结构体
typedef struct _SAssignConfiguration //仅存放与赋值架构有关的变量，与指令有关的变量或结构体放入对应指令模块中
{
    uint8 ucChannelNum;   //通道号
    uint8 ucStatus;       //赋值状态机
    bool bAssignFinished; //20250122：不能够通过赋值结构体知道赋值是否完成，lxw新增此标记

    int8 cPart; //切片标志   标志作用：判断走到哪个流程
    int8 ucTargetStatus;

    uint16 usMainProStartLine; //主程序起始单节(行）

    //int16 sBeginLineNum;                            //G代码开始的行号，第一个%号行（放入程序节点中？？？）
    //int32 nEndLineNum;                              //G代码最后一行的行号，第二个%号行	（放入程序节点中？？？）

    int8 cCommandSegmentCnt; //当前指令程序段计数

    uint32 unTotalCommandCnt; //总指令计数，用于判断检查赋值进度
    uint32 unLineCnt;

    SGetTextStateInfo SGetTextStateInfo; //获取文本状态信息（结构体内不需开辟动态内存）

    //int8* pCurrPtr;                                  //当前字符的指针，用于移动（lxw移动到文本结构体）
    //int8* pStartPtr;                                 //指针指向词法单元开头
    //int8* pEndPtr;                                   //指针指向词法单元结尾的后一位

    //struct SGCodeModalInfo SModalInfo;			//模态结构体（结构体内不需开辟动态内存）（lxw移动到扫描结构体）
    SScan SScanContainer; //扫描结构体（结构体内不需开辟动态内存）

    int16 sCurrCommandVal; //用于判断占用多行程序段的指令，每一行程序段是否相同

    SGCommandRegulation* pSCurrCommandRegulation; //当前执行Ga、Gz组指令规则

    struct _SGCodeSysState SGCodeCurrSysState; //当前通道状态结构体     赋值前使用函数进行初始化一次
    struct SGCodeParse* pGCodeBuf;             //指针指向该通道对应缓冲区

    //SPointDouble STotalOffset; //实时总偏置                     //总的偏移量 注意:总偏移的X是半径值 yt
    //SPointDouble SG50Offset;//G50实时偏置值			注：不能放入G代码系统状态中，因为使用时非实时值（后续将G代码系统状态改为实时更新）

    //其他偏置值可根据刀补号，工件坐标号在系统参数中查询到值，G50必须记录下来
    //G50若记录在缓冲区中则内存消耗大
    //SPointDouble SWorkPieceOffsetByG50;             //因G50引起的工件坐标偏移量（改为SG50Offset）

    int8 cSpindleStatus;       //1:正转 0:停止 -1:反转 2:夹紧（只更新，不继承）（后续移入G代码系统状态中）
    int8 cSSecondpindleStatus; //1:正转 0:停止 -1:反转（只更新，不继承）（后续移入G代码系统状态中）

    int8
        cSpindleSpeedAssignFlag; //lxw:为了区分程序段单独出现S，S值该赋给主轴速度or第二主轴速度，若标志为0，S赋给主轴；若标志为1，S赋给第二主轴。
                                 //该标志在主轴/第二主轴控制指令赋值接口中写入（M3M4/M51M52）

    //	struct _SPriority SPriorityContainer;           //优先级结构体           给一行赋值前初始化

    uint16 usIndex; //G代码写入缓冲区的索引		//lxw弃用

    //union UCycleStruct *pCycleStructPtr;           //循环共用体
    SList* pList;             //G71~G73数据链表     （移到G71G72G73指令模块参数结构体）
    bool bG70Flag;            //TRUE:G70模式 FALSE：非G70模式
    SList* SGrooveTrackList;  //存放加工槽信息的链表
    bool bLineNum;            //行号标记位,用以G71中行号NS~NF段的判断
    uint8 ucG71G72AssistLine; //记录G71第二段的行号,即PQ行
    //bool bG10Flag;

    ///自动运行模式专属
    bool bFinshed; //当前行赋值完成标记

    //等待指令相关
    uint8 ucRelevantChannel; //等待标记相关的通道  说明:例如WAITM(4,1,3,5),此时sWaitMark=4，ucRelevantChannel=0001 0101

    uint8 ucG66MarkAssign;
    uint8 ucLetterIJKCnt[3]; //IJK计数器，与G65、G66的IJK传参方式有关

#if POLAR_COORD_TEST
    float fCylinderRadius; // 圆柱半径 mm  2023.3.29 从朱稳中处复制
    int8 cPolarHorDir;     //极坐标横轴方向  1和-1
#endif

    struct _SQuickHandleFlag SQuickHandleFlag; //G代码快速处理结构体	陈敏聪新增

    int8 cPartPreMd; //前作用D组M码辅助标记

    int8* cOriginalStr;  //当前行字符串
    uint8 ucOriStrIndex; //当前行字符串的索引
    uint8 ucInsertIndex; //插入索引
    bool bIgnored;       //忽略括号中内容，TRUE:括号内  FALSE:非括号内

    int8 cAssistPart; //切片标记 辅助用

    //SGCommandRegulation* pSGCommandRegulation;//G代码规则指针  （保存当前G指令节点地址）
    //************************************双主轴机相关***************************************
    bool bToolChange;         //通道换刀标志位，用于通道间换刀的判断
    bool bToolCmpBuild;       //刀补建立标志位
    bool bToolCmpPreCancel;   //刀补预取消标志位
    bool bG40Appear;          //G40出现标志位
    VAL_STRUCT_TYPE ToolAxic; //刀补占用轴

    VAL_STRUCT_TYPE GcodeAxic;        //G代码占用轴
    bool bWorkPieceOffsetNumChange;   //工件坐标号改变标志位
    bool bWorkPieceOffsetByG50Change; //G50改变标志位
    bool bG92IsWorkingByGcode;        //测试用
    struct _SGetAxisMaxAndMin
        SGetAxisMaxAndMin; //圆弧最值点计算结构体 洪冠鹏新增	（lxw：与赋值框架无关，移动到G2指令模块）

} SAssignConfiguration;

/**********************/
//函数声明
/**********************/

//获取赋值结构体
SAssignConfiguration* getAssignConfig(uint8 ucChannelNum);

//初始化赋值结构体
void initAssignConfig(struct _SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum);

//赋值状态机
void assignStatusMachine(SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum);

//赋值程序段总接口
void assignSegments(struct _SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum);

//
void assignRun2Finished(SAssignConfiguration* pSAssignConfig, uint8 ucChannelNum);

//赋值cGa、cGz以外的G指令
int8 assignGbCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGcCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGdCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGeCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGfCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGgCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGjCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 assignGhCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 assignGxCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

//赋值FST指令
// 1 成功或未写 F；-1 已报 130，写槽不变，调用方必须停止当前段。
int8 assignFCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 assignSCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 assignTCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

int8 assignSpecialCommand(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
