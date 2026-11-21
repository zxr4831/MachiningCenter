#ifndef __GCODECOMMANDAPI_H
#define __GCODECOMMANDAPI_H


#include "GCodeG0.h"
#include "GCodeG1.h"
#include "GCodeG2.h"
#include "GCodeG3.h"
#include "GCodeG4.h"

#include "GCodeG7.1.h"

#include "GCodeG10.h"
#include "GCodeG15G16.h"

#include "GCodeG17G18G19.h"
#include "GCodeG20G21.h"

#include "GCodeG28.h"

#include "GCodeG31.h"

#include "GCodeG32.h"
#include "GCodeG32.2.h"
#include "GCodeG32.3.h"
#include"GCodeG33.h"
#include "GCodeG34.h"
#include "GCodeG40G41G42.h"

#include "GCodeG50.h"

#include "GCodeG50.2.h"
#include "GCodeG51.2.h"


#include "GCodeG54.h"

#include "GCodeG65.h"
#include "GCodeG66.h"

#include "GCodeG70.h"

#include "GCodeG71.h"

#include "GCodeG73.h"

#include "GCodeG74.h"
#include "GCodeG75.h"
#include "GCodeG76.h"


#include "GCodeG80.h"
#include "GCodeG83.h"
#include "GCodeG83.1.h"
#include "GCodeG83.2.h"

#include"GCodeG84.h"

#include "GCodeG87.h"
#include "GCodeG87.1.h"
#include "GCodeG87.2.h"
#include "GCodeG88.h"
#include "GCodeG90G91.h"
#include "GCodeG92.h"
#include "GCodeG93.h"
#include "GCodeG94.h"

#include "GCodeG96G97.h"
#include "GCodeG98G99.h"

#include"MCodeMa.h"
#include"MCodeMb.h"
#include"MCodeMc.h"
#include"MCodeMe.h"
#include"MCodeMf.h"
#include"MCodeMg.h"
#include"MCodeMh.h"
#include"MCodeMi.h"
#include"MCodeMj.h"
#include"MCodeMk.h"

#include "GCodeIfAndWhile.h"

/**********************/
//结构体定义
/**********************/

typedef struct _SDot
{
	int16 sCommand;//源于哪个指令（如G1的直线还是G93的直线）
	SPointDouble SPoint;//点（机械坐标）
	//VAL_STRUCT_TYPE ucRelaVal;//坐标以相对值方式计算的标识符（暂时删除）
}SDot;

//直线段信息结构体
typedef struct _SLine
{
	enum ENTITY EntityType;		//实体类型，OTHERTYPE: LINE:1, ARC:3
	SPointDouble SStartPoint;//起点（机械坐标）
	SPointDouble SEndPoint;//终点（机械坐标）
	int16 sCommand;//源于哪个指令（如G1的直线还是G93的直线）是否有必要?lxw：暂时弃用该变量

	//bool bIsG0;
	//VAL_STRUCT_TYPE ucRelaVal;//坐标以相对值方式计算的标识符（暂时删除）
}SLine;

//圆弧段信息结构体（不能为整圆）
typedef struct _SArc
{
	enum ENTITY EntityType;		//实体类型，LINE:1, ARC:3
	SPointDouble SStartPoint;//起点（机械坐标）
	SPointDouble SEndPoint;//终点（机械坐标）
	SPointDouble SCenterPoint; //圆心

	double dRadius;//圆弧半径
	int8 cArcDir;//圆弧方向
	int16 sCommand;//源于哪个指令（如G1的直线还是G93的直线）（lxw：暂时弃用）

	//VAL_STRUCT_TYPE ucRelaVal;//相对轴
}SArc;

//螺纹段信息结构体
typedef struct _SThread
{
	enum ENTITY EntityType;		//实体类型，LINE:1, ARC:3

	SPointDouble SStartPoint;//起点（机械坐标）
	SPointDouble SEndPoint;//终点（机械坐标）

	double dF;
	double dJ;      //[短轴退尾量   G32参数模态  G92模态]  [G76 0～99999999×最小输入增量 非模态]
	double dK;      //[长轴退尾量   G32参数模态  G92模态]  [G76 0～99999999×最小输入增量 非模态]
	double dQ;      //[G32:1.起始角度 非模态]   [G76: 第1行.Q(△dmin):螺纹粗车时的最小切削量 0～99999,  第2行.Q(△d):第一次螺纹切削深度 1～99999999]
	uint16 usL;  

	bool bRepeatG32Flag;    //是否为连续G32
	bool bG32LetterQ;       //是否出现Q
	uint8 ucG32SubMode;		//G32的子类	用于区分G32.2/G32.3/G32.4 陈敏聪增 2023/6/13

	//G32.2 和G32.3圆弧螺纹专用
	double dRadius;    //半径
	int8 cArcDir;         //圆弧方向
	SPointDouble SCenterPoint;//圆心
}SThread;

//攻牙段信息结构体
typedef struct _STapping
{
	enum ENTITY EntityType;		//实体类型，LINE:1, ARC:3
	int16 sCommand;//源于哪个指令（如G1的直线还是G93的直线）
	SPointDouble SStartPoint;//起点（机械坐标）
	SPointDouble SEndPoint;//终点（机械坐标）
    
	int8 G33RotateAxis;//  G33 旋转轴  可选A轴或C轴
	int8 G33TappingMode;		//G33攻牙模式 0:刚性攻牙 1:柔性攻牙
	uint16 usL;      //螺纹头数
	double dF;// 螺纹导程

}STapping;



//抛物线段信息结构体(暂时没有)
typedef struct _SParabola
{
	int a;
}SParabola;

//Nurbs段信息结构体(暂时没有)
typedef struct _SNurbs
{
	int a;
}SNurbs;




typedef struct _SPathCurrState
{
	int8 sGa;				//用于继承
	int8 sGz;
	SGCommandRegulation* pSGaRegulation;
	SGCommandRegulation* pSGzRegulation;

	SPointDouble SCurrPoint;		//用于继承
}SPathCurrState;

typedef struct _SCheckPathInfo
{
	int8 cStep;
	int8 cPart;

	int8 cCommandSegmentCnt;//用于记录多程序段指令时，当前处理的程序段数，

	int16 sCurrCommandVal;
	SGCommandRegulation* pSCurrCommandRegulation;

	SGetTextStateInfo SGetTextStateInfo;
	SScan SScanContainer;

	SPointDouble STotalOffset;
	SPathCurrState SCurrState;


	SPointDouble SPointA;//起刀点
	SPointDouble SPointC;//加工轨迹终点

	bool bIsXMotionInNS;		//程序段NS中X轴是否移动
	bool bIsZMotionInNS;		//程序段NS中Z轴是否移动
	bool cXMonotonicityInNS;	//程序段NS中X轴的单调性
	bool cZMonotonicityInNS;	//程序段NS中Z轴的单调性

	int8 cXMonotonicityInPath;        //X的单调性  0：暂时未知道单调性  1：递增  -1：递减        此外还有另一作用，G71二型分槽时，X还是表示单调性，而Z=0表示目前处于起始边，Z=1处于终止边
	int8 cZMonotonicityInPath;        //Z的单调性  0：暂时未知道单调性  1：递增  -1：递减



}SCheckPathInfo;

typedef struct _SGetPathInfo
{
	int8 cStep;
	int8 cPart;

	int8 cCommandSegmentCnt;

	int16 sCurrCommandVal;
	SGCommandRegulation* pSCurrCommandRegulation;

	SGetTextStateInfo SGetTextStateInfo;
	SScan SScanContainer;
	
	SPointDouble STotalOffset;
	SPathCurrState SCurrState;

	SList* pSPathList;

}SGetPathInfo;







/**********************/
//函数声明
/**********************/

//添加所有G指令规则
void addAllGCommandRegulationAPI();
void initGCodeCommand(uint8 ucChannelNum);

//添加所有M指令规则
void addAllMCommandRegulationAPI();
void freeAllCommandParaStruct(uint8 ucChannelNum);

//检查参数字、参数值范围
int8 checkParaLetter(char* pAllowLetter, SScan* pSScanContainer);
int8 checkParaRange(int8 cLetter, SScan* pSScanContainer, double dMin, double dMax);
int8 checkPolarCoordinateInput(const SScan* pSScanContainer, const SGCodeSysState* pSGCodeCurrSysState);
int8 checkParaInteger(int8 cLetter, SScan* pSScanContainer);  
bool isSegmentHaveCoordInfo(SScan* pSScanContainer);
void getCoordInfoFromScanInfo(SCoord* pSCoordInfo, SScan* pSScanContainer);

int8 calculateTotalOffset(SGCodeSysState* pSGCodeSysState);

//获取程序段中的绝对坐标
void getMacCoordByScanInfo(uint8 ucChannelNum, SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSTargetPoint, struct _SGCodeSysState* pSGCodeCurrSysState);
void getMacCoordByCoordInfo(uint8 ucChannelNum, SCoord* pSCoordInfo, SPointDouble* pSStartPoint, SPointDouble* pSTargetPoint, struct _SGCodeSysState* pSGCodeCurrSysState);
void getLeadByCoordInfo(SScan* pSScanContainer, double* pLead);


//void getMacCoordByScanInfo(uint8 ucChannelNum, SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSTargetPoint, SPointDouble* pSTotalOffset, bool bIsAbsMethod);
//void getMacCoordByCoordInfo(uint8 ucChannelNum, SCoord* pSCoordInfo, SPointDouble* pSStartPoint, SPointDouble* pSTargetPoint, SPointDouble* pSTotalOffset, bool bIsAbsMethod);

//获取横轴纵轴（结构体轴）
int8 getHorAndVerAxis(uint8* pHor, uint8* pVer, int8 cInterpolationState, int16 sGc);

//两点坐标是否相同
bool isSamePoint(SPointDouble* pSPoint1, SPointDouble* pSPoint2);

//获取圆弧的圆心与半径
double getArcCenterPointAndRadius(SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cArcDir, SPointDouble* pSCenterPoint, double* pRadius, uint8 ucHorAxis, uint8 ucVerAxis);

//获取圆弧类型
int8 getArcType(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, SPointDouble* pSCenterPoint, double dRadius, uint8 ucHor, uint8 ucVer);

//将直线段信息SLine写入解析结构体
void writeLineIntoGCodeBuf(SLine* pSLineInfo, struct SGCodeParse* pSGCodeBufUnit);

//将圆弧段信息SArc写入解析结构体
void writeArcIntoGCodeBuf(SArc* pSArcInfo, struct SGCodeParse* pSGCodeBufUnit);

//将直线段信息SThread写入解析结构体
void writeThreadIntoGCodeBuf(SThread* pSThreadInfo, struct SGCodeParse* pSGCodeBufUnit);
//将攻牙段信息SThread写入解析结构体
void writeTrappingIntoGCodeBuf(STapping* pSTappingInfo, struct SGCodeParse* pSGCodeBufUnit);

void writeEntityIntoGCodeBuf(enum ENTITY* pEntity, struct SGCodeParse* pSGCodeBufUnit, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cMachineDir);

int8 writePathListIntoGCodeBuf(SList* pSPathList, uint16 usStartPos, uint16 usEndPos, uint16* pCurPos, struct SGCodeParse* pSGCodeBufUnit, int8 cMachineDir);

//当前坐标编程方式是否为绝对编程（是绝对编程还是相对编程）
bool isAbsProgrammingMethod(uint8 ucMod, uint8 ucChannelNum);

int8 findTargetSegment(uint8 ucMod, uint8 ucChannelNum, SProgram* pSCurrProgram, uint32 unTargetN, SGetTextStateInfo* pSTargetTextInfo);
int8 findIfAndWhileSegment(uint8 ucMod, uint8 ucChannelNum, SProgram* pSCurrProgram, SGetTextStateInfo* pSGetTextStateInfo);


bool isBasicMotionCommand(int16 sCommandVal);

bool isAllowGCommandInPath(SScan* pSScanContainer);
void getArcMonotonicity(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, SPointDouble* pSCenterPoint, int8 cArcDir, bool* pXIsMono, bool* pZIsMono, int8* pXCurrMono, int8* pZCurrMono);

int8 checkMachiningPathMonotonicity(SProgram* pSCurrProgram, uint32 unNS, uint32 unNF, SGetTextStateInfo* pSTargetTextInfo, uint8 ucChannelNum, int8 cNSMod, bool bXRequestMono, bool bZRequestMono);
int8 checkPathMonotonicity(SList* pPathList);
int8 checkPathAxisMonotonicity(SList* pPathList, uint8 ucAxis);
int8 checkPathXZMonotonicity(SList* pPathList);//判断G7172H0时XZ单调性


int8 getMachiningPath(uint8 ucMod, SProgram* pSCurrProgram, uint32 unNS, uint32 unNF, double dDeltaU, double dDeltaW, SList** ppSPathList, SGetTextStateInfo* pSTargetTextInfo, uint8 ucChannelNum, int8 cG71orG72);
void removeEntryAndExitPath(SList* pSPathList, int8 cAxis, bool bRemoveEntry, bool bRemoveExit);

void offsetEntity(enum ENTITY* pEntity, double dDeltaX, double dDeltaZ);

void offsetPathList(SList** ppSPathList, double dDeltaX, double dDeltaZ);

int8 offsetPathList_2(SList* pSPathList, SList** ppSPathList_2, double dDeltaX, double dDeltaZ);

void mirrorEntity(enum ENTITY* pEntity);
//镜像轨迹链表
void mirrorPathList(SList** ppSPathList);

int8 getLocalMacroVarBySegment(SGetTextStateInfo* pSGetTextStateInfo, SScan* pScan, SLocalMacroVar* pSLocalMacroVar);

//判断圆弧是否为凹槽
int8 isArcIsGroove(SPointDouble* SStartPt, SPointDouble* SEndPt, SPointDouble* SCenterPt, int8  cArcDir, int8 cFlag);
int8 getPathDataMonotonicity(enum ENTITY* pSPathData, int8 cAxisXorZ);
//分槽
bool divideGroove(SList** ppPathList, SList** ppGrooveList, int8 cAxisXorZ, int8 cFeedDir);
//bool divideGroove(SList** pathList, SList** pGrooveList, int8 cAxisXorZ, int16* sCurrPathPos, int16* sStartOrEnd, int8* cMonotonicity);

int8 getPathMachineDir(SList* pPathList, int8* cMachineXDir, int8* cMachineZDir);


int16 findEntityPosInGroove(SGrooveData* pSGrooveData, bool bFeed, SPointDouble* pSTargetPoint, bool bStartLine);


SGrooveData* getGrooveDataByGrooveNum(SList* pGrooveList, int16 cGrooveNum);
int8 findTargetPointInEntity(enum ENTITY* pEntity, int8 cAixsXorZ, double dXorZVal, double* pTargetVal);
int8 findTargetPointInGroove(SGrooveData* pSGrooveData, bool bStartPath, int8 cAixsXorZ, double dXorZVal, double* pTargetVal);
//int8 findCutEndPoint(SList* pGrooveList, int8 cStartGrooveNum, int8 cAxisXorZ, double dXorZVal, int8 cDir, double* pTargetVal);
int8 findCutEndPoint(SList* pGrooveList, int16 cStartGrooveNum, int8 cAxisXorZ, double dXorZVal, int8 cDir, double* pTargetVal, int8 cMachineXDir);

int8 splitPathByBlank(SList* pSPathList, double dX, double dZ);
int8 entityCrossBlank(SLine* pEntity, double dX, double dZ, double* pTargetX, double* pTargetZ);
void deleteEntityInPath(SList* pSPathList, double dX, double dZ);
int8 addEntityInPath(SList* pSPathList);

int8 isArcCrossQuadrant(SArc* pSArc, SPointDouble* pSPoint);
void divideArcInPath(SList* pSPathList);

//等距实体
void equidistantEntity(enum ENTITY* pEntity, double dDist, int8 cDir);
//等距轨迹链表
void equidistantPath(SList* pSOriPathList, SList** ppSNewPathList, double dDist, int8 cDir);
//在等距线间添加圆弧
void addArcInequidistanLine(SList* pSOriPathList, SList* pSNewPathList, double dDist);
//删除环
void deleteRings(SList* pSPathList);
//计算直线段与直线段交点坐标
int8 calcIntersectionInLineAndLine(SLine* pSLine1, SLine* pSLine2, SPointDouble* pSPoint);
//计算直线段与圆弧段交点坐标（输入圆弧不能跨象限）
int8 calcIntersectionInLineAndArc(SLine* pSLine, SArc* pSArc, SPointDouble* pSPoint);
//计算圆弧段与圆弧段交点坐标（输入圆弧不能跨象限）
int8 calcIntersectionInArcAndArc(SArc* pSArc1, SArc* pSArc2, SPointDouble* pSPoint);
//计算两段实体的交点坐标（相交返回1 不相交返回0）
int8 calcIntersectionInTwoEntity(void* pEntity1, void* pEntity2, SPointDouble* pSPoint);


//获取圆弧方向
void getArcDir(SArc* pSArc);
//计算圆角与直线或者圆弧交点
void calIntersectionPointsInRounded(enum ENTITY* pEntity, SPointDouble* pSPointCenter, double dRadiusCenter, SPointDouble* psPoint);
/* 自动倒圆公共接口约定：
 * 输入为 G18 平面的有限轮廓及真实毫米半径；输入轨迹不被修改。
 * 返回 1 成功，0 同向共线直线无需倒圆，-1 无解，-2 非法输入。
 * 只有返回 1 时 pArcInfo 才有效；失败或无需倒圆时保持调用前内容。
 * 模态、附带指令、非平面轴和段边界限制由写入层检查，函数说明见对应实现。
 */
//计算直线接直线 圆角轨迹信息
int8 calAngleRoundedInfoInLineAndLine(SLine* pLine1, SLine* pLine2, double dRadius, SArc* pArcInfo);
//计算直线接圆弧 圆角轨迹信息
int8 calAngleRoundedInfoInLineAndArc(SLine* pLine1, SArc* pSArc2, double dRadius, SArc* pArcInfo);
//计算圆弧接圆弧 圆角轨迹信息
int8 calAngleRoundedInfoInArcAndArc(SArc* pSArc1, SArc* pSArc2, double dRadius, SArc* pArcInfo);
//计算圆弧接直线 圆角轨迹信息
int8 calAngleRoundedInfoInArcAndLine(SArc* pSArc1, SLine* pLine2, double dRadius, SArc* pArcInfo);




//打印轨迹链表的轨迹点
void printDataG71G72I(void* data);
//打印槽轨迹点
void printDataG71G72II(void* data);
//打印槽链表中各个槽轨迹数据
void printListII(SList* pList, PrintNode print);

#endif
