#define _CRT_SECURE_NO_WARNINGS
#ifndef  __TOOLCOMPENSATION_H
#define  __TOOLCOMPENSATION_H
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include "common.h"
#include"GeneralFunctions.h"
#include"Channel.h"
#include "ToolChanger.h"
#include"lookAhead.h"
#include "Gcode.h"

//测试debug宏
#define ENABLE_TOOL_DEBUG 0

typedef struct
{
	SPointDouble SS1;
	SPointDouble SS2;
	SPointDouble SS3;
	SPointDouble SS4;
	int8 cPointNum;//记录每两段刀补后点的数量
}SSToolRunPoint;


//存放刀补过程的中间点以及刀补的一些状态
typedef struct
{
	struct SGCodeParse SGcodeRecord[CHANNEL_NUM_MAX];
	SPointDouble TcodePrevPoint[CHANNEL_NUM_MAX];//记录前一个刀补偿点
	bool bFristSavePrePointFlag[CHANNEL_NUM_MAX];//初始化
	bool bG4142Flag[CHANNEL_NUM_MAX];//刀补建立标志
	bool bPointFlag[CHANNEL_NUM_MAX];//存储首个刀补点标志
	bool bTurnBackFlag[CHANNEL_NUM_MAX];//回退标志位
	uint8 ucNoMoveCounter[CHANNEL_NUM_MAX];//连续非移动代码段数
}SSToolCompensationRecord;

extern SSToolCompensationRecord* SToolCompensationRecord;//存放刀补过程的中间点以及刀补的一些状态的结构体指针

//在动态内存中开辟存放刀补过程的中间点以及刀补的状态的结构体
SSToolCompensationRecord* initSToolCompensationRecord();
//阶跃函数
double Sgn(double r);

//求直线的方向矢量
void getLineVector(SPointDouble SP1, SPointDouble SP2, SVector *LineVec, uint8* ucAxisNum);

//求转接点在圆弧起点的方向矢量
//void getArcVectorAtStart(double dI, double dJ, SVector *ArcVec, int iArcDir, uint8* ucAxisNum);
void getArcVectorAtStart(double dK, double dI, SVector* SArcVec, int iArcDir, uint8* ucAxisNum);

//求转接点在圆弧终点的方向矢量
//void getArcVectorAtEnd(SPointDouble SP1, SPointDouble SP2, double dI, double dJ, SVector *ArcVec, int iArcDir, uint8* ucAxisNum);
void getArcVectorAtEnd(SPointDouble SPoint1, SPointDouble SPoint2, double dK, double dI, SVector* SArcVec, int iArcDir, uint8* ucAxisNum);

//转接角类型的判断
int JudgeAngle(double dr, SVector Vec1, SVector Vec2, uint8* ucAxisNum);

//特殊角180或0度判断
int SpecialAngleJudge(double dr, SVector Vec1, SVector Vec2, uint8* ucAxisNum);

//求偏移一个半径矢量的转接点函数
void offsetRadiusVector(double dToolRadius, SVector LineVec, SPointDouble SP1, SPointDouble *SP2, uint8* ucAxisNum);

//求两等距线交点
void Intersection_Of_Quidistant_Lines(double dToolRadius, SVector LineVec1, SVector LineVec2, SPointDouble SP1, SPointDouble *SP2, uint8* ucAxisNum);

//插入型偏置点1
void offsetRadiusVector1(double dToolRadius, SVector LineVec, SPointDouble SP1, SPointDouble *SP2, uint8* ucAxisNum);

//插入型偏置点2
void offsetRadiusVector2(double dToolRadius, SVector LineVec, SPointDouble SP1, SPointDouble *Sp2, uint8* ucAxisNum);

//直线与圆弧等距线的交点
void Intersection_Of_Line_And_Arc(double dToolRadius, SPointDouble SP1, SVector LineVec, double dI, double dJ, SPointDouble *SP2, int8 cArcDirection, uint8* ucAxisNum);

//圆弧转直线等距线的交点
void Intersection_Of_Arc_And_Line(double dToolRadius, SPointDouble SP1, SPointDouble SP2, SPointDouble *SP3, SVector LineVec, double dI, double dJ, int8 cArcDirection, uint8* ucAxisNum);

//圆弧转圆弧等距线的交点
void Intersection_Of_Arc_And_Arc(double dToolRadius, SPointDouble SP1, SPointDouble SP2, SPointDouble* SP3, double dK1, double dI1, double dK2, double dI2, int8 cArc1Direction, int8 cArc2Direction, uint8* ucAxisNum, uint8 ucPosFlag);

//快速运动复合位移长度及方向向量（各轴比例）计算(郭强)  2023/05/19郭强 增加VAL_STRUCT_TYPE ucRelaVal参数
int8 calcG0VectorAndDistance(SPointDouble SStartPoint, SPointDouble SEndPoint, SVector* SStartVector, SVector* SEndVector, double* dLength, int nArraySize, uint8* ucMapArray, VAL_STRUCT_TYPE* ucValStatus, VAL_STRUCT_TYPE ucRelaVal);

//多轴插补复合位移长度及方向向量（各轴比例）计算(郭强)  2023/05/19郭强 增加VAL_STRUCT_TYPE ucRelaVal参数
int8 calcG1VectorAndDistance(SPointDouble SStartPoint, SPointDouble SEndPoint, SVector* SStartVector, SVector* SEndVector, double* dLength, int nArraySize, uint8* ucMapArray, VAL_STRUCT_TYPE* ucValStatus, VAL_STRUCT_TYPE ucRelaVal);

//更新刀补轨迹点
void GcodeToTCode(struct SGCodeParse *pGcode, SPointDouble STcodePrevPoint, SPointDouble STcodeEndPoint, bool bPFlag, uint8* ucAxisNum, uint8 ucChannelNum);

//继承G代码缓冲区中公共的参数
void SaveCommonParameter(struct SGCodeParse *pGcode, SPointDouble STcodePrevPoint, SPointDouble STcodeEndPoint, uint8* ucAxisNum, uint8 ucChannelNum);

//根据刀补后的点数记录最后一个刀补点
void RecordLastToolPoint(int8 cPointNum, uint8 ucChannelNum);

//直线接直线
int8 LineToLine(struct SGCodeParse *pSGcode, struct SGCodeParse *pSNextGcode, double dToolRadius, int8 StageSelect, uint8* ucAxisNum);

//直线接圆弧
int8 LineToArc(struct SGCodeParse *pSGcode, struct SGCodeParse *pSNextGcode, double dToolRadius, int8 StageSelect, uint8* ucAxisNum);

//圆弧接直线
int8 ArcToLine(struct SGCodeParse *pSGcode, struct SGCodeParse *pSNextcode, double dToolRadius, int8 StageSelect, uint8* ucAxisNum);

//圆弧接圆弧
int8 ArcToArc(struct SGCodeParse *pSGcode, struct SGCodeParse *pSNextcode, double dToolRadius, int8 StageSelect, uint8* ucAxisNum);

//进行刀补计算
bool toolCompensation(struct SGCodeParse *pSGcode, struct SGCodeParse *pSNextGcode, uint8* ucAxisNum, uint8 ucChannelNum);

//保存刀补计算之后的补偿点并存放到刀补缓冲区
void saveToolCode(struct SGCodeParse *pSGcode, uint8* ucAxisNum, uint8 ucChannelNum);

//判断是否有不移动的情况
bool JudgeNoMoveStruct(struct SGCodeParse *pSGcode);

//初始化标志位
void toolCompensationInit(uint8 ucChannelNum);

//刀补计算流程
void ToolCompensation_Caculate(uint8 ucChannelNum);	

//处理连续出现的超过规定数量非移动代码段
int8 dealMaxCounterNoMoveTrack(struct SGCodeParse *pSGcode, uint8* ucAxisNum, uint8 ucChannelNum);

//不同平面坐标映射
void CoordinateMap(struct SGCodeParse* pSGcode, uint8* ucAxisNum);

//根据不同的假象刀尖号计算横轴偏移值
double getHorizontalAxisOffset(uint8 ucToolNum, double dToolRadius);

//根据不同的假象刀尖号计算纵轴偏移值
double getVerticalAxisOffset(uint8 ucToolNum, double dToolRadius);

//刀补前根据假象刀尖号更改坐标
int8 modifyCoorByToolNum(struct SGCodeParse* pGcode, struct SGCodeParse *SGetGode, uint8* ucAxisNum, uint8 ucChannelNum);

//判断增加的转接线段是否小于一个脉冲,若小于一个脉冲则进行优化处理，减少过渡点
int8 judgeNewLineLength(uint8* ucAxisNum, uint8 ucChannelNum);

/****以下文件调试用****/

//打开文件
void TCODE_OPEN_FILE(uint8 ucChannelNum);

//关闭文件
void CloseFile(uint8 ucChannelNum);

//根据通道打印数据
void PrintDataByChannel(uint8 ucChannelNum);

#endif
