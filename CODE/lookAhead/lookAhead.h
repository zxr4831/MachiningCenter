#ifndef _LOOKAHEAD_H
#define _LOOKAHEAD_H

#include "common.h"
#include <stdio.h>
/**
Copyright (c) 广东工业大学
@file lookAhead.h
@author 郭强
@version 24.0
@date  2023/06/23
@brief 速度前瞻程序（缓冲区）
*/

//拟定义复合运动最大速度、起跳速度最大加速度和最小加速度来源 宏定义
#define COMMON_MOTION_PARA_SOURCE_MODE   1      // 0:直接由系统指定固定参数   
												// 1：由各运动轴参数计算得到（每段复合运动指令参数不再相同）

#define LINEARAXIS_NUM_MAX	3					//线性轴总数

//三个宏定义分别用于夹角约束速度计算
#define VECTOR_N1   0
#define VECTOR_N2   1
#define VECTOR_N3   2

//前瞻功能开启标志位 (在lookAhead.c从中定义)
extern bool g_bLookAheadFunctionOpenFlag;

//定义前瞻的三个轴
extern uint8 g_ucLookAheadAxisArray[3];

//前瞻需要初始化的变量（暂）
int8 lookAheadInit(struct SChannel* pSChannel);

//判断是否为可前瞻的G0 洪冠鹏 2024/5/27
bool judgeG0(struct SToolCompensation* pToolElement, uint8 ucChannelNum, int16 sIndex);

//判断是否为平面圆弧
bool judgePlanarArc(struct SToolCompensation* pToolElement, int nArraySize);

//判断该段是否需要速度前瞻
int8 judgeG00G01G02G03(struct SToolCompensation* pToolElement, uint8 ucChannelNum);

//多轴模式下修正进给速度和低速下限值
void correctFeedrateAndLowerLimitSpeed(struct SToolCompensation* pToolcode, int nArraySize, bool bPlanarArcFlag, uint8 ucChannelNum);

//向刀补结构体中添加加速度速度以及加减速类型信息
int8 calcVAndAPara(struct SToolCompensation* pToolElement, struct SChannel* pSChannel, bool bPlanarArcFlag);

//计算进给速度对比系数(防止G99模式下进给速度单位为mm/R直接与mm/ms速度量都比较)
double calcCoefOfFeedrate(struct SToolCompensation* pToolElement, uint8 ucChannelNum);

//calcVAndAPara函数调用完毕后再次修正切削速度和低速下限值
void correctFeedrateAndLowerLimitSpeedAgain(struct SToolCompensation* pToolElement, uint8 ucChannelNum);

//在前瞻可读区间内将前瞻读写指针移动至G00G01G02G03单元
bool moveToG00G01G02G03(int16 sLookAheadRes, uint8 ucChannelNum);

//计算圆弧法向加速度约束下的最大速度
double calcArcMaxNormalAccVmax(double dR);

//计算圆弧弓高误差约束下的最大速度
double calcArcMaxChordErrorVmax(double dR, double dF);

//单轴反向时VEER计算
bool calcVeerWhenAxisRev(struct SToolCompensation* pPreTraj, uint8 ucChannelNum, double* dES_Vector_MultiArray, double* dES_Vector_SubArray, const uint8* ucMapArray);

//纯几何Veer计算后，对Veer进行修正
void correctVeerAfterCalcVeer(uint8 ucChannelNum, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj);

//计算前后段轨迹进给速度最小值  2023.6.23
double calcMinFeedrateBtnPreAndLaterTraj(uint8 ucChannelNum, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj);

//计算两相邻轨迹衔接点角度约束速度（多轴模式）
bool calcVeerMultiAxisMode(struct SChannel* pSChannel, int nArraySize, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj);

//计算两相邻轨迹衔接点角度约束速度（三线性轴模式）
bool calcVeerLinearAxisMode(struct SChannel* pSChannel, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj);

//求两数组比值结果最小值
double calcMinTwoArrays(double* dArray1, double* dArray2);

//获取位移速度计算系数
double getCoef(uint8 ucVelType);

double getSingleAxisMaxAcc(int i, uint8 ucVelType, uint8* ucMapArray);
double getSingleAxisMaxDec(int i, uint8 ucVelType, uint8* ucMapArray);

double getMaxAcc(uint8 ucVelType);

double getMaxDec(uint8 ucVelType);

//计算衔接点前段轨迹长度约束速度
double calcVac(struct SToolCompensation* p, struct SChannel* pSChannel);

//计算衔接点后段轨迹长度约束速度
double calcVde(struct SToolCompensation* p, struct SChannel* pSChannel);

//计算需要前瞻的连续轨迹段数
int16 calcNeedLookAheadNum(int16 sLookAheadWIndex, int16 sLookAheadRes, uint8 ucChannelNum);

//计算前瞻区间首段初速度
double calcFirstElementVs(struct SToolCompensation* pToolElement, uint8* ucMapArray, uint8 ucChannelNum);

//计算前瞻区间尾段末速度
double calcLastElementVe(struct SToolCompensation* pToolElement, uint8* ucMapArray, uint8 ucChannelNum);

//前瞻计算函数
uint8 calcVsVe(int16 sNeedLookAheadNum, struct SChannel* pSChannel);

//无前瞻模式
bool noLookAheadMode(uint8 ucChannelNum);

//有前瞻模式
int8 lookAheadMode(int16 sLookAheadRes, struct SChannel* pSChannel);

//速度不前瞻
bool lookAheadForbidden(struct SChannel* pSChannel);

//速度前瞻
bool velocityLookAhead(struct SChannel* pSChannel);

//测试用求衔点角度
double calcAngle(struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj);


#endif
