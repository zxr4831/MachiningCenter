#ifndef __G0_H
#define __G0_H

#include "common.h"
#include "Spindle.h"

//G0标志位的数据类型
#define G0_SIGN_TYPE uint8

#define SINGLEAXIS_MODE SPINDLE_SINGLEAXIS_MODE   // 单轴模式

struct SSingleAxis // 2022.3.14 单轴结构体  单轴插补的执行结构体
{
	// 特殊参数  （只有C轴能使用）
	uint32 unGCodeLineNum;			// 解析到第几行G代码
	uint32 unCommandType;			//指令类型
	double dSpindleSpeed;			//主轴速度
	double dLimitedSpindleSpeed;	// G50 限制的主轴转速(r/min)
	int8 cSpindleDir;				//主轴方向，正转=ARC_CW（1），反转=ARC_ANTI_CW（-1）

	double dSpindleSpeedForChangeAcc;  //主轴加速度改变的检测转速（r/min） 洪冠鹏 2024/3/23
	bool bIsChangeAcc; //改变加速度标志位
	uint8 ucGearForChangeRate;//加速度档位（%）
	
	uint8 ucAxis;//当前轴号 0 ~ AXIS_NUM_MAX-1   //G0程序内部 根据当前轴号和第二层映射 选系统参数

	//struct SInputSingle//（用户设置）参数     相对定义的结构体  ①坐标点变为单个点  ②没有半径和圆心
	//{
	//	double dDistance;		//总距离  插补距离	mm
	//	double dTargetDist;		//目标距离
	//	double dPreDist;		//速度规划前   已走距离
	//	double dCurLeftDist;	//当前留下未移动的距离

	//	double dFeedrate;		//进给速度（最大速度) （用户设置）
	//	double dFeedRatio;		//进给倍率  （用户设置）
	//	double dAcc;			//加速段加速度	mm/ms2
	//	double dDec;			//减速段加速度	mm/ms2
	//	double dVs;				//初始速度	mm/ms
	//	//double dVe;			//最终速度	mm/ms
	//	double dPrimalVe;		//从前瞻继承的最终速度 mm/ms

	//	uint8 ucAccVelPlanType;		//加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	//	uint8 ucDecVelPlanType;		//减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	//	uint8 ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数

	//	double dStartVector;		//起点矢量（单位向量）

	//	double dStartPoint;			//起点，单位：mm
	//	double dEndPoint;			//终点，单位：mm //2021-11-09  朱稳中

	//	int8 cInterpDir;			//插补方向 (手轮方向)

	//	//int8 cArcDir;				//圆弧方向，顺时针=1，逆时针=-1
	//	//double dRadius;			//半径，单位：mm
	//	//SPointDouble SCenterPoint;	//圆心，单位：mm（相对起点)

	//}SUserData; // 用户（输入）参数

	struct SInput SUserData;//（用户设置）参数

	struct SPlan SPlanData;  //速度规划参数

	/******************* 运动参数  与点有关的部分 *****************/
	double dStartVector;	//起点矢量（单位向量）
	double dStartPoint;		//起点，单位：mm
	double dEndPoint;		//终点，单位：mm

	/***************************************************************/

	/*********************内部进给速度 倍率*************************/
	double dFeedrate;		//进给速度  mm/ms 12-30 朱稳中
	double dFeedRatio;		//进给倍率  12-30 朱稳中
	double dAcc; // 加速度  原始值
	double dDec; //减速度  原始值
	/***************************************************************/

	double dCurrentSpeed;		//当前速度 mm/ms

	uint8 ucInpStatus;		//插补状态，INITIATION    ASSIGNMENT_START    RUNNING    PAUSING    FINISHED
	uint8 ucRunMode;		//模式 (单轴恒为SINGLEAXIS_MODE  主轴停下时 为SINGLEAXIS_MODE  运动时为其他 详情见Spindle.h)

	bool bUseSRunFeedRatio;	//是否使用SRun倍率

	double dPulseToMm;		//脉冲当量的倒数  单个轴

	double dReCalculateVe;  //根据软限位点重算末速度 2024/12/12 洪冠鹏
};


struct SG0   //G0运行结构体
{
	struct SChannel *pSChannel; // 通道指针  (父)

	VAL_STRUCT_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12
	struct SInput SUserData;	// 用户（输入）参数

	bool bIsUserData; //使用用户数据    2022.12.6

	G0_SIGN_TYPE ucCurrentSigned;  //当前标志
	G0_SIGN_TYPE ucFinishedSigned; //完成标志
	uint8 ucRunStep;  //步骤
	struct SSingleAxis *pSSingleAxisArray;   //初始化时指向 g_SSingleAxisArray
	SPointDouble SEndPoint; //终点
	int8 cInterpDir; //插补方向
};


//extern struct SG0 g_SG0;
//extern struct SSingleAxis g_SSingleAxisArray[AXIS_NUM_MAX];  //单轴结构体  （数组）

int8 SG0LoadSysPara(struct SG0* pSG0);

int8 SG0Init(struct SG0* pSG0);

//获取单轴的终点坐标
double getSingleAxisEndPoint(struct SChannel* pSChannel, uint8 ucAxis);
 
//设置单轴完成
int8 setFinishedStatus(struct SG0* pSG0Pointer, int nAxisNum);

//单轴暂停初始化函数
int8 prepareToPauseSingle(struct SSingleAxis* pSSingleAxis, double dVmin, double dDec);

//快速运动回写
int8 quickMoveWriteBack(struct SG0* pSG0);

//单轴移动指定距离
int8 singleAxisPre(struct SG0* pSG0Pointer, uint8 ucAxis, SPointDouble SStartPoint, SPointDouble SEndPoint, bool bReCalculate, struct SInput* pSInputPointer);

//单轴移动指定距离循环
int8 singleAxisPreLoop(struct SG0* pSG0Pointer, SPointDouble SStartPoint, SPointDouble SEndPoint, struct SInput* pSInput, VAL_STRUCT_TYPE* ucValStatus, bool bReCalculate);

//从Input参数赋值到G0/单轴结构体
int8 loadSG0DataFromInput(struct SG0* pSG0Pointer, struct SInput* pSInput);

//单轴插补运动
int8 singleAxisMotion(struct SG0* pSG0, uint8 ucAxis, bool bPauseOn, bool bRatioOn);

//单轴插补运动循环
int8 singleAxisMotionLoop(struct SG0* pSG0Pointer,int nAxisNum, bool bPauseOn, bool bRatioOn);

//快速运动函数
int8 motionQuickMove(struct SG0* pSG0Pointer);

// 检查当前轴是否触发软限位
int8 checkSingleAxisSoftLimit(struct SChannel *pSChannel, uint8 ucAxis, SPointDouble SMaxVal, SPointDouble SMinVal);

#endif
