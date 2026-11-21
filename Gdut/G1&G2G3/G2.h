#ifndef __G2_H
#define __G2_H
#include "common.h"


//#define G02_AXIS 3   //G02插补的轴数
#define EllipseTEST		0   //椭圆代码测试

struct SG2     // 2022.3.14    G2 执行结构体
{
	struct SChannel *pSChannel; // 通道指针  (父)

	uint32 unCommandType;			//指令类型
	VAL_STRUCT_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12

	struct SInput SUserData;	// 用户（输入）参数
	struct SPlan SPlanData;			//速度规划参数

	/******************* 运动参数  与点有关的部分 *****************/
	SVector SStartVector;			//起点矢量（单位向量）
	SPointDouble SStartPoint;		//起点，单位：mm
	SPointDouble SEndPoint;			//终点，单位：mm

	SPointDouble SMaxVal;//运动轨迹上各轴最大值点的集合，单位：mm   2024-4-11 洪冠鹏
	SPointDouble SMinVal;//运动轨迹上各轴最小值点的集合，单位：mm

	float dA; //椭圆参数A
	float dB; //椭圆参数B
	SPointDouble SEllipseSCenterPoint;			//圆心，单位：mm
	double dCoordRotation; // 坐标系旋转弧度  rad
	float  fCylinderRadius; // 圆柱半径 mm  2023.3.29 朱稳中

	int8 cArcDir;					//圆弧方向，顺时针=1，逆时针=-1
	double dRad;					//弧度
	double dRadius;					//半径，单位：mm
	SPointDouble SCenterPoint;		//圆心，单位：mm（相对起点

	/***************************************************************/

	/*********************内部进给速度 倍率*************************/
	double dFeedrate;				//进给速度  mm/ms
	double dFeedRatio;				//进给倍率
	double dAcc; // 加速度  原始值
	double dDec; //减速度  原始值
	/***************************************************************/

	double dCurrentSpeed;		//当前速度 mm/ms

	uint8 ucInpStatus;					//插补状态，INITIATION    ASSIGNMENT_START    RUNNING    PAUSING    FINISHED
	//uint8 ucRunStep;					//步骤

	uint8 ucAxis[2];  //执行插补的轴号  [0] ->> 横轴   [1] ->> 纵轴

	bool bUseSRunFeedRatio;	//是否使用SRun倍率
	bool bIsUserData; //使用用户数据    2022.12.6
	double dPulseToMm[AXIS_NUM_MAX];	//脉冲当量的倒数  各个轴  XYZ ABC

	double dCurEndPointAngle;       //当前段终点处速度与横轴间的夹角 2024/4/12 洪冠鹏
	double dNextStartPointAngle;	//下一段起点处速度与横轴间的夹角 2024/4/12 洪冠鹏
	bool bReCalculateDec;			//减速到软限位点的减速度重算标志位 2024/4/12 洪冠鹏
	double dReCalculateVe;          //根据软限位点重算末速度 2024/4/12 洪冠鹏
};

//extern struct SG2 g_SG2;

//G2加载系统参数（初始化）
int8 SG2LoadSysPara(struct SG2* pSG2);

//选择插补平面  G17 G18 G19
int8 chooseInterpPlane(uint8* ucAxisArray, uint32 unCommandType);


//圆弧计算长轴和距离分量
int8 calArcVectorAndDistance(struct SG2* pSG2Pointer);

//计算圆弧坐标点函数
int8 calculateArcInterpolationPoint(double dDeltaL, struct SG2* pSG2Pointer, double* pdTargetPoint);

//普通圆弧转化为椭圆
int8 commonArc2Ellipse(double *dPoint ,double *dCenterPoint,double *dOutputPoint,int nArraySize,double dA,double dB , double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis);

//椭圆转化为普通圆
int8 ellipse2CommonArc(double* dPoint, double* dCenterPoint, double* dOutputPoint, int nArraySize, double dA, double dB, double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis);

//暂停初始化函数
int8 prepareToPauseG2(struct SG2* pSG2Pointer, double dVmin, double dDec, bool bWriteBackFlag);

//从Input参数赋值到G2结构体
int8 loadSG2DataFromInput(struct SG2* pSG2Pointer, struct SInput* pSInput);

//圆弧插补运动
int8 motionArc(struct SG2* pSG2Pointer, bool bPauseOn, bool bRatioOn);

// 计算圆弧角度
double calcArcAngle(double *pCenterPoint, double *pStartPoint, double *pEndPoint, uint8 ucInpDir);

// 检查G2哪根轴最先触发软限位
int8 checkG2FirstAxisSoftLimit(struct SChannel *pSChannel, SPointDouble SStartPoint, SPointDouble SMaxVal, SPointDouble SMinVal);

#endif // __G2_H

