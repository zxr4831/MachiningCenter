#ifndef __SPINDLE_H
#define __SPINDLE_H

#include "common.h"
#include "G0.h"



#define DEGREE_OF_CIRCLE 360.0
#define SPINDLE_FOLLOW_AXIS		AXIS_X //定义主轴G96转速跟随的轴  当前 X轴
#define SPINDLE_AXIS			AXIS_C//定义主轴使用的   起点终点 系统参数等
#define POWERKNIFE_AXIS			AXIS_A	//动力刀的轴
#define G50_INITIAL_SPEED		999999

//ucRunMode的值  表示主轴处于不同的模式
#define SPINDLE_SINGLEAXIS_MODE		0	//单轴模式

#define SPINDLE_G97_MODE			1	//G97
#define SPINDLE_G96ANDG0_MODE		2	//G96 G0
#define SPINDLE_G96ANDG1_MODE		3	//G96 G1+
#define SPINDLE_G96_MODE			7	//G96
#define SPINDLE_COMMONSTOP_MODE		4	//减速停
#define SPINDLE_PRECISESTOP_MODE	5	//准停
#define SPINDLE_HANDWHEEL_MODE		6	//手轮模式

//朱稳中 2023.6.21
#define SINGLEAXIS_LIMIT_POS		7	//正限位
#define SINGLEAXIS_LIMIT_NEG		8	//负限位


#define SPINDLE_INVALID_VAL			-1  //无效的值  （应用于主轴转速被读取后，置为该值）

//系统主轴 停止模式  //朱稳中注释  2023.5.19
//#define SPINDLE_STOP_MODE	SPINDLE_PRECISESTOP_MODE   //SPINDLE_COMMONSTOP_MODE 普通停模式   SPINDLE_PRECISESTOP_MODE 准停模式

struct _SEncoderTest  // 主轴转速检测结构体
{
	struct SChannel* pSChannel;//通道指针

	uint16 usPeriod;//周期数
	int32 nTotalPulse;//时间段内主轴编码器的总值
	double dSpindleAvgSpeed;//主轴平均转速
	uint8 ucStep;//步骤
	uint8 ucSmoothCount;//反向的瞬间平滑滤波次数 2024/3/19
};



struct _SSpindle		// 主轴结构体
{
	struct SChannel* pSChannel;//通道指针
    /* 苏向标 2026-10-14：主轴请求与使能模块：独立保存使能准备占用和计时，不另设停止阶段。 */
    VAL_STRUCT_TYPE ucEnableOccupation; /**< 本对象为使能操作临时占用的逻辑轴位图。 */
    uint32 unEnableRunTime; /**< 当前主轴使能等待的计时状态。 */
    uint32 unDisableRunTime; /**< 当前主轴取消使能等待的计时状态。 */
    volatile uint32 unEnableRequestVersion; /**< 停止时递增，使旧使能请求失效。 */
    volatile bool bStopRequested; /**< 普通停止请求；调用方置位，周期路径读取并清零。 */
    volatile bool bStopNowRequested; /**< 立即停止请求；独立存储，避免普通停止覆盖立即停。 */
    int8 cStopError; /**< 停止失败原因，负值供调试定位。 */
	VAL_STRUCT_TYPE ucOccupation;//保存当前已占用的轴
	uint8 spindleFollowAxis;//主轴跟随的轴号
	struct SSingleAxis* pSSpindleAxis;   //主轴的单轴结构体指针  初始化时指向 g_SSingleAxisArray

	double dSpindleSpeed;			//主轴转速  r/min
	double dSpindleLineSpeed;		//主轴线速度 m/min
	double dLimitedSpindleSpeed;	// G50 限制的主轴转速(r/min)

	float fPreciseStopDegree;//准停角度
	bool toDisableAxis;   //轴停下轴取消使能主轴
	int8 cSpindleClampStatus;	//主轴锁紧状态
	bool toUpdateG96SpindleSpeed;  //更新G96的主轴转速  2023.5.13 朱稳中
	double dSpeedPulseRemainder; //zxr: 速度模式有符号小数脉冲余量，各主轴对象独立保存
	// zxr: 各主轴独立保存准停反馈及有界计数，不改变持久参数
	int64 lPreciseStopLastEncoder;
	uint16 usPreciseStopWaitTicks;
	uint16 usPreciseStopStillTicks;
};


#ifdef SPINDLE_README
主轴插补存在的状态//在common.h中定义
#define INP_FINISHED			0		//插补完成
#define INP_INITIATION			1		//初始化
#define INP_RUNNING				4		//运行
#define INP_DECELERATE			6		//减速

主轴单轴结构体中变量说明
cArcDir					表示主轴的方向					ARC_CW  正转 （1）   ARC_ANTI_CW反转(-1)
dSpindleSpeed			表示主轴G97指令下的转速			单位(度 / ms)  (不带方向)
dFeedRate				表示主轴G96指令下的转速			单位(m / ms)	(不带方向)
dLimitedSpindleSpeed	表示G50指令限制的主轴最高转速	单位(度 / ms)	(不带方向)
//dCurrentSpeed 		    表示当前转速(G96G97单位相同)	单位(度 / ms)  (不带方向)

主轴结构体中的变量
dSpindleSpeed			表示主轴G97指令下的转速			单位(r / min)	(不带方向)				被C轴读取后置为 - 1(SPINDLE_INVALID_VAL)
dLimitedSpindleSpeed	表示G50指令限制的主轴最高转速	单位(r / min)	(不带方向)				被C轴读取后置为 - 1(SPINDLE_INVALID_VAL)
dSpindleLineSpeed		表示主轴G96指令下的转速			单位(m / min)   (不带方向)				被C轴读取后置为 - 1(SPINDLE_INVALID_VAL)
#endif // SPINDLE_README



/**********************************函数*******************************************/


//初始化主轴结构体
int8 spindleInit(struct _SSpindle* pSSpindle, struct SSingleAxis* pSSingleAxisPointer, uint8 ucFollowAxis);

//函数入口  主轴运动
int8 spindleMotion(struct _SSpindle* pSSpindlePointer);

//恒转速
int8 constVelMotion(struct _SSpindle* pSSpindlePointer);

//恒线速度
int8 constLineVelMotion(struct _SSpindle* pSSpindlePointer);

//主轴减速停
int8 commonStopMotion(struct _SSpindle* pSSpindlePointer);

//主轴准停
int8 preciseStopMotion(struct _SSpindle* pSSpindlePointer);

//检测主轴平均转速
int8 spindleSpeedTest(struct _SEncoderTest* pSpindleTest, struct _SSpindle* pSSpindlePointer);

/***********************************接口函数****************************************/
//设置主轴方向接口
int8 setSpindleDir(struct _SSpindle* pSSpindlePointer, int8 cDir);

//主轴启动接口
int8 spindleBegin(struct _SSpindle* pSSpindlePointer);

//主轴停接口
int8 spindleCommonStop(struct _SSpindle* pSSpindlePointer);

//主轴准停接口
int8 spindlePreciseStop(struct _SSpindle* pSSpindlePointer, float* preciseStopDegree);

//主轴立即停接口
int8 spindleStopNow(struct _SSpindle* pSSpindlePointer);

//根据当前主轴的状态和主轴转速计算当前点的理论转速(r/ms 带方向)
// double calTargetSpindleSpeed(struct _SSpindle* pSSpindlePointer);
int8 calTargetSpindleSpeed(struct _SSpindle* pSSpindlePointer, double* pTargetSpeed);

//返回G97恒转速控制模式下的主轴转速(r/ms)不带方向
double getG97AbsSpindleSpeed(struct _SSpindle* pSSpindlePointer);

//从插补结构体中读取主轴转速和指令类型
int8 readCommandAndSpindleSpeed(struct SInterpolationPre* pSPrePointer, struct _SSpindle* pSSpindlePointer);

//设置主轴转速
int8 setSpindleSpeed(struct _SSpindle* pSSpindlePointer, double dSpindleSpeed, uint16 type);

////使能主轴
//int8 setSpindleEnabled(struct _SSpindle* pSSpindlePointer,bool isEnable);

/* 仅运动周期调用；后台必须通过主轴 IO 槽位请求，不得直接操作输出。 */
int8 setSpindleEnabled(struct _SSpindle* pSSpindlePointer, bool isEnable);

/* 仅运动周期调用，保留 M15 的原运行模式语义。 */
int8 IOSetSpindleEnabled(struct _SSpindle* pSSpindlePointer, bool isEnable);

//主轴停接口 //朱稳中 2023.5.19
int8 spindleStopAPI(struct _SSpindle* pSSpindlePointer, float* preciseStopDegree, bool bForcePreciseStop);

/*************************************************
  函数名称：processSpindleStop
  函数功能：周期执行公共停止，不经过自动使能入口
  输入参数：pSSpindle - 主轴或动力刀控制结构
  输出参数：更新控制状态
  返 回 值：1 - 完成；0 - 等待；负值 - 异常
  备    注：苏向标 2026-10-14
****************************************/
int8 processSpindleStop(struct _SSpindle* pSSpindle);
/*************************************************
  函数名称：processSpindleRequests
  函数功能：周期路径接收停止请求并选择原有插补状态
  输入参数：pSSpindle - 主轴或副轴结构
  输出参数：更新当前主轴控制数据
  返 回 值：无
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
void processSpindleRequests(struct _SSpindle* pSSpindle);
/*************************************************
  函数名称：isSpindleStopping
  函数功能：判断是否存在未完成或失败的停止请求
  输入参数：pSSpindle - 主轴或动力刀控制结构
  输出参数：无
  返 回 值：TRUE - 停止未完成；FALSE - 无待完成停止
  备    注：苏向标 2026-10-14
****************************************/
bool isSpindleStopping(const struct _SSpindle* pSSpindle);
/*************************************************
  函数名称：isSpindleStopFinished
  函数功能：判断复位需要等待的主轴收尾是否完成
  输入参数：pSSpindle - 主轴或动力刀控制结构
  输出参数：无
  返 回 值：TRUE - 完成或无此主轴；FALSE - 未完成
  备    注：苏向标 2026-10-14
****************************************/
bool isSpindleStopFinished(const struct _SSpindle* pSSpindle);

#endif

