#ifndef __ERROR_COMPENSATION_H
#define __ERROR_COMPENSATION_H
#include "common.h"

//VS 螺距补偿测试,模拟加载FLASH
#define SCREWCOMPTESTCODE 0
//脉冲打印
#define PULSESUM_PRINT 0
//补偿绝对值打印
#define ABSCOMPVAL_PRINT 0


#define NO_SCREW_COMP			0	//没有补偿
#define SCREW_AND_REVCLEARANCE	1	//螺距误差补偿+ 反向间隙补偿
#define TWOWAY_SCREW_COMP		2	//双向螺距误差补偿
//螺距补偿功能选择
#define ERROR_COMP_TYPE NO_SCREW_COMP //补偿类型 0：没有补偿  1： 螺距误差补偿+ 反向间隙补偿  2：双向螺距误差补偿



//状态宏定义
#define ERROR_COMPENSATION_INIT     0 //初始阶段 需要给定方向
#define ERROR_COMPENSATION_OFF		1 //反向误差补偿关闭
#define ERROR_COMPENSATION_ON		2 //反向误差补偿开启
#define ERROR_COMPENSATION_MODIFY	3 //重新设置反向误差或速度参数后 待currentT回到0时，才能重新规划 使用新的补偿距离


typedef struct    // 反向间隙误差补偿结构体
{
	uint8 ucStatus;// 状态   
	int8 cCurrentDir; // 当前方向  使用上个周期的pulse值不能准确的知道方向  需要记录
	int8 cDir;        // 反向间隙方向   与回零方向有关
	double dCurDist; //当前已补偿值
	struct SPlan SPlanData;//速度规划结构体
}SRevClearanceComp;




//初始化单轴固定参数
int8 initAxisRevData(SRevClearanceComp* pSRevClearanceComp);

//更新单轴反向间隙补偿的速度规划（用系统参数）
int8 updateSingleAxisRevPlan(uint8 ucPhysicalAxis);

//初始化单轴反向间隙补偿  首次初始化的方向：与回零方向有关
int8 initSingleAxisRev(uint8 ucPhysicalAxis, int8 cDir);

//使能反向间隙补偿
int8 revCompEnabled(uint8 ucPhysicalAxis, bool bEnable);
/*******************************反向间隙API函数*************************************/

//初始化反向误差补偿 
int8 initAxisRevDataAPI();

//使能反向间隙补偿
int8 revCompEnabledAPI(uint8 ucChannelNum, uint8 ucAxis, bool bEnable);//回零前必须关闭使能

//回零后  初始化单轴反向间隙补偿接口函数
int8 initRevCompAfterHomeAPI(uint8 ucChannelNum, uint8 ucAxis);

//距离参数编辑后  初始化单轴反向间隙补偿接口函数
int8 initRevCompAfterDistEditedAPI(uint8 ucPhysicalAxis);

//初始化（所有）物理轴反向误差循环接口
int8 initSingleAxisRevLoopAPI();

// zxr: 停止时保持补偿进度，等待新的运动方向
void holdRevCompensationAPI(uint8 ucPhysicalAxis);

//获取物理轴补偿值
int16 getCompensationPulseAPI(uint8 ucPhysicalAxis, int8 cDir);

//只读检查物理轴反向间隙补偿规划是否完成
bool isRevCompFinishedAPI(uint8 ucPhysicalAxis);



/*******************************螺距补偿*************************************/

typedef struct
{
	bool bEnabled;		//当前状态   
	int16 sCurSeg;		//当前区间号码
	int8 cCurError;		//当前区间螺距误差 (um) 
	double dDeltaError;	//移动1mm需要补偿的值（um)  螺距误差/螺距区间长度
	double dRecorder;	//记录器 记录当前未输出或多输出的脉冲
}SScrewComp;

int8 screwCompDisableAPI(uint8 ucChannelNum,uint8 ucAxis);//取消单表螺距误差补偿   2023.3.24 朱稳中

int8 initScrewComp(uint8 ucPhysicalAxis); // 初始化，并开始螺距补偿

int8 outPutAbsPointWithScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus);

double outPutAbsPointWithScrewCompForG0(struct _SRun* pSRun, double dTargetPoint, uint8 ucAxis, const uint8* ucAxisMap, double dPulseToMm);


/*******************************螺距补偿*************************************/
#if 0  // 系统参数
bool bBilateralScrewEnable[AXIS_PHYSICS_NUM_MAX];	//双向螺距误差使能位
uint8 ucBilateralScrewGap[AXIS_PHYSICS_NUM_MAX];		//螺距补偿间隔  mm
int8 cAheadScrewCompVal[AXIS_PHYSICS_NUM_MAX][SCREWDEVIATION_NUM_MAX];  //正向补偿值  um   走一个间隔需要补偿的量  走10mm 测得9.98mm  则填入20um
int8 cBackScrewCompVal[AXIS_PHYSICS_NUM_MAX][SCREWDEVIATION_NUM_MAX];  //反向补偿值  um   走一个间隔需要补偿的量  走10mm 测得9.98mm  则填入20um
int16 sScrewReferenceNum[AXIS_PHYSICS_NUM_MAX]; // 参考点位置号
float fScrewReferencePoint[AXIS_PHYSICS_NUM_MAX]; // 参考点坐标
#endif // 0

#define SCREWCOMP_AHEAD		1
#define SCREWCOMP_BACK		0

//双向螺距补偿状态
#define BILATERAL_SCREW_OFF		0
#define BILATERAL_SCREW_ON		1	//开启状态
#define BILATERAL_SCREW_ACTIVE	2	//补偿活跃状态

typedef struct
{
	//bool bEnabled;		//当前状态   
	//int16 sCurSeg;		//当前区间号码
	//int8 cCurError;		//当前区间螺距误差 (um) 
	//double dDeltaError;	//移动1mm需要补偿的值（um)  螺距误差/螺距区间长度
	//double dRecorder;	//记录器 记录当前未输出或多输出的脉冲

	uint8 ucStatus;		//当前状态   
	int8 cMoveDir;			//当前补偿方向
	int16 sCurSeg;		//当前区间号码

	int16 sAbsCompVal[2];	//正反向补偿值 （绝对值）

	double dCurAbsCompVal;	//当前补偿绝对值

	double dRecorder;	//记录器 记录当前未输出或多输出的脉冲

	double dDeltaComp[2];	//正反向移动1mm需要补偿的值（um)  螺距误差/螺距区间长度

	double dDivisionPoint;//当前的分割点

	//速度控制
	int8 cDir;	//方向
	float fVel; //速度
	float fAcc; //加速度
	double dLimitVmax;	//最大速度	//2023/2/28	陈敏聪

}SBilateralScrewComp;



//初始化双向螺距   需赋方向
int8 initSBilateralScrewComp(uint8 ucPhysicalAxis, int8 cDir);

//根据坐标获取区间号码
int16 getSegNum(double curPoint, uint8 ucPhysicalAxis);

//获取区间绝对值
int8 getCompVal(int16 sSeg, uint8 ucPhysicalAxis, SBilateralScrewComp* pSScrewComp);

//移动区间
int8 moveScrewSeg(int16 sTargetSeg, uint8 ucPhysicalAxis, SBilateralScrewComp* pSScrewComp);

//双表螺距补偿速度限制
double screwVelLimit(double  dTargetComp, SBilateralScrewComp* pSScrewComp);

// 输出
int8 outPutAbsPointWithBilateralScrewComp(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus);

double outPutAbsPointWithBilateralScrewCompForG0(struct _SRun* pSRun, double dTargetPoint, uint8 ucAxis, const uint8* ucAxisMap, double dPulseToMm);

int8 BilateralScrewComp(struct _SRun* pSRun, int nArraySize, const uint8* ucAxisMap);

/*******************************双表API函数*************************************/
int8 initSBilateralScrewCompAfterHomeAPI(uint8 ucChannelNum, uint8 ucAxis, int8 cDir);//回零后双表初始化

int8 bilateralScrewCompDisableAPI(uint8 ucChannelNum, uint8 ucAxis);//取消双表螺距误差补偿 2023.3.24 朱稳中
#endif
