#ifndef __G1_H
#define __G1_H

#include "common.h"


#define ROTATION_AXIS_DIST_CAL_MODE  0  //旋转轴距离计算方式   0  短距离优先     1 根据符号

/* 每通道 G84 同步状态，随既有 SChannel 动态内存生存；中断内不申请释放。 */
struct SG84Follow
{
    int64 m_lLastEncoder; /**< 上次消费的实际编码器累计计数，相邻攻丝段保留，退出或复位清除。 */
    double m_dStartC; /**< 建立参考时的 C 指令坐标，度。 */
    double m_dStartZ; /**< 建立参考时的 Z 指令坐标，mm。 */
    double m_dActualC; /**< 实际有符号编码器增量累计的 C 相位，度。 */
    double m_dZPerDegree; /**< Z/C 有符号运动比，mm/度；暂停重规划不改参考。 */
    double m_dEncoderToDegree; /**< 每个真实四倍频计数对应的角度。 */
    uint32 m_unNoFeedbackTicks; /**< 指令与反馈不同步且无计数变化的连续拍数。 */
    uint32 m_unWaitTicks; /**< C 规划结束后的有限等待拍数。 */
    uint16 m_usCountsPerRev; /**< 建立参考时的每圈计数，运行中不允许修改。 */
    uint8 m_ucEncoderMap; /**< 当前通道主轴编码器物理映射，只允许 TIM1/TIM2。 */
    uint8 m_ucSettledTicks; /**< 终点连续确认拍数，防止提交末拍即交接。 */
    bool m_bActive; /**< 本段参考有效；普通段、人工恢复移动及复位清除。 */
};

struct SG1     // 2022.3.14    G1 执行结构体
{
	struct SChannel* pSChannel; // 通道指针  (父)

	uint32 unCommandType;			//指令类型
	VAL_STRUCT_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12

	struct SInput SUserData;	// 用户（输入）参数
	struct SPlan SPlanData;			//速度规划参数

	/******************* 运动参数  与点有关的部分 *****************/
	SVector SStartVector;			//×起点矢量（单位向量） √位移分量
	SPointDouble SStartPoint;		//起点，单位：mm
	SPointDouble SEndPoint;			//终点，单位：mm
	float  fCylinderRadius; // 圆柱半径 mm  2023.3.29 朱稳中
	//int8 cArcDir;					//圆弧方向，顺时针=1，逆时针=-1
	//double dRadius;					//半径，单位：mm
	//SPointDouble SCenterPoint;		//圆心，单位：mm（相对起点
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

	bool bUseSRunFeedRatio;	//是否使用SRun倍率
	bool bIsUserData; //使用用户数据    2022.12.6

	double dPulseToMm[AXIS_NUM_MAX];	//脉冲当量的倒数  各个轴  XYZ ABC
	bool bReCalculateDec;	//减速到软限位点的减速度重算标志位 2024/4/11 洪冠鹏
	double dReCalculateVe;  //根据软限位点重算末速度 2024/4/11 洪冠鹏

	double dG84Ratio; //用于G84倍率 2025.11.15
    struct SG84Follow SG84Follow; /**< 本通道本段的实际 C 相位跟随状态。 */
};

//extern struct SG1 g_SG1;

void SG1LoadSysPara(struct SG1* pSG1);

//修正旋转轴终点
int8 modifyRotationAxisPoint(double* pdStartPoint, double* pdEndPoint, double* pdVector, int nArraySize, uint8* ucMapArray, VAL_STRUCT_TYPE* ucValStatus, VAL_STRUCT_TYPE	ucRelaVal);

//计算长轴和距离分量
int8 calVectorAndDistance(double* pdStatPoint, double* pdEndPoint, double* dDistance, double* pdOutputVector, int nArraySize, VAL_STRUCT_TYPE ucValStatus);

// 计算G84长轴和距离分量
int8 calcG84VectorAndDistance(double *pdStatPoint, double *pdEndPoint, double *pdDistance, double *pdOutputVector, int nArraySize, VAL_STRUCT_TYPE ucValStatus, double dMotionRatio);

//计算直线插补的坐标点函数
int8 calculateLinearInterpolationPoint(double* pdStartPoint, double* pdVector, double dDeltaL, double* pdOutPutPoint, int nArraySize);

//暂停初始化函数
int8 prepareToPauseG1(struct SG1* pSG1Pointer, double dVmin, double dDec, bool bWriteBackFlag);

//从Input参数赋值到G1结构体
int8 loadSG1DataFromInput(struct SG1* pSG1Pointer, struct SInput* pSInput);

//直线运动插补
int8 motionLine(struct SG1* pSG1Pointer, bool bPauseOn, bool bRatioOn);

//暂停后加载参数赋值到G1结构体
int8 loadSG1DataAfterPause(struct SG1* pSG1Pointer, SPointDouble SStartPoint, SPointDouble SEndPoint, bool bIsG0Move, struct SInput* pSInput);

// 检查G1哪根轴最先触发软限位
int8 checkG1FirstAxisSoftLimit(struct SChannel *pSChannel, SPointDouble SStartPoint, SPointDouble SMaxVal, SPointDouble SMinVal);

#endif
