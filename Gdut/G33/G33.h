#ifndef __G33_H
#define __G33_H
#include <math.h>
#include <stdlib.h>
#include "common.h"
#include "Spindle.h"

/**
Copyright (c) 广东工业大学
@file G33.h
@author 郭强
@version 22.0
@date 2023-06-08
@brief G33Z向攻牙循环
*/

/*  插桩信息宏  用于G33调试打印运行状态；*/
#define G33ProcLog_DEBUG 0

/*  插桩信息宏  用于G33调试打印运行数据，控制台最多打印不到10000行信息*/
#define G33_MOTION_DATA_DEBUG 0

/*  模拟编码器OR真实编码器   */ //当宏为"1"时使用真实编码器上机运行
                                //当宏为"0"时使用模拟编码器仿真
#define G33_USE_REAL_ENCODER 1

#define G33_TIMERANGE 10000 //超时阈值

//编码器数值（****模拟编码器用****）
extern int64 g_lSpindleEncoderValue;

//测试仿真用Z轴浮动量（****模拟仿真用****）
extern double g_dZAxisFloatingLength;

//主轴实际转速
extern double g_dActualSpindleSpeed;

enum G33MotionStates //G33运行状态枚举类型
{
    //正向阶段
    SPINDLE_ACC_BEFORE_ZAXIS_FEEDING, //Z轴进刀前主轴加速（多头）
    ZAXIS_ACC_BEFORE_FOL_UP,          //Z轴随动前加速（包括多头螺纹等待合适的编码器数值）
    ZAXIS_FOL_UP_TAPPING,             //Z轴随动攻丝（此阶段主轴匀速，Z轴随动）
    SPINDLE_DEC_TO_STOP_BEFORE_REV,   //主轴反转前减速停(2023.5.11 郭强)

    //暂停阶段
    PAUSE_BEFORE_REVERSAL, //暂停阶段(2023.5.11 郭强)

    //反向阶段
    SPINDLE_REVERSE_RETURN, //主轴反转退刀(2023.5.11 郭强)

    //正反向均有(待删)
    ZAXIS_FOL_UP_DEC_AND_RETURN, //Z轴随动减速及退刀（Z轴超过理论终点后主轴减速，然后反向加速，Z轴一直随动最终到达避空距离内开始减速至起刀点）

    //反向阶段
    SPINDLE_DEC_TO_STOP_AFTER_ZAXIS_RETURN //退刀完成后主轴减速停
};

struct G33
{
    VAL_STRUCT_TYPE ucRelaVal;   //相对值标识变量
    VAL_STRUCT_TYPE ucValStatus; //有效值（轴）类型

    uint8 ucAxis[2];            //执行插补的轴号
    struct SChannel* pSChannel; // 通道指针  (父)
    struct SPlan SPlanData;     //速度规划参数，此结构体用于速度规划
    double dCurrentSpeed;       //当前速度 mm/ms

    //G33 运行时的规划段起点终点向量
    SPointDouble SStartPoint; //起点，单位：mm
    SPointDouble SEndPoint;   //终点，单位：mm
    SVector SStartVector;     //向量

    int8 G33RotateAxis; //G33 旋转轴  可选A轴或C轴

    int8 G33TappingMode; //G33攻牙模式 0:刚性攻牙 1:柔性攻牙

    //记录G33运行时的主轴转速，以便退刀时反向加倍转速退出（2023/3/23郭强）
    double dG33SpindleSpeed; //（r/min）

    uint8 ucL; // 多头螺纹头数1-99

    uint8 ucAlreadyL; //已加工头数

    double dF; //螺纹螺距

    enum G33MotionStates G33MotionState; //G33运行状态

    int8 cTappingSpindleDir; //记录攻丝主轴转向

    bool bPreFinishedFlag; //各阶段预处理完成标志位

    bool bZAxisFeedSignal; //多头螺纹攻丝时Z轴进给信号

    double dZAxisStartEncoderRelValue; //保存计算出的Z轴启动时主轴编码器理论相对值

    bool bG33AccPreFirstTime; //G33加速阶段反算加速度第一次，用于添加最大启动误差补偿位移

    double dSpindleDecPointPreDist; //G33主轴减速并反转时的减速点距G代码起点的长度

    bool bSpindleRevBegin; //主轴反转开始

    double dSpindleRevCompZAxisCumulaDis; //主轴反转补偿期间Z轴累积距离
    bool bSpindleRevCompEnd;              //主轴反转补偿结束标志位
    bool bSpindleRevCompEndFirstTc;       //主轴反转补偿结束后的第一个周期

    //G33加速阶段最终加速度，减速度
    double dG33Acc, dG33Dec;

    //G33 G代码起点终点信息
    SPointDouble G33StartPoint; //起点，单位：mm
    SPointDouble G33EndPoint;   //终点，单位：mm

    //G33 G代码中的起点方向向量信息 （用于反转补偿规划时更改方向向量）
    SVector G33StartVector;

    //G33Z轴启动前的实时点
    SPointDouble G33StartRealTimePoint;

    //G33模拟运行时的过程实时点
    SPointDouble G33AnalogRealTimePoint;

    //G33主轴反转前暂停时间(ms)//2023.5.8 郭强
    uint32 unG33SpindlePauseTimeBeforeRev;

    // zxr: 各通道独立记录随动阶段连续无反馈次数，计划停留不累计
    uint32 unFollowTimeOut;
};

// //初始化G33全局变量
bool InitG33GlobalVariable(struct SChannel* pSChannel);

int8 G33initStructByTool(struct SToolCompensation* pSToolCompPointer, struct SInterpolationPre* pSPrePointer);

//初始化G33插补平面
int8 chooseInterpPlaneForG33(struct G33* pG33, struct SInterpolationPre* pSPrePointer);

//检测向量相同
bool checkSameVector(struct G33* pSG33, SVector SVector1, SVector Svector2);

/*********************************模拟编码器用*************************************/
//获取编码器数值（分为真实编码器和模拟编码器）
int64 getSpindleEncoderValue(struct SEncoder* pSpindleEncoder);

//模拟主轴编码器计数（****模拟编码器用****）
void AnalogSpindleEncoderCount(struct G33* pSG33, int16 sSpindlePulseValue, struct SEncoder* pSpindleEncoder);

//仿真计算Z轴浮动量
bool calcZAxisFloatingLength(struct G33* pSG33, int32 nZAxisDeltaPulse, int32 nCAxisDeltaPulse);
/***********************************************************************************/

//G33主轴超速检查（G33底层运动内部调用）
int8 G33SpindleOverSpeedCheck(struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//G33主轴超速检查（外部调用）
int8 CheckG33SpindleOverSpeed(struct SChannel* pSChannel);

//初始化G33结构体
int8 initSG33Data(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33,
                  struct SEncoder* pSpindleEncoder);

//获取主轴真实转速
double getSpindleRealSpeed(struct G33* pSG33);

//计算多头螺纹Z轴启动时编码器值
double calcZAxisStartEncoderRelValue(struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//主轴加减速完成后改变G33运行状态
int8 changeG33StateAfterSpindleAccOrDec(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle,
                                        struct G33* pSG33);

//单轴指定速度和位移（Vs Vmax Ve）速度规划,指定速度类型为S4(待删)
int8 singleAxisInitAndPrePlanS4(uint8 ucAxis, double dDistance, double dVs, double dVmax, double dVe);

//Z轴进刀前主轴加速
void motionSpindleAccBeforeZAxisFeeding(struct G33* pSG33);

//计算加速阶段最大补偿位移
double calcMaxStartErrCompenDis(struct G33* pSG33);

//Z轴加速预处理
int8 ZAxisAccPre(struct G33* pSG33, struct SEncoder* pSpindleEncoder, int8 cSpindleRevFlag);

//计算主轴编码器一个周期增量对应Z轴新增位移
double calcZAxisNewAddDist(struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//运动输出
int8 newAddDistOutPut(struct G33* pSG33);

//Z轴随动前加速
int8 motionZAxisAccBeforeFolUp(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//估算主轴减速Z轴超出距离
double calcSpindleDecZAxicSurpassLength(struct _SSpindle* pSSpindle, struct G33* pSG33);

//计算主轴反转规划时相对G33起点要走的距离
int8 calcSpindleDecPointPreDist(struct _SSpindle* pSSpindle, struct G33* pSG33);

//判断Z轴越过主轴反转规划标志点
bool judgeZAxisAcrossEndPoint(struct G33* pSG33);

//Z轴随动攻丝（此阶段主轴匀速，Z轴随动）
int8 motionZAxisFolUpTapping(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//主轴反转前减速停（2023.6.8郭强，增加输入参数）
int8 motionSpindleDecToStopBeforeRev(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//主轴反转前暂停
int8 motionPauseBeforeSpindleReversal(struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//Z轴随动运动(异常反向直接反向)
int32 ZAxisFolUpMove(struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//Z轴不完全随动运动（异常反向不直接反向，要走完剩余距离）
int32 ZAxisNotExactlyFolUpMove(struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//计算主轴减速过程浮动量
//int8 calcSpindleDecCompenL();

//主轴反转补偿规划（初始化结构体+速度规划和平摊）
int8 spindleRevCompPlan(struct G33* pSG33);

//主轴反转补偿运动
int8 spindleRevCompMoveOutPut(struct G33* pSG33);

//仿真计算G33超出距离
void calcG33SurpassLength(struct _SSpindle* pSSpindle, struct G33* pSG33);

//根据长度起点方向向量计算目标点
SPointDouble calcPoint(struct G33* pSG33, SPointDouble SStartPoint, SVector SStartVector, double dLength);

//主轴减速及退刀阶段，反转需要补偿的运动函数
int8 needReveComMove(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//Z轴随动减速和退刀(待删)
int8 motionZAxisFolUpDecAndReturn(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//主轴反转退刀
int8 motionSpindleReverseAndReturn(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder);

//退刀完成后主轴减速停
int8 motionSpindleDecToStopAfterZAxisReturn(struct G33* pSG33);

//G33运行过程中的复位初始化
int8 G33ResetInit(struct SChannel* pSChannel);

//G33动作
int8 G33Action(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33,
               struct SEncoder* pSpindleEncoder);

//G33运动接口函数
int8 MotionG33(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33,
               struct SEncoder* pSpindleEncoder);

#endif
