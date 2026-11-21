#ifndef _TOOLAXIS_H
#define _TOOLAXIS_H

#include "common.h"

#define TOOLAXIS_TEST 0     //刀具轴运动测试（刀具轴运动开关）
#define TOOLAXIS_DEBUG_32 0 //刀具轴调试_32模式（屏幕显示）
#define TOOLAXIS_DEBUG_VS 0 //刀具轴调试_VS模式（输出文本）

//规划方式
//1：对剩余距离进行速度规划，时间节点不圆整。若有加速周期则加速，若有匀速周期则匀速，若有减速周期则减速
//1：对剩余距离与未来新增距离进行速度规划，时间节点不圆整。若有加速周期则加速，若有匀速周期则匀速，若有减速周期则减速

//刀具轴轴名
#define TOOL_AXIS AXIS_B
//刀具轴编码器号
#define TOOLAXIS_ENCODER 2

//ucRunMode的值  表示主轴处于不同的模式
#define TOOLAXIS_SINGLE_MODE 0     //单轴模式（空闲）
#define TOOLAXIS_FOLLOW_MODE 1     //跟随模式
#define TOOLAXIS_COMMON_MODE 2     //恒转速模式
#define TOOLAXIS_COMMONSTOP_MODE 3 //减速停模式

#define PARA_EDITABLE 0
#define WAIT_PERIOD 100 //跟踪误差测试前等待周期
#define TEST_PERIOD 50  //跟踪误差测试周期

//跟随方式
#define PLANNING_METHOD 2 //0：1：自适应插补2：自适应插补（二次速度规划）
#define ENCODER_FOLLOW 0  //0：跟随主轴坐标值	1：跟随主轴编码器值

#define INP_STAGE_1 11 //刀具轴跟随阶段1（速度过渡）
#define INP_TEST_1 12  //刀具轴编码器滞后测试阶段1（等待刀具轴编码器达到目标转速）
#define INP_TEST_2 13  //刀具轴编码器滞后测试阶段2（计算刀具轴编码器滞后角度）
#define INP_STAGE_2 14 //刀具轴跟随阶段2（角度过渡）
#define INP_WAIT 15

#define TOOLAXIS_FOLLOWDIR 1 //
//角度过渡
typedef struct _SToolAxis
{
    uint32 unTime; //485测试用

    struct SChannel* pSChannel; //通道指针

    double dCommandRatio; //指令转速比（指令原始数据）
    double dCommandAngle; //指令角度（指令原始数据）

    double dLastCommandRatio; //上一指令转速比
    double dLastCommandAngle; //上一指令角度

    double dTotalTwistAngle;   //当前总扭转角度
    double dCurTwistAngle;     //当前扭转角度
    int8 cTwistCommand;        //扭转指令
    double dCommandTwistAngle; //指令扭转角度

    double dLagAngle; //滞后角度（伺服驱动器存在响应时间导致）

    double dSpindleSpeed;
    double dLastSpindleSpeed;

    uint32 ucT;

    bool bFollowed;

    struct SSingleAxis* pSToolAxis; //单轴结构体指针

} SToolAxis;

extern SToolAxis g_SToolAxisInfo;
extern struct SEncoder g_SToolAxisEncoder;

//初始化刀具轴模块
int8 initToolAxis(struct SChannel* pSChannel, struct SSingleAxis* pSSingleAxisPointer);

//初始化刀具轴参数
void initToolAxisPara();

//刀具轴运动
int8 toolAxisMotion(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo, struct SEncoder* pSSpindleEncoder);

//刀具轴跟随模式
void toolAxisFollowMotion(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo,
                          struct SEncoder* pSSpindleEncoder);
void toolAxisFollowMotion2(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo,
                           struct SEncoder* pSSpindleEncoder);
void toolAxisFollowMotion3(SToolAxis* pSToolAxisInfo, struct _SSpindle* pSSpindleInfo,
                           struct SEncoder* pSSpindleEncoder);

//刀具轴减速停模式
int8 toolAxisCommonStop(SToolAxis* pSToolAxisPointer);
void toolAxisCommonMotion(SToolAxis* pSToolAxisInfo);

double calcCurCurTwistAngle();

//速度规划
int8 preparePlanningTNewAddForToolAxis(struct SPlan* pSPlanPointer);
int8 preparePlanningTNewAddForToolAxis_1(struct SPlan* pSPlanPointer);
int8 preparePlanningTNewAddForToolAxis_2(struct SPlan* pSPlanPointer);

//计算运动角度
double nextStepPlanningAndDistReAvgToolAxis(uint32 unK, struct SPlan* pSPlanPointer);
double nextStepPlanningAndDistReAvgToolAxis_1(struct SPlan* pSPlanPointer);
double nextStepPlanningAndDistReAvgToolAxis_2(struct SPlan* pSPlanPointer);

double calcAllowVmax(struct SPlan* pSPlanPointer);

//跟随判断
bool checkFollowed(double dDeltaL, struct SPlan* pSPlanPointer);

//获取值对应的角度
double getEncoderPosByValue(int64 lEncoderValue, uint16 usEncoderCirclePulseNum);

//计算目标编码器的角度[0-360)
double getEncoderPos(uint8 ucEncoderNum);

//获取当前主轴编码器角度
double getSpindleEncoderPos(uint8 ucChannelNum);

//获取当前刀具轴编码器角度
double getToolAxisEncoderPos(uint8 ucChannelNum);

//获取刀具轴编码器目标角度
double getToolAxisEncoderTargetPos(double ucFollowAngle, bool bFlag);

#if STM32_MODE
void toolAxisScreenPrintInit();

void toolAxisScreenPrint();

void output485();

void outputEncoderDelta(uint8 ucEncoderNum);

void outputEncoderDir(uint8 ucEncoderNum);

void outputEncoderPos(uint8 ucEncoderNum);

void outputIdealDelta(uint8 ucAxisNum);

void outputIdealDir(uint8 ucAxisNum);

void outputIdealPos(uint8 ucAxisNum);

void outputNextPulse();
#else
void outputVS();
void outputVS_2();

#endif

#endif // !_TOOLAXIS_H
