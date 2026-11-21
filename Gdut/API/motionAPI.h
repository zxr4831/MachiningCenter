#ifndef __MOTIONAPI_H
#define __MOTIONAPI_H

#include "common.h"
#include "Spindle.h"
#include "G32.h"
#include "G33.h"

#define GCODE_GROUP_MOTION			1 //(运动组G0 G1 G2 。。。
#define GCODE_GROUP_PLANE			2 // 平面选择G17 G18 G19 
#define GCODE_GROUP_TOOLCOMP		3 // 刀补 G40 G41 G42
#define GCODE_GROUP_WORKPIECE		4 //偏移G54 G55 。。。
#define GCODE_GROUP_SPINDLE_SPEED	5 //主轴速度 G96 G97
#define GCODE_GROUP_FEED_TYPE		6 //(进给G98 G99
#define GCODE_GROUP_

/*  定义IOCommand.h
#define MCODE_GROUP_NUM					3 //M代码组数
#define MCODE_GROUP_SPINDLE				0 //主轴 M3 M4 M5
#define MCODE_GROUP_FUILD				1 //冷却液 M8 M9
#define MCODE_GROUP_CHUCK				2 //夹头M10、M11
*/



/********************************状态接口*******************************/

//获取当前真实速率0~100
int getRealFeedRatio(uint8 ucChannelNum);

//获取当前进给速度
double getCurrentFeedRate(uint8 ucChannelNum, uint8 ucAxis);

//获取当前真实进给速度
double getCurrentRealFeedRate(uint8 ucChannelNum, uint8* pMaxAxis);

//获取剩余距离
double getCurLeftDist(uint8 ucChannelNum, uint8 ucAxis);

//获取各轴未规划的脉冲数
int32 getAxisAccmulate(uint8 ucChannelNum, uint8 ucAxis);

//获取G代码状态
int getGCodeStatusAPI(uint8 ucChannelNum, uint8 ucGroupNum);

//获取M代码状态
int getMCodeStatusAPI(uint8 ucChannelNum, uint8 ucGroupNum);

//获取单轴状态
int8 getSingleAxisStatus(uint8 ucChannelNum, uint8 ucAxis);

/********************************运动接口*******************************/

//设置G0减速停 立即停
int8 setSingleAxisStop(uint8 ucChannelNum, uint8 ucAxis, uint8 mode, double dDec);

//设置G0的速度参数
int8 setSG0SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucAxis, uint8 ucType);

//设置G1的速度参数
int8 setSG1SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucType);

//设置G2的速度参数
int8 setSG2SpeedPara(double dVal, uint8 ucChannelNum, uint8 ucType);

//设置当前的速度参数
int8 setCurSpeedPara(double dVal ,uint8 ucChannelNum, uint8 ucAxis, uint8 ucType);

//将SG0 SG1 SG2的SUserData（自定义参数）或SPreProcess（缓存区）的SUserData的数据写入SInput结构体
/*自定义表：
struct|ucAccVelPlanType |ucDecVelPlanType |        dVs        |      dFeedrate     |     dPrimalVe     |dFeedRatio|    dAcc     |    dDec     |  SEndPoint  |
------|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
   |X |VEL_PLAN_S4_CURVE|VEL_PLAN_S4_CURVE|   200.0/60/1000   |  18000.0/60/1000   |   200.0/60/1000   |   1.0    |    0.004    |    0.004    |{0,0,0,0,0,0}|
   |--|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
   |Y |VEL_PLAN_S4_CURVE|VEL_PLAN_S4_CURVE|   200.0/60/1000   |   3000.0/60/1000   |   200.0/60/1000   |   1.0    |    0.002    |    0.002    |{0,0,0,0,0,0}|
   |--|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
SG0|Z |VEL_PLAN_S4_CURVE|VEL_PLAN_S4_CURVE|   200.0/60/1000   |  18000.0/60/1000   |   200.0/60/1000   |   1.0    |    0.002    |    0.002    |{0,0,0,0,0,0}|
   |--|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
   |A |VEL_PLAN_V3_CURVE|VEL_PLAN_V3_CURVE|200.0/60/1000*360.0|3000.0/60/1000*360.0|200.0/60/1000*360.0|   1.0    |0.0001*360.0 |0.0001*360.0 |{0,0,0,0,0,0}|
   |--|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
   |B |VEL_PLAN_S4_CURVE|VEL_PLAN_S4_CURVE|   200.0/60/1000   |   3000.0/60/1000   |   200.0/60/1000   |   1.0    |    0.05     |    0.05     |{0,0,0,0,0,0}|
   |--|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
   |C |VEL_PLAN_V3_CURVE|VEL_PLAN_V3_CURVE|200.0/60/1000*360.0|3000.0/60/1000*360.0|200.0/60/1000*360.0|   1.0    |0.00005*360.0|0.00005*360.0|{0,0,0,0,0,0}|
------|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
  SG1 |VEL_PLAN_S4_CURVE|VEL_PLAN_S4_CURVE|   200.0/60/1000   |  10000.0/60/1000   |   200.0/60/1000   |   1.0    |    0.001    |    0.001    |{0,0,0,0,0,0}|
------|-----------------|-----------------|-------------------|--------------------|-------------------|----------|-------------|-------------|-------------|
  SG2 |VEL_PLAN_S4_CURVE|VEL_PLAN_S4_CURVE|   200.0/60/1000   |  10000.0/60/1000   |   200.0/60/1000   |   1.0    |    0.001    |    0.001    |{0,0,0,0,0,0}|
*/
int8 writeSInputAPI(uint8 ucChannelNum, struct SInput* pSUserData, int8 cType);

//G123接口函数
int8 applyMotionParaAPI(uint8 ucChannelNum, struct SInput* pSInputPointer, int8 cType);  //0 快速运动  1直线  2顺圆 3逆圆

//减速度调整
int8 adjustDec(uint8 ucChannelNum, struct SInput* pSInputPointer, int8 cType, uint8 ucAxis, int8 cUserDataCount);

//G32接口函数
int8 applyG32MotionParaAPI(struct _SG32* pSG32);

//G33接口函数
int8 applyG33ParaAPI(struct SInterpolationPre* pSPreProcess, struct G33* pSG33);

//返回开始随动所需的长度(开始随动后，才能获得)
double returendStartFollowDisAPI();

//获取运动状态
int8 getMotionStatus(uint8 ucChannelNum);

//返回g_RunMode
enum RUNMODE getRunMode();

/*******************************主轴接口*********************************/

//获取主轴状态
int8 getSpindleStatus(uint8 ucChannelNum);

//设置主轴模式 G96 G97,开启中断前，与机床主轴正转和反转按钮有关
int8 setSpindleModeAPI(uint8 ucChannelNum, int8 cMode);

//插补中设置主轴模式 G96 G97，
int8 setSpindleModeInInterpolation(uint8 ucChannelNum, int8 cMode);

// 获取主轴编码器转速
double getSpindleEncoderSpeed(uint8 ucChannelNum);

// 获取主轴理论转速
double getSpindleSpeedAPI(uint8 ucChannelNum);

//设置主轴方向接口
int8 setSpindleDirAPI(uint8 ucChannelNum, int8 cDir);

//主轴启动接口
int8 spindleBeginAPI(uint8 ucChannelNum);

//主轴停接口
int8 spindleCommonStopAPI(uint8 ucChannelNum);

//主轴准停接口
int8 spindlePreciseStopAPI(uint8 ucChannelNum);

//主轴立即停接口
int8 spindleStopNowAPI(uint8 ucChannelNum);

//设置主轴转速
int8 setSpindleSpeedAPI(uint8 ucChannelNum, double dSpindleSpeed, uint16 type);

//获得该通道主轴发送脉冲和主轴编码器的偏差
int16 getSpindlePulseDeviation(uint8 ucChannelNum);

////使能主轴
//int8 setSpindleEnabledAPI(uint8 ucChannelNum,bool isEnable);

int8 setSpindleEnabledAPI(uint8 ucChannelNum, bool isEnable);

#endif // !__COORDAPI_H
