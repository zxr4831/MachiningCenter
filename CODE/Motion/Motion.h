#pragma once
#ifndef __MOTION_H
#define __MOTION_H

#include "common.h"
#include "G0.h"
#include "Config.h"
#include "G1.h"



#if !STM32_MODE

#define HANDWHEEL_DIR	1
#define HANDWHEEL_DELTA	50

#endif // HANDWHEEL_SIMULATION

#define HANWHEELSIMULA_OFF		0		//手轮模拟关
#define HANWHEELSIMULA_BUILDING	1		//手轮模拟正在建立
#define HANWHEELSIMULA_ON		2		//手轮模拟开


//extern struct SInterpolationPre g_SPreProcess;

//插补初始化
bool interpolationInit(struct SInterpolationPre* pSPreProcess);

//插补初始化（加载系统参数之后）  
int8 interpolationInitAfterLoad(struct _SRun* pSRun, struct SInterpolationPre* pSPreProcess);

//软限位函数（外挂）
int8 softLimitForHandWheel(uint8 ucChannelNum);

//软限位函数（内嵌）
//int8 softLimit(struct SChannel* pSChannel, SPointDouble SStartPoint, SPointDouble SEndPoint, SPointDouble SMaxVal, SPointDouble SMinVal);
int8 softLimit(struct SChannel *pSChannel, SPointDouble SStartPoint, SPointDouble SEndPoint, SPointDouble SMaxVal, SPointDouble SMinVal, uint8 ucAxis);
#if ENCODER_MODE
//清除与手轮相关的参数
void clearHandWheelPara(struct SChannel* pSChannel);
#endif

//释放上一个指令的轴占用
int8 releaseLastCommandAxis(struct SInterpolationPre* pSPreprocess);

//清空脉冲增量缓存区
void initSRunPulseBuf(struct _SRun* pSRun, int8 cArrayNumMap);

////获取相对坐标
//SPointDouble getRelaPoint();

//理论绝对坐标生成SRun结构体数据
bool outPutAbsPoint(struct _SRun* pSRun, double* pdTargetPoint, int nArraySize, const uint8* ucAxisMap, double* dPulseToMmArray, VAL_STRUCT_TYPE* ucValStatus);

//计算重装载值 和 比较值 保存到数组中
void calculatePWMValue(struct _SRun* pSRun, const uint8* ucAxisMap, const uint8* ucPWMAxisMap, bool isCompFlag);

//计算脉冲数并检查范围
int16 calcPulseWithCheck(double dSpeed, double dPulseToMm, int16* pSpindlePulse);

//判断通道中是否是轴方向改变
bool isMotionChangeDirection(uint8 ucChannelNum);

//读取手轮的方向  转速
int8 handWheelSpeedTest(struct SChannel *pSChannel);

//初始化插补预处理结构体  (从刀补)
#if HANDWHEEL_SIM_DISTANCE_ENABLE
int8 judgeCanMoveBack(struct SToolCompensation* pSToolCompPointer);
#endif
int8 initInterpStructByTool(struct SToolCompensation* pSToolCompPointer, struct SInterpolationPre* pSPrePointer, int8 cInterpDir);

//插补预处理
int8 interpolationPre(struct SInterpolationPre* pSPreProcess);

//恢复加工函数
void recoveryToRun(struct SG1* pSG1Pointer);

//单轴减速停初始化
int8 singleAxisSlowDown(struct SSingleAxis* pSSingleAxisPointer, struct _SRun* pSRun, uint8* ucMapArray, G0_SIGN_TYPE* currentSign);

//单轴减速到软限位初始化
int8 singleAxisToLimit(struct SSingleAxis* pSSingleAxisPointer, struct _SRun* pSRun, uint8* ucMapArray, G0_SIGN_TYPE* currentSign);

//单轴减速停循环初始化
int8 singleAxisSlowDownLoop(struct SG0* pSG0Pointer, int nAxisNum);

//停止运行切换到IDLE
//int8 channelStopToIDLE(struct SChannel* pSChannel, bool bStopSpindle);
int8 channelStopToIDLE(struct SChannel* pSChannel);

//复位运动
void resetProgram(struct SChannel* pSChannel);

//报警后轴停函数
void stopAxisAfterError(struct SChannel* pSChannel);

//保存暂停点
int8 savePausePoint(struct SInterpolationPre* pSPreProcess);

//断屑功能
int8 ChipBreak(struct SChannel* pSChannel);

/* 余程回写后调用：主轴仍在停止时延后发布暂停，收尾后保存暂停点。 */
void handleSystemPause(struct SChannel* pSChannel);

//中断服务函数
void IRQHandler(struct SChannel* pSChannel);//中断服务函数

//报警后进给轴及主轴停止
void AxisStopAfterError(struct SChannel* pSChannel);





#endif 