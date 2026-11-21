#ifndef __RUNAPI_H
#define __RUNAPI_H


#include "common.h"


/****************************运行相关******************************/

//暂停按键接口函数
int8 pauseButtonAPI(uint8 ucChannelNum);

//复位按键接口函数
int8 resetButtonAPI(uint8 ucChannelNum);

//运行按键接口函数：距离手轮交接未完成时返回 0，调用方等待后重试
int8 runButtonAPI(uint8 ucChannelNum, uint8 ucIsGetBeforeProgram);

//模拟运行按钮接口函数
int8 simulaButtonAPI(uint8 ucChannelNum);

#if HANDWHEEL_SIM_DISTANCE_ENABLE
/* 串行提交明确开关；-1 通道无效，0 已关闭/拒绝，1 已选中或待交接。
 * 原运动周期完成交接，真实暂停及模式退出取消续行，复位/急停等待原停车。 */
int8 setHandwheelSimEnabledAPI(uint8 ucChannelNum, bool bEnabled);

/* 只读实际选择或未完成请求，不代表运动已经接管或机械已停稳。 */
bool isHandwheelSimSelectedAPI(uint8 ucChannelNum);

/* TRUE 表示交接等待；此时 runButtonAPI 返回 0，调用方须保留启动步骤等待。 */
bool isHandwheelSimSwitchPendingAPI(uint8 ucChannelNum);
#endif


//单节执行 接口函数
int8 singleLineButtonAPI(uint8 ucChannelNum);

//选择停止接口函数 
int8 selectableStopAPI(uint8 ucChannelNum);

//急停 接口函数 
int8 emergencyStopAPI(uint8 ucChannelNum);

//系统准备状态
int8 sysPrepare(uint8 ucChannelNum);

//G代码运行接口
int8 GCodeRunAPI(uint8 ucChannelNum);

//G代码循环运行接口（根据通道数）
void GCodeRunLoopAPI();

//中断函数接口
void interruptFuncAPI(uint8 ucChannelNum);

//中断函数循环接口
void interruptFuncLoopAPI();

//手动换刀
int8 manualChangeToolAPI(uint8 ucChannelNum, uint8 ucToolNum);

//改变倍率
void changeFeedRatioAPI(uint8 ucChannelNum, float fNewRatio, bool bIsDelta, uint8 ucType);

//设置手轮模式的轴 、倍率
int8 setHandwheelMode(uint8 ucChannelNum, uint8 ucAxis, uint8 ucWheelMultiple);

//设置运行模式：调用方须先暂停并等各通道停稳，再切换模式；不能带速直接赋模式
int8 setSysRunMode(enum SYSTEMSTATUS runMode);


//开始画图接口
int8 drawBtnAPI(uint8 ucChannelNum);


void drawInitAPI(uint8 ucChannelNum);
//绘图循环(弃用)
//void gCodeDrawLoop();


//调试API
// 
//插补预处理状态  0:Fin 1 : INIT 2 : ASS 3 : BLO 4 : RUN 5 : PAU 6 : DEC 7 : STOP 8 : OCCU
int8 getSInterPreStatus(uint8 ucChannelNum);

//阻塞锁
uint8 getIOBlockedCounter(uint8 ucChannelNum);

//运动状态  0:Fin 1 : INIT 2 : ASS 3 : BLO 4 : RUN 5 : PAU 6 : DEC 7 : STOP 8 : OCCU
int8 getMotionTESTStatus(uint8 ucChannelNum);

//当前速度  mm/min
int getCurrentSpeed(uint8 ucChannelNum);

//主轴转速误差百分比  %
double getSpindleSpeedErrorRatio(uint8 ucChannelNum);

//当前指令加速度  m/s2
double getMotionAcc(uint8 ucChannelNum);

//当前指令减速度  m/s2
double getMotionDec(uint8 ucChannelNum);

//当前指令速度规划的unT5
uint32 getUINT5(uint8 ucChannelNum);

//获取当前行缓冲区的加速度 m/s2
double getCurrentBuffAcc(uint8 ucChannelNum);

//获取当前行缓冲区的减速度 m/s2
double getCurrentBuffDec(uint8 ucChannelNum);

//获取前瞻当前行号
int getLookAheadCurrentLine(uint8 ucChannelNum);

//获取插补当前行号
int getInterpLine(uint8 ucChannelNum);

//判断当前通道环境能否运行
bool judgeRunCondition(uint8 ucChannelNum);

#endif // !__RUNAPI_H
