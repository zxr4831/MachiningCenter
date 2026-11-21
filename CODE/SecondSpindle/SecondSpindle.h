#ifndef SSecondSpindle_H
#define SSecondSpindle_H
#include "common.h"
#include "Spindle.h"
#define SECONDSPINDLE_USE_NOENCODER 1		//第二转轴不使用编码器 1 不使用	0 使用

//第二轴 / 动力刀启动接口函数  ，当主轴在停下过程或停下时，调用此函数可让主轴按照停下前的状态运动
int8 SSecondSpindleBegin(struct _SSpindle* pSSpindlePointer);

//从插补结构体中，读取第二转轴/动力刀的转速
int8 readSecondSpindleSpeed(struct SInterpolationPre* pSPrePointer, struct _SSpindle* pSSpindlePointer);

//主轴转速计算输出
int8 SecondspindleMotion(struct _SSpindle* pSSpindlePointer);

//第二转轴无编码器，通过理论当前速度和目标速度的比值粗略的判断第二转轴转速是否达到要求
int8 getSecondSpindleSpeedReached(struct _SSpindle* pSSpindlePointer);

int8 SecondspindleSpeedTest(struct _SEncoderTest* pEncoderTest, struct _SSpindle* pSSpindlePointer);
#endif
