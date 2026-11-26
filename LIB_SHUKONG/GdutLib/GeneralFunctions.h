#include "common.h"
#include <math.h>
#ifndef __GERNERALFUNCTIONS_H
#define __GERNERALFUNCTIONS_H
#include "Config.h"

#define EN_DEBUG 0 //开启debug   print输出

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b)) //两个数取小值
#endif
#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b)) //两个数取大值
#endif

#define round(x) ((x) >= 0 ? (int32)((double)((x) + 0.5)) : (int32)((double)((x) - 0.5))) //四舍五入

#define setbit(x, y) x |= (1 << y)                            //将X的第Y位置1
#define clrbit(x, y) x &= ~(1 << y)                           //将X的第Y位清0
#define getbit(x, y) (x & ((unsigned int)1 << y)) > 0 ? 1 : 0 //获取X的第Y位
#define SGN(x) ((x) > 0 ? 1 : -1)

#define SETBIT64(x, y) ((x) |= (1ULL << (y)))  //将X的第Y位置1
#define CLRBIT64(x, y) ((x) &= ~(1ULL << (y))) //将X的第Y位清0
#define GETBIT64(x, y) (((x) >> (y)) & 1ULL)   //获取X的第Y位

//把newVal中某几位的值 写入oldVal 哪几个位由addr为1的位决定
#define setBits(oldVal, newVal, addr)                                                                                  \
    oldVal = (((~addr) & oldVal) | newVal) //变量1,存放模态的变量  变量2,模态对应的宏定义 变量3 模态对应的地址宏定义

//读取val 中某几位的值  哪几个位由addr为1的位决定
#define getBits(val, addr) (addr & val) ////变量1,存放模态的变量 变量2 模态对应的地址宏定义

//角度换算为弧度
#define deg2rad(val) (val / 180.0 * PI)

extern const double EPSINON; //用于变量数值比较

//单位转换
#define UmToMm(x) ((x) / 1000.0)             //um转换到mm
#define MmToUm(x) ((x) * 1000.0)             //mm转换到um
#define PerMinToPerMs(x) ((x) / 60000.0)     // /min转换到/ms		 /60.0/1000.0
#define PerMsToPerMin(x) ((x) * 60000.0)     // /ms转换到/min		 *60.0*1000.0
#define PerSec2ToPerMs2(x) ((x) / 1000000.0) // m/s²转换到mm/ms²	 /1000.0/1000.0
#define PerMs2ToPerSec2(x) ((x) * 1000000.0) // mm/ms²转换到m/s²    *1000.0*1000.0
#define MPerSec2ToMMPerMs2(x) ((x) / 1000.0) // m/s²转换到mm/ms²	 *1000.0/1000.0/1000.0
#define MMPerMs2ToMPerSec2(x) ((x) * 1000.0) // mm/ms²转换到m/s²    /1000.0*1000.0*1000.0"
#define RToAngle(x, ucAxis) ((x) * g_SSystemPara.SAxisPara[ucAxis].fDistPerCircle) //R 转为°  *一圈位移量
#define AngleToR(x, ucAxis) ((x) / g_SSystemPara.SAxisPara[ucAxis].fDistPerCircle) //°转为R   /一圈位移量

#define MAX_LENGTH_OF_STRING 15 //字符串数值大小

/*******************************************************函数部分******************************************************************/

//判断 dA == 0 ?
bool isZero(double dA);
bool isZero_EPSINON2(double dA);

//判断 -dDeviation <= dA <= dDeviation ?
bool isZeroWithDeviation(double dA, double dDeviation);

//判断 dA <= dVal <= dB 或 dB <= dVal <= dA ?
bool isBetweenA2B(double dVal, double dA, double dB);

//判断 dA >= 0 ?
bool isEqualOrGreaterThanZero(double dA); //浮点数大于等于

//判断 dA > 0 ?
bool isUnEqualAndGreaterThanZero(double dA); //浮点数大于(不等于)

//判断 dA <= 0 ?
bool isEqualOrLessThanZero(double dA); //浮点数小于等于

//判断 dA < 0 ?
bool isUnEqualAndLessThanZero(double dA); //浮点数小于不等于

//判断 dA 的符号
int8 sgn(double dA);

//判断 nA 的符号
int8 nSgn(int nA);

//计算圆心角（输入IK）
void radCalculateIK(double* pdStartPoint, double* pdEndPoint, double* pdCenterPoint, int8 cArcDir, double* dRad,
                    uint8 ucHorizontalAxis, uint8 ucVerticalAxis);

//计算圆心角（输入R）
bool radCalculateR(double* pdStartPoint, double* pdEndPoint, double dRadius, double* pdCenterPoint, int8 cArcDir,
                   double* pRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis);

//计算圆心角
double calculateCentralAngle(double dCenter_X, double dCenter_Y, double dStart_X, double dStart_Y, double dEnd_X,
                             double dEnd_Y, int cArcDir);

//输出 dData % nRemainder（输出+or-）
double surplusDoubleData(double data, int remainder);

//输出 dData % nRemainder（输出+）
double surplusDoubleToPositive(double data, int remainder);

//两个点坐标相加
int8 addPoint(double* pSPara1, double* pSPara2, double* pSOutPut, int nArraySize, const VAL_STRUCT_TYPE* ucValStatus);

//两个点坐标相减  参数1减去参数2
int8 subtractPoint(double* pSPara1, double* pSPara2, double* pSOutPut, int nArraySize,
                   const VAL_STRUCT_TYPE* ucValStatus);

//坐标判断是否为零
bool isZeroPoint(double* dTargetPoint, int nArraySize, const VAL_STRUCT_TYPE* ucValStatus);

//根据实时点修正相对位移指定的终点坐标
int8 modifyRelaEndPoint(double* pdStartPoint, double* pdEndPoint, const uint8* ucAxisMap, int nArraySize,
                        const VAL_STRUCT_TYPE* ucRelaVal);

//数组的最大值 返回下标  （可返回最大值大小）
int doubleArrayMax(double* array, int nArraySize, double* pdMaxVal);

//判断通道间是否存在公共轴   2022.4.25 郭强
bool judgeCommonPhyAxisBtwChannels();

//极坐标转换成普通
int8 polarCoord2CommonCoord(double* dPoint, int nArraySize, uint8 ucHorizontalAxis, uint8 ucVerticalAxis, int8 cDir);

//普通坐标转换成极坐标
int8 commonCoord2PolarCoord(double* dPoint, int nArraySize, uint8 ucHorizontalAxis, uint8 ucVerticalAxis);

// 绝对坐标转换成机械坐标
int8 absPoint2RealTimePoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus);

// 机械坐标转换成绝对坐标
int8 realTimePoint2AbsPoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus);

//计算旋转坐标系后的坐标点
int8 rotatingCoordinate(double* dPoint, int nArraySize, double* dOutputPoint, double dDeltaRad, uint8 ucHorizontalAxis,
                        uint8 ucVerticalAxis);

//判断是否在椭圆上
bool isOnTheEllipse(double dH, double dV, double dH0, double dV0, double dA, double dB);

//椭圆中心点计算
int8 ellipseCenterCalculate(double* pdStartPoint, double* pdEndPoint, int nArraySize, double dA, double dB,
                            double* pdCenterPoint, int8 cArcDir, double dDeltaRad, uint8 ucHorizontalAxis,
                            uint8 ucVerticalAxis);

//普通坐标转化圆柱 2023.3.30 zwz
double commonCoord2Cylinder(double dPoint, double dCylinderRadius);

//圆柱转化普通坐标 2023.3.30 zwz
double cylinderCoord2Common(double dPoint, double dCylinderRadius);

//选择插补平面  G17 G18 G19
int8 chooseInterpPlane(uint8* ucAxisArray, uint32 unCommandType);

//字符串转为数值
double stringToValue(const int8* pStartPtr, uint8 ucLen, const uint32 unLineNum);

//字符串转换数值
double findValue(const int8* pStartPtr, uint8 ucWord_Len, const uint32 unLineNum);

//除去字符串里的空格
void eliminateBlank(int8* pString, uint16 usStrSize);

//比较字符串
bool isTagetStr_A(const int8* cStr, const int8* pCurrPtr);

//比较字符串
bool isTagetStr_B(const int8* cStr, const int8* pCurrPtr);

//整型转字符串
char* itoaNC(int value, char* str, int radix);

#endif
