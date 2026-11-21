#ifndef __CTRAJPLANNING_H
#define __CTRAJPLANNING_H


#include "common.h"
#include "G32.h"

double getFactor(uint8 ucVelPlanType);

double getSysAcc(uint8 ucVelPlanType);

double getSysDec(uint8 ucVelPlanType);

double getSingleAxisAcc(uint8 ucAxis, uint8 ucVelPlanType, const uint8* ucAxisMap);

double getSingleAxisDec(uint8 ucAxis, uint8 ucVelPlanType, const uint8* ucAxisMap);

//规划预处理函数
int8 preparePlanningT(struct SPlan* pSPlanPointer);    // T型速度规划 预处理

int8 preparePlanningS3(struct SPlan* pSPlanPointer);    // S3型速度规划 预处理

int8 preparePlanningS4(struct SPlan* pSPlanPointer);    //S4型速度规划 预处理

int8 preparePlanningCombine(struct SPlan* pSPlanPointer, bool bUseFloatingVe);//混合型速度规划 预处理

int8 preparePlanningTNewAdd(struct SPlan* pSPlanPointer);    // T型速度规划 （新增距离模式）预处理

//平摊预处理函数
int8 prepareDistReAvgT(struct SPlan* pSPlanPointer, bool bAddCycleFlag);  // 梯形平摊 预处理

int8 prepareDistReAvgS5(struct SPlan* pSPlanPointer);  //五次多项式平摊 预处理

int8 prepareDistReAvgRatio(struct SPlan* pSPlanPointer); //比例平摊 预处理


//规划nextstep函数
double nextstepPlanningT(uint32 unK, struct SPlan* pSPlanPointer);   //T型S(t) 

double nextstepPlanningS3(uint32 unK, struct SPlan* pSPlanPointer);   //S3型S(t)

double nextstepPlanningS4(uint32 unK, struct SPlan* pSPlanPointer);  //S4型S(t)

double nextstepPlanningCombine(uint32 unK, struct SPlan* pSPlanPointer);  //混合型S(t)

//平摊nextstep函数
double nextstepDistReAvgT(uint32 unK, struct SPlan* pSPlanPointer);  //梯形平摊S(t)

double nextstepDistReAvgS5(uint32 unK, struct SPlan* pSPlanPointer);  //五次多项式平摊 S(t)


//封装函数
int8 preparePlanningAndDistReAvg(struct SPlan* pSPlanPointer, bool bUseFloatingVe);  //预处理函数（速度规划+平摊）

double nextstepPlanningAndDistReAvg(uint32 unK, struct SPlan* pSPlanPointer);  //封装多种类型的S(t)

//double nextstepDeRatio();



/*****************************************<速度控制><主轴使用>********************************************/

//速度规划V3
int8 preparePlanningV3(struct SPlan* pSPlanPointer);
//混合速度规划V++
int8 preparePlanningVelCombine(struct SPlan* pSPlanPointer,double dVeerDeltaVMax);

//Vel nextstep函数
double nextstepPlanningV1(uint32 unK, struct SPlan* pSPlanPointer);

double nextstepPlanningV2(uint32 unK, struct SPlan* pSPlanPointer);

double nextstepPlanningV3(uint32 unK, struct SPlan* pSPlanPointer);

double nextstepPlanningVelCombine(uint32 unK, struct SPlan* pSPlanPointer);  //混合型V(t)


//新增距离
uint8 newAddDistace(struct _SG32* pG32, struct SPlan* pSPlanPointer, double newAddDist, double dAcc, double dDec, double dVe, double dCurrentSpeed);

//新增距离 不限制终点
uint8 newAddDistanceUnlimited(struct SPlan* pSPlanPointer, double newAddDist, double dAcc, double dDec, double dVe, bool bEnableDec, double dDecDist , double dCurrentSpeed);

uint8 handWheelAddNewPlan(struct SPlan* pSPlanPointer, double dDistance, double dVs, double dVe, double dCurrentSpeed, bool bIsRotationAxis);

//清除与平摊有关的参数
int8 clearDistReAvgPara(struct SPlan* pSPlanPointer);

//清除速度规划参数
int8 clearSPlanData(struct SPlan* pSPlanPointer);


#endif
