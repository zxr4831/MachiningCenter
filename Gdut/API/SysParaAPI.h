#ifndef __SYSPARAAPI_H
#define __SYSPARAAPI_H

#include "common.h"

#define VEL_VS 0   //起始速度
#define VEL_VMAX 1 //最大速度
#define VEL_VE 2   //末速度

#define PLAN_ACC 3 //加速段
#define PLAN_DEC 4 //减速段

#define PLAN_DISTANCE 5 //插补距离

#define MANUAL_QUICK 0 //手动快速
#define MANUAL_SLOW 1  //手动慢速

/***********************************初始化函数*************************************/
//映射初始化
void initMapArray();
// PWM映射初始化
void initPWMMapArray();

void initEncoderMapArray();

//初始化单轴参数
int8 initPhysicalAxisPara(uint8 ucPhysicalAxis, uint8 type);

//初始化通道单轴参数
int8 initStructSingleAxisPara(uint8 ucChannelNum);

//初始化物理轴+ 结构体轴参数
int8 initAxisPara();

//以自定义表初始化SG0 SG1 SG2结构体的UserData(开机自动启动)
int8 initCustomSUserData();

//可编辑参数初始化
void initEditablePara();

//开机需要初始化的参数
int8 initSystemPara();

//从Flash中加载数据后执行的初始化
void initAfterLoadSysPara();

/************************************其他参数设置***********************************/
//获取映射值
uint8 getMapValue(uint8 ucChannelNum, uint8 ucAxis);

//获取PWM映射值
uint8 getPWMMapValue(uint8 ucChannelNum, uint8 ucAxis);

//修改结构体的映射值
int8 setMapValue(uint8 ucChannelNum, uint8 ucAxis, uint8 ucPhysicalAxisNum);

//修改结构体的PWM映射值
int8 setPWMMapValue(uint8 ucChannelNum, uint8 ucAxis, uint8 ucPhysicalAxisNum);

//获取/设置手轮模拟速度
int getWheelSimulaSpeed();
int8 setWheelSimulaSpeed(int nVal);
#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
/* 以整数表示的基础距离，单位：微米 / 0.001 度 / 0.001 路径单位。 */
int getHandwheelSimDistancePerTick(void);
int8 setHandwheelSimDistancePerTick(int nVal);
#endif

//获取/设置手轮模拟速度
int getSimulaVmax();
int getSimulaG0Vmax();
int8 setSimulaVmax(int nVal);
int8 setSimulaG0Vmax(int nVal);

int8 clearMachineTimeTotal();

//设置倾斜角度 [倾斜Y轴机]
double getAngleXY1(double dXDeviation, double dY1Deviation);
/********************************合成参数**************************************/

//获取/设置合成速度
int getCommonVel(uint8 ucVelType);
int8 setCommonVel(int nVal, uint8 ucVelType);

//获取/设置合成加减速类型
int getCommonPlanType(uint8 ucAccType);
int8 setCommonPlanType(int nVal, uint8 ucAccType);

//获取/设置加减速度
int getCommonAcc(uint8 ucAccType, uint8 ucPlanType);
int8 setCommonAcc(int nVal, uint8 ucAccType, uint8 ucPlanType);

/********************************单轴参数**************************************/
//获取/设置轴类型
int getPhysicalAxisType(uint8 ucPhysicalAxis);
int8 setPhysicalAxisType(uint8 ucPhysicalAxis, int nVal);

//设置直径显示
int getShowByDiameter(uint8 ucPhysicalAxis);
int8 setShowByDiameter(uint8 ucPhysicalAxis, int nVal);

//获取/设置一转的位移
int getDistPerCycle(uint8 ucPhysicalAxis);
int8 setDistPerCycle(uint8 ucPhysicalAxis, int nVal);

//获取/设置 mm对应脉冲（或转对应脉冲）
int getPulsePerMMOrCycle(uint8 ucPhysicalAxis);
int8 setPulsePerMMOrCycle(uint8 ucPhysicalAxis, int nVal);

//获取/设置单轴速度
int getAxisVel(uint8 ucPhysicalAxis, uint8 ucVelType);
int8 setAxisVel(uint8 ucPhysicalAxis, int nVal, uint8 ucVelType);

//获取/设置主轴旋转时速度
int getSpindleAxisVel(uint8 ucVelType);
int8 setSpindleAxisVel(int nVal, uint8 ucVelType);

//获取/设置单轴加减速类型
int getAxisPlanType(uint8 ucPhysicalAxis, uint8 ucAccType);
int8 setAxisPlanType(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType);

//获取/设置单轴加速度
int getAxisAcc(uint8 ucPhysicalAxis, uint8 ucAccType, uint8 ucPlanType);
int8 setAxisAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType, uint8 ucPlanType);

//获取/设置主轴加速度
int getSpindleAxisAcc(uint8 ucPhysicalAxis);
int8 setSpindleAxisAcc(uint8 ucPhysicalAxis, int nVal);

//获取/设置主轴减速度
int getSpindleAxisDec(uint8 ucPhysicalAxis);
int8 setSpindleAxisDec(uint8 ucPhysicalAxis, int nVal);

//获取/设置手动速度
int getManualSpeed(uint8 ucPhysicalAxis, uint8 ucManualType);
int8 setManualSpeed(uint8 ucPhysicalAxis, int nVal, uint8 ucManualType);

//获取/设置手动加速度
int getManualAcc(uint8 ucPhysicalAxis, uint8 ucAccType, uint8 ucManualType);
int8 setManualAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType, uint8 ucManualType);

//获取/设置单轴转向允许变化的最大速度
int getAxisVeerDeltaVMax(uint8 ucPhysicalAxis);
int8 setAxisVeerDeltaVMax(uint8 ucPhysicalAxis, int nVal);

//获取/设置单轴间隙值
int getAxisClearance(uint8 ucPhysicalAxis);
int8 setAxisClearance(uint8 ucPhysicalAxis, int nVal);

//获取/设置单轴反向间隙值
int getAxisRevClearanceCompDist(uint8 ucPhysicalAxis);
int8 setAxisRevClearanceCompDist(uint8 ucPhysicalAxis, int nVal);

//获取/设置单轴反向间隙补偿加速度
int getAxisRevClearanceCompAcc(uint8 ucAccType);
int8 setAxisRevClearanceCompAcc(int nVal, uint8 ucAccType);

//获取/设置单轴反向间隙补偿速度
int getAxisRevClearanceCompVel(uint8 ucVelType);
int8 setAxisRevClearanceCompVel(int nVal, uint8 ucVelType);

//获取/设置单轴段间非反向时过度最大加速度  mm/ms2（°/ms2）dVeerAMax
int getVeerAMax(uint8 ucPhysicalAxis);
int8 setVeerAMax(uint8 ucPhysicalAxis, int nVal);

//获取/设置dCoefOfAngleToLength;    //旋转轴角度到长度转换系数
int getCoefOfAngleToLength(uint8 ucPhysicalAxis);
int8 setCoefOfAngleToLength(uint8 ucPhysicalAxis, int nVal);

#if (TURRET_AXIS_TEST == 1)
//设置通道与刀塔的映射
int8 setChannelToTurretMapArray(uint8 ucChannelNum, uint8 ucTurretNum, uint8 ucTargetTurretNum);
//设置刀塔与结构体轴的映射
int8 setTurretToAxisMap(uint8 ucTurretNum, uint8 ucTurretSign);
#endif

//设置机械参考点各轴的坐标值
int8 setMachine_REFPOT(uint8 ucREFPOT_Name, uint8 ucAxis, double dOffsetVal);
//获取机械参考点各轴的坐标值
double getMachine_REFPOT(uint8 ucREFPOT_Name, uint8 ucAxis);

//设置G74 G75进刀量 G76螺纹粗车最小切削量
int8 setFeedAndCuttingUnits(uint8 ucVal);
//获取G74 G75进刀量 G76螺纹粗车最小切削量
bool getFeedAndCuttingUnits(void);
//获取G84孔底暂停默认时间
uint16 getG84DefaultDelayTime(void);
//设置G84孔底暂停默认时间
int8 setG84DefaultDelayTime(uint16 usVal);
#endif // !__COORDAPI_H
