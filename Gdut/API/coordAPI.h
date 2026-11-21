#ifndef __COORDAPI_H
#define __COORDAPI_H

#include "common.h"

/************************坐标相关****************************/

//机械坐标
double getMachCoordDouble(uint8 ucChannelNum, uint8 ucAxis);     //获取Double
int getMachCoordInt(uint8 ucChannelNum, uint8 ucAxis);           //获取Int
int8 setMachCoord(uint8 ucChannelNum, uint8 ucAxis, double val); //设置

//绝对坐标
double getAbsCoordDouble(uint8 ucChannelNum, uint8 ucAxis); //获取绝对坐标Double
int getAbsCoordInt(uint8 ucChannelNum, uint8 ucAxis);       //获取绝对坐标Int

//相对坐标
double getRelaCoordDouble(uint8 ucChannelNum, uint8 ucAxis);     //获取相对坐标Double
int getRelaCoordInt(uint8 ucChannelNum, uint8 ucAxis);           //获取相对坐标Int
int8 setRelaCoord(uint8 ucChannelNum, uint8 ucAxis, double val); //设置相对坐标
//int8    clearRalaCoor(uint8 ucChannelNum, uint8 ucAxis);//清零相对坐标

////修改坐标系 + 输入
int8 modifyCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis,
                      double dOffset); // ucNum : 1 EXT; 2 G54; 3 G55 ; 4 G56 ; 5 G57 ; 6 G58 ; 7 G59 ;

//设置外部/工件坐标系
int8 setCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis,
                   double dOffset); // ucNum : 1 EXT; 2 G54; 3 G55 ; 4 G56 ; 5 G57 ; 6 G58 ; 7 G59 ;

//输入外部/工件坐标系
int8 inputCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis,
                     double dOffset); // ucNum : 1 EXT; 2 G54; 3 G55 ; 4 G56 ; 5 G57 ; 6 G58 ; 7 G59 ;

//获取外部/工件坐标系偏移
double getCoorOffset(uint8 ucChannelNum, uint8 ucNum, uint8 ucAxis);

//获取G50坐标系偏移
double getCoorOffsetByG50(uint8 ucChannelNum, uint8 ucAxis);

//设置G50坐标系偏移
int8 setCoorOffsetByG50(uint8 ucChannelNum, uint8 ucAxis, double dOffset);

//清除G50坐标系偏移
int8 clearCoorOffset(uint8 ucChannelNum);

//清除坐标系总偏移
int8 clearTotalCoorOffset(uint8 ucChannelNum);

#endif // !__COORDAPI_H
