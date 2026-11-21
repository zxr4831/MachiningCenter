#ifndef __TOOLCHANGER_H
#define __TOOLCHANGER_H

#include "common.h"

//
//int8 toolInitAfterLoad();
//
////清除所有刀具ID的偏移值
//int8 clearAllToolOffset(SToolParaInfo *STargetPara,uint8 ucTargetParaSize);
//
////清除一行刀具ID的偏移值
//int8 clearSingleToolOffset(uint8 ucToolID,SToolParaInfo* STargetPara,  uint8 ucTargetParaSize);
//
////设置指定刀具的偏移值
//int8 setToolOffset(uint8 ucToolID,uint8 ucAxis ,double dOffsetVal,SToolParaInfo *STargetPara,uint8 ucTargetParaSize);
//
////输入偏移值
//int8 inputToolOffset( uint8 ucToolID,uint8 ucAxis ,double dOffsetVal,SToolParaInfo *STargetPara,uint8 ucTargetParaSize, bool bIsRelaVal);

/*****************************************刀具偏移***********************************************/
// 
//清除所有刀具ID的偏移值
int8 clearAllToolOffset(uint8 ucChannelNum);

//清除一行刀具ID的偏移值
int8 clearSingleToolOffset(uint8 ucChannelNum,uint8 ucToolID);

//设置指定刀具的偏移值
int8 setToolOffset(uint8 ucChannelNum,uint8 ucToolID, uint8 ucAxis, double dOffsetVal);

//输入偏移值
int8 inputToolOffset(uint8 ucChannelNum,uint8 ucToolID, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal);

//获取偏移值
double getToolOffset(uint8 ucChannelNum, uint8 ucToolID, uint8 ucAxis);
/*****************************************刀具半径***********************************************/
//
//清除所有刀具ID的半径值
int8 clearAllToolRadius(uint8 ucChannelNum);

//清除一行刀具ID的半径值
int8 clearSingleToolRadius(uint8 ucChannelNum,uint8 ucToolID);

//输入半径值
int8 inputToolRadius(uint8 ucChannelNum, uint8 ucToolID, double dToolRadius);

//获取刀具半径
double getToolRadius(uint8 ucChannelNum, uint8 ucToolID);
/*****************************************刀具磨损***********************************************/
//
//清除所有刀具ID的磨损值
int8 clearAllToolWear(uint8 ucChannelNum);

//清除一行刀具ID的磨损值
int8 clearSingleToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum);

//输入磨损值
int8 inputToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum, uint8 ucAxis, double dOffsetVal, bool bIsRelaVal);

//获取刀具磨损值
double getToolWear(uint8 ucChannelNum, uint8 ucToolID, uint8 ucGroupNum, uint8 ucAxis);






#endif