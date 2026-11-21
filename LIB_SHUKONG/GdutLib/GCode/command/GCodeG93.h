#ifndef __GCODE_G93_H
#define __GCODE_G93_H

//#include "common.h"
//#include "GCodeCommon.h"

typedef struct _SG93Para
{
    int8 cStep;          //切片辅助标志
    double dR;           //R值
    double dRRaw;        // 新输入的 R 原始值，单位随编程单位。
    bool bRNeedsConvert; // 新 R 待处理标志，本行单位生效后处理一次。

    SCoord SCoordInfo;    //程序段坐标信息
    SPointDouble SPointA; //A点坐标（机械坐标）
    SPointDouble SPointC; //C点坐标（机械坐标）

} SG93Para;

void addG93RegulationAPI();

int8 mallocG93ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG93ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG93Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG93(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG93(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
