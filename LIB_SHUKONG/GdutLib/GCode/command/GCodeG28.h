#ifndef __GCODE_G28_H
#define __GCODE_G28_H

//#include "common.h"
//#include "GCodeCommon.h"

typedef struct _SG28Para
{
    int8 cStep; //切片辅助标志

} SG28Para;

void addG28RegulationAPI();

int8 mallocG28ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG28ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG28Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG28(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG28(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
