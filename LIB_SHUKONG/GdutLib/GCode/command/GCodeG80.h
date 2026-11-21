#ifndef __GCODE_G80_H
#define __GCODE_G80_H

void addG80RegulationAPI();
//记录执行钻孔指令前的运动模态指令
void setGaBeforeG8x(uint8 ucMode, uint8 ucChannelNum, int16 sGaBeforeG8x, SGCommandRegulation* pGaBeforeG8xRegulation);
//获取执行钻孔指令前的运动模态指令
void getGaBeforeG8x(uint8 ucMode, int16* sBeforeG8x, SGCommandRegulation** pGaBeforeG8xRegulation);

//赋值G80
int8 AssignG80(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
//语法处理G80
int8 CheckG80(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
