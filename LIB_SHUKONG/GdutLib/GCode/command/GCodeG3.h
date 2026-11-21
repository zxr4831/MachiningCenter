#ifndef __GCODE_G3_H
#define __GCODE_G3_H


typedef struct _SG3Para
{
	int8 cStep;

	uint8 ucG3Hor;
	uint8 ucG3Ver;
	SPointDouble SG3StartPoint;
	SPointDouble SG3EndPoint;
	SPointDouble SG3CenterPoint;
	double dG3Radius;
	int8 cG3Dir;

	SPointDouble SG3MiddlePoint;//整圆赋值需要用

	int8 cType;//0：点 1：圆弧or竖直线 2：整圆

}SG3Para;

void addG3RegulationAPI();

int8 mallocG3ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG3ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG3Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG3(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG3(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
