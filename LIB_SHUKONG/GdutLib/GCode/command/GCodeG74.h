#ifndef __GCODE_G74_H
#define __GCODE_G74_H

typedef struct _SG74Para
{
	int8 cStep;                 //切片辅助标志

	SCoord SCoordInfo;
	SPointDouble SStartPoint;	//指令起点
	SPointDouble SEndPoint;		//指令终点

	int8 cDir_X;		//X轴加工方向
	int8 cDir_Z;		//Z轴加工方向

	double dFeed_X;		//X轴进刀量
	double dRetract_X;	//X轴退刀量

	double dFeed_Z;		//Z轴进刀量
	double dRetract_Z;	//Z轴退刀量

	bool bFirstFeed_Z;	//Z轴首次进刀
	bool bLastFeed;		//最后一次循环

}SG74Para;


void addG74RegulationAPI();

int8 mallocG74ParaStruct(uint8 ucMode, uint8 ucChannelNum);

int8 freeG74ParaStruct(uint8 ucMode, uint8 ucChannelNum);

int8 getG74Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG74(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 AssignG74(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);



#endif