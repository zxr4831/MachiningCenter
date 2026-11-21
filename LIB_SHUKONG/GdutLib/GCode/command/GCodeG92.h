#ifndef __GCODE_G92_H
#define __GCODE_G92_H


//#include "common.h"
//#include "GCodeCommon.h"


typedef struct _SG92Para
{
	int8 cStep;                 //切片辅助标志
	double dR;					//R值

	SCoord SCoordInfo;			//程序段坐标信息
	SPointDouble SPointA;		//A点坐标（机械坐标）
	SPointDouble SPointC;		//C点坐标（机械坐标）

	double dJ;			//短轴退尾量（模态）
	double dK;			//长轴退尾量（模态）
	double dF;          //导程（模态）
	double dQ;			//起始角度（非模态）
	uint8 ucL;                  //多头螺纹头数（为模态）
	uint8 ucLNow;               //当前切削的头数

	bool bRepeatG32Flag;    //是否为连续G32
	bool bG32LetterQ;    //G32是否出现Q的标志位
	uint8 ucG32SubMode;		//G32的子类	用于区分G32.2/G32.3/G32.4 陈敏聪增 2023/6/13

	bool bExecute; // 执行标志位  执行的标志为G92程序段中有关键字X




}SG92Para;


void addG92RegulationAPI();

int8 mallocG92ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG92ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG92Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG92(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG92(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif

