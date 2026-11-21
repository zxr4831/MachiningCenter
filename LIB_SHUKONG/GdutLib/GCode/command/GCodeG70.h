#ifndef __GCODE_G70_H
#define __GCODE_G70_H


#include "common.h"
//#include "GCodeCommon.h"


typedef struct _SG70Para
{
	SGetTextStateInfo SGetTextStateInfo;			//用于记录G70的NS至NF程序段获取文本状态
	SScan SScanContainer;							//1.用于记录G70的NS-NF程序段扫描结果 or 保存赋值结构体中的扫描结构体，G70执行完后再还回去
//	struct _SGCodeSysState SGCodeSysState;			//用于记录赋值G70的NS-NF程序段时的系统状态

	int8 cCommandSegmentCnt;						//用于判断多程序段是否相同
	int16 sCurrCommandVal;							
	SGCommandRegulation* pSCurrCommandRegulation;	//用于记录G70内部当前执行Ga、Gz组指令规则

	int8 cStep;                 //切片辅助标志
	int8 cPart;					//赋值切片

	uint32 unNS;				//NS行号
	uint32 unNF;				//NF行号
	SPointDouble SCycleStartPoint;  //本次G70循环起点，内部机械坐标

	double dDelta[3];      //X余量 (直径, 单位：mm / inch，有符号) ，默认为0  △u 轨迹X偏移值
	//double dDeltaW;      //Z余量 (直径, 单位：mm / inch，有符号) ，默认为0  △w 轨迹Z偏移值

	uint8 ucH;//类型 0：正向	1：正向镜像		2：反向		3：反向镜像


	SList* pSPathList;//原始轨迹链表（此链表记录NS-NF轨迹，记录后不做修改）
	uint16 usCurPos;				//当前写入实体在加工轨迹链表中的位置

}SG70Para;

void addG70RegulationAPI();

int8 mallocG70ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG70ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG70Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG70(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG70(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
