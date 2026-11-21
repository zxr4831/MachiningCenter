#ifndef __GCODE_G71_H
#define __GCODE_G71_H


#include "common.h"
//#include "GCodeCommon.h"

#define G71_DEBUG (1&(!STM32_MODE))

typedef struct _SG71Para
{
	bool bG72;					//G72标志
	bool bSwapFlag;				//交换坐标标志

	int8 cStep;                 //切片辅助标志
	int8 cStepInBlock;			//块中的切片

	//G71指令参数
	uint8 ucH;            // 0: I型   1: II型  默认为0

	double dDeltaD;		//切削量	0.001～99.999 (单位：mm / inch，半径值)，无符号  △d Cutting Depth
	double dDeltaE;		//退刀量	0～99.999(单位：mm / inch，半径值)，无符号        e

	uint32 unNS;		//起始行号
	uint32 unNF;		//结束行号

	double dDeltaU;      //取值范围 - 99999.999～99999.999 (直径, 单位：mm / inch，有符号) ，默认为0  △u 轨迹X偏移值
	double dDeltaW;      //取值范围 - 99999.999～99999.999 (直径, 单位：mm / inch，有符号) ，默认为0  △w 轨迹Z偏移值


	//轨迹参数
	SList* pSPathList;  //轨迹链表
	SPointDouble SPointA1;//A点		G71起点
	SPointDouble SPointA2;//A'点	偏置后的加工轨迹起点
	//SPointDouble SPointB2;//B'点	作用？
	SPointDouble SPointC2;//C'点

	int8 cMachineXDir;			//轨迹X加工方向	1：+向-加工 -1：-向+加工
	int8 cMachineZDir;			//轨迹X加工方向	1：+向-加工 -1：-向+加工

	SList* pSGrooveList;  //槽链表 双层链表

	int16 usCurPos;  //写入加工轨迹链表用 G71Ⅰ型用
	
	int16 cCurrMachineGrooveNum;		//当前加工槽号（新）
	int16 cCurrPointGrooveNum;		//当前点所在槽号（新）

	int8 cCurrMachineDir;		//当前加工方向（当反向加工时，圆弧方向需要取反）

	bool bOutOfEdge;//回切段超出起始边or终止边
	double dOutVal;//超出值

	int8 cStepInCut_2;

	//int8 cFeedPos;	//进给段所在实体节点位置
	//int8 BackPos;	//回切段所在实体节点位置

	double dRestFeedVal;		//剩余进给量 
	double dRestBackVal;		//剩余回切量



	SPoint SCycleBlockPointA;	//循环块A点
	SPoint SCycleBlockPointB;	//循环块B点

	SPoint STransitBlockPointA;	//收尾块A点
	SPoint STransitBlockPointB;	//收尾块B点
	SPoint STransitBlockPointC;	//收尾块C点
	int16 cBeforeTransitGrooveNum;
	int16 cAfterTransitGrooveNum;

	double dCurrCutDepth;	//当前切削深度

	//SPoint SBackBlockPointA;
	//SPoint SBackBlockPointB;




	//获取轨迹链表时所需的数据
	SGetTextStateInfo SGetTextStateInfo;
	SScan SScanContainer;
	SGCommandRegulation* pSCurrCommandRegulation;//当前执行Ga、Gz组指令规则
	int8 cCommandSegmentCnt;

	int8 cEndPointPos;//0：A点	1：A'点	2：C'点

	bool bFinish;


}SG71Para;


typedef struct _SGrooveData   //G71 II型槽数据
{
	uint16 ucGrooveNum;				//槽号  
	SList* pSGroovePathList;		//槽轨迹链表
	int8 cGrooveBottomPos;			//槽底在槽轨迹链表中的位置 (位置由最后节点倒数过去) 用于区分起始边和终止边  
	                                //小于（槽轨迹链表大小-cGrooveBottomPos）为终止边   大于等于（槽轨迹链表大小-cGrooveBottomPos）为起始边
	
	int16 cTotalStartLineEntityNum;	//槽起始边的实体数量
	int16 cTotalEndLineEntityNum;	//槽终止边的实体数量

	SPoint SPointA;					//起始边顶点
	SPoint SPointB;					//终止边顶点
	SPoint SPointC;					//槽底点

	//以下为加工数据
	bool bPointA2;
	SPoint SPointA2;	//起始边第一刀深度（可能超过槽底）
	bool bPointB2;
	SPoint SPointB2;	//终止边第一刀深度（可能超过槽底）

	double dResCutDistance;			//每个槽的剩余切深深度

	bool bFinished;

}SGrooveData;

void addG71RegulationAPI();
void addG72RegulationAPI();

int8 mallocG71ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG71ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG71Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG71(struct _SSyntaxConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG71(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 feed(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cFeedAxis);
int8 cut_1(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cCutAxis);
int8 cut_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cCutAxis);
int8 back(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cBackAxis);
int8 locate(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit, int8 cFeedAxis);
void swapPathListXZCoord(SList** ppSPathList);
void swapBufXZCoord(struct SGCodeParse* pSGCodeParseUnit);
void swapVal(double* pA, double* pB);
void freeGroovePathList(void* pData);

#endif
