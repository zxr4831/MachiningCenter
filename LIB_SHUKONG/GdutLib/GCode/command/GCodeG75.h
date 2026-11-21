#ifndef  _GCODE_G75_H_
#define  _GCODE_G75_H_

typedef struct _SG75LoopPara
{
	int8 cStepInLoop;
	double dSX;
	double dSZ;
	double dEX;
	double dEZ;

	int8 cDir_X;		//X轴加工方向（终点X-起点X）        1：沿X坐标轴负向向正向     -1：沿X坐标轴正向向负向
	int8 cDir_Z;		//Z轴加工方向（终点Z-起点Z）       1.：沿Z坐标轴负向向正向   -1： 沿Z坐标轴正向向负向

	double dFeed_X;		//X轴进刀量
	double dRetract_X;	//X轴退刀量

	double dFeed_Z;		//Z轴进刀量
	double dRetract_Z;	//Z轴退刀量

	bool bFirstFeed_X;	//Z轴首次进刀
	bool bLastFeed;		//最后一次循环
}SG75LoopPara;

typedef struct _SG75Para
{
	int8 cStep;                 //切片辅助标志

	bool bG74;					//G74标志
	uint8 ucH;					//类型

	SCoord SCoordInfo;			//坐标信息
	SPointDouble SStartPoint;	//指令起点
	SPointDouble SEndPoint;		//指令终点

	double dFeed_X;		//X轴进刀量
	double dRetract_X;	//X轴退刀量

	double dFeed_Z;		//Z轴进刀量
	double dRetract_Z;	//Z轴退刀量


	int8 cCommandDir_X;		//指令X轴加工方向（终点X-起点X）        1：沿X坐标轴负向向正向     -1：沿X坐标轴正向向负向
	int8 cCommandDir_Z;		//指令Z轴加工方向（终点Z-起点Z）       1.：沿Z坐标轴负向向正向   -1： 沿Z坐标轴正向向负向

	int8 cCurrMachineDir;	//Ⅲ型的当前加工方向

	//循环参数
	SG75LoopPara SG75LoopPara;

	//层参数
	int8 cCurrLayNum;		//当前层数
	int8 cTotalLayNum;		//总层数
	double dLayerDepth;

	double dCurrLayStartX;	//当前层起点X
	double dCurrLayEndX;	//当前层终点X

	bool bFirstFeed_X;	//Z轴首次进刀
	bool bLastFeed;		//最后一次循环

}SG75Para;


void addG75RegulationAPI();

int8 mallocG75ParaStruct(uint8 ucMode, uint8 ucChannelNum);

int8 freeG75ParaStruct(uint8 ucMode, uint8 ucChannelNum);

int8 getG75Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG75(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);

int8 AssignG75(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);


int8 calcG75Loop(SG75LoopPara* pSG75LoopPara, SPointDouble SCurPoint, struct SGCodeParse* pSGCodeParseUnit);


#endif // ! _GCODE_G75_H_

