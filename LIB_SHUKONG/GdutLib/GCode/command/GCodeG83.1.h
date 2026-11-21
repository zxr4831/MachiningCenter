#ifndef _GCODE_G83_1_H
#define _GCODE_G83_1_H

#include "common.h"
#include "GCodeCommon.h"

typedef struct _SG83_1Para
{
	int8 cG83_1Step;//G83赋值步骤   

	double dR;//R值（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
	double dQ;//Q值 （每次进给深度(正负号无效) ）
	uint32 unDelaytime;		//洞底暂停时间(有小数点，以秒为单位；无小数点，参考Pr17与Pr3241)
	int8 cK;				//重复次数
	int8 cFeedAxis;			//进给轴
	int8 cFeedDir;			//进给方向

	SCoord SCoordInfo;		//程序段坐标信息(未加总偏移值)
	SPointDouble SFinalPoint;//孔底坐标
	SPointDouble  SFirstPoint;//初始点
	SPointDouble  SRPoint;//R点
	SPointDouble SCuttingStartPoint;//切削起始点

	bool bIsFirstFeed;//是否为第一次进给
	bool bIsFirstHole;//是否为第一个孔
	bool bIsNotFirstG83_1;//是否为第一个G83.1  连续G83.1时，当为第一次G83.1时，需要记录第一次的初始点，后续的G83.1的R点都根据第一次的初始点计算
	int8 cGaBeforeG83_1;		//保存运行前的模态
	SGCommandRegulation* pGaBeforeG83_1Regulation;//保存运行前的模态规则地址

}SG83_1Para;

//添加G83规则结构体
void addG83_1RegulationAPI();

//开辟G83参数结构体
int8 mallocG83_1ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//释放G83参数结构体
int8 freeG83_1ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//  获取G83参数
int8 getG83_1Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState);

// 检查G83
int8 CheckG83_1(struct _SSyntaxConfiguration*pSSyntaxConfig,  struct SGCodeParse* pGCodeBufUnit);

// 赋值G83
int8 AssignG83_1(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);







#endif