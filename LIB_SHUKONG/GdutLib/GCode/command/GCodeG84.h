#ifndef _GCODE_G84_H
#define _GCODE_G84_H


typedef struct _SG84Para
{
	int8 cG84Step;//G84赋值步骤

	double dR;//R值（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
	double dQ;//Q值 （每次进给深度(正负号无效) ）

	int cFeedQTimes;//进给Q的次数
	double dF;				//螺纹的导程
	double dD; //每次回退高度
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

	int8 cGaBeforeG84;		//保存运行前的模态
	SGCommandRegulation* pGaBeforeG84Regulation;//保存运行前的模态规则地址

}SG84Para;

//添加G84规则结构体
void addG84RegulationAPI();  

//开辟G84参数结构体
int8 mallocG84ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//释放G84参数结构体
int8 freeG84ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//  获取G84参数
int8 getG84Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState);

// 赋值G84
int8 AssignG84(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

int8 CheckG84(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);




























#endif // !GCode_G84_H

