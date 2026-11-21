#ifndef _GCODE_G83_H
#define _GCODE_G83_H

#include "common.h"
#include "GCodeCommon.h"

typedef struct _SG83Para
{
    int8 cG83Step; //G83赋值步骤

    double dR;          //R值（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
    double dQ;          //Q值 （每次进给深度(正负号无效) ）
    uint32 unDelaytime; //洞底暂停时间(有小数点，以秒为单位；无小数点，参考Pr17与Pr3241)
    int8 cK;            //重复次数
    int8 cFeedAxis;     //进给轴
    int8 cFeedDir;      //进给方向

    SCoord SCoordInfo;               //程序段坐标信息(未加总偏移值)
    SPointDouble SFinalPoint;        //孔底坐标
    SPointDouble SFirstPoint;        //初始点
    SPointDouble SRPoint;            //R点
    SPointDouble SCuttingStartPoint; //切削起始点

    bool bIsFirstFeed; //是否为第一次进给
    bool bIsFirstHole; //是否为第一个孔   （当次数K不为0时）

    double dD; //D值，每次回退高度 （不分正负号）

    bool bG84Flag;
    double dF; //G84螺纹的导程

    /* SCoordInfo 中保存的 XYZ/UVW 的单位：TRUE 为英寸，FALSE 为毫米。
	 * 不表示 Q/R/D 的单位，也不用于 ABC 角度。
	 */
    bool bCoordImperial;

    //int8 cGaBeforeG83;		//保存运行前的模态
    //SGCommandRegulation* pGaBeforeG83Regulation;//保存运行前的模态规则地址

} SG83Para;

//添加G83规则结构体
void addG83RegulationAPI();
////添加G84规则结构体
//void addG84RegulationAPI();

//开辟G83参数结构体
int8 mallocG83ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//释放G83参数结构体
int8 freeG83ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//  获取G83参数
int8 getG83Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState);

// 赋值G83
int8 AssignG83(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

int8 CheckG83_3(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

#endif // !_GCODE_G83_H
