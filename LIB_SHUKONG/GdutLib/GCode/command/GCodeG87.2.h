#ifndef _GCODE_G87_2_H
#define _GCODE_G87_2_H

#include "common.h"
#include "GCodeCommon.h"

typedef struct _SG87_2Para
{
    int8 cG87_2Step; //G87_2赋值步骤

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
    bool bIsFirstHole; //是否为第一个孔

    int8 cGaBeforeG87_2;                           //保存运行前的模态
    SGCommandRegulation* pGaBeforeG87_2Regulation; //保存运行前的模态规则地址

} SG87_2Para;

//添加G87_2规则结构体
void addG87_2RegulationAPI();

//开辟G87_2参数结构体
int8 mallocG87_2ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//释放G87_2参数结构体
int8 freeG87_2ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//  获取G87_2参数
int8 getG87_2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState);

// 赋值G87_2
int8 AssignG87_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

int8 CheckG87_2(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
#endif // !_GCODE_G87_2_H
