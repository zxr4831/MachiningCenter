#ifndef __GCODE_G2_H
#define __GCODE_G2_H

typedef struct _SG2Para
{
    int8 cStep;

    uint8 ucG2Hor;               //G2横轴
    uint8 ucG2Ver;               //G2纵轴
    SPointDouble SG2StartPoint;  //G2起点
    SPointDouble SG2EndPoint;    //G2终点
    SPointDouble SG2CenterPoint; //G2圆心
    double dG2Radius;            //G2半径
    int8 cG2Dir;                 //G2方向

    SPointDouble SG2MiddlePoint; //整圆赋值需要用

    int8 cType; //0：点 1：圆弧or竖直线 2：整圆

} SG2Para;

void addG2RegulationAPI();

int8 mallocG2ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG2ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG2(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
