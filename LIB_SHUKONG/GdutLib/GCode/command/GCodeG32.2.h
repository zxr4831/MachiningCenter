#ifndef __GCODE_G32_2_H
#define __GCODE_G32_2_H

typedef struct _SG32_2Para
{
    int8 cStep; //切片辅助标志

    SPointDouble SStartPoint;
    SPointDouble SEndPoint;

    double dF; //导程（模态）
    double dQ; //起始角度（非模态）

    double dRadius;            //半径
    int8 cDir;                 //圆弧方向
    SPointDouble SCenterPoint; //圆心

    uint16 usL;          //螺纹头数
    bool bRepeatG32Flag; //是否为连续G32
    bool bG32LetterQ;    //是否出现Q
    uint8 ucG32SubMode;  //G32的子类	用于区分G32.2/G32.3/G32.4 陈敏聪增 2023/6/13

} SG32_2Para;

void addG32_2RegulationAPI();

int8 mallocG32_2ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG32_2ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG32_2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG32_2(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG32_2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
uint8 getQuadrant(SPointDouble SCenter, SPointDouble SPoint, uint8 ucHorAxis, uint8 ucVerAxis);
int8 checkArcG32(SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cArcDir,
                 SPointDouble* pSCenterPoint, double* pRadius, uint8 ucHorAxis, uint8 ucVerAxis);
#endif
