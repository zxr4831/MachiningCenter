#ifndef __GCODE_G33_H
#define __GCODE_G33_H

void addG33RegulationAPI();

typedef struct _SG33Para
{
    int8 cStep; //切片辅助标志

    SPointDouble SStartPoint;
    SPointDouble SEndPoint;

    double dF;           //导程（模态）
    uint16 usL;          //螺纹头数
    int8 G33RotateAxis;  //  G33 旋转轴  可选A轴或C轴
    int8 G33TappingMode; //G33攻牙模式 0:刚性攻牙 1:柔性攻牙

} SG33Para;

void addG33RegulationAPI();

int8 mallocG33ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG33ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG33Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG33(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG33(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
