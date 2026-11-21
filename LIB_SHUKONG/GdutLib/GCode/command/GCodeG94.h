#ifndef __GCODE_G94_H
#define __GCODE_G94_H

typedef struct _SG94Para
{
    int8 cStep;          //切片辅助标志
    double dR;           //R值
    double dRRaw;        // 新输入的 R 原始值，单位随编程单位。
    bool bRNeedsConvert; // 新 R 待处理标志，本行单位生效后处理一次。

    SCoord SCoordInfo;    //程序段坐标信息
    SPointDouble SPointA; //A点坐标（机械坐标）
    SPointDouble SPointC; //C点坐标（机械坐标）

} SG94Para;

void addG94RegulationAPI();

//开辟G94参数结构体
int8 mallocG94ParaStruct(uint8 ucMode, uint8 ucChannelNum);
//释放G94参数结构体
int8 freeG94ParaStruct(uint8 ucMode, uint8 ucChannelNum);

//获取G94参数
int8 getG94Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

//检查G94
int8 CheckG94(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

//赋值G94
int8 AssignG94(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif
