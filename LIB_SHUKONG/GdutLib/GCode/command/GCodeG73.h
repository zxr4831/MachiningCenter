#ifndef __GCODE_G73_H
#define __GCODE_G73_H

#define G73_DEBUG (0 & (!STM32_MODE))

typedef struct _SG73Para
{
    int8 cStep; //切片辅助标志

    double dDeltaI; //X轴粗车退刀量，半径值（第一行U）
    double dDeltaK; //Z轴粗车退刀量（第一行W）

    uint16 ucD; //切削次数（第一行R）（实际切削次数需要加1。ucD为1，则循环切削2次）
    uint8 ucH;  //G73轨迹类型（第一行H）

    uint32 unNS;    //起始行号（第二行P）
    uint32 unNF;    //结束行号（第二行Q）
    double dDeltaU; //工件轮廓X轴偏移值，直径值（第二行U）
    double dDeltaW; //工件轮廓Z轴偏移值（第二行W）

    SList* pSPathList;            //原始轨迹链表（此链表记录NS-NF轨迹，记录后不做修改）
    SList* pSCurrMachinePathList; //当前加工轨迹链表（此链表记录当前加工轨迹，链表中轨迹全部写入完成后释放）

    double dRestDeltaI; //X轴粗车剩余退刀量，半径值
    double dRestDeltaK; //Z轴粗车剩余退刀量

    SPointDouble SPointA; //起刀点，即毛坯坐标
    SPointDouble SPointA1;

    uint16 usCurPos;   //当前写入实体在加工轨迹链表中的位置
    uint16 cRestCount; //当前循环切削次数

    int8 cCurrMachineDir; //当前加工方向（Ⅲ型用到）

    SGetTextStateInfo SGetTextStateInfo; //G73指令执行完成之后，下一次应执行NF之后的程序

} SG73Para;

void addG73RegulationAPI();
int8 mallocG73ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG73ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG73Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG73(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG73(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

#endif
