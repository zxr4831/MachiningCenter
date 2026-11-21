#ifndef _GCODE_G76_H_
#define _GCODE_G76_H_

typedef struct _SG76Para
{
    int8 cStep;        //切片辅助标志
    SCoord SCoordInfo; //程序段坐标信息
    //G76第一行参数   G76指令第1行 P(m)(r)(a) Q(△dmin) R(d) J_ K_；（第一行参数可以全部省略） 第一行和第二行的区别在于第二行有F（I）
    uint8 ucPm;    //螺纹精车次数P(m)
    uint8 ucPr;    //螺纹退尾系数P（r）
    uint8 ucPa;    //相邻两牙螺纹的夹角，取值范围为00～99，单位：度(°)   P（a）
    double dQdmin; //螺纹粗车时的最小切削量 Q(△dmin)
    double dRd;    //螺纹精车余量    R(d)
    double dJ;     // 短轴退尾量
    double dK;     //长轴退尾量

    bool bCalculateBackTailByJK; //退尾量由JK参数计算标志位
    bool
        bCalculateBackTailByPr; //退尾量由Pr参数计算标志位  在没有JK的前提下，螺纹退尾量=Pr*0.1*F，但F又是出现在第二行，所以先记录Pr的值以及出现标志，在第二行参数获取时再计算。但是如果出现JK。Pr就没参与计算了

    //关于G76的退尾量有3个来源 ：1.参数J K；2.参数P（r）；3.默认系统参数  优先级次序从高到低为 1.2.3

    //G76第二行参数    G76 X(U)_ Z(W)_ R(i) P(k)_ Q(△d)_ F(I)_ ；
    double dRi; //螺纹锥度
    double dPk; //螺纹牙高
    double dQd; //第一次螺纹切削深度
    double dF;  //螺纹导程

    //轨迹点计算所需的关键点
    SPointDouble SPointA;  //起点坐标
    SPointDouble SPointB;  //螺纹切深参考点
    SPointDouble SPointD;  //螺纹终点坐标
    SPointDouble SPointBn; //粗车螺纹切削起点

    //轨迹计算所需的变量
    int nCurCutTime;        //当前的切削次数
    double dThreadCutSlope; //螺纹切削斜率
    bool bFineCutFlag;      //精车标志位（区分粗、精车）

    //方向系数    A—>D
    int8 cDir_X; //X轴的加工方向  1：沿X坐标轴负向向正向     -1：沿X坐标轴正向向负向
    int8 cDir_Z; //Z轴的加工方向  1.：沿Z坐标轴负向向正向   -1： 沿Z坐标轴正向向负向

} SG76Para;

void addG76RegulationAPI();

int8 mallocG76ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG76ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG76Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG76(struct _SSyntaxConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
int8 AssignG76(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

#endif // !_GCODE76_H_
