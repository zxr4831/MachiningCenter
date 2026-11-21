#ifndef __GCODE_G34_H
#define __GCODE_G34_H

typedef struct _SG34Para
{
	int8 cStep;                      //切片辅助标志

	int nG32Order;                   //G32次序 

	SPointDouble SStartPoint;        //每一段螺纹的起点坐标 （赋值一段螺纹完毕时更新）
	SPointDouble SEndPoint;          //每一段螺纹的终点坐标（在获取参数阶段以及跨越退尾点时更新）

	double dR;                       //R值
	double dF;                       //螺纹导程

	uint8 ucLongAxis, ucShortAxis;   //长轴，短轴、
	double dLSAxisRatio;             //长短轴的比例（带有短轴向量）（在获取参数阶段以及跨越退尾点时更新）
	double dLongAxisVector;          //长轴向量
	bool bG34WorkFlag;                //G34指令有效运行标志位  （长轴距离不为0）
	double dJ;						 //短轴退尾量（记录退尾量后，用变锥度过渡代替退尾）
	double dK;						 //长轴退尾量
	double dTailIdealPos;             //理想退尾位置
	bool bLeftTailFlag;              //退尾预处理标志位  （开始退尾标志）
	bool bSameG32Flag;               //同一个G32标志位  （跨越退尾点的螺纹段被拆分成两段后的后一段的标志）
	double dLongAxisTmpPos;          //跨越退尾点的螺纹段被拆分成两段后的后一段在长轴方向的终点位置 

}SG34Para;


//添加G34规则结构体
void addG34RegulationAPI();
//开辟G34参数结构体（总框架）
int8 mallocG34ParaStruct(uint8 ucMode, uint8 ucChannelNum);
//释放G34参数结构体（总框架）
int8 freeG34ParaStruct(uint8 ucMode, uint8 ucChannelNum);
//获取G34参数（总框架）
int8 getG34Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);
//检查G34
int8 CheckG34(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
//赋值G34
int8 AssignG34(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

#endif

