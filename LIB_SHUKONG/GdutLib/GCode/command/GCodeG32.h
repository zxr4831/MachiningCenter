#ifndef __GCODE_G32_H
#define __GCODE_G32_H

typedef struct _SG32Para
{	
	int8 cStep;                 //切片辅助标志

	SPointDouble SStartPoint;
	SPointDouble SEndPoint;

	double dJ;			//短轴退尾量（模态）
	double dK;			//长轴退尾量（模态）
	double dF;          //导程（模态）
	double dQ;			//起始角度（非模态）

	uint16  usL;     //螺纹头数 
	bool bRepeatG32Flag;    //是否为连续G32
	bool bG32LetterQ;       //是否出现Q
	uint8 ucG32SubMode;		//G32的子类	用于区分G32.2/G32.3/G32.4 陈敏聪增 2023/6/13

	double dSpindeSpeed;

}SG32Para;



void addG32RegulationAPI();

int8 mallocG32ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 freeG32ParaStruct(uint8 ucMode, uint8 ucChannelNum);
int8 getG32Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState);

int8 CheckG32(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG32(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 checkG32Speed(struct _SSyntaxConfiguration* SSyntaxConfig, double dF, SPointDouble SStartPoint, SPointDouble SEndPoint, uint8 ucHorAxis, uint8 ucVerAxis);
//int8 checkG32Speed(struct _SSyntaxConfiguration* SSyntaxConfig, SPointDouble SStartPoint, SPointDouble SEndPoint, uint8 ucHorAxis, uint8 ucVerAxis);

#endif
