#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"

#include "GCode.h"

static SG32_2Para* pSG32_2ParaCheck[CHANNEL_NUM_MAX];
static SG32_2Para* pSG32_2ParaAssign[CHANNEL_NUM_MAX];

//添加G32_2规则结构体
void addG32_2RegulationAPI()
{
	//指令：G32_2
	//组别：sGa
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(32.2, sGa, 1, mallocG32_2ParaStruct, freeG32_2ParaStruct, getG32_2Para, CheckG32_2, AssignG32_2);
}


int8 mallocG32_2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG32_2ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG32_2ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG32_2Para), ORAM_NUM406);//暂未对内存编号
		if (pSG32_2ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG32_2ParaCheck[ucChannelNum], 0, sizeof(SG32_2Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG32_2ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG32_2ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG32_2Para), ORAM_NUM407);
		if (pSG32_2ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG32_2ParaAssign[ucChannelNum], 0, sizeof(SG32_2Para));
		return 1;
	}
}

int8 freeG32_2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG32_2ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG32_2ParaCheck[ucChannelNum]);
			pSG32_2ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG32_2ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG32_2ParaAssign[ucChannelNum]);
			pSG32_2ParaAssign[ucChannelNum] = NULL;
		}
	}
}

int8 getG32_2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	if (ucMode == GC_SYNTAX)
	{
		
		//.获取导程
		getLeadByCoordInfo(pSScanContainer, &(pSG32_2ParaCheck[ucChannelNum]->dF));

		//.获取起始角度
		if (pSScanContainer->bQ == TRUE)
		{
			pSG32_2ParaCheck[ucChannelNum]->dQ = pSScanContainer->dQ;
			pSG32_2ParaCheck[ucChannelNum]->bG32LetterQ = TRUE;
		}

		//.获取半径R或者IK，IK和R同时出现时，以R为准
		//if (pSScanContainer->bRIJK[0] == TRUE)
		//{
		//	pSG32_2ParaCheck[ucChannelNum]->dRadius = pSScanContainer->dRIJK[0];
		//}
		//else if (pSScanContainer->bRIJK[1] == TRUE || pSScanContainer->bRIJK[3] == TRUE)
		//{
		//	if (pSScanContainer->bRIJK[1])
		//	{
		//		pSG32_2ParaCheck[ucChannelNum]->dI = pSScanContainer->dRIJK[1];
		//	}
		//	if (pSScanContainer->bRIJK[3] == TRUE)
		//	{
		//		pSG32_2ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
		//	}

		//}

		//.判断是否为连续G32
		if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			pSG32_2ParaCheck[ucChannelNum]->bRepeatG32Flag = TRUE;
		}

		//圆弧方向
		pSG32_2ParaCheck[ucChannelNum]->cDir = ARC_CW;

		//.G32子类
		pSG32_2ParaCheck[ucChannelNum]->ucG32SubMode = 2;

		//.步骤
		pSG32_2ParaCheck[ucChannelNum]->cStep = 0;

	}

	else if (ucMode == GC_ASSIGN)
	{
		
		//.获取导程
		getLeadByCoordInfo(pSScanContainer, &(pSG32_2ParaAssign[ucChannelNum]->dF));
	
		//.获取起始角度
		if (pSScanContainer->bQ == TRUE)
		{
			pSG32_2ParaAssign[ucChannelNum]->dQ = pSScanContainer->dQ;
			pSG32_2ParaAssign[ucChannelNum]->bG32LetterQ = TRUE;
		}

		////.获取半径R或者IK，IK和R同时出现时，以R为准
		//if (pSScanContainer->bRIJK[0]==TRUE)
		//{
		//	pSG32_2ParaAssign[ucChannelNum]->dRadius = pSScanContainer->dRIJK[0];
		//}
		//else if(pSScanContainer->bRIJK[1]==TRUE||pSScanContainer->bRIJK[3]==TRUE)
		//{
		//	if (pSScanContainer->bRIJK[1])
		//	{
		//		pSG32_2ParaAssign[ucChannelNum]->dI = pSScanContainer->dRIJK[1];
		//	}
		//	if (pSScanContainer->bRIJK[3] == TRUE)
		//	{
		//		pSG32_2ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
		//	}
	
		//}

		//.判断是否为连续G32
		if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			pSG32_2ParaAssign[ucChannelNum]->bRepeatG32Flag = TRUE;
		}

		//圆弧方向
		pSG32_2ParaAssign[ucChannelNum]->cDir = ARC_CW;
		//.G32子类
		pSG32_2ParaAssign[ucChannelNum]->ucG32SubMode = 2;

		//.步骤
		pSG32_2ParaAssign[ucChannelNum]->cStep = 0;


	}
}

//赋值G32
int8 AssignG32_2(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	int8 cRet = 0;
	uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
	SG32_2Para* pSG32_2Para = pSG32_2ParaAssign[pSAssignConfig->ucChannelNum];
	
	SThread SThreadInfo;//存直线段信息（可能要将此变量放在赋值结构体中）
	bool bIsAbsMethod;
	memset(&SThreadInfo, 0, sizeof(SThread));

	//1.1获取起点终点（机械坐标）
	SThreadInfo.SEndPoint = SThreadInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
	//获取当前的坐标编程方式
	bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
	//计算机械坐标
	getMacCoordByScanInfo(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &SThreadInfo.SStartPoint, &SThreadInfo.SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
	
	//2.获取圆弧方向
	SThreadInfo.cArcDir = pSG32_2Para->cDir;

	//3.计算圆心和半径
	//3.1获取横轴、纵轴
	cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSAssignConfig->SGCodeCurrSysState.cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);
	if (cRet < 0)
	{
		return -1;
	}
	//3.2获取圆心，半径
	getArcCenterPointAndRadius(&pSAssignConfig->SScanContainer, &SThreadInfo.SStartPoint, &SThreadInfo.SEndPoint, SThreadInfo.cArcDir, &SThreadInfo.SCenterPoint, &SThreadInfo.dRadius, ucHorAxis, ucVerAxis);

	//4.获取导程
	SThreadInfo.dF = pSG32_2Para->dF;
    
	//5.获取起始角Q
	//起始角Q
	SThreadInfo.dQ = pSG32_2Para->dQ;
	SThreadInfo.bG32LetterQ = pSG32_2Para->bG32LetterQ;

	//6.连续G32标志位
	SThreadInfo.bRepeatG32Flag = pSG32_2Para->bRepeatG32Flag;
	//7.实体类型
	SThreadInfo.EntityType = THREAD;
	//8.子类型
	SThreadInfo.ucG32SubMode = pSG32_2Para->ucG32SubMode;

	//.将SThread写入缓冲区
	writeThreadIntoGCodeBuf(&SThreadInfo, pGCodeBufUnit);
	return 2;
}


// 检查G32
int8 CheckG32_2(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	//0.判断G3程序段中是否出现坐标信息 
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		SG32_2Para* pSG32_2Para = pSG32_2ParaCheck[pSSyntaxConfig->ucChannelNum];
		int8 cRet = 0;
		//1.指令字符检查
		checkParaLetter("XYZUVWRKIF", &pSSyntaxConfig->SScanContainer);

		//2.参数值范围检查
		checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('K', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('I', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('F', &pSSyntaxConfig->SScanContainer, 0.001, 500);
		checkParaRange('Q', &pSSyntaxConfig->SScanContainer, 0, 360000);

		//出现坐标关键字才检查
		//0.判断G92程序段中是否出现坐标信息
		cRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
		//没有出现，该G32为被动继承，直接跳过
		if (cRet == FALSE)
		{
			return 2;
		}


		//2.检查转速 导程
		if (!(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed > 0 && pSG32_2Para->dF > 0))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM405);
		}

		//3.根据加工参数的来源检查长轴、短轴以当前的最大速度能否追上主轴
		uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
		//1.1初始化起点与终点
		pSG32_2Para->SStartPoint = pSG32_2Para->SEndPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		bool bIsAbsMethod;
		//获取终点
		bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
		getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSG32_2Para->SStartPoint, &pSG32_2Para->SEndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
		//获取横轴、纵轴
		cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pSSyntaxConfig->SCurrRecord.SModalInfo.sGc);
		if (cRet < 0)
		{
			return -1;
		}
		cRet = checkG32Speed(pSSyntaxConfig, pSG32_2Para->dF, pSG32_2Para->SStartPoint, pSG32_2Para->SEndPoint, ucHorAxis, ucVerAxis);
		if (cRet < 0)
		{
			return -1;
		}

		//4.检查圆弧的合法性（确保长短轴不变）
		cRet = checkArcG32(&pSSyntaxConfig->SScanContainer, &pSG32_2Para->SStartPoint, &pSG32_2Para->SEndPoint, pSG32_2Para->cDir, &pSG32_2Para->SCenterPoint, &pSG32_2Para->dRadius, ucHorAxis, ucVerAxis);
		if (cRet < 0)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM434);
			return -1;
		}

		//5.更新起点 终点
		pGCodeBufUnit->SStartPoint = pSG32_2Para->SStartPoint;
		pGCodeBufUnit->SEndPoint = pSG32_2Para->SEndPoint;
		return 2;
	}
	return 2;
}

/*************************************************
  函数名称int8 checkArcG32(SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cArcDir, SPointDouble* pSCenterPoint, double* pRadius, uint8 ucHorAxis, uint8 ucVerAxis)
  函数功能:检查圆弧G32的圆弧合法性
  输入参数：
  输出参数：
  返 回 值：-1 报错
            1 正确

  备    注：
**************************************************/
int8 checkArcG32(SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cArcDir, SPointDouble* pSCenterPoint, double* pRadius, uint8 ucHorAxis, uint8 ucVerAxis)
{
	if (pSScanContainer == NULL || pSStartPoint == NULL || pSEndPoint == NULL || pSCenterPoint == NULL || pRadius == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	double dRad = 0;   //弧度
	uint8 ucStartQuadrant;   //起点象限
	const double halfPI = 1.5707963268;   //π/2对应弧度
	double HorPt;    //横轴45度边界交点值
	double VerPt;    //纵轴45度边界交点值
	int8 cHorSign = 1, cVerSign = 1;  //边界交点值的符号，默认第一象限边界
	double dHorS;   //起点相对圆心的横轴距离 
	double dVerS;   //起点相对圆心的纵轴距离
	double radStart2Edge;  //起点到边界的弧度

	//计算圆心和半径
	dRad=getArcCenterPointAndRadius(pSScanContainer, pSStartPoint, pSEndPoint, cArcDir, pSCenterPoint, pRadius, ucHorAxis, ucVerAxis);
	//还要检查长短轴是否变化
	if (dRad > halfPI)
	{
		return -1;
	}

	if (isZero(dRad))  //弧度太小
	{
		return -1;
	}

	ucStartQuadrant = getQuadrant(*pSCenterPoint, *pSStartPoint, ucHorAxis, ucVerAxis); //判断起点象限
	HorPt = 0.70710678118654752 * (*pRadius);  //根号2/2*半径 = 45度边界交点值
	VerPt = HorPt;
	dHorS = pSStartPoint->dArray[ucHorAxis] - pSCenterPoint->dArray[ucHorAxis];
	dVerS = pSStartPoint->dArray[ucVerAxis] - pSCenterPoint->dArray[ucVerAxis];
	if (ucStartQuadrant == 1)
	{
		if (cArcDir == ARC_CW) //顺圆
		{
			if (!(dHorS < HorPt))  //边界右方
			{
				cVerSign = -1;   //取第四象限边界
			}
		}
		else  //逆圆
		{
			if (!(dHorS > HorPt))  //边界左方
			{
				cHorSign = -1;   //取第二象限边界
			}
		}
	}
	else if (ucStartQuadrant == 2)
	{
		if (cArcDir == ARC_CW) //顺圆
		{
			if (dHorS < -HorPt)   //边界右方
			{
				cHorSign = -1;   //取第二象限边界
			}

		}
		else  //逆圆
		{
			if (dHorS > -HorPt)   //边界右方
			{
				cHorSign = -1;   //取第二象限边界
			}
			else    //边界左方
			{
				cHorSign = -1;
				cVerSign = -1;   //取第三象限边界
			}
		}
	}
	else if (ucStartQuadrant == 3)
	{
		if (cArcDir == ARC_CW) //顺圆
		{
			if (dHorS > -HorPt)     //边界右方
			{
				cHorSign = -1;
				cVerSign = -1;   //取第三象限边界
			}
			else   //边界左方
			{
				cHorSign = -1;   //取第二象限边界
			}
		}
		else  //逆圆
		{
			if (dHorS < -HorPt)   //边界左方
			{
				cHorSign = -1;
				cVerSign = -1;   //取第三象限边界
			}
			else    //边界右方
			{
				cVerSign = -1;   //取第四象限边界
			}
		}
	}
	else
	{
		if (cArcDir == ARC_CW)//顺圆
		{
			if (dHorS > HorPt)   //边界右方
			{
				cVerSign = -1;   //取第四象限边界
			}
			else   //边界左方
			{
				cHorSign = -1;
				cVerSign = -1;   //取第三象限边界
			}

		}
		else  //逆圆
		{
			if (dHorS < HorPt)   //边界左方
			{
				cVerSign = -1;   //取第四象限边界
			}

		}

	}
	radStart2Edge = acos((dHorS * cHorSign + dVerS * cVerSign) / ((*pRadius) * 1.414213562373095));  //与边界上的点（cHorSign*1,cVerSign*1）计算求弧度

	if (dRad > radStart2Edge)  //圆弧弧度大于起点到交界线的弧度，报错
	{
		return -1;    //弧度大于π/2,报错
	}
	return 1;
}


/*************************************************
  函数名称：uint8 getQuadrant(SPointDouble SCenter, SPointDouble SPoint,uint8 ucHorAxis, uint8 ucVerAxis)
  函数功能: 判断象限
  输入参数：SCenter圆心，SPoint起点，ucHorAxis横轴，ucVerAxis纵轴
  输出参数：
  返 回 值：象限号，例如1是第一象限

  备    注：
**************************************************/
 uint8 getQuadrant(SPointDouble SCenter, SPointDouble SPoint, uint8 ucHorAxis, uint8 ucVerAxis)
{
	if (SCenter.dArray[ucHorAxis] < SPoint.dArray[ucHorAxis]   //Z
		&& (SCenter.dArray[ucVerAxis] < SPoint.dArray[ucVerAxis]  //X
			|| isZero(SCenter.dArray[ucVerAxis] - SPoint.dArray[ucVerAxis])))  //[0,90)
	{
		return 1;
	}
	else if ((SCenter.dArray[ucHorAxis] > SPoint.dArray[ucHorAxis]
		|| isZero(SCenter.dArray[ucHorAxis] - SPoint.dArray[ucHorAxis]))
		&& SCenter.dArray[ucVerAxis] < SPoint.dArray[ucVerAxis]) //[90,180)
	{
		return 2;
	}
	else if (SCenter.dArray[ucHorAxis] > SPoint.dArray[ucHorAxis]
		&& (SCenter.dArray[ucVerAxis] > SPoint.dArray[ucVerAxis]
			|| isZero(SCenter.dArray[ucVerAxis] - SPoint.dArray[ucVerAxis]))) //[180,270)
	{
		return 3;
	}
	else  //[270,0)
	{
		return 4;
	}
}