#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"

#include "GCode.h"

static SG32Para* pSG32ParaCheck[CHANNEL_NUM_MAX];
static SG32Para* pSG32ParaAssign[CHANNEL_NUM_MAX];

//添加G32规则结构体
void addG32RegulationAPI()
{
	//指令：G32
	//组别：sGa
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(32, sGa, 1, mallocG32ParaStruct, freeG32ParaStruct, getG32Para, CheckG32, AssignG32);
}


int8 mallocG32ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG32ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG32ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG32Para), ORAM_NUM404);//暂未对内存编号
		if (pSG32ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG32ParaCheck[ucChannelNum], 0, sizeof(SG32Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG32ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG32ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG32Para), ORAM_NUM405);
		if (pSG32ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG32ParaAssign[ucChannelNum], 0, sizeof(SG32Para));
		return 1;
	}
}

int8 freeG32ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG32ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG32ParaCheck[ucChannelNum]);
			pSG32ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG32ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG32ParaAssign[ucChannelNum]);
			pSG32ParaAssign[ucChannelNum] = NULL;
		}
	}
}

int8 getG32Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	// 本行单位在取参后才更新，显式 F 优先采用本行 G20/G21；省略时继承已生效单位。
	bool bFImperial = SGCodeCurrSysState->bImperialProgramming;
	const SGCommandRegulation * pUnitRegulation = pSScanContainer->SScanModalInfo.pSGeRegulation;
	if (pUnitRegulation != NULL)
	{
		bFImperial = pUnitRegulation->nCommandVal == G20 * pow(10, GCOMMAND_DECIMAL_PLACE);
	}

	if (ucMode == GC_SYNTAX)
	{
		//1.手册要求,获取短轴退尾量dJ 、 长轴退尾量dK
		//1.1 G32省略退尾时，有默认退尾值，为面板上的默认参数
		if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == FALSE)
		{
			//J、K是模态代码，上一个程序段为G32，当前程序段的G32省略J、K时，按前面的J、K值进行退尾，
			if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{

			}
			//省略J或J、K时且不是连续螺纹的下一段，无退尾；
			else
			{
				pSG32ParaCheck[ucChannelNum]->dJ = 0;
				pSG32ParaCheck[ucChannelNum]->dK = 0;
			}
		}
		//1.2 省略J时，无退尾
		else if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG32ParaCheck[ucChannelNum]->dJ = g_SSystemPara.dJDefault;
			pSG32ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
		}
		//1.3  省略K时，按K=J退尾；
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG32ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
			pSG32ParaCheck[ucChannelNum]->dK = pSG32ParaCheck[ucChannelNum]->dJ;
		}
		//1.4  J和K都不省略时
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == TRUE)
		{
			//3.4.1 J=0或J=0、K=0时，无退尾； J=0，K≠0时，无退尾；-》   即J=0时无退尾
			if (isZero(pSScanContainer->dRIJK[2]) == TRUE)
			{
				pSG32ParaCheck[ucChannelNum]->dJ = 0;
				pSG32ParaCheck[ucChannelNum]->dK = 0;
			}
			//3.4.2 J≠0，K=0时，按J=K退尾；
			else if (isZero(pSScanContainer->dRIJK[2]) == FALSE && isZero(pSScanContainer->dRIJK[3]) == TRUE)
			{
				//pSG32ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
				//pSG32ParaCheck[ucChannelNum]->dJ = pSG32ParaCheck[ucChannelNum]->dK;
				pSG32ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG32ParaCheck[ucChannelNum]->dK = pSG32ParaCheck[ucChannelNum]->dJ;

			}
			else
			{
				pSG32ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG32ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
			}

		}
		//2.获取导程
		if (pSScanContainer->bF == TRUE)
		{
			//pSG32ParaCheck[ucChannelNum]->dF = pSScanContainer->dF;
			pSG32ParaCheck[ucChannelNum]->dF = pSScanContainer->dF * (bFImperial ? 25.4 : 1.0);
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			pSG32ParaCheck[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
		}



		//3.获取起始角度
		if (pSScanContainer->bQ == TRUE)
		{
			pSG32ParaCheck[ucChannelNum]->dQ = pSScanContainer->dQ;
			pSG32ParaCheck[ucChannelNum]->bG32LetterQ = TRUE;
		}

		//4.判断是否为连续G32
		if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			pSG32ParaCheck[ucChannelNum]->bRepeatG32Flag = TRUE;
		}

		//5.G32子类
		pSG32ParaCheck[ucChannelNum]->ucG32SubMode = 0;

		//6.步骤
		pSG32ParaCheck[ucChannelNum]->cStep = 0;
		
	}

	else if (ucMode == GC_ASSIGN)
	{
		//1.手册要求,获取短轴退尾量dJ 、 长轴退尾量dK
		//1.1 G32省略退尾时，有默认退尾值，为面板上的默认参数
		if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == FALSE)
		{
			//J、K是模态代码，上一个程序段为G32，当前程序段的G32省略J、K时，按前面的J、K值进行退尾，
			if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{

			}
			//省略J或J、K时且不是连续螺纹的下一段，无退尾；
			else
			{
				pSG32ParaAssign[ucChannelNum]->dJ = 0;
				pSG32ParaAssign[ucChannelNum]->dK = 0;
			}
		}
		//1.2 省略J时，无退尾
		else if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG32ParaAssign[ucChannelNum]->dJ = g_SSystemPara.dJDefault;
			pSG32ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
		}
		//1.3  省略K时，按K=J退尾；
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG32ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
			pSG32ParaAssign[ucChannelNum]->dK = pSG32ParaAssign[ucChannelNum]->dJ;
		}
		//1.4  J和K都不省略时
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == TRUE)
		{
			//3.4.1 J=0或J=0、K=0时，无退尾； J=0，K≠0时，无退尾；-》   即J=0时无退尾
			if (isZero(pSScanContainer->dRIJK[2]) == TRUE)
			{
				pSG32ParaAssign[ucChannelNum]->dJ = 0;
				pSG32ParaAssign[ucChannelNum]->dK = 0;
			}
			//3.4.2 J≠0，K=0时，按J=K退尾；
			else if (isZero(pSScanContainer->dRIJK[2]) == FALSE && isZero(pSScanContainer->dRIJK[3]) == TRUE)
			{
				//pSG32ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
				//pSG32ParaAssign[ucChannelNum]->dJ = pSG32ParaAssign[ucChannelNum]->dK;
				pSG32ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG32ParaAssign[ucChannelNum]->dK = pSG32ParaAssign[ucChannelNum]->dJ;
			}
			else
			{
				pSG32ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG32ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
			}

		}
		//2.获取导程
		if (pSScanContainer->bF==TRUE)
		{
			//pSG32ParaAssign[ucChannelNum]->dF = pSScanContainer->dF;
			pSG32ParaAssign[ucChannelNum]->dF = pSScanContainer->dF * (bFImperial ? 25.4 : 1.0);
		}
		else if(pSScanContainer->bRIJK[1]==TRUE)
		{
			pSG32ParaAssign[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
		}



		//3.获取起始角度
		if (pSScanContainer->bQ == TRUE)
		{
			pSG32ParaAssign[ucChannelNum]->dQ = pSScanContainer->dQ;
			pSG32ParaAssign[ucChannelNum]->bG32LetterQ = TRUE;
		}

		//5.G32子类
		pSG32ParaAssign[ucChannelNum]->ucG32SubMode = 0;

		//6.步骤
		pSG32ParaAssign[ucChannelNum]->cStep = 0;


	}
}

//检查G32
int8 CheckG32(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	bool bRet = 0;
	SG32Para* pSG32Para = pSG32ParaCheck[pSSyntaxConfig->ucChannelNum];
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);

	// 单独一行指定的导程也会被下一段继承，须在判断坐标之前校验。
	if (pSSyntaxConfig->SScanContainer.bF == TRUE && !isBetweenA2B(pSG32Para->dF, 0.001, 500.0))
	{
	    parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM130);
		return -1;
	}

	if (bRet == TRUE)
	{
		int8 cRet = 0;
		//1.指令字符检查
		checkParaLetter("XYZUVWRJKFIQ", &pSSyntaxConfig->SScanContainer);

		//2.参数值范围检查
		checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('J', &pSSyntaxConfig->SScanContainer, -99999999 * TEMP_LEAST_INPUT_INCREMENT, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
		checkParaRange('K', &pSSyntaxConfig->SScanContainer, -99999999 * TEMP_LEAST_INPUT_INCREMENT, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
		//checkParaRange('F', &pSSyntaxConfig->SScanContainer, 0.001, 500);
		checkParaRange('I', &pSSyntaxConfig->SScanContainer, 0.06, 25400);
		checkParaRange('Q', &pSSyntaxConfig->SScanContainer, 0, 360000);

		//1.检查I和F的互斥性
		if (pSSyntaxConfig->SScanContainer.bF == TRUE && pSSyntaxConfig->SScanContainer.bRIJK[1] == TRUE)
		{
			const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
			while (*pStartPtr != 'r' && *pStartPtr != 'R')
			{
				pStartPtr += 1;
			}
			setErrorType(GCODE_ERROR_NUM410);
			findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
			return -1;
		}
		//2.检查转速 导程
		if (!(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed > 0 && pSG32Para->dF > 0))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM405);
			return -1;
		}

		//3.根据加工参数的来源检查长轴、短轴以当前的最大速度能否追上主轴
		uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
		//1.1初始化起点与终点
		pSG32Para->SStartPoint = pSG32Para->SEndPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		//获取终点
		getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSG32Para->SStartPoint, &pSG32Para->SEndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
		//获取横轴、纵轴
		cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pSSyntaxConfig->SCurrRecord.SModalInfo.sGc);
		if (cRet < 0)
		{
			return -1;
		}
		//cRet = checkG32Speed(pSSyntaxConfig, pSG32Para->SStartPoint, pSG32Para->SEndPoint, ucHorAxis, ucVerAxis);
		cRet = checkG32Speed(pSSyntaxConfig, pSG32Para->dF, pSG32Para->SStartPoint, pSG32Para->SEndPoint, ucHorAxis, ucVerAxis);
		if (cRet < 0)
		{
			return -1;
		}
		//4.更新起点 终点
		pGCodeBufUnit->SStartPoint = pSG32Para->SStartPoint;
		pGCodeBufUnit->SEndPoint = pSG32Para->SEndPoint;
		return 2;
	}
	return 2;
}

//赋值G32
int8 AssignG32(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SG32Para* pSG32Para = pSG32ParaAssign[pSAssignConfig->ucChannelNum];
	// 无坐标的模态段保留原有 I/O，不生成螺纹运动或重复累计退尾坐标。
    if (!isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer))
	{
		SLine SLineInfo;
		memset(&SLineInfo, 0, sizeof(SLineInfo));
		SLineInfo.SStartPoint = SLineInfo.SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		SLineInfo.EntityType = OTHERTYPE;
		writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);
		// 本行没有螺纹运动，下一刀须重新取得主轴同步信号。
		pSG32Para->bRepeatG32Flag = FALSE;
		return 2;
	}

	//1.填写SThreadInfo
	SThread SThreadInfo;//存直线段信息（可能要将此变量放在赋值结构体中）
	bool bIsAbsMethod;
	memset(&SThreadInfo, 0, sizeof(SThread));
	//1.1获取SLineInfo的起点终点（机械坐标）
	SThreadInfo.SEndPoint = SThreadInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
	//计算机械坐标
	getMacCoordByScanInfo(pSAssignConfig->ucChannelNum,&pSAssignConfig->SScanContainer, &SThreadInfo.SStartPoint, &SThreadInfo.SEndPoint, &pSAssignConfig->SGCodeCurrSysState);

	//导程
	SThreadInfo.dF = pSG32Para->dF;
	
	//起始角Q
	SThreadInfo.dQ = pSG32Para->dQ;
	SThreadInfo.bG32LetterQ = pSG32Para->bG32LetterQ;
	//退尾量
	SThreadInfo.dJ = pSG32Para->dJ;
	SThreadInfo.dK = pSG32Para->dK;
	//连续G32标志位
	SThreadInfo.bRepeatG32Flag = pSG32Para->bRepeatG32Flag;
	//实体类型
	SThreadInfo.EntityType = THREAD;
	//子类型
	SThreadInfo.ucG32SubMode = pSG32Para->ucG32SubMode;

	//2.将SThread写入缓冲区
	writeThreadIntoGCodeBuf(&SThreadInfo, pGCodeBufUnit);
	// 只有本行写出螺纹运动，紧接的下一刀才使用连续切削状态。
	pSG32Para->bRepeatG32Flag = TRUE;

	return 2;

}

//根据加工参数的来源检查长轴、短轴以当前的最大速度能否追上主轴
//int8 checkG32Speed(struct _SSyntaxConfiguration* SSyntaxConfig, SPointDouble SStartPoint, SPointDouble SEndPoint, uint8 ucHorAxis, uint8 ucVerAxis)
int8 checkG32Speed(struct _SSyntaxConfiguration* SSyntaxConfig, double dF, SPointDouble SStartPoint, SPointDouble SEndPoint, uint8 ucHorAxis, uint8 ucVerAxis)
{

	double dLengthHors = 0;
	double dLengthVers = 0;
	uint8 ucLongAxisNum;   //长轴编号
	uint8 ucShortAxisNum;  //短轴编号
	double dSpindeSpeed = 0;
	double dRatioLongAndShort = 0;  //长短轴的比例（螺纹锥度）


	dLengthHors = fabs(SStartPoint.dArray[ucHorAxis] - SEndPoint.dArray[ucHorAxis]);
	dLengthVers = fabs(SStartPoint.dArray[ucVerAxis] - SEndPoint.dArray[ucVerAxis]);
	ucLongAxisNum = dLengthHors >= dLengthVers ? ucHorAxis : ucVerAxis;
	ucShortAxisNum = ucLongAxisNum == ucHorAxis ? ucVerAxis : ucHorAxis;

	dSpindeSpeed = SSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed / 60.0 / 1000.0;

	if (!isZero(dLengthHors) && !isZero(dLengthVers))
	{
		dRatioLongAndShort = fabs(dLengthHors / dLengthVers) > 1 ? (dLengthVers / dLengthHors) : (dLengthHors / dLengthVers);
	}
	//G32系统参数来源，0为加工系统参数，
	if (g_SSystemPara.ucG32InputMode == 0)  //2023.5.19
	{
		//
		if (g_SSystemPara.dSysMaxVel < dSpindeSpeed * dF * 1.05)
		{
			parseGCodeErrorB(SSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM454);
			return -1;
		}

	}
	//G32系统参数来源，，1为对应单轴的系统参数  
	else
	{
		if (g_SSystemPara.SAxisPara[ucLongAxisNum].dVmax < dSpindeSpeed * dF * 1.05)
		{
			parseGCodeErrorB(SSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM454);
			return -1;
		}
		if (g_SSystemPara.SAxisPara[ucShortAxisNum].dVmax < dSpindeSpeed * dF * 1.05 * dRatioLongAndShort)
		{
			parseGCodeErrorB(SSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM455);
			return -1;
		}

	}

	return 1;

}