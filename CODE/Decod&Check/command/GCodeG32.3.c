#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"

#include "GCode.h"

static SG32_3Para* pSG32_3ParaCheck[CHANNEL_NUM_MAX];
static SG32_3Para* pSG32_3ParaAssign[CHANNEL_NUM_MAX];

//添加G32_3规则结构体
void addG32_3RegulationAPI()
{
	addGCommadRegulation(32.3, sGa, 1, mallocG32_3ParaStruct, freeG32_3ParaStruct, getG32_3Para, CheckG32_3, AssignG32_3);
}


int8 mallocG32_3ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG32_3ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG32_3ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG32_3Para), ORAM_NUM408);//暂未对内存编号
		if (pSG32_3ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG32_3ParaCheck[ucChannelNum], 0, sizeof(SG32_3Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG32_3ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG32_3ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG32_3Para), ORAM_NUM409);
		if (pSG32_3ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG32_3ParaAssign[ucChannelNum], 0, sizeof(SG32_3Para));
		return 1;
	}
}

int8 freeG32_3ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG32_3ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG32_3ParaCheck[ucChannelNum]);
			pSG32_3ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG32_3ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG32_3ParaAssign[ucChannelNum]);
			pSG32_3ParaAssign[ucChannelNum] = NULL;
		}
	}
	return 1;
}

int8 getG32_3Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	if (ucMode == GC_SYNTAX)
	{

		//.获取导程
		getLeadByCoordInfo(pSScanContainer, &(pSG32_3ParaCheck[ucChannelNum]->dF));

		//.获取起始角度
		if (pSScanContainer->bQ == TRUE)
		{
			pSG32_3ParaCheck[ucChannelNum]->dQ = pSScanContainer->dQ;
			pSG32_3ParaCheck[ucChannelNum]->bG32LetterQ = TRUE;
		}


		//.判断是否为连续G32
		if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			pSG32_3ParaCheck[ucChannelNum]->bRepeatG32Flag = TRUE;
		}

		//圆弧方向
		pSG32_3ParaCheck[ucChannelNum]->cDir = ARC_ANTI_CW;

		//.G32子类
		pSG32_3ParaCheck[ucChannelNum]->ucG32SubMode = 3;

		//.步骤
		pSG32_3ParaCheck[ucChannelNum]->cStep = 0;

	}

	else if (ucMode == GC_ASSIGN)
	{

		//.获取导程
		getLeadByCoordInfo(pSScanContainer, &(pSG32_3ParaAssign[ucChannelNum]->dF));

		//.获取起始角度
		if (pSScanContainer->bQ == TRUE)
		{
			pSG32_3ParaAssign[ucChannelNum]->dQ = pSScanContainer->dQ;
			pSG32_3ParaAssign[ucChannelNum]->bG32LetterQ = TRUE;
		}


		//.判断是否为连续G32
		if (SGCodeCurrSysState->SModalInfo.sGa == G32 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			pSG32_3ParaAssign[ucChannelNum]->bRepeatG32Flag = TRUE;
		}

		//圆弧方向
		pSG32_3ParaAssign[ucChannelNum]->cDir = ARC_ANTI_CW;
		//.G32子类
		pSG32_3ParaAssign[ucChannelNum]->ucG32SubMode =3;

		//.步骤
		pSG32_3ParaAssign[ucChannelNum]->cStep = 0;


	}
}

//赋值G32
int8 AssignG32_3(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	int8 cRet = 0;
	uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
	SG32_3Para* pSG32_3Para = pSG32_3ParaAssign[pSAssignConfig->ucChannelNum];

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
	SThreadInfo.cArcDir = pSG32_3Para->cDir;

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
	SThreadInfo.dF = pSG32_3Para->dF;

	//5.获取起始角Q
	//起始角Q
	SThreadInfo.dQ = pSG32_3Para->dQ;
	SThreadInfo.bG32LetterQ = pSG32_3Para->bG32LetterQ;

	//6.连续G32标志位
	SThreadInfo.bRepeatG32Flag = pSG32_3Para->bRepeatG32Flag;
	//7.实体类型
	SThreadInfo.EntityType = THREAD;
	//8.子类型
	SThreadInfo.ucG32SubMode = pSG32_3Para->ucG32SubMode;

	//.将SThread写入缓冲区
	writeThreadIntoGCodeBuf(&SThreadInfo, pGCodeBufUnit);
	return 2;
}


// 检查G32
int8 CheckG32_3(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		SG32_3Para* pSG32_3Para = pSG32_3ParaCheck[pSSyntaxConfig->ucChannelNum];
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
		if (!(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed > 0 && pSG32_3Para->dF > 0))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM405);
		}

		//3.根据加工参数的来源检查长轴、短轴以当前的最大速度能否追上主轴
		uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
		//1.1初始化起点与终点
		pSG32_3Para->SStartPoint = pSG32_3Para->SEndPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		bool bIsAbsMethod;
		//获取终点
		bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
		getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSG32_3Para->SStartPoint, &pSG32_3Para->SEndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
		//获取横轴、纵轴
		cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pSSyntaxConfig->SCurrRecord.SModalInfo.sGc);
		if (cRet < 0)
		{
			return -1;
		}
		cRet = checkG32Speed(pSSyntaxConfig, pSG32_3Para->dF, pSG32_3Para->SStartPoint, pSG32_3Para->SEndPoint, ucHorAxis, ucVerAxis);
		if (cRet < 0)
		{
			return -1;
		}

		//4.检查圆弧的合法性（确保长短轴不变）

		cRet = checkArcG32(&pSSyntaxConfig->SScanContainer, &pSG32_3Para->SStartPoint, &pSG32_3Para->SEndPoint, pSG32_3Para->cDir, &pSG32_3Para->SCenterPoint, &pSG32_3Para->dRadius, ucHorAxis, ucVerAxis);
		if (cRet < 0)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM434);
			return -1;
		}

		//5.更新起点 终点
		pGCodeBufUnit->SStartPoint = pSG32_3Para->SStartPoint;
		pGCodeBufUnit->SEndPoint = pSG32_3Para->SEndPoint;
		return 2;
	}
	return 2;
}

