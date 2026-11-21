#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"

#include "GCode.h"

static SG33Para* pSG33ParaCheck[CHANNEL_NUM_MAX];
static SG33Para* pSG33ParaAssign[CHANNEL_NUM_MAX];

//添加G33规则结构体
void addG33RegulationAPI()
{
	addGCommadRegulation(33, sGa, 1, mallocG33ParaStruct, freeG33ParaStruct, getG33Para, CheckG33, AssignG33);
}

int8 mallocG33ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG33ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG33ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG33Para), ORAM_NUM410);//暂未对内存编号
		if (pSG33ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG33ParaCheck[ucChannelNum], 0, sizeof(SG33Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG33ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG33ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG33Para), ORAM_NUM411);
		if (pSG33ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG33ParaAssign[ucChannelNum], 0, sizeof(SG33Para));
		return 1;
	}
}

int8 freeG33ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG33ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG33ParaCheck[ucChannelNum]);
			pSG33ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG33ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG33ParaAssign[ucChannelNum]);
			pSG33ParaAssign[ucChannelNum] = NULL;
		}
	}
}

//获取G33参数
int8 getG33Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	if (ucMode == GC_SYNTAX)
	{

		//1.获取导程
		getLeadByCoordInfo(pSScanContainer, &(pSG33ParaCheck[ucChannelNum]->dF));

		//2.获取头数
		if (pSScanContainer->bL == TRUE)
		{
			pSG33ParaCheck[ucChannelNum]->usL = pSScanContainer->dL;
		}
		else
		{
			pSG33ParaCheck[ucChannelNum]->usL = 1;
		}

	}
		

	else if (ucMode == GC_ASSIGN)
	{
		//1.获取导程
		getLeadByCoordInfo(pSScanContainer, &(pSG33ParaAssign[ucChannelNum]->dF));

		//2.获取头数
		if (pSScanContainer->bL == TRUE)
		{
			pSG33ParaAssign[ucChannelNum]->usL = pSScanContainer->dL;
		}
		//默认值1
		else
		{
			pSG33ParaAssign[ucChannelNum]->usL = 1;
		}


	}
}

//检查G33
int8 CheckG33(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	//0.判断G33程序段中是否出现坐标信息 
	int8 cRet = 0;
	cRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (cRet == TRUE)
	{
		SG33Para* pSG33Para = pSG33ParaCheck[pSSyntaxConfig->ucChannelNum];
		int8 cRet = 0;
		//1.指令字符检查
		checkParaLetter("ZWLFI", &pSSyntaxConfig->SScanContainer);

		//2.参数值范围检查
		checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('F', &pSSyntaxConfig->SScanContainer, 0.001, 500);
		checkParaRange('I', &pSSyntaxConfig->SScanContainer, 0.06, 25400);
		checkParaRange('L', &pSSyntaxConfig->SScanContainer, 1, 99);

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
		if (!(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed > 0 && pSG33Para->dF > 0))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM405);
		}

		//3.1.检查转速 导程
		if (!(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed > 0 && pSG33Para->dF > 0))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM405);
		}
		//3.2检查以当前的最大速度能否追上主轴
		if (!(pSSyntaxConfig->SCurrRecord.dSpindleSpeed * pSG33Para->dF / 60000 < g_SSystemPara.dG33MaxVel))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM434);
		}

		//4.更新起点 终点 (G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹,直接终点==起点)
		pSG33Para->SStartPoint = pSG33Para->SEndPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		pGCodeBufUnit->SStartPoint = pSG33Para->SStartPoint;
		pGCodeBufUnit->SEndPoint = pSG33Para->SEndPoint;
		return 2;
	}

	return 2;

}

//赋值G33
int8 AssignG33(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig==NULL||pGCodeBufUnit==NULL)
	{
		return -1;
	}

	//0.判断G33程序段中是否出现坐标信息 
	int8 cRet = 0;
	cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
	if (cRet == TRUE)
	{
		bool bIsAbsMethod;
		SG33Para* pSG33Para = pSG33ParaAssign[pSAssignConfig->ucChannelNum];
		//1.创建STapping并填写
		STapping STappingInfo;
		memset(&STappingInfo, 0, sizeof(STapping));
		//2初始化起点终点（机械坐标）
		STappingInfo.SEndPoint = STappingInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//获取当前的坐标编程方式
		bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
		//计算机械坐标
		getMacCoordByScanInfo(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &STappingInfo.SStartPoint, &STappingInfo.SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
		//3.赋值G33旋转轴
		STappingInfo.G33RotateAxis = (pSAssignConfig->cSSecondpindleStatus != 0) ? AXIS_A : AXIS_C;
		//4. 赋值G33攻牙模式 : A轴旋转：柔性攻牙    C轴旋转：刚性攻牙
		STappingInfo.G33TappingMode = (pSAssignConfig->cSSecondpindleStatus != 0) ? 1 : 0;
		//5.赋值导程
		STappingInfo.dF = pSG33Para->dF;
		//6.赋值头数
		STappingInfo.usL = pSG33Para->usL;
		//7.赋值实体类型
		STappingInfo.EntityType = TAPPING;
		//8.将STapping写入缓冲区
		writeTrappingIntoGCodeBuf(&STappingInfo, pGCodeBufUnit);
		return 2;
	}

	return 2;
}

