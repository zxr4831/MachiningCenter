#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"
#include <stdint.h>
#define FLEXIBLE_TAPPING   1

static SG88Para* pSG88ParaCheck[CHANNEL_NUM_MAX];
static SG88Para* pSG88ParaAssign[CHANNEL_NUM_MAX];

//添加G88规则结构体
void addG88RegulationAPI()
{
	//指令：G88
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(88, sGa, 1, mallocG88ParaStruct, freeG88ParaStruct, getG88Para, CheckG88, AssignG88);
}


/*************************************************
  函数名称：int8 mallocG88ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G88参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG88ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG88ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG88ParaCheck[ucChannelNum] = (SG88Para*)mallocGCode(sizeof(SG88Para), ORAM_NUM444);//暂未对内存编号
		if (pSG88ParaCheck[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG88ParaCheck[ucChannelNum], 0, sizeof(SG88Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG88ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG88ParaAssign[ucChannelNum] = (SG88Para*)mallocGCode(sizeof(SG88Para), ORAM_NUM445);
		if (pSG88ParaAssign[ucChannelNum] == NULL)
		{
			return-1;
		}
		memset(pSG88ParaAssign[ucChannelNum], 0, sizeof(SG88Para));
		//运行G88前的模态置为-1
		pSG88ParaAssign[ucChannelNum]->cGaBeforeG88 = UNASSIGNED_VALUE;

		return 1;
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：释放G88参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG88ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG88ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG88ParaCheck[ucChannelNum]);
			pSG88ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG88ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG88ParaAssign[ucChannelNum]);
			pSG88ParaAssign[ucChannelNum] = NULL;
		}
	}
}

/*************************************************
  函数名称：int8 getG88Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能   获取G88参数（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG88Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
{
	if (pSScanContainer == NULL || pSGCodeCurrSysState == NULL)
	{
		return -1;
	}



	if (ucMode == GC_SYNTAX)
	{
		pSG88ParaCheck[ucChannelNum]->cG88Step = 0;


		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG88ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);

		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG88ParaCheck[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG88ParaCheck[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG88ParaCheck[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		//4.获取K（G88重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG88ParaCheck[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
		}
		else//无输入K时，默认1次
		{
			pSG88ParaCheck[ucChannelNum]->cK = 1;
		}

		//获取F（导程）
		if (pSScanContainer->bF == TRUE)
		{
			pSG88ParaCheck[ucChannelNum]->dF = fabs(pSScanContainer->dF);
		}
		else //无输入F时，默认1次
		{
			pSG88ParaCheck[ucChannelNum]->dF = 1;
		}

		//记录G88上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG88ParaCheck[ucChannelNum]->bIsFirstHole = TRUE;

		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG88ParaAssign[ucChannelNum]->cG88Step = 0;


		//1.获取程序段坐标信息（为加入总偏移值的孔底坐标）
		getCoordInfoFromScanInfo(&pSG88ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);

		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG88ParaAssign[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG88ParaAssign[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG88ParaAssign[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		//4.获取K（G88重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG88ParaAssign[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
		}
		else//无输入K时，默认1次
		{
			pSG88ParaAssign[ucChannelNum]->cK = 1;
		}

		//获取F（导程）
		if (pSScanContainer->bF == TRUE)
		{
			pSG88ParaAssign[ucChannelNum]->dF = fabs(pSScanContainer->dF);
		}
		else //无输入F时，默认1次
		{
			pSG88ParaAssign[ucChannelNum]->dF = 1;
		}

		//记录G88上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG88ParaAssign[ucChannelNum]->bIsFirstHole = TRUE;

		return 1;
	}
}
                                       


/*************************************************
  函数名称：int8 AssignG88(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值G88
  输入参数：struct _SAssignConfiguration* pSAssignConfig		赋值结构体指针
			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG88(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}
	SPointDouble* pSRPoint = NULL;
	SPointDouble* pSFinalPoint = NULL;
	SPointDouble* pSFristPoint = NULL;
	SPointDouble SCurPoint;
	SG88Para* pSG88Para = NULL;

	pSG88Para = pSG88ParaAssign[pSAssignConfig->ucChannelNum];
	pSRPoint = &pSG88Para->SRPoint;//R点
	pSFinalPoint = &pSG88Para->SFinalPoint;//孔底点
	pSFristPoint = &pSG88Para->SFirstPoint;//初始点
	SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//当前点
	bool bIsAbsMethod;

	switch (pSG88Para->cG88Step)
	{
		//0 赋值预处理
	case 0:
	{
		//0.1判断G88执行次数是否为0
		if (pSG88Para->cK == 0)
		{
			//不执行G83
			return 2;
		}

		//0.2判断G88程序段中是否出现坐标信息
		int8 cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
		//0.2.1程序段出现了坐标信息G83才赋值
		if (cRet == TRUE)
		{

			//1. 第一次进给标志位
			pSG88Para->bIsFirstFeed = TRUE;
			//2.进给轴（G88的进给轴为Z轴）
			pSG88Para->cFeedAxis = AXIS_X;
			//3.计算孔底坐标

			*pSFinalPoint = SCurPoint;
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			//计算孔底坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG88Para->SCoordInfo, &SCurPoint, pSFinalPoint, &pSAssignConfig->SGCodeCurrSysState);


			//4.进给方向  （-1：从X正向往X负向，1：从X负向往X正向）
			pSG88Para->cFeedDir = (pSFinalPoint->dArray[pSG88Para->cFeedAxis] - SCurPoint.dArray[pSG88Para->cFeedAxis] < 0) ? -1 : 1;

			//5.计算R点
			//第一个孔
			if (pSG88Para->bIsFirstHole == TRUE)
			{
				//R点在非进给方向的坐标
				*pSRPoint = *pSFinalPoint;
				//R点在进给方向的坐标
				pSRPoint->dArray[pSG88Para->cFeedAxis] = SCurPoint.dArray[pSG88Para->cFeedAxis] + pSG88Para->cFeedDir * pSG88Para->dR;

			}
			//非第一个孔
			else
			{
				uint8 temp = 0;
				for (; temp < AXIS_NUM_MAX; temp++)
				{
					//R点在非进给方向的坐标
					//R点在进给方向的坐标无需更新
					if (temp != pSG88Para->cFeedAxis)
					{
						pSRPoint->dArray[temp] = pSFinalPoint->dArray[temp];
					}
				}
			}


			//6.计算起始点
		   //起始点在非进给方向的坐标
			*pSFristPoint = *pSFinalPoint;
			//起始点在进给方向的坐标
			pSFristPoint->dArray[pSG88Para->cFeedAxis] = SCurPoint.dArray[pSG88Para->cFeedAxis];





			pSG88Para->cG88Step = 1;

			return 0;
		}
		else
		{
			//不执行G88
			return 2;
		}
	}

	//1.从当前点快速移动到初始点
	case 1:
	{
		//1.1.填写SLine
		SLine SLineInfo_0;
		memset(&SLineInfo_0, 0, sizeof(SLine));
		//1.2初始化SLine的起点终点（机械坐标）
		SLineInfo_0.SEndPoint = SLineInfo_0.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.3 终点为初始点
		SLineInfo_0.SEndPoint = *pSFristPoint;
		//1.4 填写实体类型 G0
		SLineInfo_0.EntityType = OTHERTYPE;//G0
		//1.5 将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_0, pGCodeBufUnit);

		pSG88Para->cG88Step = 2;
		return 1;
	}

	//2.如果是A轴作为旋转运动加工轴，先主轴刹车松开
	case 2:
	{
		//A轴已被开启
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG88Para->cG88Step = 3;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车松开，也无需返回，继续执行函数
		{
			pSG88Para->cG88Step = 5;
		}

	}
	//3.如果是A轴作为旋转加工轴，C轴分度 G0
	case 3:
	{
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			//3.1.填写SLine
			SLine SLineInfo_1;
			memset(&SLineInfo_1, 0, sizeof(SLine));
			//3.2初始化SLine的起点终点（机械坐标）
			SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
			//3.3 终点C轴分度
			SLineInfo_1.SEndPoint.dArray[AXIS_C] = pSFinalPoint->dArray[AXIS_C];
			//3.4 填写实体类型 G0
			SLineInfo_1.EntityType = OTHERTYPE;//G0
			//3.5 将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo_1, pGCodeBufUnit);
			pSG88Para->cG88Step = 4;
			return 1;
		}
		else//A轴没启动，无需执行C分度，也无需返回，继续执行函数
		{
			pSG88Para->cG88Step = 5;
		}

	}
	//4.如果是A轴作为旋转加工轴，C轴度后主轴刹车锁紧
	case 4:
	{
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG88Para->cG88Step = 5;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车锁紧，也无需返回，继续执行函数
		{
			pSG88Para->cG88Step = 5;
		}

	}

	//5.运动轴快速移动到R点
	case 5:
	{
		//5.1.创建SLine
		SLine SLineInfo_2;
		memset(&SLineInfo_2, 0, sizeof(SLine));
		//5.2初始化SLine的起点终点（机械坐标）
		SLineInfo_2.SEndPoint = SLineInfo_2.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//5.3 终点为R点(c轴坐标不为R点的C，因为如果运动轴不为A轴时，把终点的C轴坐标赋值为R点的C坐标后，可能会导致执行C轴分度)
		SLineInfo_2.SEndPoint.dArray[AXIS_Z] = pSRPoint->dArray[AXIS_Z];
		SLineInfo_2.SEndPoint.dArray[AXIS_Y] = pSRPoint->dArray[AXIS_Y];
		SLineInfo_2.SEndPoint.dArray[AXIS_X] = pSRPoint->dArray[AXIS_X];
		//5.4 填写实体类型
		SLineInfo_2.EntityType = OTHERTYPE;
		//5.5 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_2, pGCodeBufUnit);
		pSG88Para->cG88Step = 6;
		return 1;
	}

	//6.攻牙到孔底  (G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹)
	case 6:
	{
		//6.1.创建SThread
		STapping STappingInfo_3;
		memset(&STappingInfo_3, 0, sizeof(STapping));
		//6.2初始化SLine的起点终点（机械坐标）
		STappingInfo_3.SEndPoint = STappingInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
	   //6.3赋值终点
		STappingInfo_3.SEndPoint.dArray[pSG88Para->cFeedAxis] = pSG88Para->SFinalPoint.dArray[pSG88Para->cFeedAxis];
		//6.4 赋值G33旋转轴
		STappingInfo_3.G33RotateAxis = (pSAssignConfig->cSSecondpindleStatus != 0) ? AXIS_A : AXIS_C;
		//6.5 赋值G33攻牙模式 : A轴旋转：柔性攻牙    C轴旋转：刚性攻牙
		STappingInfo_3.G33TappingMode = (pSAssignConfig->cSSecondpindleStatus != 0) ? FLEXIBLE_TAPPING : 0;
		//6.6 赋值G33头数
		STappingInfo_3.usL = 1;
		//6.7 赋值导程
		STappingInfo_3.dF = pSG88Para->dF;
		//6.8 赋值实体类型
		STappingInfo_3.EntityType = TAPPING;

		//6.8 将STapping写入缓冲区
		writeTrappingIntoGCodeBuf(&STappingInfo_3, pGCodeBufUnit);
		pSG88Para->cG88Step = 7;
		return 1;
	}

	//从R点快速移到初始点 (G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹，即孔底-R点)
	case 7:
	{
		//7.1.创建SLine
		SLine SLineInfo_3;
		memset(&SLineInfo_3, 0, sizeof(SLine));
		//7.2初始化SLine的起点终点（机械坐标）
		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//7.3赋值终点
		SLineInfo_3.SEndPoint.dArray[pSG88Para->cFeedAxis] = pSG88Para->SFirstPoint.dArray[pSG88Para->cFeedAxis];
		//7.4 填写实体类型
		SLineInfo_3.EntityType = OTHERTYPE;
		//7.5 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);
		pSG88Para->cG88Step = 8;

		//9.5 判断钻孔执行次数,并赋值G84的步骤
		pSG88Para->cK -= 1;
		if (pSG88Para->cK == 0)
		{
			pSG88Para->cG88Step = 8;
		}
		else
		{
			pSG88Para->cG88Step = 0;
			pSG88Para->bIsFirstFeed = TRUE;
			pSG88Para->bIsFirstHole = FALSE;
		}
		return 1;
	}

	//主轴刹车松开
	case 8:
	{
		//如果是A轴作为旋转运动加工轴,最后要把主轴刹车松开
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
		}
		pSG88Para->bIsFirstFeed = TRUE;
		pSG88Para->cG88Step = 0;
		return 2;
	}
	default:
		break;
	}
}


int8 CheckG88(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		if (pSSyntaxConfig->SScanContainer.bRIJK[3] == TRUE)
		{
			if (checkParaRange('K', &pSSyntaxConfig->SScanContainer, 1, 100) == -1)
			{
				return -1;
			}
			if (checkParaInteger('K', &pSSyntaxConfig->SScanContainer) == -1)
			{
				return -1;
			}
		}
		if (pSSyntaxConfig->SScanContainer.bP == TRUE)
		{
			if (checkParaRange('P', &pSSyntaxConfig->SScanContainer, 0, (double)UINT32_MAX / 1000.0) == -1)
			{
				return -1;
			}
		}
		return 2;
	}
	return 2;
}