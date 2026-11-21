#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"
#include <stdint.h>
#define HIGHSPEEDDRILLING_OF_DEEPHOLES 1	//高速钻深孔	G83.1											运行到加工点，退刀至R点，再进刀刀加工点，以此循环，最后运动到R点，分度在R点
#define COUNTERSUNKHOLE_PECKING_DRILL  2	//沉孔啄钻		G83.2	公司自己的逻辑							运行到加工点，退刀至NC089,再进刀刀加工点，以此循环，最后运动到起始点，分度在起始点
#define GENERALDEEPHOLE_PECKING_DRILL  3   //一般深孔啄钻	G83.3	默认(公司加工师傅要求) 跟新代系统一样，	进到到加工点，退刀到R点，再进刀刀加工点，以此循环，最后运动到起始点，分度在起始点


static SG83_2Para* pSG83_2ParaCheck[CHANNEL_NUM_MAX];
static SG83_2Para* pSG83_2ParaAssign[CHANNEL_NUM_MAX];

//添加G83_2规则结构体
void addG83_2RegulationAPI()
{
	//指令：G83_2
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(83.2, sGa, 1, mallocG83_2ParaStruct, freeG83_2ParaStruct, getG83_2Para, CheckG83_2, AssignG83_2);
}


/*************************************************
  函数名称：int8 mallocG83_2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G83_2参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG83_2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG83_2ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG83_2ParaCheck[ucChannelNum] = (SG83_2Para*)mallocGCode(sizeof(SG83_2Para), ORAM_NUM434);//暂未对内存编号
		if (pSG83_2ParaCheck[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG83_2ParaCheck[ucChannelNum], 0, sizeof(SG83_2Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG83_2ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG83_2ParaAssign[ucChannelNum] = (SG83_2Para*)mallocGCode(sizeof(SG83_2Para), ORAM_NUM435);
		if (pSG83_2ParaAssign[ucChannelNum] == NULL)
		{
			return-1;
		}
		memset(pSG83_2ParaAssign[ucChannelNum], 0, sizeof(SG83_2Para));
		//运行G83_2前的模态置为-1
		pSG83_2ParaAssign[ucChannelNum]->cGaBeforeG83_2 = UNASSIGNED_VALUE;

		return 1;
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：释放G83_2参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG83_2ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG83_2ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG83_2ParaCheck[ucChannelNum]);
			pSG83_2ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG83_2ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG83_2ParaAssign[ucChannelNum]);
			pSG83_2ParaAssign[ucChannelNum] = NULL;
		}
	}
}

/*************************************************
  函数名称：int8 getG83_2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能   获取G83_2参数（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG83_2Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
{
	if (pSScanContainer == NULL || pSGCodeCurrSysState == NULL)
	{
		return -1;
	}



	if (ucMode == GC_SYNTAX)
	{
		pSG83_2ParaCheck[ucChannelNum]->cG83_2Step = 0;


		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG83_2ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);


		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG83_2ParaCheck[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG83_2ParaCheck[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG83_2ParaCheck[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		//4.获取K（G83_2重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG83_2ParaCheck[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
		}
		else//无输入K时，默认1次
		{
			pSG83_2ParaCheck[ucChannelNum]->cK = 1;
		}

		//记录G83_2上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG83_2ParaCheck[ucChannelNum]->bIsFirstHole = TRUE;
		return 1;

	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG83_2ParaAssign[ucChannelNum]->cG83_2Step = 0;



		//1.获取程序段坐标信息（为加入总偏移值的孔底坐标）
		getCoordInfoFromScanInfo(&pSG83_2ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);

		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG83_2ParaAssign[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG83_2ParaAssign[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG83_2ParaAssign[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		//4.获取K（G83_2重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG83_2ParaAssign[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
		}
		else//无输入K时，默认1次
		{
			pSG83_2ParaAssign[ucChannelNum]->cK = 1;
		}

		//记录G83_2上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG83_2ParaAssign[ucChannelNum]->bIsFirstHole = TRUE;
		return 1;
	}
}

int8 CheckG83_2(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
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




/*************************************************
  函数名称：int8 AssignG83_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值G83_2
  输入参数：struct _SAssignConfiguration* pSAssignConfig		赋值结构体指针
			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG83_2(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}
	SPointDouble* pSRPoint = NULL;
	SPointDouble* pSFinalPoint = NULL;
	SPointDouble* pSFristPoint = NULL;
	SPointDouble SCurPoint;
	SG83_2Para* pSG83_2Para = NULL;

	pSG83_2Para = pSG83_2ParaAssign[pSAssignConfig->ucChannelNum];
	pSRPoint = &pSG83_2Para->SRPoint;//R点
	pSFinalPoint = &pSG83_2Para->SFinalPoint;//孔底点
	pSFristPoint = &pSG83_2Para->SFirstPoint;//初始点
	SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//当前点
	bool bIsAbsMethod;

	switch (pSG83_2Para->cG83_2Step)
	{
		//0 赋值预处理
	case 0:
	{
		//0.1判断G83_2执行次数是否为0
		if (pSG83_2Para->cK == 0)
		{
			//不执行G83_2
			return 2;
		}

		//0.2判断G83_2程序段中是否出现坐标信息
		int8 cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
		//0.2.1程序段出现了坐标信息G83_2才赋值
		if (cRet == TRUE)
		{

			//1. 第一次进给标志位
			pSG83_2Para->bIsFirstFeed = TRUE;
			//2.进给轴（G83_2的进给轴为Z轴）
			pSG83_2Para->cFeedAxis = AXIS_Z;

			//3.计算孔底坐标

			*pSFinalPoint = SCurPoint;
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			//计算孔底坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG83_2Para->SCoordInfo, &SCurPoint, pSFinalPoint, &pSAssignConfig->SGCodeCurrSysState);


			//4.进给方向  （-1：从Z正向往Z负向，1：从Z负向往Z正向）
			pSG83_2Para->cFeedDir = (pSFinalPoint->dArray[pSG83_2Para->cFeedAxis] - SCurPoint.dArray[pSG83_2Para->cFeedAxis] < 0) ? -1 : 1;

			//5.计算R点
			//第一个孔
			if (pSG83_2Para->bIsFirstHole == TRUE)
			{
				//R点在非进给方向的坐标
				*pSRPoint = *pSFinalPoint;
				//R点在进给方向的坐标
				pSRPoint->dArray[pSG83_2Para->cFeedAxis] = SCurPoint.dArray[pSG83_2Para->cFeedAxis] + pSG83_2Para->cFeedDir * pSG83_2Para->dR;
		
			}
			//非第一个孔
			else
			{
				uint8 temp = 0;
				for (; temp < AXIS_NUM_MAX; temp++)
				{
					//R点在非进给方向的坐标
					//R点在进给方向的坐标无需更新
					if (temp != pSG83_2Para->cFeedAxis)
					{
						pSRPoint->dArray[temp] = pSFinalPoint->dArray[temp];
					}
				}
			}

			//6.计算起始点
		   //起始点在非进给方向的坐标
			*pSFristPoint = *pSRPoint;
			//起始点在进给方向的坐标
			pSFristPoint->dArray[pSG83_2Para->cFeedAxis] = SCurPoint.dArray[pSG83_2Para->cFeedAxis];



			pSG83_2Para->cG83_2Step = 1;

			return 0;
		}
		else
		{
			//不执行G83_2
			return 2;
		}
	}

	//1.从当前点快速移动到初始点
	case 1:
	{
		//2.1.填写SLine
		SLine SLineInfo_0;
		memset(&SLineInfo_0, 0, sizeof(SLine));
		//2.2初始化SLine的起点终点（机械坐标）
		SLineInfo_0.SEndPoint = SLineInfo_0.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//2.3 终点为初始点
		SLineInfo_0.SEndPoint = *pSFristPoint;
		//2.4 填写实体类型 G0
		SLineInfo_0.EntityType = OTHERTYPE;//G0
		//2.5 将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_0, pGCodeBufUnit);

		pSG83_2Para->cG83_2Step = 2;
		return 1;
	}



	//2.如果是A轴作为旋转运动加工轴，先主轴刹车松开
	case 2:
	{
		//A轴已被开启
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG83_2Para->cG83_2Step =3;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车松开，也无需返回，继续执行函数
		{
			pSG83_2Para->cG83_2Step = 5;
		}

	}
	//3.如果是A轴作为旋转加工轴，C轴分度 G0
	case 3:
	{
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			//2.1.填写SLine
			SLine SLineInfo_1;
			memset(&SLineInfo_1, 0, sizeof(SLine));
			//2.2初始化SLine的起点终点（机械坐标）
			SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
			//2.3 终点C轴分度
			SLineInfo_1.SEndPoint.dArray[AXIS_C] = pSFinalPoint->dArray[AXIS_C];
			//2.4 填写实体类型 G0
			SLineInfo_1.EntityType = OTHERTYPE;//G0
			//2.5 将SLine写入缓冲区
			writeLineIntoGCodeBuf(&SLineInfo_1, pGCodeBufUnit);
			pSG83_2Para->cG83_2Step = 4;
			return 1;
		}
		else//A轴没启动，无需执行C分度，也无需返回，继续执行函数
		{
			pSG83_2Para->cG83_2Step = 5;
		}

	}
	//4.如果是A轴作为旋转加工轴，C轴度后主轴刹车锁紧
	case 4:
	{
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG83_2Para->cG83_2Step = 5;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车锁紧，也无需返回，继续执行函数
		{
			pSG83_2Para->cG83_2Step = 5;
		}

	}
	//5.运动轴快速移动到R点
	case 5:
	{
		//4.1.创建SLine
		SLine SLineInfo_2;
		memset(&SLineInfo_2, 0, sizeof(SLine));
		//4.2初始化SLine的起点终点（机械坐标）
		SLineInfo_2.SEndPoint = SLineInfo_2.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//4.3 终点为R点(c轴坐标不为R点的C，因为如果运动轴不为A轴时，把终点的C轴坐标赋值为R点的C坐标后，可能会导致执行C轴分度)
		SLineInfo_2.SEndPoint.dArray[AXIS_Z] = pSRPoint->dArray[AXIS_Z];
		SLineInfo_2.SEndPoint.dArray[AXIS_Y] = pSRPoint->dArray[AXIS_Y];
		SLineInfo_2.SEndPoint.dArray[AXIS_X] = pSRPoint->dArray[AXIS_X];
		//4.4 填写实体类型
		SLineInfo_2.EntityType = OTHERTYPE;
		//4.5 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_2, pGCodeBufUnit);
		pSG83_2Para->cG83_2Step = 6;
		return 1;
	}
	//6.钻孔进给深度Q
	case 6:
	{
		//5.1.创建SLine
		SLine SLineInfo_3;
		double dTemp_Znextpoint;//进给终点坐标
		memset(&SLineInfo_3, 0, sizeof(SLine));
		//5.2初始化SLine的起点终点（机械坐标）
		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点


		//5.3 计算进给终点坐标
		//5.3.1 Q值为0，则直接到孔底
		if (isZero(pSG83_2Para->dQ))
		{
			dTemp_Znextpoint = pSFinalPoint->dArray[pSG83_2Para->cFeedAxis];
		}
		//5.3.2 Q值不为0，则需要分第一次进给与非第一次进给
		else
		{
			//第一次进给，进给量为Q，进给终点坐标为当前点坐标+方向系数*Q
			if (pSG83_2Para->bIsFirstFeed == TRUE)
			{
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG83_2Para->cFeedAxis] + pSG83_2Para->cFeedDir * pSG83_2Para->dQ;
			}
			//非第一次进给，进给量为Q+d ，进给终点坐标为当前点坐标+方向系数*（Q+d）   d：系统参数：G83_2toG83_2退刀参数_退刀长度 um
			else
			{
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG83_2Para->cFeedAxis] + pSG83_2Para->cFeedDir * (pSG83_2Para->dQ + g_SSystemPara.sG83toG89Para_RetractionLength * TEMP_LEAST_INPUT_INCREMENT);
			}
		}

		//5.4 判断进给终点是否超过孔底坐标，并赋值步骤
		//本次进给抵达孔底或者超过孔底
		if(isEqualOrGreaterThanZero((dTemp_Znextpoint - pSFinalPoint->dArray[pSG83_2Para->cFeedAxis]) * pSG83_2Para->cFeedDir))
		{
			//约束本次进给终点坐标为孔底坐标
			dTemp_Znextpoint = pSFinalPoint->dArray[pSG83_2Para->cFeedAxis];
			// 赋值步骤
			pSG83_2Para->cG83_2Step = 8;

		}
		//本次进给没有抵达孔底或者没有超过孔底
		else
		{

			pSG83_2Para->cG83_2Step = 7;
		}
		//5.5如果为第一次进给，第一次进给计算完毕后将第一次标志位置为FALSE
		if (pSG83_2Para->bIsFirstFeed == TRUE)
		{
			pSG83_2Para->bIsFirstFeed = FALSE;
		}

		//5.6 将轨迹写入赋值缓冲区
		//填写实体类型
		SLineInfo_3.EntityType = L1NE;
		//赋值Sline终点坐标
		SLineInfo_3.SEndPoint.dArray[pSG83_2Para->cFeedAxis] = dTemp_Znextpoint;
		// 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);
		return 1;
	}
	//7.快速退刀并计算下一次切削起始点  G83.2的快速退刀终点为下一次切削起始点
	case 7:
	{
		//6.1.创建SLine
		SLine SLineInfo_4;
		memset(&SLineInfo_4, 0, sizeof(SLine));
		//6.2计算下一次进给的切削起始点
		//非进给方向坐标
		pSG83_2Para->SCuttingStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		//进给方向的坐标=当前点-（方向系数*d）
		pSG83_2Para->SCuttingStartPoint.dArray[pSG83_2Para->cFeedAxis] -= pSG83_2Para->cFeedDir * g_SSystemPara.sG83toG89Para_RetractionLength * TEMP_LEAST_INPUT_INCREMENT;
		//6.3初始化SLine的起点终点（机械坐标）
		SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//6.4计算SLine终点:下一次进给的切削起始点
		SLineInfo_4.SEndPoint.dArray[pSG83_2Para->cFeedAxis] = pSG83_2Para->SCuttingStartPoint.dArray[pSG83_2Para->cFeedAxis];
		
		//6.5 将轨迹写入缓冲区中
		//填写实体类型 G0
		SLineInfo_4.EntityType = OTHERTYPE;
		// 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_4, pGCodeBufUnit);
		//6.6赋值G83_2步骤
		pSG83_2Para->cG83_2Step = 6;
		return 1;
	}
	//8.孔底暂停
	case 8:
	{
		//暂停时间不为0
		if (pSG83_2Para->unDelaytime > 0)
		{
			//赋值延时时间
			pGCodeBufUnit->unDelayTime = pSG83_2Para->unDelaytime;
			pSG83_2Para->cG83_2Step =9;
			return 1;
		}
		else //没有暂停时间，无需退出函数，直接下一步
		{
			pSG83_2Para->cG83_2Step = 9;
		}
	}
	//9.快速退刀至初始点,并判断钻孔执行次数
	case 9:
	{
		//8.1 创建SLine
		SLine SlineInfo_6;
		memset(&SlineInfo_6, 0, sizeof(SLine));
		//8.2初始化SLine的起点终点（机械坐标）
		SlineInfo_6.SEndPoint = SlineInfo_6.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//8.3计算Sline终点  :初始点
		SlineInfo_6.SEndPoint.dArray[pSG83_2Para->cFeedAxis] = pSFristPoint->dArray[pSG83_2Para->cFeedAxis];
		//8.4 将轨迹写入缓冲区中
		//填写实体类型 G0
		SlineInfo_6.EntityType = OTHERTYPE;
		//将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SlineInfo_6, pGCodeBufUnit);

		//8.5 判断钻孔执行次数,并赋值G83_2的步骤
		pSG83_2Para->cK -= 1;
		if (pSG83_2Para->cK == 0)
		{
			pSG83_2Para->cG83_2Step = 10;
		}
		else
		{
			pSG83_2Para->cG83_2Step = 0;
			pSG83_2Para->bIsFirstFeed = TRUE;
			pSG83_2Para->bIsFirstHole = FALSE;
		}
		return 1;
	}
	//9.主轴刹车松开
	case 10:
	{
		//如果是A轴作为旋转运动加工轴,最后要把主轴刹车松开
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
		}
		pSG83_2Para->bIsFirstFeed = TRUE;
		pSG83_2Para->cG83_2Step = 0;
		return 2;
	}
	default:
		break;
	}
}


