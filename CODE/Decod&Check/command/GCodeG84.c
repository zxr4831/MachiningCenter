#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"
#include "GeneralFunctions.h"
#include <math.h>

//啄式加工
#define FLEXIBLE_TAPPING   1

static SG84Para* pSG84ParaCheck[CHANNEL_NUM_MAX];
static SG84Para* pSG84ParaAssign[CHANNEL_NUM_MAX];

//添加G84规则结构体
void addG84RegulationAPI()
{
	//指令：G84
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(84, sGa, 1, mallocG84ParaStruct, freeG84ParaStruct, getG84Para, CheckG84, AssignG84);

}


/*************************************************
  函数名称：int8 mallocG84ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G84参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG84ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG84ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG84ParaCheck[ucChannelNum] = (SG84Para*)mallocGCode(sizeof(SG84Para), ORAM_NUM436);//暂未对内存编号
		if (pSG84ParaCheck[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG84ParaCheck[ucChannelNum], 0, sizeof(SG84Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG84ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG84ParaAssign[ucChannelNum] = (SG84Para*)mallocGCode(sizeof(SG84Para), ORAM_NUM437);
		if (pSG84ParaAssign[ucChannelNum] == NULL)
		{
			return-1;
		}
		memset(pSG84ParaAssign[ucChannelNum], 0, sizeof(SG84Para));
		//运行G84前的模态置为-1
		pSG84ParaAssign[ucChannelNum]->cGaBeforeG84 = UNASSIGNED_VALUE;

		return 1;
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：释放G84参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG84ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG84ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG84ParaCheck[ucChannelNum]);
			pSG84ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG84ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG84ParaAssign[ucChannelNum]);
			pSG84ParaAssign[ucChannelNum] = NULL;
		}
	}
}
/*************************************************
  函数名称：checkG84I
  函数功能：检查采用 I 导程时的除法和同步倍率数值有效性
  输入参数：pScan 非空；I 为有符号的每英寸牙数
  输出参数：失败写入报警130，不修改扫描数据或循环参数
  返 回 值：1 未采用I或数值有效；-1 数值无效
  备    注：保留F优先；I正负号决定旋向，不随G20/G21缩放
**************************************************/
static int8 checkG84I(const SScan* pScan)
{
    if (pScan->bF == TRUE || pScan->bRIJK[1] != TRUE)
    {
        return 1;
    }

    double dI = pScan->dRIJK[1];
    if (!isfinite(dI) || dI == 0.0)
    {
        parseGCodeErrorB(pScan->unLineNum, GCODE_ERROR_NUM130);
        return -1;
    }

    double dLead = 25.4 / dI;
    if (!isfinite(dLead) || dLead == 0.0 || !isfinite(360.0 / dLead))
    {
        parseGCodeErrorB(pScan->unLineNum, GCODE_ERROR_NUM130);
        return -1;
    }

    return 1;
}

//底层走G1的G84的指令循环如下：
/*************************************************
  函数名称：int8 getG84Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能   获取G84参数（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG84Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
{
	if (pSScanContainer == NULL || pSGCodeCurrSysState == NULL)
	{
		return -1;
	}
	//校验扫描得到的 G84 重复次数，防止非法转换
	 if (pSScanContainer->bRIJK[3] == TRUE)
    {
        double dK = pSScanContainer->dRIJK[3];

        if (!isfinite(dK) ||
            dK < 0.0 ||
            dK > 127.0 ||
            dK != floor(dK))
        {
            parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
            return -1;
        }
    }

	/* 先校验新采用的I，避免失败时污染已保存的循环参数。 */
	if (checkG84I(pSScanContainer) < 0)
	{
		return -1;
	}

	if (ucMode == GC_SYNTAX)
	{
		pSG84ParaCheck[ucChannelNum]->cG84Step = 0;


		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG84ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);
		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG84ParaCheck[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG84ParaCheck[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG84ParaCheck[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		else
		{
			pSG84ParaCheck[ucChannelNum]->unDelaytime = g_SSystemPara.unG84DefaultDelayTime;  //G84孔底暂停默认时间，单位：ms
		}
		//4.获取K（G84重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG84ParaCheck[ucChannelNum]->cK = (int8)pSScanContainer->dRIJK[3];
		}
		else//无输入K时，默认1次
		{
			pSG84ParaCheck[ucChannelNum]->cK = 1;
		}

		if (pSScanContainer->bD == TRUE)
		{
			pSG84ParaCheck[ucChannelNum]->dD = (double)fabs(pSScanContainer->dD);  //zxd:20251024
		}
		else
		{
			pSG84ParaCheck[ucChannelNum]->dD = 0;
		}

	
			//获取F（导程）
		if (pSScanContainer->bF == TRUE)
		{
			pSG84ParaCheck[ucChannelNum]->dF = pSScanContainer->dF;
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			/* I为牙/英寸，保存有符号毫米导程，不受坐标单位影响。 */
			pSG84ParaCheck[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];
		}
		else //无输入F时，默认1次
		{
			pSG84ParaCheck[ucChannelNum]->dF = 1;
		}
		//记录G84上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG84ParaCheck[ucChannelNum]->bIsFirstHole = TRUE;

		return 1;


	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG84ParaAssign[ucChannelNum]->cG84Step = 0;

		//1.获取程序段坐标信息（为加入总偏移值的孔底坐标）
		getCoordInfoFromScanInfo(&pSG84ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);

		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG84ParaAssign[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG84ParaAssign[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG84ParaAssign[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		else
		{
			pSG84ParaAssign[ucChannelNum]->unDelaytime = g_SSystemPara.unG84DefaultDelayTime; //G84孔底暂停默认时间，单位：ms
		}
		//4.获取K（G84重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG84ParaAssign[ucChannelNum]->cK = (int8)pSScanContainer->dRIJK[3];
		}
		else//无输入K时，默认1次
		{
			pSG84ParaAssign[ucChannelNum]->cK = 1;
		}
		if (pSScanContainer->bD == TRUE)
		{
			pSG84ParaAssign[ucChannelNum]->dD = (double)fabs(pSScanContainer->dD);  //zxd:20251024
		}
		else
		{
			pSG84ParaAssign[ucChannelNum]->dD = 0;
		}

			//获取F（导程）
		if (pSScanContainer->bF == TRUE)
		{
			pSG84ParaAssign[ucChannelNum]->dF = pSScanContainer->dF;
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			/* I为牙/英寸，保存有符号毫米导程，不受坐标单位影响。 */
			pSG84ParaAssign[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];
		}
		else //无输入F且没输入I值时，默认1次
		{
			pSG84ParaAssign[ucChannelNum]->dF = 1;
		}
		//记录G84上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG84ParaAssign[ucChannelNum]->bIsFirstHole = TRUE;

		return 1;
	}
}

/*************************************************
  函数名称：int8 AssignG84(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值G84
  输入参数：struct _SAssignConfiguration* pSAssignConfig		赋值结构体指针
			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG84(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}
	/* 上层即使忽略取参失败，也不能用旧导程执行非法I。 */
	if (checkG84I(&pSAssignConfig->SScanContainer) < 0)
	{
		return -1;
	}
	SPointDouble* pSRPoint = NULL;
	SPointDouble* pSFinalPoint = NULL;
	SPointDouble* pSFristPoint = NULL;
	SPointDouble SCurPoint;
	SG84Para* pSG84Para = NULL;

	pSG84Para = pSG84ParaAssign[pSAssignConfig->ucChannelNum];
	pSRPoint = &pSG84Para->SRPoint;//R点
	pSFinalPoint = &pSG84Para->SFinalPoint;//孔底点
	pSFristPoint = &pSG84Para->SFirstPoint;//初始点
	SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//当前点
	bool bIsAbsMethod;
	
	switch (pSG84Para->cG84Step)
	{
		//0 赋值预处理
	case 0:
	{
		//0.1判断G84执行次数是否为0
		if (pSG84Para->cK == 0)
		{
			//不执行G84
			return 2;
		}

		//0.2判断G84程序段中是否出现坐标信息
		int8 cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
		//0.2.1程序段出现了坐标信息G84才赋值
		if (cRet == TRUE)
		{

			//1. 第一次进给标志位
			pSG84Para->bIsFirstFeed = TRUE;
			//2.进给轴（G84的进给轴为Z轴）
			pSG84Para->cFeedAxis = AXIS_Z;
			//3.计算孔底坐标

			*pSFinalPoint = SCurPoint;
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			//计算孔底坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG84Para->SCoordInfo, &SCurPoint, pSFinalPoint, &pSAssignConfig->SGCodeCurrSysState);


			//4.进给方向  （-1：从Z正向往Z负向，1：从Z负向往Z正向）
			pSG84Para->cFeedDir = (pSFinalPoint->dArray[pSG84Para->cFeedAxis] - SCurPoint.dArray[pSG84Para->cFeedAxis] < 0) ? -1 : 1;

			//5.计算R点
			//第一个孔
			if (pSG84Para->bIsFirstHole == TRUE)
			{
				//R点在非进给方向的坐标
				*pSRPoint = *pSFinalPoint;
				//R点在进给方向的坐标
				pSRPoint->dArray[pSG84Para->cFeedAxis] = SCurPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dR;

			}
			//非第一个孔
			else
			{
				uint8 temp = 0;
				for (; temp < AXIS_NUM_MAX; temp++)
				{
					//R点在非进给方向的坐标
					//R点在进给方向的坐标无需更新
					if (temp != pSG84Para->cFeedAxis)
					{
						pSRPoint->dArray[temp] = pSFinalPoint->dArray[temp];
					}
				}
			}


			//6.计算起始点
		   //起始点在非进给方向的坐标
			*pSFristPoint = *pSFinalPoint;
			//起始点在进给方向的坐标
			pSFristPoint->dArray[pSG84Para->cFeedAxis] = SCurPoint.dArray[pSG84Para->cFeedAxis];

			pSG84Para->cG84Step = 1;

			return 0;
		}
		else
		{
			//不执行G84
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

		pSG84Para->cG84Step = 2;
		return 1;
	}


	//1.如果是A轴作为旋转运动加工轴，先主轴刹车松开
	case 2:
	{
		//A轴已被开启
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG84Para->cG84Step = 3;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车松开，也无需返回，继续执行函数
		{
			pSG84Para->cG84Step = 5;
		}

	}
	//2.如果是A轴作为旋转加工轴，C轴分度 G0
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
			pSG84Para->cG84Step = 4;
			return 1;
		}
		else//A轴没启动，无需执行C分度，也无需返回，继续执行函数
		{
			pSG84Para->cG84Step = 5;
		}

	}
	//3.如果是A轴作为旋转加工轴，C轴度后主轴刹车锁紧
	case 4:
	{
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG84Para->cG84Step = 5;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车锁紧，也无需返回，继续执行函数
		{
			pSG84Para->cG84Step = 5;
		}

	}
	//4.运动轴快速移动到R点
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
		pSG84Para->cG84Step = 6;
		return 1;
	}
	//5.钻孔进给深度Q
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
		if (isZero(pSG84Para->dQ))
		{
			dTemp_Znextpoint = pSFinalPoint->dArray[pSG84Para->cFeedAxis];
		}
		//5.3.2 Q值不为0，则需要分第一次进给与非第一次进给
		else
		{
			//第一次进给，进给量为Q，进给终点坐标为当前点坐标+方向系数*Q
			if (pSG84Para->bIsFirstFeed == TRUE)
			{
				pSG84Para->cFeedQTimes = 1;
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dQ;
			}
			else if(pSG84Para->dD == 0)
			{
				pSG84Para->cFeedQTimes++;
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dQ * pSG84Para->cFeedQTimes;
			}
			//非第一次进给，进给量为Q+d ，进给终点坐标为当前点坐标+方向系数*（Q+d） 
			else
			{
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * (pSG84Para->dQ + pSG84Para->dD);
			}
		}

		//5.4 判断进给终点是否超过孔底坐标，并赋值步骤
		//本次进给抵达孔底或者超过孔底
		if (isEqualOrGreaterThanZero((dTemp_Znextpoint - pSFinalPoint->dArray[pSG84Para->cFeedAxis]) * pSG84Para->cFeedDir))
		{
			//约束本次进给终点坐标为孔底坐标
			dTemp_Znextpoint = pSFinalPoint->dArray[pSG84Para->cFeedAxis];
			// 赋值步骤
			pSG84Para->cG84Step = 8;

		}
		//本次进给没有抵达孔底或者没有超过孔底
		else
		{

			pSG84Para->cG84Step = 11;
		}
		//5.5如果为第一次进给，第一次进给计算完毕后将第一次标志位置为FALSE
		if (pSG84Para->bIsFirstFeed == TRUE)
		{
			pSG84Para->bIsFirstFeed = FALSE;
		}

		//5.6 将轨迹写入赋值缓冲区
		//赋值Sline终点坐标
		SLineInfo_3.SEndPoint.dArray[pSG84Para->cFeedAxis] = dTemp_Znextpoint;

		//1.修改实体类型
		SLineInfo_3.EntityType = L1NE;
		//2.修正进给速度(进给速度 = 转速 *螺距、60/1000) 转化为ms
		pGCodeBufUnit->dFeedrate = fabs(pSG84Para->dF) * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
		//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
		pGCodeBufUnit->dG84Ratio = 360 / pSG84Para->dF * sgn(SLineInfo_3.SStartPoint.dArray[pSG84Para->cFeedAxis] - SLineInfo_3.SEndPoint.dArray[pSG84Para->cFeedAxis]);
		//4.设置C轴为有效轴
		setbit(pGCodeBufUnit->ucValStatus, AXIS_C);

		// 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);
		return 1;
	}
	//7.退刀并计算下一次切削起始点
	case 7:
	{
		//6.1.创建SLine
		SLine SLineInfo_4;
		memset(&SLineInfo_4, 0, sizeof(SLine));
		if (pSG84Para->dD == 0)//没有输入D参数 ，回退到R点
		{
			//6.2初始化SLine的起点终点（机械坐标）
			SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
			//6.3 计算SLine终点:R点
			SLineInfo_4.SEndPoint.dArray[pSG84Para->cFeedAxis] = pSRPoint->dArray[pSG84Para->cFeedAxis];
			 
		}
		else
		{
			SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
			//6.3 计算SLine终点:回退点D点
			SLineInfo_4.SEndPoint.dArray[pSG84Para->cFeedAxis] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] - pSG84Para->cFeedDir * pSG84Para->dD;
		}

		//6.4 计算下一次进给的切削起始点
		//非进给方向坐标
		pSG84Para->SCuttingStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		//进给方向的坐标=当前点-（方向系数*d）
		pSG84Para->SCuttingStartPoint.dArray[pSG84Para->cFeedAxis] -= pSG84Para->cFeedDir;
		//6.5 将轨迹写入缓冲区中

		//1.修改实体类型
		SLineInfo_4.EntityType = L1NE;
		//2.修正进给速度(进给速度 = 转速 *螺距)
		pGCodeBufUnit->dFeedrate = fabs(pSG84Para->dF) * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
		//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
		pGCodeBufUnit->dG84Ratio = 360 / pSG84Para->dF * sgn(SLineInfo_4.SStartPoint.dArray[pSG84Para->cFeedAxis] - SLineInfo_4.SEndPoint.dArray[pSG84Para->cFeedAxis]);
		//4.设置C轴为有效轴
		setbit(pGCodeBufUnit->ucValStatus, AXIS_C);
		
		// 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_4, pGCodeBufUnit);
		//6.6赋值G84步骤
		pSG84Para->cG84Step = 6;
		return 1;
	}

	//8.孔底暂停
	case 8:
	{
		//暂停时间不为0
		if (pSG84Para->unDelaytime > 0)
		{
			//赋值延时时间
			pGCodeBufUnit->unDelayTime = pSG84Para->unDelaytime;
            /* 复用同步比例标识 G84 停留；运行层据此保留轴和反馈相位，普通 G04 仍释放。 */
            pGCodeBufUnit->dG84Ratio = 360.0 / pSG84Para->dF;
			pSG84Para->cG84Step = 9;
			return 1;
		}
		else //没有暂停时间，无需退出函数，直接下一步
		{
			pSG84Para->cG84Step = 9;
		}
	}
	//9.快速退刀至初始点,并判断钻孔执行次数
	case 9:
	{
		//9.1 创建SLine
		SLine SlineInfo_6;
		memset(&SlineInfo_6, 0, sizeof(SLine));
		//9.2初始化SLine的起点终点（机械坐标）
		SlineInfo_6.SEndPoint = SlineInfo_6.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//9.3计算Sline终点  :初始点
		SlineInfo_6.SEndPoint.dArray[pSG84Para->cFeedAxis] = pSFristPoint->dArray[pSG84Para->cFeedAxis];
		//9.4 将轨迹写入缓冲区中
		//填写实体类型 G0
	
		//1.修改实体类型
		SlineInfo_6.EntityType = L1NE;
		//2.修正进给速度(进给速度 = 转速 *螺距)
		pGCodeBufUnit->dFeedrate = fabs(pSG84Para->dF) * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
		//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
		pGCodeBufUnit->dG84Ratio = 360 / pSG84Para->dF * sgn(SlineInfo_6.SStartPoint.dArray[pSG84Para->cFeedAxis] - SlineInfo_6.SEndPoint.dArray[pSG84Para->cFeedAxis]);
		//4.设置C轴为有效轴
		setbit(pGCodeBufUnit->ucValStatus, AXIS_C);

		writeLineIntoGCodeBuf(&SlineInfo_6, pGCodeBufUnit);

		//9.5 判断钻孔执行次数,并赋值G84的步骤
		pSG84Para->cK -= 1;
		if (pSG84Para->cK == 0)
		{
			pSG84Para->cG84Step = 10;
		}
		else
		{
			pSG84Para->cG84Step = 0;
			pSG84Para->bIsFirstFeed = TRUE;
			pSG84Para->bIsFirstHole = FALSE;
		}
		return 1;
	}
	//10.主轴刹车松开
	case 10:
	{
		//如果是A轴作为旋转运动加工轴,最后要把主轴刹车松开
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
		}
		pSG84Para->bIsFirstFeed = TRUE;
		pSG84Para->cG84Step = 0;
		return 2;
	}
	case 11:
	{
		if (pSG84Para->unDelaytime > 0)
		{
			pGCodeBufUnit->unDelayTime = pSG84Para->unDelaytime;
            /* 复用同步比例标识 G84 停留；运行层据此保留轴和反馈相位，普通 G04 仍释放。 */
            pGCodeBufUnit->dG84Ratio = 360.0 / pSG84Para->dF;
		}

		pSG84Para->cG84Step = 7;
		return 1;
	}
	default:
		break;
	}
}



int8 CheckG84(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{

	if (pSSyntaxConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	//检查K是否有效
	if (pSSyntaxConfig->SScanContainer.bRIJK[3] == TRUE)
    {
        double dK = pSSyntaxConfig->SScanContainer.dRIJK[3];

        if (!isfinite(dK) ||
            dK < 0.0 ||
            dK > 127.0 ||
            dK != floor(dK))
        {
            parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
            return -1;
        }
    }

	if (checkG84I(&pSSyntaxConfig->SScanContainer) < 0)
	{
		return -1;
	}

	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		//G84的M29检查
		if (isZero(pSSyntaxConfig->SGCodeCurrSysState.dTappingSpeed))
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
		}
		return 2;
	}
	return 2;
}


//底层走G33的G84的指令循环如下：
///*************************************************
//  函数名称：int8 getG84Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
//  函数功能   获取G84参数（总框架）
//  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
//			ucChannelNum	通道号
//  输出参数：
//  返 回 值：
//  备    注：
//**************************************************/
//int8 getG84Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
//{
//	if (pSScanContainer == NULL || pSGCodeCurrSysState == NULL)
//	{
//		return -1;
//	}
//
//
//
//	if (ucMode == GC_SYNTAX)
//	{
//		pSG84ParaCheck[ucChannelNum]->cG84Step = 0;
//
//
//		//2.获取坐标信息
//		getCoordInfoFromScanInfo(&pSG84ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);
//		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
//		if (pSScanContainer->bRIJK[0] == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
//		}
//		//2.获取Q  （每次进给深度(正负号无效) ）
//		if (pSScanContainer->bQ == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
//		}
//		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
//		if (pSScanContainer->bP == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
//		}
//		//4.获取K（G84重复执行的次数）
//		if (pSScanContainer->bRIJK[3] == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
//		}
//		else//无输入K时，默认1次
//		{
//			pSG84ParaCheck[ucChannelNum]->cK = 1;
//		}
//
//		//获取F（导程）
//		if (pSScanContainer->bF == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->dF = fabs(pSScanContainer->dF);
//		}
//		else //无输入F时，默认1次
//		{
//			pSG84ParaCheck[ucChannelNum]->dF = 1;
//		}
//		if (pSScanContainer->bD == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->dD = fabs(pSScanContainer->dD);
//		}
//		//记录G84上一行的模态
//		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);
//
//		pSG84ParaCheck[ucChannelNum]->bIsFirstHole = TRUE;
//
//		return 1;
//	}
//	else if (ucMode == GC_ASSIGN)
//	{
//		pSG84ParaAssign[ucChannelNum]->cG84Step = 0;
//
//
//		//1.获取程序段坐标信息（为加入总偏移值的孔底坐标）
//		getCoordInfoFromScanInfo(&pSG84ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);
//
//		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
//		if (pSScanContainer->bRIJK[0] == TRUE)
//		{
//			pSG84ParaAssign[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0]);
//		}
//		//2.获取Q  （每次进给深度(正负号无效) ）
//		if (pSScanContainer->bQ == TRUE)
//		{
//			pSG84ParaAssign[ucChannelNum]->dQ = fabs(pSScanContainer->dQ);
//		}
//		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
//		if (pSScanContainer->bP == TRUE)
//		{
//			pSG84ParaAssign[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
//		}
//		//4.获取K（G84重复执行的次数）
//		if (pSScanContainer->bRIJK[3] == TRUE)
//		{
//			pSG84ParaAssign[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
//		}
//		else//无输入K时，默认1次
//		{
//			pSG84ParaAssign[ucChannelNum]->cK = 1;
//		}
//
//		//获取F（导程）
//		if (pSScanContainer->bF==TRUE)
//		{
//			pSG84ParaAssign[ucChannelNum]->dF = fabs(pSScanContainer->dF);
//		}
//		else //无输入F时，默认1次
//		{
//			pSG84ParaAssign[ucChannelNum]->dF = 1;
//		}
//		//获取D回退高度
//		if (pSScanContainer->bD == TRUE)
//		{
//			pSG84ParaCheck[ucChannelNum]->dD = fabs(pSScanContainer->dD);
//		}
//
//		//记录G84上一行的模态
//		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);
//
//		pSG84ParaAssign[ucChannelNum]->bIsFirstHole = TRUE;
//
//		return 1;
//	}
//}
//
//
//
///*************************************************
//  函数名称：int8 AssignG84(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
//  函数功能：赋值G84
//  输入参数：struct _SAssignConfiguration* pSAssignConfig		赋值结构体指针
//			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
//  输出参数：
//  返 回 值：0	赋值预处理（未写G代码缓冲区）
//			1	写了，没写完
//			2	赋值完成
//  备    注：
//**************************************************/
//int8 AssignG84(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
//{
//	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
//	{
//		return -1;
//	}
//	SPointDouble* pSRPoint = NULL;
//	SPointDouble* pSFinalPoint = NULL;
//	SPointDouble* pSFristPoint = NULL;
//	SPointDouble SCurPoint;
//	SG84Para* pSG84Para = NULL;
//
//	pSG84Para = pSG84ParaAssign[pSAssignConfig->ucChannelNum];
//	pSRPoint = &pSG84Para->SRPoint;//R点
//	pSFinalPoint = &pSG84Para->SFinalPoint;//孔底点
//	pSFristPoint = &pSG84Para->SFirstPoint;//初始点
//	SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//当前点
//	bool bIsAbsMethod;
//
//	switch (pSG84Para->cG84Step)
//	{
//		//0 赋值预处理
//	case 0:
//	{
//		//0.1判断G84执行次数是否为0
//		if (pSG84Para->cK == 0)
//		{
//			//不执行G83
//			return 2;
//		}
//
//		//0.2判断G84程序段中是否出现坐标信息
//		int8 cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
//		//0.2.1程序段出现了坐标信息G83才赋值
//		if (cRet == TRUE)
//		{
//
//			//1. 第一次进给标志位
//			pSG84Para->bIsFirstFeed = TRUE;
//			//2.进给轴（G84的进给轴为Z轴）
//			pSG84Para->cFeedAxis = AXIS_Z;
//			//3.计算孔底坐标
//
//			*pSFinalPoint = SCurPoint;
//			//获取当前的坐标编程方式
//			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
//			//计算孔底坐标
//			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG84Para->SCoordInfo, &SCurPoint, pSFinalPoint, &pSAssignConfig->SGCodeCurrSysState);
//
//
//			//4.进给方向  （-1：从Z正向往Z负向，1：从Z负向往Z正向）
//			pSG84Para->cFeedDir = (pSFinalPoint->dArray[pSG84Para->cFeedAxis] - SCurPoint.dArray[pSG84Para->cFeedAxis] < 0) ? -1 : 1;
//
//			//5.计算R点
//			//第一个孔
//			if (pSG84Para->bIsFirstHole == TRUE)
//			{
//				//R点在非进给方向的坐标
//				*pSRPoint = *pSFinalPoint;
//				//R点在进给方向的坐标
//				pSRPoint->dArray[pSG84Para->cFeedAxis] = SCurPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dR;
//
//			}
//			//非第一个孔
//			else
//			{
//				uint8 temp = 0;
//				for (; temp < AXIS_NUM_MAX; temp++)
//				{
//					//R点在非进给方向的坐标
//					//R点在进给方向的坐标无需更新
//					if (temp != pSG84Para->cFeedAxis)
//					{
//						pSRPoint->dArray[temp] = pSFinalPoint->dArray[temp];
//					}
//				}
//			}
//
//
//			//6.计算起始点
//		   //起始点在非进给方向的坐标
//			*pSFristPoint = *pSFinalPoint;
//			//起始点在进给方向的坐标
//			pSFristPoint->dArray[pSG84Para->cFeedAxis] = SCurPoint.dArray[pSG84Para->cFeedAxis];
//
//
//
//
//
//			pSG84Para->cG84Step = 1;
//
//			return 0;
//		}
//		else
//		{
//			//不执行G84
//			return 2;
//		}
//	}
//
//	//1.从当前点快速移动到初始点
//	case 1:
//	{
//		//1.1.填写SLine
//		SLine SLineInfo_0;
//		memset(&SLineInfo_0, 0, sizeof(SLine));
//		//1.2初始化SLine的起点终点（机械坐标）
//		SLineInfo_0.SEndPoint = SLineInfo_0.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//		//1.3 终点为初始点
//		SLineInfo_0.SEndPoint = *pSFristPoint;
//		//1.4 填写实体类型 G0
//		SLineInfo_0.EntityType = OTHERTYPE;//G0
//		//1.5 将SLine写入缓冲区
//		writeLineIntoGCodeBuf(&SLineInfo_0, pGCodeBufUnit);
//
//		pSG84Para->cG84Step = 2;
//		return 1;
//	}
//
//	//2.如果是A轴作为旋转运动加工轴，先主轴刹车松开
//	case 2:
//	{
//		//A轴已被开启
//		if (pSAssignConfig->cSSecondpindleStatus != 0)
//		{
//			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
//			pSG84Para->cG84Step = 3;
//			return 1;
//		}
//		else//A轴没启动，无需执行主轴刹车松开，也无需返回，继续执行函数
//		{
//			pSG84Para->cG84Step = 5;
//		}
//
//	}
//	//3.如果是A轴作为旋转加工轴，C轴分度 G0
//	case 3:
//	{
//		if (pSAssignConfig->cSSecondpindleStatus != 0)
//		{
//			//3.1.填写SLine
//			SLine SLineInfo_1;
//			memset(&SLineInfo_1, 0, sizeof(SLine));
//			//3.2初始化SLine的起点终点（机械坐标）
//			SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//			//3.3 终点C轴分度
//			SLineInfo_1.SEndPoint.dArray[AXIS_C] = pSFinalPoint->dArray[AXIS_C];
//			//3.4 填写实体类型 G0
//			SLineInfo_1.EntityType = OTHERTYPE;//G0
//			//3.5 将SLine写入缓冲区
//			writeLineIntoGCodeBuf(&SLineInfo_1, pGCodeBufUnit);
//			pSG84Para->cG84Step = 4;
//			return 1;
//		}
//		else//A轴没启动，无需执行C分度，也无需返回，继续执行函数
//		{
//			pSG84Para->cG84Step = 5;
//		}
//
//	}
//	//4.如果是A轴作为旋转加工轴，C轴度后主轴刹车锁紧
//	case 4:
//	{
//		if (pSAssignConfig->cSSecondpindleStatus != 0)
//		{
//			setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
//			pSG84Para->cG84Step = 5;
//			return 1;
//		}
//		else//A轴没启动，无需执行主轴刹车锁紧，也无需返回，继续执行函数
//		{
//			pSG84Para->cG84Step = 5;
//		}
//
//	}
//
//	//5.运动轴快速移动到R点
//	case 5:
//	{
//		//5.1.创建SLine
//		SLine SLineInfo_2;
//		memset(&SLineInfo_2, 0, sizeof(SLine));
//		//5.2初始化SLine的起点终点（机械坐标）
//		SLineInfo_2.SEndPoint = SLineInfo_2.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//		//5.3 终点为R点(c轴坐标不为R点的C，因为如果运动轴不为A轴时，把终点的C轴坐标赋值为R点的C坐标后，可能会导致执行C轴分度)
//		SLineInfo_2.SEndPoint.dArray[AXIS_Z] = pSRPoint->dArray[AXIS_Z];
//		SLineInfo_2.SEndPoint.dArray[AXIS_Y] = pSRPoint->dArray[AXIS_Y];
//		SLineInfo_2.SEndPoint.dArray[AXIS_X] = pSRPoint->dArray[AXIS_X];
//		//5.4 填写实体类型
//		SLineInfo_2.EntityType = OTHERTYPE;
//		//5.5 将Sline写入缓冲区中
//		writeLineIntoGCodeBuf(&SLineInfo_2, pGCodeBufUnit);
//		pSG84Para->cG84Step = 6;
//		return 1;
//	}
//
//	//6.按照切削深度Q进行攻牙(G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹)
//	case 6:
//	{
//		//6.1.创建STapping
//		STapping STappingInfo_3;
//		memset(&STappingInfo_3, 0, sizeof(STapping));
//		double dTemp_Znextpoint;//进给终点坐标
//		//6.2初始化SLine的起点终点（机械坐标）
//		STappingInfo_3.SEndPoint = STappingInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//
//		//6.3 Q值为0时 直接打到底部
//		if (isZero(pSG84Para->dQ))
//		{
//			dTemp_Znextpoint = pSFinalPoint->dArray[pSG84Para->cFeedAxis];
//		}
//		//Q值不为0时，需要分第几次进给Q
//		else
//		{
//			//第一次进给，进给量为Q，进给终点坐标为当前点坐标+方向系数*Q
//			if (pSG84Para->bIsFirstFeed == TRUE)
//			{
//				pSG84Para->cFeedQTimes = 1;
//				pSG84Para->bIsFirstFeed = FALSE;
//				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dQ * pSG84Para->cFeedQTimes;
//			}
//			// 非第一次进给，进给量为nQ,即cFeedQTimes*pSG84Para->dQ
//			else
//			{
//				pSG84Para->cFeedQTimes++;
//				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dQ * pSG84Para->cFeedQTimes;
//			}
//		}
//
//		//6.4 判断进给终点是否超过孔底坐标，并赋值步骤
//		// 进给终点超过孔底坐标,如果没有，则依然循环步骤6
//		if (isEqualOrGreaterThanZero((dTemp_Znextpoint - pSFinalPoint->dArray[pSG84Para->cFeedAxis]) * pSG84Para->cFeedDir))
//		{
//			//约束本次进给终点坐标为孔底坐标
//			dTemp_Znextpoint = pSFinalPoint->dArray[pSG84Para->cFeedAxis];
//			// 赋值步骤
//			pSG84Para->cG84Step = 7;
//
//		}
//		else
//		{
//            pSG84Para->cG84Step = 6;
//		}
//       //6.5赋值终点
//		STappingInfo_3.SEndPoint.dArray[pSG84Para->cFeedAxis] = dTemp_Znextpoint;
//		//6.6 赋值G33旋转轴
//		STappingInfo_3.G33RotateAxis=(pSAssignConfig->cSSecondpindleStatus != 0)? AXIS_A : AXIS_C;
//		//6.7 赋值G33攻牙模式 : A轴旋转：柔性攻牙    C轴旋转：刚性攻牙
//		STappingInfo_3.G33TappingMode=(pSAssignConfig->cSSecondpindleStatus != 0)? FLEXIBLE_TAPPING : 0;
//		//6.8 赋值G33头数
//		STappingInfo_3.usL = 1;
//        //6.9 赋值导程
//		STappingInfo_3.dF = pSG84Para->dF;
//		//6.10 赋值实体类型
//		STappingInfo_3.EntityType = TAPPING;
//
//		//6.11 将STapping写入缓冲区
//		writeTrappingIntoGCodeBuf(&STappingInfo_3,pGCodeBufUnit);
//		AssignM03(pSAssignConfig, pGCodeBufUnit);//重新开启主轴
//		return 1;
//	}
//
//	//从R点快速移到初始点 (G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹，即孔底-R点)
//	case 7:
//	{
//		//7.1.创建SLine
//		SLine SLineInfo_3;
//		memset(&SLineInfo_3, 0, sizeof(SLine));
//		//7.2初始化SLine的起点终点（机械坐标）
//		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//		//7.3赋值终点
//		SLineInfo_3.SEndPoint.dArray[pSG84Para->cFeedAxis] = pSG84Para->SFirstPoint.dArray[pSG84Para->cFeedAxis];
//		//7.4 填写实体类型
//		SLineInfo_3.EntityType = OTHERTYPE;
//		//7.5 将Sline写入缓冲区中
//		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);
//		pSG84Para->cG84Step = 8;
//		return 1;
//	}
//
//	//主轴刹车松开
//	case 8:
//	{
//		//如果是A轴作为旋转运动加工轴,最后要把主轴刹车松开
//		if (pSAssignConfig->cSSecondpindleStatus != 0)
//		{
//			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
//		}
//		pSG84Para->bIsFirstFeed = TRUE;
//		pSG84Para->cG84Step = 0;
//		return 2;
//	}
//	default:
//		break;
//	}
//}
//



//int8 CheckG84(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
//{
//	if (pSSyntaxConfig == NULL || pGCodeBufUnit == NULL)
//	{
//		return -1;
//	}
//	SPointDouble* pSRPoint = NULL;
//	SPointDouble* pSFinalPoint = NULL;
//	SPointDouble* pSFristPoint = NULL;
//	SPointDouble SCurPoint;
//	SG84Para* pSG84Para = NULL;
//
//	pSG84Para = pSG84ParaCheck[pSSyntaxConfig->ucChannelNum];
//	pSRPoint = &pSG84Para->SRPoint;//R点
//	pSFinalPoint = &pSG84Para->SFinalPoint;//孔底点
//	pSFristPoint = &pSG84Para->SFirstPoint;//初始点
//	SCurPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//当前点
//	bool bIsAbsMethod;
//
//	switch (pSG84Para->cG84Step)
//	{
//		//0 赋值预处理
//	case 0:
//	{
//		//0.1判断G84执行次数是否为0
//		if (pSG84Para->cK == 0)
//		{
//			//不执行G83
//			return 2;
//		}
//
//		//0.2判断G84程序段中是否出现坐标信息
//		int8 cRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
//		//0.2.1程序段出现了坐标信息G83才赋值
//		if (cRet == TRUE)
//		{
//
//			//1. 第一次进给标志位
//			pSG84Para->bIsFirstFeed = TRUE;
//			//2.进给轴（G84的进给轴为Z轴）
//			pSG84Para->cFeedAxis = AXIS_Z;
//			//3.计算孔底坐标
//
//			*pSFinalPoint = SCurPoint;
//			//获取当前的坐标编程方式
//			bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
//			//计算孔底坐标
//			getMacCoordByCoordInfo(pSSyntaxConfig->ucChannelNum, &pSG84Para->SCoordInfo, &SCurPoint, pSFinalPoint, &pSSyntaxConfig->SGCodeCurrSysState);
//
//
//			//4.进给方向  （-1：从Z正向往Z负向，1：从Z负向往Z正向）
//			pSG84Para->cFeedDir = (pSFinalPoint->dArray[pSG84Para->cFeedAxis] - SCurPoint.dArray[pSG84Para->cFeedAxis] < 0) ? -1 : 1;
//
//			//5.计算R点
//			//第一个孔
//			if (pSG84Para->bIsFirstHole == TRUE)
//			{
//				//R点在非进给方向的坐标
//				*pSRPoint = *pSFinalPoint;
//				//R点在进给方向的坐标
//				pSRPoint->dArray[pSG84Para->cFeedAxis] = SCurPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dR;
//
//			}
//			//非第一个孔
//			else
//			{
//				uint8 temp = 0;
//				for (; temp < AXIS_NUM_MAX; temp++)
//				{
//					//R点在非进给方向的坐标
//					//R点在进给方向的坐标无需更新
//					if (temp != pSG84Para->cFeedAxis)
//					{
//						pSRPoint->dArray[temp] = pSFinalPoint->dArray[temp];
//					}
//				}
//			}
//
//
//			//6.计算起始点
//		   //起始点在非进给方向的坐标
//			*pSFristPoint = *pSFinalPoint;
//			//起始点在进给方向的坐标
//			pSFristPoint->dArray[pSG84Para->cFeedAxis] = SCurPoint.dArray[pSG84Para->cFeedAxis];
//
//
//
//
//
//			pSG84Para->cG84Step = 1;
//
//			return 0;
//		}
//		else
//		{
//			//不执行G84
//			return 2;
//		}
//	}
//
//	//1.从当前点快速移动到初始点
//	case 1:
//	{
//		//1.1.填写SLine
//		SLine SLineInfo_0;
//		memset(&SLineInfo_0, 0, sizeof(SLine));
//		//1.2初始化SLine的起点终点（机械坐标）
//		SLineInfo_0.SEndPoint = SLineInfo_0.SStartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//		//1.3 终点为初始点
//		SLineInfo_0.SEndPoint = *pSFristPoint;
//		//1.4 填写实体类型 G0
//		SLineInfo_0.EntityType = OTHERTYPE;//G0
//		//1.5 将SLine写入缓冲区
//		writeLineIntoGCodeBuf(&SLineInfo_0, pGCodeBufUnit);
//
//		pSG84Para->cG84Step = 2;
//		return 1;
//	}
//
//	//2.如果是A轴作为旋转运动加工轴，先主轴刹车松开
//	case 2:
//	{
//		//A轴已被开启
//		if (pSSyntaxConfig->cSSecondpindleStatus != 0)
//		{
//			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
//			pSG84Para->cG84Step = 3;
//			return 1;
//		}
//		else//A轴没启动，无需执行主轴刹车松开，也无需返回，继续执行函数
//		{
//			pSG84Para->cG84Step = 5;
//		}
//
//	}
//	//3.如果是A轴作为旋转加工轴，C轴分度 G0
//	case 3:
//	{
//		if (pSSyntaxConfig->cSSecondpindleStatus != 0)
//		{
//			//3.1.填写SLine
//			SLine SLineInfo_1;
//			memset(&SLineInfo_1, 0, sizeof(SLine));
//			//3.2初始化SLine的起点终点（机械坐标）
//			SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//			//3.3 终点C轴分度
//			SLineInfo_1.SEndPoint.dArray[AXIS_C] = pSFinalPoint->dArray[AXIS_C];
//			//3.4 填写实体类型 G0
//			SLineInfo_1.EntityType = OTHERTYPE;//G0
//			//3.5 将SLine写入缓冲区
//			writeLineIntoGCodeBuf(&SLineInfo_1, pGCodeBufUnit);
//			pSG84Para->cG84Step = 4;
//			return 1;
//		}
//		else//A轴没启动，无需执行C分度，也无需返回，继续执行函数
//		{
//			pSG84Para->cG84Step = 5;
//		}
//
//	}
//	//4.如果是A轴作为旋转加工轴，C轴度后主轴刹车锁紧
//	case 4:
//	{
//		if (pSSyntaxConfig->cSSecondpindleStatus != 0)
//		{
//			setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
//			pSG84Para->cG84Step = 5;
//			return 1;
//		}
//		else//A轴没启动，无需执行主轴刹车锁紧，也无需返回，继续执行函数
//		{
//			pSG84Para->cG84Step = 5;
//		}
//
//	}
//
//	//5.运动轴快速移动到R点
//	case 5:
//	{
//		//5.1.创建SLine
//		SLine SLineInfo_2;
//		memset(&SLineInfo_2, 0, sizeof(SLine));
//		//5.2初始化SLine的起点终点（机械坐标）
//		SLineInfo_2.SEndPoint = SLineInfo_2.SStartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//		//5.3 终点为R点(c轴坐标不为R点的C，因为如果运动轴不为A轴时，把终点的C轴坐标赋值为R点的C坐标后，可能会导致执行C轴分度)
//		SLineInfo_2.SEndPoint.dArray[AXIS_Z] = pSRPoint->dArray[AXIS_Z];
//		SLineInfo_2.SEndPoint.dArray[AXIS_Y] = pSRPoint->dArray[AXIS_Y];
//		SLineInfo_2.SEndPoint.dArray[AXIS_X] = pSRPoint->dArray[AXIS_X];
//		//5.4 填写实体类型
//		SLineInfo_2.EntityType = OTHERTYPE;
//		//5.5 将Sline写入缓冲区中
//		writeLineIntoGCodeBuf(&SLineInfo_2, pGCodeBufUnit);
//		pSG84Para->cG84Step = 6;
//		return 1;
//	}
//
//	//6.按照切削深度Q进行攻牙(G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹)
//	case 6:
//	{
//		//6.1.创建STapping
//		STapping STappingInfo_3;
//		memset(&STappingInfo_3, 0, sizeof(STapping));
//		double dTemp_Znextpoint;//进给终点坐标
//		//6.2初始化SLine的起点终点（机械坐标）
//		STappingInfo_3.SEndPoint = STappingInfo_3.SStartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//
//		//6.3 Q值为0时 直接打到底部
//		if (isZero(pSG84Para->dQ))
//		{
//			dTemp_Znextpoint = pSFinalPoint->dArray[pSG84Para->cFeedAxis];
//		}
//		//Q值不为0时，需要分第几次进给Q
//		else
//		{
//			//第一次进给，进给量为Q，进给终点坐标为当前点坐标+方向系数*Q
//			if (pSG84Para->bIsFirstFeed == TRUE)
//			{
//				pSG84Para->cFeedQTimes = 1;
//				pSG84Para->bIsFirstFeed = FALSE;
//				dTemp_Znextpoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dQ * pSG84Para->cFeedQTimes;
//			}
//			// 非第一次进给，进给量为nQ,即cFeedQTimes*pSG84Para->dQ
//			else
//			{
//				pSG84Para->cFeedQTimes++;
//				dTemp_Znextpoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG84Para->cFeedAxis] + pSG84Para->cFeedDir * pSG84Para->dQ * pSG84Para->cFeedQTimes;
//			}
//		}
//
//		//6.4 判断进给终点是否超过孔底坐标，并赋值步骤
//		// 进给终点超过孔底坐标,如果没有，则依然循环步骤6
//		if (isEqualOrGreaterThanZero((dTemp_Znextpoint - pSFinalPoint->dArray[pSG84Para->cFeedAxis]) * pSG84Para->cFeedDir))
//		{
//			//约束本次进给终点坐标为孔底坐标
//			dTemp_Znextpoint = pSFinalPoint->dArray[pSG84Para->cFeedAxis];
//			// 赋值步骤
//			pSG84Para->cG84Step = 7;
//
//		}
//		else
//		{
//			pSG84Para->cG84Step =6;
//		}
//		//6.5赋值终点
//		STappingInfo_3.SEndPoint.dArray[pSG84Para->cFeedAxis] = dTemp_Znextpoint;
//		//6.6 赋值G33旋转轴
//		STappingInfo_3.G33RotateAxis = (pSSyntaxConfig->cSSecondpindleStatus != 0) ? AXIS_A : AXIS_C;
//		//6.7 赋值G33攻牙模式 : A轴旋转：柔性攻牙    C轴旋转：刚性攻牙
//		STappingInfo_3.G33TappingMode = (pSSyntaxConfig->cSSecondpindleStatus != 0) ? FLEXIBLE_TAPPING : 0;
//		//6.8 赋值G33头数
//		STappingInfo_3.usL = 1;
//		//6.9 赋值导程
//		STappingInfo_3.dF = pSG84Para->dF;
//		//6.10 赋值实体类型
//		STappingInfo_3.EntityType = TAPPING;
//
//		//6.11 将STapping写入缓冲区
//		writeTrappingIntoGCodeBuf(&STappingInfo_3, pGCodeBufUnit);
//		return 1;
//	}
//
//	//从R点快速移到初始点 (G33底层运动会走两段：起点-终点，终点-起点，所以无需赋值终点-起点这段轨迹，即孔底-R点)
//	case 7:
//	{
//		//7.1.创建SLine
//		SLine SLineInfo_3;
//		memset(&SLineInfo_3, 0, sizeof(SLine));
//		//7.2初始化SLine的起点终点（机械坐标）
//		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
//		//7.3赋值终点
//		SLineInfo_3.SEndPoint.dArray[pSG84Para->cFeedAxis] = pSG84Para->SFirstPoint.dArray[pSG84Para->cFeedAxis];
//		//7.4 填写实体类型
//		SLineInfo_3.EntityType = OTHERTYPE;
//		//7.5 将Sline写入缓冲区中
//		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);
//		pSG84Para->cG84Step = 8;
//		return 1;
//	}
//
//	//主轴刹车松开
//	case 8:
//	{
//		//如果是A轴作为旋转运动加工轴,最后要把主轴刹车松开
//		if (pSSyntaxConfig->cSSecondpindleStatus != 0)
//		{
//			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
//		}
//		pSG84Para->bIsFirstFeed = TRUE;
//		pSG84Para->cG84Step = 0;
//		return 2;
//	}
//	default:
//		break;
//	}
//
//}