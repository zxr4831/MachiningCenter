#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"
#include "GeneralFunctions.h"
#include <stdint.h>
#include <math.h>
#define HIGHSPEEDDRILLING_OF_DEEPHOLES 1	//高速钻深孔	G83.1											运行到加工点，退刀至R点，再进刀刀加工点，以此循环，最后运动到R点，分度在R点
#define COUNTERSUNKHOLE_PECKING_DRILL  2	//沉孔啄钻		G83.2	公司自己的逻辑							运行到加工点，退刀至NC089,再进刀刀加工点，以此循环，最后运动到起始点，分度在起始点
#define GENERALDEEPHOLE_PECKING_DRILL  3   //一般深孔啄钻	G83.3	默认(公司加工师傅要求) 跟新代系统一样，	进到到加工点，退刀到R点，再进刀刀加工点，以此循环，最后运动到起始点，分度在起始点


static SG83Para* pSG83ParaCheck[CHANNEL_NUM_MAX];
static SG83Para* pSG83ParaAssign[CHANNEL_NUM_MAX];

//添加G83规则结构体
void addG83RegulationAPI()
{
	//指令：G83
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(83, sGa, 1, mallocG83ParaStruct, freeG83ParaStruct, getG83Para, CheckG83_3, AssignG83);
}
//添加G84规则结构体
//void addG84RegulationAPI()
//{
//	//指令：G84
//	//组别：sGa
//	//指令占程序段数：1
//	addGCommadRegulation(84, sGa, 1, mallocG83ParaStruct, freeG83ParaStruct, getG83Para, CheckG83_3, AssignG83);
//}

/*************************************************
  函数名称：int8 mallocG83ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G83参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG83ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG83ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG83ParaCheck[ucChannelNum] =(SG83Para*) mallocGCode(sizeof(SG83Para), ORAM_NUM430);//暂未对内存编号
		if (pSG83ParaCheck[ucChannelNum] == NULL)
		{
			return -1;
		}
		memset(pSG83ParaCheck[ucChannelNum], 0, sizeof(SG83Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG83ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG83ParaAssign[ucChannelNum] = (SG83Para*)mallocGCode(sizeof(SG83Para), ORAM_NUM431);
		if (pSG83ParaAssign[ucChannelNum] == NULL)
		{
			return-1;
		}
		memset(pSG83ParaAssign[ucChannelNum], 0, sizeof(SG83Para));


		return 1;
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：释放G83参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG83ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG83ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG83ParaCheck[ucChannelNum]);
			pSG83ParaCheck[ucChannelNum] = NULL;
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG83ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG83ParaAssign[ucChannelNum]);
			pSG83ParaAssign[ucChannelNum] = NULL;
			return 1;
		}
	}
	return 0;
}

/*************************************************
  函数名称：int8 getG83Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能   获取G83参数（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG83Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
{
	if (pSScanContainer == NULL ||pSGCodeCurrSysState == NULL ||ucChannelNum >= CHANNEL_NUM_MAX 
		||(ucMode != GC_SYNTAX && ucMode != GC_ASSIGN))
	{
		return -1;
	}
	/* 检查和赋值各自维护坐标及单位，不能跨模式访问参数。 */
	SG83Para* pSG83Para = (ucMode == GC_SYNTAX) ?pSG83ParaCheck[ucChannelNum] : pSG83ParaAssign[ucChannelNum];
	if (pSG83Para == NULL)
	{
		parseGCodeErrorB(pSScanContainer->unLineNum,GCODE_ERROR_NUM300);
		return -1;
	}

	//获取默认单位
	bool bImperial = pSGCodeCurrSysState->bImperialProgramming;
	const SGCommandRegulation* pUnitRegulation =pSScanContainer->SScanModalInfo.pSGeRegulation;

	//如果已有单位指令，根据单位指令判断是否为英制单位
	if (pUnitRegulation != NULL)
	{
		bImperial =pUnitRegulation->nCommandVal ==G20 * pow(10, GCOMMAND_DECIMAL_PLACE);
	}
	double dLengthScale = bImperial ? 25.4 : 1.0;
	
	/*
	* 先把旧坐标换成当前段的单位，再由下面原有代码合并新坐标。
	* 这样省略的轴保持原物理长度，新输入的轴按当前段单位解释。
	*/
	if (pSG83Para->bCoordImperial != bImperial)
	{
		double dCoordScale =
			pSG83Para->bCoordImperial ? 25.4 : (1.0 / 25.4);

		for (uint8 i = 0; i < 3; ++i)
		{
			if (pSG83Para->SCoordInfo.bXYZ[i] == TRUE)
			{
				pSG83Para->SCoordInfo.dXYZ[i] *= dCoordScale;
			}

			if (pSG83Para->SCoordInfo.bUVW[i] == TRUE)
			{
				pSG83Para->SCoordInfo.dUVW[i] *= dCoordScale;
			}
		}
	}

	/* 保存的是坐标当前实际采用的单位，而不是最初输入时的单位。 */
	pSG83Para->bCoordImperial = bImperial;

	if (ucMode == GC_SYNTAX)
	{
		pSG83ParaCheck[ucChannelNum]->cG83Step = 0;
	

		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG83ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);
		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入考虑过单位的半径量)）
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG83ParaCheck[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0])* dLengthScale;
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG83ParaCheck[ucChannelNum]->dQ = fabs(pSScanContainer->dQ)* dLengthScale;
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG83ParaCheck[ucChannelNum]->unDelaytime = (uint32)(fabs(pSScanContainer->dP) * 1000);
		}
		//4.获取K（G83重复执行的次数）
		if (pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG83ParaCheck[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
		}
		else//无输入K时，默认1次
		{
			pSG83ParaCheck[ucChannelNum]->cK = 1;
		}

		if (pSScanContainer->bD == TRUE)
		{
			pSG83ParaCheck[ucChannelNum]->dD = (double)fabs(pSScanContainer->dD)* dLengthScale;  //zxd:20251024
		}
		else
		{
			pSG83ParaCheck[ucChannelNum]->dD = 0;
		}
		if (s_pSChannelConfig[0]->SSyntaxConfig.pSCurrCommandRegulation->nCommandVal == (G84 * pow(10, GCOMMAND_DECIMAL_PLACE)))
		//if (pSScanContainer->SScanModalInfo.pSGaRegulation->nCommandVal == (G84 * pow(10, GCOMMAND_DECIMAL_PLACE)))
		{
			pSG83ParaCheck[ucChannelNum]->bG84Flag = TRUE;
			//获取F（导程）
			if (pSScanContainer->bF == TRUE)
			{
				pSG83ParaCheck[ucChannelNum]->dF = fabs(pSScanContainer->dF);
			}
			else //无输入F时，默认1次
			{
				pSG83ParaCheck[ucChannelNum]->dF = 1;
			}
		}

		//记录G83上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);

		pSG83ParaCheck[ucChannelNum]->bIsFirstHole = TRUE;

		return 1;


	}
	else if (ucMode == GC_ASSIGN)
	{
		
		pSG83ParaAssign[ucChannelNum]->cG83Step = 0;
	
		//1.获取程序段坐标信息（为加入总偏移值的孔底坐标）
		getCoordInfoFromScanInfo(&pSG83ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);

		//1.获取R（初始点到R点的增量值(正负号无效，无论钻孔方向之轴向为直/半径轴，R引数带入半径量即可)）
		if (pSScanContainer->bRIJK[0]==TRUE)
		{
			pSG83ParaAssign[ucChannelNum]->dR = fabs(pSScanContainer->dRIJK[0])* dLengthScale;
		}
		//2.获取Q  （每次进给深度(正负号无效) ）
		if (pSScanContainer->bQ==TRUE)
		{
			pSG83ParaAssign[ucChannelNum]->dQ= fabs(pSScanContainer->dQ)* dLengthScale;
		}
		//3.获取P（洞底暂停时间(有小数点，以秒为单位；）
		if (pSScanContainer->bP == TRUE)
		{
			pSG83ParaAssign[ucChannelNum]->unDelaytime =(uint32)(fabs(pSScanContainer->dP)*1000); 
		}
		//4.获取K（G83重复执行的次数）
		if (pSScanContainer->bRIJK[3]==TRUE)
		{
			pSG83ParaAssign[ucChannelNum]->cK = (int8)fabs(pSScanContainer->dRIJK[3]);
		}
		else//无输入K时，默认1次
		{
			pSG83ParaAssign[ucChannelNum]->cK = 1;
		}
		if (pSScanContainer->bD == TRUE)
		{
			pSG83ParaAssign[ucChannelNum]->dD = (double)fabs(pSScanContainer->dD)* dLengthScale;  //zxd:20251024
		}
		else
		{
			pSG83ParaAssign[ucChannelNum]->dD = 0;
		}
		if (s_pSChannelConfig[0]->SAssignConfig.pSCurrCommandRegulation->nCommandVal == (G84 * pow(10, GCOMMAND_DECIMAL_PLACE)))
		//if (pSScanContainer->SScanModalInfo.pSGaRegulation->nCommandVal == (G84 * pow(10, GCOMMAND_DECIMAL_PLACE)))
		{
			pSG83ParaAssign[ucChannelNum]->bG84Flag = TRUE;
			//获取F（导程）
			if (pSScanContainer->bF == TRUE)
			{
				
				pSG83ParaAssign[ucChannelNum]->dF = fabs(pSScanContainer->dF);
			}
			else //无输入F时，默认1次
			{
				pSG83ParaAssign[ucChannelNum]->dF = 1;
			}
		}
		//记录G83上一行的模态
		setGaBeforeG8x(ucMode, ucChannelNum, pSGCodeCurrSysState->SModalInfo.sGa, pSGCodeCurrSysState->SModalInfo.pSGaRegulation);
	  
		pSG83ParaAssign[ucChannelNum]->bIsFirstHole = TRUE;
	
		return 1;
	}
}

/*************************************************
  函数名称：int8 AssignG83(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值G83
  输入参数：struct _SAssignConfiguration* pSAssignConfig		赋值结构体指针
			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
  输出参数：
  返 回 值：-1	参数无效；孔底边界无效时登记报警 301
			0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG83(struct _SAssignConfiguration*pSAssignConfig, struct SGCodeParse*pGCodeBufUnit)
{
	if (pSAssignConfig==NULL||pGCodeBufUnit==NULL)
	{
		return -1;
	}
	SPointDouble* pSRPoint = NULL;
	SPointDouble* pSFinalPoint = NULL;
	SPointDouble* pSFristPoint = NULL;
	SPointDouble SCurPoint ;
	SG83Para* pSG83Para = NULL;

	pSG83Para = pSG83ParaAssign[pSAssignConfig->ucChannelNum];
	pSRPoint = &pSG83Para->SRPoint;//R点
	pSFinalPoint = &pSG83Para->SFinalPoint;//孔底点
	pSFristPoint = &pSG83Para->SFirstPoint;//初始点
    SCurPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//当前点
	bool bIsAbsMethod;

	switch (pSG83Para->cG83Step)
	{
     //0 赋值预处理
	case 0:
	{
		//0.1判断G83执行次数是否为0
		if (pSG83Para->cK == 0)
		{
			//不执行G83
			return 2;
		}

		//0.2判断G83程序段中是否出现坐标信息
		int8 cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
		//0.2.1程序段出现了坐标信息G83才赋值
		if (cRet==TRUE)
		{

			//1. 第一次进给标志位
			pSG83Para->bIsFirstFeed = TRUE;
			//2.进给轴（G83的进给轴为Z轴）
			pSG83Para->cFeedAxis = AXIS_Z;
			//3.计算孔底坐标

			*pSFinalPoint = SCurPoint;
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			
			//计算孔底坐标。保护原始坐标基准，产生的修改留在 SCoordForCalc，不写回 pSG83Para->SCoordInfo。
			//原始坐标基准
			SCoord SCoordForCalc = pSG83Para->SCoordInfo;
			//计算机械孔底坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum,&SCoordForCalc,&SCurPoint,pSFinalPoint,&pSAssignConfig->SGCodeCurrSysState);

			//4.进给方向  （-1：从Z正向往Z负向，1：从Z负向往Z正向）
			pSG83Para->cFeedDir = (pSFinalPoint->dArray[pSG83Para->cFeedAxis] - SCurPoint.dArray[pSG83Para->cFeedAxis] < 0) ? -1 : 1;

			//5.计算R点
			//第一个孔
			if (pSG83Para->bIsFirstHole ==TRUE)
			{
				//R点在非进给方向的坐标
				*pSRPoint = *pSFinalPoint;
				//R点在进给方向的坐标
				pSRPoint->dArray[pSG83Para->cFeedAxis] = SCurPoint.dArray[pSG83Para->cFeedAxis] + pSG83Para->cFeedDir * pSG83Para->dR;
		
			}
			//非第一个孔
			else
			{
				uint8 temp = 0;
				for (;temp< AXIS_NUM_MAX;temp++)
				{
					//R点在非进给方向的坐标
					//R点在进给方向的坐标无需更新
					if (temp!= pSG83Para->cFeedAxis)
					{
						pSRPoint->dArray[temp] = pSFinalPoint->dArray[temp];
					}
				}
			}

	     
			/* 机械坐标单位为 mm，首孔与重复孔均检查本次有效 R 点。 */
			double dBottomPos = pSFinalPoint->dArray[pSG83Para->cFeedAxis];
			double dRPos = pSRPoint->dArray[pSG83Para->cFeedAxis];
			double dFeedDistance = (dBottomPos - dRPos) * pSG83Para->cFeedDir;

			/* 越底、容差内等深或非有限结果均报错，不生成本孔运动段。 */
			if (!isfinite(dFeedDistance) || dFeedDistance < 0.0 || isZero(dFeedDistance))
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
				return -1;
			}

			/* 只读系统参数中的逻辑轴映射和物理轴当量，不修改轴配置。 */
			uint8 ucFeedPhysicalAxis = g_SSystemPara.ucAxisMapArray[pSAssignConfig->ucChannelNum][pSG83Para->cFeedAxis];
			/* 未映射或越界的进给轴不可读取参数，沿用轴映射报警。 */
			if (ucFeedPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM150);
				return -1;
			}
			double dFeedPulseToMm = g_SSystemPara.SAxisPara[ucFeedPhysicalAxis].dPulseToMm;
			/* 当量须为有效正数，零值容差与实际脉冲输出入口一致。 */
			if (!isfinite(dFeedPulseToMm) || dFeedPulseToMm <= 0.0 || isZero(dFeedPulseToMm))
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM153);
				return -1;
			}
			/* 计划起点可能含未输出的亚脉冲位移，只读实时指令机械坐标确定格点。 */
			double dFeedReferencePos = 0.0;
			/* 无法取得有效参考位置时拒绝本孔，不以计划坐标代替。 */
			if (getRealTimePoint(&dFeedReferencePos, 1, &ucFeedPhysicalAxis, NULL) != 1 || !isfinite(dFeedReferencePos))
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
				return -1;
			}
			/* 半脉冲边界取靠近孔底的一侧，再核对剩余进给；不改写轨迹坐标。 */
			double dRPulseOffset = (dRPos - dFeedReferencePos) / dFeedPulseToMm;
			double dRPulse = pSG83Para->cFeedDir > 0 ? floor(dRPulseOffset + 0.5) : ceil(dRPulseOffset - 0.5);
			double dActualRPos = dFeedReferencePos + dRPulse * dFeedPulseToMm;
			double dFeedPulses = round((dBottomPos - dActualRPos) / dFeedPulseToMm) * pSG83Para->cFeedDir;

			/* 几何正残差不足以形成一个正向进给脉冲时，仍按无有效孔深报错。 */
			if (!isfinite(dFeedPulses) || dFeedPulses < 1.0)
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
				return -1;
			}

            //6.计算起始点
		   //起始点在非进给方向的坐标
			*pSFristPoint = *pSFinalPoint;
			//起始点在进给方向的坐标
			pSFristPoint->dArray[pSG83Para->cFeedAxis] = SCurPoint.dArray[pSG83Para->cFeedAxis];

			pSG83Para->cG83Step = 1;

			return 0 ;
		}
		else
		{
			//不执行G83
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
		SLineInfo_0.SEndPoint= *pSFristPoint;
		//2.4 填写实体类型 G0
		SLineInfo_0.EntityType = OTHERTYPE;//G0
		//2.5 将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_0, pGCodeBufUnit);

		pSG83Para->cG83Step =2;
		return 1;
	}


	//1.如果是A轴作为旋转运动加工轴，先主轴刹车松开
	case 2:
	{
		//A轴已被开启
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG83Para->cG83Step = 3;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车松开，也无需返回，继续执行函数
		{
			pSG83Para->cG83Step = 5;
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
			pSG83Para->cG83Step = 4;
			return 1;
		}
		else//A轴没启动，无需执行C分度，也无需返回，继续执行函数
		{
			pSG83Para->cG83Step = 5;
		}

	}
	//3.如果是A轴作为旋转加工轴，C轴度后主轴刹车锁紧
	case 4:
	{
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M38, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
			pSG83Para->cG83Step =5;
			return 1;
		}
		else//A轴没启动，无需执行主轴刹车锁紧，也无需返回，继续执行函数
		{
			pSG83Para->cG83Step = 5;
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
		pSG83Para->cG83Step = 6;
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
		if (isZero(pSG83Para->dQ))
		{
			dTemp_Znextpoint = pSFinalPoint->dArray[pSG83Para->cFeedAxis];
		}
		//5.3.2 Q值不为0，则需要分第一次进给与非第一次进给
		else
		{
			//第一次进给，进给量为Q，进给终点坐标为当前点坐标+方向系数*Q
			if (pSG83Para->bIsFirstFeed==TRUE)
			{
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG83Para->cFeedAxis] + pSG83Para->cFeedDir * pSG83Para->dQ;
			}
			//非第一次进给，进给量为Q+d ，进给终点坐标为当前点坐标+方向系数*（Q+d）   d：系统参数：G83toG83退刀参数_退刀长度 um
			else
			{
				dTemp_Znextpoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG83Para->cFeedAxis] + pSG83Para->cFeedDir * (pSG83Para->dQ + g_SSystemPara.sG83toG89Para_RetractionLength * TEMP_LEAST_INPUT_INCREMENT);
			}
		}

		//5.4 判断进给终点是否超过孔底坐标，并赋值步骤
		//本次进给抵达孔底或者超过孔底
		if (isEqualOrGreaterThanZero((dTemp_Znextpoint - pSFinalPoint->dArray[pSG83Para->cFeedAxis]) * pSG83Para->cFeedDir))
		{
			//约束本次进给终点坐标为孔底坐标
			dTemp_Znextpoint = pSFinalPoint->dArray[pSG83Para->cFeedAxis];
			// 赋值步骤
			pSG83Para->cG83Step = 9;

		}
		//本次进给没有抵达孔底或者没有超过孔底
		else
		{

			pSG83Para->cG83Step =7;
		}
		//5.5如果为第一次进给，第一次进给计算完毕后将第一次标志位置为FALSE
		if (pSG83Para->bIsFirstFeed==TRUE)
		{
			pSG83Para->bIsFirstFeed = FALSE;
		}

		//5.6 将轨迹写入赋值缓冲区
		//填写实体类型
		SLineInfo_3.EntityType = L1NE;
		//赋值Sline终点坐标
		SLineInfo_3.SEndPoint.dArray[pSG83Para->cFeedAxis] = dTemp_Znextpoint;
		//G84需要加参数(参数结构体内添加变量bG84Flag)
		if (pSG83Para->bG84Flag == TRUE)
		{
			//1.修改实体类型
			SLineInfo_3.EntityType = L1NE;
			//2.修正进给速度(进给速度 = 转速 *螺距、60/1000) 转化为ms
			pGCodeBufUnit->dFeedrate = pSG83Para->dF * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
			//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
			pGCodeBufUnit->dG84Ratio = 360 / pSG83Para->dF*sgn(SLineInfo_3.SStartPoint.dArray[pSG83Para->cFeedAxis] - SLineInfo_3.SEndPoint.dArray[pSG83Para->cFeedAxis]);
			//4.设置C轴为有效轴
			setbit(pGCodeBufUnit->ucValStatus, AXIS_C);
		}
		// 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);
		return 1;
	}
	//7.快速退刀并计算下一次切削起始点
	case 7:
	{
		//6.1.创建SLine
		SLine SLineInfo_4;
		memset(&SLineInfo_4, 0, sizeof(SLine));
		if (pSG83Para->dD == 0)//没有输入D参数 ，回退到R点
		{
			//6.2初始化SLine的起点终点（机械坐标）
			SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
			//6.3 计算SLine终点:R点
			SLineInfo_4.SEndPoint.dArray[pSG83Para->cFeedAxis] = pSRPoint->dArray[pSG83Para->cFeedAxis];
		}
		else
		{
			SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
			//6.3 计算SLine终点:回退点D点
			SLineInfo_4.SEndPoint.dArray[pSG83Para->cFeedAxis] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[pSG83Para->cFeedAxis] - pSG83Para->cFeedDir * pSG83Para->dD;
		}

		//6.4 计算下一次进给的切削起始点
		//非进给方向坐标
		pSG83Para->SCuttingStartPoint= pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		//进给方向的坐标=当前点-（方向系数*d）
		pSG83Para->SCuttingStartPoint.dArray[pSG83Para->cFeedAxis]-= pSG83Para->cFeedDir* g_SSystemPara.sG83toG89Para_RetractionLength* TEMP_LEAST_INPUT_INCREMENT;
		//6.5 将轨迹写入缓冲区中
		//填写实体类型 G0
		SLineInfo_4.EntityType = OTHERTYPE;
		//G84需要加参数(参数结构体内添加变量bG84Flag)
		if (pSG83Para->bG84Flag == TRUE)
		{
			//1.修改实体类型
			SLineInfo_4.EntityType = L1NE;
			//2.修正进给速度(进给速度 = 转速 *螺距)
			pGCodeBufUnit->dFeedrate = pSG83Para->dF * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
			//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
			pGCodeBufUnit->dG84Ratio = 360 / pSG83Para->dF * sgn(SLineInfo_4.SStartPoint.dArray[pSG83Para->cFeedAxis] - SLineInfo_4.SEndPoint.dArray[pSG83Para->cFeedAxis]);
			//4.设置C轴为有效轴
			setbit(pGCodeBufUnit->ucValStatus, AXIS_C);
		}
		// 将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SLineInfo_4, pGCodeBufUnit);
		//6.6赋值G83步骤
		pSG83Para->cG83Step = 8;
		return 1;
	}
	//8.快速移动到切削起始点
	case 8:
	{
		//6.1 创建SLine
		SLine SlineInfo_5;
		memset(&SlineInfo_5, 0, sizeof(SLine));
		//6.2初始化SLine的起点终点（机械坐标）
		SlineInfo_5.SEndPoint = SlineInfo_5.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//6.3计算Sline终点
		SlineInfo_5.SEndPoint.dArray[pSG83Para->cFeedAxis] = pSG83Para->SCuttingStartPoint.dArray[pSG83Para->cFeedAxis];
		//6.4将轨迹写入缓冲区中
		//填写实体类型  G0
		SlineInfo_5.EntityType = OTHERTYPE;
		//G84需要加参数(参数结构体内添加变量bG84Flag)
		if (pSG83Para->bG84Flag == TRUE)
		{
			//1.修改实体类型
			SlineInfo_5.EntityType = L1NE;
			//2.修正进给速度(进给速度 = 转速 *螺距)
			pGCodeBufUnit->dFeedrate = pSG83Para->dF * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
			//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
			pGCodeBufUnit->dG84Ratio = 360 / pSG83Para->dF * sgn(SlineInfo_5.SStartPoint.dArray[pSG83Para->cFeedAxis] - SlineInfo_5.SEndPoint.dArray[pSG83Para->cFeedAxis]);
			//4.设置C轴为有效轴
			setbit(pGCodeBufUnit->ucValStatus,AXIS_C);
		}
		//将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SlineInfo_5, pGCodeBufUnit);

		//6.5赋值G83步骤
		pSG83Para->cG83Step = 6;
		return 1;
	}
	//8.孔底暂停
	case 9:
	{
		//暂停时间不为0
		if (pSG83Para->unDelaytime>0)
		{
			//赋值延时时间
			pGCodeBufUnit->unDelayTime = pSG83Para->unDelaytime;
	       pSG83Para->cG83Step = 10;
			return 1;
		}
		else //没有暂停时间，无需退出函数，直接下一步
		{
			pSG83Para->cG83Step = 10;
		}
	}
	//9.快速退刀至初始点,并判断钻孔执行次数
	case 10:
	{
		//9.1 创建SLine
		SLine SlineInfo_6;
		memset(&SlineInfo_6, 0, sizeof(SLine));
		//9.2初始化SLine的起点终点（机械坐标）
		SlineInfo_6.SEndPoint = SlineInfo_6.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//9.3计算Sline终点  :初始点
		SlineInfo_6.SEndPoint.dArray[pSG83Para->cFeedAxis] = pSFristPoint->dArray[pSG83Para->cFeedAxis];
		//9.4 将轨迹写入缓冲区中
		//填写实体类型 G0
		SlineInfo_6.EntityType = OTHERTYPE;
		if (pSG83Para->bG84Flag == TRUE)
		{
			//1.修改实体类型
			SlineInfo_6.EntityType = L1NE;
			//2.修正进给速度(进给速度 = 转速 *螺距)
			pGCodeBufUnit->dFeedrate = pSG83Para->dF * pSAssignConfig->SGCodeCurrSysState.dTappingSpeed / 60 / 1000;
			//3.C轴相对于Z轴的倍率(缓冲区内添加变量dG84Ratio)
			pGCodeBufUnit->dG84Ratio = 360 / pSG83Para->dF * sgn(SlineInfo_6.SStartPoint.dArray[pSG83Para->cFeedAxis] - SlineInfo_6.SEndPoint.dArray[pSG83Para->cFeedAxis]);
			//4.设置C轴为有效轴
			setbit(pGCodeBufUnit->ucValStatus, AXIS_C);
		}
		//将Sline写入缓冲区中
		writeLineIntoGCodeBuf(&SlineInfo_6, pGCodeBufUnit);

		//9.5 判断钻孔执行次数,并赋值G83的步骤
		pSG83Para->cK -= 1;
		if (pSG83Para->cK==0)
		{
			pSG83Para->cG83Step = 11;
		}
		else
		{
			pSG83Para->cG83Step =0;
			pSG83Para->bIsFirstFeed = TRUE;
			pSG83Para->bIsFirstHole = FALSE;
		}
		return 1;
	}
	//10.主轴刹车松开
	case 11:
	{
		//如果是A轴作为旋转运动加工轴,最后要把主轴刹车松开
		if (pSAssignConfig->cSSecondpindleStatus != 0)
		{
			setIO(M39, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
		}
		pSG83Para->bIsFirstFeed = TRUE;
		pSG83Para->cG83Step =0;
		return 2;
	}
	default:
		return 1;
		break;
	}
}

int8 CheckG83_3(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
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
