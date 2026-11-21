#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"
#include"GeneralFunctions.h"
#include "GCode.h"

static SG34Para* pSG34ParaCheck[CHANNEL_NUM_MAX];
static SG34Para* pSG34ParaAssign[CHANNEL_NUM_MAX];

/*
指令格式：
G34 X(U)__ Z(W)__ F(I)__ J__ K__ R__ ；
代码说明：
（1）G34为模态G代码；
（2）X：切削终点X轴绝对坐标；
（3）U：切削终点与起点X轴绝对坐标的差值；
（4）Z：切削终点Z轴绝对坐标；
（5）W：切削终点与起点Z轴绝对坐标的差值；
（6）F：螺纹导程，F指定值执行后保持，可省略输入；
（7）I：螺纹每英寸牙数， I 指定值执行后保持，可省略输入；
（8）J：螺纹退尾时在短轴方向的移动量，取值范围0～99999999×最小输入增量， 单位：mm./inch。不带方向(根据程序起点位置自动确定退尾方向)，模态参数，如果短轴是X轴，则该值为半径指定；
（9）K：螺纹退尾时在长轴方向的长度，取值范围0～99999999×最小输入增量，单 位：mm/inch。不带方向，模态参数，如长轴是X轴，该值为半径指定；
（10）R：主轴每转螺距的增量值或减量值，R=F2-F1，R带有方向；F1>F2时，R为负 值时螺距递减；F1<F2 时，R为正值时螺距递增。
	 R 值的范围：±0.001～±500.000 毫米/每螺距(公制螺纹)；±0.060～±2540 牙/每英寸(英制螺纹)。 当R值超过上述范围值和因R的增加/减小使螺距超过允许值或螺距出现负值时产生报警。
*/


//添加G34规则结构体
void addG34RegulationAPI()
{

	addGCommadRegulation(34, sGa, 1, mallocG34ParaStruct, freeG34ParaStruct, getG34Para, CheckG34, AssignG34);
}

/*************************************************
  函数名称：int8 mallocG34ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G34参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG34ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG34ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG34ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG34Para), ORAM_NUM412);//暂未对内存编号
		if (pSG34ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG34ParaCheck[ucChannelNum], 0, sizeof(SG34Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG34ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG34ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG34Para), ORAM_NUM413);
		if (pSG34ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG34ParaAssign[ucChannelNum], 0, sizeof(SG34Para));
		return 1;
	}
}

/*************************************************
  函数名称：int8 freeG34ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：释放G34参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG34ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG34ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG34ParaCheck[ucChannelNum]);
			pSG34ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG34ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG34ParaAssign[ucChannelNum]);
			pSG34ParaAssign[ucChannelNum] = NULL;
		}
	}
}

/*************************************************
  函数名称：int8 getG34Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能   获取G34参数（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG34Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	if (SGCodeCurrSysState == NULL || pSScanContainer == NULL)
	{
		return -1;
	}
	if (ucMode == GC_SYNTAX)
	{
		pSG34ParaCheck[ucChannelNum]->cStep = 0;

		//1.获取R
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG34ParaCheck[ucChannelNum]->dR = pSScanContainer->dRIJK[0];
		}
		//2.获取导程F  (F与I互斥，先获取导程值（这里为值覆盖），后面再判断互斥性)
		if (pSScanContainer->bF == TRUE)
		{
			pSG34ParaCheck[ucChannelNum]->dF = pSScanContainer->dF;
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			pSG34ParaCheck[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
		}

		//3.获取长轴退尾量、短轴退尾量
		//3.根据手册要求获取短轴退尾量，长轴退尾量
		//3.1 G32省略退尾时 ：省略J或J、K时，无退尾；
		if (pSScanContainer->bRIJK[2] == FALSE)
		{
			//J、K是模态代码，上一个程序段为G34，当前程序段的G34省略J、K时，按前面的J、K值进行退尾，
			if (SGCodeCurrSysState->SModalInfo.sGa == G34 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{

			}
			//省略J或J、K时且不是连续螺纹的下一段，无退尾；
			else
			{
				pSG34ParaCheck[ucChannelNum]->dJ = 0;
				pSG34ParaCheck[ucChannelNum]->dK = 0;
			}
		}
		//3.2  省略K时，按K=J退尾；
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG34ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
			pSG34ParaCheck[ucChannelNum]->dK = pSG34ParaCheck[ucChannelNum]->dJ;
		}
		//3.3  J和K都不省略 
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == TRUE)
		{
			//3.3.1 J=0或J=0、K=0时，无退尾； J=0，K≠0时，无退尾；-》   即J=0时无退尾
			if (isZero(pSScanContainer->dRIJK[2]) == TRUE)
			{
				pSG34ParaCheck[ucChannelNum]->dJ = 0;
				pSG34ParaCheck[ucChannelNum]->dK = 0;
			}
			//3.3.2 J≠0，K=0时，按J=K退尾；
			else if (isZero(pSScanContainer->dRIJK[2]) == FALSE && isZero(pSScanContainer->dRIJK[3]) == TRUE)
			{

				pSG34ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
				pSG34ParaCheck[ucChannelNum]->dJ = pSG34ParaCheck[ucChannelNum]->dK;
			}
			else
			{
				pSG34ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG34ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
			}

		}

		//4.退尾预处理标志位初始化
		pSG34ParaCheck[ucChannelNum]->bLeftTailFlag = FALSE;

		//5.同一个G32标志位初始化
		pSG34ParaCheck[ucChannelNum]->bSameG32Flag = FALSE;

		//6.初始化跨越退尾点的螺纹段被拆分成两段后的后一段的终点
		pSG34ParaCheck[ucChannelNum]->dLongAxisTmpPos = 0;

		//7.初始化G34指令有效运行标志位
		pSG34ParaCheck[ucChannelNum]->bG34WorkFlag = FALSE;

		//8.初始化G32次序
		pSG34ParaCheck[ucChannelNum]->nG32Order = 0;


	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG34ParaAssign[ucChannelNum]->cStep = 0;

		//1.获取R
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG34ParaAssign[ucChannelNum]->dR = pSScanContainer->dRIJK[0];
		}
		//2.获取导程F  (F与I互斥，先获取导程值（这里为值覆盖），后面再判断互斥性)
		if (pSScanContainer->bF == TRUE)
		{
			pSG34ParaAssign[ucChannelNum]->dF = pSScanContainer->dF;
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			pSG34ParaAssign[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
		}

		//3.获取长轴退尾量、短轴退尾量
		//3.根据手册要求获取短轴退尾量，长轴退尾量
		//3.1 G32省略退尾时 ：省略J或J、K时，无退尾；
		if (pSScanContainer->bRIJK[2] == FALSE)
		{
			//J、K是模态代码，上一个程序段为G34，当前程序段的G34省略J、K时，按前面的J、K值进行退尾，
			if (SGCodeCurrSysState->SModalInfo.sGa == G34 * pow(10, GCOMMAND_DECIMAL_PLACE))
			{

			}
			//省略J或J、K时且不是连续螺纹的下一段，无退尾；
			else
			{
				pSG34ParaAssign[ucChannelNum]->dJ = 0;
				pSG34ParaAssign[ucChannelNum]->dK = 0;
			}
		}
		//3.2  省略K时，按K=J退尾；
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG34ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
			pSG34ParaAssign[ucChannelNum]->dK = pSG34ParaAssign[ucChannelNum]->dJ;
		}
		//3.3  J和K都不省略 
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == TRUE)
		{
			//3.3.1 J=0或J=0、K=0时，无退尾； J=0，K≠0时，无退尾；-》   即J=0时无退尾
			if (isZero(pSScanContainer->dRIJK[2]) == TRUE)
			{
				pSG34ParaAssign[ucChannelNum]->dJ = 0;
				pSG34ParaAssign[ucChannelNum]->dK = 0;
			}
			//3.3.2 J≠0，K=0时，按J=K退尾；
			else if (isZero(pSScanContainer->dRIJK[2]) == FALSE && isZero(pSScanContainer->dRIJK[3]) == TRUE)
			{

				pSG34ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
				pSG34ParaAssign[ucChannelNum]->dJ = pSG34ParaAssign[ucChannelNum]->dK;
			}
			else
			{
				pSG34ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG34ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
			}

		}

		//4.退尾预处理标志位初始化
		pSG34ParaAssign[ucChannelNum]->bLeftTailFlag = FALSE;

		//5.同一个G32标志位初始化
		pSG34ParaAssign[ucChannelNum]->bSameG32Flag = FALSE;

		//6.初始化跨越退尾点的螺纹段被拆分成两段后的后一段的终点
		pSG34ParaAssign[ucChannelNum]->dLongAxisTmpPos = 0;

		//7.初始化G34指令有效运行标志位
		pSG34ParaAssign[ucChannelNum]->bG34WorkFlag = FALSE;

		//8.初始化G32次序
		pSG34ParaAssign[ucChannelNum]->nG32Order = 0;

	}

}

//检查G34
int8 CheckG34(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSSyntaxConfig == NULL)
	{
		return -1;
	}
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		SG34Para* pSG34Para = pSG34ParaCheck[pSSyntaxConfig->ucChannelNum];
		uint8* ucLongAxis = &pSG34Para->ucLongAxis;        //长轴轴号
		uint8* ucShortAxis = &pSG34Para->ucShortAxis;       //短轴轴号
		double* dLongAxisVector = &pSG34Para->dLongAxisVector;      //长轴的向量
		double* dLSAxisRatio = &pSG34Para->dLSAxisRatio; //长轴与短轴的比例
		double dHorAxisLen, dVerAxisLen;	 //横轴长度、纵轴长度（带方向）
		double dLongAxisLen, dShortAxisLen;  //长轴长度、短轴长度（带方向）

		//G34检查第一步
		//1.检查参数关键字和参数范围
		//2.检查F和I的互斥性
		//3.获取G34的起点与终点
		//4.获取G34长轴和短轴
		//5. 获取G34长轴向量
		//6.更新步骤
		switch (pSG34Para->cStep)
		{
		case 0:
		{
			//1.指令字符检查
			checkParaLetter("XYZUVWFIJKR", &pSSyntaxConfig->SScanContainer);

			//2.参数值范围检查
			checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
			checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
			checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
			checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
			checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
			checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
			checkParaRange('F', &pSSyntaxConfig->SScanContainer, 0.001, 500);
			checkParaRange('I', &pSSyntaxConfig->SScanContainer, 0.06, 25400);
			checkParaRange('J', &pSSyntaxConfig->SScanContainer, -99999999 * TEMP_LEAST_INPUT_INCREMENT, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
			checkParaRange('K', &pSSyntaxConfig->SScanContainer, -99999999 * TEMP_LEAST_INPUT_INCREMENT, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
			if (pSSyntaxConfig->SScanContainer.bF == TRUE)
			{
				//checkParaRange('R', &pSSyntaxConfig->SScanContainer, -0.001, 500.000);
				double dR = pSSyntaxConfig->SScanContainer.dRIJK[0];
				checkParaRange('R', &pSSyntaxConfig->SScanContainer,dR < 0 ? -500.000 : 0.001,dR < 0 ? -0.001 : 500.000);
			}
			else if (pSSyntaxConfig->SScanContainer.bRIJK[1] == TRUE)
			{
				//checkParaRange('R', &pSSyntaxConfig->SScanContainer, -0.06, 25400);
				double dR = pSSyntaxConfig->SScanContainer.dRIJK[0];
                checkParaRange('R',&pSSyntaxConfig->SScanContainer,dR < 0 ? -25400.000 : 0.060,dR < 0 ? -0.060 : 25400.000);
			}



			//3.F与I不能同时出现
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

			int8 cRet = 0;
			uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
			//1.1初始化起点与终点
			pSG34Para->SStartPoint = pSG34Para->SEndPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
			bool bIsAbsMethod;
			//获取终点
			bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
			getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &pSG34Para->SStartPoint, &pSG34Para->SEndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
			//获取横轴、纵轴
			cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pSSyntaxConfig->SCurrRecord.SModalInfo.sGc);
			if (cRet < 0)
			{
				return -1;
			}

			//5.获取长短轴
			//2.2 计算横轴长度与纵轴长度（带方向）
			dHorAxisLen = pSG34Para->SEndPoint.dArray[ucHorAxis] - pSG34Para->SStartPoint.dArray[ucHorAxis];
			dVerAxisLen = pSG34Para->SEndPoint.dArray[ucVerAxis] - pSG34Para->SStartPoint.dArray[ucVerAxis];
			//2.3 根据横轴长度与纵轴长度获取长轴轴号与短轴轴号
			if (fabs(dHorAxisLen) > fabs(dVerAxisLen))
			{
				*ucLongAxis = ucHorAxis;//长轴轴号
				*ucShortAxis = ucVerAxis;//短轴轴号
				dLongAxisLen = dHorAxisLen;//长轴长度
				dShortAxisLen = dVerAxisLen;//短轴长度
			}
			else
			{
				*ucLongAxis = ucVerAxis;
				*ucShortAxis = ucHorAxis;
				dLongAxisLen = dVerAxisLen;
				dShortAxisLen = dHorAxisLen;
			}

			//3.获取长轴向量
			*dLongAxisVector = sgn(dLongAxisLen);

			//4.获取退尾前的短轴与长轴的比例（带短轴的方向矢量）
			*dLSAxisRatio = dShortAxisLen / fabs(dLongAxisLen);

			//5.计算长轴方向的理想退尾位置
			if (isZero(pSG34Para->dJ) == FALSE)
			{
				pSG34Para->dTailIdealPos = pSG34Para->SEndPoint.dArray[*ucLongAxis] - pSG34Para->dK * (*dLongAxisVector);
			}

			//6.判断是否执行G34（长轴方向的总位移量不为0）
			if (isZero(dLongAxisLen) == FALSE)
			{
				pSG34Para->bG34WorkFlag = TRUE;
			}
			//7.更新终点
			pGCodeBufUnit->SEndPoint = pSG34Para->SEndPoint;

			//8.步骤
			//G34可执行则继续执行下一个步骤
			if (pSG34Para->bG34WorkFlag == TRUE)
			{
				pSG34Para->cStep = 1;
				return 1;
			}
			//G34不可执行则跳过指令
			else
			{
				return 2;
			}
		}

		//G34检查第二步
		//1.获取当前螺纹段的导程值
		//2.如果当前螺纹段导程小于0以及转速为0，报错
		//3.根据加工参数的来源检查长轴、短轴以当前的最大速度能否追上主轴
		//4.更新终点

		case 1:
		{
			//相关变量初始化
			double dCurrF = 0;           //当前G32的导程值
			SPointDouble SEndPoint = { 0 };  //当前螺纹段的理论终点坐标
			bool bFinshFlag = FALSE; //G34完成标志位
			double dLongAxisDelta = 0;   //长轴的位移增量(绝对值)
			double dLSAxisRatioForTail = 0;  //退尾后长短轴的比例
			double dDeltaForEndPos = 0;      //当前位置距离终点的距离
			int8 cRet = 0;
			uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴


			//1.计算当前螺纹段的导程
			dCurrF = pSG34Para->dF + pSG34Para->dR * pSG34Para->nG32Order; //（等差数列求项）
			//初始化长轴的位移增量                 
			dLongAxisDelta = dCurrF;
			//如果当前螺纹段导程小于0以及转速为0，报错
			if (!isfinite(dCurrF) || dCurrF <= 0 ||!isfinite(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed) ||pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed <= 0)
			//if (!(pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed > 0 && pSSyntaxConfig->SCurrRecord.dFeedrate > 0))
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM405);
				return -1;
			}

			//2.计算当前螺纹段在长轴方向的理论终点
			//2.1 当前螺纹段不是跨越退尾点的螺纹段被拆分成两段后的后一段
			if (!pSG34Para->bSameG32Flag)
			{
				SEndPoint.dArray[*ucLongAxis] = pSG34Para->SStartPoint.dArray[*ucLongAxis] + (*dLongAxisVector) * dCurrF;
			}
			//2.2当前螺纹段是跨越退尾点的螺纹段被拆分成两段后的后一段
			else
			{
				SEndPoint.dArray[*ucLongAxis] = pSG34Para->dLongAxisTmpPos;
				//重算长轴的位移增量
				dLongAxisDelta = fabs(SEndPoint.dArray[*ucLongAxis] - pSG34Para->SStartPoint.dArray[*ucLongAxis]);
				pSG34Para->bSameG32Flag = FALSE;
			}

			//3.如果G34有退尾但当前的螺纹拆分还没到达退尾点处
			if (!isZero(pSG34Para->dJ) && !pSG34Para->bLeftTailFlag)
			{
				//3.1 计算当前螺纹段的理论终点与理想退尾点在长轴方向的距离(带方向)
				double dDelta = (SEndPoint.dArray[*ucLongAxis] - pSG34Para->dTailIdealPos) * (*dLongAxisVector);
				//3.2 如果当前螺纹段的理论终点在长轴方向上超过或抵达退尾点处
				if (dDelta > 0 || isZero(dDelta))
				{
					//3.2.1 如果当前螺纹段在长轴方向上超过了退尾点
					if (!isZero(dDelta))
					{
						pSG34Para->bSameG32Flag = TRUE;
					}
					//3.2.1 记录跨当前螺纹段的理论终点，作为下一个螺纹段的起点
					pSG34Para->dLongAxisTmpPos = SEndPoint.dArray[*ucLongAxis];
					//3.2.2 重算退尾后G34的终点（短轴方向）
					pSG34Para->SEndPoint.dArray[*ucShortAxis] += pSG34Para->dJ;
					//3.2.3 重算当前螺纹段的理论终点（让当前螺纹段的终点为退尾点）
					SEndPoint.dArray[*ucLongAxis] = pSG34Para->dTailIdealPos;
					//3.2.4 重算当前螺纹段在长轴的位移增量
					dLongAxisDelta = fabs(SEndPoint.dArray[*ucLongAxis] - pSG34Para->SStartPoint.dArray[*ucLongAxis]);
					//3.2.5 重算退尾后的长短轴的比例
					// 计算长轴方向的剩余切削量	(绝对值)	
					double dCutLeftDisLongAxis = fabs(pSG34Para->SEndPoint.dArray[*ucLongAxis] - SEndPoint.dArray[*ucLongAxis]);
					//计算短轴方向的剩余切削量(带方向)
					double dCutLeftDisShortAxis = pSG34Para->SEndPoint.dArray[*ucShortAxis] - (pSG34Para->SStartPoint.dArray[*ucShortAxis] + dLongAxisDelta * (*dLSAxisRatio));
					//计算长短轴比例
					dLSAxisRatioForTail = dCutLeftDisShortAxis / dCutLeftDisLongAxis;
					//3.2.6 G32次序-1 （后边的流程会使G32运行次序+1，因为退尾所增加的G32不会改变导程，所以先减后加使退尾次序不变）
					pSG34Para->nG32Order--;
					//3.2.7 退尾已处理标志位置为true
					pSG34Para->bLeftTailFlag = TRUE;
				}
			}

			//如果是最后一段螺纹
			dDeltaForEndPos = (SEndPoint.dArray[*ucLongAxis] - pSG34Para->SEndPoint.dArray[*ucLongAxis]) * (*dLongAxisVector);
			if (dDeltaForEndPos > 0 || isZero(dDeltaForEndPos))
			{
				//重新赋值当前G32的理论终点
				SEndPoint.dArray[*ucLongAxis] = pSG34Para->SEndPoint.dArray[*ucLongAxis];
				//重新计算长轴的位移增量
				dLongAxisDelta = fabs(SEndPoint.dArray[*ucLongAxis] - pSG34Para->SStartPoint.dArray[*ucLongAxis]);
				bFinshFlag = TRUE;
			}

			//4.根据长短轴的比例以及长轴的位移增量计算短轴方向的理论终点
			pSG34Para->SEndPoint.dArray[*ucShortAxis] = pSG34Para->SStartPoint.dArray[*ucShortAxis] + dLongAxisDelta * (*dLSAxisRatio);
			SEndPoint.dArray[*ucShortAxis] = pSG34Para->SEndPoint.dArray[*ucShortAxis];

			//5.更新终点,起点
			pGCodeBufUnit->SEndPoint = SEndPoint;
			pGCodeBufUnit->SStartPoint = pSG34Para->SStartPoint;
			//根据加工参数的来源检查长轴、短轴以当前的最大速度能否追上主轴
			cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);   //获取横轴，纵轴
			if (cRet < 0)
			{
				return;
			}
			checkG32Speed(pSSyntaxConfig, dCurrF, pGCodeBufUnit->SStartPoint, pGCodeBufUnit->SEndPoint, ucHorAxis, ucVerAxis);

			//6.如果进行了退尾处理（退尾后的长短轴比例不为0），更新G34的长短轴比例，用于退尾后的螺纹段
			if (!isZero(dLSAxisRatioForTail))
			{
				(*dLSAxisRatio) = dLSAxisRatioForTail;
			}
			//7.更新G34参数结构体
			pSG34Para->nG32Order++;// G32运行次序加1
			pSG34Para->SStartPoint = SEndPoint;//更新G34的起点（下一次G32的起点）

			//8.判断G34是否拆分完毕
			if (bFinshFlag)
			{
				pSG34Para->cStep = 0;
				return 2;
			}
			else
			{
				return 1;
			}

		}
		default:

			break;


		}

	}
	return 2;

}









//赋值G34
int8 AssignG34(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SG34Para* pSG34Para = pSG34ParaAssign[pSAssignConfig->ucChannelNum];
	uint8* ucLongAxis = &pSG34Para->ucLongAxis;        //长轴轴号
	uint8* ucShortAxis = &pSG34Para->ucShortAxis;       //短轴轴号
	double* dLongAxisVector = &pSG34Para->dLongAxisVector;      //长轴的向量
	double* dLSAxisRatio = &pSG34Para->dLSAxisRatio; //长轴与短轴的比例



	switch (pSG34Para->cStep)
	{
		//G34预处理
		//1.机械G34终点坐标
		//2.确定长轴 短轴
		//3.计算长轴向量
		//4.确定长短轴比例（退尾前）
		//5.确定是否执行G34
	case 0:
	{
		uint8 ucHorAxis, ucVerAxis;			 //横轴，纵轴
		double dHorAxisLen, dVerAxisLen;	 //横轴长度、纵轴长度（带方向）
		double dLongAxisLen, dShortAxisLen;  //长轴长度、短轴长度（带方向）

		//1.获取G34终点机械坐标
		//1.1初始化起点与终点
		pSG34Para->SStartPoint = pSG34Para->SEndPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
		//1.2 计算终点的机械坐标
		// 获取当前的坐标编程方式
		bool bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
		getMacCoordByScanInfo(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &pSG34Para->SStartPoint, &pSG34Para->SEndPoint, &pSAssignConfig->SGCodeCurrSysState);

		//2.获取长轴轴号以及短轴轴号
		//2.1  获取横轴和纵轴
		int8 cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSAssignConfig->SGCodeCurrSysState.cInterpolationState, pGCodeBufUnit->SModalInfo.sGc);
		if (cRet < 0)
		{
			return -1;
		}
		//2.2 计算横轴长度与纵轴长度（带方向）
		dHorAxisLen = pSG34Para->SEndPoint.dArray[ucHorAxis] - pSG34Para->SStartPoint.dArray[ucHorAxis];
		dVerAxisLen = pSG34Para->SEndPoint.dArray[ucVerAxis] - pSG34Para->SStartPoint.dArray[ucVerAxis];
		//2.3 根据横轴长度与纵轴长度获取长轴轴号与短轴轴号
		if (fabs(dHorAxisLen) > fabs(dVerAxisLen))
		{
			*ucLongAxis = ucHorAxis;//长轴轴号
			*ucShortAxis = ucVerAxis;//短轴轴号
			dLongAxisLen = dHorAxisLen;//长轴长度
			dShortAxisLen = dVerAxisLen;//短轴长度
		}
		else
		{
			*ucLongAxis = ucVerAxis;
			*ucShortAxis = ucHorAxis;
			dLongAxisLen = dVerAxisLen;
			dShortAxisLen = dHorAxisLen;
		}

		//3.获取长轴向量
		*dLongAxisVector = sgn(dLongAxisLen);
		if (isZero(dLongAxisLen) == TRUE)
		{
			return 2;
		}

		//4.获取退尾前的短轴与长轴的比例（带短轴的方向矢量）
		*dLSAxisRatio = dShortAxisLen / fabs(dLongAxisLen);

		//5.计算长轴方向的理想退尾位置
		if (isZero(pSG34Para->dJ) == FALSE)
		{
			pSG34Para->dTailIdealPos = pSG34Para->SEndPoint.dArray[*ucLongAxis] - pSG34Para->dK * (*dLongAxisVector);
		}

		//6.判断是否执行G34（长轴方向的总位移量不为0）
		if (isZero(dLongAxisLen) == FALSE)
		{
			pSG34Para->bG34WorkFlag = TRUE;
		}

		//G34特殊处理（shit）
		{
			pGCodeBufUnit->SEndPoint = pSG34Para->SEndPoint;
			pGCodeBufUnit->SStartPoint = pSG34Para->SStartPoint;
		}


		//7.步骤
		//G34可执行则继续执行下一个步骤
		if (pSG34Para->bG34WorkFlag == TRUE)
		{
			pSG34Para->cStep = 1;
			return 1;
		}
		//G34不可执行则跳过指令
		else
		{
			return 2;
		}


	}

	//将G34拆分成多个G32
	case 1:
	{
		//相关变量初始化
		double dCurrF = 0;           //当前G32的导程值
		SPointDouble SEndPoint = { 0 };  //当前螺纹段的理论终点坐标
		bool bFinshFlag = FALSE; //G34完成标志位
		double dLongAxisDelta = 0;   //长轴的位移增量(绝对值)
		double dLSAxisRatioForTail = 0;  //退尾后长短轴的比例
		double dDeltaForEndPos = 0;      //当前位置距离终点的距离

		//1.计算当前螺纹段的导程
		dCurrF = pSG34Para->dF + pSG34Para->dR * pSG34Para->nG32Order; //（等差数列求项）
		//初始化长轴的位移增量
		dLongAxisDelta = dCurrF;

		//2.计算当前螺纹段在长轴方向的理论终点
		//2.1 当前螺纹段不是跨越退尾点的螺纹段被拆分成两段后的后一段
		if (!pSG34Para->bSameG32Flag)
		{
			SEndPoint.dArray[*ucLongAxis] = pSG34Para->SStartPoint.dArray[*ucLongAxis] + (*dLongAxisVector) * dCurrF;
		}
		//2.2当前螺纹段是跨越退尾点的螺纹段被拆分成两段后的后一段
		else
		{
			SEndPoint.dArray[*ucLongAxis] = pSG34Para->dLongAxisTmpPos;
			//重算长轴的位移增量
			dLongAxisDelta = fabs(SEndPoint.dArray[*ucLongAxis] - pSG34Para->SStartPoint.dArray[*ucLongAxis]);
			pSG34Para->bSameG32Flag = FALSE;
		}

		if (pSG34Para->bLeftTailFlag == TRUE)
		{
			//当前螺纹段为G34的退尾段
			pGCodeBufUnit->bG34BackTailFlag = TRUE;
		}

		//3.如果G34有退尾但当前的螺纹拆分还没到达退尾点处
		if (!isZero(pSG34Para->dJ) && !pSG34Para->bLeftTailFlag)
		{
			//3.1 计算当前螺纹段的理论终点与理想退尾点在长轴方向的距离(带方向)
			double dDelta = (SEndPoint.dArray[*ucLongAxis] - pSG34Para->dTailIdealPos) * (*dLongAxisVector);
			//3.2 如果当前螺纹段的理论终点在长轴方向上超过或抵达退尾点处
			if (dDelta > 0 || isZero(dDelta))
			{
				//3.2.1 如果当前螺纹段在长轴方向上超过了退尾点
				if (!isZero(dDelta))
				{
					pSG34Para->bSameG32Flag = TRUE;
				}
				//3.2.1 记录跨当前螺纹段的理论终点，作为下一个螺纹段的起点
				pSG34Para->dLongAxisTmpPos = SEndPoint.dArray[*ucLongAxis];
				//3.2.2 重算退尾后G34的终点（短轴方向）
				pSG34Para->SEndPoint.dArray[*ucShortAxis] += pSG34Para->dJ;
				//3.2.3 重算当前螺纹段的理论终点（让当前螺纹段的终点为退尾点）
				SEndPoint.dArray[*ucLongAxis] = pSG34Para->dTailIdealPos;
				//3.2.4 重算当前螺纹段在长轴的位移增量
				dLongAxisDelta = fabs(SEndPoint.dArray[*ucLongAxis] - pSG34Para->SStartPoint.dArray[*ucLongAxis]);
				//3.2.5 重算退尾后的长短轴的比例
				// 计算长轴方向的剩余切削量	(绝对值)	
				double dCutLeftDisLongAxis = fabs(pSG34Para->SEndPoint.dArray[*ucLongAxis] - SEndPoint.dArray[*ucLongAxis]);
				//计算短轴方向的剩余切削量(带方向)
				double dCutLeftDisShortAxis = pSG34Para->SEndPoint.dArray[*ucShortAxis] - (pSG34Para->SStartPoint.dArray[*ucShortAxis] + dLongAxisDelta * (*dLSAxisRatio));
				//计算长短轴比例
				dLSAxisRatioForTail = dCutLeftDisShortAxis / dCutLeftDisLongAxis;
				//3.2.6 G32次序-1 （后边的流程会使G32运行次序+1，因为退尾所增加的G32不会改变导程，所以先减后加使退尾次序不变）
				pSG34Para->nG32Order--;
				//3.2.7 退尾已处理标志位置为true
				pSG34Para->bLeftTailFlag = TRUE;
			}


		}


		//如果是最后一段螺纹
		dDeltaForEndPos = (SEndPoint.dArray[*ucLongAxis] - pSG34Para->SEndPoint.dArray[*ucLongAxis]) * (*dLongAxisVector);
		if (dDeltaForEndPos > 0 || isZero(dDeltaForEndPos))
		{
			//重新赋值当前G32的理论终点
			SEndPoint.dArray[*ucLongAxis] = pSG34Para->SEndPoint.dArray[*ucLongAxis];
			//重新计算长轴的位移增量
			dLongAxisDelta = fabs(SEndPoint.dArray[*ucLongAxis] - pSG34Para->SStartPoint.dArray[*ucLongAxis]);
			bFinshFlag = TRUE;
		}

		//4.根据长短轴的比例以及长轴的位移增量计算短轴方向的理论终点
		pSG34Para->SEndPoint.dArray[*ucShortAxis] = pSG34Para->SStartPoint.dArray[*ucShortAxis] + dLongAxisDelta * (*dLSAxisRatio);
		SEndPoint.dArray[*ucShortAxis] = pSG34Para->SEndPoint.dArray[*ucShortAxis];
		//5.如果进行了退尾处理（退尾后的长短轴比例不为0），更新G34的长短轴比例，用于退尾后的螺纹段
		if (!isZero(dLSAxisRatioForTail))
		{
			(*dLSAxisRatio) = dLSAxisRatioForTail;
		}

		//6.填写Thread结构体并赋值到缓冲区
		SThread SThreadInfo = { 0 };
		//6.1 填写螺纹段的起点、终点
		SThreadInfo.SStartPoint = pSG34Para->SStartPoint;
		SThreadInfo.SEndPoint = SEndPoint;
		SThreadInfo.SEndPoint.dArray[AXIS_C] = SThreadInfo.SStartPoint.dArray[AXIS_C];
		//6.2 填写导程
		SThreadInfo.dF = dCurrF;
		//6.3 填写连续G32标志
		if (!isZero(pSG34Para->nG32Order))
		{
			SThreadInfo.bRepeatG32Flag = TRUE;
		}
		//6.4 填写实体类型
		SThreadInfo.EntityType = THREAD;
		//将SThread写入缓冲区
		writeThreadIntoGCodeBuf(&SThreadInfo, pGCodeBufUnit);

		//7.赋值其他
		if (!isZero(pSG34Para->nG32Order))
		{
			//G34运行标志位，用于G34从终点反算何时减速
			pGCodeBufUnit->bG34RunFlag = TRUE;
		}

		
		//8.更新G34参数结构体
		pSG34Para->nG32Order++;// G32运行次序加1
		pSG34Para->SStartPoint = SEndPoint;//更新G34的起点（下一次G32的起点）

		//9.判断G34是否拆分完毕
		if (bFinshFlag)
		{
			pSG34Para->cStep = 0;
			return 2;
		}
		else
		{
			return 1;
		}
	}
	default:
		break;
	}

}


