#include"ErrorFunction.h"
#include "GCode.h"
#include"NCDataPara.h"
#include"GeneralFunctions.h"
#include <math.h>

/*
指令名称：G76 复合型螺纹切削固定循环
指令格式：G76 P(m)(r)(a) Q(△dmin) R(d) J_ K_；
G76 X(U)_ Z(W)_ R(i) P(k)_ Q(△d)_ F(I)_ ；

相关定义：
（1）X：螺纹终点X 轴绝对坐标；
（2）U：螺纹终点与起点X 轴绝对坐标的差值；
（3）Z：螺纹终点Z 轴的绝对坐标值；
（4）W：螺纹终点与起点Z 轴绝对坐标的差值；
（5）P(m)：螺纹精车次数00～99 (单位：次) ，m 指定值执行后保持有效，未输入m 时，以系统数据参数043号（G76 循环精加工的重复次数(1～99)，默认为1）的值作为精车次数。
		   在螺纹精车时，每次的进给的切削量等于螺纹精车的切削量即第一次精车切削量为d ，其后的精车切削量为0，
（6）P(r)：螺纹退尾长度00～99(单位：0.1×L，L 为螺纹螺距)，r 指定值执行后保持有效，
	  未输入r 时，以系统参数所设的默认退尾值，作为退尾量。
	  若使用P（r）默认长轴退尾量=短轴退尾量=P（r）【若无j、K则退尾按照P（r）退尾，若编写了j、k退尾方式和G32、G76相同】
（7）P(a)：相邻两牙螺纹的夹角，取值范围为00～99，单位：度(°)，a 指定值执行后保持有效，未输入a 时，以系统数据参数044号（0）的值作为螺纹牙的角度。
（8）Q(△dmin)：螺纹粗车时的最小切削量，取值范围为0～99999 (单位：mm(原文是um，企业为了方便输入，统一为mm下同)，半径值)。
	 Q(△dmin)执行后，指定值△dmin 保持有效，未输入Q(△dmin) 时，以系统数据参数045号（0）的值作为最小切削量；
（9）R(d)：螺纹精车的切削量，取值范围为00～99.999，(单位：mm/inch，无符号，半径值)，半径值等于螺纹精车切入点Be 与最后一次螺纹粗车切入点Bf 的X 轴绝对坐标的差值。R(d) 执行后，指定值d 保持有效，未输入R(d)时，以系统数据参数046（0）的值作为螺纹精车切削量；
(10) R(i) ：螺纹锥度，螺纹起点与螺纹终点X 轴绝对坐标的差值, 取值范围为-99999.999～ 99999.999 (单位：mm/inch,半径值)。未输入R(i)时，系统按R(i)=0(直螺纹)处理；
（11）P(k)：螺纹牙高，螺纹总切削深度, 取值范围为1～99999999 (单位：mm，半径值、无符号)。未输入P(k)时，系统报警；
（12）Q(△d)：第一次螺纹切削深度, 取值范围为1～99999999 (单位：mm，半径值、无符号)。未输入△d 时，系统报警；
（13）F：螺纹导程； I：螺纹每英寸的螺纹牙数；
（14）J：螺纹退尾时在短轴方向的移动量(退尾量)(取值范围0～99999999（不可以为负数）×最小输入增量，单位mm/inch,不带方向)；如果短轴是X 轴，该值为半径指定,非模态参数；
（15）K：螺纹退尾时在长轴方向的长度(取值范围 0～99999999×最小输入增量，单位 mm/inch,不带 方向)；如果长轴是 X 轴，则该值为半径指定，非模态参数；
*/


static SG76Para* pSG76ParaCheck[CHANNEL_NUM_MAX];	//检查流程使用
static SG76Para* pSG76ParaAssign[CHANNEL_NUM_MAX];	//赋值流程使用



//添加G76规则结构体
void addG76RegulationAPI()
{
	//指令：G76
	//组别：sGa
	//指令占程序段数：2
	addGCommadRegulation(76, sGz, 2, mallocG76ParaStruct, freeG76ParaStruct, getG76Para, CheckG76, AssignG76);
}

/*************************************************
  函数名称：int8 mallocG76ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G76参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG76ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG76ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG76Para), ORAM_NUM428);//暂未对内存编号
		if (pSG76ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG76ParaCheck[ucChannelNum], 0, sizeof(SG76Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG76ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG76Para), ORAM_NUM429);
		if (pSG76ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG76ParaAssign[ucChannelNum], 0, sizeof(SG76Para));
		return 1;
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG76ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG76ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG76ParaCheck[ucChannelNum]);
			pSG76ParaCheck[ucChannelNum] = NULL;
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG76ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG76ParaAssign[ucChannelNum]);
			pSG76ParaAssign[ucChannelNum] = NULL;
		}
		return 1;
	}
}


/*************************************************
  函数名称：int8 getG76Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能：获取G76参数
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
			SScan* pSScanContainer  扫描结构体
			SGCodeSysState* SGCodeCurrSysState  通道状态结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG76Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* pSGCodeCurrSysState)
{
	SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);//获取通道结构体
	if (isErrorHasBeenWritten())
		return -1;
	if (ucMode == GC_SYNTAX)
	{
		pSG76ParaCheck[ucChannelNum]->cStep = 0;
		struct _SSyntaxConfiguration* pSSyntaxConfig = &pChannelConfig->SSyntaxConfig;       //获取赋值结构体
		if (pSSyntaxConfig->cCommandSegmentCnt == 1)  //G76 P(m)(r)(a) Q(△dmin) R(d) J_ K_；
		{
			//1.指令字符检查
			checkParaLetter("PQRJK", &pSSyntaxConfig->SScanContainer);

			//2.参数值范围检查
			checkParaRange('Q', pSScanContainer, 0, 99999);
			checkParaRange('R', pSScanContainer, 0, 99.999);
			checkParaRange('J', pSScanContainer, 0, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
			checkParaRange('K', pSScanContainer, 0, 99999999 * TEMP_LEAST_INPUT_INCREMENT);

			// 第一段 P 是复合整数编码，单独校验；第二段 P 是允许小数的牙高。 
			if (pSScanContainer->bP)
			{
				double dP = pSScanContainer->dP;

				if (!isfinite(dP) || dP <= 0.0 || dP > 999999.0 ||
					dP != floor(dP))
				{
					parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
					return -1;
				}

				
			}
			return 1;
		}
		else if (pSSyntaxConfig->cCommandSegmentCnt == 2) // 第二行 G76 X(U)_ Z(W)_ R(i) P(k)_ Q(△d)_ F(I)_ ；
		{
			//1.指令字符检查
			checkParaLetter("XZUWRPQFI", &pSSyntaxConfig->SScanContainer);

			//2.参数值范围检查
			checkParaRange('X', pSScanContainer, -99999999, 99999999);
			checkParaRange('Z', pSScanContainer, -99999999, 99999999);
			checkParaRange('U', pSScanContainer, -99999999, 99999999);
			checkParaRange('W', pSScanContainer, -99999999, 99999999);
			checkParaRange('R', pSScanContainer, -99999.999,99999.999);
			checkParaRange('P', pSScanContainer, 0, 99999999);
			checkParaRange('Q', pSScanContainer, 0, 99999999);
			checkParaRange('F', pSScanContainer, 0.001, 500);
			checkParaRange('I', pSScanContainer, 0.06, 25400);

			//3.检查I和F的互斥性
			if (pSScanContainer->bF == TRUE && pSScanContainer->bRIJK[1] == TRUE)
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
			//4.检查是否出现P
			if (pSScanContainer->bP==FALSE)
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM112);
				return -1;
			}

			//5检测是否出现Q
			//if (pSScanContainer->bQ == FALSE)
			if (pSScanContainer->bQ == FALSE || !isfinite(pSScanContainer->dQ) || pSScanContainer->dQ <= 0.0)
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM113);
				return -1;
			}
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{

		SAssignConfiguration* pSAssignConfig = &pChannelConfig->SAssignConfig;       //获取赋值结构体
		SG76Para* pSG76Para = pSG76ParaAssign[pSAssignConfig->ucChannelNum];



		//1.1赋值时，获取第一程序段的参数
		if (pSAssignConfig->cCommandSegmentCnt == 1)
		{
			//1.1.1.获取参数 P(m)(r)(a)
			//根据P(m)(r)(a)值的简写规则：m、r、a用同一个代码地址P一次输入，m、r、a全部省略时，按数据参数043、026、044号设定值运行；
			//地址P输入1位或2位数时取值为a；地址P输入3位或4位数时取值为r与a；
			{
				//出现P
				if (pSScanContainer->bP == TRUE)
				{
					//1.P值输入了1位或2位数
					if (0 < pSScanContainer->dP && pSScanContainer->dP < 100)
					{
						//获取P(a)
						pSG76Para->ucPa = pSScanContainer->dP;

						//获取P（r）：设置P（r）出现标志位为FALSE
						pSG76Para->bCalculateBackTailByPr = FALSE;  //没出现P(r)

						//获取P(m):设置P(m)为默认值
						pSG76Para->ucPm = NC_DATA_PARAMETER_043;
					}
					//2.P值输入了3位或4位数
					else if (100 <= pSScanContainer->dP && pSScanContainer->dP < 10000)
					{
						//获取P(a)
						pSG76Para->ucPa = (int32)pSScanContainer->dP % 100;

						//获取P（r）：设置P（r）出现标志位为TRUE
						pSG76Para->ucPr = (int32)pSScanContainer->dP / 100;  //获取P(r)
						pSG76Para->bCalculateBackTailByPr = TRUE;

						//获取P(m):设置P(m)为默认值
						pSG76Para->ucPm = NC_DATA_PARAMETER_043;

					}
					//3.P值输入了5位或6位数
					else if (10000 <= pSScanContainer->dP && pSScanContainer->dP < 1000000)
					{
						//获取P(a)
						pSG76Para->ucPa = (int32)pSScanContainer->dP % 100;

						//获取P（r）：设置P（r）出现标志位为TRUE
						pSG76Para->ucPr = ((int32)pSScanContainer->dP / 100) % 100;  //获取P(r)
						pSG76Para->bCalculateBackTailByPr = TRUE;

						//获取P(m)
						pSG76Para->ucPm = (int32)pSScanContainer->dP / 10000;
					}


				}
				//没出现P
				else
				{
					//获取P(a): 设置P(a)为默认值
					pSG76Para->ucPa = NC_DATA_PARAMETER_044;

					//获取P（r）：设置P（r）出现标志位为FALSE
					pSG76Para->bCalculateBackTailByPr = FALSE;  //没出现P(r)

					//获取P(m):设置P(m)为默认值
					pSG76Para->ucPm = NC_DATA_PARAMETER_043;
				}
			}
			//1.1.2 获取参数Q(△dmin)
			{


				//出现Q
				if (pSScanContainer->bQ == TRUE)
				{
					pSG76Para->dQdmin = pSScanContainer->dQ;
					if (g_SSystemPara.bSetG7456mmTOum == TRUE)
					{
						pSG76Para->dQdmin *= 0.001;
					}
				}
				//没出现Q
				else
				{
					//设置为默认值
					pSG76Para->dQdmin = NC_DATA_PARAMETER_045;
				}

				//1.1.3 获取参数R（d）
				//出现R
				if (pSScanContainer->bRIJK[0] == TRUE)
				{
					pSG76Para->dRd = pSScanContainer->dRIJK[0];

				}
				//没出现R
				else
				{
					//设置R为默认值
					pSG76Para->dRd = NC_DATA_PARAMETER_046;
				}
			}
			//1.1.3 计算退尾量 :关于G76的退尾量有3个来源 ：1.参数J K；2.参数P（r）；3.默认系统参数  优先级次序从高到低为 1.2.3
			//出现JK
			{
				if (pSScanContainer->bRIJK[2] == TRUE || pSScanContainer->bRIJK[3] == TRUE)
				{
					if (pSScanContainer->bRIJK[2] == TRUE)
					{
						pSG76Para->dJ = pSScanContainer->dRIJK[2];
					}
					if (pSScanContainer->bRIJK[3] == TRUE)
					{
						pSG76Para->dK = pSScanContainer->dRIJK[3];
					}
					pSG76Para->bCalculateBackTailByJK = TRUE;
				}
			}


		}
		//X(U)_ Z(W)_ R(i) P(k)_ Q(△d)_ F(I)_ ；
		//1.2赋值时，获取第二程序段的参数
		else if (pSAssignConfig->cCommandSegmentCnt == 2)
		{
			//1.2.1 获取程序段的坐标信息

			getCoordInfoFromScanInfo(&pSG76Para->SCoordInfo, pSScanContainer);

			//1.2.2 获取R(i)
			if (pSScanContainer->bRIJK[0] == TRUE)
			{
				pSG76Para->dRi = pSScanContainer->dRIJK[0];
			}
			//1.2.3 获取P（k）
			if (pSScanContainer->bP == TRUE)
			{
				pSG76Para->dPk = pSScanContainer->dP;
			}
			//1.2.4获取Q(△d)
			if (pSScanContainer->bQ == TRUE)
			{
				pSG76Para->dQd = pSScanContainer->dQ;
			}
			//1.2.5 获取导程
			if (pSScanContainer->bF == TRUE)
			{
				pSG76Para->dF = pSScanContainer->dF;
			}
			else if (pSScanContainer->bRIJK[1] == TRUE)
			{
				pSG76Para->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
			}
			//1.2.6 根据导程以及JK、P（r）的出现情况计算退尾量
			// 计算退尾量 :关于G76的退尾量有3个来源 ：1.参数J K；2.参数P（r）；3.默认系统参数  优先级次序从高到低为 1.2.3
				//第一行参数中没有JK，但有P（r）
			if (pSG76Para->bCalculateBackTailByJK == FALSE && pSG76Para->bCalculateBackTailByPr == TRUE)
			{
				pSG76Para->dJ = pSG76Para->ucPr * 0.1 * pSG76Para->dF;
				pSG76Para->dK = pSG76Para->dJ;
			}
			//第一行参数中没有JK，也没有P(r)
			else if (pSG76Para->bCalculateBackTailByJK == FALSE && pSG76Para->bCalculateBackTailByPr == FALSE)
			{
				pSG76Para->dJ = g_SSystemPara.dJDefault;
				pSG76Para->dK = g_SSystemPara.dKDefault;
			}
			

			//1.2.7 初始化赋值步骤
			pSG76Para->cStep = 0;

		}
	}
	return 1;
}

int8 CheckG76(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	return 2;
	
}

int8 AssignG76(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
	if (pSAssignConfig == NULL || pSGCodeParseUnit == NULL || isErrorHasBeenWritten())
	{
		return -1;
	}
	SG76Para* pSG76Para = pSG76ParaAssign[pSAssignConfig->ucChannelNum];

	switch (pSG76Para->cStep)
	{
	//0.赋值预处理（计算轨迹所需的关键点以及关键参数）
	case 0:
	{
	// 宏运行中可能改变实际生成刀路前复核首刀切深。
	if (!isfinite(pSG76Para->dQd) || pSG76Para->dQd <= 0.0)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM113);
		return -1;
	}

		int8 cRet = 0;
		bool bIsAbsMethod;
		SPointDouble SPointC = { 0 };                                                //螺纹起点
		double dXLengthBeC;                                                        //Be点到C点X方向的长度
		double dXLengthBC;                                                         //B点到C点X方向的长度
		double dA = pSG76Para->ucPa / 180.0 * PI;                                  //相邻螺纹两牙之间的夹角(弧度形式)
		double dD = pSG76Para->dRd;                                               //螺纹精车的切削量
		double dRi = pSG76Para->dRi;                                               //获取螺纹锥度
		double dK = pSG76Para->dPk;                                                //获取螺纹牙高

		cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);

		//0.1程序段出现了坐标信息
		if (cRet == TRUE)
		{
			//1.获取A点
			pSG76Para->SPointA.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			pSG76Para->SPointA.dArray[AXIS_Z] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_Z];

			//2.计算D点
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			//计算D点的机械坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG76Para->SCoordInfo, &pSG76Para->SPointA, &pSG76Para->SPointD, &pSAssignConfig->SGCodeCurrSysState);

			//7 获取加工方向
			pSG76Para->cDir_X = sgn(pSG76Para->SPointD.dArray[AXIS_X] - pSG76Para->SPointA.dArray[AXIS_X]);
			pSG76Para->cDir_Z = sgn(pSG76Para->SPointD.dArray[AXIS_Z] - pSG76Para->SPointA.dArray[AXIS_Z]);



			//3.计算C点
			SPointC.dArray[AXIS_X] = pSG76Para->SPointD.dArray[AXIS_X] + pSG76Para->dRi;
			SPointC.dArray[AXIS_Z] = pSG76Para->SPointA.dArray[AXIS_Z];

			//4.计算B点
			//4.1计算Be点到C点X方向的长度
			dXLengthBeC = tan(dA / 2) * (dK - dD) * fabs(dRi)
			             / fabs(pSG76Para->SPointD.dArray[AXIS_Z] - pSG76Para->SPointA.dArray[AXIS_Z]);
			//4.2 计算B点到C点X方向的长度
			dXLengthBC= dXLengthBeC + dK;  
			//4.3 计算B点坐标
			pSG76Para->SPointB.dArray[AXIS_X] = SPointC.dArray[AXIS_X] - dXLengthBC*pSG76Para->cDir_X;
			pSG76Para->SPointB.dArray[AXIS_Z] = pSG76Para->SPointA.dArray[AXIS_Z];

			//5.计算螺纹斜率
			pSG76Para->dThreadCutSlope = (pSG76Para->SPointD.dArray[AXIS_Z] - SPointC.dArray[AXIS_Z]) /
				(pSG76Para->SPointD.dArray[AXIS_X] - SPointC.dArray[AXIS_X]);		

			//6.初始化精车标志位
			pSG76Para->bFineCutFlag = FALSE;

			//7.初始化当前切削次数
			pSG76Para->nCurCutTime = 0;

			//8.转步骤1
			pSG76Para->cStep = 1;

	    }
		//0.2程序段未出现坐标信息不赋值	
		else
		{
			return 2;
		}
		return 0;
		break;

	}
	//1.识别当前是粗车还是精车，计算Bn点
	case 1:
	{
		double dCutLengthX;  //X方向的切削深度
		double dCutLengthZ;  //Z方向的切削深度
		double dQd = pSG76Para->dQd;               //第一次切削深度
		double dQDmin = pSG76Para->dQdmin;         //螺纹粗车时的最小切削量
		double dA = pSG76Para->ucPa / 180.0 * PI;  //相邻螺纹两牙之间的夹角(弧度形式)
		double dK = pSG76Para->dPk;                 //获取螺纹牙高
		double dD = pSG76Para->dRd;                 //螺纹精车的切削量

		//切削次数+1
		pSG76Para->nCurCutTime++;
		//1.1判断接下来是粗车还是精车
		//粗车
		if (pSG76Para->bFineCutFlag==FALSE)
		{
			dCutLengthX = max(sqrt(pSG76Para->nCurCutTime) * dQd, sqrt(pSG76Para->nCurCutTime - 1) * dQd + dQDmin);  //获取本次X方向的切削深度
			//若切深将大于粗车的切深范围（本次粗车结束之后开始精车）
			if (dCutLengthX>(dK-dD))
			{
				//重算本次切削X方向的切削深度
				dCutLengthX = dK - dD;
				//重置切削次数用以记录精车次数
				pSG76Para->nCurCutTime = 0;	  	//下一次为精车	
			
				
			}
			//获取本次Z方向的切削深度	
			dCutLengthZ = dCutLengthX * tan(dA / 2);
		}
		//精车
		else
		{
			dCutLengthX = dK;  //牙高
			dCutLengthZ = (dK - dD) * tan(dA / 2);  //获取本次Z方向的切削深度
		}

		//1.2计算Bn点
		pSG76Para->SPointBn.dArray[AXIS_X] = pSG76Para->SPointB.dArray[AXIS_X] + pSG76Para->cDir_X * dCutLengthX;
		pSG76Para->SPointBn.dArray[AXIS_Z] = pSG76Para->SPointB.dArray[AXIS_Z] + pSG76Para->cDir_Z* dCutLengthZ;

		//.转步骤2
		pSG76Para->cStep = 2;
		return 0;
	}
	//2.A -> Bn G0
	case 2: 
	{
		SLine SLineInfo_1;
		memset(&SLineInfo_1, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSG76Para->SPointA;//A点
		SLineInfo_1.SEndPoint = pSG76Para->SPointBn;//Bn点
		//1.2获取SLine的实体类型
		SLineInfo_1.EntityType =isSamePoint(&SLineInfo_1.SStartPoint, &SLineInfo_1.SEndPoint) ? DOT : OTHERTYPE; //快速运动

		//2.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_1, pSGCodeParseUnit);

		pSG76Para->cStep = 3;
		return 1;
	}
	//3. Bn->Dn G32
	case 3:
	{
		//1.填写SThreadInfo
		SThread SThreadInfo;//存螺纹段信息（可能要将此变量放在赋值结构体中）
		double dVectorJ; //短轴方向的退尾向量
		memset(&SThreadInfo, 0, sizeof(SThread));

		//1.1初始化SThreadInfo的起点终点（机械坐标）
		SThreadInfo.SEndPoint = SThreadInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SThreadInfo的终点Dn（机械坐标）
		//粗车
		if (pSG76Para->bFineCutFlag == FALSE)
		{
			//带锥度
			if (!(isZero(pSG76Para->dThreadCutSlope)))
			{
				SThreadInfo.SEndPoint.dArray[AXIS_X] = (pSG76Para->SPointD.dArray[AXIS_Z] - pSG76Para->SPointBn.dArray[AXIS_Z]) / pSG76Para->dThreadCutSlope + pSG76Para->SPointBn.dArray[AXIS_X];
			}
			//直螺纹
			else
			{
				SThreadInfo.SEndPoint.dArray[AXIS_X] = pSAssignConfig->SGCodeCurrSysState.SCurrPoint.dArray[AXIS_X];
			}
		}
		//精车
		else
		{
			SThreadInfo.SEndPoint = pSG76Para->SPointD;
		}
	
		SThreadInfo.SEndPoint.dArray[AXIS_Z] = pSG76Para->SPointD.dArray[AXIS_Z];
		//1.3获取退尾量
		SThreadInfo.dJ = -pSG76Para->cDir_X * pSG76Para->dJ;
		SThreadInfo.dK = pSG76Para->dK;
		//1.4获取导程
		SThreadInfo.dF = pSG76Para->dF;
		//1.5实体类型
		SThreadInfo.EntityType= isSamePoint(&SThreadInfo.SStartPoint, &SThreadInfo.SEndPoint) ? DOT : THREAD; //螺纹切削
		//1.6将SThread写入缓冲区
		writeThreadIntoGCodeBuf(&SThreadInfo, pSGCodeParseUnit);

		pSG76Para->cStep = 4;


		//若成立，本次循环为粗车最后一个刀，下一个循环开始为精车
		if (!pSG76Para->bFineCutFlag && pSG76Para->nCurCutTime == 0)
		{
			pSG76Para->bFineCutFlag = TRUE;  //精车标志位置为真

		}

		return 1;
	}
	//4.Dn->E G0
	case 4:
	{
		SLine SLineInfo_2;
		memset(&SLineInfo_2, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_2.SEndPoint = SLineInfo_2.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//Dn点
		SLineInfo_2.SEndPoint.dArray[AXIS_X] = pSG76Para->SPointA.dArray[AXIS_X];//E点
			//1.2获取SLine的实体类型
		SLineInfo_2.EntityType = isSamePoint(&SLineInfo_2.SStartPoint, &SLineInfo_2.SEndPoint) ? DOT : OTHERTYPE; //快速运动

		//2.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_2, pSGCodeParseUnit);

		pSG76Para->cStep = 5;
		return 1;
	}
	//5.E->A
	case 5:
	{
		SLine SLineInfo_3;
		memset(&SLineInfo_3, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//Dn点
		SLineInfo_3.SEndPoint = pSG76Para->SPointA;//A点
			//1.2获取SLine的实体类型
		SLineInfo_3.EntityType = isSamePoint(&SLineInfo_3.SStartPoint, &SLineInfo_3.SEndPoint) ? DOT : OTHERTYPE; //快速运动

		//2.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_3, pSGCodeParseUnit);

		//如果当前为精车且精车次数达到要求,G76结束
		if (pSG76Para->bFineCutFlag==TRUE&&pSG76Para->nCurCutTime==pSG76Para->ucPm)
		{
			pSG76Para->cStep = 0;
			return 2;
		}
		else
		{
			pSG76Para->cStep = 1;
			return 1;
		}

	}
	default:
		break;

	}
}



