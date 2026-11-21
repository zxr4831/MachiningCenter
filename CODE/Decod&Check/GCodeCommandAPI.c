/*
说明：解析指令函数中，可通过调用此文件中的函数接口，以简便指令的扩展
*/


#include "ErrorFunction.h"
#include "GeneralFunctions.h"

#include "Gcode.h"
#include "ToolCompensation.h"
#include "ringbuff.h"
#include "Channel.h"
#include "list.h"
static SGetTextStateInfo* g_pSTextInfoCheck[CHANNEL_NUM_MAX];	//用于检查流程函数findTargetSegment()切片产生数据的存放
static SGetTextStateInfo* g_pSTextInfoAssign[CHANNEL_NUM_MAX];	//用于函数findTargetSegment()切片产生数据的存放

static SCheckPathInfo* g_pSCheckPathInfo[CHANNEL_NUM_MAX];		//用于函数getMachiningPath()切片产生数据的存放
static SGetPathInfo* g_pSGetPathInfo[CHANNEL_NUM_MAX];			//用于函数

/*************************************************
函数名称：void addAllGCommandRegulationAPI()
函数功能：添加所有G指令规则函数接口
输入参数：
输出参数：
返 回 值：
备    注：开机初始化调用
**************************************************/
static int8 checkMillingG94(SSyntaxConfiguration* config, struct SGCodeParse* block)
{
    block->SModalInfo.sGg = G94 * pow(10, GCOMMAND_DECIMAL_PLACE);
    return 2;
}
static int8 assignMillingG94(SAssignConfiguration* config, struct SGCodeParse* block)
{
    block->SModalInfo.sGg = G94 * pow(10, GCOMMAND_DECIMAL_PLACE);
    return 2;
}
void addAllGCommandRegulationAPI()
{
	addG0RegulationAPI();
	addG1RegulationAPI();
	addG2RegulationAPI();
	addG3RegulationAPI();

	addG4RegulationAPI();

	addG17RegulationAPI();
	addG18RegulationAPI();
	addG19RegulationAPI();

	addG20RegulationAPI();
	addG21RegulationAPI();

	addG40RegulationAPI();

	addG54RegulationAPI();
	addG55RegulationAPI();
	addG56RegulationAPI();
	addG57RegulationAPI();
	addG58RegulationAPI();
	addG59RegulationAPI();


	addG90RegulationAPI();//企业已经没有这个指令
	addG91RegulationAPI();
    addGCommadRegulation(94, sGg, 1, NULL, NULL, NULL, checkMillingG94, assignMillingG94);

}

/*************************************************
函数名称：void addAllMCommandRegulationAPI()
函数功能：添加所有M指令规则函数接口
输入参数：
输出参数：
返 回 值：
备    注：开机初始化调用
**************************************************/
void addAllMCommandRegulationAPI()
{

	addM00RegulationAPI();
	addM01RegulationAPI();
	addM02RegulationAPI();
	addM30RegulationAPI();

}

/*************************************************
函数名称：void freeAllCommandParaStruct()
函数功能：释放所有指令的参数结构体
输入参数：
输出参数：
返 回 值：
备    注：
**************************************************/
void initGCodeCommand(uint8 ucChannelNum)
{
	//1.释放指令参数结构体
	freeAllCommandParaStruct(ucChannelNum);


	//2.初始化扩展指令接口中的全局变量
	if (g_pSTextInfoCheck[ucChannelNum] != NULL)//用于检查流程函数findTargetSegment()切片产生数据的存放
	{
		freeGCode(g_pSTextInfoCheck[ucChannelNum]);
		g_pSTextInfoCheck[ucChannelNum] = NULL;
	}		
	if (g_pSTextInfoAssign[ucChannelNum] != NULL)	//用于函数findTargetSegment()切片产生数据的存放
	{
		freeGCode(g_pSTextInfoAssign[ucChannelNum]);
		g_pSTextInfoAssign[ucChannelNum] = NULL;
	}
	if (g_pSCheckPathInfo[ucChannelNum] != NULL)	//用于函数getMachiningPath()切片产生数据的存放
	{
		freeGCode(g_pSCheckPathInfo[ucChannelNum]);
		g_pSCheckPathInfo[ucChannelNum] = NULL;
	}
	if (g_pSGetPathInfo[ucChannelNum] != NULL)		//用于函数
	{
		//内部含有动态内存
		freeGCode(g_pSGetPathInfo[ucChannelNum]);
		g_pSGetPathInfo[ucChannelNum] = NULL;
	}

}

/*************************************************
函数名称：void freeAllCommandParaStruct()
函数功能：释放所有指令的参数结构体
输入参数：
输出参数：
返 回 值：
备    注：
**************************************************/
void freeAllCommandParaStruct(uint8 ucChannelNum)
{
	//ucMod == 0：释放用于检查的参数结构体
	//ucMod == 1：释放用于赋值的参数结构体
    for (uint8 ucMod = 0; ucMod < 2; ++ucMod)
	{
		//Ga组
		freeG2ParaStruct(ucMod, ucChannelNum);
		freeG3ParaStruct(ucMod, ucChannelNum);


	}
}



/*************************************************
函数名称：
函数功能：检查指令带的参数字是否符合要求
输入参数：pAllowLetter		允许参数字符串
          pSScanContainer	扫描结构体
输出参数：
返 回 值：	1	正确
			-1	错误
备    注：1.传入pAllowLetter格式必须为字符串格式，如： "XYZ" 或 "X"（因为字符串格式带有'\0'）
		  2.禁止输入单个字符格式，如'X'（因为字符格式不带有'\0'）
		  3.传入的字符串必须全为大写
		  4.多程序段指令暂时不能使用该函数进行检查
**************************************************/
int8 checkParaLetter(char* pAllowLetter, SScan* pSScanContainer)
{
	uint8 i = 0;
	uint8 j;
	int8 cAllowLetter[26];
	memset(cAllowLetter, 0, 26 * sizeof(int8));

	//1.在cAllowLetter中写入指令允许的参数字
	while (pAllowLetter[i] != '\0')
	{
		cAllowLetter[pAllowLetter[i] - 65] = 1;
		i++;
	}

	//2.在cAllowLetter中写入公共参数字  //GM FST RC
	i = 0;
	while (CommonPara[i] != '\0')
	{
		cAllowLetter[CommonPara[i] - 65] = 1;
		i++;
	}

	//3.比较指令实际出现的参数字与cAllowLetter
	for (i = 0; i < 26; i++)
	{
		//出现了允许参数字以外的字
		if (pSScanContainer->bA2Z[i] == TRUE && cAllowLetter[i] == FALSE)
		{
			//报错
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM403);
			return -1;
		}
	}
	
	return 1;
}

/*************************************************
函数名称：int8 checkParaRange(int8 cLetter, SScan* pSScanContainer, double dMin, double dMax)
函数功能：检查参数字的范围是否符合要求
输入参数：cLetter			允许参数字符串
		  pSScanContainer	扫描结构体
		  dMin				最小值
		  dMax				最大值
输出参数：
返 回 值：
备    注：1.传入的字符cLetter必须全为大写
		  2.多行指令字的检查要在获取参数函数中调用
**************************************************/
int8 checkParaRange(int8 cLetter, SScan* pSScanContainer, double dMin, double dMax)
{
	int8 ucRet = 1;

	//仅出现该指令字才判断值是否在要求范围内
	if (pSScanContainer->bA2Z[cLetter-65] == TRUE)
	{
		//dMin <= 目标值 && 目标值 <= dMax
		if (isEqualOrGreaterThanZero(pSScanContainer->dA2Z[cLetter - 65] - dMin) &&
			isEqualOrLessThanZero(pSScanContainer->dA2Z[cLetter - 65] - dMax))
		{
			return 1;
		}
		else
		{
			//报错
			//parseGCodeError()
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
			return -1;
		}
	}

	return 1;
}


/*************************************************
  函数名称：checkPolarCoordinateInput
  函数功能：检查 G16 普通运动段的平面坐标输入。
  输入参数：pSScanContainer 为尚未进行单位和极坐标转换的扫描输入；
            pSGCodeCurrSysState 为本段有效模态，两个指针均须有效。
  输出参数：非法参数记录报警；不修改扫描输入和系统状态。
  返 回 值：1 检查通过或非极坐标模式；-1 输入非法。
  备    注：供 G0～G3 检查和赋值使用；允许零极径及缺省 X/C。
**************************************************/
int8 checkPolarCoordinateInput(const SScan* pSScanContainer, const SGCodeSysState* pSGCodeCurrSysState)
{
	if (!pSGCodeCurrSysState->bPolarCoordinateProgramming)
	{
		return 1;
	}

	// 按参数是否出现判断，Y0/U0/V0 也不能被静默忽略。
	if (pSScanContainer->bXYZ[AXIS_Y] || pSScanContainer->bUVW[0] || pSScanContainer->bUVW[1])
	{
		parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM403);
		return -1;
	}

	if (pSScanContainer->bXYZ[AXIS_X])
	{
		double dRadius = pSScanContainer->dXYZ[AXIS_X]; // 本段显式极径。
		// 仅用局部值检查英制换算溢出，不提前修改扫描输入。
		if (pSGCodeCurrSysState->bImperialProgramming)
		{
			dRadius *= 25.4;
		}
		if (!isfinite(dRadius) || dRadius <= 0.0)
		{
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
			return -1;
		}
	}

	if (pSScanContainer->bABC[2] && !isfinite(pSScanContainer->dABC[2]))
	{
		parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
		return -1;
	}
	return 1;
}

/*************************************************
函数名称：int8 checkParaInteger(int8 cLetter, SScan* pSScanContainer)
函数功能：检查参数字的范围是否符合要求
输入参数：cLetter			允许参数字符串
		  pSScanContainer	扫描结构体

输出参数：
返 回 值：
备    注：1.传入的字符cLetter必须全为大写
		  2.多行指令字的检查要在获取参数函数中调用
**************************************************/
int8 checkParaInteger(int8 cLetter, SScan* pSScanContainer)
{
	double dValue;
	double dIntegerValue;

	if (pSScanContainer->bA2Z[cLetter - 65] == TRUE)
	{
		dValue = pSScanContainer->dA2Z[cLetter - 65];
		dIntegerValue = round(dValue);

		if (isZero(dValue - dIntegerValue))
		{
			return 1;
		}
		else
		{
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
			return -1;
		}
	}

	return -1;
}

/*************************************************
  函数名称：getMacCoordByScanInfo
  函数功能：将扫描坐标和总偏置转换为机械终点
  输入参数：ucChannelNum为有效通道；起点和总偏置为内部机械量；
            pSScanContainer为本段输入；除扫描指针检查外其余指针须有效
  输出参数：pSTargetPoint继承未指定轴；扫描数据沿用原有单位及极坐标修正
  返 回 值：无；扫描指针为空时报300并返回
  备    注：仅在Part4之后调用；英制线性输入原地换算一次，不可重复调用；
            G16固定X极径/C角度转机械X/Y，以工件原点计算，显式X按直径设置换算；
            无X/C时保留X/Y，缺省角从起点恢复，极点处约定0度；
            Z/W、A/B复用普通规则，C仅作极角，不作旋转轴目标
**************************************************/
void getMacCoordByScanInfo(uint8 ucChannelNum, SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSTargetPoint, struct _SGCodeSysState* pSGCodeCurrSysState)
{
	if (pSScanContainer == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//相对坐标编程标志
	bool bRelativeCoordinateProgramming = pSGCodeCurrSysState->bRelativeCoordinateProgramming;
	//极坐标编程标志
	bool bPolarCoordinateProgramming = pSGCodeCurrSysState->bPolarCoordinateProgramming;
	//英制编程标志
	bool bImperialProgramming = pSGCodeCurrSysState->bImperialProgramming;
	//当前总偏置
	SPointDouble* pSTotalOffset = &pSGCodeCurrSysState->SAllOffsetOfCoord.STotalOffset;

	//0.初始化目标点
	*pSTargetPoint = *pSStartPoint;

	//修正英制
	if (bImperialProgramming)
	{
		int i;
		for (i = 0; i < 3; i++)
		{
			pSScanContainer->dXYZ[i] *= 25.4;
			pSScanContainer->dUVW[i] *= 25.4;
			pSScanContainer->dRIJK[i] *= 25.4;

		}
		pSScanContainer->dRIJK[3] *= 25.4;
	}

	//1.当前终点坐标通过极坐标表达：将极坐标转为直角坐标（XC -> XY）
	if (bPolarCoordinateProgramming && (pSScanContainer->bXYZ[AXIS_X] || pSScanContainer->bABC[2]))
	{
		double dStartWorkX = pSStartPoint->dArray[AXIS_X] - pSTotalOffset->dArray[AXIS_X];
		double dStartWorkY = pSStartPoint->dArray[AXIS_Y] - pSTotalOffset->dArray[AXIS_Y];
		//修正C值
		pSScanContainer->dABC[2] = surplusDoubleToPositive(pSScanContainer->dABC[2], 360);
		//继承起点的极坐标
		if (pSScanContainer->bXYZ[0] == FALSE)
		{
			//pSScanContainer->dXYZ[0] = sqrt(pow(pSStartPoint->dArray[0], 2) + pow(pSStartPoint->dArray[1], 2));
			pSScanContainer->dXYZ[0] = sqrt(pow(dStartWorkX, 2) + pow(dStartWorkY, 2));
		}
		else if (g_SSystemPara.SAxisPara[AXIS_X].bShowByDiameter)
		{
			pSScanContainer->dXYZ[0] /= 2;
		}
		if (pSScanContainer->bABC[2] == FALSE)
		{
			// 角度继承，极点处沿用0度约定；其他位置从起点恢复角度，不能把省略C当成C0。
			//pSScanContainer->dABC[2] = (pSStartPoint->dArray[AXIS_X] == 0.0 && pSStartPoint->dArray[AXIS_Y] == 0.0)
			//	? 0.0 : atan2(pSStartPoint->dArray[AXIS_Y], pSStartPoint->dArray[AXIS_X]) * 180 / PI;
			pSScanContainer->dABC[2] = (dStartWorkX == 0.0 && dStartWorkY == 0.0) ? 0.0 : atan2(dStartWorkY, dStartWorkX) * 180 / PI;
		}
		//计算直角坐标(X,Y)
		//pSTargetPoint->dArray[AXIS_X] = pSScanContainer->dXYZ[0] * cos(pSScanContainer->dABC[2] * PI / 180);//x = x * cos(c)
		//pSTargetPoint->dArray[AXIS_Y] = pSScanContainer->dXYZ[0] * sin(pSScanContainer->dABC[2] * PI / 180);//y = x * sin(c)
		pSTargetPoint->dArray[AXIS_X] = pSScanContainer->dXYZ[0] * cos(pSScanContainer->dABC[2] * PI / 180) + pSTotalOffset->dArray[AXIS_X];//x = x * cos(c) + offsetX
		pSTargetPoint->dArray[AXIS_Y] = pSScanContainer->dXYZ[0] * sin(pSScanContainer->dABC[2] * PI / 180) + pSTotalOffset->dArray[AXIS_Y];//y = x * sin(c) + offsetY
	
	}

	//1.读取XYZ。
		for (uint8 ucTemp = 0; ucTemp < sizeof(pSScanContainer->bXYZ) / sizeof(bool); ucTemp++)
		{
		//极坐标G16条件下复用普通坐标规则；只跳过已由极坐标处理的X/Y和极角C。
		if (bPolarCoordinateProgramming && (ucTemp == AXIS_X || ucTemp == AXIS_Y))
		{
			continue;
		}

		//1.出现XYZ：机械坐标 = 工件坐标 + 总偏置值
			if (pSScanContainer->bXYZ[ucTemp] == TRUE)
			{
				switch (ucTemp)
				{
				case AXIS_X:
				case AXIS_Y:
				case AXIS_Z:
				if (bRelativeCoordinateProgramming)
					{
                    pSTargetPoint->dArray[ucTemp] = pSStartPoint->dArray[ucTemp] + pSScanContainer->dXYZ[ucTemp];
					}
					else
					{
						pSTargetPoint->dArray[ucTemp] = pSScanContainer->dXYZ[ucTemp] + pSTotalOffset->dArray[ucTemp];
					}
				default:
					break;
				}
			}
			//2.没出现XYZ，出现UVW：机械坐标 = 起点机械坐标 + 增量工件坐标
			else if (pSScanContainer->bUVW[ucTemp] == TRUE)
			{
				switch (ucTemp)
				{
				case AXIS_X:
				case AXIS_Y:case AXIS_Z:
					if (g_SSystemPara.SAxisPara[ucTemp].bShowByDiameter)	//直径显示	//陈敏聪	2023/7/22
					{
						//
						pSTargetPoint->dArray[ucTemp] = pSStartPoint->dArray[ucTemp] + pSScanContainer->dUVW[ucTemp] / 2;
					}
					else
					{
						pSTargetPoint->dArray[ucTemp] = pSStartPoint->dArray[ucTemp] + pSScanContainer->dUVW[ucTemp];
					}
				default:
					break;
				}
			}




		//2.读取ABC
		for (uint8 ucTemp = 0; ucTemp < sizeof(pSScanContainer->bABC) / sizeof(bool); ucTemp++)
		{
			if (bPolarCoordinateProgramming && AXIS_A + ucTemp == AXIS_C)
			{
				continue;
			}

			//1.出现ABC
			if (pSScanContainer->bABC[ucTemp] == TRUE)
			{
				//绝对编程
				if (bRelativeCoordinateProgramming == FALSE)
				{
					//计算目标坐标：机械坐标 = 工件坐标 + 总偏置值
					pSTargetPoint->dArray[AXIS_A + ucTemp] = pSScanContainer->dABC[ucTemp] + pSTotalOffset->dArray[AXIS_A + ucTemp];
				}
				//增量编程
				else
				{
					// zxr: 增量只加到机械起点，不重复叠加偏移。
					pSTargetPoint->dArray[AXIS_A + ucTemp] = pSStartPoint->dArray[AXIS_A + ucTemp] +
						pSScanContainer->dABC[ucTemp];
				}

				//极坐标插补模式

				//普通插补模式

				//主轴作为插补联动轴时
				//当前轴号为主轴轴号
				if ((AXIS_A + ucTemp) == g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucAxis)
				{
					//修正主轴坐标为脉冲当量的整倍数
					uint8* ucMapArray;//通道映射
					double  dSpindleAxisPulseEquivalent;//主轴脉冲当量   dSpindleAxisPulseEquivalent
					int32 nPulseRound;//脉冲圆整结果

					ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;//获取通道映射
					dSpindleAxisPulseEquivalent = g_SSystemPara.SAxisPara[ucMapArray[AXIS_A + ucTemp]].dPulseToMm;//C轴脉冲当量  度/pulse
					nPulseRound = round(pSTargetPoint->dArray[AXIS_A + ucTemp] / dSpindleAxisPulseEquivalent);

					pSTargetPoint->dArray[AXIS_A + ucTemp] = nPulseRound * dSpindleAxisPulseEquivalent;
				}
			}


		}

	}



}

/*************************************************
  函数名称：getMacCoordByCoordInfo
  函数功能：将已提取坐标和总偏置转换为机械终点
  输入参数：ucChannelNum为有效通道；起点和总偏置为内部机械量；
            pSCoordInfo为本段坐标；除坐标指针检查外其余指针须有效
  输出参数：pSTargetPoint继承未指定轴；坐标数据沿用原有单位及极坐标修正
  返 回 值：无；坐标指针为空时报300并返回
  备    注：英制线性输入原地换算一次，不可重复调用；
            G16固定X极径/C角度转机械X/Y，以工件原点计算，显式X按直径设置换算；
            无X/C时保留X/Y，缺省角从起点恢复，极点处约定0度；
            Z/W、A/B复用普通规则，C仅作极角，不作旋转轴目标
**************************************************/
void getMacCoordByCoordInfo(uint8 ucChannelNum, SCoord* pSCoordInfo, SPointDouble* pSStartPoint, SPointDouble* pSTargetPoint, struct _SGCodeSysState* pSGCodeCurrSysState)
{
	if (pSCoordInfo == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}
	//相对坐标编程标志
	bool bRelativeCoordinateProgramming = pSGCodeCurrSysState->bRelativeCoordinateProgramming;
	//极坐标编程标志
	bool bPolarCoordinateProgramming = pSGCodeCurrSysState->bPolarCoordinateProgramming;
	//英制编程标志
	bool bImperialProgramming = pSGCodeCurrSysState->bImperialProgramming;
	//当前总偏置
	SPointDouble* pSTotalOffset = &pSGCodeCurrSysState->SAllOffsetOfCoord.STotalOffset;

	//0.初始化目标点
	*pSTargetPoint = *pSStartPoint;
	
	//修正英制
	if (bImperialProgramming)
	{
		int i;
		for (i = 0; i < 3; i++)
		{
			pSCoordInfo->dXYZ[i] *= 25.4;
			pSCoordInfo->dUVW[i] *= 25.4;
		}
	}
	//1.当前终点坐标通过极坐标表达：将极坐标转为直角坐标（XC -> XY）
	if (bPolarCoordinateProgramming && (pSCoordInfo->bXYZ[AXIS_X] || pSCoordInfo->bABC[2]))
	{
		double dStartWorkX = pSStartPoint->dArray[AXIS_X] - pSTotalOffset->dArray[AXIS_X];
		double dStartWorkY = pSStartPoint->dArray[AXIS_Y] - pSTotalOffset->dArray[AXIS_Y];
		//修正C值
		pSCoordInfo->dABC[2] = surplusDoubleToPositive(pSCoordInfo->dABC[2], 360);
		//继承起点的极坐标
		if (pSCoordInfo->bXYZ[0] == FALSE)
		{
			//pSCoordInfo->dXYZ[0] = sqrt(pow(pSStartPoint->dArray[0], 2) + pow(pSStartPoint->dArray[1], 2));
			pSCoordInfo->dXYZ[0] = sqrt(pow(dStartWorkX, 2) + pow(dStartWorkY, 2));
		}
		else if (g_SSystemPara.SAxisPara[AXIS_X].bShowByDiameter)
		{
			pSCoordInfo->dXYZ[0] /= 2;
		}
		if (pSCoordInfo->bABC[2] == FALSE)
		{
			//// 极点处沿用0度约定；其他位置从起点恢复角度，不能把省略C当成C0。
			//pSCoordInfo->dABC[2] = (pSStartPoint->dArray[AXIS_X] == 0.0 && pSStartPoint->dArray[AXIS_Y] == 0.0)
			//	? 0.0 : atan2(pSStartPoint->dArray[AXIS_Y], pSStartPoint->dArray[AXIS_X]) * 180 / PI;

		pSCoordInfo->dABC[2] = (dStartWorkX == 0.0 && dStartWorkY == 0.0) ? 0.0 : atan2(dStartWorkY, dStartWorkX) * 180 / PI;
		}
		//计算直角坐标
		//pSTargetPoint->dArray[AXIS_X] = pSCoordInfo->dXYZ[0] * cos(pSCoordInfo->dABC[2] * PI / 180);//x = x * cos(c)
		//pSTargetPoint->dArray[AXIS_Y] = pSCoordInfo->dXYZ[0] * sin(pSCoordInfo->dABC[2] * PI / 180);//y = x * sin(c)
		pSTargetPoint->dArray[AXIS_X] = pSCoordInfo->dXYZ[0] * cos(pSCoordInfo->dABC[2] * PI / 180) + pSTotalOffset->dArray[AXIS_X];//x = x * cos(c) + offsetX
		pSTargetPoint->dArray[AXIS_Y] = pSCoordInfo->dXYZ[0] * sin(pSCoordInfo->dABC[2] * PI / 180) + pSTotalOffset->dArray[AXIS_Y];//y = x * sin(c) + offsetY
	}
	//2.复用普通坐标规则；G16只跳过已由极坐标处理的X/Y和极角C。
	{
		//1.读取XYZ
		for (uint8 ucTemp = 0; ucTemp < sizeof(pSCoordInfo->bXYZ) / sizeof(bool); ucTemp++)
		{
			if (bPolarCoordinateProgramming && (ucTemp == AXIS_X || ucTemp == AXIS_Y))
			{
				continue;
			}

			//1.出现XYZ：机械坐标 = 工件坐标 + 总偏置值
			if (pSCoordInfo->bXYZ[ucTemp] == TRUE)
			{
				switch (ucTemp)
				{
				case AXIS_X:
				case AXIS_Y:
				case AXIS_Z:
					if (bRelativeCoordinateProgramming)
					{
                        pSTargetPoint->dArray[ucTemp] = pSStartPoint->dArray[ucTemp] + pSCoordInfo->dXYZ[ucTemp];
					}
					else
					{
						pSTargetPoint->dArray[ucTemp] = pSCoordInfo->dXYZ[ucTemp] + pSTotalOffset->dArray[ucTemp];
					}
				default:
					break;
				}
			}
			//2.没出现XYZ，出现UVW：机械坐标 = 起点机械坐标 + 增量工件坐标
			else if (pSCoordInfo->bUVW[ucTemp] == TRUE)
			{
				switch (ucTemp)
				{
				case AXIS_X:
				case AXIS_Y:case AXIS_Z:
					if (g_SSystemPara.SAxisPara[ucTemp].bShowByDiameter)	//直径显示	//陈敏聪	2023/7/22
					{
						//
						pSTargetPoint->dArray[ucTemp] = pSStartPoint->dArray[ucTemp] + pSCoordInfo->dUVW[ucTemp] / 2;
					}
					else
					{
						pSTargetPoint->dArray[ucTemp] = pSStartPoint->dArray[ucTemp] + pSCoordInfo->dUVW[ucTemp];
					}
				default:
					break;
				}
			}


		}

		//2.读取ABC
		for (uint8 ucTemp = 0; ucTemp < sizeof(pSCoordInfo->bABC) / sizeof(bool); ucTemp++)
		{
			if (bPolarCoordinateProgramming && AXIS_A + ucTemp == AXIS_C)
			{
				continue;
			}

			//1.出现ABC
			if (pSCoordInfo->bABC[ucTemp] == TRUE)
			{
				//绝对编程
				if (bRelativeCoordinateProgramming == FALSE)
				{
					//计算目标坐标：机械坐标 = 工件坐标 + 总偏置值
					pSTargetPoint->dArray[AXIS_A + ucTemp] = pSCoordInfo->dABC[ucTemp] + pSTotalOffset->dArray[AXIS_A + ucTemp];
				}
				//增量编程
				else
				{
					// zxr: 增量只加到机械起点，不重复叠加偏移。
					pSTargetPoint->dArray[AXIS_A + ucTemp] = pSStartPoint->dArray[AXIS_A + ucTemp] +
						pSCoordInfo->dABC[ucTemp];
				}

				//极坐标插补模式

				//普通插补模式

				//主轴作为插补联动轴时
				//当前轴号为主轴轴号
				if ((AXIS_A + ucTemp) == g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucAxis)
				{
					//修正主轴坐标为脉冲当量的整倍数
					uint8* ucMapArray;//通道映射
					double  dSpindleAxisPulseEquivalent;//主轴脉冲当量   dSpindleAxisPulseEquivalent
					int32 nPulseRound;//脉冲圆整结果

					ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;//获取通道映射
					dSpindleAxisPulseEquivalent = g_SSystemPara.SAxisPara[ucMapArray[AXIS_A + ucTemp]].dPulseToMm;//C轴脉冲当量  度/pulse
					nPulseRound = round(pSTargetPoint->dArray[AXIS_A + ucTemp] / dSpindleAxisPulseEquivalent);

					pSTargetPoint->dArray[AXIS_A + ucTemp] = nPulseRound * dSpindleAxisPulseEquivalent;
				}
			}


		}

	}
}

//获取扫描结构体中螺距的导程
void getLeadByCoordInfo(SScan* pSScanContainer, double* pLead)
{
	if (pSScanContainer == NULL || pLead == NULL)
	{
		return;
	}
	if (pSScanContainer->bF)
	{
		*pLead = pSScanContainer->dF;
	}
	else if (pSScanContainer->bRIJK[1])
	{
		if (pSScanContainer->dRIJK[1] == 0)
		{
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM412);
			return;
		}
		//1英寸（25.4mm）长度上螺纹的牙数为I
		*pLead = 25.4 / pSScanContainer->dRIJK[1];
	}
}

//当前坐标编程方式是否为绝对编程（是绝对编程还是相对编程）
bool isAbsProgrammingMethod(uint8 ucMod, uint8 ucChannelNum)
{

	struct SGCodeParse* pSGCodeBuffUnit = NULL;
	SSyntaxConfiguration* pSSyntax = NULL;
	SAssignConfiguration* pSAssign = NULL;

	//语法检查
	if (ucMod== GC_SYNTAX)
	{
		//获取通道的语法检查结构体
		pSSyntax = getSyntaxConfig(ucChannelNum);
		//获取语法检查的缓冲区
		pSGCodeBuffUnit = &pSSyntax->SCurrRecord;

	}
	//语法赋值
	else
	{
		//获取通道的语法赋值结构体
		pSAssign = getAssignConfig(ucChannelNum);

		//获取语法赋值的G代码解析后的缓冲区
		pSGCodeBuffUnit = &pSAssign->pGCodeBuf[getSGCodeParsePointerIndex(GCODE_WRITE, ucChannelNum)];

	}

	if (pSGCodeBuffUnit->SModalInfo.sGh == G90 * pow(10, GCOMMAND_DECIMAL_PLACE))
	{
		return TRUE;
	}
	else
	{
		return  FALSE;
	}

}



//判断扫描结构体中是否含有坐标信息
bool isSegmentHaveCoordInfo(SScan* pSScanContainer)
{
	for (int8 i = 0; i < 3; i++)
	{
		if (pSScanContainer->bXYZ[i] == TRUE ||
			pSScanContainer->bUVW[i] == TRUE ||
			pSScanContainer->bABC[i] == TRUE)
		{
			return TRUE;
		}
	}
	
	return FALSE;
}

void getCoordInfoFromScanInfo(SCoord* pSCoordInfo, SScan* pSScanContainer)
{
	uint8 i;
	for (i=0;i<3;i++)
	{
		if (pSScanContainer->bXYZ[i] == TRUE && pSScanContainer->bUVW[i])
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM309);
			return;
		}
	}

	for (i = 0; i < 3; i++)
	{
		if (pSScanContainer->bABC[i] == TRUE)
		{
			pSCoordInfo->bABC[i] = pSScanContainer->bABC[i];
			pSCoordInfo->dABC[i] = pSScanContainer->dABC[i];
		}
		if (pSScanContainer->bXYZ[i] == TRUE)
		{
			pSCoordInfo->bUVW[i] = FALSE;
			pSCoordInfo->dUVW[i] = 0;
			pSCoordInfo->bXYZ[i] = pSScanContainer->bXYZ[i];
			pSCoordInfo->dXYZ[i] = pSScanContainer->dXYZ[i];
		}
		if (pSScanContainer->bUVW[i] == TRUE)
		{
			pSCoordInfo->bXYZ[i] = FALSE;
			pSCoordInfo->dXYZ[i] = 0;
			pSCoordInfo->bUVW[i] = pSScanContainer->bUVW[i];
			pSCoordInfo->dUVW[i] = pSScanContainer->dUVW[i];
		}
	}	
}



//指令类型赋值暂时这样写死
//未在规则中声明指令类型，则写不活，必须进行穷举
void getCommandType(int16 sCommand, uint32* pCommandType)
{
	switch (sCommand)
	{
	case G00 * GCOMMAND_DECIMAL_PLACE * 10:
		setBits(*pCommandType, CMD_MOTION_QUICKMOVE, CMD_MOTION_ADDR);
		break;
	case G01 * GCOMMAND_DECIMAL_PLACE * 10:
		setBits(*pCommandType, CMD_MOTION_LINE, CMD_MOTION_ADDR);
		break;
	case G02 * GCOMMAND_DECIMAL_PLACE * 10:
		setBits(*pCommandType, CMD_MOTION_ARC, CMD_MOTION_ADDR);
		break;
	case G03 * GCOMMAND_DECIMAL_PLACE * 10:
		setBits(*pCommandType, CMD_MOTION_ANTI_ARC, CMD_MOTION_ADDR);
		break;

	default: //默认G00
		setBits(*pCommandType, CMD_MOTION_QUICKMOVE, CMD_MOTION_ADDR);
		break;
	}
	
}

/*************************************************
  函数名称：
  函数功能  赋值/检查流程中找到目标程序段N
  输入参数：ucMod				模式（检查/赋值）
			ucChannelNum		通道号
			pSCurrProgram		程序结构体
			unTargetN			目标程序段号N
			pReturnTextInfo		获取文本状态信息（输出）
  输出参数：
  返 回 值：1	已找到
			0	未找完
			-1	未找到
			-2	错误
  备    注：从当前程序文本的sBeginLineNum开始查找
**************************************************/
int8 findTargetSegment(uint8 ucMod, uint8 ucChannelNum, SProgram* pSCurrProgram, uint32 unTargetN, SGetTextStateInfo* pSTargetTextInfo)
{	
	if (pSCurrProgram == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -2;
	}

	SGetTextStateInfo** ppSGetTextInfo;

	ppSGetTextInfo = ucMod == GC_SYNTAX ? &g_pSTextInfoCheck[ucChannelNum] : &g_pSTextInfoAssign[ucChannelNum];
	
	//1.若第一次执行，则从程序的起始行开始
	if ((*ppSGetTextInfo) == NULL)
	{
		(*ppSGetTextInfo) = mallocGCode(sizeof(SGetTextStateInfo), ORAM_NUM350);

		if ((*ppSGetTextInfo) == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -2;
		}

		//初始化获取文本状态结构体
		memset(*ppSGetTextInfo, 0, sizeof(SGetTextStateInfo));
		//获取行号
		(*ppSGetTextInfo)->unGetLineNum = pSCurrProgram->sBeginLineNum;
	}

	//2.记录当前获取文本状态信息
	if ((*ppSGetTextInfo)->ucStep == 0)
	{
		*pSTargetTextInfo = **ppSGetTextInfo;
	}

	//3.处理文本
	int8 cRet;
	cRet = processProgramText(ucMod, *ppSGetTextInfo, ucChannelNum, pSCurrProgram);
	
	//3.1程序段未处理完成
	if (cRet == 0)
	{
		return 0;
	}
	//3.2已得到普通程序段（替换宏名+去除空格）
	else if (cRet == 1)
	{
		if (*(*ppSGetTextInfo)->cStr == 'N'|| *(*ppSGetTextInfo)->cStr == 'n')
		{
			int8* pVal;
			pVal = (*ppSGetTextInfo)->cStr + 1;
			if (isZero(atof(pVal) - (int32)atof(pVal)))//N值为整数	//注意：此处不能用atof，要用getNumbers防止N0X100出错
			{
				if ((int32)atof(pVal) == unTargetN)//（已找到）
				{
					//释放
					freeGCode(*ppSGetTextInfo);
					(*ppSGetTextInfo) = NULL;
					return 1;
				}
				else//（未找完）
				{
					return 0;
				}
			}

		}
	}
	else if (cRet == 2 || cRet == 3)
	{
		//找目标程序段不允许
		parseGCodeErrorB((*ppSGetTextInfo)->unCurrLineNum, GCODE_ERROR_NUM448); //zxd :找不到行号报错
		//释放
		freeGCode(*ppSGetTextInfo);
		(*ppSGetTextInfo) = NULL;
		return -1;
	}
	
	return 0;
}


int8 findIfAndWhileSegment(uint8 ucMod, uint8 ucChannelNum, SProgram* pSCurrProgram, SGetTextStateInfo* pSGetTextStateInfo)
{
	if (pSCurrProgram == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -2;
	}

	SGetTextStateInfo** ppSGetTextInfo;

	ppSGetTextInfo = ucMod == GC_SYNTAX ? &g_pSTextInfoCheck[ucChannelNum] : &g_pSTextInfoAssign[ucChannelNum];

	//1.若第一次执行，则从程序的起始行开始
	if ((*ppSGetTextInfo) == NULL)
	{
		(*ppSGetTextInfo) = mallocGCode(sizeof(SGetTextStateInfo), ORAM_NUM350);

		if ((*ppSGetTextInfo) == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -2;
		}

		//初始化获取文本状态结构体
		memset(*ppSGetTextInfo, 0, sizeof(SGetTextStateInfo));

		**ppSGetTextInfo = *pSGetTextStateInfo;
		
	}

	//2.记录当前获取文本状态信息
	if ((*ppSGetTextInfo)->ucStep == 0)
	{
		*pSGetTextStateInfo = **ppSGetTextInfo;
	}

	//3.处理文本
	int8 cRet;
	cRet = processProgramText(ucMod, *ppSGetTextInfo, ucChannelNum, pSCurrProgram);

	//3.1程序段未处理完成
	if (cRet == 0)
	{
		return 0;
	}
	//3.2已得到普通程序段（替换宏名+去除空格）
	else if (cRet == 1)
	{
		//3.2.1找到含有关键字的程序段
		if (isTagetStr_B("IF", &(*ppSGetTextInfo)->cStr) ||
			isTagetStr_B("ELSEIF", &(*ppSGetTextInfo)->cStr) ||
			isTagetStr_B("ELSE", &(*ppSGetTextInfo)->cStr) ||
			isTagetStr_B("ENDIF", &(*ppSGetTextInfo)->cStr) ||
			isTagetStr_B("WHILE", &(*ppSGetTextInfo)->cStr) ||
			isTagetStr_B("ENDWHILE", &(*ppSGetTextInfo)->cStr))
		{
			//释放
			freeGCode(*ppSGetTextInfo);
			(*ppSGetTextInfo) = NULL;
			return 1;

		}
		//3.2.2当前程序段无关键字
		else
		{
			return 0;
		}
	}
	 else if (cRet == 2 || cRet == 3)
	{
		// freeGCode(*ppSGetTextInfo);
		// (*ppSGetTextInfo) = NULL;
		// return -1;
	}
	else if (cRet < 0)
	{
		freeGCode(*ppSGetTextInfo);
		(*ppSGetTextInfo) = NULL;
		return -1;
	}

}



/*************************************************
  函数名称：int8 checkMachiningPathMonotonicity(SProgram* pSCurrProgram, uint32 unNS, uint32 unNF, SGetTextStateInfo* pSTargetTextInfo, uint8 ucChannelNum,int8 cNSMod, bool bXRequestMono, bool bZRequestMono)
  函数功能  检查加工轨迹单调性与起到点坐标
  输入参数：ucMod				模式（检查/赋值）
			pSCurrProgram		当前程序文本
			unNS				起始程序段
			unNF				结束程序段
			cNSMod				NS进刀方式（0：XZ轴都移动 1：仅X轴移动 2：仅Z轴移动）
			bXRequestMono		X方向单调要求
			bZRequestMono		Z方向单调要求
			pSPathList			轨迹链表（输出）
			pSTargetTextInfo	文本获取状态（输出）
			ucChannelNum		通道号
  输出参数：
  返 回 值：1	已获取所有轨迹
			0	轨迹未获取完成
			-1	错误
  备    注：该函数仅被检查流程使用
**************************************************/
int8 checkMachiningPathMonotonicity(SProgram* pSCurrProgram, uint32 unNS, uint32 unNF, SGetTextStateInfo* pSTargetTextInfo, uint8 ucChannelNum,int8 cNSMod, bool bXRequestMono, bool bZRequestMono)
{
	if (pSCurrProgram == NULL)
	{
		//报错
	}

	SCheckPathInfo** ppSCheckPathInfo;
	ppSCheckPathInfo = &g_pSCheckPathInfo[ucChannelNum];
	SChannelConfiguration* pSGCodeChannelConfig = getChannelConfig(ucChannelNum);
	SGCodeSysState* pSCurrGCodeSysState = &pSGCodeChannelConfig->SAssignConfig.SGCodeCurrSysState;

	//判断是否需要初始化SCheckPathInfo
	if ((*ppSCheckPathInfo) == NULL)
	{
		//开辟获取轨迹信息结构体
		(*ppSCheckPathInfo) = mallocGCode(sizeof(SCheckPathInfo), ORAM_NUM351);
		//获取轨迹信息结构体置0
		memset(*ppSCheckPathInfo, 0, sizeof(SCheckPathInfo));
		//初始化获取轨迹结构体非0变量
		(*ppSCheckPathInfo)->cPart = PART_1;


	

		(*ppSCheckPathInfo)->STotalOffset = pSCurrGCodeSysState->SAllOffsetOfCoord.STotalOffset;
		(*ppSCheckPathInfo)->SCurrState.sGa = pSCurrGCodeSysState->SModalInfo.sGa;
		(*ppSCheckPathInfo)->SCurrState.pSGaRegulation = pSCurrGCodeSysState->SModalInfo.pSGaRegulation;
		(*ppSCheckPathInfo)->SCurrState.sGz = pSCurrGCodeSysState->SModalInfo.sGz;
		(*ppSCheckPathInfo)->SCurrState.pSGzRegulation = pSCurrGCodeSysState->SModalInfo.pSGzRegulation;
		(*ppSCheckPathInfo)->SCurrState.SCurrPoint = pSCurrGCodeSysState->SCurrPoint;
		
		(*ppSCheckPathInfo)->SPointA = pSCurrGCodeSysState->SCurrPoint;  //起刀点

		//(*ppSCheckPathInfo)->pSPathList = initList(0, 0);//暂未对动态内存编号

	}

	switch ((*ppSCheckPathInfo)->cStep)
	{
	//0.找到目标程序段号NS
	case 0:
	{
		int8 cRet;
		cRet = findTargetSegment(GC_SYNTAX, ucChannelNum, pSCurrProgram, unNS, &(*ppSCheckPathInfo)->SGetTextStateInfo);
		if (cRet == 1)
		{
			(*ppSCheckPathInfo)->cStep = 1;
		}
		break;
	}

	//1.判断NS-NF数据
	case 1:
	{
		switch ((*ppSCheckPathInfo)->cPart)
		{
		//处理文本
		case SYNTAX_PART_1:
		{
			int8 cRet;
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//2.处理程序文本
			cRet = processProgramText(GC_SYNTAX, &(*ppSCheckPathInfo)->SGetTextStateInfo, ucChannelNum, pSCurrProgram);
			if (cRet == 1)
			{
				(*ppSCheckPathInfo)->cCommandSegmentCnt++;
				(*ppSCheckPathInfo)->cPart = PART_2;
			}
			break;
		}

		//初始化扫描结构体
		case SYNTAX_PART_2:
		{
			//1.扫描前需要初始化扫描结构体
			//1.1当前扫描程序段为指令的第一段，需要清空扫描结构体
			if ((*ppSCheckPathInfo)->cCommandSegmentCnt == 1)
			{
				initScanContainer(&(*ppSCheckPathInfo)->SScanContainer, TRUE);
			}
			//1.2当前扫描程序段非指令的第一段，不需清空MFST值，防止多程序段丢失MFST的值
			else
			{
				initScanContainer(&(*ppSCheckPathInfo)->SScanContainer, FALSE);
			}

			//2.pCurrPtr需要重新指向程序段字符串首字符
			(*ppSCheckPathInfo)->SGetTextStateInfo.pCurrPtr = (*ppSCheckPathInfo)->SGetTextStateInfo.cStr;

			//3.进入下一切片
			(*ppSCheckPathInfo)->cPart = PART_3;

			break;
		}

		//扫描
		case SYNTAX_PART_3:
		{
			int8 cRet;

			int16 sLastCommandVal;
			int16 sCurrCommandVal;
			const SGCommandRegulation* pSLastCommandRegulation;
			const SGCommandRegulation* pSCurrCommandRegulation;

			SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
			if (pCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}

			//扫描程序段
			cRet = scanSegment(GC_SYNTAX, &(*ppSCheckPathInfo)->SGetTextStateInfo, &(*ppSCheckPathInfo)->SScanContainer, pCurrProgram, ucChannelNum);
			//当前程序段全部扫描完成
			if (cRet == TRUE)
			{
				//1.获取上一程序段运动相关指令（Gz组优先级高于Ga组）
				if ((*ppSCheckPathInfo)->SCurrState.sGz != 0)
				{
					sLastCommandVal = (*ppSCheckPathInfo)->SCurrState.sGz;
					pSLastCommandRegulation = (*ppSCheckPathInfo)->SCurrState.pSGzRegulation;
				}
				else
				{
					sLastCommandVal = (*ppSCheckPathInfo)->SCurrState.sGa;
					pSLastCommandRegulation = (*ppSCheckPathInfo)->SCurrState.pSGaRegulation;
				}

				//2.获取当前程序段运动相关指令
				//2.1当前程序段出现Ga组指令
				if ((*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
				{
					sCurrCommandVal = (*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
					pSCurrCommandRegulation = (*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation;
					(*ppSCheckPathInfo)->pSCurrCommandRegulation = (*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation;
				}
				//2.2当前程序段出现Gz组指令
				else if ((*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
				{
					sCurrCommandVal = (*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
					pSCurrCommandRegulation = (*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation;
					(*ppSCheckPathInfo)->pSCurrCommandRegulation = (*ppSCheckPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation;
				}
				//2.3当前程序段未出现GaGz组指令,则取当前系统状态Ga组指令
				else
				{
					sCurrCommandVal = (*ppSCheckPathInfo)->SCurrState.sGa;
					pSCurrCommandRegulation = (*ppSCheckPathInfo)->SCurrState.pSGaRegulation;
					(*ppSCheckPathInfo)->pSCurrCommandRegulation = (*ppSCheckPathInfo)->SCurrState.pSGaRegulation;
				}

				//3.判断多程序段指令的每一程序段运动相关指令是否一致（赋值不需要判断）
				if ((*ppSCheckPathInfo)->cCommandSegmentCnt == 1)
				{
					(*ppSCheckPathInfo)->sCurrCommandVal = sCurrCommandVal;
				}
				else
				{
					if ((*ppSCheckPathInfo)->sCurrCommandVal != sCurrCommandVal)
					{
						//多程序段指令的第一段程序段的指令与后续程序段指令不一致，如：第一段为G71，第二段非G71
						parseGCodeErrorB((*ppSCheckPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
						break;
					}
				}


				//4.判断是否需要释放开辟指令参数结构体内存（仅运动相关指令第一程序段需要判断）
				if ((*ppSCheckPathInfo)->cCommandSegmentCnt == 1 && (sLastCommandVal != sCurrCommandVal))
				{
					//4.1释放上一指令字的参数结构体
					if (pSLastCommandRegulation->pFreeParaStructAPI != NULL)
					{
						pSLastCommandRegulation->pFreeParaStructAPI(GC_SYNTAX, ucChannelNum);
					}
					//4.2开辟当前指令字的参数结构体
					if (pSCurrCommandRegulation->pMallocParaStructAPI != NULL)
					{
						pSCurrCommandRegulation->pMallocParaStructAPI(GC_SYNTAX, ucChannelNum);
					}
				}

				//5.判断当前指令字是否需要写参数结构体
				if (pSCurrCommandRegulation->pGetParaAPI != NULL)
				{
					pSCurrCommandRegulation->pGetParaAPI(GC_SYNTAX, &(*ppSCheckPathInfo)->SScanContainer, ucChannelNum,NULL);
				}

				//6.判断cGa,cGz指令所有程序段是否扫描完成
				if ((*ppSCheckPathInfo)->cCommandSegmentCnt == pSCurrCommandRegulation->cSegmentNum)
				{
					//6.1运动相关指令所有程序段已全部扫描
					(*ppSCheckPathInfo)->cPart = PART_4;
				}
				else
				{
					//6.2还需继续获取运动相关指令程序段
					(*ppSCheckPathInfo)->cPart = PART_1;
				}
			}

			break;
		}

		//检查轨迹中的指令
		case SYNTAX_PART_4:
		{
			int8 cRet;
			cRet=isAllowGCommandInPath(&(*ppSCheckPathInfo)->SScanContainer);
			if (cRet == FALSE)
			{
				//报错
				parseGCodeErrorB((*ppSCheckPathInfo)->SGetTextStateInfo.unCurrLineNum,GCODE_ERROR_NUM130);
				return -1;
			}
		} 

		//判断当前程序段轨迹的单调性
		case SYNTAX_PART_5:
		{
			//仅记录轨迹移动的底层轨迹 !!!
			SPointDouble SCurrStartPoint;
			SPointDouble SCurrEndPoint;
			SCurrEndPoint = SCurrStartPoint = (*ppSCheckPathInfo)->SCurrState.SCurrPoint;

			//NS程序段
			if ((*ppSCheckPathInfo)->SScanContainer.bN == TRUE && (*ppSCheckPathInfo)->SScanContainer.dN == unNS)
			{
				//1.NS程序段当前指令必须为G00/G01
				if (((*ppSCheckPathInfo)->sCurrCommandVal != G00 * pow(10, GCOMMAND_DECIMAL_PLACE)) &&
					((*ppSCheckPathInfo)->sCurrCommandVal != G01 * pow(10, GCOMMAND_DECIMAL_PLACE)))
				{
					//报错
					parseGCodeErrorB(0, 0);
				}

				//2.获取终点坐标
				getMacCoordByScanInfo(ucChannelNum,&(*ppSCheckPathInfo)->SScanContainer, &SCurrStartPoint, &SCurrEndPoint, pSCurrGCodeSysState);

				//3.根据程序段NS的加工方向确定对应方向的单调性
				//3.1程序段NS为X方向加工
				if (!isZero(SCurrEndPoint.dArray[AXIS_X] - SCurrStartPoint.dArray[AXIS_X]))
				{
					//标记X轴在程序段NS中发生移动
					(*ppSCheckPathInfo)->bIsXMotionInNS = TRUE;
					//标记X轴在程序段NS中的单调性
					(*ppSCheckPathInfo)->cXMonotonicityInNS = SCurrEndPoint.dArray[AXIS_X] > SCurrStartPoint.dArray[AXIS_X] ? 1 : -1;
					//要求仅X方向进刀，则后续轨迹中X的单调性取决于NS中X的单调性
					if (cNSMod == 1)
					{
						//标记X轴在后续轨迹中的单调性（详见手册）
						(*ppSCheckPathInfo)->cXMonotonicityInPath = -(*ppSCheckPathInfo)->cXMonotonicityInNS;
					}
					//要求仅Z方向进刀，但NS程序段X轴发生移动
					else if (cNSMod == 2)
					{
						parseGCodeErrorB(0, 0);
					} 
				}
				//3.2程序段NS为Z方向加工
				if (!isZero(SCurrEndPoint.dArray[AXIS_Z] - SCurrStartPoint.dArray[AXIS_Z]))
				{	
					//标记X轴在程序段NS中发生移动
					(*ppSCheckPathInfo)->bIsZMotionInNS = TRUE;
					//标记程序段NS的单调性
					(*ppSCheckPathInfo)->cZMonotonicityInNS = SCurrEndPoint.dArray[AXIS_Z] > SCurrStartPoint.dArray[AXIS_Z] ? 1 : -1;
					//要求仅Z方向进刀，则后续轨迹中Z的单调性取决于NS中Z的单调性
					if (cNSMod == 2)
					{
						//后续加工轨迹Z方向的单调性取决于NS（详见手册）
						(*ppSCheckPathInfo)->cZMonotonicityInPath = -(*ppSCheckPathInfo)->cZMonotonicityInNS;
					}
					//3.2.2要求仅X方向进刀，但NS程序段Z轴发生移动
					else if (cNSMod == 1)
					{
						parseGCodeErrorB(0, 0);
					}		
				}
				//3.3程序段NS的XZ方向均未移动
				if (isZero(SCurrEndPoint.dArray[AXIS_X] - SCurrStartPoint.dArray[AXIS_X]) &&
					isZero(SCurrEndPoint.dArray[AXIS_Z] - SCurrStartPoint.dArray[AXIS_Z]))
				{
					//报错
					parseGCodeErrorB(0, 0);
				}

			}
			
			//非NS程序段
			else
			{
				//当前为底层运动指令 且 程序段中出现了坐标信息
				if (isBasicMotionCommand((*ppSCheckPathInfo)->sCurrCommandVal))
				{
					bool bXIsMono = TRUE;		//当前程序段X方向是否单调
					bool bZIsMono = TRUE;		//当前程序段Z方向是否单调
					int8 cXCurrMono;		//当前程序段X方向单调性
					int8 cZCurrMono;		//当前程序段Z方向单调性

					//1.获取轨迹的起点与终点
					getMacCoordByScanInfo(ucChannelNum,&(*ppSCheckPathInfo)->SScanContainer, &SCurrStartPoint, &SCurrEndPoint, pSCurrGCodeSysState);

					//2.获取X当前轨迹单调性	注：（仅适合线性轨迹的单调性判断）
					if (isZero(SCurrEndPoint.dArray[AXIS_X] - SCurrStartPoint.dArray[AXIS_X]))
					{
						cXCurrMono = 0;
					}
					else
					{
						cXCurrMono = SCurrEndPoint.dArray[AXIS_X] > SCurrStartPoint.dArray[AXIS_X] ? 1 : -1;
					}
				
					//3.获取Z当前轨迹单调性	注：（仅适合线性轨迹的单调性判断）
					if (isZero(SCurrEndPoint.dArray[AXIS_Z] - SCurrStartPoint.dArray[AXIS_Z]))
					{
						cZCurrMono = 0;
					}
					else
					{
						cZCurrMono = SCurrEndPoint.dArray[AXIS_Z] > SCurrStartPoint.dArray[AXIS_Z] ? 1 : -1;
					}
				
					//3.获取非线性轨迹单调性
					if ((*ppSCheckPathInfo)->sCurrCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE) ||
						(*ppSCheckPathInfo)->sCurrCommandVal == G03 * pow(10, GCOMMAND_DECIMAL_PLACE))
					{		
						SPointDouble SCurrCenterPoint;
						memset(&SCurrCenterPoint, 0, sizeof(SPointDouble));
						int8 cArcDir = (*ppSCheckPathInfo)->sCurrCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE) ? ARC_CW : ARC_ANTI_CW;
						double dArcRadius;
						
						getArcCenterPointAndRadius(&(*ppSCheckPathInfo)->SScanContainer, &SCurrStartPoint, &SCurrEndPoint, cArcDir, &SCurrCenterPoint, &dArcRadius, AXIS_Z, AXIS_X);

						getArcMonotonicity(&SCurrStartPoint, &SCurrEndPoint, &SCurrCenterPoint, cArcDir, &bXIsMono, &bZIsMono, &cXCurrMono, &cZCurrMono);
					}

					//4.判断XZ的单调性是否正确
					//4.1判断X方向单调性
					if (bXRequestMono)
					{
						//4.1.1当前轨迹不单调
						if (bXIsMono == FALSE)
						{
							//报错
							parseGCodeErrorB(0, 0);
						}
					
						//4.1.2未记录轨迹单调性，记录（理应不存在此情况，NS段已记录）
						if ((*ppSCheckPathInfo)->cXMonotonicityInPath == 0)
						{
							(*ppSCheckPathInfo)->cXMonotonicityInPath = cXCurrMono;
						}
						
						//4.1.3已记录轨迹单调性，判断
						else
						{
							//记录轨迹单调性 与 当前轨迹单调性不一致
							if (cXCurrMono != 0 && (*ppSCheckPathInfo)->cXMonotonicityInPath != cXCurrMono)
							{
								//报警
								parseGCodeErrorB(0, 0);
							}
						}
					}
					//4.2判断Z方向单调性
					if (bZRequestMono)
					{
						//4.2.1当前轨迹不单调
						if (bZIsMono == FALSE)
						{
							//报错
							parseGCodeErrorB(0, 0);
						}

						//4.2.2未记录轨迹单调性，记录（理应不存在此情况，NS段已记录）
						if ((*ppSCheckPathInfo)->cZMonotonicityInPath == 0)
						{
							(*ppSCheckPathInfo)->cZMonotonicityInPath = cZCurrMono;
						}

						//4.2.3已记录轨迹单调性，判断
						else
						{
							//记录轨迹单调性 与 当前轨迹单调性不一致
							if (cZCurrMono != 0 && (*ppSCheckPathInfo)->cZMonotonicityInPath != cZCurrMono)
							{
								//报警
								parseGCodeErrorB(0, 0);
							}
						}
					}

					//5.判断起刀点是否正确
					if (1)
					{
						//5.1比较起到点X 与 当前轨迹终点X
						if ((*ppSCheckPathInfo)->cXMonotonicityInNS == 1)
						{
							if (!(isEqualOrGreaterThanZero(SCurrEndPoint.dArray[AXIS_X] - (*ppSCheckPathInfo)->SPointA.dArray[AXIS_X])))
							{
								//报警
								parseGCodeErrorB(0, 0);
							}
						}
						else if ((*ppSCheckPathInfo)->cXMonotonicityInNS == -1)
						{
							if (!(isEqualOrGreaterThanZero((*ppSCheckPathInfo)->SPointA.dArray[AXIS_X] - SCurrEndPoint.dArray[AXIS_X])))
							{
								//报警
								parseGCodeErrorB(0, 0);
							}
						}

						//5.2比较起到点Z 与 当前轨迹终点Z
						if ((*ppSCheckPathInfo)->cZMonotonicityInNS == 1)
						{
							if (!(isEqualOrGreaterThanZero(SCurrEndPoint.dArray[AXIS_Z] - (*ppSCheckPathInfo)->SPointA.dArray[AXIS_Z])))
							{
								//报警
								parseGCodeErrorB(0, 0);
							}
						}
						else if ((*ppSCheckPathInfo)->cZMonotonicityInNS == -1)
						{
							if (!(isEqualOrGreaterThanZero((*ppSCheckPathInfo)->SPointA.dArray[AXIS_Z] - SCurrEndPoint.dArray[AXIS_Z])))
							{
								//报警
								parseGCodeErrorB(0, 0);
							}
						}
					}

				}

			}
			
			//更新状态
			(*ppSCheckPathInfo)->SCurrState.SCurrPoint = SCurrEndPoint;//记录偏移前的终点坐标
			if ((*ppSCheckPathInfo)->pSCurrCommandRegulation->cGroup == sGa)
			{
				(*ppSCheckPathInfo)->SCurrState.sGa = (*ppSCheckPathInfo)->sCurrCommandVal;
				(*ppSCheckPathInfo)->SCurrState.pSGaRegulation = (*ppSCheckPathInfo)->pSCurrCommandRegulation;
				(*ppSCheckPathInfo)->SCurrState.sGz = 0;
				(*ppSCheckPathInfo)->SCurrState.pSGzRegulation = NULL;
			}
			else if ((*ppSCheckPathInfo)->pSCurrCommandRegulation->cGroup == sGz)
			{
				(*ppSCheckPathInfo)->SCurrState.sGz = (*ppSCheckPathInfo)->sCurrCommandVal;
				(*ppSCheckPathInfo)->SCurrState.pSGzRegulation = (*ppSCheckPathInfo)->pSCurrCommandRegulation;
			}

			(*ppSCheckPathInfo)->cPart = ASSIGN_PART_6;

			break;
		}

		//判断NS至NF程序段是否获取完成
		case SYNTAX_PART_6:
		{
			//当前程序段段号为NF，获取完成
			if ((*ppSCheckPathInfo)->SScanContainer.bN == TRUE && (*ppSCheckPathInfo)->SScanContainer.dN == unNF)
			{
				(*ppSCheckPathInfo)->cStep = 2;
			}
			else
			{
				(*ppSCheckPathInfo)->cCommandSegmentCnt = 0;
				(*ppSCheckPathInfo)->cPart = PART_1;
			}
			break;
		}

		default:
			break;
		}
		break;
	}

	//2.轨迹单调性已检查完成
	case 2:
	{
		//1.输出pSTargetTextInfo
		*pSTargetTextInfo = (*ppSCheckPathInfo)->SGetTextStateInfo;

		//2.释放(*ppSCheckPathInfo)
		freeGCode(*ppSCheckPathInfo);

		(*ppSCheckPathInfo) = NULL;

		return 1;

		break;
	}

	default:
		break;
	}
	return 0;
}

int8 checkPathMonotonicity(SList* pPathList)
{
	int8 cSign;
	int i = 0;
	SNode* pCurNode = pPathList->pHead->pNextNode;

	//获取单调性
	for (; i < pPathList->cSize; i++)
	{
		SLine* pSLine = pCurNode->pData;
		cSign = sgn(pSLine->SEndPoint.dArray[AXIS_Z] - pSLine->SStartPoint.dArray[AXIS_Z]);
		
		if (cSign != 0)
		{
			break;
		}
		pCurNode = pCurNode->pNextNode;
	}
	//检查
	for (; i < pPathList->cSize; i++)
	{
		SLine* pSLine = pCurNode->pData;
		if (sgn(pSLine->SEndPoint.dArray[AXIS_Z] - pSLine->SStartPoint.dArray[AXIS_Z]) != 0 &&
			cSign != sgn(pSLine->SEndPoint.dArray[AXIS_Z] - pSLine->SStartPoint.dArray[AXIS_Z]))
		{
			return -1;
		}
		pCurNode = pCurNode->pNextNode;
	}
	
	return 1;
}
int8 checkPathAxisMonotonicity(SList* pPathList, uint8 ucAxis)
{
    int i = 0;
    int8 cSign = 0;       /* 记住首次出现的非零方向 */
    int8 cCurSign = 0;
    SNode* pCurNode = pPathList->pHead->pNextNode;

    //获取单调性
    for (; i < pPathList->cSize; i++)
    {
        SLine* pSLine = pCurNode->pData;
        cSign = sgn(pSLine->SEndPoint.dArray[ucAxis] - pSLine->SStartPoint.dArray[ucAxis]);
        if (cSign != 0)
            break;
        pCurNode = pCurNode->pNextNode;
    }

    //检查
    for (; i < pPathList->cSize; i++)
    {
        SLine* pSLine = pCurNode->pData;
        cCurSign = sgn(pSLine->SEndPoint.dArray[ucAxis] - pSLine->SStartPoint.dArray[ucAxis]);
        if (cCurSign != 0 && cCurSign != cSign)
            return -1;
        pCurNode = pCurNode->pNextNode;
    }

    return 1;  
}
int8 checkPathXZMonotonicity(SList* pPathList)
{
    if (checkPathAxisMonotonicity(pPathList, AXIS_X) == -1)
        return -1;

    if (checkPathAxisMonotonicity(pPathList, AXIS_Z) == -1)
        return -1;

    return 1;
}
/*************************************************
  函数名称：
  函数功能  生成轨迹链表
  输入参数：ucMod				模式（检查/赋值）
			pSCurrProgram		程序文本
			unNS				起始行号
			unNF				结束行号
			dDeltaU				轨迹X方向偏置距离
			dDeltaW				轨迹Z方向偏置距离

			pSPathList			轨迹链表（输出）
			pSTargetTextInfo	获取文本状态（输出）

			ucChannelNum		通道号
  输出参数：
  返 回 值：1	已获取所有轨迹
			0	轨迹未获取完成
			-1	错误
  备    注：1.该函数仅被赋值流程使用
			2.G70不使用该函数
			//3.若轨迹终点X低于起刀点X，则会补充一段实体使终点X等于起刀点X
**************************************************/
int8 getMachiningPath(uint8 ucMod, SProgram* pSCurrProgram, uint32 unNS, uint32 unNF, double dDeltaU, double dDeltaW, SList** ppSPathList, SGetTextStateInfo* pSTargetTextInfo, uint8 ucChannelNum, int8 cG71orG72)
{
	if (pSCurrProgram == NULL)
	{
		return -1;
	}

	SGetPathInfo** ppSGetPathInfo;
	ppSGetPathInfo = &g_pSGetPathInfo[ucChannelNum];
	
	SGCodeSysState* pSCurrGCodeSysState = getCurrSystemState(ucChannelNum, ucMod);
	int16 sCurGz = pSCurrGCodeSysState->SModalInfo.sGz;//获取系统状态的cGz指令，用于判别当前是G71还是G72
	//
	if ((*ppSGetPathInfo) == NULL)
	{	
		//1.开辟获取轨迹信息结构体
		(*ppSGetPathInfo) = mallocGCode(sizeof(SGetPathInfo), ORAM_NUM352);

		if ((*ppSGetPathInfo) == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		//2.获取轨迹信息结构体置0
		memset(*ppSGetPathInfo, 0, sizeof(SGetPathInfo));

		//3.初始化获取轨迹结构体非0变量
		(*ppSGetPathInfo)->cPart = PART_1;

		//读取当前系统状态
		(*ppSGetPathInfo)->STotalOffset = pSCurrGCodeSysState->SAllOffsetOfCoord.STotalOffset;
		(*ppSGetPathInfo)->SCurrState.sGa = pSCurrGCodeSysState->SModalInfo.sGa;
		(*ppSGetPathInfo)->SCurrState.pSGaRegulation = pSCurrGCodeSysState->SModalInfo.pSGaRegulation;
		(*ppSGetPathInfo)->SCurrState.sGz = pSCurrGCodeSysState->SModalInfo.sGz;
		(*ppSGetPathInfo)->SCurrState.pSGzRegulation = pSCurrGCodeSysState->SModalInfo.pSGzRegulation;
		(*ppSGetPathInfo)->SCurrState.SCurrPoint = pSCurrGCodeSysState->SCurrPoint;

		(*ppSGetPathInfo)->SGetTextStateInfo.unGetLineNum = pSCurrProgram->sBeginLineNum;

		(*ppSGetPathInfo)->pSPathList = initList(ORAM_NUM353, ORAM_NUM354);
		if ((*ppSGetPathInfo)->pSPathList == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM406);
			return -1;
		}
		
	}

	switch ((*ppSGetPathInfo)->cStep)
	{
	//0.找到目标程序段号NS
	case 0:
	{
		int8 cRet;
		cRet = findTargetSegment(ucMod, ucChannelNum, pSCurrProgram, unNS, &(*ppSGetPathInfo)->SGetTextStateInfo);

		if (cRet < 0)
		{
			freeList(&(*ppSGetPathInfo)->pSPathList);
			freeGCode((*ppSGetPathInfo));
			return -1;
		}
		else if (cRet == 1)
		{
			(*ppSGetPathInfo)->cStep = 1;
		}
		break;
	}

	//1.获取NS-NF数据，生成轨迹链表
	case 1:
	{
		switch ((*ppSGetPathInfo)->cPart)
		{
		//处理文本
		case PART_1:
		{
			int8 cRet;
			SProgram* pSCurrProgram = NULL;   //当前程序结构体
			//1.获取当前程序结构体
			pSCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);
			if (pSCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}
			//2.处理程序文本
			cRet = processProgramText(ucMod, &(*ppSGetPathInfo)->SGetTextStateInfo, ucChannelNum, pSCurrProgram);
			if (cRet == 1)
			{
				(*ppSGetPathInfo)->cCommandSegmentCnt++;
				(*ppSGetPathInfo)->cPart = PART_2;
			}
			else if (cRet == 2)
			{
				//未找到轨迹链表的NF
				return -1;
			}
			break;
		}

		//初始化扫描结构体
		case PART_2:
		{
			//1.扫描前需要初始化扫描结构体
			//1.1当前扫描程序段为指令的第一段，需要清空扫描结构体
			if ((*ppSGetPathInfo)->cCommandSegmentCnt == 1)
			{
				initScanContainer(&(*ppSGetPathInfo)->SScanContainer, TRUE);
			}
			//1.2当前扫描程序段非指令的第一段，不需清空MFST值，防止多程序段丢失MFST的值
			else
			{
				initScanContainer(&(*ppSGetPathInfo)->SScanContainer, FALSE);
			}

			//2.pCurrPtr需要重新指向程序段字符串首字符
			(*ppSGetPathInfo)->SGetTextStateInfo.pCurrPtr = (*ppSGetPathInfo)->SGetTextStateInfo.cStr;

			//3.进入下一切片
			(*ppSGetPathInfo)->cPart = PART_3;

			break;
		}

		//扫描
		case PART_3:
		{
			int8 cRet;

			int16 sLastCommandVal;
			int16 sCurrCommandVal;
			const SGCommandRegulation* pSLastCommandRegulation;
			const SGCommandRegulation* pSCurrCommandRegulation;

			SProgram* pCurrProgram = FindNodeData(getProgramList(ucChannelNum), 0);		//当前程序结构体
			if (pCurrProgram == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM300);
				return -1;
			}

			//扫描程序段
			cRet = scanSegment(ucMod, &(*ppSGetPathInfo)->SGetTextStateInfo, &(*ppSGetPathInfo)->SScanContainer, pCurrProgram, ucChannelNum);
			//当前程序段全部扫描完成
			if (cRet == TRUE)
			{
				//1.获取上一程序段运动相关指令（Gz组优先级高于Ga组）
				if ((*ppSGetPathInfo)->SCurrState.sGz != 0)
				{
					sLastCommandVal = (*ppSGetPathInfo)->SCurrState.sGz;
					pSLastCommandRegulation = (*ppSGetPathInfo)->SCurrState.pSGzRegulation;
				}
				else
				{
					sLastCommandVal = (*ppSGetPathInfo)->SCurrState.sGa;
					pSLastCommandRegulation = (*ppSGetPathInfo)->SCurrState.pSGaRegulation;
				}

				//2.获取当前程序段运动相关指令
				//2.1当前程序段出现Ga组指令
				if ((*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation != NULL)
				{
					sCurrCommandVal = (*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation->nCommandVal;
					pSCurrCommandRegulation = (*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation;
					(*ppSGetPathInfo)->pSCurrCommandRegulation = (*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGaRegulation;
				}
				//2.2当前程序段出现Gz组指令
				else if ((*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation != NULL)
				{
					sCurrCommandVal = (*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation->nCommandVal;
					pSCurrCommandRegulation = (*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation;
					(*ppSGetPathInfo)->pSCurrCommandRegulation = (*ppSGetPathInfo)->SScanContainer.SScanModalInfo.pSGzRegulation;
				}
				//2.3当前程序段未出现GaGz组指令,则取当前系统状态Ga组指令
				else
				{
					sCurrCommandVal = (*ppSGetPathInfo)->SCurrState.sGa;
					pSCurrCommandRegulation = (*ppSGetPathInfo)->SCurrState.pSGaRegulation;
					(*ppSGetPathInfo)->pSCurrCommandRegulation = (*ppSGetPathInfo)->SCurrState.pSGaRegulation;
	      }
 
				//3.判断多程序段指令的每一程序段运动相关指令是否一致（赋值不需要判断）
				if ((*ppSGetPathInfo)->cCommandSegmentCnt == 1)
				{
					(*ppSGetPathInfo)->sCurrCommandVal = sCurrCommandVal;
				}
				else
				{
					if ((*ppSGetPathInfo)->sCurrCommandVal != sCurrCommandVal)
					{
						//多程序段指令的第一段程序段的指令与后续程序段指令不一致，如：第一段为G71，第二段非G71
						parseGCodeErrorB((*ppSGetPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM130);
						break;
					}
				}


				//4.判断是否需要释放开辟指令参数结构体内存（仅运动相关指令第一程序段需要判断）
				if ((*ppSGetPathInfo)->cCommandSegmentCnt == 1 && (sLastCommandVal != sCurrCommandVal))
				{
					//4.1释放上一指令字的参数结构体
					if (pSLastCommandRegulation->pFreeParaStructAPI != NULL)
					{
						pSLastCommandRegulation->pFreeParaStructAPI(ucMod, ucChannelNum);
					}
					//4.2开辟当前指令字的参数结构体
					if (pSCurrCommandRegulation->pMallocParaStructAPI != NULL)
					{

						if (pSCurrCommandRegulation->pMallocParaStructAPI(ucMod, ucChannelNum) <= 0)
						{
							parseGCodeErrorB((*ppSGetPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
							break;
						}
					}
				}

				//5.判断当前指令字是否需要写参数结构体
				if (pSCurrCommandRegulation->pGetParaAPI != NULL)
				{
				    pSCurrCommandRegulation->pGetParaAPI(ucMod, &(*ppSGetPathInfo)->SScanContainer, ucChannelNum, NULL);
				}

				//6.判断cGa,cGz指令所有程序段是否扫描完成
				if ((*ppSGetPathInfo)->cCommandSegmentCnt == pSCurrCommandRegulation->cSegmentNum)
				{
					//6.1运动相关指令所有程序段已全部扫描
					(*ppSGetPathInfo)->cPart = PART_4;
				}
				else
				{
					//6.2还需继续获取运动相关指令程序段
					(*ppSGetPathInfo)->cPart = PART_1;
				}
			}

			break;
		}

		//计算当前程序段对应的粗加工轨迹，并存入轨迹链表
		case PART_4:
		{
			//仅记录轨迹移动的底层轨迹 !!!
			SPointDouble SOriginalEndPoint = (*ppSGetPathInfo)->SCurrState.SCurrPoint;//记录轨迹数据偏移前的终点，用于继承

			//底层指令G01、G01、G02、G03
			//不为点
			if (isBasicMotionCommand((*ppSGetPathInfo)->sCurrCommandVal) &&
				isSegmentHaveCoordInfo(&(*ppSGetPathInfo)->SScanContainer))
			{
				//1.获取偏移后的轨迹数据，并存入轨迹链表
				//1.1底层运动指令为G0
				if ((*ppSGetPathInfo)->sCurrCommandVal == G00 * pow(10, GCOMMAND_DECIMAL_PLACE))
				{
					//1.1.1开辟内存
					SLine* pSPathData;
					pSPathData = mallocGCode(sizeof(SLine), ORAM_NUM355);//内存未编号
					if (pSPathData == NULL)
					{
						parseGCodeErrorB((*ppSGetPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM406);
						return -1;
					}
					memset(pSPathData, 0, sizeof(SLine));

					//1.1.2写入程序段原始轨迹数据
					pSPathData->SEndPoint = pSPathData->SStartPoint = (*ppSGetPathInfo)->SCurrState.SCurrPoint;
					getMacCoordByScanInfo(ucChannelNum, &(*ppSGetPathInfo)->SScanContainer, &pSPathData->SStartPoint, &pSPathData->SEndPoint, pSCurrGCodeSysState);
					//当前程序段段号为NS
					if ((*ppSGetPathInfo)->SScanContainer.bN == TRUE && (*ppSGetPathInfo)->SScanContainer.dN == unNS)
					{
						pSPathData->EntityType = isSamePoint(&pSPathData->SStartPoint, &pSPathData->SEndPoint) ? DOT : OTHERTYPE;//获取轨迹中，G0实体类型为OTHERTYPE
					}
					else
					{
						pSPathData->EntityType = isSamePoint(&pSPathData->SStartPoint, &pSPathData->SEndPoint) ? DOT : L1NE;//获取轨迹中，G0实体类型为L1NE
					}


					SOriginalEndPoint = pSPathData->SEndPoint;

					//1.1.3将原始轨迹数据进行偏移
					offsetEntity((enum ENTITY*)pSPathData, dDeltaU, dDeltaW);

					//1.1.4将偏移后的轨迹数据插入链表
					if(insertList((*ppSGetPathInfo)->pSPathList, 0, pSPathData, ORAM_NUM354) == FALSE)//内存未编号
					{
						freeGCode(pSPathData);
						freeList(&(*ppSGetPathInfo)->pSPathList);
						freeGCode((*ppSGetPathInfo));
						(*ppSGetPathInfo) = NULL;
						parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						return -1;
					}

				}
				//1.2底层运动指令为G1
				else if ((*ppSGetPathInfo)->sCurrCommandVal == G01 * pow(10, GCOMMAND_DECIMAL_PLACE))
				{
					//1.2.1开辟内存
					SLine* pSPathData;
					pSPathData = mallocGCode(sizeof(SLine), ORAM_NUM355);//内存未编号
					if (pSPathData == NULL)
					{
						parseGCodeErrorB((*ppSGetPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM406);
						return -1;
					}
					memset(pSPathData, 0, sizeof(SLine));

					//1.2.2写入程序段原始轨迹数据
					pSPathData->SEndPoint = pSPathData->SStartPoint = (*ppSGetPathInfo)->SCurrState.SCurrPoint;
					getMacCoordByScanInfo(ucChannelNum, &(*ppSGetPathInfo)->SScanContainer, &pSPathData->SStartPoint, &pSPathData->SEndPoint, pSCurrGCodeSysState);
					pSPathData->EntityType = isSamePoint(&pSPathData->SStartPoint, &pSPathData->SEndPoint) ? DOT : L1NE;
					SOriginalEndPoint = pSPathData->SEndPoint;

					//1.2.3将原始轨迹数据进行偏移
					offsetEntity((enum ENTITY*)pSPathData, dDeltaU, dDeltaW);

					//1.2.4将偏移后的轨迹数据插入链表
					if (insertList((*ppSGetPathInfo)->pSPathList, 0, pSPathData, ORAM_NUM354) == FALSE)//内存未编号
					{
						freeGCode(pSPathData);
						freeList(&(*ppSGetPathInfo)->pSPathList);
						freeGCode((*ppSGetPathInfo));
						(*ppSGetPathInfo) = NULL;
						parseGCodeErrorB(0, GCODE_ERROR_NUM406);
						return -1;
					}

				}
				//1.3底层运动指令为G2
				else if ((*ppSGetPathInfo)->sCurrCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE)
					|| (*ppSGetPathInfo)->sCurrCommandVal == G03 * pow(10, GCOMMAND_DECIMAL_PLACE))
				{
					//1.3.1开辟内存
					SArc* pSPathData;
					pSPathData = mallocGCode(sizeof(SArc), ORAM_NUM356);//内存未编号
					if (pSPathData == NULL)
					{
						parseGCodeErrorB((*ppSGetPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM406);
						return -1;
					}
					memset(pSPathData, 0, sizeof(SArc));

					//1.3.2写入程序段原始轨迹数据
					pSPathData->SEndPoint = pSPathData->SStartPoint = (*ppSGetPathInfo)->SCurrState.SCurrPoint;
					pSPathData->cArcDir = (*ppSGetPathInfo)->sCurrCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE) ? ARC_CW : ARC_ANTI_CW;
					getMacCoordByScanInfo(ucChannelNum, &(*ppSGetPathInfo)->SScanContainer, &pSPathData->SStartPoint, &pSPathData->SEndPoint, pSCurrGCodeSysState);
					SOriginalEndPoint = pSPathData->SEndPoint;

					uint8 ucHor, ucVer;

					SGCodeSysState* pSCurrSysState = getCurrSystemState(ucChannelNum, ucMod);
					getHorAndVerAxis(&ucHor, &ucVer, pSCurrSysState->cInterpolationState, pSCurrSysState->SModalInfo.sGc);
					getArcCenterPointAndRadius(&(*ppSGetPathInfo)->SScanContainer, &pSPathData->SStartPoint, &pSPathData->SEndPoint,
					pSPathData->cArcDir, &pSPathData->SCenterPoint, &pSPathData->dRadius, ucHor, ucVer);

					pSPathData->EntityType = isSamePoint(&pSPathData->SStartPoint, &pSPathData->SEndPoint) ? DOT : ARC;

					// 1.3.3 II型的额外处理  跨象限圆弧处理，分割跨象限圆弧
					int8 cResult = isArcIsGroove(&pSPathData->SStartPoint, &pSPathData->SEndPoint, &pSPathData ->SCenterPoint, pSPathData->cArcDir, cG71orG72);

					//1.3.3.1跨象限
					if (cResult == 1 || cResult == 2)
					{
						//1.求拆圆时后部分起点的加工方向系数
						//G71跨一二象限时或G72跨一四象限时，系数为1
						int8 cSign = (cResult == 1) ? 1 : -1;
						//2.初始化存放拆分圆弧的后部分数据结构体
						SArc* pSLastPathData;
						pSLastPathData = mallocGCode(sizeof(SArc), ORAM_NUM356);//内存未编号
						if (pSLastPathData == NULL)
						{
							parseGCodeErrorB((*ppSGetPathInfo)->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM406);
							return -1;
						}
						memset(pSLastPathData, 0, sizeof(SArc));
						//先初始化和前部分数据一样
						*pSLastPathData= *pSPathData;
						//3.计算拆圆后的数据
						//当前为G71指令
						if (cG71orG72 == G71)
						{
							//3.1计算后部分的起点以及终点
							pSLastPathData->SStartPoint.dArray[AXIS_X] = pSPathData->SCenterPoint.dArray[AXIS_X] + cSign * pSPathData->dRadius;     //根据象限，圆心X与半径求和
								
							pSLastPathData->SStartPoint.dArray[AXIS_Z] = pSPathData->SCenterPoint.dArray[AXIS_Z];
							pSLastPathData->SEndPoint = pSPathData->SEndPoint;
							//3.2修改前部分的终点
							pSPathData->SEndPoint = pSLastPathData->SStartPoint;
						}
						//当前为G72指令
						else if (cG71orG72 == G72)
						{
							//3.1计算后部分的起点以及终点
							pSLastPathData->SStartPoint.dArray[AXIS_Z] = pSPathData->SCenterPoint.dArray[AXIS_Z] + cSign * pSPathData->dRadius;     //根据象限，圆心X与半径求和

							pSLastPathData->SStartPoint.dArray[AXIS_X] = pSPathData->SCenterPoint.dArray[AXIS_X];
							pSLastPathData->SEndPoint = pSPathData->SEndPoint;
							//3.2修改前部分的终点
							pSPathData->SEndPoint = pSLastPathData->SStartPoint;
						}


						//4.将原始轨迹数据进行偏移
						offsetEntity((enum ENTITY*)pSPathData, dDeltaU, dDeltaW);
						offsetEntity((enum ENTITY*)pSLastPathData, dDeltaU, dDeltaW);
						//5.将偏移后的轨迹数据插入链表
						//先插前部分数据
						if (insertList((*ppSGetPathInfo)->pSPathList, 0, pSPathData, ORAM_NUM354) == FALSE)//内存未编号
						{
							freeGCode(pSPathData);
							freeGCode(pSLastPathData);
							freeList(&(*ppSGetPathInfo)->pSPathList);
							freeGCode((*ppSGetPathInfo));
							(*ppSGetPathInfo) = NULL;
							parseGCodeErrorB(0, GCODE_ERROR_NUM406);
							return -1;
						}
						//再插后部分的数据
						if (insertList((*ppSGetPathInfo)->pSPathList, 0, pSLastPathData, ORAM_NUM354) == FALSE)//内存未编号
						{
							freeGCode(pSLastPathData);
							freeList(&(*ppSGetPathInfo)->pSPathList);
							freeGCode((*ppSGetPathInfo));
							(*ppSGetPathInfo) = NULL;
							parseGCodeErrorB(0, GCODE_ERROR_NUM406);
							return -1;
						}

					}
					//1.3.3.2没有跨象限
					else
					{
						//3.将原始轨迹数据进行偏移
						offsetEntity((enum ENTITY*)pSPathData, dDeltaU, dDeltaW);

						//4将偏移后的轨迹数据插入链表
						if (insertList((*ppSGetPathInfo)->pSPathList, 0, pSPathData, ORAM_NUM354) == FALSE)//内存未编号
						{
							freeGCode(pSPathData);
							freeList(&(*ppSGetPathInfo)->pSPathList);
							freeGCode((*ppSGetPathInfo));
							(*ppSGetPathInfo) = NULL;
							parseGCodeErrorB(0, GCODE_ERROR_NUM406);
							return -1;
						}

					}				
				}
			}
			
			//2.更新状态
			(*ppSGetPathInfo)->SCurrState.SCurrPoint = SOriginalEndPoint;//记录偏移前的终点坐标
			if ((*ppSGetPathInfo)->pSCurrCommandRegulation->cGroup == sGa)
			{
				(*ppSGetPathInfo)->SCurrState.sGa = (*ppSGetPathInfo)->sCurrCommandVal;
				(*ppSGetPathInfo)->SCurrState.pSGaRegulation = (*ppSGetPathInfo)->pSCurrCommandRegulation;
				(*ppSGetPathInfo)->SCurrState.sGz = 0;
				(*ppSGetPathInfo)->SCurrState.pSGzRegulation = NULL;
			}
			else if ((*ppSGetPathInfo)->pSCurrCommandRegulation->cGroup == sGz)
			{
				(*ppSGetPathInfo)->SCurrState.sGz = (*ppSGetPathInfo)->sCurrCommandVal;
				(*ppSGetPathInfo)->SCurrState.pSGzRegulation = (*ppSGetPathInfo)->pSCurrCommandRegulation;
			}

			(*ppSGetPathInfo)->cPart = ASSIGN_PART_5;

			break;
		}

		//IF/WHILE
		case PART_5:
		{
			int8 cRet = 1;
			//1.B类宏关键字
			if ((*ppSGetPathInfo)->SScanContainer.cMacroType != 0)
			{
				switch ((*ppSGetPathInfo)->SScanContainer.cMacroType)
				{
				//WHILE
				case 6:
				{
					cRet = assignWHILE(ucChannelNum, &(*ppSGetPathInfo)->SScanContainer, &(*ppSGetPathInfo)->SGetTextStateInfo);
					break;

				}
				//ENDWHILE
				case 7:
				{
					cRet = assignENDWHILE(ucChannelNum, &(*ppSGetPathInfo)->SGetTextStateInfo);
					break;
				}
				default:
					cRet = 1;
					break;
				}

			}
			if (cRet == 1)
			{
				(*ppSGetPathInfo)->cPart = PART_6;
			}
			break;
		}

		//判断NS至NF程序段是否获取完成
		case PART_6:
		{
			//当前程序段段号为NF，获取完成
			if ((*ppSGetPathInfo)->SScanContainer.bN == TRUE && (*ppSGetPathInfo)->SScanContainer.dN == unNF)
			{
				(*ppSGetPathInfo)->cStep = 2;
			}
			else
			{
				(*ppSGetPathInfo)->cCommandSegmentCnt = 0;
				(*ppSGetPathInfo)->cPart = PART_1;
			}
			break;
		}
		
		default:
			break;
		}
		break;
	}
	
	//2.轨迹链表已生成
	case 2:
	{
		//1.输出
		//1.1输出pSPathList
		*ppSPathList = (*ppSGetPathInfo)->pSPathList;
		//1.2输出pSTargetTextInfo
		*pSTargetTextInfo = (*ppSGetPathInfo)->SGetTextStateInfo;

		//注意！！！轨迹链表的节点内存不需要释放，此内存后续被pSPathList维护
		
		//2.释放(*ppSGetPathInfo)
		freeGCode(*ppSGetPathInfo);

		(*ppSGetPathInfo) = NULL;

		return 1;
		break;
	}

	default:
		break;
	}

	return 0;

}

/*************************************************
  函数名称：
  函数功能  生成轨迹链表
  输入参数：ucMod				模式（检查/赋值）
  输出参数：
  返 回 值：1	已获取所有轨迹
			0	轨迹未获取完成
			-1	错误
  备    注：1.
**************************************************/
void removeEntryAndExitPath(SList* pSPathList,int8 cAxis, bool bRemoveEntry, bool bRemoveExit)
{		
	SLine* pEntity;
	//进刀轴
	int8 cAxis1 = cAxis;
	//非进刀轴
	int8 cAxis2 = cAxis == AXIS_X ? AXIS_Z : AXIS_X;

	//去除进刀段
	if (bRemoveEntry)
	{
		int16 sCurrPos = pSPathList->cSize - 1;
		while (sCurrPos >= 0)
		{
			pEntity = FindNodeData(pSPathList, sCurrPos);
			if ((pEntity->EntityType == L1NE || pEntity->EntityType == OTHERTYPE))
			{
				//判断是否进刀（仅进刀轴发生移动）
				if (!isZero(pEntity->SStartPoint.dArray[cAxis1] - pEntity->SEndPoint.dArray[cAxis1]) &&
					isZero(pEntity->SStartPoint.dArray[cAxis2] - pEntity->SEndPoint.dArray[cAxis2]))
				{
					removeNode(pSPathList, sCurrPos, FALSE);
				}
				else
					break;
			}
			sCurrPos--;
		}
	}

	//去除退刀段
	if (bRemoveExit)
	{
		int16 sCurrPos = 0;
		while (sCurrPos <= pSPathList->cSize - 1)
		{
			pEntity = FindNodeData(pSPathList, sCurrPos);
			if ((pEntity->EntityType == L1NE || pEntity->EntityType == OTHERTYPE))
			{
				//判断是否进刀（仅进刀轴发生移动）
				if (!isZero(pEntity->SStartPoint.dArray[cAxis1] - pEntity->SEndPoint.dArray[cAxis1]) &&
					isZero(pEntity->SStartPoint.dArray[cAxis2] - pEntity->SEndPoint.dArray[cAxis2]))
				{
					removeNode(pSPathList, sCurrPos, FALSE);
				}
				else
					break;
			}
			else
			{
				sCurrPos++;
			}

		}
	}

}

void offsetEntity(enum ENTITY* pEntity, double dDeltaX, double dDeltaZ)
{
	switch (*pEntity)
	{
	case OTHERTYPE:case L1NE:case DOT:
		SLine* pSLine = (SLine*)pEntity;
		pSLine->SStartPoint.dArray[AXIS_X] += dDeltaX;
		pSLine->SStartPoint.dArray[AXIS_Z] += dDeltaZ;
		pSLine->SEndPoint.dArray[AXIS_X] += dDeltaX;
		pSLine->SEndPoint.dArray[AXIS_Z] += dDeltaZ;

		break;
	case ARC:
		SArc* pSArc = (SArc*)pEntity;
		pSArc->SStartPoint.dArray[AXIS_X] += dDeltaX;
		pSArc->SStartPoint.dArray[AXIS_Z] += dDeltaZ;
		pSArc->SEndPoint.dArray[AXIS_X] += dDeltaX;
		pSArc->SEndPoint.dArray[AXIS_Z] += dDeltaZ;
		pSArc->SCenterPoint.dArray[AXIS_X] += dDeltaX;
		pSArc->SCenterPoint.dArray[AXIS_Z] += dDeltaZ;
		break;
	default:
		break;
	}
}
//在原始轨迹链表上修改
void offsetPathList(SList** ppSPathList, double dDeltaX, double dDeltaZ)
{
	for (int i = 0; i < (*ppSPathList)->cSize; i++)
	{
		enum ENTITY* pEntity;
		pEntity = FindNodeData(*ppSPathList, i);
		offsetEntity(pEntity, dDeltaX, dDeltaZ);
	}
}
//不修改原始轨迹链表
int8 offsetPathList_2(SList* pSPathList, SList** ppSPathList_2, double dDeltaX, double dDeltaZ)
{
	SList* pSNewPathList;
	pSNewPathList = initList(ORAM_NUM353, ORAM_NUM354);
    if (pSNewPathList == NULL)
    {
        *ppSPathList_2 = NULL;
        return -1;
    }
	int16 sCurrPos;
	sCurrPos = pSPathList->cSize - 1;
	//复制轨迹中的实体
	while (sCurrPos >= 0)
	{
		//复制当前实体节点
		SLine* pEntity;
		SLine* pNewEntity = NULL;
		pEntity = FindNodeData(pSPathList, sCurrPos);
		switch (pEntity->EntityType)
		{
		case OTHERTYPE:case L1NE:case DOT:
		{	
			pNewEntity = mallocGCode(sizeof(SLine), ORAM_NUM355);
		     if (pNewEntity == NULL)
			{
		    	 freeList(&pSNewPathList);
				*ppSPathList_2 = NULL;
				return -1;
			}
			*(SLine*)pNewEntity = *(SLine*)pEntity;
			break;
		}
		case ARC:
		{			
			pNewEntity = mallocGCode(sizeof(SArc), ORAM_NUM356);
		    if (pNewEntity == NULL)
			{
		    	freeList(&pSNewPathList);
				*ppSPathList_2 = NULL;
				return -1;
			}
			*(SArc*)pNewEntity = *(SArc*)pEntity;
			break;
		}
		default:
			break;
		}
		
		//偏移
		offsetEntity(pNewEntity, dDeltaX, dDeltaZ);

		//将新实体插入新轨迹链表
		if(insertList(pSNewPathList, 0, pNewEntity, ORAM_NUM354) == FALSE )
	    {
			     freeGCode(pNewEntity);       // 释放本条未入链的实体
	            freeList(&pSNewPathList); // 销毁临时链表
	            *ppSPathList_2 = NULL;
	            return -1;
	     }
		sCurrPos--;
	}

	*ppSPathList_2 = pSNewPathList;
	return 0;
}
//镜像实体
void mirrorEntity(enum ENTITY* pEntity)
{
	switch (*pEntity)
	{
	case OTHERTYPE:case L1NE:case DOT:
		SLine* pSLine = (SLine*)pEntity;
		pSLine->SStartPoint.dArray[AXIS_X] *= -1;
		pSLine->SEndPoint.dArray[AXIS_X] *= -1;

		break;
	case ARC:
		SArc* pSArc = (SArc*)pEntity;
		pSArc->SStartPoint.dArray[AXIS_X] *= -1;
		pSArc->SEndPoint.dArray[AXIS_X] *= -1;
		pSArc->SCenterPoint.dArray[AXIS_X] *= -1;
		pSArc->cArcDir *= -1;

		break;
	default:
		break;
	}
}
//镜像轨迹链表
void mirrorPathList(SList** ppSPathList)
{
	for (int i = 0; i < (*ppSPathList)->cSize; i++)
	{
		enum ENTITY* pEntity;
		pEntity = FindNodeData(*ppSPathList, i);
		mirrorEntity(pEntity);
		
	}
}


/*************************************************
  函数名称：void getArcMonotonicity(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, SPointDouble* pSCenterPoint, int8  cArcDir, bool* pXIsMono, bool* pZIsMono, int8* pXCurrMono, int8* pZCurrMono)
  函数功能：判断是否为基础轨迹指令
  输入参数：sCommandVal
  输出参数：
  返 回 值：
  备    注：目前基础轨迹指令有：G0,G1,G2,G3
**************************************************/
bool isBasicMotionCommand(int16 sCommandVal)
{
	if ((sCommandVal == G00 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//快速运动
		(sCommandVal == G01 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//直线插补
		(sCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//顺时针圆弧
		(sCommandVal == G03 * pow(10, GCOMMAND_DECIMAL_PLACE)))		//逆时针圆弧
	{
		return TRUE;
	}
	return FALSE;
}

bool isAllowGCommandInPath(SScan* pSScanContainer)
{
	if (pSScanContainer->SScanModalInfo.pSGaRegulation != NULL)
	{
		if (!((pSScanContainer->SScanModalInfo.pSGaRegulation->nCommandVal == G00 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//快速运动
			(pSScanContainer->SScanModalInfo.pSGaRegulation->nCommandVal == G01 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//直线插补
			(pSScanContainer->SScanModalInfo.pSGaRegulation->nCommandVal == G02 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//顺时针圆弧
			(pSScanContainer->SScanModalInfo.pSGaRegulation->nCommandVal == G03 * pow(10, GCOMMAND_DECIMAL_PLACE))))		//逆时针圆弧
		{
			return FALSE;
		}
	}
	if (pSScanContainer->SScanModalInfo.pSGbRegulation != NULL)
	{
		return FALSE;
	}	
	if (pSScanContainer->SScanModalInfo.pSGcRegulation != NULL)
	{
		if (!((pSScanContainer->SScanModalInfo.pSGcRegulation->nCommandVal == G17 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//XY平面
			(pSScanContainer->SScanModalInfo.pSGcRegulation->nCommandVal == G18 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//ZX平面
			(pSScanContainer->SScanModalInfo.pSGcRegulation->nCommandVal == G19 * pow(10, GCOMMAND_DECIMAL_PLACE))))		//YZ平面
		{
			return FALSE;
		}
	}	
	if (pSScanContainer->SScanModalInfo.pSGdRegulation != NULL)
	{
		if (!((pSScanContainer->SScanModalInfo.pSGdRegulation->nCommandVal == G40 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//取消刀补
			(pSScanContainer->SScanModalInfo.pSGdRegulation->nCommandVal == G41 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//左刀补
			(pSScanContainer->SScanModalInfo.pSGdRegulation->nCommandVal == G42 * pow(10, GCOMMAND_DECIMAL_PLACE))))		//右刀补
		{
			return FALSE;
		}
	}	
	if (pSScanContainer->SScanModalInfo.pSGeRegulation != NULL)
	{
		return FALSE;
	}
	if (pSScanContainer->SScanModalInfo.pSGfRegulation != NULL)
	{
		if (!((pSScanContainer->SScanModalInfo.pSGfRegulation->nCommandVal == G96 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//恒线速
			(pSScanContainer->SScanModalInfo.pSGfRegulation->nCommandVal == G97 * pow(10, GCOMMAND_DECIMAL_PLACE))))		//恒转速
		{
			return FALSE;
		}
	}
	if (pSScanContainer->SScanModalInfo.pSGgRegulation != NULL)
	{
		if (!((pSScanContainer->SScanModalInfo.pSGgRegulation->nCommandVal == G98 * pow(10, GCOMMAND_DECIMAL_PLACE)) ||	//每分进给
			(pSScanContainer->SScanModalInfo.pSGgRegulation->nCommandVal == G99 * pow(10, GCOMMAND_DECIMAL_PLACE))))		//每转进给
		{
			return FALSE;
		}

	}
	if (pSScanContainer->SScanModalInfo.pSGhRegulation != NULL)
	{
		return FALSE;
	}
	if (pSScanContainer->SScanModalInfo.pSGzRegulation != NULL)
	{
		if (!((pSScanContainer->SScanModalInfo.pSGzRegulation->nCommandVal == G04 * pow(10, GCOMMAND_DECIMAL_PLACE))))	//延时
		{
			return FALSE;
		}
	}
	if (pSScanContainer->SScanNonModalInfo.pSGxRegulation != NULL)
	{
		return FALSE;
	}

	return TRUE;
}

/*************************************************
  函数名称：void getArcMonotonicity(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, SPointDouble* pSCenterPoint, int8  cArcDir, bool* pXIsMono, bool* pZIsMono, int8* pXCurrMono, int8* pZCurrMono)
  函数功能  获取圆弧的单调性
  输入参数：pSStartPoint			起点坐标
			pSEndPoint				终点坐标
			pSCenterPoint			圆心坐标
			cArcDir					圆弧方向
			pXIsMono				X轴是否单调（用于输出）
			pZIsMono				Z轴是否单调（用于输出）
			pXCurrMono				X轴单调值（用于输出）
			pZCurrMono				Z轴单调值（用于输出）
  输出参数：
  返 回 值：
  备    注：0.本函数未对圆弧参数进行检查!!!
			1.若单调增，则单调值为1
			2.若单调减，则单调值为-1
**************************************************/
void getArcMonotonicity(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, SPointDouble* pSCenterPoint, int8 cArcDir, bool* pXIsMono, bool* pZIsMono, int8* pXCurrMono, int8* pZCurrMono)
{
	// 计算起点和终点相对于圆心的相对位置
	double xStartRel = pSStartPoint->dArray[AXIS_X] - pSCenterPoint->dArray[AXIS_X];
	double zStartRel = pSStartPoint->dArray[AXIS_Z] - pSCenterPoint->dArray[AXIS_Z];
	double xEndRel = pSEndPoint->dArray[AXIS_X] - pSCenterPoint->dArray[AXIS_X];
	double zEndRel = pSEndPoint->dArray[AXIS_Z] - pSCenterPoint->dArray[AXIS_Z];

	//0.初始化
	*pXIsMono = *pZIsMono = FALSE;
	*pXCurrMono = *pZCurrMono = 0;

	//1.起点终点重合，则为点or整圆
	if (isZero(xStartRel - xEndRel) && isZero(zStartRel - zEndRel))  //起点终点重合
							
	{	
		//1.1起点圆心重合，则为点
		if (isZero(xStartRel) && isZero(zStartRel))
		{
			*pXIsMono = *pZIsMono = TRUE;	//XZ有单调性
			*pXCurrMono = *pZCurrMono = 0;	//XZ单调值为0
		}
		//1.2起点圆心不重合，则为整圆
		else
		{
			*pXIsMono = *pZIsMono = FALSE;	//XZ有单调性
			*pXCurrMono = *pZCurrMono = 0;	//XZ单调值为0
		}
	}
	//2.起点终点不重合，则为圆弧
	else
	{
		//2.1若Z1Z2>=0，则起点终点位于X轴的同一侧，X方向可能存在单调性
		if (zStartRel * zEndRel >= 0)
		{
			//2.1.1起点X大于终点X
			if (xStartRel > xEndRel)
			{
				if (isZero(zStartRel) && isZero(zEndRel))
				{
					//情况1：X轴半圆 & 起点X大于终点X & 顺时针 -> X方向单调减
					if (cArcDir == ARC_CW)
					{
						*pXIsMono = TRUE;
						*pXCurrMono = -1;
					}
					//情况2：X轴半圆 & 起点X大于终点X & 逆时针 -> X方向单调减
					else
					{
						*pXIsMono = TRUE;
						*pXCurrMono = -1;
					}
				}
				else
				{			
					if (zStartRel > 0 || zEndRel > 0)
					{							
						//情况3：X轴上非半圆 & 起点X大于终点X & 顺时针 -> X方向单调减
						if (cArcDir == ARC_CW)
						{
							*pXIsMono = TRUE;
							*pXCurrMono = -1;
						}
						//情况4：X轴上非半圆 & 起点X大于终点X & 逆时针 -> X方向无单调
						else
						{
							*pXIsMono = FALSE;
							*pXCurrMono = 0;
						}
					}
					else
					{
						//情况5：X轴下非半圆 & 起点X大于终点X & 顺时针 -> X方向无单调
						if (cArcDir == ARC_CW)
						{
							*pXIsMono = FALSE;
							*pXCurrMono = 0;
						}
						//情况6：X轴上非半圆 & 起点X大于终点X & 逆时针 -> X方向单调减
						else
						{
							*pXIsMono = TRUE;
							*pXCurrMono = -1;
						}
					}

				}

			}
			//2.1.2起点X小于终点X
			else
			{
				if (isZero(zStartRel) && isZero(zEndRel))
				{
					//情况7：X轴半圆 & 起点X小于终点X & 顺时针 -> X方向单调增
					if (cArcDir == ARC_CW)
					{
						*pXIsMono = TRUE;
						*pXCurrMono = 1;
					}
					//情况8：X轴半圆 & 起点X小于终点X & 逆时针 -> X方向单调增
					else
					{
						*pXIsMono = TRUE;
						*pXCurrMono = 1;
					}
				}
				else
				{
					if (zStartRel > 0 || zEndRel > 0)
					{
						//情况9：X轴上非半圆 & 起点X小于终点X & 顺时针 -> X方向无单调
						if (cArcDir == ARC_CW)
						{
							*pXIsMono = FALSE;
							*pXCurrMono = 0;
						}
						//情况10：X轴上非半圆 & 起点X小于终点X & 逆时针 -> X方向单调增
						else
						{
							*pXIsMono = TRUE;
							*pXCurrMono = 1;
						}
					}
					else
					{
						//情况11：X轴下非半圆 & 起点X小于终点X & 顺时针 -> X方向单调增
						if (cArcDir == ARC_CW)
						{
							*pXIsMono = TRUE;
							*pXCurrMono = 1;
						}
						//情况12：X轴下非半圆 & 起点X小于终点X & 逆时针 -> X方向无单调
						else
						{
							*pXIsMono = FALSE;
							*pXCurrMono = 0;
						}
					}

				}
			}

		}
		
		//2.2若X1X2>=0，则起点终点位于Z轴的同一侧，Z方向可能存在单调性
		if (xStartRel * xEndRel >= 0)
		{
			if (zStartRel > zEndRel)
			{
				if (isZero(xStartRel) && isZero(xEndRel))
				{
					//情况13：Z轴半圆 & 起点Z大于终点Z & 顺时针 -> Z方向单调减
					if (cArcDir == ARC_CW)
					{
						*pZIsMono = TRUE;
						*pZCurrMono = -1;
					}
					//情况14：Z轴半圆 & 起点Z大于终点Z & 逆时针 -> Z方向单调减
					else
					{
						*pZIsMono = TRUE;
						*pZCurrMono = -1;
					}
				}
				else
				{
					if (xStartRel > 0 || xEndRel > 0)
					{
						//情况15：Z轴右非半圆 & 起点Z大于终点Z & 顺时针 -> Z方向无单调
						if (cArcDir == ARC_CW)
						{
							*pZIsMono = FALSE;
							*pZCurrMono = 0;
						}
						//情况16：Z轴右非半圆 & 起点Z大于终点Z & 逆时针 -> Z方向单调减
						else
						{
							*pZIsMono = TRUE;
							*pZCurrMono = -1;
						}
					}
					else
					{
						//情况17：Z轴左非半圆 & 起点Z大于终点Z & 顺时针 -> Z方向单调减
						if (cArcDir == ARC_CW)
						{
							*pZIsMono = TRUE;
							*pZCurrMono = -1;
						}
						//情况18：Z轴左非半圆 & 起点Z大于终点Z & 逆时针 -> Z方向无单调
						else
						{
							*pZIsMono = FALSE;
							*pZCurrMono = 0;
						}
					}

				}

			}
			//2.1.2起点X小于终点X
			else
			{
				if (isZero(xStartRel) && isZero(xEndRel))
				{
					//情况19：Z轴半圆 & 起点Z小于终点Z & 顺时针 -> Z方向单调增
					if (cArcDir == ARC_CW)
					{
						*pZIsMono = TRUE;
						*pZCurrMono = 1;
					}
					//情况20：Z轴半圆 & 起点Z小于终点Z & 逆时针 -> Z方向单调增
					else
					{
						*pZIsMono = TRUE;
						*pZCurrMono = 1;
					}
				}
				else
				{
					if (xStartRel > 0 || xEndRel > 0)
					{
						//情况21：Z轴右非半圆 & 起点Z小于终点Z & 顺时针 -> Z方向单调增
						if (cArcDir == ARC_CW)
						{
							*pZIsMono = TRUE;
							*pZCurrMono = 1;
						}
						//情况22：Z轴右非半圆 & 起点Z小于终点Z & 逆时针 -> Z方向无单调
						else
						{
							*pZIsMono = FALSE;
							*pZCurrMono = 0;
						}
					}
					else
					{
						//情况23：Z轴左非半圆 & 起点Z小于终点Z & 顺时针 -> Z方向无单调
						if (cArcDir == ARC_CW)
						{
							*pZIsMono = FALSE;
							*pZCurrMono = 0;	
						}
						//情况24：Z轴左非半圆 & 起点Z小于终点Z & 逆时针 -> Z方向单调增
						else
						{
							*pZIsMono = TRUE;
							*pZCurrMono = 1;
						}
					}

				}
			}

		}
	}
}

/*************************************************
  函数名称：int8 isArcGroove(SPointDouble SStart, SPointDouble SEnd, SPointDouble SCenter,int8 cGa)
  函数功能: 圆弧是否形成槽，用于判断圆弧跨越象限
  输入参数：SStartqi起点，SEnd终点，SCenter圆心，cGa组， int8 cFlag：G71orG72
  输出参数：
  返 回 值：-1: 不符合
			1 : 跨一二象限（G71）、跨一四象限（G72）
			2 : 跨三四象限（G71）、跨二三象限（G72）
			#define ARC_CW 1          //顺时针圆弧
         #define ARC_ANTI_CW -1    //逆时针圆弧
  备    注：
**************************************************/
int8 isArcIsGroove(SPointDouble* SStartPt, SPointDouble* SEndPt, SPointDouble* SCenterPt, int8  cArcDir, int8 cFlag)
{
	bool bResultX1;//true：起点和圆心的x指相同，即起点和圆心在同一水平线上
	bool bResultX2;////true：终点点和圆心的x指相同，即终点点和圆心在同一水平线上
	SPoint SStart;
	SPoint SEnd;
	SPoint SCenter;
	double dTemp;
	if (cFlag == G71)
	{
		SStart.dX = SStartPt->dArray[AXIS_X];
		SStart.dZ = SStartPt->dArray[AXIS_Z];
		SEnd.dX = SEndPt->dArray[AXIS_X];
		SEnd.dZ = SEndPt->dArray[AXIS_Z];
		SCenter.dX = SCenterPt->dArray[AXIS_X];
		SCenter.dZ = SCenterPt->dArray[AXIS_Z];
	}
	else
	{   //G72 时，因为代码是按G71写的，将坐标调转，重用代码
		SStart.dX = SStartPt->dArray[AXIS_Z];
		SStart.dZ = SStartPt->dArray[AXIS_X];
		SEnd.dX = SEndPt->dArray[AXIS_Z];
		SEnd.dZ = SEndPt->dArray[AXIS_X];
		SCenter.dX = SCenterPt->dArray[AXIS_Z];
		SCenter.dZ = SCenterPt->dArray[AXIS_X];
		cArcDir = (cArcDir == ARC_CW) ? ARC_ANTI_CW : ARC_CW;//调转后，顺逆也会互换
	}

	dTemp = fabs(SEnd.dX - SCenter.dX);
	if (dTemp < 0.0002)
	{
		bResultX2 = TRUE;
	}
	else
	{
		bResultX2 = FALSE;
	}
	dTemp = fabs(SStart.dX - SCenter.dX);
	if (dTemp < 0.0002)
	{
		bResultX1 = TRUE;
	}
	else
	{
		bResultX1 = FALSE;
	}
	if (bResultX1)//起点在Z轴上，要根据顺逆判断  （起点和圆心在同一水平线）
	{
		if (SStart.dZ > SCenter.dZ && cArcDir == ARC_ANTI_CW )
		{
			if ((SEnd.dX > SCenter.dX && SEnd.dZ < SCenter.dZ) || (bResultX2 && SEnd.dZ < SCenter.dZ))                               //起点第一象限，终点第二象限
			{
				return 1;
			}
		}
		else if (SStart.dZ < SCenter.dZ && cArcDir == ARC_CW )
		{
			if ((SEnd.dX > SCenter.dX && SEnd.dZ > SCenter.dZ) || (bResultX2 && SEnd.dZ > SCenter.dZ))                               //起点第二象限，终点第一象限
			{
				return 1;
			}
		}
		else if (SStart.dZ < SCenter.dZ && cArcDir == ARC_ANTI_CW)
		{
			if ((SEnd.dX < SCenter.dX && SEnd.dZ > SCenter.dZ) || (bResultX2 && SEnd.dZ > SCenter.dZ))                               //起点第三象限，终点第四象限
			{
				return 2;
			}
		}
		else if (SStart.dZ > SCenter.dZ && cArcDir == ARC_CW)
		{
			if ((SEnd.dX < SCenter.dX && SEnd.dZ < SCenter.dZ) || (bResultX2 && SEnd.dZ < SCenter.dZ))                                //起点第四象限，终点第三象限
			{
				return 2;
			}
		}
	}
	else if (SStart.dX > SCenter.dX)
	{
		if (SStart.dZ > SCenter.dZ && ((SEnd.dX > SCenter.dX && SEnd.dZ < SCenter.dZ) || (bResultX2 && SEnd.dZ < SCenter.dZ)))        //起点第一象限，终点第二象限
		{
			return 1;
		}
		else if (SStart.dZ < SCenter.dZ && ((SEnd.dX > SCenter.dX && SEnd.dZ > SCenter.dZ) || (bResultX2 && SEnd.dZ > SCenter.dZ)))   //起点第二象限，终点第一象限
		{
			return 1;
		}
	}
	else //if (SStart.dX < SCenter.dX) 
	{
		if (SStart.dZ < SCenter.dZ && ((SEnd.dX < SCenter.dX && SEnd.dZ > SCenter.dZ) || (bResultX2 && SEnd.dZ > SCenter.dZ)))        //起点第三象限，终点第四象限
		{
			return 2;
		}
		else if (SStart.dZ > SCenter.dZ && ((SEnd.dX < SCenter.dX && SEnd.dZ < SCenter.dZ) || (bResultX2 && SEnd.dZ < SCenter.dZ)))   //起点第四象限，终点第三象限
		{
			return 2;
		}

	}


	return -1;
}

/*************************************************
  函数名称：bool divideGroove()
  函数功能  将轨迹链表按照输入的XorZ单调变化来生成槽链表
  输入参数：SList**pathList 轨迹链表,
             SList**pGrooveList 槽链表,
			 int8 cMonotonicity 输入的XorZ单调变化
			 cAxisXorZ		根据XorZ轴单调性划分槽
  输出参数：-1 错误
			0 未完成
			1 完成
  返 回 值：
  备    注：
**************************************************/
int8 divideGroove(SList** ppPathList, SList** ppGrooveList, int8 cAxisXorZ, int8 cFeedDir)
{
	if (ppPathList == NULL || ppGrooveList == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;

	}
	SLine* pCurrEntityData = NULL;      //轨迹链表的当前数据节点(用SLine接所有实体数据)
	int8 cEntityMonotonicity;

	SNode* pPathListNewTailNode = NULL;		//轨迹链表新的尾节点
	SNode* pGrooveListHead = NULL;			//槽链表头节点

	//0.新增槽数据节点
	//0.1开辟槽数据结构体的内存
	SGrooveData* pSNewGrooveData = mallocGCode(sizeof(SGrooveData), ORAM_NUM359);	//内存暂未编号
	if (pSNewGrooveData == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM302);
		return -1;
	}
	//0.2初始化槽数据结构体
	memset(pSNewGrooveData, 0, sizeof(SGrooveData));
	pSNewGrooveData->ucGrooveNum = getListSize(*ppGrooveList) + 1;			//初始化槽号
	pSNewGrooveData->pSGroovePathList= initList(ORAM_NUM360, ORAM_NUM361);  //初始化槽轨迹链表
	if (pSNewGrooveData->pSGroovePathList == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM302);
		return -1;
	}
	//0.3插入槽链表
	if (insertList(*ppGrooveList, 0, pSNewGrooveData, ORAM_NUM358) == FALSE)
	{
		freeList(pSNewGrooveData->pSGroovePathList);
		freeGCode(pSNewGrooveData);
		parseGCodeErrorB(0, GCODE_ERROR_NUM302);
		return -1;
	}
	
	//1.获取轨迹链表尾的位置
	int16 sCurrPos;
	sCurrPos = getListSize(*ppPathList) - 1;

	//2.记录槽起始边顶点坐标
	pCurrEntityData = (SLine*)FindNodeData(*ppPathList, sCurrPos);
	pSNewGrooveData->SPointA.dX = pCurrEntityData->SStartPoint.dArray[AXIS_X];
	pSNewGrooveData->SPointA.dZ = pCurrEntityData->SStartPoint.dArray[AXIS_Z];

	//3.找到槽所有起始边实体
	while (sCurrPos >= 0)
	{
		//获取轨迹链表中sCurrPos节点的实体数据
		pCurrEntityData = (SLine*)FindNodeData(*ppPathList, sCurrPos);

		//获取实体的单调性
		cEntityMonotonicity = getPathDataMonotonicity((enum ENTITY*)pCurrEntityData, cAxisXorZ);

		//判断单调性是否改变
		if (cEntityMonotonicity * cFeedDir < 0)
		{
			break;
		}

		pSNewGrooveData->cTotalStartLineEntityNum++;
		sCurrPos--;
	}

	//4.记录槽底点坐标
	pSNewGrooveData->SPointC.dX = pCurrEntityData->SStartPoint.dArray[AXIS_X];
	pSNewGrooveData->SPointC.dZ = pCurrEntityData->SStartPoint.dArray[AXIS_Z];

	//5.找到槽所有终止边实体
	while (sCurrPos >= 0)
	{
		//获取轨迹链表中sCurrPos节点的实体数据
		pCurrEntityData = (enum ENTITY*)FindNodeData(*ppPathList, sCurrPos);

		//获取实体的单调性
		cEntityMonotonicity = getPathDataMonotonicity(pCurrEntityData, cAxisXorZ);

		//判断单调性是否改变
		if (cEntityMonotonicity * -cFeedDir < 0)
		{
			break;
		}

		pSNewGrooveData->cTotalEndLineEntityNum++;
		sCurrPos--;
	}

	//6.记录槽终止边顶点坐标
	pSNewGrooveData->SPointB.dX = sCurrPos < 0 ? pCurrEntityData->SEndPoint.dArray[AXIS_X] : pCurrEntityData->SStartPoint.dArray[AXIS_X];
	pSNewGrooveData->SPointB.dZ = sCurrPos < 0 ? pCurrEntityData->SEndPoint.dArray[AXIS_Z] : pCurrEntityData->SStartPoint.dArray[AXIS_Z];

	//7.从轨迹链表中取下槽轨迹链表
	pPathListNewTailNode = sCurrPos < 0 ? (*ppPathList)->pHead : FindNode(*ppPathList, sCurrPos);
	pGrooveListHead = pPathListNewTailNode->pNextNode;
	pSNewGrooveData->pSGroovePathList->pHead->pNextNode = pGrooveListHead;
	pSNewGrooveData->pSGroovePathList->cSize = pSNewGrooveData->cTotalStartLineEntityNum + pSNewGrooveData->cTotalEndLineEntityNum;
	pPathListNewTailNode->pNextNode = NULL;
	(*ppPathList)->cSize -= pSNewGrooveData->cTotalStartLineEntityNum + pSNewGrooveData->cTotalEndLineEntityNum;

	//8.判断所有槽是否划分完成
	if (sCurrPos < 0)
	{
		return 1;
	}
	return 0;
}

/*************************************************
  函数名称：int8 getPathDataMonotonicity(enum ENTITY* pSPath, int8 cAxisXorZ)
  函数功能  根据输入是X轴还是Z轴来获取轨迹数据节点中相应的单调性
  输入参数：pSLineInfo		直线段信息
			pSGCodeBufUnit	G代码缓冲区单元

  输出参数：-2 出错
            -1 递减
			0  没有单调性
			1 递增
  返 回 值：
  备    注：
**************************************************/
int8 getPathDataMonotonicity(enum ENTITY* pSPathData, int8 cAxisXorZ)
{
	if (pSPathData == NULL )
	{
		//报错
		return -2;
	}
	int8 Monotonicity = 0;//求出的单调性，作为返回值
	double res = 0;
	//获取要求单调性变化发的轴
	int8 AXIS = (cAxisXorZ == AXIS_X) ? AXIS_X : AXIS_Z;

	switch (*pSPathData)
	{
	case L1NE:case OTHERTYPE:case DOT:
	{
		SLine* pSLine = (SLine*)pSPathData;
		res = pSLine->SEndPoint.dArray[AXIS] - pSLine->SStartPoint.dArray[AXIS];
		break;
	}
	case ARC:
	{
		SArc* pArc = (SArc*)pSPathData;
		res = pArc->SEndPoint.dArray[AXIS] - pArc->SStartPoint.dArray[AXIS];
		break;
	}
	default:
		break;
	}
	if (isZero(res))
	{
		Monotonicity = 0;
	}
	else if (res > 0)
	{
		Monotonicity = 1;
	}
	else
	{
		Monotonicity = -1;
	}

	return Monotonicity;

}

//打印轨迹链表的轨迹点
void printDataG71G72I(void* data)
{
	if (data == NULL)
	{
		return;
	}

	enum ENTITY* pSPathData = (enum ENTITY*)data;
	switch (*pSPathData)
	{
	case L1NE:case OTHERTYPE:case DOT:
	{
		SLine* pSLine = (SLine*)pSPathData;
		printf("{ 实体类型%d,起点:{%f,%f},终点:{%f,%f} }\n", pSLine->EntityType, pSLine->SStartPoint.dArray[AXIS_X], pSLine->SStartPoint.dArray[AXIS_Z],
			pSLine->SEndPoint.dArray[AXIS_X], pSLine->SEndPoint.dArray[AXIS_Z]);
		break;
	}
	case ARC:
	{
		SArc* pArc = (SArc*)pSPathData;
		printf("{ 实体类型%d,起点:{%f,%f},终点:{%f,%f},圆心{%f,%f},半径{%f}}\n", pArc->EntityType, pArc->SStartPoint.dArray[AXIS_X], pArc->SStartPoint.dArray[AXIS_Z],
			pArc->SEndPoint.dArray[AXIS_X], pArc->SEndPoint.dArray[AXIS_Z], pArc->SCenterPoint.dArray[AXIS_X], pArc->SCenterPoint.dArray[AXIS_Z], pArc->dRadius);
		break;
		break;
	}
	default:
		break;
	}
	

}

//
/*************************************************
  函数名称：void printDataG71G72II(void* data)
  函数功能  打印槽轨迹点
  输入参数：void* data	槽轨迹链表指针

  输出参数：
  返 回 值：
  备    注：  void* data	为槽轨迹链表指针
**************************************************/
void printDataG71G72II(void* data)
{
	if (data == NULL)
	{
		return;
	}

	SList* plist = (SList*)data;//获取槽轨迹链表地址
	SNode* pCurr = plist->pHead->pNextNode;//指向第一个节点
	while (1)
	{
		//全部链表节点打印完毕，退出函数
		if (pCurr == NULL)
		{
			return;
		}
		//打印槽轨迹链表数据
		enum ENTITY* pSPathData = (enum ENTITY*)pCurr->pData;
		switch (*pSPathData)
		{
		case L1NE:case OTHERTYPE:case DOT:
		{
			SLine* pSLine = (SLine*)pSPathData;
			printf("{ 实体类型%d,起点:{%f,%f},终点:{%f,%f} }\n", pSLine->EntityType, pSLine->SStartPoint.dArray[AXIS_X], pSLine->SStartPoint.dArray[AXIS_Z],
				pSLine->SEndPoint.dArray[AXIS_X], pSLine->SEndPoint.dArray[AXIS_Z]);
			break;
		}
		//圆
		case ARC:
		{
			SArc* pArc = (SArc*)pSPathData;
			printf("{ 实体类型%d,起点:{%f,%f},终点:{%f,%f},圆心{%f,%f},半径{%f}}\n", pArc->EntityType, pArc->SStartPoint.dArray[AXIS_X], pArc->SStartPoint.dArray[AXIS_Z],
				pArc->SEndPoint.dArray[AXIS_X], pArc->SEndPoint.dArray[AXIS_Z], pArc->SCenterPoint.dArray[AXIS_X], pArc->SCenterPoint.dArray[AXIS_Z],pArc->dRadius);
			break;
		}
		default:
			break;
		}
		//当前指针指向下一个节点地址
		pCurr = pCurr->pNextNode;
	}
	
}




/*************************************************
  函数名称：void printListII(SList* pList, PrintNode print)
  函数功能  打印槽链表中各个槽结构体的槽轨迹数据
  输入参数：SList* pList		槽链表地址
			 PrintNode print	函数指针，该函数定义了槽结构体中的槽轨迹链表的打印方式
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void printListII(SList* pList, PrintNode print)
{
	SNode* pCurr;  //当前节点
	SGrooveData* SG71GrooveData;

	if (pList == NULL)
	{
		return;
	}
	pCurr = pList->pHead->pNextNode;//指向第一个节点

	while (pCurr != NULL)
	{
		SG71GrooveData = (SGrooveData*)(pCurr->pData);//获取节点数据域地址，即槽结构体所在的内存地址
		//打印槽结构体中的数据
		printf("槽号:%d A点:%f,%f B点:%f,%f 槽底值:%f,%f 终止边实体数量:%d\n", SG71GrooveData->ucGrooveNum,
			SG71GrooveData->SPointA.dX, SG71GrooveData->SPointA.dZ,
			SG71GrooveData->SPointB.dX, SG71GrooveData->SPointB.dZ,
			SG71GrooveData->SPointC.dX, SG71GrooveData->SPointC.dZ,
			SG71GrooveData->cTotalEndLineEntityNum);
		//打印槽结构体中的槽轨迹链表数据
		print(SG71GrooveData->pSGroovePathList);

		printf("-------------------------------\n");
		pCurr = pCurr->pNextNode;
	}


	if (pCurr == NULL)
	{
		printf("\n");
	}

}


int8 getPathMachineDir(SList* pPathList, int8* cMachineXDir, int8* cMachineZDir)
{
	int16 sLastEntityPos;
	SLine* pSEntity = NULL;
	bool bXDir = FALSE;
	bool bZDir = FALSE;

	sLastEntityPos = getListSize(pPathList) - 1;
	while (sLastEntityPos > 0)
	{
		pSEntity = (SLine*)FindNodeData(pPathList, sLastEntityPos);
		//获取X加工方向
		if (bXDir == FALSE)
		{
			if (pSEntity->SEndPoint.dArray[AXIS_X] - pSEntity->SStartPoint.dArray[AXIS_X] > 0)
			{
				*cMachineXDir = 1;
				bXDir = TRUE;
			}
			else if(pSEntity->SEndPoint.dArray[AXIS_X] - pSEntity->SStartPoint.dArray[AXIS_X] < 0)
			{
				*cMachineXDir = -1;
				bXDir = TRUE;
			}
		}
		//获取Z加工方向
		if (bZDir == FALSE)
		{
			if (pSEntity->SEndPoint.dArray[AXIS_Z] - pSEntity->SStartPoint.dArray[AXIS_Z] > 0)
			{
				*cMachineZDir = 1;
				bZDir = TRUE;
			}
			else if (pSEntity->SEndPoint.dArray[AXIS_Z] - pSEntity->SStartPoint.dArray[AXIS_Z] < 0)
			{
				*cMachineZDir = -1;
				bZDir = TRUE;
			}
		}
		if (bXDir == TRUE && bZDir == TRUE)
		{
			return 1;
		}
		sLastEntityPos--;
	}
	return -1;
}

SGrooveData* getGrooveDataByGrooveNum(SList* pGrooveList, int16 cGrooveNum)
{
	if (pGrooveList == NULL || cGrooveNum <= 0 || pGrooveList->cSize < cGrooveNum)
	{
	    // **不要写回 cCurrPointGrooveNum**
	    // **不要继续取槽**
	    return 0; // 或错误码
	}

	int16 cNodePos;
	cNodePos = getListSize(pGrooveList) - cGrooveNum;
	SGrooveData* pGrooveData = FindNodeData(pGrooveList, cNodePos);
	return pGrooveData;
}


//槽底点
int16 findEntityPosInGroove(SGrooveData* pSGrooveData, bool bFeed, SPointDouble* pSTargetPoint, bool bStartLine)
{
	SLine* pSEntity = NULL;
	int16 cEntityPos;

	//1.进刀
	if (bFeed == TRUE)
	{
		//从槽底向起始边顶点搜索
		if (bStartLine == TRUE)
		{
			for (int i = 0; i < pSGrooveData->cTotalStartLineEntityNum; i++)
			{
				cEntityPos = pSGrooveData->cTotalEndLineEntityNum + i;
				pSEntity = (enum ENTITY*)FindNodeData(pSGrooveData->pSGroovePathList, cEntityPos);
				if (isBetweenA2B(pSTargetPoint->dArray[AXIS_X], pSEntity->SStartPoint.dArray[AXIS_X], pSEntity->SEndPoint.dArray[AXIS_X]) &&
					isBetweenA2B(pSTargetPoint->dArray[AXIS_Z], pSEntity->SStartPoint.dArray[AXIS_Z], pSEntity->SEndPoint.dArray[AXIS_Z]))
				{
					//找到返回节点Pos
					return cEntityPos;
				}
			}
		}
		//从槽底向终止边顶点搜索
		else
		{
			for (int i = 0; i < pSGrooveData->cTotalEndLineEntityNum; i++)
			{
				cEntityPos = pSGrooveData->cTotalEndLineEntityNum - 1 - i;
				pSEntity = (enum ENTITY*)FindNodeData(pSGrooveData->pSGroovePathList, cEntityPos);
				if (isBetweenA2B(pSTargetPoint->dArray[AXIS_X], pSEntity->SStartPoint.dArray[AXIS_X], pSEntity->SEndPoint.dArray[AXIS_X]) &&
					isBetweenA2B(pSTargetPoint->dArray[AXIS_Z], pSEntity->SStartPoint.dArray[AXIS_Z], pSEntity->SEndPoint.dArray[AXIS_Z]))
				{
					//找到返回节点Pos
					return cEntityPos;
				}
			}
		}
	}
	//2.退刀
	else if (bFeed == FALSE)
	{
		//从起始边顶点向槽底搜索
		if (bStartLine == 1)
		{
			for (int i = 0; i < pSGrooveData->cTotalStartLineEntityNum; i++)
			{
				cEntityPos = pSGrooveData->cTotalStartLineEntityNum + pSGrooveData->cTotalEndLineEntityNum - 1 - i;
				pSEntity = (enum ENTITY*)FindNodeData(pSGrooveData->pSGroovePathList, cEntityPos);
				if (isBetweenA2B(pSTargetPoint->dArray[AXIS_X], pSEntity->SStartPoint.dArray[AXIS_X], pSEntity->SEndPoint.dArray[AXIS_X]) &&
					isBetweenA2B(pSTargetPoint->dArray[AXIS_Z], pSEntity->SStartPoint.dArray[AXIS_Z], pSEntity->SEndPoint.dArray[AXIS_Z]))
				{
					//找到返回节点Pos
					return cEntityPos;
				}
			}
		}
		//从终止边顶点向槽底搜索
		else
		{
			for (int i = 0; i < pSGrooveData->cTotalEndLineEntityNum; i++)
			{
				cEntityPos = i;
				pSEntity = (enum ENTITY*)FindNodeData(pSGrooveData->pSGroovePathList, cEntityPos);
				if (isBetweenA2B(pSTargetPoint->dArray[AXIS_X], pSEntity->SStartPoint.dArray[AXIS_X], pSEntity->SEndPoint.dArray[AXIS_X]) &&
					isBetweenA2B(pSTargetPoint->dArray[AXIS_Z], pSEntity->SStartPoint.dArray[AXIS_Z], pSEntity->SEndPoint.dArray[AXIS_Z]))
				{
					//找到返回节点Pos
					return cEntityPos;
				}
			}

		}

	}

}

//实体各轴必须单调 返回-1未找到返回1找到
int8 findTargetPointInEntity(enum ENTITY* pEntity, int8 cAixsXorZ, double dXorZVal, double* pTargetVal)
{
	switch (*pEntity)
	{
	case L1NE:case OTHERTYPE:case DOT:
	{
		SLine* pSLine = (SLine*)pEntity;
		double dSX = pSLine->SStartPoint.dArray[AXIS_X];
		double dSZ = pSLine->SStartPoint.dArray[AXIS_Z];
		double dEX = pSLine->SEndPoint.dArray[AXIS_X];
		double dEZ = pSLine->SEndPoint.dArray[AXIS_Z];

		//根据X求Z
		if (cAixsXorZ == AXIS_X)
		{
			//已知X超出范围
			if (!isBetweenA2B(dXorZVal, dSX, dEX))
			{
				return -1;
			}
			////已知X对应多个Z（直线水平）
			//else if (isZero(dSZ, dEZ))
			//{

			//}
			else
			{
				*pTargetVal = dSZ + (dEZ - dSZ) * (dXorZVal - dSX) / (dEX - dSX);
			}
		}
		//根据Z求X
		else if (cAixsXorZ == AXIS_Z)
		{
			//已知Z超出范围
			if (!isBetweenA2B(dXorZVal, dSZ, dEZ))
			{
				return -1;
			}
			else
			{
				*pTargetVal = dSX + (dEX - dSX) * (dXorZVal - dSZ) / (dEZ - dSZ);
			}
		}
		break;
	}
	case ARC:
	{
		SArc* pSArc = (SArc*)pEntity;
		double dSX = pSArc->SStartPoint.dArray[AXIS_X];
		double dSZ = pSArc->SStartPoint.dArray[AXIS_Z];
		double dEX = pSArc->SEndPoint.dArray[AXIS_X];
		double dEZ = pSArc->SEndPoint.dArray[AXIS_Z];
		double dCX = pSArc->SCenterPoint.dArray[AXIS_X];
		double dCZ = pSArc->SCenterPoint.dArray[AXIS_Z];
		double dR = pSArc->dRadius;

		//根据X求Z
		if (cAixsXorZ == AXIS_X)
		{
			//已知X超出范围
			if (!isBetweenA2B(dXorZVal, dSX, dEX))
			{
				return -1;
			}
			else
			{
				double dTargetVal_1;
				double dTargetVal_2;
				dTargetVal_1 = dCZ + (sqrt(dR * dR - (dXorZVal - dCX) * (dXorZVal - dCX)));
				dTargetVal_2 = dCZ - (sqrt(dR * dR - (dXorZVal - dCX) * (dXorZVal - dCX)));

				*pTargetVal = isBetweenA2B(dTargetVal_1, dSZ, dEZ) ? dTargetVal_1 : dTargetVal_2;
			}
		}
		//根据Z求X
		else if (cAixsXorZ == AXIS_Z)
		{
			//已知Z超出范围
			if (!isBetweenA2B(dXorZVal, dSZ, dEZ))
			{
				return -1;
			}
			else
			{
				double dTargetVal_1;
				double dTargetVal_2;
				dTargetVal_1 = dCX + (sqrt(dR * dR - (dXorZVal - dCZ) * (dXorZVal - dCZ)));
				dTargetVal_2 = dCX - (sqrt(dR * dR - (dXorZVal - dCZ) * (dXorZVal - dCZ)));

				*pTargetVal = isBetweenA2B(dTargetVal_1, dSX, dEX) ? dTargetVal_1 : dTargetVal_2;
			}
		}
		break;
	}
	default:
		break;

	}
	return 1;
}
int8 findTargetPointInGroove(SGrooveData* pSGrooveData, bool bStartPath, int8 cAixsXorZ, double dXorZVal, double* pTargetVal)
{
	//起始边实体数量
	int16 cTotalStartLineEntityNum = pSGrooveData->cTotalStartLineEntityNum;
	//终止边实体数量
	int16 cTotalEndLineEntityNum = pSGrooveData->cTotalEndLineEntityNum;

	//1.槽起始边内，找dXorZVal对应的值（链表应该向槽底方向搜寻）
	if (bStartPath == 1)
	{
		
		for (int16 i = 0; i < cTotalStartLineEntityNum; i++)
		{
			//获取当前实体
			enum ENTITY* pEntity = (enum ENTITY*)FindNodeData(pSGrooveData->pSGroovePathList, cTotalEndLineEntityNum + i);

			//当前实体内根据X值找到TargetVal
			if (findTargetPointInEntity(pEntity, AXIS_X, dXorZVal, pTargetVal) > 0)
			{
				return cTotalEndLineEntityNum + i;
			}
		}
	}
	//2.槽终止边内，找dXorZVal对应的值（链表应该向槽底方向搜寻）
	else
	{
		for (int16 i = 0; i < cTotalEndLineEntityNum; i++)
		{
			//获取当前实体
			enum ENTITY* pEntity = (enum ENTITY*)FindNodeData(pSGrooveData->pSGroovePathList, i);
			
			//当前实体内根据X值找到TargetVal
			if (findTargetPointInEntity(pEntity, AXIS_X, dXorZVal, pTargetVal) > 0)
			{
				return i;
			}
		}
		
	}
	//槽边中内，未找到dXorZVal对应的值
	return -1;
}
/*************************************************
  函数名称：int8 findCutEndPoint(SList* pGrooveList, int8 cStartGrooveNum, int8 cAxisXorZ, double dXorZVal, int8 cDir, SPoint* pSTargetPoint)
  函数功能  找到进给终点坐标
  输入参数：pGrooveList			槽链表
			cStartGrooveNum		起始槽号
			cAxisXorZ			方向
			dXorZVal			值
			cDir				查找方向
			pTargetVal			终点坐标
  输出参数：
  返 回 值：0	未找到终点坐标
			n	槽号
  备    注：
**************************************************/
int8 findCutEndPoint(SList* pGrooveList, int16 cStartGrooveNum, int8 cAxisXorZ, double dXorZVal, int8 cDir, double* pTargetVal, int8 cMachineXDir)
{
	//总槽个数
	int16 cTotalGrooveNum = getListSize(pGrooveList);

	//剩余槽个数
	int16 cRestGrooveNum = cDir == 1 ? cTotalGrooveNum - cStartGrooveNum + 1 : cStartGrooveNum;

	//当前槽号
	int16 cCurrGrooveNum = cStartGrooveNum;
	SGrooveData* pSCurrGrooveData = NULL;

	//判断是否dXorZVal值落在哪个槽内
	while (cRestGrooveNum > 0)
	{
		//1.获取当前槽数据节点
		pSCurrGrooveData = getGrooveDataByGrooveNum(pGrooveList, cCurrGrooveNum);
		if (cMachineXDir == -1)
		{
			//2.判断dXorZVal是否在当前槽边范围内
			//2.1方向为正，判断dXorZVal是否在槽终止边范围内
			if (cDir == 1)
			{
				double dEndLineTop = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointB.dX : pSCurrGrooveData->SPointB.dZ;
				double dBottom = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointC.dX : pSCurrGrooveData->SPointC.dZ;
				//dXorZVal在槽终止边范围内（槽底底 <= dXorZVal < 槽终止边顶）
				if (isEqualOrGreaterThanZero(dXorZVal - dBottom) && isUnEqualAndLessThanZero(dXorZVal - dEndLineTop))
				{
					//计算dXorZVal对应槽边的坐标
					findTargetPointInGroove(pSCurrGrooveData, FALSE, cAxisXorZ, dXorZVal, pTargetVal);

					return cCurrGrooveNum;
				}
				cCurrGrooveNum++;
			}
			//2.2方向为负，判断dXorZVal是否在槽起始边范围内
			else if (cDir == -1)
			{
				double dStartLineTop = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointA.dX : pSCurrGrooveData->SPointA.dZ;
				double dBottom = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointC.dX : pSCurrGrooveData->SPointC.dZ;
				//dXorZVal在槽起始边范围内（槽底底 <= dXorZVal < 槽起始边顶）
				if (isEqualOrGreaterThanZero(dXorZVal - dBottom) && isUnEqualAndLessThanZero(dXorZVal - dStartLineTop))
				{
					//计算dXorZVal对应槽边的坐标
					findTargetPointInGroove(pSCurrGrooveData, TRUE, cAxisXorZ, dXorZVal, pTargetVal);

					return cCurrGrooveNum;
				}
				cCurrGrooveNum--;
			}
		}
		else 
		{
			//2.判断dXorZVal是否在当前槽边范围内
			//2.1方向为正，判断dXorZVal是否在槽终止边范围内
			if (cDir == 1)
			{
				double dEndLineTop = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointB.dX : pSCurrGrooveData->SPointB.dZ;
				double dBottom = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointC.dX : pSCurrGrooveData->SPointC.dZ;
				//dXorZVal在槽终止边范围内（槽底底 <= dXorZVal < 槽终止边顶）
				if (isEqualOrGreaterThanZero(dXorZVal - dEndLineTop) && isUnEqualAndLessThanZero(dXorZVal - dBottom))
				{
					//计算dXorZVal对应槽边的坐标
					findTargetPointInGroove(pSCurrGrooveData, FALSE, cAxisXorZ, dXorZVal, pTargetVal);

					return cCurrGrooveNum;
				}
				cCurrGrooveNum++;
			}
			//2.2方向为负，判断dXorZVal是否在槽起始边范围内
			else if (cDir == -1)
			{
				double dStartLineTop = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointA.dX : pSCurrGrooveData->SPointA.dZ;
				double dBottom = cAxisXorZ == AXIS_X ? pSCurrGrooveData->SPointC.dX : pSCurrGrooveData->SPointC.dZ;
				//dXorZVal在槽起始边范围内（槽底底 <= dXorZVal < 槽起始边顶）
				if (isEqualOrGreaterThanZero(dXorZVal - dStartLineTop) && isUnEqualAndLessThanZero(dXorZVal - dBottom))
				{
					//计算dXorZVal对应槽边的坐标
					findTargetPointInGroove(pSCurrGrooveData, TRUE, cAxisXorZ, dXorZVal, pTargetVal);

					return cCurrGrooveNum;
				}
				cCurrGrooveNum--;
			}
		}


		//3.剩余槽号--
		cRestGrooveNum--;
	}
	return 0;
}



/*************************************************
  函数名称：
  函数功能  将直线段的相关参数写入G代码缓冲区
  输入参数：pSLineInfo		直线段信息
			pSGCodeBufUnit	G代码缓冲区单元

  输出参数：
  返 回 值：
  备    注：1.该函数仅对缓冲区单元写入，写完后不移动指针
**************************************************/
void writeDotIntoGCodeBuf(SDot* pSDotInfo, struct SGCodeParse* pSGCodeBufUnit)
{
	if (pSDotInfo == NULL || pSGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.写入起点终点坐标
	pSGCodeBufUnit->SEndPoint = pSGCodeBufUnit->SStartPoint = pSDotInfo->SPoint;

	//3.写入实体类型
	pSGCodeBufUnit->EntityType = DOT;

	//5.写入指令类型
	getCommandType(pSDotInfo->sCommand, &pSGCodeBufUnit->unCommandType);

	////3.写入有效轴
	//getValStatus(&pSLineInfo->SStartPoint, &pSLineInfo->SEndPoint, TRUE, pSGCodeBufUnit->cInterpolationState, sGc, &pSGCodeBufUnit->ucValStatus);

	////4.写入刀塔轴与刀塔号
	//getTurretNumAndTurretAxis(pSGCodeBufUnit->ucToolID, &pSGCodeBufUnit->ucTurretNum, &pSGCodeBufUnit->ucTurretAxis);

}

/*************************************************
  函数名称：void writeLineIntoGCodeBuf(SLine* pSLineInfo, struct SGCodeParse* pSGCodeBufUnit)
  函数功能  将直线段的相关参数写入G代码缓冲区
  输入参数：pSLineInfo		直线段信息
			pSGCodeBufUnit	G代码缓冲区单元

  输出参数：
  返 回 值：
  备    注：1.该函数仅对缓冲区单元写入，写完后不移动指针
**************************************************/
void writeLineIntoGCodeBuf(SLine* pSLineInfo, struct SGCodeParse* pSGCodeBufUnit)
{
	if (pSLineInfo == NULL || pSGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.写入起点坐标
	pSGCodeBufUnit->SStartPoint = pSLineInfo->SStartPoint;

	//2.写入终点坐标
	pSGCodeBufUnit->SEndPoint = pSLineInfo->SEndPoint;

	//3.写入实体类型
	pSGCodeBufUnit->EntityType = pSLineInfo->EntityType;

}

/*************************************************
  函数名称：void writeArcIntoGCodeBuf(SArc* pSArcInfo, struct SGCodeParse* pSGCodeBufUnit)
  函数功能  将圆弧段的相关参数写入G代码缓冲区
  输入参数：pSArcInfo		直线段信息
			pSGCodeBufUnit	G代码缓冲区单元
  输出参数：
  返 回 值：
  备    注：1.该函数仅对缓冲区单元写入，写完后不移动指针
			2.该函数仅写入圆弧数据，不能写入整圆数据
			3.写入整圆数据时，必须拆分成两个半圆后，调用两次此函数进行写入
**************************************************/
void writeArcIntoGCodeBuf(SArc* pSArcInfo, struct SGCodeParse* pSGCodeBufUnit)
{
	if (pSArcInfo == NULL || pSGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.写入起点坐标
	pSGCodeBufUnit->SStartPoint = pSArcInfo->SStartPoint;

	//2.写入终点坐标
	pSGCodeBufUnit->SEndPoint = pSArcInfo->SEndPoint;

	//3.写入圆心点坐标
	pSGCodeBufUnit->SCenterPoint = pSArcInfo->SCenterPoint;

	//4.写入半径值
	pSGCodeBufUnit->dRadius = pSArcInfo->dRadius;

	//5.写入圆弧方向
	pSGCodeBufUnit->cArcDir = pSArcInfo->cArcDir;

	//8.写入指令类型
	getCommandType(pSArcInfo->sCommand, &pSGCodeBufUnit->unCommandType);

	//9.写入实体类型
	pSGCodeBufUnit->EntityType = ARC;	

	//写入极值点

	////6.写入有效轴
	//getValStatus(&pSArcInfo->SStartPoint, &pSArcInfo->SEndPoint, FALSE, pSGCodeBufUnit->cInterpolationState, sGc, &pSGCodeBufUnit->ucValStatus);

	////7.写入刀塔轴与刀塔号
	//getTurretNumAndTurretAxis(pSGCodeBufUnit->ucToolID, &pSGCodeBufUnit->ucTurretNum, &pSGCodeBufUnit->ucTurretAxis);

}

/*************************************************
  函数名称：void writeThreadIntoGCodeBuf(SThread* pSThreadInfo, struct SGCodeParse* pSGCodeBufUnit)
  函数功能  将螺纹段的相关参数写入G代码缓冲区
  输入参数：pSThreadInfo		螺纹段信息
			pSGCodeBufUnit	G代码缓冲区单元

  输出参数：
  返 回 值：
  备    注：1.该函数仅对缓冲区单元写入，写完后不移动指针
**************************************************/
void writeThreadIntoGCodeBuf(SThread* pSThreadInfo, struct SGCodeParse* pSGCodeBufUnit)
{
	if (pSThreadInfo == NULL || pSGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.写入起点坐标
	pSGCodeBufUnit->SStartPoint = pSThreadInfo->SStartPoint;

	//2.写入终点坐标
	pSGCodeBufUnit->SEndPoint = pSThreadInfo->SEndPoint;
	
	pSGCodeBufUnit->dFeedrate = pSThreadInfo->dF;
	pSGCodeBufUnit->dJ = pSThreadInfo->dJ;
	pSGCodeBufUnit->dK = pSThreadInfo->dK;
	pSGCodeBufUnit->dQ = pSThreadInfo->dQ;
	pSGCodeBufUnit->usL = pSThreadInfo->usL;

	pSGCodeBufUnit->bG32LetterQ = pSThreadInfo->bG32LetterQ;
	pSGCodeBufUnit->bRepeatG32Flag = pSThreadInfo->bRepeatG32Flag;
	pSGCodeBufUnit->ucG32SubMode = pSThreadInfo->ucG32SubMode;
	pSGCodeBufUnit->cArcDir = pSThreadInfo->cArcDir;
	pSGCodeBufUnit->SCenterPoint = pSThreadInfo->SCenterPoint;
	pSGCodeBufUnit->dRadius = pSThreadInfo->dRadius;


	//9.写入实体类型
	pSGCodeBufUnit->EntityType = pSThreadInfo->EntityType;
}


/*************************************************
  函数名称：void writeTrappingIntoGCodeBuf(STapping* pSTappingInfo, struct SGCodeParse* pSGCodeBufUnit)
  函数功能  将攻牙段的相关参数写入G代码缓冲区
  输入参数：pSTappingInfo		攻牙段信息
			pSGCodeBufUnit	G代码缓冲区单元

  输出参数：
  返 回 值：
  备    注：1.该函数仅对缓冲区单元写入，写完后不移动指针
**************************************************/
void writeTrappingIntoGCodeBuf(STapping* pSTappingInfo, struct SGCodeParse* pSGCodeBufUnit)
{
	if (pSTappingInfo == NULL || pSGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.写入起点坐标
	pSGCodeBufUnit->SStartPoint = pSTappingInfo->SStartPoint;

	//2.写入终点坐标
	pSGCodeBufUnit->SEndPoint = pSTappingInfo->SEndPoint;

	//3.写入实体类型
	pSGCodeBufUnit->EntityType = pSTappingInfo->EntityType;

	//4.写入螺纹头数
	pSGCodeBufUnit->usL = pSTappingInfo->usL;

	//5.写入导程
	pSGCodeBufUnit->dFeedrate = pSTappingInfo->dF;

	//6.写入G33旋转轴

	pSGCodeBufUnit->G33RotateAxis = pSTappingInfo->G33RotateAxis;

	//7.写入攻牙模式
	pSGCodeBufUnit->G33TappingMode = pSTappingInfo->G33TappingMode;

	//8.写入实体类型
	pSGCodeBufUnit->EntityType =TAPPING;

}

void writeEntityIntoGCodeBuf(enum ENTITY* pEntity, struct SGCodeParse* pSGCodeBufUnit, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cMachineDir)
{
	switch (*pEntity)
	{
	case L1NE:case OTHERTYPE:case DOT:
	{
		SLine SLineInfo;
		SLineInfo = *(SLine*)pEntity;
		SLineInfo.SStartPoint = *pSStartPoint;
		SLineInfo.SEndPoint = *pSEndPoint;
		writeLineIntoGCodeBuf(&SLineInfo, pSGCodeBufUnit);
		break;
	}
	case ARC:
	{
		SArc SArcInfo;
		SArcInfo = *(SArc*)pEntity;
		SArcInfo.SStartPoint = *pSStartPoint;
		SArcInfo.SEndPoint = *pSEndPoint;
		SArcInfo.cArcDir = cMachineDir * SArcInfo.cArcDir;
		writeArcIntoGCodeBuf(&SArcInfo, pSGCodeBufUnit);
		break;
	}
	default:
		break;
	}
}

int8 writePathListIntoGCodeBuf(SList* pSPathList, uint16 usStartPos, uint16 usEndPos, uint16* pCurPos, struct SGCodeParse* pSGCodeBufUnit, int8 cMachineDir)
{
	//读取轨迹链表中usPos的实体
	SLine* pEntity;
	pEntity = (SLine*)FindNodeData(pSPathList, *pCurPos);

	if(pEntity == NULL) return -1;
	//将该实体写入缓冲区
	if (cMachineDir == 1)
		writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeBufUnit, &pEntity->SStartPoint, &pEntity->SEndPoint, cMachineDir);
	else
		writeEntityIntoGCodeBuf((enum ENTITY*)pEntity, pSGCodeBufUnit, &pEntity->SEndPoint, &pEntity->SStartPoint, cMachineDir);

	//判断轨迹链表中目标实体是否全部写入缓冲区
	if (cMachineDir == 1 && (*pCurPos) == usStartPos ||
		cMachineDir == -1 && (*pCurPos) == usEndPos)
	{
		return 1;
	}

	//更新当前写入的位置
	*pCurPos = cMachineDir == 1 ? *pCurPos - 1 : *pCurPos + 1;

	return 0;
}




/*************************************************
  函数名称：int8 calculateTotalOffset(uint8 ucChannelNum, uint8 ucToolCompNum, int8 sWorkPieceNum, SPointDouble SWorkPieceOffsetByG50, SPointDouble* pSTotalOffset)
  函数功能：根据刀补号、工件坐标号，G50偏置得到总偏置
  输入参数：ucChannelNum			通道号
			ucToolCompNum			刀补号
			sWorkPieceNum			工件坐标号
			SWorkPieceOffsetByG50	G50偏置值
			pSTotalOffset			总偏置值（输出）

  输出参数：
  返 回 值：-1 指针为空
			-2 刀补号错误
			 1 处理成功
  备    注：总偏置 =
			刀具偏置（刀补号得出）+
			刀具磨损偏置（刀补号得出）+
			外部坐标偏移 +
			工件坐标偏置（工件坐标号得出）+
			G50偏置（G50偏置值得出）
			SPointDouble占用内存过大，在计算总偏置时，通过刀补号与工件坐标号，在系统参数中获取对应偏置值
			（只能G代码使用!!!）
**************************************************/
int8 calculateTotalOffset(SGCodeSysState* pSGCodeSysState)
{
	if (pSGCodeSysState == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	SPointDouble SWorkPieceOffset = pSGCodeSysState->SAllOffsetOfCoord.SWorkPieceOffset;

	SPointDouble SToolLengthOffset = pSGCodeSysState->SAllOffsetOfCoord.SToolLengthOffset;
	SPointDouble SToolWearOffset = pSGCodeSysState->SAllOffsetOfCoord.SToolWearOffset;

	SPointDouble SG50Offset = pSGCodeSysState->SAllOffsetOfCoord.SG50Offset;

	SPointDouble* pSTotalOffset = &pSGCodeSysState->SAllOffsetOfCoord.STotalOffset;

	//每个通道的都有一个独立的坐标系，但机床的刀库只有一个
	for (uint8 ucTemp = 0; ucTemp < AXIS_NUM_MAX; ++ucTemp)  //YZABC轴
	{
		if (g_SSystemPara.SAxisPara[ucTemp].bShowByDiameter)	//直径显示	//陈敏聪	2023/7/22
		{
			pSTotalOffset->dArray[ucTemp] = 
				  SWorkPieceOffset.dArray[ucTemp] / 2
				+ pSGCodeSysState->SAllOffsetOfCoord.SToolSettingOffset.dArray[ucTemp] / 2 // zxr: 外部偏移按直径折半。
				+ SToolLengthOffset.dArray[ucTemp]
				+ SToolWearOffset.dArray[ucTemp]
				+ SG50Offset.dArray[ucTemp];
		}
		else
		{
			pSTotalOffset->dArray[ucTemp] = 
				  SWorkPieceOffset.dArray[ucTemp]
				+ pSGCodeSysState->SAllOffsetOfCoord.SToolSettingOffset.dArray[ucTemp] // zxr: 计入外部偏移。
				+ SToolLengthOffset.dArray[ucTemp]
				+ SToolWearOffset.dArray[ucTemp]
				+ SG50Offset.dArray[ucTemp];
		}
	}

	return 1;
}

/*************************************************
  函数名称：int8 getHorAndVerAxis(uint8 *ucHor, uint8 *ucVer,struct SGCodeParse *SCurrRecord)
  函数功能：从缓冲区结构体中获取横轴纵轴
  输入参数：ucHor横轴，ucVer纵轴，SCurrRecord缓冲区结构体
  输出参数：ucHor横轴，ucVer纵轴
  返 回 值：-1 传入变量为空
			0 正常
			-2 不支持的指令
  备    注：更新了SCurrRecord后，才能使用此函数？
**************************************************/
int8 getHorAndVerAxis(uint8* pHor, uint8* pVer, int8 cInterpolationState, int16 sGc)
{
	if (pHor == NULL || pVer == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	//极坐标模式：3
	if (cInterpolationState == 3)
	{
		*pHor = g_SSystemPara.ucPolarInterpolationAxis[0];
		*pVer = g_SSystemPara.ucPolarInterpolationAxis[1];
	}
	//圆柱插补：5
	else if (cInterpolationState == 5)
	{
		if (sGc == G17 * GCOMMAND_DECIMAL_PLACE * 10)  //XY平面
		{
			*pHor = AXIS_X;
			*pVer = AXIS_C;
		}
		else if (sGc == G18 * GCOMMAND_DECIMAL_PLACE * 10)    //ZX平面
		{
			*pHor = AXIS_Z;
			*pVer = AXIS_C;
		}
		else if (sGc == G19 * GCOMMAND_DECIMAL_PLACE * 10)  //YZ平面
		{
			*pHor = AXIS_C;
			*pVer = AXIS_Z;
		}
		else
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM87);
			return -2;
		}
	}
	//普通插补
	else//普通插补
	{
		if (sGc == G17 * GCOMMAND_DECIMAL_PLACE * 10)  //XY平面
		{
			*pHor = AXIS_X;
			*pVer = AXIS_Y;
			//setbit(SCurrRecord->ucValStatus, AXIS_X);		//陈敏聪2023.11.12注释	只能根据标识符设置有效轴，不能通过插补平面设置有效轴
			//setbit(SCurrRecord->ucValStatus, AXIS_Y);
		}
		else if (sGc == G18 * GCOMMAND_DECIMAL_PLACE * 10)    //ZX平面
		{
			*pHor = AXIS_Z;
			*pVer = AXIS_X;
		}
		else if (sGc == G19 * GCOMMAND_DECIMAL_PLACE * 10)  //YZ平面
		{
			*pHor = AXIS_Y;
			*pVer = AXIS_Z;
		}
		else
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM87);
			return -2;
		}
	}

	return 0;
}

/*************************************************
  函数名称：uint8 isSamePoint(SPointDouble* pSPoint1, SPointDouble* pSPoint2)
  函数功能：判断两点各轴坐标是否相同
  输入参数：pSPoint1	点1
			pSPoint2	点2
  输出参数：
  返 回 值：TRUE	坐标相同
			FALSE	坐标不同
  备    注：
**************************************************/
bool isSamePoint(SPointDouble* pSPoint1, SPointDouble* pSPoint2)
{
	uint8 ucTemp;
	//获取移动轴
	for (ucTemp = 0; ucTemp < AXIS_NUM_MAX; ucTemp++)
	{
		if (!isZero(pSPoint1->dArray[ucTemp] - pSPoint2->dArray[ucTemp]))
		{
			return FALSE;
		}
	}
	return TRUE;
}


/*************************************************
  函数名称：void getArcCenterPointAndRadius(SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint,
			int8 cArcDir, SPointDouble* pSCenterPoint, double* pRadius, uint8 ucHorAxis, uint8 ucVerAxis)
  函数功能：获取圆心坐标与半径
  输入参数：pSScanContainer		提供 圆心坐标 or 半径
			pSStartPoint		圆弧起点坐标
			pSEndPoint			圆弧终点坐标
			cArcDir				圆弧方向（ARC_CW、ARC_ANTI_CW）
			pSCenterPoint		圆心坐标（输出）
			pRadius				半径（输出）

  输出参数：
  返 回 值：
  备    注：lxw20241105更新
			0.圆心坐标仅输出横轴纵轴值
			1.由起点、终点、半径      得到 圆心坐标+半径
			2.由起点、终点、圆心坐标  得到 圆心坐标+半径
			3.不论优弧劣弧，半径值输出为正
**************************************************/
double getArcCenterPointAndRadius(SScan* pSScanContainer, SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, int8 cArcDir, SPointDouble* pSCenterPoint, double* pRadius, uint8 ucHorAxis, uint8 ucVerAxis)
{
	double dRad = 0;		//圆弧角，未用到，补足函数传参
//	SPointDouble pS, pE;     //圆弧起点，终点
//	uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
	int8 cRet;
	uint8 ucArcG32Type;

	if (pSScanContainer == NULL || pSStartPoint == NULL || pSEndPoint == NULL || pSCenterPoint == NULL || pRadius == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	//1.起点终点重合
	if (isZero(pSStartPoint->dArray[ucHorAxis] - pSEndPoint->dArray[ucHorAxis]) &&
		isZero(pSStartPoint->dArray[ucVerAxis] - pSEndPoint->dArray[ucVerAxis]))
	{
		// Validate an explicit R before the tolerant point/full-circle shortcut can hide an impossible arc.
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			SPointDouble SCheckCenter = *pSStartPoint;
			double dCheckRad;
			if (radCalculateR(pSStartPoint->dArray, pSEndPoint->dArray, pSScanContainer->dRIJK[0],
				SCheckCenter.dArray, cArcDir, &dCheckRad, ucHorAxis, ucVerAxis) == FALSE)
			{
				parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM99);
				return -1;
			}
		}
		//1.1通过IJK指定圆心
		if (pSScanContainer->bRIJK[ucHorAxis + 1] == TRUE ||
			pSScanContainer->bRIJK[ucVerAxis + 1] == TRUE)
		{
			//计算圆心坐标
			pSCenterPoint->dArray[ucHorAxis] = pSStartPoint->dArray[ucHorAxis] + pSScanContainer->dRIJK[ucHorAxis + 1];
			pSCenterPoint->dArray[ucVerAxis] = pSStartPoint->dArray[ucVerAxis] + pSScanContainer->dRIJK[ucVerAxis + 1];
			//计算半径值
			*pRadius = sqrt(pow(pSScanContainer->dRIJK[ucHorAxis + 1], 2)
				+ pow(pSScanContainer->dRIJK[ucVerAxis + 1], 2));

			return 2 * PI;
		}
		//1.2未通过IJK指定圆心
		else
		{
			*pSCenterPoint = *pSStartPoint;
			*pRadius = 0;
			return 0;
		}
		
	}

	//2.R指定的圆弧
	if (pSScanContainer->bRIJK[0] == TRUE)
	{
		//2.1计算半径
		*pRadius = pSScanContainer->dRIJK[0] > 0 ? pSScanContainer->dRIJK[0] : -pSScanContainer->dRIJK[0];
		//2.2计算圆心坐标
		cRet = radCalculateR((double*)pSStartPoint, (double*)pSEndPoint, pSScanContainer->dRIJK[0],
			(double*)pSCenterPoint, cArcDir, &dRad, ucHorAxis, ucVerAxis);
		//2.3输入参数错误
		if (cRet == FALSE)
		{
			/*原来逻辑
			//圆弧两端点值距离大于直径时，将半径值修改为两点距离的一半
			*pRadius = sqrt((pSEndPoint->dArray[ucHorAxis] - pSStartPoint->dArray[ucHorAxis]) * (pSEndPoint->dArray[ucHorAxis] - pSStartPoint->dArray[ucHorAxis]) +
				(pSEndPoint->dArray[ucVerAxis] - pSStartPoint->dArray[ucVerAxis]) * (pSEndPoint->dArray[ucVerAxis] - pSStartPoint->dArray[ucVerAxis])) / 2;

			//再次计算修改后的圆心角和圆心
			radCalculateR((double*)pSStartPoint, (double*)pSEndPoint, *pRadius,
				(double*)pSCenterPoint, cArcDir, &dRad, ucHorAxis, ucVerAxis);
			*/
			 // 给定半径无解时直接拒绝，不能改成半弦长后继续加工。
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM99);
			return -1;

		}
		
	}

	//3.IJK指定的圆弧
	else if (pSScanContainer->bRIJK[ucHorAxis + 1] == TRUE || pSScanContainer->bRIJK[ucVerAxis + 1] == TRUE)
	{
		pSCenterPoint->dArray[ucHorAxis] = pSStartPoint->dArray[ucHorAxis] + pSScanContainer->dRIJK[ucHorAxis + 1];
		pSCenterPoint->dArray[ucVerAxis] = pSStartPoint->dArray[ucVerAxis] + pSScanContainer->dRIJK[ucVerAxis + 1];
			
		double dR1;//起点-圆心距离平方
		dR1 = pow(pSCenterPoint->dArray[ucHorAxis] - pSStartPoint->dArray[ucHorAxis], 2) +
			pow(pSCenterPoint->dArray[ucVerAxis] - pSStartPoint->dArray[ucVerAxis], 2);
		double dR2;//终点-圆心距离平方
		dR2 = pow(pSCenterPoint->dArray[ucHorAxis] - pSEndPoint->dArray[ucHorAxis], 2) +
			pow(pSCenterPoint->dArray[ucVerAxis] - pSEndPoint->dArray[ucVerAxis], 2);
		//3.1IJK指定的圆心坐标正确
		if (isZero(dR1 - dR2))
		{
			*pRadius = sqrt(dR1);
		}
		//3.2输入参数错误（起点--圆心--终点构成不了圆弧）
		else
		{
			/*
			//圆弧两端点值距离大于直径时，将半径值修改为两点距离的一半
			*pRadius = sqrt((pSEndPoint->dArray[ucHorAxis] - pSStartPoint->dArray[ucHorAxis]) * (pSEndPoint->dArray[ucHorAxis] - pSStartPoint->dArray[ucHorAxis]) +
				(pSEndPoint->dArray[ucVerAxis] - pSStartPoint->dArray[ucVerAxis]) * (pSEndPoint->dArray[ucVerAxis] - pSStartPoint->dArray[ucVerAxis])) / 2;

			//再次计算修改后的圆心角和圆心
			radCalculateR((double*)pSStartPoint, (double*)pSEndPoint, *pRadius,
				(double*)pSCenterPoint, cArcDir, &dRad, ucHorAxis, ucVerAxis);
				*/
			parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM404);
			return -1;
		}

	}

	//4.未出现圆弧任何参数
	else
	{
		*pSCenterPoint = *pSStartPoint;
		*pRadius = 0;
		return 0;
	}
	return dRad;
	//获取圆弧极值点（后续用于软限位）
	// 
	//SAssignConfig->SGetAxisMaxAndMin.cArcDir = SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].cArcDir;
	//SAssignConfig->SGetAxisMaxAndMin.dRadius = SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].dRadius;
	//SAssignConfig->SGetAxisMaxAndMin.SStartPoint = &SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].SStartPoint;
	//SAssignConfig->SGetAxisMaxAndMin.SEndPoint = &SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].SEndPoint;
	//SAssignConfig->SGetAxisMaxAndMin.SCenterPoint = &SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].SCenterPoint;
	//SAssignConfig->SGetAxisMaxAndMin.ucHor = ucHorAxis;
	//SAssignConfig->SGetAxisMaxAndMin.ucVer = ucVerAxis;
	//getAxisMaxAndMin(&SAssignConfig->SGetAxisMaxAndMin);
	//SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].dHorMax = SAssignConfig->SGetAxisMaxAndMin.dHorMax;
	//SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].dHorMin = SAssignConfig->SGetAxisMaxAndMin.dHorMin;
	//SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].dVerMax = SAssignConfig->SGetAxisMaxAndMin.dVerMax;
	//SAssignConfig->pGCodeBuf[SAssignConfig->usIndex].dVerMin = SAssignConfig->SGetAxisMaxAndMin.dVerMin;


}

/*************************************************
  函数名称：
  函数功能：获取圆弧类型
  输入参数：pSScanContainer		提供 圆心坐标 or 半径
			pSStartPoint		圆弧起点坐标
			pSEndPoint			圆弧终点坐标
			cArcDir				圆弧方向（ARC_CW、ARC_ANTI_CW）
			pSCenterPoint		圆心坐标（输出）
			pRadius				半径（输出）

  输出参数：
  返 回 值：0	点
			1	圆弧
			2	整圆
  备    注：lxw20241105更新
			0.圆心坐标仅输出横轴纵轴值
			1.由起点、终点、半径      得到 圆心坐标+半径
			2.由起点、终点、圆心坐标  得到 圆心坐标+半径
			3.不论优弧劣弧，半径值输出为正
**************************************************/
int8 getArcType(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, SPointDouble* pSCenterPoint, double dRadius,uint8 ucHor,uint8 ucVer)
{
	//1.ARC为点
	if (isSamePoint(pSStartPoint, pSEndPoint) && isZero(dRadius))
	{
		return 0;
	}
	//ARC为整圆
	else if (isZero(pSStartPoint->dArray[ucHor] - pSEndPoint->dArray[ucHor]) &&
		isZero(pSStartPoint->dArray[ucVer] - pSEndPoint->dArray[ucVer]) &&
		!isZero(dRadius))
	{
		return 2;
	}
	//ARC为圆弧
	else
	{
		return 1;
	}
}


/*************************************************
  函数名称：
  函数功能：从程序段中获取局部宏变量的值
  输入参数：

  输出参数：
  返 回 值
  备    注：由于G65/G66程序段中可以重复出现IJK，导致普通扫描报错
			所以当扫描到G65/G66 P**时，停止了后续扫描，所以需要重新扫描
**************************************************/
int8 getLocalMacroVarBySegment(SGetTextStateInfo* pSGetTextStateInfo, SScan* pScan, SLocalMacroVar* pSLocalMacroVar)
{
	uint8 ucLetterIJKCnt[3] = { 0 };    //IJK计数器，与G65、G66的IJK传参方式有关

	while (!isSegmentFinishSign(pSGetTextStateInfo->pCurrPtr) &&
		!isLineFinishSign(pSGetTextStateInfo->pCurrPtr))
	{
		//1.pStartPtr指向指令字的首字符
		pSGetTextStateInfo->pStartPtr = pSGetTextStateInfo->pCurrPtr;
		pSGetTextStateInfo->pCurrPtr++;

		int8* pTemp;
		pTemp = pSGetTextStateInfo->pCurrPtr;//记录表达式首字符

		//2.pCurrPtr指向指令字表达式的首字符，pEndPtr指向指令字的末字符后一位
		int8 cRet = 0;
		cRet = getWholeExpression(&pSGetTextStateInfo->pCurrPtr, pSGetTextStateInfo->unCurrLineNum);    //截取整个宏单元
		if (cRet == 1)
		{
			//已获取完整表达式，pEndPtr指向表达式后一位
			pSGetTextStateInfo->pEndPtr = pSGetTextStateInfo->pCurrPtr;
			//将pCurrPtr重新移动到表达式开头
			pSGetTextStateInfo->pCurrPtr = pTemp;
			uint8 ucExpressionLen = pSGetTextStateInfo->pEndPtr - pSGetTextStateInfo->pCurrPtr;
			if (isNeedCalculate(pSGetTextStateInfo->pCurrPtr, ucExpressionLen))
			{
				//G65/G66程序段不支持计算
				setErrorType(GCODE_ERROR_NUM84);
				findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
			}
		}
		else
		{
			setErrorType(GCODE_ERROR_NUM84);
			findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
			return -1;
		}

		//3.获取对应的数值
		uint8 ucPos = 0;
		double dVal = 0.0;
		if(getNumbers(pSGetTextStateInfo->pCurrPtr, &ucPos, &dVal, pSGetTextStateInfo->unCurrLineNum) != 0)
		{
			setErrorType(GCODE_ERROR_NUM84);
			findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
			return -1;
		}
		//4.记录到局部宏变量
		switch (*pSGetTextStateInfo->pStartPtr)
		{
		case'P':case'p':
		{
			pScan->bP = TRUE;
			pScan->dP = dVal;
			break;
		}
		case'L':case'l':
		{
			pScan->bL = TRUE;
			pScan->dL = dVal;
			break;
		}

		case'A':case'a':
		{
			pSLocalMacroVar->dValArr[0] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 0);
			break;
		}

		case'B':case'b':
		{
			pSLocalMacroVar->dValArr[1] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 1);
			break;
		}

		case'C':case'c':
		{
			pSLocalMacroVar->dValArr[2] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 2);
			break;
		}

		case'D':case'd':
		{
			pSLocalMacroVar->dValArr[6] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 6);
			break;
		}

		case'E':case'e':
		{
			pSLocalMacroVar->dValArr[7] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 7);
			break;
		}

		case'F':case'f':
		{
			pSLocalMacroVar->dValArr[8] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 8);
			break;
		}

		case'H':case'h':
		{
			pSLocalMacroVar->dValArr[10] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 10);
			break;
		}

		case'I':case'i':
		{
			pSLocalMacroVar->dValArr[ucLetterIJKCnt[0] * 3 + 3] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, (ucLetterIJKCnt[0] * 3 + 3));
			ucLetterIJKCnt[0] = ucLetterIJKCnt[0] < 9 ? ucLetterIJKCnt[0] + 1 : ucLetterIJKCnt[0];
			break;
		}

		case'J':case'j':
		{
			pSLocalMacroVar->dValArr[ucLetterIJKCnt[1] * 3 + 4] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, (ucLetterIJKCnt[1] * 3 + 4));
			ucLetterIJKCnt[1] = ucLetterIJKCnt[1] < 9 ? ucLetterIJKCnt[1] + 1 : ucLetterIJKCnt[1];
			break;
		}

		case'K':case'k':
		{
			pSLocalMacroVar->dValArr[ucLetterIJKCnt[2] * 3 + 5] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, (ucLetterIJKCnt[2] * 3 + 5));
			ucLetterIJKCnt[2] = ucLetterIJKCnt[2] < 9 ? ucLetterIJKCnt[2] + 1 : ucLetterIJKCnt[2];
			break;
		}

		case'M':case'm':
		{
			pSLocalMacroVar->dValArr[12] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 12);
			break;
		}

		case'Q':case'q':
		{
			pSLocalMacroVar->dValArr[16] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 16);
			break;
		}
		case'R':case'r':
		{
			pSLocalMacroVar->dValArr[17] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 17);
			break;
		}


		case'S':case's':
		{
			pSLocalMacroVar->dValArr[18] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 18);
			break;
		}
		case'T':case't':
		{
			pSLocalMacroVar->dValArr[19] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 19);
			break;
		}

		case'U':case'u':
		{
			pSLocalMacroVar->dValArr[20] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 20);
			break;
		}

		case'V':case'v':
		{
			pSLocalMacroVar->dValArr[21] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 21);
			break;
		}

		case'W':case'w':
		{
			pSLocalMacroVar->dValArr[22] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 22);
			break;
		}

		case'X':case'x':
		{
			pSLocalMacroVar->dValArr[23] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 23);
			break;
		}

		case'Y':case'y':
		{
			pSLocalMacroVar->dValArr[25] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 24);
			break;
		}
		case'Z':case'z':
		{
			pSLocalMacroVar->dValArr[25] = dVal;
			clrbit(pSLocalMacroVar->ulIsNull, 25);
			break;
		}
		default:
			break;
		}

		//4.
		pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;

	}
	return 1;

}


//分割轨迹
int8 splitPathByBlank(SList* pSPathList, double dX, double dZ)
{
	int16 sCurrPos;
	sCurrPos = getListSize(pSPathList) - 1;
	while (sCurrPos >= 0)
	{
		//1.找到当前实体
		SLine* pCurrEntity;
		pCurrEntity = FindNodeData(pSPathList, sCurrPos);

		double dTargetX;
		double dTargetZ;
		//2.当前实体穿越毛坯
		if (entityCrossBlank(pCurrEntity, dX, dZ, &dTargetX, &dTargetZ))
		{
			switch (pCurrEntity->EntityType)
			{
			case OTHERTYPE:case L1NE:
			{
				SLine* pSLine;
				pSLine = mallocGCode(sizeof(SLine), ORAM_NUM355);//内存未编号
				if (pSLine == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return -1;
				}

				*pSLine = *(SLine*)pCurrEntity;

				pSLine->SEndPoint.dArray[AXIS_X] = pCurrEntity->SStartPoint.dArray[AXIS_X] = dTargetX;
				pSLine->SEndPoint.dArray[AXIS_Z] = pCurrEntity->SStartPoint.dArray[AXIS_Z] = dTargetZ;

				int cRet = 0;
				cRet = insertList(pSPathList, sCurrPos + 1, pSLine, ORAM_NUM354);//内存未编号
				if (cRet == FALSE)
				{
					freeGCode(pSLine);
					pSLine = NULL;
					parseGCodeErrorB(0, GCODE_ERROR_NUM303);
					return -1;
				}

				break;
			}
			case ARC:
			{
				SArc* pSArc;
				pSArc = mallocGCode(sizeof(SArc), ORAM_NUM356);//内存未编号
				if (pSArc == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return -1;
				}

				*pSArc = *(SArc*)pCurrEntity;

				pSArc->SEndPoint.dArray[AXIS_X] = pCurrEntity->SStartPoint.dArray[AXIS_X] = dTargetX;
				pSArc->SEndPoint.dArray[AXIS_Z] = pCurrEntity->SStartPoint.dArray[AXIS_Z] = dTargetZ;

				int cRet = 0;

				cRet = insertList(pSPathList, sCurrPos + 1, pSArc, ORAM_NUM354);//内存未编号
				if (cRet == FALSE)
				{
					freeGCode(pSArc);
					pSArc = NULL;
					parseGCodeErrorB(0, GCODE_ERROR_NUM303);
					return -1;
				}

				break;
			}
			default:
				break;
			}
		}
		//当前实体未穿越毛坯
		else
		{
			sCurrPos--;
		}

	}
	return 0;
}

//判断实体是否穿越毛坯
int8 entityCrossBlank(SLine* pEntity, double dX, double dZ, double* pTargetX, double* pTargetZ)
{
	bool bFlag_1 = FALSE;	//实体穿越端面标志
	bool bFlag_2 = FALSE;	//实体穿越柱面标志
	double dTargetX;
	double dTargetZ;

	//判断实体是否穿越毛坯端面
	if (isUnEqualAndLessThanZero((pEntity->SStartPoint.dArray[AXIS_Z] - dZ) * (pEntity->SEndPoint.dArray[AXIS_Z] - dZ)))
	{
		findTargetPointInEntity((enum ENTITY*)pEntity, AXIS_Z, dZ, &dTargetX);
		if (dTargetX <= dX)
			bFlag_1 = TRUE;
	}
	//判断实体是否穿越毛坯柱面
	if (isUnEqualAndLessThanZero((pEntity->SStartPoint.dArray[AXIS_X] - dX) * (pEntity->SEndPoint.dArray[AXIS_X] - dX)))
	{
		findTargetPointInEntity((enum ENTITY*)pEntity, AXIS_X, dX, &dTargetZ);
		if (dTargetZ <= dZ)
			bFlag_2 = TRUE;
	}

	if (bFlag_1 == FALSE && bFlag_2 == FALSE)
		return 0;
	if (bFlag_1 == TRUE && bFlag_2 == TRUE)
	{
		if (fabs(pEntity->SStartPoint.dArray[AXIS_Z] - dZ) < fabs(pEntity->SStartPoint.dArray[AXIS_Z] - dTargetZ))
		{
			*pTargetX = dTargetX;
			*pTargetZ = dZ;
		}
		else
		{
			*pTargetX = dX;
			*pTargetZ = dTargetZ;
		}
	}
	else if (bFlag_1 == TRUE)
	{
		*pTargetX = dTargetX;
		*pTargetZ = dZ;
	}
	else if (bFlag_2 == TRUE)
	{
		*pTargetX = dX;
		*pTargetZ = dTargetZ;
	}
	return 1;
}

//删除轨迹中的实体（根据毛坯XZ删除）
void deleteEntityInPath(SList* pSPathList, double dX, double dZ)
{
	int16 sCurrPos;
	sCurrPos = getListSize(pSPathList) - 1;
	while (sCurrPos >= 0)
	{
		//1.找到当前实体
		SLine* pCurrEntity;
		pCurrEntity = FindNodeData(pSPathList, sCurrPos);
		
		//起点或终点在毛坯之外，则删除实体
		if (isUnEqualAndGreaterThanZero(pCurrEntity->SStartPoint.dArray[AXIS_X] - dX) ||
			isUnEqualAndGreaterThanZero(pCurrEntity->SStartPoint.dArray[AXIS_Z] - dZ) ||
			isUnEqualAndGreaterThanZero(pCurrEntity->SEndPoint.dArray[AXIS_X] - dX) || 
			isUnEqualAndGreaterThanZero(pCurrEntity->SEndPoint.dArray[AXIS_Z] - dZ))
		{
			//删除实体节点
			removeNode(pSPathList, sCurrPos, FALSE);
		}

		sCurrPos--;
		
	}
}

//补充轨迹中的实体
int8 addEntityInPath(SList* pSPathList)
{
	int16 sCurrPos;
	sCurrPos = getListSize(pSPathList) - 1;
	while (sCurrPos >= 1)
	{
		//1.找到当前实体
		SLine* pCurrEntity;
		SLine* pNextEntity;
		pCurrEntity = FindNodeData(pSPathList, sCurrPos);
		pNextEntity = FindNodeData(pSPathList, sCurrPos - 1);

		//2.当前实体终点 != 下一实体起点，则需要添加实体
		if (!isZero(pCurrEntity->SEndPoint.dArray[AXIS_X] - pNextEntity->SStartPoint.dArray[AXIS_X]) ||
			!isZero(pCurrEntity->SEndPoint.dArray[AXIS_Z] - pNextEntity->SStartPoint.dArray[AXIS_Z]))
		{
			SLine* pSLine;
			pSLine = mallocGCode(sizeof(SLine), ORAM_NUM355);
			if (pSLine == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				return -1;
			}
			pSLine->EntityType = L1NE;
			pSLine->SStartPoint.dArray[AXIS_X] = pCurrEntity->SEndPoint.dArray[AXIS_X];
			pSLine->SStartPoint.dArray[AXIS_Z] = pCurrEntity->SEndPoint.dArray[AXIS_Z];
			pSLine->SEndPoint.dArray[AXIS_X] = pNextEntity->SStartPoint.dArray[AXIS_X];
			pSLine->SEndPoint.dArray[AXIS_Z] = pNextEntity->SStartPoint.dArray[AXIS_Z];

			int cRet = 0;
			cRet = insertList(pSPathList, sCurrPos, pSLine, ORAM_NUM354);//内存未编号
			if (cRet == FALSE)
			{
				freeGCode(pSLine);
				pSLine = NULL;
				parseGCodeErrorB(0, GCODE_ERROR_NUM303);
				return -1;
			}
		}
		else
		{
			sCurrPos--;
		}
	}
	return 0;
}

/*************************************************
  函数名称：int8 isArcCrossQuadrant(SArc* pSArc, SPointDouble* pSPoint)
  函数功能：判断圆弧是否跨越象限
  输入参数：pSArc		圆弧
			pSPoint		第一跨越点
  输出参数：
  返 回 值：1	跨越
			0	未跨越
  备    注：若圆弧跨越象限，则通过pSPoint返回第一跨越点坐标
**************************************************/
int8 isArcCrossQuadrant(SArc* pSArc, SPointDouble* pSPoint)
{
	//弧段中途跨越坐标轴跨象限
	if (pSArc->dRadius < 0)
	{
		return 1;				
	}
	//起点/终点相对于圆心坐标
	double dSX;
	double dSZ;
	double dEX;
	double dEZ;
	dSX = pSArc->SStartPoint.dArray[AXIS_X] - pSArc->SCenterPoint.dArray[AXIS_X];
	dSZ = pSArc->SStartPoint.dArray[AXIS_Z] - pSArc->SCenterPoint.dArray[AXIS_Z];
	dEX = pSArc->SEndPoint.dArray[AXIS_X] - pSArc->SCenterPoint.dArray[AXIS_X];
	dEZ = pSArc->SEndPoint.dArray[AXIS_Z] - pSArc->SCenterPoint.dArray[AXIS_Z];
	
	//判断原理：若起点终点位于X轴两侧 或 起点终点位于Z轴两侧，则跨越象限
	if (isUnEqualAndLessThanZero(dSX * dEX) || isUnEqualAndLessThanZero(dSZ * dEZ))
	{
		//获取圆弧起点象限
		int8 cStart = 0;
		//逆时针（Z轴正半轴属于第一象限，X轴正半轴属于第二象限）
		if (pSArc->cArcDir == ARC_ANTI_CW)
		{
			if (isEqualOrGreaterThanZero(dSX) && isUnEqualAndGreaterThanZero(dSZ))
			{
				//dSX >= 0 && dSZ > 0，第一象限
				cStart = 1;
			}
			else if (isUnEqualAndGreaterThanZero(dSX) && isEqualOrLessThanZero(dSZ))
			{
				//dSX > 0 && dSZ <= 0，第二象限
				cStart = 2;
			}
			else if (isEqualOrLessThanZero(dSX) && isUnEqualAndLessThanZero(dSZ))
			{
				//dSX <= 0 && dSZ < 0，第三象限
				cStart = 3;
			}
			else if (isUnEqualAndLessThanZero(dSX) && isEqualOrGreaterThanZero(dSZ)) 
			{
				//dSX < 0 && dSZ >= 0，第四象限
				cStart = 4;
			}
		}
		//顺时针（Z轴正半轴属于第四象限，X轴正半轴属于第一象限）
		else
		{
			if (isUnEqualAndGreaterThanZero(dSX) && isEqualOrGreaterThanZero(dSZ))
			{
				//dSX > 0 && dSZ >= 0，第一象限
				cStart = 1;
			}
			else if (isEqualOrGreaterThanZero(dSX) && isUnEqualAndLessThanZero(dSZ))
			{
				//dSX >= 0 && dSZ < 0，第二象限
				cStart = 2;
			}
			else if (isUnEqualAndLessThanZero(dSX) && isEqualOrLessThanZero(dSZ))
			{
				//dSX < 0 && dSZ <= 0，第三象限
				cStart = 3;
			}
			else if (isEqualOrLessThanZero(dSX) && isUnEqualAndGreaterThanZero(dSZ))
			{
				//dSX <= 0 && dSZ > 0，第四象限
				cStart = 4;
			}

		}

		//计算第一跨越点
		if ((cStart == 1 && pSArc->cArcDir == ARC_ANTI_CW) ||
			(cStart == 2 && pSArc->cArcDir == ARC_CW))
		{
			//跨越X轴正半轴
			pSPoint->dArray[AXIS_X] = pSArc->SCenterPoint.dArray[AXIS_X] + pSArc->dRadius;
			pSPoint->dArray[AXIS_Z] = pSArc->SCenterPoint.dArray[AXIS_Z];
		}
		else if ((cStart == 2 && pSArc->cArcDir == ARC_ANTI_CW) ||
			(cStart == 3 && pSArc->cArcDir == ARC_CW))
		{
			//跨越Z轴负半轴
			pSPoint->dArray[AXIS_X] = pSArc->SCenterPoint.dArray[AXIS_X];
			pSPoint->dArray[AXIS_Z] = pSArc->SCenterPoint.dArray[AXIS_Z] - pSArc->dRadius;
		}
		else if ((cStart == 3 && pSArc->cArcDir == ARC_ANTI_CW) ||
			(cStart == 4 && pSArc->cArcDir == ARC_CW))
		{
			//跨越X轴负半轴
			pSPoint->dArray[AXIS_X] = pSArc->SCenterPoint.dArray[AXIS_X] - pSArc->dRadius;
			pSPoint->dArray[AXIS_Z] = pSArc->SCenterPoint.dArray[AXIS_Z];
		}
		else if ((cStart == 4 && pSArc->cArcDir == ARC_ANTI_CW) ||
			(cStart == 1 && pSArc->cArcDir == ARC_CW))
		{
			pSPoint->dArray[AXIS_X] = pSArc->SCenterPoint.dArray[AXIS_X];
			pSPoint->dArray[AXIS_Z] = pSArc->SCenterPoint.dArray[AXIS_Z] + pSArc->dRadius;
		}

		return 1;
	} 


	return 0;
}

/*************************************************
  函数名称：void divideArcInPath(SList* pSPathList)
  函数功能：分割轨迹中的圆弧
  输入参数：pSPathList		轨迹链表
  输出参数：
  返 回 值：
  备    注：将轨迹中跨越象限的圆弧进行拆分
**************************************************/
void divideArcInPath(SList* pSPathList)
{
	int16 sCurrPos;
	sCurrPos = getListSize(pSPathList) - 1;
	while (sCurrPos >= 1)
	{
		SArc* pCurrEntity;
		pCurrEntity = FindNodeData(pSPathList, sCurrPos);
		if (pCurrEntity->EntityType == ARC)
		{
			SPointDouble SPoint;
			//跨象限
			if (isArcCrossQuadrant(pCurrEntity, &SPoint))
			{
				//创建圆弧实体
				SArc* pSNewArc;
				pSNewArc = mallocGCode(sizeof(SArc), ORAM_NUM356);
				if (pSNewArc == NULL)
				{
					parseGCodeErrorB(0, GCODE_ERROR_NUM406);
					return;
				}
				*pSNewArc = *pCurrEntity;

				//修正圆弧起点终点
				pSNewArc->SStartPoint.dArray[AXIS_X] = SPoint.dArray[AXIS_X];
				pSNewArc->SStartPoint.dArray[AXIS_Z] = SPoint.dArray[AXIS_Z];
				pCurrEntity->SEndPoint.dArray[AXIS_X] = SPoint.dArray[AXIS_X];
				pCurrEntity->SEndPoint.dArray[AXIS_Z] = SPoint.dArray[AXIS_Z];

				//插入链表
				int cRet = 0;

				cRet = insertList(pSPathList, sCurrPos, pSNewArc, ORAM_NUM354);
				if (cRet == FALSE)
				{
					freeGCode(pSNewArc);
					pSNewArc == NULL;
					parseGCodeErrorB(0, GCODE_ERROR_NUM303);
					return;
				}
			}
			else
			{
				sCurrPos--;
			}
		}
		else
		{
			sCurrPos--;
		}

	}
}

/*************************************************
  函数名称：void equidistantEntity(enum ENTITY* pEntity, double dDist, int8 cDir)
  函数功能：等距实体
  输入参数：pEntity		实体
			dDist		等距距离
			cDir		等距方向
  输出参数：
  返 回 值：
  备    注：cDir = 1	实体向起点-终点左侧等距
			cDir = -1	实体向起点-终点右侧等距
**************************************************/
void equidistantEntity(enum ENTITY* pEntity, double dDist, int8 cDir)
{
	switch (*pEntity)
	{
	case OTHERTYPE:case L1NE:
	{
		SLine* pSLine = (SLine*)pEntity;
		double SX = pSLine->SStartPoint.dArray[AXIS_X];
		double SZ = pSLine->SStartPoint.dArray[AXIS_Z];
		double EX = pSLine->SEndPoint.dArray[AXIS_X];
		double EZ = pSLine->SEndPoint.dArray[AXIS_Z];
		double dSE = sqrt((SX - EX) * (SX - EX) + (SZ - EZ) * (SZ - EZ));

		//计算单位法向矢量
		double dN_Z = -(EX - SX) / dSE;
		double dN_X = (EZ - SZ) / dSE;

		//计算等距后的起点
		pSLine->SStartPoint.dArray[AXIS_X] += cDir * dDist * dN_X;
		pSLine->SStartPoint.dArray[AXIS_Z] += cDir * dDist * dN_Z;

		//计算等距后的终点
		pSLine->SEndPoint.dArray[AXIS_X] += cDir * dDist * dN_X;
		pSLine->SEndPoint.dArray[AXIS_Z] += cDir * dDist * dN_Z;		
		
		break;
	}
	case ARC:
	{
		SArc* pSArc = (SArc*)pEntity;
		double SX = pSArc->SStartPoint.dArray[AXIS_X];
		double SZ = pSArc->SStartPoint.dArray[AXIS_Z];
		double EX = pSArc->SEndPoint.dArray[AXIS_X];
		double EZ = pSArc->SEndPoint.dArray[AXIS_Z];
		double CX = pSArc->SCenterPoint.dArray[AXIS_X];
		double CZ = pSArc->SCenterPoint.dArray[AXIS_Z];

		double r = pSArc->dRadius;

		//计算起点单位法向矢量
		double dN1_Z = (SZ - CZ) / r;
		double dN1_X = (SX - CX) / r;
		//计算终点单位法向矢量
		double dN2_Z = (EZ - CZ) / r;
		double dN2_X = (EX - CX) / r;

		//计算等距后的起点
		if (pSArc->cArcDir == ARC_ANTI_CW)
		{
			pSArc->SStartPoint.dArray[AXIS_X] -= cDir * dDist * dN1_X;
			pSArc->SStartPoint.dArray[AXIS_Z] -= cDir * dDist * dN1_Z;
		}
		else
		{
			pSArc->SStartPoint.dArray[AXIS_X] += cDir * dDist * dN1_X;
			pSArc->SStartPoint.dArray[AXIS_Z] += cDir * dDist * dN1_Z;
		}
		//计算等距后的终点
		if (pSArc->cArcDir == ARC_ANTI_CW)
		{
			pSArc->SEndPoint.dArray[AXIS_X] -= cDir * dDist * dN2_X;
			pSArc->SEndPoint.dArray[AXIS_Z] -= cDir * dDist * dN2_Z;
		}
		else
		{
			pSArc->SEndPoint.dArray[AXIS_X] += cDir * dDist * dN2_X;
			pSArc->SEndPoint.dArray[AXIS_Z] += cDir * dDist * dN2_Z;

		}
		//计算等距后的半径
		pSArc->dRadius = fabs(pSArc->dRadius + cDir * pSArc->cArcDir * dDist);
		
		break;
	}
	default:
		break;
	}
}

/*************************************************
  函数名称：void equidistantPath(SList* pSOriPathList, SList** ppSNewPathList, double dDist, int8 cDir)
  函数功能：等距轨迹
  输入参数：pSOriPathList		原始轨迹
			ppSNewPathList		等距后的轨迹
			dDist		等距距离
			cDir		等距方向
  输出参数：
  返 回 值：
  备    注：cDir = 1	实体向起点-终点左侧等距
			cDir = -1	实体向起点-终点右侧等距
			仅对轨迹中所有实体进行等距，等距后的新轨迹不一定连续
			进行等距时不会修改原始轨迹
			*ppSNewPathList必须传入空指针
**************************************************/
void equidistantPath(SList* pSOriPathList, SList** ppSNewPathList, double dDist, int8 cDir)
{
	if ((*ppSNewPathList) != NULL)
		return;

	SList* pSNewPathList;
	pSNewPathList = initList(ORAM_NUM353, ORAM_NUM354);
	
	int16 sCurrPos;
	sCurrPos = pSOriPathList->cSize - 1;

	//复制轨迹中的实体
	while (sCurrPos >= 0)
	{
		//复制当前实体节点
		SLine* pOriEntity;
		SLine* pNewEntity = NULL;
		pOriEntity = FindNodeData(pSOriPathList, sCurrPos);
		switch (pOriEntity->EntityType)
		{
		case OTHERTYPE:case L1NE:case DOT:
		{
			pNewEntity = mallocGCode(sizeof(SLine), ORAM_NUM355);
			*(SLine*)pNewEntity = *(SLine*)pOriEntity;
			break;
		}
		case ARC:
		{
			pNewEntity = mallocGCode(sizeof(SArc), ORAM_NUM356);
			*(SArc*)pNewEntity = *(SArc*)pOriEntity;
			break;
		}
		default:
			break;
		}

		//等距当前实体
		equidistantEntity(pNewEntity, dDist, cDir);
		
		//将新实体插入新轨迹链表
		insertList(pSNewPathList, 0, pNewEntity, ORAM_NUM354);

		sCurrPos--;
	}

	//返回偏置后的轨迹链表
	*ppSNewPathList = pSNewPathList;

}

/*************************************************
  函数名称：void addArcInequidistanLine(SList* pSOriPathList, SList* pSNewPathList, double dDist)
  函数功能：在等距轨迹中添加圆弧
  输入参数：pSOriPathList		原始轨迹
			pSNewPathList		等距后的轨迹
			dDist		等距距离
  输出参数：
  返 回 值：
  备    注：cDir = 1	实体向起点-终点左侧等距
			cDir = -1	实体向起点-终点右侧等距
			通过函数equidistantPath得到的等距轨迹不一定连续，调用此函数
			添加圆弧使等距轨迹变得连续
**************************************************/
void addArcInequidistanLine(SList* pSOriPathList, SList* pSNewPathList, double dDist)
{
	int16 sCurrPos;
	sCurrPos = getListSize(pSNewPathList) - 1;
	while (sCurrPos >= 1)
	{
		//1.找到当前实体与下一实体
		SLine* pCurrOriEntity;
		SLine* pNextOriEntity;
		SLine* pCurrEntity;
		SLine* pNextEntity;

		pCurrOriEntity = FindNodeData(pSOriPathList, sCurrPos);
		pNextOriEntity = FindNodeData(pSOriPathList, sCurrPos - 1);
		pCurrEntity= FindNodeData(pSNewPathList, sCurrPos);
		pNextEntity = FindNodeData(pSNewPathList, sCurrPos - 1);

		uint8 ucAxisNum[2];
		ucAxisNum[0] = AXIS_Z;
		ucAxisNum[1] = AXIS_X;

		//计算当前实体的终点向量
		SVector SVector1 = { 0 };
		if (pCurrOriEntity->EntityType == ARC)
		{
			SArc* pSArc = (SArc*)pCurrOriEntity;
			double dI = pSArc->SStartPoint.dArray[AXIS_X] - pSArc->SCenterPoint.dArray[AXIS_X];
			double dK = pSArc->SStartPoint.dArray[AXIS_Z] - pSArc->SCenterPoint.dArray[AXIS_Z];
			getArcVectorAtEnd(pSArc->SStartPoint, pSArc->SEndPoint, dK, dI, &SVector1, pSArc->cArcDir, ucAxisNum);
		}
		else if (pCurrOriEntity->EntityType == OTHERTYPE || pCurrOriEntity->EntityType == L1NE)
		{
			getLineVector(pCurrOriEntity->SStartPoint, pCurrOriEntity->SEndPoint, &SVector1, ucAxisNum);
		}

		SVector SVector2 = { 0 };
		//计算下一实体的起点向量
		if (pNextOriEntity->EntityType == ARC)
		{
			SArc* pSArc = (SArc*)pNextOriEntity;
			double dI = pSArc->SStartPoint.dArray[AXIS_X] - pSArc->SCenterPoint.dArray[AXIS_X];
			double dK = pSArc->SStartPoint.dArray[AXIS_Z] - pSArc->SCenterPoint.dArray[AXIS_Z];
			getArcVectorAtStart(dK, dI, &SVector2, pSArc->cArcDir, ucAxisNum);
		}
		else if (pNextOriEntity->EntityType == OTHERTYPE || pNextOriEntity->EntityType == L1NE)
		{
			getLineVector(pNextOriEntity->SStartPoint, pNextOriEntity->SEndPoint, &SVector2, ucAxisNum);
		}

		//若 当前实体 与 下一实体 不相切
		//即 当前实体的终点向量 != 下一实体的起点向量，则插入圆弧
		if (!isZero(SVector1.dArray[AXIS_X] - SVector2.dArray[AXIS_X]) || !isZero(SVector1.dArray[AXIS_Z] - SVector2.dArray[AXIS_Z]))
		{
			SArc* pSArc;
			pSArc = mallocGCode(sizeof(SArc), ORAM_NUM356);
			if (pSArc == NULL)
			{
				parseGCodeErrorB(0, GCODE_ERROR_NUM406);
				return -1;
			}
			pSArc->EntityType = ARC;
			pSArc->SStartPoint = pCurrEntity->SEndPoint;
			pSArc->SEndPoint = pNextEntity->SStartPoint;
			pSArc->SCenterPoint = pCurrOriEntity->SEndPoint;
			pSArc->dRadius = dDist;

			//已知劣弧求方向
			//叉乘 dCross = Ux * Vy - Uy * Vx
			//叉乘 dCross > 0 为顺；叉乘 dCross < 0 为逆
			//U (SZ - CZ ,SX - CX)
			//V (EZ - CZ ,EX - CX)
			double dSX = pSArc->SStartPoint.dArray[AXIS_X];
			double dSZ = pSArc->SStartPoint.dArray[AXIS_Z];
			double dEX = pSArc->SEndPoint.dArray[AXIS_X];
			double dEZ = pSArc->SEndPoint.dArray[AXIS_Z];
			double dCX = pSArc->SCenterPoint.dArray[AXIS_X];
			double dCZ = pSArc->SCenterPoint.dArray[AXIS_Z];
			double dCross;
			dCross = (dSZ - dCZ) * (dEX - dCX) - (dSX - dCX) * (dEZ - dCZ);

			//右等距
			pSArc->cArcDir = isEqualOrGreaterThanZero(dCross) ? ARC_ANTI_CW : ARC_CW;
			//左等距
			//pSArc->cArcDir = isEqualOrGreaterThanZero(dCross) ? ARC_CW : ARC_ANTI_CW;

			insertList(pSNewPathList, sCurrPos, pSArc, ORAM_NUM354);
		}

		sCurrPos--;

	}


}

/*************************************************
  函数名称：int8 deleteRings(SList* pSPathList)
  函数功能：删除环
  输入参数：pSPathList		轨迹链表
  输出参数：
  返 回 值：
  备    注：删除轨迹中的环
**************************************************/
void deleteRings(SList* pSPathList)
{
	//找到链表的第三段轨迹
	int16 sCurrPos;
	sCurrPos = getListSize(pSPathList) - 3;
	//从链表的第三段轨迹开始遍历
	while (sCurrPos >= 0)
	{
		int16 sCheckPos;
		sCheckPos = getListSize(pSPathList) - 1;
		//检查sCurrPos对应实体是否与之前的实体相交
		while (sCheckPos >= sCurrPos + 2)
		{
			//找到sCurrPos实体
			SLine* pEntity1;
			SLine* pEntity2;
			SPointDouble SIntersectionPoint;
			pEntity1 = FindNodeData(pSPathList, sCurrPos);
			pEntity2 = FindNodeData(pSPathList, sCheckPos);
			//需要初始化交点坐标，防止丢失非XZ值
			SIntersectionPoint = pEntity1->SStartPoint;

			//判断是否存在交点
			if (calcIntersectionInTwoEntity((SLine*)pEntity1, (SLine*)pEntity2, &SIntersectionPoint))
			{
				//是否要修正圆弧参数？因为计算存在误差，可能导致圆弧参数错误（起点终点圆心不能构成圆弧）
				//修正pEntity1的起点
				pEntity1->SStartPoint = SIntersectionPoint;
				//修正pEntity2的终点
				pEntity2->SEndPoint = SIntersectionPoint;
				//计算删除节点数量
				int8 cDeleteNum;
				cDeleteNum = sCheckPos - sCurrPos - 1;
				//删除节点
				while (cDeleteNum > 0)
				{
					removeNode(pSPathList, sCurrPos + 1, FALSE);
					cDeleteNum--;
				}
				break;
			}
			else
			{
				sCheckPos--;
			}
				
		
		}

		sCurrPos--;
	}

}


/*************************************************
  函数名称：int8 calcIntersectionInLineAndLine(SLine* pSLine1, SLine* pSLine2, SPointDouble* pSPoint)
  函数功能：计算直线段与直线段交点坐标
  输入参数：pSLine1		直线段1
			pSLine2		直线段2
			pSPoint		交点
  输出参数：
  返 回 值：1	相交
			0	不相交
  备    注：未对两直线重合情况进行考虑？？
**************************************************/
int8 calcIntersectionInLineAndLine(SLine* pSLine1, SLine* pSLine2, SPointDouble* pSPoint)
{
	double dSX_1 = pSLine1->SStartPoint.dArray[AXIS_X]; // Line1起点X
	double dSZ_1 = pSLine1->SStartPoint.dArray[AXIS_Z]; // Line1起点Z
	double dEX_1 = pSLine1->SEndPoint.dArray[AXIS_X];   // Line1终点X
	double dEZ_1 = pSLine1->SEndPoint.dArray[AXIS_Z];   // Line1终点Z

	double dSX_2 = pSLine2->SStartPoint.dArray[AXIS_X]; // Line2起点X
	double dSZ_2 = pSLine2->SStartPoint.dArray[AXIS_Z]; // Line2起点Z
	double dEX_2 = pSLine2->SEndPoint.dArray[AXIS_X];   // Line2终点X
	double dEZ_2 = pSLine2->SEndPoint.dArray[AXIS_Z];   // Line2终点Z

	//计算分母（叉积）
	double denominator = (dSX_1 - dEX_1) * (dSZ_2 - dEZ_2) - (dSZ_1 - dEZ_1) * (dSX_2 - dEX_2);

	//平行或重合，无交点
	if (isZero(denominator))
	{
		return 0; 
	}

	//计算交点坐标
	double numeratorX = (dSX_1 * dEZ_1 - dSZ_1 * dEX_1) * (dSX_2 - dEX_2) -
		(dSX_1 - dEX_1) * (dSX_2 * dEZ_2 - dSZ_2 * dEX_2);
	double numeratorZ = (dSX_1 * dEZ_1 - dSZ_1 * dEX_1) * (dSZ_2 - dEZ_2) -
		(dSZ_1 - dEZ_1) * (dSX_2 * dEZ_2 - dSZ_2 * dEX_2);

	//交点坐标
	double dPX = numeratorX / denominator;
	double dPZ = numeratorZ / denominator;

	//判断交点是否在线段范围内
	if (!isBetweenA2B(dPX, dSX_1, dEX_1) || !isBetweenA2B(dPX, dSX_2, dEX_2) ||
		!isBetweenA2B(dPZ, dSZ_1, dEZ_1) || !isBetweenA2B(dPZ, dSZ_2, dEZ_2))
	{
		return 0; // 交点不在线段范围内
	}

	// 写入交点
	pSPoint->dArray[AXIS_X] = dPX;
	pSPoint->dArray[AXIS_Z] = dPZ;

	return 1; // 存在交点，且在两条线段内
}

/*************************************************
  函数名称：int8 calcIntersectionInLineAndArc(SLine* pSLine, SArc* pSArc, SPointDouble* pSPoint)
  函数功能：计算直线段与圆弧段交点坐标
  输入参数：pSLine		直线段
			pSArc		圆弧段
			pSPoint		交点
  输出参数：
  返 回 值：1	相交
			0	不相交
  备    注：输入的圆弧不能跨象限
**************************************************/
int8 calcIntersectionInLineAndArc(SLine* pSLine, SArc* pSArc, SPointDouble* pSPoint)
{
	// 直线起终点坐标
	double dSX_1 = pSLine->SStartPoint.dArray[AXIS_X];
	double dSZ_1 = pSLine->SStartPoint.dArray[AXIS_Z];
	double dEX_1 = pSLine->SEndPoint.dArray[AXIS_X];
	double dEZ_1 = pSLine->SEndPoint.dArray[AXIS_Z];

	// 圆心坐标及弧段起终点坐标
	double dCX = pSArc->SCenterPoint.dArray[AXIS_X];
	double dCZ = pSArc->SCenterPoint.dArray[AXIS_Z];
	double dSX_2 = pSArc->SStartPoint.dArray[AXIS_X];
	double dSZ_2 = pSArc->SStartPoint.dArray[AXIS_Z];
	double dEX_2 = pSArc->SEndPoint.dArray[AXIS_X];
	double dEZ_2 = pSArc->SEndPoint.dArray[AXIS_Z];

	// 计算半径
	double r = pSArc->dRadius;

	// 直线方向向量
	double dx = dEX_1 - dSX_1;
	double dz = dEZ_1 - dSZ_1;

	// 二次方程系数 A t^2 + B t + C = 0
	double A = dx * dx + dz * dz;
	double B = 2 * (dx * (dSX_1 - dCX) + dz * (dSZ_1 - dCZ));
	double C = (dSX_1 - dCX) * (dSX_1 - dCX) + (dSZ_1 - dCZ) * (dSZ_1 - dCZ) - r * r;

	double discriminant = B * B - 4 * A * C;
	if (discriminant < 0)
	{
		return 0; // 无交点
	}
	if (isZero(A))
	{
		return 0;
	}
	double sqrtDiscriminant = sqrt(discriminant);
	double t1 = (-B - sqrtDiscriminant) / (2 * A);
	double t2 = (-B + sqrtDiscriminant) / (2 * A);

	double ts[2] = { t1, t2 };

	for (int i = 0; i < 2; i++)
	{
		double t = ts[i];
		if (t >= 0 && t <= 1)
		{
			// 计算交点坐标
			double ix = dSX_1 + t * dx;
			double iz = dSZ_1 + t * dz;

			// 判断交点是否在弧段范围内（x,z单调）
			if(isBetweenA2B(ix, dSX_2, dEX_2) && isBetweenA2B(iz, dSZ_2, dEZ_2))
			{
				// 写入交点
				pSPoint->dArray[AXIS_X] = ix;
				pSPoint->dArray[AXIS_Z] = iz;
				return 1; // 找到交点
			}
		}
	}

	return 0; // 无交点
}

/*************************************************
  函数名称：int8 calcIntersectionInArcAndArc(SLine* pSLine, SArc* pSArc, SPointDouble* pSPoint)
  函数功能：计算圆弧段与圆弧段交点坐标
  输入参数：pSArc1		圆弧段1
			pSArc2		圆弧段2
			pSPoint		交点
  输出参数：
  返 回 值：1	相交
			0	不相交
  备    注：输入的圆弧不能跨象限
**************************************************/
int8 calcIntersectionInArcAndArc(SArc* pSArc1, SArc* pSArc2, SPointDouble* pSPoint)
{
	double dCX_1 = pSArc1->SCenterPoint.dArray[AXIS_X];
	double dCZ_1 = pSArc1->SCenterPoint.dArray[AXIS_Z];
	double dSX_1 = pSArc1->SStartPoint.dArray[AXIS_X];
	double dSZ_1 = pSArc1->SStartPoint.dArray[AXIS_Z];
	double dEX_1 = pSArc1->SEndPoint.dArray[AXIS_X];
	double dEZ_1 = pSArc1->SEndPoint.dArray[AXIS_Z];

	double dCX_2 = pSArc2->SCenterPoint.dArray[AXIS_X];
	double dCZ_2 = pSArc2->SCenterPoint.dArray[AXIS_Z];
	double dSX_2 = pSArc2->SStartPoint.dArray[AXIS_X];
	double dSZ_2 = pSArc2->SStartPoint.dArray[AXIS_Z];
	double dEX_2 = pSArc2->SEndPoint.dArray[AXIS_X];
	double dEZ_2 = pSArc2->SEndPoint.dArray[AXIS_Z];

	double dR_1 = pSArc1->dRadius;
	double dR_2 = pSArc2->dRadius;

	double dDX = dCX_2 - dCX_1;
	double dDZ = dCZ_2 - dCZ_1;
	double dDist = sqrt(dDX * dDX + dDZ * dDZ);

	if (dDist > dR_1 + dR_2 || dDist < fabs(dR_1 - dR_2) || isZero(dDist))
	{
		return 0; // 无交点或同心圆
	}

	double dA = (dR_1 * dR_1 - dR_2 * dR_2 + dDist * dDist) / (2 * dDist);
	double dH = sqrt(dR_1 * dR_1 - dA * dA);

	double dVX = dDX / dDist;
	double dVZ = dDZ / dDist;

	double dPX_M = dCX_1 + dA * dVX;
	double dPZ_M = dCZ_1 + dA * dVZ;

	double dInter1X = dPX_M + dH * (-dVZ);
	double dInter1Z = dPZ_M + dH * dVX;

	double dInter2X = dPX_M - dH * (-dVZ);
	double dInter2Z = dPZ_M - dH * dVX;


	if (isBetweenA2B(dInter1X, dSX_1, dEX_1) && isBetweenA2B(dInter1Z, dSZ_1, dEZ_1) &&
		isBetweenA2B(dInter1X, dSX_2, dEX_2) && isBetweenA2B(dInter1Z, dSZ_2, dEZ_2))
	{
		pSPoint->dArray[AXIS_X] = dInter1X;
		pSPoint->dArray[AXIS_Z] = dInter1Z;
		return 1;
	}


	if (isBetweenA2B(dInter2X, dSX_1, dEX_1) && isBetweenA2B(dInter2Z, dSZ_1, dEZ_1) &&
		isBetweenA2B(dInter2X, dSX_2, dEX_2) && isBetweenA2B(dInter2Z, dSZ_2, dEZ_2))

	{
		pSPoint->dArray[AXIS_X] = dInter2X;
		pSPoint->dArray[AXIS_Z] = dInter2Z;
		return 1;
	}

	return 0;
}

/*************************************************
  函数名称：int8 int8 calcIntersectionInTwoEntity(enum ENTITYTYPE* pEntity1, enum ENTITYTYPE* pEntity2, SPointDouble* pSPoint)
  函数功能：计算两实体的交点坐标
  输入参数：pEntity1		实体1
			pEntity2		实体2
			pSPoint		交点
  输出参数：
  返 回 值：1	相交
			0	不相交
  备    注：输入的圆弧不能跨象限
**************************************************/
int8 calcIntersectionInTwoEntity(void* pEntity1, void* pEntity2, SPointDouble* pSPoint)
{
	int8 cRet = 0;
	enum ENTITY EntityType1 = *(enum ENTITY*)pEntity1;
	enum ENTITY EntityType2 = *(enum ENTITY*)pEntity2;

	if (EntityType1 == L1NE && EntityType2 == L1NE)
	{
		SLine* pSLine1 = (SLine*)pEntity1;
		SLine* pSLine2 = (SLine*)pEntity2;
		cRet = calcIntersectionInLineAndLine(pSLine1, pSLine2, pSPoint);
	}
	else if (EntityType1 == L1NE && EntityType2 == ARC)
	{
		SLine* pSLine = (SLine*)pEntity1;
		SArc* pSArc = (SArc*)pEntity2;
		cRet = calcIntersectionInLineAndArc(pSLine, pSArc, pSPoint);
	}
	else if (EntityType1 == ARC && EntityType2 == L1NE)
	{
		SLine* pSLine = (SLine*)pEntity2;
		SArc* pSArc = (SArc*)pEntity1;
		cRet = calcIntersectionInLineAndArc(pSLine, pSArc, pSPoint);
	}
	else if (EntityType1 == ARC && EntityType2 == ARC)
	{
		SArc* pSArc1 = (SArc*)pEntity1;
		SArc* pSArc2 = (SArc*)pEntity2;
		cRet = calcIntersectionInArcAndArc(pSArc1, pSArc2, pSPoint);
	}
	return cRet;
}

//计算交点测试函数
void calcIntersectionInLineAndLine_test()
{
	SLine SLine1;
	SLine SLine2;
	SPointDouble SPoint = { 0 };

	// 线段1: (0,0) -> (4,4)
	SLine1.SStartPoint.dArray[AXIS_X] = 0.0;
	SLine1.SStartPoint.dArray[AXIS_Z] = 0.0;
	SLine1.SEndPoint.dArray[AXIS_X] = 4.0;
	SLine1.SEndPoint.dArray[AXIS_Z] = 4.0;

	// 线段2: (4,4) -> (8,0)
	SLine2.SStartPoint.dArray[AXIS_X] = 4.0;
	SLine2.SStartPoint.dArray[AXIS_Z] = 4.0;
	SLine2.SEndPoint.dArray[AXIS_X] = 8.0;
	SLine2.SEndPoint.dArray[AXIS_Z] = 0.0;

	if (calcIntersectionInLineAndLine(&SLine1, &SLine2, &SPoint))
	{
		printf("相交\n");
		printf("X: %f\n", SPoint.dArray[AXIS_X]);
		printf("Z: %f", SPoint.dArray[AXIS_Z]);
	}
	else
	{
		printf("不相交");
	}
}
void calcIntersectionInLineAndArc_test()
{
	SLine SLine;
	SArc SArc;
	SPointDouble SPoint = { 0 };

	// 直线段: (0,2) -> (6,2)
	SLine.SStartPoint.dArray[AXIS_X] = 10;
	SLine.SStartPoint.dArray[AXIS_Z] = -15;
	SLine.SEndPoint.dArray[AXIS_X] = 0;
	SLine.SEndPoint.dArray[AXIS_Z] = 20;

	// 圆弧段（单调递增）
	SArc.SStartPoint.dArray[AXIS_X] = 0;
	SArc.SStartPoint.dArray[AXIS_Z] = 10;
	SArc.SEndPoint.dArray[AXIS_X] = 10;
	SArc.SEndPoint.dArray[AXIS_Z] = 0;
	SArc.SCenterPoint.dArray[AXIS_X] = 0;
	SArc.SCenterPoint.dArray[AXIS_Z] = 0;
	SArc.dRadius = 10;
	
	if (calcIntersectionInLineAndArc(&SLine, &SArc, &SPoint))
	{
		printf("相交\n");
		printf("X: %f\n", SPoint.dArray[AXIS_X]);
		printf("Z: %f", SPoint.dArray[AXIS_Z]);
	}
	else
	{
		printf("不相交");
	}
}
void calcIntersectionInArcAndArc_test()
{
	SArc SArc1;
	SArc SArc2;
	SPointDouble SPoint = { 0 };

	// 圆弧段（单调递增）
	SArc1.SStartPoint.dArray[AXIS_X] = 0;
	SArc1.SStartPoint.dArray[AXIS_Z] = 1;
	SArc1.SEndPoint.dArray[AXIS_X] = 1;
	SArc1.SEndPoint.dArray[AXIS_Z] = 0;
	SArc1.SCenterPoint.dArray[AXIS_X] = 0;
	SArc1.SCenterPoint.dArray[AXIS_Z] = 0;
	SArc1.dRadius = 1;

	// 圆弧段（单调递增）
	SArc2.SStartPoint.dArray[AXIS_X] = 2;
	SArc2.SStartPoint.dArray[AXIS_Z] = 2;
	SArc2.SEndPoint.dArray[AXIS_X] = 0;
	SArc2.SEndPoint.dArray[AXIS_Z] = 0;
	SArc2.SCenterPoint.dArray[AXIS_X] = 0;
	SArc2.SCenterPoint.dArray[AXIS_Z] = 2;
	SArc2.dRadius = 2;

	if (calcIntersectionInArcAndArc(&SArc1, &SArc2, &SPoint))
	{
		printf("相交\n");
		printf("X: %f\n", SPoint.dArray[AXIS_X]);
		printf("Z: %f", SPoint.dArray[AXIS_Z]);
	}
	else
	{
		printf("不相交");
	}
}


/*************************************************
  函数名称：void calIntersectionPointsInLineWithCircleCenter(SLine* pLine, SPointDouble* pSPoint)
  函数功能：计算直线与圆心交点
  输入参数：pLine		线段
			pSPoint		圆心

  输出参数：
			交点
  备    注：
**************************************************/
//void calIntersectionPointsInLineWithCircleCenter(SLine* pLine, SPointDouble* pSPoint, SPointDouble*psPoint2)
//{
//	SPointDouble SPoint;
//	SPoint = *pSPoint;
//	double dSX_0 = SPoint.dArray[AXIS_X];
//	double dSZ_0 = SPoint.dArray[AXIS_Z];
//
//	double dSX_1 = pLine->SStartPoint.dArray[AXIS_X]; // Line1起点X
//	double dSZ_1 = pLine->SStartPoint.dArray[AXIS_Z]; // Line1起点Z
//	double dEX_1 = pLine->SEndPoint.dArray[AXIS_X];   // Line1终点X
//	double dEZ_1 = pLine->SEndPoint.dArray[AXIS_Z];   // Line1终点Z
//
//	double dT = -((dSX_1- dSX_0)*(dEX_1-dSX_1)+(dSZ_1- dSZ_0)*(dEZ_1-dSZ_1))/((dEX_1- dSX_1)* (dEX_1 - dSX_1)+ (dEZ_1- dSZ_1)* (dEZ_1 - dSZ_1));
//	psPoint2->dArray[AXIS_X] = dSX_1 + dT * (dEX_1 - dSX_1);
//	psPoint2->dArray[AXIS_Z] = dSZ_1 + dT * (dEZ_1 - dSZ_1);
//}

/*************************************************
  函数名称：int8 getArcDir(SArc* pSArc)
  函数功能：获取劣弧的方向
  输入参数：
  输出参数：
  备    注：
**************************************************/
void getArcDir(SArc* pSArc)
{
	//已知劣弧求方向
	//叉乘 dCross = Ux * Vy - Uy * Vx
	//叉乘 dCross > 0 为顺；叉乘 dCross < 0 为逆
	//U (SZ - CZ ,SX - CX)
	//V (EZ - CZ ,EX - CX)
	double dSX = pSArc->SStartPoint.dArray[AXIS_X];
	double dSZ = pSArc->SStartPoint.dArray[AXIS_Z];
	double dEX = pSArc->SEndPoint.dArray[AXIS_X];
	double dEZ = pSArc->SEndPoint.dArray[AXIS_Z];
	double dCX = pSArc->SCenterPoint.dArray[AXIS_X];
	double dCZ = pSArc->SCenterPoint.dArray[AXIS_Z];
	double dCross;
	dCross = (dSZ - dCZ) * (dEX - dCX) - (dSX - dCX) * (dEZ - dCZ);

	//计算方向
	pSArc->cArcDir = isEqualOrGreaterThanZero(dCross) ? ARC_ANTI_CW : ARC_CW;

}
/*************************************************
  函数名称：void calIntersectionPointsInRounded(enum ENTITY* pEntity, SPointDouble* pSPointCenter, double dRadiusCenter, SPointDouble* psPoint)
  函数功能：计算圆角与直线或者圆弧交点
  输入参数：pEntity		        类型
			pSPointCenter		圆角圆心
			dRadiusCenter		圆角半径
            psPoint             交点
  输出参数：
			交点
  备    注：
**************************************************/
void calIntersectionPointsInRounded(enum ENTITY* pEntity, SPointDouble* pSPointCenter, double dRadiusCenter, SPointDouble* psPoint)
{	
	//圆角圆心
	SPointDouble SPoint;
	SPoint = *pSPointCenter;
	double dSX_0 = SPoint.dArray[AXIS_X];
	double dSZ_0 = SPoint.dArray[AXIS_Z];
	switch (*pEntity)
	{
		case L1NE: 
			SLine* pSLine = (SLine*)pEntity;
			double dSX_1 = pSLine->SStartPoint.dArray[AXIS_X]; // Line1起点X
			double dSZ_1 = pSLine->SStartPoint.dArray[AXIS_Z]; // Line1起点Z
			double dEX_1 = pSLine->SEndPoint.dArray[AXIS_X];   // Line1终点X
			double dEZ_1 = pSLine->SEndPoint.dArray[AXIS_Z];   // Line1终点Z

			double dT = -((dSX_1- dSX_0)*(dEX_1-dSX_1)+(dSZ_1- dSZ_0)*(dEZ_1-dSZ_1))/((dEX_1- dSX_1)* (dEX_1 - dSX_1)+ (dEZ_1- dSZ_1)* (dEZ_1 - dSZ_1));
			psPoint->dArray[AXIS_X] = dSX_1 + dT * (dEX_1 - dSX_1);
			psPoint->dArray[AXIS_Z] = dSZ_1 + dT * (dEZ_1 - dSZ_1);
            break;
		case ARC:
			SArc* pSArc = (SArc*)pEntity;
			double dCX_1 = pSArc->SCenterPoint.dArray[AXIS_X]; // SArc圆心X
			double dCZ_1 = pSArc->SCenterPoint.dArray[AXIS_Z]; // SArc圆心Z
			double dx = dCX_1 - dSX_0;
			double dz = dCZ_1 - dSZ_0;
			double dRadius = pSArc->dRadius;                  // SArc半径
			double dRadio = dRadiusCenter / (dRadiusCenter + dRadius);  // 圆心半径比
			psPoint->dArray[AXIS_X] = dSX_0 + dRadio * dx;
			psPoint->dArray[AXIS_Z] = dSZ_0 + dRadio * dz;
			break;
		default:
			break;
	}
}

/*************************************************
  函数名称：
  函数功能  获取有效轴的值（弃用）
  输入参数：pSStartPoint		起点坐标地址
			pSEndPoint			终点坐标地址
			bIsLiner			轨迹为线性
			cInterpolationState	插补模式
			sGc					插补平面
			pValStatus			有效轴的值（输出）
  输出参数：
  返 回 值：
  备    注：有效轴（结构体轴）为加工程序段时产生运动的轴，该值由实体类型运动决定
			1.极坐标插补，不管是否发生运动，横轴纵轴均为有效轴
			2.！！！有效轴后续不在G代码解析时标记
**************************************************/
void getValStatus(SPointDouble* pSStartPoint, SPointDouble* pSEndPoint, bool bIsLiner, int8 cInterpolationState, int16 sGc, VAL_STRUCT_TYPE* pValStatus)
{
	if (pSStartPoint == NULL || pSEndPoint == NULL || pValStatus == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//临时变量，用于遍历各个轴的坐标
	uint8 ucTemp;
	//横轴，用于根据插补模式获取横轴用
	uint8 ucHorAxis;
	//纵轴，用于根据插补模式获取纵轴用
	uint8 ucVerAxis;

	//1.根据插补模式获取插补平面的横轴和纵轴
	int8 cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, cInterpolationState, sGc);   //获取横轴，纵轴
	if (cRet < 0)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM301);
		return;
	}
	//2.1极坐标插补模式
	if (cInterpolationState == 3)
	{
		//设置横轴为有效轴
		setbit(*pValStatus, ucHorAxis);
		//设置纵轴为有效轴
		setbit(*pValStatus, ucVerAxis);
	}
	//2.2圆柱插补模式
	else if (cInterpolationState == 5)
	{
		//设置横轴为有效轴
		setbit(*pValStatus, ucHorAxis);
		//设置纵轴为有效轴
		setbit(*pValStatus, ucVerAxis);
	}
	//2.3普通插补模式
	else
	{
		//获取移动轴
		for (ucTemp = 0; ucTemp < AXIS_NUM_MAX; ucTemp++)
		{
			if (!isZero(pSEndPoint->dArray[ucTemp] - pSStartPoint->dArray[ucTemp]))
			{
				setbit(*pValStatus, ucTemp);
			}
		}
		//非线性运动，有效轴为加工平面的轴号
		if (!bIsLiner)
		{
			//设置横轴为有效轴
			setbit(*pValStatus, ucHorAxis);
			//设置纵轴为有效轴
			setbit(*pValStatus, ucVerAxis);
		}
	}
}

/*************************************************
  函数名称：int8 getTurretNumAndTurretAxis(int8 cToolNum, int8* pTurretNum, VAL_STRUCT_TYPE* pTurretAxis)
  函数功能  获取刀塔号及刀塔轴（弃用）
  输入参数：cToolNum		刀具号
			pTurretNum		刀塔号地址（输出）
			pTurretAxis		刀塔轴地址（输出）
  输出参数：
  返 回 值：-1	报错
			1
  备    注：刀塔轴（物理轴）：加工程序段时会影响加工的轴，该值由机床结构决定
			暂时不能使用该函数
**************************************************/
int8 getTurretNumAndTurretAxis(int8 cToolNum, int8* pTurretNum, VAL_STRUCT_TYPE* pTurretAxis)
{
	//if (pTurretNum == NULL || pTurretAxis == NULL)
	//{
	//	//报错
	//	return -1;
	//}

	////1.遍历数组TurretToToolMap，找到当前刀具号所在的刀塔号
	//int8 cTurretCount;
	//int8 cToolCount;
	//bool bFindTurret = 0;
	//for (cTurretCount = 0; cTurretCount < TURRET_NUM_MAX; cTurretCount++)
	//{
	//	for (cToolCount = 0; cToolCount < TOOLPOS_NUM_MAX; cToolCount++)
	//	{
	//		if (g_SSystemPara.TurretToToolMap[cTurretCount][cToolCount] == cToolNum)
	//		{
	//			*pTurretNum = cTurretCount;
	//			bFindTurret = TRUE;
	//			break;
	//		}
	//	}
	//	if (bFindTurret)
	//	{
	//		break;
	//	}
	//}
	//if (bFindTurret == FALSE)
	//{
	//	//未找到刀具号
	//	return -1;
	//}

	////2.通过TurretToAxisMap及刀塔号，获取刀塔轴
	//*pTurretAxis = g_SSystemPara.TurretToAxisMap[*pTurretNum];

}









//AI写的，待测试
double CalculateZAtX_LINE(SLine* pSLineInfo, double dX) 
{
	// 获取起点和终点的x和z坐标
	double x1 = pSLineInfo->SStartPoint.dArray[0];
	double z1 = pSLineInfo->SStartPoint.dArray[2];
	double x2 = pSLineInfo->SEndPoint.dArray[0];
	double z2 = pSLineInfo->SEndPoint.dArray[2];

	// 检查线段是否垂直于x轴
	if (x2 - x1 == 0) {
		// 如果线段垂直于x轴，则返回起点的z值（因为z值不随x变化）
		return z1;
	}

	// 使用线性插值公式计算z值
	double z = z1 + (dX - x1) * (z2 - z1) / (x2 - x1);
	return z;
}
//AI写的，待测试
double CalculateZAtX_ARC(SArc* pSArcInfo, double dX) 
{
	// 获取圆心的x和z坐标，以及半径
	double xC = pSArcInfo->SCenterPoint.dArray[0];
	double zC = pSArcInfo->SCenterPoint.dArray[2];
	double radius = pSArcInfo->dRadius;

	// 计算x方向上与圆弧对应的点到圆心的水平距离
	double dx = dX - xC;

	// 检查给定的x是否在圆弧的范围内
	double startX = pSArcInfo->SStartPoint.dArray[0];
	double endX = pSArcInfo->SEndPoint.dArray[0];
	if ((dX < startX && dX < endX) || (dX > startX && dX > endX)) {
		// 如果给定的x不在起点和终点范围内，则返回错误值
		return NAN; // 非数值，表示错误
	}

	// 计算z值（基于圆的方程：(x - xC)^2 + (z - zC)^2 = radius^2）
	double dz_squared = radius * radius - dx * dx;

	// 如果结果为负数，表示给定x无解
	if (dz_squared < 0) {
		return NAN; // 无解
	}

	// 根据圆弧方向选择合适的z解
	double dz = sqrt(dz_squared);
	double z1 = zC + dz;
	double z2 = zC - dz;

	// 判断哪个z解符合圆弧的方向和范围
	double startZ = pSArcInfo->SStartPoint.dArray[2];
	double endZ = pSArcInfo->SEndPoint.dArray[2];
	if (pSArcInfo->cArcDir > 0) {
		// 顺时针方向，返回较小的z值
		return (z1 >= startZ && z1 <= endZ) ? z1 : z2;
	}
	else {
		// 逆时针方向，返回较大的z值
		return (z2 >= startZ && z2 <= endZ) ? z2 : z1;
	}
}

