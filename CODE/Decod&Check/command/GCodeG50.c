
#include "GCode.h"








//添加G50规则结构体
void addG50RegulationAPI()
{
	//指令：G50
	//组别：sGz
	//指令占程序段数：1
	addGCommadRegulation(50, sGz, 1, NULL, NULL, NULL, CheckG50, AssignG50);
}


int8 CheckG50(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	//1.G50为限制主轴转速
	if (pSSyntaxConfig->SScanContainer.bS == TRUE)
	{
		// 1.指令字符检查
		if (checkParaLetter("S", &pSSyntaxConfig->SScanContainer) == -1) 
		{
			return -1;
		}
		//1.1取消对主轴转速的赋值
		pSGCodeBufUnit->dSpindleSpeed = pSSyntaxConfig->SGCodeCurrSysState.dSpindleSpeed;
		pSGCodeBufUnit->dSecondSpindleSpeed = pSSyntaxConfig->SGCodeCurrSysState.dSecondSpindleSpeed;

		//1.2赋值主轴转速限制
		pSGCodeBufUnit->dLimitedSpindleSpeed = pSSyntaxConfig->SScanContainer.dS;
		return 2;

	}

	//2.设置G50偏置值
	else
	{
		// 1.指令字符检查
		if (checkParaLetter("XYZUVW", &pSSyntaxConfig->SScanContainer) == -1)
		{
			return -1;
		}
		//1.获取当前点机械坐标与当前G50偏置值
		SPointDouble SCurrPoint = pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;

		SPointDouble SCurrTotalOffset = pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset;

		SPointDouble SCurrG50Offset = pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SG50Offset;

		SPointDouble SNewG50Offset = { 0 };

		//2.计算新G50偏置
		calculateG50Offset(GC_SYNTAX, pSSyntaxConfig->ucChannelNum, &SCurrPoint, &SCurrG50Offset, &SCurrTotalOffset, &SNewG50Offset, &pSSyntaxConfig->SScanContainer, pSSyntaxConfig->SGCodeCurrSysState.bImperialProgramming);

		//3.写入缓冲区中(没必要)
		//pSGCodeBufUnit->pSG50Offset = mallocGCode(sizeof(SPointDouble), 0);
		//*(pSGCodeBufUnit->pSG50Offset) = SNewG50Offset;

		//4.更新状态结构体中的G50偏移值
		pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SG50Offset = SNewG50Offset;

		//5.更新G代码系统状态总偏置
		calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

		return 2;
	}


}


//赋值G50
int8 AssignG50(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	//1.G50为限制主轴转速
	if (pSAssignConfig->SScanContainer.bS == TRUE)
	{
		//1.1取消对主轴转速的赋值
		pSGCodeBufUnit->dSpindleSpeed = pSAssignConfig->SGCodeCurrSysState.dSpindleSpeed;
		pSGCodeBufUnit->dSecondSpindleSpeed = pSAssignConfig->SGCodeCurrSysState.dSecondSpindleSpeed;
		
		//1.2赋值主轴转速限制
		pSGCodeBufUnit->dLimitedSpindleSpeed = pSAssignConfig->SScanContainer.dS;
	}
	
	//2.设置G50偏置值
	else
	{
		//1.获取当前点机械坐标与当前G50偏置值
		SPointDouble SCurrPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;

		SPointDouble SCurrTotalOffset = pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.STotalOffset;
		
		SPointDouble SCurrG50Offset = pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SG50Offset;
		
		SPointDouble SNewG50Offset = { 0 };

		//2.计算新G50偏置
		calculateG50Offset(GC_ASSIGN, pSAssignConfig->ucChannelNum, &SCurrPoint, &SCurrG50Offset, &SCurrTotalOffset, &SNewG50Offset, &pSAssignConfig->SScanContainer, pSAssignConfig->SGCodeCurrSysState.bImperialProgramming);

		//3.写入缓冲区中
		pSGCodeBufUnit->cPointType = G50;
		pSGCodeBufUnit->SPointInfo = SNewG50Offset;

		//4.更新状态结构体中的G50偏移值
		pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SG50Offset = SNewG50Offset;

		//5.更新G代码系统状态总偏置
		calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	}

	return 2;
}

 /*************************************************
  函数名称：calculateG50Offset
  函数功能：根据本段坐标设定计算新的G50偏置
  输入参数：ucMod、ucChannelNum用于查询原有ABC编程方式；
            机械坐标和已有偏置中的直线轴分量均为毫米；
            pSScan保留本段原始编程数值；
            bImperialProgramming为当前检查或赋值阶段的英制标志
  输出参数：pSNewG50Offset保存新偏置，未指定轴保留原值；
            空指针检查失败时不写输出
  返 回 值：1成功；-1存在空指针参数
  备    注：只换算XYZ/UVW，不修改扫描数据；
            不改变ABC分支
**************************************************/
int8 calculateG50Offset(uint8 ucMod, uint8 ucChannelNum, SPointDouble* pSCurrMacPoint, SPointDouble* pSCurrG50Offset, SPointDouble* pSCurrTotalOffset, SPointDouble* pSNewG50Offset, SScan* pSScan, bool bImperialProgramming)
{
	//1.参数检查
	if (pSCurrMacPoint == NULL || pSCurrG50Offset == NULL || pSCurrTotalOffset == NULL || pSNewG50Offset == NULL || pSScan == NULL)
	{
		return -1;
	}

	/*
     * 仅在使用本段长度时换算为毫米，不回写扫描值，
     * 避免同一输入再次处理时重复乘25.4。
     */
    double dUnitScale = bImperialProgramming ? 25.4 : 1.0;

	//2.初始化新G50偏置值
	*pSNewG50Offset = *pSCurrG50Offset;

	int8 cTemp;

	//3.获取其他偏置值（G50偏置以外的偏置值之和）
	SPointDouble SOtherOffset = { 0 };
	for (cTemp = 0; cTemp < AXIS_NUM_MAX; cTemp++)
	{
		SOtherOffset.dArray[cTemp] = pSCurrTotalOffset->dArray[cTemp] - pSCurrG50Offset->dArray[cTemp];
	}

	//计算新G50偏置值
	for (cTemp = 0; cTemp < 3; cTemp++)
	{
		//直径轴的编程值换算为真实机械位移。 
		double dCoordScale = dUnitScale * (g_SSystemPara.SAxisPara[cTemp].bShowByDiameter ? 0.5 : 1.0);
		//1.计算新G50偏置值的XYZ
		//1.1程序段出现XYZ，设定当前位置的工件坐标
		if (pSScan->bXYZ[cTemp] == TRUE)
		{
			pSNewG50Offset->dArray[cTemp] = pSCurrMacPoint->dArray[cTemp]
        	- pSScan->dXYZ[cTemp] * dCoordScale - SOtherOffset.dArray[cTemp];
		}
		//1.2程序段未出现XYZ，出现UVW，增量调整坐标设定
		else if (pSScan->bUVW[cTemp] == TRUE)
		{
			 pSNewG50Offset->dArray[cTemp] = pSCurrG50Offset->dArray[cTemp] - pSScan->dUVW[cTemp] * dCoordScale;
		}

		//2.计算新G50偏置值的ABC
		//2.1当前为绝对值编程
		if (isAbsProgrammingMethod(ucMod, ucChannelNum))
		{
			if (pSScan->bABC[cTemp] == TRUE)
			{
				pSNewG50Offset->dArray[cTemp + 3] = pSCurrMacPoint->dArray[cTemp + 3] - pSScan->dABC[cTemp ] - SOtherOffset.dArray[cTemp + 3];

			}
		}
		//2.2当前为相对值编程
		else
		{
			if (pSScan->bABC[cTemp] == TRUE)
			{
				pSNewG50Offset->dArray[cTemp + 3] = pSCurrG50Offset->dArray[cTemp + 3] - pSScan->dABC[cTemp];

			}
		}
	}
	
	return 1;

}
