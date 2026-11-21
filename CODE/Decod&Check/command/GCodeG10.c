#include "GCode.h"
#include "ErrorFunction.h"
#include "GeneralFunctions.h"

//指令格式： G10 D_ U_ W_
//           D_:补偿刀号
//           U： 补偿X方向的磨损
//           W:_补偿Z方向的磨损




//添加G10规则结构体
//void addG10RegulationAPI()
//{
//	//指令：G50
//	//组别：sGz
//	//指令占程序段数：1
//	addGCommadRegulation(10, sGz, 1, NULL, NULL, NULL, CheckG10, AssignG10);
//}

//添加G1规则结构体
void addG10RegulationAPI()
{
	//指令：G10
	//组别：sGz
	//运行指令字：32位
	//指令占程序段数：1
	//参数结构体：无
	//char8* str=“”
	addGCommadRegulation(10, sGz, 1, NULL, NULL, NULL, CheckG10, AssignG10);
}

/*************************************************
  函数名称：CheckG10
  函数功能：检查刀号并记录本条G10磨耗增量，更新检查阶段的预测偏置
  输入参数：pSSyntaxConfig 检查配置；U/W按当前G20/G21单位输入
  输出参数：pSGCodeBufUnit 记录目标刀及毫米增量；X按U/2保存径向量
  返 回 值：2 检查完成；-1 刀号错误
  备    注：不修改实时刀表；预测磨耗仍用于后续解析
**************************************************/
int8 CheckG10(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	uint8 ucToolID;
	double dToolID;
	double dU;
	double dW;
	double dUnitScale;
	//刀号检查
	dToolID = pSSyntaxConfig->SScanContainer.dD;
	if (dToolID < 1 || dToolID > TOOL_NUM_MAX || !isZero(dToolID - (int32)dToolID))
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM129);
		return -1;
	}

	ucToolID = (uint8)dToolID;
	dU = pSSyntaxConfig->SScanContainer.dUVW[0];
	dW = pSSyntaxConfig->SScanContainer.dUVW[2];
	//G20 下 U/W 英寸转毫米
	dUnitScale = pSSyntaxConfig->SGCodeCurrSysState.bImperialProgramming ? 25.4 : 1.0;
	dU *= dUnitScale;
	dW *= dUnitScale;

	if (pSSyntaxConfig->SScanContainer.bUVW[0])
	{
	    int mode = getShowByDiameter(getMapValue(pSSyntaxConfig->ucChannelNum, AXIS_X));
	if (mode < 0)
	{
		parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM150);
		return -1;
	}
	if (mode == 1)
		dU /= 2.0;
	}

	//1.更新解析阶段的预测磨耗，不写实时刀表
	//pSSyntaxConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1].dArray[AXIS_X] += dU / 2;
	pSSyntaxConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1].dArray[AXIS_X] += dU;
	pSSyntaxConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1].dArray[AXIS_Z] += dW;
	//pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolWearOffset = pSSyntaxConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1];
	if (pSSyntaxConfig->SGCodeCurrSysState.ucToolCompNum == ucToolID)
	{
		pSSyntaxConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolWearOffset = pSSyntaxConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1];
	}

	//2.记录本条G10的磨耗增量，执行时累加到实时刀表
	pSGCodeBufUnit->uG10ToolID = ucToolID;
	pSGCodeBufUnit->cPointType = G10;
	//dU 已是 X 的径向磨耗增量，单位为毫米。
	memset(&pSGCodeBufUnit->SPointInfo, 0, sizeof(pSGCodeBufUnit->SPointInfo));
	//pSGCodeBufUnit->SPointInfo.dArray[AXIS_X] = dU / 2.0;
	pSGCodeBufUnit->SPointInfo.dArray[AXIS_X] = dU;
	pSGCodeBufUnit->SPointInfo.dArray[AXIS_Z] = dW;

	//3.更新状态结构体中的总偏置
	calculateTotalOffset(&pSSyntaxConfig->SGCodeCurrSysState);

	return 2;

}

/*************************************************
  函数名称：AssignG10
  函数功能：记录本条G10磨耗增量，更新赋值阶段的预测偏置
  输入参数：pSAssignConfig 赋值配置；沿用现有刀号和U/W输入契约
  输出参数：pSGCodeBufUnit 记录目标刀及毫米增量；X按U/2保存径向量
  返 回 值：2 赋值完成
  备    注：实时刀表由Motion前向消费时累加，避免旧绝对值覆盖面板修改
**************************************************/
int8 AssignG10(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeBufUnit)
{
	uint8 ucToolID;
	double dToolID; //保留原始刀号，检查通过后再转换为整数
	double dU;
	double dW;
	double dUnitScale;

	//宏变量在检查后可能变化，赋值时须在刀号转换和磨耗写入前重新校验。
	dToolID = pSAssignConfig->SScanContainer.dD;
	if (!isfinite(dToolID) || dToolID < 1 || dToolID > TOOL_NUM_MAX || !isZero(dToolID - (int32)dToolID))
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM129);
		return -1;
	}
	ucToolID = (uint8)dToolID;

	dU = pSAssignConfig->SScanContainer.dUVW[0];
	dW = pSAssignConfig->SScanContainer.dUVW[2];
	//G20 下 U/W 英寸转毫米
	dUnitScale = pSAssignConfig->SGCodeCurrSysState.bImperialProgramming ? 25.4 : 1.0;
	dU *= dUnitScale;
	dW *= dUnitScale;

	if (pSAssignConfig->SScanContainer.bUVW[0])
	{
		int mode = getShowByDiameter(getMapValue(pSAssignConfig->ucChannelNum, AXIS_X));
	if (mode < 0)
	 {
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM150);
		return -1;
	}
	 if (mode == 1)
		 dU /= 2.0;
	}

	//1.更新解析阶段的预测磨耗，不写实时刀表
	//pSAssignConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1].dArray[AXIS_X] += dU/2;
	pSAssignConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1].dArray[AXIS_X] += dU;
	pSAssignConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1].dArray[AXIS_Z] += dW;
	//pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolWearOffset = pSAssignConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1];
	if (pSAssignConfig->SGCodeCurrSysState.ucToolCompNum == ucToolID)
   {
	   pSAssignConfig->SGCodeCurrSysState.SAllOffsetOfCoord.SToolWearOffset = pSAssignConfig->SGCodeCurrSysState.SToolWearPara[ucToolID - 1];
    }

	//2.记录本条G10的磨耗增量，执行时累加到实时刀表
	pSGCodeBufUnit->uG10ToolID = ucToolID;
	pSGCodeBufUnit->cPointType = G10;
	//dU 已是 X 的径向磨耗增量，单位为毫米。
	memset(&pSGCodeBufUnit->SPointInfo, 0, sizeof(pSGCodeBufUnit->SPointInfo));
	//pSGCodeBufUnit->SPointInfo.dArray[AXIS_X] = dU / 2.0;
	pSGCodeBufUnit->SPointInfo.dArray[AXIS_X] = dU;
	pSGCodeBufUnit->SPointInfo.dArray[AXIS_Z] = dW;

	//3.更新状态结构体中的总偏置
	calculateTotalOffset(&pSAssignConfig->SGCodeCurrSysState);

	return 2;
}