#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"

static SG28Para* pSG28ParaCheck[CHANNEL_NUM_MAX];	//检查流程使用
static SG28Para* pSG28ParaAssign[CHANNEL_NUM_MAX];	//赋值流程使用

//添加G28规则结构体
void addG28RegulationAPI()
{
	//指令：G28
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(28, sGz, 1, mallocG28ParaStruct, freeG28ParaStruct, getG28Para, CheckG28, AssignG28);
}

/*************************************************
  函数名称：int8 mallocG28ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G28参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG28ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG28ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG28Para), ORAM_NUM452);
		if (pSG28ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG28ParaCheck[ucChannelNum], 0, sizeof(SG28Para));
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG28ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG28Para), ORAM_NUM453);
		if (pSG28ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG28ParaAssign[ucChannelNum], 0, sizeof(SG28Para));
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
int8 freeG28ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG28ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG28ParaCheck[ucChannelNum]);
			pSG28ParaCheck[ucChannelNum] = NULL;
			return 1;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG28ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG28ParaAssign[ucChannelNum]);
			pSG28ParaAssign[ucChannelNum] = NULL;
			return 1;
		}
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
int8 getG28Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	if (ucMode == GC_SYNTAX)
	{
		pSG28ParaCheck[ucChannelNum]->cStep = 0;
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG28ParaAssign[ucChannelNum]->cStep = 0;
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
int8 CheckG28(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	return 2;
}


/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG28(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	switch (pSG28ParaAssign[pSAssignConfig->ucChannelNum]->cStep)
	{
	//1.写入当前点->坐标点
	case 0:
	{
		//1.填写SLineInfo
		SLine SLineInfo;//存直线段信息（可能要将此变量放在赋值结构体中）
		bool bIsAbsMethod;
		memset(&SLineInfo, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo.SEndPoint = SLineInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		//说明：运动与手册描述不同，控制参数中的轴回到机床零点
		if (pSAssignConfig->SScanContainer.bXYZ[0]|| pSAssignConfig->SScanContainer.bUVW[0])
		{
			SLineInfo.SEndPoint.dArray[AXIS_X] = 0;
		}
		if (pSAssignConfig->SScanContainer.bXYZ[2] || pSAssignConfig->SScanContainer.bUVW[2])
		{
			SLineInfo.SEndPoint.dArray[AXIS_Z] = 0;
		}
		//1.3
		SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;

		//2.将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);

		pSG28ParaAssign[pSAssignConfig->ucChannelNum]->cStep = 2;

		return 2;
		break;
	}
	default:
		break;
	}
}

