#include "common.h"
#include "GCode.h"
#include"ErrorFunction.h"

#define OUTPUT_MAX_MS  86400000
static int8 ConvertSecondsToMsChecked(double sec, int32* outMs, int32 maxMs);
//*******************************M20***************************************
//添加M20规则
void addM20RegulationAPI()
{
	addMCommadRegulation(20, IO_PRE_ACTION, cMj, 1, NULL, NULL, NULL, NULL, AssignM20);
}

int8 AssignM20(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

	//获取赋值结构体中的扫描结构体
	SScan* pScan = &pSAssignConfig->SScanContainer;

	//1.参数K  赋值外部输出端口以及外部输出IO控制的电平类型
	if (pScan->bRIJK[3])
	{
		pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)fabs(pScan->dRIJK[3]);
		pGCodeBufUnit->SIOContrlInfo.cOutputControlType= HIGHLEVEL;
	}
	else
	{
		pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;
		pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
	}

	// 参数J 赋值外部输出持续时间
	if (pScan->bRIJK[2])
	{
	    int32 ms = 0;
	    // 这里 maxMs 需要你根据字段定义决定：
	    // - 若 nOutputRunTims 是 int32，maxMs 可设 24h=86400000 或协议上限
	    // - 若是 uint16/int16，则 maxMs 必须 <= 65535/32767
	    if (ConvertSecondsToMsChecked(pScan->dRIJK[2], &ms, OUTPUT_MAX_MS) < 0)
	    {
//	    	  pSAssignConfig->uErrCode = GCODE_ERROR_NUM30;
//	    	  pSAssignConfig->nErrArg  = 9999; // 或 rawK
	    	  parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
	        return -1; // **门禁：J 非法不入队**
	    }
	    pGCodeBufUnit->SIOContrlInfo.nOutputRunTims = ms;
	}
	else
	{
		pGCodeBufUnit->SIOContrlInfo.nOutputRunTims = UNASSIGNED_VALUE;
	}

	// K 参数校验 + 赋值
	if (pScan->bRIJK[3])
	{
	    double raw = fabs(pScan->dRIJK[3]);

	    // **门禁1：必须整数**
	    if (raw != floor(raw)){
	  	  parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
		  return -1;
	    }


	    // **门禁2：范围**
	    if (raw < 0.0 || raw > 37.0) {
	    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
	    	return -1;
	    }

	    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)raw;

	    // 这里按 M20/M21/M22 设控制类型
	    // M20: HIGHLEVEL, M21: LOWLEVEL, M22: PULSE_TYPE
	}
	else
	{
	    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;
	    // 若协议要求必须给K，则这里应 return -1，而不是 UNASSIGNED
	}

	setIO(M20, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M21***************************************
//添加M21规则
void addM21RegulationAPI()
{
	addMCommadRegulation(21, IO_PRE_ACTION, cMj, 1, NULL, NULL, NULL, NULL, AssignM21);
}

int8 AssignM21(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

	//获取赋值结构体中的扫描结构体
	SScan* pScan = &pSAssignConfig->SScanContainer;

	//1.参数K  赋值外部输出端口以及外部输出IO控制的电平类型
	if (pScan->bRIJK[3])
	{
		pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)fabs(pScan->dRIJK[3]);
		pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
	}
	else
	{
		pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;
		pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
	}

	// 参数J 赋值外部输出持续时间
	if (pScan->bRIJK[2])
	{
	    int32 ms = 0;
	    // 这里 maxMs 需要你根据字段定义决定：
	    // - 若 nOutputRunTims 是 int32，maxMs 可设 24h=86400000 或协议上限
	    // - 若是 uint16/int16，则 maxMs 必须 <= 65535/32767
	    if (ConvertSecondsToMsChecked(pScan->dRIJK[2], &ms, OUTPUT_MAX_MS) < 0)
	    {
	    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
	        return -1; // **门禁：J 非法不入队**
	    }
	    pGCodeBufUnit->SIOContrlInfo.nOutputRunTims = ms;
	}
	else
	{
		pGCodeBufUnit->SIOContrlInfo.nOutputRunTims = UNASSIGNED_VALUE;
	}
	// K 参数校验 + 赋值
	if (pScan->bRIJK[3])
	{
	    double raw = fabs(pScan->dRIJK[3]);

	    // **门禁1：必须整数**
	    if (raw != floor(raw)) {
	    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
	    	return -1;
	    }

	    // **门禁2：范围**
	    if (raw < 0.0 || raw > 37.0) {
	    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
	    	return -1;
	    }

	    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)raw;

	    // 这里按 M20/M21/M22 设控制类型
	    // M20: HIGHLEVEL, M21: LOWLEVEL, M22: PULSE_TYPE
	}
	else
	{
	    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;
	    // 若协议要求必须给K，则这里应 return -1，而不是 UNASSIGNED
	}

	setIO(M21, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}


//**********************************M22****************************************
//添加M22规则
void addM22RegulationAPI()
{
	addMCommadRegulation(22, IO_PRE_ACTION, cMj, 1, NULL, NULL, NULL, NULL, AssignM22);
}

int8 AssignM22(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	//获取赋值结构体中的扫描结构体
	SScan* pScan = &pSAssignConfig->SScanContainer;

	//1.参数K  赋值外部输出端口以及外部输出IO控制的电平类型
	if (pScan->bRIJK[3])
	{
		pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)fabs(pScan->dRIJK[3]);
		pGCodeBufUnit->SIOContrlInfo.cOutputControlType = PULSE_TYPE;
	}
	else
	{
		pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;
		pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
	}



	// 参数J 赋值外部输出持续时间
	if (pScan->bRIJK[2])
	{
	    int32 ms = 0;
	    // 这里 maxMs 需要你根据字段定义决定：
	    // - 若 nOutputRunTims 是 int32，maxMs 可设 24h=86400000 或协议上限
	    // - 若是 uint16/int16，则 maxMs 必须 <= 65535/32767
	    if (ConvertSecondsToMsChecked(pScan->dRIJK[2], &ms, OUTPUT_MAX_MS) < 0)
	    {
	        return -1; // **门禁：J 非法不入队**
	    }
	    pGCodeBufUnit->SIOContrlInfo.nOutputRunTims = ms;
	}
	else
	{
		pGCodeBufUnit->SIOContrlInfo.nOutputRunTims = UNASSIGNED_VALUE;
	}

	// K 参数校验 + 赋值
	if (pScan->bRIJK[3])
	{
	    double raw = fabs(pScan->dRIJK[3]);

	    // **门禁1：必须整数**
	    if (raw != floor(raw)) return -1;

	    // **门禁2：范围**
	    if (raw < 0.0 || raw > 37.0) return -1;

	    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)raw;

	    // 这里按 M20/M21/M22 设控制类型
	    // M20: HIGHLEVEL, M21: LOWLEVEL, M22: PULSE_TYPE
	}
	else
	{
	    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;
	    // 若协议要求必须给K，则这里应 return -1，而不是 UNASSIGNED
	}

	setIO(M22, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M35**************************************
//添加M35规则
void addM35RegulationAPI()
{
	addMCommadRegulation(35, IO_PRE_ACTION, cMj, 1, NULL, NULL, NULL, NULL, AssignM35);
}

//赋值M35
int8 AssignM35(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

	//获取赋值结构体中的扫描结构体
	SScan* pScan = &pSAssignConfig->SScanContainer;

    // *** 关键门禁 1：I 必须存在，否则不允许入队 M35 ***
    if (!pScan->bRIJK[1])
    {
        // 不 setIO(M35)
    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
        return -1;
    }
    double raw = fabs(pScan->dRIJK[1]);

    // 1) 必须为整数（避免 1.2 这种值）
    if (raw != floor(raw)) {
    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
        return -1;
    }

    // 2) 必须在合法范围
    if (raw < 0.0 || raw > 37.0) {
    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
        return -1;
    }

    // I 存在 -> 赋值输出 IO 索引（注意：这里不要 *1000）
    pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)raw;
    pGCodeBufUnit->SIOContrlInfo.cOutputControlType = HIGHLEVEL;

    // *** 关键门禁 2：I 必须合法（防止后续 runM35Command 越界）***
    if (pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex < 0 ||
        pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex >= 38)
    {
        // 不 setIO(M35)
    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
        return -1;
    }
	//存在参数L或K
	// L，K：IO输入端口号
	// L的IO电平有效类型为低电平有效
	//K的IO电平有效类型为高电平有效
	if (pScan->bL || pScan->bRIJK[3])
	{
		//1.参数L和K同时存在，IO输入端口以及IO输入IO电平有效类型的赋值以K为准，忽略L
		if (pScan->bL && pScan->bRIJK[3])
		{
			pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = (int8)fabs(pScan->dRIJK[3]);
			pGCodeBufUnit->SIOContrlInfo.cInputValidType = HIGHLEVEL_VALID;
		}
		//只存在L
		else if (pScan->bL)
		{
			pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = (int8)fabs(pScan->dL);
			pGCodeBufUnit->SIOContrlInfo.cInputValidType = LOWLEVEL_VALID;
		}
		//只存在K
		else
		{
			pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = (int8)fabs(pScan->dRIJK[3]);
			pGCodeBufUnit->SIOContrlInfo.cInputValidType = HIGHLEVEL_VALID;
		}
	    // **门禁：输入IO索引范围合法**
	    if (pGCodeBufUnit->SIOContrlInfo.cInputIOIndex < 0 ||
	        pGCodeBufUnit->SIOContrlInfo.cInputIOIndex >= 38)
	    {
	        // 非法 L/K：拒绝入队
	    	parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
	        return -1;
	    }

		//2.参数J存在，赋值IO的输入等待检测时间
		if (pScan->bRIJK[2])
		{
			int32 nDetectTimeMs;
			/* J按秒校验后换算，避免小数先截断成无限等待。 */
			if (ConvertSecondsToMsChecked(pScan->dRIJK[2], &nDetectTimeMs, OUTPUT_MAX_MS) < 0)
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
				return -1;
			}
			/* 正数至少等待1ms；只有J0或省略J保留无限等待。 */
			if (pScan->dRIJK[2] > 0.0 && nDetectTimeMs == 0)
			{
				nDetectTimeMs = 1;
			}
			pGCodeBufUnit->SIOContrlInfo.nInputDetectTime = nDetectTimeMs;
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.nInputDetectTime = UNASSIGNED_VALUE;
		}

		//3.参数I存在，赋值外部输出IO以及外部输出IO控制的电平类型:高电平
		if (pScan->bRIJK[1])
		{
			pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)fabs(pScan->dRIJK[1]);//单位转换 s->ms
			pGCodeBufUnit->SIOContrlInfo.cOutputControlType = HIGHLEVEL;
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;//单位转换 s->ms
			pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
		}


		//4.参数R，赋值上料退回延长时间
		if (pScan->bRIJK[0])
		{
			pGCodeBufUnit->SIOContrlInfo.nFeedingBackTime = (int8)fabs(pScan->dRIJK[0]);//单位转换 s->ms
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.nFeedingBackTime = UNASSIGNED_VALUE;
		}

		//5.参数P，赋值上料失败后重复执行次数
		if (pScan->bP)
		{
			pGCodeBufUnit->SIOContrlInfo.ucFeeding_RepeatTimes = (int8)fabs(pScan->dP);
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.ucFeeding_RepeatTimes = UNASSIGNED_VALUE;
		}
	}
	//在没输入L / K时不清除M35, 而是把IO输入端口索引值赋 - 1，IO输入IO电平有效类型赋值为低电平有效
	else
	{
		pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = UNASSIGNED_VALUE;
		pGCodeBufUnit->SIOContrlInfo.cInputValidType = LOWLEVEL_VALID;
	}

	setIO(M35, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//************************************M36******************************************
//添加M36规则
void addM36RegulationAPI()
{
	addMCommadRegulation(36, IO_PRE_ACTION, cMj, 1, NULL, NULL, NULL, NULL, AssignM36);
}

//赋值M36
int8 AssignM36(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{

	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	//获取赋值结构体中的扫描结构体
	SScan* pScan = &pSAssignConfig->SScanContainer;
    // =========================
    // **门禁1：I 必须存在**
    // =========================
    if (pScan->bRIJK[1] == FALSE)
    {
        // 缺 I：拒绝入队
        return -1;
    }

    double raw = fabs(pScan->dRIJK[1]);

    // 1) 必须为整数（避免 1.2 这种值）
    if (raw != floor(raw)) {
        return -1;
    }

    // 2) 必须在合法范围
    if (raw < 0.0 || raw > 37.0) {
        return -1;
    }

    // I -> 输出IO索引（**禁止做 *1000**）
	  pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)raw;

	  // **门禁2：I 范围合法**
	  if (pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex < 0 ||
		  pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex >= 38)
	  {
		  // 非法 I：拒绝入队
		  return -1;
	  }



	//存在参数L或K
	// L，K：IO输入端口号
	// L的IO电平有效类型为低电平有效
	//K的IO电平有效类型为高电平有效
	if (pScan->bL || pScan->bRIJK[3])
	{
		//1.参数L和K同时存在，IO输入端口以及IO输入IO电平有效类型的赋值以K为准，忽略L
		if (pScan->bL && pScan->bRIJK[3])
		{
			pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = (int8)fabs(pScan->dRIJK[3]);
			pGCodeBufUnit->SIOContrlInfo.cInputValidType = HIGHLEVEL_VALID;
		}
		//只存在L
		else if (pScan->bL)
		{
			pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = (int8)fabs(pScan->dL);
			pGCodeBufUnit->SIOContrlInfo.cInputValidType = LOWLEVEL_VALID;
		}
		//只存在K
		else
		{
			pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = (int8)fabs(pScan->dRIJK[3]);
			pGCodeBufUnit->SIOContrlInfo.cInputValidType = HIGHLEVEL_VALID;
		}

		//2.参数J存在，赋值IO的输入等待检测时间
		if (pScan->bRIJK[2])
		{
			int32 nDetectTimeMs;
			/* J按秒校验后换算，避免小数先截断成无限等待。 */
			if (ConvertSecondsToMsChecked(pScan->dRIJK[2], &nDetectTimeMs, OUTPUT_MAX_MS) < 0)
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM301);
				return -1;
			}
			/* 正数至少等待1ms；只有J0或省略J保留无限等待。 */
			if (pScan->dRIJK[2] > 0.0 && nDetectTimeMs == 0)
			{
				nDetectTimeMs = 1;
			}
			pGCodeBufUnit->SIOContrlInfo.nInputDetectTime = nDetectTimeMs;
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.nInputDetectTime = UNASSIGNED_VALUE;
		}

		//3.参数I存在，赋值外部输出IO以及外部输出IO控制的电平类型:低电平
		if (pScan->bRIJK[1])
		{
			pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = (int8)fabs(pScan->dRIJK[1]);
			pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.cOutputIOIndex = UNASSIGNED_VALUE;//单位转换 s->ms
			pGCodeBufUnit->SIOContrlInfo.cOutputControlType = LOWLEVEL;
		}


		//4.参数R，赋值上料退回延长时间
		if (pScan->bRIJK[0])
		{
			pGCodeBufUnit->SIOContrlInfo.nFeedingBackTime = (int8)fabs(pScan->dRIJK[0]);//单位转换 s->ms
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.nFeedingBackTime = UNASSIGNED_VALUE;
		}

		//5.参数P，赋值上料失败后重复执行次数
		if (pScan->bP)
		{
			pGCodeBufUnit->SIOContrlInfo.ucFeeding_RepeatTimes = (int8)fabs(pScan->dP);
		}
		else
		{
			pGCodeBufUnit->SIOContrlInfo.ucFeeding_RepeatTimes = UNASSIGNED_VALUE;
		}
	}
	//在没输入L / K时不清除M35, 而是把IO输入端口索引值赋 - 1，IO输入IO电平有效类型赋值为低电平有效
	else
	{
		pGCodeBufUnit->SIOContrlInfo.cInputIOIndex = UNASSIGNED_VALUE;
		pGCodeBufUnit->SIOContrlInfo.cInputValidType = LOWLEVEL_VALID;
	}

	setIO(M36, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

// 根据你工程的需要改返回类型/最大值
// maxMs 建议按字段类型/业务定义
static int8 ConvertSecondsToMsChecked(double sec, int32* outMs, int32 maxMs)
{
    if (outMs == NULL) return -1;
    if (!isfinite(sec)) return -1;              // 防 NaN/Inf
    if (sec < 0.0) return -1;                   // 不允许负数（如允许可调整）

    double msD = fabs(sec) * 1000.0;

    // 可选：要求 J 必须是有限小数并允许小数秒，ms 向下取整 or 四舍五入
    if (msD > (double)maxMs) return -1;

    *outMs = (int32)(msD + 0.5);                // 四舍五入
    return 1;
}
