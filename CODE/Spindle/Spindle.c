#include "Spindle.h"
#include <stdlib.h>
#include "G1.h" 
#include "G0.h"  
#include "Motion.h"	
#include "GeneralFunctions.h"
#include "CTrajPlanning.h"
#include "G33.h"
#include "Channel.h"
#include "Config.h"
#include "IOCommand.h"
#include "SecondSpindle.h" // zxr: 沿用无编码器动力刀配置，不把模拟反馈作为机械到位依据
#if STM32_MODE
#include "sys.h"
/* 苏向标 2026-10-14：使能模块使用板端非阻塞计时接口。 */
#include "time.h"
#endif // STM32_MODE


//2022.3.8  修改G96功能状态下，absX为零时，产生除零的问题


//struct _SSpindle g_SSpindle;//主轴结构体

//struct _SSpindleTest g_SSpindleTest;//主轴转速检测结构体




// zxr: 1ms周期下连续稳定100ms；5秒是反馈故障上限，不是完成延时
#define PRECISE_STOP_STILL_TICKS 100U
#define PRECISE_STOP_TIMEOUT_TICKS 5000U

/*************************************************
  zxr: 初始化、新准停及停止接管时清理独立计数
  函数名称：clearPreciseStopFeedback
  函数功能：清除本主轴本轮准停反馈观察
  输入参数：pSSpindle - 主轴对象
  输出参数：上次反馈及等待、稳定计数
  返 回 值：无
  备    注：
**************************************************/
static void clearPreciseStopFeedback(struct _SSpindle* pSSpindle)
{
    pSSpindle->lPreciseStopLastEncoder = 0;
    pSSpindle->usPreciseStopWaitTicks = 0;
    pSSpindle->usPreciseStopStillTicks = 0;
}

/* 苏向标 2026-10-14：初始化主轴请求及独立使能计时。 */
/*************************************************
  函数名称：spindleInit
  函数功能：初始化主轴对象、停止请求及独立使能计时
  输入参数：pSSpindle - 主轴对象；pSSingleAxisPointer - 对应单轴；ucFollowAxis - G96跟随逻辑轴
  输出参数：更新主轴初始化字段
  返 回 值：1 - 初始化完成；-1 - 主轴对象为空
  备    注：苏向标 2026-10-14；不改变通道和轴映射配置
****************************************/
int8 spindleInit(struct _SSpindle* pSSpindle, struct SSingleAxis* pSSingleAxisPointer, uint8 ucFollowAxis)
{
	if (pSSpindle == NULL || pSSingleAxisPointer == NULL)
	{
		return -1;
	}
    /* 苏向标 2026-10-14：初始化主副轴各自的请求、使能计时和使能准备占用。 */
    pSSpindle->ucEnableOccupation = 0;
    pSSpindle->unEnableRunTime = 0;
    pSSpindle->unDisableRunTime = 0;
    pSSpindle->unEnableRequestVersion = 0;
    pSSpindle->bStopRequested = FALSE;
    pSSpindle->bStopNowRequested = FALSE;
    pSSpindle->cStopError = 0;
    clearPreciseStopFeedback(pSSpindle); // zxr: 新对象不继承上次准停观察
	//zxr: 新主轴对象从零脉冲余量开始
	pSSpindle->dSpeedPulseRemainder = 0.0;
	pSSpindle->pSSpindleAxis = pSSingleAxisPointer;//主轴的单轴结构体
	pSSpindle->spindleFollowAxis = ucFollowAxis;//G96跟随的轴
		//主轴初始化
	pSSpindle->pSSpindleAxis->SPlanData.dVe = 0.0 / (60 * 1000);
	pSSpindle->pSSpindleAxis->SPlanData.dVs = 0.0 / (60 * 1000);
	pSSpindle->pSSpindleAxis->SPlanData.unT1 = 0;
	pSSpindle->pSSpindleAxis->SPlanData.unT2 = 0;
	pSSpindle->pSSpindleAxis->SPlanData.unT3 = 0;
	pSSpindle->pSSpindleAxis->SPlanData.unT4 = 0;
	pSSpindle->pSSpindleAxis->SPlanData.unT5 = 0;
	pSSpindle->pSSpindleAxis->dSpindleSpeed = 0.0 / (60 * 1000);
	pSSpindle->pSSpindleAxis->SPlanData.unCurrentT = 1;

	pSSpindle->pSSpindleAxis->SPlanData.dTc = g_SSystemPara.dInterpolationTime;//插补周期
	pSSpindle->pSSpindleAxis->ucRunMode = SPINDLE_SINGLEAXIS_MODE;
	pSSpindle->pSSpindleAxis->cSpindleDir = ARC_CW;
	pSSpindle->pSSpindleAxis->dLimitedSpindleSpeed = G50_INITIAL_SPEED;// 开机后 G50限制的速度
	pSSpindle->pSSpindleAxis->dFeedRatio = 1.0;
	pSSpindle->pSSpindleAxis->dSpindleSpeedForChangeAcc = g_SSystemPara.Speed[pSSpindle->pSChannel->ucChannelNum];
	pSSpindle->pSSpindleAxis->ucGearForChangeRate = g_SSystemPara.acceleration[pSSpindle->pSChannel->ucChannelNum];
	pSSpindle->pSSpindleAxis->bIsChangeAcc = FALSE;

	return 1;
}

/*************************************************
  函数名称：isUnconfiguredSpindleIdle
  函数功能：判断主轴是否未配置且无需执行停止或资源收尾
  输入参数：pSSpindle - 主轴或副轴结构
  输出参数：无
  返 回 值：TRUE - 空映射且空闲无资源；FALSE - 已配置或状态异常
  备    注：只读通道映射和本轴状态，不将运行中映射丢失或停止故障视为完成
****************************************/
static bool isUnconfiguredSpindleIdle(const struct _SSpindle* pSSpindle)
{
    /* 空对象、无映射表或逻辑轴越界不代表机型未配置主轴。 */
    if (pSSpindle == NULL || pSSpindle->pSSpindleAxis == NULL || pSSpindle->pSChannel == NULL
        || pSSpindle->pSChannel->pMapAxisArray == NULL || pSSpindle->pSSpindleAxis->ucAxis >= AXIS_NUM_MAX)
    {
        return FALSE;
    }
    return pSSpindle->pSChannel->pMapAxisArray[pSSpindle->pSSpindleAxis->ucAxis] == AXIS_NULL
        && pSSpindle->ucOccupation == 0 && pSSpindle->ucEnableOccupation == 0
        && pSSpindle->cStopError == 0 && pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE
        && isZero(pSSpindle->pSSpindleAxis->dCurrentSpeed)
        && pSSpindle->pSChannel->SRun.sAxisDeltaPulse[pSSpindle->pSSpindleAxis->ucAxis] == 0;
}

/* 苏向标 2026-10-14：独立推进主轴停止，避免停止依赖启动使能完成。 */
/*************************************************
  函数名称：isSpindleStopping
  函数功能：由请求、原运行模式及故障判断停止是否未完成
  输入参数：pSSpindle - 主轴或副轴结构
  输出参数：无
  返 回 值：TRUE - 停止未完成；FALSE - 无停止任务
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
bool isSpindleStopping(const struct _SSpindle* pSSpindle)
{
    return pSSpindle != NULL && (pSSpindle->bStopRequested || pSSpindle->bStopNowRequested
        || pSSpindle->cStopError < 0 || (pSSpindle->pSSpindleAxis != NULL
        && pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_COMMONSTOP_MODE));
}
/*************************************************
  函数名称：isSpindleStopFinished
  函数功能：复位等待完整收尾，不能只检查插补完成
  输入参数：pSSpindle - 主轴或动力刀控制结构
  输出参数：无
  返 回 值：TRUE - 完成；FALSE - 未完成
  备    注：苏向标 2026-10-14
****************************************/

bool isSpindleStopFinished(const struct _SSpindle* pSSpindle)
{
    if (pSSpindle == NULL || pSSpindle->pSSpindleAxis == NULL)
    {
        return TRUE;
    }
    // zxr: 命令插补结束后仍须等待准停反馈确认及原收尾
    if (isSpindleStopping(pSSpindle)
        || pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE)
    {
        return FALSE;
    }
    return pSSpindle->pSSpindleAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE
        || pSSpindle->pSSpindleAxis->ucInpStatus == INP_FINISHED;
}
/*************************************************
  函数名称：getSpindleEnableOutput
  函数功能：读取有效物理轴的使能输出
  输入参数：ucPhysicalAxis - 已校验的物理轴号
  输出参数：无
  返 回 值：当前输出值
  备    注：苏向标 2026-10-14
****************************************/

static uint8 getSpindleEnableOutput(uint8 ucPhysicalAxis)
{
    /* pAxisEnable是共享物理输出映射；这里只读取指定轴，不获取控制权。 */
#if CotrexF4 == 1
    return *pAxisEnable[ucPhysicalAxis];
#else
    return pAxisEnable(ucPhysicalAxis);
#endif
}
/*************************************************
  函数名称：setSpindleEnableOutput
  函数功能：设置有效物理轴的使能输出
  输入参数：ucPhysicalAxis - 已校验的物理轴号；ucOutput - 输出值
  输出参数：更新物理输出
  返 回 值：无
  备    注：苏向标 2026-10-14
****************************************/

static void setSpindleEnableOutput(uint8 ucPhysicalAxis, uint8 ucOutput)
{
    /* pAxisEnable是共享物理输出映射；仅周期持有本轴控制权时允许写输出。 */
#if CotrexF4 == 1
    *pAxisEnable[ucPhysicalAxis] = ucOutput;
#else
    pAxisEnableSet(ucPhysicalAxis, ucOutput);
#endif
}
/*************************************************
  函数名称：failSpindleStop
  函数功能：锁存控制权或停止异常，不把异常当成停止完成
  输入参数：pSSpindle - 主轴结构；cError - 错误原因
  输出参数：更新当前主轴控制数据
  返 回 值：负值 - 锁存的故障原因
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
static int8 failSpindleStop(struct _SSpindle* pSSpindle, int8 cError)
{
    if (pSSpindle->cStopError == 0)
    {
        if (pSSpindle->pSSpindleAxis != NULL
            && pSSpindle->pSSpindleAxis->ucAxis < AXIS_NUM_MAX
            && (getbit(pSSpindle->ucOccupation, pSSpindle->pSSpindleAxis->ucAxis)))
        {
            pSSpindle->pSChannel->SRun.sAxisDeltaPulse[pSSpindle->pSSpindleAxis->ucAxis] = 0;
        }
        pSSpindle->cStopError = cError;
        parseRunningError(pSSpindle->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM155, 9999);
    }
    return pSSpindle->cStopError;
}
/*************************************************
  函数名称：prepareSpindleStop
  函数功能：停止专用占用判断：取消等待、取消准备、进入原减速
  输入参数：pSSpindle - 已验证映射的主轴结构
  输出参数：更新当前主轴控制数据
  返 回 值：1 - 取消等待完成；0 - 进入减速或收尾；负值 - 故障
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
static int8 prepareSpindleStop(struct _SSpindle* pSSpindle)
{
    /* g_SSystemPara提供本通道减速倍率参数；此处只读，不改变设备配置。 */
    struct SSingleAxis* pSAxis = pSSpindle->pSSpindleAxis; /**< 原执行阶段。 */
    VAL_STRUCT_TYPE ucSign = (VAL_STRUCT_TYPE)(1U << pSAxis->ucAxis); /**< 当前逻辑轴。 */
    bool bWaiting = pSSpindle->ucOccupation == 0
        && (pSAxis->ucInpStatus == INP_OCCUPATION || pSAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE)
        && isZero(pSAxis->dCurrentSpeed)
        && pSSpindle->pSChannel->SRun.sAxisDeltaPulse[pSAxis->ucAxis] == 0; /**< 只在本次分类使用。 */
    int8 cResult; /**< 不触发使能的占用结果。 */
    pSAxis->ucRunMode = SPINDLE_COMMONSTOP_MODE;
    pSSpindle->ucOccupation |= pSSpindle->ucEnableOccupation;
    pSSpindle->ucEnableOccupation = 0;
    if ((pSSpindle->ucOccupation & ucSign) == 0)
    {
        if (!bWaiting)
        {
            return failSpindleStop(pSSpindle, -2);
        }
        cResult = reserveAxisControl(ucSign, pSSpindle->pSChannel->pMapAxisArray, &pSSpindle->ucOccupation);
        if (cResult == 0)
        {
            /* 尚未接管且轴属于别人：只取消自己的等待，不改别人的输出。 */
            pSAxis->ucRunMode = SPINDLE_SINGLEAXIS_MODE;
            pSAxis->ucInpStatus = INP_FINISHED;
            return 1;
        }
        if (cResult < 0)
        {
            return failSpindleStop(pSSpindle, cResult);
        }
    }
    /* 准备中直接收尾；已进入运动则复用原减速初始化。 */
    pSAxis->ucInpStatus = bWaiting ? INP_FINISHED : INP_INITIATION;
    pSAxis->bIsChangeAcc = FALSE;
    pSAxis->dSpindleSpeedForChangeAcc = g_SSystemPara.Speed[pSSpindle->pSChannel->ucChannelNum];
    pSAxis->ucGearForChangeRate = g_SSystemPara.acceleration[pSSpindle->pSChannel->ucChannelNum];
    return 0;
}
/*************************************************
  函数名称：processSpindleRequests
  函数功能：仅由运动周期接收请求，不在低优先级入口修改多个运动字段
  输入参数：pSSpindle - 主轴或副轴结构
  输出参数：更新当前主轴控制数据
  返 回 值：无
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
void processSpindleRequests(struct _SSpindle* pSSpindle)
{
    struct SSingleAxis* pSAxis; /**< 原运行状态。 */
    bool bImmediate; /**< 本周期是否需要立即停止。 */
    if (pSSpindle == NULL || pSSpindle->pSSpindleAxis == NULL || pSSpindle->pSChannel == NULL)
    {
        return;
    }
    //zxr: 周期入口统一清理停机、故障及非速度模式余量，连续变速和减速期间保留
    pSAxis = pSSpindle->pSSpindleAxis;
    if (pSSpindle->bStopNowRequested || pSSpindle->cStopError < 0 ||
        (pSAxis->ucRunMode == SPINDLE_COMMONSTOP_MODE && pSAxis->ucInpStatus == INP_FINISHED) ||
        (pSAxis->ucRunMode != SPINDLE_G97_MODE && pSAxis->ucRunMode != SPINDLE_G96_MODE &&
         pSAxis->ucRunMode != SPINDLE_G96ANDG0_MODE && pSAxis->ucRunMode != SPINDLE_G96ANDG1_MODE &&
         pSAxis->ucRunMode != SPINDLE_COMMONSTOP_MODE))
    {
        pSSpindle->dSpeedPulseRemainder = 0.0;
    }
    // zxr: 普通停、立即停、故障或退出准停时取消本轮反馈观察
    if (pSAxis->ucRunMode != SPINDLE_PRECISESTOP_MODE
        || pSSpindle->bStopRequested || pSSpindle->bStopNowRequested || pSSpindle->cStopError < 0)
    {
        clearPreciseStopFeedback(pSSpindle);
    }
    if (!pSSpindle->bStopRequested && !pSSpindle->bStopNowRequested)
    {
        return;
    }
    bImmediate = pSSpindle->bStopNowRequested;
    pSSpindle->bStopRequested = FALSE;
    pSSpindle->bStopNowRequested = FALSE;
    pSAxis = pSSpindle->pSSpindleAxis;
    /* 未配置且空闲的主副轴只消费停止请求，不占轴、不操作使能，也不锁存映射故障。 */
    if (isUnconfiguredSpindleIdle(pSSpindle))
    {
        return;
    }
    if (pSSpindle->cStopError < 0 || pSAxis->ucRunMode == SPINDLE_HANDWHEEL_MODE)
    {
        return;
    }
    if (pSAxis->ucAxis >= AXIS_NUM_MAX || pSSpindle->pSChannel->pMapAxisArray == NULL
        || pSSpindle->pSChannel->pMapAxisArray[pSAxis->ucAxis] >= AXIS_PHYSICS_NUM_MAX)
    {
        failSpindleStop(pSSpindle, -3);
        return;
    }
    if (pSAxis->ucRunMode != SPINDLE_COMMONSTOP_MODE)
    {
        ++pSSpindle->unEnableRequestVersion;
        pSSpindle->unEnableRunTime = 0;
        if (pSAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE && pSSpindle->ucEnableOccupation == 0)
        {
            if (bImmediate)
            {
                pSAxis->ucInpStatus = INP_STOP;
            }
            return;
        }
        if (prepareSpindleStop(pSSpindle) != 0)
        {
            return;
        }
    }
    if (bImmediate)
    {
        pSAxis->dCurrentSpeed = 0.0;
        pSSpindle->pSChannel->SRun.sAxisDeltaPulse[pSAxis->ucAxis] = 0;
        pSAxis->ucInpStatus = INP_FINISHED;
    }
}

/*************************************************
  函数名称：processSpindleStop
  函数功能：普通停止不走自动使能入口，只推进原插补状态
  输入参数：pSSpindle - 主轴或副轴结构
  输出参数：更新当前主轴控制数据
  返 回 值：1 - 停止完成；0 - 等待；负值 - 故障
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
int8 processSpindleStop(struct _SSpindle* pSSpindle)
{
    /* g_ucAxisOccupation与g_ucAxisExactEnable是共享物理轴位图；这里只校验，不把全局占用当成自己的占用。 */
    struct SSingleAxis* pSAxis; /**< 当前轴状态。 */
    uint8 ucPhysicalAxis; /**< 已校验的物理轴号。 */
    int8 cResult; /**< 原减速与收尾的处理结果。 */
    if (pSSpindle == NULL || pSSpindle->pSSpindleAxis == NULL || pSSpindle->pSChannel == NULL)
    {
        return -1;
    }
    processSpindleRequests(pSSpindle);
    if (pSSpindle->cStopError < 0)
    {
        return pSSpindle->cStopError;
    }
    pSAxis = pSSpindle->pSSpindleAxis;
    if (pSAxis->ucRunMode != SPINDLE_COMMONSTOP_MODE)
    {
        return 1;
    }
    if (pSAxis->ucAxis >= AXIS_NUM_MAX || pSSpindle->pSChannel->pMapAxisArray == NULL)
    {
        return failSpindleStop(pSSpindle, -3);
    }
    ucPhysicalAxis = pSSpindle->pSChannel->pMapAxisArray[pSAxis->ucAxis];
    if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX || !isfinite(pSAxis->dCurrentSpeed)
        || !(getbit(pSSpindle->ucOccupation, pSAxis->ucAxis))
        || !(getbit(g_ucAxisOccupation, ucPhysicalAxis)))
    {
        return failSpindleStop(pSSpindle, -4);
    }
    if (!isZero(pSAxis->dCurrentSpeed) && (!(getbit(g_ucAxisExactEnable, ucPhysicalAxis))
        || getSpindleEnableOutput(ucPhysicalAxis) != IO_SET))
    {
        return failSpindleStop(pSSpindle, -5);
    }
    if (pSAxis->ucInpStatus == INP_OCCUPATION)
    {
        pSAxis->ucInpStatus = INP_INITIATION;
    }
    cResult = commonStopMotion(pSSpindle);
    if (cResult < 0)
    {
        return failSpindleStop(pSSpindle, cResult);
    }
    return pSAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE ? 1 : 0;
}

/* 苏向标 2026-10-14：周期先接收停止，停止分支不重新触发启动使能。 */
/*************************************************
  函数名称：spindleMotion
  函数功能：按原模式推进主轴速度及坐标，优先处理停止请求
  输入参数：pSSpindlePointer - 有效通道所属主轴对象
  输出参数：更新运动状态、周期脉冲与坐标
  返 回 值：1 - 本周期处理结束；0 - 等待占用；负值 - 参数或停止异常
  备    注：苏向标 2026-10-14；仅运动周期调用；不是机械停稳确认
****************************************/
int8 spindleMotion(struct _SSpindle* pSSpindlePointer)
{
	double dCurrentPoint;//当前坐标
	struct SInterpolationPre* pSPreprocess;//预处理指针
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射
	uint8 ucAxis;

	if (pSSpindlePointer == NULL || pSSpindlePointer->pSSpindleAxis == NULL || pSSpindlePointer->pSChannel == NULL)
	{
		return -1;
	}
    /* 苏向标 2026-10-14：停止请求只在本周期接收。 */
    processSpindleRequests(pSSpindlePointer);
    /* 无主轴通道不执行主轴运动；有占用、运动或故障时仍走原异常和收尾路径。 */
    if (isUnconfiguredSpindleIdle(pSSpindlePointer))
    {
        return 1;
    }
    if (pSSpindlePointer->cStopError < 0)
    {
        return pSSpindlePointer->cStopError;
    }
	//单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;
	//预处理结构体
	pSPreprocess = &pSSpindlePointer->pSChannel->SPreProcess;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		parseRunningError(9999,GCODE_ERROR_NUM150,9999); 
		return -3;
	}
// 	//2025.9.22 熊骏
// 	if (
// #if STM32_MODE
// 		Check_bootwarning() == 0 || //正在报警则退出函数，企业管理的报警主轴报警、伺服报警、超时报警、限位报警
// #endif
// 		isErrorHasBeenWritten() == TRUE)//正在报警则退出函数，学校管理的报警  2024/10/16
// 	{
// 		return;
// 	}

	if (pSSpindlePointer->dLimitedSpindleSpeed >= 0)//G50限制速度有效  2023.3.13
	{
		double dLimitedSpindleSpeed = pSSpindlePointer->dLimitedSpindleSpeed / (60.0 * 1000) * DEGREE_OF_CIRCLE;
		if (!isZero(pSSingleAxisPointer->dLimitedSpindleSpeed - dLimitedSpindleSpeed))
		{
			pSSingleAxisPointer->dLimitedSpindleSpeed = dLimitedSpindleSpeed;
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG0_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG1_MODE)
			{//处于G96G97的功能状态下
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		pSSpindlePointer->dLimitedSpindleSpeed = SPINDLE_INVALID_VAL;//置为无效
	}


	if (getBits(pSSingleAxisPointer->unCommandType, CMD_SPINDLE_SPEED_ADDR) != getBits(pSPreprocess->unCommandType, CMD_SPINDLE_SPEED_ADDR))
	{//G96  G97模式转换

		pSSingleAxisPointer->unCommandType = pSPreprocess->unCommandType;//指令类型继承
		if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG0_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG1_MODE)
		{//处于G96G97的功能状态下
			switch (getBits(pSSingleAxisPointer->unCommandType, CMD_SPINDLE_SPEED_ADDR))
			{
			case CMD_SPINDLE_SPEED_CV://恒转速
				if (pSSpindlePointer->dSpindleSpeed < 0)//无主轴转速   根据当前转速进行转换
				{
					pSSingleAxisPointer->dSpindleSpeed = pSSingleAxisPointer->dCurrentSpeed;//继承当前转速(r/min)
				}
				pSSingleAxisPointer->ucRunMode = SPINDLE_G97_MODE;//设置G97 功能状态
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态

				break;

			case CMD_SPINDLE_SPEED_CLV://恒线速
				//不改变线速度  继承之前G96的S值  如果有新转速值 则在下面流程读取
				pSSingleAxisPointer->ucRunMode = SPINDLE_G96ANDG1_MODE;//设置G96 功能状态
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
				break;
			}
		}
	}

	if (pSSpindlePointer->dSpindleSpeed >= 0)// 主轴转速有效2023.3.13
	{
		double dSpindleSpeed;
		dSpindleSpeed = pSSpindlePointer->dSpindleSpeed / (60.0 * 1000) * DEGREE_OF_CIRCLE;//主轴转速 r/min->度/ms
		if (!isZero(dSpindleSpeed - pSSingleAxisPointer->dSpindleSpeed))
		{
			pSSingleAxisPointer->dSpindleSpeed = dSpindleSpeed;
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE)
			{//处于G97的功能状态下
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		pSSpindlePointer->dSpindleSpeed = SPINDLE_INVALID_VAL;//已读取  置为无效

	}
	if (pSSpindlePointer->dSpindleLineSpeed >= 0)// 主轴线速度有效2023.3.13
	{
		double dSpindleLineSpeed;
		dSpindleLineSpeed = pSSpindlePointer->dSpindleLineSpeed / (60.0 * 1000);//赋值  主轴转速 m/ms
		if (!isZero(dSpindleLineSpeed - pSSingleAxisPointer->dFeedrate))
		{
			pSSingleAxisPointer->dFeedrate = dSpindleLineSpeed;
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG0_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG1_MODE)
			{//处于G96的功能状态下
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		pSSpindlePointer->dSpindleLineSpeed = SPINDLE_INVALID_VAL;
	}



	if (pSSingleAxisPointer->dFeedRatio != pSRun->dSpindleSpeedRatio && pSSingleAxisPointer->ucRunMode != SPINDLE_SINGLEAXIS_MODE)//2026.6.2避免c轴分度运动手轮模拟时倍率被修改
	{//主轴倍率改变
		pSSingleAxisPointer->dFeedRatio = pSRun->dSpindleSpeedRatio;//赋值  进给倍率

		if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE)//G96不受倍率影响
		{
			pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		}
	}
	//主轴转速在检测转速之上时改变加速度，在检测转速之下时保持/恢复原有加速度。 2024/3/23 洪冠鹏
	/* 苏向标 2026-10-14：收尾和故障阶段不再重启速度规划。 */
    if (pSSpindlePointer->cStopError == 0
        // zxr: 命令结束后的反馈等待也不重启变加速度规划
        && !((isSpindleStopping(pSSpindlePointer) || pSSingleAxisPointer->ucRunMode == SPINDLE_PRECISESTOP_MODE)
            && pSSingleAxisPointer->ucInpStatus == INP_FINISHED)
        && pSSingleAxisPointer->dSpindleSpeedForChangeAcc > 0 && pSSingleAxisPointer->ucGearForChangeRate != 0)//功能参数页 主轴检测转速和改变加速度百分比设定值都不为0时该功能生效
	{
		//主轴转速达到检测转速后标志位置TRUE，在初始化阶段依据标志位，读取根据百分比改变后的加速度，重新速度规划
		if (fabs(pSSingleAxisPointer->dCurrentSpeed * (60.0 * 1000) / DEGREE_OF_CIRCLE) > fabs(pSSingleAxisPointer->dSpindleSpeedForChangeAcc) && pSSingleAxisPointer->bIsChangeAcc == FALSE)
		{
			pSSingleAxisPointer->bIsChangeAcc = TRUE;//改变加速度标志位置TRUE，保证只改变一次加速度
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_COMMONSTOP_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_PRECISESTOP_MODE)//G96不受影响
			{
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		//主轴转速低于检测转速后标志位置FALSE，在初始化阶段依据标志位，读取原有的加速度，重新速度规划
		else if (fabs(pSSingleAxisPointer->dCurrentSpeed * (60.0 * 1000) / DEGREE_OF_CIRCLE) < fabs(pSSingleAxisPointer->dSpindleSpeedForChangeAcc) && pSSingleAxisPointer->bIsChangeAcc == TRUE)
		{
			pSSingleAxisPointer->bIsChangeAcc = FALSE;//改变加速度标志位置FALSE，保证只改变一次加速度
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_COMMONSTOP_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_PRECISESTOP_MODE)//G96不受影响
			{
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
				if (pSSingleAxisPointer->ucRunMode != SPINDLE_G97_MODE)pSSingleAxisPointer->dSpindleSpeedForChangeAcc = SPINDLE_INVALID_VAL;//将检测转速置为无效，保证减速/准停过程中能停下来。
			}
		}
	}
    /* 苏向标 2026-10-14：停止先行，保留后面的坐标回写。 */
    if (isSpindleStopping(pSSpindlePointer))
    {
        if (processSpindleStop(pSSpindlePointer) < 0)
        {
            return -1;
        }
    }
    else
    {
        /* 轴占用判断 */
        if (pSSpindlePointer->pSSpindleAxis->ucInpStatus == INP_OCCUPATION)
        {
#if(TURRET_AXIS_TEST == 1)
            {
                if (getbit(pSSpindlePointer->pSChannel->SPreProcess.ucOccupideValStatus, pSSingleAxisPointer->ucAxis) == TRUE && pSSpindlePointer->pSChannel->SPreProcess.ucInpStatus == INP_FINISHED)//上一指令为分度指令//释放上一指令的轴占用 2024/7/12 洪冠鹏
                {
                    axisManagerReleaseByChannel(1 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->ucChannelNum, &pSSpindlePointer->pSChannel->SPreProcess.ucOccupideValStatus);

                }
                if (axisManagerOccupyByChannel(1 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->ucChannelNum, &pSSpindlePointer->ucOccupation) == 1)
                {
                    /* 占用成功 */
                    if (setSpindleEnabled(pSSpindlePointer, TRUE) == 1)//设置主轴使能
                    {
                        pSSpindlePointer->pSSpindleAxis->ucInpStatus = INP_INITIATION;//使能成功  进入初始化状态
                    }
                }
                else
                {
                    /* 占用失败 */
                    return 0;
                }
            }
#else

            {
                if (getbit(pSSpindlePointer->pSChannel->SPreProcess.ucLastValStatus, pSSingleAxisPointer->ucAxis) == TRUE && pSSpindlePointer->pSChannel->SPreProcess.ucInpStatus == INP_FINISHED)//上一指令为分度指令//释放上一指令的轴占用 2024/7/12 洪冠鹏
                {
                    axisManagerReleaseByChannel(1 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->ucChannelNum, &pSSpindlePointer->ucOccupation);
                    clrbit(pSSpindlePointer->pSChannel->SPreProcess.ucLastValStatus, pSSingleAxisPointer->ucAxis);//上一指令的C轴占用置为空  防止重复释放
                }
                if (axisManagerOccupyByChannel(1 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->ucChannelNum, &pSSpindlePointer->ucOccupation) == 1)
                {
                    /* 占用成功 */
                    if (setSpindleEnabled(pSSpindlePointer, TRUE) == 1)//设置主轴使能
                    {
                        pSSpindlePointer->pSSpindleAxis->ucInpStatus = INP_INITIATION;//使能成功  进入初始化状态
                    }
                }
                else
                {
                    /* 占用失败 */
                    return 0;
                }
            }

#endif

        }


        switch (pSSingleAxisPointer->ucRunMode)//根据Mode 执行不同的功能
        {

        case SPINDLE_G97_MODE: //G97恒转速
            constVelMotion(pSSpindlePointer);
            break;
        case SPINDLE_G96_MODE:
        case SPINDLE_G96ANDG0_MODE://G96恒线速度
        case SPINDLE_G96ANDG1_MODE:
            constLineVelMotion(pSSpindlePointer);
            break;

        case SPINDLE_COMMONSTOP_MODE://减速停
            commonStopMotion(pSSpindlePointer);
            break;

        case SPINDLE_PRECISESTOP_MODE://准停
            preciseStopMotion(pSSpindlePointer);
            break;
        }

    }

	//根据脉冲反算得到C轴实际坐标
	getSingleAxisRealTimePoint(&dCurrentPoint, pSSingleAxisPointer->ucAxis, ucMapArray);

	if (pSSpindlePointer->pSSpindleAxis->ucRunMode != SPINDLE_SINGLEAXIS_MODE)
	{
		dCurrentPoint += pSRun->sAxisDeltaPulse[ucAxis] * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm;//脉冲数 × 每脉冲对应的角度
		dCurrentPoint = surplusDoubleData(dCurrentPoint + DEGREE_OF_CIRCLE, (int)DEGREE_OF_CIRCLE);//取余
		//更新坐标
		updateSingleAxisRealTimePoint(&dCurrentPoint, ucAxis, ucMapArray);
	}

	return 1;
}




/*************************************************
zxr: 速度输出共用小数脉冲累计，位置准停仍使用原目标差算法
  函数名称：calcSpindleSpeedPulse
  函数功能：将本周期角位移与余量合并量化，保留正常舍入误差
  输入参数：pSSpindle - 主轴对象；dDist - 本周期角位移；dPulseToMm - 每脉冲角位移
  输出参数：pPulse - 本周期脉冲数；更新本主轴的小数余量
  返 回 值：1 - 已处理；-1 - 输入或脉冲范围错误
  备    注：零速度及拒绝输出不累计补发，沿用原单周期输出上限
****************************************/
static int16 calcSpindleSpeedPulse(struct _SSpindle* pSSpindle, double dDist, double dPulseToMm, int16* pPulse)
{
    double dRawPulse;
    double dPulse;
    int16 cResult;
    *pPulse = 0;
    if (!isfinite(dDist) || !isfinite(dPulseToMm) || dPulseToMm <= 0.0 || isZero(dPulseToMm) ||
        !isfinite(pSSpindle->dSpeedPulseRemainder))
    {
        pSSpindle->dSpeedPulseRemainder = 0.0;
        parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
        return -1;
    }
    if (dDist == 0.0)
    {
        pSSpindle->dSpeedPulseRemainder = 0.0;
        return 1;
    }
    dRawPulse = dDist / dPulseToMm;
    dPulse = dRawPulse + (fabs(dRawPulse) <= 1000.0 ? pSSpindle->dSpeedPulseRemainder : 0.0);
    //zxr: 原round宏先转整数，必须在调用原脉冲接口前检查范围
    if (!isfinite(dPulse) || dPulse >= 32767.5 || dPulse <= -32768.5)
    {
        pSSpindle->dSpeedPulseRemainder = 0.0;
        parseRunningError(9999, GCODE_ERROR_NUM152, 9999);
        return -1;
    }
    //zxr: 余量进位会超过原上限时回退本拍值，由原接口统一取整一次
    if (fabs(dPulse) >= 1000.5 && fabs(dRawPulse) <= 1000.0)
    {
        dPulse = dRawPulse;
    }
    cResult = calcPulseWithCheck(dPulse, 1.0, pPulse);
    pSSpindle->dSpeedPulseRemainder = (cResult == 1 && fabs(dRawPulse) <= 1000.0)
        ? dPulse - *pPulse : 0.0;
    return cResult;
}

/*************************************************
  函数名称：constVelMotion(struct _SSpindle* pSSpindlePointer)
  函数功能  G97 恒转速控制
  输入参数：pSSpindlePointer，主轴结构体指针
  输出参数：
  返 回 值：-1  参数或脉冲计算错误；-3  物理轴映射无效
			1	完成
  备    注：
**************************************************/
int8 constVelMotion(struct _SSpindle* pSSpindlePointer)
{
	double dSpeed;//速度输出
	struct SSingleAxis* pSSingleAxisPointer;
	uint8 ucAxis;
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射
	if (pSSpindlePointer == NULL)
	{
		return -1;
	}
	//单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return -3;
	}
	
	if(isZero(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm))//增加校验
	{
		//zxr: 非法当量退出时清除本拍输出和余量，恢复后不补发
		pSRun->sAxisDeltaPulse[ucAxis] = 0;
		pSSpindlePointer->dSpeedPulseRemainder = 0.0;
			parseRunningError(9999,GCODE_ERROR_NUM153,9999);
			return -1;
	}

	switch (pSSingleAxisPointer->ucInpStatus)
	{
	case INP_INITIATION://初始化 ::G97恒转速
		//目标末速度 受单轴最大速度限制
		pSSingleAxisPointer->SPlanData.dPrimalVe = pSSingleAxisPointer->cSpindleDir * min(pSSingleAxisPointer->dSpindleSpeed * pSSingleAxisPointer->dFeedRatio, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax);
		//末速度 目标末速度为0时 末速度为起跳速度
		pSSingleAxisPointer->SPlanData.dVe = isZero(pSSingleAxisPointer->SPlanData.dPrimalVe) ? sgn(pSSingleAxisPointer->dCurrentSpeed) * g_SSystemPara.dSpindleVe : pSSingleAxisPointer->SPlanData.dPrimalVe;
		//起始速度 当前速度为0时 获取起跳速度
		pSSingleAxisPointer->SPlanData.dVs = isZero(pSSingleAxisPointer->dCurrentSpeed) ? sgn(pSSingleAxisPointer->SPlanData.dVe) * min(g_SSystemPara.dSpindleVs, fabs(pSSingleAxisPointer->SPlanData.dVe)) : pSSingleAxisPointer->dCurrentSpeed;

		//加速曲线
		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
		//减速曲线
		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		// pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
		// pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);
		pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? g_SSystemPara.dSysPlanSpindleAcc * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : g_SSystemPara.dSysPlanSpindleAcc;//修改为专用于主轴加速度
		pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? g_SSystemPara.dSysPlanSpindleDec * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : g_SSystemPara.dSysPlanSpindleDec;//修改为专用于主轴减速度

		//速度规划
		preparePlanningVelCombine(&pSSingleAxisPointer->SPlanData,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVeerDeltaVMax);

		//初始化变量
		pSSingleAxisPointer->SPlanData.unCurrentT = 0;
		pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//跳转到 RUNNING://输出速度曲线

	case INP_RUNNING://输出速度曲线

		//获取速度
		pSSingleAxisPointer->SPlanData.unCurrentT += 1;
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);
		//计算脉冲增量
		//pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
		//zxr: 本周期未发出的小数脉冲留给下周期，不因低速或重规划丢失
		if (calcSpindleSpeedPulse(pSSpindlePointer, dSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,
			&pSRun->sAxisDeltaPulse[ucAxis]) != 1)
		{
			return -1;
		}

		//加速/减速完成?
		if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)
		{
			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成  //跳转到加速完成  匀速部分
		}
		break;

	case INP_FINISHED://加速完成  匀速部分
		//计算当前速度
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = pSSingleAxisPointer->SPlanData.dPrimalVe;
		//根据速度换算脉冲
		//pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
		//zxr: 匀速输出累计小数脉冲，直接返回原脉冲接口结果
		return (int8)calcSpindleSpeedPulse(pSSpindlePointer, dSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,
			&pSRun->sAxisDeltaPulse[ucAxis]);
	}
	return 1;
}




/*************************************************
  函数名称：constLineVelMotion(struct _SSpindle* pSSpindlePointer)
  函数功能  G96 恒线速度控制   G0指令下 直接加速到G0终点的转速，G1+指令下 跟随X坐标加减速
  输入参数：pSSpindlePointer，主轴结构体指针
  输出参数：
  返 回 值：-1  参数或脉冲计算错误；-3  物理轴映射无效
			1	完成
  备    注：
**************************************************/
int8 constLineVelMotion(struct _SSpindle* pSSpindlePointer)
{
	double dSpeed;//速度输出
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体
	uint8 ucAxis;		//当前轴号
	uint8 ucFollowAxis;//跟随的轴
	double dRealTimePointFollow;//跟随的轴实时点坐标
	double absX;	//直径
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射
	struct SInterpolationPre* pSPreprocess;//预处理指针
	double dTargetVe;//目标末速度
	if (pSSpindlePointer == NULL)
	{
		return -1;
	}

	//单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;
	//获取预处理结构体
	pSPreprocess = &pSSpindlePointer->pSChannel->SPreProcess;
	//跟随的轴
	ucFollowAxis = pSSpindlePointer->spindleFollowAxis;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return -3;
	}

	if(isZero(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm))//增加校验
	{
		//zxr: 非法当量退出时清除本拍输出和余量，恢复后不补发
		pSRun->sAxisDeltaPulse[ucAxis] = 0;
		pSSpindlePointer->dSpeedPulseRemainder = 0.0;
		parseRunningError(9999,GCODE_ERROR_NUM153,9999);
		return -1;
	}

	//获取跟随轴的实时坐标
	if(getSingleAxisRealTimePoint(&dRealTimePointFollow, pSSpindlePointer->spindleFollowAxis, ucMapArray) != 1)//
	{
		parseRunningError(9999,GCODE_ERROR_NUM150,9999);
		return -1;
	}


	//判断行号是否一样
	//if (pSSingleAxisPointer->unGCodeLineNum != pSPreprocess->unGCodeLineNum)
	//{
	if (pSSpindlePointer->toUpdateG96SpindleSpeed == TRUE)//更新G96的主轴转速  2023.5.13 朱稳中
	{
		//继承行号
		pSSingleAxisPointer->unGCodeLineNum = pSPreprocess->unGCodeLineNum;
		//继承指令类型
		pSSingleAxisPointer->unCommandType = pSPreprocess->unCommandType;
		pSSingleAxisPointer->ucInpStatus = INP_INITIATION;//重新规划
	}
	if (getBits(pSSingleAxisPointer->unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_QUICKMOVE)
	{
		pSSingleAxisPointer->ucInpStatus = INP_INITIATION;//重新规划
	}


	switch (pSSingleAxisPointer->ucInpStatus)
	{
	case INP_INITIATION://初始化

		if (getBits(pSSingleAxisPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) // 快速运动下 根据运动终点加减速
		{
			//直径值
			absX = 2 * fabs(getSingleAxisEndPoint(pSSpindlePointer->pSChannel, ucFollowAxis) - pSRun->SCurrentOffset.dArray[ucFollowAxis]);
		}
		else
		{
			//获取跟随轴的直径 （减去偏移值）
			absX = fabs(2 * (dRealTimePointFollow - pSRun->SCurrentOffset.dArray[pSSpindlePointer->spindleFollowAxis]));
		}

		//末速度(度/ms) 不受倍率影响   ①G96跟随的轴坐标为0时出现除0问题，坐标看作为EPSINON计算   ②其他情况  根据线速度计算
		dTargetVe = isZero(absX) ? pSSingleAxisPointer->cSpindleDir * pSSingleAxisPointer->dFeedrate * 1000.0 / (PI * 2 * EPSINON) * DEGREE_OF_CIRCLE
			: pSSingleAxisPointer->cSpindleDir * pSSingleAxisPointer->dFeedrate * 1000.0 / (PI * absX) * DEGREE_OF_CIRCLE;
		//目标末速度 G50(度/ms)、目标速度(度/ms)、最大速度(度/ms) 取最小值

		dTargetVe = pSSingleAxisPointer->cSpindleDir * min(fabs(dTargetVe), min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax));

		if (isZero(dTargetVe - pSSingleAxisPointer->SPlanData.dPrimalVe))
		{
			//zxr: 目标未变且曲线已结束时保持目标速度，未结束则继续原加减速
			if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)
			{
				pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
				pSSingleAxisPointer->dCurrentSpeed = dTargetVe;
				return (int8)calcSpindleSpeedPulse(pSSpindlePointer, dTargetVe, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,
					&pSRun->sAxisDeltaPulse[ucAxis]);
			}
		}
		else//速度不相等
		{
			//目标末速度 受单轴最大速度限制
			pSSingleAxisPointer->SPlanData.dPrimalVe = dTargetVe;
			//zxr: 保留真实零目标，限速为零时沿原曲线减速到零
			pSSingleAxisPointer->SPlanData.dVe = pSSingleAxisPointer->SPlanData.dPrimalVe;
			//起始速度 当前速度为0时 获取起跳速度
			pSSingleAxisPointer->SPlanData.dVs = isZero(pSSingleAxisPointer->dCurrentSpeed) ? sgn(pSSingleAxisPointer->SPlanData.dVe) * min(g_SSystemPara.dSpindleVs, fabs(pSSingleAxisPointer->SPlanData.dVe)) : pSSingleAxisPointer->dCurrentSpeed;

			//加速曲线
			pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
			//减速曲线
			pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
			//从系统参数获取加速度(上一段加速度重算后与系统参数不一致)
			// pSSingleAxisPointer->SPlanData.dAcc = getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
			// pSSingleAxisPointer->SPlanData.dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);
			pSSingleAxisPointer->SPlanData.dAcc = g_SSystemPara.dSysPlanSpindleAcc;//修改为专用于主轴加速度
			pSSingleAxisPointer->SPlanData.dDec = g_SSystemPara.dSysPlanSpindleDec;//修改为专用于主轴减速度
			//速度规划
			preparePlanningVelCombine(&pSSingleAxisPointer->SPlanData,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVeerDeltaVMax);

			//初始化变量
			pSSingleAxisPointer->SPlanData.unCurrentT = 0;
		}

		pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//跳转到 RUNNING://输出速度曲线

	case INP_RUNNING://输出速度曲线

		//获取速度
		pSSingleAxisPointer->SPlanData.unCurrentT += 1;
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);
		//计算脉冲增量
		//pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
		//zxr: 本周期未发出的小数脉冲留给下周期，不因低速或重规划丢失
		if (calcSpindleSpeedPulse(pSSpindlePointer, dSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,
			&pSRun->sAxisDeltaPulse[ucAxis]) != 1)
		{
			return -1;
		}
		//加速/减速完成?
		if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)
		{
			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成  //跳转到加速完成  匀速部分
		}
		break;

	case INP_FINISHED://加速完成  匀速部分
		//计算当前速度
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = pSSingleAxisPointer->SPlanData.dPrimalVe;
		//根据速度换算脉冲
		//pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
		//zxr: 匀速输出累计小数脉冲，直接返回原脉冲接口结果
		return (int8)calcSpindleSpeedPulse(pSSpindlePointer, dSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,
			&pSRun->sAxisDeltaPulse[ucAxis]);
	}
	return 1;

	//if (getBits(pSSingleAxisPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)//当前指令类型为快速运动
	//{//快速运动下的G96
	//	switch (pSSingleAxisPointer->ucInpStatus)
	//	{
	//	case INP_INITIATION://G96  G0  类似G97的方式加速（不跟随X轴）

	//		//末速度(度/ms) 不受倍率影响   ①G96跟随的轴坐标为0时出现除0问题，坐标看作为EPSINON计算   ②其他情况  根据线速度计算
	//		pSSingleAxisPointer->SPlanData.dPrimalVe = isZero(getSingleAxisEndPoint(pSSpindlePointer->pSChannel, ucFollowAxis) - pSRun->SCurrentOffset.dArray[ucFollowAxis - 1]) ?
	//			pSSingleAxisPointer->cSpindleDir * pSSingleAxisPointer->dFeedrate * 1000.0 / (PI * 2 * EPSINON) * DEGREE_OF_CIRCLE
	//			: pSSingleAxisPointer->cSpindleDir * pSSingleAxisPointer->dFeedrate * 1000.0 / (PI * 2 * fabs(getSingleAxisEndPoint(pSSpindlePointer->pSChannel, ucFollowAxis) - pSRun->SCurrentOffset.dArray[ucFollowAxis - 1])) * DEGREE_OF_CIRCLE;
	//		//目标末速度 G50(度/ms)、目标速度(度/ms)、最大速度(度/ms) 取最小值
	//		pSSingleAxisPointer->SPlanData.dPrimalVe = pSSingleAxisPointer->cSpindleDir * min(fabs(pSSingleAxisPointer->SPlanData.dPrimalVe), min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax));
	//		//末速度  继承目标末速度  目标末速度为0时获取系统参数
	//		pSSingleAxisPointer->SPlanData.dVe = isZero(pSSingleAxisPointer->SPlanData.dPrimalVe) ? pSSingleAxisPointer->cSpindleDir * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe : pSSingleAxisPointer->SPlanData.dPrimalVe;
	//		//起始速度（继承当前速度，当前速度为0时则继承起跳速度与末速度的最小值)
	//		pSSingleAxisPointer->SPlanData.dVs = isZero(pSSingleAxisPointer->dCurrentSpeed) ? pSSingleAxisPointer->cSpindleDir * min(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVs, fabs(pSSingleAxisPointer->SPlanData.dVe)) : pSSingleAxisPointer->dCurrentSpeed;

	//		//加减速曲线
	//		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
	//		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
	//		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致
	//		pSSingleAxisPointer->SPlanData.dAcc = getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
	//		pSSingleAxisPointer->SPlanData.dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);

	//		//速度规划
	//		preparePlanningVelCombine(&pSSingleAxisPointer->SPlanData,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVeerDeltaVMax);
	//		//初始化变量
	//		pSSingleAxisPointer->SPlanData.unCurrentT = 0;//当前时间为0
	//		pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//跳转到 21://输出速度曲线

	//	case INP_RUNNING://输出速度曲线
	//		//获取速度
	//		pSSingleAxisPointer->SPlanData.unCurrentT += 1;//时间增加
	//		//获取当前速度(r/ms)
	//		pSSingleAxisPointer->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);

	//		//计算脉冲增量
	//		pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);

	//		if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)//加速/减速完成?
	//		{
	//			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成  跳转到:加速完成  匀速部分
	//		}
	//		break;

	//	case INP_FINISHED://加速完成  匀速部分
	//		//输出目标末速度
	//		pSSingleAxisPointer->dCurrentSpeed = dSpeed = pSSingleAxisPointer->SPlanData.dPrimalVe;
	//		//计算脉冲增量
	//		pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
	//		break;
	//	}
	//}
	//else
	//{
	//	switch (pSSingleAxisPointer->ucInpStatus)
	//	{
	//	case INP_INITIATION:// G96  G1+

	//		pSSingleAxisPointer->SPlanData.dPrimalVe = pSSingleAxisPointer->cSpindleDir * pSSingleAxisPointer->dFeedrate;//末速度 m/ms (不受倍率影响)

	//		//判断 ①当前转速超过G50限制转速？  ②当前转速超过系统最大转速？  ③G96跟随的轴 当前坐标为0？
	//		if (fabs(pSSingleAxisPointer->dCurrentSpeed) > pSSingleAxisPointer->dLimitedSpindleSpeed
	//			|| fabs(pSSingleAxisPointer->dCurrentSpeed) > g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax
	//			|| isZero(absX))
	//		{
	//			//以G97的方式减速 (度/ms)

	//			//如果指令为0 修正为EPSINON 防止除0
	//			if (isZero(absX))
	//			{
	//				absX = EPSINON;
	//			}

	//			//末速度  G50(度/ms)   最大速度(度/ms)    目标速度(度/ms) 取最小值
	//			pSSingleAxisPointer->SPlanData.dVe = pSSingleAxisPointer->cSpindleDir * min(min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax), fabs(pSSingleAxisPointer->SPlanData.dPrimalVe * 1000 / (PI * absX) * DEGREE_OF_CIRCLE));//G50限制的转速
	//			pSSingleAxisPointer->SPlanData.dVs = pSSingleAxisPointer->dCurrentSpeed;//起始速度为当前转速（r/ms)

	//			//加减速曲线
	//			pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
	//			pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
	//			//从系统参数获取加速度(上一段加速度重算后与系统参数不一致)
	//			pSSingleAxisPointer->SPlanData.dAcc = getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
	//			pSSingleAxisPointer->SPlanData.dDec = getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);

	//			pSSingleAxisPointer->ucInpStatus = INP_DECELERATE;//G96  G1+输出速度曲线
	//		}
	//		else
	//		{//以G96的方式减速 (m/ms)

	//			//结合当前点计算 当前点的转速超过G50 单轴最大速度
	//			if (pSSingleAxisPointer->SPlanData.dPrimalVe * 1000 / (PI * absX) * DEGREE_OF_CIRCLE > min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax))
	//			{
	//				//以G50 单轴最大速度反算线速度  作为规划的目标线速度
	//				pSSingleAxisPointer->SPlanData.dVe = min(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax, pSSingleAxisPointer->dLimitedSpindleSpeed) * PI * absX / 1000.0 / DEGREE_OF_CIRCLE;
	//			}
	//			else
	//			{
	//				//末速度   继承目标末速度  目标末速度为0时 由起跳速度反算当前点线速度
	//				pSSingleAxisPointer->SPlanData.dVe = isZero(pSSingleAxisPointer->SPlanData.dPrimalVe) ? sgn(pSSingleAxisPointer->dCurrentSpeed) * (PI * absX * (g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe / DEGREE_OF_CIRCLE / 1000.0)) : pSSingleAxisPointer->SPlanData.dPrimalVe;
	//			}
	//			//起始速度 继承当前速度(转化为当前点的线速度)  当前速度(°/ms)为0时，由起跳速度反算当前点的线速度
	//			pSSingleAxisPointer->SPlanData.dVs = isZero(pSSingleAxisPointer->dCurrentSpeed) ?
	//				pSSingleAxisPointer->cSpindleDir * min((PI * absX * g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVs / 1000.0 / DEGREE_OF_CIRCLE), fabs(pSSingleAxisPointer->SPlanData.dVe)) : PI * absX * pSSingleAxisPointer->dCurrentSpeed / 1000.0 / DEGREE_OF_CIRCLE;

	//			//加减速曲线
	//			pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
	//			pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;

	//			//从系统参数获取加速度 并修正
	//			pSSingleAxisPointer->SPlanData.dAcc = PI * absX * getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray) / 1000.0 / DEGREE_OF_CIRCLE;
	//			pSSingleAxisPointer->SPlanData.dDec = PI * absX * getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray) / 1000.0 / DEGREE_OF_CIRCLE;
	//			pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//G96  G1+输出速度曲线
	//		}
	//		//速度规划
	//		preparePlanningVelCombine(&pSSingleAxisPointer->SPlanData,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVeerDeltaVMax);
	//		//初始化变量
	//		pSSingleAxisPointer->SPlanData.unCurrentT = 0;//时间置0

	//	case INP_DECELERATE://C轴减速
	//		if (pSSingleAxisPointer->ucInpStatus == INP_DECELERATE)
	//		{
	//			pSSingleAxisPointer->SPlanData.unCurrentT += 1;//时间增加
	//			//获取速度 不受最大速度和G50限制
	//			pSSingleAxisPointer->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);

	//			//计算脉冲增量
	//			pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);

	//			if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)//加速/减速完成?
	//			{
	//				pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成  跳转到完成部分
	//			}
	//			break;
	//		}

	//	case INP_RUNNING://G96  G1+输出速度曲线
	//		pSSingleAxisPointer->SPlanData.unCurrentT += 1;//时间增加
	//		//获取速度(m/ms)并换算成转速(度/ms)
	//		dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData) * 1000 / (PI * absX) * DEGREE_OF_CIRCLE;

	//		//最大速度  和 G50限制主轴转速 (r/min)->(r/ms)
	//		pSSingleAxisPointer->dCurrentSpeed = dSpeed = pSSingleAxisPointer->cSpindleDir * min(min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax), fabs(dSpeed));

	//		//计算脉冲增量
	//		pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);

	//		//加速/减速完成?
	//		if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)
	//		{
	//			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成///跳转到G96  G1+加速完成  匀速部分
	//		}

	//		break;

	//	case INP_FINISHED://G96  G1+加速完成  匀速部分
	//		//输出目标末速度(m/ms)并转化为转速(度/ms)
	//		dSpeed = pSSingleAxisPointer->SPlanData.dPrimalVe * 1000 / (PI * absX) * DEGREE_OF_CIRCLE;

	//		//G50限制主轴转速 (r/min)->(r/ms)
	//		pSSingleAxisPointer->dCurrentSpeed = dSpeed = pSSingleAxisPointer->cSpindleDir * min(min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax), fabs(dSpeed));
	//		//计算脉冲增量
	//		pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
	//		break;

	//	}
	//}
}




/* 苏向标 2026-10-14：减速结束后等待取消使能和占用释放。 */
/*************************************************
  函数名称：commonStopMotion
  函数功能：复用原减速规划，插补结束后推进取消使能并释放本对象占用
  输入参数：pSSpindlePointer - 已获得本轴控制权的主轴对象
  输出参数：更新减速状态、脉冲、使能及占用
  返 回 值：1 - 本次阶段处理结束；0 - 取消使能等待；负值 - 参数或规划异常
  备    注：苏向标 2026-10-14；返回1不单独代表停止收尾完成，完成状态由isSpindleStopFinished判断
****************************************/
int8 commonStopMotion(struct _SSpindle* pSSpindlePointer)
{
	double dSpeed;//速度输出
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体
	uint8 ucAxis;//当前轴号
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射

	if (pSSpindlePointer == NULL)//空指针
	{
		return -1;
	}

	//获取单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return -3;
	}

	switch (pSSingleAxisPointer->ucInpStatus)
	{
	case INP_INITIATION: //减速停
		//目标末速度
		pSSingleAxisPointer->SPlanData.dPrimalVe = 0;
		//起跳速度  与当前速度取最小值
		pSSingleAxisPointer->SPlanData.dVe = sgn(pSSingleAxisPointer->dCurrentSpeed) * min(fabs(pSSingleAxisPointer->dCurrentSpeed), g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVe);
		//起始速度(r/ms)
		pSSingleAxisPointer->SPlanData.dVs = pSSingleAxisPointer->dCurrentSpeed;

		//加减速曲线
		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		// pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
		// pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);
		pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? g_SSystemPara.dSysPlanSpindleAcc * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : g_SSystemPara.dSysPlanSpindleAcc;//修改为专用于主轴加速度
		pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? g_SSystemPara.dSysPlanSpindleDec * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : g_SSystemPara.dSysPlanSpindleDec;//修改为专用于主轴减速度

		if(pSSingleAxisPointer->SPlanData.dAcc <= 0 || pSSingleAxisPointer->SPlanData.dDec <= 0)//增加校验
		{
			return -1;
		}

		//速度规划
		preparePlanningVelCombine(&pSSingleAxisPointer->SPlanData,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVeerDeltaVMax);
		//初始化变量
		pSSingleAxisPointer->SPlanData.unCurrentT = 0;//时间置0
		pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//跳转到输出速度

	case INP_RUNNING://输出速度
		//时间增加
		pSSingleAxisPointer->SPlanData.unCurrentT += 1;
		//获得当前速度 (r/ms)
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = nextstepPlanningVelCombine(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);
		//计算脉冲增量
		//pSRun->sAxisDeltaPulse[ucAxis] = round(dSpeed / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
		//zxr: 本周期未发出的小数脉冲留给下周期，不因低速或重规划丢失
		if (calcSpindleSpeedPulse(pSSpindlePointer, dSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,
			&pSRun->sAxisDeltaPulse[ucAxis]) != 1)
		{
			return -1;
		}
		if (pSSingleAxisPointer->SPlanData.unCurrentT >= pSSingleAxisPointer->SPlanData.unT5)//加速/减速完成?
		{
			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;//插补完成
		}
		break;

    case INP_FINISHED:
        /* 苏向标 2026-10-14：插补完成后，等待独立取消使能和资源收尾。 */
        pSSingleAxisPointer->dCurrentSpeed = 0.0;
        pSRun->sAxisDeltaPulse[ucAxis] = 0;
        pSSpindlePointer->toDisableAxis = TRUE;
        {
            int8 cResult = setSpindleEnabled(pSSpindlePointer, FALSE); /**< 收尾结果。 */
            if (cResult != 1)
            {
                return cResult;
            }
        }
        pSSpindlePointer->toDisableAxis = FALSE;
        axisManagerReleaseByStruct(pSSpindlePointer->ucOccupation,
            ucMapArray, AXIS_NUM_MAX, &pSSpindlePointer->ucOccupation);
        pSSingleAxisPointer->ucRunMode = SPINDLE_SINGLEAXIS_MODE;
		break;
	}
	return 1;
}


/*************************************************
  zxr: 错角或无效反馈不得作为正常完成，沿原停轴与使能同步恢复
  函数名称：failPreciseStopFeedback
  函数功能：准停反馈失败时报警并请求原普通停止
  输入参数：pSSpindle - 主轴对象；usError - 原有报警号
  输出参数：清观察计数，发布报警及停止请求
  返 回 值：-1 - 本轮准停失败
  备    注：
**************************************************/
static int8 failPreciseStopFeedback(struct _SSpindle* pSSpindle, uint16 usError)
{
    clearPreciseStopFeedback(pSSpindle);
    parseRunningError(pSSpindle->pSChannel->SRun.unGCodeLineNum, usError, 9999);
    spindleCommonStop(pSSpindle);
    return -1;
}

/*************************************************
  zxr: 有限分辨率下到角且连续稳定才完成，无反馈动力刀保留原软件收尾
  函数名称：waitPreciseStopFeedback
  函数功能：命令结束后核对对应编码器角度和连续稳定
  输入参数：pSSpindle - 当前准停对象
  输出参数：更新本主轴等待及稳定计数
  返 回 值：1 - 确认完成；0 - 等待；-1 - 已报警并请求停止
  备    注：
**************************************************/
static int8 waitPreciseStopFeedback(struct _SSpindle* pSSpindle)
{
    struct SChannel* pChannel = pSSpindle->pSChannel;
    struct SSingleAxis* pAxis = pSSpindle->pSSpindleAxis;
    struct SEncoder* pEncoder;
    int64 lEncoder;
    int64 lCirclePulse;
    double dPulseDegree;
    double dActualDegree;
    double dTargetDegree;
    double dAngleError;
    double dTolerance;
    uint8 ucAxis = pAxis->ucAxis;

    if (pSSpindle == &pChannel->SSpindle)
    {
        pEncoder = pChannel->pSSpindleEncoder;
    }
    else if (pSSpindle == &pChannel->SSecondSpindle)
    {
#if SECONDSPINDLE_USE_NOENCODER
        // zxr: 已配置无编码器动力刀保持命令完成语义，不以模拟计数证明机械停稳
        return 1;
#else
        pEncoder = pChannel->pSSecondSpindleEncoder;
#endif
    }
    else
    {
        return failPreciseStopFeedback(pSSpindle, GCODE_ERROR_NUM153);
    }
    if (pEncoder == NULL || pEncoder->usEncoderCirclePulseNum < 4
        || ucAxis >= AXIS_NUM_MAX || pChannel->pMapAxisArray == NULL
        || pChannel->pMapAxisArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX
        || !isfinite(pAxis->SPlanData.dTc) || !isZero(pAxis->SPlanData.dTc - 1.0)
        || !isfinite(pSSpindle->fPreciseStopDegree))
    {
        return failPreciseStopFeedback(pSSpindle, GCODE_ERROR_NUM153);
    }
    dPulseDegree = g_SSystemPara.SAxisPara[pChannel->pMapAxisArray[ucAxis]].dPulseToMm;
    if (!isfinite(dPulseDegree) || dPulseDegree <= 0.0 || dPulseDegree > DEGREE_OF_CIRCLE)
    {
        return failPreciseStopFeedback(pSSpindle, GCODE_ERROR_NUM153);
    }
    lEncoder = pEncoder->lEncoderAbsValue;
    lCirclePulse = lEncoder % (int64)pEncoder->usEncoderCirclePulseNum;
    if (lCirclePulse < 0)
    {
        lCirclePulse += pEncoder->usEncoderCirclePulseNum;
    }
    dActualDegree = (double)lCirclePulse * DEGREE_OF_CIRCLE / pEncoder->usEncoderCirclePulseNum;
    dTargetDegree = surplusDoubleToPositive(pSSpindle->fPreciseStopDegree, (int)DEGREE_OF_CIRCLE);
    dAngleError = fabs(dActualDegree - dTargetDegree);
    if (dAngleError > DEGREE_OF_CIRCLE / 2.0)
    {
        dAngleError = DEGREE_OF_CIRCLE - dAngleError;
    }
    // zxr: 容差仅覆盖一个反馈计数与半个命令脉冲的量化误差
    dTolerance = DEGREE_OF_CIRCLE / pEncoder->usEncoderCirclePulseNum + dPulseDegree / 2.0;
    if (pSSpindle->usPreciseStopWaitTicks == 0
        || lEncoder != pSSpindle->lPreciseStopLastEncoder || pEncoder->usDelta != 0
        || pChannel->SRun.sLastDeltaPulse[ucAxis] != 0 || dAngleError > dTolerance)
    {
        pSSpindle->usPreciseStopStillTicks = 0;
    }
    else if (pSSpindle->usPreciseStopStillTicks < PRECISE_STOP_STILL_TICKS)
    {
        pSSpindle->usPreciseStopStillTicks++;
    }
    pSSpindle->lPreciseStopLastEncoder = lEncoder;
    // zxr: 到位稳定后的取消使能延时不算反馈超时，反馈再次变化才恢复等待
    if (pSSpindle->usPreciseStopStillTicks >= PRECISE_STOP_STILL_TICKS)
    {
        return 1;
    }
    if (pSSpindle->usPreciseStopWaitTicks < PRECISE_STOP_TIMEOUT_TICKS)
    {
        pSSpindle->usPreciseStopWaitTicks++;
    }
    if (pSSpindle->usPreciseStopWaitTicks >= PRECISE_STOP_TIMEOUT_TICKS)
    {
        return failPreciseStopFeedback(pSSpindle, GCODE_ERROR_NUM154);
    }
    return 0;
}

/*************************************************
  函数名称：preciseStopMotion(struct _SSpindle* pSSpindlePointer)
  函数功能  主轴准停， 根据系统参数设置主轴准停的位置，进行距离规划，减速停到目标点
  输入参数：*pSSpindlePointer，结构体struct _SSpindle指针
  输出参数：更新准停规划、状态与脉冲
  返 回 值：负值  参数、映射、补充定位规划或反馈确认错误
			0	等待实际反馈到位稳定
			1	本周期处理结束，不代表准停已经完成
  备    注：
**************************************************/
int8 preciseStopMotion(struct _SSpindle* pSSpindlePointer)
{
	struct SSingleAxis* pSSingleAxisPointer; //单轴结构体
	const uint8* ucMapArray;//映射
	double dRelaEndPoint;	//减速停点 准停用
	struct _SRun* pSRun;	//运行结构体
	double dRelaDist;		//目标点和减速停点的距离
	double dDeltaL;		//相对起点的距离 主轴准停输出时用
	double dSpeed;	//速度输出
	uint8 ucAxis;	//当前轴号
	double dSde;	//减速距离   准停用来计算
	bool bZeroSpeedPosition; // zxr: 标记本次是否补充静止零起跳定位速度

	if (pSSpindlePointer == NULL)
	{
		return -1;
	}

	//获取单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return -3;
	}

	switch (pSSingleAxisPointer->ucInpStatus)
	{

	case INP_INITIATION://准停
		clearPreciseStopFeedback(pSSpindlePointer); // zxr: 每次新规划重新观察反馈
		//根据当前速度判断方向  方向保存在起点向量
		pSSingleAxisPointer->dStartVector = sgn(pSSingleAxisPointer->dCurrentSpeed);
		//没有方向(主轴当前转速为0时出现)
		if (isZero(pSSingleAxisPointer->dStartVector))
		{
			pSSingleAxisPointer->dStartVector = ARC_CW;//正转
		}
		//末速度(不带方向) 选择当前速度和系统单轴末速度的最小值
		pSSingleAxisPointer->SPlanData.dVe = min(fabs(pSSingleAxisPointer->dCurrentSpeed), g_SSystemPara.dSpindleVe);
		//目标速度为0 因为要让它停下
		pSSingleAxisPointer->SPlanData.dPrimalVe = 0;
		//初速度和最大速度为当前速度
		pSSingleAxisPointer->SPlanData.dVs = pSSingleAxisPointer->SPlanData.dVmax = isZero(pSSingleAxisPointer->dCurrentSpeed) ?
			g_SSystemPara.dSpindleVs : fabs(pSSingleAxisPointer->dCurrentSpeed);

		//加减速曲线
		pSSingleAxisPointer->SPlanData.ucAccVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucAccVelPlanType;
		pSSingleAxisPointer->SPlanData.ucDecVelPlanType = g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].ucDecVelPlanType;
		//从系统参数获取加速度(上一段加速度重算后与系统参数不一致) //到达指定转速后依据指定加速度比例减小加速度 洪冠鹏2024/3/23
		// pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisAcc(ucAxis, pSSingleAxisPointer->SPlanData.ucAccVelPlanType, ucMapArray);
		// pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray) * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : getSingleAxisDec(ucAxis, pSSingleAxisPointer->SPlanData.ucDecVelPlanType, ucMapArray);
		pSSingleAxisPointer->SPlanData.dAcc = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? g_SSystemPara.dSysPlanSpindleAcc * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : g_SSystemPara.dSysPlanSpindleAcc;//修改为专用于主轴加速度
		pSSingleAxisPointer->SPlanData.dDec = (pSSingleAxisPointer->bIsChangeAcc == TRUE) ? g_SSystemPara.dSysPlanSpindleDec * pSSingleAxisPointer->ucGearForChangeRate / 100.0 : g_SSystemPara.dSysPlanSpindleDec;//修改为专用于主轴减速度

		if(pSSingleAxisPointer->SPlanData.dAcc <= 0 || pSSingleAxisPointer->SPlanData.dDec <= 0)//增加校验
		{
			return -1;
		}


		//根据曲线类型和加减速度 利用距离公式计算减速距离  不带方向 单位(度)
		if(fabs(pSSingleAxisPointer->dCurrentSpeed) <= g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVs)
		{
		   dSde = 0;
		}
		else
		{
			dSde = fabs(getFactor(pSSingleAxisPointer->SPlanData.ucDecVelPlanType) * (pSSingleAxisPointer->SPlanData.dVs * pSSingleAxisPointer->SPlanData.dVs - pSSingleAxisPointer->SPlanData.dVe * pSSingleAxisPointer->SPlanData.dVe) / pSSingleAxisPointer->SPlanData.dDec);
		}
		//获取当前点坐标
		getSingleAxisRealTimePoint(&pSSingleAxisPointer->dStartPoint, ucAxis, ucMapArray);
		pSSingleAxisPointer->dStartPoint = surplusDoubleToPositive(pSSingleAxisPointer->dStartPoint, 360); //取余
		updateSingleAxisRealTimePoint(&pSSingleAxisPointer->dStartPoint, ucAxis, ucMapArray);        //更新取余后的起点坐标
		//根据方向和起点(r)  计算终点位置(r)
		dRelaEndPoint = pSSingleAxisPointer->dStartPoint + pSSingleAxisPointer->dStartVector * dSde;

		//终点位置转化为相对的角度位置(r)  A
		dRelaEndPoint = surplusDoubleData(dRelaEndPoint, 360);//取余
		if (dRelaEndPoint < 0)//取余后为负值
		{
			dRelaEndPoint += DEGREE_OF_CIRCLE; //+1圈(360°)
		}

		//修正(补偿)距离  使其达到准停的角度
		if (isZero(pSSingleAxisPointer->dStartVector - ARC_CW))  // 正转
		{
			//与准停点的差值   根据准停的位置(°->r) 和终点位置计算
			dRelaDist = (double)pSSpindlePointer->fPreciseStopDegree - dRelaEndPoint;
		}
		else//反转
		{
			//与准停点的差值（取反)  根据准停的位置(°->r) 和终点位置计算
			dRelaDist = dRelaEndPoint - (double)pSSpindlePointer->fPreciseStopDegree;
		}
		/* 补转角与减速距离均小于半个脉冲角时，才允许直接完成。 */
		if (fabs(dRelaDist) < g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm / 2.0 &&
			dSde < g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm / 2.0)
		{
			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
			break;
		}
		if (dRelaDist < 0)//差值为负
		{
			dRelaDist += DEGREE_OF_CIRCLE;//+1圈
		}

		//规划的距离 = 减速距离+准停点与终点位置的相对距离
		pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SPlanData.dDistance = pSSingleAxisPointer->SPlanData.dTargetDist = dSde + dRelaDist;

		//防止不为0 影响输出
		pSSingleAxisPointer->SPlanData.dPreDist = 0;
		// zxr: 仅静止且起跳速度为零时补充定位速度，上限沿用旋转轴定位及主轴限速
		bZeroSpeedPosition = isZero(pSSingleAxisPointer->dCurrentSpeed) && isZero(pSSingleAxisPointer->SPlanData.dVmax);
		if (bZeroSpeedPosition)
		{
			pSSingleAxisPointer->SPlanData.dVs = 0.0;
			pSSingleAxisPointer->SPlanData.dVmax = min(g_SSystemPara.dRotationAxisG0SpeedMax,
				min(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax, pSSingleAxisPointer->dLimitedSpindleSpeed));
			if (!isfinite(g_SSystemPara.dRotationAxisG0SpeedMax)
				|| !isfinite(g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax)
				|| !isfinite(pSSingleAxisPointer->dLimitedSpindleSpeed)
				|| pSSingleAxisPointer->SPlanData.dVmax <= 0.0 || isZero(pSSingleAxisPointer->SPlanData.dVmax))
			{
				// zxr: 参数错误沿用153报警及普通停止，不额外锁存需重新初始化的155故障
				parseRunningError(pSSpindlePointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
				spindleCommonStop(pSSpindlePointer);
				return -1;
			}
		}
		//速度规划+平摊
		if (preparePlanningAndDistReAvg(&pSSingleAxisPointer->SPlanData, TRUE) != 1
			&& bZeroSpeedPosition) // zxr: 仅补充定位路径处理规划失败，原旋转准停流程保持不变
		{
			parseRunningError(pSSpindlePointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
			spindleCommonStop(pSSpindlePointer);
			return -1;
		}
		pSSingleAxisPointer->ucInpStatus = INP_RUNNING;//状态
		pSSingleAxisPointer->SPlanData.unCurrentT = 0;//时间


	case INP_RUNNING://C轴准停输出
		pSSingleAxisPointer->SPlanData.unCurrentT += 1;//时间增加
		//相对起点的距离
		dDeltaL = nextstepPlanningAndDistReAvg(pSSingleAxisPointer->SPlanData.unCurrentT, &pSSingleAxisPointer->SPlanData);
		//计算插补点的位置
		pSRun->SAbsCoord.dArray[ucAxis] = pSSingleAxisPointer->dStartPoint + pSSingleAxisPointer->dStartVector * dDeltaL;
		//取余
		pSRun->SAbsCoord.dArray[ucAxis] = surplusDoubleData(pSRun->SAbsCoord.dArray[ucAxis], 360);

		if (pSRun->SAbsCoord.dArray[ucAxis] < 0)//变成正的相对值
		{
			pSRun->SAbsCoord.dArray[ucAxis] += DEGREE_OF_CIRCLE;//加一圈
		}

		//目标点和当前点的距离  单位(度)
		dRelaDist = pSRun->SAbsCoord.dArray[ucAxis] - getSingleAxisEndPoint(pSSpindlePointer->pSChannel, ucAxis);

		//根据方向判断是否合理  不合理时根据方向加减圈数
		if (dRelaDist * pSSingleAxisPointer->dStartVector < 0)
		{
			if (fabs(dRelaDist) > g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm / 2.0) //避免由于圆整情况，导致当前点超过目标点
			{
				//根据方向加减圈数
				dRelaDist += sgn(pSSingleAxisPointer->dStartVector) * DEGREE_OF_CIRCLE;
			}
			else
			{
				dRelaDist = 0;
			}

		}

		//计算脉冲增量
		//pSRun->sAxisDeltaPulse[ucAxis] = round(dRelaDist / g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm);
		calcPulseWithCheck(dRelaDist,g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm,&pSRun->sAxisDeltaPulse[ucAxis]);
		//计算当前速度
		pSSingleAxisPointer->dCurrentSpeed = pSSingleAxisPointer->cSpindleDir *  fabs(pSSingleAxisPointer->SPlanData.dCurLeftDist - (pSSingleAxisPointer->SPlanData.dTargetDist - dDeltaL));  //2023/10/12 洪冠鹏 当前速度带方向

		//计算   !理论! 未走距离
		pSSingleAxisPointer->SPlanData.dCurLeftDist = pSSingleAxisPointer->SPlanData.dTargetDist - dDeltaL;

		if (pSSingleAxisPointer->SPlanData.dCurLeftDist < g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm / 2.0)//判断是否结束g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dPulseToMm / 2.0
		{
			pSSingleAxisPointer->ucInpStatus = INP_FINISHED;
		}
		break;


	case INP_FINISHED:
		//准停停下  0输出
		pSSingleAxisPointer->dCurrentSpeed = dSpeed = 0.0;
		pSRun->sAxisDeltaPulse[ucAxis] = 0.0;
        // zxr: 保持准停模式及占用，实际到角稳定后再执行原收尾
        {
            int8 cFeedbackResult = waitPreciseStopFeedback(pSSpindlePointer);
            if (cFeedbackResult != 1)
            {
                return cFeedbackResult;
            }
        }
		////轴释放
		//axisManagerReleaseByStruct(0x01 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSSpindlePointer->ucOccupation);
		////模式切换成单轴模式
		//pSSingleAxisPointer->ucRunMode = SPINDLE_SINGLEAXIS_MODE;
		//if (pSSpindlePointer->toDisableAxis == TRUE)
		//{
		//	setSpindleEnabled(pSSpindlePointer,FALSE);//取消主轴使能
		//	pSSpindlePointer->toDisableAxis = FALSE;//标志位置FALSE
		//}
		if (pSSpindlePointer->toDisableAxis == TRUE)// 准备取消使能主轴
		{
			if (setSpindleEnabled(pSSpindlePointer, FALSE) == 1)//取消主轴使能
			{//取消使能成功
				pSSpindlePointer->toDisableAxis = FALSE;//标志位置FALSE
				//轴释放
				axisManagerReleaseByStruct(0x01 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSSpindlePointer->ucOccupation);
				//主轴模式切换到单轴模式
				pSSingleAxisPointer->ucRunMode = SPINDLE_SINGLEAXIS_MODE;
			}
		}
		else
		{
			//轴释放
			axisManagerReleaseByStruct(0x01 << pSSingleAxisPointer->ucAxis, pSSpindlePointer->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSSpindlePointer->ucOccupation);
			//主轴模式切换到单轴模式
			pSSingleAxisPointer->ucRunMode = SPINDLE_SINGLEAXIS_MODE;
		}
		break;
	}
	return 1;
}



/*************************************************
  函数名称：spindleSpeedTest(struct _SEncoderTest* pEncoderTest, struct _SSpindle* pSSpindlePointer)
  函数功能  根据编码器计算平均转速，监控运行中的相位和速度突变
  输入参数：pEncoderTest，主轴测速结构体指针
			pSSpindlePointer，主轴结构体指针
  输出参数：更新平均转速、测速步骤和本通道达速标志
  返 回 值：-2  轴映射或通道下标无效
			-1  空指针、编码器每转计数无效或目标速度获取失败
			1	完成
  备    注：
**************************************************/
int8 spindleSpeedTest(struct _SEncoderTest* pEncoderTest, struct _SSpindle* pSSpindlePointer)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
	const uint8* ucMapArray;//映射
	uint8 ucAxis;//轴号
	struct SChannel* pSChannel;
	double dTargetSpeed = 0;
	// zxr: 达速标志按通道保存，停止清理不影响其他通道
	static bool bSpeedTransientArray[CHANNEL_NUM_MAX] = { FALSE };
	// zxr: 按通道记住目标方向，捕获一个测速周期内已完成的软件换向
	static int8 cLastSpindleDirArray[CHANNEL_NUM_MAX] = { 0 };
	bool bSuspendSpeedTransient;// zxr: 本周期是否暂停达速及速度突变判断
	if (pSSpindlePointer == NULL || pEncoderTest == NULL)//空指针
	{
		return -1;
	}

	//指向主轴单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//轴号
	ucAxis = pSSpindlePointer->pSSpindleAxis->ucAxis;
	//映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//通道
	pSChannel = pSSpindlePointer->pSChannel;
	if(pSChannel->pSSpindleEncoder == NULL)
	{
		parseRunningError(9999,GCODE_ERROR_NUM300,9999);
		return -1;
	}


	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
	{
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE)
		pEncoderTest->dSpindleAvgSpeed = g_SChannel[0].SSpindleTest.dSpindleAvgSpeed;	//Y轴机，通道二获取通道一的主轴速度，为的是通道二能使用G99
#endif
		return -2;
	}

	// zxr: 使用本通道标志前校验下标
	if (pSChannel->ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -2;
	}

	// zxr: 复用本主轴的平滑计数：目标变向或软件速度尚未过零时重置两次采样等待
	if (cLastSpindleDirArray[pSChannel->ucChannelNum] != pSSingleAxisPointer->cSpindleDir ||
		sgn(pSSingleAxisPointer->dCurrentSpeed) != pSSingleAxisPointer->cSpindleDir)
	{
		pEncoderTest->ucSmoothCount = 2;
	}
	cLastSpindleDirArray[pSChannel->ucChannelNum] = pSSingleAxisPointer->cSpindleDir;

	// zxr: 停止及换向采样等待期间，每周期撤销旧达速标志
	bSuspendSpeedTransient = isSpindleStopping(pSSpindlePointer) ||
		pSSingleAxisPointer->ucRunMode == SPINDLE_PRECISESTOP_MODE ||
		pSSingleAxisPointer->ucRunMode == SPINDLE_SINGLEAXIS_MODE ||
		pEncoderTest->ucSmoothCount > 0;
	if (bSuspendSpeedTransient)
	{
		bSpeedTransientArray[pSChannel->ucChannelNum] = FALSE;
	}

	switch (pEncoderTest->ucStep)
	{
	case 0:
		pEncoderTest->usPeriod = 0;//初始化周期
		pEncoderTest->nTotalPulse = 0;//初始化脉冲
		if (pSChannel->bCalculateSpindleSpeedFlag && pSSingleAxisPointer->ucInpStatus == INP_FINISHED)
		{//检测主轴转速的条件
			pEncoderTest->ucStep = 2;
		}
		else
		{
			pEncoderTest->ucStep = 1;
		}
	case 1:
	case 2:
		pEncoderTest->usPeriod++;//周期++
		pEncoderTest->nTotalPulse += (int)pSChannel->pSSpindleEncoder->cEncoderDir * (int)pSChannel->pSSpindleEncoder->usDelta;//脉冲数叠加
		if (pEncoderTest->usPeriod >= g_SSystemPara.usSpindleAvgPeriod)//计数达到目标计数
		{
			//计算主轴平均转速  (r/ms)
			if (pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum == 0)
				return -1;
			pEncoderTest->dSpindleAvgSpeed = (double)pEncoderTest->nTotalPulse / pEncoderTest->usPeriod / (pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum);

			if(calTargetSpindleSpeed(&pSChannel->SSpindle,&dTargetSpeed)!= 1)return -1;

			if (g_SSystemPara.bSpindleAlarm)//是否开启编码器相位错误和速度突变报警
			{
				// zxr: 换向后的旧采样窗口经过后再检查相位，不以反馈同向作为恢复条件
				if (pEncoderTest->ucSmoothCount == 0 &&
					sgn(pEncoderTest->dSpindleAvgSpeed*dTargetSpeed) == -1)//目标转速与平均转速反向
				{
					parseRunningError(pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM151, 9999);//编码器相位错误报警
					//resetButtonAPI(pSChannel->ucChannelNum);
				}
				// zxr: 停止及换向等待时不重新置位达速标志，恢复后保留原报警判断
				if (pSChannel->ucChannelStatus == SYS_IDLE && !bSuspendSpeedTransient)
				{
					if ((pEncoderTest->dSpindleAvgSpeed < dTargetSpeed*(1.0 + g_SSystemPara.fSpindleSpeedDeviation) &&
						pEncoderTest->dSpindleAvgSpeed > dTargetSpeed*(1.0 - g_SSystemPara.fSpindleSpeedDeviation))
						&& SGN(dTargetSpeed) > 0)//主轴已加速至目标转速允许范围之内
					{
						bSpeedTransientArray[pSChannel->ucChannelNum] = TRUE;// zxr: 本通道到速后开启监控
					}
					else if ((pEncoderTest->dSpindleAvgSpeed > dTargetSpeed*(1.0 + g_SSystemPara.fSpindleSpeedDeviation) &&
						pEncoderTest->dSpindleAvgSpeed < dTargetSpeed*(1.0 - g_SSystemPara.fSpindleSpeedDeviation))
						&& SGN(dTargetSpeed) < 0)//主轴已加速至目标转速允许范围之内
					{
						bSpeedTransientArray[pSChannel->ucChannelNum] = TRUE;// zxr: 本通道到速后开启监控
					}
					else//此时主轴转速在目标转速允许范围之外
					{
						// zxr: 停止清理已在每周期入口完成，此处只使用本通道标志
						if (bSpeedTransientArray[pSChannel->ucChannelNum])// zxr: 检查本通道已达速状态
						{
							parseRunningError(pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM152, 9999);//此时速度突变报警
							//resetButtonAPI(pSChannel->ucChannelNum);
							bSpeedTransientArray[pSChannel->ucChannelNum] = FALSE;// zxr: 报警后等待本通道重新到速
						}
					}

				}
			}
			// zxr: 每个平均窗口仅递减一次，跳过混合窗口及其后的完整窗口，测速照常更新
			if (pEncoderTest->ucSmoothCount > 0)
			{
				pEncoderTest->ucSmoothCount--;
			}
			if (pEncoderTest->ucStep == 2)//G33 G32需要检测主轴转速
			{
				pSChannel->dSpindleRealSpeed = pEncoderTest->dSpindleAvgSpeed;//赋值到实际主轴转速
				pSChannel->bCalculateSpindleSpeedFlag = FALSE;//标志位置回原来的值
			}
			pEncoderTest->ucStep = 0;//初始化
		}
		break;
	default:
		pEncoderTest->ucStep = 0;
	}

	return 1;
}




/*************************************************
  函数名称：setSpindleDir(struct SSingleAxis* pSSingleAxisPointer, int8 cDir)
  函数功能  设置主轴转动方向     （M03 M04调用次函数可实现）
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
			cDir    ARC_CW  正转 （1）   ARC_ANTI_CW反转 (-1)
  输出参数：
  返 回 值：-2  参数错误  方向必须为1或-1
			-1  空指针
			0   方向相同 无需设置
			1	完成
  备    注：
**************************************************/
int8 setSpindleDir(struct _SSpindle* pSSpindlePointer, int8 cDir)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针

	if (pSSpindlePointer == NULL)//空指针
	{
		return -1;
	}
	//指向主轴单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;

	if (pSSingleAxisPointer->cSpindleDir == cDir)//判断目标方向与输入的方向是否相同
	{
		return 0;//相同  返回
	}

	//ARC_CW 和 ARC_ANTI_CW  判断合法性   不合法的方向会影响主轴运动
	if (cDir != 1 && cDir != -1)
	{
		return -2;
	}

	pSSingleAxisPointer->cSpindleDir = cDir;// 设置方向

	switch (pSSingleAxisPointer->ucRunMode)
	{
	case SPINDLE_G97_MODE:
	case SPINDLE_G96ANDG0_MODE:
	case SPINDLE_G96ANDG1_MODE:
		// 如果在G96G97的运动中  则重新规划
		pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		break;

	case SPINDLE_HANDWHEEL_MODE:
	case SPINDLE_SINGLEAXIS_MODE:
	case SPINDLE_PRECISESTOP_MODE:
	case SPINDLE_COMMONSTOP_MODE:
	default:
		break;
	}
	return 1;
}


/* 苏向标 2026-10-14：停止收尾未结束时不接收新启动。 */
/*************************************************
  函数名称：spindleBegin
  函数功能：请求启动主轴，停止收尾未完成时保留停止流程
  输入参数：pSSpindlePointer - 主轴对象
  输出参数：更新允许启动时的模式和插补阶段
  返 回 值：1 - 已进入启动流程；0 - 暂不启动或模式不切换；-1 - 参数错误
  备    注：苏向标 2026-10-14；不再用新启动覆盖尚未结束的普通停止
****************************************/
int8 spindleBegin(struct _SSpindle* pSSpindlePointer)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
	const uint8* ucMapArray;//映射
	uint8 ucAxis;//轴号

	if (pSSpindlePointer == NULL || pSSpindlePointer->pSChannel == NULL || pSSpindlePointer->pSSpindleAxis == NULL)//空指针
	{
		return -1;
	}
    /* 苏向标 2026-10-14：独立停止必须收尾后才接受新运动。 */
    if (isSpindleStopping(pSSpindlePointer))
    {
        return 0;
    }
	//指向主轴单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;

	pSSingleAxisPointer->bIsChangeAcc = FALSE; //初始化标志位：主轴转速到达指定转速后修改加速度 洪冠鹏 2024/3/23
	pSSingleAxisPointer->dSpindleSpeedForChangeAcc = g_SSystemPara.Speed[pSSpindlePointer->pSChannel->ucChannelNum];
	pSSingleAxisPointer->ucGearForChangeRate = g_SSystemPara.acceleration[pSSpindlePointer->pSChannel->ucChannelNum];

	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;  //映射
	ucAxis = pSSingleAxisPointer->ucAxis;//读取轴号

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//未映射物理轴  洪冠鹏 2024/6/11
	{
		parseRunningError(pSSpindlePointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
		channelStopToIDLE(pSSpindlePointer->pSChannel);
		return -3;
	}
	if ((pSSpindlePointer->pSChannel->bChuckALoose == TRUE && pSSpindlePointer->dSpindleSpeed > g_SSystemPara.SpindleChuck[0]) ||
		(pSSpindlePointer->pSChannel->bChuckBLoose == TRUE && pSSpindlePointer->dSpindleSpeed > g_SSystemPara.SpindleChuck[1]))//达到指定转速后，夹头松开，再次启动主轴则报警  2023/10/14 洪冠鹏
	{
		parseRunningError(pSSpindlePointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM149, 9999);
		return -3;
	}
	if (pSSpindlePointer->cSpindleClampStatus == STATUS_CLAMPED)			//C轴被刹车锁紧了不能运动
	{
		parseRunningError(pSSpindlePointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM148, 9999);
		return -3;
	}

	//setSpindleEnabled(pSSpindlePointer, TRUE);//使能主轴
	switch (pSSingleAxisPointer->ucRunMode)// 根据当前的功能状态设置
	{
	case SPINDLE_G97_MODE:
	case SPINDLE_G96ANDG0_MODE:
	case SPINDLE_G96ANDG1_MODE:
	case SPINDLE_HANDWHEEL_MODE:
		//已启动或者在手轮状态下，直接返回
		break;

	case SPINDLE_SINGLEAXIS_MODE:
	case SPINDLE_PRECISESTOP_MODE:
	case SPINDLE_COMMONSTOP_MODE:
		//根据指令类型设置主轴的功能（step），并重新规划
		switch (getBits(pSSingleAxisPointer->unCommandType, CMD_SPINDLE_SPEED_ADDR))
		{
		case CMD_SPINDLE_SPEED_CV:
			pSSingleAxisPointer->ucRunMode = SPINDLE_G97_MODE;//设置功能
			break;
		case CMD_SPINDLE_SPEED_CLV:
			pSSingleAxisPointer->ucRunMode = SPINDLE_G96ANDG1_MODE;//设置功能
			break;
		}
		pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
	}
	return 0;
}


/*************************************************
  函数名称：spindleCommonStop
  函数功能：仅提交普通停止请求，下一次运动周期分类处理
  输入参数：pSSpindlePointer - 主轴或副轴结构
  输出参数：置位普通停止请求
  返 回 值：1 - 已提交；0 - 已在停止或手轮模式；负值 - 参数错误
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
int8 spindleCommonStop(struct _SSpindle* pSSpindlePointer)
{
    if (pSSpindlePointer == NULL || pSSpindlePointer->pSSpindleAxis == NULL || pSSpindlePointer->pSChannel == NULL)
    {
        return -1;
    }
    if (isSpindleStopping(pSSpindlePointer)
        || pSSpindlePointer->pSSpindleAxis->ucRunMode == SPINDLE_HANDWHEEL_MODE)
    {
        return 0;
    }
    /* 单字节置位；不在 TIM6 内覆盖正在运行的插补阶段。 */
    pSSpindlePointer->bStopRequested = TRUE;
    return 1;
}


/* 苏向标 2026-10-14：普通停止未收尾时不切换准停。 */
/*************************************************
  函数名称：spindlePreciseStop
  函数功能：在允许模式下请求准停，普通停止期间不切换模式
  输入参数：pSSpindlePointer - 主轴对象；preciseStopDegree - 可选准停角度指针，NULL使用配置值
  输出参数：更新准停角度、模式及插补阶段
  返 回 值：1 - 已设置准停流程；0 - 暂不切换；-1 - 参数错误；-2 - 模式不允许
  备    注：苏向标 2026-10-14；保留原准停规划，不代表已到达目标角度
****************************************/
int8 spindlePreciseStop(struct _SSpindle* pSSpindlePointer, float* preciseStopDegree)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针

	if (pSSpindlePointer == NULL || pSSpindlePointer->pSSpindleAxis == NULL)//空指针
	{
		return -1;
	}
    /* 苏向标 2026-10-14：独立停止必须收尾后才接受新运动。 */
    if (isSpindleStopping(pSSpindlePointer))
    {
        return 0;
    }
	//指向主轴单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
    // zxr: 活动准停及反馈等待不接受重复请求改写本轮目标
    if (pSSingleAxisPointer->ucRunMode == SPINDLE_PRECISESTOP_MODE)
    {
        return 0;
    }
	pSSingleAxisPointer->bIsChangeAcc = FALSE; //初始化标志位：主轴转速到达指定转速后修改加速度 洪冠鹏 2024/3/23
	pSSingleAxisPointer->dSpindleSpeedForChangeAcc = g_SSystemPara.Speed[pSSpindlePointer->pSChannel->ucChannelNum];
	pSSingleAxisPointer->ucGearForChangeRate = g_SSystemPara.acceleration[pSSpindlePointer->pSChannel->ucChannelNum];
	if (preciseStopDegree != NULL)
	{
		pSSpindlePointer->fPreciseStopDegree = *preciseStopDegree;// 根据传入参数
	}
	else
	{
		pSSpindlePointer->fPreciseStopDegree = g_SSystemPara.fSpindleStopDegree;
	}
	//pSSpindlePointer->preciseStopDegree = preciseStopDegree == NULL ? g_SSystemPara.usSpindleStopDegree : *preciseStopDegree;// 根据传入参数

	switch (pSSingleAxisPointer->ucRunMode)// 根据当前的功能状态设置
	{
	case SPINDLE_SINGLEAXIS_MODE:
		if (pSSingleAxisPointer->ucInpStatus == INP_FINISHED && pSSpindlePointer->pSChannel->SPreProcess.bValidQ == TRUE)//主轴未启动时 但有指定准停角度时要设置主轴准停//2024/7/11
		{
			//单轴模式下需要进行轴占用才能进准停模式
		}
		else
		{ //单轴模式下 运动未完成 不能设置主轴准停
			return -2;
		}
		//屏蔽原因： 重复按两下准停导致主轴转动 //2024/7/12 经测试不存在该问题，因此取消屏蔽
		//return -2;

	case SPINDLE_G97_MODE:
	case SPINDLE_G96ANDG0_MODE:
	case SPINDLE_G96ANDG1_MODE:
	case SPINDLE_COMMONSTOP_MODE:
		//可以准停的模式 ：单轴模式 G97 G96 减速停
		clearPreciseStopFeedback(pSSpindlePointer); // zxr: 新请求清除上次观察
		pSSingleAxisPointer->ucRunMode = SPINDLE_PRECISESTOP_MODE;//设置准停状态
		pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		return 1;

	case SPINDLE_HANDWHEEL_MODE:
	case SPINDLE_PRECISESTOP_MODE:
	default:
		// 处于手轮模式，准停模式，直接返回
		break;
	}
	return 0;
}


/*************************************************
  函数名称：getTargetSpindleSpeedAPI(struct SSingleAxis* pSSingleAxisPointer, double absX)
  函数功能  返回主轴当前的目标转速
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
  输出参数：
  返 回 值：主轴当前点的目标转速(r/ms)
  备    注：
**************************************************/
//double getTargetSpindleSpeedAPI(struct SSingleAxis* pSSingleAxisPointer, double absX)
//{
//	if (pSSingleAxisPointer == NULL)
//	{
//		return -1;
//	}
//	absX = fabs(absX);
//
//	//primaldVe
//
//	switch (pSSingleAxisPointer->ucRunStep)
//	{
//	case SPINDLE_G97_MODE://恒转速
//		return pSSingleAxisPointer->dSpindleSpeed / (60 * 1000.0) * pSSingleAxisPointer->dFeedRatio; // 根据进给速度和倍率计算  换算为 r/ms
//
//	case SPINDLE_G96ANDG0_MODE://恒线速度G0    //根据G50判断
//		return pSSingleAxisPointer->cArcDir * min(fabs(pSSingleAxisPointer->dPrimalVe), pSSingleAxisPointer->dLimitedSpindleSpeed / (60.0 * 1000));
//
//	case SPINDLE_G96ANDG1_MODE://恒线速度G1   //根据G502判断
//		//return pSSingleAxisPointer->dPrimalVe * 1000 / (PI * absX);
//		return pSSingleAxisPointer->cArcDir * min(fabs(pSSingleAxisPointer->dPrimalVe * 1000 / (PI * absX)), pSSingleAxisPointer->dLimitedSpindleSpeed / (60.0 * 1000));
//	case SPINDLE_COMMONSTOP_MODE://减速停
//	case SPINDLE_PRECISESTOP_MODE://准停
//		return 0.0;
//
//	}
//}

/*************************************************
  函数名称：spindleStopNow
  函数功能：提交立即停止请求，在下一次运动周期清零脉冲并收尾
  输入参数：pSSpindlePointer - 主轴或副轴结构
  输出参数：置位立即停止请求
  返 回 值：1 - 已提交，并非物理停稳；0 - 手轮模式；负值 - 参数错误
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
int8 spindleStopNow(struct _SSpindle* pSSpindlePointer)
{
    if (pSSpindlePointer == NULL || pSSpindlePointer->pSSpindleAxis == NULL || pSSpindlePointer->pSChannel == NULL)
    {
        return -1;
    }
    if (pSSpindlePointer->pSSpindleAxis->ucRunMode == SPINDLE_HANDWHEEL_MODE)
    {
        return 0;
    }
    pSSpindlePointer->bStopNowRequested = TRUE;
    return 1;
}

/*************************************************
  函数名称：calTargetSpindleSpeed(struct _SSpindle* pSSpindlePointer, double* pTargetSpeed)
  函数功能  根据指令类型和主轴转速计算当前点的理论主轴转速，
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
  输出参数： pTargetSpeed
  返 回 值：错误码
  备    注：//修改为状态码 + 输出参数形式，2026.4.23
**************************************************/
int8 calTargetSpindleSpeed(struct _SSpindle* pSSpindlePointer, double* pTargetSpeed)
{
	struct SSingleAxis* pSSingleAxisPointer;
	uint8 ucAxis;
	double absX;
	double dRealTimePointFollow;
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射
	double dCalcSpeed = 0; 

	if (pSSpindlePointer == NULL || pSSpindlePointer->pSSpindleAxis == NULL)
	{
		return -1;
	}
	//单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;//单轴结构体

	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;

	if(ucMapArray == NULL || ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}

	//获取跟随轴的当前坐标
	if(getSingleAxisRealTimePoint(&dRealTimePointFollow, pSSpindlePointer->spindleFollowAxis, ucMapArray) != 1)
	{
		parseRunningError(pSRun->unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
		return -3;
	}
	//计算直径（绝对值）
	absX = fabs(2 * (dRealTimePointFollow - pSRun->SCurrentOffset.dArray[pSSpindlePointer->spindleFollowAxis]));
	//读取轴号

	switch (pSSingleAxisPointer->ucRunMode)
	{
	case SPINDLE_G97_MODE://恒转速
		// 根据进给速度和倍率计算  换算为 r/ms  最大速度限制
		dCalcSpeed = pSSingleAxisPointer->cSpindleDir * min(pSSingleAxisPointer->dSpindleSpeed * pSSingleAxisPointer->dFeedRatio, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax) / DEGREE_OF_CIRCLE;
		break;
	case SPINDLE_G96ANDG0_MODE://恒线速度G0   //结合G50
	case SPINDLE_G96ANDG1_MODE://恒线速度G1   //结合G50

		//根据线速度和当前点X轴的坐标  计算得到当前点的主轴转速(r/min)
		dCalcSpeed = pSSingleAxisPointer->cSpindleDir * min(pSSingleAxisPointer->dFeedrate * 1000 / (PI * absX), min(pSSingleAxisPointer->dLimitedSpindleSpeed, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax) / DEGREE_OF_CIRCLE);
		break;
	case SPINDLE_COMMONSTOP_MODE://减速停
	case SPINDLE_PRECISESTOP_MODE://准停
	case SPINDLE_SINGLEAXIS_MODE://单轴模式
	case SPINDLE_HANDWHEEL_MODE://手轮模式
	default:
		dCalcSpeed = 0;
		break;
	}
	*pTargetSpeed = dCalcSpeed;
	return 1;
}


/*************************************************
  函数名称：getG97AbsSpindleSpeed(struct SSingleAxis* pSSingleAxisPointer)
  函数功能  返回主轴G97恒转速控制下主轴实际转速值
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
  输出参数：
  返 回 值：主轴G97理论转速(r/ms)(不带方向)
  备    注：
**************************************************/
double getG97AbsSpindleSpeed(struct _SSpindle* pSSpindlePointer)
{
	struct SSingleAxis* pSSingleAxisPointer;
	const uint8* ucMapArray;//映射
	uint8 ucAxis;//轴号
	if (pSSpindlePointer == NULL)
	{
		return 0.0;
	}
	//单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	if (pSSingleAxisPointer == NULL)
	{
		return 0.0;
	}

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return 0.0;
	}
	return min(pSSingleAxisPointer->dSpindleSpeed * pSSingleAxisPointer->dFeedRatio, g_SSystemPara.SAxisPara[ucMapArray[ucAxis]].dVmax) / DEGREE_OF_CIRCLE; // 根据进给速度和倍率计算  换算为 r/ms  最大速度限制
}




/*************************************************
  函数名称：readCommandAndSpindleSpeed(struct SSingleAxis* pSSingleAxisPointer)
  函数功能  从插补结构体中，读取主轴的转速和指令类型
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
  输出参数：
  返 回 值：-1 空指针
			1  执行成功
  备    注：
**************************************************/
int8 readCommandAndSpindleSpeed(struct SInterpolationPre* pSPrePointer, struct _SSpindle* pSSpindlePointer)
{
	double dCurrentPoint;//当前坐标
	struct SInterpolationPre* pSPreprocess;//预处理指针
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
	struct _SRun* pSRun;//运行结构体
	const uint8* ucMapArray;//映射
	uint8 ucAxis;

	if (pSSpindlePointer == NULL)
	{
		return -1;
	}
	//单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//读取轴号
	ucAxis = pSSingleAxisPointer->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//获取运行结构体
	pSRun = &pSSpindlePointer->pSChannel->SRun;
	//预处理结构体
	pSPreprocess = &pSSpindlePointer->pSChannel->SPreProcess;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空 返回
	{
		return -3;
	}

	if (pSSpindlePointer->dLimitedSpindleSpeed >= 0)//G50限制速度有效  2023.3.13
	{
		double dLimitedSpindleSpeed = pSSpindlePointer->dLimitedSpindleSpeed / (60.0 * 1000) * DEGREE_OF_CIRCLE;
		if (!isZero(pSSingleAxisPointer->dLimitedSpindleSpeed - dLimitedSpindleSpeed))
		{
			pSSingleAxisPointer->dLimitedSpindleSpeed = dLimitedSpindleSpeed;
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG0_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG1_MODE)
			{//处于G96G97的功能状态下
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		pSSpindlePointer->dLimitedSpindleSpeed = SPINDLE_INVALID_VAL;//置为无效
	}


	if (getBits(pSSingleAxisPointer->unCommandType, CMD_SPINDLE_SPEED_ADDR) != getBits(pSPreprocess->unCommandType, CMD_SPINDLE_SPEED_ADDR))
	{//G96  G97模式转换

		pSSingleAxisPointer->unCommandType = pSPreprocess->unCommandType;//指令类型继承
		if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG0_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG1_MODE)
		{//处于G96G97的功能状态下
			switch (getBits(pSSingleAxisPointer->unCommandType, CMD_SPINDLE_SPEED_ADDR))
			{
			case CMD_SPINDLE_SPEED_CV://恒转速
				if (pSSpindlePointer->dSpindleSpeed < 0)//无主轴转速   根据当前转速进行转换
				{
					pSSingleAxisPointer->dSpindleSpeed = pSSingleAxisPointer->dCurrentSpeed;//继承当前转速(r/min)
				}
				pSSingleAxisPointer->ucRunMode = SPINDLE_G97_MODE;//设置G97 功能状态
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态

				break;

			case CMD_SPINDLE_SPEED_CLV://恒线速
				//不改变线速度  继承之前G96的S值  如果有新转速值 则在下面流程读取
				pSSingleAxisPointer->ucRunMode = SPINDLE_G96ANDG1_MODE;//设置G96 功能状态
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
				break;
			}
		}
	}

	if (pSSpindlePointer->dSpindleSpeed >= 0)// 主轴转速有效2023.3.13
	{
		double dSpindleSpeed;
		dSpindleSpeed = pSSpindlePointer->dSpindleSpeed / (60.0 * 1000) * DEGREE_OF_CIRCLE;//主轴转速 r/min->度/ms
		if (!isZero(dSpindleSpeed - pSSingleAxisPointer->dSpindleSpeed))
		{
			pSSingleAxisPointer->dSpindleSpeed = dSpindleSpeed;
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE)
			{//处于G97的功能状态下
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		pSSpindlePointer->dSpindleSpeed = SPINDLE_INVALID_VAL;//已读取  置为无效

	}
	if (pSSpindlePointer->dSpindleLineSpeed >= 0)// 主轴线速度有效2023.3.13
	{
		double dSpindleLineSpeed;
		dSpindleLineSpeed = pSSpindlePointer->dSpindleLineSpeed / (60.0 * 1000);//赋值  主轴转速 m/ms
		if (!isZero(dSpindleLineSpeed - pSSingleAxisPointer->dFeedrate))
		{
			pSSingleAxisPointer->dFeedrate = dSpindleLineSpeed;
			if (pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG0_MODE || pSSingleAxisPointer->ucRunMode == SPINDLE_G96ANDG1_MODE)
			{//处于G96的功能状态下
				pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
			}
		}
		pSSpindlePointer->dSpindleLineSpeed = SPINDLE_INVALID_VAL;
	}



	if (pSSingleAxisPointer->dFeedRatio != pSRun->dSpindleSpeedRatio)
	{//主轴倍率改变
		pSSingleAxisPointer->dFeedRatio = pSRun->dSpindleSpeedRatio;//赋值  进给倍率

		if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE)//G96不受倍率影响
		{
			pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		}
	}

	return 1;
}


/*************************************************
  函数名称：setSpindleSpeed(struct SSingleAxis* pSSingleAxisPointer, double dSpindleSpeed, uint8 type)
  函数功能  从插补结构体中，读取主轴的转速和指令类型
  输入参数：*pSSingleAxisPointer		结构体struct SSingleAxis指针
			dSpindleSpeed			主轴转速G96(m/min) G97(r/min)
			type					CMD_SPINDLE_SPEED_CLV -> G96(m/min)   ; CMD_SPINDLE_SPEED_CV -> G97(r/min)
  输出参数：
  返 回 值：-1 空指针
			1  执行成功
  备    注：
**************************************************/
int8 setSpindleSpeed(struct _SSpindle* pSSpindlePointer, double dSpindleSpeed, uint16 type)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针

	if (pSSpindlePointer == NULL)//空指针
	{
		return -1;
	}
	//指向主轴单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;

	if (pSSingleAxisPointer == NULL)//单轴结构体为空
	{
		return -2;
	}
	if 	(pSSpindlePointer->pSChannel->pMapAxisArray[pSSingleAxisPointer->ucAxis] >= AXIS_PHYSICS_NUM_MAX)//物理轴为空
	{
		return -1;
	}
	if (dSpindleSpeed < 0)//无效的主轴转速
	{
		return -3;
	}

	if (type == CMD_SPINDLE_SPEED_CLV)//G96 恒线速度控制
	{
		pSSingleAxisPointer->dFeedrate = dSpindleSpeed / (60 * 1000.0);  //(m/min)->(m/ms)
	}
	else//G97 恒转速控制
	{
		pSSingleAxisPointer->dSpindleSpeed = dSpindleSpeed / (60 * 1000.0) * 360.0;   //(r/min)->(°/ms)
	}

	if (pSSingleAxisPointer->ucRunMode != SPINDLE_COMMONSTOP_MODE && pSSingleAxisPointer->ucRunMode != SPINDLE_PRECISESTOP_MODE
		&& pSSingleAxisPointer->ucRunMode != SINGLEAXIS_MODE)
	{//不处于停的功能状态
		pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION;//进入轴占用状态

	}

	return 1;

}



////使能主轴
//int8 setSpindleEnabled(struct _SSpindle* pSSpindlePointer, bool isEnable)
//{
//	if (pSSpindlePointer == NULL)//空指针
//	{
//		return -1;
//	}
//	unsigned char physicalAxis = pSSpindlePointer->pSChannel->pMapAxisArray[pSSpindlePointer->pSSpindleAxis->ucAxis];
//
//	if (isEnable == TRUE)
//	{
//		//pSSpindlePointer->toDisableAxis = FALSE;
//		if (*pAxisEnable[physicalAxis] != IO_SET)//不使能时 使能  并且读取编码器更新C坐标
//		{
//			double dRealTimePoint;
//			*pAxisEnable[physicalAxis] = IO_SET;// 对应的轴使能打开
//			dRealTimePoint = surplusDoubleToPositive(
//				(double)pSSpindlePointer->pSChannel->pSSpindleEncoder->lEncoderAbsValue/(double)g_SSystemPara.SAxisPara[pSSpindlePointer->pSChannel->pMapAxisArray[pSSpindlePointer->pSSpindleAxis->ucAxis]].usMmToPulse, 
//				DEGREE_OF_CIRCLE);
//			updateSingleAxisRealTimePoint(&dRealTimePoint, pSSpindlePointer->pSSpindleAxis->ucAxis, pSSpindlePointer->pSChannel->pMapAxisArray);//更新坐标
//			pSSpindlePointer->toDisableAxis = FALSE;
//		}
//
//	}
//	else// FALSE   关闭轴使能
//	{
//		switch (pSSpindlePointer->pSSpindleAxis->ucRunMode)
//		{
//		case SPINDLE_G97_MODE://恒转速
//		case SPINDLE_G96ANDG0_MODE://恒线速度G0
//		case SPINDLE_G96ANDG1_MODE://恒线速度G1
//			pSSpindlePointer->fPreciseStopDegree = g_SSystemPara.fSpindleStopDegree;//写入准停角度
//			spindlePreciseStop(pSSpindlePointer,NULL);// 主轴准停
//			//break;
//		case SPINDLE_COMMONSTOP_MODE://减速停
//		case SPINDLE_PRECISESTOP_MODE://准停
//			pSSpindlePointer->toDisableAxis = TRUE;
//			break;
//		case SPINDLE_SINGLEAXIS_MODE://单轴模式
//		case SPINDLE_HANDWHEEL_MODE://手轮模式  直接关闭
//			*pAxisEnable[physicalAxis] = IO_RESET; // 对应的轴使能关闭
//			break;
//		default:
//			break;
//		}
//	}
//	return 1;
//
//}


/*************************************************
  函数名称：releaseSpindleEnableControl
  函数功能：释放仅由使能准备临时持有的资源
  输入参数：pSSpindle - 主轴或动力刀控制结构
  输出参数：更新临时资源
  返 回 值：无
  备    注：苏向标 2026-10-14
****************************************/

static void releaseSpindleEnableControl(struct _SSpindle* pSSpindle)
{
    if (pSSpindle->ucEnableOccupation != 0)
    {
        axisManagerReleaseByStruct(pSSpindle->ucEnableOccupation,
            pSSpindle->pSChannel->pMapAxisArray, AXIS_NUM_MAX, &pSSpindle->ucEnableOccupation);
    }
}
/*************************************************
  函数名称：syncSpindleEnablePosition
  函数功能：使能完成后同步对应主轴编码器坐标
  输入参数：pSSpindle - 主轴或动力刀控制结构
  输出参数：更新对应轴坐标
  返 回 值：1 - 完成或无编码器；-1 - 编码器每圈计数为0
  备    注：苏向标 2026-10-14
****************************************/

static int8 syncSpindleEnablePosition(struct _SSpindle* pSSpindle)
{
    /* zxr: 坐标同步使用本主轴对应编码器的每圈计数。 */
#if STM32_MODE
    struct SEncoder* pSEncoder = pSSpindle == &pSSpindle->pSChannel->SSpindle
        ? pSSpindle->pSChannel->pSSpindleEncoder : pSSpindle->pSChannel->pSSecondSpindleEncoder; /**< 对应编码器。 */
    uint8 ucAxis = pSSpindle->pSSpindleAxis->ucAxis; /**< 逻辑轴。 */
    if (pSEncoder != NULL)
    {
        /* zxr: 就地校验分母，先取圈内整数余数再换算角度，兼容反转和大累计计数。 */
        uint16 usEncoderCirclePulseNum = pSEncoder->usEncoderCirclePulseNum;
        int64 lEncoderCirclePulse;
        double dPoint;
        if (usEncoderCirclePulseNum == 0)
        {
            parseRunningError(pSSpindle->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
            return -1;
        }
        lEncoderCirclePulse = pSEncoder->lEncoderAbsValue % (int64)usEncoderCirclePulseNum;
        if (lEncoderCirclePulse < 0)
        {
            lEncoderCirclePulse += usEncoderCirclePulseNum;
        }
        dPoint = (double)lEncoderCirclePulse * DEGREE_OF_CIRCLE / (double)usEncoderCirclePulseNum;
        updateSingleAxisRealTimePoint(&dPoint, ucAxis, pSSpindle->pSChannel->pMapAxisArray);
    }
#else
    (void)pSSpindle;
#endif
    return 1;
}
/*************************************************
  函数名称：advanceSpindleEnable
  函数功能：按目标推进独立使能计时，取消不依赖普通IO队列
  输入参数：pSSpindle - 主轴结构；ucPhysicalAxis - 已校验的物理轴；bEnable - 目标使能状态
  输出参数：更新控制状态
  返 回 值：1 - 完成；0 - 等待；负值 - 异常
  备    注：苏向标 2026-10-14
****************************************/

static int8 advanceSpindleEnable(struct _SSpindle* pSSpindle, uint8 ucPhysicalAxis, bool bEnable)
{
    u32 unRunTime; /**< 使用板端计时接口原生类型，避免不同类型指针互转。 */
    int8 cResult; /**< 本次计时推进结果。 */
    /* g_SSystemPara提供只读延时配置，g_ucAxisExactEnable提供共享软件就绪状态。 */
    if (bEnable)
    {
        pSSpindle->unDisableRunTime = 0;
        if (pSSpindle->unEnableRunTime == 0 && getSpindleEnableOutput(ucPhysicalAxis) == IO_SET
            && (getbit(g_ucAxisExactEnable, ucPhysicalAxis)))
        {
            releaseSpindleEnableControl(pSSpindle);
            return 1;
        }
        setSpindleEnableOutput(ucPhysicalAxis, IO_SET);
        /* 苏向标 2026-10-14：周期内按值适配u32计时类型，原状态仍保存回本主轴。 */
        unRunTime = pSSpindle->unEnableRunTime;
        cResult = delays_ms(g_SSystemPara.unSpindleEnableTime, &unRunTime);
        pSSpindle->unEnableRunTime = unRunTime;
        if (cResult != 1)
        {
            return 0;
        }
        /* zxr: 同步失败时清除软件就绪位，避免下一次调用跳过同步而误报成功。 */
        if (syncSpindleEnablePosition(pSSpindle) < 0)
        {
            setAxisEnableStatus(ucPhysicalAxis, 0);
            return -1;
        }
        setAxisEnableStatus(ucPhysicalAxis, 1);
    }
    else
    {
        pSSpindle->unEnableRunTime = 0;
        /* 苏向标 2026-10-14：保留新项目先关输出的顺序，延时结束后才交还占用。
         * 输出已关闭但计时非零时仍要继续等，不能下一拍就跳过取消使能延时。 */
        if (getSpindleEnableOutput(ucPhysicalAxis) != IO_RESET || pSSpindle->unDisableRunTime != 0)
        {
            setSpindleEnableOutput(ucPhysicalAxis, IO_RESET);
            unRunTime = pSSpindle->unDisableRunTime;
            cResult = delays_ms(g_SSystemPara.unSpindleDisableTime, &unRunTime);
            pSSpindle->unDisableRunTime = unRunTime;
            if (cResult != 1)
            {
                return 0;
            }
        }
        setAxisEnableStatus(ucPhysicalAxis, 0);
        pSSpindle->unDisableRunTime = 0;
    }
    releaseSpindleEnableControl(pSSpindle);
    return 1;
}
/*************************************************
  函数名称：setSpindleEnabled
  函数功能：在有效控制权下推进使能，主副轴各自计时
  输入参数：pSSpindlePointer - 主轴结构；isEnable - TRUE使能，FALSE取消
  输出参数：更新控制状态
  返 回 值：1 - 完成；0 - 等待；负值 - 异常
  备    注：苏向标 2026-10-14
****************************************/

int8 setSpindleEnabled(struct _SSpindle* pSSpindlePointer, bool isEnable)
{
    uint8 ucAxis; /**< 逻辑轴号。 */
    uint8 ucPhysicalAxis; /**< 物理轴号。 */
    int8 cResult = 1; /**< 控制权和使能处理结果。 */
    if (pSSpindlePointer == NULL || pSSpindlePointer->pSChannel == NULL
        || pSSpindlePointer->pSSpindleAxis == NULL || pSSpindlePointer->pSChannel->pMapAxisArray == NULL)
    {
        return -1;
    }
    ucAxis = pSSpindlePointer->pSSpindleAxis->ucAxis;
    if (ucAxis >= AXIS_NUM_MAX)
    {
        return -1;
    }
    ucPhysicalAxis = pSSpindlePointer->pSChannel->pMapAxisArray[ucAxis];
    if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
    {
        return -1;
    }
    if (isEnable && (isSpindleStopping(pSSpindlePointer)
        || (pSSpindlePointer->pSChannel->ucChannelStatus == SYS_RESET
            && !pSSpindlePointer->pSChannel->bIsStopSpindleAndCoolantwhileReset)))
    {
        return 0;
    }
    if (!(getbit(pSSpindlePointer->ucOccupation, ucAxis)))
    {
        cResult = reserveAxisControl((VAL_STRUCT_TYPE)(1U << ucAxis),
            pSSpindlePointer->pSChannel->pMapAxisArray, &pSSpindlePointer->ucEnableOccupation);
    }
    if (cResult == 1)
    {
        cResult = advanceSpindleEnable(pSSpindlePointer, ucPhysicalAxis, isEnable);
    }
    return cResult;
}


/*************************************************
  函数名称：IOSetSpindleEnabled(struct _SSpindle* pSSpindlePointer, bool isEnable)
  函数功能  IO设置主轴使能  （需要循环进入该函数）
  输入参数：pSSpindlePointer	主轴结构体指针
			isEnable			是否使能   TRUE -- 使能主轴    FALSE -- 取消使能
  输出参数：
  返 回 值：-1 空指针
			1  执行成功
			0  未成功
  备    注：
**************************************************/
int8 IOSetSpindleEnabled(struct _SSpindle* pSSpindlePointer, bool isEnable)
{
	if (pSSpindlePointer == NULL || pSSpindlePointer->pSChannel == NULL || pSSpindlePointer->pSSpindleAxis == NULL) return -1;
	//物理轴号
	uint8 ucPhysicalAxis = pSSpindlePointer->pSChannel->pMapAxisArray[pSSpindlePointer->pSSpindleAxis->ucAxis];
	if (isEnable == TRUE)
	{
		return setSpindleEnabled(pSSpindlePointer, isEnable);
	}
	else
	{
		switch (pSSpindlePointer->pSSpindleAxis->ucRunMode)
		{
		case SINGLEAXIS_MODE:			//单轴模式
		case SPINDLE_HANDWHEEL_MODE:	//手轮模式
		default:
			if (isZero(pSSpindlePointer->pSSpindleAxis->dCurrentSpeed))// 当前速度为零
			{//进入取消使能流程
				return setSpindleEnabled(pSSpindlePointer, isEnable);
			}
			break;
		case SPINDLE_G97_MODE:			//G97
		case SPINDLE_G96ANDG0_MODE:			//G96 G0
		case SPINDLE_G96ANDG1_MODE:			//G96 G1+
			return 1;  ////朱稳中 2023.5.19  M15 + M5  主轴停后仍使能
		case SPINDLE_COMMONSTOP_MODE:			//减速停
		case SPINDLE_PRECISESTOP_MODE:		//准停
			//主轴运动过程中   取消主轴使能  可以交付给主轴  然后删除IO指令
			pSSpindlePointer->toDisableAxis = TRUE;


			//spindleStopAPI(pSSpindlePointer, NULL);//主轴准停，利用系统参数
			//pSSpindlePointer->pSSpindleAxis->ucRunMode = SPINDLE_PRECISESTOP_MODE;
			return 1;//处理完成  删除IO
		}
	}
	return 0;
}

//总的主轴停接口  根据宏定义设置停止方式
int8 spindleStopAPI(struct _SSpindle* pSSpindlePointer, float* preciseStopDegree, bool bForcePreciseStop)
{
	const uint8* ucMapArray;//映射
	uint8 ucAxis;
	struct _SEncoderTest* pEncoderTest;
	
	if (pSSpindlePointer == NULL || pSSpindlePointer->pSChannel == NULL || pSSpindlePointer->pSSpindleAxis == NULL)
	{
		return -1;
	}

	//读取轴号
	ucAxis = pSSpindlePointer->pSSpindleAxis->ucAxis;
	//获取映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX) // 物理轴为空 返回
	{
		return -3;
	}

	if (preciseStopDegree != NULL)// 修正角度，若不修正表现为多圈停机或停机时间显著拉长 2026.3.14
    {
        *preciseStopDegree = surplusDoubleData(*preciseStopDegree, 360);
    }

	pEncoderTest = pSSpindlePointer->pSSpindleAxis->ucAxis == AXIS_C ? &g_SChannel[pSSpindlePointer->pSChannel->ucChannelNum].SSpindleTest : &g_SChannel[pSSpindlePointer->pSChannel->ucChannelNum].SSecondSpindleTest;
	// zxr: 强制准停不计算无关平均转速，反馈由准停完成门禁校验
	if (bForcePreciseStop == FALSE)
	{
		pEncoderTest->dSpindleAvgSpeed =
			(double)pEncoderTest->nTotalPulse / pEncoderTest->usPeriod / (pSSpindlePointer->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum);
	}

	// if (isZero(pEncoderTest->dSpindleAvgSpeed == 0)) //注释： 2026.3.28 原来是只要转速为0就走减速停，修改为准停有效就走准停
	// {
	// 	return spindleCommonStop(pSSpindlePointer);
	// }
	// 根据系统参数选择停止方式  //朱稳中 2023.5.19
	if (bForcePreciseStop == FALSE && (g_SSystemPara.ucSpindleStopMode == SPINDLE_STOP_MODE_COMMONSTOP_MODE|| isZero(pEncoderTest->dSpindleAvgSpeed))) //2026.3.28增加将转速为0且非准停，走减速停
	{ // 非强制准停  而且 系统参数为减速停
		return spindleCommonStop(pSSpindlePointer);
	}
	else
	{																	// 强制准停  或  系统参数为准停
		return spindlePreciseStop(pSSpindlePointer, preciseStopDegree); // 主轴准停，利用系统参数
	}
}

