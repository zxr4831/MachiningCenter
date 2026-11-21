#include "SecondSpindle.h"
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
#if STM32_MODE
#include "sys.h"
#endif // STM32_MODE


/* 苏向标 2026-10-14：副轴启动保留停止门控。 */
/*************************************************
  函数名称：SSecondSpindleBegin
  函数功能：请求启动副轴，停止收尾期间不覆盖停止状态
  输入参数：pSSpindlePointer - 副轴对象
  输出参数：更新允许启动时的模式及阶段
  返 回 值：1 - 已设置启动；0 - 暂不切换；负值 - 参数错误
  备    注：苏向标 2026-10-14；保留动力刀原启动路径
****************************************/
int8 SSecondSpindleBegin(struct _SSpindle* pSSpindlePointer)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
	const uint8* ucMapArray;//映射
	uint8 ucAxis;//轴号

	if (pSSpindlePointer == NULL || pSSpindlePointer->pSSpindleAxis == NULL || pSSpindlePointer->pSChannel == NULL)//空指针
	{
		return -1;
	}
    /* 苏向标 2026-10-14：停止收尾期间不重新启动动力刀。 */
    if (isSpindleStopping(pSSpindlePointer))
    {
        return 0;
    }
	//指向单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;  //映射
	ucAxis = pSSingleAxisPointer->ucAxis;//读取轴号

	if (pSSingleAxisPointer == NULL)//单轴结构体为空
	{
		return -2;
	}
	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)//未映射物理轴  洪冠鹏 2024/6/11
	{
		parseRunningError(pSSpindlePointer->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
		channelStopToIDLE(pSSpindlePointer->pSChannel);
		return -3;
	}
	switch (pSSingleAxisPointer->ucRunMode)// 根据当前的功能状态设置
	{
		case SPINDLE_G96ANDG0_MODE:
		case SPINDLE_G96ANDG1_MODE:
		break;
		
	case SPINDLE_G97_MODE:
		//已启动状态下，直接返回
		break;

	case SPINDLE_SINGLEAXIS_MODE:
	case SPINDLE_PRECISESTOP_MODE:
	case SPINDLE_COMMONSTOP_MODE:
		pSSingleAxisPointer->ucRunMode = SPINDLE_G97_MODE;//设置功能
		pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		break;
	}
	return 0;
}

/*************************************************
  函数名称：readSecondSpindleSpeed(struct SSingleAxis* pSSingleAxisPointer)
  函数功能  从插补结构体中，读取第二转轴/动力刀的转速
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
  输出参数：
  返 回 值：-1 空指针
			1  执行成功
  备    注：
**************************************************/
int8 readSecondSpindleSpeed(struct SInterpolationPre* pSPrePointer, struct _SSpindle* pSSpindlePointer)
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

	if (pSPrePointer->dSecondSpindleSpeed >= 0)// 主轴转速有效
	{
		pSSingleAxisPointer->dSpindleSpeed = pSPrePointer->dSecondSpindleSpeed / (60.0 * 1000) * DEGREE_OF_CIRCLE;//赋值  主轴转速 r/min->度/ms

		if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE)
		{//处于G96G97的功能状态下
			pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION;//进入轴占用状态
		}
		pSPrePointer->dSecondSpindleSpeed = SPINDLE_INVALID_VAL;//已读取  置为无效

	}

	return 1;
}


/* 苏向标 2026-10-14：副轴周期复用停止分类与收尾。 */
/*************************************************
  函数名称：SecondspindleMotion
  函数功能：推进副轴速度及坐标，优先处理停止请求
  输入参数：pSSpindlePointer - 有效通道所属副轴对象
  输出参数：更新副轴运动状态、周期脉冲与坐标
  返 回 值：1 - 本周期处理结束；0 - 等待占用；负值 - 参数或停止异常
  备    注：苏向标 2026-10-14；复用公共停止入口，保留坐标回写
****************************************/
int8 SecondspindleMotion(struct _SSpindle* pSSpindlePointer)
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
    /* 苏向标 2026-10-14：副轴与主轴共享周期停止入口。 */
    processSpindleRequests(pSSpindlePointer);
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
		return -3;
	}
    /* 2026-10-24：报警禁止普通运动，但已接收的停止仍须减速并完成取消使能。 */
	//2025.9.22 熊骏
	if (!isSpindleStopping(pSSpindlePointer) && (
#if STM32_MODE
		Check_bootwarning() == 0 || //正在报警则退出函数，企业管理的报警主轴报警、伺服报警、超时报警、限位报警
#endif
		isErrorHasBeenWritten() == TRUE))//正在报警则退出函数，学校管理的报警  2024/10/16
	{
		return -2;
	}
	if (pSPreprocess->dSecondSpindleSpeed >= 0)// 主轴转速有效
	{

		//恒转速
		pSSingleAxisPointer->dSpindleSpeed = pSPreprocess->dSecondSpindleSpeed / (60.0 * 1000) * DEGREE_OF_CIRCLE;//赋值  主轴转速 r/ms->度/ms

		if (pSSingleAxisPointer->ucRunMode == SPINDLE_G97_MODE)
		{//处于G96G97的功能状态下
			pSSingleAxisPointer->ucInpStatus = INP_OCCUPATION; //进入轴占用状态
		}
		pSPreprocess->dSecondSpindleSpeed = SPINDLE_INVALID_VAL;//已读取  置为无效
	}

	//倍率改变？

    /* 苏向标 2026-10-14：动力刀复用同一公共停止入口。 */
    if (isSpindleStopping(pSSpindlePointer))
    {
        if (processSpindleStop(pSSpindlePointer) < 0)
        {
            return -1;
        }
    }
    else
    {
        /* 轴占用判断	//陈敏聪	2023/12/19 */
        if (pSSpindlePointer->pSSpindleAxis->ucInpStatus == INP_OCCUPATION)
        {
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

        switch (pSSingleAxisPointer->ucRunMode)//根据Mode 执行不同的功能
        {

        case SPINDLE_G97_MODE: //G97恒转速
            constVelMotion(pSSpindlePointer);
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
  函数名称：getSecondSpindleSpeedReached(struct SSingleAxis* pSSingleAxisPointer)
  函数功能：第二转轴无编码器，通过理论当前速度和目标速度的比值粗略的判断第二转轴转速是否达到要求
  输入参数：*pSSingleAxisPointer，结构体struct SSingleAxis指针
  输出参数：
  返 回 值：TRUE

  备    注：
**************************************************/
int8 getSecondSpindleSpeedReached(struct _SSpindle* pSSpindlePointer)
{
	if (pSSpindlePointer->pSSpindleAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE
		|| pSSpindlePointer->pSSpindleAxis->ucRunMode == SPINDLE_COMMONSTOP_MODE || pSSpindlePointer->pSSpindleAxis->ucRunMode == SPINDLE_PRECISESTOP_MODE
		|| pSSpindlePointer->pSSpindleAxis->ucInpStatus == INP_FINISHED)//不进行主轴转速检测的状态 //2024/7/6 应企业要求，增加主轴停止时不检测以及转速未发生改变时不检测 洪冠鹏
	{
		return TRUE;
	}
	else
	{
		double dTargetSpeed = 0;
		if(calTargetSpindleSpeed(pSSpindlePointer,&dTargetSpeed)!= 1)return FALSE;
		if (isZero(dTargetSpeed))
		{
			if (isZero(pSSpindlePointer->pSSpindleAxis->dCurrentSpeed) || isZero(g_SSystemPara.fSpindleSpeedDeviation - 1.0))//编码器相对值为0  或者允许误差为100%
			{
				return TRUE;
			}
		}
		else
		{
			double dSpeedPercentage;
			if (pSSpindlePointer->pSSpindleAxis->cSpindleDir == ARC_CW)//正转
			{
				dSpeedPercentage = (pSSpindlePointer->pSSpindleAxis->dCurrentSpeed / DEGREE_OF_CIRCLE) / dTargetSpeed;
			}
			else//反转
			{
				dSpeedPercentage = -1.0 * (pSSpindlePointer->pSSpindleAxis->dCurrentSpeed / DEGREE_OF_CIRCLE) / dTargetSpeed;
			}
			if ((fabs(dSpeedPercentage - 1.0 * SGN(dTargetSpeed)) <= g_SSystemPara.fSpindleSpeedDeviation)  //误差范围内内且方向相同
				&& (pSSpindlePointer->pSSpindleAxis->ucInpStatus == INP_FINISHED || getBits(pSSpindlePointer->pSSpindleAxis->unCommandType, CMD_SPINDLE_SPEED_ADDR) == CMD_SPINDLE_SPEED_CV))//主轴插补完成 或为恒转速
			{ //G96一定要加速完（status = FINISHED)才能启动G1+

				return TRUE;
			}
		}
	}
	return FALSE;
}

/*************************************************
  函数名称：spindleSpeedTest(struct SSpindleTest* pSpindleTest)
  函数功能  主轴实际转速检测，根据编码器取值，计算主轴的平均转速  计算得到的主轴转速保存在   pSpindleTest->dSpindleAvgSpeed
  输入参数：*pSpindleTest，主轴检测结构体指针
  输出参数：
  返 回 值：-1  空指针
			1	完成
  备    注：
**************************************************/
int8 SecondspindleSpeedTest(struct _SEncoderTest* pEncoderTest, struct _SSpindle* pSSpindlePointer)
{
	struct SSingleAxis* pSSingleAxisPointer;//单轴结构体指针
	const uint8* ucMapArray;//映射
	uint8 ucAxis;//轴号
	struct SChannel* pSChannel;

	if (pSSpindlePointer == NULL || pEncoderTest == NULL)//空指针
	{
		return -1;
	}
	//指向动力刀单轴结构体
	pSSingleAxisPointer = pSSpindlePointer->pSSpindleAxis;
	//轴号
	ucAxis = pSSpindlePointer->pSSpindleAxis->ucAxis;
	//映射
	ucMapArray = pSSpindlePointer->pSChannel->pMapAxisArray;
	//通道
	pSChannel = pSSpindlePointer->pSChannel;

	if (ucMapArray[ucAxis] >= AXIS_PHYSICS_NUM_MAX)
	{
		return -2;
	}
#if SECONDSPINDLE_NOENCODER
	pEncoderTest->ucStep = 2;
#endif

	switch (pEncoderTest->ucStep)
	{
	case 0:
		pEncoderTest->usPeriod = 0;//初始化周期
		pEncoderTest->nTotalPulse = 0;//初始化脉冲
		if (pSChannel->bCalculateSecondSpindleSpeedFlag && pSSingleAxisPointer->ucInpStatus == INP_FINISHED)	//陈敏聪	2023/6/22
		{//检测主轴转速的条件
			pEncoderTest->ucStep = 2;
		}
		else
		{
			pEncoderTest->ucStep = 1;
		}
	case 1:
	case 2:
#if !SECONDSPINDLE_USE_NOENCODER	//第二转轴使用编码器	20个周期才采样完
		pEncoderTest->usPeriod++;//周期++
		pEncoderTest->nTotalPulse += (int)pSChannel->pSSpindleEncoder->cEncoderDir * (int)pSChannel->pSSpindleEncoder->usDelta;//脉冲数叠加
		if (pEncoderTest->usPeriod >= g_SSystemPara.usSpindleAvgPeriod)//计数达到目标计数
		{
			//计算主轴平均转速  (r/ms)
			pEncoderTest->dSpindleAvgSpeed = (double)pEncoderTest->nTotalPulse / pEncoderTest->usPeriod / (pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum);


			if (pEncoderTest->ucStep == 2)//G33 G32需要检测主轴转速
			{
				pSChannel->dSpindleRealSpeed = pEncoderTest->dSpindleAvgSpeed;//赋值到实际主轴转速
				pSChannel->bCalculateSecondSpindleSpeedFlag = FALSE;//标志位置回原来的值
			}
			pEncoderTest->ucStep = 0;//初始化
		}
#else	//第二转轴不使用编码器 每一周期都采样
		if (pEncoderTest->ucStep == 2)//G33 G32需要检测主轴转速
		{
			pEncoderTest->nTotalPulse = (int)pSChannel->pSSpindleEncoder->cEncoderDir * (int)pSChannel->pSSpindleEncoder->usDelta;
			pSChannel->dSecondSpindleAnalogSpeed = pEncoderTest->dSpindleAvgSpeed = (double)pEncoderTest->nTotalPulse / (pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum);
			pSChannel->bCalculateSecondSpindleSpeedFlag = FALSE;	//陈敏聪	2023/6/22
			pEncoderTest->nTotalPulse = 0;
		}
		pEncoderTest->ucStep = 0;
#endif
		break;
	default:
		pEncoderTest->ucStep = 0;
	}

	return 1;
}
