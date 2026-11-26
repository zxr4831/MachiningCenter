/************************************************************
  Copyright (C), 广东工业大学. 2021
  文件名：G33.c
  作者：郭强    版本：V22.0    创建日期：2023/06/08
  模块描述：G33Z向攻牙循环
  历史修改记录：<author><time><version ><desc>
	**    2021/09/16     V1.0
	**    2021/10/14     V2.0            1.更改Z轴启动判断函数：不用while
										2.加入Z轴浮动量g_dZAxisFloatingLength仿真计算
											在起刀点和工件端面过程中不产生浮动，但G代码中没有该段长度，
											这里设定长度为10mm.

	**    2021/11/04     V3.0            1.更改G33攻丝时为随动状态
										2.Z轴启动时对启动偏差进行补偿
										3.默认G33加工第一头时Q值为0（原数控手册中G33无Q值）

	**    2021/11/15     V4.0			   1.原本多头螺纹启动时编码器相对值为整形，这里更改calcZAxisStartEncoderValue()函数
											使得理论值为double类型
										2.根据主轴编码器的值获取主轴转速，根据在理想Q值就随动运行反算处启动时的理论编码器值
										（主轴测速在1ms中断服务函数中依靠全局变量g_bCalculateSpindleSpeedFlag标志位决定是否测速）

	**    2021/11/18     V5.0			   1.针对IO存储发生改变做出调整#include "IOCommand.h"
										2.将攻丝完主轴减速停和z轴随动退刀阶段合并为一个运动阶段
										3.对G33暂停恢复加工代码修改，有回写
										4.更改G33预处理位置
										（加速阶段主轴误差未改动）

	**    2021/11/24     V6.0			   1.对于新方案的尝试性修改

	**    2021/12/19     V7.0		       1.将G33正向攻丝阶段主轴转向信息保存在G33结构体中，不再保存在g_SPreProcess.cArcDir中

	**    2022/01/04     V8.0			   1.主轴反转补偿改为普通规划（新增函数用于补偿时不能保证速度的连续性为保证速度的连续性）

	**    2022/01/11     V9.0			   1.系统参数加入bG33RecalDecPointFlag-----G33重算减速点标志位，当开启该标志位时，
											要根据主轴转速和导程估算主轴减速阶段Z轴继续攻丝长度，为保证不超出理论终点，重算理论终点坐标
											理论终点坐标为主轴减速标志点

	**    2022/01/14     V10.0		   1.针对主轴规划（多种曲线组合）改变作出先应调整

	**    2022/03/02     V11.0		   1.G33主轴加速完成和减速停完成判断位置进行修改，将判断放入G33内部程序，不再放入主轴函数内部
										2.修复G33同一行有M30时暂停的BUG

	**    2022/03/16     V12.0		   1.针对不再使用 SInterpolationPre类型结构体g_SPreProcess做插补进行调整
											G33结构体内增加SPlan结构体用于速度规划，G33指令类型起终点向量等保存在新结构体 g_SPreProcess的SUserData结构体成员中

	**    2022/08/06     V13.0		   1.针对企业提出的G33大方向要根据编码器正反走正反进行修改
										（原方案和G32一样只看编码器绝对变化量，默认全程只换向一次
											企业观点：在主轴从正到反时可能不止反向一次，要看编码器反馈的方向）
										2.增加G33自己的最大速度参数，以及G33运行时编码器反馈主轴超速标志变量供系统使用
										3.反向补偿只补偿一次，主轴设置反向后第一次反向。其他情况的反向不补偿。
										4.异常反向处理：
											G33结构体中要时刻记录上一周期的方向来看规划的向量起终点是否要改变然后重新规划
											4.1异常反向不能超过G33的起点
											4.2
											①Z轴随动前加速阶段异常反向：
											虚拟运行阶段反向，重新开始虚拟运行阶段，不动
											加速运行阶段反向，直接进入随动攻牙阶段，不能超出起点，到达起点直接转为随动前加速初始阶段

											②Z轴随动攻丝阶段异常反向：随动攻丝改为识别编码器方向，与上一周期反向时要重新规划反向起终点
											当反向要超过起点时只走到起点然后直接进入主轴减速停阶段

											③Z轴随动减速和退刀阶段异常反向（难点）
											此运动正常包含几个阶段：
											③.1主轴减速并在反向前随动
											③.2第一次反向的反向补偿阶段，此时累积记录编码器的变化量对应的位移
											③.3补偿完将累计位移和上一周期新增位移累加开始追赶追赶完成后随动
											③.4当进入避空距离内时减速至起点

										5.企业提出最多两个轴的斜攻牙（反向补偿方向是攻牙方向，参数不再是之前说的每个轴都有一个对应的补偿距离参数）


**    2022/08/12     V14.0		   bug修复工作：
			 1.JOG模式下，按正转超过G33终点后停下继续按正转

	++++++++++++++++以上为单通道版本++++++++++++++
**    2022/08/12     V15.0		        1.多通道版本
**    2022/10/24     V16.0		        1.加入导程修正百分比系统参数(范围是101%-99%)：
										g_SSystemPara.dPercentageOfG33EnterLeadCorrection = 1.0;      //G33攻牙攻入导程修正百分比（99%-101%）
										g_SSystemPara.dPercentageOfG33ReturnLeadCorrection = 1.0;	  //G33攻牙退刀导程修正百分比（99%-101%)

**    2023/03/23     V17.0		        1.增加退刀主轴转速倍率功能，G33结构体新增dG33SpindleSpeed（r/min）用于退刀后主轴减速停 还原最初主轴速度


**    2023/05/03     V18.0(陈敏聪)	    1.增加G84，G88动力刀攻牙功能

**    2023/05/11     V19.0       	    1.增加主轴反转前暂停功能

**	  2023/05/15     V20.0(陈敏聪)      1.修正G84暂停前减速停和暂停后主轴反转的调用API接口，调用API接口增加A轴

**    2023/05/27     V21.0       	    1.将主轴反转倍率参数改为主轴反转转速百分比
 
**    2023/06/08     V22.0       	    1.修复更新程序时错漏，主轴反转前暂停过程Z轴随动
*************************************************************/

#include "G33.h"
#include "G32.h"
#include "IOCommand.h"
#include "GeneralFunctions.h"
#include "Motion.h"
#include "CTrajPlanning.h"
#include "lookAhead.h"
#include "Spindle.h"
#include "SecondSpindle.h"
#include "motionAPI.h"
#include "ringbuff.h"
#include "G0.h"
#include "G1.h"
#include "ToolCompensation.h"
#include "Channel.h"
#include "ErrorCompensation.h"

//G33保存数据用文件指针
FILE* fpG33Write;

//测试仿真用编码器数值全局变量
int64 g_lSpindleEncoderValue;

//测试仿真用Z轴浮动量
double g_dZAxisFloatingLength;

double g_dActualSpindleSpeed;

/*************************************************
  函数名称：InitG33GlobalVariable(struct SChannel* pSChannel)
  函数功能  初始化G33全局变量
  输入参数：pSChannel  :通道指针
  输出参数：
  返 回 值：
  备    注：(报警信号初始化)
**************************************************/
bool InitG33GlobalVariable(struct SChannel* pSChannel)
{
    if (pSChannel == NULL)
    {
        return FALSE;
    }
    pSChannel->bG33SpindleOverSpeedWarningSigns = FALSE;
    // zxr: 开机或新程序初始化时清零本通道随动无反馈计数
    pSChannel->SG33.unFollowTimeOut = 0;
    return TRUE;
}

int8 G33initStructByTool(struct SToolCompensation* pSToolCompPointer, struct SInterpolationPre* pSPrePointer)
{
    struct G33* pSG33;
    if (pSToolCompPointer == NULL || pSPrePointer == NULL)
    {
        return -1;
    }
    pSG33 = &pSPrePointer->pSChannel->SG33;

    pSG33->dF = pSToolCompPointer->dF;
    pSG33->ucL = pSToolCompPointer->ucL;

    pSG33->G33RotateAxis = pSToolCompPointer->G33RotateAxis;
    pSG33->G33TappingMode = pSToolCompPointer->G33TappingMode;

    chooseInterpPlaneForG33(pSG33, pSPrePointer);

    pSG33->ucValStatus = pSPrePointer->SUserData.ucValStatus;
    for (int i = 0; i < 2; i++)
    {
        setbit(pSG33->ucValStatus, pSG33->ucAxis[i]);
    }
    pSG33->ucRelaVal = pSPrePointer->SUserData.ucRelaVal; //相对值标识变量

    pSG33->SStartPoint = pSPrePointer->SUserData.SStartPoint;
    pSG33->SEndPoint = pSPrePointer->SUserData.SEndPoint;
    pSG33->SStartVector = pSPrePointer->SUserData.SStartVector;

    //此时是否初始化待定
    pSG33->G33StartPoint = pSPrePointer->SUserData.SStartPoint;
    pSG33->G33EndPoint = pSPrePointer->SUserData.SEndPoint;
    pSG33->G33StartVector =
        pSPrePointer->SUserData.SStartVector; //记录正向攻丝方向向量，用于G33反向补偿时，初始化插补结构体

    return 1;
}

/*************************************************
  函数名称：int8 chooseInterpPlaneForG32(struct G33* pG33)
  函数功能：选择G33插补平面
  输入参数：*pG33，   结构体struct G33指针

  返 回 值：无
  备    注：无
**************************************************/
int8 chooseInterpPlaneForG33(struct G33* pG33, struct SInterpolationPre* pSPrePointer)
{
    if (pG33 == NULL || pSPrePointer == NULL)
        return -1;
    switch (getBits(pSPrePointer->unCommandType, CMD_PLANE_ADDR))
    {
    case CMD_PLANE_G18:
    default:
        pG33->ucAxis[0] = AXIS_Z;
        pG33->ucAxis[1] = AXIS_X;
        break;
    case CMD_PLANE_G17:
        pG33->ucAxis[0] = AXIS_X;
        pG33->ucAxis[1] = AXIS_Y;
        break;
    case CMD_PLANE_G19:
        pG33->ucAxis[0] = AXIS_Y;
        pG33->ucAxis[1] = AXIS_Z;
        break;
    }
    return 1;
}

/*************************************************
  函数名称：checkSameVector(struct G33* pSG33, SVector SVector1, SVector Svector2)
  函数功能  检测向量同向
  输入参数：
  输出参数：
  返 回 值：FALSE : 向量不同
		   TRUE  ：向量相同
  备    注：(目前对比的是向量分量的符号，并非看向量的)
**************************************************/
bool checkSameVector(struct G33* pSG33, SVector SVector1, SVector Svector2)
{
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];
    //if ((sgn(SVector1.dIX) != sgn(Svector2.dIX)) || (sgn(SVector1.dIY) != sgn(Svector2.dIY)) || (sgn(SVector1.dIZ) != sgn(Svector2.dIZ)))
    //{
    //	return FALSE;
    //}
    if (!isZero(pSG33->G33EndPoint.dArray[ucG33Axis1] - pSG33->G33StartPoint.dArray[ucG33Axis1]) &&
        (sgn(SVector1.dArray[ucG33Axis1]) == sgn(Svector2.dArray[ucG33Axis1])))
    {
        return TRUE;
    }
    if (!isZero(pSG33->G33EndPoint.dArray[ucG33Axis2] - pSG33->G33StartPoint.dArray[ucG33Axis2]) &&
        (sgn(SVector1.dArray[ucG33Axis2]) == sgn(Svector2.dArray[ucG33Axis2])))
    {
        return TRUE;
    }

    return FALSE;
}

/*************************************************
  函数名称：getSpindleEncoderValue(struct SEncoder* pSpindleEncoder)
  函数功能  获取编码器数值
  输入参数：
  输出参数：
  返 回 值：根据宏定义返回模拟编码器值或真实编码器值
  备    注：
**************************************************/
int64 getSpindleEncoderValue(struct SEncoder* pSpindleEncoder)
{
    if (pSpindleEncoder == NULL)
    {
        return 0;
    }
    return pSpindleEncoder->lEncoderAbsValue;
}

/*************************************************
  函数名称：AnalogSpindleEncoderCount(int16 sSpindlePulseValue)
  函数功能  模拟主轴编码器计数
  输入参数：sSpindlePulseValue  单个周期对主轴发出的脉冲（有符号）
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void AnalogSpindleEncoderCount(struct G33* pSG33, int16 sSpindlePulseValue, struct SEncoder* pSpindleEncoder)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray;                                    //通道轴映射数组
    int64 lCAxisMmToPulse = (int64)g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].usMmToPulse; //主轴一转脉冲数(uint16)

    int64 lSpindleRevsToEncoderCounterDelta =
        (int64)pSpindleEncoder->usEncoderCirclePulseNum; //主轴转一转编码器计数器增量(uint32)

    g_lSpindleEncoderValue += (int64)sSpindlePulseValue * lSpindleRevsToEncoderCounterDelta / lCAxisMmToPulse;

#if !G33_USE_REAL_ENCODER
    //仿真编码器绝对数值
    pSpindleEncoder->lEncoderAbsValue = g_lSpindleEncoderValue;

    //实际应用中使用编码器方向和编码器前后两次绝对差值对"编码器增量进行计算"
    //G33当编码器差值方向与攻丝方向一致时为正向攻丝阶段，丝锥正向"起点"--》》"终点"
    //G33当编码器差值方向与攻丝方向不一致时为退刀阶段，丝锥--》》"起刀点运动"
    //仿真编码器差值，绝对值
    pSpindleEncoder->usDelta = abs(sSpindlePulseValue * lSpindleRevsToEncoderCounterDelta / lCAxisMmToPulse);

    //模拟编码器方向
    if (sSpindlePulseValue > 0)
    {
        pSpindleEncoder->cEncoderDir = 1; //正向
    }
    //else
    //{
    //	g_SEncoder[SPINDLE_ENCODER - 1].cEncoderDir = -1;//反向
    //}
    else if (sSpindlePulseValue < 0)
    {
        pSpindleEncoder->cEncoderDir = -1; //反向
    }
#endif
}

/*************************************************
  函数名称：calcZAxisFloatingLength(int32 nZAxisDeltaPulse , int32 nCAxisDeltaPulse)
  函数功能  仿真计算Z轴浮动量
  输入参数：nZAxisDeltaPulse  Z轴脉冲（整形）
			nCAxisDeltaPulse  主轴脉冲（整形）
  输出参数：
  返 回 值：TRUE:  进行浮动计算
			FALSE  不进行浮动计算
  备    注：不产生浮动的阶段如下：起点距当前点小于10mm时，默认空走段长10mm
			传入脉冲为为绝对值
**************************************************/
bool calcZAxisFloatingLength(struct G33* pSG33, int32 nZAxisDeltaPulse, int32 nCAxisDeltaPulse)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    int32 nCPulse = (abs)(nCAxisDeltaPulse);

    int32 nZPulse = (abs)(nZAxisDeltaPulse);

    double dA, dB;

    //单个周期的浮动量
    double dFloatingLength;

    //主轴单个周期走转数
    double dCAxisDeltaR;

    //Z轴单个周期位移
    double dZAxisDeltaL;

    //只有丝锥和工件接触时才会因为速度不成比例产生浮动，其他阶段返回
    if (pSG33->G33StartPoint.dArray[AXIS_Z] - g_dRealTimePointArray[AXIS_Z] < 1)
    {
        g_dZAxisFloatingLength = 0; //此时浮动量为零
        return FALSE;
    }
    dA = g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dPulseToMm;
    dB = g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dPulseToMm;
    //主轴单个周期走转数
    dCAxisDeltaR = (double)(nCPulse * g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dPulseToMm / 360.0);

    //Z轴单个周期位移
    dZAxisDeltaL = (double)(nZPulse * g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dPulseToMm);

    dFloatingLength = dCAxisDeltaR * pSG33->dF - dZAxisDeltaL;

    //通过比较C轴当前脉冲正负与攻丝过程中方向符号是否对应判定是退刀过程还是攻丝过程
    //当g_SPreProcessAxisC.dSpindleSpeed = 0时，为退刀完成后主轴减速，此过程不会产生浮动，在上方已经return了
    //主轴攻丝过程
    if (sgn((double)nCAxisDeltaPulse) == sgn((double)pSG33->cTappingSpindleDir))
    {
        g_dZAxisFloatingLength += dFloatingLength;
    }
    else //主轴退刀过程
    {
        g_dZAxisFloatingLength -= dFloatingLength;
    }

    return TRUE;
}

/*************************************************
  函数名称：getSpindleRealSpeed()
  函数功能  获取主轴实际转速
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
double getSpindleRealSpeed(struct G33* pSG33)
{
    double dSpindleRealSpeed = 0;
    if (pSG33->G33RotateAxis == AXIS_A)
    {
        pSG33->pSChannel->dSecondSpindleAnalogSpeed; //陈敏聪 2023/6/22
    }
    else
    {
        pSG33->pSChannel->dSpindleRealSpeed;
    }
    return dSpindleRealSpeed;
}

/*************************************************
  函数名称：calcZAxisStartEncoderRelValue(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  计算多头螺纹Z轴启动时编码器相对值
  输入参数：pSG33
			pSpindleEncoder
  输出参数：
  返 回 值：输出Z轴理论上启动时编码器相对值
  备    注：
			默认第一头螺纹启动编码器相对值为0，计算达到在此值启动就随动的效果的真正启动值
			返回值为double，虽然获取的编码器值nEncoderAbsValue为整形，但理论启动值可以为小数
**************************************************/
double calcZAxisStartEncoderRelValue(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    double dSpindleRevsToEDelta = (double)pSpindleEncoder->usEncoderCirclePulseNum; //主轴转一转编码器计数器增量

    //double dSpindleV = getSpindleRealSpeed();//主轴转速

    uint8 ucAlreadyL = pSG33->ucAlreadyL; //已加工头数

    uint8 ucL = pSG33->ucL; //要加工头数

    //启动即随动的编码器相对值
    double dZAxisStartERValue = (double)(ucAlreadyL * dSpindleRevsToEDelta / ucL);

    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return 0;
    }

#if G33ProcLog_DEBUG
    printf("第%u头螺纹加工时编码器值应为：%lf\n", ucAlreadyL + 1, dZAxisStartERValue);
#endif
    return dZAxisStartERValue;
}

/*************************************************
  函数名称：G33SpindleOverSpeedCheck(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  G33主轴超速检查
  输入参数：pSG33 gG33结构体指针
		   pSpindleEncoder  主轴编码器结构体指针
  输出参数：
  返 回 值： -1 ：指针为空
			1 超速
			2未超速

  备    注：此函数必须在G33结构体初始化后调用（要用到导程等信息）
			G33内部底层调用
**************************************************/
int8 G33SpindleOverSpeedCheck(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    double dZAxisNewAddDist;
    double dV;

    dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
    dV = dZAxisNewAddDist / pSG33->SPlanData.dTc;

    if (dV > g_SSystemPara.dG33MaxVel) //G33超速报警检测
    {
        pSG33->pSChannel->bG33SpindleOverSpeedWarningSigns = TRUE;
        return 1;
    }

    return 2;
}

/*************************************************
  函数名称：CheckG33SpindleOverSpeed()
  函数功能  G33主轴超速检查（外部调用）
  输入参数：
  输出参数：
  返 回 值：-1指针为空
			1 超速
			0 未超速

  备    注：应用层调用
**************************************************/
int8 CheckG33SpindleOverSpeed(struct SChannel* pSChannel)
{
    if (pSChannel == NULL)
    {
        return -1;
    }
    if (pSChannel->bG33SpindleOverSpeedWarningSigns)
    {
        return 1;
    }
    return 0;
}

/*************************************************
  zxr: 同步初始化接口及输入参数说明
  函数名称：initSG33Data(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  1.初始化G33结构体数据
			2.开启主轴测速
			3.记录G33运行时的主轴转向信息到cTappingSpindleDir
  输入参数：pInterpPrePointer 总插补预处理结构体指针
			pSSpindle 主轴结构体指针
			pSG33 G33结构体指针
			pSpindleEncoder 主轴编码器结构体指针
  输出参数：
  返 回 值：-1  指针为空，初始化失败
			-2  暂停恢复加工
			-3  G33的G代码中起点终点相同（长度为零），直接结束
			 0  初始化G33时发现主轴还未加减速完成（此时不测主轴转速，不进行G33运动）
			 1  初始化成功

  备    注：当刚进入G33时，在Z轴速度规划前要测主轴速度，初始化时要将测速标志位改为TRUE
**************************************************/
int8 initSG33Data(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33,
                  struct SEncoder* pSpindleEncoder)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组

    if (pInterpPrePointer == NULL || pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    // zxr: 新指令或暂停恢复重新初始化时，不沿用上次随动无反馈计数
    pSG33->unFollowTimeOut = 0;

    if (isZero(pInterpPrePointer->SUserData.dDistance)) //G33的G代码中起点终点相同（长度为零），直接结束
    {
        pSG33->ucAlreadyL = 0;                         //防止最初进入G33时被误认为暂停恢复加工
        pInterpPrePointer->ucInpStatus = INP_FINISHED; //改变总插补预处理结构体状态为完成
        return -3;
    }

    //当主轴未加减速完成时（前后作用延时时间太短），不进行G33运动
    //pG32->pSChannel->SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED
    if (pSSpindle->pSSpindleAxis->ucInpStatus != INP_FINISHED)
    {
#if G33ProcLog_DEBUG
        printf(
            "**********************gG33结构体初始化时，主轴加减速未完成，不进行G33运动！****************************\n");
#endif
        return 0;
    }

    //多通道坐标修正
    //判断相对坐标  修正终点
    if (pSG33->ucRelaVal != VAL_ABS_POINT) //不完全是绝对坐标
    {
        modifyRelaEndPoint((double*)&pSG33->SStartPoint, (double*)&pSG33->SEndPoint, ucMapArray, AXIS_NUM_MAX,
                           &pSG33->ucRelaVal);
    }
    //获取当前坐标作为起点
    getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

    //计算距离和向量
    calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                            &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                            pSG33->ucRelaVal);

    //重新赋值G33原始起终点及向量
    pSG33->G33StartPoint = pSG33->SStartPoint;
    pSG33->G33EndPoint = pSG33->SEndPoint;
    pSG33->G33StartVector = pSG33->SStartVector;

    //2022.3.16(插补结构体调整后)
    //对当前速度，SPlanData结构体相关变量初始化
    pSG33->dCurrentSpeed = 0;                                //当前速度置零
    pSG33->SPlanData.dTc = g_SSystemPara.dInterpolationTime; //速度规划结构体插补周期初始化
    pSG33->SPlanData.dCurLeftDist = 0;
    pSG33->SPlanData.dTargetDist = 0;
    pSG33->SPlanData.dPreDist = 0;

    if (pSG33->ucAlreadyL > 0)
    {
#if G33ProcLog_DEBUG
        printf("**********************暂停恢复加工，gG33结构体初始化！****************************\n");
#endif
        //获取实时点，用于加速阶段计算加速度虚拟运行阶段真实实时点的还原
        getRealTimePoint((double*)&pSG33->G33StartRealTimePoint, AXIS_NUM_MAX, pSG33->pSChannel->pMapAxisArray,
                         &pSG33->ucValStatus);
        getRealTimePoint((double*)&pSG33->G33AnalogRealTimePoint, AXIS_NUM_MAX, pSG33->pSChannel->pMapAxisArray,
                         &pSG33->ucValStatus);
        return -2;
    }

#if G33ProcLog_DEBUG
    printf("**********************gG33结构体初始化开始！****************************\n");
#endif

    //1.gG33结构体初始化
    //初次攻牙进行初始化
    //pSG33->dF   	pSG33->ucL      内螺纹导程、螺纹头数已在总初始化中赋值initInterpStructByTool()函数

    pSG33->dG33SpindleSpeed = getG97AbsSpindleSpeed(pSSpindle) * 60 * 1000; //(r/min)（2023 / 3 / 23郭强）

    pSG33->ucAlreadyL = 0; //已加工头数为零

    pSG33->G33MotionState = ZAXIS_ACC_BEFORE_FOL_UP; //初次攻牙默认存在G32中的0"Q值"

    pSG33->bPreFinishedFlag = FALSE; //未进行速度规划或平摊预处理或主轴未进行速度规划

    pSG33->bZAxisFeedSignal = FALSE; //Z轴多启动加工信号

    pSG33->bG33AccPreFirstTime =
        TRUE; //加速预处理第一个周期，此标志位用于在加速预处理第一个周期将启动最大偏差放入规划距离中

    //主轴反转第一个周期标志位，初始状态为FALSE,当通过编码器方向判断主轴反转时改为TRUE
    pSG33->bSpindleRevBegin = FALSE;

    pSG33->bSpindleRevCompEnd = FALSE;        //主轴反转补偿结束标志位
    pSG33->bSpindleRevCompEndFirstTc = TRUE;  //主轴反转补偿结束后第一个周期标志位
    pSG33->dSpindleRevCompZAxisCumulaDis = 0; //主轴反转补偿期间Z轴累积距离

    pSG33->dG33Acc = g_SSystemPara.dSysPlanTMaxAcc; //G33运行过程中新增函数用加速度加速度
    pSG33->dG33Dec = g_SSystemPara.dSysPlanTMaxDec;

    //获取实时点，用于加速阶段计算加速度虚拟运行阶段真实实时点的还原
    getRealTimePoint((double*)&pSG33->G33StartRealTimePoint, AXIS_NUM_MAX, pSG33->pSChannel->pMapAxisArray,
                     &pSG33->ucValStatus);
    getRealTimePoint((double*)&pSG33->G33AnalogRealTimePoint, AXIS_NUM_MAX, pSG33->pSChannel->pMapAxisArray,
                     &pSG33->ucValStatus);

    //2.保存G33正向攻丝运行时主轴转向
    pSG33->cTappingSpindleDir = pSSpindle->pSSpindleAxis->cSpindleDir > 0 ? 1 : -1;

    //3.主轴测速
#if !G33_USE_REAL_ENCODER
    if (pSG33->G33RotateAxis == AXIS_A)
    {
        pSG33->pSChannel->bCalculateSecondSpindleSpeedFlag = FALSE; //模拟编码器时不用测速 陈敏聪	6/22
        pSG33->pSChannel->dSecondSpindleAnalogSpeed =
            round(getG97AbsSpindleSpeed(pSSpindle) * g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse) /
            (double)g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse; //主轴实际转速
    }
    else
    {
        pSG33->pSChannel->bCalculateSpindleSpeedFlag = FALSE; //模拟编码器时不用测速
        pSG33->pSChannel->dSpindleRealSpeed =
            round(getG97AbsSpindleSpeed(pSSpindle) * g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse) /
            (double)g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse; //主轴实际转速
    }
    pSpindleEncoder->cEncoderDir = pSSpindle->pSSpindleAxis->cSpindleDir > 0 ? 1 : -1;
#else
    if (pSG33->G33RotateAxis == AXIS_A)
    {
        pSG33->pSChannel->bCalculateSecondSpindleSpeedFlag = TRUE; //模拟编码器测量主轴转速	陈敏聪	2023/6/22
    }
    else
    {
        pSG33->pSChannel->bCalculateSpindleSpeedFlag = TRUE; //真实编码器测量主轴转速
    }
#endif

    //4.初始化G33主轴反转前暂停时间变量       2023.5.8       郭强
    pSG33->unG33SpindlePauseTimeBeforeRev = g_SSystemPara.unG33SpindlePauseTimeBeforeRev;

#if G33ProcLog_DEBUG
    printf("导程:dF=%lf\n", pSG33->dF);
    printf("G33初始化运行状态:G33MotionState=%d\n", pSG33->G33MotionState);
    printf("bPreFinishedFlag=%d\n", pSG33->bPreFinishedFlag);
    printf("bZAxisFeedSignal=%d\n", pSG33->bZAxisFeedSignal);
    printf("ucL=%u\n", pSG33->ucL);
    printf("ucAlreadyL=%u\n", pSG33->ucAlreadyL);
    printf("**********************gG33结构体初始化完成！****************************\n");
#endif
    return 1;
}

/*************************************************
  函数名称：changeG33StateAfterSpindleAccOrDec(struct G33 *pSG33)
  函数功能  主轴加减速完成后改变G33运行状态
  输入参数：pSG33 G33结构体指针
  输出参数：
  返 回 值：-1 :指针为NULL
			0 :不用更改状态
			1 :攻丝前主轴加速完成
			2 :主轴减速停完成，攻丝完成
			3 ：主轴减速停完成，攻丝未完成
			4 :在主轴减速停完成后需要暂停
			5 :主轴反转退刀前减速停完成
  备    注：
**************************************************/
int8 changeG33StateAfterSpindleAccOrDec(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle,
                                        struct G33* pSG33)
{
    struct SSingleAxis* pSingleAxisPointer = pSSpindle->pSSpindleAxis;
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组

    if (pInterpPrePointer == NULL || pSG33 == NULL || pSSpindle == NULL)
    {
        return -1;
    }

    if (pSG33->G33MotionState == SPINDLE_ACC_BEFORE_ZAXIS_FEEDING && pSG33->bPreFinishedFlag == TRUE &&
        pSingleAxisPointer->ucInpStatus == INP_FINISHED) //攻丝前加速完成
    {
        pSG33->G33MotionState = ZAXIS_ACC_BEFORE_FOL_UP;
        pSG33->bPreFinishedFlag = FALSE;

        //加速规划前要测主轴实际转速
#if !G33_USE_REAL_ENCODER
        if (pSG33->G33RotateAxis == AXIS_A)
        {
            pSG33->pSChannel->bCalculateSecondSpindleSpeedFlag = FALSE; //模拟编码器时不用测速 陈敏聪	6/22
            pSG33->pSChannel->dSecondSpindleAnalogSpeed =
                round(getG97AbsSpindleSpeed(pSSpindle) *
                      g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse) /
                (double)g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse; //主轴实际转速
        }
        else
        {
            pSG33->pSChannel->bCalculateSpindleSpeedFlag = FALSE; //模拟编码器时不用测速
            pSG33->pSChannel->dSpindleRealSpeed =
                round(getG97AbsSpindleSpeed(pSSpindle) *
                      g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse) /
                (double)g_SSystemPara.SAxisPara[ucMapArray[SPINDLE_AXIS]].usMmToPulse; //主轴实际转速
        }
#else
        if (pSG33->G33RotateAxis == AXIS_A)
        {
            pSG33->pSChannel->bCalculateSecondSpindleSpeedFlag = TRUE; //模拟编码器测量主轴转速	陈敏聪	2023/6/22
        }
        else
        {
            pSG33->pSChannel->bCalculateSpindleSpeedFlag = TRUE; //真实编码器测量主轴转速
        }
#endif

#if G33ProcLog_DEBUG
        printf("*****************攻丝前主轴加速完成****************\n");
#endif
        return 1;
    }
    else if (pSG33->G33MotionState == SPINDLE_DEC_TO_STOP_BEFORE_REV && pSG33->bPreFinishedFlag == TRUE &&
             isSpindleStopFinished(pSSpindle)) // zxr: 等完整停机收尾后再反转，避免零停留启动被拒绝
    {
#if G33ProcLog_DEBUG
        printf("*****************主轴反转前减速完成！****************\n");
#endif

        //1.根据反转前是否需要暂停转换运动状态
        if (!isZero(g_SSystemPara.unG33SpindlePauseTimeBeforeRev))
        {
            pSG33->G33MotionState = PAUSE_BEFORE_REVERSAL; //暂停时间不为0，进入暂停阶段
        }
        else
        {
            pSG33->G33MotionState = SPINDLE_REVERSE_RETURN; //暂停时间为0，直接反转
        }

        //2.置FALSE运动阶段预处理标志位
        pSG33->bPreFinishedFlag = FALSE;

        return 5;
    }
    else if (pSG33->G33MotionState == SPINDLE_DEC_TO_STOP_AFTER_ZAXIS_RETURN && pSG33->bPreFinishedFlag == TRUE &&
             pSingleAxisPointer->ucInpStatus == INP_FINISHED) //退刀后主轴减速完成
    {
        // zxr: 本头攻牙完成后清零随动无反馈计数，避免带入后续加工
        pSG33->unFollowTimeOut = 0;
        pSG33->ucAlreadyL++; //已加工头数+1

        //还原攻牙转速(郭强，2023/3/23)
        //setSpindleSpeedAPI(pSG33->pSChannel->ucChannelNum, pSG33->dG33SpindleSpeed, CMD_SPINDLE_SPEED_CV);
        setSpindleSpeed(pSSpindle, pSG33->dG33SpindleSpeed,
                        CMD_SPINDLE_SPEED_CV); //根据G33RotateAxis恢复对应轴的转速，2026.4.28
#if G33ProcLog_DEBUG
        printf("*****退刀后主轴减速完成,加工完成头数：%u,共%u头****************\n", pSG33->ucAlreadyL, pSG33->ucL);
#endif

        if (pSG33->ucAlreadyL == pSG33->ucL) //攻丝完毕
        {
            pSG33->ucAlreadyL = 0; //防止最初进入G33时被误认为时暂停恢复加工
            pSG33->bPreFinishedFlag = FALSE;
            pSG33->pSChannel->bG33SpindleOverSpeedWarningSigns = FALSE; //超速警告标志位置FALSE;
            pInterpPrePointer->ucInpStatus = INP_FINISHED;              //改变总插补预处理结构体状态为完成

            if (pSG33->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE) //系统从运行到暂停状态
            {
                pSG33->pSChannel->ucChannelStatus = SYS_PAUSE;

                return 4;
            }
            return 2;
        }
        else //未加工完
        {
            pSG33->G33MotionState = SPINDLE_ACC_BEFORE_ZAXIS_FEEDING;
            pSG33->bPreFinishedFlag = FALSE;

            pSG33->bG33AccPreFirstTime = TRUE; //首次加速预处理，用于增加最大启动偏差

            pSG33->bSpindleRevBegin = FALSE;

            pSG33->bSpindleRevCompEnd = FALSE;        //主轴反转补偿结束标志位
            pSG33->bSpindleRevCompEndFirstTc = TRUE;  //主轴反转补偿结束后第一个周期标志位
            pSG33->dSpindleRevCompZAxisCumulaDis = 0; //主轴反转补偿期间Z轴累积距离

            //将G33运行加速度改为最初时系统参数，待进入加速预处理阶段进行调整
            pSG33->dG33Acc = g_SSystemPara.dSysPlanTMaxAcc;
            pSG33->dG33Dec = g_SSystemPara.dSysPlanTMaxDec;
            getRealTimePoint((double*)&pSG33->G33AnalogRealTimePoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

            //更新插补结构体起点终点
            pSG33->SEndPoint = pSG33->G33EndPoint;
            getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

            //当前速度
            pSG33->dCurrentSpeed = 0;
            //向量和距离
            calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                    &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                    pSG33->ucRelaVal);
            pSG33->SPlanData.dPreDist = 0;
            pSG33->SPlanData.dCurLeftDist = 0;
            pSG33->SPlanData.dTargetDist = 0;

            if (pSG33->pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE) //系统从运行到暂停状态
            {
                pSG33->pSChannel->ucChannelStatus = SYS_PAUSE;

                pSG33->pSChannel->bG33SpindleOverSpeedWarningSigns = FALSE; //超速警告标志位置FALSE;

                //回写
                //g_SRingBuff[pSG33->pSChannel->ucChannelNum].sSToolDealedNum += 1;// 刀补处理过的数据+1  插补读指针往回移动一个位置
                //g_SRingBuff[pSG33->pSChannel->ucChannelNum].sLookAheadDealedNum += 1;
                //2023.5.25 朱稳中
                moveSToolCompBufPointer(INTERPOLATION_READ, -1, pSG33->pSChannel->ucChannelNum);
                setBits(pInterpPrePointer->unCommandType, CMD_PROGRAM_CONTINUE,
                        CMD_PROGRAM_ADDR);                     //防止当前段的M30/M02，导致程序提前停止
                pInterpPrePointer->ucInpStatus = INP_FINISHED; //改变总插补预处理结构体状态为完成

                return 4;
            }
            return 3;
        }
    }
    else
    {
        return 0;
    }
}

/*************************************************
* （待删）
  函数名称：singleAxisInitAndPrePlanS4
  函数功能  指定速度位移的单轴初始化及速度规划（不含余量分摊）
  输入参数：dDistance   ：该轴目标点-当前点坐标
  输出参数：
  返 回 值：1：正常运行
			0：输入的距离dDistance为0
			-1：ucAxis输入参数错误
  备    注：为了G33加速阶段增加不带余量分摊
			速度曲线类型：S4
			分摊类型    ：T型
**************************************************/

int8 singleAxisInitAndPrePlanS4(uint8 ucAxis, double dDistance, double dVs, double dVmax, double dVe)
{
    //struct SInterpolationPre* pInterpPrePointer; //预处理结构体指针

    //if (isZero(dDistance)) //移动距离为0，直接返回1
    //{
    //	return 0;
    //}

    ////根据轴号选择不同的预处理结构体
    //switch (ucAxis)
    //{
    //case AXIS_X://X轴
    //	pInterpPrePointer = &g_SPreProcessAxisX;
    //	pInterpPrePointer->SStartVector.dIX = sgn(dDistance);//确定插补方向
    //	pInterpPrePointer->SStartPoint.dX = g_SRealTimePoint.dX;//获取当前点坐标 作为起点
    //	break;
    //case AXIS_Y://Y轴
    //	pInterpPrePointer = &g_SPreProcessAxisY;
    //	pInterpPrePointer->SStartVector.dIY = sgn(dDistance);//确定插补方向
    //	pInterpPrePointer->SStartPoint.dY = g_SRealTimePoint.dY;//获取当前点坐标 作为起点
    //	break;
    //case AXIS_Z://Z轴
    //	pInterpPrePointer = &g_SPreProcessAxisZ;
    //	pInterpPrePointer->SStartVector.dIZ = sgn(dDistance);//确定插补方向
    //	pInterpPrePointer->SStartPoint.dZ = g_SRealTimePoint.dZ;//获取当前点坐标 作为起点
    //	break;
    //default:
    //	return -1;//轴号错误
    //}

    //pInterpPrePointer->ucInpStatus = INP_ASSIGNMENT_START;//插补预处理结构体置为赋值状态

    ////初始化参数
    ////pInterpPrePointer->ucVelPlanType		= 3;			//速度曲线类型，1：梯形，2：3次S型，3：4次S型
    //pInterpPrePointer->ucDistReAverageType = 1;		//平摊曲线类型，1：梯形，2：5次，3：比例系数

    //pInterpPrePointer->dVmax = dVmax;	//最大速度 mm/ms
    //pInterpPrePointer->dVs = dVs;		//起始速度 mm/ms
    //pInterpPrePointer->dVe = dVe;		//终点速度 mm/ms

    //pInterpPrePointer->dTc = g_SSystemPara.dInterpolationTime;		//插补周期

    //pInterpPrePointer->dAcc = getSingleAxisAcc(ucAxis, pInterpPrePointer->ucAccVelPlanType);
    //pInterpPrePointer->dDec = getSingleAxisDec(ucAxis, pInterpPrePointer->ucDecVelPlanType);

    //pInterpPrePointer->dDistance = fabs(dDistance);//距离的符号为正
    //pInterpPrePointer->dCurLeftDist = pInterpPrePointer->dDistance;//当前剩余的距离

    //if (pInterpPrePointer->ucInpStatus == INP_ASSIGNMENT_START)
    //{
    //	//S4速度规划，先不平摊,不平摊是因为后期可能将补偿需要的位移加载余量上，所以分开
    //	preparePlanningS4(pInterpPrePointer);//S4型速度规划
    //}

    //return 1;

    //若函数外进行平摊之后要
    // 	1.	pInterpPrePointer->unCurrentT = 0;
    // 	2.	pInterpPrePointer->ucInpStatus = INP_RUNNING; //先准备好数据，再改状态。否则，中断进入后，会出问题。
}

/*************************************************
  函数名称：motionSpindleAccBeforeZAxisFeeding()
  函数功能：Z轴进刀前主轴加速
  输入参数：
  输出参数：
  返 回 值：
  备    注：攻多头螺纹才有这种状态
**************************************************/
void motionSpindleAccBeforeZAxisFeeding(struct G33* pSG33)
{
    if (pSG33->bPreFinishedFlag == FALSE)
    {
        //主轴加速预处理
        //1.根据G33正向攻丝时转向设置主轴
        if (pSG33->cTappingSpindleDir == 1) //M03正转
        {
            //setSpindleDirAPI(pSG33->pSChannel->ucChannelNum, ARC_CW);
            if (pSG33->G33RotateAxis == AXIS_A) //A轴
            {
                setSpindleDir(&g_SChannel[pSG33->pSChannel->ucChannelNum].SSecondSpindle, ARC_CW);
            }
            else //C轴
            {
                setSpindleDir(&g_SChannel[pSG33->pSChannel->ucChannelNum].SSpindle, ARC_CW);
            }
        }
        else //反转
        {
            //setSpindleDirAPI(pSG33->pSChannel->ucChannelNum, ARC_ANTI_CW);
            if (pSG33->G33RotateAxis == AXIS_A) //A轴
            {
                setSpindleDir(&g_SChannel[pSG33->pSChannel->ucChannelNum].SSecondSpindle, ARC_ANTI_CW);
            }
            else //C轴
            {
                setSpindleDir(&g_SChannel[pSG33->pSChannel->ucChannelNum].SSpindle, ARC_ANTI_CW);
            }
        }

        //spindleBeginAPI(pSG33->pSChannel->ucChannelNum);
        if (pSG33->G33RotateAxis == AXIS_A) //A轴
        {
            SSecondSpindleBegin(&g_SChannel[pSG33->pSChannel->ucChannelNum].SSecondSpindle);
        }
        else //C轴
        {
            spindleBegin(&g_SChannel[pSG33->pSChannel->ucChannelNum].SSpindle);
        }

#if G33ProcLog_DEBUG
        printf("*****************Z轴进刀前主轴开始加速****************\n");
#endif
        //将主轴预处理状态位置TRUE
        pSG33->bPreFinishedFlag = TRUE;
    }
}

/*************************************************
  函数名称：calcMaxStartErrCompenDis()
  函数功能: 计算加速阶段最大补偿位移
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
double calcMaxStartErrCompenDis(struct G33* pSG33)
{
    if (pSG33 == NULL)
    {
        return 0;
    }
    double MaxStartErrCompenDis = 0;
    if (pSG33->G33RotateAxis == AXIS_A)
    {
        MaxStartErrCompenDis =
            fabs(0.55 * pSG33->pSChannel->dSecondSpindleAnalogSpeed * pSG33->dF * pSG33->SPlanData.dTc); //陈敏聪 6/22
    }
    else
    {
        MaxStartErrCompenDis = fabs(0.55 * pSG33->pSChannel->dSpindleRealSpeed * pSG33->dF * pSG33->SPlanData.dTc);
    }
    return MaxStartErrCompenDis;
}

/*************************************************
  函数名称：ZAxisAccPre(struct G33* pSG33, struct SEncoder* pSpindleEncoder,int8 cSpindleRevFlag)
  函数功能  Z轴加速预处理
		   1.计算编码器启动值
		   2.模拟运行，计算满足避空距离要求的加速度和减速度
  输入参数： pSG33
		   pSpindleEncoder
		   cSpindleRevFlag
  输出参数：
  返 回 值：-1加速度未计算完毕
			1加速度计算完毕

			-2指针为空
  备    注：
**************************************************/
int8 ZAxisAccPre(struct G33* pSG33, struct SEncoder* pSpindleEncoder, int8 cSpindleRevFlag)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    double dZAxisNewAddDist;                             //新增距离
    double dVe;

    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];

    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -2;
    }

    //当避空距离为0时，不用虚拟运行，直接将加速度改为系统参数的10倍
    if (isZero(g_SSystemPara.dG33AvoidEmptyDistance))
    {
#if G33ProcLog_DEBUG
        printf("*****************避空距离为0,加速度扩大10倍,Z轴加速度计算完毕！****************\n");
#endif
        pSG33->dZAxisStartEncoderRelValue = calcZAxisStartEncoderRelValue(pSG33, pSpindleEncoder);
        pSG33->dG33Acc = pSG33->dG33Acc * 10;
        pSG33->dG33Dec = pSG33->dG33Dec * 10;
        pSG33->bPreFinishedFlag = TRUE;
        return 1;
    }

    //首先进行编码器方向检测，异常反向直接返回（2022/8/6）
    if (cSpindleRevFlag == TRUE)
    {
#if G33ProcLog_DEBUG
        printf("*****************加速预处理阶段异常反向、重新进行预处理！！****************\n");
#endif
        pSG33->bG33AccPreFirstTime = TRUE; //恢复到第一次加速预处理的状态
        //当前速度置零
        pSG33->dCurrentSpeed = 0;

        //模拟实时点还原为真实实时点
        pSG33->G33AnalogRealTimePoint = pSG33->G33StartRealTimePoint;
        return -1;
    }

    //计算上一周期主轴编码器增量对应的z轴位移
    dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
    dVe = dZAxisNewAddDist / pSG33->SPlanData.dTc;

    double dDeltaL;            //相对起点的距离 (或弧长)
    SPointDouble STargetPoint; //当前点的理论绝对坐标

    //将实时点改为模拟运行时的实时点坐标
    //g_SRealTimePoint = pSG33->G33AnalogRealTimePoint;
    g_dRealTimePointArray[ucMapArray[ucG33Axis1]] = pSG33->G33AnalogRealTimePoint.dArray[ucG33Axis1];
    g_dRealTimePointArray[ucMapArray[ucG33Axis2]] = pSG33->G33AnalogRealTimePoint.dArray[ucG33Axis2];

    pSG33->dZAxisStartEncoderRelValue = calcZAxisStartEncoderRelValue(pSG33, pSpindleEncoder);

    //1.'第一次走'，将启动最大可能偏差的补偿位移放入（注意只添加一次）
    if (pSG33->bG33AccPreFirstTime)
    {
        //计算实际主轴转速下最大启动偏差补偿位移
        double dMaxStartErrCompenDis = calcMaxStartErrCompenDis(pSG33);
        pSG33->SPlanData.dPreDist = 0;
        pSG33->SPlanData.dCurLeftDist = dMaxStartErrCompenDis;
        pSG33->SPlanData.dTargetDist = pSG33->SPlanData.dCurLeftDist;

        pSG33->bG33AccPreFirstTime = FALSE;
    }

    //主轴停止特殊情况处理
    if (isZero(dVe))
    {
        //将实时点换回真实实时点
        //g_SRealTimePoint = pSG33->G33StartRealTimePoint;
        g_dRealTimePointArray[ucMapArray[ucG33Axis1]] = pSG33->G33StartRealTimePoint.dArray[ucG33Axis1];
        g_dRealTimePointArray[ucMapArray[ucG33Axis2]] = pSG33->G33StartRealTimePoint.dArray[ucG33Axis2];

        //当前速度置零
        pSG33->dCurrentSpeed = 0;

        //直接返回
        return -1;
    }

    //2.根据主轴上一周期的转角对应的Z轴位移，调用新增函数
    if (newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe, FALSE,
                                g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed) == 1)
    {
        //计算当前点距起点距离
        double distance;
        distance =
            sqrt(pow(g_dRealTimePointArray[ucMapArray[ucG33Axis1]] - pSG33->G33StartPoint.dArray[ucG33Axis1], 2) +
                 pow(g_dRealTimePointArray[ucMapArray[ucG33Axis2]] - pSG33->G33StartPoint.dArray[ucG33Axis2], 2));

        //无需输出，直接获取实时点的位置，与起刀点作差，看是否在避空距离内
        if (distance > g_SSystemPara.dG33AvoidEmptyDistance)
        {
            pSG33->dG33Acc = pSG33->dG33Acc * distance / g_SSystemPara.dG33AvoidEmptyDistance;
            pSG33->dG33Dec = pSG33->dG33Dec * distance / g_SSystemPara.dG33AvoidEmptyDistance;
        }

        //加速度已计算出
        pSG33->bPreFinishedFlag = TRUE;

        //结构体最初的当前速度还原
        pSG33->dCurrentSpeed = 0;
        pSG33->SPlanData.dPreDist = 0;
        pSG33->SPlanData.dCurLeftDist = 0;
        pSG33->SPlanData.dTargetDist = 0;

        //将实时点换回真实实时点
        //g_SRealTimePoint = pSG33->G33StartRealTimePoint;
        g_dRealTimePointArray[ucMapArray[ucG33Axis1]] = pSG33->G33StartRealTimePoint.dArray[ucG33Axis1];
        g_dRealTimePointArray[ucMapArray[ucG33Axis2]] = pSG33->G33StartRealTimePoint.dArray[ucG33Axis2];

        //清空G33插补轴脉冲，达到虚拟运行的效果(对多通道是否有影响，后期处理)
        //g_SRun.sAxisDeltaPulse[AXIS_Z - 1] = 0;
        //g_SRun.sAxisDeltaPulse[AXIS_X - 1] = 0;
        //g_SRun.sAxisDeltaPulse[AXIS_Y - 1] = 0;

        pSG33->pSChannel->SRun.sAxisDeltaPulse[ucG33Axis1] = 0;
        pSG33->pSChannel->SRun.sAxisDeltaPulse[ucG33Axis2] = 0;

#if G33ProcLog_DEBUG
        printf("*****************Z轴加速度计算完毕！****************\n");
#endif
        return 1;
    }
    else //仍在加速,未达到随动阶段
    {
        //3.模拟输出（和真实运行一样一个周期输出一次，调用一次）
        //运行时间增加
        pSG33->SPlanData.unCurrentT += 1;
        //获取currentT时刻的弧长(圆弧）或 线段长（直线）
        dDeltaL = nextstepPlanningAndDistReAvg(pSG33->SPlanData.unCurrentT, &pSG33->SPlanData);

        //计算长轴理论坐标点
        //STargetPoint = calG33ExactInterpolationPoint(dDeltaL, pSG33);
        calculateLinearInterpolationPoint((double*)&pSG33->SStartPoint, (double*)&pSG33->SStartVector, dDeltaL,
                                          (double*)&STargetPoint, AXIS_NUM_MAX);

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
        //利用理论坐标生成SRun结构体数据
        outPutAbsPointWithScrewComp(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, NULL,
                                    &pSG33->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
        outPutAbsPointWithBilateralScrewComp(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray,
                                             NULL, &pSG33->ucValStatus);
#else
        //利用理论坐标生成SRun结构体数据
        outPutAbsPoint(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, NULL,
                       &pSG33->ucValStatus);
#endif

        //计算当前速度
        pSG33->dCurrentSpeed =
            fabs(pSG33->SPlanData.dCurLeftDist - (pSG33->SPlanData.dTargetDist - dDeltaL)) / pSG33->SPlanData.dTc;

        //计算   !理论! 未走距离
        pSG33->SPlanData.dCurLeftDist = pSG33->SPlanData.dTargetDist - dDeltaL;

        //更新已加工距离（原位于新增函数内部）
        pSG33->SPlanData.dPreDist = pSG33->SPlanData.dTargetDist - pSG33->SPlanData.dCurLeftDist; //已插补长度

        //保存实时点到G33结构体
        //pSG33->G33AnalogRealTimePoint = g_SRealTimePoint;
        pSG33->G33AnalogRealTimePoint.dArray[ucG33Axis1] = g_dRealTimePointArray[ucMapArray[ucG33Axis1]];
        pSG33->G33AnalogRealTimePoint.dArray[ucG33Axis2] = g_dRealTimePointArray[ucMapArray[ucG33Axis2]];
    }

    //将实时点换回真实实时点
    //g_SRealTimePoint = pSG33->G33StartRealTimePoint;
    g_dRealTimePointArray[ucMapArray[ucG33Axis1]] = pSG33->G33StartRealTimePoint.dArray[ucG33Axis1];
    g_dRealTimePointArray[ucMapArray[ucG33Axis2]] = pSG33->G33StartRealTimePoint.dArray[ucG33Axis2];

    //清空Z轴脉冲，达到虚拟运行的效果(对多通道是否有影响，后期处理)
    //g_SRun.sAxisDeltaPulse[AXIS_Z - 1] = 0;
    //g_SRun.sAxisDeltaPulse[AXIS_X - 1] = 0;
    //g_SRun.sAxisDeltaPulse[AXIS_Y - 1] = 0;

    pSG33->pSChannel->SRun.sAxisDeltaPulse[ucG33Axis1] = 0;
    pSG33->pSChannel->SRun.sAxisDeltaPulse[ucG33Axis2] = 0;

    return -1;
}

/*************************************************
  函数名称：calcZAxisNewAddDist(struct G33 *pSG33,struct SEncoder *pSpinddleEncoder)
  函数功能  //计算主轴编码器一个周期增量对应Z轴新增位移
  输入参数：pSG33
		   pSpindleEncoder
  输出参数：
  返 回 值：

  备    注：
**************************************************/

double calcZAxisNewAddDist(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return 0;
    }
    double dSpindleRevsToEDelta = (double)pSpindleEncoder->usEncoderCirclePulseNum; //主轴转一转编码器计数器增量
    double dEncoderDelta = (double)pSpindleEncoder->usDelta;

    //2022-10-25加入导程修正百分比，根据编码器方向和攻牙正向是否相同决定采用哪一个修正百分比
    double dCorrectedG33F; //G33修正后导程值

    //判断主轴编码器方向，确定对应导程修正值
    if (pSpindleEncoder->cEncoderDir == pSG33->cTappingSpindleDir) //方向为：正向攻牙
    {
        dCorrectedG33F = pSG33->dF * g_SSystemPara.dPercentageOfG33EnterLeadCorrection; //攻牙方向修正百分比
    }
    else //方向为：退刀
    {
        dCorrectedG33F = pSG33->dF * g_SSystemPara.dPercentageOfG33ReturnLeadCorrection; //退刀方向修正百分比
    }

    return dEncoderDelta * dCorrectedG33F / dSpindleRevsToEDelta;
}

/*************************************************
  函数名称：newAddDistOutPut(struct G33* pSG33)
  函数功能  实时规划方案输出（计算坐标，计算当前点实时点）
  输入参数：
  输出参数：
  返 回 值：

  备    注：当Z轴获得进给信号后才能准确算出要补偿的位移
**************************************************/
int8 newAddDistOutPut(struct G33* pSG33)
{
    double dDeltaL;                                      //相对起点的距离 (或弧长)
    SPointDouble STargetPoint;                           //当前点的理论绝对坐标
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组

    if (pSG33 == NULL)
    {
        return -1;
    }

    //运行时间增加
    pSG33->SPlanData.unCurrentT += 1;
    //获取currentT时刻的弧长(圆弧）或 线段长（直线）
    dDeltaL = nextstepPlanningAndDistReAvg(pSG33->SPlanData.unCurrentT, &pSG33->SPlanData);

    ////计算理论坐标点
    //STargetPoint = calG33ExactInterpolationPoint(dDeltaL, pSG33);

    //outPutAbsPoint(STargetPoint);  //利用理论坐标生成SRun结构体数据

    calculateLinearInterpolationPoint((double*)&pSG33->SStartPoint, (double*)&pSG33->SStartVector, dDeltaL,
                                      (double*)&STargetPoint, AXIS_NUM_MAX);

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
    //利用理论坐标生成SRun结构体数据
    outPutAbsPointWithScrewComp(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, NULL,
                                &pSG33->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
    outPutAbsPointWithBilateralScrewComp(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray,
                                         NULL, &pSG33->ucValStatus);
#else
    //利用理论坐标生成SRun结构体数据
    outPutAbsPoint(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, NULL,
                   &pSG33->ucValStatus);
#endif

    //计算当前速度
    pSG33->dCurrentSpeed =
        fabs(pSG33->SPlanData.dCurLeftDist - (pSG33->SPlanData.dTargetDist - dDeltaL)) / pSG33->SPlanData.dTc;

    //计算   !理论! 未走距离
    pSG33->SPlanData.dCurLeftDist = pSG33->SPlanData.dTargetDist - dDeltaL;

    //更新已加工距离（原位于新增函数内部）
    pSG33->SPlanData.dPreDist = pSG33->SPlanData.dTargetDist - pSG33->SPlanData.dCurLeftDist; //已插补长度

    return 1;
}

/*************************************************
  函数名称：motionZAxisAccBeforeFolUp(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  Z轴随动前加速
  输入参数：
  输出参数：
  返 回 值：
			2 :指针为空
			1：正常运行
			0：不符合启动条件，不启动
		   -1: 加速阶段的加速度未计算出，不启动
		   -2：加速的阶段主轴编码器前后周期差值为0（主轴异常，不转），此时不规划，不输出脉冲
		   -3：等待编码器启动信号阶段异常反向，不启动
		   -4：加速已启动但是主轴异常反向，此时不再管对准问题直接转如随动攻牙阶段

  备    注：当Z轴获得进给信号后才能准确算出要补偿的位移
**************************************************/
int8 motionZAxisAccBeforeFolUp(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    const uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    double dZAxisNewAddDist;                                   //新增距离
    double dVe;
    int ret;
    int8 cSpindleRevFlag;        //主轴异常反向标志
    static uint32 unTimeOut = 0; //记录主轴异常反向连续次数 // 2026.5.9
    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return 2;
    }

    //首先进行编码器方向检测（2022/8/8）
    cSpindleRevFlag = pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir ? TRUE : FALSE;

    unTimeOut = cSpindleRevFlag == TRUE ? unTimeOut + 1 : 0; //记录主轴异常反向连续次数

    if (unTimeOut > G33_TIMERANGE) //连续主轴异常反向，则报错
    {
        parseRunningError(pSG33->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM154, 9999);
        return -3;
    }

    if (pSG33->bZAxisFeedSignal == FALSE) //Z轴未获得同步进给信号
    {
        if (pSG33->bPreFinishedFlag ==
            FALSE) //此预处理未完成代表的是Z轴启动时编码器理论相对值未计算出且加速阶段加速度和减速度未规划完成
        {
#if G33ProcLog_DEBUG
            printf("*****************Z轴加速规划开始！****************\n");
#endif
            if (ZAxisAccPre(pSG33, pSpindleEncoder, cSpindleRevFlag) == -1)
            {
#if G33ProcLog_DEBUG
                printf("*****************Z轴加速阶段加速度未规划完毕！****************\n");
#endif
                return -1;
            }
        }

        //判断当前周期能否启动：
        // 不能启动直接返回return；
        // 可以启动：根据启动偏差量计算Z轴补偿量
        //           改变原Z轴速度规划的残余距离和目标距离
        //           更改主轴进给信号标志位为TRUE
        if (isGetSyncSignal(pSG33->dZAxisStartEncoderRelValue, pSG33->pSChannel,
                            pSpindleEncoder)) //符合启动条件(函数在G32.c中)
        {
            double dZAxisComDistance = 0; //加速阶段启动误差补偿距离
            //主轴转一转编码器计数器增量
            double dSpindleRevsToEDelta;
            //导程
            double dF;

            //首先进行编码器方向检测，异常反向即使主轴位置合适也直接返回，不启动（2022/8/6）
            if (cSpindleRevFlag == TRUE)
            {
#if G33ProcLog_DEBUG
                printf("**编码器值合适，但主轴异常反向，不启动！**\n");
#endif
                return -3;
            }

            dSpindleRevsToEDelta = (double)pSpindleEncoder->usEncoderCirclePulseNum;
            dF = pSG33->dF;

            //导程修正
            //判断主轴编码器方向，确定对应导程修正值
            //if (cSpindleRevFlag == FALSE)
            if (pSpindleEncoder->cEncoderDir == pSG33->cTappingSpindleDir) //方向为：正向攻牙 2026.5.9修改判断条件
            {
                dF = dF * g_SSystemPara.dPercentageOfG33EnterLeadCorrection; //攻牙方向修正百分比
            }
            else //方向为：退刀
            {
                dF = dF * g_SSystemPara.dPercentageOfG33ReturnLeadCorrection; //退刀方向修正百分比
            }

            //根据启动偏差量计算Z轴补偿量
            dZAxisComDistance = pSpindleEncoder->dEncoderNumDeviation * dF / dSpindleRevsToEDelta;

            pSG33->SPlanData.dCurLeftDist = dZAxisComDistance;
            pSG33->SPlanData.dTargetDist = pSG33->SPlanData.dCurLeftDist;

            pSG33->bZAxisFeedSignal = TRUE;

            //注：isGetSyncSignal()函数内部对编码器绝对值累积了一个变化量，一次弥补编码器滞后的问题
#if G33ProcLog_DEBUG
            printf("****Z轴启动！**理论启动值：%lf**实际启动值：%d**脉冲偏差：%lf\n", pSG33->dZAxisStartEncoderRelValue,
                   pSpindleEncoder->lEncoderAbsValue, pSpindleEncoder->dEncoderNumDeviation);
#endif
        }
        else
        {
#if G33ProcLog_DEBUG
            printf("**编码器值不合适，不启动！**理论启动值：%lf**本周期编码器绝对值：%d\n",
                   pSG33->dZAxisStartEncoderRelValue, pSpindleEncoder->lEncoderAbsValue);
#endif
            return 0; //不启动
        }
    }

    //加速已启动阶段，主轴异常反向，直接进入随动攻丝阶段，注意随动攻丝阶段的修改
    if (cSpindleRevFlag == TRUE)
    {
#if G33ProcLog_DEBUG
        printf("**加速已启动阶段主轴异常反向，直接转入随动攻丝阶段！**\n");
#endif
        pSG33->G33MotionState = ZAXIS_FOL_UP_TAPPING;
        pSG33->bPreFinishedFlag = FALSE;
        pSG33->bZAxisFeedSignal = FALSE;

        //调用随动攻牙函数
        motionZAxisFolUpTapping(pSSpindle, pSG33, pSpindleEncoder);
        return -4;
    }

    //计算上一周期主轴编码器增量对应的z轴位移
    dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
    dVe = dZAxisNewAddDist / pSG33->SPlanData.dTc;

    //对于主轴加速阶段主轴上一周期脉冲为0，即主轴停
    //不在规划，即不输出脉冲（此时新增距离为零，所以不规划没影响）
    if (isZero(dVe))
    {
#if G33ProcLog_DEBUG
        printf("*****************上周期主轴编码器差值为0（主轴未转），不输出脉冲！****************\n");
#endif
        pSG33->dCurrentSpeed = 0;
        return -2;
    }
    ret = newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe, FALSE,
                                  g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed);

    //运动输出
    newAddDistOutPut(pSG33);

    //判断加速追赶阶段是否完成（注意此时G33状态改变的位置和主轴加减速阶段完成后改变G33状态的位置区别）
    if (ret == 1)
    {
        pSG33->G33MotionState = ZAXIS_FOL_UP_TAPPING; //将G33运动状态改变为随动攻丝

        pSG33->bZAxisFeedSignal = FALSE; //将Z轴进给信号置FALSE,用于下一头螺纹加工，重新等待

        pSG33->bPreFinishedFlag = FALSE; //置FALSE表示下一阶段主轴随动未预处理

#if G33ProcLog_DEBUG
        printf("********Z轴随动前加速完成！Z轴坐标：%lf**编码器绝对数值%d\n", g_dRealTimePointArray[ucMapArray[AXIS_Z]],
               getSpindleEncoderValue(pSpindleEncoder));
        printf("*****************下一行数据为本阶段最后一行！！*******\n");
#endif
    }

    return 1; //正常运行
}

/*************************************************
  函数名称：ZAxisFolUpMove(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  Z轴随动运动
  输入参数：
  输出参数：
  返 回 值：-2 指针为空
		   -1 主轴停转（特殊情况）不规划，不发脉冲
			0 加速阶段
			1 随动阶段   （随动阶段要是  新增距离!=当前速度  会进入加速(追赶)阶段)
			2 减速阶段	（如果有）
			3 完成阶段	（与终点重合）

  备    注： 2022/8/8  针对编码器正Z正编码器负Z负进行修改
			 当异常反向时，不管正向是否有未走距离，直接清空，只规划上一周期编码器反馈的对应位移
**************************************************/
int32 ZAxisFolUpMove(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];
    double dZAxisNewAddDist; //新增距离
    double dVe;

    int32 ret = -1;

    bool bDecFlag; //减速标志位（是否超越终点）

    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -2;
    }

    //编码器反向检测（这里的反向指的是与规划方向相反）,若反向则根据情况重新规划
    //编码器方向<>正向攻丝编码器方向（主轴正向攻丝方向）          当前向量<>正向攻丝向量
    //分为以下四个情况：
    //1.编码器反，上一周期规划的也是反向
    //2.编码器反，上一周期规划的是正（重新规划）
    //3.编码器正，上一周期规划的也是正向
    //4.编码器正，上一周期规划的是反（重新规划）

    if (pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir &&
        checkSameVector(pSG33, pSG33->G33StartVector, pSG33->SStartVector)) //情况2
    {
#if G33ProcLog_DEBUG
        printf("*****************随动攻丝阶段异常反向、重新进行预处理！！****************\n");
#endif
        //情况2：编码器反，上一周期规划的是正（重新规划）
        //重新规划
        //不可超越攻牙起点

        //①当前速度清零
        pSG33->dCurrentSpeed = 0;

        //②起点终点，向量，距离
        pSG33->SEndPoint = pSG33->G33StartPoint;
        //获取当前坐标作为起点
        getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

        //计算距离和向量
        calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                pSG33->ucRelaVal);

        //③目标距离，已走距离，剩余距离清零
        pSG33->SPlanData.dPreDist = 0;
        pSG33->SPlanData.dTargetDist = 0;
        pSG33->SPlanData.dCurLeftDist = 0;

        //④不可超越终点，有减速
        bDecFlag = TRUE;
    }
    else if (pSpindleEncoder->cEncoderDir == pSG33->cTappingSpindleDir &&
             !checkSameVector(pSG33, pSG33->G33StartVector, pSG33->SStartVector)) //情况4
    {
#if G33ProcLog_DEBUG
        printf("*****************随动攻丝阶段异常反向、重新进行预处理！！****************\n");
#endif
        //情况4：编码器正，上一周期规划的是反（重新规划）
        //重新规划
        //可以超越攻牙终点

        //①当前速度清零
        pSG33->dCurrentSpeed = 0;

        //②起点终点，向量，距离
        pSG33->SEndPoint = pSG33->G33EndPoint;
        //获取当前坐标作为起点
        getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);
        //计算距离和向量
        calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                pSG33->ucRelaVal);

        //③目标距离，已走距离，剩余距离清零
        pSG33->SPlanData.dPreDist = 0;
        pSG33->SPlanData.dTargetDist = 0;
        pSG33->SPlanData.dCurLeftDist = 0;

        //④不可超越终点，不减速
        bDecFlag = FALSE;
    }
    else if (pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir &&
             !checkSameVector(pSG33, pSG33->G33StartVector, pSG33->SStartVector))
    {
        //情况1：编码器反，上一周期规划的也是反向
        //不用重新规划
        //不可以超越攻牙起点，减速
        bDecFlag = TRUE;
    }
    else
    {
        //情况3：编码器正，上一周期规划的也是正向
        //不用重新规划
        //可以超越攻牙终点，不减速
        bDecFlag = FALSE;
    }

    //计算上一周期主轴编码器增量对应的z轴位移
    dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
    dVe = dZAxisNewAddDist / pSG33->SPlanData.dTc;
    //随动过程（包括减速阶段）新增距离为0，即置零当前速度，直接返回，不规划，不发脉冲
    if (isZero(dVe))
    {
#if G33ProcLog_DEBUG
        printf("*****************上周期主轴编码器差值为0（主轴未转），不输出脉冲！****************\n");
#endif
        pSG33->dCurrentSpeed = 0;
        return ret;
    }
    ret = (int32)newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe,
                                         bDecFlag, g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed);

    //运动输出
    newAddDistOutPut(pSG33);

    return ret;
}

/*************************************************
  zxr: 正常停留不累计反馈超时，运动阶段按本通道计数保护
  函数名称：ZAxisNotExactlyFolUpMove(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  Z轴不完全随动运动
  输入参数：pSG33 G33结构体指针
			pSpindleEncoder 主轴编码器结构体指针
  输出参数：
  返 回 值：-3 反馈超时或ret初始状态
		   -2 指针为空
		   -1 主轴停转（特殊情况）不规划，不发脉冲
			0 加速阶段
			1 随动阶段   （随动阶段要是  新增距离!=当前速度  会进入加速(追赶)阶段)
			2 减速阶段	（如果有）
			3 完成阶段	（与终点重合）

  备    注： 2022/8/8  针对编码器正Z正编码器负Z负进行修改
			 当异常反向时，要管正向是否有未走距离，不能直接反向
**************************************************/
int32 ZAxisNotExactlyFolUpMove(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];
    double dZAxisNewAddDist; //新增距离
    double dVe;

    int32 ret = -3;
    bool bDecFlag; //减速标志位（是否超越终点）

    if (pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -2;
    }

    //计算上一周期主轴编码器增量对应的z轴位移
    dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
    dVe = dZAxisNewAddDist / pSG33->SPlanData.dTc;

    //随动过程（包括减速阶段）新增距离为0，即置零当前速度，直接返回，不规划，不发脉冲
    if (isZero(dVe))
    {
#if G33ProcLog_DEBUG
        printf("*****************上周期主轴编码器差值为0（主轴未转），不输出脉冲！****************\n");
#endif
        ret = -1;
        pSG33->dCurrentSpeed = 0;
        // zxr: 计划停留允许主轴静止，仅清零本通道计数；有反馈时仍按原流程随动
        if (pSG33->G33MotionState == PAUSE_BEFORE_REVERSAL)
        {
            pSG33->unFollowTimeOut = 0;
            return ret;
        }
        // zxr: 需要主轴运动的阶段仍按原阈值累计本通道连续无反馈次数
        pSG33->unFollowTimeOut++;
        if (pSG33->unFollowTimeOut > G33_TIMERANGE)
        {
            parseRunningError(pSG33->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM154, 9999);
            return -3;
        }
        return ret;
    }
    else
    {
        // zxr: 本通道恢复反馈后清零，不影响其他通道的故障计时
        pSG33->unFollowTimeOut = 0;
    }

    //编码器反向检测（这里的反向指的是与规划方向相反）,若反向则根据情况重新规划
    //编码器方向<>正向攻丝编码器方向（主轴正向攻丝方向）          当前向量<>正向攻丝向量
    //分为以下四个情况：
    //1.编码器反，上一周期规划的也是反向
    //2.编码器反，上一周期规划的是正
    //3.编码器正，上一周期规划的也是正向
    //4.编码器正，上一周期规划的是反

    if (pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir &&
        checkSameVector(pSG33, pSG33->G33StartVector, pSG33->SStartVector)) //情况2
    {

        //情况2：编码器反，上一周期规划的是正
        //本周期新增位移和剩余位移叠加

        //判断是否有剩余距离
        if (pSG33->SPlanData.dCurLeftDist > dZAxisNewAddDist) //残余距离> 新增位移 不需重新规划
        {
#if G33ProcLog_DEBUG
            printf("*****************主轴减速并退刀阶段异常反向、且不能抵消剩余位移，不反向！！****************\n");
#endif
            pSG33->SPlanData.dCurLeftDist -= dZAxisNewAddDist;
            pSG33->SPlanData.dTargetDist -= dZAxisNewAddDist;

            //取零
            dZAxisNewAddDist = 0;

            //可超越终点
            bDecFlag = FALSE;
        }
        else //需要反向
        {
#if G33ProcLog_DEBUG
            printf("*****************主轴减速并退刀阶段异常反向、新增位移大于剩余位移，反向！！****************\n");
#endif
            //①
            dZAxisNewAddDist -= pSG33->SPlanData.dCurLeftDist;

            //②当前速度清零
            pSG33->dCurrentSpeed = 0;

            //③起点终点，向量，距离

            pSG33->SEndPoint = pSG33->G33StartPoint;
            //获取当前坐标作为起点
            getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

            //计算距离和向量
            calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                    &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                    pSG33->ucRelaVal);

            //④目标距离，已走距离，剩余距离清零
            pSG33->SPlanData.dPreDist = 0;
            pSG33->SPlanData.dTargetDist = 0;
            pSG33->SPlanData.dCurLeftDist = 0;

            //⑤不可超越终点，有减速
            bDecFlag = TRUE;
        }
    }
    else if (pSpindleEncoder->cEncoderDir == pSG33->cTappingSpindleDir &&
             !checkSameVector(pSG33, pSG33->G33StartVector, pSG33->SStartVector)) //情况4
    {
        //情况4：编码器正，上一周期规划的是反（重新规划）
        //判断是否有剩余距离
        if (pSG33->SPlanData.dCurLeftDist > dZAxisNewAddDist) //残余距离> 新增位移 不需重新规划
        {
#if G33ProcLog_DEBUG
            printf("*****************主轴减速并退刀阶段异常反向、且不能抵消剩余位移，不反向！！****************\n");
#endif
            pSG33->SPlanData.dCurLeftDist -= dZAxisNewAddDist;
            pSG33->SPlanData.dTargetDist -= dZAxisNewAddDist;

            //取零
            dZAxisNewAddDist = 0;

            //有减速
            bDecFlag = TRUE;
        }
        else //需要重新规划
        {
#if G33ProcLog_DEBUG
            printf("*****************主轴减速并退刀阶段异常反向、新增位移大于剩余位移，反向！！****************\n");
#endif
            double distance1, distance2;
            //①
            dZAxisNewAddDist -= pSG33->SPlanData.dCurLeftDist;

            //②当前速度清零
            pSG33->dCurrentSpeed = 0;

            //③起点终点，向量，距离
            getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

            //核对当前是否越过G33终点
            distance1 =
                sqrt(pow(g_dRealTimePointArray[ucMapArray[ucG33Axis1]] - pSG33->G33StartPoint.dArray[ucG33Axis1], 2) +
                     pow(g_dRealTimePointArray[ucMapArray[ucG33Axis2]] - pSG33->G33StartPoint.dArray[ucG33Axis2], 2));

            distance2 = sqrt(pow(pSG33->G33EndPoint.dArray[ucG33Axis1] - pSG33->G33StartPoint.dArray[ucG33Axis1], 2) +
                             pow(pSG33->G33EndPoint.dArray[ucG33Axis2] - pSG33->G33StartPoint.dArray[ucG33Axis2], 2));

            if (distance1 > distance2) //越过G33终点
            {
#if G33ProcLog_DEBUG
                printf("*****************越过G33终点，另算一个更远的终点规划！****************\n");
#endif
                //计算出一个更远的目标点
                pSG33->SEndPoint = calcPoint(pSG33, pSG33->SStartPoint, pSG33->G33StartVector, 10);
            }
            else
            {
#if G33ProcLog_DEBUG
                printf("*****************未越过G33终点，以G33EndPoint为终点规划！****************\n");
#endif
                pSG33->SEndPoint = pSG33->G33EndPoint;
            }

            calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                    &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                    pSG33->ucRelaVal);

            //④目标距离，已走距离，剩余距离清零
            pSG33->SPlanData.dPreDist = 0;
            pSG33->SPlanData.dTargetDist = 0;
            pSG33->SPlanData.dCurLeftDist = 0;

            //⑤可超越终点，不减速
            bDecFlag = FALSE;
        }
    }
    else if (pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir &&
             !checkSameVector(pSG33, pSG33->G33StartVector, pSG33->SStartVector))
    {
        //情况1：编码器反，上一周期规划的也是反向
        //不用重新规划
        //不可以超越攻牙起点，减速
        bDecFlag = TRUE;
    }
    else
    {
        //情况3：编码器正，上一周期规划的也是正向
        //不用重新规划
        //可以超越攻牙终点，不减速
        bDecFlag = FALSE;
    }

    if (bDecFlag)
    {
        ret = (int32)newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe,
                                             bDecFlag, g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed);
    }
    else
    {
        ret = (int32)newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe,
                                             bDecFlag, g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed);
    }

    //运动输出
    newAddDistOutPut(pSG33);

    return ret;
}

/*************************************************
  函数名称：calcSpindleDecZAxicSurpassLength(struct SSingleAxis* pSSpindle, struct G33* pSG33)
  函数功能  估算主轴减速Z轴超出距离
  输入参数：
  输出参数：
  返 回 值：超出距离
  备    注：
**************************************************/

double calcSpindleDecZAxicSurpassLength(struct _SSpindle* pSSpindle, struct G33* pSG33)
{
    double dDec;
    double dVs, dVe;
    uint32 unDecTime;
    double dDecSFactor;
    double dTc = pSG33->SPlanData.dTc;
    double dEstiSurpassLength;                                 //估算的主轴减速到停时Z轴超过减速点的距离
    const uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组

    if (pSSpindle == NULL || pSG33 == NULL)
    {
        return 0;
    }
    int8 cG33RotateAxis = pSG33->G33RotateAxis;
    //注：多通道版本旋转轴参数单位是(°/ms)
    //1.计算超出长度(后期封装为函数)
    dDec = getSingleAxisDec(cG33RotateAxis, g_SSystemPara.SAxisPara[ucMapArray[cG33RotateAxis]].ucDecVelPlanType,
                            ucMapArray); //从系统参数获取减速度(上一段加速度重算后与系统参数不一致)
    dVs = getG97AbsSpindleSpeed(pSSpindle) * 360.0;
    dVe = g_SSystemPara.SAxisPara[ucMapArray[cG33RotateAxis]].dVeerDeltaVMax;

    if (dDec <= 0)
    {
        parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
        return 0;
    }

    //根据主轴减速曲线类型获取位移系数
    dDecSFactor = getFactor(g_SSystemPara.SAxisPara[ucMapArray[cG33RotateAxis]].ucDecVelPlanType);

    //减速周期
    unDecTime = (int32)ceil((pSSpindle->pSSpindleAxis->SUserData.ucDecVelPlanType == VEL_PLAN_V2_CURVE)
                                ? fabs(dDecSFactor * (dVs - dVe) / (dDec * dTc))
                                : fabs(2 * dDecSFactor * (dVs - dVe) / (dDec * dTc)));

    if (unDecTime != 0)
    {
        //重算减速度
        dDec = (pSSpindle->pSSpindleAxis->SUserData.ucDecVelPlanType == VEL_PLAN_V2_CURVE)
                   ? fabs(dDecSFactor * (dVs - dVe) / (unDecTime * dTc))
                   : fabs(2 * dDecSFactor * (dVs - dVe) / (unDecTime * dTc)); //重算减速度
        dEstiSurpassLength = dDecSFactor * fabs(pow(dVs, 2) - pow(dVe, 2)) / dDec / 360.0 * pSG33->dF *
                             g_SSystemPara.dPercentageOfG33EnterLeadCorrection;
    }
    else
    {
        dEstiSurpassLength = 0;
    }

    return dEstiSurpassLength;
}

/*************************************************
  函数名称：calcSpindleDecPointPreDist(struct SSingleAxis* pSSpindle, struct G33* pSG33)
  函数功能  计算主轴反转规划时相对G33起点要走的距离，此距离保存在G33结构体中作为一个成员
  输入参数：
  输出参数：
  返 回 值：-2: 指针为空
		   -1:不需要重算减速点
			1:需要重算减速点
  备    注：Z轴随动攻丝运动第一个周期，即预处理时调用
**************************************************/
int8 calcSpindleDecPointPreDist(struct _SSpindle* pSSpindle, struct G33* pSG33)
{
    if (pSSpindle == NULL || pSG33 == NULL)
    {
        return -2;
    }

#if G33ProcLog_DEBUG
    printf("*****************开始计算和主轴反转规划时减速点******************\n");
#endif
    //根据系统参数中G33重算减速点标志位决定何时主轴反转规划
    if (g_SSystemPara.bG33RecalDecPointFlag)
    {
        double dEstiSurpassLength; //估算的主轴减速到停时Z轴超过减速点的距离
        //1.计算超出长度
        dEstiSurpassLength = calcSpindleDecZAxicSurpassLength(pSSpindle, pSG33);

        //2.根据情况更改减速点处位移
        if (dEstiSurpassLength < 1) //超出距离小于1，不改变减速点位置
        {
            pSG33->dSpindleDecPointPreDist = pSG33->SPlanData.dDistance;
        }
        else
        {
            pSG33->dSpindleDecPointPreDist = pSG33->SPlanData.dDistance - dEstiSurpassLength + 1;
        }

#if G33ProcLog_DEBUG
        printf("*****************需要重算减速点！！！************\n");
        printf("*****************Z轴计算超出距离：%lf\tG33理论长度：%lf\t规划后减速点PreDist:%lf****************\n",
               dEstiSurpassLength, pSG33->SPlanData.dDistance, pSG33->dSpindleDecPointPreDist);
#endif
        return 1;
    }
    else //不重算减速点，则在Z轴运行超过理论终点时主轴反转规划
    {
#if G33ProcLog_DEBUG
        printf("*****************不重算减速点************\n");
#endif
        pSG33->dSpindleDecPointPreDist = pSG33->SPlanData.dDistance;

        return -1;
    }
}

/*************************************************
  函数名称：judgeZAxisAcrossEndPoint(struct G33* pSG33)
  函数功能  判断Z轴越过主轴反转规划标志点
  输入参数：
  输出参数：
  返 回 值：
  备    注：Z轴随动攻丝运动函数用
			2022/8/8   由于存在异常反向，不再用SPlanData.dPreDist 与 pSG33->dSpindleDecPointPreDist对比
					   改为计算当前点和G33起点距离，距离与pSG33->dSpindleDecPointPreDist 进行对比
**************************************************/
bool judgeZAxisAcrossEndPoint(struct G33* pSG33)
{

    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];

    //计算当前点距离G33起点距离
    double distance;
    distance = sqrt(pow(g_dRealTimePointArray[ucMapArray[ucG33Axis1]] - pSG33->G33StartPoint.dArray[ucG33Axis1], 2) +
                    pow(g_dRealTimePointArray[ucMapArray[ucG33Axis2]] - pSG33->G33StartPoint.dArray[ucG33Axis2], 2));

    if (distance > pSG33->dSpindleDecPointPreDist || isZero(distance - pSG33->dSpindleDecPointPreDist))
    {
        return TRUE;
    }
    return FALSE;
}

/*************************************************
  函数名称：motionZAxisFolUpTapping()
  函数功能  Z轴随动攻丝（此阶段主轴匀速，Z轴随动）
  输入参数：
  输出参数：
  返 回 值：-1 指针为空
			1 指针非空
  备    注：Z轴随动攻丝运动函数
**************************************************/
int8 motionZAxisFolUpTapping(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    if (pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    //判断
    if (pSG33->bPreFinishedFlag == FALSE)
    {
        //1.进入随动状态后第一个周期将，加速度变为5倍
        pSG33->dG33Acc = 5.0 * g_SSystemPara.dSysPlanTMaxAcc;
        pSG33->dG33Dec = 5.0 * g_SSystemPara.dSysPlanTMaxDec;

        //2.根据系统参数中G33重算减速点标志位，确定主轴反转时的PreDist值，即减速点（无需求出坐标点，只用求出距起点的长度即可）
        //然后下方judgeZAxisAcrossEndPoint()函数根据此值与PreDist值比较，确定是否进入主轴反转规划（ZAXIS_FOL_UP_DEC_AND_RETURN）阶段
        calcSpindleDecPointPreDist(pSSpindle, pSG33);

        //3.判断进入随动攻丝第一个周期时是否超过减速度点
        //当超过理论终点时更改运动状态为攻丝后主轴减速及退刀,并运行该状态运动函数
        if (judgeZAxisAcrossEndPoint(pSG33))
        {
            //直接进入主轴反转前减速停阶段
            pSG33->G33MotionState = SPINDLE_DEC_TO_STOP_BEFORE_REV;
            motionSpindleDecToStopBeforeRev(pSSpindle, pSG33, pSpindleEncoder);
            return 1;
        }
        pSG33->bPreFinishedFlag = TRUE;
    }

    //随动
    //ZAxisFolUpMove(pSG33, pSpindleEncoder);
    ZAxisNotExactlyFolUpMove(pSG33, pSpindleEncoder);

    //当超过理论终点时更改运动状态为攻丝后主轴减速及退刀
    //(备注：一旦超越减速点，就会进入下一阶段，所以不用担心，由于来回正反向导致反复越过减速点导致的多次设置主轴减速反向加速的情况)
    if (judgeZAxisAcrossEndPoint(pSG33))
    {
        pSG33->G33MotionState = SPINDLE_DEC_TO_STOP_BEFORE_REV; //主轴反转前减速停
        pSG33->bPreFinishedFlag = FALSE;                        //置FALSE表示下一阶段未预处理
    }
    return 1;
}

/*************************************************
  函数名称：int8 motionSpindleDecToStopBeforeRev(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能：主轴反转前减速停
  输入参数：pSG33
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 motionSpindleDecToStopBeforeRev(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    if (pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    if (pSG33->bPreFinishedFlag == FALSE)
    {
#if G33ProcLog_DEBUG
        printf("********主轴反转前减速停开始**********\n");
#endif

        //struct _SSpindle* pG33RotateAxis = (pSG33->G33RotateAxis == AXIS_A) ? (&pSG33->pSChannel->SSecondSpindle) : (&pSG33->pSChannel->SSpindle);
        spindleCommonStop(pSSpindle);

        //spindleCommonStopAPI(pSG33->pSChannel->ucChannelNum);	陈敏聪 5/15

        //将主轴预处理状态位置TRUE
        pSG33->bPreFinishedFlag = TRUE;
    }

    //主轴减速过程中Z轴随动   2023.6.8郭强
    ZAxisNotExactlyFolUpMove(pSG33, pSpindleEncoder);

    return 1;
}

/*************************************************
  函数名称：int8 spindleRevCompPlan(struct G33* pSG33)
  函数功能：主轴反转前暂停
  输入参数：
  输出参数：
  返 回 值：    -1指针为空
				1暂停完成
				2暂停未完成
  备    注：
**************************************************/
int8 motionPauseBeforeSpindleReversal(struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    double dZAxisNewAddDist; //新增距离

    if (pSG33 == NULL)
    {
        return -1;
    }

    if (pSG33->bPreFinishedFlag == FALSE)
    {
#if G33ProcLog_DEBUG
        printf("*****************************************************\n");
        printf("*****************主轴反转前暂停开始！****************\n");
        printf("*****************************************************\n");
#endif
        //将主轴预处理状态位置TRUE
        pSG33->bPreFinishedFlag = TRUE;
    }

    //为防止暂停期间主轴仍然在“动”，也加入编码器方向跟随运动函数
    ZAxisNotExactlyFolUpMove(pSG33, pSpindleEncoder);

    //递减暂停时间
    pSG33->unG33SpindlePauseTimeBeforeRev--;

    if (pSG33->unG33SpindlePauseTimeBeforeRev == 0)
    {
#if G33ProcLog_DEBUG
        printf("********************************************************************************\n");
        printf("*****************主轴反转前暂停结束！更改G33状态为主轴反转退刀！****************\n");
        printf("********************************************************************************\n");
#endif
        pSG33->G33MotionState = SPINDLE_REVERSE_RETURN; //更改G33状态为主轴反转退刀
        pSG33->bPreFinishedFlag = FALSE;
        pSG33->unG33SpindlePauseTimeBeforeRev =
            g_SSystemPara.unG33SpindlePauseTimeBeforeRev; //还原G33主轴反转前暂停时间
        return 1;
    }

    return 2;
}

/*************************************************
  函数名称：int8 spindleRevCompPlan(struct G33* pSG33)
  函数功能 主轴反转补偿规划（初始化结构体+速度规划和平摊）
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 spindleRevCompPlan(struct G33* pSG33)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];

    if (pSG33 == NULL)
    {
        return -1;
    }
    //走普通复合运动规划，带余量分摊
    //初始化
    //1.初速度2.末速度3.最大速度4.distance  5.tagetDist 6.preDist 7.加速减速曲线类型 8加速度，减速度 9平摊类型 10起点向量 11起点
    pSG33->SPlanData.dVs = g_SSystemPara.dRevClearanceCompVs;
    pSG33->SPlanData.dVe = g_SSystemPara.dRevClearanceCompVs;
    pSG33->SPlanData.dVmax = g_SSystemPara.dRevClearanceCompVmax;

    if (!isZero(pSG33->SPlanData.dCurLeftDist)) //补偿阶段主轴停止重新规划
    {
        pSG33->SPlanData.dDistance = pSG33->SPlanData.dTargetDist = pSG33->SPlanData.dCurLeftDist;
    }
    else
    {
        pSG33->SPlanData.dDistance = pSG33->SPlanData.dTargetDist = pSG33->SPlanData.dCurLeftDist =
            g_SSystemPara.dG33RevCompDistance;
    }

    pSG33->SPlanData.dPreDist = 0; //已走位移
    getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);

    //取反方向向量
    pSG33->SStartVector.dArray[ucG33Axis1] = -pSG33->G33StartVector.dArray[ucG33Axis1];
    pSG33->SStartVector.dArray[ucG33Axis2] = -pSG33->G33StartVector.dArray[ucG33Axis2];

    //最大加减速度和加减速类型
    //最大加速度、减速度
    pSG33->SPlanData.dAcc = g_SSystemPara.dRevClearanceCompAcc;
    pSG33->SPlanData.dDec = g_SSystemPara.dRevClearanceCompDec;

    //加减速度速度曲线类型
    pSG33->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;
    pSG33->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;

    //平摊类型
    pSG33->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE; //平摊类型梯形平摊

    //速度规划+平摊
    preparePlanningAndDistReAvg(&pSG33->SPlanData, TRUE); //速度规划+平摊
    pSG33->SPlanData.unCurrentT = 0;

    return 1;
}

/*************************************************
  函数名称：spindleRevCompMoveOutPut(struct G33* pSG33)
  函数功能 主轴反转补偿运动
  输入参数：struct G33* pSG33

  输出参数：
  返 回 值： -1	输入错误
			 0	插补完成
			 1	插补中
  备    注：
**************************************************/
int8 spindleRevCompMoveOutPut(struct G33* pSG33)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    double dDeltaL;                                      //相对起点的距离 (或弧长)
    SPointDouble STargetPoint;                           //当前点的理论绝对坐标

    if (pSG33 == NULL) //空指针  不运行
    {
        return -1;
    }

    //运行时间增加
    pSG33->SPlanData.unCurrentT += 1;
    //获取currentT时刻的弧长(圆弧）或 线段长（直线）
    dDeltaL = nextstepPlanningAndDistReAvg(pSG33->SPlanData.unCurrentT, &pSG33->SPlanData);

    //计算理论坐标点
    //STargetPoint = calG33ExactInterpolationPoint(dDeltaL, pSG33);

    //outPutAbsPoint(STargetPoint);  //利用理论坐标生成SRun结构体数据
    calculateLinearInterpolationPoint((double*)&pSG33->SStartPoint, (double*)&pSG33->SStartVector, dDeltaL,
                                      (double*)&STargetPoint, AXIS_NUM_MAX);

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE
    //利用理论坐标生成SRun结构体数据
    outPutAbsPointWithScrewComp(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, NULL,
                                &pSG33->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
    outPutAbsPointWithBilateralScrewComp(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray,
                                         NULL, &pSG33->ucValStatus);
#else
    //利用理论坐标生成SRun结构体数据
    outPutAbsPoint(&pSG33->pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, ucMapArray, NULL,
                   &pSG33->ucValStatus);
#endif

    //计算当前速度
    pSG33->dCurrentSpeed =
        fabs(pSG33->SPlanData.dCurLeftDist - (pSG33->SPlanData.dTargetDist - dDeltaL)) / pSG33->SPlanData.dTc;

    //计算   !理论! 未走距离
    pSG33->SPlanData.dCurLeftDist = pSG33->SPlanData.dTargetDist - dDeltaL;

    if (pSG33->SPlanData.dCurLeftDist <= EPSINON) //运行结束条件
    {
        return 0;
    }
    return 1;
}

/*************************************************
  函数名称：calcG33SurpassLength(struct SSingleAxis* pSSpindle, struct G33* pSG33)
  函数功能  仿真计算G33超出距离（测试用）
  输入参数：
  输出参数：
  返 回 值：
  备    注：分别计算实际超出值，理论超出值

		   当判断为主轴反转第一个周期时，调用该程序
**************************************************/
void calcG33SurpassLength(struct _SSpindle* pSSpindle, struct G33* pSG33)
{
    double dActualSurpassLength, dEstiSurpassLength;

    dActualSurpassLength = fabs(g_dRealTimePointArray[AXIS_Z] - pSG33->G33EndPoint.dArray[AXIS_Z]);

    dEstiSurpassLength = calcSpindleDecZAxicSurpassLength(pSSpindle, pSG33);

#if G33ProcLog_DEBUG
    printf("*****************Z轴实际超出距离：%lf\tdEstiSurpassLength=%lf\t****************\n", dActualSurpassLength,
           dEstiSurpassLength);
#endif
}

/*************************************************
  函数名称：calcPoint(SPointDouble SStartPoint, SVector SStartVector, double dLength)
  函数功能：根据长度起点方向向量计算目标点
  输入参数： SStartPoint
			SStartVector
			dLength
  输出参数：
  返 回 值：目标点
  备    注：
**************************************************/
SPointDouble calcPoint(struct G33* pSG33, SPointDouble SStartPoint, SVector SStartVector, double dLength)
{
    SPointDouble STargetPoint = SStartPoint;
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];

    STargetPoint.dArray[ucG33Axis1] = SStartPoint.dArray[ucG33Axis1] + dLength * SStartVector.dArray[ucG33Axis1];
    STargetPoint.dArray[ucG33Axis2] = SStartPoint.dArray[ucG33Axis2] + dLength * SStartVector.dArray[ucG33Axis2];
    return STargetPoint;
}

/*************************************************
  函数名称：needReveComMove()
  函数功能
  输入参数： pSG33
			pSpindleEncoder
  输出参数：
  返 回 值：
		    -3:超时
			-2:指针为空
			-1:主轴停止，不动
			1：未完成退刀
			3：退刀到起点

  备    注：
**************************************************/
int8 needReveComMove(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    uint8* ucMapArray = pSG33->pSChannel->pMapAxisArray; //通道轴映射数组
    uint8 ucG33Axis1 = pSG33->ucAxis[0];
    uint8 ucG33Axis2 = pSG33->ucAxis[1];
    static uint32 unTimeOut = 0; //记录主轴异常连续次数 // 2026.5.9
    if (pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -2;
    }

    int32 ret = 1;

    //识别反转第一个周期进行T型带分摊的反转补偿规划
    if ((pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir) &&
        (pSG33->bSpindleRevBegin == FALSE)) //反转第一个周期，此时该标志位 还未改变为TRUE
    {
#if G33ProcLog_DEBUG
        printf("*****************反转第一个周期，需要补偿！****************\n");
        printf("*****************反转补偿开始！****************\n");
#endif
        //计算超出距离（测试）
        //calcG33SurpassLength(pSSpindle, pSG33);

        //清零当前速度（2022/8/8）
        pSG33->dCurrentSpeed = 0;

        //反转补偿规划
        spindleRevCompPlan(pSG33);

        //更改主轴反转标志位
        pSG33->bSpindleRevBegin = TRUE;
    }

    //在反转补偿期间输出并累积编码器
    if (pSG33->bSpindleRevBegin == TRUE && pSG33->bSpindleRevCompEnd == FALSE) //反转补偿开始且未结束
    {
        int8 retRevCom;
        double dZAxisNewAddDist; //新增距离
        double dVe;

        //计算上一周期主轴编码器增量对应的z轴位移
        dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
        dVe = dZAxisNewAddDist / pSG33->SPlanData.dTc;

        //1.对主轴反转补偿期间Z轴累积距离进行计算和记录
        //（2022/8/8  设定累积距离正值方向为退刀方向，负值为攻丝方向）
        if (pSpindleEncoder->cEncoderDir == pSG33->cTappingSpindleDir)
        {
            dZAxisNewAddDist = -1.0 * dZAxisNewAddDist;
        }

        pSG33->dSpindleRevCompZAxisCumulaDis += dZAxisNewAddDist;

        //2.若主轴停止，不输出，置零当前速度 、将反转开始置FALSE，下一周期就会重新进行补偿规划
        if (isZero(dVe))
        {
#if G33ProcLog_DEBUG
            printf("*****************上周期主轴编码器差值为0（主轴未转），不输出脉冲！****************\n");
#endif
            pSG33->dCurrentSpeed = 0;
            pSG33->bSpindleRevBegin = FALSE;
            unTimeOut++;                   //记录异常次数
            if (unTimeOut > G33_TIMERANGE) //异常连续次数超过1000次，主轴异常，停止
            {
                parseRunningError(pSG33->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM154, 9999);
                return -3;
            }
            return -1;
        }
        else
        {
            unTimeOut = 0; //异常次数清零
        }

        //3.补偿阶段输出
        retRevCom = spindleRevCompMoveOutPut(pSG33);

        //4.判断是否补偿完毕
        //if ( g_SPreProcess.ucInpStatus == INP_FINISHED)//对反转补偿位移补偿完毕进行判断()老方案
        if (retRevCom == 0) //对反转补偿位移补偿完毕进行判断
        {
#if G33ProcLog_DEBUG
            printf("*****************反转补偿结束！反转补偿累积Z轴距离:%lf****************\n",
                   pSG33->dSpindleRevCompZAxisCumulaDis);
#endif
            //反转补偿完毕标志位置TRUE
            pSG33->bSpindleRevCompEnd = TRUE;
        }

        return ret;
    }

    //识别反转补偿完第一个周期，将累积值加入新增规划
    //注：苏总要求在反转补偿完后，根据累积值得方向走，加入后边反向了，不能直接反向，还要走完未走的距离
    //主轴编码器不会方向跳崖式突变，是平滑的过程
    if (pSG33->bSpindleRevCompEnd == TRUE && pSG33->bSpindleRevCompEndFirstTc == TRUE)
    {
#if G33ProcLog_DEBUG
        printf("*****************反转补偿完成后第一个周期！****************\n");
#endif
        double dZAxisNewAddDist; //新增距离
        double dVe;

        //将标志位改为FALSE
        pSG33->bSpindleRevCompEndFirstTc = FALSE;

        //计算上一周期主轴编码器增量对应的z轴位移
        dZAxisNewAddDist = calcZAxisNewAddDist(pSG33, pSpindleEncoder);
        dVe = dZAxisNewAddDist / pSG33->SPlanData.dTc;

        //1.对主轴反转补偿期间Z轴累积距离进行计算和记录
        //（2022/8/8  设定累积距离正值方向为退刀方向，负值为攻丝方向）
        if (pSpindleEncoder->cEncoderDir == pSG33->cTappingSpindleDir)
        {
            dZAxisNewAddDist = -1.0 * dZAxisNewAddDist;
        }

        //反转补偿期间所有累计值
        dZAxisNewAddDist = pSG33->dSpindleRevCompZAxisCumulaDis + dZAxisNewAddDist;

        //查看累积距离正负
        //1.累积距离为正(不能超过起点)
        if (dZAxisNewAddDist > 0 || isZero(dZAxisNewAddDist))
        {
#if G33ProcLog_DEBUG
            printf("*****************补偿阶段累积距离为正，退刀方向规划！****************\n");
#endif
            getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);
            pSG33->SEndPoint = pSG33->G33StartPoint;
            //calVectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SPlanData.dDistance, &pSG33->SStartVector);
            calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                    &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                    pSG33->ucRelaVal);
            pSG33->SPlanData.dPreDist = 0;
            pSG33->SPlanData.dTargetDist = 0;
            pSG33->SPlanData.dCurLeftDist = 0;

            ret = newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe,
                                          TRUE, g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed);
        }
        else //2.累积距离为负（可以超过终点）
        {
#if G33ProcLog_DEBUG
            printf("*****************补偿阶段累积距离为负，向攻牙方向规划！****************\n");
#endif
            double distance1, distance2;
            dZAxisNewAddDist = fabs(dZAxisNewAddDist); //取正新增位移
            pSG33->dCurrentSpeed = 0;
            getRealTimePoint((double*)&pSG33->SStartPoint, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus);
            //这里的规划终点不能直接设置为G33的终点，
            //存在异常情况：补偿累积位移为负，而此时实时点越过了G33终点，此时若选择G33终点为规划终点，走的方向就是错的
            //解决方案为：  规划终点     《----     当前点     《----     G33终点
            //核对当前是否越过G33终点
            distance1 =
                sqrt(pow(g_dRealTimePointArray[ucMapArray[ucG33Axis1]] - pSG33->G33StartPoint.dArray[ucG33Axis1], 2) +
                     pow(g_dRealTimePointArray[ucMapArray[ucG33Axis2]] - pSG33->G33StartPoint.dArray[ucG33Axis2], 2));

            distance2 = sqrt(pow(pSG33->G33EndPoint.dArray[ucG33Axis1] - pSG33->G33StartPoint.dArray[ucG33Axis1], 2) +
                             pow(pSG33->G33EndPoint.dArray[ucG33Axis2] - pSG33->G33StartPoint.dArray[ucG33Axis2], 2));
            if (distance1 > distance2) //越过G33终点
            {
#if G33ProcLog_DEBUG
                printf("*****************补偿阶段完实时点越过G33终点，另算一个更远的终点规划！****************\n");
#endif
                //计算出一个更远的目标点
                pSG33->SEndPoint = calcPoint(pSG33, pSG33->SStartPoint, pSG33->G33StartVector, 10);
            }
            else
            {
#if G33ProcLog_DEBUG
                printf("*****************补偿阶段完实时点未越过G33终点，以G33EndPoint为终点规划！****************\n");
#endif
                pSG33->SEndPoint = pSG33->G33EndPoint;
            }

            calcG1VectorAndDistance(pSG33->SStartPoint, pSG33->SEndPoint, &pSG33->SStartVector, &pSG33->SStartVector,
                                    &pSG33->SPlanData.dDistance, AXIS_NUM_MAX, ucMapArray, &pSG33->ucValStatus,
                                    pSG33->ucRelaVal);
            pSG33->SPlanData.dPreDist = 0;
            pSG33->SPlanData.dTargetDist = 0;
            pSG33->SPlanData.dCurLeftDist = 0;
            newAddDistanceUnlimited(&pSG33->SPlanData, dZAxisNewAddDist, pSG33->dG33Acc, pSG33->dG33Dec, dVe, FALSE,
                                    g_SSystemPara.dG33AvoidEmptyDistance, pSG33->dCurrentSpeed);
        }
        //运动输出
        newAddDistOutPut(pSG33);

        return ret;
    }

    //其他情况下的运动（第一次反转之前、反转补偿完毕第一个周期之后的过程）
    //反向时不能直接清空未走距离直接反向，要把未走的走完
    //识别退刀方向运动是否到达终点
    ret = ZAxisNotExactlyFolUpMove(pSG33, pSpindleEncoder);

    return ret;
}

/*************************************************
  函数名称：motionZAxisFolUpDecAndReturn()
  函数功能  Z轴随动减速和退刀
  输入参数：
  输出参数：
  返 回 值：-1 指针为空
			1 非空
  备    注：攻丝时Z轴超过理论终点后主轴减速，然后反向加速，Z轴一直随动直到到达减速点
			2022/8/8  异常反向处理
**************************************************/
int8 motionZAxisFolUpDecAndReturn(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    int32 ret = 1;

    if (pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    //主轴反向加速预处理
    if (pSG33->bPreFinishedFlag == FALSE)
    {
        double dReturnSpindleRatio = g_SSystemPara.dG33ReturnSpindleRatio;

        if (isZero(dReturnSpindleRatio)) //主轴反转转速百分比被误设为0，使用原始主轴速度
        {
            dReturnSpindleRatio = 1.0;
        }

        struct _SSpindle* pG33RotateAxis =
            (pSG33->G33RotateAxis == AXIS_A) ? (&pSG33->pSChannel->SSecondSpindle) : (&pSG33->pSChannel->SSpindle);

        //主轴加速预处理
        if (pSG33->cTappingSpindleDir == 1) //M03正转，则启动反转
        {
            setSpindleDir(pG33RotateAxis, ARC_ANTI_CW);
        }
        else //反转则启动正传
        {
            setSpindleDir(pG33RotateAxis, ARC_CW);
        }

        setSpindleSpeed(pG33RotateAxis, pSG33->dG33SpindleSpeed * dReturnSpindleRatio, CMD_SPINDLE_SPEED_CV);

#if G33ProcLog_DEBUG
        printf("*****************Z轴到达理论终点后主轴减速并反向加速开始！****************\n");
#endif
        //将预处理状态位置TRUE
        pSG33->bPreFinishedFlag = TRUE;
    }

    //2022/8/8  异常反向处理（流程有变）
    //识别第一次反转并进行反转补偿
    //if ((pSpindleEncoder->cEncoderDir != pSG33->cTappingSpindleDir) && (pSG33->bSpindleRevBegin == FALSE))//反转第一个周期，此时该标志位 还未改变为TRUE
    if (!isZero(g_SSystemPara.dG33RevCompDistance)) //反转需要补偿
    {
        //在退刀时要查看是否到达G33起点，ret == 3时即到达起点
        ret = needReveComMove(pSSpindle, pSG33, pSpindleEncoder);
    }
    else //反转不需要补偿（这里处理就简单了）
    {
        ret = ZAxisNotExactlyFolUpMove(pSG33, pSpindleEncoder);
    }

    //当Z轴减速完成到达起刀点后更改状态
    if (ret == 3)
    {
        pSG33->G33MotionState = SPINDLE_DEC_TO_STOP_AFTER_ZAXIS_RETURN; //将G33运动状态改变为退刀完成后主轴减速
        pSG33->bPreFinishedFlag = FALSE;                                //置FALSE表示主轴减速未预处理

#if G33ProcLog_DEBUG
        printf("********Z轴退刀回起刀点完成**********\n");
        printf("*****************下一行数据为本阶段最后一行！！*******\n");
#endif
    }

    return 1;
}

/*************************************************
  函数名称：motionSpindleReverseAndReturn
  函数功能  主轴反转退刀
  输入参数：
  输出参数：
  返 回 值：-1 指针为空
			 1 非空
  备    注：
**************************************************/
int8 motionSpindleReverseAndReturn(struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
{
    int32 ret = 1;

    if (pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    //1.主轴反向加速预处理
    if (pSG33->bPreFinishedFlag == FALSE)
    {
        double dReturnSpindleRatio = g_SSystemPara.dG33ReturnSpindleRatio; //2023/5/27郭强

        if (isZero(dReturnSpindleRatio)) //主轴反转转速百分比被误设为0，使用原始主轴速度 2023/5/27郭强
        {
            dReturnSpindleRatio = 1.0;
        }

        struct _SSpindle* pG33RotateAxis =
            (pSG33->G33RotateAxis == AXIS_A) ? (&pSG33->pSChannel->SSecondSpindle) : (&pSG33->pSChannel->SSpindle);
        //主轴加速预处理
        if (pSG33->cTappingSpindleDir == 1) //M03正转，则启动反转
        {
            setSpindleDir(pG33RotateAxis, ARC_ANTI_CW);
        }
        else //反转则启动正传
        {
            setSpindleDir(pG33RotateAxis, ARC_CW);
        }

        setSpindleSpeed(pG33RotateAxis, pSG33->dG33SpindleSpeed * dReturnSpindleRatio, CMD_SPINDLE_SPEED_CV);

        spindleBegin(pG33RotateAxis);

        ////主轴加速预处理				//陈敏聪改 5/15
        //if (pSG33->cTappingSpindleDir == 1)//M03正转，则启动反转
        //{
        //	setSpindleDirAPI(pSG33->pSChannel->ucChannelNum, ARC_ANTI_CW);
        //}
        //else //反转则启动正转
        //{
        //	setSpindleDirAPI(pSG33->pSChannel->ucChannelNum, ARC_CW);
        //}

        //setSpindleSpeedAPI(pSG33->pSChannel->ucChannelNum, pSG33->dG33SpindleSpeed * dReturnSpindleRatio, CMD_SPINDLE_SPEED_CV);

        //spindleBeginAPI(pSG33->pSChannel->ucChannelNum);

#if G33ProcLog_DEBUG
        printf("*****************主轴反向加速开始！****************\n");
#endif
        //将预处理状态位置TRUE
        pSG33->bPreFinishedFlag = TRUE;
    }

    //2.先判断反转补偿距离是否为零
    //2.1不需要反向补偿
    //2.2需要补偿先补偿再随动
    if (!isZero(g_SSystemPara.dG33RevCompDistance)) //反转需要补偿
    {
        //在退刀时要查看是否到达G33起点，ret == 3时即到达起点
        ret = needReveComMove(pSSpindle, pSG33, pSpindleEncoder);
    }
    else //反转不需要补偿(这里处理就简单了)
    {
        ret = ZAxisNotExactlyFolUpMove(pSG33, pSpindleEncoder);
    }
    if (ret == -3)
        return ret; //主轴异常

    //3.退刀完成识别+更改运行状态
    if (ret == 3)
    {
        pSG33->G33MotionState = SPINDLE_DEC_TO_STOP_AFTER_ZAXIS_RETURN; //将G33运动状态改变为退刀完成后主轴减速
        pSG33->bPreFinishedFlag = FALSE;                                //置FALSE表示主轴减速未预处理

#if G33ProcLog_DEBUG
        printf("********Z轴退刀回起刀点完成**********\n");
        printf("*****************下一行数据为本阶段最后一行！！*******\n");
#endif
    }

    return 1;
}

/*************************************************
  函数名称：motionSpindleDecToStopAfterZAxisReturn(struct G33* pSG33)
  函数功能  退刀完成后主轴减速
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 motionSpindleDecToStopAfterZAxisReturn(struct G33* pSG33)
{
    if (pSG33 == NULL)
    {
        return -1;
    }

    if (pSG33->bPreFinishedFlag == FALSE)
    {
#if G33ProcLog_DEBUG
        printf("********退刀后主轴开始减速**********\n");
#endif

        struct _SSpindle* pG33RotateAxis =
            (pSG33->G33RotateAxis == AXIS_A) ? (&pSG33->pSChannel->SSecondSpindle) : (&pSG33->pSChannel->SSpindle);
        //spindleCommonStop(pG33RotateAxis); //修改为根据系统参数确定准停还是减速停 2026.5.21
        spindleStopAPI(pG33RotateAxis, NULL, FALSE);
        //将主轴预处理状态位置TRUE
        pSG33->bPreFinishedFlag = TRUE;
    }

    return 1;
}

//G33运行过程中的复位初始化
/*************************************************
  zxr: 同步通道接口说明，复位及报警收尾清零随动计数
  函数名称：G33ResetInit(struct SChannel* pSChannel)
  函数功能  G33运行过程中的复位初始化
  输入参数：pSChannel 通道结构体指针
  输出参数：
  返 回 值： -1 指针为空
			 1 指针不为空,为G33复位
			 2 不为G33复位
  备    注：
**************************************************/
int8 G33ResetInit(struct SChannel* pSChannel)
{
    if (pSChannel == NULL)
    {
        return -1;
    }
    // zxr: 复位或报警停止收尾时清零，即使当前指令类型已经被改写
    pSChannel->SG33.unFollowTimeOut = 0;
    //当当前指令类型为G33,将gG33中已加工头数清零
    if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_G33)
    {
        pSChannel->bG33SpindleOverSpeedWarningSigns = FALSE;
        pSChannel->SG33.ucAlreadyL = 0;
        return 1;
    }

    return 2;
}

/*************************************************
  函数名称：G33Action（）
  函数功能  G33运动函数
  输入参数：pInterpPrePointer    总插补预处理结构体指针
		   pSSpindle   单轴插补预处理结构体指针
		   pSG33                G33运行结构体指针
		   pSpindleEncoder             编码器结构体指针
  输出参数：
  返 回 值：-1 指针为空
			1 指针非空
  备    注：
**************************************************/
int8 G33Action(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33,
               struct SEncoder* pSpindleEncoder)
{
    int8 ret;

    if (pInterpPrePointer == NULL || pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    //超速检测 维护G33主轴超速警告全局变量
    ret = G33SpindleOverSpeedCheck(pSG33, pSpindleEncoder);

    if (ret == 1) //超速
    {
        parseRunningError(9999, GCODE_ERROR_NUM152, 9999);
        return 1;
    }

    //识别G33主轴加速或减速停完成，更新运动状态
    ret = changeG33StateAfterSpindleAccOrDec(pInterpPrePointer, pSSpindle, pSG33);

    //ret == 2：攻丝完毕，直接退出   ret == 4：主轴减速停后需要暂停，直接退出
    if (ret == 2 || ret == 4)
    {
        return 1;
    }

    //根据不同运动状态，进行G33运动
    switch (pSG33->G33MotionState)
    {
    case SPINDLE_ACC_BEFORE_ZAXIS_FEEDING: //Z轴进刀前主轴加速（多头）
        motionSpindleAccBeforeZAxisFeeding(pSG33);
        break;
    case ZAXIS_ACC_BEFORE_FOL_UP: //Z轴随动前加速
        motionZAxisAccBeforeFolUp(pSSpindle, pSG33, pSpindleEncoder);
        break;
    case ZAXIS_FOL_UP_TAPPING: //Z轴随动攻丝（此阶段主轴匀速，Z轴随动）
        motionZAxisFolUpTapping(pSSpindle, pSG33, pSpindleEncoder);
        break;
    case SPINDLE_DEC_TO_STOP_BEFORE_REV: //主轴反转前减速停
        motionSpindleDecToStopBeforeRev(pSSpindle, pSG33, pSpindleEncoder);
        break;
    case PAUSE_BEFORE_REVERSAL: //主轴反转前暂停
        motionPauseBeforeSpindleReversal(pSG33, pSpindleEncoder);
        break;
    case SPINDLE_REVERSE_RETURN: //主轴反转退刀
        motionSpindleReverseAndReturn(pSSpindle, pSG33, pSpindleEncoder);
        break;
        //case ZAXIS_FOL_UP_DEC_AND_RETURN:      //Z轴随动减速和退刀(待删)
        //	motionZAxisFolUpDecAndReturn(pSSpindle, pSG33, pSpindleEncoder);
        //	break;
    case SPINDLE_DEC_TO_STOP_AFTER_ZAXIS_RETURN: //退刀完成后主轴减速
        motionSpindleDecToStopAfterZAxisReturn(pSG33);
        break;
    default:
#if G33ProcLog_DEBUG
        printf("G33状态有误!\n");
#endif
        break;
    }

    return 1;
}

/*************************************************
  zxr: 检查 G33 刚性攻牙起终点及主轴减速随动冲程是否超出已使能软限位
  函数名称：checkG33StartEndSoftLimit
  函数功能：检查 G33 刚性攻牙起终点及减速随动冲程是否超出软限位
  输入参数：pSSpindle，主轴结构体指针；pSG33，结构体 struct G33 指针
  输出参数：无
  返 回 值：0：通过；非 0：对应的报警号（41~46为正限位，47~52为负限位）
  备    注：切入进给前静态检查，精准计入主轴刹车随动冲程（支持 bG33RecalDecPointFlag 开关）
**************************************************/
static uint16 checkG33StartEndSoftLimit(struct _SSpindle* pSSpindle, const struct G33* pSG33)
{
    SPointDouble SStartPoint;
    SPointDouble SEndPoint;
    const uint8* pMapAxisArray;
    uint8 ucAxis;
    double dSurpassLength = 0.0;

    if (pSG33 == NULL || pSG33->pSChannel == NULL)
    {
        return GCODE_ERROR_NUM153;
    }
    if (pSG33->pSChannel->ucChannelNum >= CHANNEL_NUM_MAX || pSG33->pSChannel->pMapAxisArray == NULL)
    {
        return GCODE_ERROR_NUM150;
    }

    // zxr: 计入攻牙到底后主轴减速刹车期间 Z 轴随动滑行冲程：
    // 1. 若未开启重算减速点(!bG33RecalDecPointFlag)或估算冲程<1mm，实际超程为 dEsti；
    // 2. 若开启重算减速点且冲程>=1mm，原系统减速点提前 (dEsti-1)，实际仍保留约 1mm 超程。
    if (pSSpindle != NULL)
    {
        double dEsti = calcSpindleDecZAxicSurpassLength(pSSpindle, (struct G33*)pSG33);
        if (isfinite(dEsti) && dEsti > 0.0)
        {
            if (g_SSystemPara.bG33RecalDecPointFlag && dEsti >= 1.0)
            {
                dSurpassLength = 1.0;
            }
            else
            {
                dSurpassLength = dEsti;
            }
        }
    }

    pMapAxisArray = pSG33->pSChannel->pMapAxisArray;
    SStartPoint = pSG33->G33StartPoint;
    SEndPoint = pSG33->G33EndPoint;

    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ucAxis++)
    {
        uint8 ucPhysicalAxis;
        double dStart;
        double dEnd;
        double dPeakPos;
        double dPeakNeg;

        if (VAL_VALID != getbit(pSG33->ucValStatus, ucAxis))
        {
            continue;
        }
        ucPhysicalAxis = pMapAxisArray[ucAxis];
        if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX ||
            ucPhysicalAxis >= sizeof(g_SSystemPara.MOTOR_WARN) / sizeof(g_SSystemPara.MOTOR_WARN[0]))
        {
            return GCODE_ERROR_NUM150;
        }
        if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
        {
            continue;
        }

        dStart = SStartPoint.dArray[ucAxis];
        dEnd = SEndPoint.dArray[ucAxis];
        if (!isfinite(dStart) || !isfinite(dEnd))
        {
            return GCODE_ERROR_NUM153;
        }

        // 计算包含主轴减速随动冲程的最大可达极值位置
        dPeakPos = dEnd;
        dPeakNeg = dEnd;
        if (dSurpassLength > 0.0)
        {
            double dVec = pSG33->G33StartVector.dArray[ucAxis];
            if (dVec > 0.0)
            {
                dPeakPos += dVec * dSurpassLength;
            }
            else if (dVec < 0.0)
            {
                dPeakNeg += dVec * dSurpassLength;
            }
        }

        // 正向软限位检查（检查起点、终点及正向冲程峰值）
        if (g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].soft_pos == 1)
        {
            double dLimit = g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].pos_soft;
            if (!isfinite(dLimit))
            {
                return GCODE_ERROR_NUM153;
            }
            if (dStart > dLimit || dEnd > dLimit || dPeakPos > dLimit)
            {
                return (uint16)(GCODE_ERROR_NUM41 + ucAxis);
            }
        }

        // 负向软限位检查（检查起点、终点及负向冲程峰值）
        if (g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].soft_neg == 1)
        {
            double dLimit = g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].neg_soft;
            if (!isfinite(dLimit))
            {
                return GCODE_ERROR_NUM153;
            }
            if (dStart < dLimit || dEnd < dLimit || dPeakNeg < dLimit)
            {
                return (uint16)(GCODE_ERROR_NUM47 + ucAxis);
            }
        }
    }

    return 0;
}

/*************************************************
  zxr: G33运动中新报警从原停轴首步接管，避免复用加工步骤跳过减速
  函数名称：MotionG33(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33, struct SEncoder* pSpindleEncoder)
  函数功能  G33运动接口函数，在总中断服务函数调用
  输入参数：pInterpPrePointer    总插补预处理结构体指针
		   pSSpindle   C单轴插补预处理结构体指针
		   pSG33                G33运行结构体指针
		   pSpindleEncoder      编码器结构体指针
  输出参数：
  返 回 值：-1 :指针为空或已转入报警停轴
			1  ：正常运行
  备    注：
**************************************************/
int8 MotionG33(struct SInterpolationPre* pInterpPrePointer, struct _SSpindle* pSSpindle, struct G33* pSG33,
               struct SEncoder* pSpindleEncoder)
{
    if (pInterpPrePointer == NULL || pSSpindle == NULL || pSG33 == NULL || pSpindleEncoder == NULL)
    {
        return -1;
    }

    switch (pInterpPrePointer->ucRunStep)
    {
        int8 ret;
    case 0:
        //轴占用阶段
        //释放上一指令的轴占用
        releaseLastCommandAxis(pInterpPrePointer);
#if (TURRET_AXIS_TEST == 1)
        //判断轴占用(根据结构体轴+映射)   目标结构体轴减去刀塔轴
        if (axisManagerOccupyByChannel(pSG33->ucValStatus & (~pInterpPrePointer->ucOccupideTurretAxis),
                                       pSG33->pSChannel->ucChannelNum, &pInterpPrePointer->ucOccupideValStatus) == 1)
        {
            //占用成功
            pInterpPrePointer->ucRunStep = 1;
        }
        else
        { //占用失败
            break;
        }
#else
        //判断轴占用(根据有效轴+映射)   有效轴减去刀补轴
        if (axisManagerOccupyByChannel(pSG33->ucValStatus & (~pInterpPrePointer->ucToolAxis),
                                       pSG33->pSChannel->ucChannelNum, NULL) == 1) //洪冠鹏 陈敏聪 2024/3/1
        {                                                                          //占用成功
            //上一条指令的占用（除去刀补）
            pInterpPrePointer->ucLastValStatus = pSG33->ucValStatus & (~pInterpPrePointer->ucToolAxis);
            //G1有效轴（不能除去刀补）
            pInterpPrePointer->ucRunStep = 1;
        }
        else
        { //占用失败
            break;
        }
#endif
    case 1:
        //进入G33初始化后要进行主轴测速，所以第一次初始化G33后直到测速完前不会有运动产生
        ret = initSG33Data(pInterpPrePointer, pSSpindle, pSG33, pSpindleEncoder);
        if (ret == 0 || ret == -3) //此时进入G33时主轴未加减速完毕，不运动或G33长度为0，直接跳过
        {
            return 1;
        }
        // zxr: 进入刚性攻牙实际运动前检查起终点软限位（含主轴减速随动冲程）
        {
            uint16 usLimitError = checkG33StartEndSoftLimit(pSSpindle, pSG33);
            if (usLimitError != 0)
            {
                parseRunningError(pSG33->pSChannel->SRun.unGCodeLineNum, usLimitError, 9999);
                pInterpPrePointer->ucInpStatus = INP_RUNNING;
                pInterpPrePointer->ucRunStep = 0;
                stopAxisAfterError(pSG33->pSChannel);
                return -1;
            }
        }
        pInterpPrePointer->ucRunStep = 2;
    case 2:
        if ((pSG33->G33RotateAxis != AXIS_A && !pInterpPrePointer->pSChannel->bCalculateSpindleSpeedFlag) //主轴测速完成
            || (pSG33->G33RotateAxis == AXIS_A &&
                !pInterpPrePointer->pSChannel->bCalculateSecondSpindleSpeedFlag)) //第二转轴测速完成 陈敏聪	2023/6/22
        {
            G33Action(pInterpPrePointer, pSSpindle, pSG33, pSpindleEncoder);
            // zxr: 加工阶段新报错后从报警停轴首步接管，避免沿用加工步骤跳过减速
            if (isErrorHasBeenWritten())
            {
                pInterpPrePointer->ucInpStatus = INP_RUNNING;
                pInterpPrePointer->ucRunStep = 0;
                stopAxisAfterError(pSG33->pSChannel);
                return -1;
            }
        }
        break;
    default:
        break;
    }

    return 1;
}
