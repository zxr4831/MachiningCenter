/************************************************************
  Copyright (C), 广东工业大学. 2021
  文件名：lookAhead.c
  作者：郭强    版本：V24.0    创建日期：2023/06/23
  模块描述：速度前瞻相关函数实现
  历史修改记录：<author><time><version ><desc>
				  **    2021/08/10     V3.0      前瞻段数添加最小前瞻段数限制
				  **    2021/08/12     V4.0      1.新增衔接点个数函数加入计算衔接点角度约束函数中
												 2.对全局变量三个标志位操作不再使用指针改用extern直接操作

				  **    2021/08/15     V5.0      1.对刀补缓冲区中单元可能不需速度前瞻的情况进行处理
												 2.不再要求刀补每次循环提供写入个数，速度前瞻反算过程中再计算Veer
												   在刀补结构体中增加bool bHasVeerFlag；标志位用于记录Veer是否计算过；
												   需要刀补写入时将该标志位 置FALSE;
												 3.目前暂定刀补结构体的变量：ucCommandType == 1 对应为需要速度前瞻的进给指令

				  **    2021/08/18     V6.0      1.简化程序，当需要进行前瞻计算时，对计算过程封装成函数

				  **    2021/08/19     V7.0      1.改变g_bLookAheadNewBeginFlag取反位置，在判断为首段就取反

				  **    2021/08/20     V8.0      1.更正求速度系数函数getCoef(),
												   调用该函数时对应轨迹的速度曲线类型变量ucVelPlanType还未赋值
												   应通过系统参数结构体获取速度曲线类型

												 2.将原本在calcVeer（）函数内的对bHasVeerFlag置TRUE的操作，
												   移到函数外（避免隐含操作）

				  **    2021/08/22     V9.0      1.仿真

				  **    2021/08/25     V10.0     1.将前瞻新的开始标志位在速度前瞻.c中定义

												 2.前瞻函数开始加入对系统手动暂停状态的判断，如果是SYS_RUN_TO_PAUSE
												 则不前瞻，并将bLookAheadNewBeginFlag置TRUE

				  **	2021/09/01     V10.0     1.增加控制指令类型中定义，#define CMD_TYPE_LINE   1  //直线运动
																		   #define CMD_TYPE_ARC   2  //圆弧运动
																		   相应修改judgeG01G02G03函数

												 2.增加速度曲线类型宏定义，相应改变getCoef()函数

												 3.改用宏定义min()函数

				  **	2021/09/05     V11.0     1.将实体类型EntityType = DOT的G01G02G03归为不需前瞻类型，这种G代码只改变IO状态但继承了G**模态

				  **    2021/11/18     V12.0     1.对主轴的前后作用带来的阻塞对前瞻的影响进行修改
												   当存在阻塞但需要前瞻的单元出现时处理如下：
													 存在前阻塞则为独立前瞻区间的开始
													 存在后阻塞则为一段独立的前瞻区间最后一个单元
													 特殊情况：当一单元既存在前阻塞又存在后阻塞，将其归为不需前瞻单元，但要对其赋值VsVe

												 2.对前瞻时的要求进给速度dFeedrate<dSysMinVel时，运动段VsVe按dFeedrate处理

				  **    2021/11/26     V13.0     1.针对系统参数中加速段和减速段速度曲线类型分开设置相应调整，更改或添加如下函数
												   double getCoef(uint8 ucVelType);double getMaxAcc(uint8 ucVelType);double getMaxDec(uint8 ucVelType);

				  **	2021/12/30     V14.0     1.更正之前对同时存在前后类型的G123的处理
												 2.加入倍率，使用最大倍率计算前后段长度的速度约束，G代码的原始dFeedrate不再改变
												 3.刀补结构体新增参数dVmax记录衔接点前段轨迹长度约束速度

				  **	2022/03/01     V15.0     1.针对G99每转进给对衔接点前后段长度约束速度进行修改
				  *
				  **	2022/04/11     V16.0     1.添加前瞻功能开启全局变量

				  **	2022/04/25     V17.0     1.针对轴数组及多通道模式进行修改,
												   前瞻三个轴，哪三个轴在lookAheadInit（）中初始化，并存在	g_ucLookAheadAxisArray[3]数组中

				  **	2022/06/15     V18.0     1.多轴前瞻更改（Version1）

				  **	2022/06/29     V19.0     1.多轴前瞻更改（Version2）（主要更改复合运动最大速度最小速度加速度减速度由单轴及算得到）
				  
				  **    2022/11/22     V20.0     1.根据系统参数中的前瞻模式和插补方式参数集成多种前瞻
				                                 2.增加苏总要求的切削进给段间低速下限值及档位功能
												 3.发现之前多通道初版（未公开）前瞻moveToG01G02G03()函数的错误并修改（2022/11/25）

				  **	2023/02/16     V21.0     1.增加修正进给速度dFeedrate时对G99模式的判断和处理(之前版本均默认G98模式)
				  
                  **	2023/05/26     V22.0     1.圆弧最大法向加速度约束对应速度计算函数calcArcMaxNormalAccVmax()增加对0值的判断，若为零，使用一个非零值进行计算
				                                 2.圆弧弓高误差速度计算函数calcArcMaxChordErrorVmax()增加对0值的判断，若为零，使用一个非零值进行计算
												   更改误差对应速度的计算方式，分三种情况讨论。

				  **	2023/06/16     V23.0     1.计算最小初速度、最小末速度、衔接点速度时增加(轨迹长度/插补周期)约束
				  
				  **	2023/06/23     V24.0     1.更改前瞻多轴模式下夹角约束计算中dTempVeer的初始化
				  
				  **    2024/05/23     V25.0     1.增加G0前瞻，G0与G1间单轴同向时前瞻
*************************************************************/

#include "lookAhead.h"
#include "GeneralFunctions.h"
#include "IOCommand.h"
#include <math.h>
#include <stdlib.h>
#include "Channel.h"
#include "CTrajPlanning.h"
#include "ringbuff.h"

//前瞻功能开启标志位
bool g_bLookAheadFunctionOpenFlag;

//定义前瞻的三个轴（用于只有三移动轴参与联动插补的应用场景）
uint8 g_ucLookAheadAxisArray[3];

/*************************************************
  函数名称：lookAheadInit(uint8 ucChannelNum)
  函数功能  前瞻需要初始化的参数
  输入参数：通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 lookAheadInit(struct SChannel* pSChannel)
{
    if (pSChannel == NULL)
    {
        return -1;
    }
    pSChannel->bLookAheadNewBeginFlag = TRUE; //开机、新开始需要置TRUE

    //初始化前瞻的三个轴,此初始化用于三轴前瞻
    g_ucLookAheadAxisArray[0] = AXIS_X;
    g_ucLookAheadAxisArray[1] = AXIS_Y;
    g_ucLookAheadAxisArray[2] = AXIS_Z;

    return 1;
}
/*************************************************
  函数名称：judgeG0(struct SToolCompensation* pToolElement, uint8 ucChannelNum, int16 sIndex)
  函数功能  判断是否为需要前瞻的GO
  输入参数：pToolElement  刀补缓冲区单元指针
			ucChannelNum  通道号
			sIndex        刀补缓冲区单元指针索引号
  输出参数：
  返 回 值:	 0  ：当前单元和前一个单元的轴号不一致或方向不一致->不前瞻
			 1  ：当前单元和前一个单元为相同单轴的同向运动->前瞻
			 -1 ：前一个单元不为单轴运动
  备    注：
**************************************************/
int8 judgeG0(struct SToolCompensation* pToolElement, uint8 ucChannelNum, int16 sIndex)
{
    int16 sBeforeIndex = sIndex - 1; //获取前一个单元的索引

    struct SToolCompensation* pBeforeToolElement = NULL;                         //前一个单元的指针初始化
    pBeforeToolElement = getSToolCompPointerByIndex(sBeforeIndex, ucChannelNum); //获取前一个单元的数据
    uint8 ucBeforeAxisNumOfNeedLookAhead;                                        //保存前一个单元单轴运动的轴号
    uint8 ucBeforeAxisDirOfNeedLookAhead;                                        //保存前一个单元单轴运动的方向

    //前一个单元不为单轴运动
    if (getBits(pBeforeToolElement->unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_QUICKMOVE)
    {
        return -1;
    }

    //获取前一个单元需要前瞻的插补单轴轴号
    if (pBeforeToolElement == NULL)
    {
        ucBeforeAxisNumOfNeedLookAhead = AXIS_NULL;
        ucBeforeAxisDirOfNeedLookAhead = INP_GO_AHEAD;
    }
    else
    {
        ucBeforeAxisNumOfNeedLookAhead = pBeforeToolElement->ucAxisNumOfNeedLookAhead;
        ucBeforeAxisDirOfNeedLookAhead = pBeforeToolElement->cAxisDirOfNeedLookAhead;
    }

    //当前单元和前一个单元为相同单轴的同向运动
    if (pToolElement->ucAxisNumOfNeedLookAhead == ucBeforeAxisNumOfNeedLookAhead &&
        pToolElement->cAxisDirOfNeedLookAhead == ucBeforeAxisDirOfNeedLookAhead)
    {
        return 1; //加入需要前瞻连续轨迹段
    }
    //当前单元和前一单元轴号不同/方向不同
    pToolElement->ucAxisNumOfNeedLookAhead = AXIS_NULL; //将“需要前瞻的单轴轴号”置为空
    return 0;                                           //不加入需要前瞻连续轨迹段
}

/*************************************************
  函数名称：judgeCircularArc(struct SToolCompensation* pToolElement, int nArraySize)
  函数功能  判断是否为平面圆弧
  输入参数：pToolElement  刀补缓冲区单元指针
		   nArraySize    轴数
  输出参数：
  返 回 值： TRUE   : 是平面圆弧
			FALSE  : 非平面圆弧
  备    注：此判断默认G2/3指令中无旋转轴移动（2022/06/15）
            记录长轴轴号保存在 pToolcode->ucLongAxisNum 中
			当圆弧轴为长轴，长轴轴号置为AXIS_NULL，
			外部程序可通过pToolcode->ucLongAxisNum是否为AXIS_NULL确定长轴是否为第三轴,
			当存在第三轴，其长度相对于圆弧长度的比率保存在起点向量中（有符号）
**************************************************/
bool judgePlanarArc(struct SToolCompensation* pToolElement, int nArraySize)
{
    int i = 0, res = 0, Axis1 = 0, Axis2 = 0;

    double dTempLength = 0;                    //临时变量长度
    double dOtherAxisMaxLength = 0;            //记录其他轴最大位移量
    double dArcLength = pToolElement->dLength; //平面圆弧的长度

    //当最大轴数为2时为平面圆弧
    if (nArraySize == 2)
    {
        return TRUE;
    }

    //确定插补平面
    switch (getBits(pToolElement->unCommandType, CMD_PLANE_ADDR))
    {
    case CMD_PLANE_G17:
        Axis1 = AXIS_X;
        Axis2 = AXIS_Y;
        break;
    case CMD_PLANE_G18:
        Axis1 = AXIS_Z;
        Axis2 = AXIS_X;
        break;
    case CMD_PLANE_G19:
        Axis1 = AXIS_Y;
        Axis2 = AXIS_Z;
        break;
    default:
        Axis1 = AXIS_Z;
        Axis2 = AXIS_X;
        break;
    }

    //判断插补平面外的轴起终点是否相等
    for (i = 0; i < nArraySize; i++)
    {
        if (i == Axis1 || i == Axis2)
        {
            continue; //插补平面轴跳过
        }
        else
        {
            dTempLength = fabs(pToolElement->SEndPoint.dArray[i] - pToolElement->SStartPoint.dArray[i]);
            //当其他轴起终点不同，此时非平面圆弧
            if (!isZero(dTempLength))
            {
                res = 1;
                if (dTempLength > dOtherAxisMaxLength)
                {
                    pToolElement->ucLongAxisNum = i;   //更新长轴轴号
                    dOtherAxisMaxLength = dTempLength; //更新最长轴位移量
                }
            }
        }
    }

    //对比其他轴最大增量和圆弧长度，维护长轴轴号
    if (dArcLength < dOtherAxisMaxLength) //第三轴为长轴
    {
        //计算第三轴长轴位移增量相对于平面圆弧的倍率保存在向量中
        pToolElement->SStartVector.dArray[pToolElement->ucLongAxisNum] =
            (pToolElement->SEndPoint.dArray[pToolElement->ucLongAxisNum] -
             pToolElement->SStartPoint.dArray[pToolElement->ucLongAxisNum]) /
            dArcLength;

        //此时长轴轴号即为正确，不用修改

        return FALSE;
    }
    else
    {
        if (res == 1) //存在第三轴但不是长轴
        {
            //计算第三轴位移增量相对于平面圆弧的倍率保存在向量中
            pToolElement->SStartVector.dArray[pToolElement->ucLongAxisNum] =
                (pToolElement->SEndPoint.dArray[pToolElement->ucLongAxisNum] -
                 pToolElement->SStartPoint.dArray[pToolElement->ucLongAxisNum]) /
                dArcLength;

            pToolElement->ucArcThirdAxisNum = pToolElement->ucLongAxisNum; //保存非长轴第三轴轴号
        }

        pToolElement->ucLongAxisNum = AXIS_NULL; //圆弧轴为长轴，长轴轴号置为AXIS_NULL
    }

    if (res == 1)
    {
        return FALSE; //存在第三轴，但在这里返回第三轴不是长轴
    }

    return TRUE; //平面圆弧
}

/*************************************************
  函数名称：judgeG00G01G02G03(struct *SToolCompensation pToolElement, uint8 ucChannelNum)
  函数功能  判断刀补写入的单元是否需要速度前瞻即是否是G01G02G03
  输入参数：pToolElement  刀补缓冲区单元指针 ucChannelNum 通道号
  输出参数：
  返 回 值： -1:指针为空
			 0：不需前瞻单元
			 1: 需要前瞻单元
			 2：原本需要前瞻，但存在前后阻塞的单元M03/M04 + M05（这种单元处理为会被跳过的单元）
			 3：原本需要前瞻，但存在前作用和通道等待，此时相当于2，这种情况处理同2
			 4：圆弧指令运动非平面圆弧
			 5：前瞻首单元且含有后作用
  备    注：  有通道等待无前作用时为需要前瞻单元，且为最后一段
**************************************************/
int8 judgeG00G01G02G03(struct SToolCompensation* pToolElement, uint8 ucChannelNum)
{
    if (pToolElement == NULL)
    {
        return -1;
    }

    if (pToolElement->EntityType == DOT ||
        isZero(pToolElement->dLength)) //点类型不需前瞻//2023.3.23加入对dLength为零判断
    {
        return 0;
    }

    switch (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR))
    {
    case CMD_MOTION_QUICKMOVE: //快速运动  2024/5/27 洪冠鹏 增加G0的前瞻
        //单个轴运动
        if (pToolElement->ucAxisNumOfNeedLookAhead != AXIS_NULL)
        {
            return 1; //单个轴运动，要前瞻
        }
        return 0; //多个轴运动，不前瞻
        break;

    case CMD_MOTION_LINE: //直线
        //当同时前后阻塞此段不需前瞻M03/M04 + M05
        if (isBlockedInCommand(pToolElement->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 3)
        {
            return 2;
        }
        //当存在前阻塞和通道等待指令时
        if (isBlockedInCommand(pToolElement->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 1 &&
            pToolElement->sChannelCommand.ucType != UNASSIGNED_VALUE)
        {
            return 3;
        }
        if (isBlockedInCommand(pToolElement->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 2 &&
            g_SChannel->bLookAheadNewBeginFlag == TRUE) //前瞻首单元且含有后作用	陈敏聪	7/15
        {
            return 5;
        }
        return 1;
        break;

    case CMD_MOTION_ARC:      //顺圆
    case CMD_MOTION_ANTI_ARC: //逆圆

        //判断圆弧插补平面轴外是否有另外的轴需要移动，若存在不前瞻，此判断优先级高于后边两种情况
        if (!judgePlanarArc(pToolElement, AXIS_NUM_MAX))
        {
            return 4;
        }
        //当同时前后阻塞此段不需前瞻M03/M04 + M05
        if (isBlockedInCommand(pToolElement->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 3)
        {
            return 2;
        }
        //当存在前阻塞和通道等待指令时
        if (isBlockedInCommand(pToolElement->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 1 &&
            pToolElement->sChannelCommand.ucType != UNASSIGNED_VALUE)
        {
            return 3;
        }
        if (isBlockedInCommand(pToolElement->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE) == 2 &&
            g_SChannel->bLookAheadNewBeginFlag == TRUE)
        {
            return 5;
        }
        return 1;
        break;

    default: //其他情况不前瞻
        return 0;
        break;
    }
}

/*************************************************
  函数名称：correctFeedrateAndLowerLimitSpeed(struct SToolCompensation* pToolcode, SVector SStartVector, int nArraySize)
  输入参数：
			Toolcode	       刀补后的结构体
			nArraySize         轴数组大小
			bPlanarArcFlag     平面圆弧标志位 ：平面圆弧TRUE、非平面圆弧FALSE
			uint8 ucChannelNum  通道号
  输出参数：
  返 回 值：
  备    注：1.多轴模式下修正长轴进给速度对应的合成速度
			2.记录长轴轴号保存在 pToolcode->ucLongAxisNum 中
			3.根据长轴轴号反算对应的复合运动
			4.非平面圆弧时，长轴为第三轴修正圆弧弓高误差及最大法向加速度误差对应速度
			注：此函数中无进给速度dFeedrate和其他速度作对比，所以无需判断G99
**************************************************/
void correctFeedrateAndLowerLimitSpeed(struct SToolCompensation* pToolcode, int nArraySize, bool bPlanarArcFlag,
                                       uint8 ucChannelNum)
{
    double dCoefOfFeedrate;

    if (pToolcode == NULL)
    {
        return;
    }

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //计算dFeedrate变换到mm/ms系数
    dCoefOfFeedrate =
        getBits(pToolcode->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
            ? 1.0
            : calcCoefOfFeedrate(pToolcode, ucChannelNum); //2024/5/27 洪冠鹏 G0不受G98或G99影响,转换系数设为1

    double dSpeedGear = 0; //切削进给段间低速下限值档位

    if (g_SSystemPara.ucGearOfSysLowerLimitSpeed == 1)
    {
        dSpeedGear = 1.0;
    }
    else if (g_SSystemPara.ucGearOfSysLowerLimitSpeed == 2)
    {
        dSpeedGear = 1.25;
    }
    else //第三档位
    {
        dSpeedGear = 1.5;
    }
    //根据指令作进一步计算
    if (getBits(pToolcode->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) //G0  洪冠鹏 2024/5/25
    {
        if (pToolcode->ucAxisNumOfNeedLookAhead != AXIS_NULL) //需要前瞻的G0
        {
            if (pToolcode->ucAxisNumOfNeedLookAhead >= AXIS_NUM_MAX ||
                g_SChannel[ucChannelNum].pMapAxisArray[pToolcode->ucAxisNumOfNeedLookAhead] >= AXIS_PHYSICS_NUM_MAX)
            {
                parseRunningError(9999, GCODE_ERROR_NUM150, 9999);
                return;
            }
            //1.获取进给速度
            pToolcode->dFeedrate =
                g_SSystemPara.SAxisPara[g_SChannel[ucChannelNum].pMapAxisArray[pToolcode->ucAxisNumOfNeedLookAhead]]
                    .dVmax;
            //2.获取切削进给段间低速下限值
            pToolcode->dLowerLimitSpeed = g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear;
        }
    }
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE          //倾斜Y轴机
    else if (getBits(pToolcode->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE) //G1
    {
        if (g_SChannel[ucChannelNum].bUseVirtualY == TRUE) //开启合成虚拟Y轴功能：与常规方式不同，重新计算物理轴长轴
        {
            double dMaxNum = 0; //记录物理轴移动量最大值
            double dTemp = 0;
            double dLengthTemp[AXIS_NUM_MAX] = {0}; //记录物理轴的移动距离，用于筛选出长轴
            double dLength = 0;                     //各物理轴移动量合成的运动总量
            //计算物理轴的移动量
            for (int i = 0; i < nArraySize; i++)
            {
                if (i == AXIS_X) //X1=X+Y*tanθ
                {
                    dLengthTemp[i] = pToolcode->SStartVector.dArray[i] * pToolcode->dLength +
                                     1.0 * g_SChannel[ucChannelNum].dTANAngleYY1 *
                                         pToolcode->SStartVector.dArray[AXIS_Y] * pToolcode->dLength;
                    dLength += pow(dLengthTemp[i], 2);
                }
                else if (i == AXIS_Y) //Y1=Y/cosθ
                {
                    dLengthTemp[i] =
                        pToolcode->SStartVector.dArray[i] * pToolcode->dLength / g_SChannel[ucChannelNum].dCOSAngleYY1;
                    dLength += pow(dLengthTemp[i], 2);
                }
                else
                {
                    dLengthTemp[i] = pToolcode->SStartVector.dArray[i] * pToolcode->dLength;
                    dLength += pow(dLengthTemp[i], 2);
                }
            }
            //筛选长轴
            dLength = sqrt(dLength);
            for (int i = 0; i < nArraySize; i++)
            {
                dTemp = fabs(dLengthTemp[i] / dLength);
                if (dTemp > dMaxNum)
                {
                    pToolcode->ucLongAxisNum = i; //更新长轴轴号
                    dMaxNum = dTemp;
                }
            }

            if (!isZero(dMaxNum))
            {
                if (pToolcode->ucLongAxisNum == AXIS_X) //X1轴是长轴
                {
                    //1.修正指令轴合成方向的进给速度，保证物理轴长轴最大速度为F值
                    pToolcode->dFeedrate = pToolcode->dFeedrate / (fabs(pToolcode->SStartVector.dArray[AXIS_X]) +
                                                                   1.0 * fabs(pToolcode->SStartVector.dArray[AXIS_Y]) *
                                                                       g_SChannel[ucChannelNum].dTANAngleYY1);
                    //2.根据长轴轴号反算对应的复合运动切削进给段间低速下限值
                    pToolcode->dLowerLimitSpeed =
                        g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear /
                        (fabs(pToolcode->SStartVector.dArray[AXIS_X]) +
                         1.0 * fabs(pToolcode->SStartVector.dArray[AXIS_Y]) * g_SChannel[ucChannelNum].dTANAngleYY1);
                }
                else if (pToolcode->ucLongAxisNum == AXIS_Y) //Y1轴是长轴
                {
                    //1.修正指令轴合成方向的进给速度，保证物理轴长轴最大速度为F值
                    pToolcode->dFeedrate = pToolcode->dFeedrate * g_SChannel[ucChannelNum].dCOSAngleYY1 /
                                           (fabs(pToolcode->SStartVector.dArray[AXIS_Y]));
                    //2.根据长轴轴号反算对应的复合运动切削进给段间低速下限值
                    pToolcode->dLowerLimitSpeed = g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear *
                                                  g_SChannel[ucChannelNum].dCOSAngleYY1 /
                                                  (fabs(pToolcode->SStartVector.dArray[AXIS_Y]));
                }
                else
                {
                    //1.修正进给速度
                    pToolcode->dFeedrate = pToolcode->dFeedrate / dMaxNum;
                    //2.根据长轴轴号反算对应的复合运动切削进给段间低速下限值
                    pToolcode->dLowerLimitSpeed = g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear / dMaxNum;
                }
            }
        }
        else //未开启合成虚拟Y轴功能：与常规方式一致，长轴为指令轴最长轴
        {
            int i = 0;
            double dMaxNum = 0; //记录向量分量最大值
            double dTemp = 0;
            pToolcode->ucLongAxisNum = 0; //长轴轴号
            if (pToolcode == NULL)
            {
                return;
            }

            for (i = 0; i < nArraySize; i++)
            {
                dTemp = fabs(pToolcode->SStartVector.dArray[i]);
                if (dTemp > dMaxNum)
                {
                    pToolcode->ucLongAxisNum = i; //更新长轴轴号
                    dMaxNum = dTemp;
                }
            }

            if (!isZero(dMaxNum))
            {
                //1.修正进给速度
                pToolcode->dFeedrate = pToolcode->dFeedrate / dMaxNum;

                //2，根据长轴轴号反算对应的复合运动切削进给段间低速下限值
                pToolcode->dLowerLimitSpeed = g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear / dMaxNum;
            }
            else
            {
                pToolcode->dFeedrate = 0;
                pToolcode->dLowerLimitSpeed = 0;
            }
        }
    }
#else
    else if (getBits(pToolcode->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE) //G1
    {
        int i = 0;
        double dMaxNum = 0; //记录向量分量最大值
        double dTemp = 0;
        pToolcode->ucLongAxisNum = 0; //长轴轴号
        if (pToolcode == NULL)
        {
            return;
        }

        for (i = 0; i < nArraySize; i++)
        {
            dTemp = fabs(pToolcode->SStartVector.dArray[i]);
            if (dTemp > dMaxNum)
            {
                pToolcode->ucLongAxisNum = i; //更新长轴轴号
                dMaxNum = dTemp;
            }
        }

        if (!isZero(dMaxNum))
        {
            //1.修正进给速度
            pToolcode->dFeedrate = pToolcode->dFeedrate / dMaxNum;

            //2，根据长轴轴号反算对应的复合运动切削进给段间低速下限值
            pToolcode->dLowerLimitSpeed = g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear / dMaxNum;
        }
        else
        {
            pToolcode->dFeedrate = 0;
            pToolcode->dLowerLimitSpeed = 0;
        }
    }
#endif
    else //G2G3
    {
        //1.段间下限值不用修正但是要算出来赋值（初版的这个值是在刀补处计算的，但设计速度这里改为在前瞻计算）
        pToolcode->dLowerLimitSpeed = g_SSystemPara.dSysLowerLimitSpeed * dSpeedGear;

        //判断是否是平面圆弧
        if (!bPlanarArcFlag) //非平面圆弧
        {
#if MC_PROFILE_XYZ
            /* G94 的 F 表示沿三维螺旋线的路径速度。原有插补器使用参数 q 推进，
             * 其长度取平面弧长与垂直轴行程的较大值。
             * 将实际路径速度换算到同一 q 参数，保证规划与插补使用一致的长度基准。 */
            uint8 plane[2];
            chooseInterpPlane(plane, pToolcode->unCommandType);
            uint8 perpendicular = (uint8)(AXIS_X + AXIS_Y + AXIS_Z - plane[0] - plane[1]);
            double planar = fabs(pToolcode->dLength);
            double axial =
                fabs(pToolcode->SEndPoint.dArray[perpendicular] - pToolcode->SStartPoint.dArray[perpendicular]);
            double spatial = hypot(planar, axial);
            if (spatial > 0.0)
            {
                double parameter_ratio = max(planar, axial) / spatial;
                pToolcode->dFeedrate *= parameter_ratio;
                pToolcode->dLowerLimitSpeed *= parameter_ratio;
            }
#endif
            if (pToolcode->ucLongAxisNum != AXIS_NULL) //长轴为第三轴
            {
                //修正之前计算的圆弧弓高误差及法向加速度误差对应的第三轴长轴速度
                //注意圆弧的长轴轴号是在函数judgePlanarArc()中计算维护的，G1的长轴轴号在此函数中维护（暂时没用）
                double dRatio = fabs(pToolcode->SStartVector.dArray[pToolcode->ucLongAxisNum]);

                //弓高误差速度约束
                pToolcode->dArcMaxChordErrorVmax = dRatio * pToolcode->dArcMaxChordErrorVmax;
                //最大法向加速度速度约束
                pToolcode->dArcMaxNormalAccVmax = dRatio * pToolcode->dArcMaxNormalAccVmax;
            }
        }

        //2.修正圆弧进给速度不超过弓高和法向加速度约束（2023.3.14郭强）
        pToolcode->dFeedrate = pToolcode->dFeedrate * dCoefOfFeedrate > pToolcode->dArcMaxChordErrorVmax
                                   ? pToolcode->dArcMaxChordErrorVmax / dCoefOfFeedrate
                                   : pToolcode->dFeedrate;
        pToolcode->dFeedrate = pToolcode->dFeedrate * dCoefOfFeedrate > pToolcode->dArcMaxNormalAccVmax
                                   ? pToolcode->dArcMaxNormalAccVmax / dCoefOfFeedrate
                                   : pToolcode->dFeedrate;
    }
}

/*************************************************
  函数名称：calcVAndAPara(struct SToolCompensation* pToolElement, struct SChannel* pSChannel)
  函数功能: 向刀补结构体中添加 加速度、速度以及加减速类型信息
		   包括：
			double dAcc;	    ///<加速段加速度	mm/ms2
			double dDec;	    ///<减速段加速度	mm/ms2

			double dCommonVMaxFromSingleAxis;    //G1，G2,G3复合运动最大速度由各插补轴计算得到        郭强 2022/6/16
			double dCommonVsMinFromSingleAxis;    //G1，G2,G3复合运动最小起跳速度由各插补轴计算得到   郭强 2022/6/16
			double dCommonVeMinFromSingleAxis;    //G1，G2,G3复合运动最小终止速度由各插补轴计算得到   郭强 2022/6/16

			uint8 ucAccVelPlanType; //加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
			uint8 ucDecVelPlanType; //减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型

  输入参数：struct SToolCompensation* pToolElement    刀补缓冲区指针
		   pSChannel                   通道指针
  输出参数：
  返 回 值：-1 指针为空
			1 计算成功
  备    注：
**************************************************/
int8 calcVAndAPara(struct SToolCompensation* pToolElement, struct SChannel* pSChannel, bool bPlanarArcFlag)
{
    uint8* ucMapArray = pSChannel->pMapAxisArray; //映射

    if (pToolElement == NULL || pSChannel == NULL || ucMapArray == NULL)
    {
        return -1;
    }

    //加减速类型
    pToolElement->ucAccVelPlanType = g_SSystemPara.ucCommonAccVelPlanType;
    pToolElement->ucDecVelPlanType = g_SSystemPara.ucCommonDecVelPlanType;

    //加速度减速度
    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE) //自适应参数插补模式
    {
        int i = 0;
        double dAccArray[AXIS_NUM_MAX];
        double dDecArray[AXIS_NUM_MAX];
        double dVMaxArray[AXIS_NUM_MAX];
        double dVsArray[AXIS_NUM_MAX];
        double dVeArray[AXIS_NUM_MAX];

        for (i = 0; i < AXIS_NUM_MAX; i++)
        {
            if (ucMapArray[i] == AXIS_NULL) //2023.3.23对空轴判断郭强
            {
                dAccArray[i] = 0;
                dDecArray[i] = 0;
                dVMaxArray[i] = 0;
                dVsArray[i] = 0;
                dVeArray[i] = 0;
                continue;
            }

            dAccArray[i] = getSingleAxisMaxAcc(i, g_SSystemPara.ucCommonAccVelPlanType, ucMapArray);
            dDecArray[i] = getSingleAxisMaxDec(i, g_SSystemPara.ucCommonDecVelPlanType, ucMapArray);
            dVMaxArray[i] = g_SSystemPara.SAxisPara[ucMapArray[i]].dVmax;
            dVsArray[i] = g_SSystemPara.SAxisPara[ucMapArray[i]].dVs;
            dVeArray[i] = g_SSystemPara.SAxisPara[ucMapArray[i]].dVe;
        }
        //分为直线和圆弧两种情况
        if (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE) //直线按各轴及其比例系数计算
        {
            pToolElement->dAcc = calcMinTwoArrays(dAccArray, pToolElement->SStartVector.dArray);
            pToolElement->dDec = calcMinTwoArrays(dDecArray, pToolElement->SStartVector.dArray);
            pToolElement->dCommonVMaxFromSingleAxis = calcMinTwoArrays(dVMaxArray, pToolElement->SStartVector.dArray);
            pToolElement->dCommonVsMinFromSingleAxis = calcMinTwoArrays(dVsArray, pToolElement->SStartVector.dArray);
            pToolElement->dCommonVeMinFromSingleAxis = calcMinTwoArrays(dVeArray, pToolElement->SStartVector.dArray);
        }
        else if (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
                 getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) ==
                     CMD_MOTION_ANTI_ARC) //圆弧，需要分情况讨论是否有第三轴
        {
            int Axis1 = 0, Axis2 = 0;
            //确定插补平面
            switch (getBits(pToolElement->unCommandType, CMD_PLANE_ADDR))
            {
            case CMD_PLANE_G17:
                Axis1 = AXIS_X;
                Axis2 = AXIS_Y;
                break;
            case CMD_PLANE_G18:
                Axis1 = AXIS_Z;
                Axis2 = AXIS_X;
                break;
            case CMD_PLANE_G19:
                Axis1 = AXIS_Y;
                Axis2 = AXIS_Z;
                break;
            default:
                Axis1 = AXIS_Z;
                Axis2 = AXIS_X;
                break;
            }

            //根据插补平面选出插补轴中值较小的那个参数
            pToolElement->dAcc = min(dAccArray[Axis1], dAccArray[Axis2]);
            pToolElement->dDec = min(dDecArray[Axis1], dDecArray[Axis2]);
            pToolElement->dCommonVMaxFromSingleAxis = min(dVMaxArray[Axis1], dVMaxArray[Axis2]);
            pToolElement->dCommonVsMinFromSingleAxis = min(dVsArray[Axis1], dVsArray[Axis2]);
            pToolElement->dCommonVeMinFromSingleAxis = min(dVeArray[Axis1], dVeArray[Axis2]);

            if (!bPlanarArcFlag) //非平面圆弧，要在插补平面和第三轴中选择出最终的参数
            {
                double dRatio = 0;
                if (pToolElement->ucLongAxisNum == AXIS_NULL) //长轴是插补平面
                {
                    dRatio =
                        fabs(pToolElement->SStartVector.dArray[pToolElement->ucArcThirdAxisNum]); //注意此时第三轴轴号
                    pToolElement->dAcc = min(pToolElement->dAcc, dAccArray[pToolElement->ucArcThirdAxisNum] / dRatio);
                    pToolElement->dDec = min(pToolElement->dDec, dDecArray[pToolElement->ucArcThirdAxisNum] / dRatio);
                    pToolElement->dCommonVMaxFromSingleAxis = min(pToolElement->dCommonVMaxFromSingleAxis,
                                                                  dVMaxArray[pToolElement->ucArcThirdAxisNum] / dRatio);
                    pToolElement->dCommonVsMinFromSingleAxis = min(pToolElement->dCommonVsMinFromSingleAxis,
                                                                   dVsArray[pToolElement->ucArcThirdAxisNum] / dRatio);
                    pToolElement->dCommonVeMinFromSingleAxis = min(pToolElement->dCommonVeMinFromSingleAxis,
                                                                   dVeArray[pToolElement->ucArcThirdAxisNum] / dRatio);
                }
                else //长轴是第三轴
                {
                    dRatio =
                        fabs(pToolElement->SStartVector.dArray[pToolElement->ucLongAxisNum]); //注意此时用的长轴轴号
                    pToolElement->dAcc = min(pToolElement->dAcc * dRatio, dAccArray[pToolElement->ucLongAxisNum]);
                    pToolElement->dDec = min(pToolElement->dDec * dRatio, dDecArray[pToolElement->ucLongAxisNum]);
                    pToolElement->dCommonVMaxFromSingleAxis =
                        min(pToolElement->dCommonVMaxFromSingleAxis * dRatio, dVMaxArray[pToolElement->ucLongAxisNum]);
                    pToolElement->dCommonVsMinFromSingleAxis =
                        min(pToolElement->dCommonVsMinFromSingleAxis * dRatio, dVsArray[pToolElement->ucLongAxisNum]);
                    pToolElement->dCommonVeMinFromSingleAxis =
                        min(pToolElement->dCommonVeMinFromSingleAxis * dRatio, dVeArray[pToolElement->ucLongAxisNum]);
                }
            }
        }
        else //单轴运动的G0  2024/5/27 洪冠鹏
        {
            if (pToolElement->ucAxisNumOfNeedLookAhead >= AXIS_PHYSICS_NUM_MAX)
            {
                return -1;
            }
            pToolElement->dAcc = dAccArray[(pToolElement->ucAxisNumOfNeedLookAhead)];
            pToolElement->dDec = dDecArray[(pToolElement->ucAxisNumOfNeedLookAhead)];
            pToolElement->dCommonVMaxFromSingleAxis = dVMaxArray[(pToolElement->ucAxisNumOfNeedLookAhead)];
            pToolElement->dCommonVsMinFromSingleAxis = dVsArray[(pToolElement->ucAxisNumOfNeedLookAhead)];
            pToolElement->dCommonVeMinFromSingleAxis = dVeArray[(pToolElement->ucAxisNumOfNeedLookAhead)];
        }
    }
    else //长轴模式和固定参数扩展线位移模式
    {
        if (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) //2024/5/27 洪冠鹏
        {
            if (pToolElement->ucAxisNumOfNeedLookAhead >= AXIS_PHYSICS_NUM_MAX)
            {
                return -1;
            }
            pToolElement->dAcc = getSingleAxisMaxAcc(pToolElement->ucAxisNumOfNeedLookAhead,
                                                     g_SSystemPara.ucCommonAccVelPlanType, ucMapArray);
            pToolElement->dDec = getSingleAxisMaxDec(pToolElement->ucAxisNumOfNeedLookAhead,
                                                     g_SSystemPara.ucCommonDecVelPlanType, ucMapArray);
        }
        else
        {
            pToolElement->dAcc = getMaxAcc(g_SSystemPara.ucCommonAccVelPlanType);
            pToolElement->dDec = getMaxDec(g_SSystemPara.ucCommonDecVelPlanType);
        }
    }

    return 1;
}

/*************************************************
  函数名称：calcCoefOfFeedrate(struct SToolCompensation* pToolElement, uint8 ucChannelNum)
  输入参数：pToolElement	       刀补后的结构体
			uint8 ucChannelNum     通道号
  输出参数：
  返 回 值：系数
  备    注：计算进给速度对比系数(防止G99模式下进给速度单位为mm/R直接与mm/ms速度量都比较)
**************************************************/
double calcCoefOfFeedrate(struct SToolCompensation* pToolElement, uint8 ucChannelNum)
{
    double dCoef;
    double dG97AbsSpindleSpeed;

    if (pToolElement == NULL)
    {
        return 1.0;
    }
    //获取主轴转速r/ms
    // 不用上方函数是因为通道结构体中的主轴转速只是当前正在插补的指令的目标主轴转速，并不代表前瞻当前处理的主轴转速。
    //刀补缓冲区中主轴转速单位为r/min,注意转换
    dG97AbsSpindleSpeed = pToolElement->dSpindleSpeed / 60 / 1000;

#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE && ucChannelNum == 1)
    dCoef = getBits(pToolElement->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN
                ? 1.0
                : g_SSystemPara.SAxisPara[g_SChannel[0].pMapAxisArray[AXIS_C]].dVmax /
                      DEGREE_OF_CIRCLE; //陈敏聪	2023/7/31 通道二运行G99时，将通道一的主轴最高转速用于前瞻。
#else
    //判断当前指令是否是G99模式，G98时系数为1.0
    dCoef = getBits(pToolElement->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN ? 1.0 : dG97AbsSpindleSpeed;
#endif

    //防止分母为零
    if (isZero(dG97AbsSpindleSpeed))
    {
        dCoef = 1.0;
    }

    return dCoef;
}

/*************************************************
  函数名称：correctFeedrateAndLowerLimitSpeedAgain(struct SToolCompensation* pToolElement, uint8 ucChannelNum)
  输入参数：pToolElement	       刀补后的结构体
			uint8 ucChannelNum     通道号
  输出参数：
  返 回 值：
  备    注：calcVAndAPara函数调用完毕后已经计算出最大速度要求
            再次修正切削速度和低速下限值
			进给速度不能高于最大速度、低速下限值不能高于进给速度
**************************************************/
void correctFeedrateAndLowerLimitSpeedAgain(struct SToolCompensation* pToolElement, uint8 ucChannelNum)
{
    double dCoefOfFeedrate;

    if (pToolElement == NULL)
    {
        return;
    }

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dCoefOfFeedrate =
        getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
            ? 1.0
            : calcCoefOfFeedrate(pToolElement, ucChannelNum); //2024/5/27 洪冠鹏 G0不受G98或G99影响,转换系数设为1

    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE) //自适应参数
    {
        pToolElement->dFeedrate = (pToolElement->dFeedrate * dCoefOfFeedrate) > pToolElement->dCommonVMaxFromSingleAxis
                                      ? (pToolElement->dCommonVMaxFromSingleAxis / dCoefOfFeedrate)
                                      : pToolElement->dFeedrate;
    }
    else //固定参数
    {
        if (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) //2024/5/27 洪冠鹏
        {
            if (pToolElement->ucAxisNumOfNeedLookAhead >= AXIS_NUM_MAX ||
                g_SChannel[ucChannelNum].pMapAxisArray[pToolElement->ucAxisNumOfNeedLookAhead] >= AXIS_PHYSICS_NUM_MAX)
            {
                parseRunningError(9999, GCODE_ERROR_NUM150, 9999);
                return;
            }
            pToolElement->dFeedrate =
                g_SSystemPara.SAxisPara[g_SChannel[ucChannelNum].pMapAxisArray[pToolElement->ucAxisNumOfNeedLookAhead]]
                    .dVmax;
        }
        else
        {
            pToolElement->dFeedrate = (pToolElement->dFeedrate * dCoefOfFeedrate) > g_SSystemPara.dSysMaxVel
                                          ? (g_SSystemPara.dSysMaxVel / dCoefOfFeedrate)
                                          : pToolElement->dFeedrate;
        }
    }

    pToolElement->dLowerLimitSpeed = pToolElement->dLowerLimitSpeed > (pToolElement->dFeedrate * dCoefOfFeedrate)
                                         ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                                         : pToolElement->dLowerLimitSpeed;
}

/*************************************************
  函数名称：moveToG0G01G02G03(int16 sLookAheadRes,struct SChannel* pSChannel)
  函数功能  将前瞻读写指针移动至G0/G01/ G02 /G03单元，其他单元添加加速度，速度类型信息
  输入参数：sLookAheadRes 前瞻可读区间个数
		   ucChannelNum  通道号
  输出参数：
  返 回 值：FALSE:指针未移动，开始指向需前瞻单元
			TRUE:指针移动，开始指向不需前瞻单元

  备    注：最多循环sLookAheadRes次对应前瞻可读区间全为不需要前瞻的指令
			也可能一循环，刀补写指针上来就指向需要前瞻的单元
**************************************************/
bool moveToG00G01G02G03(int16 sLookAheadRes, uint8 ucChannelNum)
{
    int16 sNum = 0; //用于记录循环次数

    struct SToolCompensation* pLookAheadRW =
        getSToolCompPointerByType(LOOKAHEAD_READWRITE, ucChannelNum); //前瞻读写指针

    int8 res = judgeG00G01G02G03(pLookAheadRW, ucChannelNum);

    while ((res == 0 || res == 2 || res == 3 || res == 4 || res == 5) && sNum < sLookAheadRes)
    {
        //不需前瞻单元中的
        if (res == 2 || res == 3 || res == 4 || res == 5) //G123
        {
            bool bPlanarArcFlag;

            if (res == 4)
            {
                bPlanarArcFlag = FALSE;
            }
            else
            {
                bPlanarArcFlag = TRUE;
            }

            //移动前瞻读写指针前对结构体速度信息赋值
            //初末速度
            //1.圆弧法向及弓高误差速度计算
            if (getBits(pLookAheadRW->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC ||
                getBits(pLookAheadRW->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
            {
                double dCoefOfFeedrate = calcCoefOfFeedrate(pLookAheadRW, ucChannelNum);
                double dF = dCoefOfFeedrate * pLookAheadRW->dFeedrate;

                pLookAheadRW->dArcMaxNormalAccVmax = calcArcMaxNormalAccVmax(pLookAheadRW->dRadius);

                pLookAheadRW->dArcMaxChordErrorVmax = calcArcMaxChordErrorVmax(pLookAheadRW->dRadius, dF);
            }

            //2.修正进给速度并计算切削进给段间低速下限值，并修正非平面圆弧第三轴是长轴时的弓高误差及法向加速度误差
            correctFeedrateAndLowerLimitSpeed(pLookAheadRW, AXIS_NUM_MAX, bPlanarArcFlag, ucChannelNum);

            //3.根据插补模式填入速度类型，加速度减速度
            //自适应参数插补模式，要多计算由各轴反算得到的参数：
            //pToolElement->dCommonVMaxFromSingleAxis
            //pToolElement->dCommonVsMinFromSingleAxis
            //pToolElement->dCommonVeMinFromSingleAxis
            calcVAndAPara(pLookAheadRW, &g_SChannel[ucChannelNum], bPlanarArcFlag);

            //4.再次修正进给速度和切削进给段间低速下限值，进给速度不能高于最大速度,低速下限值不能高于进给速度
            correctFeedrateAndLowerLimitSpeedAgain(pLookAheadRW, ucChannelNum);

            //5.校正VsVe，主要是保证vsve不超出进给速度，圆弧初末速度不能超过弓高误差和法向加速度，加入切削进给段间低速下限值约束
            pLookAheadRW->dMinVs =
                calcFirstElementVs(pLookAheadRW, g_SChannel[ucChannelNum].pMapAxisArray, ucChannelNum);
            pLookAheadRW->dMinVe =
                calcLastElementVe(pLookAheadRW, g_SChannel[ucChannelNum].pMapAxisArray, ucChannelNum);

            //6.相关速度计算完毕标志位置TRUE,即完成以上5步骤
            pLookAheadRW->bLookAheadVParaCalcOverFlag = TRUE;

            //赋值初末速度
            pLookAheadRW->dVs = pLookAheadRW->dMinVs;
            pLookAheadRW->dVe = pLookAheadRW->dMinVe;
        }

        //移动前瞻读写指针
        moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_AHEAD, ucChannelNum);

        //更改之前多通道版本的致命错误，更新pLookAheadRW指向
        pLookAheadRW = getSToolCompPointerByType(LOOKAHEAD_READWRITE, ucChannelNum); //前瞻读写指针

        //更改之前多通道版本的致命错误，更新res的值
        res = judgeG00G01G02G03(pLookAheadRW, ucChannelNum);

        sNum++;
    }

    if (sNum == 0)
    {
        return FALSE;
    }
    return TRUE;
}

/*************************************************
  函数名称：calcArcMaxNormalAccVmax(double dR)
  函数功能  计算圆弧法向加速度约束下的最大速度
  输入参数：dR 圆弧半径
  输出参数：
  返 回 值：约束速度
  备    注：
**************************************************/
double calcArcMaxNormalAccVmax(double dR)
{
    double dArcMaxNormalAcc;

    //对圆弧法向加速度参数是否为0进行判断，为0时取0.001mm/ms2，防止圆弧最大速度设置为0  郭强 2023.5.26
    if (isZero(g_SSystemPara.dArcMaxNormalAcc))
    {
        dArcMaxNormalAcc = 0.001;
    }
    else
    {
        dArcMaxNormalAcc = g_SSystemPara.dArcMaxNormalAcc;
    }

    return sqrt(dArcMaxNormalAcc * fabs(dR));
}

/*************************************************
  函数名称：calcArcMaxChordErrorVmax(double dR)
  函数功能  计算圆弧弓高误差约束下的最大速度
  输入参数：dR 圆弧半径
  输出参数：
  返 回 值：约束速度
  备    注：
**************************************************/
double calcArcMaxChordErrorVmax(double dR, double dF)
{
    double dr = fabs(dR);
    double dArcMaxChordError = g_SSystemPara.dArcMaxChordError;

    //对弓高误差进行0判断，为零时默认取0.05mm,确保圆弧插补能够运动起来 郭强 2023.5.26
    if (isZero(dArcMaxChordError))
    {
        dArcMaxChordError = 0.05;
    }

    //分三种情况对弓高误差和半径的大小关系情况进行处理      郭强 2023.5.26
    if (!(dr < dArcMaxChordError)) //dr >= dArcMaxChordError
    {
        return 2 * dr * acos((dr - dArcMaxChordError) / dr) / g_SSystemPara.dInterpolationTime;
    }
    else if ((dArcMaxChordError > dr) && (!(dArcMaxChordError > 2 * dr))) //   dr < dArcMaxChordError <= 2 * dr
    {
        return (2 * PI * dr - 2 * dr * acos((dArcMaxChordError - dr) / dr)) / g_SSystemPara.dInterpolationTime;
    }
    else //dArcMaxChordError > 2 * dr，此时弓高误差超过了直径值，直接返回进给速度
    {
        return dF;
    }
}

/*************************************************
  函数名称：correctVeerAfterCalcVeer(uint8 ucChannelNum, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj)
  函数功能  纯几何Veer计算后，对Veer进行修正
  输入参数：
		   pPreTraj ： 衔接点前段轨迹指针
		   pLaterTraj：衔接点后一段轨迹指针
		   uint8 ucChannelNum ：通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void correctVeerAfterCalcVeer(uint8 ucChannelNum, struct SToolCompensation* pPreTraj,
                              struct SToolCompensation* pLaterTraj)
{
    double dV;

    double dPreCoefOfFeedrate;
    double dLaterCoefOfFeedrate;

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    //2024.5.25增加G0的处理
    dPreCoefOfFeedrate = getBits(pPreTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
                             ? 1.0
                             : calcCoefOfFeedrate(pPreTraj, ucChannelNum);
    dLaterCoefOfFeedrate = getBits(pLaterTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
                               ? 1.0
                               : calcCoefOfFeedrate(pLaterTraj, ucChannelNum);

    if (pPreTraj == NULL || pLaterTraj == NULL)
    {
        return;
    }

    //1.在前段dMinVe和后段dMinVs中选择一个较小值
    dV = pPreTraj->dMinVe < pLaterTraj->dMinVs ? pPreTraj->dMinVe : pLaterTraj->dMinVs;

    //2.取Veer和dV中的较大值
    dV = pPreTraj->dVeer > dV ? pPreTraj->dVeer : dV;

    //3.dV不能超过前后段的进给速度
    dV = dV > (pPreTraj->dFeedrate * dPreCoefOfFeedrate) ? (pPreTraj->dFeedrate * dPreCoefOfFeedrate) : dV;
    dV = dV > (pLaterTraj->dFeedrate * dLaterCoefOfFeedrate) ? (pLaterTraj->dFeedrate * dLaterCoefOfFeedrate) : dV;

    //4.弓高误差、最大法向加速度比较，取最小值
    if (getBits(pPreTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
        getBits(pPreTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
    {
        dV = dV > pPreTraj->dArcMaxChordErrorVmax ? pPreTraj->dArcMaxChordErrorVmax : dV;
        dV = dV > pPreTraj->dArcMaxNormalAccVmax ? pPreTraj->dArcMaxNormalAccVmax : dV;
    }

    if (getBits(pLaterTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
        getBits(pLaterTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
    {
        dV = dV > pLaterTraj->dArcMaxChordErrorVmax ? pLaterTraj->dArcMaxChordErrorVmax : dV;
        dV = dV > pLaterTraj->dArcMaxNormalAccVmax ? pLaterTraj->dArcMaxNormalAccVmax : dV;
    }

    //5.将修正后的dV赋给前段dVeer
    pPreTraj->dVeer = dV;
}

/*************************************************
  函数名称：calcMinFeedrateBtnPreAndLaterTraj(uint8 ucChannelNum, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj)
  函数功能 计算前后段轨迹进给速度最小值
  输入参数：
		   pPreTraj ： 衔接点前段轨迹指针
		   pLaterTraj：衔接点后一段轨迹指针
		   uint8 ucChannelNum ：通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
double calcMinFeedrateBtnPreAndLaterTraj(uint8 ucChannelNum, struct SToolCompensation* pPreTraj,
                                         struct SToolCompensation* pLaterTraj)
{
    double dV;

    double dPreCoefOfFeedrate;
    double dLaterCoefOfFeedrate;

    if (pPreTraj == NULL || pLaterTraj == NULL)
    {
        return 0;
    }

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dPreCoefOfFeedrate = getBits(pPreTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
                             ? 1.0
                             : calcCoefOfFeedrate(pPreTraj, ucChannelNum);
    dLaterCoefOfFeedrate = getBits(pLaterTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
                               ? 1.0
                               : calcCoefOfFeedrate(pLaterTraj, ucChannelNum);

    //dV取前后轨迹段的进给速度极小值
    dV = min(pPreTraj->dFeedrate * dPreCoefOfFeedrate, pLaterTraj->dFeedrate * dLaterCoefOfFeedrate);

    return dV;
}

/*************************************************
  函数名称：calcVeerLinearAxisMode(struct SChannel* pSChannel, struct SToolCompensation *pPreTraj , struct SToolCompensation *pLaterTraj)
  函数功能  计算两相邻轨迹衔接点角度约束速度
  输入参数：pSChannel:  通道指针
		   pPreTraj ： 衔接点前段轨迹指针
		   pLaterTraj：衔接点后一段轨迹指针
  输出参数：
  返 回 值：TRUE ：传入指针不为空
			FALSE:传入指针为空，不计算
  备    注：多通道情况下，暂时前瞻XYZ三个直线轴
**************************************************/
bool calcVeerLinearAxisMode(struct SChannel* pSChannel, struct SToolCompensation* pPreTraj,
                            struct SToolCompensation* pLaterTraj)
{
    //分别存放前后轨迹XYZ向量分量的乘积和差（前段轨迹的终点方向向量 - 后段轨迹的起点方向向量）
    double dES_Vector_MultiArray[3];
    double dES_Vector_SubArray[3];
    const uint8* ucMapArray = pSChannel->pMapAxisArray; //通道轴映射数组

    double dPreCoefOfFeedrate;

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dPreCoefOfFeedrate =
        getBits(pPreTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
            ? 1.0
            : calcCoefOfFeedrate(pPreTraj, pSChannel->ucChannelNum); //2024/5/25 增加对G0指令的处理 洪冠鹏

    //指针判定
    if (pPreTraj == NULL || pLaterTraj == NULL)
    {
        return FALSE;
    }

    //取值
    //相邻轨迹方向向量某轴对应分量的乘积为负,则该轴运动方向发生改变
    dES_Vector_MultiArray[VECTOR_N1] = pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[0]] *
                                       pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[0]];
    dES_Vector_MultiArray[VECTOR_N2] = pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[1]] *
                                       pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[1]];
    dES_Vector_MultiArray[VECTOR_N3] = pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[2]] *
                                       pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[2]];

    //方向向量对应分量作差判断是否为零
    dES_Vector_SubArray[VECTOR_N1] = pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[0]] -
                                     pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[0]];
    dES_Vector_SubArray[VECTOR_N2] = pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[1]] -
                                     pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[1]];
    dES_Vector_SubArray[VECTOR_N3] = pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[2]] -
                                     pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[2]];

    //存在单轴方向改变
    if (dES_Vector_MultiArray[VECTOR_N1] < 0 || dES_Vector_MultiArray[VECTOR_N2] < 0 ||
        dES_Vector_MultiArray[VECTOR_N3] < 0)
    {
        calcVeerWhenAxisRev(pPreTraj, pSChannel->ucChannelNum, dES_Vector_MultiArray, dES_Vector_SubArray, ucMapArray);
    }
    else //不存在单轴运动方向的改变
    {
        //半角正弦，用于计算夹角速度
        double dSin_half_angle =
            0.5 * sqrt(pow(dES_Vector_SubArray[VECTOR_N1], 2) + pow(dES_Vector_SubArray[VECTOR_N2], 2) +
                       pow(dES_Vector_SubArray[VECTOR_N3], 2));

        //判断夹角是否为零
        if (isZero(dSin_half_angle)) //夹角为零
        {
            pPreTraj->dVeer = pPreTraj->dFeedrate * dPreCoefOfFeedrate;
        }
        else //夹角不为零
        {
            //double dAcc = getMaxAcc(g_SSystemPara.ucCommonAccVelPlanType);
            double dAcc = (pPreTraj->dAcc + pLaterTraj->dAcc) / 2; //由于自适应参数法前后段的加速度不同这里取平均值

            //夹角不为零时速度公式，时间常数用插补周期
            double dV = dAcc * g_SSystemPara.dInterpolationTime / 2 / dSin_half_angle;

            //判断下一段轨迹一个插补周期是否可以插补完
            if (pLaterTraj->dLength / dV < g_SSystemPara.dInterpolationTime) //不足一个插补周期就可插补完
            {
                //此时时间常数不为插补周期
                pPreTraj->dVeer = sqrt(dAcc * pLaterTraj->dLength / 2 / dSin_half_angle);
            }
            else
            {
                pPreTraj->dVeer = dV;
            }
        }
    }

    //以上为纯几何意义上的Veer,计算完毕后要对其进行修正，主要保证最终的Veer不小于前段的dMinVe，后段的dMinVs，又不高于前后段进给速度，圆弧两个误差
    correctVeerAfterCalcVeer(pSChannel->ucChannelNum, pPreTraj, pLaterTraj);

    return TRUE;
}

/*************************************************
  函数名称：initMultiAndSubArray(int nArraySize,double* dES_Vector_MultiArray, double* dES_Vector_SubArray, struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj)
  函数功能  初始化上方两数组
  输入参数：nArraySize: 轴数（数组长度）
		   dES_Vector_MultiArray ： 数组指针
		   dES_Vector_SubArray   ： 数组指针
		   pPreTraj ： 衔接点前段轨迹指针
		   pLaterTraj：衔接点后一段轨迹指针
  输出参数：
		   dES_Vector_MultiArray ： 数组指针 局部变量
		   dES_Vector_SubArray   ： 数组指针 局部变量
  返 回 值： 1：传入指针不为空
			-1:传入指针为空，不计算
  备    注：
**************************************************/
int8 initMultiAndSubArray(int nArraySize, double* dES_Vector_MultiArray, double* dES_Vector_SubArray,
                          struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj)
{
    int i = 0;
    if (dES_Vector_MultiArray == NULL || dES_Vector_SubArray == NULL || pPreTraj == NULL || pLaterTraj == NULL)
    {
        return -1;
    }

    for (i = 0; i < nArraySize; i++)
    {
        dES_Vector_MultiArray[i] = pPreTraj->SEndVector.dArray[i] * pLaterTraj->SStartVector.dArray[i];
        dES_Vector_SubArray[i] = pPreTraj->SEndVector.dArray[i] - pLaterTraj->SStartVector.dArray[i];
    }

    return 1;
}

/*************************************************
  函数名称：calcVeerMultiAxisMode(struct SChannel* pSChannel, struct SToolCompensation *pPreTraj , struct SToolCompensation *pLaterTraj)
  函数功能  计算两相邻轨迹衔接点角度约束速度
  输入参数：pSChannel:  通道指针
		   nArraySize: 轴数
		   pPreTraj ： 衔接点前段轨迹指针
		   pLaterTraj：衔接点后一段轨迹指针
  输出参数：
  返 回 值：TRUE ：传入指针不为空
			FALSE:传入指针为空，不计算
  备    注：段间衔接点速度由各单轴速度变化性能计算得到
**************************************************/
bool calcVeerMultiAxisMode(struct SChannel* pSChannel, int nArraySize, struct SToolCompensation* pPreTraj,
                           struct SToolCompensation* pLaterTraj)
{
    int i;
    const uint8* ucMapArray = pSChannel->pMapAxisArray; //通道轴映射数组
    double dTc = g_SSystemPara.dInterpolationTime;      //插补周期

    //分别存放前后轨迹向量分量的乘积和差（前段轨迹的终点方向向量 - 后段轨迹的起点方向向量）
    double dES_Vector_MultiArray[AXIS_NUM_MAX];
    double dES_Vector_SubArray[AXIS_NUM_MAX];

    //存放由各单轴反算的衔接点复合当量速度
    double dVeerFromSingleAxis[AXIS_NUM_MAX] = {0.0};

    //当dES_Vector_SubArray[i] = 0时，为避免除0填入的速度(初始值为前后段轨迹的进给速度最小值) 2023.6.23 郭强
    double dTempVeer = calcMinFeedrateBtnPreAndLaterTraj(pSChannel->ucChannelNum, pPreTraj, pLaterTraj);

    //dES_Vector_SubArray[i] = 0 标记数组，当为0时置1
    int nSubZeroFlag[AXIS_NUM_MAX] = {0}; //全部初始化为0；

    //指针判定
    if (pPreTraj == NULL || pLaterTraj == NULL)
    {
        return FALSE;
    }

    //初始化上方两数组
    initMultiAndSubArray(nArraySize, dES_Vector_MultiArray, dES_Vector_SubArray, pPreTraj, pLaterTraj);

    //判断单轴是否反向根据反向非反向计算各单轴速度变化约束下对应的衔接点复合速度，取最小值（注意有些轴在前后段间速度变化量为0，此时注意分母为零的特殊情况）
    for (i = 0; i < nArraySize; i++)
    {
        //比较特殊的情况是某些轴在前后段轨迹中的比例不变(k1 - k2 = 0)
        // 情况1：有可能此轴前后都参与插补但前段终点和后段起点比例系数相等（k1 = k2 但k1 != 0）
        // 情况2：也有可能前后两段此轴均不参与插补）(k1 = k2 = 0)
        // 此时理论上无论衔接点复合速度多大，该轴都不会超过约束，要注意此时该轴对应dVeerFromSingleAxis[]数组应该填入多大值

        if (dES_Vector_MultiArray[i] < 0) //此时当前轴反向（不包括 k1 - k2 = 0）
        {
            dVeerFromSingleAxis[i] =
                g_SSystemPara.SAxisPara[ucMapArray[i]].dVeerDeltaVMax / fabs(dES_Vector_SubArray[i]);

            //与dTempVeer对比，若dVeerFromSingleAxis[i]比dTempVeer大，更新dTempVeer为较大值
            dTempVeer = dTempVeer < dVeerFromSingleAxis[i] ? dVeerFromSingleAxis[i] : dTempVeer;
        }
        else //无反向(注意：存在k1 - k2 = 0的特殊情况)
        {
            if (isZero(dES_Vector_SubArray[i])) //k1 - k2 = 0
            {
                //标记数组置1；
                nSubZeroFlag[i] = 1;
            }
            else
            {
                dVeerFromSingleAxis[i] =
                    g_SSystemPara.SAxisPara[ucMapArray[i]].dVeerAMax * dTc / fabs(dES_Vector_SubArray[i]);

                //与dTempVeer对比，若dVeerFromSingleAxis[i]比dTempVeer大，更新dTempVeer为较大值
                dTempVeer = dTempVeer < dVeerFromSingleAxis[i] ? dVeerFromSingleAxis[i] : dTempVeer;
            }
        }
    }

    //对于isZero(dES_Vector_SubArray[i])的轴对应的dVeerFromSingleAxis赋值
    for (i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (nSubZeroFlag[i] == 1)
        {
            dVeerFromSingleAxis[i] = dTempVeer;
        }
    }

    //循环比较各单轴反算速度，取最小值
    pPreTraj->dVeer = dVeerFromSingleAxis[0];
    for (i = 0; i < AXIS_NUM_MAX - 1; i++)
    {
        pPreTraj->dVeer = pPreTraj->dVeer < dVeerFromSingleAxis[i + 1] ? pPreTraj->dVeer : dVeerFromSingleAxis[i + 1];
    }

    //单轴速度变化量上分析的dVeer计算完毕，要对其进行修正，主要保证最终的Veer不小于前段的dMinVe，后段的dMinVs，又不高于前后段进给速度，圆弧两个误差
    correctVeerAfterCalcVeer(pSChannel->ucChannelNum, pPreTraj, pLaterTraj);

    return TRUE;
}

/*************************************************
  函数名称：getCoef()
  函数功能: 根据轨迹速度曲线类型，获取位移速度计算系数
  输入参数：uint8 ucVelType 速度类型
  输出参数：
  返 回 值：计算位移速度公式中的参数
  备    注：
**************************************************/
double getCoef(uint8 ucVelType)
{
    //从系统参数结构体中获取速度曲线类型
    //uint8 ucVelType = g_SSystemPara.ucCommonAccVelPlanType;

    //获取计算系数
    if (ucVelType == VEL_PLAN_T_CURVE) //T型
    {
        return 0.5;
    }
    else if (ucVelType == VEL_PLAN_S3_CURVE) //三次S型
    {
        return 1.0;
    }
    else //四次S型
    {
        return 0.75;
    }
}

/*************************************************
  函数名称：getSingleAxisMaxAcc(int i, uint8 ucVelType, uint8* ucMapArray)
  函数功能: 根据第一层映射的轴号，速度类型获取该轴最大加速度
  输入参数：i            第一层映射的轴号
		   ucVelType    速度类型
		   ucMapArray   周映射
  输出参数：
  返 回 值：加速度值
  备    注：
**************************************************/
double getSingleAxisMaxAcc(int i, uint8 ucVelType, uint8* ucMapArray)
{
    if (ucMapArray == NULL || i >= AXIS_NUM_MAX || ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX) //2023.3.23对空轴判断郭强
    {
        return 0.0;
    }

    switch (ucVelType)
    {
    case VEL_PLAN_T_CURVE:
        return g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxAcc;
        break;
    case VEL_PLAN_S3_CURVE:
        return g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanS3MaxAcc;
        break;
    default:
        return g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanS4MaxAcc;
        break;
    }
}

/*************************************************
  函数名称：getSingleAxisMaxDec(int i, uint8 ucVelType, uint8* ucMapArray)
  函数功能: 根据第一层映射的轴号，速度类型获取该轴最大减速度
  输入参数：i            第一层映射的轴号
		   ucVelType    速度类型
		   ucMapArray   周映射
  输出参数：
  返 回 值：减速度值
  备    注：
**************************************************/
double getSingleAxisMaxDec(int i, uint8 ucVelType, uint8* ucMapArray)
{
    if (ucMapArray == NULL || i >= AXIS_NUM_MAX || ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX) //2023.3.23对空轴判断郭强
    {
        return 0.0;
    }

    switch (ucVelType)
    {
    case VEL_PLAN_T_CURVE:
        return g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanTMaxDec;
        break;
    case VEL_PLAN_S3_CURVE:
        return g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanS3MaxDec;
        break;
    default:
        return g_SSystemPara.SAxisPara[ucMapArray[i]].dSysPlanS4MaxDec;
        break;
    }
}

/*************************************************
  函数名称：calcMinTwoArrays(double* dArray1, double* dArray2)
  函数功能: 求两数组比值结果最小值（相同位置数值的比值）
  输入参数：dArray1    分子数组名
		   dArray2    分母数组名
  输出参数：
  返 回 值：比值中的最小值
  备    注：主要处理了分母为零时的问题
**************************************************/
double calcMinTwoArrays(double* dArray1, double* dArray2)
{
    int i = 0;
    double res = 0;
    //存放由各单轴反算的衔接点复合当量速度
    double dNum[AXIS_NUM_MAX] = {0};

    //当dES_Vector_SubArray[i] = 0时，为避免除0填入的速度(初始值为0.05)
    double dTemp = 0.05;

    //第二个数组相应位的值为0时置1
    int nSubZeroFlag[AXIS_NUM_MAX] = {0}; //全部初始化为0；

    for (int i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (!isZero(dArray2[i]))
        {
            dNum[i] = fabs(dArray1[i] / dArray2[i]);
            dTemp = dTemp > dNum[i] ? dTemp : dNum[i]; //让dTemp始终保持最大
        }
        else
        {
            nSubZeroFlag[i] = 1;
        }
    }

    //将dArray2位置为零的值全部置一个最大值dTemp，因为最重要取最小值
    for (i = 0; i < AXIS_NUM_MAX; i++)
    {
        if (nSubZeroFlag[i] == 1)
        {
            dNum[i] = dTemp;
        }
    }

    //循环得到最小值返回
    res = dNum[0];
    for (i = 0; i < AXIS_NUM_MAX - 1; i++)
    {
        res = res < dNum[i + 1] ? res : dNum[i + 1];
    }
    return res;
}

/*************************************************
  函数名称：getMaxAcc(uint8 ucVelType)
  函数功能: 根据轨迹加速曲线类型，获取加速段最大加速度
  输入参数：uint8 ucVelType 速度类型
  输出参数：
  返 回 值：最大加速度
  备    注：固定参数复合运动参数
**************************************************/
double getMaxAcc(uint8 ucVelType)
{
    if (ucVelType == VEL_PLAN_T_CURVE) //T型
    {
        return g_SSystemPara.dSysPlanTMaxAcc;
    }
    else if (ucVelType == VEL_PLAN_S3_CURVE) //三次S型
    {
        return g_SSystemPara.dSysPlanS3MaxAcc; //S3型曲线加减速度
    }
    else //四次S型
    {
        return g_SSystemPara.dSysPlanS4MaxAcc; //S4型曲线加减速度
    }
}

/*************************************************
  函数名称：getMaxDec(uint8 ucVelType)
  函数功能: 根据轨迹减速曲线类型，获取加速段最大减速度
  输入参数：uint8 ucVelType 速度类型
  输出参数：
  返 回 值：最大减速度
  备    注：固定参数复合运动参数
**************************************************/
double getMaxDec(uint8 ucVelType)
{
    if (ucVelType == VEL_PLAN_T_CURVE) //T型
    {
        return g_SSystemPara.dSysPlanTMaxDec;
    }
    else if (ucVelType == VEL_PLAN_S3_CURVE) //三次S型
    {
        return g_SSystemPara.dSysPlanS3MaxDec;
    }
    else //四次S型
    {
        return g_SSystemPara.dSysPlanS4MaxDec;
    }
}

/*************************************************
  函数名称：double calcVac(struct SToolCompensation *p , struct SChannel* struct SChannel* pSChannel);
  函数功能: 计算衔接点前段轨迹长度约束速度
  输入参数：p ：轨迹指针
		   struct SChannel* pSChannel :通道指针
  输出参数：
  返 回 值：  -1：指针为空
			正值：衔接点前段轨迹长度约束速度
  备    注：长度约束速度只保证了不大于最大速度和圆弧弓高及法向加速度约束，为保证进给速度，但不用担心，
			最终取得是dVac,veer,dVde三者中极小值，veer在计算后进行了修正，保证了不大于前后段的进给速度
**************************************************/
double calcVac(struct SToolCompensation* p, struct SChannel* pSChannel)
{
    double dSac; //加速距离
    double dAcc;
    double dCoef;                                 //位移速度计算系数
    double dVmax;                                 //存放最大进给速度（分G98/G99）
    uint8* ucMapArray = pSChannel->pMapAxisArray; //映射
    double dLengthV;

    if (p == NULL)
    {
        return -1;
    }

    //加速度
    dAcc = p->dAcc;
    if (dAcc <= 0)
    {
        return 0;
    }
    //系数
    dCoef = getCoef(g_SSystemPara.ucCommonAccVelPlanType);

    //G98每分钟进给模式\G99每转进给模式
    //注意：这里G99主轴用的是C轴，在车铣复合四轴(此时主轴为动力刀)程序上是否有有G99待确定
    //注意:G99时dFeedrate单位为(mm/转)，旋转轴速度单位为(°/ms),注意单位转化
    dVmax = getBits(p->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN
                ? p->dFeedrate * MAX_FEED_RATIO
                : p->dFeedrate * (p->dSpindleSpeed / 60 / 1000) * MAX_FEED_RATIO;

    //与复合运动最大速度作比较，由于乘了倍率，所以要重新比较一下
    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE) //自适应参数
    {
        dVmax = dVmax > p->dCommonVMaxFromSingleAxis ? p->dCommonVMaxFromSingleAxis : dVmax;
    }
    else
    {
        dVmax = dVmax > g_SSystemPara.dSysMaxVel ? g_SSystemPara.dSysMaxVel : dVmax;
    }

    //若为圆弧指令，还要与法向加速度、弓高误差约束速度比较
    if (getBits(p->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
        getBits(p->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
    {
        dVmax = dVmax < p->dArcMaxNormalAccVmax ? dVmax : p->dArcMaxNormalAccVmax;
        dVmax = dVmax < p->dArcMaxChordErrorVmax ? dVmax : p->dArcMaxChordErrorVmax;
    }

    //计算衔接点前段轨迹长度速度约束
    dSac = dCoef * (pow(dVmax, 2) - pow(p->dVs, 2)) / dAcc;

    //长度不够加速到最大速度，重算最大速度,否则最大速度不变
    dVmax =
        (p->dLength - p->dPreDist) < dSac ? sqrt((p->dLength - p->dPreDist) * dAcc / dCoef + pow(p->dVs, 2)) : dVmax;

    //加入轨迹长度约束 2023 / 06 / 16 郭强
    dLengthV = p->dLength / g_SSystemPara.dInterpolationTime;

    dVmax = min(dVmax, dLengthV);

    return dVmax;
}

/*************************************************
  函数名称：calcVde(struct SToolCompensation *p ,struct SChannel* pSChannel)
  函数功能: 计算衔接点后段轨迹长度约束速度
  输入参数：p ：轨迹指针
		   struct SChannel* pSChannel：通道指针
  输出参数：
  返 回 值：  0：指针为空/减速度小于0
			正值：衔接点后段轨迹长度约束速度
  备    注：长度约束速度只保证了不大于最大速度和圆弧弓高及法向加速度约束，为保证进给速度，但不用担心，
			最终取得是dVac,veer,dVde三者中极小值，veer在计算后进行了修正，保证了不大于前后段的进给速度
**************************************************/
double calcVde(struct SToolCompensation* p, struct SChannel* pSChannel)
{
    double dSde; //减速距离
    double dDec;
    double dCoef; //位移速度计算系数
    double dVmax; //存放最大进给速度（分G98/G99）
    uint8* ucMapArray = pSChannel->pMapAxisArray;
    double dLengthV;

    if (p == NULL)
    {
        return 0;
    }

    //减速度
    dDec = p->dDec;

    if (dDec <= 0)
    {
        return 0;
    }

    dCoef = getCoef(g_SSystemPara.ucCommonDecVelPlanType);

    //G98每分钟进给模式\G99每转进给模式
    dVmax = getBits(p->unCommandType, CMD_FEED_ADDR) == CMD_FEED_PER_MIN
                ? p->dFeedrate * MAX_FEED_RATIO
                : p->dFeedrate * (p->dSpindleSpeed / 60 / 1000) * MAX_FEED_RATIO;

    //与复合运动最大速度作比较
    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE) //自适应参数
    {
        dVmax = dVmax > p->dCommonVMaxFromSingleAxis ? p->dCommonVMaxFromSingleAxis : dVmax;
    }
    else
    {
        dVmax = dVmax > g_SSystemPara.dSysMaxVel ? g_SSystemPara.dSysMaxVel : dVmax;
    }

    //若为圆弧指令，还要与法向加速度、弓高误差约束速度比较，此时后段轨迹的这两个约束速度要先计算出来
    if (getBits(p->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC ||
        getBits(p->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
    {
        dVmax = dVmax < p->dArcMaxNormalAccVmax ? dVmax : p->dArcMaxNormalAccVmax;
        dVmax = dVmax < p->dArcMaxChordErrorVmax ? dVmax : p->dArcMaxChordErrorVmax;
    }

    //计算衔接点后段轨迹长度速度约束
    dSde = dCoef * (pow(dVmax, 2) - pow(p->dVe, 2)) / dDec;

    //长度不够减速到最大速度，重算最大速度（变小）,否则最大速度不变
    dVmax = p->dLength < dSde ? sqrt(p->dLength * dDec / dCoef + pow(p->dVe, 2)) : dVmax;

    //加入轨迹长度约束 2023 / 06 / 16 郭强
    dLengthV = p->dLength / g_SSystemPara.dInterpolationTime;

    dVmax = min(dVmax, dLengthV);

    return dVmax;
}

/*************************************************
  函数名称：calcNeedLookAheadNum(int16 sLookAheadWIndex , int16 sLookAheadRes)
  函数功能  计算需要前瞻的连续轨迹段数
  输入参数：sLookAheadWIndex ：前瞻读写指针索引  sLookAheadRes：前瞻可读区间段数  uint8 ucChannelNum:通道号
  输出参数：
  返 回 值：sNeedLookAheadNum ：需要前瞻的连续轨迹段数
  备    注：由于刀补写入的某个结构体单元可能不需要前瞻G00,G04等，
			另外，由于M03M04MO5前后阻塞的原因，即使相邻两单元都需要前瞻，但是仍然会被阻塞打断运动
			因此需要处理多段独立前瞻区间
			加入通道等待判断
**************************************************/
int16 calcNeedLookAheadNum(int16 sLookAheadWIndex, int16 sLookAheadRes, uint8 ucChannelNum)
{
    int16 sNeedLookAheadNum = 1; //需要前瞻的连续单元数量，初始值为1，即前瞻读写指针当前指向单元
    int16 i = 1;                 //循环变量

    //动态指针索引
    int16 sPIndex;

    //动态指针 指向前瞻可读区间单元
    struct SToolCompensation* p;

    //循环计算需要前瞻的连续单元数量
    for (i = 1; i < sLookAheadRes; i++) //当sLookAheadRes=1时，不进入循环ucMapArray
    {
        sPIndex = sLookAheadWIndex + i; //从前瞻读写指针下一个开始检查
        p = getSToolCompPointerByIndex(sPIndex, ucChannelNum);

        if (judgeG00G01G02G03(p, ucChannelNum) == 1 || judgeG00G01G02G03(p, ucChannelNum) == 5)
        {
            if (!(getBits(p->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)) //不为G0指令
            {
                uint8 res = isBlockedInCommand(p->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
                //看此单元是否存在前后阻塞
                if (res == 1) //前阻塞
                {
                    return sNeedLookAheadNum; //此单元不包含在当前独立前瞻区间内
                }
                if (res == 2 || p->sChannelCommand.ucType != UNASSIGNED_VALUE) //后阻塞或有通道等待
                {
                    sNeedLookAheadNum++;
                    return sNeedLookAheadNum; //此单元包含在当前独立前瞻区间内，为最后一个单元
                }
                if (getBits(p->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE) //直线指令
                {
                    if (p->ucAxisNumOfNeedLookAhead == AXIS_NULL &&
                        judgeG0(p, ucChannelNum, sPIndex) == 0) //多轴运动，但前一单元为G0单轴运动
                    {
                        return sNeedLookAheadNum; //此单元不包含在当前独立前瞻区间内
                    }
                }
            }
            else //为G0指令
            {
                if (p->sChannelCommand.ucType != UNASSIGNED_VALUE) //有通道等待
                {
                    sNeedLookAheadNum++;
                    return sNeedLookAheadNum; //此单元包含在当前独立前瞻区间内，为最后一个单元
                }
                if (p->ucAxisNumOfNeedLookAhead != AXIS_NULL &&
                    judgeG0(p, ucChannelNum, sPIndex) !=
                        1) //单轴运动，但与前一单元轴号/方向不一致  //2026.3.28 修改为单轴运动且不为前瞻
                {
                    return sNeedLookAheadNum; //此单元不包含在当前独立前瞻区间内
                }
            }

            sNeedLookAheadNum++;
        }
        else
        {
            break;
        }
    }

    //返回需要前瞻的连续轨迹段数
    return sNeedLookAheadNum;
}

/*************************************************
  函数名称：calcFirstElementVs(struct SToolCompensation* pToolElement, uint8* ucMapArray, uint8 ucChannelNum)
  函数功能: 计算前瞻区间首段初速度
  输入参数：pToolElement 刀补缓冲区指针
            uint8 ucChannelNum  通道号
  输出参数：
  返 回 值：
  备    注：2022/11/25加入切削进给段间低速下限值约束
**************************************************/
double calcFirstElementVs(struct SToolCompensation* pToolElement, uint8* ucMapArray, uint8 ucChannelNum)
{
    double dVs = 0;
    double dCoefOfFeedrate;
    double dLengthV; //轨迹长度约束下的最大Vs

    if (pToolElement == NULL)
    {
        return 0;
    }

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dCoefOfFeedrate =
        getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
            ? 1.0
            : calcCoefOfFeedrate(pToolElement, ucChannelNum); //2024/5/27 洪冠鹏 G0不受G98或G99影响,转换系数设为1

    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE) //自适应参数
    {

        if (pToolElement->dCommonVsMinFromSingleAxis <
            pToolElement->dLowerLimitSpeed) //反算初速度比下限值小，此时用下限值
        {
            dVs = pToolElement->dLowerLimitSpeed;
        }
        else //不比下限值小,确保反算值不超过进给速度
        {
            dVs = pToolElement->dCommonVsMinFromSingleAxis > (pToolElement->dFeedrate * dCoefOfFeedrate)
                      ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                      : pToolElement->dCommonVsMinFromSingleAxis;
        }
    }
    else
    {
        if (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) //单轴的G0
        {
            if (pToolElement->ucAxisNumOfNeedLookAhead >= AXIS_NUM_MAX ||
                ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead] >= AXIS_PHYSICS_NUM_MAX)
            {
                return 0;
            }
            if (g_SSystemPara.SAxisPara[ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead]].dVs <
                pToolElement->dLowerLimitSpeed) //系统最小初速度比下限值小，此时用下限值
            {
                dVs = pToolElement->dLowerLimitSpeed;
            }
            else //不比下限值小,确保系统最小初速度不超过进给速度
            {
                dVs = g_SSystemPara.SAxisPara[ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead]].dVs >
                              (pToolElement->dFeedrate * dCoefOfFeedrate)
                          ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                          : g_SSystemPara.SAxisPara[ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead]].dVs;
            }
        }
        else //G01G02G03
        {
            if (g_SSystemPara.dSysMinVel < pToolElement->dLowerLimitSpeed) //系统最小初速度比下限值小，此时用下限值
            {
                dVs = pToolElement->dLowerLimitSpeed;
            }
            else //不比下限值小,确保系统最小初速度不超过进给速度
            {
                dVs = g_SSystemPara.dSysMinVel > (pToolElement->dFeedrate * dCoefOfFeedrate)
                          ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                          : g_SSystemPara.dSysMinVel;
            }
        }
    }

    //圆弧需要再加上弓高和最大法向加速度约束
    if ((getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC) ||
        (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC))
    {
        //dVs再与法向、弓高得到最小值
        int i;
        double dV[3] = {0}; //开辟变量存放上方三个速度，以便算出最小值

        dV[0] = dVs;
        dV[1] = pToolElement->dArcMaxNormalAccVmax;
        dV[2] = pToolElement->dArcMaxChordErrorVmax;

        for (i = 0; i < 3 - 1; i++)
        {
            dVs = dVs < dV[i + 1] ? dVs : dV[i + 1];
        }
    }

    //加入轨迹长度约束 2023/06/16 郭强
    dLengthV = pToolElement->dLength / g_SSystemPara.dInterpolationTime;

    dVs = min(dVs, dLengthV);

    return dVs;
}

/*************************************************
  函数名称：calcLastElementVe(struct SToolCompensation* pToolElement)
  函数功能: 计算前瞻区间末段末速度
  输入参数：pToolElement         前瞻读写指针
            uint8 ucChannelNum   通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
double calcLastElementVe(struct SToolCompensation* pToolElement, uint8* ucMapArray, uint8 ucChannelNum)
{
    double dVe = 0;
    double dCoefOfFeedrate;

    double dLengthV; //轨迹长度约束下的最大Vs

    if (pToolElement == NULL)
    {
        return 0;
    }

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dCoefOfFeedrate =
        getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
            ? 1.0
            : calcCoefOfFeedrate(pToolElement, ucChannelNum); //2024/5/27 洪冠鹏 G0不受G98或G99影响,转换系数设为1

    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_ADAPTIVE_PARAMETER_MODE) //自适应参数
    {

        if (pToolElement->dCommonVeMinFromSingleAxis <
            pToolElement->dLowerLimitSpeed) //反算初速度比下限值小，此时用下限值
        {
            dVe = pToolElement->dLowerLimitSpeed;
        }
        else //不比下限值小,确保反算值不超过进给速度
        {
            dVe = pToolElement->dCommonVeMinFromSingleAxis > (pToolElement->dFeedrate * dCoefOfFeedrate)
                      ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                      : pToolElement->dCommonVeMinFromSingleAxis;
        }
    }
    else
    {
        if (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE) //单轴G0
        {
            if (pToolElement->ucAxisNumOfNeedLookAhead >= AXIS_NUM_MAX ||
                ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead] >= AXIS_PHYSICS_NUM_MAX)
            {
                return 0;
            }
            if (g_SSystemPara.SAxisPara[ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead]].dVe <
                pToolElement->dLowerLimitSpeed) //系统最小末速度比下限值小，此时用下限值
            {
                dVe = pToolElement->dLowerLimitSpeed;
            }
            else //不比下限值小,确保系统最小末速度不超过进给速度
            {
                dVe = g_SSystemPara.SAxisPara[ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead]].dVe >
                              (pToolElement->dFeedrate * dCoefOfFeedrate)
                          ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                          : g_SSystemPara.SAxisPara[ucMapArray[pToolElement->ucAxisNumOfNeedLookAhead]].dVe;
            }
        }
        else //G01G02G03
        {
            if (g_SSystemPara.dSysMinVel < pToolElement->dLowerLimitSpeed) //系统最小初速度比下限值小，此时用下限值
            {
                dVe = pToolElement->dLowerLimitSpeed;
            }
            else //不比下限值小,确保系统最小初速度不超过进给速度
            {
                dVe = g_SSystemPara.dSysMinVel > (pToolElement->dFeedrate * dCoefOfFeedrate)
                          ? (pToolElement->dFeedrate * dCoefOfFeedrate)
                          : g_SSystemPara.dSysMinVel;
            }
        }
    }

    //圆弧需要再加上弓高和最大法向加速度约束
    if ((getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC) ||
        (getBits(pToolElement->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC))
    {
        //dVs再与法向、弓高得到最小值
        int i;
        double dV[3] = {0}; //开辟变量存放上方三个速度，以便算出最小值

        dV[0] = dVe;
        dV[1] = pToolElement->dArcMaxNormalAccVmax;
        dV[2] = pToolElement->dArcMaxChordErrorVmax;

        for (i = 0; i < 3 - 1; i++)
        {
            dVe = dVe < dV[i + 1] ? dVe : dV[i + 1];
        }
    }

    //加入轨迹长度约束  2023/06/16 郭强
    dLengthV = pToolElement->dLength / g_SSystemPara.dInterpolationTime;

    dVe = min(dVe, dLengthV);

    return dVe;
}

/*************************************************
  函数名称：calcVsVe(int16 sNeedLookAheadNum, struct SChannel* pSChannel)
  函数功能: 当满足前瞻计算条件时，此函数用于前瞻计算
  输入参数：sNeedLookAheadNum 需要前瞻的连续区间长度
		  struct SChannel* pSChannel 通道指针
  输出参数：
  返 回 值：1：前瞻处理独立前瞻区间最后一段
			2：前瞻处理非最后一段
  备    注：
**************************************************/
uint8 calcVsVe(int16 sNeedLookAheadNum, struct SChannel* pSChannel)
{
    //获取通道号
    uint8 ucChannelNum = pSChannel->ucChannelNum;

    //前瞻读写指针索引
    int16 sLookAheadWIndex = getSToolCompPointerIndex(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

    //前瞻读写指针
    struct SToolCompensation* pLookAheadW = getSToolCompPointerByType(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

    if (!pLookAheadW->bLookAheadVParaCalcOverFlag) //相关速度参数未计算过
    {
        //1.当当前段为平面圆弧时，计算当前段圆弧法向加速度约束速度和弓高误差约束速度
        if (getBits(pLookAheadW->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC ||
            getBits(pLookAheadW->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
        {
            double dCoefOfFeedrate = calcCoefOfFeedrate(pLookAheadW, ucChannelNum);
            double dF = dCoefOfFeedrate * pLookAheadW->dFeedrate;

            pLookAheadW->dArcMaxNormalAccVmax = calcArcMaxNormalAccVmax(pLookAheadW->dRadius);

            pLookAheadW->dArcMaxChordErrorVmax = calcArcMaxChordErrorVmax(pLookAheadW->dRadius, dF);
        }

        //2.修正进给速度并计算切削进给段间低速下限值,进入前瞻的圆弧都是平面圆弧，所以参数都是TRUE
        correctFeedrateAndLowerLimitSpeed(pLookAheadW, AXIS_NUM_MAX, TRUE, ucChannelNum);

        //3.添加当前指针的加速度，速度类型，速度信息,进入前瞻的圆弧都是平面圆弧，所以参数都是TRUE
        calcVAndAPara(pLookAheadW, pSChannel, TRUE);

        //4.再次修正进给速度和切削进给段间低速下限值，进给速度不能高于最大速度,低速下限值不能高于进给速度
        correctFeedrateAndLowerLimitSpeedAgain(pLookAheadW, ucChannelNum);

        //5.最小初速度末速度
        pLookAheadW->dMinVs = calcFirstElementVs(pLookAheadW, pSChannel->pMapAxisArray, ucChannelNum);
        pLookAheadW->dMinVe = calcLastElementVe(pLookAheadW, pSChannel->pMapAxisArray, ucChannelNum);

        //6.相关速度计算完毕标志位置TRUE,即完成以上5步骤
        pLookAheadW->bLookAheadVParaCalcOverFlag = TRUE;
    }

    //当为独立连续前瞻区间起始段时
    //独立前瞻区间：刀补缓冲区中需要前瞻的单元可能并非完全连续，
    //              被分割成若干个不连续的进给运动区间，在前瞻时，称这些区间为"独立前瞻区间"
    if (pSChannel->bLookAheadNewBeginFlag == TRUE)
    {
        pLookAheadW->dVs = pLookAheadW->dMinVs;
        //程序进入这里可对前瞻新的开始标志位置FALSE
        //注意：如不置FALSE，前瞻区间每个单元都会被认为是前瞻区间首段，
        //      初速度为最小速度，前瞻计算就会出错
        pSChannel->bLookAheadNewBeginFlag = FALSE;
    }

    //独立前瞻区间的最后一段
    if (sNeedLookAheadNum == 1)
    {

        pLookAheadW->dVe = pLookAheadW->dMinVe;
        //独立前瞻区间段处理完毕
        //之后若出现需前瞻单元，则是新的前瞻开始,
        pSChannel->bLookAheadNewBeginFlag = TRUE; //置TRUE，是为了新的独立前瞻段的设置（初速度，前瞻段数）

        //移动前瞻读写指针
        moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_AHEAD, pSChannel->ucChannelNum);

        return 1;
    }
    else //需要前瞻的连续单元个数大于一，需要反算
    {
        //系统要求最大前瞻段数
        int16 sLookAheadNum = (int16)g_SSystemPara.ucLookAheadNum;

        int16 sK; //记录前瞻段数

        double dVde; //用于记录反算过程中后段轨迹长度约束速度

        int16 sLookAheadEndIndex;                //前瞻区间最后一段索引
        struct SToolCompensation* pLookAheadEnd; //临时指针 指向前瞻区间最后一段

        int16 sNextIndex;                //前瞻写指针下一段轨迹索引
        struct SToolCompensation* pNext; //指向前瞻写指针下一段轨迹

        //得到当前段（前瞻写指向轨迹）的下一段索引
        sNextIndex = sLookAheadWIndex + 1;

        //得到当前段（前瞻写指向轨迹）的下一段指针，
        //衔接点速度计算完后赋值给此段作为dVs
        pNext = getSToolCompPointerByIndex(sNextIndex, pSChannel->ucChannelNum);

        //计算衔接点前段轨迹长度速度约束
        //pLookAheadW->dFeedrate = calcVac(pLookAheadW);
        pLookAheadW->dVmax = calcVac(pLookAheadW, pSChannel);

        //计算前瞻段数，确定前瞻计算区间
        if (sNeedLookAheadNum >= sLookAheadNum + 1) //需要前瞻的段数比系统前瞻段数还多，则按系统参数进行前瞻
        {
            sK = sLookAheadNum;
        }
        else
        {
            sK = sNeedLookAheadNum - 1;
        }

        //得到前瞻区间最后段索引
        sLookAheadEndIndex = sLookAheadWIndex + sK;

        //得到前瞻区间最后段指针
        pLookAheadEnd = getSToolCompPointerByIndex(sLookAheadEndIndex, pSChannel->ucChannelNum);

        //计算最后段单元的速度信息
        if (!pLookAheadEnd->bLookAheadVParaCalcOverFlag)
        {
            //1.当当前段为平面圆弧时，计算当前段圆弧法向加速度约束速度和弓高误差约束速度（反算过程中会重读计算，计算量不大）
            if (getBits(pLookAheadEnd->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC ||
                getBits(pLookAheadEnd->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
            {
                double dCoefOfFeedrate = calcCoefOfFeedrate(pLookAheadEnd, ucChannelNum);
                double dF = dCoefOfFeedrate * pLookAheadEnd->dFeedrate;

                pLookAheadEnd->dArcMaxNormalAccVmax = calcArcMaxNormalAccVmax(pLookAheadEnd->dRadius);
                pLookAheadEnd->dArcMaxChordErrorVmax = calcArcMaxChordErrorVmax(pLookAheadEnd->dRadius, dF);
            }

            //2.修正进给速度并计算切削进给段间低速下限值,进入前瞻的圆弧都是平面圆弧，所以参数都是TRUE
            correctFeedrateAndLowerLimitSpeed(pLookAheadEnd, AXIS_NUM_MAX, TRUE, ucChannelNum);

            //3.添加当前指针的加速度，速度类型，速度信息,进入前瞻的圆弧都是平面圆弧，所以参数都是TRUE
            calcVAndAPara(pLookAheadEnd, pSChannel, TRUE);

            //4.再次修正进给速度和切削进给段间低速下限值，进给速度不能高于最大速度,低速下限值不能高于进给速度
            correctFeedrateAndLowerLimitSpeedAgain(pLookAheadEnd, ucChannelNum);

            //5.最小初速度末速度
            pLookAheadEnd->dMinVs = calcFirstElementVs(pLookAheadEnd, pSChannel->pMapAxisArray, ucChannelNum);
            pLookAheadEnd->dMinVe = calcLastElementVe(pLookAheadEnd, pSChannel->pMapAxisArray, ucChannelNum);

            //6.相关速度计算完毕标志位置TRUE,即完成以上5步骤
            pLookAheadEnd->bLookAheadVParaCalcOverFlag = TRUE;
        }

        //令其终点速度为最小速度
        pLookAheadEnd->dVe = pLookAheadEnd->dMinVe;

        //反向计算到当前段轨迹"终点速度"为止
        //此时"终点速度"为角度约束速度和后一段长度约束速度的极小值,还需与前段轨迹长度约束速度比较
        while (sK >= 1)
        {
            int16 sTempIndex = sLookAheadWIndex + sK;        //长度约束段索引
            int16 sPreTempIndex = sLookAheadWIndex + sK - 1; //长度约束段前一段索引
            double dLengthV;

            //动态指针 指向长度约束段
            struct SToolCompensation* pTemp = getSToolCompPointerByIndex(sTempIndex, pSChannel->ucChannelNum);
            //动态指针 指向反算过程中长度约束段的前一段
            struct SToolCompensation* pPreTemp = getSToolCompPointerByIndex(sPreTempIndex, pSChannel->ucChannelNum);

            //此时只用计算pPreTemp的相关速度参数
            if (!pPreTemp->bLookAheadVParaCalcOverFlag)
            {
                //1.当为圆弧时，计算当前段圆弧法向加速度约束速度和弓高误差约束速度
                if (getBits(pPreTemp->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC ||
                    getBits(pPreTemp->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
                {
                    double dCoefOfFeedrate = calcCoefOfFeedrate(pPreTemp, ucChannelNum);
                    double dF = dCoefOfFeedrate * pPreTemp->dFeedrate;

                    pPreTemp->dArcMaxNormalAccVmax = calcArcMaxNormalAccVmax(pPreTemp->dRadius);
                    pPreTemp->dArcMaxChordErrorVmax = calcArcMaxChordErrorVmax(pPreTemp->dRadius, dF);
                }

                //2.修正进给速度并计算切削进给段间低速下限值,进入前瞻的圆弧都是平面圆弧，所以参数都是TRUE
                correctFeedrateAndLowerLimitSpeed(pPreTemp, AXIS_NUM_MAX, TRUE, ucChannelNum);

                //3.添加当前指针的加速度，速度类型，速度信息
                calcVAndAPara(pPreTemp, pSChannel, TRUE);

                //4.再次修正进给速度和切削进给段间低速下限值，进给速度不能高于最大速度,低速下限值不能高于进给速度
                correctFeedrateAndLowerLimitSpeedAgain(pPreTemp, ucChannelNum);

                //5.最小初速度末速度
                pPreTemp->dMinVs = calcFirstElementVs(pPreTemp, pSChannel->pMapAxisArray, ucChannelNum);
                pPreTemp->dMinVe = calcLastElementVe(pPreTemp, pSChannel->pMapAxisArray, ucChannelNum);

                //6.相关速度计算完毕标志位置TRUE,即完成以上5步骤
                pPreTemp->bLookAheadVParaCalcOverFlag = TRUE;
            }

            //计算此段长度约束速度（此段终点速度已算出，看此段长度约束（以及圆弧弓高和法向）下最高速度为多少）
            dVde = calcVde(pTemp, pSChannel);

            //前一段轨迹的dVeer若未计算先计算
            if (pPreTemp->bHasVeerFlag == FALSE)
            {
                if (g_SSystemPara.ucLookAheadMode == LOOKAHEAD_MULTI_AXIS_MODE) //3多轴前瞻（含旋转轴）模式
                {
                    if (calcVeerMultiAxisMode(pSChannel, AXIS_NUM_MAX, pPreTemp, pTemp) == TRUE)
                    {
                        //将该标志位置TRUE
                        pPreTemp->bHasVeerFlag = TRUE;
                    }
                }
                else //ucLookAheadMode的值1和2时采用三直线轴夹角约束计算
                {
                    if (calcVeerLinearAxisMode(pSChannel, pPreTemp, pTemp) == TRUE)
                    {
                        //将该标志位置TRUE
                        pPreTemp->bHasVeerFlag = TRUE;
                    }
                }
            }

            //比较dVde和前一段轨迹的dVeer，若前一段轨迹为圆弧还需加入法向加速度和弓高误差约束，取最小值作为前一段的终点速度
            //第一轮比较：
            pPreTemp->dVe = min(dVde, pPreTemp->dVeer);

            //加入轨迹长度约束  2023/06/16 郭强
            dLengthV = pPreTemp->dLength / g_SSystemPara.dInterpolationTime;

            pPreTemp->dVe = min(pPreTemp->dVe, dLengthV);

            sK--;
        }

        //比较衔接点处前段轨迹长度约束速度和反算过程中的dVe,取最小值作为衔接点速度
        pLookAheadW->dVe = min(pLookAheadW->dVe, pLookAheadW->dVmax);

        //当前点终点速度赋值给下一段起点
        pNext->dVs = pLookAheadW->dVe;

        //移动前瞻读写指针
        moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_AHEAD, pSChannel->ucChannelNum);

        return 2;
    }
}

/*************************************************
  函数名称：noLookAheadMode()
  函数功能  不进行前瞻模式
  输入参数：uint8  ucChannelNum 通道号
  输出参数：
  返 回 值：TRUE  前瞻指针移动
  备    注：长轴模式调用无前瞻模式
**************************************************/
bool noLookAheadMode(uint8 ucChannelNum)
{
    double dCoefOfFeedrate;

    //刀补指针
    struct SToolCompensation* pLookAheadRW = getSToolCompPointerByType(LOOKAHEAD_READWRITE, ucChannelNum);

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dCoefOfFeedrate = getBits(pLookAheadRW->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
                          ? 1.0
                          : calcCoefOfFeedrate(pLookAheadRW, ucChannelNum);

    //当要求速度比系统复合运动最小速度参数小时，VeVs与要求的dFeedrate相同
    pLookAheadRW->dVs = pLookAheadRW->dVe = (pLookAheadRW->dFeedrate * dCoefOfFeedrate) > g_SSystemPara.dSysMinVel
                                                ? g_SSystemPara.dSysMinVel
                                                : (pLookAheadRW->dFeedrate * dCoefOfFeedrate);
    pLookAheadRW->dVmax = g_SSystemPara.dSysMaxVel;                        //最大速度
    pLookAheadRW->dAcc = getMaxAcc(g_SSystemPara.ucCommonAccVelPlanType);  //加速度
    pLookAheadRW->dDec = getMaxDec(g_SSystemPara.ucCommonDecVelPlanType);  //减速度
    pLookAheadRW->ucAccVelPlanType = g_SSystemPara.ucCommonAccVelPlanType; //加速类型
    pLookAheadRW->ucDecVelPlanType = g_SSystemPara.ucCommonDecVelPlanType; //减速类型

    pLookAheadRW->dMinVs = pLookAheadRW->dMinVe = g_SSystemPara.dSysMinVel;

    //移动指针
    moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_AHEAD, ucChannelNum);
    return TRUE;
}

/*************************************************
  函数名称：lookAheadMode(int16 sLookAheadRes,struct SChannel* pSChannel)
  函数功能  前瞻模式
  输入参数：sLookAheadRes 前瞻可读段数
		   pSChannel 通道指针
  输出参数：
  返 回 值：1:进行前瞻了
		   -1：未进行前瞻
  备    注：
**************************************************/
int8 lookAheadMode(int16 sLookAheadRes, struct SChannel* pSChannel)
{
    //系统要求最大前瞻段数
    int16 sLookAheadNum = (int16)g_SSystemPara.ucLookAheadNum;

    //接下来要判断是不是要前瞻，前瞻段数多少

    //前瞻读写指针索引 用于计算sNeedLookAheadNum
    int16 sLookAheadWIndex = getSToolCompPointerIndex(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

    //需要前瞻的连续单元数量（可以前瞻段数）
    int16 sNeedLookAheadNum = calcNeedLookAheadNum(sLookAheadWIndex, sLookAheadRes, pSChannel->ucChannelNum);

    //与前瞻可读段数比较可判断前瞻可读区间是否有不需要前瞻的单元存在
    if (sNeedLookAheadNum < sLookAheadRes) //有不需要前瞻的单元,此时必须要进行前瞻计算
    {
        calcVsVe(sNeedLookAheadNum, pSChannel);

        return 1;
    }
    else //前瞻可读区间全部为需要前瞻的轨迹
    {
        //最小前瞻段数，当刀补未完全写入且前瞻可读区间全为需要前瞻的轨迹且非第一次前瞻处理时，
        //前瞻区间达到此值即可前瞻处理(系统最大段数 - 5)
        int16 sMinLookAheadNum = sLookAheadNum - 5;

        //前瞻区间段数限制变量
        int16 sLookAheadLimitNum;

        if (pSChannel->bLookAheadNewBeginFlag == TRUE)
        {
            //前瞻可读区间全需要前瞻，前瞻第一次写且刀补未完全写入时，前瞻区间达到sLookAheadNum为前瞻开始的条件
            sLookAheadLimitNum = sLookAheadNum;
        }
        else
        {
            //前瞻可读区间全需要前瞻，非第一次前瞻写，当刀补未完全写入时，前瞻区间达到sMinLookAheadNum即可开始前瞻
            sLookAheadLimitNum = sMinLookAheadNum;
        }

        //刀补未写入完且前瞻区间段数不满足要求
        if (pSChannel->bToolWriteOverFlag == FALSE && sNeedLookAheadNum < sLookAheadLimitNum + 1)
        {
            return -1; //此时不前瞻计算
        }
        else
        {
            //此时对应两种情况,此两种情况均可继续前瞻写
            //1.(sNeedLookAheadNum >= sLookAheadLimitNum + 1)//需要前瞻的区间长度达到要求前瞻段数
            //2.(g_bToolWriteOverFlag == TRUE &&  sNeedLookAheadNum < sLookAheadLimitNum + 1)刀补写入完毕且需要前瞻区间未达到要求前瞻段数

            calcVsVe(sNeedLookAheadNum, pSChannel);
            return 1;
        }
    }
}

/*************************************************
  函数名称：lookAheadForbidden(struct SChannel* pSChannel)
  函数功能  不前瞻(直接填入系统参数)
  输入参数：pSChannel通道指针
  输出参数：
  返 回 值：TRUE  ：进行了处理（进行了赋值）
			FALSE ：没有处理
  备    注：
  **************************************************/
bool lookAheadForbidden(struct SChannel* pSChannel)
{
    //前瞻段数
    int16 sLookAheadRes = getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

    //大于0  赋值
    if (sLookAheadRes > 0)
    {
        //刀补指针
        struct SToolCompensation* pSToolPointer =
            getSToolCompPointerByType(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

        pSToolPointer->dVs = pSToolPointer->dVe = g_SSystemPara.dSysMinVel;     //起始速度、终止速度
        pSToolPointer->dVmax = g_SSystemPara.dSysMaxVel;                        //最大速度
        pSToolPointer->dAcc = getMaxAcc(g_SSystemPara.ucCommonAccVelPlanType);  //加速度
        pSToolPointer->dDec = getMaxDec(g_SSystemPara.ucCommonDecVelPlanType);  //减速度
        pSToolPointer->ucAccVelPlanType = g_SSystemPara.ucCommonAccVelPlanType; //加速类型
        pSToolPointer->ucDecVelPlanType = g_SSystemPara.ucCommonDecVelPlanType; //减速类型

        //移动指针
        moveSToolCompBufPointer(LOOKAHEAD_READWRITE, BUF_AHEAD, pSChannel->ucChannelNum);
        return TRUE;
    }
    return FALSE;
}

/*************************************************
  函数名称：velocityLookAhead(struct SChannel* pSChannel)
  函数功能  速度前瞻
  输入参数：pSChannel通道指针
  输出参数：
  返 回 值：TRUE  ：进行了前瞻处理（对需要前瞻的单元进行了赋值）
			FALSE ：没有前瞻处理（包括只跳过不需前瞻单元，不前瞻处理的情况）
  备    注：速度前瞻函数要判断当前状态是否能够进行前瞻处理，前瞻处理后将信息写入刀补缓冲区
**************************************************/
bool velocityLookAhead(struct SChannel* pSChannel)
{
    //前瞻可读段数
    int16 sLookAheadRes = getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

    //前瞻无可读段数
    if (pSChannel == NULL || sLookAheadRes <= 0)
    {
        return FALSE;
    }

    if (g_SSystemPara.ucInterpolationMode == INTERPOLATION_LONGAXIS_MODE) //长轴插补模式采用无前瞻模式
    {
        noLookAheadMode(pSChannel->ucChannelNum);
        return TRUE;
    }

    //判断系统状态若为从运行到暂停，不前瞻（多通道状态模式系统状态改为通道结构体成员）
    if (pSChannel->ucChannelStatus == SYS_RUN_TO_PAUSE)
    {
        pSChannel->bLookAheadNewBeginFlag = TRUE;
        return FALSE;
    }

    //移动前瞻读写指针直至到达需要前瞻单元
    //特殊情况：1.当可读区间没有需要前瞻的单元，前瞻读写指针移动到刀补写指针处
    //          2.前瞻指针不需移动，因为一开始就指向需要前瞻的单元
    moveToG00G01G02G03(sLookAheadRes, pSChannel->ucChannelNum);

    //重新判断前瞻可读段数
    sLookAheadRes = getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, pSChannel->ucChannelNum);

    if (sLookAheadRes <= 0)
    {
        return FALSE;
    }
    else //此时对应的前瞻可读区间内含有需要前瞻的单元（且刀补读写最初就指向了需要前瞻的单元）
    {

        int8 res = 0;

        //前瞻
        res = lookAheadMode(sLookAheadRes, pSChannel);

        if (res == 1) //进行了前瞻处理
        {
            return TRUE;
        }
        return FALSE;
    }
}

//测试用求衔点角度
/*************************************************
  函数名称：calcAngle(struct SToolCompensation *pPreTraj , struct SToolCompensation *pLaterTraj)
  函数功能  测试用求衔点角度
  输入参数：pPreTraj ： 衔接点前段轨迹指针  pLaterTraj：衔接点后一段轨迹指针
  输出参数：
  返 回 值：返回的是°（非弧度）
  备    注：（笛卡尔空间，XYZ三轴）
**************************************************/
double calcAngle(struct SToolCompensation* pPreTraj, struct SToolCompensation* pLaterTraj)
{
    //半角正弦
    double dSin_half_angle = 0.5 * sqrt(pow(pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[0]] -
                                                pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[0]],
                                            2) +
                                        pow(pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[1]] -
                                                pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[1]],
                                            2) +
                                        pow(pPreTraj->SEndVector.dArray[g_ucLookAheadAxisArray[2]] -
                                                pLaterTraj->SStartVector.dArray[g_ucLookAheadAxisArray[2]],
                                            2));

    //角度
    return acos(1 - 2 * pow(dSin_half_angle, 2)) * 180.0 / PI;
}

/*************************************************
  函数名称：calcVeerWhenAxisRev(struct SToolCompensation* pPreTraj, double* dES_Vector_MultiArray, double* dES_Vector_SubArray, const uint8* ucMapArray)
  函数功能  单轴反向时VEER计算
  输入参数：pPreTraj:  衔接点前段轨迹指针
		   dES_Vector_MultiArray ： 向量分量相乘数组指针
		   dES_Vector_SubArray：    向量分量相减数组指针
		   const uint8* ucMapArray ：     当前通道轴映射数组指针
			ucChannelNum        通道号
  输出参数：
  返 回 值：TRUE ：传入指针不为空
			FALSE:传入指针为空，不计算
  备    注：多通道情况下，暂时前瞻三个轴
**************************************************/
bool calcVeerWhenAxisRev(struct SToolCompensation* pPreTraj, uint8 ucChannelNum, double* dES_Vector_MultiArray,
                         double* dES_Vector_SubArray, const uint8* ucMapArray)
{
    if (pPreTraj == NULL || dES_Vector_MultiArray == NULL || dES_Vector_SubArray == NULL || ucMapArray == NULL)
    {
        return FALSE;
    }

    //根据各轴反向情况分情况计算角度约束速度
    //从系统参数结构体中获取需要的参数    单轴转向允许变化的最大速度 mm/s
    //AXIS_X  AXIS_Z 在common.h中宏定义
    double dVeerDeltaVMaxArrays[LINEARAXIS_NUM_MAX]; //过渡速度数组
    double dAxisAccArrays[LINEARAXIS_NUM_MAX];       //加速度数组
    double dVeercArrays[LINEARAXIS_NUM_MAX];         //过渡速度数组
    double dVeercVaildArrays[LINEARAXIS_NUM_MAX];    //过渡速度有效数组
    int8 cValidAxis[LINEARAXIS_NUM_MAX];
    int8 cArraySize = 0;
    int8 cArrayIndex = 0;

    memset(dVeerDeltaVMaxArrays, 0, sizeof(double) * LINEARAXIS_NUM_MAX);
    memset(dAxisAccArrays, 0, sizeof(double) * LINEARAXIS_NUM_MAX);
    memset(dVeercArrays, 0, sizeof(double) * LINEARAXIS_NUM_MAX);
    memset(dVeercVaildArrays, 0, sizeof(double) * LINEARAXIS_NUM_MAX);
    memset(cValidAxis, 0, sizeof(int8) * LINEARAXIS_NUM_MAX);

    for (int i = 0; i < LINEARAXIS_NUM_MAX; i++)
    {
        if (AXIS_NULL == ucMapArray[g_ucLookAheadAxisArray[i]] ||
            ucMapArray[g_ucLookAheadAxisArray[i]] >= AXIS_PHYSICS_NUM_MAX)
            continue;
        cArraySize++;
        dVeerDeltaVMaxArrays[i] = g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[i]]].dVeerDeltaVMax;
        cValidAxis[i] = TRUE;
        dAxisAccArrays[i] = getSingleAxisAcc(
            g_ucLookAheadAxisArray[i], g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[i]]].ucAccVelPlanType,
            ucMapArray);
    }

    double dTc = g_SSystemPara.dInterpolationTime;

    double dCoefOfFeedrate;

    //在进给速度与其他速度对比之前要对G99进行判断
    //因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
    //因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
    //计算dFeedrate变换到mm/ms系数
    dCoefOfFeedrate = getBits(pPreTraj->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
                          ? 1.0
                          : calcCoefOfFeedrate(pPreTraj, ucChannelNum);
    for (int i = 0; i < LINEARAXIS_NUM_MAX; i++)
    {
        if (!cValidAxis[i])
            continue;
        if (dES_Vector_MultiArray[i] < 0)
        {
            dVeercArrays[i] = fabs(dVeerDeltaVMaxArrays[i] / dES_Vector_SubArray[i]);
        }
        else
        {
            dVeercArrays[i] = isZero(dES_Vector_SubArray[i]) == TRUE
                                  ? (pPreTraj->dFeedrate * dCoefOfFeedrate)
                                  : fabs(dAxisAccArrays[i] * dTc / dES_Vector_SubArray[i]);
        }
        dVeercVaildArrays[cArrayIndex++] = dVeercArrays[i];
    }
    double dVeerMin = 0;

    if (cArraySize > 1) //如果数组有效值大于1，找到数组的最小值
    {
        for (int i = 1; i < cArraySize; i++) //数量很小，直接用插入排序
        {
            double dTemp = dVeercVaildArrays[i];
            int j = i;
            for (; j > 0 && dVeercVaildArrays[j - 1] > dTemp; j--)
                dVeercVaildArrays[j] = dVeercVaildArrays[j - 1];
            dVeercVaildArrays[j] = dTemp;
        }
    }

    dVeerMin = (cArraySize >= 1) ? dVeercVaildArrays[0] : 0;

    pPreTraj->dVeer = dVeerMin;

    return TRUE;
}

///*************************************************
//  函数名称：calcVeerWhenAxisRev(struct SToolCompensation* pPreTraj, double* dES_Vector_MultiArray, double* dES_Vector_SubArray, const uint8* ucMapArray)
//  函数功能  单轴反向时VEER计算
//  输入参数：pPreTraj:  衔接点前段轨迹指针
//		   dES_Vector_MultiArray ： 向量分量相乘数组指针
//		   dES_Vector_SubArray：    向量分量相减数组指针
//		   const uint8* ucMapArray ：     当前通道轴映射数组指针
//		    ucChannelNum        通道号
//  输出参数：
//  返 回 值：TRUE ：传入指针不为空
//			FALSE:传入指针为空，不计算
//  备    注：多通道情况下，暂时前瞻三个轴
//**************************************************/
//bool calcVeerWhenAxisRev(struct SToolCompensation* pPreTraj, uint8 ucChannelNum, double* dES_Vector_MultiArray, double* dES_Vector_SubArray, const uint8* ucMapArray)
//{
//	if (pPreTraj == NULL || dES_Vector_MultiArray == NULL || dES_Vector_SubArray == NULL || ucMapArray == NULL)
//	{
//		return FALSE;
//	}
//
//	//根据各轴反向情况分情况计算角度约束速度
//	//从系统参数结构体中获取需要的参数    单轴转向允许变化的最大速度 mm/s
//	//AXIS_X  AXIS_Z 在common.h中宏定义
//
//	double dN1VeerDeltaVMax = g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[0]]].dVeerDeltaVMax;
//	double dN2VeerDeltaVMax = g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[1]]].dVeerDeltaVMax;
//	double dN3VeerDeltaVMax = g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[2]]].dVeerDeltaVMax;
//
//	double dTc = g_SSystemPara.dInterpolationTime;
//
//	double dN1AxisAcc = getSingleAxisAcc(g_ucLookAheadAxisArray[0], g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[0]]].ucAccVelPlanType, ucMapArray);
//	double dN2AxisAcc = getSingleAxisAcc(g_ucLookAheadAxisArray[1], g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[1]]].ucAccVelPlanType, ucMapArray);;
//	double dN3AxisAcc = getSingleAxisAcc(g_ucLookAheadAxisArray[2], g_SSystemPara.SAxisPara[ucMapArray[g_ucLookAheadAxisArray[2]]].ucAccVelPlanType, ucMapArray);;
//
//	double dN1Veer;//1轴约束速度
//	double dN2Veer;//2轴约束速度
//	double dN3Veer;//3轴约束速度
//
//	double dCoefOfFeedrate;
//
//	//在进给速度与其他速度对比之前要对G99进行判断
//	//因为G99模式下进给速度并非以mm/ms为单位，而是以mm/r为单位，
//	//因此要知道主轴转速大小,通Spindle.c中函数getG97AbsSpindleSpeed()可得到理论主轴转速
//	//计算dFeedrate变换到mm/ms系数
//	dCoefOfFeedrate = calcCoefOfFeedrate(pPreTraj, ucChannelNum);
//
//	//根据各轴反向情况分类计算(三个轴的反向一共7中情况)
//	if (dES_Vector_MultiArray[VECTOR_N1] < 0)
//	{
//		dN1Veer = fabs(dN1VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N1]);
//		if (dES_Vector_MultiArray[VECTOR_N2] < 0)
//		{
//			dN2Veer = fabs(dN2VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N2]);
//			if (dES_Vector_MultiArray[VECTOR_N3] < 0)//1：X反 Y反 Z反
//			{
//				dN3Veer = fabs(dN3VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N3]);
//			}
//			else//2：X反 Y反 Z同
//			{
//				dN3Veer = isZero(dES_Vector_SubArray[VECTOR_N3]) == TRUE ? (pPreTraj->dFeedrate * dCoefOfFeedrate) : fabs(dN3AxisAcc * dTc / dES_Vector_SubArray[VECTOR_N3]);
//			}
//		}
//		else
//		{
//			//二号轴
//			dN2Veer = isZero(dES_Vector_SubArray[VECTOR_N2]) == TRUE ? (pPreTraj->dFeedrate * dCoefOfFeedrate) : fabs(dN2AxisAcc * dTc / dES_Vector_SubArray[VECTOR_N2]);
//
//			if (dES_Vector_MultiArray[VECTOR_N3] < 0)//3：X反 Y同 Z反
//			{
//				dN3Veer = fabs(dN3VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N3]);
//			}
//			else//4：X反 Y同 Z同
//			{
//				dN3Veer = isZero(dES_Vector_SubArray[VECTOR_N3]) == TRUE ? (pPreTraj->dFeedrate * dCoefOfFeedrate) : fabs(dN3AxisAcc * dTc / dES_Vector_SubArray[VECTOR_N3]);
//			}
//		}
//	}
//	else//X轴不反向
//	{
//		//1号轴约束
//		dN1Veer = isZero(dES_Vector_SubArray[VECTOR_N1]) == TRUE ? (pPreTraj->dFeedrate * dCoefOfFeedrate) : fabs(dN1AxisAcc * dTc / dES_Vector_SubArray[VECTOR_N1]);
//
//		if (dES_Vector_MultiArray[VECTOR_N2] < 0)
//		{
//			dN2Veer = fabs(dN2VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N2]);
//
//			if (dES_Vector_MultiArray[VECTOR_N3] < 0)//5: X同 Y反 Z反
//			{
//				dN3Veer = fabs(dN3VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N3]);
//			}
//			else//6: X同 Y反 Z同
//			{
//				dN3Veer = isZero(dES_Vector_SubArray[VECTOR_N3]) == TRUE ? (pPreTraj->dFeedrate * dCoefOfFeedrate) : fabs(dN3AxisAcc * dTc / dES_Vector_SubArray[VECTOR_N3]);
//			}
//		}
//		else//此时只有一种可能： 7: X同 Y同 Z反
//		{
//			dN2Veer = isZero(dES_Vector_SubArray[VECTOR_N2]) == TRUE ? (pPreTraj->dFeedrate * dCoefOfFeedrate) : fabs(dN2AxisAcc * dTc / dES_Vector_SubArray[VECTOR_N2]);
//			dN3Veer = fabs(dN3VeerDeltaVMax / dES_Vector_SubArray[VECTOR_N3]);
//		}
//	}
//
//	pPreTraj->dVeer = min(dN1Veer, dN2Veer);
//	pPreTraj->dVeer = min(pPreTraj->dVeer, dN3Veer);
//
//	return TRUE;
//}
