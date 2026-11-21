#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Gcode.h"
#include "Config.h"

#include "ringbuff.h"
#include "ErrorFunction.h"
#include "GeneralFunctions.h"
#include "channel.h"

#include "common.h"
#include "GCodeCommandAPI.h"
#include "RoundGeometry.h"
#include "CornerTransition.h"
#if STM32_MODE
#include "flread_task.h"

#endif

/*************************************************
  函数名称：void initGetTextStateInfo(SGetTextStateInfo* pSGetTextStateInfo)
  函数功能：初始化文本状态结构体
  输入参数：pSGetTextStateInfo	获取文本状态结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void initGetTextStateInfo(SGetTextStateInfo* pSGetTextStateInfo)
{
    if (pSGetTextStateInfo == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }
    memset(pSGetTextStateInfo, 0, sizeof(SGetTextStateInfo));
}

/*************************************************
  函数名称：void initGCodeSystemState(SGCodeSysState* pSGCodeSysState)
  函数功能：G代码系统状态初始化
  输入参数：pSGCodeSysState		G代码系统状态结构体
  输出参数：
  返 回 值：
  备    注：是否需要从SRun中继承???
			G代码解析只需初始化一次
**************************************************/
void initGCodeSystemState(SGCodeSysState* pSGCodeSysState, uint8 ucChannelNum)
{
    if (pSGCodeSysState == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    //1.扫描结构体置0
    memset(pSGCodeSysState, 0, sizeof(SGCodeSysState));

    //2.1初始化模态结构体
    initGCodeModalInfo(&pSGCodeSysState->SModalInfo);

    //3.从SRun中读取数据
#if STM32_MODE
    getInfoFromSRun(pSGCodeSysState, ucChannelNum, g_RunMode);

#else
    //实际应从运行结构体中读
    pSGCodeSysState->ucToolID = 1;
    pSGCodeSysState->ucToolCompNum = 1;

    pSGCodeSysState->sWorkpieceCoordNum = G54 * pow(10, GCOMMAND_DECIMAL_PLACE);
#endif

    /* 继承主轴已生效的限速，避免解析状态清零后下发零限速；度/ms转换为r/min。 */
    pSGCodeSysState->dLimitedSpindleSpeed =
        g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->dLimitedSpindleSpeed * (60.0 * 1000) / DEGREE_OF_CIRCLE;
}

/*************************************************
  函数名称：struct SGCodeParse *getGCodeHeadAddress()
  函数功能  释放G代码缓冲区单元内开辟的内存
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void freeGCodeBufUnitData(struct SGCodeParse* pSGCodeParseUnit)
{
    if (pSGCodeParseUnit == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }
    if (pSGCodeParseUnit->pSG50Offset != NULL)
    {
        freeGCode(pSGCodeParseUnit->pSG50Offset);
    }
}

/*************************************************
  函数名称：struct SGCodeParse *getGCodeHeadAddress()
  函数功能  初始化G代码缓冲区单元
  输入参数：
  输出参数：
  返 回 值：
  备    注：首次写缓冲区单元前，需要调用此函数进行初始化
**************************************************/
void initGCodeBufUnit(struct SGCodeParse* pSGCodeParseUnit)
{
    if (pSGCodeParseUnit == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    //释放缓冲区单元内开辟的内存
    freeGCodeBufUnitData(pSGCodeParseUnit);

    //1.清空缓冲区单元
    memset(pSGCodeParseUnit, 0, sizeof(struct SGCodeParse));

    //2.对初始值非0的变量初始化
    pSGCodeParseUnit->EntityType = DOT;

    //2.2通道指令信息置为无效
    pSGCodeParseUnit->sChannelCommand.ucType = UNASSIGNED_VALUE;

    //注：缓冲区的起点终点在继承缓冲区状态中赋值为当前点坐标
}

/*************************************************
  函数名称：void initScanModalInfo(SScanModalInfo* pSScanModalInfo)
  函数功能：模态信息结构体初始化
  输入参数：pSGCodeModalInfo		模态信息结构体地址
  输出参数：
  返 回 值：
  备    注：G代码解析只初始化一次？
**************************************************/
void initGCodeModalInfo(SGCodeModalInfo* pSGCodeModalInfo)
{
    if (pSGCodeModalInfo == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }
    //1.模态结构体置0
    memset(pSGCodeModalInfo, 0, sizeof(SGCodeModalInfo));

    //2.初始化
    pSGCodeModalInfo->sGa = G00 * pow(10, GCOMMAND_DECIMAL_PLACE); //Ga组初始值：G00
    pSGCodeModalInfo->sGb = G54 * pow(10, GCOMMAND_DECIMAL_PLACE); //Gb组初始值：G54
    pSGCodeModalInfo->sGc = G17 * pow(10, GCOMMAND_DECIMAL_PLACE); //Gc组初始值：加工中心 XY 平面 G17
    pSGCodeModalInfo->sGd = G40 * pow(10, GCOMMAND_DECIMAL_PLACE); //Gd组初始值：G40
    pSGCodeModalInfo->sGe = G21 * pow(10, GCOMMAND_DECIMAL_PLACE); //Ge组初始值：G21
    pSGCodeModalInfo->sGf = G97 * pow(10, GCOMMAND_DECIMAL_PLACE); //Gf组初始值：G97
    pSGCodeModalInfo->sGg = G94 * pow(10, GCOMMAND_DECIMAL_PLACE); //Gg组初始值：G94
    pSGCodeModalInfo->sGh = G90 * pow(10, GCOMMAND_DECIMAL_PLACE); //Gh组初始值：G90

    pSGCodeModalInfo->pSGaRegulation =
        (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, G00 * pow(10, GCOMMAND_DECIMAL_PLACE));
}

/*************************************************
  函数名称：void getInfoFromSRun(SGCodeSysState* pSGCodeSysState, uint8 ucChannelNum, enum RUNMODE runMode)
  函数功能：从SRun中获取信息
  输入参数：
  输出参数：
  返 回 值：

  备    注：STM32模式使用，需要继承当前SRun中的状态
**************************************************/
void getInfoFromSRun(SGCodeSysState* pSGCodeSysState, uint8 ucChannelNum, enum RUNMODE runMode)
{
    struct _SRun* pSRun = &g_SChannel[ucChannelNum].SRun;

    //1.读取模态信息
    {
        int32 nRet = 0;
        //SRun中获取sGa
        nRet = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR);
        switch (nRet)
        {
        case CMD_MOTION_QUICKMOVE:
            pSGCodeSysState->SModalInfo.sGa = G00 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_MOTION_LINE:
            pSGCodeSysState->SModalInfo.sGa = G01 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_MOTION_ARC:
            pSGCodeSysState->SModalInfo.sGa = G02 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_MOTION_ANTI_ARC:
            pSGCodeSysState->SModalInfo.sGa = G03 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_MOTION_SCREWCUT:
            pSGCodeSysState->SModalInfo.sGa = G32 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_MOTION_SCREWCUT_LOOP:
            pSGCodeSysState->SModalInfo.sGa = G92 * pow(10, GCOMMAND_DECIMAL_PLACE); //SRUN无G92，后续要修正这问题
            break;
        case CMD_MOTION_G33:
            pSGCodeSysState->SModalInfo.sGa = G33 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_MOTION_SCREWCUT_VP:
            pSGCodeSysState->SModalInfo.sGa = G34 * pow(10, GCOMMAND_DECIMAL_PLACE); //SRUN无G34，后续要修正这问题
            break;
        default:
            pSGCodeSysState->SModalInfo.sGa = G00 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        }
        pSGCodeSysState->SModalInfo.pSGaRegulation =
            (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, pSGCodeSysState->SModalInfo.sGa);
        //获取sGb
        pSGCodeSysState->SModalInfo.sGb =
            g_SChannel[ucChannelNum].SRun.cWorkpieceCoordNum * pow(10, GCOMMAND_DECIMAL_PLACE);

        //SRun中获取sGc
        nRet = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_PLANE_ADDR);
        switch (nRet)
        {
        case CMD_PLANE_G17:
            pSGCodeSysState->SModalInfo.sGc = G17 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_PLANE_G18:
            pSGCodeSysState->SModalInfo.sGc = G18 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_PLANE_G19:
            pSGCodeSysState->SModalInfo.sGc = G19 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        default:
            pSGCodeSysState->SModalInfo.sGc = G17 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        }

        //SRun中获取sGd
        pSGCodeSysState->SModalInfo.sGd = UNASSIGNED_VALUE; //MDI不允许刀补

        //SRun中获取sGf
        nRet = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_SPINDLE_SPEED_ADDR);
        switch (nRet)
        {
        case CMD_SPINDLE_SPEED_CV:
            pSGCodeSysState->SModalInfo.sGf = G97 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_SPINDLE_SPEED_CLV:
            pSGCodeSysState->SModalInfo.sGf = G96 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        default:
            pSGCodeSysState->SModalInfo.sGf = G97 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        }

        //SRun中获取sGg
        nRet = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_FEED_ADDR);
        switch (nRet)
        {
        case CMD_FEED_PER_MIN:
            pSGCodeSysState->SModalInfo.sGg = G94 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_FEED_PER_R:
            pSGCodeSysState->SModalInfo.sGg = G95 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        default:
            pSGCodeSysState->SModalInfo.sGg = G94 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        }
        //SRun中获取sGh
        nRet = getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_G90G91_ADDR);
        switch (nRet)
        {
        case CMD_G90G91_G90:
            pSGCodeSysState->SModalInfo.sGh = G90 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        case CMD_G90G91_G91:
            pSGCodeSysState->SModalInfo.sGh = G91 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        default:
            pSGCodeSysState->SModalInfo.sGh = G90 * pow(10, GCOMMAND_DECIMAL_PLACE);
            break;
        }
    }

    // zxr: 坐标计算与继承的 G91 模态保持一致。
    pSGCodeSysState->bRelativeCoordinateProgramming =
        (pSGCodeSysState->SModalInfo.sGh == G91 * pow(10, GCOMMAND_DECIMAL_PLACE));

    //2.读取偏置信息
    {
        //2.1读取所有刀具磨耗
        for (int i = 0; i < TOOL_NUM_MAX; i++)
        {
            pSGCodeSysState->SToolWearPara[i] = g_SSystemPara.SWearPara[ucChannelNum][i].SToolCoord;
        }

        //2.2读取刀号坐标号
        pSGCodeSysState->ucToolID = pSRun->ucToolNum;
        pSGCodeSysState->ucToolCompNum = pSRun->ucToolCompNum;
        pSGCodeSysState->sWorkpieceCoordNum = pSRun->cWorkpieceCoordNum;

        //2.3读取当前偏置值
        // zxr: 读取本通道外部偏移，供程序目标计算使用。
        pSGCodeSysState->SAllOffsetOfCoord.SToolSettingOffset = g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset;
        pSGCodeSysState->SAllOffsetOfCoord.SG50Offset = pSRun->SWorkPieceOffsetByG50;
        pSGCodeSysState->SAllOffsetOfCoord.SToolWearOffset =
            g_SSystemPara.SWearPara[ucChannelNum][pSRun->ucToolCompNum - 1].SToolCoord;
        pSGCodeSysState->SAllOffsetOfCoord.SToolLengthOffset =
            g_SSystemPara.SToolPara[ucChannelNum][pSRun->ucToolCompNum - 1].SToolCoord;
        pSGCodeSysState->SAllOffsetOfCoord.SWorkPieceOffset =
            g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffset[pSRun->cWorkpieceCoordNum - 54];
        calculateTotalOffset(pSGCodeSysState);

        //pSGCodeSysState->SAllOffsetOfCoord.SG50Offset = g_SZeroPtOfCoordSys->SWorkPieceOffsetByG50;
        //uint8 ucToolNum = 0;
        //if (g_SChannel[ucChannelNum].SRun.ucToolNum > 0 && g_SChannel[ucChannelNum].SRun.ucToolNum <= AXIS_NUM_MAX)
        //	ucToolNum = g_SChannel[ucChannelNum].SRun.ucToolNum;

        //pSGCodeSysState->ucToolID = g_SSystemPara.SToolPara[ucChannelNum][ucToolNum - 1].ucToolID;
    }
    //3.读取所有偏置值

    //主轴转速赋值
    double dSpinleSpeed = UNASSIGNED_VALUE;
    if (pSGCodeSysState->SModalInfo.sGf == G97 * pow(10, GCOMMAND_DECIMAL_PLACE)) //赋值主轴转速
    {
        dSpinleSpeed = g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->dSpindleSpeed * (60.0 * 1000) /
                       DEGREE_OF_CIRCLE; ////主轴转速 度/ms->r/min
    }
    else
    {
        dSpinleSpeed = g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->dFeedrate *
                       (60.0 * 1000); ////主轴转速  主轴转速 mm/ms->mm/min
    }
    pSGCodeSysState->dSpindleSpeed = dSpinleSpeed;

    //获取当前坐标
    getRealTimePoint((double*)&pSGCodeSysState->SCurrPoint, AXIS_NUM_MAX,
                     (const uint8*)&g_SSystemParaUnion.SSystemParaInfo.ucAxisMapArray[ucChannelNum],
                     NULL); //获取实时坐标

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    if (g_SChannel[ucChannelNum].bUseVirtualY == TRUE) //开启合成虚拟Y轴功能：将倾斜坐标系的实时点换算至直角坐标系
    {
        pSGCodeSysState->SCurrPoint.dArray[AXIS_X] -=
            1.0 * pSGCodeSysState->SCurrPoint.dArray[AXIS_Y] * g_SChannel[ucChannelNum].dSINAngleYY1;
        pSGCodeSysState->SCurrPoint.dArray[AXIS_Y] =
            pSGCodeSysState->SCurrPoint.dArray[AXIS_Y] * g_SChannel[ucChannelNum].dCOSAngleYY1;
    }
#endif
}

/*************************************************
  函数名称：void initScanModalInfo(SScanModalInfo* pSScanModalInfo)
  函数功能：扫描模态结构体初始化
  输入参数：pSScanModalInfo		扫描模态结构体地址
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void initScanModalInfo(SScanModalInfo* pSScanModalInfo)
{
    if (pSScanModalInfo == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }
    //1.模态结构体置0
    memset(pSScanModalInfo, 0, sizeof(SScanModalInfo));
}

/*************************************************
  函数名称：void initNoModalInfo(struct SGCodeNonModalInfo* pSGCodeNoModalInfo)
  函数功能：非模态结构体初始化
  输入参数：pSGCodeNoModalInfo		非模态结构体地址
  输出参数：
  返 回 值：
  备    注：结构体内无动态内存
**************************************************/
void initScanNonModalInfo(SScanNonModalInfo* pSScanNonModalInfo)
{
    if (pSScanNonModalInfo == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    //1.非模态结构体置0
    memset(pSScanNonModalInfo, 0, sizeof(SScanNonModalInfo));
}

/*************************************************
  函数名称：void initScanContainer(SScan* pSScanContainer,bool bClearAll)
  函数功能：初始化扫描结构体
  输入参数：
  输出参数：
  返 回 值：

  备    注：初始化结构体变量的步骤如下
			1.释放结构体中开辟的内存
			2.结构体内全部置0
			3.对结构体变量初始化
			4.对初始值非0的变量赋值
**************************************************/
void initScanContainer(SScan* pSScanContainer, bool bClearAll)
{
    //1.暂时保存模态非模态及FST指令
    //SScanModalInfo STempScanModalInfo;
    //SScanNonModalInfo STempScanNonModalInfo;

    //STempScanModalInfo = pSScanContainer->SScanModalInfo;
    //STempScanNonModalInfo = pSScanContainer->SScanNonModalInfo;

    bool bF = pSScanContainer->bF;
    double dF = pSScanContainer->dF;

    bool bS = pSScanContainer->bS;
    double dS = pSScanContainer->dS;

    bool bT = pSScanContainer->bT;
    double dT = pSScanContainer->dT;

    bool bH = pSScanContainer->bH; //G71等2行参数检查
    double dH = pSScanContainer->dH;
    //2.SScan置0
    memset(pSScanContainer, 0, sizeof(SScan));

    //3.1初始化SScanModalInfo
    initScanModalInfo(&pSScanContainer->SScanModalInfo);

    //3.2初始化SScanNonModalInfo
    initScanNonModalInfo(&pSScanContainer->SScanNonModalInfo);

    //4.恢复模态非模态与FST指令
    if (!bClearAll)
    {
        //pSScanContainer->SScanModalInfo = STempScanModalInfo;
        //pSScanContainer->SScanNonModalInfo = STempScanNonModalInfo;  //zxd:不继承模态 两行指令都会写两次Gxxx

        pSScanContainer->bF = bF;
        pSScanContainer->dF = dF;

        pSScanContainer->bS = bS;
        pSScanContainer->dS = dS;

        pSScanContainer->bT = bT;
        pSScanContainer->dT = dT;

        pSScanContainer->bH = bH;
        pSScanContainer->dH = dH;
    }
}

/*************************************************
  函数名称：SGCodeSysState* getCurrSystemState(uint8 ucChannelNum)
  函数功能：获取G代码系统状态结构体
  输入参数：ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
SGCodeSysState* getCurrSystemState(uint8 ucChannelNum, uint8 ucMod)
{
    SChannelConfiguration* pSGCodeChannelConfig = getChannelConfig(ucChannelNum);
    if (ucMod == GC_SYNTAX)
    {
        return &pSGCodeChannelConfig->SSyntaxConfig.SGCodeCurrSysState;
    }
    else if (ucMod == GC_ASSIGN)
    {
        return &pSGCodeChannelConfig->SAssignConfig.SGCodeCurrSysState;
    }
}

/*************************************************
  函数名称：void inheritFromGCodeSystemState(struct _SGCodeSysState* pSGCodeSysState, struct SGCodeParse* pSGCodeParseUnit)
  函数功能：将G代码缓冲区单元中数据更新到当前通道G代码解析状态
  输入参数：
  输出参数：
  返 回 值：

  备    注：每写完一个缓冲区单元，必须要调用此函数对G代码系统状态进行更新
			由于多通道运动间的相互影响，
			此函数对G代码系统状态更新有滞后，后续将取消该函数，直接对系统状态进行更新
**************************************************/
void updataGCodeSystemState(struct _SGCodeSysState* pSGCodeSysState, struct SGCodeParse* pSGCodeParseUnit)
{

    //更新模态信息
    pSGCodeSysState->SModalInfo = pSGCodeParseUnit->SModalInfo;

    //1.更新当前点
    pSGCodeSysState->SCurrPoint =
        pSGCodeParseUnit->SEndPoint; //注意：若实体类型为点，在运动时，起点终点坐标都需要更新 ?
    //特殊情况：对于G33，G代码缓冲区虽然只给起点和终点，但G33底层会走两段轨迹：起点-终点、终点-起点，底层运动根本不按G代码缓冲区去走，所以在此处特殊处理
    if (pSGCodeParseUnit->EntityType == TAPPING)
    {
        pSGCodeSysState->SCurrPoint = pSGCodeParseUnit->SStartPoint;
    }

    //特殊情况：对于G32，退尾终点坐标变化
    if (pSGCodeParseUnit->EntityType == THREAD)
    {
        pSGCodeSysState->SCurrPoint.dArray[AXIS_X] += pSGCodeParseUnit->dJ;
    }

    ////2.更新当前刀具号、刀补号
    //pSGCodeSysState->ucToolID = pSGCodeParseUnit->ucToolID;
    //pSGCodeSysState->ucToolCompNum = pSGCodeParseUnit->ucToolCompNum;

    //工件坐标号
    //pSGCodeSysState->sWorkpieceCoordNum = pSGCodeParseUnit->sWorkpieceCoordNum;

    ////G50偏置值
    //if (pSGCodeParseUnit->pSG50Offset != NULL)
    //{
    //	pSGCodeSysState->SAllOffsetOfCoord.SG50Offset = *(pSGCodeParseUnit->pSG50Offset);
    //}

    ////更新总偏置（总偏置 = 刀具偏置 + 工件坐标系偏置 + G50偏置）
    //TChangeTotalOffset(0, pSGCodeParseUnit->ucToolCompNum, pSGCodeParseUnit->SModalInfo.sGb, &pSGCodeSysState->SAllOffsetOfCoord.STotalOffset, &pSGCodeSysState->SAllOffsetOfCoord.SG50Offset);
    //

    //3.更新当前主轴速度、第二转轴速度
    pSGCodeSysState->dSpindleSpeed = pSGCodeParseUnit->dSpindleSpeed;
    pSGCodeSysState->dSecondSpindleSpeed = pSGCodeParseUnit->dSecondSpindleSpeed;

    //4.继承当前主轴转速限制
    pSGCodeSysState->dLimitedSpindleSpeed = pSGCodeParseUnit->dLimitedSpindleSpeed;

    //5.继承当前进给速度
    pSGCodeSysState->dFeedrate = pSGCodeParseUnit->dFeedrate;

    //6.继承插补模式
    pSGCodeSysState->cInterpolationState = pSGCodeParseUnit->cInterpolationState;
}

void inheritFromGCodeSystemState(struct _SGCodeSysState* pSGCodeSysState, struct SGCodeParse* pSGCodeParseUnit)
{
    //继承模态信息
    pSGCodeParseUnit->SModalInfo = pSGCodeSysState->SModalInfo;
    pSGCodeParseUnit->SModalInfo.sGz = 0;               //Gz组指令非模态，需要取消
    pSGCodeParseUnit->SModalInfo.pSGzRegulation = NULL; //Gz组指令非模态，需要取消
    //1.继承当前点
    pSGCodeParseUnit->SStartPoint =
        pSGCodeSysState->SCurrPoint; //注意：若实体类型为点，在运动时，起点终点坐标都需要更新 ?
    pSGCodeParseUnit->SEndPoint =
        pSGCodeSysState->SCurrPoint; //注意：若实体类型为点，在运动时，起点终点坐标都需要更新 ?

    //2.继承当前刀具号、刀补号
    pSGCodeParseUnit->ucToolID = pSGCodeSysState->ucToolID;
    pSGCodeParseUnit->ucToolCompNum = pSGCodeSysState->ucToolCompNum;

    //3.继承当前主轴速度、第二转轴速度
    pSGCodeParseUnit->dSpindleSpeed = pSGCodeSysState->dSpindleSpeed;
    pSGCodeParseUnit->dSecondSpindleSpeed = pSGCodeSysState->dSecondSpindleSpeed;

    //4.继承当前主轴转速限制
    pSGCodeParseUnit->dLimitedSpindleSpeed = pSGCodeSysState->dLimitedSpindleSpeed;

    //5.继承当前进给速度
    pSGCodeParseUnit->dFeedrate = pSGCodeSysState->dFeedrate;

    //6.继承插补模式
    pSGCodeParseUnit->cInterpolationState = pSGCodeSysState->cInterpolationState;
    pSGCodeParseUnit->fCylinderRadius = pSGCodeSysState->fCylinderRadius;

    //7.继承断屑指令
    pSGCodeParseUnit->bChipBreakFlag = pSGCodeSysState->bChipBreakFlag;
    pSGCodeParseUnit->dChipBreakLength = pSGCodeSysState->dChipBreakLength;
    pSGCodeParseUnit->unChipBreakDelayTime = pSGCodeSysState->unChipBreakDelayTime;
}

/*************************************************
  函数名称：int8 processProgramText(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram)
  函数功能：赋值时处理程序文本（函数内分步执行）
  输入参数：
  输出参数：
  返 回 值：0		程序段未处理完成
			1		已得到普通程序段（替换宏名+去除空格）
			2		已处理%
			3		已记录define
			-1		出错
  备    注：！！！此函数需要调整：输入参数pSAssignConfig改为pSGetTextStateInfo
			1.
			2.获取的程序段为原始程序段
**************************************************/
int8 processProgramText(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram)
{
    if (pSGetTextStateInfo == NULL || pSCurrProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }

    switch (pSGetTextStateInfo->ucStep)
    {
    //0.获取一段程序段字符
    case 0:
    {
        uint8 ucRet;
        //1.获取原始程序段
        ucRet = getProgramSegment(pSGetTextStateInfo, ucChannelNum, pSCurrProgram);
        //1.1获取到非空程序段
        if (ucRet == 2)
        {
            pSGetTextStateInfo->ucStep = 1;
        }
        //1.2读取文本需要等待 || 空行空程序段
        else if (ucRet == 0xF1 || ucRet == 1)
        {
            pSGetTextStateInfo->ucStep = 0;
        }
        //1.3读取文本超出范围
        else if (ucRet == 0xF2)
        {
            pSGetTextStateInfo->ucStep = 0;
            //补程序文本结束标志%
            processSecondPercentSign(ucMod, ucChannelNum);
            return 2;
        }
        //1.4打开文本失败
        else if (ucRet == 0xF3)
        {
            parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM89);
        }
        else if (ucRet == 0xF4)
        {
            parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM89);
        }
        //成功从文本中读取一段原始程序段：包含%，记录define，替换define，普通程序段
        return 0;
    }

    //1.判断%，判断define
    case 1:
    {
        int8* pTemp;
        pTemp = pSGetTextStateInfo->cStr;
        //1.1获取程序段中第一个非空格的字符
        while (*pTemp == ' ' || *pTemp == '\t')
        {
            pTemp++;
        }
        //1.2是否为%
        if (*pTemp == '%')
        {
            pSGetTextStateInfo->ucStep = 2;
        }
        //1.3是否为define
        else if (*pTemp == 'D' && *(pTemp + 1) == 'E' && *(pTemp + 2) == 'F' && *(pTemp + 3) == 'I' &&
                 *(pTemp + 4) == 'N' && *(pTemp + 5) == 'E')
        {
            pSGetTextStateInfo->ucStep = 3;
        }
        //1.4非%非define（程序正文），进入宏名替换
        else
        {
            pSGetTextStateInfo->ucStep = 4;
        }

        return 0;
    }

    //2.处理%
    case 2:
    {
        //起始行号已赋值：处理第二个%（第一个%跳过不处理）
        if (pSCurrProgram->sBeginLineNum != UNASSIGNED_VALUE)
        {
            SChannelConfiguration* pChannelConfig = getChannelConfig(ucChannelNum);
            if (ucMod == GC_SYNTAX)
            {
                SSyntaxConfiguration* pSSyntaxConfig = &pChannelConfig->SSyntaxConfig;
                processSecondPercentSign(ucMod, ucChannelNum);
            }
            else if (ucMod == GC_ASSIGN)
            {
                SAssignConfiguration* pSAssignConfig = &pChannelConfig->SAssignConfig;
                processSecondPercentSign(ucMod, ucChannelNum);
            }
        }
        pSGetTextStateInfo->ucStep = 0;
        return 2;
    }

    //3.记录define	//赋值流程不需要记录DEFINE
    case 3:
    {
        if (pSCurrProgram->sBeginLineNum != UNASSIGNED_VALUE)
        {
            //程序正文中不允许出现自定义宏（必须写在开头报错）
            parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, 0);
            return -1;
        }
        if (recordDefine(pSGetTextStateInfo, pSCurrProgram->SMacro.pDefineList) < 0)
        {

            return -1;
        }
        pSGetTextStateInfo->ucStep = 0;
        return 3;
    }

    //4.替换define
    case 4:
    {
#if !STM32_MODE
        if (pSCurrProgram->sBeginLineNum == UNASSIGNED_VALUE)
        {
            pSCurrProgram->sBeginLineNum = pSGetTextStateInfo->unCurrLineNum;
            SAssignConfiguration* pSAssignConfig = getAssignConfig(ucChannelNum);
            pSAssignConfig->SGetTextStateInfo.unGetLineNum = pSCurrProgram->sBeginLineNum;
        }
#else
        // STM32 下 sBeginLineNum 应该由 runButton 启动前预扫描设置
        if (pSCurrProgram->sBeginLineNum == UNASSIGNED_VALUE)
        {
            parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM300);
            return -1;
        }
#endif

        pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->cStr;
        uint8 ucRet;
        ucRet = replaceDefineInSegment(pSGetTextStateInfo, pSCurrProgram->SMacro.pDefineList);
        //自定义宏名替换完成
        if (ucRet == 1)
        {
            //去空格
            eliminateBlank(pSGetTextStateInfo->cStr, MAX_LENGTH_OF_LINE); //消除空白
            pSGetTextStateInfo->ucStep = 0;

            //去空格后再次判断是否为空程序段
            if (pSGetTextStateInfo->cStr[0] == '\0')
            {
                return 0;
            }

            return 1;
        }
        else
        {
            return 0;
        }
    }
    }

    return 0;
}

/*************************************************
  函数名称：uint8 getProgramSegment(SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pCurrProgram)//获取程序段
  函数功能：获取程序段（检查赋值共用）
  输入参数：pSGetTextStateInfo		获取文本状态信息结构体
			ucChannelNum			通道号
			pSCurrProgram			当前程序结构体地址
  输出参数：
  返 回 值：0 默认值
			1 空行或空程序段
			2 获取程序段成功
			0xF1 读取本文，需要等待
			0xF2 读取本文，超出范围
			0xF3 读取本文，出错
  备    注：1.本函数内未对0xF1,0xF2,0xF3进行处理，直接返回
			2.获取的程序段为原始程序段,如："  G1X  1Z 1 F SPEED"
**************************************************/
uint8 getProgramSegment(SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram) //获取程序段
{
    bool bFlag = FALSE;    //是否读取新的一行成功
    uint8 ucReturnVal = 0; //0：默认值，1：空行

    if (NULL == pSCurrProgram || NULL == pSGetTextStateInfo)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return 0;
    }

    //需要获取新的一行原始字符串
    if (0 == pSGetTextStateInfo->ucOriStrIndex)
    {
        uint8 ucRet = 0;

        int16 cListSize = getListSize(getProgramList(ucChannelNum));

        ucRet = tryToGetLine(pSCurrProgram->cFileName, pSGetTextStateInfo->unGetLineNum, cListSize,
                             ucChannelNum); //尝试获取文本

        if (ucRet >= 0 && ucRet < 20) //能够获取文本
        {
            int8 cRet;

            getGCodeStr(pSGetTextStateInfo->cOriginalStr, ucRet, MAX_LENGTH_OF_LINE); //获取文本

#if !STM32_MODE
            initDesinatedBuf(ucRet); //清除对应文本缓冲
#endif
            bFlag = TRUE;
            pSGetTextStateInfo->ucInsertIndex = 0; //读取新一行，程序段也是新的一段，插入索引置0
        }

        else if (ucRet == 0xF1)
        {
            return 0xF1;
        }
        else if (ucRet == 0xF2) //超出范围，说明全部文本读完,跳出循环
        {
            return 0xF2;
        }
        else if (ucRet == 0xF3) //出错
        {
            return 0xF3;
        }
        else if (ucRet == 0xF4) //出错
        {
            return 0xF4;
        }
    }

    //开始从行字符串内存中读取程序段放入程序段内存
    if (bFlag || 0 != pSGetTextStateInfo->ucOriStrIndex)
    {
        memset(pSGetTextStateInfo->cStr, 0, MAX_LENGTH_OF_LINE); //初始化程序段
        (pSGetTextStateInfo->ucInsertIndex) = 0;

        for (; (pSGetTextStateInfo->ucOriStrIndex) < MAX_LENGTH_OF_LINE; pSGetTextStateInfo->ucOriStrIndex++)
        {
            //当前字符为括号内容
            if (pSGetTextStateInfo->bIgnored)
            {
                if (')' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                {
                    pSGetTextStateInfo->bIgnored = FALSE;
                    //(*pUcOriStrIndex)++;
                }
                else if ('\0' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex) ||
                         '\r' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex) ||
                         '\n' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                {
                    if (0 != pSGetTextStateInfo->ucInsertIndex)
                    {
                        ucReturnVal = 2;
                    }
                    else //整行是注释，解析下一行
                    {
                        pSGetTextStateInfo->unGetLineNum += 1;
                    }
                    pSGetTextStateInfo->ucOriStrIndex = 0;
                    break;
                }
                else
                {
                    //(*pUcOriStrIndex)++;
                }
            }

            //当前字符非括号内容
            else
            {
                //当前字符非括号内容，且程序段头为/
                if (0 == pSGetTextStateInfo->ucInsertIndex &&
                    '/' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                {
                    //双斜杠，说明是注释内容
                    if ('/' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex + 1))
                    {
                        pSGetTextStateInfo->unGetLineNum += 1; //此行是注释，解析下一行
                        pSGetTextStateInfo->ucOriStrIndex = 0;
                        ucReturnVal = 1; //注释行等于空行
                        break;
                    }
                    //单斜杠，跳段内容
                    else
                    {
                        //if (g_SChannel[ucChannelNum].bSkipMode)//跳段模式开启？
                        if (s_pSChannelConfig[ucChannelNum]->bSkipMode)
                        {
                            //跳过此程序段
                            //寻找分号或者行结束符号
                            while (!isLineFinishSign(pSGetTextStateInfo->cOriginalStr +
                                                     pSGetTextStateInfo->ucOriStrIndex) &&
                                   !isSegmentFinishSign(pSGetTextStateInfo->cOriginalStr +
                                                        pSGetTextStateInfo->ucOriStrIndex))
                            {
                                pSGetTextStateInfo->ucOriStrIndex++;
                            }
                            //运行至此，说明找到了程序段尾部
                            if (';' == *(pSGetTextStateInfo->cOriginalStr +
                                         pSGetTextStateInfo->ucOriStrIndex)) //若为分号(程序段结束符号)，往前移动一位
                            {
                                //(*pUcOriStrIndex)++;
                            }
                            else //行结束符号
                            {
                                pSGetTextStateInfo->unGetLineNum += 1; //解析下一行
                                pSGetTextStateInfo->ucOriStrIndex = 0;
                                ucReturnVal = 1; //跳过此程序段，等于空行
                                break;
                            }
                        }
                        //else//未开启跳段模式，此段代码可注释
                        //{
                        //    (*pUcOriStrIndex)++;//跳过单斜杠，接着获取程序段
                        //}
                    }
                }

                //当前字符非括号内容，且程序段头非/
                else
                {
                    if ('(' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                    {
                        pSGetTextStateInfo->bIgnored = TRUE;
                    }
                    //程序段结束标识符
                    else if (isSegmentFinishSign(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                    //else if (';' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                    {
                        pSGetTextStateInfo->ucOriStrIndex++;
                        if (0 == pSGetTextStateInfo->ucInsertIndex) //若插入索引为0，说明是空程序段
                        {
                            ucReturnVal = 1;
                            break;
                        }
                        ucReturnVal = 2; //获取程序段成功
                        pSGetTextStateInfo->unCurrLineNum = pSGetTextStateInfo->unGetLineNum;
                        break;
                    }

                    //行结束标识符
                    else if (isLineFinishSign(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                    {

#if !STM32_MODE
                        //若插入索引为0，说明是空程序段，不进入Part2
                        if (0 == pSGetTextStateInfo->ucInsertIndex)
                        {
                            if ('\0' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex))
                            {
                                ucReturnVal =
                                    0xF1; //陈敏聪	//2023/9/1暂改，出现了vs用fgets库函数从文件中读取新的一行全为空但实际文件中对应的行不为空的情况
                            }
                            else
                            {
                                pSGetTextStateInfo->unGetLineNum += 1; //读取下一行
                                ucReturnVal = 1;
                            }
                        }
                        else
                        {
                            pSGetTextStateInfo->unGetLineNum += 1; //读取下一行(lxw:统一在这处理获取行号)
                            //getProgramSegmentSubFunc(ucSyntaxOrAssign, pSyntax, pAssign, pCPart);
                            ucReturnVal = 2; //获取程序段成功
                            pSGetTextStateInfo->unCurrLineNum = pSGetTextStateInfo->unGetLineNum - 1;
                        }
#else STM32_MODE
                        if (0 == pSGetTextStateInfo->ucInsertIndex) //若插入索引为0，说明是空程序段，不进入Part2
                        {
                            pSGetTextStateInfo->unGetLineNum += 1; //读取下一行
                            ucReturnVal = 1;
                        }
                        else
                        {
                            //getProgramSegmentSubFunc(ucSyntaxOrAssign, pSyntax, pAssign, pCPart);
                            pSGetTextStateInfo->unGetLineNum += 1;
                            ucReturnVal = 2; //获取程序段成功
                            pSGetTextStateInfo->unCurrLineNum = pSGetTextStateInfo->unGetLineNum - 1;
                        }
#endif
                        pSGetTextStateInfo->ucOriStrIndex = 0;
                        break;
                    }
                    //
                    else if (33 <= *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex) &&
                                 *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex) <=
                                     126 //33和126根据ASCII码表而来
                             || (' ' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex) ||
                                 '\t' == *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex)))
                    {
                        *(pSGetTextStateInfo->cStr + pSGetTextStateInfo->ucInsertIndex) =
                            *(pSGetTextStateInfo->cOriginalStr + pSGetTextStateInfo->ucOriStrIndex);
                        pSGetTextStateInfo->ucInsertIndex++;
                    }
                    else
                    {
                        parseGCodeErrorB(pSGetTextStateInfo->unGetLineNum, GCODE_ERROR_NUM84); // 程序输入的代码值非法
                    }
                }
            }
        }
    }

    return ucReturnVal;
}

/*************************************************
  函数名称：static void getGCodeStr(char *pString, uint8 ucIdx, uint16 usStrSize)
  函数功能  获取字符串
  输入参数：pString字符串，ucIdx全局变量g_textBuf的索引  测试用
  输出参数：
  返 回 值：

  备    注：
**************************************************/
void getGCodeStr(int8* pString, uint8 ucIdx, uint16 usStrSize)
{
    uint16 usTemp;

    memset(pString, 0, usStrSize);

#if STM32_MODE
    for (usTemp = 0; mapbuff[ucIdx][usTemp] != 0 && usTemp < usStrSize; usTemp++)
    {
        pString[usTemp] = mapbuff[ucIdx][usTemp];
    }
#else
    for (usTemp = 0; g_textBuf[ucIdx][usTemp] != '\0' && usTemp < usStrSize; usTemp++) //陈敏聪 9/1修改 以结束符'\0'结束
    {
        pString[usTemp] = g_textBuf[ucIdx][usTemp];
    }
#endif

    pString[usStrSize - 1] = 0; //加截止符，防止出错
}

/*************************************************
  函数名称：int8 processSecondPercentSign(uint8 ucMod, uint8 ucChannelNum)
  函数功能：处理程序文本中第二个%
  输入参数：ucMod			模式
			ucChannelNum	通道号
  输出参数：
  返 回 值：1		处理完成
			-1		错误
  备    注：1.第一个%不需处理
			2.只有主程序才可能解析到第二个%（子程序解析到第二个百分号报错，因为必须要返回主程序）
**************************************************/
int8 processSecondPercentSign(uint8 ucMod, uint8 ucChannelNum)
{
    SChannelConfiguration* pSGCodeChannel = getChannelConfig(ucChannelNum);
    struct _SSyntaxConfiguration* pSSyntaxConfig = &pSGCodeChannel->SSyntaxConfig;
    struct _SAssignConfiguration* pSAssignConfig = &pSGCodeChannel->SAssignConfig;

    //1.检查
    if (ucMod == GC_SYNTAX)
    {
        //1.子程序不应该获取到第二个%
        if (!isMainProgram(ucChannelNum))
        {
            parseGCodeErrorB(0, 0);
            return -1;
        }
        //2.不是在Part1中获取到第二个%
        if (pSGCodeChannel->SSyntaxConfig.cPart != SYNTAX_PART_1)
        {
            //parseGCodeErrorB(0, 0);
            return -1;
        }

        pSGCodeChannel->SSyntaxConfig.bCheckFinished = TRUE;
        changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN2FINISHED, TRUE, 0);
        pSSyntaxConfig->SGetTextStateInfo.nEndLineNum = pSSyntaxConfig->SGetTextStateInfo.unGetLineNum;
    }
    //2.赋值
    else
    {
        if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) <= 0)
        {
            return -1; //缓冲区满需要等待
        }
        pSGCodeChannel->SAssignConfig.bAssignFinished = TRUE;

        struct SGCodeParse* pSGCodeBuffUnit = getSGCodeParsePointerByType(GCODE_WRITE, ucChannelNum);

        //2.1缓冲区当前G代码缓冲区写单元
        initGCodeBufUnit(pSGCodeBuffUnit);

        //2.2继承上一缓冲区部分数据（如：刀具号，刀补号等）
        inheritFromGCodeSystemState(&pSAssignConfig->SGCodeCurrSysState, pSGCodeBuffUnit);

        //写文件名
        SProgram* pSCurrProgram = getCurrProgramStruct(ucChannelNum);
        strncpy(pSGCodeBuffUnit->cFilename, pSCurrProgram->cFileName, FILENAME_LENGTH_LIMIT);
        pSGCodeBuffUnit->cFilename[FILENAME_LENGTH_LIMIT - 1] = 0;
        //写缓冲区中G代码行号信息
        pSGCodeBuffUnit->unGCodeLineNum = pSAssignConfig->SGetTextStateInfo.unGetLineNum;

        //2.3写指令类型
        setBits(pSGCodeBuffUnit->unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);

        //2.4设置赋值目标状态
        changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN2FINISHED, TRUE, 0);

        pSAssignConfig->SGetTextStateInfo.nEndLineNum = pSAssignConfig->SGetTextStateInfo.unGetLineNum;

        //2.5标记bLastLineMark
        pSGCodeBuffUnit->bLastLineMark = TRUE;

        moveGCodeWritePtr(ucChannelNum);
    }
}

static bool isEndWhileTailValid(const int8* pAfterEndWhile)
{
    if (pAfterEndWhile == NULL)
        return FALSE;

    // 允许空白
    while (*pAfterEndWhile == ' ' || *pAfterEndWhile == '\t')
    {
        pAfterEndWhile++;
    }

    // 允许：段结束/行结束/注释
    if (isSegmentFinishSign((const char*)pAfterEndWhile) || isLineFinishSign((const char*)pAfterEndWhile))
    {
        return TRUE;
    }

    return FALSE;
}

/*************************************************
  函数名称：bool scanSegment(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo,
			SScan* pSScanContainer, SProgram* pCurrProgram, uint8 ucChannelNum)
  函数功能：扫描程序段
  输入参数：
  输出参数：
  返 回 值：TRUE	程序段扫描完成
			FALSE	程序段扫描未完成
  备    注：
**************************************************/
bool scanSegment(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, SProgram* pCurrProgram,
                 uint8 ucChannelNum)
{
    uint8 ucWordLen = 0;       //词法单元长度
    uint8 ucExpressionLen = 0; //表达式长度
    uint8 cLoopCnt = 0;        //循环次数限制
    int8 cRet = 0;

    if (pSGetTextStateInfo == NULL || pSScanContainer == NULL || pCurrProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return FALSE;
    }

    if (ucChannelNum > CHANNEL_NUM_MAX - 1)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM301);
        return FALSE;
    }

    //执行一次循环，获取一个指令字
    //如:X10+20F1000
    //pStartPtr指向X，pEndPtr指向F，pCurrPtr用于移动
    while (cLoopCnt < SCAN_LOOP_LIMIT_MACRO)
    {
        //当前指向不是换行符、回车符、结束符，说明后面还有字符，继续扫描
        //if (isSegmentFinishSign(*pSGetTextStateInfo->pCurrPtr != '\n') && (*pSGetTextStateInfo->pCurrPtr != '\0') && (*pSGetTextStateInfo->pCurrPtr != '\r') && (*pSGetTextStateInfo->pCurrPtr != ';'))
        if (pSScanContainer->cScanFinishFlag == TRUE)
        {
            pSScanContainer->unLineNum = pSGetTextStateInfo->unCurrLineNum;
            return TRUE;
        }
        //已出现错误，停止扫描
        if (isErrorHasBeenWritten())
        {
            return FALSE;
        }

        //1.扫描第0步：找到指令字符，pStartPtr指向指令字首字符，pCurrPtr最后指向表达式首字符
        if (pSScanContainer->ucScanStep == 0)
        {
            pSGetTextStateInfo->pStartPtr = pSGetTextStateInfo->pCurrPtr; //先记录每一个词法单元搜索的起点
            if (isTagetStr_B("GOTO", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到GOTO的后一位
                pSGetTextStateInfo->pCurrPtr += 4;
                pSScanContainer->bGOTO = TRUE;
                pSScanContainer->cMacroType = 1;
            }
            else if (isTagetStr_B("IF", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到IF的后一位
                pSGetTextStateInfo->pCurrPtr += 2;
                pSScanContainer->bIF = TRUE;
                pSScanContainer->cMacroType = 2;
            }
            else if (isTagetStr_B("ELSEIF", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到ELSEIF的后一位
                pSGetTextStateInfo->pCurrPtr += 6;
                pSScanContainer->bELSEIF = TRUE;
                pSScanContainer->cMacroType = 3;
            }
            else if (isTagetStr_B("ELSE", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到ELSE的后一位
                pSGetTextStateInfo->pCurrPtr += 4;
                pSScanContainer->bELSE = TRUE;
                pSScanContainer->cMacroType = 4;
            }
            else if (isTagetStr_B("ENDIF", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到ENDIF的后一位
                pSGetTextStateInfo->pCurrPtr += 5;
                pSScanContainer->bENDIF = TRUE;
                pSScanContainer->cMacroType = 5;
            }
            else if (isTagetStr_B("WHILE", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到WHILE的后一位
                pSGetTextStateInfo->pCurrPtr += 5;
                pSScanContainer->bWHILE = TRUE;
                pSScanContainer->cMacroType = 6;
            }
            else if (isTagetStr_B("ENDWHILE", pSGetTextStateInfo->pStartPtr))
            {
                //pCurrPtr移到ENDWHILE的后一位
                pSGetTextStateInfo->pCurrPtr += 8;
                // 2) 立刻校验后继字符：只允许段结束或注释/行结束
                if (!isEndWhileTailValid(pSGetTextStateInfo->pCurrPtr))
                {
                    setErrorType(GCODE_ERROR_NUM134); // 示例：表达式/宏语法错误（按你项目习惯调整）
                    findError(pSGetTextStateInfo->pCurrPtr, pSGetTextStateInfo->unCurrLineNum);
                    return FALSE; // **立刻终止当前段扫描**
                }

                pSScanContainer->bENDWHILE = TRUE;
                pSScanContainer->cMacroType = 7;
            }
            else if (isTagetStr_B("#", pSGetTextStateInfo->pStartPtr))
            {
                //#时，必须要有赋值符号'=',不然没有任何意义，不做解引用和运算
                cRet = isAssignable(pSGetTextStateInfo->pStartPtr);
                if (cRet == TRUE)
                {
                    pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pStartPtr; //#就是表达式首字符
                }
                else
                {
                    //可以不报错，但要移动指针到合适的位置，后面再处理
                    parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM131);
                }
            }
            else if ((*pSGetTextStateInfo->pCurrPtr >= 'A' && *pSGetTextStateInfo->pCurrPtr <= 'Z') ||
                     (*pSGetTextStateInfo->pCurrPtr >= 'a' && *pSGetTextStateInfo->pCurrPtr <= 'z'))
            {
                pSGetTextStateInfo->pCurrPtr += 1;
                if (*pSGetTextStateInfo->pStartPtr == 'm' || *pSGetTextStateInfo->pStartPtr == 'M')
                {
                    pSScanContainer->SScanNonModalInfo.cMLetterCnt++;
                    if (pSScanContainer->SScanNonModalInfo.cMLetterCnt > 6) //错误类型：M功能出现次数不能大于6
                    {
                        setErrorFlag(TRUE);
                        setErrorType(GCODE_ERROR_NUM399);
                        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                    }
                }
            }
            else if (isSegmentFinishSign(pSGetTextStateInfo->pCurrPtr) ||
                     isLineFinishSign(pSGetTextStateInfo->pCurrPtr))
            {
                pSScanContainer->cScanFinishFlag = TRUE;
                continue;
            }
            else
            {
                setErrorType(GCODE_ERROR_NUM84); // 错误类型：非字母开头的字符串
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            }

            //pSScanContainer->ucScanStep = 1;

            /* 针对 ENDIF 的修复 */
            if (pSScanContainer->cMacroType == 5) /* IF 块结束词 ENDIF */
            {
                int8* pTail = pSGetTextStateInfo->pCurrPtr;

                /* ENDIF 不允许携带操作数，后面必须是合法的段结束或行结束标志。 */
                if (!isSegmentFinishSign((const char*)pTail) && !isLineFinishSign((const char*)pTail))
                {
                    setErrorType(GCODE_ERROR_NUM134);
                    findError(pTail, pSGetTextStateInfo->unCurrLineNum);
                    pSScanContainer->ucScanStep = 0;
                    return FALSE;
                }

                /* ENDIF 没有表达式，将两个指针都设置到已确认合法的结束边界。 */
                pSGetTextStateInfo->pCurrPtr = pTail;
                pSGetTextStateInfo->pEndPtr = pTail;

                /* 跳过表达式解析，进入原有的宏控制词专用处理步骤。 */
                pSScanContainer->ucScanStep = 4;
            }
            else
            {
                /* 其他指令保持原有流程，进入表达式扫描步骤。 */
                pSScanContainer->ucScanStep = 1;
            }
        }

        //2.扫描第1步：找到完整表达式，pEndPtr指向表达式后一位，pCurrPtr最后指向表达式首字符
        else if (pSScanContainer->ucScanStep == 1)
        {
            int8* pTemp;
            pTemp = pSGetTextStateInfo->pCurrPtr; //记录表达式首字符
            cRet =
                getWholeExpression(&pSGetTextStateInfo->pCurrPtr, pSGetTextStateInfo->unCurrLineNum); //截取整个宏单元
            if (cRet > 0)
            {
                //已获取完整表达式，pEndPtr指向表达式后一位
                pSGetTextStateInfo->pEndPtr = pSGetTextStateInfo->pCurrPtr;
                //将pCurrPtr重新移动到表达式开头
                pSGetTextStateInfo->pCurrPtr = pTemp;

                //表达式长度
                ucExpressionLen = (uint8)(pSGetTextStateInfo->pEndPtr - pSGetTextStateInfo->pCurrPtr);
                if (ucExpressionLen == 0)
                {
                    setErrorType(GCODE_ERROR_NUM84);
                    findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                }
                if (isNeedCalculate(pSGetTextStateInfo->pCurrPtr, ucExpressionLen) == 1)
                {
                    pSScanContainer->ucScanStep = 2;
                    //初始宏计算结构体
                    SMacroCalculate* pSMacroCal =
                        ucMod == GC_SYNTAX ? &g_SMacroCalCheck[ucChannelNum] : &g_SMacroCalAssign[ucChannelNum];
                    initCalculate(pSMacroCal); //lxw:全部初始化 or 仅将计算步骤置为0
                }
                else
                {
                    pSScanContainer->ucScanStep = 3;
                }
            }
            else
            {
                setErrorType(GCODE_ERROR_NUM84);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            }
        }

        //3.扫描第2步：表达式需要计算，计算表达式（表达式头pCurrPtr，表达式尾pEndPtr）
        else if (pSScanContainer->ucScanStep == 2)
        {
            //表达式长度
            ucExpressionLen = (uint8)(pSGetTextStateInfo->pEndPtr - pSGetTextStateInfo->pCurrPtr);
            uint16 ucPoundVal = 0; //记录#i中i的值

            //检查和赋值独立内存区域
            SMacroCalculate* pSMacroCal =
                ucMod == GC_SYNTAX ? &g_SMacroCalCheck[ucChannelNum] : &g_SMacroCalAssign[ucChannelNum];

            //计算表达式结果，dLexicalUnitValue
            cRet = calculateExpression(pSGetTextStateInfo->pCurrPtr, ucExpressionLen, pSMacroCal,
                                       &pCurrProgram->SMacro.SLocalVarAssign, &pSScanContainer->dLexicalUnitValue,
                                       &ucPoundVal, pSGetTextStateInfo->unCurrLineNum);
            //3.1计算完成，结果不需解引用    or解引用不为NULL
            if (cRet == 1)
            {
                pSScanContainer->ucScanStep = 4;
            }
            //3.2计算完成，结果需要解引用,且解引用不为NULL
            else if (cRet == 2)
            {
                //对于PQR需要记录原始#i的i值  (A类宏G65  G65 Hm P#i Q#j R#k)
                if (*pSGetTextStateInfo->pStartPtr == 'P' || *pSGetTextStateInfo->pStartPtr == 'p')
                {
                    pSScanContainer->usPoundP = ucPoundVal;
                    pSScanContainer->bPoundP = TRUE;
                }
                else if (*pSGetTextStateInfo->pStartPtr == 'Q' || *pSGetTextStateInfo->pStartPtr == 'q')
                {
                    pSScanContainer->usPoundQ = ucPoundVal;
                    pSScanContainer->bPoundQ = TRUE;
                }
                else if (*pSGetTextStateInfo->pStartPtr == 'R' || *pSGetTextStateInfo->pStartPtr == 'r')
                {
                    pSScanContainer->usPoundR = ucPoundVal;
                    pSScanContainer->bPoundR = TRUE;
                }
                pSScanContainer->ucScanStep = 4;
            }
            //3.3计算完成，结果需要解引用,且解引用为NULL
            else if (cRet == 3)
            {
                //对于PQR需要记录原始#i的i值  (A类宏G65  G65 Hm P#i Q#j R#k)
                if (*pSGetTextStateInfo->pStartPtr == 'P' || *pSGetTextStateInfo->pStartPtr == 'p')
                {
                    pSScanContainer->usPoundP = ucPoundVal;
                    pSScanContainer->bPoundP = TRUE;
                }
                else if (*pSGetTextStateInfo->pStartPtr == 'Q' || *pSGetTextStateInfo->pStartPtr == 'q')
                {
                    pSScanContainer->usPoundQ = ucPoundVal;
                    pSScanContainer->bPoundQ = TRUE;
                }
                else if (*pSGetTextStateInfo->pStartPtr == 'R' || *pSGetTextStateInfo->pStartPtr == 'r')
                {
                    pSScanContainer->usPoundR = ucPoundVal;
                    pSScanContainer->bPoundR = TRUE;
                }
                //放弃对当前指令字的记录，进入下一个指令字的扫描（G1 X1 Z#0）
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;
                cLoopCnt++;
                pSScanContainer->ucScanStep = 0;
                continue;
            }
            else if (cRet < 0)
            {
                setErrorType(GCODE_ERROR_NUM84);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            }
        }

        //4.扫描第3步：表达式不需计算，获取指令值
        else if (pSScanContainer->ucScanStep == 3)
        {
            //pSScanContainer->dLexicalUnitValue = atof(pSGetTextStateInfo->pCurrPtr);//用此函数会出错，如G0X100会输出G256

            //将表达式转为数值
            int8 cPos = 0;
            int8 cRet = 0;
            double dLexicalUnitValue;

            cRet =
                getNumbers(pSGetTextStateInfo->pCurrPtr, &cPos, &dLexicalUnitValue, pSGetTextStateInfo->unCurrLineNum);
            if (cRet == 0)
            {
                /*
				 精度检查，是否超出支持位数
                （仅检查普通 G 指令的字符。格式和长度已由 getNumbers 检查；避免额外小数位在 float 转换后丢失。）
                 */
                if (pSScanContainer->cMacroType == 0 &&
                    (*pSGetTextStateInfo->pStartPtr == 'G' || *pSGetTextStateInfo->pStartPtr == 'g'))
                {
                    const int8* p;
                    bool bAfterPoint = FALSE;
                    uint8 ucDecimalDigits = 0;

                    for (p = pSGetTextStateInfo->pCurrPtr; p < pSGetTextStateInfo->pEndPtr; ++p)
                    {
                        if (*p == '.')
                        {
                            bAfterPoint = TRUE;
                        }
                        else if (bAfterPoint && *p >= '0' && *p <= '9')
                        {
                            ++ucDecimalDigits;

                            /* 超出支持位数后只允许尾随零。 */
                            if (ucDecimalDigits > GCOMMAND_DECIMAL_PLACE && *p != '0')
                            {
                                setErrorType(GCODE_ERROR_NUM87);
                                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                                return FALSE;
                            }
                        }
                    }
                }

                pSScanContainer->dLexicalUnitValue = (double)dLexicalUnitValue;
                pSScanContainer->ucScanStep = 4;
            }
        }

        //5.扫描第4步：已得到 指令字符+指令值。记录到 模态or扫描结构体
        else if (pSScanContainer->ucScanStep == 4)
        {
            switch (pSScanContainer->cMacroType)
            {
            //普通指令字
            case 0:
            {
                recordLexicalUnit(ucMod, pSGetTextStateInfo, pSScanContainer, pSScanContainer->dLexicalUnitValue);
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;
                break;
            }
            //GOTO
            case 1:
                pSScanContainer->dGOTO = pSScanContainer->dLexicalUnitValue;

                if (!isSegmentFinishSign(pSGetTextStateInfo->pEndPtr))
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;
                break;
            //IF
            case 2:
                pSScanContainer->dIF = pSScanContainer->dLexicalUnitValue;
                if (isTagetStr_B("THEN", pSGetTextStateInfo->pEndPtr))
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr + 4; //pCurrPtr移动到THEN之后

                break;
            //ELSEIF
            case 3:
                pSScanContainer->dELSEIF = pSScanContainer->dLexicalUnitValue;

                if (isTagetStr_B("THEN", pSGetTextStateInfo->pEndPtr))
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr + 4; //pCurrPtr移动到THEN之后

                break;
            //ELSE
            case 4:
                if (pSGetTextStateInfo->pCurrPtr != pSGetTextStateInfo->pEndPtr)
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;

                break;
            //ENDIF
            case 5:
                if (pSGetTextStateInfo->pCurrPtr != pSGetTextStateInfo->pEndPtr)
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;

                break;
            //WHILE
            case 6:
                pSScanContainer->dWHILE = pSScanContainer->dLexicalUnitValue;
                if (isTagetStr_B("DO", pSGetTextStateInfo->pEndPtr))
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr + 2; //pCurrPtr移动到THEN之后

                break;
            //ENDWHILE
            case 7:
                if (pSGetTextStateInfo->pCurrPtr != pSGetTextStateInfo->pEndPtr)
                {
                    //报错
                }
                pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->pEndPtr;

                break;
            default:
                break;
            }
            pSScanContainer->ucScanStep = 0;
        }

        //当前指向程序段结束标志，程序段扫描完成
        cLoopCnt++;
    }
    return FALSE;
}

/*************************************************
  函数名称：void recordLexicalUnit(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo,
			SScan* pSScanContainer, double dLexicalUnitValue)
  函数功能：将指令字记录到 扫描结构体 中
  输入参数：ucMod					模式（检查or赋值）
			pSGetTextStateInfo		获取文本结构体
			pSScanContainer			扫描结构体
			dLexicalUnitValue		指令字值
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void recordLexicalUnit(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer,
                       double dLexicalUnitValue)
{
    if (pSGetTextStateInfo == NULL || pSScanContainer == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    /* 车床专用语法和待实现的机床输入输出指令在此明确报错。
     * 当前进给采用 G94，刀具与主轴指令需完成加工中心实现后再开放。 */
    switch (*pSGetTextStateInfo->pStartPtr)
    {
    case 'A':
    case 'a':
    case 'B':
    case 'b':
    case 'C':
    case 'c':
    case 'U':
    case 'u':
    case 'V':
    case 'v':
    case 'W':
    case 'w':
    case 'S':
    case 's':
    case 'T':
    case 't':
    case 'H':
    case 'h':
    case 'D':
    case 'd':
    case 'L':
    case 'l':
    case 'Q':
    case 'q':
        setErrorType(GCODE_ERROR_NUM87);
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
    default:
        break;
    }
    if (!isfinite(dLexicalUnitValue))
    {
        setErrorType(GCODE_ERROR_NUM130);
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
    }

    switch (*pSGetTextStateInfo->pStartPtr)
    {
    case 'A':
    case 'a':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bABC[0] == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dABC[0] = dLexicalUnitValue;
        pSScanContainer->bABC[0] = TRUE;
        break;
    }
    case 'B':
    case 'b':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bABC[1] == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dABC[1] = dLexicalUnitValue;
        pSScanContainer->bABC[1] = TRUE;
        break;
    }
    case 'C':
    case 'c':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bABC[2] == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dABC[2] = dLexicalUnitValue;
        pSScanContainer->bABC[2] = TRUE;
        break;
    }

    case 'X':
    case 'x':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bXYZ[0] == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dXYZ[0] = dLexicalUnitValue;
        pSScanContainer->bXYZ[0] = TRUE;
        break;
    }
    case 'Y':
    case 'y':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bXYZ[1] == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dXYZ[1] = dLexicalUnitValue; //(绝对坐标)
        pSScanContainer->bXYZ[1] = TRUE;
        break;
    }
    case 'Z':
    case 'z':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bXYZ[2] == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dXYZ[2] = dLexicalUnitValue;
        pSScanContainer->bXYZ[2] = TRUE;
        break;
    }

    case 'U':
    case 'u':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bUVW[0] == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dUVW[0] = dLexicalUnitValue; //先保留 看看有没有出现X,判断有没有超系统最大范围
        pSScanContainer->bUVW[0] = TRUE;
        break;
    }
    case 'V':
    case 'v':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bUVW[1] == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dUVW[1] = dLexicalUnitValue;
        pSScanContainer->bUVW[1] = TRUE;
        break;
    }
    case 'W':
    case 'w':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bUVW[2] == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dUVW[2] = dLexicalUnitValue;
        pSScanContainer->bUVW[2] = TRUE;
        break;
    }

    case 'R':
    case 'r':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bRIJK[0] == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402); //报错
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dRIJK[0] = dLexicalUnitValue; //记录R的值
        pSScanContainer->bRIJK[0] = TRUE;              //记录R已出现 a
        break;
    }
    case 'I':
    case 'i':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bRIJK[1] == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dRIJK[1] = dLexicalUnitValue;
        pSScanContainer->bRIJK[1] = TRUE;
        break;
    }
    case 'J':
    case 'j':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bRIJK[2] == TRUE) //G67时，不允许多个IJK
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        }
        pSScanContainer->dRIJK[2] = dLexicalUnitValue;
        pSScanContainer->bRIJK[2] = TRUE;
        break;
    }
    case 'K':
    case 'k':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bRIJK[3] == TRUE) //G67时，不允许多个IJK
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dRIJK[3] = dLexicalUnitValue;
        pSScanContainer->bRIJK[3] = TRUE;
        break;
    }

    case 'F':
    case 'f': //printf("F:%f\n", dLexicalUnitValue);
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bF == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dF = dLexicalUnitValue;
        pSScanContainer->bF = TRUE;
        break;
    }
    case 'S':
    case 's': //printf("S:%f\n", dLexicalUnitValue);            //二义性，S指主轴或动力头的速度		//陈敏聪 2023/3/6改
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bS == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dS = dLexicalUnitValue; //陈敏聪 2023/3/6改
        pSScanContainer->bS = TRUE;
        break;
    }
    case 'T':
    case 't': //printf("T:%f\n", dLexicalUnitValue);			  //无二义性
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bT == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dT = dLexicalUnitValue;
        pSScanContainer->bT = TRUE;
        break;
    }

    case 'P':
    case 'p':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bP == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        //P,H先后判断
        if (pSScanContainer->bH == FALSE)
        {
            pSScanContainer->bPFirst = TRUE;
            //非模态/模态 宏程序调用停止后续扫描（G65/G66程序段会出现多个IJK指令字）
            if (pSScanContainer->SScanModalInfo.pSGzRegulation != NULL)
            {
                if (pSScanContainer->SScanModalInfo.pSGzRegulation->nCommandVal ==
                        G65 * pow(10, GCOMMAND_DECIMAL_PLACE) ||
                    pSScanContainer->SScanModalInfo.pSGzRegulation->nCommandVal ==
                        G66 * pow(10, GCOMMAND_DECIMAL_PLACE))
                {
                    pSScanContainer->cScanFinishFlag = TRUE;
                }
            }
        }
        pSScanContainer->dP = dLexicalUnitValue;
        pSScanContainer->bP = TRUE;

        break;
    }
    case 'H':
    case 'h':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bH == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        //P,H先后判断
        if (pSScanContainer->bP == FALSE)
        {
            pSScanContainer->bHFirst = TRUE;
        }

        pSScanContainer->dH = dLexicalUnitValue;
        pSScanContainer->bH = TRUE;
        break;
    }

    case 'N':
    case 'n':
    {
        if (ucMod == GC_SYNTAX && ((dLexicalUnitValue < 0) ||
                                   (!isZero(dLexicalUnitValue - (int32)dLexicalUnitValue)))) //段号不能出现负值或小数
        {
            setErrorType(GCODE_ERROR_NUM82);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        if (ucMod == GC_SYNTAX &&
            (*pSGetTextStateInfo->pStartPtr != *pSGetTextStateInfo->cStr)) //判断字母N是否为此程序段的第一个字母
        {
            setErrorType(GCODE_ERROR_NUM80);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum); //报错
        }
        pSScanContainer->dN = dLexicalUnitValue;
        pSScanContainer->bN = TRUE;
        break;
    }
    case 'Q':
    case 'q':
    {
        if (ucMod == GC_SYNTAX &&
            pSScanContainer->bQ ==
                TRUE) // && pSSyntaxConfig->SCurrRecord.SNonModalInfo.sGz != G65)      //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dQ = dLexicalUnitValue;
        pSScanContainer->bQ = TRUE;
        break;
    }
    case 'L':
    case 'l': //printf("L:%f\n", dLexicalUnitValue);
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bL == TRUE)
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dL = dLexicalUnitValue;
        pSScanContainer->bL = TRUE;
        break;
    }
    case 'D':
    case 'd': //G10
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bD == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dD = dLexicalUnitValue;
        pSScanContainer->bD = TRUE;
        break;
    }
    case 'O':
    case 'o':
    {
        if (ucMod == GC_SYNTAX && pSScanContainer->bO == TRUE) //防止一行中出现两次
        {
            setErrorType(GCODE_ERROR_NUM402);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->dO = dLexicalUnitValue;
        pSScanContainer->bO = TRUE;
        break;
    }

    case 'G':
    case 'g':
        recordGLetter(ucMod, pSGetTextStateInfo, pSScanContainer, dLexicalUnitValue);
        break;
    case 'M':
    case 'm':
        recordMLetter(ucMod, pSGetTextStateInfo, pSScanContainer, dLexicalUnitValue);
        break;

    case '#':
        break;
    default:
        setErrorType(GCODE_ERROR_NUM87);
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        break;
    }

    //大小写参数使用同一检查索引，保留原始程序文本,允许A-Z。
    int8 cLetter = *pSGetTextStateInfo->pStartPtr;
    if (cLetter >= 'a' && cLetter <= 'z')
    {
        cLetter = (int8)(cLetter - 'a' + 'A');
    }

    if (cLetter >= 'A' && cLetter <= 'Z')
    {
        pSScanContainer->bA2Z[cLetter - 'A'] = TRUE;
        pSScanContainer->dA2Z[cLetter - 'A'] = dLexicalUnitValue;
    }
}

/*************************************************
  函数名称：
  函数功能：记录G指令字
  输入参数：pSSyntaxConfig 语法检查结构体,dLexicalUnitValue词法单元的值，cChannelNum 通道号
  输出参数：
  返 回 值：
  备    注：与其他指令字不同，GM指令字需要分组记录
**************************************************/
void recordGLetter(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, double dLexicalUnitValue)
{
    if (pSGetTextStateInfo == NULL || pSScanContainer == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    SGCommandRegulation* pSGCodeRegulation = NULL;
    double dScaleVal = dLexicalUnitValue * pow(10, GCOMMAND_DECIMAL_PLACE);
    //将G指令值从浮点型放大到整形，后续按整形进行处理。如G4.1放大为G41。
    //int32 nCommandVal = (int32)(dLexicalUnitValue * pow(10, GCOMMAND_DECIMAL_PLACE));
    int32 nCommandVal = round(dScaleVal);

    if (!isZero(dScaleVal - nCommandVal) &&
        dLexicalUnitValue != (double)(float)(nCommandVal / pow(10, GCOMMAND_DECIMAL_PLACE)))
    {
        setErrorType(GCODE_ERROR_NUM87); //不存在此功能
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
    }

    //在G代码规则树中寻找该指令字
    pSGCodeRegulation = (SGCommandRegulation*)findTreeNodeData(g_pSGCommandRegulationTree->Root, nCommandVal);
    if (pSGCodeRegulation == NULL)
    {
        setErrorType(GCODE_ERROR_NUM87); //不存在此功能
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
    }

    //
    switch (pSGCodeRegulation->cGroup)
    {
    case sGa: //sGa组  运动模态G指令
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGaRegulation != NULL ||
                                   pSScanContainer->SScanModalInfo.pSGzRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGaRegulation = pSGCodeRegulation;
        break;
    }
    case sGz: //sGz组   非模态  G指令
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGaRegulation != NULL ||
                                   pSScanContainer->SScanModalInfo.pSGzRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGzRegulation = pSGCodeRegulation;
        break;
    }
    case sGb: //sGb组   模态G指令  工件坐标系 G54-G59
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGbRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGbRegulation = pSGCodeRegulation;
        break;
    }
    case sGc: //sGc组  插补平面  G17—G19
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGcRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGcRegulation = pSGCodeRegulation;
        break;
    }
    case sGd: //sGd   刀具补偿 G40-G42
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGdRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGdRegulation = pSGCodeRegulation;
        break;
    }
    case sGe: //  单位选择：英制/公制 G20，G21
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGeRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGeRegulation = pSGCodeRegulation;
        break;
    }
    case sGf: //恒线速控制 G96、恒转速控制 G97		主轴转速控制
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGfRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGfRegulation = pSGCodeRegulation;
        break;
    }
    case sGg: //定每分进给速度 G94 、以毫米 / 转为单位给定切削进给速度 G95	、断削指令G4.9	FEED     运动轴进给控制
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGgRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGgRegulation = pSGCodeRegulation;
        break;
    }
    case sGj: //定每分进给速度 G94 、以毫米 / 转为单位给定切削进给速度 G95	、断削指令G4.9	FEED     运动轴进给控制
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGjRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGjRegulation = pSGCodeRegulation;
        break;
    }

    case sGh: //绝对编程G90  编程G91  默认G90
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanModalInfo.pSGhRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanModalInfo.pSGhRegulation = pSGCodeRegulation;
        break;
    }
    case sGx:
    {
        if (ucMod == GC_SYNTAX && (pSScanContainer->SScanNonModalInfo.pSGxRegulation != NULL))
        {
            setErrorType(GCODE_ERROR_NUM127);
            findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
            break;
        }
        pSScanContainer->SScanNonModalInfo.pSGxRegulation = pSGCodeRegulation;
        break;
    }

    default:
        setErrorType(GCODE_ERROR_NUM87); //不存在此功能
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
        break;
    }
}

/*************************************************
 函数名称：
 函数功能：记录M指令字
 输入参数：pSSyntaxConfig 语法检查结构体,dLexicalUnitValue词法单元的值
 输出参数：
 返 回 值：

 备    注：与其他指令字不同，GM指令字需要分组记录
**************************************************/
void recordMLetter(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, double dLexicalUnitValue)
{
    if (pSGetTextStateInfo == NULL || pSScanContainer == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    SMCommandRegulation* pSMCodeRegulation = NULL;

    if (!isZero(dLexicalUnitValue - (int32)dLexicalUnitValue))
    {
        setErrorType(GCODE_ERROR_NUM87); //不存在此功能
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
    }

    //将M指令值从浮点型放大到整形，后续按整形进行处理。如M4.1放大为M41。
    int32 nCommandVal = (int32)dLexicalUnitValue * pow(10, MCOMMAND_DECIMAL_PLACE);

    //特殊处理：当值为100-199时，变为100，因为这个范围为等待指令，都是执行相同的功能
    //这种处理应该不好，因为涉及到了具体指令的操作，但现在也没别的方法
    //如果后续不采用M100-M199这种等待指令的方式，这里的条件语句应该去掉
    if (nCommandVal >= CHANNEL_COMMAND_MIN_VAL && nCommandVal <= CHANNEL_COMMAND_MAX_VAL)
    {
        nCommandVal = CHANNEL_COMMAND_MIN_VAL;
    }

    //在M代码规则树中寻找该指令字
    pSMCodeRegulation = (SMCommandRegulation*)findTreeNodeData(g_pSMCommandRegulationTree->Root, nCommandVal);
    if (pSMCodeRegulation == NULL)
    {
        setErrorType(GCODE_ERROR_NUM87); //不存在此功能
        findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
        return;
    }
    //检查冲突
    if (ucMod == GC_SYNTAX)
    {
        switch (pSMCodeRegulation->cGroup)
        {
        case cMa: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMa == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMa = TRUE;
            break;
        }
        case cMb: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMb == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMb = TRUE;
            break;
        }
        case cMc: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMc == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMc = TRUE;
            break;
        }
        case cMe: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMe == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMe = TRUE;
            break;
        }
        case cMf: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMf == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMf = TRUE;
            break;
        }
        case cMg: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMg == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMg = TRUE;
            break;
        }
        case cMh: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMh == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMh = TRUE;
            break;
        }
        case cMi: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMi == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMi = TRUE;
            break;
        }
        case cMj: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMj == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMj = TRUE;
            break;
        }
        case cMk: //sGa组  运动模态G指令
        {
            if (pSScanContainer->SScanNonModalInfo.bMk == TRUE)
            {
                setErrorType(GCODE_ERROR_NUM127);
                findError(pSGetTextStateInfo->pStartPtr, pSGetTextStateInfo->unCurrLineNum);
                break;
            }
            pSScanContainer->SScanNonModalInfo.bMk = TRUE;
            break;
        }
        }
    }

    int8 cIndex = pSScanContainer->SScanNonModalInfo.cMLetterCnt - 1;
    pSScanContainer->SScanNonModalInfo.pSMCommandArray[cIndex] = pSMCodeRegulation;
}

/*************************************************
  函数名称：int8 isSegmentFinishSign(char* pChar)
  函数功能  判断输入字符是否为程序段的结束标志
  输入参数：
  输出参数：
  返 回 值：

  备    注：1.程序段结束标识符
			2.程序段结束标识符后一定还有程序段
			3.\0，\n，\r 不属于 程序段结束标志符!!!
**************************************************/
int8 isSegmentFinishSign(const char* pChar)
{
    //1.字符;
    if (';' == *pChar)
    {
        int8 i = 1;
        while (' ' == *(pChar + i) || '\t' == *(pChar + i))
        {
            i++;
        }
        //2.且字符;后一位非空格的字符不为\n \r \0
        if ('\n' != *(pChar + i) && '\r' != *(pChar + i) && '\0' != *(pChar + i))
        {
            return 1;
        }
    }
    return 0;
}

/*************************************************
  函数名称：int8 isSegmentFinishSign(char* pChar)
  函数功能  判断输入字符是否为程序段的结束标志
  输入参数：
  输出参数：
  返 回 值：

  备    注：
**************************************************/
int8 isLineFinishSign(const char* pChar)
{
    if (';' == *pChar)
    {
        int8 i = 1;
        //获取分号后一位非空格的字符
        while (' ' == *(pChar + i) || '\t' == *(pChar + i))
        {
            i++;
        }
        if ('\n' == *(pChar + i) || '\r' == *(pChar + i) || '\0' == *(pChar + i))
        {
            return 1;
        }
    }
    else if (('/' == *pChar && '/' == *(pChar + 1)) || '\0' == *pChar || '\r' == *pChar || '\n' == *pChar)
    {
        return 1;
    }
    return 0;
}

/*************************************************
  函数名称：
  函数功能: 根据刀号和磨损号改变总偏移值
  输入参数：ucToolCompNum			刀补号（根据刀补号在系统参数中找到对应偏置值）
			sGb						工件坐标号（根据工件坐标号在系统参数中找到对应偏置值）
			SWorkPieceOffsetByG50	G50偏置值

			SToatalOffset			总偏置
			ucChannelNum			通道号
  输出参数：
  返 回 值：-1 指针为空
			-2 磨损号或刀号错误
			 1 处理成功
  备    注：为了节省内存（总偏置需要用点结构体表示，点结构体过大），不通过缓冲区直接传递总偏置，而是传递刀补号，工件坐标号等。
			运动时
			总偏置 = 刀具偏置 + 工件坐标偏置 + G50偏置
			TChangeTotalOffset（运动用到）与calculateTotalOffset（G代码用到）内容相同
**************************************************/
int8 TChangeTotalOffset(uint8 ucChannelNum, uint8 ucToolCompNum, int8 sGb, SPointDouble* STotalOffset,
                        SPointDouble* SWorkPieceOffsetByG50)
{
    if (STotalOffset == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }
    if (ucToolCompNum > TOOL_NUM_MAX || ucToolCompNum < 1)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM129);
        return -2;
    }
    if (ucChannelNum > CHANNEL_NUM_MAX - 1)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM301);
        return -3;
    }

    if (sGb >= G54 && sGb <= G59)
    {
        //每个通道的都有一个独立的坐标系，但机床的刀库只有一个
        for (uint8 ucTemp = 0; ucTemp < AXIS_NUM_MAX; ++ucTemp) //YZABC轴
        {
            if (g_SSystemPara.SAxisPara[ucTemp].bShowByDiameter) //直径显示	//陈敏聪	2023/7/22
            {
                STotalOffset->dArray[ucTemp] =
                    g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset.dArray[ucTemp] / 2 +
                    g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffset[sGb - G54].dArray[ucTemp] / 2 +
                    g_SSystemPara.SToolPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    g_SSystemPara.SWearPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    SWorkPieceOffsetByG50->dArray[ucTemp];
            }
            else
            {
                STotalOffset->dArray[ucTemp] =
                    g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset.dArray[ucTemp] +
                    g_SZeroPtOfCoordSys[ucChannelNum].SWorkPieceOffset[sGb - G54].dArray[ucTemp] +
                    g_SSystemPara.SToolPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    g_SSystemPara.SWearPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    SWorkPieceOffsetByG50->dArray[ucTemp];
            }
        }
    }
    else
    {
        for (uint8 ucTemp = 0; ucTemp < AXIS_NUM_MAX; ++ucTemp) //YZABC轴
        {
            if (g_SSystemPara.SAxisPara[ucTemp].bShowByDiameter) //直径显示	//陈敏聪	2023/7/22
            {
                STotalOffset->dArray[ucTemp] =
                    g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset.dArray[ucTemp] / 2 +
                    g_SSystemPara.SToolPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    g_SSystemPara.SWearPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    SWorkPieceOffsetByG50->dArray[ucTemp];
            }
            else
            {
                STotalOffset->dArray[ucTemp] =
                    g_SZeroPtOfCoordSys[ucChannelNum].SToolSettingOffset.dArray[ucTemp] +
                    g_SSystemPara.SToolPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    g_SSystemPara.SWearPara[ucChannelNum][ucToolCompNum - 1].SToolCoord.dArray[ucTemp] +
                    SWorkPieceOffsetByG50->dArray[ucTemp];
            }
        }
    }

    return 1;
}

/*************************************************
  函数名称：static void updateChannelStatus(uint8 ucChannelNum)
  函数功能: 更新通道的状态
  输入参数：ucChannelNum 通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void updateGCodeChannelStatus(uint8 ucChannelNum)
{
    uint8 ucStatusSyntax = s_pSChannelConfig[ucChannelNum]->SSyntaxConfig.ucStatus;
    uint8 ucStatusAssign = s_pSChannelConfig[ucChannelNum]->SAssignConfig.ucStatus;

    if (ucStatusSyntax == GCODE_IDLE && ucStatusAssign == GCODE_IDLE)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_IDLE;
    }
    else if (ucStatusSyntax == GCODE_RUN || ucStatusAssign == GCODE_RUN)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_RUN;
    }
    else if (ucStatusSyntax == GCODE_WAIT && ucStatusAssign == GCODE_WAIT)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_WAIT;
    }
    else if (ucStatusSyntax == GCODE_FINISHED && ucStatusAssign == GCODE_FINISHED)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_FINISHED;
    }
    else if (ucStatusSyntax == GCODE_WARN || ucStatusAssign == GCODE_WARN)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_WARN;
    }
    else if (ucStatusSyntax == GCODE_SYNTAX_WAIT || ucStatusAssign == GCODE_ASSIGN_WAIT)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_RUN;
    }
    else if (ucStatusSyntax == GCODE_RUN2FINISHED || ucStatusAssign == GCODE_RUN2FINISHED)
    {
        s_pSChannelConfig[ucChannelNum]->ucStatus = GCODE_RUN;
    }
}

/*************************************************
  函数名称：int8 changeGCodeStatus(uint8 ucType, uint8 ucChannelNum, uint8 ucTargetStatus)
  函数功能: 改变语法检查或赋值的状态
  输入参数：ucSyntaxOrAssign	模式（检查/赋值）
			ucChannelNum		通道号
            ucTargetStatus		目标状态
			bReport				是否检查改变结果
			unLineNum			行号（用于报错）
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void changeGCodeStatus(uint8 ucSyntaxOrAssign, uint8 ucChannelNum, uint8 ucTargetStatus, bool bReport, uint32 unLineNum)
{
    int8 cChangeRet = -1; //结果   -1:改变状态失败  1:改变状态成功
    uint8 ucStatus = ucSyntaxOrAssign == 0 ? s_pSChannelConfig[ucChannelNum]->SSyntaxConfig.ucStatus
                                           : s_pSChannelConfig[ucChannelNum]->SAssignConfig.ucStatus;

    if (MDI_MODE == getRunMode())
    {
        //return;
    }

    //检查/赋值 当前状态
    switch (ucStatus)
    {
    case GCODE_IDLE:
    {
        if (ucTargetStatus == GCODE_RUN ||
            ucTargetStatus == GCODE_ASSIGN_WAIT) //陈敏聪	2023/9/1修改	空闲状态能转换成赋值等待空闲
        {
            cChangeRet = 1;
        }
        break;
    }

    case GCODE_RUN:
    {
        if (ucTargetStatus == GCODE_WARN || ucTargetStatus == GCODE_WAIT || ucTargetStatus == GCODE_RUN2FINISHED ||
            (ucTargetStatus == GCODE_SYNTAX_WAIT && ucSyntaxOrAssign == GC_SYNTAX) ||
            (ucTargetStatus == GCODE_ASSIGN_WAIT &&
             ucSyntaxOrAssign == GC_ASSIGN)) // || (ucTargetStatus == GCODE_WAIT_M96 && ucType == 1)
        {
            cChangeRet = 1;
        }

        break;
    }
    case GCODE_RUN2FINISHED:
    {
        if (ucTargetStatus == GCODE_FINISHED)
        {
            cChangeRet = 1;
        }
        break;
    }
    case GCODE_FINISHED:
    {
        if (ucTargetStatus == GCODE_IDLE)
        {
            cChangeRet = 1;
        }
        else if (ucTargetStatus == GCODE_RUN && ucSyntaxOrAssign == GC_SYNTAX)
        {
            //initBeforeCheckAgain(&s_pSChannelConfig[ucChannelNum]->SSyntaxConfig, ucChannelNum);
            cChangeRet = 1;
        }
        else if (ucTargetStatus == GCODE_RUN && ucSyntaxOrAssign == GC_ASSIGN)
        {
            //initBeforeAssignAgain(&s_pSChannelConfig[ucChannelNum]->SAssignConfig, ucChannelNum);
            cChangeRet = 1;
        }
        break;
    }

    case GCODE_WARN:
    {
        if (ucTargetStatus == GCODE_RUN)
        {
            cChangeRet = 1;
        }
        break;
    }
    case GCODE_WAIT:
    {
        if (ucTargetStatus == GCODE_RUN || (ucTargetStatus == GCODE_ASSIGN_WAIT && ucSyntaxOrAssign == GC_ASSIGN))
        {
            cChangeRet = 1;
        }
        break;
    }

    case GCODE_SYNTAX_WAIT:
    {
        if (ucTargetStatus == GCODE_RUN && ucSyntaxOrAssign == GC_SYNTAX) //重新检查，部分变量需要初始化
        {
            cChangeRet = 1;
        }
        break;
    }
    case GCODE_ASSIGN_WAIT:
    {
        if (ucTargetStatus == GCODE_RUN && ucSyntaxOrAssign == GC_ASSIGN)
        {
            cChangeRet = 1;
        }
        break;
    }

    default:
        break;
    }

    if (cChangeRet == 1) //更新状态
    {
        if (ucSyntaxOrAssign == GC_SYNTAX)
        {
            s_pSChannelConfig[ucChannelNum]->SSyntaxConfig.ucStatus = ucTargetStatus;
        }
        else
        {
            s_pSChannelConfig[ucChannelNum]->SAssignConfig.ucStatus = ucTargetStatus;
        }
        updateGCodeChannelStatus(ucChannelNum); //更新通道状态
    }
    else //改变失败
    {
        if (bReport == TRUE) //改变失败时报错
        {
            parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM306);
        }
    }
}

/*************************************************
  函数名称：uint8 getGCodeStatus(uint8 ucType, uint8 ucChannelNum)
  函数功能: 获取语法检查或赋值的状态
  输入参数：ucChannelNum 通道号
            ucType类型  CHECK:语法检查 ASSIGN:赋值 GC_CHANNEL:通道结构体
			
  输出参数：
  返 回 值：检查或赋值的状态
  备    注：
**************************************************/
uint8 getGCodeStatus(uint8 ucSyntaxOrAssign, uint8 ucChannelNum)
{
    if (ucSyntaxOrAssign == GC_SYNTAX)
    {
        return s_pSChannelConfig[ucChannelNum]->SSyntaxConfig.ucStatus;
    }
    else if (ucSyntaxOrAssign == GC_ASSIGN)
    {
        return s_pSChannelConfig[ucChannelNum]->SAssignConfig.ucStatus;
    }
    else
    {
        return s_pSChannelConfig[ucChannelNum]->ucStatus;
    }
}

/*************************************************
  函数名称：void convertGCodeBuff(struct SGCodeParse* pSGCodeBuffUnit)
  函数功能：将解析阶段的模态编码转换为下游编码，并刷新运动信息。
  输入参数：pSGCodeBuffUnit 尚未转换模态编码的缓冲单元，不能为空。
  输出参数：pSGCodeBuffUnit 原地更新模态、指令位和有效轴。
  返 回 值：无。
  备    注：每个原始记录只调用一次；已有段裁剪后只调用 refreshGCodeGeometry。
**************************************************/
void convertGCodeBuff(struct SGCodeParse* pSGCodeBuffUnit)
{
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    pSGCodeBuffUnit->bHandwheelAuto = pSGCodeBuffUnit->SModalInfo.sGa == G84 * pow(10, GCOMMAND_DECIMAL_PLACE);
#endif
    pSGCodeBuffUnit->SModalInfo.sGb /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGc /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGd /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGe /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGf /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGg /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGh /= pow(10, GCOMMAND_DECIMAL_PLACE);
    pSGCodeBuffUnit->SModalInfo.sGz /= pow(10, GCOMMAND_DECIMAL_PLACE);

    //修正Ga
    pSGCodeBuffUnit->SModalInfo.sGa /= 10;
    if (pSGCodeBuffUnit->EntityType == OTHERTYPE)
    {
        pSGCodeBuffUnit->SModalInfo.sGa = 0;
    }
    else if (pSGCodeBuffUnit->EntityType == L1NE)
    {
        pSGCodeBuffUnit->SModalInfo.sGa = 1;
    }
    else if (pSGCodeBuffUnit->EntityType == ARC && pSGCodeBuffUnit->cArcDir == 1)
    {
        pSGCodeBuffUnit->SModalInfo.sGa = 2;
    }
    else if (pSGCodeBuffUnit->EntityType == ARC && pSGCodeBuffUnit->cArcDir == -1)
    {
        pSGCodeBuffUnit->SModalInfo.sGa = 3;
    }
    else if (pSGCodeBuffUnit->EntityType == THREAD)
    {
        pSGCodeBuffUnit->SModalInfo.sGa = 32;
    }
    else if (pSGCodeBuffUnit->EntityType == TAPPING)
    {
        pSGCodeBuffUnit->SModalInfo.sGa = 33;
    }
    else
    {
        pSGCodeBuffUnit->SModalInfo.sGa = pSGCodeBuffUnit->SModalInfo.sGa;
    }

    refreshGCodeGeometry(pSGCodeBuffUnit);
}

/*************************************************
  函数名称：void refreshGCodeGeometry(struct SGCodeParse* pSGCodeBuffUnit)
  函数功能：根据当前实体及已转换模态刷新运动指令位和有效轴。
  输入参数：pSGCodeBuffUnit 已完成模态转换的记录，不能为空。
  输出参数：原地更新实体类型、unCommandType 和 ucValStatus。
  返 回 值：无。
  备    注：不再次缩放模态，不计算圆弧极值；倒圆裁剪后的极值由 updateCornerBounds 重算。
**************************************************/
void refreshGCodeGeometry(struct SGCodeParse* pSGCodeBuffUnit)
{
    //修改实体类型
    {
        //2.1修改为DOT
        if (isSamePoint(&pSGCodeBuffUnit->SStartPoint, &pSGCodeBuffUnit->SEndPoint))
        {
            pSGCodeBuffUnit->EntityType = DOT;
        }
        //2.2修改为OTHERTYPE
        //if (pSGCodeBuffUnit->EntityType != L1NE && pSGCodeBuffUnit->EntityType != ARC)
        //{
        //	pSGCodeBuffUnit->EntityType = OTHERTYPE;
        //}
    }

    //1.赋值G指令类型
    {
        switch (pSGCodeBuffUnit->EntityType)
        {
        case L1NE:
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_LINE, CMD_MOTION_ADDR);
            break;
        }
        case ARC:
        {
            if (pSGCodeBuffUnit->cArcDir == 1)
            {
                setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_ARC, CMD_MOTION_ADDR);
                break;
            }
            else
            {
                setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_ANTI_ARC, CMD_MOTION_ADDR);
                break;
            }
        }
        case THREAD:
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_SCREWCUT, CMD_MOTION_ADDR);
            break;
        }
        case TAPPING:
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_G33, CMD_MOTION_ADDR);
            break;
        }
        default:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_QUICKMOVE, CMD_MOTION_ADDR);
            break;
        }

        //G34特殊处理（shit!!!!）
        if (pSGCodeBuffUnit->SModalInfo.sGa == G34)
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_MOTION_SCREWCUT_VP, CMD_MOTION_ADDR);
        }

        //1.2sGc
        switch (pSGCodeBuffUnit->SModalInfo.sGc)
        {
        case G17:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_PLANE_G17, CMD_PLANE_ADDR);
            break;
        case G18:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_PLANE_G18, CMD_PLANE_ADDR);
            break;
        case G19:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_PLANE_G19, CMD_PLANE_ADDR);
            break;
        default:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_PLANE_G17, CMD_PLANE_ADDR);
            break;
        }
        //1.3sGd
        int8 cGd = pSGCodeBuffUnit->SModalInfo.sGd;
        switch (cGd)
        {
        case G40:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_TOOLCOMP_CANCEL, CMD_TOOLCOMP_ADDR);
            break;
        case G41:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_TOOLCOMP_LEFT, CMD_TOOLCOMP_ADDR);
            break;
        case G42:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_TOOLCOMP_RIGHT, CMD_TOOLCOMP_ADDR);
            break;
        default:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_TOOLCOMP_CANCEL, CMD_TOOLCOMP_ADDR);
            break;
        }
        //1.4sGf
        int8 cGf = pSGCodeBuffUnit->SModalInfo.sGf;
        switch (cGf)
        {
        case G96:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_SPINDLE_SPEED_CLV, CMD_SPINDLE_SPEED_ADDR);
            break;
        case G97:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);
            break;
        default: //默认G97
            setBits(pSGCodeBuffUnit->unCommandType, CMD_SPINDLE_SPEED_CV, CMD_SPINDLE_SPEED_ADDR);
            break;
        }
        //1.5sGg
        int8 cGg = pSGCodeBuffUnit->SModalInfo.sGg;
        switch (cGg)
        {
        case G94:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_FEED_PER_MIN, CMD_FEED_ADDR);
            break;
        case G95:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_FEED_PER_R, CMD_FEED_ADDR);
            break;
        default: //默认G94
            setBits(pSGCodeBuffUnit->unCommandType, CMD_FEED_PER_MIN, CMD_FEED_ADDR);
            break;
        }
        //1.6sGh
        int8 cGh = pSGCodeBuffUnit->SModalInfo.sGh;
        switch (cGh)
        {
        case G90:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_G90G91_G90, CMD_G90G91_ADDR);
            break;
        case G91:
            setBits(pSGCodeBuffUnit->unCommandType, CMD_G90G91_G91, CMD_G90G91_ADDR);
            break;
        default: //默认G90
            setBits(pSGCodeBuffUnit->unCommandType, CMD_G90G91_G90, CMD_G90G91_ADDR);
            break;
        }
        //1.7sGz
        int8 cGz = pSGCodeBuffUnit->SModalInfo.sGz;
        switch (cGz)
        {
            break;
        default:
            break;
        }

        //1.8插补模式
        switch (pSGCodeBuffUnit->cInterpolationState)
        {
        case 0: //普通插补模式
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_INTERPOLATION_COMMON, CMD_INTERPOLATION_MODE_ADDR); //置1
            break;
        }
        case 1: //极坐标插补模式
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_INTERPOLATION_POLAR, CMD_INTERPOLATION_MODE_ADDR); //置1
            break;
        }
        case 2: //圆柱插补模式
        {
            setBits(pSGCodeBuffUnit->unCommandType, CMD_INTERPOLATION_CYLINDRICAL, CMD_INTERPOLATION_MODE_ADDR); //置1
            break;
        }
        default:
            break;
        }
    }

    //4.处理有效轴问题（不做处理？运动预处理统一处理）
    {
        //setbit(pSGCodeBuffUnit->ucValStatus, AXIS_X);       适配双通道
        //setbit(pSGCodeBuffUnit->ucValStatus, AXIS_Z);

        for (int8 cTemp = 0; cTemp < AXIS_NUM_MAX; cTemp++)
        {
            if (!isZero(pSGCodeBuffUnit->SStartPoint.dArray[cTemp] - pSGCodeBuffUnit->SEndPoint.dArray[cTemp]))
            {
                setbit(pSGCodeBuffUnit->ucValStatus, cTemp);
            }
        }
        //G33特殊处理（shit）
        if (pSGCodeBuffUnit->SModalInfo.sGa == 33)
        {
            setbit(pSGCodeBuffUnit->ucValStatus, AXIS_Z);
        }
        if (pSGCodeBuffUnit->SModalInfo.sGa == 2 || pSGCodeBuffUnit->SModalInfo.sGa == 3)
        {
            setbit(pSGCodeBuffUnit->ucValStatus, AXIS_X);
            setbit(pSGCodeBuffUnit->ucValStatus, AXIS_Z);
        }
    }
}

//================================================================
// 自动倒角/倒圆：几何求解与缓冲尾部插入  2026-10-15
//================================================================

/* 尾部插入约定：解析器保留带 D/L 的尾段，完成裁剪后下游才能消费。
 * 编程状态使用原始终点，执行轨迹使用切点；生产与消费在同一规划上下文串行执行。
 */
/*************************************************
  函数名称：static bool hasCornerRequest(const struct SGCodeParse *p)
  函数功能：判断该段末端是否仍有待处理的 D/L 请求。
  输入参数：p 缓冲记录，不能为空。
  输出参数：无。
  返 回 值：TRUE 存在非零请求；FALSE 无请求。
  备    注：这里只检查是否存在请求；负值、非有限值及支持范围由写入流程校验。
**************************************************/
static bool hasCornerRequest(const struct SGCodeParse* p)
{
    return p->dRoundedRadius != 0.0 || p->dChamferLen != 0.0;
}
/* 日志与下游消费遵守同一规则：只发布几何已经定型的段。 */
/*************************************************
  函数名称：static void printFinalizedGCodeAt(int16 index, uint8 channel)
  函数功能：只打印已经完成倒角处理、几何不再变化的缓冲记录。
  输入参数：index 环形槽位索引；channel 有效通道号。
  输出参数：按打印模式追加已定型轨迹日志。
  返 回 值：无。
  备    注：仍带 D/L 的尾段延后输出；非主机或非轨迹打印模式下不执行打印。
**************************************************/
static void printFinalizedGCodeAt(int16 index, uint8 channel)
{
#if !STM32_MODE && (PRINTMODE == 1 || PRINTMODE == 2)
    void printGCodeAt(int16 index, uint8 channel);
    if (!hasCornerRequest(getSGCodeParsePointerByIndex(index, channel)))
        printGCodeAt(index, channel);
#else
    (void)index;
    (void)channel;
#endif
}
/*************************************************
  函数名称：void moveGCodeWritePtr(uint8 ucChannelNum)
  函数功能：提交当前解析段；有倒角/倒圆请求时按 a、new、b 顺序插入过渡段。
  输入参数：ucChannelNum 有效通道号，当前写槽已完成赋值。
  输出参数：成功时更新轨迹槽位及写指针；错误通过现有报警接口报告。
  返 回 值：无；调用方需检查 isErrorHasBeenWritten()。
  备    注：原始编程终点须先更新；局部构造全部成功再提交，b 自身请求保留供下一连接使用。
**************************************************/
void moveGCodeWritePtr(uint8 ucChannelNum)
{
    struct SGCodeParse *current, *previous, *next;
    struct SGCodeParse b, a, bridge, trimmedNext;
    int16 index;
    int8 status;
    bool pending;
    if (isErrorHasBeenWritten())
        return;

    //获取当前段和前一段
    current = getSGCodeParsePointerByType(GCODE_WRITE, ucChannelNum);
    index = getSGCodeParsePointerIndex(GCODE_WRITE, ucChannelNum);
    previous = getSGCodeParsePointerByIndex(index - 1, ucChannelNum);

    //至少一段已经写入了缓冲区，但还没被刀补模块读走且前段声明了倒角/圆角请求
    pending = getSGCodeParsePointerBufRes(GCODE_READ, ucChannelNum) > 0 && hasCornerRequest(previous);

    //convertGCodeBuff会把所有直线实体(包括 G00 快速移动)统一映射成 sGa=1,转换后无法区分 G00 和 G01
    //因此必须在转换前用原始 sGa 排除不支持倒角/圆角的运动类型。此处只允许 G01(10)、G02(20)、G03(30) 带 D/L 请求。
    if ((pending || hasCornerRequest(current)) && current->SModalInfo.sGa != 10 && current->SModalInfo.sGa != 20 &&
        current->SModalInfo.sGa != 30)
    {
        parseGCodeErrorB(current->unGCodeLineNum, 283);
        return;
    }

    // 只转换局部副本；任何校验失败均不覆盖原始写槽。
    b = *current;
    convertGCodeBuff(&b);
    if (hasCornerRequest(&b) && !isCornerRequestValid(&b))
    {
        parseGCodeErrorB(b.unGCodeLineNum, 283);
        return;
    }

    // 前段无请求时，当前段独立提交
    if (!pending)
    {
        *current = b;
        printFinalizedGCodeAt(index, ucChannelNum);
        moveSGCodeParsePointer(GCODE_WRITE, ucChannelNum);
        return;
    }

    // 容量 & 兼容性检查
    if (getSGCodeParsePointerBufRes(STOOL_READ_AHEAD, ucChannelNum) < 1 || !isCornerPairCompatible(previous, &b))
    {
        parseGCodeErrorB(previous->unGCodeLineNum, 283);
        return;
    }
    if (getSGCodeParsePointerBufRes(GCODE_WRITE, ucChannelNum) < 2)
    {
        parseGCodeErrorB(previous->unGCodeLineNum, 284);
        return;
    }

    //拐角过渡计算，前段请求在后段到齐后同步求解；计算与缓冲提交分离。
    status = calcCornerTransition(previous, &b, &a, &bridge, &trimmedNext);
    if (status < 0)
    {
        parseGCodeErrorB(previous->unGCodeLineNum,
                         status == RG_INVALID ? 283 : (previous->dChamferLen > 0 ? 281 : 282));
        return;
    }
    if (status == RG_UNCHANGED)
    {
        previous->dRoundedRadius = previous->dChamferLen = 0.0;
        *current = b;
        printFinalizedGCodeAt((int16)(index - 1), ucChannelNum);
        printFinalizedGCodeAt(index, ucChannelNum);
        moveSGCodeParsePointer(GCODE_WRITE, ucChannelNum);
        return;
    }

    // 提交区不再求解：将 b 及其资源所有权移到后一槽，原写槽由 new 替换。
    next = getSGCodeParsePointerByIndex(index + 1, ucChannelNum);
    initGCodeBufUnit(next);
    *previous = a;
    *next = trimmedNext;
    *current = bridge;
    printFinalizedGCodeAt((int16)(index - 1), ucChannelNum);
    printFinalizedGCodeAt(index, ucChannelNum);
    printFinalizedGCodeAt((int16)(index + 1), ucChannelNum);
    moveSGCodeParsePointer(GCODE_WRITE, ucChannelNum);
    moveSGCodeParsePointer(GCODE_WRITE, ucChannelNum);
}

struct SG50RingBuff g_SG50RingBuff[CHANNEL_NUM_MAX]; //G50环形缓冲区
/*************************************************
  函数名称：void assignG50(struct SGCodeParse *SCurrRecord, struct _SPriority *SPriorityContainer, SPointDouble *STotalOffset)
  函数功能：G50赋值
  输入参数：SCurrRecord本行的信息记录，SPriorityContainer优先级容器，STotalOffset总偏移值
  输出参数：
  返 回 值：

  备    注：目前只支持XZ
**************************************************/
void assignG50(uint8 ucSyntaxOrAssign, uint8 ucChannelNum, SSyntaxConfiguration* pSyntax, SAssignConfiguration* pAssign)
{
}

/*************************************************
  函数名称：int8 initG50RingBuffer(uint8 ucChannelNum)
  函数功能：初始化G50环形缓冲区
  输入参数：uint8 ucChannelNum 通道号
  输出参数：
  返 回 值：-1 出错  1 初始化完成

  备    注：
**************************************************/
int8 initG50RingBuffer(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return -1;
    }

    else
    {
        g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex = 0; //将对应通道号的G50环形缓冲区中的G50写索引初始化为0
        g_SG50RingBuff[ucChannelNum].sSG50_WRITEDNum =
            0; //将对应通道号的G50环形缓冲区中的G50已经写入且未被读走的个数初始化为0
        return 1;
    }
    return -1;
}

/*************************************************
  函数名称：int16 getSG50PointerIndex(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
  函数功能：获取G50指针的索引值（通过类型）
  输入参数：enum G50RingBufPointerType pointerTpye G50指针枚举变量   uint8 ucChannelNum通道号
  输出参数：
  返 回 值： int16 指针对应元素的索引 范围：（0~G50_INFO_BUF_MAX-1）
			-1	  输入的通道号或G50指针枚举类型错误

  备    注：使用时应该对获取的索引值进行判断，如果等于-1，说明出错
**************************************************/
int16 getSG50PointerIndex(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
    int16 sG50Index;

    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return -1;
    }

    switch (pointerTpye)
    {
    case SG50_WRITR:
        sG50Index = g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex;
        break;
    case SG50_READ:
        sG50Index = g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex - g_SG50RingBuff[ucChannelNum].sSG50_WRITEDNum;
        break;
    default:
        sG50Index = 0;
        break;
    }

    sG50Index %= G50_INFO_BUF_MAX; //对G50索引值取余，防止超出G50环形数组范围
    if (sG50Index < 0)             //如果索引值小于0，要对索引值进行平移
    {
        sG50Index += G50_INFO_BUF_MAX;
    }

    return sG50Index;
}

/*************************************************
  函数名称：struct  SG50Info* getSG50PointerByIndex(short sIndex, uint8 ucChannelNum)
  函数功能：获取G50指针（通过索引值）
  输入参数：short sIndex 索引值  uint8 ucChannelNum 通道号
  输出参数：
  返 回 值：struct  SG50Info* 数组元素指针

  备    注：
**************************************************/
union G50G10Info* getSG50PointerByIndex(short sIndex, uint8 ucChannelNum)
{

    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return NULL;
    }
    sIndex %= G50_INFO_BUF_MAX; //对索引进行取余，防止超出数组范围
    if (sIndex < 0)             //索引小于0 ，要进行平移
    {
        sIndex += G50_INFO_BUF_MAX;
    }
    return g_SG50RingBuff[ucChannelNum].G50G10Ringbuf + sIndex;
}

/*************************************************
  函数名称：struct SG50Info* getSG50PointerByType(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
  函数功能：获取G50指针（通过类型）
  输入参数：enum G50RingBufPointerType pointerTpye G50指针枚举变量  uint8 ucChannelNum 通道号
  输出参数：
  返 回 值：struct  SG50Info* 数组元素指针

  备    注：
**************************************************/
union G50G10Info* getSG50PointerByType(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
    int16 sG50Index;           //索引值
    union G50G10Info* pointer; //G50信息结构体指针

    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return NULL;
    }

    sG50Index = getSG50PointerIndex(pointerTpye, ucChannelNum); //获取对应类型的索引
    pointer = getSG50PointerByIndex(sG50Index, ucChannelNum);   //根据索引获取指针

    return pointer;
}

/*************************************************
  函数名称：int16 getSG50PointerBufRes(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
  函数功能：获取G50指针剩余容量（通过类型）
  输入参数：enum G50RingBufPointerType pointerTpye G50指针枚举变量  uint8 ucChannelNum 通道号
  输出参数：
  返 回 值：0~10：剩余容量大小       -1：输入的通道号或G50指针枚举类型错误

  备    注：使用时应该对返回值进行判断，如果返回值为-1，说明出错
**************************************************/
int16 getSG50PointerBufRes(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
    int16 sBufRes; //    能读/写容量

    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return -1;
    }

    switch (pointerTpye)
    {
    case SG50_WRITR:
        sBufRes = G50_INFO_BUF_MAX - g_SG50RingBuff[ucChannelNum].sSG50_WRITEDNum;
        break;
    case SG50_READ:
        sBufRes = g_SG50RingBuff[ucChannelNum].sSG50_WRITEDNum;
        break;
    default:
        sBufRes = -1;
        break;
    }

    return sBufRes;
}

/*************************************************
  函数名称：bool moveSG50Pointer(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
  函数功能：移动G50指针（通过类型）
  输入参数：enum G50RingBufPointerType pointerTpye G50指针枚举变量  uint8 ucChannelNum 通道号
  输出参数：
  返 回 值：TRUE	移动成功
			FALSE	移动失败

  备    注：使用时应该对返回值进行判断
**************************************************/
bool moveSG50Pointer(enum G50RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return FALSE;
    }

    int16 sBufRes; //    能读/写容量
    sBufRes = getSG50PointerBufRes(pointerTpye, ucChannelNum);
    if (sBufRes < 0)
    {
        return FALSE;
    }

    switch (pointerTpye)
    {
    case SG50_WRITR:
        g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex += 1;
        g_SG50RingBuff[ucChannelNum].sSG50_WRITEDNum += 1;
        break;
    case SG50_READ:

        g_SG50RingBuff[ucChannelNum].sSG50_WRITEDNum -= 1;
        break;
    default:
        return FALSE;
    }

    //读G50写入索引sSG50_WRITEIndex取余，防止超出数组范围
    g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex %= G50_INFO_BUF_MAX;
    if (g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex <
        0) //如果写入索引sSG50_WRITEIndex为负数，需要平移，防止Index小于零，返回野指针
    {
        g_SG50RingBuff[ucChannelNum].sSG50_WRITEIndex += G50_INFO_BUF_MAX;
    }
    return TRUE;
}

/*************************************************
  函数名称：struct SG50Info* getG50RingBuffHeadAddr(uint8 ucChannelNum)
  函数功能  返回G50缓冲区的首地址
  输入参数：uint8 ucChannelNum 通道号
  输出参数：
  返 回 值：struct SG50Info* 缓冲区首地址
  备    注：
**************************************************/
union G50G10Info* getG50RingBuffHeadAddr(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX) //超出数组范围
    {
        return NULL;
    }
    return g_SG50RingBuff[ucChannelNum].G50G10Ringbuf;
}

//适配企业不从第一行开始运行
int32 scanRealProgramBeginLine(uint8 ucChannelNum, SProgram* pSCurrProgram)
{
    SGetTextStateInfo STextInfo;
    uint8 ucRet;

    if (pSCurrProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }

    initGetTextStateInfo(&STextInfo);
    STextInfo.unGetLineNum = 0;

    while (1)
    {
        ucRet = getProgramSegment(&STextInfo, ucChannelNum, pSCurrProgram);

        // 空行/注释行/需要等待的行，跳过继续
        if (ucRet == 0 || ucRet == 1 || ucRet == 0xF1)
        {
            continue;
        }

        // 程序结束/文件结束，没找到普通程序段
        if (ucRet == 0xF2)
        {
            return -1;
        }

        // 其他异常
        if (ucRet == 0xF3 || ucRet == 0xF4)
        {
            return -1;
        }

        // ucRet == 2：有效程序段
        if (ucRet == 2)
        {
            int8* pTemp = STextInfo.cStr;

            // 跳过前导空白
            while (*pTemp == ' ' || *pTemp == '\t')
            {
                pTemp++;
            }

            // 空串，跳过
            if (*pTemp == '\0')
            {
                continue;
            }

            // 程序头 %，跳过
            if (*pTemp == '%')
            {
                continue;
            }

            // DEFINE 行，跳过
            if (pTemp[0] == 'D' && pTemp[1] == 'E' && pTemp[2] == 'F' && pTemp[3] == 'I' && pTemp[4] == 'N' &&
                pTemp[5] == 'E')
            {
                continue;
            }

            // 找到第一个普通程序段，返回其行号
            return (int32)STextInfo.unCurrLineNum;
        }
    }
}

int8 prepareProgramBeginLine(uint8 ucChannelNum)
{
    SProgram* pSCurrProgram = getCurrentProgram(ucChannelNum);
    if (pSCurrProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }

    if (pSCurrProgram->sBeginLineNum == UNASSIGNED_VALUE)
    {
        int32 beginLine = scanRealProgramBeginLine(ucChannelNum, pSCurrProgram);
        if (beginLine < 0)
            return -1;
        pSCurrProgram->sBeginLineNum = beginLine;
    }

    return 1;
}
