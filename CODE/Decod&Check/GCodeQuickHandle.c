#include "Gcode.h"
#include "ErrorFunction.h"

/*************************************************
  函数名称：IsGodeCoordSustained()
  函数功能  判断模态G代码能否快速处理
  输入参数：
  输出参数：
  返 回 值：TRUE 能快速处理 FALSE 不能快速处理
  备    注：
**************************************************/
bool IsGodeCoordSustained(SAssignConfiguration* pSAssignConfig)
{
    if (pSAssignConfig->pGCodeBuf[pSAssignConfig->usIndex].EntityType == DOT)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/*************************************************
  函数名称：IsModalInfoChanged()
  函数功能  判断该行模态G代码是否改变
  输入参数：
  输出参数：
  返 回 值：TRUE 模态信息改变 FALSE 模态信息未改变
  备    注：
**************************************************/
bool IsModalInfoChanged(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf,
                        enum ModalNum* pModalChangedNumArray, int8* pCounter)
{
    if (pModalChangedNumArray == NULL)
    {
        return TRUE;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa != pGCodeBuf->SModalInfo.sGa)
    {
        enum ModalNum ModalNumNode = sGa;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGb != pGCodeBuf->SModalInfo.sGb)
    {
        enum ModalNum ModalNumNode = sGb;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGc != pGCodeBuf->SModalInfo.sGc)
    {
        enum ModalNum ModalNumNode = sGc;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGd != pGCodeBuf->SModalInfo.sGd)
    {
        enum ModalNum ModalNumNode = sGd;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGe != pGCodeBuf->SModalInfo.sGe)
    {
        enum ModalNum ModalNumNode = sGe;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGf != pGCodeBuf->SModalInfo.sGf)
    {
        enum ModalNum ModalNumNode = sGf;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGg != pGCodeBuf->SModalInfo.sGg)
    {
        enum ModalNum ModalNumNode = sGg;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGh != pGCodeBuf->SModalInfo.sGh)
    {
        enum ModalNum ModalNumNode = sGh;
        pModalChangedNumArray[*pCounter] = ModalNumNode;
        (*pCounter)++;
    }
    return (*pCounter) ? TRUE : FALSE;
}

/*************************************************
  函数名称：IsModalInfoChangedAPI
  函数功能  判断该行模态G代码是否改变
  输入参数：
  输出参数：
  返 回 值：TRUE 改变了 FALSE 未改变
  备    注：
**************************************************/
bool IsModalInfoChangedAPI(SAssignConfiguration* pSAssignConfig)
{
    return (pSAssignConfig->SQuickHandleFlag.bIsModalInfoChangFlag) ? TRUE : FALSE;
}

/*************************************************
  函数名称：CanModalChangedQuickHandle()
  函数功能  判断该行模态G代码是否改变，能否快速处理
  输入参数：
  输出参数：
  返 回 值：TRUE 能快速处理 FALSE 不能快速处理
  备    注：
**************************************************/
bool CanModalChangedQuickHandle(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf,
                                enum ModalNum* pModalChangedNumArray, int8* pCounter)
{
    int nCounter = 0;
    int8 cCanHandleFlag = 1;
    if (pSAssignConfig == NULL)
    {
        return FALSE;
    }
    if (pGCodeBuf == NULL)
    {
        return FALSE;
    }
    if (pModalChangedNumArray == NULL)
    {
        return FALSE;
    }
    for (nCounter = 0; nCounter < (*pCounter); nCounter++)
    {
        //int8 sGa, sGb, sGc, sGd, sGe, sGf, sGg, sGh;
        /// G代码分类参数，
        /// sGa：运动类 G00，G01，G02，G03，G32,G33,G76,G92						MOCTION
        /// sGb：工件坐标系 G54-G59												COOR
        /// sGc：定义插补平面 G17-G19
        /// sGd：刀具补偿 G40-G42												TOOLCOMP
        /// sGe：单位选择：英制/公制 G20，G21									UNITS   INCHS MM
        /****** 新增模态组 *******/ //2021-9-13  冯英华
        /// sGf: 恒线速控制 G96、恒转速控制 G97									SPINDLE_SPEED
        /// sGg: 给定每分进给速度 G98 、以毫米/转为单位给定切削进给速度 G99		FEED
        /// sGh: 绝对编程G90/ 相对编程G91  默认G90
        ///
        int nTemp = pModalChangedNumArray[nCounter];
        switch (nTemp)
        {
        case sGa: //坐标不改变的情况G0G1的切换允许快速处理
            if (pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa == G00 ||
                pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa == G01)
            {
                if ((pGCodeBuf->SModalInfo.sGa == G00 || pGCodeBuf->SModalInfo.sGa == G01))
                {
                    break;
                }
            }
            cCanHandleFlag &= 0;
            break;
        case sGb: //G54到G59坐标系的切换允许快速处里
            break;
        default:
            cCanHandleFlag &= 0;
            break;
        }
    }
    return (cCanHandleFlag) ? TRUE : FALSE;
}

/*************************************************
  函数名称：CanModalChangedQuickHandle()
  函数功能  判断该行模态G代码是否改变，能否快速处理
  输入参数：
  输出参数：
  返 回 值：TRUE 能快速处理 FALSE 不能快速处理
  备    注：
**************************************************/
bool CanModalChangedQuickHandleAPI(SAssignConfiguration* pSAssignConfig)
{
    return (pSAssignConfig->SQuickHandleFlag.bCanModalInfoQuickHandleFlag) ? TRUE : FALSE;
}

/*************************************************
  函数名称：IsNonModalMcodeInfoChanged()
  函数功能  判断非模态M代码是否改变
  输入参数：
  输出参数：
  返 回 值：TRUE 改变了 FALSE 未改变
  备    注：
**************************************************/
bool IsNonModalMcodeInfoChanged(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf, uint8 ucChannelNum)
{

    //int8 cMa, cMb, cMc, cMd, cMe, cMf, cMg, cMh;
    /// M代码分类参数，
    /// cMa：主轴正/反转/停		/M03/M04/M05(非模态）
    /// cMb：程序停/程序结束	/M00/M01/M02/M30(非模态)	//可快速处理，不能扔
    /// cMc：冷却液开/关		/M08/M09(非模态)			//可快速处理，不能扔
    /// cMd：M96 M97 M98 M99
    /// cMe：润滑油开/关        /M32/M33					//可快速处理，不能扔
    /// cMf：工件夹紧 / 松开 / M10 / M11
    /// cMg：M14 / M15
    /// cMh: M50/51/52

    //if (
    //	(pGCodeBuf->SNonModalInfo.cMa != 0)
    //	|| (pGCodeBuf->SNonModalInfo.sMb != -1)
    //	|| (pGCodeBuf->SNonModalInfo.cMc != 0)
    //	|| (pGCodeBuf->SNonModalInfo.cMe != 0)
    //	|| (pGCodeBuf->SNonModalInfo.cMf != 0)
    //	|| (pGCodeBuf->SNonModalInfo.cMg != 0)
    //	|| (pGCodeBuf->SNonModalInfo.cMh != 0)
    //	)
    //{
    //	return TRUE;
    //}

    //return FALSE;
}

/*************************************************
  函数名称：CanNonModalMcodeQuickHandle()
  函数功能  判断非模态M代码改变的情况下能否快速处理
  输入参数：
  输出参数：
  返 回 值：TRUE 能快速处理 FALSE 不能快速处理
  备    注：
**************************************************/
bool CanNonModalMcodeQuickHandle(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf,
                                 uint8 ucChannelNum)
{
    //int8 cNeedHandleFlag = 1;

    //switch (pGCodeBuf->SNonModalInfo.cMa)
    //{
    //case 0:
    //	break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //switch (pGCodeBuf->SNonModalInfo.sMb)
    //{
    //case 0:
    //	break;
    //case -1:
    //	break;
    //   case M99:
    //       break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //// cMc：冷却液开/关		/M08/M09(非模态)
    //switch (pGCodeBuf->SNonModalInfo.cMc)
    //{
    //case 0:
    //	break;
    //case M08:
    //	break;
    //case M09:
    //	break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //// cMe：润滑油开/关        /M32/M33
    //switch (pGCodeBuf->SNonModalInfo.cMe)
    //{
    //case 0:
    //	break;
    //case M32:
    //	break;
    //case M33:
    //	break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //switch (pGCodeBuf->SNonModalInfo.cMf)
    //{
    //case 0:
    //	break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //switch (pGCodeBuf->SNonModalInfo.cMg)
    //{
    //case 0:
    //	break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //switch (pGCodeBuf->SNonModalInfo.cMh)
    //{
    //case 0:
    //	break;
    //default:
    //	cNeedHandleFlag &= 0;
    //	break;
    //}

    //return ((cNeedHandleFlag) ? TRUE : FALSE);
}

/*************************************************
  函数名称：CanNonModalGcodeQuickHandle()
  函数功能  判断非模态G代码改变的情况下能否快速处理
  输入参数：
  输出参数：
  返 回 值：TRUE 能快速处理 FALSE 不能快速处理
  备    注：
**************************************************/
bool CanNonModalGcodeQuickHandle(struct SGCodeParse* pGCodeBuf, uint8 ucChannelNum)
{
    //// sGz：非模态组别
    ////case G04:case G50:case G65:case G66:case G67:case G70:case G71:case 72:case G73:case G74:case G75:case G76:case G10:

    //switch (pGCodeBuf->SNonModalInfo.sGz)
    //{
    //case G04:	////G04延时时间为0
    //{
    //	if (pGCodeBuf->unDelayTime == 0)
    //	{
    //		return TRUE;
    //	}
    //}
    //break;

    //case G50:	//G50没输入S值
    //{
    //	if (pGCodeBuf->dSpindleSpeed == UNASSIGNED_VALUE)
    //	{
    //		return TRUE;
    //	}
    //}
    //break;

    //default:
    //	break;
    //}

    //return FALSE;
}

/*************************************************
  函数名称：IsNonModalGcodeInfoChanged()
  函数功能  判断非模态G代码是否改变
  输入参数：
  输出参数：
  返 回 值：TRUE 改变了 FALSE 没改变
  备    注：
**************************************************/
bool IsNonModalGcodeInfoChanged(struct SGCodeParse* pGCodeBuf, uint8 ucChannelNum)
{
    //if (pGCodeBuf->SNonModalInfo.sGz)
    //{
    //	return TRUE;
    //}
    //return FALSE;
}

/*************************************************
  函数名称：IsTurretModeTCodeChanged()
  函数功能  判断刀塔机模式下刀具是否改变
  输入参数：
  输出参数：
  返 回 值：FALSE 刀具没改变 TRUE 刀具改变了
  备    注：
**************************************************/
bool IsTurretModeTCodeChanged(SAssignConfiguration* pSAssignConfig)
{
    //if (g_SSystemPara.ucToolDevie == TOOL_TURRET_MODE)
    //{
    //	if (pSAssignConfig->SPriorityContainer.bT)
    //	{
    //		return TRUE;
    //	}
    //}

    //return FALSE;
}

/*************************************************
  函数名称：IsToolRawModeTCodeChanged()
  函数功能  判断排刀机模式下刀具是否改变
  输入参数：
  输出参数：
  返 回 值：FALSE 刀具没改变 TRUE 刀具改变了
  备    注：
**************************************************/
bool IsToolRawModeTCodeChanged(SAssignConfiguration* pSAssignConfig)
{
    //if (g_SSystemPara.ucToolDevie == TOOL_RAWTOOL_MODE)
    //{
    //	if (pSAssignConfig->SPriorityContainer.bT)
    //	{
    //		return TRUE;
    //	}
    //}

    //return FALSE;
}

/*************************************************
  函数名称：IsFeedrateOrSpindleSpeedChanged()
  函数功能  判断进给速度或转速是否有改变
  输入参数：
  输出参数：
  返 回 值：TRUE 改变了 FALSE 没改变
  备    注：
**************************************************/
bool IsFeedrateOrSpindleSpeedChanged(SAssignConfiguration* pSAssignConfig)
{
    //if (pSAssignConfig->SPriorityContainer.bS || pSAssignConfig->SPriorityContainer.bF)
    //{
    //	return TRUE;
    //}

    //return FALSE;
}

/*************************************************
  函数名称：JudgeNonModalGcodeNeedQuickHandle()
  函数功能  判断非模态G代码能否快速处理
  输入参数：
  输出参数：
  返 回 值：2  GCODE_CAN_MERGED 能合并
			-1 不能合并
  备    注：
**************************************************/
int8 JudgeNonModalGcodeNeedQuickHandle(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf,
                                       uint8 ucChannelNum)
{
    if (IsNonModalGcodeInfoChanged(pGCodeBuf, ucChannelNum))
    {
        if (CanNonModalGcodeQuickHandle(pGCodeBuf, ucChannelNum))
        {
            return GCODE_CAN_MERGED;
        }
        else
        {
            return -1; //非模态G代码不能快速处理
        }
    }
    else
    {
        return GCODE_CAN_MERGED;
    }
}

/*************************************************
  函数名称：JudgeNonModalCodeNeedQuickHandle()
  函数功能  判断非模态代码能否快速处理,调用了判断非模态G代码能否快速处理的函数和判断非模态M代码能否快速处理的函数
  输入参数：
  输出参数：
  返 回 值：1 GCODE_CAN_QUICKHANDLE  能快速处理
			2 GCODE_CAN_MERGED		能合并
			-2 非模态G代码不能快速处理
			-3 非模态M代码不能快速处理
  备    注：
**************************************************/
int8 JudgeNonModalCodeNeedQuickHandle(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf,
                                      uint8 ucChannelNum)
{
    if (IsNonModalMcodeInfoChanged(pSAssignConfig, pGCodeBuf, ucChannelNum)) //判断非模态M指令是否改变
    {
        if (CanNonModalMcodeQuickHandle(pSAssignConfig, pGCodeBuf,
                                        ucChannelNum)) //判断改变的非模态M指令是否可以快速处理
        {
            if (JudgeNonModalGcodeNeedQuickHandle(pSAssignConfig, pGCodeBuf, ucChannelNum) ==
                GCODE_CAN_MERGED) //判断非模态G指令是否改变或是否能快速处理
            {
                return GCODE_CAN_QUICKHANDLE; //非模态G代码能快速处理,非模态M代码也能快速处理，但非模态M代码改变，不能融合
            }
            else
            {
                return -2; //非模态G代码不能快速处理
            }
        }
        else
        {
            return -3; //但非模态M代码改变，且不能快速处理
        }
    }
    else
    {
        return JudgeNonModalGcodeNeedQuickHandle(pSAssignConfig, pGCodeBuf,
                                                 ucChannelNum); //判断非模态G指令是否改变或是否能快速处理
    }
}

/*************************************************
  函数名称：JudgeGodeQuickHandleType()
  函数功能  判断该行NC代码能否快速处理
  输入参数：
  输出参数：
  返 回 值：	2	G代码能合并
				1	G代码能快速处理
				-2	模态指令没改变，但非模态G代码不能快速处理
				-3 模态指令没改变，但非模态G代码不能快速处理
				-4 模态指令改变，该指令能快速处理，但非模态G/M代码不能快速处理
				-5 模态指令改变，且该模态不能快速处理
				-6 1.程序有输入F/S值	2.终点改变	3.刀架模式为刀塔模式
  备    注：
**************************************************/
int8 JudgeGodeQuickHandleType(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf, uint8 ucChannelNum)
{
    //不快速处理的情况:1.程序有输入F/S值	2.终点改变	3.刀架模式为刀塔模式改变刀具
    if (!IsFeedrateOrSpindleSpeedChanged(pSAssignConfig) && IsGodeCoordSustained(pSAssignConfig) &&
        !IsTurretModeTCodeChanged(pSAssignConfig))
    {
        //模态改变或排刀机模式下改变刀具
        if (IsModalInfoChangedAPI(pSAssignConfig) || IsToolRawModeTCodeChanged(pSAssignConfig))
        {
            if (CanModalChangedQuickHandleAPI(pSAssignConfig)) //但仍能根据模态改变的信息决定是否能快速处理
            {
                if (JudgeNonModalCodeNeedQuickHandle(pSAssignConfig, pGCodeBuf, ucChannelNum) > 0) //判断非模态指令
                {
                    return GCODE_CAN_QUICKHANDLE; //非模态G代码能快速处理,非模态M代码也能快速处理，但模态代码改变，不能融合,只能将类型设为快速处理
                }
                else
                {
                    return -4; //非模态G/M代码不能快速处理
                }
            }
            else
            {
                return -5; //模态改变不能快速处理
            }
        }
        else
        {
            return JudgeNonModalCodeNeedQuickHandle(pSAssignConfig, pGCodeBuf, ucChannelNum); //判断非模态指令
        }
    }
    return -6;
}
