
#include "common.h"
#include "ringbuff.h"
#include "ErrorFunction.h"

#include "GCode.h"

#define CHIPBREAK_DELAY_MIN_MS (0.0)
#define CHIPBREAK_DELAY_MAX_MS (99999999.0)

// 判断 double 是否为整数值（允许 1.0 / 10.0 这类输入）
static bool isIntegerDouble(double v)
{
    // v 已保证 isfinite
    double iv = floor(v);
    return (v == iv);
}

// 统一的 P 校验：**0~99999999 且必须为整数 ms**
static int8 validateChipBreakP(SSyntaxConfiguration* pSSyntaxConfig, double dP)
{
    if (!isfinite(dP))
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // **范围：0 <= P <= 99999999**
    if (dP < CHIPBREAK_DELAY_MIN_MS || dP > CHIPBREAK_DELAY_MAX_MS)
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // **必须为整数 ms**
    if (!isIntegerDouble(dP))
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    return 2;
}

static int8 validateChipBreakP_Assign(SAssignConfiguration* pSAssignConfig, double dP)
{
    if (!isfinite(dP))
    {
        parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    if (dP < CHIPBREAK_DELAY_MIN_MS || dP > CHIPBREAK_DELAY_MAX_MS)
    {
        parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    if (!isIntegerDouble(dP))
    {
        parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    return 2;
}
//添加G4规则结构体
void addG4RegulationAPI()
{
    //指令：G4
    //组别：sGa
    //运行指令字：32位
    //指令占程序段数：1
    //参数结构体：无
    //char8* str=“”
    addGCommadRegulation(4, sGz, 1, NULL, NULL, NULL, CheckG4, AssignG4);
}

//检查G4
int8 CheckG4(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    int8 ret = 0;
    //1.参数检查
    //1.1指令字符检查
    ret = checkParaLetter("PXU", &pSSyntaxConfig->SScanContainer);
    if (ret == -1)
    {
        parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }
    // 2) 仅对“出现了”的参数做范围校验（避免未出现被 checkParaRange 判失败）
    if (pSSyntaxConfig->SScanContainer.bXYZ[0] == TRUE)
    {
        //1.2参数值范围检查
        ret = checkParaRange('X', &pSSyntaxConfig->SScanContainer, 0, 9999);
        if (ret == -1)
        {
            parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM134);
            return -1;
        }
    }
    if (pSSyntaxConfig->SScanContainer.bUVW[0] == TRUE)
    {
        ret = checkParaRange('U', &pSSyntaxConfig->SScanContainer, 0, 9999);
        if (ret == -1)
        {
            parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM134);
            return -1;
        }
    }
    if (pSSyntaxConfig->SScanContainer.bP == TRUE)
    {
        ret = checkParaRange('P', &pSSyntaxConfig->SScanContainer, 0, 99999);
        if (!isIntegerDouble(pSSyntaxConfig->SScanContainer.dP))
            ret = -1;
        if (ret == -1)
        {
            parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM134);
            return -1;
        }
    }

    // X/U/P 是同一等待时间的不同写法，不能同时指定。
    if ((pSSyntaxConfig->SScanContainer.bXYZ[0] == TRUE) + (pSSyntaxConfig->SScanContainer.bUVW[0] == TRUE) +
            (pSSyntaxConfig->SScanContainer.bP == TRUE) >
        1)
    {
        parseGCodeErrorB(pSSyntaxConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM442);
        return -1;
    }

    //2.将G4写入缓冲区
    if (pSSyntaxConfig->SScanContainer.bXYZ[0] == TRUE)
    {
        pSSyntaxConfig->SCurrRecord.unDelayTime = pSSyntaxConfig->SScanContainer.dXYZ[0] * 1000;
    }
    else if (pSSyntaxConfig->SScanContainer.bUVW[0] == TRUE)
    {
        pSSyntaxConfig->SCurrRecord.unDelayTime = pSSyntaxConfig->SScanContainer.dUVW[0] * 1000;
    }
    else if (pSSyntaxConfig->SScanContainer.bP == TRUE)
    {
        pSSyntaxConfig->SCurrRecord.unDelayTime = pSSyntaxConfig->SScanContainer.dP;
    }
    return 2;
}

//赋值G4
int8 AssignG4(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    /* P 按整数毫秒解释，小数延时按非法参数报错。 */
    if (pSAssignConfig->SScanContainer.bP &&
        (!isfinite(pSAssignConfig->SScanContainer.dP) || pSAssignConfig->SScanContainer.dP < 0 ||
         pSAssignConfig->SScanContainer.dP > 99999 || !isIntegerDouble(pSAssignConfig->SScanContainer.dP)))
    {
        parseGCodeErrorB(pSAssignConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // X/U/P 是同一等待时间的不同写法，不能同时指定。
    if ((pSAssignConfig->SScanContainer.bXYZ[0] == TRUE) + (pSAssignConfig->SScanContainer.bUVW[0] == TRUE) +
            (pSAssignConfig->SScanContainer.bP == TRUE) >
        1)
    {
        parseGCodeErrorB(pSAssignConfig->SScanContainer.unLineNum, GCODE_ERROR_NUM442);
        return -1;
    }

    //1.将G4写入缓冲区
    if (pSAssignConfig->SScanContainer.bXYZ[0] == TRUE)
    {
        pSGCodeParseUnit->unDelayTime = pSAssignConfig->SScanContainer.dXYZ[0] * 1000;
    }
    else if (pSAssignConfig->SScanContainer.bUVW[0] == TRUE)
    {
        pSGCodeParseUnit->unDelayTime = pSAssignConfig->SScanContainer.dUVW[0] * 1000;
    }
    else if (pSAssignConfig->SScanContainer.bP == TRUE)
    {
        pSGCodeParseUnit->unDelayTime = pSAssignConfig->SScanContainer.dP;
    }
    return 2;
}

//添加G4规则结构体
void addG4_9RegulationAPI()
{
    //指令：G4
    //组别：sGa
    //运行指令字：32位
    //指令占程序段数：1
    //参数结构体：无
    //char8* str=“”
    addGCommadRegulation(4.9, sGz, 1, NULL, NULL, NULL, CheckG4_9, AssignG4_9);
}

//检查G4
int8 CheckG4_9(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    bool bP = (pSSyntaxConfig->SScanContainer.bP == TRUE);
    bool bQ = (pSSyntaxConfig->SScanContainer.bQ == TRUE);

    // **关闭断屑：不带任何参数**
    if (!bP && !bQ)
    {
        return 2;
    }

    // **开启断屑：必须同时给出 P 和 Q**
    if (!(bP && bQ))
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }
    // Q 约束：**Q >= 1.0**
    double dQ = pSSyntaxConfig->SScanContainer.dQ;
    if (dQ < 1.0)
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // P 约束：**0~99999999ms 且整数**
    double dP = pSSyntaxConfig->SScanContainer.dP;
    if (validateChipBreakP(pSSyntaxConfig, dP) < 0)
    {
        return -1;
    }

    return 2;
}

//赋值G4
int8 AssignG4_9(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    bool bP = (pSAssignConfig->SScanContainer.bP == TRUE);
    bool bQ = (pSAssignConfig->SScanContainer.bQ == TRUE);

    // **关闭断屑：仅允许无参**
    if (!bP && !bQ)
    {
        pSAssignConfig->SGCodeCurrSysState.bChipBreakFlag = FALSE;
        pSAssignConfig->SGCodeCurrSysState.dChipBreakLength = 0;
        pSAssignConfig->SGCodeCurrSysState.unChipBreakDelayTime = 0;

        pSGCodeParseUnit->bChipBreakFlag = FALSE;
        pSGCodeParseUnit->dChipBreakLength = 0;
        pSGCodeParseUnit->unChipBreakDelayTime = 0;
        return 2;
    }

    // **开启断屑：必须同时给 P/Q，否则拒绝赋值并报错**
    if (!(bP && bQ))
    {
        parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    double dQ = pSAssignConfig->SScanContainer.dQ;

    // **Q>=1，否则拒绝赋值**
    if (dQ < 1.0)
    {
        parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unGetLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // ***关键修复：P 的边界/整数/finite 校验（Assign侧闭环）***
    double dP = pSAssignConfig->SScanContainer.dP;
    if (validateChipBreakP_Assign(pSAssignConfig, dP) < 0)
    {
        return -1;
    }

    // 开启断屑（参数明确给出）
    pSAssignConfig->SGCodeCurrSysState.bChipBreakFlag = TRUE;
    pSAssignConfig->SGCodeCurrSysState.dChipBreakLength = dQ;
    pSAssignConfig->SGCodeCurrSysState.unChipBreakDelayTime = pSAssignConfig->SScanContainer.dP;

    pSGCodeParseUnit->bChipBreakFlag = TRUE;
    pSGCodeParseUnit->dChipBreakLength = dQ;
    pSGCodeParseUnit->unChipBreakDelayTime = pSAssignConfig->SScanContainer.dP;

    return 2;
}
