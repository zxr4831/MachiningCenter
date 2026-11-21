#include "GCode.h"
#include "ErrorFunction.h"

static int8 validateGotoValue(double dGoto, uint32* pOutSeg, uint32 unLineNum);

//添加checkGOTO在检查状态机  2025.11.27
int8 checkGOTO(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
    if (pSSyntaxConfig->SScanContainer.bGOTO == TRUE)
    {
        int8 cRet = 0;
        uint32 unTargetSegment;

        //1.获取当前解析的程序文本
        SProgram* pCurrProgram = FindNodeData(getProgramList(pSSyntaxConfig->ucChannelNum), 0); //当前程序结构体
        if (pCurrProgram == NULL)
        {
            parseGCodeErrorB(0, GCODE_ERROR_NUM300);
            return -1;
        }
        // **新增：非负 + 整数 + 范围校验（在(uint32)转换前）**
        if (validateGotoValue(pSSyntaxConfig->SScanContainer.dGOTO, &unTargetSegment,
                              pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum) < 0)
        {

            return -1;
        }
        //3.找
        cRet = findTargetSegment(GC_SYNTAX, pSSyntaxConfig->ucChannelNum, pCurrProgram, unTargetSegment,
                                 &pSSyntaxConfig->SGetTextStateInfo);

        return cRet;
    }
    return 0;
}

int8 assignGOTO(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
    if (pSAssignConfig->SScanContainer.bGOTO == TRUE)
    {
        int8 cRet = 0;
        uint32 unTargetSegment;

        //1.获取当前解析的程序文本
        SProgram* pCurrProgram = FindNodeData(getProgramList(pSAssignConfig->ucChannelNum), 0); //当前程序结构体
        if (pCurrProgram == NULL)
        {
            parseGCodeErrorB(0, GCODE_ERROR_NUM300);
            return -1;
        }
        // **新增：非负 + 整数 + 范围校验（在(uint32)转换前）**
        if (validateGotoValue(pSAssignConfig->SScanContainer.dGOTO, &unTargetSegment,
                              pSAssignConfig->SGetTextStateInfo.unCurrLineNum) < 0)
        {

            return -1;
        }
        //3.找
        cRet = findTargetSegment(GC_ASSIGN, pSAssignConfig->ucChannelNum, pCurrProgram, unTargetSegment,
                                 &pSAssignConfig->SGetTextStateInfo);

        return cRet;
    }
    return 0;
}

static int8 validateGotoValue(double dGoto, uint32* pOutSeg, uint32 unLineNum)
{
    if (pOutSeg == NULL)
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
        return -1;
    }

    // **1) 必须非负**
    if (dGoto < 0.0)
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // **2) 必须是整数：dGoto == floor(dGoto)**
    // 这样能拦住 1.2、3.0(允许)、12.999 等
    if (dGoto != floor(dGoto))
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    // **3) 必须在 uint32 范围内**
    if (dGoto > (double)0xffffffff)
    {
        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
        return -1;
    }

    *pOutSeg = (uint32)dGoto;
    return 1;
}
