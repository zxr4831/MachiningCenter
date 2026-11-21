#include "GCode.h"
#include "ErrorFunction.h"

int8 CheckG40(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 CheckG41(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 CheckG42(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit);

int8 AssignG40(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG41(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);
int8 AssignG42(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit);

void addG40RegulationAPI()
{
    //指令：G2
    //组别：sGa
    //指令占程序段数：1
    //参数结构体：无
    addGCommadRegulation(40, sGd, 1, NULL, NULL, NULL, CheckG40, AssignG40);
}
void addG41RegulationAPI()
{
    //指令：G2
    //组别：sGa
    //指令占程序段数：1
    //参数结构体：无
    addGCommadRegulation(41, sGd, 1, NULL, NULL, NULL, CheckG41, AssignG41);
}

//添加G41规则结构体
void addG42RegulationAPI()
{
    //指令：G2
    //组别：sGa
    //指令占程序段数：1
    //参数结构体：无
    addGCommadRegulation(42, sGd, 1, NULL, NULL, NULL, CheckG42, AssignG42);
}

int8 CheckG40(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
    //0.判断G40程序段中是否出现坐标信息 即G40 G0/G1 等指令
    //int8 cRet = 0;
    //cRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);

    //if (cRet == FALSE)
    //{
    //	parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM409);
    //}

    return 2;
}
int8 CheckG41(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
    int8 cRet = 0;
    cRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);

    if (cRet == FALSE)
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM409);
    }
    pGCodeBufUnit->SModalInfo.sGd = G41 * pow(10, GCOMMAND_DECIMAL_PLACE);
    return 2;
}
//检查G41
int8 CheckG42(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
    int8 cRet = 0;
    cRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);

    if (cRet == FALSE)
    {
        parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM409);
    }
    pGCodeBufUnit->SModalInfo.sGd = G42 * pow(10, GCOMMAND_DECIMAL_PLACE);

    return 2;
}

int8 AssignG40(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
    pGCodeBufUnit->SModalInfo.sGd = G40 * pow(10, GCOMMAND_DECIMAL_PLACE);
    return 2;
}
int8 AssignG41(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
    pGCodeBufUnit->SModalInfo.sGd = G41 * pow(10, GCOMMAND_DECIMAL_PLACE);
    return 2;
}
int8 AssignG42(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
    pGCodeBufUnit->SModalInfo.sGd = G42 * pow(10, GCOMMAND_DECIMAL_PLACE);
    return 2;
}
