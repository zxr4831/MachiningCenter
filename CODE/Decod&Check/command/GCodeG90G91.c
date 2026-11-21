
#include "common.h"

#include "GCode.h"

//int8 CheckG90(SSyntaxConfiguration* pSSyntaxConfig);
//int8 CheckG91(SSyntaxConfiguration* pSSyntaxConfig);
//int8 AssignG90(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);
//int8 AssignG91(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit);

//添加G90规则结构体
void addG90RegulationAPI()
{
    // 铣削配置将 G90 解释为绝对坐标编程。
    //组别：sGh
    //运行指令字：32位
    //指令占程序段数：1
    //参数结构体：无
    //char8* str=“”
    addGCommadRegulation(90, sGh, 1, NULL, NULL, NULL, CheckG90, AssignG90);
}
//添加G91规则结构体
void addG91RegulationAPI()
{
    //指令：G91
    //组别：sGa
    //运行指令字：32位
    //指令占程序段数：1
    //参数结构体：无
    //char8* str=“”
    addGCommadRegulation(91, sGh, 1, NULL, NULL, NULL, CheckG91, AssignG91);
}

//检查G90
int8 CheckG90(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    pSGCodeParseUnit->SModalInfo.sGh = G90 * pow(10, GCOMMAND_DECIMAL_PLACE);
    pSSyntaxConfig->SGCodeCurrSysState.bRelativeCoordinateProgramming = FALSE;
    return 2;
}
//检查G91
int8 CheckG91(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    pSGCodeParseUnit->SModalInfo.sGh = G91 * pow(10, GCOMMAND_DECIMAL_PLACE);
    pSSyntaxConfig->SGCodeCurrSysState.bRelativeCoordinateProgramming = TRUE;
    return 2;
}

//赋值G90
int8 AssignG90(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    pSGCodeParseUnit->SModalInfo.sGh = G90 * pow(10, GCOMMAND_DECIMAL_PLACE);
    pSAssignConfig->SGCodeCurrSysState.bRelativeCoordinateProgramming = FALSE;
    return 2;
}
//赋值G91
int8 AssignG91(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pSGCodeParseUnit)
{
    pSGCodeParseUnit->SModalInfo.sGh = G91 * pow(10, GCOMMAND_DECIMAL_PLACE);
    pSAssignConfig->SGCodeCurrSysState.bRelativeCoordinateProgramming = TRUE;
    return 2;
}
