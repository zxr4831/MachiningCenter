
#include "common.h"
#include "GCode.h"
#include "GeneralFunctions.h"

//添加G1规则结构体
void addG0RegulationAPI()
{
    //指令：G1
    //组别：sGa
    //运行指令字：32位
    //指令占程序段数：1
    //参数结构体：无
    //char8* str=“”
    addGCommadRegulation(0, sGa, 1, NULL, NULL, NULL, CheckG0, AssignG0);
}

/*************************************************
  函数名称：CheckG0
  函数功能：检查 G0 的坐标输入并计算当前检查记录。
  输入参数：有效检查配置和记录；本段非运动模态已更新，扫描坐标尚未转换。
  输出参数：沿用原坐标转换和记录写入；G16 输入拒绝时扫描数据和记录不变。
  返 回 值：2 本段检查完成；-1 G16 输入非法或原有失败分支。
  备    注：坐标处理区先校验 G16 输入；其他参数和几何检查沿用原流程。
**************************************************/
int8 CheckG0(SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
    //0.判断G0程序段中是否出现坐标信息
    bool bRet = 0;
    bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
    if (bRet == TRUE)
    {
        // 按本段坐标模态检查输入，通过后再进行坐标转换。
        if (checkPolarCoordinateInput(&pSSyntaxConfig->SScanContainer, &pSSyntaxConfig->SGCodeCurrSysState) < 0)
        {
            return -1;
        }

        //1.指令字符检查
        checkParaLetter("XYZABCUVW", &pSSyntaxConfig->SScanContainer);

        //2.参数值范围检查
        checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('A', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('B', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('C', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
        checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);

        //1.填写SLineInfo
        SLine SLineInfo; //存直线段信息（可能要将此变量放在赋值结构体中）
        bool bIsAbsMethod;
        memset(&SLineInfo, 0, sizeof(SLine));
        //1.1初始化SLine的起点终点（机械坐标）
        SLineInfo.SEndPoint = SLineInfo.SStartPoint =
            pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint; //系统状态结构体的当前点
        //1.2获取SLine的终点（机械坐标）
        //获取当前的坐标编程方式
        bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSSyntaxConfig->ucChannelNum);
        //计算坐标点
        getMacCoordByScanInfo(pSSyntaxConfig->ucChannelNum, &pSSyntaxConfig->SScanContainer, &SLineInfo.SStartPoint,
                              &SLineInfo.SEndPoint, &pSSyntaxConfig->SGCodeCurrSysState);
        //1.3
        SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;
        //1.2获取SLineInfo的指令值
        //SLineInfo.sCommand = G01 * GCOMMAND_DECIMAL_PLACE * 10;

        //2.将SLine写入缓冲区
        writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);
        return 2;
    }
    return 2;
}

/*************************************************
  函数名称：AssignG0
  函数功能：根据赋值状态计算快速定位段并填写当前记录
  输入参数：有效赋值配置与输出记录；扫描输入沿用坐标转换的单位契约
  输出参数：写入起终点和实体类型；普通显式C设置C轴有效位
  返 回 值：2 本段赋值完成；-1 G16 输入非法，扫描数据和输出记录不变。
  备    注：G16的C仅作X/Y极角，不额外设置物理C轴有效位
**************************************************/
int8 AssignG0(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
    //1.填写SLineInfo
    SLine SLineInfo; //存直线段信息（可能要将此变量放在赋值结构体中）
    bool bIsAbsMethod;
    memset(&SLineInfo, 0, sizeof(SLine));
    //1.1初始化SLine的起点终点（机械坐标）
    SLineInfo.SEndPoint = SLineInfo.SStartPoint =
        pSAssignConfig->SGCodeCurrSysState.SCurrPoint; //系统状态结构体的当前点
    //1.2获取SLine的终点（机械坐标）
    //获取当前的坐标编程方式
    bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
    //计算坐标点
    // 赋值重新扫描并求值，校验本次实际用于生成运动的输入。
    if (checkPolarCoordinateInput(&pSAssignConfig->SScanContainer, &pSAssignConfig->SGCodeCurrSysState) < 0)
    {
        return -1;
    }

    getMacCoordByScanInfo(pSAssignConfig->ucChannelNum, &pSAssignConfig->SScanContainer, &SLineInfo.SStartPoint,
                          &SLineInfo.SEndPoint, &pSAssignConfig->SGCodeCurrSysState);
    //1.3
    SLineInfo.EntityType = isSamePoint(&SLineInfo.SStartPoint, &SLineInfo.SEndPoint) ? DOT : OTHERTYPE;
    //1.2获取SLineInfo的指令值
    //SLineInfo.sCommand = G01 * GCOMMAND_DECIMAL_PLACE * 10;

    //主轴减速停时不知道位置
    // G16的C是平面极角，不请求物理C轴参与运动。
    if (pSAssignConfig->SScanContainer.bABC[2] && !pSAssignConfig->SGCodeCurrSysState.bPolarCoordinateProgramming)
    {
        setbit(pGCodeBufUnit->ucValStatus, AXIS_C);
    }

    //2.将SLine写入缓冲区
    writeLineIntoGCodeBuf(&SLineInfo, pGCodeBufUnit);

    return 2;
}
