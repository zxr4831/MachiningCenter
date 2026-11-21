#include <stdio.h>
#include "tree.h"
#include "ErrorFunction.h"

#include "GCode.h"
#include "GCodeRegulation.h"
#include "GCodeCommon.h"

#if STM32_MODE
#include "dy_link.h"
#endif
#include "Gcode.h"

/*************************************************
  函数名称：int8 addGCommadRegulation(double dCommandVal, int8 cGroup, int8 cSegmentNum,
				MallocParaStruct pMallocParaStructAPI, FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI,
				CheckCommand pCheckCommandAPI, AssignCommand pAssignCommandAPI)

  函数功能: 添加G代码指令字及其规则到相应的规则结构体数组位置
  输入参数：dCommandVal				指令值
			cGroup					指令值所属组别
			cSegmentNum				指令字占用程序段数
			pMallocParaStructAPI	开辟参数结构体内存API
			pFreeParaStructAPI		释放参数结构体内存API
			pGetParaAPI				获取指令字参数API
			pCheckCommandAPI		检查指令字API
			pAssignCommandAPI		赋值指令字API
  输出参数：
  返 回 值：-1失败  1成功
  备    注：
**************************************************/
int8 addGCommadRegulation(double dCommandVal, int8 cGroup, int8 cSegmentNum, MallocParaStruct pMallocParaStructAPI,
                          FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI, CheckCommand pCheckCommandAPI,
                          AssignCommand pAssignCommandAPI)
{
    int8 cErrorSign = 0;

    SGCommandRegulation* pSGCommandRegulation; //规则结构体指针

    pSGCommandRegulation = mallocGCode(sizeof(SGCommandRegulation), ORAM_NUM326); //开辟规则结构体
    if (pSGCommandRegulation == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }
    int a = sizeof(SGCommandRegulation);
    memset(pSGCommandRegulation, 0, sizeof(SGCommandRegulation));

    pSGCommandRegulation->nCommandVal =
        round(dCommandVal * pow(10, GCOMMAND_DECIMAL_PLACE)); //写入当前规则结构体的指令字值
    pSGCommandRegulation->cGroup = cGroup;                    //规定该指令字的组别
    pSGCommandRegulation->cSegmentNum = cSegmentNum;          //规定该指令字的程序段数

    pSGCommandRegulation->pMallocParaStructAPI = pMallocParaStructAPI; //规定该指令字的参数结构体内存开辟
    pSGCommandRegulation->pFreeParaStructAPI = pFreeParaStructAPI;     //规定该指令字的参数结构体内存释放
    pSGCommandRegulation->pGetParaAPI = pGetParaAPI;                   //规定该指令字的参数获取
    pSGCommandRegulation->pCheckCommandAPI = pCheckCommandAPI;         //规定该指令字的检查
    pSGCommandRegulation->pAssignCommandAPI = pAssignCommandAPI;       //规定该指令字的赋值

    //将指令对应的规则结构体插入G规则树中
    g_pSGCommandRegulationTree->Root =
        insertTreeNode(g_pSGCommandRegulationTree->Root, pSGCommandRegulation, pSGCommandRegulation->nCommandVal,
                       &cErrorSign, ORAM_NUM325); //将规则添加在树中
    if (cErrorSign == 1 || g_pSGCommandRegulationTree->Root == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }
    //插入完成
    g_pSGCommandRegulationTree->cize++;
    return 1;
}

/*************************************************
  函数名称：int8 addMCommadRegulation(double dCommandVal, int8 cGroup, int8 cSegmentNum,
				MallocParaStruct pMallocParaStructAPI, FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI,
				CheckCommand pCheckCommandAPI, AssignCommand pAssignCommandAPI)

  函数功能: 添加M代码指令字及其规则到相应的规则结构体数组位置
  输入参数：dCommandVal				指令值
			cGroup					指令值所属组别
			cSegmentNum				指令字占用程序段数
			pMallocParaStructAPI	开辟参数结构体内存API
			pFreeParaStructAPI		释放参数结构体内存API
			pGetParaAPI				获取指令字参数API
			pCheckCommandAPI		检查指令字API
			pAssignCommandAPI		赋值指令字API
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 addMCommadRegulation(double dCommandVal, int8 cType, int8 cGroup, int8 cSegmentNum,
                          MallocParaStruct pMallocParaStructAPI, FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI,
                          CheckCommand pCheckCommandAPI, AssignCommand pAssignCommandAPI)
{
    int8 cErrorSign = 0;

    SMCommandRegulation* pSMCommandRegulation;                                    //M码规则结构体指针
    pSMCommandRegulation = mallocGCode(sizeof(SMCommandRegulation), ORAM_NUM329); //开辟规则结构体
    if (pSMCommandRegulation == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }

    pSMCommandRegulation->nCommandVal =
        (int32)dCommandVal * pow(10, MCOMMAND_DECIMAL_PLACE); //写入当前规则结构体的指令字值
    pSMCommandRegulation->cType = cType;                      //规定该指令的前后作用类型
    pSMCommandRegulation->cGroup = cGroup;                    //规定该指令字的组别
    pSMCommandRegulation->cSegmentNum = cSegmentNum;          //规定该指令字的程序段数

    pSMCommandRegulation->pMallocParaStructAPI = pMallocParaStructAPI; //规定该指令字的参数结构体内存开辟
    pSMCommandRegulation->pFreeParaStructAPI = pFreeParaStructAPI;     //规定该指令字的参数结构体内存释放
    pSMCommandRegulation->pGetParaAPI = pGetParaAPI;                   //规定该指令字的参数获取
    pSMCommandRegulation->pCheckCommandAPI = pCheckCommandAPI;         //规定该指令字的检查
    pSMCommandRegulation->pAssignCommandAPI = pAssignCommandAPI;       //规定该指令字的赋值

    //将指令对应的规则结构体插入M规则树中
    g_pSMCommandRegulationTree->Root =
        insertTreeNode(g_pSMCommandRegulationTree->Root, pSMCommandRegulation, pSMCommandRegulation->nCommandVal,
                       &cErrorSign, ORAM_NUM328); //将规则添加在树中
    if (cErrorSign == 1 || g_pSMCommandRegulationTree->Root == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }
    //插入完成
    g_pSMCommandRegulationTree->cize++;
    return 1;
}

/*************************************************
  函数名称：int8 addMCommadRegulation(double dCommandVal, int8 cGroup, int8 cSegmentNum,
				MallocParaStruct pMallocParaStructAPI, FreeParaStruct pFreeParaStructAPI, GetPara pGetParaAPI,
				CheckCommand pCheckCommandAPI, AssignCommand pAssignCommandAPI)

  函数功能: 添加M代码指令字及其规则到相应的规则结构体数组位置
  输入参数：dCommandVal				指令值
			cGroup					指令值所属组别
			cSegmentNum				指令字占用程序段数
			pMallocParaStructAPI	开辟参数结构体内存API
			pFreeParaStructAPI		释放参数结构体内存API
			pGetParaAPI				获取指令字参数API
			pCheckCommandAPI		检查指令字API
			pAssignCommandAPI		赋值指令字API
  输出参数：
  返 回 值：
  备    注：释放所有指令的参数结构体
**************************************************/
void freeAllParaStruct()
{
    //遍历规则结构体数，若规则结构体中释放函数不为NULL，则调用释放参数结构体函数
}
