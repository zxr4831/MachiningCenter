#include <stdlib.h>
#include <string.h>

#include "IOCommand.h"
#include "Gcode.h"

#include "ErrorFunction.h"

#if STM32_MODE
#include"dy_link.h"
#include "dir.h"
#include "app_num.h"
#include "menu_app.h"
#endif




/*************************************************
  函数名称：void initSProgram(SProgram* pSProgram)
  函数功能: 初始化程序结构体
  输入参数：*pSProgram   程序结构体地址
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 initProgramData(SProgram* pSProgram)
{
    if (pSProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }

    //1.释放程序结构体中开辟的内存
    freeProgramData(pSProgram);

    //2.程序结构体置0
    memset(pSProgram, 0, sizeof(SProgram));

    //3.1初始化剩余调用次数
    pSProgram->sRemainingCalls = UNASSIGNED_VALUE;

    //3.2初始化父程序返回行号
    pSProgram->unFatherLineNum = UNASSIGNED_VALUE;

    //3.3初始化宏状态
    pSProgram->cMacroStatus = GC_MODE_NORMAL;

    //3.4初始化起始结束行号
    pSProgram->usBeginLineNumMarco = UNASSIGNED_VALUE;
    pSProgram->sBeginLineNum = UNASSIGNED_VALUE;
    pSProgram->nEndLineNum = UNASSIGNED_VALUE;

    //3.5.1初始化宏结构体
    if(initMacroStruct(&pSProgram->SMacro) != 1)
        return -1;

    return 1;
}

/*************************************************
  函数名称：initMacroStruct(SMacro* pSMacro)
  函数功能: 初始化宏结构体
  输入参数：pSMacro   宏结构体地址
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 initMacroStruct(SMacro* pSMacro)
{
    if (pSMacro == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return -1;
    }

    //1.释放宏结构体中开辟的内存
    freeMacroData(pSMacro);

    //2.宏结构体置0
    memset(pSMacro, 0, sizeof(SMacro));

    //3.1初始化自定义宏链表
    pSMacro->pDefineList = initList(ORAM_NUM319, ORAM_NUM320);   //初始化自定义宏链表
    if (pSMacro->pDefineList == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }

    //3.2初始化检查局部宏变量结构体
    initLocalVar(&pSMacro->SLocalVarSyntax);
    initLocalVar(&pSMacro->SLocalVarAssign);

    //3.3初始化检查可执行栈
    pSMacro->pEnableStackSyntax = initList(ORAM_NUM304, ORAM_NUM305);
    if (pSMacro->pEnableStackSyntax == NULL)
    {
        freeList_2(&(pSMacro->pDefineList), freeDefineData);
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }

    //3.4初始化赋值可执行栈
    pSMacro->pEnableStackAssign = initList(ORAM_NUM307, ORAM_NUM308);   //初始化可执行栈 赋值
    if (pSMacro->pEnableStackAssign == NULL)
    {
        freeList_2(&(pSMacro->pDefineList), freeDefineData);
        freeList(&pSMacro->pEnableStackSyntax);
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }

    ///if (pChannelConfig->ucMainMacroTablePos == 0) //如果这是第一次出现宏程序
    //{
    //	pChannelConfig->ucMainMacroTablePos = getListSize(pChannelConfig->pProgramList);  //记录位置
    //}
    return 1;
}

/*************************************************
  函数名称：void initLocalVar(SLocalMacroVar* pSLocalMacroVar)
  函数功能: 初始化局部变量
  输入参数：pSMacro   宏结构体地址
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void initLocalVar(SLocalMacroVar* pSLocalMacroVar)
{
    memset(pSLocalMacroVar, 0, sizeof(SLocalMacroVar));
    pSLocalMacroVar->ulIsNull = 0xFFFFFFFFFFFFFFFF;    //64位全置1
}


/*************************************************
  函数名称：void freeProgramData(SProgram *pData)
  函数功能: 释放程序结构体中开辟的内存
  输入参数：pSProgram    程序结构体地址
  输出参数：
  返 回 值：

  备    注：仅释放程序结构体内部开辟的动态内存
**************************************************/
void freeProgramData(SProgram* pSProgram)
{
    //pSProgram允许为空
    if (pSProgram == NULL)    //不需要释放
    {
        return;
    }

    //释放宏结构体中开辟的内存
    freeMacroData(&pSProgram->SMacro);
}

/*************************************************
  函数名称：void freeProgramData(SProgram *pData)
  函数功能: 释放宏结构体中开辟的内存
  输入参数：pSMacro  宏结构体地址
  输出参数：
  返 回 值：
  备    注：宏结构体自身内存不释放
**************************************************/
void freeMacroData(SMacro* pSMacro)
{
    //pSMacro不允许为空
    if (pSMacro == NULL)    //不需要释放
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return;
    }

    //释放自定义宏链表
    freeList_2(&pSMacro->pDefineList, freeDefineData);

    //释放检查的可执行栈
    freeList(&pSMacro->pEnableStackSyntax);//罗祥玮241006：计算结构体中的pList的数据节点中未开辟内存，不需要用freeList_2()进行释放
    //freeList_2(&pSMacro->pEnableStackSyntax, freeExecutableData);

    //释放赋值的可执行栈
    freeList(&pSMacro->pEnableStackAssign);//罗祥玮241006：计算结构体中的pList的数据节点中未开辟内存，不需要用freeList_2()进行释放
    //freeList_2(&pSMacro->pEnableStackAssign, freeExecutableData);
}

#if STM32_MODE
/*************************************************
  函数名称：static char* getExecuteFileName()
  函数功能：STM32模式，获取要运行的文件名
  输入参数：
  输出参数：
  返 回 值：

  备    注：
**************************************************/
extern u8 page_num;
static char* getExecuteFileName(uint8 ucChannelNum)
{
    if (MDI_MODE == getRunMode())
    {
        if (APP_MDI_OT == page_num)
        {
            return MDI_name.Name;
        }
        else if (APP_ZUOBIAO == page_num || APP_COORDINATE == page_num)
        {
            return SI_name.Name;
        }
    }

    return Npro_name[ucChannelNum].Name;
}
#endif



/*************************************************
  函数名称：SList* getProgramList(uint8 ucChannelNum)
  函数功能: 获取某个通道的程序表
  输入参数：ucChannelNum通道号

  输出参数：
  返 回 值：程序表地址
  备    注：
**************************************************/
SList* getProgramList(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM301);
        return NULL;
    }
    return s_pSChannelConfig[ucChannelNum]->pProgramList;
}

/*************************************************
  函数名称：SProgram* getCurrentProgram(uint8 ucChannelNum)
  函数功能: 获取某个通道当前程序结构体
  输入参数：ucChannelNum通道号
  输出参数：
  返 回 值：程序结构体地址
  备    注：
**************************************************/
SProgram* getCurrentProgram(uint8 ucChannelNum)
{
    SProgram* pCurrProgram = NULL;   //当前程序结构体

    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM301);
        return NULL;
    }

    pCurrProgram = FindNodeData(s_pSChannelConfig[ucChannelNum]->pProgramList, 0);  //获取当前程序结构体
    if (pCurrProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return NULL;
    }
    return pCurrProgram;

}

/*************************************************
  函数名称：SList* initProgramList(uint8 ucChannelNum)
  函数功能：添加主程序节点
  输入参数：pSProgramList    程序链表
            cFileName        文件名
            ucChannelNum     通道号
  输出参数：
  返 回 值：
  备    注：1.仅runButtonAPI()中调用
            2.所有通道都要添加主程序???

            取消该函数内部对通道的循环
**************************************************/
int8 addMainProgramNode(uint8 ucChannelNum)
{

    SChannelConfiguration* pSChannelConfig = NULL;
    SList* pSProgramList = NULL;
    SProgram* pMainProgramNode = NULL;             //主程序结构体指针
    int8 cRet;

    //1.获取第i通道的通道结构体地址
    pSChannelConfig = getChannelConfig(ucChannelNum);

    //2.获取第i通道的程序链表地址
    pSProgramList = pSChannelConfig->pProgramList;
    if (pSProgramList ==NULL )
    {
         pSProgramList = initList(ORAM_NUM301, ORAM_NUM302);
         pSChannelConfig->pProgramList = pSProgramList;
        if (pSProgramList == NULL)
        {
            parseGCodeErrorB(0, GCODE_ERROR_NUM406);
            return -1;
        }
    }

    //3.1开辟主程序节点内存
    pMainProgramNode = mallocGCode(sizeof(SProgram), ORAM_NUM303);
    if (pMainProgramNode == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        //freeList(&pSProgramList);
        return -1;
    }
    memset(pMainProgramNode, 0, sizeof(SProgram));//对开辟内存进行初始化前，内存要置0

    //3.2初始化程序节点
    if(initProgramData(pMainProgramNode)!=1)
    {
        freeProgramData(pMainProgramNode);
        freeGCode(pMainProgramNode);
    	parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }
    //3.3将程序节点头插入链表
    cRet = insertList(pSProgramList, 0, pMainProgramNode, ORAM_NUM302);
    //插入链表失败，立即释放链表
    if (cRet == FALSE)
    {
        freeProgramData(pMainProgramNode);
        freeGCode(pMainProgramNode);
        //freeList(&pSProgramList);
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }

    //3.4写入程序节点的文件名
#if STM32_MODE
    strncpy(pMainProgramNode->cFileName, getExecuteFileName(ucChannelNum),FILENAME_LENGTH_LIMIT);
    pMainProgramNode->cFileName[FILENAME_LENGTH_LIMIT-1] =0;
    ///预留接口
#else
    if (MDI_MODE == getRunMode())
    {
        strcpy(pMainProgramNode->cFileName, "MDIMUTI");
    }
    else
    {
        if (ucChannelNum == 0)
        {
            g_cName_C0[19] = 0;   //防止strcpy越界
            strcpy(pMainProgramNode->cFileName, g_cName_C0);
        }
        else if (ucChannelNum == 1)
        {
            g_cName_C1[19] = 0;
            strcpy(pMainProgramNode->cFileName, g_cName_C1);
        }
    }
#endif
    return 1;

}



/*************************************************
  函数名称：SList* initProgramList(uint8 ucChannelNum)
  函数功能：添加程序节点
  输入参数：pSProgramList    程序链表
            cFileName        文件名
            ucChannelNum     通道号
  输出参数：
  返 回 值：
  备    注：1.在此不对行号信息，及标记进行赋值（在解析时才进行赋值）
**************************************************/
 int8 addProgramNode( int32 cFileNameVal, uint8 ucChannelNum)
 {
     SChannelConfiguration* pSChannelConfig = NULL;//G代码通道结构体指针
     SList* pSProgramList = NULL;                    //程序链表指针
    SProgram* pProgramNode = NULL;             //程序结构体指针
    int8 cTempArr[TEMP_FILENAME_MAX_LENGTH] = { 0 };   //文件名
    int8 cRet;

    //1.获取第i通道的通道结构体地址
    pSChannelConfig = getChannelConfig(ucChannelNum);

    //2.获取第i通道的程序链表地址
    pSProgramList = pSChannelConfig->pProgramList;
    if (pSProgramList == NULL)
    {
        pSProgramList = initList(ORAM_NUM301, ORAM_NUM302);
        pSChannelConfig->pProgramList = pSProgramList;
        if (pSProgramList == NULL)
        {
            parseGCodeErrorB(0, GCODE_ERROR_NUM406);
            return -1;
        }
    }

    //3.开辟程序节点内存
    pProgramNode = mallocGCode(sizeof(SProgram), ORAM_NUM303);
    if (pProgramNode == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }
    memset(pProgramNode, 0, sizeof(SProgram));//对开辟内存进行初始化前，内存要置0

    //4.初始化程序节点
    if (initProgramData(pProgramNode) != 1)
    {
        freeProgramData(pProgramNode);
        freeGCode(pProgramNode);
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }

    //5.将程序节点头插入链表
    cRet = insertList(pSProgramList, 0, pProgramNode, ORAM_NUM302);
    //若插入链表失败
    if (cRet == FALSE)
    {

        freeProgramData(pProgramNode);
        freeGCode(pProgramNode);
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return -1;
    }

    //6.将文件名的数值转为字符串
    convertProgramName(cFileNameVal, cTempArr);   //数值转字符串 获取文件名


    //7.写入程序节点的文件名
    strncpy(pProgramNode->cFileName, cTempArr,FILENAME_LENGTH_LIMIT);    //改变调用的主程序
    pProgramNode->cFileName[FILENAME_LENGTH_LIMIT-1] =0;
    return 1;
        
}

SProgram* createAProgramStruct()
{
    SProgram* pSProgram = NULL;

    pSProgram = mallocGCode(sizeof(SProgram), ORAM_NUM303);  //分配内存
    if (pSProgram == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM406);
        return pSProgram;
    }
    memset(pSProgram, 0, sizeof(SProgram));

    if(initProgramData(pSProgram) != 1)
    {
    	freeProgramData(pSProgram);
		freeGCode(pSProgram);
    	return NULL;
    }

    return pSProgram;
}

//获取当前程序结构体
SProgram* getCurrProgramStruct(uint8 ucChannelNum)
{
	if(ucChannelNum >= CHANNEL_NUM_MAX)
	return NULL;
    SChannelConfiguration* pSGCodeChannelConfig = getChannelConfig(ucChannelNum);
    if(pSGCodeChannelConfig == NULL)
    return NULL;
    SProgram* pCurrProgram = FindNodeData(pSGCodeChannelConfig->pProgramList, 0);
    if(pCurrProgram == NULL)
    return NULL;
    return pCurrProgram;
}


//当前程序是否为主程序
int8 isMainProgram(uint8 ucChannelNum)
{
    SList* pSProgramList = NULL;
    SProgram* pSCurrProgram = NULL;
    pSProgramList = getProgramList(ucChannelNum);

    //子程序
    if (pSProgramList->cSize>1)
    {
        return FALSE;
    }
    //主程序
    else
    {
        return TRUE;
    }
} 

/*************************************************
  函数名称：void convertProgramName(int32 nVal, int8 cArr[FILENAME_MAX_LENGTH])
  函数功能: 传入数值，返回文件名 例如输入1  返回"0001"的字符串
  输入参数：

  输出参数：
  返 回 值：
  备    注：
**************************************************/
void convertProgramName(int32 nVal, int8 cArr[TEMP_FILENAME_MAX_LENGTH])
{
    int8 cTempArray[TEMP_FILENAME_MAX_LENGTH] = { 0 };
    int8 cStrLen = 0;        //字符串长度
    int8 cIdx;
    int8 MaxIndex = 4 - 1;	//目前文件名最大为四个字符,因此最大数组下标为四减一
    itoaNC(nVal, cTempArray, 10);  //整型转字符串

    cStrLen = (int8)strlen(cTempArray) - 1;

    for (cIdx = MaxIndex; cStrLen >= 0 && cIdx >= 0; cStrLen--, cIdx--)	//将临时数组的字符串移给目标字符串	//修正部分文件名转换为字符串错误的BUG	//陈敏聪	2023/9/3
    {
        cArr[cIdx] = cTempArray[cStrLen];
    }
    cIdx = 0;	//索引复位
    cStrLen = (int8)strlen(cTempArray) - 1;	//最大索引复位
    while (cIdx < MaxIndex - cStrLen)	//目标字符串前面补零
    {
        cArr[cIdx++] = '0';
    }
#if STM32_MODE	//STM32模式下，字符串头加O,字符串尾加.TXT	//陈敏聪	2023/9/3
    cIdx = MaxIndex;
    while (cIdx >= 0)
    {
        cArr[cIdx + 1] = cArr[cIdx];
        cIdx--;
    }
    cArr[0] = 'O';
    cIdx = MaxIndex + 1;
    cArr[++cIdx] = '.';
    cArr[++cIdx] = 'T';
    cArr[++cIdx] = 'X';
    cArr[++cIdx] = 'T';
#endif
    cArr[TEMP_FILENAME_MAX_LENGTH - 1] = '\0';	//将末尾修正为'/0'结束符	//陈敏聪	2023/9/3
}
