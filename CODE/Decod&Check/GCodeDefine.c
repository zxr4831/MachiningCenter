#include<math.h>
#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include"Gcode.h"
#include"ErrorFunction.h"
#include "Config.h"
#if STM32_MODE
#include "flread_task.h"
#endif

/*************************************************
  函数名称：void freeDefineData(SUserDefineData *pData)
  函数功能: 释放自定义宏结构体中开辟的内存
  输入参数：pData	数据结构体地址
  输出参数：
  返 回 值：

  备    注：1.释放pName
			2.释放pReplaceStr
**************************************************/
void freeDefineData(SUserDefineData* pData)
{
	if (pData == NULL)
	{
		return;
	}

	freeGCode(pData->pName);
	pData->pName = NULL;

	freeGCode(pData->pReplaceStr);
	pData->pReplaceStr = NULL;

	//freeGCode(pData);
}

/*************************************************
  函数名称：void recordDefineMacro(SSyntaxConfiguration *SSyntaxConfig, uint8 ucChannelNum, SProgram *pCurrProgram)
  函数功能: 记录自定义宏
  输入参数：SSyntaxConfig		语法结构体
			pDefineList			自定义宏数据链表
  输出参数：
  返 回 值：1	记录完成
			-1	错误
  备    注：1.经过checkDefine函数后才能进入此函数，即进入此函数一定会记录define
			2.暂时认为一个程序段只由一个define组成
**************************************************/
int8 recordDefine(SGetTextStateInfo* pSGetTextStateInfo, SList* pDefineList)
{
	uint8 ucRet;
	SUserDefineData* pDefineData = NULL;
	uint16 usLen = 0;
	if (pSGetTextStateInfo == NULL || pDefineList == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	pDefineData = mallocGCode(sizeof(SUserDefineData), ORAM_NUM321);   //给宏定义分配内存
	if (pDefineData == NULL)
	{
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM406);
		return -1;
	}
	memset(pDefineData, 0, sizeof(SUserDefineData));

	pSGetTextStateInfo->pCurrPtr = pSGetTextStateInfo->cStr;
	//1.开始找DEFINE
	while (*pSGetTextStateInfo->pCurrPtr != 'D')
	{
		pSGetTextStateInfo->pCurrPtr++;
	}
	if (*(pSGetTextStateInfo->pCurrPtr + 1) == 'E' &&
		*(pSGetTextStateInfo->pCurrPtr + 2) == 'F' &&
		*(pSGetTextStateInfo->pCurrPtr + 3) == 'I' &&
		*(pSGetTextStateInfo->pCurrPtr + 4) == 'N' &&
		*(pSGetTextStateInfo->pCurrPtr + 5) == 'E')
	{
		pSGetTextStateInfo->pCurrPtr += 6;
	}
	else
	{
		//freeDefineData(pDefineData);
		freeGCode(pDefineData);
		//报错，(执行此函数前已确认程序段中含有DEFINE)
		return -1;
	}

	//DEFINE后必须为空格
	if (*pSGetTextStateInfo->pCurrPtr == ' ' || *pSGetTextStateInfo->pCurrPtr == '\t')
	{
		pSGetTextStateInfo->pCurrPtr += 1;
	}
	else
	{
		//freeDefineData(pDefineData);
		freeGCode(pDefineData);
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM443);
		return -1;
	}

	//2.1开始找宏名首字符
	while (*pSGetTextStateInfo->pCurrPtr == ' ' || *pSGetTextStateInfo->pCurrPtr == '\t')
	{
		pSGetTextStateInfo->pCurrPtr += 1;
	}
	//宏名首字符必须为字母
	if (('a' <= *pSGetTextStateInfo->pCurrPtr && *pSGetTextStateInfo->pCurrPtr <= 'z') ||
		('A' <= *pSGetTextStateInfo->pCurrPtr && *pSGetTextStateInfo->pCurrPtr <= 'Z'))
	{
		pSGetTextStateInfo->pStartPtr = pSGetTextStateInfo->pCurrPtr;
	}
	else
	{
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM443);
		return -1;
	}
	//2.2开始找宏名末字符
	while ((*pSGetTextStateInfo->pCurrPtr >= 'a' && *pSGetTextStateInfo->pCurrPtr <= 'z')
		|| (*pSGetTextStateInfo->pCurrPtr >= 'A' && *pSGetTextStateInfo->pCurrPtr <= 'Z')
		|| (*pSGetTextStateInfo->pCurrPtr >= '0' && *pSGetTextStateInfo->pCurrPtr <= '9'))  //宏名由字母和数字组成
	{
		pSGetTextStateInfo->pCurrPtr += 1;
	}
	//宏名后必须为空格
	if (*pSGetTextStateInfo->pCurrPtr == ' ' || *pSGetTextStateInfo->pCurrPtr == '\t')
	{
		pSGetTextStateInfo->pEndPtr = pSGetTextStateInfo->pCurrPtr;
	}
	else
	{
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM443);
		return -1;
	}
	//2.3开辟内存记录宏名
	usLen = (uint16)(pSGetTextStateInfo->pCurrPtr - pSGetTextStateInfo->pStartPtr);  //找到宏名结尾,计算宏名长度
	pDefineData->pName = mallocGCode((usLen + 1), ORAM_NUM322);   //给宏名分配内存,长度加一是为了加截止符
	if (pDefineData->pName == NULL)
	{
		freeDefineData(pDefineData);
		freeGCode(pDefineData);
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM406);
		return -1;
	}
	memset(pDefineData->pName, 0, (usLen + 1));
	memcpy(pDefineData->pName, pSGetTextStateInfo->pStartPtr, usLen);   //复制宏名
	pDefineData->pName[usLen] = '\0';    //加截止符
	

	//3.1开始找字符串首字符
	while (*pSGetTextStateInfo->pCurrPtr == ' ')
	{
		pSGetTextStateInfo->pCurrPtr += 1;
	}
	pSGetTextStateInfo->pStartPtr = pSGetTextStateInfo->pCurrPtr;

	//3.2开始找字符串末字符 1000\0
	while (pSGetTextStateInfo->pCurrPtr != ';' &&
		*pSGetTextStateInfo->pCurrPtr != '\r' &&
		*pSGetTextStateInfo->pCurrPtr != '\n' &&
		*pSGetTextStateInfo->pCurrPtr != '\0')
	{
		pSGetTextStateInfo->pCurrPtr += 1;
	}
	pSGetTextStateInfo->pEndPtr = pSGetTextStateInfo->pCurrPtr;

	//3.3开辟内存记录字符串
	usLen = (uint16)(pSGetTextStateInfo->pCurrPtr - pSGetTextStateInfo->pStartPtr);  //找到字符串结尾,计算字符串长度
	pDefineData->pReplaceStr = mallocGCode((usLen + 1), ORAM_NUM323);   //给字符串分配内存,长度加一是为了加截止符
	if (pDefineData->pReplaceStr == NULL)
	{
		freeDefineData(pDefineData);
		freeGCode(pDefineData);
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM406);
		return -1;
	}
	memset(pDefineData->pReplaceStr, 0, (usLen + 1));
	memcpy(pDefineData->pReplaceStr, pSGetTextStateInfo->pStartPtr, usLen);   //复制字符串
	pDefineData->pReplaceStr[usLen] = '\0';    //加截止符

	//4  数据插入链表头
	ucRet = insertList(pDefineList, 0, pDefineData, ORAM_NUM320);
	if (ucRet == FALSE)
	{
		freeDefineData(pDefineData);
		freeGCode(pDefineData);
		parseGCodeErrorB(pSGetTextStateInfo->unCurrLineNum, GCODE_ERROR_NUM406);
		return -1;
	}

	return 1;
}


static int8 safe_append(char *dst, uint16 dstMax, const char *src)
{
    uint16 dstLen = (uint16)strlen(dst);
    uint16 srcLen = (uint16)strlen(src);

    // 需要预留 '\0'，所以 dstLen + srcLen < dstMax 才安全
    if (dstLen + srcLen >= dstMax) {
        parseGCodeErrorB(0, GCODE_ERROR_NUM134); // 或换成更合适的“行过长/替换溢出”错误码
        return -1;
    }

    memcpy(dst + dstLen, src, srcLen + 1); // 连 '\0' 一起拷贝
    return 1;
}

/*************************************************
  函数名称：int8 replaceDefineInSegment(SGetTextStateInfo* pSGetTextStateInfo, uint8 cSize, SList* pDefineList)
  函数功能: 自定义宏的替换
  输入参数：pSGetTextStateInfo	获取文本状态信息结构体
			cSize				自定义宏链表
			pDefineList			行号（用于报错）

			ucChannelNum		通道号
			pCurrProgram		当前程序结构体
  输出参数：
  返 回 值：-1	错误
			0	程序段中宏名未替换完成
			1	程序段中宏名已替换完成
  备    注：
**************************************************/
int8 replaceDefineInSegment(SGetTextStateInfo* pSGetTextStateInfo, SList* pDefineList)
{
	if (pSGetTextStateInfo == NULL || pDefineList == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	//自定义宏链表无数据，不需要替换
	if (pDefineList->cSize <= 0)
	{
		return 1;
	}
	//自定义宏链表有数据，需要找到对应替换的字符串并完成替换
	else
	{    
		uint8 ucCnt = 0;
		uint16 cIndex = 0;
		int8 cString[MAX_LENGTH_OF_LINE] = { 0 };				//存放原始连续字符串
		int8 cTargetString[MAX_LENGTH_OF_LINE] = { 0 };			//存放替换后的字符串
		int8 cFinalSegmentString[MAX_LENGTH_OF_LINE] = { 0 };	//存放替换完成的程序段字符串
		int8 *pointstart= pSGetTextStateInfo->pCurrPtr;
		//限制替换次数
		while (1)
		{	
			cIndex = 0;
			memset(cString, 0, MAX_LENGTH_OF_LINE);
			memset(cTargetString, 0, MAX_LENGTH_OF_LINE);

			//1.判断程序段是否替换完成
			if (isSegmentFinishSign(pSGetTextStateInfo->pCurrPtr) ||
				isLineFinishSign(pSGetTextStateInfo->pCurrPtr))
			{
				memset(pSGetTextStateInfo->cStr, 0, MAX_LENGTH_OF_LINE);
				strcpy(pSGetTextStateInfo->cStr, cFinalSegmentString);

				return 1;
			}

			//2.找到连续字符串首字符
			while (*pSGetTextStateInfo->pCurrPtr == ' ')
			{
				pSGetTextStateInfo->pCurrPtr++;
			}
			//1.判断程序段是否替换完成
			if (isSegmentFinishSign(pSGetTextStateInfo->pCurrPtr) ||
				isLineFinishSign(pSGetTextStateInfo->pCurrPtr))
			{
				memset(pSGetTextStateInfo->cStr, 0, MAX_LENGTH_OF_LINE);
				strcpy(pSGetTextStateInfo->cStr, cFinalSegmentString);

				return 1;
			}

			if((uint32)pSGetTextStateInfo->pCurrPtr -(uint32)pointstart >=MAX_LENGTH_OF_LINE - 1)
			{
				memset(pSGetTextStateInfo->cStr, 0, MAX_LENGTH_OF_LINE);
				strcpy(pSGetTextStateInfo->cStr,cFinalSegmentString);
				return 1;
			}
			pSGetTextStateInfo->pStartPtr  = pSGetTextStateInfo->pCurrPtr;
			cString[cIndex] = *pSGetTextStateInfo->pCurrPtr;
			cIndex++;
			pSGetTextStateInfo->pCurrPtr++;

			//3.找到连续字符串末字符
			while (*pSGetTextStateInfo->pCurrPtr != ' ' &&
				!isSegmentFinishSign(pSGetTextStateInfo->pCurrPtr) &&
				!isLineFinishSign(pSGetTextStateInfo->pCurrPtr))
			{
				cString[cIndex] = *pSGetTextStateInfo->pCurrPtr;
				cIndex++;
				pSGetTextStateInfo->pCurrPtr++;
			}
			pSGetTextStateInfo->pEndPtr = pSGetTextStateInfo->pCurrPtr;

			//4.判断连续字符串首字符是字母
			if (!(('A' <= *pSGetTextStateInfo->pStartPtr && *pSGetTextStateInfo->pStartPtr <= 'Z')
				|| ('a' <= *pSGetTextStateInfo->pStartPtr && *pSGetTextStateInfo->pStartPtr <= 'z')))
			{
			   if (safe_append((char*)cFinalSegmentString, MAX_LENGTH_OF_LINE, (const char*)cString) < 0) {
					return -1;
				}
				continue;
			}

			//5.查询连续字符串是否为宏名
			if (!getTargetStringByMacroName(cString, cIndex, cTargetString, pDefineList))
			{
			   if (safe_append((char*)cFinalSegmentString, MAX_LENGTH_OF_LINE, (const char*)cString) < 0) {
					return -1;
				}
				continue;
			}

			//6.对宏名替换为目标字符串
            if (safe_append((char*)cFinalSegmentString, MAX_LENGTH_OF_LINE, (const char*)cTargetString) < 0) {
                return -1;
            }

		}

	}
}

/*************************************************
  函数名称：int8 getTargetStringByMacroName(int8* pMacroName, int8 cNameSize, int8* pTargetString, SList* pDefineList)
  函数功能: 在自定义宏链表中找目标宏名对应替换的字符串
  输入参数：pMacroName		宏名字符串首字符
			cNameSize		宏名字符串长度
			pTargetString	目标替换的字符串内存地址（输出）
			pDefineList		自定义宏链表
  输出参数：
  返 回 值：-1	错误
			0	输入字符串非宏名
			1	找到宏名对应的目标字符串
  备    注：
**************************************************/
int8 getTargetStringByMacroName(int8* pMacroName, uint16 cNameSize, int8* pTargetString, SList* pDefineList)
{
	if (pMacroName == NULL || pDefineList == NULL)
	{
		//报错
		return -1;
	
	}
	int16 cCount = 0;
	//遍历SDefineList
	while (cCount < pDefineList->cSize)
	{
		SUserDefineData* pData;
		pData = (SUserDefineData*)FindNodeData(pDefineList, cCount);
        if(FindNodeData == NULL)
        {
        	return -1;
        }
		//1.长度比对成功
		if (cNameSize == strlen(pData->pName))
		{
			//2.逐字比对成功
			if (isTagetStr_B(pData->pName, pMacroName))
			{
				strcpy(pTargetString, pData->pReplaceStr);
				return 1;
			}
		}

		cCount++;

	}
	return 0;

}


static void printDefineData(void* data)
{
	SUserDefineData* a = (SUserDefineData*)data;
	printf("宏名:%s\t字符串:%s\n", a->pName, a->pReplaceStr);
}
