#include "common.h"
#include "manageMem.h"
#include <stdlib.h>

#if MEM_TEST
int MEM_CNT = 0;
#endif // MEM_TEST

/*************************************************
  函数名称：void* mallocGCode(uint64 ulSize)
  函数功能: 根据不同环境，使用不同的内存分配函数
  输入参数：ulSize需要开辟的字节数
  输出参数：
  返 回 值：申请成功返回地址
			申请失败返回NULL
  备    注：
**************************************************/
void* mallocGCode(uint64 ulSize, uint16 ORAM_NUM)
{
	void* pRet = NULL;
#if STM32_MODE
	pRet = apply_ram(ulSize, ORAM_NUM);
#else
	pRet = malloc(ulSize);
#endif
	if (pRet == NULL)
	{
		return NULL;
	}
	else
	{
#if MEM_TEST
		MEM_CNT += 1;
#endif // MEM_TEST
		return pRet;
	}
}

/*************************************************
  函数名称：void freeGcode(void *pFreeTarget)
  函数功能: 根据不同环境，使用不同的内存释放函数
  输入参数：pFreeTarget目标内存地址
  输出参数：
  返 回 值：
  备    注：函数内部会检查是否为空，使用时不用再提前判断
**************************************************/
void freeGCode(void* pFreeTarget)
{
	if (pFreeTarget == NULL)
	{
		return;
	}
#if STM32_MODE
	free_ram(pFreeTarget);
#else
	free(pFreeTarget);
#endif

#if MEM_TEST
	MEM_CNT -= 1;
#endif // MEM_TEST

}
