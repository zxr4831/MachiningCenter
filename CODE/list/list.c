#include<stdio.h>
#include"list.h"

//#include"Gcode.h"
#include "common.h"
#include "gcode.h"


/*************************************************
  函数名称：SList *initList(void)
  函数功能: 初始化链表
  输入参数：
  输出参数：
  返 回 值：1.如果链表开辟成功，则返回链表地址
            2.如果链表开辟失败（内存不足以开辟链表或链表头节点），则返回NULL

  备    注：
**************************************************/
SList* initList(int16 list_ORAM_NUM, int16 node_ORAM_NUM)
{
	//开辟链表结构体内存
	SList* pList = (SList*)mallocGCode(sizeof(SList), list_ORAM_NUM);
	//如果内存不足以开辟链表结构体
	if (pList == NULL)
	{
		//返回NULL
		return NULL;
	}
	//内存足以开辟链表结构体情况下：
	//开辟链表头节点
	pList->pHead = (SNode*)mallocGCode(sizeof(SNode), node_ORAM_NUM);
	//如果内存不足以开辟头节点结构体
	if (pList->pHead == NULL)
	{
		//先释放上面开辟的链表结构体
		freeGCode(pList);
		//再返回NULL
		return NULL;
	}
	//将头节点数据域指针置为NULL（头节点数据域不存放数据）
	pList->pHead->pData = NULL;
	//将头节点中的下一个节点指针置为NULL
	pList->pHead->pNextNode = NULL;
	//链表大小置为0（头节点不存放数据，故不算进链表大小里面）
	pList->cSize = 0;

	return pList;
}

/*************************************************
  函数名称：bool insertList(SList *pList, int8 cPos, void *pData)
  函数功能: 新建节点并将传入的数据赋给新节点，再根据传入的位置将新节点插入指定的链表  
  输入参数：pList链表地址，pos插入的位置,  *pData数据域内存地址
  输出参数：输出新链表地址

  返 回 值：插入成功，返回值为TRUE
            插入失败，返回值为FALSE

  备    注：插入过程中，第一个节点的位置是0,size的大小不包含头结点，最后一个节点的位置为（链表大小-1）
**************************************************/
bool insertList(SList* pList, int16 cPos, void* pData, int16 node_ORAM_NUM)
{
	int16 i;   //辅助
	SNode* pNewNode = NULL;  //新节点
	SNode* pCurr = NULL;     //当前节点为插入新节点的前一个节点

	if (pList == NULL || pData == NULL)
	{
		return FALSE;
	}
	// 新增：检查链表大小是否已达最大值
	if (pList->cSize >= INT16_MAX)
	{
		return FALSE;  // 链表已满，无法继续插入
	}
	if (cPos < 0 || cPos > pList->cSize)
	{
		//尾插
		cPos = pList->cSize;
	}


	pCurr = pList->pHead;//指向头结点

	for (i = 0; i < cPos; i++)  //找插入位置
	{
		pCurr = pCurr->pNextNode;
	}
	//新建节点
	pNewNode = (SNode*)mallocGCode(sizeof(SNode), node_ORAM_NUM);

	if (pNewNode == NULL)
	{
		return FALSE;
	}

	//将传入的数据赋给新节点的数据域
	pNewNode->pData = pData;
	//插入新节点
	pNewNode->pNextNode = pCurr->pNextNode;

	pCurr->pNextNode = pNewNode;
	//链表大小加1
	pList->cSize++;

	return TRUE;
}



/*************************************************
  函数名称：bool insertNode(SList *pList, int8 cPos,SNode *pNode)
  函数功能: 将传入的节点根据传入的位置插入链表
  输入参数：pList链表，pos位置,pNode节点
  输出参数：
  返 回 值：插入成功，返回值为TRUE
            插入失败，返回值为FALSE

  备    注：第一个节点的位置是0,size的大小不包含头结点   SNode* pNode为已有的节点内存地址
**************************************************/
bool insertNode(SList* pList, int16 cPos, SNode* pNode)
{
	int16 i;   //辅助

	SNode* pCurr;     //当前节点 用于寻找位置以便插入

	if (pList == NULL || pNode == NULL)
	{
		return FALSE;
	}

	if (cPos < 0 || cPos > pList->cSize)
	{
		//尾插
		cPos = pList->cSize;
	}

	//指向头节点
	pCurr = pList->pHead;

	for (i = 0; i < cPos; i++)  //找插入位置
	{
		pCurr = pCurr->pNextNode;
	}
	//插入节点
	pNode->pNextNode = pCurr->pNextNode;

	pCurr->pNextNode = pNode;

	pList->cSize++;

	return TRUE;
}

//查找
//bool findList(SList *pList, void *pData)
//{
//	SNode *pCurr;     //当前节点 
//
//	if (pList == NULL || pData == NULL)
//	{
//		return FALSE;
//	}
//
//	pCurr = pList->pHead->pNextNode;
//	while (pCurr->pData != pData)
//	{
//		pCurr = pCurr->pNextNode;
//		if (pCurr == NULL)
//		{
//			return FALSE;
//		}
//	}
//	return TRUE;
//
//	return TRUE;
//}



/*************************************************
  函数名称：void *FindNodeData(SList *pList, int pos)
  函数功能:查找并返回某个节点的数据域地址
  输入参数：pList链表，pos位置
  输出参数：
  返 回 值：

  备    注：pos位置从0开始
**************************************************/
void* FindNodeData(SList* pList, int pos)
{
	SNode* pCurr;     //当前节点 
	void* pData = NULL;
	int i = 0;
	if (pList == NULL)
	{
		return NULL;
	}

	if (pos < 0 || pos >= pList->cSize)
	{
		return NULL;
	}

	pCurr = pList->pHead->pNextNode;//指向第一个数据节点
	//寻找对应位置的节点
	while (i < pos)
	{
		pCurr = pCurr->pNextNode;
		if (pCurr == NULL)
		{
			return NULL;
		}
		i++;
	}
	//获取节点数据域地址
	pData = pCurr->pData;
	//返回节点数据域地址
	return pData;
}


int16 findNodePosByData(SList* pList, int16 cTargetVal)
{
	int16 cListSize = getListSize(pList);
	for (int i = 0; i < cListSize; i++)
	{
		if (cTargetVal == *(int16*)FindNodeData(pList, i))
		{
			return i;
		}
	}
	return -1;
}

/*************************************************
  函数名称：SNode *FindNode(SList *pList, int pos)
  函数功能:查找并返回某个节点地址
  输入参数：pList链表，pos位置
  输出参数：
  返 回 值：

  备    注：pos位置从0开始
**************************************************/
SNode* FindNode(SList* pList, int pos)
{
	SNode* pCurr;     //当前节点 
	int i = 0;
	if (pList == NULL)
	{
		return NULL;
	}

	if (pos < 0 || pos >= pList->cSize)
	{
		return NULL;
	}
	//指向第一个数据节点
	pCurr = pList->pHead->pNextNode;
	//寻找对应位置的节点
	while (i < pos)
	{
		pCurr = pCurr->pNextNode;
		if (pCurr == NULL)
		{
			return NULL;
		}
		i++;
	}
	//返回节点地址
	return pCurr;
}





/*************************************************
  函数名称：int8 getListSize(SList *pList)
  函数功能: 返回链表大小（链表大小不包括头节点）
  输入参数：pList链表
  输出参数：
  返 回 值：

  备    注：
**************************************************/
int16 getListSize(SList* pList)
{
	if (pList == NULL)
	{
		return -1;
	}
	return pList->cSize;
}



/*************************************************
  函数名称：void freeList(SList **pList)
  函数功能: 释放单层链表
  输入参数：*pList链表
  输出参数：
  返 回 值：

  备    注：释放单层链表的顺序：1.从链表头节点向后逐个释放   2.全部节点释放完毕后，释放链表结构体
            每个节点的释放步骤：1.先释放节点中的数据域（单层链表中，节点数据域指针指向的内存中没有再开辟内存）
			                    2.再释放节点本身
**************************************************/
void freeList(SList** pList)
{
	SNode* pCurr;  //当前节点

	if (pList == NULL || *pList == NULL)
	{
		return;
	}

	//指向链表头节点
	pCurr = (*pList)->pHead;

	while (pCurr != NULL)
	{
		SNode* pNext = pCurr->pNextNode;  //获取下一个节点的地址

		freeGCode(pCurr->pData);   //释放当前节点的数据域
		pCurr->pData = NULL;//当前节点数据域指针置为NULL

		freeGCode(pCurr);   //释放当前节点
		pCurr = NULL;

		pCurr = pNext;//当前指针指向下一个节点的地址
	}

	(*pList)->cSize = 0;//链表大小置为0

	freeGCode((*pList));//释放链表结构体

	(*pList) = NULL;
}


/*************************************************
  函数名称：void freeList_2(SList** pList, FreeData customFree)
  函数功能: 释放嵌套链表
  输入参数：**pList程序表,   customFree自定义的函数指针，该函数定义节点数据域内部开辟的内存的释放方法
  输出参数：
  返 回 值：

  备    注：嵌套链表中的每个节点的数据指针指向的内存中还开辟了内存
            嵌套链表释放顺序：1.从头节点向后逐个释放  2.释放完节点后释放链表结构体
			每个节点的释放步骤：1.先释放节点数据域内部开辟的内存，即节点数据域指针指向的内存中还有指针，且该指针指向的一块内存
			                    2.释放节点数据域指针指向的内存
								3.释放节点本身
**************************************************/
void freeList_2(SList** ppList, FreeData customFree)
{
	SNode* pCurr = NULL;
	if ((*ppList) == NULL || ppList == NULL)  //不需要释放
	{
		return;
	}
	pCurr = (*ppList)->pHead;//指向头节点

	while (pCurr != NULL)
	{
		SNode* pNext = pCurr->pNextNode;

		customFree(pCurr->pData);  //罗祥玮241006：仅释放结构体中开辟的内存，结构体自身开辟的内存不释放（释放节点数据域内部开辟的内存，即节点数据域指针指向的内存中还有指针，且该指针指向了一块内存）
		freeGCode(pCurr->pData);   //罗祥玮241006：释放结构体自身开辟内存（释放节点开辟的数据域内存，即节点数据域指针指向的内存）
		pCurr->pData = NULL;//释放完节点数据域内部开辟的内存以及节点数据域内存后，将节点数据域指针置为NULL

		freeGCode(pCurr);    //释放完数据域后，释放当前节点
		pCurr = NULL;//当前指针置为NULL
		pCurr = pNext;//当前指针地址改为下一个节点的地址
	}

	(*ppList)->cSize = 0;//链表大小置为0

	freeGCode((*ppList));//释放开辟的链表结构体
	(*ppList) = NULL;//链表指针置为NULL


}

/*************************************************
  函数名称：bool removeNode(SList *pList, int8 cPos,bool bSaveNodeFlag)
  函数功能: 删除单层链表中的节点
  输入参数：pList链表，pos位置，bSaveNodeFlag标志位
  输出参数：
  返 回 值：

  备    注：第一个数据节点的位置是0，最后一个数据节点的位置是（链表大小-1）
			注意：bSaveNodeFlag ：TRUE 不释放节点内存，只是将节点从链表中解除    FALSE 释放内存，不仅将节点从链表中解除，还释放节点内存

**************************************************/
bool removeNode(SList* pList, int16 cPos, bool bSaveNodeFlag)
{
	int16 i;   //辅助
	SNode* pCurr;     //当前节点 
	SNode* pDel;  //要删除的节点

	if (pList == NULL)
	{
		return FALSE;
	}

	if (cPos < 0 || cPos >= pList->cSize)
	{
		return FALSE;
	}

	//指向头节点
	pCurr = pList->pHead;

	for (i = 0; i < cPos; i++)  //查找被删除节点的前一个节点
	{
		pCurr = pCurr->pNextNode;
	}

	pDel = pCurr->pNextNode;//要删除的节点

	pCurr->pNextNode = pDel->pNextNode;

	//释放节点内存
	if (bSaveNodeFlag == FALSE)
	{
		//释放节点数据域
		if (pDel->pData)
		{
			freeGCode(pDel->pData); ///释放的pData不能是有指针变量是malloc出来的

			pDel->pData = NULL;
		}
		//释放节点
		if (pDel)
		{
			pDel->pNextNode = NULL;

			freeGCode(pDel);

			pDel = NULL;
		}

	}
	//不释放节点内存，只是将节点从链表中解除
	else
	{
		pDel->pNextNode = NULL;   //删除指向
	}


	pList->cSize--;

	return TRUE;
}


/*************************************************
  函数名称：bool removeNode_2(SList *pList, int8 cPos,bool bFlag)
  函数功能: 删除嵌套链表中的节点
  输入参数：pList链表，pos位置，bSaveNodeFlag标志位，customFree函数指针，该函数定义节点数据域内部开辟的内存的释放方法
  输出参数：
  返 回 值：

  备    注：第一个节点的位置是0
			bSaveNodeFlag：TRUE 不释放内存 FALSE 释放内存
**************************************************/
bool removeNode_2(SList* pList, int16 cPos, bool bSaveNodeFlag, FreeData customFree)
{
	int16 i;   //辅助
	SNode* pCurr;     //当前节点 
	SNode* pDel;  //要删除的节点

	if (pList == NULL)
	{
		return FALSE;
	}

	if (cPos < 0 || cPos >= pList->cSize)
	{
		return FALSE;
	}

	//指向头节点
	pCurr = pList->pHead;

	for (i = 0; i < cPos; i++)  //查找删除节点的前一个节点
	{
		pCurr = pCurr->pNextNode;
	}
	//获取要删除的节点
	pDel = pCurr->pNextNode;

	pCurr->pNextNode = pDel->pNextNode;

	//释放节点内存
	if (bSaveNodeFlag == FALSE)
	{
		//节点数据域不为空
		if (pDel->pData!=NULL)
		{
			//先释放数据域内部开辟的内存
			customFree(pDel->pData);
			//再释放数据域内存
			freeGCode(pDel->pData);
			//节点数据域指针置为NULL
			pDel->pData = NULL;
		}
		//释放节点本身内存
		if (pDel)
		{
			pDel->pNextNode = NULL;

			freeGCode(pDel);

			pDel = NULL;
		}

	}
	//不释放节点内存，只是将节点从链表中解除
	else
	{
		pDel->pNextNode = NULL;   //删除指向
	}


	pList->cSize--;

	return TRUE;
}




#if !STM32_MODE


//打印链表
void printList(SList* pList, PrintNode print)
{
	SNode* pCurr;  //当前节点
	int8 cCnt = 0;
	if (pList == NULL)
	{
		return;
	}
	pCurr = pList->pHead->pNextNode;

	while (pCurr != NULL)
	{

		print(pCurr->pData);
		cCnt += 1;
#if ENABLE_DEBUG_ASSIGN
		if (cCnt % 5 == 0)
		{
			printf("---------------------------------------------\n");
		}
#endif // ENABLE_DEBUG_ASSIGN


		pCurr = pCurr->pNextNode;
	}

#if ENABLE_DEBUG_ASSIGN
	if (pCurr == NULL)
	{
		printf("\n");
	}
#endif // ENABLE_DEBUG_ASSIGN

}





#endif

