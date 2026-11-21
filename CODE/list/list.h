#ifndef __LIST_H
#define __LIST_H

#include "common.h"

/*typedef的功能是定义新的类型。第一句就是定义了一种PrintNode的类型，并定义这种类型为指向某种函数的指针，这种函数以一个void*为参数并返回void类型。*/
typedef void(*PrintNode)(void*);
typedef void(*FreeData)(void*);

typedef struct _SNode  //链表节点
{
	void* pData;               //指向任意数据类型
	struct _SNode* pNextNode;  //下一个节点
}SNode;

typedef struct _SList   //链表
{
	SNode* pHead;    //头结点
	int16 cSize;     //链表长度

}SList;
#define INT16_MAX 32767


/*typedef的功能是定义新的类型。第一句就是定义了一种PrintNode的类型，并定义这种类型为指向某种函数的指针，这种函数以一个void*为参数并返回void类型。*/
typedef void(*PrintNode)(void*);
typedef void(*FreeData)(void*);



//初始化链表
SList* initList(int16 list_ORAM_NUM, int16 node_ORAM_NUM);

//生成节点，插入数据
bool insertList(SList* pList, int16 cPos, void* SData, int16 node_ORAM_NUM);

//插入节点
bool insertNode(SList* pList, int16 cPos, SNode* pNode);

//找到链表中的节点
void* FindNodeData(SList* pList, int pos);

//
int16 findNodePosByData(SList* pList, int16 cTargetVal);

SNode* FindNode(SList* pList, int pos);

//删除节点
bool removeNode(SList* pList, int16 cPos, bool bSaveNodeFlag);

//返回链表大小
int16 getListSize(SList* pList);

//释放链表
void freeList(SList** pList);

void freeList_2(SList** ppList, FreeData customFree);

//删除链表元素
bool removeNode_2(SList* pList, int16 cPos, bool bSaveNodeFlag, FreeData customFree);

//释放链表 
void freeList_2(SList** pList, FreeData customFree);

//打印链表
void printList(SList* pList, PrintNode print);

void printData(void* data);




#endif