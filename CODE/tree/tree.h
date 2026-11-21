#ifndef _TREE_H
#define _TREE_H

#include "common.h"

typedef struct TreeNodeData// 树节点数据结构体
{
	int nIDentifier;//标识符
	void* pData;//数据域指针
}TreeNodeData;

typedef struct TreeNode//树节点
{
	TreeNodeData STreeNodeData;//树节点数据结构体指针
	int8 height;//树高
	struct  TreeNode* left;//左子树
	struct  TreeNode* right;//右子树
}TreeNode;

typedef struct tree//树结构体
{
	TreeNode* Root;//节点指针
	int cize;//大小
}Tree;

Tree* creatTree(int16 tree_ORAM_NUM);
TreeNode* mallocTreeNode(void* data, double dIDentifier, int16 node_ORAM_NUM);
int8 getTreeNodeHeight(TreeNode* node);
TreeNode* leftRotate(TreeNode* root);
TreeNode* rightRotate(TreeNode* root);
int8 getBalance(TreeNode* root);
TreeNode* insertTreeNode(TreeNode* root, void* date, int32 nIDentifier, int8* cErrorSign, int16 node_ORAM_NUM);
void* findTreeNodeData(TreeNode* root, int32 dGCodeNum);


#endif