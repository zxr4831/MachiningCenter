#include <stdio.h>
#include <stdlib.h>

#include "common.h"

#include "list.h"

#include "Gcode.h"

#include "tree.h"

#include "GeneralFunctions.h"

/*************************************************
  函数名称：Tree * creatTree()
  函数功能: 创建树
  输入参数：
  输出参数：
  返 回 值：

  备    注：
**************************************************/
Tree* creatTree(int16 tree_ORAM_NUM)
{
    Tree* tree;
    tree = (Tree*)mallocGCode(sizeof(Tree), tree_ORAM_NUM);
    if (tree == NULL)
    {
        return NULL;
    }
    tree->cize = 0;
    tree->Root = NULL;
    return tree;
}
/*************************************************
  函数名称：TreeNode* mallocTreeNode(void* data, double nIDentifier)
  函数功能: 添加G代码指令字到G代码规则树中
  输入参数：void* data数据域指针   double dIDentifier 识别符
  输出参数：
  返 回 值：返回值为指向这个节点的指针

  备    注：
**************************************************/
TreeNode* mallocTreeNode(void* data, double nIDentifier, int16 node_ORAM_NUM)
{
    TreeNode* node = (TreeNode*)mallocGCode(sizeof(TreeNode), node_ORAM_NUM); //mallocGCode(uint64 ulSize,int8 ORAM_NUM)

    if (node == NULL)
    {
        return NULL;
    }

    node->STreeNodeData.nIDentifier = nIDentifier;
    node->STreeNodeData.pData = data;

    node->height = 1;
    node->left = NULL;
    node->right = NULL;
    return node;
}
/*************************************************
  函数名称：int8 getTreeNodeHeight(TreeNode* node)
  函数功能: 获取树高
  输入参数：TreeNode* node树节点
  输出参数：
  返 回 值：树高

  备    注：
**************************************************/

int8 getTreeNodeHeight(TreeNode* node)
{
    if (node == NULL)
    {
        return 0;
    }
    return node->height;
}

/*************************************************
  函数名称：TreeNode* leftRotate(TreeNode* root)
  函数功能: 左旋函数
  输入参数：TreeNode*root树节点
  输出参数：
  返 回 值：树高

  备    注：//1.旧根节点成为新根节点的左子树
//2.旧根节点的右子树成为新根节点
//3.新根节点原来的左节点成为旧根节点的右子树
**************************************************/

TreeNode* leftRotate(TreeNode* root)
{
    if (root == NULL)
    {
        return NULL;
    }

    TreeNode* newRoot = root->right;
    //用temp保存新根节点原来的左节点
    TreeNode* temp = newRoot->left;

    //1.旧根节点成为新根节点的左子树
    //2.旧根节点的右子树成为新根节点
    //3.新根节点原来的左节点成为旧根节点的右子树
    newRoot->left = root;
    root->right = temp;

    //更新新旧根节点的树高
    root->height = 1 + max(getTreeNodeHeight(root->left), getTreeNodeHeight(root->right));
    newRoot->height = 1 + max(getTreeNodeHeight(newRoot->left), getTreeNodeHeight(newRoot->right));
    return newRoot;
}

//右旋函数
//1.旧根节点成为新根节点的右节点
//2.旧根节点的左子树成为新根节点
//3.新根节点原来的右节点成为旧根节点的左子树
TreeNode* rightRotate(TreeNode* root)
{

    if (root == NULL)
    {
        return NULL;
    }
    TreeNode* newRoot = root->left;
    TreeNode* temp = newRoot->right;

    newRoot->right = root;
    root->left = temp;

    root->height = 1 + max(getTreeNodeHeight(root->left), getTreeNodeHeight(root->right));
    newRoot->height = 1 + max(getTreeNodeHeight(newRoot->left), getTreeNodeHeight(newRoot->right));
    return newRoot;
}

//获取根节点的平衡因子
int8 getBalance(TreeNode* root)
{

    if (root == NULL)
    {
        return 0;
    }
    int8 cBalance = getTreeNodeHeight(root->left) - getTreeNodeHeight(root->right);
    return cBalance;
}

/*************************************************
  函数名称：TreeNode* insertTreeNode(TreeNode* root, void* date, int32 nIDentifier, int8* cErrorSign)
  函数功能: 插入规则结构体节点的函数(加入检查版)
  输入参数：TreeNode*root树节点   GCodeReguration date规则结构体  int8* cErrorSign错误标准位
  输出参数：
  返 回 值：
  备    注： cErrorSign=1时，内存不足
**************************************************/
TreeNode* insertTreeNode(TreeNode* root, void* date, int32 nIDentifier, int8* cErrorSign, int16 node_ORAM_NUM)
{

    //根节点为空，创建节点并返回地址
    if (root == NULL)
    {
        TreeNode* node = mallocTreeNode(date, nIDentifier, node_ORAM_NUM);

        if (node == NULL)
        {
            *cErrorSign = 1;
        }
        return node; //返回节点地址
    }

    //TreeNodeData* pTreeNodedata = (TreeNodeData*)mallocGCode(sizeof(TreeNodeData), ORAM_NUM129);

    //插入节点
    //1.插入的值大于根节点的值，插入根节点的右子树

    if (nIDentifier > root->STreeNodeData.nIDentifier)
    {
        root->right = insertTreeNode(root->right, date, nIDentifier, cErrorSign, node_ORAM_NUM); //迭代插入
    }

    //2.插入的值小于根节点的值，插入根节点的左子树
    else if (nIDentifier < root->STreeNodeData.nIDentifier)
    {
        root->left = insertTreeNode(root->left, date, nIDentifier, cErrorSign, node_ORAM_NUM); //迭代插入
    }
    //3.插入的值等于根节点的值，返回根节点地址
    else
    {

        return root;
    }

    if (*cErrorSign != 1)
    {
        //插入节点后，更新根节点的树高
        (root)->height = 1 + max(getTreeNodeHeight((root)->left), getTreeNodeHeight((root)->right));

        //判断插入节点后，根节点是否失衡

        //获取根节点的平衡因子
        int8 rootbalance = getBalance((root));
        //获取根节点的左节点的平衡因子
        int8 leftBalance = getBalance((root)->left);
        //获取根节点的右节点的平衡因子
        int8 rightBalance = getBalance((root)->right);

        //4种失衡
        //1.LL
        if (rootbalance > 1 && leftBalance > 0)
        {
            //TreeNode* temp = leftRotate(root);
            TreeNode* temp = rightRotate((root));
            return temp; //返回修正后的根节点的地址
        }

        //2.LR
        if (rootbalance > 1 && leftBalance < 0)
        {
            (root)->left = leftRotate((root)->left);
            TreeNode* temp = rightRotate((root));
            return temp;
        }

        //3.RR
        if (rootbalance < -1 && rightBalance < 0)
        {

            TreeNode* temp = leftRotate(root);
            return temp; //返回修正后的根节点的地址
        }

        //4.RL
        if (rootbalance < -1 && rightBalance > 0)
        {
            (root)->right = rightRotate((root)->right);
            TreeNode* temp = leftRotate((root));
            return temp; //返回修正后的根节点的地址
        }

        //没有失衡，返回添加节点后的根节点地址
        return root;
    }
    return root;
}

/*************************************************
  函数名称：TreeNode* find(TreeNode* root, double dGCodeNum, int* counter)
  函数功能:寻找对应G指令字的规则结构体
  输入参数：TreeNode*root树节点指针    double dGCodeNum G指令字
  输出参数： 规则结构体指针
  返 回 值：
  备    注：
**************************************************/
void* findTreeNodeData(TreeNode* root, int32 dGCodeNum)
{
    TreeNode* pCur = root;
    while (pCur != NULL)
    {
        if (dGCodeNum < pCur->STreeNodeData.nIDentifier)
        {
            pCur = pCur->left;
            //(*counter)++;
        }
        else if (dGCodeNum > pCur->STreeNodeData.nIDentifier)
        {
            pCur = pCur->right;
            //(*counter)++;
        }
        else
        {
            return pCur->STreeNodeData.pData; //找到，返回找到的节点的地址
        }
    }

    //找不到，进行报警
    return NULL;
}
