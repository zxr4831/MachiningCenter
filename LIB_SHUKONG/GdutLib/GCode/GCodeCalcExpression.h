#ifndef __GCODECALCEXPRESSION_H
#define __GCODECALCEXPRESSION_H

#include "common.h"
#include "list.h"

#define GCodeCalcExpression_Debug 0

/**********************/
//枚举定义
/**********************/

//宏  优先级，不同优先级对应不同的运算符,数字越大优先级越高
enum MacroPriorityLevel
{
    MACRO_LEVEL_0,
    MACRO_LEVEL_1,
    MACRO_LEVEL_2,
    MACRO_LEVEL_3,
    MACRO_LEVEL_4,
    MACRO_LEVEL_5,
    MACRO_LEVEL_6,
    MACRO_LEVEL_7,
    MACRO_LEVEL_8,
    MACRO_LEVEL_9,
    MACRO_LEVEL_10,
    MACRO_LEVEL_11,
    MACRO_LEVEL_12
};

//宏  运算符标记
enum MacroCalculateSign
{
    //LEVEL_0
    MACRO_L_BRACKET = 0, //左括号 [

    //LEVEL_1
    MACRO_ASSIGN = 0, // =

    //LEVEL_2
    MACRO_AND = 0, //按位与  AND

    //LEVEL_3
    MACRO_XOR = 0, //按位异或   XOR

    //LEVEL_4
    MACRO_OR = 0, //按位或   OR

    //LEVEL_5
    MACRO_NE = 0, // <> NE
    MACRO_EQ = 1, // == EQ

    //LEVEL_6
    MACRO_GT = 0, // > GT
    MACRO_GE = 1, // >=  GE
    MACRO_LT = 2, // < LT
    MACRO_LE = 3, // <= LE

    //LEVEL_7
    MACRO_DOUBLE_ADD = 0, // 双目+
    MACRO_DOUBLE_SUB = 1, // 双目-

    //LEVEL_8
    MACRO_MUL = 0,     // *
    MACRO_DIV = 1,     // /
    MACRO_MODULAR = 2, // %

    //LEVEL_9 全是单目
    MACRO_SINGLE_ADD = 0, // 单目+
    MACRO_SINGLE_SUB = 1, // 单目-

    MACRO_SQRT = 3,  //SQRT
    MACRO_ABS = 4,   //ABS
    MACRO_ROUND = 5, //四舍五入  ROUND
    MACRO_FUP = 6,   //上取整  FUP
    MACRO_FIX = 7,   //下取整  FIX
    MACRO_LN = 8,    //对数  LN
    MACRO_EXP = 9,   //指数 EXP
    MACRO_SIN = 10,
    MACRO_ASIN = 11,
    MACRO_COS = 12,
    MACRO_ACOS = 13,
    MACRO_TAN = 14,
    MACRO_ATAN = 15,
    MACRO_BIN = 16, ///十进制转二进制  BIN
    MACRO_BCD = 17, ///二进制转十进制  BCD

    //LEVEL_10
    MACRO_POUND = 0, //#号    把#号的等级从9提高到10，使得-Z#5能够正常计算 陈育楠 2024.1.25

    //LEVEL_11
    MACRO_R_BRACKET = 0, //右括号  ]0

    //LEVEL_12
    MACRO_NUM = 0, //数字
};

/**********************/
//结构体定义
/**********************/

//局部宏变量
typedef struct _SLocalMacroVar
{
    uint64 ulIsNull;                           //每一位表示一个宏变量是否为空 1:为空 0:不为空  //从#1开始
    double dValArr[MAXIMUM_OF_LOCAL_VARIABLE]; //33个宏变量的值
} SLocalMacroVar;

//计算表达式用的运算单元
typedef struct _SCalculateUnit
{                  //注意:不能往里面添加需要malloc的指针变量!!!
    uint8 ucLevel; //级别:看enum MacroPriorityLevel
    uint8 ucType;  //类型:看enum MacroSign
    bool bPound;   //是否带'#'号  TRUE:带'#'  FALSE:不带'#'
                   //	uint8 ucPoundVal;
    float fVal;    //类型为数字时需要记录数值
} SCalculateUnit;

//与宏运算相关的变量（内含动态内存）
typedef struct _SMacroCalculate
{
    SList* pList;      //链表，其内容就是后缀表达式：数据是SCalculateUnit	//表达式计算完成后一定无节点
    SList* pStack;     //栈：数据是SCalculateUnit							//表达式计算完成后一定无节点
    uint8 ucMacroStep; //计算步骤：0，1，2，3
    uint8 ucMacroPos;  //记录相对于表达式起始的位置 ,比如有表达式 [#1+#2] ,当指针在'['时，Pos=0,当指针在'1'时，Pos=2
    SCalculateUnit SCurrUnit; //当前处理的单元 便于切片
    uint8 ucMacroTypeBMark;   //表达式属于哪一类指令字    0:无 1:无条件GOTO   2.IF  3.WHILE 4.#   //lxw弃用
} SMacroCalculate;

extern SMacroCalculate g_SMacroCalCheck[CHANNEL_NUM_MAX]; //与宏运算有关的结构体（内含动态内存）
extern SMacroCalculate g_SMacroCalAssign[CHANNEL_NUM_MAX];

/**********************/
//函数声明
/**********************/

//初始化所有计算结构体（检查与赋值）
void initAllCalculate(uint8 ucChannelNum);

//初始化计算结构体
void initCalculate(SMacroCalculate* pSMacroCalculate);

//释放计算结构体中开辟的内存
void freeMemCalculate(SMacroCalculate* pSMacroCalculate);

//计算表达式总接口
int8 calculateExpression(const int8* pExpression, uint8 ucLen, SMacroCalculate* pMacroCal,
                         struct _SLocalMacroVar* pSLocalMacroVar, double* pResult, uint16* pPoundVal, uint32 unLineNum);

//判断表达式是否含有赋值符=
bool isAssignable(int8 const* pCurrPtr);

//判断表达式是否需要计算
int8 isNeedCalculate(int8* pExpression, uint8 ucExpressionLen);

//将指针移动到数字的后一位
int8 movePtrOnPureNum(uint8 ucType, const int8** pExpression, uint32 unLineNum);

//将指针移动到对应右括号的后一位
int8 movePtrOnExpression(const int8** pExpression, uint32 unLineNum);

//判断是否为单目运算符
static bool isUnaryOperator(const int8** pExpression, uint32 unLineNum);

//判断是否为双目运算符
static bool isBinaryOperator(const int8** pExpression, uint32 unLineNum);

//将指令移动到表示式的后一位
int8 getWholeExpression(const int8** pExpression, uint32 unLineNum);

//根据操作符计算左值与右值
void calculateMacro(uint8 ucLevel, uint8 ucType, struct _SLocalMacroVar* pSLocalMacroVar,
                    SCalculateUnit* pLeftInputUnit, SCalculateUnit* pRightInputUnit, uint32 unLineNum);

//将计算单元从栈顶移到链表尾部
static void moveUnitFromStackTopToListTail(const SMacroCalculate* pMacroCal, uint32 unLineNum);

//将计算单元压入栈
static void pushUnitIntoStack(const SMacroCalculate* pMacroCal, uint32 unLineNum);

//从表达式中获取数值
int8 getNumbers(const int8* pExpression, uint8* pPos, double* pOutput, uint32 unLineNum);

//判断运算符类型（单目/双目）
int8 isUnaryOrBinary(uint8 cLevel);

//获取计算单元的值
static int8 getUnitVal(SLocalMacroVar* pSLocalMacroVar, SCalculateUnit* pCalculateUnit, float* pUnitVal,
                       uint32 unLineNum);

//读取变量名对应的值
int8 getMacroVar(SLocalMacroVar* pSLocalMacroVar, uint16 usTargetMacroVarName, float* pOutput, uint8** pOutput2,
                 uint32 unLineNum);

//赋予变量名对应的值
void setMacroVar(SLocalMacroVar* pSLocalMacroVar, uint16 usTargetMacroVarName, const float* pVal, uint32 unLineNum);

//判断是哪一个A开头的操作符
static int8 whichAWord(const int8* pCurrPtr, uint8 ucRemainLen, uint32 unLineNum);

#endif
