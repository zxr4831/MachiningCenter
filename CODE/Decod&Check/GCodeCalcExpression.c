
#include "ErrorFunction.h"
#include "GeneralFunctions.h"

#include "Gcode.h"
#if STM32_MODE
#include"dy_link.h"
#endif

SMacroCalculate g_SMacroCalCheck[CHANNEL_NUM_MAX];   //与宏运算有关的结构体（内含动态内存）
SMacroCalculate g_SMacroCalAssign[CHANNEL_NUM_MAX];

/*************************************************
  函数名称：void initCalculate(SMacroCalculate* pSMacroCalculate)
  函数功能: 初始化计算模块
  输入参数：pSMacroCalculate	计算结构体
  输出参数：
  返 回 值：
  备    注：初始化结构体变量的步骤如下
			1.释放结构体中开辟的内存
			2.结构体内全部置0
			3.对初始值非0的变量赋值
**************************************************/
void initAllCalculate(uint8 ucChannelNum)
{
	initCalculate(&g_SMacroCalCheck[ucChannelNum]);
	initCalculate(&g_SMacroCalAssign[ucChannelNum]);
}

/*************************************************
  函数名称：void initCalculate(SMacroCalculate* pSMacroCalculate)
  函数功能: 初始化计算模块
  输入参数：pSMacroCalculate	计算结构体
  输出参数：
  返 回 值：
  备    注：初始化结构体变量的步骤如下
			1.释放结构体中开辟的内存
			2.结构体内全部置0
			3.对初始值非0的变量赋值
**************************************************/
void initCalculate(SMacroCalculate* pSMacroCalculate)
{
	if (pSMacroCalculate == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.释放计算结构体中开辟的内存
	freeMemCalculate(pSMacroCalculate);
	
	//2.清空宏计算结构体
	memset(pSMacroCalculate, 0, sizeof(SMacroCalculate));

	//3.初始化结构体变量
	//3.1初始化后缀表达式链表
	if (pSMacroCalculate->pList == NULL)
	{
		pSMacroCalculate->pList = initList(ORAM_NUM314, ORAM_NUM315); 
		if (pSMacroCalculate->pList == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM406);
			return;
		}
	}

	//3.2初始化栈
	if (pSMacroCalculate->pStack == NULL)
	{
		pSMacroCalculate->pStack = initList(ORAM_NUM316, ORAM_NUM317);  //初始化链表
		if (pSMacroCalculate->pStack == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM406);
			return;
		}
	}

	//3.3初始化计算单元结构体
	memset(&pSMacroCalculate->SCurrUnit, 0, sizeof(SCalculateUnit));

	//4.初始化非结构体变量
	//...
}

/*************************************************
  函数名称：void freeMemCalculate(SMacroCalculate* pSMacroCalculate)
  函数功能: 释放计算结构体中开辟的内存
  输入参数：pSMacroCalculate	计算结构体
  输出参数：
  返 回 值：
  备    注：1.释放pList
			2.释放pStack
**************************************************/
void freeMemCalculate(SMacroCalculate* pSMacroCalculate)
{
	//pSMacroCalculate不能为空
	if (pSMacroCalculate == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//1.释放pList
	freeList(&pSMacroCalculate->pList);//罗祥玮241006：计算结构体中的pList的数据节点中未开辟内存，不需要用freeList_2()进行释放
	//freeList_2(&pSMacroCalculate->pList, freeCalculateData);
	
	//2.开辟pStack
	freeList(&pSMacroCalculate->pStack);//罗祥玮241006：计算结构体中的pStack的数据节点中未开辟内存，不需要用freeList_2()进行释放
	//freeList_2(&pSMacroCalculate->pStack, freeCalculateData);
}

/*************************************************
  函数名称：int8 calculateExpression(const int8* pExpression, uint8 ucLen, SMacroCalculate* pMacroCal, struct _SLocalMacroVar* pSLocalMacroVar, double* pResult
			, uint8* pPoundVal, uint32 unLineNum)
  函数功能：计算表达式的值（总接口，函数内分步执行）
  输入参数：pExpression		表达式首字符地址
			ucLen			表达式长度
			pMacroCal		计算结构体
			pSLocalMacroVar	宏变量结构体
			*pResult		计算结果

			pPoundVal		#值（用于记录G65的的PQR）

			unLineNum		行号用于报错


  输出参数：
  返 回 值：<0:出错
			0:未计算完成
			1:计算完成，结果不需解引用，通过pResult输出计算结果
			2:计算完成，结果需要解引用，且解引用后不为NULL,通过pResult输出解引用后的值
			3:计算完成，结果需要解引用，且解引用后为NULL,通过pResult输出0
  备    注：G65的A类宏，不在此记录PQR原始#i的i值
			检查赋值计算表达式，传入的宏变量结构体不同
**************************************************/
int8 calculateExpression(const int8* pExpression, uint8 ucLen, SMacroCalculate* pMacroCal, struct _SLocalMacroVar*pSLocalMacroVar, double* pResult
	, uint16* pPoundVal, uint32 unLineNum)
{
	int16 cRet = 0;
	int8  cNumOrSymbol = 0;   //运算符:0 数字:1

	if (!pExpression || !pResult || !pMacroCal)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

	//根据运算符或运算值，填写SCurrUnit
	switch (pMacroCal->ucMacroStep)
	{
		//根据当前运算单元，完成pMacroCal->SCurrUnit的赋值
	case 0:    //识别数字、运算符
	{
		//表达式已经被扫完
		if (pMacroCal->ucMacroPos >= ucLen)
		{
			//如果运算符栈不是空的，内容全部弹到链表中
			if (pMacroCal->pStack->cSize > 0)
			{
				SCalculateUnit* pTopUnit = FindNodeData(pMacroCal->pStack, 0);   //读栈顶
				if (!pTopUnit)
				{
					parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
					break;
				}

				moveUnitFromStackTopToListTail(pMacroCal, unLineNum);
				break;
			}
			else //如果为空，进入逆波兰的计算
			{
				pMacroCal->ucMacroStep = 3;
				break;
			}
		}

		//根据数字或运算符的类型，完成计算单元的填写
		switch (*(pExpression + pMacroCal->ucMacroPos))
		{

		case '[':case'(':
		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_0;
			pMacroCal->SCurrUnit.ucType = MACRO_L_BRACKET;
			pMacroCal->ucMacroPos++;
			break;
		}

		case'=':
		{
			if (*(pExpression + pMacroCal->ucMacroPos + 1) == '=')   // '==' EQ 等于
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_5;
				pMacroCal->SCurrUnit.ucType = MACRO_EQ;
				pMacroCal->ucMacroPos += 2;  //移动两个字符
			}
			else  // ‘=’ 赋值
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_1;
				pMacroCal->SCurrUnit.ucType = MACRO_ASSIGN;
				pMacroCal->ucMacroPos++;
			}

			break;
		}

		case'>':
		{
			if (*(pExpression + pMacroCal->ucMacroPos + 1) == '=')   // >= 大于等于
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_GE;
				pMacroCal->ucMacroPos += 2;  //移动两个字符
			}

			else  // >大于
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_GT;
				pMacroCal->ucMacroPos++;
			}
			break;
		}
		case'<':
		{
			if (*(pExpression + pMacroCal->ucMacroPos + 1) == '=')   // <= 小于等于
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_LE;
				pMacroCal->ucMacroPos += 2;  //移动两个字符
			}
			else if (*(pExpression + pMacroCal->ucMacroPos + 1) == '>')   // <> 不等于
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_5;
				pMacroCal->SCurrUnit.ucType = MACRO_NE;
				pMacroCal->ucMacroPos += 2;  //移动两个字符
			}
			else  // <小于
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_LT;
				pMacroCal->ucMacroPos++;
			}
			break;
		}
		case '+':
		{
			//由于每次记录计算单元pMacroCal->SCurrUnit未初始化，所以此时为上一计算单元的信息
			//上一计算单元为：双目+  双目-
			if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_7)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			}
			//上一计算单元为：单目+  单目-	//20240921注释，允许连续出现单目+-，如:++5为5，--5为5
			//else if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_9 && pMacroCal->SCurrUnit.ucType >= 0 && pMacroCal->SCurrUnit.ucType <= 2)
			//{
			//	parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			//}
			//上一计算单元不为：数值
			else if (pMacroCal->SCurrUnit.ucLevel < MACRO_LEVEL_11)   //单目 '+'号
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_SINGLE_ADD;
			}
			else  //双目'+'号
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_7;
				pMacroCal->SCurrUnit.ucType = MACRO_DOUBLE_ADD;
			}
			pMacroCal->ucMacroPos++;
			break;
		}

		case '-':
		{
			//由于每次记录计算单元pMacroCal->SCurrUnit未初始化，所以此时为上一计算单元的信息
			//上一计算单元为：双目+  双目-
			if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_7)//上一个字符为双目运算符，报错
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			}
			//上一计算单元为：单目+  单目-	//20240921注释，允许连续出现单目+-，如:++5为5，--5为5
			//else if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_9 && pMacroCal->SCurrUnit.ucType >= 0 && pMacroCal->SCurrUnit.ucType <= 2)//上一个字符为单目运算符的+ - #，报错
			//{
			//	parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			//}
			//上一计算单元不为：数值
			else if (pMacroCal->SCurrUnit.ucLevel < MACRO_LEVEL_11) //单目的负号 没想到更好的判断方法 上一个字符不为数字，那么-号为单目运算符
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_SINGLE_SUB;
			}
			else  //双目'-'号  //如果上一个字符为数字，那么-号为双目运算符
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_7;
				pMacroCal->SCurrUnit.ucType = MACRO_DOUBLE_SUB;
			}
			pMacroCal->ucMacroPos++;
			break;
		}

		case '*':
		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_8;
			pMacroCal->SCurrUnit.ucType = MACRO_MUL;
			pMacroCal->ucMacroPos++;
			break;
		}

		case '/':
		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_8;
			pMacroCal->SCurrUnit.ucType = MACRO_DIV;
			pMacroCal->ucMacroPos++;
			break;
		}

		case '%':
		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_8;
			pMacroCal->SCurrUnit.ucType = MACRO_MODULAR;
			pMacroCal->ucMacroPos++;
			break;
		}

		case '#':
		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_10;
			pMacroCal->SCurrUnit.ucType = MACRO_POUND;
			pMacroCal->ucMacroPos++;
			break;
		}

		case ']':case')':

		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_11;
			pMacroCal->SCurrUnit.ucType = MACRO_R_BRACKET;
			pMacroCal->ucMacroPos++;
			break;
		}

		case '0':case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9':case '.':   //数字
		{
			pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_12;
			pMacroCal->SCurrUnit.ucType = MACRO_NUM;
			cNumOrSymbol = 1;
			double dValue;
			cRet = getNumbers(pExpression, &pMacroCal->ucMacroPos, &dValue, unLineNum);
			if (cRet < 0)
			{
				return -1;
			}
			pMacroCal->SCurrUnit.fVal = (double)dValue;

			break;
		}

		case'A':    // AND ABS ASIN ACOS ATAN
		{
			uint8 remain = (ucLen > pMacroCal->ucMacroPos) ? (ucLen - pMacroCal->ucMacroPos) : 0;
			cRet = whichAWord(pExpression + pMacroCal->ucMacroPos, remain,unLineNum);    ///哪个以A开头的操作符 未完善
			switch (cRet)
			{
			case 1:  //AND
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_2;
				pMacroCal->SCurrUnit.ucType = MACRO_AND;
				pMacroCal->ucMacroPos += 3;
				break;
			}
			case 2:  //ABS
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_ABS;
				pMacroCal->ucMacroPos += 3;
				break;
			}
			case 3:  //ASIN
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_ASIN;
				pMacroCal->ucMacroPos += 4;
				break;
			}
			case 4:  //ACOS
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_ACOS;
				pMacroCal->ucMacroPos += 4;
				break;
			}
			case 5:  //ATAN
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_ATAN;
				pMacroCal->ucMacroPos += 4;
				break;
			}
			default:
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
				break;
			}

			break;
		}

		case'B':  //BIN BCD
		{
			if (isTagetStr_A("BIN", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_BIN;
				pMacroCal->ucMacroPos += 3;
			}
			else if (isTagetStr_A("BCD", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_BCD;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'C':    //COS
		{
			if (isTagetStr_A("COS", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_COS;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'E':    //EQ EXP
		{

			if (isTagetStr_A("EQ", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_5;
				pMacroCal->SCurrUnit.ucType = MACRO_EQ;
				pMacroCal->ucMacroPos += 2;
			}
			else if (isTagetStr_A("EXP", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_EXP;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}
		case 'F':    //FUP FIX
		{
			if (isTagetStr_A("FUP", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_FUP;
				pMacroCal->ucMacroPos += 3;
			}
			else if (isTagetStr_A("FIX", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_FIX;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'G':    //GT GE
		{
			if (isTagetStr_A("GT", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_GT;
				pMacroCal->ucMacroPos += 2;
			}
			else if (isTagetStr_A("GE", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_GE;
				pMacroCal->ucMacroPos += 2;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'L':    //LT LE LN
		{
			if (isTagetStr_A("LT", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_LT;
				pMacroCal->ucMacroPos += 2;
			}
			else if (isTagetStr_A("LE", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_6;
				pMacroCal->SCurrUnit.ucType = MACRO_LE;
				pMacroCal->ucMacroPos += 2;
			}
			else if (isTagetStr_A("LN", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_LN;
				pMacroCal->ucMacroPos += 2;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'N':    //NE
		{
			if (isTagetStr_A("NE", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_5;
				pMacroCal->SCurrUnit.ucType = MACRO_NE;
				pMacroCal->ucMacroPos += 2;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}
		case 'O':    //OR
		{
			if (isTagetStr_A("OR", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_4;
				pMacroCal->SCurrUnit.ucType = MACRO_OR;
				pMacroCal->ucMacroPos += 2;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'R':    //ROUND 
		{
			if (isTagetStr_A("ROUND", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_ROUND;
				pMacroCal->ucMacroPos += 5;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		case 'S':    //SQRT SIN
		{
			if (isTagetStr_A("SQRT", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_SQRT;
				pMacroCal->ucMacroPos += 4;
			}
			else if (isTagetStr_A("SIN", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_SIN;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}

			break;
		}
		case 'T':    //TAN
		{
			if (isTagetStr_A("TAN", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_9;
				pMacroCal->SCurrUnit.ucType = MACRO_TAN;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}
		case'X':   //XOR
		{
			if (isTagetStr_A("XOR", pExpression + pMacroCal->ucMacroPos))
			{
				pMacroCal->SCurrUnit.ucLevel = MACRO_LEVEL_3;
				pMacroCal->SCurrUnit.ucType = MACRO_XOR;
				pMacroCal->ucMacroPos += 3;
			}
			else
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			}
			break;
		}

		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			break;
		}


		if (cNumOrSymbol == 1)    //如果是数字
		{
			pMacroCal->ucMacroStep = 1;
		}
		else    //如果是运算符
		{
			pMacroCal->ucMacroStep = 2;
		}
		break;
	}

	//运算单元为数字：将pMacroCal->SCurrUnit尾插入链表
	case 1:  //处理数字   处理完后不要修改pMacroCal->SCurrUnit!!!
	{
		//1.开辟计算单元内存
		SCalculateUnit* pCalUnit = mallocGCode(sizeof(SCalculateUnit), ORAM_NUM318);    //堆区分配内存
		if (!pCalUnit)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM406);
			break;
		}
		//2.将记录的计算单元信息写入内存
		*pCalUnit = pMacroCal->SCurrUnit;

		//3.将计算单元内存插入链表尾部
		cRet = insertList(pMacroCal->pList, -1, pCalUnit, ORAM_NUM315);    //尾插入表达式链表
		if (!cRet)
		{
			freeGCode(pCalUnit);
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
			break;
		}

		//4.继续扫描表达式
		pMacroCal->ucMacroStep = 0;  //返回继续处理表达式
		break;
	}

	//运算单元为运算符：将pMacroCal->SCurrUnit分情况讨论
	case 2:   //处理运算符   处理完后不要修改pMacroCal->SCurrUnit!!!
	{
		const SCalculateUnit* pTopUnit = NULL;
		cRet = getListSize(pMacroCal->pStack);

		if (cRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
			break;
		}

		//1.栈内有元素
		if (getListSize(pMacroCal->pStack) > 0)
		{
			pTopUnit = FindNodeData(pMacroCal->pStack, 0);   //读栈顶
			if (!pTopUnit)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
				break;
			}
			//1.1当前为右括号
			if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_11)
			{
				//1.1.1 栈顶为左括号时 ，删除左括号，右括号不需要压栈
				if (pTopUnit->ucLevel == MACRO_LEVEL_0)
				{
					cRet = removeNode(pMacroCal->pStack, 0, FALSE);  //删除栈顶的计算单元： [
					if (!cRet)
					{
						parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
						break;
					}
					pMacroCal->ucMacroStep = 0;  //返回继续处理表达式
				}
				//1.1.2 栈顶为其它
				else
				{
					//移除栈顶元素，并将该元素尾插入链表
					moveUnitFromStackTopToListTail(pMacroCal, unLineNum);
				}
			}

			//1.2 当前为左括号，直接压栈
			else if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_0)
			{
				pushUnitIntoStack(pMacroCal, unLineNum);    //压栈
				pMacroCal->ucMacroStep = 0;   //返回继续处理表达式
			}

			//1.3 当前的等级是否小于等于栈顶的等级
			else if (pMacroCal->SCurrUnit.ucLevel <= pTopUnit->ucLevel)
			{
				//1.3.1 等级相同时，若为单目运算符与赋值运算符 应继续压栈。处理连续单目运算符计算：SQRT SQRT 81 = 3
				if ((pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_9 || pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_1) &&
					pMacroCal->SCurrUnit.ucLevel == pTopUnit->ucLevel)
				{
					//压栈
					pushUnitIntoStack(pMacroCal, unLineNum);    //压栈
					pMacroCal->ucMacroStep = 0;   //返回继续处理表达式
				}
				//1.3.2 其他
				else
				{
					moveUnitFromStackTopToListTail(pMacroCal, unLineNum);   //移除栈顶元素，并将该元素尾插入链表  注意这里并没有把pMacroCal->ucMacroStep置为1！！！，让下一次继续进行符号判断

				}
			}

			//1.4当前pCalUnit的等级大于栈顶的等级
			else
			{
				pushUnitIntoStack(pMacroCal, unLineNum);    //压栈
				pMacroCal->ucMacroStep = 0;   //返回继续处理表达式
			}

		}
		//2.栈内没有元素，直接压栈
		else
		{
//			//栈内没有元素，出现右括号情况应该错误？？？
//			if (pMacroCal->SCurrUnit.ucLevel != MACRO_LEVEL_11)   //右括号不压栈
//			{
//				pushUnitIntoStack(pMacroCal, unLineNum);
//				pMacroCal->ucMacroStep = 0;   //返回继续处理表达式
//			}
		    // 栈为空时遇到右括号：必错（没有可匹配的左括号）
		    if (pMacroCal->SCurrUnit.ucLevel == MACRO_LEVEL_11)  // ')'/']'
		    {
		        parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
		        return -1;
		    }

		    // 栈为空且不是右括号：正常压栈（例如遇到左括号、遇到运算符等）
				pushUnitIntoStack(pMacroCal, unLineNum);
		    pMacroCal->ucMacroStep = 0;   // 返回继续处理表达式
		}

		break;
	}

	//逆波兰表达式已经生成，栈目前是空的，开始计算逆波兰表达式，计算结果压到栈中。
	case 3:
	{
#if GCodeCalcExpression_Debug
		printCalculateUnitList(pMacroCal->pList);
#endif
		//1.取链表第一个元素
		SNode* pFirstNode = FindNode(pMacroCal->pList, 0);
		const SCalculateUnit* pFirstUnit = NULL;
		if (!pFirstNode)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
			break;
		}

		pFirstUnit = (SCalculateUnit*)pFirstNode->pData;

		//2.从栈中取数进行运算
		//2.1数字：取下链表头节点压入栈中
		if (pFirstUnit->ucLevel == MACRO_LEVEL_12)
		{
			///因为插入节点函数会改变节点的下一个节点的指向，所以要先删除，再插入
			cRet = removeNode(pMacroCal->pList, 0, TRUE);   //移除链表第一个节点 ，不释放内存
			if (!cRet)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
				break;
			}

			cRet = insertNode(pMacroCal->pStack, 0, pFirstNode);  //压栈
			if (!cRet)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
				break;
			}

		}
		//2.2运算符
		else
		{
			SNode* pCalNode_1 = NULL;   //第一个节点（栈顶）
			SNode* pCalNode_2 = NULL;   //第二个节点
			SCalculateUnit* pLeftInputUnit = NULL;   //计算结果保留在这
			SCalculateUnit* pRightInputUnit = NULL;

			pCalNode_1 = FindNode(pMacroCal->pStack, 0);  //获取栈顶节点
			if (!pCalNode_1)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
				break;
			}

			cRet = isUnaryOrBinary(pFirstUnit->ucLevel);  //判断单双目
			//2.2.1单目
			if (cRet == 1)
			{
				pLeftInputUnit = (SCalculateUnit*)pCalNode_1->pData;  //单目时，输出数据的节点在第一个节点（栈顶）
				calculateMacro(pFirstUnit->ucLevel, pFirstUnit->ucType, pSLocalMacroVar, pLeftInputUnit, pRightInputUnit, unLineNum);   //计算
			}

			//2.2.2双目
			else
			{
				pCalNode_2 = FindNode(pMacroCal->pStack, 1);
				if (!pCalNode_2)
				{
					parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
					break;
				}

				pLeftInputUnit = (SCalculateUnit*)pCalNode_2->pData;    //双目，输出数据的节点在第二个节点
				pRightInputUnit = (SCalculateUnit*)pCalNode_1->pData;

				calculateMacro(pFirstUnit->ucLevel, pFirstUnit->ucType, pSLocalMacroVar, pLeftInputUnit, pRightInputUnit, unLineNum);   //计算

				cRet = removeNode(pMacroCal->pStack, 0, FALSE);  //删除栈顶,释放内存
				if (!cRet)
				{
					parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
					break;
				}
			}

			//2.2.3计算完成，删除链表第一个元素
			cRet = removeNode(pMacroCal->pList, 0, FALSE);   //移除链表第一个节点,释放内存
			if (!cRet)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
				break;
			}
		}

		//3.判断表达式是否计算完成
		if (pMacroCal->pList->cSize == 0)
		{
			//栈大小必须为1，不然就是表达式有问题
			if (pMacroCal->pStack->cSize == 1)
			{
				pFirstUnit = FindNodeData(pMacroCal->pStack, 0);  //获取栈顶数据
				if (pFirstUnit == NULL)
				{
					parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
					break;
				}
				//创建计算单元，用于存放计算结果
				SCalculateUnit SResultUnit = *pFirstUnit;
				//3.1删除栈顶
				if (!removeNode(pMacroCal->pStack, 0, FALSE))
				{
					parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
					break;
				}

				//计算已完成，以下对计算结果进行分析
				//3.2计算结果不需要解引用
				if (SResultUnit.bPound == FALSE)
				{
					*pResult = SResultUnit.fVal;
					return 1;
				}

				//3.3计算结果需要解引用
				else
				{
					float fTemp;
					uint8* pTemp2 = NULL;
					*pPoundVal = (uint16)SResultUnit.fVal;
					//解引用不为NULL
					if (getMacroVar(pSLocalMacroVar, (uint16)SResultUnit.fVal, &fTemp, &pTemp2, unLineNum) >0)
					{
						//局部宏变量
						if (SResultUnit.fVal <= 33)
						{
							*pResult = fTemp;
						}
						//系统宏变量
						else if (SResultUnit.fVal >= 1000)
						{
							*pResult = *pTemp2;
						}
						return 2;
					}
					//解引用为NULL
					else
					{
						*pResult = 0;
						return 3;
					}
				}
			}
			//报错
			else
			{
				//pMacroCal->ucMacroStep = 123;
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
				break;
			}
		}

		break;
	}



	default:
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
		break;
	}


	return 0;
}



/*************************************************
  函数名称：bool isAssignable(int8 const* pCurrPtr)
  函数功能：判断表达式是否含有赋值符=
  输入参数：

  输出参数：
  返 回 值：TRUE	含有赋值=
			FALSE	不含赋值=
  备    注：
**************************************************/
bool isAssignable(int8 const* pCurrPtr)
{
    if (pCurrPtr == NULL)
    {
        parseGCodeErrorB(0, GCODE_ERROR_NUM300);
        return FALSE;
    }
    while (1)
	{
        // ***复用你项目的“段/行结束”语义：遇到结束就停止扫描***
        if (isSegmentFinishSign((const char*)pCurrPtr) || isLineFinishSign((const char*)pCurrPtr))
		{
			return FALSE;
		}

        if (*pCurrPtr == '\0' || *pCurrPtr == '\r' || *pCurrPtr == '\n')
        {
            return FALSE;
	}

        if (*pCurrPtr == '=')
        {
            const char prev = (pCurrPtr > (const int8*)0) ? *(pCurrPtr - 1) : 0; // prev 仅用于比较，避免误用
            const char next = *(pCurrPtr + 1);

            // ***排除比较运算符：==、<=、>=、<>***
            if (next == '=')          // ==
                return FALSE;
            if (prev == '<' || prev == '>')  // <= 或 >= 或 <>（其中 <> 的 '=' 不存在，但 prev 保护不伤）
                return FALSE;

            // 这里才认为是赋值 '='
		return TRUE;
	}

        pCurrPtr++;
    }

	return FALSE;
}

/*************************************************
  函数名称：int8 isNeedCalculate(int8* pExpression, uint8 ucLen)
  函数功能: 判断表达式Expression是否需要进行宏计算
  输入参数：pExpression		表达式首字符地址
			ucLen			表达式字符串长度
  输出参数：
  返 回 值：TRUE	需要宏计算
			FALSE	不需要宏计算
  备    注：字符串从第2位开始仅由0至9和.组成则不需要计算（+10与-10不需要计算）
**************************************************/
int8 isNeedCalculate(int8* pExpression, uint8 ucExpressionLen)
{
    if (pExpression == NULL || ucExpressionLen == 0)
    {
        return -1;
    }
	uint8 i = 0;
    uint8 digitCnt = 0;     // **至少一位数字**
    uint8 pointCnt = 0;     // **最多一个小数点**
    uint8 signCnt  = 0;     // **最多一个符号，且只能在首位**
	//表达式第1位不为 + or - or 数字 or 小数点
    // 1) 首字符允许 + - . 数字
     //    但符号只能出现在开头
     if (pExpression[0] == '+' || pExpression[0] == '-')
     {
         signCnt = 1;
         i = 1;
         // 表达式仅一个符号：如 "+" "-"，肯定不是合法纯数字
         if (i >= ucExpressionLen) return TRUE;
	}
     // 2) 扫描剩余字符：只能是 数字 或 '.'
        for (; i < ucExpressionLen; i++)
	{
            char c = pExpression[i];

            if (c >= '0' && c <= '9')
            {
                digitCnt++;
            }
            else if (c == '.')
		{
                pointCnt++;
                if (pointCnt > 1) return TRUE;   // 多个点：不是纯数字
            }
            else if ((c == '+' || c == '-') )
            {
                // 中间再出现符号：一定不是纯数字（例如 "1-2" 需要计算）
                return TRUE;
		}
		else
		{
                // 出现其它字符（#、[]、函数名等）：需要计算
			return TRUE;
		}
        }
		
        // 3) **关键：必须至少一位数字**，否则让后续流程报错（不要走 atof）
        if (digitCnt == 0)
        {
            return TRUE;
	}

        // 能走到这里：表示是合法纯数字字面量（例如 "10" "-3.5" ".5" "12."）
	return FALSE;
}

/*************************************************
  函数名称：static int8 movePtrOnPureNum(uint8 ucType, const int8** pExpression, uint32 unLineNum)
  函数功能: *pExpression移动指针到数字的后一位
  输入参数：ucType:0浮点，1有符号整型，2无符号整型,*pExpression当前字符的地址，unLineNum行号
  输出参数：
  返 回 值：bErrorFlag 真:有错误 假：无错误
  备    注：
**************************************************/
 int8 movePtrOnPureNum(uint8 ucType, const int8** pExpression, uint32 unLineNum)
{
	uint8 nSignCounter = 0;     //正负号出现次数
	uint8 nPointCounter = 0;    //小数点出现次数
	uint8 digitCnt = 0;
	const int8* pTemp = NULL;
	bool bErrorFlag = FALSE;

	if (!pExpression || !(*pExpression))
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return TRUE;
	}
	pTemp = *pExpression;

	while ((*pTemp == '+') || (*pTemp == '-'))
	{
        nSignCounter++;                         // ***原代码缺失++***
        if (nSignCounter > 1) bErrorFlag = TRUE;
        if (ucType == 2 && *pTemp == '-') bErrorFlag = TRUE;   // 无符号不允许负号

        pTemp++;
	}
	nSignCounter = 0;


	while ((((*pTemp >= '0') && (*pTemp <= '9')) || (*pTemp == '.')))   //获取0-9及小数点
	{
		if (*pTemp == '.')
		{
			nPointCounter++;
		}
	    else
		{
			digitCnt++;                          // ***统计数字***
		}
		if (nPointCounter > 1 || ucType > 0)
		{
			if (nPointCounter > 2)
			{
				pTemp += 1;
				continue;
			}
			bErrorFlag = TRUE;
			nPointCounter++;
		}

		pTemp += 1;
	}
    // 3) ***至少一位数字校验：+、-、.、-.、+. 直接判错***
    if (digitCnt == 0)
    {
        bErrorFlag = TRUE;
    }
	*pExpression = pTemp;

	return bErrorFlag;
}

/*************************************************
  函数名称：int8 movePtrOnExpression(const int8 **pExpression, uint32 unLineNum)
  函数功能: *pExpression移动指针到(以括号封闭的)表达式的后一位，必然是右括号的后一位
  输入参数：*pExpression当前字符的地址，unLineNum行号
  输出参数：
  返 回 值：-1,-2出错  1完成
  备    注：
**************************************************/
int8 movePtrOnExpression(const int8** pExpression, uint32 unLineNum)
{
	int8 cCnt = 1;   //括号计数器  为0说明移动完成
	const int8* pTemp = (*pExpression) + 1; //先移到左括号的后一位

	if (!pExpression || !(*pExpression))
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

	while (cCnt != 0)
	{
		if ((*pTemp == '[') || (*pTemp == '('))
		{
			cCnt++;
		}
		else if ((*pTemp == ']') || (*pTemp == ')'))
		{
			cCnt--;
		}
		else if (*pTemp == 0 || *pTemp == '\r' || *pTemp == '\n' || *pTemp == ';')
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			return -2;
		}
		pTemp++;
	}

	if (*pTemp == ']' || *pTemp == ')')
	{
		pTemp++;
	}
	else
	*pExpression = pTemp;//移到右括号的后一位
	return 1;
}


/*************************************************
  函数名称：static bool isUnaryOperator(const int8 **pExpression, uint32 unLineNum)
  函数功能: 判断是不是单目运算符，如果是，移动指针到单目运算符的后一位
  输入参数：*pExpression当前字符的地址，unLineNum行号
  输出参数：
  返 回 值：FALSE:出错或不是单目运算符
			TRUE :处理完成
  备    注：
**************************************************/
static bool isUnaryOperator(const int8** pExpression, uint32 unLineNum)
{
	if (pExpression == NULL || *pExpression == NULL)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return FALSE;
	}

	if (**pExpression == '+' || **pExpression == '-' || **pExpression == '#')
	{
		*pExpression += 1;
		return TRUE;
	}
	else if (isTagetStr_B("LN", *pExpression))
	{
		*pExpression += 2;
		return TRUE;
	}
	else if (isTagetStr_B("ABS", *pExpression) || isTagetStr_B("FUP", *pExpression) || isTagetStr_B("FIX", *pExpression)
		|| isTagetStr_B("EXP", *pExpression) || isTagetStr_B("SIN", *pExpression) || isTagetStr_B("COS", *pExpression)
		|| isTagetStr_B("TAN", *pExpression) || isTagetStr_B("BIN", *pExpression) || isTagetStr_B("BCD", *pExpression))
	{
		*pExpression += 3;
		return TRUE;
	}
	else if (isTagetStr_B("SQRT", *pExpression) || isTagetStr_B("ASIN", *pExpression) || isTagetStr_B("ACOS", *pExpression)
		|| isTagetStr_B("ATAN", *pExpression))
	{
		*pExpression += 4;
		return TRUE;
	}
	else if (isTagetStr_B("ROUND", *pExpression))
	{
		*pExpression += 5;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*************************************************
  函数名称：static bool isBinaryOperator(const int8 **pExpression, uint32 unLineNum)
  函数功能: 判断是不是双目运算符，如果是，移动指针到双目运算符的后一位
  输入参数：*pExpression当前字符的地址，unLineNum行号
  输出参数：
  返 回 值：FALSE:出错或不是双目运算符
			TRUE :处理完成
  备    注：
**************************************************/
static bool isBinaryOperator(const int8** pExpression, uint32 unLineNum)
{
	if (pExpression == NULL || *pExpression == NULL)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return FALSE;
	}

	if (**pExpression == '+' || **pExpression == '-' || **pExpression == '*' || **pExpression == '/' || **pExpression == '%')
	{
		*pExpression += 1;
		return TRUE;
	}
	else if (isTagetStr_B("AND", *pExpression) || isTagetStr_B("XOR", *pExpression))
	{
		*pExpression += 3;
		return TRUE;
	}
	else if (isTagetStr_B("OR", *pExpression) || isTagetStr_B("NE", *pExpression) || isTagetStr_B("EQ", *pExpression) || isTagetStr_B("GT", *pExpression)
		|| isTagetStr_B("GE", *pExpression) || isTagetStr_B("LT", *pExpression) || isTagetStr_B("LE", *pExpression) || isTagetStr_B(">=", *pExpression)
		|| isTagetStr_B("<=", *pExpression) || isTagetStr_B("<>", *pExpression) || isTagetStr_B("==", *pExpression))
	{
		*pExpression += 2;
		return TRUE;
	}
	else if (**pExpression == '>' || **pExpression == '<' || **pExpression == '=')
	{
		*pExpression += 1;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************
  函数名称：int8 getWholeExpression(const int8 **pExpression, uint32 unLineNum)
  函数功能: pExpression移动到完整的表达式后一位
  输入参数：pExpression表达式首字符地址
			unLineNum行号
  输出参数：
  返 回 值：<0  出错，表达式错误
			1   完成
  备    注：此函数能初步判断表达式是否正确
**************************************************/
int8 getWholeExpression(const int8** pExpression, uint32 unLineNum)
{
	int8 cRet = 0;
	if (pExpression == NULL || *pExpression == NULL)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return -1;
	}
	//数值，把当前指针移到最后一位数字的后面
	if ((**pExpression >= '0' && **pExpression <= '9') || **pExpression == '.')   
	{
		cRet = movePtrOnPureNum(0, pExpression, unLineNum);
		if (cRet)
		{
			setErrorType(GCODE_ERROR_NUM84);
			findError(*pExpression, unLineNum);
			return -2;
		}
	}
	// 表达式，把当前指针移到右括号的后一位
	else if ((**pExpression == '[') || (**pExpression == '(')) 
	{
		cRet = movePtrOnExpression(pExpression, unLineNum);
		if (cRet < 0)
		{
			setErrorType(GCODE_ERROR_NUM134);
			findError(*pExpression, unLineNum);
			return -3;
		}
	}
	//单目运算符，把当前指针移到单目运算符的后一位  不能连续出现两个单目运算符
	else if (isUnaryOperator(pExpression, unLineNum))     // ，这里判断不充分 可能出现##或++等指令，放到后面计算时处理，
	{
		if(getWholeExpression(pExpression, unLineNum) <= 0)  //递归  再进行移到指针操作
		{
			setErrorType(GCODE_ERROR_NUM134);
			findError(*pExpression, unLineNum);
			return -3;
		}
	}
	//错误//lxw:不检查表达式是否正确，若表达式错误，则计算时会报错
	//else
	//{
	//	setErrorType(GCODE_ERROR_NUM134);
	//	findError(*pExpression, unLineNum);
	//	return -3;
	//}
	
	//双目运算符，把当前指针移到单目运算符的后一位
	if (isBinaryOperator(pExpression, unLineNum))  //表达式还没结束  移到双目运算符的后一位
	{
		if(getWholeExpression(pExpression, unLineNum) <= 0)  //递归
		{
			setErrorType(GCODE_ERROR_NUM134);
			findError(*pExpression, unLineNum);
			return -3;
		}
	}


	return 1;
}


/*************************************************
  函数名称：
  函数功能: 根据运算符类型，取计算单元的值进行计算，计算结果保留到左输入值
  输入参数：ucLevel			运算符的优先级
			ucType			运算符的类型

			pSLocalMacroVar		局部宏变量结构体地址，用于解引用

			pLeftInputUnit	左计算单元（计算单元结构体）
			pRightInputUnit	右计算单元（计算单元结构体）
			unLineNum		行号，用于报错
  输出参数：
  返 回 值：
  备    注：1.一定有左计算单元，不一定有右计算单元
			2.左右计算单元计算完成后，结果写入左计算单元，覆盖左计算单元原有值
			3.计算完成不删除右计算单元
**************************************************/
void calculateMacro(uint8 ucLevel, uint8 ucType, struct _SLocalMacroVar* pSLocalMacroVar, SCalculateUnit* pLeftInputUnit, SCalculateUnit* pRightInputUnit,  uint32 unLineNum)
{
	float fLeftVal = 0;   //左值，接左计算单元的值（解引用后的值）
	float fRightVal = 0;  //右值，接右计算单元的值（解引用后的值）

	//所有运算中左输入值不能为空，右输入值后续根据运算符判断是否允许为空
	if (pLeftInputUnit == NULL)  
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	switch (ucLevel)
	{
	//运算符：=
	case MACRO_LEVEL_1:  
	{
		if (pRightInputUnit == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		int8 cRet;			//getMacroVar返回值，用于判断读取宏变量是否为NULL

		//左计算单元必须为宏变量，即带有#
		if (pLeftInputUnit->bPound == TRUE)  
		{
			//左计算单元为 系统宏变量 or #0
			if (pLeftInputUnit->fVal > 1000 || pLeftInputUnit->fVal == 0)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);//系统宏 or #0 不能被赋值
			}
			//获取右计算单元的值
			cRet = getUnitVal(pSLocalMacroVar, pRightInputUnit, &fRightVal, unLineNum);
			if (cRet > 0)
			{
				setMacroVar(pSLocalMacroVar, (uint16)pLeftInputUnit->fVal, &fRightVal, unLineNum);  //修改左值的宏变量
			}
			//1.2右计算单元解引用后为NULL
			else if(cRet == 0)
			{
				//手册：#2为NULL，#1 = #2。#1被赋值为NULL
				setMacroVar(pSLocalMacroVar, (uint16)pLeftInputUnit->fVal, NULL, unLineNum); //修改左值的宏变量
			}
			else
			{
				return;
			}
		}
		//左计算单元不为宏变量，报错
		else
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
		}

		break;
	}
	
	//运算符：&  ^  |
	case MACRO_LEVEL_2:case MACRO_LEVEL_3:case MACRO_LEVEL_4:  
	{
		int32 nTemp_1, nTemp_2;
		int8 cRet;
		if (pRightInputUnit == NULL)   //右输入值不能为空
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		//获取左计算单元的值
		cRet = getUnitVal(pSLocalMacroVar, pLeftInputUnit, &fLeftVal, unLineNum);
		if(cRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return;
		}
		//获取右计算单元的值
		cRet =getUnitVal(pSLocalMacroVar, pRightInputUnit, &fRightVal, unLineNum);
		if(cRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return;
		}
		nTemp_1 = (int32)fLeftVal;
		nTemp_2 = (int32)fRightVal;

		if (ucLevel == MACRO_LEVEL_2)    //按位与
		{
			pLeftInputUnit->fVal = (float)(nTemp_1 & nTemp_2);
		}
		else if (ucLevel == MACRO_LEVEL_3)   //按位异或
		{
			pLeftInputUnit->fVal = (float)(nTemp_1 ^ nTemp_2);
		}
		else   //按位或
		{
			pLeftInputUnit->fVal = (float)(nTemp_1 | nTemp_2);
		}
		break;
	}

	//运算符：==  !=
	case MACRO_LEVEL_5:
	{
		int8 cLeftRet;
		int8 cRightRet;

		if (pRightInputUnit == NULL)   //右输入值不能为空
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		//获取左计算单元的值
		cLeftRet = getUnitVal(pSLocalMacroVar, pLeftInputUnit, &fLeftVal, unLineNum);

		//获取右计算单元的值
		cRightRet = getUnitVal(pSLocalMacroVar, pRightInputUnit, &fRightVal, unLineNum);

		switch (ucType)
		{
		case MACRO_NE:   //不等于
		{
			//手册：左值为NULL，右值为NULL，则左值!=右值不成立，结果为0
			if (cLeftRet == 0 && cRightRet == 0)    //都为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值为NULL，右值不为NULL，则左值!=右值成立，结果为1
			else if ((cLeftRet == 0 && cRightRet == 1) || (cLeftRet == 1 && cRightRet == 0))  //其中一个为空变量
			{
				pLeftInputUnit->fVal = 1;
			}
			//手册：左值不为NULL，右值不为NULL，则正常判断
			else
			{
				pLeftInputUnit->fVal = (float)(isZero(fLeftVal - fRightVal) ? 0 : 1);  //得到结果
			}
			break;
		}

		case MACRO_EQ:   //等于
		{
			//手册：左值为NULL，右值为NULL，则左值==右值成立，结果为1
			if (cLeftRet == 0 && cRightRet == 0)    //都为空变量
			{
				pLeftInputUnit->fVal = 1;
			}
			//手册：左值为NULL，右值不为NULL，则左值==右值不成立，结果为0
			else if ((cLeftRet == 0 && cRightRet == 1) || (cLeftRet == 1 && cRightRet == 0))  //其中一个为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值不为NULL，右值不为NULL，则正常判断
			else
			{
				pLeftInputUnit->fVal = (float)(isZero(fLeftVal - fRightVal) ? 1 : 0);  //得到结果
			}
			break;
		}

		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			break;
		}
		break;
	}

	//运算符：>  >=  <  <=
	case MACRO_LEVEL_6:
	{
		int8 cLeftRet;
		int8 cRightRet;

		if (pRightInputUnit == NULL)   //右输入值不能为空
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}

		//获取左计算单元的值
		cLeftRet = getUnitVal(pSLocalMacroVar, pLeftInputUnit, &fLeftVal, unLineNum);

		//获取右计算单元的值
		cRightRet = getUnitVal(pSLocalMacroVar, pRightInputUnit, &fRightVal, unLineNum);

		switch (ucType)
		{
		case MACRO_GT:   //大于
		{
			//手册：左值为NULL，右值为NULL，则左值>右值不成立，结果为0
			if (cLeftRet == 0 && cRightRet == 0)    //都为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值为NULL，右值不为NULL，则左值>右值不成立，结果为0
			else if ((cLeftRet == 0 && cRightRet == 1) || (cLeftRet == 1 && cRightRet == 0))  //其中一个为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值不为NULL，右值不为NULL，则正常判断
			else
			{
				pLeftInputUnit->fVal = (float)(fLeftVal > fRightVal ? 1 : 0);  //得到结果
			}
			break;
		}

		case MACRO_GE:    //大于等于
		{
			//手册：左值为NULL，右值为NULL，则左值>=右值成立，结果为1
			if (cLeftRet == 0 && cRightRet == 0)    //都为空变量
			{
				pLeftInputUnit->fVal = 1;
			}
			//手册：左值为NULL，右值不为NULL，则左值>=右值不成立，结果为0
			else if ((cLeftRet == 0 && cRightRet == 1) || (cLeftRet == 1 && cRightRet == 0))  //其中一个为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值不为NULL，右值不为NULL，则正常判断
			else
			{
				if (fLeftVal > fRightVal || isZero(fLeftVal - fRightVal))
				{
					pLeftInputUnit->fVal = 1;
				}
				else
				{
					pLeftInputUnit->fVal = 0;
				}
			}
			break;
		}



		case MACRO_LT:    //小于
		{
			//手册：左值为NULL，右值为NULL，则左值<右值不成立，结果为0
			if (cLeftRet == 0 && cRightRet == 0)    //都为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值为NULL，右值不为NULL，则左值<右值不成立，结果为0
			else if ((cLeftRet == 0 && cRightRet == 1) || (cLeftRet == 1 && cRightRet == 0))  //其中一个为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值不为NULL，右值不为NULL，则正常判断
			else
			{
				pLeftInputUnit->fVal = (float)(fLeftVal < fRightVal ? 1 : 0);  //得到结果
			}
			break;
		}

		case MACRO_LE:   //小于等于
		{
			//手册：左值为NULL，右值为NULL，则左值<=右值成立，结果为1
			if (cLeftRet == 0 && cRightRet == 0)    //都为空变量
			{
				pLeftInputUnit->fVal = 1;
			}
			//手册：左值为NULL，右值不为NULL，则左值<=右值不成立，结果为0
			else if ((cLeftRet == 0 && cRightRet == 1) || (cLeftRet == 1 && cRightRet == 0))  //其中一个为空变量
			{
				pLeftInputUnit->fVal = 0;
			}
			//手册：左值不为NULL，右值不为NULL，则正常判断
			else
			{
				if (fLeftVal < fRightVal || isZero(fLeftVal - fRightVal))
				{
					pLeftInputUnit->fVal = 1;
				}
				else
				{
					pLeftInputUnit->fVal = 0;
				}
			}
			break;
		}

		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			break;
		}

		break;
	}
	
	//运算符：双目+  双目-
	case MACRO_LEVEL_7:
	{
		int8 cLeftRet;
		if (pRightInputUnit == NULL)   //右输入值不能为空
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		//获取左计算单元的值
		cLeftRet = getUnitVal(pSLocalMacroVar, pLeftInputUnit, &fLeftVal, unLineNum);
		if(cLeftRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return ;
		}
		//获取右计算单元的值
		cLeftRet = getUnitVal(pSLocalMacroVar, pRightInputUnit, &fRightVal, unLineNum);
		if(cLeftRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return ;
		}
		switch (ucType)
		{
		case MACRO_DOUBLE_ADD:   //双目+
		{
			pLeftInputUnit->fVal = fLeftVal + fRightVal;  //得到结果
			break;
		}
		case MACRO_DOUBLE_SUB:   //双目-
		{
			pLeftInputUnit->fVal = fLeftVal - fRightVal;  //得到结果
			break;		
		}
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			break;
		}
		break;
	}
	//运算符：*  /  %
	case MACRO_LEVEL_8:
	{
		int8 cLeftRet;
		if (pRightInputUnit == NULL)   //右输入值不能为空
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return;
		}
		//获取左计算单元的值
		cLeftRet = getUnitVal(pSLocalMacroVar, pLeftInputUnit, &fLeftVal, unLineNum);
		if(cLeftRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return;
		}
		//获取右计算单元的值
		cLeftRet = getUnitVal(pSLocalMacroVar, pRightInputUnit, &fRightVal, unLineNum);
		if(cLeftRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return ;
		}
		switch (ucType)
		{
		case MACRO_MUL:				//乘法
		{
			pLeftInputUnit->fVal = fLeftVal * fRightVal;  //得到结果
			break;
		}
		case MACRO_DIV:				//除法
		{
			if (isZero(fRightVal))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM132);   //除数不能为0
				break;
			}
			pLeftInputUnit->fVal = fLeftVal / fRightVal;  //得到结果
			break;		
		}
		case MACRO_MODULAR:			//取余
		{
			if (isZero(fRightVal))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM132);   //除数不能为0
				break;
			}
			pLeftInputUnit->fVal = (float)(fmod(fLeftVal, fRightVal));//两个浮点数取余
			break;		
		}
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			break;
		}
		break;
	}
	
	//运算符：所有单目运算符
	case MACRO_LEVEL_9:
	{
		int8 cRet;
		//获取左计算单元的值
		cRet = getUnitVal(pSLocalMacroVar, pLeftInputUnit, &fLeftVal, unLineNum);
		if(cRet < 0)
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
			return;
		}
		switch (ucType)
		{
		case MACRO_SINGLE_ADD:		//单目+ ,可忽略
			break;
		case MACRO_SINGLE_SUB:		//单目-
		{
			pLeftInputUnit->fVal = -fLeftVal;
			break;
		}
		case MACRO_SQRT:			//开根号
		{
			if (fLeftVal < 0)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM452);
				return;
			}
			pLeftInputUnit->fVal = (float)(sqrt(fLeftVal));
			break;
		}
		case MACRO_ABS:				//取绝对值
		{
			pLeftInputUnit->fVal = (float)(fabs(fLeftVal));
			break;
		}
		case MACRO_ROUND:			//四舍五入
		{
			pLeftInputUnit->fVal = (float)(round(fLeftVal));
			break;
		}
		case MACRO_FUP:				//正数向上取整，负数向下取整
		{
			bool isNegative = fLeftVal < 0 ? TRUE : FALSE;    //是否小于0
			float fTemp = (float)fabs(fLeftVal);    //取绝对值
			fTemp = (float)(ceil(fTemp));    //向上取整，即大于x的最小整数
			//正数
			if (isNegative == FALSE)
			{
				pLeftInputUnit->fVal = fTemp;
			}
			//负数
			else
			{
				pLeftInputUnit->fVal = -fTemp;
			}
			break;
		}
		case MACRO_FIX:				//正数向下取整，负数向上取整
		{
			bool isNegative = fLeftVal < 0 ? TRUE : FALSE;    //是否小于0
			float fTemp = (float)fabs(fLeftVal);    //取绝对值
			fTemp = (float)(floor(fTemp));    //向下取整，floor(x)返回的是小于或等于x的最大整数
			if (isNegative == FALSE)
			{
				pLeftInputUnit->fVal = fTemp;
			}
			else
			{
				pLeftInputUnit->fVal = -fTemp;
			}
			break;
		}
		case MACRO_LN:				//取对数
		{
			//1.double log(double x) 求的是lnx(log(e)(x))
			//2.double log10(double x) 求log(10)(x)
			if (fLeftVal < 0 || isZero(fLeftVal))   //x需要大于等于0
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM452);
				return;
			}
			pLeftInputUnit->fVal = (float)(log(fLeftVal));

			break;
		}
		case MACRO_EXP:				//自然指数
		{
			pLeftInputUnit->fVal = (float)(exp(fLeftVal));
			break;
		}
		case MACRO_SIN:				//正弦
		{
			pLeftInputUnit->fVal = (float)(sin(ANGLE_TO_RAD(fLeftVal)));
			break;
		}
		case MACRO_ASIN:			//反正弦
		{
			float fRad = 0;
			if (fLeftVal < -1 || fLeftVal > 1 || isZero(fLeftVal - 1) || isZero(fLeftVal + 1))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM452);
				return;
			}
			fRad = (float)asin(fLeftVal);    //求弧度  该函数返回以弧度表示的 x 的反正弦，弧度区间为 [-pi/2,+pi/2]。
			//pLeftInputUnit->fVal = (float)(RAD_TO_ANGEL(fRad) + 180 * TEMP_PARA_NO180);    //No.180设为1时，90°~ 270°；设为0时，-90°~ 90°；
			pLeftInputUnit->fVal = (float)(RAD_TO_ANGEL(fRad)) + (float)((90 - RAD_TO_ANGEL(fRad)) * 2);//No.180设为1时，90°~ 270°；设为0时，-90°~ 90°；
			break;
		}
		case MACRO_COS:				//余弦
		{
			pLeftInputUnit->fVal = (float)cos(ANGLE_TO_RAD(fLeftVal));
			break;
		}
		case MACRO_ACOS:			//反余弦
		{
			float fRad = 0;
			if (fLeftVal < -1 || fLeftVal > 1 || isZero(fLeftVal - 1) || isZero(fLeftVal + 1))
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM452);
				return;
			}
			fRad = (float)acos(fLeftVal);    //求弧度 该函数返回以弧度表示的 x 的反余弦，弧度区间为 [0, pi]。
			pLeftInputUnit->fVal = (float)(RAD_TO_ANGEL(fRad));    //输入范围[-1,1]，输出范围只有0~180°
			break;
		}
		case MACRO_TAN:				//正切
		{
			pLeftInputUnit->fVal = (float)tan(ANGLE_TO_RAD(fLeftVal));
			break;
		}
		case MACRO_ATAN:			//反正切
		{
			float fRad = (float)atan(fLeftVal);
			pLeftInputUnit->fVal = (float)(RAD_TO_ANGEL(fRad) + 180 * TEMP_PARA_NO180);//No.180设为1时，90°~ 270°；设为0时，-90°~ 90°；
			break;
		}
		//case MACRO_BIN:
		//{
		//	break;
		//}
		//case MACRO_BCD:
		//{
		//	break;
		//}
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			break;
		}
		break;
	}
	
	//运算符：#
	case MACRO_LEVEL_10:
	{
		switch (ucType)
		{
		//操作符：#
		case MACRO_POUND:    
		{
			//左值已被引用，报错
			if (pLeftInputUnit->bPound == TRUE)
			{
				parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			}
			pLeftInputUnit->bPound = TRUE;
			break;
		}
		default:
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
			break;
		}
		break;

	}

	default:
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM134);
		break;
	}


}

/*************************************************
  函数名称：static void moveUnitFromStackTopToListTail(const SMacroCalculate* pMacroCal, uint32 unLineNum)
  函数功能：生成后缀表达式时,将栈顶元素移动到链表尾部
  输入参数：pMacroCal		计算结构体
			unLineNum		行号用于报错
  输出参数：
  返 回 值：
  备    注：
**************************************************/
static void moveUnitFromStackTopToListTail(const SMacroCalculate* pMacroCal, uint32 unLineNum)
{
	SNode* pDel = FindNode(pMacroCal->pStack, 0);   //先获取该节点

	if (!removeNode(pMacroCal->pStack, 0, TRUE))  //从链表中移除节点，不释放内存
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
		return;
	}


	if (!insertNode(pMacroCal->pList, -1, pDel))   //节点尾插入数字链表
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM303);
		return;
	}
}
/*************************************************
  函数名称：static void pushUnitIntoStack(const SMacroCalculate* pMacroCal, uint32 unLineNum)
  函数功能：开辟内存，记录运算符信息后压入栈中
  输入参数：pMacroCal		计算结构体
			unLineNum		行号用于报错
  输出参数：
  返 回 值：
  备    注：
**************************************************/
static void pushUnitIntoStack(const SMacroCalculate* pMacroCal, uint32 unLineNum)
{
	SCalculateUnit* pCalUnit = mallocGCode(sizeof(SCalculateUnit), ORAM_NUM318);    //堆区分配内存
	if (!pCalUnit)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM406);
		return;
	}

	*pCalUnit = pMacroCal->SCurrUnit;

	if (!insertList(pMacroCal->pStack, 0, pCalUnit, ORAM_NUM317))  //压到符号栈
	{
		freeGCode(pCalUnit);
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM406);
		return;
	}
}

/*************************************************
  函数名称：static int8 getNumOnExpression(const int8* pCurrPtr, uint8* pPos, double* pOutput, uint32 unLineNum)
  函数功能：从表达式的pPos位置开始获取数字，并将数字从字符串形式转为数值形式
  输入参数：pExpression		表达式首字符
			pPos			当前表达式扫描位置
			pOutput			数值地址（用于输出）
			unLineNum		行号用于报错
  输出参数：
  返 回 值：0	获取成功
			-1	获取失败
  备    注：获取数字同时进行检查，最多允许出现一个小数点
  +60.1
**************************************************/
int8 getNumbers(const int8* pExpression, uint8* pPos, double* pOutput, uint32 unLineNum)
{
	uint8 cPointCnt = 0;       //小数点计数
	uint8 digitCnt = 0;   // ***新增：数字计数器***
	int8 cTempArr[MAX_LENGTH_OF_LEXICAL_UNIT+1] = { 0 };
	uint8 ucIdx = 0;
	int8 cCurr;

	if (!pExpression || !pOutput || !pPos)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

	cCurr = *(pExpression + *pPos);

	//1.判断首个字符是否为 + or -
	if (cCurr == '+' || cCurr == '-')
	{
		if (ucIdx >= MAX_LENGTH_OF_LEXICAL_UNIT)
		{
			setErrorType(GCODE_ERROR_NUM85);
			findError(pExpression, unLineNum);
			return -1;
		}
		cTempArr[ucIdx++] = cCurr;  //保存到数组中

		*pPos += 1;
		cCurr = *(pExpression + *pPos);
	}

	//2.
	while ((cCurr >= '0' && cCurr <= '9') || cCurr == '.') //找数字的后一个字符
	{
		if (*(pExpression + *pPos) == '.')
		{
			cPointCnt += 1;
		}
        else
        {
            digitCnt++;   // ***新增：统计数字***
        }

		if (ucIdx >= MAX_LENGTH_OF_LEXICAL_UNIT)
		{
			setErrorType(GCODE_ERROR_NUM85);
			findError(pExpression, unLineNum);
			return -1;
		}
		cTempArr[ucIdx++] = cCurr;  //保存到数组中

		*pPos += 1;
		cCurr = *(pExpression + *pPos);
	}

	if (cPointCnt > 1)
	{
		setErrorType(GCODE_ERROR_NUM84);
		findError(pExpression, unLineNum);
		return -1;
	}

    // 4) ***至少一位数字校验（核心）***
    if (digitCnt == 0)
    {
        setErrorType(GCODE_ERROR_NUM84);           // 或者你们更合适的“数字格式错误”码
        findError(pExpression + *pPos, unLineNum); // 指向非法字面量位置
        return -1;
    }

	if (ucIdx >= MAX_LENGTH_OF_LEXICAL_UNIT)
	{
		setErrorType(GCODE_ERROR_NUM85);
		findError(pExpression, unLineNum);
		return -1;
	}
	cTempArr[ucIdx] = 0;
	*pOutput = atof(cTempArr);

	return 0;
}


/*************************************************
  函数名称：int8 isUnaryOrBinary(uint8 cLevel)
  函数功能：
  输入参数：cLevel	运算符优先级
  输出参数：
  返 回 值：1	单目
			2	双目
  备    注：所有单目运算符的优先级为9或10
**************************************************/
int8 isUnaryOrBinary(uint8 cLevel)
{
	if (cLevel == MACRO_LEVEL_9 || cLevel == MACRO_LEVEL_10)
	{
		return 1;  //单目
	}
	return 2;  //双目
}


/*************************************************
  函数名称：static void getLeftAndRightVal(SLocalMacroVar* pSLocalMacroVar, SCalculateUnit* pSLeftInputUnit, SCalculateUnit* pSRightInputUnit
			, float* pLeftVal, float* pRightVal, uint32 unLineNum)
  函数功能  获取计算单元的值
  输入参数：pSLocalMacroVar		局部宏变量结构体地址（宏变量需要用到）
			pCalculateUnit		计算单元结构体
			pUnitVal			计算单元值（用于输出）
			unLineNum			行号，用于报错
  输出参数：
  返 回 值：-1	错误
			1	已获取计算单元的操作数
			0	宏变量为NULL
  备    注：1.计算单元带#，则pUnitVal为解引用后的值
			2.计算单元不带#
			获取的值为已解引用后的值，一旦调用此函数，计算单元的#号会被清除
			pCalculateUnit的fVal值:1  bPound:TRUE (对应#1的值为10)   ---->   *pUnitVal被赋值为10，fVal值:1  bPound:FALSE
**************************************************/
static int8 getUnitVal(SLocalMacroVar* pSLocalMacroVar, SCalculateUnit* pCalculateUnit, float* pUnitVal, uint32 unLineNum)
{
	if (pSLocalMacroVar == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	if (pCalculateUnit == NULL || pUnitVal == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	//计算单元中fVal需要解引用
	if (pCalculateUnit->bPound == TRUE)
	{
		float fTemp;			//接局部宏变量
		uint8* pTemp2 = NULL;	//接系统宏变量
		int8 cRet;
		cRet = getMacroVar(pSLocalMacroVar, (uint16)pCalculateUnit->fVal, &fTemp, &pTemp2, unLineNum);  //获取左值的地址
		if(cRet < 0)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}
		//为局部宏变量
		if (pCalculateUnit->fVal <= 33)
		{
			*pUnitVal = fTemp;
		}
		//为系统宏变量
		else if (pCalculateUnit->fVal >= 1000)
		{
			*pUnitVal = *pTemp2;//???用浮点型接整形
		}
		pCalculateUnit->bPound = FALSE;  //消除#号标记
		return cRet;
	}
	//计算单元fVal不需解引用
	else
	{
		*pUnitVal = pCalculateUnit->fVal;
		return 1;
	}
}

/*************************************************
  函数名称：static void getMacroVar(uint16 usTargetMacroVarName,float **pOutput, uint32 unLineNum, uint8 ucChannelNum, uint8 ucSyntaxOrAssign)
  函数功能  读取变量#usTargetMacroVarName对应的值
  输入参数：pSLocalMacroVar			局部宏变量结构体地址
			usTargetMacroVarName	目标宏变量名
			pOutput					局部宏变量名的值（浮点型，用于输出）
			pOutput2				系统宏变量名的值（整形，用于输出）
			unLineNum				行号，用于报错
  输出参数：
  返 回 值：1	宏变量不为NULL
			0	宏变量为NULL
			-1	错误
  备    注：1.获取的宏变量为NULL，输出0。计算时NULL按0处理，其他情况需要按NULL处理，
			2.不要修改pOutput的类型
**************************************************/
 int8 getMacroVar(SLocalMacroVar* pSLocalMacroVar, uint16 usTargetMacroVarName, float* pOutput, uint8** pOutput2, uint32 unLineNum)
{
	if (pSLocalMacroVar == NULL && pOutput == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	if (usTargetMacroVarName > 33 && usTargetMacroVarName < 1000)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
		return -1;
	}
	//局部宏变量#1-#33
	if (usTargetMacroVarName >= 0 && usTargetMacroVarName <= 33)
	{
		//#0 
		if (usTargetMacroVarName == 0)  
		{
			*pOutput = 0;
			return 0;
		}
		//#1-#33
		else    
		{
			//#i为NULL
			if (GETBIT64(pSLocalMacroVar->ulIsNull, (usTargetMacroVarName - 1)) == 1)
			{
				*pOutput = 0;  
				return 0;
			}
			//#i不为NULL
			else
			{
				*pOutput = pSLocalMacroVar->dValArr[usTargetMacroVarName - 1];
				return 1;
			}
		}
	}

	//系统宏变量#1000-#1037
	else if (usTargetMacroVarName >= 1000 && usTargetMacroVarName <= 1037)
	{
		uint16 Index = usTargetMacroVarName;
		if (Index % 10 == 8 || Index % 10 == 9)//尾数不能为8、9 ???
		{
			parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
			return -1;
		}
		Index %= 100;//取后面两位数
		if (Index >= 10 && Index <= 17)
		{
			Index -= 2;
		}
		else if (Index >= 20 && Index <= 27)
		{
			Index -= 4;
		}
		else if (Index >= 30 && Index <= 37)
		{
			Index -= 6;
		}
		else if (Index >= 0 && Index <= 7)
		{
		}
		//	printf("%p", (g_SystemMacro.MarcoAdd[Index]));
		*pOutput2 = g_SystemMacro.MarcoAdd[Index];
		return 1;
	}
	else
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：static int8 setMacroVar(uint16 unTarget,float fVal)
  函数功能  将值fVal写入宏变量#usTargetMacroVarName中
  输入参数：pSLocalMacroVar			局部宏变量结构体地址
			usTargetMacroVarName	目标写入宏变量名
			fVal					目标写入值
			
			unLineNum			行号，用于报错
  输出参数：
  返 回 值：
  备    注：仅宏变量#1-#33可进行赋值
**************************************************/
 void setMacroVar(SLocalMacroVar* pSLocalMacroVar, uint16 usTargetMacroVarName, const float* pVal, uint32 unLineNum) ///暂时只做局部变量，全局变量应该语法检查和赋值应该要分开!!!!
{
	//SProgram* pCurrProgram = NULL;   //当前程序结构体
	if (pSLocalMacroVar == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	//只能对宏变量#1-#33进行赋值
	if (usTargetMacroVarName > 33 || usTargetMacroVarName < 1)  //不能对#0写入
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM131);
		return;
	}

	if (pVal == NULL)
	{
		pSLocalMacroVar->dValArr[usTargetMacroVarName - 1] = 0;   //值置0
		SETBIT64(pSLocalMacroVar->ulIsNull, (usTargetMacroVarName - 1));  //标记位置1

	}
	else
	{
		pSLocalMacroVar->dValArr[usTargetMacroVarName - 1] = *pVal;
		CLRBIT64(pSLocalMacroVar->ulIsNull, (usTargetMacroVarName - 1));  //标记位置0
	}
}


 // 返回 TRUE 表示：完全匹配关键字，且关键字后不是 [A-Za-z0-9_]
 static bool isTagetStr_A_N(const char* kw, uint8 kwLen,
                            const int8* p, uint8 remainLen)
 {
     if (!kw || !p) return FALSE;
     if (remainLen < kwLen) return FALSE;

     // 1) 关键字本体完全匹配（只读 remainLen 范围内）
     for (uint8 i = 0; i < kwLen; i++)
     {
         if ((char)p[i] != kw[i]) return FALSE;
     }

     // 2) 关键字后边界校验：避免 ANDX / ABSX
     // remainLen == kwLen：刚好到末尾，合法
     if (remainLen == kwLen) return TRUE;

     char c = (char)p[kwLen];
     if ((c >= 'A' && c <= 'Z') ||
         (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') ||
         c == '_')
     {
         return FALSE;
     }

     return TRUE;
 }

/*************************************************
  函数名称：static int8 whichAWord(const int8  *pCurrPtr, uint32 unLineNum)
  函数功能: 判断是哪一个以A开头的操作符
  输入参数：pCurrPtr	当前字符的地址
			unLineNum	行号，用于报错
  输出参数：
  返 回 值：1	AND
			2	ABS  
			3	ASIN 
			4	ACOS 
			5	ATAN
			-1	空指针错误
			-2	运算符错误
  备    注：由于首字符为A的运算符较多，进行统一管理
**************************************************/

static int8 whichAWord(const int8* pCurrPtr, uint8 ucRemainLen ,uint32 unLineNum)
{
	if (!pCurrPtr)
	{
		parseGCodeErrorB(unLineNum, GCODE_ERROR_NUM300);
		return -1;
	}

    // 注意：这里依赖“带长度&边界”的 isTagetStr_A_N（见下文第2节）
    if (isTagetStr_A_N("AND", 3, pCurrPtr, ucRemainLen))
		return 1;
    if (isTagetStr_A_N("ABS", 3, pCurrPtr, ucRemainLen))
		return 2;
    if (isTagetStr_A_N("ASIN", 4, pCurrPtr, ucRemainLen))
		return 3;
    if (isTagetStr_A_N("ACOS", 4, pCurrPtr, ucRemainLen))
		return 4;
    if (isTagetStr_A_N("ATAN", 4, pCurrPtr, ucRemainLen))
		return 5;

		return -2;
}


/*************************************************
  函数名称：
  函数功能: 打印后缀表达式
  输入参数：
			unLineNum	行号，用于报错
  输出参数：
  返 回 值：1	AND
			2	ABS
			3	ASIN
			4	ACOS
			5	ATAN
			-1	空指针错误
			-2	运算符错误
  备    注：由于首字符为A的运算符较多，进行统一管理
**************************************************/
//static void printCalculateUnitList(SList* pList)
//{
//	int i=0;
//	for (i; i < pList->cSize; i++)
//	{
//		SCalculateUnit* pCurrUnit = (SCalculateUnit*)FindNodeData(pList, i);
//		printf("当前节点：%d\n", i);
//		printf("单元等级：%d\n", (int32)pCurrUnit->ucLevel);
//		//操作数
//		if (pCurrUnit->ucLevel == 12)
//		{
//			printf("数值：%f\n", pCurrUnit->fVal);
//
//		}
//		//操作符
//		else
//		{
//			printf("单元类型：%d\n", (int32)pCurrUnit->ucType);
//		}
//		printf("\n");
//	}
//
//}

#if GCodeCalcExpression_Debug
//打印逆波兰
void printReversePolishExpression(SList* pList)  //参数是逆波兰链表
{
	printf("逆波兰:");
	int pos = 0;
	while (pos < pList->cSize)
	{
		SCalculateUnit* pUnit = (SCalculateUnit*)FindNodeData(pList, pos);
		int level = pUnit->ucLevel;
		int type = pUnit->ucType;

		switch (level)
		{
		case MACRO_LEVEL_0:
		{
			printf("[");
			break;
		}
		case MACRO_LEVEL_1:
		{
			printf("=");
			break;
		}
		case MACRO_LEVEL_2:
		{
			printf("&&");
			break;
		}
		case MACRO_LEVEL_3:
		{
			printf("^");
			break;
		}
		case MACRO_LEVEL_4:
		{
			printf("||");
			break;
		}
		case MACRO_LEVEL_5:
		{
			switch (type)
			{
			case 0:
			{
				printf("<>");
				break;
			}
			case 1:
			{
				printf("==");
				break;
			}
			default:
				printf("逆波兰错误");
				break;
			}

			break;
		}

		case MACRO_LEVEL_6:
		{
			switch (type)
			{
			case 0:
				printf(">");
				break;
			case 1:
				printf(">=");
				break;
			case 2:
				printf("<");
				break;
			case 3:
				printf("<=");
				break;
			default:
				printf("逆波兰错误");
				break;
			}
			break;
		}

		case MACRO_LEVEL_7:
		{
			switch (type)
			{
			case 0:
				printf("+");
				break;
			case 1:
				printf("-");
				break;
			default:
				printf("逆波兰错误");
				break;
			}
			break;
		}




		case MACRO_LEVEL_8:
		{
			switch (type)
			{
			case 0:
				printf("*");
				break;
			case 1:
				printf("/");
				break;
			case 2:
				printf("%%");
				break;
			default:
				printf("逆波兰错误");
				break;
			}
			break;
		}


		case MACRO_LEVEL_9:
		{
			switch (type)
			{
			case 0:
				printf("+");
				break;
			case 1:
				printf("-");
				break;

			case 3:
				printf(" SQRT");
				break;
			case 4:
				printf(" ABS");
				break;
			case 5:
				printf(" ROUND");
			case 6:
				printf(" FUP");
				break;
			case 7:
				printf(" FIX");
				break;
			case 8:
				printf("LN");
				break;
			case 9:
				printf(" EXP");
				break;
			case 10:
				printf(" SIN");
				break;
			case 11:
				printf(" ASIN");
				break;
			case 12:
				printf(" COS");
				break;
			case 13:
				printf(" ACOS");
				break;
			case 14:
				printf(" TAN");
				break;
			case 15:
				printf(" ATAN");
				break;
			case 16:
				printf(" BIN");
				break;
			case 17:
				printf(" BCD");
				break;
			default:
				printf("逆波兰未添加");
				break;
			}
			break;
		}
		case MACRO_LEVEL_10:
			printf("#");
			break;
		case MACRO_LEVEL_11:
		{
			printf("]");
			break;
		}
		case MACRO_LEVEL_12:
		{
			printf("%f", pUnit->fVal);
			break;
		}

		default:
			printf("逆波兰未添加");
			break;
		}

		pos++;
	}
	printf("\n");
}

#endif
