#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"

int16 sBeforeG8xCommand_Check;		//保存运行G83~G88前的运动模态指令(检查使用)
SGCommandRegulation* pGaBeforeG8xRegulation_Check; //保存运行G83~G88前的运动模态指令的规则地址(检查使用)
int16 sBeforeG8xCommand_Assign;     //保存运行G83~G88前的模态(赋值使用)
SGCommandRegulation* pGaBeforeG8xRegulation_Assign; //保存运行G83~G88前的运动模态指令的规则地址(赋值使用)

//添加G80规则结构体
void addG80RegulationAPI()
{
	//指令：G80
	//组别：sGx
	//指令占程序段数：1
	addGCommadRegulation(80, sGx, 1, NULL, NULL, NULL, CheckG80, AssignG80);
}

/*************************************************
  函数名称：int8 AssignG80(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：赋值G80
  输入参数：struct _SAssignConfiguration* pSAssignConfig		赋值结构体指针
			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG80(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig==NULL||pGCodeBufUnit==NULL)
	{
		return -1;
	}

	int16 sLastGa;  //上一条程序段的sGa
	SGCommandRegulation* pSLastGaRegulation;//当前的sGa规则地址

	int16 sCurGa;//当前程序段的sGa
	SGCommandRegulation* pSCurGaRegulation;//当前的sGa规则地址


	int16 sBeforeG8xCommand=UNASSIGNED_VALUE;//G83~G88钻孔指令集执行前的运动模态
	SGCommandRegulation* pSGaRegulationBeforeG8x=NULL;//G83~G88钻孔指令集执行前的运动指令的规则地址

    //1.获取上一条程序段的运动组模态指令
	sLastGa = pSAssignConfig->SGCodeCurrSysState.SModalInfo.sGa;//获取系统状态结构体中的当前运动Ga组模态
	pSLastGaRegulation = pSAssignConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;//获取系统状态结构体中的当前运动Ga组模态指令的规则地址

	//2.获取当前程序段的运动模态指令
	sCurGa = pSAssignConfig->sCurrCommandVal;
	pSCurGaRegulation = pSAssignConfig->pSCurrCommandRegulation;


    //2.上一条程序段的运动模态为钻孔指令（目前的钻孔指令集中在G80~G89，如果后面添加的钻孔指令超过此范围，下面的条件就需要修改）
    //   并且当前程序段的运动模态也是钻孔指令（比如 G80 G1时，G80不生效）
	if (((sLastGa >G80* pow(10, GCOMMAND_DECIMAL_PLACE))&&(sLastGa <G90* pow(10, GCOMMAND_DECIMAL_PLACE)))&&
		((sCurGa > G80 * pow(10, GCOMMAND_DECIMAL_PLACE)) && (sCurGa < G90 * pow(10, GCOMMAND_DECIMAL_PLACE))))
	{



		//2.1获取执行钻孔指令前的运动模态
		 getGaBeforeG8x(GC_ASSIGN, &sBeforeG8xCommand, &pSGaRegulationBeforeG8x);
	
		if (pSGaRegulationBeforeG8x == NULL)
		{
	       //不存在此功能
			parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM87);
			return -1;
		}

		//2.2根据执行钻孔指令前的运动模态对应的规则结构体中是否有开辟相关参数结构体的函数来开辟参数结构体
		//执行钻孔指令前的运动模态对应的规则结构体中有参数结构体开辟函数
		if (pSGaRegulationBeforeG8x->pMallocParaStructAPI!=NULL)
		{
			if (pSGaRegulationBeforeG8x->pMallocParaStructAPI(GC_ASSIGN, pSAssignConfig->ucChannelNum) <= 0)
			{
				parseGCodeErrorB(pSAssignConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
				return -1;
			}
		}

		//2.3释放钻孔指令的参数结构体
		if (pSLastGaRegulation->pFreeParaStructAPI != NULL)
		{
			pSLastGaRegulation->pFreeParaStructAPI(GC_ASSIGN, pSAssignConfig->ucChannelNum);
		}

	
		//2.4将当前的运动组模态以及规则地址修改为执行钻孔指令前的运动指令的
       pSAssignConfig->sCurrCommandVal = sBeforeG8xCommand;
       pSAssignConfig->pSCurrCommandRegulation= pSGaRegulationBeforeG8x;
		
		return 2;
	}
}

/*************************************************
  函数名称：int8 CheckG80(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：检查处理G80
  输入参数：struct _SSyntaxConfiguration* pSSyntaxConfig		检查结构体指针
			struct SGCodeParse* pGCodeBufUnit              G 代码解析后结构体指针
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 CheckG80(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
     if (pSSyntaxConfig == NULL || pGCodeBufUnit == NULL)
     {
	   return -1;
     }

	int16 sLastGa;  //上一条程序段的sGa
	SGCommandRegulation* pSLastGaRegulation;//当前的sGa规则地址

	int16 sCurGa;//当前程序段的sGa
	SGCommandRegulation* pSCurGaRegulation;//当前的sGa规则地址

	int16 sBeforeG8xCommand = UNASSIGNED_VALUE;//G83~G88钻孔指令集执行前的运动模态
	SGCommandRegulation* pSGaRegulationBeforeG8x = NULL;//G83~G88钻孔指令集执行前的运动指令的规则地址

	 //1.获取上一条程序段的运动组模态指令
	sLastGa = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.sGa;//获取系统状态结构体中的当前运动Ga组模态
	pSLastGaRegulation = pSSyntaxConfig->SGCodeCurrSysState.SModalInfo.pSGaRegulation;//获取系统状态结构体中的当前运动Ga组模态指令的规则地址

	//2.获取当前程序段的运动模态指令
	sCurGa = pSSyntaxConfig->sCurrCommandVal;
	pSCurGaRegulation = pSSyntaxConfig->pSCurrCommandRegulation;


	//2.当前的运动模态为钻孔指令（目前的钻孔指令集中在G80~G89，如果后面添加的钻孔指令超过此范围，下面的条件就需要修改）
	if (((sLastGa > G80 * pow(10, GCOMMAND_DECIMAL_PLACE)) && (sLastGa < G90 * pow(10, GCOMMAND_DECIMAL_PLACE))) &&
		((sCurGa > G80 * pow(10, GCOMMAND_DECIMAL_PLACE)) && (sCurGa < G90 * pow(10, GCOMMAND_DECIMAL_PLACE))))
	{



		//2.1获取执行钻孔指令前的运动模态
		getGaBeforeG8x(GC_SYNTAX, &sBeforeG8xCommand, &pSGaRegulationBeforeG8x);

		if (pSGaRegulationBeforeG8x == NULL)
		{
			//不存在此功能
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM87);
			return -1;
		}

		//2.2根据执行钻孔指令前的运动模态对应的规则结构体中是否有开辟相关参数结构体的函数来开辟参数结构体
		//执行钻孔指令前的运动模态对应的规则结构体中有参数结构体开辟函数
		if (pSGaRegulationBeforeG8x->pMallocParaStructAPI != NULL)
		{
			if (pSGaRegulationBeforeG8x->pMallocParaStructAPI(GC_SYNTAX, pSSyntaxConfig->ucChannelNum) <= 0)
			{
				parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM300);
				return -1;
			}
		}

		//2.3释放钻孔指令的参数结构体
		if (pSLastGaRegulation->pFreeParaStructAPI != NULL)
		{
			pSLastGaRegulation->pFreeParaStructAPI(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);
		}


		//2.4将当前的运动组模态以及规则地址修改为执行钻孔指令前的运动指令的
		pSSyntaxConfig->sCurrCommandVal = sBeforeG8xCommand;
		pSSyntaxConfig->pSCurrCommandRegulation = pSGaRegulationBeforeG8x;

		return 2;
	}
}


/*************************************************
  函数名称：void setGaBeforeG8x(uint8 ucMode, int16 sGaBeforeG8x)
  函数功能：记录执行钻孔指令前的运动模态指令
  输入参数：uint8 ucMode	 模式（检查or赋值）
            uint8 ucChannelNum    通道号
			int16 sGaBeforeG8x    执行钻孔指令前的运动模态指令 
			SGCommandRegulation* pGaBeforeG8xRegulation    执行钻孔指令前的运动模态指令 的规则地址
  输出参数： 
  返 回 值：
  备    注：
**************************************************/
void setGaBeforeG8x(uint8 ucMode, uint8 ucChannelNum,int16 sGaBeforeG8x, SGCommandRegulation* pGaBeforeG8xRegulation)
{

	if (ucMode == GC_SYNTAX)
	{
		//获取检查结构体
		SSyntaxConfiguration* pSyntaxConfig = getSyntaxConfig(ucChannelNum);
		int16 sLastGa= pSyntaxConfig->SGCodeCurrSysState.SModalInfo.sGa;  //上一条程序段的sGa
	    //判断上一个运动模态指令不是G83~G89（钻孔指令），就将执行G83~G89之前的运动模态记录进去。
		if (sLastGa< G80 * pow(10, GCOMMAND_DECIMAL_PLACE)|| sLastGa>= G90 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			//记录
			sBeforeG8xCommand_Check = sGaBeforeG8x;
			pGaBeforeG8xRegulation_Check = pGaBeforeG8xRegulation;
		}

	}
	else if (ucMode == GC_ASSIGN)
	{
		SAssignConfiguration* pAssignConfig = getAssignConfig(ucChannelNum);
		int16 sLastGa = pAssignConfig->SGCodeCurrSysState.SModalInfo.sGa;  //上一条程序段的sGa
		//判断上一个运动模态指令不是G83~G89（钻孔指令），就将执行G83~G89之前的运动模态记录进去。
		if (sLastGa < G80 * pow(10, GCOMMAND_DECIMAL_PLACE) || sLastGa >= G90 * pow(10, GCOMMAND_DECIMAL_PLACE))
		{
			//记录
			sBeforeG8xCommand_Assign = sGaBeforeG8x;
			pGaBeforeG8xRegulation_Assign = pGaBeforeG8xRegulation;
		}

	}
	return;
}








/*************************************************
  函数名称：int16 getGaBeforeG8x(uint8 ucMode)
  函数功能：获取执行钻孔指令前的运动模态指令及其规则地址
  输入参数：uint8 ucMode	 模式（检查or赋值）
	
  输出参数：
  返 回 值：* sBeforeG8x 执行钻孔指令前的运动模态指令
            * pGaBeforeG8xRegulation  执行钻孔指令前的运动模态指令的规则地址
  备    注：
**************************************************/
void getGaBeforeG8x(uint8 ucMode, int16* sBeforeG8x, SGCommandRegulation** pGaBeforeG8xRegulation)
{
	if (ucMode == GC_SYNTAX)
	{
		*sBeforeG8x = sBeforeG8xCommand_Check;
		//用完取消sBeforeG8xCommand中记录的变量
		sBeforeG8xCommand_Check = UNASSIGNED_VALUE;

		*pGaBeforeG8xRegulation = pGaBeforeG8xRegulation_Check;
		pGaBeforeG8xRegulation_Check = NULL;

	}
	else if (ucMode == GC_ASSIGN)
	{
		*sBeforeG8x = sBeforeG8xCommand_Assign;
		//用完取消sBeforeG8xCommand中记录的变量
		sBeforeG8xCommand_Assign = UNASSIGNED_VALUE;

		*pGaBeforeG8xRegulation = pGaBeforeG8xRegulation_Assign;
		pGaBeforeG8xRegulation_Assign = NULL;
	}

}


