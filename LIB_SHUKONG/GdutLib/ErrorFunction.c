#include<string.h>
#include<stdio.h>

#include"ErrorFunction.h"
#include "Config.h"
//#include"IOCommand.h"
#if STM32_MODE
#include"ram_mu.h"
#include "app_warning.h"
#endif
static uint16 s_ErrorType;                //记录错误的类型
static bool s_bErrorFlag;                 //单个词法单元出错的标志
static bool s_bErrorHasBeenWritten;            //已有错误被写入错误缓冲区的标志
static uint8 s_ucPosSGCodeErrorBuf;       //G代码错误信息结构体数组的当前位置
static struct SGCodeError s_SGCodeErrorBuf[GCODEERRORBUF_MAX];  //解析G代码出错时，保存当前G代码错误信息的结构体数组
static char s_cStrArray[SIZE_OF_ERROR_STRING + 1];       //用于返回字符串
#if STM32_MODE
static u8 erbuff[SIZE_OF_ERROR_STRING];
extern u32 warn_num;//报整页显示链表:
extern current_warning* c_warn;			    //当前故障
extern historical_warning* h_warn;			//历史故障
#endif

/*************************************************
  函数名称：void printError()
  函数功能  错误打印函数
  输入参数：
  输出参数：
  返 回 值：


  备    注：
**************************************************/
#if !STM32_MODE
void printError(void)
{
	int i;
	printf("--------------------ERROR INFOMATION -----------------------------\n");
	for (i = 0; i < s_ucPosSGCodeErrorBuf; ++i)
	{
		/*printf("g_SGCodeErrorBuf[i].usErrorCode:%d\n", g_SGCodeErrorBuf[i].usErrorCode);*/
		printf("Type:%4d   LineNumber:%6d   ErrorCode:%5s  Info:%s\n", s_SGCodeErrorBuf[i].usErrorCode, s_SGCodeErrorBuf[i].unLineNum + 1, s_SGCodeErrorBuf[i].ucErrorGCode, getStrByNum(s_SGCodeErrorBuf[i].usErrorCode, NULL, NULL));
	}

}
#endif

/*************************************************
  函数名称：void initErrorBuf()
  函数功能: 初始化错误缓冲区及相关变量
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void initErrorBuf()
{
	s_ErrorType = -1;                     //记录错误的类型
	s_bErrorFlag = FALSE;                 //单个词法单元出错的标志
	s_bErrorHasBeenWritten = FALSE;            ///已有错误被写入错误缓冲区的标志
	s_ucPosSGCodeErrorBuf = 0;            ///G代码错误信息结构体数组的当前位置
	memset(s_SGCodeErrorBuf, 0, sizeof(struct SGCodeError) * GCODEERRORBUF_MAX);

}


/*************************************************
  函数名称：parseGCodeError(const char *pGCode, uint32 unLineNum, uint16 usErrorNum, uint8 ucWord_Len)
  函数功能  解析G代码出错时，将行号与正在解析的G代码保存到g_SGCodeErrorBuf数组中
  输入参数：pGCode指向正在解析的G代码，unLineNum为正在解析的行号,usErrorNum错误代码编号，ucWord_Len词法单元长度
  输出参数：
  返 回 值：0：函数已经正确处理完成
			1：????????
			2：？？？
  备    注：G代码的错误代码也在该函数中生成，并保存在g_SGCodeErrorBuf数组中
**************************************************/
uint8 parseGCodeError(const char *pGCode, uint32 unLineNum, uint8 ucWord_Len)
{
	//uint8 ucLoopCount;
	int i;
	if (s_ucPosSGCodeErrorBuf >= GCODEERRORBUF_MAX)    //错误缓存区满，不再存放
	{
		s_bErrorHasBeenWritten = TRUE;
		return FALSE;
	}

	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].unLineNum = unLineNum;
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].usErrorCode = s_ErrorType;
	//printf("g_SGCodeErrorBuf[i].usErrorCode:%d\tusErrorNum:%d\n", g_SGCodeErrorBuf[ucPosSGCodeErrorBuf].usErrorCode, usErrorNum);
	ucWord_Len = ucWord_Len > ErrorGCode_MAX ? ErrorGCode_MAX : ucWord_Len;
	//ucLoopCount = ucWord_Len > ErrorGCode_MAX ? ErrorGCode_MAX : ucWord_Len;    //取最小长度，避免溢出或访问越界单元
	for (i = 0; i < ucWord_Len; ++i)
	{
		s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].ucErrorGCode[i] = *(pGCode + i);
	}
	s_ucPosSGCodeErrorBuf++;

	return TRUE;
}
/*************************************************
  函数名称：uint8 parseGCodeErrorB(uint32 unLineNum, uint16 usErrorNum)
  函数功能  处理其他类型错误
  输入参数：unLineNum为正在解析的行号,usErrorNum错误代码编号
  输出参数：
  返 回 值：0：函数已经正确处理完成
			1：已满
			2：？？？
  备    注：G代码的错误代码也在该函数中生成，并保存在g_SGCodeErrorBuf数组中
**************************************************/
uint8 parseGCodeErrorB(uint32 unLineNum, uint16 usErrorNum)
{
	if (s_ucPosSGCodeErrorBuf >= GCODEERRORBUF_MAX)    //错误缓存区满，不再存放
	{
		s_bErrorHasBeenWritten = TRUE;
		return 1;
	}
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].unLineNum = unLineNum;
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].usErrorCode = usErrorNum;
	s_bErrorHasBeenWritten = TRUE;
#if STM32_MODE
	getStrByWord(usErrorNum, s_ucPosSGCodeErrorBuf, 2);   //G代码错误信息
#endif
	s_ucPosSGCodeErrorBuf++;
#if EN_GCODE_DEBUG
	printf("parseGCodeErrorB第%d个错误\n", g_ucPosSGCodeErrorBuf);
#endif
	return TRUE;
}

/*************************************************
  函数名称：uint8 parseGCodeErrorB(uint32 unLineNum, uint16 usErrorNum)
  函数功能  处理其他类型错误
  输入参数：unLineNum为正在解析的行号,usErrorNum错误代码编号
  输出参数：
  返 回 值：0：函数已经正确处理完成
			1：已满
			2：？？？
  备    注：CNC说明书中的PLC错误，解析的行号填9999
**************************************************/
uint8 parseErrorC(uint32 unLineNum, uint16 usErrorNum)
{
	if (s_ucPosSGCodeErrorBuf >= GCODEERRORBUF_MAX)    //错误缓存区满，不再存放
	{
		s_bErrorHasBeenWritten = TRUE;
		return 1;
	}
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].unLineNum = unLineNum;
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].usErrorCode = usErrorNum;
	s_bErrorHasBeenWritten = TRUE;

	s_ucPosSGCodeErrorBuf++;
#if EN_GCODE_DEBUG
	printf("parseGCodeErrorB第%d个错误\n", g_ucPosSGCodeErrorBuf);
#endif
	return TRUE;
}

/*************************************************
  函数名称：parseRunningErrorC(uint32 unLineNum, uint16 usErrorNum)
  函数功能：中断中运行中报错
  输入参数：unLineNum为正在解析的行号,usErrorNum错误代码编号
  输出参数：
  返 回 值：0：函数已经正确处理完成
			1：已满
			2：？？？
  备    注：
**************************************************/
uint8 parseRunningError(uint32 unLineNum, uint16 usErrorNum, uint32 ucIOIndex)
{
	if (s_ucPosSGCodeErrorBuf >= GCODEERRORBUF_MAX)    //错误缓存区满，不再存放
	{
		return 1;
	}

	//暂时先用G代码的缓冲区
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].unLineNum = unLineNum;
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].usErrorCode = usErrorNum;
	s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].ucIOIndex = ucIOIndex;
	s_bErrorHasBeenWritten = TRUE;
#if STM32_MODE
	getStrByWord(usErrorNum, s_ucPosSGCodeErrorBuf, 2);
#endif
	s_ucPosSGCodeErrorBuf++;
#if EN_GCODE_DEBUG
	printf("parseGCodeErrorB第%d个错误\n", g_ucPosSGCodeErrorBuf);
#endif
	return TRUE;
}
#if STM32_MODE
/*************************************************
  函数名称：int8 getStrByWord(uint16 sNum,uint16 c_num, uint16 mode)
  函数功能: 1.显示当前故障页和历史故障页的错误 2.根据错误编号传递IO端口号
  输入参数：sNum为错误编号，c_num若干错误排序序号，mode故障页
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int16 getStrByWord(uint16 sNum, uint16 c_num, uint16 mode)
{
	if (mode == 0)//当前故障
		return c_warn->warn_word[c_num];
	else if(mode == 1)//历史故障
		return h_warn->warn_word[c_num];
	else
	{
		switch (sNum)
		{
			case 31:
				c_warn->warn_word[c_num] = s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].ucIOIndex;
				break;
			
			case 101:
				c_warn->warn_word[c_num] = warn_num;
				break;

			case 406:
				c_warn->warn_word[c_num] = warn_num;
				break;
				
			default:
				break;
		}
		
	}
}
#endif
/*************************************************
  函数名称：int8 *getStrByNum(uint16 sNum, uint16 c_num, uint16 mode)
  函数功能: 返回错误号相对的内容
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 *getStrByNum(uint16 sNum, uint16 c_num, uint16 mode)
{
	int ver;
	memset(s_cStrArray, 0, SIZE_OF_ERROR_STRING);
	switch (sNum)
	{
	case 1:
		strncpy(s_cStrArray, "急停报警!急停按钮已按下或未接急停按钮!", SIZE_OF_ERROR_STRING); //急停报警!急停按钮已按下或未接急停按钮!
		break;
	//软限位报警 朱稳中 2023.6.16
	case 29:
		strncpy(s_cStrArray, " M35功能等待时间超时", SIZE_OF_ERROR_STRING); //急停报警!急停按钮已按下或未接急停按钮!
		break;
	case 30:
		strncpy(s_cStrArray, " 输入端口错误", SIZE_OF_ERROR_STRING);
		break;
	case 31:
		strncpy(s_cStrArray, "输入", 4);
#if STM32_MODE
		ver = getStrByWord(sNum, c_num, mode);
#else
		ver = s_SGCodeErrorBuf[s_ucPosSGCodeErrorBuf].ucIOIndex;
#endif
		sprintf(s_cStrArray + 4, "%d ", ver);
		strncpy(s_cStrArray + 6, "端口检测超时", SIZE_OF_ERROR_STRING);
		break;
	case 41:
	case 42:
	case 43:
		sNum += 'X' - 'A' + 3;
		//s_cStrArray[0] = 'X'+ sNum-41;
		//strncpy(s_cStrArray+1, "轴超出正向软件限位!", SIZE_OF_ERROR_STRING);
		//break;
	case 44:
	case 45:
	case 46:
		s_cStrArray[0] = 'A' + sNum - 44;
		strncpy(s_cStrArray + 1, "轴超出正向软件限位!", SIZE_OF_ERROR_STRING);
		break;
	case 47:
	case 48:
	case 49:
		sNum += 'X' - 'A' + 3;
		//s_cStrArray[0] = 'X' + sNum - 47;
		//strncpy(s_cStrArray + 1, "轴超出负向软件限位!", SIZE_OF_ERROR_STRING);
		//break;
	case 50:
	case 51:
	case 52:
		s_cStrArray[0] = 'A' + sNum - 50;
		strncpy(s_cStrArray + 1, "轴超出负向软件限位!", SIZE_OF_ERROR_STRING);
		break;




	case 81:strncpy(s_cStrArray, "代码地址输入错误", SIZE_OF_ERROR_STRING);
		break;
	case 82:strncpy(s_cStrArray, "段号不能有负值或小数点", SIZE_OF_ERROR_STRING);
		break;
	case 83:strncpy(s_cStrArray, ":G-代码不能有负数", SIZE_OF_ERROR_STRING);
		break;
	case 84:strncpy(s_cStrArray, "程序输入的代码值非法", SIZE_OF_ERROR_STRING);
		break;
	case 85:strncpy(s_cStrArray, "代码字的字符数不能小于2或大于11", SIZE_OF_ERROR_STRING);
		break;
		//case 
		// :strncpy(cStrArray, "", SIZE_OF_ERROR_STRING);
		//	break;
	case 87:strncpy(s_cStrArray, "输入了不支持的G代码", SIZE_OF_ERROR_STRING);
		break;
	case 88:strncpy(s_cStrArray, "程序段第一行或最后一行缺少标识符%号，%必须是一行的第一个字符", SIZE_OF_ERROR_STRING);
		break;
	case 89:strncpy(s_cStrArray, "要读取的程序不存在或打开失败", SIZE_OF_ERROR_STRING);
		break;
		/*case 90:
			break;*/
	case 91:strncpy(s_cStrArray, "读取程序段内容时出错", SIZE_OF_ERROR_STRING);
		break;
	case 92:strncpy(s_cStrArray, "手动输入方式下不能执行M98, M99指令", SIZE_OF_ERROR_STRING);
		break;
	case 93:strncpy(s_cStrArray, "搜索不到指定的程序段号", SIZE_OF_ERROR_STRING);
		break;
	case 94:strncpy(s_cStrArray, "子程序的嵌套层数超过最大限制", SIZE_OF_ERROR_STRING);
		break;
	case 95:strncpy(s_cStrArray, "G75指令代码格式不正确", SIZE_OF_ERROR_STRING);
		break;
	case 96:strncpy(s_cStrArray, "G74指令代码格式不正确", SIZE_OF_ERROR_STRING);
		break;
	case 97:strncpy(s_cStrArray, "G70～G73代码中精加工程序段超出限制", SIZE_OF_ERROR_STRING);
		break;
	case 98:strncpy(s_cStrArray, "圆弧参数R,I,K至少输入一个", SIZE_OF_ERROR_STRING);
		break;
	case 99:strncpy(s_cStrArray, "圆弧插补中弦长大于2倍半径R ", SIZE_OF_ERROR_STRING);
		break;
	case 100:strncpy(s_cStrArray, "ns程序段只能是G00,G01代码 ", SIZE_OF_ERROR_STRING);
		break;
	case 101:strncpy(s_cStrArray, "ns～nf程序段必需在G71~G73后一段输入 ", SIZE_OF_ERROR_STRING);
		break;
	case 102:strncpy(s_cStrArray, "G73指令代码格式不正确 ", SIZE_OF_ERROR_STRING);
		break;
	case 103:strncpy(s_cStrArray, "G71～G73代码未输入P值 ", SIZE_OF_ERROR_STRING);
		break;
	case 104:strncpy(s_cStrArray, "G71～G73代码未输入Q值 ", SIZE_OF_ERROR_STRING);
		break;
	case 105:strncpy(s_cStrArray, "G72代码第一段未输入Z或Z轴移动量为0 ", SIZE_OF_ERROR_STRING);
		break;
	case 106:strncpy(s_cStrArray, " G72代码第一段只能是Z轴快速移动或切削进给", SIZE_OF_ERROR_STRING);
		break;
	case 107:strncpy(s_cStrArray, "G72指令代码格式不正确", SIZE_OF_ERROR_STRING);
		break;
	case 108:strncpy(s_cStrArray, "G71代码第一段未输入X或X轴移动量为0", SIZE_OF_ERROR_STRING);
		break;
	case 109:strncpy(s_cStrArray, "G71代码第一段只能是X轴快速移动或切削进给 ", SIZE_OF_ERROR_STRING);
		break;
	case 110:strncpy(s_cStrArray, "G71指令代码格式不正确 ", SIZE_OF_ERROR_STRING);
		break;
	case 111:strncpy(s_cStrArray, "G76代码格式不正确 ", SIZE_OF_ERROR_STRING);
		break;
	case 112:strncpy(s_cStrArray, "G76中没有指定螺纹牙高P值或为0", SIZE_OF_ERROR_STRING);
		break;
	case 113:strncpy(s_cStrArray, "G76中没有指定第一次切削深度Q或为0 ", SIZE_OF_ERROR_STRING);
		break;
	case 114:strncpy(s_cStrArray, "G76中P值格式不正确或超出范围", SIZE_OF_ERROR_STRING);
		break;
	case 115:strncpy(s_cStrArray, "手动输入MDI模式下不能使用G31、G70～G76代码", SIZE_OF_ERROR_STRING);
		break;
	case 116:strncpy(s_cStrArray, "执行G70代码前请先执行G71～G73代码", SIZE_OF_ERROR_STRING);
		break;
	case 117:strncpy(s_cStrArray, "C刀补中不能执行T指令，请撤销C刀补", SIZE_OF_ERROR_STRING);
		break;
	case 118:strncpy(s_cStrArray, "C刀补中没有移动的代码段超过规定的数量", SIZE_OF_ERROR_STRING);
		break;
		//case 119:
		//	break;


	case 126:strncpy(s_cStrArray, "当前不是模拟主轴,无法使用G96恒线速功能", SIZE_OF_ERROR_STRING);
		break;
	case 127:strncpy(s_cStrArray, "一行出现了同组模态代码", SIZE_OF_ERROR_STRING);
		break;

	case 129:strncpy(s_cStrArray, "输入了无效的刀具号", SIZE_OF_ERROR_STRING);
		break;
	case 130:strncpy(s_cStrArray, "程序指令代码格式错误", SIZE_OF_ERROR_STRING);
		break;


    case 131:strncpy(s_cStrArray, "函数运算输入的参数无效", SIZE_OF_ERROR_STRING);
        break;
    case 132:strncpy(s_cStrArray, "除法运算除数不能为0", SIZE_OF_ERROR_STRING);
        break;

    case 133:strncpy(s_cStrArray, "平方根输入数据不能小于0", SIZE_OF_ERROR_STRING);
        break;
    case 134:strncpy(s_cStrArray, "宏语句运算格式错误", SIZE_OF_ERROR_STRING);
        break;
    case 135:strncpy(s_cStrArray, "WHILE-END循环体语句格式错误", SIZE_OF_ERROR_STRING);
        break;
    case 136:strncpy(s_cStrArray, "IF语句格式错误", SIZE_OF_ERROR_STRING);
        break;

	case 147:strncpy(s_cStrArray, "G83或G87指令没有指令进刀量Q或Q=0", SIZE_OF_ERROR_STRING);
		break;
	case 148:strncpy(s_cStrArray, "主轴被刹车锁紧，不能旋转主轴", SIZE_OF_ERROR_STRING);
		break;
	case 149:strncpy(s_cStrArray, "夹具未夹主轴不可运转", SIZE_OF_ERROR_STRING);
		break;
	case 150:strncpy(s_cStrArray, "未映射物理轴", SIZE_OF_ERROR_STRING);
		break;
	case 151:strncpy(s_cStrArray, "编码器相位错误", SIZE_OF_ERROR_STRING);
			break;
	case 152:strncpy(s_cStrArray, "速度发生突变", SIZE_OF_ERROR_STRING);
			break;
	case 153:strncpy(s_cStrArray, "参数输入 / 系统参数错误，出现了零值", SIZE_OF_ERROR_STRING);
		break;
    /* 苏向标 2026-10-14：公共停止状态异常诊断。 */
    case 155:strncpy(s_cStrArray, "主轴停止状态或控制权异常", SIZE_OF_ERROR_STRING);
        break;
	case 154:strncpy(s_cStrArray, "超时/超速错误", SIZE_OF_ERROR_STRING);
		break;
	case 160:strncpy(s_cStrArray, "该指令字未增加检查函数", SIZE_OF_ERROR_STRING);
		break;
	case 161:strncpy(s_cStrArray, "该指令字未增加参数赋值函数", SIZE_OF_ERROR_STRING);
		break;
	case 162:strncpy(s_cStrArray, "该指令字未增加轨迹计算函数", SIZE_OF_ERROR_STRING);
		break;

	case 280:strncpy(s_cStrArray, "D/L参数同时出现", SIZE_OF_ERROR_STRING);
		break;
	case 281:strncpy(s_cStrArray, "倒角长度过长", SIZE_OF_ERROR_STRING);
		break;
	case 282:strncpy(s_cStrArray, "倒圆半径过大", SIZE_OF_ERROR_STRING);
		break;
	case 283:strncpy(s_cStrArray, "倒角/倒圆前提条件不满足或几何数据有误", SIZE_OF_ERROR_STRING);
		break;
	case 284:strncpy(s_cStrArray, "自动倒角缓冲区不足", SIZE_OF_ERROR_STRING);
		break;

	case 300:strncpy(s_cStrArray, "传入指针变量为NULL", SIZE_OF_ERROR_STRING);
		break;
    case 301:strncpy(s_cStrArray, "输入了无效的函数参数", SIZE_OF_ERROR_STRING);
        break;
    case 302:strncpy(s_cStrArray, "工循环功能传入指针变量为NULL", SIZE_OF_ERROR_STRING);
        break;
    case 303:strncpy(s_cStrArray, "链表传入指针变量为NULL", SIZE_OF_ERROR_STRING);
        break;
    case 304:strncpy(s_cStrArray, "不存在此分支", SIZE_OF_ERROR_STRING);
        break;
    case 305:strncpy(s_cStrArray, "G66嵌套层数过多", SIZE_OF_ERROR_STRING);
        break;
    case 306:strncpy(s_cStrArray, "译码状态机出错", SIZE_OF_ERROR_STRING);
        break;
    case 307:strncpy(s_cStrArray, "尝试给不为NULL的地址开辟内存", SIZE_OF_ERROR_STRING);
        break;

	case 399:strncpy(s_cStrArray, "一行中出现M功能数目不能超过6个", SIZE_OF_ERROR_STRING);
		break;
	case 400:strncpy(s_cStrArray, "不能出现两个正负号", SIZE_OF_ERROR_STRING);
		break;
	case 401:strncpy(s_cStrArray, "不能出现两个小数点", SIZE_OF_ERROR_STRING);
		break;
	case 402:strncpy(s_cStrArray, "一行中重复出现该字符", SIZE_OF_ERROR_STRING);
		break;
	case 403:strncpy(s_cStrArray, "当前行模态功能不支持输入该字符功能或无模态指令", SIZE_OF_ERROR_STRING);
		break;
	case 404:strncpy(s_cStrArray, "圆弧参数I,K输入有误", SIZE_OF_ERROR_STRING);
		break;
	case 405:strncpy(s_cStrArray, "执行该模态指令缺少运行参数(主轴转向、转速、进给速度)", SIZE_OF_ERROR_STRING);
		break;
	case 406:
		//strncpy(s_cStrArray, "内存空间不足，内存分配失败", SIZE_OF_ERROR_STRING);
#if STM32_MODE
		ver = getStrByWord(sNum, c_num, mode);
		snprintf((char*)erbuff, SIZE_OF_ERROR_STRING,"动态内存分配不足,内存编号:%d 分配失败",ver);
		erbuff[SIZE_OF_ERROR_STRING - 1] = 0;
		strncpy(s_cStrArray, erbuff, SIZE_OF_ERROR_STRING);
		break;
#endif
		//case 407:strncpy(cStrArray, "", SIZE_OF_ERROR_STRING);
		//	break;
	case 408:strncpy(s_cStrArray, "更换刀补类型前，请先撤销刀补", SIZE_OF_ERROR_STRING);
		break;
	case 409:strncpy(s_cStrArray, "只能以直线进行刀补建立或撤销", SIZE_OF_ERROR_STRING);
		break;
	case 410:strncpy(s_cStrArray, "G32或G92代码参数不正确", SIZE_OF_ERROR_STRING);
		break;
	case 411:strncpy(s_cStrArray, "G90或G92参数R错误", SIZE_OF_ERROR_STRING);
		break;
	case 412:strncpy(s_cStrArray, "螺纹导程必须大于0", SIZE_OF_ERROR_STRING);
		break;
	case 413:strncpy(s_cStrArray, "螺纹退尾量不正确", SIZE_OF_ERROR_STRING);
		break;
	case 414:strncpy(s_cStrArray, "G92不能加工端面螺纹", SIZE_OF_ERROR_STRING);
		break;
	case 415:strncpy(s_cStrArray, "G代码过长，超过256个字符", SIZE_OF_ERROR_STRING);
		break;
	case 416:strncpy(s_cStrArray, "使用G71～G73功能后需要使用G70", SIZE_OF_ERROR_STRING);
		break;
	case 417:strncpy(s_cStrArray, ":G71起刀点A不合理", SIZE_OF_ERROR_STRING);
		break;
	case 418:strncpy(s_cStrArray, "G71在X方向的单调性变化不正确", SIZE_OF_ERROR_STRING);
		break;
	case 419:strncpy(s_cStrArray, "G70P值或Q值不正确或没输入", SIZE_OF_ERROR_STRING);
		break;

	case 421:strncpy(s_cStrArray, "G72在Z方向的单调性变化不正确", SIZE_OF_ERROR_STRING);
		break;
	case 422:strncpy(s_cStrArray, "G72起刀点A不合理", SIZE_OF_ERROR_STRING);
		break;
	

	case 425:strncpy(s_cStrArray, "G74指令中未输入Z的值", SIZE_OF_ERROR_STRING);
		break;
	case 426:strncpy(s_cStrArray, "G74指令中Q的值为0或未输入", SIZE_OF_ERROR_STRING);
		break;
    case 427:strncpy(s_cStrArray, "G74或G75指令中未输入X的值", SIZE_OF_ERROR_STRING);
        break;
    case 428:strncpy(s_cStrArray, "G75指令中P的值为0或未输入", SIZE_OF_ERROR_STRING);
        break;
    case 429:strncpy(s_cStrArray, "G74或G75中的单次退刀量 R(e)超出允许范围", SIZE_OF_ERROR_STRING);
        break;
    case 430:strncpy(s_cStrArray, "G74或G75中X或Z方向的单次切削量超出允许范围", SIZE_OF_ERROR_STRING);
        break;
    case 431:strncpy(s_cStrArray, "G74或G75中的单次退刀量R(Δd)超出允许范围", SIZE_OF_ERROR_STRING);
        break;
    case 432:strncpy(s_cStrArray, "连续使用G71、G72、G73功能且尚未执行G70次数过多", SIZE_OF_ERROR_STRING);
        break;
    case 433:strncpy(s_cStrArray, "G71、G72、G73使用段号重复", SIZE_OF_ERROR_STRING);
        break;
	case 434:strncpy(s_cStrArray, "G33/G32系统最大速度无法满足", SIZE_OF_ERROR_STRING);
		break;
    case 435:strncpy(s_cStrArray, "G71II型计算错误", SIZE_OF_ERROR_STRING);
        break;
    case 436:strncpy(s_cStrArray, "G71II型数据存储失败", SIZE_OF_ERROR_STRING);
        break;
    case 437:strncpy(s_cStrArray, "G71二型NF段不允许出现X值变化Z值不变时的起始边", SIZE_OF_ERROR_STRING);
        break;
    case 438:strncpy(s_cStrArray, "圆弧G32弧度不符合要求", SIZE_OF_ERROR_STRING);
        break;
    case 439:strncpy(s_cStrArray, "调用程序出错", SIZE_OF_ERROR_STRING);
        break;
    case 440:strncpy(s_cStrArray, "宏程序嵌套层数超出限制", SIZE_OF_ERROR_STRING);
        break;
    case 441:strncpy(s_cStrArray, "M97指令代码格式不正确", SIZE_OF_ERROR_STRING);
        break;
    case 442:strncpy(s_cStrArray, "等待指令错误", SIZE_OF_ERROR_STRING);
        break;
    case 443:strncpy(s_cStrArray, "自定义宏语法错误", SIZE_OF_ERROR_STRING);
        break;
    case 444:strncpy(s_cStrArray, "IF语句语法错误", SIZE_OF_ERROR_STRING);
        break;
    case 445:strncpy(s_cStrArray, "WHILE语句语法错误", SIZE_OF_ERROR_STRING);
        break;
    case 446:strncpy(s_cStrArray, "GOTO语句输入段号无效", SIZE_OF_ERROR_STRING);
        break;
    case 447:strncpy(s_cStrArray, "宏指令语法错误", SIZE_OF_ERROR_STRING);
        break;
    case 448:strncpy(s_cStrArray, "输入的程序号错误", SIZE_OF_ERROR_STRING);
        break;
    case 449:strncpy(s_cStrArray, "输入的调用次数错误", SIZE_OF_ERROR_STRING);
        break;
    case 450:strncpy(s_cStrArray, "不存在此A类宏指令", SIZE_OF_ERROR_STRING);
        break;
    case 451:strncpy(s_cStrArray, "缺少执行此A类宏指令所需的参数", SIZE_OF_ERROR_STRING);
        break;
    case 452:strncpy(s_cStrArray, "根号内的值不能为负数", SIZE_OF_ERROR_STRING);
        break;
    case 453:strncpy(s_cStrArray, "G34的R值不在范围内", SIZE_OF_ERROR_STRING);
        break;
    case 454:strncpy(s_cStrArray, "G32的长轴最大速度过小", SIZE_OF_ERROR_STRING);
        break;
    case 455:strncpy(s_cStrArray, "G32的短轴最大速度过小", SIZE_OF_ERROR_STRING);
        break;
	case 456:strncpy(s_cStrArray, "G32螺纹车削的进给速率大于1000", SIZE_OF_ERROR_STRING);
		break;

	case 457:strncpy(s_cStrArray, "子程序中没有M99主程序返回", SIZE_OF_ERROR_STRING);
		break;

	case 458:strncpy(s_cStrArray, "另一通道刀补未取消", SIZE_OF_ERROR_STRING);
		break;
	case 459:strncpy(s_cStrArray, "FLASH任务繁忙", SIZE_OF_ERROR_STRING);
		break;
	case 460:strncpy(s_cStrArray, "申请IO失败", SIZE_OF_ERROR_STRING);
		break;
	case 461:strncpy(s_cStrArray, "G90外圆车削循环只能在G18(ZX)平面执行", SIZE_OF_ERROR_STRING);
		break;

	case 1000:strncpy(s_cStrArray, "电动刀架换刀超时", SIZE_OF_ERROR_STRING);
		break;
	case 1001:strncpy(s_cStrArray, "换刀结束时，刀架未到位报警", SIZE_OF_ERROR_STRING);
		break;
	case 1002:strncpy(s_cStrArray, "换刀未完成报警", SIZE_OF_ERROR_STRING);
		break;
	case 1003:strncpy(s_cStrArray, "未检测到刀架锁紧TCP信号", SIZE_OF_ERROR_STRING);
		break;
	case 1004:strncpy(s_cStrArray, "换刀方式A或B最多只支持8把刀", SIZE_OF_ERROR_STRING);
		break;
    case 1005:strncpy(s_cStrArray, "系统断电前换刀出错", SIZE_OF_ERROR_STRING);
        break;

	case 2042:strncpy(s_cStrArray, "上电后读取刀位信号异常报警", SIZE_OF_ERROR_STRING);
		break;
	case 2155:strncpy(s_cStrArray, "换刀后的刀号与指定刀号不符", SIZE_OF_ERROR_STRING);
		break;
	case 2178:strncpy(s_cStrArray, "伺服驱动器报警", SIZE_OF_ERROR_STRING);
		break;
	case 2186:strncpy(s_cStrArray, "刀塔定位完成信号未检测到", SIZE_OF_ERROR_STRING);
		break;
	case 2187:strncpy(s_cStrArray, "未检测到刀架锁紧到位或松开到位信号", SIZE_OF_ERROR_STRING);
		break;
	case 2188:strncpy(s_cStrArray, "刀具状态未定义", SIZE_OF_ERROR_STRING);
		break;
	default:
		strncpy(s_cStrArray, "暂未添加", SIZE_OF_ERROR_STRING);
		break;
	}

	return s_cStrArray;
}

/*************************************************
  函数名称：bool isErrorFull()
  函数功能: 返回标志位，判断错误缓冲是否满
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
bool isErrorFull()
{
	if (s_ucPosSGCodeErrorBuf >= GCODEERRORBUF_MAX)    //错误缓存区满，不再存放
	{
		return TRUE;
	}

	return FALSE;
}


/*************************************************
  函数名称：void setErrorType(uint16 usType)
  函数功能: 设置错误类型
  输入参数：usType	错误代码
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void setErrorType(uint16 usType)
{
	s_ErrorType = usType;
}

/*************************************************
  函数名称：bool isErrorHasBeenWritten()
  函数功能: 判断是否已有错误写入
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
bool isErrorHasBeenWritten()
{
	return s_bErrorHasBeenWritten;
}

/*************************************************
  函数名称：void setErrorWriteFlag(bool bFlag)
  函数功能: 设置错误已写入标志位
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void setErrorWriteFlag(bool bFlag)
{
	s_bErrorHasBeenWritten = bFlag;
}


/*************************************************
  函数名称：uint8 getErrorNum()
  函数功能: 获取错误数量
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
uint8 getErrorNum()
{
	return s_ucPosSGCodeErrorBuf;
}

/*************************************************
  函数名称：void setErrorFlag(bool bFlag)
  函数功能: 设置错误标志位
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void setErrorFlag(bool bFlag)
{

	s_bErrorFlag = bFlag;
}

/*************************************************
  函数名称：bool getErrorFlag()
  函数功能: 获取错误标志位
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
bool getErrorFlag()
{
	return s_bErrorFlag;
}

/*************************************************
  函数名称：SGCodeError* getErrorStruct(int8 cIndex)
  函数功能: 返回错误结构体的地址
  输入参数：cIndex错误缓存索引
  输出参数：
  返 回 值：
  备    注：
**************************************************/
struct SGCodeError getErrorStructAdd(int8 cIndex)
{
	struct SGCodeError STemp = {0};
	if (cIndex < 0 || cIndex>GCODEERRORBUF_MAX-1)
	{
		return STemp;
		parseGCodeErrorB(0,GCODE_ERROR_NUM305);
	}
	else
	{
		STemp= s_SGCodeErrorBuf[cIndex];
		return STemp;
	}

}
