#include "ringbuff.h"
#include "GeneralFunctions.h"
#include <stdlib.h>
#include "Config.h"
#include "channel.h"

struct SRingBuff g_SRingBuff[CHANNEL_NUM_MAX];//环形缓冲区


//以下参数即将删除
//struct SGCodeParse g_SGCodeParseBuf[GCODEPARSEBUF_MAX]; //SGCodeParseBuf循环缓冲区（20条）(移动到环形缓冲结构体)

//struct SToolCompensation g_SToolCompensationBuf[TOOLCOMPBUF_MAX];//SToolCompensationBuf循环缓冲区（20条）(移动到环形缓冲结构体)

//int16 g_sSGCODE_WRITEIndex; //G代码缓存写指针索引值(移动到环形缓冲结构体)
//
//int16 g_sSGCODE_WRITEdNum;  //G代码已写入个数(且刀补未读取个数）(移动到环形缓冲结构体)
//
//int16 g_sSToolIndex = 0;      //刀补写指针的索引值
//
//int16 g_sSToolDealedNum = 0;  //刀补 处理过的数组元素个数
//
//int16 g_sLookAheadDealedNum = 0;//前瞻 处理过的数组元素个数


/*************************************************
  函数名称：getSGCodeParsePointerIndex(enum RingBufPointerType pointerTpye)
  函数功能  根据输入的指针类型，获取该指针的索引并返回
  输入参数：pointerTpye，enum RingBufPointerType 枚举变量 (0~1)
  输出参数：
  返 回 值：int16 指针对应元素的索引 范围：（0~GCODEPARSEBUF_MAX-1）
			-1	  输入的枚举类型错误
  备    注：
**************************************************/

int16 getSGCodeParsePointerIndex(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sIndex;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}

	switch (pointerTpye)
	{
	case GCODE_WRITE:
		sIndex = g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex;//G代码写指针
		break;
	case GCODE_READ:
		//G代码读指针的索引  = 写指针索引 - 已写元素个数
		sIndex = g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex - g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum;
		break;
	case STOOL_READ_AHEAD:
		//刀补预读指针索引 = 写指针索引 - 可预读元素个数
		sIndex = g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex - g_SRingBuff[ucChannelNum].sSGcodeReadAheadNum;
		break;
	default:
		sIndex = 0;
		break;
	}

	sIndex %= GCODEPARSEBUF_MAX;//对sIndex取余，防止超过数组范围

	if (sIndex < 0)   //防止Index小于零，小于数组范围
	{
		sIndex += GCODEPARSEBUF_MAX;
	}

	return sIndex;
}



/*************************************************
  函数名称：getSGCodeParsePointerByIndex(short sIndex)
  函数功能  根据输入的索引获取对应的数组元素指针
  输入参数：sIndex	索引
  输出参数：
  返 回 值：struct SGCodeParse*  数组元素指针
  备    注：
**************************************************/
struct SGCodeParse* getSGCodeParsePointerByIndex(short sIndex, uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}
	sIndex %= GCODEPARSEBUF_MAX;  //对sIndex取余数  保证Index不会超过数组最大值

	if (sIndex < 0)   //防止Index小于零，返回野指针
	{
		sIndex += GCODEPARSEBUF_MAX;
	}
	return g_SRingBuff[ucChannelNum].SGCodeParseBuf + sIndex;//返回索引值对应的元素指针
}



/*************************************************
  函数名称：getSGCodeParsePointerByType(enum RingBufPointerType pointerTpye)
  函数功能  根据输入的指针类型寻找其索引值，根据索引值找到对应的数组元素，返回其指针
  输入参数：pointerTpye，enum RingBufPointerType 枚举变量 (0~1)
  输出参数：
  返 回 值：struct SGCodeParse*  数组元素指针
  备    注：
**************************************************/
struct SGCodeParse* getSGCodeParsePointerByType(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sIndex;  //索引值
	struct SGCodeParse* pPointer; //指针
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}
	sIndex = getSGCodeParsePointerIndex(pointerTpye, ucChannelNum);  //获取索引值

	pPointer = getSGCodeParsePointerByIndex(sIndex, ucChannelNum);   //根据索引值获取指针

	return pPointer;  //返回结构体指针
}

/*************************************************
  函数名称：moveSGCodeParsePointer(enum RingBufPointerType pointerTpye)
  函数功能  根据指针类型,控制指针从当前位置往数组的下一个元素移动
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：TRUE	移动成功
			FALSE	移动失败
  备    注：
**************************************************/
bool moveSGCodeParsePointer(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sResBuf;  //剩余buf
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}
	sResBuf = getSGCodeParsePointerBufRes(pointerTpye, ucChannelNum);//获取对应 指针的剩余buf

	//剩余buf小于0不能移动,移动后剩余buf为负数，不应出现
	if (sResBuf <= 0)
	{
		return FALSE;
	}

	switch (pointerTpye)
	{
	case GCODE_WRITE:
		g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex += 1;  //写指针索引加1
		g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum += 1;	//已写元素个数加1
		g_SRingBuff[ucChannelNum].sSGcodeReadAheadNum += 1; //可预读元素个数+1
		break;
	case GCODE_READ:
		g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum -= 1;   //已写元素个数减1
		break;
	case STOOL_READ_AHEAD:
		g_SRingBuff[ucChannelNum].sSGcodeReadAheadNum -= 1; //刀补可预读元素个数减1
		break;
	default:
		return FALSE;
	}

	//对sSToolIndex取余数  保证Index不会超过数组最大值
	g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex %= GCODEPARSEBUF_MAX;

	if (g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex < 0)   //防止Index小于零，返回野指针
	{
		g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex += GCODEPARSEBUF_MAX;
	}
	return TRUE;
}

/*************************************************
  函数名称：getSGCodeParsePointerBufRes(enum RingBufPointerType pointerTpye)
  函数功能  根据输入的指针类型，获取该指针能够读/写的容量
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：int16	指针能够读/写的容量
  备    注：
**************************************************/
int16 getSGCodeParsePointerBufRes(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sBufRes; //    能读/写容量  
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}

	switch (pointerTpye)
	{
	case GCODE_WRITE:
		//G代码写指针 能够写的个数
		sBufRes = GCODEPARSEBUF_MAX - g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum;
		break;
	case GCODE_READ:
		//G代码读指针 能够读的个数
		sBufRes = g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum;
		break;
	case STOOL_READ_AHEAD:
		//刀补可预读指针个数
		sBufRes = g_SRingBuff[ucChannelNum].sSGcodeReadAheadNum;
		break;
	default:
		sBufRes = -1;
		break;
	}

	return sBufRes;
}


///打印数组指针索引，地址信息(调试) 
void printSGCodeParsePointerADDR()
{
#if EN_DEBUG
	int i;
	printf("****************************************\n");
	for (i = 0; i < 20; i++)
	{
		printf("%2.d %d\n", i, g_SGCodeParseBuf + i);
	}
	printf("****************************************\n");
	printf("BUF::\n");
	printf("GCODE_WRITE = %d\n", getSGCodeParsePointerBufRes(GCODE_WRITE));
	printf("GCODE_READ = %d\n", getSGCodeParsePointerBufRes(GCODE_READ));
	printf("****************************************\n");
	printf("INDEX AND ADDR::\n");
	printf("GCODE_WRITE:\n");
	printf("%2.d %d\n", getSGCodeParsePointerIndex(GCODE_WRITE), getSGCodeParsePointerByType(GCODE_WRITE));
	printf("GCODE_READ:\n");
	printf("%2.d %d\n", getSGCodeParsePointerIndex(GCODE_READ), getSGCodeParsePointerByType(GCODE_READ));
#endif

}



/******************SToolCompensationBuf循环缓冲区-变量函数********************************/




/*     SToolCompensationBuf循环缓冲区函数使用范例
void XXX()  //某函数实现
{
	int16 sBufRes;
	//先获取可读/写容量   如果没有容量则不进行刀补或前瞻处理
	sBufRes = getSToolCompPointerBufRes(enum XX);//获取容量

	if (sBufRes < N)  //N为需要前瞻段数  或 刀补新增的线段数
	{
		return;  //缓存区不足  不能执行前瞻或刀补 (因为数据可读 或 没有空间可写)
	}

	//先获取指针
	struct SToolCompensation *pSToolCmpPointer = getSToolCompPointerByType(enum XX);

	//执行前瞻或刀补
	//利用指针 pSToolCmpPointer 进行读写
	//存入数据后

	moveSToolCompBufPointer(enum XX);//移动指针
	//#### 需要多次读写就重新获取指针，处理完&&读出或存入数据之后，再移动指针#####//
	//处理完之后一定要手动移动指针  否则下次写入会覆盖 或 重复读
}
*/



#if NEW_BUFF == 0
/*************************************************
  函数名称：getSToolCompPointerIndex(enum SToolCompBufPointerType pointerTpye)
  函数功能  根据输入的指针类型，获取该指针的索引并返回
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：int16 指针对应元素的索引 范围：（0~TOOLCOMPBUF_MAX-1）
			-1	  输入的枚举类型错误
  备    注：输入没定义的枚举值时，会返回头元素索引
**************************************************/
int16 getSToolCompPointerIndex(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sIndex;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}

	switch (pointerTpye)
	{
	case STOOL_WRITE:
		sIndex = g_SRingBuff[ucChannelNum].sSToolIndex; //刀补写的索引值为sSToolIndex
		break;
	case LOOKAHEAD_READWRITE:
		//前瞻读写的索引为sSToolIndex - 刀补处理的 + 前瞻处理的
		sIndex = g_SRingBuff[ucChannelNum].sSToolIndex - g_SRingBuff[ucChannelNum].sSToolDealedNum + g_SRingBuff[ucChannelNum].sLookAheadDealedNum;
		break;
	case INTERPOLATION_READ:
		//插补的索引为 sSToolIndex - 刀补处理的
		sIndex = g_SRingBuff[ucChannelNum].sSToolIndex - g_SRingBuff[ucChannelNum].sSToolDealedNum;
		break;
	case INTERPOLATION_BACK:
		//倒走的索引为 sSToolIndex - 刀补处理的 - 2  //当前段的前一个   当前段为-1  前一个位-1-1
		sIndex = g_SRingBuff[ucChannelNum].sSToolIndex - g_SRingBuff[ucChannelNum].sSToolDealedNum - 2; //倒走索引  
		break;
	default:
		sIndex = 0;
		break;
	}

	sIndex %= TOOLCOMPBUF_MAX;//对sIndex取余，防止超过数组范围

	if (sIndex < 0)   //防止Index小于零，小于数组范围
	{
		sIndex += TOOLCOMPBUF_MAX;
	}

	return sIndex;
}



/*************************************************
  函数名称：getSToolCompPointerByIndex(int16 sIndex)
  函数功能  根据输入的索引获取对应的数组元素指针
  输入参数：sIndex	索引
  输出参数：
  返 回 值：struct SToolCompensation*  数组元素指针
  备    注：
**************************************************/
struct SToolCompensation* getSToolCompPointerByIndex(int16 sIndex, uint8 ucChannelNum)
{
	sIndex %= TOOLCOMPBUF_MAX;  //对sIndex取余数  保证Index不会超过数组最大值
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}
	if (sIndex < 0)   //防止Index小于零，返回野指针
	{
		sIndex += TOOLCOMPBUF_MAX;
	}
	return g_SRingBuff[ucChannelNum].SToolCompensationBuf + sIndex;//返回索引值对应的元素指针
}



/*************************************************
  函数名称：getSToolCompPointerByType(enum SToolCompBufPointerType pointerTpye)
  函数功能  根据输入的指针类型寻找其索引值，根据索引值找到对应的数组元素，返回其指针
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：struct SToolCompensation*  数组元素指针
  备    注：
**************************************************/
struct SToolCompensation* getSToolCompPointerByType(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sIndex;  //索引值
	struct SToolCompensation* pPointer; //指针
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}
	sIndex = getSToolCompPointerIndex(pointerTpye, ucChannelNum);  //获取索引值

	pPointer = getSToolCompPointerByIndex(sIndex, ucChannelNum);   //根据索引值获取指针

	return pPointer;  //返回结构体指针
}



/*************************************************
  函数名称：moveSToolCompBufPointer(enum SToolCompBufPointerType pointerTpye)
  函数功能  根据指针类型,控制指针从当前位置往数组的下一个元素移动
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：TRUE	移动成功
			FALSE	移动失败
  备    注：
**************************************************/
bool moveSToolCompBufPointer(enum RingBufPointerType pointerTpye, short sDist, uint8 ucChannelNum)
{
	int16 sResBuf;  //剩余buf
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}
	sResBuf = getSToolCompPointerBufRes(pointerTpye, ucChannelNum);//获取对应 指针的剩余buf

	//剩余buf小于0不能移动,移动后剩余buf为负数，不应出现
	if (sResBuf - sDist < 0 || sResBuf - sDist > TOOLCOMPBUF_MAX)
	{
		return FALSE;
	}

	//判断指针类型，移动对应指针
	switch (pointerTpye)
	{
	case STOOL_WRITE:
		g_SRingBuff[ucChannelNum].sSToolIndex += sDist;    //刀补写指针移动
		g_SRingBuff[ucChannelNum].sSToolDealedNum += sDist;//刀补处理过的元素加1
		break;
	case LOOKAHEAD_READWRITE:
		g_SRingBuff[ucChannelNum].sLookAheadDealedNum += sDist;   //前瞻处理过的元素加1
		break;
	case INTERPOLATION_READ:
		g_SRingBuff[ucChannelNum].sSToolDealedNum -= sDist;      //刀补处理过的元素减1
		g_SRingBuff[ucChannelNum].sLookAheadDealedNum -= sDist;   //前瞻处理过的元素减1
		//插补处理过的段数+1  不能超过倒走段数
		g_SRingBuff[ucChannelNum].sInterpolationDealedNum = (g_SRingBuff[ucChannelNum].sInterpolationDealedNum + sDist > BACKWARDBUF_NUM) ? BACKWARDBUF_NUM : max(g_SRingBuff[ucChannelNum].sInterpolationDealedNum + sDist, 0);
		break;
	case INTERPOLATION_BACK:
		g_SRingBuff[ucChannelNum].sSToolDealedNum += sDist;      //刀补处理过的元素减1
		g_SRingBuff[ucChannelNum].sLookAheadDealedNum += sDist;   //前瞻处理过的元素减1
		//插补处理过的段数+1  不能超过倒走段数
		g_SRingBuff[ucChannelNum].sInterpolationDealedNum = (g_SRingBuff[ucChannelNum].sInterpolationDealedNum - sDist < 0) ? 0 : g_SRingBuff[ucChannelNum].sInterpolationDealedNum - sDist;
		break;
	default:
		return FALSE;
		break;
	}

	//对sSToolIndex取余数  保证Index不会超过数组最大值
	g_SRingBuff[ucChannelNum].sSToolIndex %= TOOLCOMPBUF_MAX;

	if (g_SRingBuff[ucChannelNum].sSToolIndex < 0)   //防止Index小于零，返回野指针
	{
		g_SRingBuff[ucChannelNum].sSToolIndex += TOOLCOMPBUF_MAX;
	}
	return TRUE;
}



/*************************************************
  函数名称：getSToolCompPointerBufRes(enum SToolCompBufPointerType pointerTpye)
  函数功能  根据输入的指针类型，获取该指针能够读/写的容量
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：int16	指针能够读/写的容量
  备    注：
**************************************************/
int16 getSToolCompPointerBufRes(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sBufRes = 0; //    能读/写容量  
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}

	switch (pointerTpye)
	{
	case STOOL_WRITE:
		//刀补写能写的数组元素个数
		sBufRes = TOOLCOMPBUF_MAX - g_SRingBuff[ucChannelNum].sSToolDealedNum - 1 - g_SRingBuff[ucChannelNum].sInterpolationDealedNum;  //插补处理过的段数不能写入数据  倒走需要
		break;
	case LOOKAHEAD_READWRITE:
		//前瞻能够处理的数组元素个数
		sBufRes = g_SRingBuff[ucChannelNum].sSToolDealedNum - g_SRingBuff[ucChannelNum].sLookAheadDealedNum;
		break;
	case INTERPOLATION_READ:
		//插补能够读的数组元素个数
		sBufRes = g_SRingBuff[ucChannelNum].sLookAheadDealedNum;
		break;
	case INTERPOLATION_BACK:
		//插补倒走能够读的数组元素个数
		sBufRes = g_SRingBuff[ucChannelNum].sInterpolationDealedNum - 1;
		break;

	default:
		sBufRes = -1;
		break;
	}
	return sBufRes;
}


//	初始化前瞻指针
bool initLookAheadDealed(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return FALSE;
	}
	g_SRingBuff[ucChannelNum].sLookAheadDealedNum = 0;
	return TRUE;
}

//清除插补处理过的数据
bool clearInterpolationDealedNum(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return FALSE;
	}
	g_SRingBuff[ucChannelNum].sInterpolationDealedNum = 1;// 
}


/*************************************************
  函数名称：printSToolCompPointerADDR()
  函数功能  输出三个指针的地址、索引、和指针可读/写容量
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void printSToolCompPointerADDR()
{
#if EN_DEBUG
	int i;
	printf("****************************************\n");
	for (i = 0; i < 20; i++)
	{
		printf("%2.d %d\n", i, g_SToolCompensationBuf + i);
	}
	printf("****************************************\n");
	printf("BUF::\n");
	printf("STOOL_WRITE = %d\n", getSToolCompPointerBufRes(STOOL_WRITE));
	printf("LOOKAHEAD_READWRITE = %d\n", getSToolCompPointerBufRes(LOOKAHEAD_READWRITE));
	printf("INTERPOLATION_READ = %d\n", getSToolCompPointerBufRes(INTERPOLATION_READ));
	printf("****************************************\n");
	printf("INDEX AND ADDR::\n");
	printf("STOOL_WRITE:\n");
	printf("%2.d %d\n", getSToolCompPointerIndex(STOOL_WRITE), getSToolCompPointerByType(STOOL_WRITE));
	printf("LOOKAHEAD_READWRITE:\n");
	printf("%2.d %d\n", getSToolCompPointerIndex(LOOKAHEAD_READWRITE), getSToolCompPointerByType(LOOKAHEAD_READWRITE));
	printf("INTERPOLATION_READ:\n");
	printf("%2.d %d\n", getSToolCompPointerIndex(INTERPOLATION_READ), getSToolCompPointerByType(INTERPOLATION_READ));
#endif

}

//初始化2个缓冲区数组
int8 initAllBuffer(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}
	g_SRingBuff[ucChannelNum].sSToolIndex = 0;      //刀补写指针的索引值
	g_SRingBuff[ucChannelNum].sSToolDealedNum = 0;  //刀补 处理过的数组元素个数
	g_SRingBuff[ucChannelNum].sLookAheadDealedNum = 0;//前瞻 处理过的数组元素个数
	g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex = 0; //G代码缓存写指针索引值
	g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum = 0;  //G代码已写入个数(且刀补未读取个数）
	g_SRingBuff[ucChannelNum].sSToolDealedNum = 0;//G代码缓冲区刀补可预读个数
	g_SRingBuff[ucChannelNum].sInterpolationDealedNum = 0; // 插补处理过的元素个数  （倒走）
}
#else  //2023.5.25 朱稳中 

//	获取指针的索引值
INDEX_DATA_TYPE getSToolCompPointerIndex(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	INDEX_DATA_TYPE nIndex;
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}

	switch (pointerTpye)
	{
	case STOOL_WRITE:
		//刀补写索引
		nIndex = g_SRingBuff[ucChannelNum].nSToolIndex; //刀补写的索引值为sSToolIndex
		break;
	case LOOKAHEAD_READWRITE:
		//前瞻读写的索引
		nIndex = g_SRingBuff[ucChannelNum].nLookAheadIndex;
		break;
	case INTERPOLATION_READ:
		//插补的索引
		nIndex = g_SRingBuff[ucChannelNum].nInterpIndex;
		break;
	case INTERPOLATION_BACK:
		//倒走的索引 插补 - 2    //当前段的前一个   当前段为-1  前一个位-1-1
		nIndex = g_SRingBuff[ucChannelNum].nInterpIndex - 2; //倒走索引  
		break;
	case DRAW_READ:
		nIndex = g_SRingBuff[ucChannelNum].nDrawIndex;
		break;
	default:
		nIndex = 0;
		break;
	}

	nIndex %= TOOLCOMPBUF_MAX;//对sIndex取余，防止超过数组范围

	return (int16)nIndex;
}


//	获取数组元素指针(通过索引)
struct SToolCompensation* getSToolCompPointerByIndex(INDEX_DATA_TYPE nIndex, uint8 ucChannelNum)
{
	nIndex += TOOLCOMPBUF_MAX;
	nIndex %= TOOLCOMPBUF_MAX;  //对sIndex取余数  保证Index不会超过数组最大值
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}

	return &g_SRingBuff[ucChannelNum].SToolCompensationBuf[nIndex];//返回索引值对应的元素指针
}

//	获取数组元素指针(通过类型)
struct SToolCompensation* getSToolCompPointerByType(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	INDEX_DATA_TYPE nIndex;  //索引值
	struct SToolCompensation* pPointer; //指针
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}
	nIndex = getSToolCompPointerIndex(pointerTpye, ucChannelNum);  //获取索引值

	pPointer = getSToolCompPointerByIndex(nIndex, ucChannelNum);   //根据索引值获取指针

	return pPointer;  //返回结构体指针


}


/*************************************************
  函数名称：moveSToolCompBufPointer(enum SToolCompBufPointerType pointerTpye)
  函数功能  根据指针类型,控制指针从当前位置往数组的下一个元素移动
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：TRUE    移动成功
            FALSE   移动失败
  备    注：
**************************************************/
bool moveSToolCompBufPointer(enum RingBufPointerType pointerTpye, short sDist, uint8 ucChannelNum)
{
	int16 sResBuf;  //剩余buf
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}
	sResBuf = getSToolCompPointerBufRes(pointerTpye, ucChannelNum);//获取对应 指针的剩余buf

	//剩余buf小于0不能移动,移动后剩余buf为负数，不应出现
	if (sResBuf - sDist < 0 || sResBuf - sDist > TOOLCOMPBUF_MAX)
	{
		return FALSE;
	}

	//判断指针类型，移动对应指针
	switch (pointerTpye)
	{
	case STOOL_WRITE:
		g_SRingBuff[ucChannelNum].nSToolIndex += sDist;    //刀补索引加 sDist
		break;
	case LOOKAHEAD_READWRITE:
		g_SRingBuff[ucChannelNum].nLookAheadIndex += sDist;   //前瞻索引加 sDist
		break;
	case INTERPOLATION_READ:
		g_SRingBuff[ucChannelNum].nInterpIndex += sDist;  //插补索引加 sDist
		//插补处理过的段数+1  不能超过倒走段数
		if (sDist>=0)
		{
			g_SRingBuff[ucChannelNum].nBackIndex = max(g_SRingBuff[ucChannelNum].nBackIndex, g_SRingBuff[ucChannelNum].nInterpIndex - BACKWARDBUF_NUM);
		}
		//g_SRingBuff[ucChannelNum].nBackIndex = (g_SRingBuff[ucChannelNum].nBackIndex + sDist > BACKWARDBUF_NUM) ? BACKWARDBUF_NUM : max(g_SRingBuff[ucChannelNum].nBackIndex + sDist, 0);
		break;
	case INTERPOLATION_BACK:
		g_SRingBuff[ucChannelNum].nInterpIndex -= sDist; //插补索引减 sDist
		////插补处理过的段数+1  不能超过倒走段数
		//g_SRingBuff[ucChannelNum].nBackIndex = (g_SRingBuff[ucChannelNum].nBackIndex - sDist < 0) ? 0 : g_SRingBuff[ucChannelNum].nBackIndex - sDist;
		break;
	case DRAW_READ:
		g_SRingBuff[ucChannelNum].nDrawIndex += sDist;   //画图索引加 sDist
		break;
	default:
		return FALSE;
		break;
	}

	return TRUE;
}

/*************************************************
  函数名称：getSToolCompPointerBufRes(enum SToolCompBufPointerType pointerTpye)
  函数功能  根据输入的指针类型，获取该指针能够读/写的容量
  输入参数：pointerTpye，enum SToolCompBufPointerType 枚举变量 (0~2)
  输出参数：
  返 回 值：int16   指针能够读/写的容量
  备    注：
**************************************************/
//	获取指针剩余容量
int16 getSToolCompPointerBufRes(enum RingBufPointerType pointerTpye, uint8 ucChannelNum)
{
	int16 sBufRes = 0; //    能读/写容量  
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}

	switch (pointerTpye)
	{
	case STOOL_WRITE:
		//1.画图模式
		if (g_SChannel[ucChannelNum].cSubRunMode == 0)
		{
			sBufRes = TOOLCOMPBUF_MAX - g_SRingBuff[ucChannelNum].nSToolIndex + g_SRingBuff[ucChannelNum].nDrawIndex;
		}
		//2.运动模式//刀补写能写的数组元素个数    (MAX+倒走-刀补)
		else if (g_SChannel[ucChannelNum].cSubRunMode == 1)
		{
			sBufRes = TOOLCOMPBUF_MAX - g_SRingBuff[ucChannelNum].nSToolIndex + g_SRingBuff[ucChannelNum].nBackIndex;
		}
		
		break;
	case LOOKAHEAD_READWRITE:
		//前瞻能够处理的数组元素个数
		sBufRes = g_SRingBuff[ucChannelNum].nSToolIndex - g_SRingBuff[ucChannelNum].nLookAheadIndex;
		break;
	case INTERPOLATION_READ:
		//插补能够读的数组元素个数
		sBufRes = g_SRingBuff[ucChannelNum].nLookAheadIndex - g_SRingBuff[ucChannelNum].nInterpIndex;
		break;
	case INTERPOLATION_BACK:
		//插补倒走能够读的数组元素个数
		sBufRes = g_SRingBuff[ucChannelNum].nInterpIndex-g_SRingBuff[ucChannelNum].nBackIndex - 1;
		break;
	case DRAW_READ:
		sBufRes = g_SRingBuff[ucChannelNum].nSToolIndex - g_SRingBuff[ucChannelNum].nDrawIndex;
		break;
	default:
		sBufRes = -1;
		break;
	}
	return sBufRes;
}

//  清除插补已处理过的数据  
bool clearInterpolationDealedNum(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return FALSE;
	}
	g_SRingBuff[ucChannelNum].nBackIndex = g_SRingBuff[ucChannelNum].nInterpIndex-1;// 
	return TRUE;
}
#include <stdio.h>
//	打印数组指针索引，地址信息(调试)
void printSToolCompPointerADDR()
{
	printf("****************************************\n");
	printf("刀补：索引 = %d\t可读写 = %d\n", getSToolCompPointerIndex(STOOL_WRITE, 0), getSToolCompPointerBufRes(STOOL_WRITE, 0));
	printf("前瞻：索引 = %d\t可读写 = %d\n", getSToolCompPointerIndex(LOOKAHEAD_READWRITE, 0), getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, 0));
	printf("插补：索引 = %d\t可读写 = %d\n", getSToolCompPointerIndex(INTERPOLATION_READ, 0), getSToolCompPointerBufRes(INTERPOLATION_READ, 0));
	printf("倒走：索引 = %d\t可读写 = %d\n", getSToolCompPointerIndex(INTERPOLATION_BACK, 0), getSToolCompPointerBufRes(INTERPOLATION_BACK, 0));
	printf("****************************************\n");
}

//	初始化前瞻指针   //将前瞻索引赋值为插补索引
bool initLookAheadDealed(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return FALSE;
	}
	g_SRingBuff[ucChannelNum].nLookAheadIndex = g_SRingBuff[ucChannelNum].nInterpIndex;
	return TRUE;
}

//初始化缓冲区
int8 initAllBuffer(uint8 ucChannelNum)
{

	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return -1;
	}
	//g_SRingBuff[ucChannelNum].sSToolIndex = 0;      //刀补写指针的索引值
	//g_SRingBuff[ucChannelNum].sSToolDealedNum = 0;  //刀补 处理过的数组元素个数
	//g_SRingBuff[ucChannelNum].sLookAheadDealedNum = 0;//前瞻 处理过的数组元素个数
	g_SRingBuff[ucChannelNum].sSGCODE_WRITEIndex = 0; //G代码缓存写指针索引值
	g_SRingBuff[ucChannelNum].sSGCODE_WRITEdNum = 0;  //G代码已写入个数(且刀补未读取个数）
	//g_SRingBuff[ucChannelNum].sSToolDealedNum = 0;//G代码缓冲区刀补可预读个数
	//g_SRingBuff[ucChannelNum].sInterpolationDealedNum = 0; // 插补处理过的元素个数  （倒走）
    g_SRingBuff[ucChannelNum].sSGcodeReadAheadNum = 0;
	g_SRingBuff[ucChannelNum].nBackIndex = 0;
	g_SRingBuff[ucChannelNum].nInterpIndex = 0;
	g_SRingBuff[ucChannelNum].nLookAheadIndex = 0;
	g_SRingBuff[ucChannelNum].nSToolIndex = 0;

	//画图读
	g_SRingBuff[ucChannelNum].nDrawIndex = 0;

	return 1;
}


#endif

/*************************************************
  函数名称：struct SGCodeParse *getGCodeHeadAddress()
  函数功能  返回G代码缓冲区的首地址
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
struct SGCodeParse* getGCodeHeadAddress(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超出数组范围
	{
		return NULL;
	}
	return g_SRingBuff[ucChannelNum].SGCodeParseBuf;
}














//int8 DrawDataBuff_1[512];//512B
//int8 DrawDataBuff_2[512];//512B
/*************************************************
  函数名称：int8 getDrawData(SDrawData* pWriteAdress, int8 cWriteNum)
  函数功能  获取刀补缓冲区的轨迹数据
  输入参数：
  输出参数：
  返 回 值：-2	加工程序错误
			-3	传参错误
			0	画图读容量不足
			n	写入轨迹
			0xAF	写入完成
  备    注：
**************************************************/
int8 cTempBuff[512];
int8 cNum = 0;
int nPos = 0;
int8 cGetDrawData_Step = 0;
int8 getDrawData(int8* pBuffAdress, int8 cWriteNum, int8 ucChannelNum)
{
	//检查系统是否为运行状态
	if (g_SChannel[ucChannelNum].ucChannelStatus != SYS_RUN)
		return 0xAF;
	//检查加工程序是否出现错误
	if (isErrorHasBeenWritten() == TRUE)
		return -2;
	//检查传参
	if (pBuffAdress == NULL || cWriteNum <= 0)
	{
		return -3;
	}
	switch (cGetDrawData_Step)
	{
	//初始化
	case 0:
	{
		cNum = 0;
		nPos = 0;
		cGetDrawData_Step = 1;
	}
	//写入临时缓冲区
	case 1:
	{
		bool bFull = FALSE;
		bool bFinish = FALSE;
		while (cNum < cWriteNum)
		{
			//1.判断画图读容量
			if (getSToolCompPointerBufRes(DRAW_READ, ucChannelNum) <= 0)
				break;
			//2.获取画图读缓冲区单元
			struct SToolCompensation* pToolBuf;
			pToolBuf = getSToolCompPointerByType(DRAW_READ, ucChannelNum);

			//3.判断当前刀补缓冲区单元是否有效
			if (!isSamePoint(&pToolBuf->SStartPoint, &pToolBuf->SEndPoint))
			{
				//3.1计算Buff容量
				int nBufRest = 512 - nPos;
				pToolBuf->ucToolCompNum;

				//3.2计算总偏置
				SPointDouble SG50Offset = s_pSChannelConfig[0]->SAssignConfig.SGCodeCurrSysState.SAllOffsetOfCoord.SG50Offset;
				SPointDouble STotalOffset;
				TChangeTotalOffset(ucChannelNum, pToolBuf->ucToolCompNum, pToolBuf->cWorkpieceCoordNum, &STotalOffset, &SG50Offset);
				
				//写入画图数据
				switch (getBits(pToolBuf->unCommandType, CMD_MOTION_ADDR))
				{
				//虚线、直线轨迹大小：17字节
				case CMD_MOTION_QUICKMOVE:
				case CMD_MOTION_LINE:
				case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
				case CMD_MOTION_SCREWCUT_VP:
				case CMD_MOTION_SCREWCUT:  //螺纹切削
				case CMD_MOTION_G33://G33螺纹攻牙循环
				{
					//判断画图缓冲区剩余内存
					if (nBufRest < 17)
					{
						bFull = TRUE;
						break;
					}

					//写入类型
					if (getBits(pToolBuf->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)
						*(cTempBuff + nPos) = 0;//虚线
					else
						*(cTempBuff + nPos) = 1;//实线
					nPos += 1;

					//写入起点X
					float fStartPoint_X;
					fStartPoint_X = (float)pToolBuf->SStartPoint.dArray[AXIS_X];
					fStartPoint_X -= STotalOffset.dArray[AXIS_X];
					memcpy(cTempBuff + nPos, &fStartPoint_X, 4);
					nPos += 4;

					//写入起点Z
					float fStartPoint_Z;
					fStartPoint_Z = (float)pToolBuf->SStartPoint.dArray[AXIS_Z];
					fStartPoint_Z -= STotalOffset.dArray[AXIS_Z];
					memcpy(cTempBuff + nPos, &fStartPoint_Z, 4);
					nPos += 4;

					//写入终点X
					float fEndPoint_X;
					fEndPoint_X = (float)pToolBuf->SEndPoint.dArray[AXIS_X];
					fEndPoint_X -= STotalOffset.dArray[AXIS_X];
					memcpy(cTempBuff + nPos, &fEndPoint_X, 4);
					nPos += 4;

					//写入终点Z
					float fEndPoint_Z;
					fEndPoint_Z = (float)pToolBuf->SEndPoint.dArray[AXIS_Z];
					fEndPoint_Z -= STotalOffset.dArray[AXIS_Z];
					memcpy(cTempBuff + nPos, &fEndPoint_Z, 4);
					nPos += 4;

					//移指针
					moveSToolCompBufPointer(DRAW_READ, BUF_AHEAD, ucChannelNum);

					break;
				}

				//圆弧轨迹大小：26字节
				case CMD_MOTION_ARC:
				case CMD_MOTION_ANTI_ARC:	//逆圆
				case CMD_MOTION_ELLIPSE_ARC:	//顺椭圆
				case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆
				{
					//判断画图缓冲区剩余内存
					if (nBufRest < 26)
					{
						bFull = TRUE;
						break;
					}

					//写入类型
					*(cTempBuff + nPos) = 2;//圆弧
					nPos += 1;

					//写入方向
					*(cTempBuff + nPos) = pToolBuf->cArcDir;
					nPos += 1;

					//写入起点X
					float fStartPoint_X;
					fStartPoint_X = (float)pToolBuf->SStartPoint.dArray[AXIS_X];
					fStartPoint_X -= STotalOffset.dArray[AXIS_X];
					memcpy(cTempBuff + nPos, &fStartPoint_X, 4);
					nPos += 4;

					//写入起点Z
					float fStartPoint_Z;
					fStartPoint_Z = (float)pToolBuf->SStartPoint.dArray[AXIS_Z];
					fStartPoint_Z -= STotalOffset.dArray[AXIS_Z];
					memcpy(cTempBuff + nPos, &fStartPoint_Z, 4);
					nPos += 4;

					//写入终点X
					float fEndPoint_X;
					fEndPoint_X = (float)pToolBuf->SEndPoint.dArray[AXIS_X];
					fEndPoint_X -= STotalOffset.dArray[AXIS_X];
					memcpy(cTempBuff + nPos, &fEndPoint_X, 4);
					nPos += 4;

					//写入终点Z
					float fEndPoint_Z;
					fEndPoint_Z = (float)pToolBuf->SEndPoint.dArray[AXIS_Z];
					fEndPoint_Z -= STotalOffset.dArray[AXIS_Z];
					memcpy(cTempBuff + nPos, &fEndPoint_Z, 4);
					nPos += 4;

					//写入圆心X
					float fCenterPoint_X;
					fCenterPoint_X = (float)pToolBuf->SCenterPoint.dArray[AXIS_X];
					fCenterPoint_X -= STotalOffset.dArray[AXIS_X];
					memcpy(cTempBuff + nPos, &fCenterPoint_X, 4);
					nPos += 4;

					//写入圆心Z
					float fCenterPoint_Z;
					fCenterPoint_Z = (float)pToolBuf->SCenterPoint.dArray[AXIS_Z];
					fCenterPoint_Z -= STotalOffset.dArray[AXIS_Z];
					memcpy(cTempBuff + nPos, &fCenterPoint_Z, 4);
					nPos += 4;

					//移指针
					moveSToolCompBufPointer(DRAW_READ, BUF_AHEAD, ucChannelNum);

					break;

				}

				default:
					break;
				}

				cNum++;
			}
			else
			{
				moveSToolCompBufPointer(DRAW_READ, BUF_AHEAD, ucChannelNum);
			}
			

			//4.加工程序全部读取完成
			if (pToolBuf->bLastLineMark)
			{
				g_SChannel[ucChannelNum].ucChannelStatus = SYS_IDLE;
				bFinish = TRUE;
			}
			

			if (bFull || bFinish)
				break;//Buf写满跳出while
		}

		//Buf已写入目标数量有效轨迹 || Buf写满 || 所有程序读完
		if (cNum == cWriteNum || bFull || bFinish)
			cGetDrawData_Step = 2;
		else
			return 0;
		if (bFinish)
			g_SChannel[0].cSubRunMode = 1;
	}
	//写入目标缓冲区
	case 2:
	{			
		memcpy(pBuffAdress, cTempBuff, 512);
		cGetDrawData_Step = 0;
		
		return cNum;
	}
	default:
		break;
	}


	////0.初始化画图缓冲区内存
	//memset(pBuffAdress, -1, 512);

	////1.判断刀补缓冲区可读容量
	//int16 sRest;
	//sRest = getSToolCompPointerBufRes(DRAW_READ, ucChannelNum);
	////1.G代码运行时可读容量不足，则不进行写入
	//if (getGCodeStatus(GC_ASSIGN, ucChannelNum) == GCODE_RUN && sRest < cWriteNum)
	//{
	//	return 0;
	//}
	//
	////2.写入轨迹数据（刀补缓冲区->画图缓冲区）
	//int16 sCurrByte = 0;	//画图缓冲区已写入字节数
	//int i;
	//for (i = 0; i < cWriteNum; i++)
	//{
	//	//0.画图缓冲区剩余字节数
	//	int16 sBufRest;
	//	sBufRest = 512 - sCurrByte;

	//	//1.刀补缓冲区数据
	//	struct SToolCompensation* pToolBuf;
	//	pToolBuf = getSToolCompPointerByType(DRAW_READ, ucChannelNum);

	//	//2.根据不同轨迹类型进行写入
	//	switch (getBits(pToolBuf->unCommandType, CMD_MOTION_ADDR))
	//	{
	//	//虚线、直线轨迹大小：17字节
	//	case CMD_MOTION_QUICKMOVE:
	//	case CMD_MOTION_LINE:		
	//	case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
	//	case CMD_MOTION_SCREWCUT_VP:
	//	case CMD_MOTION_SCREWCUT:  //螺纹切削
	//	case CMD_MOTION_G33://G33螺纹攻牙循环
	//	{
	//		//判断画图缓冲区剩余内存
	//		if (sBufRest < 17)
	//			return i;

	//		//写入类型
	//		if (getBits(pToolBuf->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)
	//			*(pBuffAdress + sCurrByte) = 0;//虚线
	//		else
	//			*(pBuffAdress + sCurrByte) = 1;//实线
	//		sCurrByte += 1;

	//		//写入起点X
	//		float fStartPoint_X;
	//		fStartPoint_X = (float)pToolBuf->SStartPoint.dArray[AXIS_X];
	//		memcpy(pBuffAdress + sCurrByte, &fStartPoint_X, 4);
	//		sCurrByte += 4;

	//		//写入起点Z
	//		float fStartPoint_Z;
	//		fStartPoint_Z = (float)pToolBuf->SStartPoint.dArray[AXIS_Z];
	//		memcpy(pBuffAdress + sCurrByte, &fStartPoint_Z, 4);
	//		sCurrByte += 4;

	//		//写入终点X
	//		float fEndPoint_X;
	//		fEndPoint_X = (float)pToolBuf->SEndPoint.dArray[AXIS_X];
	//		memcpy(pBuffAdress + sCurrByte, &fEndPoint_X, 4);
	//		sCurrByte += 4;

	//		//写入终点Z
	//		float fEndPoint_Z;
	//		fEndPoint_Z = (float)pToolBuf->SEndPoint.dArray[AXIS_Z];
	//		memcpy(pBuffAdress + sCurrByte, &fEndPoint_Z, 4);
	//		sCurrByte += 4;

	//		break;
	//	}
	//		
	//	//圆弧轨迹大小：26字节
	//	case CMD_MOTION_ARC:		
	//	case CMD_MOTION_ANTI_ARC:	//逆圆
	//	case CMD_MOTION_ELLIPSE_ARC:	//顺椭圆
	//	case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆
	//	{
	//		//判断画图缓冲区剩余内存
	//		if (sBufRest < 26)
	//			return i;

	//		//写入类型
	//		*(pBuffAdress + sCurrByte) = 2;//圆弧
	//		sCurrByte += 1;

	//		//写入方向
	//		*(pBuffAdress + sCurrByte) = pToolBuf->cArcDir;
	//		sCurrByte += 1;

	//		//写入起点X
	//		float fStartPoint_X;
	//		fStartPoint_X = (float)pToolBuf->SStartPoint.dArray[AXIS_X];
	//		memcpy(pBuffAdress + sCurrByte, &fStartPoint_X, 4);
	//		sCurrByte += 4;

	//		//写入起点Z
	//		float fStartPoint_Z;
	//		fStartPoint_Z = (float)pToolBuf->SStartPoint.dArray[AXIS_Z];
	//		memcpy(pBuffAdress + sCurrByte, &fStartPoint_Z, 4);
	//		sCurrByte += 4;

	//		//写入终点X
	//		float fEndPoint_X;
	//		fEndPoint_X = (float)pToolBuf->SEndPoint.dArray[AXIS_X];
	//		memcpy(pBuffAdress + sCurrByte, &fEndPoint_X, 4);
	//		sCurrByte += 4;

	//		//写入终点Z
	//		float fEndPoint_Z;
	//		fEndPoint_Z = (float)pToolBuf->SEndPoint.dArray[AXIS_Z];
	//		memcpy(pBuffAdress + sCurrByte, &fEndPoint_Z, 4);
	//		sCurrByte += 4;

	//		//写入圆心X
	//		float fCenterPoint_X;
	//		fCenterPoint_X = (float)pToolBuf->SCenterPoint.dArray[AXIS_X];
	//		memcpy(pBuffAdress + sCurrByte, &fCenterPoint_X, 4);
	//		sCurrByte += 4;

	//		//写入圆心Z
	//		float fCenterPoint_Z;
	//		fCenterPoint_Z = (float)pToolBuf->SCenterPoint.dArray[AXIS_Z];
	//		memcpy(pBuffAdress + sCurrByte, &fStartPoint_Z, 4);
	//		sCurrByte += 4;

	//		break;

	//	}

	//	}

	//	//3.判断是否为最后一个缓冲区单元
	//	if (pToolBuf->bLastLineMark == TRUE)
	//	{
	//		g_SChannel[ucChannelNum].ucChannelStatus = SYS_IDLE;

	//		return i + 1;
	//	}

	//	//4.移动画图读指针
	//	moveSToolCompBufPointer(DRAW_READ, BUF_AHEAD, ucChannelNum);
	//}

	//return i;
}


