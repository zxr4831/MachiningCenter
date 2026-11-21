#ifndef __RINGBUFF_H
#define __RINGBUFF_H

#include "common.h"

#include "Gcode.h"

//#include "GCodeCommon.h"

#define GCODEPARSEBUF_MAX	20		//SGCodeParseBuf循环缓冲区 容量

#define TOOLCOMPBUF_MAX		30		//SToolCompensationBuf循环缓冲区 容量 

#define BACKWARDBUF_NUM		10		// 刀补缓冲区 倒走预留的缓冲区容量

#define BUF_AHEAD	1	//移动方向：向前（正方向）zxr 沿所选指针的处理方向推进一步
#define BUF_BACK	-1	//移动方向：返回（反方向）zxr 沿所选指针的处理方向退回一步

//2023.5.25 朱稳中 
#define NEW_BUFF  1  //新刀补缓冲区   为0时使用旧的刀补缓冲区 为1时使用新的刀补缓冲区

#define INDEX_DATA_TYPE  int64  //索引的数据类型   //朱稳中 2023.6.15 int32修改为int64

struct SRingBuff  //环形缓冲区结构体    2022.4.19  朱稳中
{
	struct SGCodeParse SGCodeParseBuf[GCODEPARSEBUF_MAX]; //G代码缓冲区

	struct SToolCompensation SToolCompensationBuf[TOOLCOMPBUF_MAX];//刀补缓冲区

	int16 sSGCODE_WRITEIndex; //G代码缓存写指针索引值

	int16 sSGCODE_WRITEdNum;  //G代码已写入个数(且刀补未读取个数）

	int16 sSGcodeReadAheadNum; //G代码已写入个数且刀补可预读个数
#if NEW_BUFF == 0
	int16 sSToolIndex;      //刀补写指针的索引值

	int16 sSToolDealedNum;  //刀补 处理过的数组元素个数

	int16 sLookAheadDealedNum;//前瞻 处理过的数组元素个数

	int16 sInterpolationDealedNum; //插补处理过的数组元素个数 （用于倒走）

#endif  
	//2023.5.25 朱稳中   
	volatile INDEX_DATA_TYPE nSToolIndex;		//刀补写
	volatile INDEX_DATA_TYPE nLookAheadIndex;	//前瞻读写
	volatile INDEX_DATA_TYPE nInterpIndex;		//插补读
	volatile INDEX_DATA_TYPE nBackIndex;		//倒走最大可读索引 // 朱稳中 2023.6.3
	volatile INDEX_DATA_TYPE nDrawIndex;		//画图读

};


enum RingBufPointerType  //环形缓存的指针类型
{
	GCODE_WRITE,			//0 G代码缓存写指针    //2021-8-20
	GCODE_READ,				//1 G代码缓存读指针		 //2021-8-20
	STOOL_WRITE,			//2 刀补的写指针
	LOOKAHEAD_READWRITE,	//3 前瞻的读写指针
	INTERPOLATION_READ,		//4 插补的读指针
	INTERPOLATION_BACK,		//5 插补的反向运动读指针
	STOOL_READ_AHEAD,        //6 刀补预读指针
	DRAW_READ
};
   
extern struct SRingBuff g_SRingBuff[CHANNEL_NUM_MAX];//环形缓冲区

/***********************SGCodeParseBuf循环缓冲区-变量函数*********************************/

//	获取指针的索引值
int16 getSGCodeParsePointerIndex(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	获取数组元素指针(通过索引)
struct SGCodeParse* getSGCodeParsePointerByIndex(short sIndex, uint8 ucChannelNum);

//	获取数组元素指针(通过类型)
struct SGCodeParse* getSGCodeParsePointerByType(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	移动指针
bool moveSGCodeParsePointer(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	获取指针剩余容量
int16 getSGCodeParsePointerBufRes(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	打印数组指针索引，地址信息(调试) 
void printSGCodeParsePointerADDR();

/****************************************************************************************/


/******************SToolCompensationBuf循环缓冲区-变量函数*******************************/

#if NEW_BUFF == 0  //旧刀补缓冲区

//	获取指针的索引值
int16 getSToolCompPointerIndex(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	获取数组元素指针(通过索引)
struct SToolCompensation* getSToolCompPointerByIndex(int16 sIndex, uint8 ucChannelNum);

//	获取数组元素指针(通过类型)
struct SToolCompensation* getSToolCompPointerByType(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	移动指针
bool moveSToolCompBufPointer(enum RingBufPointerType pointerTpye, short sDist, uint8 ucChannelNum);

//	获取指针剩余容量
int16 getSToolCompPointerBufRes(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//  清除前瞻已处理过的数据
bool clearInterpolationDealedNum(uint8 ucChannelNum);

//	打印数组指针索引，地址信息(调试)
void printSToolCompPointerADDR();

//	初始化前瞻指针
bool initLookAheadDealed(uint8 ucChannelNum);

#else //新刀补缓冲区
//	获取指针的索引值
INDEX_DATA_TYPE getSToolCompPointerIndex(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	获取数组元素指针(通过索引)
struct SToolCompensation* getSToolCompPointerByIndex(INDEX_DATA_TYPE nIndex, uint8 ucChannelNum);

//	获取数组元素指针(通过类型)
struct SToolCompensation* getSToolCompPointerByType(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//	移动指针
bool moveSToolCompBufPointer(enum RingBufPointerType pointerTpye, short sDist, uint8 ucChannelNum);

//	获取指针剩余容量
int16 getSToolCompPointerBufRes(enum RingBufPointerType pointerTpye, uint8 ucChannelNum);

//  清除前瞻已处理过的数据
bool clearInterpolationDealedNum(uint8 ucChannelNum);

//	打印数组指针索引，地址信息(调试)
void printSToolCompPointerADDR();

//	初始化前瞻指针
bool initLookAheadDealed(uint8 ucChannelNum);

//初始化缓冲区
int8 initAllBuffer(uint8 ucChannelNum);
#endif

/****************************************************************************************/

//初始化缓冲区
int8 initAllBuffer(uint8 ucChannelNum);

//	返回G代码写缓冲区的地址   冯英华  22.2.28
struct SGCodeParse* getGCodeHeadAddress(uint8 ucChannelNum);
/****************************************************************************************/

int8 getDrawData(int8* pBuffAdress, int8 cWriteNum, int8 ucChannelNum);

#endif // !__RINGBUFF_H

