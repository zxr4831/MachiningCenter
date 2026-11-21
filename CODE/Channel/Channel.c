#include "Channel.h"
#include "G0.h"
#include "G1.h"
#include "G2.h"
#include "G32.h"
#include "G33.h"
#include "GeneralFunctions.h"
#include "Motion.h"
#include "toolAxis.h"
#include"Gcode.h"
#include "ToolCompensation.h"
#include "lookAhead.h"
#include "Config.h"

#if STM32_MODE
#include "dy_link.h"
#endif
//struct SChannel g_SChannel[CHANNEL_NUM_MAX]; //注释原因：企业要以动态内存的形式管理
struct SChannel* g_SChannel = NULL;//通道结构体指针 指向两个通道的首地址

//struct SChannelManager* g_SChannelManager=NULL;//通道信号管理结构体指针
struct SChannelWaitSignalMannager* g_pChannelWaitSignalMannager = NULL;//通道等待信号管理器

//bool g_bSChannelManagerApplyByMain = FALSE;//通道信号管理器只能在Main中运行  Main运行g_SChannelManager置true，此时不允许中断访问g_SChannelManager，Main运行完毕后置为FALSE。



/*************************************************
  函数名称：struct SChannel* initSChannel()
  函数功能: 在动态内存中开辟通道结构体
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
struct SChannel* initSChannel()
{
#if CHANNEL_NUM_MAX>1
	struct SChannel* pChannel = (struct SChannel*)mallocGCode(sizeof(struct SChannel) * CHANNEL_NUM_MAX, ORAM_NUM341);
#else
	struct SChannel* pChannel = (struct SChannel*)mallocGCode(sizeof(struct SChannel) * CHANNEL_NUM_MAX, ORAM_NUM341);
#endif
	if (pChannel == NULL)
	{
		parseGCodeErrorB(9999, GCODE_ERROR_NUM406);
		return NULL;
	}
	return pChannel;
}

///*************************************************
//  函数名称：struct SChannel* initSChannel()
//  函数功能: 在动态内存中开辟通道信号管理结构体
//  输入参数：
//  输出参数：
//  返 回 值：
//  备    注：
//**************************************************/
//struct SChannelManager* initChannelManager()
//{
//	struct SChannelManager*pChannelManager= (struct SChannelManager*)mallocGCode(sizeof(struct SChannelManager), ORAM_NUM342);
//	if (pChannelManager == NULL)
//	{
//		parseGCodeErrorB(9999, GCODE_ERROR_NUM406);
//		return NULL;
//	}
//	return pChannelManager;
//}

/*************************************************
  函数名称：struct SChannelWaitSignalMannager* initChannelWaitSignalManager()
  函数功能: 在动态内存中开辟通道等待信号管理结构体
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
struct SChannelWaitSignalMannager* initChannelWaitSignalManager()
{
	struct SChannelWaitSignalMannager* pChannelWaitSignalMannager = (struct SChannelWaitSignalMannager*)mallocGCode(sizeof(struct SChannelWaitSignalMannager), ORAM_NUM342);
	if (pChannelWaitSignalMannager == NULL)
	{
		parseGCodeErrorB(9999, GCODE_ERROR_NUM406);
		return NULL;
	}
	return pChannelWaitSignalMannager;
}


/*************************************************
  函数名称：initChannel(struct SChannel* pSChannel, uint8 ucChannelNum, uint8* ucAxisMapArray, uint8* ucPWMAxisMapArray ,struct SEncoder* pSSpindleEncoder, struct SEncoder* pSHandleWheelEncoder)
  函数功能  将通道内的成员连接到通道结构体，赋值映射数组，确定主轴编码器
  输入参数：pSChannel			通道结构体指针
			ucChannelNum		通道号
			ucAxisMapArray		映射

  输出参数：
  返 回 值：
  备    注：
**************************************************/
void initChannel(struct SChannel* pSChannel, uint8 ucChannelNum, uint8* ucAxisMapArray, uint8* ucPWMAxisMapArray, struct SEncoder**pEncoderArrays)
{
    int i = 0;
	//通道-》结构体
	pSChannel->ucChannelNum = ucChannelNum; //通道号  从0开始
	pSChannel->pMapAxisArray = ucAxisMapArray;//映射
	pSChannel->pPWMMapAxisArray = ucPWMAxisMapArray;//映射
#if( TURRET_AXIS_TEST==1)
	pSChannel->pChannelToTurretmap = g_SSystemPara.ucChannelToTurretmap[ucChannelNum];//通道0加载通道0与刀塔的映射
#endif
	pSChannel->pSSpindleEncoder = pEncoderArrays[SPINDLE_ENCODER - 1];    //主轴编码器
	// if (pSChannel->pSSpindleEncoder == NULL)
	// return;
	pSChannel->pSCommonEncoder = pEncoderArrays[COMMON_ENCODER - 1];
	pSChannel->pSHandleWheelEncoder = pEncoderArrays[HANDEWHEEL_ENCODER - 1];
	pSChannel->pSSecondSpindleEncoder = pEncoderArrays[SECONDSPINDLE_ANALOGENCODER - 1];    //A轴编码器

	//结构体-》通道
	pSChannel->SPreProcess.pSChannel = pSChannel;
	pSChannel->SG00.pSChannel = pSChannel;
	pSChannel->SG01.pSChannel = pSChannel;
	pSChannel->SG02.pSChannel = pSChannel;
	pSChannel->SG32.pSChannel = pSChannel;
	pSChannel->SG33.pSChannel = pSChannel;

	pSChannel->SIOCommand.pSChannel = pSChannel;
	pSChannel->SG00.pSSingleAxisArray = pSChannel->SSingleAxisArray;
	pSChannel->SHandWheelTest.pSChannel = pSChannel;
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* 初始化独立的程序手轮状态，不复用普通选轴手轮的累计距离。 */
    resetHandwheelSimGroup(&pSChannel->SHandwheelSim);
#endif

	//主轴
	pSChannel->SSpindle.pSChannel = pSChannel;
	//动力刀
	pSChannel->SSecondSpindle.pSChannel = pSChannel;

	spindleInit(&pSChannel->SSpindle, &pSChannel->SSingleAxisArray[SPINDLE_AXIS], AXIS_X);//主轴初始化
	spindleInit(&pSChannel->SSecondSpindle, &pSChannel->SSingleAxisArray[POWERKNIFE_AXIS], AXIS_X);//动力刀初始化

	initToolAxis(pSChannel, &pSChannel->SSingleAxisArray[TOOL_AXIS]);

	pSChannel->SSpindleTest.pSChannel = pSChannel;

	//刀具
	pSChannel->STIOCommand.pSChannel = pSChannel;

	//运行结构体
	pSChannel->SRun.pSChannel = pSChannel;

	pSChannel->lIdealSpindlePulse = 0;

    //for (i = 0; i < AXIS_NUM_MAX; i++)
    //{
    //    if ( ucAxisMapArray[i]>= AXIS_PHYSICS_NUM_MAX)
    //    {
    //        break;
    //    }
    //    pSChannel->SG00.ucFinishedSigned += 1 << i;  //生成G0完成标志
    //}

}

/*************************************************
  函数名称：getChannelStatus(uint8 ucChannelNum)
  函数功能  获取通道状态
  输入参数：ucChannelNum	通道号
  输出参数：
  返 回 值：enum SYSTEMSTATUS	通道（系统）状态
  备    注：
**************************************************/
enum SYSTEMSTATUS getChannelStatus(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return SYS_IDLE;
	}

	return g_SChannel[ucChannelNum].ucChannelStatus;//返回对应通道状态
}

//通道内轴使能
int8 channelAxisEnable(uint8 ucChannelNum)
{
	int i;
	const uint8* ucMapArray;//映射
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX)
		{
			continue;
		}
		if (i == AXIS_C)
		{
			setSpindleEnabledAPI(ucChannelNum, TRUE);
			continue;
		}
		setAxisEnableStatus(ucMapArray[i], TRUE);
	}
	return 1;
}


/*************************************************
  函数名称：waitSignalWrite(uint8 ucTag, uint8 ucFinishedSign)
  函数功能  通道等待信号写入
  输入参数：ucTag	标签
			ucFinishedSign	涉及的通道  用位表示  第0位表示通道0
  输出参数：
  返 回 值：返回数组元素下标
			-1 数组已满
  备    注：
**************************************************/
//int8 waitSignalWrite(uint8 ucTag, uint8 ucFinishedSign)
//{
//	int i;
//	short sEmptyElement = -1;
//	g_bSChannelManagerApplyByMain = TRUE;//mian正在执行
//	//寻找当前的等待信号是否已经有ucTag
//	for (i = CHANNEL_BUFF_SIZE - 1; i >= 0; i--)
//	{
//		if ((1 == (g_SChannelManager->SBuff[i].bApply) && (g_SChannelManager->SBuff[i].bOccupy == 0)))//数组元素非空
//		{
//			if (g_SChannelManager->SBuff[i].UData.SWaitSignal.ucTag == ucTag && g_SChannelManager->SBuff[i].ucType == CHANNEL_SIGNAL_WAIT)
//			{ //①类型为等待信号 ②标签一致 ③数组元素非空   -》》 表示已写入
//				g_SChannelManager->SBuff[i].bOccupy = 1;
//				g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//				return i+1;//已经写入,返回数组下标
//			}
//		}
//		else if ((0 == (g_SChannelManager->SBuff[i].bApply)) && (g_SChannelManager->SBuff[i].bOccupy == 0))
//		{
//			sEmptyElement = i;//记录空的元素
//		}
//	}
//
//	//查找一遍之后发现没有空的元素
//	if (sEmptyElement == -1)
//	{
//		g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//		return -1;
//	}
//
//	//写入新的
//	g_SChannelManager->SBuff[sEmptyElement].ucType = CHANNEL_SIGNAL_WAIT; //设置类型
//	g_SChannelManager->SBuff[sEmptyElement].UData.SWaitSignal.bStatus = FALSE;	//状态  未完成
//	g_SChannelManager->SBuff[sEmptyElement].UData.SWaitSignal.ucCurrentSign
//		= g_SChannelManager->SBuff[sEmptyElement].UData.SWaitSignal.ucFinishedSign
//		= ucFinishedSign;									//标记  (当前标记  完成标记)
//	g_SChannelManager->SBuff[sEmptyElement].UData.SWaitSignal.ucTag = ucTag;	//标签
//	//setbit(g_SChannelManager->>usStatus, sEmptyElement);		//设置为有效数据
//	g_SChannelManager->SBuff[sEmptyElement].bApply = 1;
//	g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//	return sEmptyElement + 1;//返回元素下标(范围 1~MAX)
//}



///*************************************************
//  函数名称：startSignalWrite(uint8 ucTargetChannelNum)
//  函数功能  通道启动信号写入
//  输入参数：ucTargetChannelNum	 目标通道号
//            int8 cFileNameVal  文件名数值
//  输出参数：
//  返 回 值：返回数组元素下标
//			-1 数组已满
//  备    注：
//**************************************************/
//int8 startSignalWrite(uint8 ucTargetChannelNum,int8 cFileNameVal)
//{
//	int i;
//	g_bSChannelManagerApplyByMain = TRUE;//mian正在执行
//	//寻找空的数组元素
//	for (i = 0; i < CHANNEL_BUFF_SIZE; i++)
//	{
//		if (0 == (g_SChannelManager->SBuff[i].bApply) && (g_SChannelManager->SBuff[i].bOccupy == 0))//数组元素非空
//		{
//			break;//打断for循环  在i的位置写入数据
//		}
//	}
//
//	if (i >= CHANNEL_BUFF_SIZE)//没有空的位置
//	{
//		g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//		return -1;
//	}
//
//	g_SChannelManager->SBuff[i].bApply = 1;
//	g_SChannelManager->SBuff[i].ucType = CHANNEL_SIGNAL_START;//通道启动指令
//	g_SChannelManager->SBuff[i].UData.ucChannelNum = ucTargetChannelNum;//写入通道号码
//	g_SChannelManager->SBuff[i].UData.cFileNameVal = cFileNameVal;//写入文件数值
//	g_SChannelManager->SBuff[i].bOccupy = 1;
//	g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//	return i + 1;//返回数组元素下标(范围 1~MAX)
//}



///*************************************************
//  函数名称：checkSignalWrite(uint8 ucTargetChannelNum)
//  函数功能  检测通道状态信号写入
//  输入参数：ucTargetChannelNum	 目标通道号
//  输出参数：
//  返 回 值：返回数组元素下标
//			-1 数组已满
//  备    注：
//**************************************************/
//int8 checkSignalWrite(uint8 ucTargetChannelNum)
//{
//	int i;
//	g_bSChannelManagerApplyByMain = TRUE;//mian正在执行
//	//寻找空的数组元素
//	for (i = 0; i < CHANNEL_BUFF_SIZE; i++)
//	{
//		if (0 == (g_SChannelManager->SBuff[i].bApply) || (g_SChannelManager->SBuff[i].bOccupy == 0))//数组元素非空
//		{
//			break;//打断for循环  在i的位置写入数据
//		}
//	}
//
//	if (i >= CHANNEL_BUFF_SIZE)//没有空的位置
//	{
//		g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//		return -1;
//	}
//
//	g_SChannelManager->SBuff[i].bApply = 1;//设置数组元素非空
//	g_SChannelManager->SBuff[i].ucType = CHANNEL_SIGNAL_CHECK;//通道检查指令
//	g_SChannelManager->SBuff[i].UData.ucChannelNum = ucTargetChannelNum;//写入通道号码
//	g_SChannelManager->SBuff[i].bOccupy = 1;
//	g_bSChannelManagerApplyByMain = FALSE;//mian执行完成
//	return i + 1;//返回数组元素下标(范围 1~MAX)
//}



///*************************************************
//  函数名称：channelSignalApply(int8 cSignalBufNum, uint8 ucChannelNum)
//  函数功能  通道信号执行函数（总函数）
//  输入参数：cSignalBufNum	数组元素下标
//			ucChannelNum	调用该函数的通道号（当前通道号）
//  输出参数：
//  返 回 值：1	已完成 请继续执行
//			-1	输入错误
//			-2	无效的数组元素
//  备    注：
//**************************************************/
//int8 channelSignalApply(int8 cSignalBufNum, uint8 ucChannelNum)
//{
//	cSignalBufNum = cSignalBufNum - 1; //(cSignalBufNum为范围 1~MAX)  将其转化为范围 0~MAX-1
//	if (cSignalBufNum < 0 || cSignalBufNum >= CHANNEL_BUFF_SIZE || ucChannelNum >= CHANNEL_NUM_MAX)
//	{ //检查数组下标合法性  通道号合法性
//		return -1;
//	}
//	if (g_bSChannelManagerApplyByMain == TRUE)//mian正在执行，不允许中断同时执行
//	{
//		return -2;
//	}
//	if (0 == g_SChannelManager->SBuff[cSignalBufNum].bApply)
//	{
//		//数组元素的状态为空  当做已完成
//		return 1;
//	}
//
//	switch (g_SChannelManager->SBuff[cSignalBufNum].ucType)//根据数据的类型执行命令
//	{
//	case CHANNEL_SIGNAL_WAIT:
//		return waitSignalApply(cSignalBufNum, ucChannelNum);
//	case CHANNEL_SIGNAL_START:
//		return startSignalApply(cSignalBufNum);
//	case CHANNEL_SIGNAL_CHECK:
//		return checkSignalApply(cSignalBufNum);
//	default:
//		break;
//	}
//
//}


///*************************************************
//  函数名称：waitSignalApply(int8 cSignalBufNum, uint8 ucChannelNum)
//  函数功能  通道等待信号执行函数   等待所有目标通道进入等待状态 再一起启动
//  输入参数：cSignalBufNum	数组元素下标
//			ucChannelNum	调用该函数的通道号（当前通道号）
//  输出参数：
//  返 回 值：1	已完成 请继续执行
//			-1	输入错误
//  备    注：
//**************************************************/
//int8 waitSignalApply(int8 cSignalBufNum, uint8 ucChannelNum)
//{
//	struct SWaitSignal* pSWaitSignal = &g_SChannelManager->SBuff[cSignalBufNum].UData.SWaitSignal;
//
//	if (0 == g_SChannelManager->SBuff[cSignalBufNum].bApply || g_SChannelManager->SBuff[cSignalBufNum].bOccupy == 0)
//	{
//		//数组元素的状态为空
//		return -1;
//	}
//	if (g_bSChannelManagerApplyByMain == TRUE)//mian正在执行，不允许中断同时执行
//	{
//		return -2;
//	}
//
//	if (pSWaitSignal->bStatus == TRUE)//已经收集到所有通道的等待信号
//	{
//		//当前通道从等待转换成启动
//		setbit(pSWaitSignal->ucCurrentSign, ucChannelNum);
//
//		if (pSWaitSignal->ucCurrentSign == pSWaitSignal->ucFinishedSign)
//		{//所有通道都从等待转换成启动
//			//clrbit(g_SChannelManager->usStatus, cSignalBufNum);//数组元素标记为空
//			g_SChannelManager->SBuff[cSignalBufNum].bApply = 0;
//			g_SChannelManager->SBuff[cSignalBufNum].bOccupy = 0;
//		}
//		return 1;
//	}
//	else//等待的通道还欠缺
//	{
//		clrbit(pSWaitSignal->ucCurrentSign, ucChannelNum);
//
//		if (pSWaitSignal->ucCurrentSign == 0x0)//所有位都置0了  表示当前标签需要等待的通道都在等待状态
//		{
//			pSWaitSignal->bStatus = TRUE;//已经收集到所有通道的等待信号
//
//			//当前通道从等待转换成启动
//			setbit(pSWaitSignal->ucCurrentSign, ucChannelNum);
//			return 1;
//		}
//		return 0;
//	}
//
//}


///*************************************************
//  函数名称：startSignalApply(int8 cSignalBufNum)
//  函数功能  启动通道
//  输入参数：cSignalBufNum	数组元素下标
//  输出参数：
//  返 回 值：1	已完成 请继续执行
//			-1	输入错误
//  备    注：
//**************************************************/
//int8 startSignalApply(int8 cSignalBufNum)
//{
//	uint8 ucChannelNum; // 目标通道号
//	if ((0 == (g_SChannelManager->SBuff[cSignalBufNum].bApply)) || (g_SChannelManager->SBuff[cSignalBufNum].bOccupy == 0))
//	{
//		//数组元素的状态为空
//		return -1;
//	}
//	if (g_bSChannelManagerApplyByMain == TRUE)//mian正在执行，不允许中断同时执行
//	{
//		return -2;
//	}
//	ucChannelNum = g_SChannelManager->SBuff[cSignalBufNum].UData.ucChannelNum;
//
//	//发送启动通道信号
//	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_IDLE)
//	{
//
//   //1.初始化计件
//		initWorkpieceNum(ucChannelNum);
//
//
//	//2.G代码解析模块初始化
//		//2.1G代码解析初始化
//		gcodeInit(ucChannelNum, TRUE, FALSE);
//
//		//2.2添加程序
//		addProgramNode(g_SChannelManager->SBuff[cSignalBufNum].UData.cFileNameVal ,ucChannelNum);
//
//		//2.3G代码解析状态置为GCODE_RUN
//		changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN, TRUE, 0);   //启动对应的通道
//		changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, 0);   //启动对应的通道
//
//	//3..刀补模块初始化
//		toolCompensationInit(ucChannelNum);
//    //4.//速度前瞻初始化
//		lookAheadInit(&g_SChannel[ucChannelNum]);
//	//5.插补初始化
//		interpolationInit(&g_SChannel[ucChannelNum].SPreProcess);
//    //6 缓冲区初始化
//		initAllBuffer(ucChannelNum); //复位  
//	//7.通道状态初始化
//		g_SChannel[ucChannelNum].ucChannelStatus = SYS_RUN;
//	//8.模式
//		g_SChannel[ucChannelNum].cSubRunMode = 1;//0 画图 1 运动 
//    //9.指令管理器对应的单元置为空闲
//		g_SChannelManager->SBuff[cSignalBufNum].bApply = 0;
//		g_SChannelManager->SBuff[cSignalBufNum].bOccupy = 0;
//		return 1;
//	}
//	else
//	{
//		return -2;
//	}
//
//}



///*************************************************
//  函数名称：checkSignalApply(int8 cSignalBufNum)
//  函数功能  检测通道信号，检测目标通道为空闲时，当前通道继续执行
//  输入参数：cSignalBufNum	数组元素下标
//  输出参数：
//  返 回 值：1	已完成 请继续执行
//			-1	输入错误
//  备    注：
//**************************************************/
//int8 checkSignalApply(int8 cSignalBufNum)
//{
//	if ((0 == (g_SChannelManager->SBuff[cSignalBufNum].bApply)) || (g_SChannelManager->SBuff[cSignalBufNum].bOccupy == 0))
//	{
//		//数组元素的状态为空
//		return -1;
//	}
//	if (g_bSChannelManagerApplyByMain == TRUE)//mian正在执行，不允许中断同时执行
//	{
//		return -2;
//	}
//
//	if (getChannelStatus(g_SChannelManager->SBuff[cSignalBufNum].UData.ucChannelNum) == SYS_IDLE)
//	{//检查对应通道的状态  如果为空闲 则返回1
//		//clrbit(g_SChannelManager->usStatus, cSignalBufNum);//数组元素标记为空
//		g_SChannelManager->SBuff[cSignalBufNum].bApply = 0;
//		g_SChannelManager->SBuff[cSignalBufNum].bOccupy = 0;
//		return 1;
//	}
//	return 0;
//}

/*************************************************
  函数名称：waitSignalWrite(uint8 ucTag, uint8 ucFinishedSign)
  函数功能  通道等待信号写入
  输入参数：
			ucFinishedSign	相互等待的通道  用位表示  第0位表示通道0
  输出参数：
  返 回 值：返回数组元素下标
			-1 数组已满
  备    注：
**************************************************/
int8 waitSignalWrite(uint8 ucFinishedSign)
{
	int i;//遍历通道等待信号管理器的索引
	short sEmptyElement = -1;//记录通道等待信号管理器中的空闲单元索引

	//1.寻找当前的等待信号是否已经写入
	for (i = CHANNEL_BUFF_SIZE - 1; i >= 0; i--)
	{
		//1.1数组元素非空
		if (g_pChannelWaitSignalMannager->SBuff[i].bOccupied==TRUE)
		{
			//当前等待信号已经写入
			if (g_pChannelWaitSignalMannager->SBuff[i].ucFinishedSign == ucFinishedSign )
			{ 

				return i+1;//已经写入,返回数组下标
			}
		}
		//1.2 数组元素为空闲
		else 
		{
			sEmptyElement = i;//记录空的元素
		}
	}

	//2.1查找一遍之后发现没有空的元素
	if (sEmptyElement == -1)
	{
		return -1;
	}
	//2.2有空闲的元素
	else
	{
		//2.1 将当前单元置为被占领状态以及将目标标记写入当前单元
		g_pChannelWaitSignalMannager->SBuff[sEmptyElement].bOccupied = TRUE;
		g_pChannelWaitSignalMannager->SBuff[sEmptyElement].ucFinishedSign = ucFinishedSign;
		g_pChannelWaitSignalMannager->SBuff[sEmptyElement].ucCurrentSign = ucFinishedSign;
		return sEmptyElement + 1;//返回元素下标(范围 1~MAX)
	}
}
/*************************************************
  函数名称：channelSignalApply(int8 cSignalBufNum, uint8 ucChannelNum)
  函数功能  通道信号执行函数（总函数）
  输入参数：cSignalBufNum	数组元素下标
			ucChannelNum	调用该函数的通道号（当前通道号）
  输出参数：
  返 回 值：1	已完成 请继续执行
			-1	输入错误
			-2	无效的数组元素
  备    注：
**************************************************/
int8 channelSignalApply(int8 *cSignalBufNum, uint8 ucChannelNum, struct SChannelCommand* psChannelCommand)
{
	if (cSignalBufNum == NULL || psChannelCommand == NULL)
	{
		return -1;
	}

 
	switch (psChannelCommand->ucType)//根据数据的类型执行命令
	{
	case CHANNEL_SIGNAL_WAIT:
		return waitSignalApply(cSignalBufNum, psChannelCommand->SChannelData.ucTargetChannelNum,ucChannelNum);
	case CHANNEL_SIGNAL_START:
		return startSignalApply(psChannelCommand->SChannelData.ucTargetChannelNum,psChannelCommand->SChannelData.cFileNameVal);
	case CHANNEL_SIGNAL_CHECK:
		return checkSignalApply(psChannelCommand->SChannelData.ucTargetChannelNum);
	default:
		break;
	}

}
/*************************************************
  函数名称：int8 checkSignalApply(uint8 ucTargetChannelNum)
  函数功能  检测通道信号，检测目标通道为空闲时，当前通道继续执行
  输入参数：struct SChannelCommand* psChannelCommand	通道指令结构体
  输出参数：
  返 回 值：1	已完成 请继续执行
			-1	输入错误
			0  目标通道不为空闲状态，退出执行
  备    注：
**************************************************/
int8 checkSignalApply(uint8 ucTargetChannelNum)
{
	if (getChannelStatus(ucTargetChannelNum) == SYS_IDLE)
	{//检查对应通道的状态  如果为空闲 则返回1

		return 1;
	}
	return 0;
}

/*************************************************
  函数名称：startSignalApply(int8 cSignalBufNum)
  函数功能  启动通道
  输入参数：ucTargetChannelNum	目标通道号
             int32 cTargetFileNameVal 目标文件名数值
  输出参数：
  返 回 值：1	已完成 请继续执行
			-1	输入错误
  备    注：
**************************************************/
int8 startSignalApply(uint8 ucTargetChannelNum, int32 cTargetFileNameVal)
{
	uint8 ucChannelNum= ucTargetChannelNum; // 目标通道号
	int32  cFileNameVal = cTargetFileNameVal;//目标文件名数值
	//1.如果目标通道为空闲状态
	//发送启动通道信号
	if (getChannelStatus(ucChannelNum) == SYS_IDLE)
	{

		//1.1初始化计件
		initWorkpieceNum(ucChannelNum);

		//1.2.G代码解析模块初始化
			//2.1G代码解析初始化
		gcodeInit(ucChannelNum, TRUE, FALSE);

		//1.2.2添加程序
		if(addProgramNode(cFileNameVal, ucChannelNum) == -1)
		{
			return -1;
		}
		//1.2.3G代码解析状态置为GCODE_RUN
		changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN, TRUE, 0);   //启动对应的通道
		changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, 0);   //启动对应的通道

	    //1.3刀补模块初始化
		toolCompensationInit(ucChannelNum);
		//1.4.//速度前瞻初始化
		lookAheadInit(&g_SChannel[ucChannelNum]);
		//1.5.插补初始化
		interpolationInit(&g_SChannel[ucChannelNum].SPreProcess);
		//1.6 缓冲区初始化
		initAllBuffer(ucChannelNum); //复位  
	//1.7.通道状态初始化
		g_SChannel[ucChannelNum].ucChannelStatus = SYS_RUN;
		//1.8.模式
		g_SChannel[ucChannelNum].cSubRunMode = 1;//0 画图 1 运动 
		if (g_SChannel[ucChannelNum].bSingleLineModeUnFinished == FALSE)
		{
			g_SChannel[ucChannelNum].ucRunCounter = 1;
		}
		return 1;
	}
	else
	{
		return 0;
	}

}

/*************************************************
  函数名称：int8 waitSignalApply(int8* cSignalBufNum, uint8 ucTargetChannelNum, uint8 ucChannelNum)
  函数功能  通道等待信号执行函数   等待所有目标通道进入等待状态 再一起启动
  输入参数：cSignalBufNum	数组元素下标
			ucChannelNum	调用该函数的通道号（当前通道号）
			uint8 ucTargetChannelNum, 相互等待的目标通道
  输出参数：
  返 回 值：1	已完成 请继续执行
            0   未执行完成
			-1	输入错误
  备    注：
**************************************************/
int8 waitSignalApply(int8* cSignalBufNum, uint8 ucTargetChannelNum, uint8 ucChannelNum)
{
	if (cSignalBufNum == NULL )
	{
		return -1;
	}

	//1.如果还没有找到在通道等待信号管理器的写入位置
	if (*cSignalBufNum<=0)
	{
		//1.1 在通道等待信号管理器中寻找可写入的位置
		*cSignalBufNum = waitSignalWrite(ucTargetChannelNum);
		//1.2 若没找到，退出
		if (*cSignalBufNum < 0)
		{
			return 0;
		}
	}
	//2.获取等待信号在通道等待信号管理器中的写入位置
	struct SChannelWaitSignalData* pSWaitSignal = &g_pChannelWaitSignalMannager->SBuff[*cSignalBufNum - 1];
	//2.执行通道等待信号
	//2.1 已经收集到所有通道的等待信号
	if (pSWaitSignal->bStatus==TRUE)
	{
		//2.1.1当前通道由等待转换成运行
		setbit(pSWaitSignal->ucCurrentSign, ucChannelNum);
		//2.1.2若所有目标通道都由等待转换成运行
		if (pSWaitSignal->ucCurrentSign == pSWaitSignal->ucFinishedSign)
		{
			//清空等待信号管理器单元的信息
			memset(pSWaitSignal, 0, sizeof(struct SChannelWaitSignalData));
		}
		return 1;
	}
    //2.2 等待的通道还欠缺
	else
	{
		//2.2.1当前通道标记为进入等待
		clrbit(pSWaitSignal->ucCurrentSign, ucChannelNum);
		//2.2.2 //所有位都置0了  表示当前标签需要等待的通道都在等待状态
		if (pSWaitSignal->ucCurrentSign == 0x0)
		{
			//已经收集到所有通道的等待信号
			pSWaitSignal->bStatus = TRUE;
			//当前通道从等待转换成启动
			setbit(pSWaitSignal->ucCurrentSign, ucChannelNum);
			return 1;
		}
		return 0;
	}

}

/*************************************************
  函数名称：initInclinedYMachinePara(struct SChannel* pSChannel，double dAngleYY1)
  函数功能：初始化倾斜Y轴机参数
  输入参数：pSChannel 通道指针
			dAngleYY1 Y轴与Y1轴间的夹角 单位：弧度（rad）
  输出参数：
  返 回 值：1	完成
			-1  通道错误
			-2  参数错误
  备    注：倾斜Y轴机
**************************************************/
int8 initInclinedYMachinePara(struct SChannel* pSChannel, double dAngleYY1)
{
	//通道为空
	if (pSChannel == NULL)return -1;
	//输入参数错误
	if (dAngleYY1 - PI > EPSINON || dAngleYY1 < EPSINON)return -2;

	pSChannel->dSINAngleYY1 = sin(dAngleYY1);
	pSChannel->dCOSAngleYY1 = cos(dAngleYY1);
	pSChannel->dTANAngleYY1 = tan(dAngleYY1);

	return 1;
}


