#include "common.h"
#include "IOCommand.h"
#include <stdio.h>
#include "GeneralFunctions.h"
#include "time.h"
#include "G1.h" 
#include "G0.h"  
#include "Motion.h"	
#include "Spindle.h"
#include "Channel.h"
#include <string.h>
#include "Config.h"
#include "Gcode.h"
//#include "libGCodeExpandFunc.h"
/**********************************************变量定义*****************************************************/



uint8 g_ucIOBlockedCounter = 0;  //阻塞锁

uint8 g_ucIORunBuff[IO_MAXGROUP_IN_SINGLE_LINE];  //执行IO的缓存

uint8 g_ucIOToolRun;

uint8 g_ucMCodeStatus[CHANNEL_NUM_MAX][MCODE_GROUP_NUM]; // 记录当前M代码

uint8 g_ucMCodeRunStatus[CHANNEL_NUM_MAX][2];  //夹头，切削液状态

#if  !STM32_MODE 

unsigned long AxisEnable[AXIS_PHYSICS_NUM_MAX];//各个轴的使能位


#if !STM32_MODE 
int ioM10M11;	//夹头AIO
int ioM12M13;	//夹头BIO
int ioM08M09;	//切削液IO
int ioM38M39;	//主轴刹车IO
int ioEMERGENCY_STOP = 1; //急停IO  //朱稳中 2023.6.24
uint8 out[30];					//企业的输出IO数组,与企业同名
uint8 attribute[40][3];			//企业的输入IO数组,与企业同名
#endif






uint32 timeji = 0;

int delays_ms(u32 ms, u32* delang)//计时函数1ms
{
	u32 delays_j;
	u32 delays_i;

	if (*delang == 0)
	{
		*delang = timeji;
		if (*delang == 0)
		{
			*delang = 1;

		}
	}



	if ((timeji < *delang) && timeji != 0)//已溢出
	{
		delays_j = 0xffffffff - *delang;
		delays_i = delays_j + timeji;//计算已经延时的时间

		if (delays_i >= ms)//超过目标时间
		{
			*delang = 0;//关闭延时
			return 1;
		}
		else return 0;
	}
	else
	{
		delays_i = *delang + ms;
		if (delays_i < *delang) //将要溢出
		{
			if (timeji < *delang)
			{
				if (timeji >= delays_i)
				{
					*delang = 0;
					return 1;
				}
				else return 0;
			}
			else return 0;
		}
		else
		{
			if (timeji >= (*delang + ms))
			{
				*delang = 0;
				return 1;
			}
			else return 0;
		}
	}

}


#else
#include "io_exc.h"


#if (MACHINE_CONFIG != NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
#define ioM10M11		out[3]	//out3//PG(10)
#define ioM08M09		out[4]	//out4//PG(9)
#else
#define ioM08M09		out[0]	//PG(13)
#define ioM10M11		out[12]	//PE(8)
#define ioM12M13		out[13]	//PC(1)
#endif
#define ioM38M39;		out[10] //out12//PA(10)//主轴刹车IO
#endif 







/*************************************************
  函数名称：findIOType(uint8 ucNum)
  函数功能  根据M代码寻找该M代码的前后作用类型
  输入参数：ucNum	M代码的号码
  输出参数：
  返 回 值：int8 前后作用类型
  备    注：
**************************************************/
int8 findIOType(uint8 ucNum)
{
	switch (ucNum)
	{
	case 3:case 4:case 71: case 50:	case 51:case 39:case M20:case M21:case M22:case M35:case M36:case M01:	//M03 M04 主轴正转反转	M39主轴刹车松开	//暂定IO控制类，20、21、22、35、36为前作用
	case 10:case 11:case 12:case 13:
		return IO_PRE_ACTION;//前作用

	case 5:	case 72: case 52:case 38:case 15:	//M05主轴停	M38主轴刹车锁紧
		return IO_POST_ACTION;//后作用

	default:
		return IO_PRE_ACTION;//缺省：前作用
	}
}

/*************************************************
  函数名称：setMCodeRunStatus(uint8 ucChannelNum, uint8 ucNum,uint8 ucStatus)
  函数功能  设置夹头/切削液状态
  输入参数：uint8 ucChannelNum通道号, ucNum 0夹头,1切削液，ucStatus状态
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 setMCodeRunStatus(uint8 ucChannelNum, uint8 ucNum,uint8 ucStatus)
{
 	g_ucMCodeRunStatus[CHANNEL_NUM_MAX][2] = ucStatus;
	return 1;
}

/*************************************************
  函数名称：getMCodeRunStatus(uint8 ucChannelNum, uint8 ucNum)
  函数功能: 获取夹头/切削液状态
  输入参数：uint8 ucChannelNum通道号, ucNum 0夹头,1切削液
  输出参数：
  返 回 值：状态
  备    注：
**************************************************/
int8 getMCodeRunStatus(uint8 ucChannelNum, uint8 ucNum)
{
 	return g_ucMCodeRunStatus[CHANNEL_NUM_MAX][ucNum];
}

/*************************************************
  函数名称：setIO(uint8 ucNum,UMcode *ucIOcommand)
  函数功能  输入M代码的num  查找该M代码的作用类型 存储到ucIOCommand数组中 （从0开始存放）
  输入参数：ucNum				M代码的号码
			*ucIOCommandArray	M代码结构体数组
			ucSize				数组长度
  输出参数：
  返 回 值：1 成功
			0 数组已满
			-1 指针为空
  备    注：
**************************************************/
int8 setIO(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize)
{
	uint8 ucOffsetRecorder; //记录偏移的次数
	uint8 ucCommandType;	//作用类型  （前后作用）

	if (ucIOCommandArray == NULL)
	{
		return -1;
	}

	ucCommandType = findIOType(ucNum);//获取当前代码的作用类型

	for (ucOffsetRecorder = 0; ucOffsetRecorder < ucSize; ucOffsetRecorder++)
	{
		if (ucIOCommandArray[ucOffsetRecorder] == 0) //从地址读出的数据为0  说明为空
		{
			setBits(ucIOCommandArray[ucOffsetRecorder], ucNum, IO_NUM_ADDR);  //赋值 M代码号
			setBits(ucIOCommandArray[ucOffsetRecorder], (ucCommandType << 7), IO_TYPE_ADDR); //赋值 前后作用类型
			return 1;
		}
		else if (getBits(ucIOCommandArray[ucOffsetRecorder], IO_NUM_ADDR) == ucNum)//已有该M代码
		{
			return 1;
		}
	}

	return 0;//变量已满
}


/*************************************************
  函数名称：isBlockedIO(UMcode ucIOCommand)
  函数功能  根据M代码判断该M代码是否阻塞
  输入参数：ucNum		M代码的号码
  输出参数：
  返 回 值：非0	M代码 为   阻塞M代码
			0   M代码 不是 阻塞M代码
  备    注：
**************************************************/
uint8 isBlockedIO(uint8 ucNum)
{
	switch (ucNum)
	{
	case 3:case 4:case 51:case 50:
	case 10:case 12:case 71:case 35:case 36://夹头锁紧
	case M20:case M21:case M22:case M01:case M39:
		return 1;
	case 5:case 52:case M38:
	case 11:case 13:case 72://夹头锁紧
		return 2;
	default:
		return 0;
	}
}

/*************************************************
  函数名称：isVariableIO(UMcode ucIOCommand)
  函数功能  根据M代码判断该是否IO可变指令
  输入参数：ucNum		M代码的号码
  输出参数：
  返 回 值：非0	M代码 为   可变IO M代码
			0   M代码 不是 不可变IO M代码
  备    注：
**************************************************/
uint8 isVariableIO(uint8 ucNum)
{
	switch (ucNum)
	{
	case M35:case M36:case M20:case M21:case M22:case M01:
		return 1;
	default:
		return 0;
	}
}



/*************************************************
  函数名称：IOStructToBuff
  函数功能：登记IO请求及入队版本，识别停止前排队的旧使能
  输入参数：ucIOCommand - M指令；pSIOCommand - 队列；pIOContrlInfo - 可变IO参数
  输出参数：更新队列和阻塞计数
  返 回 值：1 - 入队；0 - 队列满或参数无效
  备    注：苏向标 2026-10-14
****************************************/

uint8 IOStructToBuff(uint8 ucIOCommand, struct _SIOCommand* pSIOCommand, SIOContrlInfo* pIOContrlInfo)
{
    uint8 ucCounter; /**< 空闲槽索引。 */
    uint32 unSpindleVersion; /**< 本次入队的取消版本快照。 */
    setBits(ucIOCommand, 0x00, IO_TYPE_ADDR);
    if (pSIOCommand == NULL || pSIOCommand->pSChannel == NULL
        || (isVariableIO(ucIOCommand) && pIOContrlInfo == NULL))
    {
        return 0;
    }
    unSpindleVersion = pSIOCommand->pSChannel->SSpindle.unEnableRequestVersion;
    for (ucCounter = 0; ucCounter < IO_MAXGROUP_IN_SINGLE_LINE; ++ucCounter)
    {
        if (pSIOCommand->ucIORunBuff[ucCounter] == 0)
        {
            pSIOCommand->unSpindleRequestVersionArray[ucCounter] = unSpindleVersion;
            pSIOCommand->ucPreAndPostDelayStatus[ucCounter] = 0;
            pSIOCommand->ucIOBlockedCounter += isBlockedIO(ucIOCommand) != 0 ? 1 : 0;
            if (isVariableIO(ucIOCommand))
            {
                pSIOCommand->SVaribleIOInfoArray[ucCounter].pIOContrlInfo = pIOContrlInfo;
                pSIOCommand->SVaribleIOInfoArray[ucCounter].ucStatus = 0;
            }
            pSIOCommand->ucIORunBuff[ucCounter] = ucIOCommand;
            return 1;
        }
    }
    return 0;
}

/*************************************************
  函数名称：ApplyIO(uint8* ucOCommandArray, uint8 ucSize, uint8 ucCommandType, struct _SIOCommand* pSIOCommand)
  函数功能  根据输入的作用类型(前后作用） 使能（放入运行buff中）ucIOcommand数组中的M代码
  输入参数：*ucIOCommandArray	M代码结构体数组
			ucSize				数组长度
			ucCommandType		前/后作用类型
			pSIOCommand			IO结构体指针
  输出参数：
  返 回 值：0	失败  数组指针为空
			1	成功
  备    注：
**************************************************/
uint8 ApplyIO(uint8* ucOCommandArray, uint8 ucSize, uint8 ucCommandType, struct _SIOCommand* pSIOCommand, SIOContrlInfo* pIOContrlInfo)
{
	uint8 ucCounter; //计数器
	uint8 res = 0;
  uint8 ucNum;
	if (ucOCommandArray == NULL)
	{
		return 0;
	}

	for (ucCounter = 0; ucCounter < ucSize; ucCounter++)  //从0开始查找
	{

		if (getBits(ucOCommandArray[ucCounter], IO_TYPE_ADDR) == (ucCommandType << 7))//作用类型符合要求
		{
			ucNum = getBits(ucOCommandArray[ucCounter], IO_NUM_ADDR);
			if (!isVariableIO(ucOCommandArray[ucCounter]) && findIO(ucNum, pSIOCommand->ucIORunBuff, IO_MAXGROUP_IN_SINGLE_LINE) == ucNum)//2024/10/9 提高M代码执行效率的指令应只涉及控制单个IO的M代码指令
			{
				continue;
			}

			if(IOStructToBuff(ucOCommandArray[ucCounter], pSIOCommand, pIOContrlInfo) == 0)//放到BUFF中执行
			{
				return 0;
			}
			checkAndsetIOBlockedBeforG0(ucOCommandArray[ucCounter], pSIOCommand);	//检查IO,并根据部分IO，设置相关的状态变量使得G0运动阻塞 2023/4/6陈敏聪
			switch (getBits(ucOCommandArray[ucCounter], IO_NUM_ADDR)) //熊骏 2024-10-15
			{
			case 3:
			case 4:
			case 5:
				g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_SPINDLE] = getBits(ucOCommandArray[ucCounter], IO_NUM_ADDR);
				break;
			case 8:
			case 9:
				g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_FUILD] = getBits(ucOCommandArray[ucCounter], IO_NUM_ADDR);
				break;
			case 10:
			case 11:
				g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_CHUCK] = getBits(ucOCommandArray[ucCounter], IO_NUM_ADDR);
			}
		}
	}

	return 1;
}

/* 苏向标 2026-10-14：保留原IO执行分支，主轴四类IO由周期调用，完成计数由后台确认。 */
/*************************************************
  函数名称：runIOCore
  函数功能：执行原M代码分支，主轴使能与刹车分支仅由周期路径调用
  输入参数：pSIOCommand - IO队列；ucIOCommand - M代码；pPreAndPostDelayStatus - 阶段；pSVaribleIOInfo - 可变IO参数
  输出参数：更新对应输出、阶段与普通IO的阻塞计数
  返 回 值：1 - 完成；0 - 等待；负值 - 参数或执行异常
  备    注：苏向标 2026-10-14；M38/M39完成计数移交后台runIO确认；其他分支保留原逻辑
****************************************/
static int8 runIOCore(struct _SIOCommand* pSIOCommand, uint8 ucIOCommand,uint8* pPreAndPostDelayStatus, SVaribleIOInfo *pSVaribleIOInfo)
{
	if (pSIOCommand == NULL)
	{
		return -1;
	}
	if (ucIOCommand == 0)//0表示空 不执行  返回
	{
		return 0;
	}

	switch (ucIOCommand)
	{
	case 1:	//2023/8/5	陈敏聪
		if (pSIOCommand->pSChannel->ucChannelStatus != SYS_PAUSE)	//如果不为暂停的话，调用暂停API
		{
			pauseButtonAPI(pSIOCommand->pSChannel->ucChannelNum);
			return 0;
		}
		if (detectM01Command(pSIOCommand, pSVaribleIOInfo))	//检测M01指定的IO是否完成
		{
			if (pSVaribleIOInfo->ucStatus == VARIABLE_IO_ERROR)	//超时报警
			{
				parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM31, pSVaribleIOInfo->pIOContrlInfo->cInputIOIndex);
			}
			else
			{
				runButtonAPI(pSIOCommand->pSChannel->ucChannelNum, FALSE); //M01执行完成，程序暂停切换至运行
				pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1 //临时改动位置，企业需求：检测端口未输入报警后，不按复位按启动即可重新运行 2023/10/24 
			}
			//pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			break;
		}
		return 0;
	case 3:	//M03 
	case 4:	//M04
		//if (g_SApplyTime.unSpindlePreRunTime == 0)
		//{
		//	setSpindleDirAPI(&g_SInterpolationAxisC, ((ucIOCommand == 3)? INP_GO_AHEAD: INP_GO_BACK));
		//	spindleBeginAPI(&g_SInterpolationAxisC);
		//}
		if (delays_ms(g_SSystemPara.unSpindlePreDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindlePreRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			break;
		}
		return 0;

	case 5:	//M05
		//if (g_SApplyTime.unSpindlePreRunTime == 0)
		//{
		//	spindleCommonStopAPI(&g_SInterpolationAxisC);
		//}
		if (delays_ms(g_SSystemPara.unSpindlePostDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindlePostRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			break;
		}
		return 0;

	case 8:	//M08
#if STM32_MODE
		ioM08M09 = IO_SET;	//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
		PGout(9) = IO_SET;	//保证实时输出
#else
		PGout(13) = IO_SET;	//保证实时输出
#endif
#else
		ioM08M09 = IO_SET;
#endif // STM32_MODE
		break;

	case 9:	//M09
#if STM32_MODE
		ioM08M09 = IO_RESET;	//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空

#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
		PGout(9) = IO_RESET;	//保证实时输出
#else
		PGout(13) = IO_RESET;	//保证实时输出
#endif	//(MACHINE_CONFIG != NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)

#else
		ioM08M09 = IO_RESET;
#endif // STM32_MODE
		break;

	case 10://M10
#if STM32_MODE
		if (g_SSystemPara.userset.Chuck_status[0] == 1)//松开为夹紧
		{
			ioM10M11 = IO_SET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
			PIout(4) = IO_SET;	//保证实时输出
#else
			PGout(10) = IO_SET;		//保证实时输出;	
#endif	//(MACHINE_CONFIG != NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)

	    }
		else
		{
			ioM10M11 = IO_RESET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
			PIout(4) = IO_RESET;	//保证实时输出
#else
			PGout(10) = IO_RESET;		//保证实时输出;	
#endif	//(MACHINE_CONFIG != NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)

		}
#else
		ioM10M11 = IO_SET;
#endif // STM32_MODE
		if (delays_ms(g_SSystemPara.unChuckCloseDelayTime, &pSIOCommand->pSChannel->SApplyTime.unChuckAOpenRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			pSIOCommand->pSChannel->bChuckALoose = FALSE; //夹头锁紧 2023/10/14
			setMCodeRunStatus(pSIOCommand->pSChannel->ucChannelNum,0,1); //夹头锁紧
			return 1;
			break;
		}
		
		return 0;

	case 11://M11
		if (pSIOCommand->pSChannel->SSingleAxisArray[AXIS_C].dCurrentSpeed * (60.0 * 1000) / DEGREE_OF_CIRCLE > g_SSystemPara.SpindleChuck[0])//主轴转速大于指定转速开启检测，主轴执行到M11夹头松开时进行报警处理 2023/10/14 洪冠鹏  //2026.7.1将主轴理论转速修改为实际转速
		{
			parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM149, 9999);
			return 1;
		}
#if STM32_MODE
		if (g_SSystemPara.userset.Chuck_status[0] == 1)//松开为夹紧
		{
			ioM10M11 = IO_RESET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
			PIout(4) = IO_RESET;	//保证实时输出
#else
			PGout(10) = IO_RESET;		//保证实时输出;	
#endif	//(MACHINE_CONFIG != NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)

		}
		else
		{
			ioM10M11 = IO_SET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
			PIout(4) = IO_SET;	//保证实时输出
#else
			PGout(10) = IO_SET;		//保证实时输出;	
#endif	//(MACHINE_CONFIG != NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
		}
#else
		ioM10M11 = IO_RESET;
#endif // STM32_MODE		
		if (delays_ms(g_SSystemPara.unChuckOpenDelayTime, &pSIOCommand->pSChannel->SApplyTime.unChuckAOpenRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			pSIOCommand->pSChannel->bChuckALoose = TRUE; //夹头松夹 2023/10/14
			break;
		}
		return 0;
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)
	case 12://M12
#if STM32_MODE
		if (g_SSystemPara.userset.Chuck_status[0] == 1)//松开为夹紧
		{
			ioM12M13 = IO_SET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
			PGout(3) = IO_SET;	//保证实时输出


		}
		else
		{
			ioM12M13 = IO_RESET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
			PIout(3) = IO_RESET;	//保证实时输出

		}
#else
		ioM12M13 = IO_SET;
#endif // STM32_MODE
		if (delays_ms(g_SSystemPara.unChuckCloseDelayTime, &pSIOCommand->pSChannel->SApplyTime.unChuckBOpenRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			pSIOCommand->pSChannel->bChuckBLoose = FALSE; //夹头锁紧状态 2023/10/14
			break;
		}

		return 0;
		break;

	case 13://M13
		if (pSIOCommand->pSChannel->SSingleAxisArray[AXIS_C].dSpindleSpeed * (60.0 * 1000) / DEGREE_OF_CIRCLE > g_SSystemPara.SpindleChuck[1])//主轴转速大于指定转速开启检测，主轴执行到M13夹头松开时进行报警处理 2023/10/14 洪冠鹏
		{
			parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM149, 9999);
			return 1;
		}
#if STM32_MODE
		if (g_SSystemPara.userset.Chuck_status[0] == 1)//松开为夹紧
		{
			ioM12M13 = IO_RESET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
			PGout(3) = IO_RESET;	//保证实时输出


		}
		else
		{
			ioM12M13 = IO_SET;		//保持IO持续输出，保证下一周期不会被企业的IO管理函数清空
			PIout(3) = IO_SET;	//保证实时输出

		}
#else
		ioM12M13 = IO_RESET;
#endif // STM32_MODE
		if (delays_ms(g_SSystemPara.unChuckCloseDelayTime, &pSIOCommand->pSChannel->SApplyTime.unChuckBCloseRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			pSIOCommand->pSChannel->bChuckBLoose = TRUE; //夹头松夹状态 2023/10/14
			return 1;
			break;
		}
		return 0;
		break;
		break;
#else

	case 12:
		pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
		break;
	case 13:
		pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
		break;

#endif
	case 14://M14  主轴使能
		//pSIOCommand->pSChannel->SSpindle.toDisableAxis = FALSE;
		//setSpindleEnabled(&pSIOCommand->pSChannel->SSpindle, TRUE);
		//if (delays_ms(g_SSystemPara.unChuckOpenDelayTime, &pSIOCommand->pSChannel->SApplyTime.unChuckOpenRunTime) == 1)//延时是否完成
		//{
		//	break;
		//}
		//return 0;
		return IOSetSpindleEnabled(&pSIOCommand->pSChannel->SSpindle,TRUE);

	case 15://M15

		return IOSetSpindleEnabled(&pSIOCommand->pSChannel->SSpindle, FALSE);
		//setSpindleEnabled(&pSIOCommand->pSChannel->SSpindle, FALSE);
		//break;

	case 16://M16
		break;

	case 17://M17
		break;
	case 38:	//刹车锁紧
		switch (*pPreAndPostDelayStatus)
		{
		case 0:
		{
			//开主轴刹车输出 2025.11.7修改M38流程
			if (isZero(pSIOCommand->pSChannel->SSpindleTest.dSpindleAvgSpeed) && pSIOCommand->pSChannel->SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED)
			{
#if STM32_MODE
					ioM38M39 = IO_SET;
					PAout(10) = IO_SET; // PG10
#else
					ioM38M39 = IO_SET;
#endif
					*pPreAndPostDelayStatus = 1;
			}
			else
			{
				spindleCommonStop(&pSIOCommand->pSChannel->SSpindle); // 先让主轴停
				break;
			}
		}

		case 1:
		{
			//前延时
			if (delays_ms(g_SSystemPara.unSpindleLockPreDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindleLockRunTime) == 1)//延时是否完成
			{
				*pPreAndPostDelayStatus = 2;
				
			}
			else
			{
				break;
			}
		}

		case 2:
		{
			//关闭主轴使能
			if (setSpindleEnabled(&pSIOCommand->pSChannel->SSpindle, FALSE) == 1)
			{
				*pPreAndPostDelayStatus = 3;
			}
			else
			{
				break;
			}
		}

		case 3:
		{
			//后延时
			if (delays_ms(g_SSystemPara.unSpindleLockPostDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindleLockRunTime) == 1)//延时是否完成
			{
				/* 苏向标 2026-10-14：完成后由后台应答入口统一扣减阻塞计数。 */
				*pPreAndPostDelayStatus = 0;
				pSIOCommand->pSChannel->SSpindle.cSpindleClampStatus = STATUS_CLAMPED;
				return 1;
			}
			break;
		}
		}
		return 0;
		break;
	case 39:  //刹车松开
		switch (*pPreAndPostDelayStatus)
		{
		case 0:
		{
			// 主轴使能 2025.11.7修改M39流程
			if (setSpindleEnabled(&pSIOCommand->pSChannel->SSpindle, TRUE) == 1)
			{
				*pPreAndPostDelayStatus = 1;
				
			}
			else
			{
				break;	
			}
			
		}

		case 1:
		{
			//延时
			if (delays_ms(g_SSystemPara.unSpindleUnLockPreDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindleUnLocktRunTime) == 1) // 延时是否完成
			{
				*pPreAndPostDelayStatus = 2;
				
			}
			break;	
		}
		case 2:
		{
			//关主轴刹车输出
#if STM32_MODE
			ioM38M39 = IO_RESET;
			PAout(10) = IO_RESET;	// PG10
#else
			ioM38M39 = IO_RESET;
#endif
			* pPreAndPostDelayStatus = 3;

		}

		case 3:
		{
			//延时
			if (delays_ms(g_SSystemPara.unSpindleUnLockPostDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindleUnLocktRunTime) == 1)//延时是否完成
			{
				/* 苏向标 2026-10-14：完成后由后台应答入口统一扣减阻塞计数。 */
				*pPreAndPostDelayStatus = 0;
				pSIOCommand->pSChannel->SSpindle.cSpindleClampStatus = STATUS_UNCLAMPED;
				return 1;
			}
			break;
		}
		}
		return 0;
		break;
	case M20:case M21:case M22:
		if (runM20M21M22Command(pSIOCommand, pSVaribleIOInfo))
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			return 1;
			break;
		}
		return 0;
		break;
	case 23://M23
		break;

	case 24://M24
		break;

	case 25://M25
		break;

	case 32://M32
		break;

	case 33://M33
		break;

	case 35:case 36://M35
		if (runM35Command(pSIOCommand, pSVaribleIOInfo))
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			return 1;
			break;
		}
		return 0;
		break;

	case 41://M41
		break;

	case 42://M42
		break;
	case 43://M43
		break;

	case 44://M44
		break;
	case 50:
	case 51://M51
		if (delays_ms(g_SSystemPara.unSpindlePreDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindlePreRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			return 1;
			break;
		}
		return 0;
		break;

	case 52://M52
		if (delays_ms(g_SSystemPara.unSpindlePostDelayTime, &pSIOCommand->pSChannel->SApplyTime.unSpindlePostRunTime) == 1)//延时是否完成
		{
			pSIOCommand->ucIOBlockedCounter -= 1;//阻塞锁-1
			return 1;
			break;
		}
		return 0;
		break;
		break;

	case 53://M53
		break;

	case 54://M54
		break;

	case 55://M55
		break;

	case 56://M56
		break;

	case 57://M57
		break;

	case 58://M58
		break;

	case 59://M59
		break;

	case 60://M60
		break;

	case 61://M61
		break;

	case 62://M62
		break;

	case 63://M63
		break;

	case 64://M64
		break;

	case 65://M65
		break;

	case 66://M66
		break;

	case 71://M66
		break;
	case 72://M66

		break;
	default:
		break;
		 //return runIOExpand(pSIOCommand,ucIOCommand, pPreAndPostDelayStatus, pSVaribleIOInfo);
	}
	return 1;
}


/* 苏向标 2026-10-14：独立推进主轴停止，避免停止依赖启动使能完成。 */
/*************************************************
  函数名称：runSpindleIOCore
  函数功能：在运动周期校验旧使能请求版本，不进入临界区
  输入参数：pSIOCommand - 队列；ucIOCommand - M指令；pPreAndPostDelayStatus - 槽状态；pSVaribleIOInfo - 可变IO参数
  输出参数：更新对应IO状态
  返 回 值：1 - 完成；0 - 等待；负值 - 异常
  备    注：苏向标 2026-10-14
****************************************/

static int8 runSpindleIOCore(struct _SIOCommand* pSIOCommand, uint8 ucIOCommand, uint8* pPreAndPostDelayStatus, SVaribleIOInfo* pSVaribleIOInfo)
{
    uint8 ucCounter; /**< 对应入队槽。 */
    int8 cResult; /**< 指令执行结果。 */
    if (pSIOCommand == NULL || pSIOCommand->pSChannel == NULL)
    {
        return -1;
    }
    if (ucIOCommand != 14 && ucIOCommand != 39)
    {
        return runIOCore(pSIOCommand, ucIOCommand, pPreAndPostDelayStatus, pSVaribleIOInfo);
    }
    for (ucCounter = 0; ucCounter < IO_MAXGROUP_IN_SINGLE_LINE; ++ucCounter)
    {
        if (pPreAndPostDelayStatus == &pSIOCommand->ucPreAndPostDelayStatus[ucCounter]
            && pSIOCommand->unSpindleRequestVersionArray[ucCounter] != pSIOCommand->pSChannel->SSpindle.unEnableRequestVersion)
        {
            if (ucIOCommand == 39)
            {
                pSIOCommand->pSChannel->SSpindle.cSpindleClampStatus =
                    (*pPreAndPostDelayStatus >= 3) ? STATUS_UNCLAMPED : STATUS_CLAMPED;
            }
            *pPreAndPostDelayStatus = 0;
            return 1;
        }
    }
    cResult = runIOCore(pSIOCommand, ucIOCommand, pPreAndPostDelayStatus, pSVaribleIOInfo);
    return cResult;
}

/*************************************************
  函数名称：runIO
  函数功能：后台执行普通 IO；主轴相关 IO 只发布槽位并等待应答
  输入参数：pSIOCommand - IO结构；ucIOCommand - 指令；pPreAndPostDelayStatus - 阶段；pSVaribleIOInfo - 可变IO信息
  输出参数：仅发布主轴槽位，不改使能输出
  返 回 值：1 - 完成；0 - 等待；负值 - 参数错误
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
int8 runIO(struct _SIOCommand* pSIOCommand, uint8 ucIOCommand, uint8* pPreAndPostDelayStatus, SVaribleIOInfo* pSVaribleIOInfo)
{
    uint8 ucCounter; /**< 待提交的队列槽号。 */
    int8 cSlot; /**< 单字节槽位快照。 */
    if (pSIOCommand == NULL || pSIOCommand->pSChannel == NULL)
    {
        return -1;
    }
    if (ucIOCommand != 14 && ucIOCommand != 15 && ucIOCommand != 38 && ucIOCommand != 39)
    {
        return runIOCore(pSIOCommand, ucIOCommand, pPreAndPostDelayStatus, pSVaribleIOInfo);
    }
    for (ucCounter = 0; ucCounter < IO_MAXGROUP_IN_SINGLE_LINE; ++ucCounter)
    {
        if (pPreAndPostDelayStatus == &pSIOCommand->ucPreAndPostDelayStatus[ucCounter])
        {
            cSlot = pSIOCommand->cSpindleIOSlot;
            if (pSIOCommand->ucIORunBuff[ucCounter] != ucIOCommand)
            {
                return 1; /* 复位已清理，不重新提交旧槽位。 */
            }
            if (cSlot == -(int8)(ucCounter + 1))
            {
                if (isBlockedIO(ucIOCommand) && pSIOCommand->ucIOBlockedCounter != 0)
                {
                    --pSIOCommand->ucIOBlockedCounter;
                }
                pSIOCommand->cSpindleIOSlot = 0;
                return 1;
            }
            if (cSlot == 0)
            {
#if STM32_MODE
                __DMB(); /* 发布前完成数据写入；不屏蔽任何中断。 */
#endif
                pSIOCommand->cSpindleIOSlot = (int8)(ucCounter + 1);
            }
            return 0;
        }
    }
    return -1; /* 不接受生命周期未知的临时槽位指针。 */
}

/*************************************************
  函数名称：processSpindleIORequest
  函数功能：周期内推进一个已发布主轴 IO；复位期间不执行普通 IO
  输入参数：pSIOCommand - 通道 IO 结构
  输出参数：执行结果通过槽位符号应答
  返 回 值：无
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
void processSpindleIORequest(struct _SIOCommand* pSIOCommand)
{
    int8 cSlot; /**< 后台发布的槽号快照。 */
    uint8 ucSlot; /**< 校验后的数组下标。 */
    uint8 ucCommand; /**< 槽位指令。 */
    if (pSIOCommand == NULL || pSIOCommand->pSChannel == NULL)
    {
        return;
    }
    cSlot = pSIOCommand->cSpindleIOSlot;
    if (cSlot <= 0 || cSlot > IO_MAXGROUP_IN_SINGLE_LINE)
    {
        return;
    }
#if STM32_MODE
    __DMB(); /* 已看到请求，再读取该槽位数据。 */
#endif
    ucSlot = (uint8)(cSlot - 1);
    ucCommand = pSIOCommand->ucIORunBuff[ucSlot];
    if (ucCommand != 14 && ucCommand != 15 && ucCommand != 38 && ucCommand != 39)
    {
        pSIOCommand->cSpindleIOSlot = 0;
        return;
    }
    /* 苏向标 2026-10-14：执行前重新读取共享急停输入、极性参数和报警，不沿用后台发布时的条件。 */
    if (ioEMERGENCY_STOP == !g_SSystemPara.ucEmergencyStopType
        || pSIOCommand->pSChannel->ucChannelStatus == SYS_RESET
        || pSIOCommand->pSChannel->ucChannelStatus == SYS_EMERGENCY_STOP
        || isErrorHasBeenWritten())
    {
        return;
    }
    if (runSpindleIOCore(pSIOCommand, ucCommand, &pSIOCommand->ucPreAndPostDelayStatus[ucSlot],
        &pSIOCommand->SVaribleIOInfoArray[ucSlot]) == 1)
    {
#if STM32_MODE
        __DMB(); /* 先完成输出和阶段更新，再发布完成应答。 */
#endif
        pSIOCommand->cSpindleIOSlot = -cSlot;
    }
}

/*************************************************
  函数名称：runIOBuff(struct _SIOCommand* pSIOCommand)
  函数功能  遍历buff  执行数组元素不为0的M代码  若M代码执行完成 则初始化对应的数组元素（置0） 下一次则不会执行
  输入参数：pSIOCommand	IO结构体指针
  输出参数：
  返 回 值：1	执行完成
  备    注：
**************************************************/
uint8 runIOBuff(struct _SIOCommand* pSIOCommand)
{
	uint8 ucCounter; //计数器

	//if(getMCodeRunStatus(pSIOCommand->pSChannel->ucChannelNum,0) == 0) //2026.7.1 增加夹头松开检测,由于MDI模式下，执行M11指令，重新自动执行没有继承，故根据标志位判断
	//{
	//	g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_CHUCK] = M11; //M代码相应状态置为夹头松开M11
	//	if (pSIOCommand->pSChannel->SSingleAxisArray[AXIS_C].dCurrentSpeed * (60.0 * 1000) / DEGREE_OF_CIRCLE > g_SSystemPara.SpindleChuck[0])//主轴转速大于指定转速开启检测，主轴执行到M11夹头松开时进行报警处理
	//	{
	//		parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM149, 9999);
	//		return 1;
	//	}
	//}
	//else
	//{
	//	g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_CHUCK] = M10; //M代码相应状态置为夹头夹紧M10
	//}

	if(getMCodeRunStatus(pSIOCommand->pSChannel->ucChannelNum,1) ==1) //增加切削液M代码状态切换
	{
		g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_FUILD] = 8; //M代码相应状态置为切削液开启M8
	}
	else
	{
		g_ucMCodeStatus[pSIOCommand->pSChannel->ucChannelNum][MCODE_GROUP_FUILD] = 9; //M代码相应状态置为切削液关闭M9
	}

	for (ucCounter = 0; ucCounter < IO_MAXGROUP_IN_SINGLE_LINE; ucCounter++)  //从0开始查找
	{
		if (pSIOCommand->ucIORunBuff[ucCounter] != 0)  //不为0  非空  有效的M代码
		{
			if (runIO(pSIOCommand, pSIOCommand->ucIORunBuff[ucCounter], &(pSIOCommand->ucPreAndPostDelayStatus[ucCounter]),&pSIOCommand->SVaribleIOInfoArray[ucCounter]) == 1)//执行IO指令  
			{
				if (isVariableIO(pSIOCommand->ucIORunBuff[ucCounter]))
				{
					pSIOCommand->SVaribleIOInfoArray[ucCounter].pIOContrlInfo = NULL;
					pSIOCommand->SVaribleIOInfoArray[ucCounter].ucStatus = 0;
				}
				pSIOCommand->ucIORunBuff[ucCounter] = 0;//返回值为1时执行完成 初始化数组元素
			}
		}
	}

	if (ioEMERGENCY_STOP == !g_SSystemPara.ucEmergencyStopType)
	{//急停判断//朱稳中 2023.6.24
		emergencyStopAPI(pSIOCommand->pSChannel->ucChannelNum);
	}
	return 1;
}


/*************************************************
  函数名称：findIO(uint32 ucNum,UMcode *ucIOCommandArray,uint8 ucSize)
  函数功能  寻找组号为unGroupNum的 IO  寻找到返回不为0的结构体 否则返回全部位为0的结构体
  输入参数：ucNum		M代码的号码
			ucIOcommand	M代码结构体数组
			ucSize		数组长度
  输出参数：int8 找到的M指令号码
			0    未找到
			-1	 数组指针为空
  返 回 值：
  备    注：
**************************************************/
int8 findIO(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize)
{
	uint8 ucCounter; //计数器
    uint8 Num;
	if (ucIOCommandArray == NULL)
	{
		return -1;
	}
	ucNum = getBits(ucNum, IO_NUM_ADDR);

	for (ucCounter = 0; ucCounter < ucSize; ucCounter++)  //从0开始查找
	{
		if (getBits(ucIOCommandArray[ucCounter], IO_NUM_ADDR) == ucNum)  //判断是否为目标
		{
			return getBits(ucIOCommandArray[ucCounter], IO_NUM_ADDR);
		}
	}

	return 0;//未找到
}


/*************************************************
  函数名称：findIOPosition(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize, uint8* ucPos)
  函数功能  寻找组号为unGroupNum的 IO  寻找到返回不为0的结构体 否则返回全部位为0的结构体
  输入参数：ucNum		M代码的号码
			ucIOcommand	M代码结构体数组
			ucSize		数组长度
  输出参数：IO对应具体位置
  返 回 值：对应M代码或者0未找到
  备    注：
**************************************************/
int8 findIOPosition(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize, uint8* ucPos)
{
	uint8 ucCounter; //计数器
    uint8 Num;
	if (ucIOCommandArray == NULL)
	{
		return -1;
	}
	ucNum = getBits(ucNum, IO_NUM_ADDR);

	for (ucCounter = 0; ucCounter < ucSize; ucCounter++)  //从0开始查找
	{
		if (getBits(ucIOCommandArray[ucCounter], IO_NUM_ADDR) == ucNum)  //判断是否为目标
		{
			*ucPos = ucCounter;
			return getBits(ucIOCommandArray[ucCounter], IO_NUM_ADDR);
		}
	}

	return 0;//未找到
}

/*************************************************
  函数名称：spindleStatusInBuff()
  函数功能  寻找组号为unGroupNum的 IO  寻找到返回不为0的结构体 否则返回全部位为0的结构体
  输入参数：
  输出参数：
  返 回 值：int8 最后找到的M03 M04 M05的值    （这意味着  返回的是最后放入buff中的M代码）（一定条件下 有坑！）
  备    注：
**************************************************/
int8 checkSpindleStatus(uint8* ucIOCommandArray, uint8 ucSize)
{
	uint8 ucCounter; //计数器
	int8 cRes = 0;
	for (ucCounter = 0; ucCounter < ucSize; ucCounter++)  //从0开始查找
	{
		if (ucIOCommandArray[ucCounter] == 3 || ucIOCommandArray[ucCounter] == 4 || ucIOCommandArray[ucCounter] == 5)  //判断是否为目标
		{
			cRes = ucIOCommandArray[ucCounter];
			setBits(cRes, 0x00, IO_TYPE_ADDR);
		}
	}
	return cRes;
}


/*************************************************
  函数名称：spindleStatusInBuff()
  函数功能  寻找组号为unGroupNum的 IO  寻找到返回不为0的结构体 否则返回全部位为0的结构体
  输入参数：*ucIOCommandArray	数组指针
			ucSize				元素个数
  输出参数：0 无阻塞
			1 前阻塞
			2 后阻塞
			3 前后都阻塞
  返 回 值：int8 最后找到的M03 M04 M05的值    （这意味着  返回的是最后放入buff中的M代码）（一定条件下 有坑！）
  备    注：
**************************************************/
uint8 isBlockedInCommand(uint8* ucIOCommandArray, uint8 ucSize)
{
	uint8 ucCounter; //计数器
	uint8 ucRes = 0;
	uint8 ucReturnVal = 0;
	for (ucCounter = 0; ucCounter < ucSize; ucCounter++)  //从0开始查找
	{
		uint8 uctemp=ucIOCommandArray[ucCounter];
		setBits(ucIOCommandArray[ucCounter], 0x00, IO_TYPE_ADDR);//删除其前后作用类型	//陈敏聪	2023/7/11增
		ucRes = isBlockedIO(ucIOCommandArray[ucCounter]);
		ucIOCommandArray[ucCounter] =uctemp;
		ucReturnVal = ucReturnVal | ucRes;
	}
	return ucReturnVal;
}


/*************************************************
  函数名称：clearIOCommand(uint8* ucIOCommandArray, uint8 ucCommandType, uint8 ucSize)
  函数功能  清除目标缓存 作用类型为ucCommandType的IO
  输入参数：*ucIOCommandArray	数组指针
			ucCommandType		作用类型
			ucSize				元素个数
  输出参数：*ucIOCommandArray	数组指针
  返 回 值：int8 最后找到的M03 M04 M05的值    （这意味着  返回的是最后放入buff中的M代码）（一定条件下 有坑！）
  备    注：
**************************************************/
int8 clearIOCommand(uint8* ucIOCommandArray, uint8 ucCommandType, uint8 ucSize)
{
	int i;
	if (ucIOCommandArray == NULL)
	{
		return  -1;
	}

	switch (ucCommandType)//根据作用类型不同
	{
	case IO_PRE_ACTION://清除前作用类型
	case IO_POST_ACTION://清除后作用类型

		for ( i = 0; i < ucSize; i++)
		{

			if (getBits(ucIOCommandArray[i],IO_TYPE_ADDR)>>7 == ucCommandType )//判断作用类型
			{
				ucIOCommandArray[i] = 0;
			}
		}

		break;
	case IO_ALL_ACTION://清除所有作用类型的IO

		memset(ucIOCommandArray, 0, sizeof(uint8) * ucSize);
		break;
	default:
		return -2;
	}

	return 1;
}


/*************************************************
  函数名称：hasIOCommand(uint8* ucIOCommandArray, uint8 ucSize)
  函数功能  查询是否有M代码
  输入参数：*ucIOCommandArray	数组指针
			ucSize				元素个数
  输出参数：
  返 回 值：FALSE	无M代码
			TRUE	存在M代码
  备    注：
**************************************************/
bool hasIOCommand(uint8* ucIOCommandArray, uint8 ucSize)
{
	int i = 0;
	if (ucIOCommandArray == NULL)
	{
		return FALSE;
	}

	for (i = 0; i < ucSize; i++)
	{
		if (ucIOCommandArray[i] != 0)//存在一个不为0的M代码  说明存在M代码  返回真
		{
			return TRUE;
		}
	}
	return FALSE;// 没有找到M代码
}




/*************************************************
  函数名称：getOrSetMCodeStatus(uint8 ucGroupNum, uint8* pVal, bool bIsWrite)
  函数功能  写入或获取M代码状态
  输入参数：ucChannelNum	通道号
			ucGroupNum		M代码组号
			*pVal			写入时为输入参数
			bIsWrite		是否为写入
  输出参数：*pVal			读出时为传出参数
  返 回 值：
  备    注：
**************************************************/
int8 getOrSetMCodeStatus(uint8 ucChannelNum, uint8 ucGroupNum, uint8* pVal, bool bIsWrite) // 读或写
{
	if (ucGroupNum >= MCODE_GROUP_NUM)
	{
		return -1;
	}
	if (bIsWrite)//写入
	{
		g_ucMCodeStatus[ucChannelNum][ucGroupNum] = *pVal;
	}
	else // 读出
	{
		*pVal = g_ucMCodeStatus[ucChannelNum][ucGroupNum];
	}
	return 1;
}

/*************************************************
  函数名称：checkAndsetIOBlockedBeforG0(uint8 ucNum,struct _SIOCommand* pSIOCommand)
  函数功能：检查IO,并根据部分IO，设置相关的状态变量使得G0运动阻塞
  输入参数：ucNum IO号
  输出参数：pSIOCommand->pSChannel->SSpindle.cSpindleClampStatus 主轴刹车锁紧状态
  返 回 值：
  备    注：2023/4/6 CMC增
**************************************************/
void checkAndsetIOBlockedBeforG0(uint8 ucNum, struct _SIOCommand* pSIOCommand)
{
	setBits(ucNum, 0x00, IO_TYPE_ADDR);//删除其前后作用类型
	if (ucNum == M38 || ucNum == M39)	//主轴刹车锁紧或松开命令 M38/M39
	{
		if (ucNum == M39)	//M39
		{
			pSIOCommand->pSChannel->SSpindle.cSpindleClampStatus = STATUS_GET_UNCLAMP_INSTRUCTION;
		}

		if (ucNum == M38)	//M38
		{
			pSIOCommand->pSChannel->SSpindle.cSpindleClampStatus = STATUS_GET_CLAMP_INSTRUCTION;
		}
	}
}

//上料状态机
#define FEED_INITATION				0
#define FEED_OVERTIME_INITATION		1
#define	FEED_OVERTIME_MOVEBACK		2
#define	FEED_OPEN_OUTPUTIO			3
#define	FEED_CHECK_INTPUTIO			4
#define	FEED_OVERTIME_INTPUTIO		5
#define	FEED_FINISH_CLOSE_IO		6
#define	FEED_FINISH_RELEASE			7
#define	FEED_ERROR_WARNING			50

#define CONFIG_FINISH_CLOSEIO		0	//选项，运行结束后是否关闭IO

#define	FEED_IO_TEST 0					//VS测试选项

/*************************************************
  函数名称：cancelWaitingInputIO
  函数功能：撤销本通道尚未完成的M35/M36等待输出
  输入参数：pSIOCommand为待清理的IO队列，允许为空
  输出参数：撤销有效输出，终止活动等待状态并清除等待计时
  返 回 值：无
  备    注：在释放程序缓冲和清空队列前调用；不撤销正常成功的保持输出
****************************************/
void cancelWaitingInputIO(struct _SIOCommand* pSIOCommand)
{
	/* out是物理输出共享数组，仅改写本通道活动等待对应的已映射端口。 */
	if (pSIOCommand == NULL || pSIOCommand->pSChannel == NULL)
	{
		return;
	}
	for (uint8 ucSlot = 0; ucSlot < IO_MAXGROUP_IN_SINGLE_LINE; ucSlot++)
	{
		SVaribleIOInfo* pVariable = &pSIOCommand->SVaribleIOInfoArray[ucSlot];
		/* 非等待指令和未发布参数不参与撤销。 */
		if ((getBits(pSIOCommand->ucIORunBuff[ucSlot], IO_NUM_ADDR) != M35 && getBits(pSIOCommand->ucIORunBuff[ucSlot], IO_NUM_ADDR) != M36)
			|| pVariable->pIOContrlInfo == NULL)
		{
			continue;
		}
		/* 只在输出已生效、尚未完成收尾时释放；其它状态不持有等待输出。 */
		if (pVariable->ucStatus == FEED_CHECK_INTPUTIO || pVariable->ucStatus == FEED_OVERTIME_INTPUTIO
			|| pVariable->ucStatus == FEED_FINISH_CLOSE_IO)
		{
			int8 cOutputIOIndex = pVariable->pIOContrlInfo->cOutputIOIndex;
			/* 索引已经在执行入口映射，取消时不得再次映射或越界写出。 */
			if (cOutputIOIndex >= 0 && cOutputIOIndex < 32 && cOutputIOIndex < sizeof(out) / sizeof(out[0]))
			{
				out[cOutputIOIndex] = !pVariable->pIOContrlInfo->cOutputControlType;
			}
			pVariable->ucStatus = FEED_ERROR_WARNING;
		}
	}
	pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime = 0;
}

/*************************************************
  函数名称：runM35Command
  函数功能：上料IO执行
  输入参数：pSIOCommand 指向IO结构体的指针，pVaribleIOInfo 执行执行IO信息结构体的指针
  输出参数：
  返 回 值：1	执行完成/报警	0	未执行完成
  备    注：2023/7/11 CMC增
**************************************************/
int8 runM35Command(struct _SIOCommand* pSIOCommand, SVaribleIOInfo* pVaribleIOInfo)
{
	int8 cInputIOIndex = pVaribleIOInfo->pIOContrlInfo->cInputIOIndex;
	int8 cOutputIOIndex = pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex;
	switch (pVaribleIOInfo->ucStatus)
	{
		case FEED_INITATION:		//初始化和赋值工作
		{
			if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 38 || pVaribleIOInfo->pIOContrlInfo->cInputIOIndex < 0)		//将输入IO索引映射到企业数组的索引
			{
				pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;	//索引超出报警
				parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM30, 9999);
				return 1;
			}
			if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 10 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 17)
			{
				pVaribleIOInfo->pIOContrlInfo->cInputIOIndex -= 2;
			}
			else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 20 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 27)
			{
				pVaribleIOInfo->pIOContrlInfo->cInputIOIndex -= 4;
			}
			else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 30 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 37)
			{
				pVaribleIOInfo->pIOContrlInfo->cInputIOIndex -= 6;
			}
			else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 0 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 10)
			{

			}
			cInputIOIndex = pVaribleIOInfo->pIOContrlInfo->cInputIOIndex;
			if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 10 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 17)		//将输出IO索引映射到企业数组的索引
			{
				pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex -= 2;
			}
			else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 20 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 27)
			{
				pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex -= 4;
			}
			else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 30 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 37)
			{
				pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex -= 6;
			}
			else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 0 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 10)
			{

			}
			else
			{
				pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;	//索引超出报警
				parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM30, 9999);
				return 1;
			}
			cOutputIOIndex = pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex;
			/* VS 的输出数组只有 30 项；未配置的映射端口不能进入等待或撤销流程。 */
			if (cOutputIOIndex >= sizeof(out) / sizeof(out[0]))
			{
				pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;
				parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM30, 9999);
				return 1;
			}
			if (pVaribleIOInfo->pIOContrlInfo->ucFeeding_RepeatTimes <= 0)	//如果没输入重复次数        P
			{
				pVaribleIOInfo->pIOContrlInfo->ucFeeding_RepeatTimes = 0;	//初始化重复次数为0
			}
			if (pVaribleIOInfo->pIOContrlInfo->nInputDetectTime <= 0)		// 如果没输入IO读取时间为0    J
			{
				pVaribleIOInfo->pIOContrlInfo->nInputDetectTime = 0;	//初始化IO读取时间为0
			}
			if (pVaribleIOInfo->pIOContrlInfo->nFeedingBackTime <= 0)	// 如果没输入上料退回时间为0    R
			{
				pVaribleIOInfo->pIOContrlInfo->nFeedingBackTime = 0;	//初始化上料退回时间为0
			}
			else
			{ 
				pVaribleIOInfo->pIOContrlInfo->nFeedingBackTime *= 1000; //s转化为ms
			}
			pVaribleIOInfo->ucStatus = FEED_OPEN_OUTPUTIO;
		}
		break;	
		case FEED_OVERTIME_INITATION:		//超时，循环处理
		{
			pVaribleIOInfo->pIOContrlInfo->ucFeeding_RepeatTimes -= 1;
			if (pVaribleIOInfo->pIOContrlInfo->ucFeeding_RepeatTimes < 0)	//判断重复次数是否大于0
			{
				pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;
				parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM29, 9999);
				return 1;
			}
			else
			{
				pVaribleIOInfo->ucStatus = FEED_OVERTIME_MOVEBACK;
			}
		}
		break;	
		case FEED_OVERTIME_MOVEBACK:	//超时，等待一段延时后等上料退回后再重新输出
		{
			if (delays_ms(pVaribleIOInfo->pIOContrlInfo->nFeedingBackTime, &pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime))	
			{
				pVaribleIOInfo->ucStatus = FEED_OPEN_OUTPUTIO;
			}
		}
		break;	
		case FEED_OPEN_OUTPUTIO:		//输出控制信号至输出IO
		{
			/* 当前通道每次重新输出都独立计时，清除前条成功或重试的旧起点。 */
			pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime = 0;
			/* 本次输出接管同口旧M20，防止超时或复位清低再次激活M36。 */
			for (uint8 ucRecord = 0; ucRecord < IO_MAXGROUP_IN_SINGLE_LINE; ucRecord++)
			{
				pSIOCommand->SVaribleIOInfoArray[ucRecord].unOutputIOIndexOfM20 &= ~((uint32)1 << cOutputIOIndex);
			}
			out[cOutputIOIndex] = pVaribleIOInfo->pIOContrlInfo->cOutputControlType;
			pVaribleIOInfo->ucStatus = FEED_CHECK_INTPUTIO;
#if (FEED_IO_TEST == 1)
			attribute[cInputIOIndex][2] = pVaribleIOInfo->pIOContrlInfo->cOutputControlType;
#endif
		}	
		case FEED_CHECK_INTPUTIO:		//检测输入口是否为有效信号
		{
			if (delays_ms(pVaribleIOInfo->pIOContrlInfo->nInputDetectTime, &pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime) && pVaribleIOInfo->pIOContrlInfo->nInputDetectTime != 0)	//检测输入是否为有效信号的最长时间，注：检测时间为0会一直检测
			{
				pVaribleIOInfo->ucStatus = FEED_OVERTIME_INTPUTIO;
			}
			if (attribute[cInputIOIndex][2] == pVaribleIOInfo->pIOContrlInfo->cInputValidType) //在设定时间J里检测到有外部输入信号 2023/9/13 洪冠鹏
			{
#if	(CONFIG_FINISH_CLOSEIO == 1)
				pVaribleIOInfo->ucStatus = FEED_FINISH_CLOSE_IO;
#else
				pVaribleIOInfo->ucStatus = FEED_FINISH_RELEASE; //M35执行结束
#endif
				/* 输入成功后释放共享计时字段，后续M01/M22等不得继承起点。 */
				pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime = 0;
				pVaribleIOInfo->pIOContrlInfo->nInputDetectTime = 0;  
			}
		}
		break;
		case FEED_OVERTIME_INTPUTIO:		//检测超时，延时R后关闭输出IO
		{
			out[cOutputIOIndex] = !pVaribleIOInfo->pIOContrlInfo->cOutputControlType;
			pVaribleIOInfo->ucStatus = FEED_OVERTIME_INITATION;
		}
		break;
		case FEED_FINISH_CLOSE_IO:		//可选配置，如果运行完成会关闭输出IO
		{
			out[cOutputIOIndex] = !pVaribleIOInfo->pIOContrlInfo->cOutputControlType;
			pVaribleIOInfo->ucStatus = FEED_FINISH_RELEASE;
		}	
		case FEED_FINISH_RELEASE:		//完成
		{
			return 1;
			break;
		}		
		case FEED_ERROR_WARNING:	//报警	
		default:
			return 1;
			break;
	}
	return 0;
}

#define VARIABLE_IO_INITATION	0		//初始化
#define VARIABLE_IO_RUN			1		//执行IO
#define VARIABLE_IO_WAIT		2		//等待IO执行完
#define VARIABLE_IO_CLOSE		3		//关闭IO
#define VARIABLE_IO_FINISH		4		//完成
/*************************************************
  函数名称：runM20M21M22Command
  函数功能：M20/M21/M22指定的IO执行
  输入参数：pSIOCommand 指向IO结构体的指针，pVaribleIOInfo 执行执行IO信息结构体的指针
  输出参数：
  返 回 值：1	执行完成/报警	0	未执行完成
  备    注：
**************************************************/
int8 runM20M21M22Command(struct _SIOCommand* pSIOCommand, SVaribleIOInfo* pVaribleIOInfo)
{
	int32 cOutputIOIndex = pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex;
	switch (pVaribleIOInfo->ucStatus)
	{
	case VARIABLE_IO_INITATION:
		if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 10 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 17)		//将输出IO索引映射到企业数组的索引
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex -= 2;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 20 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 27)
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex -= 4;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 30 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 33)
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex -= 6;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 0 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 10)
		{

		}
		else
		{
			pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;	//索引超出报警
			parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM30, 9999);
		    // **硬门槛：立即终止本命令，禁止后续 setbit/out 写IO**
		    return 1;  // 你这个函数定义：1=完成/报警；用 1 可让上层停止推进
		}
		cOutputIOIndex = pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex;
		if (pVaribleIOInfo->pIOContrlInfo->cOutputControlType == 1) //识别当前指令位M20  洪冠鹏 2023/9/22
		{
			setbit(pVaribleIOInfo->unOutputIOIndexOfM20, cOutputIOIndex); //获取M20输出端口，置1
		}
		pVaribleIOInfo->ucStatus = VARIABLE_IO_RUN;

		if (pVaribleIOInfo->pIOContrlInfo->nOutputRunTims <= 0 && pVaribleIOInfo->pIOContrlInfo->cOutputControlType == PULSE_TYPE)	//如果外部输出持续时间为0，不输出IO
		{
			pVaribleIOInfo->ucStatus = VARIABLE_IO_FINISH;
		}
		break;
	case VARIABLE_IO_RUN:
		out[cOutputIOIndex] = (pVaribleIOInfo->pIOContrlInfo->cOutputControlType == PULSE_TYPE) ? HIGHLEVEL: pVaribleIOInfo->pIOContrlInfo->cOutputControlType;	//如果为脉冲型IO输出高电平，否则输出对应的电平类型
		pVaribleIOInfo->ucStatus = (pVaribleIOInfo->pIOContrlInfo->cOutputControlType == PULSE_TYPE) ? VARIABLE_IO_WAIT: VARIABLE_IO_FINISH;
		if(pVaribleIOInfo->ucStatus == VARIABLE_IO_FINISH)
		{
		if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 8 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 15) // 将企业数组的索引换回IO索引，以便统一初始化  2023/9/25 洪冠鹏 
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex += 2;
		}
			else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 16 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 23)
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex += 4;
		}
			else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 24 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 27)
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex += 6;
		}
		}

		break;

	case VARIABLE_IO_WAIT:	//脉冲型IO输出，维持一段高电平时间
		if (delays_ms(pVaribleIOInfo->pIOContrlInfo->nOutputRunTims, &pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime))
		{
			pVaribleIOInfo->ucStatus = VARIABLE_IO_CLOSE;
		}
		break;

	case VARIABLE_IO_CLOSE:	//脉冲型IO输出，关闭IO
		out[cOutputIOIndex] = LOWLEVEL;
		pVaribleIOInfo->ucStatus = VARIABLE_IO_FINISH;
		if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 8 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 15) // 将企业数组的索引换回IO索引，以便统一初始化  2023/9/25 洪冠鹏
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex += 2;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 16 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 23)
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex += 4;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex >= 24 && pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex <= 27)
		{
			pVaribleIOInfo->pIOContrlInfo->cOutputIOIndex += 6;
		}
		break;

	case VARIABLE_IO_FINISH:
		return 1;
	default:
		return 1;
		break;
	}
	return 0;
}


/*************************************************
  函数名称：detectM01Command
  函数功能：M01的指定IO检测
  输入参数：pSIOCommand 指向IO结构体的指针，pVaribleIOInfo 执行执行IO信息结构体的指针
  输出参数：
  返 回 值：1	执行完成/报警	0	未执行完成
  备    注：
**************************************************/
int8 detectM01Command(struct _SIOCommand* pSIOCommand, SVaribleIOInfo* pVaribleIOInfo)
{
	int8 cInputIOIndex = pVaribleIOInfo->pIOContrlInfo->cInputIOIndex;
	int32 nInputDetectTime = pVaribleIOInfo->pIOContrlInfo->nInputDetectTime;
	switch (pVaribleIOInfo->ucStatus)
	{
	case VARIABLE_IO_INITATION:	//初始化
		if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex % 10 == 8 || pVaribleIOInfo->pIOContrlInfo->cInputIOIndex % 10 == 9)		//输入的IO端口不存在
		{
			pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;
			parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM30, 9999);
		}
		if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 38)		//将输入IO索引映射到企业数组的索引
		{
			pVaribleIOInfo->ucStatus = FEED_ERROR_WARNING;	//索引超出报警
			parseRunningError(pSIOCommand->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM30, 9999);
		}
		if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 10 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 17)
		{
			pVaribleIOInfo->pIOContrlInfo->cInputIOIndex -= 2;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 20 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 27)
		{
			pVaribleIOInfo->pIOContrlInfo->cInputIOIndex -= 4;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 30 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 37)
		{
			pVaribleIOInfo->pIOContrlInfo->cInputIOIndex -= 6;
		}
		else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 0 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <=7)
		{
		}

		if (pVaribleIOInfo->pIOContrlInfo->nInputDetectTime <= 0)	//如果外部输出持续时间为0，不输出IO
		{
			//pVaribleIOInfo->ucStatus = VARIABLE_IO_ERROR;
			pVaribleIOInfo->pIOContrlInfo->nInputDetectTime = 100000;	//企业要求，没输入时无限延时，暂定100000ms
		}
		nInputDetectTime = pVaribleIOInfo->pIOContrlInfo->nInputDetectTime;
		cInputIOIndex = pVaribleIOInfo->pIOContrlInfo->cInputIOIndex;
		pVaribleIOInfo->ucStatus = VARIABLE_IO_DETECT;
		break;
	case VARIABLE_IO_DETECT:	//检测IO
		if (pSIOCommand->pSChannel->bProgramStop == TRUE)  //程序暂停按键在等待期间被按下 等待程序启动 2023/10/9 洪冠鹏
		{
			pVaribleIOInfo->ucStatus = VARIABLE_IO_WAIT;
			break;
		}
		if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <0) //无外部输出端口（仅有M01）   2023/9/25 洪冠鹏 新增选择停止功能
		{
			if (pSIOCommand->pSChannel->bSelectableStop == TRUE) //选择停止开关—开，进入等待
			{
				pVaribleIOInfo->ucStatus = VARIABLE_IO_WAIT;
				return 1;
			}
			else //选择停止开关—关，跳过M01
			{
				pVaribleIOInfo->ucStatus = VARIABLE_IO_FINISH;	
				pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime = 0;
			}
		}
		else
		{
			if (attribute[cInputIOIndex][2] == pVaribleIOInfo->pIOContrlInfo->cInputValidType)
			{
				pVaribleIOInfo->ucStatus = VARIABLE_IO_FINISH;	//检测到IO为匹配的类型，结束
				pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime = 0;
			}
			else
			{
				if (delays_ms(nInputDetectTime, &pSIOCommand->pSChannel->SApplyTime.unFeedingRunTime))
				{
					if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 8 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 15)
					{
						pVaribleIOInfo->pIOContrlInfo->cInputIOIndex += 2;
					}
					else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 16 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 23)
					{
						pVaribleIOInfo->pIOContrlInfo->cInputIOIndex += 4;
					}
					else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 24 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 31)
					{
						pVaribleIOInfo->pIOContrlInfo->cInputIOIndex += 6;
					}
					else if (pVaribleIOInfo->pIOContrlInfo->cInputIOIndex >= 0 && pVaribleIOInfo->pIOContrlInfo->cInputIOIndex <= 7)
					{
					}
					pVaribleIOInfo->ucStatus = VARIABLE_IO_ERROR;	//超过检测时间报警
				}
			}
		}
		break;
	case VARIABLE_IO_WAIT:
		if (pSIOCommand->pSChannel->bProgramStop == FALSE) //重新启动后重新执行M01
		{
			pVaribleIOInfo->ucStatus = VARIABLE_IO_DETECT;
		}
		break;
	case VARIABLE_IO_FINISH:
	case VARIABLE_IO_ERROR:
		pSIOCommand->pSChannel->bProgramStop = FALSE;   //初始化程序暂停标志位 
		return 1;
	default:
		return 1;
		break;
	}
	return 0;
}



/*************************************************
  函数名称：clearApplyRunTime
  函数功能：运行作用时间清零
  输入参数：pSChannel 通道结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void clearApplyRunTime(struct SChannel* pSChannel)
{



	if(pSChannel == NULL)
	{
	    return;
	}
	pSChannel->SApplyTime.unFeedingRunTime = 0;
	pSChannel->SApplyTime.unSampleIntervalRunTime = 0;
	pSChannel->SApplyTime.unSpindlePreRunTime = 0;
	pSChannel->SApplyTime.unSpindlePostRunTime = 0;
	pSChannel->SApplyTime.unChuckAOpenRunTime = 0;
	pSChannel->SApplyTime.unSpindleLockRunTime = 0;
	pSChannel->SApplyTime.unSpindleUnLocktRunTime = 0;
	//memset(&pSChannel->SApplyTime, 0, sizeof(struct SRunTime));
}

