#include "TIOCommand.h"
#include "Motion.h"


#if STM32_MODE	//32测试环境

//输出注意事项：out[]数组是企业为了接线换线做的映射数组，每周期做一次IO输出执行，       //2025/6/23 企业要求：取消使用out数组（中间过渡作用），直接读取底层IO输出信号
//			有一个弊端:IO不能实时输出，需要等运行到企业的IO输出执行函数才会输出IO电平
//			
//			因为out[]的存在，outXX和PXout(Y)只能实现一瞬间输出，运行到企业的IO输出执行函数时IO的输出会清空。
#include "io_exc.h"
#define TOOLOUTPUT_IO1				outy20//PBout(6)
#define TOOLOUTPUT_IO2				outy21//PBout(7)
#define TOOLOUTPUT_IO3				outy22//PBout(12)
#define TOOLOUTPUT_IO4				outy23//PBout(13)
#define TOOLCHANGE_TRIGSIGNAL0		outy24//PBout(14)
#define TOOLCHANGE_TRIGSIGNAL1		outy25//PBout(15)
#define MODECHANGE_IO				outy26//PDout(2)
#define VALVE_LOCK_IO				outy27//out27//PGout(15)
#define TOOLOUTPUT_ENABLE_IO		outy30//out30//PAout(11)

//输入注意事项：
// 1.企业的IO输入引脚默认为高电平
//	2.可以直接读其他提供的接口	intxXX来读输入IO的电平
//	3.企业的接口<intx == 1>代表有信号输入
//	4.IO输入需要做防抖处理，如果是读企业的接口intxXX的话，其已经经过了防抖处理

#define TOOLINPUT_IO1				intx20//PEin(15)
#define TOOLINPUT_IO2				intx21//PFin(13)
#define TOOLINPUT_IO3				intx22//PFin(14)
#define TOOLINPUT_IO4				intx23//PFin(15)
#define TOOLINPUT_IO5				intx24//PGin(0)

#define	VALVE_LOCKED_IO				intx25//PGin(14)
#define VALVE_UNLOCKED_IO			intx26//PCin(14)
#define TOOLCHANGE_INPLACE_IO		intx27//PCin(4)


#else	//VS测试环境

int8 TOOLINPUT_IO1 = 0;
int8 TOOLINPUT_IO2 = 0;
int8 TOOLINPUT_IO3 = 0;
int8 TOOLINPUT_IO4 = 0;
int8 TOOLINPUT_IO5 = 0;

int8 TOOLOUTPUT_IO1 = 0;
int8 TOOLOUTPUT_IO2 = 0;
int8 TOOLOUTPUT_IO3 = 0;
int8 TOOLOUTPUT_IO4 = 0;

int8 TOOLCHANGE_INPLACE_IO = 0;

int8 TOOLCHANGE_TRIGSIGNAL0 = 0;
int8 TOOLCHANGE_TRIGSIGNAL1 = 0;

int8 VALVE_LOCK_IO = 0;
int8 VALVE_LOCKED_IO = 0;
int8 VALVE_UNLOCKED_IO = 0;

int8 TOOLOUTPUT_ENABLE_IO = 0;
int8 TOOLINPUT_ENABLE_IO = 0;

#endif

#define TOOLCHANGE_TEST 0

#if (TOOLCHANGE_TEST == 1)
int8 ucImageTest_RunStep = 0;
#endif

/*************************************************
  函数名称：TIOSTRUCT_InitEditablePara()
  函数功能：可编辑系统参数初始化  初始化按钮调用此函数
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 TIOSTRUCT_InitEditablePara()
{
	g_SSystemPara.ucToolDevie = TOOL_RAWTOOL_MODE;	//刀具设备，排刀机/刀塔机
	g_SSystemPara.ucToolChange_TrigDelayTime = 50;	//换刀触发信号维持时间
	g_SSystemPara.usToolChange_LockDelayTime = 50;	//换刀夹紧信号维持时间
	g_SSystemPara.usToolChange_ReadOrSendIODelayTime = 50;	//换刀夹紧信号维持时间
	g_SSystemPara.usToolChange_AlarmTime = 1000;	//换刀时间过长报警时间

	g_SSystemPara.ucToolNum = 1;	//刀具号
	g_SSystemPara.ucToolCompNum = 1;	//刀补号
}
#if STM32_MODE
/*************************************************
  函数名称：TIOSTRUCT_InitAfterLoad
  函数功能：从Flash中加载数据后执行的初始化
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 ReadToolNum_AfterLoadSysPara(struct _STIOCommand* pSTIOCommand, struct _SRun* pSRun, struct SInterpolationPre* pSPreProcess)
{
	uint8 ucToolNum;
	uint8 ucToolCompNum;
	int8 ctempToolNum = 0;
	ucToolNum = g_SSystemPara.ucToolNum;
	ucToolCompNum = g_SSystemPara.ucToolCompNum;
	if (g_SSystemPara.ucToolDevie == TOOL_TURRET_MODE)
	{
		struct _SIOReadGroup sTool_InPlaceIO;
		sTool_InPlaceIO.ucNum = 1;	//读单独一个IO
		sTool_InPlaceIO.ucIOAlias[0] = 27;	//别名 X2.7
		sTool_InPlaceIO.unGPIOxArrays[0] = GPIOC;	//GPIOC
		sTool_InPlaceIO.ucPinArrays[0] = GPIO_PIN_4;	//PC4
		while (MultiSample(pSTIOCommand->pSChannel, &pSTIOCommand->ucReadIO_Counter, 1, &sTool_InPlaceIO) != 1);

		if (getToolChange_InPlaceStatus())
		{
			struct _SIOReadGroup sTool_ToolNumIO =
			{
				{5},
				{GPIOE,GPIOF,GPIOF,GPIOF,GPIOG},
				{GPIO_PIN_15,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15,GPIO_PIN_0},
				{0,0,0,0,0},
				{20,21,22,23,24}
			};
			while (MultiSample(pSTIOCommand->pSChannel, &pSTIOCommand->ucReadIO_Counter, 1, &sTool_ToolNumIO) != 1);
			ctempToolNum = getToolNumFromServo();
			if(ctempToolNum > TOOl_LOCATION_MAX_NUM-4)
			{
				parseErrorC(9999, PLC_ERROR_NUM3955);
				pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
				return -1;
			}
			if (ctempToolNum < 0)
			{
				parseErrorC(9999, PLC_ERROR_NUM3978);
			}
			else if (ctempToolNum != ucToolNum)
			{
				ucToolNum =ctempToolNum;
				parseErrorC(9999, PLC_ERROR_NUM3842);
				ucToolCompNum = 1;
			}

		}
		else
		{
			parseErrorC(9999, PLC_ERROR_NUM3986);
		}
	}
    if(ucToolNum > TOOl_LOCATION_MAX_NUM-4)
    {
    	parseErrorC(9999, PLC_ERROR_NUM3842);
    	return -1;
    }
	g_SSystemPara.ucToolNum = pSPreProcess->ucToolNum = pSRun->ucToolNum = ucToolNum;
	g_SSystemPara.ucToolCompNum = pSPreProcess->ucToolCompNum = pSRun->ucToolCompNum = ucToolCompNum;
	//计算新的偏移坐标
	TChangeTotalOffset(pSPreProcess->pSChannel->ucChannelNum, pSRun->ucToolCompNum, pSRun->cWorkpieceCoordNum, &pSRun->SCurrentOffset, &pSRun->SWorkPieceOffsetByG50);
	pSTIOCommand->ucToolNum = ucToolNum;
	pSTIOCommand->ucToolCompNum = ucToolCompNum;
	return 1;
}
#endif


/*************************************************
  函数名称：IOStructToBuff(UMcode ucIOCommand)
  函数功能  将T代码的刀具号放入刀具IO运行结构体中
  输入参数：pSTIOCommand		刀具运行结构体
  输出参数：
  返 回 值：1 放入成功
			0 放入失败
  备    注：
**************************************************/
uint8 TIOToRunBuff(uint8 ucTIOCommand, uint8 ucToolCompNum, struct _STIOCommand* pSTIOCommand)
{
	if (pSTIOCommand->ucTIORun == 0)  //读出的数据为0  表示为空
	{
		if ((ucTIOCommand > TOOl_LOCATION_MAX_NUM - 4))	//输入的刀具号错误
		{
			return -1;
		}
		else
		{
			pSTIOCommand->ucTIORun = ucTIOCommand;//将目标M代码放入其中
			pSTIOCommand->ucToolCompNumRun = ucToolCompNum;
		}
		return 1;//放入成功
	}
	return 0;//放入失败
}

/*************************************************
  函数名称：runTIO
  函数功能：执行输入的T代码
  输入参数：pSTIOCommand		刀具运行结构体
  输出参数：
  返 回 值：-3	输入的T代码代码不正确
			-2	无需要执行的换刀指令
			-1	输入的刀具运行结构体为空
			0	排刀机模式不运行
			1	正在执行换刀
  备    注：
**************************************************/
int8 runTIO(struct _STIOCommand* pSTIOCommand)
{
	int ToolNumFromServo = 0;
	int ret = 0;
	if (pSTIOCommand == NULL || pSTIOCommand->pSChannel == NULL)
	{
		return -1;
	}
	if(pSTIOCommand->pSChannel->ucChannelStatus == SYS_EMERGENCY_STOP)return 0;//急停状态不执行
	if (g_SSystemPara.ucToolDevie == TOOL_RAWTOOL_MODE)
	{
		if (pSTIOCommand->ucToolChangeStatus == TOOLCHANGE_ERROR)	//刀塔状态机处于报错状态下切换至排刀机模式后初始化刀塔状态机	6/15 CMC
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_IDLE;
		}
		return 0;
	}

	if (pSTIOCommand->ucTIORun == 0)//0表示空 不执行  返回
	{
		return -2;
	}

	if (pSTIOCommand->ucToolChangeStatus != TOOLCHANGE_ERROR)
	{
		//换刀超时报警
		if (delays_ms(g_SSystemPara.usToolChange_AlarmTime, &pSTIOCommand->pSChannel->SApplyTime.unToolChange_AlarmTime) == 1)
		{
			parseErrorC(9999, PLC_ERROR_NUM1000);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
		}
	}

	switch (pSTIOCommand->ucToolChangeStatus)
	{
	case TOOLCHANGE_IDLE:
	{
		if (pSTIOCommand->ucTIORun != 0)
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_SETSERVO_ENABLED;
		}
		else
		{
			break;
		}
	}

	case TOOLCHANGE_SETSERVO_ENABLED:
	{
		if (pSTIOCommand->ucToolNum == pSTIOCommand->ucTIORun)	//换刀的目标刀具号是否和当前的刀具号相同
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_DONE;
		}
		else
		{
			//判断刀架（电磁阀）是否锁紧
			if (getToolCarrier_LockedStatus() != LOCK)
			{
				parseErrorC(9999, PLC_ERROR_NUM3987);
				pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
				break;
			}

			//检测定位完成信号是否正常
			if (!getToolChange_InPlaceStatus())
			{
				parseErrorC(9999, PLC_ERROR_NUM3986);
				pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
				break;
			}

			//刀架松开/使能
			setToolCarrier_Locked(UNLOCK);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_OUTPUT_TOOLNUM;
		}
	}
	case TOOLCHANGE_OUTPUT_TOOLNUM:
	{
		int8 cRet = getToolCarrier_LockedStatus();
		if (cRet == UNLOCK)
		{
			set_ToolServo_EnabledStatus(IO_SET);
			SelectToolNum(pSTIOCommand->ucTIORun);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_WAITFOR_SENDTRIG;
		}
		else if (cRet == -1)
		{
			parseErrorC(9999, PLC_ERROR_NUM3988);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
		}
		else
		{
			break;
		}
	}
	case TOOLCHANGE_WAITFOR_SENDTRIG:
	{
		//说明书上显示，选定刀号后需延时一定的时间，>2ms
		if (WaitFor_SendTrigSignal(pSTIOCommand) == 1)
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_SEND_TRIGSIGN;
		}
		else
		{
			break;
		}
	}
	case TOOLCHANGE_SEND_TRIGSIGN:
	{
		//发送持续一段时间的高电平触发信号
		if (Trig_ToolChange(pSTIOCommand) == 1)
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_WAIT_GET_INPLACESIGN;
		}
		else
		{
			break;
		}
	}

	case TOOLCHANGE_WAIT_GET_INPLACESIGN:
	{
		ret = getToolChangeStatus(pSTIOCommand);	//获取换刀伺服刀位到位状态
		if (ret == TRUE)
		{
			//经实机测试，到位信号接收到后，需要延时一定的时间才能检测到正确刀号,延时时间暂时用触发时间代替
			if (delays_ms(g_SSystemPara.usToolChange_ReadOrSendIODelayTime, &pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime) == 1)
			{
				pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_WAIT_GET_TOOLNUM;
			}
			else
			{
				break;
			}
		}
		else
		{
			pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime = 0;
			break;
		}
	}
	case TOOLCHANGE_WAIT_GET_TOOLNUM:
	{
		ToolNumFromServo = getToolNumFromServo();			//读取刀号
		if(ToolNumFromServo > TOOl_LOCATION_MAX_NUM-4)
		{
			parseErrorC(9999, PLC_ERROR_NUM3955);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
			return -1;
		}
		else if(ToolNumFromServo <0)
		{
			if(ToolNumFromServo == -2)
			parseErrorC(9999, PLC_ERROR_NUM1001);
			else
			parseErrorC(9999, PLC_ERROR_NUM3988);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
			return -1;
		}
		if (ToolNumFromServo == pSTIOCommand->ucTIORun)		//读取的刀号与目标刀号相同
		{
			set_ToolServo_EnabledStatus(IO_RESET);			//失能伺服

			if (delays_ms(g_SSystemPara.usToolChange_ReadOrSendIODelayTime, &pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime) == 1)	//失能或使能伺服需要一段时间延时才能被伺服驱动识别到
			{

			}
			else
			{
				break;
			}

			setToolCarrier_Locked(LOCK);					//锁紧刀架

			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_WAIT_GETLOCK;
		}
		else if (ToolNumFromServo == 0)
		{
			//换刀伺服报错
			parseErrorC(9999, PLC_ERROR_NUM3978);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
			break;
		}
		else
		{
			//读取的刀号和目标不同，报错
			parseErrorC(9999, PLC_ERROR_NUM3955);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
			break;
		}
	}
	case TOOLCHANGE_WAIT_GETLOCK:
	{
		//应企业要求，增加刀架锁紧判断时间
		if (delays_ms(g_SSystemPara.usToolChange_LockDelayTime, &pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime) == 1)
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_WAIT_LOCKED;
		}
		else
		{
			break;
		}

	}
	case TOOLCHANGE_WAIT_LOCKED:
	{
		int8 cRet = getToolCarrier_LockedStatus();
		//如果刀架已锁，则可以判断换刀已完成
		if (cRet == LOCK)
		{
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_DONE;
		}
		else if (cRet == -1)
		{
			parseErrorC(9999, PLC_ERROR_NUM3988);
			pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_ERROR;
		}
		else
		{
			break;
		}
	}

	case TOOLCHANGE_DONE:
	{
		//换刀完成后清空变量值
		pSTIOCommand->pSChannel->SApplyTime.unToolChange_AlarmTime = 0;
		pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime = 0;
		g_SSystemPara.ucToolNum = pSTIOCommand->ucToolNum = pSTIOCommand->ucTIORun;
		g_SSystemPara.ucToolCompNum = pSTIOCommand->ucToolCompNum = pSTIOCommand->ucToolCompNumRun;
		pSTIOCommand->ucTIORun = 0;
		pSTIOCommand->ucToolCompNumRun = 0;
		pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_IDLE;
	}
	break;

	case TOOLCHANGE_ERROR:
	{
		pSTIOCommand->pSChannel->SApplyTime.unToolChange_AlarmTime = 0;
		pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime = 0;
		//还没想好怎么处理
		if (pSTIOCommand->pSChannel->SPreProcess.ucInpStatus == INP_FINISHED)
		{
			//主轴减速停
			spindleCommonStop(&pSTIOCommand->pSChannel->SSpindle);

			//动力刀减速停
			spindleCommonStop(&pSTIOCommand->pSChannel->SSecondSpindle);
			channelStopToIDLE(pSTIOCommand->pSChannel);
		}
		//pSTIOCommand->ucTIORun = 0;
		//pSTIOCommand->ucToolCompNumRun = 0;
		//pSTIOCommand->ucToolChangeStatus = TOOLCHANGE_IDLE;
	}
	break;
	default:
		break;
	}
	return 1;
}

/*************************************************
  函数名称：getToolNumFromServo
  函数功能：执行输入的T代码
  输入参数：pSTIOCommand		刀具运行结构体
  输出参数：
  返 回 值：-3	定位未完成
			-2	读取的信号无效
			0	伺服故障
			1~12	刀号


  备    注：
**************************************************/
int8 getToolNumFromServo()
{
	int8 cToolInput_8BIT = 0;
	cToolInput_8BIT = (TOOLINPUT_IO1) | (TOOLINPUT_IO2 << 1) | (TOOLINPUT_IO3 << 2) | (TOOLINPUT_IO4 << 3) | (TOOLINPUT_IO5 << 4);

#if(TOOLCHANGE_TEST == 1)
	cToolInput_8BIT = 7;	//3号刀
#endif // TOOLCHANGE_TEST

	if (cToolInput_8BIT > 4)
	{
		if(cToolInput_8BIT > TOOl_LOCATION_MAX_NUM-4)
		{
			return -2;
		}
		else
		return cToolInput_8BIT - 4;	//返回刀号
	}
	else
	{
		if (cToolInput_8BIT != 0)
		{
			if (cToolInput_8BIT == 4)
			{
				return -3;	//定位未完成
			}
			else
			{
				return -2;	//无效
			}
		}
		else
		{
			return 0;	//伺服故障
		}

	}
	return 0;	//伺服故障
}

/*************************************************
  函数名称：SelectToolNum
  函数功能：根据输入的刀具号输出刀具选择的IO
  输入参数：cToolNum		刀具号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 SelectToolNum(int8 cToolNum)
{
	//10进制转换成二进制
	TOOLOUTPUT_IO1 = (cToolNum % 2);
	TOOLOUTPUT_IO2 = ((cToolNum / 2) % 2);
	TOOLOUTPUT_IO3 = ((cToolNum / 4) % 2);
	TOOLOUTPUT_IO4 = ((cToolNum / 8) % 2);
#if STM32_MODE
	//升级H7注释
	//PEout(15) = TOOLOUTPUT_IO1;
	//PFout(13) = TOOLOUTPUT_IO2;
	//PFout(14) = TOOLOUTPUT_IO3;
	//PFout(15) = TOOLOUTPUT_IO4;
#endif
	return 1;
}

/*************************************************
  函数名称：ClearSelectedToolNumIO
  函数功能：清除刀具选择IO
  输入参数：
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void ClearSelectedToolNumIO()
{
	//10进制转换成二进制
	TOOLOUTPUT_IO1 = IO_RESET;
	TOOLOUTPUT_IO2 = IO_RESET;
	TOOLOUTPUT_IO3 = IO_RESET;
	TOOLOUTPUT_IO4 = IO_RESET;

#if STM32_MODE
	PEout(15) = IO_RESET;
	PFout(13) = IO_RESET;
	PFout(14) = IO_RESET;
	PFout(15) = IO_RESET;
#endif

}

/*************************************************
  函数名称：WaitFor_SendTrigSignal
  函数功能：根据刀塔伺服说明书，发送选刀信号后，需要一定时间的延时才能继续发送触发信号。
  输入参数：pSTIOCommand		刀具运行结构体
  输出参数：
  返 回 值：1	触发信号发送完成
			0	触发信号发送中
  备    注：
**************************************************/
int8 WaitFor_SendTrigSignal(struct _STIOCommand* pSTIOCommand)
{
	if (delays_ms(g_SSystemPara.ucToolChange_TrigDelayTime, &pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime) == 1)//延时是否完成
	{
		return 1;
	}
	return 0;

}

/*************************************************
  函数名称：Trig_ToolChange
  函数功能：换刀发送触发信号，换刀前需要拉高触发信号，并维持一段时间再拉低。
  输入参数：pSTIOCommand		刀具运行结构体
  输出参数：
  返 回 值：1	触发信号发送完成
			0	触发信号发送中
  备    注：
**************************************************/
int8 Trig_ToolChange(struct _STIOCommand* pSTIOCommand)
{
	TOOLCHANGE_TRIGSIGNAL1 = IO_SET;
	if (delays_ms(g_SSystemPara.ucToolChange_TrigDelayTime, &pSTIOCommand->pSChannel->SApplyTime.unToolChange_TrigRunTime) == 1)//延时是否完成
	{
		TOOLCHANGE_TRIGSIGNAL1 = IO_RESET;
		return 1;
	}
	return 0;

}


/*************************************************
  函数名称：getToolChangeStatus
  函数功能：查看换刀是否到位
  输入参数：pSTIOCommand		刀具运行结构体
  输出参数：
  返 回 值：1	换刀完成
			0	正在换刀
  备    注：
**************************************************/
int8 getToolChangeStatus(struct _STIOCommand* pSTIOCommand)
{
	int8 cToolChangDone_Flag = 0;
	cToolChangDone_Flag = TOOLCHANGE_INPLACE_IO;
#if(TOOLCHANGE_TEST == 1)
	cToolChangDone_Flag = 1;
#endif // TOOLCHANGE_TEST

	if (cToolChangDone_Flag == IO_SET)
	{
		return 1;
	}

	return 0;
}

/*************************************************
  函数名称：getToolCarrier_LockedStatus
  函数功能：获取刀架(电磁阀）锁紧状态
  输入参数：
  输出参数：
  返 回 值：1 锁紧
			0 松开
			-1 未知状态
  备    注：
**************************************************/
int8 getToolCarrier_LockedStatus()
{
#if(TOOLCHANGE_TEST == 1)
	if (ucImageTest_RunStep == 0)
	{
		VALVE_LOCKED_IO = IO_SET;
		VALVE_UNLOCKED_IO = IO_RESET;
		ucImageTest_RunStep++;
	}
	else
	{
		VALVE_LOCKED_IO = IO_RESET;
		VALVE_UNLOCKED_IO = IO_SET;
		ucImageTest_RunStep = 0;

	}
#endif // TOOLCHANGE_TEST
	if (VALVE_LOCKED_IO == IO_SET && VALVE_UNLOCKED_IO == IO_RESET)
	{
		return LOCK;
	}
	else if (VALVE_LOCKED_IO == IO_RESET && VALVE_UNLOCKED_IO == IO_SET)
	{
		return UNLOCK;
	}
	else
	{
		return -1;
	}
}

/*************************************************
  函数名称：setToolCarrier_Locked
  函数功能：使刀架(电磁阀）锁紧或松开
  输入参数：cCommand 锁紧/松开的指令
  输出参数：
  返 回 值：1 已输出	-1 输入指令错误
  备    注：
**************************************************/
int8 setToolCarrier_Locked(int8 cCommand)
{
	if (cCommand == LOCK)
	{
		VALVE_LOCK_IO = LOCK_ELECTRICAL_LEVEL;
#if STM32_MODE
		outy27 = LOCK_ELECTRICAL_LEVEL;
#endif
		return 1;
	}
	else if (cCommand == UNLOCK)
	{
		VALVE_LOCK_IO = UNLOCK_ELECTRICAL_LEVEL;
#if STM32_MODE
		outy27 = UNLOCK_ELECTRICAL_LEVEL;
#endif
		return 1;
	}
	else
	{
		return -1;
	}
}

/*************************************************
  函数名称：getToolChange_InPlaceStatus
  函数功能：获取换刀定位信号
  输入参数：
  输出参数：
  返 回 值：1	换刀定位完成
			0	换刀定位未完成
  备    注：
**************************************************/
int8 getToolChange_InPlaceStatus()
{
#if(TOOLCHANGE_TEST == 1)
	TOOLCHANGE_INPLACE_IO = IO_SET;
#endif // TOOLCHANGE_TEST
	if (TOOLCHANGE_INPLACE_IO == IO_SET)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/*************************************************
  函数名称：set_ToolServo_EnabledStatus
  函数功能：刀具伺服使能/失能
  输入参数：IO_SET/ SET/1	使能
			IO_RESET/ RESET/0	失能
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void set_ToolServo_EnabledStatus(int8 cCommand)
{
	if (cCommand == IO_SET)
	{
		TOOLOUTPUT_ENABLE_IO = IO_SET;
#if STM32_MODE
		outy30 = IO_SET;
#endif
	}
	else
	{
		TOOLOUTPUT_ENABLE_IO = IO_RESET;
#if STM32_MODE
		outy30 = IO_RESET;
#endif
	}

}

/*************************************************
  函数名称：MultiSample
  函数功能：模仿企业的消抖处理
			因为企业的IO是经过消抖处理，但其检查管脚IO状态的函数是在循环while(1)中调用，
			而我们需要在还没进入循环前、开机读完系统参数后对刀塔状态检查，也就是对IO进行多次采样消抖读取。
  输入参数：IO_SET/ SET/1	使能
			IO_RESET/ RESET/0	失能
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 MultiSample(struct SChannel* pSChannel, uint8* ptimes, uint8 ucDuration, struct _SIOReadGroup* pIOReadGroup)
{
	if (ptimes == NULL || pIOReadGroup == NULL)
	{
		return -1;
	}
	if(pIOReadGroup->ucNum == 0||pIOReadGroup->ucNum >READIO_NUM_MAX)
	return -1;
	uint8 ucCounter;
	if (delays_ms(ucDuration, &pSChannel->SApplyTime.unSampleIntervalRunTime) == 1)
	{
#if STM32_MODE	//32测试环境
		for (ucCounter = 0; ucCounter < pIOReadGroup->ucNum; ucCounter++)
		{
			if ((uint32)(pIOReadGroup->unGPIOxArrays[ucCounter]) < (uint32)GPIOA || (pIOReadGroup->unGPIOxArrays[ucCounter]) > (uint32)GPIOI)
			return -1;
			if (HAL_GPIO_ReadPin((GPIO_TypeDef*)pIOReadGroup->unGPIOxArrays[ucCounter], pIOReadGroup->ucPinArrays[ucCounter])==IO_RESET)
			//if (BIT_ADDR(pIOReadGroup->unGPIOxArrays[ucCounter], pIOReadGroup->ucPinArrays[ucCounter]) == IO_RESET)
			{
				pIOReadGroup->ucIOLowLevelTimesArray[ucCounter]++;
			}
		}
#endif
		(*ptimes)++;
	}

	if ((*ptimes) >= ELIMINATEDITHERING_SAMPLING_TIMES)
	{

		for (ucCounter = 0; ucCounter < pIOReadGroup->ucNum; ucCounter++)
		{
			if (pIOReadGroup->ucIOLowLevelTimesArray[ucCounter] > JUDGE_LOW_LEVEL_TIMES)
			{
#if STM32_MODE	//32测试环境
				switch (pIOReadGroup->ucIOAlias[ucCounter])
				{
				case 20:
					intx20 = IO_SET;
					break;
				case 21:
					intx21 = IO_SET;
					break;
				case 22:
					intx22 = IO_SET;
					break;
				case 23:
					intx23 = IO_SET;
					break;
				case 24:
					intx24 = IO_SET;
					break;
				case 25:
					intx25 = IO_SET;
					break;
				case 26:
					intx26 = IO_SET;
					break;
				case 27:
					intx27 = IO_SET;
					break;
				default:
					break;
				}
#endif
			}
		}
		(*ptimes) = 0;
		for (ucCounter = 0; ucCounter < pIOReadGroup->ucNum; ucCounter++)
		{
			if(pIOReadGroup->ucNum >READIO_NUM_MAX)
			break;
		pIOReadGroup->ucIOLowLevelTimesArray[ucCounter] = 0;
		}

		return 1;
	}

	return 0;

}
