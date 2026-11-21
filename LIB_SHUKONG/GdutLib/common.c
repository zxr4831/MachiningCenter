#include "common.h"
#include <stdio.h>
#include <string.h>
#include "GeneralFunctions.h"
#include "Channel.h"
#include "Config.h"
/***********************SGCodeParseBuf循环缓冲区-变量函数*********************************/

//struct SInterpolationPre g_SInterpolation;  //插补预处理结构体



volatile union  SSystemUnion g_SSystemParaUnion;  //系统参数结构体

uint16 g_usReloadValue[AXIS_PHYSICS_NUM_MAX];//各个轴发送脉冲的预装载值

uint16 g_usCompareValue[AXIS_PHYSICS_NUM_MAX];//各个轴发送脉冲的比较值

uint16 g_usCounterValue[AXIS_PHYSICS_NUM_MAX];//各个轴发送脉冲的计数器值 //增加2026.2.7

int16 g_sAxisDeltaPulse[AXIS_PHYSICS_NUM_MAX];//各个轴发送的脉冲数

int8 g_cMotionDirection[AXIS_PHYSICS_NUM_MAX];//各个轴的运行方向 //增加2026.2.7

struct SEncoder g_SEncoder[ENCODER_NUM_MAX];  //编码器结构体

enum RUNMODE g_RunMode;//运行模式

volatile VAL_PHYSICIAL_TYPE g_ucAxisOccupation;//轴占用变量  数据类型需要根据物理轴数量进行改变  当前物理轴个数为6   使用变量uint8   2022.12.6 从uint8类型修改为VAL_PHYSICIAL_TYPE

volatile VAL_PHYSICIAL_TYPE g_ucAxisExactEnable;// 物理轴的使能状态

#if(TURRET_AXIS_TEST == 1)
volatile VAL_TURRET_TYPE	g_ucTurretOccupation;//刀塔占用变量  数据类型需要根据刀塔数量进行改变  当前刀塔个数为1   使用变量uint8   0000 0101 表示刀塔1和刀塔3被占用
#endif

extern uint32 timeji;
#if CotrexF4 == 1
volatile unsigned long *pAxisEnable[AXIS_PHYSICS_NUM_MAX];// 物理轴对应的使能IO
#else

#endif // 0
//uint8 g_dAxisExactEnable[AXIS_PHYSICS_NUM_MAX];// 物理轴确切的使能状态

struct SdPiece g_sdtime; //企业定义的变量

// zxr: 保存通道0本次任务的计件资格，暂停及模式切换不改变任务来源
static volatile bool bWorkpieceCountEnabled = FALSE;



/****************************************映射相关****************************************/



/*************************************************
  函数名称：search(char array[], int n, char x)
  函数功能  查找数组元素对应的位置
  输入参数：cArray          数组指针  类型 char
			nArraySize      数组长度
			cTarget         目标元素
  输出参数：
  返 回 值：
			-1 目标元素不存在（要对这个进行判断）
  备    注：
**************************************************/
int search(uint8 cArray[], int nArraySize, uint8 cTarget)
{
	int i = 0;
	if (nArraySize == 0)  // 数组大小为0  返回-1
	{
		return -1;
	}

	for (i = 0; i < nArraySize; ++i)  //循环 nArraySize 次
	{
		if (cArray[i] == cTarget) //找到目标
		{
			return i;
		}
	}

	return -1;//未找到目标 返回-1

}


/****************************************实时坐标****************************************/

double g_dRealTimePointArray[AXIS_PHYSICS_NUM_MAX];//实时坐标  对应物理轴号


int8 clearRealTimePoint()
{
	int i = 0;
	for (i = 0; i < AXIS_PHYSICS_NUM_MAX; i++)
	{
		g_dRealTimePointArray[i] = 0;
	}
}

/*************************************************
  函数名称：getRealTimePoint(double *cArray, int nArraySize, const uint8* ucAxisMap)
  函数功能  获取实时点坐标，通过映射获取物理轴对应的坐标
  输入参数：nArraySize      数组长度
			ucAxisMap       映射关系数组
			ucValStatus		轴（值）有效标志
  输出参数：cArray			实时坐标（与结构体相对应）
  返 回 值：
			-1 数组长度为0
			-2 空指针
  备    注：
**************************************************/
int8 getRealTimePoint(double* cArray, int nArraySize, const uint8* ucAxisMap, const VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;

	if (nArraySize == 0) return -1;
	if (cArray == NULL || ucAxisMap == NULL) return -2;

	if (ucValStatus != NULL)
	{ //ucValStatus不为空  根据ucValStatus读取个别数据
		for (i = 0; i < nArraySize; i++)
		{
			if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX && VAL_VALID != getbit(*ucValStatus, i)) // 无效值时不读取 防止映射为空的物理轴  和 防止越界
			{
				continue;
			}
			else if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX && VAL_VALID == getbit(*ucValStatus, i)) //有效轴且映射为空时
			{
				parseRunningError(9999,GCODE_ERROR_NUM150,9999);
				return -2;
			}
			cArray[i] = g_dRealTimePointArray[ucAxisMap[i]];//从实时坐标获取值
		}
	}
	else
	{ //ucValStatus为空指针  全部读取
		for (i = 0; i < nArraySize; i++)
		{
			if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX) continue; //防止映射为空的物理轴  和 防止越界

			cArray[i] = g_dRealTimePointArray[ucAxisMap[i]];//从实时坐标获取值
		}
	}


	return 1;

}

/*************************************************
  函数名称：getSingleAxisRealTimePoint(double* dOutPut, uint8 ucAxis, const uint8* ucAxisMap)
  函数功能  获取实时点坐标，通过映射获取物理轴对应的坐标
  输入参数：ucAxis			轴号
			ucAxisMap       映射关系数组
  输出参数：dOutPut			输出坐标
  返 回 值：
			-1 轴号错误
			-2 空指针
			-3 无映射
  备    注：
**************************************************/
int8 getSingleAxisRealTimePoint(double* dOutPut, uint8 ucAxis, const uint8* ucAxisMap)
{
	if (ucAxis >= AXIS_NUM_MAX) return -1;
	if (ucAxisMap == NULL) return -2;
	if (ucAxisMap[ucAxis] >= AXIS_PHYSICS_NUM_MAX) return -3;
	*dOutPut = g_dRealTimePointArray[ucAxisMap[ucAxis]];
	return 1;
}

/*************************************************
  函数名称：updateRealTimePoint(double *cArray, int nArraySize, const uint8* ucAxisMap)
  函数功能  更新实时点坐标，通过映射将传入的坐标赋值到物理轴对应的坐标
  输入参数：cArray			实时坐标（与结构体相对应）
			nArraySize      数组长度
			ucAxisMap       映射关系数组
			ucValStatus		轴（值）有效标志
  输出参数：
  返 回 值：
			-1 数组长度为0
			-2 空指针
  备    注：
**************************************************/
int8 updateRealTimePoint(double* cArray, int nArraySize, const uint8* ucAxisMap, const VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;
	if (nArraySize == 0) return -1;   //数组大小为0 返回-1
	if (cArray == NULL || ucAxisMap == NULL) return -2;//空指针  返回-2

	if (ucValStatus != NULL)
	{ //ucValStatus不为空  根据ucValStatus读取个别数据
		for (i = 0; i < nArraySize; i++)
		{
			if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != getbit(*ucValStatus, i)) continue; // 轴无效时不写入  防止映射为空的物理轴 和  防止越界

			g_dRealTimePointArray[ucAxisMap[i]] = cArray[i];  //赋值到实时坐标
		}
	}
	else
	{//ucValStatus为空指针  全部写入
		for (i = 0; i < nArraySize; i++)
		{
			if (ucAxisMap[i] >= AXIS_PHYSICS_NUM_MAX) continue; //防止映射为空的物理轴 和  防止越界

			g_dRealTimePointArray[ucAxisMap[i]] = cArray[i];  //赋值到实时坐标
		}
	}


	return 1;
}

/*************************************************
  函数名称：updateSingleAxisRealTimePoint(double* dInPut, uint8 ucAxis, const uint8* ucAxisMap)
  函数功能  获取实时点坐标，通过映射获取物理轴对应的坐标
  输入参数：ucAxis			轴号
			ucAxisMap       映射关系数组
  输出参数：dOutPut			输出坐标
  返 回 值：
			-1 轴号错误
			-2 空指针
			-3 无映射
  备    注：
**************************************************/
int8 updateSingleAxisRealTimePoint(double* dInPut, uint8 ucAxis, const uint8* ucAxisMap)
{
	if (ucAxis >= AXIS_NUM_MAX) return -1;
	if (ucAxisMap == NULL) return -2;
	if (ucAxisMap[ucAxis] >= AXIS_PHYSICS_NUM_MAX) return -3;
	g_dRealTimePointArray[ucAxisMap[ucAxis]] = *dInPut;
	return 1;
}


/////////根据通道号获取坐标
// 
/*************************************************
  函数名称：getRealTimePointByChannel(uint8 ucChannelNum, double* cArray)
  函数功能  获取某通道的实时点坐标
  输入参数：ucChannelNum	通道号
  输出参数：cArray			实时坐标（与结构体相对应）
  返 回 值：
			-1 数组长度为0
			-2 空指针
			-3 通道号错误
  备    注：
**************************************************///	
int8 getRealTimePointByChannel(uint8 ucChannelNum, double* cArray)
{
	const uint8* ucMapArray;			//映射
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -3;
	}
	ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;
	return getRealTimePoint(cArray, AXIS_NUM_MAX, ucMapArray, NULL);
}



//	获取单轴实时点坐标
/*************************************************
  函数名称：getSingleAxisRealTimePointByChannel(uint8 ucChannelNum, double* dOutPut, uint8 ucAxis)
  函数功能  获取某通道的单轴实时点坐标
  输入参数：ucChannelNum	通道号
			ucAxis			轴号
  输出参数：dOutPut			输出坐标
  返 回 值：
			-1 轴号错误
			-2 空指针
			-3 无映射
			-4 通道号错误
  备    注：
**************************************************/
int8 getSingleAxisRealTimePointByChannel(uint8 ucChannelNum, double* dOutPut, uint8 ucAxis)
{
	const uint8* ucMapArray;			//映射
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -4;
	}
	ucMapArray = g_SChannel[ucChannelNum].pMapAxisArray;
	return getSingleAxisRealTimePoint(dOutPut, ucAxis, ucMapArray);
}




/****************************************************轴互斥函数*************************************************************/

int initAxisEnableStatus()
{
	g_ucAxisExactEnable = PHYSICIAL_TYPE_MAX_VAL;
}

/*************************************************
  函数名称：setAxisEnableStatus(uint8 ucPhysicalAxis,uint8 ucStatus)
  函数功能  修改轴的使能状态
  输入参数：ucPhysicalAxis		物理轴号
			ucStatus			状态   1为使能  0为不使能
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 setAxisEnableStatus(uint8 ucPhysicalAxis,uint8 ucStatus)
{
	if (ucPhysicalAxis>= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (ucStatus == 0)
	{
		clrbit(g_ucAxisExactEnable, ucPhysicalAxis);
	}
	else
	{
		setbit(g_ucAxisExactEnable, ucPhysicalAxis);
	}
	
	return 1;
}



/*************************************************
  函数名称：structToPhysical(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize, VAL_PHYSICIAL_TYPE* ucOutput)
  函数功能  从结构体 转换到 物理轴
  输入参数：ucSign		结构体的有效轴（位表示转换哪些位）
			ucAxisMap	映射
			nSize		数组大小
  输出参数：ucOutput	物理轴（位表示）
  返 回 值：对应物理轴的有效轴
  备    注：
**************************************************/
int8 structToPhysical(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize, VAL_PHYSICIAL_TYPE* ucOutput)
{
	int	i = 0;
	*ucOutput = 0;
	if (ucAxisMap == NULL)//传入的映射数组为空
	{
		return -1;
	}
	for (i = 0; i < nSize; i++)
	{
		if ((ucSign & 0x01) == 0x01)//位为1 寻找对应物理轴
		{
			if (ucAxisMap[i] < AXIS_PHYSICS_NUM_MAX)
			{	//置对应的物理轴
				setbit(*ucOutput, ucAxisMap[i]);
			}
			else //有效轴但映射为空的轴 2026.4.23
			{
				parseRunningError(9999,GCODE_ERROR_NUM150,9999);
				return -2;
			}
		}
		ucSign = ucSign >> 1;//移位
	}
	return 1;
}

/*************************************************
  函数名称：axisManagerCheck(VAL_PHYSICIAL_TYPE ucSign)
  函数功能  轴查询函数  查询对应位的轴是否空闲 空闲返回TRUE  不空闲返回FALSE
  输入参数：sign	物理轴 有效轴（用位描述指定的轴）
  输出参数：
  返 回 值：TRUE	对应轴空闲
			FALSE	对应轴被占用
  备    注：
**************************************************/
bool axisManagerCheck(VAL_PHYSICIAL_TYPE ucSign)
{
	if ((~(g_ucAxisOccupation|~g_ucAxisExactEnable) & ucSign) == ucSign)   // 朱稳中 2023.3.30  增加轴占用时 轴状态的判断 g_ucAxisExactEnable
		return TRUE;
	else
		return FALSE;
}



/*************************************************
  函数名称：axisManagerOccupy(VAL_PHYSICIAL_TYPE ucSign)
  函数功能  先查询轴是否空闲  目标所有轴都空闲时占用目标轴   不空闲时占用失败 返回FASLE
  输入参数：sign	有效轴（用位描述指定的轴）
  输出参数：
  返 回 值：TRUE	占用成功
			FALSE	占用失败
  备    注：
**************************************************/
bool axisManagerOccupy(VAL_PHYSICIAL_TYPE ucSign)
{
	if ((~(g_ucAxisOccupation | ~g_ucAxisExactEnable) & ucSign) == ucSign)
	{//对应的轴空闲  可以占用
		g_ucAxisOccupation |= ucSign; // 改变轴状态
		return TRUE;
	}
	else
	{	// 占用失败
		return FALSE;
	}
}



/*************************************************
  函数名称：axisManagerRelease(VAL_PHYSICIAL_TYPE ucSign)
  函数功能  根据目标轴释放（位置为0）对应的轴
  输入参数：sign	有效轴（用位描述指定的轴）
  输出参数：
  返 回 值：TRUE	释放成功
  备    注：
**************************************************/
bool axisManagerRelease(VAL_PHYSICIAL_TYPE ucSign)
{
	g_ucAxisOccupation &= (~ucSign);
	return TRUE;
}




/*************************************************
  函数名称：axisManagerCheckByStruct(uint8 ucSign, uint8* ucAxisMap, int nSize)
  函数功能  轴查询函数  将结构体的轴映射为物理轴  查询对应的物理轴是否空闲 空闲返回1  不空闲返回0
  输入参数：ucSign		有效轴（用位描述指定的轴）
			ucAxisMap	物理轴映射数组
			nSize		数组长度（也是有效轴的有效位个数）
  输出参数：
  返 回 值：-1	空指针
			-2	长度错误
			 0	对应轴被占用
			 1	对应轴空闲
  备    注：
**************************************************/
int8 axisManagerCheckByStruct(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize)
{
	VAL_PHYSICIAL_TYPE ucPhysicalSign = 0x0;

	if (ucAxisMap == NULL)//空指针
	{
		return -1;
	}
	if (nSize <= 0)//长度判断
	{
		return -2;
	}
	structToPhysical(ucSign, ucAxisMap, nSize, &ucPhysicalSign);//结构体轴根据映射转换为物理轴

	if (axisManagerCheck(ucPhysicalSign) == TRUE)//查询物理轴是否空闲
	{//空闲
		return 1;
	}
	else
	{//占用
		return 0;
	}

}



/*************************************************
  函数名称：axisManagerOccupyByStruct(uint8 ucSign, uint8* ucAxisMap, int nSize)
  函数功能  轴占用函数  将结构体的轴映射为物理轴  查询对应的物理轴是否空闲 空闲时占用且返回1  非空闲时返回0
  输入参数：ucSign		有效轴（用位描述指定的轴）
			ucAxisMap	物理轴映射数组
			nSize		数组长度（也是有效轴的有效位个数）
  输出参数：
  返 回 值：-1	空指针
			-2	长度错误
			 0	对应轴被占用
			 1	占用轴成功
  备    注：
**************************************************/
int8 axisManagerOccupyByStruct(VAL_STRUCT_TYPE ucSign, const  uint8* ucAxisMap, int nSize, VAL_STRUCT_TYPE *pSaveVal)
{
	VAL_PHYSICIAL_TYPE ucPhysicalSign = 0x0;

	if (ucAxisMap == NULL)//空指针
	{
		return -1;
	}
	if (nSize <= 0)//长度判断
	{
		return -2;
	}
	if (pSaveVal != NULL)
	{
		ucSign &= ~(*pSaveVal);
		if (ucSign == 0)
		{
			return 1;
		}
	}
	structToPhysical(ucSign, ucAxisMap, nSize, &ucPhysicalSign);//结构体轴根据映射转换为物理轴

	if (axisManagerOccupy(ucPhysicalSign) == TRUE)//尝试占用轴
	{
		if (pSaveVal != NULL)
		{
			*pSaveVal |= ucSign;
		}
		return 1;
	}
	else
	{
		////判断对应的轴是否使能  如果未使能则使能
		//if (g_ucAxisExactEnable & ucPhysicalSign != ucPhysicalSign)
		//{
		//	channelAxisEnable()
		//}
		return 0;
	}
}



/*************************************************
  函数名称：axisManagerReleaseByStruct(uint8 ucSign, uint8* ucAxisMap, int nSize)
  函数功能  轴释放函数  将结构体的轴映射为物理轴  释放对应的物理轴
  输入参数：ucSign		有效轴（用位描述指定的轴）
			ucAxisMap	物理轴映射数组
			nSize		数组长度（也是有效轴的有效位个数）
  输出参数：
  返 回 值：-1	空指针
			-2	长度错误
			 1	释放轴成功
  备    注：
**************************************************/
int8 axisManagerReleaseByStruct(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize, VAL_STRUCT_TYPE* pSaveVal)
{
	VAL_PHYSICIAL_TYPE ucPhysicalSign = 0x0;

	if (ucAxisMap == NULL)//空指针
	{
		return -1;
	}
	if (nSize <= 0)//长度判断
	{
		return -2;
	}
	structToPhysical(ucSign, ucAxisMap, nSize, &ucPhysicalSign);//结构体轴根据映射转换为物理轴

	//释放轴
	axisManagerRelease(ucPhysicalSign);
	if (pSaveVal != NULL)
	{
		*pSaveVal &= (~ucSign);
	}
	return 1;
}

/* 苏向标 2026-10-14：独立推进主轴停止，避免停止依赖启动使能完成。 */
/*************************************************
  函数名称：reserveAxisControl
  函数功能：仅由运动周期保留轴控制权，不请求使能，不关中断
  输入参数：ucSign - 逻辑轴掩码；pMapArray - 物理轴映射；pOccupation - 当前流程的资源记录
  输出参数：更新资源占用记录
  返 回 值：1 - 完成；0 - 等待；负值 - 异常
  备    注：苏向标 2026-10-14
****************************************/

int8 reserveAxisControl(VAL_STRUCT_TYPE ucSign, const uint8* pMapArray, VAL_STRUCT_TYPE* pOccupation)
{
    VAL_PHYSICIAL_TYPE ucPhysicalSign = 0; /**< 待保留的物理轴。 */
    int8 cResult; /**< 映射或资源申请结果。 */
    if (pMapArray == NULL || pOccupation == NULL)
    {
        return -1;
    }
    /* g_ucAxisOccupation为共享物理轴位图；仅周期检查并登记本次新取得的占用。 */
    ucSign &= ~(*pOccupation);
    cResult = structToPhysical(ucSign, pMapArray, AXIS_NUM_MAX, &ucPhysicalSign);
    if (cResult == 1 && (g_ucAxisOccupation & ucPhysicalSign) == 0)
    {
        g_ucAxisOccupation |= ucPhysicalSign;
        *pOccupation |= ucSign;
    }
    else if (cResult == 1)
    {
        cResult = 0;
    }
    return cResult;
}

//	轴占用函数（通过通道） 轴未使能时，会自动使能
int8 axisManagerOccupyByChannel(VAL_STRUCT_TYPE ucSign, uint8 ucChannelNum, VAL_STRUCT_TYPE* pSaveVal)
{
	VAL_PHYSICIAL_TYPE ucPhysicalSign = 0x0;
	const uint8* ucAxisMap;
	int8 nRes;
	if (ucChannelNum >=CHANNEL_NUM_MAX)
	{
		return -1;
	}
	ucAxisMap = g_SChannel[ucChannelNum].pMapAxisArray;
	if (ucAxisMap == NULL)//空指针
	{
		return -2;
	}
	if (pSaveVal != NULL)
	{
		ucSign &= ~(*pSaveVal);
		if (ucSign == 0)
		{
			return 1;
		}
	}
	nRes = structToPhysical(ucSign, ucAxisMap, AXIS_NUM_MAX, &ucPhysicalSign);//结构体轴根据映射转换为物理轴
	if (nRes != 1)return nRes;
	
	if (axisManagerOccupy(ucPhysicalSign) == TRUE)//尝试占用轴
	{
		if (pSaveVal != NULL)
		{
			*pSaveVal |= ucSign;
		}
		return 1;
	}
	else
	{
		//判断对应的轴是否使能  如果未使能则使能
		if ((g_ucAxisExactEnable & ucPhysicalSign) != ucPhysicalSign)
		{
			channelAxisEnable(ucChannelNum);
		}
		return 0;
	}

}

//	轴释放函数（通过通道）
int8 axisManagerReleaseByChannel(VAL_STRUCT_TYPE ucSign, uint8 ucChannelNum, VAL_STRUCT_TYPE* pSaveVal)
{
	if (ucChannelNum>= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	return axisManagerReleaseByStruct(ucSign, g_SChannel[ucChannelNum].pMapAxisArray, AXIS_NUM_MAX, pSaveVal);
}




//递归打印位
void __printfBits(int nSign, int nSize)
{
	if (nSize <= 0) return; //杀死那个递归
	__printfBits(nSign >> 1, nSize - 1);//递归递归
	printf("%1d", (nSign & 0x01));  //打印第1位的bit
	if (nSize % 4 == 0) printf(" ");//遇4不决  打个空格
}

/*************************************************
  函数名称：axisManagerCheck(uint8 sign)
  函数功能  递归打印目标变量的各个位
  输入参数：sign	目标变量
			nSize	目标变量长度（位）   = sizeof() * 8
  输出参数：
  返 回 值： 1	打印成功
			-1	失败
  备    注：
**************************************************/
int8 printfBits(int nSign, int nSize)
{
	if (nSize <= 0)return -1; // <=0 怎么打印？
	__printfBits(nSign, nSize);//打印nSign的 nSize位
	printf("\n");//最后换个行
	return 1;
}

/*************************************************
  函数名称：machineTimerStart(uint8 ucChannelNum)
  函数功能  开启加工计时
  输入参数：ucChannelNum 通道号
  输出参数：
  返 回 值： 1	成功
			-1	失败
			-2  已启动
  备    注：
**************************************************/
int8 machineTimerStart(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX)//超过通道号
	{
		return -1;
	}
	if (g_SChannel[ucChannelNum].SMachineTimer.isTimerBegined == TRUE)
	{
		return -2;
	}
	g_SChannel[ucChannelNum].SMachineTimer.isTimerBegined = TRUE;// 标记为启动
	g_SChannel[ucChannelNum].SMachineTimer.unStartTime = timeji;//记录当前工件启动时间
	
	return 1;
}


/*************************************************
  函数名称：machineTimerMark(uint8 ucChannelNum)
  函数功能  单件加工完成标记
  输入参数：ucChannelNum 通道号
  输出参数：
  返 回 值： 1	成功
			-1	失败
  备    注：
**************************************************/
int8 machineTimerMark(uint8 ucChannelNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX )//超过通道号
	{
		return -1;
	}

	if (timeji > g_SChannel[ucChannelNum].SMachineTimer.unStartTime) //判断是否溢出
	{ //计算startTime到timji的时间
		g_SChannel[ucChannelNum].SMachineTimer.unCurProcessTime = timeji - g_SChannel[ucChannelNum].SMachineTimer.unStartTime;
	}
	else
	{
		g_SChannel[ucChannelNum].SMachineTimer.unCurProcessTime = 0xFFFFFFFF - g_SChannel[ucChannelNum].SMachineTimer.unStartTime + timeji;
	}
#if STM32_MODE
	volatile int32 value = atomic_read(&timeji);//原子操作性地从timeji中获取值
	atomic_write(&g_SChannel[ucChannelNum].SMachineTimer.unStartTime, value);
#else
	g_SChannel[ucChannelNum].SMachineTimer.unStartTime = timeji;//记录当前时间为开始时间
#endif
	return 1;
}


/*************************************************
  函数名称：machineTimerInterp(uint8 ucChannelNum)
  函数功能  总加工时间计时
  输入参数：ucChannelNum 通道号
  输出参数：
  返 回 值： 1	成功
			-1	失败
  备    注：
**************************************************/
int8 machineTimerInterp(uint8 ucChannelNum)
{
	if (ucChannelNum > 0)//计时只计算通道0
	{
		return -1;
	}
	if (g_SChannel[ucChannelNum].SMachineTimer.isTimerBegined == TRUE)
	{
		g_sdtime.ucProcessTimeTotal += 1;//总加工时间自增
	}
	return 1;
}


/*************************************************
  函数名称：calWorkpieceNum(uint8 ucChannelNum)
  函数功能  工件计数功能
  输入参数：ucChannelNum 通道号
  输出参数：
  返 回 值： 1	已达到需求件数
             0  未达到需求件数
            -1	非计件通道
            -2  本次不计件或未设置需求件数
  备    注：
**************************************************/
int8 calWorkpieceNum(uint8 ucChannelNum)
{
    if (ucChannelNum > 0)
    {
        return -1;
    }
    // zxr: 维护任务不更新工件数，也不返回需求件数已达标
    if (!bWorkpieceCountEnabled)
    {
        return -2;
    }
    g_sdtime.unWorkPieceTotal += 1;
    g_sdtime.unCurWorkPieceNum += 1;//当前工件数+1
    // 计件
    if (g_SSystemPara.unNeededWorkPieceNum != 0)//需求工件数不为0
    {
        if (g_SSystemPara.unNeededWorkPieceNum <= g_sdtime.unCurWorkPieceNum)//工件数达标
        {
            return 1;
        }
        return 0;
    }
    return -2;
}

//初始化工件数 当前数大于等于目标工件数时，清零，其他情况不清零
int8 initWorkpieceNum(uint8 ucChannelNum)
{
    if (ucChannelNum > 0)
    {
        return -1;
    }
    // zxr: 新任务按启动模式刷新计件资格，维护任务不清除已达标的当前件数
    bWorkpieceCountEnabled = (g_RunMode == AUTOMATIC_MODE);
    if (!bWorkpieceCountEnabled)
    {
        return -2;
    }
    if (g_SSystemPara.unNeededWorkPieceNum != 0 &&g_SSystemPara.unNeededWorkPieceNum <= g_sdtime.unCurWorkPieceNum)//工件数达标
    {
        g_sdtime.unCurWorkPieceNum = 0;
    }
    return 1;
}

//加工时间清零
int8 machineTimerClear(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)//判断通道号
    {
        return -1;
    }
    g_sdtime.ucProcessTimeTotal = 0;
    g_SChannel[ucChannelNum].SMachineTimer.unCurProcessTime = 0;
    return 1;
}

/*************************************************
  函数名称：int32 atomic_read(volatile int32 * ptr)
  函数功能  STM32模式下原子操作性地获取全局变量的值
  输入参数：volatile int32 * ptr  全局变量地址
  输出参数：
  返 回 值： 全局变量的值
			
			
  备    注：
**************************************************/

//int32 atomic_read(volatile int32 * ptr)
//{
//	int32 value=0;//取值
//#if STM32_MODE
//
//	do
//	{
//		value = __LDREXW(ptr);//对该段内存做了独占标记
//	} while (__STREXW(value, ptr));//尝试更新该段内存的值
//#endif // STM32_MODE
//	return value;
//
//}

/*************************************************
  函数名称：void atomic_write(volatile int32* ptr, volatile int32 value)
  函数功能  STM32模式下原子操作性地给取全局变量赋值
  输入参数：volatile int32 * ptr  全局变量地址
            volatile int32 value 赋给全局变量的值
  输出参数：
  返 回 值：


  备    注：
**************************************************/
//void atomic_write(volatile int32* ptr, volatile int32 value)
//{
//  volatile int32 temp = 0;//取值
//#if STM32_MODE
//	do
//	{
//		temp = __LDREXW(ptr);//对该段内存做了独占标记
//		temp = value;//更新局部变量的值
//	} while (__STREXW(temp,ptr));//尝试更新该段内存的值
//#endif // STM32_MODE
//	return ;
//
//}

/*************************************************
  函数名称：void atomic_write(volatile int32* ptr, volatile int32 value)
  函数功能  原子操作性地对 ptr指向的内存中值进行加法运算
  输入参数：volatile int32 * ptr  全局变量地址
			volatile int32 value 赋给全局变量的值
  输出参数：
  返 回 值：
  备    注：
**************************************************/

//int32 atomic_add(volatile uint32* ptr, int32 value)
//{
//	volatile int32 temp=0;
//#if STM32_MODE
//	do
//	{
//		temp = __LDREXW(ptr);//对该段内存做了独占标记
//		temp = value+temp;//更新局部变量的值
//	} while (__STREXW(temp, ptr));//尝试更新该段内存的值
//#endif // STM32_MODE
//	return temp;
//}

#if(TURRET_AXIS_TEST == 1)


/*************************************************
  函数名称：int8 turretAxisManagerOccupyByChannel(VAL_STRUCT_TYPE ucSign, uint8 ucChannelNum, VAL_STRUCT_TYPE* pSaveValAxis, VAL_TURRET_TYPE * pSaveValTurret)
  函数功能   占用刀塔轴和刀塔，并返回对应的刀塔结构体轴，刀塔
  输入参数： VAL_STRUCT_TYPE ucValAxis  目标结构体轴（包含了刀塔轴和非刀塔轴）
			  uint8 ucChannelNum  通道号
			  VAL_STRUCT_TYPE* pOccupideAxis  已经占用的结构体轴（包含了刀塔轴和非刀塔轴）
			  VAL_STRUCT_TYPE* pOccupideTurretAxis  已经占用的刀塔轴
			  VAL_TURRET_TYPE*pOccupideTurret  已经占用的刀塔
  输出参数：    VAL_STRUCT_TYPE* pOccupideAxis  已经占用的结构体轴（包含了刀塔轴和非刀塔轴）
			  VAL_STRUCT_TYPE* pOccupideTurretAxis  已经占用的刀塔轴
			  VAL_TURRET_TYPE*pOccupideTurret  已经占用的刀塔
  返 回 值：    1：占用刀塔与刀塔轴成功
				0：占用失败的原因是刀塔物理轴未使能  ，可以继续调用该函数进行占用
			   -1：占用失败的原因是通道与刀塔的映射设置错误 或 刀塔与结构体轴的映射错误  ，不可以继续调用该函数进行占用， 而是检查映射有没有错误或报错
			   -2：传入空指针   报错
  备    注：
**************************************************/

int8 turretAxisAndTurretOccupyByChannel(VAL_STRUCT_TYPE* ucValAxis, uint8 ucChannelNum, VAL_STRUCT_TYPE* pOccupideAxis, VAL_STRUCT_TYPE* pOccupideTurretAxis, VAL_TURRET_TYPE* pOccupideTurret)
{
	VAL_PHYSICIAL_TYPE ucPhysicalSign = 0x0;
	VAL_TURRET_TYPE ucTargetTurret = 0x0;//目标刀塔
	VAL_STRUCT_TYPE ucTurretAxis = 0x0;//刀塔结构体轴

	const uint8* ucAxisMap;
	const uint8* ucTurretmap;
	uint8 temp = 0;
	uint8 tempTurretAxis = 0;


	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	ucAxisMap = g_SChannel[ucChannelNum].pMapAxisArray;//通道的结构体轴与物理轴的映射
	ucTurretmap = g_SChannel[ucChannelNum].pChannelToTurretmap;//通道对刀塔的映射
	if (ucAxisMap == NULL || ucTurretmap == NULL)//空指针
	{
		return -2;
	}
	if (pOccupideAxis != NULL)
	{
		//剔除掉已经占用的结构体轴
		*ucValAxis &= ~(*pOccupideAxis);
		//已经占用的结构体轴中包含了即将要占用的结构体轴,占用成功
		if (*ucValAxis == 0)
		{
			if (pOccupideAxis != NULL)
			{
				*ucValAxis = *pOccupideAxis;
			}
			return 1;
		}
	}

	//遍历该通道映射的刀塔，查看哪些刀塔有这根结构体轴
	for (; temp < TURRET_MAX_NUM; temp++)
	{
		//如果通道与该刀塔不存在映射，跳过
		if (ucTurretmap[temp] == TURRET_NULL)
		{
			continue;
		}
		else
		{

			tempTurretAxis = (*ucValAxis & g_SSystemPara.ucTurretToAxisMap[ucTurretmap[temp] - 1]);
			//如果即将要占用结构体轴中有属于本次遍历的刀塔的
			if (tempTurretAxis != 0)
			{
				//用局部变量记录该目标刀塔  （遍历结束后，该局部变量记录了所有目标刀塔）
				setbit(ucTargetTurret, (ucTurretmap[temp] - 1));
				//用局部变量记录刀塔结构体轴  （遍历结束后，该局部变量记录了所有目标刀塔的刀塔结构体轴）
				ucTurretAxis |= g_SSystemPara.ucTurretToAxisMap[ucTurretmap[temp] - 1];


			}
		}

	}
	//占用刀塔
	if (pOccupideTurret != NULL)
	{
		//剔除掉已经占用的刀塔
		ucTargetTurret &= ~(*pOccupideTurret);

	}
	//如果还有刀塔需要占用
	if (ucTargetTurret != 0)
	{
		//尝试占用刀塔
		if (turretManagerOccupy(ucTargetTurret) == TRUE)
		{
			//记录占用成功的刀塔
			if (pOccupideTurret != NULL)
			{
				*pOccupideTurret |= ucTargetTurret;
			}
		}
		else
		{
			//占用刀塔失败，退出
			return 0;
		}
	}
	//如果ucTargetTurret == 0，即已经占用的刀塔中包含了即将要占用的刀塔, 并不代表刀塔轴也已经占用成功，因为换刀时只占用刀塔没占用刀塔轴


	//占用刀塔轴
	//如果目标结构体轴中有属于刀塔结构体的，占用刀塔轴
	if (ucTurretAxis != 0)
	{
		if (pOccupideAxis != NULL)
		{
			//剔除掉已经占用的结构体轴
			ucTurretAxis &= ~(*pOccupideAxis);
			//已经占用的结构体轴中包含了即将要占用的结构体轴,占用成功
			if (ucTurretAxis == 0)
			{
				return 1;
			}
		}
		structToPhysical(ucTurretAxis, ucAxisMap, AXIS_NUM_MAX, &ucPhysicalSign);//结构体轴根据映射转换为物理轴

		if (axisManagerOccupy(ucPhysicalSign) == TRUE)//尝试占用刀塔轴
		{

			if (pOccupideAxis != NULL)
			{
				//在通道已经占用的结构体轴变量中记录本次占用的刀塔结构体轴
				*pOccupideAxis |= ucTurretAxis;
			}
			if (pOccupideTurretAxis != NULL)
			{
				//在通道已经占用的刀塔结构体轴变量中记录本次占用的刀塔结构体轴
				*pOccupideTurretAxis |= ucTurretAxis;
			}
			if (pOccupideAxis != NULL)
			{
				*ucValAxis = *pOccupideAxis;
			}
			return 1;
		}
		else
		{
			//判断对应的轴是否使能  如果未使能则使能
			if ((g_ucAxisExactEnable & ucPhysicalSign) != ucPhysicalSign)
			{
				channelAxisEnable(ucChannelNum);
				return 0;
			}
			else
			{
				// 目标结构体轴属于刀塔轴，但是从通道对轴的映射中查询不到该轴，说明刀塔与刀塔结构体轴的映射错误
				return -1;
			}

		}

	}
	//目标结构体轴不属于刀塔轴，返回
	else   
	{
		return 1;// 目标结构体轴不属于刀塔轴
	}
}





/*************************************************
  函数名称：axisManagerOccupy(VAL_PHYSICIAL_TYPE ucSign)
  函数功能  先查询刀塔是否空闲  目标所有刀塔都空闲时占用目标刀塔   不空闲时占用失败 返回FASLE
  输入参数：sign	有效轴（用位描述指定的轴）
  输出参数：
  返 回 值：TRUE	占用成功
			FALSE	占用失败
  备    注：
**************************************************/
bool turretManagerOccupy(VAL_TURRET_TYPE ucSign)
{
	if (((~g_ucTurretOccupation) & ucSign) == ucSign)
	{//对应的刀塔空闲  可以占用
		g_ucTurretOccupation |= ucSign; // 改变刀塔状态
		return TRUE;
	}
	else
	{	// 占用失败
		return FALSE;
	}
}


/*************************************************
  函数名称：turretManagerRelease(VAL_TURRET_TYPE ucSign)
  函数功能  根据目标刀塔释放对应的刀塔
  输入参数：sign	目标刀塔（用位描述指定的刀塔）
  输出参数：
  返 回 值：TRUE	释放成功
  备    注：
**************************************************/
bool turretManagerRelease(VAL_TURRET_TYPE ucSign)
{
	g_ucTurretOccupation &= (~ucSign);
	return TRUE;
}

/*************************************************
  函数名称：int8 releaseAllOccupiedAxis(struct SInterpolationPre* pSPreProcess)
  函数功能  释放本通道所有占用的轴
  输入参数：
  输出参数：
  返 回 值：-1 输入有误
			1 释放成功

  备    注：
**************************************************/
int8 releaseAllOccupiedAxis(struct SInterpolationPre* pSPreProcess)
{
	/* 释放前必须取得有效通道，避免访问未绑定的同步状态。 */
    if (pSPreProcess == NULL)
	{
		return -1;
	}
	/* 释放轴所有权后，旧攻丝相位不可再作为相邻段参考。 */
	uint8* ucMapArray = NULL;
	ucMapArray = pSPreProcess->pSChannel->pMapAxisArray;//获取映射
	if (ucMapArray == NULL)//空指针
	{
		return -1;
	}
	axisManagerReleaseByStruct(pSPreProcess->ucOccupideValStatus, ucMapArray, AXIS_NUM_MAX, NULL);//释放占用的所有轴
	pSPreProcess->ucOccupideTurretAxis = 0x0;//已占用的刀塔轴置0
	pSPreProcess->ucOccupideValStatus = 0x0;//已占用的结构体轴置0
	return 1;
}

/*************************************************
  函数名称：int8 releaseAllOccupiedAxis(struct SInterpolationPre* pSPreProcess)
  函数功能  释放本通道所有占用的刀塔
  输入参数：
  输出参数：
  返 回 值：-1 输入有误
			1 释放成功

  备    注：
**************************************************/
int8 releaseAllOccupiedTurret(struct SInterpolationPre* pSPreProcess)
{
	if (pSPreProcess == NULL)
	{
		return -1;
	}
	turretManagerRelease(pSPreProcess->ucOccupideTurret);//释放占用的刀塔
	pSPreProcess->ucOccupideTurret = 0x0;//已占用的刀塔置0
	return 1;
}

#endif


#if CotrexF4 == 0  //芯片选择 H7
/*************************************************
  函数名称：uint8 pAxisEnable(uint8 AXIS_PHYSICS)
  函数功能  获取IO使能状态
  输入参数：uint8 AXIS_PHYSICS  物理轴号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
uint8 pAxisEnable(uint8 AXIS_PHYSICS)
{
	switch (AXIS_PHYSICS)
	{
	case 0: return(PHout(4));
	case 1: return(PCout(3));
	case 2: return(PHout(2));
	case 3: return(PCout(2));
	case 4: return(PCout(0));
	case 5: return (PHout(5));
	}
}
/*************************************************
  函数名称：uint8 pAxisEnableSet(uint8 AXIS_PHYSICS, uint8 value)
  函数功能  设置IO使能状态
  输入参数：uint8 AXIS_PHYSICS  物理轴号
			uint8 value         使能值
  输出参数：
  返 回 值：
  备    注：
**************************************************/
uint8 pAxisEnableSet(uint8 AXIS_PHYSICS, uint8 value)
{
	switch (AXIS_PHYSICS)
	{
	case 0:(PHout(4)) = value; break;
	case 1:(PCout(3)) = value; break;
	case 2:(PHout(2)) = value; break;
	case 3:(PCout(2)) = value; break;
	case 4:(PCout(0)) = value; break;
	case 5:(PHout(5)) = value; break;
	}
}

#endif
