#include "SysParaAPI.h"
#include "Channel.h"
#include "ToolChanger.h"
#include "common.h"
#include "CTrajPlanning.h"
#include "ErrorFunction.h"
#include "Gcode.h"
#include "GeneralFunctions.h"
#include "lookAhead.h"
#include "ToolCompensation.h"
#include <stdlib.h>
#include "IOCommand.h"
#include "Spindle.h"
#include "G32.h"
#include "G1.h" 
#include "G0.h"  
#include "Motion.h"	
#include <string.h>
#include "G2.h"
#include "Channel.h"
#include "Config.h"
#include "ErrorCompensation.h"


#if STM32_MODE
#include "sysio.h"
#include "sys.h"//IO
#endif // STM32_MODE



/*************************************************
函数名称：initMapArray()
函数功能	映射初始化  初始化系统参数时调用
            1.设置通道对有效轴的映射，以及这些有效轴对物理轴的映射（该通道可以控制哪些轴，以及这些轴由哪些电机控制）
			2.设置通道对刀塔的映射（该通道可以控制哪些刀塔）
			3.设置刀塔对轴的映射（刀塔可沿哪些轴移动）
输入参数：
输出参数：
返 回 值：
备    注：  注意：如果两个通道都可以对同一个刀塔进行映射，在与这个刀塔有映射关系的有效轴中，两个通道都对某个有效轴有映射关系，
那么两个通道在对这一个有效轴设置物理轴时必须一致，否则会出现映射错误的情况。
比如刀塔1与 有效轴X，Z轴有映射关系，通道0和通道1都可以控制刀塔1，但是通道0中的X轴由0号电机控制，通道1中的X轴由1号电机控制，
那么会存在一个映射错误：刀塔1的X轴在通道0由0号电机控制，在通道1中由1号电机控制，同一个刀塔的相同的轴名对应不同的电机，
即发生同一个物理轴由不同的电机控制的错误。！！！！
本数控系统中，通道对物理轴的映射是轴映射的最底层，所有有效轴对物理轴的映射都需要通过通道对物理轴的映射去查找。所以，
在刀塔对于有效轴的映射中，这些有效轴所对应的物理轴要根据可以控制该刀塔的通道的通道对物理轴的映射去查找。
****************************************/
void initMapArray()
{
#if( TURRET_AXIS_TEST==1)
	memset(g_SSystemPara.ucChannelToTurretmap, TURRET_NULL, CHANNEL_NUM_MAX * TURRET_MAX_NUM);//初始化通道与刀塔的映射
	memset(g_SSystemPara.ucTurretToAxisMap, AXIS_NULL, TURRET_MAX_NUM);//初始化刀塔与结构体轴的映射
#endif

#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE)	//Y轴双通道机配置
	memset(g_SSystemPara.ucAxisMapArray, AXIS_NULL, CHANNEL_NUM_MAX * AXIS_NUM_MAX);//映射全部指为空
	//通道对于物理轴的映射
	g_SSystemPara.ucAxisMapArray[0][AXIS_X] = 0; // 0
	g_SSystemPara.ucAxisMapArray[0][AXIS_Y] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_Z] = 2; // 1
	g_SSystemPara.ucAxisMapArray[0][AXIS_A] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_B] = 4;
	g_SSystemPara.ucAxisMapArray[0][AXIS_C] = 3; // 4
#if( TURRET_AXIS_TEST==1)

	//通道与刀塔的映射
	//g_SSystemPara.ucChannelToTurretmap[0][0] = 1; //通道0可以使用刀塔1
	setChannelToTurretMapArray(0, 0, 1);//通道0可以使用刀塔1
	//刀塔与有效轴的映射
	//g_SSystemPara.ucTurretToAxisMap[0] = 5;// 0000 0101 （位变量，00 CBA ZYX）刀塔1

	//设置函数setTurretToAxisMap(x，y)中的参数x时要注意与通道对刀塔的映射一致，如上面设置通道0对刀塔1有映射，那么x为1
	// 设置函数setTurretToAxisMap(x,y)中的参数y时要注意与通道对结构体轴的映射一致(c轴不算)，如上面设需通道0对xcz有映射，除去c轴，那么x为000 0101的十进制5
	setTurretToAxisMap(1,5);// 0000 0101 （位变量，0000 0Z0X）刀塔1可沿zx轴移动
#endif

#if CHANNEL_NUM_MAX > 1
	//通道对于物理轴的映射
	g_SSystemPara.ucAxisMapArray[1][AXIS_X] = AXIS_NULL;//3
	g_SSystemPara.ucAxisMapArray[1][AXIS_Y] = 1;
	g_SSystemPara.ucAxisMapArray[1][AXIS_Z] = AXIS_NULL;//4
	g_SSystemPara.ucAxisMapArray[1][AXIS_A] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_B] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_C] = AXIS_NULL;//5

#if( TURRET_AXIS_TEST==1)

   //通道与刀塔的映射
//	g_SSystemPara.ucChannelToTurretmap[1][0] = 2; //通道1可以使用刀塔2
	setChannelToTurretMapArray(1, 0, 2);//通道1可以使用刀塔2
	//刀塔与有效轴的映射
	//设置函数setTurretToAxisMap(x，y)中的参数x时要注意与通道对刀塔的映射一致，如上面设置通道1对刀塔2有映射，那么x为2
	// 设置函数setTurretToAxisMap(x,y)中的参数y时要注意与通道对结构体轴的映射一致(c轴不算)，如上面设需通道1对y有映射，除去c轴，那么x为000 0010的十进制2
	//g_SSystemPara.ucTurretToAxisMap[1] = 2;// 0000 0010 （位变量，00 CBA ZYX）刀塔2
	setTurretToAxisMap(2, 2);// 0000 0010 （位变量，0000 00Y0）刀塔2可沿y轴移动
#endif
#endif

#elif (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE)	//双主轴双通道机配置
	//通道与物理轴的映射
	memset(g_SSystemPara.ucAxisMapArray, AXIS_NULL, CHANNEL_NUM_MAX * AXIS_NUM_MAX);//映射全部指为空
	g_SSystemPara.ucAxisMapArray[0][AXIS_X] = 0; // 0
	g_SSystemPara.ucAxisMapArray[0][AXIS_Y] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_Z] = 2; // 1
	g_SSystemPara.ucAxisMapArray[0][AXIS_A] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_B] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_C] = 3; // 4

#if( TURRET_AXIS_TEST==1)
	//映射全部指为空

   //通道与刀塔的映射
	//g_SSystemPara.ucChannelToTurretmap[0][0] = 1; //通道0可以使用刀塔1
	setChannelToTurretMapArray(0, 0, 1);//通道0可以使用刀塔1

	//刀塔与有效轴的映射
	//g_SSystemPara.ucTurretToAxisMap[0] = 5;// 0000 0101 （位变量，00 CBA ZYX）刀塔1
	//设置函数setTurretToAxisMap(x，y)中的参数x时要注意与通道对刀塔的映射一致，如上面设置通道0对刀塔1有映射，那么x为1
	// 设置函数setTurretToAxisMap(x,y)中的参数y时要注意与通道对结构体轴的映射一致(c轴不算)，如上面设需通道0对xcz有映射，除去c轴，那么x为000 0101的十进制5
	setTurretToAxisMap(1, 5);// 0000 0101 （位变量，0000 0Z0X）刀塔1可沿zx轴移动

#endif



#if CHANNEL_NUM_MAX > 1
	//通道与物理轴的映射
	g_SSystemPara.ucAxisMapArray[1][AXIS_X] = 0;//3
	g_SSystemPara.ucAxisMapArray[1][AXIS_Y] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_Z] = 2;//4
	g_SSystemPara.ucAxisMapArray[1][AXIS_A] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_B] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_C] = 5;//5

#if( TURRET_AXIS_TEST==1)
	//g_SSystemPara.ucChannelToTurretmap[1][0] = 1;//通道1可以使用刀塔1
	//通道与刀塔的映射
	setChannelToTurretMapArray(1, 0, 1);//通道1可以使用刀塔1
	//刀塔与有效轴的映射
	//设置函数setTurretToAxisMap(x，y)中的参数x时要注意与通道对刀塔的映射一致，如上面设置通道1对刀塔1有映射，那么x为1
	// 设置函数setTurretToAxisMap(x,y)中的参数y时要注意与通道对结构体轴的映射一致(c轴不算)，如上面设需通道1对xzc有映射，除去c轴，那么x为000 0101的十进制5
	//g_SSystemPara.ucTurretToAxisMap[1] = 2;// 0000 0010 （位变量，00 CBA ZYX）刀塔2
	setTurretToAxisMap(1, 5);// 0000 0010 （位变量，0000 00Y0）刀塔1可沿zx轴移动
#endif
#endif

#else
	memset(g_SSystemPara.ucAxisMapArray, AXIS_NULL, CHANNEL_NUM_MAX * AXIS_NUM_MAX);//映射全部指为空
	g_SSystemPara.ucAxisMapArray[0][AXIS_X] = 0; // 0
	g_SSystemPara.ucAxisMapArray[0][AXIS_Y] = 1;
	g_SSystemPara.ucAxisMapArray[0][AXIS_Z] = 2; // 1
	g_SSystemPara.ucAxisMapArray[0][AXIS_A] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_B] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[0][AXIS_C] = 3; // 4
#if( TURRET_AXIS_TEST==1)

//通道与刀塔的映射
	setChannelToTurretMapArray(0, 0, 1);//通道0可以使用刀塔1
//刀塔与有效轴的映射
	//设置函数setTurretToAxisMap(x，y)中的参数x时要注意与通道对刀塔的映射一致，如上面设置通道0对刀塔1有映射，那么x为1
	// 设置函数setTurretToAxisMap(x,y)中的参数y时要注意与通道对结构体轴的映射一致(c轴不算)，如上面设需通道0对xyzc有映射，除去c轴，那么x为000 0111的十进制7
	//如果配置为普通二轴机时，则参数y为000 0111的十进制7
	//如果配置为车铣复合机时通道对A轴有映射，则参数y为00001111的十进制15
	setTurretToAxisMap(1, 7);//7为二进制 0000 0111的十进制， （该二进制数为位变量，00000 0ZYX）表示刀塔1 可沿XYZ轴 移动

#endif


#if CHANNEL_NUM_MAX > 1
	g_SSystemPara.ucAxisMapArray[1][AXIS_X] = 4;//3
	g_SSystemPara.ucAxisMapArray[1][AXIS_Y] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_Z] = 1;//4
	g_SSystemPara.ucAxisMapArray[1][AXIS_A] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_B] = AXIS_NULL;
	g_SSystemPara.ucAxisMapArray[1][AXIS_C] = 5;//5

	//根据实际需要设置通道对刀塔的映射，调用setChannelToTurretMapArray设置


	//根据实际需要设置刀塔对有效轴的映射，调用setTurretToAxisMap设置


#endif // CHANNAL_NUM_MAX > 1
#endif // CHANNAL_NUM_MAX > 1

}

/*************************************************
函数名称：initPWMMapArray()
函数功能	 PWM映射初始化  初始化系统参数时调用
		原为一一对应，表示计算的PWM参数与对应定时器对应，从而发送PWM信号给电机，
		现可实现更改，可将0口计算的PWM从1口发送，实现后，原虚拟周-物理轴的映射无需更改

输入参数：
输出参数：
返 回 值：
备    注：
****************************************/
void initPWMMapArray()
{
	g_SSystemPara.ucPWMAxisMapArray[0][AXIS_X] = 0;
	g_SSystemPara.ucPWMAxisMapArray[0][AXIS_Y] = 1;
	g_SSystemPara.ucPWMAxisMapArray[0][AXIS_Z] = 2;
	g_SSystemPara.ucPWMAxisMapArray[0][AXIS_A] = 3;
	g_SSystemPara.ucPWMAxisMapArray[0][AXIS_B] = 4;
	g_SSystemPara.ucPWMAxisMapArray[0][AXIS_C] = 5;

}

/*************************************************
/*************************************************
  函数名称：initEncoderMapArray()
  函数功能	初始化  初始化编码器映射时调用
  输入参数：
  输出参数：
  返 回 值：
  备    注：
****************************************/
void initEncoderMapArray()
{
	memset(g_SSystemPara.ucEncoderMapArray, ENCODER_NULL, CHANNEL_NUM_MAX * ENCODER_NUM_MAX);//映射全部指为空
	g_SSystemPara.ucEncoderMapArray[0][SPINDLE_ENCODER - 1] = 0;
	g_SSystemPara.ucEncoderMapArray[0][COMMON_ENCODER - 1] = 1;
	g_SSystemPara.ucEncoderMapArray[0][HANDEWHEEL_ENCODER - 1] = 2;
	g_SSystemPara.ucEncoderMapArray[0][SECONDSPINDLE_ANALOGENCODER - 1] = 3;

#if CHANNEL_NUM_MAX > 1
	g_SSystemPara.ucEncoderMapArray[1][SPINDLE_ENCODER - 1] = 1;
	g_SSystemPara.ucEncoderMapArray[1][COMMON_ENCODER - 1] = ENCODER_NULL;
	g_SSystemPara.ucEncoderMapArray[1][HANDEWHEEL_ENCODER - 1] = 2;
	g_SSystemPara.ucEncoderMapArray[1][SECONDSPINDLE_ANALOGENCODER - 1] = 3;
#endif // CHANNAL_NUM_MAX > 1
 
}





/*************************************************
  函数名称：initPhysicalAxisPara(uint8 ucPhysicalAxis)
  函数功能  物理轴初始化参数
  输入参数：ucPhysicalAxisNum	物理轴号	0~AXIS_PHYSICS_NUM_MAX-1
			type				LINEAR_AXIS  直线轴   ROTATION_AXIS 旋转轴
  输出参数：
  返 回 值：1 设置完成
			-1 物理轴号错误
  备    注：
****************************************/
int8 initPhysicalAxisPara(uint8 ucPhysicalAxis,uint8 type)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (type == ROTATION_AXIS)//旋转轴
	{
		//C 轴
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType = ROTATION_AXIS;//旋转轴
		g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].bShowByDiameter = FALSE;//直径显示
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAccVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe = 20.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVmax = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE; //  度/ms
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVs = 20.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickVel = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE;;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowVel = 300.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm = 1.0 / 10000 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse = 10000;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerDeltaVMax = 0;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerAMax = 0.0001 * 360.0;          //非反向时速度变化加速度   郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dCoefOfAngleToLength = 1.0;          //旋转轴角度转换系数
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dClearance = 0;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist = 0;
	}
	else
	{
		//单轴 X轴初始化
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType = LINEAR_AXIS;//直线轴
		g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].bShowByDiameter = TRUE;//直径显示
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxAcc = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxDec = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxAcc = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxDec = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxAcc = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxDec = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVmax = 3000.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVs = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickVel = 0.1;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickAcc = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickDec = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowVel = 0.005;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowAcc = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowDec = 0.001;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm = 1.0 / 1000;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse = 1000;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerDeltaVMax = 0.0;     //单轴转向允许变化的最大速度 mm/s
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerAMax = 0.001;       //非反向时速度变化加速度   郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dClearance = 0;
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist = 0;
		#if CotrexF4 == 1
		if (pAxisEnable[ucPhysicalAxis] == IO_RESET)
		#else
		if (pAxisEnable(ucPhysicalAxis) == IO_RESET)
		#endif
		{
			setAxisEnableStatus(ucPhysicalAxis, 0);
		}
	}

	return 1;
}


/*************************************************
  函数名称：initSingleAxisPara(uint8 ucChannelNum)
  函数功能  单轴参数初始化 根据结构体初始化对应的 物理轴参数
  输入参数：ucChannelNum		通道号		0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：
  备    注：
****************************************/
int8 initStructSingleAxisPara(uint8 ucChannelNum)
{
	struct SChannel* pSchannel;
	int i;
	const uint8* ucMapArray;
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}

	pSchannel = &g_SChannel[ucChannelNum];
	ucMapArray = pSchannel->pMapAxisArray;

	if (pSchannel->pMapAxisArray[AXIS_X] < AXIS_PHYSICS_NUM_MAX)
	{
		//单轴 X轴初始化
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].ucAxisType = LINEAR_AXIS;//直线轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].bShowByDiameter = TRUE;//直径显示
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dSysPlanTMaxAcc =  0.004;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dSysPlanTMaxDec =  0.004;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dSysPlanS3MaxAcc = 0.004;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dSysPlanS3MaxDec = 0.004;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dSysPlanS4MaxAcc = 0.004;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dSysPlanS4MaxDec = 0.004;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dVe = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dVmax = 18000.0 / 60 / 1000;     
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dVs = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dManualQuickVel = 0.05;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dManualQuickAcc = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dManualQuickDec = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dManualSlowVel = 0.005;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dManualSlowAcc = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dManualSlowDec = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dPulseToMm = 1.0 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].usMmToPulse = 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dVeerDeltaVMax = 0.0;    //单轴转向允许变化的最大速度 mm/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dVeerAMax = 0.001;       //非反向时速度变化加速度     mm/ms2   郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dClearance = 0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].dRevClearanceCompDist = 0;
		#if CotrexF4 == 1
		if (pAxisEnable[ucMapArray[AXIS_X]] == IO_RESET)
		#else
		if (pAxisEnable(ucMapArray[AXIS_X]) == IO_RESET)
		#endif
		{
			setAxisEnableStatus(ucMapArray[AXIS_X], 0);
		}
	}
	if (pSchannel->pMapAxisArray[AXIS_Y] < AXIS_PHYSICS_NUM_MAX)
	{
		////单轴 Y轴初始化
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].ucAxisType = LINEAR_AXIS;//直线轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].bShowByDiameter = TRUE;//直径显示
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dSysPlanTMaxAcc = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dSysPlanTMaxDec = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dSysPlanS3MaxAcc = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dSysPlanS3MaxDec = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dSysPlanS4MaxAcc = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dSysPlanS4MaxDec = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dVe = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dVmax = 3000.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dVs = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dManualQuickVel = 0.05;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dManualQuickAcc = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dManualQuickDec = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dManualSlowVel = 0.005;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dManualSlowAcc = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dManualSlowDec = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dPulseToMm = 1.0 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].usMmToPulse = 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dVeerDeltaVMax = 0.0;//单轴转向允许变化的最大速度 mm/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dVeerAMax = 0.001;       //非反向时速度变化加速度  mm/ms2  郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dClearance = 0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Y]].dRevClearanceCompDist = 0;

		#if CotrexF4 == 1
		if (pAxisEnable[ucMapArray[AXIS_Y]] == IO_RESET)
		#else
		if (pAxisEnable(ucMapArray[AXIS_Y]) == IO_RESET)
		#endif
		{
			setAxisEnableStatus(ucMapArray[AXIS_Y], 0);
		}
	}
	if (pSchannel->pMapAxisArray[AXIS_Z] < AXIS_PHYSICS_NUM_MAX)
	{
		////单轴 Z轴初始化
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].ucAxisType = LINEAR_AXIS;//直线轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].bShowByDiameter = FALSE;//直径显示
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dSysPlanTMaxAcc = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dSysPlanTMaxDec = 0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dSysPlanS3MaxAcc =0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dSysPlanS3MaxDec =0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dSysPlanS4MaxAcc =0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dSysPlanS4MaxDec =0.002;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dVe = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dVmax = 18000.0 /60 /1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dVs = 200.0 / 60 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dManualQuickVel = 0.05;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dManualQuickAcc = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dManualQuickDec = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dManualSlowVel = 0.005;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dManualSlowAcc = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dManualSlowDec = 0.001;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dPulseToMm = 1.0 / 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].usMmToPulse = 1000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dVeerDeltaVMax = 0.0;//单轴转向允许变化的最大速度 mm/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dVeerAMax = 0.001;       //非反向时速度变化加速度 mm/ms2郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dClearance = 0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_Z]].dRevClearanceCompDist = 0;
		#if CotrexF4 == 1
		if (pAxisEnable[ucMapArray[AXIS_Z]] == IO_RESET)
		#else
		if (pAxisEnable(ucMapArray[AXIS_Z]) == IO_RESET)
		#endif
		{
			setAxisEnableStatus(ucMapArray[AXIS_Z], 0);
		}
	}
	if (pSchannel->pMapAxisArray[AXIS_A] < AXIS_PHYSICS_NUM_MAX)
	{
		//A轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].ucAxisType = ROTATION_AXIS;//旋转轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].bShowByDiameter = TRUE;//直径显示
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].ucAccVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].ucDecVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dSysPlanTMaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dSysPlanTMaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dSysPlanS3MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dSysPlanS3MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dSysPlanS4MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dSysPlanS4MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dVe = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dVmax = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE; //  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dVs = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dManualQuickVel = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE;;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dManualQuickAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dManualQuickDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dManualSlowVel = 300.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dManualSlowAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dManualSlowDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dPulseToMm = 1.0 / 10000 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].usMmToPulse = 10000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dVeerDeltaVMax = 0.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dVeerAMax = 0.0001 * 360.0;          //非反向时速度变化加速度   郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dCoefOfAngleToLength = 1.0;          //旋转轴角度转换系数
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dClearance = 0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_A]].dRevClearanceCompDist = 0;
		#if CotrexF4 == 1
		if (pAxisEnable[ucMapArray[AXIS_A]] == IO_RESET)
		#else
		if (pAxisEnable(ucMapArray[AXIS_A]) == IO_RESET)
		#endif
		{
			setAxisEnableStatus(ucMapArray[AXIS_A], 0);
		}
	}
	if (pSchannel->pMapAxisArray[AXIS_B] < AXIS_PHYSICS_NUM_MAX)
	{
		//B轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].ucAxisType = ROTATION_AXIS;//旋转轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].bShowByDiameter = TRUE;//直径显示
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].ucAccVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].ucDecVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dSysPlanTMaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dSysPlanTMaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dSysPlanS3MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dSysPlanS3MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dSysPlanS4MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dSysPlanS4MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dVe = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dVmax = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE; //  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dVs = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dManualQuickVel = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE;;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dManualQuickAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dManualQuickDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dManualSlowVel = 300.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dManualSlowAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dManualSlowDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dPulseToMm = 1.0 / 10000 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].usMmToPulse = 10000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dVeerDeltaVMax = 0.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dVeerAMax = 0.0001 * 360.0;          //非反向时速度变化加速度   郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dCoefOfAngleToLength = 1.0;          //旋转轴角度转换系数
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dClearance = 0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_B]].dRevClearanceCompDist = 0;
#if CotrexF4 == 1
		if (pAxisEnable[ucMapArray[AXIS_B]] == IO_RESET)
#else
		if (pAxisEnable(ucMapArray[AXIS_B]) == IO_RESET)
#endif
		{
			setAxisEnableStatus(ucMapArray[AXIS_B], 0);
		}
	}
	if (pSchannel->pMapAxisArray[AXIS_C] < AXIS_PHYSICS_NUM_MAX)
	{
		//C 轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].ucAxisType = ROTATION_AXIS;//旋转轴
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].fDistPerCircle = 360.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].bShowByDiameter = FALSE;//直径显示
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].ucAccVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].ucDecVelPlanType = VEL_PLAN_V3_CURVE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dSysPlanTMaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dSysPlanTMaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dSysPlanS3MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dSysPlanS3MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dSysPlanS4MaxAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dSysPlanS4MaxDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVe = 20.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVmax = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE; //  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVs = 20.0 / (60 * 1000) * DEGREE_OF_CIRCLE;	//  度/ms
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dManualQuickVel = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE;;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dManualQuickAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dManualQuickDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dManualSlowVel = 300.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dManualSlowAcc = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dManualSlowDec = 0.0001 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dPulseToMm = 1.0 / 10000 * DEGREE_OF_CIRCLE;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].usMmToPulse = 10000;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVeerDeltaVMax = 0.0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dVeerAMax = 0.0001 * 360.0;          //非反向时速度变化加速度   郭强 2022/06/13
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dCoefOfAngleToLength = 1.0;          //旋转轴角度转换系数
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dClearance = 0;
		g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dRevClearanceCompDist = 0;
		#if CotrexF4 == 1
		if (pAxisEnable[ucMapArray[AXIS_C]] == IO_RESET)
		#else
		if (pAxisEnable(ucMapArray[AXIS_C]) == IO_RESET)
		#endif
		{
			setAxisEnableStatus(ucMapArray[AXIS_C], 0);

		}
	}


	//initPhysicalAxisPara(pSchannel->pMapAxisArray[AXIS_A], LINEAR_AXIS);

	//g_SSystemPara.MOTOR_WARN[AXIS_X].soft_pos = 1;
	//g_SSystemPara.MOTOR_WARN[AXIS_X].soft_neg = 1;
	//g_SSystemPara.MOTOR_WARN[AXIS_Z].soft_pos = 1;
	//g_SSystemPara.MOTOR_WARN[AXIS_Z].soft_neg = 1;
	//g_SSystemPara.MOTOR_WARN[AXIS_X].pos_soft = 100;
	//g_SSystemPara.MOTOR_WARN[AXIS_X].neg_soft = -100;
	//g_SSystemPara.MOTOR_WARN[AXIS_Z].pos_soft = 100;
	//g_SSystemPara.MOTOR_WARN[AXIS_Z].neg_soft = -100 ;

	return 1;
}

/*************************************************
  函数名称：initAxisPara()
  函数功能  初始化物理轴+ 结构体轴参数
  输入参数：
  输出参数：
  返 回 值：1 完成
  备    注：
****************************************/
int8 initAxisPara()
{
	uint8 ucCounter;
	int i = 0;
	//for (i = 0; i < AXIS_PHYSICS_NUM_MAX; i++)
	//{
	//	initPhysicalAxisPara(i);
	//}
	for (i = 0; i < CHANNEL_NUM_MAX; i++)
	{
		initStructSingleAxisPara(i);
	}
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		SG0LoadSysPara(&g_SChannel[ucCounter].SG00);//G0初始化
		SG1LoadSysPara(&g_SChannel[ucCounter].SG01);//G1初始化
		SG2LoadSysPara(&g_SChannel[ucCounter].SG02);//G2初始化
		SG32LoadSysPara(&g_SChannel[ucCounter].SG32);  //G32初始化
	}
	return 1;
}

/*************************************************
  函数名称：int8 initCustomSUserData()
  函数功能  以自定义表初始化SG0 SG1 SG2结构体的UserData(开机自动启动)
  输入参数：
  输出参数：
  返 回 值：1 完成
  备    注：
****************************************/
int8 initCustomSUserData()//2024.12.9 熊骏
{
	struct SInput* pSUserData;

	for (int i = 0; i < CHANNEL_NUM_MAX; i++)
	{
		//G0初始化
		{
			if (g_SSystemPara.ucAxisMapArray[i][AXIS_X] != AXIS_NULL)
			{
				//X轴初始化
				pSUserData = &g_SChannel[i].SG00.pSSingleAxisArray[AXIS_X].SUserData;
				pSUserData->ucAccVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->ucDecVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->dVs = 200.0 / 60 / 1000;
				pSUserData->dFeedrate = 18000.0 / 60 / 1000;
				pSUserData->dPrimalVe = 200.0 / 60 / 1000;
				pSUserData->dFeedRatio = 1.0;
				pSUserData->dAcc = 0.004;
				pSUserData->dDec = 0.004;
				pSUserData->SEndPoint = (SPointDouble) { 0 };
				pSUserData->cInterpDir = 1;
			}
			if (g_SSystemPara.ucAxisMapArray[i][AXIS_Y] != AXIS_NULL)
			{
				//Y轴初始化
				pSUserData = &g_SChannel[i].SG00.pSSingleAxisArray[AXIS_Y].SUserData;
				pSUserData->ucAccVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->ucDecVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->dVs = 200.0 / 60 / 1000;
				pSUserData->dFeedrate = 3000.0 / 60 / 1000;
				pSUserData->dPrimalVe = 200.0 / 60 / 1000;
				pSUserData->dFeedRatio = 1.0;
				pSUserData->dAcc = 0.002;
				pSUserData->dDec = 0.002;
				pSUserData->SEndPoint = (SPointDouble) { 0 };
				pSUserData->cInterpDir = 1;
			}
			if (g_SSystemPara.ucAxisMapArray[i][AXIS_Z] != AXIS_NULL)
			{
				//Z轴初始化
				pSUserData = &g_SChannel[i].SG00.pSSingleAxisArray[AXIS_Z].SUserData;
				pSUserData->ucAccVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->ucDecVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->dVs = 200.0 / 60 / 1000;
				pSUserData->dFeedrate = 18000.0 / 60 / 1000;
				pSUserData->dPrimalVe = 200.0 / 60 / 1000;
				pSUserData->dFeedRatio = 1.0;
				pSUserData->dAcc = 0.002;
				pSUserData->dDec = 0.002;
				pSUserData->SEndPoint = (SPointDouble) { 0 };
				pSUserData->cInterpDir = 1;
			}
			if (g_SSystemPara.ucAxisMapArray[i][AXIS_A] != AXIS_NULL)
			{
				//A轴初始化
				pSUserData = &g_SChannel[i].SG00.pSSingleAxisArray[AXIS_A].SUserData;
				pSUserData->ucAccVelPlanType = VEL_PLAN_V3_CURVE;
				pSUserData->ucDecVelPlanType = VEL_PLAN_V3_CURVE;
				pSUserData->dVs = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
				pSUserData->dFeedrate = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
				pSUserData->dPrimalVe = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
				pSUserData->dFeedRatio = 1.0;
				pSUserData->dAcc = 0.0001 * DEGREE_OF_CIRCLE;
				pSUserData->dDec = 0.0001 * DEGREE_OF_CIRCLE;
				pSUserData->SEndPoint = (SPointDouble) { 0 };
				pSUserData->cInterpDir = 1;
			}
			if (g_SSystemPara.ucAxisMapArray[i][AXIS_B] != AXIS_NULL)
			{
				//B轴初始化
				pSUserData = &g_SChannel[i].SG00.pSSingleAxisArray[AXIS_B].SUserData;
				pSUserData->ucAccVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->ucDecVelPlanType = VEL_PLAN_S4_CURVE;
				pSUserData->dVs = 200.0 / 60 / 1000;
				pSUserData->dFeedrate = 3000.0 / 60 / 1000;
				pSUserData->dPrimalVe = 200.0 / 60 / 1000;
				pSUserData->dFeedRatio = 1.0;
				pSUserData->dAcc = 0.05;
				pSUserData->dDec = 0.05;
				pSUserData->SEndPoint = (SPointDouble) { 0 };
				pSUserData->cInterpDir = 1;
			}
			if (g_SSystemPara.ucAxisMapArray[i][AXIS_C] != AXIS_NULL)
			{
				//C轴初始化
				pSUserData = &g_SChannel[i].SG00.pSSingleAxisArray[AXIS_C].SUserData;
				pSUserData->ucAccVelPlanType = VEL_PLAN_V3_CURVE;
				pSUserData->ucDecVelPlanType = VEL_PLAN_V3_CURVE;
				pSUserData->dVs = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
				pSUserData->dFeedrate = 3000.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
				pSUserData->dPrimalVe = 200.0 / (60 * 1000) * DEGREE_OF_CIRCLE;
				pSUserData->dFeedRatio = 1.0;
				pSUserData->dAcc = 0.00005 * DEGREE_OF_CIRCLE;
				pSUserData->dDec = 0.00005 * DEGREE_OF_CIRCLE;
				pSUserData->SEndPoint = (SPointDouble) { 0 };
				pSUserData->cInterpDir = 1;
			}

		}
		//G1初始化
		pSUserData = &g_SChannel[i].SG01.SUserData;
		pSUserData->ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		pSUserData->ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		pSUserData->dVs = 200.0 / 60 / 1000;
		pSUserData->dFeedrate = 10000.0 / 60.0 / 1000;
		pSUserData->dPrimalVe = 200.0 / 60 / 1000;
		pSUserData->dFeedRatio = 1.0;
		pSUserData->dAcc = 0.001;
		pSUserData->dDec = 0.001;
		pSUserData->SEndPoint = (SPointDouble) { 0 };
		pSUserData->cInterpDir = 1;
		//G2初始化
		pSUserData = &g_SChannel[i].SG02.SUserData;
		pSUserData->ucAccVelPlanType = VEL_PLAN_S4_CURVE;
		pSUserData->ucDecVelPlanType = VEL_PLAN_S4_CURVE;
		pSUserData->dVs = 200.0 / 60 / 1000;
		pSUserData->dFeedrate = 10000.0 / 60.0 / 1000;
		pSUserData->dPrimalVe = 200.0 / 60 / 1000;
		pSUserData->dFeedRatio = 1.0;
		pSUserData->dAcc = 0.001;
		pSUserData->dDec = 0.001;
		pSUserData->SEndPoint = (SPointDouble) { 0 };
		pSUserData->cInterpDir = 1;
	}
	return 1;
}

/*************************************************
  函数名称：initEditablePara()
  函数功能  可编辑参数初始化  初始化按钮调用此函数
  输入参数：
  输出参数：
  返 回 值：
  备    注：
****************************************/
void initEditablePara()
{
#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
	setHandwheelSimDistancePerTick(HANDWHEEL_SIM_DISTANCE_DEFAULT);
#endif

	int i = 0;
	g_SSystemPara.dInterpolationTime = 1; //插补周期

	//合成
	g_SSystemPara.dSysMinVel = 200 / 60 / 1000;    //最小速度
	g_SSystemPara.dSysMaxVel = 10000.0 / 60.0 / 1000;  //最大速度
	g_SSystemPara.dRecoveryMaxVel = 3000.0;//恢复加工最大速度 洪冠鹏 2023/12/21

	g_SSystemPara.dSysPlanTMaxAcc = 0.001;    //最大加速度
	g_SSystemPara.dSysPlanTMaxDec = 0.001;   //最大减速度

	g_SSystemPara.dSysPlanS3MaxAcc = 0.001;    //最大加速度
	g_SSystemPara.dSysPlanS3MaxDec = 0.001;   //最大减速度

	g_SSystemPara.dSysPlanS4MaxAcc = 0.001;    //最大加速度
	g_SSystemPara.dSysPlanS4MaxDec = 0.001;   //最大减速度

	g_SSystemPara.dArcMaxNormalAcc = 0.001;  //系统最大法向加速度（用于圆弧插补）     郭强 2022/06/13
	g_SSystemPara.dArcMaxChordError = 0.07;  //圆弧弓高误差（用于圆弧插补）           郭强 2022/06/13

	g_SSystemPara.ucDistReAverageType = DIST_REAVG_T_CURVE;	//平摊类型梯形平摊

	g_SSystemPara.ucCommonAccVelPlanType = VEL_PLAN_S4_CURVE;		//速度曲线类型，T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
	g_SSystemPara.ucCommonDecVelPlanType = VEL_PLAN_S4_CURVE;		//速度曲线类型，T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE

	g_SSystemPara.dG33AvoidEmptyDistance = 1.0;                       //G33避空距离    郭强    12.13
	g_SSystemPara.dG33RevCompDistance = 0.0;                             //G33反转补偿长度（0-2mm）   郭强   2021-12-14
	g_SSystemPara.bG33RecalDecPointFlag = FALSE;                          //G33重算减速点标志位        郭强   2022-01-10
	g_SSystemPara.dG33MaxVel = 0.05;                                 //G33最大运行速度    郭强 2022/8/22
/***********丝杠反向间隙补偿参数*********///郭强2022-1-7（G33反向补偿时速度规划参数也使用此参数）
	g_SSystemPara.dRevClearanceCompVs = 200.0 / 60 / 1000; //反向间隙补偿初速度（末速度按苏总要求暂取和初速度相同)mm/ms
	g_SSystemPara.dRevClearanceCompVmax = 0.05;            //反向间隙补偿最大速度mm/ms
	g_SSystemPara.dRevClearanceCompAcc = 0.001;            //反向间隙补偿最大加速度mm/ms2(苏总建议：默认使用T型加减速度)
	g_SSystemPara.dRevClearanceCompDec = 0.001;            //反向间隙补偿最大减速度mm/ms2(苏总建议：默认使用T型加减速度)
/****************************************************************************************/

/*******2022/10/24新增系统参数--郭强*******/
	g_SSystemPara.ucLookAheadMode = 1;		//前瞻模式   （1:直线轴精度模式  2:多轴(含旋转轴)模式） 删除原本未开发的2段间过渡，将原本多轴模式改模式序号改为2  (2023/05/19郭强)
	g_SSystemPara.ucInterpolationMode = 3;            //插补模式    (1：长轴插补 2：固定参数扩展线位移插补 3：自适应参数扩展线位移插补)
	g_SSystemPara.dSysLowerLimitSpeed = 0.0 / 60 / 1000;          //切削进给段间低速下限值(mm/ms)
	g_SSystemPara.ucGearOfSysLowerLimitSpeed = 1;                 //切削进给段间低速下限值档位(1:100%、2:125%、3:150%)
	g_SSystemPara.dPercentageOfG33EnterLeadCorrection = 1.0;      //G33攻牙攻入导程修正百分比（99%-101%）
	g_SSystemPara.dPercentageOfG33ReturnLeadCorrection = 1.007;	  //G33攻牙退刀导程修正百分比（99%-101%）
/******************************************/

/***** 2023/3/23新增系统参数--郭强*****/
	g_SSystemPara.dG33ReturnSpindleRatio = 1.0;           //G33主轴反转转速百分比        2023/5/27   郭强
	g_SSystemPara.unG33SpindlePauseTimeBeforeRev = 500;   //G33主轴反转前暂停时间(ms)    2023.5.8    郭强
/******************************************/

	g_SSystemPara.unSpindlePreDelayTime = 20; //主轴开延时  2s
	g_SSystemPara.unSpindlePostDelayTime = 20;	//主轴关延时  2s
	g_SSystemPara.unSpindleEnableTime = 30;  //主轴使能延时
	g_SSystemPara.unSpindleDisableTime = 30;//主轴取消使能延时
	g_SSystemPara.unChuckOpenDelayTime = 2000; //夹头开延时
	g_SSystemPara.unChuckCloseDelayTime = 2000;//夹头关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[0] = 1000; //自动送料缸1开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[0] = 1000; //自动送料缸1关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[1] = 1000; //自动送料缸2开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[1] = 1000; //自动送料缸2关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[2] = 1000; //自动送料缸3开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[2] = 1000; //自动送料缸3关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[3] = 1000; //自动送料缸4开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[3] = 1000; //自动送料缸4关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[4] = 1000; //自动送料缸5开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[4] = 1000; //自动送料缸5关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[5] = 1000; //自动送料缸6开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[5] = 1000; //自动送料缸6关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[6] = 1000; //自动送料缸7开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[6] = 1000; //自动送料缸7关延时
	g_SSystemPara.unAutoFeedOpenDelayTime[7] = 1000; //自动送料缸8开延时
	g_SSystemPara.unAutoFeedCloseDelayTime[7] = 1000; //自动送料缸8关延时

	g_SSystemPara.usEncoderLinesNum[0] = 2500;
	g_SSystemPara.usEncoderLinesNum[1] = 2500;
	g_SSystemPara.usEncoderLinesNum[2] = 100;

	g_SSystemPara.fSpindleSpeedDeviation = 10.0 / 100; //主轴转速允许误差( 0.0 - 1.0)    !!   1.0表示G1+不受到主轴转速限制
	g_SSystemPara.fG99SpindleSpeedDeviation = 10.0 / 100; //主轴转速允许误差( 0.0 - 1.0)    !!   1.0表示G1+不受到主轴转速限制
	g_SSystemPara.fSpindleStopDegree = 200;   //(0° -  359°)  准停角度
	g_SSystemPara.usSpindleAvgPeriod = 20; //主轴检测转速的周期数
	g_SSystemPara.dSysPlanSpindleDec = 0.0001 * DEGREE_OF_CIRCLE;  //用于主轴运动加速度，区分分度运动
	g_SSystemPara.dSysPlanSpindleAcc = 0.0001 * DEGREE_OF_CIRCLE;  //用于主轴运动减速度，区分分度运动
	g_SSystemPara.dSpindleVe = 0;            //主轴结束起跳速度
	g_SSystemPara.dSpindleVs = 0;			 //主轴启动起跳速度

	g_SSystemPara.ucPolarInterpolationAxis[AXIS_HORIZONTAL] = AXIS_X; // 极坐标横轴 X 轴
	g_SSystemPara.ucPolarInterpolationAxis[AXIS_VERTICAL] = AXIS_C; //极坐标纵轴 C 轴

	g_SSystemPara.usHandleWheelAvgPeriod = 20;  //手轮转速检测周期
	g_SSystemPara.fHandleWheelSpeedMax = 100; //达到最大速度的手轮转速
	g_SSystemPara.dSimulaVmax = 2000.0 / 60 / 1000;// 手轮模拟非G0最大进给速度
	g_SSystemPara.dSimulaG0Vmax = 2000.0 / 60 / 1000;// 手轮模拟G0最大进给速度
	g_SSystemPara.dSimulaMaxAcc = 0.0005;// 手轮模拟最大加速度
	g_SSystemPara.dSimulaMaxDec = 0.0005;// 手轮模拟最大减速度
	g_SSystemPara.ucLookAheadNum = 10; //前瞻段数
	g_SSystemPara.unSpindleLockPreDelayTime   =  500;	//主轴刹车锁紧前延时（ms)
	g_SSystemPara.unSpindleLockPostDelayTime = 500;		//主轴刹车锁紧后延时（ms)
	g_SSystemPara.unSpindleUnLockPreDelayTime = 500;	// 主轴刹车松开前延时（ms)
	g_SSystemPara.unSpindleUnLockPostDelayTime = 500;	//主轴刹车松开后延时（ms)
	g_SSystemPara.dRotationAxisG0SpeedMax = 4000.0/ 60 / 1000 * 360.0;//旋转轴G0的最大速度

	g_SSystemPara.sGcode_MultiLineHandleMaxNum = 1;				//G代码多行处理最大行数		//2023/3/15	陈敏聪
	g_SSystemPara.sToolComp_MultiLineHandleMaxNum = 1;			//刀补多行处理最大行数
	g_SSystemPara.sInterpolation_MultiLineHandleMaxNum = 1;		//插补多行处理最大行数
	g_SSystemPara.bGcodeQuickHandleModeEnabled = 0;				//多行循环处理功能

	g_SSystemPara.sG83toG89Para_RetractionLength = 1000;	//NC_DATA_PARAMETER_089 G83toG83退刀参数_退刀长度 um
	TIOSTRUCT_InitEditablePara();
	clearMachineTimeTotal();

	g_SSystemPara.ucEncoderphase[0] = 1;  //编码器1的相位为正相
	g_SSystemPara.ucEncoderphase[1] = 1;  //编码器2的相位为正相
	g_SSystemPara.ucEncoderphase[2] = 1;  //编码器3的相位为正相
	g_SSystemPara.ThreadCutMode = 1;      //G32切削模式为允许丢脉冲模式

	g_SSystemPara.dHandWheelVmax = 0.05;
	g_SSystemPara.dHandWheelAcc = 0.001;
	g_SSystemPara.dG32SacBase = 2;
	g_SSystemPara.ucG32InputMode = 0;  //G32系统参数来源，0为加工系统参数，1为对应单轴的系统参数  2023.5.19
	g_SSystemPara.ucRotationAxisInterpolationMode = ROTATION_INTERPOLATION_SHORT_DISTANCE;//旋转轴插补方式 默认短距离优先//洪冠鹏 2024.3.25
	g_SSystemPara.ucSpindleStopMode = SPINDLE_STOP_MODE_PRECISESTOP_MODE; // 主轴停方式   默认准停
	g_SSystemPara.dJDefault = 0;  
	g_SSystemPara.dKDefault = 0;


	g_SSystemPara.ucEmergencyStopType = 1;  //急停IO类型 常闭
	g_SSystemPara.dhandwheelcontrol = 0;//企业增加 2023/12/20


	g_SSystemPara.dG32MaxVelocity = 3000.0 / 60.0 /1000.0;		//G32螺纹专用速度
	g_SSystemPara.bIsStopSpindleAndCoolantwhileM30 = TRUE;		//默认为M30执行关主轴同时关冷却
	g_SSystemPara.bIsStopSpindleAndCoolantwhileReset = FALSE;	//默认为复位执行关主轴同时关冷却
	memset(&g_SSystemPara.SZeroPointOfCoordSys[0], 0, sizeof(g_SZeroPtOfCoordSys[0]));
	g_SSystemPara.ucTconstForTrapezoid = 0;//三角形修调后匀速周期占总周期的百分比 2024/3/22 洪冠鹏
	g_SSystemPara.bM01ResetAllChannel = TRUE; //M01报警后复位所有通道 TRUE:复位所有通道 FALSE:仅复位当前通道 洪冠鹏 2024/6/5

	memset(&g_SSystemPara.SMachine_REFPOT, 0, sizeof(g_SSystemPara.SMachine_REFPOT));//机械参考点 默认（0，0，0，0，0，0）
	g_SSystemPara.bNowStopflag = TRUE;//急停下，各轴停下的方式，TRUE为立即停，FALSE为减速停2025.5.9熊骏
	//setMachine_REFPOT(1, AXIS_X, 10);
	//setMachine_REFPOT(1, AXIS_Y, 10);
	//setMachine_REFPOT(1, AXIS_Z, 10);
	//setMachine_REFPOT(1, AXIS_C, 10);
	//g_SSystemPara.SMachine_REFPOT[1].dArray[AXIS_X] = 20;
	//g_SSystemPara.SMachine_REFPOT[1].dArray[AXIS_Y] = 20;
	//g_SSystemPara.SMachine_REFPOT[1].dArray[AXIS_Z] = 20;
	//g_SSystemPara.SMachine_REFPOT[1].dArray[AXIS_C] = 20;

	g_SSystemPara.dAngleXY1 = 30.0;		///Y1轴相对X轴的倾斜角度 30°		[倾斜Y轴机专用]
	g_SSystemPara.bUseVirtualY = TRUE;	//是否开启虚拟Y轴 默认开启：TRUE	[倾斜Y轴机专用]
	g_SSystemPara.bSetG7456mmTOum = FALSE;
	g_SSystemPara.unG84DefaultDelayTime = 500;  //G84孔底暂停默认时间，单位：ms
}

/*************************************************
  函数名称：void InitSystemMacro( )
  函数功能：系统宏初始化函数
  输入参数：
  输出参数：
  返 回 值：
  备    注：IO地址
**************************************************/
void InitSystemMacro()
{
	int index = 0;
	for (; index < 32; index++)
	{
		g_SystemMacro.MarcoAdd[index] = &(attribute[index][2]);
	}
	/*attribute[13][2] = 0;
	attribute[11][2] = 1;*/

}

/*************************************************
  函数名称：void initMacroVariable(uint8 ucControl)
  函数功能: 初始化宏变量
  输入参数：ucControl控制变量
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void initMacroVariable(uint8 ucControl)
{
	if (ucControl == 1)   //初始化前面100个
	{
		memset((void*)g_SSystemPara.SMacroVariable.ucIsNullArr, 255, 12);
		setbit(g_SSystemPara.SMacroVariable.ucIsNullArr[12], 0);  //97
		setbit(g_SSystemPara.SMacroVariable.ucIsNullArr[12], 1);  //98
		setbit(g_SSystemPara.SMacroVariable.ucIsNullArr[12], 2);  //99
		setbit(g_SSystemPara.SMacroVariable.ucIsNullArr[12], 3);  //100
		memset((void*)g_SSystemPara.SMacroVariable.dValArr, 0, 400);
	}
	else  //全部初始化
	{
		memset((void*)g_SSystemPara.SMacroVariable.ucIsNullArr, 255, 75);
		memset((void*)g_SSystemPara.SMacroVariable.dValArr, 0, 2400);
	}

}
/*************************************************
  函数名称：initSystemPara()
  函数功能
  输入参数：
  输出参数：
  返 回 值：
  备    注：若变量需要在每次"运行G代码完毕"之后重置，请标注
****************************************/
int8 initSystemPara()
{
#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
	setHandwheelSimDistancePerTick(HANDWHEEL_SIM_DISTANCE_DEFAULT);
#endif

	uint8 ucCounter = 0, ucCounter1 = 0;

    
	//1.G代码解析部分
	//1.1开辟G代码通道结构体内存
	for (uint8 ucChannelNum = 0; ucChannelNum < CHANNEL_NUM_MAX; ucChannelNum++)
	{
		s_pSChannelConfig[ucChannelNum] = mallocGCodeChannelConfig();//在动态内存中开辟G代码通道结构体
		if (s_pSChannelConfig[ucChannelNum] == NULL)
		{
			parseGCodeErrorB(0, GCODE_ERROR_NUM300);
			return -1;
		}

		memset(s_pSChannelConfig[ucChannelNum], 0, sizeof(SChannelConfiguration));
	}

	//1.2开辟GM指令规则数内存
	g_pSGCommandRegulationTree = creatTree(ORAM_NUM324);//创建G代码规则树
	if (g_pSGCommandRegulationTree == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	g_pSMCommandRegulationTree = creatTree(ORAM_NUM327);//创建G代码规则树
	if (g_pSMCommandRegulationTree == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	//1.3添加GM指令规则
	addAllGCommandRegulationAPI();//加载所有G代码规则
	addAllMCommandRegulationAPI();//加载所有M代码规则

	//2.刀补计算部分
	SToolCompensationRecord= initSToolCompensationRecord();//在动态内存中开辟存放刀补过程的中间点以及刀补的状态的结构体
	if (SToolCompensationRecord == NULL)
	{
		return -1;
	}
	memset(SToolCompensationRecord, 0, sizeof(SSToolCompensationRecord));


	//1.4初始化系统宏 陈育楠 2023.10.30
	InitSystemMacro();
	//初始化宏变量
	initMacroVariable(1);


	g_SChannel = initSChannel();//在动态内存中开辟通道结构体
	if (g_SChannel == NULL)
	{

		return -1;
	}
	memset(g_SChannel, 0, sizeof(struct SChannel) * CHANNEL_NUM_MAX);

	//g_SChannelManager= initChannelManager();// 在动态内存中开辟通道信号管理结构体
	//memset(g_SChannelManager,0, sizeof(struct SChannelManager));
	g_SChannel[0].cSubRunMode = 1;

	//g_SChannelManager= initChannelManager();// 在动态内存中开辟通道信号管理结构体
	//memset(g_SChannelManager,0, sizeof(struct SChannelManager));


	g_pChannelWaitSignalMannager = initChannelWaitSignalManager();//在动态内存中开辟通道等待信号管理结构体
	if(g_pChannelWaitSignalMannager != NULL)
	{
		memset(g_pChannelWaitSignalMannager, 0, sizeof(struct SChannelWaitSignalMannager));
	}



	/*******************************************系统参数初始化******************************************/

	g_SSystemPara.ucLookAheadNum = 10; //前瞻段数
	//g_SEncoder[1-1].usEncoderCirclePulseNum = g_SEncoder[1-1].usEncoderCirclePulseNum;  //主轴编码器转一圈需要的脉冲
	g_RunMode = AUTOMATIC_MODE;
	g_SEncoder[3 - 1].ucWheelMultiple = 1;
	g_SEncoder[3 - 1].AXISNum = AXIS_X;
	g_SEncoder[1 - 1].usEncoderCirclePulseNum = 10000;
	g_SEncoder[2 - 1].usEncoderCirclePulseNum = 10000;
	g_SEncoder[4 - 1].usEncoderCirclePulseNum = 10000;

	//刀具参数初始化
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		for (ucCounter1 = 0; ucCounter1 < TOOL_NUM_MAX; ucCounter1++)
		{
			g_SSystemPara.SToolPara[ucCounter][ucCounter1].ucToolID = ucCounter1;
		}
	}

	//vs设置刀具半径
#if !STM32_MODE
	g_SSystemPara.SToolPara[0][0].ucToolID = 1;
	g_SSystemPara.SToolPara[0][0].dToolRadius = 2;
	g_SSystemPara.SToolPara[0][0].ucToolNoseID = 0;

#endif


#if !STM32_MODE
	for (ucCounter = 0; ucCounter < AXIS_PHYSICS_NUM_MAX; ucCounter++)
	{
#if CotrexF4 == 1
		pAxisEnable[ucCounter] = &AxisEnable[ucCounter];
#else
		pAxisEnable(ucCounter) = &AxisEnable[ucCounter];
#endif
	}
#else
//	pAxisEnable[0] = &(PHout(4));
//	pAxisEnable[1] = &(PCout(3));
//	pAxisEnable[2] = &(PHout(2));
//	pAxisEnable[3] = &(PCout(2));
//	pAxisEnable[4] = &(PCout(0));
//	pAxisEnable[5] = &(PHout(5));
#endif // !STM32_MODE

	/******************************************************************************************************/

//实时坐标初始化    1.vs模式下清零     2.stm32模式下，企业从时钟芯片中获取上一次关机前的坐标，然后赋给double g_dRealTimePointArray[AXIS_PHYSICS_NUM_MAX]实时坐标数组
#if !STM32_MODE
	clearRealTimePoint();
#endif

	//G代码初始化
	//memset(&g_SZeroPtOfCoordSys, 0, sizeof(SZeroPointOfCoordSys));                    //初始化总偏移值
	
	struct SEncoder* pEncoderArrays[ENCODER_NUM_MAX];	//指针数组,指向编码器的指针数组	陈敏聪	2023/8/1
	for (int i = 0; i < ENCODER_NUM_MAX; i++)
	{
		pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[0][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[0][i] > ENCODER_NUM_MAX - 1) 
							? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[0][i]]);
	}
	initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], g_SSystemPara.ucPWMAxisMapArray[0] ,pEncoderArrays);		//通道初始化（模仿加载FLASH）
#if CHANNEL_NUM_MAX >1
	struct SEncoder* Channel1_pEncoderArrays[ENCODER_NUM_MAX];	//指针数组,指向编码器的指针数组	陈敏聪	2023/8/1
	for (int i = 0; i < ENCODER_NUM_MAX; i++)
	{
		Channel1_pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[1][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[1][i] > ENCODER_NUM_MAX - 1) 
									? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[1][i]]);
	}
	initChannel(&g_SChannel[1], 1, g_SSystemPara.ucAxisMapArray[1], Channel1_pEncoderArrays);		//通道初始化（模仿加载FLASH）
#endif
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
        gcodeInit(ucCounter,FALSE,FALSE);

		//刀补初始化
		toolCompensationInit(ucCounter);

		//前瞻初始化
		lookAheadInit(&g_SChannel[ucCounter]);

		//插补初始化
		interpolationInit(&g_SChannel[ucCounter].SPreProcess);

		g_SSystemPara.cSpindleClampStatus[ucCounter] = STATUS_UNCLAMPED;							//主轴刹车锁紧状态设为松开
		g_SChannel[ucCounter].SSpindle.cSpindleClampStatus = g_SSystemPara.cSpindleClampStatus[0];
		g_SChannel[ucCounter].bIsStopSpindleAndCoolantwhileM30 = ucCounter == 0 ? g_SSystemPara.bIsStopSpindleAndCoolantwhileM30 : TRUE; //2024/7/26 M30执行关主轴关冷却液 默认为0，开启功能  	企业要求：该功能仅在通道0开启，其余通道关闭该功能 2024/7/26 洪冠鹏
	}

	

  return 1;
}

/*************************************************
  函数名称：initAfterLoadSysPara()
  函数功能  从Flash中加载数据后执行的初始化
  输入参数：
  输出参数：
  返 回 值：
  备    注：
****************************************/
void initAfterLoadSysPara()
{
#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
	setHandwheelSimDistancePerTick(getHandwheelSimDistancePerTick());
#endif

	uint8 ucCounter;
	uint8 i;
	const uint8* ucMapArray;//映射

	// 恢复旧参数时，超出 0..10000 ms 的 G84 孔底停留时间按默认 500 ms 处理。
	if (g_SSystemPara.unG84DefaultDelayTime > 10000U)
	{
		g_SSystemPara.unG84DefaultDelayTime = 500U;
	}

	toolInitAfterLoad();//刀具偏移初始化

	struct SEncoder* pEncoderArrays[ENCODER_NUM_MAX];	//指针数组
	for (int i = 0; i < ENCODER_NUM_MAX; i++)
	{
		pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[0][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[0][i] > ENCODER_NUM_MAX - 1) ? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[0][i]]);
	}
	initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], g_SSystemPara.ucPWMAxisMapArray[0], pEncoderArrays);		//通道初始化（模仿加载FLASH）
#if CHANNEL_NUM_MAX >1
	struct SEncoder* Channel1_pEncoderArrays[ENCODER_NUM_MAX];	//指针数组
	for (int i = 0; i < ENCODER_NUM_MAX; i++)
	{
		Channel1_pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[1][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[1][i] > ENCODER_NUM_MAX - 1) ? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[1][i]]);
	}
	initChannel(&g_SChannel[1], 1, g_SSystemPara.ucAxisMapArray[1], Channel1_pEncoderArrays);		//通道初始化（模仿加载FLASH）

	//if (judgeCommonPhyAxisBtwChannels())//通道间存在公共轴，此时不再前瞻  2022.4.25 郭强
	//{
	//	g_bLookAheadFunctionOpenFlag = FALSE;
	//}
	//else
	//{
	//	g_bLookAheadFunctionOpenFlag = TRUE;
	//}

#endif // CHANNEL_NUM_MAX >1

	//for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	//{
	//	ucMapArray = g_SChannel[ucCounter].pMapAxisArray;
	//	for ( i = 0; i < AXIS_NUM_MAX; i++)
	//	{
	//		setAxisEnableStatus(ucMapArray[i], 1);
	//	}
	//}
	//初始化轴的状态  开机时全部为1
	//initAxisEnableStatus();
	
	initAxisRevDataAPI();//反向误差补偿固定参数初始化
#if CHANNEL_NUM_MAX >1
	g_SZeroPtOfCoordSys[1] = g_SZeroPtOfCoordSys[0];//将通道0的各种零点集合付给通道1 2024.1.18 陈育楠
#endif // CHANNEL_NUM_MAX >1
#if(MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE&&(CHANNEL_INTERACTION==1))//双主轴模式下
	for (ucCounter = 0; ucCounter < TOOL_NUM_MAX; ucCounter++)
	{
		g_SSystemPara.SToolPara[1][ucCounter] = g_SSystemPara.SToolPara[0][ucCounter];//将通道0的刀具参数赋给通道1
		g_SSystemPara.SWearPara[1][ucCounter] = g_SSystemPara.SWearPara[0][ucCounter];//将通道0的刀具磨损参数赋给通道1
	}
#endif
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机 初始化坐标转换的三角函数
		double dAngleYY1;//Y轴与Y1轴夹角 单位：°
		dAngleYY1 = (90.0 - g_SSystemPara.dAngleXY1) * (PI / 180.0);//Y轴与Y1轴夹角 单位：rad
		initInclinedYMachinePara(&g_SChannel[ucCounter], dAngleYY1);
		g_SChannel[ucCounter].bUseVirtualY = g_SSystemPara.bUseVirtualY;//读取虚拟Y轴功能开启标志位
#endif
		SG0LoadSysPara(&g_SChannel[ucCounter].SG00);//G0初始化
		SG1LoadSysPara(&g_SChannel[ucCounter].SG01);//G1初始化
		SG2LoadSysPara(&g_SChannel[ucCounter].SG02);//G2初始化
		SG32LoadSysPara(&g_SChannel[ucCounter].SG32);  //G32初始化
		interpolationInitAfterLoad(&g_SChannel[ucCounter].SRun, &g_SChannel[ucCounter].SPreProcess);//插补SRun SPrePrecess初始化
		//setSpindleEnabled(&g_SChannel[ucCounter].SSpindle, TRUE);//主轴使能
		//if (g_SChannel[ucCounter].pMapAxisArray[AXIS_C] < AXIS_PHYSICS_NUM_MAX) // 主轴不使能
		//{
		//	clrbit(g_ucAxisExactEnable, g_SChannel[ucCounter].pMapAxisArray[AXIS_C]);
		//}

	}



	//initScrewComp(0);
	//initScrewComp(2);

	
#if STM32_MODE
	Encoder_TIM1_Init(0); //编码器1初始化;
	Encoder_TIM2_Init(0); //编码器1初始化;			//陈敏聪	2023/8/2
	HandwheelEncoder_Tim4_init();  //手轮初始化
#endif

}

/********************************其他参数**************************************/

/*************************************************
  函数名称：getMapValue(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  修改映射值
  输入参数：ucChannelNum		通道号		0~CHANNEL_NUM_MAX-1
			ucAxis				轴号		0~AXIS_NUM_MAX-1
  输出参数：
  返 回 值：轴号对应的物理轴  物理轴0~AXIS_PHYSICS_NUM_MAX-1    AXIS_NULL宏定义为254
  备    注：
****************************************/
uint8 getMapValue(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	return g_SSystemPara.ucAxisMapArray[ucChannelNum][ucAxis];
}


/*************************************************
  函数名称：getPWMMapValue(uint8 ucChannelNum, uint8 ucAxis)
  函数功能  修改映射值
  输入参数：ucChannelNum		通道号		0~CHANNEL_NUM_MAX-1
			ucAxis				轴号		0~AXIS_NUM_MAX-1
  输出参数：
  返 回 值：轴号对应的物理轴  物理轴0~AXIS_PHYSICS_NUM_MAX-1    AXIS_NULL宏定义为254
  备    注：
****************************************/
uint8 getPWMMapValue(uint8 ucChannelNum, uint8 ucAxis)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	return g_SSystemPara.ucPWMAxisMapArray[ucChannelNum][ucAxis];
}

/*************************************************
  函数名称：setMapValue(uint8 ucChannelNum, uint8 ucAxis,uint8 ucPhysicalAxisNum)
  函数功能  修改映射值
  输入参数：ucChannelNum		通道号		0~CHANNEL_NUM_MAX-1
			ucAxis				轴号		0~AXIS_NUM_MAX-1
			ucPhysicalAxisNum	物理轴号	0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：1 设置完成
			-1 通道号或轴号错误
  备    注：
****************************************/
int8 setMapValue(uint8 ucChannelNum, uint8 ucAxis, uint8 ucPhysicalAxisNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	g_SSystemPara.ucAxisMapArray[ucChannelNum][ucAxis] = ucPhysicalAxisNum;
	return 1;
}


/*************************************************
  函数名称：setPWMMapValue(uint8 ucChannelNum, uint8 ucAxis,uint8 ucPhysicalAxisNum)
  函数功能  修改映射值
  输入参数：ucChannelNum		通道号		0~CHANNEL_NUM_MAX-1
			ucAxis				轴号		0~AXIS_NUM_MAX-1
			ucPhysicalAxisNum	物理轴号	0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：1 设置完成
			-1 通道号或轴号错误
  备    注：
****************************************/
int8 setPWMMapValue(uint8 ucChannelNum, uint8 ucAxis, uint8 ucPhysicalAxisNum)
{
	if (ucChannelNum >= CHANNEL_NUM_MAX || ucAxis >= AXIS_NUM_MAX)
	{
		return -1;
	}
	if(ucPhysicalAxisNum > AXIS_NUM_MAX-1 && ucPhysicalAxisNum <= 254)
	{
		return -1;
	}
	g_SSystemPara.ucPWMAxisMapArray[ucChannelNum][ucAxis] = ucPhysicalAxisNum;
	return 1;
}

/*************************************************
  函数名称：setMapValue(uint8 ucChannelNum, uint8 ucAxis,uint8 ucPhysicalAxisNum)
  函数功能  获取手轮模拟达到最大值的速度   /100ms
  输入参数：
  输出参数：
  返 回 值：  xx pulse/100ms
  备    注：
****************************************/
int getWheelSimulaSpeed()
{
	return (int)g_SSystemPara.fHandleWheelSpeedMax;
}

/*************************************************
  函数名称：setMapValue(uint8 ucChannelNum, uint8 ucAxis,uint8 ucPhysicalAxisNum)
  函数功能  设置手轮模拟达到最大值的速度   /100ms
  输入参数：nVal  输入值  pulse/100ms
  输出参数：
  返 回 值：1 完成
  备    注：
****************************************/
int8 setWheelSimulaSpeed(int nVal)
{
	if (nVal <= 0)return -1;
	g_SSystemPara.fHandleWheelSpeedMax = (float)nVal;
	return 1;
}

#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
int getHandwheelSimDistancePerTick(void)
{
	int nDistance = g_SSystemPara.nHandwheelSimDistancePerTick; /**< volatile 字段的单次读取快照。 */
	return nDistance > 0 ? nDistance : HANDWHEEL_SIM_DISTANCE_DEFAULT;
}

int8 setHandwheelSimDistancePerTick(int nVal)
{
	if (nVal <= 0)
	{
		return -1;
	}
	g_SSystemPara.nHandwheelSimDistancePerTick = nVal;
	return 1;
}
#endif

/*************************************************
  函数名称：getSimulaVmax()
  函数功能  获取手轮模拟的最大速度   mm/min
  输入参数：
  输出参数：
  返 回 值：mm/min
  备    注：
****************************************/
int getSimulaVmax()
{
	return (int)round(g_SSystemPara.dSimulaVmax * 60 * 1000);
}
/*************************************************
  函数名称：getAngleXY1(double dYMovement, double dY1Movement)
  函数功能  获取 X1轴与Y1轴间的夹角 并更新三角函数值
  输入参数：dX1Movement	 X1轴的移动量 （mm）
			dY1Movement  Y1轴的移动量（mm）
  输出参数：
  返 回 值：dAngleXY1 X1轴与Y1轴间的夹角（ °）
			-1 通道为空
  备    注：插补Y轴机
****************************************/
double getAngleXY1(double dX1Movement, double dY1Movement)
{
	//通道为空
	if (g_SChannel == NULL)return -1;

	//参数错误,返回初始值
	if (dX1Movement < EPSINON || dY1Movement < EPSINON || isZero(dY1Movement)) return 30.0;
	if (dX1Movement / dY1Movement - 1.0 > EPSINON) return 30.0;

	//虚拟Y轴与倾斜Y1轴间的夹角(rad)
	double dAngleYY1;
	dAngleYY1 = asin(dX1Movement / dY1Movement);

	//初始化插补Y轴机的三角函数
	initInclinedYMachinePara(&g_SChannel[0], dAngleYY1);
	//返回X轴与Y1轴间的夹角（°）
	return (90 - dAngleYY1 * (180.0 / PI));
}
/*************************************************
  函数名称：getSimulaG0Vmax()
  函数功能  获取手轮模拟的G0最大速度   mm/min
  输入参数：
  输出参数：
  返 回 值：mm/min
  备    注：
****************************************/
int getSimulaG0Vmax()
{
	return (int)round(g_SSystemPara.dSimulaG0Vmax * 60 * 1000);
}

/*************************************************
  函数名称：setSimulaVmax()
  函数功能  设置手轮模拟的最大速度   mm/min
  输入参数：nVal	输入进给速度   -- mm/min
  输出参数：
  返 回 值：1  成功
  备    注：
****************************************/
int8 setSimulaVmax(int nVal)
{
	g_SSystemPara.dSimulaVmax = nVal / 60.0 / 1000.0;
	return 1;
}
/*************************************************
  函数名称：setSimulaG0Vmax()
  函数功能  设置手轮模拟的G0最大速度   mm/min
  输入参数：nVal	输入进给速度   -- mm/min
  输出参数：
  返 回 值：1  成功
  备    注：
****************************************/
int8 setSimulaG0Vmax(int nVal)
{
	g_SSystemPara.dSimulaG0Vmax = nVal / 60.0 / 1000.0;
	return 1;
}

//清除系统总加工时间
int8 clearMachineTimeTotal()
{
	g_sdtime.ucProcessTimeTotal = 0;
	return 1;
}

/********************************合成参数**************************************/


/*************************************************
  函数名称：getCommonVel(uint8 ucVelType)
  函数功能  获取合成速度
  输入参数：ucVelType	速度类型  VEL_VS:初速度  VEL_VMAX:最大速度  VEL_VE:末速度
  输出参数：
  返 回 值：对应速度  mm/min
  备    注：
****************************************/
int getCommonVel(uint8 ucVelType)
{
	double dRes = 0.0;
	switch (ucVelType)
	{
	case VEL_VS:
	case VEL_VE:
		dRes = g_SSystemPara.dSysMinVel;
		break;
	case VEL_VMAX:
		dRes = g_SSystemPara.dSysMaxVel;
		break;
	default:
		break;
	}
	return round(PerMsToPerMin(dRes));//mm/ms转mm/min
}


/*************************************************
  函数名称：setCommonVel(int nVal, uint8 ucVelType)
  函数功能  设置合成速度
  输入参数：nVal		输入值(mm/min)
			ucVelType	速度类型  VEL_VS:初速度  VEL_VMAX:最大速度  VEL_VE:末速度
  输出参数：
  返 回 值：1  成功
			-1 错误
  备    注：
****************************************/
int8 setCommonVel(int nVal, uint8 ucVelType)
{
	double dVal = PerMinToPerMs(nVal); //mm/min转mm/ms
	switch (ucVelType)
	{
	case VEL_VS:
	case VEL_VE:
		g_SSystemPara.dSysMinVel = dVal;
		break;
	case VEL_VMAX:
		g_SSystemPara.dSysMaxVel = dVal;
		break;
	default:
		return -1;
	}
	return 1;
}



/*************************************************
  函数名称：setCommonVel(int nVal, uint8 ucVelType)
  函数功能  获取合成加减速类型
  输入参数：ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
  输出参数：
  返 回 值：加速/减速类型  T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
  备    注：
****************************************/
int getCommonPlanType(uint8 ucAccType)
{
	int nRes = 0;
	switch (ucAccType)
	{
	case PLAN_ACC:
		nRes = (int)g_SSystemPara.ucCommonAccVelPlanType;
		break;
	case PLAN_DEC:
		nRes = (int)g_SSystemPara.ucCommonDecVelPlanType;
		break;
	default:
		break;
	}
	return nRes;
}

/*************************************************
  函数名称：setCommonVel(int nVal, uint8 ucVelType)
  函数功能  设置合成加减速类型
  输入参数：nVal		输入值 T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
			ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
  输出参数：
  返 回 值：1  成功
			-1 错误
  备    注：
****************************************/
int8 setCommonPlanType(int nVal, uint8 ucAccType)
{
	//增加有效性校验：规划类型应为 1(T 型)、3(S3 型)、4(S4 型)  2026.3.4
    if (nVal != VEL_PLAN_T_CURVE && 
        nVal != VEL_PLAN_S3_CURVE && 
        nVal != VEL_PLAN_S4_CURVE)
    {
        return -1;  // 非法值，拒绝写入
    }
	switch (ucAccType)
	{
	case PLAN_ACC:
		g_SSystemPara.ucCommonAccVelPlanType = (uint8)nVal;
		break;
	case PLAN_DEC:
		g_SSystemPara.ucCommonDecVelPlanType = (uint8)nVal;
		break;
	default:
		return -1;
	}
	return 1;
}


/*************************************************
  函数名称：getCommonAcc(uint8 ucAccType, uint8 ucPlanType)
  函数功能  获取加/减速度
  输入参数：ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
			ucPlanType	规划类型  T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
  输出参数：
  返 回 值：加/减速度 mm/s2
  备    注：
****************************************/
int getCommonAcc(uint8 ucAccType, uint8 ucPlanType)
{
	double dRes = 0.0;
	switch (ucAccType)
	{
	case PLAN_ACC: // 加速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE:  //T型
			dRes = g_SSystemPara.dSysPlanTMaxAcc;
			break;
		case VEL_PLAN_S3_CURVE:  //S3型
			dRes = g_SSystemPara.dSysPlanS3MaxAcc;
			break;
		case VEL_PLAN_S4_CURVE:   //S4型
			dRes = g_SSystemPara.dSysPlanS4MaxAcc;
			break;
		default:
			break;
		}
		break;
	case PLAN_DEC://减速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE://T型
			dRes = g_SSystemPara.dSysPlanTMaxDec;
			break;
		case VEL_PLAN_S3_CURVE://S3型
			dRes = g_SSystemPara.dSysPlanS3MaxDec;
			break;
		case VEL_PLAN_S4_CURVE://S4型
			dRes = g_SSystemPara.dSysPlanS4MaxDec;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return round(PerMs2ToPerSec2(dRes));// mm/ms2 -> mm/s2
}

/*************************************************
  函数名称：setCommonAcc(int nVal, uint8 ucAccType, uint8 ucPlanType)
  函数功能  获取加/减速度
  输入参数：nVal		输入值 (mm/s2)
			ucAccType	加速/减速类型 加速:PLAN_ACC		减速:PLAN_DEC
			ucPlanType	规划类型	  T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setCommonAcc(int nVal, uint8 ucAccType, uint8 ucPlanType)
{
	double dVal = 0.0;
	
	if (nVal <= 0)// 新增：加速度必须大于 0，避免后续规划中除零错误 2026.3.3
	{
		return -1;
	}
	dVal = PerSec2ToPerMs2(nVal);
	switch (ucAccType)
	{
	case PLAN_ACC://加速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE://T型
			g_SSystemPara.dSysPlanTMaxAcc = dVal;
			break;
		case VEL_PLAN_S3_CURVE://S3型
			g_SSystemPara.dSysPlanS3MaxAcc = dVal;
			break;
		case VEL_PLAN_S4_CURVE://S4型
			g_SSystemPara.dSysPlanS4MaxAcc = dVal;
			break;
		default:
			return -1;
		}
		break;
	case PLAN_DEC://减速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE://T型
			g_SSystemPara.dSysPlanTMaxDec = dVal;
			break;
		case VEL_PLAN_S3_CURVE://S3型
			g_SSystemPara.dSysPlanS3MaxDec = dVal;
			break;
		case VEL_PLAN_S4_CURVE://S4型
			g_SSystemPara.dSysPlanS4MaxDec = dVal;
			break;
		default:
			return -1;
		}
		break;
	default:
		return -1;
	}
	return 1;
}

/********************************单轴参数**************************************/

/*************************************************
  函数名称：getPhysicalAxisType(uint8 ucPhysicalAxis)
  函数功能  获取轴类型
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：0  直线轴  1旋转轴
  备    注：
****************************************/
int getPhysicalAxisType(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	return g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType;
}

/*************************************************
  函数名称：setPhysicalAxisType(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置轴类型
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal				输入值 0  直线轴  1旋转轴
  输出参数：
  返 回 值：1  成功
			-1 错误
  备    注：
****************************************/
int8 setPhysicalAxisType(uint8 ucPhysicalAxis, int nVal)
{
	uint8 ucCounter;
	// 1. 校验物理轴号合法性
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	
	// 2. 新增：强制校验nVal只能为0或1，非法值返回错误
	if (nVal != LINEAR_AXIS && nVal != ROTATION_AXIS)
	{
		return -1;
	}

	//zxr: 切换轴类型前校验基础脉冲数，避免除零或部分修改轴参数
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse == 0)
	{
		return -1;
	}

	if (nVal == ROTATION_AXIS)//旋转轴
	{
		if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType != ROTATION_AXIS)
		{
			g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle = 360.0;
			//zxr: 旋转轴当量由每转位移除以每转脉冲数计算，不使用旧派生当量
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm = 1.0 * g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle / g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse;
			g_SSystemPara.SAxisPara[ucPhysicalAxis].bShowByDiameter = FALSE;
		}

	}
	else
	{
		if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
		{
			//zxr: 切回直线轴时按每毫米脉冲数重建当量，避免往返切换累积错误
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm = 1.0 / g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse;
		}
	}
	g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType = (uint8)nVal;
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)//轴参数初始化
	{
		SG0LoadSysPara(&g_SChannel[ucCounter].SG00);//G0初始化
		SG1LoadSysPara(&g_SChannel[ucCounter].SG01);//G1初始化
		SG2LoadSysPara(&g_SChannel[ucCounter].SG02);//G2初始化
		SG32LoadSysPara(&g_SChannel[ucCounter].SG32);  //G32初始化
	}
	return 1;
}

/*************************************************
  函数名称：getShowByDiameter(uint8 ucPhysicalAxis)
  函数功能  获取轴是否直径显示
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：1  TRUE
			0  FALSE
  备    注：
****************************************/
int getShowByDiameter(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	return (int)g_SSystemPara.SAxisPara[ucPhysicalAxis].bShowByDiameter;
}

/*************************************************
  函数名称：setShowByDiameter(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置轴以直径显示
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal				输入值 1 TRUE  0 FALSE
  输出参数：
  返 回 值：1  成功
			-1 -2 错误
  备    注：
****************************************/
int8 setShowByDiameter(uint8 ucPhysicalAxis, int nVal)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (nVal != (int)TRUE && nVal != (int)FALSE)//输入的值有误
	{
		return -2;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == LINEAR_AXIS)//直线轴才能以直径显示
	{
		g_SSystemPara.SAxisPara[ucPhysicalAxis].bShowByDiameter = (bool)nVal;
	}
	return 1;
}


/*************************************************
  函数名称：getDistPerCycle(uint8 ucPhysicalAxis)
  函数功能  获取一转的位移
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：返回一转的位移
  备    注：
****************************************/
int getDistPerCycle(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == LINEAR_AXIS)
	{
		return 0;
	}
	return g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle;
}

/*************************************************
  函数名称：setDistPerCycle(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置一转的位移
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal				输入值
  输出参数：
  返 回 值：1  成功
			-1 错误
  备    注：
****************************************/
int8 setDistPerCycle(uint8 ucPhysicalAxis, int nVal)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if(nVal <= 0)
	{
		parseRunningError(9999,GCODE_ERROR_NUM153,9999);
		return -1;
	}
	
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == LINEAR_AXIS)
	{
		return 0;
	}
	//zxr: 每转脉冲数为零时拒绝修改，避免位移参数更新后当量刷新失败
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse == 0)
	{
		return -1;
	}
	g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle = (float)nVal;
	//zxr: 复用脉冲数设置接口，立即重算当量并刷新各通道运动参数
	return setPulsePerMMOrCycle(ucPhysicalAxis, g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse);
}


/*************************************************
  函数名称：getPulsePerMMOrCycle(uint8 ucPhysicalAxis)
  函数功能  获取 mm对应脉冲（或转对应脉冲）设置一转的位移
  输入参数：ucPhysicalAxis		物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：mm对应脉冲（或转对应脉冲）  pulse
  备    注：
****************************************/
int getPulsePerMMOrCycle(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	return (int)g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse;
}

/*************************************************
  函数名称：setPulsePerMMOrCycle(uint8 ucPhysicalAxis, int nVal)
  函数功能  获取 mm对应脉冲（或转对应脉冲）设置一转的位移
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal			输入值  1mm对应脉冲（或1转对应脉冲）
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setPulsePerMMOrCycle(uint8 ucPhysicalAxis, int nVal)
{
	uint8 ucCounter;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}

	if (nVal < 1 ||  nVal > 65535)
	{
		return -1;
	} 
	g_SSystemPara.SAxisPara[ucPhysicalAxis].usMmToPulse = (uint16)nVal;
	g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS ?
		1.0 / nVal * g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle : 1.0 / nVal;
	for (ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
	{
		SG0LoadSysPara(&g_SChannel[ucCounter].SG00);//G0初始化
		SG1LoadSysPara(&g_SChannel[ucCounter].SG01);//G1初始化
		SG2LoadSysPara(&g_SChannel[ucCounter].SG02);//G2初始化
		SG32LoadSysPara(&g_SChannel[ucCounter].SG32);  //G32初始化
	}
	return 1;
}


/*************************************************
  函数名称：getAxisVel(uint8 ucPhysicalAxis, uint8 ucVelType)
  函数功能  获取单轴速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			ucVelType	速度类型  VEL_VS:初速度  VEL_VMAX:最大速度  VEL_VE:末速度
  输出参数：
  返 回 值：对应速度  mm/min 或r/min
  备    注：
****************************************/
int getAxisVel(uint8 ucPhysicalAxis, uint8 ucVelType)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	switch (ucVelType)
	{
	case VEL_VS://初速度
		dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVs;
		break;
	case VEL_VMAX://最大速度
		dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVmax;
		break;
	case VEL_VE://末速度
		dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe;
		break;
	default:
		break;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = AngleToR(dRes, ucPhysicalAxis);// °->R
	}
	return round(PerMsToPerMin(dRes));//mm/ms ->mm/min
}

/*************************************************
  函数名称：setAxisVel(uint8 ucPhysicalAxis, int nVal, uint8 ucVelType)
  函数功能  设置单轴速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值    速度mm/min  或r/min
			ucVelType	速度类型  VEL_VS:初速度  VEL_VMAX:最大速度  VEL_VE:末速度
  输出参数：
  返 回 值：对应速度  mm/min
  备    注：
****************************************/
int8 setAxisVel(uint8 ucPhysicalAxis, int nVal, uint8 ucVelType)
{
	double dVal =0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	dVal =  PerMinToPerMs(nVal);//mm/min ->mm/ms
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dVal = RToAngle(dVal, ucPhysicalAxis);  //R ->°
	}
	switch (ucVelType)
	{
	case VEL_VS://初速度
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVs = dVal;
		break;
	case VEL_VMAX://最大速度
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVmax = dVal;
		break;
	case VEL_VE://末速度
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dVe = dVal;
		break;
	default:
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：getAxisVel(uint8 ucPhysicalAxis, uint8 ucVelType)
  函数功能  设置主轴速度
  输入参数：
			ucVelType	速度类型  VEL_VS:初速度   VEL_VE:末速度
  输出参数：
  返 回 值：对应速度  mm/min 或r/min
  备    注：
****************************************/
int getSpindleAxisVel(uint8 ucVelType)
{
	double dRes = 0.0;
	switch (ucVelType)
	{
	case VEL_VS://初速度
		dRes = g_SSystemPara.dSpindleVs;
		break;
	case VEL_VE://末速度
		dRes = g_SSystemPara.dSpindleVe;
		break;
	default:
		break;
	}
	dRes = dRes /DEGREE_OF_CIRCLE;// °->R
	return round(PerMsToPerMin(dRes));//mm/ms ->mm/min
}



/*************************************************
  函数名称：setSpindleAxisVel(int nVal, uint8 ucVelType)
  函数功能  设置主轴速度
  输入参数： nVal		输入值    速度mm/min  或r/min
			ucVelType	速度类型  VEL_VS:起跳初始速度  VEL_VE:起跳末速度
  输出参数：
  返 回 值：对应错误码
  备    注：
****************************************/
int8 setSpindleAxisVel(int nVal, uint8 ucVelType)
{
	double dVal =0.0;
	dVal =  PerMinToPerMs(nVal);//mm/min ->mm/ms
	dVal = dVal * DEGREE_OF_CIRCLE;  //R ->°

	switch (ucVelType)
	{
	case VEL_VS://初速度
		g_SSystemPara.dSpindleVs = dVal;
		break;
	case VEL_VE://末速度
		g_SSystemPara.dSpindleVe = dVal;
		break;
	default:
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：getAxisPlanType(uint8 ucPhysicalAxis, uint8 ucAccType)
  函数功能  获取单轴加减速类型
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
  输出参数：
  返 回 值：加速/减速类型  T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
  备    注：
****************************************/
int getAxisPlanType(uint8 ucPhysicalAxis, uint8 ucAccType)
{
	uint8 ucRes = 0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	switch (ucAccType)
	{
	case PLAN_ACC://加速
		ucRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAccVelPlanType;
		break;
	case PLAN_DEC://减速
		ucRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType;
		break;
	default:
		break;
	}
	return (int)ucRes;
}

/*************************************************
  函数名称：setAxisPlanType(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType)
  函数功能  设置单轴加减速类型
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值 T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
			ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setAxisPlanType(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType)
{
	uint8 ucRes = (uint8)nVal;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	//增加有效性校验：规划类型应为 1(T 型)、3(S3 型)、4(S4 型)  2026.3.4
    if (nVal != VEL_PLAN_T_CURVE && 
        nVal != VEL_PLAN_S3_CURVE && 
        nVal != VEL_PLAN_S4_CURVE)
    {
        return -1;  // 非法值，拒绝写入
    }
	switch (ucAccType)
	{
	case PLAN_ACC://加速
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAccVelPlanType = ucRes;
		break;
	case PLAN_DEC://减速
		g_SSystemPara.SAxisPara[ucPhysicalAxis].ucDecVelPlanType = ucRes;
		break;
	default:
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：getAxisAcc(uint8 ucPhysicalAxis, uint8 ucAccType, uint8 ucPlanType)
  函数功能  获取单轴加/减速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
			ucPlanType	规划类型  T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
  输出参数：
  返 回 值：加/减速度 mm/s2  或r/s2
  备    注：
****************************************/
int getAxisAcc(uint8 ucPhysicalAxis, uint8 ucAccType, uint8 ucPlanType)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	switch (ucAccType)
	{
	case PLAN_ACC: // 加速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE:  //T型
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxAcc;
			break;
		case VEL_PLAN_S3_CURVE:  //S3型
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxAcc;
			break;
		case VEL_PLAN_S4_CURVE:   //S4型
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxAcc;
			break;
		default:
			break;
		}
		break;
	case PLAN_DEC://减速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE://T型
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxDec;
			break;
		case VEL_PLAN_S3_CURVE://S3型
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxDec;
			break;
		case VEL_PLAN_S4_CURVE://S4型
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxDec;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = AngleToR(dRes, ucPhysicalAxis); // °->R
	}
	return round(PerMs2ToPerSec2(dRes));//mm/ms2 ->mm/s2
}


/*************************************************
  函数名称：setAxisAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType, uint8 ucPlanType)
  函数功能  设置单轴加/减速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值  mm/s2 或 r/s2
			ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
			ucPlanType	规划类型  T形:VEL_PLAN_T_CURVE  3次S型:VEL_PLAN_S3_CURVE  4次S型:VEL_PLAN_S4_CURVE
  输出参数：
  返 回 值：1 完成
			-1 错误
  备    注：
****************************************/
int8 setAxisAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType, uint8 ucPlanType)
{
	double dVal = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (nVal <= 0)// 新增：加速度必须大于 0，避免后续规划中除零错误 2026.3.3
	{
		return -1;
	}
	dVal = PerSec2ToPerMs2(nVal);//mm/s2 ->mm/ms2
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dVal = RToAngle(dVal, ucPhysicalAxis); // R->°
	}
	switch (ucAccType)
	{
	case PLAN_ACC://加速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE://T型
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxAcc = dVal;
			break;
		case VEL_PLAN_S3_CURVE://S3型
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxAcc = dVal;
			break;
		case VEL_PLAN_S4_CURVE://S4型
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxAcc = dVal;
			break;
		default:
			return -1;
		}
		break;
	case PLAN_DEC://减速段
		switch (ucPlanType)
		{
		case VEL_PLAN_T_CURVE://T型
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanTMaxDec = dVal;
			break;
		case VEL_PLAN_S3_CURVE://S3型
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS3MaxDec = dVal;
			break;
		case VEL_PLAN_S4_CURVE://S4型
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dSysPlanS4MaxDec = dVal;
			break;
		default:
			return -1;
		}
		break;
	default:
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：getSpindleAxisAcc(uint8 ucPhysicalAxis)
  函数功能  获取主轴运动加/减速度
  输入参数： ucPhysicalAxis	旋转物理轴号
  输出参数： 
  返 回 值：加速度值
			-1 错误
  备    注：用于主轴运动，区分分度运动
****************************************/
int getSpindleAxisAcc(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	double dVal = g_SSystemPara.dSysPlanSpindleAcc;
	dVal = AngleToR(dVal, ucPhysicalAxis); // °->R
	return round(PerMs2ToPerSec2(dVal));//mm/ms2 ->mm/s2
}

/*************************************************
  函数名称：setSpindleAxisAcc(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置主轴运动加/减速度
  输入参数： ucPhysicalAxis	旋转物理轴号
			nVal		输入值 r/s2
  输出参数：
  返 回 值：1 完成
			-1 错误
  备    注：用于主轴运动，区分分度运动
****************************************/
int8 setSpindleAxisAcc(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = 0.0;	
	if (nVal <= 0 || ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)//校验
	{
		return -1;
	}
	dVal = PerSec2ToPerMs2(nVal); // m/s? -> mm/ms?
	dVal = RToAngle(dVal, ucPhysicalAxis);  //R ->°
	g_SSystemPara.dSysPlanSpindleAcc = dVal;
}

/*************************************************
  函数名称：getSpindleAxisDec(uint8 ucPhysicalAxis)
  函数功能  获取主轴运动减速度
  输入参数： ucPhysicalAxis	旋转物理轴号
  输出参数： 
  返 回 值：加速度值
			-1 错误
  备    注：用于主轴运动，区分分度运动
****************************************/
int getSpindleAxisDec(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	double dVal = g_SSystemPara.dSysPlanSpindleDec;
	dVal = AngleToR(dVal, ucPhysicalAxis); // °->R
	return round(PerMs2ToPerSec2(dVal));//mm/ms2 ->mm/s2
}

/*************************************************
  函数名称：setSpindleAxisDec(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置主轴运动减速度
  输入参数： ucPhysicalAxis	旋转物理轴号
			nVal		输入值 r/s2
  输出参数：
  返 回 值：1 完成
			-1 错误
  备    注：用于主轴运动，区分分度运动
****************************************/
int8 setSpindleAxisDec(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = 0.0;	
	if (nVal <= 0 || ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)//校验
	{
		return -1;
	}
	dVal = PerSec2ToPerMs2(nVal); // m/s? -> mm/ms?
	dVal = RToAngle(dVal, ucPhysicalAxis);  //R ->°
	g_SSystemPara.dSysPlanSpindleDec = dVal;
}


/*************************************************
  函数名称：getManualSpeed(uint8 ucPhysicalAxis, uint8 ucManualType)
  函数功能  获取手动速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			ucManualType	手动类型  快速手动：MANUAL_QUICK   慢速手动：MANUAL_SLOW
  输出参数：
  返 回 值：对应速度  mm/min  或 r/min
  备    注：
****************************************/
int getManualSpeed(uint8 ucPhysicalAxis, uint8 ucManualType)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	switch (ucManualType)//手动类型
	{
	case MANUAL_QUICK://快速手动
		dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickVel;
		break;
	case MANUAL_SLOW://慢速手动
		dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowVel;
		break;
	default:
		break;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = AngleToR(dRes, ucPhysicalAxis); // °->R
	}
	return  round(PerMsToPerMin(dRes));//mm/ms->mm/min
}

/*************************************************
  函数名称：setManualSpeed(uint8 ucPhysicalAxis, int nVal, uint8 ucManualType)
  函数功能  设置手动速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal			输入值  mm/min   或r/min
			ucManualType	手动类型  快速手动：MANUAL_QUICK   慢速手动：MANUAL_SLOW
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setManualSpeed(uint8 ucPhysicalAxis, int nVal, uint8 ucManualType)
{
	double dVal = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	dVal = PerMinToPerMs(nVal);   // mm /min  ->  mm/ms
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dVal = RToAngle(dVal, ucPhysicalAxis);// R->°
	}
	switch (ucManualType)//手动类型
	{
	case MANUAL_QUICK://快速手动
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickVel = dVal;
		break;
	case MANUAL_SLOW://慢速手动
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowVel = dVal;
		break;
	default:
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：getManualAcc(uint8 ucPhysicalAxis, uint8 ucAccType, uint8 ucManualType)
  函数功能  获取手动加/减速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			ucAccType		加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
			ucManualType	手动类型  快速手动：MANUAL_QUICK   慢速手动：MANUAL_SLOW
  输出参数：
  返 回 值：加/减速度 mm/s2
  备    注：
****************************************/
int getManualAcc(uint8 ucPhysicalAxis, uint8 ucAccType, uint8 ucManualType)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	switch (ucAccType)
	{
	case PLAN_ACC:
		if (ucManualType == MANUAL_QUICK)
		{
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickAcc;
		}
		else
		{
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowAcc;
		}
		break;
	case PLAN_DEC:
		if (ucManualType == MANUAL_QUICK)
		{
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickDec;
		}
		else
		{
			dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowDec;
		}
		break;
	default:
		break;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = AngleToR(dRes,ucPhysicalAxis); //  °->R
	}
	return round(PerMs2ToPerSec2(dRes));//mm/ms2 ->mm/s2
}

/*************************************************
  函数名称：setManualAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType, uint8 ucManualType)
  函数功能  设置手动加/减速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal			输入值 mm/s2
			ucAccType		加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
			ucManualType	手动类型  快速手动：MANUAL_QUICK   慢速手动：MANUAL_SLOW
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setManualAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType, uint8 ucManualType)
{
	double dVal = PerSec2ToPerMs2(nVal);//mm/s2->mm/ms2
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dVal = RToAngle(dVal, ucPhysicalAxis); //  R->°
	}
	switch (ucAccType)
	{
	case PLAN_ACC:
		if (ucManualType == MANUAL_QUICK)
		{
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickAcc = dVal;
		}
		else
		{
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowAcc = dVal;
		}
		break;
	case PLAN_DEC:
		if (ucManualType == MANUAL_QUICK)
		{
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualQuickDec = dVal;
		}
		else
		{
			g_SSystemPara.SAxisPara[ucPhysicalAxis].dManualSlowDec = dVal;
		}
		break;
	default:
		return -1;
	}
	return 1;
}

/*************************************************
  函数名称：getAxisVeerDeltaVMax(uint8 ucPhysicalAxis)
  函数功能  获取单轴转向允许变化的最大速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：单轴转向允许变化的最大速度 mm/min
  备    注：
****************************************/
int getAxisVeerDeltaVMax(uint8 ucPhysicalAxis)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].fDistPerCircle <= 0)
	{
		parseRunningError(9999,GCODE_ERROR_NUM153,9999);
		return -1;
	}
	
	dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerDeltaVMax;//读取

	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = AngleToR(dRes, ucPhysicalAxis); //° -> R
	}

	return (int)round(PerMsToPerMin(dRes));
}

/*************************************************
  函数名称：setAxisVeerDeltaVMax(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置单轴转向允许变化的最大速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal	输入值 mm/min   或 r/min
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setAxisVeerDeltaVMax(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	} 
	dVal = PerMinToPerMs(nVal);
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dVal = RToAngle(dVal, ucPhysicalAxis);  // R->°
	}
	g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerDeltaVMax = dVal;
	return 1;
}

/*************************************************
  函数名称：getAxisClearance(uint8 ucPhysicalAxis)
  函数功能  获取单轴间隙值
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：单轴间隙值  mm
  备    注：
****************************************/
int getAxisClearance(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	return round(g_SSystemPara.SAxisPara[ucPhysicalAxis].dClearance);
}


/*************************************************
  函数名称：setAxisClearance(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置单轴间隙值
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值 mm
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setAxisClearance(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = (double)nVal;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	g_SSystemPara.SAxisPara[ucPhysicalAxis].dClearance = dVal;
	return 1;
}

/*************************************************
  函数名称：getAxisRevClearanceCompDist(uint8 ucPhysicalAxis)
  函数功能  获取单轴反向间隙值 um
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：单轴反向间隙值 um   旋转轴
  备    注：
****************************************/
int getAxisRevClearanceCompDist(uint8 ucPhysicalAxis)
{
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	return round(MmToUm(g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist));//mm
}

/*************************************************
  函数名称：setAxisRevClearanceCompDist(uint8 ucPhysicalAxis, int nVal)
  函数功能  设置单轴反向间隙值 um
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值 um
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setAxisRevClearanceCompDist(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = UmToMm(nVal);//um转化为mm
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	/*修改2026.3.2旋转轴在内部在进行处理旋转轴  单位由mm转为 脉冲 */
	// if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	// {
	// 	g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist = nVal; //旋转轴  单位为 脉冲
	// }
	// else
	// {
	// 	g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist = dVal;
	// }
	g_SSystemPara.SAxisPara[ucPhysicalAxis].dRevClearanceCompDist = dVal;
	initRevCompAfterDistEditedAPI(ucPhysicalAxis);//距离参数编辑后  初始化单轴反向间隙补偿接口函数
	return 1;
}



/*************************************************
  函数名称：getAxisRevClearanceCompAcc(uint8 ucPhysicalAxis, uint8 ucAccType)
  函数功能  获取单轴反向间隙补偿加速度
  输入参数：ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
  输出参数：
  返 回 值：速度  mm/s2
  备    注：
****************************************/
int getAxisRevClearanceCompAcc(uint8 ucAccType)
{
	double dRes = 0.0;
	switch (ucAccType)
	{
	case PLAN_ACC://加速
		dRes = g_SSystemPara.dRevClearanceCompAcc;
		break;
	case PLAN_DEC://减速
		dRes=g_SSystemPara.dRevClearanceCompDec;
		break;
	default:
		return -1;
	}
	return round(PerMs2ToPerSec2(dRes));//mm/ms2 ->mm/s2
}

/*************************************************
  函数名称：setAxisRevClearanceCompAcc(uint8 ucPhysicalAxis, int nVal, uint8 ucAccType)
  函数功能  设置单轴反向间隙补偿加速度
  输入参数：nVal		输入值  mm/s2
			ucAccType	加速/减速类型  PLAN_ACC:加速   PLAN_DEC:减速
  输出参数：
  返 回 值：1 完成
			-1 错误
  备    注：
****************************************/
int8 setAxisRevClearanceCompAcc(int nVal, uint8 ucAccType)
{
	double dVal = 0.0;
	dVal = PerSec2ToPerMs2(nVal);//mm/s2 ->mm/ms2
	switch (ucAccType)
	{
	case PLAN_ACC://加速
		g_SSystemPara.dRevClearanceCompAcc= dVal;
		break;
	case PLAN_DEC://减速
		g_SSystemPara.dRevClearanceCompDec = dVal;
		break;
	default:
		return -1;
	}
	initSingleAxisRevLoopAPI();
	return 1;
}


/*************************************************
  函数名称：getAxisRevClearanceCompVel(uint8 ucPhysicalAxis, uint8 ucVelType)
  函数功能  获取单轴反向间隙补偿速度
  输入参数：ucVelType	速度类型  VEL_VS:初速度  VEL_VMAX:最大速度  VEL_VE:末速度
  输出参数：
  返 回 值：速度mm/min
  备    注：反向间隙补偿初速度和末速度相等
****************************************/
int getAxisRevClearanceCompVel(uint8 ucVelType)
{
	double dRes = 0.0;
	switch (ucVelType)
	{
	case VEL_VS://初速度
	case VEL_VE://末速度
		dRes = g_SSystemPara.dRevClearanceCompVs;
		break;
	case VEL_VMAX://最大速度
		dRes = g_SSystemPara.dRevClearanceCompVmax;
		break;
	default:
		break;
	}
	return round(PerMsToPerMin(dRes));//mm/ms ->mm/min
}


/*************************************************
  函数名称：setAxisRevClearanceCompVel(uint8 ucPhysicalAxis, int nVal, uint8 ucVelType)
  函数功能  设置单轴反向间隙补偿速度
  输入参数：nVal		输入值    速度mm/min
			ucVelType	速度类型  VEL_VS:初速度  VEL_VMAX:最大速度  VEL_VE:末速度
  输出参数：
  返 回 值： -1 轴号错误
			  1 成功
  备    注：反向间隙补偿初速度和末速度相等
****************************************/
int8 setAxisRevClearanceCompVel(int nVal, uint8 ucVelType)
{
	double dVal = 0.0;
	dVal = PerMinToPerMs(nVal);//mm/min ->mm/ms

	switch (ucVelType)
	{
	case VEL_VS://初速度
	case VEL_VE://末速度
		g_SSystemPara.dRevClearanceCompVs= dVal;
		break;
	case VEL_VMAX://最大速度
		g_SSystemPara.dRevClearanceCompVmax = dVal;
		break;
	default:
		return -1;
	}
	initSingleAxisRevLoopAPI();
	return 1;
}



/*************************************************
  函数名称：getVeerAMax(uint8 ucPhysicalAxis)
  函数功能  单轴段间非反向时过度最大加速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值：单轴段间非反向时过度最大加速度mm/s2
  备    注：
****************************************/
int getVeerAMax(uint8 ucPhysicalAxis)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}

	dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerAMax;//读取
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = AngleToR(dRes, ucPhysicalAxis); //  °->R
	}
	return round(PerMs2ToPerSec2(dRes));//mm/ms2 ->mm/s2
}


/*************************************************
  函数名称：setVeerAMax(uint8 ucPhysicalAxis, int nVal)
  函数功能  单轴段间非反向时过度最大加速度
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值 mm/s2
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setVeerAMax(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = PerSec2ToPerMs2(nVal);//mm/s2->mm/ms2
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dVal = RToAngle(dVal, ucPhysicalAxis); //  R->°
	}

	g_SSystemPara.SAxisPara[ucPhysicalAxis].dVeerAMax= dVal;
	return 1;
}


/*************************************************
  函数名称：getCoefOfAngleToLength(uint8 ucPhysicalAxis)
  函数功能  旋转轴角度到长度转换系数
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数：
  返 回 值: 系数（扩大100倍显示）
  备    注：
****************************************/
int getCoefOfAngleToLength(uint8 ucPhysicalAxis)
{
	double dRes = 0.0;
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}

	dRes = g_SSystemPara.SAxisPara[ucPhysicalAxis].dCoefOfAngleToLength;//读取
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		dRes = dRes * 100; //  扩大一百倍
	}
	else
	{
		dRes = 0;
	}
	return round(dRes);
}


/*************************************************
  函数名称： setCoefOfAngleToLength(uint8 ucPhysicalAxis, int nVal)
  函数功能  旋转轴角度到长度转换系数
  输入参数：ucPhysicalAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			nVal		输入值
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/
int8 setCoefOfAngleToLength(uint8 ucPhysicalAxis, int nVal)
{
	double dVal = nVal / 100.0;//缩小100
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return -1;
	}
	if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
	{
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dCoefOfAngleToLength = dVal;
	}
	else
	{
		g_SSystemPara.SAxisPara[ucPhysicalAxis].dCoefOfAngleToLength = 0;
	}
	return 1;
}

#if(TURRET_AXIS_TEST == 1)
/*************************************************
  函数名称：int8 setChannelToTurretMapArray(uint8 ucChannelNum,uint8 ucTurretNum,uint8 ucTargetTurretNum)
  函数功能  设置通道与刀塔的映射
  输入参数：uint8 ucChannelNum 通道号
			uint8 ucTurretNum  通道存放其支配的刀塔号的位置索引
			uint8 ucTargetTurretNum 目标刀塔号
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：如 设置的通道数 CHANNEL_NUM_MAX==2
			   设置的刀塔数  TURRET_MAX_NUM==2
			   则通道0可支配的最大刀塔数为2，如果要设置通道0支配刀塔1和通道塔2
			   则设置如下 g_SSystemPara.ucChannelToTurretmap[0][0] =1;
						   g_SSystemPara.ucChannelToTurretmap[0][1] =2;
****************************************/
int8 setChannelToTurretMapArray(uint8 ucChannelNum, uint8 ucTurretNum, uint8 ucTargetTurretNum)
{
	//通道号大于通道数，返回
	if (ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	//通道存放其支配的刀塔号的位置索引大于等于刀塔数，返回
	if (ucTurretNum >= TURRET_MAX_NUM)
	{
		return -1;
	}

	//通道与刀塔的映射
	g_SSystemPara.ucChannelToTurretmap[ucChannelNum][ucTurretNum] = ucTargetTurretNum;
	return 1;
}




/*************************************************
  函数名称：int8 setTurretToAxisMap(uint8 ucTurretNum, uint8 ucTurretSign)
  函数功能  设置刀塔与结构体轴的映射
  输入参数	uint8 ucTurretNum  刀塔号
			uint8 ucTurretSign 结构体轴
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：如 设置的刀塔数  TURRET_MAX_NUM==2
			   如果要设置刀塔1映射XZ和通道塔2映射Y
			   则设置如下 g_SSystemPara.ucChannelToTurretmap[0] =5;//0000 0101
						   g_SSystemPara.ucChannelToTurretmap[1] =2;//0000 0010
			   注意：设置的结构体轴要和刀塔映射的通道对应的结构体轴一致，否则回出错
****************************************/
int8 setTurretToAxisMap(uint8 ucTurretNum, uint8 ucTurretSign)
{
	//通道可支配的刀塔大于最大刀塔数，返回
	if (ucTurretNum > TURRET_MAX_NUM)
	{
		return -1;
	}
	//索引
	uint8 ucTemp = ucTurretNum - 1;

	//刀塔与结构体轴的映射
	g_SSystemPara.ucTurretToAxisMap[ucTemp] = ucTurretSign;
	return 1;
}
/*************************************************
  函数名称：int8 setMachine_REFPOT(uint8 ucREFPOT_Name, uint8 ucAxis, double dOffsetVal)
  函数功能  设置机械参考点各轴的坐标值
  输入参数：ucREFPOT_Name  第几机械参考点   1~SMACHINE_REFPOT_NUM
			  ucAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
			dOffsetVal	输入值
  输出参数：
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/

int8 setMachine_REFPOT(uint8 ucREFPOT_Name, uint8 ucAxis, double dOffsetVal)
{

	if (ucREFPOT_Name > SMACHINE_REFPOT_NUM || SMACHINE_REFPOT_NUM < 1)
	{
		return -1;
	}
	if ((ucAxis > AXIS_NUM_MAX - 1) || (ucAxis < 0))//轴号错误
	{
		return -1;
	}
	//索引为输入的机械参考点号-1
	uint8 ucREFPOT_VAL = ucREFPOT_Name - 1;
	//如果该运动轴为直径显示
	if (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter)
	{
		g_SSystemPara.SMachine_REFPOT[ucREFPOT_VAL].dArray[ucAxis] = dOffsetVal / 2;
	}
	//如果该运动轴为半径显示
	else
	{
		g_SSystemPara.SMachine_REFPOT[ucREFPOT_VAL].dArray[ucAxis] = dOffsetVal;
	}
	return 1;
}
#endif

/*************************************************
  函数名称：double getMachine_REFPOT(uint8 ucREFPOT_Name, uint8 ucAxis)
  函数功能  获取机械参考点各轴的坐标值
  输入参数：ucREFPOT_Name  第几机械参考点   1~SMACHINE_REFPOT_NUM
			  ucAxis	物理轴号 0~AXIS_PHYSICS_NUM_MAX-1
  输出参数： 指定的机械参考点的指定轴的坐标值
  返 回 值：1 成功
			-1 错误
  备    注：
****************************************/

double getMachine_REFPOT(uint8 ucREFPOT_Name, uint8 ucAxis)
{
	double result=0.0;
	if (ucREFPOT_Name > SMACHINE_REFPOT_NUM || SMACHINE_REFPOT_NUM < 1)
	{
		return -1;
	}
	if ((ucAxis > AXIS_NUM_MAX - 1) || (ucAxis < 0))//轴号错误
	{
		return -1;
	}
	//索引为输入的机械参考点号-1
	uint8 ucREFPOT_VAL = ucREFPOT_Name - 1;
	//如果该运动轴为直径显示
	if (g_SSystemPara.SAxisPara[ucAxis].bShowByDiameter)
	{
		result = g_SSystemPara.SMachine_REFPOT[ucREFPOT_VAL].dArray[ucAxis]*2;
	}
	//如果该运动轴为半径显示
	else
	{
		result = g_SSystemPara.SMachine_REFPOT[ucREFPOT_VAL].dArray[ucAxis];
	}
	return result;
}
/*************************************************
  函数名称：int8 setFeedAndCuttingUnits(uint8 ucVal)
  函数功能：设置进给与切割单位（mm 或 μm）
  输入参数：bVal  单位选择标志
				0 —— mm（毫米）
				1 —— μm（微米）
  输出参数：无
  返 回 值：1  设置成功
		   -1  参数非法
  备    注：该参数影响 G74/G56 等相关加工指令的单位解析
****************************************/
int8  setFeedAndCuttingUnits(uint8  ucVal)
{
	if (ucVal != 0 && ucVal != 1)
		return -1;   // 参数非法

	g_SSystemPara.bSetG7456mmTOum = ucVal;
	return 1;		// 成功
}
/*************************************************
  函数名称：bool getFeedAndCuttingUnits(void)
  函数功能：获取当前进给与切割单位（mm 或 μm）
  输入参数：无
  输出参数：无
  返 回 值：1  当前单位为 μm（微米）
			0  当前单位为 mm（毫米）
  备    注：该单位与系统参数 bSetG7456mmTOum 同步
****************************************/
bool  getFeedAndCuttingUnits(void)
{

	return 	g_SSystemPara.bSetG7456mmTOum;
}


/*****************************************************************************
函数名称：getG84DefaultDelayTime
函数功能：获取G84孔底暂停默认时间
输入参数：无
输出参数：无
返 回 值：G84孔底暂停默认时间，单位：ms
*****************************************************************************/
uint16 getG84DefaultDelayTime(void)
{
	return g_SSystemPara.unG84DefaultDelayTime;
}

/*****************************************************************************
函数名称：setG84DefaultDelayTime
函数功能：设置G84孔底暂停默认时间
输入参数：usVal —— G84孔底暂停默认时间，单位：ms
输出参数：无
返 回 值：1 —— 设置成功
*****************************************************************************/
int8 setG84DefaultDelayTime(uint16 usVal)
{
	g_SSystemPara.unG84DefaultDelayTime = usVal;
	return 1;
}
