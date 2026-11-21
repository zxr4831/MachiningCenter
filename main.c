#include <stdio.h>
#include <string.h>
#include <windows.h>
#include "Gcode.h"            //G代码解析
#include "Channel.h"
#include "ErrorFunction.h"
#include "RunAPI.h"
#include "SysParaAPI.h"
#include "coordAPI.h"
#include "motionAPI.h"
#include "ToolCompensation.h" //刀补
#include "toolAxis.h"


FILE *pWriteFid; ///<文件指针

extern uint32 timeji;

////G33保存数据用文件指针
bool isRunOver(void)
{
    for (int i = 0; i < CHANNEL_NUM_MAX; i++)
    {
		if (g_SChannel[i].ucChannelStatus != SYS_IDLE ||
			g_SChannel[i].SSpindle.pSSpindleAxis->ucInpStatus != INP_FINISHED)
            return FALSE;
    }
    return TRUE;
}

void channelPrint(struct SChannel *pSChannel, uint8 *ucAxisArray, int nArraySize, int nCounter)
{
    SPointDouble SRealTimePoint;
    int i = 0;

	getRealTimePoint(SRealTimePoint.dArray, AXIS_NUM_MAX, pSChannel->pMapAxisArray, NULL);
    if (nCounter > 0)
    {
        printf("%5.d\t", nCounter);
    }

    if (pSChannel->ucChannelStatus == SYS_IDLE)
    {
        printf("*完成*\t");
    }
    else
    {
        printf("行号:%d\t ", pSChannel->SPreProcess.unGCodeLineNum);
        //printf("指令类型:%d ", getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR));
    }

    for (i = 0; i < nArraySize; i++)
    {
        if (AXIS_NULL != pSChannel->pMapAxisArray[ucAxisArray[i]])
        {
                printf("%c:", (char)(ucAxisArray[i] + 88));
                printf("%.3lf\t", SRealTimePoint.dArray[ucAxisArray[i]]);
            }
        }
    printf("进给:%.3lf\t", getCurrentRealFeedRate(pSChannel->ucChannelNum, NULL));
    }

    //printf("B轴转速:%.3lf\t", pSChannel->SRun.sAxisDeltaPulse[g_SToolAxisInfo.pSToolAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[g_SToolAxisInfo.pSToolAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);

//打印输出文件到test文本中
void channelPrintf(struct SChannel *pSChannel, uint8 *ucAxisArray, int nArraySize, int nCounter)
{
    fprintf(pWriteFid, "%d,%u", nCounter, pSChannel->SPreProcess.unGCodeLineNum + 1);
    for (int i = 0; i < nArraySize; i++)
        fprintf(pWriteFid, ",%.9f", getMachCoordDouble(pSChannel->ucChannelNum, ucAxisArray[i]));
    fprintf(pWriteFid, "\n");
}
//  正常流程
DWORD WINAPI PROCESST1(LPVOID param) //线程2函数内容-------1ms中断
{
    int counter = 0;
	uint8 ucAxisArray[3] = { AXIS_X, AXIS_Y,AXIS_Z};
    (void)param;
    channelPrintf(&g_SChannel[0], ucAxisArray, 3, counter);
    while (1)
    {
        if (isErrorHasBeenWritten())
            return 4;
        if (isRunOver())
        {
            printf("IDLE\n");
            break;
        }
        counter++;
        for (int i = 0; i < CHANNEL_NUM_MAX; i++)
        {
            interruptFuncAPI(i); // 中断执行
            channelPrint(&g_SChannel[i], ucAxisArray, 3, counter);
            channelPrintf(&g_SChannel[i], ucAxisArray, 3, counter);
        }
        printf("\n");
        //if (counter == 120)
        //{
        //	if (g_SChannel[0].ucChannelStatus == SYS_PAUSE)
        //	{
        //		struct SInput SInputData;//局部变量
        //		memset(&SInputData, 0, sizeof(struct SInput));//SInputData 为局部变量 需初始化
        //		SInputData.dVs = 200.0 / 60 / 1000; //起始速度(mm/ms)
        //		SInputData.dFeedrate = 3000.0 / 60 / 1000;			//进给速度(mm/ms)
        //		SInputData.dPrimalVe = 200.0 / 60 / 1000;//最终速度(mm/ms)
        //		SInputData.dFeedRatio = -1;
        //		SInputData.ucAccVelPlanType = VEL_PLAN_S3_CURVE;//加速规划类型		T : VEL_PLAN_T_CURVE   S3 : VEL_PLAN_S3_CURVE   S4 : VEL_PLAN_S4_CURVE
        //		SInputData.ucDecVelPlanType = VEL_PLAN_S3_CURVE;//减速规划类型		T : VEL_PLAN_T_CURVE   S3 : VEL_PLAN_S3_CURVE   S4 : VEL_PLAN_S4_CURVE
        //		SInputData.ucValStatus = 5;
        //		SInputData.dAcc = 0.001;//加速度 (mm/ms2)
        //		SInputData.dDec = 0.001;//减速度 (mm/ms2)

        //		getRealTimePointByChannel(0, SInputData.SStartPoint.dArray);

        //		///*************************绝对坐标*******************************/
        //		//直接指定终点坐标
        //		SInputData.SEndPoint.dArray[AXIS_X] = 5;
        //		SInputData.SEndPoint.dArray[AXIS_Z] = 5;
        //		applyMotionParaAPI(0, &SInputData, 0);
        //	}
        //}
        //if (counter == 1000)
        //{
        //	runButtonAPI(0, FALSE);
        //}
        //if (counter == 1000)
        //{
        //	resetButtonAPI(0);
        //}
        //if (counter == 1000)
        //{
        //	simulaButtonAPI(0);
        //	simulaButtonAPI(1);
        //}
        //if (counter == 1200)
        //{
        //	changeFeedRatioAPI(0, 0.99, FALSE, 1);
        //	changeFeedRatioAPI(1, 0.99, FALSE, 1);
        //}
        //if (counter == 50)
        //{
        //	simulaButtonAPI(0);
        //}
        //if (counter == 1500)
        //{
        //	g_SChannel[0].SRun.dHandWheelSpeedRatio = 0.8;
        //}
        //if (counter == 2500)
        //{
        //	g_SChannel[0].SRun.dHandWheelSpeedRatio = 0;
        //}
        //if (counter == 3500)
        //{
        //	g_SChannel[0].SRun.dHandWheelSpeedRatio = 1.0;
        //}
        timeji++;
        machineTimerInterp(0); //加工计时启动
    }
    return 0;
}

int main(void)
{

    HANDLE IRQhandle; //线程句柄

    DWORD result = 4;

    pWriteFid = fopen("./outputFile/test.txt", "w");

	if (!pWriteFid)
        return 1;
				fprintf(pWriteFid, "cycle,program_line,commanded_x_mm,commanded_y_mm,commanded_z_mm\n");
    GCODE_OPEN_FILE(0);

    //指定文本案例:打开./resouce/O0001.txt
    //方法1:令g_ucNameType = 1，并输入0001
    //方法2:令g_ucNameType = 0，并输入O0001
    //g_SChannel[0].cG65SystemMacroRun = TRUE;
    g_ucNameType = 1; //1：加O ，0：不加O  使用子程序调用必须置为1，测试MDI用0
    strcpy(g_cName_C0, "0001"); // 按原方式读取 ./resource/O0001.txt。
    /**************************************************************************************************************************/

    //inputToolOffset(0, 1, AXIS_X, 60, FALSE);
    //inputToolOffset(0, 1, AXIS_Z, 20, FALSE);
    //inputToolOffset(0, 14, AXIS_X, 60, FALSE);
    //inputToolOffset(0, 1, AXIS_X, 20, FALSE);

    //inputToolNoseNum(0, 1, 0);
    //inputToolRadius(0, 1, 5);
    //inputToolOffset(0, 1, AXIS_Z, 10, FALSE);//输入刀长偏置值  通道0 1号刀 Z轴方向偏置10
    //inputToolOffset(0, 2, AXIS_Z, 50, FALSE);
    //inputToolOffset(0, 3, AXIS_Z, 100, FALSE);
    //inputToolOffset(1, 2, AXIS_Z, 50, FALSE);

    initMapArray();        //映射初始化（模仿加载FLASH）
    initEncoderMapArray(); //初始化编码器映射	陈敏聪	2023/8/1
    initSystemPara();      //初始化系统参数
    initAxisPara();
    //VS执行  模仿加载FLASH
    struct SEncoder *pEncoderArrays[ENCODER_NUM_MAX]; //指针数组
    for (int i = 0; i < ENCODER_NUM_MAX; i++)
    {
		pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[0][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[0][i] > ENCODER_NUM_MAX - 1) ? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[0][i]]);
    }
    initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], g_SSystemPara.ucPWMAxisMapArray[0], pEncoderArrays); //通道初始化（模仿加载FLASH）

    initToolAxis(&g_SChannel[0], &g_SChannel[0].SSingleAxisArray[TOOL_AXIS]);

    initEditablePara();                                                                       //可编辑参数初始化（模仿加载FLASH）

    initAfterLoadSysPara(); //加载系统参数后执行的初始化

    //反向间隙补偿
    //initSingleAxisRev(0, INP_GO_AHEAD);

    //g_SSystemPara.SAxisPara[3].dRevClearanceCompDist = 100;
    //initSingleAxisRev( 3, INP_GO_AHEAD);

    //双表螺距补偿
    initErrorBuf();
    /***********G33********************/

    //g_dRealTimePointArray[0] = 60/2.0;  //                 30
    //inputToolOffset(0, 1, AXIS_X, 60, FALSE);  //-》30  -  15
    //inputToolOffset(0, 2, AXIS_X, 60, FALSE);

    /*	testNewAdd();*/
    //setToolOffset(0, 1, AXIS_Z, 10);
    //g_RunMode = MDI_MODE;
    g_RunMode = AUTOMATIC_MODE;
    //启动通道
    if (runButtonAPI(0, FALSE) == 1)
    {
        IRQhandle = CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL);
        if (IRQhandle)
    {

    while (1) //g_SystemStatus != SYS_IDLE
    {
        GCodeRunLoopAPI();
                if (isRunOver() || isErrorHasBeenWritten())
                break;
        }
            WaitForSingleObject(IRQhandle, INFINITE);
            GetExitCodeThread(IRQhandle, &result);

    CloseHandle(IRQhandle);

    //fclose(pFile);
}

//void channelPrint4(struct SChannel* pSChannel, uint8* ucAxisArray, int nArraySize, int nCounter)
//{
//	SPointDouble SRealTimePoint;
//	int i = 0;
//
//	getRealTimePoint(&SRealTimePoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, NULL);
//
//	if (nCounter > 0)
//	{
//		fprintf(pWriteFid,"%5.d\t", nCounter);
//	}
//
//
//
//	if (pSChannel->ucChannelStatus == SYS_IDLE)
//	{
//		fprintf(pWriteFid, "*完成*\t");
//	}
//	else
//	{
//		fprintf(pWriteFid, "行号:%d\t ", pSChannel->SPreProcess.unGCodeLineNum);
//		//printf("指令类型:%d ", getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR));
//	}
//
//
//	for (i = 0; i < nArraySize; i++)
//	{
//		if (AXIS_NULL != pSChannel->pMapAxisArray[ucAxisArray[i]])
//		{
//			switch (ucAxisArray[i])
//			{
//			case AXIS_Y:
//			case AXIS_X:
//			case AXIS_Z:
//				fprintf(pWriteFid, "%c:", (char)(ucAxisArray[i] + 88));
//				break;
//			case AXIS_A:
//			case AXIS_B:
//			case AXIS_C:
//				fprintf(pWriteFid, "%c:", (char)(ucAxisArray[i] + 62));
//			}
//
//			if (g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[ucAxisArray[i]]].ucAxisType == ROTATION_AXIS)
//			{
//				fprintf(pWriteFid, "%.3f\t", surplusDoubleToPositive(SRealTimePoint.dArray[ucAxisArray[i]], 360));
//			}
//			else
//			{
//
//				fprintf(pWriteFid, "%.3lf\t", SRealTimePoint.dArray[ucAxisArray[i]]);
//
//			}
//		}
//	}
//	uint8 ucAxis;
//	//主轴转速打印
//
//	fprintf(pWriteFid, "C轴转速:%.3lf\t", pSChannel->SRun.sAxisDeltaPulse[pSChannel->SSpindle.pSSpindleAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);
//	//printf("A轴转速:%.3lf	", pSChannel->SRun.sAxisDeltaPulse[pSChannel->SSecondSpindle.pSSpindleAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);
//	//printf("C轴目标:%.3lf", calTargetSpindleSpeed(&pSChannel->SSpindle) * 60 * 1000);
//	//printf("M3821:%d	", ioM38M39);
//	//printf("M10:%d	", ioM10M11);
//	//printf("使能:%lu	", AxisEnable[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]]);
//	//getCurrentRealFeedRate(pSChannel->ucChannelNum, &ucAxis);
//	fprintf(pWriteFid, "进给:%.3lf\t", getCurrentRealFeedRate(pSChannel->ucChannelNum, NULL));
//	fprintf(pWriteFid, "\n");
//	//printf("主轴: %lf " , getSpindleSpeedAPI(pSChannel->ucChannelNum));
//	//printf("X剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_X));
//	//printf("Y剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_Y));
//	//printf("Z剩余距离：: %lf ",getCurLeftDist(pSChannel->ucChannelNum, AXIS_Z));
//	//printf("C剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_C));
//	return;
//}

//
//
////New Add 测试流程
// //DWORD WINAPI PROCESST1(LPVOID param)//线程2函数内容-------1ms中断
// //{
// //	int counter=0;
// //	double dAcc=g_SSystemPara.dSysPlanTMaxAcc,dDec=g_SSystemPara.dSysPlanTMaxDec; //加速
// //	uint8 res=0;
// //	uint8 record=0;
// //	uint32 sleepTime = 0;
//	//double addDist=0;
// //#if EN_DEBUG
// //	printf("PROCESST1 RUNNING\n");
// //#endif
//	//fprintf(pWriteFid, "counter 起始速度 最大速度 最终速度 剩余距离+未规划距离 重算最大速度 加速度 减速度 currentT unT5 Z X\n");
// //    while(1)
// //    {
// //
// //		counter++;
//
// //		fprintf(pWriteFid,"%d ",counter);
// //
// //		if ( res ==1 )//随动
// //		{
// //			dAcc = 5.0*g_SSystemPara.dSysPlanTMaxAcc;
// //			dDec = 5.0*g_SSystemPara.dSysPlanTMaxDec;
// //		}
// //		if ( res ==2)//减速
// //		{
// //			dAcc = g_SSystemPara.dSysPlanTMaxAcc;
// //			dDec = g_SSystemPara.dSysPlanTMaxDec;
// //		}
//
// //		if (counter >=1268)
// //		{
// //			sleepTime = 1;
//	//		addDist -= 0.001;
//	//		addDist = (addDist < 0) ? 0 : addDist;
//	//		res = newAddDistaceUnlimited(&g_SInterpolation, addDist,dAcc,dDec,0.04,1,1);
// //		}
//	//	else
//	//	{
//	//		addDist += 0.001;
//	//		addDist = (addDist > 0.04) ? 0.04 : addDist;
// //			res = newAddDistaceUnlimited(&g_SInterpolation,addDist,dAcc,dDec,0.04,1,1);
//	//	}
//
// //
// //		fprintf(pWriteFid," %d %d ",g_SInterpolation.unCurrentT,g_SInterpolation.unT5);
// // 		g_SInterpolation.ucInpStatus = INP_RUNNING;
// //		if (g_SystemStatus ==SYS_IDLE)
// //		{
// //			printf("IDLE\n");
// //			break;
// //		}
// //
// //		res = max(res,record);
// //		record = res;
// //
// //		IRQHandler();
// //
// //		fprintf(pWriteFid,"%lf  %lf\n",g_SRealTimePoint.dZ,g_SRealTimePoint.dX);
// //
// //		printf(" counter = %d res = %d \n",counter,res);
// //
// //		changeSpindleSpeed(g_SRun.SAbsCoord.dX);//输出主轴转速    必须放后面  G96限制
// //        Sleep(sleepTime);
// //		timeji++;
// //    }
// //	return 0;
// //}
//
////testNewAdd()
////{
////	g_SInterpolation.SEndPoint.dZ = 20;
////	g_SInterpolation.SEndPoint.dX = 0;
////	g_SInterpolation.SEndPoint.dY = 0;
////	g_SInterpolation.SStartPoint.dX = 0;
////	g_SInterpolation.SStartPoint.dY = 0;
////	g_SInterpolation.SStartPoint.dZ = 0;
////	calVectorAndDistance(g_SInterpolation.SStartPoint, g_SInterpolation.SEndPoint, &g_SInterpolation.dDistance, &g_SInterpolation.SStartVector);//distance为总距离
////	//g_SInterpolation.dDistance = 0.0;
////	g_SInterpolation.dPreDist = 0;
////	g_SInterpolation.dTargetDist = 0;
////	g_SInterpolation.dCurLeftDist = 0;
////	g_SInterpolation.dTc = 1;
////	g_SInterpolation.dVs = g_SSystemPara.dSysMinVel;
////	g_SInterpolation.dVmax = g_SSystemPara.dSysMaxVel;
////	g_SInterpolation.dAcc = g_SSystemPara.dSysPlanTMaxAcc;
////	g_SInterpolation.dDec = g_SSystemPara.dSysPlanTMaxAcc;
////	g_SInterpolation.dVe = 0.04;
////	setBits(g_SInterpolation.unCommandType, CMD_MOTION_LINE, CMD_MOTION_ADDR);
////	setBits(g_SInterpolation.unCommandType, CMD_PROGRAM_STOP, CMD_PROGRAM_ADDR);
////	g_SInterpolation.ucDistReAverageType = DIST_REAVG_T_CURVE;
////	g_SInterpolation.ucAccVelPlanType = VEL_PLAN_T_CURVE;
////	g_SInterpolation.ucDecVelPlanType = VEL_PLAN_T_CURVE;
////	preparePlanningAndDistReAvg(&g_SInterpolation, TRUE);
////	g_SInterpolation.ucInpStatus = INP_RUNNING;
////	g_SInterpolation.ucRunStep = 1;
////	g_SInterpolation.dFeedRatio = g_SRun.dCommonFeedRatio;
////}
//
////
//// uint8 ucT = 0; //当前的主轴周期数
//// uint32 unEncoderAccumlateNum = 0;  //主轴编码器累计的数值
////
//// double dLastSpindleSpeedIdeal;  //上一次理想主轴转速，用来判断是否更换了主轴转速S
//// double dSpindleSpeedIdeal;  //本次主轴转速
//// #define SPEED_TEXT_MAX 1
//// DWORD WINAPI PROCESST1(LPVOID param)//线程2函数内容-------1ms中断
//// {
//// #if EN_DEBUG
//// 	printf("PROCESST1 RUNNING\n");
//// #endif
//// 	int counter=0;
////     while(1)
////     {
//// 		// 		if (g_SystemStatus !=SYS_IDLE &&g_SystemStatus !=SYS_PAUSE)
//// 		// 		{
//// 		//
//// #if !ENCODER_MODE
//// 		g_SEncoder[1-1].nEncoderAbsValue = (int)GetSpindleEncoder();   //获取模拟主轴编码器数据
//// #endif
////
//// 		g_SEncoder[1-1].usDelta = round(g_SInterpolation.dSpindleSpeed /(60 * 1000) * g_SEncoder[1-1].usEncoderCirclePulseNum);
//// 		g_SEncoder[1-1].cEncoderDir = 1;
////
//// 		if(g_bCalculateSpindleSpeedFlag && g_SInterpolationAxisC.ucInpStatus == INP_FINISHED)
//// 		{
//// 			if(ucT < SPEED_TEXT_MAX)
//// 			{
//// 				unEncoderAccumlateNum += g_SEncoder[1-1].usDelta;
//// 			}
////
//// 			ucT++;
////
//// 			if(ucT == SPEED_TEXT_MAX)
//// 			{
//// 				g_dSpindleRealSpeed = (double)((double)unEncoderAccumlateNum / SPEED_TEXT_MAX) * g_SSystemPara.SAxisPara[AXIS_C-1].dPulseToMm * 60 * 1000.0;
//// 				ucT = 0;
//// 				unEncoderAccumlateNum = 0;
//// 				g_bCalculateSpindleSpeedFlag = FALSE;
////
//// 			}
////
//// 		}
////
//// 		counter++;
//// 		// 		if (counter ==200)
//// 		// 		{
//// 		// 			printf("RUN_TO_PAUSE\n");
//// 		// 			g_SystemStatus = SYS_RUN_TO_PAUSE;
//// 		// 		}
//// 		if (g_SystemStatus ==SYS_IDLE)
//// 		{
//// 			printf("IDLE\n");
//// 		}
//// 		//printf("%10.6lf %10.6lf  %d \n",g_SRealTimePoint.dX,g_SRealTimePoint.dZ,g_SRun.sAxisDeltaPulse[AXIS_C-1]);
////
//// 		//
//// 		if (g_SystemStatus !=SYS_IDLE)
//// 		{
//// 			printf("%10.6lf %10.6lf  %d \n",g_SRealTimePoint.dX, g_SRealTimePoint.dZ, g_SEncoder[1-1].nEncoderAbsValue);
//// 		}
//// 		else
//// 		{
//// 			break;
//// 		}
//// 		//fprintf(pWriteFid,"%10.6lf %10.6lf\n",g_SRun.SAbsCoord.dZ,g_SRun.SAbsCoord.dX);
//// 		//printf("spindle pulse = %d pulse/min\n",g_SRun.sAxisDeltaPulse[AXIS_C-1]*(60*1000));
//// 		/*		}*/
//// 		IRQHandler();
//// 		changeSpindleSpeed(g_SRun.SAbsCoord.dX);//输出主轴转速    必须放后面  G96限制
////         Sleep(0);
//// 		timeji++;
////     }
//// }
////
//
//
    }
    if (isErrorHasBeenWritten())
        printError();
    GCODE_CLOSE_FILE(0);
    CloseFile(0);
    fclose(pWriteFid);
    return (int)result;
}
