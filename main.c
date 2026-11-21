#include <stdio.h>
#include <string.h>
#include <process.h>
#include <math.h>
#include <windows.h>
#include "Gcode.h"            //G代码解析
#include "GeneralFunctions.h" //通用函数
#include "CTrajPlanning.h"    //速度规划
#include "common.h"
#include "IOCommand.h"
#include <time.h>
#include "Spindle.h"
#include "G1.h"
#include "G0.h"
#include "Motion.h"
#include "lookAhead.h"        //前瞻
#include "NCDataPara.h"       //参数
#include "ToolCompensation.h" //刀补
#include "G32.h"
#include "G33.h"
#include "Channel.h"
#include "ErrorCompensation.h"
#include "ErrorFunction.h"
#include "ringbuff.h"
#include "toolAxis.h"

#define XHJ 0
#define FYH 1
#define LHK 2
#define GQ 3
#define ZWZ 4
#define CMC 5
#define WHO_ARE_YOU ZWZ

#define test 1

#define INCLINED_DEBUG 0 //0:VERICUT 1:MATLAB

//声明跳转
DWORD WINAPI PROCESST1(LPVOID param); //线程2函数内容-------1ms中断
void main();

#define FYH_TEST 1
#define FYH_MULTI_TEST 1
typedef unsigned int u32; //0~4294967295，变量命名示例unVar

FILE *pWriteFid; ///<文件指针

extern uint32 timeji;

////G33保存数据用文件指针
extern FILE *fpG33Write;

bool isRunOver()
{
    int i = 0;
    for (i = 0; i < CHANNEL_NUM_MAX; i++)
    {
        if (g_SChannel[i].ucChannelStatus != SYS_IDLE || g_SChannel[i].SSpindle.pSSpindleAxis->ucInpStatus != INP_FINISHED)
        {
            return FALSE;
        }
    }
    return TRUE;
}
#if (DRAW_TEST == 1)
bool isDrawOver()
{
    int i = 0;
    for (; i < CHANNEL_NUM_MAX; i++)
    {
        if (g_bGCodeDraw[i])
            return FALSE;
    }
    return TRUE;
}
#endif
void channelStatusPrint(struct SChannel *pSChannel)
{
    switch (pSChannel->ucChannelStatus)
    {
    case SYS_RUN:
        printf("R ");
        break;
    case SYS_PAUSE:
        printf("P ");
        break;
    case SYS_IDLE:
        printf("I ");
        break;
    case SYS_RUN_TO_PAUSE:
        printf("TP ");
        break;
    case SYS_PAUSE_TO_RUN:
        printf("TR ");
        break;
    }
}

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
int64 sAxisDeltaPulse[3] = {0};                                            //倾斜Y轴机测试使用
#endif

void channelPrint(struct SChannel *pSChannel, uint8 *ucAxisArray, int nArraySize, int nCounter)
{
    SPointDouble SRealTimePoint;
    int i = 0;

    getRealTimePoint(&SRealTimePoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, NULL);
#if !FYH_MULTI_TEST
    printf("%s   ", pSChannel->SRun.cFilename);
#endif
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
            switch (ucAxisArray[i])
            {
            case AXIS_Y:
            case AXIS_X:
            case AXIS_Z:
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
                printf("%c%d:", (char)(ucAxisArray[i] + 88), 1);
#else
                printf("%c:", (char)(ucAxisArray[i] + 88));
#endif
                break;
            case AXIS_A:
            case AXIS_B:
            case AXIS_C:
                printf("%c:", (char)(ucAxisArray[i] + 62));
            }

            if (g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[ucAxisArray[i]]].ucAxisType == ROTATION_AXIS)
            {
                printf("%.3f\t", surplusDoubleToPositive(SRealTimePoint.dArray[ucAxisArray[i]], 360));
            }
            else
            {
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
                printf("%lf\t", getMachCoordDouble(0, ucAxisArray[i]));
#else
                printf("%.3lf\t", SRealTimePoint.dArray[ucAxisArray[i]]);
#endif
            }
        }
    }

    //printf("B轴转速:%.3lf\t", pSChannel->SRun.sAxisDeltaPulse[g_SToolAxisInfo.pSToolAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[g_SToolAxisInfo.pSToolAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);

    uint8 ucAxis;
    //主轴转速打印
#if FYH_MULTI_TEST
    printf("C轴转速:%.3lf\t", pSChannel->SRun.sAxisDeltaPulse[pSChannel->SSpindle.pSSpindleAxis->ucAxis] *
                                  g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);
    //printf("A轴转速:%.3lf	", pSChannel->SRun.sAxisDeltaPulse[pSChannel->SSecondSpindle.pSSpindleAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);
    //printf("C轴目标:%.3lf", calTargetSpindleSpeed(&pSChannel->SSpindle) * 60 * 1000);
    //printf("M3821:%d	", ioM38M39);
    //printf("M10:%d	", ioM10M11);
    //printf("使能:%lu	", AxisEnable[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]]);
    //getCurrentRealFeedRate(pSChannel->ucChannelNum, &ucAxis);
    printf("进给:%.3lf\t", getCurrentRealFeedRate(pSChannel->ucChannelNum, NULL));
#endif
    //printf("主轴: %lf " , getSpindleSpeedAPI(pSChannel->ucChannelNum));
    //printf("X剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_X));
    //printf("Y剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_Y));
    //printf("Z剩余距离：: %lf ",getCurLeftDist(pSChannel->ucChannelNum, AXIS_Z));
    //printf("C剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_C));
#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
    //打印绝对坐标（直角坐标系）
    printf("X:%lf ", getAbsCoordDouble(0, AXIS_X));
    printf("Y:%lf ", getAbsCoordDouble(0, AXIS_Y));
    printf("Z:%lf ", getAbsCoordDouble(0, AXIS_Z));

    sAxisDeltaPulse[0] += pSChannel->SRun.sAxisDeltaPulse[AXIS_X];
    sAxisDeltaPulse[1] += pSChannel->SRun.sAxisDeltaPulse[AXIS_Y];
    sAxisDeltaPulse[2] += pSChannel->SRun.sAxisDeltaPulse[AXIS_Z];

    printf("PULSE_X1:%d ", sAxisDeltaPulse[0]);
    printf("PULSE_Y1:%d ", sAxisDeltaPulse[1]);
    printf("PULSE_Z1:%d ", sAxisDeltaPulse[2]);
#endif
}

#if WHO_ARE_YOU == ZWZ
//打印输出文件到test文本中
void channelPrintf(struct SChannel *pSChannel, uint8 *ucAxisArray, int nArraySize, int nCounter)
{
    SPointDouble SRealTimePoint;
    int i = 0;

    getRealTimePoint(&SRealTimePoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, NULL);

    if (nCounter > 0)
    {
        fprintf(pWriteFid, "%5.d\t", nCounter);
    }

    if (pSChannel->ucChannelStatus == SYS_IDLE)
    {
        fprintf(pWriteFid, "*完成*\t");
    }
    else
    {
        fprintf(pWriteFid, "行号:%d\t ", pSChannel->SPreProcess.unGCodeLineNum);
        //printf("指令类型:%d ", getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR));
    }

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE //倾斜Y轴机
#if INCLINED_DEBUG == 0                                                    //VERICUT调试：将插补点信息转换为VERICUT控制器的G代码格式
    uint32 unCommandType;                                                  //保存指令类型
    bool bG2G3 = FALSE;
    if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE)
    {
        unCommandType = 1; //G1
    }
    else if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
    {
        unCommandType = 2; //G
        bG2G3 = TRUE;
    }
    else if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
    {
        unCommandType = 3; //G3
        bG2G3 = TRUE;
    }
    else
    {
        unCommandType = 0;
    }
    if (bG2G3 == TRUE)
    {
        fprintf(pWriteFid, "G%d X%lf Y%lf Z%lf R%lf \n", unCommandType, getMachCoordDouble(0, AXIS_X), getMachCoordDouble(0, AXIS_Y), getMachCoordDouble(0, AXIS_Z), g_SChannel[0].SG02.dRadius);
    }
    else
    {
        if (getbit(g_SChannel[0].SPreProcess.SUserData.ucValStatus, AXIS_C) == TRUE)
        {
            fprintf(pWriteFid, "G%d X%lf Y%lf Z%lf C%lf \n", unCommandType, getMachCoordDouble(0, AXIS_X), getMachCoordDouble(0, AXIS_Y), getMachCoordDouble(0, AXIS_Z), getMachCoordDouble(0, AXIS_C));
        }
        else
        {
            fprintf(pWriteFid, "G%d X%lf Y%lf Z%lf \n", unCommandType, getMachCoordDouble(0, AXIS_X), getMachCoordDouble(0, AXIS_Y), getMachCoordDouble(0, AXIS_Z));
        }
    }
#else //MATLAB调试
    //fprintf(pWriteFid, "%d %lf %lf %lf\n", counter, getMachCoordDouble(0, AXIS_Y), getMachCoordDouble(0, AXIS_X), getMachCoordDouble(0, AXIS_C));
    uint32 unCommandType; //保存指令类型
    bool bG2G3 = FALSE;
    if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_LINE)
    {
        unCommandType = 1; //G1
    }
    else if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ARC)
    {
        unCommandType = 2; //G2
        bG2G3 = TRUE;
    }
    else if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_ANTI_ARC)
    {
        unCommandType = 2; //G3
        bG2G3 = TRUE;
    }
    else
    {
        unCommandType = 0;
    }
    fprintf(pWriteFid, "%d %lf %lf %lf %d\n", counter, getMachCoordDouble(0, AXIS_Z), getMachCoordDouble(0, AXIS_X), getMachCoordDouble(0, AXIS_C), unCommandType);
#endif
#else
    for (i = 0; i < nArraySize; i++)
    {
        if (AXIS_NULL != pSChannel->pMapAxisArray[ucAxisArray[i]])
        {
            switch (ucAxisArray[i])
            {
            case AXIS_Y:
            case AXIS_X:
            case AXIS_Z:
                fprintf(pWriteFid, "%c:", (char)(ucAxisArray[i] + 88));
                break;
            case AXIS_A:
            case AXIS_B:
            case AXIS_C:
                fprintf(pWriteFid, "%c:", (char)(ucAxisArray[i] + 62));
            }

            if (g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[ucAxisArray[i]]].ucAxisType == ROTATION_AXIS)
            {
                fprintf(pWriteFid, "%.3f\t", surplusDoubleToPositive(SRealTimePoint.dArray[ucAxisArray[i]], 360));
            }
            else
            {

                fprintf(pWriteFid, "%.3lf\t", SRealTimePoint.dArray[ucAxisArray[i]]);
            }
        }
    }
#endif
    uint8 ucAxis;
    //主轴转速打印

    //fprintf(pWriteFid, "C轴转速:%.3lf\t", pSChannel->SRun.sAxisDeltaPulse[pSChannel->SSpindle.pSSpindleAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);
    //printf("A轴转速:%.3lf	", pSChannel->SRun.sAxisDeltaPulse[pSChannel->SSecondSpindle.pSSpindleAxis->ucAxis] * g_SSystemPara.SAxisPara[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE);
    //printf("C轴目标:%.3lf", calTargetSpindleSpeed(&pSChannel->SSpindle) * 60 * 1000);
    //printf("M3821:%d	", ioM38M39);
    //printf("M10:%d	", ioM10M11);
    //printf("使能:%lu	", AxisEnable[pSChannel->pMapAxisArray[pSChannel->SSpindle.pSSpindleAxis->ucAxis]]);
    //getCurrentRealFeedRate(pSChannel->ucChannelNum, &ucAxis);
    //fprintf(pWriteFid, "进给:%.3lf\t", getCurrentRealFeedRate(pSChannel->ucChannelNum, NULL));
    //fprintf(pWriteFid, "\n");
    //printf("主轴: %lf " , getSpindleSpeedAPI(pSChannel->ucChannelNum));
    //printf("X剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_X));
    //printf("Y剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_Y));
    //printf("Z剩余距离：: %lf ",getCurLeftDist(pSChannel->ucChannelNum, AXIS_Z));
    //printf("C剩余距离：: %lf ", getCurLeftDist(pSChannel->ucChannelNum, AXIS_C));
    return;
}
//  正常流程
DWORD WINAPI PROCESST1(LPVOID param) //线程2函数内容-------1ms中断
{
    int i = 0;
    int counter = 0;
    int recorder = 0xFFFFFFFF;
    uint32 sleepTime = 0;
    uint8 ucAxisArray[AXIS_NUM_MAX] = {AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, AXIS_B, AXIS_C}; //打印的轴号

#if PULSESUM_PRINT
    int32 nZPulse = 0;
    int32 nXPulse = 0;
#endif

#if EN_DEBUG
    printf("PROCESST1 RUNNING\n");
#endif
    while (1)
    {
        if (isErrorHasBeenWritten())
        {
            printError();
            return;
        }

        counter++;
        if (isRunOver())
        {
#if PRINTMODE == 4
            fprintf(pWriteFid, "IDLE\n");
#endif
            fclose(pWriteFid);
            printf("IDLE\n");
            break;
        }
#if FYH_TEST
        //printf("%5.d ", counter);
        //if(counter >6303)
        //	printf("%5.d ", counter);
#endif
        //g_SEncoder[1 - 1].lEncoderAbsValue = (int)GetSpindleEncoder();   //获取模拟主轴编码器数据
        //g_SEncoder[1 - 1].usDelta = round(g_SInterpolation.dSpindleSpeed / (60 * 1000) * g_SEncoder[1 - 1].usEncoderCirclePulseNum);
        //g_SEncoder[1 - 1].cEncoderDir = 1;

        //g_SEncoder[SPINDLE_ENCODER - 1].cEncoderDir = sgn((g_SEncoder[SPINDLE_ENCODER - 1].lEncoderAbsValue + (long)g_SChannel[0].SRun.sAxisDeltaPulse[ucSpindleAxis]) - g_SEncoder[SPINDLE_ENCODER - 1].lEncoderAbsValue);
        //g_SEncoder[SPINDLE_ENCODER - 1].lEncoderAbsValue += g_SChannel[0].SRun.sAxisDeltaPulse[ucSpindleAxis];
        //g_SEncoder[SPINDLE_ENCODER - 1].usDelta = abs(g_SChannel[0].SRun.sAxisDeltaPulse[ucSpindleAxis]);
        //if (counter ==31)
        //{
        //	counter = 31;
        //}

        for (i = 0; i < CHANNEL_NUM_MAX; i++)
        {
            interruptFuncAPI(i); // 中断执行
#if FYH_TEST
            //屏幕数据打印
            channelPrint(&g_SChannel[i], ucAxisArray, AXIS_NUM_MAX, counter);

#if PULSESUM_PRINT
            nZPulse += g_SChannel[i].SRun.sAxisDeltaPulse[AXIS_Z];
            nXPulse += g_SChannel[i].SRun.sAxisDeltaPulse[AXIS_X];
            //nPulse += g_SChannel[i].SRun.;
            //printf("脉冲X = %d\t", nXPulse);//脉冲打印
            printf("脉冲Z = %d", nZPulse); //脉冲打印

#endif
            //printf(" %d %d",g_SChannel[i].SMachineTimer.unCurProcessTime, g_SSystemPara.ucProcessTimeTotal);//加工计时打印
#endif
#if PRINTMODE == 4
            channelPrintf(&g_SChannel[i], ucAxisArray, AXIS_NUM_MAX, counter);
            //fprintf(pWriteFid, "%.3lf\t%.3lf", getMachCoordDouble(0, AXIS_C), getMachCoordDouble(0, AXIS_B));

#endif
        }
#if FYH_TEST
        printf("\n");
#endif
#if PRINTMODE == 4
        //fprintf(pWriteFid, "\n");
#else
        //绝对坐标输出
        //fprintf(pWriteFid, "%d %lf %lf %lf\n", counter, getMachCoordDouble(0, AXIS_Z), getMachCoordDouble(0, AXIS_X), getMachCoordDouble(0, AXIS_C));
#endif

        //键盘仿真
        //getKeyboard();
        //打印输出文件
        if (g_SChannel->ucChannelStatus != SYS_IDLE && PRINTMODE == 3)
        {
            GCODE_PRINT(0);
        }

#if CHANNEL_NUM_MAX <= 1
        //Sleep(sleepTime);
#endif
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

int8 LineAndArcTest(uint8 ucChannelNum, double dValX, double dValZ, double dValC, uint8 ucAxis)
{
    struct SInput SInputData; //局部变量  插补结构体
    SPointDouble SRelaPoint = {0};
    /*************启动主轴************************/
    setSpindleModeAPI(CHANNEL_ZERO, 97); //设置主轴模式为G96    （默认G97）

    spindleBeginAPI(CHANNEL_ZERO); //启动主轴

    setSpindleSpeedAPI(CHANNEL_ZERO, 300, CMD_SPINDLE_SPEED_CV); //设置主轴转速  设置线速度300 (m/min)

    /*********************************************/

    //memset(&SInputData, 0, sizeof(struct SInput));//SInputData 为局部变量 需初始化

    //SInputData.dVs = 200.0 / 60 / 1000; //起始速度(mm/ms)
    //SInputData.dFeedrate = 3000.0 / 60 / 1000;			//进给速度(mm/ms)
    //SInputData.dPrimalVe = 200.0 / 60 / 1000;//最终速度(mm/ms)
    //SInputData.dFeedRatio = -1;
    //SInputData.ucAccVelPlanType = VEL_PLAN_S3_CURVE;//加速规划类型		T : VEL_PLAN_T_CURVE   S3 : VEL_PLAN_S3_CURVE   S4 : VEL_PLAN_S4_CURVE
    //SInputData.ucDecVelPlanType = VEL_PLAN_S3_CURVE;//减速规划类型		T : VEL_PLAN_T_CURVE   S3 : VEL_PLAN_S3_CURVE   S4 : VEL_PLAN_S4_CURVE

    //SInputData.dAcc = 0.001;//加速度 (mm/ms2)
    //SInputData.dDec = 0.001;//减速度 (mm/ms2)

    ////g_SRealTimePoint.dX = 50;//可人为  设置当前点(mm)
    ////g_SRealTimePoint.dZ = 0;
    //SInputData.ucValStatus = 1 << ucAxis;
    ///********************相对坐标**************************/
    ////移动的距离(相对当前坐标)
    ////SRelaPoint.dArray[AXIS_X] = dRelaX;
    ////SRelaPoint.dArray[AXIS_Z] = dRelaZ;

    //////获取当前点
    ////getRealTimePointByChannel(ucChannelNum, SInputData.SEndPoint.dArray);
    //////坐标相加  机械坐标＋相对值
    ////addPoint(SInputData.SEndPoint.dArray, SRelaPoint.dArray, SInputData.SEndPoint.dArray, AXIS_NUM_MAX, NULL);

    /////*************************绝对坐标*******************************/
    ////直接指定终点坐标
    //SInputData.SEndPoint.dArray[AXIS_X] = dValX;
    //SInputData.SEndPoint.dArray[AXIS_Z] = dValZ;
    //SInputData.SEndPoint.dArray[AXIS_C] = dValC;

    ///*************************绝对坐标C轴*******************************/
    ////SInputData.SEndPoint.dArray[AXIS_C] = 180;

    //SInputData.dRadius = -50;// 半径为25mm（>0表示小于或等于180度的圆弧；<0表示大于180°的圆弧)

    //if (applyMotionParaAPI(ucChannelNum,&SInputData,0) == 1) return 1;//调用接口函数  G03
    //else return 0;
}
//void main()
//{
//	int i;
//	//int8 FilenameArray[] = "6.txt";    //G代码文本
//	//FILE *pFile = fopen(FilenameArray, "rb");     //文件指针,fopen会创建一个输入文件缓冲区，pFile指向该缓冲区的首地址
//	double temp1 = 0.0;
//
//	HANDLE IRQhandle;//线程句柄
//
//	pWriteFid = fopen("test.txt", "w");
//	fpG33Write = fopen("G33.txt", "w");
//	//printf("ssss= %d\n", sizeof(struct SSingleAxis));
//	for (int j = 0; j < CHANNEL_NUM_MAX; j++)
//	{
//		GCODE_OPEN_FILE(j);    //VS测试  G代码打印
//	}
//
//	//指定文本案例:打开./resouce/O0001.txt
//	//方法1:令g_ucNameType = 1，并输入0001
//	//方法2:令g_ucNameType = 0，并输入O0001
//	g_ucNameType = 0;    //1：加O ，0：不加O  测试子程序调用测试必须置为1
//
//	//strcpy(g_cName_C0, "1094"); //通道0主程序名
//	strcpy(g_cName_C0, "3"); //通道0主程序名
//#if CHANNEL_NUM_MAX > 1
//	//strcpy(g_cName_C1, "1002");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
//	//strcpy(g_cName_C1, "71");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
//	strcpy(g_cName_C1, "0011");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
//
//#endif // CHANNEL_NUM_MAX > 1
//	/**************************************************************************************************************************/
//
//	initMapArray();//映射初始化（模仿加载FLASH）
//	initSystemPara();//初始化系统参数
//	initAxisPara();
//	//VS执行  模仿加载FLASH
//	initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], &g_SEncoder[SPINDLE_ENCODER - 1], &g_SEncoder[HANDEWHEEL_ENCODER - 1]);		//通道初始化（模仿加载FLASH）
//#if CHANNEL_NUM_MAX > 1
//	initChannel(&g_SChannel[1], 1, g_SSystemPara.ucAxisMapArray[1], &g_SEncoder[2 - 1], &g_SEncoder[HANDEWHEEL_ENCODER - 1]);		//通道初始化（模仿加载FLASH）
//#endif // CHANNEL_NUM_MAX > 1
//	initEditablePara();   //可编辑参数初始化（模仿加载FLASH）
//
//	initAfterLoadSysPara();//加载系统参数后执行的初始化
//
//	/***********G33********************/
//	g_dZAxisFloatingLength = 0.0;
//	g_SChannel[0].pSSpindleEncoder->lEncoderAbsValue = g_lSpindleEncoderValue = 6250;
//
//	fprintf(fpG33Write, "%lf\t%lf\t%lf\t%lf\t%hd\t%ld\n", 0.0, 5.0, 0.0, 0.0, 0, g_lSpindleEncoderValue);
//	/***********G33********************/
//
//
// /*	testNewAdd();*/
//	//setToolOffset(0, 1, AXIS_Z, 10);
//	g_RunMode = AUTOMATIC_MODE;
//	//启动通道
//	for (i = 0; i < CHANNEL_NUM_MAX; i++)
//	{
//		runButtonAPI(i);
//	}
//	//runButtonAPI(&g_SChannel[0]);
//	//LineAndArcTest(0,100,0,0,AXIS_X);
//
//	IRQhandle = (HANDLE)CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL);//创建IRQ中断线程
//	//Sleep(10);
//	//LineAndArcTest(0, 0, 100,180,AXIS_Z);
//
////刀补测试用
//#if ENABLE_TOOL_DEBUG
//	TCODE_OPEN_FILE(0);
//	TCODE_OPEN_FILE(1);
//#endif
//
//	while (1)//g_SystemStatus != SYS_IDLE
//	{
//		GCodeRunLoopAPI();
//	}
//
//	system("pause");
//	CloseHandle(IRQhandle);
//
//	//fclose(pFile);
//}
#endif // WHO_ARE_YOU == ZWZ

#if WHO_ARE_YOU == LHK
/********************************************************************************************************/
/*********************************友情提示：G32测试专用，其他测试请屏蔽下方内容******************************/
/********************************************************************************************************/
struct _SG32 pSG32;
bool bG32Flag = FALSE;
double lastEncoderBackNum = 0;
//LHK测试Pulse限定专属！！！
void channelG32Print(struct SChannel *pSChannel, int counter)
{

    /*if (getBits(pSChannel->SPreProcess.unCommandType, CMD_MOTION_ADDR) != CMD_MOTION_SCREWCUT)
	{
		return;
	}*/

    SPointDouble SRealTimePoint;
    counter++;
    getRealTimePoint(&SRealTimePoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, NULL);

    if (pSChannel->SG32.bSyncSignal || !bG32Flag)
    {
        if (pSChannel->SG32.bPrintFlag || !bG32Flag)
        {
            //	fprintf(pWriteFid, "%d ", counter);
        }
    }
    if (pSChannel->SG32.bSyncSignal || !bG32Flag)
    {
        if (pSChannel->SG32.bPrintFlag || !bG32Flag)
        {
#if !MATLAB_DEBUG

            fprintf(pWriteFid, " %d %d ", pSChannel->SG32.SPlanData.unCurrentT, pSChannel->SG32.SPlanData.unT5);
#endif
            double dEncoderBackNum = surplusDoubleData(pSChannel->pSSpindleEncoder->lEncoderAbsValue, pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum);
            while (dEncoderBackNum < lastEncoderBackNum)
            {
                double i = 1.0;
                dEncoderBackNum = dEncoderBackNum + i * pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum;
                i++;
            }
            lastEncoderBackNum = dEncoderBackNum;
            dEncoderBackNum = dEncoderBackNum * 0.036;
            if (getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE)
            {
                fprintf(pWriteFid, "G0\t");
                fprintf(pWriteFid, "X%lf\t ", SRealTimePoint.dArray[AXIS_X]);
                fprintf(pWriteFid, "Z%lf\n ", SRealTimePoint.dArray[AXIS_Z]);
            }
            else
            {
                fprintf(pWriteFid, "G1\t");
                //fprintf(pWriteFid, "\t%d\t ", pSChannel->pSSpindleEncoder->lEncoderAbsValue);
                //	fprintf(pWriteFid, "\t%lf\t ", dEncoderBackNum);
                fprintf(pWriteFid, "X%lf\t ", SRealTimePoint.dArray[AXIS_X]);
                //	fprintf(pWriteFid, "Z%lf\t  C%lf \n", SRealTimePoint.dArray[AXIS_Z], SRealTimePoint.dArray[AXIS_C]);
                fprintf(pWriteFid, "Z%lf\t ", SRealTimePoint.dArray[AXIS_Z]);
                fprintf(pWriteFid, " C%lf \n", dEncoderBackNum);
            }
            //fprintf(pWriteFid, "%lf  %lf \n", SRealTimePoint.dArray[pSChannel->SG32.ucAxis[0]], SRealTimePoint.dArray[pSChannel->SG32.ucAxis[1]]);  //第一个为长轴轴号，第二个为短轴轴号
        }
    }
}

//G32流程
DWORD WINAPI PROCESST1(LPVOID param) //线程2函数内容-------1ms中断
{
    int i = 0;
    int counter = 0;
    int recorder = 0xFFFFFFFF;
    uint32 sleepTime = 0;
    //SPointDouble SRealTimePoint;
    //uint8 ucSpindleAxis = g_SChannel[0].SSpindle.pSSpindleAxis->ucAxis; // 主轴轴号
    //uint8 ucSpindlePhysicalAxis = g_SChannel[0].pMapAxisArray[ucSpindleAxis];//主轴物理轴
    uint8 ucAxisArray[3] = {AXIS_X, AXIS_Z, AXIS_C};

#if !MATLAB_DEBUG
    fprintf(pWriteFid, "counter 起始速度 最大速度 最终速度 剩余距离+未规划距离 重算最大速度 加速度 减速度 currentT unT5 编码器数值 Z X\n");
#endif
#if EN_DEBUG
    printf("PROCESST1 RUNNING\n");
#endif
    //fprintf(pWriteFid,"counter 起始速度 最大速度 最终速度 剩余距离+未规划距离 重算最大速度 currentT unT5 Z X\n");
    while (1)
    {
        counter++;
        if (isRunOver())
        {
            fclose(pWriteFid);
            GCODE_CLOSE_FILE(0);
            printf("IDLE\n");
            break;
        }

        for (i = 0; i < CHANNEL_NUM_MAX; i++)
        {
#if !ENCODER_MODE
            simulateGetEncoder(&g_SChannel[i]);
#endif
            spindleSpeedTest(&g_SChannel[i].SSpindleTest, &g_SChannel[i].SSpindle);
            resetProgram(&g_SChannel[i]);
            IRQHandler(&g_SChannel[i]);
            spindleMotion(&g_SChannel[i].SSpindle); // 主轴运动
                                                    //channelPrint(&g_SChannel[i], ucAxisArray, 2, -1);
        }
        channelG32Print(&g_SChannel[0], counter);
        //printf("\n");

        timeji++;
    }
    return 0;
}

//void main()
//{
//	int i;
//	//int8 FilenameArray[] = "6.txt";    //G代码文本
//	//FILE *pFile = fopen(FilenameArray, "rb");     //文件指针,fopen会创建一个输入文件缓冲区，pFile指向该缓冲区的首地址
//	double temp1 = 0.0;
//
//	HANDLE IRQhandle;//线程句柄
//
//	pWriteFid = fopen("test.txt", "w");
//	//printf("ssss= %d\n", sizeof(struct SSingleAxis));
//	for (int j = 0; j < CHANNEL_NUM_MAX; j++)
//	{
//		GCODE_OPEN_FILE(j);    //VS测试  G代码打印
//	}
//
//	//指定文本案例:打开./resouce/O0001.txt
//	//方法1:令g_ucNameType = 1，并输入0001
//	//方法2:令g_ucNameType = 0，并输入O0001
//	g_ucNameType = 0;    //1：加O ，0：不加O  测试子程序调用测试必须置为1
//
//	//strcpy(g_cName_C0, "1094"); //通道0主程序名
//	//strcpy(g_cName_C0, "G72.4"); //通道0主程序名
//	//strcpy(g_cName_C0, "G76.2.2"); //通道0主程序名
//	//strcpy(g_cName_C0, "G32.5.2.1.1"); //通道0主程序名
//	strcpy(g_cName_C0, "G92.2"); //通道0主程序名
//#if CHANNEL_NUM_MAX > 1
//	//strcpy(g_cName_C1, "1002");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
//	//strcpy(g_cName_C1, "71");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
//	strcpy(g_cName_C1, "0011");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
//
//#endif // CHANNEL_NUM_MAX > 1
//	/**************************************************************************************************************************/
//
//	initMapArray();//映射初始化（模仿加载FLASH）
//	initSystemPara();//初始化系统参数
//	initAxisPara();
//	//VS执行  模仿加载FLASH
//	initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], &g_SEncoder[SPINDLE_ENCODER - 1], &g_SEncoder[HANDEWHEEL_ENCODER - 1]);		//通道初始化（模仿加载FLASH）
//#if CHANNEL_NUM_MAX > 1
//	initChannel(&g_SChannel[1], 1, g_SSystemPara.ucAxisMapArray[1], &g_SEncoder[2 - 1], &g_SEncoder[HANDEWHEEL_ENCODER - 1]);		//通道初始化（模仿加载FLASH）
//#endif // CHANNEL_NUM_MAX > 1
//	initEditablePara();   //可编辑参数初始化（模仿加载FLASH）
//
//	initAfterLoadSysPara();//加载系统参数后执行的初始化
//
//
// /*	testNewAdd();*/
//	//setToolOffset(0, 1, AXIS_Z, 10);
//	g_RunMode = AUTOMATIC_MODE;
//	//启动通道
//	for (i = 0; i < CHANNEL_NUM_MAX; i++)
//	{
//		runButtonAPI(i);
//	}
//	//runButtonAPI(&g_SChannel[0]);
//	//LineAndArcTest(0,100,0,0,AXIS_X);
//
//	IRQhandle = (HANDLE)CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL);//创建IRQ中断线程
//	//Sleep(10);
//	//LineAndArcTest(0, 0, 100,180,AXIS_Z);
//
////刀补测试用
//#if ENABLE_TOOL_DEBUG
//	TCODE_OPEN_FILE(0);
//	TCODE_OPEN_FILE(1);
//#endif
//
//	while (1)//g_SystemStatus != SYS_IDLE
//	{
//		GCodeRunLoopAPI();
//	}
//
//
//
//	system("pause");
//	CloseHandle(IRQhandle);
//
//
//	//fclose(pFile);
//
//}

//void test()
//{
//	g_SRealTimePoint.dX = 0;
//	g_SRealTimePoint.dZ = 0;
//
//	pSG32.SUserData.dVs = 200.0 / 60 / 1000;
//	pSG32.SUserData.dVe = 200.0 / 60 / 1000;
//	pSG32.SUserData.dVmax = 0.05;
//
//	pSG32.SUserData.dAcc = 0.001;
//	pSG32.SUserData.dDec = 0.001;
//
//	pSG32.SUserData.SEndPoint.dZ = g_SRealTimePoint.dZ + 20;
//	pSG32.SUserData.SEndPoint.dX = g_SRealTimePoint.dX + 15;
//	pSG32.SUserData.dJ = 2;
//	pSG32.SUserData.dK = 2;
//	pSG32.SUserData.dQ = 0;
//	pSG32.SUserData.dF = 1;
//
//	/*************启动主轴************************/
//	setSpindleModeAPI(&g_SSingleAxisC, 97);   //设置主轴模式为G97    （默认G97）
//
//	spindleBeginAPI(&g_SSingleAxisC);          //启动主轴
//
//	setSpindleSpeedAPI(&g_SSingleAxisC, 1000, CMD_SPINDLE_SPEED_CV);//设置主轴转速  设置主轴转速1000 (r/min)
//
//	/*********************************************/
//
//	g_SSingleAxisC.ucInpStatus = INP_FINISHED;
//	applyG32MotionParaAPI(&pSG32);
//
//}

#endif //WHO_ARE_YOU == LHK

// ;*/
//	//LineAndArcTest();
//
//	//启动通道
//	for (i = 0; i < CHANNEL_NUM_MAX; i++)
//	{
//		runButtonAPI(&g_SChannel[i]);
//	}
//	//runButtonAPI(&g_SChannel[0]);
//
//	IRQhandle = (HANDLE)CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL);//创建IRQ中断线程
//
//
////刀补测试用
//#if ENABLE_TOOL_DEBUG
//	TCODE_OPEN_FILE(0);
//	TCODE_OPEN_FILE(1);
//#endif
//
//	while (1)//g_SystemStatus != SYS_IDLE
//	{
//		for (i = 0; i < CHANNEL_NUM_MAX; i++)
//		{
//			GCodeRunAPI(&g_SChannel[i]);
//		}
//		//GCodeRunAPI(&g_SChannel[0]);
//	}
//
//
//	system("pause");
//	CloseHandle(IRQhandle);
//
//
//	//fclose(pFile);
//
//}

////测试端口
//void main()
//{
//
//	//int8 FilenameArray[] = "6.txt";    //G代码文本
//	//FILE *pFile = fopen(FilenameArray, "rb");     //文件指针,fopen会创建一个输入文件缓冲区，pFile指向该缓冲区的首地址
//	double temp1 = 0.0;
//	HANDLE IRQhandle;//线程句柄
//
//	pWriteFid = fopen("test.txt", "w");
//	//g_GcodeRelatedObj.sBeginLineNum = 0;                           //G代码开始的行号，第一个%号行
//	//g_GcodeRelatedObj.unEndLineNum = 0;                           //G代码最后一行的行号，第二个%号行
//	//g_GcodeRelatedObj.unLineNum = 0;               //文本文件的行号，文本最后一行的定义:最后一个回车换行符\r\n出现的那一行
//	//printf("ssss= %d\n", sizeof(struct SSingleAxis));
//	stm32TextSimulation("G32.1.txt", 1);
//	/**************************************************************************************************************************/
//	initSystemPara();//初始化系统参数
//	initAfterLoadSysPara();
//
//	//grammarCheck(NULL, &g_GcodeRelatedObj.unLineNum, &g_GcodeRelatedObj.sBeginLineNum, &g_GcodeRelatedObj.unEndLineNum);    //语法检查
//
//	//if (g_bErrorWriteFlag == TRUE)//语法检查错误  不执行
//	//{
//	//	printError();
//	//	system("pause");
//	//	return;
//	//}
//	//初始化参数
//	gcodeInit();
//	toolCompensationInit();						//刀补初始化
//	lookAheadInit();											//速度规划初始化
//	interpolationInit();										//插补初始化
//	//printf("ssss= %d\n", sizeof(g_SSystemPara));
//	g_SystemStatus = SYS_IDLE;        //空闲状态
//	test();
//
//	IRQhandle = (HANDLE)CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL);//创建IRQ中断线程
//	g_SystemStatus = SYS_RUN;
//
//	while (g_SystemStatus != SYS_IDLE)//g_SystemStatus != SYS_IDLE
//	{
//		//if (g_SystemStatus != SYS_IDLE)
//		//{
//		//	//assignSegments(NULL, &g_GcodeRelatedObj.unLineNum, g_GcodeRelatedObj.unEndLineNum);  //一行G代码赋值到解析结构体
//		//	if (mainFuncGCode() == 1)
//		//	{
//		//		CloseHandle(IRQhandle);
//		//		printError();
//		//		system("pause");
//		//		return;
//		//	}
//		//}
//		//if (g_SystemStatus != SYS_IDLE)
//		//{
//		//	ToolCompensation_Caculate();//刀补计算
//		//}
//		//if (g_SystemStatus == SYS_RUN || g_SystemStatus == SYS_PAUSE_TO_RUN)//SYS_PAUSE_TO_RUN进入速度前瞻将flag置TRUE
//		//{
//		//	velocityLookAhead();//速度前瞻
//		//}
//
//		//runIOBuff();
//
//	}
//
//
//	system("pause");
//	CloseHandle(IRQhandle);
//
//
//	//fclose(pFile);
//
//}

#if WHO_ARE_YOU == GQ
/********************************************************************************************************/
/*********************************友情提示：G33测试专用，其他测试请屏蔽下方内容******************************/
/********************************************************************************************************/
typedef unsigned int u32; //0~4294967295，变量命名示例unVar

FILE *pWriteFid; ///<文件指针

extern u32 timeji;

//int8 G33Test()
//{
//	//局部变量声明
//	struct SInterpolationPre testPreProcess;//局部变量  插补结构体
//	struct G33 testG33;  //局部变量  攻牙结构体
//	double dTappingLength; //攻牙长度
//	double dTappingDir;    //攻牙方向
//	int8 cSpindleDir;      //主轴旋转方向
//	double dSpindleSpeed;  //主轴转速
//	double dF;             //攻牙导程
//	uint8  ucL;            //攻牙头数
//
//	/*************参数设置区*****************/
//	dTappingLength = 10;  //攻牙长度 单位mm
//	dTappingDir = -1;     //攻牙方向（-1：丝锥向Z轴负方向   1：丝锥向Z轴正方向）
//	cSpindleDir = 1;     //主轴转向（-1：M04  1: M03）
//	dSpindleSpeed = 1000; //主轴转速（r/min）
//	dF = 2;               //攻牙导程（mm）
//	ucL = 1;              //攻牙头数
//	/*************参数设置完毕***************/
//
//
//	//下方为根据上方参数变量的值进行初始化设置，不用修改//
//	/*************启动主轴************************/
//	setSpindleModeAPI(&g_SSingleAxisC, 97);   //设置主轴模式为G97    （默认G97）
//
//	setSpindleDirAPI(&g_SSingleAxisC, cSpindleDir == 1 ? ARC_CW : ARC_ANTI_CW);//设置主轴转向   MO3:ARC_CW   M04:ARC_ANTI_CW
//
//	spindleBeginAPI(&g_SSingleAxisC);//启动主轴
//
//	setSpindleSpeedAPI(&g_SSingleAxisC, dSpindleSpeed, CMD_SPINDLE_SPEED_CV);//设置主轴转速(恒转速)  设置转速300 (r/min)
//	/*********************************************/
//	memset(&testPreProcess, 0, sizeof(struct SInterpolationPre));//testPreProcess 为局部变量 需初始化
//	memset(&testG33, 0, sizeof(struct G33));
//
//	//参数插补结构体赋值
//	testPreProcess.SUserData.SEndPoint.dZ = g_SRealTimePoint.dZ + dTappingDir * dTappingLength;   //攻牙Z轴终点坐标(相对于实时点向Z轴负方向攻的长度)
//	testPreProcess.SUserData.SEndPoint.dX = g_SRealTimePoint.dX;          //攻牙X轴终点坐标(建议测试前实时点X轴坐标为0)
//
//	//参数G33攻牙结构体赋值
//	testG33.dF = dF;     //导程
//	testG33.ucL = ucL;      //内螺纹头数
//
//	if (applyG33ParaAPI(&testPreProcess, &testG33) == 1) return 1;//调用接口函数  G33
//	else return 0;
//}

#if !G33_USE_REAL_ENCODER
double dActualSpindleSpeed; //由脉冲反算的主轴实际转速（郭强）仿真用
#endif

//  正常流程
DWORD WINAPI PROCESST1(LPVOID param) //线程2函数内容-------1ms中断
{

    int i = 0;
    int counter = 0;
    int recorder = 0xFFFFFFFF;
    uint32 sleepTime = 0;
    uint8 ucAxisArray[3] = {AXIS_X, AXIS_Z, AXIS_C}; //打印的轴号

#if EN_DEBUG
    printf("PROCESST1 RUNNING\n");
#endif
    while (1)
    {
        counter++;
        if (isRunOver())
        {
            fclose(pWriteFid);
            fclose(fpG33Write);
            printf("IDLE\n");
#if !STM32_MODE
            for (i = 0; i < CHANNEL_NUM_MAX; i++)
            {
                GCODE_CLOSE_FILE(i); //关闭输出文件  测试用
            }
#endif
            break;
        }
        //#if FYH_TEST
        //		printf("%5.d ", counter);
        //#endif
        for (i = 0; i < CHANNEL_NUM_MAX; i++)
        {
            interruptFuncAPI(i); // 中断执行
                                 //#if FYH_TEST
                                 //			channelPrint(&g_SChannel[i], ucAxisArray, 3, -1);
                                 //#endif
        }
        //#if FYH_TEST
        //		printf("\n");
        //#endif
        //绝对坐标输出
        //fprintf(pWriteFid, "%d %lf %lf 0.0\n", counter, getMachCoordDouble(0, AXIS_Z), getMachCoordDouble(0, AXIS_X));
#if !G33_USE_REAL_ENCODER
        if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_G33)
        {
            const uint8 *ucMapArray = g_SChannel[0].pMapAxisArray; //通道轴映射数组
            //9.21郭强，模拟编码器计数 仿真用
            AnalogSpindleEncoderCount(&g_SChannel[0].SG33, g_SChannel[0].SRun.sAxisDeltaPulse[AXIS_C], g_SChannel[0].pSSpindleEncoder);

            //9.21郭强，计算主轴实际转速 仿真用
            g_dActualSpindleSpeed = dActualSpindleSpeed = (double)g_SChannel[0].SRun.sAxisDeltaPulse[AXIS_C] * g_SSystemPara.SAxisPara[ucMapArray[AXIS_C]].dPulseToMm * 60 * 1000 / DEGREE_OF_CIRCLE;

            //10.14郭强 计算Z轴浮动量
            calcZAxisFloatingLength(&g_SChannel[0].SG33, (int32)(g_SChannel[0].SRun.sAxisDeltaPulse[AXIS_Z]), (int32)(g_SChannel[0].SRun.sAxisDeltaPulse[AXIS_C]));
        }
#endif

#if G33_MOTION_DATA_DEBUG
        if (getBits(g_SChannel[0].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_G33)
        {
            const uint8 *ucMapArray = g_SChannel[0].pMapAxisArray; //通道轴映射数组
            SPointDouble SRealTimePoint;
            getRealTimePoint((double *)&SRealTimePoint, AXIS_NUM_MAX, ucMapArray, &g_SChannel[0].SG33.ucValStatus);

            printf("%5.d\t%lf\t%lf\t", counter, SRealTimePoint.dArray[AXIS_X], SRealTimePoint.dArray[AXIS_Z]);
            printf("%lf\t%lf\t%hd\t%ld\n", g_dZAxisFloatingLength, g_dActualSpindleSpeed, g_SChannel[0].SRun.sAxisDeltaPulse[AXIS_C], g_lSpindleEncoderValue);
            fprintf(fpG33Write, "%lf\t%lf\t", SRealTimePoint.dArray[AXIS_X], SRealTimePoint.dArray[AXIS_Z]);
            fprintf(fpG33Write, "%lf\t%lf\t%hd\t%ld\n", g_dZAxisFloatingLength, g_dActualSpindleSpeed, g_SChannel[0].SRun.sAxisDeltaPulse[AXIS_C], g_lSpindleEncoderValue);
        }
#endif
#if CHANNEL_NUM_MAX <= 1
        //Sleep(sleepTime);
#endif
        if (counter == 1000)
        {
            runButtonAPI(0);
        }
        timeji++;
        machineTimerInterp(0); //加工计时启动
    }
    return 0;
}
#endif //WHO_ARE_YOU == GQ

void main()
{
    int i;
    //int8 FilenameArray[] = "6.txt";    //G代码文本
    //FILE *pFile = fopen(FilenameArray, "rb");     //文件指针,fopen会创建一个输入文件缓冲区，pFile指向该缓冲区的首地址
    double temp1 = 0.0;

    HANDLE IRQhandle; //线程句柄

    pWriteFid = fopen("test.txt", "w");
    fpG33Write = fopen("G33.txt", "w");
    //printf("ssss= %d\n", sizeof(struct SSingleAxis));
    for (int j = 0; j < CHANNEL_NUM_MAX; j++)
    {
        GCODE_OPEN_FILE(j); //VS测试  G代码打印
    }

    //指定文本案例:打开./resouce/O0001.txt
    //方法1:令g_ucNameType = 1，并输入0001
    //方法2:令g_ucNameType = 0，并输入O0001
    //g_SChannel[0].cG65SystemMacroRun = TRUE;
    g_ucNameType = 1; //1：加O ，0：不加O  使用子程序调用必须置为1，测试MDI用0
    int sss = sizeof(g_SRingBuff);
    //strcpy(g_cName_C0, "1094"); //通道0主程序名
    //strcpy(g_cName_C0, "3"); //通道0主程序名
    //strcpy(g_cName_C0, "5"); //通道0主程序名
    //strcpy(g_cName_C0, "5"); //通道0主程序名
    //strcpy(g_cName_C0, "G712.1"); //通道0主程序名
    strcpy(g_cName_C0, "0095"); //通道0主程序名
#if CHANNEL_NUM_MAX > 1
    //strcpy(g_cName_C1, "1002");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
    //strcpy(g_cName_C1, "71");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE
    //strcpy(g_cName_C1, "0004");   //通道1主程序名   注：主动启动通道1，需修改TEMP_ACTIVATIVE_CHANNEL_ONE

#endif // CHANNEL_NUM_MAX > 1
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
        pEncoderArrays[i] =
            (g_SSystemPara.ucEncoderMapArray[0][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[0][i] > ENCODER_NUM_MAX - 1) ? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[0][i]]);
    }
    initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], g_SSystemPara.ucPWMAxisMapArray[0], pEncoderArrays); //通道初始化（模仿加载FLASH）

    initToolAxis(&g_SChannel[0], &g_SChannel[0].SSingleAxisArray[TOOL_AXIS]);

#if SCREWCOMPTESTCODE
    Vs_ScrewCompLoadPara(0);
    Vs_ScrewCompLoadPara(2);
#endif

#if CHANNEL_NUM_MAX > 1
    struct SEncoder *Channel1_pEncoderArrays[ENCODER_NUM_MAX]; //指针数组
    for (int i = 0; i < ENCODER_NUM_MAX; i++)
    {
        Channel1_pEncoderArrays[i] =
            (g_SSystemPara.ucEncoderMapArray[1][i] == ENCODER_NULL || g_SSystemPara.ucEncoderMapArray[1][i] > ENCODER_NUM_MAX - 1) ? NULL : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[1][i]]);
    }
    initChannel(&g_SChannel[1], 1, g_SSystemPara.ucAxisMapArray[1], Channel1_pEncoderArrays); //通道初始化（模仿加载FLASH）
#endif                                                                                        // CHANNEL_NUM_MAX > 1
    initEditablePara();                                                                       //可编辑参数初始化（模仿加载FLASH）

    initAfterLoadSysPara(); //加载系统参数后执行的初始化

    //反向间隙补偿
    //initSingleAxisRev(0, INP_GO_AHEAD);

    //g_SSystemPara.SAxisPara[3].dRevClearanceCompDist = 100;
    //initSingleAxisRev( 3, INP_GO_AHEAD);

    //双表螺距补偿
#if ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
    g_SSystemPara.bBilateralScrewEnable[AXIS_Z] = TRUE;
    g_SSystemPara.sScrewReferenceNum[AXIS_Z] = 5;
    g_SSystemPara.fScrewReferencePoint[AXIS_Z] = 10;

    g_SSystemPara.cAheadScrewCompVal[AXIS_Z][2] = 3;
    g_SSystemPara.cAheadScrewCompVal[AXIS_Z][3] = 2;
    g_SSystemPara.cAheadScrewCompVal[AXIS_Z][4] = 1;
    g_SSystemPara.cAheadScrewCompVal[AXIS_Z][5] = 0;
    g_SSystemPara.cAheadScrewCompVal[AXIS_Z][6] = 1;
    g_SSystemPara.cAheadScrewCompVal[AXIS_Z][7] = 2;

    g_SSystemPara.cBackScrewCompVal[AXIS_Z][2] = 3;
    g_SSystemPara.cBackScrewCompVal[AXIS_Z][3] = 2;
    g_SSystemPara.cBackScrewCompVal[AXIS_Z][4] = 1;
    g_SSystemPara.cBackScrewCompVal[AXIS_Z][5] = -80;
    g_SSystemPara.cBackScrewCompVal[AXIS_Z][6] = 1;
    g_SSystemPara.cBackScrewCompVal[AXIS_Z][7] = 2;

    g_SSystemPara.ucBilateralScrewGap[AXIS_Z] = 10;

    initSBilateralScrewComp(AXIS_Z, 1);
#endif
    //g_dRealTimePointArray[g_SChannel[0].pMapAxisArray[AXIS_Z]] = 150;
    /***********G33********************/
    g_dZAxisFloatingLength = 0.0;
    //g_SChannel[0].pSSpindleEncoder->lEncoderAbsValue = g_lSpindleEncoderValue = 6250;

    fprintf(fpG33Write, "%lf\t%lf\t%lf\t%lf\t%hd\t%ld\n", 0.0, 5.0, 0.0, 0.0, 0, g_lSpindleEncoderValue);
    /***********G33********************/

    //g_dRealTimePointArray[0] = 60/2.0;  //                 30
    //inputToolOffset(0, 1, AXIS_X, 60, FALSE);  //-》30  -  15
    //inputToolOffset(0, 2, AXIS_X, 60, FALSE);

    /*	testNewAdd();*/
    //setToolOffset(0, 1, AXIS_Z, 10);
    //g_RunMode = MDI_MODE;
    g_RunMode = AUTOMATIC_MODE;
    //启动通道
    for (i = 0; i < 1; i++) //先初始化通道0
    {
        runButtonAPI(i, FALSE);
        //load2ProcessBtnAPI(i);
    }
    //runButtonAPI(&g_SChannel[0]);
    //LineAndArcTest(0,100,0,0,AXIS_X);

    IRQhandle = (HANDLE)CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL); //创建IRQ中断线程
                                                                         //Sleep(10);
                                                                         //LineAndArcTest(0, 0, 100,180,AXIS_Z);

//刀补测试用
#if ENABLE_TOOL_DEBUG
    TCODE_OPEN_FILE(0);
    TCODE_OPEN_FILE(1);
    //刀补测试输入刀具半径、刀鼻方向
    inputToolRadius(0, 2, 3);
    inputToolNoseNum(0, 2, 3);
#endif
//inputToolOffset(0, 1, AXIS_Z, 10, FALSE);//输入刀长偏置值  通道0 1号刀 Z轴方向偏置10
#if STM32_MODE
    if (g_SSystemPara.ucToolDevie == TOOL_TURRET_MODE)
    {
        for (int ucCounter = 0; ucCounter < CHANNEL_NUM_MAX; ucCounter++)
        {
            //STM32刀塔模式下开机前读取刀塔伺服的状态和刀号
            ReadToolNum_AfterLoadSysPara(&g_SChannel[ucCounter].STIOCommand, &g_SChannel[ucCounter].SRun, &g_SChannel[ucCounter].SPreProcess);
        }
    }
#endif

    while (1) //g_SystemStatus != SYS_IDLE
    {
        GCodeRunLoopAPI();
#if (DRAW_TEST == 1)
        gCodeDrawLoop();
#endif

#if !STM32_MODE
        if (isRunOver())
        {
#if (DRAW_TEST == 1)
            if (isDrawOver())
                break;
#else

            //runButtonAPI(0, TRUE);

            break;
#endif
        }
#endif
    }
    //while (1)//g_SystemStatus != SYS_IDLE
    //{
    //    GCodeRunLoopAPI();
    //}

#if !STM32_MODE
    for (i = 0; i < CHANNEL_NUM_MAX; i++)
    {
        GCODE_CLOSE_FILE(i); //关闭输出文件  测试用

        CloseFile(i);
    }

#endif
    system("pause");

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
