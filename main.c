#include <stdio.h>
#include <string.h>
#include <windows.h>

#include "Gcode.h"
#include "Channel.h"
#include "ErrorFunction.h"
#include "RunAPI.h"
#include "SysParaAPI.h"
#include "coordAPI.h"
#include "motionAPI.h"
#include "ToolCompensation.h"
#include "toolAxis.h"

FILE* pWriteFid;
extern uint32 timeji;

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

// 沿用原终端打印，显示周期、行号、XYZ 坐标和进给。
void channelPrint(struct SChannel* pSChannel, uint8* ucAxisArray, int nArraySize, int nCounter)
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

void channelPrintf(struct SChannel* pSChannel, uint8* ucAxisArray, int nArraySize, int nCounter)
{
    fprintf(pWriteFid, "%d,%u", nCounter, pSChannel->SPreProcess.unGCodeLineNum + 1);
    for (int i = 0; i < nArraySize; i++)
        fprintf(pWriteFid, ",%.9f", getMachCoordDouble(pSChannel->ucChannelNum, ucAxisArray[i]));
    fprintf(pWriteFid, "\n");
}

DWORD WINAPI PROCESST1(LPVOID param)
{
    int counter = 0;
    uint8 ucAxisArray[3] = {AXIS_X, AXIS_Y, AXIS_Z};
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
            interruptFuncAPI(i);
            channelPrint(&g_SChannel[i], ucAxisArray, 3, counter);
            channelPrintf(&g_SChannel[i], ucAxisArray, 3, counter);
        }
        printf("\n");
        timeji++;
        machineTimerInterp(0);
    }
    return 0;
}

int main(void)
{
    HANDLE IRQhandle;
    DWORD result = 4;

    SetConsoleOutputCP(CP_UTF8); // 控制台按 UTF-8 显示，与工程编译编码一致。

    pWriteFid = fopen("./outputFile/test.txt", "w");
    if (!pWriteFid)
        return 1;
    fprintf(pWriteFid, "cycle,program_line,commanded_x_mm,commanded_y_mm,commanded_z_mm\n");
    GCODE_OPEN_FILE(0);

    g_ucNameType = 1;
    strcpy(g_cName_C0, "0001"); // 按原方式读取 ./resource/O0001.txt。

    initMapArray();
    initEncoderMapArray();
    initSystemPara();
    initAxisPara();
    struct SEncoder* pEncoderArrays[ENCODER_NUM_MAX];
    for (int i = 0; i < ENCODER_NUM_MAX; i++)
    {
        pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[0][i] == ENCODER_NULL ||
                             g_SSystemPara.ucEncoderMapArray[0][i] > ENCODER_NUM_MAX - 1)
                                ? NULL
                                : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[0][i]]);
    }
    initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], g_SSystemPara.ucPWMAxisMapArray[0],
                pEncoderArrays); // 通道初始化，沿用原桌面入口顺序。
    initToolAxis(&g_SChannel[0], &g_SChannel[0].SSingleAxisArray[TOOL_AXIS]);
    initEditablePara();
    initAfterLoadSysPara(); // 加载系统参数后执行的初始化。
    initErrorBuf();

    g_RunMode = AUTOMATIC_MODE;
    if (runButtonAPI(0, FALSE) == 1)
    {
        IRQhandle = CreateThread(NULL, 0, PROCESST1, NULL, 0, NULL);
        if (IRQhandle)
        {
            while (1)
            {
                GCodeRunLoopAPI();
                if (isRunOver() || isErrorHasBeenWritten())
                    break;
            }
            WaitForSingleObject(IRQhandle, INFINITE);
            GetExitCodeThread(IRQhandle, &result);
            CloseHandle(IRQhandle);
        }
    }
    if (isErrorHasBeenWritten())
        printError();
    GCODE_CLOSE_FILE(0);
    CloseFile(0);
    fclose(pWriteFid);
    return (int)result;
}
