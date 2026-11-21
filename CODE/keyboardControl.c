#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include "common.h"
#include "API.h"
#include "Channel.h"
/*
空格      运行按钮
        暂停按钮


*/
void printInfo()
{
    uint8 ucAxisArray[5] = { AXIS_X, AXIS_Y,AXIS_Z,AXIS_A };//打印的轴号

    //获取实时点
    SPointDouble SRealTimePoint;
    getRealTimePoint(&SRealTimePoint, AXIS_NUM_MAX, g_SChannel->pMapAxisArray, NULL);

    printf("\rX:%.3f  Z:%.3f", SRealTimePoint.dArray[0], SRealTimePoint.dArray[2]);

    // channelPrint(&g_SChannel[0], ucAxisArray, 3, -1);
    fflush(stdout);            // 刷新缓冲区，确保立即输出
    Sleep(1);                 // 延时 1 毫秒

}
void getKeyboard()
{

    printInfo();

    int8 cChar;
    //检测是否有按键输入
    if (_kbhit())
    {
        //1.获取按键值
        cChar = _getch();
        //2.分析按键值
        switch (cChar)
        {
        case ' ':
            runButtonAPI(0, TRUE);//启动0通道
            break;
        case 'p':
            pauseButtonAPI(0);//调用暂停函数
            break;
        case 's':
            singleLineButtonAPI(0);
            break;
        case 'r':
            resetButtonAPI(0);
            break;
        case 'h':
            simulaButtonAPI(0);
            break;
        case 75://左键：手轮反摇
            g_SChannel->SHandWheelTest.nTotalPulse -= 10;
            break;

        case 77://右键：手轮正摇
            g_SChannel->SHandWheelTest.nTotalPulse += 10;
            break;
        default:

            break;
        }
    }
}

