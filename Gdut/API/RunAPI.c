#include "RunAPI.h"
#include "Channel.h"
#include <stdlib.h>
#include <math.h>
#include "Config.h"
#include "GeneralFunctions.h"
#include "Motion.h"
#include "common.h"
#include "CTrajPlanning.h"
#include "ErrorFunction.h"
#include "Gcode.h"
#include "GeneralFunctions.h"
#include "lookAhead.h"
#include "ToolCompensation.h"
#include <stdlib.h>
#include "IOCommand.h"
#include "ToolChanger.h"
#include "Spindle.h"
#include "toolAxis.h"
#include "G32.h"
#include "G1.h"
#include "G0.h"
#include <string.h>
#include "G2.h"
#include "Channel.h"
#include "Config.h"
#include "ErrorCompensation.h"
#include "SecondSpindle.h"
#include "Handwheel.h"
#include "ringbuff.h"
#if HANDWHEEL_SIM_DISTANCE_ENABLE
enum HANDWHEEL_SIM_REQUEST
{
    HANDWHEEL_SIM_NONE = 0,         /**< 没有交接请求。 */
    HANDWHEEL_SIM_ENABLE_REQUESTED, /**< 前台请求开启，尚未判断续行资格。 */
    HANDWHEEL_SIM_ENABLE_HOLD,      /**< 等原暂停完成再开启，但不得自动续行。 */
    HANDWHEEL_SIM_ENABLE_RESUME,    /**< 为切入模拟暂停，交接后申请恢复。 */
    HANDWHEEL_SIM_CLOSE_REQUESTED,  /**< 前台请求关闭，尚未判断续行资格。 */
    HANDWHEEL_SIM_CLOSE_HOLD,       /**< 关闭后保持暂停，重复按键不能升级。 */
    HANDWHEEL_SIM_CLOSE_RESUME,     /**< 正常退出，交接后申请恢复自动加工。 */
    HANDWHEEL_SIM_STOP_CLOSE        /**< 复位/报警/急停后的关闭，等待原停车收尾。 */
};
/* 按通道保存唯一开关请求；前台串行发布单字节，中断接收并完成运动交接。
 * 上电为 NONE；暂停、模式变化撤销续行；STOP_CLOSE 不被普通按键覆盖。
 * 不使用 BUILDING 让旧测速和 handleSystemPause 在交接前改变倍率或自动启动。 */
static volatile uint8 g_ucHandwheelSimRequestArray[CHANNEL_NUM_MAX];
#endif
#if STM32_MODE
#include "app_draw.h"
extern uint8 SpindleB;
#endif
#if !STM32_MODE
uint8 out[30];  //企业的输出IO数组,与企业同名
uint8 SpindleB; //企业手轮模式的名称,与企业同名
#endif
/*************************************************
  函数名称：pauseButtonAPI(uint8 ucChannelNum)
  函数功能  暂停按键接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1 执行完成
			0 非运行状态 不能暂停
			-1 通道号错误
  备    注：
****************************************/
int8 pauseButtonAPI(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX) //错误的通道号
    {
        return -1;
    }
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* g_ucHandwheelSimRequestArray 真实暂停取消尚未完成的开启，保留关闭并撤销自动续行。
     * 恢复运动不在这里强改状态；原恢复完成后，由退出处理请求正常暂停。 */
    if (g_ucHandwheelSimRequestArray[ucChannelNum] != HANDWHEEL_SIM_NONE &&
        g_ucHandwheelSimRequestArray[ucChannelNum] != HANDWHEEL_SIM_STOP_CLOSE)
    {
        g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_CLOSE_HOLD;
    }
#endif
    g_SChannel[ucChannelNum].bChipBreakStart = FALSE;        //开始断屑标志位置FALSE
    if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_RUN) //只有在运行状态下才能暂停
    {
        g_SChannel[ucChannelNum].ucChannelStatus = SYS_RUN_TO_PAUSE; //将系统状态置为RUN_TO_PAUSE
        return 1;
    }
    if (g_SChannel[ucChannelNum].bProgramStop == FALSE) //M01执行期间按下程序暂停  2023/10/7 洪冠鹏
    {
        for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE; ucounter++)
        {
            if (g_SChannel[ucChannelNum].SIOCommand.ucIORunBuff[ucounter] == 1 &&
                g_SChannel[ucChannelNum].SIOCommand.SVaribleIOInfoArray[ucounter].ucStatus == VARIABLE_IO_DETECT)
            {
                g_SChannel[ucChannelNum].bProgramStop = TRUE;
            }
        }
    }

    return 0;
}
#define RESET_SECONDTIMES_STOP_NOW 0
/*************************************************
  函数名称：resetButtonAPI(uint8 ucChannelNum)
  函数功能  复位按键接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1 设置完成
			-1 错误通道号
  备    注：
****************************************/
int8 resetButtonAPI(uint8 ucChannelNum)
{
    bool bAlarmStop = FALSE;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //错误的通道号
    {
        return -1;
    }

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* g_ucHandwheelSimRequestArray 记录保护后关闭；原停车仍负责速度交接及轴释放。
     * 即使下面因报警提前返回，也不能丢失关闭意图或留下自动续行请求。 */
    g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_STOP_CLOSE;
#endif
    switch (g_SChannel[ucChannelNum].ucChannelStatus)
    {
    case SYS_RESET:                                          //本来就处于复位状态
#if RESET_SECONDTIMES_STOP_NOW == 1                          //陈敏聪	8/17
        g_SChannel[ucChannelNum].SPreProcess.ucRunStep += 1; // 立即停
#endif
        break;
    case SYS_EMERGENCY_STOP: //2025/9/9 在急停状态下，按下复位按键不修改为复位状态
        break;
    default: //减速停
#if STM32_MODE
        if (Check_bootwarning() == 0) // 有伺服/主轴/限位/超时报警
        {
            bAlarmStop = TRUE;
        } //有报警则退出函数，监控企业管理的主轴报警、伺服报警、超时报警、限位报警
#endif
        if ((bAlarmStop || isErrorHasBeenWritten() == TRUE) &&
            g_SChannel[ucChannelNum].ucChannelStatus != SYS_IDLE) //有报警则退出函数，监控学校管理的报警
        {
            return 1; //有错误优先完成报警轴停运动 //增加判断2026.5.7
        }
        g_SChannel[ucChannelNum].cSubRunMode = 1;

        g_SChannel[ucChannelNum].SPreProcess.ucRunStep = 0;
        g_SChannel[ucChannelNum].ucChannelStatus = SYS_RESET;
        //if (ucChannelNum == 0)//企业要求，复位关闭主轴关闭冷却液功能仅在通道0生效   洪冠鹏 2024/6/18  //2024/7/25 企业取消仅复位通道0的要求
        //{
        g_SChannel[ucChannelNum].bIsStopSpindleAndCoolantwhileReset =
            g_SSystemPara.bIsStopSpindleAndCoolantwhileReset; //标志位在按下复位按键后从系统参数获取
        //}

        break;
    }
    return 1;
}

/*************************************************
  函数名称：runButtonAPI(uint8 ucChannelNum)
  函数功能  运行按键接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值： 1 执行成功
            0 手轮交接未完成，调用方须等待
			-1 错误的系统状态
			-2 上一次G代码运行还未结束
			-3 错误通道号
			-4 当前通道不满足运行条件
  备    注：
****************************************/
int8 runButtonAPI(uint8 ucChannelNum, uint8 ucIsGetBeforeProgram)
{
    int16 sCounter;
    if (ucChannelNum >= CHANNEL_NUM_MAX) //错误的通道号
    {
        return -3;
    }

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* g_ucHandwheelSimRequestArray 的交接尚未完成时，外部启动不得越过暂停回写。
     * 中断完成交接会先清请求，再调用本入口；返回 0 表示等待，不是已启动。 */
    if (g_ucHandwheelSimRequestArray[ucChannelNum] != HANDWHEEL_SIM_NONE)
    {
        return 0;
    }
#endif
    if (!judgeRunCondition(ucChannelNum)) //判断当前通道环境是否能运行	陈敏聪 2023/6/15
    {
        return -4;
    }

    switch (g_SChannel[ucChannelNum].ucChannelStatus)
    {
    case SYS_IDLE: //空闲状态下按启动  初始化 进行一部分解析、刀补 速度前瞻 再启动插补（置RUN）
    {
        //setSpindleEnabledAPI(ucChannelNum, TRUE);//使能主轴
        //初始化计件
        initWorkpieceNum(ucChannelNum);
        //setShowByDiameter(g_SChannel[ucChannelNum].pMapAxisArray[AXIS_X], TRUE);//X轴直径显示  //洪冠鹏注释 2024/5/20 显示方式由系统参数设定

        g_SChannel[ucChannelNum].SSpindle.toDisableAxis = FALSE; //轴停下 无动作

        if (g_RunMode != AUTOMATIC_MODE && g_RunMode != MDI_MODE && g_RunMode != HAND_MODE)
            return -1;
        if (g_SChannel[ucChannelNum].SIOCommand.ucIOBlockedCounter != 0)
            return -2;

        //g_SChannel[ucChannelNum].ucChannelStatus = SYS_PREPARE;//将系统状态置为RUN

        //1.G代码解析模块在runButtonAPI()中需要执行的内容
        //1.1G代码解析初始化
        gcodeInit(ucChannelNum, TRUE, ucIsGetBeforeProgram);

        //1.2所有通道的通道结构体中添加主运行程序
        if (addMainProgramNode(ucChannelNum) != 1)
        {
            g_SChannel[ucChannelNum].ucChannelStatus = SYS_IDLE;
            g_SChannel[ucChannelNum].cSubRunMode = 1; //0 画图 1 运动	罗祥玮20250528
            return -1;
        }

#if STM32_MODE
        prepareProgramBeginLine(ucChannelNum);
#endif

        //1.3G代码解析状态置为GCODE_RUN
        changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN, TRUE, 0); //启动对应的通道
        changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, 0); //启动对应的通道

        //2.刀补计算模块在runButtonAPI()中需要执行的内容
        toolCompensationInit(ucChannelNum); //刀补初始化

        lookAheadInit(&g_SChannel[ucChannelNum]); //速度前瞻初始化

        interpolationInit(&g_SChannel[ucChannelNum].SPreProcess); //插补初始化

        initAllBuffer(ucChannelNum); //复位  把缓冲区初始化

        //陈敏聪	2023/9/1	陈敏聪注释，提前运行G代码会让G代码状态机混乱
        //for (sCounter = 0; sCounter <= 10; sCounter++)//在IDLE状态下 运行6次G代码刀补和前瞻
        //{
        //	//assignSegments(NULL, &g_GcodeRelatedObj.unLineNum, g_GcodeRelatedObj.unEndLineNum);  //一行G代码赋值到解析结构体
        //	if (mainFuncGCode(ucChannelNum))
        //	{
        //		return -1;
        //	}
        //	ToolCompensation_Caculate(ucChannelNum);//刀补计算
        //	velocityLookAhead(&g_SChannel[ucChannelNum]);//速度前瞻
        //}
        g_SChannel[ucChannelNum].ucChannelStatus = SYS_RUN;
        g_SChannel[ucChannelNum].cSubRunMode = 1; //0 画图 1 运动	罗祥玮20250528
    }

    case SYS_RUN:
        if (g_SChannel[ucChannelNum].cSubRunMode == 0) //画图与运动互斥
            return -1;
        break;

    case SYS_PAUSE: //暂停状态下按启动 将系统状态改为PAUSE_TO_RUN

        if (g_SChannel[ucChannelNum].bSingleLineMode == TRUE) //M01单节执行的处理  2024/4/27 洪冠鹏
        {
            for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE; ucounter++)
            {
                if (g_SChannel[ucChannelNum].SIOCommand.ucIORunBuff[ucounter] == 1 &&
                    g_SChannel[ucChannelNum].SIOCommand.ucIOBlockedCounter != 0)
                {
                    if (g_SChannel[ucChannelNum].SIOCommand.SVaribleIOInfoArray[ucounter].ucStatus ==
                        VARIABLE_IO_FINISH) //M01端口有输入，处于完成状态时按运行停在当前行
                    {
                        g_SChannel[ucChannelNum].ucRunCounter = 0;
                        g_SChannel[ucChannelNum].ucChannelStatus = SYS_RUN;
                    }
                    else //M01端口无输入，处于暂停检测状态时按运行无效
                    {
                        return 1;
                    }
                }
            }
        }
        if (g_SChannel[ucChannelNum].bProgramStop == TRUE) //M01执行期间按下程序暂停后的重启  2023/10/9 洪冠鹏
        {
            for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE; ucounter++)
            {
                if (g_SChannel[ucChannelNum].SIOCommand.ucIORunBuff[ucounter] == 1 &&
                    g_SChannel[ucChannelNum].SIOCommand.SVaribleIOInfoArray[ucounter].ucStatus == VARIABLE_IO_WAIT)
                {
                    g_SChannel[ucChannelNum].bProgramStop = FALSE;
                    return 1;
                }
            }
        }
        for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE;
             ucounter++) //M01执行期间按下程序启动键无效，保持继续执行  2023/10/11 洪冠鹏
        {
            if (g_SChannel[ucChannelNum].SIOCommand.ucIORunBuff[ucounter] == 1 &&
                g_SChannel[ucChannelNum].SIOCommand.ucIOBlockedCounter != 0 &&
                g_SChannel[ucChannelNum].SIOCommand.SVaribleIOInfoArray[ucounter].ucStatus != VARIABLE_IO_FINISH)
            {
                return 1;
            }
        }

        if (g_SChannel[ucChannelNum].bSelectableStop == TRUE) //选择停止（M01）  2023/9/25 洪冠鹏
        {
            for (int8 ucounter = 0; ucounter < IO_MAXGROUP_IN_SINGLE_LINE; ucounter++)
            {
                if (g_SChannel[ucChannelNum].SIOCommand.ucIORunBuff[ucounter] == 1 &&
                    g_SChannel[ucChannelNum].SIOCommand.SVaribleIOInfoArray[ucounter].ucStatus == VARIABLE_IO_WAIT)
                {
                    g_SChannel[ucChannelNum].ucChannelStatus = SYS_PAUSE;
                    return 1;
                }
            }
        }

        g_SChannel[ucChannelNum].ucChannelStatus = SYS_PAUSE_TO_RUN;

        break;

    case SYS_PAUSE_TO_RUN: //两种状态按RUN键无操作
    case SYS_RUN_TO_PAUSE:
        break;
    default:
        return -1;
    }

    //朱稳中 2023.5.19
    //if (g_SChannel[ucChannelNum].bSingleLineMode == TRUE || g_SChannel[ucChannelNum].bSelectableStop == TRUE)
    //{
    if (g_SChannel[ucChannelNum].bSingleLineModeUnFinished == FALSE)
    {
        g_SChannel[ucChannelNum].ucRunCounter = 1;
    }
    //}
    return 1;
}

#if HANDWHEEL_SIM_DISTANCE_ENABLE
/*************************************************
  函数名称：isHandwheelSimSelectedAPI
  函数功能：查询本通道已开启或仍待交接的模拟选择，供面板决定共同目标
  输入参数：ucChannelNum - 通道号，0~CHANNEL_NUM_MAX-1
  输出参数：无
  返 回 值：TRUE - 已选中或请求未收尾；FALSE - 已关闭或通道号无效
  备    注：只读共享状态；请求存在不代表已经接管运动或机械已停稳
****************************************/
bool isHandwheelSimSelectedAPI(uint8 ucChannelNum)
{
    /* g_SChannel 是实际选择，g_ucHandwheelSimRequestArray 包含尚未生效的开关意图。 */
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return FALSE;
    }
    return g_ucHandwheelSimRequestArray[ucChannelNum] != HANDWHEEL_SIM_NONE ||
           g_SChannel[ucChannelNum].cHandWheelSimulaStatus != HANWHEELSIMULA_OFF;
}

/*************************************************
  函数名称：isHandwheelSimSwitchPendingAPI
  函数功能：查询指定通道是否仍在交接手轮模拟开关，供启动流程区分等待与失败
  输入参数：ucChannelNum - 通道号，0~CHANNEL_NUM_MAX-1
  输出参数：无
  返 回 值：TRUE - 开关或保护关闭未完成；FALSE - 无待处理请求或通道号无效
  备    注：只读请求；不表示模拟是否开启，也不替代运行入口的动态检查
****************************************/
bool isHandwheelSimSwitchPendingAPI(uint8 ucChannelNum)
{
    /* g_ucHandwheelSimRequestArray 由前台提交、中断推进；此处不消费或清除请求。 */
    return ucChannelNum < CHANNEL_NUM_MAX && g_ucHandwheelSimRequestArray[ucChannelNum] != HANDWHEEL_SIM_NONE;
}

/*************************************************
  函数名称：setHandwheelSimEnabledAPI
  函数功能：提交明确开关意图；不在前台修改运动选择、倍率或暂停状态
  输入参数：ucChannelNum - 通道号；bEnabled - TRUE 开启，FALSE 关闭
  输出参数：无指针回写；更新本通道待处理请求
  返 回 值：-1 - 通道号无效；0 - 已关闭或拒绝开启；1 - 已选中或交接待处理
  备    注：前台串行调用；关闭优先于开启，复位/报警清理不得被按键覆盖
****************************************/
int8 setHandwheelSimEnabledAPI(uint8 ucChannelNum, bool bEnabled)
{
    /* g_SChannel 仅用于当前选择与拒绝检查；中断接收时再次判断动态条件。 */
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    /* 关闭只撤销开启目标，不能把切换自身引起的暂停当成用户暂停。
     * 已被真实暂停降为 HOLD 的请求仍保持 HOLD；重复关闭也不重授续行资格。 */
    if (!bEnabled)
    {
        if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_REQUESTED)
        {
            g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_CLOSE_REQUESTED;
            return 1;
        }
        if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_RESUME)
        {
            g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_CLOSE_RESUME;
            return 1;
        }
        if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_HOLD)
        {
            g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_CLOSE_HOLD;
            return 1;
        }
        if (g_ucHandwheelSimRequestArray[ucChannelNum] != HANDWHEEL_SIM_NONE)
        {
            return 1;
        }
        if (g_SChannel[ucChannelNum].cHandWheelSimulaStatus == HANWHEELSIMULA_OFF)
        {
            return 0;
        }
        g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_CLOSE_REQUESTED;
        return 1;
    }
    /* 故障、复位、急停和非加工模式拒绝新开启，不能以按键清除保护。 */
    if ((g_RunMode != AUTOMATIC_MODE && g_RunMode != MDI_MODE) ||
        g_SChannel[ucChannelNum].ucChannelStatus == SYS_RESET ||
        g_SChannel[ucChannelNum].ucChannelStatus == SYS_EMERGENCY_STOP ||
        g_SChannel[ucChannelNum].SHandwheelSim.bFault || isErrorHasBeenWritten()
#if STM32_MODE
        || Check_bootwarning() == 0
#endif
    )
    {
        return 0;
    }
    /* 重复开启不清账本、不改倍率，也不覆盖正在进行的关闭或停车交接。 */
    if (isHandwheelSimSelectedAPI(ucChannelNum))
    {
        return 1;
    }
    g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_ENABLE_REQUESTED;
    return 1;
}
#endif

/*************************************************
  函数名称：simulaButtonAPI
  函数功能：保留单通道切换入口；距离模拟共用明确开关请求实现
  输入参数：ucChannelNum - 通道号，0~CHANNEL_NUM_MAX-1
  输出参数：无指针回写；更新模拟选择或关闭请求
  返 回 值：-1 - 通道号错误；0 - 已关闭或拒绝开启；1 - 已选中或关闭待处理
  备    注：多通道共同按键使用明确设置接口；宏关闭时保留原行为
****************************************/
int8 simulaButtonAPI(uint8 ucChannelNum)
{
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* g_SChannel 提供当前模拟选择；关闭中的重复切换仍然表示关闭。 */
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    return setHandwheelSimEnabledAPI(ucChannelNum, !isHandwheelSimSelectedAPI(ucChannelNum));
#else
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }

    //switch (g_SChannel[ucChannelNum].ucChannelStatus)
    //{
    //case SYS_PAUSE:

    switch (g_SChannel[ucChannelNum].cHandWheelSimulaStatus)
    {
    case HANWHEELSIMULA_OFF:
        if (g_SChannel[ucChannelNum].ucChannelStatus ==
            SYS_RUN) //运行过程中按下手轮模等待暂停结束后手轮模拟的限制参数才生效  2023/12/21 洪冠鹏
        {
            g_SChannel[ucChannelNum].cHandWheelSimulaStatus = HANWHEELSIMULA_BUILDING; //手轮模拟功能正在建立
            pauseButtonAPI(ucChannelNum);
        }
        else //非运行状态下按下手轮模拟后，手轮模拟的限制参数立即生效
        {
            g_SChannel[ucChannelNum].cHandWheelSimulaStatus = HANWHEELSIMULA_ON;
        }
        break;
    case HANWHEELSIMULA_BUILDING:
    case HANWHEELSIMULA_ON:
    default:
        g_SChannel[ucChannelNum].cHandWheelSimulaStatus = HANWHEELSIMULA_OFF; //关闭手轮模拟功能
        break;
    }
    g_SChannel[ucChannelNum].SRun.dHandWheelSpeedRatio =
        (g_SChannel[ucChannelNum].cHandWheelSimulaStatus == HANWHEELSIMULA_OFF) ? 1.0 : 0.0; //非手轮模拟时  倍率为1
    g_SChannel[ucChannelNum].SPreProcess.cInterpDir = INP_GO_AHEAD;                          // 运动方向：向前运动
                                                                                             //	break;
                                                                                             //default:
                                                                                             //	return 0;
                                                                                             //}

    return g_SChannel[ucChannelNum].cHandWheelSimulaStatus != HANWHEELSIMULA_OFF ? 1 : 0;
#endif
}

/*************************************************
  函数名称：singleLineButtonAPI(uint8 ucChannelNum)
  函数功能  单节执行 接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1  执行成功
			-1 通道号错误
  备    注：
****************************************/
int8 singleLineButtonAPI(uint8 ucChannelNum) //zxr bSingleLineMode置为FALSE代表连续执行模式，TRUE代表单节执行
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    g_SChannel[ucChannelNum].bSingleLineModeUnFinished =
        FALSE; //洪冠鹏2024/5/20 初始化当前行未完成标志位FALSE  zxr FALSE代表没有正在执行、尚未结束的单节，TRUE代表当前单节已经开始，但还没结束
    g_SChannel[ucChannelNum].bSingleLineMode = g_SChannel[ucChannelNum].bSingleLineMode == TRUE ? FALSE : TRUE;
    if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_RUN) //洪冠鹏 2024/5/29 运行过程中触发单节执行立即生效
    {
        g_SChannel[ucChannelNum].ucRunCounter = 0;
    }
    return g_SChannel[ucChannelNum].bSingleLineMode == TRUE ? 1 : 0;
}

/*************************************************
  函数名称：selectableStopAPI(uint8 ucChannelNum)
  函数功能  选择停止接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1  执行成功
			-1 通道号错误
  备    注：
****************************************/
int8 selectableStopAPI(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    g_SChannel[ucChannelNum].bSelectableStop = g_SChannel[ucChannelNum].bSelectableStop == TRUE ? FALSE : TRUE;
    return g_SChannel[ucChannelNum].bSelectableStop == TRUE ? 1 : 0;
}

//急停 接口函数
int8 emergencyStopAPI(uint8 ucChannelNum) //2025.5.7熊骏
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* g_ucHandwheelSimRequestArray 记录保护后关闭；原停车仍负责速度交接及轴释放。
     * 即使下面因报警提前返回，也不能丢失关闭意图或留下自动续行请求。 */
    g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_STOP_CLOSE;
#endif
    switch (g_SChannel[ucChannelNum].ucChannelStatus)
    {
    case SYS_EMERGENCY_STOP: //本来就处于急停状态
        break;
    default:
        g_SChannel[ucChannelNum].SPreProcess.ucRunStep = 0;
        g_SChannel[ucChannelNum].ucChannelStatus = SYS_EMERGENCY_STOP;
        if (g_SSystemPara.bNowStopflag)
        {
            //清空XYZABC轴的脉冲增量缓存区 0011 1111
            initSRunPulseBuf(&g_SChannel[ucChannelNum].SRun, 0x3F);
        }
        parseRunningError(9999, GCODE_ERROR_NUM01, 9999);
        break;
    }
    return 1;
}

////系统准备状态 2023.4.3朱稳中
//int8 sysPrepare(uint8 ucChannelNum)
//{
//	int i;
//	struct SChannel* pSChannel;
//	if (ucChannelNum>= CHANNEL_NUM_MAX)
//	{
//		return -1;
//	}
//	pSChannel = &g_SChannel[ucChannelNum];
//	if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_PREPARE)
//	{//系统准备状态
//		//uint8 ucPhysicalAxis;//物理轴
//		////检查所有轴  都确实使能之后再启动
//		//for ( i = 0; i < AXIS_NUM_MAX; i++)
//		//{
//
//		//	ucPhysicalAxis = pSChannel->pMapAxisArray[i];
//		//	if (ucPhysicalAxis >= AXIS_NUM_MAX)//映射为空  不判断
//		//	{
//		//		continue;
//		//	}
//		//	if (((g_ucAxisExactEnable>> ucPhysicalAxis)& 0x01) != 0x01) //轴没有使能 不启动
//		//	{
//		//		return 0;
//		//	}
//		//}
//
//		gcodeInit(ucChannelNum);
//
//		initAllBuffer(ucChannelNum); //复位  把缓冲区初始化
//
//		toolCompensationInit(ucChannelNum);	//刀补初始化
//
//		lookAheadInit(&g_SChannel[ucChannelNum]);//速度前瞻初始化
//
//		interpolationInit(&g_SChannel[ucChannelNum].SPreProcess);//插补初始化
//
//
//
//		for (i = 0; i <= 10; i++)//在IDLE状态下 运行6次G代码刀补和前瞻
//		{
//			//parseALineToGCodeStruct(NULL, &g_GcodeRelatedObj.unLineNum, g_GcodeRelatedObj.unEndLineNum);  //一行G代码赋值到解析结构体
//			if (mainFuncGCode(ucChannelNum))
//			{
//				return -1;
//			}
//			ToolCompensation_Caculate(ucChannelNum);//刀补计算
//			velocityLookAhead(&g_SChannel[ucChannelNum]);//速度前瞻
//		}
//		pSChannel->ucChannelStatus = SYS_RUN;
//	}
//	return 1;
//}

/*************************************************
  函数名称：GCodeRunAPI(uint8 ucChannelNum)
  函数功能  运动接口函数
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：1  执行成功
			-1 通道号错误
			-2 语法检查不通过  不运行
  备    注：
****************************************/
int8 GCodeRunAPI(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX || g_SChannel[ucChannelNum].ucChannelStatus == SYS_RESET) //通道号错误
    {
        return -1;
    }
    if (isErrorHasBeenWritten() == TRUE) //语法检查错误  不执行
    {
        return -2;
    }

    //系统准备状态  2023.4.3 朱稳中
    //sysPrepare(ucChannelNum);

    switch (g_SChannel[ucChannelNum].ucChannelStatus)
    {
    case SYS_IDLE: //空闲
        runIOBuff(&g_SChannel[ucChannelNum].SIOCommand);
        runTIO(&(g_SChannel[ucChannelNum].STIOCommand));
        break;
    case SYS_EMERGENCY_STOP: //急停
        break;
    default:
        mainFuncGCode(ucChannelNum);             //G代码
        ToolCompensation_Caculate(ucChannelNum); //刀补计算
        if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_RUN ||
            g_SChannel[ucChannelNum].ucChannelStatus == SYS_PAUSE_TO_RUN)
        {
            velocityLookAhead(&g_SChannel[ucChannelNum]); //速度前瞻
        }
        runIOBuff(&g_SChannel[ucChannelNum].SIOCommand);
        runTIO(&(g_SChannel[ucChannelNum].STIOCommand));
        break;
    }

    //if (g_SChannel[ucChannelNum].ucChannelStatus != SYS_IDLE)
    //{
    //	mainFuncGCode(ucChannelNum);
    //}

    //if (g_SChannel[ucChannelNum].ucChannelStatus != SYS_IDLE)
    //{
    //	ToolCompensation_Caculate(ucChannelNum);//刀补计算
    //}

    //if (g_SChannel[ucChannelNum].ucChannelStatus == SYS_RUN || g_SChannel[ucChannelNum].ucChannelStatus == SYS_PAUSE_TO_RUN)//SYS_PAUSE_TO_RUN进入速度前瞻将flag置TRUE
    //{
    //	velocityLookAhead(&g_SChannel[ucChannelNum]);//速度前瞻
    //}

    return 1;
}

/*************************************************
  函数名称：GCodeRunLoopAPI()
  函数功能  循环执行G代码  刀补  接口函数
  输入参数：
  输出参数：
  返 回 值：
  备    注：内部分为 运动的G代码解析 与 画图的G代码解析
****************************************/
void GCodeRunLoopAPI()
{
    int i;
    for (i = 0; i < CHANNEL_NUM_MAX; i++)
    {
        //运行画图时，执行画图G代码解析
        if (g_SChannel[i].cSubRunMode == 0) //
        {
            if (ioEMERGENCY_STOP == !g_SSystemPara.ucEmergencyStopType)
            { //急停判断//朱稳中 2023.6.24
                emergencyStopAPI(g_SChannel[i].ucChannelNum);
            }
            //译码
            for (int j = 0; j < 20; j++) //画图过慢，增加译码循环次数
            {
                if (isErrorHasBeenWritten() == TRUE) //语法检查错误  不执行
                {
                    return;
                }
                mainFuncGCodeAuto(i);
            }
            //刀补
            ToolCompensation_Caculate(i);
        }
        //运行运动时，执行运动G代码解析
        else
        {
            GCodeRunAPI(i);
        }
    }
}
//模拟编码器脉冲波动
int wave(int center, int range)
{
    int min = center - range;
    int max = center + range;

    // 生成两个随机数取平均，提高中间集中概率
    int r1 = rand() % (max - min + 1) + min;
    int r2 = rand() % (max - min + 1) + min;
    return (r1 + r2) / 2;
}
/*************************************************
  函数名称：simulateGetEncoder
  函数功能：按上一拍已提交的主轴脉冲生成 VS 理想编码器反馈
  输入参数：pSChannel - 已初始化的通道，主轴与编码器映射须有效
  输出参数：更新主轴编码器方向、增量、累计计数及亚计数余量
  返 回 值：无；无效映射或数值沿原运行报警接口处理
  备    注：在本拍清脉冲缓存前调用一次；不代表实物伺服动态或 Renode 反馈
****************************************/

void simulateGetEncoder(struct SChannel* pSChannel)
{
#if !STM32_MODE
    struct SEncoder* pEncoder; /**< 引用通道绑定的共享编码器，由本周期写入反馈。 */
    uint8 ucSpindleAxis, ucPhysicalAxis;
    double dPulseToDegree, dEncoderCounts, dWholeCounts;
    int nEncoderCounts;

    /* 通道及主轴对象必须已绑定，禁止由空指针推测反馈对象。 */
    if (pSChannel == NULL || pSChannel->pSSpindleEncoder == NULL || pSChannel->SSpindle.pSSpindleAxis == NULL ||
        pSChannel->pMapAxisArray == NULL)
    {
        parseRunningError(9999, GCODE_ERROR_NUM300, 9999);
        return;
    }
    ucSpindleAxis = pSChannel->SSpindle.pSSpindleAxis->ucAxis;
    /* 先检查通道和逻辑轴编号，再访问脉冲数组与物理映射。 */
    if (pSChannel->ucChannelNum >= CHANNEL_NUM_MAX || ucSpindleAxis >= AXIS_NUM_MAX)
    {
        parseRunningError(pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
        return;
    }
    pEncoder = pSChannel->pSSpindleEncoder;
    pEncoder->cEncoderDir = 0;
    pEncoder->usDelta = 0;
    ucPhysicalAxis = pSChannel->pMapAxisArray[ucSpindleAxis];
    /* Y 轴辅助通道允许不配置主轴；没有输出时保持静止，不访问 AXIS_NULL 参数。 */
    if (ucPhysicalAxis == AXIS_NULL && pSChannel->SRun.sAxisDeltaPulse[ucSpindleAxis] == 0)
    {
        /* 保留该编码器原有机械相位及亚计数余量。 */
    }
    else
    {
        /* 未映射轴不得产生脉冲，其他越界映射同样沿参数报警路径退出。 */
        if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
        {
            parseRunningError(pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
            return;
        }
        /* 只读系统中的物理轴角度当量；sAxisDeltaPulse 此时仍是上一拍提交值。 */
        dPulseToDegree = g_SSystemPara.SAxisPara[ucPhysicalAxis].dPulseToMm;
        /* 主轴须使用有效角度当量及每圈计数，余量不可包含非有限数值。 */
        if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType != ROTATION_AXIS || !isfinite(dPulseToDegree) ||
            dPulseToDegree <= 0.0 || pEncoder->usEncoderCirclePulseNum == 0 ||
            !isfinite(pEncoder->m_dSimCountRemainder))
        {
            parseRunningError(pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
            return;
        }
        dEncoderCounts = pSChannel->SRun.sAxisDeltaPulse[ucSpindleAxis] * dPulseToDegree *
                             pEncoder->usEncoderCirclePulseNum / DEGREE_OF_CIRCLE +
                         pEncoder->m_dSimCountRemainder;
        /* 固定半计数取整方向，避免静止时 +/-0.5 余量交替产生虚假反馈。 */
        dWholeCounts = floor(dEncoderCounts + 0.5);
        /* 编码器增量字段为 uint16；先检查再转换，禁止回绕伪造反馈。 */
        if (!isfinite(dWholeCounts) || fabs(dWholeCounts) > MAXIMUM_OF_UINT16)
        {
            parseRunningError(pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM153, 9999);
            return;
        }
        nEncoderCounts = (int)dWholeCounts;
        pEncoder->m_dSimCountRemainder = dEncoderCounts - dWholeCounts;
        pEncoder->cEncoderDir = sgn(nEncoderCounts);
        pEncoder->usDelta = (uint16)abs(nEncoderCounts);
        pEncoder->lEncoderAbsValue += nEncoderCounts;
    }
#else
    uint8 ucSpindleAxis = pSChannel->SSpindle.pSSpindleAxis->ucAxis;
    pSChannel->pSSpindleEncoder->cEncoderDir = sgn(pSChannel->SRun.sLastDeltaPulse[ucSpindleAxis]);
    pSChannel->pSSpindleEncoder->usDelta = abs(pSChannel->SRun.sLastDeltaPulse[ucSpindleAxis]);
    pSChannel->pSSpindleEncoder->lEncoderAbsValue += pSChannel->SRun.sLastDeltaPulse[ucSpindleAxis];
#endif

    //刀具轴
#if TOOLAXIS_TEST
    g_SEncoder[1].cEncoderDir = sgn(pSChannel->SRun.sLastDeltaPulse[g_SToolAxisInfo.pSToolAxis->ucAxis]);
    g_SEncoder[1].lEncoderAbsValue += pSChannel->SRun.sLastDeltaPulse[g_SToolAxisInfo.pSToolAxis->ucAxis];
    g_SEncoder[1].usDelta = abs(pSChannel->SRun.sLastDeltaPulse[g_SToolAxisInfo.pSToolAxis->ucAxis]);

    //记录上一周期脉冲数
#endif
#if STM32_MODE || TOOLAXIS_TEST
    /* 保留旧板端和刀具轴测试的缓存行为；普通 VS 路径由 initSRunPulseBuf 维护。 */
    for (int i = 0; i < AXIS_NUM_MAX; i++)
    {
        pSChannel->SRun.sLastDeltaPulse[i] = pSChannel->SRun.sAxisDeltaPulse[i];
    }
#endif
}

#if HANDWHEEL_SIM_DISTANCE_ENABLE
/*************************************************
  函数名称：processHandwheelSimRequest
  函数功能：在原运动中断推进开关交接，并在保护停车后清除模拟选择
  输入参数：ucChannelNum - 已校验通道号，0~CHANNEL_NUM_MAX-1
  输出参数：无指针回写；更新通道选择、倍率、请求，必要时申请原运行入口
  返 回 值：无
  备    注：原调度前后调用；不执行插补、不抢占轴、不改变定时器或清除报警
****************************************/
static void processHandwheelSimRequest(uint8 ucChannelNum)
{
    /* pChannel 引用 g_SChannel；由运动周期消费请求，调用方按原 API 调度串行提交。 */
    struct SChannel* pChannel = &g_SChannel[ucChannelNum];
    uint32 unProgram, unMotion;
    bool bEnabled, bResume;
    /* 保护取消开关及续行，但保留原运动选择让前面的采样入口先交接停车速度。 */
    if (pChannel->ucChannelStatus == SYS_RESET || pChannel->ucChannelStatus == SYS_EMERGENCY_STOP ||
        pChannel->SHandwheelSim.bFault || isErrorHasBeenWritten()
#if STM32_MODE
        || Check_bootwarning() == 0
#endif
    )
    {
        if (isHandwheelSimSelectedAPI(ucChannelNum))
        {
            g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_STOP_CLOSE;
        }
    }
    /* 不通过修改状态伪造停车完成；原复位/报警路径进入空闲或急停收尾后才清理。 */
    if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_STOP_CLOSE)
    {
        if ((pChannel->ucChannelStatus != SYS_IDLE &&
             !(pChannel->ucChannelStatus == SYS_EMERGENCY_STOP && pChannel->SPreProcess.ucRunStep == 2)) ||
            pChannel->SG00.ucCurrentSigned != pChannel->SG00.ucFinishedSigned ||
            pChannel->SHandwheelSim.bInstructionBound || pChannel->SHandwheelSim.SPath.bBound ||
            pChannel->SHandwheelSim.SPath.bPreparing || !isHandwheelSimGroupStopped(&pChannel->SHandwheelSim))
        {
            return;
        }
        resetHandwheelSimGroup(&pChannel->SHandwheelSim);
        pChannel->SHandwheelSim.bAwaitingFreshInput = TRUE;
        pChannel->SRun.dHandWheelSpeedRatio = 1.0;
        pChannel->SPreProcess.cInterpDir = INP_GO_AHEAD;
        pChannel->cHandWheelSimulaStatus = HANWHEELSIMULA_OFF;
        g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_NONE;
        return;
    }
    /* 没有意图时不介入原运行。 */
    if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_NONE)
    {
        return;
    }
    /* 开启只在本次原运行/恢复需要暂停时允许续行；既有减速或暂停保持暂停。 */
    if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_REQUESTED)
    {
        bResume = pChannel->ucChannelStatus == SYS_RUN || pChannel->ucChannelStatus == SYS_PAUSE_TO_RUN;
        g_ucHandwheelSimRequestArray[ucChannelNum] = bResume ? HANDWHEEL_SIM_ENABLE_RESUME : HANDWHEEL_SIM_ENABLE_HOLD;
    }
    /* 关闭的续行资格只判一次；重复关闭不会重新赋予资格。 */
    if (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_CLOSE_REQUESTED)
    {
        /* 运行中的自动旁路段、尚未生效就取消的开启也应交接后续行。
         * 是否允许续行由原运行状态及下方保护决定，不取决于本段插补算法。 */
        bResume = pChannel->ucChannelStatus == SYS_RUN || pChannel->ucChannelStatus == SYS_PAUSE_TO_RUN;
        g_ucHandwheelSimRequestArray[ucChannelNum] = bResume ? HANDWHEEL_SIM_CLOSE_RESUME : HANDWHEEL_SIM_CLOSE_HOLD;
    }
    /* 模式变化取消开启和续行，仅完成必要关闭。 */
    if (g_RunMode != AUTOMATIC_MODE && g_RunMode != MDI_MODE)
    {
        g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_CLOSE_HOLD;
    }
    bEnabled = g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_HOLD ||
               g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_RESUME;
    unMotion = getBits(pChannel->SRun.unCommandType, CMD_MOTION_ADDR);
    unProgram = getBits(pChannel->SPreProcess.unCommandType, CMD_PROGRAM_ADDR);
    /* 消费暂停/程序结束前锁定不续行；倍率后来恢复也不能自动解除。 */
    if (pChannel->ucChannelStatus == SYS_IDLE || pChannel->SRun.cSoftLimitedOn != 0 || pChannel->bProgramStop ||
        (pChannel->bSingleLineMode && pChannel->ucRunCounter == 0) ||
        (getBits(pChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_QUICKMOVE
             ? (!isfinite(pChannel->SRun.dQuickMoveFeedRatio) || pChannel->SRun.dQuickMoveFeedRatio <= 0.0)
             : (!isfinite(pChannel->SRun.dCommonFeedRatio) || pChannel->SRun.dCommonFeedRatio <= 0.0)) ||
        ((pChannel->SPreProcess.ucInpStatus == INP_FINISHED || pChannel->SPreProcess.ucInpStatus == INP_PAUSING) &&
         (unProgram == CMD_PROGRAM_PAUSE || unProgram == CMD_PROGRAM_OPTION_PAUSE || unProgram == CMD_PROGRAM_STOP ||
          unProgram == CMD_PROGRAM_STOP_M02)))
    {
        g_ucHandwheelSimRequestArray[ucChannelNum] = bEnabled ? HANDWHEEL_SIM_ENABLE_HOLD : HANDWHEEL_SIM_CLOSE_HOLD;
    }
    /* 恢复运动不能中途改倍率或选择；完成后再沿正常暂停路径交接。 */
    if (pChannel->ucChannelStatus == SYS_PAUSE_TO_RUN)
    {
        return;
    }
    /* 开启前仍保持 OFF 与原自动倍率；关闭前仍保持原距离控制，分别沿原路径减速。 */
    if (pChannel->ucChannelStatus == SYS_RUN)
    {
        pChannel->bChipBreakStart = FALSE;
        pChannel->ucChannelStatus = SYS_RUN_TO_PAUSE;
        return;
    }
    /* 等暂停回写和解绑；手动 G0 的 3 为运动完成，5 为初始化/复位后的空闲。
     * channelStopToIDLE 不重置预处理插补标志；仅复位收尾的完整空闲终态可不等该旧值。
     * 不清 bIsUserData，不伪造插补完成，保留手动移动后的返回点语义。 */
    if ((pChannel->ucChannelStatus != SYS_PAUSE && pChannel->ucChannelStatus != SYS_IDLE) ||
        (pChannel->SPreProcess.ucInpStatus != INP_FINISHED &&
         !(pChannel->ucChannelStatus == SYS_IDLE && pChannel->SPreProcess.ucRunStep == 0 &&
           pChannel->SG00.ucRunStep == 5 && pChannel->SG00.ucCurrentSigned == pChannel->SG00.ucFinishedSigned &&
           pChannel->SG01.ucInpStatus == INP_FINISHED && pChannel->SG02.ucInpStatus == INP_FINISHED)) ||
        pChannel->SHandwheelSim.bInstructionBound || pChannel->SHandwheelSim.SPath.bBound ||
        pChannel->SHandwheelSim.SPath.bPreparing ||
        (pChannel->SG00.bIsUserData && ((pChannel->SG00.ucRunStep != 3 && pChannel->SG00.ucRunStep != 5) ||
                                        pChannel->SG00.ucCurrentSigned != pChannel->SG00.ucFinishedSigned)) ||
        (bEnabled && unMotion == CMD_MOTION_LINE && pChannel->SG01.ucInpStatus != INP_FINISHED) ||
        (bEnabled && (unMotion == CMD_MOTION_ARC || unMotion == CMD_MOTION_ANTI_ARC) &&
         pChannel->SG02.ucInpStatus != INP_FINISHED) ||
        !isHandwheelSimGroupStopped(&pChannel->SHandwheelSim))
    {
        return;
    }
    bResume = pChannel->ucChannelStatus == SYS_PAUSE &&
              (g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_ENABLE_RESUME ||
               g_ucHandwheelSimRequestArray[ucChannelNum] == HANDWHEEL_SIM_CLOSE_RESUME);
    /* 手动移动后的关闭不自动返回加工点；完成关闭后再由显式启动恢复。 */
    if (!bEnabled && bResume && pChannel->SG00.bIsUserData)
    {
        bResume = FALSE;
    }
    leaveHandwheelSim(pChannel);
    /* 交接函数失败保留保护，由下一拍原报警停车接管，不伪装开关成功。 */
    if (pChannel->SHandwheelSim.bFault || isErrorHasBeenWritten())
    {
        g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_STOP_CLOSE;
        return;
    }
    /* 原插补已完成暂停，末拍平均速度不作为新距离段起速；不清活动轴或末拍脉冲。
     * 非有限/负值仍交给原绑定校验报错，不能借切换掩盖非法数值。 */
    if (bEnabled && pChannel->ucChannelStatus == SYS_PAUSE &&
        (unMotion == CMD_MOTION_QUICKMOVE || unMotion == CMD_MOTION_LINE || unMotion == CMD_MOTION_ARC ||
         unMotion == CMD_MOTION_ANTI_ARC) &&
        isfinite(pChannel->SRun.dCurrentSpeed) && pChannel->SRun.dCurrentSpeed >= 0.0)
    {
        pChannel->SRun.dCurrentSpeed = 0.0;
    }
    pChannel->SRun.dHandWheelSpeedRatio = bEnabled ? 0.0 : 1.0;
    pChannel->SPreProcess.cInterpDir = INP_GO_AHEAD;
    pChannel->cHandWheelSimulaStatus = bEnabled ? HANWHEELSIMULA_ON : HANWHEELSIMULA_OFF;
    g_ucHandwheelSimRequestArray[ucChannelNum] = HANDWHEEL_SIM_NONE;
    /* 原运行入口再次检查 IO 与运行条件；拒绝后不自动重试。 */
    if (bResume)
    {
        runButtonAPI(ucChannelNum, TRUE);
    }
}
#endif

/*************************************************
  函数名称：interruptFuncAPI(uint8 ucChannelNum)
  函数功能  中断函数接口
  输入参数：ucChannelNum	通道号 0~CHANNEL_NUM_MAX-1
  输出参数：
  返 回 值：
  备    注：
****************************************/
void interruptFuncAPI(uint8 ucChannelNum)
{
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* 中断处理前记录恢复状态，仅在本拍刚完成恢复加工时，才允许清除恢复末拍的平均速度。 */
    bool bHandwheelRecovery = g_SChannel[ucChannelNum].cHandWheelSimulaStatus == HANWHEELSIMULA_ON &&
                              g_SChannel[ucChannelNum].ucChannelStatus == SYS_PAUSE_TO_RUN &&
                              g_SChannel[ucChannelNum].SG00.bIsUserData;
#endif

#if !ENCODER_MODE
    simulateGetEncoder(&g_SChannel[ucChannelNum]);
#endif

#if SECONDSPINDLE_USE_NOENCODER
    uint8 ucSpindleAxis = g_SChannel[ucChannelNum].SSecondSpindle.pSSpindleAxis->ucAxis;
    g_SChannel[ucChannelNum].pSSecondSpindleEncoder->cEncoderDir =
        sgn(g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[ucSpindleAxis]);
    g_SChannel[ucChannelNum].pSSecondSpindleEncoder->lEncoderAbsValue +=
        g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[ucSpindleAxis];
    g_SChannel[ucChannelNum].pSSecondSpindleEncoder->usDelta =
        abs(g_SChannel[ucChannelNum].SRun.sAxisDeltaPulse[ucSpindleAxis]);
#endif

#if HANDWHEEL_SIM_DISTANCE_ENABLE
    if (isHandwheelDistanceControlActive(&g_SChannel[ucChannelNum]))
    {
        /* 板端定时器已更新本拍样本；每通道只消费一次，不清零共享编码器。 */
        processHandwheelInput(&g_SChannel[ucChannelNum]);
    }
    else
    {
        leaveHandwheelSim(&g_SChannel[ucChannelNum]);
#endif
        softLimitForHandWheel(ucChannelNum);           //手轮软限位检测
        handWheelSpeedTest(&g_SChannel[ucChannelNum]); //手轮编码器速度检测
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    }
#endif
    spindleSpeedTest(&g_SChannel[ucChannelNum].SSpindleTest, &g_SChannel[ucChannelNum].SSpindle); //主轴编码器速度检测
    SecondspindleSpeedTest(&g_SChannel[ucChannelNum].SSecondSpindleTest,
                           &g_SChannel[ucChannelNum].SSecondSpindle); //第二转轴编码器速度检测
    resetProgram(&g_SChannel[ucChannelNum]);                          //复位运动
    stopAxisAfterError(&g_SChannel[ucChannelNum]);                    //报警后轴停
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    /* 在原调度消费程序暂停标记前撤销续行，保留 M0/M1 和程序结束保护。 */
    processHandwheelSimRequest(ucChannelNum);
#endif
    switch (g_RunMode)
    {
    case HANDWHEEL_MODE:
        if (g_SChannel[ucChannelNum].ucChannelStatus != SYS_RESET)
        {
            if (ucChannelNum ==
                SpindleB) //SpindleB是企业的通道选择名字，如果通道名和企业的通道选择名字对上才会走手轮单轴运动	陈敏聪	2023/7/21
                getHandWheelData(&g_SChannel[ucChannelNum]);
        }
        break;
    default:
        IRQHandler(&g_SChannel[ucChannelNum]); //中断服务函数
        break;
    }
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    clearHandwheelAverageSpeedAfterRecovery(&g_SChannel[ucChannelNum], bHandwheelRecovery);
    processHandwheelSimRequest(ucChannelNum);
#endif
    /* 苏向标 2026-10-14：停止先接收；后台主轴 IO 只在本周期执行。 */
    processSpindleRequests(&g_SChannel[ucChannelNum].SSpindle);
    processSpindleRequests(&g_SChannel[ucChannelNum].SSecondSpindle);
    processSpindleIORequest(&g_SChannel[ucChannelNum].SIOCommand);
    spindleMotion(&g_SChannel[ucChannelNum].SSpindle);             // 主轴运动
    SecondspindleMotion(&g_SChannel[ucChannelNum].SSecondSpindle); //第二轴/动力刀运动
#if TOOLAXIS_TEST
    toolAxisMotion(&g_SToolAxisInfo, &g_SChannel[ucChannelNum].SSpindle, g_SChannel[ucChannelNum].pSSpindleEncoder);
#endif
#if ERROR_COMP_TYPE == TWOWAY_SCREW_COMP //双向螺距误差补偿  在此处计算脉冲  主轴暂时没有补偿
    BilateralScrewComp(&g_SChannel[ucChannelNum].SRun, AXIS_NUM_MAX, g_SChannel[ucChannelNum].pMapAxisArray);
#endif // ERROR_COMP_TYPE == TWOWAY_SCREW_COMP
#if !MC_PROFILE_XYZ
    calculatePWMValue(&g_SChannel[ucChannelNum].SRun, g_SChannel[ucChannelNum].pMapAxisArray,
                      g_SChannel[ucChannelNum].pPWMMapAxisArray, TRUE); //计算PWM值
#endif // XYZ 总线配置发布位置目标，PWM 时序由对应脉冲输出配置处理。
}

/*************************************************
  函数名称：interruptFuncLoopAPI()
  函数功能  循环执行G代码中断
  输入参数：
  输出参数：
  返 回 值：
  备    注：
****************************************/
void interruptFuncLoopAPI()
{
    int i;
    //对于总线型不用进行换向处理，脉冲型需要
    for (i = 0; i < CHANNEL_NUM_MAX; i++)
    {
        interruptFuncAPI(i); //中断服务函数，计算脉冲值
#if !MC_PROFILE_XYZ
        isMotionChangeDirection(i); //判断轴换向，且修改轴脉冲
#endif
    }
}

/*************************************************
  函数名称：RunIOBuffAPI()
  函数功能  运行缓存中的IO代码
  输入参数：
  输出参数：
  返 回 值：
  备    注：
****************************************/
//void RunIOBuffAPI()
//{
//	//runIOBuff();
//}

/*************************************************
  函数名称：inputCoorOffset(uint8 ucNum, uint8 ucAxis, double dOffset)
  函数功能  手动换刀函数
  输入参数：ucToolNum	刀号  1~TOOL_NUM_MAX
  输出参数：
  返 回 值：1 设置完成
			-1 通道号错误
			-2 错误的状态
			-3 刀号错误
  备    注：
****************************************/
int8 manualChangeToolAPI(uint8 ucChannelNum, uint8 ucToolNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX) //通道号错误
    {
        return -1;
    }
    if (g_SChannel[ucChannelNum].ucChannelStatus != SYS_IDLE &&
        g_SChannel[ucChannelNum].ucChannelStatus != SYS_PAUSE) //其他状态不能手动换刀
    {
        return -2;
    }
    if (ucToolNum > TOOL_NUM_MAX || ucToolNum < 1) //刀号超过总刀数目
    {
        return -3;
    }

    g_SChannel[ucChannelNum].SPreProcess.ucToolNum = ucToolNum;
    return 1;
}

/*************************************************
  函数名称：changeFeedRatioAPI(float fNewRatio, bool bIsDelta,uint8 ucType)
  函数功能  改变倍率函数 （包含快速运动倍率 合成运动倍率 主轴倍率）
  输入参数：ucType		0:快速运动倍率; 1:合成运动倍率; 2:主轴倍率
			fNewRatio	输入值（增量或绝对）
			bIsDelta	是否为增量
  输出参数：
  返 回 值：
  备    注：
****************************************/
void changeFeedRatioAPI(uint8 ucChannelNum, float fNewRatio, bool bIsDelta, uint8 ucType)
{
    double* pdTargetRatio;
    double dMaxVal = 1.0, dMinVal = 1.0;
    struct _SRun* pSRun;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return;
    }
    pSRun = &g_SChannel[ucChannelNum].SRun;
    switch (ucType)
    {
    case 0: //快速运动倍率
        pdTargetRatio = &pSRun->dQuickMoveFeedRatio;
        dMaxVal = MAX_FEED_RATIO;
        dMinVal = MIN_FEED_RATIO;
        break;
    case 1: //合成运动倍率
        pdTargetRatio = &pSRun->dCommonFeedRatio;
        dMaxVal = MAX_FEED_RATIO;
        dMinVal = MIN_FEED_RATIO;
        break;
    case 2: //主轴倍率
        pdTargetRatio = &pSRun->dSpindleSpeedRatio;
        dMaxVal = MAX_SPINDLE_RATIO;
        dMinVal = MIN_SPINDLE_RATIO;
        break;
    default:
        pdTargetRatio = &pSRun->dCommonFeedRatio;
        break;
    }
    double dCurRatio = *pdTargetRatio; // 读取当前倍率
    if (!(ucType == 2 &&
          (getBits(g_SChannel[ucChannelNum].SPreProcess.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT ||
           getBits(g_SChannel[ucChannelNum].SPreProcess.unCommandType, CMD_MOTION_ADDR) ==
               CMD_MOTION_G33))) //陈敏聪	2023/10/16	G32或G33运行时不能改变主轴倍率
    {
        if (bIsDelta) //是不是相对值
        {
            dCurRatio += fNewRatio; //相对值
        }
        else
        {
            dCurRatio = fNewRatio; //绝对值
        }
    }
    dCurRatio = max(dMinVal, dCurRatio); //下限限制
    dCurRatio = min(dMaxVal, dCurRatio); //上限限制
    *pdTargetRatio = dCurRatio;          //传出参数
}

/*************************************************
  函数名称：setHandwheelMode(uint8 ucAxis, uint8 ucWheelMultiple)
  函数功能  设置手轮轴 倍率
  输入参数：ucAxis	轴号
			ucWheelMultiple 手轮倍率
  输出参数：
  返 回 值：-1	系统运行状态错误
			-2	C轴未停下
			-3	轴号错误
			1	设置成功
  备    注：
****************************************/
int8 setHandwheelMode(uint8 ucChannelNum, uint8 ucAxis, uint8 ucWheelMultiple)
{
    if (g_RunMode != HANDWHEEL_MODE)
        return -1;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -2;
    }
    switch (ucAxis)
    {
    case AXIS_C:
        if (g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucRunMode == SPINDLE_SINGLEAXIS_MODE &&
            g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED)
        { // 单轴模式下 主轴状态为完成 可切换成手轮模式
            g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucRunMode = SPINDLE_HANDWHEEL_MODE;
        }
        else
        { //  其他模式 让主轴先停下来
            g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucRunMode = SPINDLE_COMMONSTOP_MODE;
            if (g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucInpStatus != INP_OCCUPATION)
            {
                g_SChannel[ucChannelNum].SSpindle.pSSpindleAxis->ucInpStatus = INP_INITIATION;
            }
            return -2;
        }
    case AXIS_X:
    case AXIS_Y:
    case AXIS_Z:
    case AXIS_A:
    case AXIS_B:

        g_SEncoder[HANDEWHEEL_ENCODER - 1].AXISNum = ucAxis;
        break;
    default:
        return -3;
    }

    g_SEncoder[HANDEWHEEL_ENCODER - 1].ucWheelMultiple = ucWheelMultiple;

    return 1;
}

/*************************************************
  函数名称：setSysRunMode(enum SYSTEMSTATUS runMode)
  函数功能  设置系统模式
  输入参数：runMode
			EDIT_MODE,			//编辑模式
			AUTOMATIC_MODE,		//自动执行
			MDI_MODE,			//MDI
			HANDWHEEL_MODE,		//手轮
			HAND_MODE			//手动
  输出参数：
  返 回 值：1		设置完成
  备    注：
  ****************************************/
int8 setSysRunMode(enum SYSTEMSTATUS runMode)
{
    if (runMode == EDIT_MODE || runMode == AUTOMATIC_MODE || runMode == MDI_MODE || runMode == HANDWHEEL_MODE ||
        runMode == HAND_MODE || runMode == RE0_MODE)
    {
#if HANDWHEEL_SIM_DISTANCE_ENABLE
        /* g_ucHandwheelSimRequestArray 的续行资格属于原模式；g_SChannel 提供实际选择。
         * 退出自动/MDI 时，已生效且无请求的通道也须关闭，不能只隐藏面板灯。 */
        if ((int)g_RunMode != (int)runMode)
        {
            for (uint8 ucChannel = 0; ucChannel < CHANNEL_NUM_MAX; ++ucChannel)
            {
                /* 模式退出统一关闭，已有交接取消续行；保护关闭不得降级。 */
                if (g_ucHandwheelSimRequestArray[ucChannel] != HANDWHEEL_SIM_STOP_CLOSE &&
                    (g_ucHandwheelSimRequestArray[ucChannel] != HANDWHEEL_SIM_NONE ||
                     (((int)runMode != (int)AUTOMATIC_MODE && (int)runMode != (int)MDI_MODE) &&
                      isHandwheelSimSelectedAPI(ucChannel))))
                {
                    g_ucHandwheelSimRequestArray[ucChannel] = HANDWHEEL_SIM_CLOSE_HOLD;
                }
            }
        }
#endif
        g_RunMode = runMode;
        return 1;
    }
    else
    {
        return -1;
    }
}

//画图按钮API
int8 drawBtnAPI(uint8 ucChannelNum)
{
    switch (g_SChannel[ucChannelNum].ucChannelStatus)
    {
    case SYS_IDLE: //空闲状态下按启动  初始化 进行一部分解析、刀补 速度前瞻 再启动插补（置RUN）
    {
        //0.开启画图标志位
        g_SChannel[ucChannelNum].ucChannelStatus = SYS_RUN;
        g_SChannel[ucChannelNum].cSubRunMode = 0; //0 画图 1运动

        //1.G代码解析模块在runButtonAPI()中需要执行的内容
        //1.1G代码解析初始化
        gcodeInit(ucChannelNum, 0, 0);

        //1.2所有通道的通道结构体中添加主运行程序
        if (addMainProgramNode(ucChannelNum) != 1)
        {
            g_SChannel[ucChannelNum].ucChannelStatus = SYS_IDLE;
            g_SChannel[ucChannelNum].cSubRunMode = 1; //0 画图 1 运动	罗祥玮20250528
            return -1;
        }
#if STM32_MODE
        prepareProgramBeginLine(ucChannelNum);
#endif
        //1.3G代码解析状态置为GCODE_RUN
        changeGCodeStatus(GC_SYNTAX, ucChannelNum, GCODE_RUN, TRUE, 0); //启动对应的通道
        changeGCodeStatus(GC_ASSIGN, ucChannelNum, GCODE_RUN, TRUE, 0); //启动对应的通道

        //2.刀补计算模块在runButtonAPI()中需要执行的内容
        toolCompensationInit(ucChannelNum); //刀补初始化

        //3.初始化缓冲区索引
        initAllBuffer(ucChannelNum); //复位  把缓冲区初始化

        extern int8 cGetDrawData_Step;
        cGetDrawData_Step = 0;

        return 1;
    }
    case SYS_RUN:
    {
        if (g_SChannel[ucChannelNum].cSubRunMode == 0) //0 画图 1运动
            return 1;
        else if (g_SChannel[ucChannelNum].cSubRunMode == 1)
            return -1;
    }
    default:
        return -1;
    }
    return -1;
}
void drawInitAPI(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
        return;
    extern int8 cGetDrawData_Step;
    cGetDrawData_Step = 0;
    g_SChannel[ucChannelNum].cSubRunMode = 1;
    g_SChannel[ucChannelNum].ucChannelStatus = SYS_IDLE;
    //drawBtnAPI(ucChannelNum);
}
/*************************************************
  函数名称：void gCodeDrawLoop()
  函数功能：绘图循环
  输入参数：
  输出参数：
  返 回 值：
  备    注：弃用
  ****************************************/
//void gCodeDrawLoop()
//{
//	uint8 ucChannelNum;
//	for (ucChannelNum = 0; ucChannelNum < CHANNEL_NUM_MAX; ucChannelNum++)
//	{
//		if (g_bGCodeDraw[ucChannelNum])
//		{
//			mainFuncGCodeAuto(ucChannelNum);
//
//#if STM32_MODE
//			drawGCode();
//#endif
//		}
//
//	}
//}
//#endif

//调试API
//插补预处理状态
int8 getSInterPreStatus(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return 111;
    }
    return g_SChannel[ucChannelNum].SPreProcess.ucInpStatus;
}

//阻塞锁
uint8 getIOBlockedCounter(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return 111;
    }
    return g_SChannel[ucChannelNum].SIOCommand.ucIOBlockedCounter;
}

//获取运动状态
int8 getMotionTESTStatus(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR)) //指令类型判断  2024/10/15 熊骏
    {
    case CMD_MOTION_QUICKMOVE: //快速运动
        return g_SChannel[ucChannelNum].SG00.ucRunStep;

    case CMD_MOTION_LINE: //合成运动
        return g_SChannel[ucChannelNum].SG01.ucInpStatus;

    case CMD_MOTION_ARC:              //顺圆
    case CMD_MOTION_ANTI_ARC:         //逆圆
    case CMD_MOTION_ELLIPSE_ARC:      //顺椭圆
    case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆

        return g_SChannel[ucChannelNum].SG02.ucInpStatus;

    case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
    case CMD_MOTION_SCREWCUT_VP:
    case CMD_MOTION_SCREWCUT: //螺纹切削
        return g_SChannel[ucChannelNum].SG32.ucG32Step;
    case CMD_MOTION_G33: //G33螺纹攻牙循环
        return g_SChannel[ucChannelNum].SG33.G33MotionState;
    case CMD_MOTION_G10: //G10自动刀具补偿功能
        return g_SChannel[ucChannelNum].SPreProcess.ucInpStatus;
    default:
        return -1;
    }
}

//当前速度
int getCurrentSpeed(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR)) //指令类型判断  2024/10/15 熊骏
    {
    case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
    case CMD_MOTION_SCREWCUT_VP:
    case CMD_MOTION_SCREWCUT: //螺纹切削
        return (int)(g_SChannel[ucChannelNum].SG32.dCurrentSpeed * 1000 * 60);
    case CMD_MOTION_G33: //G33螺纹攻牙循环
        return (int)(g_SChannel[ucChannelNum].SG33.dCurrentSpeed * 1000 * 60);
    default:
        return (int)(g_SChannel[ucChannelNum].SRun.dCurrentSpeed * 1000 * 60);
    }
}

//主轴转速误差百分比
double getSpindleSpeedErrorRatio(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    return g_SChannel[ucChannelNum].dSpindleSpeedRatio;
}

//当前指令加速度  m/s2
double getMotionAcc(uint8 ucChannelNum)
{
    double dAcc;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (g_SChannel == NULL)
    {
        return -1;
    }
    switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR)) //指令类型判断  2024/10/15 熊骏
    {
    case CMD_MOTION_QUICKMOVE: //快速运动
        dAcc = g_SChannel[ucChannelNum].SG00.pSSingleAxisArray->SPlanData.dAcc;
        break;
    case CMD_MOTION_LINE: //合成运动
        dAcc = g_SChannel[ucChannelNum].SG01.SPlanData.dAcc;
        break;
    case CMD_MOTION_ARC:              //顺圆
    case CMD_MOTION_ANTI_ARC:         //逆圆
    case CMD_MOTION_ELLIPSE_ARC:      //顺椭圆
    case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆
        dAcc = g_SChannel[ucChannelNum].SG02.SPlanData.dAcc;
        break;
    case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
    case CMD_MOTION_SCREWCUT_VP:
    case CMD_MOTION_SCREWCUT: //螺纹切削
        dAcc = g_SChannel[ucChannelNum].SG32.SPlanData.dAcc;
        break;
    case CMD_MOTION_G33: //G33螺纹攻牙循环
        dAcc = g_SChannel[ucChannelNum].SG33.SPlanData.dAcc;
        break;
    case CMD_MOTION_G10: //G10自动刀具补偿功能
    default:
        return -1;
    }
    return dAcc * 1000;
}

//当前指令减速度  m/s2
double getMotionDec(uint8 ucChannelNum)
{
    double dDec;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (g_SChannel == NULL)
    {
        return -1;
    }
    switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR)) //指令类型判断  2024/10/15 熊骏
    {
    case CMD_MOTION_QUICKMOVE: //快速运动
        dDec = g_SChannel[ucChannelNum].SG00.pSSingleAxisArray->SPlanData.dDec;
        break;
    case CMD_MOTION_LINE: //合成运动
        dDec = g_SChannel[ucChannelNum].SG01.SPlanData.dDec;
        break;
    case CMD_MOTION_ARC:              //顺圆
    case CMD_MOTION_ANTI_ARC:         //逆圆
    case CMD_MOTION_ELLIPSE_ARC:      //顺椭圆
    case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆
        dDec = g_SChannel[ucChannelNum].SG02.SPlanData.dDec;
        break;
    case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
    case CMD_MOTION_SCREWCUT_VP:
    case CMD_MOTION_SCREWCUT: //螺纹切削
        dDec = g_SChannel[ucChannelNum].SG32.SPlanData.dDec;
        break;
    case CMD_MOTION_G33: //G33螺纹攻牙循环
        dDec = g_SChannel[ucChannelNum].SG33.SPlanData.dDec;
        break;
    case CMD_MOTION_G10: //G10自动刀具补偿功能
    default:
        return -1;
    }
    return dDec * 1000;
}

//当前指令速度规划的unT5
uint32 getUINT5(uint8 ucChannelNum)
{
    uint32 unT5;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (g_SChannel == NULL)
    {
        return -1;
    }
    switch (getBits(g_SChannel[ucChannelNum].SRun.unCommandType, CMD_MOTION_ADDR)) //指令类型判断  2024/10/15 熊骏
    {
    case CMD_MOTION_QUICKMOVE: //快速运动
        unT5 = g_SChannel[ucChannelNum].SG00.pSSingleAxisArray->SPlanData.unT5;
        break;
    case CMD_MOTION_LINE: //合成运动
        unT5 = g_SChannel[ucChannelNum].SG01.SPlanData.unT5;
        break;
    case CMD_MOTION_ARC:              //顺圆
    case CMD_MOTION_ANTI_ARC:         //逆圆
    case CMD_MOTION_ELLIPSE_ARC:      //顺椭圆
    case CMD_MOTION_ELLIPSE_ANTI_ARC: //逆椭圆
        unT5 = g_SChannel[ucChannelNum].SG02.SPlanData.unT5;
        break;
    case CMD_MOTION_SCREWCUT_LOOP: //循环螺纹切削
    case CMD_MOTION_SCREWCUT_VP:
    case CMD_MOTION_SCREWCUT: //螺纹切削
        unT5 = g_SChannel[ucChannelNum].SG32.SPlanData.unT5;
        break;
    case CMD_MOTION_G33: //G33螺纹攻牙循环
        unT5 = g_SChannel[ucChannelNum].SG33.SPlanData.unT5;
        break;
    case CMD_MOTION_G10: //G10自动刀具补偿功能
    default:
        return -1;
    }
    return unT5;
}

//获取当前行缓冲区的加速度
double getCurrentBuffAcc(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (getSToolCompPointerBufRes(INTERPOLATION_READ, ucChannelNum) > 0)
    {
        return getSToolCompPointerByIndex(getSToolCompPointerIndex(INTERPOLATION_READ, ucChannelNum) - 1, ucChannelNum)
                   ->dAcc *
               1000;
    }
    else
    {
        return -1;
    }
}

//获取当前行缓冲区的减速度
double getCurrentBuffDec(uint8 ucChannelNum)
{
    double dDec;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (getSToolCompPointerBufRes(INTERPOLATION_READ, ucChannelNum) > 0)
    {
        return getSToolCompPointerByIndex(getSToolCompPointerIndex(INTERPOLATION_READ, ucChannelNum) - 1, ucChannelNum)
                   ->dDec *
               1000;
    }
    else
    {
        return -1;
    }
}

//获取前瞻当前行号
int getLookAheadCurrentLine(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (getSToolCompPointerBufRes(LOOKAHEAD_READWRITE, ucChannelNum) > 0)
    {
        return getSToolCompPointerByType(LOOKAHEAD_READWRITE, ucChannelNum)->unGCodeLineNum;
    }
    else
    {
        return -1;
    }
}

//获取插补当前行号
int getInterpLine(uint8 ucChannelNum)
{
    if (ucChannelNum >= CHANNEL_NUM_MAX)
    {
        return -1;
    }
    if (getSToolCompPointerBufRes(INTERPOLATION_READ, ucChannelNum) > 0)
    {
        return getSToolCompPointerByType(INTERPOLATION_READ, ucChannelNum)->unGCodeLineNum;
    }
    else
    {
        return -1;
    }
}

/*************************************************
  函数名称：judgeRunCondition
  函数功能  判断当前通道环境能否运行
  输入参数：ucChannelNum
  输出参数：
  返 回 值：1		能运行
			-1		不能运行
  备    注：
  ****************************************/
bool judgeRunCondition(uint8 ucChannelNum)
{
#if (DRAW_TEST == 1)
    if (g_bGCodeDraw[ucChannelNum]) //23.4.28 fyh  绘图中，不能按运行键
    {
        return 0;
    }
#endif
    if (g_SChannel == NULL)
        return 0;
    if (ucChannelNum >= CHANNEL_NUM_MAX)
        return 0;
    if (g_SChannel[ucChannelNum].STIOCommand.ucToolChangeStatus == TOOLCHANGE_ERROR) //刀塔机模式下处于出错状态，不运行
    {
        parseErrorC(9999, PLC_ERROR_NUM1000);
        return 0;
    }

    return 1;
}
