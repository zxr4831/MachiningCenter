#ifndef __IOCOMMAND_H
#define __IOCOMMAND_H

#include "common.h"

#if !STM32_MODE
extern uint8 out[30]; //企业的输出IO数组,与企业同名
#endif

//读写IO的宏定义
#define MCODE_GROUP_NUM 3     //M代码组数
#define MCODE_GROUP_SPINDLE 0 //主轴 M3 M4 M5
#define MCODE_GROUP_FUILD 1   //冷却液 M8 M9
#define MCODE_GROUP_CHUCK 2   //夹头M10、M11

//消抖
#define ELIMINATEDITHERING_SAMPLING_TIMES 7 //消除抖动的采样次数
#define JUDGE_HIGH_LEVEL_TIMES 5            //采样总数中含有多少个高电平判定为高电平
#define JUDGE_LOW_LEVEL_TIMES 5             //采样总数中含有多少个低电平判定为高电平

//M01IO处理状态
#define VARIABLE_IO_INITATION 0 //初始化
#define VARIABLE_IO_DETECT 1    //等待IO执行完
#define VARIABLE_IO_ERROR 3     //报警
#define VARIABLE_IO_FINISH 4    //完成
#define VARIABLE_IO_WAIT 2      //暂停，选择停止下的M01(无K、无J)

typedef struct
{
    uint8 ucStatus;               //状态机
    SIOContrlInfo* pIOContrlInfo; //IO信息
    uint32 unOutputIOIndexOfM20;  //按位表示M20的输出端口 洪冠鹏 2023/9/22
} SVaribleIOInfo;

struct _SIOCommand //IO指令结构体
{
    struct SChannel* pSChannel; //通道指针

    uint8 ucIOBlockedCounter; //IO阻塞锁 (阻塞时(!=0) IRQHandle()只改变主轴转速就退出)

    /* 苏向标 2026-10-14：按入队版本识别已被停止取消的异步使能操作。 */
    uint32 unSpindleRequestVersionArray[IO_MAXGROUP_IN_SINGLE_LINE]; /**< 各槽入队时的主轴请求版本。 */
    volatile int8 cSpindleIOSlot;                  /**< 0空闲；正值为请求槽号+1；负值为已完成槽号的相反数。 */
    uint8 ucIORunBuff[IO_MAXGROUP_IN_SINGLE_LINE]; //执行IO的缓存

    uint8 ucPreAndPostDelayStatus[IO_MAXGROUP_IN_SINGLE_LINE]; //前延时和后延时同时存在的时候使用的状态机

    SVaribleIOInfo SVaribleIOInfoArray[IO_MAXGROUP_IN_SINGLE_LINE]; //可变IO状态信息

    uint8 ucMStatus; //M码状态 摆臂机专用
};

struct _STIOCommand
{
    struct SChannel* pSChannel;

    uint8 ucToolNum;

    uint8 ucToolCompNum;

    uint8 ucToolChangeStatus;

    uint8 ucTIORun;

    uint8 ucToolCompNumRun;

    uint8 ucReadIO_Counter;
};

#if !STM32_MODE

typedef unsigned int u32; //0~4294967295，变量命名示例unVar

extern unsigned long AxisEnable[AXIS_PHYSICS_NUM_MAX]; //各个轴的使能位

extern int ioM10M11;                //夹头IO
extern int ioM12M13;                //夹头IO
extern int ioM38M39;                //主轴刹车IO
extern int ioEMERGENCY_STOP;        //急停IO
int delays_ms(u32 ms, u32* delang); //计时函数1ms

extern uint8 out[30];          //企业的输出IO数组,与企业同名
extern uint8 attribute[40][3]; //企业的输入IO数组,与企业同名

#else
#if (MACHINE_CONFIG == NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE) //多通道双主轴机配置	20237/29	陈敏聪
#define ioEMERGENCY_STOP intx5
#else
#define ioEMERGENCY_STOP intx0
#endif

#endif // !STM32_MODE

extern uint8 g_ucIOBlockedCounter; //IO阻塞锁 (阻塞时(!=0) IRQHandle()只改变主轴转速就退出)

extern uint8 g_ucIORunBuff[IO_MAXGROUP_IN_SINGLE_LINE]; //执行IO的缓存

extern uint8 g_ucIOToolRun; //执行IO的缓存

//
// //根据M代码寻找组号和状态
// int8 findIOGroup(uint8 ucNum,uint16 *usStatus);

//设置夹头/切削液状态
int8 setMCodeRunStatus(uint8 ucChannelNum, uint8 ucNum, uint8 ucStatus);

//获取夹头/切削液状态
int8 getMCodeRunStatus(uint8 ucChannelNum, uint8 ucNum);

//根据M代码寻找作用类型（前作用/后作用）
int8 findIOType(uint8 ucNum);

//设置M代码  存储到ucIOCommand
int8 setIO(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize);

//查询该M代码是否阻塞
uint8 isBlockedIO(uint8 ucNum);

//存放M代码到buff中执行
uint8 IOStructToBuff(uint8 ucIOCommand, struct _SIOCommand* pSIOCommand, SIOContrlInfo* pIOContrlInfo);

//执行M代码  （输入前后作用类型    把对应类型的M代码放到缓存中）
uint8 ApplyIO(uint8* ucOCommandArray, uint8 ucSize, uint8 ucCommandType, struct _SIOCommand* pSIOCommand,
              SIOContrlInfo* pIOContrlInfo);

//执行指令的M指令
int8 runIO(struct _SIOCommand* pSIOCommand, uint8 ucIOCommand, uint8* pPreAndPostDelayStatus,
           SVaribleIOInfo* pSVaribleIOInfo);

//遍历缓存  运行M指令
uint8 runIOBuff(struct _SIOCommand* pSIOCommand);
/*************************************************
  函数名称：processSpindleIORequest
  函数功能：周期路径仅执行后台已发布的主轴 IO 槽位
  输入参数：pSIOCommand - 通道 IO 结构
  输出参数：更新该槽位和完成应答
  返 回 值：无
  备    注：苏向标 2026-10-14；不关中断，不使用 goto
****************************************/
void processSpindleIORequest(struct _SIOCommand* pSIOCommand);

//寻找M代码
int8 findIO(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize);

//寻找IO在缓存中的位置
int8 findIOPosition(uint8 ucNum, uint8* ucIOCommandArray, uint8 ucSize, uint8* ucPos);

//获取Buff中的主轴状态
int8 checkSpindleStatus(uint8* ucIOCommandArray, uint8 ucSize);

//判断目标数组是否有阻塞IO
uint8 isBlockedInCommand(uint8* ucIOCommandArray, uint8 ucSize);

//清除IO指令 （可指定前后操作类型）
int8 clearIOCommand(uint8* ucIOCommandArray, uint8 ucCommandType, uint8 ucSize);

//判断是否有目标M代码
bool hasIOCommand(uint8* ucIOCommandArray, uint8 ucSize);

//获取M代码状态
int8 getOrSetMCodeStatus(uint8 ucChannelNum, uint8 ucGroupNum, uint8* pVal, bool bIsWrite); // 读或写

//检查IO, 并根据部分IO，设置相关的状态变量使得G0运动阻塞
void checkAndsetIOBlockedBeforG0(uint8 ucNum, struct _SIOCommand* pSIOCommand);

//上料IO执行
int8 runM35Command(struct _SIOCommand* pSIOCommand, SVaribleIOInfo* pVaribleIOInfo);

/* 释放程序缓冲和清空IO队列前撤销活动M35/M36等待输出。 */
void cancelWaitingInputIO(struct _SIOCommand* pSIOCommand);

//M20 / M21 / M22指定的IO执行
int8 runM20M21M22Command(struct _SIOCommand* pSIOCommand, SVaribleIOInfo* pVaribleIOInfo);

//M01的指定IO检测
int8 detectM01Command(struct _SIOCommand* pSIOCommand, SVaribleIOInfo* pVaribleIOInfo);

//运行作用时间清零
void clearApplyRunTime(struct SChannel* pSChannel);

#endif
