#ifndef __CHANNEL_H
#define __CHANNEL_H

#include "common.h"
#include "HandwheelSim.h"
#include "G0.h"
#include "G1.h"
#include "G2.h"
#include "G32.h"
#include "G33.h"
#include "IOCommand.h"
#include "TIOCommand.h"

#define CHANNEL_ZERO 0 // 宏定义  通道0  方便查找替换

#define CHANNEL_SIGNAL_WAIT		0 //通道等待指令
#define CHANNEL_SIGNAL_START	1 //通道启动指令
#define CHANNEL_SIGNAL_CHECK	2 //检测通道状态M96

#define CHANNEL_BUFF_SIZE	10//信号的缓冲区大小

struct SChannel // 通道结构体
{
	uint8 ucChannelNum;	//通道号  对应缓冲区编号 0 1 2 3 
	//uint8 ucChannelType;	// 通道类型  导向通道/跟随通道（未使用）
	
	uint8* pMapAxisArray;//该通道的轴映射

	uint8* pPWMMapAxisArray;//该通道的轴映射 2025.10.31熊骏

#if(TURRET_AXIS_TEST == 1)
	uint8* pChannelToTurretmap;//该通道对刀塔的映射
#endif

	//(原系统状态g_SystemStatus)  
	enum SYSTEMSTATUS ucChannelStatus;// 通道状态①空闲   ②运行(1运行 2暂停 3复位 4 RUN2PAUSE 5 PAUSE2RUN
	int8 cSubRunMode;//系统为运行状态下分为两个子模式：0 画图  1 运动 罗祥玮20250528：在系统进入运行状态时需要同时设置子模式

	SPointDouble SRelaCoorStartPoint;     //相对坐标起始点

	//公用结构体
	struct _SRun SRun;//运行结构体
	struct SInterpolationPre SPreProcess;//预处理结构体

	//运动结构体
	struct SG0 SG00;	//G0结构体
	struct SSingleAxis SSingleAxisArray[AXIS_NUM_MAX];//单轴结构体
	struct SG1 SG01;	//G1结构体
	struct SG2 SG02;	//G2结构体
    //G33结构体
	struct G33 SG33;
	struct _SG32 SG32;  //G32结构体

	//主轴
	struct _SSpindle SSpindle;			//主轴
	struct _SEncoderTest SSpindleTest; //主轴转速检测
	struct SEncoder* pSSpindleEncoder;	//主轴编码器
	double dSpindleRealSpeed;//主轴实际转速

	//动力刀
	struct _SSpindle SSecondSpindle;			//A轴，第二旋转轴，动力刀
	struct _SEncoderTest SSecondSpindleTest;	//A轴转速检测
	struct SEncoder* pSSecondSpindleEncoder;	//A轴编码器
	double dSecondSpindleAnalogSpeed;			//A轴理论转速

	//通用编码器
	struct SEncoder* pSCommonEncoder;   //编码器2

	//手轮
#if HANDWHEEL_SIM_DISTANCE_ENABLE
    struct SHandwheelSimGroup SHandwheelSim; /**< 当前 G0 总控及各逻辑轴独立距离账本。 */
#endif
	struct _SEncoderTest SHandWheelTest; //手轮转速检测
	struct SEncoder* pSHandleWheelEncoder; // 手轮编码器

	//IO
	struct SRunTime SApplyTime;	//作用时间
	struct _SIOCommand SIOCommand;//IO执行缓冲区

	//TIO
	struct _STIOCommand STIOCommand;//TIO执行缓冲区

	//标志位	
	bool bCalculateSpindleSpeedFlag;		//计算主轴转速标志位
	bool bCalculateSecondSpindleSpeedFlag;  //计算第二主轴转速标志位
	bool bLookAheadNewBeginFlag;	//前瞻新开始
	bool bGcodeWriteOverFlag;		//G代码写入完成标志位
	bool bToolWriteOverFlag;		//刀补写入完成标志位
	bool cHandWheelSimulaStatus;			//开启手轮模拟???

	bool bG33SpindleOverSpeedWarningSigns;  //G33超速警告标志位

	bool bSingleLineMode;//单节停止状态
	bool bSelectableStop;//选择停止状态
	bool bProgramStop;//程序暂停（M01等待期间用） 洪冠鹏 2023/10/7 增
    bool bSkipMode;//跳段模式  
	uint8 ucRunCounter; //运行计数  为0时  且单节停止状态  或选择停止状态下 段间停
	int64 lIdealSpindlePulse;  //发送的主轴脉冲总量

	bool bProgramNoFinished;	//判断程序是否结束，当程序无M30则程序还没结束

	struct _SMachineTimer SMachineTimer;//加工计时

	//调试变量
	double dSpindleSpeedRatio;

	int8 cRunChannelGcodeAgain;	//是否重新运行G代码	陈敏聪	2023/8/28增
	int8 cG65SystemMacroRun;//G代码G65系统宏继续解析 陈育楠 2023.10.31

	bool bChuckALoose;//运行过程中： 1-A夹头松开 0-A夹头夹紧 洪冠鹏 2023/10/14
	bool bChuckBLoose;//运行过程中： 1-B夹头松开 0-B夹头夹紧 

	bool bSingleLineModeUnFinished;//单节执行下当前行未执行完毕 洪冠鹏 2024/5/19
	
	bool bIsStopSpindleAndCoolantwhileReset;// 复位执行关主轴同时关冷却	 默认为0，关主轴同时关冷却
	bool bIsStopSpindleAndCoolantwhileM30;	// M30执行关主轴同时关冷却	 默认为0，关主轴同时关冷却	

	double dLessChipBreakLPreDist;    //不足断屑间距但已走的距离 洪冠鹏2024/8/1
	bool bChipBreakStart;             //开始断屑 洪冠鹏2024/8/1

	//倾斜Y轴机：虚拟Y轴与倾斜Y1轴间的夹角
	double dCOSAngleYY1;//cos60°= 0.5
	double dSINAngleYY1;//sin60°= 0.8660 2540378
	double dTANAngleYY1;//tan60°= 1.7320 5080757
	bool bUseVirtualY;//是否开启合成虚拟Y轴功能 
};

struct SWaitSignal //通道等待信号数据
{
	uint8 ucTag;			//标签
	uint8 ucFinishedSign;	//完成标记（以位来表示需要等待的通道，例如 0000 0101 表示通道0和通道2）
	uint8 ucCurrentSign;	//当前标记
	bool bStatus;	//状态
};

struct SChannelBufElement//通道信号数组元素
{
	uint8 ucType;//类型 ①通道等待  ②通道启动  ③检查通道状态M96
	bool bOccupy;//是否被占领
	bool bApply;//是否可执行
	

	union SChannelSignal
	{
		uint8 ucChannelNum;//②通道启动 ③检查通道状态M96
		struct SWaitSignal SWaitSignal;//①通道等待
	}UData;
};

struct SChannelManager//通道信号管理结构体
{
	//uint16 usStatus;//数组的状态（以位来表示有效数组元素）  第0位为1表示数组的0元素有效
	struct SChannelBufElement SBuff[CHANNEL_BUFF_SIZE];//等待信号数组
};

//通道等待信号信息结构体
struct SChannelWaitSignalData
{
	bool bOccupied;    //是否被占领     （表示单元是否有数据写入）
	uint8 ucFinishedSign;	//完成标记（以位来表示需要等待的通道，例如 0000 0101 表示通道0和通道2）
	uint8 ucCurrentSign;	//当前标记
	bool bStatus;	//状态
};

//通道等待信号管理结构体
struct SChannelWaitSignalMannager
{
	struct SChannelWaitSignalData  SBuff[CHANNEL_BUFF_SIZE];//等待信号信息数组
};

//extern struct SChannel g_SChannel[CHANNEL_NUM_MAX];//通道结构体数组
extern struct SChannel* g_SChannel;///通道结构体指针
//extern struct SChannelManager* g_SChannelManager;//通道信号管理结构体指针
extern  struct SChannelWaitSignalMannager* g_pChannelWaitSignalMannager;//通道等待信号管理结构体指针
//extern struct SChannelManager g_SChannelManager->//通道信号管理

extern bool g_bSChannelManagerApplyByMain;//通道信号管理器只能在Main中运行 

//在动态内存中开辟通道结构体
struct SChannel* initSChannel();

////在动态内存中开辟通道信号管理结构体
//struct SChannelManager* initChannelManager();

//在动态内存中开辟通道等待信号管理结构体
struct SChannelWaitSignalMannager* initChannelWaitSignalManager();
//	通道初始化函数
void initChannel(struct SChannel* pSChannel, uint8 ucChannelNum, uint8* ucAxisMapArray, uint8* ucPWMAxisMapArray, struct SEncoder**pEncoderArrays);
//	获取通道状态
enum SYSTEMSTATUS getChannelStatus(uint8 ucChannelNum);

//通道内轴使能
int8 channelAxisEnable(uint8 ucChannelNum);

////	等待信号写入函数
//int8 waitSignalWrite(uint8 ucTag, uint8 ucFinishedSign);

////	启动通道信号写入函数
//int8 startSignalWrite(uint8 ucTargetChannelNum, int8 cFileNameVal);
//
////	检测通道状态信号写入函数
//int8 checkSignalWrite(uint8 ucTargetChannelNum);

////	执行通道信号
//int8 channelSignalApply(int8 cSignalBufNum, uint8 ucChannelNum);

//私有函数
//
////通道等待信号
//int8 waitSignalApply(int8 cSignalBufNum, uint8 ucChannelNum);
//
////启动通道信号
//int8 startSignalApply(int8 cSignalBufNum);
//
////检测通道信号
//int8 checkSignalApply(int8 cSignalBufNum);

//	等待信号写入函数
int8 waitSignalWrite(uint8 ucFinishedSign);
//	执行通道信号
int8 channelSignalApply(int8* cSignalBufNum, uint8 ucChannelNum, struct SChannelCommand* psChannelCommand);
//私有函数
int8 waitSignalApply(int8* cSignalBufNum, uint8 ucTargetChannelNum, uint8 ucChannelNum);
//检测通道信号执行
int8 checkSignalApply(uint8 ucTargetChannelNum);
//启动通道信号执行
int8 startSignalApply(uint8 ucTargetChannelNum, int32 cTargetFileNameVal);
//初始化倾斜Y轴机的参数
int8 initInclinedYMachinePara(struct SChannel* pSChannel, double dAngleYY1);
#endif // !__CHANNEL_H

