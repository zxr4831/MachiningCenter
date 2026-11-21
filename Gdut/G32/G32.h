#ifndef __G32_H
#define __G32_H
#include <stdio.h>
#include "common.h"
#include "Config.h"

extern const double TAPER_45; //45°对应的弧度

//extern struct _SG32 g_SG32;

#define LHK_DEBUG 0    //1为vs测试
#define MATLAB_DEBUG 0 //用于matlab的数据（比一般的输出少一些参数）

//G32运行流程
#define G32_INIT_AND_ASSIGNMENT 0                    //初始化和赋值
#define G32_JUAGE_REPEATSTATE_AND_GET_REALCUTANGLE 1 //连续G32的判断以及获取真正切入角
#define G32_GET_SYNCSIGN 2                           //判断是否获取同步信号
#define G32_IFM_COMPENSATE 3                         //主轴不完全跟随模式下，过渡阶段的运动与补偿
#define G32_REPEATSTATE_COMPENSATE 4                 //减速规划类型连续G32长轴的运动与补偿
#define G32_RUNNING 5                                //运动部分
#define G32_OCCUPATION 6                             //轴占用
#define G32_USER_INPUT 7                             //用户输入
#define G32_BACKTAIL 8                               //退尾

enum UserInputState //用户参数输入状态
{

    NO_INPUT,               //无输入
    USER_INPUT,             //用户已输入
    FINSH_CHECK,            //用户参数通过检测
    FINSH_CHECK_SYSNCSIGNAL //用户参数通过检测,但运动需要同步信号
};

struct SInputG32 //（用户设置）参数(中间层的缓冲区)
{
    //VAL_STRUCT_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12  //这个值感觉目前没用到
    VAL_STRUCT_TYPE ucRelaVal; //相对值标识变量  2022-4-20

    double dAcc;  //加速段加速度	mm/ms2
    double dDec;  //减速段加速度	mm/ms2
    double dVs;   //初始速度	mm/ms
    double dVe;   //终点速度  mm/ms
    double dVmax; //最大速度  mm/ms

    //**************刘泓恺-G32解析所添参数*********2021—9—1**********
    double dJ; //短轴退尾量   G32参数模态  G92模态
    double dK; //长轴退尾量   G32参数模态  G92模态
    double dQ; //起始角度     G32参数非模态
    double dF; //导程

    SPointDouble
        SStarCutPoint; //切削起点，单位：mm（初始化之后，插补结构体的坐标只有长轴，所以需要在螺纹结构体保留切削起点终点）
    SPointDouble SEndPoint; //切削终点，单位：mm

    uint8
        ucUserInputState; //用户参数输入状态  0（NO_INPUT 无输入）  1（USER_INPUT 用户已输入）  2（FINSH_CHECK  完成赋值）
    uint8 ucAxis[2];      //执行插补的轴号
};

struct _SG32 //螺纹类指令参数结构体
{
    double dRatioXZ;    //X轴伺服电机与Z轴伺服电机的脉冲之比（用于切削锥螺纹,带短轴向量）
    bool bSyncSignal;   //同步信号
    bool bLeftTailFlag; //退尾标志位，有退尾位true 没有为false

    //**************刘泓恺-G32解析所添参数*********2021—9—1**********
    double dJ; //短轴退尾量   G32参数模态  G92模态
    double dK; //长轴退尾量   G32参数模态  G92模态
    double dQ; //起始角度     G32参数非模态
    double dF; //导程

    //**************刘泓恺-G32解析所添参数*********2021—9—14**********
    SPointDouble
        SStarCutPoint; //切削起点，单位：mm（初始化之后，插补结构体的坐标只有长轴，所以需要在螺纹结构体保留切削起点终点）
    SPointDouble SEndPoint;      //切削终点，单位：mm
    SPointDouble SBackTailPoint; //实际退尾点
    SVector SStartVector;        //起点矢量（单位向量）

    //**************刘泓恺-G32解析所添参数*********2021—11—17*********
    bool bRepeatG32Flag; //判断G32是否是连续G32指令
    uint8 ucG32Step;     //G32运行步骤
    double dMmToPulse;   //长轴脉冲当量

    //**************刘泓恺-G32解析所添参数*********2021—12—3*********
    uint8 ucMotionState; //运动状态  0加速 1随动 2减速 3完成

    //主轴不完全跟随模式下所记录的参数
    uint32 unCurrentT; //过渡阶段中，当前的插补时间点
    uint32 unTac;      //过渡阶段总的插补节点

    //**************刘泓恺-G32解析所添参数*********2021—12—29*********
    double dNextG32LongAxisSpeed; //若为连续G32，则下一个G32指令的长轴速度
    double
        dStartPointCompensate; //连续G32中，上一次G32终点的超过量作为当前G32起点的补偿量，使多次切削过程中第二段G32起点相同

    //**************刘泓恺-减速过渡型连续G32*********2021—1—13*********
    bool bDecPlanTransitionFlag;   //连续G32减速规划过渡标志位（适用于“换向过渡”、“减速过渡”连续G32）
    double dIdealTransmitStartPos; //理想开始减速规划过渡位置
    double dRemainDistance;        //剩余距离
    double dVs;                    //过渡阶段的起始速度
    double dDec;                   //重算的减速度
    double dIdealSpindlePosition;  //减速过渡阶段，终点处理想的主轴位置（脉冲形式）

    bool bPrintFlag; //（测试用参数）

    //**************刘泓恺-G32测试调试参数*********2021—1—18**********
    double dStartFollowDis; //开始随动所需的长度(运动起来后才能看到)（监测参数）

    //**************刘泓恺-连续G32*********2021—1—18*********
    double dSpindleSpeed;  //主轴转速，r/ms
    bool bStartCompensate; //开始补偿标志位

    //**************刘泓恺-G32框架改变*********2021—3—16*********
    struct SPlan SPlanData; //速度规划参数
    uint8 ucInpStatus;      //插补状态，INITIATION    ASSIGNMENT_START    RUNNING    PAUSING    FINISHED
    double dCurrentSpeed;   //当前速度 mm/ms

    struct SInputG32 SUserData; //（用户设置）参数(中间层的缓冲区)

    uint8 ucAxis[2];                 //执行插补的轴号 0号轴对应长轴，1号轴对应短轴
    uint32 unCommandType;            //指令类型
    VAL_STRUCT_TYPE ucValStatus;     //有效值（轴）类型
    double dPulseToMm[AXIS_NUM_MAX]; //脉冲当量的倒数  各个轴  XYZ ABC (还未赋值)

    struct SChannel* pSChannel; // 通道指针  (父)

    //**************刘泓恺-圆弧面螺纹*********2021—5—7*********
    SPointDouble SCenterPoint; //圆心，单位：mm
    double dRadius;            //半径，单位：mm
    int8 cArcDir;              //圆弧方向，顺时针=1，逆时针=-1

    //**************刘泓恺-变锥度螺纹*********2021—6—10*********
    struct SPlan*
        SExcessPlanForG32; //额外速度规划参数指针（现用于变锥度下短轴的速度规划，退尾时短轴的速度规划，用的时候才申请空间）
    bool bExcessPlanFlag;             //需要进行额外的速度规划标志位（短轴跟随长轴的规划）
    SPointDouble SLastEndPoint;       //上一个G32的切削终点，单位：mm
    double dLastRatioXZ;              //上一段G32的长短轴之比（用于切削锥螺纹,带短轴向量）
    double dShortAxisCurrentSpeed;    //短轴的当前速度
    double dShortAxisCompensate;      //短轴补偿量
    double dStartPointCompensateCopy; //起始补偿量副本
    double dShortAxisPos;             //退尾时，短轴比长轴先运动结束，记录短轴退尾后的位置

    //**************刘泓恺-G34*********2021—6—10*********
    bool bG34RunFlag;          //G34运行标志位，用于G34从终点反算何时减速
    SPointDouble SG34EndPoint; //G34的终点
    bool bG34BackTailFlag;     //G34退尾标志位
    uint8 ucLastAxis[2];       //上一个G32执行插补的轴号

    //**************刘泓恺-G34*********2021—6—10*********
    bool bSysDataInitFlag; //是否使用系统参数初始化螺纹(通常使用)

    //**************刘泓恺-G32（增加避空距离）*********2023—5—12*********
    double dAcc;        //加速段对应的加速度
    bool bAccPhaseFlag; //螺纹插补处于加速阶段标志位

    /********************************陈敏聪************************************/
    uint8 ucG32SubMode; //G32的子类	用于区分G32.2/G32.3/G32.4	2023/6/15增
};

//获取模拟主轴编码器的数值
int GetSpindleEncoder();

//载入输入的参数至G32结构体
uint8 loadSG32DataFromInput(struct SToolCompensation* pSToolCompPointer, struct SInput* pSInput, struct _SG32* pG32,
                            struct SInputG32* pSInputG32);

//获取输入参数
int8 getInputParameter(struct _SG32* pG32);

//计算G32运动过程中的绝对坐标
int8 calculateExactPointForG32(double* pdTargetPoint, double dDeltaL, struct _SG32* pG32);

//判断是否获得同步信号
bool isGetSyncSignal(double dTargetPositionPulse, struct SChannel* pSChannel, struct SEncoder* pSSpindleEncoder);

//对螺纹类参数结构体中G32指令所需的参数进行初始化
void G32Init(struct _SG32* pG32);

//获取螺纹运动中的比例系数以及长轴插补的距离
void G32Prepare(struct _SG32* pG32);

//根据刀补后的结构体参数 初始化螺纹类指令参数结构体
void initThreadPreStructData(struct _SG32* pG32);

//退尾预处理
void retreatTailPre(struct _SG32* pG32);

//连续G32预处理
void prepareRepeatG32(struct _SG32* pG32, struct SToolCompensation* pSToolCompPointer, double dDistFromSpindle);

//主轴不完全跟随模式下，过渡阶段的运动与补偿
void runAndCompForIFM(struct _SG32* pG32, double* dDistFromSpindle);

//减速规划类型连续G32长轴的运动与补偿
void runAndCompForG32RepeatDec(struct _SG32* pG32, double* dDistFromSpindle);

//G32运动坐标点的计算
int8 motionForG32(struct _SG32* pG32, double* dLongAxisDelta, double dDistFromSpindle, double dVectorFromSpindle,
                  double dAcc, double dDec);

//减速类型连续G32过渡规划的前置准备
void prepareG32RepeatDecPlanning(struct _SG32* pG32, double* dLongAxisDelta);

//加速类型连续G32的补偿量计算
void compensateForAccRepeatG32(struct _SG32* pG32);

//获得Q值切入的真正位置
double getRealTargetPulse(struct _SG32* pG32);

//“减速”类型连续G32的速度规划
void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance);

//初始化和运动准备
void prepareForMoveAndInit(struct _SG32* pG32);

//连续G32的判断以及获取真正切入角
void judgeRepeatG32AndGetRealCutAngle(struct _SG32* pG32, double dDistFromSpindle);

//判断是否获得同步信号和补偿启动偏差
void judgeSyncSignalAndCompensate(struct _SG32* pG32);

//G32运行
void G32Run(struct _SG32* pG32);

//用户外部输入（测试函数）
void UserInput(struct _SG32* pG32);

//选择G32插补平面
int8 chooseInterpPlaneForG32(struct _SG32* pG32);

//处理用户输入的插补轴轴号
void processUserInputAxis(struct _SG32* pG32);

//G32需要载入的系统参数
int8 SG32LoadSysPara(struct _SG32* pG32);

//处理用户的输入
int8 processUserInput(struct _SG32* pG32);

//计算圆弧面G32的短轴坐标
//double calcArcG32ShortAxisPos(struct _SG32* pG32, double dLongAxisPos);
int8 calcArcG32ShortAxisPos(struct _SG32* pG32, double dLongAxisPos, double* pShortAxisPos);

//G34退尾时的轴处理
void processAxisForG34BackTail(struct _SG32* pG32, double* dCutDisFirstAxis, double* dCutDisSecondAxis);

double getMoveLagComp(struct SChannel* pSChannel, double dF, uint8 ucLongAxisNum, int nTargetLagNum);

void backTailShortAxisPlan(struct _SG32* pG32, double dShortAxisDistance);

int8 calculateExactPointForG32BackTail(double* pdTargetPoint, double dLongAxisDeltal, double dShortAxisDeltal,
                                       struct _SG32* pG32);

////手轮新增
//void getHandWheelData(struct SChannel* pSChannel);

void initResetForG32(struct SChannel* pSChannel); //G32与复位的交互函数 2023.6.24

void changeNoRepeatG32(struct SChannel* pSChannel); //重置连续G32为非连续G32

#endif
