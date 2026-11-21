#ifndef __GCODECHECKANDASSIGN_H
#define __GCODECHECKANDASSIGN_H

//lxw:头文件包含关系
//#include "GCodeRegulation.h"

#include "GCodeRegulation.h"
#include"common.h"
/**********************/
//结构体定义
/**********************/

//模态指令结构体
typedef struct _SGCodeModalInfo
{
	//Ga组：G0、G1、G2、G3、G6.2、G6.3、G32、G33、G34、G92、G93、G94、G82、G83、G84、G85、G86、G87、G88、G89
	//Gz组：G4、G10、G50、G65、G66、G67、G70、G71、G72、G73、G74、G75、G76、G78、G28
	//Ge组：G20、G21
	//Gc组：G17、G18、G19
	//Gb组：G54、G55、G56、G57、G58、G59
	//Gd组：G40、G41、G42
	//Gh组：G90、G91
	//Gf组：G96、G97
	//Gg组：G98、G99
	//Gx组：G80
	
	int16 sGb, sGc, sGd, sGe, sGf, sGg, sGh;//数值为指令的10倍，G1->G10，G10->G100

	//lxw：Gz为非模态指令，但放在模态结构体体中，便于对比Ga组、Gz组指令
	//lxw：缓冲区单元从系统状态中SGCodeModalInfo后要单独取消Gz组指令的继承
	int16 sGa,sGz;
	const SGCommandRegulation* pSGaRegulation;
	const SGCommandRegulation* pSGzRegulation;
}SGCodeModalInfo;


//扫描模态结构体
typedef struct _SScanModalInfo
{
//	int16 sGa, sGb, sGc, sGd, sGe, sGf, sGg, sGh;
	const SGCommandRegulation* pSGaRegulation;
	const SGCommandRegulation* pSGbRegulation;
	const SGCommandRegulation* pSGcRegulation;
	const SGCommandRegulation* pSGdRegulation;
	const SGCommandRegulation* pSGeRegulation;
	const SGCommandRegulation* pSGfRegulation;
	const SGCommandRegulation* pSGgRegulation;
	const SGCommandRegulation* pSGjRegulation;
	const SGCommandRegulation* pSGhRegulation;


	const SGCommandRegulation* pSGzRegulation;//Gz组为非模态，放在这便于与Ga组对比
	/// G代码分类参数，
	/// sGa：运动类 G00，G01，G02，G03，G32,G33,G76,G92						MOCTION
	/// sGb：工件坐标系 G54-G59												COOR
	/// sGc：定义插补平面 G17-G19
	/// sGd：刀具补偿 G40-G42												TOOLCOMP
	/// sGe：单位选择：英制/公制 G20，G21									UNITS   INCHS MM
	/****** 新增模态组 *******/  //2021-9-13  冯英华
	/// sGf: 恒线速控制 G96、恒转速控制 G97									            SPINDLE_SPEED
	/// sGg: 给定每分进给速度 G98 、以毫米/转为单位给定切削进给速度 G99	、断削指令G4.9	FEED
	/// sGh: 绝对编程G90/ 相对编程G91  默认G90
	//int8 cDecimalVal;  //针对sGa组，判断小数后一位，例如：G32.1时，值为1， G32.2时，值为2
}SScanModalInfo;

//扫描非模态结构体
typedef struct _SScanNonModalInfo
{
	//int16 sGz;  //无:0;  G04 G50  G71 G72 G73
	/// sGz：非模态组别G04(程序暂停)、G50
	//int16 sGx;	//G80
	/// sGx: 非模态组别，用于G83~G89的模态取消

//	const SGCommandRegulation* pSGzRegulation;
	const SGCommandRegulation* pSGxRegulation;

	//bool ;//sMb初始值为-1
	//int8 cDecimalVal;  //记录小数后一位，例如：G32.1时，值为1， G32.2时，值为2	//弃用：指令已放大，按整数处理

	int8 cMLetterCnt;
	bool bMa, bMb, bMc, bMe, bMf, bMh, bMi, bMj, bMk, bMg;
	const SMCommandRegulation* pSMCommandArray[6];


	/// M代码分类参数，
	/// cMa：主轴正/反转/停		/M03/M04/M05(非模态）
	/// sMb：程序停/程序结束	/M00/M01/M02/M30(非模态)	M96 M97 M98 M99		M100~M199
	/// cMc：冷却液开/关		/M08/M09(非模态)
	/// cMe：润滑油开/关        /M32/M33
	/// cMf：工件夹紧 / 松开 / M10 / M11
	/// cMg：M14 / M15
	/// cMh：动力刀正/反转/停		/M50/M51/M52(非模态）
	/// cMi：M38 / M39
	/// cMj：M35 / M36 /M20 / M21 / M22	//自动重复上料功能	M35控制高电平输出/M36控制低电平输出
	/// cMk: M80/M81/M82/M83/M84/M85/M86/M87 //机械臂
}SScanNonModalInfo;

//扫描结构体
typedef struct _SScan
{
	uint32 unLineNum;   //当前行号

	SScanModalInfo SScanModalInfo;			//模态指令
	SScanNonModalInfo SScanNonModalInfo;     //非模态信息
	int8 cMacroType;//宏关键字类型

	uint8 ucScanStep;
	double dLexicalUnitValue;   //词法单元的值(跨步骤使用，必须保存在这，否则造成数据丢失)

	int8 cScanFinishFlag;


	bool bA2Z[26];
	double dA2Z[26];

	bool  bABC[3]; double dABC[3];
	bool  bXYZ[3]; double dXYZ[3];
	bool  bUVW[3]; double dUVW[3];
	bool  bRIJK[4]; double dRIJK[4];
	bool bPoundP; uint16 usPoundP;//P#i中的i值大小
	bool bPoundQ; uint16 usPoundQ;//Q#j中的j值大小
	bool bPoundR; uint16 usPoundR;//R#k中的k值大小

	bool bF, bS, bT; double dF, dS, dT;
	bool bP, bH; bool bPFirst, bHFirst; double dP, dH;

	bool bN, bQ, bL, bD, bO;
	double  dN, dQ, dL, dD, dO;

	bool bGOTO; double dGOTO;

	bool bIF; double dIF;
	bool bELSEIF; double dELSEIF;
	bool bELSE;
	bool bENDIF;
	bool bTHEN;

	bool bWHILE; double dWHILE;
	bool bENDWHILE;
	bool bDO;
}SScan;



//G代码缓冲区单元结构体
struct SGCodeParse
{
	int8 cFilename[FILENAME_LENGTH_LIMIT];//文件名
	uint32 unGCodeLineNum;    // 解析到第几行G代码
	//struct SGCodeModalInfo SModalInfo;	         //模态信息     //2021-8-16  冯英华
	SGCodeModalInfo SModalInfo;
	//struct SGCodeNonModalInfo SNonModalInfo;     //非模态信息   //2021-8-16  冯英华	// lxw后续删除

	SPointDouble* pSG50Offset;			//lxw添加，用于记录G50偏置,废弃


	int8 cPointType;//1(G50)  2(G10)
	uint8 uG10ToolID;//G10的刀具号
	SPointDouble SPointInfo; /**< G50为绝对偏置；G10为本条磨耗增量，单位mm，X为径向量。 */

	//bool bG50Appear;  //G50出现标志位 True：当前程序段有G50出现   False：当前程序段没有G50出现

	bool bG65SystemMacroWait;//G65系统宏等待标志

	uint32 unCommandType;     //指令类型	2021-09-17 朱稳中 uint8 ->uint16
	//（指令类型变量不利于实现模块化，或者在规则结构体中加入指令地址与指令对应的位）

	//int8 cInterpolationState;	//旧：0:普通模式 1:G13 2:G12 3:极坐标模式 4:G7 5:圆柱模式
	int8 cInterpolationState;	//新：0:普通模式 1:极坐标模式 2:圆柱模式

	VAL_STRUCT_TYPE	ucRelaVal;		//相对值标识变量  2022-4-20  朱稳中	//lxw新框架未使用该变量

	enum ENTITY EntityType;		//实体类型，LINE:1, CIRCLE:2, ARC:3
	int8 cArcDir;	         //圆弧方向，顺时针=1，逆时针=-1

	double dRadius;			//[半径，单位：mm]  [G92 切削起点与切削终点X轴绝对坐标的差值(半径值)] [G76:第1行，R(d)：螺纹精车的切削量，第2行，R(i) ：螺纹锥度，螺纹起点与螺纹终点X 轴绝对坐标的差值]
	SPointDouble SCenterPoint;	//圆心，单位：mm
	SPointDouble SStartPoint;	//起点，单位：mm
	SPointDouble SEndPoint;		//终点，单位：mm
	float  dEllipseFactorA; // 椭圆参数A   2023.3.29 朱稳中
	float  dEllipseFactorB; // 椭圆参数B   2023.3.29 朱稳中
	float  fCylinderRadius; // 圆柱半径 mm  2023.3.29 朱稳中

	//SToolParaInfo SToolPara;	//刀具参数 弃用 2023.6.9 冯英华
	uint8 ucToolID;     //刀具ID 2023.6.9 冯英华
	uint8 ucToolCompNum;//刀补号
	uint8 ucTurretNum;//刀塔号（刀座号）
	//int16 sWorkpieceCoordNum; //工件坐标号


	double dSpindleSpeed; //主轴速度  G96，单位m/min  G97，单位r/min

	double dSecondSpindleSpeed;	//第二转轴速度/动力刀，r/min

	int8 cToolAxisMod;//刀具轴模式
	double dToolAxisFollowRatio;//刀具轴转速G51.2多边形加工（转速比）
	double dToolAxisPhase;//刀具轴相位角
	int8 cTwistCommand;//刀具轴扭转指令（G0,G1,G2）
	double dTwistAngle;//刀具轴扭转角

	double dFeedrate; //进给速度  mm/ms   长轴进给量（螺纹导程）

	double dLimitedSpindleSpeed;// G50 限制的主轴转速(r/min)  朱稳中 2022-1-17  -1

	int8 cChannelBufNum;//通道等待信号数组下标（-1为无效 范围1~MAX ）

	uint8 ucIOCommandArray[IO_MAXGROUP_IN_SINGLE_LINE]; //IO类型的M代码结构体数组 //朱稳中 2021-11-09   (UIOCommandArray 改为 ucIOCommandArray 2021-12-9)


	uint32 unDelayTime;	//延时,单位:ms   G04 P（XU）_

	uint32 unChipBreakDelayTime;//断屑暂停时间,单位ms      P
	double dChipBreakLength;	//断屑长度,单位:mm         Q
	bool bChipBreakFlag;		//断屑标志位,TRUE:开启 FALSE:关闭

	bool bG31Flag;//G31跳段

	SChannelCommand sChannelCommand; //通道指令信息（M97-M100）

//**************刘泓恺-G32解析所添参数*********2021—9—1**********
	double dJ;      //[短轴退尾量   G32参数模态  G92模态]  [G76 0～99999999×最小输入增量 非模态]
	double dK;      //[长轴退尾量   G32参数模态  G92模态]  [G76 0～99999999×最小输入增量 非模态]
	double dQ;      //[G32:1.起始角度 非模态]   [G76: 第1行.Q(△dmin):螺纹粗车时的最小切削量 0～99999,  第2行.Q(△d):第一次螺纹切削深度 1～99999999]
	uint16  usL;     //[G92、G33:多头螺纹头数1-99  模态]
	bool bRepeatG32Flag;    //TRUE:连续G32 FALSE:非连续G32bool bRepeatG32Flag;    //TRUE:连续G32 FALSE:非连续G32
	bool bG34RunFlag;       //G34运行标志位（用于螺纹结构体）
	bool bG34BackTailFlag;  //G34退尾标志位
	bool bG32LetterQ;//G32是否出现Q的标志位
	uint8 ucG32SubMode;		//G32的子类	用于区分G32.2/G32.3/G32.4 陈敏聪增 2023/6/13

	bool bLastLineMark;//最后一行标记位 选择停止功能使用  工件计数标记位(M30 M02 M99 第二个%)

	int8 cQuickHandleType;		//G代码快速处理类型
	int8 cMergedCounter;		//该行G代码合并的次数

	int8 G33RotateAxis;			//G33 旋转轴  可选A轴或C轴
	int8 G33TappingMode;		//G33攻牙模式 0:刚性攻牙 1:柔性攻牙

	SIOContrlInfo SIOContrlInfo;	//IO控制信息	//陈敏聪	2023/7/10
	double dHorMax;		//插补横轴方向的最大值   //计算平面圆弧的最值   洪冠鹏 2024/4/19
	double dVerMax;		//插补纵轴方向的最大值
	double dHorMin;		//插补横轴方向的最小值
	double dVerMin;		//插补纵轴方向的最小值
	bool bValidQ;		// Q值有效 2024/7/12


	//ZXD20251115
	double dG84Ratio;  //用于G84倍率计算


	//lxw:删除此变量，目标占用轴，不在G代码中获取，在运动预处理中获取
	//lxw:G代码解析后，在后续的刀补、倒角计算中会新增线段，新增的线段中仍需处理目标占用轴
	//lxw:所以在运动预处理时，统一对目标占用轴进行获取并占用
	VAL_STRUCT_TYPE	ucValStatus;	//有效轴（结构体轴）：加工程序段时产生运动的轴，该值由实体类型运动决定
	VAL_STRUCT_TYPE ucTurretAxis;	//刀塔轴（物理轴？）：加工程序段时会影响加工的轴，该值由机床结构决定

	double dChamferLen;     //L参数:自动倒角切角长度(mm), 0=无倒角请求
	double dRoundedRadius;  //D参数:自动倒圆半径(mm), 0=无倒圆请求
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	bool bHandwheelAuto; /**< 转换为 G0/G1 前保存的整段自动执行标记。 */
#endif

};



//G代码当前系统状态结构体（原：缓冲区写完后才进行更新	新：准备实时更新（目前只对偏置实时跟新））
typedef struct _SGCodeSysState
{
	uint8 ucToolID;			//刀具号（实时更新）
	uint8 ucToolCompNum;	//刀补号（实时更新）
	int16 sWorkpieceCoordNum;      //G54~G59 工件坐标号（实时更新）

	struct 
	{
		SPointDouble SToolSettingOffset; // zxr: 外部偏移，保留参数单位。
		SPointDouble SG50Offset;		//G50实时偏置值（实时更新）
		SPointDouble SToolWearOffset;    //刀具磨损偏移值（实时更新）弃用
		SPointDouble SToolLengthOffset; //刀具长度偏移值（实时更新）
		SPointDouble SWorkPieceOffset;   //工件坐标系零点偏移值,G54-G59,下标0-5依次对应G54-G59 (实时更新)
		SPointDouble STotalOffset;		//实时总偏置（实时更新）
	}SAllOffsetOfCoord; //各种坐标偏移集合，解析后需要传递到运动用作实时显示

	//刀具磨耗（XZ轴）初始化时从系统参数读取
	SPointDouble SToolWearPara[TOOL_NUM_MAX];


	//相对值编程标志
	bool bRelativeCoordinateProgramming;
	//极坐标编程标志
	bool bPolarCoordinateProgramming;
	//英制输入标志
	bool bImperialProgramming;
	//插补模式
	int8 cInterpolationState;//插补状态；	0：普通插补		1:极坐标状态		2.圆柱插补状态
	float fCylinderRadius;
	///////////////////////////////////////////////////////////////需要继承到G代码缓冲区/////////////////


	SGCodeModalInfo SModalInfo;			//模态指令
	//struct SGCodeNonModalInfo SNonModalInfo;	//非模态信息：只写入，用于记录上一运动指令值，判断运动指令是否发生变化

	SPointDouble SCurrPoint;		//当前坐标点位置（机械坐标）

	int8 cSpindleStatus;			//主轴状态
	double dSpindleSpeed;			//主轴速度(实时更新)

	int8 cSecondSpindleStatus;		//第二主轴状态
	double dSecondSpindleSpeed;		//第二主轴速度（实时更新）

	double dTappingSpeed;          // 添加G84攻丝转速

	int8 cToolAxisStatus;			//刀具轴状态
	double dLimitedSpindleSpeed;	//主轴转速限制
	double dFeedrate;          //进给速度  mm/ms


	//指令类型是否需要继承


	//断屑参数
	uint32 unChipBreakDelayTime; //断屑暂停时间,单位ms        P
	double dChipBreakLength;	//断屑长度,单位mm             Q
	bool bChipBreakFlag;		//断屑标志位 TRUE:开启 FALSE:关闭

	int16 sMb;//暂时保留，防报错
	int8 cMa;//暂时保留，防报错
	int8 cMh;//暂时保留，防报错


}SGCodeSysState;

//获取文本状态结构体
typedef struct _SGetTextStateInfo
{
	uint32 unGetLineNum;						//获取行号（仅用于获取文本）************************* unGetLineNum决定获取的目标行号
	uint32 unCurrLineNum;						//当前行号（当前解析的行号，用于报错）

	uint32 nEndLineNum;     //MDi结束判断使用 CYN


	int8 cOriginalStr[MAX_LENGTH_OF_LINE];		//行字符串内存
	uint8 ucOriStrIndex;						//行字符串内存读索引********************************* ucOriStrIndex决定从行字符串中获取程序段的索引
	int8 cStr[MAX_LENGTH_OF_LINE];				//程序段字符内存
	uint8 ucInsertIndex;						//程序段字符内存写索引
	bool bIgnored;								//括号忽略标志
	uint8 ucStep;								//当前处理程序文本步骤(初始值为0)

	int8* pCurrPtr;                                  //当前字符的指针，用于移动
	int8* pStartPtr;                                 //指针指向词法单元开头
	int8* pEndPtr;                                   //指针指向词法单元结尾的后一位

}SGetTextStateInfo;

//偏置结构体
typedef struct _SAllOffset
{
	//刀具偏置
	SPointDouble SToolOffset;
	//工件坐标偏置
	SPointDouble SWorkPieceOffset;
	//G50偏置
	SPointDouble SOffsetByG50;
	//总偏置
	SPointDouble STotalOffset;

}SAllOffset;



/**********************/
//函数声明
/**********************/

//初始化获取文本状态结构体
void initGetTextStateInfo(SGetTextStateInfo* pSGetTextStateInfo);

//初始化G代码系统状态结构体
void initGCodeSystemState(SGCodeSysState* pSGCodeSysState, uint8 ucChannelNum);

//初始化G代码缓冲区结构体
void initGCodeBufUnit(struct SGCodeParse* pSGCodeParseUnit);

//初始化模态结构体
void initGCodeModalInfo(SGCodeModalInfo* pSGCodeModalInfo);

//初始化扫描模态结构体
void initScanModalInfo(SScanModalInfo* pSScanModalInfo);

//初始化扫描非模态结构体
void initScanNonModalInfo(SScanNonModalInfo* pSScanNonModalInfo);

//初始化扫描结构体
void initScanContainer(SScan* pSScanContainer, bool bClearAll);

//获取当前G代码系统状态结构体
SGCodeSysState* getCurrSystemState(uint8 ucChannelNum, uint8 ucMod);

//处理程序文本
int8 processProgramText(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram);

//获取程序段
uint8 getProgramSegment(SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram);

//获取一行字符串
void getGCodeStr(int8* pString, uint8 ucIdx, uint16 usStrSize);

//处理第二个百分号
int8 processSecondPercentSign(uint8 ucMod, uint8 ucChannelNum);

//扫描程序段
bool scanSegment(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, SProgram* pCurrProgram, uint8 ucChannelNum);

//记录指令字
void recordLexicalUnit(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, double dLexicalUnitValue);

//记录G指令字
void recordGLetter(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, double dLexicalUnitValue);

//记录M指令字
void recordMLetter(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, SScan* pSScanContainer, double dLexicalUnitValue);

//判断是否为程序段结束标识符
int8 isSegmentFinishSign(const char* pChar);

//判断是否为行结束标识符
int8 isLineFinishSign(const char* pChar);

//更新当前总偏置
int8 TChangeTotalOffset(uint8 ucChannelNum, uint8 ucToolCompNum, int8 sGb, SPointDouble* STotalOffset, SPointDouble* SWorkPieceOffsetByG50);

//更新G代码通道状态
void updateGCodeChannelStatus(uint8 ucChannelNum);

//改变检查or赋值状态
void changeGCodeStatus(uint8 ucSyntaxOrAssign, uint8 ucChannelNum, uint8 ucTargetStatus, bool bReport, uint32 unLineNum);

//获取检查or赋值状态
uint8 getGCodeStatus(uint8 ucSyntaxOrAssign, uint8 ucChannelNum);

//从SRun中读取模态信息
void getModalInfoFromSRun(struct SGCodeModalInfo* pSGCodeModalInfo, uint8 ucChannelNum);

//获取程序段(part1)
uint8 getProgramSegment(SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram);

int8 processProgramText(uint8 ucMod, SGetTextStateInfo* pSGetTextStateInfo, uint8 ucChannelNum, SProgram* pSCurrProgram);


//获取一行字符串(part1)
void getGCodeStr(int8* pString, uint8 ucIdx, uint16 usStrSize);

//去除字符串中的空格(part1)
void eliminateBlank(int8* pString, uint16 usStrSize);

//比对字符串(part2)
bool isTagetStr_B(const int8* cStr, const int8* pCurrPtr);

int8 isSegmentFinishSign(const char* pChar);
int8 isLineFinishSign(const char* pChar);

//继承当前通道G代码系统状态(part4)
void inheritFromGCodeSystemState(struct _SGCodeSysState* pSGCodeSysState, struct SGCodeParse* pSGCodeParseUnit);

//更新当前通道G代码系统状态(part7)
void updataGCodeSystemState(struct _SGCodeSysState* pSGCodeSysState, struct SGCodeParse* pSGCodeParseUnit);

//G代码缓冲区转换
void convertGCodeBuff(struct SGCodeParse* pSGCodeBuff);
/* 仅刷新实体、指令位及有效轴；倒角模块复用，不再次转换模态。 */
void refreshGCodeGeometry(struct SGCodeParse* pSGCodeBuffUnit);

//移动G代码环形缓冲区写指针
void moveGCodeWritePtr(uint8 ucChannelNum);

//适配企业不从第一行开始运行
int32 scanRealProgramBeginLine(uint8 ucChannelNum, SProgram* pSCurrProgram);
int8 prepareProgramBeginLine(uint8 ucChannelNum);
#endif
