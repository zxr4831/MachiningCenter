#ifndef _COMMON_H
#define _COMMON_H


#include "Config.h"
//#include "libCommand.h"

#if STM32_MODE
#include "sys.h"
#endif
/***********************************************不可修改定义**********************************************************************************/
typedef signed char bool;  //布尔类型，变量命名示例bVar   //暂时改成bool  bool在<windows.h>内重定义
typedef signed char int8;  // -128~127，变量命名示例cVar
typedef unsigned char uint8;  // 0~255，变量命名示例ucVar
typedef signed short int int16;  // -32768~32767，变量命名示例sVar
typedef unsigned short int uint16;  //0~65535，变量命名示例usVar
typedef signed int int32;  // -2147483648~2147483647，变量命名示例nVar
typedef unsigned int uint32;  //0~4294967295，变量命名示例unVar
typedef signed long long int     int64;  //......，变量命名示例lVar  long long int 为8字节 修改2026.2.3
typedef unsigned long long int   uint64;  //......，变量命名示例ulVar





#define TURRET_AXIS_TEST 1  //刀塔轴测试

#if(TURRET_AXIS_TEST == 1)
#define TURRET_MAX_NUM 1         //加工中心保留单个刀具资源组，供旧内核偏置/资源管理使用
#define TOOl_LOCATION_MAX_NUM  15//刀塔的刀位数
#define	 TURRET_NULL 0//定义空的刀塔

#endif
#define POLAR_COORD_TEST 0 //加工中心开发基线关闭车铣极坐标扩展

#define DRAW_TEST		0	//画图测试	陈敏聪
#define MINIMUM_OF_INT8 -128   //8月18 英华
#define MAXIMUM_OF_INT8 127
#define MINIMUM_OF_UINT8 0 
#define MAXIMUM_OF_UINT8 255
#define MAXIMUM_OF_INT16 32767
#define MAXIMUM_OF_UINT16 65535

#define FILENAME_LENGTH_LIMIT 16

#define ARC_CW 1          //顺时针圆弧
#define ARC_ANTI_CW -1    //逆时针圆弧

#define FALSE	0
#define TRUE	1

#if !STM32_MODE
#define SET     1
#define RESET   0
#endif // !STM32_MODE
#define XSET 1
#define XRESET 0


#define IO_SET		SET
#define IO_RESET	RESET

#define HIGHLEVEL	1	//高电平	//陈敏聪	2023/7/10
#define LOWLEVEL	0	//低电平
#define PULSE_TYPE	2	//脉冲输出

#define HIGHLEVEL_VALID	1	//高电平有效	//陈敏聪	2023/7/10
#define LOWLEVEL_VALID	0	//低电平有效




#define	PI	3.1415926535

#define SERIALCOM_NUM_MAX  2  //系统串口通讯的总数

#define AXIS_NUM_MAX  6		//系统的运动轴总数(结构体)  //2021-09-15 朱稳中

#define AXIS_PHYSICS_NUM_MAX 6	//物理轴总数   2022-4-7 朱稳中

#define TOOL_NUM_MAX  20  //系统的刀具总数

#define TOOLNOSE_NUM_MAX   10//假象刀尖号码

#define ENCODER_NUM_MAX  4		//编码器的最大数目  //2021-10-21 刘泓恺

#define WORKPIECE_OFFSET_NUM	6 //刀具偏移数目

#define CHANNEL_NUM_MAX 1//通道数

#define SMACHINE_REFPOT_NUM 4 //机械参考点数目

 //2022-04-08 朱稳中
/**********轴号定义(第一层 映射)***********/
#define  AXIS_ALL 254 //定义所有轴为255
#define	 AXIS_NULL 255 //定义空的轴

#define	 ENCODER_NULL 255 //定义空的轴

#define  AXIS_X  0  //定义X轴为0
#define  AXIS_Y  1  //定义Y轴为1
#define  AXIS_Z  2  //定义Z轴为2
#define  AXIS_A  3  //定义A轴为3
#define  AXIS_B  4  //定义B轴为4
#define	 AXIS_C	 5  //定义主轴（C轴）为5

#define AXIS_HORIZONTAL		0 //横轴
#define AXIS_VERTICAL		1 //纵轴
/***************************************************指令类型定义*************************************************************/ //2021-09-16  朱稳中
//		配合getBits  setBits(定义在GeneralFunctions.h)使用   注意unCommandType目前为16位，超过需扩容
// 例:                               变量               指令值              指令地址
//		设置G00			setBits(unCommandType,  CMD_MOCTION_QUICKMOVE  ,CMD_MOCTION_ADDR)
//      读运动类指令	getBits(unCommandType ,							CMD_MOCTION_ADDR)

#define CMD_MOTION_ADDR					0x001F  // 运动类地址（G00~G03等等)     低八位0001 1111
#define CMD_MOTION_QUICKMOVE			0x0000	// 快速运动G00(默认)            低八位0000 0000
#define CMD_MOTION_LINE					0x0001	// 直线插补G01                  低八位0000 0001
#define CMD_MOTION_ARC					0x0002	// 圆弧插补G02                  低八位0000 0010
#define CMD_MOTION_ANTI_ARC				0x0006  // 圆弧插补G03                  低八位0000 0110
#define CMD_MOTION_SCREWCUT				0x0003  // 螺牙切削（G32）              低八位0000 0011
#define CMD_MOTION_SCREWCUT_LOOP		0x0004  // 螺牙切削循环（G92）          低八位0000 0100
#define CMD_MOTION_G33                  0x0005  // G33Z向攻牙循环（G33）        低八位0000 0101  //郭强9.17
#define CMD_MOTION_SCREWCUT_VP			0x0007  // 变螺距螺纹（G34）            低八位0000 0111
#define CMD_MOTION_G10					0x0008	// 自动补偿刀补功能             低八位0000 1000
#define CMD_MOTION_ELLIPSE_ARC			0x0009  // 椭圆插补指令G6.2             低八位0000 1001
#define CMD_MOTION_ELLIPSE_ANTI_ARC		0x000A  // 椭圆插补指令G6.3             低八位0000 1010


#define CMD_PLANE_ADDR					0x0060  // 插补平面指定 地址            低八位0110 0000
#define CMD_PLANE_G17					0x0020  // G17 指定插补平面XY(横:X 纵:Y)低八位0010 0000
#define CMD_PLANE_G18					0x0000  // G18 指定插补平面ZX(横:Z 纵:X)低八位0000 0000 (默认)
#define CMD_PLANE_G19					0x0040  // G19 指定插补平面YZ(横:Y 纵:Z)低八位0100 0000

#define CMD_SPINDLE_SPEED_ADDR			0x0080  // 主轴转速地址(G97 G96)        低八位1000 0000
#define CMD_SPINDLE_SPEED_CV			0x0000  // 恒转速度控制G97              低八位0000 0000  Const Vel
#define CMD_SPINDLE_SPEED_CLV			0x0080	// 恒线速度控制G96              低八位1000 0000  Const Line Vel

#define CMD_FEED_ADDR					0x0100	// 进给地址(G98、G99)           高八位0000 0001
#define	CMD_FEED_PER_MIN				0x0000	// 转分钟进给G98                高八位0000 0000
#define CMD_FEED_PER_R					0x0100	// 每转进给G99                  高八位0000 0001

#define CMD_PROGRAM_ADDR				0x0E00	// 程序状态地址M0/M1/M2/M30     高八位0000 1110
#define CMD_PROGRAM_CONTINUE			0x0000  // 程序继续  非M0/M1/M2/M30     高八位0000 0000
#define CMD_PROGRAM_STOP				0x0200	// 程序结束  M2 M30             高八位0000 0010
#define CMD_PROGRAM_STOP_M02			0x0A00	// 程序结束  M2                 高八位0000 1010  //2025.11.27
#define CMD_PROGRAM_OPTION_PAUSE		0x0400	// 条件暂停M1                   高八位0000 0100  //陈敏聪 2023/8/5
#define CMD_PROGRAM_PAUSE				0x0600	// 程序暂停  M0                 高八位0000 0110
#define CMD_PROGRAM_FINISH				0x0800  // 手动完成                     高八位0000 1000



#define CMD_G90G91_ADDR					0x1000	// G90G91地址                   高八位0001 0000
#define	CMD_G90G91_G90				    0x0000	// 绝对编程G90                  高八位0000 0000
#define CMD_G90G91_G91					0x1000	// 增量编程G91                  高八位0001 0000

//删除  不属于指令类型  属于有效值定义
//#define CMD_IS_G50_ADDR				0x0800  // 主轴转速SpindleSpeed是否为G50 Sxxx 高八位0000 1000
//#define CMD_IS_G50_FALSE				0x0000  // 主轴转速，由G96G97指定             高八位0000 0000
//#define CMD_IS_G50_TRUE				0x0800  // 主轴转速限制的速度，由G50指定      高八位0000 1000

#define CMD_TOOLCOMP_ADDR				0x6000  // 刀补地址                     高八位0110 0000
#define CMD_TOOLCOMP_CANCEL				0x0000  // 取消刀补G40(默认)            高八位0000 0000
#define CMD_TOOLCOMP_LEFT				0x2000	// 左刀补  G41                  高八位0010 0000
#define CMD_TOOLCOMP_RIGHT				0x4000	// 右刀补  G42                  高八位0100 0000


#define CMD_INTERPOLATION_MODE_ADDR		0x18000	// 插补地址                     高八位1000 0000  //2023.3.24 朱稳中
#define CMD_INTERPOLATION_COMMON		0x00000	// 普通插补                     高八位0000 0000
#define CMD_INTERPOLATION_CYLINDRICAL	0x08000 // 圆柱插补                     高16位 0000 0000 0000 0001
#define CMD_INTERPOLATION_POLAR			0x10000	// 极坐标插补                   高八位1000 0000
#define CMD_INTERPOLATION_POLAR_START	0x18000 // 极坐标插补开始               //隶属于极坐标插补 该宏表示G12.1那一行

#define CMD_POLAR_DIR_ADDR				0x20000 // 极坐标插补方向
#define CMD_POLAR_DIR_POSITIVE			0x00000 // 极坐标插补方向+
#define CMD_POLAR_DIR_NEGATIVE			0x20000 // 极坐标插补方向-


//#define CMD_WORKPIECE_ADDR      0x3800  //工件坐标       高八位0011 1000
//#define CMD_WORKPIECE_EXT		0x0000  //外部工件坐标系 高八位0000 0000
//#define CMD_WORKPIECE_G54       0x0800	//G54工件坐标系  高八位0000 1000
//#define CMD_WORKPIECE_G55       0x1000	//G55工件坐标系  高八位0001 0000
//#define CMD_WORKPIECE_G56       0x1800	//G56工件坐标系  高八位0001 1000
//#define CMD_WORKPIECE_G57       0x2000	//G57工件坐标系  高八位0010 0000
//#define CMD_WORKPIECE_G58       0x2800	//G58工件坐标系  高八位0010 1000
//#define CMD_WORKPIECE_G59       0x3000	//G59工件坐标系  高八位0011 0000

/*******************************************************有效值(轴)定义*********************************************************/ //2022-4-12 朱稳中
//(需要根据物理轴的数量或结构体轴的数量调整数据类型)
#define VAL_STRUCT_TYPE			uint8   //结构体有效值的数据类型    当前：无符号字符型
#define VAL_PHYSICIAL_TYPE		uint8	//物理轴的数据类型			当前：无符号字符型
#define PHYSICIAL_TYPE_MAX_VAL	0xFF	//物理轴数据类型对应最大值
#define VAL_VALID				1		//有效值

#if(TURRET_AXIS_TEST == 1)
#define VAL_TURRET_TYPE		uint8	//刀塔数据类型			当前：无符号字符型
#endif

//XYZABC 与结构体数组的定义相同
//#define  AXIS_X  0  //定义X轴为0
//#define  AXIS_Y  1  //定义Y轴为1
//#define  AXIS_Z  2  //定义Z轴为2
//#define  AXIS_A  3  //定义A轴为3
//#define  AXIS_B  4  //定义B轴为4
//#define  AXIS_C  5  //定义主轴（C轴）为5
#define VAL_STATUS_G50	AXIS_NUM_MAX  //最大轴号值的后一位
#define VAL_ABS_POINT	0x0



//用法：	setbit(ucValStatus,AXIS_X);
//			×××   getbit(ucValStatus,AXIS_X) == VAL_VALID ;
//			√√√	VAL_VALID == getbit(ucValStatus,AXIS_X) ;
//			getRealTimePoint(xxx,xxx,xxx,&ucValStatus);//根据有效轴读取实时点
//			updateRealTimePoint(xxx,xxx,xxx,&ucValStatus);//根据有效轴更新实时点


/*******************************************************IO类型定义*********************************************************/ //2021-10-26 朱稳中
//	uint32 unIOCommand; 

#define IO_PRE_ACTION					0	//前作用
#define IO_POST_ACTION					1	//后作用
#define IO_ALL_ACTION					2	//前后作用
#define IO_MAXGROUP_IN_SINGLE_LINE		6	//在一行G代码中M代码的最多个数

#define IO_NUM_ADDR						0x7F        //M代码号码地址  0111 1111
#define IO_TYPE_ADDR					0x80		//M代码作用类型地址 1000 0000

typedef struct _SIOContrlInfo
{
	int8	cOutputIOIndex;			//外部输出IO
	int8	cInputIOIndex;			//外部输入IO
	int8	cOutputControlType;	//外部输出IO控制的电平类型	1为输出高电平，0为输出低电平
	int32   nOutputRunTims;		//外部输出持续时间	单位:S
	int8	cInputValidType;	//外部输入IO电平有效类型	1为高电平有效，0为低电平有效
	int32	nInputDetectTime;	//输入等待检测时间	单位：S
	int32	nFeedingBackTime;	//上料退回延长时间	单位：S
	int8	ucFeeding_RepeatTimes;		//上料失败后重复执行次数
}SIOContrlInfo;		//IO控制信息结构体



/*************插补预处理结构体状态定义************/  //2022-03-18  朱稳中
#define INP_FINISHED			0		//插补完成
#define INP_OCCUPATION			8		//轴占用阶段(刀补的轴占用)
#define INP_BLOCKED				3		//阻塞	   G1+ 等待主轴加速完成
#define INP_RUNNING				4		//运行

/*********************运动状态定义********************/
//#define INP_FINISHED			0		//插补完成
//#define INP_OCCUPATION		8		//轴占用阶段(轴占用)
#define INP_INITIATION			1		//初始化
#define INP_ASSIGNMENT_START	2		//开始赋值
//#define INP_RUNNING			4		//运行
#define INP_PAUSING				5		//正在暂停
#define INP_DECELERATE			6		//减速
#define INP_STOP				7		//立即停


/******************曲线类型定义*********************/                  //朱稳中  2021-08-30
#define VEL_PLAN_T_CURVE		1	//梯形速度规划
#define VEL_PLAN_S3_CURVE		2	//S3形速度规划
#define VEL_PLAN_S4_CURVE		3	//S4形速度规划

#define VEL_PLAN_V1_CURVE		1	//梯形速度规划
#define VEL_PLAN_V2_CURVE		2	//S3形速度规划
#define VEL_PLAN_V3_CURVE		3	//S4形速度规划

#define DIST_REAVG_T_CURVE		1	//梯形均摊
#define DIST_REAVG_S5_CURVE		2	//5次均摊
#define DIST_REAVG_RATIO		3	//比例均摊

/*********************插补方向***********************/                  //朱稳中  2022-01-3
#define INP_GO_AHEAD	  1   //插补正方向
#define INP_GO_BACK		 -1	  //插补负方向

/*************************************************************可修改定义***************************************************************************/

#define SPINDLE_TYPE	SPINDLE_SIMULA  // 定义主轴类型
#define SPINDLE_SIMULA	0x1				//模拟主轴
#define SPINDLE_SERVO   0x0				//伺服主轴

#define MAX_FEED_RATIO	  1.5  // 进给倍率的最大值
#define MIN_FEED_RATIO	  0    // 进给倍率的最小值

#define MAX_SPINDLE_RATIO 1.2  // 主轴倍率的最大值
#define MIN_SPINDLE_RATIO 0.1  // 主轴倍率的最小值

#define MAX_HANDWHEEL_RATIO 1.0 //手轮模拟倍率最大值

/**************************************编码器类型定义**************************************************/
#define SPINDLE_ENCODER				1     //主轴编码器
#define COMMON_ENCODER				2     //通用编码器
#define HANDEWHEEL_ENCODER			3     //手轮编码器
#define SECONDSPINDLE_ANALOGENCODER	4	  //A轴模拟编码器

/*********************************刀补的状态//与底层轴占用有关***************************************/
#define TOOL_COMPENSATION_IDLE			0		//没有刀补
#define TOOL_COMPENSATION_BUILD			1		//刀补建立
#define TOOL_COMPENSATION_PROCEED		2		//刀补进行
#define TOOL_COMPENSATION_CANCEL		3		//刀补取消

/*************************************旋转轴 直线轴定义********************************************/
#define LINEAR_AXIS		0	//直线轴
#define ROTATION_AXIS	1	//旋转轴

/********************************处理方式定义****************************************/
#define DEALTYPE_RECAL					0	//操作：修正  不回写
#define DEALTYPE_RECAL_WRITEBACK		1	//操作：修正  回写
#define DEALTYPE_NO_PROCESS				2	//操作：不修正
#define DEALTYPE_USER_INPUT				3   //操作：用户输入

/********************************电机相位选择****************************************/
#define MOTOR_NEGATIVE_PHASE -1  //电机反向
#define MOTOR_POSITIVE_PHASE 1   //电机正向

/********************************编码器相位选择****************************************/
#define ENCODER_NEGATIVE_PHASE -1  //编码器反向
#define ENCODER_POSITIVE_PHASE 1   //编码器正向

/********************************主轴锁紧定义****************************************/
#define STATUS_GET_CLAMP_INSTRUCTION  16	//状态，已获取主轴刹车夹紧信号，未执行
#define STATUS_GET_UNCLAMP_INSTRUCTION  17	//状态，已获取主轴刹车松开信号，未执行

#define STATUS_CLAMP_EXECUTING  18		//状态，已获取主轴刹车夹紧信号，正在执行
#define STATUS_UNCLAMPEXECUTING 19	//状态，已获取主轴刹车松开信号，正在执行

#define STATUS_CLAMPED	  20	//状态，主轴刹车已夹紧  后作用
#define STATUS_UNCLAMPED  21	//状态，主轴刹车已松开  前作用

/********************************螺距补偿表定义****************************************/
#define SCREWDEVIATION_NUM_MAX 100

/********************************前瞻模式定义****************************************/
#define	LOOKAHEAD_LINEAR_AXIS_PRECISION_MODE	1		//1:直线轴精度模式
//#define LOOKAHEAD_OPTIMIZE_TRANSITION_MODE	2	    //2:直线轴段间优化过渡模式    与企业讨论过渡模式由G代码处处理,因此删掉此模式，将多轴模式改为2  (2023/05/19郭强)
#define	LOOKAHEAD_MULTI_AXIS_MODE				2		//2:多轴(含旋转轴)模式

/********************************插补模式定义****************************************/
#define	INTERPOLATION_LONGAXIS_MODE					1	//1：	长轴插补
#define INTERPOLATION_FIXED_PARAMETER_MODE			2	//2:	固定参数扩展线位移插补
#define	INTERPOLATION_ADAPTIVE_PARAMETER_MODE		3	//3:	自适应参数扩展线位移插补
/***********************************刀具设备*******************************************/
#define	TOOL_RAWTOOL_MODE	0//排刀机
#define TOOL_TURRET_MODE	1//刀塔模式

/***********************************旋转轴插补方式*******************************************/ //朱稳中 2023.5.19
#define ROTATION_INTERPOLATION_SHORT_DISTANCE		0   //短距离优先
#define ROTATION_INTERPOLATION_ACCORDING_DIRECTION	1   //根据插补方向决定

/***********************************主轴停模式*******************************************/ //朱稳中 2023.5.19
//系统主轴 停止模式
#define SPINDLE_STOP_MODE_PRECISESTOP_MODE	0   //   准停模式
#define SPINDLE_STOP_MODE_COMMONSTOP_MODE	1   //   减速停模式


/*
--------------------------------------------------结构体定义--------------------------------------------------------------
*/
enum ENTITY  //实体类型，DOT:0,L1NE:1, CIRCLE:2, ARC:3
{
	DOT = 0,
	L1NE = 1,  //注意是1不是I
	CIRCLE = 2,
	ARC = 3,
	THREAD = 4,
	TAPPING=5,
	OTHERTYPE = 255
};

//  2021-08-20			朱稳中
enum SYSTEMSTATUS  //系统状态
{
	SYS_IDLE,			//空闲:0
//	SYS_PREPARE,		//系统准备状态 1   2023.4.3 朱稳中
	SYS_RUN,			//运行:1
	SYS_PAUSE,			//暂停:2
	SYS_PAUSE_TO_RUN,	//从暂停到运行:3
	SYS_RUN_TO_PAUSE,	//从运行到暂停:4
	SYS_RESET,			//复位状态:5        2021-11-09			朱稳中
	SYS_EMERGENCY_STOP  //急停:6
};

enum RUNMODE
{
    EDIT_MODE,      // 编辑模式：NC程序的编写与修改
    AUTOMATIC_MODE, // 自动加工模式：连续执行NC加工作业
    MDI_MODE,       // 手动数据输入模式(MDI)：即时执行单段指令
    HANDWHEEL_MODE, // 手轮模式(MPG)：旋转手摇脉冲发生器精密对刀
    HAND_MODE,      // 点动模式(JOG)：面板按键控制轴连续/步进进给
    RE0_MODE        // 回零模式(HOME/REF)：各轴返回机床机械原点
};

typedef struct
{
	//double dIX;
	//double dIY;
	//double dIZ;
	//double dIC;

	double dArray[AXIS_NUM_MAX];
}SVector;     //单位矢量，double描述类型


typedef struct
{
	//double dX;
	//double dY;
	//double dZ;
	//double dC; //C轴 (数值从0 - 359.9999...)

	double dArray[AXIS_NUM_MAX];
}SPointDouble;  //点,double描述类型

typedef struct _SPoint
{
	double dX;
	double dZ;

}SPoint;

typedef struct
{
	uint8 ucBaudRateMode;	//0~3;{"9600","19200","38400","57600"}
	uint8 ucDataBMode;		//0~1;{"7","8","",""}
	uint8 ucStopBMode;		//0~1;{"1","2","",""}
	uint8 ucParityBMode;	    //0~2;{"NO","ODD","EVEN",""}

}SSerialComParaInfo;  //串口通讯参数结构体


typedef struct
{
	uint8 ucToolID;     //刀具ID          //磨损补偿号
	uint8 ucToolNoseID;  //假想刀尖偏移号
	double dToolRadius;  //刀具半径       //磨损半径
	double dToolNoseWear;//刀尖磨损
	SPointDouble SToolCoord;  //刀具坐标  //XZ磨损

}SToolParaInfo;  //刀具参数信息   （刀具磨损信息)


typedef struct
{
	int8 cHomeDir;	//回零方向
	//2022.4.28 朱稳中
	uint8 ucAxisType;   //轴类型   (直线轴1 / 旋转轴0)

	bool bShowByDiameter;//坐标轴直径显示

	float fDistPerCircle; //旋转轴时  一圈的位移量

	double dPulseToMm;	//脉冲当量，每个脉冲多少mm（或者角度）
	uint16 usMmToPulse;	//每mm（或者1转）多少个脉冲

	double dVs;			//单轴开始速度 mm/ms  (旋转轴 0.1r/ms)
	double dVmax;		//单轴最高速度 mm/ms
	double dVe;			//单轴结束速度 mm/ms

	//double dAcc;	    //单轴加速度 mm/ms2
	//double dDec;	    //单轴减速度 mm/ms2

	double dVeerDeltaVMax;	 //单轴转向允许变化的最大速度 mm/ms
	double dVeerAMax;        //单轴段间非反向时过度最大加速度  mm/ms2（°/ms2）   郭强 2022/06/13
	double dCoefOfAngleToLength;    //旋转轴角度到长度转换系数  郭强 2022/06/13（文献中暂未找到该系数的取值，暂使用1做测试）

	double dClearance;	//单轴间隙值

	//2021-12-17新增  朱稳中
	double dSysPlanTMaxAcc;//T型曲线加减速度
	double dSysPlanTMaxDec;

	double dSysPlanS3MaxAcc;//S3型曲线加减速度
	double dSysPlanS3MaxDec;

	double dSysPlanS4MaxAcc;//S4型曲线加减速度
	double dSysPlanS4MaxDec;


	uint8 ucAccVelPlanType; //加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	uint8 ucDecVelPlanType; //减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型

	/**************手动参数********************/
	// 快速
	double dManualQuickVel; //慢速速度mm/min
	double dManualQuickAcc; //慢速加速度mm/ms2
	double dManualQuickDec; //慢速减速度mm/ms2

	//慢速
	double dManualSlowVel; //慢速速度mm/min
	double dManualSlowAcc; //慢速加速度mm/ms2
	double dManualSlowDec; //慢速减速度mm/ms2
	/***********丝杠反向间隙补偿参数*********///郭强2022-1-20
	double dRevClearanceCompDist;      //该轴反向间隙补偿量

}SAxisParaInfo;  //各运动轴的参数信息

/****** 新增模态信息结构体 *******/  //2021-8-16  冯英华
struct SGCodeModalInfo
{
    int8 cGa, cGb, cGc, cGd, cGe, cGf, cGg, cGh ;
	/// G代码分类参数，
	/// cGa：运动类 G00，G01，G02，G03，G32,G33,G76,G92						MOCTION
	/// cGb：工件坐标系 G54-G59												COOR
	/// cGc：定义插补平面 G17-G19
	/// cGd：刀具补偿 G40-G42												TOOLCOMP
	/// cGe：单位选择：英制/公制 G20，G21									UNITS   INCHS MM
	/****** 新增模态组 *******/  //2021-9-13  冯英华
	/// cGf: 恒线速控制 G96、恒转速控制 G97									            SPINDLE_SPEED
	/// cGg: 给定每分进给速度 G98 、以毫米/转为单位给定切削进给速度 G99	、断削指令G4.9	FEED
	/// cGh: 绝对编程G90/ 相对编程G91  默认G90
	int8 cDecimalVal;  //针对cGa组，判断小数后一位，例如：G32.1时，值为1， G32.2时，值为2

};

//通道指令信息结构体
typedef struct SChannelCommand//通道指令
{
	int8 ucType;//类型 ①通道等待（0）  ②通道启动（1）  ③检查通道状态M96 （2）   无效（-1）
	struct SChannelData//①通道信息
	{
		uint8 ucTargetChannelNum;// ①通道等待（位表示） ②通道启动 ③检查通道状态M96
		int32 cFileNameVal;//文件名值 （M97）
	}SChannelData;
}SChannelCommand;

/****** 新增非模态信息结构体 *******/  //2021-8-16  冯英华
struct SGCodeNonModalInfo
{
	int8 cGz;  //无:0;  G04 G50  G71 G72 G73
	/// cGz：非模态组别G04(程序暂停)、G50
	int8 cGx;	//G80
	/// cGx: 非模态组别，用于G83~G89的模态取消
    int8 cMa, cMc, cMe, cMf, cMh ,cMi, cMj, cMk ;
    int16 sMb;
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
};








struct SToolCompensation  //刀补后产生的结构体
{
    int8 cFilename[FILENAME_LENGTH_LIMIT];//文件名
	uint32 unGCodeLineNum;    // 解析到第几行G代码

	uint32 unCommandType;  //指令类型    	2021-09-17 朱稳中 uint8 ->uint16   unCommandType ->unCommandType
	VAL_STRUCT_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12
	VAL_STRUCT_TYPE	ucRelaVal;		//相对值标识变量  2022-4-20  朱稳中
	bool bG50Appear;//G50临时坐标系传入标志位 2023-8-3 洪冠鹏
	SPointDouble* pSG50Offset;			//lxw添加，用于记录G50偏置
	uint8 ucToolCompNum;
	int8 cWorkpieceCoordNum; //工件坐标号码 (54-59) 为其他值时不使用工件坐标 //朱稳中  2021-12-9

	enum ENTITY EntityType;		//实体类型，LINE:1, CIRCLE:2, ARC:3

	SPointDouble SStartPoint;	//起点，单位：mm
	SPointDouble SEndPoint;		//终点，单位：mm
	SPointDouble SCenterPoint;	//圆心，单位：mm

	int8 cArcDir;	//圆弧方向，顺时针=1，逆时针=-1
	double dRadius;			//半径，单位：mm
	double dArcStartAngle;  //起始角度，单位：rad
	double dArcEndAngle;    //终止角度，单位：rad
	float  dEllipseFactorA; // 椭圆参数A  2023.3.29 朱稳中
	float  dEllipseFactorB; // 椭圆参数B  2023.3.29 朱稳中
	float  fCylinderRadius; // 圆柱半径 mm  2023.3.29 朱稳中


	double dG84Ratio;       //用于G84倍率 2025.11.15

	SToolParaInfo SToolPara;	//刀具参数
	//uint8 ucWearGroupNum;  //刀具磨损组号   朱稳中 2021-12-6

	double dSpindleSpeed; 	//主轴转速 r/min

	double dSecondSpindleSpeed;	//第二转轴速度//动力刀

	int8 cToolAxisMod;//刀具轴模式
	double dToolAxisFollowRatio;//刀具轴转速G51.2多边形加工（转速比）
	double dToolAxisPhase;//刀具轴相位角
	int8 cTwistCommand;//刀具轴扭转指令（G0,G1,G2）
	double dTwistAngle;//刀具轴扭转角


	double dLimitedSpindleSpeed;// G50 限制的主轴转速(r/min)  朱稳中 2022-1-17

	double dFeedrate; //进给速度  mm/ms

	int8 cChannelBufNum;//通道等待信号数组下标（-1为无效 范围1~MAX ）
	SChannelCommand sChannelCommand; //通道指令信息（M97-M100）

	uint8 ucIOCommandArray[IO_MAXGROUP_IN_SINGLE_LINE]; //IO类型的M代码结构体数组 //朱稳中 2021-11-09   (UIOCommandArray 改为 ucIOCommandArray 2021-12-9)

	uint32 ulDelayTime;	//延时,单位:ms

	uint32 unChipBreakDelayTime;//断屑暂停时间,单位ms      P
	double dChipBreakLength;	//断屑长度,单位:mm         Q
	bool bChipBreakFlag;		//断屑标志位,TRUE:开启 FALSE:关闭

	int8 cPointType;//1(G50)  2(G10)
	uint8 uG10ToolID;//G10的刀具号
	SPointDouble SPointInfo; /**< G50为绝对偏置；G10为待执行磨耗增量，单位mm，消费后清零X/Z。 */

	bool bG31Flag;

	//刀补后追加
	double dLength;		//线段长度（或弧长），单位：mm
	double dPreDist;	//已走的距离

	SVector SStartVector;  //起点矢量
	SVector SEndVector;   //终点矢量   郭强 8.8 SEndtVector 改为SEndVector 此矢量需要是单位向量

	

	//速度前瞻后添加
	double dAcc;	    ///<加速段加速度	mm/ms2
	double dDec;	    ///<减速段加速度	mm/ms2
	double dVeer;     //衔接点夹角约束速度  mm/ms

	double dArcMaxNormalAccVmax;//圆弧法向加速度约束下的最大速度    郭强 2022/6/16
	double dArcMaxChordErrorVmax;//圆弧弓高误差约束下的最大速度     郭强 2022/6/16

	double dCommonVMaxFromSingleAxis;     //G1，G2,G3复合运动最大速度由各插补轴计算得到        郭强 2022/6/16
	double dCommonVsMinFromSingleAxis;    //G1，G2,G3复合运动最小起跳速度由各插补轴计算得到   郭强 2022/6/16
	double dCommonVeMinFromSingleAxis;    //G1，G2,G3复合运动最小终止速度由各插补轴计算得到   郭强 2022/6/16

	double dLowerLimitSpeed; //由长轴反算得到的复合运动切削进给段间低速下限值（郭强 2022/11/22 ）
	uint8  ucLongAxisNum;    //长轴轴号  G2G3指令若有第三轴时，当第三轴是长轴，此时轴号为长轴，当第三轴不为长轴，长轴轴号为(AXIS_NULL)（郭强 2022/11/22 ）
	uint8  ucArcThirdAxisNum;   //圆弧第三轴存在，但是不是长轴，此时第三轴轴号保存在此变量中

	double dUncorrectedFeedrate; //未修正的进给速度  mm/ms
	double dMinVs;				 //起点速度,单位:mm/ms
	double dMinVe;				 //终点速度,单位:mm/ms
	bool bLookAheadVParaCalcOverFlag; //圆弧弓高、法向加速度、速度修正、速度加速度信息、速度再次修正、该段最低起始速度和终点速度计算完毕标志位(郭强2022/11/27)


	double dVmax;   //最大速度    mm/ms 12-30 朱稳中
	uint8 ucAccVelPlanType; //加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	uint8 ucDecVelPlanType; //减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型

	double dVs;				 //起点速度,单位:mm/ms
	double dVe;				 //终点速度,单位:mm/ms

	uint8 ucAxisNumOfNeedLookAhead;	//需要前瞻的插补单轴轴号 洪冠鹏 2024/5/27
	int8 cAxisDirOfNeedLookAhead;	//需要前瞻的插补单轴方向 洪冠鹏 2024/5/27
	//**************刘泓恺-G32解析所添参数*********2021—9—1**********
	double dJ;  //短轴退尾量   G32参数模态  G92模态
	double dK;  //长轴退尾量   G32参数模态  G92模态
	double dQ;  //起始角度     G32参数非模态  //或者是主轴准停的值
	double dF;  //长轴的导程
	uint8  ucL;  // G92G33 多头螺纹头数1-99  模态
	bool bRepeatG32Flag;    //TRUE:连续G32 FALSE:非连续G32bool bRepeatG32Flag;    //TRUE:连续G32 FALSE:非连续G32
	bool bG34RunFlag;       //G34运行标志位（用于螺纹结构体）
	bool bG34BackTailFlag;  //G34退尾标志位
    bool bG32LetterQ;//G32是否出现Q的标志位
	uint8 ucG32SubMode;		//G32的子类	用于区分G32.2/G32.3/G32.4 陈敏聪 2023/6/13

	bool bHasVeerFlag;  //8.15 郭强 希望刀补追加，且追加时均为FALSE
	bool bLastLineMark;//最后一行标记位 选择停止功能使用
	//陈敏聪-G10所添参数*********2022/8/19
	//uint8 uG10ToolID;
	//陈敏聪-快速处理所添参数*********2022/4/17
	int8 cQuickHandleType;	//快速处理类型
	int8 cMergedCounter;	//该行G代码融合的次数

	int8 G33RotateAxis;			//G33 旋转轴  可选A轴或C轴
	int8 G33TappingMode;		//G33攻牙模式 0:刚性攻牙 1:柔性攻牙

	SIOContrlInfo SIOContrlInfo;	//IO控制信息	//陈敏聪	2023/7/10
	bool bG65SystemMacroWait;
	double dHorMax;		//插补横轴方向的最大值   //计算平面圆弧的最值   洪冠鹏 2024/4/19
	double dVerMax;		//插补纵轴方向的最大值
	double dHorMin;		//插补横轴方向的最小值
	double dVerMin;		//插补纵轴方向的最小值
	bool bValidQ;		// Q值有效 2024/7/12
	VAL_STRUCT_TYPE ucG28ZeroingValAxis;//G28需要回零的轴
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	bool bHandwheelAuto;
#endif

};

struct SInput // 2022.3.14  朱稳中  用户输入参数结构体  暂存参数
{
	VAL_STRUCT_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12
	VAL_STRUCT_TYPE	ucRelaVal;		//相对值标识变量  2022-4-20  朱稳中

	double dDistance;		//总距离  插补距离	mm
	double dTargetDist;		//目标距离
	double dPreDist;		//速度规划前   长轴!!已走距离  (不为0时,暂停回写进去的段，不获取当前点作为起点）
	double dCurLeftDist;	//当前留下未移动的距离

	double dG84Ratio;   //用于G84倍率 2025.11.15
	double dFeedrate;	//进给速度  mm/ms 12-30 朱稳中
	double dFeedRatio;	//进给倍率  12-30 朱稳中
	double dAcc;	    //加速段加速度	mm/ms2
	double dDec;	    //减速段加速度	mm/ms2
	double dVs;			//初始速度	mm/ms
	//double dVe;		//最终速度	mm/ms
	double dPrimalVe;   //从前瞻继承的最终速度 mm/ms

	
	double dArcMaxNormalAccVmax;//圆弧法向加速度约束下的最大速度    2022/11/29郭强
	double dArcMaxChordErrorVmax;//圆弧弓高误差约束下的最大速度     2022/11/29郭强
	double dCommonVMaxFromSingleAxis;     //G1，G2,G3复合运动最大速度由各插补轴计算得到     2022/11/29郭强
	double dUncorrectedFeedrate; //未修正的进给速度  mm/ms   2022/11/29郭强
	double dMinVs;				 //起点最小速度,单位:mm/ms       2022/11/29郭强
	double dMinVe;				 //终点最小速度,单位:mm/ms       2022/11/29郭强
	uint8 ucAxisNumOfNeedLookAhead;	//需要前瞻的单轴轴号     2024/5/27洪冠鹏

	uint8 ucAccVelPlanType;		//加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	uint8 ucDecVelPlanType;		//减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	uint8 ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数

	SVector SStartVector;  //起点矢量（单位向量）

	SPointDouble SStartPoint;	//起点，单位：mm
	SPointDouble SEndPoint;		//终点，单位：mm
	float  dEllipseFactorA; // 椭圆参数A
	float  dEllipseFactorB; // 椭圆参数B
	double dCoordRotation;// // 坐标系旋转弧度  rad
	float  fCylinderRadius; // 圆柱半径 mm  2023.3.29 朱稳中

	int8 cInterpDir;	//插补方向 (手轮方向)

	int8 cArcDir;		//圆弧方向，顺时针=1，逆时针=-1     ( C轴时为主轴方向)
	double dRadius;		//半径，单位：mm
	SPointDouble SCenterPoint;	//圆心，单位：mm

	int8 cDealType;	//处理方式
	uint8 ucTconstForTrapezoid; //三角形修调后匀速时间占总周期的百分比 （ % ） 洪冠鹏 2024/3/22
	double dHorMax;		//插补横轴方向的最大值   //计算平面圆弧的最值   洪冠鹏 2024/4/19
	double dVerMax;		//插补纵轴方向的最大值
	double dHorMin;		//插补横轴方向的最小值
	double dVerMin;		//插补纵轴方向的最小值
#if HANDWHEEL_SIM_DISTANCE_ENABLE
	bool bHandwheelAuto;
#endif

};



struct SPlan   //2022.3.14   朱稳中  速度规划的执行结构体    单轴 G1 G2G3 G32 G33 结构体内分别包含此结构体
{
	double dDistance;		//总距离  插补距离	mm
	double dTargetDist;		//目标距离
	double dPreDist;		//速度规划前   已走距离
	double dCurLeftDist;	//当前留下未移动的距离

	double dAcc;	    //加速段加速度	mm/ms2
	double dDec;	    //减速段加速度	mm/ms2
	double dVs;			//初始速度	mm/ms
	double dVe;			//最终速度	mm/ms
	double dPrimalVe;   //从前瞻继承的最终速度 mm/ms

	uint8 ucAccVelPlanType;		//加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	uint8 ucDecVelPlanType;		//减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
	uint8 ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数

	int8 cInterpDir;	//插补方向 (手轮方向)   //主轴转动方向

	double dTc;			//插补周期	ms
	double dVmax;		//最大速度	mm/ms  dFeedrate

	double dDistAcc;		//加速段位移
	double dDistConst;	    //匀速段位移
	double dDistDec;		//减速段位移

	uint32 unT1;	//时间节点T1
	uint32 unT2;	//时间节点T2
	uint32 unT3;	//时间节点T3
	uint32 unT4;	//时间节点T4
	uint32 unT5;	//时间节点T5

	double dT1;	//时间节点T1
	double dT2;	//时间节点T2
	double dT3;	//时间节点T3
	double dT4;	//时间节点T4
	double dT5;	//时间节点T5

	uint32 unDistReT1;		//分摊余量的时间节点T1
	uint32 unDistReT2;		//分摊余量的时间节点T2
	uint32 unDistReT3;		//分摊余量的时间节点T3

	double dDistRemaining;		//残余距离

	double dDistNewAddT;	//新增一个周期走的距离

	//梯形平摊参数
	double dTrapezoidAvgAcc;   //平摊梯形的加速度
	double dTrapezoidAvgVel;   //平摊梯形的最大速度

	//5次平摊参数
	double dQuinticAvgA;		//五次项系数A
	double dQuinticAvgB;		//五次项系数B
	double dQuinticAvgC;		//五次项系数C
	uint8 ucQuinticAvgTFlag;	//五次周期不足标志位

	//比例系数平摊参数
	double dAverageFactor;		//比例平摊的比例系数

	uint32 unCurrentT;			//插补的当前时间点
	uint8 ucTconstForTrapezoid; // 三角形修调后匀速时间占总周期的百分比 （ % ） 洪冠鹏 2024/3/22
	SPointDouble SMaxVal;//圆弧轨迹上最大值点集合   2024/4/12
	SPointDouble SMinVal;//圆弧轨迹上最小值点集合
};


struct SInterpolationPre    // 2022.3.14  插补预处理函数
{
	struct SChannel* pSChannel; // 通道指针  (父)

    int8 cFilename[FILENAME_LENGTH_LIMIT];//文件名	//当前正在运行的
	int8 cOriginFilename[FILENAME_LENGTH_LIMIT];//源文件名
	uint32 unGCodeLineNum;		// 解析到第几行G代码

	uint32 unCommandType;		//指令类型
	//VAL_STATUS_TYPE	ucValStatus;	//有效值（轴）类型	2022-4-12

	int8 cChannelBufNum;//通道等待信号数组下标（-1为无效 0已执行 1~MAX 存在且未执行）
	SChannelCommand sChannelCommand; //通道指令信息（M97-M100）
	uint8 ucIOCommandArray[IO_MAXGROUP_IN_SINGLE_LINE];		//IO类型的M代码结构体数组

	//double dSpindleSpeed;			//主轴速度    g_SInterpolation的主轴转速小于0时无效 ，C轴读主轴转速后将g_SInterpolation的转速置为无效
	//double dLimitedSpindleSpeed;	// G50 限制的主轴转速(r/min)

	double dSecondSpindleSpeed;		//第二根旋转轴速度或动力刀速度

	uint8 ucToolNum;			//刀具号
	//uint8 ucWearGroupNum;		//刀具磨损组号
	uint8 ucToolCompNum;
	int8 cWorkpieceCoordNum;	//工件坐标号码 (54-59) 为其他值时不使用工件坐标
	SPointDouble SWorkPieceOffsetByG50;		//G50临时坐标系偏移量 2023-8-3洪冠鹏
	bool bG50Appear;	//G50临时坐标系传入标志位 2023-8-3洪冠鹏
	SPoint SToolCMPByG10;//G10设置的刀具磨损补偿量
	uint8 uG10ToolID;  //G10的刀具号 2023/10/23
	bool bG65SystemMacroWait;


	uint8 ucRunStep;		//步骤
	uint8 ucInpStatus;			//插补状态，INITIATION    ASSIGNMENT_START    RUNNING    PAUSING    FINISHED

	uint32 ulDelayTime;			//延时,单位:ms



#if(TURRET_AXIS_TEST == 1)

	VAL_STRUCT_TYPE ucOccupideValStatus; //通道已占用的结构体轴 （非刀塔轴+刀塔轴）
	VAL_STRUCT_TYPE ucOccupideTurretAxis;//已经占用的属于刀塔的结构体轴（刀塔轴）
	VAL_TURRET_TYPE	 ucOccupideTurret;//通道已经占用的刀塔
#else
	VAL_STRUCT_TYPE ucToolAxis;//刀补有效轴
	uint8 ucToolCompensationStatus;//刀补状态
	VAL_STRUCT_TYPE ucLastValStatus; //上个指令的有效轴
#endif

	//AL_STRUCT_TYPE ucPauseValStatus; //上个指令的有效轴
	SPointDouble SPausePoint;// 保存暂停的点

	struct SInput SUserData; //用户（输入）参数

	int8 cInterpDir; // 插补方向  （手轮方向）

	float preciseStopDegree;// 准停角度

	bool bLastLineMark;//最后一行标记位 选择停止功能使用
	bool bRunPostIO;  //运行前作用IO
	bool bUnStoptableLine;//当前段为不可停下的行 朱稳中 2023.5.19  当前段为G34 单节运行不停
	bool bValidQ;  // Q值有效?  朱稳中 2023.5.19
	uint32 unChipBreakDelayTime;//断屑暂停时间,单位ms      P
	double dChipBreakLength;	//断屑间隔长度,单位:mm     Q
	bool bChipBreakFlag;		//断屑标志位,TRUE:开启 FALSE:关闭

	VAL_STRUCT_TYPE ucG28ZeroingValAxis;//G28需要回零的轴   cyn 2024.12.1

};


struct _SRun  //运行结构体
{   
	struct SChannel* pSChannel; // 通道指针  (父)
    int8 cFilename[FILENAME_LENGTH_LIMIT];//文件名
	uint32 unGCodeLineNum;//解析到第几行G代码
	uint32 unCommandType;  //指令类型 2023-03-27 朱稳中 uint16 ->uint32   unCommandType ->unCommandType

	VAL_STRUCT_TYPE ucValStatus;//有效值
	SPointDouble SAbsCoord;     //当前绝对坐标
	SPointDouble SLastAbsCoord; //上一次的绝对坐标

	int16 sAxisDeltaPulse[AXIS_NUM_MAX];	//各个轴的位移增量  单位:pulse(输出指令单位)
	int16 sLastDeltaPulse[AXIS_NUM_MAX];	//各个轴上周期的位移增量 (相当于记录了当前速度)

	double dCommonFeedRatio; //进给倍率
	double dQuickMoveFeedRatio;//快速运动进给倍率
	double dSpindleSpeedRatio;//主轴转速倍率
	double dHandWheelSpeedRatio;//手轮模拟倍率

	double dCurrentFeedRate;//当前的进给速度 （合成运动）
	double dCurrentSpeed;//当前速度 （合成）
	//int8 cPolarHorDir;//极坐标横轴方向

	//double dCurrentSpeed;//当前速度 2022.4.9  朱稳中
	/******************显示相关*********************/
	uint8 ucToolNum;	  //刀具号   /   朱稳中 2021-11-13
	//uint8 ucWearGroupNum;  //刀具磨损组号   朱稳中 2021-12-6
	uint8 ucToolCompNum;	//刀补号	陈敏聪 23-2-11
	int8 cWorkpieceCoordNum; //工件坐标号 (54-59) 为其他值时不使用工件坐标  //朱稳中2021-12-9
	SPointDouble SCurrentOffset;  //当前总偏移  绝对坐标=机械坐标-总偏移(包括对刀，磨损，G54，EXT)//朱稳中2021-12-8
	SPointDouble SWorkPieceOffsetByG50;		//G50临时坐标系偏移量 2023-8-3洪冠鹏
	int8 cSoftLimitedOn; //软限位开关标志位 2024/4/12 洪冠鹏
};


//螺纹切削模式
enum ThreadCuttingMode
{
	COMPLETE_FOLLOW_MODE,   //主轴完全跟随模式（不允许丢弃脉冲）
	INCOMPLETE_FOLLOW_MODE  //主轴不完全跟随模式（允许丢弃脉冲）

};

//加工时间  函数 start  mark  stop
struct _SMachineTimer {
	bool isTimerBegined;	//计时开始标志
	uint32 unStartTime;		//启动时间			(启动时记录,或加工完最后一行时重新记录timeji)
	uint32 unCurProcessTime;// 加工时间(当前件) (加工完最后一行时,或停止时计算) timeji-unCurProcessTime
};



union SSystemUnion
{
	struct     //系统参数结构体
	{
		///////////////////////////////////////////////////
		// 配置参数

		int fVersion;	    //版本号       2024.1.15
		uint8 ucMachType;	//机器类型

		SSerialComParaInfo SSerialComPara[SERIALCOM_NUM_MAX]; //系统的各个串口通讯参数

		uint8 ucAxisMapArray[CHANNEL_NUM_MAX][AXIS_NUM_MAX];//通道映射 2022.4.23 朱稳中

		uint8 ucEncoderMapArray[CHANNEL_NUM_MAX][ENCODER_NUM_MAX];		//手轮映射 2023.8.1	陈敏聪

		double dInterpolationTime; //插补时间，单位：ms

		uint8 ucLookAheadNum;		//前瞻段数

		SToolParaInfo SToolPara[CHANNEL_NUM_MAX][TOOL_NUM_MAX];  //刀具参数		//刀长	2023/2/15

		SToolParaInfo SWearPara[CHANNEL_NUM_MAX][TOOL_NUM_MAX];//刀具磨损参数  朱稳中 2022-1-6  刀具参数个数*刀具磨损组数


		uint8 ucToolNum;	//刀具号	//2023.2.8	陈敏聪新增
		uint8 ucToolCompNum;	//刀补号
		uint8 ucToolDevie;	//刀具设备，排刀机/刀塔机
		uint8 ucToolChange_TrigDelayTime;	//换刀触发信号维持时间
		uint16 usToolChange_AlarmTime;	//换刀时间过长时间
		uint16 usToolChange_LockDelayTime;	//换刀电磁阀夹紧延时(企业要求，防止换刀过快)
		uint16 usToolChange_ReadOrSendIODelayTime;		//读取刀号IO或伺服使能/失能延时时间

		/*****************************主轴刹车******************************************/
		int8 cSpindleClampStatus[CHANNEL_NUM_MAX];	//主轴刹车锁紧状态

		/*****************************单表螺距补偿******************************************/
		bool bScrewCompEnable[AXIS_PHYSICS_NUM_MAX];	//螺距误差使能位
		uint8 ScrewCompGap[AXIS_PHYSICS_NUM_MAX];		//螺距补偿间隔  mm
		uint16 sScrewCompHomeNum[AXIS_PHYSICS_NUM_MAX]; //原点的位置号

		/*****************************G83toG89系统参数******************************************/
		int16 sG83toG89Para_RetractionLength;		//NC_DATA_PARAMETER_089 G83toG83退刀参数_退刀长度 um	CMC 2023/4/3

		/*****************************双表螺距补偿******************************************/
		bool bBilateralScrewEnable[AXIS_PHYSICS_NUM_MAX];	//双向螺距误差使能位
		uint8 ucBilateralScrewGap[AXIS_PHYSICS_NUM_MAX];		//螺距补偿间隔  mm
		int8 cAheadScrewCompVal[AXIS_PHYSICS_NUM_MAX][SCREWDEVIATION_NUM_MAX];  //正向或单表的补偿值  um   走一个间隔需要补偿的量  走10mm 测得9.98mm  则填入20um
		int8 cBackScrewCompVal[AXIS_PHYSICS_NUM_MAX][SCREWDEVIATION_NUM_MAX];  //反向补偿值  um   走一个间隔需要补偿的量  走10mm 测得9.98mm  则填入20um
		int16 sScrewReferenceNum[AXIS_PHYSICS_NUM_MAX]; // 参考点位置号
		float fScrewReferencePoint[AXIS_PHYSICS_NUM_MAX]; // 参考点坐标

		SAxisParaInfo SAxisPara[AXIS_NUM_MAX]; //各运动轴的参数

		double dSysMinVel;//系统允许起跳速度  郭强 8.8 追加
		double dSysMaxVel;//系统最大速度  朱稳中  2021-09-15

		//下方两个圆弧约束速度优先级要高于最大倍率，即最高倍率下也不能超出如下约束
		double dArcMaxNormalAcc;   //圆弧最大法向加速度约束   郭强 2022/06/13
		double dArcMaxChordError;  //圆弧最大弓高误差约束     郭强 2022/06/13

		//各种曲线的加速度  朱稳中2021 - 11-28   (合成运动)
		double dSysPlanTMaxAcc;//T型曲线加减速度
		double dSysPlanTMaxDec;

		double dSysPlanS3MaxAcc;//S3型曲线加减速度
		double dSysPlanS3MaxDec;

		double dSysPlanS4MaxAcc;//S4型曲线加减速度
		double dSysPlanS4MaxDec;

		/*************新增变量***************///朱稳中 2021-08-10

		////G00 曲线参数  朱稳中2021 - 11-28   //2021 -12-17  快速运动参数从单轴参数中获取
		//uint8 ucQuickMoveAccVelPlanType;//加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
		//uint8 ucQuickMoveDecVelPlanType;//减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型

		//G01 G02 G03曲线参数朱稳中2021 - 11-28
		uint8 ucCommonAccVelPlanType; //加速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型
		uint8 ucCommonDecVelPlanType; //减速段  速度曲线类型，1：梯形，2：3次S型，3：4次S型

		uint8 ucDistReAverageType; //平摊曲线类型，1：梯形，2：5次，3：比例系数

		uint8 ucRotationAxisInterpolationMode; // 旋转轴插补方式    //朱稳中 2023.5.19
		//ROTATION_INTERPOLATION_SHORT_DISTANCE			0   //短距离优先
		//ROTATION_INTERPOLATION_ACCORDING_DIRECTION	1   //根据插补方向决定

		uint8 ucSpindleStopMode;   //主轴停方式         //朱稳中 2023.5.19
		//SPINDLE_STOP_MODE_PRECISESTOP_MODE	0   //   准停模式
		//SPINDLE_STOP_MODE_COMMONSTOP_MODE		1   //   减速停模式

		/*************新增变量***************///冯英华 2021-08-19

		int16 sXAxisMax, sYAxisMax, sZAxisMax; //各轴的运动范围 软限制 soft limit
		int16 sXAxisMin, sYAxisMin, sZAxisMin;

		uint32 unEncoderOneCyclePulse; //编码器转一圈发送脉冲

		double dG33AvoidEmptyDistance;  //G33避空距离   郭强   2021-12-10

		double dG33RevCompDistance;         //G33反转补偿     郭强   2021-12-14

		bool   bG33RecalDecPointFlag;       //G33重算减速点   郭强   2022-01-10

		double dG33MaxVel;                  //G33最大攻牙速度 郭强    2022/8/22

		/***********丝杠反向间隙补偿参数*********///郭强2022-1-7（G33反向补偿时规划参数也使用此参数）
		double dRevClearanceCompVs;             //反向间隙补偿初速度（末速度按苏总要求暂取和初速度相同）
		double dRevClearanceCompVmax;           //反向间隙补偿最大速度
		double dRevClearanceCompAcc;            //反向间隙补偿最大加速度(苏总：默认T型加减速度规划)
		double dRevClearanceCompDec;            //反向间隙补偿最大减速度
		/****************************************************************************************/

		/***** 2022/10/25新增系统参数--郭强*****/
		uint8 ucLookAheadMode;		                         //前瞻模式   （1：直线轴精度模式  2：直线轴段间优化过渡模式  3：多轴(含旋转轴)模式)
		uint8 ucInterpolationMode;                           //插补模式    (1：长轴插补 2：固定参数扩展线位移插补 3：自适应参数扩展线位移插补)
		double dSysLowerLimitSpeed;                          //切削进给段间低速下限值
		uint8 ucGearOfSysLowerLimitSpeed;                    //切削进给段间低速下限值档位(1:100%、2:125%、3:150%)
		double dPercentageOfG33EnterLeadCorrection;          //G33攻牙攻入导程修正百分比(99%-101%)
		double dPercentageOfG33ReturnLeadCorrection;	     //G33攻牙退刀导程修正百分比(99%-101%)
		/***************************************/

		double dG33ReturnSpindleRatio;//G33主轴反转转速百分比 /***** 2023/5/27郭强*****/

		uint32 unG33SpindlePauseTimeBeforeRev; //G33主轴反转前暂停时间(ms)    2023.5.8    郭强
		/***************************************/

		float fSpindleStopDegree; //主轴准停角度  范围 :0->359   相对于Z信号而言  朱稳中  2021-12-9

		float fSpindleSpeedDeviation;//主轴转速允许的偏差 0%~100%   (0.00~1.00)

		float fG99SpindleSpeedDeviation;//主轴转速允许的偏差 0%~100%   (0.00~1.00)

		uint16 usSpindleAvgPeriod;//主轴平均转速检测周期数

		uint16 usHandleWheelAvgPeriod; //手轮模拟采样时间
		float fHandleWheelSpeedMax;//手轮模拟达到最大转速的速度  pulse/ms
		double dSimulaVmax; // 手轮模拟非G0最大速度  mm/ms  2022.11.7

		double dRotationAxisG0SpeedMax;//旋转轴G0 最大速度  朱稳中 2023.3.28

		uint8 ucPolarInterpolationAxis[2]; //极坐标插补轴号   0-->横轴  1-->纵轴   取值范围 0~AXIS_NUM_MAX 2023.3.27朱稳中

		/*************新增变量***************///朱稳中 2021-11-09
		uint32 unSpindlePreDelayTime;   //主轴开延时（ms)
		uint32 unSpindlePostDelayTime;  //主轴关延时（ms)
		uint32 unSpindleEnableTime;		//主轴使能延时（ms）
		uint32 unSpindleDisableTime;    //主轴取消使能延时（ms）
		uint32 unChuckOpenDelayTime;	//夹头开延时（ms)
		uint32 unChuckCloseDelayTime;	//夹头关延时（ms)
		uint32 unAutoFeedOpenDelayTime[8];				//送料气缸 开延时（ms)
		uint32 unAutoFeedCloseDelayTime[8];				//送料气缸 关延时（ms)
		uint32 unSpindleSpeedUpAlarmTime;   //主轴加速报警时间（ms)

		uint32 unSpindleLockPreDelayTime;		//主轴刹车锁紧前延时（ms)
		uint32 unSpindleLockPostDelayTime;		//主轴刹车锁紧后延时（ms)
		uint32 unSpindleUnLockPreDelayTime;    //主轴刹车松开前延时（ms)
		uint32 unSpindleUnLockPostDelayTime;    //主轴刹车松开后延时（ms)

		//uint32 ucProcessTimeTotal;//总加工时间
		//uint32 unWorkPieceTotal;//总工件数
		//uint32 unCurWorkPieceNum; // 当前工件数
		uint32 unNeededWorkPieceNum;// 需求工件数


		/*************新增变量***************///陈敏聪 2023-3-15
		int16 sGcode_MultiLineHandleMaxNum;				//G代码多行处理最大行数
		int16 sToolComp_MultiLineHandleMaxNum;			//刀补多行处理最大行数
		int16 sInterpolation_MultiLineHandleMaxNum;		//插补多行处理最大行数

		int8 bGcodeQuickHandleModeEnabled;						//G代码快速处理功能	0 关  1 开

		//开机默认参数
		bool bDefaultG99; // 开机默认G99?

		struct
		{
			SPointDouble  SZeroPtNC;          //机床零点,假设机械零点与机床参考点重合 ，均为(0,0,0)，初始化为(0,0,0),不可改变
			SPointDouble  SToolSettingOffset; //外部工件坐标零点（相对于机床零点）
			SPointDouble  SWorkPieceOffset[WORKPIECE_OFFSET_NUM]; //工件坐标系零点偏移值（相对于对刀点），G54-G59,下标0-5依次对应G54-G59
			SPointDouble STotalOffset;         //总的偏移量 注意:总偏移的X是半径值
			SPointDouble SWorkPieceOffsetByG50;		//G50临时坐标系偏移量 2023-8-3洪冠鹏
		}SZeroPointOfCoordSys[CHANNEL_NUM_MAX];   //数控系统各种零点的集合  注意：除了总偏移量外，其它X的偏移量均是直径值  //2021-11-17朱稳中  从Gcode.h移到系统参数结构体

		enum ThreadCuttingMode ThreadCutMode;     //螺纹切削模式（在按键上可以选择模式）
		uint16 usEncoderLinesNum[ENCODER_NUM_MAX]; //编码器线数

		struct
		{
			uint8 ucIsNullArr[75];   // 8*75 = 600 ，每一位表示一个宏变量是否为空
			float dValArr[600];      //600个宏变量的值
		}SMacroVariable;

		struct //报警及限位方式
		{
			int8 positype;  			//正限位类型
			int8 negatype;  		 	//负限位类型
			int8 servowarn;      //伺服报警
			int8 servowarntype;  //伺服报警类型
			int8 servowarnway;   //报警处理方式
			int8 origin_dire; 	  //回原点方向
			int8 origin_way;  	  //回原点方式
			int8 soft_pos;  			//正软位检测
			int8 soft_neg;  			//负软位检测
			int8 pos_port;  			//正限位端口
			int8 neg_port;	      //负限位端口
			int32 seekspeed;	    //找限位速度
			int32 leavespeed;	  //找原点速度
			int32 leavedistance; //离开原点值
			double allowerror;	  //原点允许误差
			double seekdistance;	//回到原点值
			double pos_soft;//正软限位
			double neg_soft;//负软限位
			int8 hw_pos;   //正硬限位开关
			int8 hw_neg;   //负硬限位开关
			int8 iomode;   //回零端口模式
			int8 buff[20];
		}MOTOR_WARN[6];

		struct  //加工信息
		{
			uint32 totalartifacts;//总工件数
			uint32 BreakLine;//断点行号
			uint32 BreakPoint; //断点序号
			uint32 gcodeline; //加工行数
			uint8 toolnum;        //当前刀号
			double dCommonFeedRatio; //进给倍率  12-30 朱稳中
			double dQuickMoveFeedRatio;//快速运动进给倍率 2022-1-7
			double dSpindleSpeedRatio;//主轴转速倍率
			double SpindleSpeed;      //
			double dCurrentFeedRate;
			uint32 dHandWheelFeedRatio;//手轮倍率
			uint8  dHandmode;          //手动模式快慢速
			int8 buff[20];
		}MOTOR_WORK[CHANNEL_NUM_MAX];
		uint8 UseSpindleEncoder[9];	//0使用编码器  1不使用编码器

		int8 ucMotorphase[9];//电机相位
		int8 ucEncoderphase[3];//编码器相位

		struct
		{
			uint8 ucRegister_K[40];    //K寄存器
			uint16 ucRegister_DT[100];  //定时器预置值
			uint16 ucRegister_DC[100];  //计数器预置值
			uint8 ucRegister_D[1000];  //数据寄存器
		}SPlcRegister;//plc掉电保护的寄存器


		struct
		{
			uint8 on_offsim;//图形模拟显示
			uint8 on_offinfor;//加工资讯设定
			uint8 coord_status;//坐标页状态
		}syspage;
		struct
		{
			uint32 warntime;//超时报警时间
			uint32 worknum;//工件限件
			uint8 Daliao_warn;//打料报警
			uint8 coord_status;//润滑报警
			uint8 Chuck[2];//夹紧启动
			uint8 Chuck_Spindle[2];//夹头松开
			uint8 Chuck_status[2];//夹紧状态
			uint8 Axismode;//多轴模式
			uint8 CoordFont_size;//坐标字体大小
			uint8 MDI_mode;//MDI运行模式
			uint8 SpindleChuck[2];//主轴夹紧启动    //重做此功能，此参数需要注释  2023/11/25 洪冠鹏
			uint8 StopChuck[2];//暂停时夹头动作
			uint8 Tool_Mode1;//刀具页模式1
			uint8 Tool_Mode2;//刀具页模式2
			uint8 Tool_Mode3;//刀具页模式3
			int8 M20_mode;//M20输出模式 1-复位急停不关闭M20输出  0-关闭
			int8 RunMODE;//企业增加的 2023.11.17
			uint8 RAMCOPYMODE;
			int8 Daily_Para;

			uint32 Daily_Num;
			uint32 Iot_Id;
			int8 buff[20];
		}userset;

		double dHandWheelVmax;  //手轮的最大速度
		double dHandWheelAcc;   //手轮的加减速
		double dG32SacBase;     //螺纹车削的避空距离  2023.5.13
		double dJDefault;       //螺纹短轴默认退尾量  2023.6.6
		double dKDefault;       //螺纹长轴默认退尾量  2023.6.6

		double pro_origin[AXIS_PHYSICS_NUM_MAX];//企业加的
		uint8 ucG32InputMode;    //G32系统参数来源，0为加工系统参数，1为对应单轴的系统参数  2023.5.19
		uint8 ucEmergencyStopType; //急停IO类型

		int8 Spindlewarn[2];		//主轴报警	//企业定义的变量	//2023/7/25
		int8 Spindlewarntype[2];	//主轴报警类型	//企业定义的变量
		int8 M98TextCalltype;       //M98是否打开调用文本

		uint32 FluctuationPercentage; //波动百分比//企业定义的变量//2023/8/14
		uint32 FluctuationTime; //波动时间//企业定义的变量//2023/8/14
		uint8 ParameterPassword; //参数密码////企业定义的变量//2023/8/14
		int resize_multiple; //放大缩小图形调整倍率//企业定义的变量//2023/8/14
		double Angle_tolerance;//角度允许误差//企业定义的变量//2023/10/16

		uint32 Speed[2];//企业的变量 2023/11/17
		int16 acceleration[2];//企业的变量 2023/11/17
		double dRecoveryMaxVel;//恢复加工最大速度 2023/11/24
		double SpindleChuck[2];//检测相应转速 达到转速后开启检测主轴运转与夹头松开互斥时的报警功能 2023/11/25
		double dSimulaMaxAcc;//手轮模拟最大加减速度 2023/11/2
		uint8 dhandwheelcontrol;//企业的变量 2023/12/20
		double dG32MaxVelocity;						//G32螺纹专用速度		//陈敏聪
		bool bIsStopSpindleAndCoolantwhileM30;		// M30执行关主轴同时关冷却		默认为0，关主轴同时关冷却
		bool bIsStopSpindleAndCoolantwhileReset;	// 复位执行关主轴同时关冷却		默认为0，关主轴同时关冷却
		bool bIsG50ProgameHasFinished;  //G50所在程序完成标志位 洪冠鹏 2023/12/23
		uint32 startline;//启动按键模式 企业添加的参数 2024.1.18
		int16 PulseAllowablEerror[2];//主轴脉冲允许偏差 企业变量 2024.1.25
		uint8 ucTconstForTrapezoid; // 三角形修调后匀速时间占总周期的百分比 （ % ） 洪冠鹏 2024/3/22
		
		uint8 ucDetectTimeforSignlWait;//通道等待指令检测时间  陈育楠

		bool bM01ResetAllChannel; //M01报警后复位所有通道 TRUE:复位所有通道 FALSE:仅复位当前通道 洪冠鹏 2024/6/5

		int32 njixesosz_zuzbaox[6];		//机械手坐标X//企业定义的变量//2024/6/5
		int32 njixesosz_zuzbaob[6];		//机械手坐标b//企业定义的变量//2024/6/5
		uint32 unjixesosz_times[6];	//机械手延时//企业定义的变量//2024/6/5
		int32 njixesosz_zuweizi[6];		//力矩到位位置坐标//企业定义的变量//2024/6/5

		SPointDouble SMachine_REFPOT[SMACHINE_REFPOT_NUM];//机床参考点集合

#if(TURRET_AXIS_TEST == 1)

		uint8 ucChannelToTurretmap[CHANNEL_NUM_MAX][TURRET_MAX_NUM];//通道与刀塔的映射
		uint8 ucTurretToAxisMap[TURRET_MAX_NUM];//刀塔与有效轴的映射
		uint8 TurretToToolMap[TURRET_MAX_NUM][TOOl_LOCATION_MAX_NUM];//刀塔与刀位的映射

#endif
		uint32 unLogSavePageNum;
		bool bSpindleAlarm;   //编码器相位和速度突变开始标志
		double dSimulaG0Vmax; // 手轮模拟G0最大速度  mm/ms  2022.11.7
		double dSimulaMaxDec; // /手轮模拟最大减速度 2025/4/19
		bool bNowStopflag;//急停下，各轴停止方式，TRUE为立即停，FALSE为减速停2025.5.7

		double dAngleXY1;    //Y1轴相对X轴的倾斜角度（°）		[虚拟Y轴机专用变量]
		double dX1Movement;  //X1物理轴的移动量（mm）			0-999.999
		double dY1Movement;  //Y1物理轴的移动量（mm）			0-999.999
		bool bUseVirtualY;	 //是否开启虚拟Y轴					TRUE:开启 FALSE:关闭
		double ZeroDistance[6];

		uint8 ucPWMAxisMapArray[CHANNEL_NUM_MAX][AXIS_NUM_MAX];//PWM映射 2025.10.31熊骏

		double dSysPlanSpindleAcc;//主轴运动加减速度，用于区分分度运动  //增加 2026.4.17
		double dSysPlanSpindleDec;

		bool bSetG7456mmTOum;//设置G74 G75进刀量 G76螺纹粗车最小切削量

		uint8 ncProtec1venoomnab1e;//防护门功能(o:无效1:有效)//企业增加 2026.6.11
		uint8 ucProtectiveDoorSignal;//防护门检测信号（0：接通 1：断开）有效
		uint8 ucProtectiveDoorControl;//防护门开时(0:关闭 1:不关)主轴
		uint8 ucPressurecheckEnable;//压力检测输入信号(o不检测，o不检测
		uint8 ucPressureAlarmStopMode;//压力报警停机方式(o立即停机，1完成当前工件后停机)

		double dSpindleVs;  //主轴起始速度
		double dSpindleVe;  //主轴末速度

		uint8 ucCloudStorageSelection;//网盘模式选择//企业定义的变量  20260908
		uint8 ucToolCompInputRange;//刀补可输入范围//企业定义的变量//20260911
		uint16 unG84DefaultDelayTime; //G84孔底暂停默认时间，单位：ms


#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
		int nHandwheelSimDistancePerTick;
#endif

	}SSystemParaInfo;

	uint8 ucChar[15 * 1024];
};

struct SdPiece	//时钟芯片数据存储
{
	uint32 ucProcessTimeTotal;//总加工时间
    uint32 unWorkPieceTotal;	//总工件数
    uint32 unCurWorkPieceNum;	//当前工件数
    double oldTimePoint[AXIS_NUM_MAX];	//实时点坐标
};


struct SEncoder   //编码器结构体+
{
	uint16 usEncoderLinesNum; //编码器线数
	uint16 usEncoderCirclePulseNum;  //编码器旋转一圈的脉冲数
	int16 sEncoderCircleNum;  //编码器运行的圈数
	int8 cEncoderDir;       //编码器的方向，旧（0为正1为负）新（1为正 -1为负）
	uint16 usEncoderValue;  //本次读取的编码器的数值
	uint16 usEncoderLastValue; //编码器上一次的数值
	uint16 usDelta; //两次编码器的差值
	int64 lEncoderAbsValue;  //编码器绝对数值
#if !STM32_MODE
    /* VS 理想反馈的亚计数余量，simulateGetEncoder 独占更新；全局对象启动时为零。
     * 随编码器机械相位保留，通道复位、G84 换段不清零；不进入板端或持久参数。 */
    double m_dSimCountRemainder; /**< 编码器计数，圆整后余量范围 [-0.5, 0.5)。 */
#endif

	//螺纹类指令切削，发送同步信号所用
	double dEncoderNumDeviation;  //该插补周期编码器的位置与理想位置的偏差（pulse）

	//手轮所需参数
	uint8 ucWheelMultiple;   //手轮倍率
	int16 dDelataAccmulate;  //编码器差值累加
	uint8 AXISNum;          //控制的轴号
	uint8 ucChannelNum;     //控制的通道号
	int32 sAxisAccmulate[AXIS_NUM_MAX];     //各个运动轴未规划的脉冲数
	double dAxisCurrentSpeed[AXIS_NUM_MAX]; //各个运动轴的当前速度
	int32 sPulseForChangeDie[AXIS_NUM_MAX]; //换向时，所存储的脉冲数
	SPointDouble SStartPoint;  //手轮运动起点

    double dAxisAccmulateRemainder[AXIS_NUM_MAX];    //储存手轮反馈不足电机1个脉冲的脉冲数，如手轮一格是0.001°，电机一个脉冲是0.036°，所以需要36格才发一个脉冲
	double dPulseAccmulate[AXIS_NUM_MAX];  //存储各轴未达到脉冲发送条件的脉冲  2025-6-16 熊骏 洪冠鹏

#if HANDWHEEL_STOP_BOUNDARY_ENABLE
	/*
	 * 每轴独立，切换手轮选轴不打断其他轴的制动。
	 * 正常态：净距离是相对 SStartPoint 的有符号目标；
	 * 制动态：净距离是相对本次锁定刹停点的有符号待走量。
	 * 距离单位为 mm 或 degree，与本轴 SPlan 一致，不是脉冲数。
	 *
	 * 例：原方向 +1，段目标 100，刹停点行程 70，反向输入 35：
	 * 正常 net 先变成 65；触发时 net=65-70=-5；制动中再输入 +8，
	 * net=+3；停在 70 后从零速正向走约 3（计入指令量化尾差）。
	 * 普通输入永远只做 net+=input，切勿根据 net 的符号立即切带速轴方向。
	 *
	 * 启用时上面的 dAxisCurrentSpeed 保存周期末规划速度大小，非周期平均值；
	 * SPlan.unCurrentT 为固定制动/短段计数；正常长段每拍重规划后取第 1 拍。
	 * 修改编译开关会改变 SEncoder 布局，必须全量重编所有引用它的模块。
	 */
	bool bHandwheelBraking[AXIS_NUM_MAX];   //FALSE 正常；TRUE 锁定减速到零
	int8 cHandwheelMoveDir[AXIS_NUM_MAX];   //0 无活动方向，+1/-1 规划方向，制动时不变
	double dHandwheelNetDist[AXIS_NUM_MAX];//唯一的正反输入抵消量；正=坐标正向
	double dHandwheelStopDist[AXIS_NUM_MAX];//done+brakeDist，非仅剩余制动距离，制动时锁定
#endif

};

struct  SRunTime   //作用时间
{
	uint32 unSpindlePreRunTime;   //主轴开作用时间
	uint32 unSpindlePostRunTime;  //主轴关作用时间
	uint32 unSpindleEnableRunTime; //主轴使能作用时间
	uint32 unSpindleDisableRunTime;//主轴取消使能作用时间
	uint32 unChuckAOpenRunTime;	//夹头开作用时间
	uint32 unChuckACloseRunTime;	//夹头关作用时间
	uint32 unChuckBOpenRunTime;	//夹头开作用时间
	uint32 unChuckBCloseRunTime;	//夹头关作用时间
	uint32 unAutoFeedOpenRunTime[8];	//送料气缸 开作用时间
	uint32 unAutoFeedCloseRunTime[8];	//送料气缸 开作用时间
	uint32 unSpindleLockRunTime;   //主轴刹车开作用时间
	uint32 unSpindleUnLocktRunTime;  //主轴刹车关作用时间

	uint32 unSpindleSpeedUpAlarmRunTime;  //主轴加减速超时报警作用时间

	uint32 unToolChange_TrigRunTime;
	uint32 unToolChange_AlarmTime;
	uint32 unSampleIntervalRunTime;
	uint32 unFeedingRunTime;		//上料读取输入作用时间
};





/********************************************.c中定义的全局变量*************************************************/


//extern struct SInterpolationPre g_SInterpolation;  //插补预处理结构体

extern volatile union SSystemUnion g_SSystemParaUnion;  //系统参数结构体

extern uint16 g_usReloadValue[AXIS_PHYSICS_NUM_MAX];//各个轴发送脉冲的预装载值

extern uint16 g_usCompareValue[AXIS_PHYSICS_NUM_MAX];//各个轴发送脉冲的比较值

extern uint16 g_usCounterValue[AXIS_PHYSICS_NUM_MAX];//各个轴发送脉冲的计数器值 //增加2026.2.7

extern int16 g_sAxisDeltaPulse[AXIS_PHYSICS_NUM_MAX];//各个轴发送的脉冲数

extern int8 g_cMotionDirection[AXIS_PHYSICS_NUM_MAX];//各个轴的运行方向 //增加2026.2.7

extern struct SEncoder g_SEncoder[ENCODER_NUM_MAX];  //手轮是定时器4，数组的最后一个

extern	enum RUNMODE g_RunMode;//运行模式

extern volatile VAL_PHYSICIAL_TYPE g_ucAxisOccupation;//轴占用变量  数据类型需要根据物理轴数量进行改变  当前物理轴个数为6   使用变量uint8

extern volatile VAL_PHYSICIAL_TYPE g_ucAxisExactEnable;// 物理轴的使能状态

extern double g_dRealTimePointArray[AXIS_PHYSICS_NUM_MAX];//实时坐标  对应物理轴号   郭强2022.8.24，公开权限用于G33虚拟运行
#if CotrexF4 == 1
extern volatile unsigned long* pAxisEnable[AXIS_PHYSICS_NUM_MAX];// 物理轴对应的使能IO
#else
uint8 pAxisEnable(uint8 AXIS_PHYSICS);
uint8 pAxisEnableSet(uint8 AXIS_PHYSICS, uint8 value);
#endif 
extern struct SdPiece g_sdtime;	//企业定义的变量

#define g_SSystemPara g_SSystemParaUnion.SSystemParaInfo
#define g_SZeroPtOfCoordSys g_SSystemParaUnion.SSystemParaInfo.SZeroPointOfCoordSys


/****************************************映射相关****************************************/

//	清除坐标
int8 clearRealTimePoint();

//	获取实时点坐标
int8 getRealTimePoint(double* cArray, int nArraySize, const uint8* ucAxisMap, const VAL_STRUCT_TYPE* ucValStatus);

//	获取单轴实时点坐标
int8 getSingleAxisRealTimePoint(double* dOutPut, uint8 ucAxis, const uint8* ucAxisMap);

//	更新实时点坐标
int8 updateRealTimePoint(double* cArray, int nArraySize, const uint8* ucAxisMap, const VAL_STRUCT_TYPE* ucValStatus);

//	更新单轴实时点坐标
int8 updateSingleAxisRealTimePoint(double* dInPut, uint8 ucAxis, const uint8* ucAxisMap);


/////////根据通道号获取坐标
//	获取实时点坐标
int8 getRealTimePointByChannel(uint8 ucChannelNum, double* cArray);

//	获取单轴实时点坐标
int8 getSingleAxisRealTimePointByChannel(uint8 ucChannelNum, double* dOutPut, uint8 ucAxis);



/****************************************轴管理(轴互斥)相关****************************************/

//初始化轴的使能状态
int initAxisEnableStatus();

//设置轴的当前使能状态
int8 setAxisEnableStatus(uint8 ucPhysicalAxis, uint8 ucStatus);

//	结构体轴转换为物理轴
int8 structToPhysical(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize, VAL_PHYSICIAL_TYPE* ucOutput);

//	轴占用查询（物理轴）
bool axisManagerCheck(VAL_PHYSICIAL_TYPE ucSign);

//	轴占用函数（物理轴）
bool axisManagerOccupy(VAL_PHYSICIAL_TYPE ucSign);
/* 苏向标 2026-10-14：仅用于设备准备和收尾的控制权接口，不代表允许运动。 */
int8 reserveAxisControl(VAL_STRUCT_TYPE ucSign, const uint8* pMapArray, VAL_STRUCT_TYPE* pOccupation);

//	轴释放函数（物理轴）
bool axisManagerRelease(VAL_PHYSICIAL_TYPE ucSign);


//	轴占用查询（通过结构体）
int8 axisManagerCheckByStruct(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize);

//	轴占用函数（通过结构体）
int8 axisManagerOccupyByStruct(VAL_STRUCT_TYPE ucSign, const uint8* ucAxisMap, int nSize, VAL_STRUCT_TYPE* pSaveVal);

//	轴释放函数（通过结构体）
int8 axisManagerReleaseByStruct(VAL_STRUCT_TYPE ucSign, const  uint8* ucAxisMap, int nSize, VAL_STRUCT_TYPE* pSaveVal);

//2023.4.27 朱稳中
//	轴占用函数（通过通道）
int8 axisManagerOccupyByChannel(VAL_STRUCT_TYPE ucSign, uint8 ucChannelNum, VAL_STRUCT_TYPE* pSaveVal);

//	轴释放函数（通过通道）
int8 axisManagerReleaseByChannel(VAL_STRUCT_TYPE ucSign, uint8 ucChannelNum, VAL_STRUCT_TYPE* pSaveVal);



//	打印变量各个位
int8 printfBits(int nSign, int nSize);

//加工时间检测
int8 machineTimerStart(uint8 ucChannelNum);

//单件加工标记
int8 machineTimerMark(uint8 ucChannelNum);

//总加工时间自增
int8 machineTimerInterp(uint8 ucChannelNum);

//加工时间清零
int8 machineTimerClear(uint8 ucChannelNum);

//计件功能
int8 calWorkpieceNum(uint8 ucChannelNum);

//初始化计件
int8 initWorkpieceNum(uint8 ucChannelNum);

// STM32模式下原子操作性地获取全局变量的值
int32 atomic_read(volatile int32* ptr);

//STM32模式下原子操作性地给取全局变量赋值
void atomic_write(volatile int32* ptr, volatile int32 value);

//原子操作性地对 ptr指向的内存中值进行加法运算
int32 atomic_add(volatile uint32* ptr, int32 value);


#if(TURRET_AXIS_TEST == 1)
//占用刀塔轴和刀塔

int8 turretAxisAndTurretOccupyByChannel(VAL_STRUCT_TYPE* ucValAxis, uint8 ucChannelNum, VAL_STRUCT_TYPE* pOccupideAxis, VAL_STRUCT_TYPE* pOccupideTurretAxis, VAL_TURRET_TYPE* pOccupideTurret);
//先查询刀塔是否空闲  目标所有刀塔都空闲时占用目标刀塔   不空闲时占用失败 返回FASLE
bool turretManagerOccupy(VAL_TURRET_TYPE ucSign);

//释放刀塔
bool turretManagerRelease(VAL_TURRET_TYPE ucSign);

//释放本通道所有占用的轴
int8 releaseAllOccupiedAxis(struct SInterpolationPre* pSPreProcess);

//释放本通道所有占用的刀塔
int8 releaseAllOccupiedTurret(struct SInterpolationPre* pSPreProcess);
#endif



#endif


/*
1.常用命名
（1）名字第二个单词开始首字母都为大写，非“XXX_”的前缀也算第一个单词。const变量、宏定义和enum除外。如“dDeta”、“m_ dDeta”。
（2）类类型名：以“C”开头。如“CMotionController”。
（3）结构体类型名：以“S”开头。如“SMotionController”。
（4）类成员变量：以“m_”开头。示例：m_nValue。
（5）全局变量：以“g_”开头。示例：g_nValue。
（6）静态变量：以“s_”开头。示例：s_nValue。
（7）const变量：全部字母都大写，单词间用“_”隔开。如“TASKID_LIBRARY”。
（8）宏定义：全部字母都大写，单词间用“_”隔开。如“TASKID_LIBRARY”。
（9）enum定义：全部字母都大写，单词间用“_”隔开。如“TASKID_LIBRARY”。
（10）指针变量：以“p”开头。如“pAhead”。
（11）函数名：第一个单词首字母为小写。（与Qt等库的函数命名一致，构造函数和析构函数除外）。
（12）简单的循环变量常用i,j,k,p,it(迭代器)。遇到嵌套循环时，循环变量应该取一个好名字，如xxxLoop、xxxCounter。

2.一般常用命名
类型	变量开头	举例
bool	b	bIsParent
short	s	sStepCount
unsigned short	us	usTemp
char	c	cCount
unsigned char	uc	ucCount
wchar_t	wc	wcSum
float	f	fAvg
double	d	dDeta
long double	ld	ldDeta
int	n	nCount
unsignedint	un	unCount
long	l	lSum
unsigned long	ul	ulBroad
*/
