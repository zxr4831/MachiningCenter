#ifndef __GCODECOMMON_H
#define __GCODECOMMON_H

/************************* 临时宏定义 *************************/
#define TEMP_DOUBLE_OR_NOT 0   //临时宏定义，直径编程为0 半径编程为1
#define TEMP_ACTIVATIVE_CHANNEL_ONE 0   //临时宏定义  主动启动通道1
#define TEMP_LEAST_INPUT_INCREMENT  0.001    //最小输入增量 0.001mm
#define TEMP_PARA_NO180  0 //与宏的三角函数有关
/************************* G临时宏定义 end *************************/



/************************* 测试相关宏定义 *************************/
#define MEM_TEST 1
#define ENABLE_ASSIGN 0         //0：关闭赋值 1:开启赋值    赋值指给G代码缓冲区赋值
#define EN_GCODE_DEBUG 0        //检查流程的打印                   0:关闭DEBUG  1:开启DEBUG printf输出
#define ENABLE_DEBUG_ASSIGN 0   //赋值流程的打印
#define ENABLE_PERSONAL_TEST 0 //个人使用
#define XHJ_DEBUG 0        //G71II型打印
/*********************** G测试相关宏定义 end ***********************/



/************************* G代码相关名称宏定义 *************************/
#define GC_SYNTAX 0   //语法结构
#define GC_ASSIGN 1   //赋值结构
#define GC_CHANNEL 2  //通道结构

#define GC_G65_CALL  1    //G65是调用宏程序
#define GC_G65_MACRO_A 2   //G65是A类宏
#define GC_G66_MODLE  3         //G66模态
#define GC_G67  0         //G67
#define ANGLE_TO_RAD(angel)  ((angel) * 0.0174532925)   //角度转弧度,角度*（PI/180）rad
#define RAD_TO_ANGEL(rad)  ((rad) * 57.2957795130)   //弧度转度数,弧度*（180/PI）°

#define GCOMMAND_DECIMAL_PLACE 1 //G命令支持的小数位数
#define MCOMMAND_DECIMAL_PLACE 0 //M命令支持的小数位数

/*********************** G代码相关名称宏定义 end ***********************/



/************************* 流程分支宏定义 *************************/

///SYNTAX_PART_2和ASSIGN_PART_2的值都要为2!



#define PART_1 1
#define PART_2 2
#define PART_3 3
#define PART_4 4
#define PART_5 5
#define PART_6 6
#define PART_7 7

#define SYNTAX_PART_1 1
#define SYNTAX_PART_2 2
#define SYNTAX_PART_3 3
#define SYNTAX_PART_4 4
#define SYNTAX_PART_5 5
#define SYNTAX_PART_6 6
#define SYNTAX_PART_7 7   //语法检查的最后一part
#define SYNTAX_PART_8 8
#define SYNTAX_GROUP_CMD_PART 127  //语法检查cMd组分支

#define ASSIGN_PART_1 1
#define ASSIGN_PART_2 2
#define ASSIGN_PART_3 3
#define ASSIGN_PART_4 4
#define ASSIGN_PART_5 5
#define ASSIGN_PART_6 6
#define ASSIGN_PART_7 7
#define ASSIGN_PART_8 8
#define ASSIGN_SMD_PART 126

#define MACRO_DEFINE_PART_1 125
#define MACRO_DEFINE_PART_2 124
#define MACRO_DEFINE_PART_3 123

#define MACRO_IF_WHILE_PART_1 122    //宏指令(扫描但不执行)分支
#define MACRO_IF_WHILE_PART_2 121
#define MACRO_IF_WHILE_PART_3 120
#define MACRO_IF_WHILE_PART_4 119   //ELSEIF判断表达式值专用
#define MACRO_IF_WHILE_PART_5 118

#define MACRO_GOTO_PART_0 112
#define MACRO_GOTO_PART_1 117
#define MACRO_GOTO_PART_2 116
#define MACRO_GOTO_PART_3 115
#define MACRO_GOTO_PART_4 114


#define FIND_TARGERT_LETTER_N_PART 113

#define G65_TRANSMIT_DATA_PART_1 59
#define G65_TRANSMIT_DATA_PART_2 60
#define G65_TRANSMIT_DATA_PART_3 61

#define G66_TRANSMIT_DATA_PART_1 62
#define G66_TRANSMIT_DATA_PART_2 63
#define G66_TRANSMIT_DATA_PART_3 64



#define G65_CALL_PART 65
#define G66_CALL_PART 66


/******************** 流程分支宏定义 end ***********************/



/************************* 各种次数、数量限制的宏定义 ***********************/
#define MAX_LENGTH_OF_LINE  200         //字符串字节数长度限制
#define MAX_LENGTH_OF_LEXICAL_UNIT 11   //词法单元最大长度
#define MIN_LENGTH_OF_LEXICAL_UNIT 2    //词法单元最小长度
#define UNASSIGNED_VALUE -1             //未赋值

#define TEMP_FILENAME_MAX_LENGTH  16  //文件名最大长度
#define FILENAME_MAX_LENGTH 5
#define NESTING_LIMIT  5   //宏程序个数限制
#define MAXIMUM_OF_LOCAL_VARIABLE 33  //宏局部变量的最大数量
#define SCAN_LOOP_LIMIT_NORMAL 10     //普通模式扫描调用次数限制
#define SCAN_LOOP_LIMIT_MACRO 5      //宏模式扫描调用次数限制
#define FIND_DEFINE_MACRO_LIMIT  5   //找宏名称时遍历表格的次数

#define G50_INFO_BUF_MAX 10         //g50信息结构体环形缓冲区的大小 2023.7.28 陈育楠
/********************* 次数、数量限制宏定义 end ***********************/



/************************ 宏B 宏定义  ***************************/
#define MACRO_IF 0        //宏B IF
#define MACRO_WHILE 1     //宏B WHILE
#define MACRO_OK 0     //满足执行
#define MACRO_NOT_OK 1 //不满足执行
#define MACRO_FORBID 2 //禁止执行

#define GC_MODE_NORMAL 0//普通模式
#define GC_MODE_NORMAL_2_MACRO_1 1 //获取自定义宏阶段1
#define GC_MODE_NORMAL_2_MACRO_2 2 //获取自定义宏阶段2
#define GC_MODE_MACRO 3 //宏模式
/************************ 宏B 宏定义 end ************************/


/************************* G40G41G42相关宏定义 *************************/
#define MAX_COUNTER_NO_MOVE  15  //刀补指令中无移动代码最大行数
/************************* G40G41G42宏定义 end*************************/


/************************* G71相关宏定义 *************************/
#define CYCLE_LINES_LIMIT   40      //复合循环的ns~nf行数限制
#define UP     1                    //UP和DOWN用于单调性的判断
#define DOWN  -1
#define START_EDGE   1
#define END_EDGE -1
#define FORWARD  1    //正向
#define BACKWARD  -1  //反向
#define CHECK_NUM_LIMIT   30 //初次通道1语法检查的行数限制
#define MODIFY_LIMIT_G71G72  1   //限制G71-G73每次修改链表的数据数
#define SUCCESSIVE_G70_LIMIT   2    ///连续粗加工次数限制  弃用
#define G71II_LOOP_LIMIT         1000 //限制G71II计算循环上限，防止出现错误时死循环
#define DIR_X   1   //X方向
#define DIR_Z   -1  //Z方向  
/************************* G71相关宏定义 end*************************/


/************************* G代码快速处理相关宏定义 *************************/
//用于识别该行G代码类型
#define GCODE_CANT_QUICKHANDLE				0	//G代码不能快速处理，指当前行是不可快速处理的NC代码
#define GCODE_QUICKHANDLE					1	//G代码能快速处理，指当前行是可快速处理的M代码，IO指令
#define GCODE_FULL_MERGED					2	//G代码能完全合并，指全是G04 X0类型

//用于函数返回值
#define GCODE_CAN_QUICKHANDLE				1	//G代码能快速处理
#define GCODE_CAN_MERGED					2	//G代码能合并
/************************* G代码快速处理相关宏定义 END*************************/

/************************* G代码用户相关相关宏定义 *************************/
//画图
#define COMMON_USERID		0		//正常G代码调用
#define GCODEDRAW_USERID	5		//绘图G代码调用
/************************* G代码用户相关相关宏定义 END*************************/


#if STM32_MODE
#else
#pragma warning(disable:4996)   //处理VS环境下读文件的警告
#endif

enum GroupG
{
	/// G代码分类参数，
	/// cGa：运动类 G00，G01，G02，G03，G32,G33,G76,G92
	/// cGb：工件坐标系 G54-G59
	/// cGc：定义插补平面 G17-G19
	/// cGd：刀具补偿 G40-G42
	/// cGe：单位选择：英制/公制 G20，G21
	/// cGf: 恒线速控制 G96 、恒转速控制 G97
	/// cGg: 给定每分进给速度 G98 、以毫米/转为单位给定切削进给速度 G99
	/// ucGz：非模态组别G04(程序暂停).....
	/// ucGx: 非模态组别，用于G83~G89的模态取消
	G00, G01, G02, G03,
	G04,
	G07 = 7,
	G10 = 10,
	G15 = 15, G16 = 16,
	G17 = 17, G18 = 18, G19 = 19,
	G20 = 20, G21 = 21,
	G06 = 06,
	G32 = 32,
	G33 = 33,
	G34 = 34,
	G40 = 40, G41 = 41, G42 = 42,
	G43 = 43,
	G50 = 50,
	G54 = 54, G55 = 55, G56 = 56, G57 = 57, G58 = 58, G59 = 59,
	G65 = 65, G66 = 66, G67 = 67,
	G70 = 70,
	G71 = 71,
	G72 = 72,
	G73 = 73,
	G74 = 74,
	G75 = 75,
	G76 = 76,
	G78 = 78,
	G80 = 80,
	G83 = 83,
	G84 = 84,
	G85 = 85,
	G87 = 87,
	G88 = 88,
	G90 = 90,
	G91 = 91,
	G92 = 92,
	G93 = 93,
	G94 = 94,
	G96 = 96, G97 = 97,
	G98 = 98, G99 = 99
};

enum GroupM
{
	M00, M01, M02,
	M03 = 3, M04 = 4, M05 = 5, M06 = 6,
	M08 = 8, M09, M10, M11, M12, M13,
	M14 = 14, M15 = 15, 
	M20 = 20, M21 = 21, M22 = 22,
	//M23=23, M24=24, M25=25,

	M30 = 30,
	M32 = 32,
	M33 = 33,
	M35 = 35, M36 = 36,
	M38 = 38, M39 = 39,
	M50 = 50, M51 = 51, M52 = 52,
	M71 = 71,
	M72 = 72,

	M80 = 80, M81 = 81, M82 = 82, M83 = 83, M84 = 84, M85 = 85, M86 = 86, M87 = 87,

	M96 = 96,
	M97 = 97,
	M98 = 98,
	M99 = 99,
	M100 = 100,
	M199 = 199,
};

//G代码通道状态
enum GCodeStatus
{
	GCODE_IDLE,     //未启动
	GCODE_RUN,      //运行
	GCODE_WAIT,     //等待信号的等待
	GCODE_FINISHED, //完成
	GCODE_WARN,      //报警
	GCODE_RUN2FINISHED,  //运行到完成过渡状态

	//GCODE_WAIT_M96,     //M96专用
	GCODE_SYNTAX_WAIT,  //检查等待赋值   //因为检查超前，有时候要切换程序之类的，等检查和赋值在同一行时再切换程序
	GCODE_ASSIGN_WAIT   //赋值等待检查   //检查需要超前赋值一定的行数赋值才能启动  
};


enum ModalNum  //模态号码 陈敏聪
{
	sGa = 1,
	sGb,
	sGc,
	sGd,
	sGe,
	sGf,
	sGg,
	sGh,
	sGi,
	sGj,
	sGx,
	sGz
};

enum NonModalNum  //模态号码 陈敏聪
{
	cMa = 1,
	cMb,
	cMc,

	cMe,
	cMf,
	cMg,
	cMh,
	cMi,
	cMj,
	cMk
};



enum G50RingBufPointerType//G50环形数组指针类型
{
	SG50_WRITR,
	SG50_READ
};


typedef struct _SCoord
{
	bool bXYZ[3];
	bool bUVW[3];
	bool bABC[3];

	double dXYZ[3];
	double dUVW[3];
	double dABC[3];
}SCoord;



#endif // !__GCODECOMMON_H
