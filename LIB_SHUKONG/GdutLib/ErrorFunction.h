#ifndef _ERRORFUCTION_H
#define _ERRORFUCTION_H

void printError(void);

#include "API.h"

#if STM32_MODE
#define bool bool
#endif

#pragma warning(disable : 4996) //处理VS环境下读文件的警告
#define ErrorGCode_MAX 5        //允许保存错误G代码的最多字符数
#define GCODEERRORBUF_MAX 10    //SGCodeErrorBuf G代码错误信息缓冲区 容量
#define SIZE_OF_ERROR_STRING 38 //错误字符串大小

#define GCODE_ERROR_NUM01 1 //急停报警!急停按钮已按下或未接急停按钮!

#define GCODE_ERROR_NUM29 29 // M35 功能等待时间超时
#define GCODE_ERROR_NUM30 30 // M35/M20/M01 功能端口错误		//临时用，可被修改
#define GCODE_ERROR_NUM31 31 // M01 端口检测超时		//临时用，可被修改

#define GCODE_ERROR_NUM41 41 //X轴超出正向软件限位! //朱稳中 2023.6.16
#define GCODE_ERROR_NUM42 42 //Y轴超出正向软件限位!
#define GCODE_ERROR_NUM43 43 //Z轴超出正向软件限位!
#define GCODE_ERROR_NUM44 44 //A轴超出正向软件限位!
#define GCODE_ERROR_NUM45 45 //B轴超出正向软件限位!
#define GCODE_ERROR_NUM46 46 //C轴超出正向软件限位!

#define GCODE_ERROR_NUM47 47 //X轴超出负向软件限位!
#define GCODE_ERROR_NUM48 48 //Y轴超出负向软件限位!
#define GCODE_ERROR_NUM49 49 //Z轴超出负向软件限位!
#define GCODE_ERROR_NUM50 50 //A轴超出负向软件限位!
#define GCODE_ERROR_NUM51 51 //B轴超出负向软件限位!
#define GCODE_ERROR_NUM52 52 //C轴超出负向软件限位!

#define GCODE_ERROR_NUM80 80 //G代码错误代码80:段号必须在程序段开头!
#define GCODE_ERROR_NUM81 81 //G代码错误代码81:代码地址输入错误!
#define GCODE_ERROR_NUM82 82 //G代码错误代码82:段号不能有负值或小数点!
#define GCODE_ERROR_NUM83 83 //G代码错误代码83:G-代码不能有负数!
#define GCODE_ERROR_NUM84 84 //G代码错误代码84:程序输入的代码值非法!
#define GCODE_ERROR_NUM85 85 //G代码错误代码85:代码字的字符数不能小于2或大于11!
//#define GCODE_ERROR_NUM86 86    //G代码错误代码86:代码之间缺少空格符! //弃用
#define GCODE_ERROR_NUM87 87 //G代码错误代码87:输入了不支持的G代码!
#define GCODE_ERROR_NUM88                                                                                              \
    88 //G代码错误代码87:程序段第一行或最后一行缺少标识符%号，%必须是一行的第一个字符     注：错误过多时，unEndLineNum不会被赋值，也会报此错误
#define GCODE_ERROR_NUM89 89 //G代码错误代码89:要读取的程序不存在或打开失败!

#define GCODE_ERROR_NUM91 91   ///G代码错误代码91:读取程序段内容时出错!
#define GCODE_ERROR_NUM92 92   ///G代码错误代码92:手动输入方式下不能执行M98, M99指令!
#define GCODE_ERROR_NUM93 93   //G代码错误代码93:搜索不到指定的程序段号!
#define GCODE_ERROR_NUM94 94   //G代码错误代码94:宏程序的嵌套层数超过最大限制!
#define GCODE_ERROR_NUM95 95   ///G代码错误代码95:G75指令代码格式不正确!
#define GCODE_ERROR_NUM96 96   ///G代码错误代码96:G74指令代码格式不正确!
#define GCODE_ERROR_NUM97 97   //G代码错误代码97:G70～G73代码中精加工程序段超出限制!
#define GCODE_ERROR_NUM98 98   //G代码错误代码98:圆弧参数R,I,K至少输入一个!
#define GCODE_ERROR_NUM99 99   //G代码错误代码99:圆弧插补中弦长大于2倍半径R!
#define GCODE_ERROR_NUM100 100 //G代码错误代码100:G73代码中ns程序段只能是G00,G01代码!
#define GCODE_ERROR_NUM101 101 //G代码错误代码101:ns～nf程序段必需在G71~G73后一段输入!
#define GCODE_ERROR_NUM102 102 //G代码错误代码102:G73指令代码格式不正确!
#define GCODE_ERROR_NUM103 103 //G代码错误代码103:G71～G73代码未输入P值!
#define GCODE_ERROR_NUM104 104 //G代码错误代码104:G71～G73代码未输入Q值!
#define GCODE_ERROR_NUM105 105 //G代码错误代码105: G72代码第一段未输入Z或Z轴移动量为0!
#define GCODE_ERROR_NUM106 106 //G代码错误代码106: G72代码第一段只能是Z轴快速移动或切削进给!
#define GCODE_ERROR_NUM107 107 //G代码错误代码107: G72指令代码格式不正确!
#define GCODE_ERROR_NUM108 108 //G代码错误代码108: G71代码第一段未输入X或X轴移动量为0!
#define GCODE_ERROR_NUM109 109 //G代码错误代码109: G71代码第一段只能是X轴快速移动或切削进给!
#define GCODE_ERROR_NUM110 110 //G代码错误代码110: G71指令代码格式不正确!
#define GCODE_ERROR_NUM111 111 //G代码错误代码111:G76代码格式不正确!
#define GCODE_ERROR_NUM112 112 //G代码错误代码112:G76中没有指定螺纹牙高P值或为0!
#define GCODE_ERROR_NUM113 113 //G代码错误代码113:G76中没有指定第一次切削深度Q或为0!
#define GCODE_ERROR_NUM114 114 //G代码错误代码114:G76中P值格式不正确或超出范围!
#define GCODE_ERROR_NUM115 115 //G代码错误代码115:手动输入MDI模式下不能使用G70～G73、宏指令、通道指令!
///#define GCODE_ERROR_NUM116 116  ///G代码错误代码116:执行G70代码前请先执行G71～G73代码!  弃用
#define GCODE_ERROR_NUM117 117 //G代码错误代码117:C刀补中不能执行T指令，请撤销C刀补!
#define GCODE_ERROR_NUM118 118 //G代码错误代码118:C刀补中没有移动的代码段超过规定的最大段数!

#define GCODE_ERROR_NUM126 126 ///G代码错误代码126:当前不是模拟主轴,无法使用G96恒线速功能!
#define GCODE_ERROR_NUM127 127 //G代码错误代码127:一行出现了同组模态代码!
#define GCODE_ERROR_NUM129 129 //G代码错误代码129:输入了无效的刀具号!
#define GCODE_ERROR_NUM130 130 //G代码错误代码130:程序指令代码格式错误!

#define GCODE_ERROR_NUM131 131 //函数运算输入的参数无效!
#define GCODE_ERROR_NUM132 132 //除法运算除数不能为0!
#define GCODE_ERROR_NUM133 133 //平方根输入数据不能小于0!
#define GCODE_ERROR_NUM134 134 //宏语句运算格式错误!
#define GCODE_ERROR_NUM135 135 //WHILE-END循环体语句格式错误!
#define GCODE_ERROR_NUM136 136 //IF语句格式错误!

#define GCODE_ERROR_NUM147 147 //G83或G87指令没有指令进刀量Q或Q=0
#define GCODE_ERROR_NUM148 148 //主轴被刹车锁紧，不能旋转主轴		CMC 2023/4/4
#define GCODE_ERROR_NUM149 149 //夹具未夹主轴不可运转		洪冠鹏 2023/12/19
#define GCODE_ERROR_NUM150 150 //未映射物理轴		洪冠鹏 2024/6/11
#define GCODE_ERROR_NUM151 151 //编码器相位错误		熊骏 2025/3/29
#define GCODE_ERROR_NUM152 152 //速度发生突变		熊骏 2025/3/29
#define GCODE_ERROR_NUM153 153 //参数输入/系统参数错误，出现了零值		熊骏 2025/9/3
/* 苏向标 2026-10-14：停止异常单独报警，不伪装为复位完成。 */
#define GCODE_ERROR_NUM155 155 /* 主轴停止状态或控制权异常，查看cStopError。 */
#define GCODE_ERROR_NUM154 154 //超时/超速错误

#define GCODE_ERROR_NUM160 160 //该指令未增加检查函数
#define GCODE_ERROR_NUM161 161 //该指令未增加参数赋值函数
#define GCODE_ERROR_NUM162 162 //该指令未增加轨迹计算函数

#define GCODE_ERROR_NUM280 280 //D/L参数同时出现
#define GCODE_ERROR_NUM281 281 //倒角长度过长
#define GCODE_ERROR_NUM282 282 //倒圆半径过大
#define GCODE_ERROR_NUM283 283 //倒角/倒圆前提条件不满足或几何数据有误
#define GCODE_ERROR_NUM284 284 //自动倒角缓冲区不足

#define GCODE_ERROR_NUM300 300 //G代码错误代码300:传入指针变量为NULL!
#define GCODE_ERROR_NUM301 301 //输入了无效的函数参数!
#define GCODE_ERROR_NUM302 302 //工循环功能传入指针变量为NULL!
#define GCODE_ERROR_NUM303 303 //链表功能传入指针变量为NULL!
#define GCODE_ERROR_NUM304 304 //不存在此分支!
#define GCODE_ERROR_NUM305 305 //G66嵌套层数过多!
#define GCODE_ERROR_NUM306 306 //译码状态机出错!
#define GCODE_ERROR_NUM307 307 //尝试给不为NULL的地址开辟内存!
#define GCODE_ERROR_NUM308 308 //在批注中遇到文件结束符
#define GCODE_ERROR_NUM309 309 //

//G代码的语法错误
#define GCODE_ERROR_NUM399 399 //G代码错误代码399:一行中出现M功能数目不能超过6个!
#define GCODE_ERROR_NUM400 400 //G代码错误代码400:不能出现两个正负号!
#define GCODE_ERROR_NUM401 401 //G代码错误代码401:不能出现两个小数点!
#define GCODE_ERROR_NUM402 402 //G代码错误代码402:一行中重复出现该字符!
#define GCODE_ERROR_NUM403                                                                                             \
    403 //G代码错误代码403:当前行模态功能不支持输入该字符功能或无模态指令!  注意：发生无模态指令错误可能会导致后续代码也出错，请先解决此类错误
#define GCODE_ERROR_NUM404 404 //G代码错误代码404:圆弧参数I,K输入有误!(并不能指出哪个参数有误)
#define GCODE_ERROR_NUM405 405 //G代码错误代码405:执行该模态指令缺少运行参数(主轴转向、转速、进给速度)!
#define GCODE_ERROR_NUM406 406 //G代码错误代码406:内存空间不足，内存分配失败!
//#define GCODE_ERROR_NUM407 407      ///G代码错误代码407:C刀补尚未建立，不能执行撤销刀补!  待处理  预弃用
#define GCODE_ERROR_NUM408 408 //G代码错误代码408:更换刀补类型前，请先撤销刀补!
#define GCODE_ERROR_NUM409 409 //G代码错误代码409:只能以直线进行刀补建立或撤销
#define GCODE_ERROR_NUM410 410 //G代码错误代码410:G32或G92代码参数不正确!
#define GCODE_ERROR_NUM411 411 //G代码错误代码411:G90或G92参数R错误!
#define GCODE_ERROR_NUM412 412 //G代码错误代码412:螺纹导程必须大于0!
#define GCODE_ERROR_NUM413 413 //G代码错误代码413:螺纹退尾量不正确！
#define GCODE_ERROR_NUM414 414 //G代码错误代码414:G92不能加工端面螺纹，即不能只有X轴坐标发生变化，而Z轴坐标不动!
#define GCODE_ERROR_NUM415 415 //G代码错误代码415:G代码过长，超过256个字符!
#define GCODE_ERROR_NUM416 416 //G代码错误代码416:G90/G94格式错误!
#define GCODE_ERROR_NUM417 417 //G代码错误代码417:G71起刀点A不合理!(若单调性有问题可能会导致误判)
#define GCODE_ERROR_NUM418 418 //G代码错误代码418:G71在X方向的单调性变化不正确!(如果起刀点位置不合理也会导致此报错)
#define GCODE_ERROR_NUM419 419 //G代码错误代码419:G70P值或Q值不正确或没输入!

#define GCODE_ERROR_NUM421 421 //G代码错误代码421:G72在Z方向的单调性变化不正确!(如果起刀点位置不合理也会导致此报错)
#define GCODE_ERROR_NUM422 422 //G代码错误代码422:G72起刀点A不合理!(若单调性有问题可能会导致误判)
//#define GCODE_ERROR_NUM423 423      //G代码错误代码423:II型G71必须指定X(U)和Z(W)两个轴，当Z轴不移动时也必须指定W0!
//#define GCODE_ERROR_NUM424 424      //G代码错误代码424:II型G72必须指定X(U)和Z(W)两个轴，当X轴不移动时也必须指定U0!

#define GCODE_ERROR_NUM425 425 //G74指令中未输入Z的值!
#define GCODE_ERROR_NUM426 426 //G74指令中Q的值为0或未输入
#define GCODE_ERROR_NUM427 427 //G74或G75指令中未输入X的值
#define GCODE_ERROR_NUM428 428 //G75指令中P的值为0或未输入
#define GCODE_ERROR_NUM429 429 //G74或G75中的单次退刀量 R(e)超出允许范围
#define GCODE_ERROR_NUM430 430 //G74或G75中X或Z方向的单次切削量超出允许范围
#define GCODE_ERROR_NUM431 431 //G74或G75中的单次退刀量R(Δd)超出允许范围

#define GCODE_ERROR_NUM432 432 //G70格式不正确
#define GCODE_ERROR_NUM433 433 //G71、G72、G73使用段号重复!
#define GCODE_ERROR_NUM434 434 //G33/G32系统最大速度无法满足！

#define GCODE_ERROR_NUM435 435 //G代码错误代码435:G71II型计算错误！
#define GCODE_ERROR_NUM436 436 //G代码错误代码436:G71II型数据存储失败！
#define GCODE_ERROR_NUM437 437 //G71二型NF段不允许出现X值变化Z值不变时的起始边！

#define GCODE_ERROR_NUM438 438 //圆弧G32弧度不符合要求!
#define GCODE_ERROR_NUM439 439 //调用程序出错!
#define GCODE_ERROR_NUM440 440 //宏程序嵌套层数超出限制!
#define GCODE_ERROR_NUM441 441 //M97指令代码格式不正确!
#define GCODE_ERROR_NUM442 442 //等待指令错误！

#define GCODE_ERROR_NUM443 443 //自定义宏语法错误
#define GCODE_ERROR_NUM444 444 //IF语句语法错误
#define GCODE_ERROR_NUM445 445 //WHILE语句语法错误
#define GCODE_ERROR_NUM446 446 //GOTO语句输入段号无效
#define GCODE_ERROR_NUM447 447 //宏指令语法错误

#define GCODE_ERROR_NUM448 448 //输入的程序号错误!
#define GCODE_ERROR_NUM449 449 //输入的调用次数错误!
#define GCODE_ERROR_NUM450 450 //不存在此A类宏指令
#define GCODE_ERROR_NUM451 451 //缺少执行此A类宏指令所需的参数
#define GCODE_ERROR_NUM452 452 //根号内的值不能为负数

#define GCODE_ERROR_NUM453 453 //G34的R值不在范围内
#define GCODE_ERROR_NUM454 454 //螺纹车削的长轴最大速度过小
#define GCODE_ERROR_NUM455 455 //螺纹车削的短轴最大速度过小
#define GCODE_ERROR_NUM456 456 //螺纹车削的进给速率大于1000
#define GCODE_ERROR_NUM457 457 //子程序中没有M99主程序返回
#define GCODE_ERROR_NUM458 458 //另一通道正执行刀补，本通道不许执行其刀补平面运动

#define GCODE_ERROR_NUM459 459 //FASH任务繁忙
#define GCODE_ERROR_NUM460 460 //申请IO失败
#define GCODE_ERROR_NUM461 461 //G90外圆车削循环只能在G18(ZX)平面执行

#define PLC_ERROR_NUM1000 1000 //电动刀架换刀超时	//目前只用了1000	//陈敏聪	来自CNC说明书	2023/2/4
#define PLC_ERROR_NUM1001 1001 //换刀结束时，刀架未到位报警
#define PLC_ERROR_NUM1002 1002 //换刀未完成报警
#define PLC_ERROR_NUM1003 1003 //未检测到刀架锁紧TCP信号
#define PLC_ERROR_NUM1004 1004 //换刀方式A或B最多只支持8把刀
#define PLC_ERROR_NUM1005 1005 //系统断电前换刀出错

#define PLC_ERROR_NUM3842 2042 //上电后读取刀位信号异常报警
#define PLC_ERROR_NUM3955                                                                                              \
    2155 //换刀后的刀号与指定刀号不符	//陈敏聪	自定义	来自苏总提供的KT820系统于常州优思塔伺服刀架的说明书	2023/2/4
#define PLC_ERROR_NUM3978 2178 //伺服驱动器报警
#define PLC_ERROR_NUM3986 2186 //刀塔定位完成信号未检测到
#define PLC_ERROR_NUM3987 2187 //未检测到刀架锁紧到位或松开到位信号
#define PLC_ERROR_NUM3988 2188 //刀具状态未定义

typedef struct SGCodeError //G代码错误信息结构体
{
    uint32 unLineNum; //G代码行号

    uint16 usErrorCode; //G代码错误代码

    uint8 ucErrorGCode[ErrorGCode_MAX]; //错误G代码字符

    uint8 ucIOIndex; //IO索引端口 2023/10/24
} SGCodeError;

//报错函数1
uint8 parseGCodeError(const char* pGCode, uint32 unLineNum, uint8 ucWord_Len);

//报错函数1
uint8 parseGCodeErrorB(uint32 unLineNum, uint16 usErrorNum);

//报错函数2
uint8 parseErrorC(uint32 unLineNum, uint16 usErrorNum);

//中断运行中报错
uint8 parseRunningError(uint32 unLineNum, uint16 usErrorNum, uint32 ucIOIndex);

//返回错误号相对的内容
int16 getStrByWord(uint16 sNum, uint16 c_num, uint16 mode);
int8* getStrByNum(uint16 sNum, uint16 c_num, uint16 mode);

//返回标志位，判断错误缓冲是否满
bool isErrorFull();

//初始化错误缓冲区及相关变量
void initErrorBuf();

//设置错误类型
void setErrorType(uint16 usType);

//判断是否已有错误写入
bool isErrorHasBeenWritten();

//设置错误已写入标志位
void setErrorWriteFlag(bool bFlag);

//获取错误数量
uint8 getErrorNum();

//设置错误标志位
void setErrorFlag(bool bFlag);

//获取错误标志位
bool getErrorFlag();

//返回错误结构体的地址
struct SGCodeError getErrorStructAdd(int8 cIndex);

//struct SGCodeError
//{
//	uint32 unLineNum;  //G代码行号
//
//	uint16 usErrorCode; //G代码错误代码
//
//	uint8 *pErrorGCode; //错误G代码字符，注意要动态分配内存。
//};

#endif // !_ERRORFUCTION_H
