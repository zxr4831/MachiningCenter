/*
译码模块动态内存编号：ORAM_NUM300-ORAM_NUM499
刀补模块动态内存编号：
运动内存编号：
*/
#ifndef _MANAGEMEM_H
#define _MANAGEMEM_H

#define MEM_TEST 1


#if !STM32_MODE

#define ORAM_NUM300	300	//	G代码通道结构体
#define ORAM_NUM301	301	//	程序链表
#define ORAM_NUM302	302	//	程序链表节点(1-N)
#define ORAM_NUM303	303	//	程序数据结构体(1-N)
#define ORAM_NUM304	304	//	IF/WHILE链表_检查
#define ORAM_NUM305	305	//	IF/WHILE链表节点(1-N)_检查
#define ORAM_NUM306	306	//	IF/WHILE数据结构体(1-N)_检查
#define ORAM_NUM307	307	//	IF/WHILE链表_赋值
#define ORAM_NUM308	308	//	IF/WHILE链表节点(1-N)_赋值
#define ORAM_NUM309	309	//	IF/WHILE数据结构体(1-N)_赋值
#define ORAM_NUM310	310	//	后缀表达式链表_检查
#define ORAM_NUM311	311	//	后缀表达式链表节点(1-N)_检查
#define ORAM_NUM312	312	//	运算栈链表_检查
#define ORAM_NUM313	313	//	运算栈链表节点(1-N)_检查
#define ORAM_NUM314	314	//	后缀表达式链表_赋值
#define ORAM_NUM315	315	//	后缀表达式链表节点(1-N)_赋值
#define ORAM_NUM316	316	//	运算栈链表_赋值
#define ORAM_NUM317	317	//	运算栈链表节点(1-N)_赋值
#define ORAM_NUM318	318	//	计算单元数据结构体(1-N)
#define ORAM_NUM319	319	//	自定义宏链表
#define ORAM_NUM320	320	//	自定义宏链表节点(1-N)
#define ORAM_NUM321	321	//	自定义宏数据结构体(1-N)
#define ORAM_NUM322	322	//	自定义宏宏名(1-N)
#define ORAM_NUM323	323	//	自定义宏替换字符串(1-N)
#define ORAM_NUM324	324	//	G指令规则树
#define ORAM_NUM325	325	//	G指令规则树节点(1-N)
#define ORAM_NUM326	326	//	G指令规则数据(1-N)
#define ORAM_NUM327	327	//	M指令规则树
#define ORAM_NUM328	328	//	M指令规则树节点(1-N)
#define ORAM_NUM329	329	//	M指令规则数据(1-N)


#define ORAM_NUM340	340	//	刀补计算结构体
#define ORAM_NUM341	341	//	运动通道结构体
#define ORAM_NUM342	342	//	通道管理结构体



#define ORAM_NUM350	350	//	文本获取信息结构体
#define ORAM_NUM351	351	//	轨迹检查结构体
#define ORAM_NUM352	352	//	轨迹获取结构体
#define ORAM_NUM353	353	//	轨迹链表
#define ORAM_NUM354	354	//	轨迹链表节点(1-N)
#define ORAM_NUM355	355	//	直线轨迹数据结构体(1-N)
#define ORAM_NUM356	356	//	圆弧轨迹数据结构体(1-N)
#define ORAM_NUM357	357	//	槽链表
#define ORAM_NUM358	358	//	槽链表节点(1-N)
#define ORAM_NUM359	359	//	槽数据结构体(1-N)
#define ORAM_NUM360	360	//	槽轨迹链表(1-N)
#define ORAM_NUM361	361	//	槽轨迹链表节点(1-N)



#define ORAM_NUM400	400	//	G2参数结构体检查
#define ORAM_NUM401	401	//	G2参数结构体赋值
#define ORAM_NUM402	402	//	G3参数结构体检查
#define ORAM_NUM403	403	//	G3参数结构体赋值
#define ORAM_NUM404	404	//	G32参数结构体检查
#define ORAM_NUM405	405	//	G32参数结构体赋值
#define ORAM_NUM406	406	//	G32.2参数结构体检查
#define ORAM_NUM407	407	//	G32.2参数结构体赋值
#define ORAM_NUM408	408	//	G32.3参数结构体检查
#define ORAM_NUM409	409	//	G32.3参数结构体赋值
#define ORAM_NUM410	410	//	G33参数结构体检查
#define ORAM_NUM411	411	//	G33参数结构体赋值
#define ORAM_NUM412	412	//	G34参数结构体检查
#define ORAM_NUM413	413	//	G34参数结构体赋值
#define ORAM_NUM414	414	//	G50参数结构体检查
#define ORAM_NUM415	415	//	G50信息结构体赋值
#define ORAM_NUM416	416	//	G66参数结构体检查
#define ORAM_NUM417	417	//	G66信息结构体赋值
#define ORAM_NUM418	418	//	G70参数结构体检查
#define ORAM_NUM419	419	//	G70参数结构体赋值
#define ORAM_NUM420	420	//	G71参数结构体检查
#define ORAM_NUM421	421	//	G71参数结构体赋值
#define ORAM_NUM422	422	//	G73参数结构体检查
#define ORAM_NUM423	423	//	G73参数结构体赋值
#define ORAM_NUM424	424	//	G74参数结构体检查
#define ORAM_NUM425	425	//	G74参数结构体赋值
#define ORAM_NUM426	426	//	G75参数结构体检查
#define ORAM_NUM427	427	//	G75参数结构体赋值
#define ORAM_NUM428	428	//	G76参数结构体检查
#define ORAM_NUM429	429	//	G76参数结构体赋值
#define ORAM_NUM430	430	//	G83参数结构体检查
#define ORAM_NUM431	431	//	G83参数结构体赋值
#define ORAM_NUM432	432	//	G83.1参数结构体检查
#define ORAM_NUM433	433	//	G83.1参数结构体赋值
#define ORAM_NUM434	434	//	G83.2参数结构体检查
#define ORAM_NUM435	435	//	G83.2参数结构体赋值
#define ORAM_NUM436	436	//	G84参数结构体检查
#define ORAM_NUM437	437	//	G84参数结构体赋值
#define ORAM_NUM438	438	//	G87参数结构体检查
#define ORAM_NUM439	439	//	G87参数结构体赋值
#define ORAM_NUM440	440	//	G87.1参数结构体检查
#define ORAM_NUM441	441	//	G87.1参数结构体赋值
#define ORAM_NUM442	442	//	G87.2参数结构体检查
#define ORAM_NUM443	443	//	G87.2参数结构体赋值
#define ORAM_NUM444	444	//	G88参数结构体检查
#define ORAM_NUM445	445	//	G88参数结构体赋值
#define ORAM_NUM446	446	//	G92参数结构体检查
#define ORAM_NUM447	447	//	G92参数结构体赋值
#define ORAM_NUM448	448	//	G93参数结构体检查
#define ORAM_NUM449	449	//	G93参数结构体赋值
#define ORAM_NUM450	450	//	G94参数结构体检查
#define ORAM_NUM451	451	//	G94参数结构体赋值
#define	ORAM_NUM452	452	//	G28参数结构体检查
#define	ORAM_NUM453	453	//	G28参数结构体赋值



#endif


extern int MEM_CNT;


//根据不同环境，使用不同的内存分配函数
void* mallocGCode(uint64 ulSize, uint16 ORAM_NUM);
void freeGCode(void* pFreeTarget);





#endif // !_MANAGEMEM_H