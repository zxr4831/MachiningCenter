#ifndef CORNER_TRANSITION_H
#define CORNER_TRANSITION_H

#include "GCodeCheckAndAssign.h"

/* 均接收已转换模态的记录（G1/G2/G3 为 1/2/3），不操作全局缓冲。 */

/*************************************************
  函数名称：isCornerRequestValid
  函数功能：校验单段 D/L 数值、运动范围及结束标志。
  输入参数：pSegment 已转换的段，允许为空。
  输出参数：无。
  返 回 值：TRUE 合法；FALSE 非法；零请求不单独判错。
  备    注：供写入阶段预检和计算入口复用，不代替原始指令检查。
**************************************************/
bool isCornerRequestValid(const struct SGCodeParse *pSegment);

/*************************************************
  函数名称：isCornerPairCompatible
  函数功能：检查两段的运动、连接点、刀具及相关状态是否允许倒角。
  输入参数：a、b 为非空且已转换模态的前后段。
  输出参数：无。
  返 回 值：TRUE 兼容；FALSE 不兼容。
  备    注：写入方在容量检查前调用，以保留原有报警优先顺序。
**************************************************/
bool isCornerPairCompatible(const struct SGCodeParse *a, const struct SGCodeParse *b);

/*************************************************
  函数名称：calcCornerTransition
  函数功能：构造前段裁剪结果、过渡段、后段裁剪结果。
  输入参数：pPrevSegment、pNextSegment 已转换模态；前段须有正 D/L。
  输出参数：pTrimmedPrev、pTransition、pTrimmedNext 仅返回 1 时有效。
  返 回 值：1 成功；0 同向共线；-1 无解；-2 非法输入或不支持。
  备    注：非成功时输出不变；五个对象不得重叠，输出不得持有待释放资源。
            原段资源指针为借用，提交方负责转移所有权；本接口不分配或释放。
**************************************************/
int8 calcCornerTransition(const struct SGCodeParse *pPrevSegment,
    const struct SGCodeParse *pNextSegment, struct SGCodeParse *pTrimmedPrev,
    struct SGCodeParse *pTransition, struct SGCodeParse *pTrimmedNext);

#endif
