/**
Copyright (C) 版权归属沿用原项目，本次注释整理不变更权属。
@file HandwheelSim.h
@author Codex（本次移植与注释整理）
@version 1.5
@date 2026-10-06
@brief 声明 G0 独立轴与 G1/G2/G3 合成路径的距离授权状态及控制接口。
*/
#ifndef HANDWHEEL_SIM_H
#define HANDWHEEL_SIM_H

#include "common.h"

#if HANDWHEEL_SIM_DISTANCE_ENABLE
struct SChannel;

/** 待走距离的数值容差；不得用项目中按脉冲量级判断的 isZero() 代替。 */
#define HWSIM_DISTANCE_EPS 1.0e-12

/** 距离规划器的单拍执行结果；等待输入与程序段完成分别表示。 */
enum HANDWHEEL_SIM_STEP
{
    HWSIM_STEP_ERROR = -1,       /**< 状态或参数错误，不能输出本拍候选位置。 */
    HWSIM_STEP_WAIT = 0,         /**< 已停稳且无待走距离，等待输入，不结束当前段。 */
    HWSIM_STEP_MOVE = 1,         /**< 本拍产生有效运动候选，需要输出成功后提交。 */
    HWSIM_STEP_FORWARD_END = 2,  /**< 本轴已在段尾停稳；整条 G0 是否完成由全轴汇总判断。 */
    HWSIM_STEP_BACKWARD_END = 3, /**< 本轴已在段首停稳；由全轴汇总决定是否倒读历史段。 */
    HWSIM_STEP_STOPPED = 4,      /**< 已响应暂停请求停稳，提交时取消剩余授权。 */
    HWSIM_STEP_INPUT_IDLE = 5    /**< 停摇制动完成；取消余量但保留段绑定，不回写暂停。 */
};

/**
 * G0 每个参与轴独立持有本状态；G1/G2/G3 所有轴共用一个公共进度状态。
 * dSegmentPosition 从原几何起点计量，范围为 [0, dSegmentLength]；
 * dSegmentPosition + dPendingDistance 表示本轴授权目标；生产采样仅在到速限额后限制缓冲并可取消请求。
 * 文档将这个推导结果命名为 dTargetPosition，结构体不重复保存目标状态。
 * 边界速度是周期末速度，不是界面显示的本拍平均速度。
 * 机床上限、加减速度和周期在零速绑定段时固定；G99 的有效速限随主轴变化，减速仍逐拍执行。
 * G0 单位：直线轴为 mm，旋转轴为度；合成路径采用自动公共进度单位；时间为 ms。
 */
struct SHandwheelSim
{
    double dPendingDistance;  /**< 相对当前段进度尚未执行的有符号授权距离，单位为路径单位。 */
    double dSegmentPosition;  /**< 输出链接受的理论段进度，不代表伺服反馈位置。 */
    double dSegmentLength;    /**< 当前段固定几何长度，不随手轮授权增减。 */
    double dBoundaryVelocity; /**< 有符号周期末速度，单位为路径单位/ms。 */
    double dPeriod;           /**< 插补周期，单位为 ms。 */
    double dSpeedLimit;       /**< 标量速度上限，单位为路径单位/ms；G99 停转等待时可为零。 */
    double dAcceleration;     /**< 正值加速度上限，单位为路径单位/ms^2。 */
    double dDeceleration;     /**< 正值减速度上限，单位为路径单位/ms^2。 */
    uint8 ucAxis;             /**< 当前绑定的 G0 逻辑轴；首次绑定前为 AXIS_NULL。 */
    bool bSegmentBound;       /**< TRUE 表示当前标量控制器已绑定运动段。 */
    bool bFault;              /**< 锁存的故障标志，不能仅靠继续摇动手轮恢复。 */
    bool bInputStop;          /**< 采样入口请求停摇制动；不能直接清零实际速度。 */
    bool bFeedLimitActive;    /**< 已提交速度到达绑定上限后锁存限额，成功提交零速后解除。 */
};

/** 无副作用的单拍预览结果；同一中断内输出成功后才能提交，预览本身不扣减距离。 */
struct SHandwheelSimTrial
{
    double dPosition; /**< 本拍候选理论段进度，范围为 [0, 段长]。 */
    double dVelocity; /**< 本拍候选周期末有符号速度，单位为路径单位/ms。 */
    enum HANDWHEEL_SIM_STEP eStep; /**< 本拍等待、运动、段完成或暂停完成事件。 */
};

#define HWSIM_PATH_MAP_COUNT 256
#define HWSIM_MAP_INTERVALS_PER_TICK 4
/** 合成轴复用自动插补公共进度，不把各轴角度直接当成空间毫米。 */
struct SHandwheelPath
{
    struct SHandwheelSim SState; /**< 自动插补公共进度、速度和授权。 */
    bool bBound; /**< 完成全部校验后才接受编码器输入。 */
    bool bPreparing; /**< 直径圆弧分拍准备中；此时不接受授权、不输出运动。 */
    bool bUseDistanceMap; /**< 仅直径圆弧需要非线性输入换算。 */
    bool bArc; /**< TRUE 为 G2/G3，否则为 G1。 */
    int8 cBindResult; /**< 绑定诊断：1 成功，-2 模式，-3 几何，-4 约束，-5 状态，-6 映射表，-7 坐标。 */
    int16 sLastPulseArray[AXIS_NUM_MAX]; /**< 最后成功帧，故障候选不得覆盖原报警停车的速度来源。 */
    double dFeedRatio; /**< 绑定时的进给倍率；运行中改变时请求暂停重绑。 */
    double dFixedSpeedLimit; /**< 几何及机床约束的固定上限，G99 实时限速不能超过它。 */
    double dProgramFeed; /**< 绑定时的公共进给，G99 为公共进度单位/r。 */
    bool bPerRevolution; /**< TRUE 表示 G99，速度上限跟随实测主轴转速。 */
    bool bSpindleStop; /**< G99 主轴停转后锁存制动，零速取消余量前不接受输入。 */
    int nDistanceMapIndex; /**< 已完成的输入换算表区间数。 */
    double dInputDistanceScale; /**< 输入距离/公共进度，普通路径为 1，直径直线绑定时计算。 */
    double dInputRateToProgress; /**< 直径圆弧导数到公共进度输入单位的固定比例。 */
    double dAxisMovePerProgressArray[AXIS_NUM_MAX]; /**< 实际轴位移/公共进度；旋转轴以度计，不重复乘当量系数。 */
    double dMetricArray[HWSIM_PATH_MAP_COUNT + 1]; /**< 显示合成距离的累计积分。 */
    double dMetricRateArray[HWSIM_PATH_MAP_COUNT + 1]; /**< 对归一化几何进度的导数。 */
};

/*************************************************
  函数名称：executeHandwheelPathStep
  函数功能：预览、输出和提交一个合成轴，统一处理端点和暂停
  输入参数：pChannel - 通道；bArc - 圆弧标志
  输出参数：pChannel - 所有参与轴坐标及合成账本
  返 回 值：1 - 执行或等待；0 - 完成；-1 - 故障
  备    注：段首受原历史屏障约束，段间零速交接且不传递剩余授权
****************************************/
int8 executeHandwheelPathStep(struct SChannel* pChannel, bool bArc);

/** 通道总控；G0 分别授权各轴，G1/G2/G3 只授权一份合成路径距离。 */
struct SHandwheelSimGroup
{
    struct SHandwheelSim SAxisStateArray[AXIS_NUM_MAX]; /**< 按逻辑轴索引的独立距离账本。 */
    struct SHandwheelPath SPath; /**< G1/G2/G3 独享的合成距离账本。 */
    bool bInstructionBound; /**< TRUE 表示当前指令可接收输入；段间输入不缓存。 */
    bool bFault; /**< 任一轴发生错误后锁存，禁止继续正常运动。 */
    bool bAwaitingFreshInput; /**< 模式切换后等待编码器给出一次有效方向。 */
    bool bReverseBlocked; /**< 全轴位于可回退首端时，静默丢弃继续向外的输入。 */
    int8 cInputDirection; /**< 本段最后一份有效输入方向；零表示绑定后尚无输入。 */
    int8 cBoundaryDirection; /**< 未绑定时记录刚完成的端点方向，仅用于段间游标校准。 */
    int8 cInputRateDirection; /**< 测速方向；换向清窗口，禁止沿用旧高速状态。 */
    bool bFastInput; /**< 增益大于一的诊断标志；不以此切换控制模式。 */
    double dFilteredInputRate; /**< 逐拍低通的原始格速，格/秒。 */
    double dInputGain; /**< 平滑后的新增距离倍率，最小为 1。 */
    double dInputIdleTime; /**< 距最后有效输入的毫秒数，与测速窗口独立。 */
    double dInputGapEstimate; /**< 同方向有效输入的平均间隔，单位 ms。 */
    bool bInputActive; /**< 本段已接收生产采样，允许停摇计时；直接授权接口不置位。 */
};

/*************************************************
  函数名称：hasHandwheelReverseHistory
  函数功能：确认保留窗口内存在可回退运动，避免退入程序头空段
  输入参数：pChannel - 通道；bCompleted - TRUE 从刚完成段查起，FALSE 从上一段查起
  输出参数：无
  返 回 值：TRUE - 存在有效运动历史；FALSE - 已到回退边界
  备    注：只读扫描，不改变游标；空段前存在屏障时不得继续查找
****************************************/
bool hasHandwheelReverseHistory(const struct SChannel* pChannel, bool bCompleted);

/** 绑定一个单轴几何段所需的不可变参数，合并参数以避免接口参数过多。 */
struct SHandwheelSimSegment
{
    uint8 ucAxis;          /**< 当前段的逻辑轴号，必须小于 AXIS_NUM_MAX。 */
    double dLength;        /**< 当前段总长，G0 使用本轴单位，合成轴使用自动公共进度单位。 */
    double dPosition;      /**< 当前理论段进度，范围为 [0, dLength]。 */
    double dPeriod;        /**< 插补周期，单位为 ms。 */
    double dSpeedLimit;    /**< 正值速度上限，单位为路径单位/ms。 */
    double dAcceleration; /**< 正值加速度，单位为路径单位/ms^2。 */
    double dDeceleration; /**< 正值减速度，单位为路径单位/ms^2。 */
};

/*************************************************
  函数名称：isHandwheelDistanceControlActive
  函数功能：判断当前段是否由距离手轮接管，自动回退和恢复加工不接管
  输入参数：pChannel - 待检查通道，允许为空
  输出参数：无
  返 回 值：TRUE - 距离模式接管；FALSE - 沿用原运动路径
  备    注：不改变用户选择的开关；功能不支持时由 isHandwheelAutomaticMoveRequired 分流
****************************************/
bool isHandwheelDistanceControlActive(const struct SChannel* pChannel);

/*************************************************
  函数名称：isHandwheelAutomaticMoveRequired
  函数功能：判断当前程序段是否绕过手轮距离控制
  输入参数：pChannel - 待检查通道
  输出参数：无
  返 回 值：TRUE - 原自动路径；FALSE - 未请求手轮或可接管的运动
  备    注：只按功能能力分流，非法映射、几何及输出失败不能触发回退
****************************************/
bool isHandwheelAutomaticMoveRequired(const struct SChannel* pChannel);

/*************************************************
  函数名称：prepareHandwheelAutomaticMove
  函数功能：为自动回退段隔离旧授权并恢复正常运动倍率
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 输入状态及手轮速度倍率
  返 回 值：1 - 已准备或无需回退；-1 - 绑定未结束或已有故障
  备    注：不清除实际速度，不跳过暂停、复位、限位和报警
****************************************/
int prepareHandwheelAutomaticMove(struct SChannel* pChannel);

/*************************************************
  函数名称：resetHandwheelSim
  函数功能：清空单轴距离状态并将绑定轴置为 AXIS_NULL
  输入参数：pState - 待清理状态，允许为空
  输出参数：pState - 清零后的单轴状态
  返 回 值：无
  备    注：不是执行机构停止命令；仅初始化、确认零速或原停止路径接管后调用
****************************************/
void resetHandwheelSim(struct SHandwheelSim* pState);

/*************************************************
  函数名称：clearHandwheelSimMotion
  函数功能：全轴零速时清除已结束运动的授权和几何绑定
  输入参数：pGroup - 已初始化的通道状态
  输出参数：pGroup - 全轴零速才清理，带速或空指针时不修改
  返 回 值：无
  备    注：保留系统参数中的共享手轮档位，不用本函数代替减速停车
****************************************/
void clearHandwheelSimMotion(struct SHandwheelSimGroup* pGroup);

/*************************************************
  函数名称：resetHandwheelSimGroup
  函数功能：初始化通道及全部独立轴、合成轴的距离状态
  输入参数：pGroup - 通道状态，允许为空
  输出参数：pGroup - 清零后的控制状态；圆弧表失效并在下次绑定时重建
  返 回 值：无
  备    注：仅初始化、确认零速或原停止路径接管后调用，不修改共享手轮档位
****************************************/
void resetHandwheelSimGroup(struct SHandwheelSimGroup* pGroup);

/*************************************************
  函数名称：leaveHandwheelSim
  函数功能：退出距离型手轮模拟并交接当前周期末速度
  输入参数：pChannel - 当前通道
  输出参数：pChannel - 普通运动状态和距离模拟状态
  返 回 值：无
  备    注：幂等调用；重新接管前等待编码器给出有效方向
****************************************/
void leaveHandwheelSim(struct SChannel* pChannel);

/*************************************************
  函数名称：clearHandwheelAverageSpeedAfterRecovery
  函数功能：恢复加工完成后隔离末拍平均速度，供距离运动零速重绑
  输入参数：pChannel - 当前通道；bWasRecovering - 本拍调度前正在模拟恢复加工
  输出参数：pChannel - 仅更新已完成恢复的通道平均速度记录
  返 回 值：无
  备    注：在原 IRQ 调度后调用，不清本拍脉冲、坐标或活动轴速度
****************************************/
void clearHandwheelAverageSpeedAfterRecovery(struct SChannel* pChannel, bool bWasRecovering);

/*************************************************
  函数名称：isHandwheelSimGroupStopped
  函数功能：检查独立轴及合成轴的已提交周期末速度是否全部为零
  输入参数：pGroup - 通道距离状态
  输出参数：无
  返 回 值：TRUE - 全部为零；FALSE - 空指针或任一轴带速
  备    注：不使用平均速度判断；软件零速不代表伺服反馈机械停稳
****************************************/
bool isHandwheelSimGroupStopped(const struct SHandwheelSimGroup* pGroup);

/*************************************************
  函数名称：addHandwheelGroupAuthorizedDistance
  函数功能：提供不依赖机床轴配置的基础距离授权接口
  输入参数：pGroup - 通道状态；nSignedRaw - 有符号计数；ucMultiple - 倍率
  输出参数：pGroup - 全部校验通过后更新各轴授权
  返 回 值：1 - 接收；0 - 丢弃；-1 - 校验失败
  备    注：生产编码器采样通过 addHandwheelChannelAuthorizedDistance 按物理轴换算
****************************************/
int addHandwheelGroupAuthorizedDistance(struct SHandwheelSimGroup* pGroup, int32 nSignedRaw, uint8 ucMultiple);

/*************************************************
  函数名称：addHandwheelAuthorizedDistance
  函数功能：将原始计数按有效每格距离系统参数和手轮档位累计为待走距离
  输入参数：pState - 接收状态；nSignedRaw - 有符号原始计数；ucMultiple - 1、10、100 档位
  输出参数：pState - 待走距离；参数无效时仅锁存故障
  返 回 值：1 - 接受输入；-1 - 空指针、已有故障或参数无效
  备    注：非正数参数按默认 1 处理；仅新增授权使用新值，不参与测速
****************************************/
int addHandwheelAuthorizedDistance(struct SHandwheelSim* pState, int32 nSignedRaw, uint8 ucMultiple);

/*************************************************
  函数名称：processHandwheelInput
  函数功能：每插补周期采样一次，按转速比例倍率和直径参数累计新授权
  输入参数：pChannel - 当前通道，允许为空
  输出参数：pChannel - 输入测速及授权状态，非法参数锁存故障
  返 回 值：无
  备    注：零计数周期也更新测速；共享编码器只读，暂停减速沿用原路径
****************************************/
void processHandwheelInput(struct SChannel* pChannel);

/*************************************************
  函数名称：prepareHandwheelMotionSegment
  函数功能：在零速时绑定单轴几何段及固定规划约束
  输入参数：pState - 单轴状态；pSegment - 几何和机床约束参数
  输出参数：pState - 校验成功后的绑定状态，失败时不修改
  返 回 值：1 - 成功；-1 - 状态或参数无效
  备    注：调用者先检查轴类型、映射及限位；不强行裁剪位置或清除非零速度
****************************************/
int prepareHandwheelMotionSegment(struct SHandwheelSim* pState, const struct SHandwheelSimSegment* pSegment);

/*************************************************
  函数名称：calculateHandwheelMotionStep
  函数功能：按约束预览一拍运动、暂停或停摇制动，不修改账本
  输入参数：pState - 已绑定状态；bStop - 程序暂停请求
  输出参数：pTrial - 候选进度、周期末速度及事件
  返 回 值：1 - 成功；-1 - 参数无效或段边界内无法制动
  备    注：bInputStop 请求独立停摇事件；非零速度不得直接反向
****************************************/
int calculateHandwheelMotionStep(const struct SHandwheelSim* pState, bool bStop, struct SHandwheelSimTrial* pTrial);

/*************************************************
  函数名称：updateHandwheelMotionState
  函数功能：提交输出已接受的理论位置和末速，更新剩余授权
  输入参数：pTrial - 同一中断中通过预览及输出检查的候选
  输出参数：pState - 已提交的账本，调用方保证指针有效
  返 回 值：无
  备    注：暂停或停摇停稳时取消余量；成功零速解除到速限额，停摇保持几何绑定
****************************************/
void updateHandwheelMotionState(struct SHandwheelSim* pState, const struct SHandwheelSimTrial* pTrial);
#endif
#endif
