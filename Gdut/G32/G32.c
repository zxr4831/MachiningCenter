#include "G32.h"
#include <math.h>
#include <stdlib.h>
#include "CTrajPlanning.h"
#include "common.h"
#include "GeneralFunctions.h"
#include "Motion.h"
#include "math.h"
#include "Spindle.h"
#include "G1.h"
#include "G0.h"
#include "ringbuff.h"
#include "Channel.h"
//#include "pwm.h"
#include "Config.h"
#include "ErrorCompensation.h"
const double TAPER_45 = 3.1415926 / 4; //45°对应的弧度
//struct _SG32 g_SG32;          //螺纹类参数结构体


int8 SG32LoadSysPara(struct _SG32* pG32)
{
	int i;
	const uint8* ucMapArray;//映射
	if (pG32 == NULL)
	{
		return -1;
	}
	//映射
	ucMapArray = pG32->pSChannel->pMapAxisArray;

	//G1脉冲当量初始化
	for (i = 0; i < AXIS_NUM_MAX; i++)
	{
		if (ucMapArray[i] >= AXIS_PHYSICS_NUM_MAX) continue;//空轴 跳过
		pG32->dPulseToMm[i] = g_SSystemPara.SAxisPara[ucMapArray[i]].dPulseToMm;
	}
	//插补周期
	pG32->SPlanData.dTc = g_SSystemPara.dInterpolationTime;

	return 1;


}

int g_nSpindleAccum; //模拟编码器函数中，主轴编码器所显示的数值
///*************************************************
//  函数名称：int8 chooseInterpPlaneForG32(struct _SG32* pG32)
//  函数功能：选择G32插补平面
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
int8 chooseInterpPlaneForG32(struct _SG32* pG32)
{
	if (pG32 == NULL) return -1;
	switch (getBits(pG32->unCommandType, CMD_PLANE_ADDR))
	{
	case CMD_PLANE_G18:
	default:
		pG32->SUserData.ucAxis[0] = AXIS_Z;
		pG32->SUserData.ucAxis[1] = AXIS_X;
		break;
	case CMD_PLANE_G17:
		pG32->SUserData.ucAxis[0] = AXIS_X;
		pG32->SUserData.ucAxis[1] = AXIS_Y;
		break;
	case CMD_PLANE_G19:
		pG32->SUserData.ucAxis[0] = AXIS_Y;
		pG32->SUserData.ucAxis[1] = AXIS_Z;
		break;
	}
	return 1;
}
//
///*************************************************
//  函数名称：int GetSpindleEncoder()
//  函数功能：模拟显示主轴编码器
//  输入参数：无
//  输出参数：无
//  返 回 值：当前周期模拟主轴编码器对应的数值
//  备    注：无
//**************************************************/
int GetSpindleEncoder()  //模拟显示主轴编码器，到时候做实物测试的时候删除
{

	//g_nSpindleAccum += round(g_SChannel[0].SSpindle.pSSpindleAxis->dSpindleSpeed * g_SEncoder[SPINDLE_ENCODER - 1].usEncoderCirclePulseNum);  //当前周期模拟主轴编码器对应的数值;
	//g_nSpindleAccum += round(g_SSpindle.pSSpindleAxis->dSpindleSpeed * g_SEncoder[SPINDLE_ENCODER - 1].usEncoderCirclePulseNum);  //当前周期模拟主轴编码器对应的数值;
	return g_nSpindleAccum;  //返回该值

}


///*************************************************
//  函数名称：void getThreadCutType(struct _SG32* pG32)
//  函数功能：根据传入的螺纹类指令参数结构体，判断此次切削螺纹的类型
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
//void getThreadCutType(struct _SG32* pG32)
//{
//	double dCutDisX = 0;	//X方向切削的大小
//	double dCutDisZ = 0;	//Z方向切削的大小
//	double dTaper = 0;		//锥度
//
//	dCutDisX = fabs(pG32->SEndPoint.dX - pG32->SStarCutPoint.dX);
//	dCutDisZ = fabs(pG32->SEndPoint.dZ - pG32->SStarCutPoint.dZ);
//
//	//螺纹切削类型的判断
//	if (isZero(dCutDisZ))
//	{
//		pG32->ThreadCutType = ENDFACE_THREAD;  //为端面螺纹切削
//#if LHK_DEBUG
//		printf("端面螺纹切削\r\n");
//#endif
//
//	}
//	else
//	{
//		dTaper = atan(dCutDisX / dCutDisZ);  //螺纹锥度计算
//		if (dTaper > TAPER_45)  //锥度大于45，长轴为X轴
//		{
//
//			pG32->ThreadCutType = TAPER45_MORE_THREAD;  //切削螺纹的锥度大于45°
//#if LHK_DEBUG
//			printf("锥度大于45\r\n");
//#endif
//
//		}
//		else  ////锥度小于45，长轴为Z轴
//		{
//			pG32->ThreadCutType = TAPER45_NOMORE_THREAD;  //切削螺纹的锥度小于45°（直螺纹或锥螺纹）
//#if LHK_DEBUG
//			printf("锥度小于45\r\n");
//#endif
//
//		}
//
//	}
//
//
//}
//
//
///*************************************************
//  函数名称：bool isGetSyncSignal(double dTargetPositionPulse)
//  函数功能：判断是否达到发送同步信号的条件，从而根据条件发送同步信号
//  输入参数：dTargetPositionPulse    double          目标位置的脉冲数
//  输出参数：g_SEncoder[SPINDLE_ENCODER - 1].dEncoderNumDeviation       主轴编码器结构体的参数            该插补周期编码器的位置与理想位置的偏差（pulse）
//  返 回 值：TRUE   获取同步信号
//			FALSE  没获取同步信号
//  备    注：无
//**************************************************/
bool isGetSyncSignal(double dTargetPositionPulse, struct SChannel* pSChannel, struct SEncoder* pSSpindleEncoder)
{

	int nEncoderNum;		 //当前周期编码器的实际位置，用于和nPulseQ做比较判断是否获得同步信号
	int nDeltaQ;			 //同步信号范围区间，取当前主轴插补周期的脉冲偏差的1.1倍的一半作为范围区间
	double dPulseQLeftRegion;   //Q值的左区间值
	double dPulseQRightRegion;  //Q值的右区间值

	if (pSSpindleEncoder == NULL || pSSpindleEncoder->usEncoderCirclePulseNum <= 0)//避免编码器线数为0
	{
		parseRunningError(9999,GCODE_ERROR_NUM153,9999);
		return FALSE;
	}

	nDeltaQ = (int)ceil((1.1 * pSSpindleEncoder->usDelta) * 0.5);  //计算同步信号范围区间

	dPulseQLeftRegion = dTargetPositionPulse - nDeltaQ;  //计算Q值的左区间值
	if (dPulseQLeftRegion < 0)  //若左区间的值小于0
	{
		dPulseQLeftRegion = pSSpindleEncoder->usEncoderCirclePulseNum + dPulseQLeftRegion;  //修正左区间的值	
	}

	dPulseQRightRegion = dTargetPositionPulse + nDeltaQ;  //计算Q值的右区间值
	if (dPulseQRightRegion > pSSpindleEncoder->usEncoderCirclePulseNum)  //若右区间的值超过编码器旋转一圈的脉冲值
	{
		dPulseQRightRegion = surplusDoubleData(dPulseQRightRegion, pSSpindleEncoder->usEncoderCirclePulseNum);  //修正右区间的值
	}

	//实际获取的主轴编码器数值落后一个周期，所以在这补上
	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转

	{
		nEncoderNum = (pSSpindleEncoder->lEncoderAbsValue + pSSpindleEncoder->usDelta)
			% pSSpindleEncoder->usEncoderCirclePulseNum;  //获取当前周期主轴编码器的位置


	}
	else  //若主轴反转
	{
		nEncoderNum = (pSSpindleEncoder->lEncoderAbsValue - pSSpindleEncoder->usDelta)
			% pSSpindleEncoder->usEncoderCirclePulseNum;  //获取当前周期主轴编码器的位置

	}

	if (nEncoderNum < 0)  //若主轴为反转
	{
		nEncoderNum += pSSpindleEncoder->usEncoderCirclePulseNum;  //修正当前周期主轴编码器的位置
	}


	if (dPulseQRightRegion - dPulseQLeftRegion >= 0)  //左右区间值都在0的同一侧
	{
		if (nEncoderNum >= dPulseQLeftRegion && nEncoderNum <= dPulseQRightRegion)  //若当前周期主轴编码器的位置位于Q值的左右区间内
		{
			if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
			{
				pSSpindleEncoder->dEncoderNumDeviation = (nEncoderNum - dTargetPositionPulse);  //获得主轴启动位置偏差
			}
			else  //若主轴反转
			{
				pSSpindleEncoder->dEncoderNumDeviation = -(nEncoderNum - dTargetPositionPulse);  //获得主轴启动位置偏差

			}
			return TRUE;  //发送同步信号

		}

	}
	else  //左右区间值在0的不同测
	{
		bool bInLeftRegion = (nEncoderNum >= dPulseQLeftRegion && nEncoderNum < pSSpindleEncoder->usEncoderCirclePulseNum);  //主轴编码器的位置是是否在左区间
		bool bInRightRegion = (nEncoderNum <= dPulseQRightRegion && nEncoderNum >= 0);  //主轴编码器的位置是是否在右区间

		bool bInLeftRegionQ = (dTargetPositionPulse >= dPulseQLeftRegion && dTargetPositionPulse < pSSpindleEncoder->usEncoderCirclePulseNum);  //Q值对应的脉冲在0的左边
		bool bInRightRegionQ = (dTargetPositionPulse <= dPulseQRightRegion && dTargetPositionPulse >= 0);  //Q值对应的脉冲在0的右边

		if (bInLeftRegion || bInRightRegion)  //当前主轴编码器的位置在左区间内或右区间内
		{
			if ((bInLeftRegion && bInLeftRegionQ) || (bInRightRegion && bInRightRegionQ))  //该插补周期主轴编码器的位置与Q值在0的同一侧
			{
				if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴为正转
				{
					pSSpindleEncoder->dEncoderNumDeviation = (nEncoderNum - dTargetPositionPulse);  //获得主轴启动位置偏差
				}
				else  //若主轴为反转
				{
					pSSpindleEncoder->dEncoderNumDeviation = -(nEncoderNum - dTargetPositionPulse);  //获得主轴启动位置偏差

				}

				return TRUE;  //发送同步信号

			}
			else  //该插补周期编码器的位置与Q值在0的不同侧
			{
				double dDeltaLeft;  //左区间的脉冲差值
				double dDeltaRight; //右区间的脉冲差值


				if (bInLeftRegion && bInRightRegionQ)  //若主轴编码器在左区间且Q值对应的脉冲在0的右边
				{
					dDeltaLeft = pSSpindleEncoder->usEncoderCirclePulseNum - nEncoderNum;  //获取左区间的脉冲差值
					dDeltaRight = dTargetPositionPulse;                                                  //获取右区间的脉冲差值

				}
				else //若主轴编码器在右区间且Q值对应的脉冲在0的左边
				{
					dDeltaLeft = -(pSSpindleEncoder->usEncoderCirclePulseNum - dTargetPositionPulse);   //获取左区间的脉冲差值
					dDeltaRight = -nEncoderNum;                                                                       //获取右区间的脉冲差值

				}

				if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴为正转
				{
					pSSpindleEncoder->dEncoderNumDeviation = -(dDeltaLeft + dDeltaRight);  //获得主轴启动位置偏差
				}
				else  //若主轴为反转
				{
					pSSpindleEncoder->dEncoderNumDeviation = (dDeltaLeft + dDeltaRight);   //获得主轴启动位置偏差

				}
				return TRUE;  //发送同步信号

			}

		}

	}

	return FALSE;  //当前运动周期没有获得同步信号


}

///*************************************************
//  函数名称：void G32Init(struct _SG32 *pG32)
//  函数功能：初始化执行G32指令所需参数
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void G32Init(struct _SG32* pG32)
{
	pG32->bSyncSignal = FALSE;        //同步信号标志位初始化
	pG32->bLeftTailFlag = FALSE;      //退尾标志位初始化 
	pG32->bDecPlanTransitionFlag = FALSE; //连续G32减速过渡标志位初始化

	pG32->unCurrentT = 0;				//过渡阶段中插补时间节点初始化
	pG32->unTac = 0;                  //过渡阶段总的插补节点初始化
	pG32->dNextG32LongAxisSpeed = 0;  //连续G32中下一个G32指令的长轴速度初始化
	pG32->ucMotionState = 0;          //运动状态初始化
	pG32->dIdealTransmitStartPos = 0; //理想开始减速过渡位置初始化

	pG32->dRemainDistance = 0;        //连续G32剩余距离初始化
	pG32->dVs = 0;                    //连续G32起始速度（过渡段的起始速度是当前速度，所以需要存储记录）初始化
	pG32->dDec = 0;                   //连续G32过渡段减速度初始化
	pG32->dIdealSpindlePosition = 0;  //连续G32终点处理想的主轴位置
	pG32->dSpindleSpeed = 0;          //主轴转速初始化
	pG32->dStartFollowDis = 0;        //开始随动所需的长度初始化（用于端口查看）
	pG32->bStartCompensate = TRUE;    //开始补偿标志位初始化

	pG32->dRatioXZ = 0;               //X轴伺服电机与Z轴伺服电机的脉冲之比（用于切削锥螺纹,带短轴向量）

	free(pG32->SExcessPlanForG32);    //释放G32特殊规划指针
	pG32->SExcessPlanForG32 = NULL;   //G32特殊规划指针初始化
	pG32->dShortAxisCompensate = 0;   //短轴补偿量初始化
	pG32->dStartPointCompensateCopy = 0;  //长轴起始补偿量副本
	pG32->bExcessPlanFlag = FALSE;        //不再对短轴进行速度规划
	


}

///*************************************************
//  函数名称：void initThreadPreStructData(struct _SG32* pG32)
//  函数功能：根据刀补后的结构体参数初始化螺纹类指令参数结构体
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void initThreadPreStructData(struct _SG32* pG32)
{
	//G32螺纹切削参数初始化初始化
	G32Init(pG32);

	//计算螺纹长短轴各自所需的参数
	G32Prepare(pG32);


}

///*************************************************
//  函数名称：void processAxisForG34BackTail(struct _SG32* pG32, double* dCutDisFirstAxis, double* dCutDisSecondAxis)
//  函数功能：G34退尾时的轴处理
//  输入参数：*pG32，            结构体struct _SG32指针
//            *dCutDisFirstAxis， double*    插补0号轴的切削距离
//			  *dDistFromSpindle   double*    插补1号轴的切削距离
//  输出参数：*pG32，   结构体struct _SG32指针
//			  *dCutDisFirstAxis， double*    插补0号轴的切削距离
//			  *dDistFromSpindle   double*    插补1号轴的切削距离
//  返 回 值：无
//  备    注：无
//**************************************************/
void processAxisForG34BackTail(struct _SG32* pG32, double* dCutDisFirstAxis, double* dCutDisSecondAxis)
{
	double dIdealCutDisFirstAxis;           //0号轴理想的切削量（起点为上一段G32的终点）
	double dIdealCutDisSecondAxis;          //1号轴理想的切削量
	pG32->ucAxis[0] = pG32->ucLastAxis[0];  //重算长轴轴号
	pG32->ucAxis[1] = pG32->ucLastAxis[1];  //重算短轴轴号
	dIdealCutDisFirstAxis = pG32->SEndPoint.dArray[pG32->ucAxis[0]] - pG32->SLastEndPoint.dArray[pG32->ucAxis[0]];    //获取执行插补0号轴的切削量
	dIdealCutDisSecondAxis = pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SLastEndPoint.dArray[pG32->ucAxis[1]];   //获取执行插补1号轴的切削量
	*dCutDisFirstAxis = pG32->SEndPoint.dArray[pG32->ucAxis[0]] - pG32->SStarCutPoint.dArray[pG32->ucAxis[0]];        //获取执行插补0号轴的切削量
	*dCutDisSecondAxis = pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SStarCutPoint.dArray[pG32->ucAxis[1]];       //获取执行插补1号轴的切削量
	pG32->dRatioXZ = isZero(dIdealCutDisFirstAxis)? 0 : dIdealCutDisSecondAxis / fabs(dIdealCutDisFirstAxis);  //获取短轴伺服电机与长轴伺服电机的脉冲之比(带短轴矢量)
}

///*************************************************
//  函数名称：void G32Prepare(struct _SG32* pG32)
//  函数功能：获取螺纹运动中的比例系数以及长轴插补的距离
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void G32Prepare(struct _SG32* pG32)
{
	double dCutDisFirstAxis; 	   //执行插补0号轴的切削量（带矢量）
	double dCutDisSecondAxis;	   //执行插补1号轴的切削量（带矢量）
	bool bAxisNumChangeFlag = FALSE;       //轴号变化标志位
	uint8 *pMapAxisArray = pG32->pSChannel->pMapAxisArray;
	if(pG32->pSChannel->pMapAxisArray[pG32->ucAxis[0]] >= AXIS_PHYSICS_NUM_MAX || pG32->pSChannel->pMapAxisArray[pG32->ucAxis[1]] >= AXIS_PHYSICS_NUM_MAX)
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM150,9999);
		return;
	}

	dCutDisFirstAxis  = pG32->SEndPoint.dArray[pG32->ucAxis[0]] - pG32->SStarCutPoint.dArray[pG32->ucAxis[0]];  //获取执行插补0号轴的切削量
	dCutDisSecondAxis = pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SStarCutPoint.dArray[pG32->ucAxis[1]];  //获取执行插补1号轴的切削量

	 //如果1号轴的切削量 > 0号轴的切削量， 交换两轴的位置保证轴0所指向的轴为长轴
	if (fabs(dCutDisSecondAxis) > fabs(dCutDisFirstAxis))
	{
		uint8 ucTemp = pG32->ucAxis[0];
		double dCutDisTemp = dCutDisFirstAxis;
		pG32->ucAxis[0] = pG32->ucAxis[1];
		pG32->ucAxis[1] = ucTemp;
		dCutDisFirstAxis = dCutDisSecondAxis;
		dCutDisSecondAxis = dCutDisTemp;
		bAxisNumChangeFlag = TRUE;
	}



	pG32->dMmToPulse = 1.0 / g_SSystemPara.SAxisPara[pG32->pSChannel->pMapAxisArray[pG32->ucAxis[0]]].dPulseToMm;              //获取长轴的脉冲当量
																														   
	//根据螺纹切削的类型，对运动轴的轴号、比例系数等进行处理																														   
	if (isZero(pG32->dRadius))  //若不为圆弧面螺纹
	{
		if (pG32->bG34BackTailFlag)  //当前插补段为G34的退尾段
		{
			processAxisForG34BackTail(pG32, &dCutDisFirstAxis, &dCutDisSecondAxis);  //G34退尾段运动轴的处理
		}
		else
		{
			if (pG32->bRepeatG32Flag)  //为连续G32
			{
				double dIdealCutDisFirstAxis;  //0号轴理想的切削量（起点为上一段G32的终点）
				double dIdealCutDisSecondAxis; //1号轴理想的切削量

				dIdealCutDisFirstAxis  = pG32->SEndPoint.dArray[pG32->ucAxis[0]] - pG32->SLastEndPoint.dArray[pG32->ucAxis[0]];   //获取执行插补0号轴的切削量
				dIdealCutDisSecondAxis = pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SLastEndPoint.dArray[pG32->ucAxis[1]];   //获取执行插补1号轴的切削量

				//如果理想长轴和短轴长度相同（连续G32中，由于加速过渡会超过本次切削的距离，所以可能会导致长轴错误判断的情况），更改长短轴的轴号
				if (bAxisNumChangeFlag && (isZero(fabs(dIdealCutDisFirstAxis) - fabs(dIdealCutDisSecondAxis)) || fabs(dIdealCutDisFirstAxis) < fabs(dIdealCutDisSecondAxis)))
				{
					//更改长短轴的轴号和对应的插补距离
					uint8 ucTemp = pG32->ucAxis[0];
					double dCutDisTemp   = dCutDisFirstAxis;
					double dIdealDisTemp = dIdealCutDisFirstAxis;
					pG32->ucAxis[0] = pG32->ucAxis[1];
					pG32->ucAxis[1] = ucTemp;
					dCutDisFirstAxis = dCutDisSecondAxis;
					dCutDisSecondAxis = dCutDisTemp;
					dIdealCutDisFirstAxis = dIdealCutDisSecondAxis;
					dIdealCutDisSecondAxis = dIdealDisTemp;

				}
				pG32->dRatioXZ = isZero(dIdealCutDisFirstAxis)? 0 :dIdealCutDisSecondAxis / fabs(dIdealCutDisFirstAxis);  //获取理想的短轴伺服电机与长轴伺服电机的脉冲之比(带短轴矢量)
			}
			else  //为正常G32
			{
				pG32->dRatioXZ = isZero(dCutDisFirstAxis)? 0 :dCutDisSecondAxis / fabs(dCutDisFirstAxis);  //获取短轴伺服电机与长轴伺服电机的脉冲之比(带短轴矢量)

			}

		}
	
	}

	if (fabs(dCutDisFirstAxis) < g_SSystemPara.SAxisPara[pG32->pSChannel->pMapAxisArray[pG32->ucAxis[0]]].dPulseToMm)  dCutDisFirstAxis = 0;//若0号轴插补距离不到一个脉冲，直接置为0
	if (fabs(dCutDisSecondAxis) < g_SSystemPara.SAxisPara[pG32->pSChannel->pMapAxisArray[pG32->ucAxis[1]]].dPulseToMm) dCutDisSecondAxis = 0;//若1号轴插补距离不到一个脉冲，直接置为0
	pG32->SStartVector.dArray[pG32->ucAxis[1]] = 0;                      //重算短轴向量
	pG32->SStartVector.dArray[pG32->ucAxis[0]] = sgn(dCutDisFirstAxis);  //重算长轴向量
	pG32->SPlanData.dDistance = fabs(dCutDisFirstAxis);                  //获得长轴总的插补距离

	if (pG32->bSysDataInitFlag)
	{
		//2023.5.19
		if (g_SSystemPara.ucG32InputMode == 0)//G32的系统参数来源为加工系统参数
		{
			pG32->SUserData.dAcc  = g_SSystemPara.dSysPlanTMaxAcc;
			pG32->SUserData.dDec  = g_SSystemPara.dSysPlanTMaxDec;
			pG32->SUserData.dVs   = g_SSystemPara.dSysMinVel;
			pG32->SUserData.dVmax = g_SSystemPara.dSysMaxVel;
			pG32->SUserData.dVe   = g_SSystemPara.dSysMinVel;

		}
		else  //G32的系统参数来源为对应的单轴系统参数
		{
			pG32->SUserData.dAcc  = g_SSystemPara.SAxisPara[pMapAxisArray[pG32->ucAxis[0]]].dSysPlanTMaxAcc;
			pG32->SUserData.dDec  = g_SSystemPara.SAxisPara[pMapAxisArray[pG32->ucAxis[0]]].dSysPlanTMaxDec;
			pG32->SUserData.dVs   = g_SSystemPara.SAxisPara[pMapAxisArray[pG32->ucAxis[0]]].dVs;
			pG32->SUserData.dVmax = g_SSystemPara.SAxisPara[pMapAxisArray[pG32->ucAxis[0]]].dVmax;
			pG32->SUserData.dVe   = g_SSystemPara.SAxisPara[pMapAxisArray[pG32->ucAxis[0]]].dVe;

		}
		
		pG32->SUserData.dVmax = min(pG32->SUserData.dVmax, g_SSystemPara.dG32MaxVelocity);	//陈敏聪	2023/12/23	应企业要求增加G32专用速度
	}
	

}

void backTailShortAxisPlan(struct _SG32* pG32, double dShortAxisDistance)
{
	double dVs = fabs(pG32->dCurrentSpeed * pG32->dRatioXZ);
	double dVmax = g_SSystemPara.SAxisPara[pG32->ucAxis[1]].dVmax;
	double dVe   = g_SSystemPara.SAxisPara[pG32->ucAxis[1]].dVe;
	double dAcc  = g_SSystemPara.SAxisPara[pG32->ucAxis[1]].dSysPlanTMaxAcc;
	double dDec  = g_SSystemPara.SAxisPara[pG32->ucAxis[1]].dSysPlanTMaxDec;
	double dSac, dSconst, dSde;     //加速，匀速，减速距离
	double dDistance, dDistRemain;  //总距离,残余距离
	double dTac, dTconst, dTde;     //加速,匀速,减速周期
	double dTc;				        //插补周期
	double dTacP, dTconstP, dTdeP;  //根据长轴总的规划时间，所获得的短轴时间
	int nMaxNum = 0;  //0：加速段、 1：减速段、 2匀速段
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期

	dTc = pG32->SPlanData.dTc;
	dDistance = dShortAxisDistance;

	if (dVs < g_SSystemPara.SAxisPara[pG32->ucAxis[1]].dVs)
	{
		dVs = g_SSystemPara.SAxisPara[pG32->ucAxis[1]].dVs;

	}

	dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
	dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移

	if (dDistance < dSac + dSde) //路程不足加减速
	{
		//重算最大速度
		dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));

		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //重算加减速段位移
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
	}

	dTac = (dVmax - dVs) / (dAcc * dTc);                   //计算加速周期
	dTde = (dVmax - dVe) / (dDec * dTc);                   //计算减速周期
	dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

	if (dTconst <= 0)   //距离不足没有匀速的情况
	{
		if (dTac < 0 || dTde < 0)  //距离不足
		{
			//动态规划
			//Ve浮动
			if (dVs > dVe)//减速
			{
				dVe = sqrt(dVs * dVs - dDistance * 2.0 * dDec);//根据减速距离公式计算末速度(不会出现负值)
				dVmax = dVs;//由于是减速  最大速度等于初速度
			}
			else//加速
			{
				dVe = sqrt(dVs * dVs + 2.0 * dAcc * dDistance);//根据加速距离公式计算末速度
				dVmax = dVe;//由于是加速  最大速度等于末速度
			}
			dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		}
		dTconst = 0;//匀速周期不能小于0  将其置为0
	}

	if (dTac < 0)//距离足够 但是加速周期为负，加速段变为减速段
	{
		dTac = fabs(dTac);//加速周期为负数  取绝对值  
		dAcc = -(dAcc);//加速度为正  将其变为减速  取反
	}

	dTacP = dTac / (dTac + dTde + dTconst) * pG32->SPlanData.unT5;
	dTdeP = dTde / (dTac + dTde + dTconst) * pG32->SPlanData.unT5;
	dTconstP = dTconst / (dTac + dTde + dTconst) * pG32->SPlanData.unT5;

	if (dTacP > dTdeP && dTacP > dTconstP)
	{
		nMaxNum = 0;
	}
	else if (dTdeP > dTacP && dTdeP > dTconstP)
	{
		nMaxNum = 1;
	}
	else if (dTconstP > dTacP && dTconstP > dTdeP)
	{
		nMaxNum = 2;
	}
	else
	{
		nMaxNum = 1;
	}

	dTacP = round(dTacP);
	dTdeP = round(dTdeP);
	dTconstP = round(dTconstP);

	if (!isZero(dTacP + dTdeP + dTconstP - pG32->SPlanData.unT5))
	{
		int nExtime = pG32->SPlanData.unT5 - (dTacP + dTdeP + dTconstP);
		if (nMaxNum == 0)
		{
			dTacP += nExtime;
		}
		else if (nMaxNum == 1)
		{
			dTdeP += nExtime;
		}
		else
		{
			dTconstP += nExtime;
		}
	}

	dTac = dTacP;      //计算加速周期
	dTde = dTdeP;      //计算减速周期
	dTconst = dTconstP;   //计算匀速段周期

	if (((int)(dTac + 0.5)) == 0)//加速周期四舍五入后为0  (判断是否小于0.5)
	{
		dTac = 0;//加速周期为0
		dVs = dVmax;//起始速度等于最大速度
	}
	if (((int)(dTde + 0.5)) == 0)//减速周期四舍五入后为0 (判断是否小于0.5)
	{
		dTde = 0;//减速周期设为0
		/*		dVmax = dVe;*/
		dVe = dVmax;//末速度等于最大速度
	}


	if (isZero(dTac) || isZero(dVs - dVmax))//加速周期为0
	{
		dVs = dVmax;
	}
	else
	{
		dVmax = dAcc * (dTac * dTc) + dVs;//重算最大速度
	}

	if (isZero(dTde) || isZero(dVe - dVmax))  //减速周期为0 （前提是最大速度不会比终点速度小很多(>dDec)）   普通规划限制了末速度不超过最大速度  新增距离的规划没有限制  因此需要判断<=0
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{

		dDec = (dVmax - dVe) / (dTde * dTc);

	}

	dSac = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //加速段位移（时间取整后）
	dSde = (dVmax * dVmax - dVe * dVe) / (dDec * 2);   //减速段位移 

	//dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期
	//if (dTconst <= 0)  //没有匀速周期
	//{
	//	dTconst = 0;
	//}

	unT1 = unT2 = (uint32)dTac;               //时间节点T1  （绝对时间） 
	unT3 = unT2 + (uint32)dTconst; //时间节点T2  （绝对时间） 
	unT4 = unT5 = unT3 + (uint32)dTde;       //时间节点T3  （绝对时间） 
	dSconst = dVmax * (unT3 - unT2) * dTc; //计算匀速段位移
	dDistRemain = dDistance - dSac - dSde - dSconst;  //计算残余距离

	//传出参数
	pG32->SExcessPlanForG32->unT1 = unT1;		//时间节点T1,2,3
	pG32->SExcessPlanForG32->unT2 = unT2;
	pG32->SExcessPlanForG32->unT3 = unT3;
	pG32->SExcessPlanForG32->unT4 = unT4;
	pG32->SExcessPlanForG32->unT5 = unT5;

	pG32->SExcessPlanForG32->dDistAcc = dSac;			//加速,匀速,减速距离
	pG32->SExcessPlanForG32->dDistConst = dSconst;
	pG32->SExcessPlanForG32->dDistDec = dSde;

	pG32->SExcessPlanForG32->dVmax = dVmax;	//最大速度，终点速度
	pG32->SExcessPlanForG32->dVs = dVs;
	pG32->SExcessPlanForG32->dVe = dVe;
	pG32->SExcessPlanForG32->dAcc = dAcc;		//加速度，减速度
	pG32->SExcessPlanForG32->dDec = dDec;
	pG32->SExcessPlanForG32->dDistRemaining = dDistRemain; //残余距离

	pG32->SExcessPlanForG32->ucAccVelPlanType = VEL_PLAN_T_CURVE;
	pG32->SExcessPlanForG32->ucDecVelPlanType = VEL_PLAN_T_CURVE;
	pG32->SExcessPlanForG32->ucDistReAverageType = DIST_REAVG_T_CURVE;
	pG32->SExcessPlanForG32->dTc = dTc;

}

///*************************************************
//  函数名称：void retreatTailPre(struct _SG32* pG32)
//  函数功能：退尾预处理
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void retreatTailPre(struct _SG32* pG32)
{
	//double dRetreatRange = g_SEncoder[SPINDLE_ENCODER - 1].usDelta / g_SEncoder[SPINDLE_ENCODER - 1].lEncoderAbsValue * pG32->dF; //退尾范围
	//double dRetreatRange = g_SEncoder[SPINDLE_ENCODER - 1].usDelta * pG32->dF / g_SEncoder[SPINDLE_ENCODER - 1].usEncoderCirclePulseNum / 2.0; //退尾范围
	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
	if (pSSpindleEncoder == NULL || pSSpindleEncoder->usEncoderCirclePulseNum <= 0 )//如果编码器圈数等于0  则使用编码器值
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return;
	}
	double dRetreatRange = pSSpindleEncoder->usDelta * pG32->dF / pSSpindleEncoder->usEncoderCirclePulseNum / 2.0; //退尾范围

	double dTailSignDis; 					//理想退尾位置
	double dLongAxisVector;			        //长轴向量
	//double dJVector;				        //短轴退尾向量
	double dCutLeftDisLongAxis = 0; 	    //X方向剩余的切削量
	double dCutLeftDisShortAxis = 0;	    //Z方向剩余的切削量
	double dLongAxisDistance = 0;           //长轴距离
	double dShortAxisDistance = 0;		    //短轴距离
	uint8 ucLongAxisNum = pG32->ucAxis[0];  //获取长轴对应的轴号
	uint8 ucShortAxisNum = pG32->ucAxis[1]; //获取短轴对应的轴号
	SPointDouble SRealTimePoint;

	dLongAxisVector = pG32->SStartVector.dArray[ucLongAxisNum];  //获取长轴向量
	//dTailSignDis = pG32->SEndPoint.dArray[ucLongAxisNum] - pG32->dK * dLongAxisVector;  //获取长轴理想退尾位置
	// 长轴退尾量按长度使用，径向退尾方向仍由带符号的 J 决定。
    dTailSignDis = pG32->SEndPoint.dArray[ucLongAxisNum] - pG32->dK * dLongAxisVector;  //获取长轴理想退尾位置

	getRealTimePoint((double*)&SRealTimePoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, &pG32->ucValStatus);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机 
	if (pG32->pSChannel->bUseVirtualY == TRUE)
	{
		if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_X))
		{
			double dY1Mach = 0;
			getSingleAxisRealTimePoint(&dY1Mach, AXIS_Y, pG32->pSChannel->pMapAxisArray);//当前Y1轴的位置
			SRealTimePoint.dArray[AXIS_X] -= 1.0 * dY1Mach * pG32->pSChannel->dSINAngleYY1;
		}
		if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_Y))
		{
			SRealTimePoint.dArray[AXIS_Y] = SRealTimePoint.dArray[AXIS_Y] * pG32->pSChannel->dCOSAngleYY1;
		}
	}
#endif

	////超过理想开始退尾位置或在退尾区间内
	//if ((dTailSignDis * dLongAxisVector < SRealTimePoint.dArray[ucLongAxisNum] * dLongAxisVector) || (fabs(dTailSignDis - SRealTimePoint.dArray[ucLongAxisNum]) <= dRetreatRange))
	//{
	//	pG32->SEndPoint.dArray[ucShortAxisNum] += pG32->dJ;  //重算切削终点(短轴的终点被修改了，长轴终点不变)
	//	dCutLeftDisLongAxis = fabs(pG32->SEndPoint.dArray[ucLongAxisNum] - SRealTimePoint.dArray[ucLongAxisNum]); //获取长轴方向剩余的切削量（不带向量）
	//	dCutLeftDisShortAxis = pG32->SEndPoint.dArray[ucShortAxisNum] - SRealTimePoint.dArray[ucShortAxisNum];     //获取短轴方向剩余的切削量（带有向量）
	//	pG32->dRatioXZ = (dCutLeftDisShortAxis / dCutLeftDisLongAxis);   //重算长短轴之间的比例（带有短轴方向的向量）
	//	pG32->SBackTailPoint = SRealTimePoint;                           //获得实际的退尾位置
	//	pG32->bLeftTailFlag = TRUE;                                      //已完成退尾的预处理

	//}

	//超过理想开始退尾位置或在退尾区间内
	if ((dTailSignDis * dLongAxisVector < SRealTimePoint.dArray[ucLongAxisNum] * dLongAxisVector) || (fabs(dTailSignDis - SRealTimePoint.dArray[ucLongAxisNum]) <= dRetreatRange))
	{
		//长轴的速度规划
		pG32->SPlanData.dVs   = pG32->dCurrentSpeed;
		pG32->SPlanData.dVmax = pG32->dCurrentSpeed;
		pG32->SPlanData.dVe   = pG32->SUserData.dVe;
		pG32->SPlanData.dAcc  = pG32->SUserData.dAcc;
		pG32->SPlanData.dDec  = pG32->SUserData.dDec;
		pG32->SPlanData.dCurLeftDist = fabs(pG32->SEndPoint.dArray[ucLongAxisNum] - SRealTimePoint.dArray[ucLongAxisNum]);
		pG32->SPlanData.dDistance = pG32->SPlanData.dCurLeftDist;
		pG32->SPlanData.dPreDist = 0;
		pG32->SPlanData.dTargetDist = pG32->SPlanData.dCurLeftDist;
		pG32->SPlanData.cInterpDir = INP_GO_AHEAD;

		pG32->SPlanData.ucAccVelPlanType = VEL_PLAN_T_CURVE;
		pG32->SPlanData.ucDecVelPlanType = VEL_PLAN_T_CURVE;
		pG32->SPlanData.ucDistReAverageType = DIST_REAVG_T_CURVE;
		preparePlanningCombine(&pG32->SPlanData, TRUE);
		prepareDistReAvgT(&pG32->SPlanData, FALSE);

		//pG32->SStarCutPoint = SRealTimePoint;
		pG32->SEndPoint.dArray[ucShortAxisNum] += pG32->dJ;  //重算切削终点(短轴的终点被修改了，长轴终点不变)
		pG32->SBackTailPoint = SRealTimePoint;                           //获得实际的退尾位置
		pG32->SPlanData.unCurrentT = 0;
		pG32->SStartVector.dArray[pG32->ucAxis[1]] = 0;                     			 //重算短轴向量
		pG32->SStartVector.dArray[pG32->ucAxis[0]] = sgn(pG32->SEndPoint.dArray[ucLongAxisNum] - SRealTimePoint.dArray[ucLongAxisNum]);  //重算长轴向量 //2026.4.27
		dLongAxisDistance = pG32->SEndPoint.dArray[pG32->ucAxis[0]] - pG32->SBackTailPoint.dArray[pG32->ucAxis[0]];
		dShortAxisDistance = pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SBackTailPoint.dArray[pG32->ucAxis[1]];
		pG32->dRatioXZ = isZero(dLongAxisDistance) ? 0 : dShortAxisDistance/dLongAxisDistance;			//更换计算比值位置,2026.4.21
		pG32->bLeftTailFlag = TRUE;                                      //已完成退尾的预处理

		/*修改2025.11.27 使用G1进行退尾，不需要短轴进行速度规划*/
		// //短轴的速度规划
		// if (pG32->SExcessPlanForG32 == NULL)
		// {
		// 	pG32->SExcessPlanForG32 = malloc(sizeof(struct SPlan));  //申请存储短轴速度规划参数的空间
		// }
		
		// //申请空间成功
		// if (pG32->SExcessPlanForG32 != NULL)
		// {
			
		// 	pG32->SExcessPlanForG32->dCurLeftDist = fabs(pG32->SEndPoint.dArray[ucShortAxisNum] - SRealTimePoint.dArray[ucShortAxisNum]);
		// 	pG32->SExcessPlanForG32->dDistance = pG32->SExcessPlanForG32->dCurLeftDist;
		// 	pG32->SExcessPlanForG32->dPreDist = 0;
		// 	pG32->SExcessPlanForG32->dTargetDist = pG32->SExcessPlanForG32->dCurLeftDist;
		// 	pG32->SExcessPlanForG32->cInterpDir = INP_GO_AHEAD;
			
		// 	backTailShortAxisPlan(pG32, pG32->SExcessPlanForG32->dCurLeftDist);
		// 	prepareDistReAvgT(pG32->SExcessPlanForG32, FALSE);
		// }


	}


}
//
//不带避空距离
/////*************************************************
////  函数名称：double getRealTargetPulse(struct _SG32* pG32)
////  函数功能：获得真正的初始角Q对应的相对脉冲
////  输入参数：*pG32，   结构体struct _SG32指针
////  输出参数：无
////  返 回 值：真正的初始角Q对应的相对脉冲
////  备    注：无
////**************************************************/
//double getRealTargetPulse(struct _SG32* pG32)
//{
//
//	double dTac;//加速周期
//	double dSac;//加速距离
//
//	double dVs, dVmax;    //起始，最大速度
//	double dAcc;          //加速段的加速度
//	double dDistance;     //总插补距离
//	double dTc;			  //插补周期
//	double dPulseQ;       //启动位置Q的脉冲形式
//	double dSpindleSpeed; //主轴平均转速（pulse / ms）
//
//	double dEncoderPreAbsNum;       //开始随动时主轴编码器的绝对数值
//	double dEncoderBackAbsNum;		//从开始随动的位置反转到起始位置时，主轴编码器的绝对数值
//	double dEncoderBackNum;			//从开始随动的位置反转到起始位置时，主轴编码器的相对数值
//	double dSpindleRealSpeed = pG32->pSChannel->dSpindleRealSpeed;
//	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
//
//	//初始化参数(从结构体获取）
//	//dVs = g_SSystemPara.dSysMinVel;						//获取起始速度
//	dVs = pG32->SUserData.dVs;
//	dVmax = (dSpindleRealSpeed * pG32->dF);	//将主轴的随动速度作为最大速度
//	//dAcc = g_SSystemPara.dSysPlanTMaxAcc;					//将系统参数中的T型曲线加速度作为加速度
//	dAcc = pG32->SUserData.dAcc;
//	dTc = pG32->SPlanData.dTc;							//获取插补周期
//
//	//若最大起跳速度大于随动速度
//	if (dVs > dVmax)
//	{
//		dVs = dVmax;  //则另随动速度作为起始速度
//	}
//
//	dDistance = pG32->SPlanData.dDistance;  //获得长轴总的插补距离
//
//	//计算加速时间和加速位移
//	dTac = fabs((dVmax - dVs) / (dAcc * dTc));		   //获取加速周期
//	dTac = ceil(dTac);								   //加速时间向上取整
//	dVmax = dAcc * (dTac * dTc) + dVs;				   //重算最大速度
//	dSac = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //重算加速段位移
//	//dSac += dVs + 0.5 * dAcc * dTc;
//
//	if (dSac > dDistance || isZero(pG32->dF))  //若加速距离大于总的插补距离,或者没有导程
//	{
//		return 0;  //距离太短，没办法加速成功
//
//	}
//
//	dPulseQ = (pG32->dQ / 360000.0) * pSSpindleEncoder->usEncoderCirclePulseNum;  //获取启动位置Q的脉冲形式
//	dSpindleSpeed = dSpindleRealSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;         //获取主轴平均转速
//
//	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
//	{
//		//起始位置->随动位置
//		//dEncoderPreAbsNum = dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum + dPulseQ + dSpindleSpeed * g_SInterpolation.dTc;
//		dEncoderPreAbsNum = dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum + dPulseQ;
//
//
//		//随动位置->起始位置
//		dEncoderBackAbsNum = dEncoderPreAbsNum - dSpindleSpeed * dTac;
//		dEncoderBackNum = surplusDoubleData(dEncoderBackAbsNum, pSSpindleEncoder->usEncoderCirclePulseNum); //对主轴编码器取余
//
//
//	}
//	else  //若主轴反转
//	{
//		//起始位置->随动位置
//		//dEncoderPreAbsNum = dPulseQ - dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum - dSpindleSpeed * g_SInterpolation.dTc;
//		dEncoderPreAbsNum = dPulseQ - dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum;
//
//		//随动位置->起始位置
//		dEncoderBackAbsNum = dEncoderPreAbsNum + dSpindleSpeed * dTac;
//		dEncoderBackNum = surplusDoubleData(dEncoderBackAbsNum, pSSpindleEncoder->usEncoderCirclePulseNum);  //对主轴编码器取余
//
//
//	}
//
//	if (dEncoderBackNum < 0)  //若计算所得的主轴编码器的位置为负数
//	{
//		dEncoderBackNum += pSSpindleEncoder->usEncoderCirclePulseNum;  //修正主轴编码器的位置
//
//	}
//
//	pG32->unTac = (uint32)dTac;  //获取过渡阶段总的插补节点
//	pG32->dVs = dVs;              //获得过渡阶段的起始速度
//	return dEncoderBackNum;                 //返回初始角对应的相对脉冲
//
//}


//带避空距离
///*************************************************
//  函数名称：double getRealTargetPulse(struct _SG32* pG32)
//  函数功能：获得真正的初始角Q对应的相对脉冲
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：无
//  返 回 值：真正的初始角Q对应的相对脉冲
//  备    注：无
//**************************************************/
double getRealTargetPulse(struct _SG32* pG32)
{

	double dTac;//加速周期
	double dSac;//加速距离

	double dVs, dVmax;    //起始，最大速度
	double dAcc;          //加速段的加速度
	double dDistance;     //总插补距离
	double dTc;			  //插补周期
	double dPulseQ;       //启动位置Q的脉冲形式
	double dSpindleSpeed; //主轴平均转速（pulse / ms）

	double dEncoderPreAbsNum;       //开始随动时主轴编码器的绝对数值
	double dEncoderBackAbsNum;		//从开始随动的位置反转到起始位置时，主轴编码器的绝对数值
	double dEncoderBackNum;			//从开始随动的位置反转到起始位置时，主轴编码器的相对数值
	double dSpindleRealSpeed = pG32->pSChannel->dSpindleRealSpeed;
	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;

	double dSacBase = 0;         //螺纹车削的避空距离
	double dLongAxisAbsPos = 0;  //长轴的绝对位置

	//初始化参数(从结构体获取）
	//dVs = g_SSystemPara.dSysMinVel;						//获取起始速度
	dVs = pG32->SUserData.dVs;
	dVmax = fabs(dSpindleRealSpeed * pG32->dF);	//将主轴的随动速度作为最大速度
	//dAcc = g_SSystemPara.dSysPlanTMaxAcc;					//将系统参数中的T型曲线加速度作为加速度
	dAcc = pG32->SUserData.dAcc;
	dTc = pG32->SPlanData.dTc;							//获取插补周期

	if(dAcc <= 0 || dTc <= 0 || pSSpindleEncoder->usEncoderCirclePulseNum == 0)
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return 0;
	}

	//若最大起跳速度大于随动速度
	if (dVs > dVmax)
	{
		dVs = dVmax;  //则另随动速度作为起始速度
	}

	dDistance = pG32->SPlanData.dDistance;  //获得长轴总的插补距离

	//计算加速时间和加速位移
	dTac = fabs((dVmax - dVs) / (dAcc * dTc));		   //获取加速周期
	dTac = ceil(dTac);								   //加速时间向上取整
	dVmax = dAcc * (dTac * dTc) + dVs;				   //重算最大速度
	dSac = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //重算加速段位移
	//dSac += dVs + 0.5 * dAcc * dTc;

	dLongAxisAbsPos = pG32->SStarCutPoint.dArray[pG32->ucAxis[0]] - pG32->pSChannel->SRun.SCurrentOffset.dArray[pG32->ucAxis[0]];
	//获取避空距离
	if (isZero(dLongAxisAbsPos) || dLongAxisAbsPos < 0)  //从工件内开始切螺纹时，使用系统参数的最大避空距离，作为避空距离
	{
		if (isZero(g_SSystemPara.dG32SacBase))
		{
			parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
			return 0;
		}
		dSacBase = g_SSystemPara.dG32SacBase;
	}
	else  //从工件外开始切螺纹，以螺纹车削起点位置作为避空距离
	{
		dSacBase = dLongAxisAbsPos;
	}

	dSacBase = dSacBase * 0.95; //以理想的避空距离的0.95倍作为最终要求的避空距离

	//按系统所设的加速度计算得到的加速距离Sac,大于避空距离dSacBase
	if (dSac > dSacBase)
	{
		dSac = dSacBase;
		dVmax = (dSpindleRealSpeed * pG32->dF);	  //将主轴的随动速度作为最大速度
		dAcc  = (dVmax * dVmax - dVs * dVs) / (dSac * 2);
		dTac  = fabs((dVmax - dVs) / (dAcc * dTc));		   //获取加速周期
		dTac  = floor(dTac);								   //加速时间向下取整
		dVmax = dAcc * (dTac * dTc) + dVs;				   //重算最大速度
		dSac  = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //获得最终的加速段位移
	}

	if (dSac > dDistance || isZero(pG32->dF))  //若加速距离大于总的插补距离,或者没有导程
	{
		return 0;  //距离太短，没办法加速成功

	}

	dPulseQ = (pG32->dQ / 360000.0) * pSSpindleEncoder->usEncoderCirclePulseNum;  //获取启动位置Q的脉冲形式
	dSpindleSpeed = dSpindleRealSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;         //获取主轴平均转速

	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
	{
		//起始位置->随动位置
		//dEncoderPreAbsNum = dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum + dPulseQ + dSpindleSpeed * g_SInterpolation.dTc;
		dEncoderPreAbsNum = dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum + dPulseQ;


		//随动位置->起始位置
		dEncoderBackAbsNum = dEncoderPreAbsNum - dSpindleSpeed * dTac;
		dEncoderBackNum = surplusDoubleData(dEncoderBackAbsNum, pSSpindleEncoder->usEncoderCirclePulseNum); //对主轴编码器取余


	}
	else  //若主轴反转
	{
		//起始位置->随动位置
		//dEncoderPreAbsNum = dPulseQ - dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum - dSpindleSpeed * g_SInterpolation.dTc;
		dEncoderPreAbsNum = dPulseQ - dSac / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum;

		//随动位置->起始位置
		dEncoderBackAbsNum = dEncoderPreAbsNum + dSpindleSpeed * dTac;
		dEncoderBackNum = surplusDoubleData(dEncoderBackAbsNum, pSSpindleEncoder->usEncoderCirclePulseNum);  //对主轴编码器取余


	}

	if (dEncoderBackNum < 0)  //若计算所得的主轴编码器的位置为负数
	{
		dEncoderBackNum += pSSpindleEncoder->usEncoderCirclePulseNum;  //修正主轴编码器的位置

	}

	pG32->unTac = (uint32)dTac;   //获取过渡阶段总的插补节点
	pG32->dVs   = dVs;            //获得过渡阶段的起始速度
	pG32->dAcc  = dAcc;		      //获得过渡阶段的加速度
	return dEncoderBackNum;       //返回初始角对应的相对脉冲

}


/////*************************************************
////  函数名称：void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance)
////  函数功能：“减速”类型连续G32的速度规划
////  输入参数：*pG32，   结构体struct _SG32指针	
////			dLeftDistance       double              当前位置距离插补终点的距离
////  输出参数：*pG32，   结构体struct _SG32指针
////  返 回 值：无
////  备    注：无
////**************************************************/
//void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance)
//{
//	double dDistance = dLeftDistance; //长轴距离终点的长度（减速段位移）
//	double dRemainDistance;           //残余距离
//	double dTac;                      //减速周期
//
//	double dVs, dVe;    //起始，终点速度（最大速度为两者的最大值）
//	double dDec;        //减速度
//	double dTc;		    //插补周期
//
//	double dSpindleSpeed;               //主轴理论转速（pulse / ms）
//	double dDelta;                      //理想减速位置距离终点的距离
//	double dNowG32LongAxisSpeed;        //当前G32长轴的随动速度
//	double dIdealSpindlePosition;       //终点处主轴编码器的数值
//	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
//	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
//
//	dTc = pG32->SPlanData.dTc;  //获取插补周期
//	dNowG32LongAxisSpeed = pG32->dSpindleSpeed * pG32->dF;  //获取当前G32长轴的随动速度
//
//	//pInterpPrePointer->dCurrentSpeed = dNowG32LongAxisSpeed;  //重算当前速度（保证速度相同是为了保证过渡时间相同）
//	//dVs = pInterpPrePointer->dCurrentSpeed;                   //以当前速度作为起始速度
//	dVs = dNowG32LongAxisSpeed;                  //以当前G32的随动速度作为起始速度
//	dVe = pG32->dNextG32LongAxisSpeed;           //以下一个G32长轴的随动速度作为终点速度
//
//	//计算理想减速位置距离终点的距离
//	dDelta = (pG32->SEndPoint.dArray[ucLongAxisNum] - pG32->dIdealTransmitStartPos) * pG32->SStartVector.dArray[ucLongAxisNum];  //获取理想减速位置距离终点的距离
//	
//	dDec = (dVe * dVe - dVs * dVs) / (dDelta * 2);  //获取减速度
//	
//
//	if (isZero(dDec))  //若终点速度和当前速度相同，导致减速度为0
//	{
//		dTac = round(dDistance / dVs);              //将到达终点所需的时间向上取整
//		dRemainDistance = dDistance - dTac * dVs;   //获取时间取整后插补的残余距离
//	}
//	else  //若减速度不为0
//	{
//		dTac = fabs((dVe - dVs) / (dDec * dTc));           //获取减速时间
//		dTac = round(dTac);							       //减速时间四舍五入取整
//		if (isZero(dTac))
//		{
//			dTac = 1;
//		}
//		dVe = dDec * (dTac * dTc) + dVs;                   //重算终点速度
//		dDistance = (dVe * dVe - dVs * dVs) / (dDec * 2);  //重算减速段位移
//		dRemainDistance = dLeftDistance - dDistance;       //获取时间取整后插补的残余距离
//	}
//
//
//	dSpindleSpeed = pG32->dSpindleSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;  //获取主轴理论转速
//
//	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
//	{
//		dIdealSpindlePosition = pSSpindleEncoder->lEncoderAbsValue + pSSpindleEncoder->usDelta + dSpindleSpeed * dTac;  //获取终点处理想的主轴编码器数值
//		pG32->dIdealSpindlePosition = surplusDoubleData(dIdealSpindlePosition, pSSpindleEncoder->usEncoderCirclePulseNum);  //对理想编码器数值取余进而获得理论主轴编码器的位置
//
//	}
//	else  //若主轴反转
//	{
//		dIdealSpindlePosition = pSSpindleEncoder->lEncoderAbsValue - pSSpindleEncoder->usDelta - dSpindleSpeed * dTac;  //获取终点处理想的主轴编码器数值
//		pG32->dIdealSpindlePosition = surplusDoubleData(dIdealSpindlePosition, pSSpindleEncoder->usEncoderCirclePulseNum);  //对理想编码器数值取余进而获得理论主轴编码器的位置
//
//	}
//
//	if (isZero(pG32->dIdealSpindlePosition))
//	{
//		pG32->dIdealSpindlePosition = pSSpindleEncoder->usEncoderCirclePulseNum;
//
//	}
//
//	//将下述参数赋值到螺纹插补结构体
//	pG32->dRemainDistance = dRemainDistance;
//	pG32->unTac = dTac;
//	pG32->dVs = dVs;
//	pG32->dDec = dDec;
//
//}

/////*************************************************
////  函数名称：void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance)
////  函数功能：“减速”类型连续G32的速度规划
////  输入参数：*pG32，   结构体struct _SG32指针	
////			dLeftDistance       double              当前位置距离插补终点的距离
////  输出参数：*pG32，   结构体struct _SG32指针
////  返 回 值：无
////  备    注：无
////**************************************************/
//void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance)
//{
//	double dDistance = dLeftDistance; //长轴距离终点的长度（减速段位移）
//	double dRemainDistance;           //残余距离
//	double dTac;                      //实际减速时间
//	double dTacIdeal;                 //理论减速时间     
//
//	double dVs, dVe;    //起始，终点速度（最大速度为两者的最大值）
//	double dVsIdeal;    //理想的起始速度
//	double dDec;        //减速度
//	double dTc;		    //插补周期
//
//	double dSpindleSpeed;               //主轴理论转速（pulse / ms）
//	double dDelta;                      //理想减速位置距离终点的距离
//	double dNowG32LongAxisSpeed;        //当前G32长轴的随动速度
//	double dIdealSpindlePosition;       //终点处主轴编码器的数值
//	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
//	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
//
//	dTc = pG32->SPlanData.dTc;  //获取插补周期
//	dNowG32LongAxisSpeed = pG32->dSpindleSpeed * pG32->dF;  //获取当前G32长轴的随动速度
//
//	//pInterpPrePointer->dCurrentSpeed = dNowG32LongAxisSpeed;  //重算当前速度（保证速度相同是为了保证过渡时间相同）
//	dVs = pG32->dCurrentSpeed;                   //以当前速度作为起始速度
//	dVsIdeal = dNowG32LongAxisSpeed;			 //以当前G32的理想随动速度作为理想起始速度
//	dVe = pG32->dNextG32LongAxisSpeed;           //以下一个G32长轴的随动速度作为终点速度
//
//	//计算理想减速位置至终点所需要的时间
//	dDelta = (pG32->SEndPoint.dArray[ucLongAxisNum] - pG32->dIdealTransmitStartPos) * pG32->SStartVector.dArray[ucLongAxisNum];  //获取理想减速位置距离终点的距离
//	dDec = (dVe * dVe - dVsIdeal * dVsIdeal) / (dDelta * 2); //获取理想减速度
//	dTacIdeal = fabs((dVe - dVsIdeal) / (dDec * dTc));       //获取理想减速时间
//	dTacIdeal = round(dTacIdeal);
//	if (isZero(dTacIdeal))
//	{
//		dTacIdeal = 1;
//
//	}
//
//	//计算用于规划的减速度
//	dDec = (dVe * dVe - dVs * dVs) / (dLeftDistance * 2);
//
//	if (isZero(dDec))  //若终点速度和当前速度相同，导致减速度为0
//	{
//		dTac = round(dDistance / dVs);              //将到达终点所需的时间向上取整
//		dRemainDistance = dDistance - dTac * dVs;   //获取时间取整后插补的残余距离
//	}
//	else  //若减速度不为0
//	{
//		dTac = fabs((dVe - dVs) / (dDec * dTc));           //获取减速时间
//		dTac = round(dTac);							       //减速时间四舍五入取整
//		if (isZero(dTac))
//		{
//			dTac = 1;
//		}
//		dVe = dDec * (dTac * dTc) + dVs;                   //重算终点速度
//		dDistance = (dVe * dVe - dVs * dVs) / (dDec * 2);  //重算减速段位移
//		dRemainDistance = dLeftDistance - dDistance;       //获取时间取整后插补的残余距离
//	}
//
//	pG32->dStartPointCompensate += (dTacIdeal - dTac) * pG32->dNextG32LongAxisSpeed;
//
//
//	dSpindleSpeed = pG32->dSpindleSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;  //获取主轴理论转速
//
//	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
//	{
//		dIdealSpindlePosition = pSSpindleEncoder->lEncoderAbsValue + pSSpindleEncoder->usDelta + dSpindleSpeed * dTac;  //获取终点处理想的主轴编码器数值
//		pG32->dIdealSpindlePosition = surplusDoubleData(dIdealSpindlePosition, pSSpindleEncoder->usEncoderCirclePulseNum);  //对理想编码器数值取余进而获得理论主轴编码器的位置
//
//	}
//	else  //若主轴反转
//	{
//		dIdealSpindlePosition = pSSpindleEncoder->lEncoderAbsValue - pSSpindleEncoder->usDelta - dSpindleSpeed * dTac;  //获取终点处理想的主轴编码器数值
//		pG32->dIdealSpindlePosition = surplusDoubleData(dIdealSpindlePosition, pSSpindleEncoder->usEncoderCirclePulseNum);  //对理想编码器数值取余进而获得理论主轴编码器的位置
//
//	}
//
//	if (isZero(pG32->dIdealSpindlePosition))
//	{
//		pG32->dIdealSpindlePosition = pSSpindleEncoder->usEncoderCirclePulseNum;
//
//	}
//
//	//将下述参数赋值到螺纹插补结构体
//	pG32->dRemainDistance = dRemainDistance;
//	pG32->unTac = dTac;
//	pG32->dVs = dVs;
//	pG32->dDec = dDec;
//
//}

///*************************************************
//  函数名称：void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance)
//  函数功能：“减速”类型连续G32的速度规划
//  输入参数：*pG32，   结构体struct _SG32指针	
//			dLeftDistance       double              当前位置距离插补终点的距离
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void planDecForRepeatG32(struct _SG32* pG32, double dLeftDistance)
{
	double dDistance = dLeftDistance; //长轴距离终点的长度（减速段位移）
	double dRemainDistance;           //残余距离
	double dTac;                      //实际减速时间
	double dTacIdeal;                 //理论减速时间     

	double dVs, dVe;    //起始，终点速度（最大速度为两者的最大值）
	double dVsIdeal;    //理想的起始速度
	double dDec;        //减速度
	double dTc;		    //插补周期

	double dSpindleSpeed;               //主轴理论转速（pulse / ms）
	double dDelta;                      //理想减速位置距离终点的距离
	double dNowG32LongAxisSpeed;        //当前G32长轴的随动速度
	double dIdealSpindlePosition;       //终点处主轴编码器的数值
	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
	double dOverIdealPosComp = 0;           //当前位置所应该增加的主轴脉冲补偿量
	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;

	struct SToolCompensation* pSToolCompPointer = NULL;                //刀补后产生的结构体指针，用于读取下一个指令类型
	double dNextG32F;  //下一个G32的导程值

	if (pG32->pSChannel->pSSpindleEncoder == NULL)
	{
		return;
	}
	if(dLeftDistance <= 0 || pSSpindleEncoder->usEncoderCirclePulseNum == 0)
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return;
	}

	pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum); //获取"速度规划读"
	dNextG32F = pSToolCompPointer->dF;                                 //获取下一个G32的导程值

	dTc = pG32->SPlanData.dTc;  //获取插补周期
	dNowG32LongAxisSpeed = pG32->dSpindleSpeed * pG32->dF;  //获取当前G32长轴的随动速度

	dVs = pG32->dCurrentSpeed;                   //以当前速度作为起始速度
	dVsIdeal = dNowG32LongAxisSpeed;			 //以当前G32的理想随动速度作为理想起始速度
	dVe = pG32->dNextG32LongAxisSpeed;           //以下一个G32长轴的随动速度作为终点速度

	//计算理想减速位置至终点所需要的时间
	dDelta = (pG32->SEndPoint.dArray[ucLongAxisNum] - pG32->dIdealTransmitStartPos) * pG32->SStartVector.dArray[ucLongAxisNum];  //获取理想减速位置距离终点的距离
	if(isZero(dDelta) || isZero(dVe - dVsIdeal))
	{
		dTacIdeal = 0;
	}
	else
	{
		dDec = (dVe * dVe - dVsIdeal * dVsIdeal) / (dDelta * 2); //获取理想减速度
		dTacIdeal = fabs((dVe - dVsIdeal) / (dDec * dTc));       //获取理想减速时间
	}
	dTacIdeal = round(dTacIdeal);
	if (isZero(dTacIdeal))
	{
		dTacIdeal = 1;
	}

	//计算用于规划的减速度
	dDec = (dVe * dVe - dVs * dVs) / (dLeftDistance * 2);

	if (isZero(dDec))  //若终点速度和当前速度相同，导致减速度为0
	{
		dTac = round(dDistance / dVs);              //将到达终点所需的时间向上取整
		dRemainDistance = dDistance - dTac * dVs;   //获取时间取整后插补的残余距离
	}
	else  //若减速度不为0
	{
		dTac = fabs((dVe - dVs) / (dDec * dTc));           //获取减速时间
		dTac = ceil(dTac);							       //减速时间四舍五入取整
		if (isZero(dTac))
		{
			dTac = 1;
		}
		dVe = dDec * (dTac * dTc) + dVs;                   //重算终点速度
		dDistance = (dVe * dVe - dVs * dVs) / (dDec * 2);  //重算减速段位移
		dRemainDistance = dDistance - dLeftDistance;       //获取时间取整后插补的残余距离（超过当前终点的距离）
	}

	dOverIdealPosComp = (dDelta - dLeftDistance) / pG32->dF * pSSpindleEncoder->usEncoderCirclePulseNum;
	dSpindleSpeed = pG32->dSpindleSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;  //获取主轴理论转速

	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
	{
		dIdealSpindlePosition = pSSpindleEncoder->lEncoderAbsValue + pSSpindleEncoder->usDelta - dOverIdealPosComp + dSpindleSpeed * dTacIdeal;  //获取终点处理想的主轴编码器数值
		//减速过渡类型连续螺纹
		if (!isZero(pG32->dNextG32LongAxisSpeed - pG32->SUserData.dVe))
		{
			dIdealSpindlePosition += dRemainDistance / dNextG32F * pSSpindleEncoder->usEncoderCirclePulseNum;
		}
		pG32->dIdealSpindlePosition = surplusDoubleData(dIdealSpindlePosition, pSSpindleEncoder->usEncoderCirclePulseNum);  //对理想编码器数值取余进而获得理论主轴编码器的位置
		//pG32->dIdealSpindlePosition = dIdealSpindlePosition;

	}
	else  //若主轴反转
	{
		dIdealSpindlePosition = pSSpindleEncoder->lEncoderAbsValue - pSSpindleEncoder->usDelta + dOverIdealPosComp - dSpindleSpeed * dTacIdeal;  //获取终点处理想的主轴编码器数值
		//减速过渡类型连续螺纹
		if (!isZero(pG32->dNextG32LongAxisSpeed - pG32->SUserData.dVe))
		{
			dIdealSpindlePosition += -(dRemainDistance / dNextG32F * pSSpindleEncoder->usEncoderCirclePulseNum);
		}
		pG32->dIdealSpindlePosition = surplusDoubleData(dIdealSpindlePosition, pSSpindleEncoder->usEncoderCirclePulseNum);  //对理想编码器数值取余进而获得理论主轴编码器的位置
		//pG32->dIdealSpindlePosition = dIdealSpindlePosition; 
	}

	/*if (isZero(pG32->dIdealSpindlePosition))
	{
		pG32->dIdealSpindlePosition = pSSpindleEncoder->usEncoderCirclePulseNum;

	}*/

	//将下述参数赋值到螺纹插补结构体
	pG32->dRemainDistance = dRemainDistance;
	pG32->unTac = dTac;
	pG32->dVs = dVs;
	pG32->dDec = dDec;

}

///*************************************************
//  函数名称：void prepareRepeatG32(struct _SG32* pG32, struct SToolCompensation* pSToolCompPointer, double dDistFromSpindle)
//  函数功能：连续G32预处理
//  输入参数：*pG32，               结构体struct _SG32指针
//            *SToolCompensation	结构体struct SToolCompensation指针 
//			  dDistFromSpindle      double              当前主轴转速，长轴应该行走的距离
//  输出参数：无
//  返 回 值：无
//  备    注：无
//**************************************************/
void prepareRepeatG32(struct _SG32* pG32, struct SToolCompensation* pSToolCompPointer, double dDistFromSpindle)
{
	//若下一段G32的插补长度>本次G32插补随动段两倍插补时间的距离，则为正常连续G32插补（6.24）
	if (getBits(pSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT && ((pSToolCompPointer->dLength > dDistFromSpindle * 2) || pSToolCompPointer->ucG32SubMode == 2 || pSToolCompPointer->ucG32SubMode == 3))
	{
		
		bool bChangeDirFlag = FALSE;
		uint8 ucLongAxisNum = pG32->ucAxis[0];  //获取长轴轴号
		//pG32->dNextG32LongAxisSpeed = pG32->dSpindleSpeed * pSToolCompPointer->dF / (g_SInterpolation.dTc);  //下一个G32指令的长轴移动移动速度
		pG32->dNextG32LongAxisSpeed = pG32->dSpindleSpeed * pSToolCompPointer->dF;  //下一个G32指令的长轴移动移动速度

		//如果下一次G32长轴的向量发生变化，则为换向类型连续G32插补（6.24）
		if (sgn(pG32->SStartVector.dArray[ucLongAxisNum]) != sgn(pSToolCompPointer->SStartVector.dArray[ucLongAxisNum]) || pSToolCompPointer->ucG32SubMode == 2 || pSToolCompPointer->ucG32SubMode == 3)
		{
			pG32->dNextG32LongAxisSpeed = pG32->SUserData.dVe;  //对于当前G32下一个G32的长轴速度为系统允许起跳速度
			bChangeDirFlag = TRUE;
		}


		//当为减速过渡型连续G32 或换向连续G32
		if (pG32->dF > pSToolCompPointer->dF || bChangeDirFlag)
		{
			double dNowG32LongAxisSpeed;   //当前g32随动的速度
			double dNextG32LongAxisSpeed;  //下一个G32随动的速度   
			double dDec;				   //减速度为系统减速度
			double dSdec;                  //理想减速点距离终点的距离
			double dTdc;                   //理论减速时间

			dNowG32LongAxisSpeed = pG32->dSpindleSpeed * pG32->dF;  //获取当前g32随动的速度
			dNextG32LongAxisSpeed = pG32->dNextG32LongAxisSpeed;  //获取下一个G32随动的速度
			dDec = pG32->SUserData.dDec;  //减速度为系统减速度

			if(dDec <= 0)
			{
				parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
				return;
			}

			dTdc = fabs((dNextG32LongAxisSpeed - dNowG32LongAxisSpeed) / (dDec * pG32->SPlanData.dTc));
			dTdc = round(dTdc);
			if (isZero(dTdc))
			{
				dTdc = 1;

			}
			dDec = fabs(dNextG32LongAxisSpeed - dNowG32LongAxisSpeed) / (dTdc * pG32->SPlanData.dTc);

			dSdec = isZero(dNextG32LongAxisSpeed - dNowG32LongAxisSpeed)? 0 : fabs((dNextG32LongAxisSpeed * dNextG32LongAxisSpeed - dNowG32LongAxisSpeed * dNowG32LongAxisSpeed) / (2 * dDec) + dNowG32LongAxisSpeed * dTdc);  //理想减速点距离终点的距离

			pG32->bDecPlanTransitionFlag = TRUE;  //当前G32为非加速过渡类型
			pG32->dIdealTransmitStartPos = pG32->SEndPoint.dArray[ucLongAxisNum] - dSdec * pG32->SStartVector.dArray[ucLongAxisNum];  //获得长轴理想过渡起点位置

		}

	}
			
	
		
}

//void prepareRepeatG32(struct _SG32* pG32, struct SToolCompensation* pSToolCompPointer, double dDistFromSpindle)
//{
//	//若下一段G32的插补长度>本次G32插补随动段两倍插补时间的距离，则为正常连续G32插补
//	if (getBits(pSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT && (pSToolCompPointer->dLength > dDistFromSpindle * 2))
//	{
//		if (pSToolCompPointer->ucG32SubMode == 0)
//		{
//			bool bChangeDirFlag = FALSE;
//			uint8 ucLongAxisNum = pG32->ucAxis[0];  //获取长轴轴号
//			//pG32->dNextG32LongAxisSpeed = pG32->dSpindleSpeed * pSToolCompPointer->dF / (g_SInterpolation.dTc);  //下一个G32指令的长轴移动移动速度
//			pG32->dNextG32LongAxisSpeed = pG32->dSpindleSpeed * pSToolCompPointer->dF;  //下一个G32指令的长轴移动移动速度
//
//			//如果下一次G32长轴的向量发生变化，则为换向类型连续G32插补
//			if (sgn(pG32->SStartVector.dArray[ucLongAxisNum]) != sgn(pSToolCompPointer->SStartVector.dArray[ucLongAxisNum]))
//			{
//				pG32->dNextG32LongAxisSpeed = pG32->SUserData.dVe;  //对于当前G32下一个G32的长轴速度为系统允许起跳速度
//				bChangeDirFlag = TRUE;
//			}
//
//
//			//当为减速过渡型连续G32 或换向连续G32
//			if (pG32->dF > pSToolCompPointer->dF || bChangeDirFlag)
//			{
//				double dNowG32LongAxisSpeed;   //当前g32随动的速度
//				double dNextG32LongAxisSpeed;  //下一个G32随动的速度   
//				double dDec;				   //减速度为系统减速度
//				double dSdec;                  //理想减速点距离终点的距离
//				double dTdc;                   //理论减速时间
//
//				dNowG32LongAxisSpeed = pG32->dSpindleSpeed * pG32->dF;  //获取当前g32随动的速度
//				dNextG32LongAxisSpeed = pG32->dNextG32LongAxisSpeed;  //获取下一个G32随动的速度
//				dDec = pG32->SUserData.dDec;  //减速度为系统减速度
//
//				dTdc = fabs((dNextG32LongAxisSpeed - dNowG32LongAxisSpeed) / (dDec * pG32->SPlanData.dTc));
//				dTdc = round(dTdc);
//				if (isZero(dTdc))
//				{
//					dTdc = 1;
//
//				}
//				dDec = (dNextG32LongAxisSpeed - dNowG32LongAxisSpeed) / (dTdc * pG32->SPlanData.dTc);
//
//				dSdec = fabs((dNextG32LongAxisSpeed * dNextG32LongAxisSpeed - dNowG32LongAxisSpeed * dNowG32LongAxisSpeed) / (2 * dDec) + dNowG32LongAxisSpeed * dTdc);  //理想减速点距离终点的距离
//
//				pG32->bDecPlanTransitionFlag = TRUE;  //当前G32为非加速过渡类型
//				pG32->dIdealTransmitStartPos = pG32->SEndPoint.dArray[ucLongAxisNum] - dSdec * pG32->SStartVector.dArray[ucLongAxisNum];  //获得长轴理想过渡起点位置
//
//			}
//
//		}
//
//	}
//
//
//
//}

///*************************************************
//  函数名称：void runAndCompForIFM(struct _SG32* pG32, double* dDistFromSpindle)
//  函数功能：主轴不完全跟随模式下，过渡阶段的运动与补偿
//  输入参数：*pG32，   结构体struct _SG32指针
//			dDistFromSpindle     double *  当前主轴转速，长轴应该行走的距离
//  输出参数：*pG32，   结构体struct _SG32指针
//			dDistFromSpindle     double *  当前主轴转速，长轴应该行走的距离
//  返 回 值：无
//  备    注：无
//**************************************************/
void runAndCompForIFM(struct _SG32* pG32, double* dDistFromSpindle)
{
	pG32->unCurrentT += 1;  //运行时间增加

	if (pG32->pSChannel->pSSpindleEncoder == NULL || pG32->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum <= 0)//避免编码器线数为0
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return;
	}

	//若当前插补时间节点<=过渡阶段总的插补节点
	if (pG32->unCurrentT <= pG32->unTac)
	{
		double dDisNow;                             //本周期的插补距离
		double dDisBefore;                          //上一周期的插补距离
		double dVs = pG32->dVs;                     //起始速度
		uint32 unK = pG32->unCurrentT;              //插补时间点
		double dTc = pG32->SPlanData.dTc;           //插补周期

		//double dAcc = pG32->SUserData.dAcc;  //T型加速度
		double dAcc = pG32->dAcc;    //避空距离所计算的加速度
		double dSpindleSpeed;                         //主轴平均转速（pulse / ms）
		double dLongAxisDelta;                        //本周期长轴均摊量
		struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;


		dDisNow = dVs * unK * dTc + dAcc * unK * dTc * unK * dTc / 2;
		dDisBefore = dVs * (unK - 1) * dTc + dAcc * (unK - 1) * dTc * (unK - 1) * dTc / 2;

		//dSpindleSpeed = g_dSpindleRealSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;
		dSpindleSpeed = fabs(pG32->pSChannel->dSpindleRealSpeed) * pSSpindleEncoder->usEncoderCirclePulseNum;
		dLongAxisDelta = (pSSpindleEncoder->usDelta - dSpindleSpeed * dTc + (pSSpindleEncoder->dEncoderNumDeviation / pG32->unTac)) * pG32->dF / pSSpindleEncoder->usEncoderCirclePulseNum;

		*dDistFromSpindle = dDisNow - dDisBefore + dLongAxisDelta;  //更新当前主轴转速下，长轴应该新增的距离

	}
	else //过渡阶段运动完成
	{
		pG32->ucG32Step = G32_RUNNING;  //运动部分
		pG32->unTac = 0;                //初始化过渡阶段总的插补节点
		pG32->unCurrentT = 0;		    //初始化过渡阶段的插补节点
		pG32->bAccPhaseFlag = FALSE;    //加速阶段结束


	}

	////过渡阶段运动完成
	//if (pG32->unCurrentT == pG32->unTac || pG32->unTac == 0)
	//{
	//	pG32->ucG32Step = G32_RUNNING;     //运动部分
	//	pG32->unTac = 0;         //初始化过渡阶段总的插补节点
	//	pG32->unCurrentT = 0;    //初始化过渡阶段的插补节点

	//}

}


///*************************************************
//  函数名称：void runAndCompForG32RepeatDec(struct _SG32* pG32, double* dDistFromSpindle)
//  函数功能：减速规划类型连续G32长轴的运动与补偿
//  输入参数：*pG32，   结构体struct _SG32指针
//			dDistFromSpindle     double *  当前主轴转速，长轴应该行走的距离
//  输出参数：*pG32，   结构体struct _SG32指针
//			dDistFromSpindle     double *  当前主轴转速，长轴应该行走的距离
//  返 回 值：无
//  备    注：无
//**************************************************/
void runAndCompForG32RepeatDec(struct _SG32* pG32, double* dDistFromSpindle)
{
	pG32->unCurrentT += 1;  //插补时间节点增加

	if (pG32->pSChannel->pSSpindleEncoder == NULL || pG32->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum <= 0)//避免编码器线数为0
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return;
	}

	if (pG32->unCurrentT <= pG32->unTac)
	{
		double dDisNow;                                        //本周期的插补距离
		double dDisBefore;							           //上一周期的插补距离
		double dVs = pG32->dVs;                             //起始速度
		uint32 unK = pG32->unCurrentT;                      //插补时间点
		double dTc = pG32->SPlanData.dTc;                   //插补周期
		double dDec = pG32->dDec;                            //速度规划获得的减速度
		double dDelta = pG32->dRemainDistance / pG32->unTac;   //当前周期的均摊距离

		dDisNow = dVs * unK * dTc + dDec * unK * dTc * unK * dTc / 2;
		dDisBefore = dVs * (unK - 1) * dTc + dDec * (unK - 1) * dTc * (unK - 1) * dTc / 2;

		//*dDistFromSpindle = dDisNow - dDisBefore + dDelta; //更新当前主轴转速下，长轴应该新增的距离
		if (isZero(pG32->dNextG32LongAxisSpeed - pG32->SUserData.dVe))
		{
			*dDistFromSpindle = dDisNow - dDisBefore - dDelta; //更新当前主轴转速下，长轴应该新增的距离
		}
		else
		{
			*dDistFromSpindle = dDisNow - dDisBefore;
			double b = dDisNow - dDisBefore;
			int a = 0;
		}
		//*dDistFromSpindle = dDisNow - dDisBefore;
		//*dDistFromSpindle = dDisNow - dDisBefore; //更新当前主轴转速下，长轴应该新增的距离
		

	}

	//过渡阶段运动完成
	if (pG32->unCurrentT == pG32->unTac || pG32->unTac == 0)
	{
		double dSpindlePulseDelta;   //主轴脉冲偏差
		double dSpindleSpeed;        //主轴转速脉冲形式
		double dLongAxisDelta;	     //长轴均摊量
		double dRealSpindlePosition; //终点处实际主轴位置
		struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
		int	nEncoderValue = pSSpindleEncoder->lEncoderAbsValue % pSSpindleEncoder->usEncoderCirclePulseNum; //过渡时编码器所在的位置
		dSpindleSpeed = pG32->dSpindleSpeed * pSSpindleEncoder->usEncoderCirclePulseNum;
		double dEncoderDeviation;    //终点处编码器位置偏差
		struct SToolCompensation* pSToolCompPointer = NULL;   //刀补后产生的结构体指针，用于读取下一个指令类型
		double dNextG32F;            //下一个G32的导程值 
		double dDeltaPos;            //理想主轴与实际主轴的位置偏差
		double dCompDir;             //补偿方向

		//主轴正转
		if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)
		{
			dRealSpindlePosition = surplusDoubleData(nEncoderValue + pSSpindleEncoder->usDelta, pSSpindleEncoder->usEncoderCirclePulseNum);
			dDeltaPos = fabs(dRealSpindlePosition - pG32->dIdealSpindlePosition);
			dDeltaPos = dDeltaPos < (pSSpindleEncoder->usEncoderCirclePulseNum - dDeltaPos) ? dDeltaPos : (pSSpindleEncoder->usEncoderCirclePulseNum - dDeltaPos);
			if (isZero(surplusDoubleData(pG32->dIdealSpindlePosition + dDeltaPos, pSSpindleEncoder->usEncoderCirclePulseNum) - dRealSpindlePosition))
			{
				dCompDir = 1;
			}
			else
			{
				dCompDir = -1;

			}
			dEncoderDeviation = dCompDir * dDeltaPos;
		}
		else  //主轴反转
		{
			dRealSpindlePosition = surplusDoubleData(nEncoderValue - pSSpindleEncoder->usDelta, pSSpindleEncoder->usEncoderCirclePulseNum);
			dDeltaPos = fabs(dRealSpindlePosition - pG32->dIdealSpindlePosition);
			dDeltaPos = dDeltaPos < (pSSpindleEncoder->usEncoderCirclePulseNum - dDeltaPos) ? dDeltaPos : (pSSpindleEncoder->usEncoderCirclePulseNum - dDeltaPos);
			if (isZero(surplusDoubleData(pG32->dIdealSpindlePosition - dDeltaPos, pSSpindleEncoder->usEncoderCirclePulseNum) - dRealSpindlePosition))
			{
				dCompDir = 1;
			}
			else
			{
				dCompDir = -1;
			}
			dEncoderDeviation = dCompDir * dDeltaPos;
		}
		 
		pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum); //获取"速度规划读"指针
		dNextG32F = pSToolCompPointer->dF;

		dLongAxisDelta = dEncoderDeviation * dNextG32F / pSSpindleEncoder->usEncoderCirclePulseNum;
		pG32->dStartPointCompensate += dLongAxisDelta;  //下一次G32起点位置的补偿

		pG32->ucG32Step = G32_RUNNING;  //运动部分
	}

}


///*************************************************
//  函数名称：int8 motionForG32(struct _SG32* pG32,  double* dLongAxisDelta)
//  函数功能：G32运动坐标点的计算
//  输入参数：*pG32，   结构体struct _SG32指针
//			*dDistFromSpindle     double *  当前主轴转速，长轴应该行走的距离
//  输出参数：无
//  返 回 值：无
//  备    注：无
//**************************************************/
int8 motionForG32(struct _SG32* pG32, double* dLongAxisDelta, double dDistFromSpindle, double dVectorFromSpindle, double dAcc, double dDec)
{
	double dDeltaLongAxisDis;   //长轴运动的长度
	double dDeltaL;				//相对起点的距离 
	SPointDouble STargetPoint;  //当前点的理论绝对坐标
	SPointDouble SStarPoint;
	uint8 ucLongAxisNum  = pG32->ucAxis[0];  //获取长轴轴号
	uint8 ucShortAxisNum = pG32->ucAxis[1];  //获取短轴轴号
	double dDeltaLForShortAxis;				 //变锥度连续G32，短轴相对起点的距离 
	
	struct _SRun* pSRun;//运行结构体
	pSRun = &pG32->pSChannel->SRun;//运行结构体

	//运行时间增加
	pG32->SPlanData.unCurrentT += 1;

	//获取currentT时刻的线段长（直线）
	dDeltaL = nextstepPlanningAndDistReAvg(pG32->SPlanData.unCurrentT, &(pG32->SPlanData));

	//计算长轴理论坐标点 
	calculateExactPointForG32((double*)&STargetPoint, dDeltaL, pG32);

	if (pG32->bLeftTailFlag)  //退尾预处理是否完成
	{
		SStarPoint = pG32->SBackTailPoint;  //完成则有退尾，下方的坐标计算以退尾点作为起点
	}
	else
	{
		SStarPoint = pG32->SStarCutPoint;   //无退尾，则以切削起点作为坐标计算的起点
	}

	//为变锥度连续G32
	if (pG32->bRepeatG32Flag && !isZero(pG32->dRatioXZ - pG32->dLastRatioXZ) && !pG32->bExcessPlanFlag)
	{
		double dLongAxisOverLength;	//长轴当前位置超过插补起点的距离
		pG32->bExcessPlanFlag = TRUE;  //需要对短轴进行速度规划
		pG32->SExcessPlanForG32 = malloc(sizeof(struct SPlan));  //申请存储短轴速度规划参数的空间
		
		//申请空间成功
		if (pG32->SExcessPlanForG32 != NULL)
		{
			uint8 ucLongAxisNum = pG32->ucAxis[0];   //获取长轴轴号	
			uint8 ucShortAxisNum = pG32->ucAxis[1];  //获取短轴轴号
			SPointDouble SRealTimePoint;             //实时点
			double dShortAxisOverLength;
			double dRatioXZ = fabs(pG32->dRatioXZ);
			//获取长轴当前位置超过插补起点的距离
			dLongAxisOverLength  = fabs(pG32->SStarCutPoint.dArray[ucLongAxisNum] - pG32->SLastEndPoint.dArray[ucLongAxisNum]);  
			dShortAxisOverLength = fabs(pG32->SStarCutPoint.dArray[ucShortAxisNum] - pG32->SLastEndPoint.dArray[ucShortAxisNum]);
			getRealTimePoint((double*)&SRealTimePoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, &pG32->ucValStatus);  //获取实时点

			//初始化短轴速度规划参数
			pG32->SExcessPlanForG32->dTc = pG32->SPlanData.dTc;  //获取短轴插补周期
			pG32->SExcessPlanForG32->dDistance = pG32->SEndPoint.dArray[ucShortAxisNum] - SRealTimePoint.dArray[ucShortAxisNum];   //获取短轴插补距离
			//pG32->SExcessPlanForG32->dTargetDist  = dRealOverValue * (pG32->dRatioXZ - pG32->dLastRatioXZ) + pG32->dStartPointCompensateCopy * pG32->dRatioXZ;  //获取短轴起始位置补偿量
			//pG32->SExcessPlanForG32->dCurLeftDist = dRealOverValue * (pG32->dRatioXZ - pG32->dLastRatioXZ) + pG32->dStartPointCompensateCopy * pG32->dRatioXZ;
			pG32->SExcessPlanForG32->dTargetDist  = dLongAxisOverLength * dRatioXZ - dShortAxisOverLength + pG32->dStartPointCompensateCopy * dRatioXZ;  //获取短轴起始位置补偿量
			pG32->SExcessPlanForG32->dCurLeftDist = dLongAxisOverLength * dRatioXZ - dShortAxisOverLength + pG32->dStartPointCompensateCopy * dRatioXZ;
		
			pG32->SExcessPlanForG32->dPreDist = 0;    //已插补的距离初始化
			pG32->SExcessPlanForG32->unCurrentT = 0;  //插补时间节点初始化
			pG32->dShortAxisCurrentSpeed = pG32->dCurrentSpeed * fabs(pG32->dLastRatioXZ);   //获取短轴当前的速度
			pG32->dStartPointCompensateCopy = 0;  //起始补充量（副本）初始化
		}
		else
		{
			pG32->bExcessPlanFlag = FALSE;
			return -1;  //申请空间失败，运动报错
		}
		
		
	}

	//普通螺纹，根据长短轴的比例计算当前点的理论短轴绝对坐标
	if (isZero(pG32->dRadius))
	{
		
		dDeltaLongAxisDis = fabs(STargetPoint.dArray[ucLongAxisNum] - SStarPoint.dArray[ucLongAxisNum]);  //获取长轴运动的增量

		//不需要进行额外的短轴速度规划时，计算理论短轴绝对坐标
		if (!pG32->bExcessPlanFlag || isZero(pG32->dRatioXZ))
		{
			//一般情况
			if (isZero(pG32->dShortAxisPos))
			{

				//STargetPoint.dArray[ucShortAxisNum] = SStarPoint.dArray[ucShortAxisNum] + (dDeltaLongAxisDis + pG32->dShortAxisCompensate) * pG32->dRatioXZ;
				STargetPoint.dArray[ucShortAxisNum] = SStarPoint.dArray[ucShortAxisNum] + (dDeltaLongAxisDis + pG32->dShortAxisCompensate) * pG32->dRatioXZ;
				//STargetPoint.dArray[ucShortAxisNum] = pG32->SLastEndPoint.dArray[ucShortAxisNum] + (dDeltaLongAxisDis) * pG32->dRatioXZ;

			}
			else  //以变锥度代替退尾时，出现距离太短，短轴还未跟随上长轴，短轴就运动结束的情况；这种情况记录短轴运动结束的位置即可，不需要根据比例重算短轴的位置
			{
				STargetPoint.dArray[ucShortAxisNum] = pG32->dShortAxisPos;  
			}
		}
		else  //需要进行额外的短轴速度规划
		{
			if (pG32->SExcessPlanForG32 != NULL)
			{
				uint8 ucMotionState;                                               //变锥度连续G32，短轴速度规划的状态
				double dNewAddDistance = dDistFromSpindle * fabs(pG32->dRatioXZ);  //短轴当前周期新增的距离
				double dVe = pG32->dCurrentSpeed * fabs(pG32->dRatioXZ);           //短轴规划所要达到的最终速度
				double dShortAxisDelta = 0;  //短轴当前位置距离终点的长度
				
				ucMotionState = newAddDistace(pG32, pG32->SExcessPlanForG32, dNewAddDistance, dAcc, dDec, dVe, pG32->dShortAxisCurrentSpeed);  //对短轴进行速度规划
				pG32->SExcessPlanForG32->unCurrentT += 1;  //短轴运行时间增加

				dDeltaLForShortAxis = nextstepPlanningAndDistReAvg(pG32->SExcessPlanForG32->unCurrentT, pG32->SExcessPlanForG32);     //获得变锥度连续G32，短轴相对起点的距离 
				STargetPoint.dArray[ucShortAxisNum] = SStarPoint.dArray[ucShortAxisNum] + dDeltaLForShortAxis * sgn(pG32->dRatioXZ);  //计算理论短轴绝对坐标
				dShortAxisDelta = (STargetPoint.dArray[ucShortAxisNum] - pG32->SEndPoint.dArray[ucShortAxisNum]) * sgn(pG32->dRatioXZ);

				//情况1：当长轴短轴都进入了随动阶段,短轴速度规划结束; 
				//情况2：或短轴插补距离过短，不足以跟上长轴时，短轴插补结束，速度规划结束
				if ((ucMotionState == 1 && pG32->ucMotionState == 1) || (dShortAxisDelta > 0 || isZero(dShortAxisDelta)))
				{
				
					if (dShortAxisDelta > 0 || isZero(dShortAxisDelta))  //如果是情况2
					{
						pG32->dShortAxisPos = STargetPoint.dArray[ucShortAxisNum];  //记录短轴运动结束后的位置
					}
					pG32->dShortAxisCompensate = dDeltaLForShortAxis / fabs(pG32->dRatioXZ) - dDeltaLongAxisDis;  //获取短轴补偿量
					pG32->dLastRatioXZ = pG32->dRatioXZ;  //重算下一次G32的短轴比例
					pG32->bExcessPlanFlag = FALSE;        //不再对短轴进行速度规划
					free(pG32->SExcessPlanForG32);        //释放短轴速度规划参数指针
					pG32->SExcessPlanForG32 = NULL;

				}
				
			}
				
		}	

		*dLongAxisDelta = (STargetPoint.dArray[ucLongAxisNum] - pG32->SEndPoint.dArray[ucLongAxisNum]) * pG32->SStartVector.dArray[ucLongAxisNum];  //长轴距离终点的长度
	}
	else  //圆弧面螺纹
	{
		int8 cRes = calcArcG32ShortAxisPos(pG32, STargetPoint.dArray[ucLongAxisNum],&STargetPoint.dArray[ucShortAxisNum]);  //计算长轴当前位置下，短轴的绝对位置
		if(cRes == -1)return -1;
		*dLongAxisDelta = (STargetPoint.dArray[ucLongAxisNum] - pG32->SEndPoint.dArray[ucLongAxisNum]) * pG32->SStartVector.dArray[ucLongAxisNum];  //长轴距离终点的长度
	}

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机 
	if (pG32->pSChannel->bUseVirtualY == TRUE)
	{
		//将直角坐标系下的坐标点转化为倾斜坐标系下的点
		if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_X))
		{
			STargetPoint.dArray[AXIS_X] += 1.0 * STargetPoint.dArray[AXIS_Y] * pG32->pSChannel->dTANAngleYY1;
		}
		if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_Y))
		{
			STargetPoint.dArray[AXIS_Y] = STargetPoint.dArray[AXIS_Y] / pG32->pSChannel->dCOSAngleYY1;
		}
	}
#endif

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE //单表螺距补偿+ 反向间隙补偿
	//利用理论坐标生成SRun结构体数据
	outPutAbsPointWithScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, pG32->dPulseToMm, &pG32->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP //双表螺距补偿
	outPutAbsPointWithBilateralScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, pG32->dPulseToMm, &pG32->ucValStatus);
#else
	//利用理论坐标生成SRun结构体数据
	outPutAbsPoint(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, pG32->dPulseToMm, &pG32->ucValStatus); //利用理论坐标生成SRun结构体数据（未完成）
#endif

	//如果当前留下未移动的距离大于等于0
	if (pG32->SPlanData.dCurLeftDist > 0 || isZero(pG32->SPlanData.dCurLeftDist))
	{
		//计算当前理论速度
		pG32->dCurrentSpeed = fabs(pG32->SPlanData.dCurLeftDist - (pG32->SPlanData.dTargetDist - dDeltaL)) / pG32->SPlanData.dTc;
		pG32->SPlanData.dCurLeftDist = pG32->SPlanData.dTargetDist - dDeltaL;  //计算理论未走距离
	}
	else
	{
		pG32->dCurrentSpeed = 0;  //当前速度为0
	}

	//如果需要对短轴进行额外的速度规划
	if (pG32->SExcessPlanForG32 != NULL && pG32->bExcessPlanFlag)
	{
		if (pG32->SExcessPlanForG32->dCurLeftDist >= 0)
		{
			//计算短轴当前理论速度
			pG32->dShortAxisCurrentSpeed = fabs(pG32->SExcessPlanForG32->dCurLeftDist - (pG32->SExcessPlanForG32->dTargetDist - dDeltaLForShortAxis))
				/ pG32->SExcessPlanForG32->dTc;
			pG32->SExcessPlanForG32->dCurLeftDist = pG32->SExcessPlanForG32->dTargetDist - dDeltaLForShortAxis;

		}
		else
		{
			pG32->dShortAxisCurrentSpeed = 0;  //短轴当前速度为0
		}

	}

	return 1;
	

}

///*************************************************
//  函数名称：void prepareG32RepeatDecPlanning(struct _SG32* pG32, double* dLongAxisDelta)
//  函数功能：减速类型连续G32过渡规划的前置准备
//  输入参数：*pG32，   结构体struct _SG32指针
//			dDistFromSpindle     double *  当前主轴转速，长轴应该行走的距离
//  输出参数：无
//  返 回 值：无
//  备    注：无
//**************************************************/
void prepareG32RepeatDecPlanning(struct _SG32* pG32, double* dLongAxisDelta)
{
	double dDelta;  //长轴距离理想减速点的实际长度
	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
	struct _SRun* pSRun;//运行结构体
	pSRun = &pG32->pSChannel->SRun;//运行结构体

	dDelta = (pSRun->SAbsCoord.dArray[ucLongAxisNum] - pG32->dIdealTransmitStartPos) * pG32->SStartVector.dArray[ucLongAxisNum];
	if (!(dDelta < 0))  //若当前位置大于等于理想减速点的位置
	{
		struct SToolCompensation* pSToolCompPointer = NULL;                //刀补后产生的结构体指针，用于读取下一个指令类型
		double dNextG32F;  //下一个G32的导程值
		pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum); //获取"速度规划读"
		dNextG32F = pSToolCompPointer->dF;                                 //获取下一个G32的导程值

		planDecForRepeatG32(pG32, fabs(*dLongAxisDelta));                        //减速规划
		//pG32->dStartPointCompensate += dDelta * (dNextG32F / pG32->dF);//超过理想减速点的值均摊至下一段G32
		//pG32->dStartPointCompensate += pG32->dRemainDistance;
		pG32->ucG32Step = G32_REPEATSTATE_COMPENSATE;   //G32运行步骤切换为4（减速规划类型连续G32长轴的运动与补偿）
		pG32->bStartCompensate = FALSE;   //开始补偿标志位置位置为FALSE


	}

}

///*************************************************
//  函数名称：void compensateForAccRepeatG32(struct _SG32* pG32)
//  函数功能：加速类型连续G32的补偿量计算
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void compensateForAccRepeatG32(struct _SG32* pG32)
{
	struct SToolCompensation* pSToolCompPointer = NULL;                //刀补后产生的结构体指针，用于读取下一个指令类型
	double dNextG32F;       //下一个G32的导程值
	double dRealOverValue;  //长轴实际超过终点量
	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
	struct _SRun* pSRun;//运行结构体
	pSRun = &pG32->pSChannel->SRun;//运行结构体

	pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum); //获取"速度规划读"
	dNextG32F = pSToolCompPointer->dF;   //获取下一个G32的导程值
	dRealOverValue = (pSRun->SAbsCoord.dArray[ucLongAxisNum] - pG32->SEndPoint.dArray[ucLongAxisNum]) * pG32->SStartVector.dArray[ucLongAxisNum];

	//if (pG32->ucLongAxisNum == AXIS_Z)  //长轴为Z轴
	//{
	//	dRealOverValue = (g_SRun.SAbsCoord.dZ - pG32->SEndPoint.dZ) * pG32->SStartVector.dIZ;  //获取长轴距离终点的长度

	//}
	//else  //长轴为X轴
	//{
	//	dRealOverValue = (g_SRun.SAbsCoord.dX - pG32->SEndPoint.dX) * pG32->SStartVector.dIX;  //获取长轴距离终点的长度

	//}

	pG32->dStartPointCompensate += dRealOverValue * (dNextG32F / pG32->dF - 1);//超过理想减速点的值均摊至下一段G32	
	pG32->bStartCompensate = FALSE;  //开始补偿标志位置为FALSE

}

//*********************************测试用全局变量********************************
int g_nTestNum = 0;
extern double g_dTime;
double test_dG32Time;
double test_dTargetPos;
//*********************************测试用全局变量********************************
//
///*************************************************
//  函数名称：void prepareForMoveAndInit(struct _SG32* pG32)
//  函数功能：初始化和运动准备
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void prepareForMoveAndInit(struct _SG32* pG32)
{
	initThreadPreStructData(pG32);            //根据刀补后的结构体参数初始化螺纹类指令参数结构体
	
#if !ENCODER_MODE
	g_SSystemPara.ThreadCutMode = INCOMPLETE_FOLLOW_MODE;  //设当前模式为“主轴不完全跟随”模式（测试用)。

#endif

	pG32->pSChannel->bCalculateSpindleSpeedFlag = TRUE;   //计算主轴实际转速标志位

	pG32->ucG32Step = G32_JUAGE_REPEATSTATE_AND_GET_REALCUTANGLE;    //设置G32运行步骤为“连续G32的判断以及获取真正切入角”
	//pG32->SPlanData.dTargetDist  = 0;  //插补结构体目标插补距离和未插补距离初始化
	//pG32->SPlanData.dCurLeftDist = 0;
	pG32->ucInpStatus = INP_RUNNING;  //插补结构体的状态为“运行”

	pG32->dSpindleSpeed = getG97AbsSpindleSpeed(&pG32->pSChannel->SSpindle);  //获得不同主轴倍率下的转速（运动后不会改变该数值）


	//当前指令为连续G32指令
	if (pG32->bRepeatG32Flag)
	{
		//直接进行运动，不用获取同步信号,也不需要计算主轴实际转速
		pG32->bSyncSignal = TRUE;
		pG32->pSChannel->bCalculateSpindleSpeedFlag = FALSE;

	}

	//*********************************文本输出第N个G32起的数据********************************
	if (test_dG32Time >= 0)
	{
		pG32->bPrintFlag = TRUE;
	}

	test_dG32Time++;
	//*********************************文本输出第N个G32起的数据********************************


}

///*************************************************
//  函数名称：void judgeRepeatG32AndGetRealCutAngle(struct _SG32* pG32, double dDistFromSpindle)
//  函数功能：连续G32的判断以及获取真正切入角
//  输入参数：*pG32，   结构体struct _SG32指针
//			dDistFromSpindle     double    当前主轴转速，长轴应该行走的距离
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void judgeRepeatG32AndGetRealCutAngle(struct _SG32* pG32, double dDistFromSpindle)
{
	struct SToolCompensation* pSToolCompPointer = NULL;                //刀补后产生的结构体指针，用于读取下一个指令类型
	pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum); //获取"速度规划读"

	//若"速度规划读"指针有内容可读
	if (getSToolCompPointerBufRes(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum) > 0)
	{
		prepareRepeatG32(pG32, pSToolCompPointer, dDistFromSpindle);  //连续G32预处理

	}

	pG32->bAccPhaseFlag = FALSE;  //加速阶段标志位初始化
	//不为连续G32指令
	if (!pG32->bRepeatG32Flag)
	{
		//运行模式为：主轴不完全跟随模式
		if (g_SSystemPara.ThreadCutMode == INCOMPLETE_FOLLOW_MODE && pG32->ucG32SubMode != 4)	//当不为G32.4且为主轴不完全跟随模式才重新计算切入角	陈敏聪 2023/6/15
		{
			pG32->dQ = getRealTargetPulse(pG32); //计算真正的起始切入角
			pG32->bAccPhaseFlag = TRUE;

		}

		pG32->ucG32Step = G32_GET_SYNCSIGN; //判断是否获取同步信号

	}
	else
	{
		pG32->ucG32Step = G32_RUNNING;      //运行

	}

}

///*************************************************
//  函数名称：void judgeSyncSignalAndCompensate(struct _SG32* pG32)
//  函数功能：判断是否获得同步信号和补偿启动偏差
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针		
//  返 回 值：无
//  备    注：无
//**************************************************/
void judgeSyncSignalAndCompensate(struct _SG32* pG32)
{
	double dPulseQ;  //起始切入角的脉冲表示形式
	struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
	if (g_SSystemPara.ThreadCutMode == INCOMPLETE_FOLLOW_MODE)  //主轴不完全跟随模式
	{
		dPulseQ = pG32->dQ;  //计算真正的起始切入角的步骤，以将其转换为脉冲表示形式

	}
	else   //主轴完全跟随模式
	{
		dPulseQ = (pG32->dQ / 360000.0) * pSSpindleEncoder->usEncoderCirclePulseNum;  //转换为脉冲表示形式
	}

	//dPulseQ += getMoveLagComp(pG32->pSChannel, pG32->dF, pG32->ucAxis[0], nLongAxisLogValue);
	//dPulseQ = surplusDoubleData(dPulseQ, pSSpindleEncoder->usEncoderCirclePulseNum);
	pG32->bSyncSignal = isGetSyncSignal(dPulseQ, pG32->pSChannel, pSSpindleEncoder);  //判断是否获得同步信号，并在获取同步信号时记录启动偏差

	//获得同步信号
	if (pG32->bSyncSignal)
	{
		if (pG32->ucG32SubMode == 4)	//如果是G32.4，获取到同步信号后就结束运行	陈敏聪	2023/6/15
		{
			pG32->ucInpStatus = INP_FINISHED;           //状态改变为FINISHED
			pG32->pSChannel->SPreProcess.ucInpStatus = INP_FINISHED;
			pG32->ucG32Step = G32_OCCUPATION;           //运行步骤初始化
			g_dTime = 0;                                //G32运行次数（测试）

			if (pG32->SExcessPlanForG32 != NULL)
			{
				free(pG32->SExcessPlanForG32);        //释放短轴速度规划参数指针
				pG32->SExcessPlanForG32 = NULL;
			}
		}
		else
		{
			if (g_SSystemPara.ThreadCutMode == INCOMPLETE_FOLLOW_MODE)  //主轴不完全跟随模式
			{
				pG32->ucG32Step = G32_IFM_COMPENSATE;  //主轴不完全跟随模式下，过渡阶段的运动与补偿
			}
			else  //主轴完全跟随模式
			{

				//如果为主轴完全跟随模式，则直接将启动偏差赋值给“目标距离”和“当前留下未移动距离”
				pG32->SPlanData.dCurLeftDist = pSSpindleEncoder->dEncoderNumDeviation * pG32->dF / pSSpindleEncoder->usEncoderCirclePulseNum;  //主轴编码器位置的偏差
				pG32->SPlanData.dTargetDist = pG32->SPlanData.dCurLeftDist;
				pG32->ucG32Step = G32_RUNNING;  //运动部分

			}
		}

	}

}

///*************************************************
//  函数名称：uint8 loadSG32DataFromInput(struct SToolCompensation* pSToolCompPointer, struct SInterpolationPre* pSPrePointer, struct _SG32* pG32, struct SInputG32* pSInputG32)
//  函数功能：载入输入的参数至G32结构体
//  输入参数：*pG32，              结构体struct _SG32指针
//            *pSToolCompPointer   结构体struct SToolCompensation指针
//			*pSPrePointer        结构体struct SInterpolationPre指针
//			*pSInputG32          结构体struct SInputG32 指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
uint8 loadSG32DataFromInput(struct SToolCompensation* pSToolCompPointer, struct SInput* pSInput, struct _SG32* pG32, struct SInputG32* pSInputG32)
{
	bool bAssignBySys = FALSE; //是否根据系统参数初始化G32

	if (pSToolCompPointer == NULL || pSInput == NULL)//空指针  不运行
	{
		return -1;
	}

	pG32->ucValStatus = pSInput->ucValStatus;     //有效轴
	for (int i = 0; i < 2; i++)
	{
		setbit(pG32->ucValStatus, pG32->SUserData.ucAxis[i]);
	}
	pG32->SUserData.ucRelaVal = pSInput->ucRelaVal;  //相对值标识变量

	if (pSInputG32 == NULL)
	{
		pG32->bSysDataInitFlag = TRUE;
		

	}

	if (pG32->bSysDataInitFlag)  //根据系统参数初始化G32
	{
		//螺纹参数初始化
		pG32->SUserData.dJ = pSToolCompPointer->dJ;                //短轴退尾量
		pG32->SUserData.dK = pSToolCompPointer->dK;                //长轴退尾量
		pG32->SUserData.dQ = pSToolCompPointer->dQ;                //起始角
		pG32->SUserData.dF = pSToolCompPointer->dF;                //长轴螺纹导程
		pG32->SUserData.SEndPoint = pSToolCompPointer->SEndPoint;                //螺纹切削终点
		pG32->SUserData.SStarCutPoint = pSToolCompPointer->SStartPoint;              //螺纹切削理论起点

		//getRealTimePoint(&pG32->SUserData.SStarCutPoint, AXIS_NUM_MAX, g_ucAxisMapArray[0], &pG32->ucValStatus);  //在运行部分再修改为实时点
		//getRealTimePoint(&pG32->SUserData.SStarCutPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, &pG32->ucValStatus);
		//圆弧G32的参数
		pG32->cArcDir = pSToolCompPointer->cArcDir;
		pG32->dRadius = pSToolCompPointer->dRadius;
		pG32->SCenterPoint = pSToolCompPointer->SCenterPoint;

		//以下参数得判断长短轴后才能初始化，在函数G32Prepare()实现初始化
		//pG32->SUserData.dAcc = g_SSystemPara.dSysPlanTMaxAcc;  //前瞻后才会增加加速度，所以G32的加速度只能在系统参数或者外部输入获得
		//pG32->SUserData.dDec = g_SSystemPara.dSysPlanTMaxDec;
		//pG32->SUserData.dVs = g_SSystemPara.dSysMinVel;
		//pG32->SUserData.dVmax = g_SSystemPara.dSysMaxVel;
		//pG32->SUserData.dVe = g_SSystemPara.dSysMinVel;

		pG32->ucG32SubMode = pSToolCompPointer->ucG32SubMode;	//陈敏聪	2023/6/15增

	}
	else  //根据用户输入参数初始化G32
	{

		pG32->SUserData = *pSInputG32;

	}
	pG32->bG34RunFlag = pSToolCompPointer->bG34RunFlag;           //判断当前指令是否为G34
	pG32->bG34BackTailFlag = pSToolCompPointer->bG34BackTailFlag; //判断当前指令G34指令是否处于退尾的状态
	pG32->bRepeatG32Flag = pSToolCompPointer->bRepeatG32Flag;     //是否为连续G32
	pG32->ucG32Step = G32_OCCUPATION;                             //螺纹运行步骤
	return 1;



}

///*************************************************
//  函数名称：void processUserInputAxis(struct _SG32* pG32)
//  函数功能：处理用户输入的插补轴轴号
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
void processUserInputAxis(struct _SG32* pG32)
{
	double dCutDisFirstAxis; 	   //用户参数输入后执行插补0号轴的切削量（带矢量）
	double dCutDisSecondAxis;	   //用户参数输入后执行插补1号轴的切削量（带矢量）
	uint8 ucLongAxisNum;           //输入用户参数后长轴的轴号
	uint8 ucShortAxisNum;          //输入用户参数后短轴的轴号

	dCutDisFirstAxis = pG32->SUserData.SEndPoint.dArray[pG32->SUserData.ucAxis[0]] - pG32->SUserData.SStarCutPoint.dArray[pG32->SUserData.ucAxis[0]];  //获取执行插补0号轴的切削量
	dCutDisSecondAxis = pG32->SUserData.SEndPoint.dArray[pG32->SUserData.ucAxis[1]] - pG32->SUserData.SStarCutPoint.dArray[pG32->SUserData.ucAxis[1]];  //获取执行插补1号轴的切削量

	ucLongAxisNum = fabs(dCutDisFirstAxis) > fabs(dCutDisSecondAxis) ? pG32->SUserData.ucAxis[0] : pG32->SUserData.ucAxis[1];  //获取用户输入的长轴轴号
	ucShortAxisNum = ucLongAxisNum == pG32->SUserData.ucAxis[0] ? pG32->SUserData.ucAxis[1] : pG32->SUserData.ucAxis[0];       //获取用户输入的短轴轴号

	//判断用户输入的插补轴与修改前的插补轴轴号情况
	for (int i = 0; i < 2; i++)
	{
		if (pG32->ucAxis[i] != pG32->SUserData.ucAxis[0] && pG32->ucAxis[i] != pG32->SUserData.ucAxis[1])  //如果用户输入的插补轴轴号不包含旧的插补轴号
		{
			pG32->SUserData.SEndPoint.dArray[pG32->ucAxis[i]] = pG32->SUserData.SStarCutPoint.dArray[pG32->ucAxis[i]];  //令该旧的插补轴的终点等于当前点

		}


	}

	//若插补轴的长轴轴号发生改变
	if (pG32->SUserData.ucAxis[0] != ucLongAxisNum)
	{
		if (pG32->SUserData.ucAxis[0] == ucShortAxisNum)  //插补轴长轴轴号为原先的短轴
		{
			pG32->dCurrentSpeed = pG32->dCurrentSpeed * fabs(pG32->dRatioXZ);  //由于新的规划的长轴为原先规划的短轴，所以规划中当前速度应该更新为原先短轴的速度

		}
		else  //插补轴长轴轴号不等于原先的任意一个插补轴
		{
			pG32->dCurrentSpeed = 0;

		}

	}


}

///*************************************************
//  函数名称：void getInputParameter(struct _SG32* pG32)
//  函数功能：获取输入参数
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：*pG32，   结构体struct _SG32指针
//  返 回 值：无
//  备    注：无
//**************************************************/
int8 getInputParameter(struct _SG32* pG32)
{
	const uint8* ucMapArray = pG32->pSChannel->pMapAxisArray;//映射

	pG32->SPlanData.dTc = g_SSystemPara.dInterpolationTime;  //获取插补周期

	//判断相对坐标  修正终点
	if (pG32->SUserData.ucRelaVal != VAL_ABS_POINT)//不完全是绝对坐标
	{
		modifyRelaEndPoint((double*)&pG32->SUserData.SStarCutPoint, (double*)&pG32->SUserData.SEndPoint, ucMapArray, AXIS_NUM_MAX, &pG32->SUserData.ucRelaVal);
	}
	//获取当前坐标作为起点
	getRealTimePoint((double*)&pG32->SUserData.SStarCutPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, &pG32->ucValStatus);

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机 
	if (pG32->pSChannel->bUseVirtualY == TRUE)
	{
		if(VAL_VALID == getbit(pG32->ucValStatus, AXIS_X))
		{
			double dY1Mach = 0;
			getSingleAxisRealTimePoint(&dY1Mach, AXIS_Y, ucMapArray);//当前Y1轴的位置
			pG32->SUserData.SStarCutPoint.dArray[AXIS_X] -= 1.0 * dY1Mach * pG32->pSChannel->dSINAngleYY1;
			//pG32->SUserData.SStarCutPoint.dArray[AXIS_X] -= (round(1.0 * dY1Mach * pG32->pSChannel->dSINAngleYY1 * g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].usMmToPulse) / (double)g_SSystemPara.SAxisPara[ucMapArray[AXIS_X]].usMmToPulse);
		}
		if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_Y))
		{
			pG32->SUserData.SStarCutPoint.dArray[AXIS_Y] = pG32->SUserData.SStarCutPoint.dArray[AXIS_Y] * pG32->pSChannel->dCOSAngleYY1;
		}
	}
#endif
	//初始化插补参数
	pG32->SPlanData.dTargetDist = 0;
	pG32->SPlanData.dCurLeftDist = 0;
	pG32->SPlanData.dPreDist = 0;
	pG32->SPlanData.dDistance = 0;
	pG32->SPlanData.unCurrentT = 0;

	pG32->dJ = pG32->SUserData.dJ;	//短轴退尾量
	pG32->dK = pG32->SUserData.dK;  //长轴退尾量
	pG32->dQ = pG32->SUserData.dQ;  //起始角
	pG32->dF = pG32->SUserData.dF;  //长轴螺纹导程

	pG32->SStarCutPoint = pG32->SUserData.SStarCutPoint;  //切削起点
	pG32->SLastEndPoint = pG32->SEndPoint;                //上一次G32的终点，用于计算变锥度螺纹时，保证计算锥度的起始统一。
	pG32->SEndPoint = pG32->SUserData.SEndPoint;          //终点
	pG32->ucAxis[0] = pG32->SUserData.ucAxis[0];
	pG32->ucAxis[1] = pG32->SUserData.ucAxis[1];
	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		pG32->SStartVector.dArray[i] = 0;  //初始化G32插补向量
	}

	// //当M30写在G32下一行时，由于G32是模态量所以M30行会有G32，但由于F不是模态量，所以F为0，即F = 0时默认G32无效  //注释M30不会进入G32指令
	// if (isZero(pG32->dF))
	// {
	// 	pG32->SEndPoint = pG32->SStarCutPoint;
	// }
	return 1;
}

///*************************************************
//  函数名称：int8 calculateExactPointForG32(double* pdTargetPoint, double dDeltaL, struct _SG32* pG32)
//  函数功能：获取输入参数
//  输入参数：*pG32，   结构体struct _SG32指针
//            dDeltaL   double类型数据  运动增量（绝对值）
//  输出参数：*pdTargetPoint，   double类型指针
//  返 回 值：无
//  备    注：无
//**************************************************/
int8 calculateExactPointForG32(double* pdTargetPoint, double dDeltaL, struct _SG32* pG32)
{
	SPointDouble SStartPoint;	//起点，单位：mm

	if (pG32 == NULL)
	{
		return -1;
	}

	SStartPoint = pG32->SStarCutPoint;	//起点，单位：mm

	//计算各个轴的绝对坐标
	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		pdTargetPoint[i] = SStartPoint.dArray[i] + pG32->SStartVector.dArray[i] * dDeltaL;
	}

	return 1;

}


///*************************************************
//  函数名称：void UserInput(struct _SG32* pG32)
//  函数功能：用于输入参数（测试用）
//  输入参数：*pG32，   结构体struct _SG32指针
//  输出参数：无
//  返 回 值：无
//  备    注：无
//**************************************************/
void UserInput(struct _SG32* pG32)
{
	//用户输入参数
	pG32->SUserData.dJ = 0;              //短轴退尾量
	pG32->SUserData.dK = 0;              //长轴退尾量
	pG32->SUserData.dQ = 0;              //起始角
	pG32->SUserData.dF = 3;              //长轴螺纹导程

	pG32->SUserData.ucAxis[0] = AXIS_Z;
	pG32->SUserData.ucAxis[1] = AXIS_X;
	//pG32->SUserData.ucAxis[0] = AXIS_Y;
	//pG32->SUserData.ucAxis[1] = AXIS_Z;

	pG32->SUserData.SEndPoint.dArray[pG32->SUserData.ucAxis[0]] = 50;   //螺纹切削终点
	pG32->SUserData.SEndPoint.dArray[pG32->SUserData.ucAxis[1]] = 20;

	pG32->SUserData.dAcc = g_SSystemPara.dSysPlanTMaxAcc;   //加速度
	pG32->SUserData.dDec = g_SSystemPara.dSysPlanTMaxDec;   //减速度
	pG32->SUserData.dVs = g_SSystemPara.dSysMinVel;         //起始速度
	pG32->SUserData.dVmax = g_SSystemPara.dSysMaxVel;       //最大速度
	pG32->SUserData.dVe = g_SSystemPara.dSysMinVel;         //最小速度
	pG32->SUserData.ucUserInputState = USER_INPUT;          //用户输入状态置为用户已输入

}


int8 processUserInput(struct _SG32* pG32)
{
	//判断是否需要添加有效轴号
	for (int i = 0; i < 2; i++)
	{
		if (pG32->SUserData.ucAxis[i] != pG32->ucAxis[0] && pG32->SUserData.ucAxis[i] != pG32->ucAxis[1])
		{
			setbit(pG32->ucValStatus, pG32->SUserData.ucAxis[i]);  //若修改了插补轴，则完善有效轴号
		}

	}
	getRealTimePoint((double*)&pG32->SUserData.SStarCutPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, &pG32->ucValStatus);
	//getRealTimePoint(&pG32->SUserData.SStarCutPoint, AXIS_NUM_MAX, g_ucAxisMapArray[0], &pG32->ucValStatus);  //以当前点作为切削起点

	//******************************用户输入参数错误检查************************************************
	if (pG32->SUserData.ucAxis[0] == pG32->SUserData.ucAxis[1])  //如果用户输入的两轴轴号相同
	{
		pG32->SUserData.SEndPoint.dArray[pG32->ucAxis[0]] = pG32->SUserData.SStarCutPoint.dArray[pG32->ucAxis[0]];
		pG32->SUserData.SEndPoint.dArray[pG32->ucAxis[1]] = pG32->SUserData.SStarCutPoint.dArray[pG32->ucAxis[1]];
		//pG32->dCurrentSpeed = 0;
		return -1;  //获取参数失败
	}

	
	if (pG32->SUserData.dK < 0 || pG32->SUserData.dF < 0 || pG32->SUserData.dQ < 0)
	{
		return -1;  //获取参数失败

	}
	//******************************用户输入参数错误检查************************************************



	//处理用户输入轴
	processUserInputAxis(pG32);

	pG32->SUserData.ucUserInputState = FINSH_CHECK;   //完成用户输入参数的赋值

	if (!pG32->bSyncSignal)  //若用户输入参数时，之前的G32还未启动
	{
		pG32->SUserData.ucUserInputState = FINSH_CHECK_SYSNCSIGNAL;  //用户参数输入状态置为完成赋值，但需要计算新的同步信号

	}
	return 1;


}

//double calcArcG32ShortAxisPos(struct _SG32* pG32, double dLongAxisPos)
//{
//	//圆的方程为x^2 + y^2 + D*x + E*y + F = 0
//	// D = -2a;
//	// E = -2b;
//	// F = a^2 + b^2 -R^2  圆心坐标（a, b），半径R
//	//其中x统一为短轴的轴号， y为长轴的轴号
//	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
//	uint8 ucShortAxisNum = pG32->ucAxis[1];  //短轴轴号
//	double dA = pG32->SCenterPoint.dArray[ucShortAxisNum];
//	double dB = pG32->SCenterPoint.dArray[ucLongAxisNum];
//	double dR = pG32->dRadius;
//	double dY = dLongAxisPos;
//
//	double dD = -2 * dA;
//	double dE = -2 * dB;
//	double dF = dA * dA + dB * dB - dR * dR;
//	double dX1;
//	double dX2;
//	double dAnswer;
//
//	//圆的方程可整理为 x^2 + Dx + y^2 + E*y + F = 0  等价于 x^2 + Dx + C = 0
//	double dC = dY * dY + dE * dY + dF;
//
//	dX1 = (-dD / 2) + sqrt((dD * dD / 4) - dC);
//	dX2 = (-dD / 2) - sqrt((dD * dD / 4) - dC);
//
//
//	/*dX1 = dX1 >= 0 ? dX1 : 0;
//	dX2 = dX2 >= 0 ? dX2 : 0;*/
//
//
//	if (pG32->cArcDir == ARC_CW)  //为顺时针圆弧
//	{
//		return dX1 > dX2 ? dX2 : dX1;
//
//		/*if (dX1 > 0 && dX2 > 0)
//		{
//			return dX1 > dX2 ? dX2 : dX1;
//
//		}
//		else
//		{
//			if (dX1 > 0)
//			{
//				return dX1;
//			}
//			else
//			{
//				return dX2;
//			}
//
//		}*/
//
//
//	}
//	else
//	{
//		return dX1 > dX2 ? dX1 : dX2;
//
//	}
//
//}
///*************************************************
//  函数名称：int8 calcArcG32ShortAxisPos(struct _SG32* pG32, double dLongAxisPos, double* pShortAxisPos)
//  函数功能：计算G32圆弧插补短轴位置，增加判别式保护
//  输入参数：*pG32: G32结构体指针
//            dLongAxisPos: 长轴当前位置
//            *pShortAxisPos: 输出参数，计算得到的短轴位置
//  返回值： 1: 成功
//          -1: 失败 (判别式小于-EPS，无实数解)
//  备注：    
///**************************************************/
int8 calcArcG32ShortAxisPos(struct _SG32* pG32, double dLongAxisPos,double* pShortAxisPos)
{
	//圆的方程为x^2 + y^2 + D*x + E*y + F = 0
	// D = -2a;
	// E = -2b;
	// F = a^2 + b^2 -R^2  圆心坐标（a, b），半径R
	//其中x统一为短轴的轴号， y为长轴的轴号
	uint8 ucLongAxisNum = pG32->ucAxis[0];  //长轴轴号
	uint8 ucShortAxisNum = pG32->ucAxis[1];  //短轴轴号
	double dA = pG32->SCenterPoint.dArray[ucShortAxisNum];
	double dB = pG32->SCenterPoint.dArray[ucLongAxisNum];
	double dR = pG32->dRadius;
	double dY = dLongAxisPos;
    
	if (pG32 == NULL || pShortAxisPos == NULL)
    {
        return -1;
    }

	if (fabs(dLongAxisPos) - 37.80 < 0)
	{
		int a = 0;
	}

	double dD = -2 * dA;
	double dE = -2 * dB;
	double dF = dA * dA + dB * dB - dR * dR;
	double dX1;
	double dX2;
	double dAnswer;

	//圆的方程可整理为 x^2 + Dx + y^2 + E*y + F = 0  等价于 x^2 + Dx + C = 0
	double dC = dY * dY + dE * dY + dF;
	double disc = (dD * dD / 4.0) - dC;
	double dResult = 0;

	if (disc < 0)
    {
        parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum, GCODE_ERROR_NUM133, 9999);
        return -1;
    }
	dX1 = (-dD / 2) + sqrt(disc);
	dX2 = (-dD / 2) - sqrt(disc);

	if (pG32->cArcDir == ARC_CW)  //为顺时针圆弧
	{
		if (pG32->SStartVector.dArray[pG32->ucAxis[0]] > 0)
		{
			if (pG32->ucAxis[0] == AXIS_Z)
			{
				dResult = dX1 > dX2 ? dX1 : dX2;
			}
			else
			{
				dResult = dX1 > dX2 ? dX2 : dX1;
			}

		}
		else
		{
			if (pG32->ucAxis[0] == AXIS_Z)
			{
				dResult = dX1 > dX2 ? dX2 : dX1;
			}
			else
			{
				dResult = dX1 > dX2 ? dX1 : dX2;
			}

		}

	}
	else
	{
		if (pG32->SStartVector.dArray[pG32->ucAxis[0]] > 0)
		{
			if (pG32->ucAxis[0] == AXIS_Z)
			{
				dResult = dX1 > dX2 ? dX2 : dX1;
			}
			else
			{
				dResult = dX1 > dX2 ? dX1 : dX2;
			}

		}
		else
		{
			if (pG32->ucAxis[0] == AXIS_Z)
			{
				dResult = dX1 > dX2 ? dX1 : dX2;
			}
			else
			{
				dResult = dX1 > dX2 ? dX2 : dX1;
			}

		}

	}
	*pShortAxisPos = dResult;
    return 1;

	

}

double getMoveLagComp(struct SChannel* pSChannel, double dF, uint8 ucLongAxisNum, int nTargetLagNum)
{
	//参考值：
	//转速2500，滞后750个脉冲
	//当前测试值
	//转速200， 滞后60个脉冲

	//补偿思路：参考转速的滞后值 - 当前转速的滞后值得到滞后的偏差值，
	//将滞后的偏差值换算成主轴所需的脉冲，返回即可。
	struct SEncoder* pSSpindleEncoder = pSChannel->pSSpindleEncoder;
	int nReferenceLagNum = 750;
	int nLagDeviation = nReferenceLagNum - nTargetLagNum;  //当前转速滞后值与参考转速滞后值的偏差
	double spindlePulseComp = 0;  //主轴同步信号位置补偿值

	if (pSSpindleEncoder->cEncoderDir == INP_GO_AHEAD)  //若主轴正转
	{
		spindlePulseComp = (nLagDeviation * g_SSystemPara.SAxisPara[ucLongAxisNum].dPulseToMm) / dF
			* pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum;
	}
	else  //若主轴反转
	{
		spindlePulseComp = -(nLagDeviation * g_SSystemPara.SAxisPara[ucLongAxisNum].dPulseToMm) / dF
			* pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum;
	}

	return spindlePulseComp;

}

int8 calculateExactPointForG32BackTail(double* pdTargetPoint, double dLongAxisDeltal, double dShortAxisDeltal, struct _SG32* pG32)
{
	SPointDouble SStartPoint;	//起点，单位：mm
	uint8 ucLongAxisNum  = pG32->ucAxis[0];
	uint8 ucShortAxisNum = pG32->ucAxis[1];


	if (pG32 == NULL)
	{
		return -1;
	}

	SStartPoint = pG32->SStarCutPoint;	//起点，单位：mm

	//初始化各个轴的绝对坐标
	for (int i = 0; i < AXIS_NUM_MAX; i++)
	{
		pdTargetPoint[i] = SStartPoint.dArray[i];
	}

	pdTargetPoint[ucLongAxisNum]  = pG32->SBackTailPoint.dArray[ucLongAxisNum]  + dLongAxisDeltal * pG32->SStartVector.dArray[pG32->ucAxis[0]];
	// pdTargetPoint[ucShortAxisNum] = pG32->SBackTailPoint.dArray[ucShortAxisNum] + dShortAxisDeltal * sgn(pG32->dJ);
	//修改2026.1.14 由于dShortAxisDeltal 计算时只是乘以短轴与长轴的相对比值，所以此处需要乘以长轴与规划距离的比值
	pdTargetPoint[ucShortAxisNum]  = pG32->SBackTailPoint.dArray[ucShortAxisNum]  + dShortAxisDeltal * pG32->SStartVector.dArray[pG32->ucAxis[0]];

	return 1;

}


/*************************************************
  zxr: 检查 G32 起终点及退尾是否超出已使能软限位
  函数名称：checkG32StartEndSoftLimit
  函数功能：检查 G32 起点、终点及退尾是否超出软限位
  输入参数：pG32，结构体 struct _SG32 指针
  输出参数：无
  返 回 值：0：通过；非 0：对应的报警号（41~46为正限位，47~52为负限位）
  备    注：仅在切入进给前静态检查，不占用 1ms 中断循环；旋转轴不限位。
**************************************************/
static uint16 checkG32StartEndSoftLimit(const struct _SG32* pG32)
{
    SPointDouble SStartPoint;
    SPointDouble SEndPoint;
    SPointDouble STailPoint;
    const uint8* pMapAxisArray;
    uint8 ucAxis;
    bool bHasTail = FALSE;

    if (pG32 == NULL || pG32->pSChannel == NULL)
    {
        return GCODE_ERROR_NUM153;
    }
    if (pG32->pSChannel->ucChannelNum >= CHANNEL_NUM_MAX ||
        pG32->pSChannel->pMapAxisArray == NULL)
    {
        return GCODE_ERROR_NUM150;
    }

    pMapAxisArray = pG32->pSChannel->pMapAxisArray;
    SStartPoint = pG32->SStarCutPoint;
    SEndPoint = pG32->SEndPoint;
    STailPoint = SEndPoint;

    // zxr: 若有短轴退尾量 dJ，计算退尾终点（短轴加上退尾量）
    if (!isZero(pG32->dJ) && pG32->ucAxis[1] < AXIS_NUM_MAX)
    {
        STailPoint.dArray[pG32->ucAxis[1]] += pG32->dJ;
        bHasTail = TRUE;
    }

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE
    /* zxr: 与 motionForG32 实际输出转换保持一致，倾斜 Y 机型换算为实际机械坐标 */
    if (pG32->pSChannel->bUseVirtualY == TRUE)
    {
        if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_X))
        {
            SStartPoint.dArray[AXIS_X] += SStartPoint.dArray[AXIS_Y] * pG32->pSChannel->dTANAngleYY1;
            SEndPoint.dArray[AXIS_X] += SEndPoint.dArray[AXIS_Y] * pG32->pSChannel->dTANAngleYY1;
            if (bHasTail)
            {
                STailPoint.dArray[AXIS_X] += STailPoint.dArray[AXIS_Y] * pG32->pSChannel->dTANAngleYY1;
            }
        }
        if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_Y))
        {
            double dCosAngle = pG32->pSChannel->dCOSAngleYY1;
            if (isfinite(dCosAngle) && !isZero(dCosAngle))
            {
                SStartPoint.dArray[AXIS_Y] /= dCosAngle;
                SEndPoint.dArray[AXIS_Y] /= dCosAngle;
                if (bHasTail)
                {
                    STailPoint.dArray[AXIS_Y] /= dCosAngle;
                }
            }
        }
    }
#endif

    for (ucAxis = 0; ucAxis < AXIS_NUM_MAX; ucAxis++)
    {
        uint8 ucPhysicalAxis;
        double dStart;
        double dEnd;

        if (VAL_VALID != getbit(pG32->ucValStatus, ucAxis))
        {
            continue;
        }
        ucPhysicalAxis = pMapAxisArray[ucAxis];
        if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX ||
            ucPhysicalAxis >= sizeof(g_SSystemPara.MOTOR_WARN) / sizeof(g_SSystemPara.MOTOR_WARN[0]))
        {
            return GCODE_ERROR_NUM150;
        }
        if (g_SSystemPara.SAxisPara[ucPhysicalAxis].ucAxisType == ROTATION_AXIS)
        {
            continue;
        }

        dStart = SStartPoint.dArray[ucAxis];
        dEnd = SEndPoint.dArray[ucAxis];
        if (!isfinite(dStart) || !isfinite(dEnd))
        {
            return GCODE_ERROR_NUM153;
        }

        // 正向软限位检查
        if (g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].soft_pos == 1)
        {
            double dLimit = g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].pos_soft;
            if (!isfinite(dLimit))
            {
                return GCODE_ERROR_NUM153;
            }
            if (dStart > dLimit || dEnd > dLimit || (bHasTail && STailPoint.dArray[ucAxis] > dLimit))
            {
                return (uint16)(GCODE_ERROR_NUM41 + ucAxis);
            }
        }

        // 负向软限位检查
        if (g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].soft_neg == 1)
        {
            double dLimit = g_SSystemPara.MOTOR_WARN[ucPhysicalAxis].neg_soft;
            if (!isfinite(dLimit))
            {
                return GCODE_ERROR_NUM153;
            }
            if (dStart < dLimit || dEnd < dLimit || (bHasTail && STailPoint.dArray[ucAxis] < dLimit))
            {
                return (uint16)(GCODE_ERROR_NUM47 + ucAxis);
            }
        }
    }

    return 0;
}

/*************************************************
  函数名称：void G32Run(struct _SG32* pG32, struct SInterpolationPre* pInterpPrePointer)
  函数功能：G32运行调度
  输入参数：*pG32，   结构体struct _SG32指针
			*pInterpPrePointer，结构体struct SInterpolationPre指针
  输出参数：*pG32，   结构体struct _SG32指针
			*pInterpPrePointer，结构体struct SInterpolationPre指针
			g_SRun              SRun结构体全局变量
  返 回 值：无
  备    注：无
**************************************************/
void G32Run(struct _SG32* pG32)
{

	if (pG32->pSChannel->pSSpindleEncoder == NULL || pG32->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum <= 0)//避免编码器线数为0
	{
		parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
		return;
	}

	double dDistFromSpindle = pG32->pSChannel->pSSpindleEncoder->usDelta * pG32->dF / pG32->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum; //当前主轴转速，长轴应该行走的距离
	double dVectorFromSpindle = dDistFromSpindle / pG32->SPlanData.dTc;  //当前主轴转速下，长轴的运动速度（随动）
	double dAcc = pG32->SUserData.dAcc;
	double dDec = pG32->SUserData.dDec;
	struct SInterpolationPre* pSPreprocess; //预处理指针
	const uint8* ucMapArray;                //映射
	enum SYSTEMSTATUS* pSystemStatus;       //系统状态

	pSystemStatus = &pG32->pSChannel->ucChannelStatus;//通道状态
	pSPreprocess = &pG32->pSChannel->SPreProcess;//预处理指针
	ucMapArray = pG32->pSChannel->pMapAxisArray;//映射

	if (pG32->SUserData.ucUserInputState == USER_INPUT)  //若发现用户外部输入参数（动态修改）
	{
		pG32->ucG32Step = G32_OCCUPATION;  //流程重新开始（进入轴占用）
	}

	//根据G32的运行步骤编号，执行相应步骤
	switch (pG32->ucG32Step)
	{
	case G32_OCCUPATION:  //G32的轴占用
	{
		//释放上一指令的轴占用
		releaseLastCommandAxis(pSPreprocess);
#if(TURRET_AXIS_TEST == 1)
		if (axisManagerOccupyByChannel(pG32->ucValStatus & (~pSPreprocess->ucOccupideTurretAxis), pG32->pSChannel->ucChannelNum, &pSPreprocess->ucOccupideValStatus) == 1)
		{
			//占用成功			
			if (pG32->SUserData.ucUserInputState == USER_INPUT)  //若有用户输入参数
			{
				pG32->ucG32Step = G32_USER_INPUT;  //下一运动阶段为用户运行阶段

			}
			else
			{
				pG32->ucG32Step = G32_INIT_AND_ASSIGNMENT;  //下一运动阶段为初始化和赋值阶段

			}
		}
		else
		{
			break;
		}

#else
		//判断轴占用(根据有效轴+映射)   有效轴减去刀补轴 
		if (axisManagerOccupyByChannel(pG32->ucValStatus & (~pSPreprocess->ucToolAxis), pG32->pSChannel->ucChannelNum,NULL) == 1)
		{	//占用成功
			pSPreprocess->ucLastValStatus = pG32->ucValStatus;

			if (pG32->SUserData.ucUserInputState == USER_INPUT)  //若有用户输入参数
			{
				pG32->ucG32Step = G32_USER_INPUT;  //下一运动阶段为用户运行阶段

			}
			else
			{
				pG32->ucG32Step = G32_INIT_AND_ASSIGNMENT;  //下一运动阶段为初始化和赋值阶段

			}
		}
		else
		{
			break;
		}
#endif
	}
	case G32_USER_INPUT: //用户外部输入参数
	{
		//若有用户输入外部参数
		if (pG32->SUserData.ucUserInputState == USER_INPUT)
		{
			int8 cState = processUserInput(pG32);  //参数错误检查+更迭长轴用于规划的运动参数
			//若用户参数获取失败或无效，直接停止当前运动
			if (cState == -1)
			{
				enum SYSTEMSTATUS* pSystemStatus;			//系统状态
				pG32->ucInpStatus = INP_FINISHED;           //状态改变为FINISHED
				pSPreprocess->ucInpStatus = INP_FINISHED;
				pSystemStatus = &pG32->pSChannel->ucChannelStatus;//通道状态
				*pSystemStatus = SYS_RUN_TO_PAUSE;  //将系统状态变为暂停
				return;

			}
			pG32->ucG32Step = G32_INIT_AND_ASSIGNMENT;


		}

	}
	if (!pG32->bRepeatG32Flag)  //如果当前指令为连续G32指令，则所有螺纹步骤都得在同个插补周期完成
	{
		break;

	}
	case G32_INIT_AND_ASSIGNMENT:  //初始化和赋值
	{
		struct SEncoder* pSSpindleEncoder = pG32->pSChannel->pSSpindleEncoder;
		//当主轴未加减速完成时（前后作用延时时间太短），不进行G32运动
		if (pG32->pSChannel->SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED)
		{
			getInputParameter(pG32);      //获取输入参数
            /* zxr: 进入实际运动前检查起终点及退尾软限位 */
            {
                uint16 usLimitError = checkG32StartEndSoftLimit(pG32);
                if (usLimitError != 0)
                {
                    parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum, usLimitError, 9999);
                    return;
                }
            }
			prepareForMoveAndInit(pG32);  //初始化和运动准备
			dDistFromSpindle = pG32->pSChannel->pSSpindleEncoder->usDelta * pG32->dF / pG32->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum; //当前主轴转速，长轴应该行走的距离
			dVectorFromSpindle = dDistFromSpindle / pG32->SPlanData.dTc;  //当前主轴转速下，长轴的运动速度（随动）

		}

		//******************************************存在用户外部输入的处理***********************************
		if (pG32->SUserData.ucUserInputState == FINSH_CHECK)  //如果用户外部输入数据，同时不需要同步信号
		{
			pG32->bSyncSignal = TRUE;                                     //直接获取同步信号
			pG32->bRepeatG32Flag = TRUE;                                  //初始化后该插补周期直接运动
			pG32->pSChannel->bCalculateSpindleSpeedFlag = FALSE;          //不需要计算主轴实际转速
			pG32->ucG32Step = G32_RUNNING;                                //G32直接进入运行阶段
			dDistFromSpindle = pSSpindleEncoder->usDelta * pG32->dF / pSSpindleEncoder->usEncoderCirclePulseNum; //重算新增距离
			dVectorFromSpindle = dDistFromSpindle / pG32->SPlanData.dTc;  //重算随动速度
			pG32->SUserData.ucUserInputState = NO_INPUT;                 //重置用户外部输入状态

		}
		else if (pG32->SUserData.ucUserInputState == FINSH_CHECK_SYSNCSIGNAL)     //如果用户外部输入数据，同时需要同步信号
		{
			dDistFromSpindle = pSSpindleEncoder->usDelta * pG32->dF / pSSpindleEncoder->usEncoderCirclePulseNum; //重算新增距离
			dVectorFromSpindle = dDistFromSpindle / pG32->SPlanData.dTc;               //重算随动速度
			pG32->SUserData.ucUserInputState = NO_INPUT;                              //重置用户外部输入状态，然后按新的参数获取走普通流程获取同步信号

		}
		//******************************************存在用户外部输入的处理***********************************

	}
	if (!pG32->bRepeatG32Flag)  //如果当前指令为连续G32指令，则所有螺纹步骤都得在同个插补周期完成
	{
		break;

	}
	case G32_JUAGE_REPEATSTATE_AND_GET_REALCUTANGLE:  //连续G32的判断以及获取真正切入角
	{
		if (!pG32->pSChannel->bCalculateSpindleSpeedFlag) //不需要计算主轴实际转速)  //已经获得主轴实际转速
		{
			judgeRepeatG32AndGetRealCutAngle(pG32, dDistFromSpindle);  //连续G32的判断以及获取真正切入角

		}

	}
	if (!pG32->bRepeatG32Flag)  //如果当前指令为连续G32指令，则所有螺纹步骤都得在同个插补周期完成
	{
		break;

	}
	case G32_GET_SYNCSIGN:  //判断是否获取同步信号
	{
		//没有获得同步信号
		if (!pG32->bSyncSignal)
		{
			judgeSyncSignalAndCompensate(pG32);  //判断是否获取同步信号

		}

	}
	case G32_IFM_COMPENSATE:  //主轴不完全跟随模式下，过渡阶段的运动与补偿
	{
		//获得同步信号，同时运行步骤为G32_IFM_COMPENSATE
		if (pG32->bSyncSignal && pG32->ucG32Step == G32_IFM_COMPENSATE)
		{
			runAndCompForIFM(pG32, &dDistFromSpindle);  //主轴不完全跟随模式下，过渡阶段的运动与补偿

		}

	}
	case G32_REPEATSTATE_COMPENSATE:  //减速规划类型连续G32长轴的运动与补偿
	{
		if (pG32->ucG32Step == G32_REPEATSTATE_COMPENSATE)
		{
			runAndCompForG32RepeatDec(pG32, &dDistFromSpindle);  //减速规划类型连续G32长轴的运动与补偿

		}

	}
	case G32_RUNNING:  //运动部分
	{
		//如果获得同步信号
		if (pG32->bSyncSignal && pG32->pSChannel->SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED)
		{
			uint8 ucMotionStateLast = pG32->ucMotionState;  //上一次运动的状态
			int8  cRunState; //计算G32运动插补点的状态
			double dLongAxisDelta;     //长轴距离终点的长度
			double dCompensate;

			//*********************************测试********************************
			g_nTestNum++;
			if (g_nTestNum >= 100 && g_nTestNum < 101)
			{
				//resetButtonAPI(0);
				/*dDistFromSpindle = 0;
				dVectorFromSpindle = 0;*/
				//dDistFromSpindle = 0.05;
				//dVectorFromSpindle = 0.05;

			}

			if (g_nTestNum == 900)
			{
				//UserInput(pG32);  //用户外部输入测试

			}
			//*********************************测试********************************

			//如果运动状态为随动，则加减速度为系统参数的五倍
			if (pG32->ucMotionState == 1)
			{
				dAcc = dAcc * 5;
				dDec = dDec * 5;

			}
			
			if (pG32->bAccPhaseFlag)
			{
				dAcc = pG32->dAcc;
				dDec = pG32->dAcc;
			}
			

			//如果当前主轴转速为0，则长轴的运动速度也为0
			if (isZero(dVectorFromSpindle))
			{
				//g_SInterpolation.dCurrentSpeed = 0;
				pG32->dCurrentSpeed = 0;
			}

			//连续G32如果有起始补偿量
			if (!isZero(pG32->dStartPointCompensate) && pG32->bStartCompensate)
			{

				if (isZero(pG32->dStartPointCompensateCopy))
				{
					pG32->dStartPointCompensateCopy = pG32->dStartPointCompensate;  //获取起始补偿量副本（用于变锥度螺纹，短轴的速度规划补偿）

				}

				if (pG32->dStartPointCompensate < 0)
				{
					double dMaxDelta = pG32->SUserData.dDec;  //单次补偿所允许的最高值
					if (fabs(pG32->dStartPointCompensate) > dMaxDelta)
					{
						pG32->SPlanData.dTargetDist -= dMaxDelta;
						pG32->SPlanData.dCurLeftDist -= dMaxDelta;
						pG32->dStartPointCompensate += dMaxDelta;
					}
					else
					{
						pG32->SPlanData.dTargetDist  += pG32->dStartPointCompensate;
						pG32->SPlanData.dCurLeftDist += pG32->dStartPointCompensate;
						pG32->dStartPointCompensate = 0;

					}

				}
				else
				{
					//速度增加会被规划曲线限制，所有直接将起始补偿量全部放入规划未移动的距离即可
					pG32->SPlanData.dTargetDist += pG32->dStartPointCompensate;
					pG32->SPlanData.dCurLeftDist += pG32->dStartPointCompensate;
					//pG32->dStartPointCompensateCopy = pG32->dStartPointCompensate;  //获取起始补偿量副本（用于变锥度螺纹，短轴的速度规划补偿）
					pG32->dStartPointCompensate = 0;
				}

			}

			//进行速度规划，并返回当前的运动状态
			pG32->ucMotionState = newAddDistace(pG32, &pG32->SPlanData, dDistFromSpindle, dAcc, dDec, dVectorFromSpindle, pG32->dCurrentSpeed);

			//上周期的规划是随动状态，本周期的规划是加速状态，则默认为随动阶段内的加速
			if (ucMotionStateLast == 1 && pG32->ucMotionState == 0)
			{
				pG32->ucMotionState = 1;  //运动状态为加速
			}
			else if (pG32->ucMotionState < 0)//速度规划失败
			{
				parseRunningError(pG32->pSChannel->SRun.unGCodeLineNum,GCODE_ERROR_NUM153,9999);
				return; 
			}

			
			cRunState = motionForG32(pG32, &dLongAxisDelta, dDistFromSpindle, dVectorFromSpindle, dAcc, dDec);  //获取计算G32运动插补点的状态
			//若计算插补点失败，则暂停
			if (cRunState == -1)
			{
				// *pSystemStatus = SYS_RUN_TO_PAUSE;  //系统状态置为暂停
				pG32->dShortAxisPos = 0;
				pG32->ucInpStatus = INP_FINISHED;           //状态改变为FINISHED
				pSPreprocess->ucInpStatus = INP_FINISHED;
				pG32->ucG32Step = G32_OCCUPATION;           //运行步骤初始化
				g_dTime = 0;                                //G32运行次数（测试）
				*pSystemStatus = SYS_PAUSE;					//系统状态置为暂停
				return;
			}

			//若为减速规划类型连续G32,同时还未进行减速规划
			if (pG32->bDecPlanTransitionFlag && pG32->unTac == 0)
			{
				prepareG32RepeatDecPlanning(pG32, &dLongAxisDelta);  //减速类型连续G32过渡规划

			}

			//退尾处理
			if (!isZero(pG32->dJ) && !pG32->bLeftTailFlag)
			{
				retreatTailPre(pG32); //退尾预处理
				if (pG32->bLeftTailFlag)
				{
					pG32->ucG32Step = G32_BACKTAIL;
				}
			}

			//判断是否到达终点(或坐标点计算失败需暂停)
			if (dLongAxisDelta > 0 || isZero(dLongAxisDelta))
			{
				//若能正常计算插补点
				//为加速类型的连续G32
				if (!pG32->bDecPlanTransitionFlag && !isZero(pG32->dNextG32LongAxisSpeed))
				{
					compensateForAccRepeatG32(pG32);  //加速类型连续G32的补偿量计算

				}
				//当为连续G32时存储本次G32的锥度，用于下一段G32的锥度比较
				if (getSToolCompPointerBufRes(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum) > 0)  //若"速度规划读"指针有内容可读
				{
					struct SToolCompensation* pSToolCompPointer = NULL;                //刀补后产生的结构体指针，用于读取下一个指令类型
					pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pG32->pSChannel->ucChannelNum); //获取"速度规划读"
					if (getBits(pSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT)
					{
						pG32->dLastRatioXZ = pG32->dRatioXZ;  //存储本次G32的锥度
					}

					}
				else
				{
					pG32->dLastRatioXZ = 0;  //不需要存储本次G32的锥度

				}
				pG32->dShortAxisPos = 0;
				pG32->ucInpStatus = INP_FINISHED;           //状态改变为FINISHED
				pSPreprocess->ucInpStatus = INP_FINISHED;
				pG32->ucG32Step = G32_OCCUPATION;           //运行步骤初始化
				g_dTime = 0;                                //G32运行次数（测试）

				//G32运行过程中，不暂停，运行完后才暂停
				if (*pSystemStatus == SYS_RUN_TO_PAUSE && pSPreprocess->ucInpStatus == INP_FINISHED)
				{
					*pSystemStatus = SYS_PAUSE;  //系统状态置为暂停

				}
			}

		}

	}
	break;
	case G32_BACKTAIL:
	{
		double dLongAxisDeltal;          //长轴相对起点的距离 
		double dShortAxisDeltal;        //短轴相对起点的距离
		SPointDouble STargetPoint;       //当前点的理论绝对坐标
		struct _SRun* pSRun;             //运行结构体
		pSRun = &pG32->pSChannel->SRun;  //运行结构体

		pG32->SPlanData.unCurrentT += 1;  //运行时间增加
		//获取currentT时刻的线段长（直线）

		/*修改2025.11.27 使用G1进行退尾，不需要短轴进行速度规划*/
			// if (pG32->SExcessPlanForG32 != NULL)
			// {
			// 	dLongAxisDeltal  = nextstepPlanningAndDistReAvg(pG32->SPlanData.unCurrentT, &(pG32->SPlanData));
			// 	dShortAxisDeltal = nextstepPlanningAndDistReAvg(pG32->SPlanData.unCurrentT, pG32->SExcessPlanForG32);
			// 	//计算长轴理论坐标点
			// 	calculateExactPointForG32BackTail((double*)&STargetPoint, dLongAxisDeltal, dShortAxisDeltal, pG32);
			// }
		dLongAxisDeltal = nextstepPlanningAndDistReAvg(pG32->SPlanData.unCurrentT, &(pG32->SPlanData));//计算长轴距离
//		pG32->dRatioXZ = (pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SBackTailPoint.dArray[pG32->ucAxis[1]]) / pG32->dK;//计算短轴与长轴的比值	
//      pG32->dRatioXZ = (pG32->SEndPoint.dArray[pG32->ucAxis[1]] - pG32->SBackTailPoint.dArray[pG32->ucAxis[1]]) / (pG32->SEndPoint.dArray[pG32->ucAxis[0]] - pG32->SBackTailPoint.dArray[pG32->ucAxis[0]]);//修改2026.1.14 计算短轴与长轴的比值
		dShortAxisDeltal = dLongAxisDeltal * pG32->dRatioXZ ; // 计算短轴距离

		// 计算长短轴理论坐标点
		calculateExactPointForG32BackTail((double *)&STargetPoint, dLongAxisDeltal, dShortAxisDeltal, pG32);
		/*修改2025.11.27 使用G1进行退尾，不需要短轴进行速度规划*/

#if MACHINE_CONFIG == TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE//倾斜Y轴机 
		if (pG32->pSChannel->bUseVirtualY == TRUE)
		{
			if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_X))
			{
				STargetPoint.dArray[AXIS_X] += 1.0 * STargetPoint.dArray[AXIS_Y] * pG32->pSChannel->dTANAngleYY1;
			}
			if (VAL_VALID == getbit(pG32->ucValStatus, AXIS_Y))
			{
				STargetPoint.dArray[AXIS_Y] = STargetPoint.dArray[AXIS_Y] / pG32->pSChannel->dCOSAngleYY1;
			}
		}
#endif

#if ERROR_COMP_TYPE == SCREW_AND_REVCLEARANCE //单表螺距补偿+ 反向间隙补偿
		//利用理论坐标生成SRun结构体数据
		outPutAbsPointWithScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, pG32->dPulseToMm, &pG32->ucValStatus);
#elif ERROR_COMP_TYPE == TWOWAY_SCREW_COMP //双表螺距补偿
		outPutAbsPointWithBilateralScrewComp(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, pG32->dPulseToMm, &pG32->ucValStatus);
#else
		// 利用理论坐标生成SRun结构体数据
		outPutAbsPoint(pSRun, (double*)&STargetPoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, pG32->dPulseToMm, &pG32->ucValStatus); //利用理论坐标生成SRun结构体数据（未完成）
#endif

		//计算当前速度
		pG32->dCurrentSpeed = fabs(pG32->SPlanData.dCurLeftDist - (pG32->SPlanData.dTargetDist - dLongAxisDeltal)) / pG32->SPlanData.dTc;

		//计算   !理论! 未走距离
		pG32->SPlanData.dCurLeftDist = pG32->SPlanData.dTargetDist - dLongAxisDeltal;

		//运行结束条件
		if ((pG32->SPlanData.dCurLeftDist <= EPSINON))
		{
			pG32->ucInpStatus = INP_FINISHED;           //状态改变为FINISHED
			pSPreprocess->ucInpStatus = INP_FINISHED;
			pG32->ucG32Step = G32_OCCUPATION;           //运行步骤初始化
			g_dTime = 0;                                //G32运行次数（测试）

			if (pG32->SExcessPlanForG32 != NULL)
			{
				free(pG32->SExcessPlanForG32);        //释放短轴速度规划参数指针
				pG32->SExcessPlanForG32 = NULL;
			}
			

			//G32运行过程中，不暂停，运行完后才暂停
			if (*pSystemStatus == SYS_RUN_TO_PAUSE && pSPreprocess->ucInpStatus == INP_FINISHED)
			{
				*pSystemStatus = SYS_PAUSE;  //系统状态置为暂停
				changeNoRepeatG32(pG32->pSChannel);//将下一段G32修改为非连续G32
			}
			
		}
	}
	break;
	default:
		break;

	}

}

void initResetForG32(struct SChannel* pSChannel)
{
	// if(getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT) 暂时注释，没用作用
	// {
	// 	pSChannel->SG32.SPlanData.dTargetDist += pSChannel->SG00.pSSingleAxisArray[pSChannel->SG32.ucAxis[0]].SPlanData.dDistance;
	// }
	changeNoRepeatG32(pSChannel);
}


/*************************************************
  函数名称：void changeNoRepeatG32(struct SChannel* pSChannel)
  函数功能：修改为非连续G32
  输入参数：*pSChannel，   通道结构体指针
  输出参数：无
  返 回 值：无
  备    注：无
**************************************************/
void changeNoRepeatG32(struct SChannel* pSChannel)
{
	struct SToolCompensation* pSToolCompPointer = NULL;                //刀补后产生的结构体指针，用于读取下一个指令类型

	//若"速度规划读"指针有内容可读
	if (getSToolCompPointerBufRes(INTERPOLATION_READ, pSChannel->ucChannelNum) > 0)
	{
		pSToolCompPointer = getSToolCompPointerByType(INTERPOLATION_READ, pSChannel->ucChannelNum); //获取"速度规划读"
		if (getBits(pSToolCompPointer->unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT)
		{
			 pSToolCompPointer->bRepeatG32Flag = FALSE;//将下一段G32设置为非连续
		}
	}
	if(pSChannel->ucChannelStatus == SYS_IDLE && getBits(pSChannel->SRun.unCommandType, CMD_MOTION_ADDR) == CMD_MOTION_SCREWCUT)//空闲状态将G32改为默认G0
	{
		setBits(pSChannel->SRun.unCommandType, CMD_MOTION_QUICKMOVE, CMD_MOTION_ADDR); // 改为快速移动
	}
}

//void getHandWheelData(struct SChannel* pSChannel)
//{
//	double dDecimalPart;
//	int16 usSendValue;
//	struct SEncoder* pEncoder;
//	pEncoder = pSChannel->pSHandleWheelEncoder;
//	uint8 ucAXISNum = g_SEncoder[HANDEWHEEL_ENCODER - 1].AXISNum;
//	uint8* pMapAxisArray = pSChannel->pMapAxisArray;
//	int i = 0;  //循环次数
//	double dDeltaL; //各个轴的位移增量
//	VAL_STRUCT_TYPE ucValStatus = 0;
//
//	//获取手轮旋转对应的脉冲数
//	if (pEncoder->cEncoderDir == INP_GO_BACK)
//	{
//		usSendValue = -pEncoder->usDelta * pEncoder->ucWheelMultiple;
//	}
//	else
//	{
//		usSendValue = pEncoder->usDelta * pEncoder->ucWheelMultiple;
//	}
//
//	//得到允许去频化的脉冲
//	if (abs(usSendValue) % 4 != 0)
//	{
//		pEncoder->dDelataAccmulate += usSendValue;
//		usSendValue = 0;
//		if (abs(pEncoder->dDelataAccmulate) >= 4)
//		{
//			usSendValue = pEncoder->dDelataAccmulate - (pEncoder->dDelataAccmulate % 4);
//			pEncoder->dDelataAccmulate -= usSendValue;
//		}
//	}
//
//	//虚拟轴的映射不为空
//	if (pMapAxisArray[ucAXISNum] != AXIS_NULL)
//	{
//		double dAxisPulseToMm = 0;  //运动轴脉冲数对应的距离
//		SPointDouble STargetPoint;  //当前点的理论绝对坐标
//		struct SPlan* pSPlanPointer = &pSChannel->SSingleAxisArray[ucAXISNum].SPlanData;  //手轮所控制运动轴的速度规划结构体
//		double dPreDist = 0;  //已插补长度
//		dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;
//		uint8 ucPlanState = 0;//速度规划状态标志位
//
//		int8 cVector = sgn(pEncoder->sAxisAccmulate[ucAXISNum] - round(dPreDist / g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm));
//		int8 cVectorBack = sgn(usSendValue);
//
//		//手轮有拨动
//		if (usSendValue != 0)
//		{
//			//手轮方向发生变化
//			if (cVector != cVectorBack)
//			{
//				pEncoder->sPulseForChangeDie[ucAXISNum] += usSendValue; //换向时的脉冲缓冲（数值不为0即为换向标志位）
//			}
//			else
//			{
//				pEncoder->sAxisAccmulate[ucAXISNum] += usSendValue;   //虚拟轴
//			}
//
//		}
//
//		SPointDouble SRealTimePoint;  //获取当前实时点
//		getRealTimePoint((double*)&SRealTimePoint, AXIS_NUM_MAX, pMapAxisArray, NULL);  //获取实时坐标
//
//		//由于可能存在多轴运动的情况，所以得这么处理
//		for (i = 0; i < AXIS_NUM_MAX; i++)
//		{
//			dDeltaL = 0;				//初始化各个轴的位移增量
//			STargetPoint.dArray[i] = 0; //初始化各个轴目标坐标
//
//			//运动轴的存在未规划的脉冲数
//			if (pEncoder->sAxisAccmulate[i] != 0)
//			{
//				pSChannel->SSingleAxisArray[i].SPlanData.cInterpDir = sgn(pEncoder->sAxisAccmulate[i]);  //方向判断方式待定
//				dAxisPulseToMm = fabs(pEncoder->sAxisAccmulate[i]) * g_SSystemPara.SAxisPara[pMapAxisArray[i]].dPulseToMm;
//				ucPlanState = handWheelAddNewPlan(&pSChannel->SSingleAxisArray[i].SPlanData, dAxisPulseToMm, g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVs, g_SSystemPara.SAxisPara[pMapAxisArray[i]].dVe, pEncoder->dAxisCurrentSpeed[i]);
//				dDeltaL = nextstepPlanningAndDistReAvg(1, &pSChannel->SSingleAxisArray[i].SPlanData);
//				STargetPoint.dArray[i] = SRealTimePoint.dArray[i] + pSChannel->SSingleAxisArray[i].SPlanData.cInterpDir * dDeltaL;
//				setbit(ucValStatus, i);
//			}
//		}
//
//		outPutAbsPoint(&pSChannel->SRun, (double*)&STargetPoint, AXIS_NUM_MAX, pSChannel->pMapAxisArray, pSChannel->SG01.dPulseToMm, &ucValStatus); //利用理论坐标生成SRun结构体数据（未完成）
//
//		for (i = 0; i < AXIS_NUM_MAX; i++)
//		{
//			if (pSChannel->pMapAxisArray[i] >= AXIS_PHYSICS_NUM_MAX || VAL_VALID != getbit(ucValStatus, i)) continue;  //映射为空的物理轴  不计算
//
//			//如果当前留下未移动的距离大于等于0
//			if (pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist > 0 || isZero(pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist))
//			{
//				//计算当前理论速度
//				pEncoder->dAxisCurrentSpeed[i] = fabs(pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist - (pSChannel->SSingleAxisArray[i].SPlanData.dTargetDist - dDeltaL)) / pSChannel->SSingleAxisArray[i].SPlanData.dTc;
//				pSChannel->SSingleAxisArray[i].SPlanData.dCurLeftDist = pSChannel->SSingleAxisArray[i].SPlanData.dTargetDist - dDeltaL;  //计算理论未走距离
//			}
//			else
//			{
//				pEncoder->dAxisCurrentSpeed[i] = 0;  //当前速度为0
//			}
//
//		}
//
//		//插补结束
//		if (ucPlanState == 3)
//		{
//			pSPlanPointer->dDistance = 0;   //长轴的插补长度
//			pSPlanPointer->dPreDist = 0;   //已插补长度
//			pSPlanPointer->dTargetDist = 0;   //目标距离无总长度限制
//			pSPlanPointer->dCurLeftDist = 0;   //计算剩余距离（规划用)
//			pEncoder->sAxisAccmulate[ucAXISNum] = 0;
//
//			//换向还未移动的距离
//			if (pEncoder->sPulseForChangeDie[ucAXISNum] != 0)
//			{
//				pEncoder->sAxisAccmulate[ucAXISNum] += pEncoder->sPulseForChangeDie[ucAXISNum];
//				pEncoder->sPulseForChangeDie[ucAXISNum] = 0;
//			}
//
//		}
//
//
//	}
//
//
//}



