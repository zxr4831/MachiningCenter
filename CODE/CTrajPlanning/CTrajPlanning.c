#include "CTrajPlanning.h"
#include "math.h"
#include <stdio.h>
#include "common.h"
#include "GeneralFunctions.h"
#include "G32.h"
#include <stdlib.h>
#include "Channel.h"
#include "Config.h"
/*************************************************
  函数名称：getFactor(uint8 ucVelPlanType)
  函数功能  根据输入的速度曲线类型  返回距离计算公式的计算系数  默认状态下返回S4的系数
  输入参数：ucVelPlanType	曲线类型
  输出参数：
  返 回 值：计算系数
  备    注：
**************************************************/
double getFactor(uint8 ucVelPlanType)
{
	switch (ucVelPlanType)
	{
	case VEL_PLAN_T_CURVE://T型
		return 0.5;
		break;
	case VEL_PLAN_S3_CURVE://S3型
		return 1.0;
		break;
	case VEL_PLAN_S4_CURVE://S4型
		return 0.75;
		break;
	default:
		return 0.75;
	}
}

/*************************************************
  函数名称：getFactor(uint8 ucVelPlanType)
  函数功能  根据输入的速度曲线类型  返回系统参数对应的加速度
  输入参数：ucVelPlanType	曲线类型
  输出参数：
  返 回 值：加速度
  备    注：
**************************************************/
double getSysAcc(uint8 ucVelPlanType)
{
	switch (ucVelPlanType)
	{
	case VEL_PLAN_T_CURVE://T型
		return g_SSystemPara.dSysPlanTMaxAcc;
		break;
	case VEL_PLAN_S3_CURVE://S3型
		return g_SSystemPara.dSysPlanS3MaxAcc;
		break;
	case VEL_PLAN_S4_CURVE://S4型
		return g_SSystemPara.dSysPlanS4MaxAcc;
		break;
	default:
		return g_SSystemPara.dSysPlanS4MaxAcc;
	}
}


/*************************************************
  函数名称：getFactor(uint8 ucVelPlanType)
  函数功能  根据输入的速度曲线类型  返回系统参数对应的减速度
  输入参数：ucVelPlanType	曲线类型
  输出参数：
  返 回 值：减速度
  备    注：
**************************************************/
double getSysDec(uint8 ucVelPlanType)
{
	switch (ucVelPlanType)
	{
	case VEL_PLAN_T_CURVE://T型
		return g_SSystemPara.dSysPlanTMaxDec;
		break;
	case VEL_PLAN_S3_CURVE://S3型
		return g_SSystemPara.dSysPlanS3MaxDec;
		break;
	case VEL_PLAN_S4_CURVE://S4型
		return g_SSystemPara.dSysPlanS4MaxDec;
		break;
	default:
		return g_SSystemPara.dSysPlanS4MaxDec;
	}
}


double getSingleAxisAcc(uint8 ucAxis, uint8 ucVelPlanType, const uint8* ucAxisMap)
{
	uint8 ucPhysicalAxis;
	if (ucAxis  >= AXIS_NUM_MAX || ucAxisMap == NULL)//超过轴数量或空指针
	{
		return 0;
	}
	ucPhysicalAxis = ucAxisMap[ucAxis ];
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return 0.0;
	}

	switch (ucVelPlanType)
	{
	case VEL_PLAN_T_CURVE://T型
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanTMaxAcc;
		break;
	case VEL_PLAN_S3_CURVE://S3型
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanS3MaxAcc;
		break;
	case VEL_PLAN_S4_CURVE://S4型
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanS4MaxAcc;
		break;
	default:
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanS4MaxAcc;
	}
}

double getSingleAxisDec(uint8 ucAxis, uint8 ucVelPlanType, const uint8* ucAxisMap)
{
	uint8 ucPhysicalAxis;
	if (ucAxis  >= AXIS_NUM_MAX || ucAxisMap == NULL)//超过轴数量或空指针
	{
		return 0;
	}
	ucPhysicalAxis = ucAxisMap[ucAxis ];
	if (ucPhysicalAxis >= AXIS_PHYSICS_NUM_MAX)
	{
		return 0.0;
	}

	switch (ucVelPlanType)
	{
	case VEL_PLAN_T_CURVE://T型
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanTMaxDec;
		break;
	case VEL_PLAN_S3_CURVE://S3型
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanS3MaxDec;
		break;
	case VEL_PLAN_S4_CURVE://S4型
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanS4MaxDec;
		break;
	default:
		return g_SSystemPara.SAxisPara[ucPhysicalAxis ].dSysPlanS4MaxDec;
	}
}

/*************************************************
  函数名称：preparePlanningT(struct SPlan *pSPlanPointer)
  函数功能  根据线段长度，速度参数，加速度参数，进行梯形速度规划，得到各段位移，时间节点，残余距离
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：2   规划成功 但Vmax=0
			1   规划成功 Vmax≠0
			0	距离太短，忽略
			-1	输入错误
  备    注：
**************************************************/
int8 preparePlanningT(struct SPlan* pSPlanPointer)
{
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离

	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	dTc = pSPlanPointer->dTc;		//插补周期



	if (pSPlanPointer->cInterpDir != INP_GO_BACK)//正走
	{
		dDistance = pSPlanPointer->dCurLeftDist;//需要规划的距离
	}
	else//反走
	{
		dDistance = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//需要规划的距离
	}

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}

	//函数运行开始
	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3
			= pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0; //距离对应的时间节点置0
		return 0;
	}
	if (dVe > dVmax)
	{
		dVe = dVmax;//限制末速度
	}

	pSPlanPointer->dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//规划前已走

	dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
	dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移

	if (dDistance < dSac + dSde) //路程不足加减速
	{
		//重算最大速度
		dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));

		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //重算加减速段位移
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
#if EN_DEBUG
		printf("dDistance < dSac + dSde\r\n");
#endif
	}

	dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
	dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
	dTconst = (isZero(dVmax)) ? 0 : (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期  最大速度为0时 匀速周期为0

	if (dTconst < 0 && !isZero(dTconst))   //距离不足没有匀速的情况
	{
		if (dTac < 0 || dTde < 0)  //距离不足
		{
			//动态规划
			//Ve浮动
			if (dVs > dVe)//减速
			{
				dVe = sqrt(dVs * dVs - dDistance * 2.0 * dDec);
				dVmax = dVs;
			}
			else//加速
			{
				dVe = sqrt(dVs * dVs + 2.0 * dAcc * dDistance);
				dVmax = dVe;
			}
			dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		}
		dTconst = 0;
	}

	if (dTac < 0)//距离足够 但是加速周期为负，加速段变为减速段
	{
		dTac = fabs(dTac);
		dAcc = -(dAcc);
	}

	// 	dTac = (int)(dTac + 0.5);    //加速周期四舍五入取整  
	// 	dTde = (int)(dTde + 0.5);    //减速周期四舍五入取整  
	dTac = ceil(dTac);//向上取整
	dTde = ceil(dTde);

	//	dVmax = dTac * dTc * dAcc + dVs;        //重算最大速度  （因为加速时间取整）	

	if (isZero(dTac))//加速周期为0
	{
		dVs = dVmax;
	}
	else
	{
		dAcc = (dVmax - dVs) / (dTac * dTc);//重算加速度
	}

	if (isZero(dTde))  //减速周期为0 
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{
		dDec = (dVmax - dVe) / (dTde * dTc);      //重算减速段加速度（因为减速时间取整）
	}

	dSac = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //加速段位移（时间取整后）
	dSde = (dVmax * dVmax - dVe * dVe) / (dDec * 2);   //减速段位移 

	//dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期
	dTconst = (isZero(dVmax)) ? 0 : (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期  最大速度为0时 匀速周期为0

	if (dTconst < 0)  //没有匀速周期
	{
		dTconst = 0;
	}
	dTconst = (int)(dTconst + 0.5);  //匀速周期取整

	unT1 = unT2 = (int32)dTac;               //时间节点T1  （绝对时间） 
	unT3 = unT2 + (int32)dTconst; //时间节点T2  （绝对时间） 
	unT4 = unT5 = unT3 + (int32)dTde;       //时间节点T3  （绝对时间） 

	dSconst = dVmax * (unT3 - unT1) * dTc; //计算匀速段位移

	dDistRemain = dDistance - dSac - dSde - dSconst;  //计算残余距离

	//传出参数
	pSPlanPointer->unT1 = unT1;		//时间节点T1,2,3
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;

	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("T形速度规划\n");
	printf("dTac = %lf\r\n", dTac);
	printf("dTconst = %lf\r\n", dTconst);
	printf("dTde = %lf\r\n", dTde);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dDe = %lf\r\n", dDistRemain);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("dVmax = %lf\r\n", dVmax);
#endif
	return (isZero(dVmax)) ? 2 : 1;
}


/*************************************************
  函数名称：preparePlanningS3(struct SPlan *pSPlanPointer)
  函数功能  根据线段长度，速度参数，加速度参数，进行S3型速度规划，得到各段位移，时间节点，残余距离
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：2   规划成功 但Vmax=0
			1   规划成功 Vmax≠0
			0	距离太短，忽略
			-1	输入错误
  备    注：
**************************************************/
int8 preparePlanningS3(struct SPlan* pSPlanPointer)
{
	//不需初始化的参数
	double dTacHalf, dTconst, dTdeHalf;//加速,匀速,减速周期
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期
	double dDistAcc, dDistConst, dDistDec;//加速，匀速，减速距离

	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	//dDistance	= pSPlanPointer->dDistance; //总距离
	dTc = pSPlanPointer->dTc;		//插补周期

	if (pSPlanPointer->cInterpDir != INP_GO_BACK)//正走
	{
		dDistance = pSPlanPointer->dCurLeftDist;//需要规划的距离
	}
	else//反走
	{
		dDistance = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//需要规划的距离
	}

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}

	//开始速度规划
	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3
			= pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0; //距离对应的时间节点置0
		return 0;
	}
	if (dVe > dVmax)
	{
		dVe = dVmax;//限制末速度
	}

	pSPlanPointer->dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//规划前已走

	dDistAcc = fabs((dVmax * dVmax - dVs * dVs) / dAcc);   //计算加速段位移
	dDistDec = fabs((dVmax * dVmax - dVe * dVe) / dDec);   //计算减速段位移

	if (dDistance < dDistAcc + dDistDec)  //距离不够  重算
	{
		dVmax = sqrt((dAcc * dDec * dDistance + dDec * dVs * dVs + dAcc * dVe * dVe) / (dAcc + dDec));   //重算最大速度
		//重算加减速距离
		dDistAcc = fabs((dVmax * dVmax - dVs * dVs) / dAcc);   //重算加速距离
		dDistDec = fabs((dVmax * dVmax - dVe * dVe) / dDec);   //重算减速距离
	}

	dTacHalf = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
	dTdeHalf = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
	dTconst = (isZero(dVmax)) ? 0 : (dDistance - dDistAcc - dDistDec) / (dVmax * dTc);//计算匀速周期  最大速度为0时 匀速周期为0

	if (dTconst < 0 && !isZero(dTconst))   //距离不足（没有匀速）的情况
	{
		if (dTacHalf < 0 || dTdeHalf < 0)  //距离不足 （只加速/减速 
		{
			if (dVs > dVe)//减速
			{
				dVe = sqrt(dVs * dVs - dDistance * dDec);
				dVmax = dVs;
			}
			else//加速
			{
				dVe = sqrt(dVs * dVs + dDistance * dAcc);
				dVmax = dVe;
			}
			dTacHalf = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			dTdeHalf = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
		}
		dTconst = 0;
	}
	if (dTacHalf < 0)//距离足够 但是加速周期为负，加速段变为减速段
	{
		dTacHalf = fabs(dTacHalf);
		dAcc = -dAcc;
	}


	// 	dTacHalf = (int)(dTacHalf + 0.5);  //加速周期四舍五入 (dTac1 2 现在为整数）
	// 	dTdeHalf = (int)(dTdeHalf + 0.5);  //减速周期四舍五入 (dTde1 2 现在为整数）

	dTacHalf = ceil(dTacHalf);//向上取整
	dTdeHalf = ceil(dTdeHalf);

	//	dVmax = (double)dTacHalf * dAcc * dTc + dVs;  //重算最大速度

	if (isZero(dTacHalf))//加速周期为0
	{
		dVs = dVmax;
	}
	else
	{
		dAcc = (dVmax - dVs) / (dTacHalf * dTc);//重算加速度
	}

	if (isZero(dTdeHalf))  //减速周期为0 （前提是最大速度不会比终点速度小很多(>dDec)）
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{
		dDec = (dVmax - dVe) / (dTdeHalf * dTc);  //重算减速段加速度
	}

	//重算加减速距离
	dDistAcc = (dVmax * dVmax - dVs * dVs) / dAcc;
	dDistDec = (dVmax * dVmax - dVe * dVe) / dDec;

	dTconst = (isZero(dVmax)) ? 0 : (dDistance - dDistAcc - dDistDec) / (dVmax * dTc);   //计算匀速周期
	dTconst = (int)(dTconst + 0.5);       //匀速周期取整
	if (dTconst < 0)
	{
		dTconst = 0;  //没有匀速周期
	}

	//计算时间节点  （绝对时间）
	unT1 = (int32)(dTacHalf);
	unT2 = 2 * unT1;
	unT3 = unT2 + (int32)dTconst;
	unT4 = unT3 + (int32)dTdeHalf;
	unT5 = unT4 + (int32)dTdeHalf;

	dDistConst = dVmax * dTc * (int)dTconst;  //计算匀速距离

	dDistRemain = dDistance - dDistAcc - dDistDec - dDistConst;  //计算残余距离

	//传出参数
	pSPlanPointer->unT1 = unT1;		//时间节点T1,2,3,4,5
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;

	pSPlanPointer->dDistAcc = dDistAcc;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dDistConst;
	pSPlanPointer->dDistDec = dDistDec;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;

	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;

	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("S3形速度规划\n");
	printf("dTac1 = %lf\r\n", dTacHalf);
	printf("dTconst = %lf\r\n", dTconst);
	printf("dTde1 = %lf\r\n", dTdeHalf);
	printf("dSac = %20.16lf\r\n", dDistAcc);
	printf("dSconst = %20.16lf\r\n", dDistConst);
	printf("dSde = %20.16lf\r\n", dDistDec);
	printf("dDe = %lf\r\n", dDistRemain);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("nT4 = %d\r\n", unT4);
	printf("nT5 = %d\r\n", unT5);
	printf("dAcc = %20.16lf\r\n", dAcc);
	printf("dDec = %20.16lf\r\n", dDec);
	printf("dVs = %20.16lf\r\n", dVs);
	printf("dVe = %20.16lf\r\n", dVe);
	printf("dVmax = %20.16lf\r\n", dVmax);
	printf("dTc   = %20.16lf\n", dTc);
#endif

	return (isZero(dVmax)) ? 2 : 1;

}


/*************************************************
  函数名称：preparePlanningS4(struct SPlan *pSPlanPointer)
  函数功能  根据线段长度，速度参数，加速度参数，进行S4型速度规划，得到各段位移，时间节点，残余距离
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：2   规划成功 但Vmax=0
			1   规划成功 Vmax≠0
			0	距离太短，忽略
			-1	输入错误
  备    注：
**************************************************/
int8 preparePlanningS4(struct SPlan* pSPlanPointer)
{
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离

	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	dDistance = pSPlanPointer->dDistance; //总距离
	dTc = pSPlanPointer->dTc;		//插补周期

	if (pSPlanPointer->cInterpDir != INP_GO_BACK)//正走
	{
		dDistance = pSPlanPointer->dCurLeftDist;//需要规划的距离
	}
	else//反走
	{
		dDistance = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//需要规划的距离
	}

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}

	//开始速度规划
	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3
			= pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0; //距离对应的时间节点置0
		return 0;
	}
	if (dVe > dVmax)
	{
		dVe = dVmax;//限制末速度
	}

	pSPlanPointer->dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//规划前已走
	//计算加减速距离
	dSac = fabs(3 * (dVmax * dVmax - dVs * dVs) / (4 * dAcc));  //ps:减档减速,最大速度小于起始速度 加速段位移取绝对值
	dSde = fabs(3 * (dVmax * dVmax - dVe * dVe) / (4 * dDec));

	if (dDistance < dSac + dSde) //距离不够 重算最大速度 
	{
		//重算最大速度  
		dVmax = sqrt((4 * dAcc * dDec * dDistance + 3 * dDec * dVs * dVs + 3 * dAcc * dVe * dVe) / (3 * (dAcc + dDec)));

		dSac = fabs((3 * (dVmax * dVmax - dVs * dVs)) / (4 * dAcc));  //重算加速距离  
		dSde = fabs((3 * (dVmax * dVmax - dVe * dVe)) / (4 * dDec));  //重算减速距离         
	}

	//计算时间增量
	dTac = (3 * (dVmax - dVs)) / (2 * dAcc * dTc);
	dTde = (3 * (dVmax - dVe)) / (2 * dDec * dTc);
	dTconst = (isZero(dVmax)) ? 0 : (dDistance - (dSac + dSde)) / (dVmax * dTc);//计算匀速周期  最大速度为0时 匀速周期为0

	if (dTconst < 0 && !isZero(dTconst))   //没有匀速的情况
	{
		if (dTac < 0 || dTde < 0)  //距离不足 （只加速/减速 
		{
			if (dVs > dVe)//减速
			{
				dVe = sqrt(dVs * dVs - dDistance * dDec / 0.75);
				dVmax = dVs;
			}
			else//加速
			{
				dVe = sqrt(dVs * dVs + dDistance * dAcc / 0.75);
				dVmax = dVe;
			}
			//dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			//dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
			dTac = (3 * (dVmax - dVs)) / (2 * dAcc * dTc);
			dTde = (3 * (dVmax - dVe)) / (2 * dDec * dTc);
		}
		dTconst = 0;
	}
	if (dTac < 0)//距离足够 但是加速周期为负，加速段变为减速段
	{
		dTac = fabs(dTac);//时间取绝对值
		dAcc = -dAcc;   //加速度取反
	}

	//加减速周期取整
// 	dTac=(int)(dTac+0.5);
// 	dTde=(int)(dTde+0.5);  

	dTac = ceil(dTac);//向上取整
	dTde = ceil(dTde);

	// 	//旧方案：加速时间圆整后，需重算最大速度，由于减速时间固定，此时要重算减速度   
	//     dVmax = 2.0 * dAcc * (double)dTac * dTc / 3.0 + dVs; //重算最大速度
		//新方案：加速时间为0时 起始速度等于最大速度   加速时间不为0时，重算加速度
	if (isZero(dTac))
	{
		dVs = dVmax;
	}
	else
	{
		dAcc = (3 * (dVmax - dVs)) / (2 * dTac * dTc);
	}


	if (isZero(dTde))  //减速周期为0，修整最终速度
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{
		dDec = 3 * (dVmax - dVe) / (2.0 * (double)dTde * dTc); //重算减速度
	}

	//重算加减速距离  
	dSac = fabs(3 * (dVmax * dVmax - dVs * dVs) / (4 * dAcc));
	dSde = fabs(3 * (dVmax * dVmax - dVe * dVe) / (4 * dDec));

	dTconst = (isZero(dVmax)) ? 0 : (dDistance - dSac - dSde) / (dVmax * dTc); //计算匀速周期  
	if (dTconst < 0)  // 匀速周期为负时，取0
	{
		dTconst = 0;
	}
	dTconst = (int)(dTconst + 0.5);//匀速周期取整

	//时间取整后，计算时间节点（绝对时间）  
	unT1 = unT2 = (int32)dTac;
	unT3 = unT2 + (int32)dTconst;
	unT4 = unT5 = unT3 + (int32)dTde;

	dSconst = dVmax * (int32)dTconst * dTc; //计算匀速距离  

	dDistRemain = dDistance - dSac - dSde - dSconst; //残余距离 

	//传出参数
	pSPlanPointer->unT1 = unT1;		//时间节点T1,2,3
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;


	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("S4形速度规划\n");
	printf("**************************\n");
	printf("k_acc = %lf\r\n", dTac);
	printf("k_const = %lf\r\n", dTconst);
	printf("k_dec = %lf\r\n", dTde);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dAcc = %20.16lf\r\n", dAcc);
	printf("dDec = %20.16lf\r\n", dDec);
	printf("dDe  = %20.16lf\r\n", dDistRemain);
	printf("dVs = %20.16lf\r\n", dVs);
	printf("dVe = %20.16lf\r\n", dVe);
	printf("dVmax = %20.16lf\r\n", dVmax);
	printf("**************************\n");
#endif
	return (isZero(dVmax)) ? 2 : 1;
}



/*************************************************
  函数名称：preparePlanningCombine(struct SPlan *pSPlanPointer)
  函数功能  根据线段长度，速度参数，加速度参数，进行混合型速度规划，得到各段位移，时间节点，残余距离
  输入参数：*pSPlanPointer，结构体struct SPlan指针
			bUseFloatingVe		是否使用浮动的速度规划
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：
			 2  输入的最大速度为0
			 1  规划成功
			 0	距离太短,忽略
			-1	输入错误
			-2	距离不足以加减速
  备    注：
**************************************************/
int8 preparePlanningCombine(struct SPlan* pSPlanPointer, bool bUseFloatingVe)
{
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离

	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期
	double dAccFactor = 0, dDecFactor = 0; //加速公式系数  减速公式系数  （根据曲线类型取值）
	double dPreDist = 0;

	/*三角形曲线修正为梯形曲线*///  洪冠鹏 2024/3/23
	bool bTriangleToTrapezoid = FALSE;//三角形速度曲线修正标志位
	double dTconstForTrapezoid = 0;//三角形速度曲线修正为梯形速度曲线下的匀速时间
	double t1, t2;//中间变量

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	dDistance = pSPlanPointer->dCurLeftDist; //总距离
	dTc = pSPlanPointer->dTc;		//插补周期

	if (pSPlanPointer->cInterpDir != INP_GO_BACK)//正走
	{
		dDistance = pSPlanPointer->dCurLeftDist;//需要规划的距离
	}
	else//反走
	{
		dDistance = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//需要规划的距离
	}

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}

	dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//规划前已走

	//开始速度规划
	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3
			= pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0; //距离对应的时间节点置0
		pSPlanPointer->dDistAcc = 0;
    	pSPlanPointer->dDistDec = 0;
    	pSPlanPointer->dDistConst = 0;
    	pSPlanPointer->dDistRemaining = dDistance;
		pSPlanPointer->dPreDist = dPreDist;
		return 0;
	}
	if (dVe > dVmax)
	{
		dVe = dVmax;//限制末速度
	}

	//dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist;//规划前已走


	dAccFactor = getFactor(pSPlanPointer->ucAccVelPlanType);//根据加速曲线类型获取系数
	dDecFactor = getFactor(pSPlanPointer->ucDecVelPlanType);//根据减速曲线类型获取系数



	//计算加减速距离
	dSac = fabs(dAccFactor * (dVmax * dVmax - dVs * dVs) / dAcc);  //ps:减档减速,最大速度小于起始速度 加速段位移取绝对值
	dSde = fabs(dDecFactor * (dVmax * dVmax - dVe * dVe) / dDec);

	//     if(dDistance < dSac + dSde) //距离不够 重算最大速度 
	//     {  
	// 		//重算最大速度  
	//         dVmax = sqrt((4 * dAcc * dDec * dDistance  + 3 * dDec *  dVs * dVs + 3 * dAcc * dVe * dVe) / (3*(dAcc + dDec)));  
	// 
	//         dSac = fabs((3 * (dVmax * dVmax - dVs * dVs)) / (4 * dAcc));  //重算加速距离  
	// 		dSde = fabs((3 * (dVmax * dVmax - dVe * dVe)) / (4 * dDec));  //重算减速距离         
	//     }     


	if (dDistance < dSac + dSde) //距离不够 重算最大速度 
	{
		//double g_dVmaxPre = dVmax;

		//重算最大速度  
		dVmax = sqrt((dAcc * dDec * dDistance + dAccFactor * dDec * dVs * dVs + dDecFactor * dAcc * dVe * dVe) / (dDecFactor * dAcc + dAccFactor * dDec));
		if (dVmax > dVs && dVmax > dVe)//三角形速度曲线  2024/3/23
		{
			bTriangleToTrapezoid = TRUE;
		}
		dSac = fabs(dAccFactor * (dVmax * dVmax - dVs * dVs) / dAcc);  //重算加速距离 
		dSde = fabs(dDecFactor * (dVmax * dVmax - dVe * dVe) / dDec); //重算减速距离 
	}

	//计算时间增量
	dTac = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? dAccFactor * (dVmax - dVs) / (dAcc * dTc) : dAccFactor * 2.0 * (dVmax - dVs) / (dAcc * dTc);
	dTde = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_S3_CURVE) ? dDecFactor * (dVmax - dVe) / (dDec * dTc) : dDecFactor * 2.0 * (dVmax - dVe) / (dDec * dTc);
	dTconst = (isZero(dVmax)) ? 0 : (dDistance - (dSac + dSde)) / (dVmax * dTc);//计算匀速周期  最大速度为0时   匀速周期为0


	if (dTconst < 0 && !isZero(dTconst))   //没有匀速的情况    
	{
		if (dTac < 0 || dTde < 0)  //距离不足
		{
			if (!bUseFloatingVe)
			{
				return -2;//距离不足以加减速
			}
			//动态规划
			//Ve浮动
			if (dVs > dVe)//减速
			{
				dVe = sqrt(dVs * dVs - dDistance * dDec / dAccFactor);
				dVmax = dVs;
			}
			else//加速
			{
				dVe = sqrt(dVs * dVs + dDistance * dAcc / dAccFactor);
				dVmax = dVe;
			}
			//dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
			//dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期

			dTac = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? dAccFactor * (dVmax - dVs) / (dAcc * dTc) : dAccFactor * 2.0 * (dVmax - dVs) / (dAcc * dTc);
			dTde = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_S3_CURVE) ? dDecFactor * (dVmax - dVe) / (dDec * dTc) : dDecFactor * 2.0 * (dVmax - dVe) / (dDec * dTc);
		}
		dTconst = 0;
	}
	if (dTac < 0)//距离足够 但是加速周期为负，加速段变为减速段
	{
		dTac = fabs(dTac);//时间取绝对值
		dAcc = -dAcc;   //加速度取反
	}

	if (bTriangleToTrapezoid == TRUE && pSPlanPointer->ucTconstForTrapezoid != 0)//三角形速度曲线修正为梯形曲线：重算最大速度dVmax、加速度dAcc、减速度dDec 2024/3/22 洪冠鹏
	{
		if (pSPlanPointer->ucTconstForTrapezoid == 100)
		{
			//匀速时间占总时长的比例设为100时，企业未想好采用何种方案，暂时不进行处理，即不进行修调。
		}
		else
		{
			dTac = ceil(dTac);//向上取整  企业要求：改变后的曲线总时长不变，在修正前要获取三角形速度曲线下的实际时间。
			dTde = ceil(dTde);//向上取整

			//根据系统参数设置的匀速时间占总时长的比例，计算匀速时间
			dTconstForTrapezoid = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? pSPlanPointer->ucTconstForTrapezoid / 100.0 * (2.0 * dTac + 2.0 * dTde + dTconst) :
				pSPlanPointer->ucTconstForTrapezoid / 100.0 * (dTac + dTde + dTconst);
			t1 = (dTac / (dTac + dTde)) * dTconstForTrapezoid;//中间变量
			t2 = (dTde / (dTac + dTde)) * dTconstForTrapezoid;//中间变量

			//根据总位移不变原则，利用速度曲线围成的面积等于位移的原理，反算出梯形曲线下的最大速度。
			//1.S3型速度曲线的加减速段关于中间时刻点中心对称，其位移可用梯形曲线算出；
			//2.S4型速度曲线的加减速段没有对称型，但经推导，当S4型加速度等于1.5倍T型加速度，其位移也可用梯形曲线算出，该倍数关系体现在加减速度系数。
			dVmax = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? (2 * dDistance - (2.0 * dTac - t1) * dVs - (2.0 * dTde - t2) * dVe) / (2.0 * dTac + 2.0 * dTde + dTconstForTrapezoid) :
				(2 * dDistance - (dTac - t1) * dVs - (dTde - t2) * dVe) / (dTac + dTde + dTconstForTrapezoid);

			dAcc = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? dAccFactor * (dVmax - dVs) / ((dTac - 0.5 * t1) * dTc) : dAccFactor * 2.0 * (dVmax - dVs) / ((dTac - t1) * dTc);//重算加速度
			dDec = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_S3_CURVE) ? dDecFactor * (dVmax - dVe) / ((dTde - 0.5 * t2) * dTc) : dDecFactor * 2.0 * (dVmax - dVe) / ((dTde - t2) * dTc);//重算减速度
			dTac = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? dAccFactor * (dVmax - dVs) / (dAcc * dTc) : dAccFactor * 2.0 * (dVmax - dVs) / (dAcc * dTc);//重算加速周期
			dTde = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_S3_CURVE) ? dDecFactor * (dVmax - dVe) / (dDec * dTc) : dDecFactor * 2.0 * (dVmax - dVe) / (dDec * dTc);//重算减速周期
		}
	}
	if (isZero(dTconst))	//陈敏聪	2023/7/29	判断是否无匀速过程，只有加减速过程，且加减速周期总和小于一个插补周期
	{
		if ((dTac + dTde < dTc) && dTac > 0 && dTde > 0 &&  !isZero(dVmax))//!isZero(dVmax)避免音手轮模拟转动太小时，出现最大速度异常大
		{
			dVmax = dDistance / dTc;	//最大速度设为插补距离/插补周期，初速度和末速度在下方的代码因为加减速周期为0设为和最大速度一样的速度
			dTac = dTde = 0;			//加减速周期数设为0
			dTconst = 1;				//匀速段周期数设为1
		}
	}

	dTac = ceil(dTac);//向上取整
	dTde = ceil(dTde);

	// 	//旧方案：加速时间圆整后，需重算最大速度，由于减速时间固定，此时要重算减速度   
	//     dVmax = 2.0 * dAcc * (double)dTac * dTc / 3.0 + dVs; //重算最大速度
		//新方案：加速时间为0时 起始速度等于最大速度   加速时间不为0时，重算加速度
	if (isZero(dTac))
	{
		dVs = dVmax;
	}
	else
	{
		dAcc = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE) ? dAccFactor * (dVmax - dVs) / (dTac * dTc) : dAccFactor * 2.0 * (dVmax - dVs) / (dTac * dTc);
	}


	if (isZero(dTde))  //减速周期为0，修整最终速度
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{
		dDec = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_S3_CURVE) ? dDecFactor * (dVmax - dVe) / (dTde * dTc) : dDecFactor * 2.0 * (dVmax - dVe) / (dTde * dTc); //重算减速度
	}

	//重算加减速距离  
	dSac = fabs(dAccFactor * (dVmax * dVmax - dVs * dVs) / dAcc);  //重算加速距离 
	dSde = fabs(dDecFactor * (dVmax * dVmax - dVe * dVe) / dDec); //重算减速距离 

	dTconst = (isZero(dVmax)) ? 0 : (dDistance - dSac - dSde) / (dVmax * dTc); //计算匀速周期    最大速度为0时  匀速周期 为0

	dTconst = (int32)(dTconst + 0.5);//匀速周期取整
	if (dTconst < 0)  // 匀速周期为负时，取0
	{
		dTconst = 0;
	}

	//时间取整后，计算时间节点（绝对时间）  
	if (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_S3_CURVE)//加速
	{
		unT1 = (int32)dTac;
		unT2 = 2 * unT1;
	}
	else
	{
		unT1 = unT2 = (int32)dTac;
	}

	unT3 = unT2 + (int32)dTconst; //匀速

	if (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_S3_CURVE)//减速
	{
		unT4 = unT3 + (int32)dTde;
		unT5 = unT4 + (int32)dTde;
	}
	else
	{
		unT4 = unT5 = unT3 + (int32)dTde;
	}


	dSconst = dVmax * (int)dTconst * dTc; //计算匀速距离  

	dDistRemain = dDistance - dSac - dSde - dSconst; //残余距离 

	//传出参数
	pSPlanPointer->unT1 = unT1;		//时间节点T1,2,3,4,5
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;

	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离
	pSPlanPointer->dPreDist = dPreDist;

#if EN_DEBUG
	printf("混合速度规划\n");
	printf("**************************\n");
	printf("k_acc = %lf\r\n", dTac);
	printf("k_const = %lf\r\n", dTconst);
	printf("k_dec = %lf\r\n", dTde);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("nT4 = %d\r\n", unT4);
	printf("nT5 = %d\r\n", unT5);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dAcc = %20.16lf\r\n", dAcc);
	printf("dDec = %20.16lf\r\n", dDec);
	printf("dDe  = %20.16lf\r\n", dDistRemain);
	printf("dVs = %20.16lf\r\n", dVs);
	printf("dVe = %20.16lf\r\n", dVe);
	printf("dVmax = %20.16lf\r\n", dVmax);
	printf("**************************\n");
#endif
	return (isZero(dVmax)) ? 2 : 1;
}

extern FILE* pWriteFid;

/*************************************************
  函数名称：preparePlanningTNewAdd(struct SPlan *pSPlanPointer)
  函数功能  新增距离模式的T型速度规划  与普通模式的速度规划不同  该模式在距离不为0时规划一定成功
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值： 2  随动阶段
			 1  加速阶段
			 0	距离为0/负
			-1	输入错误
  备    注：
**************************************************/
int8 preparePlanningTNewAdd(struct SPlan* pSPlanPointer)
{
	int8 res = 1;
	//不需初始化的参数
	double dTac, dTconst, dTde;//加速,匀速,减速周期
	uint32 unT1, unT2, unT3, unT4, unT5;   //加速,匀速,减速周期
	double dSac, dSconst, dSde;//加速，匀速，减速距离
	uint8 flag = FALSE;//输入是否为最大速度
	//需要初始化的参数
	double dVs, dVmax, dVe;    //起始，最大，终点速度
	double dAcc, dDec;        //加速段，减速段的减速度
	double dDistance, dDistRemain;        //总距离,残余距离
	double dTc;				 //插补周期

	struct _SG32* pG32 = &g_SChannel[0].SG32;


	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化参数(从结构体获取）
	dVs = pSPlanPointer->dVs;		//起始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度
	dVe = pSPlanPointer->dVe;		//最终速度
	dAcc = pSPlanPointer->dAcc;		//加速度
	dDec = pSPlanPointer->dDec;		//减速度
	//dDistance	= pSPlanPointer->dDistance; //总距离
	dDistance = pSPlanPointer->dCurLeftDist; //对剩余距离进行规划
	dTc = pSPlanPointer->dTc;		//插补周期

	if (dAcc <= 0 || dDec <= 0)	//增加校验
	{
		return -1;
	}


#if !ENCODER_MODE
	if (pG32->bPrintFlag == TRUE)
	{
#if !MATLAB_DEBUG
		fprintf(pWriteFid, " %lf %lf %lf %lf ", dVs, dVmax, dVe, dDistance);
#endif
	}
#endif
	//函数运行开始
	if (isZero(dVmax - g_SSystemPara.dSysMaxVel))
	{//输入的最大速度为系统参数的最大速度
		//作用：区分   追赶、随动阶段  与  减速阶段，不同阶段使用不同的取整方式(Tconst)，重算的对象(Ve, Dec)也不同
		flag = TRUE;
	}

	if (dDistance < 0)//距离为负 置0忽略后返回 洪冠鹏 2024/5/6
	{
		dDistance = 0;
	}


	if (isZero(dDistance))//距离太短 忽略
	{
		pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3 = 0; //距离对应的时间节点置0
		pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0;
#if !ENCODER_MODE
#if !MATLAB_DEBUG
		if (pG32->bPrintFlag == TRUE)
		{
			fprintf(pWriteFid, " %lf ", dVmax);
			fprintf(pWriteFid, " %lf %lf", dAcc, dDec);

	}
#endif
#endif

		return 0;
}

	dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //计算加速段位移
	dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));   //计算减速段位移

	if (dDistance < dSac + dSde) //路程不足加减速
	{
		//重算最大速度
		dVmax = sqrt((2 * dDistance * dAcc * dDec + dVs * dVs * dDec + dVe * dVe * dAcc) / (dAcc + dDec));

#if !ENCODER_MODE
#if !MATLAB_DEBUG
		if (pG32->bPrintFlag == TRUE)
		{
			fprintf(pWriteFid, " %lf ", dVmax);
		}
#endif	
#endif

		dSac = fabs((dVmax * dVmax - dVs * dVs) / (dAcc * 2));   //重算加减速段位移
		dSde = fabs((dVmax * dVmax - dVe * dVe) / (dDec * 2));
#if EN_DEBUG
		printf("dDistance < dSac + dSde\r\n");
#endif
	}
	else
	{
#if !ENCODER_MODE
#if !MATLAB_DEBUG
		if (pG32->bPrintFlag == TRUE)
		{
			fprintf(pWriteFid, " %lf ", dVmax);
		}
#endif
#endif

	}


	dTac = (dVmax - dVs) / (dAcc * dTc);    //计算加速周期
	dTde = (dVmax - dVe) / (dDec * dTc);    //计算减速周期
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
	if (isZero(dTde) && isZero(dTac) && dVmax * dTc >= dDistance)
	{
		res = 2;//随动阶段
	}
	// 	dTac = (int)(dTac + 0.5);    //加速周期四舍五入取整  
	// 	dTde = (int)(dTde + 0.5);    //减速周期四舍五入取整


	dTac = ceil(dTac);//向上取整
	dTde = ceil(dTde);


	if (isZero(dTac) || isZero(dVs - dVmax))//加速周期为0
	{
		dVs = dVmax;
		dTac = 0;
	}
	else
	{
		//		dAcc = (dVmax - dVs)/(dTac * dTc);//重算加速度		
		 //		dVmax = dAcc * (dTac * dTc) + dVs;//重算最大速度	
		if (1)
		{
			dVmax = dAcc * (dTac * dTc) + dVs;//重算最大速度
		}
		else
		{
			dAcc = (dVmax - dVs) / (dTac * dTc);//重算加速度
		}

	}

	if (dTde < 0 || isZero(dTde))  //减速周期为0 （前提是最大速度不会比终点速度小很多(>dDec)）   普通规划限制了末速度不超过最大速度  新增距离的规划没有限制  因此需要判断<=0
	{
		dVe = dVmax;  //最终速度等于最大速度
	}
	else
	{                                  //以免减速度重算为0，影响计算
//		dDec = (dVmax - dVe) / (dTde * dTc);      //重算减速段加速度（因为减速时间取整）		
//		dVe = dVmax - dDec* (dTde * dTc);
		if (flag)
		{
			dVe = dVmax - dDec * (dTde * dTc);
		}
		else
		{
			dDec = (dVmax - dVe) / (dTde * dTc);
		}
	}

	dSac = (dVmax * dVmax - dVs * dVs) / (dAcc * 2);   //加速段位移（时间取整后）
	dSde = (dVmax * dVmax - dVe * dVe) / (dDec * 2);   //减速段位移 

	dTconst = (dDistance - dSac - dSde) / (dVmax * dTc);   //计算匀速段周期

#if !ENCODER_MODE
#if !MATLAB_DEBUG
	if (pG32->bPrintFlag == TRUE)
	{
		fprintf(pWriteFid, " %lf %lf", dAcc, dDec);
	}
#endif
#endif


	if (dTconst <= 0)  //没有匀速周期
	{
		dTconst = 0;
	}

	if (flag || (isZero(dTac) && isZero(dTde)))
	{
		dTconst = ceil(dTconst);  //匀速周期取整
	}
	else
	{
		dTconst = (int)(dTconst + 0.5);  //匀速周期取整
	}


	unT1 = unT2 = (uint32)dTac;               //时间节点T1  （绝对时间） 
	unT3 = unT2 + (uint32)dTconst; //时间节点T2  （绝对时间） 
	unT4 = unT5 = unT3 + (uint32)dTde;       //时间节点T3  （绝对时间） 

	dSconst = dVmax * (unT3 - unT2) * dTc; //计算匀速段位移

	dDistRemain = dDistance - dSac - dSde - dSconst;  //计算残余距离

	//传出参数
	pSPlanPointer->unT1 = unT1;		//时间节点T1,2,3
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;

	pSPlanPointer->dDistAcc = dSac;			//加速,匀速,减速距离
	pSPlanPointer->dDistConst = dSconst;
	pSPlanPointer->dDistDec = dSde;

	pSPlanPointer->dVmax = dVmax;	//最大速度，终点速度
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dAcc = dAcc;		//加速度，减速度
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dDistRemaining = dDistRemain; //残余距离

#if EN_DEBUG
	printf("T形速度规划\n");
	printf("dTac = %lf\r\n", dTac);
	printf("dTconst = %lf\r\n", dTconst);
	printf("dTde = %lf\r\n", dTde);
	printf("dSac = %20.16lf\r\n", dSac);
	printf("dSconst = %20.16lf\r\n", dSconst);
	printf("dSde = %20.16lf\r\n", dSde);
	printf("dDe = %lf\r\n", dDistRemain);
	printf("nT1 = %d\r\n", unT1);
	printf("nT2 = %d\r\n", unT2);
	printf("nT3 = %d\r\n", unT3);
	printf("dVmax = %lf\r\n", dVmax);
#endif
	return res;

}


/*************************************************
  函数名称：prepareDistReAvgT(struct SPlan *pSPlanPointer)
  函数功能  将速度规划后剩余的插补余量,根据等腰梯形平摊
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：-1	输入错误
			1	平摊完成
  备    注：
**************************************************/
int8 prepareDistReAvgT(struct SPlan* pSPlanPointer, bool bAddCycleFlag)
{
	//不需初始化的参数
	double dFactor;		  //比例因子 用于判断是否增加周期
	double dDistNewAddT; ///<新增一个周期走的距离
	uint32 unDistReT1;	 ///<分摊余量的时间节点T1
	uint32 unDistReT2;  ///<分摊余量的时间节点T2
	uint32 unDistReT3;  ///<分摊余量的时间节点T3
	double dTrapezoidAvgAcc;   ///平摊梯形的加速度
	double dTrapezoidAvgVel;   ///平摊梯形的最大速度

	//需初始化的参数
	double dVe;			//终点速度
	double dDistRemain;	//残余距离
	double dTc;			//插补周期
	uint32 unSumT;		//总周期数（T型，S4型为T3，S3型为T5）

	if (pSPlanPointer == NULL)  //空指针  不运行
	{
		return -1;
	}

	//初始化
	dVe = pSPlanPointer->dVe;				//终点速度
	dDistRemain = pSPlanPointer->dDistRemaining;//残余距离
	dTc = pSPlanPointer->dTc;				//插补周期
	////总周期数（T型，S4型为T3，S3型为T5）	
	unSumT = pSPlanPointer->unT5;

	dFactor = (isZero(dVe)) ? 0.1 : dDistRemain / (dVe * dTc); //计算比例因子  判断是否增加周期

	if (unSumT <= 0)
	{
		return -1;
	}
	
	if (bAddCycleFlag)
	{
		if (dFactor >= 0.5)
		{ //增加一个周期
			unDistReT3 = unSumT + 1;
			dDistNewAddT = dVe * dTc;  //新增减速周期的位移
			dDistRemain -= dVe * dTc;      //更新残余距离
		}
		else
		{
			//不增加周期
			dDistNewAddT = 0;
			unDistReT3 = unSumT;
		}
	}
	else
	{
		//不增加周期
		dDistNewAddT = 0;
		unDistReT3 = unSumT;

	}

	unDistReT1 = unDistReT3 / 3;  //时间节点T1
	unDistReT2 = unDistReT3 - unDistReT1;        //时间节点T2
	unDistReT3 = unDistReT3;           //时间节点T3


	if (unDistReT1 <= 1)    //特殊情况：加减速周期很短 平摊到匀速部分
	{
		dTrapezoidAvgAcc = dDistRemain / (unDistReT3 - 2 * unDistReT1);
		dTrapezoidAvgVel = dDistRemain / (unDistReT3 - 2 * unDistReT1);
	}
	else
	{
		dTrapezoidAvgAcc = dDistRemain / ((unDistReT3 - unDistReT1) * (unDistReT1 - 1));  //计算平摊因子d(=g_dD)
		dTrapezoidAvgVel = (unDistReT1 - 1) * dTrapezoidAvgAcc;            //计算平摊的最大速度(=(n-1)d)
	}

	//传出参数
	pSPlanPointer->dDistNewAddT = dDistNewAddT;		///<新增一个周期走的距离
	pSPlanPointer->unDistReT1 = unDistReT1;		///<分摊余量的时间节点T1
	pSPlanPointer->unDistReT2 = unDistReT2;		///<分摊余量的时间节点T2
	pSPlanPointer->unDistReT3 = unDistReT3;		///<分摊余量的时间节点T3
	pSPlanPointer->dTrapezoidAvgAcc = dTrapezoidAvgAcc;	///<平摊梯形的加速度
	pSPlanPointer->dTrapezoidAvgVel = dTrapezoidAvgVel;	///<平摊梯形的最大速度
	pSPlanPointer->dDistRemaining = dDistRemain;		///<残余距离

#if EN_DEBUG
	printf("梯形平摊\n");
#endif

	return 1;
}



/*************************************************
  函数名称：prepareDistReAvgS5(struct SPlan *pSPlanPointer)
  函数功能  将速度规划后剩余的插补余量,根据五次多项式平摊
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：-1	输入错误
			1	平摊完成
  备    注：
**************************************************/
int8 prepareDistReAvgS5(struct SPlan* pSPlanPointer)
{
	//不需初始化的参数
	double dQuinticAvgA = 0;	///五次项系数A
	double dQuinticAvgB = 0;	///五次项系数B
	double dQuinticAvgC = 0;	///五次项系数C
	uint32 unDistReT3;      ///均摊的时间节点T3
	uint8 ucQuinticAvgTFlag; ///五次周期不足标志位
	double dTrapezoidAvgVel = 0;  //周期不足时， 平摊最大速度

	//需要初始化的参数
	double dDistRemain;	//残余距离
	uint32 unSumT;		//总周期数（T型，S4型为T3，S3型为T5）

	if (pSPlanPointer == NULL)   //空指针  不运行
	{
		return -1;
	}

	//初始化
	dDistRemain = pSPlanPointer->dDistRemaining; //残余距离
	////总周期数（T型，S4型为T3，S3型为T5）
	unSumT = pSPlanPointer->unT5;

	if (unSumT <= 0)
	{
		return -1;
	}
	

	if (unSumT > 6)  //周期数大于6
	{
		double dPeriod = unSumT - 2;  //第一个周期和最后一个周期不平摊

		dQuinticAvgA = 6 * dDistRemain / (dPeriod * dPeriod * dPeriod * dPeriod * dPeriod);  //计算五次项系数A的值
		dQuinticAvgB = -15 * dDistRemain / (dPeriod * dPeriod * dPeriod * dPeriod);   //计算四次项系数B的值
		dQuinticAvgC = 10 * dDistRemain / (dPeriod * dPeriod * dPeriod);     //计算三次项系数C的值
		ucQuinticAvgTFlag = 0; //周期数大于6  周期足够 标志位置0
	}
	else//周期数小于6  周期不足
	{
		dTrapezoidAvgVel = dDistRemain / unSumT;  //直接平摊
		ucQuinticAvgTFlag = 1;      //标志位置1
	}
	unDistReT3 = unSumT;  //改变平均余量的周期数

	//传出参数
	pSPlanPointer->dQuinticAvgA = dQuinticAvgA;	///五次项系数A
	pSPlanPointer->dQuinticAvgB = dQuinticAvgB;	///五次项系数B
	pSPlanPointer->dQuinticAvgC = dQuinticAvgC;	///五次项系数C
	pSPlanPointer->ucQuinticAvgTFlag = ucQuinticAvgTFlag;///五次周期不足标志位
	pSPlanPointer->unDistReT3 = unDistReT3;		///<分摊余量的时间节点T3
	pSPlanPointer->dTrapezoidAvgVel = dTrapezoidAvgVel; //周期不足时， 平摊最大速度

#if EN_DEBUG
	printf("五次多项式平摊\n");
#endif

	return 1;

}


/*************************************************
  函数名称：prepareDistReAvgRatio(struct SPlan *pSPlanPointer)
  函数功能  将速度规划后剩余的插补余量,按比例平摊
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：-1	输入错误
			1	平摊完成
  备    注：
**************************************************/
int8 prepareDistReAvgRatio(struct SPlan* pSPlanPointer)
{
	//不需要初始化的参数
	double dAverageFactor;	///<比例平摊的比例系数
	uint32 unDistReT3;      ///均摊的时间节点T3

	//需要初始化的参数
	double dSumDist;	//已速度规划的距离
	double dDistance;	///<插补距离	mm 
	uint32 unSumT;		//总周期数（T型，S4型为T3，S3型为T5）

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}
	///初始化
	//已规划的距离
	dSumDist = pSPlanPointer->dDistAcc + pSPlanPointer->dDistConst + pSPlanPointer->dDistDec;
	dDistance = pSPlanPointer->dDistance;   //线段长度
	////总周期数（T型，S4型为T3，S3型为T5）
	unSumT = pSPlanPointer->unT5;

	if(dSumDist <= 0 || isnan(dSumDist))
	{
		return -1;
	}

	dAverageFactor = dDistance / dSumDist; //计算比例系数
	unDistReT3 = unSumT; //改变平均余量的周期数

	//传出参数
	pSPlanPointer->unDistReT3 = unDistReT3;    ///均摊的时间节点T3
	pSPlanPointer->dAverageFactor = dAverageFactor;///<比例平摊的比例系数

#if EN_DEBUG
	printf("比例系数平摊\n");
#endif

	return 1;
}


/*************************************************
  函数名称：nextstepPlanningT(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  根据梯形速度规划预处理,和输入的时间节点unK,计算并返回该时间节点的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 绝对位移（相对起点）
			-1		输入错误
  备    注：
**************************************************/
double nextstepPlanningT(uint32 unK, struct SPlan* pSPlanPointer)   //T型S(t) 
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dDistAcc;		///<加速段位移
	double dDistConst;	    ///<匀速段位移
	double dDistDec;		///<减速段位移
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5

	double dDistNewAddT;	///<新增一个周期走的距离

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}
	//初始化
	dVs = pSPlanPointer->dVs;		//初始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度	
	dVe = pSPlanPointer->dVe;		//终点速度
	dDistAcc = pSPlanPointer->dDistAcc;	//加速段位移
	dDistConst = pSPlanPointer->dDistConst;//匀速段位移
	dDistDec = pSPlanPointer->dDistDec;	//减速段位移
	dAcc = pSPlanPointer->dAcc;		//加速段加速度	mm/ms2
	dDec = pSPlanPointer->dDec;		//减速段加速度	mm/ms2
	dTc = pSPlanPointer->dTc;		//插补周期
	unT1 = pSPlanPointer->unT1;		//时间节点T1
	unT2 = pSPlanPointer->unT2;		//时间节点T2
	unT3 = pSPlanPointer->unT3;		//时间节点T3
	unT4 = pSPlanPointer->unT4;		//时间节点T4
	unT5 = pSPlanPointer->unT5;		//时间节点T5
	dDistNewAddT = pSPlanPointer->dDistNewAddT;//新增一个周期走的距离

	if (unK == 0) //防止加速周期为0   输入unK又为0  输出为 -nan
	{
		return 0.0;
	}
	else if (unK <= unT1)   //加速  unT2 = unT1
	{
		return dVs * unK * dTc + dAcc * unK * dTc * unK * dTc / 2;
	}
	else if (unK <= unT3)   //匀速
	{

		return dDistAcc + dVmax * (unK - unT2) * dTc;
	}
	else if (unK <= unT5)   //减速 unT4 = unT5
	{
		int dDeltaK = (int)(unK - unT5);
		return dDistAcc + dDistConst + dDistDec - dDec * dDeltaK * dTc * dDeltaK * dTc / 2 + dVe * dDeltaK * dTc;

	}
	else   //超出范围
	{
		return dDistAcc + dDistConst + dDistDec + dDistNewAddT;
	}
}


/*************************************************
  函数名称：nextstepPlanningS3(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  根据S3型速度规划预处理,和输入的时间节点unK,计算并返回该时间节点的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 绝对位移（相对起点）
			-1		输入错误
  备    注：
**************************************************/
double nextstepPlanningS3(uint32 unK, struct SPlan* pSPlanPointer)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dDistAcc;		///<加速段位移
	double dDistConst;	    ///<匀速段位移
	double dDistDec;		///<减速段位移
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5
	double dDistNewAddT;	///<新增一个周期走的距离

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	dVs = pSPlanPointer->dVs;		//初始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度	
	dVe = pSPlanPointer->dVe;		//终点速度
	dDistAcc = pSPlanPointer->dDistAcc;	//加速段位移
	dDistConst = pSPlanPointer->dDistConst;//匀速段位移
	dDistDec = pSPlanPointer->dDistDec;	//减速段位移
	dAcc = pSPlanPointer->dAcc;		//加速段加速度	mm/ms2
	dDec = pSPlanPointer->dDec;		//减速段加速度	mm/ms2
	dTc = pSPlanPointer->dTc;		//插补周期
	unT1 = pSPlanPointer->unT1;		//时间节点T1
	unT2 = pSPlanPointer->unT2;		//时间节点T2
	unT3 = pSPlanPointer->unT3;		//时间节点T3
	unT4 = pSPlanPointer->unT4;		//时间节点T4
	unT5 = pSPlanPointer->unT5;		//时间节点T5
	dDistNewAddT = pSPlanPointer->dDistNewAddT;//新增一个周期走的距离

	if (unK == 0) //防止加速周期为0   输入unK又为0  输出为 -nan
	{
		return 0.0;
	}
	else if (unK <= unT1) //加速1
	{
		return dAcc * dAcc * unK * dTc * unK * dTc * unK * dTc / (6 * (dVmax - dVs)) + dVs * unK * dTc;  //S型速度曲线 S(t) = t3 
	}
	else if (unK <= unT2)  //加速2
	{
		int dDeltaK = (int)(unK - unT2); //用double会输出错误
		return dAcc * dAcc * dDeltaK * dTc * dDeltaK * dTc * dDeltaK * dTc / (6 * (dVs - dVmax)) + dVmax * (unK - unT1) * dTc + dVs * unT1 * dTc;
	}
	else if (unK <= unT3)  //匀速
	{
		return dDistAcc + dVmax * (unK - unT2) * dTc;
	}
	else if (unK <= unT4)  //减速1
	{
		int dDeltaK = (int)(unK - unT3);
		return dDistAcc + dDec * dDec * dDeltaK * dTc * dDeltaK * dTc * dDeltaK * dTc / (6 * (dVe - dVmax)) + dVmax * (unK - unT2) * dTc;
	}
	else if (unK <= unT5)   //减速2
	{
		int dDeltaK = (int)(unK - unT5);
		return dDistAcc + dVmax * (unT4 - unT2) * dTc
			+ dDec * dDec * dDeltaK * dTc * dDeltaK * dTc * dDeltaK * dTc / (6 * (dVmax - dVe))
			+ dVe * (unK - unT4) * dTc;
	}
	else    //超出范围
	{
		return dDistAcc + dDistConst + dDistDec + dDistNewAddT;
	}

}



/*************************************************
  函数名称：nextstepPlanningS4(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  根据S4型速度规划预处理,和输入的时间节点unK,计算并返回该时间节点的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 绝对位移（相对起点）
			-1		输入错误
  备    注：
**************************************************/
double nextstepPlanningS4(uint32 unK, struct SPlan* pSPlanPointer)  //S4型S(t)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dDistAcc;		///<加速段位移
	double dDistConst;	    ///<匀速段位移
	double dDistDec;		///<减速段位移
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5
	double dDistNewAddT;	///<新增一个周期走的距离

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	dVs = pSPlanPointer->dVs;		//初始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度	
	dVe = pSPlanPointer->dVe;		//终点速度
	dDistAcc = pSPlanPointer->dDistAcc;	//加速段位移
	dDistConst = pSPlanPointer->dDistConst;//匀速段位移
	dDistDec = pSPlanPointer->dDistDec;	//减速段位移
	dAcc = pSPlanPointer->dAcc;		//加速段加速度	mm/ms2
	dDec = pSPlanPointer->dDec;		//减速段加速度	mm/ms2
	dTc = pSPlanPointer->dTc;		//插补周期
	unT1 = pSPlanPointer->unT1;		//时间节点T1
	unT2 = pSPlanPointer->unT2;		//时间节点T2
	unT3 = pSPlanPointer->unT3;		//时间节点T3
	unT4 = pSPlanPointer->unT4;		//时间节点T4
	unT5 = pSPlanPointer->unT5;		//时间节点T5

	dDistNewAddT = pSPlanPointer->dDistNewAddT;//新增一个周期走的距离

	if (unK == 0) //防止加速周期为0   输入unK又为0  输出为 -nan
	{
		return 0.0;
	}
	else if (unK <= unT1) // 加速  unT2=unT1 //2025.10.11 修改 (3 * unT1 * unT1)为(3.0 * unT1 * unT1)，若unT1过大可能超出整形范围，导致溢出
	{
		return -dAcc / (3.0 * unT1 * unT1) * unK * unK * unK * unK * dTc * dTc + 2 * dAcc / (3 * unT1) * unK * unK * unK * dTc * dTc + dVs * unK * dTc;
	}
	else if (unK <= unT3) //匀速  
	{
		return dDistAcc + dVmax * (unK - unT1) * dTc;

	}
	else if (unK <= unT5) //减速  unT4=unT5
	{
		double dDeltaK = (double)(unK - unT3);
		// 		printf("%lf ",dDec / (3 * (unT5 - unT3) * (unT5 - unT3)) * dDeltaK * dDeltaK * dDeltaK * dDeltaK * dTc * dTc 
		// 			- 2 * dDec / (3 * (unT5 - unT3)) * dDeltaK * dDeltaK * dDeltaK * dTc * dTc
		// 			+ dVmax * dDeltaK * dTc + dDistAcc + dDistConst);
		return dDec / (3.0 * (unT5 - unT3) * (unT5 - unT3)) * dDeltaK * dDeltaK * dDeltaK * dDeltaK * dTc * dTc
			- 2 * dDec / (3.0 * (unT5 - unT3)) * dDeltaK * dDeltaK * dDeltaK * dTc * dTc
			+ dVmax * dDeltaK * dTc + dDistAcc + dDistConst;
	}
	else //超出范围  
	{
		return dDistAcc + dDistConst + dDistDec + dDistNewAddT;
	}
}



/*************************************************
  函数名称：nextstepPlanningCombine(uint32 unK,struct SPlan *pSPlanPointer)  //混合型S(t)
  函数功能  根据混合型速度规划预处理,和输入的时间节点unK,计算并返回该时间节点的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 绝对位移（相对起点）
			-1		输入错误
  备    注：
**************************************************/
double nextstepPlanningCombine(uint32 unK, struct SPlan* pSPlanPointer)  //混合型S(t)
{
	uint8 ucVelPlanType = 0;//曲线类型
	//unT3为分界线
	ucVelPlanType = (pSPlanPointer->unCurrentT <= pSPlanPointer->unT3) ? pSPlanPointer->ucAccVelPlanType : pSPlanPointer->ucDecVelPlanType;

	switch (ucVelPlanType)//根据当前的曲线类型调用距离函数
	{
	case VEL_PLAN_T_CURVE://T型
		return nextstepPlanningT(unK, pSPlanPointer);
		break;
	case VEL_PLAN_S3_CURVE://S3型
		return nextstepPlanningS3(unK, pSPlanPointer);
		break;
	case VEL_PLAN_S4_CURVE://S4型
		return nextstepPlanningS4(unK, pSPlanPointer);
		break;
	default:
		return nextstepPlanningS4(unK, pSPlanPointer);
	}

}



/*************************************************
  函数名称：nextstepDistReAvgT(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  根据梯形平摊预处理,和输入的时间节点unK,计算并返回该时间节点的梯形平摊的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 平摊的绝对位移
			-1		输入错误
  备    注：
**************************************************/
double nextstepDistReAvgT(uint32 unK, struct SPlan* pSPlanPointer)
{
	uint32 unDistReT1, unDistReT2, unDistReT3;   //平摊时间节点T1,T2,T3
	double dTrapezoidAvgAcc;   ///平摊梯形的加速度
	double dTrapezoidAvgVel;   ///平摊梯形的最大速度
	double dDistRemaining;		///<残余距离

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	unDistReT1 = pSPlanPointer->unDistReT1;		//平摊时间节点T1
	unDistReT2 = pSPlanPointer->unDistReT2;		//平摊时间节点T2
	unDistReT3 = pSPlanPointer->unDistReT3;		//平摊时间节点T3
	dTrapezoidAvgAcc = pSPlanPointer->dTrapezoidAvgAcc;	///平摊梯形的加速度
	dTrapezoidAvgVel = pSPlanPointer->dTrapezoidAvgVel;	///平摊梯形的最大速度
	dDistRemaining = pSPlanPointer->dDistRemaining;	///<残余距离


	if (unK <= unDistReT1)   //加速
	{
		return dTrapezoidAvgAcc * (unK - 1) * unK / 2;
	}
	else if (unK <= unDistReT2)  //匀速
	{
		return dTrapezoidAvgAcc * (unDistReT1 - 1) * unDistReT1 / 2 + dTrapezoidAvgVel * (unK - unDistReT1);
	}
	else if (unK <= unDistReT3)   //减速
	{
		return dDistRemaining - dTrapezoidAvgAcc * (unDistReT3 - unK) * (unDistReT3 - unK - 1) / 2;
	}
	else   //超出范围
	{
		return dDistRemaining;
	}
}


/*************************************************
  函数名称：nextstepDistReAvgS5(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  根据五次多项式平摊预处理,和输入的时间节点unK,计算并返回该时间节点的五次多项式平摊的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 平摊的绝对位移
			-1		输入错误
  备    注：
**************************************************/
double nextstepDistReAvgS5(uint32 unK, struct SPlan* pSPlanPointer)
{
	uint32 unDistReT3;   //平摊时间节点T3
	double dQuinticAvgA;	///五次项系数A
	double dQuinticAvgB;	///五次项系数B
	double dQuinticAvgC;	///五次项系数C
	double dTrapezoidAvgVel;//周期不足时， 平摊最大速度
	uint8 ucQuinticAvgTFlag; ///五次周期不足标志位
	double dDistRemaining;		///<残余距离

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	unDistReT3 = pSPlanPointer->unDistReT3;		//平摊时间节点T3
	dQuinticAvgA = pSPlanPointer->dQuinticAvgA;	//////五次项系数A
	dQuinticAvgB = pSPlanPointer->dQuinticAvgB;	//////五次项系数B
	dQuinticAvgC = pSPlanPointer->dQuinticAvgC;	//////五次项系数C
	dTrapezoidAvgVel = pSPlanPointer->dTrapezoidAvgVel;	//周期不足时， 平摊最大速度
	ucQuinticAvgTFlag = pSPlanPointer->ucQuinticAvgTFlag;	//五次周期不足标志位
	dDistRemaining = pSPlanPointer->dDistRemaining;	///<残余距离




	if (ucQuinticAvgTFlag)  //直接平摊
	{
		return unK * dTrapezoidAvgVel;
	}
	else   //五次多项式平摊
	{
		if (unK <= 0)  //第一个周期不平摊
		{
			return 0.0;
		}
		else if (unK <= (unDistReT3 - 1))  //五次多项式平摊
		{
			return 6 * dDistRemaining * pow(unK, 5) / pow(unDistReT3 - 2, 5)
				- 15 * dDistRemaining * pow(unK, 4) / pow(unDistReT3 - 2, 4)
				+ 10 * dDistRemaining * pow(unK, 3) / pow(unDistReT3 - 2, 3);
		}
		else
		{
			return dDistRemaining;
		}
	}
}



/*************************************************
  函数名称：preparePlanningAndDistReAvg(SPlan *pSPlanPointer)
  函数功能  根据ucVelPlanType和ucDistReAverageType的值调用不同的速度规划和平摊预处理函数
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：*pSPlanPointer，结构体struct SPlan指针
  返 回 值：
			-2  距离不足以加减速
			-1	输入错误
			1	规划&平摊完成
  备    注：
**************************************************/
int8 preparePlanningAndDistReAvg(struct SPlan* pSPlanPointer, bool bUseFloatingVe)
{
	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	int8 res = preparePlanningCombine(pSPlanPointer, bUseFloatingVe);
	int8 cReAvgRes;
	if (res == 1)//混合速度规划   (2:输入最大速度为0   -2:距离不足以加减速  0:距离太短,忽略，同时清空平摊 -1:输入错误)
	{
		switch (pSPlanPointer->ucDistReAverageType)//根据预设的DEAL_DE_TYPE值调用不同的平摊预处理函数
		{
		case 1:
			cReAvgRes = prepareDistReAvgT(pSPlanPointer, TRUE);//梯形平摊
			break;
		case 2:
			cReAvgRes = prepareDistReAvgS5(pSPlanPointer);	//5次多项式平摊
			break;
		case 3:
			cReAvgRes = prepareDistReAvgRatio(pSPlanPointer);//比例系数平摊
			break;
		default:
			cReAvgRes = prepareDistReAvgT(pSPlanPointer, TRUE);//Default:梯形平摊
		}
		if (cReAvgRes == -1) //平摊错误
		{
			clearDistReAvgPara(pSPlanPointer);	
			return -1;
		}
	}
	else // ==2  最大速度vmax为0  不平摊 清空平摊参数（倍率为0时出现）
	{
		if (res == -2)
		{
			return -2;
		}
		clearDistReAvgPara(pSPlanPointer);
	}


	////赋值完成
	//pSPlanPointer->ucInpStatus = INP_RUNNING;  //赋值完成  运行

	return 1;
}



/*************************************************
  函数名称：nextstepPlanningAndDistReAvg(uint32 unK, struct SPlan *pSPlanPointer)
  函数功能  根据ucVelPlanType和ucDistReAverageType的值调用不同的位移函数,获得规划好、平摊后的绝对位移S(t)。
			输出的距离由目标距离限制，即输出不会超过目标距离
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 绝对位移S(t)
			0		输入错误或距离为0
  备    注：
**************************************************/
double nextstepPlanningAndDistReAvg(uint32 unK, struct SPlan* pSPlanPointer)
{
	double dDistance = 0; //速度规划走的距离
	double dPreDist = 0;//规划前已走的距离
	double dTotalDist = 0;
	uint8 ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数
	double dAverageFactor;		///<比例平摊的比例系数

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return 0;
	}
	if (isZero(pSPlanPointer->dDistance))
	{
		//pSPlanPointer->ucInpStatus = INP_FINISHED;
		return 0;
	}

	//初始化
	ucDistReAverageType = pSPlanPointer->ucDistReAverageType;	//平摊曲线类型，1：梯形，2：5次，3：比例系数
	dAverageFactor = pSPlanPointer->dAverageFactor;		///<比例平摊的比例系数
	dPreDist = pSPlanPointer->dPreDist;

	dDistance = nextstepPlanningCombine(unK, pSPlanPointer);


	switch (ucDistReAverageType)//根据预设的DEAL_DE_TYPE值,处理速度规划后的位移
	{
	case 1:
		dDistance += nextstepDistReAvgT(unK, pSPlanPointer);//梯形均摊后的位移S(t)
		break;
	case 2:
		dDistance += nextstepDistReAvgS5(unK, pSPlanPointer);//五次多项式均摊后的位移S(t)
		break;
	case 3:
		dDistance *= dAverageFactor;//按比例均摊后的位移S(t)
		break;
	default:
		dDistance += nextstepDistReAvgT(unK, pSPlanPointer);//Default:梯形均摊后的位移S(t)
	}

	dTotalDist = dPreDist;//不算规划  已走的距离
	dTotalDist += (pSPlanPointer->cInterpDir == INP_GO_BACK) ? (-dDistance) : dDistance;//根据方向计算距离

	if ((fabs(dTotalDist) > 1e15 || isnan(dTotalDist) || isinf(dTotalDist))) //异常值防护
	{
		return 0;  
	}

	if (dTotalDist > pSPlanPointer->dTargetDist)//目标距离限制输出
	{
		dTotalDist = pSPlanPointer->dTargetDist;
	}
	if (dTotalDist < 0)//负数限制
	{
		dTotalDist = 0;
	}
	return dTotalDist;//返回均摊后的总位移
}


/*************************************************
  函数名称：preparePlanningV3(struct SPlan *pSPlanPointer)
  函数功能  V关于t的三次方速度规划  （只考虑加速 或减速）
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：1		预处理完成
			-1		输入错误
  备    注：只加速 或 只减速
			使用前spindle的单位需要统一  如果为G96则化为mm/min   如果为G97则化为r/min
**************************************************/
int8 preparePlanningV3(struct SPlan* pSPlanPointer)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unCalTempK;
	uint32 unAccTime = 0, unDecTime = 0;
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5

	if (pSPlanPointer == NULL)//空指针  不运行
	{
		return -1;
	}

	//读出数据
	dVs = pSPlanPointer->dVs;
	dVmax = pSPlanPointer->dVmax;
	dVe = pSPlanPointer->dVe;
	dDec = pSPlanPointer->dDec;
	dAcc = pSPlanPointer->dAcc;
	dTc = pSPlanPointer->dTc;

	//符号相反 曲线为 减速 匀速 反向加速   （匀速段速度为0
	if ((fabs(dVs) + fabs(dVe)) > fabs(dVs + dVe))
	{
		unAccTime = (uint32)ceil(fabs(3 * (0.0 - dVs) / (2 * dAcc * dTc)));//计算到0的时间

		unDecTime = (uint32)ceil(fabs(3 * (dVe - 0.0) / (2 * dDec * dTc)));//计算到Ve的时间

		if (unAccTime != 0)
		{
			dAcc = 3 * (0.0 - dVs) / (2 * unAccTime * dTc);	//反算加速度
		}

		if (unDecTime != 0)
		{
			dDec = 3 * (0.0 - dVe) / (2 * unDecTime * dTc);	//反算加速度
		}
		unT1 = unT2 = unAccTime;
		unT3 = unAccTime + unDecTime;
		dVmax = 0;
	}
	else if (dVs < dVe) //同向加速
	{
		unCalTempK = (uint32)ceil(3 * (dVe - dVs) / (2 * dAcc * dTc));//计算节点
		if (unCalTempK != 0)
		{
			dAcc = 3 * (dVe - dVs) / (2 * unCalTempK * dTc);	//反算加速度
		}
		dVmax = dVe;//只有加速段 最大速度即末速度
		unT1 = unT2 = unT3 = unCalTempK;//只有加速段节点T1 2 3 重合
	}
	else   //同向减速
	{
		unCalTempK = (uint32)ceil(3 * (dVs - dVe) / (2 * dDec * dTc));//计算节点
		if (unCalTempK != 0)
		{
			dDec = 3 * (dVs - dVe) / (2 * unCalTempK * dTc);	//反算加速度
		}
		dVmax = dVs;//只有减速段  最大速度即起始速度
		unT1 = unT2 = 0;//只有减速段T1 T2为0
		unT3 = unCalTempK + unT2;
	}

	//写入数据
	pSPlanPointer->dVs = dVs;
	pSPlanPointer->dVmax = dVmax;
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dAcc = dAcc;
	pSPlanPointer->unT1 = unT1;
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;

#if EN_DEBUG
	printf("Planning V3\n");
	printf("dVs		=%10.16lf\n", dVs);
	printf("dVmax	=%10.16lf\n", dVmax);
	printf("dVe		=%10.16lf\n", dVe);
	printf("dDec	=%10.16lf\n", dDec);
	printf("dAcc	=%10.16lf\n", dAcc);
	printf("unT1	=%d\n", unT1);
	printf("unT2	=%d\n", unT2);
	printf("unT3	=%d\n", unT3);

#endif

	return 1;
}

/*************************************************
  函数名称：preparePlanningVelCombine(struct SPlan *pSPlanPointer,double dVeerDeltaVMax)
  函数功能  Vel组合型速度规划   （只考虑加速 或减速）
  输入参数：*pSPlanPointer，结构体struct SPlan指针 ,dVeerDeltaVMax 单轴反向转速
  输出参数：
  返 回 值：1		预处理完成
			-1		输入错误
  备    注：只加速 或 只减速
			使用前spindle的单位需要统一  如果为G96则化为m/ms   如果为G97则化为r/ms
			(2.20 整理注释)
**************************************************/
int8 preparePlanningVelCombine(struct SPlan* pSPlanPointer,double dVeerDeltaVMax)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unAccTime = 0, unDecTime = 0;
	uint32 unT1 = 0, unT2 = 0, unT3 = 0, unT4 = 0, unT5 = 0;	///<时间节点T1
	double dAccFactor = 0, dDecFactor = 0;//系数

	if (pSPlanPointer == NULL)//空指针  不运行
	{
		return -1;
	}

	//读出数据
	dVs = pSPlanPointer->dVs;
	dVmax = pSPlanPointer->dVmax;
	dVe = pSPlanPointer->dVe;
	dDec = pSPlanPointer->dDec;
	dAcc = pSPlanPointer->dAcc;
	dTc = pSPlanPointer->dTc;
	dAccFactor = getFactor(pSPlanPointer->ucAccVelPlanType);//根据加速曲线类型获取系数
	dDecFactor = getFactor(pSPlanPointer->ucDecVelPlanType);//根据减速曲线类型获取系数

	if (dAcc <= 0 || dDec <= 0)	//增加校验
	{
		return -1;
	}

	if (isZero((dVe - dVs) / dAcc) && isZero((dVs - dVe) / dDec))//速度相等  T = 1 时  直接输出 dVe
	{
		pSPlanPointer->unT1
			= pSPlanPointer->unT2
			= pSPlanPointer->unT3
			= pSPlanPointer->unT4
			= pSPlanPointer->unT5 = 0;
		return 1;
	}

	//符号相反 曲线为 减速 匀速 反向加速   
	if ((fabs(dVs) + fabs(dVe)) > fabs(dVs + dVe))
	{
		//调换加减速参数
		uint8 ucTempUc;
		double dTempDouble;

		// double dVeerDeltaVMax = min(g_SSystemPara.SAxisPara[AXIS_C ].dVeerDeltaVMax, fabs(dVs));//单轴反向转速 !!!
		dVeerDeltaVMax = min(dVeerDeltaVMax, fabs(dVs));	//修改2026.3.18,当函数被非C轴调用时，参数不一致 

		ucTempUc = pSPlanPointer->ucAccVelPlanType;//调转曲线类型
		pSPlanPointer->ucAccVelPlanType = pSPlanPointer->ucDecVelPlanType;
		pSPlanPointer->ucDecVelPlanType = ucTempUc;

		dTempDouble = dAcc;//调转加速度和减速度
		dAcc = dDec;
		dDec = dTempDouble;

		dAccFactor = getFactor(pSPlanPointer->ucAccVelPlanType);//根据加速曲线类型获取系数
		dDecFactor = getFactor(pSPlanPointer->ucDecVelPlanType);//根据减速曲线类型获取系数

		//加速周期
		unAccTime = (int32)ceil((pSPlanPointer->ucAccVelPlanType == VEL_PLAN_V2_CURVE) ? fabs(dAccFactor * (sgn(dVs) * dVeerDeltaVMax - dVs) / (dAcc * dTc)) : fabs(dAccFactor * 2.0 * (sgn(dVs) * dVeerDeltaVMax - dVs) / (dAcc * dTc)));
		//减速周期
		unDecTime = (int32)ceil((pSPlanPointer->ucDecVelPlanType == VEL_PLAN_V2_CURVE) ? fabs(dDecFactor * (dVe - sgn(dVe) * dVeerDeltaVMax) / (dDec * dTc)) : fabs(dDecFactor * 2.0 * (dVe - sgn(dVe) * dVeerDeltaVMax) / (dDec * dTc)));

		if (unAccTime != 0)
		{//重算加速度
			dAcc = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_V2_CURVE) ? dAccFactor * (sgn(dVs) * dVeerDeltaVMax - dVs) / (unAccTime * dTc) : dAccFactor * 2.0 * (sgn(dVs) * dVeerDeltaVMax - dVs) / (unAccTime * dTc);
		}

		if (unDecTime != 0)
		{//重算减速度
			dDec = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_V2_CURVE) ? dDecFactor * (sgn(dVe) * dVeerDeltaVMax - dVe) / (unDecTime * dTc) : dDecFactor * 2.0 * (sgn(dVe) * dVeerDeltaVMax - dVe) / (unDecTime * dTc);
		}

		//计算时间节点
		//加速时间节点
		if (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_V2_CURVE)
		{//V2(S3)型速度规划
			unT1 = unAccTime;
			unT2 = unT1 + unAccTime;
			unT3 = unT2 + 0;
			dVmax = dVeerDeltaVMax;
		}
		else
		{
			unT1 = unT2 = unT3 = unAccTime;
			dVmax = dVeerDeltaVMax;
		}

		//减速时间节点
		if (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_V2_CURVE)
		{//V2(S3)型速度规划
			unT4 = unT3 + unDecTime;
			unT5 = unT4 + unDecTime;
		}
		else
		{
			unT4 = unT5 = unT3 + unDecTime;
		}
	}
	else if (fabs(dVs) < fabs(dVe)) //同向加速
	{
		//加速周期
		unAccTime = (int32)ceil((pSPlanPointer->ucAccVelPlanType == VEL_PLAN_V2_CURVE) ? fabs(dAccFactor * (dVe - dVs) / (dAcc * dTc)) : fabs(dAccFactor * 2.0 * (dVe - dVs) / (dAcc * dTc)));

		if (unAccTime != 0)
		{//重算加速度
			dAcc = (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_V2_CURVE) ? dAccFactor * (dVe - dVs) / (unAccTime * dTc) : dAccFactor * 2.0 * (dVe - dVs) / (unAccTime * dTc);//重算加速度
		}

		dVmax = dVe;//只有加速段 最大速度即末速度

		//计算时间节点
		if (pSPlanPointer->ucAccVelPlanType == VEL_PLAN_V2_CURVE)
		{//V2(S3)型速度规划
			unT1 = unAccTime;
			unT2 = unT3 = unT4 = unT5 = unT1 + unAccTime;
		}
		else
		{
			unT1 = unT2 = unT3 = unT4 = unT5 = unAccTime;//只有加速段节点T1 2 3 4 5重合
		}

	}
	else   //同向减速
	{
		//减速周期
		unDecTime = (int32)ceil((pSPlanPointer->ucDecVelPlanType == VEL_PLAN_V2_CURVE) ? fabs(dDecFactor * (dVs - dVe) / (dDec * dTc)) : fabs(dDecFactor * 2.0 * (dVs - dVe) / (dDec * dTc)));

		if (unDecTime != 0)
		{//重算减速度
			dDec = (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_V2_CURVE) ? dDecFactor * (dVs - dVe) / (unDecTime * dTc) : dDecFactor * 2.0 * (dVs - dVe) / (unDecTime * dTc); //重算减速度
		}

		dVmax = dVs;//只有减速段  最大速度即起始速度

		//计算时间节点
		unT1 = unT2 = unT3 = 0;//只有减速段   T1 T2 T3为0
		if (pSPlanPointer->ucDecVelPlanType == VEL_PLAN_V2_CURVE)
		{//V2(S3)型速度规划
			unT4 = unDecTime + unT3;
			unT5 = unDecTime + unT4;
		}
		else
		{
			unT4 = unT5 = unDecTime + unT3;
		}

	}

	//写入数据
	pSPlanPointer->dVs = dVs;
	pSPlanPointer->dVmax = dVmax;
	pSPlanPointer->dVe = dVe;
	pSPlanPointer->dDec = dDec;
	pSPlanPointer->dAcc = dAcc;
	pSPlanPointer->unT1 = unT1;
	pSPlanPointer->unT2 = unT2;
	pSPlanPointer->unT3 = unT3;
	pSPlanPointer->unT4 = unT4;
	pSPlanPointer->unT5 = unT5;

#if EN_DEBUG
	printf("Planning V3\n");
	printf("dVs		=%10.16lf\n", dVs);
	printf("dVmax	=%10.16lf\n", dVmax);
	printf("dVe		=%10.16lf\n", dVe);
	printf("dDec	=%10.16lf\n", dDec);
	printf("dAcc	=%10.16lf\n", dAcc);
	printf("unT1	=%d\n", unT1);
	printf("unT2	=%d\n", unT2);
	printf("unT3	=%d\n", unT3);

#endif

	return 1;
}


/*************************************************
  函数名称：nextstepPlanningV1(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  V关于t的一次方速度规划  （只考虑加速 或减速）
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：1		预处理完成
			-1		输入错误
  备    注：只加速 或 只减速
			使用前spindle的单位需要统一  如果为G96则化为mm/min   如果为G97则化为r/min
**************************************************/
double nextstepPlanningV1(uint32 unK, struct SPlan* pSPlanPointer)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	dVs = pSPlanPointer->dVs;		//初始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度	
	dVe = pSPlanPointer->dVe;		//终点速度
	dAcc = pSPlanPointer->dAcc;		//加速段加速度	mm/ms2
	dDec = pSPlanPointer->dDec;		//减速段加速度	mm/ms2
	dTc = pSPlanPointer->dTc;		//插补周期
	unT1 = pSPlanPointer->unT1;		//时间节点T1
	unT2 = pSPlanPointer->unT2;		//时间节点T2
	unT3 = pSPlanPointer->unT3;		//时间节点T3
	unT4 = pSPlanPointer->unT4;		//时间节点T2
	unT5 = pSPlanPointer->unT5;		//时间节点T3

	if (unK == 0)
	{
		return dVs;
	}
	else if (unK <= unT1)//加速  
	{
		return dAcc * unK + dVs;
	}
	else if (unK <= unT3) //匀速  
	{
		return dVmax;
	}
	else if (unK <= unT5) //减速  
	{
		return dVe - (int)(unK - unT5) * dDec;
	}
	else //超出范围  
	{
		return dVe;
	}
}


/*************************************************
  函数名称：nextstepPlanningV2(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  V关于t的二次方速度规划  （只考虑加速 或减速）
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：1		预处理完成
			-1		输入错误
  备    注：只加速 或 只减速
			使用前spindle的单位需要统一  如果为G96则化为mm/min   如果为G97则化为r/min
**************************************************/
double nextstepPlanningV2(uint32 unK, struct SPlan* pSPlanPointer)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	dVs = pSPlanPointer->dVs;		//初始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度	
	dVe = pSPlanPointer->dVe;		//终点速度
	dAcc = pSPlanPointer->dAcc;		//加速段加速度	mm/ms2
	dDec = pSPlanPointer->dDec;		//减速段加速度	mm/ms2
	dTc = pSPlanPointer->dTc;		//插补周期
	unT1 = pSPlanPointer->unT1;		//时间节点T1
	unT2 = pSPlanPointer->unT2;		//时间节点T2
	unT3 = pSPlanPointer->unT3;		//时间节点T3
	unT4 = pSPlanPointer->unT4;		//时间节点T2
	unT5 = pSPlanPointer->unT5;		//时间节点T3

	if (unK == 0)
	{
		return dVs;
	}
	else if (unK <= unT1)//加速  
	{
		return dAcc * dAcc * unK * unK / (2.0 * (dVmax - dVs)) + dVs;
	}
	else if (unK <= unT2)//加速  
	{
		dVmax = sgn(dVs) == 0 ? dVmax : sgn(dVs) * fabs(dVmax);//根据Vs的符号判断Veer的符号
		return dAcc * dAcc * (int)(unK - unT2) * (int)(unK - unT2) / (2.0 * (dVs - dVmax)) + dVmax;
	}
	else if (unK <= unT3) //匀速  
	{
		return dVmax;
	}
	else if (unK <= unT4) //减速  
	{
		dVmax = sgn(dVe) == 0 ? dVmax : sgn(dVe) * fabs(dVmax); //根据Ve的符号判断Veer的符号
		return dVmax + dDec * dDec * (int)(unK - unT3) * (int)(unK - unT3) / (2.0 * (dVe - dVmax));
	}
	else if (unK <= unT5) //减速  
	{
		dVmax = sgn(dVe) == 0 ? dVmax : sgn(dVe) * fabs(dVmax);//根据Ve的符号判断Veer的符号
		return dVe + dDec * dDec * (int)(unK - unT5) * (int)(unK - unT5) / (2.0 * (dVmax - dVe));
	}
	else //超出范围  
	{
		return dVe;
	}
}

/*************************************************
  函数名称：nextstepPlanningV3(uint32 unK,struct SPlan *pSPlanPointer)
  函数功能  V关于t的三次方速度规划  （只考虑加速 或减速）
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：1		预处理完成
			-1		输入错误
  备    注：只加速 或 只减速
			使用前spindle的单位需要统一  如果为G96则化为mm/min   如果为G97则化为r/min
**************************************************/
double nextstepPlanningV3(uint32 unK, struct SPlan* pSPlanPointer)
{
	double dVs, dVmax, dVe;   //初始，匀速，终点速度
	double dAcc;		///<加速段加速度	mm/ms2
	double dDec;		///<减速段加速度	mm/ms2
	double dTc;			///<插补周期	ms
	uint32 unT1;	///<时间节点T1
	uint32 unT2;	///<时间节点T2
	uint32 unT3;	///<时间节点T3
	uint32 unT4;	///<时间节点T4
	uint32 unT5;	///<时间节点T5

	if (pSPlanPointer == NULL) //空指针 不运行
	{
		return -1;
	}

	//初始化
	dVs = pSPlanPointer->dVs;		//初始速度
	dVmax = pSPlanPointer->dVmax;		//最大速度	
	dVe = pSPlanPointer->dVe;		//终点速度
	dAcc = pSPlanPointer->dAcc;		//加速段加速度	mm/ms2
	dDec = pSPlanPointer->dDec;		//减速段加速度	mm/ms2
	dTc = pSPlanPointer->dTc;		//插补周期
	unT1 = pSPlanPointer->unT1;		//时间节点T1
	unT2 = pSPlanPointer->unT2;		//时间节点T2
	unT3 = pSPlanPointer->unT3;		//时间节点T3
	unT4 = pSPlanPointer->unT4;		//时间节点T2
	unT5 = pSPlanPointer->unT5;		//时间节点T3

	if (unK == 0)
	{
		return dVs;
	}
	else if (unK <= unT2) // 加速 //2025.10.11 修改 (3 * unT2 * unT2)为(3.0 * unT2 * unT2)，若unT2过大可能超出整形范围，导致溢出
	{
		return -4 * dAcc / (3.0 * unT2 * unT2) * unK * dTc * unK * dTc * unK * dTc + 2 * dAcc / unT2 * unK * dTc * unK * dTc + dVs;
	}
	else if (unK <= unT3) //匀速  
	{
		return dVmax;
	}
	else if (unK <= unT5) //减速  
	{
		dVmax = sgn(dVe) == 0 ? dVmax : sgn(dVe) * fabs(dVmax);//根据Ve的符号判断Veer的符号
		return dVmax + 4 * dDec / (3 * pow(unT5 - unT3, 2)) * pow((unK - unT3) * dTc, 3) - 2 * dDec / (unT5 - unT3) * (unK - unT3) * dTc * (unK - unT3) * dTc;
	}
	else //超出范围  
	{
		return dVe;
	}
}

/*************************************************
  函数名称：nextstepPlanningCombine(uint32 unK,struct SPlan *pSPlanPointer)  //混合型S(t)
  函数功能  根据混合型速度规划预处理,和输入的时间节点unK,计算并返回该时间节点的绝对位移s(t)
  输入参数：*pSPlanPointer，结构体struct SPlan指针
  输出参数：
  返 回 值：double 绝对位移（相对起点）
			-1		输入错误
  备    注：
**************************************************/
double nextstepPlanningVelCombine(uint32 unK, struct SPlan* pSPlanPointer)  //混合型V(t)
{
	uint8 ucVelPlanType = 0;//曲线类型
	//unT3为分界线
	ucVelPlanType = (pSPlanPointer->unCurrentT <= pSPlanPointer->unT3) ? pSPlanPointer->ucAccVelPlanType : pSPlanPointer->ucDecVelPlanType;

	switch (ucVelPlanType)//根据当前的曲线类型调用距离函数
	{
	case VEL_PLAN_V1_CURVE://T型
		return nextstepPlanningV1(unK, pSPlanPointer);
		break;
	case VEL_PLAN_V2_CURVE://S3型
		return nextstepPlanningV2(unK, pSPlanPointer);
		break;
	case VEL_PLAN_V3_CURVE://S4型
		return nextstepPlanningV3(unK, pSPlanPointer);
		break;
	default:
		return nextstepPlanningV3(unK, pSPlanPointer);
	}

}


#define  REPEATG32_NUM  4  //连续G32的个数(测试用)
//测试用全局变量
double g_dLongAxisPos[REPEATG32_NUM * 2];
double g_dShortAxisPos[REPEATG32_NUM * 2];
double g_dEncoderNum[REPEATG32_NUM * 2];
double g_dDeltaPulse[REPEATG32_NUM];
double g_dDeltaLongAxis[REPEATG32_NUM];
double g_dDeltaShortAxis[REPEATG32_NUM];
double g_dTime;
int    g_nI;
int    g_nDeltaI;
/*************************************************
  函数名称：newAddDistace(struct SPlan *pSPlanPointer,double newAddDist)
  函数功能  新增距离
  输入参数：pSPlanPointer  预处理结构体指针
			newAddDist	新增距离
			dAcc		加速度
			dDec		减速度
			dVe			随动的理想目标速度
  输出参数：
  返 回 值：0 加速阶段
			1 随动阶段  （随动阶段要是  新增距离!=当前速度  会进入加速(追赶)阶段)
			2 减速阶段
			3 完成阶段
  备    注：
**************************************************/
uint8 newAddDistace(struct _SG32* pG32, struct SPlan* pSPlanPointer, double newAddDist, double dAcc, double dDec, double dVe, double dCurrentSpeed)
{
	uint8 res = 0; //返回值
	double dMaxDistance;
	double dBackCalVmax;
	int8 cMotionState;

	pSPlanPointer->dDistance = fabs(pSPlanPointer->dDistance);                          //长轴的插补长度
	pSPlanPointer->dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist; //已插补长度

	//if (!isZero(pG32->dNextG32LongAxisSpeed) && !pG32->bDecPlanTransitionFlag)          //为非减速过渡的连续G32
	//{
	//	pSPlanPointer->dTargetDist = pSPlanPointer->dTargetDist + newAddDist;           //目标距离无总长度限制
	//}
	if ((!isZero(pG32->dNextG32LongAxisSpeed) && !pG32->bDecPlanTransitionFlag) ||
		(!isZero(pG32->dNextG32LongAxisSpeed - pG32->SUserData.dVe) && pG32->bDecPlanTransitionFlag))          //为非换向过渡的连续G32
	{
		pSPlanPointer->dTargetDist = pSPlanPointer->dTargetDist + newAddDist;           //目标距离无总长度限制
	}
	else
	{
		pSPlanPointer->dTargetDist = min(pSPlanPointer->dDistance, pSPlanPointer->dTargetDist + newAddDist);//确定目标距离  (总长度对其限制)
	}
	pSPlanPointer->dCurLeftDist = pSPlanPointer->dTargetDist - pSPlanPointer->dPreDist; //计算剩余距离（规划用)
	dMaxDistance = pSPlanPointer->dDistance - pSPlanPointer->dPreDist;                  //距终点的距离

	if (dMaxDistance < 0)
	{
		dMaxDistance = 0;
	}

	//2赋新值
	pSPlanPointer->dAcc = 2.0 * dAcc;// 加速度(2倍)
	pSPlanPointer->dDec = 2.0 * dDec;// 加速度(2倍)
	pSPlanPointer->ucAccVelPlanType = VEL_PLAN_T_CURVE;       //加速曲线类型
	pSPlanPointer->ucDecVelPlanType = VEL_PLAN_T_CURVE;       //减速曲线类型
	pSPlanPointer->ucDistReAverageType = DIST_REAVG_T_CURVE;  //平摊类型（无用）

	//计算反算速度
	if (pG32->bDecPlanTransitionFlag)  //为减速过渡型连续G32
	{
		dBackCalVmax = pG32->SUserData.dVmax;   //反算速度为系统最大速度
	}
	else
	{
		//若当前的指令为G34的长轴速度规划（若为G34的长轴规划，还得计算G34的终点到当前位置的反算速度）
		if (pG32->bG34RunFlag && (pSPlanPointer != pG32->SExcessPlanForG32))
		{
			uint8 ucLongAxisNum = pG32->ucAxis[0];  //获取长轴轴号
			SPointDouble SRealTimePoint;            //实时点
			double dLongAxisVector = pG32->SStartVector.dArray[ucLongAxisNum];  //长轴向量
			double dMaxDistanceForG34;  //当前点距离G34的距离
			double dLimitSpeed;         //约束速度

			getRealTimePoint((double*)&SRealTimePoint, AXIS_NUM_MAX, pG32->pSChannel->pMapAxisArray, &pG32->ucValStatus);  //获取实时点
			dMaxDistanceForG34 = fabs(pG32->SG34EndPoint.dArray[ucLongAxisNum] * dLongAxisVector - SRealTimePoint.dArray[ucLongAxisNum] * dLongAxisVector);    //获取当前点距离G34的距离
			dLimitSpeed = min(sqrt(dMaxDistanceForG34 * 2.0 * pG32->SUserData.dDec + pG32->SUserData.dVe * pG32->SUserData.dVe), pG32->dNextG32LongAxisSpeed); //获取约束速度
			dBackCalVmax = max(sqrt(dMaxDistance * 2.0 * pG32->SUserData.dDec + pG32->SUserData.dVe * pG32->SUserData.dVe), dLimitSpeed);  //获取反算速度

		}
		else if (pSPlanPointer == pG32->SExcessPlanForG32)  //短轴速度规划
		{
			dBackCalVmax = pG32->SUserData.dVmax; //反算速度为系统最大速度

		}
		else
		{
			dBackCalVmax = max(sqrt(dMaxDistance * 2.0 * pG32->SUserData.dDec + pG32->SUserData.dVe * pG32->SUserData.dVe), pG32->dNextG32LongAxisSpeed); //反算的最大速度
		}

	}

	if (dCurrentSpeed <= dBackCalVmax)//不受到反算速度的限制
	{
		pSPlanPointer->dVmax = min(dBackCalVmax, pG32->SUserData.dVmax);//确定最大速度
		pSPlanPointer->dVe = min(dVe, pSPlanPointer->dVmax); //末速度  取传入参数dVe 与最大速度的最小值 
		pSPlanPointer->dVs = isZero(dCurrentSpeed) ? pG32->SUserData.dVs : dCurrentSpeed; //确定起始速度
	}
	else//受到反算速度的限制
	{
		pSPlanPointer->dVmax = dBackCalVmax;//确定最大速度
		pSPlanPointer->dVe = pG32->SUserData.dVe; //末速度   减速时末速度等于最大速度
		pSPlanPointer->dVs = isZero(dCurrentSpeed) ? pG32->SUserData.dVs : dBackCalVmax; //确定起始速度
		pSPlanPointer->dAcc = 1.0 * pG32->SUserData.dAcc;// 加速度 
		pSPlanPointer->dDec = 1.0 * pG32->SUserData.dDec;// 加速度
		res = 2;//已进入减速阶段
	}

	cMotionState = preparePlanningTNewAdd(pSPlanPointer);
	if (cMotionState == 2)
	{

		//*********************************多次切削是否同步测试********************************
		g_dTime++;
		if (g_dTime == 30)
		{
			uint8 ucLongAxisNum = pG32->ucAxis[0];    //获得长轴轴号
			uint8 ucShortAxisNum = pG32->ucAxis[1];  //获得短轴轴号
			struct _SRun* pSRun;//运行结构体
			pSRun = &pG32->pSChannel->SRun;//运行结构体
			//根据长轴的比例计算短轴的坐标
			pG32->dStartFollowDis = fabs(pSRun->SAbsCoord.dArray[ucLongAxisNum] - pG32->SStarCutPoint.dArray[ucLongAxisNum]);
			g_dLongAxisPos[g_nI] = pSRun->SAbsCoord.dArray[ucLongAxisNum];
			g_dShortAxisPos[g_nI] = pSRun->SAbsCoord.dArray[ucShortAxisNum];
			//g_dEncoderNum[g_nI] = g_SEncoder[SPINDLE_ENCODER - 1].lEncoderAbsValue % g_SEncoder[SPINDLE_ENCODER - 1].usEncoderCirclePulseNum;
			g_dEncoderNum[g_nI] = pG32->pSChannel->pSSpindleEncoder->lEncoderAbsValue % pG32->pSChannel->pSSpindleEncoder->usEncoderCirclePulseNum;
			if (g_nI >= REPEATG32_NUM)
			{
				g_dDeltaPulse[g_nDeltaI] = g_dEncoderNum[g_nI] - g_dEncoderNum[g_nI - REPEATG32_NUM];
				g_dDeltaLongAxis[g_nDeltaI] = g_dLongAxisPos[g_nI] - g_dLongAxisPos[g_nI - REPEATG32_NUM];
				g_dDeltaShortAxis[g_nDeltaI] = g_dShortAxisPos[g_nI] - g_dShortAxisPos[g_nI - REPEATG32_NUM];
				g_nDeltaI++;

			}
			g_nI++;
			if (g_nI == REPEATG32_NUM * 2)
			{
#if LHK_DEBUG

				for (int i = 0; i < g_nI; i++)
				{
					/*printf("g_dZ[%d] = %f \r\n", i, g_dZ[i]);
					printf("g_dEncoderNum[%d] = %f \r\n", i, g_dEncoderNum[i]);*/

				}

				for (int i = 0; i < g_nDeltaI; i++)
				{
					printf("g_dDeltaPulse[%d] = %f \r\n", i, g_dDeltaPulse[i]);
					printf("AxisNum:%d g_dDeltaLongAxis[%d] = %f \r\n", ucLongAxisNum, i, g_dDeltaLongAxis[i]);
					printf("AxisNum:%d g_dDeltaLongAxis[%d] = %f \r\n", ucShortAxisNum, i, g_dDeltaShortAxis[i]);
			}
				printf("\r\n");
#endif

				g_nDeltaI = 0;
				g_nI = 0;

			}


		}
		//*********************************多次切削是否同步测试********************************
		res = max(1, res);//随动阶段

	}
	else if (cMotionState < 0)
	{
		clearSPlanData(pSPlanPointer);
		return -1;
	}
	
	if (isZero(dMaxDistance))
	{
		res = 3;//完成
	}
	pSPlanPointer->unCurrentT = 0;  //时间为0
	pSPlanPointer->dDistRemaining = 0;//平摊的剩余距离设为0

	return res;
}



/*************************************************
  函数名称：newAddDistanceUnlimited(struct SPlan* pSPlanPointer, double newAddDist, double dAcc, double dDec, double dVe, bool bEnableDec, double dDecDist)
  函数功能  新增距离(可不限制终点)
			bEnableDec == TRUE   有减速阶段  限制终点
			bEnableDec == FALSE  无减速阶段  不限制终点
  输入参数：pSPlanPointer  预处理结构体指针
			newAddDist	新增距离
			dAcc		规划的加速度
			dDec		规划的减速度
			dVe			随动的理想目标速度
			bEnableDec	是否有减速段（有减速段->不能越过终点,无减速段->能越过终点）
			dDecDist	减速段长度
  输出参数：
  返 回 值：0 追赶阶段
			1 随动阶段  （随动阶段要是  新增距离!=当前速度  会进入加速(追赶)阶段)
			2 减速阶段	（如果有）
			3 完成阶段	（与终点重合）
  备    注：
**************************************************/
uint8 newAddDistanceUnlimited(struct SPlan* pSPlanPointer, double newAddDist, double dAcc, double dDec, double dVe, bool bEnableDec, double dDecDist, double dCurrentSpeed)
{
	uint8 res = 0; //返回值
	int8 cPlanRes;
	//SPointDouble SCurrentPoint;
	double dMaxDistance;
	double dBackCalVmax;
	double dCalDec = 0;
	//SVector currentVector;
	//SCurrentPoint = calculateExactInterpolationPoint(pSPlanPointer->dDistance - pSPlanPointer->dCurLeftDist, pSPlanPointer);//计算理论起点
	//calVectorAndDistance(SCurrentPoint, pSPlanPointer->SEndPoint, &dMaxDistance, &currentVector);

	//pSPlanPointer->dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist; //已插补长度

	if (bEnableDec == FALSE)//无减速段
	{
		pSPlanPointer->dTargetDist += newAddDist;
	}
	else//有减速段   总距离对目标距离进行限制
	{
		pSPlanPointer->dTargetDist = min(pSPlanPointer->dDistance, pSPlanPointer->dTargetDist + newAddDist);//确定目标距离  (总长度对其限制)
	}

	pSPlanPointer->dCurLeftDist = pSPlanPointer->dTargetDist - pSPlanPointer->dPreDist; // 计算剩余距离（规划用)
	dMaxDistance = pSPlanPointer->dDistance - pSPlanPointer->dPreDist;//距终点的距离   (超过终点时为负数)


	//2赋新值
	//pSPlanPointer->dDistance = pSPlanPointer->dCurLeftDist + newAddDist;//确定插补距离 
	pSPlanPointer->dAcc = 2.0 * dAcc;// 加速度(2倍)
	pSPlanPointer->dDec = 2.0 * dDec;// 加速度(2倍)
	pSPlanPointer->ucAccVelPlanType = VEL_PLAN_T_CURVE;  //加速曲线类型
	pSPlanPointer->ucDecVelPlanType = VEL_PLAN_T_CURVE;  //减速曲线类型
	pSPlanPointer->ucDistReAverageType = DIST_REAVG_T_CURVE;  //平摊类型（无用）



	if (bEnableDec == FALSE || dMaxDistance > dDecDist)//加速或者随动段
	{
		pSPlanPointer->dVmax = g_SSystemPara.dG33MaxVel;          //确定最大速度
		pSPlanPointer->dVe = min(dVe, pSPlanPointer->dVmax); //末速度  （加速和随动阶段末速度!=最大速度  减速时末速度等于最大速度)
		pSPlanPointer->dVs = isZero(dCurrentSpeed) ? g_SSystemPara.dSysMinVel : dCurrentSpeed; //确定起始速度
	}
	else  //减速段
	{
		if (!isZero(dMaxDistance + dCurrentSpeed))
		{
			//此时Vs为上一周期的数据  距离为上一个周期的剩余距离   据此计算减速度
			//dCurrentSpeed是上个周期走的距离，这样计算的原因是，如果不加上个周期走的距离，减速度将会越来越大，导致最终的速度远大于系统起跳速度
			if (isZero(pSPlanPointer->dVs - g_SSystemPara.dSysMinVel))//特殊情况：两速度相等，赋系统参数
			{
				dCalDec = g_SSystemPara.dSysPlanTMaxDec;//最大减速度
			}
			else
			{
				dCalDec = (pow(pSPlanPointer->dVs, 2) - pow(g_SSystemPara.dSysMinVel, 2)) / (2.0 * (dMaxDistance + dCurrentSpeed));
			}
		}
		
		dBackCalVmax = (dMaxDistance < 0) ? g_SSystemPara.dSysMinVel : sqrt(dMaxDistance * 2.0 * dCalDec + g_SSystemPara.dSysMinVel * g_SSystemPara.dSysMinVel); //反算的最大速度 //2026.4.2超过终点时最大速度取系统参数
		//pSPlanPointer->dDistance = min(dMaxDistance, (pSPlanPointer->dCurLeftDist + newAddDist));//确定插补距离 
		pSPlanPointer->dVmax = dBackCalVmax;//确定最大速度
		pSPlanPointer->dVe = g_SSystemPara.dSysMinVel; //末速度  （加速和随动阶段末速度!=最大速度  减速时末速度等于最大速度)
		pSPlanPointer->dVs = isZero(dCurrentSpeed) ? g_SSystemPara.dSysMinVel : dBackCalVmax;//确定起始速度  dCurrentSpeed直接改为dBackCalVmax，存在突跳，但是在减速度允许的范围内
		pSPlanPointer->dAcc = pSPlanPointer->dDec = 1.0 * dCalDec;// 加速度
		res = 2;
	}

	cPlanRes = preparePlanningTNewAdd(pSPlanPointer);
	if (cPlanRes== 2)
	{
		res = max(1, res);//随动阶段
	}
	else if (cPlanRes == -1)
	{
		clearSPlanData(pSPlanPointer);
		return -1;
	}
	
	//if (crossOfBivector(currentVector, pSPlanPointer->SStartVector) <= 0)
	// zxr: 无终点限制的加速追赶保留随动结果，仅限距减速时判断终点完成
	if (bEnableDec && (isZero(dMaxDistance) || dMaxDistance < 0))  //当前点到终点的距离=0
	{
		res = 3;//完成
	}
	//else if (dMaxDistance < 0)//加速追赶阶段有可能超过G33理论长度，此时返回4会将追赶完成的返回值1覆盖掉，导致加速阶段加速度一直计算不出来，卡死
	//{
	//	res = 4;//超越
	//}

	//pSPlanPointer->SStartPoint = SCurrentPoint;//起点
	//pSPlanPointer->dCurLeftDist = pSPlanPointer->dDistance;
	pSPlanPointer->unCurrentT = 0;
	pSPlanPointer->dDistRemaining = 0;

	return res;
}
/*************************************************
  函数名称：handWheelAddNewPlan(struct SPlan* pSPlanPointer, double newAddDist, double dvs, double dVe, double dCurrentSpeed, bool bIsRotationAxis)
  函数功能  手轮加减速新增距离
  输入参数：pSPlanPointer  预处理结构体指针
			newAddDist	新增距离
			dAcc		加速度
			dDec		减速度
			dVe			随动的理想目标速度
			bIsRotationAxis 是否为旋转轴
  输出参数：
  返 回 值：1 进入手轮加减速规划
			3 完成阶段
  备    注：
**************************************************/
uint8 handWheelAddNewPlan(struct SPlan* pSPlanPointer, double dDistance, double dVs, double dVe, double dCurrentSpeed, bool bIsRotationAxis)
{
	uint8 res = 1; //返回值
	double dMaxDistance;
	double dBackCalVmax;
	double dAcc;
	double dDec;
	double dVmax;


	pSPlanPointer->dDistance = dDistance;                          //长轴的插补长度
	pSPlanPointer->dPreDist = pSPlanPointer->dTargetDist - pSPlanPointer->dCurLeftDist; //已插补长度
	pSPlanPointer->dTargetDist = dDistance;           //目标距离无总长度限制
	pSPlanPointer->dCurLeftDist = pSPlanPointer->dTargetDist - pSPlanPointer->dPreDist; //计算剩余距离（规划用)
	dMaxDistance = pSPlanPointer->dDistance - pSPlanPointer->dPreDist;                  //距终点的距离
	dAcc = dDec = bIsRotationAxis == FALSE ? g_SSystemPara.dHandWheelAcc : g_SSystemPara.dHandWheelAcc * DEGREE_OF_CIRCLE;//手轮的最大减速度 单位：mm/ms2（r/ms2） 计算旋转轴时统一单位°/ms2
	dVmax = bIsRotationAxis == FALSE ? pSPlanPointer->dVmax : pSPlanPointer->dVmax * DEGREE_OF_CIRCLE;  //手轮的最大速度 单位：mm/ms（r/ms） 计算旋转轴时统一单位°/ms
	
	if (dAcc <= 0 || dDec <= 0)
	{
		return 1;
	}
	

	//2赋新值
	pSPlanPointer->dAcc = 2.0 * dAcc;// 加速度(2倍)
	pSPlanPointer->dDec = 2.0 * dDec;// 加速度(2倍)
	pSPlanPointer->ucAccVelPlanType = VEL_PLAN_T_CURVE;       //加速曲线类型
	pSPlanPointer->ucDecVelPlanType = VEL_PLAN_T_CURVE;       //减速曲线类型
	pSPlanPointer->ucDistReAverageType = DIST_REAVG_T_CURVE;  //平摊类型（无用）

	dMaxDistance = max(0,dMaxDistance);

	dBackCalVmax = sqrt(dMaxDistance * 2.0 * dDec + dVe * dVe);  //获取反算速度


	if (dCurrentSpeed <= dBackCalVmax)//不受到反算速度的限制
	{
		pSPlanPointer->dVmax = min(dBackCalVmax, dVmax);//确定最大速度
		pSPlanPointer->dVe = min(dVe, pSPlanPointer->dVmax); //末速度  取传入参数dVe 与最大速度的最小值 
		pSPlanPointer->dVs = isZero(dCurrentSpeed) ? dVs : dCurrentSpeed; //确定起始速度
	}
	else//受到反算速度的限制
	{
		pSPlanPointer->dVmax = dBackCalVmax;//确定最大速度
		pSPlanPointer->dVe = dVe; //末速度   减速时末速度等于最大速度
		pSPlanPointer->dVs = isZero(dCurrentSpeed) ? dVs : dBackCalVmax; //确定起始速度
		pSPlanPointer->dAcc = 1.0 * dAcc;// 加速度 
		pSPlanPointer->dDec = 1.0 * dDec;// 加速度
		res = 2;//已进入减速阶段
	}

	if (preparePlanningTNewAdd(pSPlanPointer) == -1)
	{
		return 1;
	}
		

	if (isZero(dMaxDistance))
	{
		res = 3;//完成
	}
	pSPlanPointer->unCurrentT = 0;  //时间为0
	pSPlanPointer->dDistRemaining = 0;//平摊的剩余距离设为0

	return res;
}




//清除与平摊有关的参数
int8 clearDistReAvgPara(struct SPlan* pSPlanPointer)
{
	if (pSPlanPointer == NULL)
	{
		return -1;
	}

	pSPlanPointer->unDistReT1 = 0;
	pSPlanPointer->unDistReT2 = 0;
	pSPlanPointer->unDistReT3 = 0;
	pSPlanPointer->dDistRemaining = 0;
	pSPlanPointer->dDistNewAddT = 0;
	pSPlanPointer->dTrapezoidAvgAcc = 0;
	pSPlanPointer->dTrapezoidAvgVel = 0;
	pSPlanPointer->dAverageFactor = 1.0;
}

//清楚与速度规划有关的参数
int8 clearSPlanData(struct SPlan* pSPlanPointer)
{
	if (pSPlanPointer==NULL)
	{
		return -1;
	}
	pSPlanPointer->dDistance = pSPlanPointer->dTargetDist = pSPlanPointer->dCurLeftDist = pSPlanPointer->dPreDist = 0;
	pSPlanPointer->dDistAcc = pSPlanPointer->dDistConst = pSPlanPointer->dDistDec = pSPlanPointer->dDistNewAddT=0;
	pSPlanPointer->unT1 = pSPlanPointer->unT2 = pSPlanPointer->unT3 = pSPlanPointer->unT4 = pSPlanPointer->unT5 = 0;	//6/24 陈敏聪
	pSPlanPointer->unCurrentT = 0xFFFFFFFF;
	clearDistReAvgPara(pSPlanPointer);
	return 1;
}

