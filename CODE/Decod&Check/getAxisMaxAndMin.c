
#include"ErrorFunction.h"
#include "common.h"
#include"GeneralFunctions.h"
#include "Gcode.h"

double getHorAxisMax(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin, double dStartHorVal, double dStartVerVal, double dEndHorVal, double dEndVerVal)
{
	uint8 unStartAngle;//仅用1，0表示，用于判断起点终点角度大小
	uint8 unEndAngle;
	uint8 ucHor = SGetAxisMaxAndMin->ucHor;
	uint8 ucVer = SGetAxisMaxAndMin->ucVer;
	SPointDouble SCenterPoint = *SGetAxisMaxAndMin->SCenterPoint;
	double dRadius = SGetAxisMaxAndMin->dRadius;
	int8 cArcDir = SGetAxisMaxAndMin->cArcDir;


	//如果起点纵坐标与终点纵坐标值 的符号相同
	if ((isEqualOrGreaterThanZero(dStartVerVal) && isEqualOrGreaterThanZero(dEndVerVal))		//起点纵坐标终点纵坐标 >= 0
		|| (!isEqualOrGreaterThanZero(dStartVerVal) && !isEqualOrGreaterThanZero(dEndVerVal))	//起点纵坐标终点纵坐标 < 0
		)
	{
		//纵坐标 >= 0
		if (isEqualOrGreaterThanZero(dStartVerVal))
		{
			//横坐标大，对应角度小
			if (dStartHorVal > dEndHorVal)
			{
				unStartAngle = 0;
				unEndAngle = 1;
				
			}
			else
			{
				unStartAngle = 1;
				unEndAngle = 0;
			}
		}
		//纵坐标 < 0
		else
		{
			//横坐标大，对应角度大
			if (dStartHorVal > dEndHorVal)
			{
				unStartAngle = 1;
				unEndAngle = 0;
			}
			else
			{
				unStartAngle = 0;
				unEndAngle = 1;
			}
		}
	}
	//如果起点纵坐标与终点纵坐标值 的符号不相同
	else
	{
		//纵坐标大，对应角度小
		if (dStartVerVal > dEndVerVal)
		{
			unStartAngle = 0;
			unEndAngle = 1;
		}
		else
		{
			unStartAngle = 1;
			unEndAngle = 0;
		}
	}

	//起点终点角度已判断完成，以下计算横轴最大值
	//顺弧
	if (cArcDir == ARC_CW)
	{
		//顺时针圆弧，起点角度 < 终点角度，越过X正半轴，X轴最大值 = 圆心X + R
		if (unStartAngle < unEndAngle)
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dHorMax = SCenterPoint.dArray[ucHor] + dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dHorMax = SCenterPoint.dArray[ucHor] - dRadius;
			}
		}
		//未越过X正半轴，X轴最大值 = 起点终点X较大者
		else
		{
			SGetAxisMaxAndMin->dHorMax = dStartHorVal > dEndHorVal ? SCenterPoint.dArray[ucHor] + dStartHorVal : SCenterPoint.dArray[ucHor] + dEndHorVal;
		}
	}
	//逆弧
	else
	{
		//逆时针圆弧，起点角度 < 终点角度，未越过X正半轴，X轴最大值 = 起点终点X较大者
		if (unStartAngle < unEndAngle)
		{
			SGetAxisMaxAndMin->dHorMax = dStartHorVal > dEndHorVal ? SCenterPoint.dArray[ucHor] + dStartHorVal : SCenterPoint.dArray[ucHor] + dEndHorVal;
		}
		//逆时针圆弧，起点角度 > 终点角度，越过X正半轴，X轴最大值 = 圆心X + R
		else
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dHorMax = SCenterPoint.dArray[ucHor] + dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dHorMax = SCenterPoint.dArray[ucHor] - dRadius;
			}
		}
	}
}

double getHorAxisMin(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin, double dStartHorVal, double dStartVerVal, double dEndHorVal, double dEndVerVal)
{
	uint8 unStartAngle;//仅用1，0表示，用于判断起点终点角度大小
	uint8 unEndAngle;
	uint8 ucHor = SGetAxisMaxAndMin->ucHor;
	uint8 ucVer = SGetAxisMaxAndMin->ucVer;
	SPointDouble SCenterPoint = *SGetAxisMaxAndMin->SCenterPoint;
	double dRadius = SGetAxisMaxAndMin->dRadius;
	int8 cArcDir = SGetAxisMaxAndMin->cArcDir;


	//如果起点纵坐标与终点纵坐标值 的符号相同
	if ((isEqualOrGreaterThanZero(dStartVerVal) && isEqualOrGreaterThanZero(dEndVerVal))		//起点纵坐标终点纵坐标 >= 0
		|| (!isEqualOrGreaterThanZero(dStartVerVal) && !isEqualOrGreaterThanZero(dEndVerVal))	//起点纵坐标终点纵坐标 < 0
		)
	{
		//纵坐标 >= 0
		if (isEqualOrGreaterThanZero(dStartVerVal))
		{
			//横坐标大，对应角度小
			if (dStartHorVal > dEndHorVal)
			{
				unStartAngle = 0;
				unEndAngle = 1;

			}
			else
			{
				unStartAngle = 1;
				unEndAngle = 0;
			}
		}
		//纵坐标 < 0
		else
		{
			//横坐标大，对应角度大
			if (dStartHorVal > dEndHorVal)
			{
				unStartAngle = 1;
				unEndAngle = 0;
			}
			else
			{
				unStartAngle = 0;
				unEndAngle = 1;
			}
		}
	}
	//如果起点纵坐标与终点纵坐标值 的符号不相同
	else
	{
		//纵坐标大，对应角度大
		if (dStartVerVal > dEndVerVal)
		{
			unStartAngle = 1;
			unEndAngle = 0;
		}
		else
		{
			unStartAngle = 0;
			unEndAngle = 1;
		}
	}

	//起点终点角度已判断完成，以下计算横轴最大值
	//顺弧
	if (cArcDir == ARC_CW)
	{
		//顺时针圆弧，起点角度 < 终点角度，越过X负半轴，X轴最小值 = 圆心X - R
		if (unStartAngle < unEndAngle)
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dHorMin = SCenterPoint.dArray[ucHor] - dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dHorMin = SCenterPoint.dArray[ucHor] + dRadius;
			}
		}
		//未越过X负半轴，X轴最大值 = 起点终点X较小者
		else
		{
			SGetAxisMaxAndMin->dHorMin = dStartHorVal < dEndHorVal ? SCenterPoint.dArray[ucHor] + dStartHorVal : SCenterPoint.dArray[ucHor] + dEndHorVal;
		}
	}
	//逆弧
	else
	{
		//逆时针圆弧，起点角度 < 终点角度，未越过X负半轴
		if (unStartAngle < unEndAngle)
		{
			SGetAxisMaxAndMin->dHorMin = dStartHorVal < dEndHorVal ? SCenterPoint.dArray[ucHor] + dStartHorVal : SCenterPoint.dArray[ucHor] + dEndHorVal;
		}
		else
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dHorMin = SCenterPoint.dArray[ucHor] - dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dHorMin = SCenterPoint.dArray[ucHor] + dRadius;
			}
		}
	}
}

double getVerAxisMax(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin, double dStartHorVal, double dStartVerVal, double dEndHorVal, double dEndVerVal)
{
	uint8 unStartAngle;//仅用1，0表示，用于判断起点终点角度大小
	uint8 unEndAngle;
	uint8 ucHor = SGetAxisMaxAndMin->ucHor;
	uint8 ucVer = SGetAxisMaxAndMin->ucVer;
	SPointDouble SCenterPoint = *SGetAxisMaxAndMin->SCenterPoint;
	double dRadius = SGetAxisMaxAndMin->dRadius;
	int8 cArcDir = SGetAxisMaxAndMin->cArcDir;


	//如果起点横坐标 与 横点纵坐标值 的符号相同
	if ((isEqualOrGreaterThanZero(dStartHorVal) && isEqualOrGreaterThanZero(dEndHorVal))		//起点纵坐标终点纵坐标 >= 0
		|| (!isEqualOrGreaterThanZero(dStartHorVal) && !isEqualOrGreaterThanZero(dEndHorVal))	//起点纵坐标终点纵坐标 < 0
		)
	{
		//横坐标 >= 0
		if (isEqualOrGreaterThanZero(dStartHorVal))
		{
			//纵坐标大，对应角度大
			if (dStartVerVal > dEndVerVal)
			{
				unStartAngle = 1;
				unEndAngle = 0;

			}
			else
			{
				unStartAngle = 0;
				unEndAngle = 1;
			}
		}
		//横坐标 < 0
		else
		{
			//纵坐标大，对应角度小
			if (dStartVerVal > dEndVerVal)
			{
				unStartAngle = 0;
				unEndAngle = 1;
			}
			else
			{
				unStartAngle = 1;
				unEndAngle = 0;
			}
		}
	}
	//如果起点横坐标 与 终点横坐标值 的符号不相同
	else
	{
		//横坐标大，对应角度大
		if (dStartHorVal > dEndHorVal)
		{
			unStartAngle = 1;
			unEndAngle = 0;
		}
		else
		{
			unStartAngle = 0;
			unEndAngle = 1;
		}
	}

	//起点终点角度已判断完成，以下计算纵轴最大值
	//顺弧
	if (cArcDir == ARC_CW)
	{
		//顺时针圆弧，起点角度 < 终点角度，越过Y正半轴
		if (unStartAngle < unEndAngle)
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dVerMax = SCenterPoint.dArray[ucVer] + dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dVerMax = SCenterPoint.dArray[ucVer] - dRadius;
			}
		}
		else
		{
			SGetAxisMaxAndMin->dVerMax = dStartVerVal > dEndVerVal ? SCenterPoint.dArray[ucVer] + dStartVerVal : SCenterPoint.dArray[ucVer] + dEndVerVal;
		}
	}
	//逆弧
	else
	{
		//逆时针圆弧，起点角度 < 终点角度，未越过Y正半轴
		if (unStartAngle < unEndAngle)
		{
			SGetAxisMaxAndMin->dVerMax = dStartVerVal > dEndVerVal ? SCenterPoint.dArray[ucVer] + dStartVerVal : SCenterPoint.dArray[ucVer] + dEndVerVal;
		}
		else
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dVerMax = SCenterPoint.dArray[ucVer] + dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dVerMax = SCenterPoint.dArray[ucVer] - dRadius;
			}
		}
	}
}

double getVerAxisMin(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin, double dStartHorVal, double dStartVerVal, double dEndHorVal, double dEndVerVal)
{
	uint8 unStartAngle;//仅用1，0表示，用于判断起点终点角度大小
	uint8 unEndAngle;
	uint8 ucHor = SGetAxisMaxAndMin->ucHor;
	uint8 ucVer = SGetAxisMaxAndMin->ucVer;
	SPointDouble SCenterPoint = *SGetAxisMaxAndMin->SCenterPoint;
	double dRadius = SGetAxisMaxAndMin->dRadius;
	int8 cArcDir = SGetAxisMaxAndMin->cArcDir;


	//如果起点横坐标 与 横点纵坐标值 的符号相同
	if ((isEqualOrGreaterThanZero(dStartHorVal) && isEqualOrGreaterThanZero(dEndHorVal))		//起点纵坐标终点纵坐标 >= 0
		|| (!isEqualOrGreaterThanZero(dStartHorVal) && !isEqualOrGreaterThanZero(dEndHorVal))	//起点纵坐标终点纵坐标 < 0
		)
	{
		//横坐标 >= 0
		if (isEqualOrGreaterThanZero(dStartHorVal))
		{
			//纵坐标大，对应角度大
			if (dStartVerVal > dEndVerVal)
			{
				unStartAngle = 1;
				unEndAngle = 0;

			}
			else
			{
				unStartAngle = 0;
				unEndAngle = 1;
			}
		}
		//横坐标 < 0
		else
		{
			//纵坐标大，对应角度小
			if (dStartVerVal > dEndVerVal)
			{
				unStartAngle = 0;
				unEndAngle = 1;
			}
			else
			{
				unStartAngle = 1;
				unEndAngle = 0;
			}
		}
	}
	//如果起点横坐标 与 终点横坐标值 的符号不相同
	else
	{
		//横坐标大，对应角度小
		if (dStartHorVal > dEndHorVal)
		{
			unStartAngle = 0;
			unEndAngle = 1;
		}
		else
		{
			unStartAngle = 1;
			unEndAngle = 0;
		}
	}

	//起点终点角度已判断完成，以下计算纵轴最小值
	//顺弧
	if (cArcDir == ARC_CW)
	{
		//顺时针圆弧，起点角度 < 终点角度，越过Y负半轴
		if (unStartAngle < unEndAngle)
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dVerMin = SCenterPoint.dArray[ucVer] - dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dVerMin = SCenterPoint.dArray[ucVer] + dRadius;
			}
		}
		else
		{
			SGetAxisMaxAndMin->dVerMin = dStartVerVal < dEndVerVal ? SCenterPoint.dArray[ucVer] + dStartVerVal : SCenterPoint.dArray[ucVer] + dEndVerVal;
		}
	}
	//逆弧
	else
	{
		//逆时针圆弧，起点角度 < 终点角度，未越过Y正半轴
		if (unStartAngle < unEndAngle)
		{
			SGetAxisMaxAndMin->dVerMin = dStartVerVal < dEndVerVal ? (SCenterPoint.dArray[ucVer] + dStartVerVal) : (SCenterPoint.dArray[ucVer] + dEndVerVal);
		}
		else
		{
			if (dRadius > 0)
			{
				SGetAxisMaxAndMin->dVerMin = SCenterPoint.dArray[ucVer] - dRadius;
			}
			//优弧R值为负
			else
			{
				SGetAxisMaxAndMin->dVerMin = SCenterPoint.dArray[ucVer] + dRadius;
			}
		}
	}
}



/****************************************
 函数名称：void getAxisMaxAndMin(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin)
 函数功能：求解圆弧最值点
 输入参数： ucHor               横轴
			ucVer               纵轴
			pdStartPoint		起点坐标（绝对坐标）
			pdEndPoint			终点坐标（绝对坐标）
			pSCenterPoint       圆心坐标（绝对坐标）
			dRadius				圆弧半径（>0表示小于或等于180度的圆弧；<0表示大于180°的圆弧)
			cArcDir				圆弧方向，顺时针=1，逆时针=-1
 输出参数： pSMaxVal            最大值
			pSMinVal            最小值
返 回 值：
备    注：
****************************************/
void getAxisMaxAndMin(struct _SGetAxisMaxAndMin* SGetAxisMaxAndMin)
{	
	//起点相对圆心横轴纵轴的差值
	double dStartPoint_Hor;
	double dStartPoint_Ver;
	//终点相对圆心横轴纵轴的差值
	double dEndPoint_Hor;
	double dEndPoint_Ver;
	if (SGetAxisMaxAndMin == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}
	
	//圆弧起点终点重合判断
	if(isZero(SGetAxisMaxAndMin->SStartPoint->dArray[SGetAxisMaxAndMin->ucHor] - SGetAxisMaxAndMin->SEndPoint->dArray[SGetAxisMaxAndMin->ucHor])
	&& isZero(SGetAxisMaxAndMin->SStartPoint->dArray[SGetAxisMaxAndMin->ucVer] - SGetAxisMaxAndMin->SEndPoint->dArray[SGetAxisMaxAndMin->ucVer]))
	{
		//起点终点重合，输出圆四个顶点
		SGetAxisMaxAndMin->dHorMax = SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucHor] + SGetAxisMaxAndMin->dRadius;
		SGetAxisMaxAndMin->dHorMin = SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucHor] - SGetAxisMaxAndMin->dRadius;
		SGetAxisMaxAndMin->dVerMax = SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucVer] + SGetAxisMaxAndMin->dRadius;
		SGetAxisMaxAndMin->dVerMin = SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucVer] - SGetAxisMaxAndMin->dRadius;
		return ;
	}
	//计算起点相对圆心横轴纵轴的差值
	dStartPoint_Hor = SGetAxisMaxAndMin->SStartPoint->dArray[SGetAxisMaxAndMin->ucHor] - SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucHor];
	dStartPoint_Ver = SGetAxisMaxAndMin->SStartPoint->dArray[SGetAxisMaxAndMin->ucVer] - SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucVer];
	//计算终点相对圆心横轴纵轴的差值
	dEndPoint_Hor = SGetAxisMaxAndMin->SEndPoint->dArray[SGetAxisMaxAndMin->ucHor] - SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucHor];
	dEndPoint_Ver = SGetAxisMaxAndMin->SEndPoint->dArray[SGetAxisMaxAndMin->ucVer] - SGetAxisMaxAndMin->SCenterPoint->dArray[SGetAxisMaxAndMin->ucVer];


	//获取横轴最大值
	getHorAxisMax(SGetAxisMaxAndMin, dStartPoint_Hor, dStartPoint_Ver, dEndPoint_Hor, dEndPoint_Ver);
	//获取横轴最小值
	getHorAxisMin(SGetAxisMaxAndMin, dStartPoint_Hor, dStartPoint_Ver, dEndPoint_Hor, dEndPoint_Ver);
	//获取纵轴最大值
	getVerAxisMax(SGetAxisMaxAndMin, dStartPoint_Hor, dStartPoint_Ver, dEndPoint_Hor, dEndPoint_Ver);
	//获取纵轴最小值
	getVerAxisMin(SGetAxisMaxAndMin, dStartPoint_Hor, dStartPoint_Ver, dEndPoint_Hor, dEndPoint_Ver);

}

