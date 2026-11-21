#include "common.h"
#include "math.h"
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include "GeneralFunctions.h"
#include "Config.h"
#include "Channel.h"
const double EPSINON = 0.000001;//用于变量数值比较 (10^-6)
const double EPSINON2 = 0.001;//用于变量数值比较

/****************************************
  函数名称：isZero(double dA)
  函数功能：判断double变量是否为0
  输入参数：
  输出参数：
  返 回 值：TRUE:  dA等于0，
			FALSE: dA不等于0
  备    注：dA == 0
****************************************/
bool isZero(double dA) //利用EPSINON判断浮点数dA是否为零
{
	if ((dA >= -EPSINON) && (dA <= EPSINON))
	{
		return TRUE;
	}

	return FALSE;
}
/****************************************
  函数名称：isZero(double dA)
  函数功能：判断double变量是否为0
  输入参数：
  输出参数：
  返 回 值：TRUE:  dA等于0，
			FALSE: dA不等于0
  备    注：用EPSINON2做比较
****************************************/
bool isZero_EPSINON2(double dA) //利用EPSINON判断浮点数dA是否为零
{
	if ((dA >= -EPSINON2) && (dA <= EPSINON2))
	{
		return TRUE;
	}

	return FALSE;
}

/*************************************************
  函数名称：isZeroWithDeviation(double dA, double dDeviation)
  函数功能  利用dDeviation判断浮点数为零
  输入参数：dA			double型变量
			dDeviation	允许误差
  输出参数：
  返 回 值：TRUE:  dA在误差范围内等于0， 
			FALSE: dA在误差范围内不等于0
  备    注：-dDeviation <= dA <= dDeviation ?
**************************************************/
bool isZeroWithDeviation(double dA, double dDeviation)
{
	if ((dA >= -dDeviation) && (dA <= dDeviation))
	{
		return TRUE;
	}

	return FALSE;
}

/*************************************************
  函数名称：bool isBetweenA2B(double dVal, double dA, double dB)
  函数功能  判断浮点变量dVal是否在dA与dB之间
  输入参数：
  输出参数：
  返 回 值：TRUE 符合条件
			FALSE 不符合条件
  备    注：dA <= dVal <= dB 或 dB <= dVal <= dA ?
**************************************************/
bool isBetweenA2B(double dVal, double dA, double dB)
{
	//1.dA <= dVal <= dB
	if (isEqualOrGreaterThanZero(dVal - dA) && isEqualOrLessThanZero(dVal - dB))
	{
		return TRUE;
	}
	//2.dB <= dVal <= dA
	if (isEqualOrGreaterThanZero(dVal - dB) && isEqualOrLessThanZero(dVal - dA))
	{
		return TRUE;
	}
	return FALSE;
}

/*************************************************
  函数名称：isEqualOrGreaterThanZero(double dA)
  函数功能  判断浮点变量是否大于等于零
  输入参数：dA，double型变量
  输出参数：
  返 回 值：TRUE 符合条件
			FALSE 不符合条件
  备    注：dA >= 0 ?
**************************************************/
bool isEqualOrGreaterThanZero(double dA)
{
	if (dA > 0 ||isZero(dA)) //大于零 或者 为零 符合条件
	{
		return TRUE;
	}

	return FALSE;//不为零 且不大于零
}

/*************************************************
  函数名称：isUnEqualAndGreaterThanZero(double dA)
  函数功能  判断浮点变量是否大于零 但不等于零
  输入参数：dA，double型变量
  输出参数：
  返 回 值：TRUE 符合条件
			FALSE 不符合条件
  备    注：dA > 0 ?
**************************************************/
bool isUnEqualAndGreaterThanZero(double dA)
{
	if (isZero(dA)) // 为零 不符合条件
	{
		return FALSE;
	}
	else if (dA < 0) //不为零  但小于零 不符合条件
	{
		return FALSE;
	}
	return TRUE; //不为零 且大于零
}

/*************************************************
  函数名称：isEqualOrLessThanZero(double dA)
  函数功能  判断浮点变量是否小于等于零
  输入参数：dA，double型变量
  输出参数：
  返 回 值：TRUE 符合条件
			FALSE 不符合条件
  备    注：dA <= 0 ?
**************************************************/
bool isEqualOrLessThanZero(double dA)
{
	if (dA < 0 ||isZero(dA)) //小于零 或者为零 符合条件
	{
		return TRUE;
	}

	return FALSE;//不为零 且大于零
}

/*************************************************
  函数名称：isUnEqualAndLessThanZero(double dA)
  函数功能  判断浮点变量是否小于零 但不等于零
  输入参数：dA，double型变量
  输出参数：
  返 回 值：TRUE 符合条件
			FALSE 不符合条件
  备    注：dA < 0 ?
**************************************************/
bool isUnEqualAndLessThanZero(double dA)
{
	if (isZero(dA)) // 为零 不符合条件
	{
		return FALSE;
	}
	else if (dA > 0) //不为零  但大于零 不符合条件
	{
		return FALSE;
	}
	return TRUE; //不为零 且小于零
}

/*************************************************
  函数名称：sgn(double dA)
  函数功能  符号判断函数，与0比较判断正负
  输入参数：dA，double型变量
  输出参数：
  返 回 值：1:  dA符号为正
			-1: dA符号为负
			0:	dA等于0
  备    注：sgn(dA)
**************************************************/
int8 sgn(double dA)
{
	
	if (isUnEqualAndGreaterThanZero(dA))
	{
		return 1;
	}
	if (isUnEqualAndLessThanZero(dA))
	{
		return -1;
	}
	return 0;
}

/*************************************************
  函数名称：nSgn(int dA)
  函数功能  符号判断函数，与0比较判断正负
  输入参数：nA，double型变量
  输出参数：
  返 回 值：1:  nA符号为正
			-1: nA符号为负
			0:	nA等于0
  备    注：sgn(nA)
**************************************************/
int8 nSgn(int nA)
{
	if (nA > 0)
	{
		return 1;
	}
	if (nA < 0)
	{
		return -1;
	}
	return 0;
}

/*************************************************
  函数名称：radCalculateIK(SPointDouble SStartPoint,SPointDouble SEndPoint,SPointDouble SCenterPoint,int8 cArcDir,double *dRad)
  函数功能  利用指定的IK，起点和终点坐标来求圆心角
  输入参数：pdStartPoint	起点坐标（绝对坐标）
			pdEndPoint		终点坐标（绝对坐标）
			pdCenterPoint	圆心坐标（绝对坐标）
			cArcDir			圆弧方向，顺时针=1，逆时针=-1
			ucHorizontalAxis	横轴轴号（注意减 1 ）
			ucVerticalAxis		纵轴轴号（注意减 1 ）
  输出参数：dRad		插补圆弧对应的圆心角（弧度）
  返 回 值：
  备    注：此函数不能判断，起点与圆心构成的圆 = 终点与圆心构成的圆  （是否同圆）
**************************************************/
void radCalculateIK(double* pdStartPoint, double* pdEndPoint, double* pdCenterPoint, int8 cArcDir, double* dRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{
	//cArcDir = -cArcDir;//后置刀架坐标系
	pdCenterPoint[ucHorizontalAxis] -= pdStartPoint[ucHorizontalAxis];//将圆心绝对坐标转换为相对坐标（相对起点）
	pdCenterPoint[ucVerticalAxis] -= pdStartPoint[ucVerticalAxis];

	if (isZero(pdStartPoint[ucHorizontalAxis] - pdEndPoint[ucHorizontalAxis]) && isZero(pdStartPoint[ucVerticalAxis] - pdEndPoint[ucVerticalAxis]))
	{
		*dRad = 2 * PI;  //起点和终点坐标相同 返回360
		return;
	}
	else
	{

		double rad;   //返回值  圆心角

		double dRelaXe = pdEndPoint[ucHorizontalAxis] - pdStartPoint[ucHorizontalAxis];//终点坐标（相对起点）
		double dRelaZe = pdEndPoint[ucVerticalAxis] - pdStartPoint[ucVerticalAxis];

		double dVectorSx = -pdCenterPoint[ucHorizontalAxis];   //圆心→起点 的向量O'S'
		double dVectorSz = -pdCenterPoint[ucVerticalAxis];

		double dVectorEx = -pdCenterPoint[ucHorizontalAxis] + dRelaXe;   //圆心→终点 的 向量O'E'
		double dVectorEz = -pdCenterPoint[ucVerticalAxis] + dRelaZe;

		double dVectorRx = dVectorSz;   //与向量S垂直 且指向其右侧的向量 O'R
		double dVectorRz = -dVectorSx;

		double dDotER = dVectorRx * dVectorEx + dVectorRz * dVectorEz;   //向量O'E'和向量O'R的内积
		double dDotSE = dVectorSx * dVectorEx + dVectorSz * dVectorEz;    //向量O'S'和向量O'E'的内积

		double flag = (dDotER * cArcDir);     //（O'E'和O'R的内积）* cArcDir
		if (isZero(dDotER))
		{//内积为0  垂直 圆心角为PI
			*dRad = PI;
			return;
		}

		//不为0，计算圆心角（小于PI）
		rad = acos((dDotSE) / (sqrt(dVectorSx * dVectorSx + dVectorSz * dVectorSz) * sqrt(dVectorEx * dVectorEx + dVectorEz * dVectorEz)));

		//结合【顺】 【逆】 和向量S与E的位置关系  即可知道路径是否大于PI
		rad = (flag > 0) ? rad : 2 * PI - rad;

		*dRad = rad;//传出圆心角

		return;
	}//end of else
}

/*************************************************
  函数名称：radCalculateR(SPointDouble SStartPoint,SPointDouble SEndPoint,double dRadius, SPointDouble *SCenterPoint,int8 cArcDir,double *dRad)
  函数功能  利用圆弧半径R(带符号)，起点和终点坐标来求圆心角
  输入参数：pdStartPoint		起点坐标（绝对坐标）
			pdEndPoint			终点坐标（绝对坐标）
			dRadius				圆弧半径（>0表示小于或等于180度的圆弧；<0表示大于180°的圆弧)
			cArcDir				圆弧方向，顺时针=1，逆时针=-1
			ucHorizontalAxis	横轴轴号（注意减 1）
			ucVerticalAxis		纵轴轴号（注意减 1）
  输出参数：pdCenterPoint		圆心坐标（绝对坐标）
			dRad		插补圆弧对应的圆心角（弧度）
  返 回 值：TRUE:  计算成功
			FALSE: 参数错误
  备    注：1.由 起点、终点、圆弧方向、圆弧半径 得到 圆心坐标、圆心角
			2.不能对整圆进行判断，输入圆弧的起点
**************************************************/
bool radCalculateR(double* pdStartPoint, double* pdEndPoint, double dRadius, double* pdCenterPoint,
	int8 cArcDir, double* pRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{
	double dDx = pdEndPoint[ucHorizontalAxis] - pdStartPoint[ucHorizontalAxis];
	double dDy = pdEndPoint[ucVerticalAxis] - pdStartPoint[ucVerticalAxis];
	double dAbsRadius = fabs(dRadius);
	double dHalfChord = hypot(0.5 * dDx, 0.5 * dDy);
	double dTolerance = 8.0 * DBL_EPSILON * fmax(dAbsRadius, dHalfChord);

	// Compare lengths with a relative roundoff allowance, not the global squared-distance tolerance.
	if (!isfinite(dDx) || !isfinite(dDy) || !isfinite(dAbsRadius) || !isfinite(dHalfChord)
		|| dHalfChord - dAbsRadius > dTolerance)
	{
		*pRad = 0.0;
		return FALSE;
	}
	// R cannot define a full circle; preserve the exact coincident-point convention.
	if (dDx == 0.0 && dDy == 0.0)
	{
		pdCenterPoint[ucHorizontalAxis] = pdStartPoint[ucHorizontalAxis];
		pdCenterPoint[ucVerticalAxis] = pdStartPoint[ucVerticalAxis];
		*pRad = 0.0;
		return TRUE;
	}
	if (dAbsRadius == 0.0 || dHalfChord == 0.0)
	{
		*pRad = 0.0;
		return FALSE;
	}
	if (fabs(dHalfChord - dAbsRadius) <= dTolerance)
	{
		pdCenterPoint[ucHorizontalAxis] = pdStartPoint[ucHorizontalAxis] + 0.5 * dDx;
		pdCenterPoint[ucVerticalAxis] = pdStartPoint[ucVerticalAxis] + 0.5 * dDy;
		*pRad = PI;
		return TRUE;
	}
	else
	{
		// Normalize before multiplication to avoid squaring very small or large lengths.
		double dRatio = dHalfChord / dAbsRadius;
		double dHeight = dAbsRadius * sqrt((1.0 - dRatio) * (1.0 + dRatio));
		int signR = dRadius < 0.0 ? -1 : 1; // sgn() would discard the sign of a tiny R.
		double dSide = signR * cArcDir > 0 ? 1.0 : -1.0;
		double dCenterH = pdStartPoint[ucHorizontalAxis] + 0.5 * dDx
			+ dSide * ((0.5 * dDy) / dHalfChord) * dHeight;
		double dCenterV = pdStartPoint[ucVerticalAxis] + 0.5 * dDy
			- dSide * ((0.5 * dDx) / dHalfChord) * dHeight;
		double dRad = 2.0 * asin(dRatio);
		dRad = signR > 0 ? dRad : 2.0 * PI - dRad;
		if (!isfinite(dCenterH) || !isfinite(dCenterV) || !isfinite(dRad))
		{
			*pRad = 0.0;
			return FALSE;
		}
		pdCenterPoint[ucHorizontalAxis] = dCenterH;
		pdCenterPoint[ucVerticalAxis] = dCenterV;
		*pRad = dRad;
		return TRUE;
	}
}

/*************************************************
  函数名称：
  函数功能  计算圆心角
  输入参数：
  输出参数：pdCenterPoint		圆心坐标（绝对坐标）
			dRad		插补圆弧对应的圆心角（弧度）
  返 回 值：TRUE:  计算成功
			FALSE: 参数错误
  备    注：1
**************************************************/
double calculateCentralAngle(double dCenter_X, double dCenter_Y, double dStart_X, double dStart_Y, double dEnd_X, double dEnd_Y, int cArcDir) 
{
	// 计算向量 OA 和 OB
	double dOA_X = dStart_X - dCenter_X;
	double dOA_Y = dStart_Y - dCenter_Y;
	double dOB_X = dEnd_X - dCenter_X;
	double dOB_Y = dEnd_Y - dCenter_Y;

	// 计算向量 OA 和 OB 的长度
	double OA_length = sqrt(dOA_X * dOA_X + dOA_Y * dOA_Y);
	double OB_length = sqrt(dOB_X * dOB_X + dOB_Y * dOB_Y);

	// 计算向量 OA 和 OB 的点积
	double dot_product = dOA_X * dOB_X + dOA_Y * dOB_Y;

	// 计算夹角的余弦值
	double cos_theta = dot_product / (OA_length * OB_length);

	// 通过反余弦函数求出夹角（以弧度为单位）
	double theta = acos(cos_theta);

	// 计算向量叉积来判断方向
	double cross_product = dOA_X * dOB_Y - dOA_Y * dOB_X;

	// 根据方向调整角度，顺时针时用 2*PI - theta
	if ((cArcDir && cross_product > 0) || (!cArcDir && cross_product < 0)) {
		theta = 2 * PI - theta;
	}

	// 返回角度（弧度）
	return theta;
}



/*************************************************
  函数名称：double surplusDoubleData(double data, int remainder)
  函数功能  计算浮点类型数据取余后的结果
  输入参数：data 被余数
			remainder 余数
  输出参数：无
  返 回 值：浮点类型数据取余后的结果
  备    注：dData % nRemainder（输出+or-）
**************************************************/
double surplusDoubleData(double dData, int nRemainder)
{
	int64 nintegerPart;     //整数部分
	double dDecimalPart;  //小数部分
	int64 AfterSurpulsNum;  //整数部分取余后的数值

	if (nRemainder == 0)
	{
		parseRunningError(9999,GCODE_ERROR_NUM153,9999);
		return 0;
	}
	
	dDecimalPart = dData - (int64)dData;        //计算目标值的小数部分
	nintegerPart = (int64)dData;               //计算目标值的整数部分
	AfterSurpulsNum = nintegerPart % nRemainder;   //计算目标值整数部分取余后的数值
	return AfterSurpulsNum + dDecimalPart;      //返回浮点类型数据取余后的结果
}

/*************************************************
  函数名称：surplusDoubleToPositive(double data, int remainder)
  函数功能  计算浮点类型数据取余后的结果
  输入参数：data 被余数
			remainder 余数
  输出参数：无
  返 回 值：浮点类型数据取余后的结果  （正数）
  备    注：dData % nRemainder（输出+）
**************************************************/
double surplusDoubleToPositive(double dData, int nRemainder)
{
	// int nIntegerPart;     //整数部分
	// double dDecimalPart;  //小数部分
	// int AfterSurpulsNum;  //整数部分取余后的数值

	// dDecimalPart = dData - (int)dData;        //计算目标值的小数部分
	// nIntegerPart = (int)dData;               //计算目标值的整数部分
	// // AfterSurpulsNum = (dData >= 0) ? nIntegerPart % nRemainder : nIntegerPart % nRemainder + nRemainder;   //计算目标值整数部分取余后的数值

	// return AfterSurpulsNum + dDecimalPart;      //返回浮点类型数据取余后的结果
	
	//修改计算目标值整数部分取余后的数值 2026.5.7
	if (nRemainder == 0)
	{
		parseRunningError(9999, GCODE_ERROR_NUM153, 9999);
		return 0;
	}

	double dResult = fmod(dData, (double)nRemainder); //利用 fmod 获取余数，若为负则加上除数

	// 标准化到 [0, nRemainder)
	if (dResult < 0)
	{
		dResult += (double)nRemainder;
	}

	return dResult;
}

/*************************************************
  函数名称：addPoint(double* pSPara1, double* pSPara2, double* pSOutPut, int nArraySize)
  函数功能  计算两个坐标相加（向量相加）
  输入参数：pSPara1			坐标1
			pSPara2			坐标2
			nArraySize		数组长度
			ucValStatus		有效值
  输出参数：pSOutPut		输出坐标
  返 回 值：-1 数组长度错误
			-2 空指针
			1  计算完成
  备    注：
**************************************************/
int8 addPoint(double* pSPara1, double* pSPara2, double* pSOutPut, int nArraySize, const VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;

	if (nArraySize <= 0) return -1; // 非法值
	if (pSOutPut == NULL) return -2;//空指针

	if (ucValStatus == NULL)
	{
		//逐项相加
		for (i = 0; i < nArraySize; i++)
		{
			pSOutPut[i] = pSPara1[i] + pSPara2[i];
		}
	}
	else
	{
		//逐项相加
		for (i = 0; i < nArraySize; i++)
		{
			if (VAL_VALID != getbit(*ucValStatus, i)) continue;
			pSOutPut[i] = pSPara1[i] + pSPara2[i];
		}
	}

	return 1;
}

/*************************************************
  函数名称：subtractPoint(double * pSPara1, double * pSPara2, double * pSOutPut, int nArraySize)
  函数功能  计算两个坐标相减（向量相减） 坐标1 - 坐标2
  输入参数：pSPara1			坐标1
			pSPara2			坐标2
			nArraySize		数组长度
			ucValStatus		有效值
  输出参数：pSOutPut		输出坐标
  返 回 值：-1 数组长度错误
			-2 空指针
			1  计算完成
  备    注：
**************************************************/
int8 subtractPoint(double* pSPara1, double* pSPara2, double* pSOutPut, int nArraySize, const VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;

	if (nArraySize <= 0) return -1; // 非法值
	if (pSOutPut == NULL) return -2;//空指针

	if (ucValStatus == NULL)
	{
		//逐项相减
		for (i = 0; i < nArraySize; i++)
		{
			pSOutPut[i] = pSPara1[i] - pSPara2[i];
		}
	}
	else
	{
		//逐项相减
		for (i = 0; i < nArraySize; i++)
		{
			if (VAL_VALID != getbit(*ucValStatus, i)) continue;
			pSOutPut[i] = pSPara1[i] - pSPara2[i];
		}
	}



	return 1;
}

/*************************************************
  函数名称：isZeroPoint(double* dTargetPoint, int nArraySize, const VAL_STRUCT_TYPE* ucValStatus)
  函数功能  坐标判断是否为零
  输入参数：pdTargetPoint	目标点
			nArraySize		数组大小
			ucValStatus		有效值
  输出参数：
  返 回 值：TRUE 零点
			FALSE 不是零点
  备    注：
**************************************************/
bool isZeroPoint(double* pdTargetPoint, int nArraySize, const VAL_STRUCT_TYPE* ucValStatus)
{
	int i = 0;
	if (pdTargetPoint == NULL)//空指针 返回
	{
		return FALSE;
	}
	
	if (ucValStatus == NULL)
	{
		for ( i = 0; i < nArraySize; i++)
		{
			if (!isZero(pdTargetPoint[i]))// 不为0 返回FALSE
			{
				return FALSE;
			}
		}
	}
	else
	{
		for (i = 0; i < nArraySize; i++)
		{
			if ((VAL_VALID == getbit(*ucValStatus,i)) && !isZero(pdTargetPoint[i]))//轴有效 且不为0时
			{
				return FALSE;
			}
		}
	}

	return TRUE;//所有坐标为0 返回TRUE
}


/*************************************************
  函数名称：modifyRelaEndPoint(double *pdStartPoint, double *pdEndPoint, uint8* ucAxisMap, int nArraySize,const VAL_STRUCT_TYPE *ucRelaVal)
  函数功能  根据实时坐标修正  《由相对位移指定的终点坐标》  
  输入参数：pdStartPoint	起点坐标
			pdEndPoint		终点坐标
			ucAxisMap		映射
			nArraySize		数组长度
			ucRelaVal		有效值
  输出参数：pdEndPoint		输出终点坐标
  返 回 值：-1 空指针
			-2 数组长度错误
			1  计算完成
  备    注：
**************************************************/
int8 modifyRelaEndPoint(double *pdStartPoint, double *pdEndPoint,const uint8* ucAxisMap, int nArraySize,const VAL_STRUCT_TYPE *ucRelaVal)
{
	double dDeltaArray[AXIS_NUM_MAX] = { 0 };
	if (pdStartPoint == NULL || pdEndPoint == NULL || ucAxisMap == NULL)
	{
		return -1;
	}
	if (AXIS_NUM_MAX < nArraySize)//Delta数组空间不足
	{
		return -2;
	}
	subtractPoint(pdEndPoint, pdStartPoint, dDeltaArray, nArraySize, ucRelaVal);  //终点减去起点  得到delta值
	getRealTimePoint(pdStartPoint, nArraySize, ucAxisMap, ucRelaVal);					//终点获取实时坐标
	addPoint(pdStartPoint, dDeltaArray, pdEndPoint, nArraySize, ucRelaVal);			//实时坐标+终点  等于目标终点
	return 1;
}

/*************************************************
  函数名称：doubleArrayMax(double* array, int nArraySize, double* pdMaxVal)
  函数功能  寻找数组最大值的下标  （可选 输出最大值）
  输入参数：array			数组首地址
			nArraySize		数组长度
  输出参数：pdMaxVal		输出最大值
  返 回 值：-1 空指针
			1  计算完成
  备    注：
**************************************************/
int doubleArrayMax(double* array, int nArraySize, double* pdMaxVal)
{
	double maxVal;
	int nNum = 0;
	int i;
	if (array == NULL)
	{
		return -1;
	}
	maxVal = array[0];
	for (i = 1; i < nArraySize; i++)
	{
		if (array[i] > maxVal)
		{
			maxVal = array[i];
			nNum = i;
		}
	}
	if (pdMaxVal != NULL)
	{
		*pdMaxVal = maxVal;
	}
	return nNum;
}

/*************************************************
  函数名称：
  函数功能  判断通道间是否存在公共轴   2022.4.25 郭强
  输入参数：
  输出参数：
  返 回 值： TRUE:  有公共轴
			FALSE: 无公共轴
  备    注：
**************************************************/
bool judgeCommonPhyAxisBtwChannels()
{
	//CHANNEL_NUM_MAX   AXIS_NUM_MAX
	//g_SSystemPara.ucAxisMapArray[0][AXIS_X]
	int i = 0;
	int j = 0;

	uint8 ucNumI;
	uint8 ucNumJ;
	for (i = 0; i < CHANNEL_NUM_MAX * AXIS_NUM_MAX - 1; i++)
	{
		ucNumI = g_SSystemPara.ucAxisMapArray[i / AXIS_NUM_MAX][i % AXIS_NUM_MAX];
		if (ucNumI < AXIS_PHYSICS_NUM_MAX)//i代表轴非空
		{
			for (j = i + 1; j < CHANNEL_NUM_MAX * AXIS_NUM_MAX; j++)
			{
				ucNumJ = g_SSystemPara.ucAxisMapArray[j / AXIS_NUM_MAX][j % AXIS_NUM_MAX];
				if (ucNumJ < AXIS_PHYSICS_NUM_MAX)//j代表轴非空
				{
					if (ucNumI == ucNumJ)
					{
						return TRUE;
					}
				}
			}
		}

	}
	return FALSE;
}


/*************************************************
  函数名称：polarCoord2CommonCoord(double* dPoint, int nArraySize, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
  函数功能  //极坐标转换成普通
  输入参数：dPoint				转换的点
			nArraySize			数组大小
			ucHorizontalAxis	横轴
			ucVerticalAxis		纵轴
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
int8 polarCoord2CommonCoord(double* dPoint, int nArraySize, uint8 ucHorizontalAxis, uint8 ucVerticalAxis,int8 cDir)
{ // 根据极坐标方向转换
	double dPolarCoord[2];//极坐标
	double dCommonCoord[2];//普通坐标
	if (dPoint == NULL || ucHorizontalAxis >= nArraySize || ucVerticalAxis >= nArraySize)
	{
		return -1;
	}
	//读取坐标
	dPolarCoord[0] = dPoint[ucHorizontalAxis];//横轴
	dPolarCoord[1] = dPoint[ucVerticalAxis];//纵轴

	//普X = sqrt[(极坐标X)^2 + (极坐标C)^2]
	//普C = atan2(极坐标C, 极坐标X);
	dCommonCoord[0] = cDir*sqrt(dPolarCoord[0] * dPolarCoord[0] + dPolarCoord[1] * dPolarCoord[1]);
	dCommonCoord[1] = atan2(dPolarCoord[1], dPolarCoord[0]) * 180 / PI;
	//if (cDir<0 )
	//{
	//	dCommonCoord[1] = atan2(dPolarCoord[1], -dPolarCoord[0]) * 180 / PI;
	//}
	//写入坐标
	dPoint[ucHorizontalAxis] = dCommonCoord[0];
	dPoint[ucVerticalAxis] = dCommonCoord[1];
	return 1;
}



/*************************************************
  函数名称：commonCoord2PolarCoord(double* dPoint, int nArraySize, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
  函数功能  //普通坐标转换成极坐标
  输入参数：dPoint				转换的点
			nArraySize			数组大小
			ucHorizontalAxis	横轴
			ucVerticalAxis		纵轴
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
int8 commonCoord2PolarCoord(double* dPoint, int nArraySize, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{
	double dPolarCoord[2];//极坐标
	double dCommonCoord[2];//普通坐标
	if (dPoint == NULL || ucHorizontalAxis >= nArraySize || ucVerticalAxis >= nArraySize)
	{
		return -1;
	}

	//读取坐标
	dCommonCoord[0] = dPoint[ucHorizontalAxis];//横轴
	dCommonCoord[1] = dPoint[ucVerticalAxis];//纵轴

	//极坐标X = 普X * sin[(90-普C)/360]
	//极坐标C = 普X * cos[(90 - 普C) / 360]
	//double sss = sin((90 - dCommonCoord[1]) * PI / 180.0);
	dPolarCoord[0] = dCommonCoord[0] * sin((90 - dCommonCoord[1]) * PI / 180.0);
	dPolarCoord[1] = dCommonCoord[0] * cos((90 - dCommonCoord[1]) * PI / 180.0);

	//写入坐标
	dPoint[ucHorizontalAxis] = dPolarCoord[0];
	dPoint[ucVerticalAxis] = dPolarCoord[1];
	return 1;
}


/*************************************************
  函数名称：absPoint2RealTimePoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus)
  函数功能  // 部分绝对坐标转换成机械坐标
  输入参数：ucChannelNum		通道号  -》 需要知道总偏移值
			dPoint				需要转换的坐标
			nArraySize			数组大小
			ucValStatus			有效值
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
int8 absPoint2RealTimePoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus)
{
	if (dPoint == NULL || ucChannelNum>= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	addPoint(dPoint, (double*)&g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray, dPoint, nArraySize, &ucValStatus);
	//subtractPoint(dPoint,(double*)&g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray, dPoint, nArraySize, &ucValStatus);
	return 1;
}


/*************************************************
  函数名称：realTimePoint2AbsPoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus)
  函数功能  // 部分机械坐标转换成绝对坐标
  输入参数：ucChannelNum		通道号  -》 需要知道总偏移值
			dPoint				需要转换的坐标
			nArraySize			数组大小
			ucValStatus			有效值
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
int8 realTimePoint2AbsPoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus)
{
	if (dPoint == NULL || ucChannelNum >= CHANNEL_NUM_MAX)
	{
		return -1;
	}
	subtractPoint(dPoint, (double*)&g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray, dPoint, nArraySize, &ucValStatus);
	//addPoint(dPoint, (double*)&g_SChannel[ucChannelNum].SRun.SCurrentOffset.dArray, dPoint, nArraySize, &ucValStatus);
	return 1;
}


/*************************************************
  函数名称：realTimePoint2AbsPoint(uint8 ucChannelNum, double* dPoint, int nArraySize, const VAL_STRUCT_TYPE ucValStatus)
  函数功能  //计算旋转坐标系后的点坐标
  输入参数：dPoint				需要转换的坐标
			nArraySize			数组大小
			dOutputPoint		输出坐标
			dDeltaRad			旋转弧度大小
			ucHorizontalAxis	横轴
			ucVerticalAxis		纵轴
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
int8 rotatingCoordinate(double* dPoint, int nArraySize,double *dOutputPoint, double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{
	double dTempPoint[2];
	if (dPoint == NULL || dOutputPoint == NULL || ucHorizontalAxis >= nArraySize || ucVerticalAxis >= nArraySize)
	{
		return -1;
	}

	dTempPoint[0] = dPoint[ucHorizontalAxis] * cos(dDeltaRad) + dPoint[ucVerticalAxis] * sin(dDeltaRad);
	dTempPoint[1] = -dPoint[ucHorizontalAxis] * sin(dDeltaRad) + dPoint[ucVerticalAxis] * cos(dDeltaRad);
	dOutputPoint[ucHorizontalAxis] = dTempPoint[0];
	dOutputPoint[ucVerticalAxis] = dTempPoint[1];
	return 1;
}

/*************************************************
  函数名称：isOnTheEllipse(double dH, double dV, double dH0, double dV0, double dA, double dB)
  函数功能  判断点  是否在椭圆弧上
  输入参数：dH					横轴坐标
			dV					纵轴坐标
			dH0					中心点横轴坐标
			dV0					中心点纵轴坐标	
			dA					椭圆参数A
			dB					椭圆参数B
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
bool isOnTheEllipse(double dH, double dV, double dH0, double dV0, double dA, double dB)
{
	double dResult;

	//椭圆标准方程
	dResult = pow((dH - dH0)  / dA,  2 )+ pow((dV - dV0)  / dB , 2) - 1.0;

	if (isZero(dResult))
	{
		return TRUE;
	}
	return FALSE;
}

/*************************************************
  函数名称：ellipseCenterCalculate(double* pdStartPoint, double* pdEndPoint, int nArraySize, double dA,double dB, double* pdCenterPoint, int8 cArcDir, double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
  函数功能  椭圆弧中心点计算
  输入参数：pdStartPoint		起点
			pdEndPoint			终点
			nArraySize			数组大小
			dA					椭圆参数A
			dB					椭圆参数B
			pdCenterPoint		输出坐标
			cArcDir				圆弧方向   1为顺时针  -1为逆时针
			dDeltaRad			旋转弧度大小
			ucHorizontalAxis	横轴
			ucVerticalAxis		纵轴
  输出参数：
  返 回 值： -1 错误值
			  1 成功
  备    注：
**************************************************/
int8 ellipseCenterCalculate(double* pdStartPoint, double* pdEndPoint, int nArraySize, double dA,double dB, double* pdCenterPoint, int8 cArcDir, double dDeltaRad, uint8 ucHorizontalAxis, uint8 ucVerticalAxis)
{
	double SStartPoint[2]; // 起点
	double SEndPoint[2];   // 终点
	double SCenterPoint[2]; // 椭圆心
	double SVector[2];  //向量
	double dC, dD, dE; // 计算系数
	double dSquareA = dA*dA;
	double dSquareB = dB*dB;
	double dSquareV2subV1; // (终点纵坐标-起点纵坐标)^2
	double dDelta;//求根公式delta
	double dDot; // 内积

	if (pdStartPoint==NULL || pdEndPoint == NULL || pdCenterPoint == NULL //|| pDeltaRad == NULL
		|| ucHorizontalAxis>= nArraySize|| ucVerticalAxis >= nArraySize  )
	{
		return -1;
	}
	// 起点
	SStartPoint[AXIS_HORIZONTAL] = pdStartPoint[ucHorizontalAxis]; 
	SStartPoint[AXIS_VERTICAL] = pdStartPoint[ucVerticalAxis];
	// 终点
	SEndPoint[AXIS_HORIZONTAL] = pdEndPoint[ucHorizontalAxis];
	SEndPoint[AXIS_VERTICAL] = pdEndPoint[ucVerticalAxis];

	//旋转坐标系
	rotatingCoordinate(SStartPoint, 2, SStartPoint, dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);
	rotatingCoordinate(SEndPoint, 2, SEndPoint, dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);

	if (isZero(SStartPoint[AXIS_VERTICAL] - SEndPoint[AXIS_VERTICAL]) )
	{ //起点终点横坐标相等 直接计算
		SCenterPoint[AXIS_HORIZONTAL] = (SStartPoint[AXIS_HORIZONTAL] + SEndPoint[AXIS_HORIZONTAL]) / 2;
		SCenterPoint[AXIS_VERTICAL] = SStartPoint[AXIS_VERTICAL] - dB * sqrt(1 - pow((SStartPoint[AXIS_HORIZONTAL] - SCenterPoint[AXIS_HORIZONTAL]) / dA, 2));
		if (!isOnTheEllipse(SEndPoint[AXIS_VERTICAL], SEndPoint[AXIS_VERTICAL], SCenterPoint[AXIS_HORIZONTAL], SCenterPoint[AXIS_VERTICAL], dA, dB))
		{
			SCenterPoint[AXIS_VERTICAL] = SStartPoint[AXIS_VERTICAL] + dB * sqrt(1 - pow((SStartPoint[AXIS_HORIZONTAL] - SCenterPoint[AXIS_HORIZONTAL]) / dA, 2));
		}
	}
	else if (isZero(SStartPoint[AXIS_HORIZONTAL] - SEndPoint[AXIS_HORIZONTAL]))
	{//起点终点横坐标相等 直接计算
		SCenterPoint[AXIS_VERTICAL] = (SStartPoint[AXIS_VERTICAL] + SEndPoint[AXIS_VERTICAL]) / 2;
		SCenterPoint[AXIS_HORIZONTAL] = SStartPoint[AXIS_HORIZONTAL] - dA * sqrt(1 - pow((SStartPoint[AXIS_VERTICAL] - SCenterPoint[AXIS_VERTICAL]) / dB, 2));
		if (!isOnTheEllipse(SEndPoint[AXIS_VERTICAL], SEndPoint[AXIS_VERTICAL], SCenterPoint[AXIS_HORIZONTAL], SCenterPoint[AXIS_VERTICAL], dA, dB))
		{
			SCenterPoint[AXIS_HORIZONTAL] = SStartPoint[AXIS_HORIZONTAL] + dA * sqrt(1 - pow((SStartPoint[AXIS_VERTICAL] - SCenterPoint[AXIS_VERTICAL]) / dB, 2));
		}
	}
	else
	{
		dSquareV2subV1 = (SEndPoint[AXIS_VERTICAL] - SStartPoint[AXIS_VERTICAL])* (SEndPoint[AXIS_VERTICAL] - SStartPoint[AXIS_VERTICAL]);
	
		//向量  起点--》终点   //转换为垂直向右向量
		SVector[AXIS_HORIZONTAL] = (SEndPoint[AXIS_VERTICAL] - SStartPoint[AXIS_VERTICAL]);
		SVector[AXIS_HORIZONTAL] = -(SEndPoint[AXIS_HORIZONTAL] - SStartPoint[AXIS_HORIZONTAL]);
	
		//一元二次方程系数
		dC = 4 * dSquareB * (dSquareB * (SStartPoint[AXIS_HORIZONTAL] - SEndPoint[AXIS_HORIZONTAL]) *(SStartPoint[AXIS_HORIZONTAL] - SEndPoint[AXIS_HORIZONTAL]) + dSquareA * dSquareV2subV1);
		dD = 4 * dSquareB * (dSquareB * (SEndPoint[AXIS_HORIZONTAL] * SEndPoint[AXIS_HORIZONTAL] - SStartPoint[AXIS_HORIZONTAL]* SStartPoint[AXIS_HORIZONTAL]) * (SStartPoint[AXIS_HORIZONTAL] - SEndPoint[AXIS_HORIZONTAL]) - dSquareA * dSquareV2subV1 * (SStartPoint[AXIS_HORIZONTAL] + SEndPoint[AXIS_HORIZONTAL]));
		dE = pow(dSquareB * (SEndPoint[AXIS_HORIZONTAL] * SEndPoint[AXIS_HORIZONTAL] - SStartPoint[AXIS_HORIZONTAL] * SStartPoint[AXIS_HORIZONTAL] ) + dSquareA * dSquareV2subV1,2) - 4 * dSquareA * dSquareA * dSquareB * dSquareV2subV1 * (1 - SStartPoint[AXIS_HORIZONTAL]* SStartPoint[AXIS_HORIZONTAL] / dSquareA);

		//% 求根公式
		dDelta = dD *dD - 4 * dC *dE;

		//圆心1 
		SCenterPoint[AXIS_HORIZONTAL] = (-dD + sqrt(dDelta)) / (2 * dC);
		SCenterPoint[AXIS_VERTICAL] = SStartPoint[AXIS_VERTICAL] - dB * sqrt(1 - pow((SStartPoint[AXIS_HORIZONTAL] - SCenterPoint[AXIS_HORIZONTAL])  / dA,2));
		if (!isOnTheEllipse(SEndPoint[AXIS_VERTICAL], SEndPoint[AXIS_VERTICAL], SCenterPoint[AXIS_HORIZONTAL], SCenterPoint[AXIS_VERTICAL],dA,dB))
		{
			SCenterPoint[AXIS_VERTICAL] = SStartPoint[AXIS_VERTICAL] + dB * sqrt(1 - pow((SStartPoint[AXIS_HORIZONTAL] - SCenterPoint[AXIS_HORIZONTAL]) / dA, 2));
		}

		//向量 终点-起点
		dDot = SVector[AXIS_HORIZONTAL] * (SCenterPoint[AXIS_HORIZONTAL]- SStartPoint[AXIS_HORIZONTAL]) 
			+ SVector[AXIS_VERTICAL] * (SCenterPoint[AXIS_VERTICAL] - SStartPoint[AXIS_VERTICAL]);   //内积

		//double dDotSE = dVectorSx * dVectorEx + dVectorSz * dVectorEz;    //向量O'S'和向量O'E'的内积
		if (cArcDir * dDot > 0)
		{
			//圆心2
			SCenterPoint[AXIS_HORIZONTAL] = (-dD - sqrt(dDelta)) / (2 * dC);
			SCenterPoint[AXIS_VERTICAL] = SStartPoint[AXIS_VERTICAL] - dB * sqrt(1 - pow((SStartPoint[AXIS_HORIZONTAL] - SCenterPoint[AXIS_HORIZONTAL]) / dA, 2));
			if (!isOnTheEllipse(SEndPoint[AXIS_VERTICAL], SEndPoint[AXIS_VERTICAL], SCenterPoint[AXIS_HORIZONTAL], SCenterPoint[AXIS_VERTICAL], dA, dB))
			{
				SCenterPoint[AXIS_VERTICAL] = SStartPoint[AXIS_VERTICAL] + dB * sqrt(1 - pow((SStartPoint[AXIS_HORIZONTAL] - SCenterPoint[AXIS_HORIZONTAL]) / dA, 2));
			}
		}
	}
	//圆心旋转坐标系
	rotatingCoordinate(SCenterPoint, 2, SCenterPoint, -dDeltaRad, AXIS_HORIZONTAL, AXIS_VERTICAL);
	
	//传出中心点
	pdCenterPoint[ucHorizontalAxis] = SCenterPoint[AXIS_HORIZONTAL];
	pdCenterPoint[ucVerticalAxis] = SCenterPoint[AXIS_VERTICAL];

	return 1;
}

//圆柱坐标转化
double commonCoord2Cylinder(double dPoint, double dCylinderRadius)
{
	double dPerimeter = 2 * PI * dCylinderRadius;
	if (isZero(dPerimeter))
	{
		return 0;
	}
	return dPoint/DEGREE_OF_CIRCLE * dPerimeter;
}

//圆柱转化普通坐标
double cylinderCoord2Common(double dPoint, double dCylinderRadius)
{
	double dPerimeter = 2 * PI * dCylinderRadius;
	if (isZero(dPerimeter))
	{
		return 0;
	}
	return dPoint * DEGREE_OF_CIRCLE / dPerimeter;
}


/*************************************************
  函数名称：calArcVectorAndDistance(uint8* ucAxisArray, uint32 unCommandType)
  函数功能  根据指令类型选择插补平面   横纵轴要对应   G17 ->XY   G18 ->ZX   G19 ->YZ
  输入参数：ucAxisArray		轴数组  长度必须大于等于2
			unCommandType	指令类型
  输出参数：
  返 回 值：-1 空指针
			1  处理完成
  备    注：
**************************************************/
int8 chooseInterpPlane(uint8* ucAxisArray, uint32 unCommandType)
{
	if (ucAxisArray == NULL) return -1;

	if (getBits(unCommandType, CMD_INTERPOLATION_MODE_ADDR) == CMD_INTERPOLATION_CYLINDRICAL)
	{ //圆柱插补
		switch (getBits(unCommandType, CMD_PLANE_ADDR))//读取指令类型
		{
		case CMD_PLANE_G18: //G18
		default://默认G18
			ucAxisArray[0] = AXIS_Z;
			ucAxisArray[1] = AXIS_C;
			break;
		case CMD_PLANE_G17: //G17
			ucAxisArray[0] = AXIS_X;
			ucAxisArray[1] = AXIS_C;
			break;
		case CMD_PLANE_G19: //G19
			ucAxisArray[0] = AXIS_C;
			ucAxisArray[1] = AXIS_Z;
			break;
		}
	}
	else
	{ //普通插补
		switch (getBits(unCommandType, CMD_PLANE_ADDR))//读取指令类型
		{
		case CMD_PLANE_G18: //G18
		default://默认G18
			ucAxisArray[0] = AXIS_Z;
			ucAxisArray[1] = AXIS_X;
			break;
		case CMD_PLANE_G17: //G17
			ucAxisArray[0] = AXIS_X;
			ucAxisArray[1] = AXIS_Y;
			break;
		case CMD_PLANE_G19: //G19
			ucAxisArray[0] = AXIS_Y;
			ucAxisArray[1] = AXIS_Z;
			break;
		}
	}
	return 1;
}

/*************************************************
  函数名称：double findValue(const int8* const pStartPtr, uint8 ucWord_Len,const uint32 unLineNum)
  函数功能  将字符串转换数值
  输入参数：pStartPtr		指向字符串首字符地址
			ucLen			字符串长度
			unLineNum		行号
  输出参数：
  返 回 值：词法单元的数值

  备    注：
**************************************************/
double stringToValue(const int8* pStartPtr, uint8 ucLen, const uint32 unLineNum)
{
	if (pStartPtr == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	if ((ucLen < 2) || (ucLen > 11))       //词法单元长度需在2到11内
	{
		setErrorType(GCODE_ERROR_NUM85);
		findError(pStartPtr, unLineNum);
		return MAXIMUM_OF_UINT8;
	}

	int8 cDataStrBuf[MAX_LENGTH_OF_STRING + 1] = { 0 };    //字符串数组:存放字符串的数字
	const int8* pTemp = pStartPtr;                               //创建一个跟pStart_Ptr指向一样的指针
	uint8 i = 0;

	for (; i < ucLen; i++)
	{
		cDataStrBuf[i] = *(pTemp + i);
	}
	//atof会出现问题：如"0X100"会输出256，应为0X被识别为16进制，最好用atoi（10进制）
	return atof(cDataStrBuf);        //atof:把字符串的转成浮点数 例如："123"==>123.000000
}

/*************************************************
  函数名称：double findValue(const int8* const pStartPtr, uint8 ucWord_Len,const uint32 unLineNum)
  函数功能  将字符串转换数值
  输入参数：指向char数据类型的指针，pStart_Ptr指向词法单元开头,sWord_Len词法单元长度,unLineNum行号
  输出参数：
  返 回 值：词法单元的数值

  备    注：string -> value
**************************************************/
double findValue(const int8* pStartPtr, uint8 ucWord_Len, const uint32 unLineNum)
{
	if (pStartPtr == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}
	
	int8 cDataStrBuf[MAX_LENGTH_OF_STRING + 1] = { 0 };    //字符串数组:存放字符串的数字
	const int8* pTemp = pStartPtr;                                    //创建一个跟pStart_Ptr指向一样的指针
	uint8 i = 0;

	if ((ucWord_Len < 2) || (ucWord_Len > 11))       //词法单元长度需在2到11内
	{
		setErrorType(GCODE_ERROR_NUM85);
		findError(pStartPtr, unLineNum);
		return MAXIMUM_OF_UINT8;
	}
	pTemp++;                                                  //指针移动一位，跳过字母
	ucWord_Len--;                                              //数字长度为词法单元长度-1
	while (ucWord_Len-- != 0)
	{
		cDataStrBuf[i] = *pTemp;
		*(cDataStrBuf + i) = *pTemp;
		pTemp++;
		i++;
	}

	return atof(cDataStrBuf);        //atof:把字符串的转成浮点数 例如："123"==>123.000000
	//return stringToDouble(cDataStrBuf);  
}

/*************************************************
  函数名称：void eliminateBlank(int8* pString, uint16 usStrSize)
  函数功能  除去字符串里的空格
  输入参数：pString		字符串地址
			usStrSize	字符串大小
  输出参数：
  返 回 值：
  备    注：
**************************************************/
void eliminateBlank(int8* pString, uint16 usStrSize)
{
	int8* pSlow, * pFast;   //快慢指针
	uint16 cnt = 0;
	if (pString == NULL)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return;
	}

	pSlow = pString;
	pFast = pString;
	for (; cnt < usStrSize && *pFast != '\0'; cnt++, pFast++)
	{
		if (*pFast != ' ' && *pFast != '\t')
		{
			*pSlow = *pFast;
			pSlow++;
		}

	}
	*pSlow = '\0';
	pString[usStrSize - 1] = '\0';

}

/*************************************************
  函数名称：void freeGcode(void *pFreeTarget)
  函数功能: 是否为目标字符串
  输入参数：cStr目标字符串，pCurrPtr当前字符的地址
  输出参数：
  返 回 值：
  备    注：不比较第一个字符!
**************************************************/
bool isTagetStr_A(const int8* cStr, const int8* pCurrPtr)
{
	if (!cStr || !pCurrPtr)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	uint8 ucIdx = 1;    //不比较第一个字符
	size_t len = strlen(cStr);
	while (ucIdx < len)
	{
		if (*(pCurrPtr + ucIdx) != cStr[ucIdx])
		{
			return FALSE;
		}
		ucIdx++;
	}
	return TRUE;
}


/*************************************************
  函数名称：bool isTagetStr_B(const int8 *cStr, const int8 *pCurrPtr)
  函数功能: 是否为目标字符串
  输入参数：cStr目标字符串，pCurrPtr当前字符的地址
  输出参数：
  返 回 值：
  备    注：比较第一个字符!
**************************************************/
bool isTagetStr_B(const int8* cStr, const int8* pCurrPtr)
{
	if (!cStr || !pCurrPtr)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM300);
		return -1;
	}

	uint8 ucIdx = 0;    //比较第一个字符
	size_t len = strlen(cStr);
	while (ucIdx < len)
	{
		if (*(pCurrPtr + ucIdx) != cStr[ucIdx])
		{
			return FALSE;
		}
		ucIdx++;
	}
	return TRUE;
}

/*************************************************
  函数名称：char* itoaNC(int value, char* str, int radix)
  函数功能: itoa（整型转字符串）的实现，单片机没有这个函数
  输入参数：value数字，str目标字符串，radix进制
  输出参数：
  返 回 值：str目标字符串
  备    注：
**************************************************/
char* itoaNC(int value, char* str, int radix)
{
	static char dig[] =
		"0123456789"
		"abcdefghijklmnopqrstuvwxyz";
	int n = 0, neg = 0;
	unsigned int v;
	char* p, * q;
	char c;
	/* 负数：十进制直接在前面加负号，其它进制用补码 */
	if (radix == 10 && value < 0)
	{
		v = -value;
		neg = 1;//负号标志位为1，表示为负数
	}
	else
	{
		v = value;
	}

	/* 其它进制的负数转为unsigned int类型，即取了补码 */
	do
	{
		str[n++] = dig[v % radix];//v%radix就是取最后一个的数字，然后这个数字对应数组的位置刚好就是对应字符
		v /= radix;//v /= radix 除去最后一个数字
	} while (v);

	if (neg)
		str[n++] = '-'; //补上负号

	str[n] = '\0';//截至符号
	/* 字符串反转 */
	for (p = str, q = p + (n - 1); p < q; ++p, --q)//翻转字符串 从索引0 到索引（n-1）
		c = *p, * p = *q, * q = c;
	return str;
}
