#include "common.h"
#include "ringbuff.h"
#include"ErrorFunction.h"
#include "GCode.h"

/*
代码格式：G92 X(U)_ Z(W)_ F_ J_ K_ L ； (公制直螺纹切削循环)
G92 X(U)_ Z(W)_ I_ J_ K_ L ； (英制直螺纹切削循环)
G92 X(U)_ Z(W)_ R_ F_ J_ K_ L ； (公制锥螺纹切削循环)
G92 X(U)_ Z(W)_ R_ I_ J_ K_ L ； (英制锥螺纹切削循环)

代码说明：G92 为模态 G 代码；
切削起点：螺纹插补的起始位置；
切削终点：螺纹插补的结束位置；
X：切削终点 X 轴绝对坐标；
U：切削终点与起点 X 轴绝对坐标的差值；
Z：切削终点 Z 轴绝对坐标；
W：切削终点与起点 Z 轴绝对坐标的差值；
R：切削起点与切削终点 X 轴绝对坐标的差值(半径值)，当 R 与 U 的符号不一致时，要求∣R│≤│U/2│；
F：螺纹导程，取值范围见表 1-9，F 指定值执行后保持，可省略输入；
I：螺纹每英寸牙数，取值范围见表 1-9，I 指定值执行后保持，可省略输入；
J：螺纹退尾时在短轴方向的移动量，取值范围 0~99999999×最小输入增量，不带方向(根据程序起点位置自动确定退尾方向)，模态参数，如果短轴是 X 轴，则该值为半径指定；
K：螺纹退尾时在长轴方向的长度，取值范围 0~99999999×最小输入增量，。不带方向，模态参数，如长轴是 X 轴，该值为半径指定；
L：多头螺纹的头数，该值的范围是：1～99，模态参数。(省略 L 时默认为单头螺纹)
● 省略J、K时，按NO.19号参数设定值退尾；
● 省略J时，长轴方向按K退尾，短轴方向按NO.19号参数设定值退尾；
● 省略K时，按J=K退尾；
● J=0或J=0、K=0时，无退尾；
● J≠0，K=0时，按K=J退尾；
● J=0，K≠0时，无退尾；
    J、K输入负值时，按正值处理；
*/


static SG92Para* pSG92ParaCheck[CHANNEL_NUM_MAX];	//检查流程使用
static SG92Para* pSG92ParaAssign[CHANNEL_NUM_MAX];	//赋值流程使用



//添加G92规则结构体
void addG92RegulationAPI()
{
	//指令：G92
	//组别：sGa
	//指令占程序段数：1
	addGCommadRegulation(92, sGa, 1, mallocG92ParaStruct, freeG92ParaStruct, getG92Para, CheckG92, AssignG92);
}

/*************************************************
  函数名称：int8 mallocG92ParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：开辟G92参数结构体（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：1.由于各通道的检查赋值同时进行，防止参数结构体的使用冲突，必须根据通道号与解析模式进行开辟
			2.开辟后同时进行初始化
**************************************************/
int8 mallocG92ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG92ParaCheck[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG92ParaCheck[ucChannelNum] = mallocGCode(sizeof(SG92Para), ORAM_NUM446);//暂未对内存编号
		if (pSG92ParaCheck[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG92ParaCheck[ucChannelNum], 0, sizeof(SG92Para));
		pSG92ParaCheck[ucChannelNum]->ucL = 1;
		return 1;
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG92ParaAssign[ucChannelNum] != NULL)
		{
			return 1;
		}
		pSG92ParaAssign[ucChannelNum] = mallocGCode(sizeof(SG92Para), ORAM_NUM447);
		if (pSG92ParaAssign[ucChannelNum] == NULL)
		{
			return 0;
		}
		memset(pSG92ParaAssign[ucChannelNum], 0, sizeof(SG92Para));
		pSG92ParaAssign[ucChannelNum]->ucL = 1;    //默认单头   zxd:20251031
		return 1;
	}
}

/*************************************************
  函数名称：int8 freeParaStruct(uint8 ucMode, uint8 ucChannelNum)
  函数功能：检查cGa、cGz程序段（总框架）
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 freeG92ParaStruct(uint8 ucMode, uint8 ucChannelNum)
{
	if (ucMode == GC_SYNTAX)
	{
		if (pSG92ParaCheck[ucChannelNum] != NULL)
		{
			freeGCode(pSG92ParaCheck[ucChannelNum]);
			pSG92ParaCheck[ucChannelNum] = NULL;
		}
	}
	else if (ucMode == GC_ASSIGN)
	{
		if (pSG92ParaAssign[ucChannelNum] != NULL)
		{
			freeGCode(pSG92ParaAssign[ucChannelNum]);
			pSG92ParaAssign[ucChannelNum] = NULL;
		}
	}
}

/*************************************************
  函数名称：int8 getG92Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
  函数功能：获取G92参数
  输入参数：ucMode			模式（检查模式：0 赋值模式：1）
			ucChannelNum	通道号
			SScan* pSScanContainer  扫描结构体
			SGCodeSysState* SGCodeCurrSysState  通道状态结构体
  输出参数：
  返 回 值：
  备    注：
**************************************************/
int8 getG92Para(uint8 ucMode, SScan* pSScanContainer, uint8 ucChannelNum, SGCodeSysState* SGCodeCurrSysState)
{
	if (pSScanContainer->bF == TRUE && pSScanContainer->bRIJK[1] == TRUE)
		 {
		parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM410);
		return -1;
		}

	 if (pSScanContainer->bL &&
		(!isfinite(pSScanContainer->dL) || pSScanContainer->dL < 1.0 ||pSScanContainer->dL > 99.0 || pSScanContainer->dL != floor(pSScanContainer->dL)))
		 {
		parseGCodeErrorB(pSScanContainer->unLineNum, GCODE_ERROR_NUM130);
		return -1;
		}

	if (ucMode == GC_SYNTAX)
	{
		pSG92ParaCheck[ucChannelNum]->cStep = 0;
		//1.获取R
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->dR = pSScanContainer->dRIJK[0];
		}

		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG92ParaCheck[ucChannelNum]->SCoordInfo, pSScanContainer);

		//3.手册要求,获取短轴退尾量dJ 、 长轴退尾量dK
	     //3.1 G92省略退尾时，有默认退尾值，为面板上的默认参数
		if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG92ParaCheck[ucChannelNum]->dJ = g_SSystemPara.dJDefault;
			pSG92ParaCheck[ucChannelNum]->dK = g_SSystemPara.dKDefault;
		}
		//3.2 省略J时，长轴方向按K退尾，短轴方向数据参数026号设定值退尾；
		else if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->dJ = g_SSystemPara.dJDefault;
			pSG92ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
		}
		//3.3  省略K时，按J=K退尾；
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG92ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
			pSG92ParaCheck[ucChannelNum]->dK = pSG92ParaCheck[ucChannelNum]->dJ;
		}
		//3.4  J和K都不省略时
		else if (pSScanContainer->bRIJK[2] == TRUE && pSScanContainer->bRIJK[3] == TRUE)
		{
			//3.4.1 J=0或J=0、K=0时，无退尾； J=0，K≠0时，无退尾；-》   即J=0时无退尾
			if (isZero(pSScanContainer->dRIJK[2] == TRUE))
			{
				pSG92ParaCheck[ucChannelNum]->dJ = 0;
				pSG92ParaCheck[ucChannelNum]->dK = 0;
			}
			//3.4.2 J≠0，K=0时，按K=J退尾；
			else if (isZero(pSScanContainer->dRIJK[2] == FALSE) && isZero(pSScanContainer->dRIJK[3] == TRUE))
			{
				pSG92ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG92ParaCheck[ucChannelNum]->dK = pSG92ParaCheck[ucChannelNum]->dJ;
			}
			else//3.4.3 J≠0,K≠0时
			{
				pSG92ParaCheck[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG92ParaCheck[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
			}
		}
		//4.获取起始角度dQ  输入的Q为0~360000（单位为0.001度）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->dQ = pSScanContainer->dQ;
		}
		//5.获取导程F  (F与I互斥，先获取导程值（这里为值覆盖），后面再判断互斥性)
		if (pSScanContainer->bF == TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->dF = pSScanContainer->dF;
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
		}
		//6.获取螺纹头数
		//出现L
		if (pSScanContainer->bL==TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->ucL = (uint8)(pSScanContainer->dL);
		}

		//7.初始化当前的螺纹头数
		pSG92ParaCheck[ucChannelNum]->ucLNow = 0;
		//8.判断是否执行：程序段中出现X或U
		if (pSScanContainer->bXYZ[0]==TRUE||pSScanContainer->bUVW[0]==TRUE)
		{
			pSG92ParaCheck[ucChannelNum]->bExecute = TRUE;
		}

	}
	else if (ucMode == GC_ASSIGN)
	{
		pSG92ParaAssign[ucChannelNum]->cStep = 0;

		//1.获取R
		if (pSScanContainer->bRIJK[0] == TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->dR = pSScanContainer->dRIJK[0];
		}

		//2.获取坐标信息
		getCoordInfoFromScanInfo(&pSG92ParaAssign[ucChannelNum]->SCoordInfo, pSScanContainer);

		//3.手册要求,获取短轴退尾量dJ 、 长轴退尾量dK
		//3.1 G92省略退尾时，有默认退尾值，为面板上的默认参数
		if (pSScanContainer->bRIJK[2] == FALSE && pSScanContainer->bRIJK[3] == FALSE)
		{
			if (isZero(pSG92ParaAssign[ucChannelNum]->dJ == TRUE) && isZero(pSG92ParaAssign[ucChannelNum]->dK == TRUE))
			{
				pSG92ParaAssign[ucChannelNum]->dJ = g_SSystemPara.dJDefault;
				pSG92ParaAssign[ucChannelNum]->dK= g_SSystemPara.dKDefault;
			}
		}
		//3.2 省略J时，长轴方向按K退尾，短轴方向数据参数026号设定值退尾；
		else if (pSScanContainer->bRIJK[2] == FALSE&& pSScanContainer->bRIJK[3] ==TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->dJ = g_SSystemPara.dJDefault;
			pSG92ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
		}
		//3.3  省略K时，按J=K退尾；
		else if (pSScanContainer->bRIJK[2] ==TRUE && pSScanContainer->bRIJK[3] == FALSE)
		{
			pSG92ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
			pSG92ParaAssign[ucChannelNum]->dK = pSG92ParaAssign[ucChannelNum]->dJ;
		}
		//3.4  J和K都不省略时
		else if (pSScanContainer->bRIJK[2] == TRUE&&pSScanContainer->bRIJK[3]==TRUE)
		{
			//3.4.1 J=0或J=0、K=0时，无退尾； J=0，K≠0时，无退尾；-》   即J=0时无退尾
			if (isZero(pSScanContainer->dRIJK[2]==TRUE))
			{
				pSG92ParaAssign[ucChannelNum]->dJ = 0;
				pSG92ParaAssign[ucChannelNum]->dK = 0;
			}
			//3.4.2 J≠0，K=0时，按K=J退尾；
			else if (isZero(pSScanContainer->dRIJK[2] == FALSE)&& isZero(pSScanContainer->dRIJK[3] == TRUE))
			{
				pSG92ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG92ParaAssign[ucChannelNum]->dK = pSG92ParaAssign[ucChannelNum]->dJ;
			}
			else
			{
				pSG92ParaAssign[ucChannelNum]->dJ = pSScanContainer->dRIJK[2];
				pSG92ParaAssign[ucChannelNum]->dK = pSScanContainer->dRIJK[3];
			}

		}

		//4.获取起始角度dQ  输入的Q为0~360000（单位为0.001度）
		if (pSScanContainer->bQ == TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->dQ = pSScanContainer->dQ;
		}
		//5.获取导程F  (F与I互斥，先获取导程值（这里为值覆盖），后面再判断互斥性)
		if (pSScanContainer->bF == TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->dF = pSScanContainer->dF;
		}
		else if (pSScanContainer->bRIJK[1] == TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->dF = 25.4 / pSScanContainer->dRIJK[1];//I作为螺纹每英寸牙数时，导程为F=25.4/I,将I转成参数F
		}
		//6.获取螺纹头数
		//出现L
		if (pSScanContainer->bL == TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->ucL = (uint8)(pSScanContainer->dL);
		}


		//7.初始化当前的螺纹头数
		pSG92ParaAssign[ucChannelNum]->ucLNow = 0;

		//8.判断是否执行：程序段中出现X或U
		if (pSScanContainer->bXYZ[0] == TRUE || pSScanContainer->bUVW[0] == TRUE)
		{
			pSG92ParaAssign[ucChannelNum]->bExecute = TRUE;
		}

	}
}


///*************************************************
//  函数名称：int8 CheckG92(struct _SSyntaxConfiguration* pSSyntaxConfig)
//  函数功能：检查G92
//  输入参数： struct _SSyntaxConfiguration* pSSyntaxConfig 检查结构体		
//  输出参数：
//  返 回 值：
//  备    注：
//**************************************************/
int8 CheckG92(struct _SSyntaxConfiguration* pSSyntaxConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SG92Para* pSG92Para = pSG92ParaCheck[pSSyntaxConfig->ucChannelNum];

	if (pSSyntaxConfig == NULL)
	{
		//报错
		return -1;
	}
	bool bRet = 0;
	bRet = isSegmentHaveCoordInfo(&pSSyntaxConfig->SScanContainer);
	if (bRet == TRUE)
	{
		int8 cRet = 0;
		//1.指令字符检查
		checkParaLetter("XYZUVWRJKLFI", &pSSyntaxConfig->SScanContainer);

		//2.参数值范围检查
		checkParaRange('X', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Y', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('Z', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('U', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('V', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('W', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('R', &pSSyntaxConfig->SScanContainer, -99999999, 99999999);
		checkParaRange('J', &pSSyntaxConfig->SScanContainer, -99999999 * TEMP_LEAST_INPUT_INCREMENT, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
		checkParaRange('K', &pSSyntaxConfig->SScanContainer, -99999999 * TEMP_LEAST_INPUT_INCREMENT, 99999999 * TEMP_LEAST_INPUT_INCREMENT);
		checkParaRange('L', &pSSyntaxConfig->SScanContainer, 1, 99);
		checkParaRange('F', &pSSyntaxConfig->SScanContainer, 0.001, 500);
		checkParaRange('I', &pSSyntaxConfig->SScanContainer, 0.06, 25400);


		//3.当R与W的符号不同时，要求│R│≤│U/2│
		SPointDouble* pSG92PointA = NULL; //起点
		SPointDouble  pSG92PointC = { 0 };//切削终点
		double dG92R;//切削起点与切削终点Z 轴绝对坐标的差值，
		double dG92U;//切削终点与起点X 轴绝对坐标的差值；
		double dG92W;//切削终点与起点Z轴绝对坐标的差值

		bool bIsAbsMethod;
		pSG92PointA = &pSSyntaxConfig->SGCodeCurrSysState.SCurrPoint;
		bIsAbsMethod = isAbsProgrammingMethod(GC_SYNTAX, pSSyntaxConfig->ucChannelNum);

		//计算C点的机械坐标
		getMacCoordByCoordInfo(pSSyntaxConfig->ucChannelNum, &pSG92ParaCheck[pSSyntaxConfig->ucChannelNum]->SCoordInfo, pSG92PointA, &pSG92PointC, &pSSyntaxConfig->SGCodeCurrSysState);

		dG92R = pSG92ParaCheck[pSSyntaxConfig->ucChannelNum]->dR;

		dG92U = pSG92PointC.dArray[AXIS_X] - pSG92PointA->dArray[AXIS_X];

		dG92W = pSG92PointC.dArray[AXIS_Z] - pSG92PointA->dArray[AXIS_Z];

		//当 R 与 U 的符号不一致时
		if (dG92R * dG92U <= 0)
		{
			if (fabs(dG92U / 2) < fabs(dG92R))
			{
				const int8* pStartPtr = pSSyntaxConfig->SGetTextStateInfo.cStr;
				while (*pStartPtr != 'r' && *pStartPtr != 'R')
				{
					pStartPtr += 1;
				}
				setErrorType(GCODE_ERROR_NUM411);
				findError(pStartPtr, pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum);
				return -1;
			}
		}

		//4.G92不能够加工端面螺纹(即不能X坐标移动，Z坐标不变化)，如果X轴坐标发生变化，Z必须发生变化,否则报错
		if (isZero(dG92U) == FALSE && isZero(dG92W) == TRUE)
		{
			parseGCodeErrorB(pSSyntaxConfig->SGetTextStateInfo.unCurrLineNum, GCODE_ERROR_NUM414);

		}

		//5.检查长轴以当前的最大速度能否追上主轴

		uint8 ucHorAxis, ucVerAxis;   //横轴，纵轴
		SPointDouble SStartRPoint = *pSG92PointA;//R点
		SStartRPoint.dArray[AXIS_X] += dG92R;
		//获取横轴、纵轴
		cRet = getHorAndVerAxis(&ucHorAxis, &ucVerAxis, pSSyntaxConfig->SGCodeCurrSysState.cInterpolationState, pSSyntaxConfig->SCurrRecord.SModalInfo.sGc);
		if (cRet < 0)
		{
			return -1;
		}
		cRet = checkG32Speed(pSSyntaxConfig, pSG92Para->dF, SStartRPoint, pSG92PointC, ucHorAxis, ucVerAxis);
		if (cRet != 1)
		{
			return -1;
		}
		//6.赋值检查的最终终点
		pSSyntaxConfig->SCurrRecord.SEndPoint = *pSG92PointA;
		return 2;
	}
	return 2;
}


/*************************************************
  函数名称：int8 AssignG92(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
  函数功能：G92轨迹赋值
  输入参数：struct _SAssignConfiguration* pSAssignConfig 赋值结构体
			struct SGCodeParse* pGCodeBufUnit
  输出参数：
  返 回 值：0	赋值预处理（未写G代码缓冲区）
			1	写了，没写完
			2	赋值完成
  备    注：
**************************************************/
int8 AssignG92(struct _SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	SPointDouble* pSG92PointA = NULL;
	SPointDouble* pSG92PointC = NULL;
	double dG92R = pSG92ParaAssign[pSAssignConfig->ucChannelNum]->dR;
	SG92Para* pSG92Para = pSG92ParaAssign[pSAssignConfig->ucChannelNum];
	pSG92PointA = &pSG92ParaAssign[pSAssignConfig->ucChannelNum]->SPointA;
	pSG92PointC = &pSG92ParaAssign[pSAssignConfig->ucChannelNum]->SPointC;


	switch (pSG92Para->cStep)
	{
		//0.赋值预处理（整理参数结构体数据）
	case 0:
	{
		int8 cRet = 0;
		bool bIsAbsMethod;
		//0.判断G92程序段中是否出现坐标信息
		cRet = isSegmentHaveCoordInfo(&pSAssignConfig->SScanContainer);
		//0.1程序段出现了坐标信息X或U才赋值
		if (pSG92Para->bExecute == TRUE&&cRet==TRUE)
		{    
			//1.计算C点机械坐标
			//1.1获取G92的A点坐标
			*pSG92PointC = *pSG92PointA = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;
			//1.2获取G92的C点坐标
			//获取当前的坐标编程方式
			bIsAbsMethod = isAbsProgrammingMethod(GC_ASSIGN, pSAssignConfig->ucChannelNum);
			//计算C点的机械坐标
			getMacCoordByCoordInfo(pSAssignConfig->ucChannelNum, &pSG92Para->SCoordInfo, pSG92PointA, pSG92PointC, &pSAssignConfig->SGCodeCurrSysState);
			pSG92Para->cStep = 1;
		}
		//0.2程序段未出现坐标信息X或U不赋值	
		else
		{
			return 2;
		}
		return 0;
		break;
	}

	//写入AB段:G0
	case 1:
	{
		//1.填写SLine
		SLine SLineInfo_1;
		memset(&SLineInfo_1, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_1.SEndPoint = SLineInfo_1.SStartPoint = pSG92ParaAssign[pSAssignConfig->ucChannelNum]->SPointA;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_1.SEndPoint.dArray[AXIS_X] = pSG92PointC->dArray[AXIS_X] + dG92R;
		//1.3获取SLine的实体类型
		SLineInfo_1.EntityType = OTHERTYPE;//快速运动

		//2.将SLine_1写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_1, pGCodeBufUnit);

		pSG92Para->cStep = 2;

		return 1;
	}

	//写入BC段 :G32
	case 2:
	{
		//1.填写SThreadInfo
		SThread SThreadInfo;//存螺纹段信息（可能要将此变量放在赋值结构体中）
		double dQDelta;  //Q值的Delta增量
		double dVectorJ; //短轴方向的退尾向量
		memset(&SThreadInfo, 0, sizeof(SThread));
		//1.1初始化SThreadInfo的起点终点（机械坐标）
		SThreadInfo.SEndPoint = SThreadInfo.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SThreadInfo的终点（机械坐标）
		SThreadInfo.SEndPoint = *pSG92PointC;

		//1.3 获取退尾量J K
		//计算短轴方向的退尾向量
		if (pSG92PointC->dArray[AXIS_X] >= pSG92PointA->dArray[AXIS_X])  //切削终点X轴方向的位置 > 起点X轴方向的位置（短轴退尾方向指向起点）
		{
			dVectorJ = -1;

		}
		else
		{
			dVectorJ = 1;

		}
		SThreadInfo.dJ = dVectorJ * pSG92Para->dJ;  
		SThreadInfo.dK = pSG92Para->dK;

		//1.4  当前切削的头数+1
		pSG92Para->ucLNow++;

		//1.5 获取获取起始角 Q以及 是否出现Q的标志位
		dQDelta = 360000 / pSG92Para->ucL;   //获取Q值的Delta增量
		SThreadInfo.dQ = dQDelta * (pSG92Para->ucLNow - 1);
		SThreadInfo.bG32LetterQ=TRUE;

	    //1.6  连续螺纹标志位
		SThreadInfo.bRepeatG32Flag = FALSE;

		//1.7 获取G32子类
		SThreadInfo.ucG32SubMode = 0;

		//1.8 写入实体类型
		SThreadInfo.EntityType = THREAD;

		//1.9 写入导程
		SThreadInfo.dF = pSG92Para->dF;

		//1.10.将SThread写入缓冲区
		writeThreadIntoGCodeBuf(&SThreadInfo, pGCodeBufUnit);
	
		pSG92Para->cStep = 3;

		return 1;
	}

	//写入CD段 :G00
	case 3:
	{

		//1.填写SLine
		SLine SLineInfo_3;
		memset(&SLineInfo_3, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_3.SEndPoint = SLineInfo_3.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_3.SEndPoint.dArray[AXIS_X] = pSG92PointA->dArray[AXIS_X];

		SLineInfo_3.EntityType = OTHERTYPE;//快速运动

	   //将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_3, pGCodeBufUnit);

		pSG92Para->cStep = 4;

		return 1;
	}

	//写入DA段 :G00
	case 4:
	{
		//1.填写SLine
		SLine SLineInfo_4;
		memset(&SLineInfo_4, 0, sizeof(SLine));
		//1.1初始化SLine的起点终点（机械坐标）
		SLineInfo_4.SEndPoint = SLineInfo_4.SStartPoint = pSAssignConfig->SGCodeCurrSysState.SCurrPoint;//系统状态结构体的当前点
		//1.2获取SLine的终点（机械坐标）
		SLineInfo_4.SEndPoint = *pSG92PointA;


		SLineInfo_4.EntityType = OTHERTYPE;//G0

	    //将SLine写入缓冲区
		writeLineIntoGCodeBuf(&SLineInfo_4, pGCodeBufUnit);

       //判断G92是否加工完毕
		//螺纹头数达到要求
		if (pSG92Para->ucLNow==pSG92Para->ucL)
		{
			pSG92Para->cStep = 0;
			return 2;
		}
		//螺纹头数还没达到要求
		else
		{
			pSG92Para->cStep = 1;
			return 1;
		}
		
	}

	default:
		break;
	}
}