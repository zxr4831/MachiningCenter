#include<stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
//#include <cstring>
#include"Gcode.h"
#include"ErrorFunction.h"
#include"GeneralFunctions.h"
#include "Config.h"
#include "ringbuff.h"

#if STM32_MODE
#include "flash_task.h"
#endif


uint8 g_ucNameType = 1;     //1：加O ，0：不加O    测试子程序调用测试必须置为1
#if !STM32_MODE
//int8  g_cMapBufferS0[FILESIZE];      //通道0
//int8  g_cMapBufferS1[1234];      //通道1

static FILE *s_p0, *s_p1;//存放输出文件的指针
static int32 s_idx_0, s_idx_1;//当前输出行号


int8 g_cName_C0[20];    //通道0的主程序名
int8 g_cName_C1[20];    //通道1的主程序名


int8 g_textBuf[20][MAX_LENGTH_OF_LINE];//存放一行字符串
bool s_bIsWritable[20];  //1为可写 0为不可写


//给文件名增加路径和后缀,用于打开文件
void addPathVsFileName(int8 cOutput[50], int8 *pInput)
{
	uint8 ucIdx;
	memset(cOutput, 0, 50);
	if (g_ucNameType == 1 && MDI_MODE != getRunMode())
	{
		strcpy(cOutput, "./resource/O");
	}
	else
	{
		strcpy(cOutput, "./resource/");
	}
	ucIdx = (uint8)strlen(cOutput);
	for (uint8 i = 0; pInput[i] != 0; ucIdx++, i++)
	{
		cOutput[ucIdx] = pInput[i];
	}
	cOutput[ucIdx++] = '.';
	cOutput[ucIdx++] = 't';
	cOutput[ucIdx++] = 'x';
	cOutput[ucIdx++] = 't';
	cOutput[ucIdx] = 0;
}

//打开文件
void GCODE_OPEN_FILE(uint8 ucChannelNum)
{
	FILE* file1 = NULL;
	switch (ucChannelNum)
	{
	case 0:
		file1 = fopen("./outputFile/Gnc0.txt", "w");  //只写方式打开，会清空
		break;
	case 1:
		file1 = fopen("./outputFile/Gnc1.txt", "w");
		break;
	default:
		break;
	}

	if (!file1)
	{
		fclose(file1);
		exit(0);
	}
	fclose(file1);

	switch (ucChannelNum)
	{
	case 0:
		file1 = fopen("./outputFile/Gnc0.txt", "a");  //追加方式方式打开
		break;
	case 1:
		file1 = fopen("./outputFile/Gnc1.txt", "a");
		break;
	default:
		break;
	}
	
	if (!file1)
	{
		fclose(file1);
		exit(0);
	}

	switch (ucChannelNum)
	{
	case 0:
		s_p0 = file1;
		break;
	case 1:
		s_p1 = file1;
		break;
	default:
		break;
	}

	
}

//关闭文件
void GCODE_CLOSE_FILE(uint8 ucChannelNum)
{
	FILE* file1 = NULL;
	switch (ucChannelNum)
	{
	case 0:
		file1 = s_p0;
		break;
	case 1:
		file1 = s_p1;
		break;
	default:
		break;
	}

	if (file1)
	{
		fclose(file1);
		file1 = NULL;
	}
	switch (ucChannelNum)
	{
	case 0:
		s_p0 = file1;
		break;
	case 1:
		s_p1 = file1;
		break;
	default:
		break;
	}
}

 //初始化测试文本缓冲区
void initTextBuf()
{
	memset(g_textBuf, 0, MAX_LENGTH_OF_LINE * 20);
	memset(s_bIsWritable, 1, 20);
}

//初始化指定缓冲区
void initDesinatedBuf(uint8 ucIdx)
{
	memset(g_textBuf[ucIdx], 0, MAX_LENGTH_OF_LINE);
	s_bIsWritable[ucIdx] = 1;
}



//按指定槽位追加已定型轨迹，供倒圆后的延迟打印使用。
/*************************************************
  函数名称：void printGCodeAt(int16 idx, uint8 ucChannelNum)
  函数功能：按指定环形缓冲索引输出轨迹或插补日志。
  输入参数：idx 槽位索引，可越界回绕；ucChannelNum 通道号。
  输出参数：对应通道日志及打印序号；无有效文件或通道时直接返回。
  返 回 值：无。
  备    注：轨迹模式调用方须保证该段已定型；模态格式适配只改显示副本，不改执行记录。
**************************************************/
void printGCodeAt(int16 idx, uint8 ucChannelNum)
{
    FILE* file1=NULL;
    idx=(idx % GCODEPARSEBUF_MAX + GCODEPARSEBUF_MAX) % GCODEPARSEBUF_MAX;
	int32 *lineNum  = NULL;
	switch (ucChannelNum)
	{
	case 0:
		file1 = s_p0;
		lineNum = &s_idx_0;
		break;
	case 1:
		file1 = s_p1;
		lineNum = &s_idx_1;
		break;
	default:
		break;
	}
	if (!file1 || !lineNum) return;
	*lineNum += 1;
	//fprintf(file1, "行号%d\n", g_SRingBuff[ucChannelNum].SGCodeParseBuf[idx].unGCodeLineNum);

#if PRINTMODE == 1
	//打印G代码缓冲区单元内容
	{
        struct SGCodeParse display=g_SRingBuff[ucChannelNum].SGCodeParseBuf[idx];
        /* 旧文本格式化器需要解析阶段编码；仅在显示副本中放大，禁止改原槽位。 */
        if (display.SModalInfo.sGc >= 17 && display.SModalInfo.sGc <= 19) {
            display.SModalInfo.sGa *= 10; display.SModalInfo.sGb *= 10;
            display.SModalInfo.sGc *= 10; display.SModalInfo.sGd *= 10;
            display.SModalInfo.sGe *= 10; display.SModalInfo.sGf *= 10;
            display.SModalInfo.sGg *= 10; display.SModalInfo.sGh *= 10;
        }
        printGCodeBuffUnit(&display,*lineNum);
    }
#elif PRINTMODE == 2
	//打印G代码NC文件
	printGCodeNCFile(&g_SRingBuff[ucChannelNum].SGCodeParseBuf[idx], 0);
#elif PRINTMODE == 3
	//打印运动插补点NC文件
	printInterpolatePointNCFile(*lineNum);


#endif

}

#endif


/*************************************************
  函数名称：uint8 tryToGetLine(int8 *pName, uint32 unTargetLineNum)
  函数功能: 获取目标文本的某一行
  输入参数：pName文件名，unTargetLineNum目标行号
  输出参数：
  返 回 值：返回的数组下标0-19共20个
			返回 0xF1时需要等待
			返回 0XF2时超出范围
			返回 0XF3时出错
  备    注：例如:需要打开./resouce/O0001.txt, pName只需要输入0001（g_ucNameType为1时）或O0001（g_ucNameType为0时）
**************************************************/
uint8 tryToGetLine(int8* pName, uint32 unTargetLineNum, int8 cProgramListSize, uint8 ucChannelNum)
{
	uint32 unWriteIdx = 0;
	FILE* pFile = NULL;

	uint32  unCurrLine = 0;
	int8 cPathName[50];
	if (!pName)
	{
		parseGCodeErrorB(0, GCODE_ERROR_NUM89);
		return 0xF3;  //出错
	}


#if STM32_MODE
	memset(cPathName, 0, 50);

	addPathSTM32FileName(cPathName, pName, cProgramListSize);  //如果pName输入0001，cPathName等于O0001.TXT



	return Flash_listRe((char*)cPathName, unTargetLineNum, ucChannelNum);


#else
	for (; unWriteIdx < 20; unWriteIdx++)
	{
		if (s_bIsWritable[unWriteIdx] == TRUE)
		{
			break;
		}
	}
	if (unWriteIdx >= 20)
	{
		return 0xF1;  //需要等待
	}
	else
	{
		int8 cAStatamentArray[MAX_LENGTH_OF_LINE];    //一行文本字符串

		memset(cPathName, 0, 50);
		memset(cAStatamentArray, 0, MAX_LENGTH_OF_LINE);

		addPathVsFileName(cPathName, pName);  //增加文件路径和后缀
		pFile = fopen(cPathName, "r");

		if (!pFile)
		{
			int errNum = errno;
			printf("open fail errno = %d reason =  \n", errno);
			parseGCodeErrorB(0, GCODE_ERROR_NUM89);
			return 0xF3;   //出错
		}

		for (; unCurrLine <= unTargetLineNum; unCurrLine++)
		{
			if (!fgets(cAStatamentArray, MAX_LENGTH_OF_LINE, pFile))
			{
				memset(cAStatamentArray, 0, MAX_LENGTH_OF_LINE);
				fclose(pFile);
				return 0xF2;  //超出范围
			}
		}

		//能到这里就是获取成功
		g_textBuf[unWriteIdx];//写入缓冲
		memset(g_textBuf[unWriteIdx], 0, MAX_LENGTH_OF_LINE);
		strcpy(g_textBuf[unWriteIdx], cAStatamentArray);
		s_bIsWritable[unWriteIdx] = FALSE;  //置为不可写
		fclose(pFile);
		return unWriteIdx;

	}
#endif

}



/*************************************************
  函数名称：void addPathSTM32FileName(int8 cOutput[50], int8 *pInput)
  函数功能: 单片机环境下添加修改文件名
  输入参数：pInput输入字符串
  输出参数：cOutput输出字符串
  返 回 值：
  备    注：因为第一次是界面传过来是完整文件名，不用加前后缀，只有子程序需要
**************************************************/
void addPathSTM32FileName(int8 *cOutput, int8 *pInput, int8 cProgramListSize)
{
	uint8 ucIdx;
#if STM32_MODE
	if (cProgramListSize < 0)//子程序 需要先加字符'O'
	{
#endif
		strcpy((char*)cOutput, "O");
#if STM32_MODE
	}
#endif
	ucIdx = (uint8)strlen((char*)cOutput);
	for (uint8 i = 0; pInput[i] != 0; ucIdx++, i++)
	{
		cOutput[ucIdx] = pInput[i];
	}
#if STM32_MODE
	if (cProgramListSize < 0)//子程序 需要先加字符'O'
	{
#endif
		cOutput[ucIdx++] = '.';
		cOutput[ucIdx++] = 'T';
		cOutput[ucIdx++] = 'X';
		cOutput[ucIdx++] = 'T';
#if STM32_MODE
	}
#endif
	cOutput[ucIdx] = 0;
}





#if !STM32_MODE
/*************************************************
  函数名称：
  函数功能: 打印G代码缓冲区数据
  输入参数：nNo		编号
  输出参数：
  返 回 值：
  备    注：lxw打印G代码缓冲区单元数据
**************************************************/
void printGCodeBuffUnit(struct SGCodeParse* pSGCodeParseUnit, int32 nNo)
{
	//1.打印缓冲区序号
	fprintf(s_p0, "缓冲区单元%d：（行号：%d）\n", nNo, pSGCodeParseUnit->unGCodeLineNum);

	//2.打印缓冲区状态
	{
		int8 cGa = pSGCodeParseUnit->SModalInfo.sGa / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGb = pSGCodeParseUnit->SModalInfo.sGb / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGc = pSGCodeParseUnit->SModalInfo.sGc / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGd = pSGCodeParseUnit->SModalInfo.sGd / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGe = pSGCodeParseUnit->SModalInfo.sGe / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGf = pSGCodeParseUnit->SModalInfo.sGf / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGg = pSGCodeParseUnit->SModalInfo.sGg / (pow(10, GCOMMAND_DECIMAL_PLACE));
		int8 cGh = pSGCodeParseUnit->SModalInfo.sGh / (pow(10, GCOMMAND_DECIMAL_PLACE));

		fprintf(s_p0, "模态指令：");
		fprintf(s_p0, "G%d、", cGa);
		fprintf(s_p0, "G%d、", cGb);
		fprintf(s_p0, "G%d、", cGc);
		fprintf(s_p0, "G%d、", cGd);
		fprintf(s_p0, "G%d、", cGe);
		fprintf(s_p0, "G%d、", cGf);
		fprintf(s_p0, "G%d、", cGg);
		fprintf(s_p0, "G%d\n", cGh);

		fprintf(s_p0, "进给速度：%d\t", (int32)pSGCodeParseUnit->dFeedrate);
		fprintf(s_p0, "主轴转速：%d\t", (int32)pSGCodeParseUnit->dSpindleSpeed);
		fprintf(s_p0, "第二主轴转速：%d\t", (int32)pSGCodeParseUnit->dSecondSpindleSpeed);
		fprintf(s_p0, "刀号：%d\t刀补号：%d\n", pSGCodeParseUnit->ucToolID, pSGCodeParseUnit->ucToolCompNum);


	}

	//打印实体信息
	{
		fprintf(s_p0, "实体类型：");
		switch (pSGCodeParseUnit->EntityType)
		{
		case DOT:
			fprintf(s_p0, "DOT\n" );
			break;
		case L1NE:
			fprintf(s_p0, "LINE\n");
			break;
		case ARC:
			fprintf(s_p0, "ARC\t");
			fprintf(s_p0, "圆心坐标：");
			fprintf(s_p0, "X：%.3f\t\t", pSGCodeParseUnit->SCenterPoint.dArray[0]);//X
			fprintf(s_p0, "Y：%.3f\t\t", pSGCodeParseUnit->SCenterPoint.dArray[1]);//Y
			fprintf(s_p0, "Z：%.3f\n", pSGCodeParseUnit->SCenterPoint.dArray[2]);//Z
			break;
		case TAPPING:
			fprintf(s_p0, "TAPPING\n");
			break;
		default:
			fprintf(s_p0, "OTHERTYPE\n");
			break;
		}


	}

	//打印坐标信息
	{
		//打印起点坐标
		fprintf(s_p0, "起点坐标：");
		fprintf(s_p0, "X：%.3f\t\t", pSGCodeParseUnit->SStartPoint.dArray[0]);//X
		fprintf(s_p0, "Y：%.3f\t\t", pSGCodeParseUnit->SStartPoint.dArray[1]);//Y
		fprintf(s_p0, "Z：%.3f\n", pSGCodeParseUnit->SStartPoint.dArray[2]);//Z
		fprintf(s_p0, "C：%.3f\n", pSGCodeParseUnit->SStartPoint.dArray[5]);//C

		//打印终点坐标
		fprintf(s_p0, "终点坐标：");
		fprintf(s_p0, "X：%.3f\t\t", pSGCodeParseUnit->SEndPoint.dArray[0]);//X
		fprintf(s_p0, "Y：%.3f\t\t", pSGCodeParseUnit->SEndPoint.dArray[1]);//Y
		fprintf(s_p0, "Z：%.3f\n", pSGCodeParseUnit->SEndPoint.dArray[2]);//Z
		fprintf(s_p0, "C：%.3f\n", pSGCodeParseUnit->SEndPoint.dArray[5]);//C

	}
	//打印延时时间
	{
		if (pSGCodeParseUnit->unDelayTime != 0)
		{
			fprintf(s_p0, "延时时间：%dms", pSGCodeParseUnit->unDelayTime);
		}
	}




	//缓冲区数据全部打印
	fprintf(s_p0, "\n");

}

/*************************************************
  函数名称：
  函数功能: 打印G代码轨迹的NC文件
  输入参数：nNo		编号
  输出参数：
  返 回 值：
  备    注：lxw打印G代码缓冲区单元数据
**************************************************/
void printGCodeNCFile(struct SGCodeParse* pSGCodeParseUnit, int32 nNo)
{
	//2.打印G值
	switch (pSGCodeParseUnit->EntityType)
	{
	case OTHERTYPE:
		fprintf(s_p0, "G0\t");
#if CONSOLEPRINT
		printf("G0\t");
#endif

		break;
	case L1NE:
		fprintf(s_p0, "G1\t");
#if CONSOLEPRINT
		printf("G1\t");
#endif

		break;
	case ARC:
		if (pSGCodeParseUnit->cArcDir == 1)
		{
			fprintf(s_p0, "G2\tR%.3f\t", pSGCodeParseUnit->dRadius);
#if CONSOLEPRINT
			printf("G2\tR%.3f\t", pSGCodeParseUnit->dRadius);
#endif

		}
		else
		{
			fprintf(s_p0, "G3\tR%.3f\t", pSGCodeParseUnit->dRadius);
#if CONSOLEPRINT
			printf("G3\tR%.3f\t", pSGCodeParseUnit->dRadius);
#endif

		}
		break;
	default:
		break;
	}

	//3.打印坐标值
	fprintf(s_p0, "X%.3f\tY%.3f\tZ%.3f\n", pSGCodeParseUnit->SEndPoint.dArray[0]*2, pSGCodeParseUnit->SEndPoint.dArray[1], pSGCodeParseUnit->SEndPoint.dArray[2]);
#if CONSOLEPRINT
	printf("X%.3f\tY%.3f\tZ%.3f\n", pSGCodeParseUnit->SEndPoint.dArray[0] * 2, pSGCodeParseUnit->SEndPoint.dArray[1], pSGCodeParseUnit->SEndPoint.dArray[2]);
#endif
}

/*************************************************
  函数名称：
  函数功能: 打印G代码轨迹的NC文件
  输入参数：nNo		编号
  输出参数：
  返 回 值：
  备    注：lxw打印G代码缓冲区单元数据
**************************************************/
void printInterpolatePointNCFile(int32 nLineNum)
{
	//绝对坐标输出
	fprintf(s_p0, "G1 Z%.3f X%.3f(%d)\n", getMachCoordDouble(0, AXIS_Z), getMachCoordDouble(0, AXIS_X), nLineNum);
}


/*************************************************
  函数名称：void GCODE_PRINT(uint8 ucChannelNum)
  函数功能：保留旧打印入口，转发当前写槽索引。
  输入参数：ucChannelNum 有效通道号。
  输出参数：与 printGCodeAt 相同。
  返 回 值：无。
  备    注：倒圆的延迟轨迹打印使用显式索引，不能假定待打印段始终在当前写槽。
**************************************************/
void GCODE_PRINT(uint8 ucChannelNum)
{
    printGCodeAt(getSGCodeParsePointerIndex(GCODE_WRITE,ucChannelNum),ucChannelNum);
}

#endif
