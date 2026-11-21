#ifndef __GCODETEST_H
#define __GCODETEST_H

//修改文件名
void addPathSTM32FileName(int8* cOutput, int8* pInput, int8 cProgramListSize);

//打开文本
void GCODE_OPEN_FILE(uint8 ucChannelNum);

//关闭文本
void GCODE_CLOSE_FILE(uint8 ucChannelNum);

//初始化目标字符串缓冲区
void initTextBuf();

//初始化目标字符串缓冲区
void initDesinatedBuf(uint8 ucIdx);

//获取一行字符串
uint8 tryToGetLine(int8* pName, uint32 unTargetLineNum, int8 cProgramListSize, uint8 ucChannelNum);

//打印G代码缓冲区单元
void printGCodeBuffUnit(struct SGCodeParse* pSGCodeParseUnit, int32 nNo);

//打印G代码NC文件
void printGCodeNCFile(struct SGCodeParse* pSGCodeParseUnit, int32 nNo);

//打印插补点NC文件
void printInterpolatePointNCFile(int32 nLineNum);

#endif // !__GCODETEST_H
