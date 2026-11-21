#ifndef __GCODEDEFINE_H
#define __GCODEDEFINE_H

/**********************/
//结构体定义
/**********************/

//自定义宏数据
typedef struct _SUserDefineData
{
    char* pName;       //宏名
    char* pReplaceStr; //要替换的字符串

} SUserDefineData;

/**********************/
//函数声明
/**********************/

//释放自定义宏数据
void freeDefineData(SUserDefineData* pData);

//记录自定义宏
int8 recordDefine(SGetTextStateInfo* pSGetTextStateInfo, SList* pDefineList);

//替换程序段中的宏名
int8 replaceDefineInSegment(SGetTextStateInfo* pSGetTextStateInfo, SList* pDefineList);

//获取宏名对应的字符串
int8 getTargetStringByMacroName(int8* pMacroName, uint16 cNameSize, int8* pTargetString, SList* pDefineList);

#endif // !__GCODEDEFINE_H
