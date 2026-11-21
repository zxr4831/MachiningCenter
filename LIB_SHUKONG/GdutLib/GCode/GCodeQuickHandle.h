#ifndef __GCODEQUICKHANDLE_H
#define __GCODEQUICKHANDLE_H

//快速处理标志位结构体		陈敏聪 4/17
typedef struct _SQuickHandleFlag
{
    bool bIsModalInfoChangFlag;        //模态改变标志，用于G代码解析函数中
    bool bCanModalInfoQuickHandleFlag; //模态改变能快速处理标志为，用于G代码解析函数中
    int8 cNeedQuickHandleFlag;         //需要多行快速处理标志位，G代码、刀补、中断都用上
    int8 cCycleCounter;                //循环次数，G代码、刀补、中断都用上
    int8 cCycleSum;                    //循环总数，测试用
    int8 cFunctionReturnValue;         //函数返回值，用于判定中断服务函数退出循环
} SQuickHandleFlag;

//int8 JudgeGodeQuickHandleType(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf, uint8 ucChannelNum);
//
//bool CanModalChangedQuickHandle(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf, enum ModalNum* pModalChangedNumArray, int8* pCounter);
//
//bool IsModalInfoChanged(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBuf, enum ModalNum* pModalChangedNumArray, int8* pCounter);

#endif // !__GCODEQUICKHANDLE_H
