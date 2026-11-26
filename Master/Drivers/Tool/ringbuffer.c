#include "ringbuffer.h"

/*--------------------- 内部工具函数 ---------------------*/
/**
 * @brief   计算最高有效位位置
 * @param   x       输入数值
 * @return  最高位位置（1-32），0表示输入为0
 */
static int findLastSetBit(int x)
{
    int iPos = 32;
    if (!x)
        return 0;
    if (!(x & 0xffff0000u))
    {
        x <<= 16;
        iPos -= 16;
    }
    if (!(x & 0xff000000u))
    {
        x <<= 8;
        iPos -= 8;
    }
    if (!(x & 0xf0000000u))
    {
        x <<= 4;
        iPos -= 4;
    }
    if (!(x & 0xc0000000u))
    {
        x <<= 2;
        iPos -= 2;
    }
    if (!(x & 0x80000000u))
    {
        x <<= 1;
        iPos -= 1;
    }
    return iPos;
}

/**
 * @brief   对齐到最近的2的幂
 * @param   ulSize  原始大小
 * @return  对齐后的2的幂大小
 */
static uint32_t alignPowerOfTwo(uint32_t ulSize)
{
    return (1UL << (findLastSetBit(ulSize - 1) - 1));
}

/*--------------------- 对外接口函数 ---------------------*/
/**
 * @brief   创建环形缓冲区
 * @param   pRb         环形缓冲区指针
 * @param   p_ucBuffer   预分配的缓冲区内存
 * @param   ulSize      缓冲区大小（自动对齐）
 * @return  enOK:成功 enNull:空指针 enSize:大小错误
 */
ErrStatus createRingBuffer(RingBuffer *pRb, uint8_t *p_ucBuffer, uint32_t ulSize)
{
    if ((pRb == NULL) || (p_ucBuffer == NULL) || (ulSize == 0))
    {
        return enNull;
    }

    if (ulSize & (ulSize - 1))
    {
        ulSize = alignPowerOfTwo(ulSize);
    }

    pRb->p_ucBuffer = p_ucBuffer;
    pRb->ulSize = ulSize;
    pRb->ulIn = pRb->ulOut = 0;

    return enOK;
}

/**
 * @brief   删除环形缓冲区
 * @param   pRb     环形缓冲区指针
 * @return  enOK:成功 enNull:空指针
 */
ErrStatus deleteRingBuffer(RingBuffer *pRb)
{
    if (pRb == NULL)
    {
        return enNull;
    }

    pRb->p_ucBuffer = NULL;
    pRb->ulSize = 0;
    pRb->ulIn = pRb->ulOut = 0;

    return enOK;
}

/**
 * @brief   写入数据到缓冲区
 * @param   pRb         环形缓冲区指针
 * @param   p_ucWBuffer  写入数据地址
 * @param   ulLen       期望写入长度
 * @return  实际写入的数据长度
 */
uint32_t writeRingBuffer(RingBuffer *pRb, const uint8_t *p_ucWBuffer, uint32_t ulLen)
{
    uint32_t ulTempLen;
    uint32_t ulFreeSpace = pRb->ulSize - (pRb->ulIn - pRb->ulOut);

    if (!p_ucWBuffer || !ulLen)
        return 0;

    ulLen = min(ulLen, ulFreeSpace);

    /* 分两段拷贝 */
    ulTempLen = min(ulLen, pRb->ulSize - (pRb->ulIn & (pRb->ulSize - 1)));
    memcpy(pRb->p_ucBuffer + (pRb->ulIn & (pRb->ulSize - 1)), p_ucWBuffer, ulTempLen);
    memcpy(pRb->p_ucBuffer, p_ucWBuffer + ulTempLen, ulLen - ulTempLen);

    pRb->ulIn += ulLen;
    return ulLen;
}

/**
 * @brief   从缓冲区读取数据
 * @param   pRb         环形缓冲区指针
 * @param   ucpRBuffer  读取数据存储地址
 * @param   ulLen       期望读取长度
 * @return  实际读取的数据长度
 */
uint32_t readRingBuffer(RingBuffer *pRb, uint8_t *ucpRBuffer, uint32_t ulLen)
{
    uint32_t ulTempLen;
    uint32_t ulDataLen = pRb->ulIn - pRb->ulOut;

    if (!ucpRBuffer || !ulLen)
        return 0;

    ulLen = min(ulLen, ulDataLen);

    /* 分两段读取 */
    ulTempLen = min(ulLen, pRb->ulSize - (pRb->ulOut & (pRb->ulSize - 1)));
    memcpy(ucpRBuffer, pRb->p_ucBuffer + (pRb->ulOut & (pRb->ulSize - 1)), ulTempLen);
    memcpy(ucpRBuffer + ulTempLen, pRb->p_ucBuffer, ulLen - ulTempLen);

    pRb->ulOut += ulLen;
    return ulLen;
}

/**
 * @brief   获取可读数据量
 * @param   pRb     环形缓冲区指针
 * @return  可读取的字节数
 */
uint32_t canReadRingBuffer(const RingBuffer *pRb)
{
    if (pRb->ulIn == pRb->ulOut)
        return 0;
    return (pRb->ulIn > pRb->ulOut) ? (pRb->ulIn - pRb->ulOut) : (pRb->ulSize - (pRb->ulOut - pRb->ulIn));
}

/**
 * @brief   获取可写空间
 * @param   pRb     环形缓冲区指针
 * @return  剩余可写字节数
 */
uint32_t canWriteRingBuffer(const RingBuffer *pRb)
{
    return (pRb->ulSize - canReadRingBuffer(pRb));
}

/**
 * @brief   判断环形缓冲区是否为空
 * @param   pRb     环形缓冲区指针
 * @return  1: 缓冲区为空, 0: 缓冲区有数据
 */
uint8_t isRingBufferEmpty(const RingBuffer* pRb)
{
    /* 当读写位置相同时，缓冲区为空 */
    return (pRb->ulIn == pRb->ulOut) ? 1 : 0;
}
