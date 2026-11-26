#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <string.h>

typedef struct ringBuffer {
    uint8_t   *p_ucBuffer;    /* 数据存储区 */
    uint32_t    ulSize;       /* 缓冲区大小（2^n） */
    uint32_t    ulIn;         /* 写入位置指针 */
    uint32_t    ulOut;        /* 读取位置指针 */
} RingBuffer;

/* 错误码 */
typedef enum {
    enOK      = 0,      /* 操作成功 */
    enNull    = -1,     /* 空指针错误 */
    enSize    = -2      /* 大小错误 */
} ErrStatus;

/* 接口函数 */
ErrStatus createRingBuffer(RingBuffer* pRb, uint8_t* p_ucBuffer, uint32_t ulSize);
ErrStatus deleteRingBuffer(RingBuffer* pRb);
uint32_t writeRingBuffer(RingBuffer* pRb, const uint8_t* p_ucWBuffer, uint32_t ulLen);
uint32_t readRingBuffer(RingBuffer* pRb, uint8_t* p_ucRBuffer, uint32_t ulLen);
uint32_t canReadRingBuffer(const RingBuffer* pRb);
uint32_t canWriteRingBuffer(const RingBuffer* pRb);
uint8_t isRingBufferEmpty(const RingBuffer* pRb);

/* 辅助宏 */
#define min(a, b) (((a) < (b)) ? (a) : (b))

#endif /* RING_BUFFER_H */

