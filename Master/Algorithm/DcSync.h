#ifndef DC_SYNC_H
#define DC_SYNC_H

// 1. 头文件
#include <stdint.h>

// 2. 函数声明
void dcSync_calculateOffset(int64_t lRefTimeNs, int64_t lCycleTimeUs, int64_t *plOffsetTimeUs);

#endif // DC_SYNC_H
