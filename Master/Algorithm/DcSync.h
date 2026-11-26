#ifndef __DCSYNC_H__
#define __DCSYNC_H__

// 1. 头文件
#include <stdint.h>

// 2. 函数声明
void masterSync(int64_t lRefTimeNs, int64_t lCycleTimeUs, int64_t *pOffsetTimeUs);

#endif // __DCSYNC_H__
