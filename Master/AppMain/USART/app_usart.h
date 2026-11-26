#ifndef __APP_USART_H__
#define __APP_USART_H__

/* 头文件 */
#include <stdint.h>
#include <stdio.h>

#include "ringbuffer.h"

/* 宏定义 */
#define FRAME_TOTAL_LEN 9   // 帧总长度
#define FRAME_HEADER 0xAA   // 帧头
#define FRAME_DATA_LEN 0x06 // 数据长度
#define FRAME_TAIL 0x55     // 帧尾

// 全局变量声明
extern RingBuffer g_usartRingBuffer; // 缓冲区实例

/**
 * @brief: 串口缓冲区初始化
 * @return 状态
 *  @retval 0 初始化成功
 *  @retval 1 初始化失败
 */
uint8_t uartRingBufferInit(void);

/**
 * @brief       解析串口帧数据
 * @param[out]  pIndex      : 解析出的伺服从站索引
 * @param[out]  pDirection : 解析出的方向值（1字节）
 * @param[out]  pTtarget    : 解析出的目标位置值（4字节）
 * @return      uint8_t   : 解析结果，0-失败，1-成功
 */
uint8_t parseUsartData(uint8_t *pIndex, uint8_t *pDirection, uint32_t *pTarget);

#endif /* __APP_USART_H__ */
