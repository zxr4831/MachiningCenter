/**
 * @file         : app_usart.c
 * @author       : ShadowFury
 * @date         : 2025-07-10
 * @version      : 1.0
 * @brief        : EtherCAT 主站应用层代码
 */
#include "app_usart.h"
#include "ringbuffer.h"

// 全局变量 串口缓冲区
RingBuffer g_usartRingBuffer;        // 缓冲区实例
static uint8_t s_ucUsartBuffer[256]; // 数据缓冲区

/**
 * @brief: 串口缓冲区初始化
 * @return 状态
 *  @retval 0 初始化成功
 *  @retval 1 初始化失败
 */
uint8_t uartRingBufferInit(void)
{
    if (createRingBuffer(&g_usartRingBuffer, s_ucUsartBuffer, 256) != enOK)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief       解析串口帧数据
 * @param[out]  pIndex      : 解析出的伺服从站索引
 * @param[out]  pDirection : 解析出的方向值
 * @param[out]  pTtarget    : 解析出的目标位置值
 * @return      uint8_t   : 解析结果，0-失败，1-成功
 */
uint8_t parseUsartData(uint8_t *pIndex, uint8_t *pDirection, uint32_t *pTarget)
{
    uint8_t ucRxData[FRAME_TOTAL_LEN];

    if (pDirection == NULL || pTarget == NULL)
    {
        return 0;
    }

    // 获取缓冲区的数据 9个字节
    if (readRingBuffer(&g_usartRingBuffer, ucRxData, FRAME_TOTAL_LEN) != FRAME_TOTAL_LEN)
    {
        return 0;
    }

    // 帧头 数据长度 帧尾校验
    if (ucRxData[0] != FRAME_HEADER || ucRxData[1] != FRAME_DATA_LEN || ucRxData[8] != FRAME_TAIL)
    {
        return 0;
    }

    // 提取从站索引
    *pIndex = ucRxData[2];

    // 提取方向
    *pDirection = ucRxData[3];

    // 提取目标位置（大端序）
    *pTarget = ((uint32_t)ucRxData[4] << 24) | ((uint32_t)ucRxData[5] << 16) | ((uint32_t)ucRxData[6] << 8) | ucRxData[7];

    return 1;
}
