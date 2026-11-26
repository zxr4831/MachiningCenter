#ifndef __ETHERNET_TEST_H
#define __ETHERNET_TEST_H

#include "stdint.h"

/**
 * @brief       以太网发送测试函数
 * @note        发送测试帧到网络，可在PC端用Wireshark抓包分析
 *              支持发送预定义的ARP测试帧或自定义数据
 * @retval      1: 成功 0:失败
 */
int ethernet_send_test(void);

/**
 * @brief       网络连接状态测试（适用于循环调用）
 * @note        检测网线连接状态，仅在状态变化时打印
 * @retval      网络连接状态，0: 未连接，1: 已连接
 */
uint8_t ethernet_link_test(void);

/**
 * @brief       以太网接收测试（非阻塞版本）
 * @note        调用一次读取一个数据包，立即返回。适合在主循环中轮询。
 *              如需长时间测试，请在外层调用循环中实现超时控制。
 * @retval      -1: 错误(未初始化), 0: 无数据/无效帧, 1: 成功捕获有效帧
 */
int ethernet_receive_test(void);

// 发送成功后，接收一次数据
void ethernet_send_receive_test(void);

// 接收成功后发送数据
void ethernet_receive_send_test(void);

#endif
