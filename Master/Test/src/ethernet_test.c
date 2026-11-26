/****************************************************************************************************
 * @file        ethernet_test.c
 * @author      ShadowFury
 * @version     V1.0
 * @date        2025-11-12
 * @brief       以太网驱动测试程序
 *              - ethernet_send_test() 发送数据，在Wireshark查看
 *              - ethernet_link_test() 网络连接状态测试（适用于循环调用）
 *              - ethernet_receive_test() 接收数据测试，验证dm9162_read_packet函数
 ****************************************************************************************************/
#include "ethernet_test.h"
#include "ethernet.h"
#include "ethernet_chip.h"
#include <stdio.h>
#include <string.h>

// 测试数据：模拟一个简单的ARP请求帧
static const uint8_t test_frame[] = {
    // 目的MAC地址 (广播地址)
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    // 源MAC地址 (开发板MAC)
    0x08,
    0x00,
    0x06,
    0x00,
    0x00,
    0x09,
    // 帧类型: ARP (0x0806)
    0x08,
    0x06,
    // ARP数据 (28字节)
    0x00,
    0x01,
    0x08,
    0x00,
    0x06,
    0x04,
    0x00,
    0x01,
    0x08,
    0x00,
    0x06,
    0x00,
    0x00,
    0x09,
    0xC0,
    0xA8,
    0x01,
    0x0A,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xC0,
    0xA8,
    0x01,
    0x01,
    // 填充数据 (达到最小帧长64字节)
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00};

// 自定义测试数据
static const char custom_data[] = "Hello from ALIENTEK STM32H743! This is a test frame for Wireshark capture.";

// static函数声明
static void print_hex_ascii_dump(const uint8_t *data, int32_t length);

/**
 * @brief       以太网发送测试函数
 * @note        发送测试帧到网络，可在PC端用Wireshark抓包分析
 *              支持发送预定义的ARP测试帧或自定义数据
 * @retval      1: 成功 0:失败
 */
int ethernet_send_test(void)
{
    int ret = 0;

    // 确保以太网已初始化
    if (g_eth_handler.Instance == NULL)
    {
        printf("Ethernet not initialized! Please call ethernet_init() first.\r\n");
        return 0;
    }

    // 检查网络连接状态
    while (!ethernet_link_status())
    {
        return 0;
    }

    // 方式1：发送预定义ARP测试帧
    //    ret = dm9162_write_packet((void *)test_frame, sizeof(test_frame));
    //    if (ret > 0)
    //    {
    //        printf("ARP test frame sent successfully! Size: %d bytes\r\n", ret);
    //    }
    //    else
    //    {
    //        printf("Failed to send ARP test frame!\r\n");
    //    }

    //    HAL_Delay(100);

    // 方式2：发送自定义数据帧
    uint8_t custom_frame[128];
    // 构造以太网帧头 (广播地址 + 源MAC)
    memcpy(custom_frame, "\xFF\xFF\xFF\xFF\xFF\xFF\x08\x00\x06\x00\x00\x09", 12);
    // 设置帧类型为IPv4 (0x0800)
    custom_frame[12] = 0x08;
    custom_frame[13] = 0x00;
    // 填充数据
    memcpy(&custom_frame[14], custom_data, sizeof(custom_data));
    // 发送帧 (总长度: 14字节头 + 数据长度)
    ret = dm9162_write_packet((void *)custom_frame, 14 + sizeof(custom_data));

    if (ret > 0)
    {
        printf("Custom data frame sent successfully! Size: %d bytes\r\n", ret);
        return 1;
    }

    printf("Failed to send custom data frame!\r\n");
    return 0;
}

/**
 * @brief       网络连接状态测试（适用于循环调用）
 * @note        检测网线连接状态，仅在状态变化时打印
 * @retval      网络连接状态，0: 未连接，1: 已连接
 */
uint8_t ethernet_link_test(void)
{
    static uint8_t last_link_status = 0xFF; // 初始化为未知状态
    uint8_t current_status;

    current_status = ethernet_link_status();

    // 仅在状态变化时打印
    if (current_status != last_link_status)
    {
        if (current_status)
        {
            printf("[ETH] connected\r\n");
        }
        else
        {
            printf("[ETH] disconnected\r\n");
        }
        last_link_status = current_status;
    }

    HAL_Delay(1000);

    return current_status;
}

/**
 * @brief       以太网接收测试
 * @note        调用一次读取一个数据包，立即返回。适合在主循环中轮询。
 * @retval      -1: 错误(未初始化), 0: 无数据/无效帧, 1: 成功捕获有效帧
 */
int ethernet_receive_test(void)
{
    static uint32_t frame_count = 0; // 静态变量保留统计
    static uint32_t valid_frame_count = 0;

    uint8_t rx_buffer[ETH_MAX_PACKET_SIZE];
    int32_t recv_len;
    uint8_t current_mac[6] = {0x08, 0x00, 0x06, 0x00, 0x00, 0x09};

    /* 检查以太网初始化 */
    if (g_eth_handler.Instance == NULL)
    {
        printf("[ETH_TEST] Error: Ethernet not initialized!\r\n");
        return -1;
    }

    /* 单次读取数据包（非阻塞） */
    recv_len = dm9162_read_packet(rx_buffer);

    if (recv_len > 0)
    {
        frame_count++;

        /* 验证帧最小长度 */
        if (recv_len >= 14) /* 以太网帧头14字节 */
        {
            /* 检查是否为广播或发往本机的帧 */
            uint8_t is_target = 0;
            if (rx_buffer[0] == 0xFF) /* 广播MAC */
            {
                is_target = 1;
            }
            else if (memcmp(&rx_buffer[0], current_mac, 6) == 0) /* 本机MAC */
            {
                is_target = 1;
            }

            if (is_target)
            {
                valid_frame_count++;

                /* 解析帧类型 */
                uint16_t eth_type = (rx_buffer[12] << 8) | rx_buffer[13];

                printf("[ETH_RX] Frame #%d captured! Len: %d bytes\r\n", frame_count, recv_len);

                /* 识别帧类型 */
                if (eth_type == 0x0806) /* ARP */
                {
                    printf("         Type: ARP\r\n");
                }
                else if (eth_type == 0x0800) /* IPv4 */
                {
                    printf("         Type: IPv4 - ");
                    if (recv_len >= 34 && rx_buffer[23] == 0x01) /* ICMP协议 */
                    {
                        printf("ICMP Echo Request [PING DETECTED!]");
                    }
                    else
                    {
                        printf("Other Protocol");
                    }
                    printf("\r\n");
                }
                else
                {
                    printf("         Type: 0x%04X\r\n", eth_type);
                }

                /* 打印源MAC */
                printf("         SRC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", rx_buffer[6], rx_buffer[7], rx_buffer[8], rx_buffer[9], rx_buffer[10], rx_buffer[11]);

                /* 打印数据内容（十六进制和ASCII） */
                print_hex_ascii_dump(rx_buffer, recv_len);
            }

            return 0; /* 捕获到帧但无效 */
        }

        return 0; /* 无数据 */
    }
}

// 发送成功后，接收一次数据
void ethernet_send_receive_test(void)
{
    // 发送测试数据
    if (ethernet_send_test())
    {
        // 成功:接收数据
        HAL_Delay(1);
        ethernet_receive_test();
    }
}

// 接收成功后发送数据
void ethernet_receive_send_test(void)
{
    if (ethernet_receive_test())
    {
        ethernet_send_test();
    }
}

/**
 * @brief       打印数据内容的十六进制和ASCII转储
 * @param       data: 数据缓冲区指针
 * @param       length: 数据长度
 */
static void print_hex_ascii_dump(const uint8_t *data, int32_t length)
{
    printf("         Data:\r\n");
    for (int i = 0; i < length; i += 16)
    {
        printf("         %04X: ", i);

        /* 十六进制部分 */
        for (int j = 0; j < 16; j++)
        {
            if (i + j < length)
            {
                printf("%02X ", data[i + j]);
            }
            else
            {
                printf("   ");
            }
            if (j == 7)
                printf(" ");
        }

        printf(" |");

        /* ASCII部分 */
        for (int j = 0; j < 16 && (i + j) < length; j++)
        {
            uint8_t c = data[i + j];
            printf("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        printf("|\r\n");
    }
}