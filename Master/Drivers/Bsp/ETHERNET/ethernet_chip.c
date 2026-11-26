/**
 ****************************************************************************************************
 * @file        ethernet_chip.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-08-01
 * @brief       PHY芯片配置驱动
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */
#include "ethernet_chip.h"
#include "ethernet.h"
#include "tim.h" /* zxr: 使用 TIM2 微秒计数限制发送等待。 */
#include <stdio.h>
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static int32_t ETH_PHY_IO_Init(void);
static int32_t ETH_PHY_IO_DeInit(void);
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
static int32_t ETH_PHY_IO_GetTick(void);

ETH_TxPacketConfigTypeDef TxConfig = {0};
eth_chip_object_t ETHCHIP;
eth_chip_ioc_tx_t DM9162_IOCtx = {ETH_PHY_IO_Init, ETH_PHY_IO_DeInit, ETH_PHY_IO_WriteReg, ETH_PHY_IO_ReadReg, ETH_PHY_IO_GetTick};

#define ETH_CHIP_SW_RESET_TO ((uint32_t)500U) /* 软件复位等待时间 */
#define ETH_CHIP_INIT_TO ((uint32_t)2000U)    /* 初始化等待时间 */
#define ETH_CHIP_MAX_DEV_ADDR ((uint32_t)31U) /* PHY地址的最大值 */

#define ETH_TX_TIMEOUT_US (200U) /* zxr: DMA 发送等待上限，单位 us。 */

// 接收缓冲区
/* zxr: GCC 链接段固定到原 DMA 地址 0x30040200。 */
__attribute__((section(".eth_rx_buffer"), aligned(32))) static uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */
/* zxr: GCC 链接段固定到原 DMA 地址 0x30042000。 */
__attribute__((section(".eth_tx_buffer"), aligned(32))) static uint8_t Tx_Buff[ETH_TX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Transmit Buffers */

static EMAC_CTRL Emac;

/* zxr: 发送异常只计数，避免周期路径打印；超时同时计入失败次数。 */
static volatile uint32_t s_tx_error_count = 0;
static volatile uint32_t s_tx_timeout_count = 0;

/**
 * @brief       将IO函数注册到组件对象
 * @param       pobj：设备对象
 * @param       ioctx：保存设备IO功能
 * @retval      ETH_CHIP_STATUS_OK：OK
 *              ETH_CHIP_STATUS_ERROR：缺少功能
 */
int32_t eth_chip_regster_bus_io(eth_chip_object_t *pobj, eth_chip_ioc_tx_t *ioctx)
{
    if (!pobj || !ioctx->readreg || !ioctx->writereg || !ioctx->gettick)
    {
        return ETH_CHIP_STATUS_ERROR;
    }

    pobj->io.init = ioctx->init;
    pobj->io.deinit = ioctx->deinit;
    pobj->io.readreg = ioctx->readreg;
    pobj->io.writereg = ioctx->writereg;
    pobj->io.gettick = ioctx->gettick;

    return ETH_CHIP_STATUS_OK;
}

/**
  * @brief       初始化ETH_CHIP并配置所需的硬件资源
  * @param       pobj: 设备对象
  * @retval      ETH_CHIP_STATUS_OK：初始化ETH_CHIP并配置所需的硬件资源成功
                 ETH_CHIP_STATUS_ADDRESS_ERROR：找不到设备地址
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR：不能写入寄存器
                 ETH_CHIP_STATUS_RESET_TIMEOUT：无法执行软件复位
  */
int32_t eth_chip_init(eth_chip_object_t *pobj)
{
    uint32_t tickstart = 0, regvalue = 0, addr = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->is_initialized == 0)
    {
        if (pobj->io.init != 0)
        {
            /* MDC时钟 */
            pobj->io.init();
        }

        /* 设置PHY地址为32 */
        pobj->devaddr = ETH_CHIP_MAX_DEV_ADDR + 1;

        /* 主要为了查找PHY地址 */
        for (addr = 0; addr <= ETH_CHIP_MAX_DEV_ADDR; addr++)
        {
            if (pobj->io.readreg(addr, ETH_CHIP_PHYSCSR, &regvalue) < 0)
            {
                status = ETH_CHIP_STATUS_READ_ERROR;
                /* 无法读取这个设备地址继续下一个地址 */
                continue;
            }
            /* 已经找到PHY地址了 */
            if ((regvalue & ETH_CHIP_PHY_COUNT) == addr)
            {
                pobj->devaddr = addr;
                status = ETH_CHIP_STATUS_OK;
                break;
            }
        }

        /* 判断这个PHY地址是否大于32（2^5）*/
        if (pobj->devaddr > ETH_CHIP_MAX_DEV_ADDR)
        {
            status = ETH_CHIP_STATUS_ADDRESS_ERROR;
        }

        /* 如果PHY地址有效 */
        if (status == ETH_CHIP_STATUS_OK)
        {
            /* 设置软件复位  */
            if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, ETH_CHIP_BCR_SOFT_RESET) >= 0)
            {
                /* 获取软件重置状态 */
                if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &regvalue) >= 0)
                {
                    tickstart = pobj->io.gettick();

                    /* 等待软件复位完成或超时  */
                    while (regvalue & ETH_CHIP_BCR_SOFT_RESET)
                    {
                        if ((pobj->io.gettick() - tickstart) <= ETH_CHIP_SW_RESET_TO)
                        {
                            if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &regvalue) < 0)
                            {
                                status = ETH_CHIP_STATUS_READ_ERROR;
                                break;
                            }
                        }
                        else
                        {
                            status = ETH_CHIP_STATUS_RESET_TIMEOUT;
                            break;
                        }
                    }
                }
                else
                {
                    status = ETH_CHIP_STATUS_READ_ERROR;
                }
            }
            else
            {
                status = ETH_CHIP_STATUS_WRITE_ERROR;
            }
        }
    }

    /* 到了这里，初始化完成！！！ */
    if (status == ETH_CHIP_STATUS_OK)
    {
        tickstart = pobj->io.gettick();

        /* 等待2s进行初始化 */
        while ((pobj->io.gettick() - tickstart) <= ETH_CHIP_INIT_TO)
        {
        }
        pobj->is_initialized = 1;
    }

    return status;
}

/**
  * @brief       反初始化ETH_CHIP及其硬件资源
  * @param       pobj: 设备对象
  * @retval      ETH_CHIP_STATUS_OK：反初始化失败成功
                 ETH_CHIP_STATUS_ERROR：反初始化失败
  */
int32_t eth_chip_deinit(eth_chip_object_t *pobj)
{
    if (pobj->is_initialized)
    {
        if (pobj->io.deinit != 0)
        {
            if (pobj->io.deinit() < 0)
            {
                return ETH_CHIP_STATUS_ERROR;
            }
        }

        pobj->is_initialized = 0;
    }

    return ETH_CHIP_STATUS_OK;
}

/**
  * @brief       关闭ETH_CHIP的下电模式
  * @param       pobj: 设备对象
  * @retval      ETH_CHIP_STATUS_OK：关闭成功
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR：不能写寄存器
  */
int32_t eth_chip_disable_power_down_mode(eth_chip_object_t *pobj)
{
    uint32_t readval = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &readval) >= 0)
    {
        readval &= ~ETH_CHIP_BCR_POWER_DOWN;

        /* 清除下电模式 */
        if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, readval) < 0)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    return status;
}

/**
  * @brief       使能ETH_CHIP的下电模式
  * @param       pobj: 设备对象
  * @retval      ETH_CHIP_STATUS_OK：关闭成功
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR：不能写寄存器
  */
int32_t eth_chip_enable_power_down_mode(eth_chip_object_t *pobj)
{
    uint32_t readval = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &readval) >= 0)
    {
        readval |= ETH_CHIP_BCR_POWER_DOWN;

        /* 使能下电模式 */
        if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, readval) < 0)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    return status;
}

/**
  * @brief       启动自动协商过程
  * @param       pobj: 设备对象
  * @retval      ETH_CHIP_STATUS_OK：开启成功
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR：不能写寄存器
  */
int32_t eth_chip_start_auto_nego(eth_chip_object_t *pobj)
{
    uint32_t readval = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &readval) >= 0)
    {
        readval |= ETH_CHIP_BCR_AUTONEGO_EN;

        /* 启动自动协商 */
        if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, readval) < 0)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    return status;
}

/**
  * @brief       获取ETH_CHIP设备的链路状态
  * @param       pobj: 设备对象
  * @param       pLinkState: 指向链路状态的指针
  * @retval      ETH_CHIP_STATUS_100MBITS_FULLDUPLEX：100M，全双工
                 ETH_CHIP_STATUS_100MBITS_HALFDUPLEX ：100M，半双工
                 ETH_CHIP_STATUS_10MBITS_FULLDUPLEX：10M，全双工
                 ETH_CHIP_STATUS_10MBITS_HALFDUPLEX ：10M，半双工
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
  */
int32_t eth_chip_get_link_state(eth_chip_object_t *pobj)
{
    uint32_t readval = 0;

    /* 检测特殊功能寄存器链接值 */
    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_PHYSCSR, &readval) < 0)
    {
        return ETH_CHIP_STATUS_READ_ERROR;
    }

    if (((readval & ETH_CHIP_SPEED_STATUS) != ETH_CHIP_SPEED_STATUS) && ((readval & ETH_CHIP_DUPLEX_STATUS) != 0))
    {
        return ETH_CHIP_STATUS_100MBITS_FULLDUPLEX;
    }
    else if (((readval & ETH_CHIP_SPEED_STATUS) != ETH_CHIP_SPEED_STATUS))
    {
        return ETH_CHIP_STATUS_100MBITS_HALFDUPLEX;
    }
    else if (((readval & ETH_CHIP_BCR_DUPLEX_MODE) != ETH_CHIP_BCR_DUPLEX_MODE))
    {
        return ETH_CHIP_STATUS_10MBITS_FULLDUPLEX;
    }
    else
    {
        return ETH_CHIP_STATUS_10MBITS_HALFDUPLEX;
    }
}

/**
  * @brief       设置ETH_CHIP设备的链路状态
  * @param       pobj: 设备对象
  * @param       pLinkState: 指向链路状态的指针
  * @retval      ETH_CHIP_STATUS_OK：设置成功
                 ETH_CHIP_STATUS_ERROR ：设置失败
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR ：不能写入寄存器
  */
int32_t eth_chip_set_link_state(eth_chip_object_t *pobj, uint32_t LinkState)
{
    uint32_t bcrvalue = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &bcrvalue) >= 0)
    {
        /* 禁用链路配置(自动协商，速度和双工) */
        bcrvalue &= ~(ETH_CHIP_BCR_AUTONEGO_EN | ETH_CHIP_BCR_SPEED_SELECT | ETH_CHIP_BCR_DUPLEX_MODE);

        if (LinkState == ETH_CHIP_STATUS_100MBITS_FULLDUPLEX)
        {
            bcrvalue |= (ETH_CHIP_BCR_SPEED_SELECT | ETH_CHIP_BCR_DUPLEX_MODE);
        }
        else if (LinkState == ETH_CHIP_STATUS_100MBITS_HALFDUPLEX)
        {
            bcrvalue |= ETH_CHIP_BCR_SPEED_SELECT;
        }
        else if (LinkState == ETH_CHIP_STATUS_10MBITS_FULLDUPLEX)
        {
            bcrvalue |= ETH_CHIP_BCR_DUPLEX_MODE;
        }
        else
        {
            /* 错误的链路状态参数 */
            status = ETH_CHIP_STATUS_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    if (status == ETH_CHIP_STATUS_OK)
    {
        /* 写入链路状态 */
        if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, bcrvalue) < 0)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }

    return status;
}

/**
  * @brief       启用环回模式
  * @param       pobj: 设备对象
  * @param       pLinkState: 指向链路状态的指针
  * @retval      ETH_CHIP_STATUS_OK：设置成功
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR ：不能写入寄存器
  */
int32_t eth_chip_enable_loop_back_mode(eth_chip_object_t *pobj)
{
    uint32_t readval = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &readval) >= 0)
    {
        readval |= ETH_CHIP_BCR_LOOPBACK;

        /* 启用环回模式 */
        if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, readval) < 0)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    return status;
}

/**
  * @brief       禁用环回模式
  * @param       pobj: 设备对象
  * @param       pLinkState: 指向链路状态的指针
  * @retval      ETH_CHIP_STATUS_OK：设置成功
                 ETH_CHIP_STATUS_READ_ERROR：不能读取寄存器
                 ETH_CHIP_STATUS_WRITE_ERROR ：不能写入寄存器
  */
int32_t eth_chip_disable_loop_back_mode(eth_chip_object_t *pobj)
{
    uint32_t readval = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (pobj->io.readreg(pobj->devaddr, ETH_CHIP_BCR, &readval) >= 0)
    {
        readval &= ~ETH_CHIP_BCR_LOOPBACK;

        /* 禁用环回模式 */
        if (pobj->io.writereg(pobj->devaddr, ETH_CHIP_BCR, readval) < 0)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    return status;
}

/**
 * @brief  Initializes the MDIO interface GPIO and clocks.
 * @param  None
 * @retval 0 if OK, -1 if ERROR
 */
static int32_t ETH_PHY_IO_Init(void)
{
    /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here
    */

    /* Configure the MDIO Clock */
    HAL_ETH_SetMDIOClockRange(&g_eth_handler);

    return 0;
}

/**
 * @brief  De-Initializes the MDIO interface .
 * @param  None
 * @retval 0 if OK, -1 if ERROR
 */
static int32_t ETH_PHY_IO_DeInit(void) { return 0; }

/**
 * @brief  Read a PHY register through the MDIO interface.
 * @param  DevAddr: PHY port address
 * @param  RegAddr: PHY register address
 * @param  pRegVal: pointer to hold the register value
 * @retval 0 if OK -1 if Error
 */
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
    if (HAL_ETH_ReadPHYRegister(&g_eth_handler, DevAddr, RegAddr, pRegVal) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  Write a value to a PHY register through the MDIO interface.
 * @param  DevAddr: PHY port address
 * @param  RegAddr: PHY register address
 * @param  RegVal: Value to be written
 * @retval 0 if OK -1 if Error
 */
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
    if (HAL_ETH_WritePHYRegister(&g_eth_handler, DevAddr, RegAddr, RegVal) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  Get the time in millisecons used for internal PHY driver process.
 * @retval Time value
 */
static int32_t ETH_PHY_IO_GetTick(void) { return HAL_GetTick(); }

/**
 * @brief 以太网芯片初始化
 * @retval 初始化状态，1: 失败, 0: 成功
 */
int dm9162_init(void)
{
    int32_t phy_link_state = 0;
    uint32_t duplex = 0;
    uint32_t speed = 0;
    ETH_MACConfigTypeDef g_eth_macconfig_handler = {0};

    /* 配置发送报文配置常用参数 */
    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfigTypeDef));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    /* 设置PHY IO功能 */
    eth_chip_regster_bus_io(&ETHCHIP, &DM9162_IOCtx);

    /* 初始化 ETH PHY */
    if (eth_chip_init(&ETHCHIP) != ETH_CHIP_STATUS_OK)
    {
        return -1;
    }

    /* 必须开启自动协商功能 */
    if (eth_chip_start_auto_nego(&ETHCHIP) != ETH_CHIP_STATUS_OK)
    {
        printf("start auto nego failed\r\n");
    }

    // 关键!!! 等待PHY芯片初始化完成
    HAL_Delay(4000);

    /* 获取链路状态 */
    phy_link_state = eth_chip_get_link_state(&ETHCHIP);

    if (phy_link_state != ETH_CHIP_STATUS_READ_ERROR)
    {
        switch (phy_link_state)
        {
        case ETH_CHIP_STATUS_100MBITS_FULLDUPLEX:
            duplex = ETH_FULLDUPLEX_MODE;
            speed = ETH_SPEED_100M;
            printf("ETH_CHIP_STATUS_100MBITS_FULLDUPLEX\r\n");
            break;
        case ETH_CHIP_STATUS_100MBITS_HALFDUPLEX:
            duplex = ETH_HALFDUPLEX_MODE;
            speed = ETH_SPEED_100M;
            printf("ETH_CHIP_STATUS_100MBITS_HALFDUPLEX\r\n");
            break;
        case ETH_CHIP_STATUS_10MBITS_FULLDUPLEX:
            duplex = ETH_FULLDUPLEX_MODE;
            speed = ETH_SPEED_10M;
            printf("ETH_CHIP_STATUS_10MBITS_HALFDUPLEX\r\n");
            break;
        case ETH_CHIP_STATUS_10MBITS_HALFDUPLEX:
            duplex = ETH_HALFDUPLEX_MODE;
            speed = ETH_SPEED_10M;
            printf("ETH_CHIP_STATUS_10MBITS_HALFDUPLEX\n");
            break;
        default:
            duplex = ETH_FULLDUPLEX_MODE;
            speed = ETH_SPEED_100M;
            printf("default:ETH_CHIP_STATUS_100MBITS_FULLDUPLEX\n");
            break;
        }
    }
    else
    {
        return -1;
    }

    /* Clear control structure */
    memset(&Emac, 0, sizeof(EMAC_CTRL));

    /* 配置MAC */
    HAL_ETH_GetMACConfig(&g_eth_handler, &g_eth_macconfig_handler);
    g_eth_macconfig_handler.DuplexMode = duplex;
    g_eth_macconfig_handler.Speed = speed;
    if (HAL_ETH_SetMACConfig(&g_eth_handler, &g_eth_macconfig_handler) != HAL_OK)
    {
        printf("[ETH] ERROR: Failed to set MAC config\r\n");
        return -1;
    }

    /* 启动ETH */
    if (HAL_ETH_Start(&g_eth_handler) != HAL_OK)
    {
        return -1;
    }

    /* 检查MCU与PHY芯片通信成功 */
    uint32_t timeout = 1000;
    while (!ethernet_read_phy(ETH_CHIP_PHYSCSR) && timeout--)
    {
        HAL_Delay(1);
    }

    if (timeout == 0)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief       读取以太网数据帧
 * @param       pBuff: 用户缓冲区指针，用于存储接收到的数据
 * @retval      成功返回数据长度(字节)，无数据返回0
 */
int dm9162_read_packet(void *pBuff)
{
    void *pPacket;
    uint32_t length;

    if (pBuff == NULL)
    {
        return 0;
    }

    if (HAL_ETH_ReadData(&g_eth_handler, &pPacket) == HAL_OK)
    {
        length = Emac.rx_buf.len;

        if (length > 0)
        {
            /* zxr: Rx_Buff 已由 MPU 设为 Normal 非缓存区，直接读取 DMA 数据，无需全局清理 Cache。 */
            memcpy(pBuff, Emac.rx_buf.buffer, length);

            return length;
        }
    }

    return 0;
}

/**
 * @brief       打印十六进制数据（调试用）
 * @param       data: 数据指针
 * @param       length: 数据长度
 * @param       title: 打印标题
 */
static void print_hex_data(const uint8_t *data, int length, const char *title)
{
    printf("\r\n[ETH TX] %s (Len=%d)\r\n", title, length);
    for (int i = 0; i < length; i++)
    {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) // 每16字节换行
        {
            printf("\r\n");
        }
        else if ((i + 1) % 8 == 0) // 每8字节加空格
        {
            printf("  ");
        }
    }
    printf("\r\n");
}

/**
 * @brief       写入以太网数据帧
 * @param       pBuff: 要发送的数据缓冲区
 * @param       len: 数据长度(字节)
 * @retval      成功返回发送长度，失败返回0
 */
int dm9162_write_packet(void *pBuff, int len)
{
    /* zxr: 每次调用提交一个完整帧；失败返回 0，不在实时路径打印。 */
    if (pBuff == NULL || len <= 0 || (uint32_t)len > ETH_MAX_PACKET_SIZE)
    {
        s_tx_error_count++;
        return 0;
    }

    /* zxr: 回收已完成的记录；仍由 DMA 占用的描述符和缓冲区保持原样。 */
    HAL_ETH_ReleaseTxPacket(&g_eth_handler);
    uint32_t tx_index = g_eth_handler.TxDescList.CurTxDesc;
    ETH_DMADescTypeDef *tx_desc = (ETH_DMADescTypeDef *)g_eth_handler.TxDescList.TxDesc[tx_index];
    if ((tx_desc->DESC3 & ETH_DMATXNDESCWBF_OWN) != 0U || g_eth_handler.TxDescList.PacketAddress[tx_index] != NULL || (g_eth_handler.Instance->DMACSR & ETH_DMACSR_FBE) != 0U)
    {
        s_tx_error_count++;
        return 0;
    }

    /* zxr: 使用描述符对应的独立缓冲区，交给 HAL 保存回收标识。 */
    Emac.tx_buf.buffer = Tx_Buff[tx_index];
    Emac.tx_buf.next = NULL;
    Emac.tx_buf.len = (uint32_t)len;
    memcpy(Emac.tx_buf.buffer, pBuff, len);
    TxConfig.TxBuffer = &Emac.tx_buf;
    TxConfig.Length = (uint32_t)len;
    TxConfig.pData = Emac.tx_buf.buffer;
    /* zxr: Tx_Buff 已由 MPU 设为 Normal 非缓存区；描述符提交顺序仍由 HAL 内存屏障保证。 */

    /* zxr: 此 HAL 接口只提交发送；沿用 HAL_ETH_Start，由下面轮询完成，不启用 ETH 中断。 */
    uint32_t start_us = get_us();
    HAL_StatusTypeDef status = HAL_ETH_Transmit_IT(&g_eth_handler, &TxConfig);
    Emac.tx_buf.len = 0;
    if (status != HAL_OK)
    {
        s_tx_error_count++;
        return 0;
    }

    /* zxr: 超时或 DMA 报错立即退出，不强制清 OWN，也不复用仍在发送的缓冲区。 */
    while ((tx_desc->DESC3 & ETH_DMATXNDESCWBF_OWN) != 0U)
    {
        if ((g_eth_handler.Instance->DMACSR & ETH_DMACSR_FBE) != 0U)
        {
            g_eth_handler.ErrorCode |= HAL_ETH_ERROR_DMA;
            g_eth_handler.DMAErrorCode = g_eth_handler.Instance->DMACSR;
            s_tx_error_count++;
            return 0;
        }
        if ((uint32_t)(get_us() - start_us) >= ETH_TX_TIMEOUT_US)
        {
            g_eth_handler.ErrorCode |= HAL_ETH_ERROR_TIMEOUT;
            s_tx_timeout_count++;
            s_tx_error_count++;
            return 0;
        }
    }

    /* zxr: DMA 完成后检查发送错误并回收记录，供后续帧复用。 */
    uint32_t tx_status = tx_desc->DESC3;
    HAL_ETH_ReleaseTxPacket(&g_eth_handler);
    if ((tx_status & ETH_DMATXNDESCWBF_ES) != 0U)
    {
        s_tx_error_count++;
        return 0;
    }
    return len;
}

/**
  \brief       Rx Allocate callback.
  \param[in]   buff  pointer to allocated buffer
  \return      None
*/
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    /* Allocate one of the RX-DMA buffers sequentially */
    *buff = &Rx_Buff[Emac.alloc_idx][0];
    if (++Emac.alloc_idx >= ETH_RX_DESC_CNT)
    {
        Emac.alloc_idx = 0;
    }
}

/**
  \brief       Rx Link callback.
  \param[in]   pStart  pointer to packet start
  \param[in]   pStart  pointer to packet end
  \param[in]   buff    pointer to received data
  \param[in]   Length  received data length
  \return      None
*/
void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
    (void)pStart;
    (void)pEnd;

    Emac.rx_buf.buffer = buff;
    Emac.rx_buf.len = Length;
}
