/**
 ****************************************************************************************************
 * @file        ethernet.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-08-01
 * @brief       ETHERNET 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 *
 ****************************************************************************************************
 */

#include "ethernet.h"
#include "ethernet_chip.h"
#include "pcf8574.h"
#include "delay.h"

/* 以太网句柄 */
ETH_HandleTypeDef g_eth_handler;
/* Ethernet Rx DMA 描述符 */
/* zxr: GCC 链接段固定到原 DMA 地址 0x30040000。 */
__attribute__((section(".eth_rx_desc"), aligned(32))) ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT];
/* Ethernet Tx DMA 描述符 */
/* zxr: GCC 链接段固定到原 DMA 地址 0x30040060。 */
__attribute__((section(".eth_tx_desc"), aligned(32))) ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT];

/**
 * @brief       ETH外设初始化
 * @param       无
 * @retval      0,成功
 *              1,失败
 */
uint8_t ethernet_init(void)
{
    static uint8_t macaddress[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00}; /* MAC地址 */

    g_eth_handler.Instance = ETH;
    g_eth_handler.Init.MACAddr = macaddress;
    g_eth_handler.Init.MediaInterface = HAL_ETH_RMII_MODE;
    g_eth_handler.Init.RxDesc = DMARxDscrTab;
    g_eth_handler.Init.TxDesc = DMATxDscrTab;
    g_eth_handler.Init.RxBuffLen = ETH_MAX_PACKET_SIZE;

    if (HAL_ETH_Init(&g_eth_handler) != HAL_OK)
    {
        return 1; /* 失败 */
    }

    ETH_MACFilterConfigTypeDef filterDef;

    // 获取当前硬件配置
    HAL_ETH_GetMACFilterConfig(&g_eth_handler, &filterDef);

    // 启用混杂模式
    filterDef.PromiscuousMode = ENABLE;

    // 将配置写回硬件寄存器
    HAL_ETH_SetMACFilterConfig(&g_eth_handler, &filterDef);

    return 0; /* 成功 */
}

/**
 * @brief       ETH底层驱动，时钟使能，引脚配置
 *    @note     此函数会被HAL_ETH_Init()调用
 * @param       heth:以太网句柄
 * @retval      无
 */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    GPIO_InitTypeDef gpio_init_struct;

    ETH_CLK_GPIO_CLK_ENABLE();   /* 开启ETH_CLK时钟 */
    ETH_MDIO_GPIO_CLK_ENABLE();  /* 开启ETH_MDIO时钟 */
    ETH_CRS_GPIO_CLK_ENABLE();   /* 开启ETH_CRS时钟 */
    ETH_MDC_GPIO_CLK_ENABLE();   /* 开启ETH_MDC时钟 */
    ETH_RXD0_GPIO_CLK_ENABLE();  /* 开启ETH_RXD0时钟 */
    ETH_RXD1_GPIO_CLK_ENABLE();  /* 开启ETH_RXD1时钟 */
    ETH_TX_EN_GPIO_CLK_ENABLE(); /* 开启ETH_TX_EN时钟 */
    ETH_TXD0_GPIO_CLK_ENABLE();  /* 开启ETH_TXD0时钟 */
    ETH_TXD1_GPIO_CLK_ENABLE();  /* 开启ETH_TXD1时钟 */
    // ETH_RESET_GPIO_CLK_ENABLE();        /* 开启ETH_RESET时钟 */
    __HAL_RCC_ETH1MAC_CLK_ENABLE(); /* 开启ETH时钟 */
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();

    /* 网络引脚设置 RMII接口
     * ETH_MDIO -------------------------> PA2
     * ETH_MDC --------------------------> PC1
     * ETH_RMII_REF_CLK------------------> PA1
     * ETH_RMII_CRS_DV ------------------> PA7
     * ETH_RMII_RXD0 --------------------> PC4
     * ETH_RMII_RXD1 --------------------> PC5
     * ETH_RMII_TX_EN -------------------> PG11
     * ETH_RMII_TXD0 --------------------> PG13
     * ETH_RMII_TXD1 --------------------> PG14
     */

    /* PA1,2,7 */
    gpio_init_struct.Pin = ETH_CLK_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;             /* 推挽复用 */
    gpio_init_struct.Pull = GPIO_NOPULL;                 /* 不带上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;            /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF11_ETH;          /* 复用为ETH功能 */
    HAL_GPIO_Init(ETH_CLK_GPIO_PORT, &gpio_init_struct); /* ETH_CLK引脚模式设置 */

    gpio_init_struct.Pin = ETH_MDIO_GPIO_PIN;
    HAL_GPIO_Init(ETH_MDIO_GPIO_PORT, &gpio_init_struct); /* ETH_MDIO引脚模式设置 */

    gpio_init_struct.Pin = ETH_CRS_GPIO_PIN;
    HAL_GPIO_Init(ETH_CRS_GPIO_PORT, &gpio_init_struct); /* ETH_CRS引脚模式设置 */

    /* PC1 */
    gpio_init_struct.Pin = ETH_MDC_GPIO_PIN;
    HAL_GPIO_Init(ETH_MDC_GPIO_PORT, &gpio_init_struct); /* ETH_MDC初始化 */

    /* PC4 */
    gpio_init_struct.Pin = ETH_RXD0_GPIO_PIN;
    HAL_GPIO_Init(ETH_RXD0_GPIO_PORT, &gpio_init_struct); /* ETH_RXD0初始化 */

    /* PC5 */
    gpio_init_struct.Pin = ETH_RXD1_GPIO_PIN;
    HAL_GPIO_Init(ETH_RXD1_GPIO_PORT, &gpio_init_struct); /* ETH_RXD1初始化 */

    /* PG11,13,14 */
    gpio_init_struct.Pin = ETH_TX_EN_GPIO_PIN;
    HAL_GPIO_Init(ETH_TX_EN_GPIO_PORT, &gpio_init_struct); /* ETH_TX_EN初始化 */

    gpio_init_struct.Pin = ETH_TXD0_GPIO_PIN;
    HAL_GPIO_Init(ETH_TXD0_GPIO_PORT, &gpio_init_struct); /* ETH_TXD0初始化 */

    gpio_init_struct.Pin = ETH_TXD1_GPIO_PIN;
    HAL_GPIO_Init(ETH_TXD1_GPIO_PORT, &gpio_init_struct); /* ETH_TXD1初始化 */

    /* 复位引脚 */
    sys_intx_disable();                 /* 关闭所有中断，复位过程不能被打断！ */
    pcf8574_write_bit(ETH_RESET_IO, 0); /* 硬件复位 */
    delay_ms(100);
    pcf8574_write_bit(ETH_RESET_IO, 1); /* 复位结束 */
    delay_ms(100);
    sys_intx_enable(); /* 开启所有中断 */
}

/**
 * @breif       读取以太网芯片寄存器值
 * @param       reg：读取的寄存器地址
 * @retval      regval：返回读取的寄存器值
 */
uint32_t ethernet_read_phy(uint16_t reg)
{
    uint32_t regval = 0;

    HAL_ETH_ReadPHYRegister(&g_eth_handler, ETH_CHIP_ADDR, reg, &regval);
    return regval;
}

/**
 * @breif       向以太网芯片指定地址写入寄存器值
 * @param       reg   : 要写入的寄存器
 * @param       value : 要写入的寄存器
 * @retval      无
 */
void ethernet_write_phy(uint16_t reg, uint16_t value)
{
    uint32_t temp = value;

    HAL_ETH_ReadPHYRegister(&g_eth_handler, ETH_CHIP_ADDR, reg, &temp);
}

/**
 * @breif       获得网络芯片的速度模式
 * @param       无
 * @retval      1:100M
                0:10M
 */
uint8_t ethernet_chip_get_speed(void)
{
    uint8_t speed;
#if (PHY_TYPE == LAN8720)
    speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 4); /* 从LAN8720的31号寄存器中读取网络速度和双工模式 */
#elif (PHY_TYPE == SR8201F)
    speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 13); /* 从SR8201F的0号寄存器中读取网络速度和双工模式 */
#elif (PHY_TYPE == YT8512C)
    speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 14); /* 从YT8512C的17号寄存器中读取网络速度和双工模式 */
#elif (PHY_TYPE == RTL8201)
    speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 1); /* 从RTL8201的16号寄存器中读取网络速度和双工模式 */
#elif (PHY_TYPE == DM9162)
    speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 15); /* 从DM9161的17号寄存器中读取网络速度和双工模式 */
#endif
    return speed;
}

/**
 * @brief       判断网线是否连接
 * @return      连接状态
 *   @retval    1:已连接
 *   @retval    0:未连接
 */
uint8_t ethernet_link_status(void)
{
    uint32_t phy_reg_value = 0;

    if (HAL_ETH_ReadPHYRegister(&g_eth_handler, ETH_CHIP_ADDR, ETH_CHIP_BSR, &phy_reg_value) == HAL_OK)
    {
        /* 检查连接状态位（bit2） */
        if (phy_reg_value & (1 << 2))
        {
            return 1; /* 已连接 */
        }
    }

    return 0; /* 未连接 */
}
