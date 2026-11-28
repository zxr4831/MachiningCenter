/**
 * @file         : app_soem.h
 * @author       : ShadowFury
 * @date         : 2025-06-19
 * @version      : 1.0
 * @brief        : EtherCAT 主站应用层代码
 */

#ifndef APP_SOEM_H
#define APP_SOEM_H

/* 1. 头文件 */
#include "ethercat.h"
#include "ethernet.h"
#include "slave_config.h"

/* 2. 宏定义 */
// 接收帧的超时时间
#define EC_RX_TIMEOUT_US 200

/* zxr: 对外保留主站初始化、从站配置及启动核验结果读取接口，SDO 辅助函数由 .c 内部管理。 */
/**
 * @brief: 主站配置
 * @return 1: 成功, 0: 失败
 */
int master_init(void);

/**
 * @brief: 从站配置与预运行核验
 * @return 1: 成功扫描、核验并配置所有从站到 OP 状态, 0: 失败
 */
int master_initSlaves(void);

/**
 * @brief: 按配置槽位读取最近一次上电核验结果（0 <= ucSlot < slaveConfig_getCount()）
 * @return 只读核验报告指针，越界返回 NULL
 */
const SSlaveVerifyReport *master_getSlaveVerifyReport(uint8_t ucSlot);

#endif /* APP_SOEM_H */
