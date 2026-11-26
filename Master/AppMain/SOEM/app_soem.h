/**
 * @file         : app_soem.h
 * @author       : ShadowFury
 * @date         : 2025-06-19
 * @version      : 1.0
 * @brief        : EtherCAT 主站应用层代码
 */

#ifndef __APP_ETHERCAT_MASTER_H__
#define __APP_ETHERCAT_MASTER_H__

/* 1. 头文件 */
#include "ethercat.h"
#include "ethernet.h"
#include "slave_config.h"

/* 2. 宏定义 */
// 接收帧的超时时间
#define EC_RX_TIMEOUT_US 200

/* zxr: 保留初始化所需的 PDO 指针，移除旧运动状态字段。 */
/* zxr: GCC 按字节紧凑布局，保持原 PDO 字段偏移。 */
typedef struct __attribute__((packed)) ServoDrive
{
    PdoOutput *pPdoOutput; // PDO输出指针
    PdoInput *pPdoInput;   // PDO输入指针
} ServoDrive;

/* zxr: 对外仅保留主站和从站初始化接口，SDO 辅助函数由 .c 文件内部管理。 */
/**
 * @brief: 主站配置
 * @return 1: 成功, 0: 失败
 */
int masterInit(void);

/**
 * @brief: 从站配置
 * @return 1: 成功扫描并配置所有从站到 OP 状态, 0: 未找到从站或配置失败
 */
int masterSlaveInfoInit(void);

#endif /* __APP_ETHERCAT_MASTER_H__ */
