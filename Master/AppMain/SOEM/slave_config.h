/**
 * @file         : slave_config.h
 * @author       : ShadowFury
 * @date         : 2025-06-18
 * @version      : 1.0
 * @brief        : 从站配置文件，包括从站ID，PDO配置等
 */
#ifndef __SLAVE_CONFIG_H
#define __SLAVE_CONFIG_H

// 1. 头文件
#include <stdint.h>

// 2. 宏定义
#define SLAVE_EEP_ID 0x00000001 // 从站 EEP ID
#define SLAVE_NUM 2             // 从站数量

/**
 * @defgroup 控制模式
 */
#define CSP_CONTROL_MODE 0x08 // CSP运行模式

/**
 * @defgroup 映射对象索引
 */
#define RXPDO1_INDEX_1600 0x1600 // RXPDO1索引
#define TXPDO1_INDEX_1A00 0x1A00 // TXPDO1索引

/**
 * @defgroup RXPDO 目录
 */
#define TARGET_POSITION 0x607A0020 // 目标位置 (I32)
#define TARGET_VELOCITY 0x60FF0020 // 目标速度 (I32)
#define TARGET_TORQUE 0x60710010   // 目标转矩 (I16)
#define CONTROL_WORD 0x60400010    // 控制字   (U16)
#define CONTROL_MODE 0x60600008    // 控制模式 (U8)

/**
 * @defgroup TXPDO 目录
 */
#define ACTUAL_POSITION 0x60640020 // 实际位置 (I32)
#define ACTUAL_VELOCITY 0x606C0020 // 实际速度 (I32)
#define ACTUAL_TORQUE 0x60770010   // 实际扭矩 (I16)
#define STATUS_WORD 0x60410010     // 状态字   (U16)
#define CURRENT_MODE 0x60610008    // 当前运行模式(U8)

// 3. 类型定义

// PdoOutput 类
/* zxr: GCC 按字节紧凑布局，保持原 PDO 字段偏移。 */
typedef struct __attribute__((packed))
{
    int32_t nTargetPosition; // 目标位置
    uint16_t usControlWord;  // 控制字
    uint8_t ucControlMode;   // 控制模式
} PdoOutput;

// PdoInput 类
/* zxr: GCC 按字节紧凑布局，保持原 PDO 字段偏移。 */
typedef struct __attribute__((packed))
{
    int32_t nActualPosition; // 实际位置
    uint16_t usStatusWord;   // 状态字
    uint8_t ucCurrentMode;   // 当前运行模式
} PdoInput;

typedef struct
{
    uint32_t object;  // 对象字典项
    uint8_t subindex; // 子索引
} PdoMappingEntry;

// RXPDO映射配置表
static const PdoMappingEntry rxPdoConfig[] = {
    {TARGET_POSITION, 0x00}, // 目标位置
    {CONTROL_WORD, 0x00},    // 控制字
    {CONTROL_MODE, 0x00}     // 控制模式
};

// TXPDO映射配置表
static const PdoMappingEntry txPdoConfig[] = {
    {ACTUAL_POSITION, 0x00}, // 实际位置
    {STATUS_WORD, 0x00},     // 状态字
    {CURRENT_MODE, 0x00}     // 当前模式
};

#define RXPDO_MAP_COUNT (sizeof(rxPdoConfig) / sizeof(rxPdoConfig[0]))
#define TXPDO_MAP_COUNT (sizeof(txPdoConfig) / sizeof(txPdoConfig[0]))

#endif /* __SLAVE_CONFIG_H */
