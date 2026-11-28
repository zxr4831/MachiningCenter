/**
 * @file         : slave_config.h
 * @author       : zxr
 * @brief        : 从站设备描述、逻辑轴绑定表、PDO 偏移访问及启动配置核验结构
 */
#ifndef SLAVE_CONFIG_H
#define SLAVE_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ==================== 从站数量与身份标识常量 ==================== */
/* zxr: 当前总线在线伺服从站数量（3轴默认=3，双轴台架可切为2）；与机床参数层完全解耦。 */
#ifndef SLAVE_NUM
#define SLAVE_NUM 3U
#endif
#define SLAVE_EEP_ID_DRIVE_A 0x00000001U
#define SLAVE_VERIFY_ANY_U32 0U

/* ==================== 控制模式与 PDO 对象字典常量 ==================== */
#define CSP_CONTROL_MODE 0x08U /* CSP 周期同步位置模式 */

#define RXPDO1_INDEX_1600 0x1600U
#define TXPDO1_INDEX_1A00 0x1A00U

/* RXPDO 对象字典条目 (index[16] | subindex[8] | bitlen[8]) */
#define TARGET_POSITION 0x607A0020U /* 目标位置 (I32) */
#define TARGET_VELOCITY 0x60FF0020U /* 目标速度 (I32) */
#define TARGET_TORQUE 0x60710010U   /* 目标转矩 (I16) */
#define CONTROL_WORD 0x60400010U    /* 控制字   (U16) */
#define CONTROL_MODE 0x60600008U    /* 控制模式 (U8)  */

/* TXPDO 对象字典条目 (index[16] | subindex[8] | bitlen[8]) */
#define ACTUAL_POSITION 0x60640020U /* 实际位置 (I32) */
#define ACTUAL_VELOCITY 0x606C0020U /* 实际速度 (I32) */
#define ACTUAL_TORQUE 0x60770010U   /* 实际扭矩 (I16) */
#define STATUS_WORD 0x60410010U     /* 状态字   (U16) */
#define CURRENT_MODE 0x60610008U    /* 当前模式 (U8)  */

/* ==================== 第 1 层：设备描述 Profile ==================== */
typedef struct
{
    uint32_t unObject;
} SPdoMappingEntry;

typedef struct
{
    uint16_t usRxBytes;              /* RXPDO 总字节数 */
    uint16_t usTxBytes;              /* TXPDO 总字节数 */
    uint16_t usOffsetTargetPosition; /* 0x607A 字节偏移 (I32) */
    uint16_t usOffsetControlWord;    /* 0x6040 字节偏移 (U16) */
    uint16_t usOffsetControlMode;    /* 0x6060 字节偏移 (U8)  */
    uint16_t usOffsetActualPosition; /* 0x6064 字节偏移 (I32) */
    uint16_t usOffsetStatusWord;     /* 0x6041 字节偏移 (U16) */
    uint16_t usOffsetCurrentMode;    /* 0x6061 字节偏移 (U8)  */
} SServoPdoLayout;

typedef struct
{
    const char *pcName;
    uint32_t unVendorId; /* 0 (SLAVE_VERIFY_ANY_U32) 表示仅回读记录、不拦截 */
    uint32_t unProductCode;
    uint32_t unRevision; /* 0 (SLAVE_VERIFY_ANY_U32) 表示仅回读记录、不拦截 */
    uint16_t usRxPdoIndex;
    uint16_t usTxPdoIndex;
    uint8_t ucRxMapCount;
    uint8_t ucTxMapCount;
    const SPdoMappingEntry *pSRxMap;
    const SPdoMappingEntry *pSTxMap;
    SServoPdoLayout SPdoLayout;
    uint8_t ucControlMode;
    uint32_t unDcCycleNs;
    int32_t nDcShiftNs;
    /* 可选换算对象预期值 (0x608F / 0x6091)；0 表示仅回读不拦截 */
    uint32_t unExpectedEncoderIncrements;
    uint32_t unExpectedEncoderRevolutions;
    uint32_t unExpectedGearNumerator;
    uint32_t unExpectedGearDenominator;
} SServoProfile;

/* ==================== 第 3 层：从站索引与物理轴绑定 ==================== */
typedef struct
{
    uint16_t usSlaveIndex;          /* EtherCAT 主站扫描从站序号 (1 开始) */
    uint8_t ucPhysicalAxis;         /* 对应数控/机床物理轴号 (0 开始) */
    const SServoProfile *pSProfile; /* 指向该从站选用的设备描述 Profile */
} SSlaveBinding;

/* ==================== 上电配置核验清单与错误归因 ==================== */
typedef enum
{
    SLAVE_VERIFY_OK = 0,
    SLAVE_VERIFY_ERR_BINDING,     /* 绑定表本身配置非法或越界 */
    SLAVE_VERIFY_ERR_COUNT,       /* 在线从站数与预期数量不符 */
    SLAVE_VERIFY_ERR_IDENTITY,    /* unVendorId / unProductCode / unRevision 不匹配 */
    SLAVE_VERIFY_ERR_SDO_MAP,     /* SM/PDO 映射写入或 SDO 回读校验失败 */
    SLAVE_VERIFY_ERR_SAFE_OP,     /* 从站未能进入 SAFE_OP */
    SLAVE_VERIFY_ERR_PDO_LAYOUT,  /* 映射后的 PDO 字节/位长或指针非法 */
    SLAVE_VERIFY_ERR_SCALE_PARAM, /* 0x608F / 0x6091 与预期值不匹配 */
    SLAVE_VERIFY_ERR_OP           /* 未能进入 OP 或 WKC 异常 */
} SLAVE_VERIFY_ERROR;

typedef struct
{
    uint16_t usSlaveIndex;
    uint8_t ucPhysicalAxis;
    uint8_t ucVerified; /* 1: 全部核验项通过并进入 OP; 0: 未完成或存在失败项 */
    SLAVE_VERIFY_ERROR eError;
    uint16_t usFailedSdoIndex;
    uint8_t ucFailedSdoSubindex;

    uint32_t unExpectedVendorId;
    uint32_t unActualVendorId;
    uint32_t unExpectedProductCode;
    uint32_t unActualProductCode;
    uint32_t unExpectedRevision;
    uint32_t unActualRevision;

    uint16_t usExpectedRxBytes;
    uint16_t usActualRxBytes;
    uint16_t usActualOutputBits;
    uint16_t usExpectedTxBytes;
    uint16_t usActualTxBytes;
    uint16_t usActualInputBits;

    uint8_t ucExpectedControlMode;
    uint8_t ucActualControlMode;
    uint32_t unExpectedDcCycleNs;
    uint8_t ucHasDc;

    uint32_t unExpectedEncoderIncrements;
    uint32_t unActualEncoderIncrements;
    int nEncoderIncrementsReadStatus;
    uint32_t unExpectedEncoderRevolutions;
    uint32_t unActualEncoderRevolutions;
    int nEncoderRevolutionsReadStatus;
    uint32_t unExpectedGearNumerator;
    uint32_t unActualGearNumerator;
    int nGearNumeratorReadStatus;
    uint32_t unExpectedGearDenominator;
    uint32_t unActualGearDenominator;
    int nGearDenominatorReadStatus;
} SSlaveVerifyReport;

/* ==================== PDO 布局校验与偏移读写工具 ==================== */
static inline bool doSlaveRangesOverlap(uint16_t usFirstOffset, uint16_t usFirstLength, uint16_t usSecondOffset, uint16_t usSecondLength)
{
    uint32_t unFirstEnd = (uint32_t)usFirstOffset + (uint32_t)usFirstLength;
    uint32_t unSecondEnd = (uint32_t)usSecondOffset + (uint32_t)usSecondLength;
    return (uint32_t)usFirstOffset < unSecondEnd && (uint32_t)usSecondOffset < unFirstEnd;
}

static inline bool slaveProfile_validateLayout(const SServoProfile *pSProfile)
{
    if (pSProfile == NULL)
    {
        return false;
    }

    const SServoPdoLayout *pSLayout = &pSProfile->SPdoLayout;
    if (pSLayout->usRxBytes < 7U || pSLayout->usTxBytes < 7U)
    {
        return false;
    }

    if ((uint32_t)pSLayout->usOffsetTargetPosition + 4U > pSLayout->usRxBytes || (uint32_t)pSLayout->usOffsetControlWord + 2U > pSLayout->usRxBytes || (uint32_t)pSLayout->usOffsetControlMode + 1U > pSLayout->usRxBytes)
    {
        return false;
    }
    if (doSlaveRangesOverlap(pSLayout->usOffsetTargetPosition, 4U, pSLayout->usOffsetControlWord, 2U) || doSlaveRangesOverlap(pSLayout->usOffsetTargetPosition, 4U, pSLayout->usOffsetControlMode, 1U)
        || doSlaveRangesOverlap(pSLayout->usOffsetControlWord, 2U, pSLayout->usOffsetControlMode, 1U))
    {
        return false;
    }

    if ((uint32_t)pSLayout->usOffsetActualPosition + 4U > pSLayout->usTxBytes || (uint32_t)pSLayout->usOffsetStatusWord + 2U > pSLayout->usTxBytes || (uint32_t)pSLayout->usOffsetCurrentMode + 1U > pSLayout->usTxBytes)
    {
        return false;
    }
    if (doSlaveRangesOverlap(pSLayout->usOffsetActualPosition, 4U, pSLayout->usOffsetStatusWord, 2U) || doSlaveRangesOverlap(pSLayout->usOffsetActualPosition, 4U, pSLayout->usOffsetCurrentMode, 1U)
        || doSlaveRangesOverlap(pSLayout->usOffsetStatusWord, 2U, pSLayout->usOffsetCurrentMode, 1U))
    {
        return false;
    }

    return true;
}

static inline void slavePdo_writeI32(uint8_t *pucBase, uint16_t usOffset, int32_t nValue) { memcpy(pucBase + usOffset, &nValue, sizeof(nValue)); }
static inline void slavePdo_writeU16(uint8_t *pucBase, uint16_t usOffset, uint16_t usValue) { memcpy(pucBase + usOffset, &usValue, sizeof(usValue)); }
static inline void slavePdo_writeU8(uint8_t *pucBase, uint16_t usOffset, uint8_t ucValue) { pucBase[usOffset] = ucValue; }

static inline int32_t slavePdo_readI32(const uint8_t *pucBase, uint16_t usOffset)
{
    int32_t nValue = 0;
    memcpy(&nValue, pucBase + usOffset, sizeof(nValue));
    return nValue;
}

static inline uint16_t slavePdo_readU16(const uint8_t *pucBase, uint16_t usOffset)
{
    uint16_t usValue = 0;
    memcpy(&usValue, pucBase + usOffset, sizeof(usValue));
    return usValue;
}

static inline uint8_t slavePdo_readU8(const uint8_t *pucBase, uint16_t usOffset) { return pucBase[usOffset]; }

/* ==================== 从站配置与绑定表查询接口 ==================== */
uint8_t slaveConfig_getCount(void);
const SSlaveBinding *slaveConfig_getBinding(uint8_t ucSlot);
const SSlaveBinding *slaveConfig_findBySlaveIndex(uint16_t usSlaveIndex);
const SSlaveBinding *slaveConfig_findByPhysicalAxis(uint8_t ucPhysicalAxis);

#endif /* SLAVE_CONFIG_H */
