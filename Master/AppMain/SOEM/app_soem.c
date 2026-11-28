/**
 * @file         : app_soem.c
 * @author       : ShadowFury
 * @date         : 2025-06-19
 * @version      : 1.0
 * @brief        : EtherCAT 主站应用层代码
 * @note         : zxr: 清理已停用的旧运动与状态机代码，保留总线初始化和 SDO 配置。
 */
#include <stdio.h>
#include <string.h>

#include "app_soem.h"

// 定义PDO映射
static char s_cIoMap[4096];

/* zxr: 记录各从站 PRE_OP -> SAFE_OP SDO 映射配置状态，供初始化失败时精准定位。 */
typedef struct
{
    uint8_t ucSdoCalled;
    uint8_t ucSdoFailed;
    uint16_t usFailedIndex;
    uint8_t ucFailedSubindex;
} SSlaveSdoStatus;

typedef struct
{
    uint32_t unValue;
    int nReadStatus;
} SServoParamU32;

/* ==================== 第 1 层：同型号伺服默认设备描述（Drive A） ==================== */
static const SPdoMappingEntry RX_PDO_DRIVE_A[] = {
    {TARGET_POSITION},
    {CONTROL_WORD},
    {CONTROL_MODE},
};

static const SPdoMappingEntry TX_PDO_DRIVE_A[] = {
    {ACTUAL_POSITION},
    {STATUS_WORD},
    {CURRENT_MODE},
};

static const SServoProfile PROFILE_DRIVE_A = {
    .pcName = "DriveA_CSP",
    .unVendorId = SLAVE_VERIFY_ANY_U32,
    .unProductCode = SLAVE_EEP_ID_DRIVE_A,
    .unRevision = SLAVE_VERIFY_ANY_U32,
    .usRxPdoIndex = RXPDO1_INDEX_1600,
    .usTxPdoIndex = TXPDO1_INDEX_1A00,
    .ucRxMapCount = (uint8_t)(sizeof(RX_PDO_DRIVE_A) / sizeof(RX_PDO_DRIVE_A[0])),
    .ucTxMapCount = (uint8_t)(sizeof(TX_PDO_DRIVE_A) / sizeof(TX_PDO_DRIVE_A[0])),
    .pSRxMap = RX_PDO_DRIVE_A,
    .pSTxMap = TX_PDO_DRIVE_A,
    .SPdoLayout =
        {
            .usRxBytes = 7U,
            .usTxBytes = 7U,
            .usOffsetTargetPosition = 0U,
            .usOffsetControlWord = 4U,
            .usOffsetControlMode = 6U,
            .usOffsetActualPosition = 0U,
            .usOffsetStatusWord = 4U,
            .usOffsetCurrentMode = 6U,
        },
    .ucControlMode = CSP_CONTROL_MODE,                    /* CiA402 模式 (0x6060): 0x08=CSP 周期同步位置 */
    .unDcCycleNs = 1000000U,                              /* DC SYNC0 周期 1ms，与 TIM3 对齐 */
    .nDcShiftNs = 250000,                                 /* DC SYNC0 偏移 250us，错开 PDO 发帧窗口 */
    .unExpectedEncoderIncrements = SLAVE_VERIFY_ANY_U32,  /* 0x608F:01 编码器分辨率分子（分母指定圈数对应的编码器增量数）；0=仅回读不拦截 */
    .unExpectedEncoderRevolutions = SLAVE_VERIFY_ANY_U32, /* 0x608F:02 编码器分辨率分母（电机圈数） */
    .unExpectedGearNumerator = SLAVE_VERIFY_ANY_U32,      /* 0x6091:01 电子齿轮比分子（电机圈数） */
    .unExpectedGearDenominator = SLAVE_VERIFY_ANY_U32,    /* 0x6091:02 电子齿轮比分母（输出轴圈数） */
};

/* ==================== 第 3 层：从站序号 -> 物理轴 -> 设备 Profile 绑定表 ==================== */
static const SSlaveBinding SLAVE_BINDINGS[] = {
    {.usSlaveIndex = 1U, .ucPhysicalAxis = 0U, .pSProfile = &PROFILE_DRIVE_A},
    {.usSlaveIndex = 2U, .ucPhysicalAxis = 1U, .pSProfile = &PROFILE_DRIVE_A},
    {.usSlaveIndex = 3U, .ucPhysicalAxis = 2U, .pSProfile = &PROFILE_DRIVE_A},
};

#define SLAVE_BINDING_ARRAY_COUNT (sizeof(SLAVE_BINDINGS) / sizeof(SLAVE_BINDINGS[0]))

static volatile SSlaveSdoStatus s_SSdoStatus[SLAVE_NUM];
static SSlaveVerifyReport s_SVerifyReports[SLAVE_NUM];

// 调试宏 用于打印信息
#define SOEM_DEBUG 0

/* zxr: SDO 配置是本文件内部回调，供主站初始化注册。 */
static int configureServoSdo(uint16_t usSlave);

uint8_t slaveConfig_getCount(void)
{
    uint32_t unMaxCount = (uint32_t)SLAVE_BINDING_ARRAY_COUNT;
    if ((uint32_t)SLAVE_NUM < unMaxCount)
    {
        return (uint8_t)SLAVE_NUM;
    }
    return (uint8_t)unMaxCount;
}

const SSlaveBinding *slaveConfig_getBinding(uint8_t ucSlot)
{
    if (ucSlot >= slaveConfig_getCount())
    {
        return NULL;
    }
    return &SLAVE_BINDINGS[ucSlot];
}

static int findSlotBySlaveIndex(uint16_t usSlaveIndex)
{
    uint8_t ucCount = slaveConfig_getCount();
    for (uint8_t i = 0; i < ucCount; i++)
    {
        if (SLAVE_BINDINGS[i].usSlaveIndex == usSlaveIndex)
        {
            return (int)i;
        }
    }
    return -1;
}

const SSlaveBinding *slaveConfig_findBySlaveIndex(uint16_t usSlaveIndex)
{
    int nSlot = findSlotBySlaveIndex(usSlaveIndex);
    return nSlot >= 0 ? &SLAVE_BINDINGS[nSlot] : NULL;
}

const SSlaveBinding *slaveConfig_findByPhysicalAxis(uint8_t ucPhysicalAxis)
{
    uint8_t ucCount = slaveConfig_getCount();
    for (uint8_t i = 0; i < ucCount; i++)
    {
        if (SLAVE_BINDINGS[i].ucPhysicalAxis == ucPhysicalAxis)
        {
            return &SLAVE_BINDINGS[i];
        }
    }
    return NULL;
}

const SSlaveVerifyReport *master_getSlaveVerifyReport(uint8_t ucSlot)
{
    if (ucSlot >= slaveConfig_getCount())
    {
        return NULL;
    }
    return &s_SVerifyReports[ucSlot];
}

static bool validateBindingTable(void)
{
    if ((uint32_t)SLAVE_NUM == 0U || (uint32_t)SLAVE_NUM > SLAVE_BINDING_ARRAY_COUNT)
    {
        return false;
    }

    uint8_t ucCount = slaveConfig_getCount();
    bool bSeenSlave[SLAVE_NUM] = {false};
    bool bSeenAxis[SLAVE_NUM] = {false};

    for (uint8_t i = 0; i < ucCount; i++)
    {
        const SSlaveBinding *pSBinding = &SLAVE_BINDINGS[i];
        const SServoProfile *pSProfile = pSBinding->pSProfile;
        if (pSBinding->usSlaveIndex < 1U || pSBinding->usSlaveIndex > ucCount || bSeenSlave[pSBinding->usSlaveIndex - 1U])
        {
            s_SVerifyReports[i].eError = SLAVE_VERIFY_ERR_BINDING;
            return false;
        }
        if (pSBinding->ucPhysicalAxis >= ucCount || bSeenAxis[pSBinding->ucPhysicalAxis])
        {
            s_SVerifyReports[i].eError = SLAVE_VERIFY_ERR_BINDING;
            return false;
        }
        if (pSProfile == NULL || pSProfile->pSRxMap == NULL || pSProfile->pSTxMap == NULL || pSProfile->ucRxMapCount == 0U || pSProfile->ucTxMapCount == 0U || !slaveProfile_validateLayout(pSProfile))
        {
            s_SVerifyReports[i].eError = SLAVE_VERIFY_ERR_BINDING;
            return false;
        }
        bSeenSlave[pSBinding->usSlaveIndex - 1U] = true;
        bSeenAxis[pSBinding->ucPhysicalAxis] = true;

        s_SVerifyReports[i].usSlaveIndex = pSBinding->usSlaveIndex;
        s_SVerifyReports[i].ucPhysicalAxis = pSBinding->ucPhysicalAxis;
        s_SVerifyReports[i].unExpectedVendorId = pSProfile->unVendorId;
        s_SVerifyReports[i].unExpectedProductCode = pSProfile->unProductCode;
        s_SVerifyReports[i].unExpectedRevision = pSProfile->unRevision;
        s_SVerifyReports[i].usExpectedRxBytes = pSProfile->SPdoLayout.usRxBytes;
        s_SVerifyReports[i].usExpectedTxBytes = pSProfile->SPdoLayout.usTxBytes;
        s_SVerifyReports[i].ucExpectedControlMode = pSProfile->ucControlMode;
        s_SVerifyReports[i].unExpectedDcCycleNs = pSProfile->unDcCycleNs;
        s_SVerifyReports[i].unExpectedEncoderIncrements = pSProfile->unExpectedEncoderIncrements;
        s_SVerifyReports[i].unExpectedEncoderRevolutions = pSProfile->unExpectedEncoderRevolutions;
        s_SVerifyReports[i].unExpectedGearNumerator = pSProfile->unExpectedGearNumerator;
        s_SVerifyReports[i].unExpectedGearDenominator = pSProfile->unExpectedGearDenominator;
    }

    return true;
}

/**
 * @brief: 主站配置
 * @return 1: 成功, 0: 失败
 */
int master_init(void)
{
    while (!ethernet_link_status())
    {
        printf("Please plug in the Ethernet cable first\r\n");
        HAL_Delay(1000);
    }

    // 初始化 SOEM 主站协议栈与网口上下文（不切换从站状态）
    if (!ec_init("test"))
    {
        printf("ec_init error\r\n");
        return 0;
    }

    return 1;
}

/**
 * @brief: 从站配置
 * @return 1: 成功扫描并配置所有从站到 OP 状态, 0: 未找到从站或配置失败
 */
int master_initSlaves(void)
{
    /* zxr: 每次初始化清空核验清单，避免读取失败或从站缺失时保留旧数据。 */
    const SSlaveSdoStatus EMPTY_SDO_STATUS = {0};
    for (uint8_t i = 0; i < SLAVE_NUM; i++)
    {
        memset(&s_SVerifyReports[i], 0, sizeof(s_SVerifyReports[i]));
        s_SSdoStatus[i] = EMPTY_SDO_STATUS;
    }

    if (!validateBindingTable())
    {
        printf("Error: Slave binding/profile table invalid!\r\n");
        return 0;
    }

    uint8_t ucExpectedCount = slaveConfig_getCount();

    // [INIT -> PRE_OP] 扫描并初始化从站：先切入 INIT，再请求进入 PRE_OP，写入 ec_slavecount
    if (ec_config_init(TRUE) <= 0)
    {
        printf("No slaves found!\r\n");
        s_SVerifyReports[0].eError = SLAVE_VERIFY_ERR_COUNT;
        return 0;
    }

    printf("%d slaves found and configured.\r\n", ec_slavecount);

    // 强制要求接满全部预定从站，避免少接或多接后按固定数组越界/空指针访问
    if (ec_slavecount != ucExpectedCount)
    {
        printf("Error: Slave count mismatch! expected=%u, found=%d\r\n", (unsigned int)ucExpectedCount, ec_slavecount);
        for (uint8_t i = 0; i < ucExpectedCount; i++)
        {
            s_SVerifyReports[i].eError = SLAVE_VERIFY_ERR_COUNT;
        }
        return 0;
    }

    // 读取每个从站的状态并按绑定表逐台核对身份
    ec_readstate();
    for (uint16_t usSlaveIndex = 1; usSlaveIndex <= (uint16_t)ec_slavecount; usSlaveIndex++)
    {
        int nSlot = findSlotBySlaveIndex(usSlaveIndex);
        if (nSlot < 0)
        {
            printf("Error: Slave %u missing binding entry\r\n", (unsigned int)usSlaveIndex);
            return 0;
        }

        const SSlaveBinding *pSBinding = slaveConfig_getBinding((uint8_t)nSlot);
        const SServoProfile *pSProfile = pSBinding->pSProfile;
        SSlaveVerifyReport *pSReport = &s_SVerifyReports[nSlot];

        pSReport->unActualVendorId = ec_slave[usSlaveIndex].eep_man;
        pSReport->unActualProductCode = ec_slave[usSlaveIndex].eep_id;
        pSReport->unActualRevision = ec_slave[usSlaveIndex].eep_rev;
        pSReport->ucHasDc = ec_slave[usSlaveIndex].hasdc;

        if ((pSProfile->unVendorId != SLAVE_VERIFY_ANY_U32 && ec_slave[usSlaveIndex].eep_man != pSProfile->unVendorId) || (pSProfile->unProductCode != SLAVE_VERIFY_ANY_U32 && ec_slave[usSlaveIndex].eep_id != pSProfile->unProductCode)
            || (pSProfile->unRevision != SLAVE_VERIFY_ANY_U32 && ec_slave[usSlaveIndex].eep_rev != pSProfile->unRevision))
        {
            pSReport->eError = SLAVE_VERIFY_ERR_IDENTITY;
            printf("Error: Slave %u (%s) identity mismatch! exp=(0x%08lX,0x%08lX,0x%08lX) act=(0x%08lX,0x%08lX,0x%08lX)\r\n",
                   (unsigned int)usSlaveIndex,
                   pSProfile->pcName,
                   (unsigned long)pSProfile->unVendorId,
                   (unsigned long)pSProfile->unProductCode,
                   (unsigned long)pSProfile->unRevision,
                   (unsigned long)ec_slave[usSlaveIndex].eep_man,
                   (unsigned long)ec_slave[usSlaveIndex].eep_id,
                   (unsigned long)ec_slave[usSlaveIndex].eep_rev);
            return 0;
        }

        printf("Slave %u (axis=%u, %s) Identity: eep_man=0x%08lX eep_id=0x%08lX eep_rev=0x%08lX\r\n",
               (unsigned int)usSlaveIndex,
               (unsigned int)pSBinding->ucPhysicalAxis,
               pSProfile->pcName,
               (unsigned long)ec_slave[usSlaveIndex].eep_man,
               (unsigned long)ec_slave[usSlaveIndex].eep_id,
               (unsigned long)ec_slave[usSlaveIndex].eep_rev);

        // 注册 PRE_OP -> SAFE_OP 的 SDO 配置回调（由 ec_config_map 触发）
        ec_slave[usSlaveIndex].PO2SOconfig = &configureServoSdo;
    }

    // 按各从站 Profile 配置 DC 同步模式
    ec_configdc();
    for (uint16_t usSlaveIndex = 1; usSlaveIndex <= (uint16_t)ec_slavecount; usSlaveIndex++)
    {
        const SSlaveBinding *pSBinding = slaveConfig_findBySlaveIndex(usSlaveIndex);
        const SServoProfile *pSProfile = pSBinding->pSProfile;
        if (pSProfile->unDcCycleNs > 0U)
        {
            ec_dcsync0(usSlaveIndex, TRUE, pSProfile->unDcCycleNs, pSProfile->nDcShiftNs);
        }
    }

    // [PRE_OP -> SAFE_OP] 确认进入 PRE_OP、执行 PO2SOconfig、映射 PDO 到 s_cIoMap，并请求进入 SAFE_OP
    int nIoMapSize = ec_config_map(&s_cIoMap);
    if (nIoMapSize <= 0 || nIoMapSize > (int)sizeof(s_cIoMap))
    {
        printf("Error: ec_config_map failed! size=%d (max=%u)\r\n", nIoMapSize, (unsigned int)sizeof(s_cIoMap));
        for (uint8_t i = 0; i < ucExpectedCount; i++)
        {
            s_SVerifyReports[i].eError = SLAVE_VERIFY_ERR_SDO_MAP;
        }
        return 0;
    }

    // 确认每台从站的 SDO 映射与比例核验均已执行且全部通过
    for (uint16_t usSlaveIndex = 1; usSlaveIndex <= (uint16_t)ec_slavecount; usSlaveIndex++)
    {
        int nSlot = findSlotBySlaveIndex(usSlaveIndex);
        if (nSlot < 0)
        {
            return 0;
        }
        if (!s_SSdoStatus[nSlot].ucSdoCalled || s_SSdoStatus[nSlot].ucSdoFailed)
        {
            if (s_SVerifyReports[nSlot].eError == SLAVE_VERIFY_OK)
            {
                s_SVerifyReports[nSlot].eError = SLAVE_VERIFY_ERR_SDO_MAP;
                s_SVerifyReports[nSlot].usFailedSdoIndex = s_SSdoStatus[nSlot].usFailedIndex;
                s_SVerifyReports[nSlot].ucFailedSdoSubindex = s_SSdoStatus[nSlot].ucFailedSubindex;
            }
            printf("Error: Slave %u SDO config/verify failed! called=%u index=0x%04X sub=%02X\r\n",
                   (unsigned int)usSlaveIndex,
                   (unsigned int)s_SSdoStatus[nSlot].ucSdoCalled,
                   (unsigned int)s_SSdoStatus[nSlot].usFailedIndex,
                   (unsigned int)s_SSdoStatus[nSlot].ucFailedSubindex);
            return 0;
        }
    }

    printf("Slaves mapped, state to SAFE_OP.\n");
    // 等待所有从站确认进入 SAFE_OP
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    ec_readstate();

    for (uint16_t usSlaveIndex = 1; usSlaveIndex <= (uint16_t)ec_slavecount; usSlaveIndex++)
    {
        int nSlot = findSlotBySlaveIndex(usSlaveIndex);
        if (nSlot < 0)
        {
            return 0;
        }
        const SSlaveBinding *pSBinding = slaveConfig_getBinding((uint8_t)nSlot);
        const SServoProfile *pSProfile = pSBinding->pSProfile;
        SSlaveVerifyReport *pSReport = &s_SVerifyReports[nSlot];

        printf("Slave %u State = 0x%04X\r\n", (unsigned int)usSlaveIndex, (unsigned int)ec_slave[usSlaveIndex].state);
        if (ec_slave[usSlaveIndex].state != EC_STATE_SAFE_OP)
        {
            pSReport->eError = SLAVE_VERIFY_ERR_SAFE_OP;
            printf("Error: Slave %u failed to reach SAFE_OP! state=0x%04X ALcode=0x%04X\r\n", (unsigned int)usSlaveIndex, (unsigned int)ec_slave[usSlaveIndex].state, (unsigned int)ec_slave[usSlaveIndex].ALstatuscode);
            return 0;
        }

        pSReport->usActualOutputBits = ec_slave[usSlaveIndex].Obits;
        pSReport->usActualInputBits = ec_slave[usSlaveIndex].Ibits;
        pSReport->usActualRxBytes = (uint16_t)ec_slave[usSlaveIndex].Obytes;
        pSReport->usActualTxBytes = (uint16_t)ec_slave[usSlaveIndex].Ibytes;

        if (ec_slave[usSlaveIndex].outputs == NULL || ec_slave[usSlaveIndex].inputs == NULL || ec_slave[usSlaveIndex].Obits != (uint16_t)(pSProfile->SPdoLayout.usRxBytes * 8U)
            || ec_slave[usSlaveIndex].Ibits != (uint16_t)(pSProfile->SPdoLayout.usTxBytes * 8U) || ec_slave[usSlaveIndex].Ostartbit != 0 || ec_slave[usSlaveIndex].Istartbit != 0 || ec_slave[usSlaveIndex].Obytes < pSProfile->SPdoLayout.usRxBytes
            || ec_slave[usSlaveIndex].Ibytes < pSProfile->SPdoLayout.usTxBytes)
        {
            pSReport->eError = SLAVE_VERIFY_ERR_PDO_LAYOUT;
            printf("Error: Slave %u PDO layout invalid! out=%p (%u bits/%u start/%luB, exp=%uB) in=%p (%u bits/%u start/%luB, exp=%uB)\r\n",
                   (unsigned int)usSlaveIndex,
                   ec_slave[usSlaveIndex].outputs,
                   (unsigned int)ec_slave[usSlaveIndex].Obits,
                   (unsigned int)ec_slave[usSlaveIndex].Ostartbit,
                   (unsigned long)ec_slave[usSlaveIndex].Obytes,
                   (unsigned int)pSProfile->SPdoLayout.usRxBytes,
                   ec_slave[usSlaveIndex].inputs,
                   (unsigned int)ec_slave[usSlaveIndex].Ibits,
                   (unsigned int)ec_slave[usSlaveIndex].Istartbit,
                   (unsigned long)ec_slave[usSlaveIndex].Ibytes,
                   (unsigned int)pSProfile->SPdoLayout.usTxBytes);
            return 0;
        }
    }

#if SOEM_DEBUG
    // 打印分配的信息
    for (uint16_t usSlaveIndex = 1; usSlaveIndex <= ec_slavecount; usSlaveIndex++)
    {
        printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n "
               "State: %d\n Delay: %d[ns]\n Has DC: %d\n",
               usSlaveIndex,
               ec_slave[usSlaveIndex].name,
               ec_slave[usSlaveIndex].Obits,
               ec_slave[usSlaveIndex].Ibits,
               ec_slave[usSlaveIndex].state,
               ec_slave[usSlaveIndex].pdelay,
               ec_slave[usSlaveIndex].hasdc);
        printf(" Configured address: %x\n", ec_slave[usSlaveIndex].configadr);
        printf(" Outputs address: %x\n", ec_slave[usSlaveIndex].outputs);
        printf(" Inputs address: %x\n", ec_slave[usSlaveIndex].inputs);

        for (uint16_t j = 0; j < ec_slave[usSlaveIndex].FMMUunused; j++)
        {
            printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n",
                   j,
                   (int)ec_slave[usSlaveIndex].FMMU[j].LogStart,
                   ec_slave[usSlaveIndex].FMMU[j].LogLength,
                   ec_slave[usSlaveIndex].FMMU[j].LogStartbit,
                   ec_slave[usSlaveIndex].FMMU[j].LogEndbit,
                   ec_slave[usSlaveIndex].FMMU[j].PhysStart,
                   ec_slave[usSlaveIndex].FMMU[j].PhysStartBit,
                   ec_slave[usSlaveIndex].FMMU[j].FMMUtype,
                   ec_slave[usSlaveIndex].FMMU[j].FMMUactive);
        }
        printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n", ec_slave[usSlaveIndex].FMMU0func, ec_slave[usSlaveIndex].FMMU1func, ec_slave[usSlaveIndex].FMMU2func, ec_slave[usSlaveIndex].FMMU3func);
    }
#endif

    printf("Request operational state for all slaves\r\n");

    int nOpWkc = 0;
    int nExpectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    int nAllOp = 0;
    uint32_t unOpStartMs;

    // [SAFE_OP -> OP] 先发一拍有效 PDO，再通过 ec_writestate(0) 广播请求所有从站进入 OP
    ec_slave[0].state = EC_STATE_OPERATIONAL; // 0 表示广播所有从站
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    unOpStartMs = HAL_GetTick();

    // 持续收发 PDO 并轮询状态，等待所有从站确认进入 OP 且 WKC 正常
    do
    {
        ec_send_processdata();
        nOpWkc = ec_receive_processdata(EC_RX_TIMEOUT_US);

        // 读取当前状态，不长时间等待状态变化
        ec_readstate();

        nAllOp = 1;
        for (uint16_t i = 1; i <= ec_slavecount; i++)
        {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                nAllOp = 0;
        }

        if (nAllOp && nOpWkc == nExpectedWkc)
            break;

        osal_usleep(500);
    } while ((uint32_t)(HAL_GetTick() - unOpStartMs) < 5000U);

    // 只在失败时打印；成功则尽快返回，启动 TIM3
    if (!nAllOp || nOpWkc != nExpectedWkc)
    {
        printf("OP failed: WKC=%d/%d\r\n", nOpWkc, nExpectedWkc);

        for (uint16_t i = 1; i <= (uint16_t)ec_slavecount; i++)
        {
            int nSlot = findSlotBySlaveIndex(i);
            if (nSlot >= 0)
            {
                s_SVerifyReports[nSlot].eError = SLAVE_VERIFY_ERR_OP;
            }
            printf("Slave %u ESM=0x%04X ALcode=0x%04X\r\n", (unsigned int)i, (unsigned int)ec_slave[i].state, (unsigned int)ec_slave[i].ALstatuscode);
        }

        return 0;
    }

    // 进入 OP 后将各从站目标位置对齐到实际位置并核验运行模式，防止抖动/飞车
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        for (uint16_t i = 1; i <= (uint16_t)ec_slavecount; i++)
        {
            int nSlot = findSlotBySlaveIndex(i);
            if (nSlot < 0)
            {
                return 0;
            }
            const SSlaveBinding *pSBinding = slaveConfig_getBinding((uint8_t)nSlot);
            const SServoProfile *pSProfile = pSBinding->pSProfile;
            const SServoPdoLayout *pSLayout = &pSProfile->SPdoLayout;
            SSlaveVerifyReport *pSReport = &s_SVerifyReports[nSlot];

            uint8_t *pucOutputPdo = (uint8_t *)ec_slave[i].outputs;
            const uint8_t *pucInputPdo = (const uint8_t *)ec_slave[i].inputs;
            if (pucOutputPdo == NULL || pucInputPdo == NULL)
            {
                pSReport->eError = SLAVE_VERIFY_ERR_PDO_LAYOUT;
                printf("Error: Slave %u PDO pointer NULL in OP\r\n", (unsigned int)i);
                return 0;
            }

            int32_t nActualPosition = slavePdo_readI32(pucInputPdo, pSLayout->usOffsetActualPosition);
            slavePdo_writeI32(pucOutputPdo, pSLayout->usOffsetTargetPosition, nActualPosition);
            slavePdo_writeU16(pucOutputPdo, pSLayout->usOffsetControlWord, 0x0000U);
            slavePdo_writeU8(pucOutputPdo, pSLayout->usOffsetControlMode, pSProfile->ucControlMode);

            pSReport->ucActualControlMode = slavePdo_readU8(pucInputPdo, pSLayout->usOffsetCurrentMode);
            pSReport->ucVerified = 1U;
            pSReport->eError = SLAVE_VERIFY_OK;
        }

        return 1;
    }

    printf("Not all slaves reached operational state.\n");
    ec_readstate();
    for (uint16_t i = 1; i <= (uint16_t)ec_slavecount; i++)
    {
        if (ec_slave[i].state != EC_STATE_OPERATIONAL)
        {
            int nSlot = findSlotBySlaveIndex(i);
            if (nSlot >= 0)
            {
                s_SVerifyReports[nSlot].eError = SLAVE_VERIFY_ERR_OP;
            }
            printf("Slave %d State=0x%04x StatusCode=0x%04x\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode);
        }
    }

    return 0;
}

/* zxr: 启动阶段读取 UINT32 参数；通信成功且长度为 4 字节才保存有效值。 */
static SServoParamU32 readSdoU32(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex)
{
    SServoParamU32 SResult = {0, -1};
    uint32_t unValue = 0;
    int nSize = sizeof(unValue);
    int nWkc = ec_SDOread(usSlave, usIndex, ucSubindex, FALSE, &nSize, &unValue, EC_TIMEOUTRXM);

    if (nWkc > 0 && nSize == sizeof(unValue))
    {
        SResult.unValue = etohl(unValue);
        SResult.nReadStatus = 1;
    }
    return SResult;
}

static int writeSdoValue(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nSize, void *pBuffer, uint32_t unLogValue)
{
    int nResult = ec_SDOwrite(usSlave, usIndex, ucSubindex, FALSE, nSize, pBuffer, EC_TIMEOUTRXM);

    if (nResult == 0)
    {
        printf("SDO write to index 0x%04X failed!!!\r\n", usIndex);
    }
    else if (SOEM_DEBUG)
    {
        printf("SDO write to slave %u, index 0x%04X, value 0x%lX succeeded!!!\r\n", (unsigned int)usSlave, (unsigned int)usIndex, (unsigned long)unLogValue);
    }

    return nResult;
}

static int writeSdoU8(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nValue)
{
    uint8_t ucTemp = (uint8_t)nValue;
    return writeSdoValue(usSlave, usIndex, ucSubindex, sizeof(ucTemp), &ucTemp, ucTemp);
}

static int writeSdoU16(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nValue)
{
    uint16_t usTemp = (uint16_t)nValue;
    return writeSdoValue(usSlave, usIndex, ucSubindex, sizeof(usTemp), &usTemp, usTemp);
}

static int writeSdoU32(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nValue)
{
    uint32_t unTemp = (uint32_t)nValue;
    return writeSdoValue(usSlave, usIndex, ucSubindex, sizeof(unTemp), &unTemp, unTemp);
}

static int checkSdoWriteResult(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nWriteResult)
{
    if (nWriteResult > 0)
    {
        return 1;
    }
    int nSlot = findSlotBySlaveIndex(usSlave);
    if (nSlot >= 0)
    {
        s_SSdoStatus[nSlot].ucSdoFailed = 1;
        s_SSdoStatus[nSlot].usFailedIndex = usIndex;
        s_SSdoStatus[nSlot].ucFailedSubindex = ucSubindex;
    }
    printf("Slave %u SDO setup failed at 0x%04X:%02X\r\n", (unsigned int)usSlave, (unsigned int)usIndex, (unsigned int)ucSubindex);
    return 0;
}

static int verifySdoValue(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nExpectedSize, uint32_t unExpectedValue)
{
    uint8_t ucBuffer[4] = {0};
    int nSize = nExpectedSize;
    int nWkc = ec_SDOread(usSlave, usIndex, ucSubindex, FALSE, &nSize, ucBuffer, EC_TIMEOUTRXM);
    uint32_t unActual = 0;

    if (nWkc > 0 && nSize == nExpectedSize)
    {
        if (nExpectedSize == 1)
        {
            unActual = ucBuffer[0];
        }
        else if (nExpectedSize == 2)
        {
            uint16_t usRaw16 = 0;
            memcpy(&usRaw16, ucBuffer, sizeof(usRaw16));
            unActual = etohs(usRaw16);
        }
        else if (nExpectedSize == 4)
        {
            uint32_t unRaw32 = 0;
            memcpy(&unRaw32, ucBuffer, sizeof(unRaw32));
            unActual = etohl(unRaw32);
        }

        if (unActual == unExpectedValue)
        {
            return 1;
        }
    }

    int nSlot = findSlotBySlaveIndex(usSlave);
    if (nSlot >= 0)
    {
        s_SSdoStatus[nSlot].ucSdoFailed = 1;
        s_SSdoStatus[nSlot].usFailedIndex = usIndex;
        s_SSdoStatus[nSlot].ucFailedSubindex = ucSubindex;
    }
    printf("Slave %u SDO verify failed at 0x%04X:%02X wkc=%d size=%d/%d actual=0x%08lX expected=0x%08lX\r\n",
           (unsigned int)usSlave,
           (unsigned int)usIndex,
           (unsigned int)ucSubindex,
           nWkc,
           nSize,
           nExpectedSize,
           (unsigned long)unActual,
           (unsigned long)unExpectedValue);
    return 0;
}

static int verifySdoU8(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, uint8_t ucExpected) { return verifySdoValue(usSlave, usIndex, ucSubindex, 1, ucExpected); }

static int verifySdoU16(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, uint16_t usExpected) { return verifySdoValue(usSlave, usIndex, ucSubindex, 2, usExpected); }

static int verifySdoU32(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, uint32_t unExpected) { return verifySdoValue(usSlave, usIndex, ucSubindex, 4, unExpected); }

static int readAndVerifyScaleParam(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, uint32_t unExpectedValue, uint32_t *punActualValue, int *pnReadStatus, SSlaveVerifyReport *pSReport)
{
    SServoParamU32 SParam = readSdoU32(usSlave, usIndex, ucSubindex);
    *punActualValue = SParam.unValue;
    *pnReadStatus = SParam.nReadStatus;

    if (unExpectedValue != SLAVE_VERIFY_ANY_U32 && (SParam.nReadStatus != 1 || SParam.unValue != unExpectedValue))
    {
        pSReport->eError = SLAVE_VERIFY_ERR_SCALE_PARAM;
        pSReport->usFailedSdoIndex = usIndex;
        pSReport->ucFailedSdoSubindex = ucSubindex;
        printf("Error: Slave %u scale param mismatch at 0x%04X:%02X status=%d act=%lu exp=%lu\r\n",
               (unsigned int)usSlave,
               (unsigned int)usIndex,
               (unsigned int)ucSubindex,
               SParam.nReadStatus,
               (unsigned long)SParam.unValue,
               (unsigned long)unExpectedValue);
        return 0;
    }

    return 1;
}

static int setupAndVerifyPdoMap(uint16_t usSlave, uint16_t usSmAssignIndex, uint16_t usPdoMapIndex, const SPdoMappingEntry *pSEntries, uint8_t ucEntryCount)
{
    // 1. 写入 SyncManager 分配对象（先清零再分配再置计数）
    if (!checkSdoWriteResult(usSlave, usSmAssignIndex, 0x00, writeSdoU8(usSlave, usSmAssignIndex, 0x00, 0x00)) || !checkSdoWriteResult(usSlave, usSmAssignIndex, 0x01, writeSdoU16(usSlave, usSmAssignIndex, 0x01, usPdoMapIndex))
        || !checkSdoWriteResult(usSlave, usSmAssignIndex, 0x00, writeSdoU8(usSlave, usSmAssignIndex, 0x00, 0x01)))
    {
        return 0;
    }

    // 2. 写入 PDO 映射对象（先清零再逐项写对象字典条目并激活计数）
    if (!checkSdoWriteResult(usSlave, usPdoMapIndex, 0x00, writeSdoU8(usSlave, usPdoMapIndex, 0x00, 0x00)))
    {
        return 0;
    }
    for (uint8_t i = 0; i < ucEntryCount; i++)
    {
        uint8_t ucSub = (uint8_t)(i + 1U);
        if (!checkSdoWriteResult(usSlave, usPdoMapIndex, ucSub, writeSdoU32(usSlave, usPdoMapIndex, ucSub, (int)pSEntries[i].unObject)))
        {
            return 0;
        }
    }
    if (!checkSdoWriteResult(usSlave, usPdoMapIndex, 0x00, writeSdoU8(usSlave, usPdoMapIndex, 0x00, ucEntryCount)))
    {
        return 0;
    }

    // 3. 写后读回校验：逐项核对条目数、分配对象及字段顺序
    if (!verifySdoU8(usSlave, usSmAssignIndex, 0x00, 0x01) || !verifySdoU16(usSlave, usSmAssignIndex, 0x01, usPdoMapIndex) || !verifySdoU8(usSlave, usPdoMapIndex, 0x00, ucEntryCount))
    {
        return 0;
    }
    for (uint8_t i = 0; i < ucEntryCount; i++)
    {
        uint8_t ucSub = (uint8_t)(i + 1U);
        if (!verifySdoU32(usSlave, usPdoMapIndex, ucSub, pSEntries[i].unObject))
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief: 按绑定表执行单台从站的 SDO PDO 映射配置及换算对象回读/核验
 * @param {uint16_t} usSlave 扫描从站序号 (1 开始)
 * @return 1: 成功, 0: 失败
 */
static int configureServoSdo(uint16_t usSlave)
{
    int nSlot = findSlotBySlaveIndex(usSlave);
    if (nSlot < 0)
    {
        printf("Error: configureServoSdo invalid slave %u\r\n", (unsigned int)usSlave);
        return 0;
    }

    const SSlaveBinding *pSBinding = slaveConfig_getBinding((uint8_t)nSlot);
    const SServoProfile *pSProfile = pSBinding->pSProfile;
    SSlaveVerifyReport *pSReport = &s_SVerifyReports[nSlot];

    s_SSdoStatus[nSlot].ucSdoCalled = 1;

    // 1. 按当前从站 Profile 写入 SyncManager2 与 RXPDO 映射并回读校验
    if (!setupAndVerifyPdoMap(usSlave, 0x1C12, pSProfile->usRxPdoIndex, pSProfile->pSRxMap, pSProfile->ucRxMapCount))
    {
        return 0;
    }

    // 2. 按当前从站 Profile 写入 SyncManager3 与 TXPDO 映射并回读校验
    if (!setupAndVerifyPdoMap(usSlave, 0x1C13, pSProfile->usTxPdoIndex, pSProfile->pSTxMap, pSProfile->ucTxMapCount))
    {
        return 0;
    }

    // 3. 在 PRE_OP 阶段回读 0x608F / 0x6091，非零预期项执行强校验
    int nScaleOk = 1;
    nScaleOk &= readAndVerifyScaleParam(usSlave, 0x608F, 0x01, pSProfile->unExpectedEncoderIncrements, &pSReport->unActualEncoderIncrements, &pSReport->nEncoderIncrementsReadStatus, pSReport);
    nScaleOk &= readAndVerifyScaleParam(usSlave, 0x608F, 0x02, pSProfile->unExpectedEncoderRevolutions, &pSReport->unActualEncoderRevolutions, &pSReport->nEncoderRevolutionsReadStatus, pSReport);
    nScaleOk &= readAndVerifyScaleParam(usSlave, 0x6091, 0x01, pSProfile->unExpectedGearNumerator, &pSReport->unActualGearNumerator, &pSReport->nGearNumeratorReadStatus, pSReport);
    nScaleOk &= readAndVerifyScaleParam(usSlave, 0x6091, 0x02, pSProfile->unExpectedGearDenominator, &pSReport->unActualGearDenominator, &pSReport->nGearDenominatorReadStatus, pSReport);
    if (!nScaleOk)
    {
        s_SSdoStatus[nSlot].ucSdoFailed = 1;
        s_SSdoStatus[nSlot].usFailedIndex = pSReport->usFailedSdoIndex;
        s_SSdoStatus[nSlot].ucFailedSubindex = pSReport->ucFailedSdoSubindex;
        return 0;
    }

    return 1;
}
