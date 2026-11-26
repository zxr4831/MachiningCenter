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
static char IOMap[4096];

/* zxr: 记录各从站 PRE_OP -> SAFE_OP SDO 映射配置状态，供初始化失败时精准定位。 */
typedef struct
{
    uint8_t sdo_called;
    uint8_t sdo_failed;
    uint16_t failed_index;
    uint8_t failed_subindex;
} SlaveSdoStatus_t;

static volatile SlaveSdoStatus_t s_sdo_status[SLAVE_NUM];

/* zxr: 参数只读观测；read_status 为 0 未读取、1 成功、-1 失败，失败时 value 为 0。 */
typedef struct
{
    uint32_t value;
    int read_status;
} ServoParamU32_t;

typedef struct
{
    ServoParamU32_t encoder_increments;  // 0x608F:01 编码器计数
    ServoParamU32_t encoder_revolutions; // 0x608F:02 上述计数对应的电机圈数
    ServoParamU32_t gear_numerator;      // 0x6091:01 电子齿轮分子，对应 PA12
    ServoParamU32_t gear_denominator;    // 0x6091:02 电子齿轮分母，对应 PA13
} ServoScaleParams_t;

/* zxr: Keil 展开查看，[0] 对应从站 1，[1] 对应从站 2。 */
static volatile ServoScaleParams_t s_servo_params[SLAVE_NUM];

// 调试宏 用于打印信息
#define SOEM_DEBUG 0

/* zxr: SDO 配置是本文件内部回调，供主站初始化注册。 */
static int servoSdoSetup(uint16_t slave);

/**
 * @brief: 主站配置
 * @return 1: 成功, 0: 失败
 */
int masterInit(void)
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
int masterSlaveInfoInit(void)
{
    /* zxr: 每次初始化清空观测值，避免读取失败或从站缺失时保留旧数据。 */
    const ServoScaleParams_t empty_params = {0};
    const SlaveSdoStatus_t empty_sdo_status = {0};
    for (uint8_t i = 0; i < SLAVE_NUM; i++)
    {
        s_servo_params[i] = empty_params;
        s_sdo_status[i] = empty_sdo_status;
    }

    // [INIT -> PRE_OP] 扫描并初始化从站：先切入 INIT，再请求进入 PRE_OP，写入 ec_slavecount
    if (ec_config_init(TRUE) <= 0)
    {
        printf("No slaves found!\r\n");
        return 0;
    }

    printf("%d slaves found and configured.\r\n", ec_slavecount);

    // 强制要求接满全部预定从站，避免少接或多接后按固定数组越界/空指针访问
    if (ec_slavecount != SLAVE_NUM)
    {
        printf("Error: Slave count mismatch! expected=%u, found=%d\r\n", (unsigned int)SLAVE_NUM, ec_slavecount);
        return 0;
    }

    // 读取每个从站的状态并逐台核对型号
    ec_readstate();
    for (uint8_t cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        if (ec_slave[cnt].eep_id != SLAVE_EEP_ID)
        {
            printf("Error: Slave %u eep_id mismatch! expected=0x%08lX, actual=0x%08lX (eep_man=0x%08lX)\r\n", (unsigned int)cnt, (unsigned long)SLAVE_EEP_ID, (unsigned long)ec_slave[cnt].eep_id, (unsigned long)ec_slave[cnt].eep_man);
            return 0;
        }

        printf("Slave %u Identity: eep_man=0x%08lX eep_id=0x%08lX\r\n", (unsigned int)cnt, (unsigned long)ec_slave[cnt].eep_man, (unsigned long)ec_slave[cnt].eep_id);

        // 检查伺服ID，注册 PRE_OP -> SAFE_OP 的 SDO 配置回调（由 ec_config_map 触发）
        ec_slave[cnt].PO2SOconfig = &servoSdoSetup;
    }

    // 配置DC模式
    ec_configdc();
    for (uint8_t cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        ec_dcsync0(cnt, TRUE, 1000000, 250000);
    }

    // [PRE_OP -> SAFE_OP] 确认进入 PRE_OP、执行 PO2SOconfig、映射 PDO 到 IOMap，并请求进入 SAFE_OP
    int iomap_size = ec_config_map(&IOMap);
    if (iomap_size <= 0 || iomap_size > (int)sizeof(IOMap))
    {
        printf("Error: ec_config_map failed! size=%d (max=%u)\r\n", iomap_size, (unsigned int)sizeof(IOMap));
        return 0;
    }

    // 确认每台从站的 SDO 映射回调均已执行且全部写入成功
    for (uint8_t cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        if (!s_sdo_status[cnt - 1].sdo_called || s_sdo_status[cnt - 1].sdo_failed)
        {
            printf("Error: Slave %u SDO config failed! called=%u index=0x%04X sub=%02X\r\n",
                   (unsigned int)cnt,
                   (unsigned int)s_sdo_status[cnt - 1].sdo_called,
                   (unsigned int)s_sdo_status[cnt - 1].failed_index,
                   (unsigned int)s_sdo_status[cnt - 1].failed_subindex);
            return 0;
        }
    }

    printf("Slaves mapped, state to SAFE_OP.\n");
    // 等待所有从站确认进入 SAFE_OP
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    ec_readstate();

    for (uint8_t cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        printf("Slave %u State = 0x%04X\r\n", (unsigned int)cnt, (unsigned int)ec_slave[cnt].state);
        if (ec_slave[cnt].state != EC_STATE_SAFE_OP)
        {
            printf("Error: Slave %u failed to reach SAFE_OP! state=0x%04X ALcode=0x%04X\r\n", (unsigned int)cnt, (unsigned int)ec_slave[cnt].state, (unsigned int)ec_slave[cnt].ALstatuscode);
            return 0;
        }
        if (ec_slave[cnt].outputs == NULL || ec_slave[cnt].inputs == NULL || ec_slave[cnt].Obits != sizeof(PdoOutput) * 8U || ec_slave[cnt].Ibits != sizeof(PdoInput) * 8U || ec_slave[cnt].Ostartbit != 0 || ec_slave[cnt].Istartbit != 0
            || ec_slave[cnt].Obytes < sizeof(PdoOutput) || ec_slave[cnt].Ibytes < sizeof(PdoInput))
        {
            printf("Error: Slave %u PDO layout invalid! out=%p (%u bits/%u start/%luB) in=%p (%u bits/%u start/%luB)\r\n",
                   (unsigned int)cnt,
                   ec_slave[cnt].outputs,
                   (unsigned int)ec_slave[cnt].Obits,
                   (unsigned int)ec_slave[cnt].Ostartbit,
                   (unsigned long)ec_slave[cnt].Obytes,
                   ec_slave[cnt].inputs,
                   (unsigned int)ec_slave[cnt].Ibits,
                   (unsigned int)ec_slave[cnt].Istartbit,
                   (unsigned long)ec_slave[cnt].Ibytes);
            return 0;
        }
    }

    // 打印分配的信息
    for (uint16_t cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n "
               "State: %d\n Delay: %d[ns]\n Has DC: %d\n",
               cnt,
               ec_slave[cnt].name,
               ec_slave[cnt].Obits,
               ec_slave[cnt].Ibits,
               ec_slave[cnt].state,
               ec_slave[cnt].pdelay,
               ec_slave[cnt].hasdc);
        printf(" Configured address: %x\n", ec_slave[cnt].configadr);
        printf(" Outputs address: %x\n", ec_slave[cnt].outputs);
        printf(" Inputs address: %x\n", ec_slave[cnt].inputs);

        for (uint16_t j = 0; j < ec_slave[cnt].FMMUunused; j++)
        {
            printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n",
                   j,
                   (int)ec_slave[cnt].FMMU[j].LogStart,
                   ec_slave[cnt].FMMU[j].LogLength,
                   ec_slave[cnt].FMMU[j].LogStartbit,
                   ec_slave[cnt].FMMU[j].LogEndbit,
                   ec_slave[cnt].FMMU[j].PhysStart,
                   ec_slave[cnt].FMMU[j].PhysStartBit,
                   ec_slave[cnt].FMMU[j].FMMUtype,
                   ec_slave[cnt].FMMU[j].FMMUactive);
        }
        printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n", ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
    }

    printf("Request operational state for all slaves\r\n");

    int opWkc = 0;
    int expectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    int allOp = 0;
    uint32_t opStart;

    // [SAFE_OP -> OP] 先发一拍有效 PDO，再通过 ec_writestate(0) 广播请求所有从站进入 OP
    ec_slave[0].state = EC_STATE_OPERATIONAL; // 0 表示广播所有从站
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    opStart = HAL_GetTick();

    // 持续收发 PDO 并轮询状态，等待所有从站确认进入 OP 且 WKC 正常
    do
    {
        ec_send_processdata();
        opWkc = ec_receive_processdata(EC_RX_TIMEOUT_US);

        // 读取当前状态，不长时间等待状态变化
        ec_readstate();

        allOp = 1;
        for (uint16_t i = 1; i <= ec_slavecount; i++)
        {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                allOp = 0;
        }

        if (allOp && opWkc == expectedWkc)
            break;

        osal_usleep(500);
    } while ((uint32_t)(HAL_GetTick() - opStart) < 5000U);

    // 只在失败时打印；成功则尽快返回，启动 TIM3
    if (!allOp || opWkc != expectedWkc)
    {
        printf("OP failed: WKC=%d/%d\r\n", opWkc, expectedWkc);

        for (uint16_t i = 1; i <= ec_slavecount; i++)
        {
            printf("Slave %u ESM=0x%04X ALcode=0x%04X\r\n", (unsigned int)i, (unsigned int)ec_slave[i].state, (unsigned int)ec_slave[i].ALstatuscode);
        }

        return 0;
    }
    // 进入 OP 后将各从站目标位置对齐到实际位置，防止抖动/飞车
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        for (uint16_t i = 1; i <= ec_slavecount; i++)
        {
            PdoOutput *pOut = (PdoOutput *)ec_slave[i].outputs;
            const PdoInput *pIn = (const PdoInput *)ec_slave[i].inputs;
            if (pOut == NULL || pIn == NULL)
            {
                printf("Error: Slave %u PDO pointer NULL in OP\r\n", (unsigned int)i);
                return 0;
            }
            pOut->nTargetPosition = pIn->nActualPosition;
        }

        return 1;
    }

    printf("Not all slaves reached operational state.\n");
    ec_readstate();
    for (uint16_t i = 1; i <= ec_slavecount; i++)
    {
        if (ec_slave[i].state != EC_STATE_OPERATIONAL)
        {
            printf("Slave %d State=0x%04x StatusCode=0x%04x\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode);
        }
    }

    return 0;
}

/* zxr: 启动阶段读取 UINT32 参数；通信成功且长度为 4 字节才保存有效值。 */
static ServoParamU32_t sdoReadU32(uint16_t slave, uint16_t index, uint8_t subindex)
{
    ServoParamU32_t result = {0, -1};
    uint32_t value = 0;
    int size = sizeof(value);
    int wkc = ec_SDOread(slave, index, subindex, FALSE, &size, &value, EC_TIMEOUTRXM);

    if (wkc > 0 && size == sizeof(value))
    {
        result.value = etohl(value);
        result.read_status = 1;
    }
    return result;
}

/**
 * @brief: 写入 8 位 SDO 数据
 * @param {uint16_t} usSlave 从站索引
 * @param {uint16_t} usIndex 目标索引
 * @param {uint8_t} ucSubindex 子索引
 * @param {int} nValue 要写入的值
 * @return 写入结果
 *  @retval 0: 写入失败
 *  @retval >0: 写入成功
 */
static int sdoWriteU8(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nValue)
{
    uint8_t ucTemp = (uint8_t)nValue;
    int nRet = ec_SDOwrite(usSlave, usIndex, ucSubindex, FALSE, sizeof(ucTemp), &ucTemp, EC_TIMEOUTRXM);

    if (nRet == 0)
    {
        printf("SDO write to index 0x%04X failed!!!\r\n", usIndex);
    }
    else if (SOEM_DEBUG)
    {
        printf("SDO write to slave %d, index 0x%04X, value 0x%X succeeded!!!\r\n", usSlave, usIndex, ucTemp);
    }

    return nRet;
}

/**
 * @brief: 写入 16 位 SDO 数据
 * @param {uint16_t} usSlave 从站索引
 * @param {uint16_t} usIndex 目标索引
 * @param {uint8_t} ucSubindex 子索引
 * @param {int} nValue 要写入的值
 * @return 写入结果
 *  @retval 0: 写入失败
 *  @retval >0: 写入成功
 */
static int sdoWriteU16(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nValue)
{
    uint16_t usTemp = (uint16_t)nValue;
    int nRet = ec_SDOwrite(usSlave, usIndex, ucSubindex, FALSE, sizeof(usTemp), &usTemp, EC_TIMEOUTRXM);

    if (nRet == 0)
    {
        printf("SDO write to index 0x%04X failed!!!\r\n", usIndex);
    }
    else if (SOEM_DEBUG)
    {
        printf("SDO write to slave %d, index 0x%04X, value 0x%X succeeded!!!\r\n", usSlave, usIndex, usTemp);
    }

    return nRet;
}

/**
 * @brief: 写入 32 位 SDO 数据
 * @param {uint16_t} usSlave 从站索引
 * @param {uint16_t} usIndex 目标索引
 * @param {uint8_t} ucSubindex 子索引
 * @param {int} nValue 要写入的值
 * @return 写入结果
 *  @retval 0: 写入失败
 *  @retval >0: 写入成功
 */
static int sdoWriteU32(uint16_t usSlave, uint16_t usIndex, uint8_t ucSubindex, int nValue)
{
    uint32_t unTemp = (uint32_t)nValue;
    int nRet = ec_SDOwrite(usSlave, usIndex, ucSubindex, FALSE, sizeof(unTemp), &unTemp, EC_TIMEOUTRXM);

    if (nRet == 0)
    {
        printf("SDO write to index 0x%04X failed!!!\r\n", usIndex);
    }
    else if (SOEM_DEBUG)
    {
        printf("SDO write to slave %d, index 0x%04X, value 0x%X succeeded!!!\r\n", usSlave, usIndex, unTemp);
    }

    return nRet;
}

static int sdoSetupStep(uint16_t slave, uint16_t index, uint8_t subindex, int write_ret)
{
    if (write_ret > 0)
    {
        return 1;
    }
    s_sdo_status[slave - 1].sdo_failed = 1;
    s_sdo_status[slave - 1].failed_index = index;
    s_sdo_status[slave - 1].failed_subindex = subindex;
    printf("Slave %u SDO setup failed at 0x%04X:%02X\r\n", (unsigned int)slave, (unsigned int)index, (unsigned int)subindex);
    return 0;
}

static int sdoVerifyValue(uint16_t slave, uint16_t index, uint8_t subindex, int expected_size, uint32_t expected_val)
{
    uint8_t buf[4] = {0};
    int size = expected_size;
    int wkc = ec_SDOread(slave, index, subindex, FALSE, &size, buf, EC_TIMEOUTRXM);
    uint32_t actual = 0;

    if (wkc > 0 && size == expected_size)
    {
        if (expected_size == 1)
        {
            actual = buf[0];
        }
        else if (expected_size == 2)
        {
            uint16_t raw16 = 0;
            memcpy(&raw16, buf, sizeof(raw16));
            actual = etohs(raw16);
        }
        else if (expected_size == 4)
        {
            uint32_t raw32 = 0;
            memcpy(&raw32, buf, sizeof(raw32));
            actual = etohl(raw32);
        }

        if (actual == expected_val)
        {
            return 1;
        }
    }

    s_sdo_status[slave - 1].sdo_failed = 1;
    s_sdo_status[slave - 1].failed_index = index;
    s_sdo_status[slave - 1].failed_subindex = subindex;
    printf("Slave %u SDO verify failed at 0x%04X:%02X wkc=%d size=%d/%d actual=0x%08lX expected=0x%08lX\r\n",
           (unsigned int)slave,
           (unsigned int)index,
           (unsigned int)subindex,
           wkc,
           size,
           expected_size,
           (unsigned long)actual,
           (unsigned long)expected_val);
    return 0;
}

static int sdoVerifyU8(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t expected) { return sdoVerifyValue(slave, index, subindex, 1, expected); }

static int sdoVerifyU16(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t expected) { return sdoVerifyValue(slave, index, subindex, 2, expected); }

static int sdoVerifyU32(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t expected) { return sdoVerifyValue(slave, index, subindex, 4, expected); }

static int setupAndVerifyPdoMap(uint16_t slave, uint16_t sm_assign_idx, uint16_t pdo_map_idx, const PdoMappingEntry *entries, uint8_t entry_count)
{
    // 1. 写入 SyncManager 分配对象（先清零再分配再置计数）
    if (!sdoSetupStep(slave, sm_assign_idx, 0x00, sdoWriteU8(slave, sm_assign_idx, 0x00, 0x00)) || !sdoSetupStep(slave, sm_assign_idx, 0x01, sdoWriteU16(slave, sm_assign_idx, 0x01, pdo_map_idx))
        || !sdoSetupStep(slave, sm_assign_idx, 0x00, sdoWriteU8(slave, sm_assign_idx, 0x00, 0x01)))
    {
        return 0;
    }

    // 2. 写入 PDO 映射对象（先清零再逐项写对象字典条目并激活计数）
    if (!sdoSetupStep(slave, pdo_map_idx, 0x00, sdoWriteU8(slave, pdo_map_idx, 0x00, 0x00)))
    {
        return 0;
    }
    for (uint8_t i = 0; i < entry_count; i++)
    {
        uint8_t sub = (uint8_t)(i + 1U);
        if (!sdoSetupStep(slave, pdo_map_idx, sub, sdoWriteU32(slave, pdo_map_idx, sub, (int)entries[i].object)))
        {
            return 0;
        }
    }
    if (!sdoSetupStep(slave, pdo_map_idx, 0x00, sdoWriteU8(slave, pdo_map_idx, 0x00, entry_count)))
    {
        return 0;
    }

    // 3. 写后读回校验：逐项核对条目数、分配对象及字段顺序
    if (!sdoVerifyU8(slave, sm_assign_idx, 0x00, 0x01) || !sdoVerifyU16(slave, sm_assign_idx, 0x01, pdo_map_idx) || !sdoVerifyU8(slave, pdo_map_idx, 0x00, entry_count))
    {
        return 0;
    }
    for (uint8_t i = 0; i < entry_count; i++)
    {
        uint8_t sub = (uint8_t)(i + 1U);
        if (!sdoVerifyU32(slave, pdo_map_idx, sub, entries[i].object))
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief: 伺服 SDO 配置
 * @note 参见伺服驱动器的手册，增删PDO只需修改配置表即可！无需修改此函数
 * @param {uint16_t} slave 伺服ID
 * @return 配置结果
 *  @retval 1: 配置成功
 *  @retval 0: 配置失败
 */
static int servoSdoSetup(uint16_t slave)
{
    if (slave < 1 || slave > SLAVE_NUM)
    {
        printf("Error: servoSdoSetup invalid slave %u\r\n", (unsigned int)slave);
        return 0;
    }

    s_sdo_status[slave - 1].sdo_called = 1;

    // 1. SyncManager2 与 RXPDO 映射配置及回读校验
    if (!setupAndVerifyPdoMap(slave, 0x1C12, RXPDO1_INDEX_1600, rxPdoConfig, (uint8_t)RXPDO_MAP_COUNT))
    {
        return 0;
    }

    // 2. SyncManager3 与 TXPDO 映射配置及回读校验
    if (!setupAndVerifyPdoMap(slave, 0x1C13, TXPDO1_INDEX_1A00, txPdoConfig, (uint8_t)TXPDO_MAP_COUNT))
    {
        return 0;
    }

    /* zxr: 在 PRE_OP 配置阶段读取参数，仅供观测，不参与运动换算或改变初始化结果。 */
    if (slave >= 1 && slave <= SLAVE_NUM)
    {
        s_servo_params[slave - 1].encoder_increments = sdoReadU32(slave, 0x608F, 0x01);
        s_servo_params[slave - 1].encoder_revolutions = sdoReadU32(slave, 0x608F, 0x02);
        s_servo_params[slave - 1].gear_numerator = sdoReadU32(slave, 0x6091, 0x01);
        s_servo_params[slave - 1].gear_denominator = sdoReadU32(slave, 0x6091, 0x02);
    }

    return 1;
}
