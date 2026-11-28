/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "axis_interface.h"  /* zxr */
#include "machine_control.h" /* zxr: 独立主站的整组轴服务入口。 */

/* 解除 <stdbool.h> 的 bool 宏定义，避免与数控内核 common.h 的 typedef signed char bool 冲突。 */
#undef bool
#include "Channel.h"
#include "ErrorFunction.h"
#include "RunAPI.h"
#include "common.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 200 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFU; /* zxr: 1MHz 的 32 位自由计数器，约 71.6 分钟回绕。 */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 200 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

    if (tim_baseHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* zxr: TIM2 只提供硬件时间，不启用计数中断。 */
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
    else if (tim_baseHandle->Instance == TIM3)
    {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();

        /* TIM3 interrupt Init */
        HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

    if (tim_baseHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /* TIM2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    }
    else if (tim_baseHandle->Instance == TIM3)
    {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();

        /* TIM3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
extern uint32 timeji;
extern volatile uint8_t g_nc_running;
extern volatile uint8_t g_nc_master_fault_reported;
extern volatile uint32_t g_nc_cycle_counter;

/* zxr: 监控主站收发加数控插补的整拍总耗时（微秒）。 */
volatile uint32_t g_nc_total_cycle_us = 0;
volatile uint32_t g_nc_max_total_cycle_us = 0;

/**
 * @brief  定时器中断回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) /* zxr: TIM2 不再依靠中断累计秒数。 */
    {
        uint32_t begin = tim_getTimeUs();

        // 1. EtherCAT 周期性交换数据与整组安全检查
        mc_runCycle();

        // 2. 整组就绪时推进一拍数控插补并提交下拍总线增量；异常时通知数控侧停机
        if (g_nc_running != 0U)
        {
            if (mc_canStep())
            {
                if (isErrorHasBeenWritten() == TRUE)
                {
                    g_nc_running = 0U;
                }
                else if (g_SChannel[0].ucChannelStatus == SYS_IDLE &&
                         g_SChannel[0].SSpindle.pSSpindleAxis->ucInpStatus == INP_FINISHED)
                {
                    g_nc_running = 0U;
                }
                else
                {
                    int32_t bus_delta[MACHINE_PHYSICAL_AXIS_COUNT] = {0};
                    int i;

                    interruptFuncAPI(0);
                    timeji++;
                    machineTimerInterp(0);
                    g_nc_cycle_counter++;

                    for (i = 0; i < MACHINE_PHYSICAL_AXIS_COUNT && i < AXIS_PHYSICS_NUM_MAX; i++)
                    {
                        bus_delta[i] = (int32_t)g_sBusAxisDeltaPulse[i];
                    }

                    if (!mc_submitStep(bus_delta))
                    {
                        g_nc_running = 0U;
                        if (g_nc_master_fault_reported == 0U)
                        {
                            g_nc_master_fault_reported = 1U;
                            parseRunningError(g_SChannel[0].SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
                        }
                    }
                }
            }
            else if (mc_getState() != MC_STATE_READY)
            {
                g_nc_running = 0U;
                if (g_nc_master_fault_reported == 0U)
                {
                    g_nc_master_fault_reported = 1U;
                    parseRunningError(g_SChannel[0].SPreProcess.unGCodeLineNum, GCODE_ERROR_NUM150, 9999);
                }
            }
        }

        uint32_t elapsed = (uint32_t)(tim_getTimeUs() - begin);
        g_nc_total_cycle_us = elapsed;
        if (elapsed > g_nc_max_total_cycle_us)
        {
            g_nc_max_total_cycle_us = elapsed;
        }
    }
}

/* zxr: 直接读硬件计数；用 uint32_t 差值计算耗时，可跨一次计数回绕。 */
uint32_t tim_getTimeUs(void)
{
    return TIM2->CNT;
}
/* USER CODE END 1 */
