/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_usart.h"
#include "axis_interface.h" /* zxr */
#include "ethernet_test.h"
#include "machine_control.h" /* zxr: 启动轴执行层，统一装载映射和保护参数。 */

#include "delay.h"
#include "sys.h"

#include "ethernet_chip.h"
#include "pcf8574.h"

#include "ethernet.h"
#include "mpu.h"

/* 解除 <stdbool.h> 的 bool 宏定义，避免与数控内核 common.h 的 typedef signed char bool 冲突。 */
#undef bool
#include "Channel.h"
#include "ErrorFunction.h"
#include "Gcode.h"
#include "Motion.h"
#include "RunAPI.h"
#include "SysParaAPI.h"
#include "ToolCompensation.h"
#include "common.h"
#include "lookAhead.h"
#include "toolAxis.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 满足 CTrajPlanning.c / toolAxis.c 中 extern FILE* pWriteFid 的链接符号；板端不写桌面文件。 */
FILE* pWriteFid = NULL;
extern uint32 timeji;

/* zxr: CubeIDE 现场表达式调试控制：g_nc_enable_cmd(1=使能,2=去使能)、g_nc_start_cmd(1=启动)、g_nc_reset_cmd(1=复位)，消费后清零。 */
volatile uint32_t g_nc_enable_cmd = 0;
volatile uint32_t g_nc_start_cmd = 0;
volatile uint32_t g_nc_reset_cmd = 0;
volatile uint8_t g_nc_running = 0;
volatile uint8_t g_nc_master_fault_reported = 0;
volatile uint32_t g_nc_cycle_counter = 0;
static uint8_t s_nc_reset_pending = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 内置纯 XY 慢速测试程序 "0001"：F60 (1 mm/s) 按 0.001 mm/pulse 对应 1 pulse/ms，远低于主站 16 pulse/ms 保护限值。 */
static const char* const s_xy_box_program[] = {
    "G21 G17 G90 G94\n", "G54\n", "G01 X5 Y0 F60\n", "G01 X5 Y5 F60\n", "G01 X0 Y5 F60\n", "G01 X0 Y0 F60\n", "M30\n",
};

int ncEmbeddedGetProgramLine(const char* pName, uint32 unTargetLineNum, char* pOutBuf, uint32 unBufSize)
{
    uint32 line_count = (uint32)(sizeof(s_xy_box_program) / sizeof(s_xy_box_program[0]));
    if (pName == NULL || pOutBuf == NULL || unBufSize == 0U)
    {
        return -1;
    }
    if (strcmp(pName, "0001") != 0 && strcmp(pName, "O0001") != 0)
    {
        return -1;
    }
    if (unTargetLineNum >= line_count)
    {
        return 0;
    }

    strncpy(pOutBuf, s_xy_box_program[unTargetLineNum], (size_t)(unBufSize - 1U));
    pOutBuf[unBufSize - 1U] = '\0';
    return 1;
}

static void ncInitKernel(void)
{
    struct SEncoder* pEncoderArrays[ENCODER_NUM_MAX];
    int i;

    g_nc_running = 0U;
    g_nc_master_fault_reported = 0U;
    g_nc_cycle_counter = 0U;
    timeji = 0U;
    g_ucNameType = 1U;

    initMapArray();
    initEncoderMapArray();
    initSystemPara();
    initAxisPara();

    for (i = 0; i < ENCODER_NUM_MAX; i++)
    {
        pEncoderArrays[i] = (g_SSystemPara.ucEncoderMapArray[0][i] == ENCODER_NULL ||
                             g_SSystemPara.ucEncoderMapArray[0][i] > ENCODER_NUM_MAX - 1)
                                ? NULL
                                : &(g_SEncoder[g_SSystemPara.ucEncoderMapArray[0][i]]);
    }

    initChannel(&g_SChannel[0], 0, g_SSystemPara.ucAxisMapArray[0], g_SSystemPara.ucPWMAxisMapArray[0], pEncoderArrays);
    initToolAxis(&g_SChannel[0], &g_SChannel[0].SSingleAxisArray[TOOL_AXIS]);
    initEditablePara();
    initAfterLoadSysPara();
    initErrorBuf();

    g_RunMode = AUTOMATIC_MODE;
    memset(g_sBusAxisDeltaPulse, 0, sizeof(g_sBusAxisDeltaPulse));
}

static void ncResetKernel(void)
{
    g_nc_running = 0U;
    g_nc_master_fault_reported = 0U;
    g_nc_cycle_counter = 0U;
    timeji = 0U;

    if (g_SChannel != NULL)
    {
        /* 释放中途停机未归还的全局轴/刀塔占用，并把通道、缓冲区和插补状态收回 SYS_IDLE。 */
        channelStopToIDLE(&g_SChannel[0]);
        gcodeInit(0, FALSE, FALSE);
        toolCompensationInit(0);
        lookAheadInit(&g_SChannel[0]);
        interpolationInit(&g_SChannel[0].SPreProcess);
    }

    initErrorBuf();
    g_RunMode = AUTOMATIC_MODE;
    memset(g_sBusAxisDeltaPulse, 0, sizeof(g_sBusAxisDeltaPulse));
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* 配置MPU */
    MPU_Config();
    sys_cache_enable();
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* zxr: 恢复原主站使用的 SRAM 时钟，在以太网 DMA 初始化前开启。 */
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
    __HAL_RCC_BKPRAM_CLKAM_ENABLE();
    __HAL_RCC_D3SRAM1_CLKAM_ENABLE();

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */
    // 延时初始化
    delay_init(400);

    while (pcf8574_init())
    {
        printf("PCF8574 Check Failed!");
    }

    HAL_TIM_Base_Start(&htim2); /* zxr: 仅启动硬件计数，不产生 TIM2 中断。 */

    uartRingBufferInit();

    // ETH外设初始化
    while (ethernet_init())
    {
        printf("ethernet_init error\r\n");
        HAL_Delay(500);
    }
    // 以太网芯片初始化
    while (dm9162_init())
    {
        printf("dm9162_init error\r\n");
        HAL_Delay(500);
    }
    HAL_Delay(2000);

    if (axis_initSystem())
    {
        ncInitKernel();
        const uint8_t ucAxisCount = axis_getActiveCount();
        SMachineAxisLimits limits[AXIS_MAX_NUM] = {0};
        /* 总线增量沿用数控指令当量。联调阶段沿用台架方向和保护上限，整机按实测参数标定。 */
        for (uint8_t axis = 0; axis < ucAxisCount && axis < AXIS_MAX_NUM; axis++)
        {
            const double dVmax = g_SSystemPara.SAxisPara[axis].dVmax;
            const double dPulseToMm = g_SSystemPara.SAxisPara[axis].dPulseToMm;
            if (g_SSystemPara.SAxisPara[axis].ucAxisType == LINEAR_AXIS && isfinite(dVmax) && dVmax > 0.0 &&
                isfinite(dPulseToMm) && dPulseToMm > 0.0)
            {
                /* mm/ms × 周期(ms) ÷ mm/指令单位；同时遵守原台架 16 单位/拍的上限。 */
                double dStep = ceil(dVmax * (AXIS_CYCLE_PERIOD_US / 1000.0) / dPulseToMm);
                limits[axis].nMaxStepLimit = (int32_t)fmin(dStep, 16.0);
                limits[axis].cDirectionInvert = 1;
                limits[axis].nMaxFollowingErrorLimit = 285;
            }
        }
        mc_init(limits, ucAxisCount);
        HAL_TIM_Base_Start_IT(&htim3);
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (g_nc_enable_cmd != 0U)
        {
            uint32_t cmd = g_nc_enable_cmd;
            g_nc_enable_cmd = 0U;
            if (cmd == 1U)
            {
                (void)mc_requestCommand(MC_CMD_ENABLE);
            }
            else if (cmd == 2U)
            {
                (void)mc_requestCommand(MC_CMD_DISABLE);
            }
        }

        if (g_nc_reset_cmd != 0U)
        {
            g_nc_reset_cmd = 0U;
            MACHINE_STATE mc_state = mc_getState();
            if (mc_state == MC_STATE_FAULT && mc_getError() != MC_ERROR_CONFIG)
            {
                if (mc_requestCommand(MC_CMD_RESET))
                {
                    s_nc_reset_pending = 1U;
                }
            }
            else if (mc_state != MC_STATE_RESETTING && g_nc_running == 0U)
            {
                ncResetKernel();
            }
        }

        if (s_nc_reset_pending != 0U)
        {
            MACHINE_STATE mc_state = mc_getState();
            if (mc_state == MC_STATE_NOT_READY)
            {
                ncResetKernel();
                s_nc_reset_pending = 0U;
            }
            else if (mc_state == MC_STATE_FAULT)
            {
                s_nc_reset_pending = 0U;
            }
        }

        if (g_nc_start_cmd != 0U)
        {
            g_nc_start_cmd = 0U;
            if (mc_getState() == MC_STATE_READY && g_nc_running == 0U && isErrorHasBeenWritten() == FALSE)
            {
                memset((void*)g_cName_C0, 0, sizeof(g_cName_C0));
                strncpy((char*)g_cName_C0, "0001", sizeof(g_cName_C0) - 1U);
                g_nc_master_fault_reported = 0U;
                memset(g_sBusAxisDeltaPulse, 0, sizeof(g_sBusAxisDeltaPulse));
                if (runButtonAPI(0, FALSE) == 1)
                {
                    g_nc_running = 1U;
                }
            }
        }

        if (g_nc_running != 0U && isErrorHasBeenWritten() == FALSE)
        {
            GCodeRunLoopAPI();
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 160;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
