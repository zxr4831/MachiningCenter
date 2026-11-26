/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.h
 * @brief   This file contains all the function prototypes for
 *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    extern TIM_HandleTypeDef htim2;

    extern TIM_HandleTypeDef htim3;

    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

    void MX_TIM2_Init(void);
    void MX_TIM3_Init(void);

    /* USER CODE BEGIN Prototypes */
    /* zxr: TIM2 启动后的 32 位微秒计数，约 71.6 分钟回绕。
     * 计时使用 (uint32_t)(get_us() - start_us)，测量间隔须小于一次完整回绕。 */
    uint32_t get_us(void);
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
